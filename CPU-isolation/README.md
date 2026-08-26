# Overview

This setup isolates CPU cores so that virtualized robotic functions
(localization, planner, BT orchestrator, controller, sensor pre-processing)
run on **dedicated P-cores** with no OS interference — replicating the
noisy-neighbor study from the AIRIC paper.

The AIRIC paper used `systemd CPUAffinity` to move the OS onto dedicated
cores and let the **Linux CFS scheduler freely distribute** container processes
across the experiment pool. We do **not** use `isolcpus` — it disables the
scheduler's load balancing on isolated cores, which prevents processes from
spreading across the pool.

```
┌──────────────────────────────────────────────────┐
│         Experiment Pool: 1 P-Core (default)      │
│                                                  │
│  P-Core 0                                        │
│  CPUs 0,1      4700 MHz      L2 #0              │
│  Shared by all containers (CFS load balancing)   │
│                                                  │
├──────────────────────────────────────────────────┤
│         Available for expanded experiments       │
│                                                  │
│  P-Core 1      P-Core 4                         │
│  CPUs 2,3      CPUs 8,9                         │
│  4700 MHz      4700 MHz                          │
│  L2 #1         L2 #4                            │
│                                                  │
├──────────────────────────────────────────────────┤
│         Unused P-Cores (reserve)                 │
│                                                  │
│  P-Core 2      P-Core 3      P-Core 5           │
│  CPUs 4,5      CPUs 6,7      CPUs 10,11         │
│  4800 MHz      4800 MHz      4700 MHz            │
│                                                  │
├──────────────────────────────────────────────────┤
│         E-Cores (OS & background)                │
│                                                  │
│  CPUs 12-19    3600 MHz      L2 #6, #7          │
│  systemd, IRQs, Docker daemon, kworkers          │
│                                                  │
└──────────────────────────────────────────────────┘
```

## Why 1 P-core (2 threads) as the default pool?

The paper used 3 physical cores (6 threads) for vBSs consuming ~80% of a
core each. Our robotic containers use ~30% each. To match the paper's
pressure levels:

| | Paper (i7-7700K) | This setup |
|---|---|---|
| Pool | 3 P-cores (6 threads) | **1 P-core (2 threads)** |
| Per-container CPU | ~80% of a core | ~30% of a core |
| 4 containers avg/thread | ~53% | **~75%** |
| 5 containers avg/thread | ~67% | **~90%+** |
| Errors begin at | 4–5 vBSs | 4–5 containers |

## Why NOT `isolcpus`?

We initially used `isolcpus=0-3,8-9` but discovered it **breaks CFS load
balancing** — all container processes pile onto a single CPU instead of
spreading across the pool. The paper did NOT use `isolcpus`:

> *"Six computing cores are reserved for the shared pool by means of
> **systemd's CPUAffinity**"* — Paper 1

Without `isolcpus`, the CFS scheduler **actively migrates** processes across
cores in the cpuset, which is exactly how the paper's shared pool worked.

---

## CPU Topology Reference

```bash
$ sudo lscpu --all --extended
CPU  CORE  L2   MAXMHZ   ROLE
 0     0   #0   4700     Experiment pool (default)
 1     0   #0   4700     Experiment pool (default)
 2     1   #1   4700     Experiment pool (expanded)
 3     1   #1   4700     Experiment pool (expanded)
 4     2   #2   4800     Unused (different freq)
 5     2   #2   4800     Unused (different freq)
 6     3   #3   4800     Unused (different freq)
 7     3   #3   4800     Unused (different freq)
 8     4   #4   4700     Experiment pool (expanded)
 9     4   #4   4700     Experiment pool (expanded)
10     5   #5   4700     Unused (reserve)
11     5   #5   4700     Unused (reserve)
12     6   #6   3600     OS
13     7   #6   3600     OS
14     8   #6   3600     OS
15     9   #6   3600     OS
16    10   #7   3600     OS
17    11   #7   3600     OS
18    12   #7   3600     OS
19    13   #7   3600     OS
```

---

## Step 1: Isolation Setup (one-time)

## 1.1 Disable Turbo Boost (critical for reproducibility)

```bash
# Check current state
cat /sys/devices/system/cpu/intel_pstate/no_turbo

# Disable turbo boost
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo

# Make persistent across reboots
echo 'w /sys/devices/system/cpu/intel_pstate/no_turbo - - - - 1' | \
  sudo tee /etc/tmpfiles.d/no-turbo.conf
```

## 1.2 Set CPU governor to performance (experiment P-cores only)

```bash
# Install cpufrequtils if needed
sudo apt install cpufrequtils

# Set the experiment P-cores to performance governor
for cpu in 0 1 2 3 8 9; do
  echo performance | sudo tee /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
done

# Verify
for cpu in 0 1 2 3 8 9; do
  echo -n "CPU $cpu: "; cat /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
done
```

## 1.3 Move OS to E-cores (systemd CPUAffinity)

```bash
sudo nano /etc/systemd/system.conf
```

Add or modify:
```ini
CPUAffinity=12 13 14 15 16 17 18 19
```

## 1.4 Move unbound workqueues to E-cores

```bash
# Run once after boot
echo 000ff000 | sudo tee /sys/devices/virtual/workqueue/*/cpumask
```

To make persistent, enable the systemd service:
```bash
sudo tee /etc/systemd/system/isolate-workqueues.service << 'EOF'
[Unit]
Description=Move unbound workqueues to E-cores
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo 000ff000 > /sys/devices/virtual/workqueue/*/cpumask'

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable isolate-workqueues.service
```

## 1.5 Move IRQs to E-cores

```bash
for irq in $(ls /proc/irq/ | grep -E '^[0-9]+$'); do
  echo "000ff000" | sudo tee /proc/irq/$irq/smp_affinity 2>/dev/null || true
done
```

> **Note:** IRQ affinity does not persist across reboots. Add the above
> command to the `isolate-workqueues.service` or a separate systemd service,
> or run it manually after each reboot.

## 1.6 Reboot and verify GRUB

Ensure `isolcpus` is **NOT** set in GRUB (it breaks CFS load balancing):

```bash
sudo nano /etc/default/grub
```

Confirm:
```ini
GRUB_CMDLINE_LINUX=""
```

If it contains `isolcpus=...`, remove it:
```bash
sudo update-grub
sudo reboot
```

---

## Step 2: Post-Reboot Checklist

Run **all** of these after every reboot:

```bash
# 1. Confirm isolcpus is NOT active (should be empty)
cat /sys/devices/system/cpu/isolated
# Expected: (empty line)

# 2. Confirm systemd is on E-cores
taskset -cp 1
# Expected: pid 1's current affinity list: 12-19

# 3. Move workqueues to E-cores (if service didn't run yet)
echo 000ff000 | sudo tee /sys/devices/virtual/workqueue/*/cpumask

# 4. Confirm workqueue cpumask
cat /sys/devices/virtual/workqueue/*/cpumask | sort -u
# Expected: 000ff000

# 5. Move IRQs to E-cores
for irq in $(ls /proc/irq/ | grep -E '^[0-9]+$'); do
  echo "000ff000" | sudo tee /proc/irq/$irq/smp_affinity 2>/dev/null || true
done

# 6. Disable turbo boost
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo

# 7. Set performance governor on experiment cores
for cpu in 0 1 2 3 8 9; do
  echo performance | sudo tee /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
done

# 8. Verify no major userspace processes on experiment cores
ps -eo pid,psr,comm | awk '$2 >= 0 && $2 <= 3 {print}' | head -20
# Expected: only kernel threads (kworker, migration, ksoftirqd, etc.)
# Some stray OS processes may appear — this is normal without isolcpus.
# CPUAffinity keeps the vast majority on E-cores.
```

> **Tip:** Create a script `post_reboot_setup.sh` with steps 3–7 so you
> only need to run one command after each reboot.

---

## Step 3: Run Experiments

## Default experiment pool: 1 P-core (CPUs 0,1)

All containers share the **same cpuset** and the Linux CFS scheduler
distributes processes freely — exactly as the paper did.

## 3.1 Build the test image

```bash
cd test/
docker build -f Dockerfile.robot-workload -t robot-workload .
```

## 3.2 Scaling experiment (paper's main study)

Scale from 1 to 5 containers on the **same pool** (`--cpuset-cpus="0,1"`):

```bash
# Experiment 1: 1 container — baseline
docker run -d --rm --name robot1 --cpuset-cpus="0,1" robot-workload

# Experiment 2: 2 containers
docker run -d --rm --name robot1 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="0,1" robot-workload

# Experiment 3: 3 containers
docker run -d --rm --name robot1 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot3 --cpuset-cpus="0,1" robot-workload

# Experiment 4: 4 containers
docker run -d --rm --name robot1 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot3 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot4 --cpuset-cpus="0,1" robot-workload

# Experiment 5: 5 containers
docker run -d --rm --name robot1 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot3 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot4 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot5 --cpuset-cpus="0,1" robot-workload
```

## 3.3 Stop all containers

```bash
docker stop robot1 robot2 robot3 robot4 robot5 2>/dev/null
```

## 3.4 Expected utilization at each scale point

| Containers | Per-container CPU | Total CPU | Avg/thread | Expected behavior |
|---|---|---|---|---|
| 1 | ~30% | ~30% | ~15% | Baseline, no contention |
| 2 | ~30% | ~60% | ~30% | Comfortable, minimal sharing |
| 3 | ~30% | ~90% | ~45% | Contention begins |
| 4 | ~30% | ~120% | ~60–75% | **Noisy neighbor visible** |
| 5 | ~30% | ~150% | ~75–90%+ | **Control loop misses expected** |

## 3.5 Monitor CPU utilization during experiments

Use the `cpu_monitor.sh` script to validate utilization in real-time:

```bash
# In a separate terminal:
./cpu_monitor.sh 2 "0,1"
```

---

## Step 4: Measure (perf)

## 4.1 Per-container perf (one perf per container — required)

The paper measured IPC and MPKI **per individual vBS**. This is critical to
see which function degrades most as you scale.

```bash
# Get the PIDs of a container's worker processes
CONTAINER_PIDS=$(docker top robot1 -eo pid,comm | grep -v PID | awk '{print $1}' | tr '\n' ',' | sed 's/,$//')

# Measure IPC, cache misses, context switches for 60 seconds
sudo perf stat -p "$CONTAINER_PIDS" \
  -e cpu_core/cycles/,cpu_core/instructions/,cpu_core/cache-misses/,cpu_core/cache-references/ \
  -e context-switches,cpu-migrations \
  --timeout 60000
```

## 4.2 System-wide power (RAPL)

```bash
sudo perf stat -a \
  -e power/energy-pkg/,power/energy-cores/ \
  -I 1000 \
  --timeout 60000
```

## 4.3 Docker stats (CPU%, memory)

```bash
docker stats --no-stream
```

## 4.4 Key metrics to record

| Metric | Formula | What it tells you |
|---|---|---|
| **IPC** | instructions / cycles | Drops below 1.0 = memory-bounded (cache contention) |
| **MPKI** | cache-misses / (instructions / 1000) | Increases with more containers = cache eviction |
| **Context switches** | Direct from perf | Increases with CPU time contention |
| **CPU migrations** | Direct from perf | Process moved to different core (cache cold start) |
| **Control loop miss rate** | From ROS 2 logs | Your equivalent of the paper's `explode` column |

## 4.5 What "good" vs "degraded" looks like (from the paper)

| Metric | 1 container | 5 containers | Meaning |
|---|---|---|---|
| IPC | ~1.1 | ~0.6 | 45% less useful work per cycle |
| MPKI | ~5 | ~25 | 500% more cache misses |
| CPU usage | Linear | Non-linear (+40% overhead) | Extra cycles wasted on memory stalls |

---

## Experiment Matrix

## Primary: scaling on 1 P-core (replicates AIRIC Fig. 1, 10, 11)

| # | Containers | cpuset-cpus | What it measures |
|---|---|---|---|
| 1 | 1 | `0,1` | Baseline IPC, MPKI, CPU |
| 2 | 2 | `0,1` | Noisy neighbor onset |
| 3 | 3 | `0,1` | Moderate contention |
| 4 | 4 | `0,1` | Heavy contention (~75% avg/thread) |
| 5 | 5 | `0,1` | Near saturation, control loop misses |

For each experiment, record:
1. `perf stat` per container (IPC, MPKI, context switches, migrations)
2. `docker stats` (CPU%, memory)
3. Control loop miss rate (from ROS 2 container logs)
4. System power (RAPL)

## Secondary: pinned experiments (replicates AIRIC Fig. 8, 9)

| # | Containers | cpuset-cpus | What it measures |
|---|---|---|---|
| 6 | 3 pinned, each on own P-core | `0,1` + `2,3` + `8,9` | Full L2 isolation, L3 shared only |
| 7 | 3 shared, 2 P-core pool | `0-3` | Medium pool, CFS distributes |
| 8 | 3 shared, 3 P-core pool | `0-3,8-9` | Full pool, one per core (no sharing) |

Compare experiment 3 (3 containers, 1 P-core) vs 6, 7, 8 to see how
pool size affects the noisy neighbor overhead.

---

## Understanding "Control Loop Missed" = Paper's "explode"

The paper explicitly tracked failures:

> *"when column **explode** takes the value True, it indicates that the
> **traffic demand has not been served correctly**, which is correlated
> to the lack of computational resources"*

Your ROS 2 "Control loop missed its desired rate" is the **exact equivalent**.
This is **valuable data, not a bug** — it marks the point where CPU contention
degrades the service. Record the miss rate at each scale point.

---

## Folder Structure

```
CPU-isolation/
├── README.md                          ← this file
├── cpu_monitor.sh                     ← real-time CPU utilization monitor
├── post_reboot_setup.sh               ← run after every reboot
└── test/
    ├── Dockerfile.robot-workload      ← test container image
    └── robot_workload.sh              ← workload script
```

---

## Undo / Revert

To remove isolation and restore normal CPU scheduling:

```bash
# 1. Remove CPUAffinity from systemd
sudo nano /etc/systemd/system.conf
# Comment out or remove CPUAffinity line

# 2. Disable workqueue service
sudo systemctl disable isolate-workqueues.service

# 3. Remove turbo boost persistence
sudo rm /etc/tmpfiles.d/no-turbo.conf

# 4. Reboot
sudo reboot
```

---

## Troubleshooting

| Issue | Cause | Fix |
|---|---|---|
| `exec format error` on container | Windows line endings (CRLF) | `sed -i 's/\r$//' script.sh` or set VS Code EOL to `\n` |
| All processes on CPU 0, no spreading | `isolcpus` is set | Remove `isolcpus` from GRUB, `sudo update-grub`, reboot |
| kworkers on experiment cores | Per-CPU kernel threads (normal) | Harmless — near-zero CPU time |
| Stray OS processes on experiment cores | Expected without `isolcpus` | CPUAffinity keeps 99% on E-cores; rare strays won't affect results |
| Container can't use RT scheduling | Need kernel option | Recompile with `CONFIG_RT_GROUP_SCHED=y` (not needed for this study) |
| VS Code saves with CRLF | Default Windows line endings | Set `Files: Eol → \n` in settings, add `.editorconfig` with `end_of_line = lf` |
| CPU frequency fluctuating | Turbo boost or powersave governor | Disable turbo + set `performance` governor (Steps 1.1, 1.2) |
