# CPU & Memory Isolation for Virtual Robot Function Experiments

## Overview

This setup isolates CPU cores so that virtualized robotic functions
(localization, planner, BT orchestrator, controller, sensor pre-processing)
run on **dedicated P-cores** with no OS interference — replicating the
noisy-neighbor study from the AIRIC paper.

```
┌──────────────────────────────────────────────────┐
│              Experiment P-Cores (isolated)        │
│                                                  │
│  P-Core 0      P-Core 1      P-Core 4           │
│  CPUs 0,1      CPUs 2,3      CPUs 8,9           │
│  4700 MHz      4700 MHz      4700 MHz            │
│  L2 #0         L2 #1         L2 #4              │
│                                                  │
├──────────────────────────────────────────────────┤
│              Unused P-Cores (reserve)            │
│                                                  │
│  P-Core 2      P-Core 3      P-Core 5           │
│  CPUs 4,5      CPUs 6,7      CPUs 10,11         │
│  4800 MHz      4800 MHz      4700 MHz            │
│                                                  │
├──────────────────────────────────────────────────┤
│              E-Cores (OS & background)           │
│                                                  │
│  CPUs 12-19    3600 MHz      L2 #6, #7          │
│  systemd, IRQs, Docker daemon, kworkers          │
│                                                  │
└──────────────────────────────────────────────────┘
```

---

## CPU Topology Reference

```bash
$ sudo lscpu --all --extended
CPU  CORE  L2   MAXMHZ   ROLE
 0     0   #0   4700     Experiment pool
 1     0   #0   4700     Experiment pool
 2     1   #1   4700     Experiment pool
 3     1   #1   4700     Experiment pool
 4     2   #2   4800     Unused (different freq)
 5     2   #2   4800     Unused (different freq)
 6     3   #3   4800     Unused (different freq)
 7     3   #3   4800     Unused (different freq)
 8     4   #4   4700     Experiment pool
 9     4   #4   4700     Experiment pool
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

### 1.1 Disable Turbo boost (critical for reproducibility)

```bash
# Check current state
cat /sys/devices/system/cpu/intel_pstate/no_turbo

# Disable turbo boost
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo

# Make persistent across reboots
echo 'w /sys/devices/system/cpu/intel_pstate/no_turbo - - - - 1' | \
  sudo tee /etc/tmpfiles.d/no-turbo.conf
```

### 1.2 Set CPU governor to performance (experiment P-cores only)
```bash
# Install cpufrequtils if needed
sudo apt install cpufrequtils

# Set the 3 experiment P-cores to performance governor
for cpu in 0 1 2 3 8 9; do
  echo performance | sudo tee /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
done

# Verify
for cpu in 0 1 2 3 8 9; do
  echo -n "CPU $cpu: "; cat /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
done
```
### 1.3 Move OS to E-cores

```bash
sudo nano /etc/systemd/system.conf
```

Set:
```ini
CPUAffinity=12 13 14 15 16 17 18 19
```

### 1.4 Kernel-level core isolation

```bash
sudo nano /etc/default/grub
```

Set:
```ini
GRUB_CMDLINE_LINUX="isolcpus=0-3,8-9"
```

Apply and reboot:
```bash
sudo update-grub
sudo reboot
```

### 1.5 Move unbound workqueues to E-cores

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

### 1.6 Move IRQs to E-cores

```bash
for irq in $(ls /proc/irq/ | grep -E '^[0-9]+$'); do
  echo "000ff000" | sudo tee /proc/irq/$irq/smp_affinity 2>/dev/null || true
done
```

---

## Step 2: Verify Isolation

Run all of these after reboot:

```bash
# 1. Confirm isolated CPUs
cat /sys/devices/system/cpu/isolated
# Expected: 0-3,8-9

# 2. Confirm kernel cmdline
cat /proc/cmdline
# Should contain: isolcpus=0-3,8-9

# 3. Confirm systemd is on E-cores
taskset -cp 1
# Expected: pid 1's current affinity list: 12-19

# 4. Confirm workqueue cpumask
cat /sys/devices/virtual/workqueue/*/cpumask | sort -u
# Expected: ff000

# 5. Confirm no userspace processes on experiment cores
ps -eo pid,psr,comm | awk '($2 >= 0 && $2 <= 3) || $2 == 8 || $2 == 9 {print}' \
  | grep -v kworker | grep -v migration | grep -v ksoftirqd \
  | grep -v cpuhp | grep -v idle_inject | grep -v kthreadd \
  | grep -v rcu | grep -v kprobe | grep -v pool_workqueue
# Expected: empty (no output)

# 6. Confirm remaining kernel threads use ~0 CPU time
ps -eo pid,psr,comm,time | awk '($2 >= 0 && $2 <= 3) || $2 == 8 || $2 == 9 {print}'
# Expected: all showing 00:00:00
```

> **Note:** Per-CPU kernel threads (`migration/N`, `ksoftirqd/N`, `cpuhp/N`,
> `kworker/N:*`) will always appear on isolated cores. This is normal —
> they are architecturally required and consume near-zero CPU time.

---

## Step 3: Run Test Workload

### 3.1 Build the test image

```bash
cd test/
docker build -f Dockerfile.robot-workload -t robot-workload .
```

### 3.2 Run on ALL experiment cores (shared pool — paper default)

```bash
docker run --rm --name robot1 --cpuset-cpus="0-3,8-9" robot-workload
```

### 3.3 Run PINNED to a specific P-core

```bash
# Pin to P-Core 0 (CPUs 0,1)
docker run --rm --name robot1 --cpuset-cpus="0,1" robot-workload

# Pin to P-Core 1 (CPUs 2,3)
docker run --rm --name robot2 --cpuset-cpus="2,3" robot-workload

# Pin to P-Core 4 (CPUs 8,9)
docker run --rm --name robot3 --cpuset-cpus="8,9" robot-workload
```

### 3.4 Run multiple containers (noisy neighbor experiment)

```bash
# 2 containers sharing all experiment cores
docker run -d --rm --name robot1 --cpuset-cpus="0-3,8-9" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="0-3,8-9" robot-workload

# 3 containers, each pinned to its own P-core (isolated)
docker run -d --rm --name robot1 --cpuset-cpus="0,1" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="2,3" robot-workload
docker run -d --rm --name robot3 --cpuset-cpus="8,9" robot-workload

# 3 containers sharing all experiment cores (noisy)
docker run -d --rm --name robot1 --cpuset-cpus="0-3,8-9" robot-workload
docker run -d --rm --name robot2 --cpuset-cpus="0-3,8-9" robot-workload
docker run -d --rm --name robot3 --cpuset-cpus="0-3,8-9" robot-workload
```

### 3.5 Stop all containers

```bash
docker stop robot1 robot2 robot3 2>/dev/null
```

---

## Step 4: Measure (perf)

### 4.1 Per-container CPU usage

```bash
# While containers are running:
docker stats --no-stream
```

### 4.2 Cache misses and IPC (key AIRIC metrics)

```bash
# Get the PID of a container's main process
CONTAINER_PID=$(docker inspect --format '{{.State.Pid}}' robot1)

# Measure IPC and cache misses for 10 seconds
sudo perf stat -e cycles,instructions,cache-misses,cache-references \
  -p $CONTAINER_PID --timeout 10000
```

Key metrics to record:
- **IPC** (instructions per cycle) — drops below 1.0 = memory-bounded (cache contention)
- **MPKI** (cache misses per 1000 instructions) — increases with more containers

### 4.3 Context switches

```bash
sudo perf stat -e context-switches,cpu-migrations \
  -p $CONTAINER_PID --timeout 10000
```

### 4.4 Full experiment measurement script

```bash
#!/bin/bash
# Usage: ./measure.sh <container_name> <duration_seconds>
CONTAINER=$1
DURATION=${2:-10}

PID=$(docker inspect --format '{{.State.Pid}}' "$CONTAINER")
if [ -z "$PID" ]; then
  echo "Container $CONTAINER not found"
  exit 1
fi

echo "=== Measuring $CONTAINER (PID $PID) for ${DURATION}s ==="
sudo perf stat \
  -e cycles,instructions,cache-misses,cache-references,context-switches,cpu-migrations \
  -p "$PID" --timeout $((DURATION * 1000)) 2>&1
```

---

## Experiment Matrix

These are the experiments to replicate the AIRIC study:

| # | Experiment | cpuset-cpus | What it measures |
|---|---|---|---|
| 1 | 1 container, shared pool | `0-3,8-9` | Baseline CPU & cache usage |
| 2 | 2 containers, shared pool | `0-3,8-9` | Noisy neighbor overhead (2) |
| 3 | 3 containers, shared pool | `0-3,8-9` | Noisy neighbor overhead (3) |
| 4 | 2 containers, pinned (same L2) | `0,1` + `0,1` | L1/L2 cache contention |
| 5 | 2 containers, pinned (diff L2) | `0,1` + `2,3` | L2 isolated, L3 shared |
| 6 | 3 containers, each pinned | `0,1` + `2,3` + `8,9` | Full L2 isolation |

For each experiment, measure:
1. CPU usage (`docker stats`)
2. IPC and MPKI (`perf stat`)
3. Context switches (`perf stat`)
4. Your application-level metric (latency, throughput, etc.)

---

## Folder Structure

```
CPU-isolation/
├── README.md                          ← this file
└── test/
    ├── Dockerfile.robot-workload      ← test container image
    └── robot_workload.sh              ← workload script
```

---

## Mapping to AIRIC Paper

| Paper (i7-7700K) | This setup |
|---|---|
| Cores 0,4 → OS | E-cores (CPUs 12-19) → OS |
| Core 1,5 → vBS pool (Phys #1) | CPUs 0,1 → experiment (P-Core 0) |
| Core 2,6 → vBS pool (Phys #2) | CPUs 2,3 → experiment (P-Core 1) |
| Core 3,7 → vBS pool (Phys #3) | CPUs 8,9 → experiment (P-Core 4) |
| `CONFIG_RT_GROUP_SCHED` | Not needed (robot functions use normal scheduling) |
| `systemd CPUAffinity` | ✅ `CPUAffinity=12-19` |
| `isolcpus` | ✅ `isolcpus=0-3,8-9` |
| Docker containers | ✅ Docker with `--cpuset-cpus` |

---

## Undo / Revert

To remove isolation and restore normal CPU scheduling:

```bash
# 1. Remove isolcpus from GRUB
sudo nano /etc/default/grub
# Remove isolcpus=0-3,8-9 from GRUB_CMDLINE_LINUX

# 2. Remove CPUAffinity from systemd
sudo nano /etc/systemd/system.conf
# Comment out or remove CPUAffinity line

# 3. Disable workqueue service
sudo systemctl disable isolate-workqueues.service

# 4. Apply and reboot
sudo update-grub
sudo reboot
```

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `exec format error` when running container | Fix Windows line endings: `sed -i 's/\r$//' script.sh` |
| `isolcpus` not in `/proc/cmdline` after reboot | Check you edited `GRUB_CMDLINE_LINUX` (not `_DEFAULT`) |
| kworkers still on experiment cores | Per-CPU kernel threads are normal and harmless (near-zero CPU time) |
| Container can't use RT scheduling | Need `CONFIG_RT_GROUP_SCHED=y` in kernel (recompile required) |
| VS Code saves with CRLF | Set `Files: Eol → \n` in settings, add `.editorconfig` with `end_of_line = lf` |
