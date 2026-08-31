# Data Collection

Profiling harness for the virtual-robot function experiments. Collects hardware
counters (`perf stat`), system power (RAPL), and container metrics (`docker
stats`) while the experiment workload is running.

## Requirements

- `perf` installed, with access to `power/energy-*` (RAPL) events
- `docker` CLI + permission to run `docker stats` on the experiment containers
- `sudo` (for `perf stat` on other processes and system-wide power)

## Usage

```bash
sudo ./collect_hw_and_os_metrics.sh <experiment_name> <duration_secs> <container1> [container2] [container3] [container4] [container5]
```

| Argument         | Description                                                    |
| ---------------- | -------------------------------------------------------------- |
| `experiment_name`| Label; results go to `results/<name>_<ts>/`                   |
| `duration_secs`  | Collection duration in seconds                                 |
| `container1..5`  | Containers to profile (1–5; at least 1 required)               |

Listed containers that are not running are skipped with a warning; the script
errors out only if none of them are running. This makes it easy to run scaling
experiments from 1 to 5 robots with the same command.

### Examples

```bash
# 1 robot, 5 minutes
sudo ./collect_hw_and_os_metrics.sh controller_1robot 300 panther_controller

# 2 robots, 10 minutes
sudo ./collect_hw_and_os_metrics.sh controller_2robots 600 panther_controller panther2_controller

# 3 robots
sudo ./collect_hw_and_os_metrics.sh controller_3robots 300 panther_controller panther2_controller panther3_controller

# 5 robots
sudo ./collect_hw_and_os_metrics.sh controller_5robots 600 panther_controller panther2_controller panther3_controller panther4_controller panther5_controller

# CPU-isolation stress test (robot1 + robot2, pinned to P-cores)
sudo ./collect_hw_and_os_metrics.sh two_robots 300 robot1 robot2
```

Start the experiment containers *before* running the script — it resolves the
worker PIDs from the running containers.

## Worker PID discovery

For each container the script profiles the actual workload processes, skipping
shells, the `ros2 launch` wrapper, and orchestrator/parent processes:

1. **stress-ng workers** — processes whose CMD contains `[run]`
2. **compiled ROS2 nodes** — processes running binaries under
   `/opt/ros/<distro>/lib/` (e.g. `controller_server`, `velocity_smoother`,
   `lifecycle_manager`)
3. **leaf processes** — non-shell/sleep processes with no children (also
   excludes the container init)
4. **container init PID** — last resort

## Output

Each run writes to `results/<experiment_name>_<YYYYMMDD_HHMMSS>/`:

| File                        | Contents                                                     |
| --------------------------- | ------------------------------------------------------------ |
| `perf_<container>.txt`      | Per-container perf counters, 1 Hz: cycles, instructions, cache misses/references, LLC miss/reference, context switches, migrations, page faults (one file per active container) |
| `power.txt`                 | System-wide RAPL energy: pkg, cores, gpu                      |
| `docker_stats.csv`          | `timestamp,name,cpu_perc,mem_usage,net_io,block_io,pids` (~2 s cadence, all active containers) |
| `experiment_info.txt`       | Experiment metadata (active containers, PIDs, CPU, kernel, docker) |
| `patrol_log.txt`            | Container logs (`experiment_runner`, else first active container) |
| `*.err`                     | perf stderr (only present if a collector had errors)          |

## Notes

- The docker stats loop takes ~2 s per sample (`docker stats --no-stream`
  sampling window), so a `DURATION` run yields roughly `DURATION/2` samples.
- perf events are pinned to the `cpu_core/` PMU. Containers are cpuset-pinned
  to P-cores, so P/E-core hybrid splits (`cpu_atom` rows) are excluded.
- Derived metrics to compute from the raw counters:
  - `IPC = instructions / cycles`
  - `MPKI = cache-misses / (instructions / 1000)`
  - `LLC hit rate = 1 - (LLC.miss / LLC.reference)`
  - `Power/goal = total_energy / goals_completed`
- Ctrl+C stops all collectors early; a post-run validation step reports which
  collectors failed or produced empty output.

  # TO DO:

  # Experiment 1 — Pinned: 1 Dedicated CPU per Controller

Each controller gets **exactly 1 CPU core**, no sharing.

| Variant | Controllers | Pinning |
|---|---|---|
| 1a | 1 controller | CPU 0 |
| 1b | 2 controllers | Controller 1 → CPU 0, Controller 2 → CPU 1 |
| 1c | 3 controllers | Controller 1 → CPU 0, Controller 2 → CPU 1, Controller 3 → CPU 2 |
| 1d | 4 controllers | Controller 1 → CPU 0, Controller 2 → CPU 1, Controller 3 → CPU 2, Controller 4 → CPU 3 |


  ## Experiment 2 — Shared: All Controllers Compete for 2 CPUs

All controllers share the **same 2 cores** (CPU 0 and CPU 1).

| Variant | Controllers | Pinning |
|---|---|---|
| 2a | 1 controller | Shared on CPU 0–1 |
| 2b | 2 controllers | Shared on CPU 0–1 |
| 2c | 3 controllers | Shared on CPU 0–1 |
| 2d | 4 controllers | Shared on CPU 0–1 |

Docker compose example:
```yaml
controller_panther:
  cpuset: "0-1"
controller_panther2:
  cpuset: "0-1"
controller_panther3:
  cpuset: "0-1"
controller_panther4:
  cpuset: "0-1"
```

- [ ] Run each variant (2a–2d), same navigation task, **10× per variant**
- [ ] Collect all metrics per run