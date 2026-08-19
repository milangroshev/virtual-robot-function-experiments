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
sudo ./collect_hw_and_os_metrics.sh <experiment_name> <duration_secs> [container1] [container2]
```

| Argument         | Default              | Description                                  |
| ---------------- | -------------------- | -------------------------------------------- |
| `experiment_name`| `baseline`           | Label; results go to `results/<name>_<ts>/` |
| `duration_secs`  | `300`                | Collection duration in seconds               |
| `container1`     | `panther_controller` | First container to profile                   |
| `container2`     | `panther2_controller`| Second container (omit for single-container) |

### Examples

```bash
# Single container, 5 minutes
sudo ./collect_hw_and_os_metrics.sh controller_baseline 300 panther_controller

# Two containers, 10 minutes
sudo ./collect_hw_and_os_metrics.sh controller_2robots 600 panther_controller panther2_controller

# CPU-isolation stress test (robot1 + robot2, pinned to P-cores)
sudo ./collect_hw_and_os_metrics.sh two_robots 300 robot1 robot2
```

Start the experiment containers *before* running the script — it resolves the
worker PIDs from the running containers and fails if `container1` has none.

## Output

Each run writes to `results/<experiment_name>_<YYYYMMDD_HHMMSS>/`:

| File                        | Contents                                                     |
| --------------------------- | ------------------------------------------------------------ |
| `perf_<container>.txt`      | Per-PID perf counters, 1 Hz: cycles, instructions, cache misses/references, LLC miss/reference, context switches, migrations, page faults |
| `power.txt`                 | System-wide RAPL energy: pkg, cores, gpu                      |
| `docker_stats.csv`          | `timestamp,name,cpu_perc,mem_usage,net_io,block_io,pids` (~2 s cadence) |
| `experiment_info.txt`       | Experiment metadata (containers, PIDs, CPU, kernel, docker)   |
| `patrol_log.txt`            | Container logs (`experiment_runner`, else `container1`)       |
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