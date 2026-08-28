# Replicating the AIRIC methodology for ROS2 robot controllers

This report summarizes the analysis of the CPU-scaling experiments in
`data-collection/results/`, conducted by adapting the methodology of the
AIRIC paper [1] (a vRAN study) to ROS2 navigation controllers. All 7 plots
and the underlying metrics are reproducible from the scripts in
`data-collection/analysis/`.

[1] J. X. Salvat Lozano et al., "AIRIC: Orchestration of Virtualized Radio
Access Networks with Noisy Neighbours," IEEE Journal on Selected Areas in
Communications, 2023 (arXiv:2311.04649v1).

## Methodology mapping (paper -> our setup)

| Paper concept (vRAN)              | Our setup (robot controllers)                          |
|-----------------------------------|-------------------------------------------------------|
| Virtualized base station (vBS)    | ROS2 navigation stack (Panther controller) in Docker  |
| srsRAN stack                      | Nav2 + AMCL + Velocity Smoother + Velodyne pipeline   |
| USRP B210 SDR front-ends          | Simulated (patrol mission goals)                       |
| Docker container per vBS         | Docker container per robot (`<ns>_controller`)        |
| Shared CPU pool, CPU pinning      | All containers pinned to CPUs 0-1                      |
| `perf stat` for HW counters       | Same: cycles, instructions, cache misses, ctx switches |
| RAPL `power/energy-pkg/`          | Same                                                   |
| Number of vBS (1-5)               | Number of robots (1-4)                                 |
| Throughput = wireless throughput   | `cmd_vel_nav` publish rate (Hz) + goals/hour           |
| Ideal isolation = N x single usage | Same formula                                          |

### What we could not replicate and why

- **Fig 1 vs Fig 8 split (no-pinning vs pinning):** every run already pins
  containers to CPUs 0-1, so we only have the "pinned" case (Fig 8). The
  paper's Fig 1 shows the unpinned baseline. Would need a second
  experiment without `CpusetCpus`.
- **Fig 2 (throughput vs CPU allocation):** the paper sweeps CPU
  allocations (0.2-0.8 cores). We have only one allocation (CPUs 0-1, 2
  cores), so the curve would be a single point. We use a *throughput vs
  #robots* view instead as a proxy.
- **Fig 6 (hyper-threading on/off), Fig 7 (netns / seccomp toggles):**
  require additional experiment configurations we did not run.
- **Fig 17-22 (AIRIC RL agent):** require training a Relation Network + DQN
  controller. Out of scope for this replication; the raw power-vs-#robots
  curve in `power_vs_robots.png` could serve as an input to a future RL
  agent.
- **Scale:** paper uses 1-5 vBS on a 4-core i7-7700K; we use 1-4 robots on a
  14-core (20-thread) i7-1280P, with 2 cores dedicated to the experiment.

## Experiment setup

- **Hardware:** 12th Gen Intel Core i7-1280P (14 cores, 20 threads). Turbo
  disabled, governor=performance on the experiment CPUs.
- **CPU pool:** CPUs 0-1 (one physical core, 2 threads) for all containers;
  metrics collection on CPUs 2-19.
- **ROS2:** RMW = `rmw_cyclonedds_cpp`, ROS_DOMAIN_ID=0.
- **Duration:** 500 s per run, 1 s `perf stat -I` interval, ~2 s docker stats
  interval.
- **Runs:** 1, 2, 3, 4 concurrent robot controllers (homogeneous, each
  running a patrol mission).

## Replicated plots

All PNGs are in `data-collection/analysis/plots/`. Run any plot script with
`python3 data-collection/analysis/plot_<name>.py`.

### 1. `fig1_8_cpu_vs_robots.png` — Figs 1 & 8 analog
Aggregated CPU usage (cores, 95th pct) vs #robots, with the ideal-isolation
baseline `N x (1-robot usage)`. Annotated with the overhead % above ideal.
- Source: `plot_cpu_vs_robots.py`
- **Finding:** 0.50 -> 0.93 -> 1.48 -> 2.03 cores. Overhead vs ideal is only
  +2.2% at 4 robots. The 2-core pool is not saturated at 4 robots, so the
  CPU-time overhead is small in absolute terms. Per-robot CPU stays ~0.50
  cores until n=4, where it bumps to 0.57-0.60.

### 2. `fig9a_context_switches.png` — Fig 9a analog
Context switches per second of one robot (`panther`) vs #robots, with CPU
pinning. Mean with p5-p95 error bars.
- Source: `plot_context_switches.py`
- **Finding:** ~9000 ctx switches/s and roughly flat across 1-4 robots.
  Matches the paper's "with pinning" expectation (Fig 9a): pinned threads
  only contend with sibling threads of the same instance, so context
  switching is independent of #robots.

### 3. `fig10_ipc.png` — Fig 10 analog
Instructions per cycle (IPC) of one robot vs #robots, with the red IPC=1
boundary line.
- Source: `plot_ipc.py`
- **Finding:** IPC stays at ~1.18 across 1-4 robots (above the IPC=1
  boundary). Unlike the paper, where IPC dropped below 1 at 3+ vBS, our
  robot controllers stay instruction-bounded at this scale. This is the
  flip side of the small CPU overhead: more instructions are being
  executed, but each one still completes in ~1 cycle on average.

### 4. `fig11_mpki.png` — Fig 11 analog
Misses per 1000 instructions (MPKI) of one robot vs #robots, including the
LLC-only MPKI variant.
- Source: `plot_mpki.py`
- **Finding:** **MPKI triples: 1.43 -> 2.45 -> 3.16 -> 4.27** (a ~200%
  increase at 4 robots). LLC MPKI tracks it almost exactly. This is the
  paper's main finding reproduced in our setup: **cache contention is the
  dominant noisy-neighbor effect**, even when CPU time and IPC look
  healthy. The cache is where the 4 concurrent controllers step on each
  other.

### 5. `power_vs_robots.png` — new plot (not in paper)
RAPL package power (W) vs #robots, with cores-power overlay and per-step
marginal power annotations.
- Source: `plot_power.py`
- **Finding:** 6.04 -> 6.90 -> 8.16 -> 9.65 W (package). Marginal cost per
  added robot: +0.86, +1.25, +1.50 W. Useful as a raw input for any
  energy-aware controller. The paper's Fig 20 reports *savings* of an RL
  agent, which we do not have.

### 6. `fig2_cmd_vel_nav_rate.png` — Fig 2 analog (throughput proxy)
`cmd_vel_nav` publish rate (Hz) vs #robots: mean and 5th percentile, with
  the 10 Hz setpoint reference.
- Source: `plot_ros2_rates.py`
- **Finding:** the rate holds at 10 Hz for 1-3 robots, then **collapses at
  4**: mean 9.18 Hz, 5th-pct 6.3 Hz. Jitter explodes from 1.5 ms (1 robot)
  to 60 ms (4 robots), p95 jitter 155 ms. This is the user-visible
  throughput collapse the paper warns about: cache contention eventually
  destabilises the control loop, even though CPU usage looks fine.

### 7. `ros2_goals_vs_robots.png` — new plot (ROS2-specific, beyond paper)
Two-panel: (left) boxplot of navigation goal durations per #robots with
  individual goals overlaid; (right) mean goal duration and goals/hour on
  twin axes.
- Source: `plot_ros2_goals.py`
- **Finding:** `duration_p95`: 65.7 -> 105 -> 148 -> 262 s (4x slowdown).
  Goals/hour *peaks at 3 robots* (108/h) then drops to 57.6/h at 4. So at
  4 robots, each robot is slower *and* the fleet completes fewer goals
  per hour overall. This is the application-level view of the noisy
  neighbor effect.

## Headline conclusions

1. **Cache contention is the bottleneck**, reproducing the paper's main
   finding: MPKI triples from 1 to 4 robots while CPU usage stays close to
   ideal and IPC stays >1. The paper traces this to LLC contention.
2. **Throughput collapses at 4 robots** in our 2-core pool: `cmd_vel_nav`
   drops to 6.3 Hz (5th pct) and goals/hour falls below the 1-robot level.
3. **CPU pinning works as advertised** for context switches (flat ~9000/s),
   matching the paper's Fig 9a.
4. **3 robots is the sweet spot** for goal throughput in this
   configuration; 4 robots hurts both per-robot latency and fleet-wide
   throughput.

## File layout

```
data-collection/analysis/
  parse.py              # per-file-type parsers -> pandas DataFrames
  metrics.py            # cross-run aggregations (CPU, perf, power, ros2)
  plot_cpu_vs_robots.py
  plot_context_switches.py
  plot_ipc.py
  plot_mpki.py
  plot_power.py
  plot_ros2_rates.py
  plot_ros2_goals.py
  plots/
    fig1_8_cpu_vs_robots.png
    fig9a_context_switches.png
    fig10_ipc.png
    fig11_mpki.png
    power_vs_robots.png
    fig2_cmd_vel_nav_rate.png
    ros2_goals_vs_robots.png
  report.md             # this file
```

## How to reproduce

```
cd data-collection/analysis
python3 parse.py        # smoke-test the parsers
python3 metrics.py      # print all metric tables
python3 plot_cpu_vs_robots.py
python3 plot_context_switches.py
python3 plot_ipc.py
python3 plot_mpki.py
python3 plot_power.py
python3 plot_ros2_rates.py
python3 plot_ros2_goals.py
```

Dependencies (all installed on this system): `python3-pandas`,
`python3-matplotlib`, `python3-seaborn`, `python3-numpy`.