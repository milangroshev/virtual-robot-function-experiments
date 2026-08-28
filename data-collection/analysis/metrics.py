"""Aggregate metrics across all runs for plotting.

Loads every run under data-collection/results/ via parse.py and produces
tidy DataFrames keyed by n_robots, ready for the Phase 3 plot scripts.

Public API (each returns a DataFrame; see docstrings for columns):
  load_all_runs()            -> dict[int, RunData]
  cpu_usage_df()             -> per-run aggregated CPU usage vs #robots
  cpu_usage_per_robot_df()   -> per-(run, robot) CPU usage vs #robots
  perf_first_robot_df()      -> per-interval perf counters for the first
                               robot in each run (panther), with n_robots
  perf_summary_df()          -> per-run summary stats (mean/median/p95) of
                               IPC, MPKI, context switches for the first robot
  power_df()                 -> per-interval RAPL power per run
  power_summary_df()         -> per-run mean/median/p95 power
  ros2_cmd_vel_df()          -> per-interval cmd_vel_nav hz per (run, robot)
  ros2_cmd_vel_summary_df() -> per-run mean/5th-pct cmd_vel_nav hz
  ros2_goals_df()            -> all goals across runs with n_robots col
  ros2_goals_summary_df()    -> per-run goal completion stats

Warmup handling: the first WARMUP_S seconds of each run are dropped from
per-interval (perf, power, ros2_rates) metrics to exclude startup
transients. docker_stats uses a quantile over the full series (paper-style
95th percentile), which is robust to a few warmup rows.

Zeros in ros2_rates (topic not yet publishing) are dropped before computing
rate statistics.
"""

from __future__ import annotations

import os
from typing import Dict, Optional

import numpy as np
import pandas as pd

from parse import RunData, Run, discover_runs, load_run

# Seconds to drop at the start of each run for per-interval metrics.
WARMUP_S = 5.0
# Topic used as the throughput proxy (Fig 2 analog).
THROUGHPUT_TOPIC = "cmd_vel_nav"
# First-robot namespace used for per-instance perf analysis (paper picks one).
FIRST_ROBOT_NS = "panther"


# ---------------------------------------------------------------------------
# Load all runs once
# ---------------------------------------------------------------------------


def load_all_runs(results_root: Optional[str] = None) -> Dict[int, RunData]:
    """Load every run; return {n_robots: RunData}."""
    if results_root is None:
        here = os.path.dirname(os.path.abspath(__file__))
        results_root = os.path.normpath(os.path.join(here, "..", "results"))
    runs = discover_runs(results_root)
    return {r.n_robots: load_run(r) for r in runs}


# ---------------------------------------------------------------------------
# CPU usage (docker_stats) — Figs 1 & 8 analog
# ---------------------------------------------------------------------------


def _trim_warmup(df: pd.DataFrame, time_col: str,
                 warmup: float = WARMUP_S) -> pd.DataFrame:
    """Drop rows whose time_col value is < warmup seconds from the first."""
    if df.empty:
        return df
    t0 = df[time_col].min()
    return df[df[time_col] >= t0 + warmup].copy()


def cpu_usage_df(runs: Dict[int, RunData]) -> pd.DataFrame:
    """Per-run aggregated CPU usage vs #robots (one row per #robots).

    Aggregates the docker CPU across containers at each timestamp, then
    takes the 95th percentile across the whole run. Also reports per-robot
    95th-pct usage and the 'ideal isolation' baseline = N * single-robot
    usage.

    Columns:
      n_robots
      cpu_p95         : 95th pct of summed cpu_cores across robots
      cpu_mean        : mean of summed cpu_cores
      cpu_per_robot_p95: 95th pct of an individual robot's cpu_cores
                        (averaged across robots when >1)
      cpu_ideal       : n_robots * cpu_per_robot_p95 at n=1
    """
    rows = []
    one_robot_per_robot = None
    for n, rd in sorted(runs.items()):
        d = rd.docker
        # Sum across containers at each timestamp.
        agg = (d.groupby("timestamp")["cpu_cores"]
                 .sum()
                 .reset_index(name="cpu_cores_sum"))
        per_robot = (d.groupby("timestamp")["cpu_cores"]
                       .mean()
                       .reset_index(name="cpu_cores_per_robot"))
        cpu_p95 = float(agg["cpu_cores_sum"].quantile(0.95))
        cpu_mean = float(agg["cpu_cores_sum"].mean())
        cpu_per_robot_p95 = float(per_robot["cpu_cores_per_robot"].quantile(0.95))
        if n == 1:
            one_robot_per_robot = cpu_per_robot_p95
        rows.append({
            "n_robots": n,
            "cpu_p95": cpu_p95,
            "cpu_mean": cpu_mean,
            "cpu_per_robot_p95": cpu_per_robot_p95,
        })
    df = pd.DataFrame(rows)
    if one_robot_per_robot is not None and one_robot_per_robot > 0:
        df["cpu_ideal"] = df["n_robots"] * one_robot_per_robot
    else:
        df["cpu_ideal"] = np.nan
    return df


def cpu_usage_per_robot_df(runs: Dict[int, RunData]) -> pd.DataFrame:
    """Per-(run, robot) 95th-pct CPU usage, long format.

    Columns: n_robots, namespace, cpu_p95
    """
    rows = []
    for n, rd in sorted(runs.items()):
        for ns, grp in rd.docker.groupby("namespace"):
            rows.append({
                "n_robots": n,
                "namespace": ns,
                "cpu_p95": float(grp["cpu_cores"].quantile(0.95)),
            })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Perf counters (first robot) — Figs 9a, 10, 11 analog
# ---------------------------------------------------------------------------


def perf_first_robot_df(runs: Dict[int, RunData],
                        ns: str = FIRST_ROBOT_NS) -> pd.DataFrame:
    """Per-interval perf counters for the first robot in each run, long.

    Columns: n_robots, time, cycles, instructions, cache_misses,
             cache_references, llc_miss, llc_reference, context_switches,
             cpu_migrations, page_faults, ipc, mpki, llc_mpki
    """
    frames = []
    for n, rd in sorted(runs.items()):
        if ns not in rd.perf:
            continue
        p = rd.perf[ns].copy()
        p = _trim_warmup(p, "time")
        p["n_robots"] = n
        frames.append(p)
    if not frames:
        return pd.DataFrame()
    df = pd.concat(frames, ignore_index=True)
    return df


def perf_summary_df(runs: Dict[int, RunData],
                    ns: str = FIRST_ROBOT_NS) -> pd.DataFrame:
    """Per-run summary of IPC, MPKI, LLC MPKI, context switches for the
    first robot. One row per n_robots.

    Columns: n_robots, ipc_mean, ipc_p05, ipc_p50, ipc_p95,
             mpki_mean, mpki_p05, mpki_p50, mpki_p95,
             llc_mpki_mean, llc_mpki_p50,
             ctxsw_mean, ctxsw_p05, ctxsw_p50, ctxsw_p95
    """
    rows = []
    for n, rd in sorted(runs.items()):
        if ns not in rd.perf:
            continue
        p = _trim_warmup(rd.perf[ns], "time")
        def q(col, ql):
            return float(p[col].quantile(ql))
        rows.append({
            "n_robots": n,
            "ipc_mean": float(p["ipc"].mean()),
            "ipc_p05": q("ipc", 0.05),
            "ipc_p50": q("ipc", 0.50),
            "ipc_p95": q("ipc", 0.95),
            "mpki_mean": float(p["mpki"].mean()),
            "mpki_p05": q("mpki", 0.05),
            "mpki_p50": q("mpki", 0.50),
            "mpki_p95": q("mpki", 0.95),
            "llc_mpki_mean": float(p["llc_mpki"].mean()),
            "llc_mpki_p50": q("llc_mpki", 0.50),
            "ctxsw_mean": float(p["context_switches"].mean()),
            "ctxsw_p05": q("context_switches", 0.05),
            "ctxsw_p50": q("context_switches", 0.50),
            "ctxsw_p95": q("context_switches", 0.95),
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Power (RAPL)
# ---------------------------------------------------------------------------


def power_df(runs: Dict[int, RunData]) -> pd.DataFrame:
    """Per-interval RAPL power for each run, long.

    Columns: n_robots, time, energy_pkg_j, energy_cores_j, energy_psys_j,
             power_pkg_w, power_cores_w, power_psys_w
    """
    frames = []
    for n, rd in sorted(runs.items()):
        p = _trim_warmup(rd.power.copy(), "time")
        p["n_robots"] = n
        frames.append(p)
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def power_summary_df(runs: Dict[int, RunData]) -> pd.DataFrame:
    """Per-run power summary. One row per n_robots.

    Columns: n_robots, power_pkg_mean, power_pkg_p50, power_pkg_p95,
             power_cores_mean, power_cores_p50, power_cores_p95
    """
    rows = []
    for n, rd in sorted(runs.items()):
        p = _trim_warmup(rd.power, "time")
        rows.append({
            "n_robots": n,
            "power_pkg_mean": float(p["power_pkg_w"].mean()),
            "power_pkg_p50": float(p["power_pkg_w"].quantile(0.50)),
            "power_pkg_p95": float(p["power_pkg_w"].quantile(0.95)),
            "power_cores_mean": float(p["power_cores_w"].mean()),
            "power_cores_p50": float(p["power_cores_w"].quantile(0.50)),
            "power_cores_p95": float(p["power_cores_w"].quantile(0.95)),
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# ROS2 cmd_vel_nav rate (Fig 2 throughput proxy)
# ---------------------------------------------------------------------------


def ros2_cmd_vel_df(runs: Dict[int, RunData],
                    topic: str = THROUGHPUT_TOPIC) -> pd.DataFrame:
    """Per-interval cmd_vel_nav hz for each (run, robot), long.

    Zeros are kept here (caller can filter for stats). Warmup is trimmed.

    Columns: n_robots, namespace, elapsed_s, hz, jitter_ms
    """
    frames = []
    for n, rd in sorted(runs.items()):
        if rd.ros2_rates is None:
            continue
        sub = rd.ros2_rates[rd.ros2_rates["topic"] == topic].copy()
        sub = _trim_warmup(sub, "wall_time")
        sub = sub.rename(columns={"wall_time": "time"})
        sub["n_robots"] = n
        frames.append(sub[["n_robots", "namespace", "time", "elapsed_s",
                           "hz", "jitter_ms"]])
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def ros2_cmd_vel_summary_df(runs: Dict[int, RunData],
                            topic: str = THROUGHPUT_TOPIC) -> pd.DataFrame:
    """Per-run summary of cmd_vel_nav hz. One row per n_robots, aggregating
    across robots (each robot's non-zero hz samples are averaged, then
    robots are averaged).

    Columns: n_robots, hz_mean, hz_p05, hz_p50, hz_p95,
             jitter_mean, jitter_p95
    """
    rows = []
    for n, rd in sorted(runs.items()):
        if rd.ros2_rates is None:
            continue
        sub = rd.ros2_rates[rd.ros2_rates["topic"] == topic].copy()
        sub = _trim_warmup(sub, "wall_time")
        # Per-robot: drop zeros (topic not yet publishing), then take mean.
        sub_nz = sub[sub["hz"] > 0]
        if sub_nz.empty:
            continue
        per_robot = sub_nz.groupby("namespace")["hz"].mean()
        per_robot_jitter = sub_nz.groupby("namespace")["jitter_ms"].mean()
        rows.append({
            "n_robots": n,
            "hz_mean": float(per_robot.mean()),
            "hz_p05": float(sub_nz["hz"].quantile(0.05)),
            "hz_p50": float(sub_nz["hz"].quantile(0.50)),
            "hz_p95": float(sub_nz["hz"].quantile(0.95)),
            "jitter_mean": float(per_robot_jitter.mean()),
            "jitter_p95": float(sub_nz["jitter_ms"].quantile(0.95)),
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# ROS2 goals
# ---------------------------------------------------------------------------


def ros2_goals_df(runs: Dict[int, RunData]) -> pd.DataFrame:
    """All goals across runs with n_robots column.

    Columns: n_robots, wall_time, namespace, status, duration_s,
             path_length_m, recoveries_spin, recoveries_backup,
             recoveries_wait, recoveries_drive_on_heading
    """
    frames = []
    for n, rd in sorted(runs.items()):
        if rd.ros2_goals is None or rd.ros2_goals.empty:
            continue
        g = rd.ros2_goals.copy()
        g["n_robots"] = n
        frames.append(g)
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def ros2_goals_summary_df(runs: Dict[int, RunData]) -> pd.DataFrame:
    """Per-run goal-completion summary. One row per n_robots.

    Columns: n_robots, n_goals, duration_mean, duration_p50, duration_p95,
             goals_per_hour
    """
    rows = []
    for n, rd in sorted(runs.items()):
        if rd.ros2_goals is None or rd.ros2_goals.empty:
            continue
        g = rd.ros2_goals
        duration = g["duration_s"]
        # goals/hour based on experiment duration (default 500s)
        dur_total = rd.info.duration_s or 500
        rows.append({
            "n_robots": n,
            "n_goals": int(len(g)),
            "duration_mean": float(duration.mean()),
            "duration_p50": float(duration.quantile(0.50)),
            "duration_p95": float(duration.quantile(0.95)),
            "goals_per_hour": float(len(g)) / dur_total * 3600.0,
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# CLI smoke-test
# ---------------------------------------------------------------------------


def _main() -> None:
    runs = load_all_runs()
    print(f"Loaded {len(runs)} runs: n_robots = {sorted(runs)}")

    print("\n--- CPU usage summary ---")
    print(cpu_usage_df(runs).to_string(index=False))

    print("\n--- CPU usage per robot (first 10) ---")
    print(cpu_usage_per_robot_df(runs).head(10).to_string(index=False))

    print("\n--- Perf summary (first robot = panther) ---")
    print(perf_summary_df(runs).to_string(index=False))

    print("\n--- Power summary ---")
    print(power_summary_df(runs).to_string(index=False))

    print("\n--- cmd_vel_nav rate summary ---")
    print(ros2_cmd_vel_summary_df(runs).to_string(index=False))

    print("\n--- ROS2 goals summary ---")
    print(ros2_goals_summary_df(runs).to_string(index=False))


if __name__ == "__main__":
    _main()