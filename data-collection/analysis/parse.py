"""Parsers for the data-collection result files (pandas-based).

Reads the raw per-run output produced by `collect_hw_os_and_ros_metrics.sh`
and returns tidy pandas DataFrames keyed by (n_robots, namespace, time).

Dependencies (all installed):
  - pandas >= 3.0
  - numpy >= 1.26
  - matplotlib, seaborn (used only by plot scripts, not here)

File formats (verified in Phase 0):
  - docker_stats.csv : timestamp,name,cpu_perc,mem_usage,net_io,block_io,pids
                       cpu_perc is a string like "42.26%"; name is "<ns>_controller"
  - perf_<container>.txt : `perf stat -I 1s` output. 9 events per 1s interval
                       (cycles, instructions, cache-misses, cache-references,
                        longest_lat_cache.miss, longest_lat_cache.reference,
                        context-switches, cpu-migrations, page-faults)
                       Occasional scaling annotation "(100.01%)" after a count
                       must be stripped before parsing.
  - power.txt : `perf stat -I 1s` for RAPL. 3 events per interval
                (power/energy-pkg, power/energy-cores, power/energy-psys) in Joules
  - ros2_rates.csv : wide format, per-namespace column pairs
                     <ns>/<topic>_hz, <ns>/<topic>_jitter_ms for 5 topics,
                     plus <ns>/path_length_m
  - ros2_goals.csv : one row per completed navigation goal
  - experiment_info.txt : provenance metadata (parsed minimally)
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import pandas as pd
import numpy as np

# ---------------------------------------------------------------------------
# Run discovery
# ---------------------------------------------------------------------------

# Result directories are named "<n>_robot_<timestamp>". We infer n_robots
# from the leading word ("one", "two", ...).
_N_WORDS = {
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10,
}


def _n_robots_from_dirname(dirname: str) -> int:
    """`one_robot_20260826_150837` -> 1."""
    first = dirname.split("_")[0]
    if first not in _N_WORDS:
        raise ValueError(f"Cannot parse n_robots from directory name: {dirname!r}")
    return _N_WORDS[first]


@dataclass
class Run:
    """One experiment run (a directory under data-collection/results/)."""

    n_robots: int
    dirpath: str
    dirname: str
    namespaces: List[str] = field(default_factory=list)
    perf_files: Dict[str, str] = field(default_factory=dict)
    docker_stats_csv: str = ""
    power_txt: str = ""
    ros2_rates_csv: str = ""
    ros2_goals_csv: str = ""
    experiment_info_txt: str = ""


def discover_runs(results_root: str) -> List[Run]:
    """Find all `<n>_robot_*` directories under `results_root` (sorted by n)."""
    runs: List[Run] = []
    for entry in sorted(os.listdir(results_root)):
        full = os.path.join(results_root, entry)
        if not os.path.isdir(full) or "_robot_" not in entry:
            continue
        try:
            n = _n_robots_from_dirname(entry)
        except ValueError:
            continue
        run = Run(n_robots=n, dirpath=full, dirname=entry)
        for p in sorted(os.listdir(full)):
            m = re.match(r"perf_(.+)_controller\.txt$", p)
            if m:
                ns = m.group(1)
                run.perf_files[ns] = os.path.join(full, p)
        run.namespaces = sorted(run.perf_files.keys())
        run.docker_stats_csv = os.path.join(full, "docker_stats.csv")
        run.power_txt = os.path.join(full, "power.txt")
        run.ros2_rates_csv = os.path.join(full, "ros2_rates.csv")
        run.ros2_goals_csv = os.path.join(full, "ros2_goals.csv")
        run.experiment_info_txt = os.path.join(full, "experiment_info.txt")
        runs.append(run)
    runs.sort(key=lambda r: r.n_robots)
    return runs


# ---------------------------------------------------------------------------
# docker_stats.csv
# ---------------------------------------------------------------------------


def parse_docker_stats(path: str) -> pd.DataFrame:
    """Parse docker_stats.csv -> tidy DataFrame.

    Columns: timestamp, name, namespace, cpu_cores, mem_mib, pids
      - cpu_perc "42.26%" -> cpu_cores 0.4226 (fraction of one core)
      - mem_usage "79.88MiB / 31.05GiB" -> mem_mib 79.88
      - name "panther2_controller" -> namespace "panther2"
    """
    df = pd.read_csv(path)
    df["cpu_cores"] = df["cpu_perc"].str.rstrip("%").astype(float) / 100.0
    df["mem_mib"] = (
        df["mem_usage"].str.extract(r"([\d.]+)\s*MiB", expand=False).astype(float)
    )
    df["namespace"] = df["name"].str.replace(r"_controller$", "", regex=True)
    keep = ["timestamp", "name", "namespace", "cpu_cores", "mem_mib", "pids"]
    return df[keep].copy()


# ---------------------------------------------------------------------------
# perf stat -I output (per-container HW counters and RAPL power)
# ---------------------------------------------------------------------------

# Event name -> short key.
_PERF_EVENTS = {
    "cpu_core/cycles/": "cycles",
    "cpu_core/instructions/": "instructions",
    "cpu_core/cache-misses/": "cache_misses",
    "cpu_core/cache-references/": "cache_references",
    "cpu_core/longest_lat_cache.miss/": "llc_miss",
    "cpu_core/longest_lat_cache.reference/": "llc_reference",
    "context-switches": "context_switches",
    "cpu-migrations": "cpu_migrations",
    "page-faults": "page_faults",
}

_POWER_EVENTS = {
    "power/energy-pkg/": "energy_pkg_j",
    "power/energy-cores/": "energy_cores_j",
    "power/energy-psys/": "energy_psys_j",
}

# Matches a perf scaling annotation like "(100.01%)" or "(99.99%)".
_SCALING_RE = re.compile(r"\(\d+(?:\.\d+)?%\)")


def _parse_perf_stat(path: str, event_map: Dict[str, str]) -> pd.DataFrame:
    """Generic `perf stat -I 1s` parser -> DataFrame with one row per interval.

    Columns: time + one column per short event key.
    Handles:
      - comment/blank lines (skipped)
      - thousands commas in counts, e.g. "504,096,501"
      - '<not counted>' / '<not supported>' -> 0
      - scaling annotation "(100.01%)" after a count (stripped)
    """
    records: List[dict] = []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            # Drop any trailing "(NN.NN%)" scaling annotation.
            line = _SCALING_RE.sub("", line).strip()
            toks = line.split()
            if len(toks) < 3:
                continue
            try:
                t = float(toks[0])
            except ValueError:
                continue
            # Last token is the event name; remove it.
            event = toks[-1]
            if event not in event_map:
                continue
            # Middle tokens = count (+ optional unit). Drop trailing unit if
            # non-numeric (e.g. "Joules").
            middle = toks[1:-1]
            if middle and not re.search(r"[0-9<>]", middle[-1]):
                middle = middle[:-1]
            if not middle:
                continue
            count_str = "".join(middle).replace(",", "")
            if "<" in count_str:
                count = 0.0
            else:
                try:
                    count = float(count_str)
                except ValueError:
                    count = 0.0
            short = event_map[event]
            records.append({"time": t, short: count})

    if not records:
        return pd.DataFrame(columns=["time"] + list(event_map.values()))

    df = pd.DataFrame(records)
    # Group by time -> one row per interval with all events as columns.
    df = df.groupby("time", as_index=False).first()
    # Ensure all expected columns exist and order them.
    for k in event_map.values():
        if k not in df:
            df[k] = 0.0
    cols = ["time"] + list(event_map.values())
    return df[cols].sort_values("time").reset_index(drop=True)


def parse_perf_container(path: str, namespace: str = "") -> pd.DataFrame:
    """Parse perf_<container>.txt -> DataFrame.

    Columns: time, cycles, instructions, cache_misses, cache_references,
             llc_miss, llc_reference, context_switches, cpu_migrations,
             page_faults, ipc, mpki, llc_mpki.

    Derived per-interval:
      ipc      = instructions / cycles   (0 if cycles==0)
      mpki     = cache_misses / instructions * 1000
      llc_mpki = llc_miss / instructions * 1000
    """
    df = _parse_perf_stat(path, _PERF_EVENTS)
    if df.empty:
        return df
    cyc = df["cycles"]
    ins = df["instructions"]
    with np.errstate(divide="ignore", invalid="ignore"):
        df["ipc"] = np.where(cyc > 0, ins / cyc, 0.0)
        df["mpki"] = np.where(ins > 0, df["cache_misses"] / ins * 1000.0, 0.0)
        df["llc_mpki"] = np.where(ins > 0, df["llc_miss"] / ins * 1000.0, 0.0)
    if namespace:
        df.insert(0, "namespace", namespace)
    return df


def parse_power(path: str) -> pd.DataFrame:
    """Parse power.txt -> DataFrame.

    Columns: time, energy_pkg_j, energy_cores_j, energy_psys_j,
             power_pkg_w, power_cores_w, power_psys_w.

    For `perf stat -I 1s`, the energy counter is joules consumed in the
    1s interval, so power (W) == energy (J).
    """
    df = _parse_perf_stat(path, _POWER_EVENTS)
    if df.empty:
        return df
    df["power_pkg_w"] = df["energy_pkg_j"]
    df["power_cores_w"] = df["energy_cores_j"]
    df["power_psys_w"] = df["energy_psys_j"]
    return df


# ---------------------------------------------------------------------------
# ros2_rates.csv  (wide format, per-namespace column pairs)
# ---------------------------------------------------------------------------

ROS2_RATE_TOPICS = ["cmd_vel_nav", "amcl_pose", "plan",
                    "local_costmap", "velodyne_filtered"]


def parse_ros2_rates(path: str) -> pd.DataFrame:
    """Parse the wide ros2_rates.csv into a tidy long DataFrame.

    Columns: wall_time, elapsed_s, namespace, topic, hz, jitter_ms,
             path_length_m

    One row per (sample, namespace, topic) triple. path_length_m is a
    per-namespace column replicated across its topic rows (NaN if absent).
    """
    wide = pd.read_csv(path)
    # Discover namespaces + topic suffix columns.
    ns_to_hz_cols: Dict[str, List[str]] = {}
    ns_to_path_col: Dict[str, str] = {}
    for col in wide.columns:
        m = re.match(r"^(.+?)/(.+?)_hz$", col)
        if m:
            ns, topic = m.group(1), m.group(2)
            ns_to_hz_cols.setdefault(ns, []).append(col)
        elif col.endswith("/path_length_m"):
            ns_to_path_col[col.rsplit("/", 1)[0]] = col

    frames: List[pd.DataFrame] = []
    for ns, hz_cols in ns_to_hz_cols.items():
        for hz_col in hz_cols:
            topic = hz_col.rsplit("/", 1)[1].rsplit("_hz", 1)[0]
            jitter_col = f"{ns}/{topic}_jitter_ms"
            sub = pd.DataFrame({
                "wall_time": wide["wall_time"],
                "elapsed_s": wide["elapsed_s"],
                "namespace": ns,
                "topic": topic,
                "hz": wide[hz_col].astype(float),
                "jitter_ms": wide[jitter_col].astype(float)
                if jitter_col in wide.columns
                else np.nan,
            })
            if ns in ns_to_path_col:
                sub["path_length_m"] = wide[ns_to_path_col[ns]].astype(float)
            else:
                sub["path_length_m"] = np.nan
            frames.append(sub)

    if not frames:
        return pd.DataFrame(columns=["wall_time", "elapsed_s", "namespace",
                                     "topic", "hz", "jitter_ms", "path_length_m"])
    return pd.concat(frames, ignore_index=True)


# ---------------------------------------------------------------------------
# ros2_goals.csv
# ---------------------------------------------------------------------------


def parse_ros2_goals(path: str) -> pd.DataFrame:
    """Parse ros2_goals.csv -> DataFrame (one row per completed goal)."""
    df = pd.read_csv(path)
    return df


# ---------------------------------------------------------------------------
# experiment_info.txt (minimal provenance extraction)
# ---------------------------------------------------------------------------


@dataclass
class ExperimentInfo:
    raw: str
    duration_s: Optional[int] = None
    rmw: Optional[str] = None
    cpuset_cpus: Optional[str] = None
    experiment_cpus: Optional[str] = None
    metrics_cpus: Optional[str] = None
    cpu_model: Optional[str] = None
    turbo_disabled: Optional[bool] = None


def parse_experiment_info(path: str) -> ExperimentInfo:
    with open(path) as f:
        raw = f.read()
    info = ExperimentInfo(raw=raw)

    def _find(prefix: str) -> Optional[str]:
        for line in raw.splitlines():
            if line.strip().startswith(prefix):
                return line.split(":", 1)[1].strip()
        return None

    dur = _find("Duration")
    if dur:
        info.duration_s = int(dur.rstrip("s"))
    info.rmw = _find("RMW")
    info.cpu_model = _find("CPU")
    info.experiment_cpus = _find("Experiment CPUs")
    info.metrics_cpus = _find("Metrics CPUs")
    # CpusetCpus appears indented inside per-container blocks; take first.
    for line in raw.splitlines():
        if line.strip().startswith("CpusetCpus:"):
            info.cpuset_cpus = line.split(":", 1)[1].strip()
            break
    for line in raw.splitlines():
        if line.strip().startswith("Turbo boost disabled:"):
            info.turbo_disabled = (line.split(":", 1)[1].strip() == "1")
            break
    return info


# ---------------------------------------------------------------------------
# Convenience: load everything for a run into a RunData dataclass
# ---------------------------------------------------------------------------


@dataclass
class RunData:
    run: Run
    info: ExperimentInfo
    docker: pd.DataFrame
    perf: Dict[str, pd.DataFrame]   # namespace -> perf DataFrame
    power: pd.DataFrame
    ros2_rates: Optional[pd.DataFrame]
    ros2_goals: Optional[pd.DataFrame]


def load_run(run: Run) -> RunData:
    """Load all data files for one run. ros2 files are optional."""
    info = parse_experiment_info(run.experiment_info_txt)
    docker = parse_docker_stats(run.docker_stats_csv)
    perf = {ns: parse_perf_container(p, namespace=ns)
            for ns, p in run.perf_files.items()}
    power = parse_power(run.power_txt)
    ros2_rates = None
    ros2_goals = None
    if os.path.exists(run.ros2_rates_csv) and os.path.getsize(run.ros2_rates_csv) > 0:
        ros2_rates = parse_ros2_rates(run.ros2_rates_csv)
    if os.path.exists(run.ros2_goals_csv) and os.path.getsize(run.ros2_goals_csv) > 0:
        ros2_goals = parse_ros2_goals(run.ros2_goals_csv)
    return RunData(run=run, info=info, docker=docker, perf=perf, power=power,
                   ros2_rates=ros2_rates, ros2_goals=ros2_goals)


# ---------------------------------------------------------------------------
# CLI smoke-test: `python3 parse.py` prints a summary per run
# ---------------------------------------------------------------------------


def _main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    results_root = os.path.normpath(os.path.join(here, "..", "results"))
    runs = discover_runs(results_root)
    print(f"Discovered {len(runs)} runs under {results_root}")
    for run in runs:
        print(f"\n=== {run.dirname} (n_robots={run.n_robots}) ===")
        print(f"  namespaces: {run.namespaces}")
        rd = load_run(run)
        d = rd.docker
        print(f"  docker_stats: {len(d)} rows, "
              f"{d['name'].nunique()} containers")
        # perf: first namespace
        ns0 = run.namespaces[0]
        p = rd.perf[ns0]
        print(f"  perf[{ns0}]: {len(p)} intervals, cols={list(p.columns)}")
        if not p.empty and "ipc" in p:
            print(f"    IPC  mean={p['ipc'].mean():.3f} "
                  f"min={p['ipc'].min():.3f} max={p['ipc'].max():.3f}")
        if not p.empty and "context_switches" in p:
            print(f"    ctxsw/s mean={p['context_switches'].mean():.0f}")
        pw = rd.power
        print(f"  power: {len(pw)} intervals")
        if not pw.empty and "power_pkg_w" in pw:
            print(f"    pkg W mean={pw['power_pkg_w'].mean():.2f}")
        if rd.ros2_rates is not None:
            print(f"  ros2_rates: {len(rd.ros2_rates)} rows, "
                  f"ns={sorted(rd.ros2_rates['namespace'].unique().tolist())}")
        if rd.ros2_goals is not None:
            print(f"  ros2_goals: {len(rd.ros2_goals)} goals")
        print(f"  info: duration={rd.info.duration_s}s rmw={rd.info.rmw} "
              f"cpuset={rd.info.cpuset_cpus} turbo_off={rd.info.turbo_disabled}")


if __name__ == "__main__":
    _main()