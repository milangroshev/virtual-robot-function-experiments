#!/bin/bash
# =============================================================================
# run_experiment.sh — Single-script orchestrator for all metrics collection
# =============================================================================
# Collects in parallel:
#   • perf stat         — per-container HW counters (IPC, cache, ctx switches)
#   • RAPL power        — system-wide energy (pkg, cores, gpu)
#   • docker stats      — CPU%, memory, net/block I/O per container
#   • ROS2 app metrics  — topic rates, jitter, goal durations, recoveries
#                         (runs inside a Docker container for DDS visibility)
#
# Everything shares one output directory. No sub-scripts.
#
# Usage:
#   ./run_experiment.sh <experiment_name> <duration_secs> <ros2_namespaces> <container1> [container2 ... container5]
#
# Arguments:
#   experiment_name   : Descriptive label (used in output directory name)
#   duration_secs     : Collection window in seconds
#   ros2_namespaces   : Comma-separated ROS2 namespaces (e.g. panther,panther2)
#   containerN        : Docker containers to profile with perf/docker-stats
#
# Environment (optional):
#   ROS_DOMAIN_ID     : DDS domain ID                   (default: 0)
#   RMW_IMPLEMENTATION: RMW backend                     (default: rmw_cyclonedds_cpp)
#   EXPERIMENT_CPUS   : CPUs reserved for experiment     (default: 0-3)
#   METRICS_CPUSET    : CPUs for metrics container       (default: 4-19)
#
# Examples:
#   # Baseline — 2 controllers on isolated cores
#   ./run_experiment.sh baseline_ctrl 300 panther,panther2 panther_controller panther2_controller
#
#   # Noisy-neighbor — 2 controllers + 1 contending planner
#   ./run_experiment.sh ctrl_plus_planner 300 panther,panther2 \
#       panther_controller panther2_controller panther_planner
#
#   # Custom CPU pool (1 P-core)
#   EXPERIMENT_CPUS="0,1" METRICS_CPUSET="2-19" \
#       ./run_experiment.sh small_pool 300 panther panther_controller
# =============================================================================

set -e

# ── Argument parsing ─────────────────────────────────────────────────────
if [ "$#" -lt 4 ]; then
    echo "Usage: $0 <experiment_name> <duration_secs> <ros2_namespaces> <container1> [container2 ... container5]"
    echo ""
    echo "  ros2_namespaces : comma-separated ROS2 namespaces (e.g. panther,panther2)"
    echo "  containerN      : Docker containers to profile with perf/docker-stats"
    echo ""
    echo "Environment variables:"
    echo "  EXPERIMENT_CPUS  : CPUs for experiment pool  (default: 0-3)"
    echo "  METRICS_CPUSET   : CPUs for metrics container (default: 4-19)"
    echo "  ROS_DOMAIN_ID    : DDS domain ID              (default: 0)"
    echo "  RMW_IMPLEMENTATION : RMW backend              (default: rmw_cyclonedds_cpp)"
    exit 1
fi

EXPERIMENT="$1"
DURATION="$2"
ROS2_NS="$3"
shift 3
CONTAINERS=("$@")

MAX_CONTAINERS=5
if [ "${#CONTAINERS[@]}" -gt "$MAX_CONTAINERS" ]; then
    echo "ERROR: too many containers (${#CONTAINERS[@]}). Max is $MAX_CONTAINERS."
    exit 1
fi

# ── Paths & tunables ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd "$SCRIPT_DIR/../docker/config" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPL="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ── CPU topology (hardcoded to match isolation setup) ────────────────────
# Default: 2 P-cores for experiments, everything else for metrics/OS
#
#   Experiment pool:  CPUs 0-1   (P-Core 0, 4700 MHz)
#   Metrics/OS:       CPUs 2-19  (unused P-cores + E-cores)
#
# Override via environment variables for different pool sizes:
#   1 P-core:  EXPERIMENT_CPUS="0,1"     METRICS_CPUSET="2-19"
#   2 P-cores: EXPERIMENT_CPUS="0-3"     METRICS_CPUSET="4-19"
#   3 P-cores: EXPERIMENT_CPUS="0-3,8-9" METRICS_CPUSET="4-7,10-19"
EXPERIMENT_CPUS="${EXPERIMENT_CPUS:-0-1}"
METRICS_CPUSET="${METRICS_CPUSET:-2-19}"

OUTPUT_DIR="results/${EXPERIMENT}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"
ABS_OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

ROS2_CONTAINER_NAME="ros2_metrics_${EXPERIMENT}_$$"

echo "=============================================="
echo " Experiment: $EXPERIMENT"
echo "=============================================="
echo " Duration:        ${DURATION}s"
echo " ROS2 NS:         $ROS2_NS"
echo " Containers:      ${CONTAINERS[*]}"
echo " Experiment CPUs: $EXPERIMENT_CPUS"
echo " Metrics CPUs:    $METRICS_CPUSET"
echo " Config dir:      $CONFIG_DIR"
echo " ROS_DOMAIN_ID:   $ROS_DOMAIN_ID"
echo " RMW:             $RMW_IMPL"
echo " Output:          $OUTPUT_DIR"
echo "=============================================="

# ── Validate prerequisites ───────────────────────────────────────────────
PREREQ_OK=true

if [ ! -d "$CONFIG_DIR" ]; then
    echo "ERROR: Config directory not found at $CONFIG_DIR"
    PREREQ_OK=false
fi

if [ ! -f "$CONFIG_DIR/cyclonedds-edge.xml" ]; then
    echo "ERROR: CycloneDDS config not found at $CONFIG_DIR/cyclonedds-edge.xml"
    PREREQ_OK=false
fi

if ! docker image inspect nav2-profiling:latest &>/dev/null; then
    echo "ERROR: Docker image 'nav2-profiling:latest' not found. Build it first."
    PREREQ_OK=false
fi

if [ ! -f "$SCRIPT_DIR/collect_ros2_metrics.py" ]; then
    echo "ERROR: collect_ros2_metrics.py not found in $SCRIPT_DIR"
    PREREQ_OK=false
fi

if [ "$PREREQ_OK" = false ]; then
    echo "Aborting due to missing prerequisites."
    exit 1
fi

# =============================================================================
# get_worker_pids <container_name>
#   Returns comma-separated PIDs of actual workload processes, skipping
#   shells, ros2 launch wrappers, and parent orchestrator processes.
# =============================================================================
get_worker_pids() {
    local cname="$1"

    if ! docker inspect "$cname" --format '{{.State.Pid}}' &>/dev/null; then
        echo ""
        return
    fi

    local init_pid
    init_pid=$(docker inspect "$cname" --format '{{.State.Pid}}' 2>/dev/null)
    if [ -z "$init_pid" ] || [ "$init_pid" = "0" ]; then
        echo ""
        return
    fi

    local pids

    # Strategy 1: stress-ng workers (CMD contains "[run]")
    pids=$(docker top "$cname" -eo pid,cmd 2>/dev/null \
        | grep '\[run\]' \
        | awk '{print $1}' \
        | tr '\n' ',' | sed 's/,$//')
    if [ -n "$pids" ]; then echo "$pids"; return; fi

    # Strategy 2: compiled ROS2 node binaries (/opt/ros/<distro>/lib/...)
    pids=$(docker top "$cname" -eo pid,cmd 2>/dev/null \
        | grep -E '/opt/ros/[^/]+/lib/' \
        | awk '{print $1}' \
        | tr '\n' ',' | sed 's/,$//')
    if [ -n "$pids" ]; then echo "$pids"; return; fi

    # Strategy 3: leaf processes (no children, not shells/sleep)
    pids=$(docker top "$cname" -eo pid,ppid,cmd 2>/dev/null \
        | tail -n +2 \
        | awk '!/bash|\/bin\/sh|sleep/ {pids[$1]=$1; parents[$2]=$2}
               END {for (p in pids) if (!(p in parents)) print p}' \
        | tr '\n' ',' | sed 's/,$//')
    if [ -n "$pids" ]; then echo "$pids"; return; fi

    # Strategy 4: last resort — container init PID
    echo "$init_pid"
}

# ── Resolve worker PIDs ─────────────────────────────────────────────────
ACTIVE_CONTAINERS=()
ACTIVE_PIDS=()

echo ""
echo "--- Resolving worker PIDs ---"
for cname in "${CONTAINERS[@]}"; do
    pids=$(get_worker_pids "$cname")
    if [ -z "$pids" ]; then
        echo "WARNING: Container $cname not running or no workers — skipping"
        continue
    fi
    ACTIVE_CONTAINERS+=("$cname")
    ACTIVE_PIDS+=("$pids")
    echo "  $cname → PIDs: $pids"
done

if [ "${#ACTIVE_CONTAINERS[@]}" -eq 0 ]; then
    echo "ERROR: no running containers with worker processes found. Aborting."
    exit 1
fi

echo ""
echo "Profiling ${#ACTIVE_CONTAINERS[@]} of ${#CONTAINERS[@]} requested container(s)."

# ── Show process trees ──────────────────────────────────────────────────
echo ""
echo "--- Processes being profiled ---"
for cname in "${ACTIVE_CONTAINERS[@]}"; do
    echo "[$cname]"
    docker top "$cname" -eo pid,ppid,psr,pcpu,comm 2>/dev/null | head -20
    echo ""
done

# ── Save experiment metadata ────────────────────────────────────────────
{
    echo "Experiment: $EXPERIMENT"
    echo "Date: $(date)"
    echo "Duration: ${DURATION}s"
    echo "ROS2 Namespaces: $ROS2_NS"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "RMW: $RMW_IMPL"
    echo "CycloneDDS: $CONFIG_DIR/cyclonedds-edge.xml"
    echo ""

    echo "=== Container Configuration ==="
    for i in "${!ACTIVE_CONTAINERS[@]}"; do
        cname="${ACTIVE_CONTAINERS[$i]}"
        cpuset=$(docker inspect "$cname" --format '{{.HostConfig.CpusetCpus}}' 2>/dev/null || echo "unknown")
        mem_limit=$(docker inspect "$cname" --format '{{.HostConfig.Memory}}' 2>/dev/null || echo "unknown")
        echo "Container $((i+1)): $cname"
        echo "  Worker PIDs: ${ACTIVE_PIDS[$i]}"
        echo "  CpusetCpus: $cpuset"
        echo "  MemoryLimit: $mem_limit"
    done
    echo ""

    echo "=== Experiment CPU Pool ==="
    echo "Experiment CPUs: $EXPERIMENT_CPUS"
    echo "Metrics CPUs: $METRICS_CPUSET"
    echo ""

    echo "=== Hardware ==="
    echo "CPU: $(lscpu | grep "Model name" | sed 's/Model name:\s*//')"
    echo "Cores: $(nproc)"
    echo "Kernel: $(uname -r)"
    echo "Docker: $(docker version --format '{{.Server.Version}}' 2>/dev/null || echo "unknown")"
    echo ""

    echo "=== CPU Topology ==="
    lscpu --all --extended 2>/dev/null || echo "lscpu --extended not available"
    echo ""

    echo "=== CPU Isolation State ==="
    echo "Isolated CPUs: $(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo 'none')"
    echo "systemd affinity (PID 1): $(taskset -cp 1 2>&1)"
    echo "Workqueue cpumask: $(cat /sys/devices/virtual/workqueue/*/cpumask 2>/dev/null | sort -u || echo 'unknown')"
    echo ""

    echo "=== CPU Frequency & Power ==="
    echo "Turbo boost disabled: $(cat /sys/devices/system/cpu/intel_pstate/no_turbo 2>/dev/null || echo 'unknown')"
    # Show governor for experiment CPUs (expand ranges like 0-3 to individual CPUs)
    for cpu in $(echo "$EXPERIMENT_CPUS" | tr ',' '\n' | while read range; do
        if echo "$range" | grep -q '-'; then
            start=$(echo "$range" | cut -d- -f1)
            end=$(echo "$range" | cut -d- -f2)
            seq "$start" "$end"
        else
            echo "$range"
        fi
    done); do
        gov=$(cat /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
        freq=$(cat /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_cur_freq 2>/dev/null || echo "unknown")
        echo "CPU $cpu: governor=$gov  current_freq=${freq}kHz"
    done
} > "$OUTPUT_DIR/experiment_info.txt"

echo "Metadata saved to $OUTPUT_DIR/experiment_info.txt"

# =============================================================================
# Cleanup — stops ALL collectors on exit/Ctrl+C
# =============================================================================
CLEANUP_DONE=false

cleanup() {
    # Prevent double cleanup
    if [ "$CLEANUP_DONE" = true ]; then return; fi
    CLEANUP_DONE=true

    echo ""
    echo "--- Stopping all collectors ---"

    # Kill native background processes
    for pid in "${PERF_PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    kill "$POWER_PID" 2>/dev/null
    kill "$DOCKER_PID" 2>/dev/null

    # Stop ROS2 metrics container gracefully (SIGINT for clean CSV flush)
    if docker inspect "$ROS2_CONTAINER_NAME" &>/dev/null; then
        echo "Stopping ROS2 metrics container (graceful flush)..."
        docker kill --signal SIGINT "$ROS2_CONTAINER_NAME" 2>/dev/null
        # Wait up to 10s for graceful shutdown, then force
        docker wait "$ROS2_CONTAINER_NAME" &>/dev/null &
        WAIT_PID=$!
        ( sleep 10 && kill "$WAIT_PID" 2>/dev/null && docker rm -f "$ROS2_CONTAINER_NAME" 2>/dev/null ) &
        wait "$WAIT_PID" 2>/dev/null
        docker rm -f "$ROS2_CONTAINER_NAME" 2>/dev/null
    fi

    wait 2>/dev/null
    echo "All collectors stopped."
}
trap cleanup SIGINT SIGTERM EXIT

echo ""
echo "--- Starting data collection ---"
echo ""

N_ACTIVE=${#ACTIVE_CONTAINERS[@]}
TOTAL_STEPS=$((N_ACTIVE + 3))   # perf × N + RAPL + docker stats + ROS2
step=1

# =============================================================================
# 1. PERF STAT — one per container: IPC, cache, context switches
# =============================================================================
PERF_PIDS=()
for i in "${!ACTIVE_CONTAINERS[@]}"; do
    cname="${ACTIVE_CONTAINERS[$i]}"
    pids="${ACTIVE_PIDS[$i]}"
    echo "[$step/$TOTAL_STEPS] perf stat: $cname (PIDs: $pids)"
    sudo perf stat -p "$pids" \
        -e cpu_core/cycles/,cpu_core/instructions/,cpu_core/cache-misses/,cpu_core/cache-references/,cpu_core/longest_lat_cache.miss/,cpu_core/longest_lat_cache.reference/,context-switches,cpu-migrations,page-faults \
        -I 1000 \
        -o "$OUTPUT_DIR/perf_${cname}.txt" \
        2>"$OUTPUT_DIR/perf_${cname}.err" \
        sleep "$DURATION" &
    PERF_PIDS+=($!)
    step=$((step + 1))
done

# =============================================================================
# 2. POWER — System-wide RAPL
# =============================================================================
echo "[$step/$TOTAL_STEPS] perf stat: System power (RAPL)"
sudo perf stat -a \
    -e power/energy-pkg/,power/energy-cores/,power/energy-psys/ \
    -I 1000 \
    -o "$OUTPUT_DIR/power.txt" \
    2>"$OUTPUT_DIR/power.err" \
    sleep "$DURATION" &
POWER_PID=$!
step=$((step + 1))

# =============================================================================
# 3. DOCKER STATS — CPU%, Memory, Network, Block I/O
# =============================================================================
echo "[$step/$TOTAL_STEPS] docker stats: CPU%, memory, network (~2s cadence)"
echo "timestamp,name,cpu_perc,mem_usage,net_io,block_io,pids" \
    > "$OUTPUT_DIR/docker_stats.csv"

CONTAINERS_LIST="${ACTIVE_CONTAINERS[*]}"

END=$((SECONDS + DURATION))
while [ $SECONDS -lt $END ]; do
    TIMESTAMP=$(date +%s.%N)
    docker stats $CONTAINERS_LIST --no-stream \
        --format "${TIMESTAMP},{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.NetIO}},{{.BlockIO}},{{.PIDs}}" \
        >> "$OUTPUT_DIR/docker_stats.csv" 2>/dev/null
done &
DOCKER_PID=$!
step=$((step + 1))

# =============================================================================
# 4. ROS2 APPLICATION METRICS — Dockerized collector
# =============================================================================
# Runs collect_ros2_metrics.py inside nav2-profiling:latest with:
#   - Same DDS config as the edge compose (host network, CycloneDDS)
#   - Config dir mounted as /config (matching compose convention)
#   - Output dir bind-mounted so CSVs land alongside HW metrics
#   - Env vars identical to docker-compose-edge.yml
#   - Pinned to non-experiment CPUs to avoid contaminating measurements
# =============================================================================
echo "[$step/$TOTAL_STEPS] ROS2 metrics: goal durations, topic rates, jitter (Docker)"
echo "  Metrics container pinned to CPUs: $METRICS_CPUSET (outside experiment pool: $EXPERIMENT_CPUS)"

docker run -d \
    --name "$ROS2_CONTAINER_NAME" \
    --network host \
    --ipc host \
    --pid host \
    --cpuset-cpus="$METRICS_CPUSET" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION="$RMW_IMPL" \
    -e CYCLONEDDS_URI=file:///config/cyclonedds-edge.xml \
    -v "$CONFIG_DIR":/config:ro \
    -v "$SCRIPT_DIR/collect_ros2_metrics.py":/scripts/collect_ros2_metrics.py:ro \
    -v "$ABS_OUTPUT_DIR":/output \
    nav2-profiling:latest \
    bash -c "source /ros_entrypoint.sh && python3 /scripts/collect_ros2_metrics.py $ROS2_NS /output $DURATION"

ROS2_CID=$(docker inspect --format '{{.Id}}' "$ROS2_CONTAINER_NAME" 2>/dev/null)
if [ -z "$ROS2_CID" ]; then
    echo "WARNING: ROS2 metrics container failed to start — continuing without app metrics"
else
    echo "  Container: $ROS2_CONTAINER_NAME (${ROS2_CID:0:12})"
fi

# =============================================================================
# Wait for all collectors
# =============================================================================
echo ""
echo "=============================================="
echo " All collectors running for ${DURATION}s"
echo " Ctrl+C to stop early"
echo "=============================================="

# Wait for native collectors
set +e
PERF_STATUSES=()
for pid in "${PERF_PIDS[@]}"; do
    wait "$pid" 2>/dev/null
    PERF_STATUSES+=($?)
done
wait "$POWER_PID" 2>/dev/null
POWER_STATUS=$?
wait "$DOCKER_PID" 2>/dev/null
DOCKER_STATUS=$?

# Wait for ROS2 container
ROS2_STATUS=0
if [ -n "$ROS2_CID" ]; then
    echo "Waiting for ROS2 metrics container to finish..."
    ROS2_STATUS=$(docker wait "$ROS2_CONTAINER_NAME" 2>/dev/null)
    # Save container logs for debugging
    docker logs "$ROS2_CONTAINER_NAME" > "$OUTPUT_DIR/ros2_metrics.log" 2>&1
    docker rm -f "$ROS2_CONTAINER_NAME" 2>/dev/null
fi
set -e

# Prevent cleanup from trying to stop already-finished collectors
trap - EXIT

# =============================================================================
# Save container process trees and logs
# =============================================================================
echo ""
echo "--- Saving experiment logs ---"
FIRST_CONTAINER="${ACTIVE_CONTAINERS[0]}"
if docker inspect experiment_runner &>/dev/null; then
    docker logs experiment_runner > "$OUTPUT_DIR/patrol_log.txt" 2>&1
elif docker inspect experiment_panther &>/dev/null; then
    docker logs experiment_panther > "$OUTPUT_DIR/patrol_log.txt" 2>&1
elif docker inspect "$FIRST_CONTAINER" &>/dev/null; then
    docker logs "$FIRST_CONTAINER" > "$OUTPUT_DIR/patrol_log.txt" 2>&1
else
    echo "No experiment container found for logs" > "$OUTPUT_DIR/patrol_log.txt"
fi

# =============================================================================
# Validate collected outputs
# =============================================================================
echo ""
echo "--- Validating collectors ---"
for i in "${!ACTIVE_CONTAINERS[@]}"; do
    cname="${ACTIVE_CONTAINERS[$i]}"
    status="${PERF_STATUSES[$i]}"
    if [ "$status" = "0" ]; then
        # Also check file is non-empty
        if [ -s "$OUTPUT_DIR/perf_${cname}.txt" ]; then
            echo "  ✅ perf $cname: OK"
        else
            echo "  ⚠️  perf $cname: exited OK but output is empty"
        fi
    else
        echo "  ❌ perf $cname: exited with status $status"
    fi
done

if [ "$POWER_STATUS" = "0" ] && [ -s "$OUTPUT_DIR/power.txt" ]; then
    echo "  ✅ power (RAPL): OK"
else
    echo "  ⚠️  power (RAPL): exited with status $POWER_STATUS"
fi

if [ "$DOCKER_STATUS" = "0" ]; then
    DOCKER_LINES=$(wc -l < "$OUTPUT_DIR/docker_stats.csv")
    echo "  ✅ docker stats: OK ($((DOCKER_LINES - 1)) samples)"
else
    echo "  ⚠️  docker stats: exited with status $DOCKER_STATUS"
fi

if [ "$ROS2_STATUS" = "0" ]; then
    echo "  ✅ ROS2 metrics: OK"
else
    echo "  ⚠️  ROS2 metrics: exited with status $ROS2_STATUS (check ros2_metrics.log)"
fi

# Check for missing expected files
for f in "$OUTPUT_DIR"/ros2_rates.csv "$OUTPUT_DIR"/ros2_goals.csv; do
    [ -f "$f" ] || echo "  ⚠️  $(basename "$f") was not created"
done

# Report any stderr content from perf
for f in "$OUTPUT_DIR"/*.err; do
    [ -s "$f" ] && echo "  ⚠️  errors in $(basename "$f"):" && head -5 "$f"
done

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "=============================================="
echo " Collection Complete!"
echo "=============================================="
echo " Results: $OUTPUT_DIR"
echo ""
ls -lh "$OUTPUT_DIR/"
echo ""
echo " Output files:"
echo "   experiment_info.txt — Full metadata (CPU topology, isolation, governors)"
echo "   perf_*.txt          — HW counters per container (IPC, cache, ctx switches)"
echo "   power.txt           — System-wide RAPL energy"
echo "   docker_stats.csv    — CPU%, memory, net/block I/O per container"
echo "   ros2_rates.csv      — Topic Hz + jitter (cmd_vel, amcl, plan, costmap)"
echo "   ros2_goals.csv      — Per-goal duration, status, recovery counts"
echo ""
echo " Derived metrics to calculate:"
echo "   IPC  = instructions / cycles"
echo "   MPKI = cache-misses / (instructions / 1000)"
echo "   LLC hit rate = 1 - (LLC.miss / LLC.reference)"
echo "   Power/goal = total_energy / goals_completed"
echo "=============================================="
