#!/bin/bash
# =============================================================================
# collect_hw_and_os_metrics.sh — HW, OS, and Container resource profiling
# =============================================================================
# Collects hardware counters (perf stat), power (RAPL), and container
# metrics (docker stats) during Nav2 profiling experiments.
#
# Usage:
#   ./collect_hw_and_os_metrics.sh <experiment_name> <duration_secs> [container1 ... container5]
#
# Supports 1 to 5 containers (e.g. scaling experiments from 1..N robots).
# Containers that are listed but not running are skipped; at least one must run.
#
# Examples:
#   ./collect_hw_and_os_metrics.sh single_ctrl 300 panther_controller
#   ./collect_hw_and_os_metrics.sh two_robots 300 panther_controller panther2_controller
#   ./collect_hw_and_os_metrics.sh three_robots 300 panther_controller panther2_controller panther3_controller
#   ./collect_hw_and_os_metrics.sh five_robots 600 panther_controller panther2_controller panther3_controller panther4_controller panther5_controller
# =============================================================================

set -e

EXPERIMENT=${1:-"baseline"}
DURATION=${2:-300}
shift 2 2>/dev/null || true

CONTAINERS=("$@")
MAX_CONTAINERS=5

if [ "${#CONTAINERS[@]}" -eq 0 ]; then
    echo "ERROR: provide at least one container name (up to $MAX_CONTAINERS)."
    echo "Usage: $0 <experiment_name> <duration_secs> <container1> [container2 ... container5]"
    exit 1
fi
if [ "${#CONTAINERS[@]}" -gt "$MAX_CONTAINERS" ]; then
    echo "ERROR: too many containers (${#CONTAINERS[@]}). Max is $MAX_CONTAINERS."
    exit 1
fi

OUTPUT_DIR="results/${EXPERIMENT}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "=============================================="
echo " Resource Profiling: $EXPERIMENT"
echo "=============================================="
echo " Duration:    ${DURATION}s"
echo " Requested:   ${#CONTAINERS[@]} container(s)"
for i in "${!CONTAINERS[@]}"; do
    echo "   $((i+1)). ${CONTAINERS[$i]}"
done
echo " Output:      $OUTPUT_DIR"
echo "=============================================="

# =============================================================================
# get_worker_pids <container_name>
#   Returns comma-separated PIDs of actual workload processes inside a
#   container, skipping shells, the ros2 launch wrapper, and parent
#   orchestrator processes.
#
#   Strategy 1: stress-ng workers (CMD contains "[run]")
#   Strategy 2: compiled ROS2 node binaries (CMD under /opt/ros/*/lib/)
#   Strategy 3: leaf processes — non-shell, non-sleep processes that have no
#               children (also excludes the ros2 launch python wrapper and the
#               container init, since both are parents)
#   Strategy 4: last resort — container init PID
# =============================================================================
get_worker_pids() {
    local cname="$1"

    # First, check that the container is running
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

    # Strategy 1: find stress-ng worker processes (CMD contains "[run]")
    pids=$(docker top "$cname" -eo pid,cmd 2>/dev/null \
        | grep '\[run\]' \
        | awk '{print $1}' \
        | tr '\n' ',' | sed 's/,$//')

    if [ -n "$pids" ]; then
        echo "$pids"
        return
    fi

    # Strategy 2: compiled ROS2 node binaries
    # (CMD starts with /opt/ros/<distro>/lib/<pkg>/<node>)
    pids=$(docker top "$cname" -eo pid,cmd 2>/dev/null \
        | grep -E '/opt/ros/[^/]+/lib/' \
        | awk '{print $1}' \
        | tr '\n' ',' | sed 's/,$//')

    if [ -n "$pids" ]; then
        echo "$pids"
        return
    fi

    # Strategy 3: leaf processes (non-shell, non-sleep, no children).
    # A process is a leaf if its PID never appears as another process's PPID,
    # which automatically excludes bash/sh, the ros2 launch python wrapper,
    # and the container init.
    pids=$(docker top "$cname" -eo pid,ppid,cmd 2>/dev/null \
        | tail -n +2 \
        | awk '!/bash|\/bin\/sh|sleep/ {pids[$1]=$1; parents[$2]=$2}
               END {for (p in pids) if (!(p in parents)) print p}' \
        | tr '\n' ',' | sed 's/,$//')

    if [ -n "$pids" ]; then
        echo "$pids"
        return
    fi

    # Strategy 4: last resort — use the container init PID
    echo "$init_pid"
}

# --- Resolve worker PIDs for each requested container ---
ACTIVE_CONTAINERS=()
ACTIVE_PIDS=()

echo ""
echo "--- Resolving worker PIDs ---"
for cname in "${CONTAINERS[@]}"; do
    pids=$(get_worker_pids "$cname")
    if [ -z "$pids" ]; then
        echo "WARNING: Container $cname not running or has no worker processes — skipping"
        continue
    fi
    ACTIVE_CONTAINERS+=("$cname")
    ACTIVE_PIDS+=("$pids")
    echo "Worker PIDs for $cname: $pids"
done

if [ "${#ACTIVE_CONTAINERS[@]}" -eq 0 ]; then
    echo "ERROR: no running containers with worker processes found. Aborting."
    exit 1
fi

echo ""
echo "Profiling ${#ACTIVE_CONTAINERS[@]} of ${#CONTAINERS[@]} requested container(s)."

# Show what processes we're actually tracking
echo ""
echo "--- Processes being profiled ---"
for cname in "${ACTIVE_CONTAINERS[@]}"; do
    echo "[$cname]"
    docker top "$cname" -eo pid,ppid,cmd 2>/dev/null | head -20
    echo ""
done

# --- Save experiment metadata ---
{
    echo "Experiment: $EXPERIMENT"
    echo "Date: $(date)"
    echo "Duration: ${DURATION}s"
    for i in "${!ACTIVE_CONTAINERS[@]}"; do
        echo "Container $((i+1)): ${ACTIVE_CONTAINERS[$i]} (Worker PIDs: ${ACTIVE_PIDS[$i]})"
    done
    echo "CPU: $(lscpu | grep "Model name" | sed 's/Model name:\s*//')"
    echo "Cores: $(nproc)"
    echo "Kernel: $(uname -r)"
    echo "Docker: $(docker version --format '{{.Server.Version}}' 2>/dev/null || echo "unknown")"
} > "$OUTPUT_DIR/experiment_info.txt"

echo ""
echo "--- Starting data collection ---"
echo ""

N_CONTAINERS=${#ACTIVE_CONTAINERS[@]}
TOTAL_STEPS=$((N_CONTAINERS + 2))

# =============================================================================
# PERF STAT — one per container: IPC, cache, context switches
# =============================================================================
# Events are pinned to cpu_core/ PMU explicitly: on hybrid (P/E-core) CPUs the
# generic names expand to both cpu_core and cpu_atom, and since experiment
# containers are cpuset-pinned to P-cores the cpu_atom rows are always
# "<not counted>". Using cpu_core/... emits core counters only.
PERF_PIDS=()
step=1
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
# POWER — System-wide RAPL
# =============================================================================
echo "[$step/$TOTAL_STEPS] perf stat: System power (RAPL)"
sudo perf stat -a \
    -e power/energy-pkg/,power/energy-cores/,power/energy-gpu/ \
    -I 1000 \
    -o "$OUTPUT_DIR/power.txt" \
    2>"$OUTPUT_DIR/power.err" \
    sleep "$DURATION" &
POWER_PID=$!
step=$((step + 1))

# =============================================================================
# DOCKER STATS — CPU%, Memory, Network, Block I/O
# =============================================================================
# Note: docker stats --no-stream takes ~2s per call (daemon sampling window),
# so with a tight loop the real cadence is ~2s, not 1 Hz.
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

# =============================================================================
# Wait for all collectors
# =============================================================================
echo ""
echo "=============================================="
echo " All collectors running for ${DURATION}s"
echo " Ctrl+C to stop early"
echo "=============================================="

# Trap Ctrl+C to clean up
cleanup() {
    echo ""
    echo "--- Stopping collectors ---"
    for pid in "${PERF_PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    kill "$POWER_PID" 2>/dev/null
    kill "$DOCKER_PID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup SIGINT SIGTERM

# Wait for all collectors. Done in the main shell (a subshell cannot wait on
# this shell's children) and with `set +e` so a non-zero collector exit (e.g.
# perf event unavailable) does not abort the script.
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
set -e

# =============================================================================
# Save container process trees and logs
# =============================================================================
echo ""
echo "--- Saving experiment logs ---"
FIRST_CONTAINER="${ACTIVE_CONTAINERS[0]}"
if docker inspect experiment_runner &>/dev/null; then
    docker logs experiment_runner > "$OUTPUT_DIR/patrol_log.txt" 2>&1
elif docker inspect "$FIRST_CONTAINER" &>/dev/null; then
    docker logs "$FIRST_CONTAINER" > "$OUTPUT_DIR/patrol_log.txt" 2>&1
else
    echo "No experiment_runner or $FIRST_CONTAINER container found" > "$OUTPUT_DIR/patrol_log.txt"
fi

# =============================================================================
# Validate collected outputs
# =============================================================================
echo ""
echo "--- Validating collectors ---"
for i in "${!ACTIVE_CONTAINERS[@]}"; do
    cname="${ACTIVE_CONTAINERS[$i]}"
    [ "${PERF_STATUSES[$i]}" = 0 ] \
        && echo "perf $cname: OK" \
        || echo "WARNING: perf $cname exited with status ${PERF_STATUSES[$i]}"
done
[ "$POWER_STATUS" = 0 ] \
    && echo "power (RAPL): OK" \
    || echo "WARNING: power (RAPL) exited with status $POWER_STATUS"
[ "$DOCKER_STATUS" = 0 ] \
    && echo "docker stats: OK" \
    || echo "WARNING: docker stats exited with status $DOCKER_STATUS"

for f in "$OUTPUT_DIR"/perf_*.txt "$OUTPUT_DIR"/power.txt "$OUTPUT_DIR"/docker_stats.csv; do
    [ -f "$f" ] || continue
    [ -s "$f" ] || echo "WARNING: $f is empty (collector produced no output)"
done
for f in "$OUTPUT_DIR"/*.err; do
    [ -s "$f" ] && echo "WARNING: errors in $f:" && cat "$f"
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
echo " Derived metrics to calculate:"
echo "   IPC = instructions / cycles"
echo "   MPKI = cache-misses / (instructions / 1000)"
echo "   LLC hit rate = 1 - (LLC.miss / LLC.reference)"
echo "   Power/goal = total_energy / goals_completed"
echo "=============================================="