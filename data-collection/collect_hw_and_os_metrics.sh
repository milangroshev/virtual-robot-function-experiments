#!/bin/bash
# =============================================================================
# collect_metrics.sh — HW, OS, and Container resource profiling
# =============================================================================
# Collects hardware counters (perf stat), power (RAPL), and container
# metrics (docker stats) during Nav2 profiling experiments.
#
# Usage: ./collect_metrics.sh <experiment_name> <duration_secs> [container1] [container2]
#
# Examples:
#   ./collect_metrics.sh controller_baseline 300
#   ./collect_metrics.sh controller_2robots 300 panther_controller panther2_controller
#   ./collect_metrics.sh planner_test 600 panther_planner panther2_planner
#   ./collect_metrics.sh single_ctrl 300 panther_controller
# =============================================================================

set -e

EXPERIMENT=${1:-"baseline"}
DURATION=${2:-300}
CONTAINER1=${3:-"panther_controller"}
CONTAINER2=${4:-"panther2_controller"}
OUTPUT_DIR="results/${EXPERIMENT}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "=============================================="
echo " Resource Profiling: $EXPERIMENT"
echo "=============================================="
echo " Duration:    ${DURATION}s"
echo " Container 1: $CONTAINER1"
echo " Container 2: $CONTAINER2"
echo " Output:      $OUTPUT_DIR"
echo "=============================================="

# =============================================================================
# get_worker_pids <container_name>
#   Returns comma-separated PIDs of actual workload processes inside a
#   container, skipping shell scripts and parent stress-ng processes.
#   Looks for processes whose CMD contains "[run]" (stress-ng workers),
#   or falls back to all non-bash, non-sleep leaf processes.
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

    # Strategy 1: find stress-ng worker processes (CMD contains "[run]")
    local pids
    pids=$(docker top "$cname" -eo pid,cmd 2>/dev/null \
        | grep '\[run\]' \
        | awk '{print $1}' \
        | tr '\n' ',' | sed 's/,$//')

    if [ -n "$pids" ]; then
        echo "$pids"
        return
    fi

    # Strategy 2: find any non-shell, non-sleep leaf processes
    # (excludes bash, sh, sleep, and the parent stress-ng launchers)
    pids=$(docker top "$cname" -eo pid,ppid,cmd 2>/dev/null \
        | tail -n +2 \
        | grep -v -E '(bash|/bin/sh|sleep)' \
        | awk -v init="$init_pid" '$2 != "PID" {print $1}' \
        | tr '\n' ',' | sed 's/,$//')

    if [ -n "$pids" ]; then
        echo "$pids"
        return
    fi

    # Strategy 3: last resort — use the container init PID
    echo "$init_pid"
}

# --- Resolve worker PIDs ---
PIDS1=$(get_worker_pids "$CONTAINER1")

if [ -z "$PIDS1" ]; then
    echo "ERROR: Container $CONTAINER1 not running or has no worker processes!"
    exit 1
fi

PIDS2=$(get_worker_pids "$CONTAINER2")

if [ -z "$PIDS2" ]; then
    echo "WARNING: Container $CONTAINER2 not running — single container mode"
    SINGLE_CONTAINER=true
else
    SINGLE_CONTAINER=false
fi

echo ""
echo "Worker PIDs for $CONTAINER1: $PIDS1"
[ "$SINGLE_CONTAINER" = false ] && echo "Worker PIDs for $CONTAINER2: $PIDS2"

# Show what processes we're actually tracking
echo ""
echo "--- Processes being profiled ---"
echo "[$CONTAINER1]"
docker top "$CONTAINER1" -eo pid,ppid,cmd 2>/dev/null | head -20
if [ "$SINGLE_CONTAINER" = false ]; then
    echo ""
    echo "[$CONTAINER2]"
    docker top "$CONTAINER2" -eo pid,ppid,cmd 2>/dev/null | head -20
fi
echo ""

# --- Save experiment metadata ---
cat > "$OUTPUT_DIR/experiment_info.txt" <<EOF
Experiment: $EXPERIMENT
Date: $(date)
Duration: ${DURATION}s
Container 1: $CONTAINER1 (Worker PIDs: $PIDS1)
Container 2: $CONTAINER2 (Worker PIDs: ${PIDS2:-N/A})
CPU: $(lscpu | grep "Model name" | sed 's/Model name:\s*//')
Cores: $(nproc)
Kernel: $(uname -r)
Docker: $(docker version --format '{{.Server.Version}}' 2>/dev/null || echo "unknown")
EOF

echo ""
echo "--- Starting data collection ---"
echo ""

# =============================================================================
# 1. PERF STAT — Container 1: IPC, cache, context switches
# =============================================================================
echo "[1/4] perf stat: $CONTAINER1 (PIDs: $PIDS1)"
sudo perf stat -p "$PIDS1" \
    -e cycles,instructions,cache-misses,cache-references,longest_lat_cache.miss,longest_lat_cache.reference,context-switches,cpu-migrations,page-faults \
    -I 1000 \
    -o "$OUTPUT_DIR/perf_${CONTAINER1}.txt" \
    sleep "$DURATION" &
PERF1_PID=$!

# =============================================================================
# 2. PERF STAT — Container 2: IPC, cache, context switches
# =============================================================================
if [ "$SINGLE_CONTAINER" = false ]; then
    echo "[2/4] perf stat: $CONTAINER2 (PIDs: $PIDS2)"
    sudo perf stat -p "$PIDS2" \
        -e cycles,instructions,cache-misses,cache-references,longest_lat_cache.miss,longest_lat_cache.reference,context-switches,cpu-migrations,page-faults \
        -I 1000 \
        -o "$OUTPUT_DIR/perf_${CONTAINER2}.txt" \
        sleep "$DURATION" &
    PERF2_PID=$!
else
    echo "[2/4] Skipping — single container mode"
    PERF2_PID=""
fi

# =============================================================================
# 3. PERF STAT — System-wide power (RAPL)
# =============================================================================
echo "[3/4] perf stat: System power (RAPL)"
sudo perf stat -a \
    -e power/energy-pkg/,power/energy-cores/,power/energy-gpu/ \
    -I 1000 \
    -o "$OUTPUT_DIR/power.txt" \
    sleep "$DURATION" &
POWER_PID=$!

# =============================================================================
# 4. DOCKER STATS — CPU%, Memory, Network, Block I/O (1 Hz)
# =============================================================================
echo "[4/4] docker stats: CPU%, memory, network (1 Hz)"
echo "timestamp,name,cpu_perc,mem_usage,net_io,block_io,pids" \
    > "$OUTPUT_DIR/docker_stats.csv"

CONTAINERS_LIST="$CONTAINER1"
[ "$SINGLE_CONTAINER" = false ] && CONTAINERS_LIST="$CONTAINER1 $CONTAINER2"

for i in $(seq 1 "$DURATION"); do
    TIMESTAMP=$(date +%s.%N)
    docker stats $CONTAINERS_LIST --no-stream \
        --format "${TIMESTAMP},{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.NetIO}},{{.BlockIO}},{{.PIDs}}" \
        >> "$OUTPUT_DIR/docker_stats.csv" 2>/dev/null
    sleep 1
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
    kill $PERF1_PID 2>/dev/null
    [ -n "$PERF2_PID" ] && kill $PERF2_PID 2>/dev/null
    kill $POWER_PID 2>/dev/null
    kill $DOCKER_PID 2>/dev/null
    wait 2>/dev/null
}
trap cleanup SIGINT SIGTERM

wait $PERF1_PID 2>/dev/null
[ -n "$PERF2_PID" ] && wait $PERF2_PID 2>/dev/null
wait $POWER_PID 2>/dev/null
wait $DOCKER_PID 2>/dev/null

# =============================================================================
# Save container process trees and logs
# =============================================================================
echo ""
echo "--- Saving experiment logs ---"
docker logs experiment_runner > "$OUTPUT_DIR/patrol_log.txt" 2>&1 || \
    echo "No experiment_runner container found" > "$OUTPUT_DIR/patrol_log.txt"

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
