#!/bin/bash
# collect_metrics.sh — Run on HOST 2 during experiments
# Usage: ./collect_metrics.sh <experiment_name> <duration_secs>

EXPERIMENT=${1:-"baseline"}
DURATION=${2:-300}
OUTPUT_DIR="results/${EXPERIMENT}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "=== Collecting metrics for experiment: $EXPERIMENT ==="
echo "=== Duration: ${DURATION}s ==="
echo "=== Output: $OUTPUT_DIR ==="

# Get container PIDs
PID_CTRL1=$(docker inspect panther_controller --format '{{.State.Pid}}')
PID_CTRL2=$(docker inspect panther2_controller --format '{{.State.Pid}}')
echo "Controller 1 PID: $PID_CTRL1"
echo "Controller 2 PID: $PID_CTRL2"

# 1. pqos — LLC and memory bandwidth (all cores)
sudo pqos -m all:0-15 -t $DURATION -o "$OUTPUT_DIR/pqos.csv" &
PQOS_PID=$!

# 2. perf stat — IPC, cache, context switches, power (controller 1)
sudo perf stat -p $PID_CTRL1 \
  -e cycles,instructions,cache-misses,cache-references,context-switches,cpu-migrations,page-faults,power/energy-pkg/,power/energy-ram/ \
  -I 1000 -o "$OUTPUT_DIR/perf_ctrl1.csv" \
  sleep $DURATION &
PERF1_PID=$!

# 3. perf stat — IPC, cache, context switches, power (controller 2)
sudo perf stat -p $PID_CTRL2 \
  -e cycles,instructions,cache-misses,cache-references,context-switches,cpu-migrations,page-faults \
  -I 1000 -o "$OUTPUT_DIR/perf_ctrl2.csv" \
  sleep $DURATION &
PERF2_PID=$!

# 4. docker stats — CPU and memory per container (1 Hz)
for i in $(seq 1 $DURATION); do
  docker stats panther_controller panther2_controller --no-stream \
    --format "{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.NetIO}},{{.BlockIO}}" \
    >> "$OUTPUT_DIR/docker_stats.csv"
  sleep 1
done &
DOCKER_PID=$!

# Wait for all to finish
echo "Collection running... Ctrl+C to stop early"
wait $PQOS_PID $PERF1_PID $PERF2_PID $DOCKER_PID

echo "=== Collection complete ==="
echo "Results in: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR"
