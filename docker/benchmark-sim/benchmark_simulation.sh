#!/usr/bin/env bash
# =============================================================================
# benchmark_simulation.sh
# =============================================================================
# Measures Gazebo simulation performance for a given docker-compose config.
# Benchmarks ALL topics that the Nav2 stack depends on from Gazebo.
#
# Run this AFTER docker compose up has fully stabilized (all healthchecks green).
#
# Usage:
#   chmod +x benchmark_simulation.sh
#   ./benchmark_simulation.sh <num_robots> [duration_seconds]
#
# Examples:
#   ./benchmark_simulation.sh 1          # benchmark 1-robot setup, 30s default
#   ./benchmark_simulation.sh 4 60       # benchmark 4-robot setup, 60s sample
#
# Output:
#   results/benchmark_<num_robots>_robots_<timestamp>.csv
#   results/benchmark_<num_robots>_robots_<timestamp>.log       (full log)
#   results/benchmark_<num_robots>_robots_<timestamp>_summary.txt
# =============================================================================

set -euo pipefail

NUM_ROBOTS="${1:?Usage: $0 <num_robots> [duration_seconds]}"
DURATION="${2:-30}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR="./results"
LOG_FILE="${RESULTS_DIR}/benchmark_${NUM_ROBOTS}_robots_${TIMESTAMP}.log"
CSV_FILE="${RESULTS_DIR}/benchmark_${NUM_ROBOTS}_robots_${TIMESTAMP}.csv"
SUMMARY_FILE="${RESULTS_DIR}/benchmark_${NUM_ROBOTS}_robots_${TIMESTAMP}_summary.txt"

mkdir -p "$RESULTS_DIR"

# Robot namespaces based on count
declare -a ROBOTS
case "$NUM_ROBOTS" in
  1) ROBOTS=("panther") ;;
  2) ROBOTS=("panther" "panther2") ;;
  3) ROBOTS=("panther" "panther2" "panther3") ;;
  4) ROBOTS=("panther" "panther2" "panther3" "panther4") ;;
  *) echo "ERROR: num_robots must be 1-4"; exit 1 ;;
esac

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG_FILE"; }

# =============================================================================
# 0. PRE-FLIGHT: Verify all containers are healthy
# =============================================================================
log "=========================================="
log "BENCHMARK: ${NUM_ROBOTS} robot(s), ${DURATION}s sample"
log "=========================================="

log "--- Pre-flight: checking container health ---"
GAZEBO_CONTAINER="gazebo"

# Check gazebo container
if ! docker inspect --format='{{.State.Health.Status}}' "$GAZEBO_CONTAINER" 2>/dev/null | grep -q "healthy"; then
  log "WARNING: gazebo container is not healthy. Waiting 30s..."
  sleep 30
fi

for ns in "${ROBOTS[@]}"; do
  if [ "$ns" == "panther" ]; then
    container="gazebo"
  else
    idx="${ns#panther}"
    container="gz_robot_${idx}"
  fi
  status=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "not_found")
  log "  $container: $status"
done

# Also check sensor_preproc containers
for ns in "${ROBOTS[@]}"; do
  if [ "$ns" == "panther" ]; then
    container="${ROBOT_NAMESPACE:-panther}_sensor_preproc"
  else
    container="${ns}_sensor_preproc"
  fi
  status=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "not_found")
  log "  $container: $status"
done

log ""

# =============================================================================
# 1. VERIFY OVERRIDES: Check that optimized files are in place
# =============================================================================
log "--- Verifying Gazebo overrides ---"

UPDATE_RATE=$(docker exec "$GAZEBO_CONTAINER" bash -c \
  "grep 'update_rate' /ros2_ws/install/husarion_components_description/share/husarion_components_description/urdf/velodyne_puck.urdf.xacro | head -1 | tr -dc '0-9.'")
H_SAMPLES=$(docker exec "$GAZEBO_CONTAINER" bash -c \
  "grep -A1 '<horizontal>' /ros2_ws/install/husarion_components_description/share/husarion_components_description/urdf/velodyne_puck.urdf.xacro | grep samples | tr -dc '0-9'")
V_SAMPLES=$(docker exec "$GAZEBO_CONTAINER" bash -c \
  "grep -A1 '<vertical>' /ros2_ws/install/husarion_components_description/share/husarion_components_description/urdf/velodyne_puck.urdf.xacro | grep samples | tr -dc '0-9'")
MAX_RANGE=$(docker exec "$GAZEBO_CONTAINER" bash -c \
  "grep '<max>' /ros2_ws/install/husarion_components_description/share/husarion_components_description/urdf/velodyne_puck.urdf.xacro | head -1 | tr -dc '0-9.'")
STEP_SIZE=$(docker exec "$GAZEBO_CONTAINER" bash -c \
  "grep 'max_step_size' /ros2_ws/install/husarion_gz_worlds/share/husarion_gz_worlds/worlds/husarion_world.sdf | tr -dc '0-9.'")

log "  LiDAR update_rate: ${UPDATE_RATE} Hz (expected: 5.0)"
log "  Horizontal samples: ${H_SAMPLES} (expected: 450)"
log "  Vertical samples:   ${V_SAMPLES} (expected: 8)"
log "  Max range:           ${MAX_RANGE} m (expected: 30.0)"
log "  Physics step size:   ${STEP_SIZE} s (expected: 0.004)"

OVERRIDES_OK=true
[[ "$UPDATE_RATE" != "5.0" ]] && log "  ⚠️  update_rate mismatch!" && OVERRIDES_OK=false
[[ "$H_SAMPLES" != "450" ]] && log "  ⚠️  horizontal samples mismatch!" && OVERRIDES_OK=false
[[ "$V_SAMPLES" != "8" ]] && log "  ⚠️  vertical samples mismatch!" && OVERRIDES_OK=false
[[ "$STEP_SIZE" != "0.004" ]] && log "  ⚠️  physics step size mismatch!" && OVERRIDES_OK=false

if $OVERRIDES_OK; then
  log "  ✅ All overrides verified"
else
  log "  ❌ Some overrides are NOT applied — results may not reflect optimizations"
fi
log ""

# =============================================================================
# 2. MEASURE ALL NAV2-DEPENDENT TOPIC RATES
# =============================================================================
# Topics benchmarked (per robot):
#   FROM GAZEBO (physics/sensors):
#     /<ns>/velodyne_points          — raw 3D LiDAR (gpu_lidar sensor)
#     /<ns>/odometry/filtered        — EKF fused odometry
#     /<ns>/imu/data                 — IMU sensor
#   FROM SENSOR_PREPROC (depends on Gazebo input):
#     /<ns>/velodyne_points_filtered — cropped pointcloud → STVL costmaps
#     /<ns>/scan                     — pointcloud_to_laserscan → AMCL
#   GLOBAL (shared):
#     /clock                         — simulation clock
#     /tf                            — transforms
# =============================================================================

log "--- Measuring topic rates (${DURATION}s per topic) ---"

# CSV header
echo "num_robots,robot,topic,clock_mode,avg_rate_hz,min_interval_s,max_interval_s,std_dev_s,window" > "$CSV_FILE"

measure_topic_rate() {
  local container="$1"
  local ns="$2"
  local topic="$3"
  local use_sim_time="$4"
  local label="$5"

  local sim_flag=""
  [[ "$use_sim_time" == "true" ]] && sim_flag="--use-sim-time"

  log "  [${label}] ${topic} ..."

  # Run ros2 topic hz inside the container, capture output
  local raw_output
  raw_output=$(docker exec "$container" bash -c \
    "timeout ${DURATION} /ros_entrypoint.sh ros2 topic hz ${topic} ${sim_flag} 2>&1 || true" 2>&1)

  # Get the last "average rate" line (most stable reading)
  local last_avg_line
  last_avg_line=$(echo "$raw_output" | grep "average rate" | tail -1)

  if [ -z "$last_avg_line" ]; then
    log "    ⚠️  No data received for ${topic}"
    echo "${NUM_ROBOTS},${ns},${topic},${label},0,0,0,0,0" >> "$CSV_FILE"
    return
  fi

  # Parse: "average rate: 3.548"
  local avg_rate
  avg_rate=$(echo "$last_avg_line" | grep -oP 'average rate: \K[0-9.]+')

  # Get the detail line that follows the last average rate line
  local detail
  detail=$(echo "$raw_output" | grep -A1 "average rate" | tail -1)

  local min_int max_int std_dev window
  min_int=$(echo "$detail" | grep -oP 'min: \K[0-9.]+' || echo "N/A")
  max_int=$(echo "$detail" | grep -oP 'max: \K[0-9.]+' || echo "N/A")
  std_dev=$(echo "$detail" | grep -oP 'std dev: \K[0-9.]+' || echo "N/A")
  window=$(echo "$detail" | grep -oP 'window: \K[0-9]+' || echo "N/A")

  log "    avg=${avg_rate} Hz | min=${min_int}s max=${max_int}s std=${std_dev}s (n=${window})"
  echo "${NUM_ROBOTS},${ns},${topic},${label},${avg_rate},${min_int},${max_int},${std_dev},${window}" >> "$CSV_FILE"
}

# ---- /clock (global, measure once from gazebo container) ----
log ""
log "=== GLOBAL TOPICS ==="
measure_topic_rate "$GAZEBO_CONTAINER" "global" "/clock" "" "wall_clock"

# ---- Per-robot topics ----
for ns in "${ROBOTS[@]}"; do
  log ""
  log "=== ROBOT: ${ns} ==="

  # Determine which container to exec into for Gazebo-side topics
  if [ "$ns" == "panther" ]; then
    gz_container="gazebo"
  else
    idx="${ns#panther}"
    gz_container="gz_robot_${idx}"
  fi

  # Determine sensor_preproc container name
  if [ "$ns" == "panther" ]; then
    preproc_container="${ROBOT_NAMESPACE:-panther}_sensor_preproc"
  else
    preproc_container="${ns}_sensor_preproc"
  fi

  # --- GAZEBO-ORIGINATED TOPICS (measured from gazebo container) ---

  # 1. Raw Velodyne pointcloud — wall-clock (reveals RTF)
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/${ns}/velodyne_points" "" "wall_clock"

  # 2. Raw Velodyne pointcloud — sim-time (should be exactly 5.0 Hz)
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/${ns}/velodyne_points" "true" "sim_time"

  # 3. Odometry — wall-clock (controller_server, bt_navigator depend on this)
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/${ns}/odometry/filtered" "" "wall_clock"

  # 4. Odometry — sim-time (should be ~50 Hz or whatever EKF publishes)
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/${ns}/odometry/filtered" "true" "sim_time"

  # 5. IMU — wall-clock (feeds EKF)
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/${ns}/imu/data" "" "wall_clock"

  # 6. IMU — sim-time
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/${ns}/imu/data" "true" "sim_time"

  # 7. TF — wall-clock (all Nav2 nodes need transforms)
  measure_topic_rate "$GAZEBO_CONTAINER" "$ns" "/tf" "" "wall_clock"

  # --- SENSOR_PREPROC TOPICS (measured from preproc container) ---

  # 8. Filtered pointcloud — wall-clock (STVL local+global costmap input)
  measure_topic_rate "$preproc_container" "$ns" "/${ns}/velodyne_points_filtered" "" "wall_clock"

  # 9. Filtered pointcloud — sim-time (should match raw rate: 5.0 Hz)
  measure_topic_rate "$preproc_container" "$ns" "/${ns}/velodyne_points_filtered" "true" "sim_time"

  # 10. Laser scan — wall-clock (AMCL localization input)
  measure_topic_rate "$preproc_container" "$ns" "/${ns}/scan" "" "wall_clock"

  # 11. Laser scan — sim-time (should match raw rate: 5.0 Hz)
  measure_topic_rate "$preproc_container" "$ns" "/${ns}/scan" "true" "sim_time"
done

log ""

# =============================================================================
# 3. MEASURE POINT CLOUD SIZE (verify sample reduction)
# =============================================================================
log "--- Measuring point cloud size ---"

for ns in "${ROBOTS[@]}"; do
  pc_info=$(docker exec "$GAZEBO_CONTAINER" bash -c \
    "timeout 10 /ros_entrypoint.sh ros2 topic echo /${ns}/velodyne_points \
    --field height --field width --field point_step --field row_step \
    --once --use-sim-time 2>/dev/null || echo 'TIMEOUT'" 2>&1 | head -8)

  width=$(echo "$pc_info" | grep -oP 'width: \K[0-9]+' || echo "N/A")
  height=$(echo "$pc_info" | grep -oP 'height: \K[0-9]+' || echo "N/A")
  point_step=$(echo "$pc_info" | grep -oP 'point_step: \K[0-9]+' || echo "N/A")

  if [ "$width" != "N/A" ] && [ "$height" != "N/A" ]; then
    total_points=$((width * height))
    expected_points=$((450 * 8))
    if [ "$total_points" -eq "$expected_points" ]; then
      log "  ${ns}: ${total_points} points (${width}×${height}) ✅ matches 450×8"
    else
      log "  ${ns}: ${total_points} points (${width}×${height}) ⚠️  expected ${expected_points}"
    fi
  else
    log "  ${ns}: Could not read pointcloud fields"
  fi
  echo "${NUM_ROBOTS},${ns},pointcloud_width,,${width},,,," >> "$CSV_FILE"
  echo "${NUM_ROBOTS},${ns},pointcloud_height,,${height},,,," >> "$CSV_FILE"
done

log ""

# =============================================================================
# 4. DOCKER RESOURCE USAGE SNAPSHOT
# =============================================================================
log "--- Docker container resource usage ---"

docker stats --no-stream --format \
  "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.PIDs}}" \
  2>/dev/null | tee -a "$LOG_FILE"

# Also save as CSV-friendly format
echo "" >> "$CSV_FILE"
echo "# --- Docker Stats ---" >> "$CSV_FILE"
echo "container,cpu_percent,mem_usage,pids" >> "$CSV_FILE"
docker stats --no-stream --format "{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.PIDs}}" \
  2>/dev/null >> "$CSV_FILE"

log ""

# =============================================================================
# 5. GPU UTILIZATION
# =============================================================================
log "--- GPU utilization (nvidia-smi) ---"

nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu \
  --format=csv 2>/dev/null | tee -a "$LOG_FILE"

# Save to CSV
echo "" >> "$CSV_FILE"
echo "# --- GPU Stats ---" >> "$CSV_FILE"
nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu \
  --format=csv 2>/dev/null >> "$CSV_FILE"

log ""

# =============================================================================
# 6. SUMMARY & PASS/FAIL VERDICT
# =============================================================================
log "--- SUMMARY ---"

{
  echo "============================================"
  echo "BENCHMARK SUMMARY: ${NUM_ROBOTS} Robot(s)"
  echo "Date: $(date)"
  echo "Duration per measurement: ${DURATION}s"
  echo "============================================"
  echo ""

  # Parse CSV for sim-time rates
  echo "SIM-TIME RATES (must be constant across all configs):"
  echo "-----------------------------------------------------"
  ALL_SIM_OK=true

  while IFS=',' read -r nrobots robot topic clock_mode avg_rate rest; do
    [[ "$clock_mode" != "sim_time" ]] && continue
    [[ "$avg_rate" == "0" ]] && continue
    [[ "$avg_rate" == "avg_rate_hz" ]] && continue

    # Determine expected rate based on topic
    expected=""
    tolerance=""
    case "$topic" in
      */velodyne_points|*/velodyne_points_filtered|*/scan)
        expected="5.0"
        tolerance="0.5"
        ;;
      */odometry/filtered)
        expected=""  # varies by EKF config, just report
        ;;
      */imu/data)
        expected=""  # varies by sensor config, just report
        ;;
    esac

    if [ -n "$expected" ] && [ -n "$avg_rate" ]; then
      diff=$(echo "$avg_rate $expected" | awk '{d=$1-$2; if(d<0) d=-d; print d}')
      pass=$(echo "$diff $tolerance" | awk '{print ($1 <= $2) ? "PASS" : "FAIL"}')
      echo "  ${robot} ${topic}: ${avg_rate} Hz (expected ~${expected}) [${pass}]"
      [[ "$pass" == "FAIL" ]] && ALL_SIM_OK=false
    else
      echo "  ${robot} ${topic}: ${avg_rate} Hz (informational)"
    fi
  done < "$CSV_FILE"

  echo ""
  echo "WALL-CLOCK RATES & RTF:"
  echo "-----------------------"

  while IFS=',' read -r nrobots robot topic clock_mode avg_rate rest; do
    [[ "$clock_mode" != "wall_clock" ]] && continue
    [[ "$avg_rate" == "0" ]] && continue
    [[ "$avg_rate" == "avg_rate_hz" ]] && continue

    case "$topic" in
      */velodyne_points)
        rtf=$(echo "$avg_rate" | awk '{printf "%.2f", $1 / 5.0}')
        echo "  ${robot} ${topic}: ${avg_rate} Hz wall-clock → RTF ≈ ${rtf}"
        ;;
      */velodyne_points_filtered)
        rtf=$(echo "$avg_rate" | awk '{printf "%.2f", $1 / 5.0}')
        echo "  ${robot} ${topic}: ${avg_rate} Hz wall-clock → RTF ≈ ${rtf}"
        ;;
      */scan)
        echo "  ${robot} ${topic}: ${avg_rate} Hz wall-clock"
        ;;
      */odometry/filtered)
        echo "  ${robot} ${topic}: ${avg_rate} Hz wall-clock"
        ;;
      */imu/data)
        echo "  ${robot} ${topic}: ${avg_rate} Hz wall-clock"
        ;;
      /clock)
        echo "  ${topic}: ${avg_rate} Hz wall-clock"
        ;;
      /tf)
        echo "  ${robot} ${topic}: ${avg_rate} Hz wall-clock"
        ;;
    esac
  done < "$CSV_FILE"

  echo ""
  echo "VERDICT:"
  echo "--------"
  if $ALL_SIM_OK; then
    echo "  ✅ ALL SIM-TIME RATES ARE WITHIN TOLERANCE"
    echo "  → Simulation is reliable for controller benchmarking."
    echo "  → Controllers using use_sim_time will experience identical"
    echo "    physics and sensor rates regardless of fleet size."
    echo "  → Only wall-clock experiment duration varies."
  else
    echo "  ❌ SOME SIM-TIME RATES ARE OUT OF TOLERANCE"
    echo "  → Simulation may be dropping sensor messages."
    echo "  → Controller benchmarks may be affected by sim performance."
    echo "  → Consider further reducing sensor load or physics step size."
  fi

  echo ""
  echo "FILES:"
  echo "  CSV:     ${CSV_FILE}"
  echo "  Log:     ${LOG_FILE}"
  echo "  Summary: ${SUMMARY_FILE}"
  echo "============================================"
} | tee "$SUMMARY_FILE" | tee -a "$LOG_FILE"

log ""
log "Benchmark complete. Results in ${RESULTS_DIR}/"
