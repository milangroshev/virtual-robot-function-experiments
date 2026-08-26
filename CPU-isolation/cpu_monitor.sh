#!/bin/bash
# Quick CPU monitor for experiment cores
# Usage: ./cpu_monitor.sh [interval_seconds] [cpus]
# Example: ./cpu_monitor.sh 2 "0,1"

INTERVAL=${1:-2}
CPUS=${2:-"0,1"}

echo "Monitoring CPUs: $CPUS every ${INTERVAL}s"
echo "Press Ctrl+C to stop"
echo ""

# Count threads in pool
NUM_THREADS=$(echo "$CPUS" | tr ',' '\n' | wc -l)

while true; do
    TIMESTAMP=$(date +%H:%M:%S)
    
    # Get per-CPU usage from /proc/stat
    echo "=== $TIMESTAMP === (Pool: $NUM_THREADS threads)"
    
    # Per-CPU % from top-style sampling (1 second)
    TOTAL=0
    for cpu in $(echo "$CPUS" | tr ',' ' '); do
        # Get processes on this CPU and their CPU%
        CPU_SUM=$(ps -eo psr,pcpu --no-headers | awk -v c="$cpu" '$1 == c {sum += $2} END {printf "%.1f", sum}')
        printf "  CPU %2d: %5s%%  |" "$cpu" "$CPU_SUM"
        
        # Top 3 processes on this CPU
        TOP=$(ps -eo psr,pcpu,comm --no-headers --sort=-pcpu | awk -v c="$cpu" '$1 == c && $2 > 0.5 {printf " %s(%.0f%%)", $3, $2}' | head -c 60)
        echo " $TOP"
        
        TOTAL=$(echo "$TOTAL + $CPU_SUM" | bc)
    done
    
    AVG=$(echo "scale=1; $TOTAL / $NUM_THREADS" | bc)
    
    # Count containers on this cpuset
    CONTAINERS=$(docker ps --format '{{.Names}}' 2>/dev/null | wc -l)
    
    echo "  ─────────────────────────────"
    printf "  Total: %.1f%%  Avg/thread: %s%%  Containers: %d\n" "$TOTAL" "$AVG" "$CONTAINERS"
    echo ""
    
    sleep "$INTERVAL"
done