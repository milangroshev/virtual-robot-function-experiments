# robot_workload.sh — simulates a real-time robotic control loop
#!/bin/bash
# Spin a CPU-intensive workload that mimics robot control:
# matrix ops (inverse kinematics), memory access patterns (SLAM)
stress-ng --matrix 1 --matrix-size 64 --timeout 0 &
stress-ng --cache 1 --cache-level 2 --timeout 0 &
wait