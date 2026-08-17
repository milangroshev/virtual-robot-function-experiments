#!/usr/bin/env python3
"""
Patrol navigation script for resource profiling experiments.

Navigates a single robot in a loop through 5 waypoints:
  MIDDLE → TOP_RIGHT → BOTTOM_RIGHT → BOTTOM_LEFT → TOP_LEFT → MIDDLE

Usage:
  python3 patrol_single_robot.py <namespace> [num_loops] [warmup_secs]

  namespace   : Robot namespace (e.g., 'panther', 'panther2')
  num_loops   : Number of patrol loops (0 = infinite, default: 0)
  warmup_secs : Seconds to wait after action server is found (default: 5)

The script:
  1. Waits for /clock (sim time) to start flowing
  2. Waits for the navigate_to_pose action server
  3. Disables the e-stop (hardware/e_stop_reset service)
  4. Pauses for warmup period (for profiling tools to attach)
  5. Navigates through waypoints in a continuous loop
  6. Retries failed goals once before moving on
  7. Logs timing data for each goal (useful for QoS analysis)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
import time
import sys
import signal

# =============================================================================
# WAYPOINTS (map frame coordinates from AMCL / 2D Pose Estimate in RViz)
# =============================================================================
WAYPOINTS = {
    "MIDDLE": {
        "x": 0.0, "y": 0.0,
        "qz": 0.0, "qw": 1.0,
    },
    "TOP_RIGHT": {
        "x": 11.464, "y": -9.523,
        "qz": 0.758, "qw": 0.653,
    },
    "BOTTOM_RIGHT": {
        "x": -11.056, "y": -8.540,
        "qz": -0.160, "qw": 0.987,
    },
    "BOTTOM_LEFT": {
        "x": -10.709, "y": 13.601,
        "qz": -0.735, "qw": 0.678,
    },
    "TOP_LEFT": {
        "x": 11.823, "y": 13.204,
        "qz": 1.000, "qw": 0.025,
    },
}

# Navigation order: start at MIDDLE, visit all corners, return to MIDDLE
PATROL_ORDER = [
    "TOP_RIGHT",
    "BOTTOM_RIGHT",
    "BOTTOM_LEFT",
    "TOP_LEFT",
    "MIDDLE",
]

# Action result status codes
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


class PatrolNode(Node):
    """ROS 2 node that patrols through waypoints using Nav2."""

    def __init__(self, namespace):
        super().__init__(
            'patrol_node',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        self.namespace = namespace
        self.shutdown_requested = False

        # Build action server name with namespace
        action_name = f'/{namespace}/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_name)

        # E-stop reset service client
        estop_service_name = f'/{namespace}/hardware/e_stop_reset'
        self._estop_client = self.create_client(Trigger, estop_service_name)

        self.get_logger().info(f'Patrol node created')
        self.get_logger().info(f'  Namespace:     {namespace}')
        self.get_logger().info(f'  Action server: {action_name}')
        self.get_logger().info(f'  E-stop service:{estop_service_name}')
        self.get_logger().info(f'  Waypoints:     {len(PATROL_ORDER)} per loop')
        self.get_logger().info(f'  use_sim_time:  True')

    def wait_for_clock(self, timeout_sec=120.0):
        """Wait until /clock is publishing (sim time is flowing)."""
        self.get_logger().info('Waiting for /clock (sim time)...')
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.shutdown_requested:
                return False
            rclpy.spin_once(self, timeout_sec=0.5)
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                self.get_logger().info(
                    f'Clock active: sim time = {now.nanoseconds / 1e9:.1f}s'
                )
                return True
        self.get_logger().error(f'No clock after {timeout_sec}s')
        return False

    def wait_for_server(self, timeout_sec=300.0):
        """Wait for the navigate_to_pose action server."""
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.shutdown_requested:
                return False
            if self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info('Action server available!')
                return True
            self.get_logger().info(
                f'Still waiting... ({time.time() - start:.0f}s elapsed)'
            )
        self.get_logger().error(
            f'Action server not available after {timeout_sec}s'
        )
        return False

    def disable_estop(self, max_retries=5):
        """Disable the e-stop by calling the reset service."""
        self.get_logger().info('Disabling e-stop...')

        for attempt in range(max_retries):
            if self.shutdown_requested:
                return False

            if not self._estop_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(
                    f'E-stop service not available yet '
                    f'(attempt {attempt + 1}/{max_retries})'
                )
                continue

            # Call the service
            request = Trigger.Request()
            future = self._estop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                result = future.result()
                if result.success:
                    self.get_logger().info(
                        f'✅ E-stop disabled: {result.message}'
                    )
                    return True
                else:
                    self.get_logger().warning(
                        f'E-stop reset returned failure: {result.message}'
                    )
                    # Still continue — some implementations return
                    # success=False when e-stop was already off
                    return True
            else:
                self.get_logger().warning(
                    f'E-stop service call timed out '
                    f'(attempt {attempt + 1}/{max_retries})'
                )

        self.get_logger().error(
            f'Failed to disable e-stop after {max_retries} attempts'
        )
        return False

    def send_goal(self, waypoint_name):
        """
        Send a navigation goal and wait for the result.
        Returns (success: bool, duration_sec: float).
        """
        wp = WAYPOINTS[waypoint_name]
        frame_id = f'{self.namespace}/map'

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp["x"]
        goal_msg.pose.pose.position.y = wp["y"]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = wp["qz"]
        goal_msg.pose.pose.orientation.w = wp["qw"]

        self.get_logger().info(
            f'→ Navigating to {waypoint_name} '
            f'({wp["x"]:.1f}, {wp["y"]:.1f})'
        )

        goal_start = time.time()

        # Send goal
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal to {waypoint_name} REJECTED')
            return False, 0.0

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        duration = time.time() - goal_start
        result = result_future.result()

        if result.status == STATUS_SUCCEEDED:
            self.get_logger().info(
                f'✅ Reached {waypoint_name} in {duration:.1f}s'
            )
            return True, duration
        elif result.status == STATUS_ABORTED:
            self.get_logger().warning(
                f'❌ ABORTED reaching {waypoint_name} after {duration:.1f}s'
            )
            return False, duration
        elif result.status == STATUS_CANCELED:
            self.get_logger().warning(
                f'⚠️  CANCELED goal to {waypoint_name} after {duration:.1f}s'
            )
            return False, duration
        else:
            self.get_logger().warning(
                f'❌ Failed to reach {waypoint_name} '
                f'(status={result.status}, {duration:.1f}s)'
            )
            return False, duration


def main():
    # Parse arguments
    namespace = sys.argv[1] if len(sys.argv) > 1 else 'panther'
    num_loops = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    warmup_secs = int(sys.argv[3]) if len(sys.argv) > 3 else 5

    rclpy.init()
    node = PatrolNode(namespace)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        node.get_logger().info('Shutdown requested...')
        node.shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # --- Startup sequence ---
    if not node.wait_for_clock():
        node.get_logger().error('Aborting: no clock')
        rclpy.shutdown()
        return

    if not node.wait_for_server():
        node.get_logger().error('Aborting: no action server')
        rclpy.shutdown()
        return

    # Disable e-stop
    if not node.disable_estop():
        node.get_logger().warning(
            'Could not disable e-stop — continuing anyway'
        )

    # Warmup pause (time to attach profiling tools)
    node.get_logger().info(
        f'System ready. Warmup pause: {warmup_secs}s '
        f'(attach profiling tools now!)'
    )
    time.sleep(warmup_secs)

    # --- Patrol loop ---
    loop_count = 0
    total_goals = 0
    total_successes = 0
    total_failures = 0

    loop_label = f'{num_loops} loops' if num_loops > 0 else 'infinite loops'
    node.get_logger().info(f'Starting patrol ({loop_label})')

    try:
        while not node.shutdown_requested:
            loop_count += 1
            if num_loops > 0 and loop_count > num_loops:
                break

            node.get_logger().info(
                f'═══ Loop {loop_count} '
                f'{"of " + str(num_loops) if num_loops > 0 else "(∞)"} ═══'
            )
            loop_start = time.time()

            for waypoint_name in PATROL_ORDER:
                if node.shutdown_requested:
                    break

                total_goals += 1
                success, duration = node.send_goal(waypoint_name)

                if success:
                    total_successes += 1
                else:
                    total_failures += 1
                    # Retry once on failure
                    node.get_logger().info(
                        f'Retrying {waypoint_name}...'
                    )
                    total_goals += 1
                    success, duration = node.send_goal(waypoint_name)
                    if success:
                        total_successes += 1
                    else:
                        total_failures += 1
                        node.get_logger().warning(
                            f'Skipping {waypoint_name} after retry failure'
                        )

            loop_duration = time.time() - loop_start
            success_rate = (
                total_successes / total_goals * 100
                if total_goals > 0 else 0
            )
            node.get_logger().info(
                f'Loop {loop_count} complete in {loop_duration:.1f}s | '
                f'Goals: {total_goals}, '
                f'Success: {total_successes} ({success_rate:.0f}%), '
                f'Failed: {total_failures}'
            )

    except Exception as e:
        node.get_logger().error(f'Exception: {e}')

    # --- Summary ---
    success_rate = (
        total_successes / total_goals * 100 if total_goals > 0 else 0
    )
    node.get_logger().info(f'')
    node.get_logger().info(f'══════════ PATROL SUMMARY ══════════')
    node.get_logger().info(f'  Loops completed: {loop_count}')
    node.get_logger().info(f'  Total goals:     {total_goals}')
    node.get_logger().info(f'  Successes:       {total_successes}')
    node.get_logger().info(f'  Failures:        {total_failures}')
    node.get_logger().info(f'  Success rate:    {success_rate:.1f}%')
    node.get_logger().info(f'════════════════════════════════════')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
