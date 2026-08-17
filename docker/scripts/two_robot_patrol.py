#!/usr/bin/env python3
"""
Multi-robot patrol navigation script for resource profiling experiments.

Navigates TWO robots simultaneously in loops through 5 waypoints each:
  Robot 1 (panther):  MIDDLE → TOP_RIGHT → BOTTOM_RIGHT → BOTTOM_LEFT → TOP_LEFT → MIDDLE
  Robot 2 (panther2): TOP_RIGHT → BOTTOM_RIGHT → BOTTOM_LEFT → TOP_LEFT → MIDDLE → TOP_RIGHT

Usage:
  python3 patrol_multi_robot.py [num_loops] [warmup_secs]

  num_loops   : Number of patrol loops per robot (0 = infinite, default: 0)
  warmup_secs : Seconds to wait after servers are found (default: 5)
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
import threading

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

# Robot 1 (panther): starts at MIDDLE
PATROL_ORDER_ROBOT1 = [
    "TOP_RIGHT",
    "BOTTOM_RIGHT",
    "BOTTOM_LEFT",
    "TOP_LEFT",
    "MIDDLE",
]

# Robot 2 (panther2): starts at TOP_RIGHT
PATROL_ORDER_ROBOT2 = [
    "BOTTOM_RIGHT",
    "BOTTOM_LEFT",
    "TOP_LEFT",
    "MIDDLE",
    "TOP_RIGHT",
]

# Action result status codes
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


class PatrolNode(Node):
    """ROS 2 node that patrols through waypoints using Nav2."""

    def __init__(self, namespace, node_name):
        super().__init__(
            node_name,
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

        # Stats
        self.total_goals = 0
        self.total_successes = 0
        self.total_failures = 0
        self.loop_count = 0

        self.get_logger().info(f'Patrol node created for {namespace}')
        self.get_logger().info(f'  Action server: {action_name}')
        self.get_logger().info(f'  E-stop service: {estop_service_name}')

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

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal to {waypoint_name} REJECTED')
            return False, 0.0

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


def run_patrol(node, patrol_order, num_loops):
    """Run the patrol loop for a single robot. Called in a thread."""
    try:
        while not node.shutdown_requested:
            node.loop_count += 1
            if num_loops > 0 and node.loop_count > num_loops:
                break

            node.get_logger().info(
                f'═══ [{node.namespace}] Loop {node.loop_count} '
                f'{"of " + str(num_loops) if num_loops > 0 else "(∞)"} ═══'
            )
            loop_start = time.time()

            for waypoint_name in patrol_order:
                if node.shutdown_requested:
                    break

                node.total_goals += 1
                success, duration = node.send_goal(waypoint_name)

                if success:
                    node.total_successes += 1
                else:
                    node.total_failures += 1
                    # Retry once on failure
                    node.get_logger().info(f'Retrying {waypoint_name}...')
                    node.total_goals += 1
                    success, duration = node.send_goal(waypoint_name)
                    if success:
                        node.total_successes += 1
                    else:
                        node.total_failures += 1
                        node.get_logger().warning(
                            f'Skipping {waypoint_name} after retry failure'
                        )

            loop_duration = time.time() - loop_start
            success_rate = (
                node.total_successes / node.total_goals * 100
                if node.total_goals > 0 else 0
            )
            node.get_logger().info(
                f'[{node.namespace}] Loop {node.loop_count} done in '
                f'{loop_duration:.1f}s | '
                f'Success: {node.total_successes}/{node.total_goals} '
                f'({success_rate:.0f}%)'
            )

    except Exception as e:
        node.get_logger().error(f'[{node.namespace}] Exception: {e}')


def main():
    # Parse arguments
    num_loops = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    warmup_secs = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    rclpy.init()

    # Create patrol nodes for both robots
    node1 = PatrolNode('panther', 'patrol_panther')
    node2 = PatrolNode('panther2', 'patrol_panther2')

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        node1.get_logger().info('Shutdown requested for all robots...')
        node1.shutdown_requested = True
        node2.shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # --- Startup sequence ---
    # Wait for clock (only need one node to check)
    if not node1.wait_for_clock():
        node1.get_logger().error('Aborting: no clock')
        rclpy.shutdown()
        return

    # Wait for both action servers
    node1.get_logger().info('Waiting for Robot 1 (panther) action server...')
    if not node1.wait_for_server():
        node1.get_logger().error('Aborting: panther action server not found')
        rclpy.shutdown()
        return

    node2.get_logger().info('Waiting for Robot 2 (panther2) action server...')
    if not node2.wait_for_server():
        node2.get_logger().error('Aborting: panther2 action server not found')
        rclpy.shutdown()
        return

    # Disable e-stop on both robots
    if not node1.disable_estop():
        node1.get_logger().warning(
            'Could not disable e-stop on panther — continuing anyway'
        )
    if not node2.disable_estop():
        node2.get_logger().warning(
            'Could not disable e-stop on panther2 — continuing anyway'
        )

    # Warmup pause
    node1.get_logger().info(
        f'Both robots ready. Warmup pause: {warmup_secs}s '
        f'(attach profiling tools now!)'
    )
    time.sleep(warmup_secs)

    # --- Start patrol loops in parallel threads ---
    loop_label = f'{num_loops} loops' if num_loops > 0 else 'infinite loops'
    node1.get_logger().info(
        f'Starting multi-robot patrol ({loop_label} per robot)'
    )

    thread1 = threading.Thread(
        target=run_patrol,
        args=(node1, PATROL_ORDER_ROBOT1, num_loops),
        daemon=True,
    )
    thread2 = threading.Thread(
        target=run_patrol,
        args=(node2, PATROL_ORDER_ROBOT2, num_loops),
        daemon=True,
    )

    thread1.start()
    thread2.start()

    # Wait for both threads to finish
    thread1.join()
    thread2.join()

    # --- Summary ---
    total_goals = node1.total_goals + node2.total_goals
    total_successes = node1.total_successes + node2.total_successes
    total_failures = node1.total_failures + node2.total_failures
    overall_rate = (
        total_successes / total_goals * 100 if total_goals > 0 else 0
    )

    r1_rate = (
        node1.total_successes / node1.total_goals * 100
        if node1.total_goals > 0 else 0
    )
    r2_rate = (
        node2.total_successes / node2.total_goals * 100
        if node2.total_goals > 0 else 0
    )

    node1.get_logger().info('')
    node1.get_logger().info('══════════ MULTI-ROBOT PATROL SUMMARY ══════════')
    node1.get_logger().info(
        f'  Robot 1 (panther):  {node1.loop_count} loops, '
        f'{node1.total_successes}/{node1.total_goals} goals '
        f'({r1_rate:.0f}%)'
    )
    node1.get_logger().info(
        f'  Robot 2 (panther2): {node2.loop_count} loops, '
        f'{node2.total_successes}/{node2.total_goals} goals '
        f'({r2_rate:.0f}%)'
    )
    node1.get_logger().info(
        f'  TOTAL:              '
        f'{total_successes}/{total_goals} goals '
        f'({overall_rate:.0f}%)'
    )
    node1.get_logger().info('═════════════════════════════════════════════════')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
