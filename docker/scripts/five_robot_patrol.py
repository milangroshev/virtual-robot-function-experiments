#!/usr/bin/env python3
"""
Multi-robot patrol navigation script for resource profiling experiments.

Navigates FIVE robots simultaneously in loops through 5 waypoints each:
  Robot 1 (panther):  MIDDLE → TOP_RIGHT → BOTTOM_RIGHT → BOTTOM_LEFT → TOP_LEFT
  Robot 2 (panther2): TOP_RIGHT → BOTTOM_RIGHT → BOTTOM_LEFT → TOP_LEFT → MIDDLE
  Robot 3 (panther3): BOTTOM_RIGHT → BOTTOM_LEFT → TOP_LEFT → MIDDLE → TOP_RIGHT
  Robot 4 (panther4): BOTTOM_LEFT → TOP_LEFT → MIDDLE → TOP_RIGHT → BOTTOM_RIGHT
  Robot 5 (panther5): TOP_LEFT → MIDDLE → TOP_RIGHT → BOTTOM_RIGHT → BOTTOM_LEFT

Usage:
  python3 five_robot_patrol.py [num_loops] [warmup_secs]

  num_loops   : Number of patrol loops per robot (0 = infinite, default: 0)
  warmup_secs : Seconds to wait after servers are found (default: 5)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
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

# Robot 3 (panther3): starts at BOTTOM_RIGHT
PATROL_ORDER_ROBOT3 = [
    "BOTTOM_LEFT",
    "TOP_LEFT",
    "MIDDLE",
    "TOP_RIGHT",
    "BOTTOM_RIGHT",
]

# Robot 4 (panther4): starts at BOTTOM_LEFT
PATROL_ORDER_ROBOT4 = [
    "TOP_LEFT",
    "MIDDLE",
    "TOP_RIGHT",
    "BOTTOM_RIGHT",
    "BOTTOM_LEFT",
]

# Robot 5 (panther5): starts at TOP_LEFT
PATROL_ORDER_ROBOT5 = [
    "MIDDLE",
    "TOP_RIGHT",
    "BOTTOM_RIGHT",
    "BOTTOM_LEFT",
    "TOP_LEFT",
]

# Action result status codes
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


class PatrolNode(Node):
    """ROS 2 node that patrols through waypoints using Nav2.

    Each instance gets its own SingleThreadedExecutor running in a
    dedicated background thread, so multiple PatrolNodes can run
    concurrently without 'Executor is already spinning' errors.
    """

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

        # --- Executor setup ---
        # Each node gets its own executor to avoid threading conflicts
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = None

        # Stats
        self.total_goals = 0
        self.total_successes = 0
        self.total_failures = 0
        self.loop_count = 0

        self.get_logger().info(f'Patrol node created for {namespace}')
        self.get_logger().info(f'  Action server: {action_name}')
        self.get_logger().info(f'  E-stop service: {estop_service_name}')

    def start_executor(self):
        """Start the executor spin thread."""
        self._spin_thread = threading.Thread(
            target=self._spin_executor, daemon=True
        )
        self._spin_thread.start()

    def _spin_executor(self):
        """Background thread that spins the executor."""
        try:
            while not self.shutdown_requested:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception:
            pass

    def _wait_for_future(self, future, timeout_sec=300.0):
        """Wait for a future to complete by polling (thread-safe)."""
        start = time.time()
        while not future.done():
            if self.shutdown_requested:
                return False
            if time.time() - start > timeout_sec:
                self.get_logger().error('Future timed out')
                return False
            time.sleep(0.1)
        return True

    def wait_for_clock(self, timeout_sec=120.0):
        """Wait until /clock is publishing (sim time is flowing)."""
        self.get_logger().info('Waiting for /clock (sim time)...')
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.shutdown_requested:
                return False
            time.sleep(0.5)
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

            if not self._wait_for_future(future, timeout_sec=5.0):
                self.get_logger().warning(
                    f'E-stop service call timed out '
                    f'(attempt {attempt + 1}/{max_retries})'
                )
                continue

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
                    f'E-stop service call returned None '
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

        # Send goal async
        send_future = self._action_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_future, timeout_sec=10.0):
            self.get_logger().error(f'Timeout sending goal to {waypoint_name}')
            return False, 0.0

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal to {waypoint_name} REJECTED')
            return False, 0.0

        # Wait for result
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=300.0):
            self.get_logger().error(
                f'Timeout waiting for {waypoint_name} result'
            )
            return False, time.time() - goal_start

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

    def run_patrol(self, patrol_order, num_loops):
        """Run the patrol loop for this robot."""
        try:
            while not self.shutdown_requested:
                self.loop_count += 1
                if num_loops > 0 and self.loop_count > num_loops:
                    break

                self.get_logger().info(
                    f'═══ [{self.namespace}] Loop {self.loop_count} '
                    f'{"of " + str(num_loops) if num_loops > 0 else "(∞)"} ═══'
                )
                loop_start = time.time()

                for waypoint_name in patrol_order:
                    if self.shutdown_requested:
                        break

                    self.total_goals += 1
                    success, duration = self.send_goal(waypoint_name)

                    if success:
                        self.total_successes += 1
                    else:
                        self.total_failures += 1
                        # Retry once
                        self.get_logger().info(
                            f'Retrying {waypoint_name}...'
                        )
                        self.total_goals += 1
                        success, duration = self.send_goal(waypoint_name)
                        if success:
                            self.total_successes += 1
                        else:
                            self.total_failures += 1
                            self.get_logger().warning(
                                f'Skipping {waypoint_name} after retry'
                            )

                loop_duration = time.time() - loop_start
                rate = (
                    self.total_successes / self.total_goals * 100
                    if self.total_goals > 0 else 0
                )
                self.get_logger().info(
                    f'[{self.namespace}] Loop {self.loop_count} done '
                    f'in {loop_duration:.1f}s | '
                    f'Success: {self.total_successes}/{self.total_goals} '
                    f'({rate:.0f}%)'
                )

        except Exception as e:
            self.get_logger().error(
                f'[{self.namespace}] Exception: {e}'
            )

    def cleanup(self):
        """Stop executor and clean up."""
        self.shutdown_requested = True
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._executor.shutdown()


def main():
    # Parse arguments
    num_loops = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    warmup_secs = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    rclpy.init()

    # Create all five robot nodes with unique names
    robot1 = PatrolNode('panther', 'patrol_panther')
    robot2 = PatrolNode('panther2', 'patrol_panther2')
    robot3 = PatrolNode('panther3', 'patrol_panther3')
    robot4 = PatrolNode('panther4', 'patrol_panther4')
    robot5 = PatrolNode('panther5', 'patrol_panther5')

    robots = [robot1, robot2, robot3, robot4, robot5]
    shutdown_event = threading.Event()

    # Handle Ctrl+C
    def signal_handler(sig, frame):
        robot1.get_logger().info('Shutdown requested for all robots...')
        for r in robots:
            r.shutdown_requested = True
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start executor threads FIRST (so callbacks are processed)
    for r in robots:
        r.start_executor()

    # Give executors a moment to start processing
    time.sleep(1.0)

    # --- Startup: wait for clock ---
    if not robot1.wait_for_clock():
        robot1.get_logger().error('Aborting: no clock')
        for r in robots:
            r.cleanup()
        rclpy.shutdown()
        return

    # --- Wait for all action servers ---
    for i, r in enumerate(robots, 1):
        robot1.get_logger().info(
            f'Waiting for Robot {i} ({r.namespace}) action server...'
        )
        if not r.wait_for_server():
            robot1.get_logger().error(
                f'Aborting: Robot {i} ({r.namespace}) action server not found'
            )
            for r2 in robots:
                r2.cleanup()
            rclpy.shutdown()
            return

    # --- Disable e-stops ---
    for r in robots:
        r.disable_estop()

    # --- Warmup ---
    robot1.get_logger().info(
        f'All {len(robots)} robots ready. Warmup pause: {warmup_secs}s '
        f'(attach profiling tools now!)'
    )
    time.sleep(warmup_secs)

    # --- Start patrol threads ---
    loop_label = (
        f'{num_loops} loops' if num_loops > 0 else 'infinite loops'
    )
    robot1.get_logger().info(
        f'Starting multi-robot patrol ({loop_label} per robot)'
    )

    patrol_orders = [
        PATROL_ORDER_ROBOT1,
        PATROL_ORDER_ROBOT2,
        PATROL_ORDER_ROBOT3,
        PATROL_ORDER_ROBOT4,
        PATROL_ORDER_ROBOT5,
    ]

    threads = []
    for r, order in zip(robots, patrol_orders):
        t = threading.Thread(
            target=r.run_patrol,
            args=(order, num_loops),
            daemon=True,
        )
        threads.append(t)
        t.start()

    # Wait for all to finish or shutdown
    while any(t.is_alive() for t in threads):
        if shutdown_event.is_set():
            break
        time.sleep(0.5)

    for t in threads:
        t.join(timeout=5.0)

    # --- Summary ---
    total_goals = sum(r.total_goals for r in robots)
    total_success = sum(r.total_successes for r in robots)
    total_fail = sum(r.total_failures for r in robots)
    rate = total_success / total_goals * 100 if total_goals > 0 else 0

    robot1.get_logger().info('')
    robot1.get_logger().info('══════════ MULTI-ROBOT PATROL SUMMARY ══════════')
    for i, r in enumerate(robots, 1):
        r_rate = (
            r.total_successes / r.total_goals * 100
            if r.total_goals > 0 else 0
        )
        robot1.get_logger().info(
            f'  Robot {i} ({r.namespace:>8s}): {r.loop_count} loops, '
            f'{r.total_successes}/{r.total_goals} goals '
            f'({r_rate:.0f}%)'
        )
    robot1.get_logger().info(
        f'  TOTAL:              {total_success}/{total_goals} '
        f'({rate:.0f}%)'
    )
    robot1.get_logger().info(
        '═══════════════════════════════════════════════'
    )

    # Cleanup
    for r in robots:
        r.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
