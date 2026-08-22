#!/usr/bin/env python3
"""
collect_ros2_metrics.py — ROS2 Application & QoS Metrics Collector
=============================================================================
Passively monitors Nav2 topics to collect application-level and QoS metrics
for resource-contention (noisy-neighbor) profiling experiments.

Outputs:
  ros2_rates.csv  — 1 Hz time-series of topic frequencies and jitter
  ros2_goals.csv  — per-navigation-goal event log (duration, status, recoveries)

Usage:
  python3 collect_ros2_metrics.py <ns1,ns2,...> <output_dir> [duration_secs]

Examples:
  # Two robots, 300s collection
  python3 collect_ros2_metrics.py panther,panther2 results/exp1_20260821 300

  # Single robot, run until Ctrl+C
  python3 collect_ros2_metrics.py panther results/baseline

  # Inside Docker (with ros_entrypoint sourced)
  bash -c "source /ros_entrypoint.sh && python3 /scripts/collect_ros2_metrics.py panther,panther2 /output 300"
=============================================================================
"""

import csv
import math
import os
import signal
import sys
import threading
import time
import traceback
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2

# ---------------------------------------------------------------------------
# Action status codes (action_msgs/msg/GoalStatus)
# ---------------------------------------------------------------------------
_ACCEPTED = 1
_EXECUTING = 2
_SUCCEEDED = 4
_CANCELED = 5
_ABORTED = 6
_TERMINAL = {_SUCCEEDED, _CANCELED, _ABORTED}
_STATUS_NAMES = {
    0: "UNKNOWN",
    _ACCEPTED: "ACCEPTED",
    _EXECUTING: "EXECUTING",
    3: "CANCELING",
    _SUCCEEDED: "SUCCEEDED",
    _CANCELED: "CANCELED",
    _ABORTED: "ABORTED",
}

# ---------------------------------------------------------------------------
# Topics to track for frequency measurement
# Each entry: label -> (message_type, topic_pattern with {ns} placeholder, qos_key)
#   qos_key selects which QoS profile to use (defined in __init__)
# ---------------------------------------------------------------------------
_RATE_TOPICS = {
    "cmd_vel_nav":       (TwistStamped,                "/{ns}/cmd_vel_nav",                "sensor"),
    "amcl_pose":         (PoseWithCovarianceStamped,   "/{ns}/amcl_pose",                  "sensor"),
    "plan":              (Path,                        "/{ns}/plan",                       "sensor"),
    "local_costmap":     (OccupancyGrid,               "/{ns}/local_costmap/costmap",      "costmap"),
    "velodyne_filtered": (PointCloud2,                 "/{ns}/velodyne_points_filtered",   "sensor"),
}

# Recovery behavior action names to monitor
_RECOVERY_BEHAVIORS = ("spin", "backup", "wait", "drive_on_heading")


class MetricsCollector(Node):
    """Passively monitors Nav2 topics and records application-level metrics."""

    def __init__(self, namespaces: list[str], output_dir: str, duration: int = 0):
        super().__init__(
            "ros2_metrics_collector",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )
        self.namespaces = namespaces
        self.output_dir = output_dir
        self.duration = duration
        self.start_time = time.monotonic()
        self.shutdown_flag = False

        self._lock = threading.Lock()

        # ── Rate tracking ─────────────────────────────────────────────
        # key = "ns/label" -> list of monotonic timestamps (pruned each tick)
        self._msg_stamps: dict[str, list[float]] = defaultdict(list)

        # ── Plan path length ──────────────────────────────────────────
        self._last_path_length: dict[str, float] = {}  # ns -> metres

        # ── Navigation goal lifecycle ─────────────────────────────────
        self._goal_states: dict[tuple, int] = {}       # (ns, uuid) -> last status
        self._goal_starts: dict[tuple, tuple] = {}     # (ns, uuid) -> (mono_t, wall_t)
        self._goal_log: list[dict] = []

        # ── Recovery tracking ─────────────────────────────────────────
        self._seen_recovery_ids: set[tuple] = set()    # (ns, behavior, uuid)
        # Accumulate recoveries between nav goal start and end
        self._recovery_accum: dict[str, dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # ── Diagnostics ──────────────────────────────────────────────
        self._nav_status_cb_count = 0     # how many times callback fired
        self._goals_started = 0           # goals we recorded a start for
        self._goals_completed = 0         # goals we logged as complete
        self._goals_skipped_historical = 0  # goals seen first as terminal

        # ── QoS profiles ─────────────────────────────────────────────
        qos_profiles = {
            "sensor": QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
            "costmap": QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        }
        # Nav2 Jazzy action status topics use RELIABLE + TRANSIENT_LOCAL
        qos_action_status = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Create subscriptions for each namespace ───────────────────
        for ns in namespaces:
            for label, (msg_type, pattern, qos_key) in _RATE_TOPICS.items():
                topic = pattern.replace("{ns}", ns)
                self.create_subscription(
                    msg_type,
                    topic,
                    self._make_rate_cb(ns, label, msg_type),
                    qos_profiles[qos_key],
                )

            # NavigateToPose action status
            nav_status_topic = f"/{ns}/navigate_to_pose/_action/status"
            self.create_subscription(
                GoalStatusArray,
                nav_status_topic,
                lambda msg, n=ns: self._on_nav_status(msg, n),
                qos_action_status,
            )
            self.get_logger().info(f"Subscribed to {nav_status_topic}")

            # Recovery behavior action statuses
            for beh in _RECOVERY_BEHAVIORS:
                recovery_topic = f"/{ns}/{beh}/_action/status"
                self.create_subscription(
                    GoalStatusArray,
                    recovery_topic,
                    lambda msg, n=ns, b=beh: self._on_recovery_status(msg, n, b),
                    qos_action_status,
                )

        # ── CSV setup ─────────────────────────────────────────────────
        os.makedirs(output_dir, exist_ok=True)
        self._init_rate_csv()
        self._init_goal_csv()

        # ── 1 Hz snapshot timer ───────────────────────────────────────
        self.create_timer(1.0, self._snapshot_rates)

        self.get_logger().info(
            f"Collecting metrics for {namespaces} → {output_dir}"
            + (f" ({duration}s)" if duration > 0 else " (until Ctrl+C)")
        )

    # =====================================================================
    # Subscription callbacks
    # =====================================================================

    def _make_rate_cb(self, ns: str, label: str, msg_type):
        """Return a closure that records message timestamps (and path length for plans)."""
        key = f"{ns}/{label}"
        if msg_type is Path:
            def cb(msg):
                with self._lock:
                    self._msg_stamps[key].append(time.monotonic())
                    self._last_path_length[ns] = self._path_length(msg.poses)
            return cb
        else:
            def cb(msg):
                with self._lock:
                    self._msg_stamps[key].append(time.monotonic())
            return cb

    def _on_nav_status(self, msg, ns: str):
        """Track NavigateToPose goal lifecycle: start → terminal status."""
        try:
            now_mono = time.monotonic()
            now_wall = time.time()

            with self._lock:
                self._nav_status_cb_count += 1

                for s in msg.status_list:
                    uid = tuple(s.goal_info.goal_id.uuid)
                    gkey = (ns, uid)
                    prev = self._goal_states.get(gkey)
                    curr = s.status

                    if prev == curr:
                        # No state change — skip (most common path)
                        continue

                    # ── First time seeing this goal ──────────────────
                    if prev is None:
                        if curr in _TERMINAL:
                            # Goal completed before we started — skip
                            self._goals_skipped_historical += 1
                        else:
                            # Goal is in progress — record start
                            self._goal_starts[gkey] = (now_mono, now_wall)
                            self._goals_started += 1
                            self.get_logger().info(
                                f"[{ns}] Goal started: "
                                f"status={_STATUS_NAMES.get(curr, curr)}"
                            )
                        self._goal_states[gkey] = curr
                        continue

                    # ── State changed for a known goal ───────────────
                    self._goal_states[gkey] = curr

                    if curr not in _TERMINAL:
                        # Non-terminal transition (e.g. ACCEPTED → EXECUTING)
                        continue

                    # ── Goal reached terminal state ──────────────────
                    start_info = self._goal_starts.get(gkey)
                    if start_info is None:
                        # No start recorded (historical goal re-published)
                        continue

                    mono_start, wall_start = start_info
                    duration = now_mono - mono_start
                    path_len = self._last_path_length.get(ns, 0.0)

                    # Collect recovery counts for this namespace
                    rec = self._recovery_accum.get(ns, {})
                    rec_spin = rec.get("spin", 0)
                    rec_backup = rec.get("backup", 0)
                    rec_wait = rec.get("wait", 0)
                    rec_drive = rec.get("drive_on_heading", 0)

                    entry = {
                        "wall_time": now_wall,
                        "namespace": ns,
                        "status": _STATUS_NAMES.get(curr, str(curr)),
                        "duration_s": round(duration, 3),
                        "path_length_m": round(path_len, 3),
                        "recoveries_spin": rec_spin,
                        "recoveries_backup": rec_backup,
                        "recoveries_wait": rec_wait,
                        "recoveries_drive_on_heading": rec_drive,
                    }
                    self._goal_log.append(entry)
                    self._goals_completed += 1

                    self.get_logger().info(
                        f"[{ns}] Goal {_STATUS_NAMES.get(curr, curr)} "
                        f"in {duration:.1f}s "
                        f"(recoveries: spin={rec_spin} backup={rec_backup} "
                        f"wait={rec_wait} drive={rec_drive})"
                    )

                    # Reset recovery accumulator for next goal
                    if ns in self._recovery_accum:
                        self._recovery_accum[ns] = defaultdict(int)

                    # Clean up start record
                    del self._goal_starts[gkey]

        except Exception as e:
            self.get_logger().error(
                f"Exception in _on_nav_status: {e}\n{traceback.format_exc()}"
            )

    def _on_recovery_status(self, msg, ns: str, behavior: str):
        """Count recovery behavior triggers (spin, backup, wait, drive_on_heading)."""
        try:
            with self._lock:
                for s in msg.status_list:
                    uid = tuple(s.goal_info.goal_id.uuid)
                    rkey = (ns, behavior, uid)
                    if rkey in self._seen_recovery_ids:
                        continue
                    # Count when first seen as EXECUTING or terminal
                    if s.status >= _EXECUTING:
                        self._seen_recovery_ids.add(rkey)
                        self._recovery_accum[ns][behavior] += 1
                        self.get_logger().info(
                            f"[{ns}] Recovery triggered: {behavior}"
                        )
        except Exception as e:
            self.get_logger().error(
                f"Exception in _on_recovery_status: {e}\n{traceback.format_exc()}"
            )

    # =====================================================================
    # Path length helper
    # =====================================================================

    @staticmethod
    def _path_length(poses) -> float:
        """Sum of Euclidean distances between consecutive poses."""
        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            length += math.hypot(dx, dy)
        return length

    # =====================================================================
    # CSV initialisation
    # =====================================================================

    def _init_rate_csv(self):
        rate_path = os.path.join(self.output_dir, "ros2_rates.csv")
        self._rate_file = open(rate_path, "w", newline="")
        self._rate_writer = csv.writer(self._rate_file)

        header = ["wall_time", "elapsed_s"]
        for ns in self.namespaces:
            for label in _RATE_TOPICS:
                header.append(f"{ns}/{label}_hz")
                header.append(f"{ns}/{label}_jitter_ms")
            header.append(f"{ns}/path_length_m")
        self._rate_writer.writerow(header)
        self._rate_file.flush()

    def _init_goal_csv(self):
        goal_path = os.path.join(self.output_dir, "ros2_goals.csv")
        self._goal_file = open(goal_path, "w", newline="")
        self._goal_writer = csv.DictWriter(
            self._goal_file,
            fieldnames=[
                "wall_time", "namespace", "status", "duration_s",
                "path_length_m", "recoveries_spin", "recoveries_backup",
                "recoveries_wait", "recoveries_drive_on_heading",
            ],
        )
        self._goal_writer.writeheader()
        self._goal_file.flush()
        self.get_logger().info(f"Goal CSV initialised: {goal_path}")

    # =====================================================================
    # 1 Hz snapshot — rates + flush goals
    # =====================================================================

    def _snapshot_rates(self):
        try:
            now = time.monotonic()
            elapsed = now - self.start_time

            with self._lock:
                row = [f"{time.time():.3f}", f"{elapsed:.1f}"]

                for ns in self.namespaces:
                    for label in _RATE_TOPICS:
                        key = f"{ns}/{label}"
                        stamps = self._msg_stamps[key]

                        # Prune stamps older than 2 seconds
                        cutoff = now - 2.0
                        stamps[:] = [t for t in stamps if t > cutoff]

                        if len(stamps) >= 2:
                            window = stamps[-1] - stamps[0]
                            hz = (len(stamps) - 1) / window if window > 0 else 0.0
                            intervals = [
                                stamps[i] - stamps[i - 1]
                                for i in range(1, len(stamps))
                            ]
                            mean_iv = sum(intervals) / len(intervals)
                            jitter_ms = (
                                math.sqrt(
                                    sum((iv - mean_iv) ** 2 for iv in intervals)
                                    / len(intervals)
                                )
                                * 1000.0
                            )
                        else:
                            hz = float(len(stamps)) * 0.5
                            jitter_ms = 0.0

                        row.append(f"{hz:.1f}")
                        row.append(f"{jitter_ms:.2f}")

                    row.append(
                        f"{self._last_path_length.get(ns, 0.0):.3f}"
                    )

                self._rate_writer.writerow(row)
                self._rate_file.flush()

                # ── Flush completed goals to CSV ─────────────────────
                if self._goal_log:
                    for entry in self._goal_log:
                        self._goal_writer.writerow(entry)
                    self._goal_file.flush()
                    self.get_logger().info(
                        f"Flushed {len(self._goal_log)} goal(s) to CSV"
                    )
                    self._goal_log.clear()

                # ── Periodic diagnostics (every 30s) ─────────────────
                tick = int(elapsed)
                if tick > 0 and tick % 30 == 0:
                    active = len(self._goal_starts)
                    self.get_logger().info(
                        f"[diag] elapsed={tick}s "
                        f"status_cb_calls={self._nav_status_cb_count} "
                        f"goals: started={self._goals_started} "
                        f"completed={self._goals_completed} "
                        f"active={active} "
                        f"skipped_historical={self._goals_skipped_historical}"
                    )

            # ── Check duration ───────────────────────────────────────
            if self.duration > 0 and elapsed >= self.duration:
                self.get_logger().info("Duration reached — shutting down")
                self.shutdown_flag = True

        except Exception as e:
            self.get_logger().error(
                f"Exception in _snapshot_rates: {e}\n{traceback.format_exc()}"
            )

    # =====================================================================
    # Shutdown
    # =====================================================================

    def shutdown(self):
        self.get_logger().info(
            f"Final stats: status_cb_calls={self._nav_status_cb_count} "
            f"goals_started={self._goals_started} "
            f"goals_completed={self._goals_completed} "
            f"goals_skipped_historical={self._goals_skipped_historical}"
        )

        with self._lock:
            # Flush any remaining goals
            if self._goal_log:
                for entry in self._goal_log:
                    self._goal_writer.writerow(entry)
                self._goal_file.flush()
                self.get_logger().info(
                    f"Final flush: {len(self._goal_log)} goal(s)"
                )
                self._goal_log.clear()

        if hasattr(self, "_rate_file"):
            self._rate_file.close()
        if hasattr(self, "_goal_file"):
            self._goal_file.close()
        self.get_logger().info("CSV files closed")


# =========================================================================
# Main
# =========================================================================

def main():
    if len(sys.argv) < 3:
        print(
            "Usage: collect_ros2_metrics.py <ns1,ns2,...> <output_dir> [duration_secs]"
        )
        sys.exit(1)

    namespaces = sys.argv[1].split(",")
    output_dir = sys.argv[2]
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 0

    rclpy.init()
    node = MetricsCollector(namespaces, output_dir, duration)

    shutdown_event = threading.Event()

    def _signal_handler(sig, frame):
        node.get_logger().info(f"Signal {sig} received — stopping")
        node.shutdown_flag = True
        shutdown_event.set()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
