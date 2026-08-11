# Copyright 2024 Husarion sp. z o.o.
# Decomposed for per-component resource profiling.
#
# Nodes: bt_navigator, behavior_server, waypoint_follower
# Purpose: High-level mission orchestration — decides WHEN to plan/replan,
#          handles recoveries (spin, backup, wait), follows waypoint sequences
# Expected resource profile:
#   - Lightweight, event-driven (reacts to action server calls)
#   - Cache-insensitive — minimal working set
#   - Low CPU — BT tick rate is typically 10 Hz
#   - This is your CONTROL GROUP for profiling experiments

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    husarion_ugv_navigation = FindPackageShare("husarion_ugv_navigation")
    husarion_dir = get_package_share_directory("husarion_ugv_navigation")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    robot_model = LaunchConfiguration("robot_model")

    # These are the ONLY lifecycle nodes managed by THIS container's lifecycle manager
    lifecycle_nodes = [
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Top-level namespace.",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [husarion_ugv_navigation, "config", "nav2_params.yaml"]
        ),
        description="Path to the parameters file.",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nodes.",
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes.",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level.",
    )
    declare_observation_topic_cmd = DeclareLaunchArgument(
        "observation_topic",
        default_value="",
        description="Topic name for PointCloud2 observation messages.",
    )
    declare_observation_topic_type_cmd = DeclareLaunchArgument(
        "observation_topic_type",
        default_value="pointcloud",
        description="Observation topic type.",
        choices=["laserscan", "pointcloud"],
    )
    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model.",
        choices=["lynx", "panther"],
    )

    # --- Parameter substitutions (same logic as bringup_launch.py) ---
    namespace_ext = PythonExpression(
        ["'", namespace, "' + '/' if '", namespace, "' else ''"]
    )
    scan_topic = PythonExpression(
        [
            "'scan' if '",
            observation_topic_type,
            "' == 'pointcloud' else '",
            observation_topic,
            "'",
        ]
    )
    stvl_layer = PythonExpression(
        [
            "'stvl_pointcloud_layer' if '",
            observation_topic_type,
            "' == 'pointcloud' else 'stvl_laserscan_layer'",
        ]
    )

    bb_padding = 0.03
    robot_bounding_box = {
        "panther": {
            "min_x": -0.41 - bb_padding,
            "min_y": -0.43 - bb_padding,
            "min_z": 0.05,
            "max_x": 0.41 + bb_padding,
            "max_y": 0.43 + bb_padding,
            "max_z": 0.5,
        },
        "lynx": {
            "min_x": -0.32 - bb_padding,
            "min_y": -0.27 - bb_padding,
            "min_z": 0.05,
            "max_x": 0.32 + bb_padding,
            "max_y": 0.27 + bb_padding,
            "max_z": 0.5,
        },
    }

    def override_params_file(robot_model_name):
        bounding_box = robot_bounding_box[robot_model_name]
        return ReplaceString(
            source_file=params_file,
            replacements={
                "<namespace>/": namespace_ext,
                "<min_x>": str(bounding_box["min_x"]),
                "<max_x>": str(bounding_box["max_x"]),
                "<min_y>": str(bounding_box["min_y"]),
                "<max_y>": str(bounding_box["max_y"]),
                "<min_z>": str(bounding_box["min_z"]),
                "<max_z>": str(bounding_box["max_z"]),
                "<observation_topic>": observation_topic,
                "<observation_topic_type>": observation_topic_type,
                "<scan_topic>": scan_topic,
                "<stvl_layer>": stvl_layer,
            },
            condition=IfCondition(
                PythonExpression(["'", robot_model, f"' == '{robot_model_name}'"])
            ),
        )

    # Shadow params_file — same chaining pattern as bringup_launch.py
    params_file = override_params_file("panther")
    params_file = override_params_file("lynx")

    param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Custom behavior trees from Husarion
    nav_to_pose_bt = {
        "default_nav_to_pose_bt_xml": os.path.join(
            husarion_dir,
            "behavior_trees",
            "navigate_to_pose_w_replanning_and_recovery.xml",
        )
    }
    nav_through_poses_bt = {
        "default_nav_through_poses_bt_xml": os.path.join(
            husarion_dir,
            "behavior_trees",
            "navigate_through_poses_w_replanning_and_recovery.xml",
        )
    }

    bt_orchestrator_group = GroupAction(
        [
            PushRosNamespace(namespace),
            # --- Behavior Server (recovery behaviors: spin, backup, wait) ---
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # --- BT Navigator (behavior tree execution engine) ---
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    nav_to_pose_bt,
                    nav_through_poses_bt,
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # --- Waypoint Follower ---
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # --- Lifecycle Manager (manages ONLY BT + behavior + waypoint) ---
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_bt",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_params_file_cmd,
            declare_autostart_cmd,
            declare_use_respawn_cmd,
            declare_log_level_cmd,
            declare_observation_topic_cmd,
            declare_observation_topic_type_cmd,
            declare_robot_model_cmd,
            bt_orchestrator_group,
        ]
    )
