# Copyright 2024 Husarion sp. z o.o.
# Decomposed for per-component resource profiling.
#
# Nodes (SLAM=True):  slam_toolbox, map_saver_server
# Nodes (SLAM=False): amcl, map_server
# Purpose: "Where am I?" — localization in a known or unknown map
#
# CRITICAL FIX: This wrapper must perform the SAME ReplaceString substitutions
# as bringup_launch.py before passing params_file to the sub-launches.
# The shared nav2_params.yaml contains placeholders like <namespace>/, <scan_topic>,
# <stvl_layer>, <min_x>, etc. If these aren't replaced, ROS 2 fails to parse
# the YAML (the < and > characters are invalid in namespace strings).
#
# In bringup_launch.py, this happens at the top level ONCE, and the substituted
# params_file is passed to all sub-launches. Here, each decomposed launch file
# must do it independently since they run in separate containers.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    husarion_ugv_navigation = FindPackageShare("husarion_ugv_navigation")
    launch_dir = PathJoinSubstitution([husarion_ugv_navigation, "launch"])

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    robot_model = LaunchConfiguration("robot_model")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Top-level namespace.",
    )
    declare_slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="True",
        description="Whether to run SLAM (True) or AMCL localization (False).",
    )
    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value="/maps/map.yaml",
        description="Path to map yaml file (used when slam=False).",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the localization nodes.",
    )
    declare_use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description=(
            "Use composed bringup if True. "
            "Set to False for profiling (each node = own process)."
        ),
    )
    declare_use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes.",
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [husarion_ugv_navigation, "config", "nav2_params.yaml"]
        ),
        description="Path to the parameters file.",
    )
    declare_observation_topic_arg = DeclareLaunchArgument(
        "observation_topic",
        default_value="",
        description="Topic name for PointCloud2 observation messages.",
    )
    declare_observation_topic_type_arg = DeclareLaunchArgument(
        "observation_topic_type",
        default_value="pointcloud",
        description="Observation topic type.",
        choices=["laserscan", "pointcloud"],
    )
    declare_robot_model_arg = DeclareLaunchArgument(
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

    # Shadow params_file with ReplaceString chain — same pattern as bringup_launch.py
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

    params_file = override_params_file("panther")
    params_file = override_params_file("lynx")

    # --- SLAM mode: launch slam_toolbox ---
    # slam_launch.py does NOT declare 'namespace' — it only accepts:
    #   params_file, use_sim_time, autostart, use_respawn, log_level
    # The namespace is applied by PushRosNamespace in the wrapping GroupAction.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, "slam_launch.py"])
        ),
        condition=IfCondition(slam),
        launch_arguments={
            "autostart": autostart,
            "params_file": params_file,
            "use_respawn": use_respawn,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # --- AMCL mode: launch amcl + map_server ---
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, "localization_launch.py"])
        ),
        condition=UnlessCondition(slam),
        launch_arguments={
            "autostart": autostart,
            "container_name": "nav2_container",
            "map": map_yaml,
            "namespace": namespace,
            "params_file": params_file,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Wrap in GroupAction with PushRosNamespace — this is how the original
    # bringup_launch.py provides the namespace to these sub-launches.
    localization_group = GroupAction(
        [
            PushRosNamespace(namespace),
            slam_launch,
            localization_launch,
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_namespace_arg,
            declare_slam_arg,
            declare_map_arg,
            declare_use_sim_time_arg,
            declare_autostart_arg,
            declare_use_composition_arg,
            declare_use_respawn_arg,
            declare_params_file_arg,
            declare_observation_topic_arg,
            declare_observation_topic_type_arg,
            declare_robot_model_arg,
            localization_group,
        ]
    )
