# Copyright 2024 Husarion sp. z o.o.
# Decomposed for per-component resource profiling.
#
# Nodes: pointcloud_crop_box, pointcloud_to_laserscan
# Purpose: Convert PointCloud2 → filtered PointCloud2 → LaserScan
# Expected resource profile: lightweight, CPU-bound, cache-insensitive (baseline)

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

    namespace = LaunchConfiguration("namespace")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    params_file = LaunchConfiguration("params_file")
    robot_model = LaunchConfiguration("robot_model")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
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
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [husarion_ugv_navigation, "config", "nav2_params.yaml"]
        ),
        description="Path to the parameters file.",
    )
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model.",
        choices=["lynx", "panther"],
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
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

    observation_topic_filtered = PythonExpression(
        ["'", observation_topic, "_filtered'"],
    )

    # CRITICAL: We must shadow `params_file` (not use a different variable name)
    # so that each call chains off the previous ReplaceString output.
    # See file header comment for explanation.
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

    # Shadow params_file — same pattern as bringup_launch.py
    params_file = override_params_file("panther")
    params_file = override_params_file("lynx")

    param_substitutions = {"use_sim_time": use_sim_time}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    sensor_preproc_group = GroupAction(
        [
            PushRosNamespace(namespace),
            # PointCloud crop box filter — removes self-hits from the robot body
            Node(
                condition=IfCondition(
                    PythonExpression(
                        ["'", observation_topic_type, "' == 'pointcloud'"]
                    )
                ),
                package="pointcloud_crop_box",
                executable="pointcloud_crop_box_node",
                name="pointcloud_crop_box",
                parameters=[configured_params],
                output="screen",
            ),
            # PointCloud → LaserScan conversion for 2D costmap layers
            Node(
                condition=IfCondition(
                    PythonExpression(
                        ["'", observation_topic_type, "' == 'pointcloud'"]
                    )
                ),
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                parameters=[configured_params],
                remappings=[("cloud_in", observation_topic_filtered)],
                output="screen",
            ),
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_namespace_arg,
            declare_observation_topic_arg,
            declare_observation_topic_type_arg,
            declare_params_file_arg,
            declare_robot_model_arg,
            declare_use_sim_time_arg,
            sensor_preproc_group,
        ]
    )
