"""Start the R1 control server and one master, respecting deployment YAML."""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    """Resolve optional overrides without replacing values from the YAML file."""
    config_file = LaunchConfiguration("config_file").perform(context)
    with open(config_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    parameters = {}
    for key in ("/**", "control_server"):
        parameters.update(config.get(key, {}).get("ros__parameters", {}))
    overrides = {}
    types = {
        "server_name": str,
        "master_name": str,
        "watchdog_interval_ms": int,
        "log_output": bool,
    }
    for name, value_type in types.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = ParameterValue(value, value_type=value_type)
    master_name = LaunchConfiguration("master_name").perform(context)
    if not master_name:
        master_name = parameters.get("master_name", "csm_master")
    return [
        Node(
            package="rv2_control_signal_transport",
            executable="csm_master_node",
            parameters=[{"master_name": ParameterValue(master_name, value_type=str)}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_master")),
        ),
        Node(
            package="rv2_server_control",
            executable="control_server",
            name="control_server",
            output="screen",
            parameters=[config_file, overrides],
        ),
    ]


def generate_launch_description():
    """Build the production launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rv2_server_control"),
                        "config",
                        "control_server.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("server_name", default_value=""),
            DeclareLaunchArgument("watchdog_interval_ms", default_value=""),
            DeclareLaunchArgument("log_output", default_value=""),
            DeclareLaunchArgument("master_name", default_value=""),
            DeclareLaunchArgument(
                "start_master",
                default_value="true",
                description="Start one CSM master for lifecycle reconciliation",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
