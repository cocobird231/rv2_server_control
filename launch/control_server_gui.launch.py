"""Open the status GUI, optionally starting the matching control server and master."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Keep GUI-only as the default so observing a running server is safe."""
    share = FindPackageShare("rv2_server_control")
    gui = Node(
        package="rv2_server_control",
        executable="control_server_gui",
        name="control_server_gui",
        output="screen",
        parameters=[
            {
                "server_name": ParameterValue(
                    LaunchConfiguration("server_name"), value_type=str
                ),
                "poll_interval_ms": ParameterValue(
                    LaunchConfiguration("poll_interval_ms"), value_type=int
                ),
                "service_timeout_ms": ParameterValue(
                    LaunchConfiguration("service_timeout_ms"), value_type=int
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("server_name", default_value="control_server"),
            DeclareLaunchArgument("poll_interval_ms", default_value="500"),
            DeclareLaunchArgument("service_timeout_ms", default_value="2000"),
            DeclareLaunchArgument("start_server", default_value="false"),
            DeclareLaunchArgument("start_master", default_value="true"),
            DeclareLaunchArgument("master_name", default_value=""),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "control_server.yaml"]
                ),
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration("start_server")),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [share, "launch", "control_server.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "config_file": LaunchConfiguration("config_file"),
                            "server_name": LaunchConfiguration("server_name"),
                            "master_name": LaunchConfiguration("master_name"),
                            "start_master": LaunchConfiguration("start_master"),
                        }.items(),
                    )
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gui,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="Control server GUI closed"))
                    ],
                ),
                condition=IfCondition(LaunchConfiguration("start_server")),
            ),
            gui,
        ]
    )
