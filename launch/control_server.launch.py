"""
control_server.launch.py  —  standalone production launch

Loads ControlServerNode alone in a single-threaded component container.
No FakeUnitreeApiNode — suitable for connecting to a real Unitree robot.

For the dev/debug variant (with FakeUnitreeApiNode), use:
    control_server_composition.launch.py

Launch arguments
────────────────
  server_name          CSM name exposed by this server  (default: control_server)
  watchdog_interval_ms Safety watchdog poll period [ms] (default: 100)

Usage
─────
  ros2 launch rv2_server_control control_server.launch.py
  ros2 launch rv2_server_control control_server.launch.py \\
      server_name:=my_robot  watchdog_interval_ms:=200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rv2_server_control'),
                'config',
                'control_server.yaml',
            ]),
            description='Path to ControlServerNode ROS 2 parameter YAML file'),
        DeclareLaunchArgument(
            'server_name', default_value='control_server',
            description='CSM name exposed by ControlServerNode'),
        DeclareLaunchArgument(
            'watchdog_interval_ms', default_value='100',
            description='Safety watchdog poll period in milliseconds'),
    ]

    container = ComposableNodeContainer(
        name='control_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rv2_server_control',
                plugin='ControlServerNode',
                name='control_server',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {
                        'server_name':        LaunchConfiguration('server_name'),
                        'watchdog_interval_ms': LaunchConfiguration('watchdog_interval_ms'),
                    },
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription(args + [container])
