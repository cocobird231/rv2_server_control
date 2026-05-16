"""
test_control_server.launch.py  (rv2_server_control)

Loads ControlServerNode and FakeUnitreeApiNode together into a single
rclcpp_components multi-threaded container process.

FakeUnitreeApiNode mimics the on-robot Unitree sport API so the full stack
can be validated without a physical robot.  It receives every
/api/sport/request, logs it, and publishes a success /api/sport/response.

Using component_container_mt (multi-threaded) ensures that the output timer
callbacks (ControlServerNode, 20 Hz) and the request subscription callbacks
(FakeUnitreeApiNode) are dispatched to separate threads and never starve each
other — which can happen with a single-threaded container at high output rates.

Usage:
    ros2 launch rv2_server_control test_control_server.launch.py
    ros2 launch rv2_server_control test_control_server.launch.py \
        server_name:=my_server output_interval_ms:=100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rv2_server_control'),
            'config',
            'control_server.yaml',
        ]),
        description='Path to ControlServerNode ROS 2 parameter YAML file',
    )
    server_name_arg = DeclareLaunchArgument(
        'server_name',
        default_value='control_server',
        description='Name of the control server (parameter: server_name)',
    )
    output_interval_arg = DeclareLaunchArgument(
        'output_interval_ms',
        default_value='50',
        description='Output publish interval in milliseconds',
    )
    status_timer_interval_arg = DeclareLaunchArgument(
        'status_timer_interval_ms',
        default_value='1000',
        description='CSM status/disconnect timer period in milliseconds',
    )

    # ── Composable node container (multi-threaded) ────────────────────────────
    # component_container_mt is required so that:
    #   - ControlServerNode's 20 Hz output timer and
    #   - FakeUnitreeApiNode's /api/sport/request subscription callback
    # can be dispatched to separate executor threads without starving each other.
    container = ComposableNodeContainer(
        name='control_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='rv2_server_control',
                plugin='ControlServerNode',
                name='control_server',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {
                        'server_name':              LaunchConfiguration('server_name'),
                        'output_interval_ms':       LaunchConfiguration('output_interval_ms'),
                        'status_timer_interval_ms': LaunchConfiguration('status_timer_interval_ms'),
                    },
                ],
            ),
            ComposableNode(
                package='rv2_server_control',
                plugin='FakeUnitreeApiNode',
                name='fake_unitree_api',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        server_name_arg,
        output_interval_arg,
        status_timer_interval_arg,
        container,
    ])
