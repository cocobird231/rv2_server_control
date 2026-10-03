"""Run the R1 control server and a Unitree request logging node."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the hardware-free observation launch description."""
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rv2_server_control"),
                            "launch",
                            "control_server.launch.py",
                        ]
                    )
                ),
                launch_arguments={"start_master": "true"}.items(),
            ),
            Node(
                package="rv2_server_control",
                executable="fake_unitree_api_node",
                output="screen",
            ),
        ]
    )
