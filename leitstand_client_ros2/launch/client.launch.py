"""Launch the Leitstand client node with one argument: the path to robot.yaml."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", description="Path to robot.yaml"),
            Node(
                package="leitstand_client_ros2",
                executable="client",
                arguments=["--config", config],
                output="screen",
            ),
        ]
    )
