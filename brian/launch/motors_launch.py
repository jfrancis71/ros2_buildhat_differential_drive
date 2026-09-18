from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
            [FindPackageShare("brian"), "config", "controller_description.yaml"])
    differential_drive_node = Node(
        package="buildhat_differential_drive",
        executable="differential_drive_node",
        output="both",
        parameters=[robot_controllers],
    )
    nodes = [
        differential_drive_node
    ]

    return LaunchDescription(nodes)
