from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("indy7_dcp3"))
    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="",
                description="Indy7 DCP3 controller IP; must be provided explicitly",
            ),
            Node(
                package="indy7_dcp3",
                executable="indy7_dcp3_node",
                name="indy7_dcp3",
                parameters=[str(package_share / "config" / "indy7_dcp3.yaml"), {"robot_ip": robot_ip}],
                prefix=[FindExecutable(name="python3")],
                output="screen",
            ),
        ]
    )
