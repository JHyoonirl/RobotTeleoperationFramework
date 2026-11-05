from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robot_dcp3"))
    robot_ip = LaunchConfiguration("robot_ip")
    config_file = LaunchConfiguration("config_file")
    urdf_path = LaunchConfiguration("urdf_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="",
                description="Robot controller IP; must be provided explicitly",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "indy7_dcp3.yaml"),
                description="Robot-specific ROS 2 parameter YAML",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value=str(
                    Path(get_package_share_directory("robot_description"))
                    / "urdf"
                    / "indy7.urdf"
                ),
                description="Robot-specific URDF used for joint-limit checks",
            ),
            Node(
                package="robot_dcp3",
                executable="robot_dcp3_node",
                name="robot_dcp3",
                parameters=[
                    config_file,
                    {"robot_ip": robot_ip, "urdf_path": urdf_path},
                ],
                prefix=[FindExecutable(name="python3")],
                output="screen",
            ),
        ]
    )
