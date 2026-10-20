from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("indy7_mpc"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=str(share / "config" / "indy7_vive_mapping.yaml"),
            description="Vive to Indy7 hand TCP mapping parameters",
        ),
        DeclareLaunchArgument("input_pose_topic", default_value="/tracker/right/pose"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Open the standalone Vive mapping GUI"),
        Node(
            package="indy7_mpc", executable="indy7_vive_mapper", name="indy7_vive_mapper",
            parameters=[LaunchConfiguration("config_file"),
                        {"input_pose_topic": LaunchConfiguration("input_pose_topic")}],
            additional_env={"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"},
            output="screen",
        ),
        Node(
            package="indy7_mpc", executable="indy7_vive_mapping_gui",
            name="indy7_vive_mapping_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
            additional_env={"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"},
            output="screen",
        ),
    ])
