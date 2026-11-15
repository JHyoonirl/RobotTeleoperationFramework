"""EIR position MPC on both hands plus torso, or on one selected arm."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    arm = LaunchConfiguration("arm").perform(context)
    if arm not in {"left", "right", "both"}:
        raise ValueError("arm must be left, right or both")
    share = Path(get_package_share_directory("robot_mpc"))
    description = Path(get_package_share_directory("robot_description"))
    dual = arm == "both"
    joints = (["Joint_L0", "Joint_L1"] + [
        f"Joint_L{i}_{side}" for side in ("L", "R") for i in range(2, 9)
    ] if dual else [f"Joint_L{i}_{arm[0].upper()}" for i in range(2, 9)])
    config = LaunchConfiguration("config_file").perform(context)
    if not config:
        filename = {"both": "eir_dual_mpc.yaml", "left": "eir_left_mpc.yaml", "right": "eir_mpc.yaml"}[arm]
        config = str(share / "config" / filename)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/robot_mpc.launch.py")),
            launch_arguments={
                "urdf_path": str(description / "urdf/eir.urdf"),
                "config_file": config,
                "mpc_executable": "eir_mpc_node" if dual else "crocoddyl_mpc_node",
                "controllers_file": str(share / f"config/eir_{arm}_controllers.yaml"),
                "joint_names": str(joints),
                "simulate": "true",
                "initialize_target_from_current_state": "true",
                "mpc_start_paused": "true",
                "vive_mapping": "true",
                "vive_hand_frame": "right_hand_tcp" if dual else f"{arm}_hand_tcp",
                "teleop_gui": "false",
                "teleop_minimal_gui": "false",
                "rviz": LaunchConfiguration("rviz"),
                "rviz_config_file": str(share / "config/eir.rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "plot": LaunchConfiguration("plot"),
                "diagnostic_log_directory": LaunchConfiguration("diagnostic_log_directory"),
            }.items(),
        ),
        Node(package="robot_mpc", executable="eir_teleop_gui" if dual else "robot_vive_mapping_gui",
             name="eir_teleop_gui", parameters=[] if dual else [{"simulation_only": True}],
             condition=IfCondition(LaunchConfiguration("gui")), output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="both", description="both, left or right"),
        DeclareLaunchArgument("config_file", default_value="", description="Optional MPC parameter YAML"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=EnvironmentVariable(
            "ROBOT_EIR_RVIZ_CONFIG", default_value="/tmp/eir.rviz")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("plot", default_value="true"),
        DeclareLaunchArgument("diagnostic_log_directory", default_value="/tmp/eir_mpc_logs"),
        OpaqueFunction(function=_launch),
    ])
