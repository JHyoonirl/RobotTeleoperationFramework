from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robot_mpc"))
    description_share = Path(get_package_share_directory("robot_description"))
    urdf_path = LaunchConfiguration("urdf_path")
    controllers_file = LaunchConfiguration("controllers_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    joint_names = ParameterValue(
        LaunchConfiguration("joint_names"), value_type=list
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="cat"), " ", urdf_path]), value_type=str
    )
    config_file = LaunchConfiguration("config_file")
    simulate = LaunchConfiguration("simulate")
    initialize_target_from_current_state = LaunchConfiguration(
        "initialize_target_from_current_state"
    )
    diagnostic_log_directory = LaunchConfiguration("diagnostic_log_directory")
    joint_state_gui = LaunchConfiguration("joint_state_gui")
    plot = LaunchConfiguration("plot")
    rviz = LaunchConfiguration("rviz")
    def mpc_node(*, condition, remappings):
        return Node(
            package="robot_mpc",
            executable="crocoddyl_mpc_node",
            name="crocoddyl_mpc",
            parameters=[
                config_file,
                {
                    "urdf_path": urdf_path,
                    "joint_names": joint_names,
                    "config_file_path": config_file,
                    "simulate": simulate,
                    "initialize_target_from_current_state": initialize_target_from_current_state,
                    "diagnostic_log_directory": diagnostic_log_directory,
                },
            ],
            remappings=remappings,
            condition=condition,
            # Use the active venv's Python instead of an old shebang baked into
            # the generated ament_python entry point.
            prefix=[FindExecutable(name="python3")],
            output="screen",
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(share / "config" / "indy7_mpc.yaml"),
                description="ROS2 parameter YAML for this MPC experiment",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value=str(description_share / "urdf" / "indy7.urdf"),
                description="Robot-specific URDF",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=str(share / "config" / "indy7_controllers.yaml"),
                description="Robot-specific ros2_control controller configuration",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=str(share / "config" / "indy7.rviz"),
                description="Robot-specific RViz configuration",
            ),
            DeclareLaunchArgument(
                "joint_names",
                default_value="[joint0, joint1, joint2, joint3, joint4, joint5]",
                description="Joint names in the same order as the MPC configuration",
            ),
            DeclareLaunchArgument(
                "simulate",
                default_value="true",
                description="Start ros2_control position-command simulation",
            ),
            DeclareLaunchArgument(
                "initialize_target_from_current_state",
                default_value="false",
                description="Seed the initial target from the first joint-state message",
            ),
            DeclareLaunchArgument(
                "diagnostic_log_directory",
                default_value="",
                description="Directory for MPC run logs; empty uses the workspace logs folder",
            ),
            DeclareLaunchArgument(
                "joint_state_gui",
                default_value="false",
                description="Publish manually edited joint states (do not use with a real driver)",
            ),
            DeclareLaunchArgument(
                "plot",
                default_value="true",
                description="Open the live MPC matplotlib dashboard",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(joint_state_gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            # The mock system lets the position controller
            # expose realistic joint_states without a separate Gazebo process.
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                ],
                condition=IfCondition(simulate),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="joint_state_broadcaster_spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                condition=IfCondition(simulate),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="forward_position_controller_spawner",
                arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
                condition=IfCondition(simulate),
                output="screen",
            ),
            mpc_node(
                condition=IfCondition(simulate),
                remappings=[
                    ("mpc/joint_position_cmd", "/forward_position_controller/commands")
                ],
            ),
            mpc_node(condition=UnlessCondition(simulate), remappings=[]),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(rviz),
                output="screen",
            ),
            Node(
                package="robot_mpc",
                executable="mpc_plotter",
                name="mpc_plotter",
                condition=IfCondition(plot),
                parameters=[{"urdf_path": urdf_path, "joint_names": joint_names}],
                prefix=[FindExecutable(name="python3")],
                output="screen",
            ),
        ]
    )
