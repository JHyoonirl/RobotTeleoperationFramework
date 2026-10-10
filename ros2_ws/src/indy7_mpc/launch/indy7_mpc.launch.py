from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("indy7_mpc"))
    description_share = Path(get_package_share_directory("indy7_description"))
    urdf = description_share / "urdf" / "indy7.urdf"
    controllers = share / "config" / "indy7_controllers.yaml"
    config_file = LaunchConfiguration("config_file")
    simulate = LaunchConfiguration("simulate")
    initialize_target_from_current_state = LaunchConfiguration(
        "initialize_target_from_current_state"
    )
    joint_state_gui = LaunchConfiguration("joint_state_gui")
    plot = LaunchConfiguration("plot")
    rviz = LaunchConfiguration("rviz")
    def mpc_node(*, condition, remappings):
        return Node(
            package="indy7_mpc",
            executable="crocoddyl_mpc_node",
            name="crocoddyl_mpc",
            parameters=[
                config_file,
                {
                    "urdf_path": str(urdf),
                    "simulate": simulate,
                    "initialize_target_from_current_state": initialize_target_from_current_state,
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
                parameters=[{"robot_description": urdf.read_text()}],
            ),
            # This is the same ros2_control boundary used by the Indy
            # description stack.  The mock system lets the position controller
            # expose realistic joint_states without a separate Gazebo process.
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                parameters=[
                    {"robot_description": urdf.read_text()},
                    str(controllers),
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
                arguments=["-d", str(share / "config" / "indy7.rviz")],
                condition=IfCondition(rviz),
                output="log",
            ),
            Node(
                package="indy7_mpc",
                executable="mpc_plotter",
                name="mpc_plotter",
                condition=IfCondition(plot),
                parameters=[{"urdf_path": str(urdf)}],
                prefix=[FindExecutable(name="python3")],
                output="screen",
            ),
        ]
    )
