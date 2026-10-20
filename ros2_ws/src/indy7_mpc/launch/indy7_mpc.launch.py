from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _launch_rviz(context, default_config):
    config = Path(LaunchConfiguration("rviz_config").perform(context)).expanduser()
    if not config.exists():
        contents = Path(default_config).read_bytes()
        config.parent.mkdir(parents=True, exist_ok=True)
        try:
            with config.open("xb") as stream:
                stream.write(contents)
        except FileExistsError:
            pass
    if not config.is_file():
        raise RuntimeError(f"RViz config must be a file: {config}")
    return [Node(package="rviz2", executable="rviz2", name="rviz2",
                 arguments=["-d", str(config)], output="screen")]


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
    diagnostic_log_directory = LaunchConfiguration("diagnostic_log_directory")
    joint_state_gui = LaunchConfiguration("joint_state_gui")
    plot = LaunchConfiguration("plot")
    rviz = LaunchConfiguration("rviz")
    teleop_gui = LaunchConfiguration("teleop_gui")
    teleop_minimal_gui = LaunchConfiguration("teleop_minimal_gui")
    def mpc_node(*, condition, remappings):
        return Node(
            package="indy7_mpc",
            executable="crocoddyl_mpc_node",
            name="crocoddyl_mpc",
            parameters=[
                config_file,
                {
                    "urdf_path": str(urdf),
                    "config_file_path": config_file,
                    "simulate": simulate,
                    "initialize_target_from_current_state": initialize_target_from_current_state,
                    "start_paused": LaunchConfiguration("mpc_start_paused"),
                    "enable_vive_follow": LaunchConfiguration("vive_mapping"),
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
            DeclareLaunchArgument(
                "rviz_config",
                default_value=EnvironmentVariable(
                    "INDY7_RVIZ_CONFIG", default_value=str(share / "config" / "indy7.rviz")),
                description="Writable RViz config reused across launches; seeded once if missing",
            ),
            DeclareLaunchArgument(
                "teleop_gui",
                default_value="false",
                description="Open sim-to-DCP3 mapping and teleoperation GUI",
            ),
            DeclareLaunchArgument(
                "teleop_minimal_gui",
                default_value="false",
                description="Open the compact simulation-to-DCP3 teleoperation GUI",
            ),
            DeclareLaunchArgument(
                "vive_mapping", default_value="false",
                description="Start the Vive hand pose mapper for the GUI mapping panel",
            ),
            DeclareLaunchArgument(
                "mpc_start_paused", default_value="false",
                description="Wait for F7 before running the simulation MPC",
            ),
            DeclareLaunchArgument(
                "vive_input_pose_topic", default_value="/tracker/right/pose",
                description="Vive hand PoseStamped input for the mapper",
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
            mpc_node(
                condition=UnlessCondition(simulate),
                remappings=[
                    ("joint_states", "/indy7_dcp3/joint_states"),
                    ("mpc/joint_position_cmd", "/indy7_dcp3/joint_position_cmd"),
                ],
            ),
            Node(
                package="indy7_mpc",
                executable="indy7_teleop_gui",
                name="indy7_teleop_gui",
                condition=IfCondition(teleop_gui),
                output="screen",
            ),
            Node(
                package="indy7_mpc",
                executable="indy7_teleop_minimal_gui",
                name="indy7_teleop_minimal_gui",
                condition=IfCondition(teleop_minimal_gui),
                output="screen",
            ),
            Node(
                package="indy7_mpc", executable="indy7_vive_mapper",
                name="indy7_vive_mapper",
                condition=IfCondition(LaunchConfiguration("vive_mapping")),
                parameters=[str(share / "config" / "indy7_vive_mapping.yaml"),
                            {"input_pose_topic": LaunchConfiguration("vive_input_pose_topic")}],
                additional_env={"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"},
                output="screen",
            ),
            OpaqueFunction(
                function=_launch_rviz,
                kwargs={"default_config": str(share / "config" / "indy7.rviz")},
                condition=IfCondition(rviz),
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
