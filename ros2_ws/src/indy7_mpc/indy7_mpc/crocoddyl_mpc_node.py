#!/usr/bin/env python3
"""URDF-dynamics Crocoddyl MPC with a joint-position command boundary.

The optimizer predicts rigid-body motion with Pinocchio ABA and optimizes
joint torque internally.  The actuator-facing output remains the first
predicted absolute joint position, so the same ``q_cmd`` can be sent to
``forward_position_controller`` or an IndyDCP3 teleoperation adapter.
"""

from __future__ import annotations

import ast
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

from .cartesian_path import limit_cartesian_step, project_cartesian_command, segment_error
from .mpc_cost_config import apply_runtime_cost_settings, load_runtime_cost_settings
from .se3_dynamic_model import SE3DynamicActionModel
from .vive_simulation_control import ViveSimulationControl
from .workspace_floor import limit_height_step


class CrocoddylMPCNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("indy7_crocoddyl_mpc", **kwargs)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("config_file_path", "")
        self.declare_parameter("control_mode", "task_space")
        self.declare_parameter("task_frame", "tcp")
        self.declare_parameter("pose_frame", "world")
        self.declare_parameter("teleop_pose_topic", "teleop/target_pose")
        self.declare_parameter("command_pose_topic", "mpc/command_pose")
        # Common actuator boundary: radians, absolute joint positions.
        # The launch file remaps this to forward_position_controller in sim.
        self.declare_parameter("joint_command_topic", "mpc/joint_position_cmd")
        # Legacy joint-space target, used only when control_mode is "joint".
        self.declare_parameter("goal", [0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
        self.declare_parameter("nominal_posture", [0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
        self.declare_parameter("goal_position", [0.555628, -0.028394, 0.500983])
        self.declare_parameter("goal_orientation_xyzw", [-0.019703, 0.987570, 0.119061, 0.100709])
        self.declare_parameter("horizon", 20)
        self.declare_parameter("dt", 0.005)
        self.declare_parameter("max_solver_iterations", 10)
        self.declare_parameter("simulate", False)
        self.declare_parameter("initialize_target_from_current_state", False)
        self.declare_parameter("start_paused", False)
        self.declare_parameter("enable_vive_follow", False)
        self.declare_parameter("vive_hand_frame", "hand_tcp")
        self.declare_parameter("vive_hand_pose_topic", "/indy7/vive/hand_target_pose")
        self.declare_parameter("vive_mapping_status_topic", "/indy7/vive_mapping/status")
        self.declare_parameter("vive_run_service", "/indy7/sim/vive_mpc_run")
        self.declare_parameter("vive_run_status_topic", "/indy7/sim/vive_mpc_status")
        self.declare_parameter("vive_hold_topic", "/indy7/sim/vive_mpc_hold")
        self.declare_parameter("vive_hold_timeout", 0.35)
        self.declare_parameter("teleop_input_mode", "topic")
        self.declare_parameter("teleop_mode_service", "/indy7/sim/keyboard_mode")
        self.declare_parameter("keyboard_hand_pose_topic", "/indy7/keyboard/hand_target_pose")
        self.declare_parameter("hand_target_pose_topic", "/indy7/hand_target_pose")
        self.declare_parameter("hand_target_frame", "hand_target")
        self.declare_parameter("vive_input_timeout", 0.5)
        # Keep a deterministic local plant only for standalone smoke tests.
        # Normal simulation uses ros2_control and its joint_states topic.
        self.declare_parameter("internal_simulation", False)
        self.declare_parameter("command_velocity_limits", [1.0] * 6)
        self.declare_parameter("command_increment_limits", [0.02] * 6)
        self.declare_parameter("workspace_floor_enabled", True)
        self.declare_parameter("workspace_min_z", 0.10)
        self.declare_parameter("workspace_floor_margin", 0.001)
        self.declare_parameter("workspace_floor_cost_weight", 1.0e6)
        # Cartesian reference generator.  The MPC still solves in q/qdot, but
        # each shooting stage receives a point on a bounded-speed SE(3) path.
        self.declare_parameter("cartesian_max_speed", 0.05)
        self.declare_parameter("cartesian_max_angular_speed", 0.75)
        self.declare_parameter("cartesian_speed_margin", 0.5)
        self.declare_parameter("cartesian_min_duration", 0.10)
        self.declare_parameter("cartesian_path_tolerance", 0.005)
        self.declare_parameter(
            "cartesian_reference_topic", "mpc/cartesian_reference_pose"
        )
        # Cost weights are exposed as ROS parameters so they can be tuned from launch.
        self.declare_parameter("se3_position_cost_weight", 20.0)
        self.declare_parameter("se3_orientation_cost_weight", 5.0)
        self.declare_parameter("posture_cost_weight", 0.01)
        self.declare_parameter("velocity_cost_weight", 1.0)
        self.declare_parameter("torque_cost_weight", 0.0001)
        # Kept for old YAML files; the dynamic model no longer optimizes qddot.
        self.declare_parameter("acceleration_cost_weight", 0.05)
        self.declare_parameter("joint_limit_cost_weight", 10.0)
        self.declare_parameter("running_cost_weight", 1.0)
        self.declare_parameter("terminal_cost_weight", 10.0)
        self.declare_parameter("joint_limit_margin", 0.2)
        self.declare_parameter(
            "torque_limits", [431.97, 431.97, 197.23, 79.79, 79.79, 79.79]
        )
        self.declare_parameter("acceleration_limits", [20.0] * 6)
        self.declare_parameter("ik_max_iterations", 80)
        self.declare_parameter("ik_damping", 0.001)
        self.declare_parameter("ik_step", 0.1)
        self.declare_parameter("simulation_damping", 5.0)
        self.declare_parameter("simulation_acceleration_smoothing", 0.25)
        self.declare_parameter("diagnostic_logging", True)
        # Empty means <ros2_ws>/logs/indy7_mpc, next to src/install/build.
        self.declare_parameter("diagnostic_log_directory", "")
        self.declare_parameter("diagnostic_flush_period", 1.0)
        self.declare_parameter("diagnostic_log_prediction", True)
        self.declare_parameter("stationary_hold_position_tolerance", 2.0e-4)
        self.declare_parameter("stationary_hold_orientation_tolerance", 1.5e-3)

        try:
            import crocoddyl
            import pinocchio as pin
        except ImportError as exc:
            raise RuntimeError(
                "Crocoddyl/Pinocchio가 없습니다. ROS Jazzy 패키지를 설치하세요: "
                "ros-jazzy-crocoddyl ros-jazzy-pinocchio"
            ) from exc

        urdf_path = Path(str(self.get_parameter("urdf_path").value))
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF를 찾을 수 없습니다: {urdf_path}")
        config_file_path = str(self.get_parameter("config_file_path").value).strip()
        self.cost_config_path = Path(config_file_path) if config_file_path else None
        self.cost_config_mtime_ns = None
        if self.cost_config_path is not None:
            try:
                self.cost_config_mtime_ns = self.cost_config_path.stat().st_mtime_ns
            except OSError as exc:
                self.get_logger().warning(
                    f"MPC cost YAML hot reload is unavailable: {exc}"
                )

        self.pin = pin
        self.crocoddyl = crocoddyl
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        if self.model.nq != 6 or self.model.nv != 6:
            raise RuntimeError(f"Indy7 6-DOF 모델이 아닙니다: nq={self.model.nq}, nv={self.model.nv}")

        self.joint_names = [f"joint{i}" for i in range(6)]
        self.control_mode = str(self.get_parameter("control_mode").value).strip().lower()
        if self.control_mode not in {"task_space", "joint"}:
            raise ValueError("control_mode는 task_space 또는 joint여야 합니다.")
        self.simulate = self._as_bool(self.get_parameter("simulate").value)
        self.vive_follow_enabled = self.simulate and self._as_bool(
            self.get_parameter("enable_vive_follow").value
        )
        # Vive targets describe the hand itself. Evaluate tracking, Jacobians
        # and Cartesian path limits at that point, including during rotation.
        self.task_frame = str(self.get_parameter(
            "vive_hand_frame" if self.vive_follow_enabled else "task_frame"
        ).value)
        self.pose_frame = str(self.get_parameter("pose_frame").value)
        self.teleop_pose_topic = str(self.get_parameter("teleop_pose_topic").value).strip()
        self.command_pose_topic = str(self.get_parameter("command_pose_topic").value).strip()
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value).strip()
        if not self.task_frame or not self.pose_frame or not self.command_pose_topic or not self.joint_command_topic:
            raise ValueError("task_frame, pose_frame, command_pose_topic, joint_command_topic은 비어 있을 수 없습니다.")
        self.frame_id = self.model.getFrameId(self.task_frame)
        if self.frame_id >= len(self.model.frames):
            raise ValueError(f"URDF frame을 찾을 수 없습니다: {self.task_frame}")
        self.workspace_floor_enabled = self._as_bool(self.get_parameter("workspace_floor_enabled").value)
        self.workspace_min_z = float(self.get_parameter("workspace_min_z").value)
        self.workspace_floor_margin = float(self.get_parameter("workspace_floor_margin").value)
        self.workspace_floor_cost_weight = float(self.get_parameter("workspace_floor_cost_weight").value)
        if (not np.isfinite(self.workspace_min_z)
                or not np.isfinite(self.workspace_floor_margin) or self.workspace_floor_margin <= 0.0
                or not np.isfinite(self.workspace_floor_cost_weight) or self.workspace_floor_cost_weight <= 0.0):
            raise ValueError("workspace_min_z must be finite; floor margin/weight must be positive and finite")
        if self.workspace_floor_enabled and self.pose_frame != "world":
            raise ValueError("The workspace floor requires pose_frame='world' (the URDF root)")
        if self.vive_follow_enabled:
            self.set_parameters([Parameter("task_frame", value=self.task_frame)])

        self.goal = self._parse_vector(self.get_parameter("goal").value, 6)
        self.nominal_posture = self._parse_vector(
            self.get_parameter("nominal_posture").value, 6
        )
        self.goal_position = self._parse_vector(self.get_parameter("goal_position").value, 3)
        self.goal_orientation = self._parse_quaternion(
            self.get_parameter("goal_orientation_xyzw").value
        )
        self.dt = float(self.get_parameter("dt").value)
        self.horizon = int(self.get_parameter("horizon").value)
        self.max_solver_iterations = int(self.get_parameter("max_solver_iterations").value)
        self.command_velocity_limits = self._parse_vector(
            self.get_parameter("command_velocity_limits").value, 6
        )
        self.command_increment_limits = self._parse_vector(
            self.get_parameter("command_increment_limits").value, 6
        )
        self.cartesian_max_speed = float(
            self.get_parameter("cartesian_max_speed").value
        )
        self.cartesian_max_angular_speed = float(
            self.get_parameter("cartesian_max_angular_speed").value
        )
        self.cartesian_speed_margin = float(
            self.get_parameter("cartesian_speed_margin").value
        )
        self.cartesian_min_duration = float(
            self.get_parameter("cartesian_min_duration").value
        )
        self.cartesian_path_tolerance = float(
            self.get_parameter("cartesian_path_tolerance").value
        )
        self.cartesian_reference_topic = str(
            self.get_parameter("cartesian_reference_topic").value
        ).strip()
        # Never allow a user parameter to exceed the URDF's physical velocity
        # limit; a lower configured value remains available for conservative
        # sim-to-real tuning.
        self.command_velocity_limits = np.minimum(
            self.command_velocity_limits, np.asarray(self.model.velocityLimit, dtype=float)
        )
        self.se3_position_cost_weight = float(
            self.get_parameter("se3_position_cost_weight").value
        )
        self.se3_orientation_cost_weight = float(
            self.get_parameter("se3_orientation_cost_weight").value
        )
        self.posture_cost_weight = float(self.get_parameter("posture_cost_weight").value)
        self.velocity_cost_weight = float(self.get_parameter("velocity_cost_weight").value)
        self.torque_cost_weight = float(self.get_parameter("torque_cost_weight").value)
        self.acceleration_cost_weight = float(
            self.get_parameter("acceleration_cost_weight").value
        )
        self.joint_limit_cost_weight = float(
            self.get_parameter("joint_limit_cost_weight").value
        )
        self.running_cost_weight = float(self.get_parameter("running_cost_weight").value)
        self.terminal_cost_weight = float(self.get_parameter("terminal_cost_weight").value)
        self.joint_limit_margin = float(self.get_parameter("joint_limit_margin").value)
        self.acceleration_limits = self._parse_vector(
            self.get_parameter("acceleration_limits").value, 6
        )
        self.torque_limits = np.minimum(
            self._parse_vector(self.get_parameter("torque_limits").value, 6),
            np.asarray(self.model.effortLimit, dtype=float),
        )
        self.ik_max_iterations = int(self.get_parameter("ik_max_iterations").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)
        self.ik_step = float(self.get_parameter("ik_step").value)
        self.simulation_damping = float(self.get_parameter("simulation_damping").value)
        self.simulation_acceleration_smoothing = float(
            self.get_parameter("simulation_acceleration_smoothing").value
        )
        self.diagnostic_logging = self._as_bool(
            self.get_parameter("diagnostic_logging").value
        )
        self.diagnostic_log_directory = str(
            self.get_parameter("diagnostic_log_directory").value
        )
        self.diagnostic_flush_period = float(
            self.get_parameter("diagnostic_flush_period").value
        )
        self.diagnostic_log_prediction = self._as_bool(
            self.get_parameter("diagnostic_log_prediction").value
        )
        self.stationary_hold_position_tolerance = float(
            self.get_parameter("stationary_hold_position_tolerance").value
        )
        self.stationary_hold_orientation_tolerance = float(
            self.get_parameter("stationary_hold_orientation_tolerance").value
        )
        self.internal_simulation = self._as_bool(
            self.get_parameter("internal_simulation").value
        )
        self.initialize_target_from_current_state = self._as_bool(
            self.get_parameter("initialize_target_from_current_state").value
        )
        self.initial_target_state_initialized = not self.initialize_target_from_current_state
        self.execution_enabled = not self._as_bool(self.get_parameter("start_paused").value)
        self._paused_alignment_target = None
        self._paused_alignment_deadline = 0.0
        self.vive_control = None
        self._last_paused_publish = -float("inf")
        cost_weights = (
            self.se3_position_cost_weight,
            self.se3_orientation_cost_weight,
            self.posture_cost_weight,
            self.velocity_cost_weight,
            self.torque_cost_weight,
            self.joint_limit_cost_weight,
            self.running_cost_weight,
            self.terminal_cost_weight,
        )
        if self.dt <= 0 or self.horizon < 1 or self.max_solver_iterations < 1:
            raise ValueError("dt는 양수, horizon/iterations는 1 이상이어야 합니다.")
        if np.any(self.command_velocity_limits <= 0.0):
            raise ValueError("command_velocity_limits는 양수인 6개 값이어야 합니다.")
        if np.any(self.command_increment_limits <= 0.0):
            raise ValueError("command_increment_limits는 양수인 6개 값이어야 합니다.")
        if np.any(self.torque_limits <= 0.0) or not np.all(np.isfinite(self.torque_limits)):
            raise ValueError("torque_limits는 양수인 6개 유한값이어야 합니다.")
        if (
            not np.isfinite(self.cartesian_max_speed)
            or self.cartesian_max_speed <= 0.0
            or not np.isfinite(self.cartesian_max_angular_speed)
            or self.cartesian_max_angular_speed <= 0.0
            or not np.isfinite(self.cartesian_speed_margin)
            or self.cartesian_speed_margin <= 0.0
            or self.cartesian_speed_margin > 1.0
            or not np.isfinite(self.cartesian_min_duration)
            or self.cartesian_min_duration <= 0.0
            or not self.cartesian_reference_topic
        ):
            raise ValueError(
                "cartesian_max_speed/cartesian_max_angular_speed/min_duration은 양수, "
                "cartesian_speed_margin은 (0, 1] 범위여야 합니다."
            )
        if not np.all(np.isfinite(cost_weights)) or any(weight <= 0.0 for weight in cost_weights):
            raise ValueError("cost weight는 모두 양수인 유한값이어야 합니다.")
        if not np.isfinite(self.cartesian_path_tolerance) or self.cartesian_path_tolerance <= 0.0:
            raise ValueError("cartesian_path_tolerance은 양수인 유한값이어야 합니다 [m].")
        # The continuous command-step bound assumes bounded revolute joints.
        if (self.control_mode == "task_space" or self.workspace_floor_enabled) and any(
            not joint.shortname().startswith("JointModelR") or joint.nq != 1 or joint.nv != 1
            for joint in list(self.model.joints)[1:]
        ):
            raise ValueError("Cartesian path guard requires bounded revolute joints")
        self.cartesian_reach_bound = sum(
            float(np.linalg.norm(placement.translation))
            for placement in self.model.jointPlacements
        ) + float(np.linalg.norm(self.model.frames[self.frame_id].placement.translation))
        if self.simulation_damping < 0.0 or not np.isfinite(self.simulation_damping):
            raise ValueError("simulation_damping은 0 이상인 유한값이어야 합니다.")
        if not 0.0 < self.simulation_acceleration_smoothing <= 1.0:
            raise ValueError("simulation_acceleration_smoothing은 (0, 1] 범위여야 합니다.")
        if self.diagnostic_flush_period <= 0.0 or not np.isfinite(self.diagnostic_flush_period):
            raise ValueError("diagnostic_flush_period는 양수인 유한값이어야 합니다.")
        if (
            self.stationary_hold_position_tolerance <= 0.0
            or self.stationary_hold_orientation_tolerance <= 0.0
            or not np.all(
                np.isfinite(
                    [
                        self.stationary_hold_position_tolerance,
                        self.stationary_hold_orientation_tolerance,
                    ]
                )
            )
        ):
            raise ValueError("stationary hold tolerance는 양수인 유한값이어야 합니다.")
        if self.joint_limit_margin <= 0.0 or not np.isfinite(self.joint_limit_margin):
            raise ValueError("joint_limit_margin은 양수인 유한값이어야 합니다.")
        if self.ik_max_iterations < 1 or self.ik_damping <= 0.0 or self.ik_step <= 0.0:
            raise ValueError("ik_max_iterations는 1 이상, ik_damping/ik_step은 양수여야 합니다.")

        self.state = crocoddyl.StateMultibody(self.model)
        self.target_pose = pin.SE3(pin.Quaternion(self.goal_orientation).matrix(), self.goal_position)
        self.sim_q = np.array([0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
        self.sim_v = np.zeros(6)
        self.sim_acceleration = np.zeros(6)
        self.pose_data = self.model.createData()
        self.ik_data = self.model.createData()
        # The line planner is deliberately anchored once per motion instead
        # of resetting its start on every teleoperation message.  This keeps
        # a stream of small target updates on one Cartesian line.
        self.cartesian_path_start_pose = self._tcp_pose(self.sim_q).copy()
        self.cartesian_path_start_time = time.monotonic()
        self.cartesian_path_duration = self.cartesian_min_duration
        self.cartesian_path_speed = self.cartesian_max_speed
        self.cartesian_path_progress = 0.0
        self.cartesian_path_deviation = 0.0
        self.cartesian_command_scale = 1.0
        self.cartesian_path_guard = ""
        self.workspace_floor_guard = ""
        self.cartesian_path_initialized_from_state = False
        self.cartesian_reference_poses = [
            self._optimization_target_pose() for _ in range(self.horizon + 1)
        ]
        if self.control_mode == "task_space":
            self.xref = np.r_[self._solve_ik(self._optimization_target_pose(), self.sim_q), np.zeros(self.model.nv)]
        else:
            self.xref = np.r_[self.goal, np.zeros(self.model.nv)]
        self.problem, self.solver = self._make_problem()
        self.xs = [self.xref.copy() for _ in range(self.horizon + 1)]
        self.us = [np.zeros(self.model.nv) for _ in range(self.horizon)]
        self.latest_state: np.ndarray | None = None
        self.warned_no_state = False
        self.last_state_update_monotonic = float("nan")
        self.last_state_stamp_s = float("nan")
        self._diagnostic_file = None
        self._diagnostic_path: Path | None = None
        self._diagnostic_last_flush = time.monotonic()
        self.last_target_acceleration = np.zeros(self.model.nv)
        self.last_applied_acceleration = np.zeros(self.model.nv)
        self.last_dynamics: dict = {}
        self.last_q_cmd: np.ndarray | None = None
        self.stationary_hold_active = False
        self.stationary_hold_target_count = -1
        self.stationary_hold_q: np.ndarray | None = None
        self.stationary_hold_cost = float("nan")

        # Both ros2_control simulation and the real driver expose joint_states.
        # A solve can exceed dt; queued feedback would make the controller chase old states.
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 1)
        self.create_subscription(
            JointState,
            "/indy7/sim/sync_joint_target",
            self._on_sim_sync_joint_target,
            10,
        )
        self.state_pub = (
            self.create_publisher(JointState, "joint_states", 10)
            if self.internal_simulation
            else None
        )
        self.command_pose_pub = self.create_publisher(PoseStamped, self.command_pose_topic, 10)
        self.cartesian_reference_pub = self.create_publisher(
            PoseStamped, self.cartesian_reference_topic, 10
        )
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, self.joint_command_topic, 1
        )
        self.desired_pose_pub = self.create_publisher(PoseStamped, "mpc/desired_pose", 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, "mpc/current_pose", 10)
        self.qref_pub = self.create_publisher(JointState, "mpc/qref", 10)
        if self.teleop_pose_topic:
            self.create_subscription(PoseStamped, self.teleop_pose_topic, self._on_target_pose, 10)
        self.cost_pub = self.create_publisher(Float64, "mpc/cost", 10)
        self.task_error_pub = self.create_publisher(Float64, "mpc/task_error", 10)
        self.position_error_pub = self.create_publisher(Float64, "mpc/position_error", 10)
        self.orientation_error_pub = self.create_publisher(Float64, "mpc/orientation_error", 10)
        self.solve_time_pub = self.create_publisher(Float64, "mpc/solve_time_ms", 10)
        self.solve_iterations_pub = self.create_publisher(Float64, "mpc/solver_iterations", 10)
        self.solve_success_pub = self.create_publisher(Float64, "mpc/solver_success", 10)
        self.status_pub = self.create_publisher(String, "mpc/status", 10)
        self.sim_sync_ack_pub = self.create_publisher(
            String, "/indy7/sim/sync_joint_target/status", 10
        )
        self.control_norm_pub = self.create_publisher(Float64, "mpc/control_norm", 10)
        self.cost_term_pubs = {
            name: self.create_publisher(Float64, f"mpc/cost/{name}", 10)
            for name in (
                "se3_tracking",
                "cartesian_path",
                "workspace_floor",
                "posture",
                "velocity",
                "torque",
                "joint_limit",
            )
        }
        self.terminal_cost_pub = self.create_publisher(Float64, "mpc/terminal_cost", 10)
        self.solve_count = 0
        self.target_update_count = 0
        self.last_target_update_monotonic = float("nan")
        self.last_target_stamp_s = float("nan")
        self.previous_target_pose = self.target_pose.copy()
        self.last_target_update_dt = float("nan")
        self.last_target_delta_rotation_world = np.zeros(3)
        self.last_target_angular_velocity_world = np.zeros(3)
        if self.vive_follow_enabled:
            self.vive_control = ViveSimulationControl(self)
        self._diagnostic_start_monotonic = time.monotonic()
        self._open_diagnostic_log(urdf_path)
        self.create_timer(self.dt, self._solve_once)
        if self.cost_config_path is not None:
            self.create_timer(0.25, self._reload_cost_config_if_changed)
            self.get_logger().info(
                f"Watching MPC cost YAML for edits: {self.cost_config_path}"
            )
        self.get_logger().info(
            "SE(3) dynamics MPC ready (%s, mode=%s): q_cmd=%s, teleop=%s, frame=%s"
            % (
                "internal simulation" if self.internal_simulation else "joint_states",
                self.control_mode,
                self.joint_command_topic,
                self.teleop_pose_topic or "disabled",
                self.task_frame,
            )
        )

    @staticmethod
    def _parse_vector(value: object, size: int) -> np.ndarray:
        if isinstance(value, str):
            try:
                parsed = ast.literal_eval(value)
            except (SyntaxError, ValueError) as exc:
                raise ValueError(f"벡터 형식이 잘못되었습니다: {value}") from exc
        else:
            parsed = value
        vector = np.asarray(parsed, dtype=float)
        if vector.shape != (size,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{size}개 유한한 rad 값이 필요합니다: {value}")
        return vector

    @staticmethod
    def _as_bool(value: object) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def _open_diagnostic_log(self, urdf_path: Path) -> None:
        """Create one self-contained JSONL trace and a parameter snapshot."""
        if not self.diagnostic_logging:
            return
        try:
            requested = self.diagnostic_log_directory.strip()
            if requested:
                root = Path(requested).expanduser()
            else:
                workspace = next(
                    (
                        parent
                        for parent in (urdf_path.parent, *urdf_path.parents)
                        if (parent / "src").is_dir() and (parent / "install").is_dir()
                    ),
                    Path.cwd(),
                )
                root = workspace / "logs" / "indy7_mpc"
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            run_dir = root / stamp
            run_dir.mkdir(parents=True, exist_ok=False)
            self._diagnostic_path = run_dir / "trace.jsonl"
            self._diagnostic_file = self._diagnostic_path.open(
                "w", encoding="utf-8", buffering=8192
            )
            parameter_names = (
                "urdf_path",
                "control_mode",
                "task_frame",
                "pose_frame",
                "teleop_pose_topic",
                "command_pose_topic",
                "joint_command_topic",
                "goal",
                "nominal_posture",
                "goal_position",
                "goal_orientation_xyzw",
                "horizon",
                "dt",
                "max_solver_iterations",
                "simulate",
                "initialize_target_from_current_state",
                "internal_simulation",
                "command_velocity_limits",
                "command_increment_limits",
                "workspace_floor_enabled",
                "workspace_min_z",
                "workspace_floor_margin",
                "workspace_floor_cost_weight",
                "cartesian_max_speed",
                "cartesian_max_angular_speed",
                "cartesian_speed_margin",
                "cartesian_min_duration",
                "cartesian_path_tolerance",
                "cartesian_reference_topic",
                "se3_position_cost_weight",
                "se3_orientation_cost_weight",
                "posture_cost_weight",
                "velocity_cost_weight",
                "torque_cost_weight",
                "acceleration_cost_weight",
                "joint_limit_cost_weight",
                "running_cost_weight",
                "terminal_cost_weight",
                "joint_limit_margin",
                "torque_limits",
                "acceleration_limits",
                "ik_max_iterations",
                "ik_damping",
                "ik_step",
                "simulation_damping",
                "simulation_acceleration_smoothing",
                "diagnostic_logging",
                "diagnostic_log_directory",
                "diagnostic_flush_period",
                "diagnostic_log_prediction",
                "stationary_hold_position_tolerance",
                "stationary_hold_orientation_tolerance",
            )
            parameters = {
                name: self.get_parameter(name).value for name in parameter_names
            }
            parameters["urdf_path"] = str(urdf_path)
            parameters["resolved_log_directory"] = str(root)
            parameters["effective_torque_limits"] = self.torque_limits
            parameters["urdf_effort_limits"] = np.asarray(self.model.effortLimit, dtype=float)
            (run_dir / "parameters.json").write_text(
                json.dumps(self._json_safe(parameters), indent=2, ensure_ascii=False)
                + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"MPC diagnostic log: {self._diagnostic_path}")
        except (OSError, TypeError, ValueError) as exc:
            if self._diagnostic_file is not None:
                self._diagnostic_file.close()
            self.diagnostic_logging = False
            self._diagnostic_file = None
            self._diagnostic_path = None
            self.get_logger().error(f"진단 로그를 열 수 없습니다: {exc}")

    @staticmethod
    def _json_safe(value):
        if isinstance(value, np.ndarray):
            return CrocoddylMPCNode._json_safe(value.tolist())
        if isinstance(value, (np.floating, np.integer, np.bool_)):
            value = value.item()
        if isinstance(value, float):
            return value if np.isfinite(value) else None
        if isinstance(value, dict):
            return {str(key): CrocoddylMPCNode._json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [CrocoddylMPCNode._json_safe(item) for item in value]
        return value

    def _pose_record(self, pose) -> dict[str, list[float]]:
        quaternion = self.pin.Quaternion(pose.rotation).coeffs()
        return {
            "position": [float(value) for value in pose.translation],
            "orientation_xyzw": [float(value) for value in quaternion],
        }

    def _joint_limit_record(self, q: np.ndarray) -> dict[str, list[float]]:
        lower = np.asarray(self.model.lowerPositionLimit, dtype=float)
        upper = np.asarray(self.model.upperPositionLimit, dtype=float)
        soft_lower = lower + self.joint_limit_margin
        soft_upper = upper - self.joint_limit_margin
        q = np.asarray(q, dtype=float)
        return {
            "lower": lower,
            "upper": upper,
            "hard_margin": np.minimum(q - lower, upper - q),
            "soft_margin": np.minimum(q - soft_lower, soft_upper - q),
            "soft_violation": np.maximum(soft_lower - q, 0.0)
            + np.maximum(q - soft_upper, 0.0),
        }

    def _jacobian_record(self, q: np.ndarray, pose) -> dict[str, object]:
        self.pin.computeJointJacobians(self.model, self.pose_data, q)
        self.pin.updateFramePlacements(self.model, self.pose_data)
        jacobian = np.asarray(
            self.pin.getFrameJacobian(
                self.model, self.pose_data, self.frame_id, self.pin.LOCAL
            ),
            dtype=float,
        ).copy()
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(singular_values[-1])
        condition = (
            float(singular_values[0] / sigma_min)
            if sigma_min > 1.0e-12
            else float("inf")
        )
        world_z_local = np.asarray(pose.rotation.T @ np.array([0.0, 0.0, 1.0]))
        world_z_twist = np.r_[np.zeros(3), world_z_local]
        world_z_qdot = np.linalg.pinv(jacobian) @ world_z_twist
        return {
            "local": jacobian,
            "singular_values": singular_values,
            "sigma_min": sigma_min,
            "condition_number": condition,
            "world_z_axis_in_tcp": world_z_local,
            "unit_world_z_yaw_qdot": world_z_qdot,
            "unit_world_z_yaw_qdot_norm": float(np.linalg.norm(world_z_qdot)),
        }

    def _write_diagnostic(
        self,
        *,
        status: str,
        state: np.ndarray,
        target,
        solve_time_ms: float = float("nan"),
        solver_success: bool = False,
        solver_iterations: int = 0,
        solver_cost: float = float("nan"),
        solver_error: str = "",
        stationary_optimum: bool = False,
        prediction_guard: str = "",
        raw_control: np.ndarray | None = None,
        q_cmd: np.ndarray | None = None,
        predicted_next_state: np.ndarray | None = None,
        applied_state: np.ndarray | None = None,
        cost_terms: dict | None = None,
        terminal_cost_terms: dict | None = None,
        current_pose=None,
        pose_error: np.ndarray | None = None,
        jacobian: dict | None = None,
        dynamics: dict | None = None,
    ) -> None:
        if self._diagnostic_file is None:
            return
        try:
            state = np.asarray(state, dtype=float)
            q = state[: self.model.nq]
            velocity = state[self.model.nq : self.model.nq + self.model.nv]
            if current_pose is None:
                current_pose = self._tcp_pose(q).copy()
            if pose_error is None:
                pose_error = self._pose_error(q, target, self.pose_data)
            if jacobian is None:
                jacobian = self._jacobian_record(q, current_pose)
            if raw_control is None:
                raw_control = np.full(self.model.nv, np.nan)
            if q_cmd is None:
                q_cmd = np.full(self.model.nq, np.nan)
            if cost_terms is None:
                cost_terms = {}
            if terminal_cost_terms is None:
                terminal_cost_terms = {}
            if dynamics is None:
                dynamics = {}
            qref = np.asarray(self.xref[: self.model.nq], dtype=float)
            angular_distance = float(
                np.linalg.norm(
                    self.pin.log3(
                        self.cartesian_path_start_pose.rotation.T
                        @ self.target_pose.rotation
                    )
                )
            )
            record = {
                "schema_version": 3,
                "elapsed_s": time.monotonic() - self._diagnostic_start_monotonic,
                "ros_time_s": self.get_clock().now().nanoseconds * 1.0e-9,
                "cycle": self.solve_count,
                "status": status,
                "state": {
                    "q": q,
                    "velocity": velocity,
                    "tcp_pose": self._pose_record(current_pose),
                },
                "state_input": {
                    "age_s": time.monotonic() - self.last_state_update_monotonic
                    if np.isfinite(self.last_state_update_monotonic)
                    else float("nan"),
                    "message_stamp_s": self.last_state_stamp_s,
                    "message_age_s": self.get_clock().now().nanoseconds * 1.0e-9
                    - self.last_state_stamp_s
                    if np.isfinite(self.last_state_stamp_s)
                    else float("nan"),
                },
                "target_pose": self._pose_record(target),
                "target_input": {
                    "update_count": self.target_update_count,
                    "age_s": time.monotonic() - self.last_target_update_monotonic
                    if np.isfinite(self.last_target_update_monotonic)
                    else float("nan"),
                    "message_stamp_s": self.last_target_stamp_s,
                    "update_dt_s": self.last_target_update_dt,
                    "delta_rotation_world_rad": self.last_target_delta_rotation_world,
                    "angular_velocity_world_rad_s": self.last_target_angular_velocity_world,
                },
                "cartesian_path": {
                    "start_pose": self._pose_record(self.cartesian_path_start_pose),
                    "goal_pose": self._pose_record(self.target_pose),
                    "reference_pose": self._pose_record(self.cartesian_reference_poses[0]),
                    "reference_horizon": [
                        self._pose_record(reference)
                        for reference in self.cartesian_reference_poses
                    ],
                    "speed_limit_m_s": self.cartesian_path_speed,
                    "angular_speed_limit_rad_s": self.cartesian_max_angular_speed,
                    "angular_distance_rad": angular_distance,
                    "duration_s": self.cartesian_path_duration,
                    "progress": self.cartesian_path_progress,
                    "deviation_m": self.cartesian_path_deviation,
                    "tolerance_m": self.cartesian_path_tolerance,
                    "command_scale": self.cartesian_command_scale,
                    "guard": self.cartesian_path_guard,
                },
                "pose_error_local": pose_error,
                "position_error_world": np.asarray(target.translation - current_pose.translation),
                "orientation_error_world": np.asarray(current_pose.rotation @ pose_error[3:]),
                "position_error_norm_m": float(np.linalg.norm(pose_error[:3])),
                "orientation_error_norm_rad": float(np.linalg.norm(pose_error[3:])),
                "qref": qref,
                "q_cmd": q_cmd,
                "q_minus_qref": q - qref,
                "joint_limits": self._joint_limit_record(q),
                "solver": {
                    "success": bool(solver_success),
                    "iterations": int(solver_iterations),
                    "solve_time_ms": float(solve_time_ms),
                    "cost": float(solver_cost),
                    "error": solver_error,
                    "stop": getattr(self.solver, "stop", ""),
                    "th_stop": getattr(self.solver, "th_stop", float("nan")),
                    "th_grad": getattr(self.solver, "th_grad", float("nan")),
                    "dV": getattr(self.solver, "dV", []),
                    "state_regularization": getattr(self.solver, "preg", float("nan")),
                    "regularization_delta": getattr(self.solver, "dreg", float("nan")),
                    "stationary_optimum": bool(stationary_optimum),
                    "prediction_guard": prediction_guard,
                },
                "control": {
                    "raw_torque": raw_control,
                    # Legacy name retained so old analysis scripts still load.
                    "raw_qddot": raw_control,
                    "raw_norm": float(np.linalg.norm(raw_control))
                    if np.all(np.isfinite(raw_control))
                    else float("nan"),
                    "command_boundary": "joint_position",
                    "applied_torque": None,
                    "target_qddot_after_clip": self.last_target_acceleration,
                    "applied_qddot": self.last_applied_acceleration,
                    "applied_norm": float(np.linalg.norm(self.last_applied_acceleration)),
                },
                "cost_terms": cost_terms,
                "terminal_cost_terms": terminal_cost_terms,
                "dynamics": dynamics,
                "jacobian": jacobian,
                "predicted_next_state": predicted_next_state,
                "applied_state": applied_state,
            }
            if self.diagnostic_log_prediction:
                record["prediction"] = {
                    "xs": [np.asarray(item, dtype=float) for item in self.xs],
                    "us": [np.asarray(item, dtype=float) for item in self.us],
                }
            self._diagnostic_file.write(
                json.dumps(self._json_safe(record), separators=(",", ":")) + "\n"
            )
            if time.monotonic() - self._diagnostic_last_flush >= self.diagnostic_flush_period:
                self._diagnostic_file.flush()
                self._diagnostic_last_flush = time.monotonic()
        except (OSError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f"진단 로그 기록 실패: {exc}")
            self._diagnostic_file.close()
            self._diagnostic_file = None

    @staticmethod
    def _parse_quaternion(value: object) -> np.ndarray:
        quaternion = CrocoddylMPCNode._parse_vector(value, 4)
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-12:
            raise ValueError("goal_orientation_xyzw는 0이 아닌 quaternion이어야 합니다.")
        return quaternion / norm

    def _pose_error(self, q: np.ndarray, target: object, data) -> np.ndarray:
        """Return a finite local SE(3) error [m, rad] for the IK outer loop."""
        self.pin.framesForwardKinematics(self.model, data, q)
        current = data.oMf[self.frame_id]
        relative = current.inverse() * target
        quaternion = self.pin.Quaternion(relative.rotation).coeffs().copy()
        if quaternion[3] < 0.0:
            quaternion = -quaternion
        sine_half = float(np.linalg.norm(quaternion[:3]))
        if sine_half < 1.0e-12:
            rotation_error = np.zeros(3)
        else:
            angle = 2.0 * np.arctan2(sine_half, abs(float(quaternion[3])))
            rotation_error = angle * quaternion[:3] / sine_half
        return np.r_[relative.translation, rotation_error]

    def _solve_ik(self, target: object, seed: np.ndarray) -> np.ndarray:
        """Damped least-squares SE(3) IK for the pose-level MPC interface."""
        q = np.asarray(seed, dtype=float).copy()
        weights = np.sqrt(
            np.r_[
                np.full(3, self.se3_position_cost_weight),
                np.full(3, self.se3_orientation_cost_weight),
            ]
        )
        for _ in range(self.ik_max_iterations):
            error = self._pose_error(q, target, self.ik_data)
            if float(np.linalg.norm(error)) < 1.0e-5:
                break
            self.pin.computeJointJacobians(self.model, self.ik_data, q)
            self.pin.updateFramePlacements(self.model, self.ik_data)
            jacobian = self.pin.getFrameJacobian(
                self.model, self.ik_data, self.frame_id, self.pin.LOCAL
            )
            weighted_jacobian = weights[:, None] * jacobian
            weighted_error = weights * error
            damping = self.ik_damping**2 * np.eye(6)
            dq = weighted_jacobian.T @ np.linalg.solve(
                weighted_jacobian @ weighted_jacobian.T + damping, weighted_error
            )
            q = self.pin.integrate(self.model, q, self.ik_step * dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        return q

    def _make_problem(self):
        c = self.crocoddyl
        nominal_posture = self.goal if self.control_mode == "joint" else self.nominal_posture
        task_weights = {
            "se3_position": self.se3_position_cost_weight
            if self.control_mode == "task_space"
            else 0.0,
            "se3_orientation": self.se3_orientation_cost_weight
            if self.control_mode == "task_space"
            else 0.0,
            "posture": self.posture_cost_weight,
            "velocity": self.velocity_cost_weight,
            "torque": self.torque_cost_weight,
            "joint_limit": self.joint_limit_cost_weight,
        }
        # Each horizon stage owns its own target.  Reusing one action model
        # would make every stage chase the same fixed pose and produces the
        # curved q-space motion that the Cartesian line reference is meant to
        # remove.
        self.running_actions = []
        running_models = []
        for reference_pose in self.cartesian_reference_poses[:-1]:
            action = SE3DynamicActionModel(
                crocoddyl=c,
                pin=self.pin,
                model=self.model,
                state=self.state,
                frame_id=self.frame_id,
                target=reference_pose,
                nominal_posture=nominal_posture,
                weights=task_weights,
                terminal_scale=self.running_cost_weight,
                joint_limit_margin=self.joint_limit_margin,
                torque_limits=self.torque_limits,
                path_tolerance=self.cartesian_path_tolerance
                if self.control_mode == "task_space" else None,
                minimum_z=self.workspace_min_z if self.workspace_floor_enabled else None,
                floor_margin=self.workspace_floor_margin,
                floor_weight=self.workspace_floor_cost_weight,
            )
            self.running_actions.append(action)
            running_models.append(c.IntegratedActionModelEuler(action, self.dt))
        self.running_action = self.running_actions[0]
        self.terminal_action = SE3DynamicActionModel(
            crocoddyl=c,
            pin=self.pin,
            model=self.model,
            state=self.state,
            frame_id=self.frame_id,
            target=self._optimization_target_pose(),
            nominal_posture=nominal_posture,
            weights=task_weights,
            terminal_scale=self.terminal_cost_weight,
            joint_limit_margin=self.joint_limit_margin,
            torque_limits=self.torque_limits,
            path_tolerance=self.cartesian_path_tolerance
            if self.control_mode == "task_space" else None,
            minimum_z=self.workspace_min_z if self.workspace_floor_enabled else None,
            floor_margin=self.workspace_floor_margin,
            floor_weight=self.workspace_floor_cost_weight,
            terminal=True,
        )
        terminal_model = c.IntegratedActionModelEuler(self.terminal_action, 0.0)
        x0 = self.xref.copy()
        problem = c.ShootingProblem(x0, running_models, terminal_model)
        return problem, c.SolverFDDP(problem)

    def _reload_cost_config_if_changed(self) -> None:
        """Apply edited cost settings without restarting the active MPC solver."""
        if self.cost_config_path is None:
            return
        try:
            modified_ns = self.cost_config_path.stat().st_mtime_ns
            if modified_ns == self.cost_config_mtime_ns:
                return
            settings = load_runtime_cost_settings(self.cost_config_path)
        except (OSError, ValueError) as exc:
            self.get_logger().warning(
                f"Could not reload MPC cost YAML; keeping the last valid costs: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        effective_settings = dict(settings)
        if self.control_mode == "joint":
            effective_settings["nominal_posture"] = self.goal.tolist()
        apply_runtime_cost_settings(
            self.running_actions, self.terminal_action, effective_settings
        )
        if self.control_mode == "task_space":
            self.nominal_posture = np.asarray(settings["nominal_posture"], dtype=float)
        for parameter_name, action_name in (
            ("se3_position_cost_weight", "se3_position"),
            ("se3_orientation_cost_weight", "se3_orientation"),
            ("posture_cost_weight", "posture"),
            ("velocity_cost_weight", "velocity"),
            ("torque_cost_weight", "torque"),
            ("joint_limit_cost_weight", "joint_limit"),
        ):
            setattr(self, parameter_name, settings["weights"][action_name])
        self.running_cost_weight = settings["running_cost_weight"]
        self.terminal_cost_weight = settings["terminal_cost_weight"]
        self.cost_config_mtime_ns = modified_ns
        self.get_logger().info(
            "Applied live MPC cost YAML update: "
            f"position={self.se3_position_cost_weight:g}, "
            f"orientation={self.se3_orientation_cost_weight:g}, "
            f"posture={self.posture_cost_weight:g}, "
            f"velocity={self.velocity_cost_weight:g}, "
            f"torque={self.torque_cost_weight:g}, "
            f"joint_limit={self.joint_limit_cost_weight:g}"
        )

    def _cartesian_speed_limit(self, q: np.ndarray, displacement: np.ndarray) -> float:
        """Return the configured speed capped by the current Jacobian/qd limits."""
        distance = float(np.linalg.norm(displacement))
        if distance < 1.0e-9:
            return self.cartesian_max_speed
        direction = np.asarray(displacement, dtype=float) / distance
        try:
            self.pin.computeJointJacobians(self.model, self.pose_data, q)
            self.pin.updateFramePlacements(self.model, self.pose_data)
            current_pose = self.pose_data.oMf[self.frame_id]
            local_jacobian = np.asarray(
                self.pin.getFrameJacobian(
                    self.model, self.pose_data, self.frame_id, self.pin.LOCAL
                ),
                dtype=float,
            )
            linear_jacobian_world = current_pose.rotation @ local_jacobian[:3, :]
            qdot_for_unit_direction = np.linalg.pinv(linear_jacobian_world) @ direction
            velocity_limits = np.asarray(self.model.velocityLimit, dtype=float)
            active = (
                np.isfinite(qdot_for_unit_direction)
                & np.isfinite(velocity_limits)
                & (velocity_limits > 0.0)
                & (np.abs(qdot_for_unit_direction) > 1.0e-9)
            )
            if np.any(active):
                jacobian_cap = float(
                    np.min(
                        self.cartesian_speed_margin
                        * velocity_limits[active]
                        / np.abs(qdot_for_unit_direction[active])
                    )
                )
                if np.isfinite(jacobian_cap) and jacobian_cap > 0.0:
                    return min(self.cartesian_max_speed, jacobian_cap)
        except (FloatingPointError, RuntimeError, ValueError, np.linalg.LinAlgError):
            pass
        return self.cartesian_max_speed

    def _interpolate_cartesian_pose(self, alpha: float):
        """Interpolate position linearly and orientation on SO(3)."""
        alpha = float(np.clip(alpha, 0.0, 1.0))
        start = self.cartesian_path_start_pose
        goal = self._optimization_target_pose()
        position = (1.0 - alpha) * start.translation + alpha * goal.translation
        rotation_delta = self.pin.log3(start.rotation.T @ goal.rotation)
        rotation = start.rotation @ self.pin.exp3(alpha * rotation_delta)
        return self.pin.SE3(rotation, position)

    def _update_cartesian_references(self, q: np.ndarray) -> None:
        """Refresh every Crocoddyl stage with a bounded-speed Cartesian line."""
        if self.control_mode != "task_space":
            self.cartesian_reference_poses = [
                self._optimization_target_pose() for _ in range(self.horizon + 1)
            ]
            self.cartesian_path_speed = 0.0
            self.cartesian_path_progress = 1.0
            self.cartesian_path_deviation = 0.0
        else:
            goal_pose = self._optimization_target_pose()
            displacement = goal_pose.translation - self.cartesian_path_start_pose.translation
            self.cartesian_path_speed = self._cartesian_speed_limit(q, displacement)
            distance = float(np.linalg.norm(displacement))
            rotation_delta = self.pin.log3(
                self.cartesian_path_start_pose.rotation.T @ goal_pose.rotation
            )
            rotation_angle = float(np.linalg.norm(rotation_delta))
            self.cartesian_path_duration = max(
                distance / max(self.cartesian_path_speed, 1.0e-9),
                rotation_angle / max(self.cartesian_max_angular_speed, 1.0e-9),
                self.cartesian_min_duration,
            )
            elapsed = max(0.0, time.monotonic() - self.cartesian_path_start_time)
            self.cartesian_path_progress = float(
                np.clip(elapsed / self.cartesian_path_duration, 0.0, 1.0)
            )
            stage_times = elapsed + self.dt * np.arange(self.horizon + 1)
            self.cartesian_reference_poses = [
                self._interpolate_cartesian_pose(stage_time / self.cartesian_path_duration)
                for stage_time in stage_times
            ]
            current_pose = self._tcp_pose(q)
            self.cartesian_path_deviation = float(
                np.linalg.norm(segment_error(
                    current_pose.translation,
                    self.cartesian_path_start_pose.translation,
                    goal_pose.translation,
                )[0])
            )

        for action, reference_pose in zip(
            self.running_actions, self.cartesian_reference_poses[:-1]
        ):
            action.update_target(reference_pose)
            action.update_path(
                self.cartesian_path_start_pose.translation,
                self._optimization_target_pose().translation,
            )
        self.terminal_action.update_target(self.cartesian_reference_poses[-1])
        self.terminal_action.update_path(
            self.cartesian_path_start_pose.translation,
            self._optimization_target_pose().translation,
        )

    def _on_target_pose(self, message: PoseStamped) -> None:
        if self.vive_control is not None and self.vive_control.following:
            return
        self._apply_target_pose(message)

    def _apply_target_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.pose_frame:
            self.get_logger().warn(
                f"teleop pose frame '{message.header.frame_id}'를 '{self.pose_frame}'로 변환할 TF가 없습니다. 무시합니다.",
                throttle_duration_sec=2.0,
            )
            return
        position = np.array(
            [message.pose.position.x, message.pose.position.y, message.pose.position.z],
            dtype=float,
        )
        orientation = np.array(
            [
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)):
            return
        try:
            orientation = self._parse_quaternion(orientation)
        except ValueError:
            self.get_logger().warn("teleop quaternion이 잘못되어 pose를 무시합니다.", throttle_duration_sec=2.0)
            return
        if self.control_mode != "task_space":
            return
        new_target = self.pin.SE3(self.pin.Quaternion(orientation).matrix(), position)
        update_now = time.monotonic()
        # Repeated identical targets must not reset a line while tracking lags.
        if (
            not np.allclose(new_target.translation, self.target_pose.translation, rtol=0.0, atol=1e-12)
            or np.linalg.norm(self.pin.log3(self.target_pose.rotation.T @ new_target.rotation)) > 1e-12
        ):
            seed = self.latest_state[: self.model.nq] if self.latest_state is not None else self.sim_q
            current_pose = self._tcp_pose(seed).copy()
            mpc_target = self._optimization_target_pose(new_target)
            new_deviation = np.linalg.norm(segment_error(
                current_pose.translation,
                self.cartesian_path_start_pose.translation,
                mpc_target.translation,
            )[0])
            if (
                update_now - self.cartesian_path_start_time >= self.cartesian_path_duration
                or new_deviation > self.cartesian_path_tolerance
            ):
                self.cartesian_path_start_pose = current_pose
                self.cartesian_path_start_time = update_now
        target_dt = update_now - self.last_target_update_monotonic
        if self.previous_target_pose is not None and target_dt > 0.0:
            delta_local = self.pin.log3(
                self.previous_target_pose.rotation.T @ new_target.rotation
            )
            self.last_target_delta_rotation_world = (
                self.previous_target_pose.rotation @ delta_local
            )
            self.last_target_update_dt = target_dt
            self.last_target_angular_velocity_world = (
                self.last_target_delta_rotation_world / target_dt
            )
        self.target_pose = new_target
        self.previous_target_pose = new_target.copy()
        self.target_update_count += 1
        self.stationary_hold_active = False
        self.last_target_update_monotonic = update_now
        self.last_target_stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        seed = self.latest_state[:6] if self.latest_state is not None else self.sim_q
        self.xref = np.r_[self._solve_ik(self._optimization_target_pose(), seed), np.zeros(self.model.nv)]

    def _optimization_target_pose(self, desired=None):
        """Project an infeasible desired goal for MPC while preserving its input pose."""
        target = (self.target_pose if desired is None else desired).copy()
        if self.workspace_floor_enabled:
            target.translation[2] = max(
                target.translation[2], self.workspace_min_z + self.workspace_floor_margin
            )
        return target

    def _below_workspace_floor(self, q):
        return self.workspace_floor_enabled and float(self._tcp_pose(q).translation[2]) < self.workspace_min_z

    def _command_guard(self):
        return self.workspace_floor_guard or self.cartesian_path_guard

    def _tcp_pose(self, q: np.ndarray):
        self.pin.framesForwardKinematics(self.model, self.pose_data, q)
        return self.pose_data.oMf[self.frame_id]

    def _publish_pose(self, publisher, pose) -> None:
        quaternion = self.pin.Quaternion(pose.rotation).coeffs()
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.pose_frame
        message.pose.position.x = float(pose.translation[0])
        message.pose.position.y = float(pose.translation[1])
        message.pose.position.z = float(pose.translation[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        publisher.publish(message)

    def _publish_command_pose(self, q: np.ndarray) -> None:
        self._publish_pose(self.command_pose_pub, self._tcp_pose(q))

    def _publish_target_pose(self) -> None:
        self._publish_pose(self.desired_pose_pub, self.target_pose)

    def _publish_qref(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = self.xref[: self.model.nq].tolist()
        message.velocity = [0.0] * self.model.nv
        self.qref_pub.publish(message)

    def _on_joint_state(self, message: JointState) -> None:
        if message.name:
            by_name = {name: i for i, name in enumerate(message.name)}
            if any(name not in by_name for name in self.joint_names):
                return
            if any(by_name[name] >= len(message.position) for name in self.joint_names):
                return
            position = np.array([message.position[by_name[name]] for name in self.joint_names])
            velocity = np.array(
                [message.velocity[by_name[name]] if by_name[name] < len(message.velocity) else 0.0 for name in self.joint_names]
            )
        elif len(message.position) >= 6:
            position = np.asarray(message.position[:6], dtype=float)
            velocity = np.asarray(message.velocity[:6], dtype=float) if len(message.velocity) >= 6 else np.zeros(6)
        else:
            return
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return
        state_stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        if self.simulate and not self.internal_simulation and not self.execution_enabled:
            # A paused alignment teleports the mock plant. That reset must not
            # become a large finite-difference velocity at the next F7 start.
            velocity = np.zeros(self.model.nv)
        elif self.simulate and not self.internal_simulation and self.latest_state is not None:
            elapsed = state_stamp_s - self.last_state_stamp_s
            if elapsed > 1.0e-6:
                # Average the mock servo's position jumps across the samples we actually consume.
                observed_velocity = (position - self.latest_state[: self.model.nq]) / elapsed
                alpha = elapsed / (elapsed + 2.0 * self.dt)
                velocity = (
                    alpha * observed_velocity
                    + (1.0 - alpha) * self.latest_state[self.model.nq :]
                )
        self.latest_state = np.r_[position, velocity]
        self.last_state_update_monotonic = time.monotonic()
        self.last_state_stamp_s = state_stamp_s
        if not self.initial_target_state_initialized:
            current_pose = self._tcp_pose(position).copy()
            self.target_pose = current_pose
            self.previous_target_pose = current_pose.copy()
            self.cartesian_path_start_pose = current_pose.copy()
            self.cartesian_path_start_time = time.monotonic()
            self.cartesian_path_duration = self.cartesian_min_duration
            self.cartesian_path_progress = 0.0
            self.cartesian_path_initialized_from_state = True
            self.cartesian_reference_poses = [
                current_pose.copy() for _ in range(self.horizon + 1)
            ]
            self.xref = np.r_[position, np.zeros(self.model.nv)]
            self.xs = [self.xref.copy() for _ in range(self.horizon + 1)]
            self.us = [np.zeros(self.model.nv) for _ in range(self.horizon)]
            self.initial_target_state_initialized = True
            self.get_logger().info(
                "Initial task target set to measured Indy7 TCP pose; waiting for teleop input."
            )

    def _on_sim_sync_joint_target(self, message: JointState) -> None:
        """Set the simulated Desired pose from a named, mapped robot posture."""
        request_id = message.header.frame_id

        def acknowledge(success: bool, detail: str) -> None:
            self.sim_sync_ack_pub.publish(
                String(
                    data=json.dumps(
                        {"request_id": request_id, "success": success, "message": detail}
                    )
                )
            )

        if not self.simulate:
            acknowledge(False, "MPC is not in simulation mode.")
            return
        if self.vive_control is not None and self.vive_control.following:
            acknowledge(False, "Pause Vive following before syncing a robot joint pose.")
            return
        if self.control_mode != "task_space":
            acknowledge(False, "Desired pose sync is available only in task_space control mode.")
            return
        if self.latest_state is None or time.monotonic() - self.last_state_update_monotonic > 0.5:
            acknowledge(False, "Simulation joint state is unavailable or stale.")
            return
        if message.name != self.joint_names or len(message.position) != self.model.nq:
            acknowledge(False, "The sync request has invalid joint names or an incorrect joint count.")
            return
        target_q = np.asarray(message.position, dtype=float)
        if not np.all(np.isfinite(target_q)):
            acknowledge(False, "The sync request contains invalid joint values.")
            return
        if np.any(target_q < self.model.lowerPositionLimit + 0.01) or np.any(
            target_q > self.model.upperPositionLimit - 0.01
        ):
            acknowledge(False, "The sync target is outside or too close to a URDF joint limit.")
            return

        if self._below_workspace_floor(target_q):
            acknowledge(False, f"Robot alignment rejected: {self.task_frame} is below world Z={self.workspace_min_z:.3f} m.")
            return

        now = time.monotonic()
        current_q = self.latest_state[: self.model.nq].copy()
        current_pose = self._tcp_pose(current_q).copy()
        target_pose = self._tcp_pose(target_q).copy()
        self.cartesian_path_start_pose = current_pose
        self.cartesian_path_start_time = now
        self.cartesian_path_duration = self.cartesian_min_duration
        self.cartesian_path_progress = 0.0
        self.cartesian_path_guard = ""
        self.cartesian_path_initialized_from_state = True
        self.cartesian_reference_poses = [
            current_pose.copy() for _ in range(self.horizon + 1)
        ]
        self.target_pose = target_pose
        self.previous_target_pose = target_pose.copy()
        self.xref = np.r_[target_q, np.zeros(self.model.nv)]
        state_reference = np.r_[target_q, np.zeros(self.model.nv)]
        self.xs = [state_reference.copy() for _ in range(self.horizon + 1)]
        self.us = [np.zeros(self.model.nv) for _ in range(self.horizon)]
        self.stationary_hold_active = False
        self.stationary_hold_q = None
        self.target_update_count += 1
        self.last_target_update_monotonic = now
        self.last_target_stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        if not self.execution_enabled:
            # Align the mock plant while paused, before F6 enables physical
            # output. This command remains on the simulation-only boundary.
            self.cartesian_path_start_pose = target_pose.copy()
            self._paused_alignment_target = target_q.copy()
            self._paused_alignment_deadline = now + 2.0
            self._publish_joint_command(target_q)
            if self.internal_simulation:
                self.sim_q = target_q.copy()
                self.sim_v.fill(0.0)
                self.sim_acceleration.fill(0.0)
                self.latest_state = np.r_[self.sim_q, self.sim_v]
                self._publish_sim_state()
        self._publish_target_pose()
        self.get_logger().info("Desired TCP pose synchronized from mapped robot joint state.")
        acknowledge(True, "Applied the desired pose computed from the robot joint posture.")

    def _solver_is_numerically_stationary(self) -> bool:
        """Recognize FDDP's false negative at a zero-step optimum."""
        try:
            stop = float(getattr(self.solver, "stop", np.nan))
            th_stop = float(getattr(self.solver, "th_stop", np.nan))
            d_v = np.asarray(getattr(self.solver, "dV", []), dtype=float)
            return bool(
                np.isfinite(stop)
                and np.isfinite(th_stop)
                and stop <= max(th_stop, 1.0e-12)
                and (d_v.size == 0 or np.all(np.isfinite(d_v)))
                and (d_v.size == 0 or float(np.max(np.abs(d_v))) <= 1.0e-10)
            )
        except (TypeError, ValueError):
            return False

    def _prediction_guard_reason(
        self, next_state: np.ndarray, raw_control: np.ndarray
    ) -> str:
        """Reject only invalid predictions before the q_cmd safety filter."""
        next_state = np.asarray(next_state, dtype=float)
        raw_control = np.asarray(raw_control, dtype=float)
        if (
            next_state.shape != (self.model.nq + self.model.nv,)
            or raw_control.shape != (self.model.nv,)
            or not np.all(np.isfinite(next_state))
            or not np.all(np.isfinite(raw_control))
        ):
            return "non_finite_prediction"
        return ""

    def _publish_stationary_hold(self, state: np.ndarray) -> None:
        """Keep the last command without invoking FDDP again."""
        if self.stationary_hold_q is not None and np.all(np.isfinite(self.stationary_hold_q)):
            q_cmd = self._safe_joint_command(self.stationary_hold_q, state)
        else:
            q_cmd = self._safe_joint_command(state[: self.model.nq], state)
        guard_success = not bool(self._command_guard())
        if not guard_success:
            self.stationary_hold_active = False
        current_pose = self._tcp_pose(state[: self.model.nq]).copy()
        pose_error = self._pose_error(state[: self.model.nq], self.target_pose, self.pose_data)
        self.last_q_cmd = q_cmd.copy()
        self.last_target_acceleration.fill(np.nan)
        self.last_applied_acceleration.fill(np.nan)
        self.solve_time_pub.publish(Float64(data=0.0))
        self.solve_iterations_pub.publish(Float64(data=0.0))
        self.solve_success_pub.publish(Float64(data=1.0))
        self.status_pub.publish(String(data=self._command_guard() or "stationary_hold"))
        self.control_norm_pub.publish(Float64(data=0.0))
        self.task_error_pub.publish(Float64(data=float(np.linalg.norm(pose_error))))
        self.position_error_pub.publish(Float64(data=float(np.linalg.norm(pose_error[:3]))))
        self.orientation_error_pub.publish(Float64(data=float(np.linalg.norm(pose_error[3:]))))
        self._publish_joint_command(q_cmd)
        self._publish_target_pose()
        self._publish_pose(self.cartesian_reference_pub, self.cartesian_reference_poses[0])
        self._publish_pose(self.current_pose_pub, current_pose)
        self._publish_qref()
        self.solve_count += 1
        self._write_diagnostic(
            status=self._command_guard() or "stationary_hold",
            state=state,
            target=self.target_pose,
            solver_success=True,
            solver_cost=self.stationary_hold_cost,
            q_cmd=q_cmd,
            current_pose=current_pose,
            pose_error=pose_error,
            stationary_optimum=True,
            cost_terms={},
            terminal_cost_terms={},
            dynamics={},
        )
        self._publish_command_pose(q_cmd)

    def _set_execution_enabled(self, enabled):
        self.execution_enabled = bool(enabled)
        self._paused_alignment_target = None
        if self.latest_state is None:
            return
        q = self.latest_state[:self.model.nq].copy()
        current = self._tcp_pose(q).copy()
        self.target_pose = current
        self.previous_target_pose = current.copy()
        self.cartesian_path_start_pose = current.copy()
        self.cartesian_path_start_time = time.monotonic()
        self.cartesian_path_duration = self.cartesian_min_duration
        self.cartesian_path_progress = 0.0
        self.xref = np.r_[q, np.zeros(self.model.nv)]
        self.xs = [self.xref.copy() for _ in range(self.horizon + 1)]
        self.us = [np.zeros(self.model.nv) for _ in range(self.horizon)]
        self.stationary_hold_active = False
        self.target_update_count += 1
        if not enabled:
            self.sim_v.fill(0.0)
            self.sim_acceleration.fill(0.0)
            if time.monotonic() - self.last_state_update_monotonic <= 0.5:
                self._publish_joint_command(q)

    def _publish_paused(self):
        now = time.monotonic()
        if now - self._last_paused_publish < 0.1:
            return
        self._last_paused_publish = now
        self._publish_pose(self.current_pose_pub, self._tcp_pose(self.latest_state[:self.model.nq]))
        self._publish_target_pose()
        self._publish_qref()
        self.status_pub.publish(String(data="paused_waiting_for_f7"))
        if self.internal_simulation:
            self._publish_sim_state()

    def _solve_once(self) -> None:
        if self.internal_simulation:
            self.latest_state = np.r_[self.sim_q, self.sim_v]
            self.last_state_update_monotonic = time.monotonic()
            self.last_state_stamp_s = float("nan")
        if self.latest_state is None:
            if not self.warned_no_state:
                self.get_logger().warn("joint_states를 기다리는 중입니다.")
                self.warned_no_state = True
            return

        if self.vive_control is not None:
            self.vive_control.before_solve()
        if not self.execution_enabled:
            if self._paused_alignment_target is not None:
                if (time.monotonic() > self._paused_alignment_deadline
                        or np.allclose(self.latest_state[:self.model.nq], self._paused_alignment_target,
                                       rtol=0, atol=1e-5)):
                    self._paused_alignment_target = None
                else:
                    # Repeat until measured feedback confirms application;
                    # the controller/subscription may still be activating.
                    self._publish_joint_command(self._paused_alignment_target)
            self._publish_paused()
            return

        state = self.latest_state.copy()
        if self._below_workspace_floor(state[:self.model.nq]):
            self._publish_joint_command(state[:self.model.nq])
            self.solve_success_pub.publish(Float64(data=0.0))
            self.status_pub.publish(String(data="workspace_floor_violation"))
            self.get_logger().error(
                f"Measured {self.task_frame} is below world Z={self.workspace_min_z:.3f} m; holding joints.",
                throttle_duration_sec=2.0,
            )
            return
        if not self.cartesian_path_initialized_from_state:
            self.cartesian_path_start_pose = self._tcp_pose(state[: self.model.nq]).copy()
            self.cartesian_path_start_time = time.monotonic()
            self.cartesian_path_initialized_from_state = True
        self._update_cartesian_references(state[: self.model.nq])
        if self.simulate and self.control_mode == "task_space" and not self.stationary_hold_active:
            pose_error = self._pose_error(
                state[: self.model.nq], self._optimization_target_pose(), self.pose_data
            )
            # The ideal position servo can stop on the measured pose without another torque solve.
            if (
                np.linalg.norm(pose_error[:3]) <= self.stationary_hold_position_tolerance
                and np.linalg.norm(pose_error[3:]) <= self.stationary_hold_orientation_tolerance
            ):
                self.stationary_hold_active = True
                self.stationary_hold_target_count = self.target_update_count
                self.stationary_hold_q = state[: self.model.nq].copy()
                self.stationary_hold_cost = float(getattr(self.solver, "cost", 0.0))
        if (
            self.stationary_hold_active
            and self.stationary_hold_target_count == self.target_update_count
            and self.stationary_hold_q is not None
            and np.all(np.isfinite(self.stationary_hold_q))
            and (
                self.simulate
                or np.max(np.abs(state[: self.model.nq] - self.stationary_hold_q)) <= 1.0e-3
            )
        ):
            self._publish_stationary_hold(state)
            return
        self.stationary_hold_active = False
        self.problem.x0 = state
        self.xs[0] = state.copy()
        self.last_target_acceleration.fill(np.nan)
        self.last_applied_acceleration.fill(np.nan)
        solve_start = time.perf_counter()
        try:
            solved = bool(self.solver.solve(self.xs, self.us, self.max_solver_iterations, False))
        except Exception as exc:
            solve_time_ms = 1.0e3 * (time.perf_counter() - solve_start)
            self.get_logger().error(f"MPC solve 실패: {exc}")
            self.solve_success_pub.publish(Float64(data=0.0))
            self.status_pub.publish(String(data="exception"))
            self._write_diagnostic(
                status="exception",
                state=state,
                target=self.target_pose,
                solve_time_ms=solve_time_ms,
                solver_iterations=int(getattr(self.solver, "iter", 0)),
                solver_error=repr(exc),
            )
            return
        solve_time_ms = 1.0e3 * (time.perf_counter() - solve_start)
        solver_iterations = int(getattr(self.solver, "iter", 0))
        solver_cost = float(getattr(self.solver, "cost", np.nan))
        self.solve_time_pub.publish(Float64(data=solve_time_ms))
        self.solve_iterations_pub.publish(Float64(data=float(solver_iterations)))
        trajectory_finite = (
            len(self.solver.xs) >= 2
            and len(self.solver.us) >= 1
            and np.all(np.isfinite(self.solver.xs[1]))
            and np.all(np.isfinite(self.solver.us[0]))
        )
        stationary_optimum = bool(
            trajectory_finite and not solved and self._solver_is_numerically_stationary()
        )
        if (
            not trajectory_finite
        ):
            self.solve_success_pub.publish(Float64(data=0.0))
            self.status_pub.publish(String(data="invalid_solution"))
            self._write_diagnostic(
                status="invalid_solution",
                state=state,
                solver_iterations=solver_iterations,
                solver_cost=solver_cost,
                target=self.target_pose,
                solve_time_ms=solve_time_ms,
            )
            return
        self.xs, self.us = self.solver.xs, self.solver.us
        cost = solver_cost
        raw_control = np.asarray(self.us[0], dtype=float).copy()
        current_pose = self._tcp_pose(state[:6]).copy()
        task_error = self._pose_error(state[:6], self.target_pose, self.pose_data)
        optimization_error = self._pose_error(
            state[:6], self._optimization_target_pose(), self.pose_data
        )
        jacobian = self._jacobian_record(state[:6], current_pose)
        cost_terms = self.running_action.cost_terms(state, raw_control)
        terminal_cost_terms = self.terminal_action.cost_terms(
            self.xs[-1], np.zeros(self.model.nv)
        )
        dynamics = self.running_action.dynamics_terms(state, raw_control)
        self.last_dynamics = dynamics
        self.cost_pub.publish(Float64(data=cost))
        for name, publisher in self.cost_term_pubs.items():
            publisher.publish(Float64(data=float(cost_terms.get(name, 0.0))))
        self.terminal_cost_pub.publish(
            Float64(data=float(sum(terminal_cost_terms.values())))
        )
        self.task_error_pub.publish(Float64(data=float(np.linalg.norm(task_error))))
        self.position_error_pub.publish(Float64(data=float(np.linalg.norm(task_error[:3]))))
        self.orientation_error_pub.publish(Float64(data=float(np.linalg.norm(task_error[3:]))))
        self.control_norm_pub.publish(Float64(data=float(np.linalg.norm(raw_control))))
        self._publish_target_pose()
        self._publish_pose(
            self.cartesian_reference_pub, self.cartesian_reference_poses[0]
        )
        self._publish_pose(self.current_pose_pub, current_pose)
        self._publish_qref()
        self.solve_count += 1
        if self.solve_count % max(1, round(1.0 / self.dt)) == 0:
            self.get_logger().info(
                "MPC cost=%.5f task=%.5f/%.5f qref=%.4f |u|=%.3f solve=%.2fms iter=%d %s"
                % (
                    cost,
                    float(np.linalg.norm(task_error[:3])),
                    float(np.linalg.norm(task_error[3:])),
                    float(np.linalg.norm(state[:6] - self.xref[:6])),
                    float(np.linalg.norm(raw_control)),
                    solve_time_ms,
                    solver_iterations,
                    cost_terms,
                )
        )
        next_state = self.xs[1]
        prediction_guard = self._prediction_guard_reason(next_state, raw_control)
        if prediction_guard:
            if self.last_q_cmd is not None and np.all(np.isfinite(self.last_q_cmd)):
                q_cmd = self._safe_joint_command(self.last_q_cmd, state)
            else:
                q_cmd = self._safe_joint_command(state[: self.model.nq], state)
        else:
            q_cmd = self._safe_joint_command(next_state[: self.model.nq], state)
        prediction_guard = prediction_guard or self._command_guard()
        self.last_q_cmd = q_cmd.copy()
        self._publish_joint_command(q_cmd)
        stationary_hold_ready = bool(
            stationary_optimum
            and not prediction_guard
            and float(np.linalg.norm(optimization_error[:3]))
            <= self.stationary_hold_position_tolerance
            and float(np.linalg.norm(optimization_error[3:]))
            <= self.stationary_hold_orientation_tolerance
        )
        solver_success = bool(solved or stationary_optimum)
        status = (
            self._command_guard()
            or ("prediction_guard_hold" if prediction_guard else "")
            or ("stationary_optimum" if stationary_optimum else "")
            or ("solved" if solved else "not_converged")
        )
        self.solve_success_pub.publish(Float64(data=1.0 if solver_success else 0.0))
        self.status_pub.publish(String(data=status))
        if stationary_hold_ready:
            self.stationary_hold_active = True
            self.stationary_hold_target_count = self.target_update_count
            self.stationary_hold_q = q_cmd.copy()
            self.stationary_hold_cost = cost
        else:
            self.stationary_hold_active = False
        applied_state = None
        if self.internal_simulation:
            self._step_simulation(q_cmd)
            self._publish_sim_state()
            applied_state = np.r_[self.sim_q, self.sim_v].copy()
        self._write_diagnostic(
            status=status,
            state=state,
            target=self.target_pose,
            solve_time_ms=solve_time_ms,
            solver_success=solver_success,
            solver_iterations=solver_iterations,
            solver_cost=cost,
            stationary_optimum=stationary_optimum,
            prediction_guard=prediction_guard,
            raw_control=raw_control,
            q_cmd=q_cmd,
            predicted_next_state=self.xs[1],
            applied_state=applied_state,
            cost_terms=cost_terms,
            terminal_cost_terms=terminal_cost_terms,
            current_pose=current_pose,
            pose_error=task_error,
            jacobian=jacobian,
            dynamics=dynamics,
        )
        self._publish_command_pose(q_cmd)

    def _safe_joint_command(self, predicted_q: np.ndarray, state: np.ndarray) -> np.ndarray:
        """Limit absolute q_cmd before it crosses the simulation/hardware boundary."""
        predicted_q = np.asarray(predicted_q, dtype=float)
        measured_q = np.asarray(state[: self.model.nq], dtype=float)
        if predicted_q.shape != (self.model.nq,) or not np.all(np.isfinite(predicted_q)):
            raise RuntimeError("MPC produced a non-finite joint-position command")
        if measured_q.shape != (self.model.nq,) or not np.all(np.isfinite(measured_q)):
            raise RuntimeError("joint state is invalid for a joint-position command")
        measured_q = np.clip(
            measured_q, self.model.lowerPositionLimit, self.model.upperPositionLimit
        )
        # Both bounds are intentional: velocity gives a physical rate limit,
        # increment caps a single bad solver cycle independently of dt.
        max_step = np.minimum(
            self.command_velocity_limits * self.dt,
            self.command_increment_limits,
        )
        lower = np.maximum(self.model.lowerPositionLimit, measured_q - max_step)
        upper = np.minimum(self.model.upperPositionLimit, measured_q + max_step)
        command = np.clip(predicted_q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        command = np.minimum(np.maximum(command, lower), upper)
        if not np.all(np.isfinite(command)):
            raise RuntimeError("joint-position safety filter produced a non-finite command")
        self.cartesian_command_scale = 1.0
        self.cartesian_path_guard = ""
        self.workspace_floor_guard = ""
        if self.control_mode == "task_space":
            start = self.cartesian_path_start_pose.translation
            goal = self._optimization_target_pose().translation
            deviation = float(np.linalg.norm(segment_error(
                self._tcp_pose(measured_q).translation, start, goal
            )[0]))
            original_command = command.copy()
            if deviation <= self.cartesian_path_tolerance:
                command = project_cartesian_command(
                    command, lower, upper,
                    lambda q: self._tcp_pose(q).translation.copy(),
                    self._tcp_position_jacobian,
                    start, goal, self.cartesian_path_tolerance,
                )
            command, self.cartesian_command_scale = limit_cartesian_step(
                measured_q, command,
                lambda q: self._tcp_pose(q).translation.copy(),
                start, goal, self.cartesian_path_tolerance, self.cartesian_reach_bound,
            )
            if deviation > self.cartesian_path_tolerance:
                self.cartesian_path_guard = "cartesian_path_violation"
                self.get_logger().error(
                    f"TCP line deviation {deviation * 1000:.3f} mm exceeds "
                    f"{self.cartesian_path_tolerance * 1000:.3f} mm; holding measured joints",
                    throttle_duration_sec=2.0,
                )
            elif self.cartesian_command_scale == 0.0:
                self.cartesian_path_guard = "cartesian_path_hold"
            elif self.cartesian_command_scale < 1.0 or not np.array_equal(command, original_command):
                self.cartesian_path_guard = "cartesian_path_limited"
        if self.workspace_floor_enabled:
            command, scale = limit_height_step(
                measured_q, command, lambda q: self._tcp_pose(q).translation.copy(),
                self._tcp_position_jacobian, self.workspace_min_z, self.cartesian_reach_bound,
            )
            if self._below_workspace_floor(measured_q):
                self.workspace_floor_guard = "workspace_floor_violation"
            elif scale == 0.0:
                self.workspace_floor_guard = "workspace_floor_hold"
            elif scale < 1.0:
                self.workspace_floor_guard = "workspace_floor_limited"
        return command

    def _tcp_position_jacobian(self, q: np.ndarray) -> np.ndarray:
        self.pin.computeJointJacobians(self.model, self.pose_data, q)
        self.pin.updateFramePlacements(self.model, self.pose_data)
        return self.pin.getFrameJacobian(
            self.model, self.pose_data, self.frame_id, self.pin.LOCAL_WORLD_ALIGNED
        )[:3].copy()

    def _publish_joint_command(self, q_cmd: np.ndarray) -> None:
        message = Float64MultiArray()
        message.data = np.asarray(q_cmd, dtype=float).tolist()
        self.joint_command_pub.publish(message)

    def _step_simulation(self, q_cmd: np.ndarray) -> None:
        """Fallback plant for standalone tests; emulate a position servo."""
        previous_velocity = self.sim_v.copy()
        delta = np.asarray(q_cmd, dtype=float) - self.sim_q
        max_step = np.minimum(
            self.command_velocity_limits * self.dt,
            self.command_increment_limits,
        )
        delta = np.clip(delta, -max_step, max_step)
        self.sim_q = np.clip(
            self.sim_q + delta,
            self.model.lowerPositionLimit,
            self.model.upperPositionLimit,
        )
        self.sim_v = delta / self.dt
        self.sim_acceleration = (self.sim_v - previous_velocity) / self.dt
        self.last_target_acceleration = np.clip(
            self.sim_acceleration,
            -self.acceleration_limits,
            self.acceleration_limits,
        )
        self.last_applied_acceleration = self.sim_acceleration.copy()

    def _publish_sim_state(self) -> None:
        if self.state_pub is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = self.sim_q.tolist()
        message.velocity = self.sim_v.tolist()
        self.state_pub.publish(message)

    def destroy_node(self):
        if self._diagnostic_file is not None:
            self._diagnostic_file.flush()
            self._diagnostic_file.close()
            self._diagnostic_file = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CrocoddylMPCNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
