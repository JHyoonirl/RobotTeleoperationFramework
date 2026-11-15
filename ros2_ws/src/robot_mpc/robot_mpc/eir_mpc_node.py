"""One shared position-command MPC for two EIR hands and the two-axis torso."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .cartesian_path import (
    limit_cartesian_step, project_cartesian_tasks, segment_error, smooth_path_progress,
)
from .crocoddyl_mpc_node import CrocoddylMPCNode
from .workspace_floor import limit_height_step


class EIRMPCNode(CrocoddylMPCNode):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.torso_preferred = self.nominal_posture[:2].copy()
        self.left_target_pose = self._left_pose(self.sim_q)
        self._mirror_home = (self._tcp_pose(self.sim_q).translation
                             - np.array([1.0, -1.0, 1.0]) * self.left_target_pose.translation)
        self.left_path_start = self.left_target_pose.copy()
        self.left_path_start_time = time.monotonic()
        self.left_path_duration = self.cartesian_min_duration
        self.left_reference_poses = [self.left_target_pose.copy() for _ in range(self.horizon + 2)]
        self._left_received = None
        self._left_stamp_s = None
        self._left_guard = ""
        self._mirror_guard = ""
        self._joint_limit_guard = ""
        self._limited_joints = []
        self._precise_hold_q = None
        self._precise_hold_target_count = -1
        self._update_torso_costs()

    def _declare_robot_parameters(self):
        self._declare_configuration_parameter("torso_cost_weight", 40.0)
        self._declare_configuration_parameter("torso_pitch_cost_weight", 80.0)
        self._declare_configuration_parameter("short_move_distance", 0.15)
        self._declare_configuration_parameter("short_move_speed", 0.05)

    def _read_robot_configuration(self):
        if not self.simulate or self.control_mode != "task_space":
            raise ValueError("EIR whole-body integration currently requires task-space simulation")
        expected = ["Joint_L0", "Joint_L1"] + [
            f"Joint_L{i}_{side}" for side in ("L", "R") for i in range(2, 9)
        ]
        if self.joint_names != expected or self.task_frame != "right_hand_tcp":
            raise ValueError("EIR MPC requires both arms, torso yaw/pitch and right_hand_tcp")
        self.left_frame_id = self.model.getFrameId("left_hand_tcp")
        if self.left_frame_id >= len(self.model.frames):
            raise ValueError("EIR URDF has no left_hand_tcp")
        self.torso_cost_weight = float(self.get_parameter("torso_cost_weight").value)
        self.torso_pitch_cost_weight = float(self.get_parameter("torso_pitch_cost_weight").value)
        self.short_move_distance = float(self.get_parameter("short_move_distance").value)
        self.short_move_speed = float(self.get_parameter("short_move_speed").value)
        if (not np.isfinite(self.torso_cost_weight) or self.torso_cost_weight <= 0
                or not np.isfinite(self.torso_pitch_cost_weight) or self.torso_pitch_cost_weight <= 0
                or not np.isfinite(self.short_move_distance) or self.short_move_distance <= 0
                or not np.isfinite(self.short_move_speed) or self.short_move_speed <= 0):
            raise ValueError("Torso and short-move settings must be positive and finite")

    def _create_interfaces(self):
        super()._create_interfaces()
        self.joint_limit_active_pub = self.create_publisher(
            String, "/mpc/joint_limit_active", 10
        )
        self.create_subscription(PoseStamped, "/eir/left/target_pose", self._apply_left_target, 1)
        self.left_current_pub = self.create_publisher(PoseStamped, "/eir/left/current_pose", 10)
        self.left_target_pub = self.create_publisher(PoseStamped, "/eir/left/desired_pose", 10)
        self.left_tf = TransformBroadcaster(self)
        self.create_timer(0.05, self._publish_left_pose)

    def _frame_pose(self, q, frame_id):
        self.pin.framesForwardKinematics(self.model, self.pose_data, q)
        return self.pose_data.oMf[frame_id].copy()

    def _left_pose(self, q):
        return self._frame_pose(q, self.left_frame_id)

    def _left_error(self, q):
        relative = self._left_pose(q).inverse() * self._optimization_target_pose(self.left_target_pose)
        return np.r_[relative.translation, self.pin.log3(relative.rotation)]

    def _fresh_stamp(self, stamp):
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        return seconds > 0 and abs(now - seconds) <= self.joint_state_timeout

    def _apply_left_target(self, message):
        if message.header.frame_id != self.pose_frame or not self._fresh_stamp(message.header.stamp):
            return
        p, q = message.pose.position, message.pose.orientation
        position = np.array([p.x, p.y, p.z])
        try:
            quaternion = self._parse_quaternion([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        if not np.all(np.isfinite(position)):
            return
        self._left_received = time.monotonic()
        self._left_stamp_s = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        target = self.pin.SE3(self.pin.Quaternion(quaternion).matrix(), position)
        if (np.allclose(position, self.left_target_pose.translation, rtol=0, atol=1e-12)
                and np.linalg.norm(self.pin.log3(self.left_target_pose.rotation.T @ target.rotation)) <= 1e-12):
            return
        current = self._left_pose(self.sim_q if self.latest_state is None else self.latest_state[:self.model.nq])
        self.left_path_start = current
        self.left_path_start_time = time.monotonic()
        self.left_target_pose = target
        self.target_update_count += 1
        self.stationary_hold_active = False

    def _execution_ready_reason(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if (self._left_received is None or now - self._left_received > self.joint_state_timeout
                or self._left_stamp_s is None or abs(ros_now - self._left_stamp_s) > self.joint_state_timeout):
            return "Waiting for fresh left-hand target; both hands must be ready"
        return ""

    def _update_torso_costs(self):
        self.torso_preferred = self.nominal_posture[:2].copy()
        weights = np.full(self.model.nq, self.posture_cost_weight)
        weights[:2] = self.torso_cost_weight
        weights[1] = self.torso_pitch_cost_weight
        for action in [*self.running_actions, self.terminal_action]:
            reference = np.asarray(action.nominal_posture, dtype=float).copy()
            reference[:2] = self.torso_preferred
            action.nominal_posture = reference
            action.update_posture_weights(weights)

    def _reload_cost_config_if_changed(self):
        settings = super()._reload_cost_config_if_changed()
        if settings is None:
            return
        parameters = []
        for name in ("torso_cost_weight", "torso_pitch_cost_weight"):
            if name in settings:
                setattr(self, name, settings[name])
                parameters.append(Parameter(name, value=settings[name]))
        if parameters:
            self.set_parameters(parameters)
            self.get_logger().info(
                f"Applied live torso costs: yaw={self.torso_cost_weight:g}, "
                f"pitch={self.torso_pitch_cost_weight:g}"
            )
        self._update_torso_costs()

    def _cartesian_speed_limit(self, q, displacement):
        speed = super()._cartesian_speed_limit(q, displacement)
        if (hasattr(self, "short_move_distance")
                and np.linalg.norm(displacement) < self.short_move_distance):
            speed = min(speed, self.short_move_speed)
        return speed

    def _left_references(self, q):
        goal = self._optimization_target_pose(self.left_target_pose)
        start = self.left_path_start
        distance = float(np.linalg.norm(goal.translation - start.translation))
        angle = float(np.linalg.norm(self.pin.log3(start.rotation.T @ goal.rotation)))
        ramp = self.cartesian_min_duration / 2
        left_speed = (min(self.cartesian_max_speed, self.short_move_speed)
                      if distance < self.short_move_distance else self.cartesian_max_speed)
        self.left_path_duration = max(distance / left_speed + ramp,
                                      angle / self.cartesian_max_angular_speed + ramp,
                                      self.cartesian_min_duration)
        elapsed = max(0.0, time.monotonic() - self.left_path_start_time)
        poses = []
        for index in range(self.horizon + 2):
            progress = smooth_path_progress((elapsed + index * self.dt) / self.left_path_duration,
                                            ramp / self.left_path_duration)
            poses.append(self.pin.SE3(start.rotation @ self.pin.exp3(
                progress * self.pin.log3(start.rotation.T @ goal.rotation)),
                start.translation + progress * (goal.translation - start.translation)))
        self.left_reference_poses = poses

    def _update_cartesian_references(self, q):
        # Hard-tube recovery may need reanchoring; soft tracking keeps the
        # original line so a tracking error cannot redefine the desired path.
        recovery_tasks = (
                (self._tcp_pose(q), self.cartesian_path_start_pose,
                 self._optimization_target_pose(), "right"),
                (self._left_pose(q), self.left_path_start,
                 self._optimization_target_pose(self.left_target_pose), "left"))
        for current, start, goal, update in (recovery_tasks if self.cartesian_path_limit_enabled else ()):
            if np.linalg.norm(segment_error(current.translation, start.translation,
                                            goal.translation)[0]) > self.cartesian_path_tolerance:
                if update == "right":
                    self.cartesian_path_start_pose = current.copy()
                    self.cartesian_path_start_time = time.monotonic()
                else:
                    self.left_path_start = current.copy()
                    self.left_path_start_time = time.monotonic()
        super()._update_cartesian_references(q)
        self._left_references(q)
        right_next = self.cartesian_reference_poses[1:] + [self._interpolate_cartesian_pose(
            (max(0.0, time.monotonic() - self.cartesian_path_start_time)
             + (self.horizon + 1) * self.dt) / self.cartesian_path_duration)]
        for index, action in enumerate([*self.running_actions, self.terminal_action]):
            left, next_left = self.left_reference_poses[index:index + 2]
            action.update_secondary_task(self.left_frame_id, left, self.left_path_start.translation,
                                         self._optimization_target_pose(self.left_target_pose).translation)
            predicted_q = q if index == 0 else self.xs[min(index + 1, self.horizon)][:self.model.nq]
            if not np.all(np.isfinite(predicted_q)):
                predicted_q = q
            action.update_velocity_reference(self._two_hand_velocity(
                predicted_q, self.cartesian_reference_poses[index], right_next[index], left, next_left))
        self._update_torso_costs()

    def _two_hand_velocity(self, q, right, next_right, left, next_left):
        # Feed back pose lag after the timed path ends, when its feedforward twist is zero.
        twists = []
        for reference, next_pose, actual in (
                (right, next_right, self._tcp_pose(q)),
                (left, next_left, self._left_pose(q))):
            linear = ((next_pose.translation - reference.translation) / self.dt
                      + 2.0 * (reference.translation - actual.translation))
            angular = (self.pin.log3(next_pose.rotation @ reference.rotation.T) / self.dt
                       + 2.0 * self.pin.log3(reference.rotation @ actual.rotation.T))
            for component, limit in ((linear, self.cartesian_max_speed),
                                     (angular, self.cartesian_max_angular_speed)):
                speed = float(np.linalg.norm(component))
                if speed > limit:
                    component *= limit / speed
            twists.append(np.r_[linear, angular])
        self.pin.computeJointJacobians(self.model, self.pose_data, q)
        self.pin.updateFramePlacements(self.model, self.pose_data)
        jacobians = [np.asarray(self.pin.getFrameJacobian(
            self.model, self.pose_data, frame, self.pin.LOCAL_WORLD_ALIGNED)).copy()
            for frame in (self.frame_id, self.left_frame_id)]
        jacobian = np.vstack(jacobians)
        twist = np.concatenate(twists)
        gram = jacobian @ jacobian.T + self.pose_velocity_feedforward_damping**2 * np.eye(12)
        velocity = jacobian.T @ np.linalg.solve(gram, twist)
        # Center pitch through task null space so it does not trade TCP accuracy for posture.
        centering = np.zeros(self.model.nv)
        centering[1] = -4.0 * (q[1] - self.torso_preferred[1])
        velocity += centering - jacobian.T @ np.linalg.solve(gram, jacobian @ centering)
        limits = np.minimum(self.command_velocity_limits, self.command_increment_limits / self.dt)
        return velocity / max(1.0, float(np.max(np.abs(velocity) / limits)))

    def _below_workspace_floor(self, q):
        if super()._below_workspace_floor(q):
            return True
        return (self.workspace_floor_enabled and hasattr(self, "left_frame_id")
                and self._left_pose(q).translation[2] < self.workspace_min_z)

    def _stationary_hold_ready(self, state, pose_error):
        left_error = self._left_error(state[:self.model.nq])
        if (np.linalg.norm(left_error[:3]) > self.stationary_hold_position_tolerance
                or np.linalg.norm(left_error[3:]) > self.stationary_hold_orientation_tolerance):
            self._stationary_hold_since = None
            self.stationary_hold_active = False
            return False
        return super()._stationary_hold_ready(state, pose_error)

    def _set_execution_enabled(self, enabled):
        super()._set_execution_enabled(enabled)
        if self.latest_state is not None and hasattr(self, "left_frame_id"):
            self.left_path_start = self._left_pose(self.latest_state[:self.model.nq])
            self.left_path_start_time = time.monotonic()

    def _safe_joint_command(self, predicted_q, state, velocity_reference=None):
        self._joint_limit_guard = ""
        measured = np.clip(state[:self.model.nq], self.model.lowerPositionLimit,
                           self.model.upperPositionLimit)
        right_error = self._pose_error(measured, self._optimization_target_pose(), self.pose_data)
        left_error = self._left_error(measured)
        position_error = max(np.linalg.norm(right_error[:3]), np.linalg.norm(left_error[:3]))
        orientation_error = max(np.linalg.norm(right_error[3:]), np.linalg.norm(left_error[3:]))
        if (self._precise_hold_target_count != self.target_update_count
                or position_error > 0.002 or orientation_error > 0.01):
            self._precise_hold_q = None
        if (self._precise_hold_q is None
                and position_error < 0.0005 and orientation_error < 0.002):
            # Latch one certified joint pose; delayed feedback must not replay
            # alternating old commands after the Cartesian goal is reached.
            self._precise_hold_q = measured.copy()
            self._precise_hold_target_count = self.target_update_count
            self.last_q_cmd = measured.copy()
        if self._precise_hold_q is not None:
            self._limited_joints = []
            self._publish_joint_limit_active()
            return super()._safe_joint_command(self._precise_hold_q, state)
        command = super()._safe_joint_command(predicted_q, state, velocity_reference)
        predicted = np.asarray(predicted_q, dtype=float)
        outside = ((predicted < self.model.lowerPositionLimit)
                   | (predicted > self.model.upperPositionLimit)
                   | (state[:self.model.nq] <= self.model.lowerPositionLimit + 1e-9)
                   | (state[:self.model.nq] >= self.model.upperPositionLimit - 1e-9))
        self._limited_joints = [name for name, active in zip(self.joint_names, outside) if active]
        if self._limited_joints:
            # The parent has already enforced hard position, velocity,
            # increment, path, and floor limits. Do not reject a feasible
            # escape step because its immediate task-space merit is worse.
            holding = np.max(np.abs(command - measured)) < 1e-9
            self._joint_limit_guard = "joint_position_limit_hold" if holding else "joint_position_limit_limited"
        self._publish_joint_limit_active()
        return command

    def _publish_joint_limit_active(self):
        self.joint_limit_active_pub.publish(String(data=",".join(self._limited_joints)))

    def _limit_task_command(self, command, measured_q, lower, upper):
        self._left_guard = ""
        self._mirror_guard = ""
        self.cartesian_path_guard = ""
        self.cartesian_command_scale = 1.0
        hand_tasks = (
            (lambda q: self._tcp_pose(q).translation.copy(), self._tcp_position_jacobian,
             self.cartesian_path_start_pose.translation, self._optimization_target_pose().translation),
            (lambda q: self._left_pose(q).translation, self._left_position_jacobian,
             self.left_path_start.translation, self._optimization_target_pose(self.left_target_pose).translation),
        )
        if self.cartesian_path_limit_enabled:
            mirror = np.array([1.0, -1.0, 1.0])
            mirror_position = lambda q: self._tcp_pose(q).translation - mirror * self._left_pose(q).translation
            mirror_start = hand_tasks[0][2] - mirror * hand_tasks[1][2]
            mirror_goal = hand_tasks[0][3] - mirror * hand_tasks[1][3]
            # Reanchoring one hand's line after feedback lag must not turn a
            # symmetric move into a path that permits one hand to lead.
            if np.linalg.norm(mirror_goal - self._mirror_home) < 0.001:
                mirror_start = mirror_goal.copy()
            tasks = (*hand_tasks, (
                mirror_position,
                self._mirror_position_jacobian,
                mirror_start, mirror_goal,
            ))
            original = command.copy()
            command = project_cartesian_tasks(command, lower, upper, tasks,
                                              self.cartesian_path_tolerance, measured_q=measured_q)
            # Correct the clipped whole-body command jointly, then certify both
            # paths. Later checks only shorten this same joint-linear step.
            for index, (position, _jacobian, start, goal) in enumerate(tasks):
                command, scale = limit_cartesian_step(
                    measured_q, command, position, start, goal,
                    self.cartesian_path_tolerance,
                    self.cartesian_reach_bound * (2 if index == 2 else 1),
                    allow_recovery=index == 2)
                if index == 0:
                    self.cartesian_command_scale = scale
                    if scale < 1 or not np.array_equal(command, original):
                        self.cartesian_path_guard = "cartesian_path_hold" if scale == 0 else "cartesian_path_limited"
                elif scale < 1:
                    if index == 1:
                        self._left_guard = "left_cartesian_path_hold" if scale == 0 else "left_cartesian_path_limited"
                    else:
                        self._mirror_guard = "mirror_path_hold" if scale == 0 else "mirror_path_limited"
        if self.workspace_floor_enabled:
            for index, (position, jacobian, _start, _goal) in enumerate(hand_tasks):
                command, scale = limit_height_step(
                    measured_q, command, position, jacobian,
                    self.workspace_min_z, self.cartesian_reach_bound)
                if scale < 1:
                    guard = "workspace_floor_hold" if scale == 0 else "workspace_floor_limited"
                    if index == 0:
                        self.workspace_floor_guard = guard
                    else:
                        self._left_guard = "left_" + guard
        return command

    def _left_position_jacobian(self, q):
        self.pin.computeJointJacobians(self.model, self.pose_data, q)
        self.pin.updateFramePlacements(self.model, self.pose_data)
        return self.pin.getFrameJacobian(self.model, self.pose_data, self.left_frame_id,
                                        self.pin.LOCAL_WORLD_ALIGNED)[:3].copy()

    def _mirror_position_jacobian(self, q):
        return self._tcp_position_jacobian(q) - np.array([1.0, -1.0, 1.0])[:, None] * self._left_position_jacobian(q)

    def _command_guard(self):
        return (getattr(self, "_mirror_guard", "")
                or getattr(self, "_left_guard", "") or super()._command_guard()
                or getattr(self, "_joint_limit_guard", ""))

    def _publish_left_pose(self):
        if self.latest_state is None:
            return
        self._publish_pose(self.left_current_pub, self._left_pose(self.latest_state[:self.model.nq]))
        self._publish_pose(self.left_target_pub, self.left_target_pose)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.pose_frame
        transform.child_frame_id = "left_hand_target"
        p, q = self.left_target_pose.translation, self.pin.Quaternion(self.left_target_pose.rotation).coeffs()
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = map(float, p)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = map(float, q)
        self.left_tf.sendTransform(transform)

    def _extra_diagnostic_record(self, state):
        return {"eir": {"left_hand": {"frame": "left_hand_tcp",
            "current_pose": self._pose_record(self._left_pose(state[:self.model.nq])),
            "target_pose": self._pose_record(self.left_target_pose),
            "error": self._left_error(state[:self.model.nq]), "guard": self._left_guard,
            "mirror_guard": self._mirror_guard,
            "path_start_position": self.left_path_start.translation,
            "path_duration_s": self.left_path_duration},
            "torso_preferred": self.torso_preferred, "torso_position": state[:2],
            "torso_cost_weight": self.torso_cost_weight,
            "torso_pitch_cost_weight": self.torso_pitch_cost_weight,
            "joint_position_limits": {"active_joints": self._limited_joints,
                                      "guard": self._joint_limit_guard}}}


def main(args=None):
    rclpy.init(args=args)
    node = EIRMPCNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
