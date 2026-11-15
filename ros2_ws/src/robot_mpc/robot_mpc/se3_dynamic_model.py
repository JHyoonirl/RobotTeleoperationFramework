"""URDF-backed task costs with rigid-body or position-servo prediction.

The public command boundary remains joint position, while the Crocoddyl
rigid-body prediction uses Pinocchio ABA. Position-servo prediction instead
uses bounded joint acceleration and does not claim actuator torque feasibility.
"""

from __future__ import annotations

import numpy as np

from .cartesian_path import segment_error


class SE3DynamicActionModel:
    """Factory for a Python Crocoddyl differential action model."""

    def __new__(
        cls,
        *,
        crocoddyl,
        pin,
        model,
        state,
        frame_id,
        target,
        nominal_posture,
        weights,
        terminal_scale=1.0,
        terminal=False,
        joint_limit_margin=0.2,
        torque_limits=None,
        path_tolerance=None,
        path_cost_weight=None,
        minimum_z=None,
        floor_margin=0.001,
        floor_weight=1.0e6,
        prediction_model="rigid_body",
        acceleration_limits=None,
    ):
        class _ActionModel(crocoddyl.DifferentialActionModelAbstract):
            def __init__(self):
                super().__init__(state, model.nv, 0)
                self.pin = pin
                self.model = model
                self.frame_id = int(frame_id)
                self.target = target.copy()
                self.path_start = target.translation.copy()
                self.path_goal = target.translation.copy()
                self.secondary_frame_id = None
                self.secondary_target = None
                self.secondary_path_start = None
                self.secondary_path_goal = None
                if path_tolerance is not None and (
                    not np.isfinite(path_tolerance) or path_tolerance <= 0.0
                ):
                    raise ValueError("path_tolerance must be positive and finite")
                if path_cost_weight is not None and (not np.isfinite(path_cost_weight) or path_cost_weight <= 0):
                    raise ValueError("path_cost_weight must be positive and finite")
                self.path_weight = (0.0 if path_tolerance is None else
                                    1.0 / path_tolerance**2 if path_cost_weight is None else float(path_cost_weight))
                if (minimum_z is not None and not np.isfinite(minimum_z)) or (
                    not np.isfinite(floor_margin) or floor_margin <= 0.0
                    or not np.isfinite(floor_weight) or floor_weight <= 0.0
                ):
                    raise ValueError("Floor height must be finite; margin and weight must be positive and finite")
                self.minimum_z = minimum_z
                self.floor_margin = float(floor_margin)
                self.floor_weight = float(floor_weight)
                self.nominal_posture = np.asarray(nominal_posture, dtype=float).copy()
                self.posture_weights = None
                self.weights = {name: float(value) for name, value in weights.items()}
                self.terminal_scale = float(terminal_scale)
                self.terminal = bool(terminal)
                if prediction_model not in {"rigid_body", "joint_acceleration"}:
                    raise ValueError("Unknown MPC prediction_model")
                self.prediction_model = prediction_model
                self.acceleration_input = prediction_model == "joint_acceleration"
                self.control_cost_name = "acceleration" if self.acceleration_input else "torque"
                self.pin_data = model.createData()
                self.nq = int(model.nq)
                self.nv = int(model.nv)
                self.velocity_reference = np.zeros(self.nv)
                margin = float(joint_limit_margin)
                self.joint_limit_margin = margin
                self.soft_lower = model.lowerPositionLimit + margin
                self.soft_upper = model.upperPositionLimit - margin
                if np.any(self.soft_lower >= self.soft_upper):
                    raise ValueError("joint_limit_margin이 관절 허용 범위보다 큽니다.")
                limits = (
                    np.asarray(model.effortLimit, dtype=float)
                    if torque_limits is None
                    else np.asarray(torque_limits, dtype=float)
                )
                if (
                    limits.shape != (self.nv,)
                    or np.any(~np.isfinite(limits))
                    or np.any(limits <= 0.0)
                ):
                    raise ValueError("torque_limits must contain one positive finite value per joint")
                self.torque_limits = np.minimum(
                    limits, np.asarray(model.effortLimit, dtype=float)
                )
                self.control_limits = self.torque_limits
                if self.acceleration_input:
                    self.control_limits = np.asarray(acceleration_limits, dtype=float)
                    if (self.control_limits.shape != (self.nv,)
                            or np.any(~np.isfinite(self.control_limits))
                            or np.any(self.control_limits <= 0)):
                        raise ValueError("acceleration_limits must contain one positive finite value per joint")
                self.u_lb = -self.control_limits
                self.u_ub = self.control_limits
                self.cost_names = (
                    "se3_tracking",
                    "cartesian_path",
                    "workspace_floor",
                    "posture",
                    "velocity",
                    self.control_cost_name,
                    "joint_limit",
                )
                self.last_cost_terms = {name: 0.0 for name in self.cost_names}

            def update_target(self, target):
                self.target = target.copy()

            def update_path(self, start, goal):
                self.path_start = np.asarray(start, dtype=float).copy()
                self.path_goal = np.asarray(goal, dtype=float).copy()

            def update_secondary_task(self, frame_id, target=None, path_start=None, path_goal=None):
                if frame_id is None:
                    self.secondary_frame_id = None
                    self.secondary_target = None
                    self.secondary_path_start = None
                    self.secondary_path_goal = None
                    return
                if (
                    not isinstance(frame_id, (int, np.integer))
                    or not 0 <= frame_id < self.model.nframes
                ):
                    raise ValueError("secondary task frame_id must identify a model frame")
                if target is None or (
                    not np.all(np.isfinite(target.translation))
                    or not np.all(np.isfinite(target.rotation))
                ):
                    raise ValueError("secondary target must be a finite SE3 pose")
                start = np.asarray(target.translation if path_start is None else path_start, dtype=float)
                goal = np.asarray(target.translation if path_goal is None else path_goal, dtype=float)
                if any(point.shape != (3,) or not np.all(np.isfinite(point)) for point in (start, goal)):
                    raise ValueError("secondary path endpoints must contain three finite values")
                self.secondary_frame_id = int(frame_id)
                self.secondary_target = target.copy()
                self.secondary_path_start = start.copy()
                self.secondary_path_goal = goal.copy()

            def update_posture_weights(self, weights):
                if weights is None:
                    self.posture_weights = None
                    return
                weights = np.asarray(weights, dtype=float)
                if weights.shape != (self.nq,) or np.any(~np.isfinite(weights)) or np.any(weights < 0.0):
                    raise ValueError("posture weights must contain one finite nonnegative value per joint")
                self.posture_weights = weights.copy()

            def update_velocity_reference(self, velocity):
                velocity = np.asarray(velocity, dtype=float)
                if velocity.shape != (self.nv,) or not np.all(np.isfinite(velocity)):
                    raise ValueError("velocity reference must be finite and match nv")
                self.velocity_reference = velocity.copy()

            def _path_error(self, frame_id, start, goal):
                return segment_error(
                    self.pin_data.oMf[frame_id].translation,
                    start,
                    goal,
                )

            def _floor_error(self, frame_id):
                if self.minimum_z is None:
                    return 0.0
                height = float(self.pin_data.oMf[frame_id].translation[2])
                return min(0.0, height - self.minimum_z - self.floor_margin)

            def _task_specs(self):
                tasks = [(self.frame_id, self.target, self.path_start, self.path_goal)]
                if self.secondary_frame_id is not None:
                    tasks.append((self.secondary_frame_id, self.secondary_target,
                                  self.secondary_path_start, self.secondary_path_goal))
                return tasks

            def _pose_error(self, frame_id, target):
                relative = self.pin_data.oMf[frame_id].inverse() * target
                return np.concatenate((relative.translation, self.pin.log3(relative.rotation)))

            def _pose_jacobian(self, frame_id):
                return -self.pin.getFrameJacobian(
                    self.model, self.pin_data, frame_id, self.pin.LOCAL
                )

            def _joint_limit_terms(self, q):
                lower_violation = (
                    np.maximum(self.soft_lower - q, 0.0) / self.joint_limit_margin
                )
                upper_violation = (
                    np.maximum(q - self.soft_upper, 0.0) / self.joint_limit_margin
                )
                violation = lower_violation + upper_violation
                weight = self.weights["joint_limit"]
                cost = 0.5 * weight * float(violation @ violation)
                gradient = (
                    weight * (upper_violation - lower_violation) / self.joint_limit_margin
                )
                hessian = (
                    weight * (violation > 0.0).astype(float) / self.joint_limit_margin**2
                )
                return cost, gradient, hessian

            def _terms(self, x, u):
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                torque = np.asarray(u, dtype=float)
                # The input penalty follows the selected input units; gravity
                # compensation belongs only to the rigid-body torque model.
                gravity = (np.zeros(self.nv) if self.acceleration_input else
                           self.pin.computeGeneralizedGravity(self.model, self.pin_data, q))
                torque_error = torque - gravity
                self.pin.forwardKinematics(self.model, self.pin_data, q)
                self.pin.updateFramePlacements(self.model, self.pin_data)
                velocity_error = velocity - self.velocity_reference
                posture_error = q - self.nominal_posture
                posture_weights = self.weights["posture"] if self.posture_weights is None else self.posture_weights
                scale = self.terminal_scale
                terms = {
                    "se3_tracking": 0.0,
                    "posture": self.terminal_scale
                    * 0.5
                    * float((posture_weights * posture_error) @ posture_error),
                    "velocity": self.terminal_scale
                    * 0.5
                    * self.weights["velocity"]
                    * float(velocity_error @ velocity_error),
                    self.control_cost_name: 0.0
                    if self.terminal
                    else self.terminal_scale
                    * 0.5
                    * self.weights["torque"]
                    * float(torque_error @ torque_error),
                }
                terms["joint_limit"] = self.terminal_scale * self._joint_limit_terms(q)[0]
                terms["cartesian_path"] = 0.0
                terms["workspace_floor"] = 0.0
                task_errors = []
                for frame_id, target, start, goal in self._task_specs():
                    pose_error = self._pose_error(frame_id, target)
                    task_errors.append((frame_id, pose_error, start, goal))
                    terms["se3_tracking"] += scale * 0.5 * (
                        self.weights["se3_position"] * float(pose_error[:3] @ pose_error[:3])
                        + self.weights["se3_orientation"] * float(pose_error[3:] @ pose_error[3:])
                    )
                    path_error, _ = self._path_error(frame_id, start, goal)
                    terms["cartesian_path"] += scale * 0.5 * self.path_weight * float(path_error @ path_error)
                    terms["workspace_floor"] += scale * 0.5 * self.floor_weight * self._floor_error(frame_id) ** 2
                return terms, task_errors

            def cost_terms(self, x, u):
                terms, _ = self._terms(x, u)
                return dict(terms)

            def dynamics_terms(self, x, u):
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                torque = np.asarray(u, dtype=float)
                acceleration = (torque.copy() if self.acceleration_input else self.pin.aba(
                    self.model, self.pin_data, q, velocity, torque))
                mass = self.pin.crba(self.model, self.pin_data, q)
                mass = np.triu(mass) + np.triu(mass, 1).T
                nonlinear = self.pin.nle(self.model, self.pin_data, q, velocity)
                gravity = self.pin.computeGeneralizedGravity(
                    self.model, self.pin_data, q
                )
                return {
                    "prediction_model": self.prediction_model,
                    "mass_matrix": mass,
                    "nonlinear_effects": nonlinear,
                    "gravity": gravity,
                    "predicted_acceleration": acceleration,
                    "kinetic_energy": float(
                        self.pin.computeKineticEnergy(
                            self.model, self.pin_data, q, velocity
                        )
                    ),
                    "potential_energy": float(
                        self.pin.computePotentialEnergy(self.model, self.pin_data, q)
                    ),
                }

            def calc(self, data, x, u=None):
                torque = np.zeros(self.nv) if u is None else np.asarray(u, dtype=float)
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                data.xout[:] = (torque if self.acceleration_input else self.pin.aba(
                    self.model, self.pin_data, q, velocity, torque))
                terms, _ = self._terms(x, torque)
                self.last_cost_terms = terms
                data.cost = float(sum(terms.values()))

            def calcDiff(self, data, x, u=None):
                torque = np.zeros(self.nv) if u is None else np.asarray(u, dtype=float)
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                data.Fx[:] = 0.0
                if self.acceleration_input:
                    data.Fu[:] = np.eye(self.nv)
                else:
                    self.pin.computeABADerivatives(self.model, self.pin_data, q, velocity, torque)
                    data.Fx[:, : self.nq] = self.pin_data.ddq_dq
                    data.Fx[:, self.nq : self.nq + self.nv] = self.pin_data.ddq_dv
                    # Pinocchio 4 exposes the torque derivative through Minv.
                    data.Fu[:] = self.pin_data.Minv
                data.Lx[:] = 0.0
                data.Lu[:] = 0.0
                data.Lxx[:] = 0.0
                data.Lxu[:] = 0.0
                data.Luu[:] = 0.0

                scale = self.terminal_scale
                self.pin.computeJointJacobians(self.model, self.pin_data, q)
                self.pin.updateFramePlacements(self.model, self.pin_data)
                position_weight = scale * self.weights["se3_position"]
                orientation_weight = scale * self.weights["se3_orientation"]
                path_weight = scale * self.path_weight
                # Jacobians already update kinematics; do not evaluate every cost again.
                for frame_id, target, start, goal in self._task_specs():
                    pose_error = self._pose_error(frame_id, target)
                    pose_jacobian = self._pose_jacobian(frame_id)
                    position_jacobian = pose_jacobian[:3]
                    orientation_jacobian = pose_jacobian[3:]
                    data.Lx[: self.nq] += position_weight * position_jacobian.T @ pose_error[:3]
                    data.Lx[: self.nq] += orientation_weight * orientation_jacobian.T @ pose_error[3:]
                    data.Lxx[: self.nq, : self.nq] += position_weight * (
                        position_jacobian.T @ position_jacobian
                    )
                    data.Lxx[: self.nq, : self.nq] += orientation_weight * (
                        orientation_jacobian.T @ orientation_jacobian
                    )

                    path_error, projection = self._path_error(frame_id, start, goal)
                    world_position_jacobian = -self.pin_data.oMf[frame_id].rotation @ position_jacobian
                    path_jacobian = projection @ world_position_jacobian
                    data.Lx[: self.nq] += path_weight * path_jacobian.T @ path_error
                    data.Lxx[: self.nq, : self.nq] += path_weight * (
                        path_jacobian.T @ path_jacobian
                    )

                    floor_error = self._floor_error(frame_id)
                    if floor_error < 0.0:
                        height_jacobian = world_position_jacobian[2]
                        floor_weight = scale * self.floor_weight
                        data.Lx[: self.nq] += floor_weight * floor_error * height_jacobian
                        data.Lxx[: self.nq, : self.nq] += floor_weight * np.outer(
                            height_jacobian, height_jacobian
                        )

                posture_weights = self.weights["posture"] if self.posture_weights is None else self.posture_weights
                posture_weights = self.terminal_scale * np.broadcast_to(posture_weights, (self.nq,))
                posture_error = q - self.nominal_posture
                data.Lx[: self.nq] += posture_weights * posture_error
                data.Lxx[: self.nq, : self.nq] += np.diag(posture_weights)

                velocity_weight = self.terminal_scale * self.weights["velocity"]
                data.Lx[self.nq : self.nq + self.nv] += velocity_weight * (
                    velocity - self.velocity_reference
                )
                data.Lxx[self.nq : self.nq + self.nv, self.nq : self.nq + self.nv] += (
                    velocity_weight * np.eye(self.nv)
                )

                if not self.terminal:
                    torque_weight = self.terminal_scale * self.weights["torque"]
                    gravity = (np.zeros(self.nv) if self.acceleration_input else
                               self.pin.computeGeneralizedGravity(self.model, self.pin_data, q).copy())
                    gravity_jacobian = (np.zeros((self.nv, self.nq)) if self.acceleration_input else
                                        self.pin.computeGeneralizedGravityDerivatives(self.model, self.pin_data, q))
                    torque_error = torque - gravity
                    data.Lx[: self.nq] -= torque_weight * gravity_jacobian.T @ torque_error
                    data.Lxx[: self.nq, : self.nq] += torque_weight * (
                        gravity_jacobian.T @ gravity_jacobian
                    )
                    data.Lxu[: self.nq, :] = -torque_weight * gravity_jacobian.T
                    data.Lu[:] = torque_weight * torque_error
                    data.Luu[:] = torque_weight * np.eye(self.nv)

                _, limit_gradient, limit_hessian = self._joint_limit_terms(q)
                data.Lx[: self.nq] += self.terminal_scale * limit_gradient
                data.Lxx[: self.nq, : self.nq] += self.terminal_scale * np.diag(limit_hessian)
                data.Lxx[:] += 1.0e-6 * np.eye(self.nq + self.nv)
                data.Luu[:] += 1.0e-6 * np.eye(self.nv)

        return _ActionModel()
