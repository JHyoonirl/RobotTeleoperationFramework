"""URDF-backed rigid-body action model for the Indy7 MPC.

The public command boundary remains joint position, while the Crocoddyl
prediction uses Pinocchio's articulated-body algorithm internally.
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
        minimum_z=None,
        floor_margin=0.001,
        floor_weight=1.0e6,
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
                if path_tolerance is not None and (
                    not np.isfinite(path_tolerance) or path_tolerance <= 0.0
                ):
                    raise ValueError("path_tolerance must be positive and finite")
                self.path_weight = 0.0 if path_tolerance is None else 1.0 / path_tolerance**2
                if (minimum_z is not None and not np.isfinite(minimum_z)) or (
                    not np.isfinite(floor_margin) or floor_margin <= 0.0
                    or not np.isfinite(floor_weight) or floor_weight <= 0.0
                ):
                    raise ValueError("Floor height must be finite; margin and weight must be positive and finite")
                self.minimum_z = minimum_z
                self.floor_margin = float(floor_margin)
                self.floor_weight = float(floor_weight)
                self.nominal_posture = np.asarray(nominal_posture, dtype=float).copy()
                self.weights = {name: float(value) for name, value in weights.items()}
                self.terminal_scale = float(terminal_scale)
                self.terminal = bool(terminal)
                self.pin_data = model.createData()
                self.nq = int(model.nq)
                self.nv = int(model.nv)
                self.velocity_reference = np.zeros(self.nv)
                margin = float(joint_limit_margin)
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
                    raise ValueError("torque_limits는 양수인 6개 유한값이어야 합니다.")
                self.torque_limits = np.minimum(
                    limits, np.asarray(model.effortLimit, dtype=float)
                )
                self.u_lb = -self.torque_limits
                self.u_ub = self.torque_limits
                self.cost_names = (
                    "se3_tracking",
                    "cartesian_path",
                    "workspace_floor",
                    "posture",
                    "velocity",
                    "torque",
                    "joint_limit",
                )
                self.last_cost_terms = {name: 0.0 for name in self.cost_names}

            def update_target(self, target):
                self.target = target.copy()

            def update_path(self, start, goal):
                self.path_start = np.asarray(start, dtype=float).copy()
                self.path_goal = np.asarray(goal, dtype=float).copy()

            def update_velocity_reference(self, velocity):
                velocity = np.asarray(velocity, dtype=float)
                if velocity.shape != (self.nv,) or not np.all(np.isfinite(velocity)):
                    raise ValueError("velocity reference must be finite and match nv")
                self.velocity_reference = velocity.copy()

            def _path_error(self):
                return segment_error(
                    self.pin_data.oMf[self.frame_id].translation,
                    self.path_start,
                    self.path_goal,
                )

            def _floor_error(self):
                if self.minimum_z is None:
                    return 0.0
                height = float(self.pin_data.oMf[self.frame_id].translation[2])
                return min(0.0, height - self.minimum_z - self.floor_margin)

            def _pose_error(self, q):
                self.pin.forwardKinematics(self.model, self.pin_data, q)
                self.pin.updateFramePlacements(self.model, self.pin_data)
                relative = self.pin_data.oMf[self.frame_id].inverse() * self.target
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

            def _pose_jacobian(self, q):
                self.pin.computeJointJacobians(self.model, self.pin_data, q)
                self.pin.updateFramePlacements(self.model, self.pin_data)
                return -self.pin.getFrameJacobian(
                    self.model, self.pin_data, self.frame_id, self.pin.LOCAL
                )

            def _joint_limit_terms(self, q):
                lower_violation = np.maximum(self.soft_lower - q, 0.0)
                upper_violation = np.maximum(q - self.soft_upper, 0.0)
                violation = lower_violation + upper_violation
                weight = self.weights["joint_limit"]
                cost = 0.5 * weight * float(violation @ violation)
                gradient = weight * (upper_violation - lower_violation)
                hessian = weight * (violation > 0.0).astype(float)
                return cost, gradient, hessian

            def _terms(self, x, u):
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                torque = np.asarray(u, dtype=float)
                pose_error = self._pose_error(q)
                position_error = pose_error[:3]
                orientation_error = pose_error[3:]
                velocity_error = velocity - self.velocity_reference
                scale = self.terminal_scale
                terms = {
                    "se3_tracking": scale
                    * 0.5
                    * (
                        self.weights["se3_position"] * float(position_error @ position_error)
                        + self.weights["se3_orientation"]
                        * float(orientation_error @ orientation_error)
                    ),
                    "posture": self.terminal_scale
                    * 0.5
                    * self.weights["posture"]
                    * float((q - self.nominal_posture) @ (q - self.nominal_posture)),
                    "velocity": self.terminal_scale
                    * 0.5
                    * self.weights["velocity"]
                    * float(velocity_error @ velocity_error),
                    "torque": 0.0
                    if self.terminal
                    else self.terminal_scale
                    * 0.5
                    * self.weights["torque"]
                    * float(torque @ torque),
                }
                terms["joint_limit"] = self.terminal_scale * self._joint_limit_terms(q)[0]
                path_error, _ = self._path_error()
                terms["cartesian_path"] = (
                    scale * 0.5 * self.path_weight * float(path_error @ path_error)
                )
                terms["workspace_floor"] = scale * 0.5 * self.floor_weight * self._floor_error() ** 2
                return terms, pose_error

            def cost_terms(self, x, u):
                terms, _ = self._terms(x, u)
                return dict(terms)

            def dynamics_terms(self, x, u):
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                torque = np.asarray(u, dtype=float)
                acceleration = self.pin.aba(
                    self.model, self.pin_data, q, velocity, torque
                )
                mass = self.pin.crba(self.model, self.pin_data, q)
                mass = np.triu(mass) + np.triu(mass, 1).T
                nonlinear = self.pin.nle(self.model, self.pin_data, q, velocity)
                gravity = self.pin.computeGeneralizedGravity(
                    self.model, self.pin_data, q
                )
                return {
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
                data.xout[:] = self.pin.aba(
                    self.model, self.pin_data, q, velocity, torque
                )
                terms, _ = self._terms(x, torque)
                self.last_cost_terms = terms
                data.cost = float(sum(terms.values()))

            def calcDiff(self, data, x, u=None):
                torque = np.zeros(self.nv) if u is None else np.asarray(u, dtype=float)
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                terms, pose_error = self._terms(x, torque)
                self.last_cost_terms = terms
                self.pin.computeABADerivatives(
                    self.model, self.pin_data, q, velocity, torque
                )
                data.Fx[:] = 0.0
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
                position_error = pose_error[:3]
                orientation_error = pose_error[3:]
                pose_jacobian = self._pose_jacobian(q)
                position_jacobian = pose_jacobian[:3]
                orientation_jacobian = pose_jacobian[3:]
                position_weight = scale * self.weights["se3_position"]
                orientation_weight = scale * self.weights["se3_orientation"]
                data.Lx[: self.nq] += position_weight * position_jacobian.T @ position_error
                data.Lx[: self.nq] += orientation_weight * orientation_jacobian.T @ orientation_error
                data.Lxx[: self.nq, : self.nq] += position_weight * (
                    position_jacobian.T @ position_jacobian
                )
                data.Lxx[: self.nq, : self.nq] += orientation_weight * (
                    orientation_jacobian.T @ orientation_jacobian
                )

                path_error, projection = self._path_error()
                world_position_jacobian = (
                    -self.pin_data.oMf[self.frame_id].rotation @ position_jacobian
                )
                path_jacobian = projection @ world_position_jacobian
                path_weight = scale * self.path_weight
                data.Lx[: self.nq] += path_weight * path_jacobian.T @ path_error
                data.Lxx[: self.nq, : self.nq] += path_weight * (
                    path_jacobian.T @ path_jacobian
                )

                floor_error = self._floor_error()
                if floor_error < 0.0:
                    height_jacobian = world_position_jacobian[2]
                    floor_weight = scale * self.floor_weight
                    data.Lx[: self.nq] += floor_weight * floor_error * height_jacobian
                    data.Lxx[: self.nq, : self.nq] += floor_weight * np.outer(
                        height_jacobian, height_jacobian
                    )

                posture_weight = self.terminal_scale * self.weights["posture"]
                posture_error = q - self.nominal_posture
                data.Lx[: self.nq] += posture_weight * posture_error
                data.Lxx[: self.nq, : self.nq] += posture_weight * np.eye(self.nq)

                velocity_weight = self.terminal_scale * self.weights["velocity"]
                data.Lx[self.nq : self.nq + self.nv] += velocity_weight * (
                    velocity - self.velocity_reference
                )
                data.Lxx[self.nq : self.nq + self.nv, self.nq : self.nq + self.nv] += (
                    velocity_weight * np.eye(self.nv)
                )

                if not self.terminal:
                    torque_weight = self.terminal_scale * self.weights["torque"]
                    data.Lu[:] = torque_weight * torque
                    data.Luu[:] = torque_weight * np.eye(self.nv)

                _, limit_gradient, limit_hessian = self._joint_limit_terms(q)
                data.Lx[: self.nq] += self.terminal_scale * limit_gradient
                data.Lxx[: self.nq, : self.nq] += self.terminal_scale * np.diag(limit_hessian)
                data.Lxx[:] += 1.0e-6 * np.eye(self.nq + self.nv)
                data.Luu[:] += 1.0e-6 * np.eye(self.nv)

        return _ActionModel()
