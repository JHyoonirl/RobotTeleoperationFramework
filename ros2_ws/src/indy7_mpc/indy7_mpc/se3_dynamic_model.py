"""URDF-backed rigid-body action model for the Indy7 MPC.

The public command boundary remains joint position, while the Crocoddyl
prediction uses Pinocchio's articulated-body algorithm internally.
"""

from __future__ import annotations

import numpy as np


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
    ):
        class _ActionModel(crocoddyl.DifferentialActionModelAbstract):
            def __init__(self):
                super().__init__(state, model.nv, 0)
                self.pin = pin
                self.model = model
                self.frame_id = int(frame_id)
                self.target = target.copy()
                self.nominal_posture = np.asarray(nominal_posture, dtype=float).copy()
                self.weights = {name: float(value) for name, value in weights.items()}
                self.terminal_scale = float(terminal_scale)
                self.terminal = bool(terminal)
                self.pin_data = model.createData()
                self.nq = int(model.nq)
                self.nv = int(model.nv)
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
                    "posture",
                    "velocity",
                    "torque",
                    "joint_limit",
                )
                self.last_cost_terms = {name: 0.0 for name in self.cost_names}

            def update_target(self, target):
                self.target = target.copy()

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
                    * float(velocity @ velocity),
                    "torque": 0.0
                    if self.terminal
                    else self.terminal_scale
                    * 0.5
                    * self.weights["torque"]
                    * float(torque @ torque),
                }
                terms["joint_limit"] = self.terminal_scale * self._joint_limit_terms(q)[0]
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

                posture_weight = self.terminal_scale * self.weights["posture"]
                posture_error = q - self.nominal_posture
                data.Lx[: self.nq] += posture_weight * posture_error
                data.Lxx[: self.nq, : self.nq] += posture_weight * np.eye(self.nq)

                velocity_weight = self.terminal_scale * self.weights["velocity"]
                data.Lx[self.nq : self.nq + self.nv] += velocity_weight * velocity
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
