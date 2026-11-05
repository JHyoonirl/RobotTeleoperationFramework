"""Small Crocoddyl action model for pose-space teleoperation MPC.

The frame residual and local Jacobian are evaluated explicitly before the
derivatives are handed to Crocoddyl's FDDP solver.
"""

from __future__ import annotations

import numpy as np


class SE3KinematicActionModel:
    """Factory for a Python-backed Crocoddyl differential action model."""

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
        acceleration_limits=None,
    ):
        # Defining the subclass here keeps the ROS node independent of the
        # Boost.Python base class at import time.
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
                    np.full(self.nv, np.inf)
                    if acceleration_limits is None
                    else np.asarray(acceleration_limits, dtype=float)
                )
                if limits.shape != (self.nv,) or np.any(~np.isfinite(limits)) or np.any(limits <= 0.0):
                    raise ValueError("acceleration_limits는 양수인 6개 유한값이어야 합니다.")
                self.u_lb = -limits
                self.u_ub = limits
                self.cost_names = (
                    "se3_tracking",
                    "posture",
                    "velocity",
                    "acceleration",
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
                # For r=log(M(q)^-1 T), -J_LOCAL is the exact first-order
                # derivative at zero error and a good Gauss-Newton Jacobian
                # during the receding-horizon rollout.
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
                acceleration = np.asarray(u, dtype=float)
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
                    "posture": scale
                    * 0.5
                    * self.weights["posture"]
                    * float((q - self.nominal_posture) @ (q - self.nominal_posture)),
                    "velocity": scale
                    * 0.5
                    * self.weights["velocity"]
                    * float(velocity @ velocity),
                    "acceleration": 0.0
                    if self.terminal
                    else scale
                    * 0.5
                    * self.weights["acceleration"]
                    * float(acceleration @ acceleration),
                }
                terms["joint_limit"] = scale * self._joint_limit_terms(q)[0]
                return terms, pose_error

            def cost_terms(self, x, u):
                terms, _ = self._terms(x, u)
                return dict(terms)

            def calc(self, data, x, u=None):
                control = np.zeros(self.nv) if u is None else np.asarray(u, dtype=float)
                data.xout[:] = control
                terms, _ = self._terms(x, control)
                self.last_cost_terms = terms
                data.cost = float(sum(terms.values()))

            def calcDiff(self, data, x, u=None):
                control = np.zeros(self.nv) if u is None else np.asarray(u, dtype=float)
                q = np.asarray(x[: self.nq], dtype=float)
                velocity = np.asarray(x[self.nq : self.nq + self.nv], dtype=float)
                terms, pose_error = self._terms(x, control)
                self.last_cost_terms = terms
                scale = self.terminal_scale
                position_error = pose_error[:3]
                orientation_error = pose_error[3:]
                pose_jacobian = self._pose_jacobian(q)
                position_jacobian = pose_jacobian[:3]
                orientation_jacobian = pose_jacobian[3:]

                data.Fx[:] = 0.0
                data.Fu[:] = np.eye(self.nv)
                data.Lx[:] = 0.0
                data.Lu[:] = 0.0
                data.Lxx[:] = 0.0
                data.Lxu[:] = 0.0
                data.Luu[:] = 0.0

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

                posture_weight = scale * self.weights["posture"]
                posture_error = q - self.nominal_posture
                data.Lx[: self.nq] += posture_weight * posture_error
                data.Lxx[: self.nq, : self.nq] += posture_weight * np.eye(self.nq)

                velocity_weight = scale * self.weights["velocity"]
                data.Lx[self.nq : self.nq + self.nv] += velocity_weight * velocity
                data.Lxx[self.nq : self.nq + self.nv, self.nq : self.nq + self.nv] += (
                    velocity_weight * np.eye(self.nv)
                )

                if not self.terminal:
                    acceleration_weight = scale * self.weights["acceleration"]
                    data.Lu[:] = acceleration_weight * control
                    data.Luu[:] = acceleration_weight * np.eye(self.nv)

                _, limit_gradient, limit_hessian = self._joint_limit_terms(q)
                data.Lx[: self.nq] += scale * limit_gradient
                data.Lxx[: self.nq, : self.nq] += scale * np.diag(limit_hessian)

                # Keep the Gauss-Newton Hessian positive definite at singular poses.
                data.Lxx[:] += 1.0e-6 * np.eye(self.nq + self.nv)
                data.Luu[:] += 1.0e-6 * np.eye(self.nv)

        return _ActionModel()
