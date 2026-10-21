"""Optional MPC log serialization and diagnostic record helpers."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


class MpcDiagnosticsMixin:
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
                "joint_limit_cost_weight",
                "running_cost_weight",
                "terminal_cost_weight",
                "joint_limit_margin",
                "torque_limits",
                "diagnostic_acceleration_clip",
                "ik_max_iterations",
                "ik_damping",
                "ik_step",
                "diagnostic_ik_enabled",
                "diagnostic_logging",
                "diagnostic_rate",
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
            return MpcDiagnosticsMixin._json_safe(value.tolist())
        if isinstance(value, (np.floating, np.integer, np.bool_)):
            value = value.item()
        if isinstance(value, float):
            return value if np.isfinite(value) else None
        if isinstance(value, dict):
            return {
                str(key): MpcDiagnosticsMixin._json_safe(item)
                for key, item in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [MpcDiagnosticsMixin._json_safe(item) for item in value]
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
                    "acceleration_limit_enforced": False,
                    "diagnostic_acceleration_clip": self.last_target_acceleration,
                    # Kept for existing trace readers; this value is diagnostic only.
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

