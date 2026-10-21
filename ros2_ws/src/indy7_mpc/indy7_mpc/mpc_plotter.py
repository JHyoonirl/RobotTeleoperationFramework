#!/usr/bin/env python3
"""Live matplotlib dashboard for the Indy7 MPC diagnostics."""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class MpcPlotter(Node):
    """Plot tracking, every running cost term, solver health and limits."""

    def __init__(self) -> None:
        super().__init__("indy7_mpc_plotter")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("plot_period", 0.1)
        self.declare_parameter("history_seconds", 4.0)
        self.start_time = time.monotonic()
        self.history_seconds = float(self.get_parameter("history_seconds").value)
        self.plot_period = float(self.get_parameter("plot_period").value)
        if self.plot_period <= 0.0 or self.history_seconds <= 0.0:
            raise ValueError("plot_period와 history_seconds는 양수여야 합니다.")
        max_points = max(100, int(self.history_seconds / 0.005))
        self.history = {
            name: deque(maxlen=max_points)
            for name in (
                "position_error",
                "orientation_error",
                "cost",
                "se3_tracking",
                "cartesian_path",
                "workspace_floor",
                "posture",
                "velocity",
                "torque",
                "joint_limit",
                "terminal_cost",
                "solve_time_ms",
                "callback_time_ms",
                "solver_iterations",
                "solver_success",
                "control_norm",
                "joint_limit_margin",
            )
        }
        self.latest = {name: float("nan") for name in self.history}
        self.latest_status = ""
        self.lower, self.upper = self._read_joint_limits(
            str(self.get_parameter("urdf_path").value)
        )

        topics = {
            "position_error": "/mpc/position_error",
            "orientation_error": "/mpc/orientation_error",
            "cost": "/mpc/cost",
            "se3_tracking": "/mpc/cost/se3_tracking",
            "cartesian_path": "/mpc/cost/cartesian_path",
            "workspace_floor": "/mpc/cost/workspace_floor",
            "posture": "/mpc/cost/posture",
            "velocity": "/mpc/cost/velocity",
            "torque": "/mpc/cost/torque",
            "joint_limit": "/mpc/cost/joint_limit",
            "terminal_cost": "/mpc/terminal_cost",
            "solve_time_ms": "/mpc/solve_time_ms",
            "callback_time_ms": "/mpc/callback_time_ms",
            "solver_iterations": "/mpc/solver_iterations",
            "solver_success": "/mpc/solver_success",
            "control_norm": "/mpc/control_norm",
        }
        for name, topic in topics.items():
            self.create_subscription(
                Float64,
                topic,
                lambda message, key=name: self._record(key, message.data),
                10,
            )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(String, "/mpc/status", self._on_status, 10)

        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "실시간 plot에는 matplotlib가 필요합니다. python3-matplotlib를 설치하세요."
            ) from exc
        self.plt = plt
        self.plt.ion()
        self.figure, axes = self.plt.subplots(2, 2, figsize=(12, 7))
        manager = getattr(self.figure.canvas, "manager", None)
        if manager is not None and hasattr(manager, "set_window_title"):
            manager.set_window_title("Indy7 MPC monitor")
        self.ax_error, self.ax_cost, self.ax_solver, self.ax_control = axes.flat
        self.ax_orientation = self.ax_error.twinx()
        self.ax_iterations = self.ax_solver.twinx()
        self.ax_success = self.ax_solver.twinx()
        self.ax_success.spines["right"].set_position(("axes", 1.12))
        self.ax_margin = self.ax_control.twinx()
        self.lines = {
            "position_error": self.ax_error.plot([], [], label="position [m]", color="tab:blue")[0],
            "orientation_error": self.ax_orientation.plot([], [], label="orientation [deg]", color="tab:orange")[0],
            "cost": self.ax_cost.plot([], [], label="total cost", color="tab:red")[0],
            "solve_time_ms": self.ax_solver.plot([], [], label="solve time [ms]", color="tab:purple")[0],
            "callback_time_ms": self.ax_solver.plot([], [], label="callback time [ms]", color="tab:red")[0],
            "solver_iterations": self.ax_iterations.plot([], [], label="iterations", color="tab:green")[0],
            "solver_success": self.ax_success.plot([], [], label="success", color="tab:cyan", drawstyle="steps-post")[0],
            "control_norm": self.ax_control.plot([], [], label="|tau|", color="tab:brown")[0],
            "joint_limit_margin": self.ax_margin.plot([], [], label="min joint margin [rad]", color="tab:pink")[0],
        }
        cost_colors = {
            "se3_tracking": "tab:green",
            "cartesian_path": "tab:orange",
            "posture": "tab:olive",
            "velocity": "tab:cyan",
            "torque": "tab:purple",
            "joint_limit": "tab:pink",
            "terminal_cost": "tab:gray",
        }
        self.lines.update(
            {
                name: self.ax_cost.plot([], [], label=name, color=color, alpha=0.85)[0]
                for name, color in cost_colors.items()
            }
        )
        self.ax_error.set_title("TCP tracking error")
        self.ax_error.set_ylabel("position [m]")
        self.ax_orientation.set_ylabel("orientation [deg]")
        self.ax_cost.set_title("MPC cost")
        self.ax_cost.set_ylabel("cost")
        self.ax_solver.set_title("Solver health")
        self.ax_solver.set_ylabel("solve time [ms]")
        self.ax_iterations.set_ylabel("iterations")
        self.ax_success.set_ylabel("success")
        self.ax_control.set_title("Input and joint-limit state")
        self.ax_control.set_ylabel("control norm")
        self.ax_margin.set_ylabel("minimum margin [rad]")
        for axis in (self.ax_error, self.ax_cost, self.ax_solver, self.ax_control):
            axis.set_xlabel("time [s]")
            axis.grid(True, alpha=0.25)
        self._refresh_legend(self.ax_error, self.ax_orientation)
        self._refresh_legend(self.ax_cost)
        self._refresh_legend(self.ax_solver, self.ax_iterations, self.ax_success)
        self._refresh_legend(self.ax_control, self.ax_margin)
        self.ax_success.set_ylim(-0.05, 1.05)
        self.figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
        self.plt.show(block=False)
        matplotlib.rcParams["figure.raise_window"] = False
        self.timer = self.create_timer(self.plot_period, self._redraw)
        self.get_logger().info(
            "Live MPC plotter started: errors, cost, solver health, control norm, joint margin"
        )

    @staticmethod
    def _read_joint_limits(path: str):
        if not path or not Path(path).is_file():
            return [-math.pi] * 6, [math.pi] * 6
        root = ET.parse(path).getroot()
        lower, upper = [], []
        for index in range(6):
            joint = root.find(f"./joint[@name='joint{index}']")
            limit = None if joint is None else joint.find("limit")
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"joint{index}의 URDF position limit을 읽을 수 없습니다.")
            lower.append(float(limit.attrib["lower"]))
            upper.append(float(limit.attrib["upper"]))
        return lower, upper

    def _now(self) -> float:
        return time.monotonic() - self.start_time

    def _record(self, name: str, value: float) -> None:
        value = float(value)
        if not math.isfinite(value):
            return
        stamp = self._now()
        self.history[name].append((stamp, value))
        self.latest[name] = value

    def _on_joint_state(self, message: JointState) -> None:
        if message.name:
            by_name = {name: index for index, name in enumerate(message.name)}
            if any(f"joint{i}" not in by_name for i in range(6)):
                return
            if any(by_name[f"joint{i}"] >= len(message.position) for i in range(6)):
                return
            position = [message.position[by_name[f"joint{i}"]] for i in range(6)]
        elif len(message.position) >= 6:
            position = message.position[:6]
        else:
            return
        margin = min(
            min(float(q) - lo, hi - float(q))
            for q, lo, hi in zip(position, self.lower, self.upper)
        )
        self._record("joint_limit_margin", margin)

    @staticmethod
    def _refresh_legend(*axes) -> None:
        handles, labels = [], []
        for axis in axes:
            axis_handles, axis_labels = axis.get_legend_handles_labels()
            handles.extend(axis_handles)
            labels.extend(axis_labels)
        axes[0].legend(handles, labels, loc="upper left", fontsize="small")

    def _series(self, name: str):
        values = list(self.history[name])
        if not values:
            return [], []
        x, y = zip(*values)
        cutoff = x[-1] - self.history_seconds
        recent = [(stamp, value) for stamp, value in zip(x, y) if stamp >= cutoff]
        x, y = zip(*recent)
        if name == "orientation_error":
            y = tuple(math.degrees(value) for value in y)
        return x, y

    def _redraw(self) -> None:
        if not self.plt.fignum_exists(self.figure.number):
            rclpy.shutdown()
            return
        for name, line in self.lines.items():
            x, y = self._series(name)
            line.set_data(x, y)
        for axis in (
            self.ax_error,
            self.ax_orientation,
            self.ax_cost,
            self.ax_solver,
            self.ax_iterations,
            self.ax_success,
            self.ax_control,
            self.ax_margin,
        ):
            axis.relim()
            axis.autoscale_view()
        self.ax_success.set_ylim(-0.05, 1.05)
        status, color = self._status_label(self.latest_status, self.latest["solver_success"])
        self.figure.suptitle(
            f"Indy7 MPC | {status} | "
            f"pos={self.latest['position_error']:.4g} m | "
            f"ori={math.degrees(self.latest['orientation_error']):.3g} deg | "
            f"margin={self.latest['joint_limit_margin']:.3g} rad",
            color=color,
        )
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def _on_status(self, message: String) -> None:
        self.latest_status = message.data

    @staticmethod
    def _status_label(status: str, success: float) -> tuple[str, str]:
        if status == "cartesian_path_violation":
            return "PATH VIOLATION | HOLD", "tab:red"
        if status.startswith("workspace_floor_violation"):
            return "FLOOR VIOLATION | HOLD", "tab:red"
        if status.startswith("workspace_floor_hold"):
            return "FLOOR HOLD", "tab:red"
        if status.startswith("workspace_floor_limited"):
            suffix = "" if success >= 0.5 else " | SOLVER NOT CONVERGED"
            return "FLOOR LIMITED" + suffix, "tab:orange"
        if status in {"joint_state_stale", "paused_joint_state_stale"}:
            return "JOINT STATE STALE", "tab:red"
        if status == "cartesian_path_hold":
            return "PATH HOLD", "tab:orange"
        if status == "cartesian_path_limited":
            suffix = "" if success >= 0.5 else " | SOLVER NOT CONVERGED"
            return "PATH LIMITED" + suffix, "tab:orange"
        if status == "not_converged":
            return "SOLVER NOT CONVERGED", "tab:orange"
        if status in {"exception", "invalid_solution", "prediction_guard_hold"}:
            return "SOLVER FAILED", "tab:red"
        if not math.isfinite(success):
            return "waiting for MPC", "tab:gray"
        if success >= 0.5:
            return "solver OK", "tab:green"
        return "SOLVER NOT CONVERGED", "tab:orange"

    def destroy_node(self):
        if hasattr(self, "figure"):
            self.plt.close(self.figure)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MpcPlotter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
