#!/usr/bin/env python3
"""Guarded ROS 2 adapter between the Indy7 MPC and Neuromeka IndyDCP3."""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool
from .safety import DOF, load_joint_limits, percent_ratio, six, validate_joint_target


OP_IDLE = 5
TELE_OP = 17
TELE_JOINT_ABSOLUTE = 10


class Indy7Dcp3Node(Node):
    """Read robot state continuously; only move after arm plus a live deadman."""

    def __init__(self) -> None:
        super().__init__("indy7_dcp3")
        self.declare_parameter("robot_ip", "")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/mpc/joint_position_cmd")
        self.declare_parameter("deadman_topic", "/indy7_dcp3/deadman")
        self.declare_parameter("arm_service", "/indy7_dcp3/arm")
        self.declare_parameter("joint_names", [f"joint{i}" for i in range(DOF)])
        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("command_timeout", 0.30)
        self.declare_parameter("deadman_timeout", 0.30)
        self.declare_parameter("joint_limit_margin", 0.05)
        self.declare_parameter("velocity_ratio", 0.1)
        self.declare_parameter("acceleration_ratio", 1.0)

        self.robot_ip = str(self.get_parameter("robot_ip").value).strip()
        if not self.robot_ip:
            raise ValueError("robot_ip is required; pass it explicitly in the launch command")
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.deadman_topic = str(self.get_parameter("deadman_topic").value)
        self.arm_service = str(self.get_parameter("arm_service").value)
        self.joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        if len(self.joint_names) != DOF or len(set(self.joint_names)) != DOF:
            raise ValueError("joint_names must contain six unique names")

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.0
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout").value)
        self.limit_margin = float(self.get_parameter("joint_limit_margin").value)
        self.velocity_ratio = percent_ratio(
            self.get_parameter("velocity_ratio").value, "velocity_ratio"
        )
        self.acceleration_ratio = percent_ratio(
            self.get_parameter("acceleration_ratio").value, "acceleration_ratio"
        )
        if (
            self.period <= 0.0
            or not math.isfinite(self.period)
            or self.command_timeout <= 0.0
            or self.deadman_timeout <= 0.0
            or not math.isfinite(self.command_timeout)
            or not math.isfinite(self.deadman_timeout)
            or self.limit_margin < 0.0
            or not math.isfinite(self.limit_margin)
        ):
            raise ValueError("invalid DCP3 rate, timeout, or joint-limit margin")

        urdf = Path(get_package_share_directory("indy7_description")) / "urdf" / "indy7.urdf"
        self.lower, self.upper = load_joint_limits(urdf, self.joint_names)
        if np.any(self.lower + self.limit_margin >= self.upper - self.limit_margin):
            raise ValueError("joint_limit_margin leaves no usable joint range")

        try:
            from neuromeka import IndyDCP3
        except ImportError as exc:
            raise RuntimeError(
                "Neuromeka DCP3 client is missing; install the 'neuromeka' Python package"
            ) from exc
        self.client = IndyDCP3(self.robot_ip)

        self.armed = False
        self.current_q: np.ndarray | None = None
        self.current_qdot: np.ndarray | None = None
        self.current_op_state: int | None = None
        self.last_feedback_time = 0.0
        self.latest_command: np.ndarray | None = None
        self.last_command_time = 0.0
        self.deadman_value = False
        self.last_deadman_time = 0.0
        self.last_sent_q: np.ndarray | None = None

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.create_subscription(Float64MultiArray, self.command_topic, self._on_command, 10)
        self.create_subscription(Bool, self.deadman_topic, self._on_deadman, 10)
        self.create_service(SetBool, self.arm_service, self._on_arm)
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f"DCP3 client created for {self.robot_ip}; output is disarmed. "
            f"Arm via {self.arm_service} with a live deadman."
        )

    def _on_command(self, message: Float64MultiArray) -> None:
        try:
            self.latest_command = six(message.data, "MPC joint command")
            self.last_command_time = time.monotonic()
        except ValueError as exc:
            self.get_logger().error(f"Rejecting MPC command: {exc}")
            if self.armed:
                self._deactivate("invalid MPC command")

    def _on_deadman(self, message: Bool) -> None:
        self.deadman_value = bool(message.data)
        self.last_deadman_time = time.monotonic()
        if self.armed and not self.deadman_value:
            self._deactivate("deadman released")

    def _deadman_is_live(self, now: float) -> bool:
        return self.deadman_value and now - self.last_deadman_time <= self.deadman_timeout

    def _on_arm(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            stop_error = self._deactivate("operator disarmed", force_stop=True)
            response.success = stop_error is None
            response.message = (
                "DCP3 teleoperation stopped and disarmed"
                if stop_error is None
                else f"Disarmed locally, but DCP3 stop failed: {stop_error}"
            )
            return response
        if self.armed:
            response.success = True
            response.message = "DCP3 teleoperation is already armed"
            return response

        now = time.monotonic()
        if not self._deadman_is_live(now):
            response.message = "Refusing to arm: deadman is not continuously active"
            return response
        if self.latest_command is None or now - self.last_command_time > self.command_timeout:
            response.message = "Refusing to arm: no fresh MPC joint command"
            return response
        if self.current_q is None or now - self.last_feedback_time > self.command_timeout:
            response.message = "Refusing to arm: no fresh DCP3 joint-state feedback"
            return response
        if np.any(self.current_q < self.lower) or np.any(self.current_q > self.upper):
            response.message = "Refusing to arm: measured joint state is outside URDF limits"
            return response
        if self.current_op_state != OP_IDLE:
            response.message = f"Refusing to arm: DCP3 op_state={self.current_op_state}, expected IDLE(5)"
            return response
        try:
            validate_joint_target(
                self.current_q,
                self.latest_command,
                self.lower,
                self.upper,
                self.limit_margin,
            )
        except ValueError as exc:
            response.message = f"Refusing to arm: joint command safety check failed: {exc}"
            return response

        try:
            self.client.start_teleop(method=TELE_JOINT_ABSOLUTE)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if int(self.client.get_control_data()["op_state"]) == TELE_OP:
                    self.armed = True
                    self.last_sent_q = None
                    response.success = True
                    response.message = "DCP3 absolute joint teleoperation started"
                    self.get_logger().warning("DCP3 armed; keep the deadman active to move")
                    return response
                time.sleep(0.1)
            self._stop_dcp3()
            response.message = "DCP3 did not enter TELE_OP within 3 seconds"
        except Exception as exc:  # Vendor RPC exceptions vary by client version.
            self.get_logger().error(f"DCP3 arm failed: {exc}")
            try:
                self._stop_dcp3()
            except Exception:
                pass
            response.message = f"DCP3 arm failed: {exc}"
        return response

    def _tick(self) -> None:
        try:
            control = self.client.get_control_data()
            q = np.radians(six(control["q"], "DCP3 q feedback"))
            qdot = np.radians(six(control["qdot"], "DCP3 qdot feedback"))
            self.current_op_state = int(control["op_state"])
            self.current_q = q
            self.current_qdot = qdot
            now = time.monotonic()
            self.last_feedback_time = now
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = self.joint_names
            state.position = q.tolist()
            state.velocity = qdot.tolist()
            self.joint_state_pub.publish(state)
        except Exception as exc:  # Vendor RPC exceptions vary by client version.
            self.get_logger().error(
                f"DCP3 feedback failed: {exc}", throttle_duration_sec=2.0
            )
            if self.armed:
                self._deactivate("DCP3 feedback lost")
            return

        if not self.armed:
            return
        if not self._deadman_is_live(now):
            self._deactivate("deadman timed out")
            return
        if self.latest_command is None or now - self.last_command_time > self.command_timeout:
            self._deactivate("MPC command timed out")
            return
        if self.current_op_state != TELE_OP:
            self._deactivate(f"DCP3 left TELE_OP (op_state={self.current_op_state})")
            return

        try:
            target = validate_joint_target(
                q,
                self.latest_command,
                self.lower,
                self.upper,
                self.limit_margin,
            )
        except ValueError as exc:
            self._deactivate(f"joint command safety check failed: {exc}")
            return
        if np.max(np.abs(target - q)) <= 1.0e-7:
            return
        if self.last_sent_q is not None and np.max(np.abs(target - self.last_sent_q)) <= 1.0e-7:
            return
        try:
            self.client.movetelej_abs(
                jpos=np.degrees(target).tolist(),
                vel_ratio=self.velocity_ratio,
                acc_ratio=self.acceleration_ratio,
            )
            self.last_sent_q = target.copy()
        except Exception as exc:  # Vendor RPC exceptions vary by client version.
            self._deactivate(f"DCP3 motion command failed: {exc}")

    def _stop_dcp3(self) -> None:
        errors = []
        for stop_method in (self.client.stop_motion, self.client.stop_teleop):
            try:
                stop_method()
            except Exception as exc:  # Try both stop paths even if one RPC fails.
                errors.append(exc)
        if errors:
            raise RuntimeError("; ".join(str(error) for error in errors)) from errors[0]

    def _deactivate(
        self, reason: str, *, attempt_stop: bool = True, force_stop: bool = False
    ) -> Exception | None:
        was_armed = self.armed
        self.armed = False
        self.last_sent_q = None
        stop_error = None
        if attempt_stop and (was_armed or force_stop):
            try:
                self._stop_dcp3()
            except Exception as exc:  # Vendor RPC exceptions vary by client version.
                stop_error = exc
                self.get_logger().error(f"DCP3 stop request failed: {exc}")
        if was_armed:
            self.get_logger().warning(f"DCP3 disarmed: {reason}")
        return stop_error

    def destroy_node(self):
        if self.armed:
            self._deactivate("ROS node shutting down")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Indy7Dcp3Node()
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
