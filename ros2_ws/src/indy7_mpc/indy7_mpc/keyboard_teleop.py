#!/usr/bin/env python3
"""Keyboard teleoperation for the Indy7 SE(3) target pose."""

from __future__ import annotations

import ast
import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def _vector(value: object, size: int, name: str) -> list[float]:
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError) as exc:
            raise ValueError(f"{name} 형식이 잘못되었습니다: {value}") from exc
    result = [float(item) for item in value]
    if len(result) != size or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name}은 유한한 {size}개 값이어야 합니다.")
    return result


def _normalize(quaternion: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError("quaternion의 크기가 0입니다.")
    return [value / norm for value in quaternion]


def _multiply_quaternions(left: list[float], right: list[float]) -> list[float]:
    """Hamilton product for [x, y, z, w] quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def _axis_quaternion(axis: int, angle: float) -> list[float]:
    half = 0.5 * angle
    component = math.sin(half)
    quaternion = [0.0, 0.0, 0.0, math.cos(half)]
    quaternion[axis] = component
    return quaternion


class KeyboardTeleop(Node):
    """Publish a keyboard-adjusted desired TCP pose."""

    def __init__(self) -> None:
        super().__init__("indy7_keyboard_teleop")
        self.declare_parameter("topic", "/teleop/target_pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("translation_step", 0.01)
        self.declare_parameter("rotation_step_deg", 5.0)
        self.declare_parameter(
            "initial_position", [0.555628, -0.028394, 0.500983]
        )
        self.declare_parameter(
            "initial_orientation_xyzw", [-0.019703, 0.987570, 0.119061, 0.100709]
        )

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate = float(self.get_parameter("rate").value)
        self.translation_step = float(self.get_parameter("translation_step").value)
        self.rotation_step = math.radians(
            float(self.get_parameter("rotation_step_deg").value)
        )
        self.position = _vector(
            self.get_parameter("initial_position").value, 3, "initial_position"
        )
        self.orientation = _normalize(
            _vector(
                self.get_parameter("initial_orientation_xyzw").value,
                4,
                "initial_orientation_xyzw",
            )
        )
        if not self.topic or not self.frame_id:
            raise ValueError("topic과 frame_id는 비어 있을 수 없습니다.")
        if self.rate <= 0.0 or self.translation_step <= 0.0 or self.rotation_step <= 0.0:
            raise ValueError("rate, translation_step, rotation_step은 양수여야 합니다.")

        self.publisher = self.create_publisher(PoseStamped, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self._terminal_settings = None
        if not sys.stdin.isatty():
            raise RuntimeError("키보드 teleop은 터미널에서 실행해야 합니다.")
        self._terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._publish()
        self.get_logger().info(
            "Keyboard teleop: w/s=x, a/d=y, r/f=z | "
            "u/o=roll, i/k=pitch, j/l=yaw | space=reset, q=quit"
        )
        self._print_pose()

    def close_terminal(self) -> None:
        if self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None
            print()

    def _tick(self) -> None:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if key == "q":
                rclpy.shutdown()
                return
            if key == " ":
                self.position = _vector(
                    self.get_parameter("initial_position").value, 3, "initial_position"
                )
                self.orientation = _normalize(
                    _vector(
                        self.get_parameter("initial_orientation_xyzw").value,
                        4,
                        "initial_orientation_xyzw",
                    )
                )
            elif key in {"w", "s", "a", "d", "r", "f"}:
                axis = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1), "r": (2, 1), "f": (2, -1)}[key]
                self.position[axis[0]] += axis[1] * self.translation_step
            elif key in {"u", "o", "i", "k", "j", "l"}:
                axis = {"u": (0, 1), "o": (0, -1), "i": (1, 1), "k": (1, -1), "j": (2, 1), "l": (2, -1)}[key]
                delta = _axis_quaternion(axis[0], axis[1] * self.rotation_step)
                self.orientation = _normalize(_multiply_quaternions(delta, self.orientation))
            else:
                continue
            self._publish()
            self._print_pose()

    def _publish(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x, message.pose.position.y, message.pose.position.z = self.position
        message.pose.orientation.x, message.pose.orientation.y = self.orientation[:2]
        message.pose.orientation.z, message.pose.orientation.w = self.orientation[2:]
        self.publisher.publish(message)

    def _print_pose(self) -> None:
        x, y, z = self.position
        print(f"\rTCP target: x={x:+.3f} y={y:+.3f} z={z:+.3f}  ", end="", flush=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_terminal()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
