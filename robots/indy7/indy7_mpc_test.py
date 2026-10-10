#!/usr/bin/env python3
"""Safe, simulation-only joint-space MPC smoke test for Neuromeka Indy7.

The controller uses a double-integrator model: state=[joint position, velocity]
and input=joint acceleration.  It deliberately has no robot-network code.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np


# Neuromeka's Indy7 is a 6-DOF arm; "7" is the product family name.
DOF = 6


@dataclass(frozen=True)
class Config:
    dt: float = 0.01
    horizon: int = 20
    max_velocity: float = 0.8  # rad/s; tune against the real robot limits.
    max_acceleration: float = 1.0  # rad/s^2; start conservatively on hardware.
    position_weight: float = 30.0
    velocity_weight: float = 1.0
    input_weight: float = 0.08


class JointSpaceMPC:
    """Unconstrained finite-horizon LQ tracker with conservative input clipping."""

    def __init__(self, config: Config) -> None:
        self.config = config
        identity = np.eye(DOF)
        zero = np.zeros((DOF, DOF))
        self.a = np.block([[identity, config.dt * identity], [zero, identity]])
        self.b = np.vstack((0.5 * config.dt**2 * identity, config.dt * identity))
        self.f, self.g = self._prediction_matrices()
        q = np.diag([config.position_weight] * DOF + [config.velocity_weight] * DOF)
        q_bar = np.kron(np.eye(config.horizon), q)
        r_bar = config.input_weight * np.eye(DOF * config.horizon)
        self.hessian = self.g.T @ q_bar @ self.g + r_bar
        self.feedback = np.linalg.solve(self.hessian, self.g.T @ q_bar)

    def _prediction_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        n = 2 * DOF
        f = np.zeros((self.config.horizon * n, n))
        g = np.zeros((self.config.horizon * n, self.config.horizon * DOF))
        a_power = np.eye(n)
        for row in range(self.config.horizon):
            a_power = a_power @ self.a
            f[row * n : (row + 1) * n] = a_power
            for col in range(row + 1):
                g[row * n : (row + 1) * n, col * DOF : (col + 1) * DOF] = (
                    np.linalg.matrix_power(self.a, row - col) @ self.b
                )
        return f, g

    def command(self, position: np.ndarray, velocity: np.ndarray, goal: np.ndarray) -> np.ndarray:
        state = np.r_[position, velocity]
        reference = np.tile(np.r_[goal, np.zeros(DOF)], self.config.horizon)
        sequence = self.feedback @ (reference - self.f @ state)
        return np.clip(
            sequence[:DOF], -self.config.max_acceleration, self.config.max_acceleration
        )


def parse_target(value: str) -> np.ndarray:
    target = np.fromstring(value, sep=",")
    if target.shape != (DOF,) or not np.all(np.isfinite(target)):
        raise argparse.ArgumentTypeError("--target은 쉼표로 구분한 유한한 6개 rad 값이어야 합니다.")
    return target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Neuromeka Indy7 6축 관절공간 MPC 드라이런")
    parser.add_argument("--duration", type=float, default=4.0, help="시뮬레이션 시간(초)")
    parser.add_argument("--target", type=parse_target, default=parse_target("0.2,-0.35,0.3,0.15,-0.2,0.25"))
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration은 0보다 커야 합니다.")
    return args


def simulate(controller: JointSpaceMPC, goal: np.ndarray, duration: float) -> tuple[np.ndarray, np.ndarray]:
    """Run the same state/command loop a hardware adapter will use, without I/O."""
    position = np.zeros(DOF)
    velocity = np.zeros(DOF)
    steps = round(duration / controller.config.dt)
    for _ in range(steps):
        acceleration = controller.command(position, velocity, goal)
        velocity = np.clip(
            velocity + controller.config.dt * acceleration,
            -controller.config.max_velocity,
            controller.config.max_velocity,
        )
        position += controller.config.dt * velocity
    return position, velocity


def main() -> None:
    args = parse_args()
    controller = JointSpaceMPC(Config())
    final_position, final_velocity = simulate(controller, args.target, args.duration)
    initial_error = float(np.linalg.norm(args.target))
    final_error = float(np.linalg.norm(args.target - final_position))
    print("simulation only — no Indy7 command was sent")
    print("target [rad]:", np.array2string(args.target, precision=3))
    print("final  [rad]:", np.array2string(final_position, precision=3))
    print(f"tracking error: {initial_error:.4f} -> {final_error:.4f} rad")
    assert final_error < initial_error, "MPC smoke test did not reduce tracking error"
    assert np.all(np.isfinite(final_velocity)), "simulation produced non-finite velocity"


if __name__ == "__main__":
    main()
