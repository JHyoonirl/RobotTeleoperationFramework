"""Smoke check for keyboard teleop quaternion updates."""

import math

from indy7_mpc.keyboard_teleop import (
    _axis_quaternion,
    _multiply_quaternions,
    _normalize,
)


def test_axis_rotation_stays_normalized():
    quaternion = _normalize(
        _multiply_quaternions(_axis_quaternion(2, math.pi / 2), [0.0, 0.0, 0.0, 1.0])
    )
    assert math.isclose(sum(value * value for value in quaternion), 1.0)
    assert quaternion[2] > 0.0 and quaternion[3] > 0.0


if __name__ == "__main__":
    test_axis_rotation_stays_normalized()
    print("keyboard teleop smoke check passed")
