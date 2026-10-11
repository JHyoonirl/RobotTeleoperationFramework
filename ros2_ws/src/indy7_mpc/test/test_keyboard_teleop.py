"""Smoke check for keyboard teleop quaternion updates."""

import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

from indy7_mpc.keyboard_teleop import (
    KeyboardTeleop,
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


def test_measured_pose_only_initializes_target_once():
    node = KeyboardTeleop.__new__(KeyboardTeleop)
    node.pose_initialized = False
    node.position = [0.0, 0.0, 0.0]
    node.orientation = [0.0, 0.0, 0.0, 1.0]
    node.home_position = [0.0, 0.0, 0.0]
    node.home_orientation = [0.0, 0.0, 0.0, 1.0]
    node._publish = Mock()

    def message(x):
        return SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.2, z=0.3),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

    with patch.object(KeyboardTeleop, "get_logger", return_value=Mock()):
        node._on_current_pose(message(1.0))
        node.position[0] = 1.5
        node._on_current_pose(message(2.0))

    assert node.position == [1.5, 0.2, 0.3]
    assert node._publish.call_count == 1


if __name__ == "__main__":
    test_axis_rotation_stays_normalized()
    test_measured_pose_only_initializes_target_once()
    print("keyboard teleop smoke check passed")
