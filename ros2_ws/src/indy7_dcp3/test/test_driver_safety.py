import numpy as np
import pytest

from indy7_dcp3.safety import limit_joint_target


LOWER = np.full(6, -1.0)
UPPER = np.full(6, 1.0)
VELOCITY = np.full(6, 0.1)


def test_target_is_velocity_limited_per_cycle():
    result = limit_joint_target(
        np.zeros(6), np.full(6, 0.5), LOWER, UPPER, VELOCITY, 0.05, 0.05
    )
    np.testing.assert_allclose(result, np.full(6, 0.005))


def test_target_outside_soft_limit_is_rejected():
    target = np.zeros(6)
    target[2] = UPPER[2]
    with pytest.raises(ValueError, match="soft joint limits"):
        limit_joint_target(np.zeros(6), target, LOWER, UPPER, VELOCITY, 0.05, 0.05)


def test_invalid_command_shape_and_nan_are_rejected():
    with pytest.raises(ValueError, match="six finite"):
        limit_joint_target(np.zeros(6), np.zeros(5), LOWER, UPPER, VELOCITY, 0.05, 0.05)
    target = np.zeros(6)
    target[0] = np.nan
    with pytest.raises(ValueError, match="six finite"):
        limit_joint_target(np.zeros(6), target, LOWER, UPPER, VELOCITY, 0.05, 0.05)
