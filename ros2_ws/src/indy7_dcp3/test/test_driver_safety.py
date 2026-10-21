import numpy as np
import pytest
from types import SimpleNamespace

from indy7_dcp3.driver_node import Indy7Dcp3Node
from indy7_dcp3.safety import percent_ratio, validate_joint_target


LOWER = np.full(6, -1.0)
UPPER = np.full(6, 1.0)


def test_valid_joint_targets_are_forwarded_without_velocity_clipping():
    requested = np.array([0.5, -0.5, 0.1, -0.1, 0.95, -0.95])
    result = validate_joint_target(np.zeros(6), requested, LOWER, UPPER, 0.05)
    np.testing.assert_array_equal(result, requested)


def test_target_outside_soft_limit_is_rejected():
    target = np.zeros(6)
    target[2] = UPPER[2]
    with pytest.raises(ValueError, match="soft joint limits"):
        validate_joint_target(np.zeros(6), target, LOWER, UPPER, 0.05)


def test_invalid_command_shape_and_nan_are_rejected():
    with pytest.raises(ValueError, match="six finite"):
        validate_joint_target(np.zeros(6), np.zeros(5), LOWER, UPPER, 0.05)
    target = np.zeros(6)
    target[0] = np.nan
    with pytest.raises(ValueError, match="six finite"):
        validate_joint_target(np.zeros(6), target, LOWER, UPPER, 0.05)


def test_measured_position_outside_hard_limits_is_rejected():
    with pytest.raises(ValueError, match="measured joint position"):
        validate_joint_target(np.full(6, 1.1), np.zeros(6), LOWER, UPPER, 0.05)


@pytest.mark.parametrize("margin", [-0.1, float("nan"), float("inf"), 1.0])
def test_invalid_joint_limit_margin_is_rejected(margin):
    with pytest.raises(ValueError, match="margin"):
        validate_joint_target(np.zeros(6), np.zeros(6), LOWER, UPPER, margin)


@pytest.mark.parametrize("value", [0.0, 20.0, 100.0])
@pytest.mark.parametrize("name", ["velocity_ratio", "acceleration_ratio"])
def test_dcp3_motion_ratio_accepts_percent_range(value, name):
    assert percent_ratio(value, name) == value


@pytest.mark.parametrize("value", [-0.01, 100.01, float("nan"), float("inf"), "invalid"])
def test_dcp3_motion_ratio_rejects_values_outside_percent_range(value):
    with pytest.raises(ValueError, match="velocity_ratio.*0 to 100"):
        percent_ratio(value, "velocity_ratio")


def test_vendor_rpc_calls_receive_bounded_deadlines():
    class Stub:
        def __init__(self, methods):
            self.calls = {}
            for name in methods:
                setattr(self, name, self._record(name))

        def _record(self, name):
            def call(*args, **kwargs):
                self.calls[name] = (args, kwargs)
                return "ok"
            return call

    rtde = Stub(("GetControlData",))
    control = Stub(("StartTeleOp", "MoveTeleJ", "StopMotion", "StopTeleOp"))
    client = SimpleNamespace(rtde=rtde, control=control)
    Indy7Dcp3Node._install_rpc_deadlines(client, 0.1)

    assert rtde.GetControlData("request") == "ok"
    assert control.StartTeleOp("request", timeout=0.2) == "ok"
    assert control.MoveTeleJ("request") == "ok"
    assert control.StopMotion("request") == "ok"
    assert control.StopTeleOp("request") == "ok"
    assert rtde.calls["GetControlData"][1]["timeout"] == 0.1
    assert control.calls["StartTeleOp"][1]["timeout"] == 0.2
    assert all(control.calls[name][1]["timeout"] == 0.1
               for name in ("MoveTeleJ", "StopMotion", "StopTeleOp"))


def test_deadline_wrapper_can_be_installed_on_neuromeka_grpc_stubs():
    neuromeka = pytest.importorskip("neuromeka")
    client = neuromeka.IndyDCP3("127.0.0.1")
    Indy7Dcp3Node._install_rpc_deadlines(client, 0.1)

    assert callable(client.rtde.GetControlData)
    assert callable(client.control.StartTeleOp)
    assert callable(client.control.MoveTeleJ)
    assert callable(client.control.StopMotion)
    assert callable(client.control.StopTeleOp)
    assert hasattr(client.control.MoveTeleJ, "__wrapped__")
