"""Smoke check for the URDF/ABA-backed SE(3) Crocoddyl action model."""

from pathlib import Path

import crocoddyl
import numpy as np
import pinocchio as pin

from robot_mpc.se3_dynamic_model import SE3DynamicActionModel


def make_action():
    urdf = Path(__file__).parents[2] / "robot_description/urdf/indy7.urdf"
    model = pin.buildModelFromUrdf(str(urdf))
    state = crocoddyl.StateMultibody(model)
    q = np.array([0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
    data = model.createData()
    frame_id = model.getFrameId("tcp")
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    weights = {
        "se3_position": 20.0,
        "se3_orientation": 5.0,
        "posture": 0.1,
        "velocity": 1.0,
        "torque": 0.0001,
        "joint_limit": 10.0,
    }
    action = SE3DynamicActionModel(
        crocoddyl=crocoddyl,
        pin=pin,
        model=model,
        state=state,
        frame_id=frame_id,
        target=data.oMf[frame_id],
        nominal_posture=q,
        weights=weights,
        torque_limits=model.effortLimit,
    )
    return action, np.r_[q, np.zeros(model.nv)]


def test_gravity_compensation_at_target_has_zero_cost():
    action, x = make_action()
    u = pin.computeGeneralizedGravity(action.model, action.model.createData(), x[:6])
    action_data = action.createData()
    action.calc(action_data, x, u)
    action.calcDiff(action_data, x, u)
    assert action_data.cost == 0.0
    assert np.all(np.isfinite(action_data.Lx))
    assert np.all(np.isfinite(action_data.Lxx))
    np.testing.assert_allclose(action_data.Lx, 0.0, atol=1e-12)
    np.testing.assert_allclose(action_data.Lu, 0.0, atol=1e-12)


def test_torque_residual_derivatives_match_finite_differences():
    action, x = make_action()
    # Isolate the gravity-dependent residual, including its state/control coupling.
    action.weights.update({name: 0.0 for name in action.weights if name != "torque"})
    action.weights["torque"] = 0.3
    u = pin.computeGeneralizedGravity(action.model, action.model.createData(), x[:6]) + 2.0
    data = action.createData()
    action.calc(data, x, u)
    action.calcDiff(data, x, u)
    step = 1e-6
    gradient = np.zeros(12)
    cross = np.zeros((12, 6))
    for index in range(12):
        delta = np.eye(12)[index] * step
        plus = action.cost_terms(x + delta, u)["torque"]
        minus = action.cost_terms(x - delta, u)["torque"]
        gradient[index] = (plus - minus) / (2 * step)
        for sign in (-1, 1):
            shifted = action.createData()
            action.calc(shifted, x + sign * delta, u)
            action.calcDiff(shifted, x + sign * delta, u)
            cross[index] += sign * shifted.Lu / (2 * step)
    np.testing.assert_allclose(data.Lx, gradient, atol=1e-7)
    np.testing.assert_allclose(data.Lxu, cross, atol=1e-7)
    np.testing.assert_allclose(data.Lu, np.full(6, 0.6), atol=1e-10)


if __name__ == "__main__":
    test_gravity_compensation_at_target_has_zero_cost()
    test_torque_residual_derivatives_match_finite_differences()
    print("SE(3) action model smoke check passed")
