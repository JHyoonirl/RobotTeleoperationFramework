"""Two-hand tracking and torso posture costs on the shared EIR model."""

from pathlib import Path

import crocoddyl
import numpy as np
import pinocchio as pin
import pytest

from robot_mpc.robot_model import load_joint_model
from robot_mpc.se3_dynamic_model import SE3DynamicActionModel


def make_action(**overrides):
    urdf = Path(__file__).parents[2] / "robot_description/urdf/eir.urdf"
    names = ["Joint_L0", "Joint_L1"]
    names += [f"Joint_L{i}_{arm}" for arm in ("L", "R") for i in range(2, 9)]
    model, q = load_joint_model(pin, urdf, names)
    q += np.array([0.10, -0.10] + [0.02, -0.03, 0.04, 0.02, -0.03, 0.04, 0.02] * 2)
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    right = model.getFrameId("right_arm_link_7")
    left = model.getFrameId("left_arm_link_7")
    target = data.oMf[right].copy()
    target.translation += [0.01, -0.02, 0.015]
    target.rotation = target.rotation @ pin.exp3(np.array([0.02, -0.03, 0.01]))
    secondary = data.oMf[left].copy()
    secondary.translation += [-0.02, 0.01, 0.025]
    secondary.rotation = secondary.rotation @ pin.exp3(np.array([-0.03, 0.01, 0.02]))
    options = dict(
        crocoddyl=crocoddyl, pin=pin, model=model,
        state=crocoddyl.StateMultibody(model), frame_id=right, target=target,
        nominal_posture=q - 0.02,
        weights={"se3_position": 20.0, "se3_orientation": 5.0,
                 "posture": 0.1, "velocity": 1.0, "torque": 0.0001,
                 "joint_limit": 10.0},
        path_tolerance=0.005,
        minimum_z=max(data.oMf[right].translation[2], data.oMf[left].translation[2]) + 0.04,
        terminal_scale=1.7,
    )
    options.update(overrides)
    action = SE3DynamicActionModel(**options)
    action.update_path(target.translation - [0.10, 0.0, 0.0],
                       target.translation + [0.10, 0.0, 0.0])
    action.update_secondary_task(left, secondary,
                                 secondary.translation - [0.10, 0.0, 0.0],
                                 secondary.translation + [0.10, 0.0, 0.0])
    x = np.r_[q, np.linspace(-0.01, 0.01, model.nv)]
    u = pin.computeGeneralizedGravity(model, model.createData(), q) + 0.1
    return action, x, u


def cost_gradient(action, x, u):
    data = action.createData()
    action.calc(data, x, u)
    action.calcDiff(data, x, u)
    return data


def numerical_gradient(action, x, u):
    gradient = np.zeros(x.size)
    for index in range(x.size):
        delta = np.eye(x.size)[index] * 1e-6
        plus = sum(action.cost_terms(x + delta, u).values())
        minus = sum(action.cost_terms(x - delta, u).values())
        gradient[index] = (plus - minus) / 2e-6
    return gradient


def test_joint_limit_cost_is_normalized_by_the_safe_margin():
    action, _, _ = make_action(path_tolerance=None, minimum_z=None)
    action.weights.update({name: 0.0 for name in action.weights})
    action.weights["joint_limit"] = 1.0
    q = 0.5 * (action.model.lowerPositionLimit + action.model.upperPositionLimit)
    joint = 0
    margin = action.joint_limit_margin
    q[joint] = action.model.lowerPositionLimit[joint]

    cost, gradient, hessian = action._joint_limit_terms(q)

    assert cost == pytest.approx(0.5)
    assert gradient[joint] == pytest.approx(-1.0 / margin)
    assert hessian[joint] == pytest.approx(1.0 / margin**2)
    q[joint] = action.soft_lower[joint]
    assert action._joint_limit_terms(q)[0] == 0.0


def test_position_servo_prediction_uses_acceleration_units_and_consistent_derivatives():
    action, x, _ = make_action(prediction_model="joint_acceleration",
                               acceleration_limits=np.full(16, 40.0))
    u = np.linspace(-2., 2., action.nv)
    data = cost_gradient(action, x, u)
    np.testing.assert_allclose(data.xout, u)
    np.testing.assert_allclose(data.Fx, 0.)
    np.testing.assert_allclose(data.Fu, np.eye(action.nv))
    np.testing.assert_allclose(action.u_lb, -40.)
    np.testing.assert_allclose(action.u_ub, 40.)
    assert "acceleration" in action.cost_terms(x, u)
    assert "torque" not in action.cost_terms(x, u)
    np.testing.assert_allclose(data.Lx, numerical_gradient(action, x, u), atol=2e-5)
    for index in range(action.nv):
        delta = np.eye(action.nv)[index] * 1e-6
        numerical = (sum(action.cost_terms(x, u + delta).values())
                     - sum(action.cost_terms(x, u - delta).values())) / 2e-6
        assert data.Lu[index] == pytest.approx(numerical, abs=1e-6)


def test_two_tasks_add_only_tracking_path_and_floor_costs():
    action, x, u = make_action()
    dual = action.cost_terms(x, u)
    dual_dynamics = cost_gradient(action, x, u).Fx.copy()
    secondary = (action.secondary_frame_id, action.secondary_target.copy(),
                 action.secondary_path_start.copy(), action.secondary_path_goal.copy())
    action.update_secondary_task(None)
    primary = action.cost_terms(x, u)
    np.testing.assert_array_equal(cost_gradient(action, x, u).Fx, dual_dynamics)
    action.frame_id, action.target, action.path_start, action.path_goal = secondary
    second = action.cost_terms(x, u)
    for name in ("se3_tracking", "cartesian_path", "workspace_floor"):
        assert dual[name] == pytest.approx(primary[name] + second[name])
    for name in ("posture", "velocity", "torque", "joint_limit"):
        assert dual[name] == pytest.approx(primary[name])


def test_explicit_soft_path_weight_is_independent_of_hard_tube_tolerance():
    first, x, u = make_action(path_tolerance=.02, path_cost_weight=250000.)
    second, _, _ = make_action(path_tolerance=.005, path_cost_weight=250000.)
    assert first.path_weight == second.path_weight == 250000.
    assert first.cost_terms(x, u)['cartesian_path'] == pytest.approx(second.cost_terms(x, u)['cartesian_path'])
    first.weights.update({name: 0. for name in first.weights})
    first.minimum_z = None
    np.testing.assert_allclose(cost_gradient(first, x, u).Lx, numerical_gradient(first, x, u),
                               rtol=1e-5, atol=1e-4)


@pytest.mark.parametrize("term", ["se3_tracking", "cartesian_path", "workspace_floor"])
@pytest.mark.parametrize("terminal", [False, True])
def test_two_hand_cost_gradients_match_finite_differences(term, terminal):
    action, x, u = make_action(terminal=terminal)
    if term != "se3_tracking":
        action.weights.update(se3_position=0.0, se3_orientation=0.0)
    action.weights.update(posture=0.0, velocity=0.0, torque=0.0, joint_limit=0.0)
    if term != "cartesian_path":
        action.path_weight = 0.0
    if term != "workspace_floor":
        action.minimum_z = None
    data = cost_gradient(action, x, u)
    np.testing.assert_allclose(data.Lx, numerical_gradient(action, x, u), rtol=1e-5, atol=1e-4)
    assert np.min(np.linalg.eigvalsh(data.Lxx)) >= 0.0
    # Both arms and the common torso must affect the shared objective.
    assert np.linalg.norm(data.Lx[2:9]) > 0.0
    assert np.linalg.norm(data.Lx[9:16]) > 0.0
    assert np.linalg.norm(data.Lx[:2]) > 0.0


def test_diagonal_posture_weights_have_exact_cost_gradient_and_hessian():
    action, x, u = make_action(path_tolerance=None, minimum_z=None)
    action.weights.update({name: 0.0 for name in action.weights})
    weights = np.array([40.0, 40.0] + [0.05] * 14)
    action.update_posture_weights(weights)
    weights[:] = 0.0  # Caller mutation must not change the configured weights.
    data = cost_gradient(action, x, u)
    error = x[:16] - action.nominal_posture
    expected = action.terminal_scale * action.posture_weights
    assert data.cost == pytest.approx(0.5 * (expected * error) @ error)
    np.testing.assert_allclose(data.Lx[:16], expected * error)
    np.testing.assert_allclose(data.Lx, numerical_gradient(action, x, u), atol=1e-8)
    np.testing.assert_allclose(data.Lxx[:16, :16], np.diag(expected + 1e-6))
    action.weights["posture"] = 0.2
    action.update_posture_weights(None)
    assert action.cost_terms(x, u)["posture"] == pytest.approx(
        action.terminal_scale * 0.5 * 0.2 * (error @ error))


def test_secondary_task_copies_inputs_and_can_be_cleared():
    action, x, u = make_action()
    target = action.secondary_target.copy()
    start, goal = target.translation.copy(), target.translation.copy()
    action.update_secondary_task(action.secondary_frame_id, target, start, goal)
    expected = action.cost_terms(x, u)
    target.translation[:] = 10.0
    start[:] = 20.0
    goal[:] = 30.0
    assert action.cost_terms(x, u) == expected
    action.update_secondary_task(None)
    assert action.secondary_target is None
    assert action.secondary_path_start is None
    assert action.secondary_path_goal is None


@pytest.mark.parametrize("invalid", [-1, 10000, 1.5])
def test_invalid_secondary_frame_is_rejected(invalid):
    action, _, _ = make_action()
    with pytest.raises(ValueError, match="frame_id"):
        action.update_secondary_task(invalid, action.target)


@pytest.mark.parametrize("invalid", [[1.0], [0.0] * 15 + [-1.0], [0.0] * 15 + [np.nan]])
def test_invalid_posture_diagonal_is_rejected(invalid):
    action, _, _ = make_action()
    with pytest.raises(ValueError, match="posture weights"):
        action.update_posture_weights(invalid)


@pytest.mark.parametrize("prediction", ["joint_acceleration", "rigid_body"])
def test_derivatives_recompute_kinematics_after_another_cost_evaluation(prediction):
    action, x, u = make_action(prediction_model=prediction, acceleration_limits=np.full(16, 40.0))
    expected = cost_gradient(action, x, u)
    gradient, hessian = expected.Lx.copy(), expected.Lxx.copy()
    other = x.copy()
    other[:16] += .1
    action.cost_terms(other, u)
    actual = action.createData()
    action.calcDiff(actual, x, u)
    np.testing.assert_allclose(actual.Lx, gradient)
    np.testing.assert_allclose(actual.Lxx, hessian)


@pytest.mark.parametrize("angle", [0.0, 1e-9, .3, np.pi - 1e-7, np.pi])
def test_pose_rotation_error_is_finite_and_uses_the_shortest_rotation(angle):
    action, x, u = make_action()
    pin.framesForwardKinematics(action.model, action.pin_data, x[:16])
    target = action.pin_data.oMf[action.frame_id].copy()
    axis = np.array([1., 2., -3.]) / np.sqrt(14.)
    target.rotation = target.rotation @ pin.exp3(angle * axis)
    error = action._pose_error(action.frame_id, target)
    assert np.all(np.isfinite(error))
    assert np.linalg.norm(error[3:]) == pytest.approx(angle, abs=1e-7)
    np.testing.assert_allclose(pin.exp3(error[3:]), pin.exp3(angle * axis), atol=1e-7)
