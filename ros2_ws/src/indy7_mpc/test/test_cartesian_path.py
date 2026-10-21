"""Geometry, continuous-step bounds and the actual post-clipping command guard."""

from pathlib import Path
import time
from unittest.mock import Mock

import crocoddyl
import numpy as np
import pinocchio as pin
import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from indy7_mpc.cartesian_path import limit_cartesian_step, segment_error
from indy7_mpc.crocoddyl_mpc_node import CrocoddylMPCNode
from indy7_mpc.mpc_plotter import MpcPlotter
from indy7_mpc.se3_dynamic_model import SE3DynamicActionModel


@pytest.mark.parametrize("point, expected", [
    ([0.2, 0.0025, 0.0], 0.0025),
    ([0.8, 0.0, 0.0], 0.0),  # Along-line tracking lag is not cross-track error.
    ([-0.003, 0.004, 0.0], 0.005),
    ([1.003, 0.004, 0.0], 0.005),
])
def test_distance_to_closed_segment(point, expected):
    residual, _ = segment_error(point, np.zeros(3), np.array([1.0, 0.0, 0.0]))
    assert np.linalg.norm(residual) == pytest.approx(expected)


def test_rotation_only_path_is_a_point():
    residual, jacobian = segment_error([0.001, 0.002, 0], np.zeros(3), np.zeros(3))
    assert np.linalg.norm(residual) == pytest.approx(np.sqrt(5) * 0.001)
    np.testing.assert_array_equal(jacobian, np.eye(3))


def test_step_is_reduced_and_measured_violation_holds():
    position = lambda q: np.r_[q, 0.0]
    start, goal = np.zeros(3), np.array([1.0, 0, 0])
    command, scale = limit_cartesian_step(
        np.zeros(2), np.array([0.01, 0.01]), position, start, goal, 0.0025, 1.0
    )
    assert 0 < scale < 1
    assert command[1] <= 0.0025
    measured = np.array([0.0, 0.003])
    command, scale = limit_cartesian_step(
        measured, np.array([0.01, 0.004]), position, start, goal, 0.0025, 1.0
    )
    np.testing.assert_array_equal(command, measured)
    assert scale == 0.0


def test_safe_endpoints_do_not_allow_a_curved_step_outside_the_tube():
    # One revolute link: endpoints are on the chord but its arc bows outward.
    position = lambda q: np.array([np.cos(q[0]), np.sin(q[0]), 0.0])
    measured, proposed = np.array([-0.2]), np.array([0.2])
    start, goal = position(measured), position(proposed)
    command, scale = limit_cartesian_step(
        measured, proposed, position, start, goal, 0.0025, 1.0
    )
    assert scale < 1.0
    deviations = [np.linalg.norm(segment_error(
        position(measured + alpha * (command - measured)), start, goal
    )[0]) for alpha in np.linspace(0, 1, 501)]
    assert max(deviations) <= 0.0025


def test_non_finite_kinematics_fails_closed():
    command, scale = limit_cartesian_step(
        np.zeros(1), np.ones(1), lambda q: np.full(3, np.nan),
        np.zeros(3), np.ones(3), 0.0025, 1.0,
    )
    np.testing.assert_array_equal(command, np.zeros(1))
    assert scale == 0.0


@pytest.fixture
def node():
    urdf = Path(__file__).parents[2] / "indy7_description/urdf/indy7.urdf"
    model = pin.buildModelFromUrdf(str(urdf))
    node = object.__new__(CrocoddylMPCNode)
    node.pin, node.crocoddyl = pin, crocoddyl
    node.model, node.state = model, crocoddyl.StateMultibody(model)
    node.frame_id, node.pose_data = model.getFrameId("tcp"), model.createData()
    node.control_mode = "task_space"
    node.execution_enabled = True
    node.vive_control = None
    node.solve_count = 0
    node.joint_state_timeout = 0.5
    node.last_state_update_monotonic = time.monotonic()
    node.diagnostic_period = 0.1
    node._last_diagnostic_time = -float("inf")
    node.diagnostic_ik_enabled = False
    node.workspace_floor_enabled = True
    node.workspace_min_z = 0.10
    node.workspace_floor_margin = 0.001
    node.workspace_floor_guard = ""
    node.dt = 0.02
    node.command_velocity_limits = np.ones(6)
    node.command_increment_limits = np.full(6, 0.02)
    node.cartesian_path_tolerance = 0.0025
    node.cartesian_reach_bound = sum(np.linalg.norm(p.translation) for p in model.jointPlacements)
    node.cartesian_reach_bound += np.linalg.norm(model.frames[node.frame_id].placement.translation)
    node.sim_q = np.array([0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
    node.cartesian_path_start_pose = node._tcp_pose(node.sim_q).copy()
    node.target_pose = node.cartesian_path_start_pose.copy()
    node.target_pose.translation += np.array([0.05, 0.0, 0.0])
    return node


def test_guard_runs_after_joint_clipping_and_preserves_joint_limits(node):
    q = node.sim_q.copy()
    state = np.r_[q, np.zeros(6)]
    node.command_increment_limits[0] = 0.001
    command = node._safe_joint_command(q + 0.5, state)
    step_limit = np.minimum(node.command_increment_limits, node.command_velocity_limits * node.dt)
    assert np.all(np.abs(command - q) <= step_limit + 1e-12)
    assert np.all(command >= node.model.lowerPositionLimit)
    assert np.all(command <= node.model.upperPositionLimit)
    assert node.cartesian_path_guard == "cartesian_path_limited"
    for alpha in np.linspace(0, 1, 201):
        error, _ = segment_error(node._tcp_pose(q + alpha * (command - q)).translation,
                                 node.cartesian_path_start_pose.translation,
                                 node.target_pose.translation)
        assert np.linalg.norm(error) <= node.cartesian_path_tolerance


def test_joint_mode_keeps_existing_command_limits(node):
    node.control_mode = "joint"
    state = np.r_[node.sim_q, np.zeros(6)]
    command = node._safe_joint_command(node.sim_q + 0.5, state)
    np.testing.assert_allclose(command, node.sim_q + 0.02)
    assert node.cartesian_command_scale == 1.0


@pytest.mark.parametrize("simulate, expected_velocity", [(True, 0.5), (False, 200.0)])
@pytest.mark.parametrize("dt", [0.005, 0.01])
@pytest.mark.parametrize("paused", [False, True])
def test_mock_velocity_uses_consumed_sample_interval_and_real_feedback_is_preserved(
    node, simulate, expected_velocity, dt, paused
):
    node.simulate = simulate
    node.execution_enabled = not paused
    node.dt = dt
    node.internal_simulation = False
    node.joint_names = [f"joint{i}" for i in range(6)]
    node.initial_target_state_initialized = True
    node.latest_state = np.r_[node.sim_q, np.zeros(6)]
    node.last_state_stamp_s = 10.0
    message = JointState()
    message.name = node.joint_names
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = round(2.0 * dt * 1e9)
    message.position = node.sim_q.tolist()
    message.position[0] += 2.0 * dt
    # A mock position jump can produce a huge derivative on a short catch-up tick.
    message.velocity = [200.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node._on_joint_state(message)
    assert node.latest_state[6] == pytest.approx(0.0 if simulate and paused else expected_velocity)
    assert node.last_state_stamp_s == pytest.approx(10.0 + 2.0 * dt)


@pytest.mark.parametrize("simulate", [True, False])
def test_mock_holds_at_target_without_requiring_a_zero_step_solver_result(node, simulate):
    node.simulate = simulate
    node.internal_simulation = False
    node.latest_state = np.r_[node.sim_q, np.zeros(6)]
    node.target_pose = node._tcp_pose(node.sim_q).copy()
    node.cartesian_path_initialized_from_state = True
    node.stationary_hold_active = False
    node.stationary_hold_position_tolerance = 0.002
    node.stationary_hold_orientation_tolerance = 0.01
    node.target_update_count = 1
    node._update_cartesian_references = Mock()
    node._publish_stationary_hold = Mock()
    node.solver = Mock(cost=0.1, iter=0)
    node.solver.solve.side_effect = RuntimeError("stop after probing solver invocation")
    node.problem = Mock()
    node.xs = [node.latest_state.copy() for _ in range(2)]
    node.us = [np.zeros(6)]
    node.max_solver_iterations = 10
    node.last_target_acceleration = np.zeros(6)
    node.last_applied_acceleration = np.zeros(6)
    node.solve_success_pub = Mock()
    node.status_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    node._write_diagnostic = Mock()
    node._solve_once()
    if not simulate:
        node.solver.solve.assert_called_once()
        node._publish_stationary_hold.assert_not_called()
        return
    node.solver.solve.assert_not_called()
    held_q = node.stationary_hold_q.copy()
    assert node.stationary_hold_active
    # A command already in flight must not restart the torque solver.
    node.latest_state[0] += 0.002
    node._solve_once()
    np.testing.assert_array_equal(node.stationary_hold_q, held_q)
    assert node._publish_stationary_hold.call_count == 2
    node.solver.solve.assert_not_called()
    # A new target resumes optimization.
    node.target_pose.translation += np.array([0.02, 0.0, 0.0])
    node.target_update_count += 1
    node.stationary_hold_active = False
    node._solve_once()
    node.solver.solve.assert_called_once()


def test_measured_path_violation_holds_and_reports_failure(node):
    node.get_logger = Mock(return_value=Mock())
    measured = node.sim_q.copy()
    measured[0] += 0.02
    command = node._safe_joint_command(node.sim_q, np.r_[measured, np.zeros(6)])
    np.testing.assert_array_equal(command, measured)
    assert node.cartesian_path_guard == "cartesian_path_violation"
    node.get_logger().error.assert_called_once()


def test_recorded_boundary_stall_can_move_inward(node):
    # First zero-step record in 20261018T185048.382150Z/trace.jsonl.
    measured = np.array([
        3.96848273071239e-05, -0.0071678717585027234, -1.5535224897034616,
        0.019773012355845915, -1.5502269876441543, -0.00918978362056589,
    ])
    predicted = np.array([
        -0.0020882194304398154, -0.014417203675715733, -1.4988100285079364,
        0.2017491635579424, -1.3893796275965893, -0.02091809869916434,
    ])
    node.cartesian_path_start_pose.translation = np.array([
        0.3503630139706111, -0.18650000008925724, 0.5217790034742422,
    ])
    node.target_pose.translation = np.array([0.555628, -0.028394, 0.500983])
    node.dt = 0.005
    node.command_velocity_limits = np.minimum(10.0, node.model.velocityLimit)
    node.command_increment_limits = np.full(6, 0.05)
    command = node._safe_joint_command(predicted, np.r_[measured, np.zeros(6)])
    assert np.linalg.norm(command - measured) > 1e-6
    for alpha in np.linspace(0, 1, 501):
        error, _ = segment_error(node._tcp_pose(measured + alpha * (command - measured)).translation,
                                 node.cartesian_path_start_pose.translation, node.target_pose.translation)
        assert np.linalg.norm(error) <= 0.0025
    assert np.linalg.norm(error) < 0.0024998


@pytest.mark.parametrize("status, success, label", [
    ("workspace_floor_limited", 1.0, "FLOOR LIMITED"),
    ("workspace_floor_hold", 0.0, "FLOOR HOLD"),
    ("workspace_floor_violation", 0.0, "FLOOR VIOLATION | HOLD"),
    ("joint_state_stale", 0.0, "JOINT STATE STALE"),
    ("cartesian_path_limited", 1.0, "PATH LIMITED"),
    ("cartesian_path_hold", 1.0, "PATH HOLD"),
    ("cartesian_path_violation", 1.0, "PATH VIOLATION | HOLD"),
    ("not_converged", 0.0, "SOLVER NOT CONVERGED"),
    ("exception", 0.0, "SOLVER FAILED"),
    ("solved", 1.0, "solver OK"),
])
def test_plot_distinguishes_solver_failure_from_path_filter(status, success, label):
    assert MpcPlotter._status_label(status, success)[0] == label


@pytest.mark.parametrize("terminal", [False, True])
def test_path_cost_gradient_matches_finite_difference(node, terminal):
    q = node.sim_q + np.array([0.01, -0.005, 0, 0, 0, 0])
    weights = {name: 0.0 for name in (
        "se3_position", "se3_orientation", "posture", "velocity", "torque", "joint_limit"
    )}
    action = SE3DynamicActionModel(
        crocoddyl=crocoddyl, pin=pin, model=node.model, state=node.state,
        frame_id=node.frame_id, target=node.target_pose, nominal_posture=q,
        weights=weights, path_tolerance=0.0025, terminal=terminal,
        terminal_scale=10.0 if terminal else 1.0,
    )
    action.update_path(node.cartesian_path_start_pose.translation, node.target_pose.translation)
    x, u = np.r_[q, np.zeros(6)], np.zeros(6)
    data = action.createData()
    action.calc(data, x, u)
    action.calcDiff(data, x, u)
    gradient = data.Lx.copy()
    for index in range(12):
        delta = np.zeros(12)
        delta[index] = 1e-7
        action.calc(data, x + delta, u)
        plus = data.cost
        action.calc(data, x - delta, u)
        assert gradient[index] == pytest.approx((plus - data.cost) / 2e-7, abs=1e-5)
    np.testing.assert_allclose(data.Lxx, data.Lxx.T, atol=1e-10)
    assert np.linalg.eigvalsh(data.Lxx).min() > 0


def test_identical_target_does_not_reanchor_an_expired_path(node):
    node.pose_frame = "world"
    node.cartesian_path_start_time = 0.0
    node.cartesian_path_duration = 0.1
    node.latest_state = np.r_[node.sim_q, np.zeros(6)]
    node.last_target_update_monotonic = 0.0
    node.previous_target_pose = None
    node.target_update_count = 0
    node._solve_ik = Mock(side_effect=lambda target, seed: seed.copy())
    message = PoseStamped()
    message.header.frame_id = "world"
    position = node.target_pose.translation
    orientation = pin.Quaternion(node.target_pose.rotation).coeffs()
    message.pose.position.x, message.pose.position.y, message.pose.position.z = position
    (message.pose.orientation.x, message.pose.orientation.y,
     message.pose.orientation.z, message.pose.orientation.w) = orientation
    original_start = node.cartesian_path_start_pose.copy()
    node._on_target_pose(message)
    node._solve_ik.assert_not_called()
    np.testing.assert_array_equal(node.cartesian_path_start_pose.translation, original_start.translation)
    assert node.cartesian_path_start_time == 0.0


def test_diagnostic_work_is_rate_limited(node, monkeypatch):
    current_time = [5.0]
    node.diagnostic_period = 0.1
    node._last_diagnostic_time = 4.95
    monkeypatch.setattr("indy7_mpc.crocoddyl_mpc_node.time.monotonic", lambda: current_time[0])

    assert not node._diagnostics_due()
    current_time[0] = 5.06
    assert node._diagnostics_due()
    assert not node._diagnostics_due()


@pytest.mark.parametrize("translation, rotation", [
    ([0.02, 0.0, 0.0], [0, 0, 0]),
    ([0.0, 0.02, 0.01], [0, 0, 0]),
    ([0.0, 0.0, 0.0], [0.1, 0.1, 0.0]),
    (None, None),  # The configured 26 cm startup motion that stalled in the log.
])
def test_real_mpc_reaches_target_inside_tube(node, translation, rotation, monkeypatch):
    config = Path(__file__).parents[1] / "config/indy7_mpc.yaml"
    settings = yaml.safe_load(config.read_text())["crocoddyl_mpc"]["ros__parameters"]
    for name, value in settings.items():
        if name != "task_frame":
            setattr(node, name, np.array(value) if isinstance(value, list) else value)
    node.command_velocity_limits = np.minimum(node.command_velocity_limits, node.model.velocityLimit)
    node.target_pose = node.cartesian_path_start_pose.copy()
    if translation is None:
        node.target_pose.translation = np.asarray(settings["goal_position"])
        node.target_pose.rotation = pin.Quaternion(np.asarray(settings["goal_orientation_xyzw"])).matrix()
    else:
        node.target_pose.translation += np.asarray(translation)
        node.target_pose.rotation = node.target_pose.rotation @ pin.exp3(np.asarray(rotation, dtype=float))
    node.xref = np.r_[node.sim_q, np.zeros(6)]
    node.cartesian_reference_poses = [node.target_pose.copy() for _ in range(node.horizon + 1)]
    node.cartesian_path_start_time = 0.0
    node.problem, node.solver = node._make_problem()
    state = node.xref.copy()
    xs = [state.copy() for _ in range(node.horizon + 1)]
    us = [np.zeros(6) for _ in range(node.horizon)]
    for cycle in range(200):
        monkeypatch.setattr("indy7_mpc.crocoddyl_mpc_node.time.monotonic", lambda: cycle * node.dt)
        node._update_cartesian_references(state[:6])
        node.problem.x0 = state
        xs[0] = state.copy()
        node.solver.solve(xs, us, node.max_solver_iterations, False)
        xs, us = node.solver.xs, node.solver.us
        command = node._safe_joint_command(xs[1][:6], state)
        # Ideal joint-position servo, without ROS publishers or hardware.
        state = np.r_[command, (command - state[:6]) / node.dt]
        error, _ = segment_error(node._tcp_pose(command).translation,
                                 node.cartesian_path_start_pose.translation,
                                 node.target_pose.translation)
        assert np.linalg.norm(error) <= node.cartesian_path_tolerance
    final_pose = node._tcp_pose(state[:6])
    assert np.linalg.norm(final_pose.translation - node.target_pose.translation) < 0.0025
    assert np.linalg.norm(pin.log3(final_pose.rotation.T @ node.target_pose.rotation)) < 0.01
