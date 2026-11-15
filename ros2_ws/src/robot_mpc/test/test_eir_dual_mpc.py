"""Exercise both EIR hands and the shared torso through the real MPC callback."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import yaml

from robot_mpc.eir_mpc_node import EIRMPCNode
from robot_mpc.cartesian_path import segment_error


PACKAGE = Path(__file__).parents[1]
URDF = PACKAGE.parent / "robot_description/urdf/eir.urdf"


def pose_message(pose, node):
    message = PoseStamped()
    message.header.frame_id = "world"
    message.header.stamp = node.get_clock().now().to_msg()
    (message.pose.position.x, message.pose.position.y,
     message.pose.position.z) = map(float, pose.translation)
    (message.pose.orientation.x, message.pose.orientation.y,
     message.pose.orientation.z, message.pose.orientation.w) = map(
        float, pin.Quaternion(pose.rotation).coeffs())
    return message


@pytest.fixture
def dual_mpc(monkeypatch):
    params = yaml.safe_load((PACKAGE / "config/eir_dual_mpc.yaml").read_text())[
        "crocoddyl_mpc"]["ros__parameters"]
    params.update(urdf_path=str(URDF), enable_vive_follow=False,
                  start_paused=False, diagnostic_logging=False,
                  cartesian_path_limit_enabled=True, cartesian_path_tolerance=.005,
                  stationary_hold_duration=30.0)
    clock = [100.0]
    monkeypatch.setattr("robot_mpc.crocoddyl_mpc_node.time.monotonic", lambda: clock[0])
    context = Context()
    rclpy.init(args=[], context=context, domain_id=92)
    node = None
    try:
        node = EIRMPCNode(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()])
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        node._diagnostics_due = lambda: False
        node.status_pub = Mock()
        root = ET.parse(URDF).getroot()
        names = [j.get("name") for j in root.findall("./joint[@type='revolute']")]
        initial = {j.get("name"): float(j.findtext(
            "./state_interface[@name='position']/param[@name='initial_value']"))
            for j in root.findall("./ros2_control/joint")}

        def feedback(q, delay=0.0):
            measured = initial | dict(zip(node.joint_names, q))
            message = JointState()
            message.name = names
            message.position = [float(measured[name]) for name in names]
            stamp = clock[0] - delay
            message.header.stamp.sec = int(stamp)
            message.header.stamp.nanosec = int((stamp % 1) * 1e9)
            node._on_joint_state(message)
            return message

        feedback(node.sim_q)
        yield SimpleNamespace(node=node, clock=clock, feedback=feedback,
                              initial=initial, names=names)
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("delay_ticks", [0, 1])
def test_both_hands_and_shared_torso_with_delayed_feedback(dual_mpc, delay_ticks):
    session, node = dual_mpc, dual_mpc.node
    assert node.model.nq == node.model.nv == 16
    assert node.joint_names == ["Joint_L0", "Joint_L1"] + [
        f"Joint_L{i}_{side}" for side in ("L", "R") for i in range(2, 9)]
    assert not {"Joint_L2_U", "Joint_L3_U"}.intersection(node.joint_names)
    q = node.latest_state[:node.model.nq].copy()
    initial_right, initial_left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    right, left = initial_right.copy(), initial_left.copy()
    right.translation[0] += 0.02
    left.translation[0] += 0.02
    node._apply_target_pose(pose_message(right, node))
    node._apply_left_target(pose_message(left, node))
    pending = [q.copy()]
    node._publish_joint_command = lambda command: pending.__setitem__(0, command.copy())
    history = [q.copy() for _ in range(delay_ticks + 1)]
    for tick in range(400):
        session.clock[0] = 100.0 + tick * node.dt
        measured = history[-delay_ticks - 1]
        feedback = session.feedback(measured, delay_ticks * node.dt)
        node._solve_once()
        command = pending[0]
        assert command.shape == (16,)
        assert np.all(np.isfinite(command))
        assert np.all(command >= node.model.lowerPositionLimit)
        assert np.all(command <= node.model.upperPositionLimit)
        assert not node.stationary_hold_active  # Settling must not conceal oscillation.
        for head in ("Joint_L2_U", "Joint_L3_U"):
            assert feedback.position[feedback.name.index(head)] == session.initial[head]
        history.append(command.copy())
    for actual, goal in ((node._tcp_pose(history[-1]), right),
                         (node._left_pose(history[-1]), left)):
        assert np.linalg.norm(actual.translation - goal.translation) < 0.002
        assert np.linalg.norm(pin.log3(goal.rotation.T @ actual.rotation)) < 0.01
    for actual, start in ((node._tcp_pose(history[-1]), initial_right),
                          (node._left_pose(history[-1]), initial_left)):
        assert actual.translation[0] - start.translation[0] > 0.015
    assert np.max(np.abs(np.diff(history[-50:], axis=0))) < 1e-4


def test_torso_pitch_prefers_zero_more_strongly_than_yaw(dual_mpc):
    node = dual_mpc.node
    assert node.torso_preferred[1] == 0.0
    assert node.torso_pitch_cost_weight > node.torso_cost_weight
    for action in [*node.running_actions, node.terminal_action]:
        assert action.nominal_posture[1] == 0.0
        assert action.posture_weights[0] == node.torso_cost_weight
        assert action.posture_weights[1] == node.torso_pitch_cost_weight


def test_soft_path_error_does_not_move_the_original_line(dual_mpc):
    node = dual_mpc.node
    node.cartesian_path_limit_enabled = False
    q = node.latest_state[:node.model.nq].copy()
    node.cartesian_path_start_pose = node._tcp_pose(q).copy()
    node.left_path_start = node._left_pose(q).copy()
    node.cartesian_path_start_pose.translation[1] += .05
    node.left_path_start.translation[1] += .05
    node.target_pose = node.cartesian_path_start_pose.copy()
    node.left_target_pose = node.left_path_start.copy()
    node.target_pose.translation[0] += .1
    node.left_target_pose.translation[0] += .1
    starts = [node.cartesian_path_start_pose.translation.copy(), node.left_path_start.translation.copy()]
    node._update_cartesian_references(q)
    np.testing.assert_array_equal(node.cartesian_path_start_pose.translation, starts[0])
    np.testing.assert_array_equal(node.left_path_start.translation, starts[1])
    assert node.cartesian_path_deviation > .04


@pytest.mark.parametrize("side", ["left", "right"])
def test_changed_target_starts_at_measured_tcp_and_repeated_target_keeps_line(dual_mpc, side):
    node = dual_mpc.node
    node.cartesian_path_limit_enabled = False
    q = node.latest_state[:node.model.nq].copy()
    original = node._left_pose(q) if side == "left" else node._tcp_pose(q)
    goal = original.copy()
    goal.translation[0] += .1
    apply = node._apply_left_target if side == "left" else node._apply_target_pose
    apply(pose_message(goal, node))
    node._update_cartesian_references(q)
    q[0] += .01
    dual_mpc.feedback(q)
    current = (node._left_pose(q) if side == "left" else node._tcp_pose(q)).copy()
    dual_mpc.clock[0] += .05  # Retarget before the first segment has finished.
    goal.translation[1] += .005  # Even changes smaller than the tube tolerance count.
    apply(pose_message(goal, node))
    start = node.left_path_start if side == "left" else node.cartesian_path_start_pose
    np.testing.assert_allclose(start.translation, current.translation, atol=1e-12)
    node._update_cartesian_references(q)
    for action in [*node.running_actions, node.terminal_action]:
        actual_start = action.secondary_path_start if side == "left" else action.path_start
        actual_goal = action.secondary_path_goal if side == "left" else action.path_goal
        np.testing.assert_allclose(actual_start, current.translation, atol=1e-12)
        np.testing.assert_allclose(actual_goal, goal.translation, atol=1e-12)
    stamp = node.left_path_start_time if side == "left" else node.cartesian_path_start_time
    q[0] += .01
    dual_mpc.feedback(q)
    dual_mpc.clock[0] += .05
    apply(pose_message(goal, node))
    repeated_start = node.left_path_start if side == "left" else node.cartesian_path_start_pose
    repeated_stamp = node.left_path_start_time if side == "left" else node.cartesian_path_start_time
    np.testing.assert_array_equal(repeated_start.translation, start.translation)
    assert repeated_stamp == stamp


def test_live_torso_costs_update_existing_horizon_and_reject_invalid_edits(dual_mpc, tmp_path):
    node = dual_mpc.node
    config = tmp_path / "eir.yaml"
    document = yaml.safe_load((PACKAGE / "config/eir_dual_mpc.yaml").read_text())
    parameters = document["crocoddyl_mpc"]["ros__parameters"]
    node._yaml_parameters = dict(parameters)
    parameters.update(torso_cost_weight=.7, torso_pitch_cost_weight=91., posture_cost_weight=.09)
    config.write_text(yaml.safe_dump(document))
    node.cost_config_path, node.cost_config_mtime_ns = config, -1
    solver = node.solver
    state = node.latest_state.copy()
    node._reload_cost_config_if_changed()
    assert node.solver is solver
    np.testing.assert_array_equal(node.latest_state, state)
    assert node.get_parameter("torso_cost_weight").value == .7
    assert node.get_parameter("torso_pitch_cost_weight").value == 91.
    for action in [*node.running_actions, node.terminal_action]:
        np.testing.assert_allclose(action.posture_weights, [.7, 91.] + [.09] * 14)
    valid_mtime = node.cost_config_mtime_ns
    parameters.update(torso_cost_weight=3., torso_pitch_cost_weight=float("nan"), posture_cost_weight=2.)
    config.write_text(yaml.safe_dump(document))
    node._reload_cost_config_if_changed()
    assert node.cost_config_mtime_ns == valid_mtime
    assert node.torso_cost_weight == .7 and node.torso_pitch_cost_weight == 91.
    assert node.posture_cost_weight == .09
    assert node.get_parameter("torso_cost_weight").value == .7
    for action in [*node.running_actions, node.terminal_action]:
        np.testing.assert_allclose(action.posture_weights, [.7, 91.] + [.09] * 14)


@pytest.mark.parametrize("unsettled", ["left", None])
def test_stationary_hold_requires_both_hands(dual_mpc, unsettled):
    session, node = dual_mpc, dual_mpc.node
    node.stationary_hold_duration = 0.1
    q = node.latest_state[:node.model.nq].copy()
    node.last_q_cmd = q.copy()
    left = node._left_pose(q).copy()
    if unsettled == "left":
        left.translation[0] += 0.01
    node._apply_left_target(pose_message(left, node))
    state = np.r_[q, np.zeros(node.model.nv)]
    assert not node._stationary_hold_ready(state, np.zeros(6))
    session.clock[0] += 0.11
    assert node._stationary_hold_ready(state, np.zeros(6)) == (unsettled is None)


def test_f7_readiness_requires_fresh_left_hand_input(dual_mpc):
    session, node = dual_mpc, dual_mpc.node
    q = node.latest_state[:node.model.nq].copy()
    assert node._left_received is None
    assert "left" in node._execution_ready_reason()
    node._apply_left_target(pose_message(node._left_pose(q), node))
    assert not node._execution_ready_reason()

    session.clock[0] += node.joint_state_timeout + 0.1
    assert "left" in node._execution_ready_reason()
    node._apply_left_target(pose_message(node._left_pose(q), node))
    assert not node._execution_ready_reason()


def test_two_hand_velocity_closes_error_after_path_reference_stops(dual_mpc):
    node = dual_mpc.node
    q = node.sim_q.copy()
    right, left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    right.translation[0] += 0.04
    left.translation[0] += 0.04
    velocity = node._two_hand_velocity(q, right, right, left, left)
    node.pin.computeJointJacobians(node.model, node.pose_data, q)
    node.pin.updateFramePlacements(node.model, node.pose_data)
    for frame in (node.frame_id, node.left_frame_id):
        jacobian = node.pin.getFrameJacobian(node.model, node.pose_data, frame,
                                             node.pin.LOCAL_WORLD_ALIGNED)
        assert (jacobian @ velocity)[0] > 0.01
    assert np.max(np.abs(velocity[:2])) > 1e-4  # Torso is a free joint in the two-hand IK.
    assert np.max(np.abs(velocity)) <= 0.4 + 1e-12


def test_pitch_centering_does_not_displace_stationary_hands(dual_mpc):
    node = dual_mpc.node
    q = node.sim_q.copy()
    q[1] = 0.1
    right, left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    velocity = node._two_hand_velocity(q, right, right, left, left)
    node.pin.computeJointJacobians(node.model, node.pose_data, q)
    node.pin.updateFramePlacements(node.model, node.pose_data)
    assert velocity[1] < 0
    for frame in (node.frame_id, node.left_frame_id):
        jacobian = node.pin.getFrameJacobian(node.model, node.pose_data, frame,
                                             node.pin.LOCAL_WORLD_ALIGNED)
        assert np.linalg.norm(jacobian @ velocity) < 0.002


def test_mirror_guard_limits_one_hand_getting_ahead_on_shared_path(dual_mpc):
    node = dual_mpc.node
    q = node.sim_q.copy()
    right, left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    node.cartesian_path_start_pose = right.copy()
    node.left_path_start = left.copy()
    right.translation[0] += 0.1
    left.translation[0] += 0.1
    node.target_pose, node.left_target_pose = right, left
    jacobian = node._left_position_jacobian(q)[:, 2:9]
    direction = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-6 * np.eye(3),
                                              np.array([0.02, 0.0, 0.0]))
    proposed = q.copy()
    proposed[2:9] += direction
    mirror = np.array([1.0, -1.0, 1.0])
    def mirror_error(state):
        return np.linalg.norm(node._tcp_pose(state).translation - mirror * node._left_pose(state).translation)
    assert mirror_error(proposed) > node.cartesian_path_tolerance
    command = node._limit_task_command(proposed, q, q - 0.1, q + 0.1)
    assert mirror_error(command) <= node.cartesian_path_tolerance + 1e-9


def test_stalled_prediction_does_not_switch_to_dls(dual_mpc):
    node = dual_mpc.node
    measured = node.sim_q.copy()
    right, left = node._tcp_pose(measured).copy(), node._left_pose(measured).copy()
    node.cartesian_path_start_pose = right.copy()
    node.left_path_start = left.copy()
    right.translation += [0.24, 0.0, 0.18]
    left.translation += [0.24, 0.0, 0.18]
    node.target_pose = right
    node.left_target_pose = left
    command = node._safe_joint_command(measured, np.r_[measured, np.zeros(node.model.nv)])
    assert node._joint_limit_guard == ""
    assert np.allclose(command, measured)
    for pose, start, goal in ((node._tcp_pose(command), node.cartesian_path_start_pose, right),
                              (node._left_pose(command), node.left_path_start, left)):
        assert np.linalg.norm(segment_error(pose.translation, start.translation,
                                            goal.translation)[0]) < node.cartesian_path_tolerance


def test_joint_limit_hold_does_not_switch_to_dls_when_a_wrist_is_at_its_limit(dual_mpc):
    node = dual_mpc.node
    q = np.array([-.2108839317, .5175921615, -.1626888578, -.3605949329,
                  -1.4134609884, -1.8511584741, .0206409555, .4215006583,
                  -1.0472, 2.5134183086, 2.0407487804, -1.2049599910,
                  1.7826754542, 2.0200206299, .0644578375, -.2945042885])
    wrist = node.joint_names.index("Joint_L8_L")
    assert q[wrist] == pytest.approx(node.model.lowerPositionLimit[wrist])
    right, left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    node.cartesian_path_start_pose = right.copy()
    node.left_path_start = left.copy()
    node.target_pose = right.copy()
    node.left_target_pose = left.copy()
    node.left_target_pose.translation[1] -= 0.04
    node.last_q_cmd = q.copy()
    command = node._safe_joint_command(q, np.r_[q, np.zeros(node.model.nv)])
    assert node._joint_limit_guard == "joint_position_limit_hold"
    assert np.allclose(command, q)
    for pose, start, goal in ((node._tcp_pose(command), right, node.target_pose),
                              (node._left_pose(command), left, node.left_target_pose)):
        assert np.linalg.norm(segment_error(pose.translation, start.translation,
                                            goal.translation)[0]) <= node.cartesian_path_tolerance


def test_recorded_joint_limit_command_remains_inside_all_bounds(dual_mpc):
    node = dual_mpc.node
    q = np.array([-.2108839317, .5175921615, -.1626888578, -.3605949329,
                  -1.4134609884, -1.8511584741, .0206409555, .4215006583,
                  -1.0472, 2.5134183086, 2.0407487804, -1.2049599910,
                  1.7826754542, 2.0200206299, .0644578375, -.2945042885])
    predicted = np.array([-.0713333099, .2423022810, -.6578407579, -.9426608379,
                          -1.0569816171, -1.3145383872, .5208303493, .3435840747,
                          -1.6436731299, 1.9910450821, 1.8232140088, -1.3252997641,
                          1.4974251457, 1.8607568440, .0006026401, -.4356883961])
    def pose(position, xyzw):
        return pin.SE3(pin.Quaternion(np.array(xyzw)).matrix(), np.array(position))
    node.target_pose = pose([.4059847346, -.1114906191, 1.1021013872],
                            [.5000203191, .61237774895, -.4999750615, .3535507191])
    node.left_target_pose = pose([.4579777189, .3194989340, 1.0501017340],
                                 [-.5000219578, .6123699854, .4999826598, .3535511032])
    node.cartesian_path_start_pose = node._tcp_pose(q).copy()
    node.cartesian_path_start_pose.translation = np.array([.4018273982, -.1115382933, 1.1209690045])
    node.left_path_start = node._left_pose(q).copy()
    node.left_path_start.translation = np.array([.4565106480, .3783632776, 1.0504142558])
    node.last_q_cmd = q.copy()
    command = node._safe_joint_command(predicted, np.r_[q, np.zeros(node.model.nv)])
    assert np.all(command >= node.model.lowerPositionLimit)
    assert np.all(command <= node.model.upperPositionLimit)
    for actual, start, goal in ((node._tcp_pose(command), node.cartesian_path_start_pose, node.target_pose),
                                (node._left_pose(command), node.left_path_start, node.left_target_pose)):
        assert np.linalg.norm(segment_error(actual.translation, start.translation,
                                            goal.translation)[0]) <= node.cartesian_path_tolerance


def test_joint_limit_guard_allows_escape_when_tracking_merit_increases(dual_mpc):
    node = dual_mpc.node
    q = node.sim_q.copy()
    wrist = node.joint_names.index("Joint_L8_L")
    q[wrist] = node.model.lowerPositionLimit[wrist]
    right, left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    right.translation[0] += 0.001
    left.translation[0] += 0.001
    node.target_pose = right
    node.left_target_pose = left
    node.cartesian_path_start_pose = node._tcp_pose(q).copy()
    node.left_path_start = node._left_pose(q).copy()
    node.cartesian_path_limit_enabled = False
    node.workspace_floor_enabled = False
    node.last_q_cmd = q.copy()
    node._precise_hold_q = None
    predicted = q.copy()
    predicted[wrist] += 0.1

    def tracking_merit(configuration):
        right_error = node._pose_error(
            configuration, node._optimization_target_pose(), node.pose_data
        )
        left_error = node._left_error(configuration)
        return (
            node.se3_position_cost_weight
            * (right_error[:3] @ right_error[:3] + left_error[:3] @ left_error[:3])
            + node.se3_orientation_cost_weight
            * (right_error[3:] @ right_error[3:] + left_error[3:] @ left_error[3:])
        )

    initial_merit = tracking_merit(q)
    command = node._safe_joint_command(predicted, np.r_[q, np.zeros(node.model.nv)])

    assert command[wrist] > q[wrist]
    assert tracking_merit(command) > initial_merit
    assert np.all(command >= node.model.lowerPositionLimit)
    assert np.all(command <= node.model.upperPositionLimit)
    assert node._joint_limit_guard == "joint_position_limit_limited"


def test_f7_rechecks_header_age_even_after_recent_receipt(dual_mpc):
    node = dual_mpc.node
    q = node.latest_state[:node.model.nq]
    node._apply_left_target(pose_message(node._left_pose(q), node))
    node._left_stamp_s -= node.joint_state_timeout + 0.1
    assert "left" in node._execution_ready_reason()
    node._apply_left_target(pose_message(node._left_pose(q), node))


def test_limit_recovery_keeps_commands_inside_bounds_after_feedback_overshoot(dual_mpc):
    node = dual_mpc.node
    measured = node.sim_q.copy()
    index = node.joint_names.index("Joint_L8_L")
    bound = node.model.upperPositionLimit[index]
    measured[index] = bound + 0.001
    predicted = measured.copy()
    predicted[index] = bound + 0.0005
    node.target_pose = node._tcp_pose(measured).copy()
    node.cartesian_path_start_pose = node.target_pose.copy()
    node.left_target_pose = node._left_pose(predicted)
    node.left_path_start = node._left_pose(measured)
    node.last_q_cmd = measured.copy()
    command = node._safe_joint_command(predicted, np.r_[measured, np.zeros(node.model.nv)])
    assert np.all(command >= node.model.lowerPositionLimit)
    assert np.all(command <= node.model.upperPositionLimit)
    assert command[index] == bound


@pytest.mark.parametrize("delay_ticks", [0, 2])
def test_joint_limit_constrained_motion_stays_safe_and_recovers_on_reversal(dual_mpc, delay_ticks):
    session, node = dual_mpc, dual_mpc.node
    # Retain the recorded narrow-limit regression despite the wider simulation model.
    elbow = node.joint_names.index("Joint_L5_L")
    node.model.lowerPositionLimit[elbow] = -1.0472
    for action in [*node.running_actions, node.terminal_action]:
        action.soft_lower[elbow] = -1.0472 + node.joint_limit_margin
    assert node.model.lowerPositionLimit[elbow] == -1.0472
    q = node.latest_state[:node.model.nq].copy()
    right, initial_left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    left = initial_left.copy()
    node._apply_target_pose(pose_message(right, node))
    pending = [q.copy()]
    node._publish_joint_command = lambda command: pending.__setitem__(0, command.copy())
    history = [q.copy() for _ in range(delay_ticks + 1)]
    tick = 0

    def advance(count):
        nonlocal tick
        for _ in range(count):
            session.clock[0] = 100.0 + tick * node.dt
            measured = history[-delay_ticks - 1]
            session.feedback(measured, delay_ticks * node.dt)
            node._solve_once()
            command = pending[0].copy()
            assert np.all(command >= node.model.lowerPositionLimit)
            assert np.all(command <= node.model.upperPositionLimit)
            step = np.minimum(node.command_velocity_limits * node.dt, node.command_increment_limits)
            assert np.all(np.abs(command - measured) <= step + 1e-12)
            for fraction in (0.25, 0.5, 0.75, 1.0):
                sample = measured + fraction * (command - measured)
                for pose, start, goal in (
                    (node._tcp_pose(sample), node.cartesian_path_start_pose, right),
                    (node._left_pose(sample), node.left_path_start, left),
                ):
                    deviation = np.linalg.norm(segment_error(
                        pose.translation, start.translation, goal.translation)[0])
                    assert deviation <= node.cartesian_path_tolerance + 1e-9
                    assert pose.translation[2] >= node.workspace_min_z
            history.append(command)
            tick += 1

    left.translation[0] += 0.05
    node._apply_left_target(pose_message(left, node))
    advance(200)
    # Replay the recorded inward 1cm jogs until the left elbow saturates.
    for _ in range(18):
        left.translation[0] -= 0.01
        node._apply_left_target(pose_message(left, node))
        advance(10)
    advance(150)
    left.translation[2] += 0.09
    node._apply_left_target(pose_message(left, node))
    advance(150)
    assert np.all(history[-1] >= node.model.lowerPositionLimit)
    assert np.all(history[-1] <= node.model.upperPositionLimit)
    left = initial_left.copy()
    left.translation[0] += 0.02
    node._apply_left_target(pose_message(left, node))
    advance(300)
    assert np.linalg.norm(node._left_pose(history[-1]).translation - left.translation) < 0.002
    assert np.linalg.norm(node._tcp_pose(history[-1]).translation - right.translation) < 0.002
    assert not node._joint_limit_guard


@pytest.mark.parametrize("delay_ticks", [0, 2])
def test_wider_simulation_elbow_crosses_old_limit_and_reaches_the_inward_upward_target(dual_mpc, delay_ticks):
    session, node = dual_mpc, dual_mpc.node
    q = node.latest_state[:node.model.nq].copy()
    elbow = node.joint_names.index("Joint_L5_L")
    assert node.model.lowerPositionLimit[elbow] == pytest.approx(-2.5307)
    right, initial_left = node._tcp_pose(q).copy(), node._left_pose(q).copy()
    left = initial_left.copy()
    left.translation += np.array([-.13, 0., .09])
    node._apply_target_pose(pose_message(right, node))
    node._apply_left_target(pose_message(left, node))
    pending = [q.copy()]
    node._publish_joint_command = lambda command: pending.__setitem__(0, command.copy())
    history = [q.copy() for _ in range(delay_ticks + 1)]
    # The reached pose may enter stationary hold after the elbow crosses the old limit.
    for tick in range(800):
        session.clock[0] = 100.0 + tick * node.dt
        measured = history[-delay_ticks - 1]
        session.feedback(measured, delay_ticks * node.dt)
        node._solve_once()
        command = pending[0].copy()
        assert np.all(np.isfinite(command))
        assert np.all(command >= node.model.lowerPositionLimit)
        assert np.all(command <= node.model.upperPositionLimit)
        step = np.minimum(node.command_velocity_limits * node.dt, node.command_increment_limits)
        assert np.all(np.abs(command - measured) <= step + 1e-12)
        for fraction in (.25, .5, .75, 1.):
            sample = measured + fraction * (command - measured)
            for pose, start, goal in (
                (node._tcp_pose(sample), node.cartesian_path_start_pose, right),
                (node._left_pose(sample), node.left_path_start, left),
            ):
                deviation = np.linalg.norm(segment_error(
                    pose.translation, start.translation, goal.translation)[0])
                assert deviation <= node.cartesian_path_tolerance + 1e-9
                assert pose.translation[2] >= node.workspace_min_z
        history.append(command)
    assert history[-1][elbow] < -1.0472 - .001
    for actual, target in ((node._tcp_pose(history[-1]), right), (node._left_pose(history[-1]), left)):
        assert np.linalg.norm(actual.translation - target.translation) < .002
        assert np.linalg.norm(pin.log3(target.rotation.T @ actual.rotation)) < .01
    assert np.max(np.abs(np.diff(history[-50:], axis=0))) < 1e-4
    for pose in (node._tcp_pose, node._left_pose):
        positions = np.array([pose(state).translation.copy() for state in history[-50:]])
        assert np.max(np.linalg.norm(np.diff(positions, axis=0), axis=1)) < 1e-5
    assert not node._joint_limit_guard



def _watch_complete_yaml(node, tmp_path):
    config = tmp_path / 'mpc.yaml'
    parameters = {name: node.get_parameter(name).value for name in node._configuration_defaults}
    parameters['config_file_path'] = str(config)
    node.set_parameters([Parameter('config_file_path', value=str(config))])
    node.cost_config_path = config
    node._yaml_parameters = dict(parameters)
    config.write_text(yaml.safe_dump({'crocoddyl_mpc': {'ros__parameters': parameters}}))
    node.cost_config_mtime_ns = config.stat().st_mtime_ns
    return config, parameters


def _save_complete_yaml(node, config, parameters):
    config.write_text(yaml.safe_dump({'crocoddyl_mpc': {'ros__parameters': parameters}}))
    node._reload_cost_config_if_changed()


def test_live_yaml_updates_speed_limits_floor_costs_and_ros_parameters(dual_mpc, tmp_path):
    node = dual_mpc.node
    config, parameters = _watch_complete_yaml(node, tmp_path)
    measured = node.latest_state.copy()
    parameters.update(cartesian_max_speed=.23, command_increment_limits=[.012, .009] + [.03]*14,
                      command_smoothing_factor=.9, acceleration_limits=[55.]*16,
                      workspace_floor_cost_weight=2000000., workspace_floor_margin=.002,
                      se3_position_cost_weight=70000., max_solver_iterations=8,
                      joint_state_timeout=.8, short_move_speed=.3, diagnostic_rate=15.,
                      stationary_hold_position_tolerance=.001, pose_velocity_feedforward_damping=.06)
    _save_complete_yaml(node, config, parameters)
    assert node.cartesian_max_speed == .23 and node.command_smoothing_factor == .9
    np.testing.assert_allclose(node.command_increment_limits, [.012, .009] + [.03]*14)
    np.testing.assert_array_equal(node.latest_state, measured)
    assert node.execution_enabled
    assert node.diagnostic_period == 1/15.
    for name in parameters:
        assert node.get_parameter(name).value == parameters[name]
    for action in [*node.running_actions, node.terminal_action]:
        assert action.floor_weight == 2000000. and action.floor_margin == .002
        assert action.weights['se3_position'] == 70000.
        np.testing.assert_allclose(action.u_ub, [55.]*16)
    node._solve_once()
    assert np.all(np.isfinite(node.last_q_cmd))


def test_live_yaml_defers_structural_changes_and_applies_other_tuning(dual_mpc, tmp_path):
    node = dual_mpc.node
    config, parameters = _watch_complete_yaml(node, tmp_path)
    solver = node.solver
    parameters.update(dt=.08, horizon=7, prediction_model='rigid_body',
                      command_pose_topic='/eir/test/new', simulate=False, cartesian_max_speed=.23)
    _save_complete_yaml(node, config, parameters)
    assert node.solver is solver and node.dt == .05 and node.horizon == 10
    assert node.get_parameter('dt').value == .05
    assert node.prediction_model == 'joint_acceleration' and node.simulate
    assert node.command_pose_pub.topic_name != '/eir/test/new'
    assert node.cartesian_max_speed == .23 and node.execution_enabled


@pytest.mark.parametrize('invalid', [
    {'cartesian_max_speed': -1.}, {'dt': float('nan')}, {'horizon': 2.5},
    {'command_increment_limits': [.03]*15}, {'acceleration_limits': [-1.]*16},
    {'joint_limit_margin': 100.}, {'cartesian_max_speeed': .3}, {'dt': True},
])
def test_invalid_complete_yaml_keeps_parameters_solver_and_state(dual_mpc, tmp_path, invalid):
    node = dual_mpc.node
    config, parameters = _watch_complete_yaml(node, tmp_path)
    solver, model, state = node.solver, node.model, node.latest_state.copy()
    old = {name: node.get_parameter(name).value for name in node._configuration_defaults}
    stamp = node.cost_config_mtime_ns
    parameters.update(se3_position_cost_weight=90000., **invalid)
    _save_complete_yaml(node, config, parameters)
    assert node.solver is solver and node.model is model
    assert node.cost_config_mtime_ns == stamp
    np.testing.assert_array_equal(node.latest_state, state)
    assert {name: node.get_parameter(name).value for name in node._configuration_defaults} == old


def test_live_yaml_applies_goal_nominal_posture_logging_and_removed_defaults(dual_mpc, tmp_path):
    node = dual_mpc.node
    config, parameters = _watch_complete_yaml(node, tmp_path)
    nominal = list(parameters['nominal_posture'])
    nominal[:2] = [.1, .02]
    target = node._tcp_pose(node.latest_state[:node.model.nq]).translation.copy()
    target[0] += .02
    parameters.update(goal_position=target.tolist(), nominal_posture=nominal,
                      command_smoothing_factor=.9, diagnostic_logging=True,
                      diagnostic_log_directory=str(tmp_path / 'traces'))
    _save_complete_yaml(node, config, parameters)
    np.testing.assert_allclose(node.target_pose.translation, target)
    np.testing.assert_allclose(node.torso_preferred, [.1, .02])
    assert node._diagnostic_file is not None and node._diagnostic_path.is_file()
    parameters.pop('command_smoothing_factor')
    parameters['diagnostic_logging'] = False
    _save_complete_yaml(node, config, parameters)
    assert node.command_smoothing_factor == .75
    assert node.get_parameter('command_smoothing_factor').value == .75
    assert node._diagnostic_file is None


def test_bad_log_directory_rejects_cost_and_ros_parameter_update(dual_mpc, tmp_path):
    node = dual_mpc.node
    config, parameters = _watch_complete_yaml(node, tmp_path)
    blocker = tmp_path / 'not_a_directory'
    blocker.write_text('keep this file')
    weight, solver = node.se3_position_cost_weight, node.solver
    parameters.update(diagnostic_logging=True, diagnostic_log_directory=str(blocker),
                      se3_position_cost_weight=90000.)
    _save_complete_yaml(node, config, parameters)
    assert node.se3_position_cost_weight == weight
    assert node.get_parameter('se3_position_cost_weight').value == weight
    assert node.solver is solver
    assert node.running_action.weights['se3_position'] == weight
    assert node._diagnostic_file is None and node.execution_enabled
