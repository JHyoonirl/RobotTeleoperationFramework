"""Floor residual derivatives, continuous steps, and the real hand-target GUI."""

import json
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from scipy.optimize import brentq
from sensor_msgs.msg import JointState

from indy7_mpc.se3_dynamic_model import SE3DynamicActionModel
from indy7_mpc.workspace_floor import limit_height_step
from test_cartesian_path import node
from test_hand_teleop_modes import session


@pytest.mark.parametrize("initial, proposed", [(0.1005, 0.08), (0.1, 0.12)])
def test_step_stops_at_floor_and_allows_upward_departure(initial, proposed):
    command, scale = limit_height_step(
        np.array([initial]), np.array([proposed]), lambda q: np.array([0, 0, q[0]]),
        lambda q: np.array([[0], [0], [1]]), 0.1, 1.0,
    )
    assert scale > 0
    assert min(initial, command[0]) >= 0.1
    if proposed > initial:
        assert command[0] == pytest.approx(proposed)
    else:
        assert scale < 1


def test_safe_endpoints_cannot_hide_a_dip_below_the_floor():
    position = lambda q: np.array([0, 0, 0.0996 + q[0] ** 2])
    jacobian = lambda q: np.array([[0], [0], [2 * q[0]]])
    initial, proposed = np.array([-0.025]), np.array([0.175])
    assert position(initial)[2] > 0.1 and position(proposed)[2] > 0.1
    # Even all five sampled positions are safe; the dip lies between them.
    assert min(position(initial + t * (proposed - initial))[2]
               for t in np.linspace(0, 1, 5)) > 0.1
    command, scale = limit_height_step(initial, proposed, position, jacobian, 0.1, 2.0)
    assert 0 < scale < 1
    assert min(position(initial + t * (command - initial))[2]
               for t in np.linspace(0, 1, 1001)) >= 0.1


@pytest.mark.parametrize("initial, invalid_fk", [(0.09, False), (0.2, True)])
def test_invalid_or_already_below_floor_holds(initial, invalid_fk):
    measured = np.array([initial])
    command, scale = limit_height_step(
        measured, measured + 0.01,
        lambda q: np.full(3, np.nan) if invalid_fk else np.array([0, 0, q[0]]),
        lambda q: np.array([[0], [0], [1]]), 0.1, 1.0,
    )
    np.testing.assert_array_equal(command, measured)
    assert scale == 0


@pytest.mark.parametrize("terminal", [False, True])
def test_mpc_floor_cost_uses_world_height_and_correct_derivative(node, terminal):
    q = node.sim_q.copy()
    height = float(node._tcp_pose(q).translation[2])
    action = SE3DynamicActionModel(
        crocoddyl=node.crocoddyl, pin=node.pin, model=node.model, state=node.state,
        frame_id=node.frame_id, target=node._tcp_pose(q), nominal_posture=q,
        weights=dict.fromkeys(("se3_position", "se3_orientation", "posture", "velocity",
                               "torque", "joint_limit"), 0.0),
        minimum_z=height + 0.01, floor_margin=0.001, floor_weight=1e6,
        terminal=terminal, terminal_scale=2.0,
    )
    x, u = np.r_[q, np.zeros(6)], np.zeros(6)
    data = action.createData()
    action.calc(data, x, u)
    action.calcDiff(data, x, u)
    gradient = data.Lx.copy()
    assert data.cost == pytest.approx(1e6 * 0.011 ** 2)
    numerical = np.zeros(6)
    for index in range(6):
        delta = np.zeros(12)
        delta[index] = 1e-6
        numerical[index] = (action.cost_terms(x + delta, u)["workspace_floor"]
                            - action.cost_terms(x - delta, u)["workspace_floor"]) / 2e-6
    np.testing.assert_allclose(gradient[:6], numerical, rtol=1e-5, atol=1e-4)
    assert np.min(np.linalg.eigvalsh(data.Lxx)) >= 0
    # Disabling the floor restores the original action cost, with no penalty.
    action.minimum_z = None
    assert action.cost_terms(x, u)["workspace_floor"] == 0


def test_real_arm_command_guard_checks_hand_during_joint_interpolation(node):
    node.frame_id = node.model.getFrameId("hand_tcp")
    node.cartesian_reach_bound += 0.2
    q = node.sim_q.copy()

    def height(angle):
        q[1] = angle
        return node._tcp_pose(q).translation[2] - 0.1002

    angles = np.linspace(node.model.lowerPositionLimit[1] + 0.02,
                         node.model.upperPositionLimit[1] - 0.02, 101)
    bracket = next((a, b) for a, b in zip(angles[:-1], angles[1:]) if height(a) * height(b) < 0)
    q[1] = brentq(height, *bracket)
    measured = q.copy()
    node.control_mode = "joint"  # Floor also applies without the line/path guard.
    descending = -node._tcp_position_jacobian(measured)[2]
    proposed = measured + 0.02 * descending / np.max(np.abs(descending))
    command = node._safe_joint_command(proposed, np.r_[measured, np.zeros(6)])
    assert node.workspace_floor_guard == "workspace_floor_limited"
    assert not np.array_equal(command, measured)
    assert min(node._tcp_pose(measured + t * (command - measured)).translation[2]
               for t in np.linspace(0, 1, 1001)) >= 0.1
    assert np.all(np.abs(command - measured) <= 0.02 + 1e-12)


@pytest.mark.usefixtures("qapp")
def test_keyboard_desired_pose_is_preserved_and_f7_stays_above_floor(session):
    s = session
    s.select_mode("keyboard")
    s.wait_for(lambda: s.client.snapshot()["can_run_simulation"])
    goal = s.client.snapshot()["keyboard_pose"].copy()
    goal[2] = -0.2
    assert s.client.set_keyboard_target(goal)
    assert s.client.snapshot()["keyboard_pose"][2] == pytest.approx(-0.2)
    s.wait_for(lambda: s.previews and s.previews[-1].pose.position.z == pytest.approx(-0.2))
    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    s.wait_for(lambda: s.current_hand()[2, 3] < 0.105, timeout=15)
    assert s.current_hand()[2, 3] >= 0.1
    assert s.mpc.target_pose.translation[2] == pytest.approx(-0.2)
    assert s.mpc._optimization_target_pose().translation[2] == pytest.approx(0.101)
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: not s.client.snapshot()["simulation_running"])


@pytest.mark.usefixtures("qapp")
def test_below_floor_start_blocks_f7_and_robot_alignment(session):
    s = session
    s.select_mode("keyboard")
    q = s.mpc.sim_q.copy()

    def height(angle):
        q[1] = angle
        return s.mpc._tcp_pose(q).translation[2] - 0.09

    angles = np.linspace(-2.0, 2.0, 101)
    bracket = next((a, b) for a, b in zip(angles[:-1], angles[1:]) if height(a) * height(b) < 0)
    q[1] = brentq(height, *bracket)
    s.mpc.sim_q = q.copy()
    s.wait_for(lambda: "below world Z" in s.client.snapshot()["simulation"].get("reason", ""))
    assert not s.client.request_run(True)
    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.app.processEvents()
    assert not s.mpc.execution_enabled and s.mpc.solve_count == 0
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    acknowledgement = Mock()
    s.mpc.sim_sync_ack_pub = acknowledgement
    request = JointState()
    request.name, request.position = s.mpc.joint_names, q.tolist()
    s.mpc._on_sim_sync_joint_target(request)
    result = json.loads(acknowledgement.publish.call_args.args[0].data)
    assert not result["success"] and "below world Z" in result["message"]
    np.testing.assert_array_equal(s.mpc.sim_q, q)


@pytest.mark.usefixtures("qapp")
@pytest.mark.parametrize("source", ["keyboard", "topic"])
def test_both_input_sources_preserve_desired_below_floor(session, source):
    s = session
    message = PoseStamped()
    message.header.frame_id = "world"
    message.header.stamp = s.peer.get_clock().now().to_msg()
    message.pose.position.x, message.pose.position.y, message.pose.position.z = 0.4, 0.2, -1.0
    message.pose.orientation.w = 1.0
    callback = s.mpc.vive_control._on_keyboard_target if source == "keyboard" else s.mpc.vive_control._on_target
    callback(message)
    target = s.mpc.vive_control._keyboard_target if source == "keyboard" else s.mpc.vive_control._target
    np.testing.assert_allclose(target[0][:3, 3], [0.4, 0.2, -1.0])
    np.testing.assert_allclose(target[0][:3, :3], np.eye(3))
    s.mpc._apply_target_pose(message)
    np.testing.assert_allclose(s.mpc.target_pose.translation, [0.4, 0.2, -1.0])
    np.testing.assert_allclose(s.mpc._optimization_target_pose().translation, [0.4, 0.2, 0.101])
