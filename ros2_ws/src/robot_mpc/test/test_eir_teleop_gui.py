"""EIR target editing and deadman controls without a robot or controller."""

import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from rclpy.context import Context
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_mpc.eir_teleop_gui import EIRTeleopNode, TORSO_JOINTS, create_window
from robot_mpc.pose_mapping import transform_from_pose, transform_from_rpy


class Client:
    is_closed = False

    def __init__(self):
        self.state = dict(simulation={"world_frame": "world", "runtime_id": "first"},
                          simulation_current=([0.3, -0.3, 1.0, 0., 0., 0., 1.], "world", time.monotonic()),
                          simulation_fresh=True, input_mode="keyboard", can_change_mode=False,
                          can_run_simulation=True, can_stop_simulation=False,
                          run_held=False, simulation_running=False, result="Ready")
        self.requests = []
        self.keyboard = None
        self.hold_count = 0

    def snapshot(self):
        return dict(self.state)

    def set_keyboard_target(self, pose):
        self.keyboard = list(pose)

    def publish_keyboard_target(self):
        pass

    def publish_run_hold(self):
        self.hold_count += 1

    def request_run(self, enabled):
        self.requests.append(enabled)
        self.state["run_held"] = enabled
        return True

    def close(self):
        self.is_closed = True


@pytest.fixture
def session(qapp):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=94)
    node = EIRTeleopNode(context=context)
    left = PoseStamped()
    left.header.frame_id = "world"
    left.pose.position.x, left.pose.position.y, left.pose.position.z = .3, .3, 1.
    left.pose.orientation.w = 1.
    node._on_left(left)
    joints = JointState(name=list(TORSO_JOINTS), position=[.1, .2])
    node._on_joints(joints)
    node.left_pub = Mock()
    client = Client()
    window = create_window(node, client)
    window.timer.stop()
    window.show()
    window.activateWindow()
    qapp.processEvents()
    try:
        yield SimpleNamespace(node=node, client=client, window=window, left=left, joints=joints, app=qapp)
    finally:
        window.close()
        node.destroy_node()
        context.shutdown()


def test_simultaneous_and_independent_hand_targets(session):
    s = session
    assert not s.client.requests
    s.window.jog_hand("left", 0, 1)
    assert s.window.targets["left"][0] == pytest.approx(.31)
    assert s.client.keyboard[0] == pytest.approx(.31)
    # New feedback must update the reading without replacing either edited goal.
    s.left.pose.position.x = .305
    s.node._on_left(s.left)
    s.window.refresh()
    assert s.window.targets["left"][0] == pytest.approx(.31)
    s.window.both_hands.setChecked(False)
    s.window.jog_hand("right", 1, -1)
    assert s.window.targets["left"][1] == pytest.approx(.3)
    assert s.client.keyboard[1] == pytest.approx(-.31)
    message = s.node.left_pub.publish.call_args.args[0]
    assert message.header.frame_id == "world"
    assert message.pose.position.x == pytest.approx(.31)


def test_world_rotation_preserves_each_hand_position(session):
    s = session
    initial = {side: list(pose) for side, pose in s.window.targets.items()}
    s.window.jog_hand("right", 2, 1, rotation=True)
    expected = transform_from_rpy([0, 0, 0], [0, 0, 5])[:3, :3]
    for side, pose in s.window.targets.items():
        np.testing.assert_allclose(pose[:3], initial[side][:3])
        np.testing.assert_allclose(transform_from_pose(pose[:3], pose[3:])[:3, :3], expected)


def test_torso_is_feedback_only_and_reset_uses_hand_feedback(session):
    s = session
    assert "automatic" in s.window.torso_label.text()
    assert not hasattr(s.node, "torso_pub")
    assert not hasattr(s.window, "torso_buttons")
    s.joints.position = [.11, .21]
    s.node._on_joints(s.joints)
    s.window.refresh()
    assert "6.30°" in s.window.torso_label.text()
    assert not s.client.requests
    assert s.window.reset_targets()
    assert s.window.targets["left"][0] == pytest.approx(.3)


def test_f7_release_focus_loss_and_f5_work_from_child_control(session):
    s = session
    child = s.window.translation_step
    child.setFocus()
    QTest.keyPress(child, Qt.Key_F7)
    assert s.client.requests == [True]
    assert not s.window.reset_targets()
    QTest.keyRelease(child, Qt.Key_F7)
    assert s.client.requests == [True, False]
    QTest.keyPress(child, Qt.Key_F7)
    QApplication.sendEvent(s.window, QEvent(QEvent.WindowDeactivate))
    assert s.client.requests[-2:] == [True, False]
    QTest.keyRelease(child, Qt.Key_F7)
    s.window.jog_hand("left", 0, 1)
    QTest.keyClick(child, Qt.Key_F5)
    assert s.window.targets["left"][0] == pytest.approx(.3)


def test_status_guides_keyboard_control_without_hiding_errors(session):
    s = session
    s.client.state["result"] = "Hold both poses still, then click Map or press F5."
    s.window.refresh()
    assert "Edit targets, then check Run MPC" in s.window.status.text()
    s.client.state["result"] = "Simulation request rejected: stale feedback"
    s.window.refresh()
    assert s.client.state["result"] in s.window.status.text()


@pytest.mark.parametrize("code,reason", [
    ("joint_position_limit_hold", "Joint limit is blocking progress"),
    ("joint_position_limit_limited", "Joint limit is constraining this move"),
    ("left_cartesian_path_hold", "Left hand path constraint blocks this move"),
    ("mirror_path_hold", "Relative hand path constraint blocks this move"),
    ("mirror_path_limited", "Relative hand path constraint limits this move"),
])
def test_controller_guard_is_visible_while_f7_execution_still_runs(session, code, reason):
    s = session
    s.client.state["simulation_running"] = True
    s.node._on_joint_limit_active(String(data="Joint_L5_L, Joint_L3_R"))
    s.node._on_control_status(String(data=code))
    s.window.refresh()
    assert "RUNNING" in s.window.status.text()
    assert reason in s.window.control_status.text()
    assert code in s.window.control_status.text()
    if code.startswith("joint_position_limit_"):
        assert "Active joints: Joint_L5_L, Joint_L3_R" in s.window.control_status.text()
    s.node._on_control_status(String(data="solved"))
    s.window.refresh()
    assert reason not in s.window.control_status.text()
    assert "Tracking target" in s.window.control_status.text()
    s.node._on_control_status(String(data="future_guard"))
    s.window.refresh()
    assert "future_guard" in s.window.control_status.text()
    s.node._control_status = (code, time.monotonic() - 2.)
    s.window.refresh()
    assert s.window.control_status.text() == "Controller: waiting for status"


def test_invalid_or_stale_feedback_cannot_start_and_stops_held_run(session):
    s = session
    original = s.node.feedback()["left"]
    s.left.pose.position.x = float("nan")
    s.node._on_left(s.left)
    assert s.node.feedback()["left"] == original
    s.window.start_run("button")
    assert s.client.requests == [True]
    pose, frame, stamp = s.node._left_current
    s.node._left_current = (pose, frame, stamp - 1.)
    s.window.refresh()
    assert s.client.requests == [True, False]
    s.window.start_run("button")
    assert s.client.requests == [True, False]
    assert not s.window.hand_buttons["left"][0].isEnabled()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('event', ['POSE_UPDATE', 'MOUSE_UP'])
def test_rviz_marker_edits_full_pose_for_one_hand_and_persists(session, side, event):
    from visualization_msgs.msg import InteractiveMarkerControl as Control, InteractiveMarkerFeedback as Feedback
    s = session
    s.node._refresh_markers()
    marker = s.node.marker_server.get(f'{side}_desired_tcp')
    assert marker is not None and marker.header.frame_id == 'world'
    for mode in (Control.MOVE_AXIS, Control.MOVE_PLANE, Control.ROTATE_AXIS):
        controls = [c for c in marker.controls if c.interaction_mode == mode]
        assert len(controls) == 3
        assert all(c.orientation_mode == Control.FIXED for c in controls)
        if mode == Control.MOVE_PLANE:
            assert all(len(c.markers) == 1 and c.markers[0].type == c.markers[0].CUBE
                       for c in controls)
        else:
            assert all(not c.markers for c in controls)  # RViz's default arrows/rings.
    other = 'right' if side == 'left' else 'left'
    original_other = list(s.window.targets[other])
    feedback = Feedback(marker_name=marker.name, event_type=getattr(Feedback, event))
    feedback.header.frame_id = 'world'
    feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = .34, .32, 1.02
    feedback.pose.orientation.z, feedback.pose.orientation.w = 1., 1.  # Normalize at input.
    s.node.marker_server.processFeedback(feedback)
    for _ in range(3):
        s.window.refresh()
        s.node._refresh_markers()
    expected = [.34, .32, 1.02, 0., 0., np.sqrt(.5), np.sqrt(.5)]
    np.testing.assert_allclose(s.window.targets[side], expected)
    assert s.window.targets[other] == original_other
    assert not s.client.requests
    if side == 'left':
        sent = s.node.left_pub.publish.call_args.args[0].pose
        np.testing.assert_allclose([sent.position.x, sent.position.y, sent.position.z,
                                    sent.orientation.x, sent.orientation.y,
                                    sent.orientation.z, sent.orientation.w], expected)
    else:
        np.testing.assert_allclose(s.client.keyboard, expected)
    assert s.window.reset_targets()
    assert s.window.targets[side][0] == .3
    np.testing.assert_allclose(s.window.targets[side][3:], [0., 0., 0., 1.])


@pytest.mark.parametrize('side', ['left', 'right'])
def test_rviz_orientation_only_edit_updates_marker_without_moving_position(session, side):
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    s = session
    s.node._refresh_markers()
    feedback = Feedback(marker_name=f'{side}_desired_tcp', event_type=Feedback.POSE_UPDATE)
    feedback.header.frame_id = 'world'
    original = list(s.window.targets[side])
    feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = original[:3]
    feedback.pose.orientation.x, feedback.pose.orientation.w = np.sin(.1), np.cos(.1)
    s.node.marker_server.processFeedback(feedback)
    s.window.refresh()
    s.node._refresh_markers()
    pose = s.node.marker_server.get(feedback.marker_name).pose
    np.testing.assert_allclose(s.window.targets[side][:3], original[:3])
    np.testing.assert_allclose(s.window.targets[side][3:], [np.sin(.1), 0., 0., np.cos(.1)])
    assert pose.orientation.x == pytest.approx(np.sin(.1))
    assert pose.orientation.w == pytest.approx(np.cos(.1))


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('quaternion', [
    (0., 0., 0., 1.),
    (0., 0., float(np.float32(np.sin(.1))), float(np.float32(np.cos(.1)))),
], ids=['identity', 'rviz_float32'])
def test_drag_feedback_is_not_echoed_but_gui_jog_still_updates_marker(session, side, quaternion):
    from unittest.mock import patch
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    s = session
    s.node._refresh_markers()
    feedback = Feedback(marker_name=f'{side}_desired_tcp', event_type=Feedback.POSE_UPDATE)
    feedback.header.frame_id = 'world'
    pose = list(s.window.targets[side])
    pose[0] += .04
    pose[3:] = (np.asarray(quaternion) / np.linalg.norm(quaternion)).tolist()
    feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = pose[:3]
    (feedback.pose.orientation.x, feedback.pose.orientation.y,
     feedback.pose.orientation.z, feedback.pose.orientation.w) = quaternion
    with patch.object(s.node.marker_server, 'setPose', wraps=s.node.marker_server.setPose) as update:
        s.node.marker_server.processFeedback(feedback)
        for _ in range(3):
            s.window.refresh()
            s.node._refresh_markers()
        update.assert_not_called()
        assert s.node.marker_server.get(feedback.marker_name).pose.position.x == pose[0]
        np.testing.assert_allclose(s.window.targets[side], pose)
        s.window.both_hands.setChecked(False)
        s.window.jog_hand(side, 0, 1)
        s.node._refresh_markers()
        update.assert_called_once()
        assert s.node.marker_server.get(feedback.marker_name).pose.position.x == pytest.approx(pose[0]+.01)
        # A final mouse-up pose can differ from the last drag feedback.
        feedback.event_type = Feedback.MOUSE_UP
        feedback.pose.position.x += .02
        s.node.marker_server.processFeedback(feedback)
        # The release must commit even if the last POSE_UPDATE was dropped,
        # before either of the 50 ms GUI/marker timers runs.
        assert s.node.marker_server.get(feedback.marker_name).pose.position.x == feedback.pose.position.x
        s.window.refresh()
        s.node._refresh_markers()
        assert update.call_count == 2  # One GUI jog and one immediate release.
        assert s.node.marker_server.get(feedback.marker_name).pose.position.x == feedback.pose.position.x


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('bad', ['frame', 'position', 'quaternion', 'zero_quaternion', 'marker', 'event'])
def test_rviz_marker_ignores_invalid_feedback(session, side, bad):
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    s = session
    original = {hand: list(pose) for hand, pose in s.window.targets.items()}
    feedback = Feedback(marker_name=f'{side}_desired_tcp', event_type=Feedback.POSE_UPDATE)
    feedback.header.frame_id = 'other_frame' if bad == 'frame' else 'world'
    feedback.pose.position.x = float('nan') if bad == 'position' else .4
    feedback.pose.orientation.w = 0. if bad == 'zero_quaternion' else 1.
    if bad == 'quaternion':
        feedback.pose.orientation.x = float('nan')
    if bad == 'marker':
        feedback.marker_name = 'unknown_tcp'
    if bad == 'event':
        feedback.event_type = Feedback.MOUSE_DOWN
    s.node._on_marker_feedback(feedback)
    s.window.refresh()
    assert s.window.targets == original


@pytest.mark.parametrize('reset', ['button', 'runtime'])
def test_reset_discards_pending_edits_for_both_hands(session, reset):
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    s = session
    for side in ('left', 'right'):
        feedback = Feedback(marker_name=f'{side}_desired_tcp', event_type=Feedback.POSE_UPDATE)
        feedback.header.frame_id = 'world'
        feedback.pose.position.x = .4
        feedback.pose.orientation.w = 1.
        s.node._on_marker_feedback(feedback)
    if reset == 'button':
        assert s.window.reset_targets()
    else:
        s.client.state['simulation'] = dict(world_frame='world', runtime_id='second')
        s.window.refresh()
    assert all(pose[0] == .3 for pose in s.window.targets.values())
    assert not s.node._marker_edits
