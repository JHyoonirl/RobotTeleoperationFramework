"""F6/F7 against the actual GUI, MPC and DCP3 adapter with a fake vendor SDK."""

import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster

from test_hand_teleop_modes import session
from indy7_mpc.pose_mapping import pose_from_transform, transform_from_pose
from indy7_mpc.vive_pose_mapper import VivePoseMapper

pytestmark = pytest.mark.usefixtures("qapp")


class FakeRobot:
    def __init__(self):
        self.q = np.degrees([0.1, -0.2, -1.1, 0.15, -1.3, 0.1])
        self.op_state = 5
        self.starts, self.moves, self.stops = [], [], []
        self.feedback_ok = True
        self.reject_start = False
        self.reject_stop = False
        self.confirm_delay = 0.15
        self.follow_fraction = 1.0
        self._started_at = None

    def get_control_data(self):
        if not self.feedback_ok:
            raise RuntimeError("injected feedback failure")
        if self._started_at is not None and time.monotonic() - self._started_at >= self.confirm_delay:
            self.op_state = 17
        return {"q": self.q.tolist(), "qdot": [0.0] * 6, "op_state": self.op_state}

    def start_teleop(self, *, method):
        self.starts.append(method)
        if self.reject_start:
            raise RuntimeError("injected start rejection")
        self._started_at = time.monotonic()

    def movetelej_abs(self, **command):
        self.moves.append(command)
        self.q += self.follow_fraction * (np.asarray(command["jpos"], dtype=float) - self.q)

    def stop_motion(self):
        self.stops.append("motion")
        if self.reject_stop:
            raise RuntimeError("injected stop rejection")

    def stop_teleop(self):
        self.stops.append("teleop")
        if self.reject_stop:
            raise RuntimeError("injected stop rejection")
        self.op_state, self._started_at = 5, None


@pytest.fixture
def robot(session, monkeypatch):
    s = session
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "indy7_dcp3"))
    from indy7_dcp3.driver_node import Indy7Dcp3Node

    fake = FakeRobot()
    driver = Indy7Dcp3Node(context=s.context, client_factory=lambda address: fake,
                          parameter_overrides=[Parameter("robot_ip", value="sdk-mock"),
                                               Parameter("publish_rate", value=100.0)])
    s.executor.add_node(driver)
    s.nodes.append(driver)
    s.wait_for(lambda: s.window.node.snapshot()["robot_q"] is not None
               and time.monotonic() - s.window.node.snapshot()["robot_status_time"] < 1)
    result = SimpleNamespace(s=s, sdk=fake, driver=driver, connection=s.window.robot_connection)
    try:
        yield result
    finally:
        fake.feedback_ok, fake.reject_stop = True, False
        if result.connection.snapshot()["active"]:
            result.connection.disconnect()
            deadline = time.monotonic() + 1.0
            while result.connection.phase != "off" and time.monotonic() < deadline:
                s.app.processEvents()
                s.executor.spin_once(timeout_sec=0.005)
                s.panel.refresh()


def connect_keyboard(robot):
    s = robot.s
    s.select_mode("keyboard")
    s.wait_for(lambda: s.panel.connect_button.isEnabled())
    s.panel.translation_step.setFocus()
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "connected")


def test_f6_aligns_and_enters_teleop_without_moving_until_f7(robot):
    s, sdk, driver = robot.s, robot.sdk, robot.driver
    original = sdk.q.copy()
    assert not sdk.starts and not sdk.moves and not driver.armed
    s.select_mode("keyboard")
    s.wait_for(lambda: s.panel.connect_button.isEnabled())
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "arming")
    assert not s.panel.run_button.isEnabled()
    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    assert not s.client.snapshot()["run_held"]
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: robot.connection.phase == "connected")
    assert driver.armed and sdk.op_state == 17 and sdk.starts == [10]
    assert not sdk.moves and s.mpc.solve_count == 0
    np.testing.assert_allclose(sdk.q, original, atol=1e-10)
    np.testing.assert_allclose(s.mpc.sim_q, np.radians(original), atol=1e-10)
    np.testing.assert_allclose(s.client.snapshot()["keyboard_pose"][:3], s.current_hand()[:3, 3], atol=1e-9)
    assert s.window.stream_check.isChecked() and s.window.deadman_check.isChecked()
    assert not s.window.arm_button.isEnabled()
    assert "TELE_OP ON" in s.panel.connection_label.text()
    s.app.processEvents()
    assert s.window.centralWidget().horizontalScrollBar().maximum() == 0
    assert s.window.grab().save("/tmp/indy7-robot-connected-gui.png")

    QTest.keyClick(s.panel.translation_step, Qt.Key_F7)  # Release before alignment is acknowledged.
    s.wait_for(lambda: not robot.connection.follow_pending and not s.client.snapshot()["pending"]
               and not s.mpc.execution_enabled)
    until = time.monotonic() + 0.1
    s.wait_for(lambda: time.monotonic() > until)
    assert not sdk.moves and s.mpc.solve_count == 0

    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    QTest.keyClick(s.panel.translation_step, Qt.Key_W)
    s.wait_for(lambda: bool(sdk.moves))
    np.testing.assert_allclose(np.radians(sdk.moves[-1]["jpos"]), driver.last_sent_q, atol=1e-12)
    assert np.linalg.norm(sdk.q - original) > 1e-5
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: not s.client.snapshot()["simulation_running"] and not s.client.snapshot()["pending"])
    assert robot.connection.phase == "connected" and driver.armed and sdk.op_state == 17
    count = s.mpc.solve_count
    until = time.monotonic() + 0.15
    s.wait_for(lambda: time.monotonic() > until)
    assert s.mpc.solve_count == count
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "off")
    assert not driver.armed and sdk.op_state == 5 and "teleop" in sdk.stops
    assert not s.window.node.snapshot()["stream_enabled"]
    assert not s.window.node.snapshot()["deadman_held"]


def test_alignment_and_start_rejections_never_send_motion(robot):
    s, sdk = robot.s, robot.sdk
    valid_q = sdk.q.copy()
    sdk.q[0] = 1000.0
    s.select_mode("keyboard")
    s.wait_for(lambda: s.window.node.snapshot()["robot_q"][0] > 10.0)
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "off" and robot.connection.error)
    assert "alignment rejected" in robot.connection.message.lower()
    assert not sdk.starts and not sdk.moves
    sdk.q = valid_q
    sdk.reject_start = True
    s.wait_for(lambda: s.window.node.snapshot()["robot_q"][0] < 1.0
               and s.panel.connect_button.isEnabled())
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: bool(sdk.starts))
    s.wait_for(lambda: robot.connection.phase == "off" and robot.connection.error)
    assert "injected start rejection" in robot.connection.message
    assert not sdk.moves and not robot.driver.armed


def test_release_holds_measured_joints_and_resume_realigns_a_lagging_robot(robot):
    s, sdk = robot.s, robot.sdk
    connect_keyboard(robot)
    sdk.follow_fraction = 0.05
    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    for _ in range(4):
        QTest.keyClick(s.panel.translation_step, Qt.Key_W)
    s.wait_for(lambda: np.linalg.norm(s.mpc.sim_q - np.radians(sdk.q)) > 0.005)
    old_sim = s.mpc.sim_q.copy()
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    measured_hold = np.asarray(s.window.node.snapshot()["robot_q"])
    s.wait_for(lambda: np.max(np.abs(robot.driver.latest_command - measured_hold)) < 1e-10)
    assert np.linalg.norm(old_sim - measured_hold) > 0.001
    s.wait_for(lambda: not s.client.snapshot()["pending"] and not s.client.snapshot()["simulation_running"])
    assert robot.driver.armed and not s.mpc.execution_enabled
    before_resume = np.radians(sdk.q.copy())
    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    assert np.linalg.norm(s.mpc.sim_q - before_resume) < np.linalg.norm(old_sim - before_resume)
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: not s.client.snapshot()["simulation_running"] and not s.client.snapshot()["pending"])
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "off")


@pytest.mark.parametrize("failure", ["focus", "close", "heartbeat", "feedback"])
def test_connected_session_stops_on_lost_control(robot, failure):
    s = robot.s
    connect_keyboard(robot)
    if failure == "focus":
        QApplication.sendEvent(s.window, QEvent(QEvent.WindowDeactivate))
    elif failure == "close":
        s.window.close()
    elif failure == "feedback":
        robot.sdk.feedback_ok = False
    else:
        s.panel.timer.stop()
        s.window.refresh_timer.stop()
        deadline = time.monotonic() + 1.5
        while robot.driver.armed and time.monotonic() < deadline:
            s.app.processEvents()
            s.executor.spin_once(timeout_sec=0.005)
        assert not robot.driver.armed
        s.panel.timer.start(50)
        s.window.refresh_timer.start(100)
    s.wait_for(lambda: robot.connection.phase == "off")
    assert not robot.driver.armed and robot.sdk.op_state == 5
    assert not robot.sdk.moves
    assert not s.window.node.snapshot()["stream_enabled"]
    assert not s.client.snapshot()["run_held"]


def test_f6_can_cancel_pending_teleop_confirmation(robot):
    s = robot.s
    robot.sdk.confirm_delay = 0.5
    s.select_mode("keyboard")
    s.wait_for(lambda: s.panel.connect_button.isEnabled())
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "arming")
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "off")
    until = time.monotonic() + 0.6
    s.wait_for(lambda: time.monotonic() > until)
    assert not robot.driver.armed and robot.sdk.op_state == 5 and not robot.sdk.moves


def test_failed_stop_is_visible_and_f6_retries(robot):
    s = robot.s
    connect_keyboard(robot)
    robot.sdk.reject_stop = True
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "stop_failed")
    assert "Retry Stop" in s.panel.connect_button.text()
    assert not s.panel.run_button.isEnabled()
    assert not s.window.node.snapshot()["stream_enabled"]
    robot.sdk.reject_stop = False
    QTest.keyClick(s.panel.translation_step, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "off")
    assert robot.sdk.op_state == 5 and not robot.driver.armed


def test_f6_remaps_topic_after_aligning_to_real_joints(robot):
    s = robot.s
    mapper = VivePoseMapper(context=s.context, parameter_overrides=[
        Parameter("input_pose_topic", value="/test_f6/pose")])
    s.executor.add_node(mapper)
    s.nodes.append(mapper)
    broadcaster = TransformBroadcaster(s.peer)
    source = PoseStamped()
    source.header.frame_id, source.pose.orientation.w = "world", 1.0
    source.pose.position.x = 1.0
    publisher = s.peer.create_publisher(PoseStamped, "/test_f6/pose", 10)

    def publish_input():
        source.header.stamp = s.peer.get_clock().now().to_msg()
        publisher.publish(source)
        transform = TransformStamped()
        transform.header.stamp = source.header.stamp
        transform.header.frame_id, transform.child_frame_id = "world", "hand_tcp"
        position, quaternion = pose_from_transform(s.current_hand())
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = map(float, position)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = map(float, quaternion)
        broadcaster.sendTransform(transform)

    s.timers.append(s.peer.create_timer(0.02, publish_input))
    s.wait_for(lambda: s.client.snapshot()["can_sync"])
    QTest.keyClick(s.window.stop_button, Qt.Key_F5)
    s.wait_for(lambda: s.client.snapshot()["can_run_simulation"])
    sequence = mapper._sync_sequence
    QTest.keyClick(s.window.stop_button, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "connected")
    assert mapper._sync_sequence > sequence
    p, q = source.pose.position, source.pose.orientation
    mapped = mapper.world_from_vive @ transform_from_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    np.testing.assert_allclose(mapped, s.current_hand(), atol=1e-8)
    assert robot.driver.armed and not robot.sdk.moves and s.mpc.solve_count == 0
    QTest.keyClick(s.window.stop_button, Qt.Key_F6)
    s.wait_for(lambda: robot.connection.phase == "off")
