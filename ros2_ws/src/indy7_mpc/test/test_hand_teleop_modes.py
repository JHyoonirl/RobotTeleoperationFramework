"""Exercise exclusive hand-target sources through the real minimal Qt GUI/MPC."""

import os
import time
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

pytest.importorskip("crocoddyl")
pin = pytest.importorskip("pinocchio")

from indy7_mpc.crocoddyl_mpc_node import CrocoddylMPCNode
from indy7_mpc.pose_mapping import inverse_transform, pose_from_transform
from indy7_mpc.teleop_gui import TeleopRosNode
from indy7_mpc.teleop_minimal_gui import create_window
from indy7_mpc.vive_pose_mapper import VivePoseMapper

pytestmark = pytest.mark.usefixtures("qapp")


@pytest.fixture
def session(monkeypatch, tmp_path):
    app = QApplication.instance() or QApplication([])
    context = Context()
    rclpy.init(args=[], context=context, domain_id=96)
    executor = SingleThreadedExecutor(context=context)
    monkeypatch.setenv("INDY7_TELEOP_MAPPING", str(tmp_path / "mapping.yaml"))
    share = Path(get_package_share_directory("indy7_mpc"))
    description = Path(get_package_share_directory("indy7_description"))
    config = yaml.safe_load((share / "config/indy7_mpc.yaml").read_text())["crocoddyl_mpc"]["ros__parameters"]
    config.update({"urdf_path": str(description / "urdf/indy7.urdf"),
                   "simulate": True, "internal_simulation": True, "enable_vive_follow": True,
                   "initialize_target_from_current_state": True, "start_paused": True,
                   "diagnostic_logging": False, "dt": 0.02, "horizon": 10})
    mpc = CrocoddylMPCNode(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in config.items()])
    gui_node = TeleopRosNode("test_hand_modes_gui", context=context)
    peer = Node("test_hand_modes_peer", context=context)
    nodes = [mpc, gui_node, peer]
    for node in nodes:
        executor.add_node(node)
    window = create_window(gui_node)
    app.installEventFilter(window)
    window.show()
    window.activateWindow()
    panel, client = window.vive_panel, window.vive_client
    data = mpc.model.createData()
    hand_id = mpc.model.getFrameId("hand_tcp")

    def current_hand():
        pin.framesForwardKinematics(mpc.model, data, mpc.sim_q)
        return data.oMf[hand_id].homogeneous.copy()

    def wait_for(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            app.processEvents()
            executor.spin_once(timeout_sec=0.005)
            panel.refresh()
            window.refresh()
        app.processEvents()
        assert predicate(), client.snapshot()

    def select_mode(mode):
        wait_for(lambda: client.snapshot()["can_change_mode"])
        index = panel.mode_combo.findData(mode)
        panel.mode_combo.setCurrentIndex(index)
        panel.mode_combo.activated[int].emit(index)
        wait_for(lambda: client.snapshot()["input_mode"] == mode and not client.snapshot()["pending"])

    previews, legacy_commands = [], []
    peer.create_subscription(PoseStamped, "/indy7/hand_target_pose", previews.append, 10)
    peer.create_subscription(PoseStamped, "/teleop/target_pose", legacy_commands.append, 10)
    timers = []
    state = SimpleNamespace(app=app, context=context, executor=executor, mpc=mpc, peer=peer,
                            nodes=nodes, window=window, panel=panel, client=client,
                            wait_for=wait_for, select_mode=select_mode, current_hand=current_hand,
                            previews=previews, legacy_commands=legacy_commands, timers=timers)
    try:
        wait_for(lambda: client.snapshot()["simulation_fresh"]
                 and client.snapshot()["simulation_current"] is not None)
        yield state
    finally:
        for timer in timers:
            timer.cancel()
        panel.timer.stop()
        window.refresh_timer.stop()
        app.removeEventFilter(window)
        window.force_close = True
        window.close()
        client.close()
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        context.shutdown()


def test_keyboard_uses_hand_target_without_a_tracker(session):
    s = session
    assert not s.client.snapshot()["status_fresh"]  # No mapper or Tracker exists.
    assert not s.client.snapshot()["can_run_simulation"]
    s.select_mode("keyboard")
    s.wait_for(lambda: s.client.snapshot()["can_run_simulation"] and s.previews)
    assert not s.window.target_box.isVisible()  # No second, competing jog path.
    initial_hand, initial_q = s.current_hand(), s.mpc.sim_q.copy()
    np.testing.assert_allclose(s.client.snapshot()["keyboard_pose"][:3], initial_hand[:3, 3])
    s.panel.translation_step.setFocus()
    QTest.keyClick(s.panel.translation_step, Qt.Key_F5)
    QTest.keyClick(s.panel.translation_step, Qt.Key_W)
    desired = initial_hand[:3, 3] + [0.01, 0, 0]
    s.wait_for(lambda: abs(s.previews[-1].pose.position.x - desired[0]) < 1e-8)
    assert s.mpc.solve_count == 0 and not s.mpc.execution_enabled
    np.testing.assert_allclose(s.mpc.sim_q, initial_q)
    s.app.processEvents()
    assert s.window.centralWidget().horizontalScrollBar().maximum() == 0
    assert s.window.grab().save("/tmp/indy7-hand-keyboard-gui.png")

    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    assert not s.panel.mode_combo.isEnabled()
    # Changing the keyboard hand goal while F7 remains held must reach MPC.
    QTest.keyClick(s.panel.translation_step, Qt.Key_W)
    desired += [0.01, 0, 0]
    s.wait_for(lambda: np.linalg.norm(s.mpc.target_pose.translation - desired) < 1e-8)
    s.wait_for(lambda: np.linalg.norm(s.current_hand()[:3, 3] - desired) < 0.003)
    assert not s.legacy_commands
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: not s.client.snapshot()["simulation_running"] and not s.client.snapshot()["pending"])
    count = s.mpc.solve_count
    until = time.monotonic() + 0.15
    s.wait_for(lambda: time.monotonic() > until)
    assert s.mpc.solve_count == count and not s.mpc.execution_enabled
    s.executor.remove_node(s.mpc)
    s.wait_for(lambda: not s.client.snapshot()["simulation_fresh"])
    assert not s.window.target_box.isVisible()  # Stale status cannot enable the legacy input.
    QTest.keyClick(s.panel.translation_step, Qt.Key_W)
    assert not s.legacy_commands


def test_topic_and_keyboard_inputs_cannot_override_each_other(session):
    s = session
    mapper = VivePoseMapper(context=s.context, parameter_overrides=[
        Parameter("input_pose_topic", value="/test_hand_modes/pose")])
    s.executor.add_node(mapper)
    s.nodes.append(mapper)
    broadcaster = TransformBroadcaster(s.peer)

    def publish_tf():
        transform = TransformStamped()
        transform.header.stamp = s.peer.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = "world", "hand_tcp"
        position, quaternion = pose_from_transform(s.current_hand())
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = map(float, position)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = map(float, quaternion)
        broadcaster.sendTransform(transform)

    s.timers.append(s.peer.create_timer(0.02, publish_tf))
    source = PoseStamped()
    source.header.frame_id = "world"
    source.pose.position.x, source.pose.orientation.w = 1.0, 1.0
    source_pub = s.peer.create_publisher(PoseStamped, "/test_hand_modes/pose", 10)

    def publish_source():
        source.header.stamp = s.peer.get_clock().now().to_msg()
        source_pub.publish(source)

    source_timer = s.peer.create_timer(0.02, publish_source)
    s.timers.append(source_timer)
    s.wait_for(lambda: s.client.snapshot()["can_sync"])
    QTest.keyClick(s.window.stop_button, Qt.Key_F5)
    s.wait_for(lambda: s.client.snapshot()["can_run_simulation"])
    s.app.processEvents()
    assert s.window.centralWidget().horizontalScrollBar().maximum() == 0
    assert s.window.grab().save("/tmp/indy7-hand-topic-gui.png")
    initial = s.current_hand()

    # Keyboard must stay usable even if a mapped topic stops or changes.
    s.select_mode("keyboard")
    s.wait_for(lambda: s.client.snapshot()["can_run_simulation"])
    QTest.keyPress(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    source_timer.cancel()
    s.wait_for(lambda: not s.client.snapshot()["status"].get("input_ready"))
    assert s.mpc.vive_control.following
    mapper.set_parameters_atomically([Parameter("input_pose_topic", value="/test_hand_modes/other")])
    s.wait_for(lambda: s.client.snapshot()["status"].get("input_revision") == 1)
    assert s.mpc.vive_control.following
    QTest.keyRelease(s.panel.translation_step, Qt.Key_F7)
    s.wait_for(lambda: not s.client.snapshot()["simulation_running"] and not s.client.snapshot()["pending"])
    mapper.set_parameters_atomically([Parameter("input_pose_topic", value="/test_hand_modes/pose")])
    source_timer.reset()
    s.select_mode("topic")
    s.wait_for(lambda: s.client.snapshot()["can_sync"])
    QTest.keyClick(s.window.stop_button, Qt.Key_F5)
    s.wait_for(lambda: s.client.snapshot()["can_run_simulation"])

    unwanted = PoseStamped()
    unwanted.header.frame_id, unwanted.pose.orientation.w = "world", 1.0
    unwanted.pose.position.x = 2.0
    keyboard_pub = s.peer.create_publisher(PoseStamped, "/indy7/keyboard/hand_target_pose", 10)

    def publish_unselected():
        unwanted.header.stamp = s.peer.get_clock().now().to_msg()
        keyboard_pub.publish(unwanted)

    competing_timer = s.peer.create_timer(0.02, publish_unselected)
    s.timers.append(competing_timer)
    QTest.keyPress(s.window.stop_button, Qt.Key_F7)
    s.wait_for(lambda: s.client.snapshot()["simulation_running"])
    desired = initial.copy()
    desired[:3, 3] += [0.015, 0, 0]
    position, quaternion = pose_from_transform(inverse_transform(mapper.world_from_vive) @ desired)
    (source.pose.position.x, source.pose.position.y, source.pose.position.z) = map(float, position)
    (source.pose.orientation.x, source.pose.orientation.y,
     source.pose.orientation.z, source.pose.orientation.w) = map(float, quaternion)
    s.wait_for(lambda: np.max(np.abs(s.mpc.target_pose.homogeneous - desired)) < 1e-8)
    QTest.keyClick(s.window.stop_button, Qt.Key_W)
    s.wait_for(lambda: np.linalg.norm(s.current_hand()[:3, 3] - desired[:3, 3]) < 0.003)
    np.testing.assert_allclose(s.mpc.target_pose.homogeneous, desired, atol=1e-8)
    assert not s.legacy_commands
    competing_timer.cancel()

    # A mode service request from another client must stop the active source.
    changed = s.client.mode_service.call_async(SetBool.Request(data=True))
    s.wait_for(changed.done)
    assert changed.result().success
    s.wait_for(lambda: not s.client.snapshot()["simulation_running"])
    assert not s.mpc.execution_enabled and s.mpc.vive_control.input_mode == "keyboard"
    count = s.mpc.solve_count
    until = time.monotonic() + 0.2
    s.wait_for(lambda: time.monotonic() > until)
    assert s.mpc.solve_count == count  # Still physically holding the old F7 press.
    QTest.keyRelease(s.window.stop_button, Qt.Key_F7)
