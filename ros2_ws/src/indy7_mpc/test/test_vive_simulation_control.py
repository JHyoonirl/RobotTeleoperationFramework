"""Run F5/F7 against the actual MPC and an internal plant in ROS domain 97."""

import os
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QLineEdit, QMainWindow, QVBoxLayout, QWidget
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

pytest.importorskip("crocoddyl")
pin = pytest.importorskip("pinocchio")

from indy7_mpc.crocoddyl_mpc_node import CrocoddylMPCNode
from indy7_mpc.pose_mapping import inverse_transform, pose_from_transform, transform_from_rpy
from indy7_mpc.vive_mapping_client import ViveMappingClient
from indy7_mpc.vive_mapping_gui import ViveMappingPanel
from indy7_mpc.vive_pose_mapper import VivePoseMapper

pytestmark = pytest.mark.usefixtures("qapp")


def test_manual_mpc_keeps_original_tcp_frame():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=97)
    description = Path(get_package_share_directory("indy7_description"))
    node = None
    try:
        config = {
            "urdf_path": str(description / "urdf/indy7.urdf"),
            "simulate": True, "internal_simulation": True,
            "enable_vive_follow": False, "start_paused": True,
            "diagnostic_logging": False, "horizon": 2,
        }
        node = CrocoddylMPCNode(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in config.items()])
        assert node.task_frame == node.get_parameter("task_frame").value == "tcp"
        assert node.vive_control is None
        tcp_id = node.model.getFrameId("tcp")
        assert all(action.frame_id == tcp_id for action in node.running_actions)
        assert node.terminal_action.frame_id == tcp_id
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_f7_runs_mpc_without_robot_and_stops_on_input_loss():
    app = QApplication.instance() or QApplication([])
    context = Context()
    rclpy.init(args=[], context=context, domain_id=97)
    executor = SingleThreadedExecutor(context=context)
    description = Path(get_package_share_directory("indy7_description"))
    share = Path(get_package_share_directory("indy7_mpc"))
    config = yaml.safe_load((share / "config/indy7_mpc.yaml").read_text())["crocoddyl_mpc"]["ros__parameters"]
    config.update({"urdf_path": str(description / "urdf/indy7.urdf"),
                   "simulate": True, "internal_simulation": True, "enable_vive_follow": True,
                   "start_paused": True, "initialize_target_from_current_state": True,
                   "diagnostic_logging": False, "dt": 0.02, "horizon": 10})
    mpc = CrocoddylMPCNode(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in config.items()])
    mapper = VivePoseMapper(context=context)
    peer = Node("test_vive_simulation_peer", context=context)
    client = ViveMappingClient(peer)
    for node in (mpc, mapper, peer):
        executor.add_node(node)
    commands = []
    peer.create_subscription(Float64MultiArray, "mpc/joint_position_cmd", commands.append, 10)
    broadcaster = TransformBroadcaster(peer)
    data = mpc.model.createData()
    hand_id = mpc.model.getFrameId("hand_tcp")

    def current_hand():
        pin.framesForwardKinematics(mpc.model, data, mpc.sim_q)
        return data.oMf[hand_id].homogeneous.copy()

    def publish_tf():
        tf = TransformStamped()
        tf.header.stamp = peer.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = "world", "hand_tcp"
        position, quaternion = pose_from_transform(current_hand())
        (tf.transform.translation.x, tf.transform.translation.y,
         tf.transform.translation.z) = map(float, position)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = map(float, quaternion)
        broadcaster.sendTransform(tf)

    tf_timer = peer.create_timer(0.02, publish_tf)
    source = PoseStamped()
    source.header.frame_id = "world"
    source.pose.position.x = 1.0
    source.pose.orientation.w = 1.0
    publisher = peer.create_publisher(PoseStamped, "/tracker/right/pose", 10)

    def publish_vive():
        source.header.stamp = peer.get_clock().now().to_msg()
        publisher.publish(source)

    input_timer = peer.create_timer(0.02, publish_vive)
    window = QMainWindow()
    root = QWidget()
    layout = QVBoxLayout(root)
    focus = QLineEdit("F7 works after clicking another control")
    panel = ViveMappingPanel(client, parent=root, shortcut_parent=window)
    layout.addWidget(focus)
    layout.addWidget(panel)
    window.setCentralWidget(root)
    window.show()
    window.activateWindow()

    def wait_for(predicate, timeout=8, refresh_panel=True):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            app.processEvents()
            executor.spin_once(timeout_sec=0.005)
            if refresh_panel:
                panel.refresh()
        app.processEvents()
        assert predicate(), client.snapshot()

    try:
        assert mpc.task_frame == "hand_tcp"
        assert mpc.get_parameter("task_frame").value == "hand_tcp"
        assert all(action.frame_id == hand_id for action in mpc.running_actions)
        assert mpc.terminal_action.frame_id == hand_id
        wait_for(lambda: client.snapshot()["simulation_fresh"] and client.snapshot()["can_sync"])
        assert not mpc.execution_enabled
        assert mpc.solve_count == 0 and not commands
        rejected = client.run_service.call_async(SetBool.Request(data=True))
        wait_for(rejected.done)
        assert not rejected.result().success

        focus.setFocus()
        app.processEvents()
        QTest.keyClick(focus, Qt.Key_F5)
        wait_for(lambda: client.snapshot()["can_run_simulation"])
        rejected = client.run_service.call_async(SetBool.Request(data=True))
        wait_for(rejected.done)
        assert not rejected.result().success and "Hold F7" in rejected.result().message
        initial_hand = current_hand()
        initial_q = mpc.sim_q.copy()
        QTest.keyPress(focus, Qt.Key_F7)
        wait_for(lambda: client.snapshot()["simulation_running"])
        assert not panel.sync_button.isEnabled()
        np.testing.assert_allclose(mpc.target_pose.homogeneous, initial_hand, atol=1e-9)
        assert "hand_tcp follows hand_target (topic)" in panel.simulation_label.text()
        # OS key repeat includes synthetic releases; these must not stop or
        # toggle a held F7, including when a child editor has focus.
        for kind in (QEvent.KeyRelease, QEvent.KeyPress):
            QApplication.sendEvent(focus, QKeyEvent(kind, Qt.Key_F7, Qt.NoModifier, "", True))
        assert client.snapshot()["run_held"]
        assert mpc.vive_control.following

        desired_hand = initial_hand.copy()
        desired_hand[:3, 3] += [0.02, 0.0, 0.0]
        desired_hand[:3, :3] = transform_from_rpy([0, 0, 0], [0, 0, 1.0])[:3, :3] @ initial_hand[:3, :3]
        def set_hand_target(target):
            vive_pose = inverse_transform(mapper.world_from_vive) @ target
            position, quaternion = pose_from_transform(vive_pose)
            (source.pose.position.x, source.pose.position.y, source.pose.position.z) = map(float, position)
            (source.pose.orientation.x, source.pose.orientation.y,
             source.pose.orientation.z, source.pose.orientation.w) = map(float, quaternion)

        def hand_reached_target():
            hand = current_hand()
            return (np.linalg.norm(hand[:3, 3] - desired_hand[:3, 3]) < 0.003
                    and np.linalg.norm(pin.log3(hand[:3, :3].T @ desired_hand[:3, :3])) < 0.02)

        set_hand_target(desired_hand)
        wait_for(lambda: np.max(np.abs(mpc.target_pose.homogeneous - desired_hand)) < 1e-8)
        mpc._on_target_pose(source)  # Manual input cannot replace a running Vive target.
        np.testing.assert_allclose(mpc.target_pose.homogeneous, desired_hand, atol=1e-8)
        wait_for(hand_reached_target)
        assert mpc.solve_count > 0 and commands
        assert np.linalg.norm(mpc.sim_q - initial_q) > 1e-3
        assert np.linalg.norm(pin.log3(current_hand()[:3, :3].T @ desired_hand[:3, :3])) < 0.02
        assert not peer.get_publishers_info_by_topic("/indy7_dcp3/joint_position_cmd")

        # Rotate about a stationary hand origin. The upstream TCP must move
        # to compensate for its offset, while the controlled hand stays put.
        pin.framesForwardKinematics(mpc.model, data, mpc.sim_q)
        tcp_id = mpc.model.getFrameId("tcp")
        tcp_before_rotation = data.oMf[tcp_id].translation.copy()
        desired_hand[:3, :3] = transform_from_rpy([0, 0, 0], [0, 0, 6.0])[:3, :3] @ initial_hand[:3, :3]
        set_hand_target(desired_hand)
        wait_for(lambda: np.max(np.abs(mpc.target_pose.homogeneous - desired_hand)) < 1e-8)
        wait_for(hand_reached_target)
        pin.framesForwardKinematics(mpc.model, data, mpc.sim_q)
        assert np.linalg.norm(data.oMf[tcp_id].translation - tcp_before_rotation) > 0.002

        editor = panel.topic_combo.lineEdit()
        editor.setFocus()
        app.processEvents()
        QTest.keyRelease(editor, Qt.Key_F7)
        wait_for(lambda: not client.snapshot()["simulation_running"] and not client.snapshot()["pending"])
        count = mpc.solve_count
        until = time.monotonic() + 0.2
        wait_for(lambda: time.monotonic() > until)
        assert mpc.solve_count == count and not mpc.execution_enabled

        # A quick tap/release before the ROS executor handles start cannot
        # leave execution latched on when the delayed messages arrive.
        QTest.keyClick(editor, Qt.Key_F7)
        wait_for(lambda: not client.snapshot()["simulation_running"] and not client.snapshot()["pending"])
        until = time.monotonic() + 0.2
        wait_for(lambda: time.monotonic() > until)
        assert not mpc.vive_control.following and not mpc.execution_enabled

        QTest.keyPress(editor, Qt.Key_F7)
        wait_for(lambda: client.snapshot()["simulation_running"])
        QApplication.sendEvent(window, QEvent(QEvent.WindowDeactivate))
        wait_for(lambda: not client.snapshot()["simulation_running"] and not client.snapshot()["pending"])
        assert not mpc.execution_enabled
        QTest.keyRelease(editor, Qt.Key_F7)

        QTest.mousePress(panel.run_button, Qt.LeftButton)
        wait_for(lambda: client.snapshot()["simulation_running"])
        QTest.mouseRelease(panel.run_button, Qt.LeftButton)
        wait_for(lambda: not client.snapshot()["simulation_running"] and not client.snapshot()["pending"])
        assert not mpc.execution_enabled

        # Heartbeats come from the Qt loop, not the still-running ROS/Vive
        # publishers. Losing the GUI loop must pause within the hold timeout.
        QTest.keyPress(editor, Qt.Key_F7)
        wait_for(lambda: client.snapshot()["simulation_running"])
        panel.timer.stop()
        wait_for(lambda: not mpc.vive_control.following, timeout=2, refresh_panel=False)
        assert "hold input timed out" in mpc.vive_control.message
        assert not mpc.execution_enabled
        panel.timer.start(50)
        wait_for(lambda: not client.snapshot()["run_held"])
        assert not mpc.vive_control.following
        QTest.keyRelease(editor, Qt.Key_F7)
        wait_for(lambda: not client.snapshot()["pending"])

        QTest.keyPress(editor, Qt.Key_F7)
        wait_for(lambda: client.snapshot()["simulation_running"])
        input_timer.cancel()
        wait_for(lambda: not mpc.vive_control.following, timeout=2)
        assert not mpc.execution_enabled
        assert "stale" in mpc.vive_control.message or "stopped" in mpc.vive_control.message

        input_timer.reset()
        wait_for(lambda: client.snapshot()["can_run_simulation"])
        assert not mpc.vive_control.following  # Recovery while held cannot restart motion.
        QTest.keyRelease(editor, Qt.Key_F7)
        wait_for(lambda: not client.snapshot()["pending"])
        QTest.keyPress(editor, Qt.Key_F7)
        wait_for(lambda: client.snapshot()["simulation_running"])
        mapper.set_parameters_atomically([Parameter("input_pose_topic", value="/another_tracker/pose")])
        wait_for(lambda: not mpc.vive_control.following)
        assert not mpc.execution_enabled
        assert "Mapping changed" in mpc.vive_control.message
    finally:
        input_timer.cancel()
        tf_timer.cancel()
        panel.timer.stop()
        window.close()
        client.close()
        executor.shutdown()
        for node in (mpc, mapper, peer):
            node.destroy_node()
        context.shutdown()
