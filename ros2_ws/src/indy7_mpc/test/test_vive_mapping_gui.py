"""Test F5 and mapping readiness with Qt and real ROS services in domain 90."""

import os
import time
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QLineEdit, QMainWindow, QVBoxLayout, QWidget
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import String

from indy7_mpc.vive_mapping_client import ViveMappingClient
from indy7_mpc.vive_mapping_gui import ViveMappingPanel
from indy7_mpc.vive_pose_mapper import VivePoseMapper

pytestmark = pytest.mark.usefixtures("qapp")


@pytest.fixture
def gui_session():
    app = QApplication.instance() or QApplication([])
    context = Context()
    rclpy.init(args=[], context=context, domain_id=90)
    executor = SingleThreadedExecutor(context=context)
    mapper_parameters = {
        "input_pose_topic": "/test_mapping_gui/input",
        "mapped_pose_topic": "/test_mapping_gui/target",
        "current_hand_pose_topic": "/test_mapping_gui/current",
        "status_topic": "/test_mapping_gui/status",
        "sync_service": "/test_mapping_gui/sync",
        "hand_frame": "gui_test_hand",
        "vive_world_frame": "gui_test_vive_world",
        "mapped_hand_frame": "gui_test_target",
        "input_timeout": 0.25,
    }
    mappers = []

    def new_mapper():
        mapper = VivePoseMapper(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in mapper_parameters.items()
        ])
        executor.add_node(mapper)
        mappers.append(mapper)
        return mapper

    mapper = new_mapper()
    peer = Node("test_mapping_gui_client", context=context, parameter_overrides=[
        Parameter("vive_mapping_status_topic", value="/test_mapping_gui/status"),
        Parameter("vive_mapping_sync_service", value="/test_mapping_gui/sync"),
        Parameter("vive_mapping_current_pose_topic", value="/test_mapping_gui/current"),
        Parameter("vive_mapping_target_pose_topic", value="/test_mapping_gui/target"),
    ])
    executor.add_node(peer)
    client = ViveMappingClient(peer, request_timeout=0.3)
    window = QMainWindow()
    root = QWidget()
    layout = QVBoxLayout(root)
    other_input = QLineEdit("Another focused GUI control")
    panel = ViveMappingPanel(client, parent=root, shortcut_parent=window)
    layout.addWidget(other_input)
    layout.addWidget(panel)
    window.setCentralWidget(root)
    window.resize(1000, 400)
    window.show()
    window.activateWindow()

    def wait_for(predicate, timeout=5):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            app.processEvents()
            executor.spin_once(timeout_sec=0.01)
            panel.refresh()
        app.processEvents()
        assert predicate()

    broadcaster = StaticTransformBroadcaster(peer)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "world", "gui_test_hand"
    tf.transform.translation.x, tf.transform.translation.z = 0.5, 0.6
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    publisher = peer.create_publisher(PoseStamped, "/test_mapping_gui/input", 10)
    message = PoseStamped()
    message.header.frame_id = "world"
    message.pose.position.x = 2.0
    message.pose.orientation.w = 1.0

    def publish():
        message.header.stamp = peer.get_clock().now().to_msg()
        publisher.publish(message)

    timer = peer.create_timer(0.02, publish)
    timer.cancel()
    session = SimpleNamespace(app=app, context=context, executor=executor, mapper=mapper,
                              client=client, window=window, panel=panel, other_input=other_input,
                              wait_for=wait_for, message=message, timer=timer, new_mapper=new_mapper)
    try:
        yield session
    finally:
        panel.timer.stop()
        window.close()
        client.close()
        executor.shutdown()
        for item in mappers:
            item.destroy_node()
        peer.destroy_node()
        context.shutdown()


def test_f5_from_child_control_and_stale_input(gui_session):
    s = gui_session
    s.wait_for(lambda: s.client.snapshot()["status_fresh"])
    assert not s.client.snapshot()["can_sync"]
    assert not s.panel.sync_button.isEnabled()
    assert not s.client.request_sync()
    assert s.mapper.world_from_vive is None

    s.timer.reset()
    s.wait_for(lambda: s.client.snapshot()["can_sync"])
    s.panel.refresh()
    s.other_input.setFocus()
    s.app.processEvents()
    assert s.other_input.hasFocus()
    QTest.keyClick(s.other_input, Qt.Key_F5)
    assert s.client.snapshot()["pending"]
    assert not s.client.request_sync()  # Repeated F5 cannot queue another sync.
    s.wait_for(lambda: s.client.snapshot()["result_kind"] == "success")
    s.wait_for(lambda: s.client.snapshot()["status"].get("calibrated")
               and s.client.snapshot()["mapped"] is not None)
    assert s.mapper._sync_sequence == 1
    np.testing.assert_allclose(s.client.snapshot()["mapped"][0][:3], [0.5, 0, 0.6])
    assert not s.panel.connect_button.isEnabled()
    assert not s.panel.run_button.isEnabled()

    s.message.pose.position.x = 2.1
    s.wait_for(lambda: abs(s.client.snapshot()["mapped"][0][0] - 0.6) < 1e-8)
    s.timer.cancel()
    s.wait_for(lambda: not s.client.snapshot()["status"].get("input_ready"))
    s.panel.refresh()
    assert not s.panel.sync_button.isEnabled()
    assert s.panel.target_label.text() == "Mapped topic target (vive_hand_target): waiting"
    previous = s.mapper.world_from_vive.copy()
    QTest.keyClick(s.other_input, Qt.Key_F5)
    assert not s.client.snapshot()["pending"]
    assert s.mapper._sync_sequence == 1
    np.testing.assert_array_equal(s.mapper.world_from_vive, previous)


def test_request_timeout_and_mapper_restart(gui_session):
    s = gui_session
    s.timer.reset()
    s.wait_for(lambda: s.client.snapshot()["can_sync"])
    # Simulate a lost service response without blocking Qt or its executor.
    s.executor.remove_node(s.mapper)
    assert s.client.request_sync()
    s.wait_for(lambda: not s.client.snapshot()["pending"])
    assert "timed out" in s.client.snapshot()["result"]
    s.wait_for(lambda: not s.client.snapshot()["status_fresh"])
    assert not s.client.snapshot()["can_sync"]
    s.panel.refresh()
    assert "unavailable" in s.panel.mapping_label.text()

    previous_session = s.client.snapshot()["status"]["session_id"]
    s.new_mapper()
    s.wait_for(lambda: s.client.snapshot()["status"].get("session_id") != previous_session)
    assert not s.client.snapshot()["status"]["calibrated"]
    assert "restarted" in s.client.snapshot()["result"]
    assert s.client.snapshot()["result_kind"] != "success"


def test_server_rejection_is_shown_without_changing_mapping(gui_session, monkeypatch):
    s = gui_session
    s.timer.reset()
    s.wait_for(lambda: s.client.snapshot()["can_sync"])
    assert s.client.request_sync()

    # The TF can disappear between the ready indication and service handling.
    def missing_tf():
        raise ValueError("Robot hand TCP TF became unavailable")

    monkeypatch.setattr(s.mapper, "_current_hand", missing_tf)
    s.wait_for(lambda: s.client.snapshot()["result_kind"] == "error")
    assert "Synchronization rejected" in s.panel.result_label.text()
    assert "TF became unavailable" in s.panel.result_label.text()
    assert s.mapper.world_from_vive is None


def test_gui_topic_switch_requires_new_sync_and_ignores_old_source(gui_session):
    s = gui_session
    s.timer.reset()
    s.wait_for(lambda: s.client.snapshot()["can_sync"] and s.client.snapshot()["can_apply_topic"])
    assert s.client.request_sync()
    s.wait_for(lambda: s.client.snapshot()["status"].get("calibrated"))
    s.wait_for(lambda: not s.client.snapshot()["pending"])
    old_callback = s.mapper._input_sub.callback

    topic = "/test_mapping_gui/another_tracker/pose"
    publisher = s.client.node.create_publisher(PoseStamped, topic, 10)
    s.wait_for(lambda: topic in s.client.pose_topics())
    s.panel.refresh_topics_button.click()
    assert s.panel.topic_combo.findText(topic) >= 0
    editor = s.panel.topic_combo.lineEdit()
    editor.setFocus()
    editor.selectAll()
    QTest.keyClicks(editor, topic)
    assert s.panel.topic_combo.currentText() == topic
    assert not s.panel.sync_button.isEnabled()
    sequence = s.mapper._sync_sequence
    QTest.keyClick(editor, Qt.Key_F5)
    assert s.mapper._sync_sequence == sequence  # An unapplied edit cannot sync the old input.
    s.panel.apply_topic_button.click()
    s.wait_for(lambda: not s.client.snapshot()["pending"]
               and s.client.snapshot()["status"].get("input_topic") == topic)
    assert s.mapper.get_parameter("input_pose_topic").value == topic
    assert s.mapper.world_from_vive is None
    assert s.client.snapshot()["mapped"] is None
    assert not s.client.snapshot()["can_sync"]
    assert s.panel.target_label.text() == "Mapped topic target (vive_hand_target): waiting"
    # Reproduce a callback already queued by the old subscription at switch time.
    old_callback(s.message)
    assert s.mapper._latest_input is None

    message = PoseStamped()
    message.header.frame_id = "world"
    message.pose.position.x = 8.0
    message.pose.orientation.w = 1.0

    def publish():
        message.header.stamp = s.client.node.get_clock().now().to_msg()
        publisher.publish(message)

    timer = s.client.node.create_timer(0.02, publish)
    try:
        s.wait_for(lambda: s.client.snapshot()["can_sync"])
        QTest.keyClick(editor, Qt.Key_F5)
        s.wait_for(lambda: s.client.snapshot()["result_kind"] == "success"
                   and s.client.snapshot()["mapped"] is not None)
        np.testing.assert_allclose(s.client.snapshot()["mapped"][0][:3], [0.5, 0, 0.6])
        message.pose.position.x = 8.2
        s.wait_for(lambda: abs(s.client.snapshot()["mapped"][0][0] - 0.7) < 1e-8)
        assert s.mapper._sync_sequence == sequence + 1
    finally:
        timer.cancel()


def test_invalid_topic_changes_preserve_active_mapping(gui_session):
    s = gui_session
    s.timer.reset()
    s.wait_for(lambda: s.client.snapshot()["can_sync"] and s.client.snapshot()["can_apply_topic"])
    assert s.client.request_sync()
    s.wait_for(lambda: s.client.snapshot()["status"].get("calibrated"))
    s.wait_for(lambda: not s.client.snapshot()["pending"])
    matrix = s.mapper.world_from_vive.copy()
    wrong_topic = "/test_mapping_gui/not_a_pose"
    s.client.node.create_publisher(String, wrong_topic, 10)
    s.wait_for(lambda: wrong_topic in dict(s.mapper.get_topic_names_and_types()))
    for topic in ("", "/bad topic", wrong_topic,
                  "/test_mapping_gui/target", "/test_mapping_gui/current"):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter("input_pose_topic", value=topic).to_parameter_msg()]
        future = s.client.topic_service.call_async(request)
        s.wait_for(future.done)
        assert not future.result().result.successful
        assert s.mapper.input_topic == "/test_mapping_gui/input"
        assert s.mapper._input_revision == 0
        np.testing.assert_array_equal(s.mapper.world_from_vive, matrix)


def test_typing_topic_does_not_jog_simulation(monkeypatch, tmp_path):
    from indy7_mpc.teleop_gui import TeleopRosNode
    from indy7_mpc.teleop_minimal_gui import create_window

    app = QApplication.instance() or QApplication([])
    monkeypatch.setenv("INDY7_TELEOP_MAPPING", str(tmp_path / "joint_mapping.yaml"))
    rclpy.init(args=[], domain_id=94)
    node = TeleopRosNode("test_topic_edit_teleop")
    node.state["desired_pose"] = [0.5, 0.0, 0.6, 0.0, 0.0, 0.0, 1.0]
    node.state["desired_time"] = time.monotonic()
    window = create_window(node)
    app.installEventFilter(window)
    window.show()
    window.activateWindow()
    try:
        editor = window.vive_panel.topic_combo.lineEdit()
        editor.setFocus()
        app.processEvents()
        assert editor.hasFocus()
        topic = "/vive/tracker/right/pose"
        editor.selectAll()
        QTest.keyClicks(editor, topic)
        assert editor.text() == topic
        assert window.target_pose_override is None
        # Keyboard jogging still works after focus leaves the topic editor.
        window.translation_step.setFocus()
        app.processEvents()
        QTest.keyClick(window.translation_step, Qt.Key_W)
        assert window.target_pose_override[0] == pytest.approx(0.51)
    finally:
        app.removeEventFilter(window)
        window.close()
        window.vive_client.close()
        node.destroy_node()
        rclpy.shutdown()
