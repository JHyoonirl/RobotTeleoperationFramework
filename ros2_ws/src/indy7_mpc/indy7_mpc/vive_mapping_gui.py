"""Reusable Qt mapping panel and standalone Vive mapping window."""

from __future__ import annotations

import math
import signal
import threading

import rclpy
from PyQt5.QtCore import QEvent, QTimer, Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QPushButton, QShortcut, QStackedWidget, QWidget,
)
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from .vive_mapping_client import ViveMappingClient


def _pose_text(label, sample):
    if sample is None:
        return f"{label}: waiting"
    pose, frame, _ = sample
    x, y, z, w = pose[3:]
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    rpy = (
        math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
        math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))),
        math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)),
    )
    xyz = ", ".join(f"{v:+.3f}" for v in pose[:3])
    angles = ", ".join(f"{math.degrees(v):+.1f}" for v in rpy)
    return f"{label} [{frame}]: XYZ [m] ({xyz})   RPY [deg] ({angles})"


class ViveMappingPanel(QGroupBox):
    """Preview, synchronize, and control simulation without direct robot commands."""

    def __init__(self, client, parent=None, shortcut_parent=None):
        super().__init__("Hand Target Teleoperation", parent)
        self.client = client
        self._topic_draft = False
        self._submitted_topic = None
        self._key_window = shortcut_parent or self.window()
        self._f7_down = False
        self._run_source = None
        self.robot_connection = None
        grid = QGridLayout(self)
        source_row = QHBoxLayout()
        source_row.addWidget(QLabel("Input mode"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Keyboard", "keyboard")
        self.mode_combo.addItem("Topic Pose", "topic")
        self.mode_combo.setCurrentIndex(1)
        self.mode_combo.activated.connect(self.select_mode)
        source_row.addWidget(self.mode_combo)
        source_row.addStretch(1)
        source_row.addWidget(QLabel("Controlled frame: hand_tcp  |  Goal: hand_target"))
        grid.addLayout(source_row, 0, 0)
        self.input_pages = QStackedWidget()
        self.topic_page = QWidget()
        topic_grid = QGridLayout(self.topic_page)
        self.input_pages.addWidget(self.topic_page)
        self.keyboard_page = QGroupBox("Keyboard Hand Target")
        keyboard_grid = QGridLayout(self.keyboard_page)
        self.keyboard_current_label = QLabel()
        self.keyboard_target_label = QLabel()
        for row, label in enumerate((self.keyboard_current_label, self.keyboard_target_label)):
            label.setWordWrap(True)
            keyboard_grid.addWidget(label, row, 0, 1, 6)
        keyboard_grid.addWidget(QLabel("Translation step"), 2, 0)
        self.translation_step = QDoubleSpinBox()
        self.translation_step.setRange(1.0, 100.0)
        self.translation_step.setValue(10.0)
        self.translation_step.setSuffix(" mm")
        keyboard_grid.addWidget(self.translation_step, 2, 1)
        keyboard_grid.addWidget(QLabel("Rotation step"), 2, 2)
        self.rotation_step = QDoubleSpinBox()
        self.rotation_step.setRange(0.1, 45.0)
        self.rotation_step.setValue(5.0)
        self.rotation_step.setSuffix(" deg")
        keyboard_grid.addWidget(self.rotation_step, 2, 3)
        for row, labels in ((3, ("X", "Y", "Z")), (4, ("Roll", "Pitch", "Yaw"))):
            for axis, label in enumerate(labels):
                for column, direction in enumerate((-1, 1)):
                    button = QPushButton(f"{label} {'-' if direction < 0 else '+'}")
                    button.clicked.connect(
                        lambda checked=False, a=axis, d=direction, r=row:
                        self.nudge_keyboard(a, d, rotation=r == 4))
                    keyboard_grid.addWidget(button, row, 2 * axis + column)
        self.keyboard_check = QCheckBox("Keyboard: W/S X, A/D Y, R/F Z, U/O roll, I/K pitch, J/L yaw (world axes)")
        self.keyboard_check.setChecked(True)
        keyboard_grid.addWidget(self.keyboard_check, 5, 0, 1, 6)
        self.input_pages.addWidget(self.keyboard_page)
        grid.addWidget(self.input_pages, 1, 0)
        topic_actions = QHBoxLayout()
        topic_actions.addWidget(QLabel("Pose topic"))
        self.topic_combo = QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.setInsertPolicy(QComboBox.NoInsert)
        self.topic_combo.setMinimumContentsLength(24)
        self.topic_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.topic_combo.lineEdit().setPlaceholderText("/tracker/right/pose")
        self.topic_combo.setToolTip("Select or enter an absolute geometry_msgs/msg/PoseStamped topic.")
        self.topic_combo.lineEdit().textEdited.connect(self._topic_edited)
        self.topic_combo.activated.connect(self._topic_edited)
        self.refresh_topics_button = QPushButton("Refresh Topics")
        self.refresh_topics_button.clicked.connect(self.refresh_topics)
        self.apply_topic_button = QPushButton("Apply Topic")
        self.apply_topic_button.clicked.connect(self.apply_topic)
        topic_actions.addWidget(self.topic_combo, 1)
        topic_actions.addWidget(self.refresh_topics_button)
        topic_actions.addWidget(self.apply_topic_button)
        topic_grid.addLayout(topic_actions, 0, 0)
        self.input_label = QLabel()
        self.robot_label = QLabel()
        self.mapping_label = QLabel()
        self.current_label = QLabel()
        self.target_label = QLabel()
        self.result_label = QLabel()
        for row, label in enumerate((self.input_label, self.robot_label, self.mapping_label,
                                     self.current_label, self.target_label), start=1):
            label.setWordWrap(True)
            topic_grid.addWidget(label, row, 0)
        self.selected_target_label = QLabel()
        self.selected_target_label.setWordWrap(True)
        grid.addWidget(self.selected_target_label, 2, 0)
        self.simulation_label = QLabel("Simulation: waiting for MPC runtime")
        self.simulation_label.setWordWrap(True)
        grid.addWidget(self.simulation_label, 3, 0)
        self.connection_label = QLabel("Robot connection unavailable in this GUI session.")
        self.connection_label.setWordWrap(True)
        grid.addWidget(self.connection_label, 4, 0)

        actions = QHBoxLayout()
        self.sync_button = QPushButton("Map / Sync Current Pose (F5)")
        self.sync_button.clicked.connect(self.request_sync)
        self.connect_button = QPushButton("F6 Connect Robot")
        self.run_button = QPushButton("Hold to Run (F7)")
        self.run_button.setFocusPolicy(Qt.NoFocus)
        self.run_button.setAutoRepeat(False)
        self.connect_button.setEnabled(False)
        self.connect_button.setToolTip("Align simulation to robot feedback and enable DCP3 TELE_OP.")
        self.connect_button.clicked.connect(self.toggle_robot_connection)
        self.run_button.setToolTip("Hold this button or F7 to follow hand_target. Release to pause.")
        self.run_button.pressed.connect(lambda: self.start_run("button"))
        self.run_button.released.connect(self.release_button)
        actions.addWidget(self.sync_button)
        actions.addWidget(self.connect_button)
        actions.addWidget(self.run_button)
        grid.addLayout(actions, 5, 0)
        self.result_label.setWordWrap(True)
        grid.addWidget(self.result_label, 6, 0)
        note = QLabel("Edit hand_target with Keyboard or select a Topic Pose input. "
                      "Hold F7 or the Run button to move. Release to pause. "
                      "F6 connects/disconnects robot teleoperation.")
        note.setWordWrap(True)
        grid.addWidget(note, 7, 0)

        # Window scope keeps F5 active after clicking any child input/control.
        self.sync_shortcut = QShortcut(QKeySequence("F5"), shortcut_parent or self)
        self.sync_shortcut.setContext(Qt.WindowShortcut)
        self.sync_shortcut.setAutoRepeat(False)
        self.sync_shortcut.activated.connect(self.request_sync)
        self.connect_shortcut = QShortcut(QKeySequence("F6"), shortcut_parent or self)
        self.connect_shortcut.setContext(Qt.WindowShortcut)
        self.connect_shortcut.setAutoRepeat(False)
        self.connect_shortcut.activated.connect(self.toggle_robot_connection)
        # A shortcut reports activation only. Observe press/release events
        # across child controls so holding F7 never becomes a toggle.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)
        self.refresh_topics()
        self.refresh()

    def attach_robot_connection(self, controller):
        self.robot_connection = controller
        self.refresh()

    def toggle_robot_connection(self):
        if self.robot_connection is None:
            return
        if self.robot_connection.snapshot()["active"]:
            self.release_run()
            self.robot_connection.disconnect()
        else:
            self.robot_connection.connect()
        self.refresh()

    def showEvent(self, event):
        QApplication.instance().installEventFilter(self)
        self.timer.start(50)
        super().showEvent(event)

    def hideEvent(self, event):
        # Detach before child widgets are destroyed. Otherwise a global
        # filter can be called on a half-destroyed window during Qt/GC cleanup.
        if not self.client.is_closed:
            self.release_run()
            if self.robot_connection is not None:
                self.robot_connection.close()
        QApplication.instance().removeEventFilter(self)
        self.timer.stop()
        super().hideEvent(event)

    def _topic_edited(self, *_):
        self._topic_draft = True
        self.refresh()

    def select_mode(self, index):
        self.client.request_mode(self.mode_combo.itemData(index))
        self.refresh()

    def nudge_keyboard(self, axis, direction, rotation=False):
        state = self.client.snapshot()
        if state["input_mode"] != "keyboard" or not state["simulation_fresh"]:
            return
        pose = state["keyboard_pose"]
        if pose is None:
            return
        if rotation:
            half = math.radians(self.rotation_step.value()) * direction / 2
            delta = [0.0, 0.0, 0.0, math.cos(half)]
            delta[axis] = math.sin(half)
            x1, y1, z1, w1 = delta
            x2, y2, z2, w2 = pose[3:]
            pose[3:] = [w1*x2 + x1*w2 + y1*z2 - z1*y2,
                        w1*y2 - x1*z2 + y1*w2 + z1*x2,
                        w1*z2 + x1*y2 - y1*x2 + z1*w2,
                        w1*w2 - x1*x2 - y1*y2 - z1*z2]
        else:
            pose[axis] += direction * self.translation_step.value() / 1000.0
        self.client.set_keyboard_target(pose)
        self.refresh()

    def is_editing_topic(self):
        return (self.topic_combo.hasFocus() or self.topic_combo.lineEdit().hasFocus()
                or self.topic_combo.view().isVisible())

    def refresh_topics(self):
        text = self.topic_combo.currentText()
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        self.topic_combo.addItems(self.client.pose_topics())
        self.topic_combo.setEditText(text)
        self.topic_combo.blockSignals(False)

    def apply_topic(self):
        topic = self.topic_combo.currentText().strip()
        self._topic_draft = True
        if self.client.request_topic(topic):
            self._submitted_topic = topic
        self.refresh()

    def request_sync(self):
        if self.client.snapshot()["input_mode"] == "keyboard":
            self.client.reset_keyboard_target()
            self.refresh()
            return
        if self.topic_combo.currentText().strip() != self.client.snapshot()["status"].get("input_topic"):
            return
        self.client.request_sync()
        self.refresh()

    def start_run(self, source):
        if self.robot_connection is not None and not self.robot_connection.snapshot()["allow_run"]:
            return
        if self._run_source is not None:
            return
        state = self.client.snapshot()
        if (state["input_mode"] == "topic"
                and self.topic_combo.currentText().strip() != state["status"].get("input_topic")):
            return
        if self.robot_connection is not None and self.robot_connection.snapshot()["connected"]:
            started = self.robot_connection.begin_follow()
        else:
            started = self.client.request_run(True)
        if started:
            self._run_source = source
        self.refresh()

    def release_run(self):
        if self.client.is_closed:
            return
        if self.robot_connection is not None:
            self.robot_connection.cancel_follow()
        state = self.client.snapshot()
        held = self._run_source is not None or state["run_held"]
        self._run_source = None
        self._f7_down = False
        self.run_button.setDown(False)
        if held or state["can_stop_simulation"]:
            self.client.request_run(False)
        self.refresh()

    def release_button(self):
        if self._run_source == "button":
            self.release_run()

    def eventFilter(self, watched, event):
        if self.client.is_closed:
            return super().eventFilter(watched, event)
        kind = event.type()
        if (self._run_source == "button" and kind == QEvent.MouseButtonRelease
                and event.button() == Qt.LeftButton):
            self.release_run()
        if kind == QEvent.ApplicationDeactivate or (
                watched in (self._key_window, self)
                and kind in (QEvent.WindowDeactivate, QEvent.Close, QEvent.Hide)):
            self.release_run()
            if self.robot_connection is not None:
                self.robot_connection.focus_lost()
        if (kind in (QEvent.ShortcutOverride, QEvent.KeyPress, QEvent.KeyRelease)
                and isinstance(watched, QWidget)
                and watched.window() is self._key_window.window()
                and event.key() == Qt.Key_F7 and self.isVisible()):
            event.accept()
            if kind == QEvent.ShortcutOverride or event.isAutoRepeat():
                return True
            if kind == QEvent.KeyPress:
                if not self._f7_down:
                    self._f7_down = True
                    self.start_run("key")
            else:
                self._f7_down = False
                if self._run_source == "key":
                    self.release_run()
            return True
        if (kind == QEvent.KeyPress and isinstance(watched, QWidget)
                and watched.window() is self._key_window.window() and self.isVisible()
                and self.client.snapshot()["input_mode"] == "keyboard"
                and self.keyboard_check.isChecked()):
            keys = {Qt.Key_W: (0, 1, False), Qt.Key_S: (0, -1, False),
                    Qt.Key_A: (1, 1, False), Qt.Key_D: (1, -1, False),
                    Qt.Key_R: (2, 1, False), Qt.Key_F: (2, -1, False),
                    Qt.Key_U: (0, 1, True), Qt.Key_O: (0, -1, True),
                    Qt.Key_I: (1, 1, True), Qt.Key_K: (1, -1, True),
                    Qt.Key_J: (2, 1, True), Qt.Key_L: (2, -1, True)}
            if event.key() in keys:
                axis, direction, rotation = keys[event.key()]
                self.nudge_keyboard(axis, direction, rotation)
                return True
        return super().eventFilter(watched, event)

    def refresh(self):
        if self.client.is_closed:
            return
        connection = None
        if self.robot_connection is not None:
            self.robot_connection.tick()
            connection = self.robot_connection.snapshot()
        self.client.publish_keyboard_target()
        self.client.publish_run_hold()
        state = self.client.snapshot()
        follow_pending = self.robot_connection is not None and self.robot_connection.follow_pending
        if self._run_source is not None and not state["run_held"] and not follow_pending:
            self._run_source = None
            self.run_button.setDown(False)
        status = state["status"]
        fresh = state["status_fresh"]
        keyboard = state["input_mode"] == "keyboard"
        if state["operation"] != "mode":
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(self.mode_combo.findData(state["input_mode"]))
            self.mode_combo.blockSignals(False)
        self.mode_combo.setEnabled(state["can_change_mode"] and not (connection and connection["lock_settings"]))
        self.input_pages.setCurrentWidget(self.keyboard_page if keyboard else self.topic_page)
        self.keyboard_page.setEnabled(state["simulation_fresh"])
        self.keyboard_current_label.setText(_pose_text("Current hand_tcp", state["simulation_current"]))
        keyboard_pose = state["keyboard_pose"]
        keyboard_sample = ((keyboard_pose, state["simulation"].get("world_frame", "world"), 0)
                           if keyboard_pose is not None else None)
        self.keyboard_target_label.setText(_pose_text("Keyboard hand_target", keyboard_sample))
        self.selected_target_label.setText(_pose_text("Selected hand_target", state["hand_target"]))
        active_topic = status.get("input_topic", "")
        if self._submitted_topic and not state["pending"]:
            if active_topic == self._submitted_topic:
                self._topic_draft = False
                self._submitted_topic = None
            elif state["result_kind"] == "error":
                self._submitted_topic = None
        if fresh and not self._topic_draft and self.topic_combo.currentText() != active_topic:
            self.topic_combo.blockSignals(True)
            self.topic_combo.setEditText(active_topic)
            self.topic_combo.blockSignals(False)
        edited_topic = self.topic_combo.currentText().strip()
        unapplied_topic = not keyboard and edited_topic != active_topic
        topic_editable = not state["pending"] and not (connection and connection["lock_settings"])
        self.topic_combo.setEnabled(topic_editable)
        self.refresh_topics_button.setEnabled(topic_editable)
        self.apply_topic_button.setEnabled(bool(topic_editable and state["can_apply_topic"] and edited_topic and unapplied_topic))
        self.apply_topic_button.setText("Applying..." if state["operation"] == "topic" else "Apply Topic")
        if not fresh:
            self.input_label.setText("Pose input: waiting for mapper status")
            self.robot_label.setText("Robot hand TCP: waiting for mapper status")
            self.mapping_label.setText("Mapping: mapper unavailable")
        else:
            input_state = "receiving" if status["input_ready"] else status.get("input_error", "waiting")
            robot_state = "ready" if status["robot_ready"] else status.get("robot_error", "waiting")
            self.input_label.setText(f"Pose input: {status.get('input_topic', '?')} | {input_state}")
            self.robot_label.setText(
                f"Robot hand TCP: {status.get('world_frame', '?')} -> "
                f"{status.get('hand_frame', '?')} | {robot_state}")
            mapped = "synchronized" if status["calibrated"] else "not synchronized"
            error = (status.get("sync_error", "") if state["service_ready"]
                     else "mapping service unavailable")
            self.mapping_label.setText(f"Mapping: {mapped}" + (f" | {error}" if error else ""))
        current = state["current"] if fresh and status.get("robot_ready") else None
        target = state["mapped"] if fresh and status.get("calibrated") and status.get("input_ready") else None
        self.current_label.setText(_pose_text("Current hand", current))
        self.target_label.setText(_pose_text("Mapped topic target (vive_hand_target)", target))
        can_reset = (state["simulation_current"] is not None and not state["pending"]
                     and not state["simulation_running"] and not state["run_held"])
        can_sync = can_reset if keyboard else state["can_sync"] and not unapplied_topic
        can_sync = can_sync and not (connection and connection["preparing"])
        self.sync_button.setEnabled(can_sync)
        self.sync_shortcut.setEnabled(bool(can_sync))
        self.sync_button.setText("Reset hand_target from Current Hand (F5)" if keyboard
                                 else "Synchronizing..." if state["operation"] == "sync"
                                 else "Map / Sync Current Pose (F5)")
        simulation = state["simulation"]
        if not state["simulation_fresh"]:
            self.simulation_label.setText("Simulation: waiting for MPC runtime")
        elif state["simulation_running"]:
            hand = simulation.get("hand_frame", "hand_tcp")
            self.simulation_label.setText(
                f"Simulation: {hand} follows hand_target ({state['input_mode']}) | release F7 to pause")
        else:
            mode = "paused" if not simulation.get("execution_enabled") else "manual MPC active"
            detail = simulation.get("reason") or "hold F7 to run"
            self.simulation_label.setText(f"Simulation: {mode} | {detail}")
        can_start = state["can_run_simulation"] and not unapplied_topic
        if connection:
            can_start = can_start and connection["allow_run"]
            self.connection_label.setText(connection["message"])
            self.connection_label.setStyleSheet("color: #b3261e;" if connection["error"] else "")
            self.connect_button.setEnabled(connection["can_toggle"])
            self.connect_shortcut.setEnabled(connection["can_toggle"])
            self.connect_button.setText("F6 Retry Stop" if connection["phase"] == "stop_failed"
                                        else "F6 Disconnect Robot" if connection["active"]
                                        else "F6 Connect Robot")
        else:
            self.connect_shortcut.setEnabled(False)
        can_stop = state["can_stop_simulation"]
        self.run_button.setEnabled(can_start or can_stop or state["run_held"] or follow_pending)
        self.run_button.setText("Release to Pause (F7)" if state["simulation_running"]
                               or state["run_held"] or follow_pending else "Hold to Run (F7)")
        message = state["result"]
        if unapplied_topic and fresh and state["result_kind"] in ("idle", "success"):
            message = ("Release F7 before applying a new topic." if state["simulation_running"]
                       else "Topic edited. Click Apply Topic before pressing F5.")
        self.result_label.setText(message)
        color = {"success": "#18723a", "error": "#b3261e"}.get(state["result_kind"], "")
        self.result_label.setStyleSheet(f"color: {color};" if color else "")


def main(args=None):
    from .robot_connection import RobotConnection
    from .teleop_ros_node import TeleopRosNode

    rclpy.init(args=args)
    node = TeleopRosNode("indy7_vive_mapping_gui")
    client = ViveMappingClient(node)
    app = QApplication([])
    window = QMainWindow()
    window.setWindowTitle("Indy7 Hand Target Teleoperation")
    panel = ViveMappingPanel(client, parent=window, shortcut_parent=window)
    connection = RobotConnection(node, client)
    panel.attach_robot_connection(connection)
    window.setCentralWidget(panel)
    window.resize(1000, 540)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, daemon=True)
    previous_sigint = signal.signal(signal.SIGINT, lambda *_: app.quit())
    window.show()
    thread.start()
    try:
        result = app.exec_()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        connection.close()
        client.close()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)
