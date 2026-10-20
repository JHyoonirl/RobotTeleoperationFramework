#!/usr/bin/env python3
"""Minimal GUI for simulation TCP teleoperation with mapped DCP3 output."""

from __future__ import annotations

import math
import threading
import time

import rclpy

from .joint_mapping import map_sim_to_robot, validate_mapping
from .teleop_gui import ROBOT_JOINTS, SIM_JOINTS, TeleopRosNode


def create_window(node):
    try:
        from PyQt5.QtCore import QEvent, QTimer, Qt
        from PyQt5.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QDoubleSpinBox,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QScrollArea,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as exc:
        raise RuntimeError("The teleoperation GUI requires python3-pyqt5.") from exc

    from .vive_mapping_client import ViveMappingClient
    from .vive_mapping_gui import ViveMappingPanel
    from .robot_connection import RobotConnection

    class MinimalTeleopWindow(QMainWindow):
        def __init__(self, node):
            super().__init__()
            self.node = node
            self.target_pose_override = None
            self.target_pose_override_time = 0.0
            self._uses_hand_controller = False
            self.pending_close = False
            self.pending_close_since = 0.0
            self.force_close = False
            self.setWindowTitle("Indy7 Simulation Teleop")
            self.resize(1040, 920)

            root = QWidget()
            layout = QVBoxLayout(root)

            status = QGroupBox("Status")
            status_grid = QGridLayout(status)
            self.sim_status = QLabel("Simulation: waiting")
            self.robot_status = QLabel("Robot: waiting")
            self.dcp3_status = QLabel("DCP3: waiting")
            self.pose_status = QLabel("Current / desired TCP: waiting")
            self.sync_status = QLabel("Mapping and alignment: ready")
            for label in (self.sim_status, self.robot_status, self.dcp3_status,
                          self.pose_status, self.sync_status):
                label.setWordWrap(True)
            status_grid.addWidget(self.sim_status, 0, 0)
            status_grid.addWidget(self.robot_status, 0, 1)
            status_grid.addWidget(self.dcp3_status, 0, 2)
            status_grid.addWidget(self.pose_status, 1, 0, 1, 2)
            status_grid.addWidget(self.sync_status, 1, 2)
            layout.addWidget(status)

            self.vive_client = ViveMappingClient(node)
            self.vive_panel = ViveMappingPanel(self.vive_client, parent=self, shortcut_parent=self)
            layout.addWidget(self.vive_panel)

            mapping = QGroupBox("Joint Mapping  |  q_robot = direction × q_sim + offset")
            mapping_layout = QVBoxLayout(mapping)
            self.table = QTableWidget(6, 4)
            self.table.setHorizontalHeaderLabels(
                ["Robot Joint", "Simulation Joint", "Direction", "Offset [deg]"]
            )
            self.table.verticalHeader().setVisible(False)
            self.table.setMinimumHeight(220)
            self.table.setMaximumHeight(220)
            for row, robot_joint in enumerate(ROBOT_JOINTS):
                joint_item = QTableWidgetItem(robot_joint)
                joint_item.setFlags(joint_item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, 0, joint_item)
                sim_combo = QComboBox()
                sim_combo.addItems(SIM_JOINTS)
                sign_combo = QComboBox()
                sign_combo.addItems(["+1", "-1"])
                offset = QDoubleSpinBox()
                offset.setRange(-720.0, 720.0)
                offset.setDecimals(3)
                offset.setSuffix("°")
                row_config = node.mapping[robot_joint]
                sim_combo.setCurrentText(row_config["simulation_joint"])
                sign_combo.setCurrentText(f"{row_config['direction']:+d}")
                offset.setValue(float(row_config["offset_deg"]))
                self.table.setCellWidget(row, 1, sim_combo)
                self.table.setCellWidget(row, 2, sign_combo)
                self.table.setCellWidget(row, 3, offset)
            self.table.resizeColumnsToContents()
            mapping_layout.addWidget(self.table)

            mapping_actions = QHBoxLayout()
            self.save_mapping_button = QPushButton("Save Mapping")
            self.save_mapping_button.clicked.connect(self.save_mapping)
            self.sync_button = QPushButton("Sync Robot Pose to Simulation")
            self.sync_button.clicked.connect(self.sync_robot_to_sim)
            self.max_start_error = QDoubleSpinBox()
            self.max_start_error.setRange(0.5, 180.0)
            self.max_start_error.setValue(10.0)
            self.max_start_error.setSuffix("° max start error")
            self.start_error_label = QLabel("Mapped start error: waiting")
            self.start_error_label.setWordWrap(True)
            mapping_actions.addWidget(self.save_mapping_button)
            mapping_actions.addWidget(self.sync_button)
            mapping_actions.addWidget(self.max_start_error)
            mapping_actions.addWidget(self.start_error_label, 1)
            mapping_layout.addLayout(mapping_actions)
            layout.addWidget(mapping)

            self.target_box = QGroupBox("Simulation TCP Control")
            target_grid = QGridLayout(self.target_box)
            self.use_current_button = QPushButton("Set Desired TCP from Simulation Current TCP")
            self.use_current_button.clicked.connect(self.use_current_tcp)
            target_grid.addWidget(self.use_current_button, 0, 0, 1, 2)
            target_grid.addWidget(QLabel("Translation step"), 0, 2)
            self.translation_step = QDoubleSpinBox()
            self.translation_step.setRange(1.0, 100.0)
            self.translation_step.setValue(10.0)
            self.translation_step.setSuffix(" mm")
            target_grid.addWidget(self.translation_step, 0, 3)
            target_grid.addWidget(QLabel("Rotation step"), 0, 4)
            self.rotation_step = QDoubleSpinBox()
            self.rotation_step.setRange(1.0, 45.0)
            self.rotation_step.setValue(5.0)
            self.rotation_step.setSuffix("°")
            target_grid.addWidget(self.rotation_step, 0, 5)
            self.keyboard_check = QCheckBox(
                "Keyboard control: W/S X, A/D Y, R/F Z, U/O roll, I/K pitch, J/L yaw"
            )
            self.keyboard_check.setChecked(True)
            target_grid.addWidget(self.keyboard_check, 1, 0, 1, 6)
            layout.addWidget(self.target_box)

            output = QGroupBox("Robot Output")
            output_layout = QHBoxLayout(output)
            self.stream_check = QCheckBox("Enable mapped joint stream")
            self.stream_check.toggled.connect(self.on_stream_toggled)
            self.deadman_check = QCheckBox("Deadman (check to enable)")
            self.deadman_check.toggled.connect(self.on_deadman_toggled)
            self.arm_button = QPushButton("Arm")
            self.arm_button.clicked.connect(lambda: node.arm(True))
            self.stop_button = QPushButton("Disarm / Stop")
            self.stop_button.clicked.connect(self.stop_all)
            output_layout.addWidget(self.stream_check)
            output_layout.addWidget(self.deadman_check)
            output_layout.addWidget(self.arm_button)
            output_layout.addWidget(self.stop_button)
            layout.addWidget(output)
            self.robot_connection = RobotConnection(node, self.vive_client, self.current_mapping)
            self.vive_panel.attach_robot_connection(self.robot_connection)

            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(root)
            self.setCentralWidget(scroll)
            self.installEventFilter(self)
            self.refresh_timer = QTimer(self)
            self.refresh_timer.timeout.connect(self.refresh)
            self.refresh_timer.start(100)
            self.refresh()

        def current_mapping(self):
            mapping = {}
            for row, robot_joint in enumerate(ROBOT_JOINTS):
                mapping[robot_joint] = {
                    "simulation_joint": self.table.cellWidget(row, 1).currentText(),
                    "direction": int(self.table.cellWidget(row, 2).currentText()),
                    "offset_deg": self.table.cellWidget(row, 3).value(),
                }
            return validate_mapping(mapping, SIM_JOINTS, ROBOT_JOINTS)

        def save_mapping(self):
            try:
                self.node.set_mapping(self.current_mapping())
                self.sync_status.setText(f"Mapping saved: {self.node.mapping_path}")
            except (OSError, KeyError, TypeError, ValueError) as exc:
                QMessageBox.warning(self, "Mapping Error", str(exc))

        def refresh(self):
            if self.vive_client.is_closed:
                return
            state = self.node.snapshot()
            vive = self.vive_client.snapshot()
            connection = self.robot_connection.snapshot()
            now = time.monotonic()
            sim_fresh = state["sim_q"] is not None and now - state["sim_time"] < 0.3
            robot_fresh = state["robot_q"] is not None and now - state["robot_time"] < 0.25
            current_fresh = state["current_pose"] is not None and now - state["current_pose_time"] < 0.5
            desired_fresh = state["desired_pose"] is not None and now - state["desired_time"] < 1.0
            self.sim_status.setText("Simulation: connected" if sim_fresh else "Simulation: waiting for joint state")
            self.robot_status.setText("Robot: feedback received" if robot_fresh else "Robot: waiting for DCP3 feedback")
            self.dcp3_status.setText(
                f"DCP3: {state['robot_status']}" if now - state["robot_status_time"] < 1.5 else "DCP3: waiting for status"
            )
            current = state["current_pose"]
            desired = self.target_pose_override or state["desired_pose"]
            task_frame = (vive["simulation"].get("task_frame", "tcp")
                          if vive["simulation_fresh"] else "TCP")
            self.target_box.setTitle(f"Simulation TCP Control ({task_frame})")
            self.use_current_button.setText(f"Set Desired {task_frame} from Current Pose")
            pose_text = f"Simulation Current {task_frame}: " + (
                f"({current[0]:+.3f}, {current[1]:+.3f}, {current[2]:+.3f}) m"
                if current_fresh else "waiting"
            )
            pose_text += f"  |  Desired {task_frame}: " + (
                f"({desired[0]:+.3f}, {desired[1]:+.3f}, {desired[2]:+.3f}) m"
                if desired is not None and (self.target_pose_override is not None or desired_fresh) else "waiting"
            )
            self.pose_status.setText(pose_text)
            # The combined hand controller owns keyboard/topic arbitration.
            # Retain the legacy controls only for launches without it.
            self._uses_hand_controller |= vive["simulation_fresh"]
            self.target_box.setVisible(not self._uses_hand_controller)

            mapped = None
            try:
                mapping = self.current_mapping()
                if sim_fresh:
                    mapped = map_sim_to_robot(state["sim_q"], SIM_JOINTS, ROBOT_JOINTS, mapping)
            except (KeyError, TypeError, ValueError) as exc:
                self.sync_status.setText(f"Mapping error: {exc}")
            mismatch = None
            if mapped is not None and robot_fresh:
                mismatch = max(abs(math.degrees(a - b)) for a, b in zip(mapped, state["robot_q"]))
                self.start_error_label.setText(
                    f"Mapped start error: {mismatch:.1f}° / {self.max_start_error.value():.1f}°"
                )
            else:
                self.start_error_label.setText("Mapped start error: waiting for feedback")
            if state["sync_result"] and now - state["sync_time"] < 5.0:
                self.sync_status.setText(f"Alignment: {state['sync_result']}")
            elif not self.sync_status.text().startswith("Mapping saved:"):
                self.sync_status.setText("Mapping and alignment: ready")

            not_armed = not state["armed"] and "arming" not in state["robot_status"].lower()
            live_target_edit = bool(
                state["armed"] and state["stream_enabled"] and state["deadman_held"]
                and sim_fresh and robot_fresh
            )
            self.target_box.setEnabled(bool((not_armed or live_target_edit)
                                           and not vive["simulation_running"]))
            self.use_current_button.setEnabled(bool(current_fresh and not_armed))
            editor_enabled = bool(not_armed and not state["stream_enabled"]
                                  and not vive["simulation_running"] and not connection["active"])
            for widget in (self.table, self.save_mapping_button, self.sync_button, self.max_start_error):
                widget.setEnabled(editor_enabled)
            self.stream_check.setEnabled(not state["armed"] and not connection["active"])
            if self.stream_check.isChecked() != state["stream_enabled"]:
                self.stream_check.blockSignals(True)
                self.stream_check.setChecked(state["stream_enabled"])
                self.stream_check.blockSignals(False)
            if self.deadman_check.isChecked() != state["deadman_held"]:
                self.deadman_check.blockSignals(True)
                self.deadman_check.setChecked(state["deadman_held"])
                self.deadman_check.blockSignals(False)
            self.deadman_check.setEnabled(
                bool(not connection["active"] and (state["deadman_held"] or (state["stream_enabled"] and sim_fresh and robot_fresh)))
            )
            mismatch_ok = mismatch is not None and mismatch <= self.max_start_error.value()
            self.arm_button.setEnabled(bool(
                not_armed and state["stream_enabled"] and state["deadman_held"]
                and sim_fresh and robot_fresh and mismatch_ok and not connection["active"]
            ))
            self.stop_button.setEnabled(bool(state["armed"] or "arming" in state["robot_status"].lower()
                                             or vive["can_stop_simulation"] or connection["active"]))
            if self.target_pose_override is not None and now - self.target_pose_override_time > 0.5:
                self.target_pose_override = None
            if self.pending_close and (
                not self.node.arm_client.service_is_ready()
                or now - self.pending_close_since > 2.5
                or "disarmed" in state["robot_status"].lower()
            ):
                self.force_close = True
                self.pending_close = False
                self.close()

        def on_stream_toggled(self, enabled):
            try:
                if enabled:
                    self.node.set_mapping(self.current_mapping())
                self.node.set_stream(enabled)
                state = self.node.snapshot()
                if not enabled and (state["armed"] or "arming" in state["robot_status"].lower()):
                    self.node.arm(False)
            except (OSError, KeyError, TypeError, ValueError) as exc:
                self.stream_check.blockSignals(True)
                self.stream_check.setChecked(False)
                self.stream_check.blockSignals(False)
                self.node.set_stream(False)
                QMessageBox.warning(self, "Stream Configuration Error", str(exc))

        def stop_all(self):
            self.vive_panel.release_run()
            if self.robot_connection.snapshot()["active"]:
                self.robot_connection.disconnect("Disarm / Stop requested.")
            else:
                self.node.arm(False)

        def on_deadman_toggled(self, enabled):
            if enabled:
                state = self.node.snapshot()
                now = time.monotonic()
                sim_fresh = state["sim_q"] is not None and now - state["sim_time"] < 0.3
                robot_fresh = state["robot_q"] is not None and now - state["robot_time"] < 0.25
                if not state["stream_enabled"] or not sim_fresh or not robot_fresh:
                    self.deadman_check.blockSignals(True)
                    self.deadman_check.setChecked(False)
                    self.deadman_check.blockSignals(False)
                    QMessageBox.warning(
                        self, "Deadman Unavailable",
                        "Enable the mapped joint stream and wait for fresh simulation and robot feedback first.",
                    )
                    return
            self.node.set_deadman(enabled)

        def eventFilter(self, watched, event):
            if self.vive_client.is_closed:
                return super().eventFilter(watched, event)
            if event.type() == QEvent.KeyPress:
                if self._uses_hand_controller:
                    return super().eventFilter(watched, event)
                if self.vive_panel.is_editing_topic():
                    return super().eventFilter(watched, event)
                translations = {
                    Qt.Key_W: (0, 1), Qt.Key_S: (0, -1),
                    Qt.Key_A: (1, 1), Qt.Key_D: (1, -1),
                    Qt.Key_R: (2, 1), Qt.Key_F: (2, -1),
                }
                rotations = {
                    Qt.Key_U: (0, 1), Qt.Key_O: (0, -1),
                    Qt.Key_I: (1, 1), Qt.Key_K: (1, -1),
                    Qt.Key_J: (2, 1), Qt.Key_L: (2, -1),
                }
                key = event.key()
                state = self.node.snapshot()
                target_fresh = self.target_pose_override is not None or (
                    state["desired_pose"] is not None and time.monotonic() - state["desired_time"] < 1.0
                )
                if self.target_box.isEnabled() and self.keyboard_check.isChecked() and target_fresh:
                    if key in translations:
                        self.nudge_position(*translations[key])
                        return True
                    if key in rotations:
                        self.nudge_rotation(*rotations[key])
                        return True
            if event.type() in (QEvent.ApplicationDeactivate, QEvent.WindowDeactivate):
                self.deadman_check.setChecked(False)
                self.node.set_deadman(False)
            return super().eventFilter(watched, event)

        def sync_robot_to_sim(self):
            if self.node.snapshot()["armed"]:
                QMessageBox.information(self, "Sync", "Disarm before syncing.")
                return
            try:
                self.node.set_mapping(self.current_mapping())
                self.target_pose_override = None
                request_id = self.node.sync_robot_to_sim()
                self.sync_status.setText(f"Sync request: {request_id[:8]}")
            except (OSError, KeyError, TypeError, ValueError) as exc:
                QMessageBox.warning(self, "Unable to Sync", str(exc))

        def use_current_tcp(self):
            state = self.node.snapshot()
            if state["current_pose"] is None or time.monotonic() - state["current_pose_time"] > 0.5:
                QMessageBox.warning(self, "Simulation TCP Unavailable", "No fresh simulation TCP pose is available.")
                return
            self.publish_target_override(list(state["current_pose"]))

        def nudge_position(self, axis, direction):
            state = self.node.snapshot()
            pose = self.target_pose_override or state["desired_pose"]
            if pose is None:
                return
            updated = list(pose)
            updated[axis] += direction * self.translation_step.value() / 1000.0
            self.publish_target_override(updated)

        @staticmethod
        def quaternion_multiply(left, right):
            x1, y1, z1, w1 = left
            x2, y2, z2, w2 = right
            return [w1*x2 + x1*w2 + y1*z2 - z1*y2,
                    w1*y2 - x1*z2 + y1*w2 + z1*x2,
                    w1*z2 + x1*y2 - y1*x2 + z1*w2,
                    w1*w2 - x1*x2 - y1*y2 - z1*z2]

        def nudge_rotation(self, axis, direction):
            state = self.node.snapshot()
            pose = self.target_pose_override or state["desired_pose"]
            if pose is None:
                return
            half = math.radians(self.rotation_step.value()) * direction / 2.0
            delta = [0.0, 0.0, 0.0, math.cos(half)]
            delta[axis] = math.sin(half)
            updated = list(pose)
            updated[3:] = self.quaternion_multiply(delta, pose[3:])
            self.publish_target_override(updated)

        def publish_target_override(self, pose):
            self.target_pose_override = list(pose)
            self.target_pose_override_time = time.monotonic()
            self.node.publish_target(self.target_pose_override)

        def closeEvent(self, event):
            if self.vive_client.is_closed:
                event.accept()
                return
            self.robot_connection.close()
            if self.force_close:
                event.accept()
                return
            state = self.node.snapshot()
            self.deadman_check.setChecked(False)
            self.node.set_deadman(False)
            if state["armed"] or "arming" in state["robot_status"].lower():
                if not self.pending_close:
                    self.pending_close = True
                    self.pending_close_since = time.monotonic()
                    self.node.arm(False)
                event.ignore()
            else:
                self.node.set_stream(False)
                event.accept()

    return MinimalTeleopWindow(node)


def main(args=None):
    from PyQt5.QtWidgets import QApplication

    rclpy.init(args=args)
    node = TeleopRosNode(node_name="indy7_teleop_minimal_gui")
    app = QApplication([])
    window = create_window(node)
    app.installEventFilter(window)
    window.show()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    result = app.exec_()
    window.vive_client.close()
    node.set_stream(False)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    raise SystemExit(result)


if __name__ == "__main__":
    main()
