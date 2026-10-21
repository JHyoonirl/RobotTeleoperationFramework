#!/usr/bin/env python3
"""Qt control panel for mapping and guarded simulation-to-Indy teleoperation."""

from __future__ import annotations

import math
import threading
import time

import rclpy

from .teleop_ros_node import ROBOT_JOINTS, SIM_JOINTS, TeleopRosNode
from .joint_mapping import map_sim_to_robot, validate_mapping


def main(args=None):
    try:
        from PyQt5.QtCore import QEvent, QTimer, Qt
        from PyQt5.QtWidgets import (
            QApplication,
            QComboBox,
            QDoubleSpinBox,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
            QCheckBox,
        )
    except ImportError as exc:
        raise RuntimeError("The teleoperation GUI requires python3-pyqt5.") from exc

    class TeleopWindow(QMainWindow):
        def __init__(self, ros_node):
            super().__init__()
            self.node = ros_node
            self.mapping = dict(ros_node.mapping)
            self.target_pose_override = None
            self.target_pose_override_time = 0.0
            self.pending_close = False
            self.pending_close_since = 0.0
            self.force_close = False
            self.setWindowTitle("Indy7 Simulation ↔ DCP3 Teleoperation")
            self.resize(1360, 920)
            root = QWidget()
            main_layout = QVBoxLayout(root)

            status_box = QGroupBox("Connection and Status")
            status_grid = QGridLayout(status_box)
            self.sim_label = QLabel("Simulation: Waiting")
            self.robot_label = QLabel("Robot: Waiting")
            self.dcp3_label = QLabel("DCP3: disarmed")
            self.current_pose_label = QLabel("Simulation TCP: Waiting")
            self.pose_label = QLabel("Desired pose: Waiting")
            self.service_label = QLabel("")
            status_grid.addWidget(self.sim_label, 0, 0)
            status_grid.addWidget(self.robot_label, 0, 1)
            status_grid.addWidget(self.dcp3_label, 0, 2)
            status_grid.addWidget(self.current_pose_label, 1, 0)
            status_grid.addWidget(self.pose_label, 1, 1)
            status_grid.addWidget(self.service_label, 2, 0, 1, 3)
            main_layout.addWidget(status_box)

            mapping_box = QGroupBox("Joint Mapping · q_robot = direction × q_sim + offset")
            mapping_layout = QVBoxLayout(mapping_box)
            self.table = QTableWidget(6, 8)
            self.table.setHorizontalHeaderLabels(
                ["Robot Joint", "Simulation Joint", "Direction", "Offset [deg]", "Simulation [deg]", "Mapped Target [deg]", "Robot Measured [deg]", "Target Error [deg]"]
            )
            self.table.verticalHeader().setVisible(False)
            self.table.setMinimumHeight(255)
            for row, robot_joint in enumerate(ROBOT_JOINTS):
                self.table.setItem(row, 0, QTableWidgetItem(robot_joint))
                self.table.item(row, 0).setFlags(
                    self.table.item(row, 0).flags() & ~Qt.ItemIsEditable
                )
                sim_combo = QComboBox()
                sim_combo.addItems(SIM_JOINTS)
                sign_combo = QComboBox()
                sign_combo.addItems(["+1", "-1"])
                offset = QDoubleSpinBox()
                offset.setRange(-720.0, 720.0)
                offset.setDecimals(3)
                offset.setSuffix("°")
                row_config = self.mapping[robot_joint]
                sim_combo.setCurrentText(row_config["simulation_joint"])
                sign_combo.setCurrentText(f"{row_config['direction']:+d}")
                offset.setValue(float(row_config["offset_deg"]))
                self.table.setCellWidget(row, 1, sim_combo)
                self.table.setCellWidget(row, 2, sign_combo)
                self.table.setCellWidget(row, 3, offset)
                for column in range(4, 8):
                    item = QTableWidgetItem("—")
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    self.table.setItem(row, column, item)
                sim_combo.currentTextChanged.connect(self.refresh)
                sign_combo.currentTextChanged.connect(self.refresh)
                offset.valueChanged.connect(self.refresh)
            self.table.resizeColumnsToContents()
            mapping_layout.addWidget(self.table)
            mapping_buttons = QHBoxLayout()
            self.save_mapping_button = QPushButton("Save Mapping")
            self.save_mapping_button.clicked.connect(self.save_current_mapping)
            self.sync_button = QPushButton("Sync Robot Pose to Simulation")
            self.sync_button.clicked.connect(self.sync_robot_to_sim)
            self.max_start_diff = QDoubleSpinBox()
            self.max_start_diff.setRange(0.5, 180.0)
            self.max_start_diff.setValue(10.0)
            self.max_start_diff.setSuffix("° max start error")
            self.sync_label = QLabel("Sync: Waiting")
            mapping_buttons.addWidget(self.save_mapping_button)
            mapping_buttons.addWidget(self.sync_button)
            mapping_buttons.addWidget(self.max_start_diff)
            mapping_buttons.addWidget(self.sync_label)
            mapping_layout.addLayout(mapping_buttons)
            main_layout.addWidget(mapping_box)

            target_box = QGroupBox("Simulation Desired TCP Pose Controls")
            self.target_box = target_box
            target_layout = QGridLayout(target_box)
            self.translation_step = QDoubleSpinBox()
            self.translation_step.setRange(1.0, 100.0)
            self.translation_step.setValue(10.0)
            self.translation_step.setSuffix(" mm")
            self.rotation_step = QDoubleSpinBox()
            self.rotation_step.setRange(1.0, 45.0)
            self.rotation_step.setValue(5.0)
            self.rotation_step.setSuffix("°")
            self.current_tcp_button = QPushButton("Set Desired TCP from Simulation Current TCP")
            self.current_tcp_button.clicked.connect(self.use_simulation_current_tcp)
            target_layout.addWidget(self.current_tcp_button, 0, 0, 1, 2)
            target_layout.addWidget(QLabel("Translation step"), 0, 2)
            target_layout.addWidget(self.translation_step, 0, 3)
            target_layout.addWidget(QLabel("Rotation step"), 0, 4)
            target_layout.addWidget(self.rotation_step, 0, 5)
            self.keyboard_check = QCheckBox(
                "Keyboard TCP control (W/S: X, A/D: Y, R/F: Z, U/O: roll, I/K: pitch, J/L: yaw)"
            )
            self.keyboard_check.setChecked(True)
            target_layout.addWidget(self.keyboard_check, 1, 0, 1, 6)

            self.target_buttons = []
            position_box = QGroupBox("Position [m]")
            position_grid = QGridLayout(position_box)
            position_buttons = [("X−", 0, 0, 0, -1), ("X+", 0, 2, 0, 1),
                                ("Y+", 1, 0, 1, 1), ("Y−", 1, 2, 1, -1),
                                ("Z+", 0, 4, 2, 1), ("Z−", 1, 4, 2, -1)]
            for text, row, column, axis, direction in position_buttons:
                button = QPushButton(text)
                button.clicked.connect(lambda checked=False, a=axis, d=direction: self.nudge_position(a, d))
                position_grid.addWidget(button, row, column)
                self.target_buttons.append(button)
            rotation_box = QGroupBox("Orientation [deg]")
            rotation_grid = QGridLayout(rotation_box)
            rotation_buttons = [("Roll−", 0, 0, 0, -1), ("Roll+", 0, 1, 0, 1),
                                ("Pitch−", 1, 0, 1, -1), ("Pitch+", 1, 1, 1, 1),
                                ("Yaw−", 2, 0, 2, -1), ("Yaw+", 2, 1, 2, 1)]
            for text, row, column, axis, direction in rotation_buttons:
                button = QPushButton(text)
                button.clicked.connect(lambda checked=False, a=axis, d=direction: self.nudge_rotation(a, d))
                rotation_grid.addWidget(button, row, column)
                self.target_buttons.append(button)
            target_layout.addWidget(position_box, 2, 0, 1, 3)
            target_layout.addWidget(rotation_box, 2, 3, 1, 3)
            target_layout.addWidget(
                QLabel("Each press moves one step; holding a key repeats. Keep the GUI focused while teleoperating."),
                3, 0, 1, 6,
            )
            main_layout.addWidget(target_box)

            control_box = QGroupBox("Simulation → Robot Teleoperation")
            control_layout = QHBoxLayout(control_box)
            self.stream_check = QCheckBox("Enable mapped simulation joint command stream")
            self.stream_check.toggled.connect(self.on_stream_toggled)
            self.deadman_check = QCheckBox("Deadman (check to enable)")
            self.deadman_check.toggled.connect(self.on_deadman_toggled)
            self.arm_button = QPushButton("Arm")
            self.arm_button.clicked.connect(lambda: self.node.arm(True))
            self.disarm_button = QPushButton("Disarm / Stop")
            self.disarm_button.clicked.connect(lambda: self.node.arm(False))
            control_layout.addWidget(self.stream_check)
            control_layout.addWidget(self.deadman_check)
            control_layout.addWidget(self.arm_button)
            control_layout.addWidget(self.disarm_button)
            main_layout.addWidget(control_box)

            self.setCentralWidget(root)
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

        def save_current_mapping(self):
            try:
                self.mapping = self.current_mapping()
                self.node.set_mapping(self.mapping)
                self.sync_label.setText(f"Mapping saved: {self.node.mapping_path}")
            except (OSError, KeyError, TypeError, ValueError) as exc:
                QMessageBox.warning(self, "Mapping Error", str(exc))

        def _display_joint(self, row, column, value):
            item = self.table.item(row, column)
            item.setText("—" if value is None else f"{math.degrees(value):+.2f}°")

        def refresh(self):
            state = self.node.snapshot()
            now = time.monotonic()
            sim_fresh = state["sim_q"] is not None and now - state["sim_time"] < 0.3
            robot_fresh = state["robot_q"] is not None and now - state["robot_time"] < 0.25
            pose_fresh = state["desired_pose"] is not None and now - state["desired_time"] < 1.0
            current_pose_fresh = state["current_pose"] is not None and now - state["current_pose_time"] < 0.5
            self.sim_label.setText("Simulation: Connected" if sim_fresh else "Simulation: Waiting for joint state / stale")
            self.robot_label.setText("Robot: Feedback received" if robot_fresh else "Robot: Waiting for DCP3 feedback / stale")
            if current_pose_fresh:
                cp = state["current_pose"]
                self.current_pose_label.setText(
                    f"Simulation TCP [m]: x={cp[0]:+.3f}, y={cp[1]:+.3f}, z={cp[2]:+.3f}"
                )
            else:
                self.current_pose_label.setText("Simulation TCP: Waiting for MPC")
            if now - state["robot_status_time"] < 1.5:
                self.dcp3_label.setText(f"DCP3: {state['robot_status']}")
            else:
                self.dcp3_label.setText("DCP3: Waiting for status / disconnected")
            if pose_fresh:
                p = state["desired_pose"]
                self.pose_label.setText(f"Desired TCP [m]: x={p[0]:+.3f}, y={p[1]:+.3f}, z={p[2]:+.3f}")
            else:
                self.pose_label.setText("Desired pose: Waiting for MPC")
            if now - state["sync_time"] < 5.0 and state["sync_result"]:
                self.sync_label.setText(f"Alignment: {state['sync_result']}")
            elif not self.sync_label.text().startswith("Mapping saved:"):
                self.sync_label.setText("Alignment: Ready")
            if now - state["service_time"] < 5.0:
                self.service_label.setText(state["service_result"])
            try:
                mapping = self.current_mapping()
                mapped = map_sim_to_robot(state["sim_q"], SIM_JOINTS, ROBOT_JOINTS, mapping) if sim_fresh else None
            except (KeyError, TypeError, ValueError) as exc:
                mapping, mapped = None, None
                self.sync_label.setText(f"Mapping error: {exc}")
            if mapped is not None and robot_fresh:
                difference_deg = max(abs(math.degrees(a - b)) for a, b in zip(mapped, state["robot_q"]))
                self.robot_label.setText(
                    f"Robot: Feedback received · max start error {difference_deg:.1f}°"
                )
                sim_by_name = dict(zip(SIM_JOINTS, state["sim_q"]))
                for row, robot_joint in enumerate(ROBOT_JOINTS):
                    selected_sim_joint = mapping[robot_joint]["simulation_joint"]
                    self._display_joint(row, 4, sim_by_name[selected_sim_joint])
                    self._display_joint(row, 5, mapped[row])
                    self._display_joint(row, 6, state["robot_q"][row])
                    self._display_joint(row, 7, state["robot_q"][row] - mapped[row])
            else:
                for row in range(6):
                    self._display_joint(row, 4, state["sim_q"][row] if sim_fresh else None)
                    self._display_joint(row, 5, mapped[row] if mapped is not None else None)
                    self._display_joint(row, 6, state["robot_q"][row] if robot_fresh else None)
                    self._display_joint(row, 7, None)
            not_armed = (
                not state["armed"]
                and "arming" not in state["robot_status"].lower()
            )
            editor_enabled = not_armed and not state["stream_enabled"]
            for widget in [self.save_mapping_button, self.table, self.sync_button]:
                widget.setEnabled(editor_enabled)
            live_target_edit = bool(
                state["armed"]
                and state["stream_enabled"]
                and state["deadman_held"]
                and sim_fresh
                and robot_fresh
            )
            target_edit_enabled = bool(not_armed or live_target_edit)
            self.target_box.setEnabled(target_edit_enabled)
            self.current_tcp_button.setEnabled(bool(current_pose_fresh and not_armed))
            target_control_fresh = bool(
                self.target_pose_override is not None or pose_fresh
            )
            for button in self.target_buttons:
                button.setEnabled(bool(target_control_fresh and target_edit_enabled))
            self.stream_check.setEnabled(not state["armed"])
            if self.deadman_check.isChecked() != state["deadman_held"]:
                self.deadman_check.blockSignals(True)
                self.deadman_check.setChecked(state["deadman_held"])
                self.deadman_check.blockSignals(False)
            self.deadman_check.setEnabled(
                bool(
                    state["deadman_held"]
                    or (state["stream_enabled"] and sim_fresh and robot_fresh)
                )
            )
            mismatch_ok = False
            if mapped is not None and robot_fresh:
                mismatch_ok = max(abs(math.degrees(a - b)) for a, b in zip(mapped, state["robot_q"])) <= self.max_start_diff.value()
            self.arm_button.setEnabled(bool(not_armed and state["stream_enabled"] and state["deadman_held"] and sim_fresh and robot_fresh and mismatch_ok))
            self.disarm_button.setEnabled(bool(state["armed"] or "arming" in state["robot_status"].lower()))
            if (
                self.target_pose_override is not None
                and now - self.target_pose_override_time > 0.5
            ):
                self.target_pose_override = None
            if self.pending_close and (not self.node.arm_client.service_is_ready() or now - self.pending_close_since > 2.5 or "disarmed" in state["robot_status"].lower()):
                self.force_close = True
                self.pending_close = False
                self.close()

        def on_stream_toggled(self, enabled):
            try:
                if enabled:
                    self.node.set_mapping(self.current_mapping())
                self.node.set_stream(enabled)
                if not enabled and (self.node.snapshot()["armed"] or "arming" in self.node.snapshot()["robot_status"].lower()):
                    self.node.arm(False)
            except (OSError, KeyError, TypeError, ValueError) as exc:
                self.stream_check.blockSignals(True)
                self.stream_check.setChecked(False)
                self.stream_check.blockSignals(False)
                self.node.set_stream(False)
                QMessageBox.warning(self, "Stream Configuration Error", str(exc))

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
                        self,
                        "Deadman Unavailable",
                        "Enable command streaming and wait for fresh simulation and robot feedback first.",
                    )
                    return
            self.node.set_deadman(enabled)

        def eventFilter(self, watched, event):
            if event.type() == QEvent.KeyPress:
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
                target_fresh = (
                    self.target_pose_override is not None
                    or (
                        state["desired_pose"] is not None
                        and time.monotonic() - state["desired_time"] < 1.0
                    )
                )
                if (
                    self.target_box.isEnabled()
                    and self.keyboard_check.isChecked()
                    and target_fresh
                ):
                    if key in translations:
                        axis, direction = translations[key]
                        self.nudge_position(axis, direction)
                        return True
                    if key in rotations:
                        axis, direction = rotations[key]
                        self.nudge_rotation(axis, direction)
                        return True
            if event.type() == QEvent.ApplicationDeactivate or event.type() == QEvent.WindowDeactivate:
                self.deadman_check.setChecked(False)
                self.node.set_deadman(False)
            return super().eventFilter(watched, event)

        def sync_robot_to_sim(self):
            if self.node.snapshot()["armed"]:
                QMessageBox.information(self, "Sync", "Disarm before syncing.")
                return
            try:
                mapping = self.current_mapping()
                self.node.set_mapping(mapping)
                self.target_pose_override = None
                request_id = self.node.sync_robot_to_sim()
                self.sync_label.setText(f"Sync request: {request_id[:8]}")
            except (OSError, KeyError, TypeError, ValueError) as exc:
                QMessageBox.warning(self, "Unable to Sync", str(exc))

        def nudge_position(self, axis, direction):
            pose = self.target_pose_override or self.node.snapshot()["desired_pose"]
            if pose is None:
                return
            updated = list(pose)
            updated[axis] += direction * self.translation_step.value() / 1000.0
            self.publish_target_override(updated)

        @staticmethod
        def _quat_multiply(left, right):
            x1, y1, z1, w1 = left
            x2, y2, z2, w2 = right
            return [w1*x2 + x1*w2 + y1*z2 - z1*y2,
                    w1*y2 - x1*z2 + y1*w2 + z1*x2,
                    w1*z2 + x1*y2 - y1*x2 + z1*w2,
                    w1*w2 - x1*x2 - y1*y2 - z1*z2]

        def nudge_rotation(self, axis, direction):
            pose = self.target_pose_override or self.node.snapshot()["desired_pose"]
            if pose is None:
                return
            half = math.radians(self.rotation_step.value()) * direction / 2.0
            delta = [0.0, 0.0, 0.0, math.cos(half)]
            delta[axis] = math.sin(half)
            updated = list(pose)
            updated[3:] = self._quat_multiply(delta, pose[3:])
            self.publish_target_override(updated)

        def use_simulation_current_tcp(self):
            state = self.node.snapshot()
            if state["current_pose"] is None or time.monotonic() - state["current_pose_time"] > 0.5:
                QMessageBox.warning(self, "Simulation TCP Unavailable", "No fresh simulation TCP pose is available.")
                return
            self.target_pose_override = list(state["current_pose"])
            self.target_pose_override_time = time.monotonic()
            self.node.publish_target(self.target_pose_override)

        def publish_target_override(self, pose):
            self.target_pose_override = list(pose)
            self.target_pose_override_time = time.monotonic()
            self.node.publish_target(self.target_pose_override)

        def closeEvent(self, event):
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

    rclpy.init(args=args)
    node = TeleopRosNode()
    app = QApplication([])
    window = TeleopWindow(node)
    app.installEventFilter(window)
    window.show()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    result = app.exec_()
    node.set_stream(False)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    raise SystemExit(result)


if __name__ == "__main__":
    main()
