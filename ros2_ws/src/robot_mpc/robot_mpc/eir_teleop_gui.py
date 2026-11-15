"""Simulation-only controls for both EIR hand targets."""

from __future__ import annotations

import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from .pose_mapping import pose_from_transform, transform_from_pose, transform_from_rpy
from .vive_mapping_client import ViveMappingClient


TORSO_JOINTS = ("Joint_L0", "Joint_L1")


class EIRTeleopNode(Node):
    def __init__(self, **kwargs):
        super().__init__("eir_teleop_gui", **kwargs)
        self._feedback_lock = threading.Lock()
        self._left_current = self._torso_current = None
        self._control_status = None
        self._joint_limit_active = None
        self._marker_targets, self._marker_edits, self._marker_displayed = {}, {}, {}
        self.marker_server = InteractiveMarkerServer(self, "/eir/goal_markers")
        self.create_timer(0.05, self._refresh_markers)
        self.left_pub = self.create_publisher(PoseStamped, "/eir/left/target_pose", 10)
        self.create_subscription(PoseStamped, "/eir/left/current_pose", self._on_left, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(String, "/mpc/status", self._on_control_status, 10)
        self.create_subscription(String, "/mpc/joint_limit_active", self._on_joint_limit_active, 10)

    def _on_left(self, message):
        p, q = message.pose.position, message.pose.orientation
        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        try:
            position, quaternion = pose_from_transform(transform_from_pose(pose[:3], pose[3:]))
        except ValueError:
            return
        with self._feedback_lock:
            self._left_current = (list(position) + list(quaternion),
                                  message.header.frame_id, time.monotonic())

    def _on_joints(self, message):
        joints = dict(zip(message.name, message.position))
        if not all(name in joints and math.isfinite(joints[name]) for name in TORSO_JOINTS):
            return
        with self._feedback_lock:
            self._torso_current = ([joints[name] for name in TORSO_JOINTS], time.monotonic())

    def _on_control_status(self, message):
        with self._feedback_lock:
            self._control_status = (message.data, time.monotonic())

    def _on_joint_limit_active(self, message):
        names = tuple(name.strip() for name in message.data.split(",") if name.strip())
        with self._feedback_lock:
            self._joint_limit_active = (names, time.monotonic())

    def feedback(self):
        with self._feedback_lock:
            now = time.monotonic()
            return {
                "left": (self._left_current if self._left_current
                         and now - self._left_current[2] < 0.5 else None),
                "torso": (self._torso_current if self._torso_current
                          and now - self._torso_current[1] < 0.5 else None),
                "control_status": (self._control_status[0] if self._control_status
                                   and now - self._control_status[1] < 1.0 else None),
                "joint_limit_active": (list(self._joint_limit_active[0])
                                       if self._joint_limit_active
                                       and now - self._joint_limit_active[1] < 1.0 else None),
            }

    def publish_targets(self, left, world_frame):
        if left is not None:
            message = PoseStamped()
            message.header.frame_id = world_frame
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, left[:3])
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = map(float, left[3:])
            self.left_pub.publish(message)

    def clear_marker_edits(self):
        with self._feedback_lock:
            self._marker_edits.clear()

    def sync_marker_targets(self, targets, world_frame):
        with self._feedback_lock:
            targets = dict(targets)
            for side, (pose, frame) in self._marker_edits.items():
                if frame == world_frame:
                    targets[side] = list(pose)
            self._marker_edits.clear()
            self._marker_targets = {side: (list(pose), world_frame)
                                    for side, pose in targets.items() if pose is not None}
            return targets

    def _on_marker_feedback(self, feedback):
        side = {"left_desired_tcp": "left", "right_desired_tcp": "right"}.get(feedback.marker_name)
        if (side is None
                or feedback.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,
                                               InteractiveMarkerFeedback.MOUSE_UP)):
            return
        p, q = feedback.pose.position, feedback.pose.orientation
        try:
            position, quaternion = pose_from_transform(
                transform_from_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        except ValueError:
            return
        with self._feedback_lock:
            target = self._marker_targets.get(side)
            if target is None or feedback.header.frame_id != target[1]:
                return
            pose = list(position) + list(quaternion)
            self._marker_edits[side] = (pose, target[1])
            if (feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE
                    and side in self._marker_displayed
                    and math.isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w,
                                     1.0, rel_tol=0.0, abs_tol=1e-6)):
                # The server already applies drag feedback; do not echo it via the slower Qt loop.
                # RViz stores its quaternion in Ogre's float precision.
                self._marker_displayed[side] = (tuple(pose), target[1])
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP and side in self._marker_displayed:
            # Commit the release now, including when the final POSE_UPDATE was dropped.
            message = PoseStamped()
            message.header.frame_id = target[1]
            message.pose.position.x, message.pose.position.y, message.pose.position.z = position
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = quaternion
            if self.marker_server.setPose(feedback.marker_name, message.pose, message.header):
                self._marker_displayed[side] = (tuple(pose), target[1])

    def _refresh_markers(self):
        # Run server operations on the ROS executor, outside the Qt feedback lock.
        with self._feedback_lock:
            targets = self._marker_targets | self._marker_edits
        for side, (pose, frame) in targets.items():
            displayed = (tuple(pose), frame)
            previous = self._marker_displayed.get(side)
            if displayed == previous:
                continue
            marker = InteractiveMarker()
            marker.header.frame_id = frame
            marker.name = f"{side}_desired_tcp"
            marker.description = f"{side.title()} Desired TCP"
            marker.scale = 0.22
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose[:3])
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = map(float, pose[3:])
            if previous is None or previous[1] != frame:
                center = InteractiveMarkerControl(always_visible=True)
                ball = Marker(type=Marker.SPHERE)
                ball.pose.orientation.w = 1.0
                ball.scale.x = ball.scale.y = ball.scale.z = .025
                ball.color.r, ball.color.g, ball.color.b, ball.color.a = (
                    (.1, .8, 1., 1.) if side == "left" else (1., .6, .1, 1.))
                center.markers.append(ball)
                marker.controls.append(center)
                half = math.sqrt(.5)
                for axis, quaternion in (("x", (0., 0., 0., 1.)),
                                         ("y", (0., 0., half, half)),
                                         ("z", (0., -half, 0., half))):
                    for mode, name in ((InteractiveMarkerControl.MOVE_AXIS, f"move_{axis}"),
                                       (InteractiveMarkerControl.ROTATE_AXIS, f"rotate_{axis}"),
                                       (InteractiveMarkerControl.MOVE_PLANE,
                                        {"x": "plane_yz", "y": "plane_xz", "z": "plane_xy"}[axis])):
                        control = InteractiveMarkerControl(name=name, interaction_mode=mode,
                                                           orientation_mode=InteractiveMarkerControl.FIXED)
                        (control.orientation.x, control.orientation.y,
                         control.orientation.z, control.orientation.w) = quaternion
                        if mode == InteractiveMarkerControl.MOVE_PLANE:
                            # RViz's default plane handle overlaps the rotation ring.
                            square = Marker(type=Marker.CUBE)
                            square.pose.orientation = control.orientation
                            (square.pose.position.x, square.pose.position.y,
                             square.pose.position.z) = [0.0 if a == axis else .055 for a in "xyz"]
                            square.scale.x, square.scale.y, square.scale.z = .003, .04, .04
                            (square.color.r, square.color.g, square.color.b,
                             square.color.a) = (*[float(a == axis) for a in "xyz"], .65)
                            control.description = f"Move in world {name[6:].upper()} plane"
                            control.markers.append(square)
                        marker.controls.append(control)
                self.marker_server.insert(marker, feedback_callback=self._on_marker_feedback)
            else:
                self.marker_server.setPose(marker.name, marker.pose, marker.header)
            self._marker_displayed[side] = displayed
        self.marker_server.applyChanges()


def create_window(node, client):
    from PyQt5.QtCore import QEvent, QTimer, Qt
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox,
        QHBoxLayout, QLabel, QMainWindow, QPushButton, QVBoxLayout, QWidget,
    )

    class EIRTeleopWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.targets = {"left": None, "right": None}
            self._runtime_id = None
            self._run_source = None
            self._f7_down = False
            self.setWindowTitle("EIR | Both hands | Simulation")
            self.resize(1150, 620)
            root = QWidget()
            layout = QVBoxLayout(root)
            self.status = QLabel("Waiting for EIR simulation feedback")
            self.status.setWordWrap(True)
            layout.addWidget(self.status)
            self.control_status = QLabel("Controller: waiting for status")
            self.control_status.setWordWrap(True)
            self.control_status.setTextFormat(Qt.PlainText)
            layout.addWidget(self.control_status)
            options = QHBoxLayout()
            self.both_hands = QCheckBox("Move both hands with either panel (same world delta)")
            self.both_hands.setChecked(True)
            options.addWidget(self.both_hands)
            self.translation_step = QDoubleSpinBox()
            self.translation_step.setRange(1, 100)
            self.translation_step.setValue(10)
            self.translation_step.setSuffix(" mm")
            options.addWidget(self.translation_step)
            self.rotation_step = QDoubleSpinBox()
            self.rotation_step.setRange(0.5, 30)
            self.rotation_step.setValue(5)
            self.rotation_step.setSuffix("°")
            options.addWidget(self.rotation_step)
            layout.addLayout(options)
            panels = QHBoxLayout()
            self.hand_labels, self.hand_buttons = {}, {}
            for side in ("left", "right"):
                box = QGroupBox(f"{side.title()} hand TCP | world frame")
                grid = QGridLayout(box)
                self.hand_labels[side] = QLabel("Current / target: waiting")
                self.hand_labels[side].setWordWrap(True)
                grid.addWidget(self.hand_labels[side], 0, 0, 1, 3)
                buttons = []
                for row, name in enumerate(("X", "Y", "Z", "Roll", "Pitch", "Yaw"), 1):
                    grid.addWidget(QLabel(name), row, 0)
                    for column, direction in ((1, -1), (2, 1)):
                        button = QPushButton("−" if direction < 0 else "+")
                        button.clicked.connect(lambda _checked=False, s=side, a=(row-1) % 3,
                                              d=direction, r=row > 3: self.jog_hand(s, a, d, r))
                        grid.addWidget(button, row, column)
                        buttons.append(button)
                self.hand_buttons[side] = buttons
                panels.addWidget(box)
            layout.addLayout(panels)
            self.torso_label = QLabel("Torso yaw, pitch: waiting (automatic)")
            layout.addWidget(self.torso_label)
            actions = QHBoxLayout()
            self.reset_button = QPushButton("Reset both targets to current (F5)")
            self.reset_button.clicked.connect(self.reset_targets)
            actions.addWidget(self.reset_button)
            self.run_checkbox = QCheckBox("Run MPC")
            self.run_checkbox.toggled.connect(self.toggle_run)
            actions.addWidget(self.run_checkbox)
            layout.addLayout(actions)
            layout.addWidget(QLabel("Check Run MPC to move; uncheck it to pause. F7 also works. "
                                    "Uncheck Move both to set independent hand goals."))
            layout.addWidget(QLabel("RViz Interact: each hand has XYZ arrows/planes for position and rings for rotation."))
            self.setCentralWidget(root)
            QApplication.instance().installEventFilter(self)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(50)
            self.refresh()

        def currents(self):
            state, feedback = client.snapshot(), node.feedback()
            world = state["simulation"].get("world_frame", "world")
            hands = {"left": feedback["left"], "right": state["simulation_current"]}
            hands = {side: value if value and value[1] == world else None
                     for side, value in hands.items()}
            return state, hands, feedback["torso"]

        def reset_targets(self):
            state, hands, torso = self.currents()
            if state["run_held"] or state["simulation_running"]:
                return False
            if not all(hands.values()) or torso is None:
                return False
            node.clear_marker_edits()
            self.targets = {side: list(value[0]) for side, value in hands.items()}
            client.set_keyboard_target(self.targets["right"])
            self.refresh()
            return True

        def jog_hand(self, side, axis, direction, rotation=False):
            _, hands, _ = self.currents()
            sides = ("left", "right") if self.both_hands.isChecked() else (side,)
            if not all(hands[s] is not None and self.targets[s] is not None for s in sides):
                return
            for selected in sides:
                pose = self.targets[selected]
                if rotation:
                    rpy = [0., 0., 0.]
                    rpy[axis] = direction * self.rotation_step.value()
                    target = transform_from_pose(pose[:3], pose[3:])
                    target[:3, :3] = transform_from_rpy([0, 0, 0], rpy)[:3, :3] @ target[:3, :3]
                    position, quaternion = pose_from_transform(target)
                    self.targets[selected] = list(position) + list(quaternion)
                else:
                    pose[axis] += direction * self.translation_step.value() / 1000.
            self.refresh()

        def start_run(self, source):
            state, hands, torso = self.currents()
            if (self._run_source is not None or not all(hands.values()) or torso is None
                    or not all(value is not None for value in self.targets.values())):
                return False
            if client.request_run(True):
                self._run_source = source
                return True
            return False

        def toggle_run(self, enabled):
            if enabled:
                if not self.start_run("checkbox"):
                    self.run_checkbox.setChecked(False)
            elif self._run_source == "checkbox":
                self.release_run()

        def release_run(self):
            state = client.snapshot()
            held = self._run_source is not None or state["run_held"]
            self._run_source = None
            self._f7_down = False
            if self.run_checkbox.isChecked():
                self.run_checkbox.setChecked(False)
            if not client.is_closed and (held or state["can_stop_simulation"]):
                client.request_run(False)

        def eventFilter(self, watched, event):
            kind = event.type()
            loses_window = kind in (QEvent.ApplicationDeactivate, QEvent.WindowDeactivate)
            closes_window = watched is self and kind in (QEvent.Hide, QEvent.Close)
            if closes_window or (loses_window and self._run_source != "checkbox"):
                self.release_run()
            if (isinstance(watched, QWidget) and watched.window() is self and self.isVisible()
                    and kind in (QEvent.ShortcutOverride, QEvent.KeyPress, QEvent.KeyRelease)
                    and event.key() in (Qt.Key_F5, Qt.Key_F7)):
                event.accept()
                if kind == QEvent.ShortcutOverride or event.isAutoRepeat():
                    return True
                if event.key() == Qt.Key_F5 and kind == QEvent.KeyPress:
                    self.reset_targets()
                elif event.key() == Qt.Key_F7:
                    if kind == QEvent.KeyPress and not self._f7_down:
                        self._f7_down = True
                        self.start_run("key")
                    elif kind == QEvent.KeyRelease and self._run_source == "key":
                        self.release_run()
                    elif kind == QEvent.KeyRelease:
                        self._f7_down = False
                return True
            return super().eventFilter(watched, event)

        def refresh(self):
            if client.is_closed:
                return
            state, hands, torso = self.currents()
            runtime = state["simulation"].get("runtime_id")
            if self._runtime_id and runtime and runtime != self._runtime_id:
                self.release_run()
                self.targets = {"left": None, "right": None}
                node.clear_marker_edits()
            self._runtime_id = runtime
            if state["input_mode"] != "keyboard" and state["can_change_mode"]:
                client.request_mode("keyboard")
            for side, current in hands.items():
                if self.targets[side] is None and current:
                    self.targets[side] = list(current[0])
            self.targets = node.sync_marker_targets(self.targets,
                                                   state["simulation"].get("world_frame", "world"))
            for side, current in hands.items():
                target = self.targets[side]
                def pose_text(pose):
                    if pose is None:
                        return "waiting"
                    xyz = ", ".join(f"{v*1000:.1f}" for v in pose[:3])
                    quat = ", ".join(f"{v:.3f}" for v in pose[3:])
                    return f"XYZ [{xyz}] mm; quat [{quat}]"
                self.hand_labels[side].setText(
                    f"Current: {pose_text(current[0] if current else None)}\nTarget: {pose_text(target)}")
                for button in self.hand_buttons[side]:
                    button.setEnabled(current is not None and target is not None)
            def angles(value):
                return ", ".join(f"{math.degrees(q):.2f}°" for q in value) if value else "waiting"
            self.torso_label.setText(f"Torso yaw, pitch: {angles(torso[0] if torso else None)} (automatic)")
            ready = all(hands.values()) and torso is not None
            self.reset_button.setEnabled(bool(ready and not state["run_held"] and not state["simulation_running"]))
            self.run_checkbox.setEnabled(bool(ready and (state["can_run_simulation"] or state["run_held"]
                                                          or state["simulation_running"])))
            if self._run_source is not None and (
                    not ready or not state["simulation_fresh"]
                    or not state["run_held"] and not state["simulation_running"]):
                self.release_run()
            message = state["result"]
            if message == "Hold both poses still, then click Map or press F5.":
                message = "Edit targets, then check Run MPC to move."
            message = message.replace("Release F7 and hold again", "uncheck Run MPC, then check it again")
            message = message.replace("release F7 and hold again", "uncheck Run MPC, then check it again")
            message = message.replace("Release F7 to pause", "uncheck Run MPC to pause")
            message = message.replace("Hold F7 to follow", "check Run MPC to follow")
            message = message.replace("hold F7 again", "check Run MPC again")
            self.status.setText(("RUNNING" if state["simulation_running"] else "PAUSED")
                                + " | " + message)
            feedback = node.feedback()
            code = feedback["control_status"]
            active_joints = feedback["joint_limit_active"]
            reasons = {
                "solved": "Tracking target",
                "stationary_hold": "Target reached; holding pose",
                "joint_position_limit_hold": "Joint limit is blocking progress",
                "joint_position_limit_limited": "Joint limit is constraining this move",
                "joint_position_limit_active": "Joint limit is active",
                "cartesian_path_hold": "Hand path constraint blocks this move",
                "left_cartesian_path_hold": "Left hand path constraint blocks this move",
                "cartesian_path_limited": "Hand path constraint limits this move",
                "left_cartesian_path_limited": "Left hand path constraint limits this move",
                "mirror_path_hold": "Relative hand path constraint blocks this move",
                "mirror_path_limited": "Relative hand path constraint limits this move",
                "workspace_floor_hold": "Floor constraint blocks this move",
                "left_workspace_floor_hold": "Left hand floor constraint blocks this move",
                "workspace_floor_limited": "Floor constraint limits this move",
                "left_workspace_floor_limited": "Left hand floor constraint limits this move",
                "workspace_floor_violation": "Hand is below the configured floor",
                "paused_waiting_for_f7": "Paused; check Run MPC or hold F7 to move",
            }
            if code in reasons:
                detail = reasons[code]
                if code.startswith("joint_position_limit_"):
                    names = ", ".join(active_joints or ()) or "unavailable"
                    detail += f" | Active joints: {names}"
                detail = f"{detail} [{code}]"
            else:
                detail = code or "waiting for status"
            self.control_status.setText("Controller: " + detail)
            if state["simulation_fresh"] and state["input_mode"] == "keyboard":
                if self.targets["right"] is not None:
                    client.set_keyboard_target(self.targets["right"])
                client.publish_keyboard_target()
                node.publish_targets(self.targets["left"],
                                     state["simulation"].get("world_frame", "world"))
            client.publish_run_hold()

        def closeEvent(self, event):
            self.timer.stop()
            self.release_run()
            client.close()
            QApplication.instance().removeEventFilter(self)
            event.accept()

    return EIRTeleopWindow()


def main(args=None):
    from PyQt5.QtWidgets import QApplication

    rclpy.init(args=args)
    node = EIRTeleopNode()
    client = ViveMappingClient(node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    app = QApplication.instance() or QApplication([])
    window = create_window(node, client)

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
        window.close()
        client.close()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)
