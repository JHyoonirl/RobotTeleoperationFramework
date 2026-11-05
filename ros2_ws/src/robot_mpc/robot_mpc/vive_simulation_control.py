"""F7-held hand target following with exclusive keyboard/topic pose inputs."""

import json
import math
import time
import uuid

import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.time import Time
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from .pose_mapping import pose_from_transform, transform_from_pose


class ViveSimulationControl:
    def __init__(self, node):
        if not node.simulate or node.control_mode != "task_space":
            raise ValueError("Vive simulation following requires simulate=true and task_space mode")
        if (not node.internal_simulation
                and node.joint_command_pub.topic_name != "/forward_position_controller/commands"):
            raise ValueError("Vive simulation output must use /forward_position_controller/commands")
        self.node = node
        self.runtime_id = uuid.uuid4().hex
        self.following = False
        self.message = "Waiting for F5 mapping. Hold F7 to run."
        self._hold_latched = False
        self._hold_received = None
        self._hold_token = ""
        self._mapping = None
        self._mapping_time = 0.0
        self._mapping_key = None
        self._target = None
        self._target_error = "No mapped hand target received"
        self._target_dirty = False
        self._keyboard_target = None
        self._keyboard_error = "Waiting for a keyboard hand target"
        self._keyboard_dirty = False
        self.input_mode = str(node.get_parameter("teleop_input_mode").value)
        if self.input_mode not in ("keyboard", "topic"):
            raise ValueError("teleop_input_mode must be keyboard or topic")
        self.hand_frame = str(node.get_parameter("vive_hand_frame").value)
        if node.task_frame != self.hand_frame:
            raise ValueError("Vive following must control the hand frame directly")
        self.timeout = float(node.get_parameter("vive_input_timeout").value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("vive_input_timeout must be positive and finite")
        self.hold_timeout = float(node.get_parameter("vive_hold_timeout").value)
        if not math.isfinite(self.hold_timeout) or self.hold_timeout <= 0:
            raise ValueError("vive_hold_timeout must be positive and finite")
        self.status_pub = node.create_publisher(
            String, str(node.get_parameter("vive_run_status_topic").value), 10)
        self.target_pub = node.create_publisher(
            PoseStamped, str(node.get_parameter("hand_target_pose_topic").value), 10)
        self.target_frame = str(node.get_parameter("hand_target_frame").value)
        if not self.target_frame or self.target_frame in (self.hand_frame, node.pose_frame):
            raise ValueError("hand_target_frame must be distinct from the world and robot hand frames")
        self._broadcaster = TransformBroadcaster(node)
        node.create_subscription(String, str(node.get_parameter("vive_mapping_status_topic").value),
                                 self._on_mapping, 10)
        node.create_subscription(PoseStamped, str(node.get_parameter("vive_hand_pose_topic").value),
                                 self._on_target, 1)
        node.create_subscription(PoseStamped, str(node.get_parameter("keyboard_hand_pose_topic").value),
                                 self._on_keyboard_target, 1)
        node.create_service(SetBool, str(node.get_parameter("vive_run_service").value), self._run)
        node.create_service(SetBool, str(node.get_parameter("teleop_mode_service").value), self._set_mode)
        node.create_subscription(String, str(node.get_parameter("vive_hold_topic").value),
                                 self._on_hold, 10)
        node.create_timer(0.2, self.publish_status)
        node.create_timer(0.05, self._publish_hand_target)

    def _set_mode(self, request, response):
        mode = "keyboard" if request.data else "topic"
        if mode != self.input_mode:
            self.stop("Input mode changed. Release F7 and hold again to run.")
            self.input_mode = mode
            # Discard poses from the previous selection; a fresh sample is
            # required, and a held key cannot restart the new source.
            self._target = self._keyboard_target = None
            self._target_dirty = self._keyboard_dirty = False
            self._target_error = "Waiting for a fresh mapped topic pose"
            self._keyboard_error = "Waiting for a keyboard hand target"
            self.node.set_parameters([Parameter("teleop_input_mode", value=mode)])
        response.success = True
        response.message = f"Selected {mode} input. Hold F7 to run."
        self.message = response.message
        self.publish_status()
        return response

    def hold_active(self):
        return (self._hold_latched and self._hold_received is not None
                and time.monotonic() - self._hold_received <= self.hold_timeout)

    def _on_hold(self, message):
        try:
            hold = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if (not isinstance(hold, dict) or not isinstance(hold.get("held"), bool)
                or not isinstance(hold.get("token"), str) or not hold["token"]):
            return
        token = hold["token"]
        if not hold["held"]:
            if token != self._hold_token:
                return
            self._hold_latched = False
            self._hold_received = None
            if self.following:
                self.stop("F7 released. Simulation paused.")
            else:
                self.publish_status()
            return
        first_press = not self._hold_latched or token != self._hold_token
        self._hold_token = token
        self._hold_latched = True
        self._hold_received = time.monotonic()
        # Only a new press starts motion. Heartbeats after a fault cannot
        # restart the controller until the user releases and presses again.
        if first_press:
            self._run(SetBool.Request(data=True), SetBool.Response())

    def _on_mapping(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(status, dict) or status.get("version") != 1:
            return
        if not all(isinstance(status.get(key), bool)
                   for key in ("calibrated", "input_ready", "robot_ready")):
            return
        key = (status.get("session_id"), status.get("input_revision"), status.get("sync_sequence"))
        if key != self._mapping_key:
            if self.following and self.input_mode == "topic":
                self.stop("Mapping changed. Release F7, synchronize, then hold F7 again.")
            self._target = None
            self._target_dirty = False
            self._target_error = "Waiting for a target from the current mapping"
        self._mapping_key = key
        self._mapping = status
        self._mapping_time = time.monotonic()

    def _validated_target(self, message):
        if message.header.frame_id != self.node.pose_frame:
            raise ValueError(f"Hand target must use frame '{self.node.pose_frame}'")
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            raise ValueError("Hand target has no timestamp")
        if abs(self._stamp_age(message.header.stamp)) > self.timeout:
            raise ValueError("Hand target timestamp is stale")
        p, q = message.pose.position, message.pose.orientation
        matrix = transform_from_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        return matrix, message.header.stamp, time.monotonic()

    def _on_keyboard_target(self, message):
        try:
            self._keyboard_target = self._validated_target(message)
        except ValueError as exc:
            self._keyboard_target = None
            self._keyboard_error = str(exc)
            return
        self._keyboard_dirty = True
        self._keyboard_error = ""

    def _on_target(self, message):
        try:
            self._target = self._validated_target(message)
        except ValueError as exc:
            self._target = None
            self._target_error = str(exc)
            return
        self._target_dirty = True
        self._target_error = ""

    def _stamp_age(self, stamp):
        return (self.node.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9

    def ready_reason(self):
        node = self.node
        if node.latest_state is None or time.monotonic() - node.last_state_update_monotonic > self.timeout:
            return "Waiting for fresh simulation joint states"
        if node._below_workspace_floor(node.latest_state[:node.model.nq]):
            return (f"Measured {node.task_frame} is below world Z={node.workspace_min_z:.3f} m. "
                    "F7 is blocked; restore a valid starting posture before running.")
        if self.input_mode == "keyboard":
            if self._keyboard_target is None:
                return self._keyboard_error
            _, stamp, received = self._keyboard_target
            if time.monotonic() - received > self.timeout or abs(self._stamp_age(stamp)) > self.timeout:
                return "Keyboard hand target stopped. Release F7 and hold again when input is ready."
            return ""
        if self._mapping is None or time.monotonic() - self._mapping_time > 1.0:
            return "Waiting for mapper status"
        if not self._mapping["calibrated"]:
            return "Press F5 to synchronize the Vive Tracker first"
        if (self._mapping.get("hand_frame") != self.hand_frame
                or self._mapping.get("world_frame") != node.pose_frame):
            return "Mapper and MPC hand/world frames do not match"
        if not self._mapping["input_ready"] or not self._mapping["robot_ready"]:
            return (self._mapping.get("input_error") or self._mapping.get("robot_error")
                    or "Mapping inputs are unavailable")
        if self._target is None:
            return self._target_error
        _, stamp, received = self._target
        if time.monotonic() - received > self.timeout or abs(self._stamp_age(stamp)) > self.timeout:
            return "Mapped Vive input stopped. Release F7 and hold again when input is ready."
        return ""

    def _run(self, request, response):
        if not request.data:
            self.stop("Simulation paused. Hold F7 to follow the hand target.")
            response.success, response.message = True, self.message
        else:
            reason = ("Hold F7 to run the simulation." if not self.hold_active()
                      else self.ready_reason())
            if reason:
                self.message = reason
                response.success, response.message = False, reason
            elif self.following:
                response.success, response.message = True, "Simulation is already following the hand target"
            else:
                self.node._set_execution_enabled(True)
                self.following = True
                self._target_dirty = True
                self._keyboard_dirty = True
                self.message = f"Following {self.input_mode} hand target with {self.hand_frame}. Release F7 to pause."
                response.success, response.message = True, self.message
        self.publish_status()
        return response

    def stop(self, reason):
        self.following = False
        self.node._set_execution_enabled(False)
        self.message = reason
        self.publish_status()

    def before_solve(self):
        if not self.following:
            return
        if not self.hold_active():
            self.stop("F7 hold input timed out. Release F7 and hold again to resume.")
            return
        reason = self.ready_reason()
        if reason:
            self.stop(reason)
            return
        dirty = self._keyboard_dirty if self.input_mode == "keyboard" else self._target_dirty
        if not dirty:
            return
        if self.input_mode == "keyboard":
            self._keyboard_dirty = False
            target = self._keyboard_target
        else:
            self._target_dirty = False
            target = self._target
        matrix, stamp, _ = target
        current_target = self.node.target_pose.homogeneous
        if np.allclose(matrix, current_target, rtol=0, atol=1e-10):
            return
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = self.node.pose_frame, stamp
        position, quaternion = pose_from_transform(matrix)
        (message.pose.position.x, message.pose.position.y,
         message.pose.position.z) = map(float, position)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, quaternion)
        self.node._apply_target_pose(message)

    def _publish_hand_target(self):
        if self.ready_reason():
            return
        matrix, stamp, _ = (self._keyboard_target if self.input_mode == "keyboard" else self._target)
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = self.node.pose_frame, stamp
        position, quaternion = pose_from_transform(matrix)
        (message.pose.position.x, message.pose.position.y,
         message.pose.position.z) = map(float, position)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, quaternion)
        self.target_pub.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, position)
        transform.transform.rotation = message.pose.orientation
        self._broadcaster.sendTransform(transform)

    def publish_status(self):
        reason = self.ready_reason()
        self.status_pub.publish(String(data=json.dumps({
            "version": 1,
            "runtime_id": self.runtime_id,
            "simulation": True,
            "following": self.following,
            "execution_enabled": self.node.execution_enabled,
            "hold_active": self.hold_active(),
            "hold_token": self._hold_token,
            "hold_timeout": self.hold_timeout,
            "input_mode": self.input_mode,
            "world_frame": self.node.pose_frame,
            "hand_target_frame": self.target_frame,
            "hand_target_topic": self.target_pub.topic_name,
            "can_run": not reason,
            "reason": reason,
            "message": self.message,
            "solve_count": self.node.solve_count,
            "task_frame": self.node.task_frame,
            "hand_frame": self.hand_frame,
            "joint_command_topic": self.node.joint_command_pub.topic_name,
            "workspace_floor_enabled": self.node.workspace_floor_enabled,
            "workspace_min_z": self.node.workspace_min_z,
            "workspace_floor_margin": self.node.workspace_floor_margin,
        })))
