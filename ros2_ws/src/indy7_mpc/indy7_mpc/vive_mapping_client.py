"""Nonblocking ROS client shared by standalone and embedded mapping panels."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.exceptions import InvalidTopicNameException
from rclpy.parameter import Parameter
from rclpy.validate_full_topic_name import validate_full_topic_name
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .pose_mapping import pose_from_transform, transform_from_pose


class ViveMappingClient:
    @property
    def is_closed(self):
        with self._lock:
            return self._closed

    def __init__(self, node, request_timeout=3.0):
        self.node = node
        self._lock = threading.RLock()
        self._status = None
        self._status_time = 0.0
        self._current = self._mapped = None
        self._simulation_current = self._hand_target = None
        self._keyboard_pose = None
        self._requested_mode = self._awaiting_mode = None
        self._simulation = {}
        self._simulation_time = 0.0
        self._run_held = False
        self._hold_started = 0.0
        self._hold_token = None
        self._closed = False
        self._future = None
        self._future_client = None
        self._operation = None
        self._requested_topic = None
        self._awaiting_topic = None
        self._request_time = 0.0
        self._request_timeout = request_timeout
        self._result = "Hold both poses still, then click Map or press F5."
        self._result_kind = "idle"
        defaults = {
            "vive_mapping_status_topic": "/indy7/vive_mapping/status",
            "vive_mapping_sync_service": "/indy7/vive_mapping/sync",
            "vive_mapping_set_parameters_service": "/indy7_vive_mapper/set_parameters_atomically",
            "vive_mapping_current_pose_topic": "/indy7/hand_tcp/current_pose",
            "vive_mapping_target_pose_topic": "/indy7/vive/hand_target_pose",
            "vive_simulation_run_service": "/indy7/sim/vive_mpc_run",
            "vive_simulation_status_topic": "/indy7/sim/vive_mpc_status",
            "vive_simulation_hold_topic": "/indy7/sim/vive_mpc_hold",
            "teleop_mode_service": "/indy7/sim/keyboard_mode",
            "keyboard_hand_pose_topic": "/indy7/keyboard/hand_target_pose",
            "hand_target_pose_topic": "/indy7/hand_target_pose",
            "hand_simulation_current_pose_topic": "/mpc/current_pose",
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value)

        def param(name):
            return str(node.get_parameter(name).value)

        self.service = node.create_client(Trigger, param("vive_mapping_sync_service"))
        self.topic_service = node.create_client(
            SetParametersAtomically, param("vive_mapping_set_parameters_service"))
        self.run_service = node.create_client(SetBool, param("vive_simulation_run_service"))
        self.mode_service = node.create_client(SetBool, param("teleop_mode_service"))
        self.keyboard_pub = node.create_publisher(PoseStamped, param("keyboard_hand_pose_topic"), 10)
        # Keep release/press edges even when the user represses before the
        # executor runs; depth 1 can replace a queued false with the next true.
        self.hold_pub = node.create_publisher(String, param("vive_simulation_hold_topic"), 10)
        node.create_subscription(String, param("vive_simulation_status_topic"), self._on_simulation, 10)
        self._output_topics = {param("vive_mapping_current_pose_topic"),
                               param("vive_mapping_target_pose_topic"),
                               param("keyboard_hand_pose_topic"), param("hand_target_pose_topic"),
                               param("hand_simulation_current_pose_topic")}
        node.create_subscription(PoseStamped, param("hand_simulation_current_pose_topic"),
                                 lambda msg: self._on_pose("_simulation_current", msg), 10)
        node.create_subscription(PoseStamped, param("hand_target_pose_topic"),
                                 lambda msg: self._on_pose("_hand_target", msg), 10)
        node.create_subscription(String, param("vive_mapping_status_topic"), self._on_status, 10)
        node.create_subscription(PoseStamped, param("vive_mapping_current_pose_topic"),
                                 lambda msg: self._on_pose("_current", msg), 10)
        node.create_subscription(PoseStamped, param("vive_mapping_target_pose_topic"),
                                 lambda msg: self._on_pose("_mapped", msg), 10)
        self._timeout_timer = node.create_timer(0.1, self._check_timeout)

    def _on_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(status, dict) or status.get("version") != 1:
            return
        if not all(isinstance(status.get(key), bool)
                   for key in ("can_sync", "calibrated", "input_ready", "robot_ready")):
            return
        with self._lock:
            if self._status and status.get("session_id") != self._status.get("session_id"):
                self._cancel_pending()
                self._current = self._mapped = None
                self._result = "Mapper restarted. Hold both poses still and synchronize again."
                self._result_kind = "idle"
            elif self._status and (
                    status.get("input_revision") != self._status.get("input_revision")
                    or status.get("input_topic") != self._status.get("input_topic")):
                self._mapped = None
                if self._operation != "topic":
                    self._cancel_pending()
                    self._result = "Vive input topic changed. Wait for input, then press F5."
                    self._result_kind = "idle"
            self._status = status
            self._status_time = time.monotonic()
            self._confirm_topic()

    def _on_pose(self, key, message):
        p, q = message.pose.position, message.pose.orientation
        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in pose) or sum(v * v for v in pose[3:]) < 1e-12:
            return
        with self._lock:
            setattr(self, key, (pose, message.header.frame_id, time.monotonic()))

    def _on_simulation(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if (not isinstance(status, dict) or status.get("version") != 1
                or status.get("simulation") is not True
                or not all(isinstance(status.get(key), bool)
                           for key in ("following", "execution_enabled", "can_run"))):
            return
        with self._lock:
            if (self._simulation.get("runtime_id") and status.get("runtime_id")
                    != self._simulation.get("runtime_id")):
                self._run_held = False
                self._publish_hold(False)
                self._cancel_pending()
                self._keyboard_pose = None
                self._result = "Simulation MPC restarted. Release F7 and prepare the target again."
                self._result_kind = "idle"
            owns_hold = status.get("hold_token") == self._hold_token
            if self._run_held and owns_hold and not status["following"] and (
                    status.get("hold_active") or self._simulation.get("following")):
                self._run_held = False
                self._publish_hold(False)
                self._result = status.get("message", "Simulation paused. Release F7 and hold again.")
                self._result_kind = "idle"
            elif self._run_held and owns_hold and status["following"]:
                self._result = "Following the hand target while held. Release F7 to pause."
                self._result_kind = "success"
            if (self._simulation.get("following") and not status["following"]
                    and self._operation not in ("run", "stop_run")):
                self._result = status.get("message", "Simulation paused.")
                self._result_kind = "idle"
            if status.get("input_mode") != self._simulation.get("input_mode"):
                self._keyboard_pose = None
            self._simulation = status
            self._simulation_time = time.monotonic()
            self._confirm_mode()

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            status = dict(self._status or {})
            fresh = self._status is not None and now - self._status_time < 1.0
            service_ready = not self._closed and self.service.service_is_ready()
            topic_service_ready = not self._closed and self.topic_service.service_is_ready()
            simulation_fresh = bool(self._simulation and now - self._simulation_time < 1.0)
            running = bool(self._simulation.get("following"))
            starting = self._run_held and not running
            pending = (self._future is not None or self._awaiting_topic is not None
                       or self._awaiting_mode is not None or starting)
            run_ready = not self._closed and self.run_service.service_is_ready()
            mode_ready = not self._closed and self.mode_service.service_is_ready()
            return {
                "status": status,
                "status_fresh": fresh,
                "service_ready": service_ready,
                "topic_service_ready": topic_service_ready,
                "pending": pending,
                "operation": self._operation or ("run" if starting else None),
                "run_held": self._run_held,
                "input_mode": self._simulation.get("input_mode", "topic"),
                "can_change_mode": bool(simulation_fresh and mode_ready and not pending and not running
                                        and not self._run_held and not self._simulation.get("hold_active")),
                "can_sync": bool(fresh and service_ready and status.get("can_sync")
                                 and not pending and not running),
                "can_apply_topic": bool(fresh and topic_service_ready and not pending and not running),
                "simulation": dict(self._simulation),
                "simulation_fresh": simulation_fresh,
                "simulation_running": running,
                "can_run_simulation": bool(simulation_fresh and run_ready and not pending
                                           and self._simulation.get("can_run") and not running),
                "can_stop_simulation": bool(run_ready and (running or self._run_held
                                                           or self._operation == "run")),
                "current": self._current if self._current and now - self._current[2] < 0.75 else None,
                "mapped": self._mapped if self._mapped and now - self._mapped[2] < 0.75 else None,
                "simulation_current": (self._simulation_current if self._simulation_current
                                       and now - self._simulation_current[2] < 0.5 else None),
                "hand_target": (self._hand_target if self._hand_target
                                and now - self._hand_target[2] < 0.75 else None),
                "keyboard_pose": None if self._keyboard_pose is None else list(self._keyboard_pose),
                "result": self._result,
                "result_kind": self._result_kind,
            }

    def request_sync(self):
        """Return immediately; the executor processes the service response."""
        with self._lock:
            if self._future is not None or self._awaiting_topic is not None:
                return False
            state = self.snapshot()
            if not state["can_sync"]:
                status = state["status"]
                if not state["status_fresh"]:
                    message = "Waiting for mapper status. Start the Vive mapping node."
                elif state["simulation_running"]:
                    message = "Release F7 before synchronizing a new mapping."
                elif not state["service_ready"]:
                    message = "Mapping service is unavailable."
                else:
                    message = (status.get("input_error") or status.get("robot_error")
                               or status.get("sync_error") or "Mapping inputs are not ready.")
                self._result, self._result_kind = message, "error"
                return False
            self._request_time = time.monotonic()
            self._operation = "sync"
            self._result, self._result_kind = "Synchronizing...", "pending"
            try:
                self._future_client = self.service
                self._future = self.service.call_async(Trigger.Request())
            except Exception as exc:  # ROS transport exceptions vary by middleware.
                self._operation = self._future_client = None
                self._result, self._result_kind = f"Mapping service error: {exc}", "error"
                return False
            self._future.add_done_callback(self._complete)
            return True

    def pose_topics(self):
        return sorted(name for name, types in self.node.get_topic_names_and_types()
                      if "geometry_msgs/msg/PoseStamped" in types and name not in self._output_topics)

    def request_mode(self, mode):
        with self._lock:
            state = self.snapshot()
            if mode not in ("keyboard", "topic") or not state["can_change_mode"]:
                self._result, self._result_kind = "Release F7 and wait for the MPC before changing input mode.", "error"
                return False
            if mode == state["input_mode"]:
                return False
            self._requested_mode = mode
            self._request_time = time.monotonic()
            self._operation = "mode"
            self._result, self._result_kind = "Changing input mode...", "pending"
            try:
                self._future_client = self.mode_service
                self._future = self.mode_service.call_async(SetBool.Request(data=mode == "keyboard"))
            except Exception as exc:
                self._operation = self._future_client = None
                self._result, self._result_kind = f"Input mode error: {exc}", "error"
                return False
            self._future.add_done_callback(self._complete)
            return True

    def _confirm_mode(self):
        if self._awaiting_mode and self._simulation.get("input_mode") == self._awaiting_mode:
            self._awaiting_mode = self._operation = None
            self._result = "Input mode selected. Hold F7 to run the hand target."
            self._result_kind = "success"

    def reset_keyboard_target(self):
        with self._lock:
            state = self.snapshot()
            current = state["simulation_current"]
            if (state["input_mode"] != "keyboard" or current is None
                    or state["run_held"] or state["simulation_running"]
                    or current[1] != state["simulation"].get("world_frame", "world")):
                return False
            self._keyboard_pose = list(current[0])
            return True

    def set_keyboard_target(self, pose):
        with self._lock:
            if self._closed or self._simulation.get("input_mode") != "keyboard":
                return False
            position, quaternion = pose_from_transform(transform_from_pose(pose[:3], pose[3:]))
            self._keyboard_pose = list(map(float, position)) + list(map(float, quaternion))
            return True

    def publish_keyboard_target(self):
        """Preview/update the GUI hand target even while F7 is released."""
        with self._lock:
            state = self.snapshot()
            if self._closed or not state["simulation_fresh"] or state["input_mode"] != "keyboard":
                return
            if self._keyboard_pose is None:
                self.reset_keyboard_target()
            if self._keyboard_pose is None:
                return
            message = PoseStamped()
            message.header.frame_id = state["simulation"].get("world_frame", "world")
            message.header.stamp = self.node.get_clock().now().to_msg()
            pose = self._keyboard_pose
            message.pose.position.x, message.pose.position.y, message.pose.position.z = pose[:3]
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = pose[3:]
            self.keyboard_pub.publish(message)

    def request_run(self, enabled):
        with self._lock:
            if self._closed:
                return False
            state = self.snapshot()
            if enabled and not state["can_run_simulation"]:
                self._result = (state["simulation"].get("reason")
                                or "Simulation MPC is unavailable or a request is pending.")
                self._result_kind = "error"
                return False
            if enabled:
                self._run_held = True
                self._hold_token = uuid.uuid4().hex
                self._hold_started = time.monotonic()
                self._result, self._result_kind = "Starting simulation while held...", "pending"
                self._publish_hold(True)
                return True
            self._run_held = False
            self._publish_hold(False)
            if not self.run_service.service_is_ready():
                self._result, self._result_kind = "Simulation run service is unavailable.", "error"
                return False
            self._cancel_pending()  # A stop takes priority over an in-flight request.
            self._operation = "stop_run"
            self._request_time = time.monotonic()
            self._result, self._result_kind = "Pausing simulation...", "pending"
            try:
                self._future_client = self.run_service
                self._future = self.run_service.call_async(SetBool.Request(data=False))
            except Exception as exc:
                self._operation = self._future_client = None
                self._result, self._result_kind = f"Simulation service error: {exc}", "error"
                return False
            self._future.add_done_callback(self._complete)
            return True

    def publish_run_hold(self):
        """Called by the Qt timer so a frozen/closed GUI cannot keep motion alive."""
        with self._lock:
            if self._run_held and not self._closed:
                self._publish_hold(True)

    def _publish_hold(self, held):
        if self._hold_token is not None:
            self.hold_pub.publish(String(data=json.dumps({"held": held, "token": self._hold_token})))

    def request_topic(self, topic):
        """Apply a live input subscription change through the mapper parameter service."""
        with self._lock:
            if self._future is not None or self._awaiting_topic is not None:
                return False
            topic = topic.strip()
            try:
                validate_full_topic_name(topic)
            except InvalidTopicNameException as exc:
                self._result, self._result_kind = str(exc), "error"
                return False
            state = self.snapshot()
            if not state["can_apply_topic"]:
                self._result = ("Release F7 before applying a new topic." if state["simulation_running"]
                                else "Waiting for mapper status and its topic configuration service.")
                self._result_kind = "error"
                return False
            if topic == state["status"].get("input_topic"):
                self._result = "Input topic is already active. Existing mapping was preserved."
                self._result_kind = "idle"
                return False
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter("input_pose_topic", value=topic).to_parameter_msg()]
            self._requested_topic = topic
            self._request_time = time.monotonic()
            self._operation = "topic"
            self._result, self._result_kind = "Applying Vive input topic...", "pending"
            try:
                self._future_client = self.topic_service
                self._future = self.topic_service.call_async(request)
            except Exception as exc:
                self._operation = self._future_client = None
                self._result, self._result_kind = f"Topic update error: {exc}", "error"
                return False
            self._future.add_done_callback(self._complete)
            return True

    def _confirm_topic(self):
        if self._awaiting_topic and self._status.get("input_topic") == self._awaiting_topic:
            self._awaiting_topic = None
            self._operation = None
            self._result = "Vive input topic applied. Wait for input, then press F5 to synchronize."
            self._result_kind = "success"

    def _complete(self, future):
        with self._lock:
            if future is not self._future:
                return
            self._future = None
            self._future_client = None
            try:
                response = future.result()
                if self._operation == "mode":
                    if response.success:
                        self._awaiting_mode = self._requested_mode
                        self._result = "Input mode changed; waiting for MPC status..."
                        self._confirm_mode()
                    else:
                        self._operation = None
                        self._result_kind, self._result = "error", response.message
                    return
                if self._operation == "topic":
                    if response.result.successful:
                        self._mapped = None
                        self._awaiting_topic = self._requested_topic
                        self._result = "Topic applied; waiting for mapper status..."
                        self._confirm_topic()
                    else:
                        self._operation = None
                        self._result_kind = "error"
                        self._result = f"Topic update rejected: {response.result.reason}"
                    return
                operation = self._operation
                self._operation = None
                self._result_kind = "success" if response.success else "error"
                if operation in ("run", "stop_run"):
                    prefix = "Simulation" if response.success else "Simulation request rejected"
                else:
                    prefix = "Synchronized" if response.success else "Synchronization rejected"
                self._result = f"{prefix}: {response.message}"
            except Exception as exc:
                self._operation = None
                self._result, self._result_kind = f"Mapping service error: {exc}", "error"

    def _cancel_pending(self):
        future, self._future = self._future, None
        service, self._future_client = self._future_client, None
        self._operation = self._awaiting_topic = self._awaiting_mode = None
        if future is not None:
            service.remove_pending_request(future)
            future.cancel()

    def _check_timeout(self):
        with self._lock:
            if (self._run_held and not self._simulation.get("following")
                    and time.monotonic() - self._hold_started > self._request_timeout):
                self.request_run(False)
                self._result = "Simulation start timed out. Release F7 and hold again."
                self._result_kind = "error"
            if ((self._future is not None or self._awaiting_topic is not None or self._awaiting_mode is not None)
                    and time.monotonic() - self._request_time > self._request_timeout):
                operation = {"topic": "Topic update", "mode": "Input mode update", "run": "Simulation request",
                             "stop_run": "Simulation request"}.get(self._operation, "Mapping request")
                self._cancel_pending()
                self._result = f"{operation} timed out. Check mapper status before retrying."
                self._result_kind = "error"

    def close(self):
        with self._lock:
            if self._closed:
                return
            held, self._run_held = self._run_held, False
            self._publish_hold(False)
            if ((self._simulation.get("following") or held or self._operation == "run")
                    and self.run_service.service_is_ready()):
                self.run_service.call_async(SetBool.Request(data=False))
            self._timeout_timer.cancel()
            self._cancel_pending()
            self._closed = True
