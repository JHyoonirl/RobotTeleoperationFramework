"""ROS bridge shared by the legacy and minimal Indy7 teleoperation GUIs."""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool

from .joint_mapping import (
    load_mapping,
    map_robot_to_sim,
    map_sim_to_robot,
    save_mapping,
    validate_mapping,
)

SIM_JOINTS = [f"joint{i}" for i in range(6)]
ROBOT_JOINTS = [f"joint{i}" for i in range(6)]


class TeleopRosNode(Node):
    """ROS bridge; publishers remain idle until the GUI explicitly enables them."""

    def __init__(self, node_name="indy7_teleop_gui", **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter("simulation_joint_state_topic", "/joint_states")
        self.declare_parameter("robot_joint_state_topic", "/indy7_dcp3/joint_states")
        self.declare_parameter("robot_command_topic", "/indy7_dcp3/joint_position_cmd")
        self.declare_parameter("deadman_topic", "/indy7_dcp3/deadman")
        self.declare_parameter("arm_service", "/indy7_dcp3/arm")
        self.declare_parameter("sim_sync_topic", "/indy7/sim/sync_joint_target")
        self.declare_parameter("sim_sync_ack_topic", "/indy7/sim/sync_joint_target/status")
        self.declare_parameter("robot_status_topic", "/indy7_dcp3/status")
        self.declare_parameter("robot_armed_topic", "/indy7_dcp3/armed")
        self.declare_parameter("current_pose_topic", "/mpc/current_pose")
        self.declare_parameter("desired_pose_topic", "/mpc/desired_pose")
        self.declare_parameter("target_pose_topic", "/teleop/target_pose")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("joint_command_rate", 200.0)
        self.sim_topic = str(self.get_parameter("simulation_joint_state_topic").value)
        self.robot_topic = str(self.get_parameter("robot_joint_state_topic").value)
        self.command_topic = str(self.get_parameter("robot_command_topic").value)
        self.deadman_topic = str(self.get_parameter("deadman_topic").value)
        self.sync_topic = str(self.get_parameter("sim_sync_topic").value)
        self.status_topic = str(self.get_parameter("robot_status_topic").value)
        self.sync_ack_topic = str(self.get_parameter("sim_sync_ack_topic").value)
        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.pose_topic = str(self.get_parameter("desired_pose_topic").value)
        self.target_topic = str(self.get_parameter("target_pose_topic").value)
        self.joint_command_rate = float(self.get_parameter("joint_command_rate").value)
        if not math.isfinite(self.joint_command_rate) or self.joint_command_rate <= 0.0:
            raise ValueError("joint_command_rate must be positive and finite")
        self.joint_command_period = 1.0 / self.joint_command_rate
        mapping_file = str(self.get_parameter("mapping_file").value).strip()
        if not mapping_file:
            mapping_file = os.environ.get(
                "INDY7_TELEOP_MAPPING",
                str(Path.home() / ".config/robot-teleoperation/indy7_joint_mapping.yaml"),
            )
        self.mapping_path = Path(mapping_file).expanduser()
        self.lock = threading.RLock()
        self.state = {
            "sim_q": None,
            "sim_time": 0.0,
            "robot_q": None,
            "robot_time": 0.0,
            "robot_status": "disarmed",
            "robot_status_time": 0.0,
            "armed": False,
            "current_pose": None,
            "current_pose_time": 0.0,
            "desired_pose": None,
            "desired_time": 0.0,
            "sync_result": "",
            "sync_time": 0.0,
            "service_result": "",
            "service_time": 0.0,
            "stream_enabled": False,
            "deadman_held": False,
            "deadman_gui_lease": False,
            "deadman_gui_time": 0.0,
            "sync_pending": False,
        }
        self.mapping = load_mapping(self.mapping_path, ROBOT_JOINTS, SIM_JOINTS)
        self._managed_output = False
        self._managed_follow = False
        self._managed_hold = None
        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.deadman_pub = self.create_publisher(Bool, self.deadman_topic, 10)
        self.sync_pub = self.create_publisher(JointState, self.sync_topic, 10)
        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.create_subscription(JointState, self.sim_topic, self._on_sim_state, 10)
        self.create_subscription(JointState, self.robot_topic, self._on_robot_state, 10)
        self.create_subscription(PoseStamped, self.current_pose_topic, self._on_current_pose, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self._on_desired_pose, 10)
        self.create_subscription(String, self.status_topic, self._on_robot_status, 10)
        self.create_subscription(Bool, str(self.get_parameter("robot_armed_topic").value), self._on_armed, 10)
        self.create_subscription(String, self.sync_ack_topic, self._on_sync_ack, 10)
        self.arm_client = self.create_client(SetBool, str(self.get_parameter("arm_service").value))
        self.pending_sync_id = ""
        # Stream from the ROS bridge's own timer, independent of Qt repaint rate.
        self._stream_timer = self.create_timer(self.joint_command_period, self._stream_tick)
        self.create_timer(0.10, self._deadman_tick)

    @staticmethod
    def _ordered_positions(message: JointState, names: list[str]) -> list[float] | None:
        if message.name:
            index = {name: i for i, name in enumerate(message.name)}
            if any(name not in index or index[name] >= len(message.position) for name in names):
                return None
            values = [float(message.position[index[name]]) for name in names]
        elif len(message.position) >= len(names):
            values = [float(value) for value in message.position[: len(names)]]
        else:
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    def _on_sim_state(self, message: JointState) -> None:
        values = self._ordered_positions(message, SIM_JOINTS)
        if values is not None:
            with self.lock:
                self.state["sim_q"] = values
                self.state["sim_time"] = time.monotonic()

    def _on_robot_state(self, message: JointState) -> None:
        values = self._ordered_positions(message, ROBOT_JOINTS)
        if values is not None:
            with self.lock:
                self.state["robot_q"] = values
                self.state["robot_time"] = time.monotonic()

    def _on_desired_pose(self, message: PoseStamped) -> None:
        self._store_pose(message, "desired_pose", "desired_time")

    def _on_current_pose(self, message: PoseStamped) -> None:
        self._store_pose(message, "current_pose", "current_pose_time")

    def _store_pose(self, message: PoseStamped, pose_key: str, time_key: str) -> None:
        p, q = message.pose.position, message.pose.orientation
        pose = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if all(math.isfinite(value) for value in pose) and sum(qv * qv for qv in pose[3:]) > 1.0e-12:
            with self.lock:
                self.state[pose_key] = pose
                self.state[time_key] = time.monotonic()

    def _on_robot_status(self, message: String) -> None:
        with self.lock:
            self.state["robot_status"] = message.data
            self.state["robot_status_time"] = time.monotonic()

    def _on_armed(self, message: Bool) -> None:
        with self.lock:
            self.state["armed"] = bool(message.data)

    def _on_sync_ack(self, message: String) -> None:
        try:
            result = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(result, dict):
            return
        with self.lock:
            if result.get("request_id") == self.pending_sync_id:
                self.state["sync_pending"] = False
                self.state["sync_result"] = str(result.get("message", ""))
                self.state["sync_time"] = time.monotonic()
                self.state["sync_success"] = bool(result.get("success", False))

    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.state)

    def set_mapping(self, mapping: dict) -> None:
        clean = validate_mapping(mapping, SIM_JOINTS, ROBOT_JOINTS)
        with self.lock:
            self.mapping = clean
        save_mapping(self.mapping_path, clean)

    def set_stream(self, enabled: bool) -> None:
        with self.lock:
            self.state["stream_enabled"] = bool(enabled)
            if not enabled:
                self.state["deadman_held"] = False
                self._managed_output = self._managed_follow = False
                self._managed_hold = None
        if not enabled:
            self.deadman_pub.publish(Bool(data=False))

    def set_deadman(self, held: bool) -> None:
        with self.lock:
            self.state["deadman_gui_lease"] = False
            self.state["deadman_held"] = bool(held and self.state["stream_enabled"])
            active = self.state["deadman_held"]
        self.deadman_pub.publish(Bool(data=active))

    def set_gui_deadman(self, held: bool) -> None:
        """F6 sessions renew this from Qt, so a frozen GUI loses the lease."""
        with self.lock:
            self.state["deadman_gui_lease"] = True
            self.state["deadman_gui_time"] = time.monotonic()
            self.state["deadman_held"] = bool(held and self.state["stream_enabled"])
            active = self.state["deadman_held"]
        self.deadman_pub.publish(Bool(data=active))

    def set_managed_follow(self, following: bool) -> None:
        with self.lock:
            if not self._managed_output or (self._managed_follow and not following):
                self._managed_hold = (None if self.state["robot_q"] is None
                                      else list(self.state["robot_q"]))
            self._managed_output = True
            self._managed_follow = bool(following)

    def _stream_tick(self) -> None:
        with self.lock:
            if not self.state["stream_enabled"] or self.state["sim_q"] is None:
                return
            if time.monotonic() - self.state["sim_time"] > 0.3:
                return
            sim_q = list(self.state["sim_q"])
            mapping = self.mapping
            if self._managed_output and not self._managed_follow:
                if self._managed_hold is None or time.monotonic() - self.state["robot_time"] > 0.3:
                    return
                self.command_pub.publish(Float64MultiArray(data=list(self._managed_hold)))
                return
        try:
            command = map_sim_to_robot(sim_q, SIM_JOINTS, ROBOT_JOINTS, mapping)
        except (KeyError, TypeError, ValueError):
            return
        self.command_pub.publish(Float64MultiArray(data=command))

    def _deadman_tick(self) -> None:
        with self.lock:
            held = bool(self.state["deadman_held"] and self.state["stream_enabled"])
            expired = bool(held and self.state["deadman_gui_lease"]
                           and time.monotonic() - self.state["deadman_gui_time"] > 0.2)
            if expired:
                self.state["deadman_held"] = held = False
        if held:
            self.deadman_pub.publish(Bool(data=True))
        elif expired:
            self.deadman_pub.publish(Bool(data=False))

    def arm(self, armed: bool):
        if not self.arm_client.service_is_ready():
            with self.lock:
                self.state["service_result"] = "DCP3 arm service is unavailable."
                self.state["service_time"] = time.monotonic()
            return
        request = SetBool.Request()
        request.data = bool(armed)
        future = self.arm_client.call_async(request)

        def complete(result_future):
            try:
                response = result_future.result()
                message = response.message if response.success else f"Request rejected: {response.message}"
            except Exception as exc:  # noqa: BLE001 - ROS service transport exceptions vary.
                message = f"Arm service error: {exc}"
            with self.lock:
                self.state["service_result"] = message
                self.state["service_time"] = time.monotonic()

        future.add_done_callback(complete)
        return future

    def sync_robot_to_sim(self) -> str:
        with self.lock:
            robot_q = self.state["robot_q"]
            robot_age = time.monotonic() - self.state["robot_time"]
            mapping = dict(self.mapping)
        if robot_q is None or robot_age > 0.5:
            raise ValueError("Robot joint state is unavailable or stale.")
        sim_q = map_robot_to_sim(robot_q, SIM_JOINTS, ROBOT_JOINTS, mapping)
        request_id = uuid.uuid4().hex
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = request_id
        message.name = SIM_JOINTS
        message.position = sim_q
        with self.lock:
            self.pending_sync_id = request_id
            self.state["sync_result"] = "Simulation sync request sent; waiting for MPC response."
            self.state["sync_success"] = False
            self.state["sync_pending"] = True
            self.state["sync_time"] = time.monotonic()
        self.sync_pub.publish(message)
        return request_id

    def publish_target(self, pose: list[float]) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose[:3]
        message.pose.orientation.x, message.pose.orientation.y = pose[3:5]
        message.pose.orientation.z, message.pose.orientation.w = pose[5:7]
        self.target_pub.publish(message)

