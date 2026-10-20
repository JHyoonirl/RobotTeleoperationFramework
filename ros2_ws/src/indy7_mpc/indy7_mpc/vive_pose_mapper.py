"""Publish a synchronized Vive hand target in the Indy7 world frame."""

import json
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import InvalidTopicNameException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.validate_full_topic_name import validate_full_topic_name
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .pose_mapping import align_worlds, pose_from_transform, transform_from_pose


def _matrix(position, orientation):
    return transform_from_pose(
        [position.x, position.y, position.z],
        [orientation.x, orientation.y, orientation.z, orientation.w],
    )


def _fill_pose(pose, matrix):
    position, quaternion = pose_from_transform(matrix)
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = map(float, quaternion)


class VivePoseMapper(Node):
    def __init__(self, **kwargs):
        super().__init__("indy7_vive_mapper", **kwargs)
        defaults = {
            "input_pose_topic": "/tracker/right/pose",
            "mapped_pose_topic": "/indy7/vive/hand_target_pose",
            "current_hand_pose_topic": "/indy7/hand_tcp/current_pose",
            "sync_service": "/indy7/vive_mapping/sync",
            "status_topic": "/indy7/vive_mapping/status",
            "world_frame": "world",
            "hand_frame": "hand_tcp",
            # The Vive driver currently labels its independent world as 'world'.
            # Treat that as a source label; never look up world -> world in TF.
            "input_frame": "world",
            "vive_world_frame": "vive_world",
            "mapped_hand_frame": "vive_hand_target",
            "input_timeout": 0.5,
            "robot_tf_timeout": 0.5,
            "publish_tf": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        def param(name):
            return self.get_parameter(name).value

        self.world_frame = str(param("world_frame")).lstrip("/")
        self.hand_frame = str(param("hand_frame")).lstrip("/")
        self.input_frame = str(param("input_frame")).lstrip("/")
        self.vive_frame = str(param("vive_world_frame")).lstrip("/")
        self.mapped_frame = str(param("mapped_hand_frame")).lstrip("/")
        frames = [self.world_frame, self.hand_frame, self.vive_frame, self.mapped_frame]
        if not all(frames) or len(set(frames)) != len(frames) or not self.input_frame:
            raise ValueError("TF frame names must be nonempty and distinct")
        self.input_timeout = float(param("input_timeout"))
        self.tf_timeout = float(param("robot_tf_timeout"))
        if not (0 < self.input_timeout < float("inf") and 0 < self.tf_timeout < float("inf")):
            raise ValueError("Pose and TF timeouts must be positive and finite")
        self.publish_tf = bool(param("publish_tf"))
        self.input_topic = str(param("input_pose_topic"))
        validate_full_topic_name(self.input_topic)
        self._input_revision = 0
        self._latest_input = None
        self._input_error = ""
        self.world_from_vive = None
        self._session_id = uuid.uuid4().hex
        self._sync_sequence = 0
        self._warnings = {}
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        self._mapped_pub = self.create_publisher(
            PoseStamped, str(param("mapped_pose_topic")), 10)
        self._current_pub = self.create_publisher(
            PoseStamped, str(param("current_hand_pose_topic")), 10)
        self._status_pub = self.create_publisher(String, str(param("status_topic")), 10)
        validated = self._validate_parameters([self.get_parameter("input_pose_topic")])
        if not validated.successful:
            raise ValueError(validated.reason)
        self._input_sub = self._subscribe_input(self.input_topic, self._input_revision)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)
        self.create_service(Trigger, str(param("sync_service")), self._sync)
        self.create_timer(0.05, self._publish_current)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f"Mapping {param('input_pose_topic')} onto {self.hand_frame} in {self.world_frame}. "
            f"Hold both poses still and call {param('sync_service')} to synchronize."
        )

    def _subscribe_input(self, topic, revision):
        def receive(message):
            # A callback already queued by the old subscription must not enter
            # the new mapping after a topic switch.
            if revision == self._input_revision:
                self._on_pose(message)

        return self.create_subscription(PoseStamped, topic, receive, 10)

    def _validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != "input_pose_topic":
                continue
            if parameter.type_ != Parameter.Type.STRING:
                return SetParametersResult(successful=False, reason="Input topic must be a string")
            try:
                validate_full_topic_name(parameter.value)
                resolved = self.resolve_topic_name(parameter.value)
                if resolved in (self._mapped_pub.topic_name, self._current_pub.topic_name,
                                "/indy7/hand_target_pose", "/indy7/keyboard/hand_target_pose",
                                "/mpc/current_pose", "/mpc/desired_pose"):
                    raise ValueError("A teleoperation output cannot be used as pose input")
                for name, types in self.get_topic_names_and_types():
                    if name == resolved and "geometry_msgs/msg/PoseStamped" not in types:
                        raise ValueError("Input topic must publish geometry_msgs/msg/PoseStamped")
            except (InvalidTopicNameException, ValueError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _apply_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != "input_pose_topic" or parameter.value == self.input_topic:
                continue
            revision = self._input_revision + 1
            subscription = self._subscribe_input(parameter.value, revision)
            previous = self._input_sub
            self._input_sub = subscription
            self.input_topic = parameter.value
            self._input_revision = revision
            self._latest_input = None
            self._input_error = ""
            self.world_from_vive = None
            self.destroy_subscription(previous)
            self.get_logger().info(
                f"Vive input changed to {self.input_topic}; mapping cleared. Synchronize again.")
            self._publish_status()

    def _warn(self, key, message):
        now = time.monotonic()
        if now - self._warnings.get(key, -float("inf")) >= 5:
            self._warnings[key] = now
            self.get_logger().warning(message)

    def _stamp_age(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise ValueError("Vive pose must have a nonzero timestamp")
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9

    def _on_pose(self, message):
        if message.header.frame_id.lstrip("/") != self.input_frame:
            self._input_error = f"Expected Vive frame '{self.input_frame}', got '{message.header.frame_id}'"
            self._warn("input_frame", self._input_error)
            return
        try:
            if abs(self._stamp_age(message.header.stamp)) > self.input_timeout:
                raise ValueError("Vive pose timestamp is stale or ahead of the ROS clock")
            matrix = _matrix(message.pose.position, message.pose.orientation)
        except ValueError as exc:
            self._input_error = str(exc)
            self._warn("input_pose", self._input_error)
            return
        self._input_error = ""
        self._latest_input = (matrix, message.header.stamp, time.monotonic())
        if self.world_from_vive is not None:
            self._publish_mapped(matrix, message.header.stamp)

    def _current_hand(self):
        result = self._buffer.lookup_transform(self.world_frame, self.hand_frame, Time())
        stamp = result.header.stamp
        # All-static TF chains have stamp zero and remain valid indefinitely.
        if stamp.sec != 0 or stamp.nanosec != 0:
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
            if abs(age) > self.tf_timeout:
                raise ValueError("Robot hand TCP TF is stale; check the joint-state source")
        return _matrix(result.transform.translation, result.transform.rotation), stamp

    def _valid_input(self):
        if self._input_error:
            raise ValueError(self._input_error)
        if self._latest_input is None:
            raise ValueError("No valid Vive pose received")
        matrix, stamp, received = self._latest_input
        if (time.monotonic() - received > self.input_timeout
                or abs(self._stamp_age(stamp)) > self.input_timeout):
            raise ValueError("Vive pose is stale; synchronization was not changed")
        return matrix, stamp

    def _check_sample_skew(self, vive_stamp, robot_stamp):
        if robot_stamp.sec != 0 or robot_stamp.nanosec != 0:
            skew = abs((Time.from_msg(vive_stamp) - Time.from_msg(robot_stamp)).nanoseconds) * 1e-9
            if skew > min(self.input_timeout, self.tf_timeout):
                raise ValueError("Vive and robot samples are too far apart in time")

    def _sync(self, request, response):
        del request
        try:
            vive_pose, stamp = self._valid_input()
            hand_pose, robot_stamp = self._current_hand()
            self._check_sample_skew(stamp, robot_stamp)
            self.world_from_vive = align_worlds(hand_pose, vive_pose)
            self._sync_sequence += 1
            self._publish_mapped(vive_pose, stamp)
            response.success = True
            response.message = f"Vive pose synchronized to {self.world_frame} -> {self.hand_frame}"
            self.get_logger().info(response.message)
        except (ValueError, TransformException) as exc:
            response.success = False
            response.message = str(exc)
        self._publish_status()
        return response

    def _publish_status(self):
        input_error = robot_error = sync_error = ""
        try:
            _, vive_stamp = self._valid_input()
        except ValueError as exc:
            input_error = str(exc)
        try:
            _, robot_stamp = self._current_hand()
        except (ValueError, TransformException) as exc:
            robot_error = str(exc)
        if not input_error and not robot_error:
            try:
                self._check_sample_skew(vive_stamp, robot_stamp)
            except ValueError as exc:
                sync_error = str(exc)
        self._status_pub.publish(String(data=json.dumps({
            "version": 1,
            "session_id": self._session_id,
            "sync_sequence": self._sync_sequence,
            "input_topic": self.input_topic,
            "input_revision": self._input_revision,
            "world_frame": self.world_frame,
            "hand_frame": self.hand_frame,
            "calibrated": self.world_from_vive is not None,
            "input_ready": not input_error,
            "robot_ready": not robot_error,
            "can_sync": not (input_error or robot_error or sync_error),
            "input_error": input_error,
            "robot_error": robot_error,
            "sync_error": sync_error,
        })))

    def _publish_mapped(self, vive_pose, stamp):
        message = PoseStamped()
        message.header.frame_id = self.world_frame
        message.header.stamp = stamp
        _fill_pose(message.pose, self.world_from_vive @ vive_pose)
        self._mapped_pub.publish(message)
        if self.publish_tf:
            self._broadcaster.sendTransform([
                self._tf(self.world_frame, self.vive_frame, self.world_from_vive, stamp),
                self._tf(self.vive_frame, self.mapped_frame, vive_pose, stamp),
            ])

    @staticmethod
    def _tf(parent, child, matrix, stamp):
        message = TransformStamped()
        message.header.frame_id = parent
        message.header.stamp = stamp
        message.child_frame_id = child
        position, quaternion = pose_from_transform(matrix)
        (message.transform.translation.x, message.transform.translation.y,
         message.transform.translation.z) = map(float, position)
        (message.transform.rotation.x, message.transform.rotation.y,
         message.transform.rotation.z, message.transform.rotation.w) = map(float, quaternion)
        return message

    def _publish_current(self):
        try:
            matrix, stamp = self._current_hand()
        except (ValueError, TransformException) as exc:
            self._warn("robot_tf", f"Waiting for {self.world_frame} -> {self.hand_frame}: {exc}")
            return
        message = PoseStamped()
        message.header.frame_id = self.world_frame
        message.header.stamp = stamp
        _fill_pose(message.pose, matrix)
        self._current_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VivePoseMapper()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
