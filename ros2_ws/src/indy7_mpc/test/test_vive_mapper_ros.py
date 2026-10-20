"""Exercise synchronization using real ROS topics, TF and a Trigger service.

Domain 88 is isolated from the application's domain 77. No robot driver runs.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from indy7_mpc.vive_pose_mapper import VivePoseMapper, _matrix


def test_sync_and_mapping_with_independent_worlds():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=88)
    executor = SingleThreadedExecutor(context=context)
    overrides = {"input_pose_topic": "/test_vive/input",
                 "mapped_pose_topic": "/test_vive/mapped",
                 "current_hand_pose_topic": "/test_vive/current",
                 "sync_service": "/test_vive/sync",
                 "hand_frame": "test_hand_tcp",
                 "vive_world_frame": "test_vive_world",
                 "mapped_hand_frame": "test_vive_target",
                 "input_timeout": 0.3}
    mapper = VivePoseMapper(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in overrides.items()
    ])
    peer = Node("test_vive_peer", context=context)
    executor.add_node(mapper)
    executor.add_node(peer)

    def wait_for(predicate):
        end = time.monotonic() + 5
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        client = peer.create_client(Trigger, "/test_vive/sync")
        wait_for(client.service_is_ready)
        missing = client.call_async(Trigger.Request())
        wait_for(missing.done)
        assert not missing.result().success

        broadcaster = StaticTransformBroadcaster(peer)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "world", "test_hand_tcp"
        tf.transform.translation.x, tf.transform.translation.z = 0.5, 0.6
        tf.transform.rotation.z = tf.transform.rotation.w = 2 ** -0.5
        broadcaster.sendTransform(tf)

        outputs, currents = [], []
        peer.create_subscription(PoseStamped, "/test_vive/mapped", outputs.append, 10)
        peer.create_subscription(PoseStamped, "/test_vive/current", currents.append, 10)
        publisher = peer.create_publisher(PoseStamped, "/test_vive/input", 10)
        message = PoseStamped()
        # Same source label as robot world, but different coordinates.
        message.header.frame_id = "world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = 2., 3., 4.
        message.pose.orientation.w = 1.

        def publish():
            message.header.stamp = peer.get_clock().now().to_msg()
            publisher.publish(message)

        timer = peer.create_timer(0.02, publish)
        wait_for(lambda: mapper._latest_input is not None and len(currents) > 0)
        assert not outputs
        synced = client.call_async(Trigger.Request())
        wait_for(synced.done)
        assert synced.result().success, synced.result().message
        wait_for(lambda: len(outputs) > 0)
        first = outputs[0]
        assert first.header.frame_id == "world"
        np.testing.assert_allclose([first.pose.position.x, first.pose.position.y, first.pose.position.z],
                                   [0.5, 0, 0.6], atol=1e-12)

        message.pose.position.x = 2.1
        message.pose.orientation.z = message.pose.orientation.w = 2 ** -0.5
        wait_for(lambda: abs(outputs[-1].pose.position.y - 0.1) < 1e-8)
        mapped = outputs[-1]
        np.testing.assert_allclose(_matrix(mapped.pose.position, mapped.pose.orientation)[:3, :3],
                                   np.diag([-1, -1, 1]), atol=1e-12)
        wait_for(lambda: mapper._buffer.can_transform("world", "test_vive_target", Time()))
        wait_for(lambda: abs(mapper._buffer.lookup_transform(
            "world", "test_vive_target", Time()).transform.translation.y - 0.1) < 1e-8)

        timer.cancel()
        last_received = mapper._latest_input[2]
        wait_for(lambda: time.monotonic() - last_received > 0.35)
        previous = mapper.world_from_vive.copy()
        stale = client.call_async(Trigger.Request())
        wait_for(stale.done)
        assert not stale.result().success
        np.testing.assert_array_equal(mapper.world_from_vive, previous)
    finally:
        executor.shutdown()
        mapper.destroy_node()
        peer.destroy_node()
        context.shutdown()
