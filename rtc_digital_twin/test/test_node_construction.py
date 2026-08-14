"""Smoke tests for DigitalTwinNode construction under a real rclpy context.

These exercise the node's __init__ wiring (no executor spin) and are the sensor
that would have caught the previously-missing imports: construction touches the
QoS profile (rclpy.qos.HistoryPolicy) unconditionally, and the optional tcp_viz
path builds a tf2 Buffer/TransformListener and resolves TransformException in
_lookup_tcp_transform. The marker/TF *content* is covered by the pure-logic
visualizer tests; here we only assert the node stands up and wires the optional
paths on/off.
"""

import contextlib
import os

import rclpy

from rtc_digital_twin.digital_twin_node import DigitalTwinNode


def test_dds_domain_is_isolated():
    """These tests open a real participant, so they must not run on domain 0.

    The text gate (repo_scripts/scripts/validate_test_domains.py) checks that
    test/conftest.py *contains* the claim; this checks that pytest actually
    applied it before a test ran, which is the part a text scan cannot see.
    The number is deliberately not asserted -- the invariant is "not the shared
    default", and pinning 58 here would make re-allocating the id a two-file
    edit for no gain.
    """
    assert os.environ.get("ROS_DOMAIN_ID", "0") != "0", (
        "test/conftest.py did not set ROS_DOMAIN_ID before the suite ran -- "
        "these tests are on the shared default domain (issue #401)"
    )


@contextlib.contextmanager
def running_node(args=None):
    """Construct a DigitalTwinNode in a fresh rclpy context (no spin)."""
    rclpy.init(args=args)
    node = None
    try:
        node = DigitalTwinNode()
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


class TestNodeConstruction:
    def test_default_construction(self):
        # No robot_description, empty source topic, no viz → minimal node.
        # Reaches the unconditional QoS block (HistoryPolicy.KEEP_LAST).
        with running_node() as node:
            assert node._sensor_viz_active is False
            assert node._tcp_viz_active is False
            assert node._sources == []
            # Closure mode is opt-in; disabled by default.
            assert node._closure_active is False

    def test_closure_path_enables_closure_mode(self):
        # closure_path set → node forwards actuated joints to a downstream
        # closure_state_publisher; loop-passive "missing joints" WARN suppressed.
        args = [
            "--ros-args",
            "-p",
            "closure_path:=/tmp/hand.closure.yaml",
        ]
        with running_node(args) as node:
            assert node._closure_active is True

    def test_optional_viz_paths_wire_up(self):
        args = [
            "--ros-args",
            "-p",
            "sensor_viz.sensor_topic:=/hand_sensors",
            "-p",
            "tcp_viz.source_topic:=tool0_actual",
        ]
        with running_node(args) as node:
            assert node._sensor_viz_active is True
            assert node._tcp_viz_active is True
            # source_topic without '/' resolves to the child frame directly.
            assert node._tcp_child_frame == "tool0_actual"
            # Empty tf2 buffer → lookup raises TransformException, caught → None.
            assert node._lookup_tcp_transform() is None

    def test_static_source_creates_subscription(self):
        args = [
            "--ros-args",
            "-p",
            "source_0.topic:=/robot/joint_states",
            "-p",
            "source_0.joint_names:=['j1','j2']",
        ]
        with running_node(args) as node:
            assert len(node._sources) == 1
            cache = node._sources[0]
            assert cache.topic == "/robot/joint_states"
            assert cache.dynamic is False
            assert cache.joint_names == ["j1", "j2"]


class TestControllerTfRebroadcast:
    """Active-controller → /tf re-broadcast wiring (Option B).

    The rewire callback and the restamp are pure wiring/data logic — they run
    without an executor spin, so these exercise them directly (no live topics).
    """

    def test_rebroadcast_enabled_by_default(self):
        with running_node() as node:
            assert node._ctf_enable is True
            assert node._ctf_broadcaster is not None
            # No active controller seen yet → no transforms subscription.
            assert node._ctf_transforms_sub is None
            assert node._ctf_active == ""

    def test_rebroadcast_can_be_disabled(self):
        args = [
            "--ros-args",
            "-p",
            "controller_tf.rebroadcast_enable:=false",
        ]
        with running_node(args) as node:
            assert node._ctf_enable is False
            assert node._ctf_broadcaster is None

    def test_active_controller_rewires_transforms_sub(self):
        from std_msgs.msg import String

        with running_node() as node:
            node._on_active_controller(String(data="demo_wbc_controller"))
            assert node._ctf_active == "demo_wbc_controller"
            first_sub = node._ctf_transforms_sub
            assert first_sub is not None
            assert first_sub.topic_name == "/demo_wbc_controller/transforms"

            # Same name → idempotent no-op (keeps the same subscription handle).
            node._on_active_controller(String(data="demo_wbc_controller"))
            assert node._ctf_transforms_sub is first_sub

            # Switch → old sub destroyed, new one bound to the new topic.
            node._on_active_controller(String(data="demo_task_controller"))
            assert node._ctf_active == "demo_task_controller"
            assert node._ctf_transforms_sub is not first_sub
            assert node._ctf_transforms_sub.topic_name == "/demo_task_controller/transforms"

            # Blank name is ignored (no spurious rewire).
            node._on_active_controller(String(data="  "))
            assert node._ctf_active == "demo_task_controller"

    def test_transforms_cb_restamps_and_broadcasts(self):
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        with running_node() as node:
            sent = []
            node._ctf_broadcaster.sendTransform = lambda tfs: sent.append(tfs)

            tf = TransformStamped()
            tf.header.frame_id = "base"
            tf.child_frame_id = "tool0_actual"
            tf.header.stamp = Time(sec=0, nanosec=0)  # stale stamp from source
            node._controller_transforms_cb(TFMessage(transforms=[tf]))

            assert len(sent) == 1
            forwarded = sent[0]
            assert len(forwarded) == 1
            # frame identity preserved, stamp replaced with node clock (non-zero).
            assert forwarded[0].header.frame_id == "base"
            assert forwarded[0].child_frame_id == "tool0_actual"
            assert (forwarded[0].header.stamp.sec, forwarded[0].header.stamp.nanosec) != (0, 0)

    def test_transforms_cb_ignores_empty_message(self):
        from tf2_msgs.msg import TFMessage

        with running_node() as node:
            sent = []
            node._ctf_broadcaster.sendTransform = lambda tfs: sent.append(tfs)
            node._controller_transforms_cb(TFMessage(transforms=[]))
            assert sent == []
