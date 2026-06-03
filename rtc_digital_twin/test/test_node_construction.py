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

import rclpy

from rtc_digital_twin.digital_twin_node import DigitalTwinNode


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
