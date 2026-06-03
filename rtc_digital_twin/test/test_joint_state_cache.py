"""Tests for JointStateCache — per-source joint cache dataclass.

Pure dataclass logic (``__post_init__``) from digital_twin_node: static vs.
dynamic mode selection and buffer sizing. Importing the module is safe (the
ROS-runtime symbols it references live inside DigitalTwinNode methods, not at
import time), so no node construction or spin is needed.
"""

from rtc_digital_twin.digital_twin_node import JointStateCache


class TestStaticMode:
    def test_named_joints_build_index_map(self):
        cache = JointStateCache(topic="/joints", joint_names=["a", "b", "c"])
        assert cache.dynamic is False
        assert cache.name_to_idx == {"a": 0, "b": 1, "c": 2}

    def test_buffers_sized_to_joint_count(self):
        cache = JointStateCache(topic="/joints", joint_names=["a", "b", "c"])
        assert cache.positions == [0.0, 0.0, 0.0]
        assert cache.velocities == [0.0, 0.0, 0.0]

    def test_topic_preserved(self):
        cache = JointStateCache(topic="/source_0", joint_names=["a"])
        assert cache.topic == "/source_0"
        assert cache.data_received is False


class TestDynamicMode:
    def test_empty_names_is_dynamic(self):
        cache = JointStateCache(topic="/joints", joint_names=[])
        assert cache.dynamic is True

    def test_dynamic_buffers_empty(self):
        cache = JointStateCache(topic="/joints", joint_names=[])
        assert cache.name_to_idx == {}
        assert cache.positions == []
        assert cache.velocities == []
        assert cache.received_names == []
