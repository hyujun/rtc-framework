"""demo_controller_sequence's rclpy side against a fake stack, in one process.

The pure suite (test_demo_sequence) trusts ``ros_io`` to fill the
Observation; this pins that it does, on the names and QoS the real stack uses:
the CM node and its two latched topics, the controller's parameter node, the
hand's joint_states and link_status, and joint_goal. Domain 52 (CMakeLists:
ENV + RESOURCE_LOCK ros_domain_52).
"""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from integrated_bringup.demo_gui.discovery import ROBOT_PROFILES
from integrated_bringup.demo_sequence.readiness import READY, assess, runtime_abort
from integrated_bringup.demo_sequence.ros_io import SequenceIo
from rtc_msgs.msg import RobotTarget
from rtc_tools.analysis.ros_stop import StopFlag

PROFILE = ROBOT_PROFILES["p2"]
ROSTER = tuple(PROFILE.shape.hand_motor_names)


def _latched() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    return qos


class FakeStack:
    """The CM, the controller's node and the hand driver, as the CLI sees them."""

    def __init__(self, names=ROSTER) -> None:
        self.cm = Node("integrated_rt_controller")
        self.ctrl = Node("hand_joint_controller", namespace="/hand_joint_controller")
        self.driver = Node("udp_hand_node")
        self.active_pub = self.cm.create_publisher(
            String, "/rtc_cm/active_controller_name", _latched()
        )
        self.estop_pub = self.cm.create_publisher(Bool, "/system/estop_status", _latched())
        self.joints_pub = self.cm.create_publisher(JointState, "/rtc_cm/p2_right/joint_states", 1)
        self.link_pub = self.driver.create_publisher(Bool, "/p2/right/link_status", _latched())
        self.ctrl.declare_parameter("mode", "tracking")
        self.ctrl.declare_parameter("max_traj_velocity", 10.0)
        self.ctrl.declare_parameter("trajectory_speed", 1.0)
        self.ctrl.add_on_set_parameters_callback(self._cap)
        self.goals: list[RobotTarget] = []
        self.ctrl.create_subscription(
            RobotTarget, "/hand_joint_controller/p2_right/joint_goal", self.goals.append, 1
        )
        self.names = list(names)
        self.cm.create_timer(0.01, self._tick)
        self.active_pub.publish(String(data="hand_joint_controller"))
        self.estop_pub.publish(Bool(data=False))
        self.link_pub.publish(Bool(data=True))
        self.executor = MultiThreadedExecutor()
        for node in (self.cm, self.ctrl, self.driver):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _cap(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "trajectory_speed" and p.value > 10.0:
                return SetParametersResult(successful=False, reason="above max_traj_velocity")
        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        msg = JointState()
        msg.name = self.names
        msg.position = [0.1 * i for i in range(len(self.names))]
        self.joints_pub.publish(msg)

    def close(self) -> None:
        self.executor.shutdown()
        for node in (self.cm, self.ctrl, self.driver):
            node.destroy_node()


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def io(ros, tmp_path):
    node = SequenceIo(PROFILE, "p2", StopFlag(), str(tmp_path / "s.json"), str(tmp_path))
    yield node
    node.close()


def _until(io, predicate, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    obs = io.observe()
    while not predicate(obs) and time.monotonic() < deadline:
        io.spin_some(0.02)
        obs = io.observe()
    return obs


def test_a_complete_stack_reads_as_ready(io):
    stack = FakeStack()
    try:
        obs = _until(io, lambda o: assess(o, True).kind == READY)
        assert assess(obs, True).kind == READY, assess(obs, True).reason
        assert (obs.cm_nodes, obs.param_nodes, obs.active_pubs, obs.estop_pubs) == (1, 1, 1, 1)
        assert obs.active == "hand_joint_controller" and obs.estop is False
        assert obs.link_pubs == 1 and obs.link is True
        assert obs.mode == "tracking" and obs.cap == 10.0
        assert obs.goal_pubs == 1 and obs.goal_matched == 1 and not obs.other_hand
        assert obs.roster_ok and obs.positions == pytest.approx([0.1 * i for i in range(10)])
    finally:
        stack.close()


def test_a_goal_carries_the_profile_roster_and_the_speed_set_answers(io):
    stack = FakeStack()
    try:
        _until(io, lambda o: o.goal_matched == 1 and o.cap is not None)
        handle = io.set_speed(2.5)
        deadline = time.monotonic() + 5.0
        while io.speed_result(handle) is None and time.monotonic() < deadline:
            io.spin_some(0.02)
        assert io.speed_result(handle) is True
        assert stack.ctrl.get_parameter("trajectory_speed").value == 2.5
        refused = io.set_speed(50.0)
        while io.speed_result(refused) is None and time.monotonic() < deadline + 5.0:
            io.spin_some(0.02)
        assert io.speed_result(refused) is False
        io.publish([0.2] * len(ROSTER))
        deadline = time.monotonic() + 5.0
        while not stack.goals and time.monotonic() < deadline:
            io.spin_some(0.02)
        assert stack.goals, "the goal never reached the controller's subscription"
        assert tuple(stack.goals[-1].joint_names) == ROSTER
        assert stack.goals[-1].goal_type == "joint"
    finally:
        stack.close()


def test_a_vanished_cm_is_no_longer_evidenced_by_its_latched_sample(io):
    stack = FakeStack()
    try:
        _until(io, lambda o: o.active_pubs == 1 and o.active is not None)
        stack.cm.destroy_publisher(stack.active_pub)
        obs = _until(io, lambda o: o.active_pubs == 0)
        assert obs.active == "hand_joint_controller", "the cached sample is still there"
        assert "gone" in runtime_abort(obs, link_required=True)
    finally:
        stack.close()


def test_the_log_growth_is_read_from_the_newest_session_under_the_stack_root(io, tmp_path):
    # Not from wherever this process runs: the session name is joined to the root.
    (tmp_path / "261120_0900").mkdir()
    (tmp_path / "261120_0900" / "old.csv").write_bytes(b"x" * 10)
    newest = tmp_path / "261120_1000" / "timing"
    newest.mkdir(parents=True)
    (newest / "cm_timing_log.csv").write_bytes(b"x" * 1000)
    assert io.log_bytes() == 1000


def test_a_foreign_roster_is_seen_as_one(io):
    stack = FakeStack(names=[f"other_{i}" for i in range(len(ROSTER))])
    try:
        obs = _until(io, lambda o: o.roster_ok is not None)
        assert obs.roster_ok is False
    finally:
        stack.close()
