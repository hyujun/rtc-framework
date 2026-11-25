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
from pathlib import Path

import pytest
import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from integrated_bringup.demo_gui import shipped as s
from integrated_bringup.demo_gui.discovery import ROBOT_PROFILES
from integrated_bringup.demo_sequence.readiness import READY, REFUSE, assess, runtime_abort
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

    def __init__(
        self, names=ROSTER, driver_params: dict | None = None, retarget: bool | None = True
    ) -> None:
        self.cm = Node("integrated_rt_controller")
        self.ctrl = Node("hand_joint_controller", namespace="/hand_joint_controller")
        self.driver = Node("udp_hand_node")
        for name, value in (driver_params or {}).items():
            # An empty list has no type to infer: declare it through a
            # descriptor, as the driver's empty per-hand defaults are.
            descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
            self.driver.declare_parameter(name, value, descriptor if value == [] else None)
        self.active_pub = self.cm.create_publisher(
            String, "/rtc_cm/active_controller_name", _latched()
        )
        self.estop_pub = self.cm.create_publisher(Bool, "/system/estop_status", _latched())
        self.joints_pub = self.cm.create_publisher(JointState, "/rtc_cm/p2_right/joint_states", 1)
        self.link_pub = self.driver.create_publisher(Bool, "/p2/right/link_status", _latched())
        self.ctrl.declare_parameter("mode", "tracking")
        self.ctrl.declare_parameter("max_traj_velocity", 10.0)
        self.ctrl.declare_parameter("trajectory_speed", 1.0)
        if retarget is not None:  # None: an older controller that does not declare it
            self.ctrl.declare_parameter("continuous_retarget", retarget)
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


def test_a_controller_without_continuous_retarget_still_reads_mode_and_cap(io):
    """Asked on its own: an undeclared continuous_retarget must not empty the
    mode / cap answer (GetParameters answers nothing for an unknown name)."""
    stack = FakeStack(retarget=None)
    try:
        obs = _until(io, lambda o: assess(o, True).kind == READY)
        assert assess(obs, True).kind == READY, assess(obs, True).reason
        assert obs.mode == "tracking" and obs.cap == 10.0
        deadline = time.monotonic() + 3.0  # both requests answered at least once
        while time.monotonic() < deadline:
            io.spin_some(0.05)
        assert io.observe().retarget is None
    finally:
        stack.close()


def test_a_complete_stack_reads_as_ready(io):
    stack = FakeStack()
    try:
        obs = _until(io, lambda o: assess(o, True).kind == READY)
        assert assess(obs, True).kind == READY, assess(obs, True).reason
        assert (obs.cm_nodes, obs.param_nodes, obs.active_pubs, obs.estop_pubs) == (1, 1, 1, 1)
        assert obs.active == "hand_joint_controller" and obs.estop is False
        assert obs.link_pubs == 1 and obs.link is True
        assert obs.mode == "tracking" and obs.cap == 10.0
        obs = _until(io, lambda o: o.retarget is not None)
        assert obs.retarget is True
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


_CONFIG = Path(__file__).resolve().parents[1] / "config"
CALIBRATION_KEYS = ("joint_state_names", "joint_direction_signs", "joint_position_offsets_deg")


def _driver_params(**per_hand) -> dict:
    """p2's shipped driver parameters as the node declares them: the flat
    keys, hand_sides, and under hand_sides the per-hand keys (empty unless
    ``per_hand`` sets one)."""
    doc = yaml.safe_load((_CONFIG / "p2" / "udp_hand_node_p2.yaml").read_text())
    params = doc["/**"]["ros__parameters"]
    out = {"hand_sides": params["hand_sides"], **{k: params[k] for k in CALIBRATION_KEYS}}
    for side in params["hand_sides"]:
        for key in CALIBRATION_KEYS:
            out[f"{side}.{key}"] = per_hand.get(key, [])
    return out


def _recorded() -> s.Calibration:
    return s.read_calibration(str(_CONFIG / "p2" / s.SHIPPED_DIR / s.CALIBRATION_FILE))


def test_the_running_driver_calibration_is_read_and_matches_the_record(io):
    stack = FakeStack(driver_params=_driver_params())
    recorded = _recorded()
    try:
        obs = _until(io, lambda o: assess(o, True, recorded).kind == READY)
        assert assess(obs, True, recorded).kind == READY, assess(obs, True, recorded).reason
        assert obs.driver_problem == "" and obs.driver_nodes == 1
        assert obs.driver_calibration.joint_position_offsets_deg == (
            recorded.joint_position_offsets_deg
        )
    finally:
        stack.close()


def test_a_per_hand_offset_on_the_running_driver_is_the_one_compared(io):
    offsets = list(_recorded().joint_position_offsets_deg)
    offsets[0] += 3.0
    stack = FakeStack(driver_params=_driver_params(joint_position_offsets_deg=offsets))
    try:
        obs = _until(io, lambda o: o.driver_calibration is not None or o.driver_problem)
        verdict = assess(obs, True, _recorded())
        assert verdict.kind == REFUSE and "restart the bring-up" in verdict.reason, verdict
    finally:
        stack.close()


class _Answer:
    """A GetParameters response as the reader sees it."""

    def __init__(self, *values) -> None:
        from rclpy.parameter import Parameter

        self.values = [Parameter("x", value=v).get_parameter_value() for v in values]


def _settled(reader, *answers):
    for answer in answers:
        reader.take(answer)
    return reader


def test_the_reader_takes_the_flat_keys_in_one_request_without_hand_sides():
    from integrated_bringup.demo_sequence.ros_io import DriverCalibrationReader

    reader = DriverCalibrationReader("right")
    assert reader.next_request()[0] == "hand_sides"
    _settled(reader, _Answer([], ["j0", "j1"], [1.0, -1.0], [0.0, 5.0]))
    assert reader.problem == "" and reader.next_request() == []
    cal = reader.calibration
    assert cal.joint_names == ("j0", "j1")
    assert cal.joint_direction_signs == (1.0, -1.0)
    assert cal.joint_position_offsets_deg == (0.0, 5.0)


def test_the_reader_asks_again_while_the_driver_has_not_declared_its_keys():
    from integrated_bringup.demo_sequence.ros_io import DriverCalibrationReader

    reader = DriverCalibrationReader("right")
    first = reader.next_request()
    reader.take(_Answer())  # an undeclared name empties the whole answer
    assert reader.next_request() == first and reader.calibration is None
    assert reader.problem == ""


def test_a_driver_running_another_hand_is_named_not_compared():
    from integrated_bringup.demo_sequence.ros_io import DriverCalibrationReader

    reader = _settled(DriverCalibrationReader("right"), _Answer(["left"], ["j0"], [1.0], [0.0]))
    assert reader.calibration is None
    assert "hand_sides ['left'], not 'right'" in reader.problem
