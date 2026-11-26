"""sim_recover against fake peers, in one process (domain 52, CMakeLists).

The fakes: the CM's two latched topics and the group's joint_states, the joint
controller's joint_goal subscriber (it "moves" the plant by echoing the goal
into the joint state, or not), and a /sim/reset server. Distances are tiny and
the margin short so the waits are a fraction of a second.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from integrated_bringup import sim_recover
from integrated_bringup.sim_recover import SimRecover
from integrated_bringup.stand_pose import stand_pose, trajectory_duration
from rtc_msgs.msg import RobotTarget

PROFILE = Path(__file__).resolve().parents[1] / "config" / "g1_loco"
CTRL = "demo_joint_controller"
MARGIN = 0.1
NAMES, POSE = stand_pose(PROFILE, CTRL, "g1")


def _latched() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Stack:
    """The fake CM + controller + sim, a SimRecover and a client, on one executor."""

    def __init__(
        self,
        *,
        offset: float,
        follows: bool,
        active: str = CTRL,
        estop: bool = False,
        sim_rate: float | None = 1.0,
        residual_tol: float = 0.0,
    ):
        # sim_rate: sim seconds per wall second; None = the sim publishes no /sim/time.
        self.sim_rate = sim_rate
        self._sim_t = 0.0
        self._last_wall = time.monotonic()
        self.fake = Node("fake_stack")
        self.client_node = Node("recover_client")
        self.node = SimRecover(
            parameter_overrides=[
                Parameter("profile_dir", value=str(PROFILE)),
                Parameter("margin_s", value=MARGIN),
                Parameter("residual_tol", value=residual_tol),
            ]
        )
        self.q = [p + offset for p in POSE]
        self.follows = follows
        self.goal_at: float | None = None
        self.reset_at: float | None = None
        self.reset_calls = 0
        self.reset_sim_t = 0.0
        self.goal_sim_t = 0.0
        active_pub = self.fake.create_publisher(
            String, "/rtc_cm/active_controller_name", _latched()
        )
        self.estop_pub = self.fake.create_publisher(Bool, "/system/estop_status", _latched())
        self.js_pub = self.fake.create_publisher(JointState, "/rtc_cm/g1/joint_states", 1)
        self.fake.create_subscription(RobotTarget, f"/{CTRL}/g1/joint_goal", self._on_goal, 1)
        self.fake.create_service(Trigger, "/sim/reset", self._on_reset)
        self.sim_time_pub = self.fake.create_publisher(Float64, "/sim/time", 1)
        self.fake.create_timer(0.01, self._tick)
        active_pub.publish(String(data=active))
        self.estop_pub.publish(Bool(data=estop))
        self.recover = self.client_node.create_client(Trigger, "/sim_recover/recover")
        self.executor = MultiThreadedExecutor(num_threads=6)
        for n in (self.fake, self.client_node, self.node):
            self.executor.add_node(n)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self.node._lock:
                got = self.node._active is not None and self.node._estop is not None
                got = got and self.node._positions_locked() is not None
                got = got and (self.node._sim_time is not None or sim_rate is None)
            if got and self.recover.service_is_ready():
                return
            time.sleep(0.01)
        raise TimeoutError("discovery")

    def _tick(self) -> None:
        msg = JointState(name=list(NAMES), position=list(self.q))
        self.js_pub.publish(msg)
        wall = time.monotonic()
        if self.sim_rate is not None:
            self._sim_t += self.sim_rate * (wall - self._last_wall)
            self.sim_time_pub.publish(Float64(data=self._sim_t))
        self._last_wall = wall

    def _on_goal(self, msg: RobotTarget) -> None:
        self.goal_at = time.monotonic()
        self.goal_sim_t = self._sim_t
        assert msg.goal_type == "joint"
        assert list(msg.joint_names) == NAMES
        assert list(msg.joint_target) == POSE
        if self.follows:
            self.q = list(msg.joint_target)

    def _on_reset(self, _req, res):
        self.reset_at = time.monotonic()
        self.reset_sim_t = self._sim_t
        self.reset_calls += 1
        res.success, res.message = True, "reset"
        return res

    def call(self) -> Trigger.Response:
        fut = self.recover.call_async(Trigger.Request())
        deadline = time.monotonic() + 15.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert fut.done()
        return fut.result()

    def wait_for_goal(self) -> None:
        deadline = time.monotonic() + 5.0
        while self.goal_at is None and time.monotonic() < deadline:
            time.sleep(0.005)
        assert self.goal_at is not None

    def close(self) -> None:
        self.executor.shutdown()
        self._thread.join(timeout=2.0)
        for n in (self.fake, self.client_node, self.node):
            n.destroy_node()


@pytest.fixture(scope="module", autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def make():
    stacks = []

    def _make(**kw):
        stacks.append(Stack(**kw))
        return stacks[-1]

    yield _make
    for s in stacks:
        s.close()


def test_happy_path_resets_only_after_the_trajectory_wait(make):
    s = make(offset=0.01, follows=True)
    t0 = time.monotonic()
    res = s.call()
    assert res.success, res.message
    assert s.reset_calls == 1
    wait = trajectory_duration(0.01, 0.78, 3.14) + MARGIN
    assert s.reset_at - s.goal_at >= wait - 0.02
    assert s.goal_at >= t0


def test_residual_failure_does_not_reset(make):
    s = make(offset=0.2, follows=False, residual_tol=0.05)
    res = s.call()
    assert not res.success
    assert "not at the stand pose" in res.message
    assert s.goal_at is not None
    assert s.reset_calls == 0


def test_refused_when_the_estop_is_latched(make):
    s = make(offset=0.01, follows=True, estop=True)
    res = s.call()
    assert not res.success and "E-STOP" in res.message
    assert s.goal_at is None and s.reset_calls == 0


def test_refused_when_another_controller_is_active(make):
    s = make(offset=0.01, follows=True, active="locomotion_controller")
    res = s.call()
    assert not res.success and "locomotion_controller" in res.message
    assert s.goal_at is None and s.reset_calls == 0


def test_a_second_request_during_a_recovery_is_refused(make):
    s = make(offset=0.5, follows=True)  # 0.64 s trajectory + margin
    first = s.recover.call_async(Trigger.Request())
    s.wait_for_goal()  # the first request is now inside its wait
    second = s.call()
    assert not second.success and "already running" in second.message
    deadline = time.monotonic() + 10.0
    while not first.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert first.done() and first.result().success
    assert s.reset_calls == 1


def test_estop_latching_during_the_wait_aborts(make):
    s = make(offset=1.0, follows=False)  # 1.28 s trajectory + margin
    fut = s.recover.call_async(Trigger.Request())
    s.wait_for_goal()
    s.estop_pub.publish(Bool(data=True))
    deadline = time.monotonic() + 10.0
    while not fut.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert fut.done()
    assert not fut.result().success and "E-STOP" in fut.result().message
    assert s.reset_calls == 0


def test_default_has_no_residual_gate_and_reports_it(make):
    s = make(offset=0.2, follows=False)  # joints stay blocked, 0.2 rad off
    res = s.call()
    assert res.success, res.message
    assert "residual 0.200 rad" in res.message
    assert s.reset_calls == 1


def test_reset_waits_for_sim_time_not_wall_time(make):
    s = make(offset=0.2, follows=True, sim_rate=0.5)
    res = s.call()
    assert res.success, res.message
    wait = trajectory_duration(0.2, 0.78, 3.14) + MARGIN
    assert s.reset_sim_t - s.goal_sim_t >= wait - 0.03
    # half-speed sim: twice the wall time
    assert s.reset_at - s.goal_at >= 2 * wait - 0.1


def test_refused_without_sim_time(make):
    s = make(offset=0.01, follows=True, sim_rate=None)
    res = s.call()
    assert not res.success and "no sim time" in res.message
    assert s.goal_at is None and s.reset_calls == 0


def test_a_stalled_sim_hits_the_wall_guard(make, monkeypatch):
    monkeypatch.setattr(sim_recover, "_WALL_GUARD_MIN_S", 0.3)
    monkeypatch.setattr(sim_recover, "_WALL_GUARD_FACTOR", 0.0)
    s = make(offset=0.2, follows=True, sim_rate=0.0)
    res = s.call()
    assert not res.success and "wall guard" in res.message
    assert s.reset_calls == 0


def test_stop_ends_a_running_wait_without_a_reset(make):
    s = make(offset=0.2, follows=True, sim_rate=0.0)  # a stalled sim: the wait would last
    fut = s.recover.call_async(Trigger.Request())
    s.wait_for_goal()
    stopped_at = time.monotonic()
    s.node.stop()  # what main() does on shutdown, before the executor's
    deadline = time.monotonic() + 5.0
    while not fut.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert fut.done() and time.monotonic() - stopped_at < 1.0
    assert not fut.result().success and "shutting down" in fut.result().message
    assert s.reset_calls == 0
