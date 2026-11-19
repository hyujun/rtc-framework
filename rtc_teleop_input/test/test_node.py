"""TeleopInputNode on real rclpy: publish tick, relay, service calls, status."""

from __future__ import annotations

import threading
import time
from array import array

import pytest
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from rtc_msgs.srv import ResetFault
from rtc_teleop_input.model import ConfigError
from rtc_teleop_input.teleop_ros import SpinThread, TeleopInputNode

DT = 0.02
QOS = QoSProfile(depth=1)


def make_node(uid, **params):
    params.setdefault("output_topic", f"/{uid}/joy")
    params.setdefault("estop_service", f"/{uid}/trigger_estop")
    params.setdefault("reset_service", f"/{uid}/reset_fault")
    results = []
    statuses = []
    node = TeleopInputNode(
        parameter_overrides=[Parameter(k, value=v) for k, v in params.items()],
        on_service_result=lambda *r: results.append(r),
        on_status=statuses.append,
    )
    return node, results, statuses


class Peer:
    """A second node on its own executor thread: publishers, subscribers, servers."""

    def __init__(self, name):
        self.node = Node(name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


def wait_until(predicate, deadline_s=3.0):
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


@pytest.fixture
def peer(ros, uid):
    p = Peer(f"peer_{uid}")
    yield p
    p.close()


def test_one_message_per_tick(ros, uid):
    node, _, _ = make_node(uid)
    sent = []
    node._joy_pub.publish = sent.append
    node.virtual.key_event("W", True, False)
    before = node.get_clock().now().nanoseconds
    for i in range(7):
        node.step(i * DT)
    after = node.get_clock().now().nanoseconds
    assert len(sent) == 7
    for msg in sent:
        assert (len(msg.axes), len(msg.buttons)) == (6, 21)
        assert msg.header.frame_id == "virtual"
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        assert before <= stamp <= after
    # the GUI draws the frame that went out last
    assert tuple(sent[-1].axes) == node.last_frame.axes
    assert tuple(sent[-1].buttons) == node.last_frame.buttons
    assert node.last_frame.axes[1] == 0.25
    node.destroy_node()


def test_refuses_to_start(ros, uid):
    with pytest.raises(ConfigError, match="rate_hz"):
        make_node(uid, rate_hz=0.0)
    with pytest.raises(ConfigError, match="output_topic"):
        make_node(uid, output_topic="")
    with pytest.raises(ConfigError, match="differ"):
        make_node(uid, input="pad", pad_topic=f"/{uid}/joy")


def test_integer_rate_parameter_is_accepted(ros, uid):
    node, _, _ = make_node(uid, rate_hz=100)
    assert node.config.rate_hz == 100.0
    node.destroy_node()


def test_live_rate(ros, uid, peer):
    """Ticks driven at 50 Hz arrive at 50 Hz (the Qt timer's rate: test_widget_offscreen)."""
    node, _, _ = make_node(uid)
    received = []
    peer.node.create_subscription(
        Joy, f"/{uid}/joy", lambda m: received.append(time.monotonic()), QOS
    )
    assert wait_until(lambda: node.count_subscribers(f"/{uid}/joy") == 1)
    start = time.monotonic()
    while time.monotonic() - start < 2.0:
        node.step(time.monotonic())
        time.sleep(DT)
    assert wait_until(lambda: len(received) > 0)
    time.sleep(0.1)
    elapsed = received[-1] - received[0]
    expected = elapsed / DT
    # sleep-paced ticks run a little slow; the count must still match the arrivals
    assert abs(len(received) - 1 - expected) <= 2 + 0.1 * expected
    node.destroy_node()


def test_pad_relay_live(ros, uid, peer):
    node, _, _ = make_node(uid, input="pad", pad_topic=f"/{uid}/pad")
    spin = SpinThread(node)
    spin.start()
    try:
        upstream = peer.node.create_publisher(Joy, f"/{uid}/pad", QOS)
        assert wait_until(lambda: node.count_publishers(f"/{uid}/pad") == 1)
        msg = Joy()
        msg.axes = array("f", [0.1, -0.7, 0.33333334, 0.0, -1.0, 1e-38])
        msg.buttons = [0] * 9 + [1] + [0] * 11
        upstream.publish(msg)
        assert wait_until(lambda: not node.relay.frame(time.monotonic()).lost)
        upstream.publish(msg)
        time.sleep(0.05)
        # keyboard and mouse do not touch the pad's output
        node.virtual.key_event("W", True, False)
        node.virtual.drag_stick("right", 1.0, 1.0)
        sent = []
        node._joy_pub.publish = sent.append
        frame = node.step(time.monotonic())
        assert array("f", frame.axes).tobytes() == msg.axes.tobytes()
        assert array("f", sent[-1].axes).tobytes() == msg.axes.tobytes()
        assert sent[-1].header.frame_id == "pad"
        assert frame.buttons[9] == 1
        # the pad goes silent: released within the timeout (+ a tick)
        silent_at = time.monotonic()
        assert wait_until(lambda: node.step(time.monotonic()).lost, 0.35)
        assert time.monotonic() - silent_at <= 0.35
        assert sent[-1].axes.tolist() == [0.0] * 6
        assert list(sent[-1].buttons) == [0] * 21
        upstream.publish(msg)
        assert wait_until(lambda: not node.step(time.monotonic()).lost)
    finally:
        spin.stop()
        node.destroy_node()


def test_estop_call_does_not_block_the_tick(ros, uid, peer):
    entered = threading.Event()
    release = threading.Event()
    answered = threading.Event()

    def slow(request, response):  # answers only when the test is over
        entered.set()
        release.wait(5.0)
        answered.set()
        return response

    peer.node.create_service(Trigger, f"/{uid}/trigger_estop", slow)
    node, results, _ = make_node(uid)
    spin = SpinThread(node)
    spin.start()
    try:
        assert wait_until(lambda: node._estop_client.service_is_ready())
        node.virtual.key_event("Escape", True, False)
        node.virtual.key_event("Escape", False, False)
        stamps = []
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            node.step(time.monotonic())
            stamps.append(time.monotonic())
            time.sleep(DT)
        assert entered.is_set()
        assert node.estop_calls_sent == 1
        gaps = [b - a for a, b in zip(stamps, stamps[1:], strict=False)]
        assert max(gaps) <= 3 * DT
        assert results == []  # still waiting, and nothing blocked on it
    finally:
        # let the server answer before its node goes away
        release.set()
        answered.wait(2.0)
        spin.stop()
        node.destroy_node()


def test_estop_reply_is_reported(ros, uid, peer):
    calls = []

    def serve(request, response):
        calls.append(1)
        response.success = True
        response.message = "latched"
        return response

    peer.node.create_service(Trigger, f"/{uid}/trigger_estop", serve)
    node, results, _ = make_node(uid)
    spin = SpinThread(node)
    spin.start()
    try:
        assert wait_until(lambda: node._estop_client.service_is_ready())
        node.actions.request_estop()
        node.step(0.0)
        assert wait_until(lambda: results == [("estop", True, "latched")])
        assert calls == [1]
    finally:
        spin.stop()
        node.destroy_node()


def test_unavailable_service_is_reported_not_sent(ros, uid):
    node, results, _ = make_node(uid)
    node.actions.request_estop()
    node.step(0.0)
    assert node.estop_calls_sent == 0
    assert results == [("estop", False, f"/{uid}/trigger_estop is not available")]
    node.destroy_node()


def _hold_reset(node, seconds):
    node.virtual.key_event("Backspace", True, False)
    t = 0.0
    for _ in range(round(seconds / DT)):
        t += DT
        node.step(t)
    node.virtual.key_event("Backspace", False, False)


def test_reset_carries_the_controller_name(ros, uid, peer):
    names = []

    def serve(request, response):
        names.append(request.controller_name)
        response.ok = True
        response.message = "cleared"
        return response

    peer.node.create_service(ResetFault, f"/{uid}/reset_fault", serve)
    node, results, _ = make_node(uid, reset_controller_name="some_controller")
    spin = SpinThread(node)
    spin.start()
    try:
        assert wait_until(lambda: node._reset_client.service_is_ready())
        _hold_reset(node, 1.5)
        assert wait_until(lambda: results == [("reset", True, "cleared")])
        assert names == ["some_controller"]
    finally:
        spin.stop()
        node.destroy_node()


def test_reset_disabled_sends_nothing(ros, uid, peer):
    names = []
    peer.node.create_service(
        ResetFault, f"/{uid}/reset_fault", lambda req, resp: names.append(1) or resp
    )
    node, results, _ = make_node(uid)
    assert wait_until(lambda: node._reset_client.service_is_ready())
    _hold_reset(node, 3.0)
    time.sleep(0.2)
    assert node.reset_calls_sent == 0
    assert names == []
    assert results == []
    node.destroy_node()


def test_status_topics(ros, uid, peer):
    node, _, statuses = make_node(uid, status_topics=[f"/{uid}/status"])
    spin = SpinThread(node)
    spin.start()
    try:
        pub = peer.node.create_publisher(DiagnosticStatus, f"/{uid}/status", QOS)
        assert wait_until(lambda: node.count_publishers(f"/{uid}/status") == 1)
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.WARN
        msg.name = "ctrl"
        msg.message = "slow"
        msg.values = [KeyValue(key="k1", value="1.5"), KeyValue(key="k2", value="x")]
        assert wait_until(lambda: pub.publish(msg) or bool(statuses))
        entry = statuses[-1]
        assert (entry.topic, entry.level, entry.name, entry.message) == (
            f"/{uid}/status",
            1,
            "ctrl",
            "slow",
        )
        assert entry.values == (("k1", "1.5"), ("k2", "x"))
    finally:
        spin.stop()
        node.destroy_node()


def test_no_status_topics_no_subscription(ros, uid):
    node, _, _ = make_node(uid)
    assert not [s for s in node.subscriptions if s.msg_type is DiagnosticStatus]
    assert not [s for s in node.subscriptions if s.msg_type is Joy]  # virtual: no pad input
    node.destroy_node()
