"""TeleopInputNode's hold buttons and controller-manager state on real rclpy."""

from __future__ import annotations

import threading
import time

import pytest
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rtc_msgs.srv import ClearEstop, SwitchController
from rtc_teleop_input.teleop_ros import SpinThread, TeleopInputNode

DT = 0.02
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
KEYS = [
    "leftshoulder:Space",
    "b:Escape",
    "back:Backspace",
    "y:1",
    "start:Return",
    "x:R",
]


def make_node(uid, **params):
    params.setdefault("output_topic", f"/{uid}/joy")
    params.setdefault("estop_service", f"/{uid}/trigger_estop")
    params.setdefault("reset_service", f"/{uid}/reset_fault")
    params.setdefault("switch_service", f"/{uid}/switch")
    params.setdefault("clear_estop_service", f"/{uid}/clear")
    params.setdefault("switch_buttons", ["y:ctrl_a"])
    params.setdefault("clear_estop_button", "start")
    params.setdefault("clear_estop_reason", "the_reason")
    params.setdefault("trigger_buttons", [f"x:/{uid}/recover"])
    params.setdefault("key_buttons", KEYS)
    params.setdefault("active_controller_topic", f"/{uid}/active")
    params.setdefault("estop_status_topic", f"/{uid}/estop")
    results = []
    states = []
    node = TeleopInputNode(
        parameter_overrides=[Parameter(k, value=v) for k, v in params.items()],
        on_service_result=lambda *r: results.append(r),
        on_cm_state=lambda *s: states.append(s),
    )
    return node, results, states


def wait_until(predicate, deadline_s=3.0):
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


@pytest.fixture
def peer(ros, uid):
    node = Node(f"peer_{uid}")
    spin = SpinThread(node)
    spin.start()
    yield node
    spin.stop()
    node.destroy_node()


@pytest.fixture
def spun(ros, uid):
    made = []

    def start(**params):
        node, results, states = make_node(uid, **params)
        spin = SpinThread(node)
        spin.start()
        made.append((node, spin))
        return node, results, states

    yield start
    for node, spin in made:
        spin.stop()
        node.destroy_node()


def hold(node, key, seconds=1.2):
    node.virtual.key_event(key, True, False)
    t = 0.0
    for _ in range(round(seconds / DT)):
        t += DT
        node.step(t)
    node.virtual.key_event(key, False, False)
    node.step(t + DT)


def test_switch_request_is_strict_with_a_timeout(uid, peer, spun):
    requests = []

    def serve(request, response):
        requests.append(request)
        response.ok = True
        response.message = "switched"
        return response

    peer.create_service(SwitchController, f"/{uid}/switch", serve)
    node, results, _ = spun()
    assert wait_until(lambda: node._hold_clients["switch:ctrl_a"].service_is_ready())
    hold(node, "1")
    assert wait_until(lambda: results == [("switch:ctrl_a", True, "switched")])
    (request,) = requests
    assert list(request.activate_controllers) == ["ctrl_a"]
    assert list(request.deactivate_controllers) == []
    assert request.strictness == SwitchController.Request.STRICT
    assert (request.timeout.sec, request.timeout.nanosec) == (1, 0)
    assert node.calls_sent == {"switch:ctrl_a": 1}
    assert node.pending("switch:ctrl_a") == 0


def test_clear_echoes_the_configured_reason_and_reports_ok_false(uid, peer, spun):
    acks = []

    def serve(request, response):
        acks.append(request.reason_ack)
        response.ok = False  # e.g. lowered but not verified: not a "refusal"
        response.message = "lowered, NOT verified"
        return response

    peer.create_service(ClearEstop, f"/{uid}/clear", serve)
    node, results, _ = spun()
    assert wait_until(lambda: node._hold_clients["clear"].service_is_ready())
    hold(node, "Return")
    assert wait_until(lambda: results == [("clear", False, "lowered, NOT verified")])
    assert acks == ["the_reason"]


def test_trigger_reply_is_read_from_success(uid, peer, spun):
    peer.create_service(
        Trigger,
        f"/{uid}/recover",
        lambda req, resp: setattr(resp, "success", True) or setattr(resp, "message", "up") or resp,
    )
    node, results, _ = spun()
    assert wait_until(lambda: node._hold_clients[f"trigger:/{uid}/recover"].service_is_ready())
    hold(node, "R")
    assert wait_until(lambda: results == [(f"trigger:/{uid}/recover", True, "up")])


def test_pending_counts_until_the_reply(uid, peer, spun):
    entered, release = threading.Event(), threading.Event()

    def slow(request, response):
        entered.set()
        release.wait(5.0)
        response.success = True
        return response

    peer.create_service(Trigger, f"/{uid}/recover", slow)
    node, results, _ = spun()
    key = f"trigger:/{uid}/recover"
    try:
        assert wait_until(lambda: node._hold_clients[key].service_is_ready())
        hold(node, "R")
        assert entered.wait(3.0)
        assert node.pending(key) == 1
    finally:
        release.set()
    assert wait_until(lambda: node.pending(key) == 0 and len(results) == 1)


def test_unavailable_hold_service_is_reported_not_sent(uid, ros):
    node, results, _ = make_node(uid)
    hold(node, "1")
    assert node.calls_sent == {}
    assert results == [("switch:ctrl_a", False, f"/{uid}/switch is not available")]
    node.destroy_node()


def test_cm_state_subscriptions_are_latched(uid, peer, spun):
    # published BEFORE the node exists: only a transient-local reader gets them
    active = peer.create_publisher(String, f"/{uid}/active", LATCHED)
    estop = peer.create_publisher(Bool, f"/{uid}/estop", LATCHED)
    active.publish(String(data="ctrl_b"))
    estop.publish(Bool(data=True))
    node, _, states = spun()
    for topic in (f"/{uid}/active", f"/{uid}/estop"):
        (info,) = [
            s for s in node.subscriptions if s.topic_name == topic
        ]  # one subscription per topic
        qos = info.qos_profile
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.depth == 1
    assert wait_until(lambda: ("active", "ctrl_b") in states and ("estop", True) in states)


def test_empty_cm_topics_subscribe_nothing(uid, ros):
    node, _, _ = make_node(uid, active_controller_topic="", estop_status_topic="")
    assert not [s for s in node.subscriptions if s.msg_type in (String, Bool)]
    node.destroy_node()


def _combo_frame(pad_pub, left, right):
    msg = Joy()
    msg.axes = [0.0, 0.0, 0.0, 0.0, left, right]
    msg.buttons = [0] * 21
    pad_pub.publish(msg)


def test_help_combo_toggles_in_pad_mode_only(uid, peer, spun):
    node, _, _ = spun(
        input="pad", pad_topic=f"/{uid}/pad", help_combo=["triggerleft", "triggerright"]
    )
    pub = peer.create_publisher(Joy, f"/{uid}/pad", QoSProfile(depth=1))
    assert wait_until(lambda: node.count_publishers(f"/{uid}/pad") == 1)

    def reach(left, right, ticks=10):
        # keep the pad talking (a silent pad reads released after 0.25 s)
        def seen():
            _combo_frame(pub, left, right)
            time.sleep(DT)
            return node.step(time.monotonic()).axes[4:] == (left, right)

        assert wait_until(seen, 2.0)
        for _ in range(ticks):
            _combo_frame(pub, left, right)
            time.sleep(DT)
            node.step(time.monotonic())

    reach(-1.0, -1.0)
    assert node.help_toggles == 1  # held for ten more ticks: still one
    reach(-1.0, 0.0)
    reach(-1.0, -1.0)
    assert node.help_toggles == 2

    virtual, _, _ = make_node(f"{uid}_v", help_combo=["triggerleft", "triggerright"])
    virtual.virtual.mouse_control("triggerleft", True)
    virtual.virtual.mouse_control("triggerright", True)
    virtual.step(0.0)
    assert virtual.help_toggles == 0
    virtual.destroy_node()


def test_an_estop_during_an_inflight_clear_is_sent_again_after_it(uid, peer, spun):
    entered, release = threading.Event(), threading.Event()
    order = []

    def slow_clear(request, response):
        entered.set()
        release.wait(5.0)
        order.append("clear")
        response.ok = True
        return response

    def estop(request, response):
        order.append("estop")
        response.success = True
        return response

    peer.create_service(ClearEstop, f"/{uid}/clear", slow_clear)
    peer.create_service(Trigger, f"/{uid}/trigger_estop", estop)
    node, results, _ = spun()
    try:
        assert wait_until(lambda: node._hold_clients["clear"].service_is_ready())
        assert wait_until(lambda: node._estop_client.service_is_ready())
        hold(node, "Return")
        assert entered.wait(3.0)
        node.actions.request_estop()  # while the clear is in flight
        node.step(100.0)
        node.step(100.02)
        assert node.estop_calls_sent == 1
    finally:
        release.set()
    assert wait_until(lambda: node.pending("clear") == 0)
    node.step(100.04)  # the tick after the clear's reply
    assert wait_until(lambda: order.count("estop") == 2)
    # the last call the server ran is an E-STOP, whatever it ran first
    assert order[-1] == "estop"
    assert node.estop_calls_sent == 2
