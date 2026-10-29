"""ROS side of the hand joint-command GUI (rclpy, no Qt).

Threading: ``JointCommandNode`` is spun by its own executor thread, and its
callbacks only fill a locked cache — the latest ``joint_states`` message, the
latest ``link_status`` and the foreign-publisher flag per hand. The decisions
and the publishing run on the caller's thread through ``CommandLoop.step``
(the Qt main thread in the GUI, a plain loop in a headless check), so the
controllers of ``joint_command_model`` are never touched by two threads.

``joint_states`` arrives at the hand's loop rate (hundreds of Hz per hand)
but is read once per publish period, so it is subscribed raw, only the
message a step actually reads is deserialized, and ``SpinThread`` prefers
rclpy's EventsExecutor.

Discovery (``DISCOVERY_PERIOD_S``): a candidate hand of ``hand_topics`` is
attached once its ``joint_states`` has a publisher AND its ``joint_command``
has a subscriber; an attached hand stays attached (a vanished hand goes stale
and auto-disables). Other publishers on an attached hand's ``joint_command``
are checked every ``FOREIGN_CHECK_PERIOD_S`` — after DDS discovery has shown
them, which this node does not control.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rtc_msgs.msg import JointCommand
from rtc_tools.gui.joint_command_model import (
    GuiConfig,
    HandController,
    HandSample,
    HandTopics,
    hand_topics,
    load_config,
    normalize_namespace,
)

DISCOVERY_PERIOD_S = 1.0
FOREIGN_CHECK_PERIOD_S = 0.1

# Matches the hand node: sensor stream and command are BEST_EFFORT, the link
# status is latched (RELIABLE + TRANSIENT_LOCAL). Depth 1 everywhere (ARCH-6).
_BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass
class HandInputs:
    """What the executor thread last saw for one attached hand."""

    topics: HandTopics
    # serialized sensor_msgs/JointState, its receive time and a counter that
    # tells the reader whether it is new
    state_raw: bytes | None = None
    state_received_s: float = 0.0
    state_seq: int = 0
    link_ok: bool | None = None
    foreign_publisher: bool = False


class JointCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_command_gui")
        self.declare_parameter("config", "")
        self.declare_parameter("namespace", "")
        config = load_config(self.get_parameter("config").value)
        namespace = self.get_parameter("namespace").value
        if namespace:
            config = replace(config, namespace=normalize_namespace(namespace))
        self.config: GuiConfig = config
        self.candidates = hand_topics(config)

        self._lock = threading.Lock()
        self._inputs: dict[str, HandInputs] = {}
        self._command_pubs = {}
        self._input_subs = []
        self._discovery = self.create_timer(DISCOVERY_PERIOD_S, self._discover)
        self._foreign_check = self.create_timer(FOREIGN_CHECK_PERIOD_S, self._check_foreign)
        self.get_logger().info(
            "looking for hands: " + ", ".join(t.state_topic for t in self.candidates)
        )

    # ── executor thread ─────────────────────────────────────────────────

    def _discover(self) -> None:
        for topics in self.candidates:
            if topics.label not in self._command_pubs and (
                self.count_publishers(topics.state_topic) > 0
                and self.count_subscribers(topics.command_topic) > 0
            ):
                self._attach(topics)

    def _check_foreign(self) -> None:
        for topics in self.candidates:
            if topics.label in self._command_pubs:
                foreign = self._foreign_publisher(topics.command_topic)
                with self._lock:
                    self._inputs[topics.label].foreign_publisher = foreign

    def _attach(self, topics: HandTopics) -> None:
        # The publisher exists before the hand shows up in snapshot(); the
        # callbacks below run on this same executor thread, so none can fire
        # before the entry they write is in place.
        label = topics.label
        self._command_pubs[label] = self.create_publisher(
            JointCommand, topics.command_topic, _BEST_EFFORT
        )
        self._input_subs.append(
            self.create_subscription(
                JointState,
                topics.state_topic,
                lambda msg, label=label: self._on_joint_states(label, msg),
                _BEST_EFFORT,
                raw=True,
            )
        )
        self._input_subs.append(
            self.create_subscription(
                Bool,
                topics.link_topic,
                lambda msg, label=label: self._on_link(label, msg),
                _LATCHED,
            )
        )
        with self._lock:
            self._inputs[label] = HandInputs(topics)
        self.get_logger().info(f"hand '{label}' found: {topics.state_topic}")

    def _foreign_publisher(self, topic: str) -> bool:
        # A second GUI carries this node's name too, so a name match counts as
        # ours only once (this node has one publisher per command topic).
        me = (self.get_name(), self.get_namespace())
        names = [(i.node_name, i.node_namespace) for i in self.get_publishers_info_by_topic(topic)]
        return any(n != me for n in names) or names.count(me) > 1

    def _on_joint_states(self, label: str, raw: bytes) -> None:
        received_s = time.monotonic()
        with self._lock:
            inputs = self._inputs[label]
            inputs.state_raw = raw
            inputs.state_received_s = received_s
            inputs.state_seq += 1

    def _on_link(self, label: str, msg: Bool) -> None:
        with self._lock:
            self._inputs[label].link_ok = bool(msg.data)

    # ── caller thread ───────────────────────────────────────────────────

    def snapshot(self) -> dict[str, HandInputs]:
        """A copy of every attached hand's latest inputs."""
        with self._lock:
            return {label: replace(i) for label, i in self._inputs.items()}

    def publish(self, label: str, names: tuple[str, ...], values_rad: list[float]) -> None:
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(names)
        msg.values = list(values_rad)
        msg.command_type = "position"
        self._command_pubs[label].publish(msg)


def decode_joint_states(raw: bytes, received_s: float) -> HandSample:
    msg = deserialize_message(raw, JointState)
    return HandSample(
        names=tuple(msg.name),
        position_rad=tuple(msg.position),
        effort=tuple(msg.effort),
        received_s=received_s,
    )


class CommandLoop:
    """Feeds the controllers from the node's cache and publishes their output."""

    def __init__(self, node: JointCommandNode) -> None:
        self.node = node
        self.controllers: dict[str, HandController] = {}
        self._fed_seq: dict[str, int] = {}

    def step(self, now_s: float) -> list[str]:
        """One publish period; returns the labels of hands seen for the first time."""
        new = []
        for label, inputs in self.node.snapshot().items():
            ctrl = self.controllers.get(label)
            if ctrl is None:
                ctrl = self.controllers[label] = HandController(label, self.node.config)
                self._fed_seq[label] = 0
                new.append(label)
            if inputs.state_seq != self._fed_seq[label]:
                was_ready = ctrl.ready
                ctrl.on_sample(decode_joint_states(inputs.state_raw, inputs.state_received_s))
                self._fed_seq[label] = inputs.state_seq
                if ctrl.ready and not was_ready:
                    self._report_layout(ctrl)
            ctrl.set_link(inputs.link_ok)
            ctrl.set_foreign_publisher(inputs.foreign_publisher)
            values = ctrl.tick(now_s)
            if values is not None:
                self.node.publish(label, ctrl.joint_names, values)
        return new

    def _report_layout(self, ctrl: HandController) -> None:
        log = self.node.get_logger()
        log.info(f"hand '{ctrl.label}': {len(ctrl.joint_names)} joints {list(ctrl.joint_names)}")
        if ctrl.unconfigured_joints():
            log.warning(
                f"hand '{ctrl.label}': no ranges_deg for {ctrl.unconfigured_joints()} — "
                "using the first measured position ± the default half-width"
            )
        if ctrl.unknown_yaml_joints():
            log.warning(
                f"hand '{ctrl.label}': ranges_deg names unknown joints "
                f"{ctrl.unknown_yaml_joints()} — ignored"
            )


def _make_executor():
    # rclpy's EventsExecutor (experimental in Jazzy, C++ dispatch) takes two
    # hands' joint_states at a fraction of SingleThreadedExecutor's CPU —
    # 3.5 % vs 18.5 % of a core for 2 x 500 Hz in a SIL measurement.
    try:
        from rclpy.experimental import EventsExecutor
    except ImportError:
        return SingleThreadedExecutor()
    return EventsExecutor()


class SpinThread:
    """Spins one node on a private executor in a daemon thread."""

    def __init__(self, node: Node) -> None:
        self._executor = _make_executor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # a shutdown race ends the spin
            if rclpy.ok():
                raise

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=2.0)
