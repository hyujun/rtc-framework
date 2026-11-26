"""ROS side of the teleop input (rclpy, no Qt).

Threading: ``TeleopInputNode`` is spun by ``SpinThread`` (its own executor
thread). Its callbacks only hand data over: an upstream pad Joy goes into the
locked ``PadRelay``, a status message and a service response go out through
the ``on_status`` / ``on_service_result`` callables, which the GUI binds to
Qt signals (queued to the GUI thread). ``step`` — publish one Joy, decide the
service calls — runs on the caller's thread: the GUI thread's publish timer,
or the test. ``VirtualPad``, ``ButtonActions`` and ``last_frame`` are touched
by that thread only.

Every service call is ``call_async``: a server that never answers cannot
stall the publish tick. A call to a server that is not up is not sent; it is
reported as unavailable instead. ``pending`` counts the calls sent and not
answered yet; it is the one value both threads touch, so it locks.

The controller manager's active controller and E-STOP state are read from
its latched topics and go to the window through ``on_cm_state``.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import fields

import rclpy
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rtc_msgs.srv import ClearEstop, ResetFault, SwitchController
from rtc_teleop_input.layout import GamepadLayout, LayoutError, load_layout
from rtc_teleop_input.model import (
    ButtonActions,
    ConfigError,
    HelpCombo,
    HoldEntry,
    PadFrame,
    PadRelay,
    StatusEntry,
    TeleopConfig,
    VirtualPad,
    released_frame,
    validate_config,
)

NODE_NAME = "teleop_input"
# ARCH-6: depth 1 everywhere. The Joy output and the pad input keep the
# default (reliable) as the joy package does. Status is read best-effort,
# which matches a reliable or a best-effort publisher.
_JOY_QOS = QoSProfile(depth=1)
_STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# The CM publishes its state latched (reliable, transient local): a volatile
# or best-effort reader would never get the sample sent before it joined.
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# switch_controller's own wait for the RT loop to take the new controller
SWITCH_TIMEOUT_S = 1

StatusCallback = Callable[[StatusEntry], None]
# (call key: "estop" or a HoldEntry.key, ok, message)
ServiceCallback = Callable[[str, bool, str], None]
# ("active", controller name) | ("estop", latched)
CmStateCallback = Callable[[str, object], None]


def _declare_config(node: Node) -> TeleopConfig:
    """Declares one parameter per ``TeleopConfig`` field, defaults from the dataclass."""
    defaults = TeleopConfig()
    values = {}
    for f in fields(TeleopConfig):
        default = getattr(defaults, f.name)
        if isinstance(default, tuple):
            if default:
                node.declare_parameter(f.name, list(default))
                values[f.name] = tuple(node.get_parameter(f.name).value)
            else:
                # an empty list has no type to infer: declare the type only
                node.declare_parameter(f.name, Parameter.Type.STRING_ARRAY)
                try:
                    value = node.get_parameter(f.name).value
                except rclpy.exceptions.ParameterUninitializedException:
                    value = None
                values[f.name] = tuple(value or ())
        elif isinstance(default, float):
            # accept 50 as well as 50.0 from a launch file
            node.declare_parameter(f.name, default, ParameterDescriptor(dynamic_typing=True))
            value = node.get_parameter(f.name).value
            if not isinstance(value, int | float) or isinstance(value, bool):
                raise ConfigError(f"parameter '{f.name}' must be a number, got {value!r}")
            values[f.name] = float(value)
        else:
            node.declare_parameter(f.name, default)
            values[f.name] = node.get_parameter(f.name).value
    return TeleopConfig(**values)


class TeleopInputNode(Node):
    def __init__(
        self,
        *,
        on_status: StatusCallback | None = None,
        on_service_result: ServiceCallback | None = None,
        on_cm_state: CmStateCallback | None = None,
        parameter_overrides: list[Parameter] | None = None,
        layout: GamepadLayout | None = None,
    ) -> None:
        super().__init__(NODE_NAME, parameter_overrides=parameter_overrides or [])
        try:
            self.layout = layout if layout is not None else load_layout()
        except LayoutError as exc:
            raise ConfigError(str(exc)) from exc
        self.config = _declare_config(self)
        self.bindings = validate_config(self.config, self.layout)
        if self.config.input == "pad" and self.config.pad_topic == self.config.output_topic:
            raise ConfigError("pad_topic and output_topic must differ (the relay would loop)")
        self._on_status = on_status
        self._on_service_result = on_service_result
        self._on_cm_state = on_cm_state

        self.virtual = VirtualPad(self.layout, self.bindings, self.config.deflection)
        self.relay: PadRelay | None = None
        self.actions = ButtonActions(self.layout, self.config)
        # the pad combination toggles the help; only a real pad can press it
        # (a key pushes a trigger to -deflection, short of HELP_AXIS_PRESSED)
        self._help_combo = HelpCombo(
            self.layout, self.config.help_combo if self.config.input == "pad" else ()
        )
        self.help_toggles = 0
        self.last_frame: PadFrame = released_frame(self.layout, self.config.input)
        # calls sent, per call key ("estop" or HoldEntry.key); read by the window
        self.calls_sent: dict[str, int] = {}
        self._pending: dict[str, int] = {}
        self._pending_lock = threading.Lock()
        # an E-STOP pressed while a clear call is in flight is sent again after
        # the clear's reply: the server may run the E-STOP first (answering
        # "latched") and the clear after it, which would lower it
        self._estop_after_clear = False

        self._joy_pub = self.create_publisher(Joy, self.config.output_topic, _JOY_QOS)
        if self.config.input == "pad":
            self.relay = PadRelay(self.layout, self.config.pad_timeout_s)
            self.create_subscription(Joy, self.config.pad_topic, self._on_pad, _JOY_QOS)
        for topic in self.config.status_topics:
            self.create_subscription(
                DiagnosticStatus,
                topic,
                lambda msg, topic=topic: self._on_status_msg(topic, msg),
                _STATUS_QOS,
            )
        for topic, msg_type, kind in (
            (self.config.active_controller_topic, String, "active"),
            (self.config.estop_status_topic, Bool, "estop"),
        ):
            if topic:
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, kind=kind: self._on_cm_msg(kind, msg.data),
                    _LATCHED_QOS,
                )
        self._estop_client = self.create_client(Trigger, self.config.estop_service)
        self._reset_client = self.create_client(ResetFault, self.config.reset_service)
        # one client per service the hold buttons call (by HoldEntry.key)
        self._hold_clients = {}
        self._clients_by_name = {}
        for hold in self.actions.holds:
            if hold.kind == "switch":
                self._hold_clients[hold.key] = self._client(
                    SwitchController, self.config.switch_service
                )
            elif hold.kind == "clear":
                self._hold_clients[hold.key] = self._client(
                    ClearEstop, self.config.clear_estop_service
                )
            elif hold.kind == "trigger":
                self._hold_clients[hold.key] = self._client(Trigger, hold.target)
        self.get_logger().info(
            f"input '{self.config.input}' -> {self.config.output_topic} at "
            f"{self.config.rate_hz:g} Hz; E-STOP '{self.config.estop_button}', reset "
            + (
                f"'{self.config.reset_button}' for '{self.config.reset_controller_name}'"
                if self.config.reset_enabled
                else "disabled (reset_controller_name is empty)"
            )
            + "".join(
                f"; '{h.button}' holds {h.key}" for h in self.actions.holds if h.kind != "reset"
            )
        )

    def _client(self, srv_type, name):
        """One client per (type, service): two switch buttons share the switch client."""
        if (srv_type, name) not in self._clients_by_name:
            self._clients_by_name[(srv_type, name)] = self.create_client(srv_type, name)
        return self._clients_by_name[(srv_type, name)]

    @property
    def estop_calls_sent(self) -> int:
        return self.calls_sent.get("estop", 0)

    @property
    def reset_calls_sent(self) -> int:
        return self.calls_sent.get("reset", 0)

    def pending(self, key: str) -> int:
        """Calls of ``key`` sent and not answered yet."""
        with self._pending_lock:
            return self._pending.get(key, 0)

    # ── executor thread ─────────────────────────────────────────────────

    def _on_cm_msg(self, kind: str, value) -> None:
        if self._on_cm_state is not None:
            self._on_cm_state(kind, value)

    def _on_pad(self, msg: Joy) -> None:
        self.relay.on_upstream(msg.axes, msg.buttons, time.monotonic())

    def _on_status_msg(self, topic: str, msg: DiagnosticStatus) -> None:
        if self._on_status is None:
            return
        level = msg.level[0] if isinstance(msg.level, bytes) else int(msg.level)
        self._on_status(
            StatusEntry(
                topic=topic,
                name=msg.name,
                level=level,
                message=msg.message,
                values=tuple((kv.key, kv.value) for kv in msg.values),
                received_s=time.monotonic(),
            )
        )

    def _report(self, kind: str, ok: bool, message: str) -> None:
        if self._on_service_result is not None:
            self._on_service_result(kind, ok, message)

    def _on_done(self, kind: str, future) -> None:
        with self._pending_lock:
            self._pending[kind] -= 1
        try:
            response = future.result()
        except Exception as exc:  # the call failed in the middleware
            self._report(kind, False, f"call failed: {exc}")
            return
        # std_srvs/Trigger answers in `success` (for E-STOP: the latch state);
        # ResetFault, SwitchController and ClearEstop in `ok`
        trigger = kind == "estop" or kind.startswith("trigger:")
        ok = response.success if trigger else response.ok
        self._report(kind, bool(ok), response.message)

    # ── caller (GUI) thread ─────────────────────────────────────────────

    def step(self, now: float) -> PadFrame:
        """One publish tick: publish the frame, then send the calls it asks for."""
        if self.relay is not None:
            frame = self.relay.frame(now)
        else:
            frame = self.virtual.frame()
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame.source
        msg.axes = list(frame.axes)
        msg.buttons = list(frame.buttons)
        self._joy_pub.publish(msg)
        self.last_frame = frame
        if self._help_combo.step(frame):
            self.help_toggles += 1
        actions = self.actions.step(frame, now)
        for _ in range(actions.estop_calls):
            self._call("estop", self._estop_client, Trigger.Request())
        if actions.estop_calls and self.pending("clear"):
            self._estop_after_clear = True
        elif self._estop_after_clear and not self.pending("clear"):
            self._estop_after_clear = False
            self._call("estop", self._estop_client, Trigger.Request())
        for hold in actions.fired:
            self._send_hold(hold)
        return frame

    def _send_hold(self, hold: HoldEntry) -> None:
        if hold.kind == "reset":
            request = ResetFault.Request()
            request.controller_name = hold.target
            self._call(hold.key, self._reset_client, request)
        elif hold.kind == "switch":
            request = SwitchController.Request()
            request.activate_controllers = [hold.target]
            request.strictness = SwitchController.Request.STRICT
            request.timeout = Duration(sec=SWITCH_TIMEOUT_S)
            self._call(hold.key, self._hold_clients[hold.key], request)
        elif hold.kind == "clear":
            request = ClearEstop.Request()
            request.reason_ack = self.config.clear_estop_reason
            self._call(hold.key, self._hold_clients[hold.key], request)
        else:
            self._call(hold.key, self._hold_clients[hold.key], Trigger.Request())

    def _call(self, kind: str, client, request) -> bool:
        """Sends one call, or reports the service missing; True when sent."""
        if not client.service_is_ready():
            self._report(kind, False, f"{client.srv_name} is not available")
            return False
        self.calls_sent[kind] = self.calls_sent.get(kind, 0) + 1
        with self._pending_lock:
            self._pending[kind] = self._pending.get(kind, 0) + 1
        client.call_async(request).add_done_callback(lambda f: self._on_done(kind, f))
        return True


class SpinThread:
    """Spins one node on a private executor in a daemon thread.

    ``stop`` ends the loop and joins the thread BEFORE shutting the executor
    down: Jazzy's ``Executor.shutdown`` does not wait for a running callback
    (its wait is skipped once the shutdown flag is set) and tears the wait set
    down under it. rtc_tools' joint_command_ros.SpinThread is the same wrapper
    without this ordering; it is not imported, to keep rtc_tools (and its
    analysis dependencies) out of this package.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def _spin(self) -> None:
        try:
            while not self._stop.is_set():
                self._executor.spin_once(timeout_sec=0.05)
        except Exception:  # the context was shut down under the loop
            if rclpy.ok() and not self._stop.is_set():
                raise

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._executor.shutdown(timeout_sec=0)
