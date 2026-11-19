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
reported as unavailable instead.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import fields

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from rtc_msgs.srv import ResetFault
from rtc_teleop_input.layout import GamepadLayout, LayoutError, load_layout
from rtc_teleop_input.model import (
    ButtonActions,
    ConfigError,
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

StatusCallback = Callable[[StatusEntry], None]
# (service kind "estop" | "reset", ok, message)
ServiceCallback = Callable[[str, bool, str], None]


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

        self.virtual = VirtualPad(self.layout, self.bindings, self.config.deflection)
        self.relay: PadRelay | None = None
        self.actions = ButtonActions(self.layout, self.config)
        self.last_frame: PadFrame = released_frame(self.layout, self.config.input)
        # calls sent (not answered); read by tests and the window
        self.estop_calls_sent = 0
        self.reset_calls_sent = 0

        self._joy_pub = self.create_publisher(Joy, self.config.output_topic, _JOY_QOS)
        self._input_subs = []
        if self.config.input == "pad":
            self.relay = PadRelay(self.layout, self.config.pad_timeout_s)
            self._input_subs.append(
                self.create_subscription(Joy, self.config.pad_topic, self._on_pad, _JOY_QOS)
            )
        for topic in self.config.status_topics:
            self._input_subs.append(
                self.create_subscription(
                    DiagnosticStatus,
                    topic,
                    lambda msg, topic=topic: self._on_status_msg(topic, msg),
                    _STATUS_QOS,
                )
            )
        self._estop_client = self.create_client(Trigger, self.config.estop_service)
        self._reset_client = self.create_client(ResetFault, self.config.reset_service)
        self.get_logger().info(
            f"input '{self.config.input}' -> {self.config.output_topic} at "
            f"{self.config.rate_hz:g} Hz; E-STOP '{self.config.estop_button}', reset "
            + (
                f"'{self.config.reset_button}' for '{self.config.reset_controller_name}'"
                if self.config.reset_enabled
                else "disabled (reset_controller_name is empty)"
            )
        )

    # ── executor thread ─────────────────────────────────────────────────

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

    def _on_estop_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # the call failed in the middleware
            self._report("estop", False, f"call failed: {exc}")
            return
        # Trigger.success is the latch state when the server answered
        self._report("estop", bool(response.success), response.message)

    def _on_reset_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._report("reset", False, f"call failed: {exc}")
            return
        self._report("reset", bool(response.ok), response.message)

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
        actions = self.actions.step(frame, now)
        for _ in range(actions.estop_calls):
            self._call_estop()
        if actions.reset_call:
            self._call_reset()
        return frame

    def _call_estop(self) -> None:
        if not self._estop_client.service_is_ready():
            self._report("estop", False, f"{self.config.estop_service} is not available")
            return
        self.estop_calls_sent += 1
        self._estop_client.call_async(Trigger.Request()).add_done_callback(self._on_estop_done)

    def _call_reset(self) -> None:
        if not self._reset_client.service_is_ready():
            self._report("reset", False, f"{self.config.reset_service} is not available")
            return
        self.reset_calls_sent += 1
        request = ResetFault.Request()
        request.controller_name = self.config.reset_controller_name
        self._reset_client.call_async(request).add_done_callback(self._on_reset_done)


class SpinThread:
    """Spins one node on a private executor in a daemon thread."""

    def __init__(self, node: Node) -> None:
        self._executor = SingleThreadedExecutor()
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
