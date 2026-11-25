"""The ROS side of the CLI: one node, the ``Io`` the orchestrator drives.

The loop never blocks. Callbacks stamp their arrival time themselves, and
``observe`` first handles every callback that is ready, so a measurement is
never judged stale because this process was busy. Services are futures with a
deadline; one that times out is removed from the client, so a 10 h run does
not pile them up. The graph (nodes, publisher counts) is read at 2 Hz.
"""

from __future__ import annotations

import contextlib
import fcntl
import os
import select
import shutil
import signal
import sys
import time
from collections.abc import Sequence
from datetime import datetime

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from rtc_msgs.msg import RobotTarget
from rtc_tools.analysis.ros_stop import StopFlag, stop_handlers

from ..demo_gui.preset_toggle import atomic_write_json
from ..demo_gui.shipped import calibration_of
from . import env
from .files import FileProblem
from .orchestrator import EXIT_NOT_READY, Orchestrator, RunConfig
from .readiness import CONTROLLER, STALE_S, Observation

CM_NODE = "integrated_rt_controller"
PARAM_NODE = f"/{CONTROLLER}/{CONTROLLER}"
# The hand driver of robot_p2.launch.py (namespace ""): its calibration is read
# from the running node (readiness module docstring).
DRIVER_NODE = "/udp_hand_node"
CALIBRATION_NAMES = ("joint_state_names", "joint_direction_signs", "joint_position_offsets_deg")
ACTIVE_TOPIC = "/rtc_cm/active_controller_name"
ESTOP_TOPIC = "/system/estop_status"
# The hand driver's link_status per profile (config/<profile>/udp_hand_node_p2.yaml
# link_status_topic; test_demo_sequence pins the two against the YAML).
LINK_TOPICS = {"p2": "/p2/right/link_status", "p2_left": "/p2/left/link_status"}
GRAPH_PERIOD_S = 0.5
PARAM_PERIOD_S = 1.0
PARAM_TIMEOUT_S = 1.0
LOOP_PERIOD_S = 0.02
# Every subscription is depth 1, so at most one sample per topic plus the three
# service answers can be waiting: this many non-blocking spins drain them all.
SPINS_PER_LOOK = 12


def _latched() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    return qos


class SequenceIo(Node):
    """``orchestrator.Io`` on rclpy. One instance per run."""

    def __init__(
        self, profile, robot: str, stop: StopFlag, summary_path: str, stack_log_root: str
    ):
        """``stack_log_root``: where the stack's launch writes its session
        folders (the workspace ``logging_data``) — the disk the checks watch."""
        super().__init__("demo_controller_sequence")
        self._profile, self._stop = profile, stop
        self._summary_path, self._log_root = summary_path, stack_log_root
        self._roster = tuple(profile.shape.hand_motor_names)
        group = profile.hand_group
        self._goal_topic = f"/{CONTROLLER}/{group}/joint_goal"
        other = profile.mirror_robot
        from ..demo_gui.discovery import ROBOT_PROFILES

        self._other_states = (
            f"/rtc_cm/{ROBOT_PROFILES[other].hand_group}/joint_states" if other else ""
        )
        self._link_topic = LINK_TOPICS.get(robot, "")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

        # This CLI's publisher first: the graph count of joint_goal publishers
        # then includes it, and exactly 1 means nobody else.
        self._goal_pub = self.create_publisher(RobotTarget, self._goal_topic, 1)
        self._joint_stamp: float | None = None
        self._stream_since: float | None = None
        self._positions: tuple[float, ...] = ()
        self._roster_ok: bool | None = None
        self._active: str | None = None
        self._estop: bool | None = None
        self._link: bool | None = None
        self._mode: str | None = None
        self._cap: float | None = None
        self._graph = {
            "cm": 0,
            "param": 0,
            "active": 0,
            "estop": 0,
            "link": 0,
            "goal": 0,
            "other": False,
        }
        self._graph_at = -1e9
        self._param_at = -1e9
        self._get_future = None
        self._get_deadline = 0.0
        self._driver = DriverCalibrationReader(profile.side)
        self.create_subscription(JointState, f"/rtc_cm/{group}/joint_states", self._on_joints, 1)
        self.create_subscription(String, ACTIVE_TOPIC, self._on_active, _latched())
        self.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, _latched())
        if self._link_topic:
            self.create_subscription(Bool, self._link_topic, self._on_link, _latched())
        self._get = self.create_client(GetParameters, f"{PARAM_NODE}/get_parameters")
        self._set = self.create_client(SetParameters, f"{PARAM_NODE}/set_parameters")
        self._driver_get = self.create_client(GetParameters, f"{DRIVER_NODE}/get_parameters")
        self._driver_future = None
        self._driver_deadline = 0.0
        self._driver_at = -1e9

    # ── Callbacks: stamp on arrival ──────────────────────────────────────────

    def _on_joints(self, msg: JointState) -> None:
        now = time.monotonic()
        if self._joint_stamp is None or now - self._joint_stamp > STALE_S:
            self._stream_since = now
        self._joint_stamp = now
        index = {n: i for i, n in enumerate(msg.name)}
        if len(msg.name) != len(self._roster) or set(index) != set(self._roster):
            self._roster_ok = False
            return
        if len(msg.position) != len(msg.name):
            self._roster_ok = False
            return
        self._roster_ok = True
        self._positions = tuple(float(msg.position[index[n]]) for n in self._roster)

    def _on_active(self, msg: String) -> None:
        self._active = msg.data

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)

    def _on_link(self, msg: Bool) -> None:
        self._link = bool(msg.data)

    # ── Io ───────────────────────────────────────────────────────────────────

    def now(self) -> float:
        return time.monotonic()

    def wall(self) -> str:
        return datetime.now().astimezone().isoformat(timespec="seconds")

    def spin_some(self, timeout_s: float) -> None:
        self._executor.spin_once(timeout_sec=timeout_s)

    def observe(self) -> Observation:
        for _ in range(SPINS_PER_LOOK):
            self._executor.spin_once(timeout_sec=0.0)
        now = time.monotonic()
        if now - self._graph_at >= GRAPH_PERIOD_S:
            self._graph_at = now
            self._read_graph()
        self._poll_params(now)
        self._poll_driver(now)
        g = self._graph
        age = None if self._joint_stamp is None else now - self._joint_stamp
        steady = (
            0.0
            if self._stream_since is None or age is None or age > STALE_S
            else now - self._stream_since
        )
        return Observation(
            now=now,
            cm_nodes=g["cm"],
            param_nodes=g["param"],
            active_pubs=g["active"],
            estop_pubs=g["estop"],
            active=self._active,
            estop=self._estop,
            joint_age=age,
            steady_s=steady,
            roster_ok=self._roster_ok,
            positions=self._positions,
            link_pubs=g["link"],
            link=self._link,
            goal_pubs=g["goal"],
            goal_matched=self._goal_pub.get_subscription_count(),
            mode=self._mode,
            cap=self._cap,
            other_hand=g["other"],
            driver_nodes=g["driver"],
            driver_calibration=self._driver.calibration,
            driver_problem=self._driver.problem,
        )

    def _read_graph(self) -> None:
        nodes = self.get_node_names_and_namespaces()
        self._graph = {
            "cm": sum(1 for name, _ in nodes if name == CM_NODE),
            "param": sum(1 for name, ns in nodes if f"{ns.rstrip('/')}/{name}" == PARAM_NODE),
            "active": self.count_publishers(ACTIVE_TOPIC),
            "estop": self.count_publishers(ESTOP_TOPIC),
            "link": self.count_publishers(self._link_topic) if self._link_topic else 0,
            "goal": self.count_publishers(self._goal_topic),
            "other": bool(self._other_states) and self.count_publishers(self._other_states) > 0,
            "driver": sum(1 for name, ns in nodes if f"{ns.rstrip('/')}/{name}" == DRIVER_NODE),
        }
        if self._graph["driver"] == 0:
            self._driver.reset()  # a restarted driver is read again

    def _poll_params(self, now: float) -> None:
        """``mode`` and the speed cap, read once a second (a set mode is the
        operator's; the cap is read-only but a restarted stack re-reads it)."""
        if self._get_future is not None:
            if self._get_future.done():
                self._take_params(self._get_future.result())
                self._get_future = None
            elif now > self._get_deadline:
                self._get.remove_pending_request(self._get_future)
                self._get_future = None
            return
        if now - self._param_at < PARAM_PERIOD_S or not self._get.service_is_ready():
            return
        self._param_at = now
        request = GetParameters.Request(names=["mode", "max_traj_velocity"])
        self._get_future = self._get.call_async(request)
        self._get_deadline = now + PARAM_TIMEOUT_S

    def _take_params(self, response) -> None:
        values = list(getattr(response, "values", []) or [])
        if len(values) != 2:
            # An undeclared name empties the whole answer: the controller is
            # not configured (yet) — both unknown.
            self._mode, self._cap = None, None
            return
        mode, cap = values
        self._mode = mode.string_value if mode.type == ParameterType.PARAMETER_STRING else None
        self._cap = cap.double_value if cap.type == ParameterType.PARAMETER_DOUBLE else None

    def _poll_driver(self, now: float) -> None:
        """The driver's calibration, until it has been read (one or two
        requests: the per-hand keys exist only under hand_sides)."""
        if self._driver_future is not None:
            if self._driver_future.done():
                self._driver.take(self._driver_future.result())
                self._driver_future = None
            elif now > self._driver_deadline:
                self._driver_get.remove_pending_request(self._driver_future)
                self._driver_future = None
            return
        names = self._driver.next_request()
        if (
            not names
            or now - self._driver_at < PARAM_PERIOD_S
            or not self._driver_get.service_is_ready()
        ):
            return
        self._driver_at = now
        self._driver_future = self._driver_get.call_async(GetParameters.Request(names=names))
        self._driver_deadline = now + PARAM_TIMEOUT_S

    def set_speed(self, speed: float) -> object:
        if not self._set.service_is_ready():
            return None
        value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(speed))
        request = SetParameters.Request(
            parameters=[ParameterMsg(name="trajectory_speed", value=value)]
        )
        return self._set.call_async(request)

    def speed_result(self, handle: object) -> bool | None:
        if not handle.done():
            return None
        result = handle.result()
        results = list(getattr(result, "results", []) or [])
        return len(results) == 1 and bool(results[0].successful)

    def drop_speed(self, handle: object) -> None:
        if not handle.done():
            self._set.remove_pending_request(handle)

    def publish(self, target_rad: Sequence[float]) -> None:
        msg = RobotTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.goal_type = "joint"
        msg.joint_names = list(self._roster)
        msg.joint_target = [float(v) for v in target_rad]
        self._goal_pub.publish(msg)

    def interrupts(self) -> int:
        return self._stop.interrupts

    def terminated(self) -> bool:
        return self._stop.terminated

    def read_line(self) -> str | None:
        """A line typed at the terminal, if one is complete; never waits."""
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        except (OSError, ValueError):
            return None
        if not ready:
            return None
        line = sys.stdin.readline()
        return line if line else None

    def say(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        for line in text.splitlines() or [""]:
            print(f"[{stamp}] {line}", flush=True)

    def environment(self) -> str:
        return env.describe(os.environ)

    def log_bytes(self) -> int:
        """Bytes in the newest session folder: what the stack is writing now."""
        from rtc_tools.utils.session_dir import list_session_dirs

        sessions = list_session_dirs(self._log_root)
        if not sessions:
            return 0
        total = 0
        for base, _, files in os.walk(os.path.join(self._log_root, sessions[-1])):
            for name in files:
                with contextlib.suppress(OSError):
                    total += os.path.getsize(os.path.join(base, name))
        return total

    def free_bytes(self) -> int:
        return shutil.disk_usage(self._log_root).free

    def write_summary(self, data: dict) -> None:
        atomic_write_json(self._summary_path, data, prefix=".sequence_run_")

    def close(self) -> None:
        self._executor.remove_node(self)
        self.destroy_node()


class DriverCalibrationReader:
    """The running hand driver's calibration from GetParameters answers.

    First the flat keys and ``hand_sides``; when hand_sides is set, then this
    side's ``<side>.`` keys, which the driver declares only in that mode (an
    undeclared name empties the whole answer). The keys are then picked as the
    driver picks them (``shipped.calibration_of``).
    """

    def __init__(self, side: str) -> None:
        self._side = side
        self.reset()

    def reset(self) -> None:
        self._flat: dict | None = None
        self.calibration = None
        self.problem = ""

    def next_request(self) -> list[str]:
        """The names to ask for next; [] once read."""
        if self.calibration is not None or self.problem:
            return []
        if self._flat is None:
            return ["hand_sides", *CALIBRATION_NAMES]
        return [f"{self._side}.{name}" for name in CALIBRATION_NAMES]

    def take(self, response) -> None:
        names = self.next_request()
        values = list(getattr(response, "values", []) or [])
        if not names or len(values) != len(names):
            return  # not declared yet (the driver is configuring): ask again
        answer = dict(zip(names, (parameter_value_to_python(v) for v in values), strict=True))
        if self._flat is None:
            sides = answer.get("hand_sides") or []
            if sides and self._side not in sides:
                # Its flat keys are another hand's: say which, not "mismatch".
                self.problem = (
                    f"it runs hand_sides {sides}, not '{self._side}' (wrong --robot, or the "
                    "bring-up's side:=?)"
                )
                return
            self._flat = answer
            if sides:
                return  # the per-hand keys next
            self._settle(answer)
            return
        prefix = f"{self._side}."
        self._settle({**self._flat, self._side: {k[len(prefix) :]: v for k, v in answer.items()}})

    def _settle(self, params: dict) -> None:
        try:
            self.calibration = calibration_of(params, self._side, DRIVER_NODE, DRIVER_NODE)
        except FileProblem as exc:
            self.problem = str(exc)


def _lock(root: str, robot: str):
    """One CLI per robot on this host (the graph check covers other hosts)."""
    folder = os.path.join(root, "sequence_runs")
    os.makedirs(folder, exist_ok=True)
    handle = open(os.path.join(folder, f".{robot}.lock"), "w")  # noqa: SIM115 — held for the run
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        return None
    return handle


def run(args, profile, inputs, root: str) -> int:
    """``root``: the workspace logging root — the summary and the lock, and the
    disk the checks watch (the stack's sessions are written there too)."""
    lock = _lock(root, args.robot)
    if lock is None:
        print(
            f"error: another demo_controller_sequence --robot {args.robot} is running",
            file=sys.stderr,
        )
        return EXIT_NOT_READY
    stamp = datetime.now().strftime("%y%m%dT%H%M%S")
    summary_path = os.path.join(root, "sequence_runs", f"{stamp}_{args.robot}.json")
    stop = StopFlag()
    with lock, stop_handlers(stop, signals=(signal.SIGINT, signal.SIGTERM, signal.SIGHUP)):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        io = SequenceIo(profile, args.robot, stop, summary_path, root)
        try:
            cfg = RunConfig(
                robot=args.robot,
                joint_names=tuple(profile.shape.hand_motor_names),
                inputs=inputs,
                duration_s=args.duration,
                startup_timeout_s=args.startup_timeout,
                hold_abort_after_s=args.hold_abort_after,
                assume_yes=args.yes,
            )
            orch = Orchestrator(io, cfg)
            io.say(f"summary: {summary_path}")
            while (outcome := orch.step()) is None:
                io.spin_some(LOOP_PERIOD_S)
            return outcome.code
        finally:
            io.close()
            rclpy.try_shutdown()
