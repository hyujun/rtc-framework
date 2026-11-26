"""``sim_recover`` -- stand a fallen simulated robot back up from one Trigger.

``/sim_recover/recover`` (std_srvs/Trigger) does what the ``reentry`` arm of
tools/standing_eval/run_arms.py does after the E-STOP clear: publish the joint
controller's stand goal, wait until its trajectory has finished, then call
``/sim/reset`` (the plant goes back to its keyframe). The service is for a sim
operator who has only a pad; it never touches a real robot's plant.

Refused (success=false, reason in ``message``) when: a recovery is already
running; the active controller is not ``joint_controller`` or is unknown; the
E-STOP is latched or unknown; no joint state of the group has arrived. After
the wait, an E-STOP latch or a controller switch aborts (no reset). With
``residual_tol`` > 0 a residual ``max|q - goal|`` above it is a failure too and
``/sim/reset`` is NOT called; the default 0 means no gate (a fallen robot's
joints can be blocked by the floor, and once the trajectory has ended the
controller's target already is the stand pose, so the reset is safe); the
residual is reported in the success message either way.

Wait = DemoJointController's own duration for the move from the MEASURED
positions (stand_pose.trajectory_duration, speed / velocity cap read from the
profile's controller YAML) + ``margin_s``, counted in SIM time: the trajectory
advances per controller tick, and under lock-step with rtf < 1 sim time runs
slower than the wall clock (the sim publishes no /clock). The sim's
``/sim/time`` (std_msgs/Float64, QoS(1)) is the clock; no sample yet refuses.
Wall guard: the call fails (no reset) once wall time passes
max(10 s, 10 x the wait), so a paused or stalled sim cannot hang the service.
The YAML values are the shipped ones; a runtime change of
``robot_trajectory_speed`` is not seen.

Threading: a MultiThreadedExecutor. The service handler blocks (it waits out
the trajectory), so it owns a ReentrantCallbackGroup; the latched-status and
joint-state subscriptions share a MutuallyExclusive group and the /sim/reset
client has its own, so a blocked handler never starves the data that it waits
on. The state they write is guarded by one lock. Only one recovery runs: a
second request is refused, not queued.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from integrated_bringup.stand_pose import stand_pose, trajectory_duration, trajectory_limits
from rtc_msgs.msg import RobotTarget

_POLL_S = 0.02
_SUBSCRIBER_WAIT_S = 1.0
_RESET_TIMEOUT_S = 10.0
# Wall guard of the wait: max(10 s, 10 x the sim-time wait).
_WALL_GUARD_MIN_S = 10.0
_WALL_GUARD_FACTOR = 10.0


def refusal(
    *,
    active: str | None,
    estop: bool | None,
    joint_controller: str,
    busy: bool,
    have_joints: bool,
    have_sim_time: bool = True,
) -> str | None:
    """Why a recovery must not start, or None. Unknown (None) is refused."""
    if busy:
        return "a recovery is already running"
    if estop is None:
        return "the E-STOP status is unknown (no /system/estop_status sample yet)"
    if estop:
        return "the E-STOP is latched; clear it first"
    if active is None:
        return "the active controller is unknown (no /rtc_cm/active_controller_name sample yet)"
    if active != joint_controller:
        return f"the active controller is '{active}', not '{joint_controller}'"
    if not have_joints:
        return "no joint state has been received yet"
    if not have_sim_time:
        return "no sim time yet (no sample on the sim time topic)"
    return None


def max_abs_diff(a: list[float], b: list[float]) -> float:
    return max((abs(x - y) for x, y in zip(a, b, strict=True)), default=0.0)


def _latched() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class SimRecover(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("sim_recover", **node_kwargs)
        self._profile = self.declare_parameter("profile", "g1_loco").value
        profile_dir = self.declare_parameter("profile_dir", "").value
        self._joint_controller = self.declare_parameter(
            "joint_controller", "demo_joint_controller"
        ).value
        self._group = self.declare_parameter("group", "g1").value
        self._margin_s = float(self.declare_parameter("margin_s", 0.5).value)
        # rad; 0 = no gate (see the module docstring), > 0 = refuse to reset when
        # max|q - goal| after the wait is larger.
        self._residual_tol = float(self.declare_parameter("residual_tol", 0.0).value)
        sim_time_topic = self.declare_parameter("sim_time_topic", "/sim/time").value
        self._reset_service = self.declare_parameter("sim_reset_service", "/sim/reset").value

        pdir = Path(profile_dir) if profile_dir else self._share_profile_dir(self._profile)
        self._names, self._pose = stand_pose(pdir, self._joint_controller, self._group)
        self._speed, self._max_vel = trajectory_limits(pdir, self._joint_controller)

        self._lock = threading.Lock()
        self._active: str | None = None
        self._estop: bool | None = None
        self._positions: dict[str, float] = {}
        self._busy = False
        self._sim_time: float | None = None

        data_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            String, "/rtc_cm/active_controller_name", self._on_active, _latched(),
            callback_group=data_group,
        )  # fmt: skip
        self.create_subscription(
            Bool, "/system/estop_status", self._on_estop, _latched(), callback_group=data_group
        )
        self.create_subscription(
            JointState,
            f"/rtc_cm/{self._group}/joint_states",
            self._on_joints,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
            callback_group=data_group,
        )
        # The sim publishes /sim/time with rclcpp::QoS(1): reliable, volatile.
        self.create_subscription(
            Float64,
            sim_time_topic,
            self._on_sim_time,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
            callback_group=data_group,
        )
        self._goal_pub = self.create_publisher(
            RobotTarget,
            f"/{self._joint_controller}/{self._group}/joint_goal",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self._reset_client = self.create_client(
            Trigger, self._reset_service, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_service(
            Trigger, "~/recover", self._on_recover, callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(
            f"sim_recover ready: {self._group} stand goal via {self._joint_controller}, "
            f"then {self._reset_service}"
        )

    @staticmethod
    def _share_profile_dir(profile: str) -> Path:
        return Path(get_package_share_directory("integrated_bringup")) / "config" / profile

    # ── data lane ──────────────────────────────────────────────────────────
    def _on_active(self, msg: String) -> None:
        with self._lock:
            self._active = msg.data

    def _on_estop(self, msg: Bool) -> None:
        with self._lock:
            self._estop = bool(msg.data)

    def _on_joints(self, msg: JointState) -> None:
        with self._lock:
            self._positions = dict(zip(msg.name, msg.position, strict=False))

    def _on_sim_time(self, msg: Float64) -> None:
        with self._lock:
            self._sim_time = float(msg.data)

    def _now_sim(self) -> float | None:
        with self._lock:
            return self._sim_time

    def _measured(self) -> list[float] | None:
        with self._lock:
            if not all(n in self._positions for n in self._names):
                return None
            return [self._positions[n] for n in self._names]

    def _abort_reason(self) -> str | None:
        with self._lock:
            if self._estop is not False:
                return "the E-STOP latched during the recovery"
            if self._active != self._joint_controller:
                return f"the active controller changed to '{self._active}' during the recovery"
        return None

    # ── service ────────────────────────────────────────────────────────────
    def _on_recover(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        with self._lock:
            have = all(n in self._positions for n in self._names)
            why = refusal(
                active=self._active,
                estop=self._estop,
                joint_controller=self._joint_controller,
                busy=self._busy,
                have_joints=have,
                have_sim_time=self._sim_time is not None,
            )
            if why is None:
                self._busy = True
        if why is not None:
            res.success, res.message = False, why
            self.get_logger().warn(f"recover refused: {why}")
            return res
        try:
            res.success, res.message = self._recover()
        except Exception as exc:  # a failed service must still answer
            res.success, res.message = False, f"recover failed: {exc!r}"
        finally:
            with self._lock:
                self._busy = False
        (self.get_logger().info if res.success else self.get_logger().error)(res.message)
        return res

    def _recover(self) -> tuple[bool, str]:
        start = self._measured()
        if start is None:
            return False, "the joint state does not name every joint of the stand pose"
        wait_s = (
            trajectory_duration(max_abs_diff(start, self._pose), self._speed, self._max_vel)
            + self._margin_s
        )
        if not self._wait_for_subscriber():
            return False, f"nothing subscribes to {self._goal_pub.topic_name}"
        goal = RobotTarget()
        goal.goal_type = "joint"
        goal.joint_names = list(self._names)
        goal.joint_target = list(self._pose)
        self._goal_pub.publish(goal)
        self.get_logger().info(f"stand goal published; waiting {wait_s:.2f} sim s")

        t_end = self._now_sim() + wait_s
        wall_limit = time.monotonic() + max(_WALL_GUARD_MIN_S, _WALL_GUARD_FACTOR * wait_s)
        gate = threading.Event()  # never set: a cancellable sleep that costs no CPU
        while True:
            why = self._abort_reason()
            if why:
                return False, why
            sim_now = self._now_sim()
            if sim_now >= t_end:
                break
            if time.monotonic() > wall_limit:
                return False, (
                    f"sim time did not advance {wait_s:.2f} s within the wall guard "
                    f"(reached {sim_now - t_end + wait_s:.2f} s); /sim/reset not called"
                )
            gate.wait(_POLL_S)

        now = self._measured()
        residual = float("inf") if now is None else max_abs_diff(now, self._pose)
        if self._residual_tol > 0.0 and residual > self._residual_tol:
            return False, (
                f"not at the stand pose after {wait_s:.2f} s: max |q - goal| = "
                f"{residual:.3f} rad > {self._residual_tol} rad; /sim/reset not called"
            )
        ok, msg = self._call_reset()
        return ok, f"{msg} (residual {residual:.3f} rad)"

    def _wait_for_subscriber(self) -> bool:
        gate = threading.Event()
        for _ in range(int(_SUBSCRIBER_WAIT_S / _POLL_S)):
            if self._goal_pub.get_subscription_count() > 0:
                return True
            gate.wait(_POLL_S)
        return self._goal_pub.get_subscription_count() > 0

    def _call_reset(self) -> tuple[bool, str]:
        if not self._reset_client.wait_for_service(timeout_sec=_RESET_TIMEOUT_S):
            return False, f"{self._reset_service} is not available"
        done = threading.Event()
        future = self._reset_client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(_RESET_TIMEOUT_S):
            future.cancel()
            return False, f"{self._reset_service} did not answer in {_RESET_TIMEOUT_S} s"
        reply = future.result()
        if reply is None:
            return False, f"{self._reset_service} call failed: {future.exception()!r}"
        return bool(reply.success), f"{self._reset_service}: {reply.message}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimRecover()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
