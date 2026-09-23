"""Throw trials at the catching controller in sim, with the arm aligned first.

The procedure is the standard one for S6/S8 sim measurements until the
controller homes by itself (S7.2) — #537 decision ④, 2026-09-23. Before every
throw the arm is sent to ``planner.wait_pose`` and the throw waits until it is
there. Without that, each trial starts wherever the previous one ended (the
first from the activation pose, later ones as low as 8 cm above the floor):
2026-09-23 measured 4.6-6.3 rad of start-pose spread, and the tracking error,
saturation and catch error of those runs described the start pose as much as
the controller.

Per trial:

1. Disarm (``catching.enable`` false), wait for IDLE. A latched FAULT is reset
   first.
2. Switch to ``demo_joint_controller`` and send it ``wait_pose`` as a joint goal.
3. Wait until ``max|q - wait_pose|`` < ``tol_q`` and ``max|qd|`` < ``tol_qd``
   hold together for ``hold_s``.
4. Switch back to ``demo_catching_controller``, arm, wait for ARMED, and check
   the arm did not drift during the hand-over.
5. Launch the ball at a stated release state (``/sim/launch_ball_at`` — a stated
   launch does not consume the sim's RNG, so the series replays exactly), record
   ground truth and mode transitions, reset the ball.

This drives an already-running sim. It launches nothing: the sim, the ball
estimator and the switch into the catching controller are the caller's
(``integrated_bringup/README.md`` §Catching sim trials has the commands).

Robot constants come from the shipped profile, not from this file: the arm
device is the first device in the catching controller's ``topics``, its joint
names and state topic are that device's roster, and the wait pose is
``planner.wait_pose``.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import os
import random
import time
from typing import Any

import yaml

CATCHING = "demo_catching_controller"
JOINT = "demo_joint_controller"

MODE_NAMES = (
    "IDLE",
    "ARMED",
    "TRACKING",
    "APPROACH",
    "COMMITTED",
    "CLOSING",
    "DECEL",
    "HOLD",
    "RETREAT",
    "ABORT_SAFE",
    "FAULT",
)
OUTCOME_NAMES = ("NONE", "CAPTURED", "MISSED", "UNDETERMINED", "ABORTED")

# The S3.5b reference throw for ur5e_p1b (1.0 m out, 0.2 m up, 4.75 m/s at 60°).
REFERENCE_RELEASE_POS = (1.0, 0.0, 0.2)
REFERENCE_RELEASE_VEL = (-2.375, 0.0, 4.11362)


@dataclasses.dataclass(frozen=True)
class ArmProfile:
    """What the driver needs to know about the arm, read from a shipped profile."""

    device: str
    joint_names: tuple[str, ...]
    state_topic: str
    wait_pose: tuple[float, ...]

    @property
    def goal_topic(self) -> str:
        return f"/{JOINT}/{self.device}/joint_goal"


def _device_entries(node: Any, device: str):
    """Every ``devices.<device>`` mapping anywhere under ``node``."""
    if isinstance(node, dict):
        devices = node.get("devices")
        if isinstance(devices, dict) and isinstance(devices.get(device), dict):
            yield devices[device]
        for value in node.values():
            yield from _device_entries(value, device)


def _find_key(node: Any, key: str):
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for value in node.values():
            found = _find_key(value, key)
            if found is not None:
                return found
    return None


def load_arm_profile(config_dir: str) -> ArmProfile:
    """Resolve the arm from ``<config_dir>`` (one robot profile's config tree).

    Raises ``ValueError`` when something is missing or inconsistent: a driver
    that guessed would home the arm to the wrong pose and every trial would
    still look like a trial.
    """
    with open(os.path.join(config_dir, "controllers", f"{CATCHING}.yaml")) as f:
        ctrl = yaml.safe_load(f)[CATCHING]
    topics = ctrl.get("topics") or {}
    if not topics:
        raise ValueError(f"{CATCHING}.yaml has no topics — cannot tell which device is the arm")
    device = next(iter(topics))
    wait_pose = ctrl["catching"]["planner"].get("wait_pose")
    if not wait_pose:
        raise ValueError(f"{CATCHING}.yaml has no catching.planner.wait_pose")

    joint_names = None
    state_topic = None
    for name in ("_base.yaml", "sim.yaml"):
        path = os.path.join(config_dir, name)
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            doc = yaml.safe_load(f)
        for entry in _device_entries(doc, device):
            joint_names = joint_names or entry.get("joint_state_names")
            state_topic = state_topic or _find_key(entry, "state_topic")
    if not joint_names:
        raise ValueError(f"no devices.{device}.joint_state_names under {config_dir}")
    if not state_topic:
        raise ValueError(f"no state_topic for device {device} under {config_dir}")
    if len(wait_pose) != len(joint_names):
        raise ValueError(
            f"planner.wait_pose has {len(wait_pose)} values but device {device} "
            f"has {len(joint_names)} joints"
        )
    return ArmProfile(
        device=device,
        joint_names=tuple(joint_names),
        state_topic=state_topic,
        wait_pose=tuple(float(v) for v in wait_pose),
    )


def alignment_error(q, qd, target) -> tuple[float, float]:
    """``(max|q - target|, max|qd|)`` — the two numbers the alignment gate reads."""
    return (
        max(abs(a - b) for a, b in zip(q, target, strict=True)),
        max(abs(v) for v in qd),
    )


def trial_throws(n_ref: int, n_pert: int, seed: int, pos, vel):
    """The throw series: ``n_ref`` reference throws, then ``n_pert`` perturbed ones.

    Perturbation (unchanged since the first S6-C run, so series stay
    comparable): speed × U(0.9, 1.1), plus U(-0.3, 0.3) m/s lateral.
    """
    rng = random.Random(seed)
    throws = [{"kind": "reference", "pos": tuple(pos), "vel": tuple(vel)} for _ in range(n_ref)]
    for _ in range(n_pert):
        scale = rng.uniform(0.9, 1.1)
        lateral = rng.uniform(-0.3, 0.3)
        vx, vy, vz = vel
        throws.append(
            {
                "kind": "varied",
                "pos": tuple(pos),
                "vel": (vx * scale, vy * scale + lateral, vz * scale),
                "mag_scale": scale,
                "lateral_y": lateral,
            }
        )
    return throws


def _make_driver(profile: ArmProfile, args):
    # ROS imports stay here so the helpers above are importable without a
    # sourced ROS environment.
    import rclpy
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    from rtc_msgs.msg import CatchingState, RobotTarget
    from rtc_msgs.srv import LaunchBall, ResetFault, SwitchController

    class TrialDriver(Node):
        def __init__(self) -> None:
            super().__init__("catching_sim_trials")
            # The sim publishes state best-effort: a default (reliable)
            # subscriber is incompatible and silently receives nothing.
            best_effort = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST
            )
            self.launch_cli = self.create_client(LaunchBall, "/sim/launch_ball_at")
            self.reset_ball_cli = self.create_client(Trigger, "/sim/reset_ball")
            self.reset_fault_cli = self.create_client(ResetFault, "/rtc_cm/reset_fault")
            self.switch_cli = self.create_client(SwitchController, "/rtc_cm/switch_controller")
            self.param_cli = self.create_client(
                SetParameters, f"/{CATCHING}/{CATCHING}/set_parameters"
            )
            self.goal_pub = self.create_publisher(RobotTarget, profile.goal_topic, 1)
            self.create_subscription(
                CatchingState, f"/{CATCHING}/catching_state", self._on_state, 10
            )
            self.create_subscription(
                Odometry, "/sim/ball/ground_truth", self._on_truth, best_effort
            )
            self.create_subscription(JointState, profile.state_topic, self._on_joints, best_effort)
            self.mode = None
            self.outcome = None
            self.mode_log = []
            self.truth_rows = []
            self.offsets = []
            self.tick_range = [None, None]
            self.recording = False
            self.q = None
            self.qd = None

        # ── subscriptions ────────────────────────────────────────────────
        def _on_state(self, msg) -> None:
            now = time.time()
            if msg.mode != self.mode:
                name = MODE_NAMES[msg.mode] if msg.mode < len(MODE_NAMES) else str(msg.mode)
                self.mode_log.append((now, name, msg.reason))
                self.mode = msg.mode
            self.outcome = msg.outcome
            if self.recording:
                if self.tick_range[0] is None:
                    self.tick_range[0] = msg.tick
                self.tick_range[1] = msg.tick
                self.offsets.append(now - msg.t_relative_s)

        def _on_truth(self, msg) -> None:
            if self.recording:
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.truth_rows.append((time.time(), stamp, p.x, p.y, p.z, v.x, v.y, v.z))

        def _on_joints(self, msg) -> None:
            index = {n: i for i, n in enumerate(msg.name)}
            if all(n in index for n in profile.joint_names):
                self.q = [msg.position[index[n]] for n in profile.joint_names]
                self.qd = (
                    [msg.velocity[index[n]] for n in profile.joint_names]
                    if msg.velocity
                    else [0.0] * len(profile.joint_names)
                )

        # ── helpers ──────────────────────────────────────────────────────
        def mode_name(self) -> str:
            return MODE_NAMES[self.mode] if self.mode is not None else "UNKNOWN"

        def call(self, client, request, timeout_s: float = 5.0):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            return future.result()

        def spin_for(self, seconds: float) -> None:
            end = time.time() + seconds
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)

        def wait_for_mode(self, name: str, timeout_s: float) -> bool:
            end = time.time() + timeout_s
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.mode_name() == name:
                    return True
            return False

        def set_enable(self, value: bool) -> bool:
            req = SetParameters.Request()
            req.parameters = [
                Parameter(
                    name="catching.enable",
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value),
                )
            ]
            res = self.call(self.param_cli, req)
            return res is not None and all(r.successful for r in res.results)

        def switch(self, activate: str, deactivate: str) -> bool:
            req = SwitchController.Request()
            req.activate_controllers = [activate]
            req.deactivate_controllers = [deactivate]
            req.strictness = 1
            req.timeout.sec = 3
            res = self.call(self.switch_cli, req, timeout_s=10.0)
            return res is not None and res.ok

        def wait_for_services(self, timeout_s: float = 10.0) -> None:
            for client, name in (
                (self.launch_cli, "/sim/launch_ball_at"),
                (self.reset_ball_cli, "/sim/reset_ball"),
                (self.switch_cli, "/rtc_cm/switch_controller"),
                (self.param_cli, f"/{CATCHING}/{CATCHING}/set_parameters"),
            ):
                if not client.wait_for_service(timeout_sec=timeout_s):
                    raise RuntimeError(f"service {name} unavailable")

        # ── the procedure ────────────────────────────────────────────────
        def home(self) -> dict:
            """Steps 1-4 of the module docstring. Raises when a step fails."""
            t0 = time.time()
            self.spin_for(0.1)
            if self.mode_name() == "FAULT":
                req = ResetFault.Request()
                req.controller_name = CATCHING
                res = self.call(self.reset_fault_cli, req)
                self.get_logger().warn(f"reset_fault before homing -> {res}")
            if not self.set_enable(False):
                raise RuntimeError("disarm failed")
            if not self.wait_for_mode("IDLE", 10.0):
                raise RuntimeError(f"not IDLE after disarm (mode {self.mode_name()})")
            if not self.switch(JOINT, CATCHING):
                raise RuntimeError(f"switch to {JOINT} failed")
            end = time.time() + 5.0
            while self.goal_pub.get_subscription_count() == 0 and time.time() < end:
                self.spin_for(0.05)
            goal = RobotTarget()
            goal.goal_type = "joint"
            goal.joint_names = list(profile.joint_names)
            goal.joint_target = list(profile.wait_pose)
            self.goal_pub.publish(goal)
            start_q = list(self.q) if self.q else None

            in_tol_since = None
            end = time.time() + args.home_timeout
            while True:
                if time.time() > end:
                    err = alignment_error(self.q, self.qd, profile.wait_pose) if self.q else None
                    raise RuntimeError(f"homing timeout, error {err}")
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.q is None:
                    continue
                eq, eqd = alignment_error(self.q, self.qd, profile.wait_pose)
                if eq < args.tol_q and eqd < args.tol_qd:
                    in_tol_since = in_tol_since or time.time()
                    if time.time() - in_tol_since >= args.hold_s:
                        break
                else:
                    in_tol_since = None
            home_s = time.time() - t0

            if not self.switch(CATCHING, JOINT):
                raise RuntimeError(f"switch back to {CATCHING} failed")
            if not self.set_enable(True):
                raise RuntimeError("arm failed")
            if not self.wait_for_mode("ARMED", 8.0):
                raise RuntimeError(f"not ARMED after re-arm (mode {self.mode_name()})")
            self.spin_for(0.5)
            # The catching controller latches the pose it finds at activation;
            # a hand-over that let the arm sag would show up here.
            eq, eqd = alignment_error(self.q, self.qd, profile.wait_pose)
            if eq > 2 * args.tol_q or eqd > 2 * args.tol_qd:
                raise RuntimeError(f"drifted after re-arm: {eq:.4f} rad, {eqd:.4f} rad/s")
            return {
                "home_start_q": start_q,
                "home_s": round(home_s, 2),
                "home_total_s": round(time.time() - t0, 2),
                "q_at_throw": [round(v, 5) for v in self.q],
                "err_q_at_throw": round(eq, 5),
                "err_qd_at_throw": round(eqd, 5),
            }

        def throw(self, idx: int, throw: dict) -> dict:
            """Step 5. Returns the trial record written to trial_results.json."""
            self.mode_log, self.truth_rows, self.offsets = [], [], []
            self.tick_range = [None, None]
            self.recording = True
            req = LaunchBall.Request()
            req.position.x, req.position.y, req.position.z = throw["pos"]
            req.velocity.x, req.velocity.y, req.velocity.z = throw["vel"]
            res = self.call(self.launch_cli, req)
            launch_wall_time = time.time()
            record = {"idx": idx, **throw, "accepted": bool(res and res.accepted)}
            record["message"] = res.message if res else "no response"
            if not record["accepted"]:
                self.recording = False
                self.get_logger().error(f"trial {idx}: launch refused: {record['message']}")
                return record
            self.spin_for(args.record_s)
            self.recording = False

            truth_csv = os.path.join(args.out_dir, f"truth_trial_{idx:02d}.csv")
            with open(truth_csv, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["wall_recv_s", "stamp_s", "x", "y", "z", "vx", "vy", "vz"])
                writer.writerows(self.truth_rows)
            offsets = sorted(self.offsets)
            self.call(self.reset_ball_cli, Trigger.Request())
            record.update(
                {
                    "mode_log": self.mode_log,
                    "n_truth_rows": len(self.truth_rows),
                    "truth_csv": truth_csv,
                    "final_outcome": (
                        OUTCOME_NAMES[self.outcome] if self.outcome is not None else None
                    ),
                    "final_mode": self.mode_name(),
                    "launch_wall_time": launch_wall_time,
                    "tick_min": self.tick_range[0],
                    "tick_max": self.tick_range[1],
                    # median of (wall − t_relative_s): joins this record to the
                    # controller CSV's time axis
                    "wall_t_relative_offset": offsets[len(offsets) // 2] if offsets else None,
                }
            )
            self.get_logger().info(
                f"trial {idx}: modes={[m[1] for m in self.mode_log]} "
                f"outcome={record['final_outcome']} final_mode={record['final_mode']}"
            )
            return record

    return rclpy, TrialDriver


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out_dir", help="where truth_trial_NN.csv and trial_results.json go")
    parser.add_argument("--profile", default="ur5e_p1b", help="robot profile (config/<profile>)")
    parser.add_argument(
        "--config-dir", help="override the profile's config directory (default: installed share)"
    )
    parser.add_argument("--n-ref", type=int, default=15, help="reference throws")
    parser.add_argument("--n-pert", type=int, default=10, help="perturbed throws after them")
    parser.add_argument("--seed", type=int, default=42, help="perturbation RNG seed")
    parser.add_argument("--release-pos", type=float, nargs=3, default=REFERENCE_RELEASE_POS)
    parser.add_argument("--release-vel", type=float, nargs=3, default=REFERENCE_RELEASE_VEL)
    parser.add_argument("--tol-q", type=float, default=0.01, help="alignment gate, rad")
    parser.add_argument("--tol-qd", type=float, default=0.01, help="alignment gate, rad/s")
    parser.add_argument("--hold-s", type=float, default=0.5, help="time inside the gate, s")
    parser.add_argument("--home-timeout", type=float, default=30.0, help="s")
    parser.add_argument("--record-s", type=float, default=3.0, help="recording window per throw")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    config_dir = args.config_dir
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("integrated_bringup")
        config_dir = os.path.join(share, "config", args.profile)
    profile = load_arm_profile(config_dir)
    os.makedirs(args.out_dir, exist_ok=True)
    throws = trial_throws(args.n_ref, args.n_pert, args.seed, args.release_pos, args.release_vel)

    rclpy, TrialDriver = _make_driver(profile, args)
    rclpy.init()
    node = TrialDriver()
    results = []
    try:
        node.wait_for_services()
        for idx, throw in enumerate(throws):
            alignment = node.home()
            node.get_logger().info(f"trial {idx}: aligned {alignment}")
            record = node.throw(idx, throw)
            record.update(alignment)
            results.append(record)
            node.spin_for(0.3)
    finally:
        with open(os.path.join(args.out_dir, "trial_results.json"), "w") as f:
            json.dump(results, f, indent=2, default=str)
        node.destroy_node()
        rclpy.shutdown()
    return 0
