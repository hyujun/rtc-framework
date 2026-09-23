"""Throw trials at the catching controller in sim, one full S7 cycle each.

From S7.2 the controller homes itself: armed, it moves the arm to
``planner.wait_pose`` in joint space and waits there with the hand at q_pre
(IDLE → ARMED), and after every trial it returns there (RETREAT → ARMED). The
external alignment this driver used to do (#537 decision ④, "until S7.2" —
switch to ``demo_joint_controller``, send the wait pose, switch back) is gone:
a trial that starts elsewhere is now the controller's bug to show, not the
driver's to hide.

Per trial:

1. A latched FAULT is reset; the controller is armed (``catching.enable``) if
   it is not; wait for ARMED — the controller's own homing — and record how far
   the measured arm is from the wait pose at that moment.
2. Launch the ball at a stated release state (``/sim/launch_ball_at`` — a stated
   launch does not consume the sim's RNG, so the series replays exactly) and
   record ground truth and every mode transition.
3. The trial ends when the controller is back in ARMED after a RETREAT (the
   cycle closed), or in IDLE / FAULT, or after ``record_s``. The attempt's
   verdict is the ``outcome`` published on the RETREAT entry (L7 §4.7).
4. Reset the ball.

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


def _cycle_closed(modes: list[str]) -> bool:
    """Whether a trial's mode sequence re-armed after its RETREAT (one S7 cycle)."""
    if "RETREAT" not in modes:
        return False
    return "ARMED" in modes[modes.index("RETREAT") + 1 :]


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

    from rtc_msgs.msg import CatchingState
    from rtc_msgs.srv import LaunchBall, ResetFault

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
            self.param_cli = self.create_client(
                SetParameters, f"/{CATCHING}/{CATCHING}/set_parameters"
            )
            self.create_subscription(
                CatchingState, f"/{CATCHING}/catching_state", self._on_state, 10
            )
            self.create_subscription(
                Odometry, "/sim/ball/ground_truth", self._on_truth, best_effort
            )
            self.create_subscription(JointState, profile.state_topic, self._on_joints, best_effort)
            self.mode = None
            self.outcome = None
            self.retreat_outcome = None
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
                if name == "RETREAT" and self.recording:
                    # The verdict the controller published on the RETREAT entry
                    # (HOLD judges on that edge; an abort sets it there).
                    self.retreat_outcome = msg.outcome
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

        def wait_for_services(self, timeout_s: float = 10.0) -> None:
            for client, name in (
                (self.launch_cli, "/sim/launch_ball_at"),
                (self.reset_ball_cli, "/sim/reset_ball"),
                (self.param_cli, f"/{CATCHING}/{CATCHING}/set_parameters"),
            ):
                if not client.wait_for_service(timeout_sec=timeout_s):
                    raise RuntimeError(f"service {name} unavailable")

        # ── the procedure ────────────────────────────────────────────────
        def home(self) -> dict:
            """Step 1 of the module docstring. Raises when it fails."""
            t0 = time.time()
            self.spin_for(0.1)
            if self.mode_name() == "FAULT":
                req = ResetFault.Request()
                req.controller_name = CATCHING
                res = self.call(self.reset_fault_cli, req)
                self.get_logger().warn(f"reset_fault before the trial -> {res}")
                self.spin_for(0.2)
            start_q = list(self.q) if self.q else None
            if self.mode_name() != "ARMED" and not self.set_enable(True):
                raise RuntimeError("arm failed")
            # The controller homes itself (S7.2); ARMED means it says it is at
            # the wait pose with the hand at q_pre.
            if not self.wait_for_mode("ARMED", args.home_timeout):
                err = alignment_error(self.q, self.qd, profile.wait_pose) if self.q else None
                raise RuntimeError(f"not ARMED (mode {self.mode_name()}), error {err}")
            self.spin_for(0.2)
            eq, eqd = alignment_error(self.q, self.qd, profile.wait_pose)
            if eq > args.tol_q or eqd > args.tol_qd:
                # Recorded AND refused: the controller's own arrival test is
                # `supervisor.ready.pose_tol`, and an ARMED arm outside this gate
                # is a finding about the homing, not a start pose to accept.
                raise RuntimeError(f"ARMED away from the wait pose: {eq:.4f} rad, {eqd:.4f} rad/s")
            return {
                "home_start_q": start_q,
                "home_total_s": round(time.time() - t0, 2),
                "q_at_throw": [round(v, 5) for v in self.q],
                "err_q_at_throw": round(eq, 5),
                "err_qd_at_throw": round(eqd, 5),
            }

        def throw(self, idx: int, throw: dict) -> dict:
            """Step 5. Returns the trial record written to trial_results.json."""
            self.mode_log, self.truth_rows, self.offsets = [], [], []
            self.retreat_outcome = None
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
            # Until the cycle closes (RETREAT → ARMED), the controller gives up
            # (IDLE, FAULT), or the window runs out.
            end = time.time() + args.record_s
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
                names = [m[1] for m in self.mode_log]
                if "RETREAT" in names and names[-1] in ("ARMED", "IDLE", "FAULT"):
                    break
                if names and names[-1] == "FAULT":
                    break
            self.spin_for(0.1)
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
                    # The attempt's verdict (L7 §4.7): published on RETREAT entry.
                    "outcome": (
                        OUTCOME_NAMES[self.retreat_outcome]
                        if self.retreat_outcome is not None
                        else None
                    ),
                    # RETREAT followed by ARMED — the controller re-armed. What
                    # comes after (a new ball seen, TRACKING) does not reopen it.
                    "cycle_closed": _cycle_closed([m[1] for m in self.mode_log]),
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
                f"outcome={record['outcome']} final_mode={record['final_mode']}"
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
    parser.add_argument(
        "--tol-q", type=float, default=0.03, help="alignment check at ARMED, rad (pose_tol 0.02)"
    )
    parser.add_argument(
        "--tol-qd", type=float, default=0.05, help="alignment check at ARMED, rad/s"
    )
    parser.add_argument("--home-timeout", type=float, default=30.0, help="s to reach ARMED")
    parser.add_argument(
        "--record-s", type=float, default=12.0, help="longest window per throw (cycle end first)"
    )
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
        verdicts: dict[str, int] = {}
        for r in results:
            key = str(r.get("outcome"))
            verdicts[key] = verdicts.get(key, 0) + 1
        node.get_logger().info(
            f"{len(results)} trials, cycles closed "
            f"{sum(1 for r in results if r.get('cycle_closed'))}, verdicts {verdicts}"
        )
    finally:
        with open(os.path.join(args.out_dir, "trial_results.json"), "w") as f:
            json.dump(results, f, indent=2, default=str)
        node.destroy_node()
        rclpy.shutdown()
    return 0
