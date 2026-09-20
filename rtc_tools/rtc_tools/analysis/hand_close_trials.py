"""Drive N hand closure trials against a running bring-up (dynamic_catching S4.2).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 S4a, law ``L6_hand.md`` §4.2.

Publishes alternating preshape/closed step targets to the catching controller's
hand group and leaves the recording to that controller's own ``DeviceStateLog``
CSV; ``analyze_hand_close`` reads it afterwards. It also writes a JSON sidecar
with the profile it used, which is what the analyser consumes.

**The profile is read from the CONTROLLER, not from the shipped YAML.** The
catching controller mirrors what it loaded into read-only parameters, so a run
analysed against those numbers is analysed against the numbers that produced it.
Reading the YAML here instead would go green on a run whose controller loaded a
different file (a stale install, another config_variant) — exactly the failure
the sidecar exists to prevent.

**Trials are separated by a dwell, not by a settle heuristic.** Each trial
commands the closed pose, waits ``--close-dwell``, commands the preshape pose,
waits ``--open-dwell``. The analyser segments on the command lane, so the dwell
only has to be long enough for the hand to finish moving — it does not have to
be measured. Pick it from the expected T_close with a wide margin; too short
silently truncates the tail of the distribution, which is the part S4.4 needs.

This measures nothing by itself and decides nothing. 200 trials per hand is the
sample size the plan asks for (a 99th percentile needs >= 100 successes).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python

from rtc_msgs.msg import RobotTarget

DEFAULT_CONFIG_KEY = "demo_catching_controller"


class HandStepRunner(Node):
    def __init__(self, config_key: str, group: str) -> None:
        super().__init__("hand_close_trials")
        self._config_key = config_key
        # The controller's LifecycleNode lives at /<config_key>/<config_key>
        # (rt_controller_node_params.cpp), and its target topic resolves under
        # that namespace.
        self._node_path = f"/{config_key}/{config_key}"
        self.publisher = self.create_publisher(RobotTarget, f"/{config_key}/{group}/joint_goal", 1)

    def read_profile(self, joint_names: list[str], timeout_s: float) -> dict:
        """Read the controller's read-only profile parameters."""
        from rcl_interfaces.srv import GetParameters

        client = self.create_client(GetParameters, f"{self._node_path}/get_parameters")
        deadline = time.monotonic() + timeout_s
        while not client.wait_for_service(timeout_sec=0.5):
            if time.monotonic() > deadline:
                raise SystemExit(
                    f"{self._node_path}/get_parameters never appeared. Is the catching "
                    "controller configured? (It refuses to configure outside sim — see "
                    "its E-8 guard.)"
                )
        names = [
            "hand.q_open",
            "hand.q_pre",
            "hand.q_close",
            "hand.caging_mask",
            "hand.eta_close",
            "hand.rho_eps",
            "hand.T_close_e2e",
            "diagnostic.hand_step",
        ]
        request = GetParameters.Request(names=names)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None or len(response.values) != len(names):
            raise SystemExit("the controller did not answer get_parameters")
        # parameter_value_to_python (rclpy jazzy) turns a rcl_interfaces
        # ParameterValue into the plain Python value for its declared type.
        values = {
            n: parameter_value_to_python(v) for n, v in zip(names, response.values, strict=True)
        }
        profile = {
            "joint_names": joint_names,
            "q_open": list(values["hand.q_open"]),
            "q_pre": list(values["hand.q_pre"]),
            "q_close": list(values["hand.q_close"]),
            "caging_mask": [bool(v) for v in values["hand.caging_mask"]],
            "eta_close": float(values["hand.eta_close"]),
            "rho_eps": float(values["hand.rho_eps"]),
            "T_close_e2e_at_run": float(values["hand.T_close_e2e"]),
            "hand_step_enabled": bool(values["diagnostic.hand_step"]),
        }
        if not profile["hand_step_enabled"]:
            raise SystemExit(
                "the controller reports diagnostic.hand_step = false, so it will refuse "
                "every step this runner sends. Enable it in the controller YAML."
            )
        return profile

    def send(self, joint_names: list[str], positions: list[float]) -> None:
        msg = RobotTarget()
        msg.goal_type = "joint"
        # Named goal: the base ingress checks the names against the device's own
        # joint list and refuses a partial or permuted goal, which a positional
        # one cannot be checked for.
        msg.joint_names = joint_names
        msg.joint_target = [float(v) for v in positions]
        self.publisher.publish(msg)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--joint-names",
        required=True,
        help="comma-separated hand joint_state_names, in device order "
        "(the same list the CSV header is built from)",
    )
    parser.add_argument("--group", required=True, help="hand device group name, e.g. p1b / leap")
    parser.add_argument("--config-key", default=DEFAULT_CONFIG_KEY)
    parser.add_argument(
        "--trials",
        type=int,
        default=200,
        help="closure trials (plan: 200 per hand; a p99 needs >= 100 successes)",
    )
    parser.add_argument(
        "--close-dwell-s",
        type=float,
        default=1.0,
        help="wall seconds held at the closed pose — must exceed the longest expected "
        "T_close or the tail is silently truncated",
    )
    parser.add_argument("--open-dwell-s", type=float, default=1.0)
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="JSON sidecar to write (feed it to analyze_hand_close --profile)",
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)

    joint_names = [n.strip() for n in args.joint_names.split(",") if n.strip()]
    if not joint_names:
        print("--joint-names is empty", file=sys.stderr)
        return 2

    rclpy.init()
    try:
        node = HandStepRunner(args.config_key, args.group)
        profile = node.read_profile(joint_names, args.timeout_s)
        if len(profile["q_pre"]) != len(joint_names):
            raise SystemExit(
                f"the controller's profile has {len(profile['q_pre'])} joints but "
                f"--joint-names lists {len(joint_names)}. The CSV columns are named from "
                "the device's joint_state_names; a mismatch means the analyser would "
                "read the wrong columns."
            )

        profile["trials"] = args.trials
        profile["group"] = args.group
        profile["config_key"] = args.config_key
        profile["close_dwell_s"] = args.close_dwell_s
        profile["open_dwell_s"] = args.open_dwell_s
        args.out.write_text(json.dumps(profile, indent=2))
        print(f"profile -> {args.out}")

        # Park at the preshape pose first: a trial whose first command is the
        # closed pose would time a closure that started from wherever the hand
        # happened to be.
        node.send(joint_names, profile["q_pre"])
        time.sleep(args.open_dwell_s)

        for trial in range(1, args.trials + 1):
            node.send(joint_names, profile["q_close"])
            time.sleep(args.close_dwell_s)
            node.send(joint_names, profile["q_pre"])
            time.sleep(args.open_dwell_s)
            if trial % 20 == 0:
                print(f"  {trial}/{args.trials}")
        print(
            f"done: {args.trials} trials. Now run analyze_hand_close on the hand's "
            f"<group>_state.csv with --profile {args.out}, and check the controller's "
            "log for CSV drop warnings first — a dropped row invalidates the tick axis."
        )
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
