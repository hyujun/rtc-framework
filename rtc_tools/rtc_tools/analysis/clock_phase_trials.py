"""Drive N launch trials against a running simulator (D-3 / S3.1a).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §5. The measurement wants
"per configuration (2 robots), >= 200 launches" of clock phase error. This issues
those launches against an already-running ``mujoco_simulator`` node and leaves
the recording to the node's clock lane; ``analyze_clock_phase`` reads the CSV
afterwards.

**Launches are STATED, not sampled** (``/sim/launch_ball_at``). Two reasons, and
the second is the one that matters: a stated launch does not consume the seeded
RNG, so a trial series can be replayed exactly; and every trial starts the ball
from the same release state, so a difference between trials is a difference in
the machine rather than in the throw. A sampled series would confound the two
and the distribution would carry throw variance that D-3 is not about.

**The ball is reset between trials.** The lane closes a trial's window when the
ball stops being active, and delta accumulates from the launch instant, so a
trial that ran until the next launch would report the gap between throws as
clock error.

This does not decide anything. It produces the CSV; the verdict stays
NOT_EVALUATED until ``eps_clk_alloc`` is chosen (plan §5).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rtc_msgs.srv import LaunchBall


class TrialRunner(Node):
    def __init__(self) -> None:
        super().__init__("clock_phase_trials")
        self.launch_at = self.create_client(LaunchBall, "/sim/launch_ball_at")
        self.reset_ball = self.create_client(Trigger, "/sim/reset_ball")

    def wait_for_services(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        for client, name in (
            (self.launch_at, "/sim/launch_ball_at"),
            (self.reset_ball, "/sim/reset_ball"),
        ):
            while not client.wait_for_service(timeout_sec=0.5):
                if time.monotonic() > deadline:
                    self.get_logger().error(f"{name} never appeared")
                    return False
        return True

    def call(self, client, request, timeout_s: float = 5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return future.result()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--trials",
        type=int,
        default=200,
        help="launches to issue (plan §5 proposes >= 200 per configuration)",
    )
    parser.add_argument(
        "--flight-s",
        type=float,
        default=1.2,
        help="wall seconds to let each throw fly before resetting",
    )
    parser.add_argument(
        "--settle-s", type=float, default=0.2, help="wall seconds parked between trials"
    )
    parser.add_argument(
        "--position",
        type=float,
        nargs=3,
        default=[4.81, 0.04, 1.75],
        help="release position [m], world",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        nargs=3,
        default=[-4.0, 0.0, 3.5805],
        help="release velocity [m/s], world",
    )
    parser.add_argument(
        "--spin",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        help="release angular velocity [rad/s], world",
    )
    parser.add_argument("--service-timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)

    rclpy.init()
    runner = TrialRunner()
    try:
        if not runner.wait_for_services(args.service_timeout_s):
            return 2

        request = LaunchBall.Request()
        request.position.x, request.position.y, request.position.z = args.position
        request.velocity.x, request.velocity.y, request.velocity.z = args.velocity
        (request.angular_velocity.x, request.angular_velocity.y, request.angular_velocity.z) = (
            args.spin
        )

        refused = 0
        for trial in range(1, args.trials + 1):
            response = runner.call(runner.launch_at, request)
            if response is None or not response.accepted:
                # Counted, not ignored. A refused launch leaves the PREVIOUS
                # ball's state in place, so a run that quietly skipped some
                # would report fewer trials than it claims while every CSV row
                # still looks well formed.
                refused += 1
                runner.get_logger().warn(
                    f"trial {trial} refused: {response.message if response else 'no response'}"
                )
            time.sleep(args.flight_s)
            runner.call(runner.reset_ball, Trigger.Request())
            time.sleep(args.settle_s)
            if trial % 25 == 0:
                runner.get_logger().info(f"{trial}/{args.trials} trials issued")

        runner.get_logger().info(
            f"done: {args.trials - refused}/{args.trials} launched, {refused} refused"
        )
        if refused:
            print(
                f"{refused} launch(es) were refused — the CSV holds fewer trials than "
                "requested; do not quote a trial count from the command line.",
                file=sys.stderr,
            )
            return 1
        return 0
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
