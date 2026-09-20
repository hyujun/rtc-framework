"""Relay the sim camera lane with drop and delay injection (S3.4).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 S3.4 — "지연·드롭
주입" and the ghost-track question (L1 §4.5, TBD-VIS-07). The estimator is
pointed at this relay's output instead of ``/sim/ball/camera_position`` and the
relay decides, per message, whether and when it goes through. Stamps are never
touched: a delayed message still says when it was captured, which is what a
transport delay looks like to the consumer.

Three knobs, all off by default (a relay with nothing set forwards everything):

* ``--drop-prob p``       — drop each message with probability p (seeded).
* ``--drop-after-s T``    — drop everything from T seconds after a flight
  starts until the flight ends. A flight starts at the first message after a
  receive gap longer than ``--flight-gap-s``. This is the ghost-track probe:
  the ball keeps flying, the estimator stops hearing about it.
* ``--delay-s d``         — hold each message d seconds before republishing.

The relay counts what it received, forwarded and dropped by which rule, and
prints them at exit. A relay that lost messages on its own input would show as
"drops" that no rule explains — so its input subscription is a plain
KEEP_LAST(1) like the consumer's, and the received count is compared with the
probe's camera count by whoever reads both.
"""

from __future__ import annotations

import argparse
import collections
import random
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEFAULT_IN_TOPIC = "/sim/ball/camera_position"
DEFAULT_OUT_TOPIC = "/sim/ball/camera_position_relay"


class CameraRelay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("camera_relay")
        self.args = args
        self.rng = random.Random(args.seed)
        self.received = 0
        self.forwarded = 0
        self.dropped_prob = 0
        self.dropped_after = 0
        self._last_recv: float | None = None
        self._flight_start: float | None = None
        self._held: collections.deque[tuple[float, PointStamped]] = collections.deque()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(PointStamped, args.out_topic, qos)
        self.create_subscription(PointStamped, args.in_topic, self._on_msg, qos)
        if args.delay_s > 0.0:
            # 1 kHz release timer: a delay quantised to 1 ms is fine for the
            # 10–50 ms delays S3.4 injects, and the quantisation is reported.
            self.create_timer(0.001, self._release)

    def _on_msg(self, msg: PointStamped) -> None:
        now = time.monotonic()
        self.received += 1
        if self._last_recv is None or (now - self._last_recv) > self.args.flight_gap_s:
            self._flight_start = now
        self._last_recv = now

        if (
            self.args.drop_after_s > 0.0
            and self._flight_start is not None
            and (now - self._flight_start) >= self.args.drop_after_s
        ):
            self.dropped_after += 1
            return
        if self.args.drop_prob > 0.0 and self.rng.random() < self.args.drop_prob:
            self.dropped_prob += 1
            return

        if self.args.delay_s > 0.0:
            self._held.append((now + self.args.delay_s, msg))
        else:
            self._pub.publish(msg)
            self.forwarded += 1

    def _release(self) -> None:
        now = time.monotonic()
        while self._held and self._held[0][0] <= now:
            _, msg = self._held.popleft()
            self._pub.publish(msg)
            self.forwarded += 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--in-topic", default=DEFAULT_IN_TOPIC)
    parser.add_argument("--out-topic", default=DEFAULT_OUT_TOPIC)
    parser.add_argument("--drop-prob", type=float, default=0.0)
    parser.add_argument("--drop-after-s", type=float, default=0.0)
    parser.add_argument("--delay-s", type=float, default=0.0)
    parser.add_argument("--flight-gap-s", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args(argv)
    if not 0.0 <= args.drop_prob <= 1.0:
        parser.error("--drop-prob must be in [0, 1]")
    if args.in_topic == args.out_topic:
        parser.error("--in-topic and --out-topic must differ (a relay feeding itself)")

    rclpy.init()
    relay = CameraRelay(args)
    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(relay, timeout_sec=0.05)
    finally:
        print(
            f"camera_relay: received {relay.received}, forwarded {relay.forwarded}, "
            f"dropped by prob {relay.dropped_prob}, dropped after flight start "
            f"{relay.dropped_after}, still held {len(relay._held)}",
            file=sys.stderr,
        )
        relay.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
