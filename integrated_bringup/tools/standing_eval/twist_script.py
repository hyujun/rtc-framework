#!/usr/bin/env python3
"""Publish a velocity command script as geometry_msgs/Twist, then rest.

The script is rtc_tools.analysis.walk_metrics.WALK_SCRIPT (sim seconds). It
waits for a subscriber (the controller's), then publishes at ``--rate`` Hz on
the wall clock, the command of sim time ``wall elapsed × --rtf``, with depth 1.
After the script it publishes the zero command ``--rest-count`` times and
exits 0; SIGINT (the caller's stop) does the same at once.

Its stdout is JSON lines: ``{"event": "start", "wall": …}`` when the script
starts (after the subscriber matched), one per segment boundary, and
``{"event": "end", …}`` with how many messages went out. A subscriber that
never comes is exit 3, a stop before it came exit 4 — neither has a start line.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time

from rtc_tools.analysis.walk_metrics import WALK_SCRIPT, command_at, script_length, segment_at


def emit(**fields) -> None:
    print(json.dumps(fields), flush=True)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--topic", default="/locomotion_controller/g1/cmd_vel")
    p.add_argument("--rtf", type=float, default=1.0, help="sim seconds per wall second")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate [Hz, wall]")
    p.add_argument("--wait-subscriber", type=float, default=60.0, help="[s, wall]")
    p.add_argument("--rest-count", type=int, default=5)
    args = p.parse_args(argv)
    if not args.rtf > 0.0 or not args.rate > 0.0:
        p.error("--rtf and --rate must be positive")

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.signals import SignalHandlerOptions

    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("walk_twist_script")
    pub = node.create_publisher(Twist, args.topic, 1)
    sent = 0
    try:
        end = time.monotonic() + args.wait_subscriber
        while pub.get_subscription_count() < 1 and not stop["now"]:
            if time.monotonic() > end:
                emit(event="no_subscriber", topic=args.topic)
                return 3
            time.sleep(0.05)
        if stop["now"]:
            emit(event="interrupted_before_start", topic=args.topic)
            return 4
        total = script_length(WALK_SCRIPT)
        t0 = time.monotonic()
        emit(event="start", wall=time.time(), topic=args.topic, rtf=args.rtf)
        period = 1.0 / args.rate
        last_seg = None
        while not stop["now"]:
            t_sim = (time.monotonic() - t0) * args.rtf
            if t_sim >= total:
                break
            seg = segment_at(WALK_SCRIPT, t_sim)
            if seg != last_seg:
                emit(event="segment", name=seg, wall=time.time(), t_sim=t_sim)
                last_seg = seg
            vx, vy, wz = command_at(WALK_SCRIPT, t_sim)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
            pub.publish(msg)
            sent += 1
            time.sleep(max(0.0, t0 + sent * period - time.monotonic()))
    finally:
        for _ in range(args.rest_count):
            pub.publish(Twist())
            sent += 1
            time.sleep(1.0 / args.rate)
        emit(event="end", wall=time.time(), sent=sent, interrupted=stop["now"])
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
