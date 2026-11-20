#!/usr/bin/env python3
"""Publish a velocity command script as geometry_msgs/Twist, then rest.

The script is one of rtc_tools.analysis.walk_metrics.COMMAND_SCRIPTS
(``--script``, default ``walk`` = WALK_SCRIPT; sim seconds). It waits for a
subscriber (the controller's), then publishes at ``--rate`` Hz on
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

from rtc_tools.analysis.walk_metrics import (
    COMMAND_SCRIPTS,
    command_at,
    script_length,
    segment_at,
)


def emit(**fields) -> None:
    print(json.dumps(fields), flush=True)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--topic", default="/locomotion_controller/g1/cmd_vel")
    p.add_argument("--rtf", type=float, default=1.0, help="sim seconds per wall second")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate [Hz, wall]")
    p.add_argument("--wait-subscriber", type=float, default=60.0, help="[s, wall]")
    p.add_argument("--rest-count", type=int, default=5)
    p.add_argument("--script", choices=sorted(COMMAND_SCRIPTS), default="walk")
    p.add_argument(
        "--push",
        type=float,
        nargs=5,
        metavar=("T", "DUR", "FX", "FY", "FZ"),
        help="one push: at T sim seconds into the script, for DUR sim seconds, a world-axis "
        "force [N] at the origin of --push-body (the simulator's /sim/set_external_wrench)",
    )
    p.add_argument("--push-body", help="the simulated body the push acts on")
    args = p.parse_args(argv)
    if args.push is not None and not args.push_body:
        p.error("--push needs --push-body")
    script = COMMAND_SCRIPTS[args.script]
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
    # The push: 0 = not yet, 1 = on, 2 = done. Its two calls are sent from this
    # loop's own clock, so the log's push times are the script's.
    push_state = 2 if args.push is None else 0
    push_client = None
    if args.push is not None:
        from rtc_msgs.srv import SetExternalWrench

        push_client = node.create_client(SetExternalWrench, "/sim/set_external_wrench")

    def push_call(force: tuple[float, float, float], event: str, t_sim: float) -> None:
        request = SetExternalWrench.Request()
        request.body_name = args.push_body
        request.wrench.force.x, request.wrench.force.y, request.wrench.force.z = force
        push_client.call_async(request)
        emit(event=event, wall=time.time(), t_sim=t_sim, body=args.push_body, force=list(force))

    try:
        if push_client is not None and not push_client.wait_for_service(timeout_sec=30.0):
            emit(event="no_push_service")
            return 5
        end = time.monotonic() + args.wait_subscriber
        while pub.get_subscription_count() < 1 and not stop["now"]:
            if time.monotonic() > end:
                emit(event="no_subscriber", topic=args.topic)
                return 3
            time.sleep(0.05)
        if stop["now"]:
            emit(event="interrupted_before_start", topic=args.topic)
            return 4
        total = script_length(script)
        t0 = time.monotonic()
        emit(event="start", wall=time.time(), topic=args.topic, rtf=args.rtf, script=args.script)
        period = 1.0 / args.rate
        last_seg = None
        while not stop["now"]:
            t_sim = (time.monotonic() - t0) * args.rtf
            if t_sim >= total:
                break
            seg = segment_at(script, t_sim)
            if seg != last_seg:
                emit(event="segment", name=seg, wall=time.time(), t_sim=t_sim)
                last_seg = seg
            vx, vy, wz = command_at(script, t_sim)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
            pub.publish(msg)
            sent += 1
            # Sleep to the next publish, waking for a push edge that comes first.
            while True:
                now = time.monotonic()
                t_sim = (now - t0) * args.rtf
                if push_state == 0 and t_sim >= args.push[0]:
                    push_call(tuple(args.push[2:]), "push_on", t_sim)
                    push_state = 1
                if push_state == 1 and t_sim >= args.push[0] + args.push[1]:
                    push_call((0.0, 0.0, 0.0), "push_off", t_sim)
                    push_state = 2
                next_publish = t0 + sent * period
                if now >= next_publish:
                    break
                wake = next_publish
                if push_state < 2:
                    edge = args.push[0] + (args.push[1] if push_state == 1 else 0.0)
                    wake = min(wake, t0 + edge / args.rtf)
                time.sleep(max(0.0, wake - now))
            if push_client is not None:
                rclpy.spin_once(node, timeout_sec=0.0)  # the service replies
    finally:
        if push_state == 1:  # interrupted under the push: do not leave it on
            push_call((0.0, 0.0, 0.0), "push_off", (time.monotonic() - t0) * args.rtf)
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
