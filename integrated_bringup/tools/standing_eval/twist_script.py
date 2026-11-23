#!/usr/bin/env python3
"""Publish a velocity command script as geometry_msgs/Twist, then rest.

The script is one of rtc_tools.analysis.walk_metrics.COMMAND_SCRIPTS
(``--script``, default ``walk`` = WALK_SCRIPT; sim seconds). It waits for a
subscriber (the controller's), then publishes at ``--rate`` Hz on
the wall clock, the command of sim time ``wall elapsed × --rtf``, with depth 1.
``--clock sim`` takes the script's time from the simulator instead: t_sim is
the latest ``/sim/time`` (std_msgs/Float64) minus its value when the script
started — the clock of an MPC lock-step run, where the real-time factor follows
the solve and ``wall × rtf`` is wrong (``--rtf`` is then ignored). The segment
lookup, the push edges and the script's length all use that t_sim; publishing
stays on the wall clock at ``--rate``.
After the script it publishes the zero command ``--rest-count`` times and
exits 0; SIGINT (the caller's stop) does the same at once.

Its stdout is JSON lines: ``{"event": "start", "wall": …, "clock": …}`` when the script
starts (after the subscriber matched), one per segment boundary, and
``{"event": "end", …}`` with how many messages went out. A subscriber that
never comes is exit 3, a stop before it came exit 4, a ``--clock sim`` run that
gets no ``/sim/time`` within ``--wait-sim-time`` exit 6 (event ``no_sim_time``) —
none has a start line.
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


class WallAxis:
    """t_sim = wall elapsed × rtf (the default clock)."""

    def __init__(self, rtf: float) -> None:
        self._rtf = rtf
        self._t0 = time.monotonic()

    def elapsed(self) -> float:
        return (time.monotonic() - self._t0) * self._rtf

    def wake_in(self, until_sim: float) -> float:
        """Wall seconds until t_sim reaches `until_sim`."""
        return (until_sim - self.elapsed()) / self._rtf


class SimAxis:
    """t_sim = the latest sim time − the sim time at the script's start (--clock sim)."""

    POLL_S = 0.01  # a sim edge cannot be predicted in wall time: look this often

    def __init__(self) -> None:
        self.latest: float | None = None
        self._origin: float | None = None

    def observe(self, sim_time: float) -> None:
        self.latest = sim_time

    def start(self) -> None:
        """The script's t_sim = 0 is the latest sim time now."""
        self._origin = self.latest

    def elapsed(self) -> float:
        if self._origin is None or self.latest is None:
            return 0.0
        return self.latest - self._origin

    def wake_in(self, until_sim: float) -> float:
        return min(self.POLL_S, max(0.0, until_sim - self.elapsed()))


def emit(**fields) -> None:
    print(json.dumps(fields), flush=True)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--topic", default="/locomotion_controller/g1/cmd_vel")
    p.add_argument(
        "--rtf",
        type=float,
        default=1.0,
        help="sim seconds per wall second (--clock wall; ignored under --clock sim)",
    )
    p.add_argument(
        "--clock",
        choices=("wall", "sim"),
        default="wall",
        help="wall: t_sim = wall elapsed × --rtf; sim: t_sim from /sim/time (MPC lock-step)",
    )
    p.add_argument("--sim-time-topic", default="/sim/time")
    p.add_argument(
        "--wait-sim-time", type=float, default=30.0, help="--clock sim: first message [s, wall]"
    )
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
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float64

    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("walk_twist_script")
    pub = node.create_publisher(Twist, args.topic, 1)
    sent = 0
    sim_axis = SimAxis()
    if args.clock == "sim":
        node.create_subscription(
            Float64,
            args.sim_time_topic,
            lambda m: sim_axis.observe(float(m.data)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
    axis: WallAxis | SimAxis = sim_axis
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
        if args.clock == "sim":
            end = time.monotonic() + args.wait_sim_time
            while sim_axis.latest is None and not stop["now"]:
                if time.monotonic() > end:
                    emit(event="no_sim_time", topic=args.sim_time_topic)
                    return 6
                rclpy.spin_once(node, timeout_sec=0.05)
            if stop["now"]:
                emit(event="interrupted_before_start", topic=args.topic)
                return 4
            sim_axis.start()
        else:
            axis = WallAxis(args.rtf)
        total = script_length(script)
        t0 = time.monotonic()
        emit(
            event="start",
            wall=time.time(),
            topic=args.topic,
            rtf=args.rtf,
            clock=args.clock,
            script=args.script,
        )
        period = 1.0 / args.rate
        last_seg = None
        while not stop["now"]:
            t_sim = axis.elapsed()
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
                if args.clock == "sim":
                    rclpy.spin_once(node, timeout_sec=0.0)  # the latest /sim/time
                t_sim = axis.elapsed()
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
                    wake = min(wake, now + axis.wake_in(edge))
                if args.clock == "sim":
                    # Spin to the wake (the subscription updates), polling for a sim edge.
                    rclpy.spin_once(node, timeout_sec=max(0.0, min(wake - now, SimAxis.POLL_S)))
                else:
                    time.sleep(max(0.0, wake - now))
            if push_client is not None and args.clock != "sim":
                rclpy.spin_once(node, timeout_sec=0.0)  # the service replies
    finally:
        if push_state == 1:  # interrupted under the push: do not leave it on
            push_call((0.0, 0.0, 0.0), "push_off", axis.elapsed())
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
