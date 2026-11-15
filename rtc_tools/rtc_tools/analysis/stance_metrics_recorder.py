"""Record a standing floating-base robot from the simulator's sensor topic, then measure it.

Subscribes to one device group's ``rtc_msgs/SimSensorState`` topic and keeps,
per message, the base pose (a framepos + framequat sensor), each foot's centre
(a framepos sensor) and each foot's normal load (the ``force`` of a MuJoCo
``<contact>`` sensor, values 1..3, sign flipped so that pressing the floor is
positive). The samples go to a CSV in the schema of
:mod:`rtc_tools.analysis.stance_metrics`, and that module's ``compute`` turns
them into the stance numbers, printed as JSON at the end.

**When the recording starts** is the measurement's main choice. A sim that
boots in position mode and switches to PD-feedforward on the controller's
first command moves through a transient at that switch; a window that
contained it would report the transient as slip and sag. So by default the
recorder waits for the first ``rtc_msgs/JointCommand`` on ``--command-topic``
whose ``command_type`` is ``--start-on`` (default ``pd_feedforward``), starts
recording at that message, and the metrics window opens ``--settle`` seconds
later (``stance_metrics.compute``) — so ``--duration`` is the whole recording
and the judged window is ``duration − settle``. ``--start-now`` records from
the first sensor message instead. A reset (``/sim/reset``) during the recording
is not detected; record after it. Nothing arriving within ``--wait-timeout``
(wall seconds, from start) is an error, not a wait: a mistyped topic or a
controller that never activates would otherwise hang the tool silently.

The subscriptions are KEEP_LAST 1 like every other topic of this framework,
so a host that cannot keep up with the sensor rate drops samples: the metrics
carry the sample period and the longest gap, and the tool warns when the gap
exceeds a few periods.

Every sensor name is an argument: the tool knows nothing about any robot.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rtc_msgs.msg import JointCommand, SimSensorState
from rtc_tools.analysis import stance_metrics as sm

CONTACT_FORCE_OFFSET = 1  # <contact data="found force ..."> → force at values[1:4]


class StanceRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stance_metrics_recorder")
        self._args = args
        self._samples: list[sm.Sample] = []
        self._started = args.start_now
        self._start_wall = None
        self._created_wall = time.monotonic()
        self._timed_out = False
        self._missing_logged = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sensor_sub = self.create_subscription(
            SimSensorState, args.sensor_topic, self._on_sensors, qos
        )
        self._command_sub = None
        if not self._started:
            # BEST_EFFORT matches a publisher of either reliability; the sim
            # backends publish their commands best-effort.
            self._command_sub = self.create_subscription(
                JointCommand, args.command_topic, self._on_command, qos
            )
            self.get_logger().info(
                f"waiting for the first {args.start_on!r} command on {args.command_topic}"
            )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_command(self, msg: JointCommand) -> None:
        if self._started or msg.command_type != self._args.start_on:
            return
        self._started = True
        self.get_logger().info(f"{msg.command_type!r} seen — recording {self._args.duration} s")

    def _on_sensors(self, msg: SimSensorState) -> None:
        if not self._started:
            return
        if self._start_wall is None:
            self._start_wall = time.monotonic()
        by_name = {s.name: s.values for s in msg.sensors}
        try:
            pos = by_name[self._args.pos]
            quat = by_name[self._args.quat]
            feet = [by_name[n] for n in self._args.foot_pos]
            contacts = [by_name[n] for n in self._args.contact]
        except KeyError as exc:
            if not self._missing_logged:
                self._missing_logged = True
                self.get_logger().error(
                    f"sensor {exc} is not on {self._args.sensor_topic}; present: {sorted(by_name)}"
                )
            return
        loads = tuple(
            self._args.contact_sign * float(c[CONTACT_FORCE_OFFSET + 2]) for c in contacts
        )
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._samples.append(
            sm.Sample(
                t=t,
                pos=(float(pos[0]), float(pos[1]), float(pos[2])),
                quat=(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
                feet=tuple((float(f[0]), float(f[1]), float(f[2])) for f in feet),
                loads=loads,
            )
        )

    # ── Driver ─────────────────────────────────────────────────────────────

    def done(self) -> bool:
        if self._start_wall is None:
            # Nothing recorded yet: bound the wait for the start condition and
            # the first sample, so a wrong topic fails instead of hanging.
            if time.monotonic() - self._created_wall > self._args.wait_timeout:
                self._timed_out = True
                return True
            return False
        if self._samples and self._samples[-1].t - self._samples[0].t >= self._args.duration:
            return True
        # Sim time may stall (paused sim); bound the wait by wall time too.
        return time.monotonic() - self._start_wall > 3.0 * self._args.duration + 5.0

    @property
    def samples(self) -> list[sm.Sample]:
        return self._samples

    @property
    def timed_out(self) -> bool:
        return self._timed_out

    @property
    def started(self) -> bool:
        return self._started


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sensor-topic", required=True, help="SimSensorState topic of the group")
    parser.add_argument("--pos", required=True, help="framepos sensor of the base")
    parser.add_argument("--quat", required=True, help="framequat sensor of the base")
    parser.add_argument(
        "--foot-pos", nargs="+", required=True, help="framepos sensor of each foot, in foot order"
    )
    parser.add_argument(
        "--contact", nargs="+", required=True, help="<contact> sensor of each foot, same order"
    )
    parser.add_argument(
        "--contact-sign",
        type=float,
        default=-1.0,
        help="multiplies the contact sensor's fz into a load. A netforce <contact> sensor "
        "reports the force its geom1 exerts on the environment, so a sole pressing the "
        "floor reads a negative fz: -1 (the default) turns that into a positive load. "
        "Use +1 for a sensor whose geom1 is the floor",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="recording length [s, sim]")
    parser.add_argument("--settle", type=float, default=1.0, help="window opens after this [s]")
    parser.add_argument("--force-threshold", type=float, default=5.0, help="load floor [N]")
    parser.add_argument("--z-ref", type=float, default=None, help="height reference [m]")
    parser.add_argument("--csv", type=Path, required=True, help="where the samples go")
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=60.0,
        help="give up when neither the start condition nor a sample arrives within this [wall s]",
    )
    parser.add_argument("--json", type=Path, default=None, help="also write the metrics here")
    start = parser.add_mutually_exclusive_group()
    start.add_argument(
        "--command-topic", default=None, help="JointCommand topic to watch for the start"
    )
    start.add_argument("--start-now", action="store_true", help="record from the first message")
    parser.add_argument(
        "--start-on", default="pd_feedforward", help="command_type that starts the recording"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not args.start_now and args.command_topic is None:
        print("error: one of --command-topic or --start-now is required", file=sys.stderr)
        return 2
    if len(args.foot_pos) != len(args.contact):
        print("error: --foot-pos and --contact must list the same feet", file=sys.stderr)
        return 2
    rclpy.init()
    node = StanceRecorder(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        samples = node.samples
        timed_out = node.timed_out
        started = node.started
        node.destroy_node()
        rclpy.shutdown()
    if timed_out:
        what = (
            f"no {args.start_on!r} command on {args.command_topic}"
            if not started
            else f"no message on {args.sensor_topic}"
        )
        print(f"error: {what} within {args.wait_timeout} s", file=sys.stderr)
        return 1
    if not samples:
        print("error: no samples recorded", file=sys.stderr)
        return 1
    sm.write_csv(args.csv, samples, len(args.foot_pos))
    try:
        metrics = sm.compute(
            samples,
            settle_s=args.settle,
            force_threshold_n=args.force_threshold,
            z_ref=args.z_ref,
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    text = json.dumps(asdict(metrics), indent=2)
    print(text)
    if metrics.sample_gap_max_s > 3.0 * metrics.sample_dt_median_s > 0.0:
        print(
            f"warning: samples were dropped — longest gap {metrics.sample_gap_max_s:.4f} s "
            f"against a {metrics.sample_dt_median_s:.4f} s period; per-sample counts "
            "(steps_without_all_feet) may have missed an event",
            file=sys.stderr,
        )
    if args.json is not None:
        args.json.write_text(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
