"""Record the ball_perception prediction lane and its inputs to CSV (S3.4).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 S3.4. This is the
recording half; ``analyze_vision_lane`` reads what it writes. It subscribes to

* ``prediction/trajectory`` (PointCloud2, D-4 layout) **twice** — once
  best-effort, once reliable, both KEEP_LAST(1) — so TBD-VIS-08 is answered by
  what each subscription actually received rather than by what DDS promises;
* the sim camera lane and ground truth (to date every prediction against the
  last measurement — the ghost-track question — and to give the e2e gate a
  truth trajectory to compare across restarts);
* the estimator's diagnostics, dumped as key/value rows so ``clock_reset`` and
  ``snapshot_sequence`` behaviour across a simulator restart is on record.

Every row carries the probe's steady-clock receive time. That is the only
clock the four lanes share (stamps are wall time on the sim side and the
estimator's own on the prediction side), so all cross-lane timing here is
"as received by this process", and the report says so.

It decides nothing. A layout mismatch is counted and the message dropped — the
count is in the summary, because a lane that silently decoded nothing would
look like a lane that published nothing.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from rtc_tools.analysis.vision_lane import (
    CAMERA_COLUMNS,
    DIAG_COLUMNS,
    PREDICTION_COLUMNS,
    TRUTH_COLUMNS,
    LayoutMismatch,
    decode_trajectory,
    snapshot_row,
)

DEFAULT_PREDICTION_TOPIC = "/ball_perception/debug/prediction/trajectory"
DEFAULT_DIAGNOSTICS_TOPIC = "/ball_perception/debug/diagnostics"
DEFAULT_CAMERA_TOPIC = "/sim/ball/camera_position"
DEFAULT_TRUTH_TOPIC = "/sim/ball/ground_truth"


def _qos(reliability: ReliabilityPolicy) -> QoSProfile:
    # depth 1 on every lane: ARCH-6, and also what the eventual rtc consumer
    # will use — measuring loss with a deeper queue would answer a different
    # question.
    return QoSProfile(depth=1, reliability=reliability)


class VisionLaneProbe(Node):
    def __init__(self, prefix: Path, args: argparse.Namespace) -> None:
        super().__init__("vision_lane_probe")
        self._files = {}
        self._writers = {}
        for lane, columns in (
            ("prediction", PREDICTION_COLUMNS),
            ("camera", CAMERA_COLUMNS),
            ("truth", TRUTH_COLUMNS),
            ("diag", DIAG_COLUMNS),
        ):
            # Long-lived by design: rows arrive from callbacks for the whole
            # run and close() flushes them at exit.
            handle = Path(f"{prefix}_{lane}.csv").open("w", newline="")  # noqa: SIM115
            self._files[lane] = handle
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            self._writers[lane] = writer

        self.counts = {"best_effort": 0, "reliable": 0, "camera": 0, "truth": 0, "diag": 0}
        self.layout_mismatches = 0
        self.first_mismatch_reason: str | None = None

        self.create_subscription(
            PointCloud2,
            args.prediction_topic,
            lambda m: self._on_prediction(m, "best_effort"),
            _qos(ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            PointCloud2,
            args.prediction_topic,
            lambda m: self._on_prediction(m, "reliable"),
            _qos(ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            PointStamped, args.camera_topic, self._on_camera, _qos(ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(
            Odometry, args.truth_topic, self._on_truth, _qos(ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(
            DiagnosticArray,
            args.diagnostics_topic,
            self._on_diag,
            _qos(ReliabilityPolicy.RELIABLE),
        )

    @staticmethod
    def _now_ns() -> int:
        return time.monotonic_ns()

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _on_prediction(self, msg: PointCloud2, sub: str) -> None:
        recv = self._now_ns()
        try:
            snap = decode_trajectory(
                stamp_ns=self._stamp_ns(msg.header.stamp),
                frame_id=msg.header.frame_id,
                fields=[(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                point_step=msg.point_step,
                is_bigendian=msg.is_bigendian,
                width=msg.width,
                height=msg.height,
                data=bytes(msg.data),
            )
        except LayoutMismatch as exc:
            self.layout_mismatches += 1
            if self.first_mismatch_reason is None:
                self.first_mismatch_reason = str(exc)
            return
        self.counts[sub] += 1
        self._writers["prediction"].writerow(snapshot_row(recv, sub, snap))

    def _on_camera(self, msg: PointStamped) -> None:
        self.counts["camera"] += 1
        self._writers["camera"].writerow(
            {
                "recv_ns": self._now_ns(),
                "stamp_ns": self._stamp_ns(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "x": msg.point.x,
                "y": msg.point.y,
                "z": msg.point.z,
            }
        )

    def _on_truth(self, msg: Odometry) -> None:
        self.counts["truth"] += 1
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._writers["truth"].writerow(
            {
                "recv_ns": self._now_ns(),
                "stamp_ns": self._stamp_ns(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "vx": v.x,
                "vy": v.y,
                "vz": v.z,
            }
        )

    def _on_diag(self, msg: DiagnosticArray) -> None:
        self.counts["diag"] += 1
        recv = self._now_ns()
        for status in msg.status:
            for kv in status.values:
                self._writers["diag"].writerow(
                    {"recv_ns": recv, "status": status.name, "key": kv.key, "value": kv.value}
                )

    def close(self) -> None:
        for handle in self._files.values():
            handle.flush()
            handle.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("prefix", help="CSV prefix: <prefix>_{prediction,camera,truth,diag}.csv")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 = until SIGINT")
    parser.add_argument("--prediction-topic", default=DEFAULT_PREDICTION_TOPIC)
    parser.add_argument("--diagnostics-topic", default=DEFAULT_DIAGNOSTICS_TOPIC)
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument("--truth-topic", default=DEFAULT_TRUTH_TOPIC)
    args = parser.parse_args(argv)

    rclpy.init()
    probe = VisionLaneProbe(Path(args.prefix), args)
    stop = {"flag": False}

    def _sigint(*_):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)
    deadline = time.monotonic() + args.duration_s if args.duration_s > 0 else None
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if deadline is not None and time.monotonic() >= deadline:
                break
    finally:
        probe.close()
        counts = probe.counts
        print(
            f"vision_lane_probe: prediction best_effort {counts['best_effort']} / reliable "
            f"{counts['reliable']}, camera {counts['camera']}, truth {counts['truth']}, "
            f"diagnostics {counts['diag']}, layout mismatches {probe.layout_mismatches}"
            + (f" (first: {probe.first_mismatch_reason})" if probe.first_mismatch_reason else ""),
            file=sys.stderr,
        )
        probe.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
