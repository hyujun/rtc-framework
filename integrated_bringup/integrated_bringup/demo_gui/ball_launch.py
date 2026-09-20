"""Ball launch panel state for demo_controller_gui (dynamic_catching §13 S3).

Two jobs, both of which exist because the alternative is a plausible-looking
wrong answer.

**Parsing the launch condition.** ``/sim/launch_ball_at`` takes a stated release
state ``(p0, v0, omega)`` and refuses anything non-finite rather than dropping
it (``rtc_msgs/srv/LaunchBall.srv``). The panel refuses the same set BEFORE the
wire, so a typo comes back as "velocity: 'l.5' is not a number" next to the box
that holds it instead of as a service response the operator has to go and read.
The refusal set is mirrored deliberately, and the mirror is what the tests pin:
a panel that accepted something the service refuses would report a launch that
never happened.

**Saying what is and is not arriving.** The panel shows ground truth and the
vision prediction as three states, not two: *never seen*, *live*, and *stale*.
Collapsing "never published" into "stale" is the failure that matters here —
during bring-up those two look identical on screen while meaning opposite
things (nothing is running vs something stopped), and the operator's next
action differs.

Pure Python — no Tk, no rclpy — so this is unit-testable without a display or a
ROS graph, following ``demo_gui.pull`` / ``demo_gui.task_frame``.

Public surface (imported by app.py):
- BALL_TRUTH_TOPIC, BALL_PREDICTION_TOPIC
- LaunchCondition, parse_launch_condition
- FeedState, FeedStatus, BallStatus
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

# Owner: rtc_mujoco_sim `projectile_ball.publish.ground_truth_topic` (shipped
# default). The panel reads the shipped name rather than taking it from a
# parameter, because a run that renamed it is a run where "nothing arriving" is
# the honest readout.
BALL_TRUTH_TOPIC = "/sim/ball/ground_truth"

# Owner: ball_perception `sim_estimator_node` debug lane. Its publisher QoS is
# hardcoded RELIABLE/VOLATILE/KEEP_LAST(10) in that package, so a subscriber
# here has to match; the profile cannot change it.
BALL_PREDICTION_TOPIC = "/ball_perception/debug/prediction/trajectory"

# A feed is stale once this much wall time has passed without a message. The
# truth lane publishes at `projectile_ball.publish.sample_rate_hz` (100 Hz
# shipped) and the prediction lane at <= 30 Hz, so half a second is many missed
# messages on either — long enough not to flicker while the ball is parked and
# the lane is legitimately silent between throws.
FEED_STALE_AFTER_S = 0.5

PLACEHOLDER = "--"


class FeedState(Enum):
    """Three states, because "never" and "stopped" call for different actions."""

    NEVER = "never"
    LIVE = "live"
    STALE = "stale"


@dataclass(frozen=True)
class LaunchCondition:
    """A validated release state, world frame."""

    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    spin_rad_s: tuple[float, float, float]


def _parse_triple(text: str, label: str) -> tuple[float, float, float]:
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError(f"{label}: expected 3 numbers, got {len(parts)}")
    values = []
    for part in parts:
        try:
            value = float(part)
        except ValueError as exc:
            raise ValueError(f"{label}: '{part}' is not a number") from exc
        if not math.isfinite(value):
            # The same refusal the service makes, and for the same reason: a
            # non-finite value reaches mjData::qpos and poisons the whole scene,
            # not just the ball.
            raise ValueError(f"{label}: '{part}' is not finite")
        values.append(value)
    return (values[0], values[1], values[2])


def parse_launch_condition(position: str, velocity: str, spin: str) -> LaunchCondition:
    """Parse the three entry boxes. Raises ValueError naming the offending field.

    Zero is accepted everywhere: a zero velocity is a drop and a zero spin is a
    spinless throw. Neither is "unset", and treating them as such would silently
    substitute a throw the operator did not ask for.
    """
    return LaunchCondition(
        position_m=_parse_triple(position, "position"),
        velocity_m_s=_parse_triple(velocity, "velocity"),
        spin_rad_s=_parse_triple(spin, "spin"),
    )


@dataclass
class FeedStatus:
    """Liveness of one topic, judged on a monotonic clock supplied by the caller."""

    name: str
    last_seen_s: float | None = None
    count: int = 0

    def mark(self, now_s: float) -> None:
        self.last_seen_s = now_s
        self.count += 1

    def state(self, now_s: float, stale_after_s: float = FEED_STALE_AFTER_S) -> FeedState:
        if self.last_seen_s is None:
            return FeedState.NEVER
        return FeedState.LIVE if (now_s - self.last_seen_s) <= stale_after_s else FeedState.STALE

    def label(self, now_s: float, stale_after_s: float = FEED_STALE_AFTER_S) -> str:
        state = self.state(now_s, stale_after_s)
        if state is FeedState.NEVER:
            return f"{self.name}: never received"
        age_ms = (now_s - (self.last_seen_s or now_s)) * 1e3
        if state is FeedState.LIVE:
            return f"{self.name}: live ({self.count} msgs)"
        return f"{self.name}: STALE {age_ms:.0f} ms ({self.count} msgs)"


@dataclass
class BallStatus:
    """The panel's readout: both feeds plus the last service outcome."""

    truth: FeedStatus = field(default_factory=lambda: FeedStatus("ground truth"))
    prediction: FeedStatus = field(default_factory=lambda: FeedStatus("vision prediction"))
    last_result: str = PLACEHOLDER
    last_accepted: bool | None = None

    def record_launch(self, accepted: bool, message: str) -> None:
        """Keep the service's own words. A refusal names its reason, and
        replacing it with a generic 'failed' throws away the one thing that
        tells the operator which field to fix."""
        self.last_accepted = accepted
        self.last_result = ("accepted: " if accepted else "REFUSED: ") + (message or PLACEHOLDER)

    def lines(self, now_s: float) -> list[str]:
        return [
            self.truth.label(now_s),
            self.prediction.label(now_s),
            f"last launch: {self.last_result}",
        ]
