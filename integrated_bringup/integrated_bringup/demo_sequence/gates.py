"""The run's own watches, beside the stack checks of ``readiness`` — rclpy-free.

- ``DwellWatch``: while the hand rests at a step, a pose that drifts far from
  the target for long is a hand that stopped holding (motor power gone with
  the joint states still coming). A short or small drift is a visitor touching
  the hand: recorded, not fatal.
- ``disk_verdict``: the logs the stack is writing, measured, must fit the rest
  of the run with a margin.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

DEFAULT_HOLD_ABORT_AFTER_S = 1.0
MIN_HOLD_ABORT_AFTER_S = 0.1
MAX_HOLD_ABORT_AFTER_S = 60.0
# The drift that counts: max(3 x tolerance, tolerance + 5 deg).
DRIFT_FACTOR = 3.0
DRIFT_MARGIN_DEG = 5.0

DISK_MARGIN = 1.5


def drift_threshold_deg(tolerance_deg: float) -> float:
    return max(DRIFT_FACTOR * tolerance_deg, tolerance_deg + DRIFT_MARGIN_DEG)


def worst_error(
    target: Sequence[float], measured: Sequence[float], names: Sequence[str]
) -> tuple[float, str]:
    """(largest |target - measured| [rad], its joint name); (inf, "") when the
    sizes differ — a pose that cannot be compared is not a held pose."""
    if len(target) != len(measured) or not target:
        return math.inf, ""
    errors = [abs(t - m) for t, m in zip(target, measured, strict=True)]
    worst = max(range(len(errors)), key=errors.__getitem__)
    return errors[worst], names[worst] if worst < len(names) else f"joint {worst + 1}"


@dataclass
class DriftRecord:
    step: int  # 1-based
    preset: str
    peak_deg: float
    duration_s: float
    joint: str


class DwellWatch:
    """Feed it every tick of a dwell; it says when a drift has lasted too long.

    ``feed`` returns the abort reason, or "". A drift that ends before the
    limit is appended to ``records`` (the summary's visitor-contact list).
    """

    def __init__(self, tolerance_deg: float, abort_after_s: float) -> None:
        self.threshold_rad = math.radians(drift_threshold_deg(tolerance_deg))
        self.abort_after_s = abort_after_s
        self.records: list[DriftRecord] = []
        self._since: float | None = None
        self._peak = 0.0
        self._joint = ""
        self._where = (0, "")

    def feed(self, now: float, step: int, preset: str, error_rad: float, joint: str) -> str:
        if error_rad <= self.threshold_rad:
            self.settle(now)
            return ""
        if self._since is None:
            self._since, self._peak, self._where = now, 0.0, (step, preset)
        if error_rad > self._peak:
            self._peak, self._joint = error_rad, joint
        held = now - self._since
        if held > self.abort_after_s:
            return (
                f"step {step} '{preset}': {self._joint or 'the hand'} drifted "
                f"{_deg(self._peak)} deg from the target for {held:.1f} s "
                f"(limit {_deg(self.threshold_rad)} deg for {self.abort_after_s:.1f} s) — "
                "the hand is not holding (motor power?)"
            )
        return ""

    def settle(self, now: float) -> None:
        """The dwell ended or the pose came back: close an open drift."""
        if self._since is not None:
            step, preset = self._where
            self.records.append(
                DriftRecord(step, preset, math.degrees(self._peak), now - self._since, self._joint)
            )
        self._since = None


def _deg(rad: float) -> str:
    return "inf" if math.isinf(rad) else f"{math.degrees(rad):.1f}"


def disk_verdict(free_bytes: int, rate_bytes_s: float, remaining_s: float) -> str:
    """Why the logs do not fit the remaining run ("" = they do)."""
    need = max(rate_bytes_s, 0.0) * max(remaining_s, 0.0) * DISK_MARGIN
    if free_bytes >= need:
        return ""
    return (
        f"{free_bytes / 1e9:.1f} GB free, the logs need {need / 1e9:.1f} GB "
        f"({rate_bytes_s / 1e3:.0f} kB/s x {remaining_s / 3600:.1f} h x {DISK_MARGIN}) — "
        "free space, or turn the high-rate logs off (operator doc)"
    )
