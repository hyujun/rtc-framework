"""The catching controller's reset table covers every RT-owned member (G8-A2).

L7 §4.8 asks that EVERY stateful member the RT tick owns is either put back by
a reset or explicitly exempt, with the reason. The table lives as a comment in
the controller header, next to the members; this test is the "existence"
half of G8-A2 — a member declared inside an ``RT-OWNED BEGIN`` / ``RT-OWNED
END`` range that the table does not name is red. (Whether the table's CLAIMS
are true is the runtime half: the poison test in
test_catching_supervisor_scenarios / test_catching_reset_probe.)

The ranges are the declaration "this member is tick state": a member added
inside one is caught here, and one added outside every range is what review
has to catch — the marker lines are what it reads.
"""

from __future__ import annotations

import re
from pathlib import Path

HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "integrated_bringup"
    / "controllers"
    / "demo_catching_controller.hpp"
)

# `Type name_{init};`, `Type name_;`, `static constexpr ... name{...};` is not a
# member of instance state and is skipped. Templates with commas are allowed.
_MEMBER = re.compile(r"^\s+(?!static\b)(?:[\w:<>,\s\*&]+?)\s+(\w+_)\s*(?:\{[^;]*\})?\s*;")


def _rt_owned_members(text: str) -> list[str]:
    members: list[str] = []
    inside = False
    for line in text.splitlines():
        if "RT-OWNED BEGIN" in line:
            inside = True
            continue
        if "RT-OWNED END" in line:
            inside = False
            continue
        if not inside or line.lstrip().startswith("//"):
            continue
        m = _MEMBER.match(line)
        if m:
            members.append(m.group(1))
    return members


def _reset_table(text: str) -> str:
    start = text.index("// ── Reset table (L7 §4.8, G8-A2)")
    end = text.index("\n\n", start)
    return text[start:end]


def test_the_header_marks_its_rt_owned_state():
    text = HEADER.read_text()
    assert text.count("RT-OWNED BEGIN") == text.count("RT-OWNED END") > 0
    # Sanity: the ranges hold the members the S5 reset always handled, so an
    # emptied range cannot pass this test by accident.
    members = _rt_owned_members(text)
    for anchor in ("mode_", "plan_active_", "arm_q_cmd_", "hand_hold_", "outcome_"):
        assert anchor in members, f"{anchor} is not inside an RT-OWNED range"


def test_every_rt_owned_member_is_in_the_reset_table():
    text = HEADER.read_text()
    table = _reset_table(text)
    missing = [m for m in _rt_owned_members(text) if not re.search(rf"\b{m}", table)]
    assert not missing, (
        "RT-owned members with no row in the reset table (L7 §4.8): "
        + ", ".join(missing)
        + " — add each to ResetForRearm / ResetTrialState, or to the table as exempt with "
        "the reason"
    )


def test_the_member_pattern_reads_the_declarations_it_must():
    # The pattern is the sensor; check it on the declaration shapes the header
    # uses, including the ones a naive pattern drops.
    sample = """
  // RT-OWNED BEGIN
  bool plain_{false};
  std::array<double, kDemoCatchingMaxArmDof> arr_{};
  rtc::catching::Mode mode_{rtc::catching::Mode::kIdle};
  std::atomic<std::uint8_t> atom_{static_cast<std::uint8_t>(1)};
  RetreatStage stage_{RetreatStage::kStop};
  static constexpr std::size_t kTips = 4;
  // commented_{}
  int no_init_;
  // RT-OWNED END
  int outside_{0};
"""
    assert _rt_owned_members(sample) == ["plain_", "arr_", "mode_", "atom_", "stage_", "no_init_"]
