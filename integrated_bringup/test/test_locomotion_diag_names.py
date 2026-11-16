"""The locomotion diagnostic's enum labels in rtc_tools match the C++ definitions.

``rtc_tools.analysis.locomotion_diag`` names the ``mode``, ``fault_cause`` and
``ref_source`` codes of ``locomotion_diag.csv`` for its plots and its summary.
Those codes are the enums of ``logging/locomotion_diag_log_pod.hpp`` (read here
from the source tree, as test_demo_gui_hand_joint reads its header): a cause
added on one side and not the other would plot as a bare number. The CSV header
and the status topic's keys are pinned by the C++ tests.
"""

from __future__ import annotations

import re
from pathlib import Path

from rtc_tools.analysis import locomotion_diag as ld

_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "integrated_bringup"
    / "logging"
    / "locomotion_diag_log_pod.hpp"
)


def _enum_values(name: str) -> dict[str, int]:
    text = _HEADER.read_text()
    m = re.search(rf"enum class {name} : std::uint8_t \{{(.*?)\}};", text, re.S)
    assert m, f"enum {name} not found in {_HEADER}"
    return {k: int(v) for k, v in re.findall(r"\bk(\w+)\s*=\s*(\d+)", m.group(1))}


def test_the_mode_names_cover_every_mode_in_order():
    modes = _enum_values("Mode")
    assert len(ld.MODE_NAMES) == len(modes)
    assert sorted(modes.values()) == list(range(len(modes)))
    assert ld.MODE_NAMES[modes["Waiting"]] == "waiting"


def test_the_fault_cause_names_cover_every_cause_in_order():
    causes = _enum_values("FaultCause")
    assert len(ld.FAULT_CAUSE_NAMES) == len(causes)
    assert sorted(causes.values()) == list(range(len(causes)))
    assert ld.FAULT_CAUSE_NAMES[causes["PlanLoss"]] == "plan loss"
    assert ld.FAULT_CAUSE_NAMES[causes["PoseGuard"]] == "pose guard"
    assert ld.FAULT_CAUSE_NAMES[causes["None"]] == "none"


def test_the_reference_source_names_cover_every_source_in_order():
    sources = _enum_values("ReferenceSource")
    assert len(ld.REFERENCE_SOURCE_NAMES) == len(sources)
    assert ld.REFERENCE_SOURCE_NAMES[sources["Walk"]] == "walk"


def test_the_pose_guard_column_is_in_the_csv_header_and_the_docstring():
    text = _HEADER.read_text()
    assert "pose_guard_mask," in text
    assert "pose_guard_mask" in ld.__doc__
