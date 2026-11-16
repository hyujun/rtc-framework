"""tools/standing_eval/walk_gate.py (not installed — read from the source tree).

What the walking judge reads off a run: the controller's cumulative MPC
counter line, the CM's refused-output lines, the gait order of the shipped
table, a check's verdict on a missing value, and the E-STOP edge of a
walk_estop run.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

_PATH = Path(__file__).resolve().parents[1] / "tools" / "standing_eval" / "walk_gate.py"
_SPEC = importlib.util.spec_from_file_location("standing_eval_walk_gate", _PATH)
walk_gate = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(walk_gate)

GAIT_YAML = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "g1_loco"
    / "controllers"
    / "locomotion"
    / "gait.yaml"
)

MPC_LINE = (
    "[INFO] [1.0] [integrated_bringup.locomotion_controller]: [locomotion] mpc: solves {n} "
    "(failed 0, deadline miss 1, withheld 2, overrun 3), solve p50 120.5 ms p99 180.0 ms; "
    "refused: epoch 0 state 0 age {age} dim 0 implausible 0 (last check 0)\n"
)


def test_the_last_counter_line_is_read(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text("[INFO] boot\n")
    assert walk_gate.mpc_counters(log) is None
    log.write_text(MPC_LINE.format(n=10, age=4) + MPC_LINE.format(n=20, age=5))
    c = walk_gate.mpc_counters(log)
    assert c["solves"] == 20 and c["refused_age"] == 5 and c["overrun"] == 3
    assert c["solve_p99_ms"] == 180.0
    assert walk_gate.mpc_counters(tmp_path / "missing.log") is None


def test_the_cms_refusals_are_counted(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text(
        "[WARN] Command-type rejected on the RT path: x\n[INFO] ok\n" + MPC_LINE.format(n=1, age=0)
    )
    assert walk_gate.cm_rejected_lines(log) == 1, "the MPC's own `refused:` line is not the CM's"


def test_the_gait_order_is_the_shipped_sequence():
    assert walk_gate.gait_names(GAIT_YAML) == ["stance", "slow_walk", "walk"]


def test_a_missing_value_fails_its_check():
    assert not walk_gate.check("x", None, 1.0, "<=")["pass"]
    assert not walk_gate.check("x", math.nan, 1.0, "<=")["pass"]
    assert walk_gate.check("x", 1.0, 1.0, "<=")["pass"]
    assert walk_gate.check("x", 0, 0, "==")["pass"]


def _estop_run(tmp_path: Path, mode_under_estop: int, fault_jump: bool) -> Path:
    run = tmp_path / "walk_estop_0"
    diag_dir = run / walk_gate.DIAG.parent
    diag_dir.mkdir(parents=True)
    cols = ["t_relative_s", "tick", "estop", "mode", "fault_count", "base_pz"]
    rows = []
    for i in range(20):
        est = int(i >= 10)
        mode = mode_under_estop if est else 1
        fault = 1 if (fault_jump and i >= 15) else 0
        rows.append([i * 0.002, i, est, mode, fault, 0.75 - 0.001 * max(0, i - 10)])
    (run / walk_gate.DIAG).write_text(
        ",".join(cols) + "\n" + "".join(",".join(str(v) for v in r) + "\n" for r in rows)
    )
    (run / "run.json").write_text(
        json.dumps({"events": [{"name": "trigger_estop", "ok": True}], "windows": {}})
    )
    (run / "launch.log").write_text("")
    return run


def test_the_estop_edge_passes_when_held_waiting_and_fault_free(tmp_path):
    r = walk_gate.judge_estop(_estop_run(tmp_path, mode_under_estop=0, fault_jump=False))
    assert r["pass"], r["failed"]
    assert r["report"]["z_at_trigger"] == 0.75
    assert r["report"]["trigger_t_s"] == 10 * 0.002


def test_the_estop_edge_fails_on_a_mode_or_a_fault(tmp_path):
    r = walk_gate.judge_estop(_estop_run(tmp_path, mode_under_estop=1, fault_jump=True))
    assert set(r["failed"]) == {"not_waiting_under_estop", "fault_count_change"}
