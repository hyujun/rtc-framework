"""tools/catching_eval (#822): a unit that measures a value, from overlay to record.

``expect_kv.py`` turns a unit's EXPECT_KV into the approach grid it must start with
(the EXPECT_NPRE / EXPECT_DTPRE ``run_unit.sh`` checks the mirror and the startup line
against), and reads the EXPECT_KV ``mk_overlay.py --set`` wrote into an overlay. What
``run_unit.sh`` records of the unit for ``summarize.py --vary`` is pinned on its text.
"""

import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools" / "catching_eval"
sys.path.insert(0, str(TOOLS))
import expect_kv as ek  # noqa: E402

GRID = "planner.segment.mpc.approach"
DOCKING = "planner.segment.mpc_docking.approach"


def test_the_grid_follows_from_what_expect_kv_sets_for_the_units_planner():
    kv = f"{GRID}.n_pre_max=8;{GRID}.dt_pre_s=0.15"
    assert ek.approach_grid("mpc", kv, None, None) == ("8", "0.150")
    # another planner's keys say nothing of this unit's grid
    assert ek.approach_grid("mpc_docking", kv, None, None) == (None, None)
    assert ek.approach_grid("mpc", f"{DOCKING}.n_pre_max=12", None, None) == (None, None)
    assert ek.approach_grid("mpc", "", None, None) == (None, None)


def test_a_grid_step_is_written_as_the_controller_prints_it():
    assert ek.dt_text("0.15") == "0.150" and ek.dt_text("0.1") == "0.100"
    assert ek.dt_text("1e-1") == "0.100"


def test_an_expect_that_agrees_is_kept_and_one_that_contradicts_is_refused():
    kv = f"{GRID}.n_pre_max=8;{GRID}.dt_pre_s=0.15"
    assert ek.approach_grid("mpc", kv, "8", "0.150") == ("8", "0.150")
    assert ek.approach_grid("mpc", "", "6", "0.100") == ("6", "0.100")
    with pytest.raises(ValueError, match="EXPECT_NPRE 6, but EXPECT_KV sets"):
        ek.approach_grid("mpc", kv, "6", None)
    with pytest.raises(ValueError, match="EXPECT_DTPRE 0.100, but EXPECT_KV sets"):
        ek.approach_grid("mpc", kv, None, "0.100")


def test_the_command_line_is_what_run_unit_sh_reads(tmp_path):
    def run(*args):
        return subprocess.run(
            [sys.executable, str(TOOLS / "expect_kv.py"), *args], capture_output=True, text=True
        )

    said = run("grid", "mpc", f"{GRID}.n_pre_max=8", "-", "-")
    assert (said.returncode, said.stdout) == (0, "8 -\n")
    assert run("grid", "mpc", "", "-", "-").stdout == "- -\n"
    refused = run("grid", "mpc", f"{GRID}.n_pre_max=8", "6", "-")
    assert refused.returncode == 2 and "EXPECT_NPRE 6" in refused.stderr
    overlay = tmp_path / "o.yaml"
    overlay.write_text(f"# catching eval overlay: x\n# EXPECT_KV={GRID}.n_pre_max=8\na: 1\n")
    assert run("overlay", str(overlay)).stdout == f"{GRID}.n_pre_max=8\n"
    plain = tmp_path / "plain.yaml"
    plain.write_text("# catching eval overlay: x\na: 1\n# EXPECT_KV=not in the header\n")
    assert run("overlay", str(plain)).stdout == "\n"


# ── run_unit.sh ──────────────────────────────────────────────────────────────
def _text():
    return (TOOLS / "run_unit.sh").read_text()


def test_run_unit_takes_the_grid_from_expect_kv_before_it_writes_or_launches():
    text = _text()
    header = text.index('EXPECT_KV=$(python3 "$D/expect_kv.py" overlay "$OV")')
    grid = text.index('GRID=$(python3 "$D/expect_kv.py" grid "${EXPECT_MODE:-mpc}"')
    assert header < grid < text.index('echo "FAIL:expect_kv"')
    assert grid < text.index('echo "date_start:') < text.index("setsid ros2 launch")
    assert '[ "$G_NPRE" != "-" ] && EXPECT_NPRE=$G_NPRE' in text


def test_run_unit_records_the_overlay_and_the_running_builds():
    text = _text()
    block = text[text.index('echo "date_start:') : text.index('} > "$OUT/conditions.txt"')]
    assert 'echo "overlay_sha256: $(sha256sum "$OV"' in block
    # the running images, after the mirror passed and before the estimator
    shas = text.index("bin_sha integrated_rt_controller controller_bin_sha256")
    estimator = text.index("exec setsid ros2 launch ball_perception_sim")
    assert text.index('echo "FAIL:overlay$why"') < shas < estimator
    assert "bin_sha mujoco_simulator_node sim_bin_sha256" in text
    bin_sha = text[text.index("bin_sha() {") : text.index("bin_sha integrated_rt_controller")]
    assert 'readlink "$p/exe"' in bin_sha and '== "$LPG"' in bin_sha  # this unit's session


def test_run_unit_keeps_the_busiest_processes_of_a_busy_host():
    text = _text()
    top = text.index("[ $rc -eq 3 ] && ps -eo pid,pcpu,etimes,args --sort=-pcpu")
    assert text.index("rc=$?") < top < text.index('echo "FAIL:host_busy"')
    tail = text[top:]
    assert tail.index("host_busy_top.txt") < tail.index("cleanup")
