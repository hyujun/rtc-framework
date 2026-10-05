"""#711: the tools of integrated_bringup refuse a renamed key and read recorded mirror names.

The helper itself is pinned in ``rtc_tools/test/test_catching_keys.py``; this is
each tool that loads a catching profile or reads a recorded ``mirror.txt``.
"""

import os
import shutil
import sys
from pathlib import Path

import pytest
import yaml

from integrated_bringup.catching_sim_trials import load_arm_profile
from rtc_tools.utils.catching_keys import MixedMirrorNamesError, RenamedCatchingKeyError

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS / "catching_eval"))
sys.path.insert(0, str(TOOLS / "catch_frame"))
import mk_overlay  # noqa: E402
import show_catch_frame  # noqa: E402
import summarize as sm  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / "config"
CONTROLLER = "demo_catching_controller"


def _profile_with(tmp_path: Path, planner_extra: dict) -> Path:
    shutil.copytree(CONFIG / "ur5e_p1b", tmp_path / "p")
    path = tmp_path / "p" / "controllers" / f"{CONTROLLER}.yaml"
    doc = yaml.safe_load(path.read_text())
    doc[CONTROLLER]["catching"].setdefault("planner", {}).update(planner_extra)
    path.write_text(yaml.safe_dump(doc))
    return tmp_path / "p"


def test_the_arm_profile_loader_refuses_a_profile_with_an_old_key(tmp_path):
    cfg = _profile_with(tmp_path, {"slice": {"dt": 0.05}})
    with pytest.raises(
        RenamedCatchingKeyError,
        match=r"catching\.planner\.slice → catching\.planner\.search\.grid\.slice",
    ):
        load_arm_profile(str(cfg))


def test_show_catch_frame_refuses_a_profile_with_the_old_pocket_key(tmp_path):
    """``planner.hand`` would read as an empty pocket and print ``d_eff=None``."""
    cfg = _profile_with(tmp_path, {"hand": {"d_eff": 0.1}})
    path = cfg / "controllers" / f"{CONTROLLER}.yaml"
    with pytest.raises(RenamedCatchingKeyError, match=r"planner\.hand"):
        show_catch_frame.catching_tree(path)


def test_show_catch_frame_reads_the_shipped_pocket_at_its_new_path():
    tree = show_catch_frame.catching_tree(
        CONFIG / "ur5e_p1b" / "controllers" / f"{CONTROLLER}.yaml"
    )
    pocket = tree["planner"]["search"]["grid"]["hand"]
    assert pocket.get("d_eff") is not None


def test_mk_overlay_writes_the_new_mode_key_and_refuses_an_old_one(tmp_path, monkeypatch):
    out = tmp_path / "o.yaml"
    mk_overlay.main([str(out), "p1b", "closed_form"])
    doc = yaml.safe_load(out.read_text())
    catching = doc["integrated_rt_controller"]["ros__parameters"][CONTROLLER]["catching"]
    assert catching["planner"]["segment"]["mode"] == "closed_form"
    assert (
        "decel" not in catching.get("supervisor", {})
        or "mode" not in catching["supervisor"]["decel"]
    )

    # an overlay source that still writes an old key is refused before anything is written
    root = tmp_path / "cfg"
    shutil.copytree(CONFIG / "ur5e_p1b", root / "ur5e_p1b")
    src = root / "ur5e_p1b" / "sim_overlays" / "catch_lead_on.yaml"
    src_doc = yaml.safe_load(src.read_text())
    catching = src_doc["integrated_rt_controller"]["ros__parameters"][CONTROLLER]["catching"]
    catching.setdefault("supervisor", {}).setdefault("decel", {})["mode"] = "mpc"
    src.write_text(yaml.safe_dump(src_doc))
    monkeypatch.setattr(mk_overlay, "REPO", root)
    with pytest.raises(
        RenamedCatchingKeyError,
        match=r"supervisor\.decel\.mode → catching\.planner\.segment\.mode",
    ):
        mk_overlay.main([str(tmp_path / "o2.yaml"), "p1b", "mpc"])
    assert not (tmp_path / "o2.yaml").exists()


def test_summarize_reads_a_mirror_txt_under_either_name_and_refuses_a_mixed_one(tmp_path):
    old = tmp_path / "old.txt"
    old.write_text(
        "planner.decel_mpc.budget.first_s: Double value is: 0.2\ncontrol.dt: Double value is: 0.002\n"
    )
    new = tmp_path / "new.txt"
    new.write_text(
        "planner.segment.mpc.budget.first_s: Double value is: 0.2\ncontrol.dt: Double value is: 0.002\n"
    )
    got_old, got_new = sm.kv_file(old), sm.kv_file(new)
    assert got_old == got_new
    assert sm.mirror_value(got_old, "planner.segment.mpc.budget.first_s") == 0.2
    mixed = tmp_path / "mixed.txt"
    mixed.write_text(old.read_text() + new.read_text())
    with pytest.raises(MixedMirrorNamesError, match="planner.decel_mpc.budget.first_s"):
        sm.kv_file(mixed)


def test_run_unit_records_and_checks_only_the_new_mirror_names():
    text = (TOOLS / "catching_eval" / "run_unit.sh").read_text()
    assert "planner.segment.mode" in text
    for old in ("planner.decel_mpc.", "supervisor.decel.mode", "supervisor.decel.switch_margin"):
        # the only place an old name may appear is the plan-refusal `case` that names it
        assert text.count(old) <= 1, old
    assert os.path.isfile(TOOLS / "catching_eval" / "run_unit.sh")
