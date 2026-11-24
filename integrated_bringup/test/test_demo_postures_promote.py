"""tools/demo_postures/promote.py (demo_gui.promote) on tmp trees only.

A source config tree, an installed one (the driver in use) and a logging root
are built under tmp_path from this package's p2 / p2_left profile files; the
real checkout, share and logging_data are never touched.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import yaml

from integrated_bringup.demo_gui import promote, shipped as s

_CONFIG = Path(__file__).resolve().parents[1] / "config"
_TOOL = Path(__file__).resolve().parents[1] / "tools" / "demo_postures" / "promote.py"
ROBOTS = {"p2": "right", "p2_left": "left"}


def _preset(side, positions=None, **extra):
    return {
        "type": "open",
        "positions_deg": [1.0 * i for i in range(10)] if positions is None else positions,
        "grasp_time": 1.0,
        "side": side,
        **extra,
    }


@pytest.fixture
def trees(tmp_path):
    src, share, log = tmp_path / "src", tmp_path / "share", tmp_path / "log"
    for robot, side in ROBOTS.items():
        (src / robot / s.SHIPPED_DIR).mkdir(parents=True)
        shutil.copy(_CONFIG / robot / "_base.yaml", src / robot / "_base.yaml")
        (share / robot).mkdir(parents=True)
        driver = share / robot / "udp_hand_node_p2.yaml"
        shutil.copy(_CONFIG / robot / "udp_hand_node_p2.yaml", driver)
        # The record is whatever the driver in use runs: the values are not
        # this test's business, only that promote compares them.
        in_use = s.driver_calibration(str(driver), side)
        (src / robot / s.SHIPPED_DIR / s.CALIBRATION_FILE).write_text(
            yaml.safe_dump(
                {
                    "driver_config": in_use.driver_config,
                    "joint_names": list(in_use.joint_names),
                    "joint_direction_signs": list(in_use.joint_direction_signs),
                    "joint_position_offsets_deg": list(in_use.joint_position_offsets_deg),
                }
            )
        )
    log.mkdir()
    for robot, side in ROBOTS.items():
        _work(log, robot, {"a": _preset(side), "b": _preset(side, [0.0] * 10)})
        _work_seq(log, robot, ["a", "b"])
    return src, share, log


def _work(log, robot, presets):
    (log / f"hand_presets_{robot}.json").write_text(json.dumps(presets))


def _work_seq(log, robot, names):
    steps = [{"preset": n, "time_s": 1.0, "dwell_s": 0.0} for n in names]
    (log / f"demo_gui_sequences_{robot}.json").write_text(
        json.dumps({"steps": steps, "tolerance_deg": 2.0})
    )


def _run(trees, *argv):
    src, share, log = trees
    return promote.main([*argv, "--share-root", str(share), "--logging-root", str(log)], str(src))


def _shipped(trees, robot, name=s.PRESETS_FILE):
    path = trees[0] / robot / s.SHIPPED_DIR / name
    return json.loads(path.read_text()) if path.exists() else None


def test_promote_writes_the_working_copy_into_the_source_tree(trees):
    assert _run(trees, "--robot", "p2", "--robot", "p2_left") == promote.EXIT_OK
    log = trees[2]
    for robot in ROBOTS:
        assert _shipped(trees, robot) == json.loads(
            (log / f"hand_presets_{robot}.json").read_text()
        )
        assert _shipped(trees, robot, s.SEQUENCE_FILE)["steps"][0]["preset"] == "a"
    # What it wrote is what the GUI and the tests accept as shipped.
    s.load_shipped(str(trees[0]), s.ShippedProfile.for_robot("p2"))


def test_check_writes_nothing(trees, capsys):
    assert _run(trees, "--robot", "p2", "--check") == promote.EXIT_OK
    assert _shipped(trees, "p2") is None
    assert "added 2: a, b" in capsys.readouterr().out


def _refused(trees, capsys, *argv, match):
    before = {r: _shipped(trees, r) for r in ROBOTS}
    assert _run(trees, *argv) == promote.EXIT_REFUSED
    assert match in capsys.readouterr().err
    assert {r: _shipped(trees, r) for r in ROBOTS} == before


@pytest.mark.parametrize(
    ("presets", "match"),
    [
        ({"a": _preset("left"), "b": _preset("right")}, "another hand"),
        ({"a": {k: v for k, v in _preset("right").items() if k != "side"}}, "no 'side: right'"),
        ({"a": _preset("right", [0.0] * 9)}, "not 10 finite"),
        ({"a": _preset("right", [200.0] + [0.0] * 9)}, "joint 0 at 200.0"),
    ],
)
def test_a_working_copy_that_is_no_release_is_refused(trees, capsys, presets, match):
    _work(trees[2], "p2", presets)
    _refused(trees, capsys, "--robot", "p2", match=match)


def test_a_sequence_step_without_its_preset_is_refused(trees, capsys):
    _work_seq(trees[2], "p2", ["a", "gone"])
    _refused(trees, capsys, "--robot", "p2", match="no longer exists")


def test_the_shipped_sequence_must_still_play_when_none_is_promoted(trees, capsys):
    assert _run(trees, "--robot", "p2") == promote.EXIT_OK
    (trees[2] / "demo_gui_sequences_p2.json").unlink()
    _work(trees[2], "p2", {"a": _preset("right")})  # 'b' dropped
    _refused(trees, capsys, "--robot", "p2", match="no longer exists")


def test_a_driver_on_another_calibration_is_refused(trees, capsys):
    driver = trees[1] / "p2" / "udp_hand_node_p2.yaml"
    doc = yaml.safe_load(driver.read_text())
    params = doc["/**"]["ros__parameters"]
    params["joint_position_offsets_deg"] = [
        v + 1.0 for v in params.get("joint_position_offsets_deg", [0.0] * 10)
    ]
    driver.write_text(yaml.safe_dump(doc))
    _refused(trees, capsys, "--robot", "p2", match="is not the calibration")


def test_no_calibration_record_is_refused(trees, capsys):
    (trees[0] / "p2" / s.SHIPPED_DIR / s.CALIBRATION_FILE).unlink()
    _refused(trees, capsys, "--robot", "p2", match="calibration.yaml does not exist")


def test_one_refused_robot_writes_none(trees, capsys):
    (trees[2] / "hand_presets_p2_left.json").unlink()
    _refused(trees, capsys, "--robot", "p2", "--robot", "p2_left", match="does not exist")


def test_the_tool_writes_into_its_own_checkout():
    # The entry point hands main() the config dir next to it, not the share.
    text = _TOOL.read_text()
    assert text.startswith("#!/usr/bin/env python3\n")
    assert 'Path(__file__).resolve().parents[2] / "config"' in text
    assert (_TOOL.parents[2] / "config").is_dir()
