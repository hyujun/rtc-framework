"""The shipped throw sets (#798): ``config/<robot>/throw_sets/*.json``.

What must hold for a set to be a population the evaluations share: every file
is a ``catching_throw_list/1`` the sim driver and the offline map read as is
(the shared loader validates it — ids unique, vectors finite), it names its
robot and set, every throw carries the offline labels of both searches, and the
sets of one robot do not share a throw (a throw thrown in two sets would be
counted twice in a pooled rate). The ids of two sets may coincide — the pairing
key is the list's sha256 AND the id (``catching_throw_list.throw_key``).
"""

from __future__ import annotations

import json
import os

import pytest

from rtc_tools.analysis import catching_throw_list as ctl

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")
ROBOTS = ("ur5e_p1b", "iiwa7_leap")
SETS = {
    "D1_regression_v0.json": ("D-1 regression", 130),
    "D2_tuning_v0.json": ("D-2 tuning", {"ur5e_p1b": 30, "iiwa7_leap": 27}),
    "candidate_box_v0.json": ("candidate box", 180),
}
LABELS = ("verdict", "reason", "first_accept_wake", "t_c_s", "lead_s", "search_wall_us")


def _sets(robot):
    out = {}
    for name in SETS:
        path = os.path.join(CONFIG, robot, "throw_sets", name)
        assert os.path.isfile(path), path
        out[name] = ctl.read_throw_list_file(path)
    return out


@pytest.mark.parametrize("robot", ROBOTS)
def test_every_set_is_a_valid_list_of_its_robot_with_the_expected_size(robot):
    for name, listed in _sets(robot).items():
        tag, n = SETS[name]
        n = n[robot] if isinstance(n, dict) else n
        assert listed.meta["robot"] == robot, name
        assert listed.meta["set"] == tag and listed.meta["version"] == "v0", name
        assert len(listed.throws) == n, (name, len(listed.throws))
        assert len({t["throw_id"] for t in listed.throws}) == n, name


@pytest.mark.parametrize("robot", ROBOTS)
def test_every_throw_carries_the_offline_labels_of_both_searches(robot):
    for name, listed in _sets(robot).items():
        for t in listed.throws:
            for search in ("grid", "nlp"):
                for label in LABELS:
                    assert f"offline_{search}_{label}" in t, (name, t["throw_id"], search, label)
                assert t[f"offline_{search}_verdict"] in ("accepted", "refused")
        assert "provenance" in listed.meta and set(listed.meta["provenance"]) >= {"grid", "nlp"}


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_sets_of_a_robot_do_not_share_a_throw(robot):
    def launch(t):
        return tuple(round(float(v), 9) for v in (*t["pos"], *t["vel"]))

    seen: dict[tuple, str] = {}
    for name, listed in _sets(robot).items():
        for t in listed.throws:
            key = launch(t)
            assert key not in seen, f"{name} and {seen[key]} both throw {key}"
            seen[key] = name


def test_the_shipped_file_is_what_the_loader_reads_bit_for_bit():
    # The pairing key is the file's sha256: the file on disk must be the document the
    # loader validates — no trailing edits, one JSON document, UTF-8.
    for robot in ROBOTS:
        for name, listed in _sets(robot).items():
            path = os.path.join(CONFIG, robot, "throw_sets", name)
            with open(path, "rb") as f:
                raw = f.read()
            on_disk = json.loads(raw.decode("utf-8"))["throws"]
            assert json.loads(json.dumps(listed.throws)) == on_disk, name
