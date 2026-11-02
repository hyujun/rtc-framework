"""Pose list, pose file and replay of the hand joint-command GUI.

Runs on a fake clock (plain floats) — no rclpy, no Qt. The controller is the
real ``HandController``; ``tick`` then ``step`` per period, as ``CommandLoop``
does.
"""

from __future__ import annotations

import math

import pytest
import yaml

from rtc_tools.gui.joint_command_model import (
    MAX_TICK_DT_S,
    GuiConfig,
    HandController,
    HandSample,
)
from rtc_tools.gui.joint_command_poses import (
    MAX_DWELL_S,
    Pose,
    PoseError,
    PoseSequencer,
    parse_pose_file,
    pose_file_data,
)

NAMES = ("j0", "j1")
STALE_S = 0.1
DT = 0.02
RANGES = {"j0": (-20.0, 20.0), "j1": (-20.0, 20.0)}


def sample(deg, t, names=NAMES):
    return HandSample(
        names=tuple(names),
        position_rad=tuple(math.radians(d) for d in deg),
        effort=tuple(0.0 for _ in deg),
        received_s=t,
    )


def make(ranges=RANGES, slew=90.0, dwell=1.0, label="right"):
    cfg = GuiConfig(
        publish_rate_hz=1.0 / DT,
        slew_rate_deg_s=slew,
        stale_timeout_s=STALE_S,
        ranges_deg={label: ranges},
    )
    ctrl = HandController(label, cfg)
    return ctrl, PoseSequencer(ctrl, dwell)


def ready(**kw):
    ctrl, seq = make(**kw)
    ctrl.on_sample(sample((0.0, 0.0), 0.0))
    return ctrl, seq


def enabled(**kw):
    ctrl, seq = ready(**kw)
    assert ctrl.request_enable(0.0) is None
    return ctrl, seq


def pose(name, j0, j1, dwell=1.0):
    return Pose(name, dwell, {"j0": j0, "j1": j1})


class Clock:
    """Feeds a fresh sample, ticks and steps once per period."""

    def __init__(self, ctrl, seq, t=0.0):
        self.ctrl, self.seq, self.t = ctrl, seq, t
        self.published = []

    def step(self, dt=DT):
        self.t += dt
        self.ctrl.on_sample(sample(self.ctrl.command_deg, self.t))
        out = self.ctrl.tick(self.t)
        self.published.append(out)
        self.seq.step(self.t)
        return out

    def run(self, n, dt=DT):
        for _ in range(n):
            self.step(dt)


# ── saving ──────────────────────────────────────────────────────────────


def test_save_stores_target_while_enabled():
    ctrl, seq = enabled()
    ctrl.set_target(0, 5.0)
    ctrl.on_sample(sample((1.0, 2.0), 0.01))
    assert seq.save_pose() is None
    assert seq.poses[0].joints_deg == {"j0": 5.0, "j1": 0.0}
    assert seq.poses[0].name == "pose 1"
    assert seq.poses[0].dwell_s == 1.0
    assert seq.dirty and seq.revision == 1


def test_save_stores_measurement_while_disabled():
    ctrl, seq = ready()
    ctrl.set_target(0, 5.0)
    ctrl.on_sample(sample((1.0, 2.0), 0.01))
    assert seq.save_pose() is None
    assert seq.poses[0].joints_deg == pytest.approx({"j0": 1.0, "j1": 2.0})


def test_save_uses_the_configured_dwell():
    ctrl, seq = ready(dwell=2.5)
    seq.save_pose()
    assert seq.poses[0].dwell_s == 2.5


def test_save_refusals():
    ctrl, seq = make()
    assert "no joint_states" in seq.save_pose()
    assert seq.message == seq.save_pose()
    ctrl.on_sample(sample((math.nan, 0.0), 0.0))
    assert "not finite" in seq.save_pose()
    assert seq.poses == []
    assert not seq.dirty


def test_update_follows_the_same_rule_as_save():
    ctrl, seq = enabled()
    seq.save_pose()
    ctrl.set_target(1, -7.0)
    assert seq.update_pose(0) is None
    assert seq.poses[0].joints_deg["j1"] == -7.0
    ctrl.disable()
    ctrl.on_sample(sample((3.0, 4.0), 0.01))
    assert seq.update_pose(0) is None
    assert seq.poses[0].joints_deg == pytest.approx({"j0": 3.0, "j1": 4.0})
    assert "no pose selected" in seq.update_pose(None)
    assert "no pose selected" in seq.update_pose(1)


def test_list_editing():
    ctrl, seq = ready()
    for i in range(3):
        ctrl.on_sample(sample((float(i), 0.0), 0.0))
        seq.save_pose()
    assert [p.name for p in seq.poses] == ["pose 1", "pose 2", "pose 3"]
    assert seq.move_pose(2, -1) is None
    assert [p.name for p in seq.poses] == ["pose 1", "pose 3", "pose 2"]
    assert seq.move_pose(0, -1) is not None
    assert seq.move_pose(2, +1) is not None
    assert seq.rename_pose(1, "  open ") is None
    assert seq.poses[1].name == "open"
    assert seq.rename_pose(1, "  ") is not None
    assert seq.set_dwell(1, 0.0) is None
    assert seq.set_dwell(1, MAX_DWELL_S) is None
    assert seq.set_dwell(1, MAX_DWELL_S + 0.01) is not None
    assert seq.set_dwell(1, -1.0) is not None
    assert seq.set_dwell(1, math.nan) is not None
    assert seq.delete_pose(1) is None
    assert [p.name for p in seq.poses] == ["pose 1", "pose 2"]
    assert seq.delete_pose(5) is not None
    assert seq.revision == 8  # 3 saves · move · rename · 2 dwells · delete


# ── replay ──────────────────────────────────────────────────────────────


def test_play_walks_the_list_with_dwell():
    ctrl, seq = enabled(slew=100.0, dwell=0.1)
    seq.poses = [pose("a", 2.0, 0.0, dwell=0.1), pose("b", 2.0, -4.0, dwell=0.1)]
    clock = Clock(ctrl, seq)
    assert seq.play() is None
    assert seq.playing and seq.current == 0
    assert ctrl.target_deg == [2.0, 0.0]
    # 2 deg at 100 deg/s = one period
    clock.step()
    assert ctrl.command_deg == [2.0, 0.0]
    arrived = clock.t
    assert seq.current == 0
    # dwell boundary: just before stays, at the deadline advances
    while clock.t + DT < arrived + 0.1 - 1e-9:
        clock.step()
        assert seq.current == 0
    clock.step(arrived + 0.1 - clock.t - 1e-6)
    assert seq.current == 0
    clock.step(1e-6)
    assert seq.current == 1
    assert ctrl.target_deg == [2.0, -4.0]
    # 4 deg at 100 deg/s: two periods, a third if the float dt fell short
    clock.run(2)
    if ctrl.command_deg != [2.0, -4.0]:
        clock.step()
    assert ctrl.command_deg == [2.0, -4.0]
    assert seq.playing
    clock.run(6)
    assert not seq.playing
    assert seq.current is None
    assert "done" in seq.message
    # the last pose is held and the hand stays enabled
    assert ctrl.target_deg == [2.0, -4.0]
    assert ctrl.enabled
    assert all(out is not None for out in clock.published)


def test_dwell_zero_advances_on_arrival():
    ctrl, seq = enabled(slew=0.0)
    seq.poses = [pose("a", 1.0, 0.0, dwell=0.0), pose("b", 2.0, 0.0, dwell=0.0)]
    clock = Clock(ctrl, seq)
    seq.play()
    clock.step()
    assert ctrl.command_deg[0] == 1.0
    assert seq.current == 1
    clock.step()
    assert ctrl.command_deg[0] == 2.0
    assert not seq.playing


def test_dwell_waits_through_a_stall():
    # a stalled GUI integrates at most MAX_TICK_DT_S per tick: the slew does
    # not jump, so arrival is still decided by the command, not by the clock
    ctrl, seq = enabled(slew=90.0)
    seq.poses = [pose("a", 18.0, 0.0, dwell=0.0)]
    clock = Clock(ctrl, seq)
    seq.play()
    clock.step(1.0)
    assert ctrl.command_deg[0] == pytest.approx(90.0 * MAX_TICK_DT_S)
    assert seq.playing


def test_play_from_arrival_holds_one_period_first():
    ctrl, seq = enabled(slew=0.0)
    seq.poses = [pose("here", 0.0, 0.0, dwell=0.0)]
    seq.play()
    assert ctrl.command_deg == ctrl.target_deg
    # not finished by play() itself: step decides
    assert seq.playing
    Clock(ctrl, seq).step()
    assert not seq.playing


@pytest.mark.parametrize(
    ("prepare", "needle"),
    [
        (lambda ctrl, seq: ctrl.disable(), "not enabled"),
        (lambda ctrl, seq: seq.poses.clear(), "no poses"),
        (lambda ctrl, seq: seq.poses.append(pose("far", 25.0, 0.0)), "'far': j0 25.0"),
        (lambda ctrl, seq: seq.play(), "in progress"),
        (lambda ctrl, seq: ctrl.set_range(0, -20.0, 5.0), "'b': j0 10.0"),
    ],
)
def test_play_refusals(prepare, needle):
    ctrl, seq = enabled(slew=0.0)
    seq.poses = [pose("a", 1.0, 0.0), pose("b", 10.0, 0.0)]
    prepare(ctrl, seq)
    reason = seq.play()
    assert reason is not None and needle in reason
    assert seq.message == reason


def test_out_of_range_pose_is_rejected_whole_not_clamped():
    ctrl, seq = enabled(slew=0.0)
    seq.poses = [pose("a", 1.0, 0.0), pose("b", 30.0, 0.0)]
    assert seq.play() is not None
    assert not seq.playing
    assert ctrl.target_deg == [0.0, 0.0]


def test_stop_freezes_where_the_command_is():
    ctrl, seq = enabled(slew=100.0)
    seq.poses = [pose("a", 10.0, 0.0), pose("b", -10.0, 0.0)]
    clock = Clock(ctrl, seq)
    seq.play()
    clock.run(2)
    assert ctrl.command_deg[0] == pytest.approx(4.0)
    seq.stop()
    assert not seq.playing
    assert ctrl.target_deg == ctrl.command_deg
    frozen = list(ctrl.command_deg)
    clock.run(10)
    assert ctrl.target_deg == frozen
    assert ctrl.command_deg == frozen
    assert ctrl.enabled
    assert "stopped" in seq.message
    # Play again starts over from the first pose
    assert seq.play() is None
    assert seq.current == 0
    assert ctrl.target_deg == [10.0, 0.0]


def test_stop_when_not_playing_is_a_noop():
    ctrl, seq = enabled()
    ctrl.set_target(0, 5.0)
    seq.stop()
    assert ctrl.target_deg == [5.0, 0.0]
    assert seq.message == ""


def _go_stale(ctrl, clock):
    # a period without a new sample past the stale timeout
    clock.t += STALE_S + 0.01
    ctrl.tick(clock.t)
    clock.seq.step(clock.t)


@pytest.mark.parametrize(
    ("cut", "needle"),
    [
        (lambda ctrl, clock: ctrl.disable(), "disabled"),
        (lambda ctrl, clock: ctrl.set_link(False), "link_status"),
        (lambda ctrl, clock: ctrl.set_foreign_publisher(True), "another node"),
        (_go_stale, "stale"),
    ],
)
def test_replay_ends_in_the_period_the_hand_is_disabled(cut, needle):
    ctrl, seq = enabled(slew=100.0)
    seq.poses = [pose("a", 10.0, 0.0), pose("b", -10.0, 0.0)]
    clock = Clock(ctrl, seq)
    seq.play()
    clock.run(2)
    target_before = list(ctrl.target_deg)
    cut(ctrl, clock)
    if seq.playing:
        clock.step()
    assert not ctrl.enabled
    assert not seq.playing
    assert needle in seq.message
    # the sequencer never writes a target while the hand is disabled
    assert ctrl.target_deg == target_before
    clock.run(3)
    assert ctrl.target_deg == target_before


def test_go_to_ends_at_arrival_without_dwell():
    ctrl, seq = enabled(slew=100.0)
    seq.poses = [pose("a", 10.0, 0.0, dwell=5.0), pose("b", -2.0, 0.0, dwell=5.0)]
    clock = Clock(ctrl, seq)
    assert seq.go_to(1) is None
    assert seq.playing and seq.current == 1
    assert ctrl.target_deg == [-2.0, 0.0]
    clock.step()
    assert ctrl.command_deg == [-2.0, 0.0]
    assert not seq.playing
    assert "reached 'b'" in seq.message
    clock.run(3)
    assert ctrl.target_deg == [-2.0, 0.0]
    assert seq.go_to(None) is not None
    assert seq.go_to(7) is not None


def test_go_to_checks_only_that_pose():
    ctrl, seq = enabled(slew=0.0)
    seq.poses = [pose("far", 25.0, 0.0), pose("near", 1.0, 0.0)]
    assert seq.go_to(1) is None
    seq.stop()
    assert seq.go_to(0) is not None


def test_list_is_locked_while_playing(tmp_path):
    ctrl, seq = enabled(slew=0.0)
    seq.poses = [pose("a", 1.0, 0.0, dwell=10.0)]
    seq.play()
    for refused in (
        seq.save_pose(),
        seq.update_pose(0),
        seq.delete_pose(0),
        seq.move_pose(0, 1),
        seq.rename_pose(0, "x"),
        seq.set_dwell(0, 1.0),
        seq.go_to(0),
        seq.load_file(tmp_path / "x.yaml"),
        seq.save_file(tmp_path / "x.yaml"),
    ):
        assert refused == "replay in progress"
    assert seq.playing
    assert seq.poses[0].name == "a"


# ── files ───────────────────────────────────────────────────────────────


def test_file_round_trip(tmp_path):
    ctrl, seq = enabled()
    ctrl.set_target(0, 1.5)
    seq.save_pose()
    seq.set_dwell(0, 0.5)
    seq.rename_pose(0, "open")
    ctrl.set_target(1, -3.0)
    seq.save_pose()
    path = tmp_path / "right.yaml"
    assert seq.save_file(path) is None
    assert not seq.dirty
    assert seq.file_path == path
    text = path.read_text()
    assert text.index("j0") < text.index("j1")  # joint order, not sorted
    assert yaml.safe_load(text) == {
        "hand": "right",
        "poses": [
            {"name": "open", "dwell_s": 0.5, "joints_deg": {"j0": 1.5, "j1": 0.0}},
            {"name": "pose 2", "dwell_s": 1.0, "joints_deg": {"j0": 1.5, "j1": -3.0}},
        ],
    }

    ctrl2, seq2 = ready()
    seq2.save_pose()
    assert seq2.load_file(path) is None
    assert seq2.poses == seq.poses
    assert not seq2.dirty
    assert seq2.file_path == path
    assert "2 poses" in seq2.message


def test_empty_list_round_trip(tmp_path):
    ctrl, seq = ready()
    path = tmp_path / "empty.yaml"
    assert seq.save_file(path) is None
    assert seq.load_file(path) is None
    assert seq.poses == []
    assert "no poses" in seq.play()


def test_load_needs_the_joint_names(tmp_path):
    ctrl, seq = make()
    path = tmp_path / "right.yaml"
    path.write_text("hand: right\nposes: []\n")
    assert "joint names unknown" in seq.load_file(path)
    assert "joint names unknown" in seq.save_file(path)


def test_load_failures_keep_the_list(tmp_path):
    ctrl, seq = ready()
    seq.save_pose()
    before = list(seq.poses)
    assert "cannot read" in seq.load_file(tmp_path / "missing.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("hand: [\n")
    assert "not valid YAML" in seq.load_file(bad)
    wrong = tmp_path / "left.yaml"
    wrong.write_text("hand: left\nposes: []\n")
    reason = seq.load_file(wrong)
    assert reason.startswith("left.yaml:") and "'left'" in reason
    assert seq.poses == before
    assert seq.dirty


def test_save_failure_is_reported(tmp_path):
    ctrl, seq = ready()
    assert "cannot write" in seq.save_file(tmp_path / "nodir" / "x.yaml")
    assert seq.file_path is None


def _file(**overrides):
    data = {
        "hand": "right",
        "poses": [{"name": "a", "dwell_s": 1.0, "joints_deg": {"j0": 1.0, "j1": 2.0}}],
    }
    data.update(overrides)
    return data


def _pose(**overrides):
    entry = {"name": "a", "dwell_s": 1.0, "joints_deg": {"j0": 1.0, "j1": 2.0}}
    entry.update(overrides)
    return _file(poses=[entry])


@pytest.mark.parametrize(
    ("data", "needle"),
    [
        ([], "mapping"),
        (None, "mapping"),
        (_file(extra=1), "unknown keys"),
        (_file(hand="left"), "'left'"),
        ({"poses": []}, "None"),
        (_file(poses={}), "must be a list"),
        (_file(poses=[1]), "must be a mapping"),
        (_pose(bogus=1), "unknown keys"),
        (_pose(name=3), "name"),
        (_pose(name=" "), "name"),
        (_pose(dwell_s="1"), "dwell_s"),
        (_pose(dwell_s=True), "dwell_s"),
        (_pose(dwell_s=-0.1), "dwell_s"),
        (_pose(dwell_s=MAX_DWELL_S + 0.01), "dwell_s"),
        (_pose(dwell_s=float("nan")), "dwell_s"),
        (_pose(joints_deg=[1.0, 2.0]), "joints_deg"),
        (_pose(joints_deg={"j0": 1.0}), "missing ['j1']"),
        (_pose(joints_deg={"j0": 1.0, "j1": 2.0, "j9": 0.0}), "unknown ['j9']"),
        (_pose(joints_deg={"j0": "1", "j1": 2.0}), "j0"),
        (_pose(joints_deg={"j0": float("inf"), "j1": 2.0}), "j0"),
        (_pose(joints_deg={"j0": 180.01, "j1": 2.0}), "beyond"),
        (_pose(joints_deg={"j0": -181, "j1": 2.0}), "beyond"),
    ],
)
def test_pose_file_rejections(data, needle):
    with pytest.raises(PoseError) as info:
        parse_pose_file(data, "right", NAMES)
    assert needle in str(info.value)


def test_pose_file_accepts_the_edges():
    data = _pose(dwell_s=MAX_DWELL_S, joints_deg={"j0": 180, "j1": -180})
    data["poses"].append({"name": "z", "dwell_s": 0, "joints_deg": {"j1": 0, "j0": 0}})
    poses = parse_pose_file(data, "right", NAMES)
    assert poses[0].dwell_s == MAX_DWELL_S
    assert poses[0].joints_deg == {"j0": 180.0, "j1": -180.0}
    # stored in the hand's joint order whatever the file's order
    assert list(poses[1].joints_deg) == ["j0", "j1"]
    assert parse_pose_file({"hand": "right"}, "right", NAMES) == []


def test_pose_file_data_matches_parse():
    poses = [pose("a", 1.0, 2.0, dwell=0.5)]
    assert parse_pose_file(pose_file_data("right", poses), "right", NAMES) == poses
