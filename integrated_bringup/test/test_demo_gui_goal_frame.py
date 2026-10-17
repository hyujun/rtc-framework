"""Task Target panel goal frame (#779) — demo_gui.goal_frame and its app.py wiring.

A task goal names its frame in ``header.frame_id``: the arm's root_link or
``scene_world``. The world is not in TF, so the panel reads the CM's
``urdf.extra_frames.scene_world`` parameters and carries the live base-frame
pose into it. What this pins:

- which CM answers make ``world (scene)`` selectable (present, confirmed, on
  the arm root) and which do not — a provisional world is refused by the
  controllers, so offering it would only produce refused goals;
- the conversion is the inverse of the controller's (``p_root = R p_world +
  xyz``), with an asymmetric placement so R and Rᵀ cannot be confused;
- the published goal carries the selected frame, and its numbers are the
  panel's; changing the frame re-seeds the panel in the new frame.
"""

from __future__ import annotations

import math
import types

import pytest

from integrated_bringup.demo_gui import goal_frame as gf

ROOT = "base"
# Asymmetric: Rz(0.7)·Rx(0.3) and an offset on every axis.
WORLD = gf.SceneWorld(parent=ROOT, xyz=(0.1, -0.2, -0.7), rpy=(0.3, 0.0, 0.7))


# ── ParameterValue stand-ins (rcl_interfaces/msg/ParameterType values) ──────


def _pv(type_: int, **fields):
    base = {
        "bool_value": False,
        "string_value": "",
        "double_array_value": [],
        "integer_array_value": [],
    }
    base.update(fields)
    return types.SimpleNamespace(type=type_, **base)


NOT_SET = _pv(0)
# What the app hands parse_scene_world when the CM declares nothing under the
# prefix (its GetParameters would answer such a request with no values at all).
NOTHING_DECLARED = gf.answer_in_request_order([], [])


def _answer(parent=ROOT, xyz=(0.0, 0.0, -0.7), rpy=(0.0, 0.0, 0.0), provisional=False):
    return [
        _pv(4, string_value=parent),
        _pv(8, double_array_value=list(xyz)),
        _pv(8, double_array_value=list(rpy)),
        NOT_SET if provisional is None else _pv(1, bool_value=provisional),
    ]


def test_a_confirmed_world_on_the_arm_root_is_usable():
    world, why = gf.parse_scene_world(_answer(), ROOT)
    assert why == ""
    assert world == gf.SceneWorld(ROOT, (0.0, 0.0, -0.7), (0.0, 0.0, 0.0))
    assert gf.frame_labels(ROOT, world) == ("base (base)", gf.WORLD_LABEL)


def test_an_integer_list_is_read_like_the_cm_reads_it():
    values = _answer()
    values[1] = _pv(7, integer_array_value=[0, 0, -1])
    world, _ = gf.parse_scene_world(values, ROOT)
    assert world is not None and world.xyz == (0.0, 0.0, -1.0)


@pytest.mark.parametrize(
    ("values", "reason"),
    [
        (NOTHING_DECLARED, "defines no scene_world"),
        (_answer(provisional=True), "provisional"),
        (_answer(provisional=None), "provisional"),  # unset parses as provisional
        (_answer(parent="base_link"), "not the arm root"),
        (_answer(xyz=(0.0, 0.0)), "three finite numbers"),
        (_answer(rpy=(0.0, math.nan, 0.0)), "three finite numbers"),
        (_answer()[:3], "could not be read"),
    ],
)
def test_a_world_the_panel_cannot_use_is_not_offered(values, reason):
    world, why = gf.parse_scene_world(values, ROOT)
    assert world is None
    assert reason in why
    assert gf.frame_labels(ROOT, world) == ("base (base)",)


def test_an_undeclared_parameter_is_read_as_not_set():
    # The CM declares only what its files set: a scene_world written without
    # `provisional` lists three names, and the missing flag parses provisional.
    names = list(gf.SCENE_WORLD_PARAMS[:3])
    values = gf.answer_in_request_order(names, _answer()[:3])
    world, why = gf.parse_scene_world(values, ROOT)
    assert world is None and "provisional" in why
    # Listed in another order, the values still land on their names.
    order = [2, 0, 3, 1]
    values = gf.answer_in_request_order(
        [gf.SCENE_WORLD_PARAMS[i] for i in order], [_answer()[i] for i in order]
    )
    assert gf.parse_scene_world(values, ROOT)[0] is not None


def test_the_frame_id_is_the_model_frame_name():
    assert gf.frame_id_for(gf.WORLD_LABEL, ROOT) == "scene_world"
    assert gf.frame_id_for("base (base)", ROOT) == ROOT
    assert gf.frame_id_for("", ROOT) == ROOT


def test_the_world_pose_is_what_the_controller_converts_back():
    # The controller: p_root = R p_world + xyz. A world point on the x axis is
    # R's first column (rtc_tools' rpy_matrix, Rz·Ry·Rx) plus xyz in the root.
    from rtc_tools.utils.rotations import rpy_matrix

    r = rpy_matrix(WORLD.rpy)
    root = gf.world_pose_to_root([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], WORLD)
    for i in range(3):
        assert root[i] == pytest.approx(r[i][0] + WORLD.xyz[i], abs=1e-12)
    assert root[3:] == pytest.approx(list(WORLD.rpy), abs=1e-12)
    # A pose read in the root is shown in the world as the inverse.
    pose = [0.4, -0.1, 0.5, 0.2, -0.4, 1.1]
    world = gf.root_pose_to_world(pose, WORLD)
    assert gf.world_pose_to_root(world, WORLD) == pytest.approx(pose, abs=1e-12)
    # R and Rᵀ differ here, so converting the wrong way does not round-trip.
    assert gf.root_pose_to_world(pose, WORLD)[:3] != pytest.approx(
        gf.world_pose_to_root(pose, WORLD)[:3], abs=1e-3
    )


def test_the_pedestal_scene_lifts_a_root_pose_by_its_height():
    pedestal = gf.SceneWorld(ROOT, (0.0, 0.0, -0.7), (0.0, 0.0, 0.0))
    assert gf.root_pose_to_world([0.3, 0.1, 0.5, 0.0, 0.0, 0.0], pedestal) == pytest.approx(
        [0.3, 0.1, 1.2, 0.0, 0.0, 0.0], abs=1e-12
    )


# ── app.py wiring ────────────────────────────────────────────────────────────


class _FakeVar:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class _FakeEntry:
    def __init__(self, text="0.0"):
        self.text = text

    def get(self):
        return self.text

    def delete(self, *_):
        self.text = ""

    def insert(self, _index, text):
        self.text = text


class _FakeWidget:
    def __init__(self):
        self.options = {}

    def configure(self, **kw):
        self.options.update(kw)

    config = configure


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    warn = error = debug = info


TASK_CTRL = "demo_task_controller"
POSE_ROOT = [0.4, -0.1, 0.5, 0.2, -0.4, 1.1]


def _panel(world_answer=None):
    """Duck-typed GUI with the real goal-frame and send methods bound to it."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = types.SimpleNamespace()
    for name in (
        "_apply_scene_world",
        "_selected_world",
        "_task_goal_frame_id",
        "_task_pose_in_goal_frame",
        "_task_entries_pose",
        "_on_goal_frame_changed",
        "_set_task_target_entries",
        "_show_root_task_goal",
        "_task_entries_as_root_preset",
        "_publish_target",
    ):
        setattr(state, name, types.MethodType(getattr(DemoControllerGUI, name), state))
    state._profile = types.SimpleNamespace(tcp_parent=ROOT)
    state._shape = types.SimpleNamespace(arm_joint_names=["j"] * 6, hand_motor_names=[])
    state._scene_world = None
    state._scene_world_why = ""
    state._scene_world_known = False
    state._scene_world_in_flight = True
    state._scene_world_gen = 0
    state._entries_world = None
    state._goal_frame_var = _FakeVar(gf.frame_labels(ROOT, None)[0])
    state._goal_frame_combo = _FakeWidget()
    state._goal_frame_note = _FakeWidget()
    state._ee_pose_box = _FakeWidget()
    state.selected_ctrl = _FakeVar(TASK_CTRL)
    state._task_space_ready = lambda _idx: True
    state._task_pose_valid = True
    state.current_task_positions = list(POSE_ROOT)
    state._task_target_entries = [_FakeEntry() for _ in range(6)]
    state._hand_target_entries = []
    state.robot_cmd_pub = _FakePub()
    state.hand_cmd_pub = _FakePub()
    state._logger = _FakeLogger()
    state.get_logger = lambda: state._logger
    if world_answer is not None:
        state._apply_scene_world(world_answer)
    return state


def _sent_task(state):
    (msg,) = [m for m in state.robot_cmd_pub.sent if m.goal_type == "task"]
    return msg


def test_without_a_world_the_goal_goes_out_in_the_arm_root():
    state = _panel(world_answer=NOTHING_DECLARED)
    assert state._goal_frame_combo.options["values"] == ("base (base)",)
    assert "defines no scene_world" in state._goal_frame_note.options["text"]
    state._set_task_target_entries(POSE_ROOT)
    state._publish_target()
    msg = _sent_task(state)
    assert msg.header.frame_id == ROOT
    assert list(msg.task_target) == pytest.approx(POSE_ROOT, abs=1e-4)


def test_a_world_goal_carries_scene_world_and_the_panels_world_numbers():
    xyz, rpy = WORLD.xyz, WORLD.rpy
    state = _panel(world_answer=_answer(xyz=xyz, rpy=rpy))
    assert state._goal_frame_combo.options["values"] == ("base (base)", gf.WORLD_LABEL)

    state._goal_frame_var.set(gf.WORLD_LABEL)
    state._on_goal_frame_changed()  # re-seeds in the world
    assert state._ee_pose_box.options["text"].endswith("scene_world")
    state._publish_target()
    msg = _sent_task(state)
    assert msg.header.frame_id == "scene_world"
    # The seed was the live pose carried into the world; converted back the
    # controller's way it is the live pose again (entries round to 1e-4).
    back = gf.world_pose_to_root(list(msg.task_target), WORLD)
    assert back == pytest.approx(POSE_ROOT, abs=1e-3)

    # Back to the root: re-seeded with the root numbers, sent in the root.
    state._goal_frame_var.set(gf.frame_labels(ROOT, WORLD)[0])
    state._on_goal_frame_changed()
    state.robot_cmd_pub.sent.clear()
    state._publish_target()
    msg = _sent_task(state)
    assert msg.header.frame_id == ROOT
    assert list(msg.task_target) == pytest.approx(POSE_ROOT, abs=1e-4)


def test_a_world_that_goes_away_falls_back_to_the_root_and_reseeds():
    state = _panel(world_answer=_answer(xyz=WORLD.xyz, rpy=WORLD.rpy))
    state._goal_frame_var.set(gf.WORLD_LABEL)
    state._on_goal_frame_changed()
    # The CM came back without a world (a real-arm bring-up).
    state._apply_scene_world(NOTHING_DECLARED)
    assert state._goal_frame_var.get() == "base (base)"
    assert [float(e.get()) for e in state._task_target_entries[:3]] == pytest.approx(
        POSE_ROOT[:3], abs=1e-4
    )
    state._publish_target()
    assert _sent_task(state).header.frame_id == ROOT


def _entries(state):
    vals = [float(e.get()) for e in state._task_target_entries]
    return [*vals[:3], *(math.radians(v) for v in vals[3:])]


def test_without_a_live_pose_a_frame_change_converts_the_entries():
    # Nothing to re-seed from: relabelling the numbers would send them off by
    # the placement, so the numbers themselves move to the new frame.
    state = _panel(world_answer=_answer(xyz=WORLD.xyz, rpy=WORLD.rpy))
    state._set_task_target_entries(POSE_ROOT)
    state._task_pose_valid = False
    state._goal_frame_var.set(gf.WORLD_LABEL)
    state._on_goal_frame_changed()
    assert gf.world_pose_to_root(_entries(state), WORLD) == pytest.approx(POSE_ROOT, abs=1e-3)
    state._publish_target()
    msg = _sent_task(state)
    assert msg.header.frame_id == "scene_world"
    # The world goes away (CM re-read) with no live pose: back to root numbers.
    state._apply_scene_world(NOTHING_DECLARED)
    assert _entries(state) == pytest.approx(POSE_ROOT, abs=1e-3)


def test_entries_in_another_frame_are_never_sent_under_the_selected_one():
    state = _panel(world_answer=_answer(xyz=WORLD.xyz, rpy=WORLD.rpy))
    state._set_task_target_entries(POSE_ROOT)  # written in the root
    state._goal_frame_var.set(gf.WORLD_LABEL)  # relabelled without the change hook
    state._publish_target()
    assert not [m for m in state.robot_cmd_pub.sent if m.goal_type == "task"]
    assert any("withheld" in line for line in state._logger.lines)


def test_a_preset_is_shown_in_the_selected_frame_and_saved_in_the_root():
    # Presets are replayed in tcp_parent; the panel shows them in its frame,
    # and saving from a world-frame panel stores the root pose.
    state = _panel(world_answer=_answer(xyz=WORLD.xyz, rpy=WORLD.rpy))
    state._goal_frame_var.set(gf.WORLD_LABEL)
    state._show_root_task_goal(POSE_ROOT)
    assert gf.world_pose_to_root(_entries(state), WORLD) == pytest.approx(POSE_ROOT, abs=1e-3)
    saved = state._task_entries_as_root_preset()
    expected = [*POSE_ROOT[:3], *(math.degrees(v) for v in POSE_ROOT[3:])]
    assert saved == pytest.approx(expected, abs=1e-2)


def test_an_answer_sent_before_a_switch_is_dropped():
    state = _panel()
    stale_gen = state._scene_world_gen
    state._scene_world_gen += 1  # a switch while the query was in flight
    state._apply_scene_world(_answer(xyz=WORLD.xyz, rpy=WORLD.rpy), stale_gen)
    assert state._scene_world is None and not state._scene_world_known
    assert not state._scene_world_in_flight  # the next poll asks again
