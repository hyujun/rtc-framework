"""rtc_tools.analysis.weight_overlay: the generator's rules, the merge and the scale helper."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from rtc_tools.analysis import weight_overlay as wo

_REPO = Path(__file__).resolve().parents[2]
SHIPPED_MPC = _REPO / "integrated_bringup/config/g1_loco/controllers/locomotion/mpc.yaml"
FIXTURES = _REPO / "integrated_bringup/test/data/weight_overlay"

SHIPPED = """
ctl:
  sec:
    pos: [1.0, 2.0, 0.0]
    ori: [4.0, 5.0, 0.0]
    joints:
      a: 0.5
      b: 0.25
      c: 8
    vel:
      a: 2.0e-3
      b: 3.0e-3
"""
GROUPS = """
node: nodeX
controller: ctl
root: sec
groups:
  position: [pos]
  orientation: [ori]
  joint: [{map: joints, keys: all}]
  velocity: [{map: vel, keys: [a, b]}]
"""


def _write(tmp_path: Path, shipped: str = SHIPPED, groups: str = GROUPS):
    s, g = tmp_path / "shipped.yaml", tmp_path / "groups.yaml"
    s.write_text(shipped)
    g.write_text(groups)
    return s, g


def _gen(tmp_path, theta, gauge="joint", shipped=SHIPPED, groups=GROUPS, **kw):
    s, g = _write(tmp_path, shipped, groups)
    out = tmp_path / "out.yaml"
    meta = wo.generate(s, g, theta, gauge, out, **kw)
    return yaml.safe_load(out.read_text()), meta, out


def _params(doc):
    return doc["nodeX"]["ros__parameters"]


# ── A.2: shipped × 10**theta, theta = 0 writes nothing ───────────────────────


def test_a_group_at_theta_writes_shipped_times_ten_to_theta(tmp_path):
    doc, _, _ = _gen(tmp_path, {"velocity": 1.0, "position": -1.0})
    sec = _params(doc)["ctl"]["sec"]
    assert sec["vel"] == {"a": 2.0e-3 * 10.0**1.0, "b": 3.0e-3 * 10.0**1.0}
    assert sec["pos"] == [1.0 * 10.0**-1.0, 2.0 * 10.0**-1.0, 0.0]
    assert set(sec) == {"vel", "pos"}  # the theta = 0 groups write no key


def test_all_theta_zero_gives_an_empty_controller_section(tmp_path):
    doc, _, out = _gen(tmp_path, {})
    assert doc == {"nodeX": {"ros__parameters": {}}}
    doc, _, _ = _gen(tmp_path, {"position": 0.0, "orientation": 0})
    assert _params(doc) == {}


def test_a_list_with_one_scaled_element_is_written_whole(tmp_path):
    groups = GROUPS.replace("orientation: [ori]", "orientation: ['ori[0]']")
    doc, _, _ = _gen(tmp_path, {"orientation": 1.0}, groups=groups)
    assert _params(doc)["ctl"]["sec"]["ori"] == [40.0, 5.0, 0.0]


def test_write_all_writes_every_leaf_as_a_float_at_theta_zero(tmp_path):
    doc, _, out = _gen(tmp_path, {}, write_all=True)
    sec = _params(doc)["ctl"]["sec"]
    assert sec["joints"]["c"] == 8.0
    assert "8.0" in out.read_text() and "c: 8\n" not in out.read_text()
    assert sec["pos"] == [1.0, 2.0, 0.0]
    assert sec["vel"] == {"a": 2.0e-3, "b": 3.0e-3}


def test_values_are_floats_with_round_trip_digits(tmp_path):
    doc, _, out = _gen(tmp_path, {"velocity": 0.3, "position": 1.0})
    text = out.read_text()
    assert "1e-" not in text.replace("0e-", "")  # a float keeps its dot: 1.0e-05
    reread = _params(yaml.safe_load(text))["ctl"]["sec"]
    assert reread["vel"]["a"] == 2.0e-3 * 10.0**0.3  # exact, not approximate
    assert all(isinstance(v, float) for v in reread["pos"])


def test_the_overlay_has_the_params_file_structure(tmp_path):
    doc, _, _ = _gen(tmp_path, {"velocity": 1.0})
    assert list(doc) == ["nodeX"]
    assert list(doc["nodeX"]) == ["ros__parameters"]
    assert list(_params(doc)) == ["ctl"]
    assert list(_params(doc)["ctl"]) == ["sec"]  # the root path is kept in the key tree


# ── A.3: refusals ────────────────────────────────────────────────────────────


def test_a_missing_key_path_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="no key 'nope'"):
        _gen(tmp_path, {}, groups=GROUPS.replace("[pos]", "[nope]"))


def test_a_missing_joint_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="no entry 'zzz'"):
        _gen(tmp_path, {}, groups=GROUPS.replace("keys: [a, b]", "keys: [a, zzz]"))


def test_a_list_element_out_of_range_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="not an element"):
        _gen(tmp_path, {}, groups=GROUPS.replace("[pos]", "['pos[3]']"))


def test_a_leaf_in_two_groups_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="in two groups"):
        _gen(tmp_path, {}, groups=GROUPS.replace("orientation: [ori]", "orientation: [ori, pos]"))
    with pytest.raises(wo.WeightOverlayError, match="in two groups"):  # an element of a list
        _gen(tmp_path, {}, groups=GROUPS.replace("orientation: [ori]", "orientation: ['pos[1]']"))


def test_theta_on_the_gauge_group_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="gauge group 'joint' must be 0"):
        _gen(tmp_path, {"joint": 0.1})


def test_an_undefined_gauge_or_theta_group_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="undefined group 'ghost'"):
        _gen(tmp_path, {"ghost": 1.0})
    with pytest.raises(wo.WeightOverlayError, match="gauge group 'ghost' is not defined"):
        _gen(tmp_path, {}, gauge="ghost")


@pytest.mark.parametrize("bad", ["nan", ".inf", "'x'", "true", "~"])
def test_a_non_finite_or_non_numeric_shipped_leaf_is_refused(tmp_path, bad):
    # refused whether or not its group is written
    with pytest.raises(wo.WeightOverlayError, match="not a finite number"):
        _gen(tmp_path, {}, shipped=SHIPPED.replace("a: 0.5", f"a: {bad}"))


def test_a_non_finite_theta_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="not a finite number"):
        _gen(tmp_path, {"velocity": float("inf")})


def test_a_theta_that_overflows_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="non-finite"):
        _gen(tmp_path, {"velocity": 400.0})


def test_a_map_entry_without_keys_is_refused(tmp_path):
    with pytest.raises(wo.WeightOverlayError, match="is a map"):
        _gen(tmp_path, {}, groups=GROUPS.replace("[pos]", "[joints]"))


def test_the_cli_exits_non_zero_with_the_message(tmp_path, capsys):
    s, g = _write(tmp_path)
    rc = wo.main(
        ["--shipped", str(s), "--groups", str(g), "--gauge", "joint", "--theta", "joint=1"]
        + ["--out", str(tmp_path / "o.yaml")]
    )
    assert rc != 0
    assert "gauge group" in capsys.readouterr().err
    assert not (tmp_path / "o.yaml").exists()


# ── A.7: sidecar ─────────────────────────────────────────────────────────────


def test_the_sidecar_records_what_made_the_overlay(tmp_path):
    other = tmp_path / "other.yaml"
    other.write_text("nodeX:\n  ros__parameters:\n    extra: 1\n")
    _, meta, out = _gen(tmp_path, {"velocity": 0.5}, merge=[other])
    side = json.loads((tmp_path / "out.meta.json").read_text())
    assert side == meta
    assert side["theta"]["velocity"] == 0.5 and side["theta"]["position"] == 0.0
    assert side["gauge"] == "joint"
    assert side["groups_sha256"] == wo.sha256_of(tmp_path / "groups.yaml")
    assert side["shipped_sha256"] == wo.sha256_of(tmp_path / "shipped.yaml")
    assert side["shipped_path"] == str(tmp_path / "shipped.yaml")
    assert [c["path"] for c in side["components"]][1:] == [str(other)]
    assert side["components"][1]["sha256"] == wo.sha256_of(other)
    assert wo.sidecar_path(out) == tmp_path / "out.meta.json"


# ── B: merge ─────────────────────────────────────────────────────────────────


def test_merge_is_deep_by_node_and_leaf():
    a = {"n1": {"ros__parameters": {"c": {"x": 1.0, "y": [1.0, 2.0]}}}}
    b = {"n1": {"ros__parameters": {"c": {"z": 3}}}, "n2": {"ros__parameters": {"k": "v"}}}
    assert wo.merge_overlays([a, b]) == {
        "n1": {"ros__parameters": {"c": {"x": 1.0, "y": [1.0, 2.0], "z": 3}}},
        "n2": {"ros__parameters": {"k": "v"}},
    }
    assert wo.merge_overlays([a, a]) == a  # identical values are fine
    assert a["n1"]["ros__parameters"]["c"] == {"x": 1.0, "y": [1.0, 2.0]}  # inputs untouched


def test_merge_names_the_conflicting_leaf():
    a = {"n": {"ros__parameters": {"c": {"x": 1.0}}}}
    b = {"n": {"ros__parameters": {"c": {"x": 2.0}}}}
    with pytest.raises(wo.WeightOverlayError, match=r"n\.ros__parameters\.c\.x"):
        wo.merge_overlays([a, b])


def test_merge_treats_an_int_and_a_float_as_different_parameters():
    a = {"n": {"ros__parameters": {"x": 5}}}
    b = {"n": {"ros__parameters": {"x": 5.0}}}
    with pytest.raises(wo.WeightOverlayError, match=r"n\.ros__parameters\.x"):
        wo.merge_overlays([a, b])


def test_merge_refuses_a_map_against_a_value():
    a = {"n": {"ros__parameters": {"x": {"y": 1.0}}}}
    b = {"n": {"ros__parameters": {"x": 1.0}}}
    with pytest.raises(wo.WeightOverlayError, match=r"n\.ros__parameters\.x"):
        wo.merge_overlays([a, b])


def test_the_generator_merges_other_overlays_in(tmp_path):
    other = tmp_path / "other.yaml"
    other.write_text("nodeX:\n  ros__parameters:\n    ctl:\n      est: inekf\n")
    doc, _, _ = _gen(tmp_path, {"velocity": 1.0}, merge=[other])
    assert _params(doc)["ctl"]["est"] == "inekf"
    assert "sec" in _params(doc)["ctl"]
    clash = tmp_path / "clash.yaml"
    clash.write_text(
        "nodeX:\n  ros__parameters:\n    ctl:\n      sec:\n        vel:\n          a: 1.0\n"
    )
    with pytest.raises(wo.WeightOverlayError, match=r"ctl\.sec\.vel\.a"):
        _gen(tmp_path, {"velocity": 1.0}, merge=[clash])


def test_merge_only_cli_writes_the_file_and_a_sidecar(tmp_path):
    a, b, out = tmp_path / "a.yaml", tmp_path / "b.yaml", tmp_path / "m.yaml"
    a.write_text("n:\n  ros__parameters:\n    p: 1.0\n")
    b.write_text("n:\n  ros__parameters:\n    q: x\n")
    assert wo.main(["--merge", str(a), "--merge", str(b), "--out", str(out)]) == 0
    assert yaml.safe_load(out.read_text()) == {"n": {"ros__parameters": {"p": 1.0, "q": "x"}}}
    side = json.loads((tmp_path / "m.meta.json").read_text())
    assert [c["path"] for c in side["components"]] == [str(a), str(b)]


def test_merging_a_generated_overlay_keeps_its_theta_record(tmp_path):
    _, _, gen_out = _gen(tmp_path, {"velocity": 1.0})
    other = tmp_path / "other.yaml"
    other.write_text("nodeX:\n  ros__parameters:\n    ctl:\n      est: inekf\n")
    out = tmp_path / "merged.yaml"
    meta = wo.merge_overlay_files([other, gen_out], out)
    assert meta["theta"]["velocity"] == 1.0
    assert [c["path"] for c in meta["components"]] == [str(other), str(gen_out)]


def test_merge_conflict_exits_non_zero(tmp_path, capsys):
    a, b = tmp_path / "a.yaml", tmp_path / "b.yaml"
    a.write_text("n:\n  ros__parameters:\n    p: 1.0\n")
    b.write_text("n:\n  ros__parameters:\n    p: 2.0\n")
    assert wo.main(["--merge", str(a), "--merge", str(b), "--out", str(tmp_path / "m.yaml")]) == 2
    assert "n.ros__parameters.p" in capsys.readouterr().err


# ── A.8: physical scale ──────────────────────────────────────────────────────

REF = wo.RobotScale(mass_kg=10.0, leg_length_m=1.0, foot_length_m=0.2, foot_width_m=0.1)


def test_the_same_robot_scales_by_one():
    assert wo.physical_scale(REF, REF) == dict.fromkeys(wo.WEIGHT_CLASSES, 1.0)


def test_scaling_follows_the_dimensional_rule():
    big = wo.RobotScale(mass_kg=40.0, leg_length_m=4.0, foot_length_m=0.8, foot_width_m=0.4)
    s = wo.physical_scale(REF, big)
    m, length, foot = 4.0, 4.0, 4.0
    t = length**0.5
    cost = m * length
    assert s["position"] == pytest.approx(cost / length**2)
    assert s["orientation"] == pytest.approx(cost)
    assert s["joint_position"] == pytest.approx(cost)
    assert s["linear_velocity"] == pytest.approx(cost / (length / t) ** 2)
    assert s["angular_velocity"] == pytest.approx(cost * t**2)
    assert s["joint_velocity"] == pytest.approx(cost * t**2)
    assert s["force"] == pytest.approx(cost / m**2)
    assert s["torque"] == pytest.approx(cost / (m * foot) ** 2)


def test_an_equal_relative_error_costs_the_same():
    # a position weight times (relative error × leg length)**2 is the same cost on both robots
    dst = wo.RobotScale(mass_kg=30.0, leg_length_m=0.5, foot_length_m=0.1, foot_width_m=0.05)
    s = wo.physical_scale(REF, dst)
    rel = 0.01
    w_src = 7.0
    cost_src = w_src * (rel * REF.leg_length_m) ** 2
    cost_dst = w_src * s["position"] * (rel * dst.leg_length_m) ** 2
    assert cost_dst / cost_src == pytest.approx(dst.mass_kg / REF.mass_kg * dst.leg_length_m)
    # ... the same shared factor (the cost scale M·L) for a force weight
    cost_f = w_src * s["force"] * (rel * dst.mass_kg) ** 2 / (w_src * (rel * REF.mass_kg) ** 2)
    assert cost_f == pytest.approx(dst.mass_kg / REF.mass_kg * dst.leg_length_m)


@pytest.mark.parametrize("field", ["mass_kg", "leg_length_m", "foot_length_m", "foot_width_m"])
@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_a_non_positive_robot_scale_is_refused(field, bad):
    args = {**REF.__dict__, field: bad}
    with pytest.raises(wo.WeightOverlayError, match="positive and finite"):
        wo.physical_scale(REF, wo.RobotScale(**args))


# ── D: the shipped mpc.yaml ──────────────────────────────────────────────────

SHIPPED_GROUPS = FIXTURES / "mpc_groups.yaml"
GAUGE = "joint_acceleration"


def _shipped_floats(doc, leaf_paths):
    out = {}
    for path, idx in leaf_paths:
        v = doc["locomotion_controller"]["mpc"]["kinodynamic"]
        for k in path:
            v = v[k]
        out[(path, idx)] = float(v if idx is None else v[idx])
    return out


def test_shipped_theta_zero_overlay_has_an_empty_controller_section(tmp_path):
    out = tmp_path / "o.yaml"
    wo.generate(SHIPPED_MPC, SHIPPED_GROUPS, {}, GAUGE, out)
    doc = yaml.safe_load(out.read_text())
    assert doc == {"integrated_rt_controller": {"ros__parameters": {}}}


def test_shipped_write_all_equals_every_shipped_float(tmp_path):
    out = tmp_path / "o.yaml"
    wo.generate(SHIPPED_MPC, SHIPPED_GROUPS, {}, GAUGE, out, write_all=True)
    written = yaml.safe_load(out.read_text())["integrated_rt_controller"]["ros__parameters"]
    shipped = yaml.safe_load(SHIPPED_MPC.read_text())
    got = written["locomotion_controller"]["mpc"]["kinodynamic"]
    want = shipped["locomotion_controller"]["mpc"]["kinodynamic"]
    n = 0

    def walk(g, w):
        nonlocal n
        for k, gv in g.items():
            if isinstance(gv, dict):
                walk(gv, w[k])
            elif isinstance(gv, list):
                assert [type(x) for x in gv] == [float] * len(gv)
                assert gv == [float(x) for x in w[k]]
                n += len(gv)
            else:
                assert type(gv) is float and gv == float(w[k])
                n += 1

    walk(got, want)

    def count(w):
        return sum(
            count(v) if isinstance(v, dict) else len(v) if isinstance(v, list) else 1
            for v in w.values()
        )

    # every number of the weight blocks in the loaded document is written, however many it has
    blocks = ("state_weights", "terminal_weights", "input_weights", "contact_tracking")
    assert n > 0 and n == sum(count(want[b]) for b in blocks)


def test_shipped_one_group_scales_state_and_terminal_together(tmp_path):
    out = tmp_path / "o.yaml"
    wo.generate(SHIPPED_MPC, SHIPPED_GROUPS, {"joint_position": 0.25}, GAUGE, out)
    kin = yaml.safe_load(out.read_text())["integrated_rt_controller"]["ros__parameters"][
        "locomotion_controller"
    ]["mpc"]["kinodynamic"]
    ship = yaml.safe_load(SHIPPED_MPC.read_text())["locomotion_controller"]["mpc"]["kinodynamic"]
    assert set(kin) == {"state_weights", "terminal_weights"}
    for blk in kin:
        assert set(kin[blk]) == {"joints"}
        for j, v in kin[blk]["joints"].items():
            assert v == ship[blk]["joints"][j] * 10.0**0.25


@pytest.mark.parametrize("group", ["joint_position", "base_orientation"])
def test_a_group_scales_every_shipped_number_it_names(tmp_path, group):
    theta = 0.5
    out = tmp_path / "o.yaml"
    wo.generate(SHIPPED_MPC, SHIPPED_GROUPS, {group: theta}, GAUGE, out)
    kin = yaml.safe_load(out.read_text())["integrated_rt_controller"]["ros__parameters"][
        "locomotion_controller"
    ]["mpc"]["kinodynamic"]
    ship = yaml.safe_load(SHIPPED_MPC.read_text())["locomotion_controller"]["mpc"]["kinodynamic"]
    seen = 0

    def walk(g, w):
        nonlocal seen
        for k, gv in g.items():
            if isinstance(gv, dict):
                walk(gv, w[k])
            else:
                got = gv if isinstance(gv, list) else [gv]
                ref = w[k] if isinstance(gv, list) else [w[k]]
                assert got == pytest.approx([float(x) * 10.0**theta for x in ref])
                seen += len(got)

    walk(kin, ship)
    assert seen > 0


# Not a value pin: integrated_bringup's test_locomotion_mpc_weight_overlay.cpp
# reads these checked-in overlays and expects them to be what the generator
# writes from the shipped mpc.yaml (theta 0 = the shipped weights, +-0.5 = the
# shipped weights x 10^theta). This fails first, and names the file, when a
# retune leaves them stale: regenerate them with tools weight_overlay.
@pytest.mark.parametrize(
    ("name", "theta", "write_all"),
    [
        ("all_keys_theta0.yaml", {}, True),
        ("joint_position_m0p5.yaml", {"joint_position": -0.5}, False),
        ("base_orientation_p0p5.yaml", {"base_orientation": 0.5}, False),
    ],
)
def test_the_checked_in_cpp_fixtures_are_what_the_generator_writes(
    tmp_path, name, theta, write_all
):
    out = tmp_path / name
    wo.generate(SHIPPED_MPC, SHIPPED_GROUPS, theta, GAUGE, out, write_all=write_all)
    assert out.read_text() == (FIXTURES / name).read_text(), (
        f"{name} is stale against the shipped mpc.yaml: regenerate it"
    )
