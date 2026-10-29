"""Pure model of the hand joint-command GUI: config, ranges, slew, safety.

Runs on a fake clock (plain floats) — no rclpy, no Qt.
"""

from __future__ import annotations

import math

import pytest

from rtc_tools.gui.joint_command_model import (
    ABS_LIMIT_DEG,
    DEFAULT_RANGE_HALF_DEG,
    MAX_PUBLISH_RATE_HZ,
    MAX_TICK_DT_S,
    MIN_PUBLISH_RATE_HZ,
    SINGLE_HAND_LABEL,
    ConfigError,
    GuiConfig,
    HandController,
    HandSample,
    hand_topics,
    load_config,
    parse_config,
)

NAMES = ("j0", "j1", "j2")
STALE_S = 0.1


def sample(deg, t, names=NAMES):
    return HandSample(
        names=tuple(names),
        position_rad=tuple(math.radians(d) for d in deg),
        effort=tuple(0.0 for _ in deg),
        received_s=t,
    )


def controller(ranges=None, slew=90.0, label="right", rate=50.0):
    cfg = GuiConfig(
        publish_rate_hz=rate,
        slew_rate_deg_s=slew,
        stale_timeout_s=STALE_S,
        ranges_deg={label: ranges} if ranges else {},
    )
    return HandController(label, cfg)


def enabled(ranges=None, slew=90.0, start=(0.0, 0.0, 0.0), t=0.0):
    c = controller(ranges, slew)
    c.on_sample(sample(start, t))
    assert c.request_enable(t) is None
    return c


# ── config ──────────────────────────────────────────────────────────────


def test_defaults():
    cfg = parse_config(None)
    assert cfg.namespace == "/hand"
    assert cfg.sides == ("right", "left")
    assert cfg.publish_rate_hz == 50.0
    assert cfg.slew_rate_deg_s == 90.0
    assert cfg.stale_timeout_s == pytest.approx(0.1)


def test_full_config(tmp_path):
    path = tmp_path / "gui.yaml"
    path.write_text(
        "namespace: /p/\n"
        "sides: [a]\n"
        "publish_rate_hz: 40\n"
        "slew_rate_deg_s: 0\n"
        "stale_timeout_ms: 250\n"
        "ranges_deg:\n"
        "  a: {j0: [-5, 5]}\n"
        "  single:\n"
    )
    cfg = load_config(path)
    assert cfg.namespace == "/p"
    assert cfg.sides == ("a",)
    assert cfg.publish_rate_hz == 40.0
    assert cfg.slew_rate_deg_s == 0.0
    assert cfg.stale_timeout_s == pytest.approx(0.25)
    assert cfg.ranges_deg == {"a": {"j0": (-5.0, 5.0)}, "single": {}}


def test_publish_rate_bounds_are_inclusive():
    assert parse_config({"publish_rate_hz": MIN_PUBLISH_RATE_HZ}).publish_rate_hz == 1.0
    assert parse_config({"publish_rate_hz": MAX_PUBLISH_RATE_HZ}).publish_rate_hz == 1000.0


def test_empty_path_is_defaults():
    assert load_config("") == GuiConfig()


@pytest.mark.parametrize(
    "data",
    [
        [],
        {"bogus": 1},
        {"namespace": "hand"},
        {"sides": "right"},
        {"sides": ["right", "right"]},
        {"sides": ["a/b"]},
        {"sides": [SINGLE_HAND_LABEL]},
        {"publish_rate_hz": 0},
        {"publish_rate_hz": MIN_PUBLISH_RATE_HZ - 0.01},
        {"publish_rate_hz": MAX_PUBLISH_RATE_HZ + 0.01},
        {"publish_rate_hz": True},
        {"slew_rate_deg_s": -1},
        {"stale_timeout_ms": 0},
        {"stale_timeout_ms": float("nan")},
        {"ranges_deg": {"nobody": {"j0": [-1, 1]}}},
        {"ranges_deg": {"right": {"j0": [1, 1]}}},
        {"ranges_deg": {"right": {"j0": [1]}}},
        {"ranges_deg": {"right": {"j0": [-181, 0]}}},
        {"ranges_deg": {"right": {"j0": [0, "x"]}}},
        {"ranges_deg": {"right": [1, 2]}},
    ],
)
def test_config_errors(data):
    with pytest.raises(ConfigError):
        parse_config(data)


def test_unreadable_and_invalid_yaml(tmp_path):
    with pytest.raises(ConfigError):
        load_config(tmp_path / "missing.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("a: [1,\n")
    with pytest.raises(ConfigError):
        load_config(bad)


def test_hand_topics():
    topics = hand_topics(GuiConfig(namespace="/p", sides=("right", "left")))
    assert [(t.label, t.state_topic, t.command_topic, t.link_topic) for t in topics] == [
        ("right", "/p/right/joint_states", "/p/right/joint_command", "/p/right/link_status"),
        ("left", "/p/left/joint_states", "/p/left/joint_command", "/p/left/link_status"),
        (SINGLE_HAND_LABEL, "/p/joint_states", "/p/joint_command", "/p/link_status"),
    ]
    root = hand_topics(GuiConfig(namespace="/", sides=()))
    assert root[0].state_topic == "/joint_states"


# ── layout lock & ranges ────────────────────────────────────────────────


def test_layout_and_default_ranges_from_first_sample():
    c = controller(ranges={"j1": (-30.0, 40.0), "nope": (-1.0, 1.0)})
    assert not c.ready
    c.on_sample(sample((5.0, 0.0, 179.0), 0.0))
    assert c.joint_names == NAMES
    assert c.ranges[0] == (5.0 - DEFAULT_RANGE_HALF_DEG, 5.0 + DEFAULT_RANGE_HALF_DEG)
    assert c.ranges[1] == (-30.0, 40.0)
    # slid inward under the cap, the measurement still inside
    assert c.ranges[2] == (ABS_LIMIT_DEG - 2 * DEFAULT_RANGE_HALF_DEG, ABS_LIMIT_DEG)
    assert c.unconfigured_joints() == ["j0", "j2"]
    assert c.unknown_yaml_joints() == ["nope"]
    assert c.target_deg == pytest.approx([5.0, 0.0, 179.0])


def test_names_change_blocks_and_disables():
    c = enabled()
    c.on_sample(sample((0.0, 0.0, 0.0), 0.01, names=("j0", "j2", "j1")))
    assert not c.enabled
    assert c.tick(0.02) is None
    assert "names changed" in c.enable_blocker(0.02)
    c.on_sample(sample((0.0, 0.0), 0.03, names=("j0", "j1")))
    assert c.enable_blocker(0.03)


def test_malformed_first_sample_names_its_cause():
    c = controller()
    bad = HandSample(names=NAMES, position_rad=(0.0,), effort=(), received_s=0.0)
    c.on_sample(bad)
    assert "names but" in c.enable_blocker(0.0)
    assert "names but" in c.status_line(0.0)


def test_status_line_shows_blocker_while_disabled():
    c = controller()
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    assert c.status_line(0.0) == "disabled"
    c.set_foreign_publisher(True)
    assert "another node" in c.status_line(0.0)
    c.set_foreign_publisher(False)
    assert c.request_enable(0.0) is None
    c.set_foreign_publisher(True)
    assert c.status_line(0.0).startswith("auto-disabled: another node")


def test_count_mismatch_inside_one_message_disables():
    c = enabled()
    bad = HandSample(names=NAMES, position_rad=(0.0, 0.0), effort=(), received_s=0.01)
    c.on_sample(bad)
    assert not c.enabled
    assert c.enable_blocker(0.01)


def test_layout_recovers_when_names_return():
    c = controller()
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    c.on_sample(sample((0.0, 0.0, 0.0), 0.01, names=("x", "y", "z")))
    assert c.enable_blocker(0.01)
    c.on_sample(sample((0.0, 0.0, 0.0), 0.02))
    assert c.enable_blocker(0.02) is None


def test_set_target_clamps_and_rejects_non_finite():
    c = controller(ranges={"j0": (-10.0, 10.0)})
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    assert c.set_target(0, 50.0) == 10.0
    assert c.set_target(0, -50.0) == -10.0
    assert c.set_target(0, math.nan) is None
    assert c.set_target(0, math.inf) is None
    assert c.target_deg[0] == -10.0


def test_set_range_rules_while_disabled():
    c = controller(ranges={"j0": (-10.0, 10.0)})
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    c.set_target(0, 8.0)
    assert c.set_range(0, 5.0, 5.0) is not None
    assert c.set_range(0, -200.0, 0.0) is not None
    assert c.set_range(0, math.nan, 1.0) is not None
    # disabled: a narrower range clamps the target instead of refusing
    assert c.set_range(0, -2.0, 2.0) is None
    assert c.target_deg[0] == 2.0
    assert c.command_deg[0] == 2.0


def test_set_range_rejected_when_it_would_cut_the_running_target():
    c = enabled(ranges={"j0": (-20.0, 20.0)})
    c.set_target(0, 15.0)
    assert c.set_range(0, -20.0, 10.0) is not None
    assert c.ranges[0] == (-20.0, 20.0)
    # the command lags the target under slew: cutting it is refused too
    c.tick(0.02)
    assert c.command_deg[0] == pytest.approx(1.8)
    c.set_target(0, -5.0)
    assert c.set_range(0, -6.0, 1.0) is not None
    assert c.set_range(0, -6.0, 2.0) is None
    assert c.enabled


def test_reset_ranges_all_or_nothing():
    c = enabled(ranges={"j0": (-10.0, 10.0)})
    assert c.set_range(0, -30.0, 30.0) is None
    for t in range(1, 30):
        c.on_sample(sample(c.command_deg, t * 0.02))
        c.set_target(0, 25.0)
        c.tick(t * 0.02)
    assert c.command_deg[0] > 10.0
    assert c.reset_ranges() is not None
    assert c.ranges[0] == (-30.0, 30.0)
    c.disable()
    assert c.reset_ranges() is None
    assert c.ranges[0] == (-10.0, 10.0)
    assert c.target_deg[0] == 10.0


# ── enable / disable ───────────────────────────────────────────────────


def test_enable_syncs_to_measurement():
    c = controller(ranges={"j0": (-50.0, 50.0)})
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    c.set_target(0, 40.0)
    c.on_sample(sample((12.0, 0.5, -0.5), 0.05))
    assert c.request_enable(0.06) is None
    assert c.target_deg == pytest.approx([12.0, 0.5, -0.5])
    out = c.tick(0.08)
    assert out == pytest.approx([math.radians(d) for d in (12.0, 0.5, -0.5)])


@pytest.mark.parametrize(
    ("setup", "needle"),
    [
        (lambda c: None, "no joint_states"),
        (lambda c: c.on_sample(sample((0.0, 0.0, 0.0), 0.0)), "stale"),
        (
            lambda c: (c.on_sample(sample((0.0, 0.0, 0.0), 1.0)), c.set_link(False)),
            "link_status",
        ),
        (
            lambda c: (c.on_sample(sample((0.0, 0.0, 0.0), 1.0)), c.set_foreign_publisher(True)),
            "another node",
        ),
        (
            lambda c: (
                c.on_sample(sample((0.0, 0.0, 0.0), 0.9)),
                c.on_sample(sample((15.0, 0.0, 0.0), 1.0)),
            ),
            "outside the range",
        ),
        (
            lambda c: (
                c.on_sample(sample((0.0, 0.0, 0.0), 0.9)),
                c.on_sample(sample((math.nan, 0.0, 0.0), 1.0)),
            ),
            "not finite",
        ),
    ],
)
def test_enable_refusals(setup, needle):
    c = controller()
    setup(c)
    reason = c.request_enable(1.0)
    assert reason is not None and needle in reason
    assert not c.enabled
    assert c.tick(1.0) is None


def test_stale_boundary_for_enable():
    c = controller()
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    assert c.enable_blocker(STALE_S) is None
    assert c.enable_blocker(STALE_S + 1e-6) is not None


def test_unknown_link_does_not_block():
    c = controller()
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    c.set_link(None)
    assert c.request_enable(0.0) is None
    c.set_link(True)
    assert c.tick(0.02) is not None


def test_disable_stops_publishing():
    c = enabled()
    assert c.tick(0.02) is not None
    c.disable()
    assert c.tick(0.04) is None
    assert c.status == "disabled"


# ── auto-disable ───────────────────────────────────────────────────────


def test_auto_disable_on_stale():
    c = enabled()
    assert c.tick(STALE_S) is not None
    assert c.tick(STALE_S + 0.001) is None
    assert not c.enabled
    assert "stale" in c.status


def test_auto_disable_on_link_down():
    c = enabled()
    c.set_link(False)
    assert not c.enabled
    assert c.tick(0.02) is None
    assert "link_status" in c.status


def test_auto_disable_on_foreign_publisher():
    c = enabled()
    c.set_foreign_publisher(True)
    assert not c.enabled
    assert c.tick(0.02) is None
    c.set_foreign_publisher(False)
    c.on_sample(sample((0.0, 0.0, 0.0), 0.03))
    assert c.request_enable(0.03) is None


# ── slew ───────────────────────────────────────────────────────────────


def test_slew_limits_the_step():
    c = enabled(ranges={"j0": (-90.0, 90.0)}, slew=90.0)
    c.set_target(0, 60.0)
    prev = 0.0
    t = 0.0
    for _ in range(50):
        t += 0.02
        c.on_sample(sample((0.0, 0.0, 0.0), t))
        out = c.tick(t)
        deg = math.degrees(out[0])
        assert deg - prev <= 90.0 * 0.02 + 1e-9
        prev = deg
    assert prev == pytest.approx(60.0)


def test_slew_dt_is_capped_after_a_stall():
    c = enabled(ranges={"j0": (-90.0, 90.0)}, slew=90.0)
    c.set_target(0, 80.0)
    c.on_sample(sample((0.0, 0.0, 0.0), 0.5))
    out = c.tick(0.5)
    assert math.degrees(out[0]) == pytest.approx(90.0 * MAX_TICK_DT_S)


def test_slew_holds_at_low_publish_rates():
    c = controller(ranges={"j0": (-90.0, 90.0)}, rate=5.0)
    c.on_sample(sample((0.0, 0.0, 0.0), 0.0))
    assert c.request_enable(0.0) is None
    c.set_target(0, 80.0)
    c.on_sample(sample((0.0, 0.0, 0.0), 0.2))
    out = c.tick(0.2)
    assert math.degrees(out[0]) == pytest.approx(90.0 * 0.2)


def test_slew_zero_is_no_limit():
    c = enabled(ranges={"j0": (-90.0, 90.0)}, slew=0.0)
    c.set_target(0, 70.0)
    out = c.tick(0.02)
    assert math.degrees(out[0]) == pytest.approx(70.0)


def test_first_tick_after_enable_holds():
    c = enabled(ranges={"j0": (-90.0, 90.0)})
    c.set_target(0, 50.0)
    out = c.tick(0.0)
    assert out[0] == pytest.approx(0.0)


def test_published_values_stay_finite_and_inside():
    c = enabled(ranges={"j0": (-5.0, 5.0)}, slew=0.0)
    for i, target in enumerate((100.0, -100.0, math.inf, math.nan, 3.0)):
        t = 0.02 * (i + 1)
        c.on_sample(sample(c.command_deg, t))
        c.set_target(0, target)
        out = c.tick(t)
        assert all(math.isfinite(v) for v in out)
        assert -5.0 <= math.degrees(out[0]) <= 5.0
