"""Ball launch panel state (dynamic_catching §13 S3).

Two contracts are pinned here, and both are about the panel NOT reporting
something that did not happen:

* the panel refuses exactly what ``/sim/launch_ball_at`` refuses, so a bad field
  never reaches the wire and never comes back as an opaque service error;
* "never received" and "stale" stay distinct, because during bring-up they look
  the same on screen and mean opposite things.
"""

import math

import pytest

from integrated_bringup.demo_gui.ball_launch import (
    BALL_PREDICTION_TOPIC,
    BALL_TRUTH_TOPIC,
    BallStatus,
    FeedState,
    FeedStatus,
    parse_launch_condition,
)


def test_parses_a_well_formed_condition():
    cond = parse_launch_condition("4.81 0.04 1.75", "-4.0, 0.0, 3.5805", "0 -40 0")
    assert cond.position_m == (4.81, 0.04, 1.75)
    assert cond.velocity_m_s == (-4.0, 0.0, 3.5805)
    assert cond.spin_rad_s == (0.0, -40.0, 0.0)


def test_zero_is_a_request_not_an_omission():
    """A zero velocity is a drop and a zero spin is a spinless throw. Treating
    either as 'unset' would substitute a throw nobody asked for."""
    cond = parse_launch_condition("0 0 2", "0 0 0", "0 0 0")
    assert cond.velocity_m_s == (0.0, 0.0, 0.0)
    assert cond.spin_rad_s == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    ("position", "velocity", "spin", "expect"),
    [
        ("1 2", "0 0 0", "0 0 0", "position"),
        ("1 2 3 4", "0 0 0", "0 0 0", "position"),
        ("1 2 3", "0 l.5 0", "0 0 0", "velocity"),
        ("1 2 3", "0 0 0", "nan 0 0", "spin"),
        ("1 2 3", "inf 0 0", "0 0 0", "velocity"),
        ("1 2 3", "0 0 0", "0 0 -inf", "spin"),
    ],
)
def test_refusals_name_the_field(position, velocity, spin, expect):
    with pytest.raises(ValueError) as excinfo:
        parse_launch_condition(position, velocity, spin)
    # The field name is the whole point: "invalid input" and "velocity: 'l.5'
    # is not a number" cost the same to produce and differ by how long it takes
    # to find the typo.
    assert expect in str(excinfo.value)


def test_non_finite_is_refused_the_way_the_service_refuses_it():
    """The mirror is deliberate — a panel that accepted what the service refuses
    would report a launch the simulator never performed."""
    for token in ("nan", "inf", "-inf"):
        with pytest.raises(ValueError):
            parse_launch_condition(f"{token} 0 0", "0 0 0", "0 0 0")
    assert not math.isfinite(float("inf"))


def test_feed_starts_as_never_not_stale():
    feed = FeedStatus("ground truth")
    assert feed.state(now_s=1000.0) is FeedState.NEVER
    assert "never received" in feed.label(now_s=1000.0)
    # Not merely "not live": a feed that has never published and one that
    # stopped publishing call for different actions, and the panel has to say
    # which it is.
    assert feed.state(now_s=1e9) is not FeedState.STALE


def test_feed_goes_live_then_stale_on_the_supplied_clock():
    feed = FeedStatus("vision prediction")
    feed.mark(now_s=10.0)
    assert feed.state(now_s=10.1) is FeedState.LIVE
    assert feed.state(now_s=10.4, stale_after_s=0.5) is FeedState.LIVE
    assert feed.state(now_s=10.6, stale_after_s=0.5) is FeedState.STALE
    label = feed.label(now_s=10.6, stale_after_s=0.5)
    assert "STALE" in label and "600 ms" in label


def test_message_count_survives_going_stale():
    feed = FeedStatus("ground truth")
    for t in (1.0, 1.01, 1.02):
        feed.mark(now_s=t)
    assert feed.count == 3
    assert "3 msgs" in feed.label(now_s=5.0)


def test_status_keeps_the_services_own_refusal_text():
    status = BallStatus()
    assert status.last_accepted is None

    status.record_launch(False, "launch position must be finite")
    assert status.last_accepted is False
    assert "REFUSED" in status.last_result
    # Replacing the reason with a generic failure throws away the one thing
    # that says which field to fix.
    assert "must be finite" in status.last_result

    status.record_launch(True, "armed at p=[1 2 3]")
    assert status.last_accepted is True
    assert status.last_result.startswith("accepted: ")


def test_status_lines_cover_both_feeds_and_the_last_result():
    status = BallStatus()
    status.truth.mark(now_s=100.0)
    lines = status.lines(now_s=100.1)
    assert len(lines) == 3
    assert "live" in lines[0]
    assert "never received" in lines[1]
    assert "last launch" in lines[2]


def test_topic_names_match_the_lanes_they_watch():
    # Pinned so a rename on either side shows up here rather than as a panel
    # that quietly reports "never received" forever.
    assert BALL_TRUTH_TOPIC == "/sim/ball/ground_truth"
    assert BALL_PREDICTION_TOPIC == "/ball_perception/debug/prediction/trajectory"
