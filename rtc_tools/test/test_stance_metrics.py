"""Stance metrics.

The module's whole job is to turn base + foot samples into four numbers, so
each fixture plants a KNOWN defect — one foot sliding, the base sagging, the
base rolling, a foot lifting — and checks that it, and only it, comes back. A
fixture whose answer is "some small number" would pass an implementation that
measured the wrong thing.
"""

import math

import pytest

from rtc_tools.analysis.stance_metrics import (
    Sample,
    columns,
    compute,
    from_row,
    read_csv,
    tilt_rad,
    to_row,
    write_csv,
)

DT = 0.002
Z0 = 0.76
LOAD = 160.0


def standing(n, dt=DT, feet=((0.05, 0.12, 0.001), (0.05, -0.12, 0.001)), load=LOAD):
    """A robot that does not move at all."""
    return [
        Sample(
            t=i * dt,
            pos=(0.0, 0.0, Z0),
            quat=(1.0, 0.0, 0.0, 0.0),
            feet=feet,
            loads=(load, load),
        )
        for i in range(n)
    ]


def test_a_motionless_robot_measures_zero():
    m = compute(standing(5500), settle_s=1.0, force_threshold_n=5.0)
    assert m.stance_hold_s == pytest.approx(10.998 - 1.0, abs=1e-9)
    assert m.foot_slip_max == 0.0
    assert m.base_height_dev_max == 0.0
    assert m.base_tilt_max == 0.0
    assert m.steps_without_all_feet == 0
    assert m.num_feet == 2
    assert m.num_samples == 5500 - 500
    assert m.z_ref == Z0


def test_a_sliding_foot_is_slip_of_that_foot_alone():
    rows = standing(2000)
    # Left foot drifts 3 mm in x over the second half; the right one stays.
    for i in range(1000, 2000):
        s = rows[i]
        rows[i] = Sample(
            s.t,
            s.pos,
            s.quat,
            ((0.05 + 0.003 * (i - 999) / 1000, 0.12, 0.001), s.feet[1]),
            s.loads,
        )
    m = compute(rows, settle_s=1.0, force_threshold_n=5.0)
    assert m.foot_slip_max == pytest.approx(0.003, abs=1e-9)
    assert m.base_height_dev_max == 0.0 and m.base_tilt_max == 0.0


def test_the_settle_window_hides_a_transient_before_it():
    rows = standing(2000)
    # A 2 cm sag and a 1 cm foot shift in the first 0.5 s, then still — at the
    # shifted place: the slip reference is the window's FIRST sample.
    for i in range(250):
        s = rows[i]
        rows[i] = Sample(
            s.t, (0.0, 0.0, Z0 - 0.02), s.quat, ((0.04, 0.12, 0.001), s.feet[1]), s.loads
        )
    m = compute(rows, settle_s=1.0, force_threshold_n=5.0)
    assert m.foot_slip_max == 0.0
    assert m.base_height_dev_max == 0.0


def test_height_deviation_is_against_the_given_reference():
    rows = standing(1000)
    # Standing 7 mm under the keyframe height the whole time: a window-relative
    # reference reads 0, the keyframe reference reads 7 mm.
    rows = [Sample(s.t, (0.0, 0.0, Z0 - 0.007), s.quat, s.feet, s.loads) for s in rows]
    assert compute(rows, settle_s=0.5, force_threshold_n=5.0).base_height_dev_max == 0.0
    m = compute(rows, settle_s=0.5, force_threshold_n=5.0, z_ref=Z0)
    assert m.base_height_dev_max == pytest.approx(0.007, abs=1e-12)
    assert m.z_ref == Z0


def test_tilt_is_the_angle_of_the_body_z_axis():
    # A roll of 0.1 rad about x: quat (cos(0.05), sin(0.05), 0, 0).
    q = (math.cos(0.05), math.sin(0.05), 0.0, 0.0)
    assert tilt_rad(q) == pytest.approx(0.1, abs=1e-12)
    # A yaw leaves the z axis alone.
    assert tilt_rad((math.cos(0.4), 0.0, 0.0, math.sin(0.4))) == pytest.approx(0.0, abs=1e-12)
    rows = standing(1000)
    rows[700] = Sample(rows[700].t, rows[700].pos, q, rows[700].feet, rows[700].loads)
    m = compute(rows, settle_s=0.5, force_threshold_n=5.0)
    assert m.base_tilt_max == pytest.approx(0.1, abs=1e-12)


def test_a_foot_at_or_under_the_threshold_counts_the_sample():
    rows = standing(1000)
    for i in (600, 601, 602):
        s = rows[i]
        rows[i] = Sample(s.t, s.pos, s.quat, s.feet, (LOAD, 5.0))  # exactly the threshold
    m = compute(rows, settle_s=0.5, force_threshold_n=5.0)
    assert m.steps_without_all_feet == 3
    # Under the window it is not counted.
    rows2 = standing(1000)
    rows2[10] = Sample(rows2[10].t, rows2[10].pos, rows2[10].quat, rows2[10].feet, (0.0, 0.0))
    assert compute(rows2, settle_s=0.5, force_threshold_n=5.0).steps_without_all_feet == 0


def test_sample_spacing_reports_the_period_and_the_longest_gap():
    rows = standing(1000)
    m = compute(rows, settle_s=0.5, force_threshold_n=5.0)
    assert m.sample_dt_median_s == pytest.approx(DT, abs=1e-12)
    assert m.sample_gap_max_s == pytest.approx(DT, abs=1e-12)
    # Five samples lost on the way: the period is unchanged, the gap says so.
    del rows[700:705]
    m = compute(rows, settle_s=0.5, force_threshold_n=5.0)
    assert m.sample_dt_median_s == pytest.approx(DT, abs=1e-12)
    assert m.sample_gap_max_s == pytest.approx(6 * DT, abs=1e-12)
    # A one-sample window has no spacing to report.
    one = compute(standing(2), settle_s=DT, force_threshold_n=5.0)
    assert (one.sample_dt_median_s, one.sample_gap_max_s) == (0.0, 0.0)


def test_an_empty_window_is_an_error_not_a_pass():
    with pytest.raises(ValueError, match="settle"):
        compute(standing(100), settle_s=1.0, force_threshold_n=5.0)
    with pytest.raises(ValueError, match="no samples"):
        compute([], settle_s=1.0, force_threshold_n=5.0)


def test_csv_round_trip(tmp_path):
    rows = standing(7)
    path = tmp_path / "stance.csv"
    write_csv(path, rows, 2)
    header = path.read_text().splitlines()[0].split(",")
    assert header == columns(2)
    assert header[-4:] == ["foot1_x", "foot1_y", "foot1_z", "foot1_load"]
    back = read_csv(path)
    assert back == rows
    assert from_row(dict(zip(columns(2), map(str, to_row(rows[3])), strict=True))) == rows[3]
