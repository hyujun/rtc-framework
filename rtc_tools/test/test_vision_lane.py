"""Prediction-lane decoder and summaries (dynamic_catching S3.4 / D-4).

The decoder's job is to REFUSE a layout that is not the promised one, so most
of these fixtures are near-misses: the same bytes with one field moved, one
datatype changed, one field missing. A decoder that read them anyway would
produce a trajectory-shaped number from garbage, and nothing downstream would
notice.
"""

import csv
import math
import struct

import pytest

from rtc_tools.analysis.vision_lane import (
    EXPECTED_FIELDS,
    EXPECTED_POINT_STEP,
    FLOAT64,
    UINT8,
    UINT32,
    VALIDITY_VALID,
    LayoutMismatch,
    check_layout,
    decode_trajectory,
    first_flight,
    max_abs_difference,
    snapshot_row,
    summarise_ghost,
    summarise_prediction,
    summarise_reliability,
)


def encode_point(*, seq=7, gen=2, horizon_ns=50_000_000, validity=VALIDITY_VALID, x=1.0):
    buf = bytearray(EXPECTED_POINT_STEP)
    struct.pack_into("<9d", buf, 0, x, 2.0, 3.0, -4.0, 0.0, 3.5, 0.0, 0.0, -9.81)
    struct.pack_into("<36d", buf, 72, *([math.nan] * 36))
    struct.pack_into("<II", buf, 360, seq & 0xFFFFFFFF, seq >> 32)
    struct.pack_into("<II", buf, 368, gen & 0xFFFFFFFF, gen >> 32)
    struct.pack_into("<I", buf, 376, horizon_ns)
    struct.pack_into("<B", buf, 380, validity)
    return bytes(buf)


def fields():
    return list(EXPECTED_FIELDS)


def decode(data, width, flds=None, **kw):
    return decode_trajectory(
        stamp_ns=1_000,
        frame_id="world",
        fields=flds if flds is not None else fields(),
        point_step=kw.pop("point_step", EXPECTED_POINT_STEP),
        is_bigendian=kw.pop("is_bigendian", False),
        width=width,
        height=kw.pop("height", 1),
        data=data,
    )


def test_decodes_the_promised_layout_by_name():
    data = b"".join(encode_point(horizon_ns=(k + 1) * 50_000_000) for k in range(16))
    snap = decode(data, 16)
    assert snap.n_points == 16
    assert snap.points[0].horizon_ns == 50_000_000
    assert snap.points[-1].horizon_ns == 800_000_000
    assert snap.points[0].position_m == (1.0, 2.0, 3.0)
    assert snap.points[0].snapshot_sequence == 7
    assert snap.points[0].generation == 2
    assert snap.validities == {VALIDITY_VALID}
    assert all(math.isnan(c) for c in snap.points[0].covariance)


def test_uint64_fields_are_reassembled_from_the_two_uint32_halves():
    big = (3 << 32) | 5
    snap = decode(encode_point(seq=big, gen=big + 1), 1)
    assert snap.points[0].snapshot_sequence == big
    assert snap.points[0].generation == big + 1


def test_a_field_name_order_change_is_still_accepted():
    """Name-based, not order-based: D-4 says parse by name."""
    flds = list(reversed(fields()))
    snap = decode(encode_point(), 1, flds)
    assert snap.n_points == 1


@pytest.mark.parametrize(
    ("mutate", "expect"),
    [
        (lambda f: [t for t in f if t[0] != "validity"], "validity"),
        (
            lambda f: [("horizon_ns", 372, UINT32, 1) if t[0] == "horizon_ns" else t for t in f],
            "horizon_ns",
        ),
        (lambda f: [("x", 0, UINT32, 1) if t[0] == "x" else t for t in f], "'x'"),
        (
            lambda f: [("covariance", 72, FLOAT64, 9) if t[0] == "covariance" else t for t in f],
            "covariance",
        ),
        (lambda f: f + [("t", 381, UINT8, 1)], "unexpected"),
    ],
)
def test_near_miss_layouts_are_refused(mutate, expect):
    with pytest.raises(LayoutMismatch) as excinfo:
        decode(encode_point(), 1, mutate(fields()))
    assert expect in str(excinfo.value)


def test_wrong_point_step_and_endianness_are_refused():
    with pytest.raises(LayoutMismatch):
        decode(encode_point(), 1, point_step=380)
    with pytest.raises(LayoutMismatch):
        decode(encode_point(), 1, is_bigendian=True)
    assert check_layout(fields(), EXPECTED_POINT_STEP, False) is None


def test_a_truncated_buffer_is_refused_not_zero_filled():
    with pytest.raises(LayoutMismatch):
        decode(encode_point()[:-8], 1)


def test_an_invalid_snapshot_decodes_to_zero_points():
    """INVALID publishes zero points with the field table intact. That is a value
    ("no prediction"), not a malformed message."""
    snap = decode(b"", 0)
    assert snap.n_points == 0
    assert snap.validities == set()
    assert snapshot_row(0, "best_effort", snap)["validity"] == ""


def test_row_reports_mixed_validity_rather_than_picking_one():
    data = encode_point(validity=1) + encode_point(validity=0)
    row = snapshot_row(5, "reliable", decode(data, 2))
    assert row["validity"] == "MIXED"
    assert row["cov_nan"] == 1


# ── summaries ───────────────────────────────────────────────────────────────


def write_pred(path, rows):
    from rtc_tools.analysis.vision_lane import PREDICTION_COLUMNS

    with path.open("w", newline="") as h:
        w = csv.DictWriter(h, fieldnames=PREDICTION_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def pred_row(recv_ns, sub="best_effort", n=16, validity="VALID", seq=1, gen=1):
    return {
        "recv_ns": recv_ns,
        "sub": sub,
        "stamp_ns": recv_ns - 5_000_000,
        "frame_id": "world",
        "n_points": n,
        "validity": validity if n else "",
        "snapshot_sequence": seq if n else "",
        "generation": gen if n else "",
        "horizon_first_ns": 50_000_000 if n else "",
        "horizon_last_ns": 800_000_000 if n else "",
        "x0": 0.0 if n else "",
        "y0": 0.0 if n else "",
        "z0": 0.0 if n else "",
        "cov_nan": 0 if n else "",
    }


def test_rate_is_measured_only_across_non_empty_snapshots(tmp_path):
    path = tmp_path / "p_prediction.csv"
    rows = [pred_row(int(k * 33e6), seq=k) for k in range(30)]
    rows.append(pred_row(int(5e9), n=0))  # a long-delayed INVALID must not become a 5 s period
    rows.append(pred_row(int(5e9 + 33e6), seq=31))
    write_pred(path, rows)
    s = summarise_prediction(path)
    p50, _, _ = s.rate_hz()
    assert p50 == pytest.approx(30.3, abs=0.5)
    assert s.n_points[16] == 31
    assert s.horizon_last_s[0.8] == 31


def test_sequence_rewind_and_generation_change_are_counted(tmp_path):
    path = tmp_path / "p_prediction.csv"
    write_pred(
        path,
        [
            pred_row(1_000_000, seq=10, gen=1),
            pred_row(2_000_000, seq=11, gen=1),
            pred_row(3_000_000, seq=3, gen=2),  # restart: sequence rewinds, generation changes
            pred_row(4_000_000, seq=4, gen=2),
        ],
    )
    s = summarise_prediction(path)
    assert s.sequence_rewinds == 1
    assert s.generation_changes == 1


def test_reliability_compares_identity_not_count(tmp_path):
    path = tmp_path / "p_prediction.csv"
    write_pred(
        path,
        [
            pred_row(1_000, "reliable", seq=1),
            pred_row(1_000, "best_effort", seq=1),
            pred_row(2_000, "reliable", seq=2),
            pred_row(3_000, "best_effort", seq=3),  # same count, different message
        ],
    )
    r = summarise_reliability(path)
    assert r.reliable == r.best_effort == 2
    assert r.only_reliable == 1 and r.only_best_effort == 1


def test_ghost_summary_measures_valid_age_after_the_camera_stops(tmp_path):
    pred = tmp_path / "p_prediction.csv"
    cam = tmp_path / "p_camera.csv"
    # camera at 100 Hz for 0.5 s, then silence
    with cam.open("w", newline="") as h:
        w = csv.writer(h)
        w.writerow(["recv_ns", "stamp_ns", "frame_id", "x", "y", "z"])
        for k in range(50):
            w.writerow([int(k * 10e6), int(k * 10e6), "world", 0, 0, 0])
    last_cam = int(49 * 10e6)
    # predictions keep coming VALID for 120 ms after the last measurement, then go empty
    rows = [pred_row(int(t * 1e6), seq=t) for t in range(0, 620, 33)]
    rows.append(pred_row(last_cam + 150_000_000, n=0))
    write_pred(pred, rows)
    g = summarise_ghost(pred, cam)
    assert g.valid_after_loss_max_s == pytest.approx((594e6 - last_cam) * 1e-9, abs=1e-9)
    assert g.loss_to_invalid_s == pytest.approx([0.150], abs=1e-9)


def test_first_flight_is_cut_at_the_receive_gap_and_compared_over_the_overlap():
    pos = [(float(i), 0.0, 0.0) for i in range(6)]
    recv = [0, 10, 20, 30, 1_000_000_000, 1_000_000_010]
    assert first_flight(pos, recv) == pos[:4]
    n, worst = max_abs_difference(pos[:4], [(0.0, 0, 0), (1.0, 0, 0), (2.0, 0, 0), (3.5, 0, 0)])
    assert n == 4 and worst == pytest.approx(0.5)
