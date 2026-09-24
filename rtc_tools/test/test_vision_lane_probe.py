"""S8-A dump mode: full per-point rows with the whole 6x6 covariance.

``vision_lane_probe --dump`` writes these rows (``vision_lane.dump_prediction_rows``,
``vision_lane.DUMP_PREDICTION_COLUMNS``), but the decoding it depends on
(``decode_trajectory``) and the row-building itself are pure — no rclpy — so
they are tested here against a synthetic PointCloud2-like byte buffer, the same
way ``test_vision_lane.py`` tests the summary path. rclpy stays out of this
file entirely: importing ``vision_lane_probe`` would require it, so this
imports only ``vision_lane``.
"""

import math
import struct

import pytest

from rtc_tools.analysis.vision_lane import (
    DUMP_PREDICTION_COLUMNS,
    EXPECTED_FIELDS,
    EXPECTED_POINT_STEP,
    INNOVATION_COLUMNS,
    NIS_COLUMNS,
    VALIDITY_NOT_EVALUATED,
    VALIDITY_VALID,
    LayoutMismatch,
    decode_trajectory,
    dump_prediction_rows,
)


def encode_point(
    *,
    x=0.0,
    seq=7,
    gen=2,
    horizon_ns=50_000_000,
    validity=VALIDITY_VALID,
    covariance=None,
):
    """One ``point_step``-sized point, layout-exact (mirrors test_vision_lane.py)."""
    buf = bytearray(EXPECTED_POINT_STEP)
    struct.pack_into(
        "<9d", buf, 0, x, x + 1.0, x + 2.0, x + 3.0, x + 4.0, x + 5.0, x + 6.0, x + 7.0, x + 8.0
    )
    cov = covariance if covariance is not None else [math.nan] * 36
    struct.pack_into("<36d", buf, 72, *cov)
    struct.pack_into("<II", buf, 360, seq & 0xFFFFFFFF, seq >> 32)
    struct.pack_into("<II", buf, 368, gen & 0xFFFFFFFF, gen >> 32)
    struct.pack_into("<I", buf, 376, horizon_ns)
    struct.pack_into("<B", buf, 380, validity)
    return bytes(buf)


def fields():
    return list(EXPECTED_FIELDS)


def decode(data, width, flds=None, **kw):
    return decode_trajectory(
        stamp_ns=kw.pop("stamp_ns", 123_456_789),
        frame_id=kw.pop("frame_id", "world"),
        fields=flds if flds is not None else fields(),
        point_step=kw.pop("point_step", EXPECTED_POINT_STEP),
        is_bigendian=kw.pop("is_bigendian", False),
        width=width,
        height=kw.pop("height", 1),
        data=data,
    )


def test_dump_rows_carry_every_point_with_distinguishable_values():
    """Three points, each with a distinct position/velocity/acceleration and a
    covariance matrix whose entries encode their own (row, col, point index), so
    a transposition or a point mix-up would fail an exact-value assertion."""
    covs = [[100 * p + 10 * r + c for r in range(6) for c in range(6)] for p in range(3)]
    data = b"".join(
        encode_point(x=10.0 * p, seq=7, gen=2, horizon_ns=(p + 1) * 50_000_000, covariance=covs[p])
        for p in range(3)
    )
    snap = decode(data, 3)
    rows = dump_prediction_rows(recv_ns=999, sub="best_effort", snap=snap)

    assert len(rows) == 3
    for p, row in enumerate(rows):
        assert set(row) == set(DUMP_PREDICTION_COLUMNS)
        assert row["recv_ns"] == 999
        assert row["sub"] == "best_effort"
        assert row["stamp_ns"] == snap.stamp_ns
        assert row["frame_id"] == "world"
        assert row["n_points"] == 3
        assert row["point_index"] == p
        assert row["snapshot_sequence"] == 7
        assert row["generation"] == 2
        assert row["horizon_ns"] == (p + 1) * 50_000_000
        assert row["validity"] == "VALID"
        assert row["x"] == pytest.approx(10.0 * p)
        assert row["y"] == pytest.approx(10.0 * p + 1.0)
        assert row["z"] == pytest.approx(10.0 * p + 2.0)
        assert row["vx"] == pytest.approx(10.0 * p + 3.0)
        assert row["vy"] == pytest.approx(10.0 * p + 4.0)
        assert row["vz"] == pytest.approx(10.0 * p + 5.0)
        assert row["ax"] == pytest.approx(10.0 * p + 6.0)
        assert row["ay"] == pytest.approx(10.0 * p + 7.0)
        assert row["az"] == pytest.approx(10.0 * p + 8.0)
        # Row-major, matching D-4 / TrajectoryPoint.covariance — cov_{row}{col}
        # must equal the (row, col) entry of THIS point's matrix, not a
        # transposed or neighbouring point's.
        for r in range(6):
            for c in range(6):
                assert row[f"cov_{r}{c}"] == pytest.approx(100 * p + 10 * r + c)


def test_dump_rows_preserve_nan_covariance_entries():
    cov = [math.nan] * 36
    cov[2 * 6 + 3] = math.nan  # unknown cross term, explicit
    cov[0 * 6 + 0] = 1.5  # a known entry alongside unknown ones
    data = encode_point(covariance=cov)
    snap = decode(data, 1)
    row = dump_prediction_rows(0, "reliable", snap)[0]
    assert row["cov_00"] == pytest.approx(1.5)
    assert math.isnan(row["cov_23"])
    assert math.isnan(row["cov_55"])


def test_dump_rows_report_each_points_own_validity_not_an_aggregate():
    """Unlike snapshot_row's MIXED aggregate, the dump is per-point."""
    data = encode_point(validity=VALIDITY_VALID) + encode_point(validity=VALIDITY_NOT_EVALUATED)
    rows = dump_prediction_rows(0, "best_effort", decode(data, 2))
    assert [r["validity"] for r in rows] == ["VALID", "NOT_EVALUATED"]


def test_an_invalid_snapshot_yields_no_dump_rows():
    """Zero points is a value ("no prediction"), not an error — and not a row
    of blank fields that would look like a decoded point."""
    snap = decode(b"", 0)
    assert dump_prediction_rows(0, "best_effort", snap) == []


def test_a_layout_missing_a_required_field_is_refused_with_a_clear_error():
    """The dump mode decodes through the same check_layout gate as the default
    path (D-4) — it must not bypass it to get at the covariance."""
    missing_covariance = [f for f in fields() if f[0] != "covariance"]
    with pytest.raises(LayoutMismatch) as excinfo:
        decode(encode_point(), 1, missing_covariance)
    assert "covariance" in str(excinfo.value)
    assert "missing" in str(excinfo.value)


def test_dump_prediction_columns_are_unique_and_include_every_covariance_cell():
    assert len(DUMP_PREDICTION_COLUMNS) == len(set(DUMP_PREDICTION_COLUMNS))
    for r in range(6):
        for c in range(6):
            assert f"cov_{r}{c}" in DUMP_PREDICTION_COLUMNS


def test_innovation_and_nis_columns_carry_both_probe_clocks():
    """nis (std_msgs/Float64) has no header/stamp at all (C-14), so recv time
    in both clocks is the only anchor these rows have."""
    assert "recv_ns" in INNOVATION_COLUMNS and "recv_wall_ns" in INNOVATION_COLUMNS
    assert "stamp_ns" in INNOVATION_COLUMNS
    assert NIS_COLUMNS == ("recv_ns", "recv_wall_ns", "value")
