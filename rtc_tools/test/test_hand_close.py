"""T_close,e2e analyser (dynamic_catching S4.2).

The oracle is a first-order closure whose crossing time is known in closed
form: with q(t) = q_pre + (q_cls - q_pre)(1 - e^{-t/tau}), rho(t) = 1 - e^{-t/tau},
so rho >= eta at t = -tau ln(1 - eta). Every timing case below recovers that
number from a synthesised CSV, which is what makes a green here evidence about
the analyser rather than about the fixture.

The shape cases are the ones a mean would pass: one lagging finger must drive
rho (it is a MINIMUM), a joint outside the caging mask must not, and a joint
whose closed pose is BELOW its preshape must count as progress rather than as
going backwards.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from rtc_tools.analysis.hand_close import (
    HandProfile,
    analyse,
    joint_progress,
    load_profile,
    main,
    quantile,
    rho,
    summarise_axis,
)

DT = 0.002
ETA = 0.9
TAU = 0.05
JOINTS = ["j0", "j1", "j2"]


def make_profile(**overrides) -> HandProfile:
    kwargs = {
        "joint_names": list(JOINTS),
        "q_pre": [0.0, 0.0, 0.0],
        "q_close": [1.0, 1.0, 1.0],
        "caging_mask": [True, True, True],
        "eta_close": ETA,
        "rho_eps": 0.02,
        "dt": DT,
    }
    kwargs.update(overrides)
    return HandProfile(**kwargs)


def analytic_t_close(tau: float = TAU, eta: float = ETA) -> float:
    return -tau * math.log(1.0 - eta)


def write_csv(
    path: Path,
    profile: HandProfile,
    trials: int = 1,
    taus: list[float] | None = None,
    close_rows: int = 400,
    open_rows: int = 200,
    drop_row: int | None = None,
) -> Path:
    """Synthesise a DeviceStateLog CSV for `trials` closure cycles.

    `taus` gives a per-joint time constant so a test can make one finger lag.
    The column names mirror the C++ header writer
    (integrated_bringup/logging/device_state_log_pod.hpp).
    """
    n = len(profile.joint_names)
    taus = taus or [TAU] * n
    header = ["t_relative_s"]
    for prefix in ("actual_pos", "actual_vel", "effort", "command", "joint_goal"):
        header += [f"{prefix}_{name}" for name in profile.joint_names]

    lines = [",".join(header)]
    t = 0.0
    row_index = 0
    q = list(profile.q_pre)
    for _ in range(trials):
        for phase, target, rows in (
            ("close", profile.q_close, close_rows),
            ("open", profile.q_pre, open_rows),
        ):
            for k in range(rows):
                if phase == "close":
                    frac = [1.0 - math.exp(-(k * DT) / taus[i]) for i in range(n)]
                    q = [
                        profile.q_pre[i] + (profile.q_close[i] - profile.q_pre[i]) * frac[i]
                        for i in range(n)
                    ]
                else:
                    # Snap back: the open phase is not measured, it only has to
                    # put the command lane back so the next trial is segmented.
                    q = list(profile.q_pre)
                row_index += 1
                t += DT
                if drop_row is not None and row_index == drop_row:
                    continue  # a dropped CSV row: time jumps, tick counting breaks
                cells = [f"{t:.9f}"]
                cells += [f"{v:.12g}" for v in q]  # actual_pos
                cells += ["0"] * n  # actual_vel
                cells += ["0"] * n  # effort
                cells += [f"{v:.12g}" for v in target]  # command
                cells += [f"{v:.12g}" for v in target]  # joint_goal
                lines.append(",".join(cells))
    path.write_text("\n".join(lines) + "\n")
    return path


# ── Timing against the closed-form oracle ───────────────────────────────────


def test_t_close_recovers_the_analytic_crossing(tmp_path):
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile)
    stats = analyse(csv, profile)

    assert len(stats.trials) == 1
    trial = stats.trials[0]
    expected = analytic_t_close()
    # The crossing is detected on a sample, so it can be late by at most one dt
    # and never early.
    assert trial.t_close_steady_s >= expected - 1e-12
    assert trial.t_close_steady_s <= expected + DT + 1e-12
    assert trial.t_close_tick_s == pytest.approx(trial.t_close_steady_s, abs=1e-9)


def test_every_trial_is_segmented(tmp_path):
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile, trials=7)
    stats = analyse(csv, profile)
    assert len(stats.trials) == 7
    expected = analytic_t_close()
    for trial in stats.trials:
        assert trial.t_close_steady_s <= expected + DT + 1e-12


def test_a_pose_that_is_neither_shipped_pose_does_not_open_a_trial(tmp_path):
    # The activation hold pose is whatever the hand was measured at when the
    # controller went active. It is not a step, but a nearest-of-two vote has
    # no way to say so — and if it happens to fall on the q_close side it opens
    # a phantom trial that never reaches eta and is counted as a timeout,
    # inflating the very exclusion count the report asks the reader to quote.
    # On the shipped ur5e_p1b profile that vote is decided by 0.107%.
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile, trials=3)

    header, *rows = csv.read_text().splitlines()
    n = len(profile.joint_names)
    # A hold pose PAST q_close: nearer to q_close than to q_pre (12 vs 27), so
    # the old two-way vote called it "close", but far outside the match band on
    # either side.
    # Partway between the two poses, on the q_close side of the midpoint: the
    # old two-way vote called it "close" (1.47 vs 0.27 squared) but it is
    # outside the match band, and its rho (0.7) is below eta so it does not
    # cage either. This is the shape of a real activation hold pose.
    hold = [0.7] * n
    hold_rows = 50
    out = [header]
    for k in range(hold_rows):
        cells = [f"{(k + 1) * DT:.9f}"]
        cells += [f"{v:.12g}" for v in hold]  # actual_pos
        cells += ["0"] * n * 2  # actual_vel, effort
        cells += [f"{v:.12g}" for v in hold]  # command
        cells += [f"{v:.12g}" for v in hold]  # joint_goal
        out.append(",".join(cells))
    for row in rows:  # shift the real run past the hold
        cells = row.split(",")
        cells[0] = f"{float(cells[0]) + hold_rows * DT:.9f}"
        out.append(",".join(cells))
    csv.write_text("\n".join(out) + "\n")

    stats = analyse(csv, profile)
    assert len(stats.trials) == 3, "the activation hold pose opened a phantom trial"
    # The damage is not only the count: a hold classified "close" runs straight
    # into the first real step, so that trial's t_cmd is the HOLD row and its
    # T_close is inflated by the whole hold.
    expected = analytic_t_close()
    assert stats.trials[0].t_close_steady_s >= expected - 1e-12
    assert stats.trials[0].t_close_steady_s <= expected + DT + 1e-12


def test_an_assumed_dt_makes_the_tick_axis_untrusted(tmp_path):
    # dt scales the tick axis AND the gap threshold that guards it, so an
    # assumed dt cannot police itself: at 1 kHz a 2 ms drop gap would be
    # compared against a 3 ms threshold and the run called clean.
    sidecar = tmp_path / "run.json"
    profile = make_profile()
    payload = {
        "joint_names": list(profile.joint_names),
        "q_pre": list(profile.q_pre),
        "q_close": list(profile.q_close),
        "caging_mask": list(profile.caging_mask),
        "eta_close": profile.eta_close,
        "rho_eps": profile.rho_eps,
    }
    sidecar.write_text(json.dumps(payload))
    loaded = load_profile(sidecar)
    assert loaded.dt_is_assumed is True

    csv = write_csv(tmp_path / "hand_state.csv", loaded)
    stats = analyse(csv, loaded)
    assert stats.tick_axis_trusted is False
    assert "ASSUMED" in stats.spacing_note

    payload["dt"] = DT
    sidecar.write_text(json.dumps(payload))
    assert load_profile(sidecar).dt_is_assumed is False


def test_a_closure_that_never_reaches_eta_is_a_nan_not_a_number(tmp_path):
    # A truncated dwell must not be reported as a fast closure.
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile, close_rows=10)
    stats = analyse(csv, profile)
    assert len(stats.trials) == 1
    assert math.isnan(stats.trials[0].t_close_steady_s)
    assert stats.trials[0].rho_final < ETA
    assert summarise_axis([t.t_close_steady_s for t in stats.trials])["n"] == 0


# ── rho is a minimum over the caging set ────────────────────────────────────


def test_rho_follows_the_slowest_caging_joint(tmp_path):
    profile = make_profile()
    slow = TAU * 3.0
    csv = write_csv(tmp_path / "hand_state.csv", profile, taus=[TAU, TAU, slow])
    stats = analyse(csv, profile)

    trial = stats.trials[0]
    assert trial.t_close_steady_s == pytest.approx(analytic_t_close(slow), abs=DT + 1e-9)
    # ... and not the fast fingers' time, which a mean would have produced.
    assert trial.t_close_steady_s > analytic_t_close(TAU) * 2


def test_a_joint_outside_the_caging_mask_does_not_hold_rho_back(tmp_path):
    profile = make_profile(caging_mask=[True, True, False])
    csv = write_csv(tmp_path / "hand_state.csv", profile, taus=[TAU, TAU, TAU * 10.0])
    stats = analyse(csv, profile)
    assert stats.trials[0].t_close_steady_s == pytest.approx(analytic_t_close(), abs=DT + 1e-9)


def test_rho_is_a_minimum_not_a_mean():
    profile = make_profile()
    # Two joints done, one barely started: caging has NOT happened.
    assert rho([1.0, 1.0, 0.1], profile) == pytest.approx(0.1)
    assert rho([0.5, 0.5, 0.5], profile) == pytest.approx(0.5)


def test_joint_progress_is_the_per_joint_primitive_rho_minimises_over():
    # A public helper (P5, shared with rtc_tools.analysis.catching_trials'
    # hand-hold-window calibration): rho() is just min() over this per joint.
    assert joint_progress(0.5, 0.0, 1.0) == pytest.approx(0.5)
    assert joint_progress(1.0, 0.0, 1.0) == pytest.approx(1.0)
    # A joint whose closed pose is BELOW its preshape closes by decreasing.
    assert joint_progress(-0.5, 0.0, -1.0) == pytest.approx(0.5)
    assert joint_progress(0.5, 0.0, -1.0) == pytest.approx(-0.5)


# ── Direction ───────────────────────────────────────────────────────────────


def test_a_joint_that_closes_by_decreasing_counts_as_progress():
    # The shipped p1b posture has joints whose closed value is below the
    # preshape; an unsigned ratio would report the whole trial as negative.
    profile = make_profile(q_pre=[0.0, 0.0, 0.0], q_close=[-1.0, -1.0, -1.0])
    assert rho([-0.5, -0.5, -0.5], profile) == pytest.approx(0.5)
    assert rho([0.5, 0.5, 0.5], profile) == pytest.approx(-0.5)


def test_mixed_direction_profile_times_the_same_as_a_positive_one(tmp_path):
    profile = make_profile(q_pre=[0.0, 0.0, 0.0], q_close=[1.0, -1.0, 1.0])
    csv = write_csv(tmp_path / "hand_state.csv", profile)
    stats = analyse(csv, profile)
    assert stats.trials[0].t_close_steady_s == pytest.approx(analytic_t_close(), abs=DT + 1e-9)


# ── The profile refuses what the C++ validator refuses ──────────────────────


def test_a_caging_joint_with_no_travel_is_refused():
    profile = make_profile(q_close=[0.0, 1.0, 1.0])
    with pytest.raises(ValueError, match="rho_eps"):
        profile.validate()


def test_an_empty_caging_set_is_refused():
    profile = make_profile(caging_mask=[False, False, False])
    with pytest.raises(ValueError, match="minimum over nothing"):
        profile.validate()


def test_a_short_array_is_refused():
    profile = make_profile(q_close=[1.0, 1.0])
    with pytest.raises(ValueError, match="q_close"):
        profile.validate()


# ── Honesty of the two axes and the tail ────────────────────────────────────


def test_a_dropped_row_makes_the_tick_axis_untrusted(tmp_path):
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile, drop_row=50)
    stats = analyse(csv, profile)
    assert not stats.tick_axis_trusted
    assert "gap" in stats.spacing_note


def test_a_clean_run_trusts_the_tick_axis(tmp_path):
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile)
    assert analyse(csv, profile).tick_axis_trusted


def test_p99_is_an_order_statistic():
    # With fewer than 100 samples the 99th percentile IS the maximum; the
    # analyser must not interpolate a value no trial produced.
    values = [float(i) for i in range(10)]
    assert quantile(values, 0.99) == 9.0
    assert quantile(values, 0.5) == 4.0
    assert math.isnan(quantile([], 0.99))


def test_a_missing_column_is_fatal(tmp_path):
    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile)
    renamed = make_profile(joint_names=["j0", "j1", "nope"])
    with pytest.raises(SystemExit, match="actual_pos_nope"):
        analyse(csv, renamed)


def test_profile_round_trips_through_the_sidecar(tmp_path):
    import json

    profile = make_profile()
    path = tmp_path / "run.json"
    path.write_text(
        json.dumps(
            {
                "joint_names": profile.joint_names,
                "q_pre": profile.q_pre,
                "q_close": profile.q_close,
                "caging_mask": profile.caging_mask,
                "eta_close": profile.eta_close,
                "rho_eps": profile.rho_eps,
                "dt": profile.dt,
            }
        )
    )
    loaded = load_profile(path)
    assert loaded.caging_indices == [0, 1, 2]
    assert loaded.eta_close == ETA


# ── CLI + plot regression ───────────────────────────────────────────────────


def test_cli_reports_both_axes_and_writes_a_plot(tmp_path, capsys):
    import json

    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile, trials=5)
    sidecar = tmp_path / "run.json"
    sidecar.write_text(
        json.dumps(
            {
                "joint_names": profile.joint_names,
                "q_pre": profile.q_pre,
                "q_close": profile.q_close,
                "caging_mask": profile.caging_mask,
                "eta_close": profile.eta_close,
                "rho_eps": profile.rho_eps,
                "dt": profile.dt,
            }
        )
    )
    plot = tmp_path / "out.png"
    rc = main([str(csv), "--profile", str(sidecar), "--plot", str(plot)])
    assert rc == 0

    out = capsys.readouterr().out
    assert "steady [ms]" in out
    assert "tick   [ms]" in out
    # The small-sample caveat must be printed, not left for the reader to know.
    assert "order statistic" in out
    assert plot.exists() and plot.stat().st_size > 0


def test_cli_refuses_a_csv_with_no_closure(tmp_path):
    import json

    profile = make_profile()
    csv = write_csv(tmp_path / "hand_state.csv", profile, trials=1)
    # Strip every closing row: the command lane never reaches the closed pose.
    lines = csv.read_text().splitlines()
    csv.write_text("\n".join([lines[0]]) + "\n")
    sidecar = tmp_path / "run.json"
    sidecar.write_text(
        json.dumps(
            {
                "joint_names": profile.joint_names,
                "q_pre": profile.q_pre,
                "q_close": profile.q_close,
                "caging_mask": profile.caging_mask,
                "eta_close": profile.eta_close,
                "rho_eps": profile.rho_eps,
                "dt": profile.dt,
            }
        )
    )
    assert main([str(csv), "--profile", str(sidecar)]) == 2
