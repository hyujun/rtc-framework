"""Clock phase analysis (D-3 / S3.1a) — plan §5.

The tool's whole job is to turn a lane CSV into per-trial numbers, so the
fixtures here inject a KNOWN defect and check it comes back. A fixture whose
answer is only "some positive number" would pass against an implementation that
measured the wrong thing.
"""

import csv

import pytest

from rtc_tools.analysis.clock_phase import quantile, read_lane, required_eps

DT = 0.002


def write_lane(path, trials, flight_steps=200, idle_steps=10, stall_s=0.0, stall_at=50):
    """A lane CSV where every flight carries exactly one stall of `stall_s`."""
    step = 0
    steady = 0.0
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["step", "sim_time_sec", "steady_ns", "launch_seq", "ball_active", "dropped_total"]
        )
        for trial in range(1, trials + 1):
            for _ in range(idle_steps):
                step += 1
                steady += DT
                writer.writerow([step, step * DT, int(steady * 1e9), trial - 1, 0, 0])
            for k in range(flight_steps):
                step += 1
                steady += DT + (stall_s if k == stall_at else 0.0)
                writer.writerow([step, step * DT, int(steady * 1e9), trial, 1, 0])
    return path


def test_recovers_an_injected_stall(tmp_path):
    csv_path = write_lane(tmp_path / "lane.csv", trials=5, stall_s=0.004)
    stats = read_lane(csv_path)

    assert len(stats.trials) == 5
    for trial in stats.trials:
        # delta accumulates and never recovers within the flight, so the single
        # 4 ms stall IS delta_max, and it is also the worst single step.
        assert trial.delta_max_s == pytest.approx(0.004, abs=1e-9)
        assert trial.max_pause_s == pytest.approx(0.004, abs=1e-9)
        assert trial.delta_signed_extreme_s > 0, "a stall means the wall clock ran ahead"


def test_a_clean_run_reports_no_phase_error(tmp_path):
    csv_path = write_lane(tmp_path / "clean.csv", trials=3, stall_s=0.0)
    stats = read_lane(csv_path)
    assert len(stats.trials) == 3
    for trial in stats.trials:
        assert trial.delta_max_s == pytest.approx(0.0, abs=1e-9)
        assert trial.max_pause_s == pytest.approx(0.0, abs=1e-9)


def test_idle_time_between_throws_is_not_counted_as_clock_error(tmp_path):
    """The window is the flight. This is the failure mode the segmentation exists for."""
    short = write_lane(tmp_path / "short_gap.csv", trials=4, idle_steps=10, stall_s=0.004)
    long = write_lane(tmp_path / "long_gap.csv", trials=4, idle_steps=2000, stall_s=0.004)

    short_deltas = [t.delta_max_s for t in read_lane(short).trials]
    long_deltas = [t.delta_max_s for t in read_lane(long).trials]

    # Two hundred times more idle time between throws, identical defect during
    # them. A tool that ran a trial to the next launch would report the gap and
    # these two would differ by orders of magnitude.
    assert short_deltas == pytest.approx(long_deltas, abs=1e-9)


def test_trials_are_cut_by_launch_even_without_a_parked_gap(tmp_path):
    """Back-to-back launches with no idle rows still segment."""
    path = tmp_path / "btb.csv"
    step = 0
    steady = 0.0
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["step", "sim_time_sec", "steady_ns", "launch_seq", "ball_active", "dropped_total"]
        )
        for trial in (1, 2):
            for _ in range(100):
                step += 1
                steady += DT
                writer.writerow([step, step * DT, int(steady * 1e9), trial, 1, 0])
    assert len(read_lane(path).trials) == 2


def test_missing_segmentation_columns_are_refused(tmp_path):
    """An older CSV cannot be segmented; averaging the whole run instead is worse
    than refusing, because the number it produces looks like an answer."""
    path = tmp_path / "old.csv"
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["step", "sim_time_sec", "steady_ns", "dropped_total"])
        writer.writerow([1, 0.002, 2_000_000, 0])
    with pytest.raises(SystemExit) as excinfo:
        read_lane(path)
    assert "launch_seq" in str(excinfo.value)


def test_drops_are_carried_through(tmp_path):
    path = tmp_path / "dropped.csv"
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["step", "sim_time_sec", "steady_ns", "launch_seq", "ball_active", "dropped_total"]
        )
        for k in range(1, 6):
            writer.writerow([k, k * DT, int(k * DT * 1e9), 1, 1, 17])
    assert read_lane(path).dropped_total == 17


def test_required_eps_takes_the_binding_half_of_the_condition():
    """Plan §5 has two inequalities; the proposal must satisfy BOTH."""
    from rtc_tools.analysis.clock_phase import Trial

    v_max, a_bound = 8.0, 12.0
    pause_bound = Trial(1, 10, 1.0, delta_max_s=0.0, delta_signed_extreme_s=0.0, max_pause_s=0.01)
    delta_bound = Trial(2, 10, 1.0, delta_max_s=0.01, delta_signed_extreme_s=0.01, max_pause_s=0.0)

    assert required_eps(pause_bound, v_max, a_bound) == pytest.approx(0.01 * v_max)
    assert required_eps(delta_bound, v_max, a_bound) == pytest.approx(
        v_max * 0.01 + 0.5 * a_bound * 0.01**2
    )


def test_quantile_is_inclusive_at_the_top():
    values = [1.0, 2.0, 3.0, 4.0]
    assert quantile(values, 1.0) == 4.0
    assert quantile(values, 0.5) == 2.0
