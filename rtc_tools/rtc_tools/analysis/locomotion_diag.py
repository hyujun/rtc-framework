"""Offline plots and a JSON summary of one whole-body locomotion session.

The locomotion controller writes one row per tick into a session directory:

    <session>/controllers/locomotion_controller/locomotion_diag.csv
    <session>/controllers/locomotion_controller/joint_ff.csv
    <session>/controllers/locomotion_controller/<body_group>_state.csv   (optional)

``locomotion_diag.csv`` (required) — fixed columns, then per contact ``c``
``foot_<c>_{x,y,z}``, ``a_base_*``, ``servo_norm``, ``ff_norm``, per contact
``lambda_<c>_{fx,fy,fz,tx,ty,tz}``, per joint ``v_<j>`` and ``tau_servo_<j>``.
Contact and joint names are read from the header (the ``lambda_*_fz`` and
``tau_servo_*`` columns), never assumed. Fixed columns used here:
``t_relative_s``, ``tick``, ``mode`` (0 waiting, 1 nominal, 2 degraded,
3 fault), ``fault_cause`` (0 none, 1 solve health, 2 source, 3 non-finite,
4 model epoch), ``fault_latched``, ``source_status`` (0 ok, 1 body unreadable,
2 body stale, 3 base missing, 4 base stale, 5 base invalid, 6 non-finite),
``health`` (0 nominal, 1 degraded, 2 fault), ``tau_valid``, ``qp_status``,
``qp_iterations``, ``solve_time_us``, ``wbc_tick_us``, ``eom_residual_inf``,
the five ``margin_*`` columns with ``margins_stale``, ``base_p{x,y,z}``,
``base_q{x,y,z,w}`` (quaternion order x, y, z, w), ``servo_norm``, ``ff_norm``.

``joint_ff.csv`` (optional) — ``tick`` plus per joint ``q_``, ``q_ref_``,
``g_``, ``tau_ff_``, ``qdd_ref_``, ``tau_inertia_`` blocks. The joint list is
the ``tau_ff_`` block (``q_`` is also the prefix of ``q_ref_``). Rows join the
diag rows on ``tick``; a diag tick with no joint_ff row reads as NaN there.

``<group>_state.csv`` (optional) — ``t_relative_s`` and ``effort_<j>`` per
joint, interpolated onto the diag time axis. ``--group`` picks the file; without
it a single ``*_state.csv`` in the directory is used.

``--stance-csv`` — the recorder CSV of :mod:`rtc_tools.analysis.stance_metrics`.
Its time axis starts at the first diag sample of the window (the two clocks
are different, so only elapsed time is comparable).

``--base-yaml`` + ``--group`` — a profile ``_base.yaml``; the joint names and
``max_torque`` of ``devices.<group>`` give the torque-limit lines.

Non-finite values are never dropped silently: the JSON summary counts them per
series (``n_nonfinite``) and leaves them out of the percentiles.

Pure Python plus numpy and matplotlib: no ROS.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

DIAG_FILE = "locomotion_diag.csv"
JOINT_FF_FILE = "joint_ff.csv"
SUBDIR = Path("controllers") / "locomotion_controller"
FIGURE_FILES = (
    "01_base_pose.png",
    "02_contact_forces.png",
    "03_joint_torque.png",
    "04_servo_tracking.png",
    "05_qp_timeline.png",
    "06_margins.png",
    "07_mode_timeline.png",
)
SUMMARY_FILE = "locomotion_diag_summary.json"
MARGINS = (
    "margin_friction_n",
    "margin_cop_nm",
    "margin_yaw_nm",
    "margin_torque_nm",
    "margin_joint_accel",
)
MODE_NAMES = ("waiting", "nominal", "degraded", "fault")
FAULT_CAUSE_NAMES = ("none", "solve health", "source", "non-finite", "model epoch")
HEALTH_NAMES = ("nominal", "degraded", "fault")
SOURCE_STATUS_NAMES = (
    "ok",
    "body unreadable",
    "body stale",
    "base missing",
    "base stale",
    "base invalid",
    "non-finite",
)

Table = dict[str, np.ndarray]


@dataclass
class DiagNames:
    contacts: list[str]
    joints: list[str]


@dataclass
class Session:
    diag: Table
    diag_names: DiagNames
    ff: Table | None = None  # joint_ff columns aligned to the diag rows (NaN = no row)
    ff_joints: list[str] = field(default_factory=list)
    effort: dict[str, np.ndarray] = field(default_factory=dict)  # per joint, on the diag axis


# ── loading ────────────────────────────────────────────────────────────────


def read_table(path: Path) -> tuple[list[str], Table]:
    """Header and float columns of a CSV; a cell that is not a number reads as NaN."""
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: empty file")
        rows = [r for r in reader if r]
    cols: Table = {}
    for i, name in enumerate(header):
        vals = []
        for r in rows:
            try:
                vals.append(float(r[i]))
            except (ValueError, IndexError):
                vals.append(math.nan)
        cols[name] = np.asarray(vals, dtype=float)
    return header, cols


def block_names(header: Sequence[str], prefix: str, suffix: str = "") -> list[str]:
    """Names between ``prefix`` and ``suffix`` over the header columns that carry both."""
    out = []
    for h in header:
        if h.startswith(prefix) and h.endswith(suffix) and len(h) > len(prefix) + len(suffix):
            out.append(h[len(prefix) : len(h) - len(suffix)] if suffix else h[len(prefix) :])
    return out


def parse_diag_names(header: Sequence[str]) -> DiagNames:
    """Contact names from ``lambda_<c>_fz`` and joint names from ``tau_servo_<j>``."""
    return DiagNames(
        contacts=block_names(header, "lambda_", "_fz"),
        joints=block_names(header, "tau_servo_"),
    )


def parse_ff_joints(header: Sequence[str]) -> list[str]:
    """Joint list of a joint_ff file: the ``tau_ff_`` block."""
    return block_names(header, "tau_ff_")


def locate_state_csv(directory: Path, group: str | None) -> Path | None:
    if group:
        path = directory / f"{group}_state.csv"
        return path if path.is_file() else None
    found = sorted(directory.glob("*_state.csv"))
    return found[0] if len(found) == 1 else None


def align_on_tick(diag_tick: np.ndarray, ff: Table) -> Table:
    """Joint_ff columns reindexed to the diag ticks; a tick with no row reads as NaN."""
    ff_tick = ff["tick"]
    index = {int(t): i for i, t in enumerate(ff_tick) if math.isfinite(t)}
    src = np.array([index.get(int(t), -1) if math.isfinite(t) else -1 for t in diag_tick])
    present = src >= 0
    out: Table = {}
    for name, col in ff.items():
        aligned = np.full(len(diag_tick), math.nan)
        aligned[present] = col[src[present]]
        out[name] = aligned
    out["_present"] = present.astype(float)
    return out


def window_mask(t: np.ndarray, t0: float | None, t1: float | None) -> np.ndarray:
    mask = np.ones(len(t), dtype=bool)
    if t0 is not None:
        mask &= t >= t0
    if t1 is not None:
        mask &= t <= t1
    return mask


def load_session(
    session_dir: Path, group: str | None = None, t0: float | None = None, t1: float | None = None
) -> Session:
    directory = session_dir / SUBDIR
    header, diag = read_table(directory / DIAG_FILE)
    for need in ("t_relative_s", "tick"):
        if need not in diag:
            raise ValueError(f"{DIAG_FILE}: missing column {need}")
    mask = window_mask(diag["t_relative_s"], t0, t1)
    if not mask.any():
        raise ValueError("no diag row inside the requested window")
    diag = {k: v[mask] for k, v in diag.items()}
    sess = Session(diag=diag, diag_names=parse_diag_names(header))

    ff_path = directory / JOINT_FF_FILE
    if ff_path.is_file():
        ff_header, ff = read_table(ff_path)
        if "tick" in ff:
            sess.ff = align_on_tick(diag["tick"], ff)
            sess.ff_joints = parse_ff_joints(ff_header)

    state_path = locate_state_csv(directory, group)
    if state_path is not None:
        _, state = read_table(state_path)
        ts = state.get("t_relative_s")
        if ts is not None and len(ts) > 1:
            for j in sess.ff_joints or sess.diag_names.joints:
                col = state.get(f"effort_{j}")
                if col is not None:
                    sess.effort[j] = np.interp(diag["t_relative_s"], ts, col)
    return sess


def read_torque_limits(base_yaml: Path, group: str) -> dict[str, float]:
    """``max_torque`` per joint name of ``devices.<group>`` in a profile base YAML."""
    import yaml

    doc = yaml.safe_load(base_yaml.read_text())
    params = doc["/**"]["ros__parameters"]["devices"][group]
    names = params["joint_state_names"]
    limits = params["joint_limits"]["max_torque"]
    return {str(n): float(v) for n, v in zip(names, limits, strict=False)}


# ── derived series ─────────────────────────────────────────────────────────


def tilt_rad(qx: np.ndarray, qy: np.ndarray) -> np.ndarray:
    """Angle between the body z axis and the world z axis for an (x, y, z, w) quaternion."""
    return np.arccos(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))


def base_series(diag: Table, z_ref: float | None = None) -> dict[str, np.ndarray]:
    """Height deviation, xy drift from the first sample and tilt, per row."""
    px, py, pz = diag["base_px"], diag["base_py"], diag["base_pz"]
    first = int(np.argmax(np.isfinite(pz) & np.isfinite(px) & np.isfinite(py)))
    ref = pz[first] if z_ref is None else z_ref
    return {
        "base_height_dev_m": pz - ref,
        "base_xy_drift_m": np.hypot(px - px[first], py - py[first]),
        "base_tilt_rad": tilt_rad(diag["base_qx"], diag["base_qy"]),
    }


def servo_share(diag: Table) -> np.ndarray:
    """servo_norm / ff_norm per row; NaN where ff_norm is not positive."""
    ff = diag["ff_norm"]
    out = np.full(len(ff), math.nan)
    ok = ff > 0
    out[ok] = diag["servo_norm"][ok] / ff[ok]
    return out


def margin_series(diag: Table) -> dict[str, np.ndarray]:
    """Margins with the rows the controller marked stale set to NaN."""
    stale = diag["margins_stale"] > 0.5
    out = {}
    for m in MARGINS:
        col = diag[m].copy()
        col[stale] = math.nan
        out[m] = col
    return out


def foot_slip(stance_pos: np.ndarray) -> np.ndarray:
    """xy distance of one foot from its first sample; ``stance_pos`` is (n, 2)."""
    return np.hypot(stance_pos[:, 0] - stance_pos[0, 0], stance_pos[:, 1] - stance_pos[0, 1])


# ── summary ────────────────────────────────────────────────────────────────


def series_stats(values: np.ndarray) -> dict[str, float | int | None]:
    """p50 / p99 / min / max over the finite values, with the non-finite count."""
    finite = values[np.isfinite(values)]
    out: dict[str, float | int | None] = {"n": int(finite.size)}
    out["n_nonfinite"] = int(values.size - finite.size)
    if finite.size == 0:
        out.update(p50=None, p99=None, min=None, max=None)
    else:
        out.update(
            p50=float(np.percentile(finite, 50)),
            p99=float(np.percentile(finite, 99)),
            min=float(finite.min()),
            max=float(finite.max()),
        )
    return out


def histogram(values: np.ndarray) -> dict[str, int]:
    finite = values[np.isfinite(values)]
    keys, counts = np.unique(finite.astype(int), return_counts=True)
    out = {str(int(k)): int(c) for k, c in zip(keys, counts, strict=True)}
    if finite.size != values.size:
        out["nonfinite"] = int(values.size - finite.size)
    return out


def _mode_counts(mode: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return np.unique(mode[np.isfinite(mode)], return_counts=True)


def summarize(diag: Table, z_ref: float | None = None) -> dict:
    base = base_series(diag, z_ref)
    series: dict[str, np.ndarray] = {
        "base_height_dev_abs_m": np.abs(base["base_height_dev_m"]),
        "base_xy_drift_m": base["base_xy_drift_m"],
        "base_tilt_rad": base["base_tilt_rad"],
        "solve_time_us": diag["solve_time_us"],
        "wbc_tick_us": diag["wbc_tick_us"],
        "eom_residual_inf": diag["eom_residual_inf"],
        "servo_share": servo_share(diag),
    }
    series.update(margin_series(diag))
    tick = diag["tick"]
    finite_tick = tick[np.isfinite(tick)]
    gaps = np.diff(finite_tick)
    latched = diag["fault_latched"] > 0.5
    return {
        "rows": int(len(tick)),
        "t_first_s": float(diag["t_relative_s"][0]),
        "t_last_s": float(diag["t_relative_s"][-1]),
        "rows_per_mode": {
            MODE_NAMES[int(m)] if 0 <= int(m) < len(MODE_NAMES) else str(int(m)): int(c)
            for m, c in zip(*_mode_counts(diag["mode"]), strict=True)
        },
        "tau_valid_false_rows": int(np.sum(diag["tau_valid"] < 0.5)),
        "qp_status_histogram": histogram(diag["qp_status"]),
        "qp_status_nonzero_rows": int(np.sum(diag["qp_status"] != 0)),
        "fault_latches": int(np.sum(latched[1:] & ~latched[:-1])),
        "fault_latched_rows": int(latched.sum()),
        "source_not_ok_rows": int(np.sum(diag["source_status"] != 0)),
        "margins_stale_rows": int(np.sum(diag["margins_stale"] > 0.5)),
        "max_tick_gap": float(gaps.max()) if gaps.size else 0.0,
        "missing_ticks": int(np.sum(gaps[gaps > 1] - 1)) if gaps.size else 0,
        "series": {name: series_stats(v) for name, v in series.items()},
    }


# ── figures ────────────────────────────────────────────────────────────────


def _grid(n: int, ncols: int = 2):
    import matplotlib.pyplot as plt

    nrows = max(1, math.ceil(n / ncols))
    fig, axes = plt.subplots(
        nrows, ncols, figsize=(6.5 * ncols, 2.4 * nrows + 1), sharex=True, squeeze=False
    )
    return fig, axes.ravel()


def _note(fig, text: str) -> None:
    fig.text(0.5, 0.5, text, ha="center", va="center")


def _save(fig, path: Path) -> None:
    import matplotlib.pyplot as plt

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_base(sess: Session, z_ref: float | None, path: Path) -> None:
    import matplotlib.pyplot as plt

    t = sess.diag["t_relative_s"]
    base = base_series(sess.diag, z_ref)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    label = "height - z_ref [m]" if z_ref is not None else "height - first sample [m]"
    axes[0].plot(t, base["base_height_dev_m"])
    axes[0].set_ylabel(label)
    axes[1].plot(t, base["base_xy_drift_m"])
    axes[1].set_ylabel("xy drift [m]")
    axes[2].plot(t, base["base_tilt_rad"])
    axes[2].set_ylabel("tilt [rad]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("Base pose")
    _save(fig, path)


def plot_contacts(sess: Session, stance: list | None, path: Path) -> None:
    import matplotlib.pyplot as plt

    t = sess.diag["t_relative_s"]
    nrows = 2 if stance else 1
    fig, axes = plt.subplots(nrows, 1, figsize=(10, 4 * nrows), sharex=True, squeeze=False)
    ax = axes[0, 0]
    for c in sess.diag_names.contacts:
        ax.plot(t, sess.diag[f"lambda_{c}_fz"], label=f"QP {c}")
    if stance:
        ts = np.array([s.t for s in stance]) - stance[0].t + t[0]
        for i in range(len(stance[0].loads)):
            ax.plot(ts, [s.loads[i] for s in stance], "--", label=f"recorded foot{i}")
        slip_ax = axes[1, 0]
        for i in range(len(stance[0].feet)):
            xy = np.array([s.feet[i][:2] for s in stance])
            slip_ax.plot(ts, foot_slip(xy), label=f"foot{i}")
        slip_ax.set_ylabel("foot xy slip [m]")
        slip_ax.legend(fontsize=8)
        slip_ax.grid(alpha=0.3)
    ax.set_ylabel("normal force [N]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel("t [s]")
    fig.suptitle("Contact normal forces")
    _save(fig, path)


def plot_joint_torque(sess: Session, limits: dict[str, float] | None, path: Path) -> None:
    if sess.ff is None:
        fig, _ = _grid(1)
        _note(fig, f"{JOINT_FF_FILE} not found")
        _save(fig, path)
        return
    t = sess.diag["t_relative_s"]
    fig, axes = _grid(len(sess.ff_joints))
    for ax, j in zip(axes, sess.ff_joints, strict=False):
        ax.plot(t, sess.ff[f"tau_ff_{j}"], label="tau_ff")
        if j in sess.effort:
            ax.plot(t, sess.effort[j], label="measured effort", alpha=0.8)
        if limits and j in limits:
            ax.axhline(limits[j], color="r", ls=":", lw=1, label="+/- max_torque")
            ax.axhline(-limits[j], color="r", ls=":", lw=1)
        ax.set_title(j, fontsize=9)
        ax.set_ylabel("[N m]")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.suptitle("Joint torque: feedforward vs measured")
    _save(fig, path)


def plot_servo_tracking(sess: Session, path: Path) -> None:
    import matplotlib.pyplot as plt

    t = sess.diag["t_relative_s"]
    joints = sess.diag_names.joints
    fig, axes = plt.subplots(
        max(1, len(joints)),
        2,
        figsize=(13, 2.2 * max(1, len(joints)) + 1),
        sharex=True,
        squeeze=False,
    )
    for row, j in zip(axes, joints, strict=False):
        if sess.ff is not None and f"q_ref_{j}" in sess.ff and f"q_{j}" in sess.ff:
            row[0].plot(t, sess.ff[f"q_ref_{j}"] - sess.ff[f"q_{j}"])
        row[0].set_ylabel(f"{j}\nq_ref - q [rad]", fontsize=8)
        row[1].plot(t, sess.diag[f"tau_servo_{j}"])
        row[1].set_ylabel("tau_servo [N m]", fontsize=8)
        for ax in row:
            ax.grid(alpha=0.3)
    if sess.ff is None:
        fig.text(0.25, 0.97, f"{JOINT_FF_FILE} not found", ha="center")
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.suptitle("Servo tracking error and servo torque")
    _save(fig, path)


def plot_qp(sess: Session, path: Path) -> None:
    import matplotlib.pyplot as plt

    t = sess.diag["t_relative_s"]
    d = sess.diag
    fig, axes = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
    axes[0].step(t, d["qp_status"], where="post")
    axes[0].set_ylabel("qp_status")
    axes[1].step(t, d["qp_iterations"], where="post")
    axes[1].set_ylabel("qp_iterations")
    axes[2].plot(t, d["solve_time_us"])
    axes[2].set_ylabel("solve_time_us")
    axes[3].plot(t, d["wbc_tick_us"])
    axes[3].set_ylabel("wbc_tick_us")
    axes[4].step(t, d["tau_valid"], where="post")
    axes[4].set_ylabel("tau_valid")
    axes[4].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("QP timeline")
    _save(fig, path)


def plot_margins(sess: Session, path: Path) -> None:
    import matplotlib.pyplot as plt

    t = sess.diag["t_relative_s"]
    stale = sess.diag["margins_stale"] > 0.5
    fig, axes = plt.subplots(len(MARGINS), 1, figsize=(10, 2.2 * len(MARGINS) + 1), sharex=True)
    for ax, m, live in zip(axes, MARGINS, margin_series(sess.diag).values(), strict=True):
        ax.plot(t, live)
        ax.axhline(0.0, color="r", lw=1)
        ax.fill_between(
            t,
            0,
            1,
            where=stale,
            transform=ax.get_xaxis_transform(),
            color="grey",
            alpha=0.25,
            lw=0,
        )
        ax.set_ylabel(m, fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t [s]  (grey = stale rows)")
    fig.suptitle("Margins (positive = inside)")
    _save(fig, path)


def plot_modes(sess: Session, path: Path) -> None:
    import matplotlib.pyplot as plt

    t = sess.diag["t_relative_s"]
    specs = (
        ("mode", MODE_NAMES),
        ("fault_cause", FAULT_CAUSE_NAMES),
        ("health", HEALTH_NAMES),
        ("source_status", SOURCE_STATUS_NAMES),
    )
    fig, axes = plt.subplots(len(specs), 1, figsize=(10, 8), sharex=True)
    for ax, (col, names) in zip(axes, specs, strict=True):
        ax.step(t, sess.diag[col], where="post")
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names, fontsize=8)
        ax.set_ylabel(col, fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Mode, fault cause, health and source status")
    _save(fig, path)


# ── entry point ────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--out", type=Path, default=None, help="default <session>/plots/locomotion_diag"
    )
    parser.add_argument(
        "--stance-csv", type=Path, default=None, help="stance_metrics recorder CSV"
    )
    parser.add_argument("--base-yaml", type=Path, default=None, help="profile _base.yaml")
    parser.add_argument("--group", default=None, help="device group (state CSV, torque limits)")
    parser.add_argument("--t0", type=float, default=None, help="window start [s]")
    parser.add_argument("--t1", type=float, default=None, help="window end [s]")
    parser.add_argument("--z-ref", type=float, default=None, help="base height reference [m]")
    parser.add_argument(
        "--no-show", action="store_true", help="accepted for symmetry; never shows"
    )
    args = parser.parse_args(argv)

    import matplotlib

    matplotlib.use("Agg")

    try:
        sess = load_session(args.session_dir, args.group, args.t0, args.t1)
        limits = None
        if args.base_yaml is not None:
            if not args.group:
                raise ValueError("--base-yaml needs --group")
            limits = read_torque_limits(args.base_yaml, args.group)
        stance = None
        if args.stance_csv is not None:
            from rtc_tools.analysis.stance_metrics import read_csv

            stance = read_csv(args.stance_csv) or None
    except (OSError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    out = args.out or args.session_dir / "plots" / "locomotion_diag"
    out.mkdir(parents=True, exist_ok=True)
    plot_base(sess, args.z_ref, out / FIGURE_FILES[0])
    plot_contacts(sess, stance, out / FIGURE_FILES[1])
    plot_joint_torque(sess, limits, out / FIGURE_FILES[2])
    plot_servo_tracking(sess, out / FIGURE_FILES[3])
    plot_qp(sess, out / FIGURE_FILES[4])
    plot_margins(sess, out / FIGURE_FILES[5])
    plot_modes(sess, out / FIGURE_FILES[6])

    summary = summarize(sess.diag, args.z_ref)
    summary["contacts"] = sess.diag_names.contacts
    summary["joints"] = sess.diag_names.joints
    summary["joint_ff_present"] = sess.ff is not None
    if sess.ff is not None:
        summary["joint_ff_missing_rows"] = int(np.sum(sess.ff["_present"] < 0.5))
    (out / SUMMARY_FILE).write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
