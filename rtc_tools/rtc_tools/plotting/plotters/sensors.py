"""Hand fingertip sensor plotters: barometer + ToF (raw / filtered / combined),
and ONNX inference output (3-head: contact / force / direction; legacy 6-head
fallback for older models).

Plotters whose name ends in `_auto` are pipeline adapters: they pull the
fingertip-label list from `df` themselves so the registry can call them with
the standard `(df, save_dir)` signature. The non-`_auto` versions still take
explicit labels for callers that already detected them.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rtc_tools.plotting.columns import (
    detect_fingertip_labels as _detect_fingertip_labels,
    detect_fingertip_labels_raw as _detect_fingertip_labels_raw,
    detect_ft_labels as _detect_ft_labels,
)
from rtc_tools.plotting.layout import auto_subplot_grid as _auto_subplot_grid


def plot_device_sensors(df, save_dir=None):
    """Figure 3: Hand sensor data (2 rows x N cols: barometer + ToF)."""
    labels = _detect_fingertip_labels(df)
    if not labels:
        print("  Skipping hand sensors plot (sensor columns not found)")
        return

    num_ft = len(labels)
    fig, axes = plt.subplots(2, num_ft, figsize=(5 * num_ft, 8))
    fig.suptitle("Device Sensor Data", fontsize=16, fontweight="bold")

    # fingertip이 1개면 axes가 1D가 되므로 2D로 보정
    if num_ft == 1:
        axes = axes.reshape(2, 1)

    t = df["timestamp"]

    # Row 1: Barometer (8ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[0, f_idx]
        for b in range(16):  # auto-stopped by column check
            col = f"baro_{label}_{b}"
            if col in df.columns:
                ax.plot(t, df[col], label=f"B{b}", alpha=0.7, linewidth=0.8)
        ax.set_title(f"{label} — Barometer")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure")
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: ToF (3ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[1, f_idx]
        for t_idx in range(8):  # auto-stopped by column check
            col = f"tof_{label}_{t_idx}"
            if col in df.columns:
                ax.plot(
                    t,
                    df[col],
                    label=f"ToF{t_idx}",
                    linewidth=1.2,
                    marker=".",
                    markersize=1,
                )
        ax.set_title(f"{label} — ToF")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_sensors.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_device_sensors_raw(df, fingertip_labels, save_dir=None):
    """Hand raw sensor data (pre-LPF): 2 rows x N cols (barometer + ToF)."""
    labels = fingertip_labels
    if not labels:
        print("  Skipping raw sensor plot (baro_raw_* columns not found)")
        return

    num_ft = len(labels)
    fig, axes = plt.subplots(2, num_ft, figsize=(5 * num_ft, 8))
    fig.suptitle("Device Sensor Data (Raw, pre-LPF)", fontsize=16, fontweight="bold")

    if num_ft == 1:
        axes = axes.reshape(2, 1)

    t = df["timestamp"]

    # Row 1: Raw Barometer (8ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[0, f_idx]
        for b in range(16):  # auto-stopped by column check
            col = f"baro_raw_{label}_{b}"
            if col in df.columns:
                ax.plot(t, df[col], label=f"B{b}", alpha=0.7, linewidth=0.8)
        ax.set_title(f"{label} — Barometer (Raw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure")
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: Raw ToF (3ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[1, f_idx]
        for t_idx in range(8):  # auto-stopped by column check
            col = f"tof_raw_{label}_{t_idx}"
            if col in df.columns:
                ax.plot(
                    t,
                    df[col],
                    label=f"ToF{t_idx}",
                    linewidth=1.2,
                    marker=".",
                    markersize=1,
                )
        ax.set_title(f"{label} — ToF (Raw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_sensors_raw.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def _plot_device_ft_legacy(df, labels, save_dir=None):
    """Legacy 6-output 모델: Force(3) + Torque(3)."""
    num_ft = len(labels)
    ncols = 2
    fig, axes = plt.subplots(num_ft, ncols, figsize=(14, 4 * num_ft), squeeze=False)
    fig.suptitle(
        "Fingertip Force/Torque (ONNX inference)", fontsize=16, fontweight="bold"
    )

    t = df["timestamp"]
    for f_idx, label in enumerate(labels):
        ax_f = axes[f_idx, 0]
        for comp in ["fx", "fy", "fz"]:
            col = f"ft_{label}_{comp}"
            if col in df.columns:
                ax_f.plot(t, df[col], label=comp.upper(), linewidth=1.2)
        ax_f.set_title(f"{label} — Force")
        ax_f.set_xlabel("Time (s)")
        ax_f.set_ylabel("Force (N)")
        ax_f.legend(fontsize=8)
        ax_f.grid(True, alpha=0.3)

        ax_t = axes[f_idx, 1]
        for comp in ["tx", "ty", "tz"]:
            col = f"ft_{label}_{comp}"
            if col in df.columns:
                ax_t.plot(t, df[col], label=comp.upper(), linewidth=1.2)
        ax_t.set_title(f"{label} — Torque")
        ax_t.set_xlabel("Time (s)")
        ax_t.set_ylabel("Torque (Nm)")
        ax_t.legend(fontsize=8)
        ax_t.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_ft_output.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_device_ft_output(df, fingertip_labels, save_dir=None):
    """Fingertip sensor + inference combined view (3-head model).

    Grid: 4 rows x N_fingertips columns:
      Row 0: Barometer (8ch)
      Row 1: Contact Probability
      Row 2: Force F (Fx, Fy, Fz)
      Row 3: Direction u (uX, uY, uZ)

    같은 column(fingertip) 내 모든 row가 x축을 공유하여
    확대 시 동기화됨.
    """
    labels = fingertip_labels
    if not labels:
        print("  Skipping F/T plot (ft_*_contact / ft_*_fx columns not found)")
        return

    num_ft = len(labels)

    # 3-head 모델 여부 감지
    has_contact = f"ft_{labels[0]}_contact" in df.columns
    if not has_contact:
        _plot_device_ft_legacy(df, labels, save_dir)
        return

    # barometer 라벨 감지 (sensor 데이터 존재 여부)
    sensor_labels = _detect_fingertip_labels(df)

    nrows = 4  # Barometer | Contact | Force | Direction
    fig, axes = plt.subplots(nrows, num_ft, figsize=(5.5 * num_ft, 14), squeeze=False)
    fig.suptitle("Fingertip Sensor + Inference Output", fontsize=16, fontweight="bold")

    # 같은 column(fingertip) 내 모든 row가 x축을 공유하여 확대 시 동기화
    for col_idx in range(num_ft):
        for row_idx in range(1, nrows):
            axes[row_idx, col_idx].sharex(axes[0, col_idx])

    t = df["timestamp"]

    for f_idx, label in enumerate(labels):
        # ── Row 0: Barometer (8ch) ──
        ax = axes[0, f_idx]
        if label in sensor_labels:
            for b in range(16):  # auto-stopped by column check
                col = f"baro_{label}_{b}"
                if col in df.columns:
                    ax.plot(t, df[col], label=f"B{b}", alpha=0.7, linewidth=0.8)
            ax.legend(fontsize=6, ncol=4)
        ax.set_title(f"{label}", fontweight="bold")
        ax.grid(True, alpha=0.3)
        if f_idx == 0:
            ax.set_ylabel("Barometer\nPressure")

        # ── Row 1: Contact probability ──
        ax = axes[1, f_idx]
        col = f"ft_{label}_contact"
        if col in df.columns:
            ax.plot(t, df[col], linewidth=1.2, color="C0")
            ax.fill_between(t, 0, df[col], alpha=0.2, color="C0")
            ax.axhline(
                y=0.1, color="red", linestyle="--", alpha=0.5, label="threshold (0.1)"
            )
        ax.set_ylim(-0.05, 1.05)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        if f_idx == 0:
            ax.set_ylabel("Contact\nProbability")

        # ── Row 2: Force vector F ──
        ax = axes[2, f_idx]
        for comp, c_label in [("fx", "Fx"), ("fy", "Fy"), ("fz", "Fz")]:
            col = f"ft_{label}_{comp}"
            if col in df.columns:
                ax.plot(t, df[col], label=c_label, linewidth=1.2)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        if f_idx == 0:
            ax.set_ylabel("Force (N)")

        # ── Row 3: Direction vector u ──
        ax = axes[3, f_idx]
        for comp, c_label in [("ux", "uX"), ("uy", "uY"), ("uz", "uZ")]:
            col = f"ft_{label}_{comp}"
            if col in df.columns:
                ax.plot(t, df[col], label=c_label, linewidth=1.2)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Time (s)")
        if f_idx == 0:
            ax.set_ylabel("Direction\nUnit vector")

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_ft_output.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_device_sensor_comparison(df, fingertip_labels, save_dir=None):
    """Sensor comparison: raw (alpha=0.3) vs filtered (alpha=1.0) overlay per fingertip."""
    labels = fingertip_labels
    if not labels:
        print("  Skipping sensor comparison plot (raw sensor columns not found)")
        return

    # filtered 라벨도 존재하는지 확인
    filtered_labels = _detect_fingertip_labels(df)
    if not filtered_labels:
        print("  Skipping sensor comparison plot (filtered sensor columns not found)")
        return

    # 두 라벨 집합의 교집합 사용
    common_labels = [l for l in labels if l in filtered_labels]
    if not common_labels:
        print("  Skipping sensor comparison plot (no matching raw/filtered labels)")
        return

    num_ft = len(common_labels)
    fig, axes = plt.subplots(2, num_ft, figsize=(5 * num_ft, 8))
    fig.suptitle("Sensor Comparison: Raw vs Filtered", fontsize=16, fontweight="bold")

    if num_ft == 1:
        axes = axes.reshape(2, 1)

    t = df["timestamp"]
    colors = plt.cm.tab10.colors

    # Row 1: Barometer comparison
    for f_idx, label in enumerate(common_labels):
        ax = axes[0, f_idx]
        for b in range(16):  # auto-stopped by column check
            color = colors[b % len(colors)]
            raw_col = f"baro_raw_{label}_{b}"
            filt_col = f"baro_{label}_{b}"
            if raw_col in df.columns:
                ax.plot(t, df[raw_col], alpha=0.3, linewidth=0.6, color=color)
            if filt_col in df.columns:
                ax.plot(
                    t,
                    df[filt_col],
                    alpha=1.0,
                    linewidth=0.8,
                    color=color,
                    label=f"B{b}",
                )
        ax.set_title(f"{label} — Barometer (solid=filtered, faint=raw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure")
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: ToF comparison
    for f_idx, label in enumerate(common_labels):
        ax = axes[1, f_idx]
        for t_idx in range(8):  # auto-stopped by column check
            color = colors[t_idx % len(colors)]
            raw_col = f"tof_raw_{label}_{t_idx}"
            filt_col = f"tof_{label}_{t_idx}"
            if raw_col in df.columns:
                ax.plot(t, df[raw_col], alpha=0.3, linewidth=0.8, color=color)
            if filt_col in df.columns:
                ax.plot(
                    t,
                    df[filt_col],
                    alpha=1.0,
                    linewidth=1.2,
                    color=color,
                    label=f"ToF{t_idx}",
                    marker=".",
                    markersize=1,
                )
        ax.set_title(f"{label} — ToF (solid=filtered, faint=raw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_sensor_comparison.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_sensor_barometer_combined(df, save_dir=None):
    """Barometer filtered + raw overlay per fingertip."""
    raw_labels = _detect_fingertip_labels_raw(df)
    filt_labels = _detect_fingertip_labels(df)
    if not raw_labels and not filt_labels:
        print("  Skipping barometer combined plot (columns not found)")
        return

    # raw/filtered 라벨 합집합 (순서 유지)
    all_labels = list(dict.fromkeys((filt_labels or []) + (raw_labels or [])))
    num_ft = len(all_labels)

    nrows, ncols = _auto_subplot_grid(num_ft)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Barometer: Filtered + Raw", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    t = df["timestamp"]
    colors = plt.cm.tab10.colors

    for f_idx, label in enumerate(all_labels):
        ax = axes[f_idx]
        for b in range(16):
            color = colors[b % len(colors)]
            raw_col = f"baro_raw_{label}_{b}"
            filt_col = f"baro_{label}_{b}"
            if raw_col in df.columns:
                ax.plot(t, df[raw_col], alpha=0.3, linewidth=0.6, color=color)
            if filt_col in df.columns:
                ax.plot(
                    t,
                    df[filt_col],
                    alpha=1.0,
                    linewidth=0.8,
                    color=color,
                    label=f"B{b}",
                )
        ax.set_title(f"{label} (solid=filtered, faint=raw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure")
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # 빈 subplot 숨기기
    for idx in range(num_ft, len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "sensor_barometer.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_sensor_tof_combined(df, save_dir=None):
    """ToF filtered + raw overlay per fingertip."""
    raw_labels = _detect_fingertip_labels_raw(df)
    filt_labels = _detect_fingertip_labels(df)
    if not raw_labels and not filt_labels:
        print("  Skipping ToF combined plot (columns not found)")
        return

    all_labels = list(dict.fromkeys((filt_labels or []) + (raw_labels or [])))
    num_ft = len(all_labels)

    nrows, ncols = _auto_subplot_grid(num_ft)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("ToF: Filtered + Raw", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    t = df["timestamp"]
    colors = plt.cm.tab10.colors

    for f_idx, label in enumerate(all_labels):
        ax = axes[f_idx]
        for t_idx in range(8):
            color = colors[t_idx % len(colors)]
            raw_col = f"tof_raw_{label}_{t_idx}"
            filt_col = f"tof_{label}_{t_idx}"
            if raw_col in df.columns:
                ax.plot(t, df[raw_col], alpha=0.3, linewidth=0.8, color=color)
            if filt_col in df.columns:
                ax.plot(
                    t,
                    df[filt_col],
                    alpha=1.0,
                    linewidth=1.2,
                    color=color,
                    label=f"ToF{t_idx}",
                    marker=".",
                    markersize=1,
                )
        ax.set_title(f"{label} (solid=filtered, faint=raw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    for idx in range(num_ft, len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "sensor_tof.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()




# ── Pipeline adapters ──────────────────────────────────────────────────────


def plot_device_ft_output_auto(df, save_dir=None):
    """Pipeline adapter: detect FT labels from df, then plot."""
    plot_device_ft_output(df, _detect_ft_labels(df), save_dir)


def plot_device_sensor_comparison_auto(df, save_dir=None):
    """Pipeline adapter: detect raw fingertip labels from df, then plot."""
    plot_device_sensor_comparison(df, _detect_fingertip_labels_raw(df), save_dir)
