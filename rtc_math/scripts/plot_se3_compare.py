#!/usr/bin/env python3
"""Plot the SE(3) error-definition comparison experiment (se3_error_compare).

Reads the CSVs written by the `se3_error_compare` executable and renders:
  S1 — 3D position paths (SplitWorld straight line vs Log6 screw curve)
  S2 — semilogy θ(t) per definition (SplitLee π-stall)
  S4 — per-component |e_i(t)| for scalar / aniso-naive / aniso-jlog
  S5 — error-vector deviation from the guaranteed e0·exp(-kt)

Usage: plot_se3_compare.py <csv_dir> [out_dir]
Headless-safe: forces the Agg backend BEFORE importing pyplot.
"""

import csv
import os
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")  # set backend before pyplot import (headless / no DISPLAY)
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def plot_s1(csv_dir, out_dir):
    path = os.path.join(csv_dir, "s1_regulation.csv")
    if not os.path.exists(path):
        return
    rows = read_csv(path)
    by_type = defaultdict(lambda: ([], [], []))
    for r in rows:
        x, y, z = by_type[r["type"]]
        x.append(float(r["px"]))
        y.append(float(r["py"]))
        z.append(float(r["pz"]))
    try:
        # True 3D when the (sometimes shadowed) mpl_toolkits.mplot3d is healthy.
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        for t, (x, y, z) in by_type.items():
            ax.plot(x, y, z, label=t)
        ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]"), ax.set_zlabel("z [m]")
        ax.set_title("S1: TCP position path — SplitWorld (line) vs Log6 (screw)")
        ax.legend()
    except Exception:  # noqa: BLE001 — broken system mpl_toolkits: 2D fallback
        fig, (axxy, axxz) = plt.subplots(1, 2, figsize=(11, 5))
        for t, (x, y, z) in by_type.items():
            axxy.plot(x, y, label=t)
            axxz.plot(x, z, label=t)
        axxy.set_xlabel("x [m]"), axxy.set_ylabel("y [m]"), axxy.set_title("top view (xy)")
        axxz.set_xlabel("x [m]"), axxz.set_ylabel("z [m]"), axxz.set_title("side view (xz)")
        axxy.legend(), axxy.grid(alpha=0.3), axxz.grid(alpha=0.3)
        fig.suptitle("S1: TCP position path — SplitWorld (line) vs Log6 (screw)")
    fig.savefig(os.path.join(out_dir, "s1_paths.png"), dpi=120, bbox_inches="tight")
    plt.close(fig)


def plot_s2(csv_dir, out_dir):
    path = os.path.join(csv_dir, "s2_rotation.csv")
    if not os.path.exists(path):
        return
    rows = read_csv(path)
    by_type = defaultdict(lambda: ([], []))
    for r in rows:
        t, th = by_type[r["type"]]
        t.append(float(r["t"]))
        th.append(max(float(r["theta_err"]), 1e-12))
    fig, ax = plt.subplots(figsize=(7, 5))
    for typ, (t, th) in by_type.items():
        ax.semilogy(t, th, label=typ)
    ax.set_xlabel("t [s]"), ax.set_ylabel("θ error [rad]")
    ax.set_title("S2: pure-rotation convergence (θ₀=150°) — SplitLee stalls")
    ax.grid(True, which="both", alpha=0.3), ax.legend()
    fig.savefig(os.path.join(out_dir, "s2_theta.png"), dpi=120, bbox_inches="tight")
    plt.close(fig)


def plot_s4(csv_dir, out_dir):
    path = os.path.join(csv_dir, "s4_gain.csv")
    if not os.path.exists(path):
        return
    rows = read_csv(path)
    laws = ["scalar", "aniso_naive", "aniso_jlog"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    for ax, law in zip(axes, laws, strict=False):
        series = defaultdict(lambda: ([], []))
        for r in rows:
            if r["law"] != law:
                continue
            for i in range(1, 7):
                t, v = series[i]
                t.append(float(r["t"]))
                v.append(abs(float(r[f"e{i}"])))
        for i, (t, v) in series.items():
            ax.semilogy(t, [max(x, 1e-14) for x in v], label=f"e{i}")
        ax.set_title(law), ax.set_xlabel("t [s]"), ax.grid(True, which="both", alpha=0.3)
    axes[0].set_ylabel("|eᵢ|"), axes[0].legend(fontsize=8, ncol=2)
    fig.suptitle("S4: BodyLog6 per-component decay (aniso gain K_rot=8k on axes 4,5)")
    fig.savefig(os.path.join(out_dir, "s4_components.png"), dpi=120, bbox_inches="tight")
    plt.close(fig)


def plot_s5(csv_dir, out_dir):
    path = os.path.join(csv_dir, "s5_tracking.csv")
    if not os.path.exists(path):
        return
    rows = read_csv(path)
    by_law = defaultdict(lambda: ([], []))
    for r in rows:
        t, d = by_law[r["law"]]
        t.append(float(r["t"]))
        d.append(max(float(r["vec_dev"]), 1e-14))
    fig, ax = plt.subplots(figsize=(7, 5))
    for law, (t, d) in by_law.items():
        ax.semilogy(t, d, label=law)
    ax.set_xlabel("t [s]"), ax.set_ylabel("‖e(t) − e₀·e^{−kt}‖")
    ax.set_title("S5: transport map vs omission — deviation from exact convergence")
    ax.grid(True, which="both", alpha=0.3), ax.legend()
    fig.savefig(os.path.join(out_dir, "s5_tracking.png"), dpi=120, bbox_inches="tight")
    plt.close(fig)


def main():
    if len(sys.argv) < 2:
        print("usage: plot_se3_compare.py <csv_dir> [out_dir]")
        sys.exit(1)
    csv_dir = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else csv_dir
    os.makedirs(out_dir, exist_ok=True)
    plot_s1(csv_dir, out_dir)
    plot_s2(csv_dir, out_dir)
    plot_s4(csv_dir, out_dir)
    plot_s5(csv_dir, out_dir)
    print(f"plots written to {out_dir}/")


if __name__ == "__main__":
    main()
