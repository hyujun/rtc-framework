#!/usr/bin/env python3
"""Record MuJoCo capsule distances as an oracle for the capsule distance core.

Writes the CSV fixtures ``rtc_urdf_bridge/test/test_capsule_mujoco_oracle.cpp``
reads. Two fixtures, two questions:

``capsule_oracle_shapes.csv``
    Random capsule pairs (including zero-length ones, i.e. spheres) placed as
    static geoms. MuJoCo's ``mj_geomDistance`` on the compiled ``geom_xpos`` /
    ``geom_xmat`` / ``geom_size``; the C++ test rebuilds each pair from those
    same world transforms, so only the shape distance convention is under
    test (MJCF ``size`` is radius + HALF length; coal's ``Capsule(r, lz)``
    takes the FULL length).

``capsule_oracle_<robot>_geoms.csv`` / ``capsule_oracle_<robot>_dist.csv``
    The capsule collision geoms of a robot MJCF (body name, local pos / quat
    / size) and, at random joint configurations, the pairwise distances after
    ``mj_forward``. The C++ test parses the same MJCF with pinocchio, attaches
    the geoms as spec capsules and compares through its own FK → ``oMg`` path.

Distances are signed (negative when penetrating) in both files; ``distmax``
is large so no row is clipped.

Deterministic (``--seed``). Needs the ``mujoco`` wheel of the workspace venv.
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys

import numpy as np

_MUJOCO_IMPORT_ERROR: str | None = None
try:
    import mujoco
except ImportError as exc:  # pragma: no cover - exercised only without the wheel
    _MUJOCO_IMPORT_ERROR = str(exc)


_DISTMAX = 100.0


def _quat_wxyz_to_mat(q: np.ndarray) -> np.ndarray:
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, q)
    return mat.reshape(3, 3)


def _random_unit_quat(rng: np.random.Generator) -> np.ndarray:
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)


def _shape_rows(n_pairs: int, rng: np.random.Generator) -> list[list[float]]:
    """Random capsule pairs; every fourth pair has a zero-length capsule."""
    rows: list[list[float]] = []
    for k in range(n_pairs):
        sizes = []
        for side in range(2):
            radius = float(rng.uniform(0.01, 0.12))
            half = 0.0 if (k % 4 == 0 and side == k % 2) else float(rng.uniform(0.0, 0.3))
            sizes.append((radius, half))
        # Centres close enough that separations and penetrations both occur.
        pos = [rng.uniform(-0.25, 0.25, size=3) for _ in range(2)]
        quat = [_random_unit_quat(rng) for _ in range(2)]
        # MuJoCo rejects a zero half-length capsule; a sphere is that shape.
        spec = "\n".join(
            (
                f'<geom name="g{i}" type="sphere" size="{sizes[i][0]}" '
                if sizes[i][1] == 0.0
                else f'<geom name="g{i}" type="capsule" size="{sizes[i][0]} {sizes[i][1]}" '
            )
            + f'pos="{pos[i][0]} {pos[i][1]} {pos[i][2]}" '
            f'quat="{quat[i][0]} {quat[i][1]} {quat[i][2]} {quat[i][3]}"/>'
            for i in range(2)
        )
        xml = f"<mujoco><worldbody>{spec}</worldbody></mujoco>"
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        fromto = np.zeros(6)
        dist = mujoco.mj_geomDistance(model, data, 0, 1, _DISTMAX, fromto)
        row: list[float] = []
        for i in range(2):
            xmat = data.geom_xmat[i].reshape(3, 3)
            quat_w = np.zeros(4)
            mujoco.mju_mat2Quat(quat_w, xmat.reshape(9))
            row += [model.geom_size[i][0], sizes[i][1]]
            row += list(data.geom_xpos[i])
            row += list(quat_w)
        row.append(float(dist))
        rows.append(row)
    return rows


def _robot_rows(
    mjcf: pathlib.Path, n_samples: int, rng: np.random.Generator
) -> tuple[list[list[object]], list[list[object]]]:
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    capsule_ids = [
        g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE
    ]
    geoms: list[list[object]] = []
    for g in capsule_ids:
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g]))
        geoms.append(
            [
                g,
                body,
                *model.geom_pos[g],
                *model.geom_quat[g],
                model.geom_size[g][0],
                model.geom_size[g][1],
            ]
        )
    lo = np.where(model.jnt_limited == 1, model.jnt_range[:, 0], -np.pi)
    hi = np.where(model.jnt_limited == 1, model.jnt_range[:, 1], np.pi)
    dists: list[list[object]] = []
    fromto = np.zeros(6)
    for s in range(n_samples):
        q = rng.uniform(lo, hi)
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        for a_i, g1 in enumerate(capsule_ids):
            for g2 in capsule_ids[a_i + 1 :]:
                if model.geom_bodyid[g1] == model.geom_bodyid[g2]:
                    continue  # same rigid body — the core rejects such pairs
                d = mujoco.mj_geomDistance(model, data, g1, g2, _DISTMAX, fromto)
                dists.append([s, g1, g2, float(d), *q])
    return geoms, dists


def _write(path: pathlib.Path, header: list[str], rows: list[list[object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{v:.17g}" if isinstance(v, float) else v for v in r])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("out_dir", type=pathlib.Path, help="rtc_urdf_bridge/test/data")
    parser.add_argument(
        "--robot-mjcf", type=pathlib.Path, help="robot MJCF with capsule collision geoms"
    )
    parser.add_argument(
        "--robot-name", default="robot", help="fixture name stem for the robot files"
    )
    parser.add_argument("--pairs", type=int, default=200)
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--seed", type=int, default=642)
    args = parser.parse_args(argv)

    if _MUJOCO_IMPORT_ERROR is not None:
        print(f"mujoco is not importable: {_MUJOCO_IMPORT_ERROR}", file=sys.stderr)
        return 2

    rng = np.random.default_rng(args.seed)
    shape_header = []
    for i in (1, 2):
        shape_header += [
            f"r{i}",
            f"hl{i}",
            f"x{i}",
            f"y{i}",
            f"z{i}",
            f"qw{i}",
            f"qx{i}",
            f"qy{i}",
            f"qz{i}",
        ]
    shape_header.append("dist")
    _write(args.out_dir / "capsule_oracle_shapes.csv", shape_header, _shape_rows(args.pairs, rng))

    if args.robot_mjcf is not None:
        geoms, dists = _robot_rows(args.robot_mjcf, args.samples, rng)
        nq = len(dists[0]) - 4 if dists else 0
        _write(
            args.out_dir / f"capsule_oracle_{args.robot_name}_geoms.csv",
            ["geom", "body", "px", "py", "pz", "qw", "qx", "qy", "qz", "r", "hl"],
            geoms,
        )
        _write(
            args.out_dir / f"capsule_oracle_{args.robot_name}_dist.csv",
            ["sample", "g1", "g2", "dist", *[f"q{i}" for i in range(nq)]],
            dists,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
