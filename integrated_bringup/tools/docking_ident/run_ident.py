"""Fly the identification protocol for one robot profile, a stage at a time.

dynamic_catching E1-F15 (#741). The protocol and its lattices are in
report.py; the rig is rig.py. Each stage leaves a store under
``$DATA/<profile>/`` and can be interrupted: what is already in the store is
not flown again. Nothing is written inside the repository.

Run with the workspace environment (its python has ``mujoco``), one BLAS
thread per worker, and no build running beside it::

    ( cd <workspace> \\
      && source <repo>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 \\
      && export DATA=<dir> OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \\
      && python3 <repo>/integrated_bringup/tools/docking_ident/run_ident.py <profile> all )

Stages, in order: ``selfcheck``, ``map-coarse``, ``map-fine``, ``lateral``,
``vperp``, ``static``, ``verify`` (``all`` runs them in that order). Then
``report.py <profile>`` prints the result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import multiprocessing
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import report as rp  # noqa: E402
import rig_config  # noqa: E402

from rtc_tools.analysis import catching_capture_set as cs  # noqa: E402

STAGES = ("selfcheck", "map-coarse", "map-fine", "lateral", "vperp", "static", "verify")
SAVE_EVERY = 200  # results between two writes of a store

# Stage numbers that seed the random streams (a stage's draws never depend on
# another's, or on how many workers ran it).
_STREAM = {"map-fine": 2, "lateral": 4, "vperp": 5, "verify": 8}

_RIG = None


def _worker_init(profile: str) -> None:
    global _RIG
    import rig as rig_module

    _RIG = rig_module.DockingRig(rig_config.load_rig_config(profile))


def _fly(spec: dict) -> tuple[str, dict]:
    result = _RIG.fly_in(spec["rho"], spec["c"], spec["delta_o"], spec["nu"], spec["s_pass"])
    return spec["id"], result


def _field_slab(xs_slab: list[float]) -> tuple[list[float], np.ndarray, np.ndarray]:
    hand, stray = _RIG.contact_field(xs_slab, rp.lattice(rp.FIELD_XY), rp.lattice(rp.FIELD_S))
    return xs_slab, hand, stray


def fly_all(pool, store: rp.Store, specs: list[dict], label: str) -> None:
    """Fly every spec that is not in the store yet, saving as it goes."""
    todo = [s for s in specs if s["id"] not in store.items]
    if not todo:
        return
    started = time.time()
    for done, (key, result) in enumerate(pool.imap_unordered(_fly, todo, chunksize=2), start=1):
        store.items[key] = result
        if done % SAVE_EVERY == 0:
            store.save()
            rate = done / (time.time() - started)
            print(f"  {label}: {done}/{len(todo)}  ({rate:.1f}/s)", flush=True)
    store.save()
    print(f"  {label}: {len(todo)} flown in {time.time() - started:.0f} s", flush=True)


def _spec(key: str, rho, c: float, delta_o: float, nu=(0.0, 0.0), s_pass: float = 0.0) -> dict:
    return {
        "id": key,
        "rho": [float(rho[0]), float(rho[1])],
        "c": float(c),
        "delta_o": float(delta_o),
        "nu": [float(nu[0]), float(nu[1])],
        "s_pass": float(s_pass),
    }


def _in_disc(rng: np.random.Generator, radius: float) -> tuple[float, float]:
    r, angle = radius * math.sqrt(rng.uniform()), rng.uniform(0.0, 2.0 * math.pi)
    return r * math.cos(angle), r * math.sin(angle)


def _in_cell(rng: np.random.Generator, a: int, b: int) -> tuple[float, float]:
    """(c, delta_o) uniform in fine cell (a, b)."""
    half_c, half_d = 0.5 * rp.FINE_DC, 0.5 * rp.FINE_DD
    return (
        rp.fine_c(a) + rng.uniform(-half_c, half_c),
        rp.fine_d(b) + rng.uniform(-half_d, half_d),
    )


# ── Stages ────────────────────────────────────────────────────────────────────


def stage_selfcheck(profile: str, directory: Path, pool) -> None:
    import mujoco
    import rig as rig_module

    cfg = rig_config.load_rig_config(profile)
    rig = rig_module.DockingRig(cfg)
    first = rig.fly_in((0.0, 0.0), 1.0, 0.0)
    again = rig.fly_in((0.0, 0.0), 1.0, 0.0)
    # The contact field skips the rest of the forward pass: check that on points
    # around the hand it sees the contacts a full one does.
    rng = np.random.default_rng([rp.SEED, 0])
    agree = checked = 0
    for _ in range(400):
        point = (rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08), rng.uniform(-0.01, 0.15))
        rig.restore()
        rig.put_ball(point)
        mujoco.mj_kinematics(rig.model, rig.data)
        mujoco.mj_collision(rig.model, rig.data)
        fast = rig._ball_contacts()
        rig.restore()
        rig.put_ball(point)
        mujoco.mj_forward(rig.model, rig.data)
        checked += 1
        agree += fast == rig._ball_contacts()
    here = Path(__file__).resolve().parent
    check = {
        "profile": profile,
        "model": Path(cfg.model_path).name,
        # What flew: the rig's and the config reader's source, by content.
        "rig_sha256": {
            name: hashlib.sha256((here / name).read_bytes()).hexdigest()
            for name in ("rig.py", "rig_config.py")
        },
        "applied": rig.applied,
        "settle_s": rig.settle_s,
        "hand_error": rig.hand_error(),
        "repeatable": first == again,
        "restitution": rig_module.measure_restitution(cfg),
        "empty_close_s": rig.empty_close_time(),
        "t_close_e2e": cfg.t_close_e2e,
        "axis_contact": rig.first_contact_on_axis(),
        "collision_agrees": agree,
        "collision_checked": checked,
        "example": first,
    }
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "selfcheck.json").write_text(json.dumps(check, indent=1))
    print(json.dumps({k: v for k, v in check.items() if k != "example"}, indent=1))
    if not check["repeatable"]:
        raise SystemExit("the same fly-in gave two results: the rig is not deterministic")


def stage_map_coarse(profile: str, directory: Path, pool) -> None:
    store = rp.Store(directory / "map_coarse.json")
    specs = [
        _spec(f"c{i}_d{j}", (0.0, 0.0), c, d)
        for i, c in enumerate(rp.lattice(rp.COARSE_C))
        for j, d in enumerate(rp.lattice(rp.COARSE_D))
    ]
    fly_all(pool, store, specs, "map-coarse")
    held = rp.coarse_held(store)
    print(f"  coarse: {int(held.sum())} of {held.size} held")
    for i in range(held.shape[0] - 1, -1, -1):
        print(
            f"  {rp.lattice(rp.COARSE_C)[i]:5.2f} " + "".join("H" if h else "." for h in held[i])
        )


def stage_map_fine(profile: str, directory: Path, pool) -> None:
    coarse = rp.Store(directory / "map_coarse.json")
    if not coarse.items:
        raise SystemExit("map-fine needs map-coarse")
    store = rp.Store(directory / "map_fine.json")
    specs = []
    for a, b in rp.fine_cells(rp.coarse_held(coarse)):
        rng = np.random.default_rng([rp.SEED, _STREAM["map-fine"], a, b + 10_000])
        for k in range(rp.TRIALS):
            c, delta_o = _in_cell(rng, a, b)
            specs.append(_spec(f"a{a}_b{b}_k{k}", _in_disc(rng, rp.RHO_JITTER), c, delta_o))
    fly_all(pool, store, specs, "map-fine")


def _boxes(directory: Path) -> tuple[rp.FineMap, dict[str, cs.Box]]:
    fine = rp.FineMap(rp.Store(directory / "map_fine.json"))
    boxes = rp.distinct_boxes(fine.boxes())
    if not boxes:
        raise SystemExit("the fine map has no box of any requested width")
    return fine, boxes


def stage_lateral(profile: str, directory: Path, pool) -> None:
    fine, boxes = _boxes(directory)
    xs = rp.lattice(rp.LATERAL)
    half = 0.5 * rp.LATERAL[1]
    for tag, box in boxes.items():
        store = rp.Store(directory / f"lateral_{tag}.json")
        alive = np.ones((xs.size, xs.size), dtype=bool)
        for q, which in enumerate(rp.CONDITIONS):
            a, b = fine.cell(box, which)
            # One pass per fly-in: a cell that has failed is not flown again.
            for k in range(rp.TRIALS):
                specs = []
                for i, j in zip(*np.nonzero(alive), strict=True):
                    rng = np.random.default_rng([rp.SEED, _STREAM["lateral"], q, i, j, k])
                    c, delta_o = _in_cell(rng, a, b)
                    rho = (xs[i] + rng.uniform(-half, half), xs[j] + rng.uniform(-half, half))
                    specs.append(_spec(f"q{q}_x{i}_y{j}_k{k}", rho, c, delta_o))
                fly_all(pool, store, specs, f"lateral {tag} {which} #{k}")
                for spec in specs:
                    _, i, j, _ = (int(part[1:]) for part in spec["id"].split("_"))
                    alive[i, j] &= bool(store.items[spec["id"]]["held"])
        print(f"  lateral {tag}: {int(alive.sum())} cells held under all conditions")


def stage_vperp(profile: str, directory: Path, pool) -> None:
    fine, boxes = _boxes(directory)
    rings = rp.lattice(rp.VPERP)
    half = 0.5 * rp.VPERP[1]
    sector = math.pi / rp.VPERP_DIRECTIONS
    for tag, box in boxes.items():
        store = rp.Store(directory / f"vperp_{tag}.json")
        cells = [fine.cell(box, which) for which in rp.CONDITIONS]
        for m, ring in enumerate(rings):
            specs = []
            for q, (a, b) in enumerate(cells):
                for d in range(rp.VPERP_DIRECTIONS):
                    for k in range(rp.TRIALS):
                        rng = np.random.default_rng([rp.SEED, _STREAM["vperp"], q, m, d, k])
                        c, delta_o = _in_cell(rng, a, b)
                        speed = ring + rng.uniform(-half, half)
                        angle = 2.0 * sector * d + rng.uniform(-sector, sector)
                        nu = (speed * math.cos(angle), speed * math.sin(angle))
                        key = f"q{q}_m{m}_d{d}_k{k}"
                        specs.append(_spec(key, _in_disc(rng, rp.RHO_JITTER), c, delta_o, nu))
            fly_all(pool, store, specs, f"vperp {tag} ring {ring:.2f}")
            # Rings are flown outward and stop at the first that fails anywhere.
            if not all(store.items[s["id"]]["held"] for s in specs):
                break
        print(f"  vperp {tag}: v_perp_max {cs.largest_held_radius(rings, rp.vperp_held(store))}")


def stage_static(profile: str, directory: Path, pool) -> None:
    path = directory / "static.npz"
    if path.is_file():
        return
    xs = rp.lattice(rp.FIELD_XY)
    started = time.time()
    slabs = [list(map(float, xs[i : i + 2])) for i in range(0, xs.size, 2)]
    parts = {tuple(x): (h, s) for x, h, s in pool.imap_unordered(_field_slab, slabs)}
    hand = np.concatenate([parts[tuple(slab)][0] for slab in slabs], axis=0)
    stray = np.concatenate([parts[tuple(slab)][1] for slab in slabs], axis=0)
    np.savez_compressed(path, xs=xs, ys=xs, ss=rp.lattice(rp.FIELD_S), hand=hand, stray=stray)
    print(
        f"  static: {hand.size} points in {time.time() - started:.0f} s — hand {int(hand.sum())}, "
        f"other {int(stray.sum())}, top level touches: {bool(hand[:, :, -1].any())}"
    )


def stage_verify(profile: str, directory: Path, pool) -> None:
    fine, boxes = _boxes(directory)
    for tag, box in boxes.items():
        ident = rp.identify(directory, tag, box, fine)
        if ident.get("polygon") is None or "v_perp_max" not in ident or "entrance" not in ident:
            print(f"  verify {tag}: skipped — the set is not identified (lateral/vperp/static)")
            continue
        whole = ident["entrance"]
        # rho is a point of the entrance plane; without a plane, of the origin plane.
        s_pass = whole.s_ent if whole is not None and whole.holds else 0.0
        width_ms = int(tag[1:])
        rng = np.random.default_rng([rp.SEED, _STREAM["verify"], width_ms])
        sample = cs.sample_capture_set(
            rng, ident["polygon"], box, ident["v_perp_max"], rp.VERIFY_N
        )
        specs = [
            _spec(f"v{n:03d}", row[0:2], row[2], row[3], row[4:6], s_pass)
            for n, row in enumerate(sample)
        ]
        store = rp.Store(directory / f"verify_{tag}.json")
        fly_all(pool, store, specs, f"verify {tag}")
        held = sum(bool(store.items[s["id"]]["held"]) for s in specs)
        print(f"  verify {tag}: {held} of {len(specs)} held")


_RUN = {
    "selfcheck": stage_selfcheck,
    "map-coarse": stage_map_coarse,
    "map-fine": stage_map_fine,
    "lateral": stage_lateral,
    "vperp": stage_vperp,
    "static": stage_static,
    "verify": stage_verify,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("profile", help="a robot profile under the bring-up's config/")
    parser.add_argument("stage", choices=(*STAGES, "all"))
    parser.add_argument("--workers", type=int, default=6, help="processes, each with its own rig")
    args = parser.parse_args(argv)
    if os.environ.get("OMP_NUM_THREADS") != "1":
        raise SystemExit("set OMP_NUM_THREADS=1: every worker would start a thread pool")
    directory = rp.data_dir(args.profile)
    directory.mkdir(parents=True, exist_ok=True)
    stages = STAGES if args.stage == "all" else (args.stage,)
    context = multiprocessing.get_context("spawn")
    with context.Pool(args.workers, initializer=_worker_init, initargs=(args.profile,)) as pool:
        for stage in stages:
            print(f"[{args.profile}] {stage}", flush=True)
            _RUN[stage](args.profile, directory, pool)
    return 0


if __name__ == "__main__":
    sys.exit(main())
