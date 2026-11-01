#!/usr/bin/env python3
"""Fit collision capsules to a URDF's link meshes, and check a capsule file.

The capsule distance core (``rtc_urdf_bridge/collision``) reads a robot-level
``collision_capsules.yaml``; this tool drafts its ``capsules`` entries from the
links' collision meshes and checks a finished file against them.

``fit``   one capsule per link: the mesh vertices (collision geometry first,
          visual as the fallback) in the LINK frame — the geometry's origin and
          scale applied — their principal axis (PCA), the segment between the
          extreme projections on it, and the largest perpendicular distance as
          the radius, so every vertex is inside. ``--tighten`` then shrinks
          each end inward as far as every vertex still stays inside. The draft
          is a start: pairs and ball capsules are written empty for the
          profile's author, and a wide link may want two capsules by hand.

``check`` every link that has a mesh and a capsule: each vertex must be inside
          one of the link's capsules (distance to that capsule's axis segment at
          most radius + ``--tol``), so a trunk may be a spine and a shoulder bar.
          A link whose mesh cannot be found is SKIPPED and said so (the exit
          code stays 0 for a skip, 1 for a failure) — record the run/skip count
          with the gate.

``package://pkg/path`` mesh URIs resolve through ``--mesh-root pkg=DIR``
(repeatable), then ``ROS_PACKAGE_PATH`` / ament, else the mesh is missing.
A ``.xacro`` URDF is expanded with the ``xacro`` executable on PATH.

The tool is robot-agnostic: the links are arguments, nothing is named here.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import yaml

_STL_BINARY_HEADER = 80
_STL_BINARY_RECORD = 50


# ── Meshes ───────────────────────────────────────────────────────────────────


def read_stl_vertices(path: pathlib.Path) -> np.ndarray:
    """The unique vertices of an STL (binary or ASCII), N × 3."""
    data = path.read_bytes()
    is_binary = len(data) >= _STL_BINARY_HEADER + 4
    if is_binary:
        (count,) = struct.unpack_from("<I", data, _STL_BINARY_HEADER)
        is_binary = len(data) == _STL_BINARY_HEADER + 4 + count * _STL_BINARY_RECORD
    if is_binary:
        (count,) = struct.unpack_from("<I", data, _STL_BINARY_HEADER)
        records = np.frombuffer(
            data,
            dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")], align=False),
            count=count,
            offset=_STL_BINARY_HEADER + 4,
        )
        verts = records["v"].reshape(-1, 3).astype(np.float64)
    else:
        text = data.decode("ascii", errors="ignore")
        rows = []
        for line in text.splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                rows.append([float(parts[1]), float(parts[2]), float(parts[3])])
        verts = np.asarray(rows, dtype=np.float64).reshape(-1, 3)
    if verts.size == 0:
        raise ValueError(f"{path}: no vertex")
    return np.unique(verts, axis=0)


def _rpy_matrix(rpy: list[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


@dataclass
class MeshRef:
    filename: str
    xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    scale: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])


def _floats(text: str | None, n: int, default: float) -> list[float]:
    if text is None:
        return [default] * n
    vals = [float(v) for v in text.split()]
    if len(vals) == 1 and n == 3:
        vals = vals * 3
    if len(vals) != n:
        raise ValueError(f"expected {n} numbers, got '{text}'")
    return vals


def load_urdf(path: pathlib.Path) -> ET.Element:
    """The URDF root; a .xacro is expanded with the xacro executable."""
    if path.suffix == ".xacro":
        exe = shutil.which("xacro")
        if exe is None:
            raise RuntimeError("a .xacro URDF needs the `xacro` executable on PATH")
        text = subprocess.run([exe, str(path)], check=True, capture_output=True, text=True).stdout
        return ET.fromstring(text)
    return ET.parse(path).getroot()


def link_meshes(root: ET.Element, link: str) -> list[MeshRef]:
    """The link's collision meshes, else its visual meshes (the link frame)."""
    node = next((e for e in root.iter("link") if e.get("name") == link), None)
    if node is None:
        raise KeyError(f"link '{link}' is not in the URDF")
    for tag in ("collision", "visual"):
        refs = []
        for geom in node.findall(tag):
            mesh = geom.find("geometry/mesh")
            if mesh is None or mesh.get("filename") is None:
                continue
            origin = geom.find("origin")
            refs.append(
                MeshRef(
                    filename=mesh.get("filename", ""),
                    xyz=_floats(origin.get("xyz") if origin is not None else None, 3, 0.0),
                    rpy=_floats(origin.get("rpy") if origin is not None else None, 3, 0.0),
                    scale=_floats(mesh.get("scale"), 3, 1.0),
                )
            )
        if refs:
            return refs
    return []


def resolve_mesh(uri: str, roots: dict[str, pathlib.Path]) -> pathlib.Path | None:
    """`package://pkg/rel` or `file://` or a plain path → an existing file, else None."""
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://") :].partition("/")
        candidates = []
        if pkg in roots:
            candidates.append(roots[pkg] / rel)
        for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
            if entry:
                candidates.append(pathlib.Path(entry) / pkg / rel)
                candidates.append(pathlib.Path(entry) / rel)
        try:  # the installed share dir, when ament is importable
            from ament_index_python.packages import get_package_share_directory

            candidates.append(pathlib.Path(get_package_share_directory(pkg)) / rel)
        except Exception:  # noqa: BLE001 - absent ament, unknown package: not a mesh
            pass
        return next((c for c in candidates if c.is_file()), None)
    if uri.startswith("file://"):
        uri = uri[len("file://") :]
    p = pathlib.Path(uri)
    return p if p.is_file() else None


def link_vertices(
    root: ET.Element, link: str, roots: dict[str, pathlib.Path]
) -> tuple[np.ndarray | None, str]:
    """All mesh vertices of the link in the LINK frame, or (None, why)."""
    try:
        refs = link_meshes(root, link)
    except KeyError:
        return None, "link not in the URDF"
    if not refs:
        return None, "no mesh geometry"
    chunks = []
    for ref in refs:
        path = resolve_mesh(ref.filename, roots)
        if path is None:
            return None, f"mesh not found: {ref.filename}"
        if path.suffix.lower() != ".stl":
            return None, f"not an STL: {path.name}"
        v = read_stl_vertices(path) * np.asarray(ref.scale)
        v = v @ _rpy_matrix(ref.rpy).T + np.asarray(ref.xyz)
        chunks.append(v)
    return np.vstack(chunks), "ok"


# ── Capsules ─────────────────────────────────────────────────────────────────


def segment_distances(v: np.ndarray, p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
    """Distance of every vertex to the segment p0–p1."""
    d = p1 - p0
    len2 = float(d @ d)
    if len2 < 1e-24:
        return np.linalg.norm(v - p0, axis=1)
    t = np.clip(((v - p0) @ d) / len2, 0.0, 1.0)
    return np.linalg.norm(v - (p0 + t[:, None] * d), axis=1)


def fit_capsule(v: np.ndarray, tighten: bool) -> tuple[np.ndarray, np.ndarray, float]:
    """PCA axis, extreme projections, largest perpendicular distance; ends shrunk
    inward while every vertex stays inside (`tighten`)."""
    c = v.mean(axis=0)
    _, _, vt = np.linalg.svd(v - c, full_matrices=False)
    u = vt[0]
    t = (v - c) @ u
    p0 = c + t.min() * u
    p1 = c + t.max() * u
    r = float(np.max(np.linalg.norm((v - c) - t[:, None] * u, axis=1)))
    if not tighten:
        return p0, p1, r
    length = float(t.max() - t.min())
    for end in (0, 1):
        lo, hi = 0.0, min(r, 0.5 * length)  # a shrink that keeps the segment a segment
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            q0 = p0 + mid * u if end == 0 else p0
            q1 = p1 - mid * u if end == 1 else p1
            if float(segment_distances(v, q0, q1).max()) <= r + 1e-12:
                lo = mid
            else:
                hi = mid
        if end == 0:
            p0 = p0 + lo * u
        else:
            p1 = p1 - lo * u
    return p0, p1, r


def _roots(args: list[str] | None) -> dict[str, pathlib.Path]:
    out: dict[str, pathlib.Path] = {}
    for item in args or []:
        pkg, _, d = item.partition("=")
        if not pkg or not d:
            raise SystemExit(f"--mesh-root wants pkg=DIR, got '{item}'")
        out[pkg] = pathlib.Path(d)
    return out


def _fmt(x: float) -> float:
    return float(f"{x:.4f}")


def _fmt_up(x: float) -> float:
    """A radius rounded UP to 0.1 mm, so the rounded capsule still contains."""
    return math.ceil(x * 1e4 - 1e-9) / 1e4


def cmd_fit(args: argparse.Namespace) -> int:
    root = load_urdf(args.urdf)
    roots = _roots(args.mesh_root)
    capsules = []
    failures = 0
    for link in args.links:
        v, why = link_vertices(root, link, roots)
        if v is None:
            print(f"{link}: SKIP ({why})", file=sys.stderr)
            failures += 1
            continue
        p0, p1, r = fit_capsule(v, args.tighten)
        r += args.inflate
        capsules.append(
            {
                "name": link,
                "link": link,
                "p0": [_fmt(x) for x in p0],
                "p1": [_fmt(x) for x in p1],
                "radius": _fmt_up(r),
            }
        )
        print(
            f"{link}: {len(v)} vertices, axis length {np.linalg.norm(p1 - p0):.3f} m, "
            f"radius {r:.3f} m",
            file=sys.stderr,
        )
    doc = {"collision_capsules": {"capsules": capsules, "pairs": [], "ball_capsules": []}}
    text = yaml.safe_dump(doc, sort_keys=False, default_flow_style=None)
    if args.out is None:
        sys.stdout.write(text)
    else:
        args.out.write_text(text)
    return 1 if failures and args.strict else 0


def cmd_check(args: argparse.Namespace) -> int:
    root = load_urdf(args.urdf)
    roots = _roots(args.mesh_root)
    spec = yaml.safe_load(args.capsules.read_text())["collision_capsules"]
    # A link may be covered by several capsules (a trunk as a spine and a
    # shoulder bar): a vertex is inside when it is inside ANY of its link's.
    by_link: dict[str, list[dict]] = {}
    for cap in spec["capsules"]:
        by_link.setdefault(cap["link"], []).append(cap)
    run = skip = failed = 0
    for link, caps in by_link.items():
        names = ", ".join(c["name"] for c in caps)
        v, why = link_vertices(root, link, roots)
        if v is None:
            print(f"{names}: SKIP ({why})")
            skip += 1
            continue
        excess = np.full(len(v), np.inf)
        for cap in caps:
            p0 = np.asarray(cap["p0"], dtype=float)
            p1 = np.asarray(cap["p1"], dtype=float)
            excess = np.minimum(excess, segment_distances(v, p0, p1) - float(cap["radius"]))
        worst = float(excess.max())
        ok = worst <= args.tol
        run += 1
        failed += 0 if ok else 1
        print(
            f"{names}: {'ok' if ok else 'FAIL'} — largest vertex excess over the radius "
            f"{worst:+.4f} m (tol {args.tol}, {len(caps)} capsule{'s' if len(caps) > 1 else ''})"
        )
    print(f"checked {run}, skipped {skip}, failed {failed}")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)

    fit = sub.add_parser("fit", help="draft capsules from the links' meshes")
    fit.add_argument("--urdf", type=pathlib.Path, required=True)
    fit.add_argument("--links", nargs="+", required=True, help="link names, one capsule each")
    fit.add_argument("--mesh-root", action="append", metavar="PKG=DIR")
    fit.add_argument("--tighten", action="store_true", help="shrink the ends while contained")
    fit.add_argument("--inflate", type=float, default=0.0, help="added to every radius [m]")
    fit.add_argument("--strict", action="store_true", help="exit 1 when a link has no mesh")
    fit.add_argument("--out", type=pathlib.Path, help="write the YAML here (default: stdout)")
    fit.set_defaults(func=cmd_fit)

    check = sub.add_parser("check", help="every capsule contains its link's mesh vertices")
    check.add_argument("--urdf", type=pathlib.Path, required=True)
    check.add_argument("--capsules", type=pathlib.Path, required=True)
    check.add_argument("--mesh-root", action="append", metavar="PKG=DIR")
    check.add_argument("--tol", type=float, default=1e-3, help="allowed excess [m]")
    check.set_defaults(func=cmd_check)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
