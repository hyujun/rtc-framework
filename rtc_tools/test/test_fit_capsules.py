"""fit_capsules: a drafted capsule contains its mesh, `check` judges a link by
the union of its capsules, a link without a mesh is a skip, not a failure."""

from __future__ import annotations

import pathlib
import struct

import numpy as np
import pytest
import yaml

from rtc_tools.validation import fit_capsules as fc


def _write_binary_stl(path: pathlib.Path, triangles: np.ndarray) -> None:
    with path.open("wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for v in tri:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def _tube_triangles(length: float, radius: float, n: int = 24) -> np.ndarray:
    """A tube along +x from 0 to `length`, as triangles (the vertices matter)."""
    ang = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    ring0 = np.stack([np.zeros(n), radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    ring1 = ring0 + np.array([length, 0.0, 0.0])
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris.append([ring0[i], ring0[j], ring1[i]])
        tris.append([ring1[i], ring0[j], ring1[j]])
    return np.asarray(tris)


def _urdf(mesh_uri: str, xyz: str = "0 0 0", rpy: str = "0 0 0") -> str:
    return f"""<robot name="t">
  <link name="base"/>
  <link name="tube">
    <collision><origin xyz="{xyz}" rpy="{rpy}"/>
      <geometry><mesh filename="{mesh_uri}"/></geometry></collision>
  </link>
  <link name="bare"/>
  <joint name="j" type="revolute"><parent link="base"/><child link="tube"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="k" type="fixed"><parent link="tube"/><child link="bare"/></joint>
</robot>"""


def _spec(capsules: list[dict]) -> str:
    return yaml.safe_dump(
        {"collision_capsules": {"capsules": capsules, "pairs": [], "ball_capsules": []}}
    )


@pytest.fixture
def scene(tmp_path: pathlib.Path) -> dict:
    mesh = tmp_path / "tube.stl"
    _write_binary_stl(mesh, _tube_triangles(0.3, 0.04))
    urdf = tmp_path / "t.urdf"
    # The mesh is rotated into the link's y axis and shifted: the fit must
    # read the origin, or the capsule lands on the wrong axis.
    urdf.write_text(_urdf(mesh.as_uri(), xyz="0.1 0 0", rpy="0 0 1.5707963267948966"))
    return {"mesh": mesh, "urdf": urdf, "tmp": tmp_path}


def test_fit_contains_every_vertex_and_reads_the_origin(scene: dict) -> None:
    out = scene["tmp"] / "caps.yaml"
    argv = ["fit", "--urdf", str(scene["urdf"]), "--links", "tube", "--tighten", "--out", str(out)]
    assert fc.main(argv) == 0
    spec = yaml.safe_load(out.read_text())["collision_capsules"]
    assert spec["pairs"] == [] and spec["ball_capsules"] == []
    (cap,) = spec["capsules"]
    assert cap["link"] == "tube"
    p0, p1 = np.asarray(cap["p0"]), np.asarray(cap["p1"])
    # The tube runs along +y from (0.1, 0, 0) after the origin. A tube has no
    # end caps to shrink into, so the tightened segment keeps the tube's length.
    axis = p1 - p0
    assert abs(abs(axis[1]) - np.linalg.norm(axis)) < 1e-6
    assert abs(np.linalg.norm(axis) - 0.3) < 1e-3
    assert abs(p0[0] - 0.1) < 1e-3 and abs(p1[0] - 0.1) < 1e-3
    assert 0.04 <= cap["radius"] <= 0.0402
    # Every vertex inside, by the tool's own measure.
    root = fc.load_urdf(scene["urdf"])
    v, why = fc.link_vertices(root, "tube", {})
    assert why == "ok"
    assert float(fc.segment_distances(v, p0, p1).max()) <= cap["radius"] + 1e-9


def test_check_judges_by_name_and_skips_links_without_a_mesh(scene: dict, capsys) -> None:
    caps = scene["tmp"] / "caps.yaml"
    caps.write_text(
        _spec(
            [
                {
                    "name": "thin",
                    "link": "tube",
                    "p0": [0.1, 0, 0],
                    "p1": [0.1, 0.3, 0],
                    "radius": 0.02,
                },
                {
                    "name": "no_mesh",
                    "link": "bare",
                    "p0": [0, 0, 0],
                    "p1": [0, 0, 0.1],
                    "radius": 0.01,
                },
                {
                    "name": "no_link",
                    "link": "ghost",
                    "p0": [0, 0, 0],
                    "p1": [0, 0, 0.1],
                    "radius": 0.01,
                },
            ]
        )
    )
    rc = fc.main(["check", "--urdf", str(scene["urdf"]), "--capsules", str(caps)])
    out = capsys.readouterr().out
    assert rc == 1
    assert "thin: FAIL" in out
    assert "no_mesh: SKIP (no mesh geometry)" in out
    assert "no_link: SKIP (link not in the URDF)" in out
    assert "checked 1, skipped 2, failed 1" in out


def test_a_link_covered_by_two_capsules_passes_as_their_union(scene: dict, capsys) -> None:
    # Two half-tube capsules: each alone leaves the other half outside, the
    # union covers the tube.
    near = {
        "name": "near",
        "link": "tube",
        "p0": [0.1, 0, 0],
        "p1": [0.1, 0.15, 0],
        "radius": 0.041,
    }
    far = {
        "name": "far",
        "link": "tube",
        "p0": [0.1, 0.15, 0],
        "p1": [0.1, 0.3, 0],
        "radius": 0.041,
    }
    caps = scene["tmp"] / "caps.yaml"
    for subset, expect, rc in (([near], "near: FAIL", 1), ([near, far], "near, far: ok", 0)):
        caps.write_text(_spec(subset))
        assert fc.main(["check", "--urdf", str(scene["urdf"]), "--capsules", str(caps)]) == rc
        assert expect in capsys.readouterr().out


def test_a_missing_mesh_file_is_a_skip(scene: dict, capsys) -> None:
    urdf = scene["tmp"] / "missing.urdf"
    urdf.write_text(_urdf("package://nowhere/meshes/x.STL"))
    caps = scene["tmp"] / "caps.yaml"
    caps.write_text(
        _spec([{"name": "c", "link": "tube", "p0": [0, 0, 0], "p1": [0, 0.3, 0], "radius": 0.05}])
    )
    assert fc.main(["check", "--urdf", str(urdf), "--capsules", str(caps)]) == 0
    assert "checked 0, skipped 1, failed 0" in capsys.readouterr().out
    # ... and the fit reports it, failing only under --strict.
    draft = str(scene["tmp"] / "d.yaml")
    assert fc.main(["fit", "--urdf", str(urdf), "--links", "tube", "--out", draft]) == 0
    assert (
        fc.main(["fit", "--urdf", str(urdf), "--links", "tube", "--strict", "--out", draft]) == 1
    )


def test_ascii_stl_reads_like_binary(tmp_path: pathlib.Path) -> None:
    tris = _tube_triangles(0.2, 0.03, n=8)
    binary = tmp_path / "b.stl"
    _write_binary_stl(binary, tris)
    ascii_path = tmp_path / "a.stl"
    lines = ["solid t"]
    for tri in tris:
        lines += ["facet normal 0 0 0", "outer loop"]
        lines += [f"vertex {v[0]} {v[1]} {v[2]}" for v in tri]
        lines += ["endloop", "endfacet"]
    lines.append("endsolid t")
    ascii_path.write_text("\n".join(lines))
    a = fc.read_stl_vertices(ascii_path)
    b = fc.read_stl_vertices(binary)
    assert a.shape == b.shape
    assert np.allclose(np.sort(a, axis=0), np.sort(b, axis=0), atol=1e-6)
