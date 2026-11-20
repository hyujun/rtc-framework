"""rtc_tools.validation.perturb_mjcf: the scaled body, and only it."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from rtc_tools.validation.perturb_mjcf import main, perturb

MJCF = """<mujoco><worldbody>
  <body name="base"><freejoint/><inertial pos="0 0 0.1" mass="2" diaginertia="0.02 0.03 0.04"/>
    <body name="arm" pos="0 0.2 0.3"><joint name="j" axis="0 0 1"/>
      <inertial pos="0.1 0 0" mass="1" fullinertia="0.002 0.004 0.004 0 0 0.0005"/>
    </body>
    <body name="bare" pos="0 0 0.5"><geom size="0.01"/></body>
  </body></worldbody></mujoco>"""


def inertials(path):
    return {b.get("name"): b.find("inertial") for b in ET.parse(path).getroot().iter("body")}


def test_the_named_body_gets_heavier_and_keeps_its_shape(tmp_path):
    src, out = tmp_path / "r.xml", tmp_path / "r_heavy.xml"
    src.write_text(MJCF)
    assert perturb(src, out, {"arm": 1.1}) == {"arm": pytest.approx(0.1)}
    got = inertials(out)
    assert float(got["arm"].get("mass")) == pytest.approx(1.1)
    assert [float(x) for x in got["arm"].get("fullinertia").split()] == pytest.approx(
        [0.0022, 0.0044, 0.0044, 0.0, 0.0, 0.00055]
    )
    assert got["arm"].get("pos") == "0.1 0 0"  # the centre of mass stays
    assert got["base"].attrib == inertials(src)["base"].attrib  # nothing else moves
    assert set(got) == {"base", "arm", "bare"}  # no body added or removed


def test_the_compiled_plant_differs_by_the_added_mass(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    src, out = tmp_path / "r.xml", tmp_path / "r_heavy.xml"
    src.write_text(MJCF)
    perturb(src, out, {"base": 1.05, "arm": 0.9})
    before = mujoco.MjModel.from_xml_path(str(src)).body_mass.sum()
    after = mujoco.MjModel.from_xml_path(str(out)).body_mass.sum()
    assert after - before == pytest.approx(2 * 0.05 - 1 * 0.1)


@pytest.mark.parametrize("factors", [{"nope": 1.1}, {"bare": 1.1}, {"arm": 0.0}, {"arm": -1.0}])
def test_what_cannot_be_scaled_is_refused(tmp_path, factors):
    src = tmp_path / "r.xml"
    src.write_text(MJCF)
    with pytest.raises(ValueError):
        perturb(src, tmp_path / "o.xml", factors)
    assert not (tmp_path / "o.xml").exists()
    assert main([str(src), str(tmp_path / "o.xml"), "--scale", "nope=1.1"]) == 2
