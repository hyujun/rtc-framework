"""``sim_overlay:=`` resolution, shared by the per-profile sim launches.

The one behaviour worth pinning is the refusal. An overlay that does not
resolve and is skipped runs the SHIPPED scene under a command line naming a
different one — every measurement from that run then describes the wrong scene,
and nothing in the log says so. Returning None on a non-empty value would be
exactly that failure, so the tests below separate "unset" from "unresolvable".
"""

import os

import pytest

from integrated_bringup.sim_overlay import resolve_sim_overlay, sim_overlay_argument_description


def test_unset_is_not_an_error():
    assert resolve_sim_overlay("", "ur5e_p1b") is None
    assert resolve_sim_overlay("   ", "ur5e_p1b") is None


def test_a_bare_name_resolves_against_the_profiles_shipped_overlays():
    path = resolve_sim_overlay("inference_pole", "ur5e_p1b")
    assert path is not None
    assert os.path.isfile(path)
    assert path.endswith(os.path.join("ur5e_p1b", "sim_overlays", "inference_pole.yaml"))


def test_the_lookup_is_profile_scoped():
    """A name shipped by one profile must not resolve for another — the overlay
    changes the scene, and another robot's scene is not a fallback."""
    with pytest.raises(RuntimeError):
        resolve_sim_overlay("inference_pole", "iiwa7_leap")


def test_an_unresolvable_name_raises_rather_than_returning_none():
    with pytest.raises(RuntimeError) as excinfo:
        resolve_sim_overlay("no_such_overlay", "ur5e_p1b")
    message = str(excinfo.value)
    # The message has to carry the resolved path and what IS shipped, because
    # the usual cause is a typo and the usual next question is "then what is
    # there?".
    assert "no_such_overlay" in message
    assert "inference_pole" in message


def test_an_explicit_path_is_taken_as_a_path(tmp_path):
    overlay = tmp_path / "scratch_overlay.yaml"
    overlay.write_text("mujoco_simulator:\n  ros__parameters:\n    max_rtf: 2.0\n")
    assert resolve_sim_overlay(str(overlay), "iiwa7_leap") == str(overlay)


def test_a_missing_explicit_path_still_raises(tmp_path):
    with pytest.raises(RuntimeError):
        resolve_sim_overlay(str(tmp_path / "absent.yaml"), "ur5e_p1b")


def test_the_argument_description_names_the_profile():
    # Both launches show the operator their OWN overlay directory; a shared
    # description that named one profile would send the other's users there.
    assert "config/iiwa7_leap/sim_overlays" in sim_overlay_argument_description("iiwa7_leap")
    assert "config/ur5e_p1b/sim_overlays" in sim_overlay_argument_description("ur5e_p1b")
