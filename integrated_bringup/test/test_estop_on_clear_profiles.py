"""Where ``estop_on_clear`` ships, and that it names a controller it can reach.

The CM's post-clear policy (``rtc_controller_manager`` README, "글로벌 E-STOP
해제") defaults to ``resume``: the controller that was active when the E-STOP
latched carries on once the clear is verified. The two humanoid sims opt into
``initial_controller`` — after a stop under the locomotion controller the CM
hands the robot back to ``demo_joint_controller``, which holds the pose with the
scene servo gains — and every other profile keeps the default. This pins both
halves: a profile that gained the key by a copy-paste would change what its
operators see after a clear, and a profile that switches to a controller it
does not ship would refuse to configure at launch rather than here.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

# (profile, mode file) → the policy the shipped YAML must carry. Absent means
# `resume`, the CM default, and is the shape every other profile keeps.
EXPECTED_INITIAL_CONTROLLER = {("g1_loco", "sim.yaml"), ("b7_loco", "sim.yaml")}

KEY = "estop_on_clear"


def _parameter_files() -> list[tuple[str, str, dict]]:
    """(profile, path relative to the profile, params) for every YAML of every
    profile that carries a ``/**/ros__parameters`` block.

    Not only ``sim.yaml`` / ``robot.yaml``: ``_base.yaml`` and the overlay
    fragments are merged into the same parameter set at launch, so a key moved
    into one of them would reach the CM just the same.
    """
    files = []
    for profile_dir in sorted(p for p in CONFIG_DIR.iterdir() if p.is_dir()):
        for path in sorted(profile_dir.rglob("*.yaml")):
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
            params = doc.get("/**", {}).get("ros__parameters") if isinstance(doc, dict) else None
            if isinstance(params, dict):
                files.append((profile_dir.name, str(path.relative_to(profile_dir)), params))
    return files


# Parsed once at import: parametrize, its ids and the two whole-set tests all
# read this list.
PARAMETER_FILES = _parameter_files()
PARAMS = {(profile, mode): params for profile, mode, params in PARAMETER_FILES}


def test_the_mode_files_exist_for_the_profiles_that_switch():
    missing = EXPECTED_INITIAL_CONTROLLER - set(PARAMS)
    assert not missing, f"expected mode files are gone: {sorted(missing)}"


@pytest.mark.parametrize(
    "profile,mode,params", PARAMETER_FILES, ids=[f"{p}/{m}" for p, m, _ in PARAMETER_FILES]
)
def test_estop_on_clear_ships_only_where_decided(profile, mode, params):
    where = f"{profile}/{mode}"
    if (profile, mode) in EXPECTED_INITIAL_CONTROLLER:
        assert params.get(KEY) == "initial_controller", (
            f"{where}: the humanoid sim must switch back to its initial controller after a clear"
        )
    else:
        assert KEY not in params, (
            f"{where}: {KEY} is set to {params[KEY]!r}; only the humanoid sims opt out of `resume`"
        )


@pytest.mark.parametrize("profile,mode", sorted(EXPECTED_INITIAL_CONTROLLER))
def test_the_switch_target_is_a_controller_the_profile_ships(profile, mode):
    params = PARAMS[(profile, mode)]
    initial = params.get("initial_controller")
    assert isinstance(initial, str) and initial, (
        f"{profile}/{mode}: initial_controller must name the post-clear target"
    )
    # Controllers are instantiated from the profile's controllers/ directory
    # (REQUIRING_CONFIG): the config_key is the file stem.
    shipped = CONFIG_DIR / profile / "controllers" / f"{initial}.yaml"
    assert shipped.exists(), f"{profile}: initial_controller {initial!r} has no {shipped}"
