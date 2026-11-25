"""Whatever ``estop_on_clear`` a profile ships is a policy the CM accepts, and reaches a controller.

The CM's post-clear policy (``rtc_controller_manager`` README, "글로벌 E-STOP
해제") defaults to ``resume``: the controller that was active when the E-STOP
latched carries on once the clear is verified. A profile may instead opt into
``initial_controller`` — the CM hands the robot back to the profile's initial
controller. Which profiles opt in is a deployment decision and not pinned here;
this holds what a launch needs of it: the value is one the CM accepts, and an
``initial_controller`` policy names a controller the profile ships (otherwise
it would refuse to configure at launch rather than here).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

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


POLICIES = {"resume", "initial_controller"}
INITIAL = [(p, m) for p, m, params in PARAMETER_FILES if params.get(KEY) == "initial_controller"]


@pytest.mark.parametrize(
    "profile,mode,params", PARAMETER_FILES, ids=[f"{p}/{m}" for p, m, _ in PARAMETER_FILES]
)
def test_estop_on_clear_is_a_policy_the_cm_accepts(profile, mode, params):
    if KEY in params:
        assert params[KEY] in POLICIES, (
            f"{profile}/{mode}: {KEY} is {params[KEY]!r}; the CM accepts {sorted(POLICIES)}"
        )


@pytest.mark.parametrize("profile,mode", INITIAL, ids=[f"{p}/{m}" for p, m in INITIAL])
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
