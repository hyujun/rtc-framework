"""Each real-robot launch feeds controller_manager a usable RT parameter file (#343).

``rtc_tools/test/test_cm_rt_params.py`` locks the generator in isolation; this
locks the *wiring* — that the shipped robot configs actually satisfy it, which no
unit test can see:

* ``control_rate`` exists in each ``_base.yaml`` and is a whole number of Hz, so
  it survives the conversion to controller_manager's integer ``update_rate``.
  The two rates were never cross-checked before: the CM ran at whatever the
  upstream default file said (500) while the RTC controller used ``control_rate``,
  and nothing would have noticed them diverging.
* ``use_cpu_affinity:=false`` drops ``cpu_affinity`` and nothing else.

The launch modules are loaded from the *installed* share directory, so a launch
file that stops calling the generator, or a config that loses ``control_rate``,
fails here rather than at robot bringup.
"""

from __future__ import annotations

import importlib.util
import os
import types

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

ROBOTS = [("ur5e_p1a", "robot_ur5e_p1a.launch.py"), ("ur5e_p1b", "robot_ur5e_p1b.launch.py")]


def _share() -> str:
    return get_package_share_directory("integrated_bringup")


def _load_launch_module(filename: str):
    path = os.path.join(_share(), "launch", filename)
    spec = importlib.util.spec_from_file_location(f"_launch_{filename}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(affinity: bool):
    return types.SimpleNamespace(
        launch_configurations={"use_cpu_affinity": "true" if affinity else "false"}
    )


def _written_params(path: str) -> dict:
    with open(path) as handle:
        return yaml.safe_load(handle)["controller_manager"]["ros__parameters"]


@pytest.mark.parametrize(("variant", "launch_file"), ROBOTS)
def test_update_rate_matches_config_control_rate(variant, launch_file, tmp_path):
    module = _load_launch_module(launch_file)
    path, _ = module._build_cm_rt_params(_context(True), str(tmp_path), variant)

    with open(os.path.join(_share(), "config", variant, "_base.yaml")) as handle:
        control_rate = yaml.safe_load(handle)["/**"]["ros__parameters"]["control_rate"]

    assert _written_params(path)["update_rate"] == int(control_rate)


@pytest.mark.parametrize(("variant", "launch_file"), ROBOTS)
def test_affinity_present_when_enabled(variant, launch_file, tmp_path):
    module = _load_launch_module(launch_file)
    path, _ = module._build_cm_rt_params(_context(True), str(tmp_path), variant)
    params = _written_params(path)

    # The pin is the whole point of the file; a run that silently loses it is the
    # pre-#343 state with better logging.
    assert "cpu_affinity" in params
    assert params["thread_priority"] > 0
    assert params["lock_memory"] is True


@pytest.mark.parametrize(("variant", "launch_file"), ROBOTS)
def test_affinity_omitted_when_disabled(variant, launch_file, tmp_path):
    module = _load_launch_module(launch_file)
    path, _ = module._build_cm_rt_params(_context(False), str(tmp_path), variant)
    params = _written_params(path)

    assert "cpu_affinity" not in params
    # Only affinity drops out — the node still needs the rate.
    assert params["update_rate"] > 0
