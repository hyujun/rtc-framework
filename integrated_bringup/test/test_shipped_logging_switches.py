"""The logging switches the shipped YAML carries (#1126).

``enable_device_log`` was removed from the CM: a key left in a shipped file
would make every bring-up of that profile print the removal warning, so none
may carry it. The hand driver's ``enable_timing_log`` is the one switch for
``timing/hand_timing_log.csv`` and is spelled out in every driver file: the
launch's empty ``enable_logging`` argument passes that value through, so the
file states what a bring-up gets.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _params(path: Path) -> dict | None:
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        return None
    params = (doc.get("/**") or {}).get("ros__parameters")
    return params if isinstance(params, dict) else None


ALL_YAML = sorted(CONFIG_DIR.rglob("*.yaml"))
DRIVER_YAML = sorted(CONFIG_DIR.rglob("udp_hand_node*.yaml"))


def test_the_scan_sees_the_profiles():
    assert len(ALL_YAML) > 20
    assert {p.parent.name for p in DRIVER_YAML} >= {"p2", "p2_left"}


@pytest.mark.parametrize("path", ALL_YAML, ids=[str(p.relative_to(CONFIG_DIR)) for p in ALL_YAML])
def test_no_shipped_file_carries_the_removed_enable_device_log(path):
    assert "enable_device_log" not in path.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    "path", DRIVER_YAML, ids=[str(p.relative_to(CONFIG_DIR)) for p in DRIVER_YAML]
)
def test_every_driver_file_spells_out_enable_timing_log(path):
    params = _params(path)
    assert params is not None
    assert params.get("enable_timing_log") is True


@pytest.mark.parametrize("profile", ["p2", "p2_left"])
def test_the_exhibition_profiles_ship_logging_on(profile):
    # Turning it off is the launch's enable_logging:=false, not the shipped state.
    assert _params(CONFIG_DIR / profile / "_base.yaml")["enable_logging"] is True
