"""Standard gamepad layout: name <-> Joy array index.

The table lives in ``config/standard_gamepad.yaml`` (the single source); this
module only loads and checks it. Every Joy this package publishes has exactly
``len(axes)`` axes and ``len(buttons)`` buttons, in that order.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

LAYOUT_FILE = "standard_gamepad.yaml"


class LayoutError(ValueError):
    """The layout file is missing, malformed or ambiguous."""


@dataclass(frozen=True)
class GamepadLayout:
    axes: tuple[str, ...]
    buttons: tuple[str, ...]

    def axis_index(self, name: str) -> int:
        try:
            return self.axes.index(name)
        except ValueError:
            raise LayoutError(
                f"unknown axis name '{name}' (axes: {', '.join(self.axes)})"
            ) from None

    def button_index(self, name: str) -> int:
        try:
            return self.buttons.index(name)
        except ValueError:
            raise LayoutError(
                f"unknown button name '{name}' (buttons: {', '.join(self.buttons)})"
            ) from None


def parse_layout(data: object) -> GamepadLayout:
    if not isinstance(data, dict) or set(data) != {"axes", "buttons"}:
        raise LayoutError("layout must be a mapping with exactly the keys 'axes' and 'buttons'")
    lists = {}
    for key in ("axes", "buttons"):
        names = data[key]
        if not isinstance(names, list) or not names:
            raise LayoutError(f"layout '{key}' must be a non-empty list")
        if not all(isinstance(n, str) and n for n in names):
            raise LayoutError(f"layout '{key}' entries must be non-empty strings")
        lists[key] = tuple(names)
    every = lists["axes"] + lists["buttons"]
    if len(set(every)) != len(every):
        raise LayoutError("layout names must be unique across axes and buttons")
    return GamepadLayout(axes=lists["axes"], buttons=lists["buttons"])


def layout_path() -> Path:
    """The installed layout file; the source tree's copy when not installed."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        return Path(get_package_share_directory("rtc_teleop_input")) / "config" / LAYOUT_FILE
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / "config" / LAYOUT_FILE


def load_layout(path: str | Path | None = None) -> GamepadLayout:
    path = Path(path) if path is not None else layout_path()
    try:
        text = path.read_text()
    except OSError as exc:
        raise LayoutError(f"cannot read layout file {path}: {exc}") from exc
    return parse_layout(yaml.safe_load(text))
