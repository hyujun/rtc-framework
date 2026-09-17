"""Pure state and persistence helpers for the GUI preset hotkey."""

from __future__ import annotations

import contextlib
import json
import os
import tempfile
from dataclasses import dataclass

DEFAULT_TOGGLE_KEYSYM = "space"
MODIFIER_KEYSYMS = frozenset(
    {
        "Alt_L",
        "Alt_R",
        "Control_L",
        "Control_R",
        "Meta_L",
        "Meta_R",
        "Shift_L",
        "Shift_R",
        "Super_L",
        "Super_R",
    }
)
TEXT_INPUT_WIDGET_CLASSES = frozenset(
    {"Entry", "Spinbox", "TCombobox", "TEntry", "TSpinbox", "Text"}
)


def normalize_keysym(value: str) -> str | None:
    """Return a stable Tk keysym for one non-modifier key."""
    if value == " ":
        return DEFAULT_TOGGLE_KEYSYM
    value = value.strip()
    if not value:
        return None
    if value.lower() == "space":
        return DEFAULT_TOGGLE_KEYSYM
    if value in MODIFIER_KEYSYMS:
        return None
    if len(value) == 1 and value.isalpha():
        return value.lower()
    return value


def format_keysym(keysym: str) -> str:
    """Return a compact operator-facing label for a normalized keysym."""
    if keysym == DEFAULT_TOGGLE_KEYSYM:
        return "Space"
    if len(keysym) == 1:
        return keysym.upper()
    return keysym


def is_text_input_widget(widget_class: str) -> bool:
    """Whether a focused Tk widget is accepting editable text."""
    return widget_class in TEXT_INPUT_WIDGET_CLASSES


def resolve_preset_pair(
    presets: dict, close_preset: str = "", open_preset: str = ""
) -> tuple[str, str]:
    """Keep valid saved choices and otherwise select the first typed presets."""
    if close_preset not in presets:
        close_preset = next(
            (name for name, data in presets.items() if data.get("type") == "close"), ""
        )
    if open_preset not in presets:
        open_preset = next(
            (name for name, data in presets.items() if data.get("type") == "open"), ""
        )
    return close_preset, open_preset


def toggle_mapping_warning(close_preset: str, open_preset: str) -> str | None:
    """Operator warning for a mapping that makes the toggle a no-op."""
    if close_preset and close_preset == open_preset:
        return "Close and Open use the same preset"
    return None


@dataclass
class PresetToggleState:
    """Close-first state machine that advances only after accepted dispatch."""

    next_role: str = "close"
    armed: bool = True

    def press(self) -> str | None:
        if not self.armed:
            return None
        self.armed = False
        return self.next_role

    def commit(self, role: str) -> None:
        if role == self.next_role:
            self.next_role = "open" if role == "close" else "close"

    def rearm(self) -> None:
        self.armed = True


def default_toggle_settings() -> dict:
    return {
        "preset_toggle": {
            "keysym": DEFAULT_TOGGLE_KEYSYM,
            "close_preset": "",
            "open_preset": "",
        }
    }


def normalize_toggle_settings(data: object) -> dict:
    """Validate persisted settings and fill missing fields with defaults."""
    settings = default_toggle_settings()
    if not isinstance(data, dict):
        return settings
    raw_toggle = data.get("preset_toggle")
    if not isinstance(raw_toggle, dict):
        return settings
    keysym = normalize_keysym(str(raw_toggle.get("keysym", "")))
    if keysym is not None:
        settings["preset_toggle"]["keysym"] = keysym
    for field in ("close_preset", "open_preset"):
        value = raw_toggle.get(field)
        if isinstance(value, str):
            settings["preset_toggle"][field] = value
    return settings


def load_toggle_settings(path: str) -> dict:
    try:
        with open(path) as settings_file:
            return normalize_toggle_settings(json.load(settings_file))
    except (OSError, json.JSONDecodeError):
        return default_toggle_settings()


def save_toggle_settings(path: str, settings: dict) -> None:
    """Atomically persist normalized settings beside the preset store."""
    normalized = normalize_toggle_settings(settings)
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    file_descriptor, temporary_path = tempfile.mkstemp(
        dir=directory, prefix=".demo_gui_settings_", suffix=".json"
    )
    try:
        with os.fdopen(file_descriptor, "w") as settings_file:
            json.dump(normalized, settings_file, indent=2)
            settings_file.write("\n")
        os.replace(temporary_path, path)
    except Exception:
        with contextlib.suppress(OSError):
            os.unlink(temporary_path)
        raise
