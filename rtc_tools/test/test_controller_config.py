"""Unit tests for :mod:`rtc_tools.utils.controller_config`.

The module is the Python twin of ``rtc_controller_manager``'s
``LoadControllerConfig``. What keeps the two on one tree is a shared fixture:
``rtc_controller_manager/config/test_include/`` holds a config split over a main
file and two fragments, and ``expected_leaves.txt`` beside it is the composed
tree. The C++ loader is checked against that file in
``test_controller_config_loader.cpp``; this module checks the Python loader
against the same file.

The fixture is read from the source tree, not through ``ament_index`` — these
tests must run with only ``rtc_tools`` installed.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from rtc_tools.utils.controller_config import (
    ControllerConfigIncludeError,
    controller_config_leaf_lines,
    load_controller_config,
)

_CM_CONFIG = Path(__file__).resolve().parents[2] / "rtc_controller_manager" / "config"
_KEY = "ctrl"


def _write(root: Path, files: dict[str, str]) -> Path:
    for relative, text in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return root / "main.yaml"


def _leaves(main: Path) -> list[str]:
    doc = load_controller_config(main, loader=yaml.BaseLoader)
    return controller_config_leaf_lines(doc[_KEY])


# ── The shared fixture: same tree as the C++ loader ──────────────────────────


def test_composed_tree_matches_the_expected_leaves_the_cpp_loader_is_held_to():
    variant = _CM_CONFIG / "test_include"
    expected = (variant / "expected_leaves.txt").read_text().splitlines()
    assert expected, "the shared fixture is empty — the comparison would be vacuous"

    doc = load_controller_config(
        variant / "controllers" / "rtc_cm_cfg_test.yaml", loader=yaml.BaseLoader
    )

    # `include` is consumed: what is left is the controller's key alone.
    assert list(doc) == ["rtc_cm_cfg_test"]
    assert controller_config_leaf_lines(doc["rtc_cm_cfg_test"]) == expected


def test_default_loader_gives_typed_values_from_every_file():
    doc = load_controller_config(_CM_CONFIG / "test_include/controllers/rtc_cm_cfg_test.yaml")
    tree = doc["rtc_cm_cfg_test"]

    assert tree["gains"]["kp"] == 1.0  # main file
    assert tree["gains"]["enabled"] is True  # first fragment
    assert tree["gains"]["vals"] == [0.5, 1.0e-4]
    assert tree["deep"]["a"] == {"b": 2.5, "note": None}  # second fragment
    # Order: the main file's keys, then each fragment's new keys in include order.
    assert list(tree) == ["gains", "topics", "tail_a", "deep", "tail_b"]
    assert list(tree["gains"]) == ["kp", "label", "enabled", "count", "vals", "tags"]


# ── Files without `include:` are returned as read ────────────────────────────


def test_a_file_without_include_is_returned_as_read():
    path = _CM_CONFIG / "test_fixtures/controllers/rtc_cm_cfg_test.yaml"
    assert load_controller_config(path) == yaml.safe_load(path.read_text())


def test_a_file_without_include_keeps_every_top_level_key(tmp_path):
    # Readers scan controllers/*.yaml for whatever top-level keys are there.
    main = _write(tmp_path, {"main.yaml": "a:\n  x: 1\nb:\n  y: 2\n"})
    assert load_controller_config(main) == {"a": {"x": 1}, "b": {"y": 2}}


def test_an_empty_file_is_an_empty_document(tmp_path):
    assert load_controller_config(_write(tmp_path, {"main.yaml": ""})) == {}


def test_a_missing_main_file_is_an_os_error_not_an_include_error(tmp_path):
    with pytest.raises(OSError):
        load_controller_config(tmp_path / "main.yaml")


def test_fragments_resolve_against_the_including_files_directory(tmp_path):
    # A profile is copied elsewhere (copytree) and read from the copy.
    main = _write(
        tmp_path / "copy" / "controllers",
        {"main.yaml": "include: [sub/a.yaml]\nctrl:\n  x: 1\n", "sub/a.yaml": "ctrl:\n  y: 2\n"},
    )
    assert _leaves(main) == ["x\t1", "y\t2"]


# ── Every broken composition is an include error naming the files ────────────

_MAIN_X = "include: [a.yaml]\nctrl:\n  x: 1\n"

_BROKEN = {
    "missing_fragment": (
        {"main.yaml": "include: [gone.yaml]\nctrl:\n  x: 1\n"},
        ["main.yaml", "gone.yaml", "cannot be opened"],
    ),
    "leaf_in_two_fragments": (
        {
            "main.yaml": "include: [a.yaml, b.yaml]\nctrl:\n  x: 1\n",
            "a.yaml": "ctrl:\n  g:\n    y: 2\n",
            "b.yaml": "ctrl:\n  g:\n    y: 2\n",
        },
        ["'g.y'", "a.yaml", "b.yaml", "set in both"],
    ),
    "leaf_in_main_and_fragment": (
        {"main.yaml": _MAIN_X, "a.yaml": "ctrl:\n  x: 1\n"},
        ["'x'", "main.yaml", "a.yaml", "set in both"],
    ),
    "sequence_is_one_leaf": (
        {
            "main.yaml": "include: [a.yaml]\nctrl:\n  logs: [one]\n",
            "a.yaml": "ctrl:\n  logs: [two]\n",
        },
        ["'logs'", "main.yaml", "a.yaml", "set in both"],
    ),
    "map_then_leaf": (
        {"main.yaml": "include: [a.yaml]\nctrl:\n  g:\n    y: 2\n", "a.yaml": "ctrl:\n  g: 5\n"},
        ["'g'", "main.yaml", "a.yaml", "a map in one file and a value in the other"],
    ),
    "leaf_then_map": (
        {"main.yaml": "include: [a.yaml]\nctrl:\n  g: ~\n", "a.yaml": "ctrl:\n  g:\n    y: 2\n"},
        ["'g'", "main.yaml", "a.yaml", "a map in one file and a value in the other"],
    ),
    "fragment_without_config_key": (
        {"main.yaml": _MAIN_X, "a.yaml": "other_ctrl:\n  y: 2\n"},
        ["a.yaml", "'other_ctrl'", "only top-level key is 'ctrl'"],
    ),
    "fragment_not_a_map": (
        {"main.yaml": _MAIN_X, "a.yaml": "- just\n- a list\n"},
        ["a.yaml", "must be a map"],
    ),
    "fragment_config_key_not_a_map": (
        {"main.yaml": _MAIN_X, "a.yaml": "ctrl: 3\n"},
        ["a.yaml", "no map under 'ctrl'"],
    ),
    "nested_include": (
        {
            "main.yaml": _MAIN_X,
            "a.yaml": "include: [b.yaml]\nctrl:\n  y: 2\n",
            "b.yaml": "ctrl:\n  z: 3\n",
        },
        ["a.yaml", "do not nest"],
    ),
    "include_not_a_list": (
        {"main.yaml": "include: a.yaml\nctrl:\n  x: 1\n", "a.yaml": "ctrl:\n  y: 2\n"},
        ["main.yaml", "must be a list"],
    ),
    "include_entry_not_a_string": (
        {"main.yaml": "include: [{path: a.yaml}]\nctrl:\n  x: 1\n"},
        ["main.yaml", "must be a path string"],
    ),
    "main_with_another_top_level_key": (
        {"main.yaml": "include: []\nctrl:\n  x: 1\nstray:\n  y: 2\n"},
        ["main.yaml", "stray"],
    ),
    "main_without_its_config_key_map": (
        {"main.yaml": "include: [a.yaml]\nctrl: 3\n", "a.yaml": "ctrl:\n  y: 2\n"},
        ["main.yaml", "no map under 'ctrl'"],
    ),
    "fragment_does_not_parse": (
        {"main.yaml": _MAIN_X, "a.yaml": "ctrl:\n  y: [1, 2\n"},
        ["a.yaml", "main.yaml", "does not parse"],
    ),
}


@pytest.mark.parametrize("case", sorted(_BROKEN))
def test_a_broken_composition_is_an_include_error_naming_the_files(tmp_path, case):
    files, needles = _BROKEN[case]
    main = _write(tmp_path, files)

    with pytest.raises(ControllerConfigIncludeError) as excinfo:
        load_controller_config(main)

    # The message, not just the type: a fixture typo failing for its own reason
    # would satisfy a bare `raises`.
    message = str(excinfo.value)
    for needle in needles:
        assert needle in message, f"message lacks {needle!r}: {message}"


def test_an_absolute_include_path_is_rejected(tmp_path):
    # The target exists and is a valid fragment: only the path form is wrong.
    target = tmp_path / "abs.yaml"
    target.write_text("ctrl:\n  y: 2\n")
    main = _write(tmp_path, {"main.yaml": f"include: ['{target}']\nctrl:\n  x: 1\n"})

    with pytest.raises(ControllerConfigIncludeError, match="not absolute"):
        load_controller_config(main)


def test_a_parent_directory_include_path_is_rejected(tmp_path):
    (tmp_path / "up.yaml").write_text("ctrl:\n  y: 2\n")
    main = _write(tmp_path / "cfg", {"main.yaml": "include: ['../up.yaml']\nctrl:\n  x: 1\n"})

    with pytest.raises(ControllerConfigIncludeError, match=r"cannot contain '\.\.'"):
        load_controller_config(main)


# ── Leaf lines ───────────────────────────────────────────────────────────────


def test_leaf_lines_keep_scalar_text_and_render_nulls_alike(tmp_path):
    # Same input and expectation as the C++ test of the same name.
    main = _write(
        tmp_path,
        {
            "main.yaml": 'ctrl:\n  a: 1.0e-4\n  b: "quoted"\n  c: ~\n  d:\n  e: ""\n  f: {}\n  g: []\n'
        },
    )
    assert _leaves(main) == [
        "a\t1.0e-4",
        "b\tquoted",
        "c\tnull",
        "d\tnull",
        "e\tnull",
        "f\t{}",
        "g\t[]",
    ]
