"""catching_throw_list — the one throw-list format (``catching_throw_list/1``).

Pinned from the rtc_tools side, where the format lives: a written file reads
back bit for bit and in order, every refusal names the file and the value, a
reserved trial-record key is refused, and one read gives the file's identity.
The sim driver's own tests pin that it throws through these functions.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from rtc_tools.analysis import catching_throw_list as ctl


def _throws():
    return [
        {
            "throw_id": 7,
            "kind": "map",
            "pos": (0.1 + 0.2, 1e-17, -3.5),
            "vel": (1.0 / 3.0, 2.0, 5e-324),
            "omega": (0.0, 1.0e-9, 123456.789),
            "score": 0.25,
            "nested": {"a": [1, 2]},
        },
        {
            "throw_id": 2,
            "kind": "list",
            "pos": (0.0, 0.0, 1.0),
            "vel": (4.0, 0.0, 1.0),
            "omega": (0.0, 0.0, 0.0),
        },
    ]


def _doc(entry_over=None, **over):
    entry = {"throw_id": 0, "pos": [0, 0, 1], "vel": [1, 2, 3]}
    entry.update(entry_over or {})
    doc = {"schema": "catching_throw_list/1", "frame": "sim_world", "meta": {}, "throws": [entry]}
    doc.update(over)
    return doc


def _file(tmp_path: Path, doc) -> Path:
    path = tmp_path / "list.json"
    path.write_text(json.dumps(doc), encoding="utf-8")
    return path


def test_round_trip_is_bit_exact_ordered_and_keeps_extras(tmp_path):
    path = tmp_path / "out.json"
    ctl.write_throw_list(path, _throws(), {"tool": "unit", "n": 2})
    throws, meta = ctl.load_throw_list(path)
    assert throws == _throws()
    assert meta == {"tool": "unit", "n": 2}
    assert [t["throw_id"] for t in throws] == [7, 2]
    assert throws[0]["pos"] == (0.1 + 0.2, 1e-17, -3.5)
    assert throws[0]["vel"][2] == 5e-324
    # the known keys first, in one order, then the entry's own keys as written
    assert list(throws[0]) == ["throw_id", "kind", "pos", "vel", "omega", "score", "nested"]
    doc = json.loads(path.read_text())
    assert (doc["schema"], doc["frame"]) == (ctl.THROW_LIST_SCHEMA, ctl.THROW_LIST_FRAME)
    assert (ctl.THROW_LIST_SCHEMA, ctl.THROW_LIST_FRAME) == ("catching_throw_list/1", "sim_world")


def test_defaults(tmp_path):
    (throw,), meta = ctl.load_throw_list(_file(tmp_path, {**_doc(), "meta": {}}))
    assert meta == {}
    assert throw == {
        "throw_id": 0,
        "kind": "list",
        "pos": (0.0, 0.0, 1.0),
        "vel": (1.0, 2.0, 3.0),
        "omega": (0.0, 0.0, 0.0),
    }
    no_meta = _doc()
    del no_meta["meta"]
    assert ctl.parse_throw_list(no_meta, "src")[1] == {}


@pytest.mark.parametrize(
    ("doc", "needle"),
    [
        ([], "top level"),
        (_doc(schema="catching_throw_list/2"), "catching_throw_list/2"),
        (_doc(frame="model_world"), "'model_world'"),
        (_doc(meta=[]), "meta"),
        (_doc(throws=[]), "non-empty"),
        (_doc(throws="x"), "non-empty"),
        (_doc(throws=[3]), r"throws\[0\] must be an object"),
    ],
)
def test_header_refusals_name_the_source_and_the_value(doc, needle):
    with pytest.raises(ValueError, match=needle) as exc:
        ctl.parse_throw_list(doc, "the-source")
    assert "the-source" in str(exc.value)


@pytest.mark.parametrize(
    ("entry", "needle"),
    [
        ({"throw_id": None}, "throw_id"),
        ({"throw_id": True}, "throw_id"),
        ({"throw_id": 1.0}, "throw_id"),
        ({"pos": [0, 0]}, "pos"),
        ({"vel": [0, 0, 0, 0]}, "vel"),
        ({"vel": "abc"}, "vel"),
        ({"pos": [0, True, 1]}, "True"),
        ({"pos": [0, "1", 1]}, "'1'"),
        ({"pos": [0, float("nan"), 1]}, "finite"),
        ({"omega": [0, 0, None]}, "omega"),
        ({"omega": [0, 0, float("inf")]}, "finite"),
        ({"kind": 3}, "kind"),
    ],
)
def test_entry_refusals(entry, needle):
    with pytest.raises(ValueError, match=needle) as exc:
        ctl.parse_throw_list(_doc(entry), "src")
    assert "throws[0]" in str(exc.value)


@pytest.mark.parametrize("key", sorted(ctl.THROW_RECORD_KEYS))
def test_every_key_the_trial_record_writes_is_refused(key):
    with pytest.raises(ValueError, match=key):
        ctl.parse_throw_list(_doc({key: 1}), "src")


def test_missing_keys_and_a_duplicated_id():
    for key in ("throw_id", "pos", "vel"):
        doc = _doc()
        del doc["throws"][0][key]
        with pytest.raises(ValueError, match=key):
            ctl.parse_throw_list(doc, "src")
    entry = {"throw_id": 4, "pos": [0, 0, 1], "vel": [1, 2, 3]}
    with pytest.raises(ValueError, match="duplicated"):
        ctl.parse_throw_list(_doc(throws=[entry, dict(entry)]), "src")


@pytest.mark.parametrize("token", ["NaN", "Infinity", "-Infinity"])
def test_a_non_finite_json_token_is_refused(tmp_path, token):
    path = tmp_path / "nf.json"
    body = json.dumps(_doc()).replace('"vel": [1, 2, 3]', f'"vel": [1, {token}, 3]')
    path.write_text(body, encoding="utf-8")
    with pytest.raises(ValueError, match="finite"):
        ctl.load_throw_list(path)


def test_a_file_that_is_not_json_or_not_there(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("{", encoding="utf-8")
    with pytest.raises(ValueError, match="bad.json"):
        ctl.load_throw_list(bad)
    bad.write_bytes(b"\xff\xfe")
    with pytest.raises(ValueError, match="UTF-8"):
        ctl.load_throw_list(bad)
    with pytest.raises(OSError):
        ctl.load_throw_list(tmp_path / "absent.json")


def test_the_writer_refuses_what_the_reader_would_before_writing(tmp_path):
    path = tmp_path / "w.json"
    for bad in (
        [{"throw_id": 1, "pos": (0.0, 0.0), "vel": (0.0, 0.0, 0.0)}],
        [{"throw_id": 1, "pos": (0.0, 0.0, 1.0), "vel": (0.0, 0.0, 0.0), "seed": 3}],
        [{"throw_id": 1, "pos": (0.0, 0.0, 1.0), "vel": (0.0, 0.0, 0.0), "kind": 2}],
        [],
    ):
        with pytest.raises(ValueError):
            ctl.write_throw_list(path, bad, {})
        assert not path.exists()


def test_one_read_gives_the_throws_and_the_files_identity(tmp_path):
    path = tmp_path / "t.json"
    ctl.write_throw_list(path, _throws(), {"source": "unit"})
    listed = ctl.read_throw_list_file(path)
    assert listed.throws == _throws()
    assert listed.meta == {"source": "unit"}
    assert listed.path == str(path.resolve())
    assert listed.sha256 == hashlib.sha256(path.read_bytes()).hexdigest()
    assert listed.record() == {
        "path": str(path.resolve()),
        "sha256": listed.sha256,
        "n_throws": 2,
        "meta": {"source": "unit"},
    }
    assert ctl.load_throw_list(path) == (listed.throws, listed.meta)
