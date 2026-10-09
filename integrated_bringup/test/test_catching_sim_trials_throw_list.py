"""catching_sim_trials — the throw list file (``--throws-file``).

What must hold for an offline tool and the sim to throw the same launches: the
writer's file reads back bit for bit and in order, the loader refuses what a
trial record or the join key could not carry, and the file replaces ``--dist``
without touching what ``--dist`` builds.
"""

from __future__ import annotations

import json

import pytest

from integrated_bringup import catching_sim_trials as cst


def _throws():
    return [
        {
            "throw_id": 7,
            "kind": "map",
            "pos": (0.1 + 0.2, 1e-17, -3.5),
            "vel": (1.0 / 3.0, 2.0, 5e-324),
            "omega": (0.0, 1.0e-9, 123456.789),
            "score": 0.25,
        },
        {
            "throw_id": 2,
            "kind": "list",
            "pos": (0.0, 0.0, 1.0),
            "vel": (4.0, 0.0, 1.0),
            "omega": (0.0, 0.0, 0.0),
        },
    ]


def _doc(**over):
    entry = {"throw_id": 0, "pos": [0, 0, 1], "vel": [1, 2, 3]}
    doc = {"schema": "catching_throw_list/1", "frame": "sim_world", "meta": {}, "throws": [entry]}
    doc.update(over)
    return doc


def _file(tmp_path, doc):
    path = tmp_path / "list.json"
    path.write_text(json.dumps(doc), encoding="utf-8")
    return str(path)


def _entry_file(tmp_path, **entry):
    base = {"throw_id": 0, "pos": [0, 0, 1], "vel": [1, 2, 3]}
    base.update(entry)
    return _file(tmp_path, _doc(throws=[base]))


def test_round_trip_is_bit_exact_and_ordered(tmp_path):
    path = str(tmp_path / "out.json")
    meta = {"source": "unit", "n": 2, "nested": {"a": [1, 2]}}
    cst.write_throw_list(path, _throws(), meta)
    throws, got_meta = cst.load_throw_list(path)
    assert throws == _throws()
    assert got_meta == meta
    assert [t["throw_id"] for t in throws] == [7, 2]
    assert throws[0]["pos"][0] == 0.1 + 0.2
    assert throws[0]["pos"][1] == 1e-17


def test_defaults_and_extras(tmp_path):
    path = _entry_file(tmp_path, tag="x", nested={"k": [1]})
    (throw,), meta = cst.load_throw_list(path)
    assert meta == {}
    assert throw["omega"] == (0.0, 0.0, 0.0)
    assert throw["kind"] == "list"
    assert throw["pos"] == (0.0, 0.0, 1.0)
    assert throw["tag"] == "x"
    assert throw["nested"] == {"k": [1]}


@pytest.mark.parametrize(
    ("doc", "needle"),
    [
        (_doc(schema="catching_throw_list/2"), "catching_throw_list/2"),
        (_doc(frame="world"), "'world'"),
        (_doc(throws=[]), "non-empty"),
        (_doc(throws="x"), "non-empty"),
        (_doc(meta=[]), "meta"),
    ],
)
def test_header_rejections_name_the_file_and_value(tmp_path, doc, needle):
    path = _file(tmp_path, doc)
    with pytest.raises(ValueError) as exc:
        cst.load_throw_list(path)
    assert path in str(exc.value)
    assert needle in str(exc.value)


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
        ({"omega": [0, 0, None]}, "omega"),
        ({"kind": 3}, "kind"),
        ({"outcome": "x"}, "outcome"),
        ({"idx": 3}, "idx"),
        ({"seed": 3}, "seed"),
        ({"controller_mirror": {}}, "controller_mirror"),
    ],
)
def test_entry_rejections(tmp_path, entry, needle):
    path = _entry_file(tmp_path, **entry)
    with pytest.raises(ValueError, match=needle) as exc:
        cst.load_throw_list(path)
    assert path in str(exc.value)


def test_missing_keys_and_duplicate_id(tmp_path):
    for key in ("throw_id", "pos", "vel"):
        entry = {"throw_id": 0, "pos": [0, 0, 1], "vel": [1, 2, 3]}
        del entry[key]
        with pytest.raises(ValueError, match=key):
            cst.load_throw_list(_file(tmp_path, _doc(throws=[entry])))
    dup = [
        {"throw_id": 4, "pos": [0, 0, 1], "vel": [1, 2, 3]},
        {"throw_id": 4, "pos": [0, 0, 1], "vel": [1, 2, 3]},
    ]
    with pytest.raises(ValueError, match="duplicated"):
        cst.load_throw_list(_file(tmp_path, _doc(throws=dup)))


@pytest.mark.parametrize("token", ["NaN", "Infinity", "-Infinity"])
def test_non_finite_is_refused(tmp_path, token):
    path = tmp_path / "nf.json"
    body = json.dumps(_doc()).replace('"vel": [1, 2, 3]', f'"vel": [1, {token}, 3]')
    path.write_text(body, encoding="utf-8")
    with pytest.raises(ValueError, match="finite"):
        cst.load_throw_list(str(path))


def test_not_json_is_refused(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{", encoding="utf-8")
    with pytest.raises(ValueError, match="bad.json"):
        cst.load_throw_list(str(path))


def test_writer_refuses_what_the_loader_would(tmp_path):
    bad = [{"throw_id": 1, "pos": (0.0, 0.0), "vel": (0.0, 0.0, 0.0)}]
    path = tmp_path / "w.json"
    with pytest.raises(ValueError, match="pos"):
        cst.write_throw_list(str(path), bad, {})
    assert not path.exists()


def test_build_throws_returns_the_file_whatever_the_profile(tmp_path):
    path = str(tmp_path / "t.json")
    cst.write_throw_list(path, _throws(), {})
    args = cst.parse_args(["out", "--throws-file", path])
    assert args.dist is None
    for profile in ("ur5e_p1b", "iiwa7_leap", "anything"):
        assert cst.build_throws(args, profile) == _throws()


def test_throws_file_and_dist_conflict(capsys):
    with pytest.raises(SystemExit):
        cst.parse_args(["out", "--throws-file", "f.json", "--dist", "s35b"])
    assert "--throws-file" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        cst.parse_args(["out", "--throws-file", "f.json", "--dist", "reference"])


def test_run_meta_record(tmp_path):
    import hashlib
    import os
    import pathlib

    path = str(tmp_path / "t.json")
    cst.write_throw_list(path, _throws(), {"source": "unit"})
    rec = cst.throw_list_record(path)
    assert rec["path"] == os.path.abspath(path)
    assert rec["sha256"] == hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
    assert rec["n_throws"] == 2
    assert rec["meta"] == {"source": "unit"}


def test_dist_series_are_unchanged():
    args = cst.parse_args(["out"])
    assert args.dist == "reference"
    assert args.throws_file is None
    assert cst.build_throws(args, "ur5e_p1b") == cst.trial_throws(
        args.n_ref, args.n_pert, args.seed, args.release_pos, args.release_vel
    )
    explicit = cst.parse_args(["out", "--dist", "reference", "--n-ref", "2", "--n-pert", "3"])
    assert cst.build_throws(explicit, "ur5e_p1b") == cst.trial_throws(
        2, 3, explicit.seed, explicit.release_pos, explicit.release_vel
    )
    args = cst.parse_args(["out", "--dist", "s35b", "--n", "4", "--seed", "9"])
    assert cst.build_throws(args, "ur5e_p1b") == cst.frozen_throws("s35b", "ur5e_p1b", 4, 9)


def test_the_format_is_rtc_tools_catching_throw_list(tmp_path):
    from rtc_tools.analysis import catching_throw_list as ctl

    for name in ("THROW_LIST_SCHEMA", "THROW_LIST_FRAME", "THROW_RECORD_KEYS"):
        assert getattr(cst, name) is getattr(ctl, name)
    # a list the shared writer wrote is the list this driver throws, and the
    # reverse — and an entry one side refuses the other refuses as well
    path = str(tmp_path / "t.json")
    ctl.write_throw_list(path, _throws(), {"tool": "x"})
    assert cst.load_throw_list(path) == ctl.load_throw_list(path)
    cst.write_throw_list(path, _throws(), {"tool": "y"})
    assert ctl.load_throw_list(path) == (_throws(), {"tool": "y"})
    bad = _doc(throws=[{"throw_id": 0, "pos": [0, 0, 1], "vel": [1, 2, 3], "kind": 3}])
    for parse in (cst.parse_throw_list, ctl.parse_throw_list):
        with pytest.raises(ValueError, match="kind"):
            parse(bad, "src")


@pytest.mark.parametrize(
    "content",
    [None, "{", json.dumps(_doc(throws=[{"throw_id": 0, "pos": [0, 0], "vel": [1, 2, 3]}]))],
)
def test_a_refused_file_ends_the_run_before_anything_is_written(tmp_path, capsys, content):
    path = tmp_path / "list.json"
    if content is not None:
        path.write_text(content, encoding="utf-8")
    out = tmp_path / "out"
    with pytest.raises(SystemExit) as exc:
        cst.main([str(out), "--throws-file", str(path)])
    assert exc.value.code == 2  # an argument error, raised before rclpy is touched
    assert str(path) in capsys.readouterr().err
    assert not out.exists()


def test_the_file_is_read_once_when_the_arguments_are_parsed(tmp_path):
    path = tmp_path / "t.json"
    cst.write_throw_list(str(path), _throws(), {"source": "unit"})
    args = cst.parse_args(["out", "--throws-file", str(path)])
    record = cst.throw_list_record(str(path))
    path.unlink()
    # the series and the run_meta record come from that one read
    assert cst.build_throws(args, "ur5e_p1b") == _throws()
    assert args.throw_list.record() == record


def test_a_file_run_records_no_seed():
    import pathlib
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        path = str(pathlib.Path(tmp) / "t.json")
        cst.write_throw_list(path, _throws(), {})
        args = cst.parse_args(["out", "--throws-file", path])
        assert args.seed is None
        meta_args = cst.run_meta_args(args)
        assert meta_args["seed"] is None and meta_args["throws_file"] == path
        assert "throw_list" not in meta_args
        json.dumps(meta_args)  # what run_meta.json is written from
        with pytest.raises(SystemExit):
            cst.parse_args(["out", "--throws-file", path, "--seed", "7"])
    # a --dist run keeps its seed, given or default
    assert cst.parse_args(["out"]).seed == cst.DEFAULT_SEED == 42
    assert cst.run_meta_args(cst.parse_args(["out", "--dist", "s35b", "--seed", "9"]))["seed"] == 9
