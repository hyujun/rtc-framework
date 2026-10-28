"""udp_hand.launch.py 가 hand-node yaml 과 손 목록을 해석하는 규칙.

`params_file` 로 받은 yaml 은 노드에 그대로 넘어가지만, launch 는 같은 파일에서
sil_mode · protocol_version · hand_sides 를 읽어 fake firmware 를 구성한다. 그 읽기가
조용히 실패하면 (경로 오타, `/**` 대신 노드 이름 최상위 키) 노드는 두 손으로 뜨는데
fake 는 하나 또는 없이 떠서, 증상은 한 손의 link_status 가 영영 false 인 것뿐이다.
yaml 의 hand_sides 를 launch 가 놓쳐도 같은 증상이다.

launch 파일은 설치본이 아니라 소스에서 읽는다 — 이 테스트가 보는 것은 편집 중인 규칙이다.
_launch_setup 은 launch 인자를 채운 LaunchContext 로 직접 평가한다 — 실행 중인 ROS graph 는
필요 없다.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext

_LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "udp_hand.launch.py"


@pytest.fixture(scope="module")
def launch():
    spec = importlib.util.spec_from_file_location("_udp_hand_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(tmp_path, text, name="hand.yaml"):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


_TWO_HAND_FIRMWARE = """\
/**:
  ros__parameters:
    sil_mode: firmware
    protocol_version: "2"
    hand_sides: [right, left]
    cmd_ack: none
"""


# ── hand_sides 해석 ──────────────────────────────────────────────────────────


def test_cli_hand_sides_wins_over_yaml(launch):
    assert launch._resolve_hand_sides("left", {"hand_sides": ["right", "left"]}) == ["left"]


def test_yaml_hand_sides_used_when_cli_empty(launch):
    assert launch._resolve_hand_sides("", {"hand_sides": ["right", "left"]}) == ["right", "left"]


def test_no_hand_sides_anywhere_is_one_hand(launch):
    assert launch._resolve_hand_sides("", {}) == []
    assert launch._resolve_hand_sides(" , ", {}) == []


def test_yaml_hand_sides_not_a_list_is_rejected(launch):
    with pytest.raises(RuntimeError, match="must be a list"):
        launch._resolve_hand_sides("", {"hand_sides": "right,left"})


# ── fake firmware 구성 ───────────────────────────────────────────────────────


@pytest.mark.parametrize("proto", ["1a", "1b"])
def test_udp_protocol_with_yaml_hand_sides_is_rejected(launch, proto):
    sides = launch._resolve_hand_sides("", {"hand_sides": ["right", "left"]})
    with pytest.raises(RuntimeError, match="needs protocol_version 2"):
        launch._firmware_fakes({}, {"protocol_version": proto}, sides, 42)


def test_two_hands_get_one_fake_each_with_distinct_seeds(launch):
    fakes = launch._firmware_fakes({}, {"protocol_version": "2"}, ["right", "left"], 42)
    assert [name for name, _ in fakes] == ["fake_hand_firmware_right", "fake_hand_firmware_left"]
    assert [p["hand_side"] for _, p in fakes] == ["right", "left"]
    assert [p["seed"] for _, p in fakes] == [42, 43]
    assert {p["can_interface"] for _, p in fakes} == {"vcan0"}


def test_cli_override_beats_yaml_for_fake_cmd_ack(launch):
    fakes = launch._firmware_fakes(
        {"cmd_ack": "echo"}, {"protocol_version": "2", "cmd_ack": "none"}, [], 42
    )
    assert fakes == [
        (
            "fake_hand_firmware",
            {
                "protocol_version": "2",
                "can_interface": "vcan0",
                "cmd_ack": "echo",
                "hand_side": "right",
            },
        )
    ]


# ── params_file 해석 · 로드 ──────────────────────────────────────────────────


def test_empty_params_file_is_the_generic_yaml(launch, tmp_path):
    assert launch._resolve_hand_config(str(tmp_path), "") == str(
        tmp_path / "config" / "udp_hand_node.yaml"
    )


def test_missing_params_file_is_rejected(launch, tmp_path):
    with pytest.raises(RuntimeError, match="does not exist"):
        launch._resolve_hand_config(str(tmp_path), str(tmp_path / "nope.yaml"))


def test_strict_load_rejects_unparsable_yaml(launch, tmp_path):
    path = _write(tmp_path, "/**:\n  ros__parameters: [unclosed\n")
    with pytest.raises(RuntimeError, match="unreadable"):
        launch._load_ros_params(path, strict=True)


@pytest.mark.parametrize(
    "text",
    [
        "/hand/udp_hand_node:\n  ros__parameters:\n    sil_mode: firmware\n",
        "/**:\n  sil_mode: firmware\n",
        "",
    ],
    ids=["node-name-key", "no-ros__parameters", "empty-file"],
)
def test_strict_load_rejects_missing_wildcard_mapping(launch, tmp_path, text):
    path = _write(tmp_path, text)
    with pytest.raises(RuntimeError, match="/\\*\\*: ros__parameters"):
        launch._load_ros_params(path, strict=True)


def test_lenient_load_still_swallows_failures(launch, tmp_path):
    # 설치된 generic · fake firmware yaml 의 기존 동작 (fallback 기본값으로 진행).
    assert launch._load_ros_params(str(tmp_path / "nope.yaml")) == {}
    assert launch._load_ros_params(_write(tmp_path, "/hand/x:\n  a: 1\n")) == {}


# ── _launch_setup 평가 ───────────────────────────────────────────────────────


def _context(launch, **args):
    configs = {
        decl.name: decl.default_value[0].text if decl.default_value else ""
        for decl in launch.generate_launch_description().entities
        if hasattr(decl, "default_value")
    }
    unknown = set(args) - set(configs)
    assert not unknown, f"undeclared launch args {unknown}"
    configs.update(args)
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


def test_params_file_hand_sides_spawn_one_fake_per_hand(launch, tmp_path, monkeypatch):
    # fake 는 launch_ros Node 로 만들어진다 — 실행 전에는 이름을 읽을 수 없어 생성 인자를 받아 둔다.
    spawned = []
    monkeypatch.setattr(launch, "Node", lambda **kwargs: spawned.append(kwargs))
    monkeypatch.setattr(launch, "_check_fd_vcan", lambda interface: None)
    path = _write(tmp_path, _TWO_HAND_FIRMWARE)
    launch._launch_setup(_context(launch, params_file=path, namespace="/p2"))
    assert [k["name"] for k in spawned] == ["fake_hand_firmware_right", "fake_hand_firmware_left"]


def test_params_file_without_wildcard_fails_the_launch(launch, tmp_path):
    path = _write(tmp_path, _TWO_HAND_FIRMWARE.replace("/**:", "/p2/udp_hand_node:"))
    with pytest.raises(RuntimeError, match="/\\*\\*: ros__parameters"):
        launch._launch_setup(_context(launch, params_file=path))
