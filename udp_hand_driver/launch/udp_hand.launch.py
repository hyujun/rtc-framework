# udp_hand.launch.py -- UdpHandNode 런치 (단일 yaml 입력, SIL 진입점 단일화)
# Event-driven request-response polling 기반 통합 핸드 UDP 노드.
#
# 입력 config 는 `config/udp_hand_node.yaml` 1개(+ 예시용 fingertip_ft_inferencer.yaml).
# SIL 모드는 그 yaml 의 `sil_mode` 파라미터가 SSoT 이고, 노드 C++ on_configure 가
# 해석한다 (use_fake_hand / target_ip 파생). launch 인자는 yaml 값의 override 로만
# 작동한다 — 미지정(빈 문자열)이면 yaml 값을 그대로 쓴다.
#
#   sil_mode (off/loopmodel/firmware):
#     off       — 실 하드웨어. socket → target_ip.
#     loopmodel — 노드 in-process 1차 LPF 모델(소켓 미오픈).
#     firmware  — device-side fake_hand_firmware 를 loopback 으로. 노드가 target_ip 를
#                 127.0.0.1 로 강제하고, 이 launch 가 firmware 프로세스를 조건부로 띄운다
#                 (프로세스 spawn 은 launch 전용 — 노드는 peer 프로세스를 못 띄운다).
#
# `sil_mode:=firmware` 처럼 CLI 로 준 값이 yaml 의 sil_mode 를 이긴다.
#
# ── 지원 범위 (issue #345) ────────────────────────────────────────────────
# 이 런치는 **프로세스 내부 pin 만** 다룬다 — 노드가 스스로 main 스레드를
# hand_driver slot 에, hand_aux_io 를 aux slot 에 붙이고, `use_cpu_affinity`
# 로 그것을 끌 수 있다. integrated_bringup 의 robot 런치에 있는 것들 —
# cset shield, CM adopt, DDS 스레드 co-pin — 은 **여기 없다.** 즉 이 런치로
# 띄운 프로세스는 shield 밖에서 돌고, DDS 스레드는 어디로든 흩어진다.
# 실기 RT 구성은 robot_ur5e_p1{a,b}.launch.py 를 쓸 것. 이 런치는 개발 PC
# 에서 노드 하나만 띄워 보는 용도다.
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

VALID_SIL_MODES = ("off", "loopmodel", "firmware")

# yaml 미기재 시 firmware peer 구성에 쓰는 fallback (노드 C++ declare 기본과 일치).
_FW_PORT_DEFAULT = 55151
_FW_PROTO_DEFAULT = "1a"


# launch arg → (yaml key, python caster). 빈 문자열이면 override 하지 않음(yaml 우선).
def _as_bool(val):
    """launch 문자열 → bool.

    `bool("false")` 는 True 이므로 직접 매핑한다. YAML 1.1 이 "off"/"on" 도 boolean
    으로 읽는 것과 맞춰 두어, sil_mode 와 달리 여기서는 그 coercion 이 의도된 것이다.
    """
    lowered = val.strip().lower()
    if lowered in ("true", "1", "yes", "on"):
        return True
    if lowered in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"invalid boolean launch value '{val}' (expected true/false)")


_OVERRIDE_ARGS = [
    ("target_ip", str),
    ("target_port", int),
    ("local_ip", str),
    ("local_interface", str),
    ("publish_rate", float),
    ("communication_mode", str),
    ("recv_timeout_ms", float),
    ("protocol_version", str),
    ("use_cpu_affinity", _as_bool),
]


def _load_ros_params(yaml_path):
    """hand-node yaml(`/**` wildcard)의 ros__parameters dict 반환 (실패 시 {})."""
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("/**", {}).get("ros__parameters", {}) or {}
    except (OSError, yaml.YAMLError):
        return {}


def _launch_setup(context):
    pkg_share = get_package_share_directory("udp_hand_driver")
    hand_config = os.path.join(pkg_share, "config", "udp_hand_node.yaml")
    # 예시용(enabled:false) F/T inferencer config — 실제 추론은 off 지만 튜닝 템플릿.
    ft_config = os.path.join(pkg_share, "config", "fingertip_ft_inferencer.yaml")
    firmware_config = os.path.join(pkg_share, "config", "fake_hand_firmware.yaml")
    yaml_params = _load_ros_params(hand_config)

    # sil_mode: CLI(빈 문자열 아님) 우선, 아니면 yaml, 아니면 off.
    cli_sil = LaunchConfiguration("sil_mode").perform(context)
    eff_sil = cli_sil if cli_sil else str(yaml_params.get("sil_mode", "off"))
    if eff_sil not in VALID_SIL_MODES:
        raise RuntimeError(f"invalid sil_mode '{eff_sil}' (expected one of {VALID_SIL_MODES})")

    # 노드 파라미터 = [단일 yaml, 예시 ft yaml] + {CLI 로 명시된 override 만}.
    # sil_mode / target_ip 파생은 노드 on_configure 가 담당하므로 여기선 주입하지 않는다.
    overrides = {}
    if cli_sil:
        overrides["sil_mode"] = cli_sil
    for key, caster in _OVERRIDE_ARGS:
        val = LaunchConfiguration(key).perform(context)
        if val:
            overrides[key] = caster(val)

    node_params = [hand_config, ft_config]
    if overrides:
        node_params.append(overrides)

    udp_hand_node = LifecycleNode(
        package="udp_hand_driver",
        executable="udp_hand_node",
        name="udp_hand_node",
        namespace="",
        output="screen",
        parameters=node_params,
        emulate_tty=True,
    )

    actions = []

    # ── Device-side firmware simulator (sil_mode=firmware 일 때만) ──────────
    # target_port 를 bind 하고 선택된 protocol_version 을 echo → 노드와 wire 일치.
    if eff_sil == "firmware":
        fw_port = int(
            overrides.get("target_port", yaml_params.get("target_port", _FW_PORT_DEFAULT))
        )
        fw_proto = str(
            overrides.get(
                "protocol_version", yaml_params.get("protocol_version", _FW_PROTO_DEFAULT)
            )
        )
        actions.append(
            Node(
                package="udp_hand_driver",
                executable="fake_hand_firmware",
                name="fake_hand_firmware",
                namespace="",
                output="screen",
                parameters=[firmware_config, {"port": fw_port, "protocol_version": fw_proto}],
                emulate_tty=True,
            )
        )

    actions.append(udp_hand_node)

    # ── Auto-configure → auto-activate chain ─────────────────────────
    actions.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=udp_hand_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=lambda n: n == udp_hand_node,
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
    )
    actions.append(
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda n: n == udp_hand_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )
    )
    return actions


def generate_launch_description():
    # 모든 인자는 override-only: 빈 문자열 default 이면 yaml 값을 그대로 사용한다.
    # (choices= 를 쓰면 빈 문자열 default 가 거부되므로 sil_mode 검증은 _launch_setup 에서.)
    override_arg_decls = [
        DeclareLaunchArgument(
            "sil_mode",
            default_value="",
            description="SIL mode override (off|loopmodel|firmware). 빈 값이면 yaml 의 sil_mode 사용.",
        ),
        DeclareLaunchArgument(
            "target_ip", default_value="", description="yaml target_ip override (빈 값=yaml)"
        ),
        DeclareLaunchArgument(
            "target_port", default_value="", description="yaml target_port override (빈 값=yaml)"
        ),
        DeclareLaunchArgument(
            "local_ip",
            default_value="",
            description=(
                "yaml local_ip override (빈 값=yaml). 소켓 source address 를 이 IP 로 bind. "
                "빈 값을 CLI 로 줘서 yaml 의 binding 을 해제할 수는 없다 (다른 override 와 동일)."
            ),
        ),
        DeclareLaunchArgument(
            "local_interface",
            default_value="",
            description=(
                "yaml local_interface override (빈 값=yaml). SO_BINDTODEVICE 로 egress NIC 고정."
            ),
        ),
        DeclareLaunchArgument(
            "publish_rate", default_value="", description="yaml publish_rate override (빈 값=yaml)"
        ),
        DeclareLaunchArgument(
            "communication_mode",
            default_value="",
            description="yaml communication_mode override (빈 값=yaml)",
        ),
        DeclareLaunchArgument(
            "recv_timeout_ms",
            default_value="",
            description="yaml recv_timeout_ms override (빈 값=yaml)",
        ),
        DeclareLaunchArgument(
            "protocol_version",
            default_value="",
            description="yaml protocol_version override (빈 값=yaml). firmware peer 에도 전파.",
        ),
        DeclareLaunchArgument(
            "use_cpu_affinity",
            default_value="",
            description=(
                "yaml use_cpu_affinity override (빈 값=yaml, 노드 기본 true). "
                "false 면 프로세스 내부 pin(main → hand_driver slot, hand_aux_io → aux slot)을 "
                "건너뛴다. 스케줄링 정책은 유지되므로 CommLoop 은 SCHED_FIFO 65 그대로다."
            ),
        ),
    ]
    return LaunchDescription([*override_arg_decls, OpaqueFunction(function=_launch_setup)])
