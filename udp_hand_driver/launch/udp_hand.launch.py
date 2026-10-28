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
#                 protocol_version "2" (CAN FD, epic #895) 에서는 loopback 이 UDP 가
#                 아니라 vcan0 다: 노드가 can_interface 를 vcan0 로 강제하고, 이 launch
#                 는 fake 를 같은 vcan0 · 같은 hand_side · 같은 cmd_ack 로 띄운다. vcan0 이 없거나
#                 FD(MTU 72) 가 아니면 여기서 거부한다 (repo_scripts/scripts/setup_vcan.sh).
#                 `hand_sides:=right,left` 면 fake 를 손마다 하나씩
#                 (`fake_hand_firmware_<side>`, seed 는 손마다 다르게) 띄운다.
#
# `sil_mode:=firmware` 처럼 CLI 로 준 값이 yaml 의 sil_mode 를 이긴다.
#
# ── 네임스페이스 (D53) ──────────────────────────────────────────────────────
# 토픽 기본값은 상대 이름 (`joint_states` …) 이고 노드 네임스페이스가 접두다.
# `namespace` 인자는 유일한 **비-override** 인자로 기본 `/hand` — 그래서 런타임
# 이름은 /hand/joint_states …, 노드 FQN 은 /hand/udp_hand_node 다. 빈 값은 다른
# 인자처럼 "yaml 값" 이 되지 않고 루트 (`/joint_states` — 팔의 토픽) 로 가 버리므로
# _launch_setup 이 거부한다.
#
# ── 두 손 (epic #906 D69 · D73) ─────────────────────────────────────────────
# 노드는 하나다. `hand_sides:=right,left` (쉼표 목록) 를 주면 그 노드가 두 손을
# 한 loop 로 돌리고 토픽은 /hand/right/* · /hand/left/*, E-Stop 은 /hand/estop 하나다.
# protocol_version "2" 전용 (노드가 거부한다). 빈 값이면 파라미터를 넘기지 않아
# yaml / `hand_side` 의 한 손 경로 그대로다. 두 손은 req_mode auto 가 broadcast 로
# 풀리므로 `loop_rate_hz:=400` 을 준다 (#956 D86 — IXXAT USB
# 실기값; 네이티브 CAN 호스트 (Jetson Thor) 는 500, D89 — broadcast 의 수신 마감은
# tick 시작 + (slot − broadcast_tail_guard_ms) 라 recv_timeout_ms 를 쓰지 않는다.
# per_hand 만 노드 configure 가 `손 수 × 2 × recv_timeout_ms < 1000/loop_rate_hz`
# 를 검사한다).
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
_FW_HAND_SIDE_DEFAULT = "right"
# 노드 C++ declare_once("cmd_ack", "none") 기본값의 거울 — 기본값을 바꿀 때 둘 다 바꾼다 (D62).
_FW_CMD_ACK_DEFAULT = "none"
# fake_hand_firmware.yaml 에 seed 가 없을 때 (fake_hand_firmware_main.cpp 의 declare 기본과 일치).
_FW_SEED_DEFAULT = 42
# protocol_version "2" 의 firmware SIL 이 쓰는 vcan 장치 — 노드 쪽 상수
# udp_hand_driver::kFirmwareSilCanInterface (udp_hand_node.hpp) 와 같은 값.
_FW_CAN_INTERFACE = "vcan0"
_CANFD_MIN_MTU = 72


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
    ("broadcast_tail_guard_ms", float),
    ("protocol_version", str),
    ("can_interface", str),
    ("hand_side", str),
    ("cmd_ack", str),
    ("req_mode", str),
    ("loop_rate_hz", float),
    ("use_cpu_affinity", _as_bool),
]


def _check_fd_vcan(interface):
    """protocol_version "2" firmware SIL 의 전제: `interface` 가 있고 CAN FD (MTU ≥ 72) 다.

    디렉토리 존재만 보면 classic (MTU 16) vcan 이 통과해 모든 configure 검사를 지난 뒤
    프레임마다 실패하므로 MTU 를 본다.
    """
    mtu_path = f"/sys/class/net/{interface}/mtu"
    try:
        with open(mtu_path) as f:
            mtu = int(f.read().strip())
    except (OSError, ValueError) as exc:
        raise RuntimeError(
            f"sil_mode=firmware with protocol_version 2 needs CAN FD interface '{interface}' "
            f"({mtu_path} unreadable: {exc}); run repo_scripts/scripts/setup_vcan.sh"
        ) from exc
    if mtu < _CANFD_MIN_MTU:
        raise RuntimeError(
            f"'{interface}' is not CAN FD (mtu {mtu} < {_CANFD_MIN_MTU}); "
            "run repo_scripts/scripts/setup_vcan.sh"
        )


def _parse_hand_sides(val):
    """`hand_sides:=right,left` → ["right", "left"] (빈 항목 무시, 빈 값이면 []).

    값 검증 (철자 · 중복 · 개수) 은 노드 configure 가 한다 — 여기서 같은 규칙을 다시
    쓰면 둘이 갈린다.
    """
    return [s.strip() for s in val.split(",") if s.strip()]


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
    # namespace 는 override 인자가 아니다: 빈 값은 "yaml 값" 이 아니라 루트 네임스페이스라
    # 상대 토픽이 /joint_states 로 가 버린다 (머리 주석). 조용히 그렇게 되지 않게 막는다.
    if not LaunchConfiguration("namespace").perform(context):
        raise RuntimeError(
            "namespace must not be empty (default '/hand'); relative topics would land in /"
        )

    # 노드 파라미터 = [단일 yaml, 예시 ft yaml] + {CLI 로 명시된 override 만}.
    # sil_mode / target_ip 파생은 노드 on_configure 가 담당하므로 여기선 주입하지 않는다.
    overrides = {}
    if cli_sil:
        overrides["sil_mode"] = cli_sil
    for key, caster in _OVERRIDE_ARGS:
        val = LaunchConfiguration(key).perform(context)
        if val:
            overrides[key] = caster(val)
    # 빈 목록은 넘기지 않는다: 빈 배열 파라미터는 타입을 잃어 노드의 string-array
    # 선언과 어긋나고, 비어 있음은 곧 한 손 경로다.
    hand_sides = _parse_hand_sides(LaunchConfiguration("hand_sides").perform(context))
    if hand_sides:
        overrides["hand_sides"] = hand_sides

    node_params = [hand_config, ft_config]
    if overrides:
        node_params.append(overrides)

    udp_hand_node = LifecycleNode(
        package="udp_hand_driver",
        executable="udp_hand_node",
        name="udp_hand_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=node_params,
        emulate_tty=True,
    )

    actions = []

    # ── Device-side firmware simulator (sil_mode=firmware 일 때만) ──────────
    # 1a / 1b: target_port 를 bind 하고 선택된 protocol_version 을 echo → 노드와 wire
    # 일치. "2": vcan0 에 노드와 같은 hand_side · cmd_ack 로 붙는다 (port 는 쓰이지 않는다).
    if eff_sil == "firmware":
        fw_proto = str(
            overrides.get(
                "protocol_version", yaml_params.get("protocol_version", _FW_PROTO_DEFAULT)
            )
        )
        # (fake 이름, 파라미터) — 한 손이면 하나, hand_sides 면 손마다 하나.
        fakes = []
        if fw_proto == "2":
            _check_fd_vcan(_FW_CAN_INTERFACE)
            fw_ack = str(overrides.get("cmd_ack", yaml_params.get("cmd_ack", _FW_CMD_ACK_DEFAULT)))
            base = {
                "protocol_version": "2",
                "can_interface": _FW_CAN_INTERFACE,
                "cmd_ack": fw_ack,
            }
            if hand_sides:
                # 손마다 seed 를 달리해 두 fake 의 센서 합성이 같은 수열이 되지 않게 한다.
                fw_seed = int(_load_ros_params(firmware_config).get("seed", _FW_SEED_DEFAULT))
                for i, side in enumerate(hand_sides):
                    fakes.append(
                        (
                            f"fake_hand_firmware_{side}",
                            {**base, "hand_side": side, "seed": fw_seed + i},
                        )
                    )
            else:
                fw_side = str(
                    overrides.get("hand_side", yaml_params.get("hand_side", _FW_HAND_SIDE_DEFAULT))
                )
                fakes.append(("fake_hand_firmware", {**base, "hand_side": fw_side}))
        else:
            if hand_sides:
                raise RuntimeError(
                    f"hand_sides needs protocol_version 2 (got {fw_proto!r}); two hands share "
                    "one bus only on CAN FD"
                )
            fw_port = int(
                overrides.get("target_port", yaml_params.get("target_port", _FW_PORT_DEFAULT))
            )
            fakes.append(("fake_hand_firmware", {"port": fw_port, "protocol_version": fw_proto}))
        for fw_name, fw_params in fakes:
            actions.append(
                Node(
                    package="udp_hand_driver",
                    executable="fake_hand_firmware",
                    name=fw_name,
                    namespace="",
                    output="screen",
                    parameters=[firmware_config, fw_params],
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
    # namespace 만 실제 기본값을 갖는다 (머리 주석) — 나머지는 override-only: 빈 문자열
    # default 이면 yaml 값을 그대로 사용한다.
    # (choices= 를 쓰면 빈 문자열 default 가 거부되므로 sil_mode 검증은 _launch_setup 에서.)
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/hand",
        description=(
            "udp_hand_node 의 네임스페이스. 상대 토픽 기본값 (joint_states …) 의 접두이자 "
            "노드 FQN (<namespace>/udp_hand_node). 다른 인자와 달리 빈 값은 거부된다."
        ),
    )
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
            "broadcast_tail_guard_ms",
            default_value="",
            description=(
                "yaml broadcast_tail_guard_ms override (빈 값=yaml) — broadcast 수신 창 = "
                "slot − 이 값 [ms], FT 추론을 켜면 그 시간만큼 올린다 (D89)"
            ),
        ),
        DeclareLaunchArgument(
            "protocol_version",
            default_value="",
            description=(
                "yaml protocol_version override (빈 값=yaml): 1a | 1b (UDP) | 2 (CAN FD). "
                "firmware peer 에도 전파."
            ),
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="",
            description=(
                "yaml can_interface override (빈 값=yaml). protocol_version 2 의 SocketCAN "
                "장치; sil_mode=firmware 는 노드가 vcan0 로 강제한다."
            ),
        ),
        DeclareLaunchArgument(
            "hand_side",
            default_value="",
            description=(
                "yaml hand_side override (빈 값=yaml, 노드 기본 right): right | left. CAN FD "
                "노드 id 를 정하고, 명시하면 stats JSON · timing CSV 이름에 _<side> 가 붙는다. "
                "firmware peer 에도 전파."
            ),
        ),
        DeclareLaunchArgument(
            "cmd_ack",
            default_value="",
            description=(
                "yaml cmd_ack override (빈 값=yaml, 노드 기본 none): echo | none. protocol_version "
                "2 펌웨어가 CMD frame 을 echo 하는지 — 펌웨어와 일치해야 한다. firmware peer 에도 "
                "전파."
            ),
        ),
        DeclareLaunchArgument(
            "req_mode",
            default_value="",
            description=(
                "yaml req_mode override (빈 값=yaml, 노드 기본 auto): auto | per_hand | broadcast "
                "(#956). broadcast 는 tick 마다 REQ 하나를 노드 0xFF 로 보내 두 손이 함께 답한다 — "
                "protocol_version 2 · cmd_ack none 전용, 수신 마감은 slot − broadcast_tail_guard_ms "
                "(recv_timeout_ms 는 쓰지 않는다 — D89). "
                "auto 는 2 · 두 손 · none 이면 broadcast, 그 밖은 per_hand (D86)."
            ),
        ),
        DeclareLaunchArgument(
            "hand_sides",
            default_value="",
            description=(
                "두 손 (protocol_version 2 전용): 쉼표 목록, 예 right,left. 노드 하나가 손마다 "
                "<namespace>/<side>/* 토픽을 쓰고, sil_mode=firmware 면 fake 를 손마다 띄운다. "
                "빈 값이면 넘기지 않는다 (hand_side 의 한 손 경로). hand_side 와 함께 주면 노드가 거부한다."
            ),
        ),
        DeclareLaunchArgument(
            "loop_rate_hz",
            default_value="",
            description=(
                "yaml loop_rate_hz override (빈 값=yaml). 두 손 broadcast 는 400 (IXXAT; Thor 500, "
                "write tick p99 × 1.1), per_hand 는 250 (T 0.9) — per_hand 는 손 수 × 2 × "
                "recv_timeout_ms < 1000/loop_rate_hz 여야 configure 를 지난다."
            ),
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
    return LaunchDescription(
        [namespace_arg, *override_arg_decls, OpaqueFunction(function=_launch_setup)]
    )
