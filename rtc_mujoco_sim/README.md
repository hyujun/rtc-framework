# rtc_mujoco_sim

![version](https://img.shields.io/badge/version-v5.20.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

**MuJoCo 3.x 물리 시뮬레이터 패키지**입니다. 실제 로봇 드라이버를 대체하여 개발 환경에서 알고리즘 검증 및 테스트를 수행할 수 있습니다. 멀티 그룹 아키텍처로 임의의 로봇 조합(ur5e + hand, kuka, franka 등)을 독립된 ROS2 토픽으로 제어합니다.
LifecycleNode 기반으로, `ros2 lifecycle` CLI를 통한 런타임 상태 제어(deactivate/activate)를 지원합니다.

## 개요

```
rtc_mujoco_sim/
├── include/rtc_mujoco_sim/
│   ├── mujoco_simulator.hpp       ← 멀티 그룹 MuJoCo 3.x 물리 래퍼
│   └── ros2_resource_provider.hpp ← package:// URI provider 등록
├── src/
│   ├── mujoco_simulator.cpp       ← Initialize, 양방향 XML 검증, 이름 기반 매핑
│   ├── mujoco_sim_loop.cpp        ← SimLoop: 동기식 물리 루프 (그룹 순회)
│   ├── viewer/                    ← GLFW 뷰어 (4-파일 구조)
│   │   ├── viewer_state.hpp       ←   공유 상태 + 함수 선언 (내부 헤더)
│   │   ├── viewer_loop.cpp        ←   ViewerLoop 멤버 함수 (~60Hz)
│   │   ├── viewer_callbacks.cpp   ←   GLFW 입력 콜백 (키/마우스)
│   │   └── viewer_overlays.cpp    ←   mjr_overlay/mjr_figure 렌더 함수
│   ├── mujoco_simulator_node.cpp  ← ROS2 LifecycleNode (robot_response / fake_response 파싱)
│   └── ros2_resource_provider.cpp ← package:// URI 해석 구현
├── config/
│   ├── mujoco_default.yaml   ← agnostic 기본값 (sync/timestep/viewer 등)
│   └── solver_param.yaml     ← MuJoCo constraint solver 파라미터 (XML 우선)
└── launch/
    └── mujoco_sim.launch.py  ← agnostic 단독 launch (mujoco_simulator_node 만 띄움)
```

> **이 패키지는 robot-agnostic입니다.** 로봇별 모델/조인트/토픽 설정과 컨트롤러 통합 launch는 robot-specific bringup 패키지(예: `ur5e_bringup`)가 소유합니다. 자세한 원칙: [agent_docs/design-principles.md](../agent_docs/design-principles.md).

**의존성 그래프 내 위치:**

```
rtc_base + rtc_msgs  ← 독립
    ↑
rtc_mujoco_sim       ← agnostic 물리 시뮬레이터 (model_path는 외부 입력)
    ↑
ur5e_bringup 등      ← robot YAML + launch에서 mujoco_simulator_node를 직접 띄움
```

---

## 핵심 아키텍처: 멀티 그룹 (robot_response / fake_response)

`rtc_mujoco_sim`은 **그룹 기반 아키텍처**로 임의 개수의 로봇/디바이스를 독립적으로 제어합니다.

### robot_response (MuJoCo 물리 시뮬레이션)

- YAML에 명시된 `command_joint_names`를 XML의 hinge+actuator 조인트와 **양방향 완전 일치 검증**
  - YAML에 있는데 XML에 없음 → **FAIL**
  - XML에 있는데 YAML에 없음 → **FAIL**
- `state_joint_names`로 state publish 시 관절 이름/순서를 별도 지정 가능 (빈 배열 = XML 전체)
- 이름 기반 qpos/qvel/actuator 인덱스 매핑 (비연속 인덱스 지원)
- 그룹별 독립 command/state 버퍼, control mode, servo gains
- 첫 번째 robot 그룹이 동기 루프의 primary 대기 대상

### fake_response (LPF 에코백)

- MuJoCo 물리를 사용하지 않는 1차 저역통과 필터 기반 에코백
- 토픽 이름 유효성만 검증 (비어있으면 에러)
- 100Hz 타이머로 LPF 진행 + JointState 퍼블리시
- XML에 해당 조인트가 없는 디바이스를 시뮬레이션할 때 사용

### 검증 규칙

| 항목 | robot_response | fake_response |
|---|---|---|
| XML 조인트 검증 | 양방향 완전 일치 필수 | 없음 |
| joint_names | YAML 필수 명시 | YAML 필수 명시 |
| 토픽 이름 | 비어있으면 에러 | 비어있으면 에러 |
| 중복 그룹 이름 | robot ∩ fake ≠ ∅ → **에러** | 동일 |

---

## 시뮬레이션 루프

동기식 루프로 동작합니다. 매 스텝마다 state 퍼블리시 → command 대기 → 물리 서브스텝을 순서대로 수행하며, `max_rtf`로 속도를 제한합니다.

```
SimLoop → 모든 robot 그룹의 state 퍼블리시
        → 모든 robot 그룹의 sensor 퍼블리시 (sensor_topic 설정 시)
        → primary 그룹의 command 대기 (sync_timeout_ms)
        → 모든 robot 그룹의 command 적용 (ApplyCommand, 1회)
        → for sub in [0, n_substeps):
        │     PreparePhysicsStep() (actuator 모드/solver 파라미터/중력/외력)
        │     mj_step()            (substep_dt = physics_timestep / n_substeps)
        │     ClearContactForces()
        → ReadSolverStats()
        → ThrottleIfNeeded(max_rtf)
        → (반복)
```

### Substepping

`n_substeps` 파라미터로 한 제어 주기 내 물리 스텝 횟수를 제어합니다. 제어 주기(`physics_timestep`)는 변하지 않고, MuJoCo의 실제 step 크기(`substep_dt`)만 작아집니다.

| n_substeps | substep_dt | 제어 주기 | 효과 |
|-----------|-----------|----------|------|
| `1` (기본) | 2.0ms | 2.0ms (500Hz) | 기존 동작과 동일 |
| `4` | 0.5ms | 2.0ms (500Hz) | 물리 해상도 4배 향상 |
| `10` | 0.2ms | 2.0ms (500Hz) | contact-rich 시나리오 안정성 극대화 |

- `ApplyCommand()`는 루프 밖에서 1회만 호출 — 서브스텝 중간에 ctrl 값 변경 없음
- `StepOnce` (`>` 버튼)은 N substeps 전부 실행 = 1 제어 주기 진행
- **Physics Load** = substep 루프 wall time / 제어 주기 (뷰어 상단에 표시, >100%면 실시간 유지 불가)

rtc_controller_manager는 `use_sim_time_sync: true` 설정 시 `condition_variable` 기반으로 state 도착 즉시 wakeup하여 ControlLoop를 실행합니다. 기존 `clock_nanosleep` 대비 round-trip 지연이 ~1ms → ~0.35ms로 감소합니다.

### Constraint Solver 설정 (`solver_param.yaml`)

MuJoCo constraint solver의 전체 파라미터를 `config/solver_param.yaml`에서 관리합니다.

**XML 우선순위 규칙:**
- MJCF XML의 `<option>` 요소에 명시적으로 설정된 속성은 YAML 값보다 우선합니다
- XML에 해당 속성이 없는 경우에만 YAML의 값이 적용됩니다
- `Initialize()` 시 tinyxml2로 원본 XML을 파싱하여 명시된 속성 집합을 감지합니다

```
XML: <option solver="CG" iterations="50"/>
YAML: solver: "Newton", iterations: 100, cone: "elliptic"
결과: solver=CG (XML), iterations=50 (XML), cone=elliptic (YAML fallback)
```

| 카테고리 | 파라미터 | 기본값 | 설명 |
|----------|----------|--------|------|
| **알고리즘** | `solver` | `"Newton"` | constraint solver (`PGS`, `CG`, `Newton`) |
| | `cone` | `"pyramidal"` | 마찰 원뿔 모델 (`pyramidal`, `elliptic`) |
| | `jacobian` | `"auto"` | Jacobian 표현 (`dense`, `sparse`, `auto`) |
| | `integrator` | `"Euler"` | 적분기 (`Euler`, `RK4`, `implicit`, `implicitfast`) |
| **반복 계산** | `iterations` | `100` | 메인 solver 최대 반복 횟수 |
| | `tolerance` | `1e-8` | 메인 solver 수렴 threshold |
| | `ls_iterations` | `50` | CG/Newton linesearch 최대 반복 |
| | `ls_tolerance` | `0.01` | linesearch 수렴 threshold |
| | `noslip_iterations` | `0` | Noslip 후처리 반복 (0=비활성화, 매니퓰레이션에서 10~20 권장) |
| | `noslip_tolerance` | `1e-6` | Noslip solver 수렴 threshold |
| | `ccd_iterations` | `50` | convex collision 최대 반복 |
| | `ccd_tolerance` | `1e-6` | convex collision 수렴 threshold |
| | `sdf_iterations` | `10` | SDF collision 반복 횟수 |
| | `sdf_initpoints` | `40` | SDF 초기 탐색 포인트 수 |
| **물리** | `impratio` | `1.0` | 마찰/법선 impedance 비율 (elliptic cone에서 5~10으로 slip 억제) |
| **플래그** | `warmstart` | `true` | 이전 timestep constraint force로 초기화 |
| | `refsafe` | `true` | `solref[0] < 2*timestep` 자동 보정 |
| | `island` | `false` | constraint island discovery (CG 병렬화) |
| | `eulerdamp` | `true` | Euler에서 joint damping implicit 처리 |
| | `filterparent` | `true` | 부모-자식 body collision 필터링 |
| **Contact Override** | `contact_override.enable` | `false` | 전체 contact 파라미터 일괄 덮어쓰기 |
| | `contact_override.o_solref` | `[0.02, 1.0]` | override solref [timeconst, dampratio] |
| | `contact_override.o_solimp` | `[0.9, 0.95, 0.001, 0.5, 2.0]` | override solimp [d0, dwidth, width, midpoint, power] |
| | `contact_override.o_friction` | `[1.0, 1.0, 0.005, 0.0001, 0.0001]` | override friction [tan1, tan2, spin, roll1, roll2] |

**Runtime 변경 가능 파라미터** (atomic, viewer 단축키 연동):
`solver`, `integrator`, `iterations`, `tolerance`, `cone`, `jacobian`, `ls_iterations`, `ls_tolerance`, `noslip_iterations`, `noslip_tolerance`, `impratio`

**Init-only 파라미터** (Initialize 시 1회 적용):
`ccd_*`, `sdf_*`, flags, contact override

**튜닝 레시피:**

```yaml
# 매니퓰레이션 (contact slip 억제)
solver:
  solver: "Newton"
  cone: "elliptic"
  impratio: 10.0
  tolerance: 1.0e-10
  noslip_iterations: 10

# 안정적 보행 시뮬레이션
solver:
  solver: "Newton"
  cone: "elliptic"
  impratio: 5.0
  iterations: 50
  tolerance: 1.0e-10
```

### max_rtf (Maximum Real-Time Factor)

| max_rtf | 동작 |
|---------|------|
| `1.0` | 실시간 (wall clock 2ms당 sim 2ms) |
| `10.0` | 10배속 |
| `0.0` | 무제한 (round-trip 속도가 곧 sim 속도) |

---

## ROS2 인터페이스

모든 그룹은 동일한 메시지 타입을 사용합니다:
- **커맨드**: `rtc_msgs/JointCommand` (joint_names + values + command_type)
- **상태**: `sensor_msgs/JointState` (position, velocity, effort)
- **센서** (선택): `rtc_msgs/SimSensorState` (MuJoCo XML 센서 데이터)

### 퍼블리시 토픽 (그룹별)

| 토픽 | 타입 | 주기 | 설명 |
|------|------|------|------|
| `<group.state_topic>` (예: `/joint_states`) | `sensor_msgs/JointState` | 매 물리 스텝 | robot 그룹: 위치/속도/토크 |
| `<group.state_topic>` (예: `/hand/joint_states`) | `sensor_msgs/JointState` | 100Hz | fake 그룹: LPF 필터링된 상태 |
| `<group.sensor_topic>` (예: `/hand/sim_sensors`) | `rtc_msgs/SimSensorState` | 매 물리 스텝 | robot 그룹: MuJoCo XML 센서 (선택, YAML에 `sensor_topic` + `sensor_names` 설정 시) |
| `/sim/status` | `std_msgs/Float64MultiArray` | 1Hz | `[step_count, sim_time_sec, rtf, paused(0/1)]` |

### 구독 토픽 (그룹별)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `<group.command_topic>` (예: `/ur5e/joint_command`) | `rtc_msgs/JointCommand` | robot 그룹: position/torque 명령 (command_type으로 자동 전환) |
| `<group.command_topic>` (예: `/hand/joint_command`) | `rtc_msgs/JointCommand` | fake 그룹: LPF 타겟 업데이트 |

> **QoS**: command subscriber와 state publisher 모두 `BEST_EFFORT` + depth 1을 사용합니다.

### 제어 모드 자동 전환 (robot 그룹)

| command_type | 제어 모드 | Gravity |
|---|---|---|
| `"position"` | Position Servo | 자동 OFF + 잠금 |
| `"torque"` | Direct Torque | 자동 ON (모든 robot 그룹이 torque일 때) |

Position servo 모드에서는 중력이 자동으로 OFF되고 잠금됩니다. 모든 robot 그룹이 torque 모드로 전환되어야 중력이 ON됩니다.

---

## 스레딩 모델

```
mujoco_simulator_node
    │
    ├── SimLoop 스레드 (jthread)
    │     └── SimLoop (동기식: state→wait→step→throttle)
    │         ├── 그룹별 cmd_mutex + cmd_pending (atomic) — 명령 전달
    │         ├── sync_cv_ — primary 그룹 명령 대기
    │         ├── 그룹별 state_mutex — 최신 상태 스냅샷 보호
    │         ├── pert_mutex_ (try_lock 전용 — 퍼튜베이션/외부힘 보호)
    │         └── solver_stats_mutex_ — solver 통계 보호
    │
    └── ViewerLoop 스레드 (jthread, 선택적)
          └── GLFW 3D 뷰어 ~60Hz
              └── viz_mutex_ (try_lock 전용 — SimLoop 절대 블로킹 안 함)

Caller 스레드 (ROS2 노드): SetCommand(group_idx), GetPositions(group_idx) 호출
Fake 타이머 (100Hz): AdvanceFakeLPF(group_idx), GetFakeState(group_idx) 호출
```

---

## 설정

설정 파일은 두 계층으로 분리되어 있습니다:
- `config/mujoco_default.yaml` — agnostic 기본값 (sync timeout, max_rtf, viewer, physics_timestep 등). `model_path`/`robot_response.*`는 비어 있어 그대로 사용 시 노드 configure에서 실패함
- `config/solver_param.yaml` — MuJoCo constraint solver 파라미터 (XML 우선순위)

robot-specific 오버라이드(예: UR5e 설정 — `model_path`, `robot_response.groups`, joint names, command/state 토픽, servo gains)는 robot bringup 패키지에 위치합니다 — 예시: [ur5e_bringup/config/mujoco_simulator.yaml](../ur5e_bringup/config/mujoco_simulator.yaml).

### `config/mujoco_simulator.yaml` (예시 — UR5e bringup 형식)

robot-specific bringup 패키지의 YAML 형태. `mujoco_default.yaml` 위에 오버레이됩니다. 아래 예시는 UR5e 6-DOF arm + 10-DOF hand 구성:

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: "robots/ur5e/mjcf/scene_with_hand.xml"
    sync_timeout_ms: 50.0        # 명령 대기 타임아웃 (ms)
    max_rtf: 1.0                 # 최대 실시간 비율 (0.0 = 무제한)
    enable_viewer: true          # GLFW 3D 뷰어 활성화
    physics_timestep: 0.002      # seconds — control period (constant)
    n_substeps: 1                # physics substeps per control cycle (substep_dt = physics_timestep / n_substeps)

    # Position servo 게인 (글로벌 기본값)
    use_yaml_servo_gains: false
    servo_kp: [500.0, 500.0, 500.0, 150.0, 150.0, 150.0]
    servo_kd: [400.0, 400.0, 400.0, 100.0, 100.0, 100.0]

    # ── Robot Response (MuJoCo 물리) ──────────────────────────────
    # command_joint_names 합집합 == XML 전체 joints (양방향 완전 일치 필수)
    robot_response:
      groups: ["ur5e", "hand"]
      ur5e:
        command_joint_names:
          - shoulder_pan_joint
          - shoulder_lift_joint
          - elbow_joint
          - wrist_1_joint
          - wrist_2_joint
          - wrist_3_joint
        state_joint_names:         # state publish용 (빈 배열 = XML 전체)
          - shoulder_pan_joint
          - shoulder_lift_joint
          - elbow_joint
          - wrist_1_joint
          - wrist_2_joint
          - wrist_3_joint
        command_topic: "/ur5e/joint_command"
        state_topic: "/joint_states"
        # sensor_topic: "/ur5e/sim_sensors"  # MuJoCo 센서 publish (선택)
        # sensor_names: ["ft_sensor"]        # XML sensor 이름 목록
      hand:
        command_joint_names:
          - thumb_cmc_aa
          - thumb_cmc_fe
          - thumb_mcp_fe
          - index_mcp_aa
          - index_mcp_fe
          - index_dip_fe
          - middle_mcp_aa
          - middle_mcp_fe
          - middle_dip_fe
          - ring_mcp_fe
        command_topic: "/hand/joint_command"
        state_topic: "/hand/joint_states"
        # sensor_topic: "/hand/sim_sensors"  # MuJoCo 센서 publish (선택)
        # sensor_names: ["thumb_touch", "index_touch"]  # XML sensor 이름 목록

    # ── Fake Response (LPF 에코백) ────────────────────────────────
    # robot_response와 같은 그룹 이름 중복 불가.
    # XML에 hand가 없는 모델 사용 시 hand를 여기로 이동.
    # fake_response:
    #   groups: ["hand"]
    #   hand:
    #     command_joint_names: [thumb_cmc_aa, ...]
    #     command_topic: "/hand/joint_command"
    #     state_topic: "/hand/joint_states"
    #     filter_alpha: 0.1

# RT 컨트롤러 노드용 시뮬레이션 오버라이드는 robot-specific bringup YAML
# (예: ur5e_bringup/config/ur5e_sim.yaml)에서 `/**:` wildcard로 관리.
```

> **참고:** `joint_names`는 `command_joint_names` 미지정 시 하위 호환 대체로 사용됩니다. `state_joint_names`가 빈 배열이면 robot 그룹은 XML 전체 조인트, fake 그룹은 command_joint_names와 동일하게 사용됩니다.

### 노드가 읽는 파라미터 목록

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `model_path` | string | `""` | MJCF 모델 경로 (필수). 빈 값이면 노드 configure 시 `runtime_error`. robot-specific bringup이 `package://<pkg>/path/to/scene.xml` 형태로 전달. |
| `enable_viewer` | bool | `true` | GLFW 3D 뷰어 활성화 |
| `sync_timeout_ms` | double | `50.0` | primary 그룹 command 대기 타임아웃 (ms) |
| `max_rtf` | double | `0.0` | 최대 실시간 비율 (0.0 = 무제한) |
| `control_rate` | double | `500.0` | 런치 파일에서 전달. physics_timestep 검증용 |
| `physics_timestep` | double | `0.0` | 0.0이면 XML 값 사용. 양수면 XML과 불일치 시 경고. 제어 주기를 의미 |
| `n_substeps` | int | `1` | 제어 주기당 물리 서브스텝 수. `substep_dt = physics_timestep / n_substeps`. 1이면 기존 동작 |
| `use_yaml_servo_gains` | bool | `false` | `true`=YAML servo_kp/kd, `false`=XML gainprm/biasprm |
| `servo_kp` | double[] | `[500, 500, 500, 150, 150, 150]` | Position servo P 게인 (글로벌) |
| `servo_kd` | double[] | `[400, 400, 400, 100, 100, 100]` | Position servo D 게인 (글로벌) |
| `robot_response.groups` | string[] | `[]` | MuJoCo 물리 그룹 이름 목록 |
| `fake_response.groups` | string[] | `[]` | LPF 에코백 그룹 이름 목록 |
| `solver.*` | (다양) | (MuJoCo 기본값) | Solver 파라미터 — 상세: [solver_param.yaml](config/solver_param.yaml) 및 위 Constraint Solver 설정 섹션 참조 |

그룹별 파라미터 (`robot_response.<name>.` / `fake_response.<name>.`):

| 파라미터 | 타입 | 설명 |
|----------|------|------|
| `command_joint_names` | string[] | command용 조인트 이름 (robot: XML actuator 매칭) |
| `state_joint_names` | string[] | state publish용 조인트 이름 (빈 배열 = 기본값) |
| `joint_names` | string[] | 하위 호환용 (`command_joint_names` 미지정 시 대체) |
| `command_topic` | string | 커맨드 구독 토픽 |
| `state_topic` | string | 상태 퍼블리시 토픽 |
| `sensor_topic` | string | MuJoCo 센서 퍼블리시 토픽 (빈 문자열 = 비활성화) |
| `sensor_names` | string[] | XML 센서 이름 목록 (빈 배열 = 센서 없음) |
| `filter_alpha` | double | fake_response 전용 LPF 계수 (기본 0.1) |
| `servo_kp` / `servo_kd` | double[] | 그룹별 servo 게인 (미지정 시 글로벌 값 상속) |

### 그룹 구성 예시

**Case 1: ur5e + hand 모두 MuJoCo 물리** (scene_with_hand.xml 사용)

```yaml
robot_response:
  groups: ["ur5e", "hand"]
  ur5e: { command_joint_names: [...], state_joint_names: [...], command_topic: "/ur5e/joint_command", state_topic: "/joint_states" }
  hand: { command_joint_names: [...], command_topic: "/hand/joint_command", state_topic: "/hand/joint_states" }
```

**Case 1b: 센서 포함** (XML에 touch/force 센서 정의된 경우)

```yaml
robot_response:
  groups: ["ur5e", "hand"]
  ur5e:
    command_joint_names: [...]
    command_topic: "/ur5e/joint_command"
    state_topic: "/joint_states"
    sensor_topic: "/ur5e/sim_sensors"
    sensor_names: ["ft_sensor"]
  hand:
    command_joint_names: [...]
    command_topic: "/hand/joint_command"
    state_topic: "/hand/joint_states"
    sensor_topic: "/hand/sim_sensors"
    sensor_names: ["thumb_touch", "index_touch", "middle_touch", "ring_touch"]
```

**Case 2: ur5e만 물리, hand는 LPF** (scene.xml 사용, hand 조인트 없음)

```yaml
robot_response:
  groups: ["ur5e"]
  ur5e: { command_joint_names: [...], command_topic: "/ur5e/joint_command", state_topic: "/joint_states" }
fake_response:
  groups: ["hand"]
  hand: { command_joint_names: [...], command_topic: "/hand/joint_command", state_topic: "/hand/joint_states", filter_alpha: 0.1 }
```

**Case 3: 다른 로봇 (kuka 등)**

```yaml
robot_response:
  groups: ["kuka"]
  kuka:
    command_joint_names: [kuka_joint_1, kuka_joint_2, kuka_joint_3, ...]
    command_topic: "/kuka/joint_command"
    state_topic: "/kuka/joint_states"
```

---

## 실행

```bash
# 기본 (뷰어 활성화, max_rtf=1.0 실시간)
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py

# MJCF 경로를 직접 지정한 단독 smoke test
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

# Robot-specific 그룹 설정이 들어간 params 파일 추가
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \
    params_file:=$(ros2 pkg prefix ur5e_bringup)/share/ur5e_bringup/config/mujoco_simulator.yaml

# 10배속 / 헤드리스
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py model_path:=... max_rtf:=10.0
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py model_path:=... enable_viewer:=false
```

> **이 launch는 mujoco_simulator_node 단독만 띄우는 agnostic smoke test입니다.** RT 컨트롤러를 함께 실행하는 통합 시나리오는 robot-specific bringup의 launch (예: `ros2 launch ur5e_bringup sim.launch.py`)를 사용하세요.

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `params_file` | `""` | `mujoco_default.yaml` 위에 오버레이할 YAML (robot 그룹 설정 등) |
| `model_path` | `""` | MJCF scene 경로 (빈값 = YAML) |
| `enable_viewer` | `""` | GLFW 3D 뷰어 활성화 (빈값 = YAML) |
| `max_rtf` | `""` | 최대 실시간 비율 (빈값 = YAML, 0.0 = 무제한) |
| `max_log_sessions` | `10` | 최대 보관 세션 폴더 수 |

세션 디렉토리 (`YYMMDD_HHMM`)를 자동 생성하고 `RTC_SESSION_DIR` 환경변수로 전파합니다. 세션 루트 결정 로직은 `rtc_tools.utils.session_dir.resolve_logging_root()`의 4단 체인을 따릅니다 (자세한 내용: [rtc_tools/README.md](../rtc_tools/README.md)).

---

## 시뮬레이터 상태 모니터링

```bash
# 시뮬 상태 확인
ros2 topic echo /sim/status

# ur5e 관절 상태
ros2 topic echo /joint_states

# hand 관절 상태
ros2 topic echo /hand/joint_states

# MuJoCo 센서 데이터 (sensor_topic 설정 시)
ros2 topic echo /hand/sim_sensors

# 퍼블리시 주파수 확인
ros2 topic hz /joint_states
ros2 topic hz /hand/joint_states
```

---

## 뷰어 상태 오버레이

뷰어 우상단에 항상 표시되는 상태 패널의 항목:

| 항목 | 설명 |
|------|------|
| Mode | 동기 모드 (`sync`) |
| Camera | 카메라 모드 (Free/Tracking/Fixed:name) |
| RTF | 실시간 비율 |
| Limit | max_rtf 설정값 |
| Sim Time | 시뮬레이션 경과 시간 |
| Steps | 누적 제어 스텝 수 |
| Contacts | 활성 접촉 수 / on\|OFF |
| Gravity | ON / OFF / OFF(lock) |
| Status | running / PAUSED / perturb |
| Integrator | Euler / RK4 / Implicit / ImplFast |
| Solver | PGS / CG / Newton |
| Iterations | 사용된 / 최대 solver 반복 |
| Residual | solver 잔차 |
| Substeps | `n_substeps (substep_dt ms)` — 예: `4 (0.50ms)` |
| Physics Load | substep 루프 wall time / 제어 주기 (%) — 100% 초과 시 실시간 유지 불가 |

---

## 뷰어 단축키

### 키보드

| 키 | 기능 |
|---|---|
| **F1** | 도움말 오버레이 순환 (off → page1 → page2 → off) |
| Space | 일시정지 / 재개 |
| Right (일시정지 중) | 한 스텝 진행 |
| + / - | RTF 속도 2배 / 0.5배 |
| R | 초기 자세로 리셋 |
| TAB | 카메라 모드 순환 (Free → Tracking → Fixed[0..N-1] → Free) |
| Esc | 카메라 초기화 (Free 모드, 기본 거리/방위각) |
| Backspace | 시각화 옵션 초기화 |
| G | 중력 ON/OFF (position servo 중 잠금) |
| N | 접촉 제약 ON/OFF |
| I | integrator 순환 (Euler → RK4 → Implicit → ImplFast) |
| S | solver 순환 (PGS → CG → Newton) |
| ] / [ | solver 반복 횟수 x2 / /2 |
| C / F | 접촉점 / 접촉력 표시 |
| T | 투명 시각화 |
| J / U | 관절 / 액추에이터 시각화 |
| E / W | 관성 / CoM 시각화 |
| L / A / X | 조명 / 텐던 / 볼록 껍질 시각화 |
| 0-5 / V | 지오메트리 그룹 토글 (V = 그룹 0 별칭) |
| F3 | RTF 프로파일러 그래프 |
| F4 | Solver 통계 오버레이 |
| F5 / F6 / F7 / F8 | 와이어프레임 / 그림자 / 스카이박스 / 반사 |
| F9 | 센서 값 오버레이 |
| F10 | 모델 정보 오버레이 |
| P | 스크린샷 저장 (PPM) |

### 마우스

| 조작 | 기능 |
|---|---|
| Left drag | 카메라 궤도 |
| Shift + Left drag | 수평축 회전 |
| Right drag | 카메라 패닝 |
| Shift + Right drag | 수평 평면 패닝 |
| Scroll / Middle drag | 줌 인/아웃 |
| Dbl-click | 물체 선택 |
| Ctrl + Left drag | 퍼튜베이션 (토크) |
| Ctrl + Right drag | 퍼튜베이션 (힘 XZ) |
| Ctrl + Shift + Right drag | 퍼튜베이션 (힘 XY) |

---

## C++ API

```cpp
auto sim = std::make_unique<MuJoCoSimulator>(cfg);
sim->Start();

// 그룹 인덱스 기반 API
sim->SetCommand(0, cmd);                   // group 0 (ur5e) 커맨드
sim->SetCommand(1, cmd);                   // group 1 (hand) 커맨드
sim->SetControlMode(0, true);             // group 0 → torque 모드
sim->SetControlMode(1, false);            // group 1 → position servo 모드

auto pos = sim->GetPositions(0);          // group 0 위치
auto vel = sim->GetVelocities(0);         // group 0 속도
auto eff = sim->GetEfforts(0);            // group 0 토크
auto names = sim->GetJointNames(1);       // group 1 command 조인트 이름
auto snames = sim->GetStateJointNames(1); // group 1 state 조인트 이름

// 센서 API (robot 그룹, YAML sensor_names 설정 시)
sim->HasSensors(0);                       // 그룹에 센서 있는지 확인
auto& infos = sim->GetSensorInfos(0);     // SensorInfo 목록 (name, type, adr, dim)
sim->SetSensorCallback(0, [](const auto& infos, const auto& values) {
  // infos: 센서 메타데이터, values: flat double 배열
});

// Fake response API (fake 그룹)
sim->SetFakeTarget(group_idx, target);    // LPF 타겟 설정
sim->AdvanceFakeLPF(group_idx);           // LPF 1스텝 진행
auto state = sim->GetFakeState(group_idx);

// 하위 호환 API (group 0 위임)
sim->SetCommand(cmd);                      // = SetCommand(0, cmd)
sim->GetPositions();                       // = GetPositions(0)

// 물리 파라미터 런타임 변경
sim->SetIntegrator(mjINT_IMPLICIT);
sim->SetSolverType(mjSOL_NEWTON);
sim->SetSolverIterations(200);
sim->SetSolverTolerance(1e-10);
sim->SetCone(mjCONE_ELLIPTIC);
sim->SetImpratio(10.0);
sim->SetNoslipIterations(10);
sim->EnableGravity(false);
sim->SetContactEnabled(false);
sim->SetExternalForce(body_id, wrench);
sim->SetMaxRtf(5.0);

// 일시정지 / 리셋
sim->Pause();
sim->Resume();
sim->StepOnce();
sim->RequestReset();

// 시뮬레이터 상태
auto steps    = sim->StepCount();
auto rtf      = sim->GetRtf();
auto solver   = sim->GetSolverStats();
auto n_groups = sim->NumGroups();
auto n_sub    = sim->GetNumSubsteps();    // n_substeps 값
auto load     = sim->GetPhysicsLoad();    // physics wall time / control period
```

---

## MuJoCo ROS2 Resource Provider

`package://` URI를 MuJoCo가 네이티브로 해석할 수 있도록 전역 resource provider를 등록합니다.

```cpp
rtc::RegisterRos2ResourceProvider();
```

MJCF 파일 안에서 `package://` URI를 별도 변환 없이 사용 가능:
```xml
<mesh file="package://ur5e_description/robots/ur5e/meshes/ur5e.dae"/>
```

내부적으로 `ament_index_cpp::get_package_share_directory()`를 사용하여 패키지 경로를 해석하고, `mjpResourceProvider` 콜백(open/read/close/getdir)을 통해 MuJoCo에 파일 데이터를 제공합니다.

---

## 의존성

### ROS2 패키지 (빌드 + 런타임)

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 |
| `std_msgs` | Float64MultiArray (sim/status) |
| `sensor_msgs` | JointState (state publish) |
| `ament_index_cpp` | package:// URI 해석 |
| `rtc_msgs` | JointCommand, SimSensorState 메시지 |
| `rtc_base` | RTC 공통 유틸리티 |
| `tinyxml2` | MJCF XML `<option>` 파싱 (solver 파라미터 XML 우선순위 감지) |

### 런타임 전용

| 패키지 | 용도 |
|--------|------|
| `ur5e_description` | MJCF 모델, URDF, 메시 파일 |
| `ur5e_hand_driver` | hand_udp_node.yaml 참조 (선택) |
| `rtc_controller_manager` | 런치 파일에서 rtc_controller_manager 노드 실행 (실행 파일: `rt_controller`) |

### 외부 라이브러리 (비ROS2)

| 라이브러리 | 용도 |
|-----------|------|
| **MuJoCo 3.x** | 물리 시뮬레이션 엔진 (필수, 없으면 노드 빌드 생략) |
| **GLFW 3** | 3D 뷰어 윈도우 (선택, 없으면 headless 전용) |

---

## 빌드

### 전제 조건: MuJoCo 3.x 설치

```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
```

### colcon 빌드

```bash
colcon build --packages-select rtc_mujoco_sim --symlink-install \
    --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x
```

> MuJoCo가 설치되지 않은 경우 `mujoco_simulator_node`는 자동으로 빌드에서 제외됩니다.
> GLFW가 설치되지 않은 경우 뷰어 없이 headless 모드로만 빌드됩니다.
> 빌드 시 C++20 표준을 사용합니다 (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`).

---

## Testing

```bash
./build.sh -p rtc_mujoco_sim sim
colcon test --packages-select rtc_mujoco_sim --event-handlers console_direct+
colcon test-result --verbose
```

77 GTest 케이스 (7 파일):

| Test | Count | Scope |
|------|-------|-------|
| `test_pure_helpers` | 15 | `SolverNameToEnum`/`ConeNameToEnum`/`JacobianNameToEnum`/`IntegratorNameToEnum`/`ApplyFakeLpfStep` (순수 로직) |
| `test_simulator_init` | 15 | `Initialize` happy/실패 경로, joint/sensor 디스커버리, 검증 |
| `test_solver_config` | 9 | XML `<option>`/`<flag>` 우선순위, YAML fallback, ContactOverride |
| `test_command_state_io` | 12 | `SetCommand`/`SetControlMode`/`SetFakeTarget` 정합성 (스레드 미사용) |
| `test_lifecycle` | 10 | Start/Stop/Pause/Resume/Reset/StepOnce/SyncTimeout |
| `test_runtime_controls` | 11 | atomic setter/getter, 클램핑, gravity lock |
| `test_data_flow` | 5 | 상태/센서 콜백 firing, StepCount 단조, RTF |

Fixture: [test/fixtures/minimal.xml](test/fixtures/minimal.xml) (2-hinge 체인 + 2 센서).
GLFW 뷰어 통합 테스트는 헤드리스 CI 제약으로 제외.

---

## 변경 내역

| 버전 | 변경 내용 |
|------|----------|
| **v5.20.0** | 단위 테스트 스위트 추가 (77 GTest 케이스, 7 파일). `SolverNameToEnum`/`ConeNameToEnum`/`JacobianNameToEnum`/`IntegratorNameToEnum`/`ApplyFakeLpfStep`를 public static helper로 추출. 소스를 `mujoco_simulator_lib` 정적 라이브러리로 분리. |
| **v5.19.0** | MuJoCo 센서 publish 지원 추가. 그룹별 `sensor_topic` + `sensor_names` YAML 설정으로 XML 센서 데이터를 `rtc_msgs/SimSensorState`로 매 physics step 퍼블리시. 로봇 독립적(robot-agnostic) 구현. |
| **v5.18.0** | `n_substeps` 파라미터 추가 — 제어 주기당 `mj_step` 호출 횟수 제어. Physics Load 측정 및 뷰어 상태 오버레이에 Substeps/Physics Load 표시 추가. |
| **v5.17.0** | `SimMode` enum (`free_run`/`sync_step`) 제거 → 동기식 단일 루프로 통합. rtc_controller_manager CV 기반 wakeup (`use_sim_time_sync`) 지원. `publish_decimation` 파라미터 제거. |
| **v5.16.1** | C++20 표준 및 컴파일러 경고 플래그 확인 완료 |

---

## 라이선스

MIT License
