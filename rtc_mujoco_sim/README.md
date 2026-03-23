# rtc_mujoco_sim

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

**MuJoCo 3.x 물리 시뮬레이터 패키지**입니다. 실제 로봇 드라이버를 대체하여 개발 환경에서 알고리즘 검증 및 테스트를 수행할 수 있습니다. 멀티 그룹 아키텍처로 임의의 로봇 조합(ur5e + hand, kuka, franka 등)을 독립된 ROS2 토픽으로 제어합니다.

## 개요

```
rtc_mujoco_sim/
├── include/rtc_mujoco_sim/
│   ├── mujoco_simulator.hpp       ← 멀티 그룹 MuJoCo 3.x 물리 래퍼
│   └── ros2_resource_provider.hpp ← package:// URI provider 등록
├── src/
│   ├── mujoco_simulator.cpp       ← Initialize, 양방향 XML 검증, 이름 기반 매핑
│   ├── mujoco_sim_loop.cpp        ← SimLoopFreeRun / SimLoopSyncStep (그룹 순회)
│   ├── viewer/                    ← GLFW 뷰어 (4-파일 구조)
│   │   ├── viewer_state.hpp       ←   공유 상태 + 함수 선언 (내부 헤더)
│   │   ├── viewer_loop.cpp        ←   ViewerLoop 멤버 함수 (~60Hz)
│   │   ├── viewer_callbacks.cpp   ←   GLFW 입력 콜백 (키/마우스)
│   │   └── viewer_overlays.cpp    ←   mjr_overlay/mjr_figure 렌더 함수
│   ├── mujoco_simulator_node.cpp  ← ROS2 노드 (robot_response / fake_response 파싱)
│   └── ros2_resource_provider.cpp ← package:// URI 해석 구현
├── config/
│   └── mujoco_simulator.yaml
└── launch/
    └── mujoco_sim.launch.py
```

> **로봇 모델 파일** (`scene.xml`, `ur5e_with_hand.xml`, 메시)은 `ur5e_description` 패키지에 있습니다.
> `package://` URI는 `Ros2ResourceProvider`가 자동 해석합니다.

**의존성 그래프 내 위치:**

```
ur5e_description  ← 독립 (MJCF/URDF/메시 제공)
    ↑
rtc_mujoco_sim  ← ur5e_description에서 모델 참조 (ament_index + package:// URI)
                    런타임에 rtc_controller_manager와 함께 실행됨
```

---

## 핵심 아키텍처: 멀티 그룹 (robot_response / fake_response)

`rtc_mujoco_sim`은 **그룹 기반 아키텍처**로 임의 개수의 로봇/디바이스를 독립적으로 제어합니다.

### robot_response (MuJoCo 물리 시뮬레이션)

- YAML에 명시된 `joint_names`를 XML의 hinge+actuator 조인트와 **양방향 완전 일치 검증**
  - YAML에 있는데 XML에 없음 → **FAIL**
  - XML에 있는데 YAML에 없음 → **FAIL**
- 이름 기반 qpos/qvel/actuator 인덱스 매핑 (비연속 인덱스 지원)
- 그룹별 독립 command/state 버퍼, control mode, servo gains
- 첫 번째 robot 그룹이 `sync_step` 모드의 primary 대기 대상

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

## 시뮬레이션 모드

### `free_run`

`mj_step()`을 최대 속도로 진행합니다. 알고리즘 검증 및 빠른 반복 개발에 적합합니다.

```
SimLoop → mj_step() (가능한 빠르게)
        → 모든 robot 그룹의 state 퍼블리시 (publish_decimation마다)
        → 모든 robot 그룹의 command 비동기 적용
```

### `sync_step` (기본값)

상태 퍼블리시 → primary 그룹 명령 대기 → 물리 스텝을 순서대로 수행합니다.

```
SimLoop → 모든 robot 그룹의 state 퍼블리시
        → primary 그룹의 command 대기 (sync_timeout_ms)
        → 모든 robot 그룹의 command 적용
        → mj_step()
        → (반복)
```

---

## ROS2 인터페이스

모든 그룹은 동일한 메시지 타입을 사용합니다:
- **커맨드**: `rtc_msgs/JointCommand` (joint_names + values + command_type)
- **상태**: `sensor_msgs/JointState` (position, velocity, effort)

### 퍼블리시 토픽 (그룹별)

| 토픽 | 타입 | 주기 | 설명 |
|------|------|------|------|
| `<group.state_topic>` (예: `/joint_states`) | `sensor_msgs/JointState` | 물리 속도 또는 데시메이션 | robot 그룹: 위치/속도/토크 |
| `<group.state_topic>` (예: `/hand/joint_states`) | `sensor_msgs/JointState` | 100Hz | fake 그룹: LPF 필터링된 상태 |
| `/sim/status` | `std_msgs/Float64MultiArray` | 1Hz | `[step_count, sim_time_sec, rtf, paused(0/1)]` |

### 구독 토픽 (그룹별)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `<group.command_topic>` (예: `/ur5e/joint_command`) | `rtc_msgs/JointCommand` | robot 그룹: position/torque 명령 (command_type으로 자동 전환) |
| `<group.command_topic>` (예: `/hand/command`) | `rtc_msgs/JointCommand` | fake 그룹: LPF 타겟 업데이트 |

> **QoS**: command subscriber는 `BEST_EFFORT` QoS를 사용합니다.

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
    │     └── SimLoopFreeRun 또는 SimLoopSyncStep
    │         ├── 그룹별 cmd_mutex + cmd_pending (atomic) — 명령 전달
    │         ├── sync_cv_ — SyncStep primary 그룹 명령 대기
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

### `config/mujoco_simulator.yaml`

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: "robots/ur5e/mjcf/scene_with_hand.xml"
    sim_mode: "sync_step"        # "free_run" 또는 "sync_step"
    publish_decimation: 1        # free_run: N 스텝마다 퍼블리시
    sync_timeout_ms: 50.0        # sync_step: 명령 대기 타임아웃 (ms)
    max_rtf: 1.0                 # 최대 실시간 비율 (0.0 = 무제한)
    enable_viewer: true          # GLFW 3D 뷰어 활성화
    physics_timestep: 0.002      # seconds (= 500 Hz)

    # Position servo 게인 (글로벌 기본값)
    use_yaml_servo_gains: false
    servo_kp: [500.0, 500.0, 500.0, 150.0, 150.0, 150.0]
    servo_kd: [400.0, 400.0, 400.0, 100.0, 100.0, 100.0]

    # ── Robot Response (MuJoCo 물리) ──────────────────────────────
    # joint_names 합집합 == XML 전체 joints (양방향 완전 일치 필수)
    robot_response:
      groups: ["ur5e", "hand"]
      ur5e:
        joint_names:
          - shoulder_pan_joint
          - shoulder_lift_joint
          - elbow_joint
          - wrist_1_joint
          - wrist_2_joint
          - wrist_3_joint
        command_topic: "/ur5e/joint_command"
        state_topic: "/joint_states"
      hand:
        joint_names:
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
        command_topic: "/hand/command"
        state_topic: "/hand/joint_states"

    # ── Fake Response (LPF 에코백) ────────────────────────────────
    # robot_response와 같은 그룹 이름 중복 불가.
    # XML에 hand가 없는 모델 사용 시 hand를 여기로 이동.
    # fake_response:
    #   groups: ["hand"]
    #   hand:
    #     joint_names: [thumb_cmc_aa, ...]
    #     command_topic: "/hand/command"
    #     state_topic: "/hand/joint_states"
    #     filter_alpha: 0.1

# rt_controller 시뮬레이션 전용 오버라이드
rt_controller:
  ros__parameters:
    enable_estop: false
    robot_timeout_ms: 10000.0
    hand_timeout_ms: 10000.0
    init_timeout_sec: 0.0
```

### 그룹 구성 예시

**Case 1: ur5e + hand 모두 MuJoCo 물리** (scene_with_hand.xml 사용)

```yaml
robot_response:
  groups: ["ur5e", "hand"]
  ur5e: { joint_names: [...], command_topic: "/ur5e/joint_command", state_topic: "/joint_states" }
  hand: { joint_names: [...], command_topic: "/hand/command", state_topic: "/hand/joint_states" }
```

**Case 2: ur5e만 물리, hand는 LPF** (scene.xml 사용, hand 조인트 없음)

```yaml
robot_response:
  groups: ["ur5e"]
  ur5e: { joint_names: [...], command_topic: "/ur5e/joint_command", state_topic: "/joint_states" }
fake_response:
  groups: ["hand"]
  hand: { joint_names: [...], command_topic: "/hand/command", state_topic: "/hand/joint_states", filter_alpha: 0.1 }
```

**Case 3: 다른 로봇 (kuka 등)**

```yaml
robot_response:
  groups: ["kuka"]
  kuka:
    joint_names: [kuka_joint_1, kuka_joint_2, kuka_joint_3, ...]
    command_topic: "/kuka/joint_command"
    state_topic: "/kuka/joint_states"
```

---

## 실행

```bash
# 기본 (sync_step, 뷰어 활성화)
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py

# free_run 모드 (최대 속도)
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py sim_mode:=free_run

# 헤드리스 (CI/서버)
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py enable_viewer:=false

# 외부 MuJoCo 모델 사용
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `model_path` | `""` | scene.xml 경로 (빈값 = YAML) |
| `sim_mode` | `""` | `free_run` 또는 `sync_step` (빈값 = YAML) |
| `enable_viewer` | `""` | GLFW 3D 뷰어 활성화 (빈값 = YAML) |
| `publish_decimation` | `""` | free_run: N 스텝마다 퍼블리시 (빈값 = YAML) |
| `sync_timeout_ms` | `""` | sync_step: 명령 대기 타임아웃 ms (빈값 = YAML) |
| `max_rtf` | `""` | 최대 실시간 비율 (빈값 = YAML, 0.0 = 무제한) |
| `use_yaml_servo_gains` | `""` | `true`=YAML servo gain, `false`=XML gain (빈값 = YAML) |
| `max_log_sessions` | `10` | 최대 보관 세션 폴더 수 |

---

## 시뮬레이터 상태 모니터링

```bash
# 시뮬 상태 확인
ros2 topic echo /sim/status

# ur5e 관절 상태
ros2 topic echo /joint_states

# hand 관절 상태
ros2 topic echo /hand/joint_states

# 퍼블리시 주파수 확인
ros2 topic hz /joint_states
ros2 topic hz /hand/joint_states
```

---

## 뷰어 단축키

#### 키보드

| 키 | 기능 |
|---|---|
| **F1** | 도움말 오버레이 순환 |
| Space | 일시정지 / 재개 |
| Right (일시정지 중) | 한 스텝 진행 |
| + / - | RTF 속도 2배 / 0.5배 |
| R | 초기 자세로 리셋 |
| TAB | 카메라 모드 순환 |
| Esc | 카메라 초기화 |
| G | 중력 ON/OFF (position servo 중 잠금) |
| N | 접촉 제약 ON/OFF |
| I | integrator 순환 (Euler → RK4 → Implicit → ImplFast) |
| S | solver 순환 (PGS → CG → Newton) |
| ] / [ | solver 반복 횟수 ×2 / ÷2 |
| C / F | 접촉점 / 접촉력 표시 |
| T / J / U | 투명 / 관절 / 액추에이터 시각화 |
| F3 / F4 | RTF 프로파일러 / Solver 통계 오버레이 |
| P | 스크린샷 저장 |

#### 마우스

| 조작 | 기능 |
|---|---|
| Left drag | 카메라 궤도 |
| Right drag | 카메라 패닝 |
| Scroll | 줌 인/아웃 |
| Dbl-click | 물체 선택 |
| Ctrl + Left/Right drag | 퍼튜베이션 (토크/힘) |

---

## Physics Solver 런타임 제어

```cpp
auto sim = std::make_unique<MuJoCoSimulator>(cfg);
sim->Start();

// 그룹 인덱스 기반 API
sim->SetCommand(0, cmd);                   // group 0 (ur5e) 커맨드
sim->SetCommand(1, cmd);                   // group 1 (hand) 커맨드
sim->SetControlMode(0, true);             // group 0 → torque 모드
sim->SetControlMode(1, false);            // group 1 → position servo 모드

auto pos = sim->GetPositions(0);          // group 0 위치
auto names = sim->GetJointNames(1);       // group 1 조인트 이름

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
sim->EnableGravity(false);
sim->SetContactEnabled(false);
sim->SetExternalForce(body_id, wrench);
sim->SetMaxRtf(5.0);

// 시뮬레이터 상태
auto steps    = sim->StepCount();
auto rtf      = sim->GetRtf();
auto solver   = sim->GetSolverStats();
auto n_groups = sim->NumGroups();
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

---

## sync_step 컴퓨트 시간 분석

```python
import pandas as pd, glob, os
sessions = sorted(glob.glob(os.path.expanduser(
    '~/ros2_ws/ur5e_ws/logging_data/??????_????')))
if sessions:
    df = pd.read_csv(os.path.join(sessions[-1], 'controller', 'timing_log.csv'))
    print(df['t_compute_us'].describe())
    print(f'P95: {df["t_compute_us"].quantile(0.95):.1f} us')
    print(f'P99: {df["t_compute_us"].quantile(0.99):.1f} us')
```

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

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | C++20 표준 및 컴파일러 경고 플래그 확인 완료 — 이미 적용됨 |

---

## 라이선스

MIT License
