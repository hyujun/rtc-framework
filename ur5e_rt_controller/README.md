# ur5e_rt_controller

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스 (v5.16.0)의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md) | 디버깅: [VSCODE_DEBUGGING.md](../docs/VSCODE_DEBUGGING.md) | 새 컨트롤러 추가: [ADDING_CONTROLLER.md](docs/ADDING_CONTROLLER.md)

UR5e 로봇 팔을 위한 **500Hz 실시간 제어기** ROS2 패키지입니다. SCHED_FIFO 멀티스레드 아키텍처, 전략 패턴 기반 컨트롤러 교체, 런타임 컨트롤러 전환, 잠금-없는 로깅 인프라를 제공합니다.

## 개요

```
ur5e_rt_controller/
├── include/ur5e_rt_controller/
│   ├── rt_controller_interface.hpp          ← 추상 기반 클래스 (Strategy Pattern)
│   ├── rt_controller_node.hpp               ← RtControllerNode 클래스 선언
│   ├── controller_timing_profiler.hpp       ← 잠금-없는 Compute() 타이밍 프로파일러
│   ├── controllers/
│   │   ├── indirect/                        ← 위치 출력 컨트롤러 (position command)
│   │   │   ├── p_controller.hpp             ← 단순 P 제어기 (개발/테스트용)
│   │   │   ├── clik_controller.hpp          ← 폐루프 IK (데카르트 3/6-DOF)
│   │   │   └── ur5e_hand_controller.hpp     ← 로봇 암 + 핸드 통합 P 제어기
│   │   └── direct/                          ← 토크 출력 컨트롤러 (torque command)
│   │       ├── joint_pd_controller.hpp      ← 관절 PD + 중력/코리올리 보상 + E-STOP
│   │       └── operational_space_controller.hpp ← 6-DOF 데카르트 PD + SO(3)
│   └── trajectory/                          ← 5차 궤적 생성 (v5.3.0+)
│       ├── trajectory_utils.hpp             ← QuinticPolynomial 스칼라 유틸리티
│       ├── task_space_trajectory.hpp        ← SE(3) 스플라인 (CLIK/OSC 사용)
│       └── joint_space_trajectory.hpp       ← 관절공간 N-DOF 스플라인
├── src/
│   ├── rt_controller_node.cpp               ← Controller Registry + 노드 구현
│   ├── rt_controller_main.cpp               ← main() — executor/RT 스레드
│   ├── rt_controller_interface.cpp          ← 기반 클래스 구현 (TopicConfig 파싱)
│   └── controllers/
│       ├── indirect/
│       │   ├── p_controller.cpp
│       │   ├── clik_controller.cpp
│       │   └── ur5e_hand_controller.cpp
│       └── direct/
│           ├── joint_pd_controller.cpp
│           └── operational_space_controller.cpp
├── config/
│   ├── ur5e_rt_controller.yaml              ← 노드 레벨 파라미터 (제어율, E-STOP, 관절 한계)
│   ├── cyclone_dds.xml                      ← CycloneDDS 스레드 Core 0-1 제한
│   └── controllers/
│       ├── indirect/
│       │   ├── p_controller.yaml
│       │   ├── clik_controller.yaml
│       │   └── ur5e_hand_controller.yaml
│       └── direct/
│           ├── joint_pd_controller.yaml
│           └── operational_space_controller.yaml
├── scripts/
│   ├── setup_irq_affinity.sh                ← NIC IRQ → Core 0-1 고정 스크립트
│   ├── setup_udp_optimization.sh            ← UDP 소켓/네트워크 최적화
│   ├── setup_nvidia_rt.sh                   ← NVIDIA 드라이버 + RT 커널 공존 설정
│   └── build_rt_kernel.sh                   ← PREEMPT_RT 커널 빌드 도우미
├── test/
│   └── test_trajectory.cpp                  ← 궤적 생성 단위 테스트
├── docs/
│   └── ADDING_CONTROLLER.md                 ← 새 컨트롤러 추가 가이드
└── launch/
    └── ur_control.launch.py                 ← 전체 시스템 (use_cpu_affinity 포함)
```

### indirect vs direct 컨트롤러

컨트롤러는 출력 타입에 따라 두 그룹으로 분류됩니다:

- **`indirect/`** — **위치 명령** (`CommandType::kPosition`) 출력. `/forward_position_controller/commands`로 퍼블리시.
  - 제어 법칙이 **증분 위치 스텝**을 계산 (현재 위치 + 보정량)
- **`direct/`** — **토크 명령** (`CommandType::kTorque`) 출력. `/forward_torque_controller/commands`로 퍼블리시.
  - 제어 법칙이 **관절 토크**를 직접 계산 (PD + 동역학 보상)

**의존성:**
- `ur5e_rt_base` — 공유 타입, 스레드 유틸리티, 로깅 인프라
- `rclcpp`, `std_msgs`, `sensor_msgs`
- `pinocchio` — 역기구학, 동역학 (모든 컨트롤러에서 FK 사용)
- `yaml-cpp` — 컨트롤러별 YAML 설정 로드

---

## 아키텍처

### 전략 패턴 + 멀티스레드 아키텍처 (v5.16.0)

```
RtControllerNode (ROS2 노드)
    │
    ├── rt_loop (Core 2, FIFO/90)         ← std::jthread, clock_nanosleep 500Hz
    │     ├── ControlLoop()               ← 절대시간 루프 (TIMER_ABSTIME)
    │     └── CheckTimeouts() 매 10틱     ← 50Hz inline (executor 미사용)
    │
    ├── publish_thread (Core 5, OTHER/-3) ← std::jthread, SPSC 드레인 → publish()
    │     └── ControlPublishBuffer        ← lock-free SPSC 링 버퍼 (512 slots)
    │
    ├── sensor_executor (Core 3, FIFO/70) ← /joint_states, /target_joint_positions [전용]
    ├── log_executor (Core 4, OTHER/-5)   ← DataLogger CSV 기록 (SpscLogBuffer 드레인)
    └── aux_executor (Core 5, OTHER/0)    ← /system/estop_status 퍼블리시

HandController (별도 jthread, event-driven)
    └── udp_recv (Core 5, FIFO/65)       ← UDP 패킷 수신 [sensor_io와 분리]

StatusMonitor (별도 jthread, enable_status_monitor: true 시)
    └── monitor_thread (Core 4, OTHER/0) ← 10Hz 비-RT 상태/안전 모니터링

HandFailureMonitor (별도 jthread)
    └── hand_monitor (Core 4, OTHER/0)   ← 핸드 통신 실패 감지

    controller_ (RTControllerInterface)
        └── [교체 가능] PController / JointPDController / ClikController /
                        OperationalSpaceController / UrFiveEHandController
```

> **v5.16.0 변경**: `rt_executor` + `create_wall_timer()` → `clock_nanosleep` jthread.
> Phase 3 `publish()` 호출 → SPSC 버퍼 + `publish_thread`로 오프로드.
> Executor 4개 → 3개. RT 경로에서 DDS 직렬화/시스템 콜 완전 제거.

`mlockall(MCL_CURRENT | MCL_FUTURE)` — 시작 시 페이지 폴트 방지

### 스레드 간 동기화

| 뮤텍스 | 보호 대상 | 사용 스레드 |
|--------|-----------|------------|
| `state_mutex_` | `/joint_states` 최신값 | sensor ↔ RT |
| `target_mutex_` | 목표 관절 위치 | sensor ↔ RT |
| `hand_mutex_` | 손 데이터 타임스탬프 | sensor ↔ RT |

RT 스레드는 `try_to_lock` 패턴을 사용하여 뮤텍스 경합 시 이전 사이클의 캐시된 데이터를 재사용합니다 (≤2ms 지연, 허용 범위).

### Global E-Stop 시스템 (v5.8.0)

v5.8.0에서 기존 컨트롤러별 개별 E-Stop을 **글로벌 E-Stop** 아키텍처로 대체했습니다.

- **핵심 상태**: `std::atomic<bool> global_estop_` — 모든 서브시스템에서 원자적으로 참조
- **트리거**: `TriggerGlobalEstop(const std::string& reason)` — 사유를 로그에 기록하고 즉시 활성화
- **전파 대상**: 컨트롤러(`controller_->TriggerEstop()`), 핸드(`SetHandEstop(true)`), 상태 모니터(`StatusMonitor`)
- **초기화 타임아웃**: `init_timeout_sec` (기본 5.0초) — 초기화 완료 전 타임아웃 발생 시 `TriggerGlobalEstop()` + 노드 셧다운
- **E-Stop 상태 퍼블리시**: `/system/estop_status` (aux_executor, 50Hz)

```
[장애 발생] → TriggerGlobalEstop(reason)
                ├── global_estop_ = true
                ├── controller_->TriggerEstop()     ← 안전 위치로 이동
                ├── hand: SetHandEstop(true)        ← 핸드 정지
                └── status_monitor: 상태 반영       ← 모니터링 알림
```

### Status Monitor 통합 (v5.8.0)

`ur5e_status_monitor` 패키지를 `RtControllerNode`에 컴포지션 방식으로 통합했습니다.

- **활성화**: YAML `enable_status_monitor: true` (시뮬레이션 기본 `false`, 실제 로봇 시 `true` 권장)
- **설정 분리**: `status_monitor.*` 파라미터는 `ur5e_status_monitor` 패키지의 YAML에서 관리. launch 파일이 두 YAML을 함께 로드
- **실행 방식**: 별도 `std::jthread` — 10Hz 주기로 비-RT 모니터링
- **모니터링 항목**: 관절 한계, 속도 초과, 통신 상태, 추적 오차 등
- **장애 콜백**: 이상 감지 시 `TriggerGlobalEstop()` 호출 → 글로벌 E-Stop 연쇄 활성화
- **의존성**: `ur5e_status_monitor` 패키지 (shared library)

---

## 컨트롤러 구현

### `PController` (indirect, index 0)

단순 비례 제어기 (E-STOP 없음, 개발/테스트용). 증분 위치 스텝 방식으로 정상상태 오차가 없습니다.

```
command[i] = current_pos[i] + kp[i] * (target[i] - current_pos[i]) * dt
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `[120.0, 120.0, 100.0, 80.0, 80.0, 80.0]` | 관절별 비례 게인 |
| `command_type` | `"position"` | 출력 타입 |

- 최대 관절 속도: `2.0 rad/s` (`kMaxJointVelocity`)
- Pinocchio FK로 TCP 위치 계산 → `actual_task_positions` 출력
- `UpdateGainsFromMsg` 레이아웃: `[kp×6]` (6개)

### `JointPDController` (direct, index 1)

관절 공간 PD 제어기 + 선택적 중력/코리올리 보상. Pinocchio RNEA 활용. **5차 관절공간 궤적 추종**. E-STOP 시 안전 위치로 이동합니다.

```
τ[i] = ff_vel[i] + Kp[i] * e[i] + Kd[i] * ė[i] [+ g(q)[i]] [+ C(q,v)·v[i]]
```

여기서 `ff_vel` 및 목표 위치는 `JointSpaceTrajectory<6>`에서 실시간으로 계산됩니다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `[200.0, 200.0, 150.0, 120.0, 120.0, 120.0]` | 비례 게인 |
| `kd` | `[30.0, 30.0, 25.0, 20.0, 20.0, 20.0]` | 미분 게인 |
| `enable_gravity_compensation` | `false` | 중력 보상 활성화 |
| `enable_coriolis_compensation` | `false` | 코리올리 보상 활성화 |
| `trajectory_speed` | `1.0` | 궤적 이동 속도 상한 [rad/s] |
| `command_type` | `"torque"` | 출력 타입 |

- `UpdateGainsFromMsg` 레이아웃: `[kp×6, kd×6, gravity(0/1), coriolis(0/1), trajectory_speed]` (15개)
- E-STOP 안전 위치: `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad
- 궤적 지속시간: `max(0.01, max_joint_dist / trajectory_speed)`
- 모든 Eigen 버퍼: 생성자에서 사전 할당 (500Hz 경로에서 힙 할당 없음)
- `target_mutex_` try_lock 패턴: `SetRobotTarget()` 호출 시 궤적 생성과 `Compute()` 루프 간의 레이스 컨디션 방지. 잠금 실패 시 이전 궤적을 계속 추종

### `ClikController` (indirect, index 2)

폐루프 역기구학(CLIK). 감쇠 야코비안 유사역행렬 + 영공간 관절 센터링. **SE(3) 5차 궤적 추종**.

**목표 규약** (`/target_joint_positions`의 6개 값):
```
[x, y, z, null_q3, null_q4, null_q5]
 ─────────── ───────────────────────
 TCP 위치(m)  영공간 참조 관절 3–5 (rad)
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `[1.0×6]` | 위치 비례 게인 |
| `damping` | `0.01` | 감쇠 유사역행렬 λ |
| `enable_null_space` | `true` | 영공간 관절 센터링 활성화 |
| `null_kp` | `0.5` | 영공간 관절 센터링 게인 |
| `trajectory_speed` | `0.1` | TCP 이동 속도 상한 [m/s] |
| `control_6dof` | `false` | 6-DOF 방향 제어 활성화 |
| `command_type` | `"position"` | 출력 타입 |

- `UpdateGainsFromMsg` 레이아웃: `[kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1)]` (10개)
- 궤적 지속시간 (3-DOF): `max(0.01, trans_dist / trajectory_speed)`
- `target_mutex_` try_lock 패턴: `SetRobotTarget()` 호출 시 궤적 생성과 `Compute()` 루프 간의 레이스 컨디션 방지. 잠금 실패 시 이전 궤적을 계속 추종

### `OperationalSpaceController` (direct, index 3)

전체 6-DOF 데카르트 PD 제어 (위치 + SO(3) 방향). Pinocchio `log3()` 사용. **SE(3) 5차 궤적 추종**.

**목표 규약** (`/target_joint_positions`의 6개 값):
```
[x, y, z, roll, pitch, yaw]
 ─────────── ─────────────
 TCP 위치(m)  ZYX 오일러 각 (rad)
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp_pos` | `[1.0×3]` | 위치 비례 게인 |
| `kd_pos` | `[0.1×3]` | 위치 미분 게인 |
| `kp_rot` | `[0.5×3]` | 회전 비례 게인 |
| `kd_rot` | `[0.05×3]` | 회전 미분 게인 |
| `damping` | `0.01` | 감쇠 유사역행렬 λ |
| `enable_gravity_compensation` | `false` | 중력 보상 활성화 (+ g(q)) |
| `trajectory_speed` | `0.1` | TCP 병진 이동 속도 상한 [m/s] |
| `trajectory_angular_speed` | `0.5` | TCP 회전 속도 상한 [rad/s] |
| `command_type` | `"torque"` | 출력 타입 |

- `UpdateGainsFromMsg` 레이아웃: `[kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, gravity(0/1), traj_speed, traj_ang_speed]` (16개)
- 궤적 지속시간: `max(0.01, max(trans_dist / traj_speed, ang_dist / traj_ang_speed))`
- `target_mutex_` try_lock 패턴: `SetRobotTarget()` 호출 시 궤적 생성과 `Compute()` 루프 간의 레이스 컨디션 방지. 잠금 실패 시 이전 궤적을 계속 추종

### `DemoJointController` (indirect, index 4)

로봇 암(6관절) + 핸드(10모터) 통합 P 위치 제어기. PController와 동일한 증분 위치 스텝 방식을 로봇/핸드 각각에 적용합니다.

```
robot_cmd[i] = current_pos[i] + robot_kp[i] * (robot_target[i] - current_pos[i]) * dt
hand_cmd[i]  = hand_pos[i]    + hand_kp[i]  * (hand_target[i]  - hand_pos[i])    * dt
```

**목표 규약** (`/target_joint_positions`):
```
data[0..5]  : 로봇 관절 목표 (rad)
data[6..15] : 핸드 모터 목표 (rad), 선택 사항 — 배열 크기 < 16이면 무시
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_kp` | `[120.0, 120.0, 100.0, 80.0, 80.0, 80.0]` | 로봇 암 비례 게인 |
| `hand_kp` | `[50.0×10]` | 핸드 모터 비례 게인 |
| `command_type` | `"position"` | 출력 타입 |

- 로봇 최대 속도: `2.0 rad/s`, 핸드 최대 속도: `1.0 rad/s`
- `UpdateGainsFromMsg` 레이아웃: `[robot_kp×6, hand_kp×10]` (16개)
- 퍼블리시: `/forward_position_controller/commands` (6) + `/hand/command` (10) + `/rt_controller/current_task_position` (6)

### `DemoTaskController` (indirect, index 5)

Task-space CLIK(로봇 암) + P 제어(핸드) 통합 컨트롤러. ClikController의 감쇠 야코비안 유사역행렬 + 영공간 관절 센터링을 사용하면서 동시에 핸드 10모터를 P 제어합니다.

```
# 로봇 암 (CLIK)
dq      = kp · J^# · pos_error + null_kp · N · (q_null − q)
q_cmd   = q + clamp(dq, ±v_max) * dt

# 핸드
hand_cmd[i] = hand_pos[i] + hand_kp[i] * (hand_target[i] - hand_pos[i]) * dt
```

**목표 규약** (`/target_joint_positions`의 6개 값):
- 3-DOF 모드: `[x, y, z, null_q3, null_q4, null_q5]`
- 6-DOF 모드: `[x, y, z, roll, pitch, yaw]`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `[1.0×6]` | 위치 비례 게인 |
| `damping` | `0.01` | 감쇠 유사역행렬 λ |
| `enable_null_space` | `true` | 영공간 관절 센터링 활성화 |
| `null_kp` | `0.5` | 영공간 관절 센터링 게인 |
| `trajectory_speed` | `0.1` | TCP 이동 속도 상한 [m/s] |
| `control_6dof` | `false` | 6-DOF 방향 제어 활성화 |
| `hand_kp` | `[50.0×10]` | 핸드 모터 비례 게인 |
| `command_type` | `"position"` | 출력 타입 |

- `UpdateGainsFromMsg` 레이아웃: `[kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1), hand_kp×10]` (20개)
- 로봇 최대 속도: `2.0 rad/s`, 핸드 최대 속도: `1.0 rad/s`
- E-STOP 안전 위치: `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad
- `target_mutex_` try_lock 패턴: RT thread blocking 방지

---

## ROS2 인터페이스

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR 드라이버 또는 시뮬레이터에서 6-DOF 위치/속도 |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 궤적 최종 목표 (goal) (컨트롤러별 해석 다름) |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | 핸드 10개 모터 위치 |
| `~/controller_type` | `std_msgs/Int32` | 런타임 컨트롤러 전환 (인덱스 아래 참조) |
| `~/controller_gains` | `std_msgs/Float64MultiArray` | 활성 컨트롤러 게인 동적 업데이트 |
| `~/request_gains` | `std_msgs/Bool` | 현재 게인 퍼블리시 요청 (GUI 연동) |

**컨트롤러 인덱스** (`~/controller_type` 값):

| 인덱스 | 컨트롤러 | 출력 타입 |
|--------|----------|-----------|
| 0 | PController | position |
| 1 | JointPDController | torque |
| 2 | ClikController | position |
| 3 | OperationalSpaceController | torque |
| 4 | DemoJointController | position |
| 5 | DemoTaskController | position |

### 퍼블리시 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 6개 위치 명령 (rad) — indirect 컨트롤러 |
| `/forward_torque_controller/commands` | `std_msgs/Float64MultiArray` | 6개 토크 명령 (Nm) — direct 컨트롤러 |
| `/hand/command` | `std_msgs/Float64MultiArray` | 10개 핸드 모터 명령 (DemoJointController / DemoTaskController) |
| `/rt_controller/current_task_position` | `std_msgs/Float64MultiArray` | 6개 FK 태스크 공간 위치 |
| `/rt_controller/trajectory_state` | `std_msgs/Float64MultiArray` | 궤적 보간 상태 (18: goal[6]+traj_pos[6]+traj_vel[6]) |
| `/rt_controller/controller_state` | `std_msgs/Float64MultiArray` | 제어기 내부 상태 (18: actual_pos[6]+actual_vel[6]+command[6]) |
| `/system/estop_status` | `std_msgs/Bool` | `true` = E-STOP 활성 |
| `~/active_controller_name` | `std_msgs/String` | 활성 컨트롤러 이름 (transient_local QoS) |
| `~/current_gains` | `std_msgs/Float64MultiArray` | `~/request_gains` 응답 |

### 컨트롤러별 토픽 설정

각 컨트롤러 YAML 파일에는 `topics:` 섹션이 있어 구독/퍼블리시 토픽을 개별 설정할 수 있습니다. 노드 시작 시 모든 컨트롤러의 토픽 합집합이 생성됩니다.

```yaml
# 예시: joint_pd_controller.yaml
topics:
  subscribe:
    # 카테고리 1: Goal State
    - topic: "/target_joint_positions"
      role: "goal"
    # 카테고리 2: Current State
    - topic: "/joint_states"
      role: "joint_state"
  publish:
    # 카테고리 3: Control Command
    - topic: "/forward_torque_controller/commands"
      role: "torque_command"
    # 카테고리 4: Logging/Monitoring
    - topic: "/rt_controller/current_task_position"
      role: "task_position"
    - topic: "/rt_controller/trajectory_state"
      role: "trajectory_state"
      data_size: 18
    - topic: "/rt_controller/controller_state"
      role: "controller_state"
      data_size: 18
```

### ROS2 파라미터 인트로스펙션 (v5.13.0)

모든 컨트롤러의 토픽 매핑이 읽기 전용 ROS2 파라미터로 노출됩니다. 런타임에 토픽 구성을 확인할 수 있지만, RT 안전성을 위해 변경은 차단됩니다.

```bash
# 토픽 파라미터 목록 확인
ros2 param list /rt_controller | grep controllers
# 출력 예시:
#   controllers.PController.subscribe.goal
#   controllers.PController.subscribe.joint_state
#   controllers.PController.publish.position_command
#   controllers.PController.publish.task_position
#   controllers.PController.publish.trajectory_state
#   controllers.PController.publish.controller_state
#   controllers.JointPDController.subscribe.goal
#   ...

# 특정 토픽 이름 조회
ros2 param get /rt_controller controllers.PController.subscribe.goal
# String value is: /target_joint_positions

# 런타임 변경 시도 → 거부됨
ros2 param set /rt_controller controllers.PController.subscribe.goal /new_topic
# Setting parameter failed: Topic parameters are read-only after initialisation
```

### ROS2 Topic Remapping (v5.13.0)

표준 ROS2 토픽 리맵핑이 네이티브로 지원됩니다. `create_publisher()`/`create_subscription()` 호출로 토픽이 생성되므로 `--ros-args -r`을 통한 리맵핑이 자동으로 적용됩니다.

```bash
# 토픽 이름 변경 (모든 컨트롤러에 적용)
ros2 run ur5e_rt_controller rt_controller \
  --ros-args -r /target_joint_positions:=/my_target \
             -r /forward_position_controller/commands:=/my_robot/commands

# launch 파일에서 리맵핑
ros2 launch ur5e_rt_controller ur_control.launch.py \
  --ros-args -r /joint_states:=/my_robot/joint_states
```

> **주의**: 리맵핑은 모든 컨트롤러에 동일하게 적용됩니다. 컨트롤러별 토픽 분리가 필요한 경우 컨트롤러 YAML 파일의 `topics:` 섹션을 수정하세요.

---

## 설정

### `config/ur5e_rt_controller.yaml` (노드 레벨)

> **주의**: 코드(`DeclareAndLoadParameters`)가 **플랫 파라미터**만 읽습니다. `estop:` 등 네스트 구조를 사용하면 파라미터가 무시됩니다.

```yaml
/**:
  ros__parameters:
    control_rate: 500.0              # Hz — 타이머 주기 = 1e6/rate µs
    initial_controller: "joint_pd_controller"  # 레지스트리 키 또는 클래스명

    init_timeout_sec: 5.0            # 초기화 타임아웃 (초) — 초과 시 E-STOP + 셧다운

    # E-STOP (플랫 파라미터 — 네스트 금지)
    enable_estop: true
    robot_timeout_ms: 100.0          # /joint_states 갭 초과 시 E-STOP (ms)
    hand_timeout_ms: 200.0           # 0.0 = 핸드 타임아웃 비활성화

    # 로깅 (플랫 파라미터 — 네스트 금지)
    enable_logging: true
    log_dir: ""                   # 세션 디렉토리 — launch 파일이 YYMMDD_HHMM 경로 설정
    max_log_sessions: 10          # 최대 보관 세션 폴더 수
    enable_timing_log: true       # 타이밍 CSV 로깅 활성화
    enable_robot_log: true        # 로봇 관절 CSV 로깅 활성화
    enable_hand_log: true         # 핸드 CSV 로깅 활성화

    # Status Monitor (파라미터는 ur5e_status_monitor 패키지 YAML에서 로드)
    enable_status_monitor: false     # 실제 로봇 시 true 권장
```

### 컨트롤러별 YAML (`config/controllers/{indirect,direct}/`)

각 컨트롤러는 개별 YAML 파일에서 게인, 출력 타입, 토픽을 정의합니다:

| 파일 | 주요 파라미터 |
|------|-------------|
| `indirect/p_controller.yaml` | `kp[6]`, `command_type: "position"` |
| `indirect/clik_controller.yaml` | `kp[6]`, `damping`, `enable_null_space`, `null_kp`, `trajectory_speed`, `control_6dof` |
| `indirect/ur5e_hand_controller.yaml` | `robot_kp[6]`, `hand_kp[10]`, `command_type: "position"` |
| `direct/joint_pd_controller.yaml` | `kp[6]`, `kd[6]`, `enable_gravity_compensation`, `enable_coriolis_compensation`, `trajectory_speed` |
| `direct/operational_space_controller.yaml` | `kp_pos[3]`, `kd_pos[3]`, `kp_rot[3]`, `kd_rot[3]`, `damping`, `enable_gravity_compensation` |

---

## 실행

### 실제 로봇 (UR 드라이버 사용)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10
```

### 가상 하드웨어 (로봇 불필요)

```bash
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_ip` | `192.168.1.10` | UR 로봇 IP |
| `use_fake_hardware` | `false` | 가상 하드웨어 모드 |
| `use_cpu_affinity` | `true` | UR 드라이버 Core 0-1 taskset 자동 적용 (3초 후) |

런치 파일이 자동 설정하는 환경변수:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` → `config/cyclone_dds.xml` (DDS recv/send 스레드 Core 0-1 제한)
- `UR5E_SESSION_DIR` → `logging_data/YYMMDD_HHMM` 세션 디렉토리 경로 (및 `log_dir` 파라미터 자동 설정)

런치 파일이 로드하는 설정 파일:
- `ur5e_rt_controller/config/ur5e_rt_controller.yaml` — 노드 레벨 파라미터
- `ur5e_status_monitor/config/ur5e_status_monitor.yaml` — Status Monitor 파라미터 (`status_monitor.*`)
- `ur5e_hand_udp/config/hand_udp_node.yaml` — Hand UDP 설정 (IP, 포트, sensor_decimation 등)

런치 시 함께 실행되는 노드:
- `rt_controller` — RT 제어기 노드

---

## 빌드

```bash
cd ~/ros2_ws/ur5e_ws
colcon build --packages-select ur5e_rt_base ur5e_rt_controller --symlink-install
source install/setup.bash
```

---

## 모니터링

```bash
# 제어 주기 확인 (약 500Hz)
ros2 topic hz /forward_position_controller/commands

# E-STOP 상태 확인
ros2 topic echo /system/estop_status

# 활성 컨트롤러 확인
ros2 topic echo /rt_controller/active_controller_name

# 현재 게인 조회
ros2 topic pub --once ~/request_gains std_msgs/msg/Bool "data: true"
ros2 topic echo ~/current_gains

# 토픽 파라미터 확인
ros2 param list /rt_controller | grep controllers
ros2 param get /rt_controller controllers.PController.subscribe.goal

# RT 스레드 확인
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

### CSV 로그 분석

v5.10.0부터 세션 디렉토리(`logging_data/YYMMDD_HHMM/controller/`) 내에 3개 파일로 분리됩니다 (`max_log_sessions: 10`개 세션 폴더 보관, 각 파일별 `enable_*_log` 파라미터로 개별 활성화):

- `logging_data/YYMMDD_HHMM/controller/timing_log.csv`
- `logging_data/YYMMDD_HHMM/controller/robot_log.csv`
- `logging_data/YYMMDD_HHMM/controller/hand_log.csv`

**timing_log CSV (6 columns):**

| 컬럼 | 설명 |
|------|------|
| `timestamp` | 타임스탬프 (초) |
| `t_state_acquire_us` | 상태 데이터 획득 소요 시간 |
| `t_compute_us` | Compute() 연산 소요 시간 |
| `t_publish_us` | 명령 퍼블리시 소요 시간 |
| `t_total_us` | 전체 제어 루프 소요 시간 |
| `jitter_us` | 제어 주기 지터 |

**robot_log CSV (49 columns, 4-카테고리):**

| 컬럼 | 카테고리 | 설명 |
|------|---------|------|
| `timestamp` | — | 타임스탬프 (초) |
| `goal_pos_0..5` | 1. Goal | 최종 목표 위치 (SetRobotTarget) |
| `actual_pos_0..5` | 2. State | 실제 관절 위치 |
| `actual_vel_0..5` | 2. State | 실제 관절 속도 |
| `actual_torque_0..5` | 2. State | 실제 관절 토크 |
| `task_pos_0..5` | 2. State | TCP 태스크 공간 위치 |
| `command_0..5` | 3. Command | 제어 출력 명령 (위치 또는 토크) |
| `command_type` | 3. Command | 0=position, 1=torque |
| `traj_pos_0..5` | 4. Trajectory | 궤적 보간 위치 |
| `traj_vel_0..5` | 4. Trajectory | 궤적 보간 속도 |

**hand_log CSV (87 columns, 4-카테고리):**

| 컬럼 | 카테고리 | 설명 |
|------|---------|------|
| `timestamp` | — | 타임스탬프 (초) |
| `hand_valid` | — | 핸드 데이터 유효성 (0/1) |
| `hand_goal_pos_0..9` | 1. Goal | 핸드 최종 목표 위치 |
| `hand_actual_pos_0..9` | 2. State | 핸드 실제 위치 |
| `hand_actual_vel_0..9` | 2. State | 핸드 실제 속도 |
| `baro_f{f}_{b}` | 2. State | 손가락 f 기압 센서 b (4×8=32) |
| `tof_f{f}_{t}` | 2. State | 손가락 f ToF 센서 t (4×3=12) |
| `hand_cmd_0..9` | 3. Command | 핸드 명령 |

**오버런 감지 (v5.16.0)**: 3단계 오버런 추적 — `overrun_count_` (RT loop tick 지연), `compute_overrun_count_` (ControlLoop 실행시간 초과), `skip_count_` (놓친 tick 수). 연속 10회 overrun 시 `TriggerGlobalEstop("consecutive_overrun")`. 로그 요약에 `pub_drops` (publish 버퍼 drop) 추가.

```python
import pandas as pd, glob, os
log_dir = os.path.expanduser('~/ros2_ws/ur5e_ws/logging_data/')
sessions = sorted(glob.glob(log_dir + '??????_????'))
if sessions:
    df = pd.read_csv(os.path.join(sessions[-1], 'controller', 'timing_log.csv'))
    print(df['t_compute_us'].describe())
    print(f'P95: {df["t_total_us"].quantile(0.95):.1f} us')
    print(f'P99: {df["t_total_us"].quantile(0.99):.1f} us')
    print(f'Jitter P99: {df["jitter_us"].quantile(0.99):.1f} us')
    print(f'Overruns: {(df["t_total_us"] > 2000).sum()}')
```

---

## 목표 위치 수동 퍼블리시

```bash
# 관절 공간 목표 (P / JointPD / UrFiveEHandController)
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
  "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"

# 데카르트 목표 (CLIK — [x, y, z, null_q3, null_q4, null_q5])
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
  "data: [0.3, -0.2, 0.5, 0.0, -1.57, 0.0]"

# 데카르트 목표 (OSC — [x, y, z, roll, pitch, yaw])
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
  "data: [0.3, -0.2, 0.5, 0.0, 3.14, 0.0]"

# 컨트롤러 전환 (예: CLIK으로 전환)
ros2 topic pub --once ~/controller_type std_msgs/msg/Int32 "data: 2"
```

---

## 궤적 생성 서브시스템 (`trajectory/`)

v5.3.0에서 추가된 헤더-전용 5차 다항식 궤적 생성 라이브러리입니다. JointPD/CLIK/OSC 제어기에서 새 목표 수신 시 부드러운 이동을 자동으로 생성합니다.

### `QuinticPolynomial` (`trajectory/trajectory_utils.hpp`)

스칼라 5차 다항식 — 위치·속도·가속도 경계 조건 6개를 모두 만족하는 계수를 계산합니다.

```cpp
QuinticPolynomial poly;
poly.compute_coefficients(p0, v0, a0, pf, vf, af, T);  // (시작, 목표, 지속시간)
auto s = poly.compute(t);  // TrajectoryState{pos, vel, acc} — t를 [0,T]로 클램프
```

### `TaskSpaceTrajectory` (`trajectory/task_space_trajectory.hpp`)

SE(3) 공간에서의 5차 스플라인 궤적. Pinocchio `log6`/`exp6`를 사용하여 두 SE(3) 포즈 사이를 부드럽게 보간합니다.
**CLIK 및 OSC 제어기**에서 `SetRobotTarget()` 호출 시 자동으로 생성됩니다.

```cpp
traj_.initialize(start_pose, pinocchio::Motion::Zero(),
                 goal_pose,  pinocchio::Motion::Zero(), duration);
auto state = traj_.compute(trajectory_time_);  // State{pose, velocity, acceleration}
```

- **CLIK**: `duration = max(0.01, trans_dist / trajectory_speed)`
- **OSC**: `duration = max(0.01, max(trans_dist / traj_speed, ang_dist / traj_ang_speed))`

힙 할당 없음 (`QuinticPolynomial × 6`, 고정 크기 배열).

### `JointSpaceTrajectory<N>` (`trajectory/joint_space_trajectory.hpp`)

N-DOF 관절공간 5차 스플라인 (템플릿). N개 관절을 독립적으로 보간합니다.
**JointPDController**에서 사용됩니다.

```cpp
JointSpaceTrajectory<6> traj;
JointSpaceTrajectory<6>::State start{q0, v0, {}};
JointSpaceTrajectory<6>::State goal {qf, {}, {}};
traj.initialize(start, goal, duration);
auto s = traj.compute(t);  // State{positions, velocities, accelerations}
```

- **JointPDController**: `duration = max(0.01, max_joint_dist / trajectory_speed)`

---

## 커스텀 컨트롤러 추가

v5.4.0부터 **Controller Registry** 패턴을 사용합니다. 새 컨트롤러를 추가할 때 수정해야 할 파일이 최소화되었습니다.

### 4단계 요약

**1. 헤더 작성** (`include/ur5e_rt_controller/controllers/{indirect,direct}/my_controller.hpp`)

```cpp
#pragma once
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include <yaml-cpp/yaml.h>

namespace ur5e_rt_controller {
class MyController final : public RTControllerInterface {
public:
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;
  void SetRobotTarget(std::span<const double, kNumRobotJoints> target) noexcept override;
  void SetHandTarget(std::span<const float, kNumHandMotors> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override { return "MyController"; }

  void TriggerEstop() noexcept override;
  void ClearEstop()   noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override;
};
} // namespace ur5e_rt_controller
```

**2. 구현 작성** (`src/controllers/{indirect,direct}/my_controller.cpp`) — `Compute()`, `LoadConfig()`, `UpdateGainsFromMsg()`, `GetCurrentGains()` 등 구현

**3. YAML 설정 추가** (`config/controllers/{indirect,direct}/my_controller.yaml`)

```yaml
my_controller:
  kp: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
  command_type: "position"    # 또는 "torque"

  topics:
    subscribe:
      - topic: "/joint_states"
        role: "joint_state"
      - topic: "/target_joint_positions"
        role: "target"
    publish:
      - topic: "/forward_position_controller/commands"
        role: "position_command"
```

**4. Registry에 등록** (`src/rt_controller_node.cpp` → `MakeControllerEntries()`)

```cpp
{"my_controller", "indirect/", [](const std::string & p) {
  return std::make_unique<urtc::MyController>(p);
}},
// ── Add new controllers here ──
```

> 자세한 전체 가이드는 [`docs/ADDING_CONTROLLER.md`](docs/ADDING_CONTROLLER.md)를 참조하세요.

### RTControllerInterface 가상 메서드

| 메서드 | noexcept | 설명 |
|--------|----------|------|
| `Compute()` | ✓ | 500Hz RT 루프에서 호출 — 반드시 noexcept |
| `SetRobotTarget()` | ✓ | 목표 관절 위치 설정 (6-DOF) |
| `SetHandTarget()` | ✓ | 핸드 목표 설정 (10 모터, float) |
| `Name()` | ✓ | 컨트롤러 이름 반환 |
| `TriggerEstop()` / `ClearEstop()` | ✓ | E-STOP 제어 (기본 no-op) |
| `GetCommandType()` | ✓ | `kPosition` 또는 `kTorque` 반환 |
| `GetTopicConfig()` | ✓ | 컨트롤러별 토픽 설정 반환 |
| `LoadConfig()` | ✗ | YAML 파싱 (예외 발생 가능, 호출 측에서 try/catch) |
| `UpdateGainsFromMsg()` | ✓ | 센서 스레드에서 호출 — 반드시 noexcept |
| `GetCurrentGains()` | ✓ | 현재 게인 반환 (GUI 연동) |

---

## 스크립트

### `setup_irq_affinity.sh` — NIC IRQ 친화도 설정

네트워크 인터페이스 인터럽트를 Core 0-1에 고정하여 RT 코어에 대한 IRQ 간섭을 방지합니다.

```bash
sudo ./scripts/setup_irq_affinity.sh            # 자동 NIC 감지
sudo ./scripts/setup_irq_affinity.sh enp3s0     # NIC 지정
```

### `setup_udp_optimization.sh` — UDP/네트워크 최적화

NIC 인터럽트 코얼레싱 비활성화, 오프로드 기능 해제, 커널 UDP 버퍼 크기 증가 등을 적용합니다.

```bash
sudo ./scripts/setup_udp_optimization.sh         # 자동 NIC 감지
```

### `setup_nvidia_rt.sh` — NVIDIA + RT 커널 공존

NVIDIA GPU(디스플레이 전용)와 PREEMPT_RT 커널이 공존하도록 설정합니다. IRQ 친화도, isolcpus, nohz_full 등 포괄적으로 구성합니다.

```bash
sudo ./scripts/setup_nvidia_rt.sh
```

### `build_rt_kernel.sh` — RT 커널 빌드

PREEMPT_RT 패치를 적용한 Linux 커널을 자동으로 다운로드, 패치, 빌드, 설치합니다.

```bash
sudo ./scripts/build_rt_kernel.sh                # 대화형 (menuconfig 포함)
sudo ./scripts/build_rt_kernel.sh --batch        # 비대화형
```

---

## 성능 특성

| 지표 | v4.2.0 이전 | v4.2.0+ | 개선 |
|------|------------|---------|------|
| 제어 지터 | ~500μs | <50μs | 10배 |
| E-STOP 응답 | ~100ms | <20ms | 5배 |
| CPU 사용률 | ~30% | ~25% | -17% |
| 컨텍스트 스위치 | ~5000/s | ~1000/s | -80% |

### 지터 검증 (cyclictest)

```bash
sudo apt install rt-tests
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo \
    --interval=2000 --loops=100000 --affinity=2 --histogram=200
# 목표: 최대 지터 < 50μs
```

---

## RT 권한 요구사항

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필요
```

자세한 RT 튜닝 가이드는 `docs/RT_OPTIMIZATION.md`를 참조하세요.

---

## 라이선스

MIT License
