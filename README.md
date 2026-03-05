# UR5e RT Controller

**Ubuntu 22.04 + ROS2 Humble | 실시간 UR5e 제어기 + 커스텀 핸드 통합 (v5.0.0)**

E-STOP 안전 시스템, PD 제어기, **Pinocchio 기반 모델 제어기 3종**, **MuJoCo 3.x 물리 시뮬레이터**, UDP 핸드 인터페이스, CSV 데이터 로깅, Qt GUI 모션 에디터를 포함한 완전한 실시간 제어 솔루션입니다.

> **v5.0.0 (멀티-패키지 분리)**: 단일 패키지에서 **5개 독립 ROS2 패키지**로 리팩터링되었습니다. 각 패키지는 `src/` 디렉터리 아래에 위치하며 각자의 `README.md`와 `CHANGELOG.md`를 포함합니다.

---

## 목차

- [기능 요약](#기능-요약)
- [프로젝트 구조](#프로젝트-구조)
- [아키텍처 개요](#아키텍처-개요)
- [MuJoCo 시뮬레이터](#mujoco-시뮬레이터)
- [Pinocchio 기반 제어기](#pinocchio-기반-제어기)
- [설치 방법](#설치-방법)
- [사용 방법](#사용-방법)
- [설정 (YAML)](#설정-yaml)
- [ROS2 토픽 인터페이스](#ros2-토픽-인터페이스)
- [UDP 핸드 프로토콜](#udp-핸드-프로토콜)
- [성능 지표](#성능-지표)
- [문제 해결](#문제-해결)
- [모니터링](#모니터링)
- [고급 사용법](#고급-사용법)

---

## 기능 요약

| 기능 | 설명 |
|------|------|
| 실시간 제어 | 500Hz PD 제어 루프 (`custom_controller`) |
| 병렬 컴퓨팅 | CallbackGroup 기반 멀티스레드 executor (v4.2.0+) |
| E-STOP 시스템 | 로봇/핸드 데이터 타임아웃 자동 감지 및 비상 정지 |
| 모델 기반 제어 | Pinocchio 라이브러리 활용 — 중력 보상, CLIK, 작업공간 제어 (v4.3.0+) |
| MuJoCo 시뮬레이션 | FreeRun / SyncStep 모드, GLFW 뷰어, RTF 측정 (v4.4.0+) |
| 인터랙티브 뷰어 | 마우스 카메라, 키보드 단축키, Ctrl+드래그 물체 힘 인가, F1 도움말 (v4.5.0+) |
| Solver 제어 | runtime에 integrator / solver type / iterations / tolerance 조정 (v4.5.0+) |
| 커스텀 핸드 통합 | UDP 기반 11-DOF 핸드 데이터 수신/송신 |
| 데이터 로깅 | CSV 형식의 제어 데이터 실시간 기록 (`DataLogger` + SPSC 링 버퍼) |
| 데이터 시각화 | Matplotlib 기반 관절 궤적 플롯 (`plot_ur_trajectory.py`) |
| 데이터 헬스 모니터 | 패킷 손실/타임아웃 통계 수집 및 JSON 내보내기 |
| Qt GUI 에디터 | 50개 포즈 저장/로드/재생 모션 에디터 |
| Strategy Pattern | `RTControllerInterface`를 상속하는 교체 가능한 제어기 구조 |
| 설치 모드 선택 | `install.sh sim / robot / full` — 환경에 맞게 선택 설치 (v4.5.0+) |

---

## 프로젝트 구조

v5.0.0부터 **5개 독립 ROS2 패키지**로 분리되어 `src/` 디렉터리 아래에 위치합니다.

```
ur5e-rt-controller/
├── README.md                              # 이 문서
├── install.sh                             # 자동 설치 스크립트 (5개 패키지 빌드)
├── requirements.txt                       # Python 의존성 목록
│
├── docs/
│   ├── CHANGELOG.md                      # 전체 버전 변경 이력
│   └── RT_OPTIMIZATION.md                # 실시간 최적화 가이드
│
└── src/                                   # ROS2 패키지 루트
    │
    ├── ur5e_rt_base/                      # 📦 공유 기반 (헤더-전용)
    │   ├── include/ur5e_rt_base/
    │   │   ├── types.hpp                 # 공유 타입: RobotState, HandState, ControllerState...
    │   │   ├── thread_config.hpp         # ThreadConfig + 사전 정의 RT 상수
    │   │   ├── thread_utils.hpp          # ApplyThreadConfig(), VerifyThreadConfig()
    │   │   ├── log_buffer.hpp            # SPSC 링 버퍼 (RT→로그, 잠금-없음)
    │   │   └── data_logger.hpp           # 비-RT CSV 로거
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   └── CHANGELOG.md
    │
    ├── ur5e_rt_controller/                # 📦 500Hz 실시간 제어기
    │   ├── include/ur5e_rt_controller/
    │   │   ├── rt_controller_interface.hpp        # 추상 기반 클래스 (Strategy Pattern)
    │   │   ├── controller_timing_profiler.hpp     # 잠금-없는 Compute() 타이밍 프로파일러
    │   │   └── controllers/
    │   │       ├── pd_controller.hpp              # PD + E-STOP (기본값)
    │   │       ├── p_controller.hpp               # 단순 P 제어기
    │   │       ├── pinocchio_controller.hpp       # PD + 중력/코리올리 보상
    │   │       ├── clik_controller.hpp            # 폐루프 IK (3-DOF)
    │   │       └── operational_space_controller.hpp # 6-DOF 데카르트 PD
    │   ├── src/custom_controller.cpp              # 메인 500Hz 노드
    │   ├── config/ur5e_rt_controller.yaml
    │   ├── launch/ur_control.launch.py
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   └── CHANGELOG.md
    │
    ├── ur5e_hand_udp/                     # 📦 UDP 손 브리지
    │   ├── include/ur5e_hand_udp/
    │   │   ├── hand_udp_receiver.hpp     # UDP 수신 (jthread, 포트 50001)
    │   │   └── hand_udp_sender.hpp       # UDP 송신 (리틀 엔디언, 포트 50002)
    │   ├── src/
    │   │   ├── hand_udp_receiver_node.cpp
    │   │   └── hand_udp_sender_node.cpp
    │   ├── config/hand_udp_receiver.yaml
    │   ├── launch/hand_udp.launch.py
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   └── CHANGELOG.md
    │
    ├── ur5e_mujoco_sim/                   # 📦 MuJoCo 3.x 시뮬레이터 (선택적)
    │   ├── include/ur5e_mujoco_sim/
    │   │   └── mujoco_simulator.hpp      # 스레드 안전 물리 래퍼
    │   ├── src/mujoco_simulator_node.cpp
    │   ├── models/ur5e/
    │   │   ├── scene.xml
    │   │   └── ur5e.xml
    │   ├── config/mujoco_simulator.yaml
    │   ├── launch/mujoco_sim.launch.py
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   └── CHANGELOG.md
    │
    └── ur5e_tools/                        # 📦 Python 개발 유틸리티
        ├── scripts/
        │   ├── plot_ur_trajectory.py     # Matplotlib 궤적 시각화
        │   ├── monitor_data_health.py    # 데이터 건강 모니터 + JSON 통계
        │   ├── motion_editor_gui.py      # Qt5 50-포즈 모션 편집기
        │   └── hand_udp_sender_example.py # 합성 UDP 손 데이터 생성기
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        └── CHANGELOG.md
```

### 패키지 의존성 그래프

```
ur5e_rt_base       ← 독립 (공유 기반, 헤더-전용)
    ↑
    ├── ur5e_rt_controller  ← ur5e_rt_base
    └── ur5e_hand_udp       ← ur5e_rt_base

ur5e_mujoco_sim    ← 독립 (MuJoCo/GLFW/stdlib만 사용)
ur5e_tools         ← 독립 (Python 전용, rclpy)
```

---

## 아키텍처 개요

### 제어 흐름

```
[UR5e 로봇]
    │  /joint_states (sensor_msgs/JointState)
    ▼
[custom_controller]  ←──  /target_joint_positions (std_msgs/Float64MultiArray)
    │  PDController::compute_command()
    │  DataLogger::log_control_data()
    │  E-STOP 감시 (check_timeouts @ 50Hz)
    ▼
/forward_position_controller/commands ──► [UR 드라이버]
    │
    ▼
/system/estop_status (std_msgs/Bool) ──► [모니터링]

[커스텀 핸드]
    │  UDP 패킷 (포트 50001)
    ▼
[hand_udp_receiver_node]
    │  /hand/joint_states (std_msgs/Float64MultiArray)
    ▼
[custom_controller]  ──►  E-STOP 핸드 감시

[hand_udp_sender_node]
    │  /hand/command 구독
    ▼
[커스텀 핸드]  ◄── UDP 패킷 (포트 50002)
```

### 주요 클래스

#### `CustomController` (`src/custom_controller.cpp`)
500Hz 제어 루프를 실행하는 메인 ROS2 노드. v4.2.0부터 4개 CallbackGroup으로 분리된 멀티스레드 executor 지원.

| 멤버 | 타입 | 역할 |
|------|------|------|
| `controller_` | `PDController` | PD 제어 계산 |
| `logger_` | `DataLogger` | CSV 로깅 (log 스레드 전용) |
| `log_buffer_` | `ControlLogBuffer` | SPSC 링 버퍼 — RT→log 스레드 전달 |
| `target_snapshot_` | `array<double,6>` | RT 루프 전용 타겟 복사본 |
| `control_timer_` | `rclcpp::TimerBase` | 500Hz 제어 루프 |
| `timeout_timer_` | `rclcpp::TimerBase` | 50Hz E-STOP 감시 |
| `drain_timer_` | `rclcpp::TimerBase` | 100Hz 링 버퍼 → CSV 드레인 (log 스레드) |
| `rt_command_pub_` | `RealtimePublisher` | RT-safe 위치 명령 퍼블리셔 |
| `state_received_` | `atomic<bool>` | 로봇 데이터 수신 플래그 |
| `target_received_` | `atomic<bool>` | 타겟 수신 플래그 |
| `hand_data_received_` | `atomic<bool>` | 핸드 데이터 수신 플래그 |
| `cb_group_rt_` | CallbackGroup | RT 제어 루프 (Core 2, SCHED_FIFO 90) |
| `cb_group_sensor_` | CallbackGroup | 센서 데이터 수신 (Core 3, SCHED_FIFO 70) |
| `cb_group_log_` | CallbackGroup | 로깅 작업 (Core 4, SCHED_OTHER) |
| `cb_group_aux_` | CallbackGroup | 보조 작업 (Core 5, SCHED_OTHER) |

파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `control_rate` | `500.0` | 제어 주파수 (Hz) |
| `kp` | `5.0` | P 게인 |
| `kd` | `0.5` | D 게인 |
| `enable_logging` | `true` | CSV 로깅 활성화 |
| `robot_timeout_ms` | `100.0` | 로봇 데이터 타임아웃 (ms) |
| `hand_timeout_ms` | `200.0` | 핸드 데이터 타임아웃 (ms) |
| `enable_estop` | `true` | E-STOP 활성화 |

#### `RTControllerInterface` (`include/ur5e_rt_controller/rt_controller_interface.hpp`)
제어기 Strategy Pattern의 추상 기반 클래스. 6-DOF 로봇 + 11-DOF 핸드 통합 상태 관리.

```cpp
namespace ur5e_rt_controller {
  struct RobotState { Eigen::VectorXd q{6}, qd{6}; Eigen::Vector3d tcp_pos; };
  struct HandState  { Eigen::VectorXd motor_pos{11}, motor_vel{11}, motor_current{11}, sensor_data{44}; };
  struct ControllerOutput { Eigen::VectorXd robot_cmd{6}, hand_cmd{11}; };

  class RTControllerInterface {
    virtual ControllerOutput compute(const ControllerState& state) noexcept = 0;
  };
}
```

#### `PDController` (`include/ur5e_rt_controller/controllers/pd_controller.hpp`)
비례-미분 제어기. E-STOP 발생 시 안전 위치 `[0, -1.57, 1.57, -1.57, -1.57, 0]`로 이동.

#### `DataLogger` (`include/ur5e_rt_controller/data_logger.hpp`)
이동 불가 복사 비허용 CSV 로거. 타임스탬프, 현재/목표 위치, 명령값 기록.
`DrainBuffer(ControlLogBuffer&)` 메서드로 SPSC 링 버퍼를 소진하여 파일에 씀 — 파일 I/O는 log 스레드(Core 4)에서만 발생.

#### `ControlLogBuffer` (`include/ur5e_rt_controller/log_buffer.hpp`)
`SpscLogBuffer<512>` 기반 lock-free 단일 생산자/단일 소비자 링 버퍼.
RT 스레드(생산자)가 `Push()`로 `LogEntry`를 넣으면, log 스레드(소비자)가 `Pop()`으로 꺼내 파일에 씀. 버퍼가 가득 차면 해당 엔트리를 드롭(RT 지터 없음).

---

## MuJoCo 시뮬레이터

v4.4.0+에서 MuJoCo 3.x 물리 엔진을 사용하는 시뮬레이터가 추가되었습니다. 실제 UR 드라이버 없이 제어기를 개발하고 검증할 수 있습니다.

### 시뮬레이션 모드

| 모드 | 설명 | 사용 사례 |
|---|---|---|
| `free_run` | 최대 속도로 물리 스텝 실행 (max_rtf로 제한 가능) | 알고리즘 검증, 빠른 반복 |
| `sync_step` | 상태 발행 → 명령 대기 → 1스텝 동기화 | 지연 측정, 실제 루프와 1:1 매핑 |

### 빠른 시작

```bash
# MuJoCo 설치 (sim/full 모드 install.sh가 자동 설치)
./install.sh sim

# Free-run 시뮬레이션 (뷰어 창 자동 오픈)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py

# Sync-step 모드
ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=sync_step

# 헤드리스 (뷰어 없음, 서버 환경)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

# 외부 MJCF 모델 사용 (MuJoCo Menagerie 등)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

# 시뮬레이션 상태 확인
ros2 topic echo /sim/status    # [step_count, sim_time_sec, rtf, paused]
```

### 뷰어 단축키 (v4.5.0+)

| 키 | 기능 |
|---|---|
| **F1** | 도움말 오버레이 (모든 키 + 현재 ON/OFF 상태) |
| Space | 일시정지 / 재개 |
| + / - | RTF 속도 2배 / 0.5배 |
| R | 초기 자세로 리셋 |
| G | 중력 ON/OFF |
| N | 접촉 제약 ON/OFF |
| I | integrator 순환 (Euler→RK4→Implicit→ImplFast) |
| S | solver 순환 (PGS→CG→Newton) |
| ] / [ | solver 반복 횟수 ×2 / ÷2 |
| C / F | 접촉점 / 힘 화살표 표시 |
| V / T | 충돌 지오메트리 / 투명 모드 |
| F3 / F4 | RTF 프로파일러 / solver 통계 오버레이 |
| Ctrl + Left drag | 물체에 스프링 힘 인가 |

### Physics Solver 런타임 제어 (v4.5.0+)

```cpp
auto sim = std::make_unique<MuJoCoSimulator>(cfg);
sim->Initialize();
sim->Start();

// Integrator 변경
sim->SetIntegrator(mjINT_IMPLICIT);   // 강성 시스템에 더 안정적

// Solver 변경
sim->SetSolverType(mjSOL_NEWTON);     // 가장 정확 (기본값)
sim->SetSolverIterations(200);        // 반복 횟수 증가

// 중력 / 접촉 토글
sim->EnableGravity(false);            // 무중력 테스트
sim->SetContactEnabled(false);        // 자유 공간 운동 테스트

// 외부 힘 인가
sim->SetExternalForce(body_id, {0.0, 0.0, 10.0, 0.0, 0.0, 0.0});  // 10N 수직

// Solver 통계 확인
auto stats = sim->GetSolverStats();
printf("iter=%d  improvement=%.3e\n", stats.iter, stats.improvement);
```

### Config 설정

```cpp
MuJoCoSimulator::Config cfg{
    .model_path        = "/path/to/scene.xml",
    .mode              = MuJoCoSimulator::SimMode::kFreeRun,
    .enable_viewer     = true,
    .max_rtf           = 5.0,            // 실시간 5배 속도
    .integrator_type   = mjINT_EULER,    // 기본값
    .solver_type       = mjSOL_NEWTON,   // 기본값
    .solver_iterations = 100,
    .solver_tolerance  = 1e-8,
    .initial_qpos      = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0},
};
```

---

## Pinocchio 기반 제어기

v4.3.0에서 [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 강체 동역학 라이브러리를 활용하는 모델 기반 제어기 3종이 추가되었습니다.
모두 `RTControllerInterface`를 구현하며 `PDController`와 **한 줄 교체**가 가능합니다.
생성자에서 URDF를 로드한 후 500 Hz 경로에서는 사전 할당된 Eigen 버퍼만 재사용하므로 힙 할당이 발생하지 않습니다.

### 제어기 비교

| 제어기 | 파일 | 타겟 입력 | 제어 공간 | 주요 기능 |
|--------|------|-----------|-----------|-----------|
| `PinocchioController` | `pinocchio_controller.hpp` | 관절 각도 6개 (rad) | 관절공간 | 중력 보상 g(q) + 선택적 코리올리 C(q,v)·v |
| `ClikController` | `clik_controller.hpp` | [x, y, z, null_q3, null_q4, null_q5] | Cartesian 위치 (3-DOF) | 감쇠 유사역행렬 J^#, null-space 관절 중심화 |
| `OperationalSpaceController` | `operational_space_controller.hpp` | [x, y, z, roll, pitch, yaw] | Cartesian 위치+자세 (6-DOF) | SO(3) 자세 오차, 태스크공간 속도 댐핑 |

### 제어 법칙 요약

**PinocchioController** (관절공간 모델 기반 PD):
```
command[i] = Kp * e[i]  +  Kd * ė[i]  +  g(q)[i]  [+  C(q,v)·v[i]]
```

**ClikController** (Closed-Loop Inverse Kinematics):
```
J_pos^#  = J_pos^T (J_pos J_pos^T + λ²I)^{-1}
N        = I − J_pos^# J_pos            ← null-space projector
dq       = kp * J_pos^# * pos_error  +  null_kp * N * (q_null − q)
q_cmd    = q + clamp(dq, ±v_max) * dt
```

**OperationalSpaceController** (작업공간 PD + SO(3) 자세 제어):
```
pos_err     = p_des − FK(q)
rot_err     = log₃(R_des * R_FK(q)^T)      ← Pinocchio SO(3) 로그맵
task_vel    = [Kp_pos * pos_err − Kd_pos * J·dq ;
               Kp_rot * rot_err − Kd_rot * J·dq]
J^#         = J^T (J J^T + λ²I₆)^{-1}
q_cmd       = q + clamp(J^# * task_vel, ±v_max) * dt
```

### 사용 방법 (custom_controller.cpp 교체)

```cpp
// 1. 헤더 교체 (기존 pd_controller.hpp 대신)
#include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"
// 또는
#include "ur5e_rt_controller/controllers/clik_controller.hpp"
// 또는
#include "ur5e_rt_controller/controllers/operational_space_controller.hpp"

// 2. controller_ 멤버 타입 변경 (≈line 340)
std::unique_ptr<urtc::RTControllerInterface> controller_;

// 3. 생성자 초기화
// PinocchioController:
controller_(std::make_unique<urtc::PinocchioController>(
    "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",
    urtc::PinocchioController::Gains{.kp = 5.0, .kd = 0.5,
                                      .enable_gravity_compensation = true}))

// ClikController:
controller_(std::make_unique<urtc::ClikController>(
    "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",
    urtc::ClikController::Gains{.kp = 1.0, .damping = 0.01, .null_kp = 0.5}))

// OperationalSpaceController:
controller_(std::make_unique<urtc::OperationalSpaceController>(
    "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",
    urtc::OperationalSpaceController::Gains{
        .kp_pos = 1.0, .kd_pos = 0.1,
        .kp_rot = 0.5, .kd_rot = 0.05, .damping = 0.01}))

// 4. DeclareAndLoadParameters()에서 set_gains() 호출 제거
```

### CLIK / OSC 타겟 퍼블리시 예시

```bash
# ClikController — TCP를 [0.3, 0.2, 0.5] m로 이동 (null-space는 기본값)
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
    "data: [0.3, 0.2, 0.5, -1.57, -1.57, 0.0]"

# OperationalSpaceController — TCP 위치 + 자세 동시 지정 (ZYX 오일러, rad)
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
    "data: [0.3, 0.2, 0.5, 0.0, 0.0, 1.57]"
```

### RT 안전성 설계

| 항목 | PinocchioController | ClikController | OperationalSpaceController |
|------|-------------------|----------------|---------------------------|
| 힙 할당 (RT 경로) | 없음 | 없음 | 없음 |
| 행렬 분해 | `LDLT<Matrix3d>` (3×3 고정) | `LDLT<Matrix3d>` (3×3 고정) | `PartialPivLU<Matrix6d>` (6×6 고정) |
| E-STOP | `kSafePosition`으로 수렴 | `kSafePosition`으로 수렴 | `kSafePosition`으로 수렴 |
| noexcept | 모든 public 메서드 | 모든 public 메서드 | 모든 public 메서드 |

---

## 설치 방법

### 1. 사전 요구사항

- Ubuntu 22.04 LTS
- ROS2 Humble
- (권장) LowLatency 또는 PREEMPT_RT 커널

```bash
# LowLatency 커널 설치 (실시간 성능 향상)
sudo apt install linux-lowlatency-hwe-22.04
sudo reboot

# 확인
uname -v  # "lowlatency" 또는 "PREEMPT_RT" 확인
```

### 2. ROS2 Humble 설치

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 3. 자동 설치 (권장)

```bash
chmod +x install.sh

# 전체 설치 (기본값): UR 드라이버 + Pinocchio + MuJoCo + RT 권한
./install.sh

# 시뮬레이션 전용: Pinocchio + MuJoCo만 설치 (개발 PC / 로봇 없는 환경)
./install.sh sim

# 실제 로봇 전용: UR 드라이버 + Pinocchio + RT 권한 (MuJoCo 없음)
./install.sh robot

# 도움말
./install.sh --help
```

**각 모드별 설치 내용**:

| 항목 | `sim` | `robot` | `full` |
|---|---|---|---|
| ROS2 빌드 도구 | ✔ | ✔ | ✔ |
| Pinocchio | ✔ | ✔ | ✔ |
| MuJoCo 3.x | ✔ | — | ✔ |
| UR 로봇 드라이버 | — | ✔ | ✔ |
| RT 권한 설정 | — | ✔ | ✔ |

`sim` 모드는 MuJoCo 3.x를 GitHub에서 자동 다운로드하여 `/opt/`에 설치합니다.

### 4. 수동 설치

```bash
# C++ 의존성
sudo apt install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-msgs \
    ros-humble-ur-description \
    ros-humble-control-msgs \
    ros-humble-industrial-msgs \
    ros-humble-ament-cmake \
    python3-colcon-common-extensions

# Pinocchio (v4.3.0+ 필수)
sudo apt install -y ros-humble-pinocchio
# 또는 robotpkg 버전
# sudo apt install -y robotpkg-py310-pinocchio

# Python 의존성
pip3 install --user -r requirements.txt

# RT 권한 설정 (v4.2.0+ 필수)
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필수!
```

### 5. 빌드

```bash
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
git clone https://github.com/hyujun/ur5e-rt-controller.git

cd ~/ur_ws
colcon build --packages-select ur5e_rt_controller --symlink-install
source install/setup.bash

# 환경변수 영구 추가
echo "source ~/ur_ws/install/setup.bash" >> ~/.bashrc
```

---

## 사용 방법

### 전체 시스템 실행

```bash
# 환경 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# UR5e 로봇 IP 확인 후 실행 (Teach Pendant → Settings → Network)
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10
```

런치 파일이 시작하는 노드:
1. `ur_robot_driver` - UR5e 드라이버 (ur_type: ur5e)
2. `custom_controller` - 500Hz PD 제어 노드 + E-STOP + 병렬 컴퓨팅
3. `data_health_monitor` - 데이터 헬스 모니터 (10Hz)

시뮬레이션(fake hardware) 테스트:
```bash
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true
```

### UDP 핸드 노드만 실행

```bash
ros2 launch ur5e_rt_controller hand_udp.launch.py \
    udp_port:=50001 \
    target_ip:=192.168.1.100 \
    target_port:=50002
```

### Qt GUI 모션 에디터

```bash
# PyQt5 설치 (없는 경우)
sudo apt install python3-pyqt5

# GUI 실행
ros2 run ur5e_rt_controller motion_editor_gui.py
```

GUI 사용법:
1. **관절 각도 확인**: 상단 패널에서 J1~J6 실시간 표시
2. **포즈 저장**: 테이블에서 행 선택 → "Save Current Pose" 클릭
3. **포즈 로드**: 저장된 행 선택 → "Load Selected Pose" 클릭 → 로봇 이동
4. **모션 재생**: 여러 행 선택 (Ctrl+클릭) → "Play Motion Sequence" 클릭 (2초 간격)
5. **파일 저장**: File → Save Motion to JSON (50개 포즈 JSON 백업)
6. **파일 로드**: File → Load Motion from JSON

### 데이터 시각화

```bash
# 모든 관절 플롯
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv

# 특정 관절만 (0~5)
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv --joint 2

# 이미지 파일로 저장
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv --save-dir ~/ur_plots

# 통계만 출력
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv --stats
```

### 핸드 UDP 테스트 (예제)

```bash
# 사인파 테스트 데이터 전송 (500Hz)
ros2 run ur5e_rt_controller hand_udp_sender_example.py
# → 1) 사인파 (동적) / 2) 고정 포즈 (정적) 선택
```

---

## 설정 (YAML)

### `config/ur5e_rt_controller.yaml`

```yaml
controller:
  control_rate: 500.0        # 제어 주파수 (Hz)
  kp: 5.0                    # P 게인
  kd: 0.5                    # D 게인
  enable_logging: true        # CSV 로깅 활성화
  log_path: "/tmp/ur5e_control_log.csv"

joint_limits:
  max_velocity: 2.0          # rad/s
  max_acceleration: 5.0      # rad/s^2
  position_limits:
    joint_0: [-6.28, 6.28]   # Base
    joint_1: [-6.28, 6.28]   # Shoulder
    joint_2: [-3.14, 3.14]   # Elbow
    joint_3: [-6.28, 6.28]   # Wrist 1
    joint_4: [-6.28, 6.28]   # Wrist 2
    joint_5: [-6.28, 6.28]   # Wrist 3

estop:
  enable_estop: true
  robot_timeout_ms: 100.0    # 로봇 데이터 100ms 미수신 시 E-STOP
  hand_timeout_ms: 200.0     # 핸드 데이터 200ms 미수신 시 핸드 E-STOP
  safe_position:             # E-STOP 복구 안전 위치 (rad)
    - 0.0   # Base
    - -1.57 # Shoulder
    - 1.57  # Elbow
    - -1.57 # Wrist 1
    - -1.57 # Wrist 2
    - 0.0   # Wrist 3

logging:
  enable_logging: true
  log_frequency: 100.0       # Hz (제어율에서 서브샘플링)
  max_log_size_mb: 100
  log_directory: "/tmp/ur5e_logs"
```

### `config/hand_udp_receiver.yaml`

```yaml
udp:
  port: 50001                # UDP 수신 포트
  buffer_size: 1024          # 바이트
  timeout_ms: 1000           # 소켓 타임아웃

publishing:
  rate: 100.0                # ROS2 퍼블리시 주파수 (Hz)
  topic: "/hand/joint_states"

hand:
  num_joints: 4
  joint_names: [finger_1, finger_2, finger_3, finger_4]
  min_position: 0.0
  max_position: 1.0
```

---

## ROS2 토픽 인터페이스

### 구독 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR 드라이버 | 6-DOF 관절 위치/속도/힘 |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 외부 노드 | 목표 관절 위치 (6개 값, rad) |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | `hand_udp_receiver_node` | 핸드 상태 (4개 값) |
| `/hand/command` | `std_msgs/Float64MultiArray` | 외부 노드 | 핸드 명령 (4개 값) |

### 발행 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | `custom_controller` | UR 위치 명령 (6개 값, rad) |
| `/system/estop_status` | `std_msgs/Bool` | `custom_controller` | E-STOP 상태 (true=활성) |
| `/joint_states` | `sensor_msgs/JointState` | `mujoco_simulator_node` | MuJoCo 시뮬 관절 위치/속도/**토크** |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | `mujoco_simulator_node` | MuJoCo 핸드 상태 (100Hz) |
| `/sim/status` | `std_msgs/Float64MultiArray` | `mujoco_simulator_node` | `[step_count, sim_time_sec, rtf, paused]` |

---

## UDP 핸드 프로토콜

핸드 시스템은 UDP로 77개의 `double` 값(616 bytes)을 전송합니다.

### 패킷 형식 (송신: 핸드 → ROS2)

```
오프셋    크기         필드
0         11 doubles   motor_pos[11]      (모터 위치)
88        11 doubles   motor_vel[11]      (모터 속도)
176       11 doubles   motor_current[11]  (모터 전류)
264       44 doubles   sensor_data[44]    (4 센서 × 11 데이터)
총계: 77 doubles = 616 bytes
```

수신 포트: **50001** (기본값, `hand_udp_receiver.yaml`에서 변경 가능)

### 패킷 형식 (수신: ROS2 → 핸드)

4개 `double` 값 → 모터 위치 명령 (정규화 0.0~1.0)

송신 포트: **50002** (기본값, `hand_udp.launch.py`에서 변경 가능)

### Python 예제 (핸드 시뮬레이터)

```python
import socket, struct, numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("127.0.0.1", 50001)

motor_pos     = [0.5] * 11
motor_vel     = [0.0] * 11
motor_current = [0.5] * 11
sensor_data   = [0.0] * 44

packet = struct.pack('77d', *(motor_pos + motor_vel + motor_current + sensor_data))
sock.sendto(packet, target)
```

---

## 성능 지표

### v4.2.0+ 병렬 컴퓨팅 개선

| 메트릭 | v4.0.0 | v4.2.0+ | 개선율 |
|--------|--------|---------|--------|
| 제어 지터 | ~500μs | <50μs | **10배** |
| E-STOP 반응 시간 | ~100ms | <20ms | **5배** |
| CPU 사용률 | ~30% | ~25% | 17% 감소 |
| Context Switch | ~5000/s | ~1000/s | 80% 감소 |

### 일반 성능

| 항목 | 값 |
|------|-----|
| 제어 주파수 | 500Hz (2ms) |
| E-STOP 감시 주기 | 50Hz (20ms) |
| 핸드 데이터 퍼블리시 | 100Hz |
| GUI 업데이트 | 100Hz (Qt 타이머 10ms) |
| 로봇 E-STOP 타임아웃 | 100ms |
| 핸드 E-STOP 타임아웃 | 200ms |
| CSV 로그 경로 | `/tmp/ur5e_control_log.csv` |
| 통계 저장 경로 | `/tmp/ur5e_stats/` |

**v4.2.0+ RT 최적화**:
- 4개 CallbackGroup 분리 (RT, Sensor, Log, Aux)
- CPU affinity (Core 2-5 전용)
- SCHED_FIFO 실시간 스케줄링
- `mlockall` — `rclcpp::init` 이전에 호출하여 DDS 힙 포함 전체 잠금
- 상세 가이드: [docs/RT_OPTIMIZATION.md](docs/RT_OPTIMIZATION.md)

**v4.2.3 RT 안전성 강화**:
- `ControlLoop()`에서 파일 I/O 완전 제거 → SPSC 링 버퍼 경유
- `RealtimePublisher` 도입으로 RT 경로 힙 할당 제거
- `atomic<bool>` 플래그로 데이터 레이스 3건 해소
- `HandUdpReceiver` jthread에 `kUdpRecvConfig` 자동 적용
- `SelectThreadConfigs()` — 런타임 CPU 수 감지로 4/6코어 자동 선택

---

## 문제 해결

### E-STOP이 계속 활성화됨

```bash
# 로봇 데이터 확인
ros2 topic hz /joint_states          # 500Hz여야 함
ros2 topic echo /system/estop_status  # E-STOP 상태 확인

# 핸드 데이터 확인 (핸드 없는 환경)
# hand_timeout_ms를 0으로 설정하거나 E-STOP 비활성화
# config/ur5e_rt_controller.yaml:
# estop:
#   enable_estop: false
```

### 500Hz 미달성

```bash
# 1) RT 커널 확인
uname -v  # "lowlatency" 또는 "PREEMPT_RT"

# 2) RT 권한 확인 (v4.2.0+ 필수)
ulimit -r  # 99여야 함
groups | grep realtime  # realtime 그룹 포함 확인

# 3) CPU 성능 모드 설정
sudo cpupower frequency-set -g performance

# 4) CPU isolation (선택, 최대 성능)
# docs/RT_OPTIMIZATION.md 참조
```

### RT 권한 부족 경고

```
[WARN] Thread config failed for 'rt_control' (need realtime permissions)
```

**해결**:
```bash
# 1. 권한 설정 확인
cat /etc/security/limits.conf | grep realtime
# @realtime - rtprio 99
# @realtime - memlock unlimited

# 2. 그룹 확인
groups | grep realtime

# 3. 로그아웃 후 재로그인 (필수!)
# 또는
newgrp realtime

# 4. 확인
ulimit -r  # 99 출력되어야 함
```

### UR 드라이버 연결 실패

```bash
# 네트워크 확인
ping 192.168.1.10

# Teach Pendant에서 External Control 프로그램 실행:
# Program → External Control → Run

# 포트 확인 (UR 기본: 50001, 30001~30004)
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=<실제IP>
```

### `forward_position_controller` 활성화 실패

```bash
# 수동 전환
ros2 control switch_controllers \
    --deactivate scaled_joint_trajectory_controller \
    --activate forward_position_controller

# 상태 확인
ros2 control list_controllers
```

### GUI 실행 오류

```bash
# PyQt5 설치
sudo apt install python3-pyqt5
# 또는
pip3 install pyqt5
```

### 목표 위치 전송

```bash
# 목표 위치 수동 퍼블리시 (홈 포즈)
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
    "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

---

## 모니터링

```bash
# 제어 주파수 확인 (목표: 500Hz)
ros2 topic hz /forward_position_controller/commands

# 관절 상태 확인
ros2 topic echo /joint_states

# E-STOP 상태 확인
ros2 topic echo /system/estop_status

# 핸드 데이터 확인
ros2 topic hz /hand/joint_states

# 컨트롤러 목록
ros2 control list_controllers -v

# 데이터 헬스 모니터 단독 실행
ros2 run ur5e_rt_controller monitor_data_health.py

# 스레드 설정 확인 (v4.2.0+)
PID=$(pgrep -f custom_controller)
ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
# 출력:
#   PID   TID CLS RTPRIO PSR COMMAND
#  1234  1235  FF     90   2 rt_control    ← Core 2, FIFO 90
#  1234  1236  FF     70   3 sensor_io     ← Core 3, FIFO 70
#  1234  1237  TS      -   4 logger        ← Core 4, OTHER
#  1234  1238  TS      -   5 aux           ← Core 5, OTHER

# 시스템 지터 측정 (RT 커널)
sudo cyclictest -l100000 -m -n -p99 -t1 -i2000
```

---

## 고급 사용법

### 커스텀 제어기 추가

1. `include/ur5e_rt_controller/controllers/my_controller.hpp` 생성:

```cpp
#pragma once
#include "ur5e_rt_controller/rt_controller_interface.hpp"

namespace ur5e_rt_controller {

class MyController final : public RTControllerInterface {
public:
  [[nodiscard]] ControllerOutput Compute(
      const ControllerState& state) noexcept override {
    ControllerOutput output;
    // 제어 로직 구현 — 반드시 noexcept
    return output;
  }
  void SetRobotTarget(std::span<const double>) noexcept override {}
  void SetHandTarget(std::span<const double>)  noexcept override {}
  [[nodiscard]] std::string_view Name() const  noexcept override { return "MyController"; }
};

}  // namespace ur5e_rt_controller
```

2. `src/custom_controller.cpp`에서 `PDController` 대신 사용:

```cpp
// controller_ 타입을 RTControllerInterface로 변경 후:
controller_(std::make_unique<urtc::MyController>())
```

3. 재빌드:

```bash
colcon build --packages-select ur5e_rt_controller
```

> **Pinocchio 제어기 교체 방법**은 [Pinocchio 기반 제어기](#pinocchio-기반-제어기) 섹션을 참조하세요.

### CSV 로그 형식

```
timestamp, current_pos_0, current_pos_1, ..., current_pos_5,
           target_pos_0, ..., target_pos_5,
           command_0, ..., command_5
```

예시:
```
0.000, 0.000, -1.570, 0.000, 0.000, 0.000, 0.000, 0.000, ...
0.002, 0.001, -1.569, 0.001, 0.001, 0.001, 0.001, 0.001, ...
```

### 모션 JSON 형식

```json
{
  "num_poses": 50,
  "poses": {
    "pose_0": [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
    "pose_1": [-0.5, -1.8, 1.2, 1.5, 0.3, 0.0]
  },
  "names": ["Pose 1", "Pose 2", ...]
}
```

---

## 의존성

### C++ 빌드 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 라이브러리 |
| `std_msgs`, `sensor_msgs` | 표준 메시지 타입 |
| `ur_msgs` | UR 전용 메시지 타입 |
| `controller_manager`, `controller_interface` | ros2_control 프레임워크 |
| `hardware_interface` | 하드웨어 추상화 레이어 |
| `realtime_tools` | 실시간 퍼블리셔/버퍼 |
| `Eigen3` | 선형대수 연산 (헤더 전용) |
| `pinocchio` | 강체 동역학 — FK, 야코비안, 중력/코리올리 계산 (v4.3.0+) |

### Python 의존성 (`requirements.txt`)

| 패키지 | 버전 | 용도 |
|--------|------|------|
| `matplotlib` | >=3.5.3 | 궤적 시각화 |
| `pandas` | >=1.5.3 | CSV 데이터 처리 |
| `numpy` | >=1.24.3 | 수치 연산 |
| `scipy` | >=1.10.1 | 신호 처리 |
| `PyQt5` | (시스템 패키지) | 모션 에디터 GUI |

---

## 참고 자료

### 공식 문서
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ROS2 Control Framework](https://control.ros.org/)
- [ROS2 Executors & Callback Groups](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)
- [PREEMPT_RT 실시간 리눅스](https://wiki.linuxfoundation.org/realtime/start)
- [Eigen3 문서](https://eigen.tuxfamily.org/dox/)

### 프로젝트 문서
- [docs/CHANGELOG.md](docs/CHANGELOG.md) - 버전별 상세 변경 이력
- [docs/RT_OPTIMIZATION.md](docs/RT_OPTIMIZATION.md) - v4.2.0 실시간 최적화 가이드
  - CallbackGroup 아키텍처
  - CPU affinity 설정
  - RT 스케줄링
  - 성능 벤치마크
  - 문제 해결

---

## 라이선스

MIT License - [LICENSE](LICENSE) 파일 참조

---

## 버전 이력

| 버전 | 주요 변경사항 |
|------|---------------|
| **v4.5.0** | 인터랙티브 MuJoCo 뷰어 (마우스/키보드), Physics solver 런타임 제어, 중력/접촉 토글, 물체 힘 인가, F1/F4 오버레이, install.sh sim/robot/full 모드 분리 |
| **v4.4.0** | MuJoCo 3.x 시뮬레이터 통합 (FreeRun/SyncStep), GLFW 뷰어, RTF 측정, ControllerTimingProfiler, /sim/status 토픽 |
| v4.3.0 | Pinocchio 모델 기반 제어기 3종 추가 (PinocchioController, ClikController, OperationalSpaceController) |
| v4.2.3 | RT 안전성 수정 9건 (SPSC 링 버퍼, RealtimePublisher, atomic 플래그, mlockall 순서 등) |
| v4.2.2 | 디렉토리 구조 개선 (docs/ 생성, LICENSE 추가, .gitignore 추가) |
| v4.2.1 | setup.py 제거, CMakeLists.txt 스크립트 정리 |
| v4.2.0 | 병렬 컴퓨팅 최적화 (CallbackGroup, RT 스케줄링, CPU affinity) |
| v4.0.0 | E-STOP 시스템, 핸드/로봇 타임아웃 감시, 표준 ROS2 구조 |
| v3.0.0 | PD 제어기 E-STOP 지원, 안전 위치 설정 |
| v2.0.0 | DataLogger CSV 로깅, 핸드 UDP 통합 |
| v1.0.0 | 초기 릴리스, P/PD 제어기, 기본 ROS2 노드 |

**최종 업데이트**: 2026-03-04
**현재 버전**: v4.5.0
