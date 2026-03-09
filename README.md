# UR5e RT Controller

![CI](https://github.com/hyujun/ur5e-rt-controller/actions/workflows/ros2-advanced-ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/hyujun/ur5e-rt-controller/branch/master/graph/badge.svg)](https://codecov.io/gh/hyujun/ur5e-rt-controller)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

**Ubuntu 22.04 (ROS 2 Humble) / Ubuntu 24.04 (ROS 2 Jazzy) | 실시간 UR5e 제어기 + 커스텀 핸드 통합 (v5.4.0)**

E-STOP 안전 시스템, PD 제어기, **Pinocchio 기반 모델 제어기 3종**, **MuJoCo 3.x 물리 시뮬레이터**, UDP 핸드 인터페이스, CSV 데이터 로깅, Qt GUI 모션 에디터를 포함한 완전한 실시간 제어 솔루션입니다.

> **v5.0.0 (멀티-패키지 분리)**: 단일 패키지에서 **6개 독립 ROS2 패키지**로 리팩터링되었습니다. 각 패키지는 레포지토리 루트 직하에 위치하며 각자의 `README.md`와 `CHANGELOG.md`를 포함합니다.
>
> **v5.1.0 (CPU 코어 할당 최적화)**: 실제 로봇 제어 시 RT 성능을 극대화하는 코어 배치 전략이 적용되었습니다. `udp_recv` Core 3→5 이동, 8코어 지원, UR 드라이버 CPU 고정, NIC IRQ 친화성, CycloneDDS 스레드 제한.
>
> **v5.2.0 (디지털 신호 필터)**: `ur5e_rt_base`에 RT-안전 헤더-전용 필터 라이브러리가 추가되었습니다. **4차 Bessel 저역통과 필터** (최대 선형 군지연, 위상 왜곡 없음)와 **이산-시간 Kalman 필터** (위치+속도 동시 추정, 미분 불필요)를 포함합니다.
>
> **v5.2.1 (빌드 버그픽스)**: MuJoCo binary tarball 설치 시 `lib/cmake/mujoco/` 부재로 cmake 탐지가 실패하던 문제 수정. `install.sh`는 `-Dmujoco_ROOT`를 전달하고, `CMakeLists.txt`는 `find_library` 폴백으로 `.so` 파일을 직접 탐지합니다.
>
> **v5.2.2 (ROS 2 Jazzy 마이그레이션 + 다중 개선)**: `ur5e_description` 패키지 신규 추가 (MJCF/URDF/메시 통합 관리), 로깅 경로 동적 해석 (`~/ros2_ws/ur5e_ws/logging_data`), `build.sh` 빌드 스크립트 추가, `ros-jazzy-rmw-cyclonedds-cpp` 의존성 추가, `mujoco_simulator_node.cpp` → 3개 파일 분리(`mujoco_sim_loop.cpp`, `mujoco_viewer.cpp`), `solver_niter`(`int*`) island별 합산 수정.
>
> **v5.3.0 (멀티 컨트롤러 런타임 전환 + GUI 게인 튜닝)**: 런타임에 P/PD/Pinocchio/CLIK/OSC 컨트롤러를 ROS2 토픽으로 전환 가능. `controller_gui.py` (tkinter) 신규 — 컨트롤러 선택, 게인 슬라이더, 타겟 전송을 단일 GUI로 통합. MuJoCo `package://` URI 네이티브 지원 (ROS2 resource provider 등록).
>
> **v5.4.0 (Controller Registry + 확장 가이드)**: 새 컨트롤러 추가 시 `MakeControllerEntries()` 한 줄 등록으로 완결. `RTControllerInterface`에 `LoadConfig()` / `UpdateGainsFromMsg()` 훅 추가로 컨트롤러별 YAML 파싱·게인 업데이트를 자기 자신이 담당. `switch`/`dynamic_cast` 코드 제거. 신규: `docs/ADDING_CONTROLLER.md` 단계별 가이드.

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
| 런타임 컨트롤러 전환 | ROS2 토픽으로 P/PD/Pinocchio/CLIK/OSC 간 즉시 전환 (v5.3.0+) |
| GUI 게인 튜닝 | tkinter 기반 `controller_gui.py` — 컨트롤러 선택·게인 설정·타겟 전송 (v5.3.0+) |
| MuJoCo `package://` URI | ROS2 resource provider 등록으로 MJCF 내 패키지 URI 네이티브 지원 (v5.3.0+) |
| 커스텀 핸드 통합 | UDP 기반 11-DOF 핸드 데이터 수신/송신 |
| 데이터 로깅 | CSV 형식의 제어 데이터 실시간 기록 (`DataLogger` + SPSC 링 버퍼) |
| 데이터 시각화 | Matplotlib 기반 관절 궤적 플롯 (`plot_ur_trajectory.py`) |
| 데이터 헬스 모니터 | 패킷 손실/타임아웃 통계 수집 및 JSON 내보내기 |
| Qt GUI 에디터 | 50개 포즈 저장/로드/재생 모션 에디터 |
| Strategy Pattern | `RTControllerInterface`를 상속하는 교체 가능한 제어기 구조 |
| Controller Registry | `MakeControllerEntries()` 한 줄 등록으로 새 컨트롤러 추가 완결 (v5.4.0+) |
| 설치 모드 선택 | `install.sh sim / robot / full` — 환경에 맞게 선택 설치 (v4.5.0+) |
| 신호 필터 | Bessel LPF (선형 위상) + Kalman 필터 (속도 추정 내장), N채널 RT-안전 (v5.2.0+) |

---

## 프로젝트 구조

v5.0.0부터 **6개 독립 ROS2 패키지**로 분리되어 레포지토리 루트 직하에 위치합니다.

```
ur5e-rt-controller/
├── README.md                              # 이 문서
├── install.sh                             # 자동 설치 스크립트 (IRQ affinity 포함)
├── build.sh                               # 빌드 스크립트 (sim/robot/full 모드 선택)
├── requirements.txt                       # Python 의존성 목록
│
├── docs/
│   ├── CHANGELOG.md                      # 전체 버전 변경 이력
│   ├── RT_OPTIMIZATION.md                # 실시간 최적화 가이드
│   ├── ADDING_CONTROLLER.md              # 새 컨트롤러 추가 단계별 가이드 (v5.4.0+)
│   └── CLAUDE.md                         # AI 어시스턴트 컨텍스트 문서
│
├── ur5e_description/                      # 📦 로봇 모델 description (신규 v5.2.2)
│   └── robots/ur5e/
│       ├── mjcf/                         # MuJoCo MJCF 모델 (scene.xml, ur5e.xml)
│       ├── urdf/                         # Pinocchio용 URDF (xacro 자동 생성)
│       └── meshes/                       # UR 공식 메시 (visual DAE + collision STL)
│
├── ur5e_rt_base/                          # 📦 공유 기반 (헤더-전용)
│   ├── include/ur5e_rt_base/
│   │   ├── types.hpp                     # 공유 타입: RobotState, HandState, ControllerState...
│   │   ├── thread_config.hpp             # ThreadConfig + 4/6/8코어 사전 정의 RT 상수
│   │   ├── thread_utils.hpp              # ApplyThreadConfig(), SelectThreadConfigs()
│   │   ├── log_buffer.hpp                # SPSC 링 버퍼 (RT→로그, 잠금-없음)
│   │   ├── data_logger.hpp               # 비-RT CSV 로거 (동적 경로 해석)
│   │   └── filters/
│   │       ├── bessel_filter.hpp         # 4차 Bessel LPF — N채널, noexcept, 선형 위상
│   │       └── kalman_filter.hpp         # 이산-시간 Kalman 필터 — 위치+속도 추정, noexcept
│   └── ...
│
├── ur5e_rt_controller/                    # 📦 500Hz 실시간 제어기
│   ├── include/ur5e_rt_controller/
│   │   ├── rt_controller_interface.hpp   # 추상 기반 클래스 (Strategy Pattern)
│   │   ├── controller_timing_profiler.hpp
│   │   ├── controllers/
│   │   │   ├── pd_controller.hpp         # PD + E-STOP (기본값)
│   │   │   ├── p_controller.hpp          # 단순 P 제어기 (테스트용)
│   │   │   ├── pinocchio_controller.hpp  # PD + 중력/코리올리 보상
│   │   │   ├── clik_controller.hpp       # 폐루프 IK (3-DOF)
│   │   │   └── operational_space_controller.hpp
│   │   └── trajectory/                   # 궤적 생성 서브시스템 (v5.3.0+)
│   │       ├── trajectory_utils.hpp      # QuinticPolynomial 스칼라 유틸
│   │       ├── task_space_trajectory.hpp # SE(3) 5차 스플라인 (CLIK/OSC 사용)
│   │       └── joint_space_trajectory.hpp# 관절공간 5차 스플라인
│   ├── include/ur5e_rt_controller/
│   │   └── rt_controller_node.hpp        # RtControllerNode 클래스 선언 (v5.5.0)
│   ├── src/
│   │   ├── rt_controller_node.cpp        # Controller Registry + 노드 구현 (v5.5.0)
│   │   ├── rt_controller_main.cpp        # main() — executor/RT 스레드 (v5.5.0)
│   │   └── controllers/                  # 각 컨트롤러 구현 (.cpp)
│   │       ├── p_controller.cpp
│   │       ├── pd_controller.cpp
│   │       ├── pinocchio_controller.cpp
│   │       ├── clik_controller.cpp
│   │       └── operational_space_controller.cpp
│   ├── config/
│   │   ├── ur5e_rt_controller.yaml       # 로깅 경로: ~/ros2_ws/ur5e_ws/logging_data
│   │   ├── cyclone_dds.xml               # CycloneDDS 스레드 Core 0-1 제한 설정
│   │   └── controllers/                  # 컨트롤러별 게인 YAML (v5.4.0+)
│   │       ├── p_controller.yaml
│   │       ├── pd_controller.yaml
│   │       ├── pinocchio_controller.yaml
│   │       ├── clik_controller.yaml
│   │       └── operational_space_controller.yaml
│   ├── scripts/
│   │   └── setup_irq_affinity.sh         # NIC IRQ → Core 0-1 고정 스크립트
│   └── launch/ur_control.launch.py       # 전체 시스템 (use_cpu_affinity 인자 포함)
│
├── ur5e_hand_udp/                         # 📦 UDP 손 브리지
├── ur5e_mujoco_sim/                       # 📦 MuJoCo 3.x 시뮬레이터 (선택적)
│   ├── include/ur5e_mujoco_sim/
│   │   └── ros2_resource_provider.hpp    # package:// URI provider 등록 (v5.3.0+)
│   ├── src/
│   │   ├── mujoco_simulator.cpp          # 생명주기 및 I/O
│   │   ├── mujoco_sim_loop.cpp           # 물리 루프 (FreeRun/SyncStep)
│   │   ├── mujoco_viewer.cpp             # GLFW 뷰어 루프 (~60Hz)
│   │   ├── mujoco_simulator_node.cpp     # ROS2 노드 래퍼
│   │   └── ros2_resource_provider.cpp    # package:// URI 해석 구현 (v5.3.0+)
│   └── ...
└── ur5e_tools/                            # 📦 Python 개발 유틸리티
    └── ur5e_tools/
        └── controller_gui.py              # 컨트롤러 GUI (v5.3.0+)
```

### 패키지 의존성 그래프

```
ur5e_description   ← 독립 (모델 파일 제공)

ur5e_rt_base       ← 독립 (공유 기반, 헤더-전용)
    ↑
    ├── ur5e_rt_controller  ← ur5e_rt_base, ur5e_description
    └── ur5e_hand_udp       ← ur5e_rt_base

ur5e_mujoco_sim    ← ur5e_description (런타임 모델 참조)
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

#### `RtControllerNode` (`src/rt_controller_node.hpp/cpp`)
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
  struct RobotState {
    std::array<double, 6>  positions{}, velocities{};
    std::array<double, 3>  tcp_position{};
    double dt{0.002}; uint64_t iteration{0};
  };
  struct HandState {
    std::array<double, 11> motor_positions{}, motor_velocities{}, motor_currents{};
    std::array<double, 44> sensor_data{};
    bool valid{false};
  };
  struct ControllerOutput {
    std::array<double, 6>  robot_commands{};
    std::array<double, 11> hand_commands{};
    bool valid{true};
  };

  class RTControllerInterface {
    [[nodiscard]] virtual ControllerOutput Compute(
        const ControllerState& state) noexcept = 0;
  };
}
```

#### `PDController` (`include/ur5e_rt_controller/controllers/pd_controller.hpp`)
비례-미분 제어기. E-STOP 발생 시 안전 위치 `[0, -1.57, 1.57, -1.57, -1.57, 0]`로 이동.

#### `DataLogger` (`ur5e_rt_base/include/ur5e_rt_base/data_logger.hpp`)
이동 전용(복사 불가) 비-RT CSV 로거. 타임스탬프, 현재/목표 위치, 명령값, `compute_time_us` 기록.
`DrainAndWrite(SpscLogBuffer<LogEntry, 512>&)` 메서드로 SPSC 링 버퍼를 소진하여 파일에 씀 — 파일 I/O는 log 스레드(Core 4)에서만 발생.

#### `SpscLogBuffer` (`ur5e_rt_base/include/ur5e_rt_base/log_buffer.hpp`)
`SpscLogBuffer<LogEntry, 512>` lock-free 단일 생산자/단일 소비자 링 버퍼.
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
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py

# Sync-step 모드
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py sim_mode:=sync_step

# 헤드리스 (뷰어 없음, 서버 환경)
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py enable_viewer:=false

# 외부 MJCF 모델 사용 (MuJoCo Menagerie 등)
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py \
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

// Solver 통계 확인 (iter = 모든 constraint island 반복 횟수 합산)
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

### 사용 방법 (rt_controller_node.cpp 등록)

```cpp
// 1. 헤더 교체 (기존 pd_controller.hpp 대신)
#include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"
// 또는
#include "ur5e_rt_controller/controllers/clik_controller.hpp"
// 또는
#include "ur5e_rt_controller/controllers/operational_space_controller.hpp"

// 2. controller_ 멤버 타입 변경 (≈line 340)
std::unique_ptr<urtc::RTControllerInterface> controller_;

// 3. 생성자 초기화 (ur5e_description 패키지 사용 — v5.2.2+)
// PinocchioController:
controller_(std::make_unique<urtc::PinocchioController>(
    "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
    urtc::PinocchioController::Gains{.kp = 5.0, .kd = 0.5,
                                      .enable_gravity_compensation = true}))

// ClikController:
controller_(std::make_unique<urtc::ClikController>(
    "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
    urtc::ClikController::Gains{.kp = 1.0, .damping = 0.01, .null_kp = 0.5}))

// OperationalSpaceController:
controller_(std::make_unique<urtc::OperationalSpaceController>(
    "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
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

---

## 궤적 생성 (`ur5e_rt_controller/trajectory/`)

v5.3.0에서 추가된 헤더-전용 궤적 생성 서브시스템입니다. CLIK 및 OSC 제어기에서 목표점으로의 부드러운 이동을 위해 사용됩니다. 모든 클래스는 `namespace ur5e_rt_controller::trajectory`에 위치합니다.

### `QuinticPolynomial` (`trajectory/trajectory_utils.hpp`)

스칼라 5차(quintic) 다항식 유틸리티. 위치/속도/가속도 경계 조건을 모두 충족하는 6개 계수를 계산합니다.

```cpp
QuinticPolynomial poly;
poly.compute_coefficients(
    p0, v0, a0,   // 시작: 위치, 속도, 가속도
    pf, vf, af,   // 목표: 위치, 속도, 가속도
    T);           // 지속 시간 (초)

auto state = poly.compute(t);  // TrajectoryState{pos, vel, acc}
```

### `TaskSpaceTrajectory` (`trajectory/task_space_trajectory.hpp`)

SE(3) 공간(위치 + 자세)에서의 5차 스플라인 궤적. Pinocchio의 `log6`/`exp6`를 사용해 시작/목표 SE(3) 포즈 사이를 부드럽게 보간합니다. **CLIK 및 OSC 제어기에서** 새 목표 수신 시 자동으로 생성됩니다.

```cpp
TaskSpaceTrajectory traj;
traj.initialize(
    start_pose, start_vel,  // pinocchio::SE3, pinocchio::Motion
    goal_pose,  goal_vel,
    duration);              // 지속 시간 (초, 자동 계산: 거리/최대속도)

// 500Hz RT 루프 내:
auto state = traj.compute(trajectory_time_);  // State{pose, velocity, acceleration}
```

| 요소 | 설명 |
|------|------|
| 보간 공간 | SE(3) 접선 공간 (Pinocchio `log6`) |
| 차수 | 5차 (속도·가속도 경계 조건 만족) |
| 독립 축 | 6개 (`QuinticPolynomial` × 6) |
| 힙 할당 | 없음 (고정 크기 배열 사용) |

### `JointSpaceTrajectory<N>` (`trajectory/joint_space_trajectory.hpp`)

N-DOF 관절공간 5차 스플라인 궤적 (템플릿). 위치/속도/가속도 경계 조건으로 N개 관절을 독립적으로 보간합니다.

```cpp
JointSpaceTrajectory<6> traj;
JointSpaceTrajectory<6>::State start{.positions = q0, .velocities = v0, .accelerations = {}};
JointSpaceTrajectory<6>::State goal {.positions = qf, .velocities = {}, .accelerations = {}};
traj.initialize(start, goal, duration);

auto state = traj.compute(t);  // State{positions, velocities, accelerations}
```

---

## 신호 필터 (`ur5e_rt_base/filters/`)

v5.2.0에서 추가된 RT-안전 헤더-전용 필터 라이브러리입니다. `Init()` 이후 모든 처리 메서드가 **`noexcept`** — 500Hz RT 루프에서 추가 의존성 없이 직접 사용 가능합니다.

### `BesselFilterN<N>` — 4차 Bessel 저역통과 필터

`#include "ur5e_rt_base/filters/bessel_filter.hpp"`

**선택 이유**: 최대 선형 군지연(Maximally Flat Group Delay) → 모든 주파수 성분이 동일 지연 → 위상 왜곡 없는 관절 궤적 평활화.

```cpp
BesselFilter6 lpf;
lpf.Init(100.0, 500.0);                  // 100 Hz 컷오프, 500 Hz 샘플레이트
// 500 Hz RT 루프:
auto smoothed = lpf.Apply(raw_positions); // noexcept
```

| 메서드 | RT 안전 | 설명 |
|--------|---------|------|
| `Init(cutoff_hz, sample_rate_hz)` | 아니오 | 계수 계산, 상태 초기화 |
| `Apply(array<double,N>)` | **예** | N채널 필터링 |
| `ApplyScalar(double, ch)` | **예** | 단일 채널 버전 |
| `Reset()` | **예** | 지연 소자 초기화 |

**별칭**: `BesselFilter6`, `BesselFilter11`, `BesselFilter1`

---

### `KalmanFilterN<N>` — 이산-시간 Kalman 필터

`#include "ur5e_rt_base/filters/kalman_filter.hpp"`

**선택 이유**: 위치 측정 하나로 **위치와 속도를 동시에 최적 추정** — 미분 없는 속도 추정으로 PD 제어기 D항에 직접 활용 가능.

**모델**: 상수-속도 운동 모델, 상태 `x = [pos, vel]ᵀ`, 관측 `H = [1, 0]`

```cpp
KalmanFilter6 kf;
kf.Init(0.001, 0.01, 0.1, 0.002);       // q_pos, q_vel, r, dt
kf.SetInitialPositions(init_pos);        // 시작 과도 현상 방지
// 500 Hz RT 루프:
auto filtered = kf.PredictAndUpdate(raw_positions); // noexcept
double vel_j0 = kf.velocity(0);          // 미분 없는 속도 추정
```

| 메서드 | RT 안전 | 설명 |
|--------|---------|------|
| `Init(q_pos, q_vel, r, dt)` | 아니오 | 파라미터 설정, 상태 초기화 |
| `Predict()` | **예** | 상태 예측 (매 틱 호출) |
| `Update(array<double,N>)` | **예** | 측정 융합, 필터 위치 반환 |
| `PredictAndUpdate(array)` | **예** | Predict + Update 단일 호출 |
| `velocity(i)` | **예** | 채널 i 속도 추정값 |
| `SetInitialPositions(array)` | **예** | 초기 상태 시드 |

**별칭**: `KalmanFilter6`, `KalmanFilter11`, `KalmanFilter1`

---

### Bessel vs Kalman 선택 기준

| 항목 | Bessel | Kalman |
|------|--------|--------|
| 주 목적 | 위상 왜곡 없는 노이즈 제거 | 위치+속도 동시 추정 |
| 파라미터 직관성 | 높음 (컷오프 Hz) | 중간 (Q/R 비율) |
| 속도 추정 | 별도 미분 필요 | 내장 `velocity()` |
| 급격한 위치 변화 추종 | 컷오프에 따라 지연 | `q_vel` 조정으로 대응 |

> 단순 궤적 평활화 → **Bessel**, 속도 추정이 필요한 PD 제어 → **Kalman**

---

## 설치 방법

### 1. 사전 요구사항

- Ubuntu 22.04 LTS (ROS 2 Humble) 또는 Ubuntu 24.04 LTS (ROS 2 Jazzy)
- (권장) LowLatency 또는 PREEMPT_RT 커널

```bash
# LowLatency 커널 설치 (실시간 성능 향상)
sudo apt install linux-lowlatency-hwe-22.04
sudo reboot

# 확인
uname -v  # "lowlatency" 또는 "PREEMPT_RT" 확인
```

### 2. ROS2 설치

**Ubuntu 22.04 (ROS 2 Humble):**
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

**Ubuntu 24.04 (ROS 2 Jazzy):**
```bash
# 위와 동일 절차로 설치 (CODENAME=noble)
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

### 3. 자동 설치 (권장)

```bash
chmod +x install.sh

# 전체 설치 (기본값): UR 드라이버 + Pinocchio + MuJoCo + RT 권한 + IRQ affinity
./install.sh

# 시뮬레이션 전용: Pinocchio + MuJoCo만 설치 (개발 PC / 로봇 없는 환경)
./install.sh sim

# 실제 로봇 전용: UR 드라이버 + Pinocchio + RT 권한 + IRQ affinity (MuJoCo 없음)
./install.sh robot

# (고급) 특정 단계 스킵 기능
./install.sh sim --skip-deps     # apt 의존성 설치 스킵
./install.sh robot --skip-rt     # RT 권한 설정 및 IRQ affinity 셋업 스킵 (도커/컨테이너 권장)
./install.sh full --skip-build   # 다운로드 및 초기 셋업만 진행 (colcon build 스킵)

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
| rmw_cyclonedds_cpp | ✔ | ✔ | ✔ |
| RT 권한 설정 | — | ✔ | ✔ |
| NIC IRQ affinity | — | ✔ | ✔ |

`sim` 모드는 MuJoCo 3.x를 GitHub에서 자동 다운로드하여 `/opt/`에 설치합니다.

### 4. 빌드 스크립트 (권장)

`install.sh` 이후 `build.sh`로 패키지를 빌드합니다:

```bash
chmod +x build.sh

# 기본 로봇/시뮬레이션/전체 모드 빌드
./build.sh sim    # 시뮬레이션 관련 패키지만 빌드
./build.sh robot  # 실제 로봇용 빌드 (ur5e_mujoco_sim 제외)
./build.sh full   # 전체 패키지 빌드

# 고급 빌드 옵션 제공
./build.sh sim -c             # 이전 빌드 파일(build/, install/, log/) 삭제 후 클린 빌드
./build.sh full -d            # CMAKE_BUILD_TYPE=Debug 빌드 (기본값 Release)
./build.sh -p ur5e_rt_base    # 특정 패키지만 선택해서 빌드
./build.sh sim -j 4           # colcon 병렬 워커 개수 제한 (OOM 방지)
./build.sh full --no-bashrc   # 빌드 후 ~/.bashrc 에 `source` 추가 생략
```

### 5. 수동 설치

```bash
# C++ 의존성 (ROS 2 Humble)
sudo apt install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-msgs \
    ros-humble-ur-description \
    ros-humble-control-msgs \
    ros-humble-industrial-msgs \
    ros-humble-ament-cmake \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# C++ 의존성 (ROS 2 Jazzy)
sudo apt install -y \
    ros-jazzy-ur-robot-driver \
    ros-jazzy-ur-msgs \
    ros-jazzy-ur-description \
    ros-jazzy-control-msgs \
    ros-jazzy-industrial-msgs \
    ros-jazzy-ament-cmake \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# Pinocchio (v4.3.0+ 필수)
sudo apt install -y ros-${ROS_DISTRO}-pinocchio

# Python 의존성
pip3 install --user -r requirements.txt

# RT 권한 설정 (v4.2.0+ 필수)
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필수! (또는 재부팅)
```

### 6. 빌드

```bash
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
git clone https://github.com/hyujun/ur5e-rt-controller.git

# 패키지를 워크스페이스 src/에 심링크 (루트 직하에 있음)
for pkg in ur5e_description ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools; do
  ln -s ur5e-rt-controller/$pkg $pkg
done

cd ~/ur_ws
colcon build --packages-select ur5e_description ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools --symlink-install
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
1. `ur_robot_driver` - UR5e 드라이버 (Core 0-1 taskset, 3초 후 자동 적용)
2. `custom_controller` - 500Hz PD 제어 노드 + E-STOP + 병렬 컴퓨팅
3. `data_health_monitor` - 데이터 헬스 모니터 (10Hz)

런치 파일이 자동 설정하는 환경변수:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` → `config/cyclone_dds.xml` (DDS 스레드 Core 0-1 제한)

| 런치 인자 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | `192.168.1.10` | UR 로봇 IP |
| `use_fake_hardware` | `false` | 가상 하드웨어 모드 |
| `use_cpu_affinity` | `true` | UR 드라이버 Core 0-1 taskset 자동 적용 |

시뮬레이션(fake hardware) 테스트:
```bash
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true use_cpu_affinity:=false
```

### UDP 핸드 노드만 실행

```bash
ros2 launch ur5e_hand_udp hand_udp.launch.py \
    udp_port:=50001 \
    target_ip:=192.168.1.100 \
    target_port:=50002
```

### Qt GUI 모션 에디터

```bash
# PyQt5 설치 (없는 경우)
sudo apt install python3-pyqt5

# GUI 실행
ros2 run ur5e_tools motion_editor_gui.py
```

GUI 사용법:
1. **관절 각도 확인**: 상단 패널에서 J1~J6 실시간 표시
2. **포즈 저장**: 테이블에서 행 선택 → "Save Current Pose" 클릭
3. **포즈 로드**: 저장된 행 선택 → "Load Selected Pose" 클릭 → 로봇 이동
4. **모션 재생**: 여러 행 선택 (Ctrl+클릭) → "Play Motion Sequence" 클릭 (2초 간격)
5. **파일 저장**: File → Save Motion to JSON (50개 포즈 JSON 백업)
6. **파일 로드**: File → Load Motion from JSON

### 컨트롤러 GUI (v5.3.0+)

```bash
# 컨트롤러 GUI 실행 (tkinter, 별도 설치 불필요)
ros2 run ur5e_tools controller_gui.py
```

GUI 기능:
1. **컨트롤러 선택**: P / PD / Pinocchio / CLIK / OSC 중 라디오 버튼으로 선택
2. **컨트롤러 전환**: "Switch Controller" 클릭 → `/custom_controller/controller_type` 토픽 발행
3. **게인 설정**: 컨트롤러별 파라미터 입력 필드(게인값) + 체크박스(bool 플래그)
4. **게인 적용**: "Apply Gains" 클릭 → `/custom_controller/controller_gains` 토픽 발행
5. **타겟 설정**: 관절 각도(관절 공간) 또는 TCP 위치(Cartesian 공간) 직접 입력
6. **현재 위치 표시**: `/joint_states` 구독 → 5Hz 주기로 실시간 갱신
7. **Copy Current → Target**: 현재 관절 위치를 타겟 필드에 복사
8. **Send Command**: `/target_joint_positions` 토픽으로 타겟 발행

### 데이터 시각화

```bash
# 모든 관절 플롯
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv

# 특정 관절만 (0~5)
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --joint 2

# 이미지 파일로 저장
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --save-dir ~/ur_plots

# 통계만 출력
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --stats
```

### 핸드 UDP 테스트 (예제)

```bash
# 사인파 테스트 데이터 전송 (500Hz)
ros2 run ur5e_tools hand_udp_sender_example.py
# → 1) 사인파 (동적) / 2) 고정 포즈 (정적) 선택
```

---

## 설정 (YAML)

### `config/ur5e_rt_controller.yaml`

```yaml
# ur5e_rt_controller.yaml (v5.2.2)
/**:
  ros__parameters:
    # Controller Parameters
    control_rate: 500.0          # Hz
    kp: 5.0                      # Proportional gain
    kd: 0.5                      # Derivative gain
    enable_logging: true         # Enable CSV logging
    log_dir: "~/ros2_ws/ur5e_ws/logging_data"  # launch 파일이 절대경로로 확장
    max_log_files: 10            # 최근 N개 로그 파일만 보관

    # Joint Limits
    joint_limits:
      max_velocity: 2.0            # rad/s
      max_acceleration: 5.0        # rad/s^2
      position_limits:
        joint_0: [-6.28, 6.28]     # Base
        joint_1: [-6.28, 6.28]     # Shoulder
        joint_2: [-3.14, 3.14]     # Elbow
        joint_3: [-6.28, 6.28]     # Wrist 1
        joint_4: [-6.28, 6.28]     # Wrist 2
        joint_5: [-6.28, 6.28]     # Wrist 3

    # E-STOP Configuration
    estop:
      enable_estop: true           # E-STOP 모니터링 활성화
      robot_timeout_ms: 100.0      # 로봇 데이터 100ms 미수신 시 E-STOP
      hand_timeout_ms: 200.0       # 핸드 데이터 200ms 미수신 시 E-STOP
      safe_position:               # E-STOP 복구 안전 위치 (rad)
        - 0.0                      # Base
        - -1.57                    # Shoulder
        - 1.57                     # Elbow
        - -1.57                    # Wrist 1
        - -1.57                    # Wrist 2
        - 0.0                      # Wrist 3

    # Logging Parameters
    logging:
      enable_logging: true
      log_frequency: 100.0         # Hz (제어율에서 서브샘플링)
      max_log_size_mb: 100
      log_directory: "~/ros2_ws/ur5e_ws/logging_data"
```

### `ur5e_hand_udp/config/hand_udp_receiver.yaml`

```yaml
udp:
  port: 50001                # UDP 수신 포트
  buffer_size: 1024          # 바이트
  timeout_ms: 1000           # 소켓 타임아웃

publishing:
  rate: 100.0                # ROS2 퍼블리시 주파수 (Hz)
  topic: "/hand/joint_states"

monitoring:
  enable_statistics: true
  statistics_period: 5.0     # 통계 출력 주기 (초)
```

---

## ROS2 토픽 인터페이스

### 구독 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR 드라이버 | 6-DOF 관절 위치/속도/힘 |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 외부 노드 / GUI | 목표 관절 위치 (6개 값, rad) |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | `hand_udp_receiver_node` | 핸드 모터 위치 (**11개** 값) |
| `/hand/command` | `std_msgs/Float64MultiArray` | 외부 노드 | 핸드 명령 (11개 값, 정규화 0.0–1.0) |
| `/custom_controller/controller_type` | `std_msgs/Int32` | GUI | 컨트롤러 전환 (0=P, 1=PD, 2=Pinocchio, 3=CLIK, 4=OSC) |
| `/custom_controller/controller_gains` | `std_msgs/Float64MultiArray` | GUI | 컨트롤러별 게인 동적 업데이트 |

### 발행 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | `custom_controller` | UR 위치 명령 (6개 값, rad) |
| `/system/estop_status` | `std_msgs/Bool` | `custom_controller` | E-STOP 상태 (true=활성) |
| `/joint_states` | `sensor_msgs/JointState` | `mujoco_simulator_node` | MuJoCo 시뮬 관절 위치/속도 |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | `mujoco_simulator_node` | MuJoCo 핸드 상태 — **11개** 모터 위치 (100Hz) |
| `/sim/status` | `std_msgs/Float64MultiArray` | `mujoco_simulator_node` | `[step_count, sim_time_sec, rtf]` (1Hz) |

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
| CSV 로그 경로 | `~/ros2_ws/ur5e_ws/logging_data/` (동적 해석) |
| 통계 저장 경로 | `/tmp/ur5e_stats/` |

**v4.2.0+ RT 최적화**:
- 4개 CallbackGroup 분리 (RT, Sensor, Log, Aux)
- CPU affinity (Core 2-5 전용)
- SCHED_FIFO 실시간 스케줄링
- `mlockall` — `rclcpp::init` 이전에 호출하여 DDS 힙 포함 전체 잠금
- 상세 가이드: [docs/RT_OPTIMIZATION.md](docs/RT_OPTIMIZATION.md)

**v5.1.0 CPU 코어 할당 최적화 (실제 로봇)**:
- `kUdpRecvConfig`: Core 3 → Core 5 (`sensor_io` 전용 Core 3 확보)
- 8코어 전용 config 추가 (`udp_recv` Core 4 독립, `aux` Core 6)
- `SelectThreadConfigs()`: 8코어/6코어/4코어 자동 분기
- `ur_control.launch.py`: UR 드라이버 Core 0-1 taskset 자동 적용
- `config/cyclone_dds.xml`: DDS 스레드 Core 0-1 제한
- `scripts/setup_irq_affinity.sh`: NIC IRQ → Core 0-1 고정
- `install.sh`: `robot`/`full` 모드에서 IRQ affinity 자동 설정

**v4.2.3 RT 안전성 강화**:
- `ControlLoop()`에서 파일 I/O 완전 제거 → SPSC 링 버퍼 경유
- `RealtimePublisher` 도입으로 RT 경로 힙 할당 제거
- `atomic<bool>` 플래그로 데이터 레이스 3건 해소
- `HandUdpReceiver` jthread에 `kUdpRecvConfig` 자동 적용
- `SelectThreadConfigs()` — 런타임 CPU 수 감지로 4/6/8코어 자동 선택

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
ros2 run ur5e_tools monitor_data_health.py

# 스레드 설정 확인 (v4.2.0+)
PID=$(pgrep -f custom_controller)
ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
# 출력 (6코어 기준):
#   PID   TID CLS RTPRIO PSR COMMAND
#  1234  1235  FF     90   2 rt_control    ← Core 2, FIFO 90
#  1234  1236  FF     70   3 sensor_io     ← Core 3, FIFO 70 (전용)
#  1234  1237  TS      -   4 logger        ← Core 4, OTHER
#  1234  1238  TS      -   5 aux           ← Core 5, OTHER
#  ????  ????  FF     65   5 udp_recv      ← Core 5, FIFO 65 (sensor_io와 분리)

# UR 드라이버 CPU 확인 (use_cpu_affinity:=true 시)
taskset -p $(pgrep -nf ur_ros2_driver)   # affinity mask: 3 = Core 0,1

# 시스템 지터 측정 (RT 커널)
sudo cyclictest -l100000 -m -n -p99 -t1 -i2000
```

---

## 고급 사용법

### 커스텀 제어기 추가

v5.4.0부터 **Controller Registry** 패턴을 사용합니다. 전체 단계: **[docs/ADDING_CONTROLLER.md](docs/ADDING_CONTROLLER.md)**

**요약 (4단계)**:

```bash
# 1. 헤더 생성
touch include/ur5e_rt_controller/controllers/my_controller.hpp
# → RTControllerInterface 상속, LoadConfig() + UpdateGainsFromMsg() 구현

# 2. 구현 생성
touch src/controllers/my_controller.cpp

# 3. YAML 생성
echo "my_controller:\n  kp: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]" \
  > config/controllers/my_controller.yaml

# 4. rt_controller_node.cpp에 한 줄 추가
# (MakeControllerEntries() 리스트에 아래 추가)
# {"my_controller", [](const std::string &) { return std::make_unique<urtc::MyController>(); }},
```

```bash
colcon build --packages-select ur5e_rt_controller
```

> **Pinocchio 제어기 추가 방법**도 동일합니다. 팩토리 람다에서 `urdf_path` 인자를 사용하면 됩니다.

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
| **v5.5.0** | `build.sh` 및 `install.sh` 파라미터 파싱 및 고급 제어 기능 구현 (`-c`, `-d`, `-r`, `-j`, `-p`, `--no-bashrc`, `--skip-*`), 초저지연 버퍼 `log_buffer.hpp` 최적화, 클래스명 변경(`CustomController`→`RtControllerNode`) 및 노드 파일 3분할 도입 |
| **v5.4.0** | Controller Registry 패턴 (`MakeControllerEntries()`), `RTControllerInterface`에 `LoadConfig()` / `UpdateGainsFromMsg()` 훅 추가, 컨트롤러별 YAML 로딩·게인 업데이트 자기 책임화, `switch`/`dynamic_cast` 제거, `docs/ADDING_CONTROLLER.md` 신규 |
| **v5.3.0** | 런타임 컨트롤러 전환 (P/PD/Pinocchio/CLIK/OSC `controller_type` 토픽), `controller_gains` 토픽으로 동적 게인 업데이트, `controller_gui.py` tkinter GUI 신규, MuJoCo `package://` URI 네이티브 지원 (`Ros2ResourceProvider`) |
| **v5.2.2** | `ur5e_description` 패키지 신규 추가 (MJCF/URDF/메시 통합), 로깅 경로 동적 해석, `build.sh` 추가, `rmw_cyclonedds_cpp` 의존성, 소스 파일 분리(`mujoco_sim_loop.cpp`, `mujoco_viewer.cpp`), `solver_niter` island 합산 수정, ROS 2 Jazzy 지원 |
| **v5.2.1** | MuJoCo binary tarball cmake 탐지 수정: `install.sh` `-Dmujoco_DIR`→`-Dmujoco_ROOT`, `CMakeLists.txt` `find_library` 폴백 추가 |
| **v5.2.0** | `ur5e_rt_base/filters/` 추가: 4차 Bessel LPF (`BesselFilterN<N>`) + 이산-시간 Kalman 필터 (`KalmanFilterN<N>`) — 모두 noexcept, RT 안전 |
| **v5.1.0** | CPU 코어 할당 최적화: udp_recv Core 3→5, 8코어 지원, UR 드라이버 taskset, CycloneDDS 스레드 제한, NIC IRQ affinity, install.sh 자동화 |
| **v5.0.0** | 6개 독립 ROS2 패키지로 분리 (ur5e_description, ur5e_rt_base, ur5e_rt_controller, ur5e_hand_udp, ur5e_mujoco_sim, ur5e_tools) |
| v4.5.0 | 인터랙티브 MuJoCo 뷰어 (마우스/키보드), Physics solver 런타임 제어, 중력/접촉 토글, 물체 힘 인가, F1/F4 오버레이, install.sh sim/robot/full 모드 분리 |
| v4.4.0 | MuJoCo 3.x 시뮬레이터 통합 (FreeRun/SyncStep), GLFW 뷰어, RTF 측정, ControllerTimingProfiler, /sim/status 토픽 |
| v4.3.0 | Pinocchio 모델 기반 제어기 3종 추가 (PinocchioController, ClikController, OperationalSpaceController) |
| v4.2.3 | RT 안전성 수정 9건 (SPSC 링 버퍼, RealtimePublisher, atomic 플래그, mlockall 순서 등) |
| v4.2.0 | 병렬 컴퓨팅 최적화 (CallbackGroup, RT 스케줄링, CPU affinity) |
| v4.0.0 | E-STOP 시스템, 핸드/로봇 타임아웃 감시, 표준 ROS2 구조 |
| v1.0.0 | 초기 릴리스, P/PD 제어기, 기본 ROS2 노드 |

**최종 업데이트**: 2026-03-09
**현재 버전**: v5.3.0
