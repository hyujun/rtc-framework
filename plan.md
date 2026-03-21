# RTC (Real-Time Controller) Repository Restructuring Plan

## 설계 원칙

1. **로봇 비종속(Robot-Agnostic)** — 모든 URDF 로봇에 적용 가능. UR5e 전용 코드는 별도 config/description으로 분리
2. **가변 DOF** — `kNumRobotJoints = 6` 컴파일 타임 상수 제거 → 런타임 URDF 파싱 기반 동적 DOF
3. **0.5ms (2kHz) RT loop 지원** — clock_nanosleep 기반, configurable sampling time (0.5ms ~ 10ms)
4. **관심사 분리** — Interface / Implementation / Manager / Communication 명확 분리
5. **`rtc_` namespace** — 범용 prefix, 로봇별 패키지만 로봇 이름 사용
6. **Inference 분리** — RT-safe 추론 엔진을 독립 패키지로 분리, 센서별 전처리는 드라이버에 잔류
7. **통신 확장성** — Transport 추상화로 UDP, CAN-FD, EtherCAT, RS485 등 새 프로토콜 쉽게 추가 가능
8. **Controller 계층 분리** — 범용 manipulator controller와 로봇+end-effector 통합 controller(demo) 명확 분리
9. **Logging 범용화** — 고정 크기 LogEntry(6-DOF, 10-motor) → 가변 DOF 로깅, 세션 디렉토리 로봇 비종속화

---

## 현재 구조의 문제점

1. **ur5e_rt_controller가 모놀리식** — Controller interface, 6개 controller 구현, RT loop, controller registry, launch files, RT setup scripts가 모두 하나의 패키지에 존재
2. **UR5e 하드코딩** — `kNumRobotJoints = 6` 컴파일 상수, URDF 경로 하드코딩, 기본 joint name이 UR5e 전용
3. **관심사 분리 부족** — controller 정의(interface)와 구현(implementations)과 관리(manager)가 혼재
4. **통신 계층 분산** — UDP 추상화는 ur5e_rt_base에, hand UDP는 별도 패키지, DDS config는 ur5e_rt_controller에 분산
5. **Bringup 분산** — launch 파일이 각 패키지에 흩어져 있고, 통합 launch 로직이 없음
6. **500Hz 고정** — sampling time이 코드에 묶여 있어 더 높은 주파수 대응 불가
7. **Inference 결합** — ONNX Runtime 추론 코드가 `ur5e_hand_udp`에 직접 임베딩. RT-safe 추론 래퍼를 다른 센서/모델에 재사용 불가
8. **통신 확장 불가** — UDP socket만 존재, CAN-FD/EtherCAT/RS485 등 다른 통신 방식을 추가하려면 전체 구조 변경 필요. Transport 추상화 부재
9. **Controller 혼재** — 범용 manipulator controller(P, PD, CLIK, OSC)와 UR5e+hand 통합 controller(DemoJoint, DemoTask)가 같은 패키지에 혼재. demo controller는 hand 제어를 포함하므로 특정 로봇 셋업에 종속
10. **Logging 고정 크기** — `LogEntry`가 `std::array<double, 6>` (robot), `std::array<float, 10>` (hand) 등 컴파일 타임 고정 크기 배열 사용. 7-DOF 로봇이나 다른 end-effector에 대응 불가. `DataLogger` CSV 컬럼도 6-joint/10-motor에 하드코딩. `UR5E_SESSION_DIR` 환경변수명이 UR5e 전용

---

## 제안하는 새 구조

```
ur5e-rt-controller/                      # (repo 이름은 유지, 내부만 범용화)
│
│  ── 범용 프레임워크 (robot-agnostic) ──
├── rtc_msgs/                            # [리네임] 커스텀 메시지 정의
├── rtc_base/                            # [리네임+축소] RT 인프라 공통 라이브러리
├── rtc_controller_interface/            # [신규] Controller 추상 인터페이스 (가변 DOF)
├── rtc_controllers/                     # [신규] 범용 manipulator Controller (P, PD, CLIK, OSC)
├── rtc_controller_manager/              # [신규] RT loop + controller lifecycle
├── rtc_communication/                   # [신규] 통신 계층 (Transport 추상화: UDP/CAN-FD/EtherCAT/RS485)
├── rtc_inference/                       # [신규] RT-safe 추론 엔진 (ONNX Runtime)
├── rtc_status_monitor/                  # [리네임] 안전 모니터링 (가변 DOF)
├── rtc_mujoco_sim/                      # [리네임] MuJoCo 시뮬레이션 (범용 URDF)
├── rtc_digital_twin/                    # [리네임] RViz2 시각화 (범용 URDF)
├── rtc_tools/                           # [리네임] 개발 유틸리티
├── rtc_scripts/                         # [신규] RT 시스템 설정/검증 스크립트 (robot-agnostic)
│
│  ── 로봇별 패키지 (robot-specific) ──
├── ur5e_description/                    # [유지] UR5e URDF/MJCF/meshes
├── ur5e_hand_driver/                    # [리네임] Hand 하드웨어 드라이버 (UR5e 전용 end-effector)
├── ur5e_bringup/                        # [신규] UR5e 전용 launch/config/demo controllers
│
│  ── 공통 ──
├── docs/
├── build.sh                             # [업데이트] 새 패키지 구조 대응
├── install.sh                           # [업데이트] rtc_scripts 기반 RT 설정
└── README.md
```

---

## 핵심 설계 변경사항

### A. 가변 DOF 지원 (kNumRobotJoints 제거)

**현재:**
```cpp
// types.hpp
inline constexpr int kNumRobotJoints = 6;  // 컴파일 타임 고정
std::array<double, kNumRobotJoints> q;      // 고정 크기 배열
```

**변경 후:**
```cpp
// rtc_base/types/robot_model.hpp
struct RobotModel {
  int num_joints;                           // URDF에서 런타임 결정
  std::vector<std::string> joint_names;
  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;
  std::vector<double> velocity_limits;
  std::vector<double> effort_limits;
  // pinocchio::Model은 controller_interface에서 관리
};

// RT-critical 경로에서는 Eigen::VectorXd 사용 (dynamic size)
// 또는 최대 DOF 템플릿으로 성능 보장:
template<int MaxDOF = 12>
using JointVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MaxDOF, 1>;
```

**전략:** Eigen::Dynamic + MaxDOF 상한을 사용하여 heap allocation 없이 가변 DOF 지원. RT loop 내에서 dynamic allocation 방지.

### B. 0.5ms (2kHz) RT Loop 지원

**현재:** `control_rate: 500` (2ms, 500Hz) 고정적 사용

**변경 후:**
```yaml
# config에서 sampling time 직접 지정
rt_controller_manager:
  ros__parameters:
    sampling_time_us: 500        # 0.5ms = 2kHz (최소값)
    # 또는
    control_rate_hz: 2000        # 둘 중 하나만 지정
```

**0.5ms 대응을 위한 아키텍처 고려사항:**
- `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 유지 — 0.5ms 정밀도 충분
- RT thread priority 99 + `SCHED_FIFO` 필수
- CPU isolation (`isolcpus`) + IRQ affinity 필수 (0.5ms에서 IRQ jitter가 deadline miss 유발)
- Lock-free publish offload 아키텍처 유지 — ROS2 publish는 RT loop 밖에서
- Controller `Compute()` 함수 실행 시간 budget 모니터링 강화 (timing profiler)
- DDS config에서 `max_message_size` 및 `receive_buffer_size` 최적화

### C. URDF 경로 외부화

**현재:** `ur5e_description` 하드코딩

**변경 후:**
```yaml
# ur5e_bringup/config/ur5e_robot.yaml
rt_controller_manager:
  ros__parameters:
    robot_description_package: "ur5e_description"
    urdf_path: "robots/ur5e/urdf/ur5e.urdf"
    # 또는 robot_description topic에서 수신
```

Launch argument로 URDF 경로를 주입하여 어떤 로봇이든 사용 가능.

### D. RT-safe Inference 엔진 분리

**현재:** `FingertipFTInferencer`에 ONNX Runtime 코드가 직접 임베딩
```cpp
// ur5e_hand_udp/fingertip_ft_inferencer.hpp 에 모든 것이 섞여 있음:
Ort::Env env_;                    // ← 범용 (추론 엔진)
Ort::Session session_;            // ← 범용 (추론 엔진)
Ort::IoBinding io_binding_;       // ← 범용 (추론 엔진)
float input_buffer_[...];        // ← 범용 (추론 엔진)
float prev_barometer_[...];      // ← 센서 전용 (전처리)
float baseline_offset_[...];     // ← 센서 전용 (캘리브레이션)
```

**변경 후:** 2계층 분리
```
rtc_inference (범용)          ur5e_hand_driver (센서 전용)
┌─────────────────────┐      ┌──────────────────────────────┐
│ OnnxEngine           │      │ FingertipFTInferencer        │
│  - Ort::Env          │◄─────│  - engine_: OnnxEngine       │
│  - Ort::Session(s)   │ uses │  - prev_barometer_           │
│  - IoBinding(s)      │      │  - baseline_offset_          │
│  - input/output buf  │      │  - FIFO history management   │
│  - Init() / Run()    │      │  - Calibration logic         │
└─────────────────────┘      │  - Barometer preprocessing   │
                              └──────────────────────────────┘
```

**재사용 시나리오:**
- 다른 센서의 F/T 추론 모델
- Contact detection 모델
- Anomaly detection 모델
- 향후 TensorRT/OpenVINO 백엔드 교체

### E. Transport 추상화 (통신 확장성)

**현재:** UDP socket만 존재 (`UdpSocket` RAII wrapper + `UdpTransceiver` template)
```cpp
// 현재: UDP에 직접 결합
class UdpSocket { /* AF_INET, SOCK_DGRAM only */ };

template <typename Codec>
class UdpTransceiver { /* UdpSocket에 직접 의존 */ };
```

**변경 후:** Transport 추상 인터페이스 + 프로토콜별 구현
```
rtc_communication (범용 Transport 계층)
┌─────────────────────────────────────────────────────────┐
│ TransportInterface (abstract)                           │
│  - Open() / Close()                                     │
│  - Send(span<const uint8_t>) noexcept → ssize_t         │
│  - Recv(span<uint8_t>) noexcept → ssize_t               │
│  - SetTimeout(ms)                                       │
│  - is_open() noexcept                                   │
├─────────────────────────────────────────────────────────┤
│ udp/UdpTransport         : TransportInterface           │
│ canfd/CanFdTransport     : TransportInterface (planned) │
│ ethercat/EcTransport     : TransportInterface (planned) │
│ serial/Rs485Transport    : TransportInterface (planned) │
├─────────────────────────────────────────────────────────┤
│ PacketCodec<C> concept   — 프로토콜 비종속 codec        │
│ Transceiver<Transport,Codec> — 범용 recv/send loop      │
└─────────────────────────────────────────────────────────┘
```

**확장 시나리오:**
- CAN-FD: Dynamixel, HEBI actuator 등 CAN 기반 servo 제어
- EtherLab 기반 EtherCAT: KUKA, Beckhoff, Elmo Gold 등 산업용 모터 드라이버
- RS485: Robotis, Feetech 등 serial servo 제어

### F. Controller 계층 분리 (범용 vs 통합 demo)

**현재:** 6개 controller가 모두 `ur5e_rt_controller`에 혼재
```
rtc_controllers/
├── p_controller           ← 범용 manipulator (any robot)
├── joint_pd_controller    ← 범용 manipulator (any robot)
├── clik_controller        ← 범용 manipulator (any robot)
├── osc_controller         ← 범용 manipulator (any robot)
├── demo_joint_controller  ← UR5e + Hand 통합 (robot-specific)
└── demo_task_controller   ← UR5e + Hand 통합 (robot-specific)
```

**변경 후:** 2계층 분리
```
rtc_controllers (범용)                  ur5e_bringup (로봇 전용)
┌──────────────────────────┐            ┌──────────────────────────────┐
│ Manipulator Controllers  │            │ Demo Controllers             │
│  - PController           │            │  - DemoJointController       │
│  - JointPDController     │            │    (arm 6-DOF + hand 10-DOF) │
│  - ClikController        │            │  - DemoTaskController        │
│  - OperationalSpaceCtrl  │            │    (CLIK arm + hand P ctrl)  │
│                          │            │                              │
│ Any URDF manipulator:    │            │ UR5e + custom hand 전용:     │
│ UR5e, KUKA, Franka, etc. │            │ ur5e_hand_driver 의존        │
└──────────────────────────┘            └──────────────────────────────┘
```

**분리 기준:**
- `SetHandTarget()` 사용 여부: demo controller만 hand를 직접 제어
- `ControllerOutput::hand_commands` 생성 여부: 범용 controller는 robot_commands만 생성
- `hand.motor_positions` 참조 여부: demo controller만 hand state를 read

### G. Logging 범용화 (가변 DOF + 세션 비종속화)

**현재:** `LogEntry`가 고정 크기 배열, `DataLogger`가 6-DOF/10-motor에 하드코딩
```cpp
// 현재 LogEntry (log_buffer.hpp) — 모든 크기가 컴파일 타임 고정
struct LogEntry {
  std::array<double, 6> goal_positions;        // ← 6-DOF 고정
  std::array<double, 6> actual_positions;      // ← 6-DOF 고정
  std::array<double, 6> robot_commands;        // ← 6-DOF 고정
  std::array<float, 10> hand_goal_positions;   // ← 10-motor 고정
  std::array<float, 10> hand_commands;         // ← 10-motor 고정
  std::array<int32_t, 88> hand_sensors;        // ← kMaxHandSensors 고정
  // ...
};
```

**변경 후:** MaxDOF 템플릿 + 런타임 실제 DOF 지정
```cpp
// rtc_base/logging/log_entry.hpp — 가변 DOF 대응
template <int MaxRobotDOF = 12, int MaxDeviceChannels = 64>
struct LogEntry {
  // ── Timing (로봇 비종속) ─────────────────────
  double timestamp;
  double t_state_acquire_us, t_compute_us, t_publish_us, t_total_us, jitter_us;

  // ── Robot state (가변 DOF) ───────────────────
  int num_robot_joints;                                     // 런타임 결정
  std::array<double, MaxRobotDOF> goal_positions{};
  std::array<double, MaxRobotDOF> actual_positions{};
  std::array<double, MaxRobotDOF> actual_velocities{};
  std::array<double, MaxRobotDOF> actual_torques{};
  std::array<double, MaxRobotDOF> robot_commands{};
  std::array<double, MaxRobotDOF> trajectory_positions{};
  std::array<double, MaxRobotDOF> trajectory_velocities{};
  std::array<double, 6> actual_task_positions{};            // TCP는 항상 6D (x,y,z,r,p,y)
  CommandType command_type{CommandType::kPosition};

  // ── Device state (end-effector, 선택적) ──────
  bool device_valid{false};
  int num_device_channels;                                  // 런타임 결정
  std::array<float, MaxDeviceChannels> device_goal{};
  std::array<float, MaxDeviceChannels> device_actual{};
  std::array<float, MaxDeviceChannels> device_commands{};

  // ── Sensor data (선택적, 로봇별) ─────────────
  int num_sensor_channels{0};
  std::array<float, 128> sensor_data{};                     // 범용 센서 버퍼
  std::array<float, 128> sensor_data_raw{};

  // ── Inference output (선택적) ────────────────
  bool inference_valid{false};
  int num_inference_values{0};
  std::array<float, 64> inference_output{};
};

// 기본 alias (기존 호환)
using DefaultLogEntry = LogEntry<12, 64>;
// UR5e 전용 (기존 크기와 동일한 성능 보장)
using Ur5eLogEntry = LogEntry<6, 10>;
```

**DataLogger 범용화:**
```cpp
// rtc_base/logging/data_logger.hpp
class DataLogger {
 public:
  struct Config {
    int num_robot_joints;                    // CSV 컬럼 수 결정
    int num_device_channels{0};             // 0이면 device 로그 생략
    int num_sensor_channels{0};
    int num_inference_values{0};
    std::vector<std::string> joint_names;    // CSV 헤더
    std::vector<std::string> device_names;   // CSV 헤더
    std::vector<std::string> sensor_names;   // CSV 헤더
    bool enable_timing_log{true};
    bool enable_robot_log{true};
    bool enable_device_log{true};            // hand_log → device_log
  };

  explicit DataLogger(const Config& config, const std::string& log_dir);
  // ...
};
```

**세션 디렉토리 범용화:**
```
// 환경변수 변경
UR5E_SESSION_DIR → RTC_SESSION_DIR

// 디렉토리 구조 범용화
logging_data/YYMMDD_HHMM/
  controller/   → 그대로 유지 (timing_log, robot_log, device_log)
  monitor/      → 그대로 유지
  device/       → hand/ 리네임 (범용 end-effector)
  sim/          → 그대로 유지
  plots/        → 그대로 유지
  motions/      → 그대로 유지
```

**핵심 변경 포인트:**
1. `LogEntry`: 고정 배열 → MaxDOF 템플릿 + `num_robot_joints` 런타임 지정
2. `DataLogger`: 하드코딩 컬럼 → `Config::num_robot_joints` 기반 동적 CSV 생성
3. `hand_log.csv` → `device_log.csv` (범용 end-effector)
4. `UR5E_SESSION_DIR` → `RTC_SESSION_DIR` (하위 호환: `UR5E_SESSION_DIR` fallback)
5. `session_dir.hpp`: 하위 디렉토리 `hand/` → `device/`
6. `SpscLogBuffer<N>`: 템플릿 파라미터로 `LogEntry` 타입 지정 가능
7. `publish_buffer.hpp`: `PublishSnapshot`도 동일하게 가변 DOF 대응

**RT 안전성 유지:**
- `MaxDOF` 상한으로 heap allocation 없이 가변 DOF 지원 (Eigen::Dynamic과 동일 패턴)
- `std::array<T, MaxDOF>`는 trivially copyable → `SeqLock`, SPSC 버퍼 호환
- `Push()`/`Pop()`은 기존과 동일하게 lock-free O(1)

---

## 패키지별 상세 계획

### 1. `rtc_msgs` — 리네임 (ur5e_msgs → rtc_msgs)

기존 7개 메시지 타입 유지. 네임스페이스만 `ur5e_msgs` → `rtc_msgs`로 변경.
UR5e 전용 필드가 있다면 범용화하거나 별도 분리.

### 2. `rtc_base` — 리네임+축소 (ur5e_rt_base → rtc_base)

Header-only RT 인프라 라이브러리. UDP 레이어는 `rtc_communication`으로 이동.

```
rtc_base/include/rtc_base/
├── types/
│   ├── types.hpp                  # 범용 type 정의 (kNumRobotJoints 제거)
│   └── robot_model.hpp            # [신규] 런타임 DOF 모델
├── threading/
│   ├── thread_utils.hpp
│   ├── thread_config.hpp
│   ├── publish_buffer.hpp         # [수정] PublishSnapshot 가변 DOF 대응
│   └── seqlock.hpp
├── filters/
│   ├── bessel_filter.hpp
│   └── kalman_filter.hpp
└── logging/
    ├── log_entry.hpp              # [수정] LogEntry 템플릿 (MaxRobotDOF, MaxDeviceChannels)
    ├── log_buffer.hpp             # [수정] SpscLogBuffer<EntryType> 타입 파라미터
    ├── data_logger.hpp            # [수정] DataLogger::Config 기반 동적 CSV 생성
    └── session_dir.hpp            # [수정] RTC_SESSION_DIR + hand/→device/ 리네임
```

**핵심 변경:**
- `kNumRobotJoints`, `kDefaultRobotJointNames` 상수 제거
- `RobotModel` 구조체 도입 (런타임 DOF)
- `udp/` 디렉토리 → `rtc_communication`으로 이동
- namespace: `ur5e_rt_base` → `rtc`

**Logging 변경:**
- `LogEntry` → `LogEntry<MaxRobotDOF, MaxDeviceChannels>` 템플릿으로 변경
  - `std::array<double, 6>` → `std::array<double, MaxRobotDOF>` + `num_robot_joints`
  - `hand_*` 필드 → `device_*` 필드 (범용 end-effector)
  - `hand_sensors` → `sensor_data` (범용 센서 버퍼)
  - F/T 전용 필드 → `inference_output` (범용 추론 결과)
- `SpscLogBuffer` → `SpscLogBuffer<EntryType>` 타입 파라미터 지원
- `DataLogger` → `DataLogger::Config` 구조체로 동적 CSV 컬럼 생성
  - `num_robot_joints`로 robot_log.csv 컬럼 수 결정
  - `num_device_channels`로 device_log.csv 컬럼 수 결정 (0이면 생략)
  - `hand_log.csv` → `device_log.csv` 리네임
- `session_dir.hpp`:
  - `UR5E_SESSION_DIR` → `RTC_SESSION_DIR` (하위 호환: `UR5E_SESSION_DIR` fallback)
  - `hand/` 하위 디렉토리 → `device/` 리네임
- `PublishSnapshot` → 가변 DOF 대응 (MaxRobotDOF 템플릿)

### 3. `rtc_controller_interface` — 신규 (ur5e_rt_controller에서 분리)

Controller의 추상 인터페이스. ros2_control의 `controller_interface` 패턴 참고.

```
rtc_controller_interface/
├── include/rtc_controller_interface/
│   ├── rt_controller_interface.hpp     # 추상 인터페이스 (가변 DOF)
│   ├── controller_types.hpp            # Controller 관련 type 정의
│   └── controller_state.hpp            # Controller state 구조체
├── src/
│   └── rt_controller_interface.cpp
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_base`, `rtc_msgs`, `sensor_msgs`, `pinocchio`

**핵심 변경:**
- `Init()`에서 `RobotModel`을 받아 DOF를 런타임에 결정
- pinocchio model 빌드를 interface 레벨에서 공통 처리
- state/command 벡터를 `Eigen::VectorXd`로 변경

### 4. `rtc_controllers` — 신규 (ur5e_rt_controller에서 분리)

**범용 manipulator controller만 포함.** 어떤 로봇(UR5e, KUKA, Franka 등)에든 사용 가능한 controller.
Demo controller (arm+hand 통합)는 `ur5e_bringup`으로 이동 (아래 14번 참조).

```
rtc_controllers/
├── include/rtc_controllers/
│   ├── direct/                          # Torque command controllers
│   │   ├── joint_pd_controller.hpp      # PD + gravity/Coriolis (any manipulator)
│   │   └── operational_space_controller.hpp  # 6-DOF Cartesian PD (any manipulator)
│   ├── indirect/                        # Position command controllers
│   │   ├── p_controller.hpp             # Simple P controller (any manipulator)
│   │   └── clik_controller.hpp          # Closed-Loop IK (any manipulator)
│   └── trajectory/
│       ├── trajectory_utils.hpp
│       ├── joint_space_trajectory.hpp
│       └── task_space_trajectory.hpp
├── src/controllers/
│   ├── direct/
│   │   ├── joint_pd_controller.cpp
│   │   └── operational_space_controller.cpp
│   └── indirect/
│       ├── p_controller.cpp
│       └── clik_controller.cpp
├── config/controllers/
│   ├── direct/
│   │   ├── joint_pd_controller.yaml
│   │   └── operational_space_controller.yaml
│   └── indirect/
│       ├── p_controller.yaml
│       └── clik_controller.yaml
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_controller_interface`, `rtc_base`, `rtc_msgs`, `pinocchio`

**핵심 변경:**
- 모든 controller가 가변 DOF 지원 (고정 크기 배열 → Eigen::VectorXd)
- **demo_joint_controller, demo_task_controller 제외** → `ur5e_bringup`으로 이동 (UR5e+hand 통합 로직이므로)
- `ur5e_hand_controller`도 제외 → `ur5e_hand_driver`에 잔류 (UR5e 전용 end-effector)
- **4개 범용 controller만 포함:** PController, JointPDController, ClikController, OperationalSpaceController
- 이 controller들은 `ControllerOutput::robot_commands`만 생성, `hand_commands`는 건드리지 않음

### 5. `rtc_controller_manager` — 신규 (ur5e_rt_controller 핵심 분리)

RT loop 실행, controller lifecycle 관리, runtime switching 담당.

```
rtc_controller_manager/
├── include/rtc_controller_manager/
│   ├── rt_controller_node.hpp
│   ├── controller_registry.hpp          # Controller 등록/검색 (node에서 분리)
│   └── controller_timing_profiler.hpp
├── src/
│   ├── rt_controller_node.cpp
│   └── rt_controller_main.cpp
├── config/
│   ├── rt_controller_manager.yaml       # 범용 config (sampling_time_us 포함)
│   └── cyclone_dds.xml
├── test/
│   └── ...
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_controller_interface`, `rtc_controllers`, `rtc_base`, `rtc_communication`, `rtc_status_monitor`, `rtc_msgs`

**핵심 변경:**
- `sampling_time_us` 파라미터로 RT loop 주기 설정 (기본 2000us=500Hz, 최소 500us=2kHz)
- URDF 경로를 파라미터/topic으로 수신 (하드코딩 제거)
- `RobotModel`을 URDF에서 빌드하여 controller에 전달
- UR5e 전용 코드 (`enable_ur5e`, `ur5e_hand_controller` 등) 제거 → bringup config로 이동
- timing profiler에서 deadline miss 경고 임계값을 sampling_time 기반으로 동적 계산

### 6. `rtc_communication` — 신규 (ur5e_rt_base UDP 이동 + Transport 추상화)

범용 하드웨어 통신 추상화. Transport interface + 프로토콜별 구현 + codec concept + transceiver template.

```
rtc_communication/
├── include/rtc_communication/
│   ├── transport_interface.hpp          # [신규] Transport 추상 인터페이스
│   ├── packet_codec.hpp                 # [리네임] 범용 패킷 codec concept (기존 udp_codec.hpp 확장)
│   ├── transceiver.hpp                  # [리네임] Transport 기반 범용 transceiver (기존 udp_transceiver.hpp 범용화)
│   ├── udp/                             # UDP transport 구현
│   │   ├── udp_transport.hpp            # TransportInterface 구현 (기존 udp_socket.hpp 래핑)
│   │   └── udp_socket.hpp               # [이동] 기존 UDP socket RAII wrapper
│   ├── canfd/                           # [planned] CAN-FD transport
│   │   └── canfd_transport.hpp          # SocketCAN AF_CAN + CANFD_MTU
│   ├── ethercat/                        # [planned] EtherLab EtherCAT transport
│   │   └── ec_transport.hpp             # ecrt_master / ecrt_domain 래핑
│   └── serial/                          # [planned] RS485 serial transport
│       └── rs485_transport.hpp          # termios B* + RS485 ioctl
├── src/
│   ├── udp/
│   │   └── udp_transport.cpp
│   ├── canfd/                           # [planned]
│   │   └── canfd_transport.cpp
│   ├── ethercat/                        # [planned]
│   │   └── ec_transport.cpp
│   └── serial/                          # [planned]
│       └── rs485_transport.cpp
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_base`, (optional: `libsocketcan`, `etherlab`, `serial`)

**핵심 설계:**

1. **Transport 추상 인터페이스:**
   ```cpp
   // rtc_communication/transport_interface.hpp
   class TransportInterface {
    public:
     virtual ~TransportInterface() = default;

     /// non-RT: transport 열기 (소켓/디바이스 초기화)
     [[nodiscard]] virtual bool Open() = 0;
     virtual void Close() noexcept = 0;

     /// RT-safe: 데이터 송수신 (allocation-free, noexcept)
     [[nodiscard]] virtual ssize_t Send(
         std::span<const uint8_t> data) noexcept = 0;
     [[nodiscard]] virtual ssize_t Recv(
         std::span<uint8_t> buffer) noexcept = 0;

     virtual void SetRecvTimeout(int timeout_ms) noexcept = 0;
     [[nodiscard]] virtual bool is_open() const noexcept = 0;
   };
   ```

2. **UdpTransport 구현 (기존 UdpSocket 래핑):**
   ```cpp
   // rtc_communication/udp/udp_transport.hpp
   class UdpTransport : public TransportInterface {
    public:
     struct Config {
       std::string bind_address{"0.0.0.0"};
       int bind_port{0};
       std::string target_address;
       int target_port{0};
       int recv_buffer_size{256 * 1024};
     };

     explicit UdpTransport(const Config& config);
     // TransportInterface 구현...
   };
   ```

3. **범용 Transceiver (Transport 기반):**
   ```cpp
   // rtc_communication/transceiver.hpp
   template <typename Codec>
     requires PacketCodec<Codec>
   class Transceiver {
    public:
     explicit Transceiver(
         std::unique_ptr<TransportInterface> transport,
         const ThreadConfig& thread_cfg);

     [[nodiscard]] bool Start();
     void Stop() noexcept;
     // recv loop는 TransportInterface::Recv()를 사용
   };
   ```

4. **PacketCodec concept (기존 UdpPacketCodec 범용화):**
   ```cpp
   // rtc_communication/packet_codec.hpp
   // 기존 UdpPacketCodec 리네임, transport 비종속
   template <typename C>
   concept PacketCodec = requires {
     typename C::RecvPacket;
     typename C::SendPacket;
     typename C::State;
   } && std::is_trivially_copyable_v<typename C::RecvPacket>
     && std::is_trivially_copyable_v<typename C::SendPacket>
     && requires(std::span<const uint8_t> buf, typename C::State& state) {
       { C::Decode(buf, state) } -> std::same_as<bool>;
     };
   ```

5. **[planned] CAN-FD Transport:**
   ```cpp
   // rtc_communication/canfd/canfd_transport.hpp
   class CanFdTransport : public TransportInterface {
    public:
     struct Config {
       std::string interface{"can0"};  // SocketCAN interface
       uint32_t bitrate{1000000};      // 1 Mbps
       uint32_t dbitrate{5000000};     // 5 Mbps data phase
       uint32_t can_id{0};            // filter CAN ID
     };
     // AF_CAN + CANFD_MTU (64 bytes)
   };
   ```

6. **[planned] EtherLab EtherCAT Transport:**
   ```cpp
   // rtc_communication/ethercat/ec_transport.hpp
   class EcTransport : public TransportInterface {
    public:
     struct Config {
       int master_index{0};
       uint16_t alias{0};
       uint16_t position{0};
       uint32_t vendor_id{0};
       uint32_t product_code{0};
     };
     // ecrt_master_create() / ecrt_domain_process()
     // cyclic PDO 기반 RT-safe data exchange
   };
   ```

7. **[planned] RS485 Serial Transport:**
   ```cpp
   // rtc_communication/serial/rs485_transport.hpp
   class Rs485Transport : public TransportInterface {
    public:
     struct Config {
       std::string device{"/dev/ttyUSB0"};
       speed_t baudrate{B1000000};     // 1 Mbps
       bool rs485_mode{true};          // SER_RS485_ENABLED
       int response_delay_us{0};
     };
     // termios + TIOCSRS485 ioctl
   };
   ```

**역할:**
- Transport 추상화로 프로토콜 비종속 통신
- 새 프로토콜 추가 시 `TransportInterface` 구현체만 작성하면 기존 Codec/Transceiver 재사용
- 로봇별 프로토콜 구현은 각 로봇 드라이버 패키지에서 Codec 담당
- **Hand UDP는 여기에 포함하지 않음** → `ur5e_hand_driver`에서 `UdpTransport` + hand-specific codec을 사용

**조건부 빌드:**
```cmake
# CMakeLists.txt
option(RTC_COMM_UDP      "Build UDP transport"      ON)
option(RTC_COMM_CANFD    "Build CAN-FD transport"   OFF)
option(RTC_COMM_ETHERCAT "Build EtherCAT transport" OFF)
option(RTC_COMM_RS485    "Build RS485 transport"     OFF)

if(RTC_COMM_CANFD)
  find_package(socketcan REQUIRED)
  add_library(rtc_canfd_transport ...)
endif()
```

### 7. `rtc_inference` — 신규 (ur5e_hand_udp에서 추론 엔진 분리)

범용 RT-safe 추론 엔진. 현재 `FingertipFTInferencer`에 임베딩된 ONNX Runtime 래퍼를 일반화.

```
rtc_inference/
├── include/rtc_inference/
│   ├── inference_engine.hpp             # 추론 엔진 추상 인터페이스
│   ├── onnx/
│   │   ├── onnx_session.hpp             # RT-safe ONNX 세션 래퍼
│   │   ├── onnx_tensor_pool.hpp         # 사전 할당 I/O 텐서 풀
│   │   └── onnx_engine.hpp              # InferenceEngine의 ONNX 구현체
│   └── inference_types.hpp              # 공통 타입 (InferenceResult, TensorSpec 등)
├── src/
│   └── onnx/
│       ├── onnx_session.cpp
│       └── onnx_engine.cpp
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_base`, ONNX Runtime (optional, `HAS_ONNXRUNTIME` 유지)

**설계 원칙:**

1. **RT-safe Inference 패턴 추출:**
   현재 `FingertipFTInferencer`에 있는 다음 패턴을 범용화:
   - `Init()`: non-RT 컨텍스트에서 Session 생성, 텐서 사전 할당, IoBinding, warmup
   - `Run()`: noexcept + allocation-free 추론 (IoBinding으로 zero-copy)
   - Stub 구현 (ONNX Runtime 미설치 시 graceful fallback)

2. **추상 인터페이스:**
   ```cpp
   // rtc_inference/inference_engine.hpp
   class InferenceEngine {
    public:
     virtual ~InferenceEngine() = default;

     /// non-RT: 모델 로드, 텐서 할당, warmup
     virtual void Init(const ModelConfig& config) = 0;

     /// RT-safe: 사전 할당된 input buffer에 데이터가 채워진 상태에서 추론 실행
     /// @return true if inference succeeded
     [[nodiscard]] virtual bool Run() noexcept = 0;

     /// 사전 할당된 input/output buffer 접근
     virtual float* input_buffer(int model_idx = 0) noexcept = 0;
     virtual const float* output_buffer(int model_idx = 0) const noexcept = 0;

     [[nodiscard]] virtual bool is_initialized() const noexcept = 0;
   };
   ```

3. **OnnxEngine 구현:**
   ```cpp
   // rtc_inference/onnx/onnx_engine.hpp
   struct ModelConfig {
     std::string model_path;
     std::vector<int64_t> input_shape;   // e.g., {1, 12, 16}
     std::vector<int64_t> output_shape;  // e.g., {1, 1, 13}
     int intra_op_threads{1};            // RT: single-threaded 추론
   };

   class OnnxEngine : public InferenceEngine {
     // Ort::Session, IoBinding, pre-allocated buffers
     // 복수 모델 지원 (fingertip처럼 per-unit 모델)
   };
   ```

4. **향후 확장:**
   - TensorRT 백엔드 (`TensorRTEngine : InferenceEngine`)
   - OpenVINO 백엔드
   - 다른 센서 모델 (force estimation, contact detection, anomaly detection 등)

**핵심 변경 (기존 FingertipFTInferencer 대비):**
- ONNX 세션 관리 (`Ort::Env`, `Ort::Session`, `Ort::IoBinding`) → `rtc_inference`로 이동
- 텐서 사전 할당 + warmup 로직 → `rtc_inference`로 이동
- Barometer 전처리, delta 계산, FIFO history, calibration → `ur5e_hand_driver`에 잔류
- `ur5e_hand_driver`의 `FingertipFTInferencer`는 `rtc_inference::OnnxEngine`을 내부적으로 사용

### 8. `rtc_status_monitor` — 리네임 (ur5e_status_monitor → rtc_status_monitor)

```
rtc_status_monitor/
└── (기존 구조 유지, namespace 변경)
```

**핵심 변경:**
- `kNumJoints` 하드코딩 제거 → `RobotModel`에서 DOF 수신
- namespace: `ur5e_status_monitor` → `rtc`

### 8. `rtc_mujoco_sim` — 리네임 (ur5e_mujoco_sim → rtc_mujoco_sim)

**핵심 변경:**
- MJCF 경로를 파라미터로 수신 (하드코딩 제거)
- 어떤 URDF/MJCF 로봇이든 시뮬레이션 가능

### 9. `rtc_digital_twin` — 리네임 (ur5e_digital_twin → rtc_digital_twin)

**핵심 변경:**
- URDF를 `robot_description` topic에서 수신 → 어떤 로봇이든 시각화 가능

### 10. `rtc_tools` — 리네임 (ur5e_tools → rtc_tools)

기존 Python 유틸리티 유지, namespace만 변경.

**`plot_ur_log.py` → `plot_rtc_log.py` 범용화:**

현재 문제점:
- Joint 수 `6` 하드코딩 (`_detect_joint_columns(df, prefix, 6)`)
- `NUM_HAND_MOTORS = 10` 하드코딩
- Subplot grid `(3, 2)` / `(2, 5)` 고정 (6-joint / 10-motor 전용)
- `UR5E_SESSION_DIR` 환경변수 참조
- `hand_log` 파일명 감지 (→ `device_log`)
- Title/description에 "UR5e" 하드코딩

변경사항:
1. **가변 DOF 자동 감지** — CSV 헤더에서 `actual_pos_*` 컬럼 수를 자동 감지하여 joint 수 결정
   ```python
   def _detect_num_joints(df, prefix='actual_pos_'):
       """CSV 헤더에서 prefix로 시작하는 컬럼 수를 자동 감지."""
       return len([c for c in df.columns if c.startswith(prefix)])
   ```
2. **동적 subplot grid** — `math.ceil(n_joints / 2)` × 2 그리드, 1-column 자동 대응
3. **`NUM_HAND_MOTORS` 제거** — device 채널 수도 CSV 헤더에서 자동 감지
4. **`UR5E_SESSION_DIR` → `RTC_SESSION_DIR`** (하위 호환 fallback 유지)
5. **`hand_log` → `device_log`** 파일명 감지 추가 (하위 호환: `hand_log`도 계속 인식)
6. **파일명 변경** — `plot_ur_log.py` → `plot_rtc_log.py`
7. **Title 범용화** — "UR5e" 제거, "Robot Joint Positions" 유지
8. **`JOINT_NAMES_DEFAULT` 제거** — CSV 헤더의 named columns에서 표시 이름 추출, 없으면 `J0`, `J1`, ... 사용

### 11. `rtc_scripts` — 신규 (ur5e_rt_controller/scripts 이동 + 범용화)

Robot-agnostic RT 시스템 설정/검증 스크립트 패키지. 기존 `ur5e_rt_controller/scripts/`의 스크립트를 로봇 비종속으로 범용화.

```
rtc_scripts/
├── scripts/
│   ├── lib/
│   │   └── rt_common.sh                 # [이동] 공통 유틸리티 (get_physical_cores, compute_cpu_layout 등)
│   ├── setup_irq_affinity.sh            # [이동] NIC IRQ affinity 설정 (Core 0-1)
│   ├── setup_udp_optimization.sh        # [이동] UDP socket/network 최적화
│   ├── setup_nvidia_rt.sh               # [이동] NVIDIA + RT kernel 공존 (DKMS RT bypass)
│   ├── build_rt_kernel.sh               # [이동] PREEMPT_RT kernel 빌드 헬퍼
│   ├── check_rt_setup.sh                # [이동] RT 설정 검증 (8 categories)
│   ├── cpu_shield.sh                    # [이동] cset shield on/off
│   └── verify_rt_runtime.sh             # [이동] RT 런타임 검증
├── package.xml                          # ament_cmake (scripts install)
└── CMakeLists.txt
```

**의존성:** 없음 (독립, shell script only)

**역할:**
- RT 커널 빌드, IRQ affinity, CPU 격리, NVIDIA RT 설정 등 **어떤 로봇에든 공통**으로 필요한 RT 시스템 스크립트
- `install.sh`와 `build.sh`가 `rtc_scripts/scripts/`를 참조
- 로봇별 bringup에서는 `rtc_scripts`를 depend하여 사용 (직접 스크립트를 복사하지 않음)

**기존 스크립트 범용화 변경:**
- `setup_irq_affinity.sh`: `ur5e_rt_controller` 경로 참조 제거, 범용 NIC 인터페이스명 파라미터화
- `check_rt_setup.sh`: 로봇 이름 하드코딩 제거, 범용 검증 항목만 유지
- `rt_common.sh`: namespace 변경 없음 (이미 범용)

### 12. `ur5e_description` — 유지 (변경 없음)

UR5e 전용 URDF/MJCF/meshes. 로봇별 패키지이므로 `ur5e_` prefix 유지.

### 13. `ur5e_hand_driver` — 리네임 (ur5e_hand_udp → ur5e_hand_driver)

UR5e 전용 end-effector 드라이버. `rtc_communication`의 UDP 추상화 + `rtc_inference`의 추론 엔진을 사용.

```
ur5e_hand_driver/
├── include/ur5e_hand_driver/
│   ├── hand_packets.hpp
│   ├── hand_udp_codec.hpp
│   ├── hand_controller.hpp              # ← 기존 ur5e_hand_controller도 여기로
│   ├── hand_failure_detector.hpp
│   └── fingertip_ft_inferencer.hpp      # 센서 전처리 + rtc_inference::OnnxEngine 사용
├── src/
│   └── hand_udp_node.cpp
├── config/
│   ├── hand_udp_node.yaml
│   └── fingertip_ft_inferencer.yaml
├── launch/
│   └── hand_udp.launch.py
├── models/                              # ONNX 모델 파일
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_communication`, `rtc_inference`, `rtc_base`, `rtc_msgs`, `rtc_controller_interface`

**핵심 변경 (inference 분리):**
- `FingertipFTInferencer`가 직접 `Ort::Session`/`Ort::IoBinding`을 관리하지 않음
- 대신 `rtc_inference::OnnxEngine`을 멤버로 소유하고, `Init()`에서 모델 로드 위임
- Barometer 전처리 → FIFO history → input buffer 채우기 → `engine.Run()` → output 해석
- Calibration (baseline offset) 로직은 그대로 `FingertipFTInferencer`에 잔류

### 14. `ur5e_bringup` — 신규 (UR5e 전용 launch/config/demo controllers 통합)

UR5e에 특화된 launch, config, 그리고 **UR5e+hand 통합 demo controller**를 포함하는 패키지.
RT 시스템 스크립트는 `rtc_scripts`에서 관리 (로봇 비종속).

```
ur5e_bringup/
├── include/ur5e_bringup/
│   └── controllers/                     # [신규] UR5e+hand 통합 controller
│       ├── demo_joint_controller.hpp    # [이동] Arm P + Hand P 통합 controller
│       └── demo_task_controller.hpp     # [이동] CLIK arm + Hand P 통합 controller
├── src/
│   └── controllers/
│       ├── demo_joint_controller.cpp
│       └── demo_task_controller.cpp
├── launch/
│   ├── robot.launch.py                  # 실물 로봇 launch
│   ├── sim.launch.py                    # MuJoCo 시뮬레이션 launch
│   ├── full.launch.py                   # 로봇 + 디지털 트윈 + 모니터링 통합
│   └── hand.launch.py                   # Hand driver launch
├── config/
│   ├── ur5e_robot.yaml                  # UR5e 전용 파라미터 (URDF 경로, joint names, limits)
│   ├── ur5e_controllers.yaml            # UR5e용 controller 파라미터 오버라이드
│   ├── ur5e_rt.yaml                     # UR5e용 RT 설정 (sampling_time, CPU cores)
│   └── controllers/
│       ├── demo_joint_controller.yaml   # [이동] Demo joint controller 설정
│       └── demo_task_controller.yaml    # [이동] Demo task controller 설정
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_controller_manager`, `rtc_controller_interface`, `rtc_controllers`, `rtc_scripts`, `rtc_mujoco_sim`, `rtc_digital_twin`, `ur5e_hand_driver`, `ur5e_description`, `pinocchio`

**역할:**
- UR5e 전용 launch (URDF 경로, joint name, sampling_time 등을 argument로 주입)
- **UR5e+hand 통합 demo controller 소유:**
  - `DemoJointController`: arm 6-DOF P control + hand 10-DOF P control (joint space)
  - `DemoTaskController`: arm CLIK (3/6-DOF task space) + hand P control + E-STOP
  - 이 controller들은 `ur5e_hand_driver`에 의존하여 hand state/command를 직접 처리
  - `rtc_controller_manager`의 controller registry에 plugin으로 등록
- 다른 로봇 사용 시 → `{robot_name}_bringup` 패키지를 만들고 자체 demo controller 포함

**Demo controller가 ur5e_bringup에 있는 이유:**
- `SetHandTarget()`, `hand.motor_positions` 등 hand-specific API를 사용
- `ur5e_hand_driver`에 직접 의존 (UR5e 셋업 전용)
- 범용 manipulator에는 hand가 없으므로 `rtc_controllers`에 부적합
- 다른 로봇+end-effector 조합은 자체 bringup 패키지에 통합 controller를 작성

---

## 패키지 의존성 그래프

```
rtc_msgs (독립)
rtc_base (독립, header-only)
    │
    ├── rtc_communication ← rtc_base                         [범용 Transport 추상화]
    │       (optional: libsocketcan, etherlab, serial)        [UDP/CAN-FD/EtherCAT/RS485]
    │
    ├── rtc_inference ← rtc_base                             [범용 RT-safe 추론]
    │                    (optional: ONNX Runtime)
    │
    ├── rtc_controller_interface ← rtc_base, rtc_msgs        [범용 인터페이스]
    │       │
    │       └── rtc_controllers ← rtc_controller_interface    [범용 manipulator 컨트롤러]
    │               │               (P, PD, CLIK, OSC만 포함)
    │               │
    │               └── rtc_controller_manager                [범용 매니저]
    │                    ← rtc_controllers, rtc_controller_interface,
    │                       rtc_communication, rtc_status_monitor
    │
    ├── rtc_status_monitor ← rtc_base, rtc_msgs              [범용 모니터링]
    │
    ├── rtc_mujoco_sim (독립, MJCF 파라미터화)                [범용 시뮬레이션]
    │
    ├── rtc_digital_twin (독립, Python)                       [범용 시각화]
    │
    rtc_tools (독립, Python)                                  [범용 유틸]
    rtc_scripts (독립, shell scripts)                          [RT 시스템 설정]

ur5e_description (독립, 로봇별)
    │
    └── ur5e_hand_driver ← rtc_communication, rtc_inference,  [UR5e 전용 드라이버]
    │                       rtc_base, rtc_msgs,
    │                       rtc_controller_interface
    │
    └── ur5e_bringup ← rtc_controller_manager,                [UR5e 전용 bringup]
                        rtc_controller_interface,
                        rtc_controllers, rtc_scripts,          [demo controller + RT scripts]
                        rtc_mujoco_sim, rtc_digital_twin,
                        ur5e_hand_driver, ur5e_description,
                        pinocchio
                        ※ DemoJointController, DemoTaskController 소유
```

**다른 로봇 추가 시:**
```
panda_description/          # Panda URDF/meshes
panda_bringup/              # Panda launch/config + Panda 전용 controller
                            # rtc_controller_manager, rtc_controllers 재사용

kuka_description/           # KUKA URDF/meshes
kuka_servo_driver/          # KUKA용 EtherCAT 드라이버 (rtc_communication::EcTransport 사용)
kuka_bringup/               # KUKA launch/config + KUKA 전용 controller
```
→ `rtc_*` 패키지는 재사용, 로봇별 패키지만 추가하면 됨.
→ 새 통신 프로토콜이 필요하면 `rtc_communication`에 Transport 구현체 추가.

---

## 마이그레이션 순서 (단계별)

### Phase 1: 기반 패키지 (의존성 없는 것부터)
1. `rtc_msgs` 생성 — ur5e_msgs 리네임 + namespace 변경
2. `rtc_base` 생성 — ur5e_rt_base 리네임, `kNumRobotJoints` 제거, `RobotModel` 도입, udp/ 분리, Logging 범용화 (LogEntry 템플릿, DataLogger Config 기반, session_dir RTC_SESSION_DIR)
3. `rtc_communication` 생성 — Transport 추상 인터페이스 + UdpTransport 구현 (ur5e_rt_base에서 udp/ 이동 + 범용화)
4. `rtc_inference` 생성 — ur5e_hand_udp에서 ONNX Runtime 래퍼 추출, 범용 추론 엔진 구현
5. `rtc_scripts` 생성 — ur5e_rt_controller/scripts/ 이동, 로봇 이름 하드코딩 제거, 경로 범용화

### Phase 2: Controller 패키지 분리
6. `rtc_controller_interface` 생성 — ur5e_rt_controller에서 interface 분리, 가변 DOF 적용
7. `rtc_controllers` 생성 — **4개 범용 manipulator controller만 이동** (P, PD, CLIK, OSC), 가변 DOF 적용
8. `rtc_status_monitor` 생성 — ur5e_status_monitor 리네임, 가변 DOF 적용

### Phase 3: Manager + 통합
9. `rtc_controller_manager` 생성 — RT loop, node, registry 이동, `sampling_time_us` 파라미터화, controller plugin 시스템
10. `rtc_mujoco_sim` 리네임 — MJCF 경로 파라미터화
11. `rtc_digital_twin` 리네임 — robot_description topic 기반
12. `rtc_tools` 리네임

### Phase 4: 로봇별 패키지
13. `ur5e_hand_driver` 생성 — ur5e_hand_udp 리네임 + `rtc_communication::UdpTransport` 사용 + `rtc_inference` 연동 + ur5e_hand_controller 이동
14. `ur5e_bringup` 생성 — launch/config 통합 + **demo_joint_controller, demo_task_controller 이동** (UR5e+hand 통합 controller)

### Phase 5: 빌드 시스템 + 정리
15. `build.sh` 업데이트 — 패키지 목록 `rtc_*` 기반으로 변경, 스크립트 경로 `rtc_scripts/` 참조
16. `install.sh` 업데이트 — RT setup 스크립트 경로 `rtc_scripts/` 기반, `rt_common.sh` 소싱 경로 변경
17. 기존 `ur5e_rt_controller`, `ur5e_hand_udp`, `ur5e_rt_base`, `ur5e_status_monitor` 패키지 제거
18. README.md 업데이트
19. CI/CD 설정 업데이트

### Phase 6: [선택] 추가 Transport 구현
20. `rtc_communication`에 CAN-FD transport 추가 (SocketCAN 기반)
21. `rtc_communication`에 EtherCAT transport 추가 (EtherLab 기반)
22. `rtc_communication`에 RS485 transport 추가 (termios 기반)

---

## 기존 → 새 구조 파일 매핑

| 기존 위치 | 새 위치 |
|-----------|---------|
| `ur5e_msgs/` | `rtc_msgs/` (namespace 변경) |
| `ur5e_rt_base/include/.../types/` | `rtc_base/include/rtc_base/types/` |
| `ur5e_rt_base/include/.../threading/` | `rtc_base/include/rtc_base/threading/` |
| `ur5e_rt_base/include/.../filters/` | `rtc_base/include/rtc_base/filters/` |
| `ur5e_rt_base/include/.../logging/log_buffer.hpp` | `rtc_base/include/rtc_base/logging/log_entry.hpp` (LogEntry 분리) + `log_buffer.hpp` (SpscLogBuffer 타입 파라미터화) |
| `ur5e_rt_base/include/.../logging/data_logger.hpp` | `rtc_base/include/rtc_base/logging/data_logger.hpp` (Config 기반 동적 CSV, hand→device 리네임) |
| `ur5e_rt_base/include/.../logging/session_dir.hpp` | `rtc_base/include/rtc_base/logging/session_dir.hpp` (UR5E_SESSION_DIR→RTC_SESSION_DIR, hand/→device/) |
| `ur5e_rt_base/include/.../udp/udp_socket.hpp` | `rtc_communication/include/rtc_communication/udp/udp_socket.hpp` |
| `ur5e_rt_base/include/.../udp/udp_codec.hpp` | `rtc_communication/include/rtc_communication/packet_codec.hpp` (범용화) |
| `ur5e_rt_base/include/.../udp/udp_transceiver.hpp` | `rtc_communication/include/rtc_communication/transceiver.hpp` (Transport 기반 범용화) |
| — (신규) | `rtc_communication/include/rtc_communication/transport_interface.hpp` |
| — (신규) | `rtc_communication/include/rtc_communication/udp/udp_transport.hpp` |
| `ur5e_rt_controller/.../rt_controller_interface.hpp` | `rtc_controller_interface/include/rtc_controller_interface/` |
| `ur5e_rt_controller/src/rt_controller_interface.cpp` | `rtc_controller_interface/src/` |
| `ur5e_rt_controller/.../controllers/direct/` | `rtc_controllers/include/rtc_controllers/direct/` |
| `ur5e_rt_controller/.../controllers/indirect/p_controller.hpp` | `rtc_controllers/include/rtc_controllers/indirect/` |
| `ur5e_rt_controller/.../controllers/indirect/clik_controller.hpp` | `rtc_controllers/include/rtc_controllers/indirect/` |
| `ur5e_rt_controller/.../controllers/indirect/demo_joint_controller.hpp` | `ur5e_bringup/include/ur5e_bringup/controllers/` (UR5e+hand 통합) |
| `ur5e_rt_controller/.../controllers/indirect/demo_task_controller.hpp` | `ur5e_bringup/include/ur5e_bringup/controllers/` (UR5e+hand 통합) |
| `ur5e_rt_controller/src/controllers/` (범용 4개) | `rtc_controllers/src/controllers/` |
| `ur5e_rt_controller/src/controllers/indirect/demo_*.cpp` | `ur5e_bringup/src/controllers/` |
| `ur5e_rt_controller/.../trajectory/` | `rtc_controllers/include/rtc_controllers/trajectory/` |
| `ur5e_rt_controller/config/controllers/` | `rtc_controllers/config/controllers/` |
| `ur5e_rt_controller/.../rt_controller_node.hpp` | `rtc_controller_manager/include/rtc_controller_manager/` |
| `ur5e_rt_controller/src/rt_controller_node.cpp` | `rtc_controller_manager/src/` |
| `ur5e_rt_controller/src/rt_controller_main.cpp` | `rtc_controller_manager/src/` |
| `ur5e_rt_controller/.../controller_timing_profiler.hpp` | `rtc_controller_manager/include/rtc_controller_manager/` |
| `ur5e_rt_controller/config/ur5e_rt_controller.yaml` | `ur5e_bringup/config/ur5e_robot.yaml` (UR5e 부분) + `rtc_controller_manager/config/rt_controller_manager.yaml` (범용 부분) |
| `ur5e_rt_controller/config/cyclone_dds.xml` | `rtc_controller_manager/config/` |
| `ur5e_rt_controller/launch/ur_control.launch.py` | `ur5e_bringup/launch/robot.launch.py` |
| `ur5e_rt_controller/scripts/setup_irq_affinity.sh` | `rtc_scripts/scripts/setup_irq_affinity.sh` |
| `ur5e_rt_controller/scripts/setup_udp_optimization.sh` | `rtc_scripts/scripts/setup_udp_optimization.sh` |
| `ur5e_rt_controller/scripts/setup_nvidia_rt.sh` | `rtc_scripts/scripts/setup_nvidia_rt.sh` |
| `ur5e_rt_controller/scripts/build_rt_kernel.sh` | `rtc_scripts/scripts/build_rt_kernel.sh` |
| `ur5e_rt_controller/scripts/check_rt_setup.sh` | `rtc_scripts/scripts/check_rt_setup.sh` |
| `ur5e_rt_controller/scripts/cpu_shield.sh` | `rtc_scripts/scripts/cpu_shield.sh` |
| `ur5e_rt_controller/scripts/verify_rt_runtime.sh` | `rtc_scripts/scripts/verify_rt_runtime.sh` |
| `ur5e_rt_controller/scripts/lib/rt_common.sh` | `rtc_scripts/scripts/lib/rt_common.sh` |
| `ur5e_status_monitor/` | `rtc_status_monitor/` (namespace 변경 + 가변 DOF) |
| `ur5e_hand_udp/.../fingertip_ft_inferencer.hpp` (ONNX 세션/텐서 부분) | `rtc_inference/` (범용 추론 엔진으로 추출) |
| `ur5e_hand_udp/.../fingertip_ft_inferencer.hpp` (전처리/calibration 부분) | `ur5e_hand_driver/` (센서 전용 로직 잔류) |
| `ur5e_hand_udp/` (나머지 전체) | `ur5e_hand_driver/` (리네임) |
| `ur5e_rt_controller/.../controllers/indirect/ur5e_hand_controller` | `ur5e_hand_driver/` (UR5e end-effector 전용) |
| `ur5e_rt_controller/config/controllers/indirect/demo_joint_controller.yaml` | `ur5e_bringup/config/controllers/` |
| `ur5e_rt_controller/config/controllers/indirect/demo_task_controller.yaml` | `ur5e_bringup/config/controllers/` |
| `ur5e_mujoco_sim/` | `rtc_mujoco_sim/` (MJCF 경로 파라미터화) |
| `ur5e_digital_twin/` | `rtc_digital_twin/` (robot_description topic 기반) |
| `ur5e_tools/` | `rtc_tools/` |
| `ur5e_tools/.../plot_ur_log.py` | `rtc_tools/.../plot_rtc_log.py` (가변 DOF, device_log 지원, RTC_SESSION_DIR) |
| `build.sh` | `build.sh` (패키지 목록 + 스크립트 경로 업데이트) |
| `install.sh` | `install.sh` (RT setup 경로 `rtc_scripts/` 기반으로 변경) |

---

## `build.sh` / `install.sh` 업데이트 계획

### `build.sh` 변경사항

**현재:** `ur5e_*` 패키지 이름 하드코딩, `ur5e_rt_controller/scripts/` 경로 참조

**변경 후:**
```bash
# 패키지 목록 업데이트
case "$MODE" in
  robot)
    PACKAGES=(
      rtc_msgs rtc_base rtc_communication rtc_controller_interface
      rtc_controllers rtc_controller_manager rtc_status_monitor
      rtc_inference rtc_scripts
      ur5e_description ur5e_hand_driver ur5e_bringup
      rtc_tools
    )
    ;;
  sim)
    PACKAGES=(
      rtc_msgs rtc_base rtc_communication rtc_controller_interface
      rtc_controllers rtc_controller_manager rtc_status_monitor
      rtc_inference rtc_mujoco_sim rtc_scripts
      ur5e_description ur5e_hand_driver ur5e_bringup
      rtc_tools
    )
    ;;
  full)
    PACKAGES=(
      rtc_msgs rtc_base rtc_communication rtc_controller_interface
      rtc_controllers rtc_controller_manager rtc_status_monitor
      rtc_inference rtc_mujoco_sim rtc_digital_twin rtc_scripts
      ur5e_description ur5e_hand_driver ur5e_bringup
      rtc_tools
    )
    ;;
esac
```

**주요 변경 포인트:**
1. 패키지 이름 `ur5e_*` → `rtc_*` 매핑 업데이트
2. `check_rt_setup.sh` 경로: `ur5e_rt_controller/scripts/` → `rtc_scripts/scripts/`
3. `cpu_shield.sh` 경로: 동일하게 `rtc_scripts/scripts/` 기반으로 변경
4. `compile_commands.json` 경로: `build/ur5e_rt_controller/` → `build/rtc_controller_manager/`
5. Banner/help 텍스트: "UR5e RT Controller" → "RTC (Real-Time Controller)"
6. `auto_release_cpu_shield()`: 스크립트 경로 `rtc_scripts/scripts/cpu_shield.sh`로 변경
7. `rt_common.sh` 경로: `rtc_scripts/scripts/lib/rt_common.sh`로 변경

### `install.sh` 변경사항

**현재:** `ur5e_rt_controller/scripts/lib/rt_common.sh` 참조, UR5e 전용 빌드 모드

**변경 후:**
```bash
# rt_common.sh 경로 변경
_RT_COMMON="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/lib/rt_common.sh"

# 빌드 모드별 패키지 업데이트 (build.sh와 동일)
# RT 설정 스크립트 경로 변경
RT_SCRIPTS_DIR="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts"

# IRQ affinity 설정
bash "${RT_SCRIPTS_DIR}/setup_irq_affinity.sh"

# UDP optimization
bash "${RT_SCRIPTS_DIR}/setup_udp_optimization.sh"

# NVIDIA RT 설정
bash "${RT_SCRIPTS_DIR}/setup_nvidia_rt.sh"

# RT 환경 검증
bash "${RT_SCRIPTS_DIR}/check_rt_setup.sh"
```

**주요 변경 포인트:**
1. `rt_common.sh` 소싱 경로: `ur5e_rt_controller/scripts/lib/` → `rtc_scripts/scripts/lib/`
2. 모든 RT setup 스크립트 경로: `ur5e_rt_controller/scripts/` → `rtc_scripts/scripts/`
3. 빌드 실행 시 `build.sh`에 새 패키지 목록 전달
4. Banner/help 텍스트: "UR5e RT Controller" → "RTC (Real-Time Controller)"
5. `--skip-deps` 동작은 유지 (apt 패키지 설치 스킵)

---

## 장점

1. **로봇 비종속** — 어떤 URDF 로봇이든 `{robot}_bringup` + `{robot}_description`만 추가하면 사용 가능
2. **가변 DOF** — 6축, 7축, 12축 등 어떤 manipulator든 대응
3. **2kHz RT 대응** — configurable sampling time으로 0.5ms까지 지원
4. **관심사 분리** — Interface / Implementation / Manager / Communication 명확 분리
5. **독립 빌드/테스트** — 각 패키지를 독립적으로 빌드/테스트 가능
6. **재사용성** — controller_interface를 기반으로 새 controller 추가 용이
7. **표준 패턴** — ros2_control, franka_ros2, lbr_fri_ros2_stack과 동일한 구조
8. **확장성** — 새 로봇, 새 end-effector, 새 통신 프로토콜 추가 시 기존 코드 수정 최소화
9. **추론 재사용** — `rtc_inference`로 RT-safe ONNX 추론을 어떤 센서/모델에든 재사용 가능 (F/T estimation, contact detection, anomaly detection 등)
10. **통신 프로토콜 확장** — Transport 추상화로 UDP 외에 CAN-FD, EtherCAT, RS485 등을 `TransportInterface` 구현만으로 추가 가능. 기존 Codec/Transceiver 코드 재사용
11. **Controller 계층 명확** — 범용 manipulator controller(any robot)와 로봇+end-effector 통합 demo controller(robot-specific)가 명확히 분리되어, 새 로봇 추가 시 범용 controller는 그대로 재사용하고 통합 controller만 자체 작성
12. **Logging 범용화** — `LogEntry<MaxDOF>` 템플릿으로 6-DOF, 7-DOF, 12-DOF 등 어떤 로봇이든 동일 로깅 인프라 사용. `DataLogger::Config`로 CSV 컬럼 동적 생성. `RTC_SESSION_DIR`로 로봇 비종속 세션 관리
