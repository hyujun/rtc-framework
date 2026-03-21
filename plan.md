# RTC (Real-Time Controller) Repository Restructuring Plan

## 설계 원칙

1. **로봇 비종속(Robot-Agnostic)** — 모든 URDF 로봇에 적용 가능. UR5e 전용 코드는 별도 config/description으로 분리
2. **가변 DOF** — `kNumRobotJoints = 6` 컴파일 타임 상수 제거 → 런타임 URDF 파싱 기반 동적 DOF
3. **0.5ms (2kHz) RT loop 지원** — clock_nanosleep 기반, configurable sampling time (0.5ms ~ 10ms)
4. **관심사 분리** — Interface / Implementation / Manager / Communication 명확 분리
5. **`rtc_` namespace** — 범용 prefix, 로봇별 패키지만 로봇 이름 사용
6. **Inference 분리** — RT-safe 추론 엔진을 독립 패키지로 분리, 센서별 전처리는 드라이버에 잔류

---

## 현재 구조의 문제점

1. **ur5e_rt_controller가 모놀리식** — Controller interface, 6개 controller 구현, RT loop, controller registry, launch files, RT setup scripts가 모두 하나의 패키지에 존재
2. **UR5e 하드코딩** — `kNumRobotJoints = 6` 컴파일 상수, URDF 경로 하드코딩, 기본 joint name이 UR5e 전용
3. **관심사 분리 부족** — controller 정의(interface)와 구현(implementations)과 관리(manager)가 혼재
4. **통신 계층 분산** — UDP 추상화는 ur5e_rt_base에, hand UDP는 별도 패키지, DDS config는 ur5e_rt_controller에 분산
5. **Bringup 분산** — launch 파일이 각 패키지에 흩어져 있고, 통합 launch 로직이 없음
6. **500Hz 고정** — sampling time이 코드에 묶여 있어 더 높은 주파수 대응 불가
7. **Inference 결합** — ONNX Runtime 추론 코드가 `ur5e_hand_udp`에 직접 임베딩. RT-safe 추론 래퍼를 다른 센서/모델에 재사용 불가

---

## 제안하는 새 구조

```
ur5e-rt-controller/                      # (repo 이름은 유지, 내부만 범용화)
│
│  ── 범용 프레임워크 (robot-agnostic) ──
├── rtc_msgs/                            # [리네임] 커스텀 메시지 정의
├── rtc_base/                            # [리네임+축소] RT 인프라 공통 라이브러리
├── rtc_controller_interface/            # [신규] Controller 추상 인터페이스 (가변 DOF)
├── rtc_controllers/                     # [신규] Controller 구현체들 (범용)
├── rtc_controller_manager/              # [신규] RT loop + controller lifecycle
├── rtc_communication/                   # [신규] 통신 계층 (UDP 추상화)
├── rtc_inference/                       # [신규] RT-safe 추론 엔진 (ONNX Runtime)
├── rtc_status_monitor/                  # [리네임] 안전 모니터링 (가변 DOF)
├── rtc_mujoco_sim/                      # [리네임] MuJoCo 시뮬레이션 (범용 URDF)
├── rtc_digital_twin/                    # [리네임] RViz2 시각화 (범용 URDF)
├── rtc_tools/                           # [리네임] 개발 유틸리티
│
│  ── 로봇별 패키지 (robot-specific) ──
├── ur5e_description/                    # [유지] UR5e URDF/MJCF/meshes
├── ur5e_hand_driver/                    # [리네임] Hand 하드웨어 드라이버 (UR5e 전용 end-effector)
├── ur5e_bringup/                        # [신규] UR5e 전용 launch/config/RT scripts
│
│  ── 공통 ──
├── docs/
├── build.sh
├── install.sh
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
│   ├── publish_buffer.hpp
│   └── seqlock.hpp
├── filters/
│   ├── bessel_filter.hpp
│   └── kalman_filter.hpp
└── logging/
    ├── data_logger.hpp
    ├── log_buffer.hpp
    └── session_dir.hpp
```

**핵심 변경:**
- `kNumRobotJoints`, `kDefaultRobotJointNames` 상수 제거
- `RobotModel` 구조체 도입 (런타임 DOF)
- `udp/` 디렉토리 → `rtc_communication`으로 이동
- namespace: `ur5e_rt_base` → `rtc`

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

6개 범용 controller 구현체. UR5e 전용 controller(hand_controller)는 `ur5e_hand_driver`에 잔류.

```
rtc_controllers/
├── include/rtc_controllers/
│   ├── direct/                          # Torque command controllers
│   │   ├── joint_pd_controller.hpp
│   │   └── operational_space_controller.hpp
│   ├── indirect/                        # Position command controllers
│   │   ├── p_controller.hpp
│   │   ├── clik_controller.hpp
│   │   ├── demo_joint_controller.hpp
│   │   └── demo_task_controller.hpp
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
│       ├── clik_controller.cpp
│       ├── demo_joint_controller.cpp
│       └── demo_task_controller.cpp
├── config/controllers/
│   ├── direct/
│   │   ├── joint_pd_controller.yaml
│   │   └── operational_space_controller.yaml
│   └── indirect/
│       ├── p_controller.yaml
│       ├── clik_controller.yaml
│       ├── demo_joint_controller.yaml
│       └── demo_task_controller.yaml
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_controller_interface`, `rtc_base`, `rtc_msgs`, `pinocchio`

**핵심 변경:**
- 모든 controller가 가변 DOF 지원 (고정 크기 배열 → Eigen::VectorXd)
- `ur5e_hand_controller`는 여기서 제외 → `ur5e_hand_driver`에 잔류 (UR5e 전용 end-effector)

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

### 6. `rtc_communication` — 신규 (ur5e_rt_base UDP 이동)

범용 하드웨어 통신 추상화. UDP socket, transceiver, codec만 포함.

```
rtc_communication/
├── include/rtc_communication/
│   ├── udp_socket.hpp
│   ├── udp_transceiver.hpp
│   └── udp_codec.hpp
├── package.xml                          # header-only
└── CMakeLists.txt
```

**의존성:** `rtc_base`

**역할:**
- UDP 통신 추상화 (socket, transceiver, codec)
- 로봇별 프로토콜 구현은 각 로봇 드라이버 패키지에서 담당
- **Hand UDP는 여기에 포함하지 않음** → `ur5e_hand_driver`에서 이 패키지를 depend하여 사용

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

### 11. `ur5e_description` — 유지 (변경 없음)

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

### 14. `ur5e_bringup` — 신규 (UR5e 전용 launch/config 통합)

UR5e에 특화된 launch, config, RT scripts를 통합하는 패키지.

```
ur5e_bringup/
├── launch/
│   ├── robot.launch.py                  # 실물 로봇 launch
│   ├── sim.launch.py                    # MuJoCo 시뮬레이션 launch
│   ├── full.launch.py                   # 로봇 + 디지털 트윈 + 모니터링 통합
│   └── hand.launch.py                   # Hand driver launch
├── config/
│   ├── ur5e_robot.yaml                  # UR5e 전용 파라미터 (URDF 경로, joint names, limits)
│   ├── ur5e_controllers.yaml            # UR5e용 controller 파라미터 오버라이드
│   └── ur5e_rt.yaml                     # UR5e용 RT 설정 (sampling_time, CPU cores)
├── scripts/
│   ├── build_rt_kernel.sh
│   ├── check_rt_setup.sh
│   ├── cpu_shield.sh
│   ├── setup_irq_affinity.sh
│   ├── setup_nvidia_rt.sh
│   ├── setup_udp_optimization.sh
│   ├── verify_rt_runtime.sh
│   └── lib/rt_common.sh
├── package.xml
└── CMakeLists.txt
```

**의존성:** `rtc_controller_manager`, `rtc_mujoco_sim`, `rtc_digital_twin`, `ur5e_hand_driver`, `ur5e_description`

**역할:**
- UR5e 전용 launch (URDF 경로, joint name, sampling_time 등을 argument로 주입)
- 다른 로봇 사용 시 → `{robot_name}_bringup` 패키지를 만들면 됨

---

## 패키지 의존성 그래프

```
rtc_msgs (독립)
rtc_base (독립, header-only)
    │
    ├── rtc_communication ← rtc_base                         [범용 UDP]
    │
    ├── rtc_inference ← rtc_base                             [범용 RT-safe 추론]
    │                    (optional: ONNX Runtime)
    │
    ├── rtc_controller_interface ← rtc_base, rtc_msgs        [범용 인터페이스]
    │       │
    │       └── rtc_controllers ← rtc_controller_interface    [범용 컨트롤러]
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

ur5e_description (독립, 로봇별)
    │
    └── ur5e_bringup ← rtc_controller_manager, rtc_mujoco_sim,
    │                   rtc_digital_twin, ur5e_hand_driver, ur5e_description
    │
    └── ur5e_hand_driver ← rtc_communication, rtc_inference,  [UR5e 전용 드라이버]
                            rtc_base, rtc_msgs, rtc_controller_interface
```

**다른 로봇 추가 시:**
```
panda_description/          # Panda URDF/meshes
panda_bringup/              # Panda launch/config (rtc_controller_manager 사용)
```
→ `rtc_*` 패키지는 재사용, 로봇별 패키지만 추가하면 됨.

---

## 마이그레이션 순서 (단계별)

### Phase 1: 기반 패키지 (의존성 없는 것부터)
1. `rtc_msgs` 생성 — ur5e_msgs 리네임 + namespace 변경
2. `rtc_base` 생성 — ur5e_rt_base 리네임, `kNumRobotJoints` 제거, `RobotModel` 도입, udp/ 분리
3. `rtc_communication` 생성 — ur5e_rt_base에서 udp/ 이동
4. `rtc_inference` 생성 — ur5e_hand_udp에서 ONNX Runtime 래퍼 추출, 범용 추론 엔진 구현

### Phase 2: Controller 패키지 분리
5. `rtc_controller_interface` 생성 — ur5e_rt_controller에서 interface 분리, 가변 DOF 적용
6. `rtc_controllers` 생성 — 6개 controller 구현 이동, 가변 DOF 적용
7. `rtc_status_monitor` 생성 — ur5e_status_monitor 리네임, 가변 DOF 적용

### Phase 3: Manager + 통합
8. `rtc_controller_manager` 생성 — RT loop, node, registry 이동, `sampling_time_us` 파라미터화
9. `rtc_mujoco_sim` 리네임 — MJCF 경로 파라미터화
10. `rtc_digital_twin` 리네임 — robot_description topic 기반
11. `rtc_tools` 리네임

### Phase 4: 로봇별 패키지
12. `ur5e_hand_driver` 생성 — ur5e_hand_udp 리네임 + `rtc_inference` 연동 + ur5e_hand_controller 이동
13. `ur5e_bringup` 생성 — launch/config/scripts 통합

### Phase 5: 정리
14. 기존 `ur5e_rt_controller`, `ur5e_hand_udp`, `ur5e_rt_base`, `ur5e_status_monitor` 패키지 제거
15. build.sh, install.sh 업데이트
16. README.md 업데이트
17. CI/CD 설정 업데이트

---

## 기존 → 새 구조 파일 매핑

| 기존 위치 | 새 위치 |
|-----------|---------|
| `ur5e_msgs/` | `rtc_msgs/` (namespace 변경) |
| `ur5e_rt_base/include/.../types/` | `rtc_base/include/rtc_base/types/` |
| `ur5e_rt_base/include/.../threading/` | `rtc_base/include/rtc_base/threading/` |
| `ur5e_rt_base/include/.../filters/` | `rtc_base/include/rtc_base/filters/` |
| `ur5e_rt_base/include/.../logging/` | `rtc_base/include/rtc_base/logging/` |
| `ur5e_rt_base/include/.../udp/` | `rtc_communication/include/rtc_communication/` |
| `ur5e_rt_controller/.../rt_controller_interface.hpp` | `rtc_controller_interface/include/rtc_controller_interface/` |
| `ur5e_rt_controller/src/rt_controller_interface.cpp` | `rtc_controller_interface/src/` |
| `ur5e_rt_controller/.../controllers/direct/` | `rtc_controllers/include/rtc_controllers/direct/` |
| `ur5e_rt_controller/.../controllers/indirect/` (hand 제외) | `rtc_controllers/include/rtc_controllers/indirect/` |
| `ur5e_rt_controller/src/controllers/` (hand 제외) | `rtc_controllers/src/controllers/` |
| `ur5e_rt_controller/.../trajectory/` | `rtc_controllers/include/rtc_controllers/trajectory/` |
| `ur5e_rt_controller/config/controllers/` | `rtc_controllers/config/controllers/` |
| `ur5e_rt_controller/.../rt_controller_node.hpp` | `rtc_controller_manager/include/rtc_controller_manager/` |
| `ur5e_rt_controller/src/rt_controller_node.cpp` | `rtc_controller_manager/src/` |
| `ur5e_rt_controller/src/rt_controller_main.cpp` | `rtc_controller_manager/src/` |
| `ur5e_rt_controller/.../controller_timing_profiler.hpp` | `rtc_controller_manager/include/rtc_controller_manager/` |
| `ur5e_rt_controller/config/ur5e_rt_controller.yaml` | `ur5e_bringup/config/ur5e_robot.yaml` (UR5e 부분) + `rtc_controller_manager/config/rt_controller_manager.yaml` (범용 부분) |
| `ur5e_rt_controller/config/cyclone_dds.xml` | `rtc_controller_manager/config/` |
| `ur5e_rt_controller/launch/ur_control.launch.py` | `ur5e_bringup/launch/robot.launch.py` |
| `ur5e_rt_controller/scripts/*.sh` | `ur5e_bringup/scripts/` |
| `ur5e_status_monitor/` | `rtc_status_monitor/` (namespace 변경 + 가변 DOF) |
| `ur5e_hand_udp/.../fingertip_ft_inferencer.hpp` (ONNX 세션/텐서 부분) | `rtc_inference/` (범용 추론 엔진으로 추출) |
| `ur5e_hand_udp/.../fingertip_ft_inferencer.hpp` (전처리/calibration 부분) | `ur5e_hand_driver/` (센서 전용 로직 잔류) |
| `ur5e_hand_udp/` (나머지 전체) | `ur5e_hand_driver/` (리네임) |
| `ur5e_rt_controller/.../controllers/indirect/ur5e_hand_controller` | `ur5e_hand_driver/` (UR5e 전용이므로) |
| `ur5e_mujoco_sim/` | `rtc_mujoco_sim/` (MJCF 경로 파라미터화) |
| `ur5e_digital_twin/` | `rtc_digital_twin/` (robot_description topic 기반) |
| `ur5e_tools/` | `rtc_tools/` |

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
