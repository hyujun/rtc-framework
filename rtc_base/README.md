# rtc_base


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **헤더 전용(header-only) 실시간 인프라 라이브러리**입니다. 결정론적 정기 tick 제어 루프 (rate-agnostic; `control_rate` YAML, default 500 Hz, 설계 범위 100 Hz–5 kHz) 에서 안전하게 사용할 수 있도록 설계되었으며, 공유 데이터 타입, 락-프리 동기화 프리미티브, 신호 처리 필터, 스레드 구성, 타이밍 프로파일링, 로깅 인프라를 제공합니다.

**핵심 설계 원칙:**
- 헤더 전용 -- 링크 타임 의존성 없음
- RT 경로에서 힙 할당 및 시스템 콜 금지
- 모든 RT-safe 함수에 `noexcept` 보장
- 캐시 라인 정렬로 false sharing 방지
- C++20 필수 -- `std::concepts`, `std::numbers`, `[[likely]]/[[unlikely]]`, `constexpr` 강화

---

## 패키지 구조

```
rtc_base/
├── CMakeLists.txt
├── package.xml
└── include/rtc_base/
    ├── types/
    │   └── types.hpp              <- 공유 데이터 타입, 상수, 열거형, 캐시 라인 상수
    ├── logging/
    │   ├── thread_csv_producer.hpp  <- ThreadCsvProducer<Payload,N>: payload-owns-row SPSC
    │   ├── thread_csv_logger.hpp    <- ThreadCsvLogger<Payload>: 헤더/행 writer 모두 caller가 결정
    │   └── session_dir.hpp          <- 세션 디렉토리 관리
    ├── filters/
    │   ├── bessel_filter.hpp          <- 4차 Bessel 저역통과 필터
    │   ├── kalman_filter.hpp          <- 이산시간 칼만 필터
    │   ├── sensor_rate_estimator.hpp  <- EMA 기반 센서 샘플링 레이트 추정
    │   └── sliding_trend_detector.hpp <- O(1) 슬라이딩 윈도우 OLS 드리프트 감지
    ├── concurrency/
    │   └── spsc_queue.hpp             <- 락-프리 SPSC 링 버퍼 (POD payload 템플릿)
    ├── timing/
    │   ├── timing_profiler_base.hpp   <- 락-프리 히스토그램 기반 타이밍 프로파일러
    │   ├── thread_timing_sample.hpp   <- ThreadTimingSample<Payload> POD (per-tick 샘플)
    │   ├── thread_timing_producer.hpp <- ThreadTimingProducer<Payload,N> SPSC + tick counter
    │   ├── thread_timing_csv_logger.hpp <- ThreadTimingCsvLogger<Payload> CSV writer
    │   └── rt_tick_timing_sample.hpp  <- RtTickTimingPayload (CM/MPC 공용) + 버퍼 alias
    └── threading/
        ├── thread_config.hpp      <- CPU 코어별 스레드 레이아웃 프리셋
        ├── thread_utils.hpp       <- 스레드 구성/검증 유틸리티
        ├── periodic_rt_thread.hpp <- 고정 주파수 RT 루프 base (CM/MPC 공유)
        ├── publish_buffer.hpp     <- 락-프리 SPSC 퍼블리시 버퍼
        └── seqlock.hpp            <- 락-프리 단일 쓰기/다중 읽기 동기화
```

---

## 모듈 상세

### 타입 (`types/types.hpp`)

프레임워크 전체에서 공유하는 컴파일 타임 상수, 데이터 구조, 열거형을 정의합니다.

#### 주요 상수

> **명명 규약 (capacity vs default)** — 프레임워크의 robot-agnostic 성질을 유지하기 위한 핵심 구분입니다.
>
> - `kNum*` : YAML이 해당 필드를 생략했을 때 사용하는 **기본값**. 실제 채널 수는 런타임에 `DeviceState::num_channels` (또는 유사 필드)로 결정됩니다. 코드는 실제 카운트가 `kNum*` 기본값과 같다고 가정해서는 안 됩니다.
> - `kMax*` : 컴파일 타임 **상한 용량**. `DeviceState`/`ControllerState` 등의 `std::array` 멤버가 trivially copyable (SeqLock 호환) 상태를 유지하고 RT 경로에서 힙 할당을 회피하도록 정해진 크기. 새로운 로봇/디바이스는 이 상한 안에 들어와야 하며, 초과해야 한다면 상한 자체를 올리되 코드에서 분기는 추가하지 마세요.

| 상수 | 값 | 분류 | 설명 |
|------|---|------|------|
| `kCacheLineSize` | 64 | HW const | 캐시 라인 크기 -- 모든 threading 프리미티브에서 공유 |
| `kNumRobotJoints` | 6 | default | YAML 미설정 시 기본 채널 수 (UR5/Franka 등 6/7-DOF 매니퓰레이터 가정). 알고리즘은 런타임 `num_channels` 사용 필수 |
| `kMaxRobotDOF` | 12 | capacity | DOF 상한 (7-DOF 암 + 리던던시까지 커버) |
| `kMaxDeviceChannels` | 64 | capacity | DeviceState 배열 상한 |
| `kMaxSensorChannels` | 128 | capacity | 디바이스 당 센서 채널 상한 |
| `kMaxInferenceValues` | 64 | capacity | ONNX 출력 값 상한 |
| `kTaskSpaceDim` | 6 | HW const | SE(3) DOF — 기하학 상수, 설정 불가 |
| `kMaxFingertips` | 8 | capacity | inference output group 등 grouped sensor block 의 컴파일타임 상한. rtc_* 코드는 의미 없이 std::array 차원 상한으로만 사용 |
| `kDefaultMaxJointVelocity` | 2.0 | 기본 최대 관절 속도 (rad/s) |
| `kDefaultMaxJointTorque` | 150.0 | 기본 최대 관절 토크 (N-m) |

> **참고:** assm_v1 hand 전용 packet/model layout 상수 (`kBarometerCount`, `kReservedCount`, `kTofCount`, `kSensorDataPerPacket`, `kSensorValuesPerFingertip`, `kMaxHandSensors`, `kNumFingertips`, `kNumHandSensors`, `kFTValuesPerFingertip`, `kFTInputSize`, `kFTHistoryLength`, `kDefaultNumFingertips`, `kMaxBaroChannels`, `kMaxTofChannels`)와 `FingertipFTState`, `BesselFilterBaro`/`BesselFilterTof`/`BarometerTrendDetector` alias 들은 `udp_hand_driver/udp_hand_constants.hpp` 로 이주했습니다. `BesselFilter6/11/1` dead alias 는 삭제. rtc_* 패키지는 robot/sensor agnostic — 디바이스별 stride 는 YAML `devices.<name>.sensor_layout` 으로 런타임 주입 (`rtc::DeviceSensorLayout`).

#### C++20 Concepts

| Concept | 설명 |
|---------|------|
| `FloatingPointType` | `std::floating_point<T>` -- 필터 계수, 게인 파라미터 타입 제약 |
| `TriviallyCopyableType` | `std::is_trivially_copyable_v<T>` -- SeqLock/SPSC 버퍼 호환 타입 제약 |

#### 열거형

| 열거형 | 값 | 설명 |
|--------|---|------|
| `CommandType` | `kPosition`, `kTorque` | 커맨드 모드 |
| `GoalType` | `kJoint`, `kTask` | 목표 공간 타입 (uint8_t 기반) |
| `PublishRole` | `kRobotTarget`, `kDigitalTwinState`, `kRobotTransforms` | 퍼블리시 역할. Phase 4 trailing cleanup (`104796f`): `kGraspState`/`kWbcState`/`kToFSnapshot` 도 enum/parser/snapshot 에서 제거되어 각 controller 가 직접 `SeqLock<{Grasp,Wbc,ToF}StateData>` + `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼로 소유. 더 일찍 제거된 항목 (`b9a2587`): `kJointCommand`/`kRos2Command`/`kDeviceStateLog`/`kDeviceSensorLog`/`kGuiPosition` — device-wire 명령은 `devices.<group>.backend`, joint state는 `/rtc_cm/<group>/joint_states`, TCP pose는 `<config_key>/transforms` |
| `DeviceCapability` | `kNone`, `kJointState`, `kMotorState`, `kSensorData`, `kInference` | 디바이스 기능 비트마스크 (RT 루프 선택적 데이터 복사) |

`GoalTypeToString()`, `PublishRoleToString()` -- `constexpr` 문자열 변환 함수.
구독 역할은 Phase 4 trailing cleanup에서 singleton (`target`) 만 남아 enum이 삭제되었고, YAML parser가 `role:` 문자열 (`target` / 호환용 `goal`) 만 validate 한다.

#### 주요 구조체

> **참고:** 레거시 타입 `UdpHandState`는 `udp_hand_driver/udp_hand_state.hpp`로 이동되었습니다. `RobotState`는 사용처가 없어 삭제되었습니다.

**일반화된 디바이스 타입 (가변 DOF):**

| 구조체 | 설명 |
|--------|------|
| `DeviceState` | 가변 채널 디바이스 상태 -- `positions[64]`, `velocities[64]`, `efforts[64]`, 모터 공간 (`motor_positions[64]`, `motor_velocities[64]`, `motor_efforts[64]`), 센서 (`sensor_data[128]`, `sensor_data_raw[128]`), 추론 (`inference_data[64]`, `inference_enable[8]`, `num_inference_fingertips`) |
| `ControllerState` | `devices[4]` (DeviceState 배열) + `num_devices`, `dt`, `iteration`, `t_relative_s` (CM 가 매 tick 채워주는 session-wide 상대 시간; controller 는 `chrono::*::now()` 대신 이 값을 읽어 로그 timestamp 으로 사용) |
| `DeviceOutput` | 가변 채널 출력 -- `commands[64]`, `goal_positions[64]`, `target_positions[64]`, `target_velocities[64]`, `trajectory_positions[64]`, `trajectory_velocities[64]`, `goal_type` |
| `ControllerOutput` | `devices[4]` (DeviceOutput 배열) + `actual_task_positions[6]`, `task_goal_positions[6]`, `trajectory_task_positions[6]`, `trajectory_task_velocities[6]`, `valid`, `command_type`, `grasp_state`, `wbc_state`, `tof_snapshot`, **TF 발행용 SE3** (`arm_tip_pose` + valid, `virtual_tcp_pose` + valid, `fingertip_poses[8]` + valid) |
| `GraspStateData` | 파지 감지 상태 (Force-PI grasp controller 전용) -- `force_magnitude[8]`, `contact_flag[8]`, `inference_valid[8]`, `num_active_contacts`, `max_force`, `grasp_detected`, `force_threshold`, `min_fingertips_for_grasp`, Force-PI 전용: `grasp_phase`, `finger_s[8]`, `finger_filtered_force[8]`, `finger_force_error[8]`, `grasp_target_force` |
| `WbcStateData` | WBC (TSID-based) 컨트롤러 상태 -- `phase` (WbcPhase enum, 0=Idle..7=Fallback), `force_magnitude[8]`, `contact_flag[8]`, `displacement[8]`, `num_active_contacts`, `max_force`, `grasp_target_force`, `grasp_detected`, `min_fingertips_for_grasp`, TSID 진단: `tsid_solve_us`, `tsid_solver_ok`, `qp_fail_count` |
| `Pose` | RT-safe SE3 carrier — `position[3]`, `quaternion[4]` (Hamilton w,x,y,z; identity = (1,0,0,0)). free-standing struct, `ToFSnapshotData::tip_poses[]` 와 `PublishSnapshot::GroupCommandSlot::{arm_tip,virtual_tcp,fingertip}_pose` 가 공유 |
| `ToFSnapshotData` | ToF 센서 스냅샷 -- `distances[24]`, `valid[24]`, `tip_poses[8]` (free-standing `Pose`), `num_fingers`, `sensors_per_finger`, `populated` |

**설정 타입:**

| 구조체 | 설명 |
|--------|------|
| `DeviceUrdfConfig` | URDF 파싱용 -- `package`, `path`, `root_link`, `tip_link` |
| `DeviceJointLimits` | 관절 한계 -- `max_velocity`, `max_acceleration`, `max_torque`, `position_lower`, `position_upper` |
| `DeviceSensorLayout` | 센서 패킹 layout -- `primary_count_per_group`, `secondary_count_per_group`, `values_per_group`, `inference_values_per_group`. rtc_* 코드는 stride/offset 계산에만 사용 (의미는 device-driver 책임) |
| `DeviceNameConfig` | 디바이스 이름, `joint_state_names`, `joint_command_names`, `motor_state_names`, `sensor_names`, URDF 설정(`DeviceUrdfConfig`), 관절 한계(`DeviceJointLimits`), 센서 layout(`DeviceSensorLayout`, optional), 안전 위치(`safe_position`) |
| `TopicConfig` | 디바이스 그룹별 구독/퍼블리시 토픽 라우팅 (`std::vector<std::pair>` 기반, YAML 삽입 순서 보존) |

**토픽 설정 보조 구조체:**

| 구조체 | 설명 |
|--------|------|
| `SubscribeTopicEntry` | `topic_name` + `TopicOwnership` (role enum은 Phase 4 trailing cleanup에서 삭제) |
| `PublishTopicEntry` | `topic_name` + `PublishRole` + `data_size` + `TopicOwnership` |
| `DeviceTopicGroup` | `subscribe` + `publish` 토픽 엔트리 벡터 |

`TopicConfig` 주요 메서드:

| 메서드 | 설명 |
|--------|------|
| `operator[](name)` | 삽입 순서 보존 접근/생성 |
| `HasGroup(name)` | 그룹 존재 및 토픽 보유 여부 확인 (`noexcept`) |
| `HasSubscribeTopic(group)` | 그룹에 구독 엔트리가 1개 이상 존재하는지 확인 (`noexcept`) |
| `GetFirstSubscribeTopic(group)` | 그룹의 첫 구독 토픽 이름 반환 (RT-unsafe, 초기화 시에만 호출) |

> **RT 안전 설계:** 모든 구조체는 zero-initialized, trivially copyable로 설계되어 SeqLock/SPSC 버퍼와 호환됩니다.
> **주의:** `TopicConfig::GetFirstSubscribeTopic()`은 `std::string`을 반환하므로 RT 경로에서 호출 금지 -- 초기화 시에만 사용.

---

### 필터 (`filters/`)

#### Bessel 필터 (`bessel_filter.hpp`)

4차 Bessel 저역통과 필터 -- 최대 평탄 군지연 특성으로 신호 형상을 보존합니다.

- 2개 연속 biquad 섹션 (bilinear transform + 주파수 prewarping, Direct Form II Transposed)
- N개 독립 채널 동시 필터링 (`static_assert(N > 0)` 컴파일 타임 검증)
- RT-safe: `Apply()`, `ApplyScalar()` 함수는 `noexcept`, 할당 없음
- `ComputeBiquad()` -- `constexpr` 지원 (고정 차단 주파수 시 컴파일 타임 계산 가능)

디바이스/센서 종류별 alias 는 사용 패키지 안에서 `BesselFilterN<N>` 로 선언합니다 (예: `udp_hand_driver` 의 `BesselFilterBaro`/`BesselFilterTof`). rtc_base 자체에는 alias 없음.

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(double cutoff_hz, double sample_rate_hz)` | 차단 주파수 + 샘플링 레이트로 계수 계산 (Nyquist 미만 필수) |
| `Apply` | `[[nodiscard]] array<double,N> Apply(const array<double,N>&) noexcept` | N채널 동시 필터링 |
| `ApplyScalar` | `[[nodiscard]] double ApplyScalar(double x, size_t ch) noexcept` | 단일 채널 필터링 |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |

```cpp
rtc::BesselFilterN<6> filter;       // 6채널 (예: 6-DOF 로봇 관절)
filter.Init(30.0, 500.0);           // 30 Hz 차단, 500 Hz 샘플링
auto filtered = filter.Apply(raw_positions);  // RT-safe
```

#### 칼만 필터 (`kalman_filter.hpp`)

이산시간 칼만 필터 -- 등속 운동 모델 기반 위치/속도 추정기입니다.

- 상태: [position, velocity]^T (채널당)
- 관측: 위치만 관측 (H = [1, 0])
- `static_assert(N > 0)` -- 컴파일 타임 채널 수 검증
- RT-safe: `Predict()`, `Update()`, `PredictAndUpdate()` 함수는 `noexcept`, 할당 없음
- 접근자 사전 조건: `position(i)`, `velocity(i)` -- `i < N` 필수 (범위 검사 미수행, 성능 우선)

| 타입 별칭 | N | 용도 |
|-----------|---|------|
| `KalmanFilter6` | 6 | 로봇 관절 |
| `KalmanFilter11` | 11 | 핑거팁 센서 |
| `KalmanFilter1` | 1 | 스칼라 |

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(double q_pos, double q_vel, double r, double dt)` | 노이즈 파라미터 + 시간 간격 설정 |
| `Init` | `void Init(const Params& p)` | Params 구조체 오버로드 |
| `Predict` | `void Predict() noexcept` | 예측 단계만 수행 |
| `Update` | `[[nodiscard]] array<double,N> Update(const array<double,N>&) noexcept` | 갱신 단계만 수행 |
| `UpdateScalar` | `[[nodiscard]] double UpdateScalar(double z, size_t ch) noexcept` | 단일 채널 갱신 |
| `PredictAndUpdate` | `[[nodiscard]] array<double,N> PredictAndUpdate(const array<double,N>&) noexcept` | 예측+갱신 통합 |
| `SetInitialPositions` | `void SetInitialPositions(const array<double,N>&) noexcept` | 초기 위치 설정 |
| `positions` | `[[nodiscard]] array<double,N> positions() const noexcept` | 추정 위치 배열 반환 |
| `velocities` | `[[nodiscard]] array<double,N> velocities() const noexcept` | 추정 속도 배열 반환 |
| `position` | `[[nodiscard]] double position(size_t i) const noexcept` | 채널별 추정 위치 |
| `velocity` | `[[nodiscard]] double velocity(size_t i) const noexcept` | 채널별 추정 속도 |
| `position_variance` | `[[nodiscard]] double position_variance(size_t i) const noexcept` | 위치 분산 (불확실성) |
| `kalman_gain` | `[[nodiscard]] double kalman_gain(size_t i) const noexcept` | 마지막 칼만 게인 |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |

```cpp
KalmanFilter6 kf;
kf.Init(1e-3, 1e-2, 1e-1, 0.002);  // q_pos, q_vel, r, dt
auto filtered = kf.PredictAndUpdate(measurements);  // RT-safe
auto vels = kf.velocities();  // 추정 속도
```

#### 센서 레이트 추정기 (`sensor_rate_estimator.hpp`)

EMA(지수 이동 평균) 기반 실시간 센서 샘플링 레이트 추정기입니다.

- `steady_clock::now()`는 Linux에서 vDSO 기반 -- syscall 없음
- Tick()당 3개 double 연산 (약 5ns 소요)
- 이상치 제거: EMA의 3배를 벗어나는 dt 값 자동 필터링
- 워밍업 단계(기본 50샘플)에서는 빠른 수렴용 alpha=0.1 사용

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(double nominal_rate_hz, double alpha=0.01, int warmup=50) noexcept` | 공칭 레이트 + EMA 파라미터 설정 |
| `Tick` | `void Tick() noexcept` | 센서 사이클마다 1회 호출 (단일 스레드 전용) |
| `rate_hz` | `[[nodiscard]] double rate_hz() const noexcept` | 추정 샘플링 레이트 (Hz) |
| `dt_sec` | `[[nodiscard]] double dt_sec() const noexcept` | 추정 샘플링 주기 (초) |
| `warmed_up` | `[[nodiscard]] bool warmed_up() const noexcept` | 워밍업 완료 여부 |
| `deviation_warning` | `[[nodiscard]] bool deviation_warning(double tol=10.0) const noexcept` | 공칭 레이트 대비 편차 경고 (기본 10%) |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |

```cpp
SensorRateEstimator rate_est;
rate_est.Init(500.0);           // 공칭 500 Hz
rate_est.Tick();                // 센서 사이클마다 호출
double actual = rate_est.rate_hz();  // 추정 레이트
```

#### 슬라이딩 트렌드 감지기 (`sliding_trend_detector.hpp`)

O(1) 슬라이딩 윈도우 OLS(최소자승법) 기반 실시간 드리프트 감지기입니다.

- 틱당 약 10 FLOP/채널 (O(1) 시간 복잡도)
- `std::array` 기반 순환 버퍼 -- 동적 할당 없음
- 워밍업 후 윈도우가 가득 차면 기울기/드리프트 플래그 반환
- 런타임 `sample_rate_hz` 설정 시 기울기를 [단위/초]로 변환

디바이스/센서 종류별 alias 는 사용 패키지 안에서 `SlidingTrendDetector<N, S>` 로 선언합니다 (예: `udp_hand_driver` 의 `BarometerTrendDetector = SlidingTrendDetector<8, 2500>`). rtc_base 자체에는 alias 없음.

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(size_t window_size, double threshold, double rate_hz=0.0) noexcept` | 윈도우 크기, 드리프트 임계값, 샘플레이트 설정 |
| `Update` | `[[nodiscard]] Result Update(const array<double,N>&) noexcept` | 샘플 입력 + 기울기/드리프트 결과 반환 (O(1)) |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |
| `set_drift_threshold` | `void set_drift_threshold(double) noexcept` | 런타임 임계값 변경 |
| `set_sample_rate_hz` | `void set_sample_rate_hz(double) noexcept` | 런타임 샘플레이트 변경 |

`Result` 구조체는 `slopes[N]`, `drift_flags[N]`, `window_full` 필드를 포함합니다.

---

### 타이밍 (`timing/`)

#### 타이밍 프로파일러 (`timing_profiler_base.hpp`)

락-프리, 단일 프로듀서 타이밍 프로파일러 기반 클래스입니다. 히스토그램 기반 백분위수를 제공합니다.

- 실행 통계: count / sum / min / max / last / mean / stddev
- 고정 폭 히스토그램 + 선형 보간 백분위수 (p95, p99)
- 오버 버짓 카운터
- 단일 프로듀서 스레드에서 `UpdateTotal()` 호출, `GetBaseStats()`는 어느 스레드에서나 호출 가능
- 서브클래스에서 히스토그램 범위를 커스터마이징하는 템플릿 파라미터: `Buckets` (기본 20), `BucketWidthUs` (기본 100us), `BudgetUs` (기본 2000us). 서브클래스에서 백분위 해상도가 부족하면 `Buckets` × `BucketWidthUs`를 같게 유지하면서 `BucketWidthUs`를 줄이세요 (예: `ControllerTimingProfiler`는 `<200, 10, 2000>`로 10 µs 해상도, `UdpHandTimingProfiler`는 `<250, 20, 2000>`로 20 µs 해상도)

**정확도 주의사항:**

- **스냅샷 일관성**: `UpdateTotal()`은 histogram 등 모든 필드를 `relaxed`로 갱신한 뒤 마지막에 `count_`를 `release`로 증가시킵니다. `GetBaseStats()`는 `count_`를 `acquire`로 먼저 읽어 `sum(histogram) ≥ count` 불변식을 보장 — `p95/p99`가 `0.0`으로 보고되는 torn-read 블립을 방지합니다.
- **오버플로 버킷 처리**: 샘플이 `kBuckets * kBucketWidthUs` (= `BudgetUs`)를 초과하면 마지막(오버플로) 버킷에 적재됩니다. p95/p99가 이 버킷에 떨어지면 `[bucket_lo, max_us]` 구간에서 선형 보간 — 단순 clip(bucket_lo 반환) 대신 관측된 `max_us`를 상한으로 사용합니다. `max_us`가 오버플로 바닥보다 훨씬 큰 경우 (예: 2000µs vs 5000µs) p99가 정확한 꼬리 위치를 반영합니다.
- **Max clamp**: `InterpolateBucket()`은 결과를 `max_us`로 clamp합니다. 정의상 p95 ≤ max, p99 ≤ max이지만, 선형 보간이 bucket 내 균등 분포를 가정하기 때문에 샘플이 bucket 하단에 클러스터된 경우 (예: 14–60 µs 샘플이 100 µs 폭 bucket에 모인 경우) clamp 없이는 추정값이 max를 초과할 수 있습니다.
- **누적 vs 최근**: 백분위수는 `ResetBase()` 이후 관측된 모든 샘플에 대해 계산됩니다. "최근 N개" 백분위를 원하면 호출자가 주기적으로 `ResetBase()`를 호출해야 합니다 (참고: `rt_controller_node`는 Summary 출력 직후 리셋).

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `UpdateTotal` | `void UpdateTotal(double us) noexcept` | 타이밍 샘플 기록 (단일 프로듀서) |
| `GetBaseStats` | `[[nodiscard]] BaseStats GetBaseStats() const noexcept` | 전체 통계 스냅샷 반환 |
| `LastUs` | `[[nodiscard]] double LastUs() const noexcept` | 마지막 기록 값 |
| `ResetBase` | `void ResetBase() noexcept` | 모든 통계 초기화 |

`BaseStats` 구조체: `count`, `min_us`, `max_us`, `mean_us`, `stddev_us`, `p95_us`, `p99_us`, `last_us`, `over_budget`, `histogram[]`.

`PhaseStats` 구조체: `mean_us`, `min_us`, `max_us` -- 서브클래스에서 단계별 추적에 사용.

---

### 스레딩 (`threading/`)

#### 스레드 구성 (`thread_config.hpp`)

`ThreadConfig` 구조체는 CPU 어피니티, 스케줄러 정책, 우선순위, nice 값, 스레드 이름을 정의합니다.

CPU 코어 수에 따른 스레드 레이아웃 프리셋을 제공합니다 (4, 6, 8, 10, 12, 14, 16코어).

**코어 수별 RT 스레드 레이아웃 (2026-04 unified layout):**

| 스레드 | 4코어 | 6코어 | 8코어 | 10코어 | 12코어 | 14코어 | 16코어 |
|--------|-------|-------|-------|--------|--------|--------|--------|
| **rt_control** (FIFO 90) | Core 1 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 |
| **sensor_io** (FIFO 70) | Core 2 | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 |
| **mpc_main** (FIFO 60) | Core 3¹ | Core 4² | **Core 4** | **Core 4** | **Core 4** | **Core 4** | **Core 9**³ |
| **mpc_worker_0** (FIFO 55) | — | — | — | **Core 5** | **Core 5** | **Core 5** | **Core 10** |
| **mpc_worker_1** (FIFO 55) | — | — | — | — | **Core 6** | **Core 6** | **Core 11** |
| **udp_recv** (FIFO 65) | Core 2 | Core 5 | Core 5 | Core 6 | Core 7 | Core 7 | Core 12 |
| **logger** (OTHER -5) | Core 3 | Core 4 | Core 6 | Core 7 | Core 8 | Core 8 | Core 13 |
| **rt_publish** (OTHER -3) | Core 3 | Core 5 | Core 7 | Core 8 | Core 9 | Core 9 | Core 14 |
| **aux** (OTHER 0) | Core 3 | Core 5 | Core 7 | Core 8 | Core 9 | Core 9 | Core 14 |

> ¹ 4코어는 MPC를 `SCHED_OTHER nice=-5`로 강등 (RT로 돌릴 여유 없음 — 소프트 RT degraded 모드, 10 Hz 권장).
> ² 6코어는 MPC가 logger와 Core 4를 공유하되 FIFO 60으로 logger(OTHER)보다 우선.
> ³ 16코어는 legacy Option A 유지 — RT가 Core 2-3(shield 아래), MPC가 Core 9-11(shield 위), 중간 Core 4-8이 user shield. 10/12/14-core는 unified low-core 배치(RT+MPC 모두 Core 2-9)로 모든 RT thread가 전용 코어 확보.
> **8코어 이상 모든 tier에서 MPC 전용 main 코어 확보**. MPC main 우선순위는 항상 sensor_io(70)보다 낮아 sensor callback이 긴 solve를 preempt.
> 10코어 이상은 worker 1–2개를 추가로 제공하여 Aligator 등 parallel rollout 솔버를 지원.
> **단조성 불변식**: 물리 코어가 증가하면 per-thread 격리는 절대 감소하지 않는다. `test/test_mpc_thread_config.cpp::TierIsolationMonotonicity`가 tier 쌍 전체에 대해 worker 수 / 전용 코어 수 단조 비감소 + 10-core↑에서 `mpc_main`/`udp_recv`/`logger` 코어 분리를 강제.

**MuJoCo 시뮬레이션 코어 레이아웃:**

`GetSimCoreLayout(physical_cores)` -- `constexpr` 함수로 시뮬레이션/뷰어 스레드 코어를 할당합니다.
2026-04 unified layout부터 10/12-core tier도 RT 범위 상단에 여유 코어를 확보했고, 14-core tier는 sim thread 전용 코어를 제공합니다.

| 물리 코어 수 | sim_thread_core | viewer_core | 비고 |
|-------------|----------------|-------------|------|
| 16+ | 15 | -1 (OS) | legacy 배치: MPC(9–11) + IO(12–14) 회피 |
| 14-15 | 10 | -1 | RT/MPC/IO가 Core 2-9 → Core 10 dedicated |
| 12-13 | 10 | -1 | RT/MPC/IO가 Core 2-9 → Core 10-11 spare |
| 10-11 | 9 | -1 | RT/MPC/IO가 Core 2-8 → Core 9 spare |
| 8-9 | -1 (없음) | -1 | sim 모드는 cset shield를 Core 2-3으로 축소, MuJoCo는 해제된 shield 범위에서 CFS 실행 |
| <8 | -1 (없음) | -1 | dedicated sim core 없음 |

#### 스레드 유틸리티 (`thread_utils.hpp`)

| 함수 | 설명 | RT-safe | `[[nodiscard]]` |
|------|------|---------|-----------------|
| `ApplyThreadConfig()` | CPU 어피니티, 스케줄러, 우선순위 설정 | N/A (초기화) | Yes |
| `ApplyThreadConfigWithFallback()` | RT 실패 시 SCHED_OTHER 폴백 | N/A (초기화) | Yes |
| `ValidateThreadConfig()` | 단일 ThreadConfig 유효성 검증 (코어 범위, 정책, 우선순위, 이름) | No | - |
| `ValidateSystemThreadConfigs()` | 전체 시스템 스레드 설정 검증 (동일 코어 동일 RT 우선순위 충돌 감지) | No | - |
| `CheckThreadHealthFast()` | 비트필드 기반 스레드 상태 검증 | **Yes** | - |
| `CheckThreadHealth()` | 문자열 기반 스레드 건강 검사 | No | - |
| `VerifyThreadConfig()` | 현재 스레드 설정 문자열 반환 | No | - |
| `GetPhysicalCpuCount()` | 물리 코어 수 (SMT 제외, sysfs 토폴로지 기반) | No | - |
| `GetOnlineCpuCount()` | 논리 CPU 수 (SMT 포함) | No | - |
| `SelectThreadConfigs()` | 코어 수 기반 레이아웃 자동 선택 (`SystemThreadConfigs` 반환) | No | - |
| `GetThreadStats()` | 레이턴시 기본 통계 (min/max/avg) | No | - |
| `GetThreadMetrics()` | 레이턴시 포괄 통계 (min/max/avg/jitter/p95/p99) | No | - |
| `SafeStrerror()` | 스레드 안전 `strerror_r()` 래퍼 | No | - |

`ThreadHealthFlag` 비트 플래그: `kOk`, `kWrongCore`, `kPolicyChanged`, `kPriorityChanged`, `kNiceChanged`.

`SystemThreadConfigs` 구조체: `rt_control`, `sensor`, `udp_recv`, `logging`, `aux`, `publish`, `mpc` (Phase 5 — `MpcThreadConfig`).

`MpcThreadConfig` 구조체 (Phase 5):
- `main`: MPC solve 스레드의 `ThreadConfig`.
- `num_workers`: 활성 worker 수 (`0 ≤ n ≤ kMpcMaxWorkers == 2`).
- `workers`: `std::array<ThreadConfig, 2>` — 앞쪽 `num_workers`개만 유효.
- `SelectThreadConfigs()`가 물리 코어 수에 맞는 `kMpcConfig{4,6,8,10,12,14,16}Core` 프리셋을 채워 반환 (tier dispatch는 `>=` 계단식이므로 `ncpu >= 14` 분기 유지 필수).
- `ValidateSystemThreadConfigs()`는 Phase 5에서 추가로: (1) MPC main priority < sensor priority, (2) worker priority ≤ main priority, (3) `num_workers ∈ [0, kMpcMaxWorkers]` 세 불변식을 검증.

`ThreadMetrics` 구조체: `min_latency_us`, `max_latency_us`, `avg_latency_us`, `jitter_us`, `percentile_95_us`, `percentile_99_us`.

#### SeqLock (`seqlock.hpp`)

락-프리 단일 쓰기 / 다중 읽기 동기화 프리미티브입니다.

- **Writer**: Wait-free (2개 atomic store + memcpy), 홀수 시퀀스 = 쓰기 진행 중
- **Reader**: Lock-free (contention 시 재시도, torn read 감지)
- 요구사항: `T`는 `std::is_trivially_copyable_v<T>` 만족 필수 (`static_assert`)
- 캐시 라인 정렬 (`kCacheLineSize` -- `types.hpp`에서 통합 정의)

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Store` | `void Store(const T& val) noexcept` | Wait-free 쓰기 (단일 producer) |
| `Load` | `[[nodiscard]] T Load() const noexcept` | Lock-free 읽기 (torn read 시 재시도) |
| `sequence` | `[[nodiscard]] uint32_t sequence() const noexcept` | 현재 시퀀스 번호 조회 |

```cpp
SeqLock<ControllerState> state_lock;
state_lock.Store(new_state);           // RT 스레드 (정기 tick @ control_rate, wait-free)
auto snapshot = state_lock.Load();     // 다른 스레드 (lock-free, 재시도 가능)
```

#### SPSC 버퍼 (`concurrency/spsc_queue.hpp` + 도메인별 alias)

RT 스레드(producer)에서 비-RT 스레드(consumer)로 데이터를 전달하는 락-프리 링 버퍼입니다. 모든 buffer는 `SpscQueue<T, N>` 템플릿의 alias이며, `T` 는 trivially copyable POD 이어야 합니다.

| 버퍼 | 데이터 | 용량 | 용도 |
|------|--------|------|------|
| `ControlPublishBuffer` | `PublishSnapshot` | 512 (~1초 @ default 500 Hz; ratio 1024/control_rate) | ROS2 퍼블리시 오프로드 |
| `CmTimingBuffer` (`ThreadTimingProducer<RtTickTimingPayload, 512>`) | `ThreadTimingSample<RtTickTimingPayload>` | 512 (~1초 @ default 500 Hz) | CM RT loop per-tick timing CSV |
| `MpcTimingBuffer` (`ThreadTimingProducer<RtTickTimingPayload, 128>`) | `ThreadTimingSample<RtTickTimingPayload>` | 128 (~6초 @20Hz) | MPC per-tick timing CSV |
| `HandUdpTimingBuffer` (`ThreadTimingProducer<RtTickTimingPayload, 512>`) | `ThreadTimingSample<RtTickTimingPayload>` | 512 (~1초 @ default 500 Hz) | hand UDP EventLoop per-tick timing CSV |

#### Per-thread timing 인프라 (`timing/thread_timing_*`)

`timing/thread_timing_sample.hpp` + `thread_timing_producer.hpp` + `thread_timing_csv_logger.hpp` 3개 헤더가 임의 RT/soft-RT thread에 대한 per-tick CSV 로깅을 일반화한다. 새 timing 채널을 추가하려면:

1. `Payload` POD struct 정의 (trivially copyable).
2. header writer (`",col1,col2"`) + row writer (`",<v1>,<v2>"`) 자유 함수.
3. `using MyBuffer = ThreadTimingProducer<Payload, N>;` alias.

CM, MPC, udp_hand_driver의 hand UDP EventLoop가 이 패턴의 세 사용처. 향후 ONNX inference 등 새 thread는 동일 인프라를 재사용한다 — `RTControllerInterface` 변경 없음.

**공통 API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Push` | `[[nodiscard]] bool Push(const T& entry) noexcept` | RT-safe, wait-free (가득 차면 `false` + `drop_count_++`) |
| `Pop` | `[[nodiscard]] bool Pop(T& out) noexcept` | 비-RT consumer (비었으면 `false`) |
| `drop_count` | `[[nodiscard]] uint64_t drop_count() const noexcept` | 드롭된 항목 수 |

- 용량 N은 반드시 2의 거듭제곱 (`static_assert`)
- 캐시 라인 정렬로 producer/consumer 인덱스 false sharing 방지
- `memory_order_acquire/release`로 동기화
- `[[unlikely]]` 분기 힌트로 hot path 최적화 (버퍼 풀 경로)
- cached head/tail 인덱스로 atomic 연산 최소화

#### PublishSnapshot 주요 필드

| 카테고리 | 필드 |
|---------|------|
| 디바이스별 (`GroupCommandSlot[4]`) | `num_channels`, `actual_num_channels`, `commands[]`, `goal_positions[]`, `target_positions[]`, `target_velocities[]`, `trajectory_positions[]`, `trajectory_velocities[]`, `actual_positions[]`, `actual_velocities[]`, `efforts[]`, `motor_positions[]`, `motor_velocities[]`, `motor_efforts[]`, `sensor_data[]`, `sensor_data_raw[]`, `inference_output[]`, `goal_type`, **TF source poses** (`arm_tip_pose` + valid, `virtual_tcp_pose` + valid, `fingertip_poses[8]` + valid — `kRobotTransforms` publish role 용). `grasp_state`/`wbc_state`/`tof_snapshot` 은 owned-topic isolation (`104796f`) 으로 슬롯에서 빠지고 controller 별 `SeqLock<{Grasp,Wbc,ToF}StateData>` 로 이관됨 — `ControllerOutput` 에는 잔존 (RT 루프가 controller 내부 SeqLock writer 로 직접 push). |
| 공유 | `command_type`, `actual_task_positions[6]`, `stamp_ns`, `active_controller_idx`, `num_groups` |

---

### 로깅 (`logging/`)

> **Phase C 정리 (2026-05-01)**: 기존 CM-side `data_logger.hpp` / `log_buffer.hpp` 는 삭제됐다. 컨트롤러 데이터 CSV 는 `rtc_controller_interface/controller_log_set.hpp` 의 `ControllerLogSet` + 각 컨트롤러가 소유한 POD 미러 (예: `integrated_bringup/include/integrated_bringup/logging/`) 로 이전. CM 은 `cm_timing_log.csv` (per-tick scheduling timing) 만 자체 소유한다. `cm_timing_log` 의 schema 는 `t_wall_ns, tick_count, t_state_us, t_compute_us, t_publish_us, t_total_us, jitter_us` — MPC / hand_udp 와 동일한 7-col `RtTickTimingPayload` 사용 (`rtc_base/timing/rt_tick_timing_sample.hpp`).
>
> **Sim 모드 (2026-05-02)**: `PeriodicRtThread::JitterMeaningful()` 가상 함수로 producer가 자기 wakeup이 deadline-driven 인지 선언한다. CM `ControlLoopThread` 는 `use_sim_time_sync=true` 일 때 `false` 반환 → base 가 `jitter_us` 를 0.0 으로 둔다 (CV cadence 대비 budget 차이는 RT 잡음 지표가 아니므로). MPC / hand_udp 는 default `true` 유지 → 기존 동작 그대로. 다른 6 개 컬럼은 두 모드 동일.

#### Generic CSV 인프라 (`thread_csv_producer.hpp` + `thread_csv_logger.hpp`)

Controller-owned 데이터 CSV 를 위한 generic 페어. `timing/thread_timing_*` 와 의도적으로 분리되어 있다 — 차이점:

| 항목 | `timing/thread_timing_*` | `logging/thread_csv_*` |
|------|-------------------------|------------------------|
| 자동 컬럼 | `t_wall_ns, tick_count` 두 개 emit | 없음 — payload 가 행 전체 소유 |
| 용도 | per-tick timing CSV (CM/MPC/hand_udp) | controller-owned data CSV (state/sensor/inference 등) |
| Schema 결정 | `WriteRtTickTimingHeader/Row` 단일 schema | caller 가 매 payload 마다 자유롭게 결정 |
| Timestamp | `t_wall_ns` (steady_clock) | `state.t_relative_s` 를 payload 안에 embed |

API:

| 클래스 | 메서드 | 설명 |
|--------|--------|------|
| `ThreadCsvProducer<Payload, N>` | `Push(payload)` | RT thread 에서 SPSC 에 push (wait-free, drop-on-full) |
| | `Drain(fn)` | 비-RT consumer 에서 FIFO 드레인 |
| | `DropCount()` | 누적 drop count |
| `ThreadCsvLogger<Payload>` | `Open(path, hdr, row)` | 헤더/행 writer 와 함께 파일 열기 (append-on-reopen) |
| | `Log(payload)` | row writer 호출 + flush |

전형적 사용 패턴 (`ControllerLogSet` 가 묶어 제공할 예정):

```cpp
struct StateLogPod {
  double t_relative_s{0.0};
  std::array<double, 16> actual_positions{};
  // … (rest mirrors rtc_msgs/DeviceStateLog)
};
static_assert(std::is_trivially_copyable_v<StateLogPod>);

void WriteHeader(std::ostream &os) { os << "t_relative_s,actual_pos_0,…"; }
void WriteRow(std::ostream &os, const StateLogPod &p) {
  os << p.t_relative_s << ',' << p.actual_positions[0] /*…*/;
}

rtc::ThreadCsvProducer<StateLogPod, 512> producer;
rtc::ThreadCsvLogger<StateLogPod> logger;
logger.Open(path, &WriteHeader, &WriteRow);

// RT tick:
producer.Push(pod);

// non-RT drain timer:
producer.Drain([&](const StateLogPod &p) { logger.Log(p); });
```

#### 세션 디렉토리 (`session_dir.hpp`)

세션 기반 로깅 디렉토리 관리 유틸리티입니다.

세션 디렉토리 구조:
```
logging_data/YYMMDD_HHMM/
  controller/                -- rtc_controller_manager 의 RT 루프 CSV (state_log, sensor_log; Phase C에서 정리됨)
  controllers/<config_key>/  -- 개별 controller LifecycleNode 가 자체 기록하는 데이터 CSV
  timing/                    -- per-tick 스레드 타이밍 CSV (cm_timing_log,
                                mpc_timing_log, hand_udp_timing_log)
  device/                    -- device 통신 통계
  sim/                       -- mujoco 스크린샷
  plots/                     -- 플롯 출력
  motions/                   -- 모션 에디터 JSON
  perf/                      -- Linux perf record 출력 (perf.data; Hotspot 으로 뷰)
```

| 함수 | 설명 |
|------|------|
| `GenerateSessionTimestamp()` | `YYMMDD_HHMM` 형식 타임스탬프 생성 |
| `ResolveLoggingRoot()` | 3단 체인으로 `logging_data` 루트 경로 결정 (아래 참고) |
| `ResolveSessionDir()` | env → `ResolveLoggingRoot()` → `YYMMDD_HHMM` 세션 디렉토리 생성 |
| `EnsureSessionSubdirs(session_dir)` | controller, timing, monitor, device, sim, plots, motions, perf 하위 폴더 생성 |
| `TimingDir(session_dir)` | per-tick timing CSV 들이 모이는 `<session>/timing/` 경로 반환 |
| `ListSessionDirs(logging_root)` | `YYMMDD_HHMM` 패턴 디렉토리 정렬 목록 반환 |
| `CleanupOldSessions(logging_root, max)` | 최대 세션 수 초과 시 오래된 세션 삭제 |

**`ResolveLoggingRoot()` 결정 체인** (위에서 아래로):

1. `$COLCON_PREFIX_PATH` 의 첫 entry 가 쓰기 가능한 디렉토리이면 그 `parent / "logging_data"`
2. cwd 에서 상위로 올라가며 `install/` + `src/` 쌍이 발견되면 그 디렉토리 `/ "logging_data"`
3. 최종 폴백: `cwd / "logging_data"`

**`ResolveSessionDir()` 결정 체인**:

1. `$RTC_SESSION_DIR`
2. `ResolveLoggingRoot() / YYMMDD_HHMM` 을 새로 생성

> **주의:** `ResolveSessionDir()`은 `std::filesystem::filesystem_error`를 throw할 수 있습니다. 초기화 시에만 호출하세요.
> Python 측 동일 로직은 `rtc_tools.utils.session_dir.resolve_logging_root()` 참고.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |

> **외부 의존성 없음** -- 표준 C++ 라이브러리와 POSIX API만 사용합니다.

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_base
source install/setup.bash
```

헤더 전용 라이브러리이므로 컴파일되는 바이너리는 없습니다. 소비자 패키지에서 `#include`하여 사용합니다.

**컴파일러 요구사항:** C++20 호환 컴파일러 (GCC 10+, Clang 13+).

**컴파일러 경고 플래그:** `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`

> Build hygiene: `thread_utils.hpp`의 `fscanf` 호출은 반환값을 검사하여 `-Wunused-result`를 제거했습니다. 실패 시 sentinel value (`quota=-1`, `period=0`)가 그대로 남아 다음 fallback stage로 진행 — behavior 동일.

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
