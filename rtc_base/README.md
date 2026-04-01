# rtc_base

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **헤더 전용(header-only) 실시간 인프라 라이브러리**입니다. 500 Hz 결정론적 제어 루프에서 안전하게 사용할 수 있도록 설계되었으며, 공유 데이터 타입, 락-프리 동기화 프리미티브, 신호 처리 필터, 스레드 구성, 타이밍 프로파일링, 로깅 인프라를 제공합니다.

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
    │   ├── log_buffer.hpp         <- 락-프리 SPSC 로그 링 버퍼
    │   ├── data_logger.hpp        <- CSV 파일 로거
    │   └── session_dir.hpp        <- 세션 디렉토리 관리
    ├── filters/
    │   ├── bessel_filter.hpp          <- 4차 Bessel 저역통과 필터
    │   ├── kalman_filter.hpp          <- 이산시간 칼만 필터
    │   ├── sensor_rate_estimator.hpp  <- EMA 기반 센서 샘플링 레이트 추정
    │   └── sliding_trend_detector.hpp <- O(1) 슬라이딩 윈도우 OLS 드리프트 감지
    ├── timing/
    │   └── timing_profiler_base.hpp   <- 락-프리 히스토그램 기반 타이밍 프로파일러
    └── threading/
        ├── thread_config.hpp      <- CPU 코어별 스레드 레이아웃 프리셋
        ├── thread_utils.hpp       <- 스레드 구성/검증 유틸리티
        ├── publish_buffer.hpp     <- 락-프리 SPSC 퍼블리시 버퍼
        └── seqlock.hpp            <- 락-프리 단일 쓰기/다중 읽기 동기화
```

---

## 모듈 상세

### 타입 (`types/types.hpp`)

프레임워크 전체에서 공유하는 컴파일 타임 상수, 데이터 구조, 열거형을 정의합니다.

#### 주요 상수

| 상수 | 값 | 설명 |
|------|---|------|
| `kCacheLineSize` | 64 (또는 HW 값) | 캐시 라인 크기 -- 모든 threading 프리미티브에서 공유 |
| `kNumRobotJoints` | 6 | UR5e 기본 관절 수 |
| `kMaxRobotDOF` | 12 | 최대 로봇 자유도 |
| `kMaxDeviceChannels` | 64 | 일반화 디바이스 최대 채널 수 |
| `kMaxSensorChannels` | 128 | 최대 센서 데이터 채널 수 |
| `kMaxInferenceValues` | 64 | 최대 추론 출력 크기 |
| `kNumHandMotors` | 10 | 핸드 모터 수 |
| `kNumHandJoints` | 10 | 레거시 별칭 (`kNumHandMotors`와 동일) |
| `kDefaultNumFingertips` | 4 | 기본 핑거팁 수 (YAML 미설정 시) |
| `kMaxFingertips` | 8 | 최대 핑거팁 수 (배열 상한) |
| `kBarometerCount` | 8 | 핑거팁 당 기압 센서 수 |
| `kReservedCount` | 5 | 패킷 내 예약 필드 (저장하지 않음) |
| `kTofCount` | 3 | 핑거팁 당 ToF 센서 수 |
| `kSensorDataPerPacket` | 16 | 패킷 당 센서 데이터 수 (baro 8 + reserved 5 + ToF 3) |
| `kSensorValuesPerFingertip` | 11 | 핑거팁 당 유효 센서 값 (baro 8 + ToF 3) |
| `kMaxHandSensors` | 88 | 최대 핸드 센서 채널 수 (`kMaxFingertips * kSensorValuesPerFingertip`) |
| `kNumFingertips` | 4 | 기본값 기반 상수 (하위 호환) |
| `kNumHandSensors` | 44 | 기본 핑거팁 수 기반 센서 수 |
| `kFTValuesPerFingertip` | 7 | 핑거팁 당 F/T 추론 출력 수 (contact(1)+F(3)+u(3)) |
| `kFTInputSize` | 16 | F/T 추론 입력 크기 (baro(8) + delta(8)) |
| `kFTHistoryLength` | 12 | F/T 추론 FIFO 히스토리 길이 |
| `kDefaultMaxJointVelocity` | 2.0 | 기본 최대 관절 속도 (rad/s) |
| `kDefaultMaxJointTorque` | 150.0 | 기본 최대 관절 토크 (N-m) |

#### 기본 이름 벡터

| 변수 | 값 |
|------|---|
| `kDefaultRobotJointNames` | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| `kDefaultHandMotorNames` | `thumb_cmc_aa`, `thumb_cmc_fe`, `thumb_mcp_fe`, `index_mcp_aa`, `index_mcp_fe`, `index_dip_fe`, `middle_mcp_aa`, `middle_mcp_fe`, `middle_dip_fe`, `ring_mcp_fe` |
| `kDefaultFingertipNames` | `thumb`, `index`, `middle`, `ring` |

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
| `SubscribeRole` | `kState`, `kMotorState`, `kSensorState`, `kTarget` | 구독 역할 |
| `PublishRole` | `kJointCommand`, `kRos2Command`, `kGuiPosition`, `kRobotTarget`, `kDeviceStateLog`, `kDeviceSensorLog`, `kDigitalTwinState`, `kGraspState` | 퍼블리시 역할 |

`GoalTypeToString()`, `SubscribeRoleToString()`, `PublishRoleToString()` -- `constexpr` 문자열 변환 함수.

#### 주요 구조체

**레거시 타입 (UR5e 전용, 향후 제거 예정):**

| 구조체 | 설명 |
|--------|------|
| `RobotState` | 6-DOF 관절 위치/속도/토크, TCP 위치(3), dt, 반복 카운터 |
| `HandState` | 모터 위치/속도/전류, 관절 위치/속도/전류, 센서 데이터 (raw + filtered), 핑거팁 수, 유효성, `received_joint_mode` (0x00=motor, 0x01=joint) |

**일반화된 디바이스 타입 (가변 DOF):**

| 구조체 | 설명 |
|--------|------|
| `DeviceState` | 가변 채널 디바이스 상태 -- `positions[64]`, `velocities[64]`, `efforts[64]`, 모터 공간 (`motor_positions[64]`, `motor_velocities[64]`, `motor_efforts[64]`), 센서 (`sensor_data[128]`, `sensor_data_raw[128]`), 추론 (`inference_data[64]`, `inference_enable[8]`, `num_inference_fingertips`) |
| `ControllerState` | `devices[4]` (DeviceState 배열) + `num_devices`, `dt`, `iteration` |
| `DeviceOutput` | 가변 채널 출력 -- `commands[64]`, `goal_positions[64]`, `target_positions[64]`, `target_velocities[64]`, `trajectory_positions[64]`, `trajectory_velocities[64]`, `goal_type` |
| `ControllerOutput` | `devices[4]` (DeviceOutput 배열) + `actual_task_positions[6]`, `task_goal_positions[6]`, `trajectory_task_positions[6]`, `trajectory_task_velocities[6]`, `valid`, `command_type`, `grasp_state` |
| `FingertipFTState` | 핑거팁 힘/토크 추론 결과 -- `ft_data[56]` (`kFTValuesPerFingertip * kMaxFingertips`), `per_fingertip_valid[8]`, `num_fingertips` |
| `GraspStateData` | 파지 감지 상태 -- `force_magnitude[8]`, `contact_flag[8]`, `inference_valid[8]`, `num_active_contacts`, `max_force`, `grasp_detected`, `force_threshold`, `min_fingertips_for_grasp` |

**설정 타입:**

| 구조체 | 설명 |
|--------|------|
| `DeviceUrdfConfig` | URDF 파싱용 -- `package`, `path`, `root_link`, `tip_link` |
| `DeviceJointLimits` | 관절 한계 -- `max_velocity`, `max_acceleration`, `max_torque`, `position_lower`, `position_upper` |
| `DeviceNameConfig` | 디바이스 이름, `joint_state_names`, `joint_command_names`, `motor_state_names`, `sensor_names`, URDF 설정(`DeviceUrdfConfig`), 관절 한계(`DeviceJointLimits`), 안전 위치(`safe_position`) |
| `TopicConfig` | 디바이스 그룹별 구독/퍼블리시 토픽 라우팅 (`std::vector<std::pair>` 기반, YAML 삽입 순서 보존) |

**토픽 설정 보조 구조체:**

| 구조체 | 설명 |
|--------|------|
| `SubscribeTopicEntry` | `topic_name` + `SubscribeRole` |
| `PublishTopicEntry` | `topic_name` + `PublishRole` + `data_size` |
| `DeviceTopicGroup` | `subscribe` + `publish` 토픽 엔트리 벡터 |

`TopicConfig` 주요 메서드:

| 메서드 | 설명 |
|--------|------|
| `operator[](name)` | 삽입 순서 보존 접근/생성 |
| `HasGroup(name)` | 그룹 존재 및 토픽 보유 여부 확인 (`noexcept`) |
| `HasSubscribeRole(group, role)` | 특정 구독 역할 보유 여부 확인 (`noexcept`) |
| `GetSubscribeTopicName(group, role)` | 구독 토픽 이름 반환 (RT-unsafe, 초기화 시에만 호출) |

> **RT 안전 설계:** 모든 구조체는 zero-initialized, trivially copyable로 설계되어 SeqLock/SPSC 버퍼와 호환됩니다.
> **주의:** `TopicConfig::GetSubscribeTopicName()`은 `std::string`을 반환하므로 RT 경로에서 호출 금지 -- 초기화 시에만 사용.

---

### 필터 (`filters/`)

#### Bessel 필터 (`bessel_filter.hpp`)

4차 Bessel 저역통과 필터 -- 최대 평탄 군지연 특성으로 신호 형상을 보존합니다.

- 2개 연속 biquad 섹션 (bilinear transform + 주파수 prewarping, Direct Form II Transposed)
- N개 독립 채널 동시 필터링 (`static_assert(N > 0)` 컴파일 타임 검증)
- RT-safe: `Apply()`, `ApplyScalar()` 함수는 `noexcept`, 할당 없음
- `ComputeBiquad()` -- `constexpr` 지원 (고정 차단 주파수 시 컴파일 타임 계산 가능)

| 타입 별칭 | N | 용도 |
|-----------|---|------|
| `BesselFilter6` | 6 | 로봇 관절 |
| `BesselFilter11` | 11 | 핑거팁 센서 |
| `BesselFilterBaro` | 64 | 기압 센서 (`kMaxFingertips * kBarometerCount`) |
| `BesselFilterTof` | 24 | ToF 센서 (`kMaxFingertips * kTofCount`) |
| `BesselFilter1` | 1 | 스칼라 |

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(double cutoff_hz, double sample_rate_hz)` | 차단 주파수 + 샘플링 레이트로 계수 계산 (Nyquist 미만 필수) |
| `Apply` | `[[nodiscard]] array<double,N> Apply(const array<double,N>&) noexcept` | N채널 동시 필터링 |
| `ApplyScalar` | `[[nodiscard]] double ApplyScalar(double x, size_t ch) noexcept` | 단일 채널 필터링 |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |

```cpp
BesselFilter6 filter;
filter.Init(30.0, 500.0);  // 30 Hz 차단, 500 Hz 샘플링
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

| 타입 별칭 | NumChannels | MaxWindowSize | 용도 |
|-----------|------------|--------------|------|
| `BarometerTrendDetector` | 8 (`kBarometerCount`) | 2500 | 핑거팁 기압 센서 드리프트 감지 (5초 @500Hz) |

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
- 서브클래스에서 히스토그램 범위를 커스터마이징하는 템플릿 파라미터: `Buckets` (기본 20), `BucketWidthUs` (기본 100us), `BudgetUs` (기본 2000us)

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

CPU 코어 수에 따른 스레드 레이아웃 프리셋을 제공합니다 (4, 6, 8, 10, 12, 16코어).

**코어 수별 RT 스레드 레이아웃:**

| 스레드 | 4코어 | 6코어 | 8코어 | 10코어 | 12코어 | 16코어 |
|--------|-------|-------|-------|--------|--------|--------|
| **rt_control** (FIFO 90) | Core 1 | Core 2 | Core 2 | Core 7 | Core 7 | Core 2 |
| **sensor_io** (FIFO 70) | Core 2 | Core 3 | Core 3 | Core 8 | Core 8 | Core 3 |
| **udp_recv** (FIFO 65) | Core 2 | Core 5 | Core 4 | Core 9 | Core 9 | Core 9 |
| **logger** (OTHER -5) | Core 3 | Core 4 | Core 5 | Core 9 | Core 10 | Core 10 |
| **rt_publish** (OTHER -3) | Core 3 | Core 5 | Core 6 | Core 9 | Core 11 | Core 11 |
| **aux** (OTHER 0) | Core 3 | Core 5 | Core 6 | Core 9 | Core 11 | Core 11 |

**MuJoCo 시뮬레이션 코어 레이아웃:**

`GetSimCoreLayout(physical_cores)` -- `constexpr` 함수로 시뮬레이션/뷰어 스레드 코어를 할당합니다.

| 물리 코어 수 | sim_thread_core | viewer_core |
|-------------|----------------|-------------|
| 16+ | 12 | 13 |
| 12+ | 11 | -1 (OS) |
| 10+ | 9 | -1 (OS) |
| 8+ | 7 | -1 (OS) |
| <8 | -1 (없음) | -1 (없음) |

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

`SystemThreadConfigs` 구조체: `rt_control`, `sensor`, `udp_recv`, `logging`, `aux`, `publish`.

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
state_lock.Store(new_state);           // RT 스레드 (500 Hz, wait-free)
auto snapshot = state_lock.Load();     // 다른 스레드 (lock-free, 재시도 가능)
```

#### SPSC 버퍼 (`log_buffer.hpp`, `publish_buffer.hpp`)

RT 스레드(producer)에서 비-RT 스레드(consumer)로 데이터를 전달하는 락-프리 링 버퍼입니다.

| 버퍼 | 데이터 | 용량 | 용도 |
|------|--------|------|------|
| `ControlLogBuffer` | `LogEntry` | 512 (~1초 @500Hz) | CSV 로깅 |
| `ControlPublishBuffer` | `PublishSnapshot` | 512 (~1초 @500Hz) | ROS2 퍼블리시 오프로드 |

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

#### LogEntry 주요 필드

| 카테고리 | 필드 |
|---------|------|
| 타이밍 | `timestamp`, `t_state_acquire_us`, `t_compute_us`, `t_publish_us`, `t_total_us`, `jitter_us` |
| 공유 | `actual_task_positions[6]`, `task_goal_positions[6]`, `trajectory_task_positions[6]`, `trajectory_task_velocities[6]`, `command_type` |
| 디바이스별 (`DeviceLogSlot[4]`) | `num_channels`, `goal_positions[]`, `actual_positions[]`, `actual_velocities[]`, `efforts[]`, `commands[]`, `trajectory_positions[]`, `trajectory_velocities[]`, `motor_positions[]`, `motor_velocities[]`, `motor_efforts[]`, `sensor_data[128]`, `sensor_data_raw[128]`, `valid`, `goal_type` |
| 추론 | `inference_output[64]`, `inference_valid`, `num_inference_values` |

#### PublishSnapshot 주요 필드

| 카테고리 | 필드 |
|---------|------|
| 디바이스별 (`GroupCommandSlot[4]`) | `num_channels`, `actual_num_channels`, `commands[]`, `goal_positions[]`, `target_positions[]`, `target_velocities[]`, `trajectory_positions[]`, `trajectory_velocities[]`, `actual_positions[]`, `actual_velocities[]`, `efforts[]`, `motor_positions[]`, `motor_velocities[]`, `motor_efforts[]`, `sensor_data[]`, `sensor_data_raw[]`, `inference_output[]`, `goal_type`, `grasp_state` |
| 공유 | `command_type`, `actual_task_positions[6]`, `task_goals[4][6]`, `stamp_ns`, `active_controller_idx`, `num_groups` |

---

### 로깅 (`logging/`)

#### 데이터 로거 (`data_logger.hpp`)

역할 기반 CSV 파일에 제어 루프 데이터를 기록합니다. `DeviceLogConfig` 구조체로 디바이스별 로그 경로, 역할, 관절/모터/센서 이름, 채널 수를 지정합니다.

**CSV 파일 역할:**

| CSV 파일 | 역할 | 헤더 컬럼 |
|----------|------|----------|
| `timing_log.csv` | (항상 생성) | `timestamp, t_state_acquire_us, t_compute_us, t_publish_us, t_total_us, jitter_us` |
| `{device}_state_log.csv` | `kDeviceStateLog` | `timestamp, actual_pos_*, actual_vel_*, effort_*, command_*, command_type, goal_type, joint_goal_*, task_goal_*, traj_pos_*, traj_vel_*, traj_task_pos_*, traj_task_vel_*, task_pos_*, motor_pos_*, motor_vel_*, motor_eff_*` |
| `{device}_sensor_log.csv` | `kDeviceSensorLog` | `timestamp, baro_raw_*, tof_raw_*, baro_*, tof_*, inference_valid, ft_*_contact/fx/fy/fz/ux/uy/uz` |

**API:**

| 메서드 | 설명 |
|--------|------|
| `DataLogger(timing_path, device_configs, num_inference_values)` | CSV 파일 열기 + 헤더 작성 |
| `LogEntry(const LogEntry&)` | 단일 항목 기록 |
| `DrainBuffer(ControlLogBuffer&)` | SPSC 버퍼 전체 드레인 -> CSV 기록 |
| `Flush()` | 파일 버퍼 플러시 |
| `IsOpen()` | 파일 열림 상태 확인 |

#### 세션 디렉토리 (`session_dir.hpp`)

세션 기반 로깅 디렉토리 관리 유틸리티입니다.

세션 디렉토리 구조:
```
logging_data/YYMMDD_HHMM/
  controller/   -- rt_controller CSV 로그
  device/       -- device 통신 통계
  sim/          -- mujoco 스크린샷
  plots/        -- 플롯 출력
  motions/      -- 모션 에디터 JSON
```

| 함수 | 설명 |
|------|------|
| `GenerateSessionTimestamp()` | `YYMMDD_HHMM` 형식 타임스탬프 생성 |
| `ResolveSessionDir(fallback_root)` | `RTC_SESSION_DIR` -> `UR5E_SESSION_DIR` -> 자동 생성 (`~` 확장 지원) |
| `EnsureSessionSubdirs(session_dir)` | controller, monitor, device, sim, plots, motions 하위 폴더 생성 |
| `ListSessionDirs(logging_root)` | `YYMMDD_HHMM` 패턴 디렉토리 정렬 목록 반환 |
| `CleanupOldSessions(logging_root, max)` | 최대 세션 수 초과 시 오래된 세션 삭제 |

> **주의:** `ResolveSessionDir()`은 `std::filesystem::filesystem_error`를 throw할 수 있습니다. 초기화 시에만 호출하세요.

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

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
