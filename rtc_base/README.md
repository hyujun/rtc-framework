# rtc_base

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **헤더 전용(header-only) 실시간 인프라 라이브러리**입니다. 500 Hz 결정론적 제어 루프에서 안전하게 사용할 수 있도록 설계되었으며, 공유 데이터 타입, 락-프리 동기화 프리미티브, 신호 처리 필터, 스레드 구성, 로깅 인프라를 제공합니다.

**핵심 설계 원칙:**
- 헤더 전용 — 링크 타임 의존성 없음
- RT 경로에서 힙 할당 및 시스템 콜 금지
- 모든 RT-safe 함수에 `noexcept` 보장
- 캐시 라인 정렬로 false sharing 방지
- C++20 필수 — `std::concepts`, `std::numbers`, `[[likely]]/[[unlikely]]`, `constexpr` 강화

---

## 패키지 구조

```
rtc_base/
├── CMakeLists.txt
├── package.xml
└── include/rtc_base/
    ├── types/
    │   └── types.hpp              ← 공유 데이터 타입, 상수, 열거형, 캐시 라인 상수
    ├── logging/
    │   ├── log_buffer.hpp         ← 락-프리 SPSC 로그 링 버퍼
    │   ├── data_logger.hpp        ← CSV 파일 로거
    │   └── session_dir.hpp        ← 세션 디렉토리 관리
    ├── filters/
    │   ├── bessel_filter.hpp          ← 4차 Bessel 저역통과 필터
    │   ├── kalman_filter.hpp          ← 이산시간 칼만 필터
    │   ├── sensor_rate_estimator.hpp  ← EMA 기반 센서 샘플링 레이트 추정
    │   └── sliding_trend_detector.hpp ← O(1) 슬라이딩 윈도우 OLS 드리프트 감지
    ├── timing/
    │   └── timing_profiler_base.hpp   ← 락-프리 히스토그램 기반 타이밍 프로파일러
    └── threading/
        ├── thread_config.hpp      ← CPU 코어별 스레드 레이아웃 프리셋
        ├── thread_utils.hpp       ← 스레드 구성/검증 유틸리티
        ├── publish_buffer.hpp     ← 락-프리 SPSC 퍼블리시 버퍼
        └── seqlock.hpp            ← 락-프리 단일 쓰기/다중 읽기 동기화
```

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **CMakeLists.txt** | C++20 표준 명시 (`CMAKE_CXX_STANDARD 20`), 엄격 컴파일러 경고 플래그 (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`) |
| **캐시 라인 상수 통합** | `kCacheLineSize`를 `types.hpp`에서 단일 정의 → SeqLock, SPSC 버퍼에서 중복 제거 (기존 `kSeqLockCacheLineSize`, `kPublishCacheLineSize` 제거) |
| **C++20 concepts** | `TriviallyCopyableType` concept 추가 — 락-프리 프리미티브 타입 제약 명시 |
| **constexpr 강화** | `SubscribeRoleToString()`, `PublishRoleToString()` → `constexpr`, `ComputeBiquad()` → `constexpr` |
| **`[[likely]]/[[unlikely]]`** | SPSC 버퍼 `Push()` hot path에 분기 힌트 추가 — 버퍼 풀 경로를 unlikely로 마킹 |
| **`[[nodiscard]]`** | `ApplyThreadConfig()`, `ApplyThreadConfigWithFallback()` 반환값 무시 방지 |
| **static_assert** | `BesselFilterN<N>`, `KalmanFilterN<N>` — `N > 0` 컴파일 타임 검증 |
| **RT-safety 문서화** | `GetSubscribeTopicName()` 힙 할당 경고 주석 추가 |
| **session_dir.hpp** | `char[]` → `std::array<char, 16>` 버퍼 안전성 강화 |

---

## 모듈 상세

### 타입 (`types/types.hpp`)

프레임워크 전체에서 공유하는 컴파일 타임 상수, 데이터 구조, 열거형을 정의합니다.

#### 주요 상수

| 상수 | 값 | 설명 |
|------|---|------|
| `kCacheLineSize` | 64 (또는 HW 값) | 캐시 라인 크기 — 모든 threading 프리미티브에서 공유 |
| `kNumRobotJoints` | 6 | UR5e 관절 수 |
| `kMaxRobotDOF` | 12 | 최대 로봇 자유도 |
| `kNumHandMotors` | 10 | 핸드 모터 수 |
| `kDefaultNumFingertips` | 4 | 기본 핑거팁 수 |
| `kMaxFingertips` | 8 | 최대 핑거팁 수 |
| `kBarometerCount` | 8 | 핑거팁 당 기압 센서 수 |
| `kTofCount` | 3 | 핑거팁 당 ToF 센서 수 |
| `kMaxHandSensors` | 88 | 최대 핸드 센서 채널 수 |
| `kMaxInferenceValues` | 64 | 최대 추론 출력 크기 |
| `kFTValuesPerFingertip` | 13 | 핑거팁 당 F/T 추론 출력 수 |
| `kFTHistoryLength` | 12 | F/T 추론 FIFO 히스토리 길이 |

#### 주요 구조체

**레거시 타입 (UR5e 전용):**

| 구조체 | 설명 |
|--------|------|
| `RobotState` | 6-DOF 관절 위치, 속도, 토크, TCP 위치, dt, 반복 카운터 |
| `HandState` | 모터 위치/속도, 센서 데이터 (raw + filtered), 핑거팁 수, 유효성 |

**일반화된 디바이스 타입 (가변 DOF):**

| 구조체 | 설명 |
|--------|------|
| `DeviceState` | 가변 채널 디바이스 상태 — `positions[64]`, `velocities[64]`, `efforts[64]`, `sensor_data[128]`, `sensor_data_raw[128]` |
| `ControllerState` | `devices[4]` (DeviceState 배열) + `num_devices`, `dt`, `iteration` |
| `DeviceOutput` | 가변 채널 출력 — `commands[64]`, `goal_positions[64]`, `target_positions[64]`, `target_velocities[64]` |
| `ControllerOutput` | `devices[4]` (DeviceOutput 배열) + `actual_task_positions[6]`, `valid`, `command_type` |
| `FingertipFTState` | 핑거팁 힘/토크 추론 결과 (`kFTValuesPerFingertip = 13`) |

**설정 타입:**

| 구조체 | 설명 |
|--------|------|
| `DeviceNameConfig` | 디바이스 이름, 관절/센서 이름 매핑, URDF 설정(`DeviceUrdfConfig`), 관절 한계(`DeviceJointLimits`) |
| `DeviceUrdfConfig` | URDF 파싱용 — `package`, `path`, `root_link`, `tip_link` |
| `DeviceJointLimits` | 최대 속도/가속도/토크, 위치 상/하한 |
| `TopicConfig` | 디바이스 그룹별 구독/퍼블리시 토픽 라우팅 (map 기반) |

#### 열거형

| 열거형 | 값 | 설명 |
|--------|---|------|
| `CommandType` | `kPosition`, `kTorque` | 커맨드 모드 |
| `SubscribeRole` | `kState`, `kSensorState`, `kTarget` | 구독 역할 |
| `PublishRole` | `kJointCommand`, `kRos2Command`, `kTaskPosition`, `kTrajectoryState`, `kControllerState` | 퍼블리시 역할 |

#### C++20 Concepts

| Concept | 설명 |
|---------|------|
| `FloatingPointType` | `std::floating_point<T>` — 필터 계수, 게인 파라미터 타입 제약 |
| `TriviallyCopyableType` | `std::is_trivially_copyable_v<T>` — SeqLock/SPSC 버퍼 호환 타입 제약 |

> `SubscribeRoleToString()` / `PublishRoleToString()` — `constexpr` 변환 함수.

> **RT 안전 설계:** 모든 구조체는 zero-initialized, trivially copyable로 설계되어 SeqLock/SPSC 버퍼와 호환됩니다.
> **주의:** `TopicConfig::GetSubscribeTopicName()`은 `std::string`을 반환하므로 RT 경로에서 호출 금지 — 초기화 시에만 사용.

---

### 필터 (`filters/`)

#### Bessel 필터 (`bessel_filter.hpp`)

4차 Bessel 저역통과 필터 — 최대 평탄 군지연 특성으로 신호 형상을 보존합니다.

- 2개 연속 biquad 섹션 (bilinear transform + 주파수 prewarping)
- N개 독립 채널 동시 필터링 (`static_assert(N > 0)` 컴파일 타임 검증)
- RT-safe: `Apply()` 함수는 `noexcept`, 할당 없음
- `ComputeBiquad()` — `constexpr` 지원 (고정 차단 주파수 시 컴파일 타임 계산 가능)

| 타입 별칭 | N | 용도 |
|-----------|---|------|
| `BesselFilter6` | 6 | 로봇 관절 |
| `BesselFilter11` | 11 | 핑거팁 센서 |
| `BesselFilterBaro` | 64 | 기압 센서 |
| `BesselFilterTof` | 24 | ToF 센서 |
| `BesselFilter1` | 1 | 스칼라 |

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(double cutoff_hz, double sample_rate_hz)` | 차단 주파수 + 샘플링 레이트로 계수 계산 |
| `Apply` | `[[nodiscard]] array<double,N> Apply(const array<double,N>&) noexcept` | N채널 동시 필터링 |
| `ApplyScalar` | `[[nodiscard]] double ApplyScalar(double x, size_t ch) noexcept` | 단일 채널 필터링 |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |

```cpp
BesselFilter6 filter;
filter.Init(30.0, 500.0);  // 30 Hz 차단, 500 Hz 샘플링
auto filtered = filter.Apply(raw_positions);  // RT-safe
```

#### 칼만 필터 (`kalman_filter.hpp`)

이산시간 칼만 필터 — 등속 운동 모델 기반 위치/속도 추정기입니다.

- 상태: [position, velocity]ᵀ (채널당)
- 관측: 위치만 관측 (H = [1, 0])
- `static_assert(N > 0)` — 컴파일 타임 채널 수 검증
- RT-safe: `PredictAndUpdate()` 함수는 `noexcept`, 할당 없음
- 접근자 사전 조건: `position(i)`, `velocity(i)` — `i < N` 필수 (범위 검사 미수행, 성능 우선)

| 타입 별칭 | N | 용도 |
|-----------|---|------|
| `KalmanFilter6` | 6 | 로봇 관절 |
| `KalmanFilter11` | 11 | 핑거팁 센서 |
| `KalmanFilter1` | 1 | 스칼라 |

**API:**

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `Init` | `void Init(double q_pos, double q_vel, double r, double dt)` | 노이즈 파라미터 + 시간 간격 설정 |
| `Predict` | `void Predict() noexcept` | 예측 단계만 수행 |
| `Update` | `[[nodiscard]] array<double,N> Update(const array<double,N>&) noexcept` | 갱신 단계만 수행 |
| `PredictAndUpdate` | `[[nodiscard]] array<double,N> PredictAndUpdate(const array<double,N>&) noexcept` | 예측+갱신 통합 |
| `SetInitialPositions` | `void SetInitialPositions(const array<double,N>&) noexcept` | 초기 위치 설정 |
| `velocities` | `[[nodiscard]] array<double,N> velocities() const noexcept` | 추정 속도 반환 |
| `Reset` | `void Reset() noexcept` | 내부 상태 초기화 |

```cpp
KalmanFilter6 kf;
kf.Init(1e-3, 1e-2, 1e-1, 0.002);  // q_pos, q_vel, r, dt
auto filtered = kf.PredictAndUpdate(measurements);  // RT-safe
auto vels = kf.velocities();  // 추정 속도
```

---

### 스레딩 (`threading/`)

#### 스레드 구성 (`thread_config.hpp`)

CPU 코어 수에 따른 스레드 레이아웃 프리셋을 제공합니다. `SelectThreadConfigs()`가 물리 코어 수를 감지하여 자동 선택합니다.

**코어 수별 RT 스레드 레이아웃:**

| 스레드 | 4코어 | 6코어 | 8코어 | 10코어 | 12코어 | 16코어 |
|--------|-------|-------|-------|--------|--------|--------|
| **rt_control** (FIFO 90) | Core 1 | Core 2 | Core 2 | Core 7 | Core 7 | Core 2 |
| **sensor_io** (FIFO 70) | Core 2 | Core 3 | Core 3 | Core 8 | Core 8 | Core 3 |
| **udp_recv** (FIFO 65) | Core 2 | Core 5 | Core 4 | Core 9 | Core 9 | Core 9 |
| **logger** (OTHER -5) | Core 3 | Core 4 | Core 5 | Core 9 | Core 10 | Core 10 |
| **rt_publish** (OTHER -3) | Core 3 | Core 5 | Core 6 | Core 9 | Core 11 | Core 11 |
| **aux** (OTHER 0) | Core 3 | Core 5 | Core 6 | Core 9 | Core 11 | Core 11 |

`SelectThreadConfigs()`가 물리 코어 수를 자동 감지하여 적절한 레이아웃을 선택합니다.

**MuJoCo 시뮬레이션 코어 레이아웃:**

`GetSimCoreLayout(physical_cores)` — `constexpr` 함수로 시뮬레이션/뷰어 스레드 코어를 할당합니다.

#### 스레드 유틸리티 (`thread_utils.hpp`)

| 함수 | 설명 | RT-safe | `[[nodiscard]]` |
|------|------|---------|-----------------|
| `ApplyThreadConfig()` | CPU 어피니티, 스케줄러, 우선순위 설정 | N/A (초기화) | Yes |
| `ApplyThreadConfigWithFallback()` | RT 실패 시 SCHED_OTHER 폴백 | N/A (초기화) | Yes |
| `CheckThreadHealthFast()` | 비트필드 기반 스레드 상태 검증 | **Yes** | - |
| `VerifyThreadConfig()` | 현재 스레드 설정 문자열 반환 | No | - |
| `GetPhysicalCpuCount()` | 물리 코어 수 (SMT 제외) | No | - |
| `GetOnlineCpuCount()` | 논리 CPU 수 (SMT 포함) | No | - |
| `SelectThreadConfigs()` | 코어 수 기반 레이아웃 자동 선택 | No | - |
| `GetThreadMetrics()` | 레이턴시 통계 (min/max/avg/p95/p99) | No | - |

#### SeqLock (`seqlock.hpp`)

락-프리 단일 쓰기 / 다중 읽기 동기화 프리미티브입니다.

- **Writer**: Wait-free (2개 atomic store + memcpy), 홀수 시퀀스 = 쓰기 진행 중
- **Reader**: Lock-free (contention 시 재시도, torn read 감지)
- 요구사항: `T`는 `std::is_trivially_copyable_v<T>` 만족 필수
- 캐시 라인 정렬 (`kCacheLineSize` — `types.hpp`에서 통합 정의)

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

---

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

- 캐시 라인 정렬로 producer/consumer 인덱스 false sharing 방지
- `memory_order_acquire/release`로 동기화
- `[[unlikely]]` 분기 힌트로 hot path 최적화 (버퍼 풀 경로)

#### LogEntry 주요 필드

| 카테고리 | 필드 |
|---------|------|
| 타이밍 | `timestamp`, `t_state_acquire_us`, `t_compute_us`, `t_publish_us`, `t_total_us`, `jitter_us` |
| 공유 | `actual_task_positions[6]`, `command_type` |
| 디바이스별 (`DeviceLogSlot[4]`) | `num_channels`, `goal_positions[]`, `actual_positions[]`, `actual_velocities[]`, `efforts[]`, `commands[]`, `trajectory_positions[]`, `trajectory_velocities[]`, `sensor_data[128]`, `sensor_data_raw[128]`, `valid` |
| 추론 | `inference_output[64]`, `inference_valid`, `num_inference_values` |

#### PublishSnapshot 주요 필드

| 카테고리 | 필드 |
|---------|------|
| 디바이스별 (`GroupCommandSlot[4]`) | `num_channels`, `commands[]`, `goal_positions[]`, `target_positions[]`, `target_velocities[]`, `actual_positions[]`, `actual_velocities[]` |
| 공유 | `command_type`, `actual_task_positions[6]`, `stamp_ns`, `active_controller_idx`, `num_groups` |

---

### 로깅 (`logging/`)

#### 데이터 로거 (`data_logger.hpp`)

타이밍 CSV 1개 + 디바이스별 CSV N개에 제어 루프 데이터를 기록합니다.

| CSV 파일 | 헤더 컬럼 |
|----------|----------|
| `timing_log.csv` | `timestamp, t_state_acquire_us, t_compute_us, t_publish_us, t_total_us, jitter_us` |
| `<device>_log.csv` | `timestamp, valid, goal_pos_*, actual_pos_*, actual_vel_*, actual_torque_*, task_pos_* (첫 디바이스만), sensor_raw_*, sensor_*, inference_* (마지막 디바이스만), command_*, command_type (첫 디바이스만), traj_pos_*, traj_vel_*` |

**설정:** `DeviceLogConfig` 구조체로 디바이스별 로그 경로, 관절/센서 이름, 채널 수를 지정합니다.

**API:**

| 메서드 | 설명 |
|--------|------|
| `DataLogger(timing_path, device_configs, num_inference_values)` | CSV 파일 열기 + 헤더 작성 |
| `LogEntry(const LogEntry&)` | 단일 항목 기록 |
| `DrainBuffer(ControlLogBuffer&)` | SPSC 버퍼 전체 드레인 → CSV 기록 |
| `Flush()` | 파일 버퍼 플러시 |
| `IsOpen()` | 파일 열림 상태 확인 |

#### 세션 디렉토리 (`session_dir.hpp`)

| 함수 | 설명 |
|------|------|
| `GenerateSessionTimestamp()` | `YYMMDD_HHMM` 형식 타임스탬프 생성 |
| `ResolveSessionDir()` | `RTC_SESSION_DIR` → `UR5E_SESSION_DIR` → 자동 생성 |
| `EnsureSessionSubdirs()` | controller, monitor, device, sim, plots, motions 하위 폴더 생성 |
| `CleanupOldSessions()` | 최대 세션 수 초과 시 오래된 세션 삭제 |

> **주의:** `ResolveSessionDir()`은 `std::filesystem::filesystem_error`를 throw할 수 있습니다. 초기화 시에만 호출하세요.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |

> **외부 의존성 없음** — 표준 C++ 라이브러리와 POSIX API만 사용합니다.

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_base
source install/setup.bash
```

헤더 전용 라이브러리이므로 컴파일되는 바이너리는 없습니다. 소비자 패키지에서 `#include`하여 사용합니다.

**컴파일러 요구사항:** C++20 호환 컴파일러 (GCC 10+, Clang 13+).

---

## 의존성 그래프 내 위치

**기반 패키지** — 프레임워크의 모든 패키지가 의존합니다.

```
rtc_base  ← 독립 (외부 의존성 없음)
    ↑
    ├── rtc_communication          (스레드 구성)
    ├── rtc_controller_interface   (타입, 토픽 설정)
    ├── rtc_controller_manager     (타입, 로깅, 스레딩, 퍼블리시 버퍼)
    ├── rtc_controllers            (타입, 필터, SeqLock)
    ├── rtc_inference              (타입)
    ├── rtc_status_monitor         (타입, 스레딩)
    ├── rtc_mujoco_sim             (타입, 스레딩)
    ├── ur5e_hand_driver           (타입, 로깅, 필터)
    └── ur5e_bringup               (타입, 로깅, 스레드 구성)
```

### 모듈별 사용 패키지

| 모듈 | 주요 사용 패키지 |
|------|-----------------|
| `types` | 전체 13개 하위 패키지 |
| `filters` | `rtc_controllers`, `ur5e_hand_driver` |
| `threading` | `rtc_controller_manager`, `rtc_communication`, `ur5e_bringup` |
| `logging` | `rtc_controller_manager`, `ur5e_bringup` |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
