# rtc_base

RTC 프레임워크의 헤더 전용 실시간 인프라 라이브러리. **유형**: 라이브러리 패키지

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다. RT 최적화: [rt-optimization.md](../docs/rt-optimization.md)

## 개요

정기 tick 제어 루프 (rate-agnostic; `control_rate` YAML — default·설계 범위는 [invariants.md](../agent_docs/invariants.md) §RT Path Invariants) 에서 쓸 수 있는 공유 데이터 타입, 락-프리 동기화 프리미티브, 신호 처리 필터, 스레드 구성, 타이밍 프로파일링, 로깅 인프라를 제공합니다. 거의 모든 `rtc_*` 패키지가 의존하므로 이 패키지의 변경은 전체 빌드·테스트 대상입니다 (PROC-3).

- 헤더 전용 -- 링크 타임 의존성 없음 (예외: 트레이싱 opt-in 시 `rtc_tracing` 공유 라이브러리)
- RT 경로에서 힙 할당·시스템 콜 금지, RT-safe 함수는 `noexcept`
- 캐시 라인 정렬 (`kCacheLineSize`) 로 false sharing 방지
- C++20 필수 (GCC 10+, Clang 13+)

## 빠른 시작

소비 패키지의 `package.xml` 에 `<depend>rtc_base</depend>`, `CMakeLists.txt` 에 다음을 추가합니다.

```cmake
find_package(rtc_base REQUIRED)
ament_target_dependencies(my_target rtc_base)
```

```cpp
#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_base/threading/seqlock.hpp"

rtc::BesselFilterN<6> filter;                 // 6채널
filter.Init(30.0, 500.0);                     // 차단 30 Hz, 샘플링 500 Hz
auto filtered = filter.Apply(raw_positions);  // RT-safe

rtc::SeqLock<rtc::ControllerState> state_lock;
state_lock.Store(new_state);                  // 단일 writer (RT tick), wait-free
auto snapshot = state_lock.Load();            // 다른 스레드, lock-free
```

빌드·테스트는 [루트 README](../README.md#빠른-시작) 를 따릅니다. 이 패키지만: `./build.sh --tests -p rtc_base`. 컴파일러 경고는 `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion` 입니다.

## 인터페이스

공개 헤더는 `include/rtc_base/<영역>/` 아래에 있고 시그니처·사전조건은 각 헤더의 Doxygen 이 SSoT 입니다.

| 영역 | 헤더 | 주요 타입 |
|------|------|----------|
| `types/` | `types.hpp` | 상수 (`kMax*`), `DeviceState`, `ControllerState`, `ControllerOutput`, 열거형, `TopicConfig` |
| `concurrency/` | `spsc_queue.hpp` | `SpscQueue<T, N>` |
| `threading/` | `seqlock.hpp` | `SeqLock<T>` |
| | `publish_buffer.hpp` | `PublishSnapshot`, `ControlPublishBuffer` |
| | `thread_config.hpp`, `thread_config_generated.hpp` | `ThreadConfig`, `SystemThreadConfigs`, `MpcThreadConfig` (tier 상수는 생성물) |
| | `thread_utils.hpp`, `cpu_topology.hpp` | `ApplyThreadConfig*`, `SelectThreadConfigs`, `NucGeneration` |
| | `periodic_rt_thread.hpp` | `PeriodicRtThread` (CM/MPC 공용 고정 주기 루프 base) |
| | `rt_heap.hpp` | `ConfigureRtHeap` |
| `filters/` | `bessel_filter.hpp`, `kalman_filter.hpp` | `BesselFilterN<N>`, `KalmanFilterN<N>` (`KalmanFilter6/11/1`) |
| | `sensor_rate_estimator.hpp`, `sliding_trend_detector.hpp` | `SensorRateEstimator`, `SlidingTrendDetector<N, S>` |
| `timing/` | `timing_profiler_base.hpp` | `TimingProfilerBase<Buckets, BucketWidthUs, BudgetUs>` |
| | `thread_timing_{sample,producer,csv_logger}.hpp`, `rt_tick_timing_sample.hpp` | `ThreadTimingProducer<Payload, N>`, `RtTickTimingPayload` |
| `logging/` | `thread_csv_{producer,logger}.hpp` | `ThreadCsvProducer<Payload, N>`, `ThreadCsvLogger<Payload>` |
| | `session_dir.hpp`, `run_id.hpp` | `ResolveSessionDir`, `EnsureSessionSubdirs` |
| `tracing/` | `trace_scope.hpp`, `rtc_tracepoints.hpp` | `RTC_TRACE_SCOPE` (opt-in) |
| `utils/` | `clamp_commands.hpp`, `device_passthrough.hpp` | `ClampSymmetric`, `ClampRange`, `PassthroughSecondaryDevices` |

`test/include/rtc_base/testing/` 의 `malloc_gate.hpp` · `no_malloc_scope.hpp` · `wait_until.hpp` 는 **test 전용**이라 install 되지 않습니다 (형제 패키지는 `../rtc_base/test/include` 로 소비). 사용 계약은 각 헤더 상단 주석이 SSoT 이고 ASan 과의 제약은 [docs/testing.md](../docs/testing.md) 에 있습니다.

### 타입 (`types/types.hpp`)

프레임워크 전체가 공유하는 컴파일 타임 상수, 데이터 구조, 열거형입니다. `rtc_base` 에는 robot/sensor-agnostic 타입만 둡니다 (hand 전용 상수·`UdpHandState` 는 `udp_hand_driver`, grasp/wbc/ToF POD 는 각 controller 패키지).

> **trivially copyable 계약** — `DeviceState`, `ControllerState`, `DeviceOutput`, `ControllerOutput`, `Pose` (기본 zero-initialized) 와 controller 의 gains struct 는 trivially copyable 이어야 합니다 (`std::string` / `std::vector` / virtual 금지). `SeqLock<T>` 와 `SpscQueue<T, N>` 이 `static_assert` 로 요구하며, 개념은 `rtc::TriviallyCopyableType` 입니다.

> **명명 규약 (capacity vs default)** — `kNum*` 는 YAML 이 필드를 생략했을 때의 **기본값**이고 실제 채널 수는 런타임 `DeviceState::num_channels` 가 정합니다. `kMax*` 는 `std::array` 멤버의 컴파일 타임 **상한 용량**입니다. 코드는 실제 카운트가 `kNum*` 와 같다고 가정하지 않고, 상한을 넘는 로봇은 상한을 올리되 코드에 분기를 추가하지 않습니다.

| 상수 | 값 | 분류 |
|------|---|------|
| `kCacheLineSize` | 64 | HW const |
| `kTaskSpaceDim` | 6 | HW const (SE(3), 설정 불가) |
| `kNumRobotJoints` | 6 | default |
| `kMaxRobotDOF` | 12 | capacity |
| `kMaxDeviceChannels` / `kMaxSensorChannels` / `kMaxInferenceValues` | 64 / 128 / 64 | capacity |
| `kMaxSensorGroups` / `kMaxTaskLinks` | 8 / 8 | capacity |
| `ControllerState::kMaxDevices` · `ControllerOutput::kMaxDevices` · `PublishSnapshot::kMaxGroups` | 8 | capacity (구조체 스코프) |
| `kDefaultMaxJointVelocity` / `kDefaultMaxJointTorque` | 2.0 rad/s / 150.0 N·m | default |

| 열거형 · 헬퍼 | 값 / 역할 |
|--------------|----------|
| `CommandType` | `kPosition`, `kTorque`, `kPdFeedforward` (PD 위치 서보 + `DeviceOutput::feedforward` 오버레이) |
| `GoalType` | `kJoint`, `kTask` |
| `PublishRole` | `kRobotTransforms` (`tf2_msgs/TFMessage`). 구독 역할은 `target` 하나뿐이라 enum 이 없다 |
| `DeviceCapability` | `kNone`, `kJointState`, `kMotorState`, `kSensorData`, `kInference` (비트마스크) |
| `StateLane` · `IsLaneReadable` · `IsDeviceReadable` | lane 별 hole_mask 로 슬롯 freshness 판정 |
| `SteadyNowNs` · `SensorGroupAgeNs` · `IsSensorGroupFresh` | 수신축 (`steady_clock`) 나이·freshness. 판정 규범은 [invariants.md](../agent_docs/invariants.md) |

`GoalTypeToString()` / `CommandTypeToString()` / `PublishRoleToString()` 는 `constexpr` 이고, `CommandTypeToString()` 은 `JointCommand.command_type` 와 같은 와이어 문자열 (`"position"` / `"torque"` / `"pd_feedforward"`) 을 반환합니다.

| 구조체 | 역할 |
|--------|------|
| `DeviceState` / `ControllerState` | 가변 채널 디바이스 상태 (position·velocity·effort, motor, sensor, inference) / `devices[8]` + `dt`, `iteration`, `t_relative_s` |
| `DeviceOutput` / `ControllerOutput` | 디바이스별 command·goal·target·trajectory·feedforward / `devices[8]` + task-space 값 + TF 발행용 SE3 |
| `Pose` | RT-safe SE3 (`position[3]`, Hamilton quaternion w,x,y,z) |
| `DeviceNameConfig` | 디바이스 이름·joint/sensor 이름·`DeviceUrdfConfig`·`DeviceJointLimits`·`DeviceSensorLayout`·`safe_position` |
| `TopicConfig` | 디바이스 그룹별 구독/퍼블리시 토픽 (YAML 삽입 순서 보존). `GetFirstSubscribeTopic()` 은 `std::string` 반환이라 초기화 시에만 |

`ControllerState::t_relative_s` 는 CM 이 매 tick 채우는 세션 상대 시간입니다. controller 는 `chrono::*::now()` 대신 이 값을 로그 timestamp 로 씁니다.

### 필터 (`filters/`)

모두 `noexcept`, 할당 없음, 채널 수 `N` 은 템플릿 인자 (`static_assert(N > 0)`) 입니다. 디바이스·센서별 alias 는 사용 패키지에서 선언합니다 (rtc_base 에는 `KalmanFilter6/11/1` 만).

| 클래스 | 용도 | 주요 API |
|--------|------|---------|
| `BesselFilterN<N>` | 4차 Bessel 저역통과 (최대 평탄 군지연) | `Init(cutoff_hz, sample_rate_hz)` (Nyquist 미만), `Apply`, `ApplyScalar`, `Reset` |
| `KalmanFilterN<N>` | 등속 모델 위치/속도 칼만 필터 (위치만 관측) | `Init(q_pos, q_vel, r, dt)`, `PredictAndUpdate`, `positions`, `velocities`, `Reset` |
| `SensorRateEstimator` | EMA 센서 샘플링 레이트 추정 (이상치 제거) | `Init(nominal_hz)`, `Tick` (단일 스레드), `rate_hz`, `warmed_up`, `deviation_warning` |
| `SlidingTrendDetector<N, S>` | O(1) 슬라이딩 윈도우 OLS 드리프트 감지 | `Init(window, threshold, rate_hz)`, `Update` → `Result{slopes, drift_flags, window_full}` |

```cpp
KalmanFilter6 kf;
kf.Init(1e-3, 1e-2, 1e-1, 0.002);                  // q_pos, q_vel, r, dt
auto filtered = kf.PredictAndUpdate(measurements);
auto vels = kf.velocities();
```

접근자 `position(i)` · `velocity(i)` 는 `i < N` 이 사전조건입니다 (범위 검사 없음).

### 타이밍 (`timing/`)

`TimingProfilerBase<Buckets, BucketWidthUs, BudgetUs>` 는 락-프리 단일 프로듀서 프로파일러입니다. 히스토그램 기반 p95/p99, count/sum/min/max/last/mean/stddev, over-budget 카운터를 제공합니다. 기본 `<20, 100, 2000>` 이며 백분위 해상도가 모자라면 `Buckets × BucketWidthUs` 를 유지한 채 폭을 줄입니다 (예: `ControllerTimingProfiler` `<200, 10, 2000>`).

| 메서드 | 설명 |
|--------|------|
| `UpdateTotal(double us)` | 샘플 기록 (프로듀서 스레드 하나) |
| `GetBaseStats()` | `BaseStats` 스냅샷 (어느 스레드에서나) |
| `LastUs()` / `ResetBase()` | 마지막 값 / 통계 초기화 |

백분위수는 `ResetBase()` 이후 **누적** 값입니다 -- "최근 N개" 가 필요하면 호출자가 주기적으로 `ResetBase()` 합니다. 스냅샷 일관성·오버플로 버킷 보간·max clamp 는 헤더 Doxygen 이 설명합니다.

**Per-thread timing 채널** (`thread_timing_sample.hpp` · `thread_timing_producer.hpp` · `thread_timing_csv_logger.hpp`): 임의 RT/soft-RT 스레드의 per-tick CSV 로깅입니다. 접두 3열 `t_wall_ns,tick_count,run_id` 는 logger 가 emit 하므로 RT producer 는 비용을 내지 않습니다. 새 채널은 다음 세 가지로 추가합니다.

1. trivially copyable `Payload` struct
2. header/row writer 자유 함수
3. `using MyBuffer = ThreadTimingProducer<Payload, N>;`

CM (`CmTimingBuffer`), MPC (`MpcTimingBuffer`), hand (`HandTimingBuffer`) 가 `RtTickTimingPayload` (`t_state_us, t_compute_us, t_publish_us, t_total_us, jitter_us`) 로 쓰는 사용처입니다.

### 스레딩 (`threading/`)

#### 스레드 구성 (`thread_config.hpp`)

`ThreadConfig` 는 CPU 어피니티, 스케줄러 정책, 우선순위, nice, 스레드 이름을 정의하고, 코어 수별 프리셋 (4, 6, 8, 10, 12, 14, 16코어) 을 제공합니다.

> **레이아웃 값의 SSoT 는 이 헤더가 아니라 [thread_layout.yaml](../repo_scripts/config/thread_layout.yaml) 입니다.** tier 상수 (`k*Config*`) 와 `SelectThreadConfigsForCoreCount()` 는 `thread_config_generated.hpp` 로 생성됩니다. 같은 manifest 가 shell 헬퍼 (`repo_scripts/scripts/lib/thread_layout_generated.sh`), Python launch 미러 (`rtc_tools/rtc_tools/launch/thread_layout_generated.py`), 아래 매트릭스를 생성합니다. tier 를 바꾸려면 manifest 를 고치고 `gen_thread_layout.py --write` 를 돌립니다.

> **선언 순서 규약** (manifest 의 role 순서가 곧 emission 순서): tier 블록은 코어 수 오름차순, 각 tier 내부는 `sched_priority` 내림차순 (rt_control 90 → rt_callback 70 → mpc 60/55 → SCHED_OTHER 0 그룹), priority 동률은 심볼명 알파벳 순 (Arm → Hand → NrtCallback → NrtLogging → SimThread → Viewer).

**코어 수별 스레드 레이아웃:**

<!-- BEGIN GENERATED: thread-layout-matrix -->
<!-- Generated from repo_scripts/config/thread_layout.yaml by repo_scripts/scripts/gen_thread_layout.py. Do not edit by hand. -->

| 스레드 | 4코어¹ | 6코어² | 8코어 | 10코어 | 12코어 | 14코어 | 16코어 |
|--------|-------|-------|-------|-------|-------|-------|-------|
| **rt_control** (FIFO 90) | Core 1 | Core 1 | Core 1 | Core 1 | Core 1 | Core 1 | Core 1 |
| **rt_callback** (FIFO 70) | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 |
| **mpc_main** (FIFO 60) | Core 3 (CFS¹) | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 |
| **arm_driver** (CFS 0⁴) | Core 0 | Core 4 | Core 4 | Core 4 | Core 4 | Core 4 | Core 4 |
| **hand_driver** (CFS 0) | Core 0 | Core 4 | Core 5 | Core 5 | Core 5 | Core 5 | Core 5 |
| **nrt_callback** (CFS 0) | Core 0 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 |
| **nrt_publish** (CFS 0) | Core 0 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 |
| **nrt_logging** (CFS -5) | Core 0 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 |
| **sim_thread** (CFS 0) | -1³ | -1³ | -1³ | -1³ | -1³ | -1³ | -1³ |
| **viewer** (CFS 0) | -1³ | -1³ | -1³ | -1³ | -1³ | -1³ | -1³ |

<!-- END GENERATED: thread-layout-matrix -->

> ¹ 4코어는 degraded mode — mpc 가 CFS 로 강등 (RT 자원 부족). Core 0 에 nrt + driver 모두 합쳐짐.
> ² 6코어는 degraded mode — arm/hand_driver 가 Core 4 공유. (nrt 3 레인이 Core 2 인 것은 모든 ≥6c tier 공통이며 6코어 특례가 아니다.)
> ³ `cpu_core = -1` sentinel: pin 없음. sim_thread / viewer 는 모든 tier 에서 cpu_shield 가 해제한 코어에서 CFS 로 roam.
> ⁴ `arm_driver` 의 CFS 0 은 *프로세스* 모델이다 — `ApplyThreadConfig` 대상이 아니라 launch 가 소비하는 코어 배치 값이고, 프로세스의 main/executor 는 실제로 CFS 다.
>   그 안의 upstream `controller_manager` 제어 루프는 FIFO 50 이며, 이 표의 코어에 핀되는 것은 프로세스가 아니라 그 루프다 (`taskset` 은 main thread 밖에 못 옮긴다). 핀은 launch 가 생성하는 CM 파라미터 파일이 나른다.
>
> **단조성 불변식**: 물리 코어가 증가하면 per-thread 격리는 절대 감소하지 않는다 (`test/test_mpc_thread_config.cpp` 가 보장).

`SystemThreadConfigs` 는 `rt_control`, `rt_callback`, `nrt_logging`, `nrt_callback`, `nrt_publish`, `arm_driver`, `hand_driver`, `sim_thread`, `viewer`, `mpc` (`MpcThreadConfig`) 를 가집니다. actuator publish 는 `rt_control` 이 tick 안에서 inline 으로 하고, hand comm thread 는 `udp_hand_driver` 내부 (`kHandCommThreadConfig`) 에 있습니다. 일반 `rtc_communication::Transceiver` 는 `kRtUdpRecvConfig` (`cpu_core=-1`) 를 호출자가 명시적으로 씁니다.

`MpcThreadConfig` 의 필드는 `main` (MPC solve 스레드의 `ThreadConfig`) 하나입니다. Aligator 의 병렬화는 OpenMP 이고 자기 스레드를 직접 소유하므로 외부 핸들을 받지 않습니다. `ValidateSystemThreadConfigs()` 는 (1) MPC main priority < rt_callback priority, (2) `arm_driver` / `hand_driver` 의 `cpu_core` 가 모든 RT controller thread (rt_control/rt_callback/mpc_main) 와 disjoint, (3) `cpu_core=-1` 은 disjointness 검사에서 제외를 검증합니다.

**MuJoCo 시뮬레이션:** 모든 tier 에서 `sim_thread` / `viewer` 의 `cpu_core = -1` (no pin) 입니다. `cpu_shield.sh --sim` 은 RT cluster 코어 (Core 1-5) 만 격리하고 나머지를 해제하므로 physics 와 GLFW viewer 는 해제된 코어에서 CFS 로 roam 합니다. 같은 값이 `rtc_tools.launch.thread_layout.get_sim_core()` / `get_viewer_core()` 로 미러링됩니다.

#### 스레드 유틸리티 (`thread_utils.hpp`)

| 함수 | 설명 |
|------|------|
| `ApplyThreadConfig()` | 스레드 이름 → CPU 어피니티 → 스케줄러·우선순위 (`[[nodiscard]]`). 이름을 먼저 붙이므로 어피니티(EINVAL)·스케줄러(EPERM) 가 실패해도 `ps -L`·검증기가 이름으로 찾는다 |
| `ApplyThreadConfigWithFallback()` | RT 실패 시 SCHED_OTHER 폴백 |
| `ApplyThreadConfigVerbose()` | `ApplyThreadConfig()` + 표준 성공/실패 로그 1줄 (런타임 스레드 진입점의 announce 단일 출처) |
| `ValidateThreadConfig()` / `ValidateSystemThreadConfigs()` | 단일 / 전체 설정 검증 (코어 범위, 정책, 우선순위, 이름, 코어·우선순위 충돌) |
| `CheckThreadHealthFast()` | 비트필드 기반 상태 검증 -- **RT-safe** (`ThreadHealthFlag`: `kOk`, `kWrongCore`, `kPolicyChanged`, `kPriorityChanged`, `kNiceChanged`) |
| `CheckThreadHealth()` / `VerifyThreadConfig()` | 문자열 기반 검사 / 현재 설정 문자열 (RT 불가) |
| `GetPhysicalCpuCount()` / `GetOnlineCpuCount()` | 물리 코어 수 (SMT 제외, sysfs) / 논리 CPU 수 |
| `SelectThreadConfigs()` | 코어 수 기반 레이아웃 자동 선택 (`>=` 계단식) |
| `GetThreadStats()` / `GetThreadMetrics()` | 레이턴시 통계 (min/max/avg) / 포괄 통계 (jitter, p95, p99 포함) |

`ConfigureRtHeap()` (`rt_heap.hpp`) 는 glibc `mallopt(M_TRIM_THRESHOLD, -1)` + `mallopt(M_MMAP_MAX, 0)` 로 warmup 이후의 할당·해제가 syscall 이 되지 않게 합니다. `mlockall` (RT-HOST-1) 과 함께 RT 프로세스 main 진입 직후 1회 부르며 (프로세스 전역), 실패는 `false` 이고 호출자가 경고하고 계속 돕니다. 대가는 RSS 가 최대치에 머무는 것입니다 ([invariants.md](../agent_docs/invariants.md) RT 절의 ONNX Runtime 수용 조건 1).

#### SeqLock (`seqlock.hpp`)

락-프리 단일 쓰기 / 다중 읽기 프리미티브입니다. **`T` 는 trivially copyable** (`static_assert`) 이어야 하고, **writer 는 하나**여야 합니다 -- `Store` 는 seq 를 RMW 없이 읽고 쓰므로 두 writer 가 겹치면 payload 가 찢기거나 seq 가 홀수로 남아 이후 모든 `Load` 가 영원히 재시도합니다.

| 메서드 | 설명 |
|--------|------|
| `Store(const T&) noexcept` | Wait-free 쓰기 (홀수 시퀀스 = 쓰기 진행 중) |
| `Load() const noexcept` | Lock-free 읽기 (torn read 시 재시도) |
| `LoadInto(T& out) const noexcept` | 호출자 저장소로 직접 읽기 -- 큰 payload 의 스택 사본 제거. 재시도 중 `out` 이 잠시 torn 일 수 있으므로 다른 스레드가 읽는 저장소면 안 된다 |
| `sequence() const noexcept` | 현재 시퀀스 번호 |

#### SPSC 버퍼 (`concurrency/spsc_queue.hpp`)

RT 스레드 (producer) 에서 비-RT 스레드 (consumer) 로 데이터를 넘기는 락-프리 링 버퍼 `SpscQueue<T, N>` 입니다. `T` 는 trivially copyable POD, 용량 `N` 은 2의 거듭제곱 (`static_assert`) 입니다.

| 메서드 | 설명 |
|--------|------|
| `Push(const T&)` | RT-safe, wait-free. 가득 차면 `false` + `drop_count` 증가 |
| `Pop(T&)` | 비-RT consumer. 비었으면 `false` |
| `drop_count()` | 드롭된 항목 수 |

| 버퍼 | 용량 | 용도 |
|------|------|------|
| `ControlPublishBuffer` (`PublishSnapshot`) | 512 | ROS 2 퍼블리시 오프로드 |
| `CmTimingBuffer` / `HandTimingBuffer` | 512 | CM RT loop / hand comm loop per-tick timing CSV |
| `MpcTimingBuffer` | 128 | MPC per-tick timing CSV |

`PublishSnapshot` 은 디바이스별 `GroupCommandSlot[8]` (command·goal·target·trajectory·feedforward·actual·motor·sensor·inference 배열, resolved `command_type`, 그룹별 `stamp_ns`, TF source pose) 과 공유 필드 (`actual_task_positions[6]`, `stamp_ns`, `active_controller_idx`, `num_groups`) 로 구성됩니다.

### 로깅 (`logging/`)

**CM per-tick timing** -- `cm_timing_log.csv` 의 스키마는 `t_wall_ns, tick_count, run_id, t_state_us, t_compute_us, t_publish_us, t_total_us, jitter_us` 입니다 (MPC / hand 와 같은 8열). `run_id` 는 `logging/run_id.hpp` 가 `Open()` 에서 한 번 정해 매 행에 찍으므로, 같은 분에 재기동해 한 세션 디렉토리를 공유한 두 런을 분석이 합쳐 읽지 못합니다. controller 데이터 CSV 는 `rtc_controller_interface` 의 `ControllerLogSet` 과 각 controller 의 POD 미러가 맡습니다.

세 phase 열은 `t_total_us` 로 합산되지 않습니다. `PeriodicRtThread::MarkPublishDone()` 을 부르는 producer (CM) 는 남는 `t_total_us − (t_state_us + t_compute_us + t_publish_us)` 가 post-publish tail 입니다 (분석 쪽 파생 열은 `rtc_tools` `plotting/plotters/timing.py` 의 `add_tail_column`). 훅 `JitterMeaningful()` · `RequestLoopExit()` · `SuppressTimingThisTick()` 의 계약은 `periodic_rt_thread.hpp` Doxygen 을 봅니다.

#### Generic CSV 인프라 (`thread_csv_producer.hpp` + `thread_csv_logger.hpp`)

controller-owned 데이터 CSV 용 페어입니다. `timing/thread_timing_*` 와 달리 자동 컬럼이 없고 payload 가 행 전체를 소유하며, 스키마는 caller 가 정합니다.

| 클래스 | 메서드 | 설명 |
|--------|--------|------|
| `ThreadCsvProducer<Payload, N>` | `Push(payload)` | RT 스레드에서 SPSC push (wait-free, drop-on-full) |
| | `Drain(fn)` / `DropCount()` | 비-RT consumer 의 FIFO 드레인 / 누적 drop |
| `ThreadCsvLogger<Payload>` | `Open(path, hdr, row)` | 헤더/행 writer 와 함께 파일 열기 (append-on-reopen) |
| | `Log(payload)` | row writer 호출 + flush |

```cpp
rtc::ThreadCsvProducer<StateLogPod, 512> producer;   // StateLogPod: trivially copyable
rtc::ThreadCsvLogger<StateLogPod> logger;
logger.Open(path, &WriteHeader, &WriteRow);          // void(std::ostream&), void(std::ostream&, const Pod&)

producer.Push(pod);                                  // RT tick
producer.Drain([&](const StateLogPod& p) { logger.Log(p); });  // non-RT timer
```

#### 세션 디렉토리 (`session_dir.hpp`)

세션 로그는 `logging_data/YYMMDD_HHMM/` 아래에 쌓입니다.

```
logging_data/YYMMDD_HHMM/
├── controllers/   # controller 별 데이터 CSV (config_key 하위)
├── timing/        # cm/mpc/hand per-tick timing CSV
├── monitor/
├── device/        # device 통신 통계
├── sim/           # mujoco 스크린샷
├── plots/
├── motions/       # 모션 에디터 JSON
└── tracing/       # LTTng CTF 트레이스
```

`EnsureSessionSubdirs()` 가 만드는 하위 폴더는 `timing`, `monitor`, `device`, `sim`, `plots`, `motions`, `tracing` 입니다 (`controllers/` 는 각 controller 가 만듭니다).

| 함수 | 설명 |
|------|------|
| `GenerateSessionTimestamp()` | `YYMMDD_HHMM` 타임스탬프 |
| `ResolveLoggingRoot()` | `logging_data` 루트 결정 (아래 체인) |
| `ResolveSessionDir()` | `$RTC_SESSION_DIR`, 없으면 루트 아래 새 세션 디렉토리 생성 |
| `EnsureSessionSubdirs(dir)` / `TimingDir(dir)` | 하위 폴더 생성 / `<session>/timing/` 경로 |
| `ListSessionDirs(root)` / `CleanupOldSessions(root, max)` | 세션 목록 / 초과분 삭제 |

`ResolveLoggingRoot()` 결정 체인: (1) `$COLCON_PREFIX_PATH` 첫 entry 가 쓰기 가능한 디렉토리이면 그 `parent / "logging_data"`, (2) cwd 에서 상위로 올라가며 `install/` + `src/` 쌍이 있는 디렉토리의 `logging_data`, (3) 폴백 `cwd / "logging_data"`.

`ResolveSessionDir()` 은 `std::filesystem::filesystem_error` 를 throw 할 수 있으므로 초기화 시에만 호출합니다. Python 측 동일 로직은 `rtc_tools.utils.session_dir.resolve_logging_root()` 입니다.

### 트레이싱 (`tracing/`)

RT hot-path 를 Perfetto 로 관찰하는 LTTng UST scoped span 입니다. **컴파일타임 opt-in** -- CMake 옵션 `RTC_ENABLE_TRACING` (default OFF) 이 켜져야만 존재하며, 꺼진 빌드는 완전 no-op (RT 비용 0, lttng-ust 의존성 0) 입니다. `RTC_TRACE_SCOPE("Name")` 이 RAII span 매크로이고, ON 빌드는 `rtc:*` probe 를 shared lib `rtc_tracing` (export target `rtc_base::rtc_tracing`) 로 컴파일합니다. 소비 패키지는 이 target 을 링크해 `RTC_TRACING_ENABLED` define 과 lttng include 를 상속받습니다. 사용법은 [docs/tracing.md](../docs/tracing.md) §RT trace spans 입니다.

## 더 읽기

- [architecture.md](../agent_docs/architecture.md) -- threading, data flow, Execution Contexts 판정표
- [invariants.md](../agent_docs/invariants.md) -- RT / ARCH 규범 (이 패키지의 헤더가 따르는 RT-1 ~ RT-10)
- [rt-optimization.md](../docs/rt-optimization.md) · [tracing.md](../docs/tracing.md) · [testing.md](../docs/testing.md)
- [thread_layout.yaml](../repo_scripts/config/thread_layout.yaml) -- 스레드 레이아웃 SSoT

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `lttng-ust` *(optional)* | `RTC_ENABLE_TRACING=ON` 일 때만. `install.sh --tracing` 이 제공 (apt `liblttng-ust-dev`) |

기본 빌드는 표준 C++ 라이브러리와 POSIX API 만 씁니다. 라이선스: MIT ([LICENSE](../LICENSE)). 의존성 목록은 [package.xml](package.xml).
