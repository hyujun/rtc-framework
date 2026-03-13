# ur5e_rt_base

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스 (v5.10.0)의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

UR5e RT Controller 스택의 **공유 기반 패키지** — 모든 패키지가 공통으로 사용하는 타입 정의, 스레드 유틸리티, 잠금-없는(lock-free) 로깅 인프라, UDP 통신 프리미티브, 디지털 신호 필터를 제공하는 헤더-전용(header-only) C++20 라이브러리입니다.

## 개요

```
ur5e_rt_base (header-only)
    ├── types/
    │   └── types.hpp              ← 공유 데이터 구조체 + 상수 + 열거형 + 토픽 라우팅
    ├── threading/
    │   ├── thread_config.hpp      ← RT 스레드 설정 구조체 + 4/6/8코어 사전 정의 상수
    │   └── thread_utils.hpp       ← ApplyThreadConfig(), 헬스체크, 지연 통계, 자동 선택
    ├── logging/
    │   ├── log_buffer.hpp         ← SPSC 링 버퍼 (RT → 로그 스레드, 잠금-없음)
    │   ├── data_logger.hpp        ← 비-RT CSV 로거
    │   └── session_dir.hpp        ← 세션 디렉토리 관리 유틸리티 (v5.10.0)
    ├── udp/
    │   ├── udp_socket.hpp         ← UDP 소켓 RAII 래퍼
    │   ├── udp_codec.hpp          ← 범용 UDP 패킷 코덱 컨셉 (C++20)
    │   └── udp_transceiver.hpp    ← 범용 UDP 송수신기 템플릿 (jthread 기반)
    └── filters/
        ├── bessel_filter.hpp      ← 4차 Bessel 저역통과 필터 (N채널, noexcept)
        └── kalman_filter.hpp      ← 이산-시간 Kalman 필터 (N채널, 위치+속도 추정)
```

이 패키지는 실행 파일을 포함하지 않으며, `install(DIRECTORY include/)` 만으로 헤더를 내보냅니다.

**핵심 설계 원칙:**
- **RT 안전**: 모든 핫-패스 함수는 `noexcept`, `Init()` 이후 힙 할당 없음
- **제로 외부 의존성**: C++20 표준 라이브러리 + POSIX API만 사용 (ROS2 비의존)
- **네임스페이스**: `ur5e_rt_controller` (전체 워크스페이스 공통)

**의존성 그래프 내 위치:**

```
ur5e_rt_base   ← (독립 — 아무 ROS2 패키지에도 의존하지 않음)
    ↑
    ├── ur5e_rt_controller
    └── ur5e_hand_udp
```

---

## 헤더 파일 설명

### `ur5e_rt_base/types/types.hpp`

500Hz 제어 루프 전체에서 공유하는 데이터 구조체, 컴파일-시간 상수, 열거형, 토픽 라우팅 설정을 정의합니다.

#### 컴파일-시간 상수

| 상수 | 값 | 설명 |
|------|----|------|
| `kNumRobotJoints` | `6` | UR5e 로봇 관절 수 |
| `kNumHandMotors` | `10` | 손 모터 수 |
| `kNumFingertips` | `4` | 손가락 끝 개수 |
| `kBarometerCount` | `8` | 손가락당 기압 센서 수 |
| `kReservedCount` | `5` | 패킷 내 예약 필드 (저장되지 않음) |
| `kTofCount` | `3` | 손가락당 ToF 센서 수 |
| `kSensorDataPerPacket` | `16` | UDP 패킷당 센서 데이터 수 (8+5+3) |
| `kSensorValuesPerFingertip` | `11` | 저장되는 값 (8 barometer + 3 ToF) |
| `kNumHandSensors` | `44` | 총 손 센서 수 (4 × 11) |
| `kNumHandJoints` | `10` | 레거시 별칭 (`= kNumHandMotors`) |

#### `FloatingPointType` 컨셉 (C++20)

```cpp
template <typename T>
concept FloatingPointType = std::floating_point<T>;
```

게인 파라미터 및 필터 계수의 템플릿 파라미터를 부동소수점 타입으로 제한합니다.

#### 데이터 구조체

```cpp
namespace ur5e_rt_controller {

struct RobotState {
  std::array<double, 6>  positions{};    // 관절 위치 (rad)
  std::array<double, 6>  velocities{};   // 관절 속도 (rad/s)
  std::array<double, 6>  torques{};      // 관절 토크 (Nm)
  std::array<double, 3>  tcp_position{}; // TCP 위치 (m, x/y/z)
  double   dt{0.002};                    // 제어 주기 (s)
  uint64_t iteration{0};                 // 누적 제어 반복 횟수
};

struct HandState {
  std::array<float, 10>  motor_positions{};   // 모터 위치 (정규화 0.0–1.0)
  std::array<float, 10>  motor_velocities{};  // 모터 속도
  std::array<float, 44>  sensor_data{};       // 센서 데이터 (4 손가락 × 11 값)
  bool valid{false};                          // 데이터 유효성 플래그
};

struct ControllerState {
  RobotState robot{};
  HandState  hand{};
  // NOTE: dt와 iteration은 robot.dt, robot.iteration의 중복.
  // 양쪽 모두 동기화 유지 필수 — 불일치 시 미묘한 버그 발생.
  double   dt{0.002};
  uint64_t iteration{0};
};

enum class CommandType { kPosition, kTorque };

struct ControllerOutput {
  std::array<double, 6>  robot_commands{};           // 로봇 관절 명령 (rad 또는 Nm)
  std::array<float, 10>  hand_commands{};            // 손 명령 (0.0–1.0 정규화)
  std::array<double, 6>  actual_target_positions{};  // 실제 목표 위치 (로깅용)
  std::array<double, 6>  actual_task_positions{};    // 태스크 공간 위치 (로깅용)
  bool        valid{true};                           // 출력 유효성 플래그
  CommandType command_type{CommandType::kPosition};   // 위치/토크 명령 구분
  // ── 로깅 확장 필드 (v5.9.0) ──
  std::array<double, 6>  goal_positions{};        // 최종 목표 위치 (SetRobotTarget)
  std::array<double, 6>  target_velocities{};     // 궤적 보간 속도
  std::array<float, 10>  hand_goal_positions{};    // 핸드 최종 목표 위치
};

} // namespace ur5e_rt_controller
```

#### 토픽 라우팅 열거형 및 구조체

컨트롤러별 ROS2 토픽 구독/발행 설정을 위한 동적 라우팅 시스템입니다.

```cpp
enum class SubscribeRole {
  kJointState,   // 로봇 관절 위치/속도 (sensor_msgs/JointState)
  kHandState,    // 손 관절 상태 (Float64MultiArray)
  kTarget,       // 목표 위치 (Float64MultiArray)
};

enum class PublishRole {
  kPositionCommand,  // 로봇 위치 명령 (Float64MultiArray)
  kTorqueCommand,    // 로봇 토크 명령 (Float64MultiArray)
  kHandCommand,      // 손 모터 명령 (Float64MultiArray)
  kTaskPosition,     // 현재 태스크 공간 위치 (Float64MultiArray)
};

struct SubscribeTopicEntry {
  std::string   topic_name;
  SubscribeRole role;
};

struct PublishTopicEntry {
  std::string topic_name;
  PublishRole role;
  int         data_size{0};  // 메시지 크기 사전 할당 (0 = 역할별 기본값)
};

struct TopicConfig {
  std::vector<SubscribeTopicEntry> subscribe;
  std::vector<PublishTopicEntry>   publish;
};
```

---

### `ur5e_rt_base/threading/thread_config.hpp`

RT 스레드 설정을 위한 `ThreadConfig` 구조체와 4/6/8코어 시스템용 사전 정의 상수를 제공합니다.

```cpp
struct ThreadConfig {
  int         cpu_core;        // CPU 코어 번호 (0-based)
  int         sched_policy;    // SCHED_FIFO / SCHED_RR / SCHED_OTHER
  int         sched_priority;  // 우선순위 (FIFO/RR: 1–99, OTHER: 무시)
  int         nice_value;      // nice 값 (SCHED_OTHER 전용, -20~19)
  const char* name;            // 스레드 이름 (최대 15자, pthread_setname_np 제한)
};
```

#### 사전 정의 ThreadConfig 상수

**6코어 시스템** (Core 0-1: OS/DDS/IRQ, Core 2-5: RT 전용, `isolcpus=2-5`):

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|----------|------|
| `kRtControlConfig` | 2 | `SCHED_FIFO` | 90 | 500Hz 제어 루프 + 50Hz E-STOP 감시 |
| `kSensorConfig` | 3 | `SCHED_FIFO` | 70 | 센서 구독 스레드 (전용) |
| `kUdpRecvConfig` | **5** | `SCHED_FIFO` | 65 | UDP 수신 스레드 (sensor_io 경합 방지) |
| `kLoggingConfig` | 4 | `SCHED_OTHER` | nice -5 | CSV 로깅 스레드 |
| `kAuxConfig` | 5 | `SCHED_OTHER` | 0 | 보조 스레드 (이벤트 기반, 경량) |
| `kStatusMonitorConfig` | 4 | `SCHED_OTHER` | nice -2 | 비-RT 상태 모니터 (10 Hz) |
| `kHandFailureConfig` | 4 | `SCHED_OTHER` | nice -2 | 비-RT 손 고장 감지 (50 Hz) |

**8코어 시스템** (Core 0-1: OS/DDS/IRQ, Core 2-6: RT 전용, Core 7: 예비, `isolcpus=2-6`):

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|----------|------|
| `kRtControlConfig8Core` | 2 | `SCHED_FIFO` | 90 | 500Hz 제어 루프 |
| `kSensorConfig8Core` | 3 | `SCHED_FIFO` | 70 | 센서 구독 스레드 |
| `kUdpRecvConfig8Core` | **4** | `SCHED_FIFO` | 65 | UDP 수신 스레드 (전용 코어) |
| `kLoggingConfig8Core` | 5 | `SCHED_OTHER` | nice -5 | CSV 로깅 스레드 |
| `kAuxConfig8Core` | 6 | `SCHED_OTHER` | 0 | 보조 스레드 |
| `kStatusMonitorConfig8Core` | 6 | `SCHED_OTHER` | nice -2 | 비-RT 상태 모니터 (10 Hz) |
| `kHandFailureConfig8Core` | 6 | `SCHED_OTHER` | nice -2 | 비-RT 손 고장 감지 (50 Hz) |

**4코어 폴백** (Core 0: OS/DDS/IRQ, Core 1-3: RT):

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|----------|------|
| `kRtControlConfig4Core` | 1 | `SCHED_FIFO` | 90 | 500Hz 제어 루프 |
| `kSensorConfig4Core` | 2 | `SCHED_FIFO` | 70 | 센서 구독 스레드 |
| `kUdpRecvConfig4Core` | 2 | `SCHED_FIFO` | 65 | UDP 수신 (sensor_io 코어 공유 불가피) |
| `kLoggingConfig4Core` | 3 | `SCHED_OTHER` | nice -5 | CSV 로깅 스레드 |
| `kAuxConfig4Core` | 3 | `SCHED_OTHER` | 0 | 보조 스레드 (logging 코어 공유) |
| `kStatusMonitorConfig4Core` | 3 | `SCHED_OTHER` | nice 0 | 비-RT 상태 모니터 (10 Hz) |
| `kHandFailureConfig4Core` | 3 | `SCHED_OTHER` | nice 0 | 비-RT 손 고장 감지 (50 Hz) |

---

### `ur5e_rt_base/threading/thread_utils.hpp`

RT 스레드 설정을 적용하고 검증하는 유틸리티 함수, 헬스체크, 지연 통계, 런타임 자동 선택 기능을 제공합니다.

#### 스레드 설정 적용

```cpp
// CPU 친화성, 스케줄러 정책, 우선순위, 스레드 이름을 한 번에 적용
// 내부적으로 ValidateThreadConfig() 호출 → 유효하지 않으면 false 반환
[[nodiscard]] bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept;

// 그레이스풀 폴백: RT 스케줄링 실패 시 SCHED_OTHER로 대체
// CPU 친화성, 스레드 이름은 가능한 한 적용
// 반환: {전체_성공, 경고_메시지}
[[nodiscard]] std::pair<bool, std::string>
ApplyThreadConfigWithFallback(const ThreadConfig& cfg) noexcept;
```

#### 설정 검증

```cpp
// ThreadConfig 유효성 검사 — 빈 문자열이면 정상
// 코어 범위, 스케줄러 정책, 우선순위 범위, nice 값, 이름 길이 검증
[[nodiscard]] std::string ValidateThreadConfig(const ThreadConfig& cfg) noexcept;

// 시스템 전체 7개 스레드 설정의 충돌 검사
// RT+RT 동일 우선순위 동일 코어 → 에러 (SCHED_FIFO 기아 발생)
// RT+non-RT 동일 코어 → 허용 (RT가 무조건 선점)
[[nodiscard]] std::string ValidateSystemThreadConfigs(
    const SystemThreadConfigs& configs) noexcept;
```

#### 디버깅 및 진단

```cpp
// 현재 스레드의 CPU 친화성, 스케줄러, nice 값, 이름을 문자열로 반환
[[nodiscard]] std::string VerifyThreadConfig() noexcept;

// 문자열 기반 상세 헬스체크 (비-RT 안전 — std::string 힙 할당)
// expected_config가 nullptr이면 일반적인 RT 상태만 확인
[[nodiscard]] std::string CheckThreadHealth(
    const ThreadConfig* expected_config = nullptr) noexcept;

// 스레드 안전 strerror 래퍼 (strerror_r 사용)
[[nodiscard]] std::string SafeStrerror(int errnum) noexcept;
```

#### RT 안전 헬스체크

```cpp
// 비트필드 기반 헬스체크 — 힙 할당 없음, 500Hz RT 루프에서 주기적 호출 가능
enum class ThreadHealthFlag : uint8_t {
  kOk             = 0,
  kWrongCore      = 1 << 0,  // CPU 친화성 이탈
  kPolicyChanged  = 1 << 1,  // 스케줄러 정책 변경
  kPriorityChanged = 1 << 2, // RT 우선순위 변경
  kNiceChanged    = 1 << 3,  // nice 값 변경 (SCHED_OTHER)
};

[[nodiscard]] ThreadHealthFlag CheckThreadHealthFast(
    const ThreadConfig& expected) noexcept;
```

#### 지연 통계

```cpp
// 기본 통계: {min, max, avg} (비-RT 안전)
[[nodiscard]] std::tuple<double, double, double> GetThreadStats(
    const std::vector<double>& latencies_us) noexcept;

// 확장 통계: jitter, p95, p99 포함 (비-RT 안전)
struct ThreadMetrics {
  double min_latency_us;
  double max_latency_us;
  double avg_latency_us;
  double jitter_us;          // 표준편차
  double percentile_95_us;
  double percentile_99_us;
};

[[nodiscard]] ThreadMetrics GetThreadMetrics(
    const std::vector<double>& latencies_us) noexcept;
```

#### 런타임 자동 선택

```cpp
// 온라인 논리 CPU 수 반환
[[nodiscard]] int GetOnlineCpuCount() noexcept;

// 모든 스레드의 ThreadConfig를 하나로 묶은 구조체
struct SystemThreadConfigs {
  ThreadConfig rt_control;
  ThreadConfig sensor;
  ThreadConfig udp_recv;
  ThreadConfig logging;
  ThreadConfig aux;
  ThreadConfig status_monitor;  // Non-RT status monitor (10 Hz)
  ThreadConfig hand_failure;    // Non-RT hand failure detector (50 Hz)
};

// 런타임 CPU 수에 따라 최적 ThreadConfig 집합 자동 반환
// ≥8코어 → 8코어 레이아웃 / ≥6코어 → 6코어 레이아웃 / <6코어 → 4코어 폴백
[[nodiscard]] SystemThreadConfigs SelectThreadConfigs() noexcept;
```

**사용 예시:**

```cpp
#include "ur5e_rt_base/threading/thread_utils.hpp"
#include "ur5e_rt_base/threading/thread_config.hpp"

// 패턴 1: 개별 스레드에 config 직접 적용
auto t = std::thread([&]() {
  if (!ApplyThreadConfig(ur5e_rt_controller::kRtControlConfig)) {
    RCLCPP_WARN(logger, "RT 설정 실패 — SCHED_OTHER로 동작 (지터 증가)");
  }
  // ... 제어 루프
});

// 패턴 2: 그레이스풀 폴백 (실패 시에도 가능한 설정 적용)
auto [ok, warnings] = ApplyThreadConfigWithFallback(kRtControlConfig);
if (!ok) RCLCPP_WARN(logger, "%s", warnings.c_str());

// 패턴 3: 런타임 CPU 수 자동 감지 후 최적 config 집합 선택
auto cfgs = ur5e_rt_controller::SelectThreadConfigs();
// cfgs.rt_control, cfgs.sensor, cfgs.udp_recv, cfgs.logging, cfgs.aux 사용

// 패턴 4: 시스템 설정 충돌 사전 검증
auto errors = ValidateSystemThreadConfigs(cfgs);
if (!errors.empty()) RCLCPP_ERROR(logger, "Thread config conflict: %s", errors.c_str());

// 패턴 5: RT 루프 내 주기적 헬스체크 (힙 할당 없음)
auto flags = CheckThreadHealthFast(kRtControlConfig);
if (flags != ThreadHealthFlag::kOk) { /* 경고 로깅 */ }
```

> **주의**: `SCHED_FIFO` 적용에는 `realtime` 그룹 멤버십과 `/etc/security/limits.conf`의 `rtprio 99` 설정이 필요합니다.

---

### `ur5e_rt_base/logging/log_buffer.hpp`

500Hz RT 스레드와 로그 스레드 간 **단일-생산자 단일-소비자(SPSC) 링 버퍼**를 구현합니다. 최근 성능 최적화를 적용하여 업계 최고 수준의 초저지연성을 확보했습니다.

```cpp
template <std::size_t N>
class SpscLogBuffer {
 public:
  // RT 스레드: 절대 블로킹/할당 없음 — 버퍼 가득 시 false 반환
  [[nodiscard]] bool Push(const LogEntry& entry) noexcept;

  // 로그 스레드: 엔트리 없으면 false 반환
  [[nodiscard]] bool Pop(LogEntry& entry) noexcept;

  // 버퍼 가득 차서 드롭된 엔트리 수
  [[nodiscard]] uint64_t drop_count() const noexcept;
};
```

**설계 및 최적화 특성:**
- 크기 `N`은 반드시 **2의 거듭제곱**이어야 합니다 (기본값: 512).
- 모듈러 연산(`% N`) 대신 **비트 AND 연산**(`& (N - 1)`)을 사용하여 인덱스 래핑(Wrapping) 연산 파이프라인을 고속화했습니다.
- 생산자와 소비자가 서로의 인덱스를 무인증으로 계속 읽어와 발생하는 캐시 무효화(Cache Invalidation, False Sharing) 페널티를 막기 위해, 자신의 캐시에 상대방의 인덱스를 저장(`cached_tail_`, `cached_head_`)하여 **원자성 동기화 횟수를 획기적으로 감축**시켰습니다.
- C++17 `<new>` 헤더의 `std::hardware_destructive_interference_size`를 사용해 **하드웨어 아키텍처(x86, ARM 등)에 맞는 최적의 캐시 라인 크기**를 동적으로 컴파일-타임에 결정(`alignas(kCacheLineSize)`)합니다.
- `Push()`: RT 경로에서 절대 블로킹/동적 할당이 일어나지 않습니다.

**`LogEntry` 구조체** (DataLogger와 연계):

```cpp
struct LogEntry {
  double timestamp{0.0};
  // ── Timing ──
  double t_state_acquire_us{0.0};
  double t_compute_us{0.0};
  double t_publish_us{0.0};
  double t_total_us{0.0};
  double jitter_us{0.0};
  // ── Robot ──
  std::array<double, 6> goal_positions{};       // 최종 목표 (SetRobotTarget)
  std::array<double, 6> target_positions{};     // 궤적 보간 위치
  std::array<double, 6> target_velocities{};    // 궤적 보간 속도
  std::array<double, 6> actual_positions{};     // 실제 관절 위치
  std::array<double, 6> actual_velocities{};    // 실제 관절 속도
  // ── Hand ──
  bool hand_valid{false};
  std::array<float, 10> hand_goal_positions{};  // 핸드 최종 목표
  std::array<float, 10> hand_commands{};        // 핸드 명령
  std::array<float, 10> hand_actual_positions{};// 핸드 실제 위치
  std::array<float, 10> hand_actual_velocities{};// 핸드 실제 속도
  std::array<float, 44> hand_sensors{};         // 손 센서 (4×11)
  [[nodiscard]] double compute_time_us() const noexcept { return t_compute_us; }
};
```

**타입 별칭:**
```cpp
constexpr std::size_t kControlLogBufferCapacity = 512;  // ≈1초 @ 500Hz
using ControlLogBuffer = SpscLogBuffer<512>;
```

---

### `ur5e_rt_base/logging/data_logger.hpp`

비-RT 스레드에서 CSV 파일에 제어 데이터를 기록하는 로거입니다. v5.9.0부터 3개 CSV 파일로 분리 기록합니다.

```cpp
class DataLogger {
 public:
  /// 3개 CSV 파일로 분리 기록 (v5.9.0). 빈 경로 → 해당 로그 비활성화.
  DataLogger(const std::filesystem::path& timing_path,
             const std::filesystem::path& robot_path,
             const std::filesystem::path& hand_path);
  ~DataLogger();  // Flush() 자동 호출

  DataLogger(DataLogger&&) = default;
  DataLogger& operator=(DataLogger&&) = default;

  void LogEntry(const ur5e_rt_controller::LogEntry& entry);
  void DrainBuffer(ControlLogBuffer& buf);
  void Flush();
  [[nodiscard]] bool IsOpen() const;
};
```

**CSV 열 형식 (3파일 분리, v5.9.0):**

- **timing_log** (6 columns):
```
timestamp, t_state_acquire_us, t_compute_us, t_publish_us, t_total_us, jitter_us
```

- **robot_log** (31 columns):
```
timestamp, goal_pos_0..5, target_pos_0..5, target_vel_0..5, actual_pos_0..5, actual_vel_0..5
```

- **hand_log** (87 columns):
```
timestamp, hand_valid, hand_goal_pos_0..9, hand_cmd_0..9, hand_actual_pos_0..9, hand_actual_vel_0..9, baro_f{f}_{b}, tof_f{f}_{t}
```

---

### `ur5e_rt_base/logging/session_dir.hpp` (v5.10.0)

모든 C++ 패키지에서 공유하는 세션 디렉토리 관리 유틸리티입니다. `logging_data/YYMMDD_HHMM/` 형식의 세션 디렉토리 생성, 환경변수 기반 경로 해석, 오래된 세션 정리 기능을 제공합니다.

```cpp
namespace ur5e_rt_controller {

// 세션 타임스탬프 생성 (YYMMDD_HHMM 형식)
[[nodiscard]] inline std::string GenerateSessionTimestamp();

// 세션 디렉토리 내 표준 서브디렉토리 생성
// controller/, monitor/, hand/, sim/, plots/, motions/
inline void EnsureSessionSubdirs(const std::filesystem::path& session_dir);

// 세션 디렉토리 해석 (우선순위: UR5E_SESSION_DIR 환경변수 → 자체 생성)
[[nodiscard]] inline std::filesystem::path ResolveSessionDir(
    const std::string& fallback_logging_root);

// 오래된 세션 폴더 정리 (YYMMDD_HHMM 패턴, max_sessions 초과 시 삭제)
inline void CleanupOldSessions(
    const std::filesystem::path& logging_root, int max_sessions);

// 세션 디렉토리 목록 조회 (정렬된 YYMMDD_HHMM 폴더)
[[nodiscard]] inline std::vector<std::filesystem::path> ListSessionDirs(
    const std::filesystem::path& logging_root);

}  // namespace ur5e_rt_controller
```

**사용 예시:**

```cpp
#include "ur5e_rt_base/logging/session_dir.hpp"

// 패턴 1: 환경변수 또는 자체 생성 (노드 단독 실행 시)
auto session = urtc::ResolveSessionDir("~/ros2_ws/ur5e_ws/logging_data");
auto ctrl_dir = session / "controller";

// 패턴 2: 오래된 세션 정리
urtc::CleanupOldSessions(logging_root, 10);
```

---

## UDP 통신 모듈 (`udp/`)

실시간 UDP 송수신을 위한 헤더-전용 라이브러리입니다. RAII 소켓 래퍼, C++20 컨셉 기반 코덱 추상화, jthread 기반 송수신기 템플릿의 3계층으로 구성됩니다.

---

### `ur5e_rt_base/udp/udp_socket.hpp`

할당-없는(allocation-free) RAII UDP 소켓 래퍼입니다.

```cpp
class UdpSocket {
 public:
  UdpSocket() noexcept = default;
  ~UdpSocket();  // Close() 자동 호출

  // 복사/이동 불가 (raw fd 소유)
  UdpSocket(const UdpSocket&)            = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&&)                 = delete;
  UdpSocket& operator=(UdpSocket&&)      = delete;

  // ── 소켓 생명주기 ──
  [[nodiscard]] bool Open() noexcept;                                  // AF_INET SOCK_DGRAM 생성
  [[nodiscard]] bool Bind(int port) noexcept;                          // INADDR_ANY:port 바인드 (수신)
  [[nodiscard]] bool Connect(std::string_view target_ip, int port) noexcept;  // 대상 설정 (송신)
  void Close() noexcept;

  // ── 소켓 옵션 ──
  void SetRecvBufferSize(int bytes) noexcept;      // SO_RCVBUF
  void SetRecvTimeout(int timeout_ms) noexcept;    // SO_RCVTIMEO

  // ── I/O (할당 없음, noexcept) ──
  [[nodiscard]] ssize_t Recv(std::span<char> buf) noexcept;            // 수신 바이트 수 반환, 에러 시 -1
  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept;  // Connect()된 대상으로 전송

  // ── 접근자 ──
  [[nodiscard]] int  fd() const noexcept;
  [[nodiscard]] bool is_open() const noexcept;
};
```

**사용 모드:**
- **수신**: `Open()` → `Bind(port)` → `Recv()`
- **송신**: `Open()` → `Connect(ip, port)` → `Send()`

---

### `ur5e_rt_base/udp/udp_codec.hpp`

타입 안전한 도메인별 패킷 인코딩/디코딩을 위한 C++20 컨셉입니다.

```cpp
// Codec 타입 C가 만족해야 할 조건:
//   C::RecvPacket  — trivially_copyable packed 구조체 (수신 와이어 포맷)
//   C::SendPacket  — trivially_copyable packed 구조체 (송신 와이어 포맷)
//   C::State       — 디코딩된 애플리케이션-레벨 상태 타입
//   C::Decode(span<const char>, State&) → bool
template <typename C>
concept UdpPacketCodec = /* ... */;
```

**헬퍼 함수:**

```cpp
// raw 버퍼 → packed 구조체 디코딩 (memcpy 기반)
// buf 크기 < sizeof(T)이면 false 반환
template <typename T>
  requires std::is_trivially_copyable_v<T>
[[nodiscard]] bool DecodePacket(std::span<const char> buf, T& out) noexcept;

// packed 구조체 → raw 바이트 버퍼 인코딩 (memcpy 기반)
template <typename T>
  requires std::is_trivially_copyable_v<T>
void EncodePacket(const T& pkt, std::span<uint8_t, sizeof(T)> out) noexcept;
```

**도메인 코덱 구현 예시** (`ur5e_hand_udp` 패키지의 `hand_packets.hpp`):

```cpp
struct HandCodec {
  struct RecvPacket { /* packed 수신 패킷 */ };
  struct SendPacket { /* packed 송신 패킷 */ };
  using State = HandState;
  static bool Decode(std::span<const char> buf, State& out) noexcept;
};
static_assert(UdpPacketCodec<HandCodec>);  // 컨셉 검증
```

---

### `ur5e_rt_base/udp/udp_transceiver.hpp`

`UdpPacketCodec` 컨셉을 만족하는 코덱과 조합하여 양방향 UDP 통신을 제공하는 제네릭 템플릿입니다. 수신은 `std::jthread` 기반 자동 루프, 송신은 할당-없는 동기 호출입니다.

```cpp
template <typename Codec>
  requires UdpPacketCodec<Codec>
class UdpTransceiver {
 public:
  using RecvPacket    = typename Codec::RecvPacket;
  using SendPacket    = typename Codec::SendPacket;
  using State         = typename Codec::State;
  using StateCallback = std::function<void(const State&)>;

  explicit UdpTransceiver(int recv_port,
                          const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept;
  ~UdpTransceiver();  // Stop() 자동 호출

  // ── 수신 경로 ──
  [[nodiscard]] bool StartRecv();                   // 소켓 바인드 + jthread 시작
  void Stop() noexcept;                             // jthread 정지 요청 + 소켓 닫기
  void SetCallback(StateCallback cb) noexcept;      // 디코딩 성공 시 콜백
  [[nodiscard]] State GetLatestState() const;       // 스레드 안전 스냅샷 (mutex)
  [[nodiscard]] bool IsRunning() const noexcept;
  [[nodiscard]] std::size_t recv_count() const noexcept;

  // ── 송신 경로 ──
  [[nodiscard]] bool InitSend(std::string_view target_ip, int target_port);
  [[nodiscard]] bool Send(const SendPacket& pkt) noexcept;  // 할당 없는 전송
  [[nodiscard]] std::size_t send_count() const noexcept;
};
```

**내부 구현 세부:**
- 수신 소켓: `SO_RCVBUF=256KB`, `SO_RCVTIMEO=100ms` (stop_token 폴링용)
- 수신 스레드: `thread_cfg`에 따라 `ApplyThreadConfig()` 적용
- 상태 스냅샷: mutex 보호 `latest_state_`
- 송신 소켓: 별도 소켓으로 할당-없는 `Send()`

**사용 예시:**

```cpp
#include "ur5e_rt_base/udp/udp_transceiver.hpp"

// 코덱 정의 (UdpPacketCodec 컨셉 만족)
// HandCodec은 ur5e_hand_udp 패키지에서 정의
UdpTransceiver<HandCodec> transceiver(5000, kUdpRecvConfig);

// 수신 시작
transceiver.SetCallback([](const HandState& state) {
  // 새 패킷 수신 시 호출
});
transceiver.StartRecv();

// 송신 초기화 + 전송
transceiver.InitSend("192.168.1.20", 5001);
HandCodec::SendPacket pkt{/* ... */};
transceiver.Send(pkt);

// 최신 상태 조회 (스레드 안전)
auto state = transceiver.GetLatestState();
```

---

## 디지털 신호 필터 (`filters/`)

헤더-전용 필터 라이브러리입니다. 두 필터 모두 `Init()` 이후 모든 처리 메서드가 **`noexcept`** 이며 힙 할당이 없어 500Hz RT 루프에서 직접 사용할 수 있습니다.

---

### `ur5e_rt_base/filters/bessel_filter.hpp`

#### 개요

4차 Bessel 저역통과 필터를 2개의 biquad 직렬 연결(Direct Form II Transposed)로 구현합니다.

**로봇 제어에 Bessel 필터를 선택하는 이유:**
Bessel 필터는 **최대 선형 군지연(Maximally Flat Group Delay)** 특성을 가집니다. 즉, 모든 주파수 성분이 동일한 시간 지연을 경험하므로 신호의 파형 형태가 보존됩니다. 관절 궤적 필터링 시 위상 왜곡 없이 매끄러운 출력을 얻을 수 있습니다.

#### 아날로그 프로토타입 (4차, -3 dB @ ω = 1 rad/s)

| 켤레 쌍 | 자연 주파수 ω₀ | Q 계수 |
|---------|---------------|--------|
| 쌍 1 | 1.4301691433 | 0.5219356105 |
| 쌍 2 | 1.6033574829 | 0.8055342053 |

디지털 변환: 컷오프 prewarping 포함 쌍선형 변환 → 디지털 -3 dB 점이 정확히 `cutoff_hz`에 위치합니다.

#### API

```cpp
template <std::size_t N>
class BesselFilterN {
 public:
  // 필터 초기화 — cutoff_hz < sample_rate_hz/2 이어야 함
  // 위반 시 std::invalid_argument 예외
  void Init(double cutoff_hz, double sample_rate_hz);

  // 지연 소자 초기화 (E-STOP 또는 재시작 후 호출)
  void Reset() noexcept;

  // N채널 입력 필터링 (noexcept — RT 안전)
  [[nodiscard]] std::array<double, N> Apply(
      const std::array<double, N>& input) noexcept;

  // 단일 채널 스칼라 버전
  [[nodiscard]] double ApplyScalar(double x,
                                   std::size_t channel = 0) noexcept;

  // 파라미터 접근자
  [[nodiscard]] bool   initialized()    const noexcept;
  [[nodiscard]] double cutoff_hz()      const noexcept;
  [[nodiscard]] double sample_rate_hz() const noexcept;
};

using BesselFilter6  = BesselFilterN<6>;   // 6-DOF 로봇 관절
using BesselFilter11 = BesselFilterN<11>;  // 11-DOF 손 관절
using BesselFilter1  = BesselFilterN<1>;   // 단일 채널 스칼라
```

#### 사용 예시

```cpp
#include "ur5e_rt_base/filters/bessel_filter.hpp"
using namespace ur5e_rt_controller;

// 초기화 (노드 생성 시)
BesselFilter6 lpf;
lpf.Init(100.0, 500.0);  // 100Hz 컷오프, 500Hz 샘플레이트

// E-STOP 해제 후 상태 초기화
lpf.Reset();

// 500Hz RT 루프 내
std::array<double,6> filtered = lpf.Apply(raw_positions);
```

#### 파라미터 튜닝 가이드

| 상황 | 권장 컷오프 |
|------|-----------|
| 빠른 추종 필요 (고속 이동) | 150–200 Hz |
| 일반 작업 (기본값) | 80–120 Hz |
| 노이즈가 심한 센서 | 40–60 Hz |
| 저속 정밀 작업 | 20–40 Hz |

> 컷오프를 낮출수록 필터링 효과가 강해지지만 위상 지연이 증가합니다.

---

### `ur5e_rt_base/filters/kalman_filter.hpp`

#### 개요

상수-속도(Constant Velocity) 운동 모델 기반 이산-시간 Kalman 필터입니다. **위치 측정만으로 위치와 속도를 동시에 추정**합니다. 미분 없이 노이즈가 없는 속도 추정값을 얻을 수 있어 PD 제어기의 D항 계산에 활용할 수 있습니다.

#### 수학적 모델

**상태 벡터** (채널당 2×1):
```
x = [position, velocity]ᵀ
```

**전이 행렬** (dt = 샘플 주기):
```
F = | 1  dt |
    | 0   1 |
```

**관측 행렬** (위치만 측정):
```
H = [1  0]
```

**잡음 공분산**:
```
Q = diag(q_pos, q_vel)   (프로세스 잡음)
R = r                    (측정 잡음, 스칼라)
```

**예측 단계**:
```
x̂⁻ = F · x̂
P⁻  = F · P · Fᵀ + Q
```

**업데이트 단계**:
```
S  = H · P⁻ · Hᵀ + R          (혁신 공분산)
K  = P⁻ · Hᵀ / S              (칼만 이득, 2×1)
x̂  = x̂⁻ + K · (z − H · x̂⁻)
P  = (I − K·H) · P⁻
```

**구현 특성**: 2×2 공분산 행렬을 `{p00, p01, p11}` 스칼라 3개로 저장 — Eigen 의존성 없음, 힙 할당 없음.

#### API

```cpp
template <std::size_t N>
class KalmanFilterN {
 public:
  struct Params {
    double q_pos{1e-3};  // 위치 프로세스 잡음 [rad²]
    double q_vel{1e-2};  // 속도 프로세스 잡음 [(rad/s)²]
    double r{1e-1};      // 측정 잡음 [rad²]
    double dt{0.002};    // 샘플 주기 [s]
  };

  // 초기화 — 잘못된 파라미터 시 std::invalid_argument 예외
  void Init(double q_pos, double q_vel, double r, double dt);
  void Init(const Params& p);

  // 상태 초기화
  void Reset() noexcept;
  void SetInitialPositions(const std::array<double, N>& positions) noexcept;

  // 예측 단계 — 매 제어 틱에 호출 (noexcept)
  void Predict() noexcept;

  // 업데이트 단계 — 측정값 융합 (noexcept)
  [[nodiscard]] std::array<double, N> Update(
      const std::array<double, N>& measurements) noexcept;

  // 예측 + 업데이트 단일 호출 (noexcept)
  [[nodiscard]] std::array<double, N> PredictAndUpdate(
      const std::array<double, N>& measurements) noexcept;

  // 상태 접근자 (noexcept)
  [[nodiscard]] double position(std::size_t i)          const noexcept;
  [[nodiscard]] double velocity(std::size_t i)          const noexcept;  // 미분 없는 속도
  [[nodiscard]] std::array<double, N> positions()       const noexcept;
  [[nodiscard]] std::array<double, N> velocities()      const noexcept;
  [[nodiscard]] double position_variance(std::size_t i) const noexcept;  // P₀₀
  [[nodiscard]] double kalman_gain(std::size_t i)       const noexcept;  // 진단용
};

using KalmanFilter6  = KalmanFilterN<6>;   // 6-DOF 로봇 관절
using KalmanFilter11 = KalmanFilterN<11>;  // 11-DOF 손 관절
using KalmanFilter1  = KalmanFilterN<1>;   // 단일 채널 스칼라
```

#### 사용 예시

```cpp
#include "ur5e_rt_base/filters/kalman_filter.hpp"
using namespace ur5e_rt_controller;

// 초기화 (노드 생성 시)
KalmanFilter6 kf;
kf.Init(0.001, 0.01, 0.1, 0.002);     // q_pos, q_vel, r, dt=2ms

// 초기 위치 시드 (시작 과도 현상 방지)
kf.SetInitialPositions(initial_positions);

// 500Hz RT 루프 내 — 패턴 1: 합산 호출
auto filtered_pos = kf.PredictAndUpdate(raw_positions);
double vel_j0     = kf.velocity(0);   // 관절 0 속도 (미분 없음)

// 패턴 2: 분리 호출 (측정값 없는 틱에서 Predict만 호출 가능)
kf.Predict();
if (new_measurement_available) {
  auto pos = kf.Update(raw_positions);
}

// PD 제어기 D항에 칼만 속도 활용
for (size_t i = 0; i < 6; ++i) {
  double err     = target[i] - kf.position(i);
  double vel_err = 0.0 - kf.velocity(i);       // 목표 속도 = 0 (정지 목표)
  cmd[i] = kp * err + kd * vel_err;
}
```

#### 파라미터 튜닝 가이드

| 파라미터 | 증가 효과 | 감소 효과 |
|---------|----------|----------|
| `q_pos` | 모델 예측 불신 → 센서 더 신뢰 | 모델 더 신뢰 → 더 매끄러운 위치 |
| `q_vel` | 빠른 속도 변화 허용 → 빠른 추종 | 속도 변화 억제 → 더 매끄러운 속도 |
| `r` | 센서 불신 → 모델 더 신뢰 → 더 매끄러운 출력 | 센서 신뢰 → 빠른 응답 |

**권장 초기값 (UR5e 500Hz):**

| 용도 | `q_pos` | `q_vel` | `r` |
|------|---------|---------|-----|
| 위치 노이즈 필터링 | `1e-4` | `1e-2` | `1e-2` |
| 속도 추정 (부드럽게) | `1e-3` | `1e-3` | `5e-2` |
| 빠른 동작 추종 | `1e-2` | `1e-1` | `1e-3` |

---

### Bessel vs Kalman 필터 선택 기준

| 항목 | Bessel | Kalman |
|------|--------|--------|
| 주 목적 | 위상 왜곡 없는 노이즈 제거 | 위치 + 속도 동시 추정 |
| 파라미터 직관성 | 높음 (컷오프 Hz) | 중간 (Q/R 비율) |
| 속도 추정 | 별도 미분 필요 | 내장 |
| 급격한 위치 변화 | 지연 발생 | `q_vel` 조정으로 대응 |
| 센서 잡음 명시 | 불가 (암묵적) | 가능 (`r` 파라미터) |
| 연산량 | 채널당 4 곱셈+덧셈 | 채널당 ~15 곱셈+덧셈 |

> **권장**: 단순 궤적 평활화 → **Bessel**, PD 제어기 속도 추정 포함 → **Kalman**

---

## 빌드

```bash
# 독립 빌드 (다른 패키지 불필요)
cd ~/ros2_ws/ur5e_ws
colcon build --packages-select ur5e_rt_base
source install/setup.bash
```

이 패키지는 **헤더-전용**이므로 컴파일 단계가 없습니다. `ament_cmake`가 헤더를 `include/` 경로로 내보냅니다.

---

## 다른 패키지에서 사용하기

**CMakeLists.txt:**
```cmake
find_package(ur5e_rt_base REQUIRED)
target_include_directories(my_target PRIVATE
  ${ur5e_rt_base_INCLUDE_DIRS}
)
```

**package.xml:**
```xml
<depend>ur5e_rt_base</depend>
```

**C++ 코드:**
```cpp
#include "ur5e_rt_base/types/types.hpp"
#include "ur5e_rt_base/threading/thread_config.hpp"
#include "ur5e_rt_base/threading/thread_utils.hpp"
#include "ur5e_rt_base/logging/log_buffer.hpp"
#include "ur5e_rt_base/logging/data_logger.hpp"
#include "ur5e_rt_base/logging/session_dir.hpp"
#include "ur5e_rt_base/udp/udp_socket.hpp"
#include "ur5e_rt_base/udp/udp_codec.hpp"
#include "ur5e_rt_base/udp/udp_transceiver.hpp"
#include "ur5e_rt_base/filters/bessel_filter.hpp"
#include "ur5e_rt_base/filters/kalman_filter.hpp"
```

---

## RT 권한 요구사항

`SCHED_FIFO` 스케줄링을 사용하려면:

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필요
ulimit -r  # 99 출력 확인
ulimit -l  # unlimited 출력 확인
```

---

## 라이선스

MIT License — 자세한 내용은 최상위 디렉터리의 LICENSE 파일을 참조하세요.
