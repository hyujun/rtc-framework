# ur5e_rt_base

UR5e RT Controller 스택의 **공유 기반 패키지** — 모든 패키지가 공통으로 사용하는 타입 정의, 스레드 유틸리티, 잠금-없는(lock-free) 로깅 인프라를 제공하는 헤더-전용(header-only) 라이브러리입니다.

## 개요

```
ur5e_rt_base (header-only)
    ├── types.hpp                  ← 공유 데이터 구조체 + 상수
    ├── thread_config.hpp          ← RT 스레드 설정 구조체 + 사전 정의 상수
    ├── thread_utils.hpp           ← ApplyThreadConfig(), VerifyThreadConfig()
    ├── log_buffer.hpp             ← SPSC 링 버퍼 (RT → 로그 스레드, 잠금-없음)
    └── data_logger.hpp            ← 비-RT CSV 로거
```

이 패키지는 실행 파일을 포함하지 않으며, `install(DIRECTORY include/)` 만으로 헤더를 내보냅니다.

**의존성 그래프 내 위치:**

```
ur5e_rt_base   ← (독립 — 아무 ROS2 패키지에도 의존하지 않음)
    ↑
    ├── ur5e_rt_controller
    └── ur5e_hand_udp
```

---

## 헤더 파일 설명

### `ur5e_rt_base/types.hpp`

500Hz 제어 루프 전체에서 공유하는 데이터 구조체와 컴파일-시간 상수를 정의합니다.

#### 컴파일-시간 상수

| 상수 | 값 | 설명 |
|------|----|------|
| `kNumRobotJoints` | `6` | UR5e 로봇 관절 수 |
| `kNumHandJoints` | `11` | 손 모터 수 |
| `kNumHandSensors` | `44` | 손 센서 수 (관절당 4개 × 11) |

#### `NonNegativeFloat` 컨셉 (C++20)

```cpp
template <typename T>
concept NonNegativeFloat = std::floating_point<T>;
```

#### 데이터 구조체

```cpp
namespace ur5e_rt_controller {

struct RobotState {
  std::array<double, 6>  positions{};    // 관절 위치 (rad)
  std::array<double, 6>  velocities{};   // 관절 속도 (rad/s)
  std::array<double, 3>  tcp_position{}; // TCP 위치 (m, x/y/z)
  double dt{0.002};                      // 제어 주기 (s)
  uint64_t iteration{0};                 // 누적 제어 반복 횟수
};

struct HandState {
  std::array<double, 11> motor_positions{};   // 모터 위치
  std::array<double, 11> motor_velocities{};  // 모터 속도
  std::array<double, 11> motor_currents{};    // 모터 전류
  std::array<double, 44> sensor_data{};       // 센서 데이터 (관절당 4개)
  bool valid{false};                          // 데이터 유효성 플래그
};

struct ControllerState {
  RobotState robot{};
  HandState  hand{};
  double     dt{0.002};
  uint64_t   iteration{0};
};

struct ControllerOutput {
  std::array<double, 6>  robot_commands{};  // 로봇 관절 명령 (rad)
  std::array<double, 11> hand_commands{};   // 손 명령 (0.0–1.0 정규화)
  bool valid{true};
};

} // namespace ur5e_rt_controller
```

---

### `ur5e_rt_base/thread_config.hpp`

RT 스레드 설정을 위한 `ThreadConfig` 구조체와 6코어 시스템용 사전 정의 상수를 제공합니다.

```cpp
struct ThreadConfig {
  int         cpu_core;        // CPU 코어 번호 (0-based)
  int         sched_policy;   // SCHED_FIFO / SCHED_RR / SCHED_OTHER
  int         sched_priority; // 우선순위 (FIFO: 1–99)
  int         nice_value;     // nice 값 (SCHED_OTHER 전용, -20~19)
  std::string name;           // 스레드 이름 (최대 15자)
};
```

#### 사전 정의 ThreadConfig 상수

**6코어 시스템** (Core 0-1: OS/DDS/IRQ, Core 2-5: RT 전용):

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|----------|------|
| `kRtControlConfig` | 2 | `SCHED_FIFO` | 90 | 500Hz 제어 루프 + 50Hz E-STOP 감시 |
| `kSensorConfig` | 3 | `SCHED_FIFO` | 70 | 센서 구독 스레드 (전용) |
| `kUdpRecvConfig` | **5** | `SCHED_FIFO` | 65 | UDP 수신 스레드 (sensor_io 경합 방지) |
| `kLoggingConfig` | 4 | `SCHED_OTHER` | nice -5 | CSV 로깅 스레드 |
| `kAuxConfig` | 5 | `SCHED_OTHER` | 0 | 보조 스레드 (이벤트 기반, 경량) |

**8코어 시스템** (Core 0-1: OS/DDS/IRQ, Core 2-6: RT 전용, Core 7: 예비):

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|----------|------|
| `kRtControlConfig8Core` | 2 | `SCHED_FIFO` | 90 | 500Hz 제어 루프 |
| `kSensorConfig8Core` | 3 | `SCHED_FIFO` | 70 | 센서 구독 스레드 |
| `kUdpRecvConfig8Core` | **4** | `SCHED_FIFO` | 65 | UDP 수신 스레드 (전용 코어) |
| `kLoggingConfig8Core` | 5 | `SCHED_OTHER` | nice -5 | CSV 로깅 스레드 |
| `kAuxConfig8Core` | 6 | `SCHED_OTHER` | 0 | 보조 스레드 |

**4코어 폴백** (Core 0: OS/DDS/IRQ, Core 1-3: RT):

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|----------|------|
| `kRtControlConfig4Core` | 1 | `SCHED_FIFO` | 90 | 500Hz 제어 루프 |
| `kSensorConfig4Core` | 2 | `SCHED_FIFO` | 70 | 센서 구독 스레드 |
| `kUdpRecvConfig4Core` | 2 | `SCHED_FIFO` | 65 | UDP 수신 (sensor_io 코어 공유 불가피) |
| `kLoggingConfig4Core` | 3 | `SCHED_OTHER` | nice -5 | CSV 로깅 스레드 |

---

### `ur5e_rt_base/thread_utils.hpp`

RT 스레드 설정을 적용하고 검증하는 유틸리티 함수와, 런타임 CPU 수에 따라 최적 config 집합을 자동 선택하는 함수를 제공합니다.

```cpp
// CPU 친화성, 스케줄러 정책, 우선순위, 스레드 이름을 한 번에 적용
// 권한 부족 시 false 반환 (노드는 SCHED_OTHER로 계속 동작)
[[nodiscard]] bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept;

// 현재 스레드의 CPU 친화성, 스케줄러, nice 값, 이름을 문자열로 반환 (디버깅용)
std::string VerifyThreadConfig() noexcept;

// 지연 측정값 벡터에서 {min, max, avg} 튜플 반환
std::tuple<double, double, double> GetThreadStats(
    const std::vector<double>& latencies_us) noexcept;

// 온라인 논리 CPU 수 반환
int GetOnlineCpuCount() noexcept;

// 모든 스레드의 ThreadConfig를 하나로 묶은 구조체
struct SystemThreadConfigs {
  ThreadConfig rt_control;
  ThreadConfig sensor;
  ThreadConfig udp_recv;   // hand UDP 수신기 (sensor_io와 분리)
  ThreadConfig logging;
  ThreadConfig aux;
};

// 런타임 CPU 수에 따라 최적 ThreadConfig 집합 자동 반환
// ≥8코어 → 8코어 레이아웃 / ≥6코어 → 6코어 레이아웃 / <6코어 → 4코어 폴백
SystemThreadConfigs SelectThreadConfigs() noexcept;
```

**사용 예시:**

```cpp
#include "ur5e_rt_base/thread_utils.hpp"
#include "ur5e_rt_base/thread_config.hpp"

// 개별 스레드에 config 직접 적용
auto t = std::thread([&]() {
  if (!ApplyThreadConfig(ur5e_rt_controller::kRtControlConfig)) {
    RCLCPP_WARN(logger, "RT 설정 실패 — SCHED_OTHER로 동작 (지터 증가)");
  }
  // ... 제어 루프
});

// 런타임 CPU 수 자동 감지 후 최적 config 집합 선택
auto cfgs = ur5e_rt_controller::SelectThreadConfigs();
// cfgs.rt_control, cfgs.sensor, cfgs.udp_recv, cfgs.logging, cfgs.aux 사용
```

> **주의**: `SCHED_FIFO` 적용에는 `realtime` 그룹 멤버십과 `/etc/security/limits.conf`의 `rtprio 99` 설정이 필요합니다.

---

### `ur5e_rt_base/log_buffer.hpp`

500Hz RT 스레드와 로그 스레드 간 **단일-생산자 단일-소비자(SPSC) 링 버퍼**를 구현합니다.

```cpp
template <typename T, std::size_t N>
class SpscLogBuffer {
 public:
  // RT 스레드: 절대 블로킹/할당 없음 — 버퍼 가득 시 false 반환
  bool Push(const T& entry) noexcept;

  // 로그 스레드: 엔트리 없으면 false 반환
  bool Pop(T& entry) noexcept;
};
```

**설계 특성:**
- 크기 `N`은 반드시 2의 거듭제곱 (기본값: 512)
- `std::atomic<std::size_t>` head/tail — 뮤텍스 없음
- `Push()`: RT 경로에서 절대 블로킹/동적 할당 없음
- 버퍼 가득 시 오래된 엔트리 드롭 (최신 우선)

**`LogEntry` 구조체** (DataLogger와 연계):

```cpp
struct LogEntry {
  double    timestamp{};
  RobotState robot_state{};
  std::array<double, 6> target_positions{};
  std::array<double, 6> commands{};
  double    compute_time_us{};  // Compute() 실행 시간 (마이크로초)
};
```

---

### `ur5e_rt_base/data_logger.hpp`

비-RT 스레드에서 CSV 파일에 제어 데이터를 기록하는 로거입니다.

```cpp
class DataLogger {
 public:
  explicit DataLogger(std::string_view path);

  // 이동 전용 (복사 불가)
  DataLogger(DataLogger&&) noexcept;

  // SpscLogBuffer에서 드레인하여 CSV 기록
  void DrainAndWrite(SpscLogBuffer<LogEntry, 512>& buffer);

  // 파일 경로 반환
  [[nodiscard]] std::string_view path() const noexcept;
};
```

**CSV 열 형식:**
```
timestamp, current_pos_0..5, target_pos_0..5, command_0..5, compute_time_us
```

기본 경로: `/tmp/ur5e_control_log.csv`

---

## 빌드

```bash
# 독립 빌드 (다른 패키지 불필요)
cd ~/ur_ws
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
#include "ur5e_rt_base/types.hpp"
#include "ur5e_rt_base/thread_config.hpp"
#include "ur5e_rt_base/thread_utils.hpp"
#include "ur5e_rt_base/log_buffer.hpp"
#include "ur5e_rt_base/data_logger.hpp"
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
