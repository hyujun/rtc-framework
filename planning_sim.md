# MuJoCo 시뮬레이터 통합 계획

**작성일:** 2026-03-03
**수정일:** 2026-03-03
**버전:** 1.1.0
**목표:** 기존 ROS2 UR5e RT 제어기를 물리 로봇 없이 MuJoCo 시뮬레이터로 검증할 수 있는 환경 구축

---

## 1. 개요 및 목표

### 1.1 동기

현재 `custom_controller.cpp`는 실제 UR5e 로봇과 통신하는 구조로, 물리 로봇 없이 제어 알고리즘을 개발·검증하기 어렵다. MuJoCo 시뮬레이터를 ROS2 노드로 래핑하여 기존 제어기 코드를 **변경 없이** 시뮬레이션 환경에서 실행할 수 있게 한다.

### 1.2 핵심 설계 원칙

- **투명한 ROS2 인터페이스**: 시뮬레이터 노드는 실제 UR 드라이버와 동일한 토픽을 게시/구독하여 제어기 코드 수정 불필요
- **기존 아키텍처 존중**: 현재 4-스레드 멀티 executor 구조, PD/OSC/CLIK 제어기를 그대로 활용
- **비실시간 우선 설계 (Non-RT First)**: 시뮬레이션 환경에서는 실시간 동기화가 불필요하다.
  시뮬레이터는 기본적으로 벽시계와 동기화하지 않고 **가능한 최대 속도로 실행**한다.
  이를 통해 알고리즘 검증 속도를 높이고, 실시간 커널 없이도 정확한 동작 분석이 가능하다.
- **제어기 연산 시간 분석**: 시뮬레이션 구동 중 각 제어기의 `Compute()` 실행 시간을
  측정·기록하여 실제 500Hz(2ms 예산) 환경에서의 실현 가능성을 정량적으로 평가한다.
- **단계적 구현**: 기본 관절 제어 → 시각화 → 핸드 시뮬레이션 순으로 확장

### 1.3 ROS2 토픽 매핑

```
[custom_controller 노드]              [mujoco_simulator 노드]
  Subscribe: /joint_states     ←←←  Publish:  /joint_states
  Publish: /forward_position_      →→→  Subscribe: /forward_position_
           controller/commands               controller/commands
  Subscribe: /hand/joint_states ←←← Publish:  /hand/joint_states (Phase 4)
```

---

## 2. 시스템 아키텍처

### 2.1 전체 구성도

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 DDS 레이어                          │
│                                                             │
│  /joint_states ──────────────────────────────────────────► │
│  /forward_position_controller/commands ◄──────────────────── │
│  /target_joint_positions ────────────────────────────────► │
│  /system/estop_status ◄───────────────────────────────────  │
│  /sim/timing_stats ◄──────────────────────────────────────  │  ← NEW
└─────────────────────────────────────────────────────────────┘
         ▲                                    ▲
         │                                    │
┌────────┴────────┐                ┌──────────┴──────────┐
│  MuJoCo         │                │  custom_controller  │
│  Simulator Node │                │  (기존 노드,         │
│                 │                │   변경 없음)          │
│  - UR5e MJCF 로드│               │                     │
│  - 물리 시뮬      │               │  - PDController      │
│  - 상태 게시      │               │  - OSC/CLIK 제어기    │
│  - 명령 수신      │               │  - E-STOP 시스템     │
│  - 뷰어 스레드    │               │  - 데이터 로깅        │
│  - 연산 시간 측정 │               │  - 연산 시간 측정     │  ← NEW
└─────────────────┘                └─────────────────────┘
```

### 2.2 MuJoCo 시뮬레이터 노드 스레드 구조

시뮬레이션은 **비실시간(Non-RT)이 기본 모드**이다. 실시간 스레드 우선순위(`SCHED_FIFO`)를 사용하지
않으며, CPU 어피니티도 선택적으로 적용한다.

```
메인 스레드 (SCHED_OTHER)
  └─ rclcpp::init, 노드 생성, executor spin

시뮬레이션 스레드 (SCHED_OTHER, [선택] Core 0)
  [free_run 모드]  mj_step()을 타이밍 제약 없이 반복 실행
  [sync_step 모드] 상태 게시 → 명령 대기 → mj_step() 순서로 1:1 동기화
  └─ /joint_states 게시 (매 시뮬레이션 스텝, 또는 N 스텝마다)
  └─ ControllerTimingProfiler 기록

수신 스레드 (SCHED_OTHER, [선택] Core 1)
  └─ /forward_position_controller/commands 구독
  └─ ctrl 버퍼 업데이트 (sync_step 모드: condition_variable 신호)

뷰어 스레드 (SCHED_OTHER, 코어 자동 선택)
  └─ MuJoCo passive viewer @ 60Hz
  └─ headless 모드에서 비활성화
```

### 2.3 시뮬레이션 모드 비교

| 항목 | `free_run` | `sync_step` |
|------|-----------|------------|
| 진행 방식 | 최대 속도, 타이밍 무관 | 제어기 응답 1회 수신 후 1 step 진행 |
| 시뮬레이션 속도 | 실시간의 수십~수백 배 | 제어기 연산 시간에 의해 결정 |
| 제어기 연산 시간 측정 | 간접적 (wall clock) | **직접 측정 (step latency = Compute 시간)** |
| E-STOP 워치독 | 비활성화 권장 | 스텝 타임아웃 기반으로 대체 |
| 주요 용도 | 알고리즘 검증, 궤적 생성 | 제어기 성능 분석, 실배포 전 검증 |

> **권장**: 제어 알고리즘 검증은 `free_run`, 실배포 가능성 평가는 `sync_step` 모드 사용

---

## 3. 구현 파일 목록

### 3.1 새로 생성할 파일

```
ur5e_rt_controller/
├── models/
│   └── ur5e/
│       ├── ur5e.xml              # UR5e MuJoCo MJCF 모델 (MuJoCo Menagerie)
│       ├── assets/               # 메시 파일 (.obj, .stl)
│       └── scene.xml             # 월드 + 로봇 씬 조합
│
├── include/ur5e_rt_controller/
│   ├── mujoco_simulator.hpp      # MuJoCo 래퍼 클래스
│   └── controller_timing_profiler.hpp   # 제어기 연산 시간 분석기 ← NEW
│
├── src/
│   └── mujoco_simulator_node.cpp # ROS2 노드 진입점
│
├── config/
│   └── mujoco_simulator.yaml     # 시뮬레이터 파라미터
│
└── launch/
    └── mujoco_sim.launch.py      # 시뮬레이션 전용 런치 파일
```

### 3.2 수정할 기존 파일

| 파일 | 수정 내용 |
|------|-----------|
| `CMakeLists.txt` | MuJoCo 의존성 추가, `mujoco_simulator_node` 실행파일 추가 |
| `package.xml` | `<depend>` 항목에 MuJoCo 관련 패키지 추가 |
| `include/.../log_buffer.hpp` | `LogEntry`에 `compute_time_us` 필드 추가 (선택) |

---

## 4. 단계별 구현 계획

---

### Phase 1: 환경 구성 및 UR5e MJCF 모델 준비

#### 1.1 MuJoCo 설치

```bash
# Python 바인딩 설치 (개발·검증용)
pip install mujoco

# C++ 헤더/라이브러리 설치 (프로덕션 빌드용)
# MuJoCo 3.x 릴리스 tarball 다운로드 후 /opt/mujoco에 배치
# https://github.com/google-deepmind/mujoco/releases

# CMakeLists.txt에 추가할 내용:
# find_package(mujoco REQUIRED)
# target_link_libraries(mujoco_simulator_node mujoco::mujoco)
```

#### 1.2 UR5e MJCF 모델 확보

MuJoCo Menagerie 프로젝트에서 공식 UR5e 모델을 가져온다.

```bash
# 방법 A: MuJoCo Menagerie 클론
git clone https://github.com/google-deepmind/mujoco_menagerie.git /tmp/menagerie
cp -r /tmp/menagerie/universal_robots_ur5e models/ur5e/

# 방법 B: 직접 URDF → MJCF 변환
# ROS2 ur_description 패키지의 URDF를 MuJoCo compile로 변환
```

**`models/ur5e/scene.xml` 구조:**

```xml
<mujoco model="ur5e_scene">
  <option timestep="0.002" gravity="0 0 -9.81"/>  <!-- 시뮬레이션 dt = 2ms -->

  <include file="ur5e.xml"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="2 2 0.1" material="groundplane"/>
  </worldbody>
</mujoco>
```

> **주의**: `timestep="0.002"`는 시뮬레이션의 논리적 dt를 정의한다.
> 비실시간 모드에서는 이 값이 벽시계 2ms를 의미하지 않는다 — 제어기에 전달하는 `state.robot.dt`
> 는 항상 이 값(0.002s)을 사용해야 한다.

#### 1.3 관절 이름-인덱스 매핑 확인

UR5e MJCF 관절 이름과 ROS2 `/joint_states` 관절 순서를 정렬:

| 인덱스 | ROS2 joint_names | MuJoCo qpos 인덱스 | MJCF 관절명 |
|--------|------------------|--------------------|------------|
| 0 | `shoulder_pan_joint` | 0 | `shoulder_pan` |
| 1 | `shoulder_lift_joint` | 1 | `shoulder_lift` |
| 2 | `elbow_joint` | 2 | `elbow` |
| 3 | `wrist_1_joint` | 3 | `wrist_1` |
| 4 | `wrist_2_joint` | 4 | `wrist_2` |
| 5 | `wrist_3_joint` | 5 | `wrist_3` |

> **런타임 검증 필수**: 정적 매핑 대신 `mj_name2id(model, mjOBJ_JOINT, "shoulder_pan")`으로
> 인덱스를 동적으로 조회하여, MJCF 모델 변경 시 조용한 오작동을 방지한다.

---

### Phase 2: MuJoCo 래퍼 클래스 구현

**파일:** `include/ur5e_rt_controller/mujoco_simulator.hpp`

```cpp
#pragma once

#include <mujoco/mujoco.h>
#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace urtc {

/// MuJoCo 물리 시뮬레이터 래퍼
class MuJoCoSimulator {
public:
  /// 시뮬레이션 실행 모드
  enum class SimMode {
    kFreeRun,   // 최대 속도로 실행 (알고리즘 검증용)
    kSyncStep,  // 명령 수신 후 1 step 진행 (연산 시간 측정용)
  };

  struct Config {
    std::string model_path;
    SimMode     mode{SimMode::kFreeRun};   // 기본값: 비실시간 자유 실행
    bool        enable_viewer{true};
    double      publish_decimation{1.0};  // N 스텝마다 /joint_states 게시 (free_run용)
    double      sync_timeout_ms{50.0};   // sync_step 명령 대기 타임아웃 (ms)
    std::array<double, 6> initial_qpos{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  };

  using StateCallback = std::function<void(
      const std::array<double, 6>& positions,
      const std::array<double, 6>& velocities)>;

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  MuJoCoSimulator(const MuJoCoSimulator&)            = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;

  [[nodiscard]] bool Initialize() noexcept;
  void Start() noexcept;
  void Stop() noexcept;

  /// 관절 위치 명령 설정 (ROS2 콜백에서 호출)
  /// sync_step 모드: 내부 condition_variable 신호 발생
  void SetCommand(const std::array<double, 6>& cmd) noexcept;
  void SetStateCallback(StateCallback cb) noexcept;

  [[nodiscard]] std::array<double, 6> GetPositions()  const noexcept;
  [[nodiscard]] std::array<double, 6> GetVelocities() const noexcept;

  [[nodiscard]] bool     IsRunning()  const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount()  const noexcept { return step_count_.load(); }
  [[nodiscard]] double   SimTimeSec() const noexcept { return sim_time_sec_.load(); }

private:
  Config cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};
  std::atomic<double>   sim_time_sec_{0.0};

  // 명령 버퍼: atomic flag로 빠른 경로 보호
  mutable std::mutex    cmd_mutex_;
  std::atomic<bool>     cmd_pending_{false};  // lock-free fast path
  std::array<double, 6> pending_cmd_{};

  // sync_step 모드 전용 동기화
  std::mutex              sync_mutex_;
  std::condition_variable sync_cv_;

  mutable std::mutex    state_mutex_;
  std::array<double, 6> latest_positions_{};
  std::array<double, 6> latest_velocities_{};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  void SimLoopFreeRun(std::stop_token stop) noexcept;
  void SimLoopSyncStep(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
  void ApplyCommand() noexcept;
  void ReadState() noexcept;
  void ResolveJointIndices() noexcept;  // mj_name2id 기반 인덱스 검증

  std::array<int, 6> joint_qpos_indices_{};  // MJCF 관절명 → qpos 인덱스
};

}  // namespace urtc
```

#### 2.1 SimLoop 구현 — `free_run` 모드

```cpp
void MuJoCoSimulator::SimLoopFreeRun(std::stop_token stop) noexcept {
  uint64_t step = 0;
  const int decim = static_cast<int>(cfg_.publish_decimation);

  while (!stop.stop_requested()) {
    // 명령 적용 (atomic flag fast path — 뮤텍스 락은 dirty일 때만)
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }

    mj_step(model_, data_);
    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    // N 스텝마다 상태 게시 (DDS 트래픽 조절)
    if (step % decim == 0) {
      ReadState();
      if (state_cb_) {
        std::lock_guard lock(state_mutex_);
        state_cb_(latest_positions_, latest_velocities_);
      }
    }
  }
}
```

#### 2.2 SimLoop 구현 — `sync_step` 모드

`sync_step` 모드에서는 시뮬레이터가 제어기 명령 수신을 기다린 후 1 step 진행한다.
**1 step 지연 시간 ≈ 제어기의 `Compute()` 실행 시간**이 되어 직접적인 성능 측정이 가능하다.

```cpp
void MuJoCoSimulator::SimLoopSyncStep(std::stop_token stop) noexcept {
  const auto timeout = std::chrono::milliseconds(
      static_cast<int>(cfg_.sync_timeout_ms));

  while (!stop.stop_requested()) {
    // 1. 현재 상태 읽기 및 게시
    ReadState();
    if (state_cb_) {
      std::lock_guard lock(state_mutex_);
      state_cb_(latest_positions_, latest_velocities_);
    }

    // 2. 제어기 명령 대기 (타임아웃 내 미수신 시 이전 명령 유지)
    {
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this] {
        return cmd_pending_.load(std::memory_order_relaxed);
      });
    }

    // 3. 명령 적용 후 시뮬레이션 1 step 진행
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
    mj_step(model_, data_);
    step_count_.fetch_add(1, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);
  }
}
```

> **`dt` 일관성**: `sync_step` 모드에서도 `custom_controller.cpp`는 wall clock 기반 500Hz 타이머로
> 동작하며 `state.robot.dt = 0.002`를 유지한다. 시뮬레이터의 `timestep`도 0.002이므로
> 제어기 관점에서 물리적으로 일관성이 유지된다.

---

### Phase 3: 제어기 연산 시간 분석기 구현 ← NEW

**파일:** `include/ur5e_rt_controller/controller_timing_profiler.hpp`

시뮬레이션 환경에서 각 제어기의 `Compute()` 실행 시간을 측정하여
실제 500Hz(2ms 예산) 환경에서의 실현 가능성을 평가한다.

#### 3.1 `ControllerTimingProfiler` 클래스 설계

```cpp
#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

namespace urtc {

/// 제어기 연산 시간 분석기
///
/// 사용법 (custom_controller.cpp ControlLoop() 내):
///   const auto output = timing_profiler_.MeasuredCompute(*controller_, state);
///
/// sync_step 모드에서 step latency = 거의 순수 Compute() 시간
/// free_run  모드에서 측정값에 OS 스케줄링 지터가 포함될 수 있음
class ControllerTimingProfiler {
public:
  // 히스토그램: 0~2000μs 범위, 100μs 단위 버킷 20개 + overflow 1개
  static constexpr int kBuckets      = 20;
  static constexpr int kBucketWidthUs = 100;
  static constexpr double kBudgetUs  = 2000.0;  // 500Hz = 2ms

  struct Stats {
    uint64_t count{0};
    double   min_us{1e9};
    double   max_us{0.0};
    double   mean_us{0.0};
    double   stddev_us{0.0};
    double   p95_us{0.0};   // 근사값 (히스토그램 기반)
    double   p99_us{0.0};   // 근사값 (히스토그램 기반)
    uint64_t over_budget{0};  // kBudgetUs 초과 횟수
    std::array<uint64_t, kBuckets + 1> histogram{};
  };

  /// Compute()를 실행하면서 시간을 측정하고 통계를 업데이트
  [[nodiscard]] ControllerOutput MeasuredCompute(
      RTControllerInterface& ctrl,
      const ControllerState& state) noexcept
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto output   = ctrl.Compute(state);
    const auto t1 = std::chrono::steady_clock::now();

    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    Update(us);
    return output;
  }

  /// 현재 통계 스냅샷 반환 (스레드 안전 — 원자 복사)
  [[nodiscard]] Stats GetStats() const noexcept {
    Stats s;
    s.count        = count_.load(std::memory_order_relaxed);
    s.min_us       = min_us_.load(std::memory_order_relaxed);
    s.max_us       = max_us_.load(std::memory_order_relaxed);
    s.over_budget  = over_budget_.load(std::memory_order_relaxed);
    // 평균/표준편차는 근사 (경쟁 조건 허용)
    if (s.count > 0) {
      s.mean_us   = sum_us_.load(std::memory_order_relaxed) / s.count;
      const double var = sum_sq_us_.load(std::memory_order_relaxed) / s.count
                         - s.mean_us * s.mean_us;
      s.stddev_us = (var > 0) ? std::sqrt(var) : 0.0;
    }
    ComputePercentiles(s);
    return s;
  }

  void Reset() noexcept {
    count_.store(0);  min_us_.store(1e9);  max_us_.store(0);
    sum_us_.store(0); sum_sq_us_.store(0); over_budget_.store(0);
    histogram_.fill(0);
  }

  /// 통계 요약 문자열 생성 (RCLCPP_INFO 출력용)
  [[nodiscard]] std::string Summary(const std::string& ctrl_name) const noexcept;

private:
  void Update(double us) noexcept {
    count_.fetch_add(1, std::memory_order_relaxed);
    AtomicMin(min_us_, us);
    AtomicMax(max_us_, us);
    // 부동소수점 atomic은 fetch_add 불가 → 별도 처리 (정밀도보다 성능 우선)
    // 실제 구현에서는 별도 뮤텍스 없이 relaxed 더블 버퍼 사용 고려
    sum_us_.store(sum_us_.load(std::memory_order_relaxed) + us,
                  std::memory_order_relaxed);
    sum_sq_us_.store(sum_sq_us_.load(std::memory_order_relaxed) + us * us,
                     std::memory_order_relaxed);
    if (us > kBudgetUs) {
      over_budget_.fetch_add(1, std::memory_order_relaxed);
    }
    const int bucket = std::min(static_cast<int>(us / kBucketWidthUs), kBuckets);
    histogram_[bucket].fetch_add(1, std::memory_order_relaxed);
  }

  void ComputePercentiles(Stats& s) const noexcept;

  static void AtomicMin(std::atomic<double>& a, double v) noexcept;
  static void AtomicMax(std::atomic<double>& a, double v) noexcept;

  std::atomic<uint64_t> count_{0};
  std::atomic<double>   min_us_{1e9};
  std::atomic<double>   max_us_{0.0};
  std::atomic<double>   sum_us_{0.0};
  std::atomic<double>   sum_sq_us_{0.0};
  std::atomic<uint64_t> over_budget_{0};
  std::array<std::atomic<uint64_t>, kBuckets + 1> histogram_{};
};

}  // namespace urtc
```

#### 3.2 `LogEntry` 확장 — 연산 시간 필드 추가

`log_buffer.hpp`의 `LogEntry`에 연산 시간 필드를 추가하여 CSV 로그에 포함:

```cpp
// log_buffer.hpp 수정
struct LogEntry {
  double timestamp;
  std::array<double, kNumRobotJoints> current_positions;
  std::array<double, kNumRobotJoints> target_positions;
  std::array<double, kNumRobotJoints> commands;
  double compute_time_us{0.0};  // ← NEW: Compute() 실행 시간 (μs)
};
```

#### 3.3 `custom_controller.cpp` 통합 — 최소 변경

`ControlLoop()` 내 `controller_->Compute(state)` 호출을 프로파일러로 래핑:

```cpp
// custom_controller.cpp ControlLoop() 수정 부분
// 기존:
//   const urtc::ControllerOutput output = controller_->Compute(state);
// 변경 후:
const urtc::ControllerOutput output =
    timing_profiler_.MeasuredCompute(*controller_, state);

// LogEntry에 연산 시간 포함
if (enable_logging_) {
  const urtc::LogEntry entry{
      .timestamp         = now().seconds(),
      .current_positions = state.robot.positions,
      .target_positions  = target_snapshot_,
      .commands          = output.robot_commands,
      .compute_time_us   = timing_profiler_.GetStats().max_us,  // 최신 측정값
  };
  log_buffer_.Push(entry);
}

// 1000 스텝마다 통계 출력
if (loop_count_ % 1000 == 0 && loop_count_ > 0) {
  RCLCPP_INFO(get_logger(), "%s",
              timing_profiler_.Summary(controller_->Name().data()).c_str());
}
```

#### 3.4 예상 제어기별 연산 시간

시뮬레이션 환경에서 측정된 예상 연산 시간 범위 (참고값, 하드웨어 의존):

| 제어기 | 평균 연산 시간 | 500Hz 2ms 예산 대비 |
|--------|--------------|---------------------|
| `PDController` | ~1–5 μs | ✅ 충분한 여유 |
| `PController` | ~1–3 μs | ✅ 충분한 여유 |
| `PinocchioController` | ~200–800 μs | ✅ 여유 있음 |
| `ClikController` (CLIK IK) | ~500–1500 μs | ⚠️ 조건에 따라 초과 가능 |
| `OperationalSpaceController` (OSC) | ~1000–3000 μs | ⚠️ 행렬 역산 비용 주의 |

> **활용**: `sync_step` 모드로 시뮬레이션 실행 후 `/tmp/ur5e_control_log.csv`의
> `compute_time_us` 컬럼을 분석하여 실제 배포 전 P99 레이턴시를 확인한다.

---

### Phase 4: ROS2 시뮬레이터 노드 구현

**파일:** `src/mujoco_simulator_node.cpp`

#### 4.1 노드 클래스 설계

```cpp
class MuJoCoSimulatorNode : public rclcpp::Node {
public:
  MuJoCoSimulatorNode();

private:
  std::unique_ptr<urtc::MuJoCoSimulator> sim_;

  // RealtimePublisher 사용: sim_thread_에서 publish() 호출 시 스레드 안전 보장
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;

  // 시뮬레이션 시간 기반 E-STOP 비활성화 (비실시간 모드)
  // custom_controller의 robot_timeout_ 워치독은 벽시계 기반이므로
  // free_run 모드에서는 false로 설정 권장
  bool disable_estop_in_sim_{true};

  static constexpr std::array<const char*, 6> kJointNames = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

  void CommandCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void PublishJointState(
      const std::array<double, 6>& pos,
      const std::array<double, 6>& vel);
};
```

#### 4.2 E-STOP 워치독과 비실시간 모드 충돌 처리

`custom_controller.cpp`의 `robot_timeout_`(기본 100ms)은 벽시계 기준이다.
비실시간 `free_run` 모드에서 시뮬레이터가 100ms 내에 `/joint_states`를 게시하지 못하면
불필요한 E-STOP이 발생한다.

**해결 방법 (우선순위 순):**

1. **런치 파일 파라미터로 우회** (권장): `enable_estop: false` 설정
2. **타임아웃 확장**: `robot_timeout_ms: 10000` (시뮬레이션용 크게 완화)
3. **시뮬레이션 전용 조건부 컴파일**: `sim_mode` 파라미터로 타임아웃 비활성화

```yaml
# custom_controller 시뮬레이션 실행 시 파라미터 오버라이드
custom_controller:
  ros__parameters:
    enable_estop: false        # 비실시간 시뮬레이션에서 E-STOP 워치독 비활성화
    robot_timeout_ms: 10000.0  # 또는 타임아웃 완화 (대안)
    hand_timeout_ms: 10000.0
```

#### 4.3 주요 흐름

```
MuJoCoSimulatorNode 초기화
  1. ROS2 파라미터 로드 (model_path, sim_mode, ...)
  2. MuJoCoSimulator 생성 및 Initialize()
  3. /joint_states publisher 생성
  4. /forward_position_controller/commands subscriber 생성
  5. MuJoCoSimulator.SetStateCallback → PublishJointState 연결
  6. MuJoCoSimulator.Start() → 시뮬레이션 루프 시작 (비실시간)

런타임 루프 [free_run 모드]
  최대 속도:
    1. cmd_pending_ 확인 (atomic, lock-free)
    2. mj_step(model, data)
    3. 매 N 스텝: ReadState + PublishJointState → /joint_states 게시

런타임 루프 [sync_step 모드]
  매 제어 사이클:
    1. ReadState + PublishJointState → /joint_states 게시
    2. condition_variable.wait_for(sync_timeout_ms) — 명령 대기
    3. ApplyCommand + mj_step — 1 step 진행

ROS2 콜백
  /forward_position_controller/commands 수신 시:
    1. 데이터 크기 검증 (6개 원소)
    2. sim_.SetCommand(cmd)
       - free_run: cmd_pending_ atomic 플래그 세트
       - sync_step: + condition_variable notify_one()
```

---

### Phase 5: 설정 파일 및 런치 파일

#### 5.1 `config/mujoco_simulator.yaml`

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: "models/ur5e/scene.xml"

    # 시뮬레이션 모드: "free_run" | "sync_step"
    # free_run  — 최대 속도 실행 (알고리즘 검증, 궤적 생성)
    # sync_step — 명령 1회 수신 후 1 step 진행 (연산 시간 직접 측정)
    sim_mode: "free_run"

    # free_run 전용: N 스텝마다 /joint_states 게시 (기본 1 = 매 스텝)
    # 시뮬이 너무 빠를 경우 DDS 트래픽 조절용으로 증가
    publish_decimation: 1

    # sync_step 전용: 제어기 명령 대기 타임아웃 (ms)
    sync_timeout_ms: 50.0

    # 뷰어
    enable_viewer: true

    # 초기 관절 위치 (rad) — 안전 자세
    initial_joint_positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    joint_names:
      - "shoulder_pan_joint"
      - "shoulder_lift_joint"
      - "elbow_joint"
      - "wrist_1_joint"
      - "wrist_2_joint"
      - "wrist_3_joint"

# custom_controller 파라미터 오버라이드 (시뮬레이션 전용)
custom_controller:
  ros__parameters:
    control_rate: 500.0
    kp: 5.0
    kd: 0.5
    enable_logging: true
    enable_estop: false        # 비실시간 시뮬레이션에서 E-STOP 비활성화
    robot_timeout_ms: 10000.0
    hand_timeout_ms: 10000.0
```

#### 5.2 `launch/mujoco_sim.launch.py`

```python
"""
MuJoCo 시뮬레이션 런치 파일

사용법:
  # free_run 모드 (기본, 최대 속도)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py

  # sync_step 모드 (제어기 연산 시간 측정)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=sync_step

  # Headless 모드 (GUI 없음)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

  # PD 제어기 게인 조정
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py kp:=10.0 kd:=1.0
"""

런치 노드 목록:
  1. mujoco_simulator_node  ← 시뮬레이터 (UR 드라이버 역할)
  2. custom_controller       ← 기존 제어기 (변경 없음, 파라미터 오버라이드만)
  3. motion_editor_gui.py   ← 선택적: 목표 위치 발행 GUI
```

---

### Phase 6: CMakeLists.txt 수정

기존 `CMakeLists.txt`에 추가할 내용:

```cmake
find_package(mujoco REQUIRED)

add_executable(mujoco_simulator_node
  src/mujoco_simulator_node.cpp
)

target_include_directories(mujoco_simulator_node PRIVATE
  include
  ${mujoco_INCLUDE_DIRS}
)

target_link_libraries(mujoco_simulator_node
  ${mujoco_LIBRARIES}
  glfw
)

ament_target_dependencies(mujoco_simulator_node
  rclcpp
  realtime_tools
  std_msgs
  sensor_msgs
)

install(TARGETS mujoco_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
```

---

### Phase 7: 핸드 시뮬레이션 (선택적 확장)

`HandState`와 UDP 핸드 프로토콜에 맞춘 확장:

#### 7.1 간이 핸드 모델

```xml
<!-- models/hand/simple_hand.xml -->
<mujoco model="simple_hand">
  <!-- 11개 관절 단순 모델 -->
</mujoco>
```

#### 7.2 추가 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | 게시 | 11개 핸드 관절 상태 |
| `/hand/command` | `std_msgs/Float64MultiArray` | 구독 | 11개 핸드 명령 (0.0–1.0) |

핸드 시뮬레이션은 실제 물리 없이 **단순 1차 저역 필터**로 대체 가능:

```cpp
// hand_state[i] += alpha * (hand_cmd[i] - hand_state[i])
// alpha ≈ 0.1 (100Hz에서 ~10ms 시정수)
```

> **E-STOP 연동 주의**: `custom_controller.cpp`의 `hand_timeout_`(기본 200ms)이
> 비실시간 모드에서 오발동할 수 있으므로 `config/mujoco_simulator.yaml`에서
> `hand_timeout_ms: 10000.0`으로 완화 설정 필수.

---

## 5. 테스트 계획

### 5.1 단계별 검증

| 단계 | 테스트 항목 | 성공 기준 |
|------|-------------|-----------:|
| Phase 2 | MuJoCo 로드 및 `mj_step()` 실행 | 크래시 없이 10,000 스텝 진행 |
| Phase 2 | 관절 인덱스 매핑 검증 | `mj_name2id()` 기반 동적 매핑 성공 |
| Phase 3 | `/joint_states` 게시 확인 | `ros2 topic hz` → 수천 Hz (free_run) |
| Phase 3 | 명령 수신 → 시뮬레이터 반영 | 관절이 목표 위치로 이동 |
| Phase 4 | E-STOP 미발동 확인 | `enable_estop: false` 시 정상 실행 |
| Phase 4 | 핸드 상태 게시 | `/hand/joint_states` 정상 게시 |

### 5.2 제어기 연산 시간 벤치마크 ← NEW

`sync_step` 모드로 각 제어기를 10,000 스텝 실행하여 통계 수집:

```bash
# PDController 벤치마크
ros2 launch ur5e_rt_controller mujoco_sim.launch.py \
  sim_mode:=sync_step enable_viewer:=false

# RCLCPP_INFO 출력 예시 (1000 스텝마다):
# [custom_controller] PDController timing:
#   count=1000  mean=3.2μs  min=2.1μs  max=18.4μs  p95=5.6μs  p99=9.2μs
#   over_budget(2ms)=0  (0.0%)
```

**목표 지표:**

| 제어기 | P99 목표 | over_budget 허용 |
|--------|---------|-----------------|
| `PDController` | < 100 μs | 0% |
| `PinocchioController` | < 1000 μs | < 0.1% |
| `ClikController` | < 1500 μs | < 1% |
| `OperationalSpaceController` | < 2000 μs | < 1% |

### 5.3 시뮬레이션 속도 측정

`free_run` 모드에서 시뮬레이션 배속 측정:

```bash
# 터미널 1: 시뮬레이션 실행
ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=free_run enable_viewer:=false

# 터미널 2: 시뮬 시간 속도 측정 (sim_time / wall_time)
ros2 topic echo /sim/timing_stats

# 기대값 예시:
# PDController:  sim_speed ≈ 50~200× (하드웨어 의존)
# OSCController: sim_speed ≈ 5~20× (연산 비용 높음)
```

### 5.4 실행 명령 예시

```bash
# 터미널 1: free_run 모드 (알고리즘 검증)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=free_run

# 터미널 2: 목표 관절 위치 발행
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
  "data: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]"

# 터미널 3: 연산 시간 로그 분석
python3 -c "
import pandas as pd
df = pd.read_csv('/tmp/ur5e_control_log.csv')
print(df['compute_time_us'].describe())
print(f'P95: {df[\"compute_time_us\"].quantile(0.95):.1f} μs')
print(f'P99: {df[\"compute_time_us\"].quantile(0.99):.1f} μs')
print(f'Over 2ms: {(df[\"compute_time_us\"] > 2000).mean()*100:.2f}%')
"

# 터미널 4: 궤적 시각화
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv
```

---

## 6. 의존성 요약

| 패키지 | 버전 | 용도 |
|--------|------|------|
| `mujoco` | ≥ 3.0.0 | 물리 엔진 C++ 라이브러리 |
| `glfw3` | ≥ 3.3 | MuJoCo 뷰어 윈도우 |
| `ur5e MJCF` | MuJoCo Menagerie | UR5e 로봇 모델 파일 |
| `pandas` | ≥ 1.5 | 연산 시간 로그 분석 스크립트 |

```bash
# 설치 명령
sudo apt install libglfw3-dev
pip install mujoco pandas  # Python 검증 및 분석용

# MuJoCo C++ 라이브러리 (시스템 설치)
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
echo 'export MUJOCO_DIR=/opt/mujoco-3.x.x' >> ~/.bashrc
```

---

## 7. 구현 우선순위 및 일정

| 우선순위 | Phase | 주요 작업 | 예상 공수 |
|----------|-------|-----------|-----------:|
| 1 (필수) | 1 | 환경 구성, UR5e MJCF 모델 확보 | 소 |
| 2 (필수) | 2 | `mujoco_simulator.hpp` — free_run/sync_step 구현 | 중 |
| 3 (필수) | 3 | `controller_timing_profiler.hpp` 구현 + 통합 | 소 |
| 4 (필수) | 4 | `mujoco_simulator_node.cpp` + ROS2 브릿지 | 중 |
| 5 (필수) | 5 | 설정 파일 + 런치 파일 (E-STOP 파라미터 포함) | 소 |
| 6 (필수) | 6 | `CMakeLists.txt` 수정, 빌드 검증 | 소 |
| 7 (선택) | 7 | 핸드 시뮬레이션 추가 | 중 |

---

## 8. 주요 설계 결정 및 트레이드오프

### 8.1 C++ vs Python 구현

| 항목 | C++ 노드 | Python 노드 |
|------|----------|-------------|
| 성능 | 높음 (native) | 낮음 (GIL) |
| 개발 속도 | 느림 | 빠름 |
| 기존 코드 통합 | 자연스러움 | 별도 프로세스 |
| 권장 용도 | 프로덕션 | 프로토타입 |

**결정: C++ 구현 채택** — 기존 코드베이스와 일관성 유지, 연산 시간 측정 정밀도 확보

### 8.2 비실시간(Non-RT) 우선 설계 ← UPDATED

**실시간 동기화 불필요**: 시뮬레이션 환경에서는 벽시계와의 동기화가 검증 목적에 방해가 된다.
`nanosleep()` 기반 동기화를 제거하고 MuJoCo가 가능한 최대 속도로 실행되도록 한다.

| 모드 | 구현 | 주 용도 |
|------|------|---------|
| `free_run` | 타이밍 제약 없는 루프 | 알고리즘 검증, 궤적 생성 |
| `sync_step` | condition_variable 기반 1:1 동기화 | 연산 시간 측정, 실배포 전 검증 |
| ~~`realtime`~~ | ~~nanosleep 벽시계 동기화~~ | ~~제거: 시뮬레이션에서 불필요~~ |

### 8.3 MuJoCo 액추에이터 타입

**`<position>` 서보 액추에이터 채택:**
- `ctrl[i] = target_position[i]` 직접 설정
- MuJoCo 내부 PD 게인으로 추종
- `/forward_position_controller/commands`의 위치 명령과 자연스럽게 대응

### 8.4 시뮬레이션 `dt` 일관성 보장

비실시간 모드에서도 `custom_controller.cpp`가 사용하는 `state.robot.dt`는
항상 모델의 `timestep` 값(0.002s)과 일치해야 한다. 벽시계 경과 시간을 `dt`로
사용하면 비실시간 모드에서 제어기 내부 미분항이 오작동한다.

```
올바른 dt: state.robot.dt = 1.0 / control_rate_ = 0.002s  (논리적 제어 주기)
잘못된 dt: state.robot.dt = elapsed_wall_clock              (비실시간 모드에서 부정확)
```

현재 `custom_controller.cpp`는 `1.0 / control_rate_`로 고정하므로 **변경 불필요**.

### 8.5 E-STOP 워치독 비활성화 필요성

`custom_controller.cpp`의 `CheckTimeouts()`는 벽시계 기준 타임아웃을 사용한다.
비실시간 `free_run` 모드에서는 시뮬레이터가 100ms 이내에 `/joint_states`를
게시하지 못할 수 있어 E-STOP이 오발동된다. **시뮬레이션 전용 런치 파일에서
`enable_estop: false`를 항상 명시**하도록 한다.

---

## 9. 참고 자료

- [MuJoCo 공식 문서](https://mujoco.readthedocs.io/)
- [MuJoCo Menagerie — UR5e 모델](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e)
- [MuJoCo C API 레퍼런스](https://mujoco.readthedocs.io/en/stable/APIreference/)
- [ROS2 + MuJoCo 통합 패턴](https://github.com/google-deepmind/mujoco/blob/main/doc/programming/index.rst)
- 기존 코드: `include/ur5e_rt_controller/rt_controller_interface.hpp` (데이터 구조)
- 기존 코드: `src/custom_controller.cpp` (제어기 아키텍처)
- 기존 코드: `include/ur5e_rt_controller/log_buffer.hpp` (SPSC 링 버퍼)
