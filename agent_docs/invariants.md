# Invariants

이 파일의 규칙은 **위반 시 아키텍처가 깨진다**. 작업 중 이 중 하나를 건드려야 할 것 같으면, 코드를 수정하기 **전에** `[CONCERN] Severity: Critical` 을 보고하고 사용자 컨펌을 받아야 한다.

규칙을 보완하는 문서:
- [design-principles.md](design-principles.md) — ARCH 섹션의 근거 (robot-agnostic, 5 principles)
- [anti-patterns.md](anti-patterns.md) — 여기 invariant을 위반했던 실제 실수 사례
- [.claude/rules/rt-safety.md](../.claude/rules/rt-safety.md) — `rtc_controller_*` / `rtc_controllers` / `udp_hand_driver`에 자동 로드되는 스코프 stub (이 파일의 RT 섹션을 가리킴)

## RT Path Invariants

**RT path 정의**: `control_rate` YAML 파라미터로 설정된 정기 tick에서 실행되는 모든 경로. 프레임워크는 rate-agnostic (설계 범위 100 Hz–5 kHz, default 500 Hz; 상수: `rtc::kMin/kMax/kDefaultControlRateHz`)으로, **"500 Hz"는 default 일 뿐 가정으로 박지 말 것** — RT 안전성은 *모든* 지원 rate에서 성립해야 한다. 구체적으로 `RtControllerNode::ControlLoop()`, `RTControllerInterface::Compute()` / `SetDeviceTarget()` / `InitializeHoldPosition()` / `PublishNonRtSnapshot()` 내부 기본 tick, UDP receive 콜백, sensor/target 구독 콜백, `CheckTimeouts` 50 Hz 분기. **비-RT path**: `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` lifecycle 콜백, `DrainLog()` aux thread, controller LifecycleNode의 1 Hz aux 타이머 (timing CSV drain 등), ROS 파라미터 콜백.

### RT callback rule

RT path 에 포함되는 subscription / UDP receive / timer callback 은 **mailbox-only** 로 운영한다. 무거운 연산은 callback 에서 수행하지 않고, `ControlLoop()` 가 다음 tick 에 mailbox snapshot 을 읽어 처리한다.

**허용 (mailbox write)**:
- Fixed-size 메시지 필드의 단순 복사 (`Eigen::Map`, `std::memcpy`)
- `std::atomic<T>::store` / `SeqLock<T>::Store` / SPSC enqueue
- Monotonic clock timestamp 캡처 (`std::chrono::steady_clock` 또는 `rclcpp::Clock(RCL_STEADY_TIME)`)

**금지**:
- `tf2_ros::Buffer::lookupTransform` — buffer mutex + dynamic 할당
- `Node::get_parameter` — parameter service mutex
- 동적 할당 (`std::string` 변환 포함)
- `std::function` 재바인딩, `std::bind`
- `RCLCPP_*` 직접 호출 (RT-3 적용)
- String formatting (`fmt::format`, `std::to_string`, `std::ostringstream`)
- 컨트롤러 / lifecycle state transition
- 무거운 수치 연산 — `ControlLoop()` 으로 위임

### RT pub/sub primitive catalog

RT path 의 publisher / state buffer / queue 선택 기준. 1순위 (wait-free + heap-free + single-owner) 를 default 로 하고, 정당한 이유 (신규 단일-토픽 publisher, MPSC 필요 등) 가 있을 때만 2순위로 내려간다. 금지 항목은 RT-1~9 위반.

| 등급 | Primitive | 출처 | 메커니즘 | 사용 가능한 시점 | 비고 |
|------|-----------|------|---------|----------------|------|
| 1 | `SeqLock<T>` | `rtc_base` | writer wait-free + reader retry, heap 0 | RT loop, RT callback, aux thread | `T` 는 `trivially_copyable` 필수. 본 repo default (latest-only state) |
| 1 | `SpscQueue<T,N>` / `SpscPublishBuffer<512>` | `rtc_base` | Boost-style wait-free SPSC | RT loop → aux thread | single-writer / single-reader 강제 |
| 1 | `std::atomic<T>` | C++ stdlib | lock-free (POD only) | 모두 | `T` 는 `is_always_lock_free` 확인 (보통 ≤ 8 bytes POD) |
| 2 | `realtime_tools::LockFreeQueue<T, spsc_queue>` | `realtime_tools` (Boost.Lockfree wrapper) | wait-free SPSC | RT loop → aux thread | `SpscQueue` 와 등가. 외부 라이브러리 호환성 필요 시 |
| 2 | `realtime_tools::RealtimePublisher::try_publish` | `realtime_tools` | `try_lock` + msg copy + cv notify → dedicated non-RT thread | RT loop (best-effort) | 신규 단일-토픽 publisher 에 가치. dedicated thread 생성 — thread_config.hpp 정합 (RT-HOST-2/3) + AP-RTT-1 회피 필수 |
| 2 | `realtime_tools::RealtimeBuffer<T>` / `RealtimeThreadSafeBox<T>` | `realtime_tools` | `try_lock` + double buffer (또는 swap pointer) | RT loop (best-effort) | ctor 에서 `new T()` 2회 — lifecycle 콜백 시점만 ctor/reset 가능 (AP-RTT-2) |
| 금지 | `std::atomic<std::shared_ptr<T>>` | C++20 stdlib | libstdc++/libc++ internal spinlock — wait-free 아님 | (RT 외만) | `SeqLock<std::shared_ptr<T>>` 도 X — RT-8 위반 |
| 금지 | `std::mutex::lock` / `lock_guard` / `scoped_lock` | C++ stdlib | blocking | (RT 외만) | RT-4 위반. `try_to_lock` 은 best-effort 로 RT-4 와 별개 |
| 금지 | `std::shared_ptr` 복사 | C++ stdlib | atomic ref-count contention | (RT 외만) | RT-8 위반. `const std::shared_ptr<T>&` 또는 raw ref 사용 |

**결정 가이드**:

- 신규 latest-only POD state buffer → 1순위 `SeqLock<T>`. `RealtimeBuffer` 는 ctor heap alloc 으로 우열 명백
- 신규 producer → consumer queue (single-writer) → 1순위 `SpscQueue<T,N>`. `LockFreeQueue<spsc_queue>` 는 동등하나 외부 의존성 추가 정당화 필요
- 신규 단일-토픽 RT publisher (디버그용 1회성 등) → 2순위 `RealtimePublisher::try_publish` 검토 가치. dedicated thread 1개 추가 비용 vs 자체 SPSC drain 인프라 작성 비용 트레이드오프
- 기존 SPSC + publish_thread 멀티플렉싱 인프라 → **교체 금지** (회귀 위험, drain 분리 / offload / session log 통합 이점 상실)
- MPSC / MPMC 필요 → ARCH-3 검토 우선 (single-writer 로 재설계 가능한가?). 정말 필요하면 `LockFreeQueue<T, queue>` (Out of scope, 별도 sprint)

| # | 금지 패턴 | 이유 | 위반 탐지 | 복구 |
|---|----------|------|-----------|------|
| RT-1 | `new` / `malloc` / `push_back` / `emplace_back` / `resize` | Heap alloc은 100 µs+ jitter + priority inversion | `grep -nE '(\\bnew [A-Za-z_]\|malloc\\(\|\\.push_back\\(\|\\.emplace_back\\(\|\\.resize\\()' <RT file>` | `std::array`, 사전 할당된 fixed-size `Eigen::Matrix<fixed>` |
| RT-2 | `throw` / `catch` | `noexcept` 위반 = unwinding latency 비결정, process kill 리스크 | `grep -nE '(\\bthrow \|\\bcatch ?\\()' <RT file>` | Error code, `std::optional`, `std::expected` |
| RT-3 | 정기 tick에서 `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 | Blocking I/O (rosout queue / network) | `grep -nE 'RCLCPP_(INFO\|WARN\|ERROR\|DEBUG\|FATAL)\\(' <RT file>` | SPSC log buffer → `DrainLog()` aux thread ([rt_controller_node_estop.cpp](../rtc_controller_manager/src/rt_controller_node_estop.cpp) 참조) |
| RT-4 | `std::mutex::lock()` / `std::lock_guard` / `std::scoped_lock` | 우선순위 역전, blocking | `grep -nE '(lock_guard\|scoped_lock\|::lock\\(\\))' <RT file>` | ① `std::atomic<T>` (POD) ② `SeqLock<T>` (default) ③ `SpscQueue<T,N>` / `SpscPublishBuffer<512>` ④ `std::try_to_lock` (last resort) ⑤ `realtime_tools::LockFreeQueue<T, spsc_queue>` ⑥ `RealtimePublisher::try_publish` ⑦ `RealtimeBuffer<T>` / `RealtimeThreadSafeBox<T>`. 전체 분류·결정 가이드는 §RT pub/sub primitive catalog |
| RT-5 | `auto` with Eigen expression | Expression template lazy-eval → aliasing 버그 (같은 메모리 r/w) | `grep -nE 'auto [^=]*=.*\\.(matrix\|transpose\|inverse\|adjoint\|block)\\(' <file>` | 명시 타입: `Eigen::MatrixXd M = ...` |
| RT-6 | Quaternion `lerp` / `nlerp` | Non-unit 결과 → 회전축 변형, drift | `grep -nE '(nlerp\|\\.lerp\\()' <file>` | `Eigen::Quaterniond::slerp(t, q_b)` only |
| RT-7 | 기존 테스트 assertion을 통과시키려 수정 | 회귀 은폐 | `git diff test/` 에서 `EXPECT_*` / `ASSERT_*` 값 변경 | 새 코드를 고쳐라. assertion이 진짜 틀렸다면 별도 commit으로 논증 |
| RT-8 | `std::shared_ptr` 복사 | Atomic ref-count contention | `grep -nE 'std::shared_ptr<' <RT file>` (값 인자/반환 검사) | Raw ref 또는 `const std::shared_ptr<T>&` |
| RT-9 | RT tick 또는 RT callback 내부에서 `get_lifecycle_state()` / `get_current_state()` 호출 | 내부 state machine 동기화 (mutex 또는 atomic load + 분기). ros2_control 공식 RT-unsafe 명시 | `grep -nE '(get_lifecycle_state\|get_current_state)\(' <RT file>` | `on_activate` 종료 직전 `std::atomic<uint8_t> lifecycle_id_cache_.store(PRIMARY_STATE_ACTIVE, std::memory_order_release)`, RT loop 는 `lifecycle_id_cache_.load(std::memory_order_acquire)` |

ros2_control jazzy 공식 wording: "Avoid using the `get_lifecycle_state()` method in the real-time control loop of the controllers and the hardware components as it is not real-time safe." 현 코드 위반 0건 (`rt_controller_main_impl.cpp:91` 와 `udp_hand_node_lifecycle.cpp:439` 모두 비-RT path). 본 invariant 는 선제적 차단 — 향후 위반 발현 시 위 복구 패턴 적용.

### RT-3 세부 스펙

- **정기 tick 경로**: `RCLCPP_*` 직접 호출 금지. SPSC → aux로 defer. 정기 tick 주파수 × 단 한 줄 블록 = 대형 지터 원인 (예: default 500 Hz × 1줄 = 500 발생/초; 2 kHz면 4배 더 심각).
- **One-shot init 경로 (허용)**: `init_timeout` fatal, `auto-hold initialized` 최초 1회 등 — 1회 발생 후 `rclcpp::shutdown()` 또는 활성화 완료로 더 이상 실행되지 않는 분기. 현재 코드에 다수 존재하며 의도된 상태.
- **THROTTLE 변종 (허용, 단 msg 최적화)**: `RCLCPP_INFO_THROTTLE` / `RCLCPP_WARN_THROTTLE` 등 허용. 단 msg 내용은 RT-safe해야 함:
  - ✅ 단순 format string + 기본 타입 (`int`, `double`, `const char*`, fixed-size `std::array<char, N>::data()`)
  - ❌ 문자열 concat (`std::string + std::string`), `std::stringstream`, `fmt::format`, `std::to_string` — 내부적으로 heap alloc
  - 예: `RCLCPP_WARN_THROTTLE(logger, clock, 1000, "MPC p99=%.0f us", p99_us)` ✅
- **권고**: THROTTLE 도 매 호출 시 clock access + format string evaluation 비용 있음. 가능하면 SPSC → `DrainLog()` 우선, THROTTLE 은 최후 수단.

### RT telemetry rule

RT loop 내부 통계는 **O(1) / fixed-size / allocation-free** 만 허용한다. 무거운 통계 (histogram, percentile 정렬, CSV write, ROS publish) 는 aux thread 에서만.

**RT loop 내부 (허용)**:
- Loop period error: `now - last_tick`, fixed-size ring buffer write
- Compute time: `start = now; ... ; elapsed = now - start;` 누적
- Device read/write time: 동일 패턴
- Activation 이후 max latency: `std::max` 갱신
- Missed deadline 카운트: `std::atomic<uint64_t>` increment

**Aux thread 또는 비-RT (필수)**:
- p50 / p99 / p99.9 percentile — RT 가 push 한 ring buffer snapshot 기반
- Histogram bucketing
- CSV write
- ROS publish — `realtime_tools::RealtimePublisher::try_publish` 검토 가치 있음 (§RT pub/sub primitive catalog 참조)

**임계값 가이드** (ros2_control jazzy controller_manager default 인용, `control_rate=500Hz` / `dt=2ms` 기준):

| 지표 | warn | error | 비고 |
|------|------|-------|------|
| Execution time mean error | 1000 µs | 2000 µs | dt 의 50% / 100% slack |
| Execution time standard deviation | 100 µs | 200 µs | jitter 분산 |
| Periodicity standard deviation | 5.0 Hz | 10.0 Hz | tick rate jitter |
| Missed deadline 카운트 (10s window) | 1 | 10 | hard deadline 위반 |

`control_rate` ≠ 500Hz 일 경우 execution time 항목은 dt 비례 재계산 (예: 2 kHz, dt=500µs → warn=250µs, error=500µs).

## RT Host / Runtime Preconditions

이 조건은 RT controller 가 운영·배포 모드로 실행될 때 host 가 만족시켜야 한다. RT path invariants (RT-1~9) 가 모두 지켜져도 host 가 잘못 설정되면 RT 안정성이 무너진다. 실패 시 코드를 수정하기 전에 host/runtime 문제인지 controller code 문제인지 분리한다.

| # | 규칙 | 이유 | 검증/구현 |
|---|------|------|----------|
| RT-HOST-1 | RT 프로세스는 main 진입 또는 `on_configure` 종료 시점에 `mlockall(MCL_CURRENT \| MCL_FUTURE)` 1회 호출 | Major/minor page fault → ms 단위 jitter | `controller_manager` 의 `lock_memory: true` 파라미터 또는 자체 `mlockall` 래퍼. `repo_scripts/scripts/verify_rt_runtime.sh` 가 `VmLck>0` 검증 |
| RT-HOST-2 | RT thread 는 SCHED_FIFO priority ∈ [50, 95] 로 `on_activate` 시점에 설정. **99 금지** (kernel watchdog 영역) | CFS 비결정성 제거 | `rtc_base/threading/thread_config.hpp` 의 `SystemThreadConfigs` SSoT 사용. 60~90 layout 이미 호환. 외부 라이브러리 (`realtime_tools` 등) 가 자체 dedicated thread 생성 시 이 layout 과 정합 필수 |
| RT-HOST-3 | RT thread 는 `thread_config.hpp` + `cpu_shield.sh` 가 정의한 CPU core 에 affinity 고정 | DDS receive thread / ROS executor / IRQ 와 동일 core 공유 시 cache pollution + 선점 jitter | `thread_config.hpp` 의 `SystemThreadConfigs::cpu_affinity` 또는 `cpu_shield.sh` runtime 격리. 외부 라이브러리 thread 생성 시 동일 정책 적용 필수 |

**System-level** (수치 박제 X, script 위임): 다음은 배포 환경 책임이며, controller 시작 시 sensor 로 확인하고 timing log 헤더에 결과를 기록한다:

- PREEMPT_RT 커널 또는 lowlatency 커널 사용 — `uname -a` 출력으로 확인
- `/etc/security/limits.conf` 의 `@realtime` 그룹 권한 (`rtprio`, `memlock unlimited`) — `ulimit -r`, `ulimit -l` 로 확인
- Isolated RT core 에 non-RT 작업 미공유 — `taskset`, `/proc/interrupts` 로 확인

세부 검증 명령은 [repo_scripts/scripts/check_rt_setup.sh](../repo_scripts/scripts/check_rt_setup.sh) 와 [verify_rt_runtime.sh](../repo_scripts/scripts/verify_rt_runtime.sh) 위임.

**권장 sensor** (배포 전 1회, 환경 변경 시 재실행):

- `cyclictest --mlockall --smp --priority=80 --interval=200` — baseline kernel jitter
- `rtla osnoise top -P F:1 -c <iso_cores>` — Ubuntu 24.04 기본 제공
- `rtla hwnoise hist` — IRQ 비활성 시 hardware-induced noise

결과는 timing CSV 와 동일 디렉토리에 저장.

**Docker 배포 노트**: 컨테이너 배포 시 `--cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1` 누락 시 RT-HOST-1, RT-HOST-2 silent fail.

## Architecture Invariants

| # | 규칙 | 이유 | 위반 탐지 |
|---|------|------|-----------|
| ARCH-1 | `rtc_*` 패키지에 로봇 이름·joint 수·HW ID 하드코딩 금지 | robot-agnostic 훼손 ([design-principles.md](design-principles.md) §Generality) | `grep -rniE '(ur5e\|6.?dof\|10.?dof\|num.?joints = [0-9])' rtc_*/` |
| ARCH-2 | 의존성 그래프 상향 의존 금지 ([architecture.md](architecture.md) §Dependency Graph) | Cyclic dep / abstraction leak | `rtc_base/`가 `rtc_controllers/` include, `rtc_*/`가 `ur5e_*/` include 등 |
| ARCH-3 | Abstract interface 없이 두 번째 구체 구현 추가 금지 | 확장성 훼손 → 세 번째 impl에서 `#ifdef` 지옥 | 새 `.cpp`에 대응하는 pure-virtual base 부재 |
| ARCH-4 | `ur5e_*` 헤더가 `rtc_*` private 헤더 include 금지 | 경계 훼손, robot-specific leak | `grep -rn '#include "rtc_.*/src/' ur5e_*/` |
| ARCH-5 | `robot_descriptions`는 data-only — build-time 의존 금지 | 빌드 토폴로지 부담 + "share만 있으면 OK" 모델 훼손 | `grep -rn 'find_package(robot_descriptions\|ament_target_dependencies.*robot_descriptions' --include=CMakeLists.txt .` 그리고 `grep -rn '<depend>robot_descriptions</depend>\|<build_depend>robot_descriptions' --include=package.xml .` |

### ARCH-5 세부 스펙

`robot_descriptions`는 C++ target / 헤더 / 라이브러리 export가 0건인 data-only 패키지다 ([robot_descriptions/CMakeLists.txt](../robot_descriptions/CMakeLists.txt)는 `install(DIRECTORY robots/)` 한 줄뿐). 소비 패키지는 다음만 사용한다:

**허용**:
- `package.xml`: `<exec_depend>robot_descriptions</exec_depend>`
- C++: `ament_index_cpp::get_package_share_directory("robot_descriptions")`
- Python: `ament_index_python.packages.get_package_share_directory("robot_descriptions")`
- URDF/MJCF/launch/YAML: `package://robot_descriptions/robots/<name>/...` URL, 또는 패키지명 문자열 (rtc_controller_manager 가 런타임 resolve — `rtc_controller_manager/src/rt_controller_node_params.cpp` 참조)

**금지**:
- `find_package(robot_descriptions ...)` (CMakeLists.txt)
- `<depend>` / `<build_depend>` (package.xml)
- `ament_target_dependencies(... robot_descriptions)`
- `ament_export_dependencies(... robot_descriptions)`

**근거**: 빌드 시점에 link 할 artifact 가 0개이므로 build-dep 효과는 0. 그러나 build-dep 을 걸면 colcon 이 강제 토폴로지 엣지를 만들어 "이 디렉토리를 워크스페이스 어디 두든 — 형제 디렉토리든 별도 overlay 든 — `install/robot_descriptions/share/` 만 있으면 동작" 모델이 깨진다 (사용자 정책).

**복구**: build-dep 줄 제거 + `<exec_depend>` 로 강등. 일반적으로 코드 수정 0 줄.

**예외**: 미래에 `robot_descriptions`가 진짜 C++ 라이브러리를 export하게 되면 별도 패키지 (`robot_descriptions_utils` 등)로 split — 이 invariant는 그대로 유지.

## Process Invariants

| # | 규칙 | 이유 |
|---|------|------|
| PROC-1 | 코드 변경 시 대응 문서·YAML·CMakeLists·package.xml 동기화 ([modification-guide.md](modification-guide.md) 6단계) | Drift 방지 — git log에서 반복 수정 커밋 다수 확인됨 |
| PROC-2 | 공개 API 변경 시 downstream 패키지 재빌드·재테스트 | ABI 호환성 |
| PROC-3 | `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·전체 테스트 | 광범위 영향 — 대부분 패키지가 의존 |
| PROC-4 | E-STOP trigger는 idempotent (`compare_exchange_strong`) | 중복 트리거 안전성 |

## Numerical Invariants

| # | 규칙 | 이유 | 구현 위치 |
|---|------|------|-----------|
| NUM-1 | 특이점 근처: damped pseudoinverse 필수 (`damping` YAML 주입) | Unbounded magnification | ClikController, OSC |
| NUM-2 | `dt` near-zero guard | `1/dt` 발산 | 모든 trajectory generator |
| NUM-3 | Quaternion 정규화 매 곱 후 | Drift → non-unit | SE3 trajectory, orientation PD |
| NUM-4 | `trajectory_speed`: `std::max(1e-6, val)` 클램프 | IEEE 754 `1/0 = INF` → hang | [archive/controller-safety-improvements.md](archive/controller-safety-improvements.md) Phase 2 R-4 |

## 이 파일의 규칙을 건드려야 할 것 같을 때

1. 수정 **전** `[CONCERN]` 보고:
   ```
   [CONCERN] <한 줄 요약>
   Severity: Critical
   Detail: <어떤 invariant에 저촉되는가, 영향 범위, 대안 검토 결과>
   Alternative: <우회 안 1개 이상 — interface 추가, SPSC defer, aux thread 이동 등>
   ```
2. 사용자 컨펌 후 진행
3. "임시로 위반 → 나중에 정리"는 허용되지 않음. Warning 이상은 별도 리팩터 task로 분리

## False-positive 처리

위 grep 명령은 **path-blind**(RT path 외 코드도 매칭)이거나 **role-blind**(one-shot init / aux thread 허용 케이스도 매칭)이다. 정당한 사용을 invariant로 잘못 차단하면 [CLAUDE.md](../CLAUDE.md) §11 *Harness pruning 신호*에 해당.

판단 절차:

1. **Path 확인**: 매칭된 파일이 RT path 정의(이 문서 §RT Path Invariants 첫 단락)에 들어가는가? 아니면 비-RT path (lifecycle 콜백 / `DrainLog()` aux / 1Hz aux 타이머 / 파라미터 콜백)인가?
2. **Role 확인** (RT-3 한정): one-shot init? `RCLCPP_*_THROTTLE` + RT-safe msg? 둘 중 하나면 **허용** ([RT-3 세부 스펙](#rt-3-세부-스펙) 참조).
3. **Aliasing 확인** (RT-5 한정): `auto`가 받는 게 단순 scalar/index인가, 아니면 Eigen expression (`.matrix()`, `.transpose()`, `.inverse()`, `.adjoint()`, `.block()`, `*` 연산)인가? Scalar/index는 false-positive.

False-positive 판정이면:
- 코드는 그대로 진행
- 한 줄 보고: `false-positive: <rule-id> at <file:line>, reason=<RT path 외 / one-shot / scalar auto / ...>`
- 동일 패턴이 반복 false-positive로 보고되면 [anti-patterns.md](anti-patterns.md) 또는 본 문서의 grep 명령을 좁히는 별도 task 후보

**금지**: false-positive 추정이라며 사용자 보고 없이 invariant 우회. 의심스러우면 §"이 파일의 규칙을 건드려야 할 것 같을 때" 의 `[CONCERN]` 절차를 따른다.
