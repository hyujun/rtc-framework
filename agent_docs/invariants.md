# Invariants

이 파일의 규칙은 **위반 시 아키텍처가 깨진다**. 작업 중 이 중 하나를 건드려야 할 것 같으면, 코드를 수정하기 **전에** `[CONCERN] Severity: Critical` 을 보고하고 사용자 컨펌을 받아야 한다.

규칙을 보완하는 문서:
- [design-principles.md](design-principles.md) — ARCH 섹션의 근거 (robot-agnostic, 5 principles)
- [anti-patterns.md](anti-patterns.md) — 여기 invariant을 위반했던 실제 실수 사례

## RT Path Invariants

**RT path 정의**: `control_rate` YAML 파라미터로 설정된 정기 tick **또는 SCHED_FIFO dedicated-core 로 실행되는 모든 thread** 에서 실행되는 모든 경로. 프레임워크는 rate-agnostic (설계 범위 100 Hz–5 kHz, default 500 Hz; 상수: `rtc::kMin/kMax/kDefaultControlRateHz`)으로, **"500 Hz"는 default 일 뿐 가정으로 박지 말 것** — RT 안전성은 *모든* 지원 rate에서 성립해야 한다. 구체적으로 `RtControllerNode::ControlLoop()`, `RTControllerInterface::Compute()` / `SetDeviceTarget()` / `InitializeHoldPosition()` / `PublishNonRtSnapshot()` 내부 기본 tick, UDP receive 콜백, sensor/target 구독 콜백, `CheckTimeouts` 50 Hz 분기, **MPC thread (`HandlerMPCThread::Tick` 등 — [architecture.md](architecture.md) §Threading Model 의 dedicated SCHED_FIFO core)**, **`UdpHandController::RunCommCycle`** (self-clocked UDP send/recv cycle — `rtc::PeriodicRtThread` 기반 CommLoop, 별도 SCHED_FIFO thread). **비-RT path**: `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` lifecycle 콜백, `DrainLog()` aux thread, controller LifecycleNode의 1 Hz aux 타이머 (timing CSV drain 등), ROS 파라미터 콜백, ROS subscription / service handler (ROS executor — aux thread).

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
- `std::condition_variable::notify_one/all` / `cv.wait*` — RT producer wake 시 mutex 보유. eventfd + non-blocking write 로 대체 (RT-10)
- String formatting (`fmt::format`, `std::to_string`, `std::ostringstream`)
- 컨트롤러 / lifecycle state transition
- 무거운 수치 연산 — `ControlLoop()` 으로 위임

### Clock 시간축 규칙

두 시간축을 명확히 분리한다. **Topic 경계 (`header.stamp`)** = ROS wall clock (`std::chrono::system_clock`, CLOCK_REALTIME) — rosbag / tf2 / message_filters / 외부 노드 호환. **내부 timing / watchdog / staleness** = monotonic (`std::chrono::steady_clock`) — NTP step·역행에 불변. **`header.stamp` 를 staleness / E-STOP / deadline 판단에 사용 금지** (wall clock 은 NTP 로 역행·점프 가능). 현 코드는 `last_state_ns_` 등 watchdog 을 steady_clock 으로 유지해 이미 준수.

### RT pub/sub primitive catalog

RT path 의 publisher / state buffer / queue 선택 기준. 1순위 (wait-free + heap-free + single-owner) 를 default 로 하고, 정당한 이유 (신규 단일-토픽 publisher, MPSC 필요 등) 가 있을 때만 2순위로 내려간다. 금지 항목은 RT-1~10 위반.

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
| 금지 | `std::condition_variable` / `std::condition_variable_any` | C++ stdlib | mutex + futex wake — `notify_*` path 가 내부 mutex 보유 | (RT 외만) | RT-10 위반. RT producer→consumer wake 는 **eventfd + non-blocking write** (CM `publish_eventfd_` 패턴, `UdpHandController::event_fd_` 패턴) 또는 SPSC + polling |

**결정 가이드**:

- 신규 latest-only POD state buffer → 1순위 `SeqLock<T>`. `RealtimeBuffer` 는 ctor heap alloc 으로 우열 명백
- 신규 producer → consumer queue (single-writer) → 1순위 `SpscQueue<T,N>`. `LockFreeQueue<spsc_queue>` 는 동등하나 외부 의존성 추가 정당화 필요
- 신규 단일-토픽 RT publisher (디버그용 1회성 등) → 2순위 `RealtimePublisher::try_publish` 검토 가치. dedicated thread 1개 추가 비용 vs 자체 SPSC drain 인프라 작성 비용 트레이드오프
- 기존 SPSC + publish_thread 멀티플렉싱 인프라 → **교체 금지** (회귀 위험, drain 분리 / offload / session log 통합 이점 상실)
- MPSC / MPMC 필요 → ARCH-3 검토 우선 (single-writer 로 재설계 가능한가?). 정말 필요하면 `LockFreeQueue<T, queue>` (Out of scope, 별도 sprint)

| # | 금지 패턴 | 이유 | 복구 |
|---|----------|------|------|
| RT-1 | `new` / `malloc` / `push_back` / `emplace_back` / `resize` | Heap alloc은 100 µs+ jitter + priority inversion | `std::array`, 사전 할당된 fixed-size `Eigen::Matrix<fixed>` |
| RT-2 | `throw` / `catch` | `noexcept` 위반 = unwinding latency 비결정, process kill 리스크 | Error code, `std::optional`, `std::expected` |
| RT-3 | 정기 tick에서 `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 | Blocking I/O (rosout queue / network) | SPSC log buffer → `DrainLog()` aux thread ([rt_controller_node_estop.cpp](../rtc_controller_manager/src/rt_controller_node_estop.cpp) 참조) |
| RT-4 | `std::mutex::lock()` / `std::lock_guard` / `std::scoped_lock` | 우선순위 역전, blocking | 1순위: `SeqLock<T>` (latest-only state, default) / `SpscQueue<T,N>` (producer→consumer) / `std::atomic<T>` (POD); last resort `std::try_to_lock`. **전체 7-등급 분류·결정 가이드는 위 §RT pub/sub primitive catalog** (중복 박제 회피) |
| RT-5 | `auto` with Eigen expression | Expression template lazy-eval → aliasing 버그 (같은 메모리 r/w) | 명시 타입: `Eigen::MatrixXd M = ...` |
| RT-6 | Quaternion `lerp` / `nlerp` | Non-unit 결과 → 회전축 변형, drift | `Eigen::Quaterniond::slerp(t, q_b)` only |
| RT-7 | *(은퇴 — [PROC-6](#process-invariants) 으로 이동)* | assertion 무결성은 timing-safety 가 아닌 process 규칙 | ↓ PROC-6 참조 |
| RT-8 | `std::shared_ptr` 복사 | Atomic ref-count contention | Raw ref 또는 `const std::shared_ptr<T>&` |
| RT-9 | RT tick 또는 RT callback 내부에서 `get_lifecycle_state()` / `get_current_state()` 호출 | 내부 state machine 동기화 (mutex 또는 atomic load + 분기). ros2_control 공식 RT-unsafe 명시 | `on_activate` 종료 직전 `std::atomic<uint8_t> lifecycle_id_cache_.store(PRIMARY_STATE_ACTIVE, std::memory_order_release)`, RT loop 는 `lifecycle_id_cache_.load(std::memory_order_acquire)` |
| RT-10 | `std::condition_variable` / `std::condition_variable_any` 의 `notify_*` / `wait*` (RT path 의 producer 또는 consumer) | `notify_one/all` 자체가 내부 mutex 잡고 thread wake — 우선순위 역전 + 비결정 latency. `wait` 은 명시 mutex lock 보유 (RT-4 결합) | (a) **eventfd + non-blocking write/poll** — CM 의 `nrt_publish_eventfd_` (`RtControllerNode::StartNrtPublishLoop` / `NrtPublishLoopEntry`); (b) `SpscQueue<T,N>` + consumer polling (wake latency = polling 주기); (c) `std::atomic<bool>` release/acquire flag + **self-clocked consumer** — `UdpHandController` 의 `event_pending_` 를 `rtc::PeriodicRtThread` 기반 CommLoop 이 매 tick latch (별도 wake 없음; 명령 없으면 read-only) — busy-spin 아님, loop 가 이미 고정주기 tick. **RT 핫패스에서의 self-termination** (예: E-Stop) 도 `RequestStop()`(pause-mutex + `pause_cv_.notify_all()` 보유) 대신 `PeriodicRtThread::RequestLoopExit()` (순수 `atomic<bool>` store) 로 예약하고, 종료 시 부수 작업은 loop unwind 후 base 가 1회 호출하는 `OnLoopAborted()` (loop 스레드, 핫패스 밖) 에서 수행 — `UdpHandController::OnCommLoopAborted()` 의 zero-write 참조 |

#### 위반 탐지 패턴

아래 패턴을 편집 중인 RT 파일에 대해 실행한다 (`<RT file>` 자리에 대상 경로). **표 셀이 아니라 fenced 블록에 두는 이유**: 마크다운 표는 이스케이프 없이 `|` 를 담을 수 없고, ERE 에서 `\|` 는 alternation 이 아니라 *리터럴 파이프* 다 — 문법은 성해서 exit 1 + 무출력, 즉 "위반 없음" 과 구분되지 않는다. 2026-07 감사에서 이 표의 패턴 10개가 전부 그 상태였다 (#213).

각 블록의 `# probe:` 는 그 패턴이 **반드시 매치해야 하는** 라인이고 `# antiprobe:` 는 **매치하면 안 되는** 라인이다. [validate_docs.py](../repo_scripts/scripts/validate_docs.py) 가 CI 에서 정적 lint + 실행으로 검증하므로, 여기서 패턴이 다시 썩으면 빌드가 빨개진다.

```detect id=RT-1
grep -nE '(\bnew [A-Za-z_]|malloc\(|\.push_back\(|\.emplace_back\(|\.resize\()' <RT file>
# probe: buffer.push_back(sample);
# antiprobe: const int renew_count = 0;
```

```detect id=RT-2
grep -nE '(\bthrow |\bcatch ?\()' <RT file>
# probe:     throw std::runtime_error("boom");
# antiprobe: // rethrows are documented in the header
```

```detect id=RT-3
grep -nE 'RCLCPP_(INFO|WARN|ERROR|DEBUG|FATAL)\(' <RT file>
# probe:   RCLCPP_WARN(get_logger(), "late tick");
# antiprobe:   RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "ok");
# exemplar: rtc_controller_manager/src/rt_controller_node_services.cpp
```

```detect id=RT-4
grep -nE '(lock_guard|scoped_lock|::lock\(\))' <RT file>
# probe:   std::lock_guard<std::mutex> guard(mutex_);
# antiprobe:   if (mutex_.try_lock()) {
# exemplar: rtc_base/include/rtc_base/threading/periodic_rt_thread.hpp
```

```detect id=RT-5
grep -nE 'auto [^=]*=.*\.(matrix|transpose|inverse|adjoint|block)\(' <file>
# probe:   auto Jt = J.transpose();
# antiprobe:   Eigen::MatrixXd Jt = J.transpose();
```

```detect id=RT-6
grep -nE '(nlerp|\.lerp\()' <file>
# probe:   q = q_a.slerp(t, q_b).nlerp(t, q_c);
# antiprobe:   q = q_a.slerp(t, q_b);
```

```detect id=RT-8
grep -nE 'std::shared_ptr<' <RT file>
# probe:   void Publish(std::shared_ptr<Msg> msg);
# antiprobe:   void Publish(const SharedMsg& msg);
```

```detect id=RT-9
grep -nE '(get_lifecycle_state|get_current_state)\(' <RT file>
# probe:   if (get_current_state().id() == kActive) {
# antiprobe:   if (lifecycle_id_cache_.load(std::memory_order_acquire) == kActive) {
# exemplar: rtc_controller_manager/src/rt_controller_main_impl.cpp
```

```detect id=RT-10
grep -nE '(std::condition_variable|\.notify_(one|all)\(|\.wait(_for|_until)?\()' <RT file>
# probe:   pause_cv_.notify_all();
# antiprobe:   eventfd_write(event_fd_, 1);
# exemplar: rtc_base/include/rtc_base/threading/periodic_rt_thread.hpp
```

`.wait(|_for|_until)\(` 처럼 **빈 alternation 분기** 를 쓰지 말 것 — GNU grep 은 관대하지만 에이전트 샌드박스의 `grep` 은 ugrep 으로 resolve 되고 거기서는 `empty (sub)expression` hard error 다. `(_for|_until)?` 로 쓴다.


ros2_control jazzy 공식 wording: "Avoid using the `get_lifecycle_state()` method in the real-time control loop of the controllers and the hardware components as it is not real-time safe." 현 호출부는 `RtControllerMain` 의 활성화 폴링과 `UdpHandNode::on_shutdown` 등 **lifecycle 경로** 뿐이라 면제 대상이다 — RT-9 는 선제적 차단이며, 위 detect 블록으로 현황을 직접 확인한다 (건수를 여기 박제하면 AP-DOC-1).

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
| ARCH-1 | `rtc_*` 패키지에 로봇 이름·joint 수·HW ID 하드코딩 금지 | robot-agnostic 훼손 ([design-principles.md](design-principles.md) §Generality) | **자동** — hook Phase 0 의 ARCH-1 검사 |
| ARCH-2 | 의존성 그래프 상향 의존 금지 ([architecture.md](architecture.md) §Dependency Graph) | Cyclic dep / abstraction leak | 수동 리뷰 — `rtc_base/`가 `rtc_controllers/` include, `rtc_*/`가 integration 패키지 include 등 |
| ARCH-3 | Abstract interface 없이 두 번째 구체 구현 추가 금지 | 확장성 훼손 → 세 번째 impl에서 `#ifdef` 지옥 | 수동 리뷰 — 새 `.cpp`에 대응하는 pure-virtual base 부재 |
| ARCH-4 | integration 패키지가 `rtc_*` private 헤더 (`rtc_*/src/`) include 금지 | 경계 훼손, robot-specific leak | **자동** — hook Phase 0 의 ARCH-4 검사 |
| ARCH-5 | `robot_descriptions`는 data-only — build-time 의존 금지 | 빌드 토폴로지 부담 + "share만 있으면 OK" 모델 훼손 | 수동 리뷰 — `find_package` / `ament_target_dependencies` / `<depend>` 에 `robot_descriptions` |
| ARCH-6 | 모든 ROS 2 topic 은 QoS history `KEEP_LAST`, depth **1** (reliability/durability 는 lane별 유지 — depth 필드만 강제) | 항상 최신 샘플 소비 — stale 큐잉 방지, RT freshness. depth 는 pub/sub 매칭 호환성과 무관하므로 안전. | **자동 (non-blocking)** — hook Phase 0b 의 ARCH-6 검사. 인자 없는 `SensorDataQoS()` (기본 depth 5) 도 `.keep_last(1)` 필요. test fixture 는 면제 |

**탐지의 SSoT 는 [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) 다** — ARCH 계열 패턴을 여기에 복제하지 않는다. 이전에는 문서가 divergent copy 를 들고 있었고, 그 사본이 hook 보다 낡은 스코프 (`ur5e_*/`, whole-file) 를 담은 채 조용히 썩었다 (#213). 정확한 정규식·스코프·면제 규칙이 필요하면 hook 을 읽는다.

RT 계열은 반대다 — hook 은 RT 검사를 **구현하지 않는다**. RT 금지는 정기 tick 경로에만 구속되는데 hook 은 파일 단위로만 보므로, blocking gate 로 만들면 `on_configure` 의 정당한 `push_back` 을 막는다. 그래서 RT 패턴은 위 §위반 탐지 패턴 의 detect 블록에 남기고 CI 가 그 유효성을 검증한다.

### ARCH-6 세부 스펙

- 변환 규칙: `rclcpp::QoS(N)`→`QoS(1)`, `.keep_last(N)`→`.keep_last(1)`, `rclcpp::SensorDataQoS()`→`SensorDataQoS().keep_last(1)` (best_effort 보존), create_pub/sub 정수 리터럴→`1`, Python `QoSProfile(depth=N)`→`depth=1`.
- **reliability / durability 는 절대 함께 바꾸지 않는다**: `transient_local` (latched), `best_effort` (sensor/RT lane), `reliable` 은 그대로. depth 만 이동.
- 적용 범위: 프로덕션 C++ + Python. test fixture 제외. 위반이 정당한 경우 (수집 버퍼 등 다중 샘플 누적 의존) 는 §Escalation `[CONCERN]` (E-1) 로 보고 후 예외 기록.

**기록된 예외 — ToF snapshot 토픽** (`<ns>/tof/snapshot`): 최대 `control_rate` (예: 500 Hz) 로 발행되는 sensor stream 이며 subscriber 가 **매 샘플을 누적**한다 (`shape_estimation` voxel cloud + snapshot_history, `ur5e_bt_coordinator` collection buffer). depth 1 이면 executor 가 못 따라갈 때 중간 스냅샷이 유실되므로 deep best_effort 큐를 유지한다 — publisher 5 (`integrated_bringup/src/support/owned_topics.cpp` `SetupToFSnapshotPublisher`), shape_estimation sub 5, bt_coordinator collection sub 100. latest-value 소비자 (wrench, marker, state 토픽) 는 예외 아님 — depth 1.

**`ARCH-6-exempt` 마커**: 예외 lane 의 QoS 코드 라인 끝에 `// ARCH-6-exempt` 주석을 달면 [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Phase 0b grep sensor 가 그 라인을 건너뛴다 (매 편집마다 재-flag 방지). 예외 근거는 코드 주석 + 본 절에 남긴다.

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
| PROC-5 | C++ ↔ Python 미러 쌍은 한쪽만 고치지 말 것 — session subdir 목록 (`rtc_base/logging/session_dir.hpp` `kSubdirs` ↔ `rtc_tools.utils.session_dir` `_SESSION_SUBDIRS`) 등 "동일 로직" 표방 미러는 동시 수정 + 동등성 테스트 통과 | 부분 수정 drift — launch (Python) 가 노드 (C++) 보다 먼저 세션 디렉토리를 만들어 한쪽 누락이 런타임에 표면화 (`test_session_dir.py::test_subdir_list_matches_cpp_mirror`) |
| PROC-6 | 기존 test assertion 을 통과시키려 **약화·수정 금지** (회귀 은폐). **예외**: assertion 이 진짜 틀렸거나 spec 이 바뀐 경우는 정당한 변경 — 새 코드 fix 와 **별도 commit** 으로 근거 제시 + 대응 regression test 갱신, 착수 전 §6 E-6 로 escalate. 탐지: `git diff test/` 의 `EXPECT_*`/`ASSERT_*` 상수 변경 (AP-PROC-4) | 회귀 은폐 방지 — 단, "test 를 절대 못 고친다" 가 아니라 "몰래 약화 금지, 정당한 변경은 근거와 함께". RT timing 과 무관한 process 규칙 (구 RT-7) |

## Numerical Invariants

| # | 규칙 | 이유 | 구현 위치 |
|---|------|------|-----------|
| NUM-1 | 특이점 근처: damped pseudoinverse 필수 (`damping` YAML 주입) | Unbounded magnification | ClikController, OSC |
| NUM-2 | `dt` near-zero guard | `1/dt` 발산 | 모든 trajectory generator |
| NUM-3 | Quaternion 정규화 매 곱 후 | Drift → non-unit | SE3 trajectory, orientation PD |
| NUM-4 | `trajectory_speed`: `std::max(1e-6, val)` 클램프 | IEEE 754 `1/0 = INF` → hang | `ClikController` 의 gains 로더 (`clik_controller.cpp`, `cfg["trajectory_speed"]` 파싱부) |

## 이 파일의 규칙을 건드려야 할 것 같을 때

1. 수정 **전** `[CONCERN] Severity: Critical` 보고 — 포맷 SSoT 는 [CLAUDE.md](../CLAUDE.md) §6 `[CONCERN] 포맷`. `Detail` 에 어떤 invariant 에 저촉되는지·영향 범위, `Alternative` 에 우회 안 1개 이상 (interface 추가 / SPSC defer / aux thread 이동 등).
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
