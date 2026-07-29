# Invariants

이 파일의 규칙은 **위반 시 아키텍처가 깨진다**. 작업 중 이 중 하나를 건드려야 할 것 같으면, 코드를 수정하기 **전에** `[CONCERN]` 을 보고하고 사용자 컨펌을 받아야 한다.

**Severity 는 파일 단위가 아니라 규칙 단위다.** [CLAUDE.md](../CLAUDE.md) §6 이 severity 의 SSoT 이며 (Critical = 컨펌 전 커밋·PR 금지, Warning = 판단에 따라 진행하되 결정 로그), 아래 Architecture 표가 Severity 열을 갖는다. 이전에 이 문단이 전 파일을 Critical 로 선언해 §6 에서 Warning 인 ARCH-3·ARCH-5 와 충돌했다 (#213).

대응은 **1:1 이 아니다** — 그렇게 적었던 문장이 스스로 반증됐으므로 실제 관계를 적는다. §6 이 이름으로 지목하는 invariant 만 전용 E-번호를 갖고 (ARCH-1→E-2, ARCH-3→E-4, ARCH-5→E-10), 나머지는 "invariants.md 규칙을 건드림" 인 **E-1 (Critical)** 로 수렴한다. 반대로 E-3·E-6~E-9·E-11 은 ARCH 표가 아니라 msgs ABI·test·thread·E-STOP 등 다른 축을 가리키므로 대응하는 행이 없다. 그리고 **탐지 sensor 의 blocking 여부와 escalation severity 는 다른 축이다**: ARCH-6 은 non-blocking sensor 로 경고만 내지만, 규칙 자체를 바꾸려면 E-1 로 컨펌을 받아야 한다.

규칙을 보완하는 문서:
- [design-principles.md](design-principles.md) — ARCH 섹션의 근거 (robot-agnostic, 5 principles)
- [anti-patterns.md](anti-patterns.md) — 여기 invariant을 위반했던 실제 실수 사례

## RT Path Invariants

**RT path 정의**: `control_rate` YAML 파라미터로 설정된 정기 tick **또는 SCHED_FIFO dedicated-core 로 실행되는 모든 thread** 에서 실행되는 모든 경로. 프레임워크는 rate-agnostic (설계 범위 100 Hz–5 kHz, default 500 Hz; 상수: `rtc::kMin/kMax/kDefaultControlRateHz`)으로, **"500 Hz"는 default 일 뿐 가정으로 박지 말 것** — RT 안전성은 *모든* 지원 rate에서 성립해야 한다.

**어떤 콜백이 RT 인지는 함수 이름이 아니라 그 콜백이 붙은 executor 의 스케줄러가 결정한다.** 판정의 SSoT 는 [architecture.md](architecture.md) §Execution Contexts 표이고, 개별 판정 절차는 [.claude/rules/rt-path.md](../.claude/rules/rt-path.md) 에 있다. 이 구분은 실제로 갈린다 — backend 의 sensor/state 구독 콜백은 `cb_group_rt_callback_` → SCHED_FIFO 라 **RT** 지만, controller 의 RobotTarget 구독 콜백은 controller LifecycleNode 의 default group → `nrt_callback_executor` → SCHED_OTHER 라 **비-RT** 다. 둘 다 "구독 콜백" 이지만 구속 여부가 반대다.

RT path 의 대표 진입점: `RtControllerNode::ControlLoop()`, `RTControllerInterface::Compute()` / `SetDeviceTarget()` / `InitializeHoldPosition()` 의 tick 경로, DeviceBackend 의 state/motor/sensor 구독 콜백, UDP receive 콜백, `CheckTimeouts` 50 Hz 분기, **MPC thread** (`MPCThread::OnTick` → 파생 `Solve()` — dedicated SCHED_FIFO core), **`UdpHandController::RunCommCycle`** (self-clocked UDP send/recv cycle — `rtc::PeriodicRtThread` 기반 CommLoop, 별도 SCHED_FIFO thread).

**비-RT path**: `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` lifecycle 콜백, `DrainLog()` aux thread, controller LifecycleNode 의 1 Hz aux 타이머 (timing CSV drain 등), ROS 파라미터 콜백, controller-owned RobotTarget 구독과 grasp_command 서비스 핸들러, 그리고 **`PublishNonRtSnapshot()`** — 이름과 달리 executor 콜백이 아니라 `NrtPublishLoopEntry` 의 전용 `std::jthread` (SCHED_OTHER) 에서 SPSC drain 으로 호출된다.

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
| 2 | `realtime_tools::RealtimeBuffer<T>` / `RealtimeThreadSafeBox<T>` | `realtime_tools` | `try_lock` + double buffer (또는 swap pointer) | RT loop (best-effort) | ctor 에서 `new T()` 2회 — lifecycle 콜백 시점만 ctor/reset 가능 (AP-RTT-1) |
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

이 조건은 RT controller 가 운영·배포 모드로 실행될 때 host 가 만족시켜야 한다. RT path invariants (RT-1~10, RT-7 은 은퇴) 가 모두 지켜져도 host 가 잘못 설정되면 RT 안정성이 무너진다. 실패 시 코드를 수정하기 전에 host/runtime 문제인지 controller code 문제인지 분리한다.

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

| # | 규칙 | Severity ([CLAUDE.md](../CLAUDE.md) §6) | 이유 | 위반 탐지 |
|---|------|---|------|-----------|
| ARCH-1 | `rtc_*` 패키지에 로봇 이름·joint 수·HW ID 하드코딩 금지 | **Critical** (E-2) | robot-agnostic 훼손 ([design-principles.md](design-principles.md) §Generality) | **자동** — hook Phase 0 의 ARCH-1 검사 |
| ARCH-2 | 의존성 그래프 상향 의존 금지 ([architecture.md](architecture.md) §Dependency Graph) | **Critical** (E-1) | Cyclic dep / abstraction leak | 수동 리뷰 — `rtc_base/`가 `rtc_controllers/` include, `rtc_*/`가 integration 패키지 include 등 |
| ARCH-3 | Abstract interface 없이 두 번째 구체 구현 추가 금지 | Warning (E-4) | 확장성 훼손 → 세 번째 impl에서 `#ifdef` 지옥 | 수동 리뷰 — 새 `.cpp`에 대응하는 pure-virtual base 부재 |
| ARCH-4 | integration 패키지가 `rtc_*` private 헤더 (`rtc_*/src/`) include 금지 | **Critical** (E-1) | 경계 훼손, robot-specific leak | **자동** — hook Phase 0 의 ARCH-4 검사 |
| ARCH-5 | `robot_descriptions`는 data-only — build-time 의존 금지 | Warning (E-10) | 빌드 토폴로지 부담 + "share만 있으면 OK" 모델 훼손 | 수동 리뷰 — `find_package` / `ament_target_dependencies` / `<depend>` 에 `robot_descriptions` |
| ARCH-6 | 모든 ROS 2 topic 은 QoS history `KEEP_LAST`, depth **1** (reliability/durability 는 lane별 유지 — depth 필드만 강제) | Warning (sensor) / E-1 (규칙 변경) | 항상 최신 샘플 소비 — stale 큐잉 방지, RT freshness. depth 는 pub/sub 매칭 호환성과 무관하므로 안전. | **자동 (non-blocking)** — hook Phase 0b 의 ARCH-6 검사. 인자 없는 `SensorDataQoS()` (기본 depth 5) 도 `.keep_last(1)` 필요. test fixture 는 면제 |
| ARCH-7 | `rtc_*` 는 control-framework runtime identity (RT 제어 루프를 구동하는 exec) 를 소유하지 않는다 | Warning (sensor) / E-1 (규칙 변경) | agnostic 패키지가 exec 를 가지면 exec ↔ 노드 ↔ pgrep ↔ logger 정렬이 깨지고 robot-specific 의존이 새어든다 | **자동** — hook Phase 0 의 ARCH-7 검사. 검사 대상은 `rtc_*/CMakeLists.txt` 에서 **HEAD 에 없던 타깃 이름** 이다 (줄 단위가 아니라 — CMake 는 줄을 제자리에서 고쳐 쓰므로 재들여쓰기가 신규 exec 로 읽혔다). `example_*` 는 이름으로 면제되고, 그 외 예외는 `add_executable` 줄 또는 바로 윗줄의 `ARCH-7-exempt` 주석으로 표시한다. 예외 범위는 [design-principles.md](design-principles.md) §ARCH-7 — robot-agnostic standalone 노드 (`mujoco_simulator_node`, `closure_state_publisher`) 와 example 타깃 |

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

| PROC-7 | Controller-owned SeqLock (`GraspState` / `WbcState` / `ToFSnapshot`) 은 `Compute()` 가 도는 **모든** tick 에서 Store — E-STOP·early-return tick 포함. 이번 tick 에 계산하지 않은 필드는 값을 얼리지 말고 명시적으로 무효화 (`FillEstopPublishState`). 탐지: `Compute()` 의 early-return 경로에 `*_state_lock_.Store` 가 없는 분기 | CM 은 tick 마다 새 `stamp_ns` 를 만들고 publish thread 는 SeqLock 을 다시 Load 하므로, Store 생략은 "미발행" 이 아니라 **stale body + fresh stamp 재발행** 이다 — E-STOP 중에 살아있어 보이는 `valid=1` telemetry 가 그 발현 (issue #234 P-1). 계약·경로는 [controllers.md](controllers.md#ros2-topics) |

## Numerical Invariants

| # | 규칙 | 이유 | 구현 위치 |
|---|------|------|-----------|
| NUM-1 | 특이점 근처: damped pseudoinverse 필수 (`max_damping` / `singularity_threshold` YAML 주입). 램프의 **양 끝단이 모두** 하한을 받는다 — λ_max 바닥 `kMinMaxDamping` 은 **값을 쓰는 지점**에 (`LoadConfig` 에만 두면 `set_gains()` / ctor default 가 우회), σ₀ 바닥 `kMinSigma0` 은 `LoadConfig` 에. σ₀ ≤ 0 은 셸을 좁히는 게 아니라 `AdaptiveDampingSquared` 를 short-circuit 시켜 λ²=0 을 상시 반환한다 | Unbounded magnification | 두 상수의 **정의는 [compliance/task_dynamics.hpp](../rtc_controllers/include/rtc_controllers/compliance/task_dynamics.hpp) 한 곳** (지키는 법칙 옆; 사본을 소비자마다 두면 이 표 한 줄을 바꾸는 데 여러 번 편집이 필요했다). λ_max 하한은 `std::max` 를 손으로 쓰지 않고 **`compliance::FloorMaxDamping`** 을 부른다 (NUM-6 의 `FloorNonNegativeGain` 과 같은 형태·같은 이유: `max(1e-4, NaN) == 1e-4` 라 비유한 λ_max 를 *그럴듯한* 값으로 세탁한다. 비유한 값은 그대로 통과시켜 하류 finite 검사로 보낸다). **configure 쪽**: `rtc_controllers/src/params/` 의 다섯 파서 (osc·clik·task_impedance·task_admittance·cascaded_compliance) 가 `compliance::FloorMaxDamping(…)` 와 `std::max(compliance::kMinSigma0, …)` 를 건다. **사용 지점 쪽 (λ_max)**: #298 S7c-2 에서 어댑터가 삭제되며 그 half 는 코드에서 사라졌고 지금은 **바인딩 요구사항** — `set_gains()` 상당 경로로 게인 POD 를 SeqLock 에 직접 쓰는 바인딩은 tick 에서 다시 `compliance::FloorMaxDamping(·)` 를 걸어야 한다 (현재 in-tree 소비자 없음; #301 에서 doc 표 한 줄 대신 **grep 되는 심볼**로 만든 이유가 이것이다). 법칙 `AdaptiveDampingSquared` 는 **자기 인자를 floor 하지 않는다** — 안에 넣으면 자기 성질 λ² ≤ λ_max² 가 1e-4 아래에서 거짓이 되고, 기존 oracle 은 전부 λ_max = 0.05 를 쓰므로 **281개 테스트가 모두 green 인 채 통과한다** (#301 에서 실측; `TaskDynamics.AdaptiveDampingDoesNotFloorItsOwnArgument` 가 그 유일한 감지기). λ 규약은 §6.5 하나뿐 — OSC·CLIK 의 상수 λ 는 #236 S2b+S3b 에서 수렴했다 |
| NUM-2 | `dt` near-zero guard | `1/dt` 발산 | 모든 trajectory generator |
| NUM-3 | Quaternion 정규화 매 곱 후 | Drift → non-unit | SE3 trajectory, orientation PD |
| NUM-4 | `trajectory_speed`: `std::max(1e-6, val)` 클램프 | IEEE 754 `1/0 = INF` → hang | `rtc_controllers/src/params/{clik,osc,joint_pd}_params.cpp` 의 `cfg["trajectory_speed"]` / `cfg["trajectory_angular_speed"]` 파싱부 (#298 S7c-2 이전에는 어댑터의 `LoadConfig`), 그리고 `integrated_bringup` 의 각 컨트롤러 `parameters.cpp` / `controller.cpp` — YAML 과 `ros2 param` 양쪽 진입점 모두 |
| NUM-5 | 폐쇄 체인 사영: seed 증분 제한 필수. residual 로 조립 분기를 판정하지 말 것 | 점 구속 loop 은 조립 분기가 여럿이고 **모두 φ=0 을 만족** → ‖φ‖ 검사를 통과한 채 반대편 분기 착지, warm-start 로 영구 고정 | `loop_projection` (`ProjectPassiveWithContinuation`), `RtClosedChainHandle` (tick 당 seed clamp) |
| NUM-6 | 영공간 자세 게인은 `rtc::FloorNonNegativeGain` 하한을 **로더와 사용 지점 양쪽**에서 받고, 사용 지점 하한은 활성 게이트 **판정 앞**에 둔다 | `K_pⁿ < 0` 은 복원이 아니라 발산 방향 (τ₀ = K_pⁿ·(q_ref − q) − K_dⁿ·q̇), `K_dⁿ < 0` 은 에너지 주입. `Nᵀ`/`N` 사영이 이를 task 로부터 가려 **fault 없이 조용히 자세가 밀린다**. 게이트가 `!= 0.0` 이라 법칙에만 걸면 *게이트는 열린 채 값만 0* 인 조합이 생긴다 | **로더 쪽**: `rtc_controllers/src/params/` 의 다섯 파서 전부 + `DemoTaskController` (`integrated_bringup`, `null_kp` 가 `ros2 param`·BT `SetGains` 로 런타임 노출되는 유일한 표면). 파서는 **키 유무와 무관하게 무조건** 이며 `if (!cfg)` 조기 반환 경로도 포함 (거기 게인은 정의상 생성자·`set_gains()` 산) — 여섯 파서 각각의 그 분기를 `test_params_schema.cpp` 의 `AnUndefinedNode*` 케이스가 pin 한다. **사용 지점 쪽** (`Compute()` 의 게이트 판정 앞) 은 #298 S7c-2 에서 어댑터와 함께 사라졌고 이제 **바인딩 요구사항** 이다 — `integrated_bringup` 의 task 컨트롤러가 in-tree 준수 사례 (`compute.cpp` 의 `null_dq_ *= FloorNonNegativeGain(gains.null_kp)`). 하한은 `std::max(0.0, ·)` 를 손으로 쓰지 않는다 — `max(0.0, NaN) == 0.0` 이라 비유한 게인을 세탁해 기존 `nan_inf` SAFE_STOP 을 지운다; `FloorNonNegativeGain` 은 비유한 값을 그대로 통과시켜 그 fault 로 보낸다 |
| NUM-6a | NUM-6 의 *파생 규칙* 두 개 (태스크 임피던스 §6.4 임계감쇠 보정 `K_dⁿ ≥ 2√K_pⁿ`, §6.1 `TRANSLATION_ONLY` 가드) 는 floor **뒤**에 두되 configure 에만 둔다 | floor 앞이면 음수 `K_pⁿ` 가 `if (kp > 0.0)` 을 건너뛰어 감쇠 보정을 통째로 놓치고, `== 0.0` 가드를 그냥 통과한다. 반대로 이 둘을 tick 으로 옮기면 게인의 *하한* 이 아니라 *다른 게인의 재작성* 이 매 tick 돌아 `get_gains()` 와 `Compute()` 가 갈리고 리터럴 oracle 전부가 configure 규칙을 미러해야 한다 | `rtc_controllers/src/params/task_impedance_params.cpp` (YAML 경로 + `if (!cfg)` 경로 둘 다 동일 3단계: floor → §6.4 → §6.1) — #298 S7c-2 이전에는 `task_impedance_controller.cpp` `LoadConfig`. §6.1 의 런타임 등가물은 이제 **바인딩 요구사항** 이다: `set_gains()` 상당 경로로 `TRANSLATION_ONLY` + `K_pⁿ ≤ 0` 에 도달하면 `ComplianceFaults::posture_authority_lost` (DEGRADED) 를 세워야 한다. **이 필드만 고아인 것이 아니다** — `ComplianceStateMachine` 자체에 출하 바인딩이 없어 `ComplianceFaults` 의 *어떤* 필드도 in-tree 기록자가 없고, 인스턴스화는 전부 테스트다 (`integrated_bringup` 은 `compliance/` 를 include 조차 하지 않는다). 조건 자체도 아직 도달 불가: `TaskSelection::kTranslationOnly` 는 `rtc_controllers` 안에만 있고 출하된 task 컨트롤러는 enum 이 아니라 `enable_null_space && !control_6dof` bool 쌍을 쓴다. 따라서 **이 한 필드에만 setter 를 다는 국소 수정은 하지 않는다** (#301 결정) — 상태 머신을 배선하는 바인딩이 생길 때 전체를 함께 세운다. 필드는 [compliance_state_machine.hpp](../rtc_controllers/include/rtc_controllers/compliance/compliance_state_machine.hpp) 에 남아 있고 `AnyDegrade()` 가 이미 읽는다 |
| NUM-6b | §5.3 안전층 게인도 같은 하한을 받는다 — `joint_limit_stiffness`(k_lim)·`joint_limit_damping`(d_lim) 은 `rtc::FloorNonNegativeGain`, `joint_limit_margin`(δ) 은 floor 가 아니라 **configure 거부** (`>= 0` 이고 유한). 세 검사 모두 키 유무와 무관하게 무조건이며 `if (!cfg)` 조기 반환 경로도 포함 | 밴드 안에서 `q − lo < 0` 이라 **부호가 자세 게인과 반대로 작동**한다: `k_lim < 0` 은 관절을 한계에서 밀어내는 대신 **한계 안쪽으로** 밀고, `d_lim < 0` 은 하드스톱 바로 앞에서 에너지를 주입한다 (출하 UR5e 는 `k_lim=0, d_lim=2` 라 d_lim 이 유일한 방어선). δ 는 clamp 로 고칠 수 없다 — `lo = q_min + δ` 이므로 δ<0 은 밴드를 한계 **밖**에 놓고 δ=NaN 은 두 비교를 모두 false 로 만들어 **반발항이 영원히 발동하지 않는데 fault 도 안 뜬다**. 0 으로 clamp 하면 그 오설정이 정상 구동되는 config 가 된다 | `rtc_controllers/src/params/` 의 세 파서 (task_impedance·cascaded_compliance·task_admittance — admittance 는 δ 만; δ 가 q_cmd clamp 밴드를 좁히는 §7.3 형태라도 판정은 같다). **사용 지점 쪽**은 NUM-1 λ_max 와 같은 **바인딩 요구사항** 이고 in-tree 소비자가 없다 — 법칙 `compliance::AddJointLimitRepulsive` 는 자기 인자를 floor **하지 않는다** (`AdaptiveDampingSquared` 와 같은 판정: 법칙이 인자를 몰래 고치면 리터럴 oracle 이 거짓이 되고, 기존 oracle 은 전부 양수 게인이라 그 변경을 못 본다). `test_params_schema.cpp` 의 `SafetyLayerGainSchema.*` 가 키×경로를 pin 하며, 11종 mutation 으로 각 케이스가 실제로 red 가 되는 것을 실측했다 (#280) |

### NUM-5 세부 스펙

`CONTACT_3D`(점) 구속으로 닫은 loop 은 4-bar 처럼 **조립 분기(assembly mode)가 둘 이상**이고, 각 분기가 φ=0 을 정확히 만족한다. 따라서 `converged` / `acceptable` / `closure_error` 를 아무리 엄격하게 잡아도 **물리적으로 틀린 분기를 검출할 수 없다** — 분기를 결정하는 것은 residual 이 아니라 **seed 에서 해까지의 homotopy 경로**다.

- **금지**: 직전 loop-consistent 해에서 크게 떨어진 actuated seed 를 한 번의 Newton 사영에 통째로 넘기는 것. proto_1b(5-loop hand) 실측 이탈 임계는 seed 증분 **0.087~0.12 rad**이며, 그 위에서는 passive 가 180° 뒤집히거나 여러 바퀴 감긴 해로 착지한다.
- **비-RT**: `ProjectPassiveWithContinuation` 을 쓴다 (증분을 `kDefaultActuatedIncrement` 단위 sub-step 으로 분할, warm-start 연쇄). `ProjectPassiveToConstraint` 직접 호출은 증분이 작다고 보장될 때만.
- **RT**: sub-step loop 은 입력 의존 → 비결정적이라 쓸 수 없다. 대신 **tick 당 seed 증분을 균일 스케일로 클램프**하고 그 tick 을 `held` 로 보고한다 — tick loop 자체가 continuation 경로가 되어 고정 K 를 유지한다. per-joint 클램프는 homotopy 경로를 꺾으므로 금지.
- **회귀 테스트 필수 요건**: "고친 경로가 옳다"만 검증하면 vacuous 하다. **끈 경로(continuation/clamp 비활성)가 실제로 분기를 이탈하는지**를 같은 테스트가 확인해야 픽스처가 결함을 계속 재현함이 보장된다.

세 가지 파생 규칙 — 완화 장치가 스스로 결함을 만들지 않도록 (PR #249 리뷰):

- **비수용(!acceptable) sub-step 을 warm-start 로 이어가지 말 것.** 그 해는 loop-consistent 가 아니라 homotopy 보장이 깨졌는데, 마지막 sub-step 만 통과하면 성공으로 보고돼 소비자가 커밋한다 — 단일 사영에는 없던 **실패 은폐** 경로다. 중간 실패는 즉시 반환해 기존 hold 정책에 맡긴다. 판정 술어는 `converged`(strict) 가 아니라 `acceptable` 이다 (#250) — URDF 좌표 불일치의 residual floor (≤1 µm, 형상 무관 상수) 는 strict 를 원리적으로 통과할 수 없지만 실질 loop-consistent 라 warm-start 로 안전하고, 분기 간 거리는 rad 단위라 µm floor 로는 homotopy 가 훼손되지 않는다. strict 로 판정하면 floor 로봇에서 continuation 이 항상 첫 sub-step 에서 끊긴다. 단 acceptance 임계(병진 1e-6 m 기본)를 분기 판정에 쓰는 것은 여전히 금지 — 위 원칙 그대로, 분기는 residual 크기가 아니라 경로가 결정한다.
- **완화는 발동한 tick 에만 적용할 것.** 클램프 미발동 tick 까지 `prev + (q_a − prev)` 재구성 경로를 태우면 부동소수 왕복에서 1 ulp 가 새어 serial 등가(구속 없는 모델 = 개방 체인 FK)가 조용히 "근사"로 격하된다. 미발동 경로는 측정값을 그대로 대입한다.
- **sub-step 수 cap 은 증분 상한 보장을 깬다.** cap 에 걸리면 sub-step 증분이 `max_actuated_increment` 를 넘으므로, cap 은 "무한 루프 방지"용 여유값이어야지 정상 동작 범위를 자르면 안 된다.

**hold 는 열화 신호이지 자기 치유 보장이 아니다.** RT clamp 의 `held` 는 walk-in(`⌈Δ/증분⌉` tick 뒤 자연 해제)뿐 아니라 해를 커밋하지 않는 가드(분기 이탈·비유한)에도 서며, 후자는 **입력이 그대로면 무기한 지속**된다. 그동안 소비자는 조용히 last-good / open-chain fallback 으로 돈다. 따라서 held 를 fault 로 승격하는 것은 금지(정상 walk-in 을 죽인다)하되, **연속 held tick 수를 관측 가능하게 노출**하고 예상 walk-in 을 크게 넘으면 off-RT 진단으로 넘긴다.

근거·실측: issue #248, PR #249 리뷰.

## 이 파일의 규칙을 건드려야 할 것 같을 때

1. 수정 **전** `[CONCERN]` 보고 (Severity 는 위 표의 해당 규칙 열을 그대로 인용) — 포맷 SSoT 는 [CLAUDE.md](../CLAUDE.md) §6 `[CONCERN] 포맷`. `Detail` 에 어떤 invariant 에 저촉되는지·영향 범위, `Alternative` 에 우회 안 1개 이상 (interface 추가 / SPSC defer / aux thread 이동 등).
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
