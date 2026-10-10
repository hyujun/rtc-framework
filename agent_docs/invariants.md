# Invariants

이 파일의 규칙은 **위반 시 아키텍처가 깨진다**. 건드려야 할 것 같으면 코드를 수정하기 **전에** `[CONCERN]` 을 보고하고 사용자 컨펌을 받는다 (§Escalation Triggers).

- **Severity 는 파일이 아니라 규칙 단위다** — §Escalation Triggers 표와 Architecture 표의 Severity 열이 정한다.
- **invariant ↔ E 번호는 1:1 이 아니다** — 이름으로 지목된 invariant 만 전용 E 번호를 갖고 (ARCH-1→E-2, ARCH-3→E-4, ARCH-5→E-10), 나머지 위반은 **E-1 (Critical)** 로 수렴한다.
- **탐지 sensor 의 blocking 여부와 escalation severity 는 다른 축이다** — non-blocking sensor 가 경고만 내는 규칙 (ARCH-6) 도 규칙 자체를 바꾸려면 E-1 컨펌이 필요하다.
- **ID 는 재사용하지 않는다** — 은퇴한 번호 (RT-7, AP 결번) 는 비워 둔다.

이유·실측·이력·구현 위치는 헌법 밖 [invariants-rationale.md](../docs/reference/invariants-rationale.md), 재발 사례는 헌법 밖 [anti-patterns.md](../docs/reference/anti-patterns.md), ARCH 의 설계 근거는 [design-principles.md](design-principles.md) 가 갖는다.

## RT Path Invariants

**RT path** = `control_rate` YAML 로 설정된 정기 tick **또는 SCHED_FIFO dedicated-core 로 실행되는 모든 thread** 에서 실행되는 모든 경로. 프레임워크는 rate-agnostic 이다 (설계 범위 100 Hz–5 kHz, default 500 Hz; 상수 `rtc::kMin/kMax/kDefaultControlRateHz`) — default 를 가정으로 박지 않으며, RT 안전성은 *모든* 지원 rate 에서 성립해야 한다.

**어떤 콜백이 RT 인지는 함수 이름이 아니라 그 콜백이 붙은 executor 의 스케줄러가 결정한다.** 판정의 SSoT 는 [architecture.md](architecture.md) §Execution Contexts 표, 개별 판정 절차는 [.claude/rules/rt-path.md](../.claude/rules/rt-path.md) 다. 같은 "구독 콜백" 이라도 callback group 에 따라 구속 여부가 반대다. lifecycle 콜백 (`on_configure` / `on_activate` / `on_deactivate` / `on_cleanup`) · aux thread · 파라미터 콜백 · test / init 코드는 비-RT 다.

### RT callback rule

RT path 에 포함되는 subscription / UDP receive / timer callback 은 **mailbox-only** 로 운영한다. 무거운 연산은 callback 에서 하지 않고, `ControlLoop()` 가 다음 tick 에 mailbox snapshot 을 읽어 처리한다.

**허용 (mailbox write)**: fixed-size 메시지 필드의 단순 복사 (`Eigen::Map`, `std::memcpy`) · `std::atomic<T>::store` / `SeqLock<T>::Store` / SPSC enqueue · monotonic clock timestamp 캡처.

**금지**:
- `tf2_ros::Buffer::lookupTransform`, `Node::get_parameter` (내부 mutex)
- 동적 할당 (`std::string` 변환 포함), string formatting (`fmt::format`, `std::to_string`, `std::ostringstream`)
- `std::function` 재바인딩, `std::bind`
- `RCLCPP_*` 직접 호출 (RT-3), `std::condition_variable` 의 `notify_*` / `wait*` (RT-10)
- 컨트롤러 / lifecycle state transition
- 무거운 수치 연산 — `ControlLoop()` 으로 위임

### Clock 시간축 규칙

**Topic 경계 (`header.stamp`)** 는 ROS wall clock (CLOCK_REALTIME), **내부 timing / watchdog / staleness** 는 monotonic (`std::chrono::steady_clock`) 이다. **`header.stamp` 를 staleness / E-STOP / deadline 판단에 쓰지 않는다** (wall clock 은 NTP 로 역행·점프한다).

**기록된 예외 — 원격 예측 궤적의 물리 샘플 시각** (dynamic_catching D-2): 수신 콜백이 `t_ref_steady = recv_steady − (recv_wall − stamp)` 를 1회 계산해 원격 예측의 물리 시각축을 steady 로 옮기는 것은 다음을 **모두** 만족할 때만 허용하고, 하나라도 깨지면 E-1 이다. 이 예외는 다른 토픽의 근거가 아니다. stamp 사용 계약: [L1_io.md](../docs/dynamic_catching/ref/L1_io.md) §4.1.

- ① freshness·stale·watchdog 판정은 `now_steady − recv_steady` 로만 한다
- ② 보정항이 음수로 `future_tol` 을 넘으면 메시지를 거부하고 센다
- ③ 송·수신이 같은 호스트의 CLOCK_REALTIME 을 공유하거나 PTP 동기가 검증된 경우로 한정한다
- ④ 보정항 분포를 진단으로 발행해 점프를 관측할 수 있게 한다

### RT pub/sub primitive catalog

RT path 의 publisher / state buffer / queue 선택 기준. 1순위 (wait-free + heap-free + single-owner) 가 default 이고, 정당한 이유가 있을 때만 2순위로 내려간다.

| 등급 | Primitive | 조건 |
|------|-----------|------|
| 1 | `SeqLock<T>` (`rtc_base`) | latest-only state 의 default. `T` 는 `trivially_copyable` 필수 |
| 1 | `SpscQueue<T,N>` / `SpscPublishBuffer<N>` (`rtc_base`) | RT loop → aux thread. single-writer / single-reader 강제 |
| 1 | `std::atomic<T>` | POD 만. `is_always_lock_free` 확인 |
| 2 | `realtime_tools::LockFreeQueue<T, spsc_queue>` | `SpscQueue` 와 등가 — 외부 의존성 추가를 정당화할 때만 |
| 2 | `realtime_tools::RealtimePublisher::try_publish` | 신규 단일-토픽 publisher 에 한해 검토. dedicated thread 가 생기므로 RT-HOST-2/3 정합 + AP-RTT-1 필수 |
| 2 | `realtime_tools::RealtimeBuffer<T>` / `RealtimeThreadSafeBox<T>` | ctor / `reset` 이 heap 을 할당하므로 lifecycle 콜백에서만 (AP-RTT-1) |
| 금지 | `std::atomic<std::shared_ptr<T>>`, `SeqLock<std::shared_ptr<T>>` | wait-free 가 아니다 (RT-8) |
| 금지 | `std::mutex::lock` / `lock_guard` / `scoped_lock` | RT-4. `try_to_lock` 은 best-effort 로 별개 |
| 금지 | `std::shared_ptr` 복사 | RT-8 |
| 금지 | `std::condition_variable` / `std::condition_variable_any` | RT-10 |

- 신규 latest-only POD state buffer → `SeqLock<T>`. 신규 single-writer queue → `SpscQueue<T,N>`.
- 기존 SPSC + publish_thread 멀티플렉싱 인프라는 **교체 금지**.
- MPSC / MPMC 가 필요해 보이면 single-writer 로 재설계할 수 있는지부터 검토한다 (ARCH-3). 그래도 필요하면 별도 sprint.

### RT 금지 패턴

| # | 금지 | 대안 |
|---|------|------|
| RT-1 | `new` / `malloc` / `push_back` / `emplace_back` / `resize` | `std::array`, 사전 할당된 fixed-size `Eigen::Matrix<fixed>` |
| RT-2 | `throw` / `catch` | Error code, `std::optional`, `std::expected` |
| RT-3 | 정기 tick 에서 `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 | SPSC log buffer → `DrainLog()` aux thread. 허용 범위는 §RT-3 세부 스펙 |
| RT-4 | `std::mutex::lock()` / `std::lock_guard` / `std::scoped_lock` | `SeqLock<T>` / `SpscQueue<T,N>` / `std::atomic<T>`, last resort `std::try_to_lock` (§RT pub/sub primitive catalog) |
| RT-5 | `auto` with Eigen expression | 명시 타입: `Eigen::MatrixXd M = ...` |
| RT-6 | Quaternion `lerp` / `nlerp` | `Eigen::Quaterniond::slerp(t, q_b)` only |
| RT-7 | *(은퇴 — [PROC-6](#process-invariants) 으로 이동)* | — |
| RT-8 | `std::shared_ptr` 복사 | Raw ref 또는 `const std::shared_ptr<T>&` |
| RT-9 | RT tick 또는 RT callback 에서 `get_lifecycle_state()` / `get_current_state()` 호출 | `on_activate` 종료 직전 `std::atomic<uint8_t>` 캐시에 `store(..., memory_order_release)`, RT loop 는 `load(memory_order_acquire)` |
| RT-10 | `std::condition_variable` / `std::condition_variable_any` 의 `notify_*` / `wait*` (RT path 의 producer 든 consumer 든) | (a) eventfd + non-blocking write/poll, (b) `SpscQueue<T,N>` + consumer polling, (c) `std::atomic<bool>` flag + self-clocked consumer (고정주기 loop 이 매 tick latch) |

- **RT-10 은 self-termination 에도 적용된다** — RT 핫패스에서 자기 loop 을 끝낼 때 (E-STOP 등) mutex + `notify` 를 잡는 `RequestStop()` 을 부르지 않고 `PeriodicRtThread::RequestLoopExit()` (순수 atomic store) 로 예약한다. 종료 시 부수 작업은 loop unwind 뒤 1회 호출되는 `OnLoopAborted()` 에서 한다.

#### 알려진 위반 — 예외가 아니라 기록이다

기록된 위반은 **그 경로에 한정**되며 새 RT 코드가 할당할 근거가 아니다 ("DDS 도 하는데" · "ORT 도 하는데" 는 근거가 아니다).

1. **actuator lane 의 DDS publish** (`DeviceBackend::WriteCommand`, #222) — 할당은 DDS 구현 안에서 일어난다. 수용은 이 한 경로뿐이다.
2. **ONNX Runtime `Run()`** (조건부 수용) — 수용 범위는 `DemoInferenceController::Compute()` 정책 step 의 `InferenceEngine::Run()` 과 `udp_hand_driver` `FingertipFTInferencer::Infer()` 의 `RunModels()` 두 호출 지점뿐이고, 같은 tick 의 나머지 (pack·unpack·FK·명령 tail) 는 RT-1 그대로다. 아래가 **모두** 성립할 때만 수용이며 하나라도 깨지면 E-1 이다:
   - **heap 정책** — RT 프로세스 main 에서 `rtc::ConfigureRtHeap()` + `mlockall` (RT-HOST-1)
   - **정상상태 heap 무성장** — warmup 뒤 반복 Run 동안 `mallinfo2()` 의 `arena`·`hblkhd` 불변
   - **호출자 몫 0** — 정책 tick 의 operator new 수 == `Run()` 의 operator new 수
   - **RT 조건 실측** — 정책 step tick 의 compute p99/max 가 tick 예산 안 (SCHED_FIFO). 제어 PC 실측이 실기 투입의 gate 다
3. **MPC thread 의 cross-mode swap** (`HandlerMPCThread::Solve` 의 `MPCFactory::Create`) — **수용이 아니다**. RT-1·RT-2 위반으로 기록돼 있고 해소는 별도 작업이다.
4. **외부 수치 라이브러리 안의 할당** (ProxQP · Pinocchio 등, #654) — 수용이며, 기록만 하고 0 을 단언하지 않는다. 범위는 경로가 아니라 **그 라이브러리가 자기 구현 안에서 하는 할당**이다 (스레드 무관 — 우리 TU 의 Eigen 식은 아니다). 호출 전후의 우리 코드는 RT-1 그대로이며, 지금 C 할당 0 을 단언하는 게이트를 이 항목으로 풀지 않는다 (E-6).

#### 위반 탐지 패턴

아래 패턴을 편집 중인 RT 파일에 대해 실행한다 (`<RT file>` 자리에 대상 경로). 각 블록의 `# probe:` 는 그 패턴이 **반드시 매치해야 하는** 라인, `# antiprobe:` 는 **매치하면 안 되는** 라인이며 [validate_docs.py](../repo_scripts/scripts/validate_docs.py) 가 검증한다. 패턴은 표 셀이 아니라 fenced `detect` 블록에 두고, **빈 alternation 분기** (`(|_for)`) 를 쓰지 않는다 — `(_for)?` 로 쓴다.

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

위반 건수·현 호출부는 여기 적지 않는다 — detect 블록으로 직접 확인한다 (AP-DOC-1).

### RT-3 세부 스펙

- **정기 tick 경로**: `RCLCPP_*` 직접 호출 금지. SPSC → aux 로 defer 한다.
- **One-shot init 경로 (허용)**: `init_timeout` fatal, 최초 1회 초기화 로그처럼 1회 발생 후 `rclcpp::shutdown()` 또는 활성화 완료로 더 이상 실행되지 않는 분기.
- **THROTTLE 변종 (허용, 단 msg 는 RT-safe)**: `RCLCPP_*_THROTTLE` 은 허용하되 msg 는 단순 format string + 기본 타입 (`int`, `double`, `const char*`, fixed-size `std::array<char, N>::data()`) 만 쓴다. 문자열 concat, `std::stringstream`, `fmt::format`, `std::to_string` 은 금지 (내부 heap alloc).
- THROTTLE 도 매 호출 clock access + format 평가 비용이 있다 — SPSC → `DrainLog()` 가 우선이고 THROTTLE 은 최후 수단이다.

### RT telemetry rule

RT loop 내부 통계는 **O(1) / fixed-size / allocation-free** 만 허용한다 (구간 시간 누적, `std::max` 갱신, `std::atomic` 카운터, fixed-size ring buffer write). percentile · histogram · CSV write · ROS publish 는 RT 가 push 한 snapshot 을 받아 **aux thread 또는 비-RT** 에서만 한다.

## RT Host / Runtime Preconditions

RT controller 가 운영·배포 모드로 실행될 때 host 가 만족시켜야 하는 조건이다. RT path invariant 가 모두 지켜져도 host 가 잘못 설정되면 RT 안정성이 무너진다 — **실패 시 코드를 고치기 전에 host/runtime 문제인지 controller code 문제인지부터 분리한다.**

| # | 규칙 | 검증 |
|---|------|------|
| RT-HOST-1 | RT 프로세스는 main 진입 또는 `on_configure` 종료 시점에 `mlockall(MCL_CURRENT \| MCL_FUTURE)` 를 1회 호출한다 | `repo_scripts/scripts/verify_rt_runtime.sh` (`VmLck>0`) |
| RT-HOST-2 | RT thread 는 SCHED_FIFO priority ∈ [50, 95] 로 `on_activate` 시점에 설정한다. **99 금지** (kernel watchdog 영역). 값은 `rtc_base/threading/thread_config.hpp` 의 `SystemThreadConfigs` 가 SSoT | `verify_rt_runtime.sh` |
| RT-HOST-3 | RT thread 는 `thread_config.hpp` + `cpu_shield.sh` 가 정의한 CPU core 에 affinity 를 고정한다 | `verify_rt_runtime.sh` |

- **외부 라이브러리가 만드는 thread** (`realtime_tools`, DDS 등) 도 같은 layout 에 맞춘다 — 라이브러리는 priority/affinity 를 설정하지 않으므로 호출자가 native handle 로 명시한다 (RT-HOST-2/3, AP-RTT-2).
- **System-level 조건** (PREEMPT_RT 또는 lowlatency 커널, `@realtime` 그룹의 `rtprio`·`memlock`, isolated RT core 에 non-RT 작업 미공유) 은 배포 환경 책임이다. 수치·명령은 여기 박제하지 않고 [check_rt_setup.sh](../repo_scripts/scripts/check_rt_setup.sh) · [verify_rt_runtime.sh](../repo_scripts/scripts/verify_rt_runtime.sh) 에 위임한다.

## Architecture Invariants

| # | 규칙 | Severity (§Escalation Triggers) | 탐지 |
|---|------|---|-----------|
| ARCH-1 | `rtc_*` 패키지에 로봇 이름·joint 수·HW ID 하드코딩 금지 | **Critical** (E-2) | 자동 — hook Phase 0 |
| ARCH-2 | 의존성 그래프 상향 의존 금지 ([architecture.md](architecture.md) §Dependency Graph) | **Critical** (E-1) | 수동 리뷰 |
| ARCH-3 | Abstract interface 없이 두 번째 구체 구현 추가 금지 | Warning (E-4) | 수동 리뷰 |
| ARCH-4 | integration 패키지가 `rtc_*` private 헤더 (`rtc_*/src/`) include 금지 | **Critical** (E-1) | 자동 — hook Phase 0 |
| ARCH-5 | `robot_descriptions` 는 data-only — build-time 의존 금지 | Warning (E-10) | 수동 리뷰 |
| ARCH-6 | 모든 ROS 2 topic 은 QoS history `KEEP_LAST`, depth **1** (reliability/durability 는 lane 별 유지 — depth 필드만 강제) | Warning (sensor) / E-1 (규칙 변경) | 자동 (non-blocking) — hook Phase 0b |
| ARCH-7 | `rtc_*` 는 control-framework runtime identity (RT 제어 루프를 구동하는 exec) 를 소유하지 않는다 | Warning (sensor) / E-1 (규칙 변경) | 자동 — hook Phase 0a |

**탐지의 SSoT 는 [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) 다** — ARCH 계열 패턴·스코프·면제 규칙을 문서에 복제하지 않는다. RT 계열은 반대다: hook 은 RT 검사를 구현하지 않고 (파일 단위로는 RT 경로를 가릴 수 없다) §위반 탐지 패턴 의 detect 블록이 갖는다. 편집 시점의 판정 기준은 [.claude/rules/](../.claude/rules/) 의 `arch-source.md` · `arch-build-meta.md` 가 갖는다.

**ARCH-7 의 범위·예외·`ARCH-7-exempt` 마커** 는 [design-principles.md](design-principles.md) §Boundary Rules 가 갖는다.

### ARCH-6 세부 스펙

- **depth 만 움직인다** — reliability / durability 는 절대 함께 바꾸지 않는다 (`transient_local`, `best_effort`, `reliable` 은 그대로).
- 인자 없는 `SensorDataQoS()` 도 대상이다 — `.keep_last(1)` 이 필요하다.
- 적용 범위는 프로덕션 C++ + Python 이고 test fixture 는 면제다.
- **예외는 매 샘플 누적이 계약인 lane 뿐이다.** latest-value 소비자 (wrench, marker, state 토픽) 는 예외가 아니다. 예외가 필요하면 `[CONCERN]` (E-1) 로 보고한 뒤 QoS 코드 라인 끝에 `// ARCH-6-exempt` 주석 + 사유를 남기고 아래에 기록한다.
- **기록된 예외**: ToF snapshot 토픽 (`<ns>/tof/snapshot`) — subscriber 가 매 샘플을 누적하므로 deep best_effort 큐를 유지한다.

### ARCH-5 세부 스펙

`robot_descriptions` 는 빌드되는 것이 0 인 data-only 패키지이고, "워크스페이스 어디 두든 `install/robot_descriptions/share/` 만 있으면 동작" 하는 모델을 지킨다.

**허용**:
- `package.xml` 의 `<exec_depend>robot_descriptions</exec_depend>`
- `package.xml` 의 `<test_depend>robot_descriptions</test_depend>` — **테스트가 아래 런타임 lookup 을 쓸 때 설치 순서를 보장하는 용도로만**
- 런타임 lookup: `ament_index_cpp::get_package_share_directory("robot_descriptions")` / Python `get_package_share_directory("robot_descriptions")`
- URDF/MJCF/launch/YAML 의 `package://robot_descriptions/robots/<name>/...` URL, 또는 런타임에 resolve 되는 패키지명 문자열

**금지**:
- `find_package(robot_descriptions ...)`, `<depend>` / `<build_depend>`, `ament_target_dependencies(... robot_descriptions)`, `ament_export_dependencies(... robot_descriptions)`
- 소스 트리 상대경로 (`${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/...`) 로 configure 시점에 경로를 박는 것

**복구**: build-dep 줄 제거 + `<exec_depend>` 로 강등. `robot_descriptions` 가 C++ 라이브러리를 export 해야 하게 되면 별도 패키지 (`robot_descriptions_utils` 등) 로 split 하고 이 invariant 는 유지한다.

## Process Invariants

| # | 규칙 |
|---|------|
| PROC-1 | 코드 변경 시 대응 문서·YAML·CMakeLists·package.xml 을 동기화한다 ([modification-guide.md](modification-guide.md) Completion Checklist) |
| PROC-2 | 공개 API 변경 시 downstream 패키지를 재빌드·재테스트한다 |
| PROC-3 | `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·전체 테스트 |
| PROC-4 | E-STOP trigger 는 idempotent 다 (`compare_exchange_strong`) |
| PROC-5 | C++ ↔ Python 미러 쌍 ("동일 로직" 을 표방하는 두 구현) 은 한쪽만 고치지 않는다 — 동시 수정 + 동등성 테스트 통과 |
| PROC-6 | 기존 test assertion 을 통과시키려 **약화·수정 금지** (회귀 은폐). assertion 이 진짜 틀렸거나 spec 이 바뀐 경우는 정당한 변경이다 — 착수 전 E-6 로 escalate 하고, 새 코드 fix 와 **별도 commit** 으로 근거 제시 + 대응 regression test 갱신. 지키려는 것은 commit 개수가 아니라 *assertion 변경의 감사 가능성* 이므로, 사용자 지시로 **squash 머지** 하는 경우에는 근거 (무엇을 무엇으로 바꿨는지 · 왜 회귀 은폐가 아닌지) 를 squash 본문에 옮겨 적는다. 탐지: `git diff test/` 의 `EXPECT_*` / `ASSERT_*` 상수 변경 |
| PROC-7 | Controller-owned SeqLock (`GraspState` / `WbcState` / `ToFSnapshot`) 은 `Compute()` 가 도는 **모든** tick 에서 Store 한다 — E-STOP·early-return tick 포함. 이번 tick 에 계산하지 않은 필드는 값을 얼리지 말고 명시적으로 무효화한다 (`FillEstopPublishState`). Store 생략은 미발행이 아니라 stale body + fresh stamp 재발행이다. 탐지: `Compute()` 의 early-return 경로에 `*_state_lock_.Store` 가 없는 분기 |
| PROC-8 | **테스트의 두 대기 원시를 섞지 않는다.** `rtc_base/test/include/rtc_base/testing/` 의 공유 대기 헬퍼는 **sleep-only** 로 유지한다 — executor 를 pump 하는 코드 (`spin_some` / `spin_once` / `spin_until_future_complete` / `add_node`) 를 넣지 않고, 호출자가 spin 루프로 감싸지도 않는다. Executor pump 가 필요한 테스트는 **자기 TU 에 local spin 헬퍼** 를 두어 무엇을 spin 하는지가 assertion 옆에 남게 한다. 탐지: hook Phase 0c |

## Numerical Invariants

| # | 규칙 |
|---|------|
| NUM-1 | 특이점 근처는 damped pseudoinverse 필수 (`max_damping` / `singularity_threshold` YAML 주입). 램프의 **양 끝단이 모두** 하한을 받는다 — λ_max 는 `compliance::FloorMaxDamping`, σ₀ 는 `compliance::FloorSigma0` 을 **로더와 값을 쓰는 지점 양쪽**에서 (§NUM floor 규약). 두 하한 상수의 정의는 한 곳 (지키는 법칙 옆) 이고 λ 규약은 compliance §6.5 하나뿐이다 |
| NUM-2 | `dt` near-zero guard (모든 trajectory generator) |
| NUM-3 | Quaternion 은 매 곱 뒤 정규화한다 |
| NUM-4 | `trajectory_speed` / `trajectory_angular_speed` 는 `std::max(1e-6, val)` 로 클램프한다 — YAML 과 `ros2 param` **양쪽 진입점** 모두 |
| NUM-5 | 폐쇄 체인 사영은 seed 증분 제한이 필수다. residual 로 조립 분기를 판정하지 않는다 (§NUM-5 세부 스펙) |
| NUM-6 | 영공간 자세 게인은 `rtc::FloorNonNegativeGain` 하한을 **로더와 사용 지점 양쪽**에서 받고, 사용 지점 하한은 활성 게이트 **판정 앞**에 둔다. 로더 쪽은 키 유무와 무관하게 무조건이며 `if (!cfg)` 경로를 포함한다 |
| NUM-6a | NUM-6 의 파생 규칙 둘 (compliance §6.4 임계감쇠 보정 `K_dⁿ ≥ 2√K_pⁿ`, compliance §6.1 `TRANSLATION_ONLY` 가드) 은 floor **뒤**에 두되 **configure 에만** 둔다 — 순서는 floor → compliance §6.4 → compliance §6.1 이고 YAML 경로와 `if (!cfg)` 경로가 같다. compliance §6.1 의 런타임 등가물은 바인딩 요구사항이다: `set_gains()` 상당 경로로 `TRANSLATION_ONLY` + `K_pⁿ ≤ 0` 에 도달하면 `ComplianceFaults::posture_authority_lost` (DEGRADED) 를 세운다. 이 한 필드에만 setter 를 달지 않는다 — 상태 머신을 배선하는 바인딩이 전체를 함께 세운다 |
| NUM-6b | compliance §5.3 안전층 게인도 같은 하한을 받는다 — `joint_limit_stiffness` · `joint_limit_damping` 은 `rtc::FloorNonNegativeGain`, `joint_limit_margin` (δ) 은 floor 가 아니라 **configure 거부** (`>= 0` 이고 유한). 세 검사 모두 키 유무와 무관하게 무조건이며 `if (!cfg)` 조기 반환 경로도 포함한다. 파서는 밴드를 못 보므로, δ 가 밴드를 뒤집는 경우는 바인딩의 `on_configure` 가 device 한계와 대조해 거부한다 (AP-PROC-9) |
| NUM-7 | **크기 비교로 만든 가드는 유한성 게이트가 아니다.** NaN 은 모든 비교에서 false 이므로 `x <= 0` · `count == 0` · `n < min` 류 검사를 통과한다. 비유한 값이 들어올 수 있는 경로는 **발행 직전 값**에 `allFinite()` / `std::isfinite` 판정을 따로 두고, 실패는 그 경로가 **이미 가진** invalid / hold-last 의미론으로 되돌린다 (새 실패 모드를 만들지 않는다) |

**NUM-7 판정**:

- 가드가 있다는 사실은 유한성 커버리지의 증거가 아니다. 유한성 검사는 부호 검사를 대체하지도, 부호 검사에서 파생되지도 않는다 (`!(x >= 0.0)` 는 `+inf` 를 통과시킨다) — **둘 다 쓴다**.
- "off" 가 `<= 0` 인 스위치 값의 가드는 `> 0` 이 아니라 `isfinite` 다 — 비양수는 정당한 비활성화이고 비유한 값만 거부한다.
- `std::max` / `std::min` 으로 박스를 조립하는 것도 NaN 을 세탁한다 — 조립 전에 입력의 유한성을 본다.
- configure 게이트는 정당한 등호 (`q_min == q_max` 처럼 잠긴 joint) 를 거부하지 않는다 — 과대 엄격의 대가는 성능 저하가 아니라 기동 실패다.
- 저장소에 `-ffast-math` / `-Ofast` 를 넣지 않는다 (유한성 검사가 최적화로 사라진다).

### NUM floor 규약 (NUM-1 · NUM-6 · NUM-6b 공통)

- **손수 `std::max` 를 쓰지 않는다** — `max(하한, NaN) == 하한` 이라 비유한 값을 그럴듯한 값으로 세탁해 fail-loud 를 fail-silent 로 바꾼다. helper (`FloorMaxDamping` / `FloorSigma0` / `FloorNonNegativeGain`) 는 비유한 값을 그대로 통과시켜 하류 finite 검사 (`compliance::AllFinite`, actuator 경계의 `urtc::ValidateControllerOutput`) 로 보낸다.
- **사용 지점 쪽은 바인딩 요구사항이다** — `set_gains()` 상당 경로로 게인 POD 를 SeqLock 에 직접 쓰는 바인딩은 tick 에서 다시 floor 를 건다.
- **법칙은 자기 인자를 floor 하지 않는다** (`AdaptiveDampingSquared`, `AddJointLimitRepulsive`) — floor 는 로더와 사용 지점의 몫이다.

### NUM-5 세부 스펙

점 (`CONTACT_3D`) 구속으로 닫은 loop 은 조립 분기가 둘 이상이고 각 분기가 φ=0 을 정확히 만족한다. `converged` / `acceptable` / `closure_error` 를 아무리 엄격하게 잡아도 **물리적으로 틀린 분기를 검출할 수 없다** — 분기를 결정하는 것은 residual 이 아니라 seed 에서 해까지의 경로다.

- **금지**: 직전 loop-consistent 해에서 크게 떨어진 actuated seed 를 한 번의 Newton 사영에 통째로 넘기는 것. λ 상향은 대체 수단이 아니다.
- **비-RT**: `ProjectPassiveWithContinuation` 을 쓴다. `ProjectPassiveToConstraint` 직접 호출은 증분이 작다고 보장될 때만.
- **RT**: sub-step loop 은 쓸 수 없다 (입력 의존 → 비결정적). **tick 당 seed 증분을 균일 스케일로 클램프**하고 그 tick 을 `held` 로 보고한다. per-joint 클램프는 금지.
- **회귀 테스트**: "고친 경로가 옳다" 만 검증하면 vacuous 하다. **끈 경로 (continuation / clamp 비활성) 가 실제로 분기를 이탈하는지**를 같은 테스트가 확인한다.
- **비수용 (`!acceptable`) sub-step 을 warm-start 로 이어가지 않는다** — 중간 실패는 즉시 반환해 기존 hold 정책에 맡긴다. 판정 술어는 `converged` (strict) 가 아니라 `acceptable` 이다. 단 acceptance 임계를 분기 판정에 쓰는 것은 여전히 금지.
- **완화는 발동한 tick 에만 적용한다** — 미발동 경로는 측정값을 그대로 대입한다 (재구성 경로를 태우지 않는다).
- **sub-step 수 cap 은 무한 루프 방지용 여유값이다** — 정상 동작 범위를 자르면 증분 상한 보장이 깨진다.
- **hold 는 열화 신호이지 자기 치유 보장이 아니다** — `held` 를 fault 로 승격하지 않되 (정상 walk-in 을 죽인다), **연속 held tick 수를 관측 가능하게 노출**하고 예상 walk-in 을 크게 넘으면 off-RT 진단으로 넘긴다.

## Anti-pattern 규범

대응하는 invariant 행이 없는 재발 패턴의 규범이다. ID 정의·사례·복구는 [anti-patterns.md](../docs/reference/anti-patterns.md).

| # | 규범 |
|---|------|
| AP-RT-6 | 여러 필드로 된 공유 상태를 보호 없이 복사하지 않는다 (torn read) — `SeqLock<T>` Load 또는 `std::atomic<T>` (POD) |
| AP-RT-8 | RT tick 에서 `std::string` 을 **값으로 반환**하는 접근자 (`Get*DeviceName()` 류) 를 호출하지 않는다 — 짧은 이름은 SSO 로 alloc 게이트를 통과하다가 이름이 길어지는 날 할당한다. configure 에서 해석해 POD 멤버로 캐시하고, tick 에서 이름으로 조회하지 않는다 |
| AP-RTT-1 | `realtime_tools` primitive 를 도입하는 순간부터: ① `RealtimePublisher` 의 dedicated thread 는 priority/affinity 를 명시해 layout 에 맞추거나 (E-7) 기존 SPSC + 단일 publish thread 에 멀티플렉싱한다, ② `RealtimeBuffer` 의 ctor · `reset()` 은 `on_configure` / `on_cleanup` 에서만 부른다, ③ `try_publish` 의 false (silent drop) 는 호출 site 마다 drop 카운터로 세어 aux 에서 내보낸다 |
| AP-THREAD-1 | `ThreadConfig::cpu_core` 는 kernel logical CPU id 가 아니라 **slot index** 다. affinity 호출 (`CPU_SET` / `CPU_ISSET`) 은 반드시 `SlotToLogicalCpu()` 를 거치고, unit test 는 `CpuTopology` 를 주입한다 (SMT-off mock 으로는 회귀가 안 보인다) |
| AP-ARCH-4 | device group 경계를 넘는 공용 버퍼 · state publisher 를 만들지 않는다 — device group 당 별도 publisher, state / target 모두 `device_idx` 로 tagging |
| AP-CTRL-1 | 게인은 `Compute()` 진입 시 **단일 snapshot** 으로 읽는다 — tick 중간에 다시 읽으면 writer 가 끼어든 절반 상태로 분기한다 |
| AP-CTRL-5 | joint reorder map 은 config 로드 시 1회 계산하고 **identity fallback 을 두지 않는다**. 인덱싱이 device 순서인지 config 순서인지 명시하고, position limit 과 velocity limit 은 따로 적용한다 |
| AP-PROC-1 | 완료를 주장하기 전에 대상 파일 목록을 명시하고 **전수** grep 한다 (예상치 vs 실측) |
| AP-PROC-5 | ROS 2 parameter 는 타입과 `ParameterDescriptor` 를 명시해 선언하고 (`declare_parameter<T>(name, default, descriptor)`), launch 의 LifecycleNode 는 `namespace=''` (빈 문자열) 로 둔다 |
| AP-PROC-6 | BT 노드를 신설하면 `registerNodeType<>` 등록과 `validate_tree()` 양쪽에 추가한다 |
| AP-PROC-7 | **추가** 변경에서 "회귀 0 fail" 은 커버리지가 아니다. 추가한 이름으로 grep 해 테스트가 그 이름을 아는지 본다 (하드카운트 단언은 추가만으로 안 깨진다). 모르면 목록을 늘리거나 기존 형제와의 등가성 테스트로 합성하고 (갈라지는 시점을 테스트에 적는다), 신규 쪽에만 건 mutation 이 red 를 내는지 확인한다 |
| AP-PROC-8 | 컨트롤러 픽스처는 `on_activate` 뒤에 target 을 넣는다 (Inactive 중의 target 은 버려진다). target 경로를 검사하는 테스트는 **이동량 단언** 을 비교·판정 앞에 둔다. 기존 테스트에 `on_activate` 를 넣는 것은 PROC-6 이 아니다 |
| AP-PROC-9 | configure-time 검증은 **그 값이 채워져 있고 거부 채널이 있는 pass** 에 둔다 — 축은 pass 번호가 아니라 값의 출처다 (3-pass 계약: [rtc_controller_interface/README.md](../rtc_controller_interface/README.md#lifecycle-훅-ros2_control-정렬-기본-구현-제공)). 모델 파생 값은 Pass 1 `LoadConfig` 에서 유효하고, device 파생 값 (한계 밴드) 의 검증은 Pass 3 `on_configure` 에 둔다. Pass 2 (`OnDeviceConfigsSet`) 는 파생·캐시 전용이며 거부하지 않는다. 검사는 RT 경로와 같은 fallback 으로 밴드를 해석하고, 테스트는 코드의 존재가 아니라 **거부 동작** 을 잘못된 config 와 정상 config 를 쌍으로 단언하며, 임계값은 device config 에서 유도한다 |
| AP-DOC-1 | 측정 시점에 의존하는 값 (패키지·테스트 수, 상수값, commit SHA, "현재 0건" · "✅ 완료" 류 status) 을 문서에 박제하지 않는다 — *재는 방법* 을 적고 값은 SSoT (코드 / git / 측정 명령) 에 맡긴다. 규칙 목록과 hook 동작의 사본도 같다: 헌법은 ID 범위와 SSoT 포인터만 갖고, "hook 이 이 목록과 같은 것을 한다" 는 등가 주장도 쓰지 않는다 |
| AP-DOC-2 | 판단의 **근거** 는 그 판단이 코드로 나타나는 지점 한 곳이 소유한다. 두 번째 위치는 자기가 소유한 규칙만 적고 나머지는 가리킨다. 주석·문서에 판단을 쓰기 전에 핵심 명사로 repo 를 grep 해 이미 서술한 곳이 있으면 가리킨다 |

```detect id=AP-THREAD-slot-mapping
grep -rnE 'CPU_(SET|ISSET)\((cfg\.)?cpu_core' rtc_base/include/rtc_base/threading/
# probe:     CPU_SET(cfg.cpu_core, &cpuset);
# antiprobe:     CPU_SET(SlotToLogicalCpu(cfg.cpu_core), &cpuset);
```

## Escalation Triggers (E-1 ~ E-11)

**이 절이 E-번호·severity·`[CONCERN]` 포맷의 SSoT 다** — tool-neutral 이며 [AGENTS.md](../AGENTS.md) §6 은 여기를 가리키기만 한다.

다음 상황에서는 코드를 쓰기 **전에** `[CONCERN]` 을 보고하고 사용자 컨펌을 기다린다.

| ID | Severity | 트리거 | 관련 규칙 |
|----|----------|--------|-----------|
| E-1 | **Critical** | 이 파일의 invariant 를 위반하거나 예외가 필요할 것 같음 | 전 규칙 (전용 번호가 없는 모든 위반이 여기로 수렴) |
| E-2 | **Critical** | `rtc_*` 패키지에 robot-specific 값을 넣어야 함 | ARCH-1 |
| E-3 | **Critical** | `rtc_msgs` / `shape_estimation_msgs` public ABI 변경 필요 — 기존 필드의 변경·삭제·재정렬**과** 새 `.msg`/`.srv`/`.action` **추가**를 모두 포함한다 (추가는 wire 호환이어도 PROC-3 전체 빌드와 소비자 동기화가 따른다) | PROC-3 |
| E-4 | **Warning** | Abstract interface 없이 두 번째 구현 추가 필요 | ARCH-3 |
| E-5 | **Warning** | Optional dep (MuJoCo, aligator) fallback 제거 필요 | — |
| E-6 | **Critical** | 기존 test assertion 을 약화·수정해야 할 것 같음 — 회귀 은폐 vs 정당한 spec 변경/test-bug 를 구분하고, 후자는 별도 commit + 근거 | PROC-6 |
| E-7 | **Critical** | Thread model (core 배치, priority) 변경 | RT-HOST-1~3 |
| E-8 | **Critical** | E-STOP 경로 수정 | PROC-4, PROC-7 |
| E-9 | **Warning** | 문서-코드 불일치를 어느 쪽에 맞출지 결정 필요 | PROC-1 |
| E-10 | **Warning** | `robot_descriptions` 를 build-time 으로 의존하려는 변경 (`find_package` / `<depend>` / `ament_target_dependencies`) | ARCH-5 |
| E-11 | **Warning** | `PublishRole` enum 에 controller-owned non-RT 토픽을 추가하려는 변경 — 새 controller-owned 토픽은 `SeqLock<T>` + `Setup*Publisher` 패턴 | ARCH-6 |

**Severity 의미**:

- **Critical** — 사용자 컨펌 전까지 커밋·PR 금지
- **Warning** — 사용자 판단에 따라 진행, 결정 로그 남김
- **Info** — 기록만, 진행 가능

**`[CONCERN]` 포맷**:

```text
[CONCERN] <한 줄 요약>
Severity: Critical | Warning | Info
Detail: <문제의 구체 내용, 저촉되는 invariant ID, 영향 범위, 검토한 대안>
Alternative: <우회 안 1개 이상 — interface 추가 / SPSC defer / aux thread 이동 등>
```

## 이 파일의 규칙을 건드려야 할 것 같을 때

1. 수정 **전** `[CONCERN]` 보고 — 트리거 표·severity·포맷은 위 §Escalation Triggers 가 SSoT.
2. 사용자 컨펌 후 진행.
3. "임시로 위반 → 나중에 정리" 는 허용되지 않는다. Warning 이상은 별도 리팩터 task 로 분리한다.

## False-positive 처리

§위반 탐지 패턴 의 grep 은 **path-blind** (RT path 밖 코드도 매칭) 이고 **role-blind** (one-shot init / aux thread 허용 케이스도 매칭) 다. 판단 절차:

1. **Path 확인**: 매칭된 파일·함수가 RT path 정의 (§RT Path Invariants) 에 들어가는가, 비-RT path 인가?
2. **Role 확인** (RT-3 한정): one-shot init 이거나 `RCLCPP_*_THROTTLE` + RT-safe msg 면 **허용** ([RT-3 세부 스펙](#rt-3-세부-스펙)).
3. **Aliasing 확인** (RT-5 한정): `auto` 가 받는 것이 단순 scalar / index 면 false-positive, Eigen expression (`.matrix()`, `.transpose()`, `.inverse()`, `.adjoint()`, `.block()`, `*` 연산) 이면 위반.

False-positive 판정이면 코드는 그대로 진행하고 한 줄로 보고한다: `false-positive: <rule-id> at <file:line>, reason=<RT path 외 / one-shot / scalar auto / ...>`. 같은 패턴이 반복되면 grep 을 좁히는 별도 task 후보이며 하네스 pruning 신호로 보고한다 (보고 경로는 각 도구의 문서가 소유 — [AGENTS.md](../AGENTS.md) §11).

**이 절차는 사람이 돌리는 grep 을 전제한다 — 차단형 sensor 에는 종결력이 없다.** 차단하는 gate 는 "false-positive 이니 그대로" 로 판정해도 다음 turn 에 같은 자리에서 다시 막는다. 종결 수단은 둘뿐이다 — (a) 지적된 줄을 고친다, (b) gate 자체의 수정을 `[CONCERN]` 으로 제안하고 컨펌을 받는다. 어느 쪽도 아닌 보고는 진행이 아니라 교착이다.

**금지**: false-positive 추정이라며 사용자 보고 없이 invariant 를 우회하는 것. 의심스러우면 `[CONCERN]` 절차를 따른다.
