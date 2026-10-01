# Invariants — 근거·실측·이력

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/invariants.md](../../agent_docs/invariants.md) 가 갖고, 여기는 그 규칙이 *왜* 생겼는지 · 무엇을 실측했는지 · 어디에 구현돼 있는지를 기록한다. 둘이 어긋나면 invariants.md 가 옳다. 수치·호출부·파일 위치는 **기록 시점의 것**이며 다시 재야 한다 — 재는 방법은 각 항목이 가리키는 issue·테스트·스크립트에 있다.

## 문서 구조가 지금 모양인 이유

- **Severity 는 규칙 단위** — 이전에는 invariants.md 머리말이 전 파일을 Critical 로 선언해 §Escalation Triggers 에서 Warning 인 ARCH-3·ARCH-5 와 충돌했다 (#213).
- **invariant ↔ E 번호가 1:1 이 아니다** — "1:1 대응" 이라 적었던 문장이 스스로 반증됐다. 이름으로 지목된 invariant 만 전용 E 번호를 갖고 나머지는 E-1 로 수렴하며, E-3·E-6~E-9·E-11 은 ARCH 표가 아닌 다른 축 (msgs ABI·test·thread·E-STOP) 을 가리킨다.
- **Escalation 표의 SSoT 가 invariants.md 한 곳** — 이전에는 AGENTS.md §6 과 양쪽이 전체 목록을 인라인 복제했고, 그 구조가 handoff 섹션 목록에서 실제 드리프트를 냈다 (AP-DOC-1).
- **ARCH 탐지 패턴을 문서에 두지 않는다** — 문서가 divergent copy 를 들고 있었고, 그 사본이 hook 보다 낡은 스코프 (`ur5e_*/`, whole-file) 를 담은 채 조용히 썩었다 (#213).
- **RT 탐지는 hook 이 구현하지 않는다** — RT 금지는 정기 tick 경로에만 구속되는데 hook 은 파일 단위로만 보므로, blocking gate 로 만들면 `on_configure` 의 정당한 `push_back` 을 막는다. 그래서 RT 패턴은 invariants.md 의 detect 블록에 남기고 CI 가 그 유효성을 검증한다.
- **detect 패턴이 표 셀이 아니라 fenced 블록에 있다** — 마크다운 표는 이스케이프 없이 `|` 를 담을 수 없고, ERE 에서 `\|` 는 alternation 이 아니라 *리터럴 파이프* 다. 문법은 성해서 exit 1 + 무출력, 즉 "위반 없음" 과 구분되지 않는다. 2026-07 감사에서 표에 있던 패턴 10개가 전부 그 상태였다 (#213). `# probe:` / `# antiprobe:` 는 [validate_docs.py](../../repo_scripts/scripts/validate_docs.py) 가 CI 에서 정적 lint + 실행으로 검증한다.
- **빈 alternation 금지** (`.wait(|_for|_until)\(`) — GNU grep 은 관대하지만 에이전트 샌드박스의 `grep` 은 ugrep 으로 resolve 되고 거기서는 `empty (sub)expression` hard error 다.
- **RT-7 은퇴** — assertion 무결성은 timing-safety 가 아니라 process 규칙이라 PROC-6 으로 옮겼다.

## RT Path

### RT path 의 대표 진입점 (기록 시점)

판정의 SSoT 는 [architecture.md](../../agent_docs/architecture.md) §Execution Contexts 표다. 아래는 그 표를 읽을 때 참고할 실례다.

- **RT**: `RtControllerNode::ControlLoop()`, `RTControllerInterface::Compute()` / `DrainPendingTargets()` 의 tick 경로, DeviceBackend 의 state/motor/sensor 구독 콜백, UDP receive 콜백, `CheckTimeouts` 50 Hz 분기, MPC thread (`MPCThread::OnTick` → 파생 `Solve()` — dedicated SCHED_FIFO core), `UdpHandController::RunCommCycle` (self-clocked UDP send/recv cycle — `rtc::PeriodicRtThread` 기반 CommLoop, 별도 SCHED_FIFO thread).
- **비-RT**: lifecycle 콜백, `DrainLog()` aux thread, controller LifecycleNode 의 1 Hz aux 타이머 (timing CSV drain 등), ROS 파라미터 콜백, controller-owned RobotTarget 구독과 grasp_command 서비스 핸들러, target marshal 쪽 `SetDeviceTarget()` (controller LifecycleNode default group), `PublishNonRtSnapshot()` (`NrtPublishLoopEntry` 의 전용 `std::jthread`, SCHED_OTHER, SPSC drain).
- **"구독 콜백" 이라는 이름으로는 갈리지 않는 실례**: backend 의 sensor/state 구독 콜백은 `cb_group_rt_callback_` → SCHED_FIFO 라 RT 지만, controller 의 RobotTarget 구독 콜백은 controller LifecycleNode 의 default group → `nrt_callback_executor` → SCHED_OTHER 라 비-RT 다.

### RT-1 ~ RT-10 의 이유

| # | 이유 |
|---|------|
| RT-1 | Heap alloc 은 100 µs+ jitter + priority inversion |
| RT-2 | `noexcept` 위반 = unwinding latency 비결정, process kill 리스크 |
| RT-3 | Blocking I/O (rosout queue / network). 정기 tick 주파수 × 단 한 줄 블록 = 대형 지터 원인 (default 500 Hz × 1줄 = 500 발생/초; 2 kHz 면 4배). SPSC → `DrainLog()` 패턴의 실례는 [rt_controller_node_estop.cpp](../../rtc_controller_manager/src/rt_controller_node_estop.cpp) |
| RT-4 | 우선순위 역전, blocking |
| RT-5 | Expression template lazy-eval → aliasing 버그 (같은 메모리 r/w) |
| RT-6 | Non-unit 결과 → 회전축 변형, drift |
| RT-8 | Atomic ref-count contention |
| RT-9 | 내부 state machine 동기화 (mutex 또는 atomic load + 분기). ros2_control jazzy 공식 wording: "Avoid using the `get_lifecycle_state()` method in the real-time control loop of the controllers and the hardware components as it is not real-time safe." 기록 시점의 호출부는 `RtControllerMain` 의 활성화 폴링과 `UdpHandNode::on_shutdown` 등 lifecycle 경로뿐이라 면제 대상이다 — RT-9 는 선제적 차단이다 |
| RT-10 | `notify_one/all` 자체가 내부 mutex 를 잡고 thread 를 깨운다 — 우선순위 역전 + 비결정 latency. `wait` 은 명시 mutex lock 보유 (RT-4 결합) |

### RT-10 대안의 in-tree 실례

- **eventfd + non-blocking write/poll** — CM 의 `nrt_publish_eventfd_` (`RtControllerNode::StartNrtPublishLoop` / `NrtPublishLoopEntry`), `UdpHandController::event_fd_`.
- **`std::atomic<bool>` release/acquire flag + self-clocked consumer** — `UdpHandController` 의 `event_pending_` 를 `rtc::PeriodicRtThread` 기반 CommLoop 이 매 tick latch 한다 (별도 wake 없음; 명령 없으면 read-only). busy-spin 이 아니다 — loop 가 이미 고정주기 tick 이다.
- **RT 핫패스 self-termination** — `RequestStop()` 은 pause-mutex + `pause_cv_.notify_all()` 을 보유하므로 E-Stop 같은 핫패스에서 못 쓴다. `PeriodicRtThread::RequestLoopExit()` (순수 `atomic<bool>` store) 로 예약하고, 종료 시 부수 작업은 loop unwind 후 base 가 1회 호출하는 `OnLoopAborted()` 에서 한다. 실례: `UdpHandController::OnCommLoopAborted()` 의 zero-write.

### RT pub/sub primitive 의 메커니즘

| Primitive | 출처 | 메커니즘 |
|-----------|------|---------|
| `SeqLock<T>` | `rtc_base` | writer wait-free + reader retry, heap 0 |
| `SpscQueue<T,N>` / `SpscPublishBuffer<512>` | `rtc_base` | Boost-style wait-free SPSC |
| `std::atomic<T>` | C++ stdlib | lock-free (POD only; 보통 ≤ 8 bytes) |
| `realtime_tools::LockFreeQueue<T, spsc_queue>` | `realtime_tools` (Boost.Lockfree wrapper) | wait-free SPSC — `SpscQueue` 와 등가 |
| `realtime_tools::RealtimePublisher::try_publish` | `realtime_tools` | `try_lock` + msg copy + cv notify → dedicated non-RT thread |
| `realtime_tools::RealtimeBuffer<T>` / `RealtimeThreadSafeBox<T>` | `realtime_tools` | `try_lock` + double buffer (또는 swap pointer). ctor 에서 `new T()` 2회 |
| `std::atomic<std::shared_ptr<T>>` | C++20 stdlib | libstdc++/libc++ internal spinlock — wait-free 아님 |
| `std::condition_variable` | C++ stdlib | mutex + futex wake — `notify_*` path 가 내부 mutex 보유 |

결정 가이드의 근거: `RealtimeBuffer` 는 ctor heap alloc 때문에 `SeqLock<T>` 대비 우열이 명백하다. `RealtimePublisher::try_publish` 는 dedicated thread 1개 추가 비용 vs 자체 SPSC drain 인프라 작성 비용의 트레이드오프다. 기존 SPSC + publish_thread 멀티플렉싱을 교체하면 drain 분리 / offload / session log 통합 이점을 잃는다.

### 알려진 RT-1 위반의 실측

**1 — actuator lane 의 DDS publish (#222).** `DeviceBackend::WriteCommand` 는 tick 마다 **2회 / 156 B** 를 할당한다. 할당은 백엔드 코드가 아니라 `libddsc`/`rmw_cyclonedds_cpp` 안에서 일어나고 (88 B serdata + 68 B CDR 버퍼), QoS 와 무관하다 (best_effort 가 바이트까지 동일). 이 스레드에서 publish 하는 한 백엔드가 피할 수 있는 것이 아니고, 비-RT 레인으로 옮기면 156 B 대신 밀리초급 비-RT 스케줄링을 얻는다 (layout v4.1 이 그래서 SPSC 인계를 없앴다). 측정 방법·대조·한계는 #222 코멘트가 SSoT 이고, 이 값을 다시 재려면 거기 부록의 `LD_PRELOAD` interposer 를 쓴다 — 기존 두 게이트는 C 라이브러리 `malloc` 을 원리적으로 못 본다 ([testing-debug.md](../../agent_docs/testing-debug.md) §게이트 표).

**2 — ONNX Runtime `Run()` (2026-09-12 사용자 결정, 조건부 수용).** ORT 의 `Session::Run` 은 IoBinding 여부와 무관하게 매 호출 heap 을 할당한다 — CPU EP 에 할당 없는 Run 은 없고, 노드 2개짜리 모델도 매 호출 `operator new` 를 부른다 (수치는 `test_demo_inference_real_model` 이 기록한다). heap 정책의 구현은 [rt_heap.hpp](../../rtc_base/include/rtc_base/threading/rt_heap.hpp) (`M_TRIM_THRESHOLD -1`, `M_MMAP_MAX 0`) 이며 대가는 RSS 가 최대치에 머무는 것이다. 센서: 조건 1 은 `rtc_base` 의 `test_rt_heap`, 조건 2·3 은 `test_demo_inference_real_model` (로컬 전용 — 정책 파일이 없으면 skip = 미검증), 호출자 몫은 `test_demo_inference_alloc` (FakeEngine) 이 0 을 지킨다. `udp_hand_driver` 경로는 조건 1 만 공유하고 2·3 은 미측정이다 (F/T 모델이 repo 밖).

**3 — MPC thread 의 cross-mode swap (2026-09-19 기록).** `HandlerMPCThread::Solve` 는 phase 가 바뀌며 `ocp_type` 이 달라지면 그 스레드에서 `MPCFactory::Create` 로 handler 를 새로 만든다 — heap 할당·YAML 파싱·`try/catch`. DemoWbc 가 light·rich factory YAML 을 둘 다 넘기므로 production 에서 도달한다. 해소 경로는 두 handler 를 configure 에서 미리 만들어 swap 을 포인터 교체로 줄이는 것이고 별도 작업이다. 같은 스레드의 Aligator solve 자체가 할당하는지는 미측정이다. 통계 mutex 와 `fprintf` 는 이 기록과 함께 제거됐다 (E-9 결정: 문서가 아니라 코드를 RT 에 맞춘다).

### Clock 예외의 승인 이력

원격 예측 궤적의 물리 샘플 시각 예외는 dynamic_catching D-2 로 2026-09-19 에 E-1 승인됐고 구현은 S1.3·S5.2 다. 이 예외는 t_c·t_cmd 같은 deadline 판정을 stamp 에서 파생시키므로 wall clock 점프가 그 판정 오차로 그대로 들어간다 — 조건 ③ 의 실기 재확인은 S10. 계약 전문: [IMPLEMENTATION_PLAN.md](../dynamic_catching/IMPLEMENTATION_PLAN.md) §3.1. 일반 규칙 쪽은 기록 시점에 `last_state_ns_` 등 watchdog 을 steady_clock 으로 유지해 준수하고 있다.

### RT telemetry 임계값 가이드

ros2_control jazzy controller_manager default 인용 (`control_rate=500Hz` / `dt=2ms` 기준):

| 지표 | warn | error | 비고 |
|------|------|-------|------|
| Execution time mean error | 1000 µs | 2000 µs | dt 의 50% / 100% slack |
| Execution time standard deviation | 100 µs | 200 µs | jitter 분산 |
| Periodicity standard deviation | 5.0 Hz | 10.0 Hz | tick rate jitter |
| Missed deadline 카운트 (10s window) | 1 | 10 | hard deadline 위반 |

`control_rate` ≠ 500Hz 일 경우 execution time 항목은 dt 비례 재계산 (예: 2 kHz, dt=500µs → warn=250µs, error=500µs).

RT loop 안에서 허용되는 통계의 형태: loop period error (`now - last_tick`, fixed-size ring buffer write), compute / device read·write time 누적, activation 이후 max latency (`std::max` 갱신), missed deadline 카운트 (`std::atomic<uint64_t>` increment).

## RT Host / Runtime Preconditions

| # | 이유 | 구현·검증 (기록 시점) |
|---|------|----------|
| RT-HOST-1 | Major/minor page fault → ms 단위 jitter | `controller_manager` 의 `lock_memory: true` 파라미터 또는 자체 `mlockall` 래퍼. `repo_scripts/scripts/verify_rt_runtime.sh` 가 `VmLck>0` 검증 |
| RT-HOST-2 | CFS 비결정성 제거. 99 는 kernel watchdog 영역 | `rtc_base/threading/thread_config.hpp` 의 `SystemThreadConfigs`. 기록 시점 layout 은 60~90 을 쓴다 (mpc_main 60 이 하한 — 외부 thread 를 아래에 끼우려면 이 하한부터 확인; #380 이 FIFO 55 의 mpc_workers 를 제거했다) |
| RT-HOST-3 | DDS receive thread / ROS executor / IRQ 와 동일 core 공유 시 cache pollution + 선점 jitter | `thread_config.hpp` 의 `SystemThreadConfigs::cpu_affinity` 또는 `cpu_shield.sh` runtime 격리 |

**System-level 확인** (배포 환경 책임, controller 시작 시 sensor 로 확인하고 timing log 헤더에 기록):

- PREEMPT_RT 커널 또는 lowlatency 커널 — `uname -a`
- `/etc/security/limits.conf` 의 `@realtime` 그룹 권한 (`rtprio`, `memlock unlimited`) — `ulimit -r`, `ulimit -l`
- Isolated RT core 에 non-RT 작업 미공유 — `taskset`, `/proc/interrupts`

세부 검증 명령은 [check_rt_setup.sh](../../repo_scripts/scripts/check_rt_setup.sh) 와 [verify_rt_runtime.sh](../../repo_scripts/scripts/verify_rt_runtime.sh) 가 갖는다. 설정 절차는 [RT_OPTIMIZATION.md](../RT_OPTIMIZATION.md).

**권장 sensor** (배포 전 1회, 환경 변경 시 재실행; 결과는 timing CSV 와 같은 디렉토리에 저장):

- `cyclictest --mlockall --smp --priority=80 --interval=200` — baseline kernel jitter
- `rtla osnoise top -P F:1 -c <iso_cores>` — Ubuntu 24.04 기본 제공
- `rtla hwnoise hist` — IRQ 비활성 시 hardware-induced noise

**Docker 배포 노트**: 컨테이너 배포 시 `--cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1` 누락 시 RT-HOST-1, RT-HOST-2 가 silent fail 한다.

## Architecture

| # | 이유 |
|---|------|
| ARCH-1 | robot-agnostic 훼손 ([design-principles.md](../../agent_docs/design-principles.md) §Five Principles (2. Generality)) |
| ARCH-2 | Cyclic dep / abstraction leak. 전형: `rtc_base/` 가 `rtc_controllers/` include, `rtc_*/` 가 integration 패키지 include |
| ARCH-3 | 확장성 훼손 → 세 번째 impl 에서 `#ifdef` 지옥. 신호: 새 `.cpp` 에 대응하는 pure-virtual base 부재 |
| ARCH-4 | 경계 훼손, robot-specific leak |
| ARCH-5 | 빌드 토폴로지 부담 + "share 만 있으면 OK" 모델 훼손 |
| ARCH-6 | 항상 최신 샘플 소비 — stale 큐잉 방지, RT freshness. depth 는 pub/sub 매칭 호환성과 무관하므로 안전 |
| ARCH-7 | agnostic 패키지가 exec 를 가지면 exec ↔ 노드 ↔ pgrep ↔ logger 정렬이 깨지고 robot-specific 의존이 새어든다 |

**ARCH-7 sensor 가 줄이 아니라 타깃 이름을 보는 이유** — CMake 는 줄을 제자리에서 고쳐 쓰므로 재들여쓰기가 신규 exec 로 읽혔다. 기록 시점의 면제 대상은 robot-agnostic standalone 노드 (`mujoco_simulator_node`, `closure_state_publisher`) 와 example 타깃이며 목록의 SSoT 는 [design-principles.md](../../agent_docs/design-principles.md) §Boundary Rules.

### ARCH-6

- **변환 규칙**: `rclcpp::QoS(N)`→`QoS(1)`, `.keep_last(N)`→`.keep_last(1)`, `rclcpp::SensorDataQoS()`→`SensorDataQoS().keep_last(1)` (best_effort 보존), create_pub/sub 정수 리터럴→`1`, Python `QoSProfile(depth=N)`→`depth=1`.
- **ToF snapshot 예외의 근거** (`<ns>/tof/snapshot`): 최대 `control_rate` 로 발행되는 sensor stream 이며 subscriber 가 매 샘플을 누적한다 (`shape_estimation` voxel cloud + snapshot_history, `ur5e_bt_coordinator` collection buffer). depth 1 이면 executor 가 못 따라갈 때 중간 스냅샷이 유실되므로 deep best_effort 큐를 유지한다. 기록 시점 값 — publisher 5 (`integrated_bringup/src/support/owned_topics.cpp` `SetupToFSnapshotPublisher`), shape_estimation sub 5, bt_coordinator collection sub 100.
- **`ARCH-6-exempt` 마커의 목적**: [verify-changes.sh](../../.claude/hooks/verify-changes.sh) Phase 0b grep sensor 가 그 라인을 건너뛰게 해 매 편집마다 재-flag 되는 것을 막는다.

### ARCH-5

`robot_descriptions` 는 C++ target / 헤더 / 라이브러리 export 가 0건인 data-only 패키지다 ([robot_descriptions/CMakeLists.txt](../../robot_descriptions/CMakeLists.txt) 는 `install(DIRECTORY)` 세 벌뿐 — `robots/` · `object_sim/` · `objects/`; 파일 수는 늘어도 **빌드되는 것은 여전히 0** 이라는 것이 이 규칙의 전제다).

**근거**: 빌드 시점에 link 할 artifact 가 0개이므로 build-dep 효과는 0. 그러나 build-dep 을 걸면 colcon 이 강제 토폴로지 엣지를 만들어 "이 디렉토리를 워크스페이스 어디 두든 — 형제 디렉토리든 별도 overlay 든 — `install/robot_descriptions/share/` 만 있으면 동작" 모델이 깨진다 (사용자 정책).

**`<test_depend>` 가 허용인 이유**도 같은 근거에서 나온다 — 그것은 런타임 lookup 방식을 동작하게 만드는 장치이지 그 방식을 대체하는 것이 아니다. colcon 토폴로지 엣지는 생기지만 (`colcon list --packages-up-to <pkg>` 에 나타난다) 위 근거가 보호하려는 것은 깨지지 않는다: `robot_descriptions` 는 `install(DIRECTORY)` 뿐이라 빌드 비용이 사실상 0 이고, 별도 overlay 에 두면 colcon 이 dep 을 해석하지 못해 엣지 자체가 생기지 않는다. 기록 시점 사례는 `rtc_controller_manager` (테스트가 `urdf.package` 파라미터로 share dir 을 런타임 resolve — 이 dep 이 없으면 병렬 빌드에서 설치 순서가 보장되지 않아 flaky; resolve 구현은 `rtc_controller_manager/src/rt_controller_node_params.cpp`). 소스 트리 상대경로 (`${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/...`) 로 configure 시점에 경로를 박는 쪽이 colcon 엣지는 안 만들지만 "어디 두든 동작" 모델은 오히려 더 훼손한다.

## Process

| # | 이유 |
|---|------|
| PROC-1 | Drift 방지 — git log 에서 반복 수정 커밋 다수 확인됨 |
| PROC-2 | ABI 호환성 |
| PROC-3 | 광범위 영향 — 대부분 패키지가 의존 |
| PROC-4 | 중복 트리거 안전성 |
| PROC-5 | 부분 수정 drift — launch (Python) 가 노드 (C++) 보다 먼저 세션 디렉토리를 만들어 한쪽 누락이 런타임에 표면화 (`test_session_dir.py::test_subdir_list_matches_cpp_mirror`). 미러 쌍의 실례: `rtc_base/logging/session_dir.hpp` `kSubdirs` ↔ `rtc_tools.utils.session_dir` `_SESSION_SUBDIRS` |
| PROC-6 | 회귀 은폐 방지. RT timing 과 무관한 process 규칙 (구 RT-7). squash 단서를 명시하는 이유는 규칙이 머지 방식을 고려하지 않고 쓰여 있어 **문자 그대로는 지킬 수 없었기** 때문이다 — #422·#423 에서 연속 발현했고, 두 번 모두 분리한 commit 이 `main` 에서 사라졌다. 감사 가능성은 no-ff 를 머지 기본으로 만든 근거의 하나다 ([conventions.md](../../agent_docs/conventions.md#commit-message-conventions) §Merge method) |
| PROC-7 | CM 은 tick 마다 새 `stamp_ns` 를 만들고 publish thread 는 SeqLock 을 다시 Load 하므로, Store 생략은 "미발행" 이 아니라 **stale body + fresh stamp 재발행** 이다 — E-STOP 중에 살아있어 보이는 `valid=1` telemetry 가 그 발현 (issue #234 P-1). 계약·경로는 [controllers.md](../../agent_docs/controllers.md#ros2-topics) |
| PROC-8 | 두 원시는 대체재가 아니다. **동일 executor 를 두 곳에서 구동하는 것은 rclcpp 가 금지** — spin 하는 fixture 가 이미 전용 스레드를 갖고 있으면 헬퍼의 추가 spin 이 UB 다. 반대 방향도 있다: 기본 callback group 밖의 lane 을 검사하는 테스트에서 기본 executor 를 pump 하면 검사 대상까지 서비스돼 assertion 이 아무것도 증명하지 못한다. 이 혼동은 이미 한 번 머지됐다 되돌려졌고 (#356), 그 뒤로 유일한 방어선이 헤더 주석 한 덩어리였다 — 주석은 다음 사람이 정리하면 센서 없이 사라진다. 탐지는 hook Phase 0c (헬퍼 헤더 한정, 주석 제외 후 grep) |

## Numerical

### 이유와 구현 위치 (기록 시점)

| # | 이유 | 구현 위치 |
|---|------|-----------|
| NUM-1 | Unbounded magnification. σ₀ ≤ 0 은 셸을 좁히는 게 아니라 `AdaptiveDampingSquared` 를 short-circuit 시켜 λ²=0 을 상시 반환한다. 하한을 `LoadConfig` 에만 두면 `set_gains()` / ctor default 가 우회한다 | 두 상수 (`kMinMaxDamping`, `kMinSigma0`) 의 정의는 `compliance/task_dynamics.hpp` 한 곳 (지키는 법칙 옆). configure 쪽은 `rtc_controllers/src/params/` 파서들 — 다섯 파서 전부 두 helper 를 쓴다 (σ₀ 수렴 #311; `cascaded` 만 `num` 이 비유한 값을 먼저 거부해 helper 의 비유한 분기가 도달 불가한 방어층). λ 규약은 compliance §6.5 하나뿐 (#236) |
| NUM-2 | `1/dt` 발산 | 모든 trajectory generator |
| NUM-3 | Drift → non-unit | SE3 trajectory, orientation PD |
| NUM-4 | IEEE 754 `1/0 = INF` → hang (crash 아님) | `rtc_controllers/src/params/{clik,osc,joint_pd}_params.cpp` 의 `cfg["trajectory_speed"]` / `cfg["trajectory_angular_speed"]` 파싱부 (#298 S7c-2 이전에는 어댑터의 `LoadConfig`), 그리고 `integrated_bringup` 의 각 컨트롤러 `parameters.cpp` / `controller.cpp` |
| NUM-5 | 점 구속 loop 은 조립 분기가 여럿이고 모두 φ=0 을 만족 → ‖φ‖ 검사를 통과한 채 반대편 분기 착지, warm-start 로 영구 고정 | `loop_projection` (`ProjectPassiveWithContinuation`), `RtClosedChainHandle` (tick 당 seed clamp) |
| NUM-6 | `K_pⁿ < 0` 은 복원이 아니라 발산 방향 (τ₀ = K_pⁿ·(q_ref − q) − K_dⁿ·q̇), `K_dⁿ < 0` 은 에너지 주입. `Nᵀ`/`N` 사영이 이를 task 로부터 가려 fault 없이 조용히 자세가 밀린다. 게이트가 `!= 0.0` 이라 법칙에만 걸면 *게이트는 열린 채 값만 0* 인 조합이 생긴다 | 로더 쪽: `rtc_controllers/src/params/` 의 영공간 자세 게인을 갖는 파서 전부 (`joint_pd` 는 그 게인이 없어 해당 없음) + `DemoTaskController` (`null_kp` 런타임 노출 표면); `test_params_schema.cpp` 의 `AnUndefinedNode*` 가 `if (!cfg)` 경로를 pin. 사용 지점 준수 사례: `null_dq_ *= FloorNonNegativeGain(gains.null_kp)` (`integrated_bringup`) |
| NUM-6a | floor 앞이면 음수 `K_pⁿ` 가 `if (kp > 0.0)` 을 건너뛰어 감쇠 보정을 통째로 놓치고, `== 0.0` 가드를 그냥 통과한다. 반대로 이 둘을 tick 으로 옮기면 게인의 *하한* 이 아니라 *다른 게인의 재작성* 이 매 tick 돌아 `get_gains()` 와 `Compute()` 가 갈리고 리터럴 oracle 전부가 configure 규칙을 미러해야 한다 | `rtc_controllers/src/params/task_impedance_params.cpp` — YAML 경로와 `if (!cfg)` 경로 둘 다 동일 3단계. 런타임 등가물은 기록 시점 출하 코드에서 도달 불가한 forward-looking spec (실측: #322). 이 한 필드에만 setter 를 달지 않는다 (#301 결정) |
| NUM-6b | 밴드 안에서 `q − lo < 0` 이라 부호가 자세 게인과 반대로 작동한다: `k_lim < 0` 은 관절을 한계 **안쪽으로** 밀고, `d_lim < 0` 은 하드스톱 앞에서 에너지를 주입한다 (출하 config 에선 d_lim 이 유일한 방어선 — #280). δ 는 clamp 로 못 고친다 — `lo = q_min + δ` 라 δ<0 은 밴드를 한계 밖에 놓고 δ=NaN 은 두 비교를 모두 false 로 만들어 반발항이 영원히 안 발동하는데 fault 도 없다; 0 clamp 는 그 오설정을 정상 구동 config 로 만든다 | `rtc_controllers/src/params/` 의 세 파서 (task_impedance·cascaded_compliance·task_admittance — admittance 는 δ 만; δ 가 q_cmd clamp 밴드를 좁히는 compliance §7.3 형태라도 판정은 같다). 밴드 대조 거부는 #478 (#473 과 같은 판정). `test_params_schema.cpp` 의 `SafetyLayerGainSchema.*` 가 키×경로를 pin (mutation 실측: #280) |

### NUM-7 의 실측

- **`ComputeVirtualTcp` (#316)**: `total_weight <= 0.0` 은 NaN 가중치에 false 라 비유한 `force_magnitude` 하나가 가중합을 오염시킨 채 지나가고, centroid 의 `count == 0` 은 비유한 `position_in_tcp` 를 애초에 보지 않는다 — 둘 다 `valid=true` 인 NaN pose 를 내보내 호출부가 그것을 제어점으로 latch 했다. 고친 형태는 `integrated_bringup/support/virtual_tcp.hpp` 의 `ComputeVirtualTcp` 단일 출구 — 3 모드 + 비유한 `T_base_tcp` 를 분기 하나로 덮고, 실패 시 identity 로 되돌려 기존 "invalid ⇒ identity" 모양을 유지한다. 하류 finite 검사 (`compliance::AllFinite` · `urtc::ValidateControllerOutput`) 와 상보다 — 이 층은 발행 *전에* 무효로 만들어 last-good 을 살린다. `-ffast-math` / `-Ofast` 는 grep 0건이었다 (2026-08-06).
- **`rtc_tsid` `ClikReferenceGenerator::Init` 의 위치 박스 게이트 (#316 Sprint 3)**: 세탁하는 것은 `std::max`/`std::min` 박스 조립이라 비유한 `q_min`/`q_max` 가 ±`v_limit` 로 바뀌어 그 joint 의 위치 바운드만 조용히 사라진다 — 출력은 유한하므로 같은 함수의 `allFinite()` 출구 가드도, `lo > hi` 붕괴 가드도 (NaN 비교가 전부 false) 안 뜬다. 게이트를 되돌리면 `-1e-3` 로 묶여야 할 `v_ref` 가 `-4.7e-2` 로 나가고 `Compute()` 는 `true` 를 반환한다.
- **과대 엄격성의 대가**: 소비자가 `Init` throw 를 catch 하면 `clik_enabled_=false` 가 되고, CLIK 이 유일한 위치 backbone 이므로 (integrator A/B shadow 는 제거됨) DEC-1 ⓐ 가 `on_configure` 를 `FAILURE` 로 떨어뜨린다 — 로봇이 아예 안 뜬다. 정당한 등호 (`q_min == q_max`, 마진으로 잠긴 joint) 를 거부하는 것은 조용한 성능 저하가 아니라 가용성 사고이고, 이 방향의 오탐 비용이 미탐 비용보다 크다.
- **같은 `Init` 의 인접 스칼라 3개 (#316 D-9)**: `v_limit`·`anchor_drift_max` 는 "off" 가 `<= 0` 인 스위치를 `> 0.0` 술어로 읽으므로 비유한 값이 기능을 인가된 off 와 구별 불가하게 꺼버린다. `w_arm`/`w_hand` 는 가드가 이미 있었는데 `x < 0.0` 이라 NaN 이 통과했다. sign 형태를 `!(x >= 0.0)` 로 고치면 NaN·-inf 는 막히지만 `+inf` 가 남는다 (그 형태만 남긴 mutation 이 신규 테스트를 red 로 만든다).

### NUM floor 규약의 근거

- `max(하한, NaN) == 하한` 이라 손수 `std::max` 는 비유한 게인·λ_max 를 *그럴듯한* 값으로 세탁해 fail-loud 를 fail-silent 로 바꾼다. σ₀ 는 어떤 도달 가능한 자세도 안 들어오는 셸이 되어 compliance §6.5 가 전 자세에서 꺼지고, NUM-6 은 기존 `nan_inf` SAFE_STOP 이 지워진다. 하류 finite 검사는 토크 레인 `compliance::AllFinite` (compliance §10.5), 위치 레인 actuator 경계의 `urtc::ValidateControllerOutput` — 후자는 tick 을 hold 로 대체하고 연속 거부 ~100 ms 후 E-STOP 으로 올린다.
- 사용 지점 하한이 바인딩 몫이 된 것은 어댑터 삭제 (#298 S7c-2) 뒤다. in-tree 준수 사례는 `integrated_bringup` task 컨트롤러 (#282); doc 표 한 줄 대신 grep 되는 심볼 (`compliance::FloorMaxDamping` 등) 로 만든 결정은 #301.
- `AdaptiveDampingSquared` 안에 floor 를 넣으면 자기 성질 λ² ≤ λ_max² 가 하한 아래에서 거짓이 되는데, 기존 oracle 은 그 영역을 안 써 어떤 테스트도 못 본다 (감지기: `TaskDynamics.AdaptiveDampingDoesNotFloorItsOwnArgument`; oracle 전수 실측: #301).

### NUM-5 의 실측 (issue #248, PR #249 리뷰)

- `CONTACT_3D` (점) 구속으로 닫은 loop 은 4-bar 처럼 조립 분기가 둘 이상이고 각 분기가 φ=0 을 정확히 만족한다 — 분기를 결정하는 것은 residual 이 아니라 seed 에서 해까지의 homotopy 경로다.
- proto_1b (5-loop hand) 실측 이탈 임계는 seed 증분 **0.087~0.12 rad** 이며, 그 위에서는 passive 가 180° 뒤집히거나 여러 바퀴 감긴 해로 착지한다.
- RT 에서 sub-step loop 을 못 쓰는 이유는 입력 의존 → 비결정적이기 때문이다. tick 당 균일 스케일 클램프는 tick loop 자체를 continuation 경로로 만들어 고정 K 를 유지한다. per-joint 클램프는 homotopy 경로를 꺾는다.
- **`acceptable` 술어 (#250)**: URDF 좌표 불일치의 residual floor (≤1 µm, 형상 무관 상수) 는 strict (`converged`) 를 원리적으로 통과할 수 없지만 실질 loop-consistent 라 warm-start 로 안전하고, 분기 간 거리는 rad 단위라 µm floor 로는 homotopy 가 훼손되지 않는다. strict 로 판정하면 floor 로봇에서 continuation 이 항상 첫 sub-step 에서 끊긴다. acceptance 임계의 기본값은 병진 1e-6 m.
- **비수용 sub-step 을 이어가면** 마지막 sub-step 만 통과해도 성공으로 보고돼 소비자가 커밋한다 — 단일 사영에는 없던 실패 은폐 경로다.
- **완화를 미발동 tick 에도 태우면** `prev + (q_a − prev)` 재구성 경로의 부동소수 왕복에서 1 ulp 가 새어 serial 등가 (구속 없는 모델 = 개방 체인 FK) 가 조용히 "근사" 로 격하된다.
- **`held`**: walk-in (`⌈Δ/증분⌉` tick 뒤 자연 해제) 뿐 아니라 해를 커밋하지 않는 가드 (분기 이탈·비유한) 에도 서며, 후자는 입력이 그대로면 무기한 지속된다. 그동안 소비자는 조용히 last-good / open-chain fallback 으로 돈다.
- λ 상향은 band-aid 다 — 실측상 λ=1e-2 는 막지만 λ=0.1 은 과감쇠로 오답을 내 마진이 좁다.

## False-positive 처리

차단형 sensor 에 "false-positive 이니 코드는 그대로" 가 통하지 않는다는 것은 2026-09-22 ARCH-1 에서 실측했다 — 차단하는 gate 는 통과 watermark 를 전진시키지 않으므로 같은 주석 한 줄에 두 turn 연속 차단됐다.
