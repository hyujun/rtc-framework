# Anti-Patterns

재발을 방지해야 할 실수 패턴. 각 항목은 동일 구조:
- **증상**: 무엇이 일어나는가
- **원인**: 왜 일어나는가
- **탐지**: grep/test 명령 (가능한 경우)
- **복구**: 올바른 방법

근거는 git log 의 historical fix-commit 이다. 절대 카운트·commit SHA·시점 의존 status (`현 baseline 0건`, `Phase X ✅`) 는 박제하지 않는다 (AP-DOC-1) — 패턴 자체가 핵심이고 측정값은 측정 시점에 의존한다.

## RT Safety

### AP-RT-1: 정기 tick에서 `RCLCPP_*` 직접 호출 ([invariants.md](invariants.md) RT-3 위반)

- **증상**: 정기 tick (`control_rate`) 루프에서 지터 스파이크, rosout queue 포화 시 RT overrun → E-STOP. Rate 가 높을수록 폭주가 빠름 (rate-proportional)
- **원인**: `RCLCPP_*` 매크로가 내부적으로 string format + rosout IPC publish → heap + blocking
- **탐지**: [invariants.md](invariants.md) §위반 탐지 패턴 의 `detect id=RT-3` 블록 — 단 one-shot init / THROTTLE 변종은 제외
- **복구**:
  - one-shot init: 현상 유지 (lifecycle 콜백 / 한 번만 실행되는 fatal 경로)
  - 정기 tick: SPSC → `DrainLog()` aux thread 패턴 (참조: `rtc_controller_manager/src/rt_controller_node_estop.cpp`)
  - throttle 필요: `RCLCPP_*_THROTTLE` + RT-safe msg (단순 format + 기본 타입만)

### AP-RT-2: Quaternion `lerp` ([invariants.md](invariants.md) RT-6 위반)

- **증상**: Non-unit quaternion → 회전축 축소/왜곡, 누적 drift
- **원인**: `Eigen::Quaterniond::slerp`가 아닌 선형 보간 오용
- **탐지**: [invariants.md](invariants.md) §위반 탐지 패턴 의 `detect id=RT-6` 블록
- **복구**: `q_a.slerp(t, q_b)` 명시

### AP-RT-3: Eigen expression에 `auto` ([invariants.md](invariants.md) RT-5 위반)

- **증상**: Expression template lazy-eval + aliasing → 같은 메모리 읽고 쓰기 → 결과 쓰레기
- **원인**: `auto M = A * B;` 는 `Eigen::Product` 를 담고 나중 평가 시 aliasing 가능
- **탐지**: [invariants.md](invariants.md) §위반 탐지 패턴 의 `detect id=RT-5` 블록
- **복구**: 명시 타입 `Eigen::MatrixXd M = A * B;` 또는 `A.noalias()` 명시

### AP-RT-4: RT 경로 `std::lock_guard` ([invariants.md](invariants.md) RT-4 위반)

- **증상**: Aux thread가 holding 중이면 RT blocks → 100 µs+ stall
- **원인**: Priority inheritance 없는 일반 mutex는 RT 태스크가 non-RT를 기다림
- **복구**: `SeqLock<T>` (single-writer/multi-reader, trivially copyable만), `SpscQueue<T,N>` / `SpscPublishBuffer<N>`, atomic, `std::try_to_lock`

### AP-RT-5: 정기 tick에서 unguarded log (AP-RT-1 과 같은 RT-3 축 — AP-RT-1 은 `RCLCPP_*` 호출 자체, 여기는 throttle 없이 반복되는 형태)

- **증상**: 1 tick당 여러 줄 로그 × 정기 tick 주파수 = rate-proportional 폭증
- **복구**: `RCLCPP_*_THROTTLE(logger, clock, period_ms, "fmt", args...)` — msg는 [invariants.md](invariants.md) RT-3 세부 스펙 준수

### AP-RT-6: torn-read snapshot

- **증상**: multi-field 구조체를 atomic 없이 복사 → 중간에 writer가 업데이트 → torn
- **원인**: `memcpy(&out, &shared, sizeof)` 도중 writer 개입
- **복구**: `SeqLock<T>` Load (reader-side retry loop), 또는 `std::atomic<T>` (POD만)

### AP-RT-7: RT path 에서 `std::condition_variable` notify/wait ([invariants.md](invariants.md) RT-10 위반)

- **증상**: RT producer 가 `cv.notify_one` 시 내부 mutex 보유 → 우선순위 역전 + 비결정 wake latency. wait side 는 명시 mutex lock 보유 (RT-4 결합)
- **원인**: producer-consumer 통지를 cv 로 구현. 직관적이나 RT 우선순위 보장 안 됨
- **본 repo 사례**: `udp_hand_driver/include/udp_hand_driver/udp_hand_controller.hpp` (`6405c76` 이전) 의 `event_mutex_ + event_cv_ + event_pending_ + staged_cmd_` 패턴. `SendCommandAndRequestStates` 의 RT producer 가 `lock_guard + notify_one`, `EventLoop` 가 `unique_lock + cv.wait_for`
- **탐지**: [invariants.md](invariants.md) §위반 탐지 패턴 의 `detect id=RT-10` 블록
- **복구**:
  - **eventfd + non-blocking write** (`::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)`, producer `::eventfd_write(fd, 1)`, consumer `::poll(&pfd, 1, timeout_ms)` + `::eventfd_read(fd, &drained)`) — CM 의 nrt-publish lane (`RtControllerNode` 의 `nrt_publish_eventfd_` 생성부) + `UdpHandController` (post-`6405c76`) 가 표준 패턴
  - SPSC + consumer polling (kEventTimeout 짧은 sleep) — wake latency = polling 주기
  - atomic_flag + busy-spin (very-low-latency consumer 만; CPU 낭비)

> **AP-RTT-3 · AP-RTT-4 는 결번**이다 (은퇴 사유는 기록되지 않았다). AP-RTT-5 는 AP-THREAD-1 로 재분류됐다. ID 는 재사용하지 않는다.

### AP-RTT-1: `realtime_tools` primitive 도입 시 예방 규칙 (dedicated thread / ctor heap / drop 추적)

> **도입 시 적용** — `realtime_tools` 를 쓰지 않는 동안은 아래 grep 이 0건이고, 채택하는 순간부터 이 블록이 구속한다. primitive 선택·금지 기준의 SSoT 는 [invariants.md](invariants.md) §RT pub/sub primitive catalog (`RealtimePublisher::try_publish` / `RealtimeBuffer` 행) 이며, 이 블록은 도입 시 검토할 세 함정만 요약한다.

- **`RealtimePublisher` dedicated thread → layout 파괴**: instance 당 `publishingLoop` 전용 thread 1개 생성 → `thread_config.hpp` `cpu_affinity` layout 초과, RT core 공유 시 cache pollution/선점. 탐지 `ps -eLf | grep <process> | wc -l` 이 `SystemThreadConfigs` 초과. 복구: (a) 토픽 N개를 SPSC + 단일 publish_thread 멀티플렉싱 (본 repo 기존 패턴), (b) 채택 시 `get_thread()` 로 priority/affinity 명시 ([CLAUDE.md](../CLAUDE.md) §6 E-7)
- **`RealtimeBuffer` ctor/reset 을 lifecycle 밖에서 호출**: ctor·`reset()` 가 double buffer 를 `new T()` 2회 → RT-1 위반. ctor/`reset` 은 `on_configure`/`on_cleanup` 에서만. RT path 재구성은 `SeqLock<T>` + writer `Store`
- **`try_publish` drop 미추적**: `try_publish` 가 false (lock 실패) 반환 시 silent drop. 호출 site 마다 `std::atomic<uint64_t> drop_count_` 증가 + aux thread 주기 publish/log (본 repo SPSC drain 은 logger 가 이미 drop counter 추적)

### AP-RTT-2: 외부 라이브러리 thread 의 priority / affinity 가 thread_config.hpp 와 불일치

- **증상**: 외부 thread 가 default policy (`SCHED_OTHER`) 로 생성되어 RT thread 와 동일 core 에 묶이면 RT thread 가 선점 받음
- **원인**: 외부 라이브러리는 thread 생성 시 priority/affinity 설정 없음 — 호출자 책임. 실제 소비자는 DDS 스레드 (#163/#164); `realtime_tools::RealtimePublisher` 는 도입 시 동일 클래스의 예시
- **탐지**: `verify_rt_runtime.sh` 의 thread 별 sched policy / affinity 검사에서 `SCHED_OTHER` thread 가 RT core 에 매핑
- **복구**: 외부 thread 의 native handle 로 `pthread_setschedparam` + `pthread_setaffinity_np` 명시 호출. RT-HOST-2/3 정합 보장

### AP-THREAD-1: `ThreadConfig::cpu_core` 를 kernel logical CPU id 로 가정

- **증상**: SMT-on hybrid (NUC13 / NUC14 / i9-13900K) 또는 AMD SMT 시스템에서 RT thread 가 의도와 달리 P-core 의 SMT sibling 에 핀됨. `rt_callback` (slot 2 = cpu 2 = P-core 1 physical) 와 `mpc_main` (slot 3 = cpu 3 = P-core 1 sibling) 이 동일 hardware execution unit 의 두 hyperthread 에 들어가 cache/port contention 발생 — RT 우선순위 우위가 무력화됨
- **원인**: `ThreadConfig::cpu_core` 는 *slot index* (physical core 번호), `CPU_SET(n, ...)` 의 `n` 은 *logical CPU id*. SMT-off 환경에서만 두 값이 일치하므로 4-core CI mock 만으로는 회귀를 잡을 수 없음
- **탐지**: `ApplyThreadConfig` / `ApplyThreadConfigWithFallback` / `CheckThreadHealth*` 가 `SlotToLogicalCpu()` 없이 `cfg.cpu_core` 를 `CPU_SET` / `CPU_ISSET` 에 직접 전달하는지:

```detect id=AP-THREAD-slot-mapping
grep -rnE 'CPU_(SET|ISSET)\((cfg\.)?cpu_core' rtc_base/include/rtc_base/threading/
# probe:     CPU_SET(cfg.cpu_core, &cpuset);
# antiprobe:     CPU_SET(SlotToLogicalCpu(cfg.cpu_core), &cpuset);
```
- **복구**: 새 affinity 호출 site 가 추가되면 반드시 `SlotToLogicalCpu(slot)` 또는 `SlotToLogicalCpu(slot, topology)` 를 거쳐 변환. unit test 는 `CpuTopology` mock 으로 직접 주입 (overload 사용)
- **비고**: 구 AP-RTT-5 (`realtime_tools` 미사용 예방 항목이 아니라 in-tree thread affinity 라이브 결함이라 재분류). detect id `AP-THREAD-slot-mapping` 는 CI (validate_docs.py D7) 가 검증하는 load-bearing 문자열이라 개명해도 유지. 같은 slot→logical 축의 GRUB 설정 통일은 #152

## Design / Architecture

### AP-ARCH-1: `rtc_*` 패키지에 robot-specific 상수 하드코딩 ([invariants.md](invariants.md) ARCH-1 위반)

- **증상**: 다른 로봇에서 재사용 불가 → 패키지 fork 압력
- **탐지**: [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Phase 0 의 ARCH-1 검사 (탐지 SSoT — `6.?dof`/`10.?dof` 가 SE(3) task-space 차원 오탐이라 제외된 이유도 거기 주석에 있다)
- **복구**: YAML 주입 또는 template parameter

### AP-ARCH-2: Interface 없이 두 번째 구현 추가 ([invariants.md](invariants.md) ARCH-3 위반)

- **증상**: 세 번째 구현 시 `#ifdef` / switch 지옥
- **복구**: 첫 두 구현 리팩터 → abstract base + `RTC_REGISTER_*` factory 패턴

### AP-ARCH-3: 역방향 include ([invariants.md](invariants.md) ARCH-4 위반)

- **증상**: `rtc_*` robot-agnostic 훼손
- **탐지**: [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Phase 0 의 ARCH-4 검사 (대상 integration 패키지 집합을 `package.xml` 에서 자동 도출하므로 패키지 rename 에 따라가지 못하는 하드코딩 glob 이 없다)
- **복구**: 공개 API만 사용, interface injection

### AP-ARCH-4: Device boundary 누설 (대응 invariant 없음 — device group 경계는 표로 규정되지 않았다)

- **증상**: Hand/ToF state가 robot arm state로 새거나 그 반대 → GUI 혼선, digital twin 틀림
- **원인**: 장치 그룹 경계 무시한 공용 버퍼 / state publisher
- **탐지**: device_group별 publisher 분리 여부, `SetDeviceTarget(device_idx)` 호출자에서 인덱스 정합
- **복구**: device_group당 별도 publisher, state/target 모두 `device_idx` tagging

### AP-ARCH-5: Topic QoS depth ≠ 1 ([invariants.md](invariants.md) ARCH-6 위반)

- **증상**: `create_publisher`/`create_subscription` 에 depth 10 (rclcpp 기본값) 또는 `SensorDataQoS()` (기본 depth 5) 를 무심코 사용 → stale 샘플 큐잉
- **탐지**: [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Phase 0b 의 ARCH-6 검사 (non-blocking). 인자 없는 `SensorDataQoS()` 도 대상
- **복구**: depth 를 1 로. reliability/durability 는 유지하고 depth 필드만 이동 (`SensorDataQoS().keep_last(1)`). 다중 샘플 누적이 정당하면 [invariants.md](invariants.md) §Escalation Triggers 의 E-1 로 예외 기록

## Process / Drift

### AP-PROC-1: "✅ complete" 주장 후 실제 미완료

- **증상**: 문서에는 complete, 코드는 일부만 마이그레이션
- **원인**: PR 리뷰에서 grep 기반 verification 누락
- **탐지**: `grep -c <new_pattern>` 예상치 vs 실측 — 대상 파일 목록 명시 후 전수 grep
- **복구**: complete 체크 전 전수 grep, 대상 파일 목록 명시 후 체크

### AP-PROC-2: Code-only without YAML/README/CMake 동기화 ([invariants.md](invariants.md) PROC-1 위반)

- **증상**: 빌드 실패, 파라미터 ParameterUninitializedException, 런타임 NotFound
- **복구**: [modification-guide.md](modification-guide.md) Completion Checklist 전항목

### AP-PROC-3: 숫자 하드코딩 후 drift

- **증상**: 문서 A = N, 문서 B = N+1, 실측 = N+k (mesh count / 테스트 수 / 패키지 수 등 historical 사례 다수)
- **복구**: 단일 출처 (코드/YAML/git) + 측정 명령 박제. 문서엔 수치 자체를 넣지 말 것

### AP-PROC-4: 기존 test assertion을 통과시키려 수정 ([invariants.md](invariants.md) PROC-6 위반)

- **증상**: 회귀 은폐
- **탐지**: `git diff test/` 에서 `EXPECT_*` / `ASSERT_*` 상수 변경
- **복구**: 새 코드를 고쳐라. assertion이 진짜 틀렸거나 spec 이 바뀐 경우는 정당한 변경 — 새 코드 fix 와 **별도 commit** 으로 근거 제시 + regression test 갱신, 착수 전 E-6 escalate. ("절대 못 고친다" 가 아니라 "몰래 약화 금지")

### AP-PROC-5: ROS 2 Jazzy 파라미터 타입·launch 호환

- **증상**: `ParameterUninitializedException`, `namespace='/'` Jazzy 거부, CycloneDDS domain 실패
- **원인**: Humble→Jazzy 전환 시 파라미터 선언 API 변경 미반영
- **복구**: `ParameterDescriptor` 명시, `declare_parameter<T>(name, default, descriptor)` 타입 파라미터 사용, LifecycleNode는 `namespace=''` (empty string)

### AP-PROC-6: BT coordinator 등록 누락

- **증상**: 런타임에 BT 노드 not found, 테스트 중복 등록, validation fail
- **원인**: `BT::BehaviorTreeFactory::registerNodeType<>` 호출 누락 또는 `validate_tree()`에 새 노드 미추가
- **탐지**: `grep -l registerNodeType ur5e_bt_coordinator/` 에 새 노드 포함 확인
- **복구**: validate_tree / registerNodeType 양쪽 업데이트 — BT 노드 신설 시 체크리스트

### AP-PROC-7: **추가** 변경에서 "회귀 0 fail" 을 커버리지로 읽기

- **증상**: N+1번째 컨트롤러 / 프로필 / backend / device group 을 추가했는데 전 테스트가 green.
  green 의 이유가 "잘 만들어서" 가 아니라 **아무도 그것을 실행하지 않아서** 다
- **원인**: per-X 스위트가 대상 목록을 **손으로 나열**한다 (`{"a","b","c"}` 루프, `_SHIPPED` 리스트,
  타입 목록, 하드코딩 코퍼스). 추가는 그 목록 밖이므로 실패가 아니라 **부재**가 되고, 잘못된
  rename·누락된 CMake 소스·형제 값을 가리킨 채 남은 상수가 전부 통과한다.
  실측: `demo_compliance_controller` 추가 커밋에서 integrated_bringup 941/941 green,
  신규 컨트롤러 실행 라인 수 **0** (#469 S2)
- **탐지**: 추가 대상 이름으로 `grep -rn` 해서 **테스트가 그 이름을 아는지** 본다. 모르면
  0 fail 은 정보가 아니다. 하드카운트 단언(`ASSERT_EQ(configs.size(), 12U)`)이 있으면 그것이
  목록을 고치라는 tripwire이지 breaker 가 아니다 — **추가만으로는 그것도 안 깨진다**
- **복구**: (a) 목록을 늘리거나, (b) 기존 형제와의 **등가성**으로 합성한다 — "기존 X 가 시나리오
  S 에서 옳다"(기존 스위트) ∧ "신규 Y 가 S 에서 같다"(신규 테스트) = 절대 명제. (b)는 Y 가 X 의
  사본인 동안만 성립하므로 갈라지는 시점을 테스트 안에 적어 둔다. 어느 쪽이든 **mutation 으로
  확인**: 신규 쪽에만 건 작은 섭동이 red 를 내는지 (관련: [invariants.md](invariants.md) PROC-6)

## Controller-Specific

> **AP-CTRL-2 · AP-CTRL-4 는 결번**이다 (은퇴 사유는 기록되지 않았다). ID 는 이력 참조를 위해 재사용하지 않는다.

### AP-CTRL-1: Mid-tick gains branch 불일치

- **증상**: `Compute()` 중간에 aux thread 의 게인 writer (parameter callback) 실행 → bool flag 절반만 업데이트된 상태로 분기
- **복구**: `Compute()` 진입 시 `const auto gains = gains_lock_.Load();` 단일 snapshot

### AP-CTRL-3: `trajectory_speed = 0` → 1/v = INF ([invariants.md](invariants.md) NUM-4 근거)

- **증상**: IEEE 754 `1/0 = INF` → trajectory hang (crash 아님)
- **복구**: `std::max(1e-6, val)` 클램프

### AP-CTRL-5: Joint reorder / device index off-by-one

- **증상**: 잘못된 joint에 명령 전송, wrist_3_joint zero command, hand→arm state leak
- **원인**:
  - URDF joint 순서 ≠ ros2_control joint 순서
  - `TopicConfig::groups`의 insertion 순서 가정
  - ClampCommands에서 position vs velocity limit 혼동
- **탐지**:
  - reorder map 초기화 경로에서 identity fallback 있는지 확인
  - `device_states_` 인덱싱이 device 순서인지 config 순서인지 명시
- **복구**:
  - reorder map은 config 로드 시 1회 계산, identity fallback 금지
  - Position limit은 lower/upper bound, velocity limit은 별도 적용

### AP-NUM-1: residual 로 폐쇄 체인 조립 분기를 판정 ([invariants.md](invariants.md) NUM-5 위반)

- **증상**: closed-chain FK 가 엉뚱한 형상으로 렌더링/제어된다. passive joint 가 ~180° 뒤집히거나 여러 바퀴 감긴 값이다. 그런데 `converged=true`, `‖φ‖≈1e-8` 로 **모든 건전성 검사가 통과**한다
- **원인**: 점(`CONTACT_3D`) 구속 loop 은 조립 분기가 여럿이고 **모두 φ=0 을 만족**한다. 무감쇠 Gauss-Newton 사영에 큰 seed 점프를 한 번에 넘기면 반대편 분기로 착지하고, warm-start 구조상 영구 고정된다 (#248)
- **발동 경로** (전부 "한 호출에서 seed 가 크게 움직임"): cold start (초기 seed=`q_ref`≈neutral → 첫 실측 포즈), 빠른 동작 (관절 속도 한계 × 발행 주기 > 이탈 임계), 메시지 유실 (`QoS(1)` 드롭 → 실효 스텝 배가)
- **탐지**: `‖φ‖` 는 무력하다. **미세 continuation 기준해와 형상을 직접 대조**하거나, 사영 전후 passive 이동량을 본다. `grep -rn "ProjectPassiveToConstraint" --include=*.cpp` 로 continuation 을 거치지 않는 직접 호출을 찾는다
- **복구**: 비-RT 는 `ProjectPassiveWithContinuation`, RT 는 tick 당 seed clamp (NUM-5 세부 스펙). λ 상향은 band-aid 다 — 실측상 λ=1e-2 는 막지만 λ=0.1 은 과감쇠로 오답을 내 마진이 좁다
- **테스트 함정**: 고친 경로만 검증하면 vacuous 하다. 끈 경로가 **실제로 이탈하는지**를 같은 테스트가 확인해야 한다

## CLAUDE.md / 문서 drift

### AP-DOC-1: 패키지 수·테스트 수·상수값·commit SHA·시점 의존 status 박제

- **증상**: 문서가 측정 시점에 의존하는 값을 박제 → 코드/측정값이 바뀌면 문서가 거짓이 됨. 예: "현 baseline 0건", "Phase X ✅ 완료", "현재 45개 상수 존재", `commit_sha 근거`
- **복구**: 측정값·status 은 SSoT (코드/git log/측정 명령) 위임. 문서엔 *어떻게 측정하는지* 박제하고 *값 자체*는 박제하지 않는다. Status 는 git log + memory 에 자연히 남는다
