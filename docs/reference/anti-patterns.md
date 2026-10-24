# Anti-Patterns — 사례집

> **이 문서는 헌법이 아니다.** 재발 패턴에서 나온 **규범**은 [agent_docs/invariants.md](../../agent_docs/invariants.md) 가 갖는다 — invariant 을 위반한 패턴은 그 invariant 행이, 대응 invariant 가 없는 패턴은 §Anti-pattern 규범 표가 AP ID 로 갖는다. 여기는 그 규범이 생긴 사례 · 증상 · 원인 · 복구 상세다. 둘이 어긋나면 invariants.md 가 옳다. **AP ID 의 정의는 이 문서의 헤더와 아래 흡수 표의 행**이며 (validate_docs.py D11), ID 는 재사용하지 않는다.

규범 행에 이미 있는 내용은 되풀이하지 않고, 그 행에 없는 증상 · 사례 · 탐지 명령만 둔다. 근거는 git log 의 fix-commit 이다.

## 흡수 · 결번

본문이 규범 행과 같아 남길 사례가 없는 ID 는 그 행으로 흡수했다 (ID 는 이력 참조용으로 유지).

| ID | 흡수처 |
|---|---|
| AP-RT-2 | invariants.md RT-6 행 (quaternion 선형 보간) |
| AP-RT-3 | invariants.md RT-5 행 (Eigen expression 의 타입 추론) |
| AP-RT-4 | invariants.md RT-4 행 (RT 경로의 mutex) |
| AP-RT-5 | AP-RT-1 · invariants.md RT-3 세부 스펙 (throttle 없이 tick 마다 반복되는 로그) |
| AP-ARCH-1 | invariants.md ARCH-1 행 — 탐지와 오탐 제외 이유는 verify-changes.sh Phase 0 주석 |
| AP-ARCH-2 | invariants.md ARCH-3 행 (interface 없는 두 번째 구현) |
| AP-ARCH-3 | invariants.md ARCH-4 행 — 대상 integration 패키지 집합 도출은 verify-changes.sh Phase 0 |
| AP-ARCH-5 | invariants.md ARCH-6 행과 ARCH-6 세부 스펙 (QoS depth 1) |
| AP-PROC-2 | invariants.md PROC-1 행 (코드 변경 시 YAML · README · CMake 동기화) |
| AP-PROC-3 | AP-DOC-1 (숫자 하드코딩 후 drift) |
| AP-PROC-4 | invariants.md PROC-6 행 (test assertion 약화) |
| AP-CTRL-3 | invariants.md NUM-4 행 (trajectory speed 0 클램프) — 1/v 가 INF 가 되어 crash 가 아니라 trajectory hang 으로 나타난다 |

> **결번**: AP-RTT-3 · AP-RTT-4 · AP-CTRL-2 · AP-CTRL-4 (은퇴 사유 미기록) · **AP-PROC-10** (2026-09-19 은퇴 — 판독 대상이던 C++ 빌드·테스트 CI `ros2-advanced-ci` 가 제거됐다; 사례는 git history). AP-RTT-5 는 `realtime_tools` 예방 항목이 아니라 in-tree affinity 라이브 결함이라 AP-THREAD-1 로 재분류됐다.

## RT Safety

### AP-RT-1: 정기 tick에서 `RCLCPP_*` 직접 호출 ([invariants.md](../../agent_docs/invariants.md) RT-3 위반)

- 규칙 · 허용 범위 · 탐지는 invariants.md RT-3 행 · RT-3 세부 스펙 · `detect id=RT-3` 블록. 고유 증상: 매크로가 string format + rosout IPC publish (heap + blocking) 를 하므로 rosout queue 가 차면 RT overrun → E-STOP 까지 가고, rate 가 높을수록 빨리 폭주한다. throttle 없이 tick 마다 반복되는 형태 (구 AP-RT-5) 도 같은 축이다
- 정기 tick 의 SPSC → `DrainLog()` 구현 예: `rtc_controller_manager/src/rt_controller_node_estop.cpp`

### AP-RT-6: torn-read snapshot

- 규범은 invariants.md §Anti-pattern 규범 AP-RT-6 행. 메커니즘: multi-field 구조체를 `memcpy` 로 복사하는 도중 writer 가 끼어들면 두 세대가 섞인 값이 나온다 — `SeqLock<T>` 의 reader-side retry 가 그 경우를 다시 읽는다

### AP-RT-7: RT path 에서 `std::condition_variable` notify/wait ([invariants.md](../../agent_docs/invariants.md) RT-10 위반)

- **증상**: RT producer 가 `cv.notify_one` 시 내부 mutex 보유 → 우선순위 역전 + 비결정 wake latency. wait side 는 명시 mutex lock 보유 (RT-4 결합)
- **원인**: producer-consumer 통지를 cv 로 구현. 직관적이나 RT 우선순위 보장 안 됨
- **본 repo 사례**: `udp_hand_driver/include/udp_hand_driver/udp_hand_controller.hpp` (eventfd 를 거쳐 현행 SeqLock + release flag 로 바뀌기 전) 의 `event_mutex_ + event_cv_ + event_pending_ + staged_cmd_` 패턴. `SendCommandAndRequestStates` 의 RT producer 가 `lock_guard + notify_one`, `EventLoop` 가 `unique_lock + cv.wait_for`
- **탐지**: [invariants.md](../../agent_docs/invariants.md) §위반 탐지 패턴 의 `detect id=RT-10` 블록
- **복구**:
  - **eventfd + non-blocking write** (`::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)`, producer `::eventfd_write(fd, 1)`, consumer `::poll(&pfd, 1, timeout_ms)` + `::eventfd_read(fd, &drained)`) — CM 의 nrt-publish lane (`RtControllerNode` 의 `nrt_publish_eventfd_` 생성부) 가 표준 패턴
  - SPSC + consumer polling (kEventTimeout 짧은 sleep) — wake latency = polling 주기
  - `std::atomic<bool>` release/acquire flag + self-clocked consumer — 고정주기 loop 이 매 tick latch 한다 (busy-spin 이 아니다; 규범은 invariants.md RT-10 (c)). 현행 `UdpHandController` 가 이 형태다 — command 를 SeqLock 에 stage 하고 `event_pending_` 을 release store, CommLoop 이 매 cycle latch

### AP-RT-8: RT tick 에서 `std::string` 을 값으로 반환하는 접근자 호출 ([invariants.md](../../agent_docs/invariants.md) §Anti-pattern 규범 AP-RT-8 — RT-1 의 할당이 SSO 뒤에 숨는 형태)

- **증상**: 없다. 이것이 이 항목의 요점이다 — 짧은 이름은 SSO(small-string optimization) 버퍼에 들어가 힙을 안 건드리므로 alloc 게이트도 green 이고 코드도 평범해 보인다. 이름이 SSO 임계(libstdc++ 15 자)를 넘는 순간 RT tick 이 `operator new` 를 부르기 시작하는데, 그때 바뀐 것은 **YAML 의 device group 이름 한 줄**뿐이라 원인이 코드에 없다
- **원인**: `RTControllerInterface::GetPrimaryDeviceName()` / `GetSecondaryDeviceName()` 은 `std::string` 을 **값으로** 반환한다 (`topic_config_.groups[i].first` 의 복사). 이름으로만 보면 조회 같아 tick 안에서 부르기 쉽다. `GetDeviceNameConfig(name)` 도 그 문자열을 인자로 받으므로 둘은 대개 붙어 다닌다
- **본 repo 사례**: `DemoInferenceController::PackObservation` 이 fingertip stride 를 tick 마다 `GetDeviceNameConfig(GetSecondaryDeviceName())` 로 읽었다 (stride 를 configure-time 에 캐시하기 전). 당시 그룹 이름이 SSO 임계보다 짧아 실측 allocation 은 0 이었고 게이트도 통과했다 — 결함이 아니라 **잠복**이었다
- **탐지**: RT tick 함수 본문에서 `Get*DeviceName()` 호출을 grep. 값 반환 접근자 일반으로 넓히려면 시그니처가 `std::string` (참조 아님) 인 것을 본다
- **복구**: configure(비-RT)에서 필요한 값을 **해석해 POD 멤버로 캐시**한다 (위 사례는 `fingertip_stride_` 하나). 문자열 자체가 tick 에 필요하면 `std::string_view` 또는 인덱스로 바꾼다. tick 에서 이름으로 map 을 조회하는 형태 자체가 이미 신호다 — device 인덱스는 `topic_config_.groups` 순서로 configure 때 고정된다

### AP-RTT-1: `realtime_tools` primitive 도입 시 예방 규칙 (dedicated thread / ctor heap / drop 추적)

- **도입 시 적용** — 세 함정 (dedicated thread · ctor/`reset` heap · `try_publish` silent drop) 의 규범은 invariants.md §Anti-pattern 규범 AP-RTT-1 행, primitive 선택은 invariants.md §RT pub/sub primitive catalog. `realtime_tools` 를 쓰지 않는 동안 대상이 0건이다
- 행에 없는 탐지: `RealtimePublisher` 는 instance 당 `publishingLoop` thread 를 하나 만들므로, `ps -eLf | grep <process> | wc -l` 이 `SystemThreadConfigs` 를 넘으면 `thread_config.hpp` layout 이 깨진 것이다. 본 repo 의 SPSC drain 경로는 logger 가 drop counter 를 이미 센다

### AP-RTT-2: 외부 라이브러리 thread 의 priority / affinity 가 thread_config.hpp 와 불일치

- 규범은 invariants.md §RT Host / Runtime Preconditions 의 외부 라이브러리 thread 불릿 (RT-HOST-2/3). 실제 소비자는 DDS thread (#163/#164) 이고 `realtime_tools::RealtimePublisher` 는 도입 시 같은 부류다
- 탐지: `verify_rt_runtime.sh` 의 thread 별 sched policy / affinity 검사에서 `SCHED_OTHER` thread 가 RT core 에 매핑돼 보인다. 복구는 native handle 로 `pthread_setschedparam` + `pthread_setaffinity_np`

### AP-THREAD-1: `ThreadConfig::cpu_core` 를 kernel logical CPU id 로 가정

- **증상**: SMT-on hybrid (NUC13 / NUC14 / i9-13900K) 또는 AMD SMT 시스템에서 RT thread 가 의도와 달리 P-core 의 SMT sibling 에 핀됨. `rt_callback` (slot 2 = cpu 2 = P-core 1 physical) 와 `mpc_main` (slot 3 = cpu 3 = P-core 1 sibling) 이 동일 hardware execution unit 의 두 hyperthread 에 들어가 cache/port contention 발생 — RT 우선순위 우위가 무력화됨
- **원인**: `ThreadConfig::cpu_core` 는 *slot index* (physical core 번호), `CPU_SET(n, ...)` 의 `n` 은 *logical CPU id*. SMT-off 환경에서만 두 값이 일치하므로 4-core CI mock 만으로는 회귀를 잡을 수 없음
- **탐지**: `ApplyThreadConfig` / `ApplyThreadConfigWithFallback` / `CheckThreadHealth*` 가 `SlotToLogicalCpu()` 없이 `cfg.cpu_core` 를 `CPU_SET` / `CPU_ISSET` 에 직접 전달하는지 — 패턴은 [invariants.md](../../agent_docs/invariants.md) §Anti-pattern 규범 의 `detect id=AP-THREAD-slot-mapping` 블록 (사본을 여기 두지 않는다)
- **복구**: 새 affinity 호출 site 가 추가되면 반드시 `SlotToLogicalCpu(slot)` 또는 `SlotToLogicalCpu(slot, topology)` 를 거쳐 변환. unit test 는 `CpuTopology` mock 으로 직접 주입 (overload 사용)
- **비고**: detect id `AP-THREAD-slot-mapping` 는 CI (validate_docs.py D7) 가 검증하는 load-bearing 문자열이라 이름을 바꾸지 않는다. 같은 slot→logical 축의 GRUB 설정 통일은 #152

## Design / Architecture

### AP-ARCH-4: Device boundary 누설 (대응 invariant 없음 — device group 경계는 표로 규정되지 않았다)

- 규범은 invariants.md §Anti-pattern 규범 AP-ARCH-4 행. 증상: Hand/ToF state 가 arm state 로 새거나 그 반대 → GUI 혼선, digital twin 오표시. 탐지: device group 별 publisher 분리 여부와 `SetDeviceTarget(device_idx)` 호출자의 인덱스 정합

## Process / Drift

### AP-PROC-1: "✅ complete" 주장 후 실제 미완료

- 규범은 invariants.md §Anti-pattern 규범 AP-PROC-1 행. 증상: 문서는 complete 인데 코드는 일부만 마이그레이션됐다 — PR 리뷰가 대상 파일 목록 없이 grep 검증을 건너뛴 결과다

### AP-PROC-5: ROS 2 Jazzy 파라미터 타입·launch 호환

- 규범은 invariants.md §Anti-pattern 규범 AP-PROC-5 행. 증상: `ParameterUninitializedException`, Jazzy 의 `namespace='/'` 거부, CycloneDDS domain 실패 — Humble→Jazzy 전환 때 파라미터 선언 API 변경을 반영하지 않은 결과다

### AP-PROC-6: BT coordinator 등록 누락

- 규범은 invariants.md §Anti-pattern 규범 AP-PROC-6 행. 증상: 런타임 BT 노드 not found, 테스트 중복 등록, validation fail. 탐지: `grep -l registerNodeType ur5e_bt_coordinator/` 결과에 새 노드가 있는지

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
  확인**: 신규 쪽에만 건 작은 섭동이 red 를 내는지 (관련: [invariants.md](../../agent_docs/invariants.md) PROC-6)

### AP-PROC-8: 컨트롤러 픽스처가 `on_activate` 없이 target 을 넣는다

- **증상**: 픽스처가 `SetDeviceTarget` 을 호출하는데 팔이 안 움직인다. 실패는 없고, 두
  컨트롤러를 비교하는 테스트라면 **둘 다 정지 상태로 green** 이다. 주석에는 "루프를 닫아
  팔이 실제로 이동한다" 라고 적혀 있다
- **원인**: #196 활성 세대(activation generation) 게이트. Inactive 중에 큐잉된 target 은
  정의상 stale 이므로 버려진다. `LoadConfig`/`SetDeviceNameConfigs` 까지만 하고
  `on_activate` 를 부르지 않은 픽스처는 **영원히 Inactive** 라 모든 target 이 그 구멍으로
  들어간다. 실측: `test_compliance_task_equivalence` 의 "nominal tracking" 프로그램이 S2~S3
  내내 정지한 팔 둘을 비교하고 있었다 (#469 S3 에서 발견)
- **탐지**: 픽스처의 bring-up 시퀀스에 `on_activate` 가 있는지 본다. 없는데 target 을 넣으면
  그 테스트는 target 경로를 안 탄다. 확인은 **이동량 단언** — `ASSERT_GT(travelled, ε)` 를
  비교/판정 *앞에* 두면 픽스처가 스스로 정지 상태임을 신고한다
- **복구**: bring-up 에 `on_activate(kInactive)` 를 넣는다. 이미 있는 테스트에 넣을 때는
  PROC-6 이 아니다 (단언 약화가 아니라 픽스처가 검사 경로를 타게 만드는 것). 넣은 뒤
  mutation 으로 그 경로가 실제로 살아났는지 확인한다

### AP-PROC-9: configure-time 검증을 그 값이 아직 없는 pass 에 둔다

- **증상**: configure-time 검증을 새로 넣었는데 **잘못된 config 가 통과하고 테스트는 green** 이다.
  검증 코드는 멀쩡히 있고 실행도 되므로 grep 으로도 리뷰로도 안 잡힌다. 반대 방향의 발현은
  요란하다 — 검증이 throw 하는데 lifecycle FAILURE 가 아니라 **configure 경로 자체가 탈출**한다
- **원인**: 파서 옆이 검증을 쓰기 가장 자연스러운 자리인데, CM 3-pass bring-up 에서는 검사 대상이
  **언제 채워지는가**가 pass 마다 다르다. 축은 "몇 번째 pass 인가" 가 **아니라 그 값이 어디서
  오는가** 다 (3-pass 계약 SSoT:
  [rtc_controller_interface/README.md](../../rtc_controller_interface/README.md#lifecycle-훅-ros2_control-정렬-기본-구현-제공)):
  - **모델 파생** — `SetSystemModelConfig` / `SetSharedModelBuilder` 는 `PreConfigure` **전**에
    주입되므로 Pass 1 `LoadConfig` 에서 이미 유효하다. `DemoWbcController` 의
    `integration.position_margin` 역전 거부가 거기 사는 이유다 (#473). **Pass 1 검증이 일반적으로
    틀린 것이 아니다**
  - **device 파생** (`device_name_configs_` → `LoadDeviceLimitsFromConfig` 가 채우는 한계 밴드)
    — Pass 1 에는 **없다**. 거기 쓴 검사는 loader 의 ±2π placeholder fallback 만 보므로 2π 미만의
    어떤 값도 통과시키는 **공허한 게이트**가 된다
  - **Pass 2** (`SetDeviceNameConfigs` → `OnDeviceConfigsSet`) 는 값은 있지만 **거부 채널이 없다**
    — CM 의 호출부가 try/catch 밖이라 여기서 던진 throw 는 FAILURE 로 번역되지 않고 configure 를
    통째로 빠져나간다. 파생·캐시 전용 자리다
  - **Pass 3** `on_configure` 만 둘 다 갖는다 — 값이 채워져 있고, 바인딩의 기존 try/catch 가
    로그를 남기고 `CallbackReturn::FAILURE` 를 내며 CM 이 그것을 bring-up 거부로 바꾼다

  실측: `#483` 이 compliance 의 `joint_limit_margin` 역전 거부를 처음 `LoadConfig` 에 넣었다.
  밴드가 없으니 ±2π fallback 만 검사하는 공허한 게이트였고, 별개 경로를 쫓다가 드러났다
- **탐지**: **코드의 존재가 아니라 거부 동작을 건다.** 검증을 넣었으면 잘못된 config 로 CM 순서
  (Pass 1 → Pass 2 → Pass 3) 를 그대로 태워 `on_configure` 가 실제로 `FAILURE` 를 내는지 테스트한다.
  정상 config 와 **쌍으로** 단언해야 "그 값 때문에" 가 성립한다 — 같은 try 블록의 다른 실패도
  똑같이 FAILURE 이므로 단독 red 는 귀속되지 않는다. 임계값은 **device config 에서 유도**하고
  소스에 박지 않는다 (박으면 한계가 바뀐 날 조용히 vacuous 해진다 — AP-DOC-1). 예:
  `test_compliance_admittance_coupling.cpp` 의 `ComplianceMarginGate`
- **복구**: 검증을 `on_configure` 로 옮기고 **RT 경로가 쓰는 것과 같은 fallback 으로** 밴드를
  해석한다 (검사와 RT clamp 가 서로 다른 관절을 얘기하면 안 된다). Pass 2 는 파생·캐시만 하고
  거부하지 않는다. 옮긴 뒤 mutation 으로 확인 — 게이트를 무력화했을 때 red 가 나야 그 테스트가
  게이트를 잡고 있는 것이다 ([invariants.md](../../agent_docs/invariants.md) PROC-6)

## Controller-Specific

### AP-CTRL-1: Mid-tick gains branch 불일치

- 규범은 invariants.md §Anti-pattern 규범 AP-CTRL-1 행. 메커니즘: aux thread 의 게인 writer (parameter callback) 가 `Compute()` 중간에 끼어들면 bool flag 가 절반만 갱신된 상태로 분기한다. 표준형은 진입 시 `const auto gains = gains_lock_.Load();` 한 번

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

### AP-NUM-1: residual 로 폐쇄 체인 조립 분기를 판정 ([invariants.md](../../agent_docs/invariants.md) NUM-5 위반)

- **증상**: closed-chain FK 가 엉뚱한 형상으로 렌더링/제어된다. passive joint 가 ~180° 뒤집히거나 여러 바퀴 감긴 값이다. 그런데 `converged=true`, `‖φ‖≈1e-8` 로 **모든 건전성 검사가 통과**한다
- **원인**: 점(`CONTACT_3D`) 구속 loop 은 조립 분기가 여럿이고 **모두 φ=0 을 만족**한다. 무감쇠 Gauss-Newton 사영에 큰 seed 점프를 한 번에 넘기면 반대편 분기로 착지하고, warm-start 구조상 영구 고정된다 (#248)
- **발동 경로** (전부 "한 호출에서 seed 가 크게 움직임"): cold start (초기 seed=`q_ref`≈neutral → 첫 실측 포즈), 빠른 동작 (관절 속도 한계 × 발행 주기 > 이탈 임계), 메시지 유실 (`QoS(1)` 드롭 → 실효 스텝 배가)
- **탐지**: `‖φ‖` 는 무력하다. **미세 continuation 기준해와 형상을 직접 대조**하거나, 사영 전후 passive 이동량을 본다. `grep -rn "ProjectPassiveToConstraint" --include=*.cpp` 로 continuation 을 거치지 않는 직접 호출을 찾는다
- **복구**: 비-RT 는 `ProjectPassiveWithContinuation`, RT 는 tick 당 seed clamp (NUM-5 세부 스펙). λ 상향은 band-aid 다 — 실측상 λ=1e-2 는 막지만 λ=0.1 은 과감쇠로 오답을 내 마진이 좁다
- **테스트 함정**: 고친 경로만 검증하면 vacuous 하다. 끈 경로가 **실제로 이탈하는지**를 같은 테스트가 확인해야 한다

## 문서 drift

### AP-DOC-1: 패키지 수·테스트 수·상수값·commit SHA·시점 의존 status 박제

- **증상**: 문서가 측정 시점에 의존하는 값을 박제 → 코드/측정값이 바뀌면 문서가 거짓이 됨. 예: "현 baseline 0건", "Phase X ✅ 완료", "현재 45개 상수 존재", `commit_sha 근거`
- **복구**: 측정값·status 은 SSoT (코드/git log/측정 명령) 위임. 문서엔 *어떻게 측정하는지* 박제하고 *값 자체*는 박제하지 않는다. Status 는 git log + memory 에 자연히 남는다
- **본 repo 사례 1 — 규칙 목록 사본**: 당시 CLAUDE.md 의 §3 에 박혀 있던 RT 금지 7개 목록이 invariants.md 의 9개와 갈라져 RT-9·RT-10 이 헌법에서 사라졌다. 같은 자리의 ARCH 5줄 사본도 ARCH-4 (integration 패키지의 `rtc_*/src/` private 헤더 include 금지, Critical) 와 ARCH-6 (QoS depth 1) 을 통째로 빠뜨린 채 굳어 있었다 (#229). 복구: 헌법은 ID 범위와 SSoT 포인터만 갖고 요약은 path-scoped rule 이 갖는다 — 어떤 파일 편집 시 로드되는지도 rule frontmatter glob 이 SSoT 이므로 박제하지 않는다
- **본 repo 사례 2 — hook 서술 사본**: 당시 CLAUDE.md 의 §4 가 Stop hook 의 변경 집합 산정·allowlist·편입 이력을 291단어로 들고 있으면서 같은 단락에서 "hook 주석이 SSoT" 라고 선언했고, modification-guide.md 는 거꾸로 그 §4 를 SSoT 로 지목해 순환이 됐다 (#527). 복구: hook 헤더가 소유, 헌법은 3줄 요약 + 포인터
- **본 repo 사례 3 — 등가 주장**: 목록을 복제하지 않고 "hook 이 이 목록과 같은 것을 수행한다" 는 한 줄만 남겨도 같은 drift 다. 헌법 통합 때 AGENTS.md §4 가 "Stop hook 이 자동 수행·차단하는 것과 같은 목록" 이라 적었으나 hook 은 포매팅·Doxygen 을 보지 않았고, 그 주장 탓에 Bash 로 쓴 파일의 포맷 누락 (PostToolUse 를 안 탐) 을 잡는 게이트가 없다는 사실이 가려졌다. 복구: 헌법은 "기계 판정 가능한 부분만 대신한다" + hook 헤더 포인터, 여집합은 modification-guide.md §Completion Checklist 가 소유

### AP-DOC-2: 판단의 *근거*를 두 번째 위치에 복사한다 (주석 포함)

- **증상**: 같은 설계 판단의 근거가 두 곳 이상에 서술돼 있고, **틀릴 때 같이 틀린다**. AP-DOC-1
  이 값의 박제라면 이건 *이유*의 박제이고, 근거에는 검증기가 없어서 코드가 그 근거를 배신해도 사본이
  모두 조용히 남는다. 실측: `command_divergence` 를 배선하지 않은 이유를 `compliance/compute.cpp`
  fault 블록 · `compliance_diag_log_pod.hpp` 헤더 · lane plan 세 곳이 똑같이 "이 바인딩은
  `integrate_from_measured: false` 를 제공하지 않는다" 고 적었지만, `WriteArmJointCommand` 가
  `desired_q_ += dq_·dt` 로 명령을 적분하므로 **그게 바로 그 모드**였다 (#469 리뷰). 소유자 한 곳
  (fault 블록) 으로 접은 뒤 그 판단이 다시 고쳐졌을 때 (#478 — 결론 "real gap" 이 틀렸고 대응물은
  `pose_error_limit`) **편집도 한 번**이었다
- **원인**: 인접 파일이 같은 주제를 다루면 "여기서도 설명해 두면 친절하다" 가 자연스럽다. 그런데
  근거를 고치는 사람은 **자기가 편집 중인 파일 하나만** 고친다 — 두 번 쓰는 사람은 있어도 두 번
  고치는 사람은 없다
- **탐지**: 주석/문서에 판단을 쓰기 전에 그 판단의 핵심 명사 (`command_divergence`,
  `integrate_from_measured`) 로 repo 를 grep 한다. 이미 서술한 곳이 있으면 **가리킨다**
- **복구**: 근거는 **그 판단이 코드로 나타나는 지점**이 소유한다 (fault 를 안 켜는 이유는 fault
  블록, 컬럼이 없는 이유는 POD 헤더). 두 번째 위치는 자기가 소유한 규칙만 적고 나머지는 파일명으로
  가리킨다 — "왜 안 켜는가" 와 "왜 컬럼이 없는가" 는 다른 질문이고 소유자도 다르다. 이 "축을
  나눈다" 기법의 전개형은 [conventions.md](../../agent_docs/conventions.md#documentation-requirements) 의
  **서비스·메시지 계약의 소유자** 항목이 이미 갖고 있다 (`.srv` / README / `.cpp` 헤더 4축).
  거기는 wire 계약, 여기는 **설계 판단** — 같은 처방, 다른 대상
- **두 번째 실측 — 축을 나눈 결과 (#488)**: `joint_limit_margin` 게이트가 왜 파서가 아니라 바인딩에
  있는가를 `compliance/lifecycle.cpp` 블록 · `ComplianceMarginGate` fixture · AP-PROC-9 가 3중으로
  유도하던 것을 접어, lifecycle 블록은 *무엇을 왜 거부하는가*, fixture 는 *무엇을 단언하는가*, pass
  선택 규칙은 **AP-PROC-9 한 곳**이 갖게 됐다
- **세 번째 실측 — 규칙이 바뀌는 날 사본 수만큼 고친다 (#742)**: 손의 폐쇄 lead 를 어느 축의 값으로
  실행하는가 (`mpc_docking` 아래의 환산) 가 두 설계 문서 · integrated_bringup
  README · 두 로봇 YAML 주석 (YAML 은 환산 수치까지) 다섯 곳에 있어, 규칙이 바뀌자 다섯 곳을 고치고
  완료는 grep 으로만 확인할 수 있었다. 접은 뒤 규칙과 식은 `EntranceCloseLead`
  ([docking_config.hpp](../../rtc_controllers/include/rtc_controllers/catching/docking_config.hpp)),
  환산 수치는 측정 표 한 곳이 갖고 나머지는 자기 축만 적고 가리킨다
