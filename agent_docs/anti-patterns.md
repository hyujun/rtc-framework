# Anti-Patterns

재발을 방지해야 할 실수 패턴. 각 항목은 동일 구조:
- **증상**: 무엇이 일어나는가
- **원인**: 왜 일어나는가
- **탐지**: grep/test 명령 (가능한 경우)
- **복구**: 올바른 방법

근거는 git log 의 historical fix-commit + [archive/controller-safety-improvements.md](archive/controller-safety-improvements.md). 절대 카운트·commit SHA·시점 의존 status (`현 baseline 0건`, `Phase X ✅`) 는 박제하지 않는다 (AP-DOC-1) — 패턴 자체가 핵심이고 측정값은 측정 시점에 의존한다.

## RT Safety

### AP-RT-1: 정기 tick에서 `RCLCPP_*` 직접 호출 ([invariants.md](invariants.md) RT-3 위반)

- **증상**: 정기 tick (`control_rate`) 루프에서 지터 스파이크, rosout queue 포화 시 RT overrun → E-STOP. Rate 가 높을수록 폭주가 빠름 (rate-proportional)
- **원인**: `RCLCPP_*` 매크로가 내부적으로 string format + rosout IPC publish → heap + blocking
- **탐지**: `grep -nE 'RCLCPP_(INFO|WARN|ERROR|DEBUG|FATAL)\(' <RT file>` — 단 one-shot init / THROTTLE 변종은 제외
- **복구**:
  - one-shot init: 현상 유지 (lifecycle 콜백 / 한 번만 실행되는 fatal 경로)
  - 정기 tick: SPSC → `DrainLog()` aux thread 패턴 (참조: `rtc_controller_manager/src/rt_controller_node_estop.cpp`)
  - throttle 필요: `RCLCPP_*_THROTTLE` + RT-safe msg (단순 format + 기본 타입만)

### AP-RT-2: Quaternion `lerp` ([invariants.md](invariants.md) RT-6 위반)

- **증상**: Non-unit quaternion → 회전축 축소/왜곡, 누적 drift
- **원인**: `Eigen::Quaterniond::slerp`가 아닌 선형 보간 오용
- **탐지**: `grep -rnE '(nlerp|\.lerp\()' --include='*.cpp' --include='*.hpp' .`
- **복구**: `q_a.slerp(t, q_b)` 명시

### AP-RT-3: Eigen expression에 `auto` ([invariants.md](invariants.md) RT-5 위반)

- **증상**: Expression template lazy-eval + aliasing → 같은 메모리 읽고 쓰기 → 결과 쓰레기
- **원인**: `auto M = A * B;` 는 `Eigen::Product` 를 담고 나중 평가 시 aliasing 가능
- **탐지**: `grep -nE 'auto [^=]*=.*\.(matrix|transpose|inverse|adjoint|block)\(' <file>`
- **복구**: 명시 타입 `Eigen::MatrixXd M = A * B;` 또는 `A.noalias()` 명시

### AP-RT-4: RT 경로 `std::lock_guard` ([invariants.md](invariants.md) RT-4 위반)

- **증상**: Aux thread가 holding 중이면 RT blocks → 100 µs+ stall
- **원인**: Priority inheritance 없는 일반 mutex는 RT 태스크가 non-RT를 기다림
- **복구**: `SeqLock<T>` (single-writer/multi-reader, trivially copyable만), `SpscBuffer`, atomic, `std::try_to_lock`

### AP-RT-5: 정기 tick에서 unguarded log

- **증상**: 1 tick당 여러 줄 로그 × 정기 tick 주파수 = rate-proportional 폭증
- **복구**: `RCLCPP_*_THROTTLE(logger, clock, period_ms, "fmt", args...)` — msg는 [invariants.md](invariants.md) RT-3 세부 스펙 준수

### AP-RT-6: torn-read snapshot

- **증상**: multi-field 구조체를 atomic 없이 복사 → 중간에 writer가 업데이트 → torn
- **원인**: `memcpy(&out, &shared, sizeof)` 도중 writer 개입
- **복구**: `SeqLock<T>` Load (reader-side retry loop), 또는 `std::atomic<T>` (POD만)

## Design / Architecture

### AP-ARCH-1: `rtc_*` 패키지에 robot-specific 상수 하드코딩 ([invariants.md](invariants.md) ARCH-1 위반)

- **증상**: 다른 로봇에서 재사용 불가 → 패키지 fork 압력
- **탐지**: `grep -rniE '(\bur5e\b|6.?dof|10.?dof|num.?joints = [0-9])' rtc_*/include rtc_*/src`
- **복구**: YAML 주입 또는 template parameter

### AP-ARCH-2: Interface 없이 두 번째 구현 추가 ([invariants.md](invariants.md) ARCH-3 위반)

- **증상**: 세 번째 구현 시 `#ifdef` / switch 지옥
- **복구**: 첫 두 구현 리팩터 → abstract base + `RTC_REGISTER_*` factory 패턴

### AP-ARCH-3: 역방향 include ([invariants.md](invariants.md) ARCH-4 위반)

- **증상**: `rtc_*` robot-agnostic 훼손
- **탐지**: `grep -rn '#include "ur5e' rtc_*/include` / `grep -rn '#include "rtc_.*/src/' ur5e_*/`
- **복구**: 공개 API만 사용, interface injection

### AP-ARCH-4: Device boundary 누설

- **증상**: Hand/ToF state가 robot arm state로 새거나 그 반대 → GUI 혼선, digital twin 틀림
- **원인**: 장치 그룹 경계 무시한 공용 버퍼 / state publisher
- **탐지**: device_group별 publisher 분리 여부, `SetDeviceTarget(device_idx)` 호출자에서 인덱스 정합
- **복구**: device_group당 별도 publisher, state/target 모두 `device_idx` tagging

## Process / Drift

### AP-PROC-1: "✅ complete" 주장 후 실제 미완료

- **증상**: 문서에는 complete, 코드는 일부만 마이그레이션
- **원인**: PR 리뷰에서 grep 기반 verification 누락
- **탐지**: `grep -c <new_pattern>` 예상치 vs 실측 — 대상 파일 목록 명시 후 전수 grep
- **복구**: complete 체크 전 전수 grep, 대상 파일 목록 명시 후 체크

### AP-PROC-2: Code-only without YAML/README/CMake 동기화 ([modification-guide.md](modification-guide.md) PROC-1 위반)

- **증상**: 빌드 실패, 파라미터 ParameterUninitializedException, 런타임 NotFound
- **복구**: [modification-guide.md](modification-guide.md) Completion Checklist 전항목

### AP-PROC-3: 숫자 하드코딩 후 drift

- **증상**: 문서 A = N, 문서 B = N+1, 실측 = N+k (mesh count / 테스트 수 / 패키지 수 등 historical 사례 다수)
- **복구**: 단일 출처 (코드/YAML/git) + 측정 명령 박제. 문서엔 수치 자체를 넣지 말 것

### AP-PROC-4: 기존 test assertion을 통과시키려 수정 ([invariants.md](invariants.md) RT-7 위반)

- **증상**: 회귀 은폐
- **탐지**: `git diff test/` 에서 `EXPECT_*` / `ASSERT_*` 상수 변경
- **복구**: 새 코드를 고쳐라. assertion이 진짜 틀렸다면 별도 commit으로 근거 제시

### AP-PROC-5: ROS 2 Jazzy 파라미터 타입·launch 호환

- **증상**: `ParameterUninitializedException`, `namespace='/'` Jazzy 거부, CycloneDDS domain 실패
- **원인**: Humble→Jazzy 전환 시 파라미터 선언 API 변경 미반영
- **복구**: `ParameterDescriptor` 명시, `declare_parameter<T>(name, default, descriptor)` 타입 파라미터 사용, LifecycleNode는 `namespace=''` (empty string)

### AP-PROC-6: BT coordinator 등록 누락

- **증상**: 런타임에 BT 노드 not found, 테스트 중복 등록, validation fail
- **원인**: `BT::BehaviorTreeFactory::registerNodeType<>` 호출 누락 또는 `validate_tree()`에 새 노드 미추가
- **탐지**: `grep -l registerNodeType ur5e_bt_coordinator/` 에 새 노드 포함 확인
- **복구**: validate_tree / registerNodeType 양쪽 업데이트 — BT 노드 신설 시 체크리스트

## Controller-Specific

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

## CLAUDE.md / 문서 drift

### AP-DOC-1: 패키지 수·테스트 수·상수값·commit SHA·시점 의존 status 박제

- **증상**: 문서가 측정 시점에 의존하는 값을 박제 → 코드/측정값이 바뀌면 문서가 거짓이 됨. 예: "현 baseline 0건", "Phase X ✅ 완료", "현재 45개 상수 존재", `commit_sha 근거`
- **복구**: 측정값·status 은 SSoT (코드/git log/측정 명령) 위임. 문서엔 *어떻게 측정하는지* 박제하고 *값 자체*는 박제하지 않는다. Status 는 git log + memory 에 자연히 남는다
