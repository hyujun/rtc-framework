# Modification Guide

수정·추가 작업의 **규범** 이다 — 어떤 순서로 하고, 어떤 게이트를 지나며, 무엇이 성립해야 완료인가. 단계별 절차와 각 단계의 이유는 헌법 밖 [docs/modification-procedures.md](../docs/modification-procedures.md) 가 갖는다 (같은 절 이름으로 찾는다).

## Workflow Loop

```
0. Type     → "수정"인가 "추가(새 기능/컨트롤러/메시지/디바이스/스레드)"인가?
              추가라면 design-principles.md 5원칙 + 본 문서 "Adding a New ..." 절을 먼저 읽는다.
              · rtc_* 에 추가 → P1·P2 + ARCH-3 (같은 종류 두 번째 구현이면 base 부터)
              · integration 패키지에 추가 → rtc_* 에 재사용·일반화할 것이 있는지 먼저 검토
1. Locate   → 파일의 RT / aux / robot-specific 역할 판단
2. Read     → package.xml + CMakeLists.txt + target file + 인접 테스트, 영향받는 invariant
3. Edit     → minimal, single-concern. RT path 여부 재확인
4. Build    → ./build.sh --tests -p <pkg> (rtc_base/rtc_msgs 변경 시 --tests full)
5. Test     → testing-debug.md Sensor Matrix. 버그 수정 시 회귀 테스트 추가
6. Verify   → 본 문서 Completion Checklist 통과
```

- **4·5·6 은 반드시 수행한다.** `--tests` 없이 빌드한 패키지의 `colcon test` 는 테스트 0개를 통과로 보고한다. 무엇을 돌릴지는 [AGENTS.md](../AGENTS.md) §4 "커밋 전에 직접 돌려야 하는 것" 이 SSoT 다.
- Claude Code 에서는 Stop hook 이 그중 기계 판정 가능한 부분을 차단하고 빌드·테스트는 verdict 만 확인한다 — 4·5 의 최종 실행은 `.claude/hooks/verify-changes.sh --run` 으로 한다. hook 이 무엇을 검사하는지는 그 헤더가 SSoT 이고, 다른 도구에서는 4·5·6 을 직접 실행한다.
- **차단 탈출은 리포트 대응뿐이다.** Claude Code 는 8회 연속 차단 뒤 hook 을 override 하고 turn 을 끝내지만, 그것은 탈출구가 아니라 미검증 종료다.

### Workflow Fail-Safe

"Try harder" 는 실패 응답이 아니다 — 누락된 capability 를 엔지니어링하거나 [AGENTS.md](../AGENTS.md) §6 Escalate.

| 실패 단계 | 증상 | 대응 |
|----------|------|------|
| 1. Locate | 파일을 찾을 수 없음 | broad search. "찾았다고 추정" 금지 |
| 2. Read | 컨텍스트 불충분 | 인접 파일 + 테스트 추가 읽기. 추측하지 말 것 |
| 3. Edit | Invariant 위반 유혹 | [invariants.md](invariants.md) 확인 후 Escalate. 우회로 찾지 말 것 |
| 4. Build | 빌드 실패 | 에러 메시지를 **먼저** 기록. 원인 파악 전 재시도 금지 |
| 5. Test | 테스트 실패 | **새 코드를 고친다.** assertion 쪽을 손대야 할 것 같으면 착수 전 PROC-6 을 편다 |
| 6. Verify | Checklist 항목 실패 | 해당 항목까지 rollback, 재실행. 부분 완료 주장 금지 (AP-PROC-1) |

## Sprint Contract & Spec (착수 전 성공 기준)

언제 협상하고 무엇이 면제인지는 [AGENTS.md](../AGENTS.md) §6.5 가 갖는다. 코드 수정 시작 *전* 1~3줄로 성공 기준을 제시하고 컨펌받는다:

```
[SPRINT] <task 한 줄 요약>
Done when:
  - <검증 가능 기준 1>
  - <검증 가능 기준 2>
Out of scope: <명시적으로 하지 않을 것 — drift 방지>
```

- 기준은 **객관 검증 가능** 해야 한다 ("test_X 통과", "grep 0건"). "깔끔하다" · "잘 작동한다" 는 기준이 아니다. task 종료 보고 ([AGENTS.md](../AGENTS.md) §11) 에서 항목별 충족 여부를 체크한다.
- **Spec-driven** (신규 abstract interface · controller · 메시지 · 디바이스 추가): Sprint Contract = spec. 구현 전 private plan 파일에 *왜 필요한가 · API surface · 검토한 alternatives* 를 적는다. 같은 파일이 progress 와 handoff 도 누적하므로 `## Spec` / `## Progress` / `## Handoff` 로 구분한다 (저장 위치는 각 도구의 문서, [handoff.md](handoff.md) §5).

## Adding a New Controller

새 컨트롤러는 **코어 (법칙) + 바인딩 (프레임워크 계약)** 둘로 나눠 쓰며, 한 클래스로 쓰지 않는다 ([design-principles.md](design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms). 옛 커밋의 상속 어댑터를 템플릿으로 삼지 않는다 — 복사할 출발점은 integration 패키지의 바인딩이다.

- **코어** — `rtc_controllers/include/rtc_controllers/<family>/` 에 법칙만, 그 옆에 `Params` POD + `ParseXxxParams` (design-principles.md §코어의 형태).
- **바인딩** — integration 패키지에서 `RTControllerInterface` 를 상속하고 코어를 멤버로 소유한다. `Compute()` · `SetDeviceTarget()` · `Name()` 은 `noexcept`. 무엇을 읽고 무엇을 스스로 계산하는지는 design-principles.md §Backend / Controller Layering.
- **`Name()` 은 전역 유일해야 한다** — `Name()` 과 `config_key` 는 하나의 lookup 네임스페이스라 겹치면 bring-up 전체가 거부된다. 한 클래스를 두 `config_key` 로 등록할 수 없다.
- **Runtime gains** — 바인딩의 LifecycleNode parameter 로 노출한다 (`DeclareGainParameters()` + on-set callback). one-shot 이벤트는 srv 를 상대 이름으로 advertise 한다. 코어는 파라미터 채널을 갖지 않는다 ([controllers.md](controllers.md) §Gains).
- **Gains struct 는 trivially copyable** 이어야 한다 (`rtc::SeqLock<Gains>`). RT path 는 method entry 에서 한 번 `Load()` 한다.
- **게인 하한은 로더와 tick 양쪽에 건다** — 바인딩 `Compute()` 가 코어 파서와 같은 floor helper 를 한 번 더 부른다 ([invariants.md](invariants.md) §NUM floor 규약). 그 패키지 테스트에 "tick 에서 floor 된다" 케이스를 넣되, σ₀ 는 랭크 결손 자세에서 판정한다 (잘 조건화된 자세에서는 공허하다).
- **YAML** — production 은 바인딩과 같은 패키지의 `config/<robot>/controllers/` 다 (`rtc_controllers/examples/` 에 추가하지 않는다). `topics:` 섹션을 반드시 포함한다. 유효한 `role:` 문자열은 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §구독 역할 · §퍼블리시 역할 이 SSoT 다 — 추측하지 말고 표를 본다 (없는 문자열은 configure 실패).
- **토픽을 소유하는 바인딩** 은 lifecycle 훅과 `PublishNonRtSnapshot` 을 override 하고 `owned_topics` 헬퍼에 위임한다. **`on_activate` override 는 base 를 먼저 호출한다** (base 가 activation generation 증분과 target 초기화 reset 을 한다 — 누락하면 stale target 이 재활성화 첫 tick 에 적용된다). target-init latch reset 은 `ResetTargetInitialization()` override 에 둔다.
- **등록** — `RTC_REGISTER_CONTROLLER()` 의 대상은 항상 바인딩이고 integration 패키지의 `controller_registration.cpp` 에 둔다. `config_key` 는 전역 유일, `config_package` 는 자기 패키지다 (`rtc_*` 이름을 넣으면 ARCH-1).

## Adding a New Message Type

- **`PublishRole` 을 늘리기 전에 소유 형태부터 정한다. 기본 답은 추가하지 않는 것이다** — controller-owned non-RT 토픽은 controller 의 `SeqLock<T>` 멤버 + `owned_topics` 의 `Setup*Publisher` 헬퍼로 만든다 ([design-principles.md](design-principles.md) §Controller-YAML Topics Are Controller-Owned).
- 그래도 새 `PublishRole` 이 필요하면 **네 곳을 같은 변경 안에서** 고친다: enum, `PublishRoleToString()`, YAML 파서 매핑, 그리고 **실제 publisher**. 앞의 셋만 하면 에러 없이 죽은 토픽이 생긴다. **E-11 이므로 착수 전 `[CONCERN]`**.
- `rtc_msgs` 에 `.msg` 를 추가하는 것은 E-3 (Critical) 이고 PROC-3 전체 빌드가 따른다.

## Adding a New Device Group

- **`devices.<group>.backend:` is the SSoT** (`{sim,robot}.yaml`) — device-wire 토픽을 controller YAML 에 선언하지 않는다.
- 설정된 모든 device group 은 자동으로 준비 게이트 + 워치독 대상이다 — timeout 목록에 없으면 `device_timeout_default_ms` 가 적용된다.
- 기존 backend 에 설정 키만 필요하면 backend 를 추가하지 않는다 (ARCH-3 의 "두 번째 구현" 이 아니다). **단일 backend 전용 신규 key 는 `rtc_base` 타입 확장이 아니라 그 backend 의 `Configure()` 에서 nested ROS 2 param 으로 읽는다** (`rtc_base` 타입 변경은 PROC-3 전체 빌드를 부른다). 같은 key 가 두 번째 backend 에 필요해지는 순간이 `rtc_base` 승격 트리거다.
- 새 backend type 은 `DeviceBackend` interface 를 구현하고 `RTC_REGISTER_DEVICE_BACKEND` 로 등록한다. `ReadState()` / `WriteCommand()` 는 RT-safe 다. 무엇을 채우는지는 design-principles.md §Backend / Controller Layering.

### Renaming a Device Group

Group 이름은 config YAML 밖에도 박혀 있어서 `devices:` 블록만 고치면 조용히 dead topic 이 남는다. 다음을 전부 grep 한다: Python 스크립트 / GUI 의 토픽 조립 경로 · **C++ 기본값** (멤버 in-class initializer / struct default / `declare_parameter`) · group 파생 helper · CSV / 로그 컬럼명 · 문서의 토픽 표. 검증 신호: rename 후 `ros2 topic list` 에 old 이름이 남거나 구독자 0 인 신규 토픽이 보인다.

## Adding a New Thread

스레드 배치는 선언형 manifest 한 곳이 소유한다 — tier 상수를 손으로 쓰지 않는다.

- [repo_scripts/config/thread_layout.yaml](../repo_scripts/config/thread_layout.yaml) 에 role 을 추가하고 **모든 tier** 에 값을 준다. 컨트롤러 프로세스 안의 스레드는 검증기 대상에도 넣는다.
- `gen_thread_layout.py --write` 로 생성물을 갱신한다. 생성 파일은 **직접 편집 금지** 다 (`--check` 가 차단).
- `SystemThreadConfigs` 필드와 generator 의 `field_order` 는 **구조체 선언 순서 그대로** 맞춘다. manifest 레벨 불변식은 generator 의 self-test 에 넣는다.
- **각 언어의 리터럴 oracle 을 갱신한다** (C++ · shell · Python 테스트의 손으로 쓴 기대값). 잘못된 manifest 는 세 언어에서 일치하므로 `--check` 로는 잡히지 않는다.
- 스레드 진입점에서 `ApplyThreadConfig()` 를 호출한다.
- layout **값** 변경 (기존 role 의 코어 이동) 은 리팩터가 아니라 thread model 변경이다 — **E-7**.

## Adding a New Package (new colcon directory)

새 `<pkg>/package.xml` 디렉토리를 추가하면 build SSoT ([rt_common.sh](../repo_scripts/scripts/lib/rt_common.sh) 의 `get_base_packages()` 또는 `get_robot_packages()`) 를 갱신한다 — 누락하면 downstream 의 클린 빌드가 `find_package` 에서 실패한다. README 패키지 표와 [architecture.md](architecture.md) dependency graph 는 PROC-1 을 따른다.

## Updating an Existing Package

코드 변경은 대응 문서·메타데이터 동기화를 포함해야 완료다 (PROC-1). 동기화 대상:

- **Tests** — affected suite 갱신 + 신규 동작의 test 추가 (기존 assertion 을 건드려야 하면 PROC-6 을 먼저 편다)
- **CMakeLists.txt** — source / install / `find_package` / test 등록의 일관성
- **package.xml** — `find_package` 와 1:1 매칭. 예외: rosdep 이 해결할 수 없는 source-install 의존은 CMake 에만 두고 `package.xml` 에 사유를 주석으로 남긴다. `<buildtool_depend>ament_python</buildtool_depend>` 은 넣지 않는다 (`<export><build_type>` 만으로 충분하다). test 전용 결합은 `<depend>` 가 아니라 `<test_depend>` 다
- **YAML config** — 추가 / 제거 / 이름변경된 parameter, `topics:` 섹션, valid range · unit 주석. robot-specific 값은 integration 패키지에, 기본값은 agnostic 패키지에
- **Doc** — package README (API / parameter / usage), inline Doxygen, cross-package 변경이면 root README + `docs/*.md`

`rtc_base` / `rtc_msgs` 변경은 전체 downstream 빌드·테스트다 (PROC-3).

## Completion Checklist

Claude Code 의 Stop hook 이 검사하는 범위의 **여집합** 이다 — hook 이 green 이어도 직접 확인한다. hook 이 없는 도구에서는 먼저 [AGENTS.md](../AGENTS.md) §4 의 빌드·테스트·포맷·doc validation 을 수행하고 그 위에 아래를 더한다.

- [ ] `package.xml` 의 **deps 의미 · version** — 선언된 dep 이 실제로 맞는가
- [ ] YAML 의 **default 값 · 유효 범위 · unit 주석**
- [ ] public header 의 **Doxygen**, cross-package doc 일관성
- [ ] **Python lint** (`ruff check`)
- [ ] RT path 변경 시 [invariants.md](invariants.md) §위반 탐지 패턴 의 `detect` 블록으로 자가검사

## Inferential review 트리거 (LLM-as-judge, 수동 trigger)

computational sensor (build / test / grep) 는 의미 회귀 — 설계 일관성, robot-agnostic 위반, abstract interface 누락, 재사용 가능성 — 를 잡지 못하고, 에이전트의 자기 평가는 그 대체가 아니다. 다음 상황에서 사용자에게 review 실행을 권한다 (`/code-review` · `/security-review` · `/simplify` 는 Claude Code slash command — 다른 도구에서는 동등한 수동 review):

- `rtc_base` / `rtc_msgs` 변경 → `/code-review`
- Abstract interface 신설 / 두 번째 구현 추가 (ARCH-3) → `/code-review`
- `rtc_*` 에 robot-specific 코드 추가 의심 (ARCH-1) → `/code-review`
- E-STOP 경로 / safety publisher / lifecycle 콜백 수정 → `/security-review` (E-8)
- PR 준비 (다파일 / 다패키지) → `/code-review ultra`
- 100+ 줄 변경 또는 신규 패키지 디렉토리 → `/code-review`
- 다파일 리팩터 / 유사 기능 중복 의심 (P5) / 변경 후 정리 → `/simplify` (버그 탐지가 아니다)

수동 trigger 다 — 모든 변경에 자동 적용하지 않는다.

## Post-task housekeeping (상세)

[AGENTS.md](../AGENTS.md) §11 각 항목의 실행 기준이다. 항목 목록·요구 수준은 §11 이 SSoT 다.

1. **완료 보고** — 실행한 검증은 명령과 결과로 적고, 돌리지 않은 검증은 "통과" 가 아니라 "생략 + 이유" 로 적는다.
2. **Issue 동기화** — 구현된 것, 미충족 acceptance criteria, 후속 작업을 갱신한다. 갱신 없이 닫지 않는다. 전부 충족했으면 close, 아니면 남은 범위를 코멘트로 남기고 open 유지.
3. **Stale artifact · 캐시 정리** — 완료된 private plan 은 내용이 git log / issue 로 복원 가능하면 삭제한다 (복원 불가한 결정 기록은 issue 코멘트로 옮긴 뒤 삭제 — [handoff.md](handoff.md) §5). 임시 파일은 scratchpad 에 만들고 task 종료 시 삭제한다. repo 안에 잘못된 cwd 로 생긴 `build/` · `install/` · `log/` 와 python 캐시 (`__pycache__`, `.pytest_cache`, `.ruff_cache`, `.mypy_cache`) 는 확인 없이 삭제한다. **colcon 정규 트리 `<rtc_ws>/{build,install,log}` 는 절대 건드리지 않는다.**
4. **Branch prune** — `main` 에 merge 된 로컬 branch 만 삭제하고 `git fetch --prune` 한다. 현재 checkout 된 branch · 미merge branch · `main` 은 건드리지 않는다.
5. **도구별 memory · harness 정리** — 그 도구의 문서가 소유한다.
