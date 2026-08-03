---
name: adding-component
description: Pre-flight and gate map for ADDING a new controller, message type / PublishRole, device group or backend, thread, or colcon package to this repo (as opposed to modifying existing code). Use before writing the first line of an addition — it routes to the SSoT procedure and names the escalation that fires.
---

# 새 구성요소를 추가할 때

이 skill 은 절차를 **복제하지 않는다**. 단계별 절차의 SSoT 는 [agent_docs/modification-guide.md](../../../agent_docs/modification-guide.md) 의 "Adding a New ..." 절이며, 그 문서는 tool-neutral 이라 Claude 밖 도구도 읽는다. 여기 있는 것은 **그 문서를 열기 전에 결정해야 하는 것**과 **이 추가에서 자동으로 발화하는 게이트**다 — 사본을 만들면 #213 이 고친 drift 가 되돌아온다.

## 0. 먼저: 정말 추가인가

[design-principles.md](../../../agent_docs/design-principles.md) P5 — 새 유틸리티를 쓰기 전에 기존 `rtc_*` 에 유사 기능이 있는지 찾는다. 맞지 않으면 fork 하지 말고 **일반화**한다. integration 패키지에 추가하려는 것이라면 재사용 가능한 부분을 `rtc_*` 로 끌어올릴 수 있는지 먼저 검토한다.

**ARCH-3 판정**: 같은 역할의 구현이 이미 하나 있으면 두 번째를 쓰기 전에 abstract interface / concept 를 정의한다. `#ifdef` 나 하드코딩 switch 로 분기하려는 충동이 그 신호다. 단일 backend 에 **설정 키만** 더하는 것은 두 번째 구현이 아니다 — 그 경우 backend 를 늘리지 말고 키를 검토한다.

## 1. Sprint Contract = spec (거의 항상 해당)

새 abstract interface · controller · 메시지 · 디바이스 · 스레드 추가는 [CLAUDE.md](../../../CLAUDE.md) §6.5 에서 **spec 이 필수**인 부류다. 코드 전에 `~/.claude/plans/<slug>.md` 에 *왜 필요한가 · API surface · 검토한 alternatives* 를 박고 `[SPRINT]` 로 컨펌받는다. 포맷은 modification-guide.md §Sprint Contract & Spec.

`~/.claude/plans/` 를 **먼저 `ls`** 한다 — 같은 작업의 plan 이 이미 있을 수 있고, 그것이 보이지 않는 SSoT 다.

## 2. 무엇을 추가하는가 → SSoT 절과 그때 발화하는 게이트

| 추가 대상 | 절차 SSoT (modification-guide.md) | 착수 전 걸리는 것 |
|---|---|---|
| Controller | §Adding a New Controller | 코어/바인딩 2계층 분리가 전제. `Name()` 과 `config_key` 는 **한 네임스페이스**라 전역 유일 — 클래스를 복사하고 문자열을 안 고치면 bring-up 전체가 거부된다 |
| Message / `PublishRole` | §Adding a New Message Type | **기본 답은 "추가하지 않는 것"** (controller-owned SeqLock + `owned_topics` 헬퍼). 그래도 필요하면 **E-11 → 착수 전 `[CONCERN]`**, 그리고 네 곳을 같은 변경 안에서 고친다 |
| Device group / backend | §Adding a New Device Group | 기존 backend 에 **설정 키만** 필요한 것은 아닌지 먼저 판정 (§0 ARCH-3). rename 이면 §Renaming a Device Group 의 grep 목록 전부 — 조용한 dead topic 이 실제로 2회 재발했다 |
| Thread | §Adding a New Thread | **E-7 (Critical)** — thread model 변경이므로 착수 전 `[CONCERN]` + `[SPRINT]`. `rtc_base` 변경이면 PROC-3 전체 빌드 |
| Package (새 colcon dir) | §Adding a New Package | **두 build SSoT 를 모두** 갱신 (`rt_common.sh` 셀렉터 + `.github/ci-packages.yml`). 누락 시 실패 양상이 다르고, CI 쪽 누락은 **테스트가 한 번도 안 돌면서 green** 이다 |

## 3. 추가에서 특히 잘 놓치는 것

- **`rtc_*` 에 추가**한다면 robot 이름·joint 수·HW ID 가 들어가면 ARCH-1 (**Critical**, E-2) 이고, exec 를 소유하면 ARCH-7 이다. 두 rule 은 파일을 열 때 자동 로드된다 ([.claude/rules/arch-source.md](../../rules/arch-source.md) · [arch-build-meta.md](../../rules/arch-build-meta.md)).
- **RT tick 경로**에 코드가 생기면 [.claude/rules/rt-path.md](../../rules/rt-path.md) 가 구속한다 — 새 스레드·새 `Compute()` 는 거의 항상 여기 해당한다.
- **PROC-1 은 blocking 이다** — 새 파일을 add 했으면 같은 turn 에 `CMakeLists.txt` / `package.xml` 을 맞춘다. README 는 public surface 가 바뀌었을 때 non-blocking checklist.
- **새 test 파일·새 launch 파일도 build 축에 들어간다** (#358 · #360) — 즉 추가한 turn 이 곧 검증되는 turn 이다. 반대로 그 축 밖(`config/` 등)에 놓인 신규 파일은 빌드되지 않는다.

## 4. 끝났다고 말하기 전

modification-guide.md §Completion Checklist 는 Stop hook 이 검사하는 범위의 **여집합**이다 — hook 이 green 이어도 그 4항목(`package.xml` dep 의미, YAML default·범위·unit, Doxygen, RT 자가검사)은 직접 확인한다. 센서 선택은 [agent_docs/testing-debug.md](../../../agent_docs/testing-debug.md) 가 SSoT.
