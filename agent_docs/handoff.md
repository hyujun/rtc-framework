# Context Handoff Contract (tool-neutral)

이 문서는 RTC Framework 에서 **에이전트 간·세션 간 context handoff 의 tool-neutral 단일 출처(SSoT)** 다. Claude·Codex 등 어떤 에이전트든 이 계약을 따른다. 이 문서는 **무엇이 handoff 이고, artifact 가 무엇을 담아야 하며, 받는 쪽이 어떻게 재개하는가** 만 정의한다 — Claude 전용 메커니즘(`/compact`·`/clear`·`# Compact instructions`·`/btw`·fork)과 능동 제안 트리거는 [../CLAUDE.md](../CLAUDE.md) §6.6 및 user-level 설정이 소유한다.

**원칙: handoff 는 받는 에이전트가 이전 transcript 를 읽지 않고도 재개할 수 있을 때에만 완료된 것이다.**

## 1. Trigger 분류 (action 분리)

네 가지 서로 다른 상황을 혼동하지 않는다 — 각각의 필요 action 이 다르다.

| 상황 | 필요한 action |
|---|---|
| 미완료 task 가 session · agent · model · 책임(ownership) 경계를 넘음 | **git issue 에 handoff artifact 생성** (§2 template, 저장은 §5) |
| 같은 task · ownership 유지, context 압박만 | **compact** (Claude 메커니즘은 [CLAUDE.md](../CLAUDE.md) §6.6). durable 결정·evidence 손실 위험이 있을 때만 artifact 생성/갱신 |
| 대용량 grep · log · test 출력 · research | **subagent 로 위임/격리**. 그 자체로는 handoff 아님 |
| 반복 실패 (3회 시도) | **재시도 중단** → evidence 기록 후 diagnose / escalate. 새 owner/session 이 필요할 때만 handoff |
| 완료된 task 다음에 무관한 task | **새 task 시작**. artifact 불필요 |

주: "반복 실패 3회 → 중단·진단" 은 tool-neutral 엔지니어링 규율이다. Claude 세션의 "동일 문제 2회 초과 교정 → `/clear`"(context 위생, [../CLAUDE.md](../CLAUDE.md) §6.6)와는 **다른 축**이며 공존한다 — 전자는 escalation, 후자는 Claude context 리셋 메커니즘.

## 2. Artifact template (independently resumable)

받는 쪽이 transcript 없이 재개할 수 있어야 완료다. 다음 섹션을 채운다 (해당 없으면 `N/A` 명시).

```md
# <task title>

## Goal
## Acceptance criteria
## Out of scope
## Current state
## Next action
## Decisions and rationale
## Evidence            # 실행한 명령 + 간결한 build/test/review 결과
## Failed approaches
## Constraints / pending human decisions
## Workspace           # branch, HEAD, git status, 미커밋 변경의 ownership
## Pointers            # 관련 파일·심볼·issue/PR 링크 8–15개
```

**금지:** credentials · secret · raw 대용량 log · 미검증 주장(unverified claim) 을 넣지 않는다.

## 3. Sender checklist

- [ ] 위 필수 섹션 전부 채움 (해당 없으면 `N/A`)
- [ ] Evidence 는 실제 실행 결과만 — `done` 을 쓰기 전에 build/test 로 검증한다 (self-eval 은 신뢰 불가; [../CLAUDE.md](../CLAUDE.md) §5.5)
- [ ] Workspace: `git status` · 현재 branch · HEAD · 미커밋 변경의 소유자 기록
- [ ] Failed approaches 에 dead-end 요약 → 받는 쪽이 같은 실패를 재시도하지 않도록
- [ ] secret · raw log 미포함 확인
- [ ] 상대 시간 표현 금지 — 절대 날짜로 (stale 방지)

## 4. Receiver / reconciliation checklist

- [ ] transcript 가 아니라 **artifact 를 읽고** 시작한다
- [ ] `git status` · HEAD · 핵심 evidence(build/test)가 artifact 와 **일치하는지 검증**한다
- [ ] 불일치 시 **구현 전에 artifact 를 먼저 갱신**한다 (현실을 SSoT 에 반영)
- [ ] Acceptance criteria · Out of scope 확인 후 Next action 부터 진행한다
- [ ] 참조하는 issue 는 **본문보다 코멘트가 최신**이다 — 종결 이슈일수록 그렇고 (#339 는 본문 자체가 틀렸다), 본문의 진단을 근거로 쓰기 전에 코멘트 전체를 읽는다

## 5. Storage / retention (purpose 별)

repo 는 plan 파일을 커밋하지 않는다 — 각 에이전트가 **자기 private 저장소**에서 스스로 보관·관리하고, durable 기록은 git 이 담당한다.

- **진행 중 plan** → 각 에이전트의 private 저장소. Claude → `~/.claude/plans/<slug>.md` ([../CLAUDE.md](../CLAUDE.md) §6.5 Sprint Contract spec 과 동일 파일에 누적), Codex → 자기 home. repo 에 커밋하지 않는다.
- **Cross-tool 인계** (미완료 task 가 Claude↔Codex 등 tool 경계를 넘음) → **git issue** 본문/코멘트에 §2 artifact 를 적어 공유한다. private plan 은 다른 tool 이 읽을 수 없으므로, 경계를 넘는 순간의 유일한 공유 매체는 issue 다.
- **완료 작업의 durable 기록** → git log(commit · merge) + git issue + memory. 별도 커밋 artifact 를 두지 않는다.
- **정리(retention)** → 완료된 private plan 은 그 내용이 git log / issue / memory 로 복원 가능하거나 보존할 가치가 없으면 **삭제**한다 ([../CLAUDE.md](../CLAUDE.md) §11). 복원 불가한데 보존 가치가 있는 결정 기록이 남아 있으면 issue 코멘트로 옮긴 뒤 삭제한다.

## Out of scope

이 계약은 RT invariant · build/test gate · hook · runtime 동작을 바꾸지 않으며, 모든 task 에 자동으로 artifact 를 만들지 않는다.
