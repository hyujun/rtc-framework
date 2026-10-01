@AGENTS.md

## Claude Code

[AGENTS.md](AGENTS.md) 가 헌법 전문이며 위 import 로 매 세션 로드된다 (규칙은 거기에만 산다). 이 절은 Claude Code 에만 있는 메커니즘을 헌법 절에 매핑하고, hook 의 동작은 서술하지 않는다 — 범위·예외·상태의 SSoT 는 각 hook 헤더 주석이고 여기에는 **해야 할 행동**만 둔다.

- **Enforcement (AGENTS.md §2 · AGENTS.md §4)** — `.claude/hooks/`
  - `format-code.sh` (PostToolUse) 는 Edit/Write 한 파일만 포맷한다 — Bash 로 쓴 파일은 직접 포맷한다
  - `verify-changes.sh` (Stop) 는 AGENTS.md §4 "커밋 전에 직접 돌려야 하는 것" 중 **기계 판정 가능한 부분만** 확인하고 실패하면 turn 을 차단한다 — 여집합은 modification-guide.md §Completion Checklist 로 직접 확인한다
    - turn 끝은 빌드·테스트를 **실행하지 않고** 변경 패키지의 green verdict 만 확인한다. verdict 는 turn 안에서 `.claude/hooks/verify-changes.sh --run` 으로 만든다 — **최종 검증은 `colcon test` 를 따로 돌리지 말고 이것으로** 하고, 그 뒤에 패키지를 고쳤으면 다시 돌린다
    - `--run` 이 오래 걸리면 백그라운드로 띄우고 **끝난 뒤** turn 을 끝낸다. 다른 빌드나 이 workspace 의 sim 이 돌고 있으면 `--run` 은 빌드하지 않는다 — 그 메시지를 따른다
    - unit 마다 sim 을 새로 띄우는 평가의 드라이버는 `repo_scripts/scripts/with_verify_hold.sh <드라이버>` 로 띄운다 (unit 사이의 틈에서 빌드가 시작되지 않게)
- **Rules (AGENTS.md §3)** — `.claude/rules/` 의 `rt-path.md` (RT) · `arch-source.md` (ARCH-1·2·3·4·6) · `arch-build-meta.md` (ARCH-2·5·7) 이 매칭 파일을 읽거나 편집할 때 로드된다 (로드 조건은 frontmatter glob 이 SSoT). rule 을 새로 쓰거나 glob 을 고쳤으면 `validate_claude_rules.py` 를 돌리고, **다음 세션에서** 매칭 파일을 열어 `.claude/instructions-loaded.log` 로 발화를 확인한다 (읽는 법: `log-instructions-loaded.sh` 헤더) — rule 이 안 뜨는 실패는 증상이 "규칙이 조용히 없는 것" 뿐이다
- **Skills (AGENTS.md §4)** — 추가 작업은 `adding-component` skill 이 진입점 (절차 SSoT 는 modification-guide.md), 런타임 검증 레시피는 `verify` skill
- **Inferential review (AGENTS.md §5.5)** — code review → `/code-review` (PR 준비는 `/code-review ultra`) · security review → `/security-review` · 재사용·단순화 review → `/simplify`. 어떤 변경이 어느 review 인지는 AGENTS.md §5.5 가 갖는다
- **Spec·plan 저장 (AGENTS.md §6.5 · AGENTS.md §6.6)** — Sprint Contract spec 과 handoff 는 `~/.claude/plans/<task-slug>.md` (`## Spec` / `## Progress` / `## Handoff`) 에서 스스로 관리한다. cross-tool 인계는 git issue
- **Context handoff 메커니즘 (AGENTS.md §6.6)** — 능동 제안 트리거와 `/clear` · `/compact` · subagent 선택은 user-level CLAUDE.md `# Context handoff policy` 가 SSoT. 그쪽의 "2회 초과 교정 → `/clear`" (context 위생) 와 handoff.md 의 "3회 → 중단·escalate" 는 다른 축이며 공존한다
- **Housekeeping (AGENTS.md §11)** — memory save / prune / Harness pruning 신호 보고는 user-level CLAUDE.md `# Post-task housekeeping` 가 SSoT (RTC 신호 카테고리: grep false-positive, hook 오차단, agent_docs 간 규칙 중복 drift). 완료된 `~/.claude/plans/*.md` 는 복원 가능하면 삭제
