@AGENTS.md

## Claude Code

[AGENTS.md](AGENTS.md) 가 헌법 전문이며 위 import 로 매 세션 로드된다 (규칙은 거기에만 산다). 이 절은 Claude Code 에만 있는 메커니즘을 헌법 절에 매핑한다.

- **Enforcement (AGENTS.md §2 · AGENTS.md §4)** — `.claude/hooks/`: `format-code.sh` (PostToolUse: clang-format / ruff) · `verify-changes.sh` (Stop: AGENTS.md §4 "커밋 전에 직접 돌려야 하는 것" 을 변경 패키지에 자동 수행하고 hard failure 시 `exit 2` 로 차단. 변경 집합은 **마지막 통과 커밋(watermark) 기준 `git diff` ∪ untracked** 이므로 turn 안에서 commit 한 변경도 검증된다. timeout·launch 실패는 "미검증" 으로 차단. 검사 범위·blocking 구분·allowlist·bound 의 SSoT 는 hook 헤더 주석) · `log-instructions-loaded.sh` (InstructionsLoaded: 로드 기록만)
- **Rules (AGENTS.md §3)** — path-scoped rule `.claude/rules/rt-path.md` (RT-1~10 요약·예외·대안) · `arch-source.md` (ARCH-1·2·3·4·6) · `arch-build-meta.md` (ARCH-2·5·7) 이 매칭 파일 편집 시 자동 로드된다 (로드 조건은 frontmatter glob 이 SSoT — 박제 금지). **rule 채널은 두 센서로 검증한다**: glob 매칭은 `validate_claude_rules.py` (변경된 rule 한정, 0건이면 차단), 실제 로드는 `.claude/instructions-loaded.log`. rule 을 새로 쓰거나 glob 을 고쳤으면 매칭 파일을 하나 열고 로그를 자기 `session_id` 로 걸러 발화를 확인한다 (읽는 법은 그 hook 헤더). rule 이 안 뜨는 실패는 증상이 "규칙이 조용히 없는 것" 뿐이다
- **Skills (AGENTS.md §4)** — 추가 작업은 `adding-component` skill 이 진입점 (P5 일반화 · ARCH-3 · spec 필요 여부 판정과 발화 게이트만; 절차 SSoT 는 modification-guide.md). 런타임 검증 레시피는 `verify` skill
- **Inferential review (AGENTS.md §5.5)** — `/code-review` (rtc_base/rtc_msgs · interface 신설/두 번째 구현 · robot-specific 의심 · 100+ 줄/신규 패키지) · `/security-review` (E-STOP/safety/lifecycle 콜백) · `/code-review ultra` (PR 준비, 현재 branch 또는 `<PR#>`; `/ultrareview` 는 deprecated alias) · `/simplify` (리팩터·중복 의심·정리 — 버그 탐지는 `/code-review`)
- **Spec·plan 저장 (AGENTS.md §6.5 · AGENTS.md §6.6)** — Sprint Contract spec 과 handoff 는 `~/.claude/plans/<task-slug>.md` (`## Spec` / `## Progress` / `## Handoff`) 에서 스스로 관리한다. cross-tool 인계는 git issue
- **Context handoff 메커니즘 (AGENTS.md §6.6)** — 능동 제안 트리거, `/rename`+`/clear` / `/compact <focus>` / subagent / `/btw` / fork 선택, 보존 우선순위는 user-level CLAUDE.md `# Context handoff policy` 가 SSoT. "동일 문제 2회 초과 교정 → `/clear`" (context 위생) 는 handoff.md 의 "3회 → 중단·escalate" 와 다른 축이며 공존한다
- **Housekeeping (AGENTS.md §11)** — memory save / prune / Harness pruning 신호 보고는 user-level CLAUDE.md `# Post-task housekeeping` 가 SSoT (RTC 신호 카테고리: grep false-positive, hook 오차단, agent_docs 간 규칙 중복 drift). 완료된 `~/.claude/plans/*.md` 는 복원 가능하면 삭제
