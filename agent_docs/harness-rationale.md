# Harness Rationale (근거·출처)

AGENTS.md §2 "Harness Overview" 의 5구성요소 조직(guides / sensors / orchestration / escalation / enforcement)이 따르는 **agent-driven engineering** 프레이밍과, 헌법의 구조·크기 규율의 근거 자료. 모델 행동에 직접 영향을 주지 않는 배경 문헌이므로 헌법 본문이 아닌 이곳에 둔다 ([anti-patterns.md](anti-patterns.md) AP-DOC-1, user-level `# CLAUDE.md hygiene`).

핵심 명제: **Agent = Model + Harness.** 모델 바깥의 guides·sensors·orchestration·escalation·enforcement 가 본 저장소의 1급 자산이며, 모델 교체와 무관하게 누적·개선된다.

## 출처

- [Fowler — *Harness engineering for coding agent users* (2026.04)](https://martinfowler.com/articles/harness-engineering.html)
- [Anthropic — *Harness design for long-running application development* (2026.04)](https://www.anthropic.com/engineering/harness-design-long-running-apps) — 에이전트 자기 평가는 신뢰 불가 → inferential sensor 를 generation *전* 명시 (AGENTS.md §5.5 · AGENTS.md §6.5 근거)
- [Osmani — *Agent Harness Engineering* (2026)](https://addyosmani.com/blog/agent-harness-engineering/)
- [Anthropic — *2026 Agentic Coding Trends Report*](https://resources.anthropic.com/2026-agentic-coding-trends-report)

## 헌법·memory 운영 근거 (Anthropic 공식, 2026-09 확인)

- [Claude Code — *How Claude remembers your project* (memory docs)](https://code.claude.com/docs/en/memory) — CLAUDE.md 파일당 200줄 이하 권장(길수록 adherence 저하), path-scoped `.claude/rules/` 로 조건부 로드, `@import` 는 launch 시 전부 로드되어 context 절감 효과 없음, CLAUDE.md 는 enforcement 가 아니라 context(강제는 hook), MEMORY.md 는 첫 200줄/25KB 만 로드. AGENTS.md 를 쓰는 저장소는 "create a CLAUDE.md that imports it so both tools read the same instructions without duplicating them" — **AGENTS.md 단일 헌법 + CLAUDE.md `@AGENTS.md` 구조의 근거**. block-level HTML 주석은 주입 전에 제거된다
- [Claude Code — *Best practices*](https://code.claude.com/docs/en/best-practices) — 각 줄에 "Would removing this cause Claude to make mistakes?" 를 묻고 아니면 삭제. 제외 목록에 "Long explanations or tutorials" · "Information that changes frequently" 가 있어 **헌법에서 사고 서술·근거를 뺀 근거** (AP-DOC-1 · AP-DOC-2 와 같은 방향). 실패 패턴 "The over-specified CLAUDE.md". Stop hook 은 8회 연속 차단 후 override 된다 (verify-changes.sh 헤더·modification-guide.md 가 인용)
- [Claude — *Steering Claude Code* (2026-06-18)](https://claude.com/blog/steering-claude-code-skills-hooks-rules-subagents-and-more) — "An unscoped rule is mechanically identical to putting the content in CLAUDE.md: always loaded, always costing tokens." → 새 rule 은 반드시 `paths:` 를 단다. 절차는 skill 로, 반드시 일어나야 하는 동작은 hook 으로
- [Claude Code — *Skills*](https://code.claude.com/docs/en/skills) — "Keep `SKILL.md` under 500 lines"; skill 본문은 사용될 때만 로드된다 (헌법에서 절차를 빼는 목적지)
- [Claude Code — *Extend Claude Code*](https://code.claude.com/docs/en/features-overview) — CLAUDE.md · rules · skills · subagents · hooks 중 어느 메커니즘에 둘지의 선택 기준
- [Claude Code — *Monorepos and large repos*](https://code.claude.com/docs/en/large-codebases) — root·디렉토리별 CLAUDE.md 와 rule 의 배치 (패키지별 nested 파일을 검토할 때의 출발점)
- [Claude Code — *Hooks reference*](https://code.claude.com/docs/en/hooks) · [*Subagents*](https://code.claude.com/docs/en/sub-agents) — hook 이벤트·exit code 의미 (InstructionsLoaded 포함), subagent frontmatter (`memory` 필드 등)
- [Anthropic — *Effective context engineering for AI agents* (2025-09-29)](https://www.anthropic.com/engineering/effective-context-engineering-for-ai-agents) — 목표는 "the smallest possible set of high-signal tokens that maximize the likelihood of some desired outcome"; 컨텍스트가 길수록 recall 이 떨어지는 context rot

## AGENTS.md 표준·실증 연구

- [agents.md](https://agents.md/) — "stewarded by the Agentic AI Foundation under the Linux Foundation". 중첩 파일은 "the closest one takes precedence", "explicit user chat prompts override everything"
- [Lulla et al. — *On the Impact of AGENTS.md Files on the Efficiency of AI Coding Agents* (arXiv 2601.20404, 2026-01)](https://arxiv.org/abs/2601.20404) — AGENTS.md 존재 시 median runtime −28.64%, output token −16.58%. 효율 지표이며 규칙 준수율을 잰 것은 아니다
- [*How Many Instructions Can LLMs Follow at Once?* (arXiv 2507.11538, 2025-07)](https://arxiv.org/abs/2507.11538) — 지시 500개 밀도에서 최상위 frontier 모델도 68% 정확도. keyword-inclusion 과제라 헌법 규칙에 수치를 직접 외삽하지 않는다 — 지시가 늘수록 준수가 공짜가 아니라는 방향 근거로만 쓴다

## 헌법 크기 예산 (validate_docs D12)

공식 권고는 줄 수(200)지만, 줄 수를 지킨 채 줄이 길어지는 방식으로 부피가 늘 수 있어 D12 는 줄 수·바이트·줄 길이를 함께 건다. `@import` 는 컨텍스트를 줄이지 않으므로 예산은 import 되는 AGENTS.md 에도 똑같이 걸린다. 한도 수치와 채택 당시 실측의 SSoT 는 `repo_scripts/scripts/validate_docs.py` 의 D12 docstring·상수다.

## Harness Overview — 5구성요소 (AGENTS.md §2 의 근거)

이 저장소의 에이전트 가이드는 **agent-driven engineering** (harness engineering + spec-driven development + Anthropic 2026 agentic SDLC patterns) 의 5구성요소로 조직되어 있다. 모델 바깥의 guides / sensors / orchestration / escalation / enforcement 가 1급 자산이다.

| 구성요소 | 목적 | 진입점 |
|---|---|---|
| **Guides** (feedforward) | 규칙·원칙·패턴 | AGENTS.md §3 Invariants, AGENTS.md §10 Style, [invariants.md](invariants.md), [design-principles.md](design-principles.md), [conventions.md](conventions.md) |
| **Sensors** (feedback, computational) | 변경 검증 (결정적·빠른) | AGENTS.md §5, [testing-debug.md](testing-debug.md), `[CONCERN]` 포맷 (AGENTS.md §6) |
| **Sensors** (feedback, inferential) | 의미 검증 (LLM-as-judge, on-demand) | AGENTS.md §5.5, [modification-guide.md](modification-guide.md) §Inferential review 트리거 |
| **Orchestration** | Workflow | AGENTS.md §4, [modification-guide.md](modification-guide.md) |
| **Escalation** | Human gate | AGENTS.md §6, AGENTS.md §6.5 Sprint Contract |
| **Enforcement** (자동) | 무인 실행/차단 | `.claude/hooks/format-code.sh` (PostToolUse: clang-format / ruff), `.claude/hooks/verify-changes.sh` (Stop: doc·CMake·build·test gate, exit 2 차단), `.claude/hooks/log-instructions-loaded.sh` (InstructionsLoaded: 어떤 CLAUDE.md·rule 이 왜 로드됐는지 `.claude/instructions-loaded.log` 에 기록 — 차단 없음) |
