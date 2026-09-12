# Harness Rationale (근거·출처)

CLAUDE.md §2 "Harness Overview" 의 5구성요소 조직(guides / sensors / orchestration / escalation / enforcement)이 따르는 **agent-driven engineering** 프레이밍의 근거 자료. 모델 행동에 직접 영향을 주지 않는 배경 문헌이므로 CLAUDE.md 본문이 아닌 이곳에 둔다 ([anti-patterns.md](anti-patterns.md) AP-DOC-1, user-level `# CLAUDE.md hygiene`).

핵심 명제: **Agent = Model + Harness.** 모델 바깥의 guides·sensors·orchestration·escalation·enforcement 가 본 저장소의 1급 자산이며, 모델 교체와 무관하게 누적·개선된다.

## 출처

- [Fowler — *Harness engineering for coding agent users* (2026.04)](https://martinfowler.com/articles/harness-engineering.html)
- [Anthropic — *Harness design for long-running application development* (2026.04)](https://www.anthropic.com/engineering/harness-design-long-running-apps) — 에이전트 자기 평가는 신뢰 불가 → inferential sensor 를 generation *전* 명시 (CLAUDE.md §5.5 · CLAUDE.md §6.5 근거)
- [Osmani — *Agent Harness Engineering* (2026)](https://addyosmani.com/blog/agent-harness-engineering/)
- [Anthropic — *2026 Agentic Coding Trends Report*](https://resources.anthropic.com/2026-agentic-coding-trends-report)

## CLAUDE.md memory 운영 근거 (Anthropic 공식)

- [Claude Code — *How Claude remembers your project* (memory docs)](https://code.claude.com/docs/en/memory) — CLAUDE.md 파일당 200줄 이하 권장(길수록 adherence 저하), path-scoped `.claude/rules/` 로 조건부 로드, `@import` 는 launch 시 전부 로드되어 context 절감 효과 없음, CLAUDE.md 는 enforcement 가 아니라 context(강제는 hook), MEMORY.md 는 첫 200줄/25KB 만 로드.

## Harness Overview — 5구성요소 (CLAUDE.md §2 의 근거)

이 저장소의 에이전트 가이드는 **agent-driven engineering** (harness engineering + spec-driven development + Anthropic 2026 agentic SDLC patterns) 의 5구성요소로 조직되어 있다. 모델 바깥의 guides / sensors / orchestration / escalation / enforcement 가 1급 자산이다.

| 구성요소 | 목적 | 진입점 |
|---|---|---|
| **Guides** (feedforward) | 규칙·원칙·패턴 | CLAUDE.md §3 Invariants, CLAUDE.md §10 Style, [invariants.md](invariants.md), [design-principles.md](design-principles.md), [conventions.md](conventions.md) |
| **Sensors** (feedback, computational) | 변경 검증 (결정적·빠른) | CLAUDE.md §5, [testing-debug.md](testing-debug.md), `[CONCERN]` 포맷 (CLAUDE.md §6) |
| **Sensors** (feedback, inferential) | 의미 검증 (LLM-as-judge, on-demand) | CLAUDE.md §5.5, [modification-guide.md](modification-guide.md) §Inferential review 트리거 |
| **Orchestration** | Workflow | CLAUDE.md §4, [modification-guide.md](modification-guide.md) |
| **Escalation** | Human gate | CLAUDE.md §6, CLAUDE.md §6.5 Sprint Contract |
| **Enforcement** (자동) | 무인 실행/차단 | `.claude/hooks/format-code.sh` (PostToolUse: clang-format / ruff), `.claude/hooks/verify-changes.sh` (Stop: doc·CMake·build·test gate, exit 2 차단), `.claude/hooks/log-instructions-loaded.sh` (InstructionsLoaded: 어떤 CLAUDE.md·rule 이 왜 로드됐는지 `.claude/instructions-loaded.log` 에 기록 — 차단 없음) |
