# Harness Rationale (근거·출처)

CLAUDE.md §2 "Harness Overview" 의 5구성요소 조직(guides / sensors / orchestration / escalation / enforcement)이 따르는 **agent-driven engineering** 프레이밍의 근거 자료. 모델 행동에 직접 영향을 주지 않는 배경 문헌이므로 CLAUDE.md 본문이 아닌 이곳에 둔다 ([anti-patterns.md](anti-patterns.md) AP-DOC-1, user-level `# CLAUDE.md hygiene`).

핵심 명제: **Agent = Model + Harness.** 모델 바깥의 guides·sensors·orchestration·escalation·enforcement 가 본 저장소의 1급 자산이며, 모델 교체와 무관하게 누적·개선된다.

## 출처

- [Fowler — *Harness engineering for coding agent users* (2026.04)](https://martinfowler.com/articles/harness-engineering.html)
- [Anthropic — *Harness design for long-running application development* (2026.04)](https://www.anthropic.com/engineering/harness-design-long-running-apps) — 에이전트 자기 평가는 신뢰 불가 → inferential sensor 를 generation *전* 명시 (CLAUDE.md §5.5·§6.5 근거)
- [Osmani — *Agent Harness Engineering* (2026)](https://addyosmani.com/blog/agent-harness-engineering/)
- [Anthropic — *2026 Agentic Coding Trends Report*](https://resources.anthropic.com/2026-agentic-coding-trends-report)

## CLAUDE.md memory 운영 근거 (Anthropic 공식)

- [Claude Code — *How Claude remembers your project* (memory docs)](https://code.claude.com/docs/en/memory) — CLAUDE.md 파일당 200줄 이하 권장(길수록 adherence 저하), path-scoped `.claude/rules/` 로 조건부 로드, `@import` 는 launch 시 전부 로드되어 context 절감 효과 없음, CLAUDE.md 는 enforcement 가 아니라 context(강제는 hook), MEMORY.md 는 첫 200줄/25KB 만 로드.
