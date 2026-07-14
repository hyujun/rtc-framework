# Execution plans (durable, cross-tool handoff artifacts)

Multi-session · multi-agent · review 관련 작업의 **durable handoff artifact** 를 두는 곳이다. 어떤 에이전트(Claude · Codex 등)든 이전 transcript 없이 재개할 수 있도록, [agent_docs/handoff.md](../../agent_docs/handoff.md) 의 계약(trigger 분류 · artifact template · sender/receiver checklist)을 따른다.

- `active/<slug>.md` — 진행 중인 작업. 이 repo artifact 가 **cross-tool SSoT** 다. Claude 전용 transient plan(`~/.claude/plans/<slug>.md`)이 함께 있으면 그쪽은 링크만 하고 diverge 시키지 않는다.
- `completed/<slug>.md` — 완료 후 **결정 기록(decision record)이 보존 가치가 있을 때만** 이동. 아니면 삭제 ([CLAUDE.md](../../CLAUDE.md) §11).

credentials · secret · raw 대용량 log · 미검증 주장은 넣지 않는다.
