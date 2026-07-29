# Copilot instructions — RTC Framework

이 저장소의 에이전트 헌법은 저장소 루트의 [`AGENTS.md`](../AGENTS.md) 다. **작업 시작 전 그 파일을 읽는다.**

`AGENTS.md` 는 tool-neutral 이며 다음을 담는다.

- **Invariants** — RT path 금지 규칙(RT-1~RT-10), architecture/process/numerical 규칙(ARCH-·PROC-·NUM-)
- **Workflow** — Type → Locate → Read → Edit → Build → Test → Verify, 그리고 커밋 전에 직접 돌려야 하는 검증(포매팅·빌드·테스트·문서 검증)
- **Escalation** — 코드를 쓰기 전에 멈추고 사람에게 물어야 하는 상황(E-1~E-11)과 `[CONCERN]` 보고 포맷
- **Build hard rules** — `colcon` 은 반드시 workspace root 에서, `.venv` 격리는 우회하지 않는다
- **Style** — namespace `rtc`, Google C++ naming, SI 단위, Hamilton quaternion

세부 규칙의 단일 출처는 [`agent_docs/`](../agent_docs/) 이며 `AGENTS.md` §10 이 각 문서의 역할을 안내한다.

`CLAUDE.md` 는 Claude Code 전용 메커니즘(hook·slash command·rule 자동 로드)을 담으므로 이 도구에는 적용되지 않는다 — 같은 규칙의 tool-neutral 판이 `AGENTS.md` 에 있다.
