# docs/reference — 헌법 밖의 참고 자료

여기 있는 문서는 **헌법이 아니다**. 에이전트가 지켜야 할 규범은 [AGENTS.md](../../AGENTS.md) · [agent_docs/](../../agent_docs/) · `.claude/rules/` 가 갖고, 이 디렉토리는 그 규범에서 덜어낸 것 — 규칙의 근거 · 실측 · 사고 이력 · 사례집 · 기록 시점의 구현 위치 — 을 보관한다.

- 규범과 여기 내용이 어긋나면 **규범이 옳다**. 여기 문장을 근거로 규칙을 우회하지 않는다.
- 수치·파일 위치·호출부는 기록 시점의 것이다. 인용하기 전에 다시 잰다.
- 크기 예산 (`repo_scripts/config/docs_budget.yaml`) 의 대상이 아니다. 링크·경로 검사 (`validate_docs.py`) 는 받는다.

| 문서 | 내용 | 대응하는 규범 |
|---|---|---|
| [invariants-rationale.md](invariants-rationale.md) | RT / ARCH / PROC / NUM 규칙의 이유 · 알려진 위반의 실측 · 구현 위치 | [invariants.md](../../agent_docs/invariants.md) |
