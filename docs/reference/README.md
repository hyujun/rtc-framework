# docs/reference — 헌법 밖의 참고 자료

여기 있는 문서는 **헌법이 아니다**. 에이전트가 지켜야 할 규범은 [AGENTS.md](../../AGENTS.md) · [agent_docs/](../../agent_docs/) · `.claude/rules/` 가 갖고, 이 디렉토리는 그 규범에서 덜어낸 것 — 규칙의 근거 · 실측 · 사고 이력 · 사례집 · 기록 시점의 구현 위치 — 과 규범을 적용하는 작성 가이드를 보관한다. `docs/` 에서 에이전트를 위한 폴더는 여기 하나다 (사람이 따라가는 how-to 는 `docs/` 바로 아래).

- 규범과 여기 내용이 어긋나면 **규범이 옳다**. 여기 문장을 근거로 규칙을 우회하지 않는다.
- 여기에 규칙 (에이전트가 적용해야 하는 판정 기준) 을 새로 두지 않는다 — 규범 문서의 예산이 모자라도 그렇다. 예산은 `docs_budget.yaml` 에 사유를 적고 올린다. 여기서 규칙을 발견해 규범으로 올릴 때는 강도 ("해도 된다" · "한다" · "금지") 와 범위를 바꾸지 않는다.
- 수치·파일 위치·호출부는 기록 시점의 것이다. 인용하기 전에 다시 잰다.
- 크기 예산 (`repo_scripts/config/docs_budget.yaml`) 의 대상이 아니다. 링크·경로 검사 (`validate_docs.py`) 는 받는다.

| 문서 | 내용 | 대응하는 규범 |
|---|---|---|
| [invariants-rationale.md](invariants-rationale.md) | RT / ARCH / PROC / NUM 규칙의 이유 · 알려진 위반의 실측 · NUM-5 / NUM-7 실측 | [invariants.md](../../agent_docs/invariants.md) |
| [architecture-rationale.md](architecture-rationale.md) | layout v3 → v4 · efforts lane · mpc tenant · launch profile · E-STOP defer · 의존 엣지의 결정 이력 | [architecture.md](../../agent_docs/architecture.md) |
| [design-principles-rationale.md](design-principles-rationale.md) | 설계 원칙별 근거 · 판정이 내려진 사례와 실측 | [design-principles.md](../../agent_docs/design-principles.md) |
| [conventions-rationale.md](conventions-rationale.md) | 규약의 근거 · 위반 사례 (uncrustify 충돌, Tier 0 예외의 배경, closing keyword 사고, merge method 선례) | [conventions.md](../../agent_docs/conventions.md) |
| [rt-setup-rationale.md](rt-setup-rationale.md) | RT 호스트 설정의 근거 — 전 tier RT 코어 집합 (#380) · CM cpuset 범위 · cset adopt fail-closed · degraded 격리 예외 (#343) · isolcpus 폐기 | [invariants.md](../../agent_docs/invariants.md#rt-host--runtime-preconditions) (RT-HOST-1~3) |
| [tracing-rationale.md](tracing-rationale.md) | tracing 계측의 근거 — 예외 L3 · cadence 지표 · span 이 없거나 따로 놓인 자리의 의도 · 계측 제외 | [testing-debug.md](../../agent_docs/testing-debug.md#tracing) |
| [testing-rationale.md](testing-rationale.md) | 테스트 규범의 사례와 실측 — false green 3형 · fixtures CI · ROS_DOMAIN_ID 격리 · 계측 함정 표 · 할당 게이트 사건 · coverage 함정 | [testing-debug.md](../../agent_docs/testing-debug.md) |
| [controllers-rationale.md](controllers-rationale.md) | 컨트롤러 설계의 실측과 기각 대안 — 코어 추출 이력 · 바인딩 함정 · Compliance · Inference · GraspController | [controllers.md](../../agent_docs/controllers.md) |
| [readme-guide.md](readme-guide.md) | README 유형별 골격 · 목적지 표 · 샘플 · 체크리스트 · 외부 출처 | [conventions.md](../../agent_docs/conventions.md#readme) |
| [anti-patterns.md](anti-patterns.md) | 재발한 실수의 사례 · 증상 · 원인 · 복구 (AP ID 의 정의) | [invariants.md](../../agent_docs/invariants.md) — 위반한 invariant 행 또는 §Anti-pattern 규범 |

## 헌법 구조의 출처

[AGENTS.md](../../AGENTS.md) §2 의 조직은 **Agent = Model + Harness** 라는 프레이밍을 따른다 — 모델 바깥의 guides · sensors · orchestration · escalation · enforcement 가 모델 교체와 무관하게 누적되는 자산이다. 구조 결정과 그 출처:

- AGENTS.md 단일 헌법 + CLAUDE.md 의 `@AGENTS.md` import — "create a CLAUDE.md that imports it so both tools read the same instructions without duplicating them" ([memory docs](https://code.claude.com/docs/en/memory)). `@import` 는 launch 시 전부 로드되어 컨텍스트를 줄이지 않으므로 크기 예산은 import 되는 AGENTS.md 에도 걸린다.
- rule 의 `paths:` 필수 — "An unscoped rule is mechanically identical to putting the content in CLAUDE.md: always loaded, always costing tokens." ([Steering Claude Code](https://claude.com/blog/steering-claude-code-skills-hooks-rules-subagents-and-more))
- 헌법에서 근거 · 사고 서술을 뺀다 (AP-DOC-1 · AP-DOC-2) — 줄마다 "Would removing this cause Claude to make mistakes?" 를 묻고, "Long explanations or tutorials" · "Information that changes frequently" 를 제외한다 ([Best practices](https://code.claude.com/docs/en/best-practices)). 목표는 "the smallest possible set of high-signal tokens" ([Effective context engineering](https://www.anthropic.com/engineering/effective-context-engineering-for-ai-agents)).
- D12 예산이 줄 수 · 바이트 · 줄 길이를 함께 건다 — 공식 권고는 CLAUDE.md 200줄 이하 (길수록 adherence 저하, [memory docs](https://code.claude.com/docs/en/memory)) 지만 줄 수를 지킨 채 줄이 길어질 수 있다.
- 에이전트 자기 평가를 믿지 않는다 → inferential sensor 를 generation *전* 에 명시 (AGENTS.md §5.5 · AGENTS.md §6.5) — [Harness design for long-running application development](https://www.anthropic.com/engineering/harness-design-long-running-apps).
- 지시가 늘수록 준수는 공짜가 아니다 — 지시 500개 밀도에서 최상위 모델도 68% ([arXiv 2507.11538](https://arxiv.org/abs/2507.11538)). keyword-inclusion 과제라 수치를 헌법에 외삽하지 않고 방향 근거로만 쓴다.
