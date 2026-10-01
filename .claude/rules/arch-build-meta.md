---
paths:
  - "**/CMakeLists.txt"
  - "**/package.xml"
  - "**/setup.py"
  - "*/CMakeLists.txt"
  - "*/package.xml"
  - "*/setup.py"
---

# Architecture invariants — build metadata 편집 시 (ARCH-2 · ARCH-5 · ARCH-7)

`CMakeLists.txt` · `package.xml` · `setup.py` 를 읽거나 편집할 때 로드되는 reminder 이며, 이 파일이 갖는 것은 **판정**이다. 규칙 전문·복구는 [invariants.md](../../agent_docs/invariants.md) §Architecture Invariants, 탐지 패턴은 [.claude/hooks/verify-changes.sh](../hooks/verify-changes.sh) (ARCH-5 · ARCH-7 은 Phase 0a) 가 SSoT 이고, 소스 축 (ARCH-1 · ARCH-3 · ARCH-4 · ARCH-6) 은 [arch-source.md](arch-source.md) 가 갖는다. frontmatter 의 glob 형태는 줄이지 않는다 — 이유와 확인 절차는 `repo_scripts/scripts/validate_claude_rules.py` 헤더.

| # | 규칙 | Severity |
|---|---|---|
| ARCH-2 | 의존성 그래프 상향 의존 금지 (`rtc_base` → `rtc_controllers`, `rtc_*` → integration 패키지 등) | **Critical** (E-1) |
| ARCH-5 | `robot_descriptions` 는 data-only — build-time 의존 금지 | Warning (E-10) |
| ARCH-7 | `rtc_*` 는 RT 제어 루프를 구동하는 exec 를 소유하지 않는다 | Warning (sensor) |

## 판정

**ARCH-5 — build-time 인가 런타임인가**: 이 파일에 `robot_descriptions` 가 나타나면 그것이 build-time 의존인지 본다 — build-time 의존은 금지이고 런타임 lookup 만 허용이다. 허용 / 금지 목록, `<test_depend>` 의 조건, 복구는 invariants.md §ARCH-5 세부 스펙이 갖는다 (여기 복제하지 않는다).

**ARCH-7 — 무엇이 걸리는가**: sensor 는 `rtc_*/CMakeLists.txt` 에서 **HEAD 에 없던 타깃 이름**을 본다 — 기존 타깃을 옮기거나 다시 들여쓰는 것은 걸리지 않는다. 예외의 범위와 `ARCH-7-exempt` 마커는 [design-principles.md](../../agent_docs/design-principles.md) §Boundary Rules 가 SSoT 다 (마커가 덮는 줄 범위는 hook). 새 exec 를 면제로 선언하기 전에 그 절을 열어 그것이 정말 robot-agnostic 한지 확인한다.

**ARCH-2 — 방향**: 새 `<depend>` / `find_package` / `target_link_libraries` 를 추가하기 전에 [architecture.md](../../agent_docs/architecture.md) §Dependency Graph 에서 두 패키지의 층을 확인한다. 아래층이 위층을 참조하면 위반이다.

## 함께 걸리는 게이트

- **CMake / `package.xml` co-update 는 blocking** 이다 (PROC-1) — 파일을 add/remove 했으면 같은 turn 에 여기도 반영한다.
- `rtc_base` / `rtc_msgs` 의 의존을 건드렸다면 **전체 빌드·테스트** (PROC-3).
- 위반이 필요하면 [AGENTS.md](../../AGENTS.md) §6 의 `[CONCERN]` 으로 보고 — ARCH-2 는 Critical 이라 승인 전 커밋·PR 금지.
