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

이 rule 은 `CMakeLists.txt` · `package.xml` · `setup.py` 를 **읽거나 편집할 때** reminder 로 로드된다. glob 형태와 그 검증 수단은 [arch-source.md](arch-source.md) 와 같다 — 특히 `**/CMakeLists.txt` 만으로는 발화하지 않았다는 실측(#229, 세션 16)과 "정적 게이트 clean ≠ 로드됨" 을 그쪽에서 읽는다. 이 저장소의 build metadata 는 전부 `<pkg>/` 바로 아래(depth 2)이므로 앵커 형태는 `*/X` 로 충분하다. 소스 축 (ARCH-1 · ARCH-3 · ARCH-4 · ARCH-6) 은 [arch-source.md](arch-source.md) 가 갖는다. 규칙 전문·복구는 [invariants.md](../../agent_docs/invariants.md) §Architecture Invariants 가 SSoT.

**탐지 패턴을 여기 복제하지 않는다** — SSoT 는 [.claude/hooks/verify-changes.sh](../hooks/verify-changes.sh) (ARCH-5 · ARCH-7 은 Phase 0a). 이 파일이 갖는 것은 **판정**이다.

| # | 규칙 | Severity |
|---|---|---|
| ARCH-2 | 의존성 그래프 상향 의존 금지 (`rtc_base` → `rtc_controllers`, `rtc_*` → integration 패키지 등) | **Critical** (E-1) |
| ARCH-5 | `robot_descriptions` 는 data-only — build-time 의존 금지 | Warning (E-10) |
| ARCH-7 | `rtc_*` 는 RT 제어 루프를 구동하는 exec 를 소유하지 않는다 | Warning (sensor) |

## 판정

**ARCH-5 — 허용/금지**: 허용은 `<exec_depend>` · ament index 런타임 lookup (`get_package_share_directory`) · `package://robot_descriptions/...` URL 이다. `<test_depend>` 도 허용되지만 **테스트가 그 런타임 lookup 을 쓸 때 설치 순서를 보장하는 용도로만** 이다. 금지는 `find_package` · `<depend>` / `<build_depend>` · `ament_target_dependencies` · `ament_export_dependencies`. 근거는 link 할 artifact 가 0 개인데 build-dep 이 colcon 토폴로지 엣지를 만들어 "어디 두든 `install/robot_descriptions/share/` 만 있으면 동작" 모델을 깨기 때문이다 (사용자 정책). 복구는 보통 코드 0 줄 — build-dep 줄 제거 + `<exec_depend>` 강등.

**ARCH-7 — 무엇이 걸리는가**: sensor 는 `rtc_*/CMakeLists.txt` 에서 **HEAD 에 없던 타깃 이름**을 본다 (줄 단위가 아니다 — CMake 줄은 제자리에서 고쳐 쓰이므로 재들여쓰기가 신규 exec 로 읽혔다). 즉 기존 타깃을 옮기거나 다시 들여쓰는 것은 걸리지 않는다. `example_*` 는 이름으로 면제된다. robot-agnostic standalone 노드는 `add_executable` 줄 또는 **바로 윗줄**에 `ARCH-7-exempt` 주석으로 표시한다. 예외 목록의 SSoT 는 [design-principles.md](../../agent_docs/design-principles.md) §Boundary Rules 이며, 새 exec 를 면제로 선언하기 전에 그것이 정말 robot-agnostic 한지 그 절을 열어 확인한다.

**ARCH-2 — 방향**: 새 `<depend>` / `find_package` / `target_link_libraries` 를 추가하기 전에 [architecture.md](../../agent_docs/architecture.md) §Dependency Graph 에서 두 패키지의 층을 확인한다. 아래층이 위층을 참조하면 위반이다.

## 함께 걸리는 게이트

- **CMake / `package.xml` co-update 는 blocking** 이다 (PROC-1) — 파일을 add/remove 했으면 같은 turn 에 여기도 반영한다.
- `rtc_base` / `rtc_msgs` 의 의존을 건드렸다면 **전체 빌드·테스트** (PROC-3).
- 위반이 필요하면 [CLAUDE.md](../../CLAUDE.md) §6 의 `[CONCERN]` 으로 보고 — ARCH-2 는 Critical 이라 승인 전 커밋·PR 금지.
