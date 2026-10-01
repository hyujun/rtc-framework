---
paths:
  - "**/*.cpp"
  - "**/*.hpp"
  - "**/*.h"
  - "**/*.cc"
  - "**/*.py"
  - "*/**/*.cpp"
  - "*/**/*.hpp"
  - "*/**/*.h"
  - "*/**/*.cc"
  - "*/**/*.py"
---

# Architecture invariants — 소스 편집 시 (ARCH-1 · ARCH-2 · ARCH-3 · ARCH-4 · ARCH-6)

소스 파일을 읽거나 편집할 때 로드되는 reminder 이며, 이 파일이 갖는 것은 **판정**이다. 규칙 전문·severity·복구는 [invariants.md](../../agent_docs/invariants.md) §Architecture Invariants, 탐지 패턴·스코프·면제는 [.claude/hooks/verify-changes.sh](../hooks/verify-changes.sh) 가 SSoT 이고, build metadata 축 (ARCH-5 · ARCH-7) 은 [arch-build-meta.md](arch-build-meta.md) 가 갖는다. frontmatter 의 glob 형태는 줄이지 않는다 — 이유와 확인 절차는 `repo_scripts/scripts/validate_claude_rules.py` 헤더.

| # | 규칙 | 구속 대상 | Severity |
|---|---|---|---|
| ARCH-1 | robot 이름 · joint 수 · HW ID 하드코딩 금지 | `rtc_*` 프로덕션 코드 (`include/` · `src/` · 모듈 dir). test 면제 | **Critical** (E-2) |
| ARCH-2 | 의존성 그래프 상향 의존 금지 | 전 패키지 (include 방향) | **Critical** (E-1) |
| ARCH-3 | abstract interface 없이 **두 번째** 구체 구현 추가 금지 | 전 패키지 | Warning (E-4) |
| ARCH-4 | integration 패키지가 `rtc_*/src/` private 헤더 include 금지 | 비-`rtc_*` 패키지 | **Critical** (E-1) |
| ARCH-6 | 모든 ROS 2 topic QoS 는 `KEEP_LAST` depth **1** | 프로덕션 C++ / Python. test fixture 면제 | Warning (sensor) |

## 판정

**ARCH-1 — "robot-specific" 인지**: 상수가 *특정 로봇에서만 참* 이면 위반이다. `ur5e` · `panda` 같은 이름, `num_joints = 6`, HW ID 리터럴. 같은 값이라도 **YAML/URDF 에서 읽어 런타임에 정해지면** 위반이 아니다. 판정이 애매하면 "이 패키지를 다른 로봇에 그대로 쓸 수 있는가" 를 묻는다. **주석·docstring 의 로봇 이름도 대상이다** — 이름은 rename 에서 썩고, agnostic 헤더에 "이 패키지는 저 로봇 것" 이라는 문서를 남긴다. gate 가 그것을 잡으면 오탐이 아니라 정상 발화이므로 robot-neutral 하게 고쳐 쓰고 (구체 예시는 소비 패키지의 README·config·plan 으로 내린다), 그 turn 이 **추가하지 않은** 줄은 애초에 스코프 밖이라 잡히지 않는다. 차단형 sensor 라 "오탐이니 그대로 둔다" 로는 종결되지 않는다 ([invariants.md](../../agent_docs/invariants.md) §False-positive 처리).

**ARCH-3 — "두 번째" 세는 법**: 같은 역할을 하는 구현이 이미 하나 있는데 두 번째를 추가하려는 순간이 트리거다. `#ifdef` 나 하드코딩 switch 로 분기하려는 충동이 곧 신호다 — 그 자리에 pure-virtual base 또는 concept 를 먼저 만든다. 세 번째에서 고치면 이미 늦다. 단일 backend 의 key 확장처럼 **아직 두 번째가 아닌** 경우는 대상이 아니다.

**ARCH-4 — 경계**: `rtc_*/src/` 아래 헤더는 private 이다. integration 패키지가 그것을 include 하고 있다면 필요한 것을 public 헤더 (`rtc_*/include/`) 로 승격하거나, 애초에 그 의존이 ARCH-2 위반이 아닌지 본다.

**ARCH-6 — depth 만 움직인다**: QoS 를 쓰거나 고칠 때 depth 만 1 로 맞추고 `reliability` / `durability` 는 건드리지 않는다. 대상 범위 (`SensorDataQoS()` 포함), 예외가 되는 조건, `ARCH-6-exempt` 마커와 예외 기록은 invariants.md §ARCH-6 세부 스펙이 갖는다 (여기 복제하지 않는다).

## 위반이 필요할 때

고쳐 쓰지 말고 [AGENTS.md](../../AGENTS.md) §6 Escalation 의 `[CONCERN]` 포맷으로 보고하고 컨펌을 기다린다. Critical (ARCH-1 · ARCH-2 · ARCH-4) 은 **승인 전 커밋·PR 금지**다.
