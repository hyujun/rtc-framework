---
paths:
  - "**/*.{cpp,hpp,h,cc,py}"
---

# Architecture invariants — 소스 편집 시 (ARCH-1 · ARCH-2 · ARCH-3 · ARCH-4 · ARCH-6)

이 rule 은 **소스 파일을 읽거나 편집할 때** reminder 로 로드된다 (path-scoped rule 은 read 에서도 발화한다). `**/*.ext` 는 모든 디렉토리의 그 확장자를 매칭하므로 앵커를 여러 벌 적을 필요가 없고, 패키지 이름은 열거하지 않는다 — 추가·rename 시 drift 가 되기 때문이다 (AP-DOC-1). 이 glob 이 실제로 파일을 잡는지는 `repo_scripts/scripts/validate_claude_rules.py` 가 검사하고 (0건이면 hook 이 차단), 실제로 *로드됐는지* 는 `.claude/instructions-loaded.log` 가 기록한다. build metadata (`CMakeLists.txt` / `package.xml` / `setup.py`) 축의 ARCH-5 · ARCH-7 은 [arch-build-meta.md](arch-build-meta.md) 가 갖는다. 규칙 전문·severity·복구는 [invariants.md](../../agent_docs/invariants.md) §Architecture Invariants 가 SSoT.

**탐지 패턴을 여기 복제하지 않는다** — 정규식·스코프·면제의 SSoT 는 [.claude/hooks/verify-changes.sh](../hooks/verify-changes.sh) 다 (#213: 문서가 들고 있던 divergent copy 가 hook 보다 낡은 스코프를 담은 채 썩었다). 이 파일이 갖는 것은 **판정**이다.

| # | 규칙 | 구속 대상 | Severity |
|---|---|---|---|
| ARCH-1 | robot 이름 · joint 수 · HW ID 하드코딩 금지 | `rtc_*` 프로덕션 코드 (`include/` · `src/` · 모듈 dir). test 면제 | **Critical** (E-2) |
| ARCH-2 | 의존성 그래프 상향 의존 금지 | 전 패키지 (include 방향) | **Critical** (E-1) |
| ARCH-3 | abstract interface 없이 **두 번째** 구체 구현 추가 금지 | 전 패키지 | Warning (E-4) |
| ARCH-4 | integration 패키지가 `rtc_*/src/` private 헤더 include 금지 | 비-`rtc_*` 패키지 | **Critical** (E-1) |
| ARCH-6 | 모든 ROS 2 topic QoS 는 `KEEP_LAST` depth **1** | 프로덕션 C++ / Python. test fixture 면제 | Warning (sensor) |

## 판정

**ARCH-1 — "robot-specific" 인지**: 상수가 *특정 로봇에서만 참* 이면 위반이다. `ur5e` · `panda` 같은 이름, `num_joints = 6`, HW ID 리터럴. 같은 값이라도 **YAML/URDF 에서 읽어 런타임에 정해지면** 위반이 아니다. 판정이 애매하면 "이 패키지를 다른 로봇에 그대로 쓸 수 있는가" 를 묻는다. 산문·주석의 로봇 이름은 위반이 아니다 (알려진 gate 오탐).

**ARCH-3 — "두 번째" 세는 법**: 같은 역할을 하는 구현이 이미 하나 있는데 두 번째를 추가하려는 순간이 트리거다. `#ifdef` 나 하드코딩 switch 로 분기하려는 충동이 곧 신호다 — 그 자리에 pure-virtual base 또는 concept 를 먼저 만든다. 세 번째에서 고치면 이미 늦다. 단일 backend 의 key 확장처럼 **아직 두 번째가 아닌** 경우는 대상이 아니다.

**ARCH-4 — 경계**: `rtc_*/src/` 아래 헤더는 private 이다. integration 패키지가 그것을 include 하고 있다면 필요한 것을 public 헤더 (`rtc_*/include/`) 로 승격하거나, 애초에 그 의존이 ARCH-2 위반이 아닌지 본다.

**ARCH-6 — depth 만 움직인다**: `reliability` / `durability` 는 절대 함께 바꾸지 않는다 (`transient_local` latched, `best_effort` sensor lane, `reliable` 은 그대로). 인자 없는 `SensorDataQoS()` 는 기본 depth 5 라 `.keep_last(1)` 이 필요하다. 매 샘플 누적이 계약인 lane (ToF snapshot 등) 은 QoS 라인 끝에 `// ARCH-6-exempt` 주석 + 사유를 남기면 sensor 가 건너뛴다. 예외 근거는 invariants.md §ARCH-6 세부 스펙에도 남긴다.

## 위반이 필요할 때

고쳐 쓰지 말고 [CLAUDE.md](../../CLAUDE.md) §6 Escalation 의 `[CONCERN]` 포맷으로 보고하고 컨펌을 기다린다. Critical (ARCH-1 · ARCH-2 · ARCH-4) 은 **승인 전 커밋·PR 금지**다.

새 유틸리티를 만들기 전에는 기존 `rtc_*` 에 유사 기능이 있는지 먼저 찾는다 — 맞지 않으면 fork 하지 말고 일반화한다 ([design-principles.md](../../agent_docs/design-principles.md) P5).
