# Conventions — 근거 · 예시 · 세부

> **이 문서는 헌법이 아니다.** 스타일·도메인·커밋 규약의 **규범** 은 [agent_docs/conventions.md](../../agent_docs/conventions.md) 가 갖고, 여기는 그 규약의 근거 · 실제 위반 사례 · 세부만 둔다. 규칙 문장은 여기 다시 적지 않는다 — 두 곳에 있던 규칙이 어긋난 것이 두 번이다 (2026-10-16 정리). 둘이 어긋나면 agent_docs 쪽이 옳다.

## Code Conventions

규범: [conventions.md](../../agent_docs/conventions.md) §Code Conventions

- **Method split 을 consumer 이름으로 하는 근거** — stage 명 (`Phase3a`, `Step2`) 은 body 의 목적을 감춰 "이 method 가 왜 존재하나" 를 다른 문서에서 찾게 만들지만, consumer-named method 아래의 모든 라인은 그 consumer 를 위한 것임이 자명하다. 같은 필드를 공유 helper 로 빼지 않고 양쪽에서 독립적으로 write 하는 것은 공유 추출이 ordering drift 를 부르고 그 비용이 중복 write 비용보다 크기 때문이다. 실례: `demo_joint_controller.hpp` 의 `WriteJointCommand` 선언 위 주석.
- **Tier 0 예외의 근거** — executor 에 넘길 callback group 은 on_cleanup→on_configure 사이클에서 같은 객체여야 하므로 configure 에서 만들 수 없다. 예외의 근거는 "생성자가 편해서" 가 아니라 "configure 가 오기 전에 이미 필요해서" 이고, 후자가 아니면 Tier 1 이다. 현재 유일한 사례는 `UdpHandNode` (`cb_group_aux_` + `aux_cpu_slot`·`use_cpu_affinity`, issue #345).
- **uncrustify 금지의 근거** — uncrustify 의 ROS 2 / Eclipse-CDT 표준 스타일 (Allman braces, `T *p`, `T &r`) 이 clang-format Google base 와 영구 충돌한다. `set(ament_cmake_uncrustify_FOUND TRUE)` 스킵 트릭은 작동하지 않는다 (extras hook 이 무조건 등록됨). 대응은 `rtc_controller_interface/CMakeLists.txt` 의 NOTE 주석. 기존 패키지에서 uncrustify 를 떼낸 직후엔 `build/<pkg>/test_results/<pkg>/uncrustify.xunit.xml` 이 남을 수 있으므로 `rm -rf build/<pkg>/test_results` 후 colcon test 로 검증.
- **Include grouping** — 분류 우선순위 목록의 SSoT 는 [`.clang-format`](../../.clang-format) (`IncludeCategories`). 단 `#ifdef ... #endif` 사이에 낀 include 는 clang-format 이 grouping 못 해 blank line 이 살아 있다.

## Logging

규범: [conventions.md](../../agent_docs/conventions.md) §Logging

- **Node-owned logger = `<exec_name>` 의 근거** — ROS 노드 이름 = 실행 파일 이름이고 `node_->get_logger()` 가 그 이름을 그대로 반환한다. `rtc_controller_manager` 는 library-only (실행 파일 없음) 라 runtime identity 는 robot-specific bringup 이 소유한다.
- **서브-로거 런타임 필터링** — `set_logger_levels` 서비스의 level 숫자는 10=DEBUG, 30=WARN, 50=FATAL 이고, 상위 이름은 하위 전체에 매칭된다.

## Controller-owned CSV logging (`logs:` schema)

규범: [conventions.md](../../agent_docs/conventions.md) §Controller-owned CSV logging

- **`topic` (옵션, 현재 unused)** — 향후 `rtc_msgs/*Log` 를 DDS 로 publish 하는 옵션을 위해 예약. Q-MSG-1(a) lock 으로 현재는 schema-only POD→SPSC→CSV.
- **`instance` 는 경로 단위로 unique** — CSV 파일 stem 으로도 쓰이므로 `(msg_type, instance)` 쌍이 아니라 파일 경로가 겹치지 않게 짓는다 (Q-MSG-3 / Option A: 같은 device 의 state·sensor 는 instance 를 분리). 중복·빈 instance 는 `RegisterLog` 가 unbound handle 을 반환한다.
- **Push site 를 `Compute()` 로 제한하는 근거 (Q-ACTIVITY-GATING)** — parameter callback / BT bridge / 비-RT thread 에서의 push 를 막아 inactive controller 의 CSV 에 row 가 쌓이는 것을 방지.
- **Timestamp 의 출처 (Q-TIME)** — `state.t_relative_s` 는 CM RT loop 가 채운다 — 실기는 `t0 - log_start_time_` (steady), sim (`use_sim_time_sync`) 은 `iteration × dt` (#566).

## Documentation Requirements

규범: [conventions.md](../../agent_docs/conventions.md) §Documentation Requirements

- **서비스·메시지 계약 축 분리의 근거 — 실제 위반** — 한 서비스는 최대 네 곳 (`.srv`/`.msg` 파일 · 구현 `.cpp` 파일 헤더 주석 · `rtc_msgs/README.md` · 소비 패키지 README) 에서 설명될 수 있다. `/rtc_cm/clear_estop` 의 관측 창 산술이 네 곳에 동시에 존재했다 (#288 직후 정리).

## Commit Message Conventions

규범: [conventions.md](../../agent_docs/conventions.md) §Commit Message Conventions

- **subject 의 50 자는 `type(scope): ` 뒤부터 센다** — `main` 의 관행이 그렇다: 최근 300 개 중 205 개가 줄 전체로는 50 자를 넘고 subject 부분으로는 지킨다.
- **issue 참조에 closing keyword 를 쓰지 않는다** — 이 저장소의 이슈는 merge 뒤 정리 · 결과 코멘트까지 Done 기준에 넣는 경우가 많아, keyword 가 merge 순간 이슈를 닫으면 남은 일이 추적에서 빠진다. 줄을 넘겨 이어진 keyword 도, project item 을 Done 으로 옮기는 것도 이슈를 닫았다 (#716, #822, #791).
- **Merge method** — no-ff merge commit 이 기본인 이유 (사용자 확정 2026-08-16): 머지 커밋이 PR 단위를 히스토리에 남기고, 브랜치 커밋 본문 (PROC-6 spec-change 근거 같은 감사 기록) 이 `main` 에 보존된다. squash 는 커밋 본문을 PR 본문으로 덮어쓴다. 선례: #442 rebase · #450 no-ff; #464 까지는 squash 관례였고 `(#N)` 접미사가 그 흔적.
