# issue-157: 주기 루프 root span 계약 검증 + integrated 제어 attribution 보강

이슈: https://github.com/hyujun/rtc-framework/issues/157
작성/검증일: 2026-07-15

## Goal

주기성 검증의 기준을 **실제 periodic thread 의 tick root span** 으로 확정하고, 그 계약(L1 tick → L2 phase, 예외적 L3+)을 `docs/tracing.md` 에 문서화한다. 그 위에서 integrated 제어 경로의 빠진 attribution(backend RT I/O, controller non-RT publish, WBC 진단, MuJoCo reset, hand E-Stop zero-write)을 채운다.

## Acceptance criteria (= Sprint Contract)

이슈 본문의 수용 기준을 그대로 승계한다.

1. `RTC_ENABLE_TRACING=ON` 빌드 + `enable_tracing:=true` 캡처에서 각 root span(`rt_control_tick`, `sim_step`, `hand_comm_tick`)의 begin timestamp 로 cadence 판별 가능.
2. 기본 trace tree 가 L1 tick → L2 phase 를 보장하고, L3 이상은 진단 목적과 함께 문서화.
3. `sim_step` 의 wait/throttle 을 physics cost 로 오해하지 않도록 `sim_wait_command` / `ThrottleIfNeeded` 와 함께 분석하도록 문서가 안내.
4. Tracing OFF 기본 빌드에서 instrumentation 은 no-op, RT alloc/lock/log 규칙 추가 위반 없음.
5. `docs/tracing.md` 의 span tree 와 실제 scope 이름이 일치.

## Out of scope

- Perfetto **viewer / 변환기** 개선 — 선행 작업으로 이미 완료 (아래 Current state). 추가 display 변경은 별도 task.
- 새 tracepoint provider · LTTng 채널 설계 변경.
- Pinocchio FK / ProxSuite QP 등 외부 라이브러리 내부 구간 계측.

## Current state

### 선행 완료 — viewer 쪽 (커밋 `90f6e0b`, 2026-07-15)

수용 기준 1(“Perfetto 에서 root-span cadence 판별”)의 **관측 도구** 가 준비됐다. `ctf_to_chrome_trace` 가 workspace 프로세스(= `rtc:span` emit) 중심으로 표시하도록 재편:
- Cpu lane 라벨 `프로세스명/comm-tid`, thread slice 는 실제 vpid 기준 process 그룹.
- `rtc:span` 없는 외부 UST 프로세스(ur_driver 등)는 프로세스당 async 요약 lane 1개로 축약, sched_switch 전용 스레드는 thread row 미생성 → row 폭증 해소.
- Escape hatch `--all-threads` / `--focus-proc`. `docs/tracing.md` View 절 갱신 완료.
- 검증: `colcon test --packages-select rtc_tools` 357 passed.

### 이슈 체크박스 7개의 실제 코드 상태 (2026-07-15 grep 검증)

이슈 본문 서술은 가설로 취급하고 전부 재확인했다. **체크박스 7개 모두 유효(열림)** 이나, 3번은 이슈 서술과 실제가 다르다.

| # | 항목 | 검증 결과 |
|---|---|---|
| 1 | cadence acceptance 기준 문서화 | **열림** — `docs/tracing.md` 에 cadence / begin-to-begin 기준 서술 없음 |
| 2 | 2-layer 계약 + deeper-span 예외 명시 | **열림** — L2 개념은 §“Executor callback 내부 L2 span”(tracing.md:186)에만 존재, 기본 L1/L2 계약과 WBC·MPC·MuJoCo 예외 규정 없음 |
| 3 | backend RT I/O attribution | **열림 — 단 이슈 서술 부정확** (아래 주석 참조) |
| 4 | controller non-RT publish attribution | **열림** — `PublishNonRtSnapshot` 3곳 모두 미계측 (wbc/controller.cpp:1879, task/controller.cpp:716, joint/controller.cpp:606) |
| 5 | WBC rich diagnostics | **열림** — `UpdatePhase`(phase.cpp:27) 미계측(phase.cpp 에 span 0개), `LogMpcSolveTimingTick`(compute.cpp:1028) 미계측 |
| 6 | MuJoCo `HandleReset` one-shot span | **열림** — mujoco_sim_loop.cpp:426 미계측 (418=`ClearContactForces`, 525=`sim_step` 사이) |
| 7 | hand `OnCommLoopAborted` sibling span | **열림** — udp_hand_controller.hpp:1223 body 미계측 |

**3번 주석 (중요):** 세 backend 파일에 `RTC_TRACE_SCOPE` 가 이미 있어 “계측됨”으로 오독하기 쉽다. 실제로는 전부 **non-RT executor callback** 쪽이며(`OnJointState`/`OnMotorState`/`OnSensorState`/`OnWrench` — `tracing.md:133` 표에 의도적으로 문서화된 L2), 이슈가 지목한 **RT path 쪽 `ReadState` / `ReadMotorState` / `ReadSensorState` / `WriteCommand` 는 전부 미계측**이다. 따라서 3번은 “없는 걸 추가”가 아니라 “있는 것과 다른 쪽(RT tick 하위)에 추가”다.

### 이슈 본문 “현재 확인된 상태” 검증 — 전부 사실

- `rt_control_tick` = rt_controller_node_rt_loop.cpp:46, `CM::*` L2 5개 동일 파일. ✓
- `sim_step` = mujoco_sim_loop.cpp:525, `sim_wait_command` = 535. ✓
- `hand_comm_tick` = udp_hand_controller.hpp:701. ✓
- WBC/Task/Joint Compute 상세 계측 (span 22/7/7). ✓

## Next action

1. **문서 먼저** (체크박스 1·2) — `docs/tracing.md` 에 (a) root span begin-to-begin cadence acceptance 기준 + CSV timing/overrun 을 보조 근거로 병기, (b) 기본 2-layer 계약과 허용 예외(WBC / MPC / MuJoCo external solver) 명시. 기존 §“Executor callback 내부 L2 span”·§“계측 제외 (의도적)” 와 중복되지 않게 통합 (AP-DOC-1).
2. **RT-path attribution** (체크박스 3) — 세 backend 의 `ReadState`/`Read*State`/`WriteCommand` 에 span 추가. RT tick 하위이므로 [.claude/rules/rt-path.md](../../../.claude/rules/rt-path.md) 구속 — `RTC_TRACE_SCOPE` 는 tracing OFF 시 no-op 임을 확인하고 넣는다.
3. 체크박스 4·5·6·7 (non-RT publish / WBC 진단 / MuJoCo reset / hand E-Stop zero-write). 7번은 **tick 자식이 아닌 sibling** 으로 — `OnCommLoopAborted` 는 loop unwind 후 호출되므로 `hand_comm_tick` 바깥이다.
4. 검증: `./build.sh sim --tracing` → `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true` → `./repo_scripts/scripts/timeline.sh` → Perfetto 에서 root cadence + nested attribution 확인.

## Decisions and rationale

- **viewer 선행** — 수용 기준이 “Perfetto 에서 판별 가능” 인데, 기존 flat 표시에서는 외부 스레드 row 폭증으로 workspace root span 을 찾기 어려웠다. 관측 도구를 먼저 고쳐야 계측 검증이 성립.
- **외부 프로세스는 async 요약 lane (완전 제거 아님)** — 사용자 결정 (2026-07-15). row 는 줄이되 ur_driver callback 타이밍은 남긴다: p1b comm cascade 류 분석에서 그 가시성이 필요했다.
- **Focus 분류는 `rtc:span` 존재 기반 (data-driven)** — 프로세스명 하드코딩은 ARCH-1 위반이자 robot/driver 교체 시 drift.
- **이슈 체크박스를 착수 전 전수 grep 검증** — [[feedback_issue_body_diagnosis_unverified]] 패턴. 3번에서 실제 반례(이미 있는 span 이 RT 쪽이 아님)를 발견.

## Evidence

- `git log --oneline -1` → `90f6e0b feat(rtc_tools): focus workspace threads in Perfetto timeline (refs #157)`
- `colcon test --packages-select rtc_tools` (ws root, env source 후) → **357 passed in 11.05s**
- `ruff check` (converter + test) → All checks passed
- 합성 babeltrace2 텍스트 end-to-end 변환 → tier 그룹핑 / sort order / async id 짝 확인 (focus pid=100 sync B/E, external pid=200 async b/e, Cpu 라벨 `ur_driver/dds_worker-201`)
- span 인벤토리 grep (위 표) — `grep -rn 'RTC_TRACE_SCOPE("' --include=*.cpp --include=*.hpp`
- **미검증**: 실 CTF 캡처의 Perfetto 육안 확인 (디스크에 기존 캡처 없음 → 다음 캡처 시 확인 필요)

## Failed approaches

- (viewer) sync B/E 로 외부 프로세스를 프로세스 lane 에 병합 → 서로 다른 tid 의 callback 이 시간축에서 겹쳐 스택이 깨진다. Chrome async event(`ph: b/e` + id)로 전환해 해결.
- (viewer) 스트리밍 중 CPU lane 라벨 확정 → tid→pid 매핑이 그 시점에 미완성. 후처리 pass 로 분리.

## Constraints / pending human decisions

- 없음. viewer 설계 결정은 사용자 컨펌 완료(2026-07-15), 계측 항목은 이슈 본문이 승인된 범위.
- 주의: 체크박스 3 은 RT hot path 편집 — rt-path rule 로드 대상. RT invariant 위반 필요시 §6 `[CONCERN]`.

## Workspace

- branch `main`, HEAD `90f6e0b` (clean; viewer 작업 커밋 완료)
- 미커밋: `docs/WBC_CONTROLLER_IMPLEMENTATION.md` (untracked) — **이 task 소유 아님**, 별건. 건드리지 말 것.

## Pointers

- 이슈: [#157](https://github.com/hyujun/rtc-framework/issues/157)
- 계약 문서 (수정 대상): [docs/tracing.md](../../tracing.md) — §RT trace spans(62~), §Executor callback 내부 L2 span(186), §계측 제외(199), §View(269~)
- Root spans: [rt_controller_node_rt_loop.cpp:46](../../../rtc_controller_manager/src/rt_controller_node_rt_loop.cpp#L46) · [mujoco_sim_loop.cpp:525](../../../rtc_mujoco_sim/src/mujoco_sim_loop.cpp#L525) · [udp_hand_controller.hpp:701](../../../udp_hand_driver/include/udp_hand_driver/udp_hand_controller.hpp#L701)
- 체크박스 3 대상: [ur_driver_native_backend.cpp:88,93](../../../integrated_bringup/src/backends/ur_driver_native_backend.cpp#L88) · [udp_hand_native_backend.cpp:176,181,189,199](../../../integrated_bringup/src/backends/udp_hand_native_backend.cpp#L176) · [mujoco_native_backend.cpp:154](../../../integrated_bringup/src/backends/mujoco_native_backend.cpp#L154)
- 체크박스 4 대상: [wbc/controller.cpp:1879](../../../integrated_bringup/src/controllers/wbc/controller.cpp#L1879) · [task/controller.cpp:716](../../../integrated_bringup/src/controllers/task/controller.cpp#L716) · [joint/controller.cpp:606](../../../integrated_bringup/src/controllers/joint/controller.cpp#L606)
- 체크박스 5 대상: [wbc/phase.cpp:27](../../../integrated_bringup/src/controllers/wbc/phase.cpp#L27) · [wbc/compute.cpp:1028](../../../integrated_bringup/src/controllers/wbc/compute.cpp#L1028)
- 체크박스 6·7 대상: [mujoco_sim_loop.cpp:426](../../../rtc_mujoco_sim/src/mujoco_sim_loop.cpp#L426) · [udp_hand_controller.hpp:1223](../../../udp_hand_driver/include/udp_hand_driver/udp_hand_controller.hpp#L1223)
- Span 매크로: [rtc_base/include/rtc_base/tracing/trace_scope.hpp](../../../rtc_base/include/rtc_base/tracing/trace_scope.hpp)
- Viewer: [ctf_to_chrome_trace.py](../../../rtc_tools/rtc_tools/conversion/ctf_to_chrome_trace.py) · [timeline.sh](../../../repo_scripts/scripts/timeline.sh)
- RT 규칙: [.claude/rules/rt-path.md](../../../.claude/rules/rt-path.md) · [agent_docs/invariants.md](../../../agent_docs/invariants.md) (RT-10 = OnCommLoopAborted 배경)
