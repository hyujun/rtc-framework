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

- Perfetto **viewer / 변환기** 개선 — 별도 artifact [timeline-perfetto-display.md](timeline-perfetto-display.md) 소유.
- 새 tracepoint provider · LTTng 채널 설계 변경.
- Pinocchio FK / ProxSuite QP 등 외부 라이브러리 내부 구간 계측.

## Current state

### 선행 — viewer 쪽 (별도 artifact 로 분리됨)

수용 기준 1(“Perfetto 에서 root-span cadence 판별”)의 **관측 도구** 는 준비됐다 (`90f6e0b` → `bdcbb7b`): workspace 프로세스 중심 표시, Cpu lane process 라벨, 캡처 결손 진단, 대용량 캡처용 `--max-duration-s`.

이 작업은 #157 의 Out of scope (계측이 아니라 표시) 이고 자체 열린 루프(사용자 제어 PC 재검증 대기)를 가지므로 **[timeline-perfetto-display.md](timeline-perfetto-display.md) 가 SSoT** 다 — 상세·실측·결정은 그쪽에만 둔다 (AP-DOC-1: 여기 복제 금지).

#157 에 필요한 것만: **실 캡처로 root span 관측이 되는 것을 확인했다** — `rt_control_tick` ×20,412 / 30 s, `sim_step` 등 포함 (아래 Evidence).

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

- **viewer 선행** — 수용 기준이 “Perfetto 에서 판별 가능” 인데, 기존 flat 표시에서는 외부 스레드 row 폭증으로 workspace root span 을 찾기 어려웠다. 관측 도구를 먼저 고쳐야 계측 검증이 성립. (viewer 내부 설계 결정은 [timeline-perfetto-display.md](timeline-perfetto-display.md) §Decisions 소유 — 여기 복제하지 않는다.)
- **이슈 체크박스를 착수 전 전수 grep 검증** — [[feedback_issue_body_diagnosis_unverified]] 패턴. 3번에서 실제 반례(이미 있는 span 이 RT 쪽이 아님)를 발견.

## Evidence

- span 인벤토리 grep (위 표) — `grep -rn 'RTC_TRACE_SCOPE("' --include=*.cpp --include=*.hpp`
- `colcon test --packages-select rtc_tools` (ws root, env source 후) → **2535 tests, 0 failures, 24 skipped** (HEAD `bdcbb7b` 기준)

### 실 캡처 end-to-end (2026-07-15, 이 워크스테이션) — #157 관련 부분만

`./build.sh sim --tracing` → `sim_ur5e_p1a.launch.py enable_tracing:=true` 30 s → `timeline.sh`. 전체 실측·변환기 쪽 수치는 [timeline-perfetto-display.md](timeline-perfetto-display.md) §Evidence.

- 빌드 산출물에 tracepoint 실재: `nm -D integrated_rt_controller` → `lttng_ust_tracepoint_rtc___span_begin` / `_span_end`.
- 캡처 census: `rtc:span_begin=972,065  sched_switch=781,897` → 결손 0.
- **root span 관측됨**: `rt_control_tick` ×20,412 (30 s), `nrt_publish_drain` ×20,412 — 수용 기준 1 의 관측 경로가 실제로 성립함을 확인.
- **재사용 fixture**: `logging_data/260715_1759/` (561 MB) — 계측 추가 후 before/after 비교에 재캡처 없이 쓸 수 있다. ⚠ `max_log_sessions=10` 로테이션 대상.
- **미검증 (잔여)**: Perfetto UI 육안 확인, 그리고 §Acceptance criteria 의 cadence 판정 자체 (begin-to-begin 간격 분석) 는 아직 수행 안 함 — 체크박스 1 의 기준을 문서화한 뒤 그 기준으로 판정할 것.

### 관찰 (이 task 범위 밖, 별건 확인 필요)

sim 실행에서 RT tick 스레드의 comm 이 `integrated_rt_c-534997` (고유 이름 미설정) 이고 Cpu 001 에서 30 s 간 14,268 switch 를 기록했다. 이 런은 memlock 상향/RT 권한 없이 돌았으므로 ApplyThreadConfig 가 적용되지 않았을 수 있다 — pinning 검증은 권한 있는 실기에서 재확인 필요.

## Failed approaches

- 계측 쪽은 아직 착수 전이라 없음. viewer 쪽 dead-end 는 [timeline-perfetto-display.md](timeline-perfetto-display.md) §Failed approaches.

## Constraints / pending human decisions

- 없음. viewer 설계 결정은 사용자 컨펌 완료(2026-07-15), 계측 항목은 이슈 본문이 승인된 범위.
- 주의: 체크박스 3 은 RT hot path 편집 — rt-path rule 로드 대상. RT invariant 위반 필요시 §6 `[CONCERN]`.

## Workspace

- branch `main`, HEAD `bdcbb7b`, origin/main 과 동기 (미푸시 커밋 없음). 계측 쪽 코드 변경은 **아직 0** — viewer 커밋만 올라가 있다.
- 미커밋: `docs/WBC_CONTROLLER_IMPLEMENTATION.md` (untracked) — **이 task 소유 아님**, 세션 시작 시점부터 존재. 건드리지 말 것.
- 빌드 상태: 이 워크스테이션은 `./build.sh sim --tracing` 으로 빌드돼 있다 (tracing ON). 계측 추가 후 재빌드 시 `--tracing` 유지할 것 — 빼면 span 이 사라져 검증이 불가능해진다.

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
