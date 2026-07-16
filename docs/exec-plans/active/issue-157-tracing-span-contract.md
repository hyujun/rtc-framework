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

| # | 항목 | 검증 결과 → **처리 (2026-07-16)** |
|---|---|---|
| 1 | cadence acceptance 기준 문서화 | ~~열림~~ **완료** — `docs/tracing.md` §“2-layer span 계약과 cadence 판정” 에 begin-to-begin cadence 기준 + CSV 보조 근거 서술 |
| 2 | 2-layer 계약 + deeper-span 예외 명시 | ~~열림~~ **완료** — 같은 §에 L1/L2 계약 + WBC·MPC·MuJoCo·backend RT I/O 예외 목록 명시 |
| 3 | backend RT I/O attribution | ~~열림~~ **완료** — 3 backend 의 RT-path `ReadState`/`Read*State`/`WriteCommand` 에 span 추가 (8개) |
| 4 | controller non-RT publish attribution | ~~열림~~ **이미 커버됨 (deviation)** — `owned_topics_publish`(owned_topics.cpp:288, 커밋 `dacabfc`, plan-verify 이전부터 존재)가 세 컨트롤러 공유 helper `PublishOwnedTopicsFromSnapshot` 안에서 전체 귀속. bdcbb7b grep 이 **controller.cpp 메서드만 보고 shared helper 를 놓친 false-open**. 매 순간 active controller 1개뿐이라 컨트롤러별 wrapper span 불필요 → 추가 안 함, `docs/tracing.md` §nrt_publish 에 근거 문서화 |
| 5 | WBC rich diagnostics | ~~열림~~ **완료** — `UpdatePhase`(RT, L3 under ComputeControl) + `LogMpcSolveTimingTick`(aux, 1 Hz timer L2) span 추가 |
| 6 | MuJoCo `HandleReset` one-shot span | ~~열림~~ **완료** — `HandleReset` body 에 span (양 call-site 커버: sibling+child) |
| 7 | hand `OnCommLoopAborted` sibling span | ~~열림~~ **완료** — `hand_estop_zero_write` span (hand_comm_tick 의 sibling) |

**3번 주석 (중요):** 세 backend 파일에 `RTC_TRACE_SCOPE` 가 이미 있어 “계측됨”으로 오독하기 쉽다. 실제로는 전부 **non-RT executor callback** 쪽이며(`OnJointState`/`OnMotorState`/`OnSensorState`/`OnWrench` — `tracing.md:133` 표에 의도적으로 문서화된 L2), 이슈가 지목한 **RT path 쪽 `ReadState` / `ReadMotorState` / `ReadSensorState` / `WriteCommand` 는 전부 미계측**이다. 따라서 3번은 “없는 걸 추가”가 아니라 “있는 것과 다른 쪽(RT tick 하위)에 추가”다.

### 이슈 본문 “현재 확인된 상태” 검증 — 전부 사실

- `rt_control_tick` = rt_controller_node_rt_loop.cpp:46, `CM::*` L2 5개 동일 파일. ✓
- `sim_step` = mujoco_sim_loop.cpp:525, `sim_wait_command` = 535. ✓
- `hand_comm_tick` = udp_hand_controller.hpp:701. ✓
- WBC/Task/Joint Compute 상세 계측 (span 22/7/7). ✓

## Next action

**체크박스 1~7 코드·문서 작업 완료 + live 검증 완료 (2026-07-16).**

Live span-emission 검증 — `sim_ur5e_p1a.launch.py enable_tracing:=true` 15 s 캡처 후 babeltrace2 로 확인 (아래 Evidence §Live capture):
- **관측됨 (이 config 에서 발생 가능한 신규 span 전부)**: `MujocoNativeBackend::ReadState`/`ReadSensorState`/`WriteCommand`(checkbox 3), `DemoWbcController::UpdatePhase`(checkbox 5, ×9,748 = 매 tick). 중첩도 문서 계약대로 — `CM::ReadDeviceState` 하위에 backend `Read*`, `ComputeControl` 하위에 `UpdatePhase`.
- **이 config 에서 미발생 (컴파일·배치 확인됨, 트리거/프로세스 부재로 관측만 안 됨)**: Ur/UdpHand backend span(sim 은 mujoco backend 만), `LogMpcSolveTimingTick`(MPC handler 모드에서만 timer 등록 — p1a 미활성), `HandleReset`(reset 명령 필요), `hand_estop_zero_write`(실 udp_hand_node + abort 필요). 이들은 해당 트리거를 주는 캡처(reset 명령 / robot launch / abort)로 관측 가능.

## Decisions and rationale

- **viewer 선행** — 수용 기준이 “Perfetto 에서 판별 가능” 인데, 기존 flat 표시에서는 외부 스레드 row 폭증으로 workspace root span 을 찾기 어려웠다. 관측 도구를 먼저 고쳐야 계측 검증이 성립. (viewer 내부 설계 결정은 [timeline-perfetto-display.md](timeline-perfetto-display.md) §Decisions 소유 — 여기 복제하지 않는다.)
- **이슈 체크박스를 착수 전 전수 grep 검증** — [[feedback_issue_body_diagnosis_unverified]] 패턴. 3번에서 실제 반례(이미 있는 span 이 RT 쪽이 아님)를 발견.

## Evidence

### 계측 구현 (2026-07-16)

- 신규 span 8+4개: backend `Read*`/`WriteCommand` (ur 2 / udp 4 / mujoco 3 — 중 mujoco `ReadSensorState`/`ReadState`/`WriteCommand`), `DemoWbcController::UpdatePhase`, `DemoWbcController::LogMpcSolveTimingTick`, `MuJoCoSimulator::HandleReset`, `hand_estop_zero_write`.
- 빌드: `./build.sh sim --tracing -p integrated_bringup,rtc_mujoco_sim,udp_hand_driver` → **clean** (유일 stderr = `joint/compute.cpp:56,65` 기존 sign-conversion warning, 이번 변경 무관).
- 테스트: `colcon test --packages-select integrated_bringup rtc_mujoco_sim udp_hand_driver` (ws root, env source 후) → **3 패키지 100% pass, 0 failures** (integrated_bringup 20/20).
- 문서-코드 이름 일치(AC5): `docs/tracing.md` 트리의 `<Backend>::ReadState` 등이 실제 scope 문자열과 1:1. 매크로 no-op(AC4)은 compile-time `#else` 분기로 보장.

### Live capture (2026-07-16, 이 워크스테이션)

- `sim_ur5e_p1a.launch.py enable_tracing:=true enable_viewer:=false use_cpu_affinity:=false` 15 s (SIGINT). RT loop 정상 — mean 68 µs, max 134 µs, overruns 0, steps 7,036. 캡처: `logging_data/260716_2036/tracing/trace/` (ust+kernel).
- `babeltrace2 <ust> | grep rtc:span_begin` name census — 신규 span 관측: `MujocoNativeBackend::ReadState` ×33,532, `ReadSensorState` ×19,496, `WriteCommand` ×19,496 (tick 당 device-group 수배), `DemoWbcController::UpdatePhase` ×9,748 (= `rt_control_tick` 수, 매 tick).
- 중첩 확인: RT thread vtid 이벤트를 순서대로 보면 `rt_control_tick` → `CM::ReadDeviceState` → `MujocoNativeBackend::ReadState`/`ReadSensorState`(begin/end 쌍 ×2 device group) → CM::ReadDeviceState end → `CM::Compute` → … → `UpdatePhase`(ComputeControl 하위). 문서 트리와 일치.

### (이전) span 인벤토리·rtc_tools

- span 인벤토리 grep — `grep -rn 'RTC_TRACE_SCOPE("' --include=*.cpp --include=*.hpp`
- rtc_tools 는 이번 task 무변경 (viewer 쪽). 최신 카운트는 [completed/timeline-perfetto-display.md](../completed/timeline-perfetto-display.md) §Evidence (369 tests).

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

- branch `main`. HEAD 는 plan 작성 시점 `bdcbb7b` 이후 `51d1031` 까지 전진(#162 등 별건 merge). 2026-07-16 세션에서 **계측 코드 변경 + `docs/tracing.md` 계약 문서화 완료** — 아직 **미커밋** (working tree).
- 변경 파일: `integrated_bringup/src/backends/{ur_driver,udp_hand,mujoco}_native_backend.cpp`, `integrated_bringup/src/controllers/wbc/{phase,compute}.cpp` (+ phase.cpp 에 `trace_scope.hpp` include 추가), `rtc_mujoco_sim/src/mujoco_sim_loop.cpp`, `udp_hand_driver/include/udp_hand_driver/udp_hand_controller.hpp`, `docs/tracing.md`.
- 미커밋(별건): `docs/WBC_CONTROLLER_IMPLEMENTATION.md` (untracked) — **이 task 소유 아님**, 건드리지 말 것.
- 빌드 상태: changed-package 를 `--tracing` 으로 재빌드해 tracing ON 유지. 재빌드 시 `--tracing` 유지 필수 — 빼면 span no-op 화.

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
