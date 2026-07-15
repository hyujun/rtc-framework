# timeline/Perfetto display — workspace-thread focus + 대용량 캡처 대응

작성/갱신: 2026-07-15. 관련 이슈: [#157](https://github.com/hyujun/rtc-framework/issues/157) (계측 쪽은 별도 artifact [issue-157-tracing-span-contract.md](issue-157-tracing-span-contract.md) — **이 문서는 viewer 전용**).

## Goal

`timeline.sh` → `ctf_to_chrome_trace` → Perfetto 표시를 **workspace 에서 tracing 가능한 thread 중심**으로 만든다. 사용자 원 요청 (2026-07-15): (1) Cpu lane 에 process 와 thread 를 함께 표시, (2) workspace 밖 thread (ur_driver 등) 는 아주 단순히 요약 — raw 가 너무 많아 다른 thread 를 볼 수 없다.

목적은 **어느 thread 가 어느 CPU 에 할당됐고 얼마나 점유하는지** 파악.

## Acceptance criteria

1. Cpu lane slice 라벨에 process 명 포함 (`프로세스명/comm-tid`) — **완료·실측 확인**
2. `rtc:span` emit 프로세스 = per-thread row, 외부 UST 프로세스 = 프로세스당 요약 lane 1개, sched_switch 전용 thread = row 0개 — **완료·실측 확인**
3. `rtc_tools` 테스트 통과, 기존 assertion 약화 없음 — **완료** (2535 tests, 0 failures)
4. **사용자가 제어 PC 에서 실제로 원하는 뷰를 얻는다** — ⚠ **미완 (열린 루프, 아래 Next action)**

## Out of scope

- `RTC_TRACE_SCOPE` 배치·span 계약 (= #157 본체). 이 문서는 이미 캡처된 트레이스의 *표시* 만 다룬다.
- 새 tracepoint provider · LTTng 채널 설계.
- Perfetto 네이티브(protobuf) 포맷 전환 — JSON 부피 문제의 근본 해법이나 큰 작업, 미착수.

## Current state

### 완료 (전부 main 에 push 됨, HEAD `bdcbb7b`)

| 커밋 | 내용 |
|---|---|
| `90f6e0b` | vpid/procname context 소비 → 실제 pid 기준 process 그룹, Cpu lane `proc/comm-tid` 라벨, focus tiering (`rtc:span` 유무로 data-driven), 외부 프로세스 async 요약 lane, `--all-threads` / `--focus-proc`, `process_sort_index` 정렬 |
| `fa0cb8a` | 캡처 결손 진단 — census 출력 + 원인별 경고. **"Cpus" 그룹을 sched_switch 0건에도 무조건 emit 하던 버그 수정** (빈 그룹 = 변환기 버그와 구분 불가) |
| `1e2268b` | JSON 크기 경고 (>250 MB) — metadata 가 앞·슬라이스가 뒤라 뷰어가 중도 포기하면 "그룹은 있고 레인은 빈" 상태가 됨을 명시 |
| `bdcbb7b` | `--max-duration-s N` — 앞 N 초만 변환. 모든 lane 유지, 크기·시간 선형 감소. `timeline.sh` forward + `docs/tracing.md` §JSON 크기 |

### 사용자 보고 증상과 그 진단 (2026-07-15)

사용자 제어 PC (`/home/kapex/ros2_ws/demo_ws`, **이 워크스테이션과 다른 호스트**) 에서: "integrated_rt 에 데이터가 없다 / Cpus 에 thread 데이터가 없다".

- **1차 가설 (캡처 결손) — 반증됨.** 제어 PC 재실행 census: `rtc:span_begin=247,560  ros2:callback_start=94,614  sched_switch=2,965,939  irq_handler_entry=186,880` → `--tracing` 빌드 O, kernel event O. 경고 0건.
- **구조적 사실**: 변환기는 **이름 붙은 프로세스 그룹을 비워서 낼 수 없다** — 그룹은 슬라이스 emit 시에만 생기는 dict 키에서 나온다. 따라서 "그룹은 있는데 비어 있다" 는 변환기 밖 원인.
- **현재 유력 가설: JSON 크기.** 제어 PC 출력 = **4,445,114 events / 485 MB**. metadata 는 파일 앞, 슬라이스는 뒤 → 뷰어가 끝까지 못 읽으면 정확히 그 증상. (제어 PC 는 sched_switch 가 이 워크스테이션 sim 의 3.8배 — UR 드라이버 스택 때문에 실기 캡처가 훨씬 크다.)
- **사용자는 구버전 실행 중이었다** — census(`fa0cb8a`)는 찍혔으나 크기 경고(`1e2268b`)는 안 찍힘. **`git pull` 필요.**

## 사용자 보고 증상 2 (2026-07-15): udp_hand_driver thread 가 Cpu lane 에 없다

"udp_hand_driver 의 thread 가 연산 중인데 Cpu 에 color bar 가 없다."

### 확인된 사실 (이 워크스테이션 실측)

- **변환기는 프로세스를 Cpu lane 에서 가릴 수 없다.** Cpu lane 은 `sched_switch` 만으로 만들어지고, focus/summary tiering·async 병합은 전부 `pid != CPU_PID` 조건이라 Cpu lane 을 안 건드린다. 실측: Cpu lane 에 **154개 distinct comm** (`rustdesk`·`Xorg`·`chrome`·`kworker` 포함 — 워크스페이스 밖 프로세스도 예외 없이 올라옴), span 을 emit 하는 8 thread **전원 Cpu bar 보유, 누락 0건**.
- **sim fixture 로는 재현 불가 — 대상이 애초에 없다.** `sim_ur5e_p1a.launch.py` 는 hand config 를 `integrated_rt_controller` 파라미터로 넘길 뿐 별도 프로세스를 안 띄운다. `udp_hand_node` 가 독립 프로세스인 건 **`robot_ur5e_p1a.launch.py:521` (실기 launch) 뿐** — `_pin_external_driver` 로 taskset 핀되는 external driver. ⚠ 지난 세션의 "sim 재현 실패 = 변환기 정상" 은 근거가 못 된다 (재현 대상이 캡처에 부재).
- udp_hand_node 는 `RTC_TRACE_SCOPE` 보유 (`hand_joint_command_cb`, `hand_detector_*`) → 빌드가 `--tracing` 이면 focus tier.
- CommLoop 은 `clock_nanosleep` 500 Hz 주기 → 매 cycle deschedule → **정상이라면 Cpu bar 가 나와야 한다.**

### 남은 가설 (순서)

1. **`--max-duration-s 5` 가 startup transient 를 자른다** — 앞 5초는 lifecycle configure→activate 구간이라 CommLoop thread 가 아직 없을 수 있다 (`Start()` 는 `on_activate`). `--since` 부재와 직결 (아래 3).
2. **커널 ring buffer event loss** — 제어 PC 는 sched_switch 296만. → **감지 추가됨** (아래 Evidence §유실 감지, Next action 1 로 판정 가능).
3. **duty cycle 이 작아 안 보임** — 500 Hz × 수십 µs 는 5 s 뷰에서 픽셀 이하. 줌인 필요.

## Next action

1. **사용자가 제어 PC 에서**: `git pull` → `./repo_scripts/scripts/timeline.sh --max-duration-s 5`. 이제 유실이 있으면 stderr 에 `DISCARDED N events (x%) / worst streams: Cpu 003=...` 가 찍힌다 → **가설 2 즉시 판정**. 485 MB → ~55 MB 도 함께 확인.
2. **캡처 vs 변환기 분리** (변환기를 건너뛰고 원본 CTF 에 직접 질의):
   ```bash
   babeltrace2 <trace_dir> | grep sched_switch \
     | grep -oP 'next_comm = "\K[^"]+' | sort | uniq -c | sort -rn | grep -iE 'hand|udp'
   ```
   - **비면 캡처 문제** (그 창에서 thread 부재 / 이름 상이) → 변환기 무죄 확정.
   - **나오는데 `trace.json` Cpu lane 에 없으면 → 처음으로 변환기 버그 가능성.** `trace.json` 을 받아야 한다.
3. (사용자 요청 시) `--since` / 구간 지정 — 현재 `--max-duration-s` 는 **앞부분만** 자른다. 관심 구간이 뒤쪽이면 못 집는다. **가설 1 의 직접 해소책.** 미구현, 사용자 판단 대기 (2026-07-15 시점 "lost-event 감지만 진행" 으로 보류됨).

## Decisions and rationale

- **외부 프로세스는 async 요약 lane, 제거 아님** (사용자 컨펌 2026-07-15). row 는 줄이되 드라이버 callback 타이밍은 남긴다 — p1b comm cascade 류 분석이 그 가시성에 의존했다. → memory [[feedback-timeline-external-proc-async-lane]].
- **Focus 분류는 `rtc:span` 존재 기반 (data-driven)** — 프로세스명 하드코딩은 ARCH-1 위반이자 로봇/드라이버 교체 시 drift.
- **sync B/E 가 아니라 async b/e 로 병합** — 한 프로세스의 여러 tid callback 이 시간축에서 겹쳐 sync 스택이 깨진다.
- **경고가 아니라 수단을 준다** — `1e2268b` 로 크기를 경고했으나 사용자에게 빠져나갈 길이 없었다. `bdcbb7b` 의 `--max-duration-s` 가 실제 해법.
- **캡처를 좁히지, 뷰를 좁히지 않는다** — 표시 단계에서 슬라이스를 버리면 "왜 안 보이지" 가 다시 발생. 창/프로세스/이벤트클래스로 입력을 줄인다.
- **빈 lane 을 조용히 그리지 않는다** — 결손은 stderr 에서 원인과 함께 지목. 캡처 결손·뷰어 한계·변환기 버그가 화면상 동일하게 보이기 때문.

## Evidence

이 워크스테이션 (`/home/junho/ros2_ws/rtc_ws`) 실측, 2026-07-15:

- 빌드: `./build.sh sim --tracing` → `nm -D integrated_rt_controller` 에 `lttng_ust_tracepoint_rtc___span_begin/_end`, `liblttng-ust.so.1` 링크 확인.
- 캡처: `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true enable_viewer:=false` 30 s (SIGINT) → `logging_data/260715_1759/tracing/trace` = 235 MB, kernel+ust.
- 변환 (전체): 295 MB / 2,919,585 events / 2m6s. census `rtc:span_begin=972,065 sched_switch=781,897` — 경고 0건.
  - `integrated_rt_c` (pid 534911): 1,313,452 slices / 6 threads, `rt_control_tick` ×20,412
  - `mujoco_simulato` (534910): 778,572 / 2 threads · `Cpus`: 796,303 / **12 lanes** · `Cpu/IRQ`: 31,226
  - Cpu 라벨 실측: `integrated_rt_c/nrt_callback-534998`, `mujoco_simulato/sim_thread-534967`
  - ⚠ **정정 (2026-07-15)**: 위 줄은 원래 "main thread 는 bare (`integrated_rt_c-534997`) — 의도대로" 였으나 **오독이다.** 그 프로세스의 pid 는 **534911** 이므로 534997 은 main thread 가 아니라 *comm 이 프로세스명 그대로인 (= 이름이 안 붙은) worker* 다 (span 82,920개 = 최다). post-pass 3 의 `base.startswith(f"{proc}-")` 휴리스틱이 "comm == 프로세스명" 인 thread 를 전부 main 으로 오인한다. 표시상 버그 — 미수정, 사용자가 찾는 이름으로 Cpu bar 가 안 뜨는 원인이 될 수 있음
- 변환 (`--max-duration-s 5`): **47 MB / 466,212 events / 21s**, Cpu lane **12개 그대로**, 프로세스 그룹 4개 그대로, `rt_control_tick` ×3,071 → 크기·시간 6배 감소, 정보 손실은 시간축뿐.
- JSON 부피 분해 (sim 트레이스): rtc_span 58.3 % (158 MB) / cpu_lane 28.8 % (78 MB) / callback 11.8 %. **제어 PC 는 sched_switch 지배** (296만 vs 78만).
- 테스트: `colcon test --packages-select rtc_tools` (ws root, env source 후) → **369 tests, 0 failures** (9개 test 파일, `build/rtc_tools/pytest.xml` = 369, `pytest --collect-only` = 369 로 교차 확인). converter 단위 테스트 41 → **46개**.
  - ⚠ **정정 (2026-07-15)**: 원래 "2535 tests, 0 failures, 24 skipped" 으로 적혀 있었으나 **재현 불가 = 틀린 수치**. rtc_tools 실제 전량은 369. (silent skip 아님을 xunit + collect-only 로 확인함.)

### 유실 감지 추가 (2026-07-15, 증상 2 가설 2 대응)

- `LossStats` + bt2 `_DiscardedEventsMessageConst`/`_DiscardedPacketsMessageConst` 소비 → `_report_event_loss()`. 기존 `_gen()` 의 `isinstance(msg, bt2._EventMessageConst)` 필터가 **유실 메시지를 조용히 버리고 있었다** (census 는 "들어온 것"만 세므로 유실은 원리적으로 안 보임).
- **검증용 lossy fixture 를 실제로 만들어서 확인**: `lttng enable-channel --kernel --subbuf-size=4096 --num-subbuf=2` + `--all` 커널 이벤트 + `ls -R /usr` 부하 → lttng 자체 경고 `3505417 events were discarded` 와 변환기 출력이 **정확히 일치** (3,505,417 / 56.8%), CPU 별 귀속도 동작 (`Cpu 003=962,814`).
- clean fixture 회귀 없음: 경고 0건, 출력 466,212 events / 233091B/233089E 로 변경 전과 **바이트 동일**.
- 스트림 이름 → CPU: 실기 `kernel/kchan_3`, 합성 `kernel/chan_3` 둘 다 `_(\d+)$` 로 매칭 (`_stream_label`).
- **CLI fallback 은 유실 검사 불가** (`LossStats.supported=False`) — 그 경로의 침묵은 "유실 없음"이 아니라 "확인 안 됨". 경고를 안 내보내도록 명시 처리 + 테스트.

**재사용 가능한 fixture**: `logging_data/260715_1759/` (561 MB) 를 남겨뒀다 — 변환기 수정 시 재캡처 없이 실 트레이스로 검증 가능. ⚠ `max_log_sessions=10` 로테이션 대상이므로 sim 을 10회 더 돌리면 사라진다. 보존하려면 옮길 것.

## Failed approaches

- **sync B/E 로 외부 프로세스 병합** → 서로 다른 tid 의 callback 이 겹쳐 스택이 깨짐. async(`ph: b/e` + id)로 전환.
- **스트리밍 중 Cpu lane 라벨 확정** → 그 시점에 tid→pid 매핑이 미완성. 후처리 pass 로 분리.
- **크기 문제를 경고로만 대응** (`1e2268b`) → 사용자에게 해결 수단이 없어 무의미. `--max-duration-s` 로 대체.
- **캡처 결손 가설로 사용자 증상 설명** → 제어 PC census 로 반증됨 (spans·sched_switch 둘 다 존재). 이 워크스테이션에서 정상 캡처로 재현 시도했으나 증상 재현 실패 = 변환기 정상.

## Constraints / pending human decisions

- **열린 루프**: Acceptance criteria 4 는 사용자의 제어 PC 재검증에 달려 있다. 이 세션에서 닫지 못했다.
- `--since`/구간 지정 필요 여부 — 사용자 판단 대기 (Next action 3).
- 제어 PC 는 별도 호스트라 이쪽에서 직접 재현·검증 불가. `trace.json` 을 받아야 더 팔 수 있다.

## Workspace

- branch `main`, HEAD `bdcbb7b`, **origin/main 과 동기 (push 완료, 미푸시 커밋 없음)**
- 미커밋: `docs/WBC_CONTROLLER_IMPLEMENTATION.md` (untracked) — **이 task 소유 아님**, 세션 시작 시점부터 존재. 건드리지 말 것.
- 이 세션의 임시 파일은 session-scoped scratchpad 에 있어 새 세션에서 접근 불가. `--max-duration-s 5` 출력은 21초면 재생성되므로 보존 불필요.

## Pointers

- 변환기: [rtc_tools/rtc_tools/conversion/ctf_to_chrome_trace.py](../../../rtc_tools/rtc_tools/conversion/ctf_to_chrome_trace.py) — `build_trace()` (tiering·post-pass 3종), `_report_capture_gaps()`, `_report_size_risk()`
- 테스트: [rtc_tools/test/test_ctf_to_chrome_trace.py](../../../rtc_tools/test/test_ctf_to_chrome_trace.py) (41개)
- 실행: [repo_scripts/scripts/timeline.sh](../../../repo_scripts/scripts/timeline.sh) — 값-인자 플래그는 forward 목록에 등록 필요 (아니면 값이 trace-dir 로 오인)
- 문서: [docs/tracing.md](../../tracing.md) §View (swimlane 구조) · §JSON 크기 (실측·대응 사다리) · §Permissions
- 캡처 설정: [rtc_tools/rtc_tools/launch/trace_action.py](../../../rtc_tools/rtc_tools/launch/trace_action.py) — `context_fields` 기본 `procname,vpid,vtid` (Trace action 기본값), launch 의 `trace_events_kernel` 기본 = `sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit`
- 계측 쪽 (별개): [issue-157-tracing-span-contract.md](issue-157-tracing-span-contract.md)
- 이슈 코멘트 (검증 기록): https://github.com/hyujun/rtc-framework/issues/157#issuecomment-4978250509
