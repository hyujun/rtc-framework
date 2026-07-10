# Tracing — ros2_tracing (LTTng UST + kernel sched_switch)

이 문서는 *operational guide* 다. RT 안전성 / invariant / sensor matrix 등 헌법성 규칙은 [../agent_docs/](../agent_docs/) 참조 — 여기는 LTTng / babeltrace2 / Perfetto 사용법만.

CSV timing log 는 per-tick 총 시간만 기록하므로, 어느 thread 가 어느 core 에서 언제 run 했는지 / 어떤 callback 이 시간을 쓰는지 모를 때 LTTng 트레이스를 캡처해 Perfetto 로 본다. sensor matrix 상 위치·언제 트레이싱을 켜는지는 SSoT [../agent_docs/testing-debug.md](../agent_docs/testing-debug.md) §Tracing 참조.

## ros2_tracing vs perf

이전엔 `perf record` sampling 을 썼지만 RT path 의 30 µs WBC tick / hand UDP burst 처럼 짧은 함수는 999 Hz sampling 으로 **통계적으로 못 잡는다** — 자고 있는 동안의 주기성도 못 본다.

`ros2_tracing` 은 **event-driven**:

* `ros2:callback_start` / `ros2:callback_end` — 정확한 begin/end 페어 (sampling 추정 X)
* `sched_switch` (kernel tracepoint) — 어느 thread 가 어느 core 에서 언제 run 했는지의 정확한 timeline
* `irq_handler_entry/exit` — IRQ leak 검출

대신 callback **enter ~ exit 사이** 만 본다 — 그 안의 함수 단위 breakdown 이나 RT pthread tick 경계는 자동 모드로 안 잡힌다 (§Limits 참조).

## One-time setup (Fresh Ubuntu 24.04)

```bash
./install.sh --tracing
# lttng-tools / lttng-modules-dkms / babeltrace2 / python3-bt2 / ros-jazzy-ros2trace
# 등 동등 패키지를 install_dev.sh install_tracing_tools() (SSoT) 가 설치한다 —
# 수동 apt 목록은 여기 박제하지 않는다 (drift 방지). 소스: repo_scripts/scripts/lib/install_dev.sh

# 별도로 tracing 그룹 멤버십만 필요하면:
sudo usermod -a -G tracing "$USER"
# 적용은 logout/login 또는 'newgrp tracing'
```

검증: `./repo_scripts/scripts/check_rt_setup.sh --summary` →
`Tracing  lttng=1 kmod=1 group=1 launch=1`

`kmod=0` 이면 `sudo dkms status lttng-modules` 로 빌드 상태 확인. `group=0` 이면 logout/login 또는 `newgrp tracing`.

### Secure Boot 호스트 (NUC / 최신 노트북 다수)

`modprobe lttng-tracer` 가 `Key was rejected by service` 로 실패하고
`lttng list --kernel` 이 `Kernel tracer not available` 을 내면, DKMS 가 빌드한
서명되지 않은 module 을 Secure Boot 가 거부하는 상태다. `check_rt_setup.sh` 의
`kmod=1` 은 module 파일이 디스크에 있다는 뜻이지 로드 가능 여부가 아니므로
이 단계에서는 통과한 것처럼 보인다.

해결 — MOK (Machine Owner Key) 등록:

```bash
./repo_scripts/scripts/enroll_lttng_mok.sh
# 1회성 helper. 키 생성 + DKMS 서명 설정 + lttng-modules 재빌드 + mokutil
# import 까지 자동. 마지막에 사용자가 직접 해야 하는 일을 안내한다.

# 그 다음 reboot → UEFI 'Perform MOK management' 화면에서
#   Enroll MOK → Continue → Yes → (helper 에서 설정한 password) → Reboot.
# 재부팅 후:
sudo modprobe lttng-tracer && lsmod | grep lttng    # 이제 module 로드됨
sudo systemctl restart lttng-sessiond
lttng list --kernel                                 # 이벤트 목록 표시됨
```

Secure Boot 가 disabled 라면 helper 는 그 사실을 알리고 즉시 종료한다.

## Capture (sim 또는 robot)

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true
# 또는
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py enable_tracing:=true

# 출력: <ws>/logging_data/<YYMMDD_HHMM>/tracing/<session_name>/  (LTTng CTF 디렉토리)
# CSV timing log 와 동일한 session_dir 트리에 들어가므로 archive / 분석이 1:1 매칭.
# session_name default = "trace" (surround session_dir 가 이미 YYMMDD_HHMM 식별자 보유).
# Ctrl+C 로 launch 종료 시 ros2_tracing 의 OnShutdown handler 가 lttng_fini 호출 — flush 자동.
```

LaunchArgument:

* `enable_tracing:=true|false` — capture 전체 on/off (기본 false).
* `trace_session_name:=<str>` — LTTng session 이름. `<session_dir>/tracing/`
  아래의 leaf 폴더 이름이 된다. 비우면 `"trace"` (기본). 같은 launch run 안에서
  여러 trace 를 캡처하려면 (예: warmup + steady-state) 명시적으로 다른 값을 준다.
* `trace_events_ust:=<comma-separated>` — UST events. 비우면 ros2_tracing 의
  `DEFAULT_EVENTS_ROS` (광범위한 `ros2:*` coverage). 좁히려면 예:
  `ros2:callback_start,ros2:callback_end`.
* `trace_events_kernel:=<comma-separated>` — kernel events. 기본은
  `sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit` —
  "어느 thread 가 어느 core 에서 언제 run 했나" 를 보기에 충분. **빈 값 = kernel
  tracing 비활성화 (UST only)** — kernel 권한 없을 때 fallback.

## Examples

```bash
# 기본 (UST 전체 + kernel sched + IRQ)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true

# UST only (kernel module / tracing group 없을 때)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true \
    trace_events_kernel:=

# callback timing 만 (좁은 UST + 풀 kernel)
# rclcpp_callback_register 를 빼면 callback 이름이 symbol 대신 주소로만 표시된다.
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true \
    trace_events_ust:=ros2:callback_start,ros2:callback_end,ros2:rclcpp_callback_register

# 명시적 session 이름 (CI / 여러 run 비교)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true \
    trace_session_name:=mpc_phase4_baseline
```

## View — timeline.sh + Perfetto UI

```bash
./repo_scripts/scripts/timeline.sh                  # 최신 logging_data/*/tracing/*/ 자동 사용
./repo_scripts/scripts/timeline.sh <trace_dir>      # 명시 입력
./repo_scripts/scripts/timeline.sh <trace_dir> out.json
```

`timeline.sh` 가 babeltrace2 (또는 `python3-bt2` binding) 로 CTF 를 읽어 Chrome trace JSON 으로 변환한다. 출력은 `<trace_dir>/trace.json`.

* https://ui.perfetto.dev 에 JSON drag-drop. 설치 0, 브라우저만.
* 세 가지 swimlane 그룹:
  - **Threads (by TID)** — `mpc_main-NNNN`, `rt_control-NNNN`, `mujoco_simulato-NNNN`
    등 각 thread 의 callback B/E 슬라이스. WBC tick 의 주기성, MPC 8 ms
    주기를 시간축에서 직접 확인.
  - **Cpus** — `Cpu 000` ~ `Cpu 011` 코어별 sched_switch 기반 timeline.
    ApplyThreadConfig pinning 의 의도대로 rt_control 이 Core 2 에 고정됐는지,
    mpc_main 이 MPC 코어에 머물렀는지 검증.
  - **Cpu/IRQ** — IRQ handler entry/exit 만 분리. RT core 에 IRQ leak 이 있는지
    한눈에.
* Perfetto UI tip: 좌측 swimlane 헤더 클릭 → 그 lane 만 펼침. 마우스 휠로 zoom,
  shift+드래그로 범위 측정. Ctrl+F 으로 callback symbol 검색.
* Callback 슬라이스의 symbol 은 `ros2:rclcpp_callback_register` 이벤트 (기본 UST
  캡처에 포함) 로 해석된다 — register 가 없는 캡처는 `callback@0x...` 주소로 표시.

### Event drop policy (JSON size)

기본 변환은 **구조화 이벤트 (callback B/E + sched_switch + irq_handler_*)** 만
slice 로 emit 한다. `sched_wakeup` / `ros2:rclcpp_publish` / `ros2:rcl_init`
등 나머지 tracepoint 는 silently drop — instant marker 로 박으면 Perfetto 에서
aggregate 안 되는 vertical line 만 늘고 JSON 이 2 배 가까이 부푼다. 변환 종료 시
stderr 에 `dropped N non-structured events (M distinct names). Top: <name>=<count> ...`
요약이 찍히므로 어떤 이벤트가 잘렸는지는 항상 가시화된다.

특정 이벤트를 다시 보고 싶을 때:

```bash
# 한두 종류만 살리기 (확장성용)
python3 -m rtc_tools.conversion.ctf_to_chrome_trace \
    --input <trace_dir> --output out.json \
    --keep-events ros2:rclcpp_publish,sched_wakeup

# legacy: 모든 unknown event 를 instant marker 로 emit (debugging 용, 큰 JSON)
python3 -m rtc_tools.conversion.ctf_to_chrome_trace \
    --input <trace_dir> --output out.json --keep-all
```

`timeline.sh` 는 default drop 모드만 노출한다 — 화이트리스트 / keep-all 은
위 처럼 직접 호출.

## babeltrace2 직접 사용 (raw inspection)

JSON 변환 없이 events 를 텍스트로 보고 싶을 때:

```bash
babeltrace2 logging_data/<session>/tracing/<lttng_session>/ | head -50
babeltrace2 logging_data/<session>/tracing/<lttng_session>/ | grep ros2:callback_start | wc -l
babeltrace2 logging_data/<session>/tracing/<lttng_session>/ | grep sched_switch | awk '{print $NF}' | sort -u
```

babeltrace2 텍스트 출력을 그대로 파이프해 Chrome trace JSON 으로 변환할 수도 있다
(`--stdin` — subprocess 없이 stdin 파싱):

```bash
babeltrace2 logging_data/<session>/tracing/<lttng_session>/ \
    | python3 -m rtc_tools.conversion.ctf_to_chrome_trace --stdin --output trace.json
```

## ros2 trace CLI (수동 capture)

launch 안에서가 아니라 별도로 trace 를 시작/정지하려면:

```bash
ros2 trace start --session-name manual_trace \
    --ust ros2:callback_start ros2:callback_end \
    --kernel sched_switch
# … 다른 터미널에서 노드 실행 …
ros2 trace stop manual_trace
# 출력: ~/.ros/tracing/manual_trace/ (CLI 기본 위치 — launch 와 다름)
#   경로는 $ROS_HOME/tracing 에서 파생 ($ROS_HOME 기본 ~/.ros). $ROS_TRACE_DIR 로 override 가능.
```

CLI 와 launch action 동시 사용 금지 (같은 lttng-sessiond 에 접근하므로 session_name 충돌).

## tracetools_analysis (Jupyter post-processing)

```bash
sudo apt install ros-jazzy-tracetools-analysis
# 그 다음 Jupyter notebook 에서:
#   from tracetools_analysis.loading import load_file
#   from tracetools_analysis.processor.ros2 import Ros2Handler
```

자세한 사용은 ros2_tracing 공식 문서 참조. (RT-specific manual tracepoint 는 아직 미착수 — §Limits.)

## Permissions

`install.sh --tracing` 이 사용자를 `tracing` 그룹에 추가한다. 그래도 다음 조건이 필요:

| 경로 | 요구 권한 |
|---|---|
| UST tracing (`ros2:*` 만) | 그룹 불필요 — user-space tracepoint |
| Kernel tracing (`sched_switch` 등) | `tracing` 그룹 멤버십 **또는** sudo |
| lttng-modules-dkms 빌드 | `linux-headers-$(uname -r)` + `build-essential` (apt 가 알아서) |

`enable_tracing:=true` + kernel events 가 EPERM 으로 실패하면 → `groups` 출력 확인 → `newgrp tracing` 또는 logout/login.

## Limits

* **Callback 내부의 함수 단위 breakdown** 은 ros2_tracing 자동 모드로는 안 보인다.
  WBC tick 안의 Pinocchio FK / ProxSuite QP 비율 같은 건 별도 manual tracepoint
  필요 (Phase 2).
* **RT thread (rt_control / mpc_main / hand UDP)** 의 tick 경계는 자동 tracepoint
  로는 잡히지 않음 — pthread 기반이라 rclcpp executor 콜백이 아니기 때문 (Phase 2).
* Trace 파일 크기는 캡처 시간 + UST/kernel event volume 에 선형. 1 분 캡처 ~ 10–100 MB.
  너무 길게 캡처하면 `lttng-sessiond` 가 디스크 부담을 받음.
* `lttng-modules` 가 빌드되지 않은 환경 (DKMS 실패 / 비표준 kernel) 에서는 kernel
  events 가 캡처되지 않는다. `trace_events_kernel:=` 로 빈 값 주면 UST only 로 fallback.
