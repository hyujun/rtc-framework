# Tracing — ros2_tracing (LTTng UST + kernel sched_switch)

이 문서는 *operational guide* 다. RT 안전성 / invariant / sensor matrix 등 헌법성 규칙은 [../agent_docs/](../agent_docs/) 참조 — 여기는 LTTng / babeltrace2 / Perfetto 사용법만.

CSV timing log 는 per-tick 총 시간만 기록하므로, 어느 thread 가 어느 core 에서 언제 run 했는지 / 어떤 callback 이 시간을 쓰는지 모를 때 LTTng 트레이스를 캡처해 Perfetto 로 본다. sensor matrix 상 위치·언제 트레이싱을 켜는지는 SSoT [../agent_docs/testing-debug.md](../agent_docs/testing-debug.md) §Tracing 참조.

## ros2_tracing vs perf

이전엔 `perf record` sampling 을 썼지만 RT path 의 30 µs WBC tick / hand UDP burst 처럼 짧은 함수는 999 Hz sampling 으로 **통계적으로 못 잡는다** — 자고 있는 동안의 주기성도 못 본다.

`ros2_tracing` 은 **event-driven**:

* `ros2:callback_start` / `ros2:callback_end` — 정확한 begin/end 페어 (sampling 추정 X)
* `sched_switch` (kernel tracepoint) — 어느 thread 가 어느 core 에서 언제 run 했는지의 정확한 timeline
* `irq_handler_entry/exit` — IRQ leak 검출

대신 자동 모드는 callback **enter ~ exit 사이** 만 본다 — 그 안의 함수 단위 breakdown 이나 RT pthread tick 경계는 `--tracing` 빌드 + `RTC_TRACE_SCOPE` 계측으로 본다 (§RT trace spans).

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

## RT trace spans (`rtc:*` — RT-tick 내부, 빌드 opt-in)

ros2_tracing 자동 모드는 rclcpp 콜백 (`ros2:callback_*`) 만 본다. RT 제어 루프는
raw `clock_nanosleep` pthread 라 콜백이 아니고, 그 안의 `Compute()` /
`ComputeControl` / WBC sub-step 은 평범한 C++ 호출이라 **자동 트레이스에 안 잡힌다**.

이를 보려면 `rtc_base` 의 `RTC_TRACE_SCOPE(name)` (LTTng UST `rtc:span_begin/end`)
로 계측된 구간을 **컴파일해 넣어야** 한다. 컴파일타임 게이트라, 켜지 않은 기본
빌드는 완전 no-op (RT hot-path 비용 0, lttng-ust 의존성 0).

```bash
# 트레이싱 빌드 (rtc:* span 컴파일). liblttng-ust-dev 필요 (install.sh --tracing).
./build.sh sim --tracing
# 또는 직접:
colcon build --cmake-args -DRTC_ENABLE_TRACING=ON
```

빌드 후 `enable_tracing:=true` 로 캡처하면 (아래 §Capture) `rtc:span_begin/end` 가
UST 채널에 함께 기록되고, `timeline.sh` 변환기가 이를 emit 스레드 레인의 **중첩
flame 스택**으로 렌더한다:

### 2-layer span 계약과 cadence 판정 (수용 기준)

**계약 (기본 2-layer)**: 모든 주기 스레드는 **L1 root span 1개**(tick 전체, raw
`clock_nanosleep`/`jthread` 경계)와 그 하위 **L2 span**(주요 phase — read / compute /
publish / write)으로 계측한다. L3 이상 깊은 분해는 **의도된 예외**이며, 비용·분기가
큰 경로에만 허용한다:

* **WBC** — `ComputeControl` 아래 CLIK/TSID QP sub-step (`ComputeKinematicWbc` /
  `ComputeDynamicWbc` / FSM 경로별 mode), `UpdatePhase` grasp FSM.
* **MPC** — `mpc_solve` 아래 `mpc_handler_solve` (aligator ProxDDP), `mpc_mode_swap`.
* **MuJoCo** — `sim_step` 아래 `sim_substep` → `mj_step` (external physics solver),
  one-shot `HandleReset`.
* **backend RT I/O** — `CM::ReadDeviceState` / `CM::WriteCommand` 아래 per-backend
  `ReadState`/`Read*State`/`WriteCommand`.

아래 트리의 들여쓰기가 이 L1/L2/L3 계층을 그대로 반영한다. 새 계측점을 넣을 때는
"이 span 이 L2(주요 phase)인가, 아니면 위 예외 목록에 드는 L3 인가"를 먼저 판정한다 —
2-layer 밖 깊은 span 을 예외 근거 없이 추가하지 않는다.

**Cadence 판정 (수용 기준 1)**: 주기성은 각 L1 root span (`rt_control_tick`,
`sim_step`, `hand_comm_tick`, `mpc_tick`) 의 **begin timestamp 간 간격
(begin-to-begin)** 으로 판정한다 — span *duration* 이 아니라 *시작 시각* 의 주기성이
기준이다. 목표 주기 (`control_rate` YAML 등) 대비 이 간격의 평균·분산·최댓값(worst
overrun) 이 판정 지표다. CSV timing/overrun 로그(동일 session_dir 트리)는 **보조
근거** 로 병기한다 — trace 의 begin-to-begin 과 CSV 의 tick period 가 어긋나면 계측
경계나 캡처 결손을 의심한다. 특히 `sim_step` 은 duration 에 command wait(`sim_wait_command`)
와 throttle(`ThrottleIfNeeded`) 이 섞이므로 **cadence 는 begin-to-begin 으로만** 보고,
physics 비용은 그 두 span 을 뺀 나머지로 해석한다 (physics 실행 시간으로 단독 오해 금지).

RT 제어 스레드 (`integrated_rt_controller` 의 `rt_control`):

```
rt_control_tick                          (RT tick 전체; raw clock_nanosleep 경계)
├─ CM::ReadDeviceState                    (backend Read* dispatch + state fill)
│  └─ <Backend>::ReadState / ReadMotorState / ReadSensorState   (per-backend RT SeqLock load, L3)
├─ CM::Compute                            (ControllerManager dispatch)
│  └─ DemoWbcController::Compute           (controller 진입)
│     └─ DemoWbcController::ComputeControl
│        ├─ UpdatePhase                    (grasp FSM transition guard + phase-enter, L3)
│        ├─ ComputeWbcCommon / ComputeTSIDPosition
│        ├─ ComputeKinematicWbc
│        └─ ComputeDynamicWbc … (FSM 경로에 따라 PositionMode/ReleaseMode/Fallback)
├─ CM::FillPublishSnapshot                (PublishSnapshot group_commands 채우기)
└─ CM::WriteCommand                       (inline actuator WriteCommand 루프 — actuator lane)
   └─ <Backend>::WriteCommand              (per-backend RT actuator publish, L3)
CM::CheckTimeouts                         (50 Hz watchdog; 10 tick 마다, rt_control_tick 밖 sibling)

`<Backend>` = `UrDriverNativeBackend` / `UdpHandNativeBackend` / `MujocoNativeBackend`
(바인딩된 device backend). 이들 RT-path `Read*`/`WriteCommand` span 은 rt_callback lane
의 `On*State` L2 (아래 §Executor callback) 와 **다른 쪽** — 전자는 RT tick 의 SeqLock
read/write, 후자는 non-RT executor callback 의 SeqLock write 다.
```

`CM::CheckTimeouts` 는 `ControlLoop()` 반환 **후** `OnTick` 에서 실행되므로
`rt_control_tick` 의 자식이 아니라 같은 레인의 형제 span 이다 (10 tick 마다만 발생).
위 스택은 대표 경로다 — WBC `Compute` 아래에는 per-tick 헬퍼 span 도 함께 찍힌다
(`ReadState` / `DrainTargetSlot` / `WriteJointCommand` / `FillLogOutput` /
`FillPublishOutput` / `ExtractFullState` / `BuildTargetPosture` /
`SeedHoldFromMeasured` / `Fill*LogPod` / `ComputeEstop`). `DemoJointController` /
`DemoTaskController` 도 동일 패턴으로 계측돼 있으며 (`Compute` → `ReadState` /
`DrainTargetSlot` / `ComputeControl` / `WriteJointCommand` / `FillLogOutput` /
`FillPublishOutput` / `ComputeEstop`), **그 컨트롤러가 active 일 때만** span 이
발생한다 (inactive controller 의 `Compute` 는 호출되지 않음).

MuJoCo sim 스레드 (`mujoco_simulator_node` 의 `sim_thread`, sim 빌드 한정):

```
sim_step                                 (sim_thread 1 iteration; raw std::jthread)
├─ MuJoCoSimulator::ReadState / ReadSensors / ReadContactWrenches
├─ MuJoCoSimulator::InvokeStateCallback / InvokeSensorCallback / InvokeContactWrenchCallback
├─ sim_wait_command                       (controller command sync_cv_ 대기; lock-step)
├─ MuJoCoSimulator::ApplyCommand
├─ sim_substep  ×n_substeps
│  └─ PreparePhysicsStep → mj_step → ClearContactForces
└─ ReadSolverStats / UpdateRtf / ThrottleIfNeeded
MuJoCoSimulator::HandleReset             (one-shot; reset 요청 시에만 발생. pre-publish reset 이면
                                          sim_step sibling, post-wait reset 이면 sim_step child —
                                          둘 다 continue 로 iteration 을 짧게 끊는다)
```

`sim_wait_command` 은 예전에 state-publish span 과 ApplyCommand span 사이의 "빈
구간" 으로 추정하던 controller 명령 대기 (sim lock-step) 를 명시적 span 으로 만든
것이다 — 이 구간이 길면 controller RT tick 이 느린 것이지 sim 물리가 느린 게 아니다.

Hand UDP 드라이버 CommLoop 스레드 (`udp_hand_node`, `-DRTC_ENABLE_TRACING=ON` 한정):

```
hand_comm_tick                           (CommLoop 1 tick; raw PeriodicRtThread 경계)
├─ hand_drain_stale                      (직전 cycle 잔여 datagram non-blocking drain)
├─ hand_write_echo                       (position write + echo; pending command 시에만)
├─ hand_read_motor / hand_read_joint / hand_read_sensor        (bulk 모드)
│  또는 hand_read_pos / hand_read_joint_pos / hand_read_vel / hand_read_sensor  (individual)
└─ hand_comm_tail
   ├─ hand_sensor_postproc               (PreFilter → ApplyFilters → DetectDrift)
   ├─ hand_ft_infer                      (FT calibration/Infer)
   └─ hand_callback                      (StateCallback → node publish 핸드오프)
```

fake 모드(`use_fake_hand`)는 read 대신 `hand_comm_tick └─ hand_fake_cycle └─ hand_fake_step`
(LPF 모델) 이 찍히고, tail 은 실모드와 공유한다. decimated-skip / E-Stop tick 도
`hand_comm_tick` 하위 없이 짧은 span 으로 나타난다.

CommLoop 이 abort 로 종료되면 (`CommLoop::OnLoopAborted`) 터미널 E-Stop zero-write 가
`hand_estop_zero_write` span 으로 한 번 찍힌다. 이는 **`hand_comm_tick` 의 자식이 아니라
같은 lane 의 형제** 다 — tick 루프가 이미 unwind 된 뒤 loop 스레드에서 호출되기 때문이다.

Hand 실패 감지 스레드 (`udp_hand_node` 의 `failure_detector`, 50 Hz aux jthread):

```
hand_detector_tick                       (감지 1 pass; IsRunning 게이트 통과 후 work 만, 20 ms sleep 제외)
├─ hand_detector_check                    (Check — motor/sensor data-validity)
├─ hand_detector_rate                     (CheckRate — polling-rate; startup grace 밖에서만)
└─ hand_detector_link                     (CheckLink — dead-link 감지; check_link 켜졌을 때만)
```

MPC 스레드 (`integrated_rt_controller` 의 `mpc_main`, `-DRTC_ENABLE_TRACING=ON` +
handler 모드 한정):

```
mpc_tick                                 (MPC solve tick; raw PeriodicRtThread 경계)
├─ mpc_read_state                         (MPCSolutionManager::ReadState)
├─ mpc_solve                              (Solve — handler 모드면 아래 L3)
│  ├─ mpc_mode_swap                        (cross-mode OCP rebuild; phase 전환 시에만)
│  └─ mpc_handler_solve                    (handler_->Solve — aligator ProxDDP)
└─ mpc_publish                            (PublishSolution SPSC 핸드오프)
```

MPC **workers 는 계측 제외** — jthread body 가 config apply 후 즉시 종료하는 passive
placeholder 이고, 실제 병렬성은 solver 라이브러리(aligator) 내부에 있다.

Non-RT publish 스레드 (`integrated_rt_controller` 의 `nrt_publish`):

```
nrt_publish_drain                        (SPSC Pop 성공 후 work 만; 1 ms poll 대기 제외)
└─ owned_topics_publish                   (controller-owned 토픽 publish — grasp/wbc/tof/tf)
```

`owned_topics_publish` 는 WBC/Task/Joint 세 컨트롤러의 `PublishNonRtSnapshot` 이 공유하는
`PublishOwnedTopicsFromSnapshot` helper 안에 있어 **하나의 span 으로 controller-owned
non-RT publish 전체를 귀속** 한다. 매 순간 active controller 는 하나뿐이므로(inactive 는
`PublishNonRtSnapshot` 이 호출되지 않음) 컨트롤러별 wrapper span 을 따로 두지 않는다.

MuJoCo viewer 스레드 (`mujoco_simulator_node` 의 `viewer`, sim + viewer 빌드 한정):

```
viewer_frame                             (1 프레임 렌더; ~60 Hz pacing sleep 제외)
├─ viewer_sync                            (viz qpos sync + mj_forward)
├─ viewer_render                          (mjv_updateScene + AddJointFrameGeoms + mjr_render)
├─ viewer_overlays                        (status/solver/help/sensor/modelinfo/RTF 오버레이)
└─ viewer_swap                            (glfwSwapBuffers + glfwPollEvents)
```

### Executor callback 내부 L2 span

ros2_tracing 의 `ros2:callback_start/end` 가 이미 각 executor 콜백 바깥을 L1 으로
감싸므로, 콜백 **본문** 에는 named L2 span 만 추가해 어느 lane 이 무슨 일을 하는지 본다.
콜백 심볼은 `ros2:rclcpp_callback_register` 로 해석된 L1 슬라이스 안에 이 L2 가 중첩된다.

| Lane (priority) | L2 span | 하는 일 |
|---|---|---|
| `rt_callback` (FIFO 70) | `MujocoNativeBackend::OnJointState` / `OnWrench`, `UrDriverNativeBackend::OnJointState`, `UdpHandNativeBackend::OnJointState` / `OnMotorState` / `OnSensorState` | device state SeqLock write |
| `nrt_callback` | `CM::TargetCb` | RobotTarget reorder + SPSC marshal |
| `nrt_logging` | `CM::DrainLog` | timing-CSV drain + summary print |
| hand node executor | `hand_joint_command_cb` | JointCommand name-reorder + offset + send |
| `mpc_timing_cb_group` (1 Hz aux timer) | `DemoWbcController::LogMpcSolveTimingTick` | MPC solve timing-CSV drain + 10 s aggregate INFO |

### 계측 제외 (의도적)

다음은 span 을 넣지 않는다 — 관측 수단이 따로 있거나 계측 대상이 아니다:

* **DDS / rclcpp 내부 스레드** — kernel `sched_switch` timeline 으로 관측 (UST span 불필요).
* **프로세스 main 스레드** — 노드 구성 후 50 ms sleep 루프만 도는 대기 스레드.
* **일시적 lifecycle 스레드** — on_configure/on_activate 1회성 경로.
* **MPC workers** — 위 참조 (passive placeholder).
* **python 노드** (target generator / GUI 등) — `RTC_TRACE_SCOPE` 는 C++ 매크로.
* **`rtc_communication::Transceiver::RecvLoop`** — production 사용처 0. 첫 실사용
  driver 도입 시 계측.

계측점 추가는 해당 함수 첫 줄에 `RTC_TRACE_SCOPE("Name");` 한 줄. RAII 라
early-return 에도 span 이 닫힌다. build flag OFF 면 그 줄은 사라진다.
런타임 게이트는 여전히 LTTng 세션 — `enable_tracing:=false` 로 실행하면 span 은
컴파일돼 있어도 기록되지 않는다.

> **빌드 전환 주의**: `RTC_ENABLE_TRACING` 을 ON↔OFF 로 바꿀 때 colcon 이 소스
> 재컴파일을 건너뛸 수 있다 (define 변경을 dependency 로 안 봄). 확실히 전환하려면
> `--cmake-clean-first` 또는 `./build.sh ... --clean`. opt-in 은 **타겟별 PRIVATE
> define** 이라, rtc_base 만 ON 으로 빌드해도 OFF 소비자는 계측되지 않는다
> (인터페이스 누출 없음).

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
./repo_scripts/scripts/timeline.sh --max-duration-s 5   # 앞 5초만 변환 (대용량 캡처)
./repo_scripts/scripts/timeline.sh --all-threads    # 외부 프로세스 요약 해제
# '-' 로 시작하는 플래그는 ctf_to_chrome_trace 에 그대로 전달된다 (--help 참조)
```

### JSON 크기 — Perfetto UI 가 안 열리거나 레인이 비어 보일 때

Chrome-Trace JSON 은 이벤트당 ~100 B 로 부피가 크고, Perfetto UI 는 파일 전체를 브라우저 탭에 올린다. **실측: traced sim 30 초 = 2.9 M events / 295 MB** (kernel event 를 켠 실기 캡처는 이보다 더 커진다 — sched_switch 가 지배적).

metadata (프로세스·스레드 이름) 는 파일 **앞**에, 슬라이스는 **뒤**에 있다. 그래서 뷰어가 큰 파일을 끝까지 못 읽으면 **그룹은 보이는데 레인이 비는** 상태가 되고, 이는 변환기 버그나 캡처 결손과 겉보기가 같다. 변환기는 250 MB 초과 시 이 실패 모드를 경고한다.

해결은 **뷰가 아니라 캡처를 좁히는 것**:

```bash
# 1순위: 창 잘라 보기 — 모든 lane 유지, 크기·변환시간 선형 감소
#   실측: 295 MB / 2m6s  →  (--max-duration-s 5)  47 MB / 21s, Cpu lane 12개 그대로
./repo_scripts/scripts/timeline.sh --max-duration-s 5

# 2순위: 필요한 프로세스만
./repo_scripts/scripts/timeline.sh --focus-proc integrated_rt

# 3순위: 캡처 자체를 좁힌다 (rtc:* span 만 필요하면 kernel event 끄기)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true trace_events_kernel:=
```

`timeline.sh` 가 babeltrace2 (또는 `python3-bt2` binding) 로 CTF 를 읽어 Chrome trace JSON 으로 변환한다. 출력은 `<trace_dir>/trace.json`.

변환 진행상황은 stderr 에 찍힌다 — 선택된 파서 (`parser: python3-bt2 binding` vs 느린 `babeltrace2 CLI`), 대용량 트레이스 순회 중 10 만 이벤트마다 `parsed N events ...`, 그리고 `writing N events → <out>`. 대형 트레이스 변환이 멈춘 게 아니라 진행 중임을 이 로그로 확인한다.

* https://ui.perfetto.dev 에 JSON drag-drop. 설치 0, 브라우저만.
* Swimlane 그룹 (위→아래 정렬 순):
  - **Workspace 프로세스 (focus tier)** — `rtc:span_*` 을 emit 한 프로세스
    (= `RTC_TRACE_SCOPE` 계측이 있는 workspace 바이너리) 는 실제 vpid 기준
    process 그룹으로 표시되고, thread 별 row 를 가진다. RT tick 주기성, MPC
    주기를 시간축에서 직접 확인. `rtc:*` span (rt_control_tick → CM::Compute →
    …ComputeControl → sub-step) 중첩 스택도 이 레인에 (위 §RT trace spans).
  - **Cpus** — `Cpu 000` ~ 코어별 sched_switch 기반 timeline. 슬라이스 라벨은
    `프로세스명/comm-tid` (vpid context 로 소속 프로세스를 prefix; main thread 는
    comm 자체가 프로세스명이므로 bare `comm-tid`). ApplyThreadConfig pinning 의
    의도대로 rt_control 이 RT 코어에 고정됐는지, migration 이 없는지 검증.
  - **외부 프로세스 요약** — `rtc:*` span 이 없는 UST 프로세스 (예: vendor
    driver 노드) 는 프로세스당 async lane 1 개로 축약된다 (`<proc> (summary)`).
    callback 슬라이스는 살아있지만 thread 별 row 를 차지하지 않는다.
    per-thread 로 되돌리려면 `--all-threads`, 특정 프로세스만 focus 로 승격하려면
    `--focus-proc <substr,...>`.
  - **Cpu/IRQ** — IRQ handler entry/exit 만 분리. RT core 에 IRQ leak 이 있는지
    한눈에.
* sched_switch 로만 보이는 스레드 (커널 태스크, 비-UST 시스템 프로세스) 는 thread
  row 를 만들지 않는다 — Cpu lane 에서만 보인다.
* Focus 분류는 data-driven (`rtc:span` 존재 여부) 이다. tracing OFF 빌드 캡처처럼
  span 이 하나도 없으면 분류 신호가 없으므로 전 프로세스 per-thread 로 fallback
  하고 stderr 에 안내를 찍는다 — 이때 `--focus-proc` 으로 수동 지정 가능.
* Perfetto UI tip: 좌측 swimlane 헤더 클릭 → 그 lane 만 펼침. 마우스 휠로 zoom,
  shift+드래그로 범위 측정. Ctrl+F 으로 callback symbol 검색.
* Callback 슬라이스의 symbol 은 `ros2:rclcpp_callback_register` 이벤트 (기본 UST
  캡처에 포함) 로 해석된다 — register 가 없는 캡처는 `callback@0x...` 주소로 표시.

### Event loss — 트레이서가 이벤트를 버린 경우

LTTng 의 per-CPU 링 버퍼는 **discard 모드**가 기본이다. 버퍼가 `lttng-consumerd`
가 비우는 속도보다 빨리 차면 트레이서는 이벤트를 그냥 **버리고**, 버린 개수만
다음 packet header 에 기록한다. 살아남은 이벤트는 멀쩡해 보이므로 *census (뭐가
들어왔나)* 로는 절대 감지되지 않는다 — 유일한 신호가 이 카운터다.

변환기는 이 카운터를 읽어 유실이 있으면 stderr 에 경고한다 (버린 총 개수, 전체
대비 비율, **CPU 별 최다 유실**). 유실은 per-CPU 버퍼에서 나므로 어느 Cpu 레인을
믿으면 안 되는지가 그대로 나온다.

```
[ctf_to_chrome] WARNING: the tracer DISCARDED 3,505,417 events (56.8% of 6,174,930) — ...
[ctf_to_chrome]   worst streams: Cpu 003=962,814, Cpu 009=519,818, ...
```

**왜 중요한가** — Cpus 레인은 `sched_switch` 를 재생해서 그리므로, switch 하나가
유실되면 **그 CPU 의 다음 switch 까지 color bar 가 통째로 사라진다.** 반면 그
스레드 자신의 `rtc:*` span 은 (다른 채널·훨씬 낮은 rate) 계속 정상으로 그려진다
→ **"스레드는 분명히 연산 중인데 CPU 에는 아무것도 없다"** 로 보인다. 구멍을
가로지르는 슬라이스는 *누락*이 아니라 **duration 이 틀린 채로** 그려진다.

**대응 (효과 순)**

1. **캡처를 좁힌다** — `trace_events_kernel:=sched_switch` 처럼 이벤트를 줄인다.
   Cpus 뷰는 `sched_switch` 만으로도 완전히 재구성된다.
2. **런을 짧게** — 유실은 부하 구간에 집중된다.
3. **버퍼 확대** — `Trace(subbuffer_size_kernel=)` (ros2_tracing 기본 128 KB/CPU).
   현재 `make_trace_action` 은 이 인자를 노출하지 않는다 — 필요해지면 배선할 것.

> `--max-duration-s` 사용 시 유실 집계도 **변환한 창 안**만 센다 (census 와 동일 규칙).
> babeltrace2 **CLI fallback** 경로 (`python3-bt2` 미설치 / `--stdin`) 는 이 카운터를
> 복구할 수 없어 **검사 자체를 못 한다** — 그 경로에서 경고가 없다는 건 "유실 없음"이
> 아니라 "확인 안 됨"이다. `python3-bt2` 설치 권장.

### Event drop policy (JSON size)

> 위 *Event loss* 와 헷갈리지 말 것: 이쪽은 변환기가 **의도적으로** 안 그리는
> 것이고, 위는 트레이서가 **사고로** 버린 것이다.

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

`timeline.sh` 는 `-` 로 시작하는 인자를 변환기에 그대로 전달하므로
`./repo_scripts/scripts/timeline.sh --keep-all` 처럼 위 플래그들을 직접 쓸 수 있다.

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

자세한 사용은 ros2_tracing 공식 문서 참조. (RT-specific manual tracepoint 는 `RTC_TRACE_SCOPE` 로 구현됨 — §RT trace spans.)

## Permissions

`install.sh --tracing` 이 사용자를 `tracing` 그룹에 추가한다. 그래도 다음 조건이 필요:

| 경로 | 요구 권한 |
|---|---|
| UST tracing (`ros2:*` 만) | 그룹 불필요 — user-space tracepoint |
| Kernel tracing (`sched_switch` 등) | `tracing` 그룹 멤버십 **또는** sudo |
| lttng-modules-dkms 빌드 | `linux-headers-$(uname -r)` + `build-essential` (apt 가 알아서) |

`enable_tracing:=true` + kernel events 가 EPERM 으로 실패하면 → `groups` 출력 확인 → `newgrp tracing` 또는 logout/login.

## Limits

* **자동 모드**로는 콜백 내부 함수 breakdown 도, RT pthread tick 경계도 안 보인다
  (rclcpp executor 콜백이 아니므로). `--tracing` 빌드 + `RTC_TRACE_SCOPE` 계측이
  이를 해결한다 (§RT trace spans) — 현재 RT control tick (CM dispatch + read/publish/
  write/watchdog L2) / WBC·Joint·Task Compute sub-step, MPC solve tick, MuJoCo
  sim_step + viewer frame, hand UDP CommLoop + failure_detector tick, nrt_publish
  drain, 그리고 executor 콜백 본문 (device backend / target / log / hand cmd) 까지
  커버. Pinocchio FK / ProxSuite QP 등 더 깊은 구간은 그 함수에 `RTC_TRACE_SCOPE`
  한 줄을 추가하면 즉시 스택에 나타난다.
* `rtc:*` span 은 **컴파일타임 opt-in** (`-DRTC_ENABLE_TRACING=ON`) — 켜지 않은
  빌드에는 존재하지 않는다. 기본 프로덕션 빌드는 RT hot-path 비용 0.
* Trace 파일 크기는 캡처 시간 + UST/kernel event volume 에 선형. 1 분 캡처 ~ 10–100 MB.
  너무 길게 캡처하면 `lttng-sessiond` 가 디스크 부담을 받음.
* `lttng-modules` 가 빌드되지 않은 환경 (DKMS 실패 / 비표준 kernel) 에서는 kernel
  events 가 캡처되지 않는다. `trace_events_kernel:=` 로 빈 값 주면 UST only 로 fallback.
