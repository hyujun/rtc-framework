# Tracing — 계측 계약·판정 근거

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/testing-debug.md](../../agent_docs/testing-debug.md) §Tracing 이 갖고, 사용법은 [docs/tracing.md](../tracing.md) 가 갖는다. 여기는 `RTC_TRACE_SCOPE` span 을 *어떤 계약으로* 넣는지, 주기성을 *어떻게 판정하는지*, 무엇을 *왜 빼는지* 를 기록한다. 둘이 어긋나면 규범이 옳다. span 이름과 계층은 **기록 시점의 것**이라 계측 코드가 바뀌면 달라진다 (`grep RTC_TRACE_SCOPE` 로 다시 센다).

## 2-layer span 계약

모든 주기 스레드는 **L1 root span 1개**와 그 하위 **L2 span** 으로 계측한다.

* L1 은 tick 전체다. raw `clock_nanosleep` 또는 `jthread` 의 한 iteration 경계와 일치시킨다.
* L2 는 주요 phase 다 (read / compute / publish / write).
* L3 이상의 깊은 분해는 **의도된 예외**이며 비용·분기가 큰 경로에만 허용한다.

| 예외 L3 | 위치 |
|---|---|
| WBC | `ComputeControl` 아래 CLIK/TSID QP sub-step (kinematic / dynamic, FSM 경로별 mode), `UpdatePhase` grasp FSM |
| MPC | `mpc_solve` 아래 `mpc_handler_solve` (aligator ProxDDP), `mpc_mode_swap` |
| MuJoCo | `sim_step` 아래 `sim_substep` → `mj_step` (외부 physics solver), one-shot `HandleReset` |
| backend RT I/O | `CM::ReadDeviceState` / `CM::WriteCommand` 아래 per-backend `ReadState` / `WriteCommand` |

새 계측점은 먼저 "L2 인가, 위 예외에 드는 L3 인가" 를 판정한다. 예외 근거 없이 2-layer 밖으로 깊이 내려가지 않는 이유는, span 하나가 RT hot path 에서 비용이고 flame 스택이 깊어질수록 사람이 읽을 수 없게 되기 때문이다. 트리의 들여쓰기가 곧 계층이다.

## Cadence 판정

주기성은 각 L1 root span (`rt_control_tick`, `sim_step`, `hand_comm_tick`, `mpc_tick`) 의 **begin timestamp 간격 (begin-to-begin)** 으로 판정한다. span *duration* 이 아니라 *시작 시각* 의 주기성이 기준이다.

* 지표는 목표 주기 (`control_rate` YAML 등) 대비 간격의 평균 · 분산 · 최댓값 (worst overrun) 이다.
* 같은 session_dir 의 CSV timing/overrun 로그는 **보조 근거**로 병기한다. trace 의 begin-to-begin 과 CSV 의 tick period 가 어긋나면 계측 경계나 캡처 결손 (event loss) 을 먼저 의심한다.
* `sim_step` 은 duration 에 command 대기 (`sim_wait_command`) 와 throttle (`ThrottleIfNeeded`) 이 섞인다. cadence 는 begin-to-begin 으로만 보고, physics 비용은 그 두 span 을 뺀 나머지로 해석한다. `sim_step` duration 을 physics 실행 시간으로 단독 해석하지 않는다.
* `sim_wait_command` 가 길면 sim 물리가 느린 것이 아니라 controller 의 RT tick 이 느린 것이다 (sim lock-step 이라 controller 명령을 기다린다).

## 스레드별 span 트리 (기록 시점)

RT 제어 스레드 (`integrated_rt_controller` 의 `rt_control`):

```
rt_control_tick
├─ CM::ReadDeviceState        └ <Backend>::ReadState / ReadMotorState / ReadSensorState  (L3, SeqLock load)
├─ CM::Compute                └ <Controller>::Compute → ComputeControl → (WBC sub-step, UpdatePhase)
├─ CM::FillPublishSnapshot
├─ CM::WriteCommand           └ <Backend>::WriteCommand  (L3)
└─ CM::PublishHandoff         (nrt_publish_buffer_ push + eventfd)
CM::CheckTimeouts             (10 tick 마다. rt_control_tick 의 자식이 아니라 같은 레인의 sibling)
```

* `<Backend>` 는 `UrDriverNativeBackend` / `UdpHandNativeBackend` / `MujocoNativeBackend` 다. 이 RT-path `Read*` / `WriteCommand` 는 RT tick 의 SeqLock load/store 이고, 아래 Executor L2 의 `On*State` 는 non-RT executor callback 의 SeqLock write 라 **서로 다른 쪽**이다.
* `CM::CheckTimeouts` 가 sibling 인 이유는 `ControlLoop()` 반환 **후** `OnTick` 에서 실행되기 때문이다.
* WBC `Compute` 아래에는 per-tick 헬퍼 span (`ReadState` / `DrainTargetSlot` / `WriteJointCommand` / `FillLogOutput` / `FillPublishOutput` / `ComputeEstop` 등) 도 찍힌다. `DemoJointController` / `DemoTaskController` 도 같은 패턴이며 **active 일 때만** 발생한다.
* `CM::PublishHandoff` 는 CSV 가 못 가르는 자리를 가르려고 둔 span 이다 (#222). `cm_timing_log.csv` 의 `t_publish_us` 는 이 handoff 끝에서 끊기므로, 잔차 (`t_total_us` − 세 phase 합) 는 어느 span 도 덮지 않는 구간, 곧 스레드가 돌지 못한 시간이다. 긴 tick 을 만나면 CSV 잔차로 tail 인지 판별하고, tail 이면 trace 에서 그것이 `CM::PublishHandoff` **안** (68 KB 스냅샷 복사) 인지 **뒤** (스케줄 손실) 인지 본다. 실측 표본에서 `CM::WriteCommand` 가 28 µs 인 tick 의 3.4 ms 가 전부 span 바깥이었고, 같은 시각 `nrt_callback` 레인의 10 Hz 로그 drain 이 7.9 ms 돌고 있었다.

나머지 스레드는 이름과 계층만 적는다.

| 스레드 | 트리 |
|---|---|
| `sim_thread` (`mujoco_simulator_node`) | `sim_step` → `MuJoCoSimulator::Read*` / `Invoke*Callback` · `sim_wait_command` · `ApplyCommand` · `sim_substep` ×n (`PreparePhysicsStep` → `mj_step` → `ClearContactForces`) · `ReadSolverStats` / `UpdateRtf` / `ThrottleIfNeeded`. `HandleReset` 은 reset 요청 시만 나오며 pre-publish reset 이면 `sim_step` sibling, post-wait reset 이면 child |
| `viewer` | `viewer_frame` → `viewer_sync` · `viewer_render` · `viewer_overlays` · `viewer_swap` (pacing sleep 제외) |
| `hand_comm` (`udp_hand_node`) | `hand_comm_tick` → `hand_drain_stale` · `hand_write_echo` · `hand_read_*` (bulk 또는 individual) · `hand_comm_tail` → `hand_sensor_postproc` / `hand_ft_infer`. fake 모드는 `hand_fake_cycle` → `hand_fake_step`. abort 시 `hand_estop_zero_write` 가 tick 의 sibling 으로 1회 |
| `failure_detector` (50 Hz aux) | `hand_detector_tick` → `hand_detector_check` / `_rate` / `_link` (sleep 제외, 켜진 검사만) |
| `mpc_main` | `mpc_tick` → `mpc_read_state` · `mpc_solve` (→ `mpc_mode_swap`, `mpc_handler_solve`) · `mpc_publish` |
| `nrt_publish` | `nrt_publish_drain` → `owned_topics_publish` (poll 대기 제외) |

* hand tail 끝에 publish span 이 없는 것은 의도다 (#345). publish 는 **pull** 이라 tick 은 SeqLock 에 store 만 하고 NRT executor 가 `state_sequence()` 를 폴링해 가져간다. publish 비용은 executor 스레드에서 찾는다.
* MPC solve 는 **단일 스레드**다. `mpc_worker_*` placeholder 는 #380 이 제거했으므로 worker span 이 없는 것은 계측 누락이 아니다. 병렬 MPC 를 도입하면 병렬성은 aligator 의 OpenMP 풀 안에 있어 계측 지점도 그 풀이 된다.
* `owned_topics_publish` 는 WBC/Task/Joint 세 컨트롤러가 공유하는 helper 안에 있어 controller-owned non-RT publish 전체를 한 span 으로 귀속한다. active controller 는 하나뿐이라 컨트롤러별 wrapper 를 두지 않는다.

## Executor callback 내부 L2

ros2_tracing 의 `ros2:callback_start/end` 가 각 executor 콜백 바깥을 이미 L1 으로 감싼다. 그래서 콜백 **본문**에는 named L2 span 만 더해 어느 lane 이 무슨 일을 하는지 본다. 콜백 심볼은 `ros2:rclcpp_callback_register` 로 해석된 L1 슬라이스 안에 이 L2 가 중첩된다.

| Lane (priority) | L2 span | 하는 일 |
|---|---|---|
| `rt_callback` (FIFO 70) | `MujocoNativeBackend::OnJointState` / `OnWrench`, `UrDriverNativeBackend::OnJointState`, `UdpHandNativeBackend::OnJointState` / `OnMotorState` / `OnSensorState` | device state SeqLock write |
| `nrt_callback` | `CM::TargetCb` | RobotTarget reorder + SPSC marshal |
| `nrt_logging` | `CM::DrainLog` | timing-CSV drain + summary print |
| hand node executor | `hand_joint_command_cb` | JointCommand name-reorder + offset + send |
| `mpc_timing_cb_group` (1 Hz aux timer) | `DemoWbcController::LogMpcSolveTimingTick` | MPC solve timing-CSV drain + 10 s aggregate INFO |

## 계측 제외 (의도적)

| 제외 대상 | 이유 |
|---|---|
| DDS / rclcpp 내부 스레드 | kernel `sched_switch` timeline 으로 관측한다 (UST span 불필요) |
| 프로세스 main 스레드 | 노드 구성 후 50 ms sleep 루프만 도는 대기 스레드 |
| 일시적 lifecycle 스레드 | `on_configure` / `on_activate` 1회성 경로 |
| python 노드 (target generator, GUI 등) | `RTC_TRACE_SCOPE` 는 C++ 매크로 |
| `rtc_communication::Transceiver::RecvLoop` | production 사용처가 없다. 첫 실사용 driver 가 들어올 때 계측한다 |

계측점은 함수 첫 줄의 `RTC_TRACE_SCOPE("Name");` 한 줄이다. RAII 라 early-return 에도 span 이 닫히고, build flag 가 OFF 면 그 줄이 사라진다. `RTC_ENABLE_TRACING` 은 타겟별 PRIVATE define 이라 rtc_base 만 ON 으로 빌드해도 OFF 소비자는 계측되지 않는다 (인터페이스 누출 없음). 현재 커버되지 않는 깊은 구간 (Pinocchio FK, ProxSuite QP 등) 은 그 함수에 한 줄을 추가하면 즉시 스택에 나타난다.
