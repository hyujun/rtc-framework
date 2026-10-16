# Tracing — 계측 계약·판정 근거

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/testing-debug.md](../../agent_docs/testing-debug.md) §Tracing 이 갖고, 사용법은 [docs/tracing.md](../tracing.md) 가 갖는다. 여기는 `RTC_TRACE_SCOPE` span 을 *어떤 계약으로* 넣는지, 주기성을 *어떻게 판정하는지*, 무엇을 *왜 빼는지* 를 기록한다. 둘이 어긋나면 규범이 옳다. span 이름·계층은 계측 코드가 바뀌면 달라지므로 적지 않는다 (`grep -rn RTC_TRACE_SCOPE` 로 다시 센다).

## 2-layer span 계약

규칙은 [testing-debug.md](../../agent_docs/testing-debug.md) §Tracing 이 갖는다. L1 은 raw `clock_nanosleep` 또는 `jthread` 의 한 iteration 경계와 일치시킨다. L3 이상은 비용·분기가 큰 경로에만 두는 의도된 예외다:

| 예외 L3 | 위치 |
|---|---|
| WBC | `ComputeControl` 아래 CLIK/TSID QP sub-step (kinematic / dynamic, FSM 경로별 mode), `UpdatePhase` grasp FSM |
| MPC | `mpc_solve` 아래 `mpc_handler_solve` (aligator ProxDDP), `mpc_mode_swap` |
| MuJoCo | `sim_step` 아래 `sim_substep` → `mj_step` (외부 physics solver), one-shot `HandleReset` |
| backend RT I/O | `CM::ReadDeviceState` / `CM::WriteCommand` 아래 per-backend `ReadState` / `WriteCommand` |

예외 근거 없이 2-layer 밖으로 깊이 내려가지 않는 이유는, span 하나가 RT hot path 에서 비용이고 flame 스택이 깊어질수록 사람이 읽을 수 없게 되기 때문이다.

executor 콜백은 ros2_tracing 의 `ros2:callback_start/end` 가 이미 바깥을 L1 으로 감싼다. 그래서 콜백 **본문**에는 이름 붙은 L2 span 만 더한다.

## Cadence 판정

판정 규칙은 [testing-debug.md](../../agent_docs/testing-debug.md) §Tracing 이 갖는다. 지표는 목표 주기 대비 간격의 평균 · 분산 · 최댓값 (worst overrun) 이다. `sim_step` 의 physics 비용은 `sim_wait_command` 와 `ThrottleIfNeeded` 를 뺀 나머지다 (sim 은 lock-step 이라 controller 명령을 기다린다).

## span 이 없거나 따로 놓인 자리 (의도)

span 트리는 `grep -rn RTC_TRACE_SCOPE` 로 재생성되므로 적지 않고, 계측 누락처럼 보이는 자리의 의도만 적는다.

* `CM::CheckTimeouts` 는 `rt_control_tick` 의 자식이 아니라 같은 레인의 sibling 이다. `ControlLoop()` 반환 **후** `OnTick` 에서 실행되기 때문이다.
* `CM::PublishHandoff` 는 CSV 가 못 가르는 자리를 가르려고 둔 span 이다 (#222). `cm_timing_log.csv` 의 `t_publish_us` 는 이 handoff 끝에서 끊기므로, 잔차 (`t_total_us` − 세 phase 합) 는 어느 span 도 덮지 않는 구간, 곧 스레드가 돌지 못한 시간이다. 긴 tick 을 만나면 CSV 잔차로 tail 인지 판별하고, tail 이면 trace 에서 그것이 `CM::PublishHandoff` **안** (68 KB 스냅샷 복사) 인지 **뒤** (스케줄 손실) 인지 본다. 실측 표본에서 `CM::WriteCommand` 가 28 µs 인 tick 의 3.4 ms 가 전부 span 바깥이었고, 같은 시각 `nrt_callback` 레인의 10 Hz 로그 drain 이 7.9 ms 돌고 있었다.
* hand tail 끝에 publish span 이 없는 것은 의도다 (#345). publish 는 **pull** 이라 tick 은 SeqLock 에 store 만 하고 NRT executor 가 `state_sequence()` 를 폴링해 가져간다. publish 비용은 executor 스레드에서 찾는다.
* MPC solve 는 **단일 스레드**다. `mpc_worker_*` placeholder 는 #380 이 제거했으므로 worker span 이 없는 것은 계측 누락이 아니다. 병렬 MPC 를 도입하면 병렬성은 aligator 의 OpenMP 풀 안에 있어 계측 지점도 그 풀이 된다.
* `owned_topics_publish` 는 WBC/Task/Joint 세 컨트롤러가 공유하는 helper 안에 있어 controller-owned non-RT publish 전체를 한 span 으로 귀속한다. active controller 는 하나뿐이라 컨트롤러별 wrapper 를 두지 않는다.
* RT-path 의 `Read*` / `WriteCommand` 는 RT tick 의 SeqLock load/store 이고, executor 의 `On*State` 콜백은 non-RT 의 SeqLock write 라 **서로 다른 쪽**이다.

## 계측 제외 (의도적)

| 제외 대상 | 이유 |
|---|---|
| DDS / rclcpp 내부 스레드 | kernel `sched_switch` timeline 으로 관측한다 (UST span 불필요) |
| 프로세스 main 스레드 | 노드 구성 후 50 ms sleep 루프만 도는 대기 스레드 |
| 일시적 lifecycle 스레드 | `on_configure` / `on_activate` 1회성 경로 |
| python 노드 (target generator, GUI 등) | `RTC_TRACE_SCOPE` 는 C++ 매크로 |
| `rtc_communication::Transceiver::RecvLoop` | production 사용처가 없다. 첫 실사용 driver 가 들어올 때 계측한다 |
