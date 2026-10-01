# Architecture — 근거·이력·현황

> **이 문서는 헌법이 아니다.** 구조에 대한 규범 (무엇이 성립해야 하는가, 어떤 코드가 RT 인가) 은 [agent_docs/architecture.md](../../agent_docs/architecture.md) 가 갖는다. 여기는 그 구조가 지금 모양이 된 이력, 결정의 근거, 기록 시점의 수치·위치다. 둘이 어긋나면 architecture.md 가 옳다. core 번호·priority·파일 위치는 기록 시점의 것이며 SSoT 는 각 항목이 가리키는 YAML·헤더다.

## Core Data Types — 소유 현황 (기록 시점)

| POD | Owner header | 의미 |
|---|---|---|
| `DeviceState` / `ControllerState` / `ControllerOutput` | `rtc_base/types/types.hpp` | Framework-wide RT trivially-copyable POD. ControllerOutput 에 `grasp_state`/`wbc_state`/`tof_snapshot` 필드 없음 — controller-owned SeqLock 으로 이관 |
| `rtc::grasp::GraspStateData` | `rtc_controllers/grasp/grasp_state.hpp` | Force-PI 데모 (DemoJoint/DemoTask) 전용. 각 controller 가 자체 `SeqLock<GraspStateData>` 소유 |
| `integrated_bringup::WbcStateData` | `integrated_bringup/controllers/wbc/wbc_state.hpp` | TSID 데모 (DemoWbc) 전용. Controller 자체 `SeqLock<WbcStateData>` |
| `integrated_bringup::ToFSnapshotData` | `integrated_bringup/controllers/tof_snapshot.hpp` | ToF 거리 + tip pose snapshot |
| Hand 도메인 상수 (`kNumHandMotors`, `kMaxFingertips`) | `udp_hand_driver/udp_hand_constants.hpp` | rtc_base 에 두면 ARCH-1 위반이므로 hand 도메인 소유 |

필드 list / capacity 값은 위 헤더 직접 참조 (`grep -n 'struct.*Data' <header>`).

### `efforts` lane 계약의 결정 이력 (#447 → PR #450)

- 소비자 (예: momentum observer) 를 command mode 로 게이팅하는 방향은 sim 결함을 실기 제약으로 굳히므로 반려됐고, sim 의 gravcomp 누락을 producer 에서 고쳤다.
- UR 은 모터 전류를 보고하며 (실측 2026-08-27) 전 관절을 덮는 변환 상수가 없어 **변환하지 않기로 결정**했다 (2026-09-12, `#502` close). 그래서 momentum observer 는 `iiwa7_leap` 만 `enabled: true` 다 — 근거는 각 프로필의 `demo_shared.yaml` 이 소유한다 (sim/실기 축이 아니라 로봇 축이다: sim 의 UR 도 꺼 둔다).

## Threading Model

### 생성 구조 (issue #153 M1)

SSoT 는 `repo_scripts/config/thread_layout.yaml` 이고 C++ tier 상수 + `SelectThreadConfigsForCoreCount()` (`thread_config_generated.hpp`), shell 헬퍼 (`repo_scripts/scripts/lib/thread_layout_generated.sh`), Python launch 미러 (`rtc_tools/rtc_tools/launch/thread_layout_generated.py`) 가 거기서 생성된다. 그 전에는 같은 표가 실행 코드 6곳에 손으로 인코딩돼 있었고 그중 5곳에 직접 테스트가 없었다. `SystemThreadConfigs` 구조체 정의와 런타임 wrapper `SelectThreadConfigs()` 는 각각 `thread_config.hpp` / `thread_utils.hpp` 에 남는다. 4/6/8/10/12/14/16-core 레이아웃을 자동 선택한다.

### 기록 시점의 priority · core

```
90 rt_control (Core 1)  >  70 rt_callback (Core 2 + DDS recv co-pin)  >  60 mpc_main (Core 3)
```

`nrt_logging` 은 SCHED_OTHER nice -5, `nrt_callback` / `nrt_publish` 는 SCHED_OTHER nice 0 이다. `hand_udp_recv` 는 FIFO 65, `arm_driver` 의 제어 루프는 FIFO 50 이다.

### layout 이력

- **v3 → v4**: 별도 `rt_outbound` jthread + `publish_buffer_` SPSC + eventfd 를 제거하고 `rt_control` 이 tick 종료 시점에 `DeviceBackend.WriteCommand` 를 직접 호출한다.
- **v4.1**: ≥ 6-core 모든 tier 에서 nrt_logging / nrt_callback / nrt_publish 가 Core 0 와 분리. `rt_callback` (FIFO) 이 DDS receive thread (CFS) 와 같은 코어를 공유 — launch-time taskset 이 controller process 의 비-RT thread (DDS / aux) 만 `rt_callback` core 로 다시 핀해서 cache locality 를 확보한다. SCHED_FIFO 가 CFS 를 무조건 선점하므로 RT 결정성은 영향 없다. core 번호는 tier-aware (`rtc_tools.launch.thread_layout.get_rt_callback_core()`). sim_thread / viewer 의 cpu_core=-1 sentinel 은 모든 tier 에서 "no pin" (cpu_shield --sim 모드에서 격리 해제된 코어 사용). hybrid CPU 분기는 `physical_core_slots` 추상화가 처리한다 — 별도 hybrid config 없음.
- **v5 (#349)**: 세 CFS lane 이 전용 코어 대신 Core 2 (aux slot, `rt_callback` 과 동거) 에 얹힌다 — 실기 실측에서 두 lane 의 합산 duty 가 한 자릿수 % 라 동거가 정당화됐다 (수치는 #349). 반환된 슬롯은 system cpuset 으로 돌아가고 cset shield 가 RT cluster 로 좁혀진다. 4-core fallback 은 이미 nrt 가 OS slot 을 공유하므로 불변. `nrt_publish` 를 `cfgs.nrt_publish` 로 이름 분리한 것은 #349 D15 — 이전에는 `nrt_callback` 과 이름이 같아 `verify_rt_runtime.sh` 의 name→TID 맵이 하나만 보관했다.
- **#380 — mpc_worker 슬롯 회수**: 10+ tier 가 예약하던 `mpc_worker_0/1` 의 jthread 들은 `ApplyThreadConfig` 호출 직후 반환해 실제로는 아무것도 실행하지 않았고, 어떤 solver 도 그것을 쓸 수 없었다. Aligator 의 병렬화는 OpenMP 이고 OpenMP 런타임이 자기 스레드를 소유·생성하므로 외부 `std::jthread` 핸들을 넘겨받는 API 자체가 없다. 지금은 tier 와 무관하게 RT 그룹이 `rt_control + rt_callback + mpc_main` 셋이라 cset shield 도 그만큼 좁다.
- **#379 — `DegradationMode` 삭제**: Stage A 가 `cpu_topology.hpp` 에 심었던 `DegradationMode { NONE, SERIAL_MPC }` 는 *"P-core worker budget 이 모자라면 Aligator 를 직렬로 떨어뜨린다"* 는 solver 축이었는데, solve 는 이미 영구 단일 스레드이고 `setNumThreads` 호출은 저장소에 0건이다 — 즉 "serial MPC" 는 강등 상태가 아니라 모든 배포의 무조건적 현재 상태여서 어떤 코드도 거기서 분기할 수 없었고, 선언 이후 소비자가 0인 채로 남아 spec 라운드를 두 번 태웠다 (#350 D10, #379). #380 이 manifest `verifier_optional` 을 지우며 박은 규칙 ("재선언이 아니라 재구현") 과 같다. "MPC 를 돌리되 전용 코어 없이" 는 solver 가 아니라 layout 질문이다 — 현 profile 은 role 을 drop 만 하므로 그런 제3 상태는 role 을 *수정*하는 신규 manifest 기능 + E-7 이다.
- **`mpc` role 의 두 tenant (dynamic_catching S6, E-7 결정 J)**: 포구 계획기 스레드 (`CatchingPlannerThread`, `DemoCatchingController` 소유) 가 `SelectThreadConfigs().mpc.main` 을 그대로 받는다. 한 코어에서 하나만 돈다는 것은 R-1 switch 테스트 `test_catching_mpc_role_switch` + sim 실측으로 확인했다. Aligator 의 OpenMP 워커도 생성 스레드 이름 `mpc_main` 을 물려받으므로 `verify_rt_runtime.sh` 가 고른 TID 가 유휴 워커일 수 있다.
- **Launch profile (issue #350)**: `enable_mpc:=false` 로 띄우면 profile `mpc_off` 가 된다. 런타임 자동 감지가 불가능한 이유 — `mpc.enabled` 는 controller YAML 이고 스레드는 `on_activate` 에서 뜨므로 세션 중 controller switch 로 켜질 수 있다. profile 불일치를 첫 side effect 전에 거부하는 이유 — controller manager 는 실패한 target 에 `on_deactivate` 를 부르지 않으므로 그 앞에서 활성화된 것은 영구히 반쪽으로 남는다. tier 표 자체는 profile 과 무관하므로 C++ 상수·Python 미러는 불변이다. GRUB `nohz_full`/`rcu_nocbs` 가 profile 을 타지 않는 이유 — boot-static 이라 반영하려면 재부팅이 필요하고, 그러면 재부팅 없는 profile 전환이 깨진다 (결정 D11(a)).
- **hand-private thread (issue #345)**: `hand_udp_recv` 는 `SystemThreadConfigs` 에 필드가 없고 package-local `kHandUdpRecvConfig` 다. 같은 프로세스는 blocking 파일 I/O 전용 `hand_aux_io` executor 스레드를 aux slot (OS slot) 에 따로 두며, 그 slot 은 ROS param `aux_cpu_slot` (기본 0, shell SSoT `get_os_cores()`) 이다. 옛 `taskset -a` 전-스레드 스윕은 `hand_aux_io` 를 도로 끌어오므로 제거됐다 — `hand_driver` 는 프로세스가 스스로 main 스레드를 핀하고 (`use_cpu_affinity` param 이 그것까지 끈다) launch 는 `rclcpp::init()` 이 노드 생성 전에 만드는 DDS 스레드만 co-pin 한다.
- **arm_driver (issue #343)**: 그 프로세스 (`ros2_control_node`) 의 제어 루프는 main thread 가 아닌 별도 스레드라 taskset 이 닿지 않으므로, upstream `controller_manager` 의 `cpu_affinity`/`thread_priority` 파라미터로 그 루프만 핀한다. 이 값은 `SystemThreadConfigs.arm_driver` 가 아니라 launch 가 생성하는 CM 파라미터 파일이 나른다.

Hybrid-CPU 감지 + BIOS 체크리스트: [NUC_HYBRID_SUPPORT.md](../NUC_HYBRID_SUPPORT.md).

### Per-thread timing CSV — 구성 파일

- `rtc_base/threading/periodic_rt_thread.hpp` — loop/lifecycle base (fixed-frequency loop, `clock_nanosleep(TIMER_ABSTIME)` cadence, overrun detection, per-tick t0~t3 capture)
- `rtc_base/timing/rt_tick_timing_sample.hpp` — unified `RtTickTimingPayload`
- `rtc_base/logging/run_id.hpp` — `ResolveRunId()` (`$RTC_RUN_ID` → `getpid()`); 로거가 매 행에 찍어 한 세션 디렉토리 안의 두 기동을 가른다 (#376)
- `rtc_base/timing/thread_timing_{sample,producer,csv_logger}.hpp` — generic transport

채널별 출력: `cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv` / `rt_callback_timing_log.csv`, 모두 `<session>/timing/` 아래. 한 launch 의 모든 프로세스가 같은 `run_id` 를 받으므로 채널 간 join 이 성립한다. 컬럼 의미와 판독은 [docs/testing.md](../testing.md) §Live Debug Topics.

## RtControllerNode

- **on_cleanup 의 eventfd 순서 (issue #224)**: backend 의 state-lane sub 은 `on_deactivate` 뒤에도 살아 있고 그 state-ready 콜백이 eventfd 에 쓴다 — 그래서 eventfd 는 device backend 보다 **뒤에** 닫는다.
- **state-ready 콜백이 mailbox 전용이 된 것 (issue #198 Phase 2)**: slot 별 dirty bit + eventfd write 만 하고 digital-twin republish 는 `nrt_publish_thread` 의 `DrainDigitalTwin()` 이 수행한다.
- **E-STOP log · status publish 의 defer (#198 Phase 3)**: `TriggerGlobalEstop` / `ClearGlobalEstop` 은 RT loop 에서 도달 가능하므로 atomic flag 만 세우고 `nrt_logging` 의 `DrainLog()` 가 실제 publish 를 한다. lifecycle 콜백은 `FlushEstopStatus()` 로 간접 요청한다.
- **CM 은 RobotTarget sub 을 만들지 않는다 (issue #138)** — manager-target 경로는 폐기됐다. controller YAML 에 `ownership:` field 가 없는 것도 같은 이슈다.
- **output validation (#196 Phase 2b)** 과 **E-STOP substitution (#198 Phase 3)** 은 카운터를 따로 둔다.
- executor 배선 구현: [rt_controller_main_impl.cpp](../../rtc_controller_manager/src/rt_controller_main_impl.cpp). controller-owned sub/pub 생성: `integrated_bringup/src/support/owned_topics.cpp`. default-group 계약을 잠그는 테스트: `integrated_bringup/test/test_controller_target_cb_group_invariant.cpp`.

## TF `_actual` 프레임

bare `tf2_ros::TransformListener` 가 `_actual` 프레임을 못 받는 함정은 digital_twin tcp_viz · bt_coordinator 두 곳에서 각각 silent-fail 버그로 발현했다. 기록 시점의 소비자: self-feed 는 ur5e_bt_coordinator `transforms_sub_`, demo_gui `_transforms_cb`; `/tf` 재발행은 `rtc_digital_twin` 의 `controller_tf` (`<active>/transforms` → restamp → `/tf`, default on).

## Session logs

`logging_data/YYMMDD_HHMM/{timing,monitor,device,sim,plots,motions,tracing}/`. Per-controller logs 는 `controllers/<config_key>/` (예: `demo_wbc_controller/mpc_solve_timing.csv`). 레거시 singular `controller/` (CM RT-loop DataLogger) 는 Phase C 에서 제거됐다 — 더 이상 생성하지 않는다. controller data CSV 는 `SpscQueue` 를 감싼 `ThreadCsvProducer<Pod, N>` 를 쓴다 (Phase C).

## Dependency Graph — 엣지의 이력

- **`rtc_controllers` → `rtc_tsid` (2026-09-20, dynamic_catching D-26)**: catch-pose IK task step 이 `QPSolverWrapper` 경유 box-constrained QP 다. `rtc_tsid` 는 `rtc_controllers` 를 의존하지 않으므로 순환은 없지만, `rtc_controller_manager` 를 포함한 `rtc_controllers` 의 모든 소비자가 ProxSuite 를 끌어온다.
- **`rtc_tools` → `rtc_controllers` exec_depend (2026-09-21)**: `catchability_map` 이 `rtc_controllers` 의 `catch_pose_ik_batch` 를 subprocess 로 부른다 — 오프라인 지도 (S3.5a) 와 런타임 계획기 (S6.2) 가 같은 판정 함수를 써야 하기 때문이다. exec 전용이고 `rtc_controllers` 는 `rtc_tools` 를 의존하지 않으므로 순환은 없다. 2026-09-22: `catch_gate_map` 도 같은 이유로 `catch_gate_batch` (S3.5b) 를 부른다.
- **`rtc_mpc`**: CMake-only 의존 (fmt >= 10, aligator — source-installed) 은 `package.xml` 에 선언돼 있지 않다.
