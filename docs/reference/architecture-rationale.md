# Architecture — 근거·이력·현황

> **이 문서는 헌법이 아니다.** 구조에 대한 규범 (무엇이 성립해야 하는가, 어떤 코드가 RT 인가) 은 [agent_docs/architecture.md](../../agent_docs/architecture.md) 가 갖는다. 여기는 그 구조가 지금 모양이 된 이력과 결정의 근거다. 둘이 어긋나면 architecture.md 가 옳다. priority·core 번호·POD 소유 헤더·파일 위치는 SSoT (`repo_scripts/config/thread_layout.yaml`, `rtc_base/types/types.hpp` 등) 를 직접 본다.

## Core Data Types

controller 별 상태 (`GraspStateData` · `WbcStateData` · `ToFSnapshotData`) 는 `ControllerOutput` 이 아니라 각 controller 가 소유한 `SeqLock` 으로 나간다. Hand 도메인 상수 (`kNumHandMotors`, `kMaxFingertips`) 는 rtc_base 에 두면 ARCH-1 위반이라 `udp_hand_driver` 가 소유한다.

### `efforts` lane 계약의 결정 이력 (#447 → PR #450)

- 소비자 (예: momentum observer) 를 command mode 로 게이팅하는 방향은 sim 결함을 실기 제약으로 굳히므로 반려됐고, sim 의 gravcomp 누락을 producer 에서 고쳤다.
- UR 은 모터 전류를 보고하며 (실측 2026-08-27) 전 관절을 덮는 변환 상수가 없어 **변환하지 않기로 결정**했다 (2026-09-12, `#502` close). 그래서 momentum observer 는 `iiwa7_leap` 만 `enabled: true` 다 — 근거는 각 프로필의 `demo_shared.yaml` 이 소유한다 (sim/실기 축이 아니라 로봇 축이다: sim 의 UR 도 꺼 둔다).

## Threading Model

### 생성 구조 (issue #153 M1)

SSoT 는 `repo_scripts/config/thread_layout.yaml` 이고 C++ tier 상수, shell 헬퍼, Python launch 미러가 거기서 생성된다. 그 전에는 같은 표가 실행 코드 6곳에 손으로 인코딩돼 있었고 그중 5곳에 직접 테스트가 없었다.

### layout 이력

v4.1 이후 (v4.1 · v5 #349 · v6 #383) 는 `thread_layout.yaml` 의 버전 주석이 소유한다. 여기는 yaml 에 없는 v3 → v4 만 둔다.

- **v3 → v4**: 별도 `rt_outbound` jthread + `publish_buffer_` SPSC + eventfd 를 제거하고 `rt_control` 이 tick 종료 시점에 `DeviceBackend.WriteCommand` 를 직접 호출한다.

mpc_worker 슬롯 회수 (#380) 는 [rt-setup-rationale.md](rt-setup-rationale.md) 가, `DegradationMode` 삭제 (#379) 는 `cpu_topology.hpp` 주석이 갖는다. 거기 없는 한 가지 — "MPC 를 돌리되 전용 코어 없이" 같은 제3 상태는 profile 이 role 을 drop 만 하므로 role 을 *수정*하는 신규 manifest 기능이고 E-7 이다.

- **`mpc` role 의 두 tenant (E-7 승인)**: 포구 계획기 스레드 (`CatchingPlannerThread`, `DemoCatchingController` 소유) 가 `SelectThreadConfigs().mpc.main` 을 그대로 받는다. 한 코어에서 하나만 돈다는 것은 controller switch 테스트 `test_catching_mpc_role_switch` + sim 실측으로 확인했다. Aligator 의 OpenMP 워커도 생성 스레드 이름 `mpc_main` 을 물려받으므로 `verify_rt_runtime.sh` 가 고른 TID 가 유휴 워커일 수 있다.
- **Launch profile (issue #350)**: `enable_mpc:=false` 로 띄우면 profile `mpc_off` 가 된다. 런타임 자동 감지가 불가능한 이유 — `mpc.enabled` 는 controller YAML 이고 스레드는 `on_activate` 에서 뜨므로 세션 중 controller switch 로 켜질 수 있다. profile 불일치를 첫 side effect 전에 거부하는 이유 — controller manager 는 실패한 target 에 `on_deactivate` 를 부르지 않으므로 그 앞에서 활성화된 것은 영구히 반쪽으로 남는다. tier 표 자체는 profile 과 무관하므로 C++ 상수·Python 미러는 불변이다. GRUB `nohz_full`/`rcu_nocbs` 가 profile 을 타지 않는 이유 — boot-static 이라 반영하려면 재부팅이 필요하고, 그러면 재부팅 없는 profile 전환이 깨진다 (결정 D11(a)).
- **hand-private thread (issue #345)**: `hand_comm` 는 `SystemThreadConfigs` 에 필드가 없는 package-local 설정이고, 옛 `taskset -a` 전-스레드 스윕은 aux slot 의 `hand_aux_io` 를 도로 끌어오므로 제거됐다 — 근거는 `rtc_tools/launch/pinning.py` 주석이 소유한다.
- **arm_driver (issue #343)**: 제어 루프의 핀 방식 (taskset 불가, CM 파라미터) 은 [rt-setup-rationale.md](rt-setup-rationale.md) 와 `cpu_shield.sh` 헤더가 갖는다.

Hybrid-CPU 감지 + BIOS 체크리스트: [rt-optimization.md](../rt-optimization.md#6-hybrid-cpu-nuc).

### Per-thread timing CSV

로거는 `ResolveRunId()` (`$RTC_RUN_ID` → `getpid()`) 를 매 행에 찍어 한 세션 디렉토리 안의 두 기동을 가른다 (#376). 컬럼 의미와 판독은 [docs/testing.md](../testing.md) §Live Debug Topics.

## RtControllerNode

- **E-STOP log · status publish 의 defer (#198 Phase 3)**: `TriggerGlobalEstop` / `ClearGlobalEstop` 은 RT loop 에서 도달 가능하므로 atomic flag 만 세우고 `nrt_logging` 의 `DrainLog()` 가 실제 publish 를 한다. lifecycle 콜백은 `FlushEstopStatus()` 로 간접 요청한다.
- **CM 은 RobotTarget sub 을 만들지 않는다 (issue #138)** — manager-target 경로는 폐기됐다. controller YAML 에 `ownership:` field 가 없는 것도 같은 이슈다.
- **output validation (#196 Phase 2b)** 과 **E-STOP substitution (#198 Phase 3)** 은 카운터를 따로 둔다.

## Session logs

레거시 singular `controller/` (CM RT-loop DataLogger) 는 Phase C 에서 제거됐다 — 더 이상 생성하지 않는다. controller data CSV 는 `SpscQueue` 를 감싼 `ThreadCsvProducer<Pod, N>` 를 쓴다.

## Dependency Graph — 엣지의 이력

- **`rtc_controllers` → `rtc_tsid` (2026-09-20)**: catch-pose IK task step 이 `QPSolverWrapper` 경유 box-constrained QP 다. `rtc_tsid` 는 `rtc_controllers` 를 의존하지 않으므로 순환은 없지만, `rtc_controller_manager` 를 포함한 `rtc_controllers` 의 모든 소비자가 ProxSuite 를 끌어온다.
- **`rtc_tools` → `rtc_controllers` exec_depend (2026-09-21)**: `catchability_map` 이 `rtc_controllers` 의 `catch_pose_ik_batch` 를 subprocess 로 부른다 — 오프라인 지도와 런타임 계획기가 같은 판정 함수를 써야 하기 때문이다. exec 전용이고 `rtc_controllers` 는 `rtc_tools` 를 의존하지 않으므로 순환은 없다. 2026-09-22: `catch_gate_map` 도 같은 이유로 `catch_gate_batch` 를 부른다.

## Controller Manager — E-STOP · command type 근거

사용자가 보는 동작은 [rtc_controller_manager/README.md](../../rtc_controller_manager/README.md) 가, 구현 상세는 `rt_controller_node.hpp` · `device_backend.hpp` Doxygen 이 갖는다. 여기는 그 동작이 그 모양이 된 이유다.

- **해제 검증 창 = `watchdog_check_divisor_ + 2` tick.** 해제 뒤 원인 detector 가 한 번은 돌아야 검증이 성립하고, 가장 느린 detector 인 디바이스 워치독은 `watchdog_check_divisor_` tick 마다만 돈다. `reset_fault` 의 2 tick 을 그대로 쓰면 워치독이 아직 안 돌아 죽은 디바이스가 "복구됨" 으로 보고된다.
- **창 동안에도 hold 를 유지한다 (#588 ③).** 래치를 먼저 내리고 창 안의 재 latch 를 기다리던 구현은, 원인이 남아 거부될 해제에서도 그 사이 tick (워치독 원인이면 ~12 tick) 동안 컨트롤러 출력을 치환 없이 내보내고 `/system/estop_status` 에 false→true 토글을 남겼다. 그래서 래치를 내리기 전에 검증 토큰을 세우고, 토큰은 서비스가 아니라 RT 가 창을 채운 뒤에 내린다 (서비스 timeout 으로 팔이 검증 없이 풀리는 경로 차단). 창이 lifecycle 전이로 사라진 것을 검증으로 읽지 않도록 RT 가 끝까지 돈 창의 토큰을 `estop_verified_token_` 에 남기고 서비스는 자기 토큰일 때만 `ok=true` 로 답한다 (#608).
- **`kRetriggered`.** clear 는 래치를 든 채 전파하므로 그 사이 trigger 는 CAS 실패로 사유도 전파도 남기지 않고 삼켜진다. CAS 성공 시에만 세는 카운터로는 원리적으로 못 잡으므로 `estop_trigger_requests_` 를 CAS 앞에서 올리고, 전파 전후 값이 달라졌으면 해제를 포기한다 (fail-closed). "래치 up + 컨트롤러 cleared" 는 머무는 상태로는 스스로 낫지 않는다. 이 경로에 걸릴 수 있는 것은 가드가 없는 `consecutive_overrun` / `sim_sync_timeout` 뿐이다.
- **`/system/estop_status` 는 transient_local (#588), 발행자는 생성 시 현재 값을 한 번 발행 (#607).** 값이 바뀔 때만 발행하므로 volatile writer 로는 latch 뒤에 뜬 구독자가 다음 전이까지 NORMAL 을 봤다. `on_error` 가 latch 를 올린 채 발행자를 없애면 새 발행자는 보관 표본이 없다.
- **hold 목표는 latch 다 (#588 ①, Q1).** 매 tick 목표를 그 tick 의 측정으로 다시 잡으면 position servo 의 정상상태 오차만큼 목표가 측정 쪽으로 따라가 누적된다 (sim 실측: E-STOP 137 s 동안 손 thumb_mcp 0.46 rad, 팔 wrist_2 41 s 동안 0.049 rad). slot 단위로 첫 치환 tick 에 캡처하는 이유: switch 가 E-STOP 검사 직후 trigger 를 만나면 device index → slot 대응이 hold 도중 바뀐다.
- **E-STOP 치환을 CM 이 한다.** actuator 안전이 각 컨트롤러 E-STOP hook 구현의 정확성에 의존해서는 안 된다. in-tree 바인딩이 전부 `SetHandEstop` 을 override 한다는 것은 오늘의 집합에 대한 관찰이지 계약이 아니다 (base 는 no-op). in-tree `ComputeEstop` 의 `safe_position_` slew 는 버린다 — 그 후퇴는 컨트롤러 단위 정책이고 CM 은 검증할 모델이 없다. 치환은 output validation **뒤에** 얹히며, 둘이 같은 hold 를 내므로 카운터를 갈라 두지 않으면 어느 가드가 막았는지 테스트가 구분하지 못한다.
- **`kTorque` hold 는 0 N·m (sag).** CM 은 동역학 모델이 없어 중력 보상 토크를 합성할 수 없다. 노출 창 (`invalid_output_estop_ticks_`, 10-tick floor) 은 compile-time 상수 `kOutputRejectEstopSeconds` 로 정해져 설정으로 줄일 수 없고, 줄이면 래치가 걸리는 시점이 바뀌므로 E-8 결정이다. 그래서 torque 페어링이 허용될 때 configure 가 실측 창을 WARN 으로 고지한다. 진짜 해답은 drive-disable 을 선언하는 backend 의 `WriteSafeCommand()` 다.
- **Backend command type 의 기본값이 position-only 인 이유.** `WriteCommand` 의 `command_type` 인자는 권고였다. 한 backend 가 그것을 무시하고 받은 값을 전부 position forwarding 토픽으로 발행하면 torque 모드 컨트롤러의 N·m 가 관절 각도로 wire 에 나가고, CM 의 hold (kTorque 는 0.0) 는 팔을 놓아주는 대신 zero configuration 으로 이동시킨다. 그 backend 소스는 이 페어링이 YAML 시점에 검증된다고 적었으나 검증하는 곳이 없었다. 선언하지 않은 backend 는 torque 를 fail-closed 로 거부하므로 torque 구성은 "YAML 한 단어 + opt-in 한 backend" 다. 전송 판정 (거부) 과 안전 판정 (경고) 은 답이 독립이라 한 walk 에서 따로 낸다.
- **tick-path 검사의 근거.** configure 게이트는 컨트롤러가 매 tick 채우는 `devices[i].command_type` 을 못 본다. WBC 가 hand 에 `kPdFeedforward` 를 per-device 로 거는 경로로 `kTorque` 가 지나가면 위 결함이 재개봉된다. `AcceptsCommandType` 은 "Not RT" 라 tick 에서 부를 수 없으므로 slot 당 3비트 마스크로 캐시하고 (`CacheSlotCommandTypeMasks`), 두 게이트가 같은 술어를 읽게 한다. 통과 기준은 backend 예외표가 아니라 `commands[]` 값의 의미다 — `kPosition` 과 `kPdFeedforward` 는 위치 목표를 싣고 feedforward 는 추가 채널이므로 graceful fallback 으로 허용하고 wire 문자열도 강등하지 않는다 (강등은 컨트롤러 의도를 rosbag 에서 지운다). `num_channels == 0` 인 device 는 판정하지 않는다 — 판정하면 침묵한 device 하나 때문에 출력 전체가 hold 로 치환되고 창이 지나면 글로벌 E-STOP 이다. hold 치환이 정답인 이유는 `BuildHoldOutput` 이 per-device override 를 지우고 configure 가 이미 대조한 controller-global 타입을 stamp 하기 때문이다. consecutive 카운터를 output validation 과 공유하는 것은 사유가 번갈아 나타나며 지속되는 것도 똑같이 안전 이벤트이기 때문이다.
- **거부 로그는 `RCLCPP_*_THROTTLE` 가 아니라 pending 플래그 + tick 시계 deadline.** THROTTLE 창은 매크로 전개점의 static 이라 프로세스 전역이고 한 프로세스의 두 노드가 서로를 침묵시킨다. 플래그만으로는 `DrainLog` 10 ms 주기에 초당 ~100줄이 되고, E-STOP 이 걸린 뒤에도 검증이 계속 돌아 그 상한이 유지되므로 `loop_count_` 기반 재출력 deadline (`kCommandTypeLogIntervalSeconds`) 을 둔다. 사유 buffer 는 읽은 **뒤에** 재무장한다 — 먼저 풀면 다음 tick 의 writer 가 포맷 중인 배열에 `snprintf` 를 걸어 서로 다른 거부의 섞인 줄이 나온다. teardown 이 `drain_timer_` 를 놓기 전에 이 lane 을 flush 하는 것도 같은 이유다 (안 하면 다음 활성화의 첫 drain 이 일어나지 않은 거부를 보고한다).
- **`WriteSafeCommand()` 는 pure virtual, 인자 없음.** CM 이 부르는 시점은 normal command 를 못 보내고 hold 도 못 만들 때 (device 가 non-finite 위치를 보고) 다. zero-length command 는 fail-closed 가 아니다 — backend 가 early-return 하면 "하던 것을 계속하라" 가 되는 침묵이고 그것은 안전 상태가 아니라 결정의 부재다. 인자가 없는 이유는 그 순간 CM 의 device 관점이 신뢰 불가이고 known-good 값을 든 유일한 당사자가 backend 이기 때문이다. `SafetyMode` enum 을 두지 않은 것도 같은 이유다. default 구현을 두지 않은 것은 침묵을 그대로 재생산하기 때문이다.
- **sim tick 은 sim step 마다 한 번 (#566).** sim 은 device group 마다 joint state 를 따로 발행하므로 메시지마다 tick 하던 구현은 group 이 2 개인 씬에서 한 step 에 tick 을 최대 2 회 돌렸다 (실측 838 Hz / 500 Hz step, `state.dt` 를 적분하는 법칙이 sim 보다 ~1.67× 빨리 흘렀다). 지금은 콜백이 slot 별 sequence 를 올리고 wait 는 `sim_sync_tick_devices` 의 모든 slot 이 전진했을 때만 깬다. 두 SCHED_FIFO 스레드가 mutex 를 공유하면 RT-4 위반이라 atomic + eventfd (counter 라 wake edge 유실 없음) 만 쓴다.
- **`DeviceStateCache` → `DeviceState` 는 필드 열거 복사다.** 두 POD 는 다른 패키지 소유의 별개 타입이라 한쪽에 필드를 추가하면 `ReadDeviceState` 의 복사에도 한 줄이 필요하다 — 빠뜨려도 빌드와 ingress 테스트는 green 이고 컨트롤러만 영구 zero-init 값을 읽는다 (`hole_mask` 처럼 zero 가 "이상 없음" 인 필드는 침묵이 곧 기능 소실). 이음매를 건너는 테스트는 `test_rt_loop_pipeline.cpp` 의 `PerSlotFreshnessReachesTheControllerUnclipped` · `PerLaneFreshnessReachesTheControllerUnclipped` · `SensorLaneReceiptReachesTheControllerPerGroup` 이며, 새 필드는 같은 모양의 단언을 함께 넣는다 (그룹 1 을 읽고, 세 lane 에 서로 다른 값을 쓴다).
- **rt_callback lane 의 `t_total_us` 는 콜백 본문 시간이다.** 스코프가 사용자 콜백 첫 문장에서 열리므로 executor dispatch 는 밖이다. `Σ t_total_us / 관측창` 은 slot 2 duty 의 하한이며 `/proc/<tid>/stat` 의 utime+stime 과 교차 검증한다.
- **컨트롤러 생성은 3-pass.** Pass 1 이 `PreConfigure` (YAML 만 파싱, 리소스 없음), Pass 2 가 device-name 해석, Pass 3 이 `on_configure` 다 — 컨트롤러가 `RegisterLog<>` 람다에 `joint_state_names` 를 capture 하므로 `on_configure` 시점에 이름이 채워져 있어야 CSV header 폭이 맞다. 컨트롤러 노드는 `use_global_arguments(false)` 가 필수다 — 부모 프로세스의 CLI remap (`-r __node:=...`) 이 글로벌 컨텍스트에 살아 있어 default 로 만들면 자식 이름이 부모로 덮이고 launch_ros 가 부모에 spurious ACTIVATE 를 또 보낸다.
- **eventfd 는 device backend 해제 후 close.** backend 의 state-lane sub 은 `on_deactivate` 로 죽지 않고 그 state-ready 콜백이 이 fd 를 쓴다.
