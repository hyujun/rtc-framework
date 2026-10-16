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

- **`mpc` role 의 두 tenant (dynamic_catching S6, E-7 결정 J)**: 포구 계획기 스레드 (`CatchingPlannerThread`, `DemoCatchingController` 소유) 가 `SelectThreadConfigs().mpc.main` 을 그대로 받는다. 한 코어에서 하나만 돈다는 것은 R-1 switch 테스트 `test_catching_mpc_role_switch` + sim 실측으로 확인했다. Aligator 의 OpenMP 워커도 생성 스레드 이름 `mpc_main` 을 물려받으므로 `verify_rt_runtime.sh` 가 고른 TID 가 유휴 워커일 수 있다.
- **Launch profile (issue #350)**: `enable_mpc:=false` 로 띄우면 profile `mpc_off` 가 된다. 런타임 자동 감지가 불가능한 이유 — `mpc.enabled` 는 controller YAML 이고 스레드는 `on_activate` 에서 뜨므로 세션 중 controller switch 로 켜질 수 있다. profile 불일치를 첫 side effect 전에 거부하는 이유 — controller manager 는 실패한 target 에 `on_deactivate` 를 부르지 않으므로 그 앞에서 활성화된 것은 영구히 반쪽으로 남는다. tier 표 자체는 profile 과 무관하므로 C++ 상수·Python 미러는 불변이다. GRUB `nohz_full`/`rcu_nocbs` 가 profile 을 타지 않는 이유 — boot-static 이라 반영하려면 재부팅이 필요하고, 그러면 재부팅 없는 profile 전환이 깨진다 (결정 D11(a)).
- **hand-private thread (issue #345)**: `hand_udp_recv` 는 `SystemThreadConfigs` 에 필드가 없는 package-local 설정이고, 옛 `taskset -a` 전-스레드 스윕은 aux slot 의 `hand_aux_io` 를 도로 끌어오므로 제거됐다 — 근거는 `rtc_tools/launch/pinning.py` 주석이 소유한다.
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

- **`rtc_controllers` → `rtc_tsid` (2026-09-20, dynamic_catching D-26)**: catch-pose IK task step 이 `QPSolverWrapper` 경유 box-constrained QP 다. `rtc_tsid` 는 `rtc_controllers` 를 의존하지 않으므로 순환은 없지만, `rtc_controller_manager` 를 포함한 `rtc_controllers` 의 모든 소비자가 ProxSuite 를 끌어온다.
- **`rtc_tools` → `rtc_controllers` exec_depend (2026-09-21)**: `catchability_map` 이 `rtc_controllers` 의 `catch_pose_ik_batch` 를 subprocess 로 부른다 — 오프라인 지도 (S3.5a) 와 런타임 계획기 (S6.2) 가 같은 판정 함수를 써야 하기 때문이다. exec 전용이고 `rtc_controllers` 는 `rtc_tools` 를 의존하지 않으므로 순환은 없다. 2026-09-22: `catch_gate_map` 도 같은 이유로 `catch_gate_batch` (S3.5b) 를 부른다.
