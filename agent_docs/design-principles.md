# Design Principles for `rtc_*` Packages

`rtc_*` packages are the **robot-agnostic** backbone of this framework. Any modification must preserve this property. Robot-specific logic, hardware assumptions, and fixed-shape constants belong in `ur5e_*` packages. When in doubt: *"Would this code still make sense on a 7-DOF arm with a 2-finger gripper?"*

> **이 파일의 규칙 위반은 [invariants.md](invariants.md) ARCH-1~4의 escalation 대상이다.** 위반이 불가피하다고 판단될 때는 코드를 쓰기 **전에** [CLAUDE.md](../CLAUDE.md) §6 포맷으로 `[CONCERN] Severity: Warning` 이상을 보고한다.

## Five Principles

1. **Extensibility** -- Adding a new robot, new DOF count, new transport, or new controller must require **zero source edits** inside `rtc_*`. Achievable via: (a) YAML config, (b) `RTC_REGISTER_CONTROLLER` from downstream, or (c) implementing an abstract interface.

2. **Generality** -- No robot names, joint counts, finger counts, topic names, or hardware identifiers hardcoded in `rtc_*`. Use YAML-injected values, template parameters, or runtime config. Constants like `kNumRobotJoints=6` are **upper-bound capacity**, not per-robot assumptions. Names describe the *role* (`num_joints`), never the *robot* (`ur5e_joints`).

3. **Modularity** -- Respect the dependency graph. Never introduce upward dependencies (e.g., `rtc_base` depending on `rtc_controllers`). Never cross-link siblings the graph doesn't connect. If a feature spans packages: (a) push abstraction down to a shared base, or (b) invert via interface injection.

4. **Interface-first** -- New functionality with multiple implementations MUST define an **abstract class, concept, or pure-virtual interface** before any concrete implementation. Follow: `RTControllerInterface`, `TransportInterface`/`PacketCodec`, `InferenceEngine`, `TaskBase`/`ConstraintBase`, `PhaseManagerBase` (`rtc_mpc` FSM boundary; concrete FSMs live downstream). Concrete classes register via factory/registry -- never `#ifdef` or hardcoded switches.

5. **Deduplication & Reuse** -- Before writing utilities, search existing `rtc_*`:
   - Lock-free, filters, logging, threading -> `rtc_base`
   - URDF, Pinocchio -> `rtc_urdf_bridge`
   - Transport, UDP, codecs -> `rtc_communication`
   - ONNX -> `rtc_inference`
   - QP tasks, constraints -> `rtc_tsid`
   If existing doesn't quite fit, **generalize it** -- don't fork.

## Boundary Rules (`rtc_*` vs `ur5e_*`)

| Belongs in `rtc_*` | Belongs in `ur5e_*` |
|--------------------|---------------------|
| Abstract interfaces, concepts, base classes | Concrete implementations via `RTC_REGISTER_CONTROLLER` |
| DOF-generic algorithms (variable `n_joints`) | Fixed-DOF launch files, URDF, MJCF, meshes |
| Transport/codec templates (`Transceiver<T,C>`) | Robot-specific packet structs as template args |
| YAML-driven parameter schemas | YAML files with actual robot values |
| Controller registry, TSID solver core | Demo controllers, BT coordinator, bringup |
| RT threading, SPSC, SeqLock, E-STOP logic | Hardware drivers (UR5e RTDE, hand UDP, ToF UART) |
| **Library only** — `rtc_*` 패키지는 자체 `add_executable` / `main()` 보유 금지. `RtControllerMain()` 등 진입 함수만 export | **Runtime identity 소유** — robot-specific 패키지가 `add_executable`로 exec 생성, `RtControllerMain(argc, argv, "<exec_name>")` 호출. ROS 노드 이름 = exec 이름 (예: `integrated_rt_controller`) |
| Agnostic launch는 자기 노드 단독만 띄움 (예: `rtc_mujoco_sim/launch/mujoco_sim.launch.py` → `mujoco_simulator_node`만). robot-specific 패키지 의존 / robot-specific exec 호출 금지 | 통합 launch (RT 컨트롤러 + 시뮬레이터 + 드라이버 chain)는 robot bringup이 소유 (예: `integrated_bringup/launch/sim.launch.py`) |

**Runtime identity rule** (위 마지막 두 행의 근거): exec ↔ ROS 노드 ↔ pgrep ↔ logger 식별자가 모두 같은 이름으로 정렬되어야 디버깅·검증·로그 추적 비용이 일정하게 유지됨. agnostic 패키지가 자체 exec를 가지면 robot-specific identity와 혼선이 생김(어느 launch에서 띄운 어느 instance인지 구분 불가). 따라서 agnostic은 library만 export.

## Two-Tier Topic Ownership (Phase 4)

Non-RT ROS I/O is split into two tiers based on RT adjacency and per-controller schema requirements:

- **Manager-owned** (`TopicOwnership::kManager`, default): RT-adjacent traffic — device state, joint/ros2 commands, state/sensor logs, digital-twin republishers. Lives on `RtControllerNode` (CM). Topic paths are stable across controller switches because the hardware protocol does not change.
- **Controller-owned** (`TopicOwnership::kController`): external GUI / BT / planner traffic whose schema or QoS may differ per controller. Two flavors: (a) PublishRole-mapped (`kRobotTarget`, `kRobotTransforms`, `kDigitalTwinState`) — declared in controller YAML `topics:`; (b) controller-private SeqLock (Grasp/Wbc/ToF) — controller owns `SeqLock<T>` + `Setup*Publisher` helper, no PublishRole/YAML entry. Both flavors are created on a per-controller `rclcpp_lifecycle::LifecycleNode` whose namespace is `/<config_key>`; relative YAML paths auto-resolve to `/<config_key>/<topic>`.

CM's publish thread drains the SPSC snapshot for manager-owned roles and then calls `controllers_[active]->PublishNonRtSnapshot(snap)` to delegate controller-owned publishing. External consumers (BT bridge, GUIs, digital_twin, shape_estimation) subscribe to `/rtc_cm/active_controller_name` (TRANSIENT_LOCAL, single-CM scope per locked decision D-A2) and rewire their sub/pubs on each transition. The CM never decides which namespace is authoritative — it exposes the current choice; everything else is pull-based. Logs (`device_state_log`, `device_sensor_log`) remain manager-owned for now; may move later when per-controller schema stabilises.

## When Generalization Requires a Design Change

If you cannot satisfy all five principles with a local edit, STOP and:
1. Report a `[CONCERN] Severity: Warning` ([CLAUDE.md](../CLAUDE.md) §6 포맷)
2. Propose an interface refactor or dependency inversion as a separate task
3. Do NOT embed robot-specific logic in `rtc_*` "for now"

### Example

```
[CONCERN] rtc_controllers/CLIK에서 hand joint 인덱스가 6~15로 하드코딩 필요
Severity: Warning
Detail: UR5e + 10-DOF hand 조합에서 task 공간 CLIK이 hand 6~15 번 joint를
  null-space에 넣어야 하는데, rtc_* 는 robot-agnostic이어야 하므로
  ARCH-1 위반. 현재 rtc_controllers는 "total DOF" 만 알고 sub-chain
  분할 정보는 없음.
Alternative:
  1) SubChain 구성을 rtc_urdf_bridge에서 YAML 주입 (선호)
  2) CLIK에 `null_space_indices: [int]` 파라미터 추가 후 integrated_bringup
     YAML에서 주입
  3) integrated_bringup에 별도 CLIKWithHand 컨트롤러를 파생해 robot-specific
     지식을 외부로 뽑아냄
권고: (1) — 다른 robot/hand 조합에서도 재사용 가능
```
