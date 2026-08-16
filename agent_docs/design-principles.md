# Design Principles for `rtc_*` Packages

`rtc_*` packages are the **robot-agnostic** backbone of this framework. Any modification must preserve this property. Robot-specific logic, hardware assumptions, and fixed-shape constants belong in the **integration packages** — today `integrated_bringup`, `udp_hand_driver`, `shape_estimation`, `ur5e_bt_coordinator` (i.e. every non-`rtc_*` package that `<depend>`s on an `rtc_*` one; the hook derives this set from `package.xml` rather than a name glob). When in doubt: *"Would this code still make sense on a 7-DOF arm with a 2-finger gripper?"*

> **이 파일의 규칙 위반은 [invariants.md](invariants.md) ARCH-1~4의 escalation 대상이다.** 위반이 불가피하다고 판단될 때는 코드를 쓰기 **전에** [CLAUDE.md](../CLAUDE.md) §6 포맷으로 `[CONCERN] Severity: Warning` 이상을 보고한다.

## Five Principles

다른 문서는 이 원칙들을 **P1~P5** 로 인용한다 (예: `modification-guide.md` 의 "P1·P2", `invariants.md` 의 "P5"). 아래 번호가 그 ID 다 — `conventions.md` 의 include-priority `P1..P4` 와 compliance 명세의 부호규약 `P2/P3` 는 **다른 네임스페이스**이므로 혼동하지 않는다.

1. **P1 — Extensibility** -- Adding a new robot, new DOF count, new transport, or new controller must require **zero source edits** inside `rtc_*`. Achievable via: (a) YAML config, (b) `RTC_REGISTER_CONTROLLER` from downstream, or (c) implementing an abstract interface.

2. **P2 — Generality** -- No robot names, joint counts, finger counts, topic names, or hardware identifiers hardcoded in `rtc_*`. Use YAML-injected values, template parameters, or runtime config. Constants like `kNumRobotJoints=6` are **upper-bound capacity**, not per-robot assumptions. Names describe the *role* (`num_joints`), never the *robot* (`ur5e_joints`).

3. **P3 — Modularity** -- Respect the dependency graph. Never introduce upward dependencies (e.g., `rtc_base` depending on `rtc_controllers`). Never cross-link siblings the graph doesn't connect. If a feature spans packages: (a) push abstraction down to a shared base, or (b) invert via interface injection.

4. **P4 — Interface-first** -- New functionality with multiple implementations MUST define an **abstract class, concept, or pure-virtual interface** before any concrete implementation. Follow: `RTControllerInterface`, `TransportInterface`/`PacketCodec`, `InferenceEngine`, `TaskBase`/`ConstraintBase`, `PhaseManagerBase` (`rtc_mpc` FSM boundary; concrete FSMs live downstream). Concrete classes register via factory/registry -- never `#ifdef` or hardcoded switches.

5. **P5 — Deduplication & Reuse** -- Before writing utilities, search existing `rtc_*`:
   - Lock-free, filters, logging, threading -> `rtc_base`
   - URDF, Pinocchio -> `rtc_urdf_bridge`
   - Transport, UDP, codecs -> `rtc_communication`
   - ONNX -> `rtc_inference`
   - QP tasks, constraints -> `rtc_tsid`
   If existing doesn't quite fit, **generalize it** -- don't fork.

## Boundary Rules (`rtc_*` vs integration packages)

| Belongs in `rtc_*` | Belongs in an integration package |
|--------------------|---------------------|
| Abstract interfaces, concepts, base classes | Concrete implementations via `RTC_REGISTER_CONTROLLER` |
| **제어 법칙 코어** — Eigen/span in-out, `RTControllerInterface` 를 모름 | **바인딩** — `RTControllerInterface` 구체 구현, mailbox 소비, `ControllerOutput` 조립, 등록 (아래 §`rtc_controllers` Controllers Are Pure Control Algorithms 가 이 행의 SSoT — 3계층 배치표·전이 상태 포함) |
| DOF-generic algorithms (variable `n_joints`) | Fixed-DOF launch files, URDF, MJCF, meshes |
| Transport/codec templates (`Transceiver<T,C>`) | Robot-specific packet structs as template args |
| YAML-driven parameter schemas | YAML files with actual robot values |
| Controller registry, TSID solver core | Demo controllers, BT coordinator, bringup |
| RT threading, SPSC, SeqLock, E-STOP logic | Hardware drivers (UR5e RTDE, hand UDP, ToF UART) |
| **Control-framework runtime identity 금지** (ARCH-7) — `rtc_*` 는 RT 제어 루프를 구동하는 exec 를 소유하지 않는다. `RtControllerMain()` 같은 진입 *함수* 만 export | **Runtime identity 소유** — integration 패키지가 `add_executable`로 exec 생성, `RtControllerMain(argc, argv, "<exec_name>")` 호출. ROS 노드 이름 = exec 이름 (예: `integrated_rt_controller`) |
| Agnostic launch는 자기 노드 단독만 띄움 (예: `rtc_mujoco_sim/launch/mujoco_sim.launch.py` → `mujoco_simulator_node`만). robot-specific 패키지 의존 / robot-specific exec 호출 금지 | 통합 launch (RT 컨트롤러 + 시뮬레이터 + 드라이버 chain)는 robot bringup이 소유 (예: `integrated_bringup/launch/sim_ur5e_p1a.launch.py`) |

**ARCH-7 의 범위 — 무엇이 예외인가**: 이 규칙이 막으려는 것은 *제어 프레임워크의 런타임 정체성* 이 agnostic 패키지로 새는 것이지, `add_executable` 자체가 아니다. 따라서 다음은 명시적 예외다.

- **Robot-agnostic standalone 노드** — `rtc_mujoco_sim` 의 `mujoco_simulator_node`, `rtc_urdf_bridge` 의 `closure_state_publisher`. 둘 다 로봇 이름을 모르고 URDF/MJCF 를 파라미터로 받으며, RT 제어 루프를 소유하지 않는다. 이들의 launch 는 자기 노드만 띄운다.
- **Example 실행 파일** — `rtc_urdf_bridge` 의 `example_*` 4종, `rtc_math` 의 `se3_error_compare`. API 사용법 데모이며 bringup chain 에 등장하지 않는다.

예외에 해당하지 않는 신규 `add_executable` 을 `rtc_*` 에 추가하려면 [invariants.md](invariants.md) §Escalation Triggers 의 `[CONCERN]` (E-1 / Critical — ARCH-7 전용 E-번호는 없다) 으로 보고한다. 이 규칙은 오랫동안 예외 서술 없이 "금지" 로만 적혀 있었고, 그 사이 7개 타깃이 축적되는 동안 아무 신호도 없었다 (#213) — 지금은 hook 이 `rtc_*/CMakeLists.txt` 의 신규 `add_executable` **타깃 이름** 을 검사한다. 면제 경로는 셋이다 — (1) HEAD 에 이미 있는 기존 타깃은 재발화하지 않고(**grandfathered** — 개수는 hook 이 세는 값이므로 박제하지 않는다; `mujoco_simulator_node`·`closure_state_publisher`·`se3_error_compare` 는 이 경로로만 통과한다), (2) `example_*` 는 이름으로 면제되며, (3) **그 밖의 신규 agnostic 노드** 는 `add_executable` 라인 위 또는 옆에 `ARCH-7-exempt` 주석을 달아 면제한다. 즉 hook 이 지금 마커를 강제하는 대상은 (3) 뿐이고 (1)의 세 노드에는 아직 마커가 없다 — 이들을 rename·재추가하면 name diff 가 신규 타깃으로 보므로, 그때 `ARCH-7-exempt` 주석을 함께 붙여 (3) 경로로 옮긴다.

**Runtime identity rule** (위 마지막 두 행의 근거): exec ↔ ROS 노드 ↔ pgrep ↔ logger 식별자가 모두 같은 이름으로 정렬되어야 디버깅·검증·로그 추적 비용이 일정하게 유지됨. agnostic 패키지가 자체 exec를 가지면 robot-specific identity와 혼선이 생김(어느 launch에서 띄운 어느 instance인지 구분 불가). 따라서 agnostic은 library만 export.

## Controller-YAML Topics Are Controller-Owned (Phase 4 → issue #138)

Every topic declared in a controller's YAML `topics:` section is **controller-owned**: created on a per-controller `rclcpp_lifecycle::LifecycleNode` whose namespace is `/<config_key>`, so relative YAML paths auto-resolve to `/<config_key>/<topic>`. These carry external GUI / BT / planner traffic whose schema or QoS may differ per controller. Two flavors: (a) PublishRole-mapped — `kRobotTransforms`, the only value left after issue #196 Phase 5 removed the roles that had no publisher behind them — declared in the YAML; (b) controller-private SeqLock (Grasp/Wbc/ToF) — controller owns `SeqLock<T>` + `Setup*Publisher` helper, no PublishRole/YAML entry.

The other two lanes are **not** controller-YAML topics and are owned elsewhere: device-wire state/command traffic lives in `devices.<group>.backend:` (DeviceBackend-owned); CM's own fixed publishers (`/rtc_cm/<group>/joint_states` digital-twin republish, `/system/estop_status`, `/rtc_cm/active_controller_name`) are hard-coded on `RtControllerNode`, not declared in any YAML. There is no manager-owned controller-YAML tier and no `ownership:` field (issue #138 removed the former `TopicOwnership` selector — controller-owned is the only model).

CM's publish thread drains the SPSC snapshot and calls `controllers_[active]->PublishNonRtSnapshot(snap)` to delegate controller-owned publishing. External consumers (BT bridge, GUIs, digital_twin, shape_estimation) subscribe to `/rtc_cm/active_controller_name` (TRANSIENT_LOCAL, single-CM scope per locked decision D-A2) and rewire their sub/pubs on each transition. The CM never decides which namespace is authoritative — it exposes the current choice; everything else is pull-based.

## `rtc_controllers` Controllers Are Pure Control Algorithms

바로 위 규칙이 controller-owned 토픽이 **어디에 사는가**를 정한다면, 이 규칙은 **누가 그것을 만드는가**를 정한다: `rtc_controllers` 는 **제어 법칙(알고리즘)만** 소유한다. 자기 노드도, publisher 도, subscription 도 만들지 않으며 — 2026-07-26 결정으로 — **`RTControllerInterface` 를 상속하지도 않는다.** ROS 배선은 전적으로 integration 패키지(`integrated_bringup/src/support/owned_topics.cpp` 의 `CreateOwnedTopics()`)가 소유한다.

### 금지되는 것

- **`rtc_controllers` 안의 `RTControllerInterface` 구체 구현.** `class X … : public RTControllerInterface` 가 이 패키지에 **새로** 생기면 안 된다. 프레임워크 계약을 구현하는 클래스 — lifecycle 훅, `Compute(ControllerState) → ControllerOutput`, target mailbox, E-STOP 훅, `Name()` / `config_key` 등록 — 는 **바인딩**이며 integration 패키지가 소유한다. `rtc_controllers` 의 `trajectory/*` · `grasp/*` 를 `integrated_bringup` 의 데모 컨트롤러가 멤버로 들고 쓰는 방식이 그 참조 형태다.
- **컨트롤러가 `RTControllerInterface::get_lifecycle_node()` 로 노드를 받아 자기 pub/sub 을 만드는 것.** 위 상속 금지의 따름정리이지만 별도로 유지한다 — 바인딩 계층에서도 노드 접근은 `CreateOwnedTopics` 경로로만 쓴다. 인터페이스가 노드 접근을 *제공한다*는 사실은 그것을 *써도 된다*는 뜻이 아니다.

### 금지되지 않는 것

- `rclcpp/logging.hpp`. 로깅은 노드를 만들지 않는다.
- 패키지 차원의 `rtc_base` 의존. 코어의 **인자**는 프레임워크-중립 타입이지만, `rtc_base` 까지 끊는 "강" 안은 채택하지 않았다 (issue #236 D-A).

### 코어의 형태

- **입출력은 Eigen / `std::span` 등 프레임워크-중립 타입.** `ControllerState` / `ControllerOutput` 의 해체·조립은 바인딩 몫이다. 참조 구현은 `rtc_controllers/include/rtc_controllers/compliance/` 의 `task_dynamics.hpp` · `impedance_law.hpp` — 이 규칙이 명문화되기 전에 이미 이 형태로 추출됐고, 그래서 규칙의 예시가 아니라 **기준**이다.
- **`Resize()`(off-RT, 할당 허용) / `Compute()`(`noexcept`, heap-free) 분리**를 유지한다 ([invariants.md](invariants.md) §RT Path Invariants).
- YAML 스키마는 코어 옆의 `Params` POD + `ParseXxxParams(YAML::Node)` 자유 함수로 둔다 (yaml-cpp 만 의존, 비-RT). 프레임워크 타입을 참조하지 않으므로 코어와 같은 층에 남는다.
- **제어 법칙은 궤적 생성기를 소유하지 않는다.** 둘 다 코어(위 3계층 표)지만 서로 다른 코어이고, 법칙은 궤적 **샘플**(`ref_pos` / `ref_vel`)을 인자로 받는다. *어느* 궤적이 *어느* 법칙을 먹이는지, 몇 개인지는 **구조 결정**이라 바인딩 몫이다 — `demo_joint_controller` 는 하나의 관절 법칙에 `JointSpaceTrajectory` 2개(arm+hand)를, `demo_wbc_controller` 는 3개를 붙인다. 궤적을 멤버로 든 법칙은 이 구성을 표현할 수 없다. 같은 이유로 duration 휴리스틱(`max_dist / speed`)과 `*_trajectory_speed` 파라미터는 궤적 파라미터화이지 법칙의 게인이 아니다. 참조 구현 `joint/joint_pd_law.hpp`.
- **태스크 공간 법칙은 오차의 *정의* 를 소유하지 않는다.** 6D pose error `e` 를 인자로 받을 뿐, 어느 `rtc::math::se3::ErrorType` 으로 계산할지는 프레임을 소유한 바인딩이 정한다 — 법칙을 Eigen 만으로 유지하고(`rtc_math`·pinocchio 불포함), 같은 선택을 echo 하는 텔레메트리 lane 옆에 결정을 남겨 둔다. 참조 구현 `compliance/impedance_law.hpp` · `task/task_accel_law.hpp` · `task/task_vel_law.hpp`.
- **같은 이유로 프레임 *전송* 도 소유하지 않는다.** 궤적 샘플의 twist 를 어느 회전으로 world-aligned 프레임에 옮길지(`R_trajectory` 인가 `R_current` 인가)는 오차 정의와 같은 범주의 **프레임 결정**이므로, 법칙은 이미 회전된 `ν_ff` 를 받는다. 법칙이 회전행렬을 아예 보지 않으면 잘못된 회전을 조용히 적용할 수 없고, 그 선택은 궤적을 샘플링하는 바인딩 옆에 남는다 (#236 S3a). 곱을 named temporary 로 hoist 하는 것은 비트 단위로 inert 하며 — 누산에 접어 넣는 것은 **아니다** — `test_task_vel_core.cpp` 의 리터럴 oracle 이 hoist 이전 형태를 그대로 들고 그것을 고정한다.
- **법칙은 자기를 먹이는 *헬퍼* 도 알지 않는다 — 그 헬퍼의 *출력* 을 인자로 받는다.** compliance §6.3 관성 성형은 `Λ_S` 를 인자로 받지 `TaskDynamics&` 를 받지 않는다. 헬퍼 타입을 이름으로 아는 법칙은 그 헬퍼를 **어디로 수렴시킬지**의 판정을 선점하며 (여기서는 OSC 인라인 Λ 블록의 흡수 방향 = S2b), 그러면 통합 리팩터가 법칙 시그니처를 바꿔야 끝난다. 인자로 받으면 수렴점이 바뀌어도 법칙은 그대로다 (#236 S4). 참조 구현 `compliance/inertia_shaping.hpp`.
- **무상태 코어가 스크래치를 필요로 하면 저장 용량이 컴파일 타임에 묶인 Eigen 타입**(`Matrix<double,Dyn,Dyn,0,MaxR,MaxC>`)**을 로컬로 둔다** — 호출자에게 버퍼를 빌리지 않는다. 순수 고정 크기(`Matrix<double,6,6>`)는 **논리 크기가 더 작을 때 깨진다** (`LLT` 에 m<6 블록을 주면 `PlainObjectBase::resize` 의 크기 assert 가 걸린다 — 다만 그 assert 는 **`NDEBUG` 에서 컴파일아웃**되고 이 저장소의 기본 빌드가 Release 이므로, abort 는 운 좋은 경우고 실제로는 resize 가 no-op 한 뒤 대입이 목적지 크기로 돌아 **쓰이지 않은 행 위에서 solve 가 진행된다**: 컴파일 통과, fault 없음, 결과만 틀림. #236 D-S4a). max-size 타입은 heap-free 이면서 Eigen 이 `MatrixXd` 와 같은 커널을 고르므로 추출이 비트 단위로 inert 하다 — 다만 그 inert 함은 **가정하지 말고 리터럴 oracle 이 옛 타입을 들고 재는 것**으로 고정한다 (`test_inertia_shaping_core.cpp`). RT-1 센서로는 Eigen 할당 tripwire 가 필수다: 이 타입이 `MatrixXd` 로 퇴화해도 숫자는 같고 operator-new 게이트는 그것을 못 본다.
- **대수적 동치는 형태를 합칠 근거가 못 된다 — 비트 동치를 *도달 가능한 상태에서* 재고 판정한다.** P 형(`kp·Δq`)을 PD 코어(`kp·Δq − kd·q̇`)의 `K_d = 0` 특수화로 흡수하는 것은 대수적으로 자명하지만 비트 단위로는 아니다: `x − 0.0·q̇` 는 유한한 비영 `x` 에 대해 `x` 지만 `x = −0.0` 이고 `q̇ < 0` 이면 IEEE-754 상 `(−0.0) − (−0.0) = +0.0` 으로 부호가 뒤집힌다. 그리고 그 상태는 병적이지 않다 — 자세 기준 `q_ref` 는 측정값으로 seed 되므로 팔이 멈춰 있는 동안 `Δq` 가 정확히 `+0.0` 이고, 음수 `K_p` 면 매 채널이 `−0.0` 이 된다 (그 도달 경로였던 "`LoadConfig` 가 음수를 받는다" 는 #277 이 여섯 소비자 전부에 `rtc::FloorNonNegativeGain` (#280 이전 이름 `joint::FloorPostureGain`) 을 걸어 닫았다 — 법칙 자체는 여전히 아무것도 floor 하지 않으므로 직접 호출자는 그대로 도달한다). 착수 전 probe 가 그 상태에서 draw 의 50% 가 뒤집히는 것을 `-O0`/`-O2`/`-O3` 모두에서 실측했다 (#236 S6) → **형태가 다르면 함수도 다르다** (S3a 의 6축/병진전용 분리와 같은 판정). 그 근거는 세션 probe 로 남기지 말고 **상주 테스트**로 옮긴다 (`test_posture_core.cpp` 의 `TheProportionalFormIsNotThePdFormWithZeroDamping`) — 나중 독자가 반드시 손대려 할 "단순화" 인데, 틀린 이유가 어떤 tolerance 테스트로도 보이지 않는 부호 있는 0 이기 때문이다.
- **레퍼런스의 *출처* 가 다른 것은 법칙이 다른 것이 아니다.** 자세 법칙 `kp·(q_ref − q) − kd·q̇` 는 impedance/cascade 가 활성화 시점의 측정값을, OSC 가 config 의 `safe_position` 을 기준으로 쓴다. 이는 **바인딩 차이**이므로 `q_ref` 를 인자로 받으면 하나의 법칙으로 수렴한다 (#236 S6, `joint/posture_law.hpp`). 반대로 **게인을 곱하는 위치가 다르면 법칙이 다르다**: CLIK 은 사영 *뒤에* 곱하고(`kp·(N·Δq)`) 나머지는 *앞에* 곱하는데(`N·(kp·Δq)`), 대수적으로 같고 비트로는 68% 가 다르다 (probe 실측) — 그 소비자를 같은 코어에 넣는 것은 그 자체로 비-inert 변경이다.
- **두 번째 소비자는 *법칙이 같은가* 로 판정한다 — 기본값·인자의 출처가 같은가로 판정하지 않는다.** 태스크 어드미턴스의 compliance §7.3 속도항은 `ν_ff` 자리에 궤적 twist 대신 compliant frame 속도 `ν_c` 가 들어갈 뿐 CLIK 과 같은 법칙이라, 새 코어를 만들지 않고 `task/task_vel_law.hpp` 를 재사용하는 것으로 판정됐다 (#236 S5, P5 — 당시 소비자였던 `TaskAdmittanceController` 어댑터는 S7c 에서 삭제됐고, 지금 그 재사용 계약을 들고 있는 것은 `test_admittance_task_vel_reuse.cpp` 다). 반대로 **코어 `Params` POD 를 컨트롤러 `Gains` 에 중첩하는 것은 기본값이 같을 때만 중복 제거**다: `ik_kp_*` 는 기본이 의도적으로 2.0 이고 `TaskVelParams` 는 1.0 이라, 중첩해도 member initializer 에 두 번째 정의가 남아 이름만 바뀐다 (D-S5b). 같은 `Gains` 의 `admittance`(= `AdmittanceParams`)·`inertia`(= `InertiaShapingParams`) 가 중첩된 것은 거기서는 기본값이 코어 것과 **같기** 때문이다. 같은 기준으로 `TaskImpedanceController::Gains` 의 느슨한 4배열(기본 200/28/20/9 = `ImpedanceParams` 와 동일)도 중첩으로 접혔다 (#236 S6b) — 그때 매 tick `ImpedanceParams{kp_pos, kd_pos, kp_rot, kd_rot}` 로 **positional 재조립**하던 것이 함께 사라진다. 그 재조립이야말로 중첩이 없앨 진짜 결함이다: 코어의 필드 *순서* 에 조용히 의존하고, 순서가 바뀌어도 컴파일되고 테스트도 통과한다.

### 3계층 배치 (issue #236 D3)

| 계층 | 무엇이 사는가 | 어디에 |
|---|---|---|
| **코어 — 알고리즘** | 제어 법칙, 수치 커널, 궤적 생성기, 상태기계, 파라미터 스키마. Eigen/span in-out | `rtc_controllers` |
| **base — 프레임워크 공통 글루** | 모든 바인딩이 *동일하게* 필요로 하는 것: target mailbox (SPSC + SeqLock + generation), submodel 선택, device 한계값 로드, device 판독가능성 게이트, E-STOP scaffolding | `rtc_controller_interface` |
| **바인딩 — 배치 고유** | `RTControllerInterface` 구체 구현, `ControllerOutput` 조립, `goal_type` 해석, E-STOP 정책, 텔레메트리, `RTC_REGISTER_CONTROLLER`, production YAML | integration 패키지 (`integrated_bringup` 등) |

경계 판정 한 줄: **"이 코드가 `RTControllerInterface` 의 존재를 알아야 하는가?"** — 아니오면 코어, 예이고 모든 컨트롤러에서 같으면 base, 예이고 배치마다 다르면 바인딩.

### 부가 입력은 인자로 받는다

외부 F/T wrench 처럼 device lane 에 없는 입력은 컨트롤러가 구독하는 것이 아니라 **비-RT setter** 로 주입한다 (`SetDeviceTarget` 과 동일 idiom, RT 와의 교환은 `SeqLock`/SPSC 로만 — RT-4). 전송 계층(누가 센서를 읽어 넣어주는가)은 컨트롤러 밖의 관심사다. 값에 딸린 freshness 는 ROS 타임스탬프가 아니라 **generation 카운터 + tick 카운팅**으로 표현한다 (RT 에서 clock 을 읽지 않기 위함; `ControllerState::dt` 사용).

### 현황 — `rtc_controllers` 는 만족한다. 바인딩 계층의 배치 전이도 끝났다

**`rtc_controllers` 쪽은 끝났다.** 이 패키지에는 `RTControllerInterface` 구체 구현이 없고 `package.xml` 도 `rtc_controller_interface` 를 의존하지 않는다 (issue #236 슬라이스 S1–S7: 법칙별 코어 추출 → G1 글루 base 상향 → 상속 클래스 삭제). 수는 여기 박제하지 않는다 — `grep -rn "public RTControllerInterface" rtc_controllers/include` 와 `grep -n rtc_controller_interface rtc_controllers/package.xml` 이 **둘 다 비어야** 한다 (AP-DOC-1).

**바인딩 계층의 배치 전이도 끝났다.** 오래 예로 들던 `demo_task_controller` 는 두 축 모두 수렴했다 — 상수-λ DLS 는 **#282 에서 `compliance::DifferentialIk` 로**, 태스크 속도 법칙 `kp ⊙ e + ν_ff` 는 **#314 에서 [task/task_vel_law.hpp](../rtc_controllers/include/rtc_controllers/task/task_vel_law.hpp) 의 `ComputeTaskVelocity` / `ComputeTranslationVelocity` 로**. 잔여를 세는 기준은 "DLS 사본" 이 아니라 **"코어 대응물이 있는데 호출하지 않는 법칙"** 이고 (PR #313 이 이 기준을 박았다), 그 기준으로 `grep -rn "cwiseProduct\|diagonal().array() +=\|LDLT<\|LLT<\|Jpinv\|pseudoInverse" integrated_bringup/src integrated_bringup/include` 는 이제 **0건**이다. 바인딩에 남은 인라인 법칙 사본은 없다.

호출자가 없는 코어(`ComputeImpedanceForce` · `ComputeJointPdCommand` · `ComputePostureTorque` / `ComputePostureVelocity` · `ComputeTaskAcceleration`)가 남아 있지만 이것은 **다른 상태**다 — S1–S7 이 추출한 뒤 S7c 가 어댑터를 지웠고, 그 법칙을 **인라인으로 다시 쓰는 살아 있는 바인딩이 없다**. 즉 "미수렴 사본" 이 아니라 "바인딩을 기다리는 코어" 이며, 위 기준의 분자에 들어가지 않는다.

**단 이 완료형은 바인딩 계층에 한정한다.** `rtc_tsid` 의 SE3 / object-SE3 / CoM / posture task 는 가속도형 법칙을 자체적으로 쓴다 (`a_ff + kp ⊙ e + kd ⊙ ė`). 이는 바인딩 누수가 아니라 **동급 코어 간 중복** 축이다 — `rtc_tsid` 는 위 P5 가 "QP tasks, constraints" 를 배정한 자기 도메인의 코어이고, 속도 오차 정의부터 다르다 (Jlog6 정확 미분 대 `ν_d − ν`). 따라서 `ComputeTaskAcceleration` 으로의 수렴은 inert 리팩터가 아니라 **법칙 변경**이며, 착수하려면 별도 판단이 필요하다 ([CLAUDE.md](../CLAUDE.md) §6 E-9).

- 규칙은 **새 코드에 즉시 구속**된다. 새 제어 법칙은 코어로 쓰고, 필요하면 바인딩을 integration 패키지에 만든다.
- DLS 축의 코어 수렴은 issue #282 가, 태스크 속도 축은 `#314` 가 **완료**했다. 두 축이 `demo_task_controller` 의 전부였다.
- **이 문서의 근거 문단에 나오는 어댑터 클래스명**(`ClikController` · `TaskImpedanceController` · `TaskAdmittanceController` 등)**은 S1–S7 슬라이스가 판정을 내리던 시점의 대상이다** — 지금 코드에 없다. 그 판정이 왜 그렇게 났는지는 여전히 유효하므로 남겨 두되, 살아 있는 코드로 읽지 않는다.
- 문서는 **"`rtc_controllers` 는 순수하다 / 더 이상 상속하지 않는다"** 와 **"바인딩 계층에 미수렴 법칙 사본이 없다"** 를 둘 다 완료형으로 서술해도 된다 (전자는 2026-07-29, 후자는 `#314` 부터). 여전히 완료형으로 쓰지 **않는** 것은 **"코어 간 법칙 중복이 없다"** 쪽이다 — 위 `rtc_tsid` 축이 열려 있어 쓰는 순간 문서-코드 불일치를 새로 만든다 ([CLAUDE.md](../CLAUDE.md) §6 E-9). 세 문장은 범위가 다르다.

**ARCH-7 과의 구별**: ARCH-7 은 *exec / 런타임 정체성* 소유를 금지한다 (위 Boundary Rules). 이 규칙은 그보다 안쪽으로, exec 를 안 만들더라도 **노드·구독을 만드는 것**과 **프레임워크 인터페이스를 상속하는 것**을 금지한다. 세 규칙은 별개이며 ARCH-7 의 standalone-node 예외(`mujoco_simulator_node` 등)는 여기에 적용되지 않는다 — 그 예외는 robot-agnostic *노드* 패키지에 대한 것이지 컨트롤러에 대한 것이 아니다.

**근거**: 컨트롤러를 순수 알고리즘으로 유지하면 (a) 단위 테스트가 ROS 컨텍스트 없이 성립하고, (b) 같은 법칙이 sim / 실기 / 오프라인 재생에서 배선만 갈아끼워 재사용되며, (c) 배선 결정(QoS, 네임스페이스, 메시지 타입)이 robot bringup 한 곳에 모인다. 상속 금지가 추가된 근거는 (d) — 프레임워크 계약을 구현하는 순간 글루가 법칙과 같은 파일에 들어오고, 그 글루의 대부분은 컨트롤러마다 **동일한 boilerplate** 라서 구현체 수만큼 복제된다. 실제로 mailbox 스켈레톤은 그렇게 복제됐고 (#206), 그 복제본들의 검증 공백에서 결함이 반복해 나왔다. 컨트롤러에 구독을 넣으려는 충동은 대개 "이 입력을 어떻게 넣지?" 에서 나오는데 답은 setter 이지 구독이며, 인터페이스를 상속하려는 충동은 "CM 이 이걸 어떻게 부르지?" 에서 나오는데 답은 바인딩이지 상속이 아니다.

위반이 필요해 보이면 [CLAUDE.md](../CLAUDE.md) §6 `[CONCERN]` (E-1 / Critical) 로 보고한다. *(결정 2026-07-25, issue #236 — impedance/admittance 슬라이스 2의 wrench 입력 경로를 정하며 노드·구독 금지를 명문화. 그 전까지 이 규칙은 코드에만 존재했고 문서에 없어 "컨트롤러가 자기 구독을 만든다"가 유효한 선택지로 검토된 적이 있다. 2026-07-26 개정 — 같은 issue 에서 상속 금지 + 3계층 배치로 강화. 노드 금지만으로는 글루 복제를 막지 못한다는 것이 기존 구현체 전반에서 실측됐기 때문이다. 2026-07-29 개정 — S7c(PR #300) 로 어댑터가 전부 삭제되면서 §현황 을 완료형으로 전환. 완료형 금지 규칙은 폐기가 아니라 **범위 축소**다: `rtc_controllers` 에 대해서만 풀리고 3계층 배치 전체에 대해서는 그대로 산다. 2026-07-29 재개정 — `#314` 가 마지막 인라인 법칙 사본(태스크 속도)을 수렴시키면서 완료형 범위를 **바인딩 계층까지** 넓혔다. 전환 근거는 "코어 대응물이 있는데 호출하지 않는 법칙" 기준의 전수 grep 이 0 이 된 것이고, 남은 금지 범위는 코어 간 중복(`rtc_tsid` 축) 하나다.)*

## Backend / Controller Layering

Within a robot bringup package and its `DeviceBackend` implementations (`mujoco_native`, `udp_hand_native`, `ur_driver_native`, future drivers), the backend ↔ controller boundary is governed by **responsibility**, not by data shape. The topic-ownership rule above governs *who owns a ROS topic*; this rule governs *who computes a value*.

- **Backend = hardware-facing.** Every backend packs the raw values its hardware publishes into `DeviceStateCache` (`state_data` / `motor_data` / `sensor_data` / `inference_data`), filling **all stride slots the layout reserves** regardless of whether the currently-active controller reads them. Unused slots are zero-filled (not skipped) so logging / digital_twin / replay tools see a single SSoT for HW state and a backend swap does not silently change consumer semantics.
- **A backend that cannot fill a joint-position slot must SAY SO — `hole_mask` (#284).** The cache is persistent, so a slot the current message did not reach keeps the previous message's value, and a downstream `num_channels >= model_dim` check cannot see that (`num_channels` is the wire length). The `positions` lane therefore carries a per-slot companion: **bit i set ⇒ slot i was not written by the most recent state message.** The polarity means a backend that never touches the field claims "no holes" and the F5 gate silently degrades to its pre-#284 strength for that device — build green, tests green. Backends that ingest a named `sensor_msgs/JointState` get this for free by going through `WriteJointStateToCache` (`integrated_bringup/include/integrated_bringup/backends/joint_state_reorder.hpp`), which is the shipped path for all three; **anything that fills `positions` by hand owns the mask too.** Field contract: `rtc_base/include/rtc_base/types/types.hpp`; consumer: `rtc::IsDeviceReadable`.
- **Controller = behavior-facing.** Controllers read only the slots they consume (e.g. WBC reads `inference_data[ft_base+1..3]` for fx/fy/fz + `inference_enable[f]`). Derived quantities (`force_magnitude`, `in_contact`, `force_rate`, `slip_rate`) are computed inside the controller from raw inputs — they do **not** appear in `DeviceStateCache`.
- **Controller-owned publish = controller's derived view.** Topics owned by a controller (`WbcState`, `GraspState`, ToF snapshot) carry the controller's behavior view, not a mirror of backend raw. A field that started as a raw mirror but became unconsumed should be derived by the controller as a transitional step; mark with `TODO(layer-d)` (or analogous) and remove in a follow-up ABI revision.
- **Controller command output stops at the actuation quantity, never the electrical quantity.** A controller emits `JointCommand` / `DeviceOutput.commands[]` carrying **position, velocity, or effort(torque)** only (`CommandType ∈ {kPosition, kTorque}`); it never emits motor **current**. Current is a hardware quantity — if a drive is current-controlled, the **backend** (or its driver/firmware) converts τ → I using the motor constants it owns (`k_t`, gear ratio), exactly as it owns the inverse on the read side (the hand UDP response already carries a current channel). This keeps `rtc_msgs` (`JointCommand`) and `rtc_base` (`DeviceOutput`) free of any `goal_current` field and the controller free of per-motor electrical models, so the same controller drives a torque-mode sim and a current-mode real drive unchanged. Hand feedforward torque (τ_ff, Stage C-3) is therefore expressed as effort/torque in the command; the τ → current conversion + wire/firmware mode lives entirely in the hand backend/driver — adding current support to a drive touches neither the controller nor the message ABI. *(Decision 2026-06-11; see [CLAUDE.md](../CLAUDE.md) E-3 — the once-assumed `goal_current` ABI change was withdrawn on this basis.)*

Concrete consequences:

- A new behavior signal (e.g. slip detection) is added in the controller, not the backend. Backend keeps publishing raw force; controller derives `slip_rate`.
- Removing a sensor channel from a backend does not require touching controllers that don't read it; conversely, dropping a derived field from a controller publish ABI does not require touching backends.
- Adding a backend that exposes a subset of the layout (e.g. mujoco fills only fx/fy/fz, leaves contact_flag/displacement zero) is a legitimate sparse backend — controllers that need only the filled slots work unchanged.

This layering pairs with the runtime contract in [architecture.md](architecture.md) §Threading Model: the same boundary that separates raw vs derived also separates the non-RT writer (backend `OnX` callback, MutuallyExclusive cb_group, SeqLock single-writer) from the RT reader (controller `ReadState` via `ControllerState`). Crossing the layering boundary in code (e.g. controller reaching into `dev1.sensor_data[]` to compute a value that the backend should have packed, or backend computing a derived value the controller should own) is an `[CONCERN] Severity: Warning` per [invariants.md](invariants.md) §Escalation Triggers.

## 게이트에 접을 것인가 — fold 는 방향이 두 개다

"강한 답을 게이트에 접으면 모든 소비자에 도달한다" 는 #284 에서 옳았다 — hole 항을 `IsDeviceReadable` 에 접었고, 그래서 마스크를 채우지 않는 backend 가 조용히 통과하지 못한다. **같은 근거가 반대 결론을 내는 축이 있다**: lane 축에서 q 만 읽는 다수를 q̇ 구멍으로 함께 닫으면 과잉거부다 (#446). 판정 기준은 **"도달 범위" 가 아니라 "그 강한 답이 이 소비자에게도 옳은가"** 이며, 아니면 접지 말고 소비자별로 남긴다.

## 배치를 정하는 것이 예산이 아니라 좌표계일 수 있다

새 계산을 RT 경로의 어디에 둘지 정할 때 µs 예산부터 재고 싶어지지만, **먼저 그 경로의 좌표계가 하나인지 본다.** Layer 1 (momentum observer) 배치를 정한 것은 실측 µs 가 아니라 좌표계였다 — `ReducedDynamicsProvider` 가 M/h/g 만 덮고 Coriolis 에는 축약 경로가 없어서, 폐쇄 체인 위에서 재면 축약 M·g 와 open-chain C 가 섞인 값을 재게 된다. **예산 실측이 배치를 정할 것 같으면 그 전에 좌표계 정합성을 확인한다** — 안 그러면 정확한 숫자로 틀린 배치를 고른다.

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
