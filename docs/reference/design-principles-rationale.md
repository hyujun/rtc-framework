# Design Principles — 근거 · 판정 사례 · 예시

> **이 문서는 헌법이 아니다.** 설계 원칙의 **규범** 은 [agent_docs/design-principles.md](../../agent_docs/design-principles.md) 가 갖는다 (P1~P5 의 정의도 거기다). 여기는 각 원칙의 근거, 그 판정이 내려진 실제 사례와 실측, 예시만 둔다. 규칙 문장은 여기 다시 적지 않는다 — 두 곳에 있던 규칙이 어긋난 것이 두 번이다 (2026-10-16 정리). 둘이 어긋나면 agent_docs 쪽이 옳다. 본문의 클래스명·파일 위치는 기록 시점의 것이다.

## Five Principles

규범: [design-principles.md](../../agent_docs/design-principles.md) §Five Principles

- **P4 가 따르는 인터페이스의 예** — Follow: `RTControllerInterface`, `TransportInterface`/`PacketCodec`, `InferenceEngine`, `TaskBase`/`ConstraintBase`, `PhaseManagerBase` (`rtc_mpc` FSM boundary; concrete FSMs live downstream).

## Boundary Rules (`rtc_*` vs integration packages)

규범: [design-principles.md](../../agent_docs/design-principles.md) §Boundary Rules

- **`rtc_inference_check` (#511) 가 ARCH-7 예외인 근거** — argv 로 받은 `.onnx` 경로와 shape 만 보고 표를 찍고 끝나는 일회성 프로세스다: 로봇을 모르고, RT 루프도 ROS 노드도 없으며, 어떤 launch 에도 등장하지 않는다. 런타임 정체성이 아니라 **개발 도구**라서 예외다 — 이 범주는 exec 를 만들되 *실행 중인 시스템의 일부가 되지 않는* 것을 뜻하고, 그 판정 기준은 "bringup chain 에 등장하는가" 다.

**ARCH-7 hook 의 근거 (#213)** — 이 규칙은 오랫동안 예외 서술 없이 "금지" 로만 적혀 있었고, 그 사이 7개 타깃이 축적되는 동안 아무 신호도 없었다 (#213) — 지금은 hook 이 `rtc_*/CMakeLists.txt` 의 신규 `add_executable` **타깃 이름** 을 검사한다. 면제 경로는 셋이다 — (1) HEAD 에 이미 있는 기존 타깃은 재발화하지 않고(**grandfathered** — 개수는 hook 이 세는 값이므로 박제하지 않는다; `mujoco_simulator_node`·`closure_state_publisher`·`se3_error_compare` 는 이 경로로만 통과한다), (2) `example_*` 는 이름으로 면제되며, (3) **그 밖의 신규 agnostic 노드** 는 `add_executable` 줄 또는 그 위에 붙은 주석 블록의 `ARCH-7-exempt` 주석으로 면제한다. 즉 hook 이 지금 마커를 강제하는 대상은 (3) 뿐이고 (1)의 세 노드에는 아직 마커가 없다 — 이들을 rename·재추가하면 name diff 가 신규 타깃으로 보므로, 그때 `ARCH-7-exempt` 주석을 함께 붙여 (3) 경로로 옮긴다.

**Runtime identity rule 의 근거** — exec ↔ ROS 노드 ↔ pgrep ↔ logger 식별자가 모두 같은 이름으로 정렬되어야 디버깅·검증·로그 추적 비용이 일정하게 유지됨. agnostic 패키지가 자체 exec를 가지면 robot-specific identity와 혼선이 생김(어느 launch에서 띄운 어느 instance인지 구분 불가). 따라서 agnostic은 library만 export.

## Controller-YAML Topics Are Controller-Owned (Phase 4 → issue #138)

규범: [design-principles.md](../../agent_docs/design-principles.md) §Controller-YAML Topics Are Controller-Owned

- **controller-owned 인 근거** — These carry external GUI / BT / planner traffic whose schema or QoS may differ per controller.
- **PublishRole 이 `kRobotTransforms` 하나뿐인 이력** — the only value left after issue #196 Phase 5 removed the roles that had no publisher behind them.

The other two lanes are **not** controller-YAML topics and are owned elsewhere: device-wire state/command traffic lives in `devices.<group>.backend:` (DeviceBackend-owned); CM's own fixed publishers (`/rtc_cm/<group>/joint_states` digital-twin republish, `/system/estop_status`, `/rtc_cm/active_controller_name`) are hard-coded on `RtControllerNode`, not declared in any YAML. There is no manager-owned controller-YAML tier and no `ownership:` field (issue #138 removed the former `TopicOwnership` selector — controller-owned is the only model).

CM's publish thread drains the SPSC snapshot and calls `controllers_[active]->PublishNonRtSnapshot(snap)` to delegate controller-owned publishing. External consumers (BT bridge, GUIs, digital_twin, shape_estimation) subscribe to `/rtc_cm/active_controller_name` (TRANSIENT_LOCAL, single-CM scope per locked decision D-A2) and rewire their sub/pubs on each transition. The CM never decides which namespace is authoritative — it exposes the current choice; everything else is pull-based.

## `rtc_controllers` Controllers Are Pure Control Algorithms

규범: [design-principles.md](../../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms

### 금지되는 것

- **바인딩이 코어를 멤버로 드는 참조 형태** — `rtc_controllers` 의 `trajectory/*` · `grasp/*` 를 `integrated_bringup` 의 데모 컨트롤러가 멤버로 들고 쓰는 방식이 그 참조 형태다.

### 금지되지 않는 것

- **`rtc_base` 의존을 허용한 결정** — 코어의 **인자**는 프레임워크-중립 타입이지만, `rtc_base` 까지 끊는 "강" 안은 채택하지 않았다 (issue #236 D-A).

### 코어의 형태

- **프레임워크-중립 입출력의 기준 구현** — 참조 구현은 `rtc_controllers/include/rtc_controllers/compliance/` 의 `task_dynamics.hpp` · `impedance_law.hpp` — 이 규칙이 명문화되기 전에 이미 이 형태로 추출됐고, 그래서 규칙의 예시가 아니라 **기준**이다.
- **YAML 스키마가 코어와 같은 층에 남는 근거** — 프레임워크 타입을 참조하지 않으므로 코어와 같은 층에 남는다.
- **법칙이 궤적 생성기를 소유하지 않는 근거** — `demo_joint_controller` 는 하나의 관절 법칙에 `JointSpaceTrajectory` 2개(arm+hand)를, `demo_wbc_controller` 는 3개를 붙인다. 궤적을 멤버로 든 법칙은 이 구성을 표현할 수 없다. 같은 이유로 duration 휴리스틱(`max_dist / speed`)과 `*_trajectory_speed` 파라미터는 궤적 파라미터화이지 법칙의 게인이 아니다. 참조 구현 `joint/joint_pd_law.hpp`.
- **오차 정의를 바인딩에 두는 근거** — 법칙을 Eigen 만으로 유지하고(`rtc_math`·pinocchio 불포함), 같은 선택을 echo 하는 텔레메트리 lane 옆에 결정을 남겨 둔다. 참조 구현 `compliance/impedance_law.hpp` · `task/task_accel_law.hpp` · `task/task_vel_law.hpp`.
- **프레임 전송을 소유하지 않는 근거** — 궤적 샘플의 twist 를 어느 회전으로 world-aligned 프레임에 옮길지(`R_trajectory` 인가 `R_current` 인가)는 오차 정의와 같은 범주의 **프레임 결정**이므로, 법칙은 이미 회전된 `ν_ff` 를 받는다. 법칙이 회전행렬을 아예 보지 않으면 잘못된 회전을 조용히 적용할 수 없고, 그 선택은 궤적을 샘플링하는 바인딩 옆에 남는다 (#236 S3a). 곱을 named temporary 로 hoist 하는 것은 비트 단위로 inert 하며 — 누산에 접어 넣는 것은 **아니다** — `test_task_vel_core.cpp` 의 리터럴 oracle 이 hoist 이전 형태를 그대로 들고 그것을 고정한다.
- **헬퍼의 출력을 인자로 받는 근거** — compliance §6.3 관성 성형은 `Λ_S` 를 인자로 받지 `TaskDynamics&` 를 받지 않는다. 헬퍼 타입을 이름으로 아는 법칙은 그 헬퍼를 **어디로 수렴시킬지**의 판정을 선점하며 (여기서는 OSC 인라인 Λ 블록의 흡수 방향 = S2b), 그러면 통합 리팩터가 법칙 시그니처를 바꿔야 끝난다. 인자로 받으면 수렴점이 바뀌어도 법칙은 그대로다 (#236 S4). 참조 구현 `compliance/inertia_shaping.hpp`.
- **스크래치를 max-size Eigen 타입으로 두는 근거** — 순수 고정 크기(`Matrix<double,6,6>`)는 **논리 크기가 더 작을 때 깨진다** (`LLT` 에 m<6 블록을 주면 `PlainObjectBase::resize` 의 크기 assert 가 걸린다 — 다만 그 assert 는 **`NDEBUG` 에서 컴파일아웃**되고 이 저장소의 기본 빌드가 Release 이므로, abort 는 운 좋은 경우고 실제로는 resize 가 no-op 한 뒤 대입이 목적지 크기로 돌아 **쓰이지 않은 행 위에서 solve 가 진행된다**: 컴파일 통과, fault 없음, 결과만 틀림. #236 D-S4a). max-size 타입은 heap-free 이면서 Eigen 이 `MatrixXd` 와 같은 커널을 고르므로 추출이 비트 단위로 inert 하다 — 다만 그 inert 함은 **가정하지 말고 리터럴 oracle 이 옛 타입을 들고 재는 것**으로 고정한다 (`test_inertia_shaping_core.cpp`). RT-1 센서로는 Eigen 할당 tripwire 가 필수다: 이 타입이 `MatrixXd` 로 퇴화해도 숫자는 같고 operator-new 게이트는 그것을 못 본다.
- **비트 동치로 판정하는 근거 — P 형 / PD 형 사례** — P 형(`kp·Δq`)을 PD 코어(`kp·Δq − kd·q̇`)의 `K_d = 0` 특수화로 흡수하는 것은 대수적으로 자명하지만 비트 단위로는 아니다: `x − 0.0·q̇` 는 유한한 비영 `x` 에 대해 `x` 지만 `x = −0.0` 이고 `q̇ < 0` 이면 IEEE-754 상 `(−0.0) − (−0.0) = +0.0` 으로 부호가 뒤집힌다. 그리고 그 상태는 병적이지 않다 — 자세 기준 `q_ref` 는 측정값으로 seed 되므로 팔이 멈춰 있는 동안 `Δq` 가 정확히 `+0.0` 이고, 음수 `K_p` 면 매 채널이 `−0.0` 이 된다 (그 도달 경로였던 "`LoadConfig` 가 음수를 받는다" 는 #277 이 여섯 소비자 전부에 `rtc::FloorNonNegativeGain` (#280 이전 이름 `joint::FloorPostureGain`) 을 걸어 닫았다 — 법칙 자체는 여전히 아무것도 floor 하지 않으므로 직접 호출자는 그대로 도달한다). 착수 전 probe 가 그 상태에서 draw 의 50% 가 뒤집히는 것을 `-O0`/`-O2`/`-O3` 모두에서 실측했다 (#236 S6) → **형태가 다르면 함수도 다르다** (S3a 의 6축/병진전용 분리와 같은 판정). 그 근거는 세션 probe 로 남기지 말고 **상주 테스트**로 옮긴다 (`test_posture_core.cpp` 의 `TheProportionalFormIsNotThePdFormWithZeroDamping`) — 나중 독자가 반드시 손대려 할 "단순화" 인데, 틀린 이유가 어떤 tolerance 테스트로도 보이지 않는 부호 있는 0 이기 때문이다.
- **레퍼런스 출처 · 게인 위치 판정의 사례** — 자세 법칙 `kp·(q_ref − q) − kd·q̇` 는 impedance/cascade 가 활성화 시점의 측정값을, OSC 가 config 의 `safe_position` 을 기준으로 쓴다. 이는 **바인딩 차이**이므로 `q_ref` 를 인자로 받으면 하나의 법칙으로 수렴한다 (#236 S6, `joint/posture_law.hpp`). 반대로 **게인을 곱하는 위치가 다르면 법칙이 다르다**: CLIK 은 사영 *뒤에* 곱하고(`kp·(N·Δq)`) 나머지는 *앞에* 곱하는데(`N·(kp·Δq)`), 대수적으로 같고 비트로는 68% 가 다르다 (probe 실측) — 그 소비자를 같은 코어에 넣는 것은 그 자체로 비-inert 변경이다.
- **두 번째 소비자 판정의 사례** — 태스크 어드미턴스의 compliance §7.3 속도항은 `ν_ff` 자리에 궤적 twist 대신 compliant frame 속도 `ν_c` 가 들어갈 뿐 CLIK 과 같은 법칙이라, 새 코어를 만들지 않고 `task/task_vel_law.hpp` 를 재사용하는 것으로 판정됐다 (#236 S5, P5 — 당시 소비자였던 `TaskAdmittanceController` 어댑터는 S7c 에서 삭제됐고, 지금 그 재사용 계약을 들고 있는 것은 `test_admittance_task_vel_reuse.cpp` 다). 반대로 **코어 `Params` POD 를 컨트롤러 `Gains` 에 중첩하는 것은 기본값이 같을 때만 중복 제거**다: `ik_kp_*` 는 기본이 의도적으로 2.0 이고 `TaskVelParams` 는 1.0 이라, 중첩해도 member initializer 에 두 번째 정의가 남아 이름만 바뀐다 (D-S5b). 같은 `Gains` 의 `admittance`(= `AdmittanceParams`)·`inertia`(= `InertiaShapingParams`) 가 중첩된 것은 거기서는 기본값이 코어 것과 **같기** 때문이다. 같은 기준으로 `TaskImpedanceController::Gains` 의 느슨한 4배열(기본 200/28/20/9 = `ImpedanceParams` 와 동일)도 중첩으로 접혔다 (#236 S6b) — 그때 매 tick `ImpedanceParams{kp_pos, kd_pos, kp_rot, kd_rot}` 로 **positional 재조립**하던 것이 함께 사라진다. 그 재조립이야말로 중첩이 없앨 진짜 결함이다: 코어의 필드 *순서* 에 조용히 의존하고, 순서가 바뀌어도 컴파일되고 테스트도 통과한다.

### 3계층 배치 (issue #236 D3)

규범: [design-principles.md](../../agent_docs/design-principles.md) §3계층 배치

### 부가 입력은 인자로 받는다

**setter 로 주입하는 근거** — 전송 계층(누가 센서를 읽어 넣어주는가)은 컨트롤러 밖의 관심사다.

### 완료 상태의 검증 기준 (수는 박제하지 않는다 — AP-DOC-1)

**호출자가 없는 코어가 위반이 아닌 근거** — 호출자가 없는 코어(`ComputeImpedanceForce` · `ComputeJointPdCommand` · `ComputePostureTorque` / `ComputePostureVelocity` · `ComputeTaskAcceleration`)는 위반이 아니다 — 그 법칙을 인라인으로 다시 쓰는 살아 있는 바인딩이 없으므로 "미수렴 사본" 이 아니라 "바인딩을 기다리는 코어" 다.

**완료형을 바인딩 계층에 한정하는 근거** — `rtc_tsid` 의 SE3 / object-SE3 / CoM / posture task 는 가속도형 법칙을 자체적으로 쓴다 (`a_ff + kp ⊙ e + kd ⊙ ė`). 이는 바인딩 누수가 아니라 **동급 코어 간 중복** 축이다 — `rtc_tsid` 는 P5 (규범의 패키지 매핑) 가 "QP tasks, constraints" 를 배정한 자기 도메인의 코어이고, 속도 오차 정의부터 다르다 (Jlog6 정확 미분 대 `ν_d − ν`). 따라서 `ComputeTaskAcceleration` 으로의 수렴은 inert 리팩터가 아니라 **법칙 변경**이며, 착수하려면 별도 판단이 필요하다 ([AGENTS.md](../../AGENTS.md) §6 E-9). 그래서 **"코어 간 법칙 중복이 없다"** 는 완료형으로 쓰지 않는다 — 쓰는 순간 문서-코드 불일치를 새로 만든다.

- **이 문서의 근거 문단에 나오는 어댑터 클래스명**(`ClikController` · `TaskImpedanceController` · `TaskAdmittanceController` 등)**은 추출 슬라이스가 판정을 내리던 시점의 대상이다** — 지금 코드에 없다. 그 판정이 왜 그렇게 났는지는 여전히 유효하므로 남겨 두되, 살아 있는 코드로 읽지 않는다.
- 수렴 이력(슬라이스 S1–S7 · DLS 축 #282 · 태스크 속도 축 #314)과 이 규칙의 결정·개정 로그는 issue #236 코멘트에 있다 — 여기 복제하지 않는다.

**ARCH-7 standalone-node 예외가 컨트롤러에 적용되지 않는 이유** — 그 예외는 robot-agnostic *노드* 패키지에 대한 것이지 컨트롤러에 대한 것이 아니다.

**근거**: 컨트롤러를 순수 알고리즘으로 유지하면 (a) 단위 테스트가 ROS 컨텍스트 없이 성립하고, (b) 같은 법칙이 sim / 실기 / 오프라인 재생에서 배선만 갈아끼워 재사용되며, (c) 배선 결정(QoS, 네임스페이스, 메시지 타입)이 robot bringup 한 곳에 모인다. 상속 금지가 추가된 근거는 (d) — 프레임워크 계약을 구현하는 순간 글루가 법칙과 같은 파일에 들어오고, 그 글루의 대부분은 컨트롤러마다 **동일한 boilerplate** 라서 구현체 수만큼 복제된다. 실제로 mailbox 스켈레톤은 그렇게 복제됐고 (#206), 그 복제본들의 검증 공백에서 결함이 반복해 나왔다. 컨트롤러에 구독을 넣으려는 충동은 대개 "이 입력을 어떻게 넣지?" 에서 나오는데 답은 setter 이지 구독이며, 인터페이스를 상속하려는 충동은 "CM 이 이걸 어떻게 부르지?" 에서 나오는데 답은 바인딩이지 상속이 아니다.

## Backend / Controller Layering

규범: [design-principles.md](../../agent_docs/design-principles.md) §Backend / Controller Layering

- **Backend 가 모든 stride slot 을 zero-fill 하는 근거** — logging / digital_twin / replay tools see a single SSoT for HW state and a backend swap does not silently change consumer semantics.
- **Controller = behavior-facing 의 예** — WBC 는 `inference_data[ft_base+1..3]` (fx/fy/fz) 와 `inference_enable[f]` 만 읽고, `force_magnitude` · `in_contact` · `force_rate` · `slip_rate` 는 그 raw 에서 컨트롤러 안에서 계산한다 — `DeviceStateCache` 에는 없다.
- **`hole_mask` 의 근거 (#284)** — The cache is persistent, so a slot the current message did not reach keeps the previous message's value, and a downstream `num_channels >= model_dim` check cannot see that (`num_channels` is the wire length). The `positions` lane therefore carries a per-slot companion: **bit i set ⇒ slot i was not written by the most recent state message.** The polarity means a backend that never touches the field claims "no holes" and the F5 gate silently degrades to its pre-#284 strength for that device — build green, tests green. Backends that ingest a named `sensor_msgs/JointState` get this for free by going through `WriteJointStateToCache` (`integrated_bringup/include/integrated_bringup/backends/joint_state_reorder.hpp`), which is the shipped path for all three; **anything that fills `positions` by hand owns the mask too.** Field contract: `rtc_base/include/rtc_base/types/types.hpp`; consumer: `rtc::IsDeviceReadable`.
- **Controller-owned publish 가 derived view 인 이력** — `WbcState` · `GraspState` · ToF snapshot 의 필드 중 backend raw 의 mirror 로 시작했다가 소비자가 사라진 것은 layer-d 정리 때 컨트롤러가 유도하는 값으로 바꾸는 전이를 거쳐 뒤의 ABI 개정에서 제거하기로 했다 (`TODO(layer-d)` 표시). 그 용도의 표시는 2026-10-16 기준 코드에 남아 있지 않다 — 지금 `layer-d` 가 붙은 주석 셋은 변형 가드 · 파지 임계 같은 다른 뜻이다.
- **Controller 출력이 actuation quantity 에서 멈추는 근거 (current 결정)** — Current is a hardware quantity — if a drive is current-controlled, the **backend** (or its driver/firmware) converts τ → I using the motor constants it owns (`k_t`, gear ratio), exactly as it owns the inverse on the read side (the hand UDP response already carries a current channel). This keeps `rtc_msgs` (`JointCommand`) and `rtc_base` (`DeviceOutput`) free of any `goal_current` field and the controller free of per-motor electrical models, so the same controller drives a torque-mode sim and a current-mode real drive unchanged. Hand feedforward torque (τ_ff, Stage C-3) is therefore expressed as effort/torque in the command; the τ → current conversion + wire/firmware mode lives entirely in the hand backend/driver — adding current support to a drive touches neither the controller nor the message ABI. *(Decision 2026-06-11; see [invariants.md](../../agent_docs/invariants.md) E-3 — the once-assumed `goal_current` ABI change was withdrawn on this basis.)*

Concrete consequences:

- Removing a sensor channel from a backend does not require touching controllers that don't read it; conversely, dropping a derived field from a controller publish ABI does not require touching backends.

This layering pairs with the runtime contract in [architecture.md](../../agent_docs/architecture.md) §Threading Model: the same boundary that separates raw vs derived also separates the non-RT writer (backend `OnX` callback, MutuallyExclusive cb_group, SeqLock single-writer) from the RT reader (controller `ReadState` via `ControllerState`).

## 게이트에 접을 것인가 — fold 는 방향이 두 개다

규범: [design-principles.md](../../agent_docs/design-principles.md) §게이트에 접을 것인가 — fold 는 방향이 두 개다

"강한 답을 게이트에 접으면 모든 소비자에 도달한다" 는 #284 에서 옳았다 — hole 항을 `IsDeviceReadable` 에 접었고, 그래서 마스크를 채우지 않는 backend 가 조용히 통과하지 못한다. **같은 근거가 반대 결론을 내는 축이 있다**: lane 축에서 q 만 읽는 다수를 q̇ 구멍으로 함께 닫으면 과잉거부다 (#446).

## 배치를 정하는 것이 예산이 아니라 좌표계일 수 있다

규범: [design-principles.md](../../agent_docs/design-principles.md) §배치를 정하는 것이 예산이 아니라 좌표계일 수 있다

**좌표계가 배치를 정한 사례** — Layer 1 (momentum observer) 배치를 정한 것은 실측 µs 가 아니라 좌표계였다 — `ReducedDynamicsProvider` 가 M/h/g 만 덮고 Coriolis 에는 축약 경로가 없어서, 폐쇄 체인 위에서 재면 축약 M·g 와 open-chain C 가 섞인 값을 재게 된다.

## When Generalization Requires a Design Change

규범: [design-principles.md](../../agent_docs/design-principles.md) §When Generalization Requires a Design Change

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
