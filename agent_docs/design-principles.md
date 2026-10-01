# Design Principles for `rtc_*` Packages

`rtc_*` packages are the **robot-agnostic** backbone of this framework. Any modification must preserve this property. Robot-specific logic, hardware assumptions, and fixed-shape constants belong in the **integration packages** — every non-`rtc_*` package that `<depend>`s on an `rtc_*` one (the hook derives this set from `package.xml`, not from a name glob). When in doubt: *"Would this code still make sense on a 7-DOF arm with a 2-finger gripper?"*

> **이 파일의 규칙 위반은 [invariants.md](invariants.md) ARCH-1~4 의 escalation 대상이다.** 위반이 불가피해 보이면 코드를 쓰기 **전에** `[CONCERN]` 을 보고한다 ([AGENTS.md](../AGENTS.md) §6).

각 원칙의 근거 · 판정이 내려진 사례와 실측 · 예시는 헌법 밖 [design-principles-rationale.md](../docs/reference/design-principles-rationale.md) 가 갖는다.

## Five Principles

다른 문서는 이 원칙들을 **P1~P5** 로 인용한다. `conventions.md` 의 include-priority `P1..P4` 와 compliance 명세의 부호규약 `P2/P3` 는 다른 네임스페이스다.

1. **P1 — Extensibility** -- Adding a new robot, new DOF count, new transport, or new controller must require **zero source edits** inside `rtc_*`. Achievable via: (a) YAML config, (b) `RTC_REGISTER_CONTROLLER` from downstream, or (c) implementing an abstract interface.
2. **P2 — Generality** -- No robot names, joint counts, finger counts, topic names, or hardware identifiers hardcoded in `rtc_*`. Use YAML-injected values, template parameters, or runtime config. Constants like `kNumRobotJoints` are **upper-bound capacity**, not per-robot assumptions. Names describe the *role* (`num_joints`), never the *robot*.
3. **P3 — Modularity** -- Respect the dependency graph. Never introduce upward dependencies. Never cross-link siblings the graph doesn't connect. If a feature spans packages: (a) push abstraction down to a shared base, or (b) invert via interface injection.
4. **P4 — Interface-first** -- New functionality with multiple implementations MUST define an **abstract class, concept, or pure-virtual interface** before any concrete implementation. Concrete classes register via factory/registry -- never `#ifdef` or hardcoded switches.
5. **P5 — Deduplication & Reuse** -- Before writing utilities, search existing `rtc_*` (lock-free · filters · logging · threading → `rtc_base`; URDF · Pinocchio → `rtc_urdf_bridge`; transport · codecs → `rtc_communication`; ONNX → `rtc_inference`; QP tasks · constraints → `rtc_tsid`). If existing doesn't quite fit, **generalize it** -- don't fork.

## Boundary Rules (`rtc_*` vs integration packages)

| Belongs in `rtc_*` | Belongs in an integration package |
|--------------------|---------------------|
| Abstract interfaces, concepts, base classes | Concrete implementations via `RTC_REGISTER_CONTROLLER` |
| **제어 법칙 코어** — Eigen/span in-out, `RTControllerInterface` 를 모름 | **바인딩** — `RTControllerInterface` 구체 구현, mailbox 소비, `ControllerOutput` 조립, 등록 |
| DOF-generic algorithms (variable `n_joints`) | Fixed-DOF launch files, URDF, MJCF, meshes |
| Transport/codec templates (`Transceiver<T,C>`) | Robot-specific packet structs as template args |
| YAML-driven parameter schemas | YAML files with actual robot values |
| Controller registry, TSID solver core | Demo controllers, BT coordinator, bringup |
| RT threading, SPSC, SeqLock, E-STOP logic | Hardware drivers |
| 진입 *함수* 만 export (`RtControllerMain()`) — RT 제어 루프를 구동하는 exec 를 소유하지 않는다 (ARCH-7) | **Runtime identity 소유** — `add_executable` 로 exec 를 만들고 `RtControllerMain(argc, argv, "<exec_name>")` 호출. ROS 노드 이름 = exec 이름 |
| Agnostic launch 는 자기 노드만 띄운다. robot-specific 패키지 의존 / exec 호출 금지 | 통합 launch (RT 컨트롤러 + 시뮬레이터 + 드라이버 chain) |

**Runtime identity rule**: exec ↔ ROS 노드 ↔ pgrep ↔ logger 식별자는 모두 같은 이름으로 정렬한다. 그래서 agnostic 패키지는 library 만 export 한다.

**ARCH-7 의 범위**: 막으려는 것은 *제어 프레임워크의 런타임 정체성* 이 agnostic 패키지로 새는 것이지 `add_executable` 자체가 아니다. 판정 기준은 **"bringup chain 에 등장하는가"** 이고, 다음은 예외다.

- **Robot-agnostic standalone 노드** (`mujoco_simulator_node`, `closure_state_publisher`) — 로봇 이름을 모르고 모델을 파라미터로 받으며 RT 제어 루프를 소유하지 않는다. 그 launch 는 자기 노드만 띄운다.
- **Example 실행 파일** (`example_*`, `se3_error_compare`) — API 사용법 데모다.
- **오프라인 검사 도구** (`rtc_inference_check`) — 일회성 프로세스이고 실행 중인 시스템의 일부가 되지 않는다.

예외에 해당하지 않는 신규 `add_executable` 을 `rtc_*` 에 추가하려면 `[CONCERN]` (E-1) 이다. 신규 agnostic 노드는 `add_executable` 줄 또는 그 위에 붙은 주석 블록의 `ARCH-7-exempt` 주석으로 표시한다 (`example_*` 는 이름으로 면제). HEAD 에 이미 있는 타깃은 재발화하지 않지만 (grandfathered), 그것을 rename · 재추가할 때는 마커를 함께 붙인다.

## Controller-YAML Topics Are Controller-Owned

- Every topic declared in a controller's YAML `topics:` section is **controller-owned**: created on a per-controller `LifecycleNode` whose namespace is `/<config_key>`, so relative YAML paths resolve to `/<config_key>/<topic>`. Two flavors: (a) PublishRole-mapped (`kRobotTransforms`), declared in the YAML; (b) controller-private SeqLock + `Setup*Publisher` helper, no PublishRole / YAML entry.
- There is no manager-owned controller-YAML tier and no `ownership:` field. 나머지 두 lane (DeviceBackend-owned, CM fixed) 과 소비자의 rewire 는 [architecture.md](architecture.md) §RT vs non-RT Topic Ownership 이 갖는다.

## `rtc_controllers` Controllers Are Pure Control Algorithms

`rtc_controllers` 는 **제어 법칙 (알고리즘) 만** 소유한다. 자기 노드도, publisher 도, subscription 도 만들지 않으며 **`RTControllerInterface` 를 상속하지도 않는다.** ROS 배선은 전적으로 integration 패키지가 소유한다. 규칙은 **새 코드에 즉시 구속** 된다 — 새 제어 법칙은 코어로 쓰고, 필요하면 바인딩을 integration 패키지에 만든다.

### 금지되는 것

- **`rtc_controllers` 안의 `RTControllerInterface` 구체 구현.** 프레임워크 계약을 구현하는 클래스 (lifecycle 훅, `Compute(ControllerState) → ControllerOutput`, target mailbox, E-STOP 훅, 등록) 는 **바인딩** 이며 integration 패키지가 소유한다. 코어는 바인딩이 멤버로 들고 쓴다.
- **컨트롤러가 `RTControllerInterface::get_lifecycle_node()` 로 노드를 받아 자기 pub/sub 을 만드는 것.** 바인딩 계층에서도 노드 접근은 `CreateOwnedTopics` 경로로만 쓴다 — 인터페이스가 노드 접근을 제공한다는 사실은 그것을 써도 된다는 뜻이 아니다.

### 금지되지 않는 것

- `rclcpp/logging.hpp` (로깅은 노드를 만들지 않는다), 패키지 차원의 `rtc_base` 의존.

### 코어의 형태

- **입출력은 Eigen / `std::span` 등 프레임워크-중립 타입** 이다. `ControllerState` / `ControllerOutput` 의 해체·조립은 바인딩 몫이다. 기준 구현은 `rtc_controllers/include/rtc_controllers/compliance/`.
- **`Resize()` (off-RT, 할당 허용) / `Compute()` (`noexcept`, heap-free) 분리** 를 유지한다.
- YAML 스키마는 코어 옆의 `Params` POD + `ParseXxxParams(YAML::Node)` 자유 함수다 (yaml-cpp 만 의존, 비-RT).
- **제어 법칙은 궤적 생성기를 소유하지 않는다** — 궤적 **샘플** 을 인자로 받는다. 어느 궤적이 어느 법칙을 먹이는지, 몇 개인지는 구조 결정이라 바인딩 몫이다. duration 휴리스틱과 `*_trajectory_speed` 는 궤적 파라미터화이지 법칙의 게인이 아니다.
- **태스크 공간 법칙은 오차의 정의를 소유하지 않는다** — pose error `e` 를 인자로 받는다. 어느 error type 으로 계산할지는 프레임을 소유한 바인딩이 정한다.
- **프레임 전송도 소유하지 않는다** — 법칙은 이미 회전된 `ν_ff` 를 받고 회전행렬을 보지 않는다.
- **법칙은 자기를 먹이는 헬퍼를 알지 않는다 — 그 헬퍼의 출력을 인자로 받는다** (`TaskDynamics&` 가 아니라 `Λ_S`). 헬퍼 타입을 이름으로 아는 법칙은 수렴 방향의 판정을 선점한다.
- **무상태 코어의 스크래치는 저장 용량이 컴파일 타임에 묶인 Eigen 타입** (`Matrix<double,Dyn,Dyn,0,MaxR,MaxC>`) 을 로컬로 둔다 — 호출자에게 버퍼를 빌리지 않는다. 순수 고정 크기는 논리 크기가 더 작을 때 Release 에서 조용히 틀린다. RT-1 센서로는 Eigen 할당 tripwire 가 필수다 (`MatrixXd` 로 퇴화해도 숫자는 같다).
- **대수적 동치는 형태를 합칠 근거가 못 된다 — 비트 동치를 도달 가능한 상태에서 재고 판정한다.** 형태가 다르면 함수도 다르다. 그 근거는 세션 probe 가 아니라 **상주 테스트** 로 남긴다.
- **레퍼런스의 출처가 다른 것은 법칙이 다른 것이 아니다** — 인자로 받으면 하나의 법칙으로 수렴한다. 반대로 **게인을 곱하는 위치가 다르면 법칙이 다르다** (사영 앞 / 뒤).
- **두 번째 소비자는 법칙이 같은가로 판정한다** — 기본값·인자의 출처가 같은가로 판정하지 않는다 (P5). 코어 `Params` POD 를 컨트롤러 `Gains` 에 중첩하는 것은 **기본값이 같을 때만** 중복 제거다. 코어 `Params` 를 매 tick positional 로 재조립하지 않는다 — 필드 순서에 조용히 의존한다.

### 3계층 배치

| 계층 | 무엇이 사는가 | 어디에 |
|---|---|---|
| **코어 — 알고리즘** | 제어 법칙, 수치 커널, 궤적 생성기, 상태기계, 파라미터 스키마. Eigen/span in-out | `rtc_controllers` |
| **base — 프레임워크 공통 글루** | 모든 바인딩이 *동일하게* 필요로 하는 것: target mailbox, submodel 선택, device 한계값 로드, device 판독가능성 게이트, E-STOP scaffolding | `rtc_controller_interface` |
| **바인딩 — 배치 고유** | `RTControllerInterface` 구체 구현, `ControllerOutput` 조립, `goal_type` 해석, E-STOP 정책, 텔레메트리, 등록, production YAML | integration 패키지 |

경계 판정 한 줄: **"이 코드가 `RTControllerInterface` 의 존재를 알아야 하는가?"** — 아니오면 코어, 예이고 모든 컨트롤러에서 같으면 base, 예이고 배치마다 다르면 바인딩.

### 부가 입력은 인자로 받는다

외부 F/T wrench 처럼 device lane 에 없는 입력은 컨트롤러가 구독하지 않고 **비-RT setter** 로 주입한다 (RT 와의 교환은 `SeqLock` / SPSC 로만). 값의 freshness 는 ROS 타임스탬프가 아니라 **generation 카운터 + tick 카운팅** 으로 표현한다 (RT 에서 clock 을 읽지 않는다).

### 완료 상태의 검증 기준

- **`rtc_controllers` 는 순수하다** — `grep -rn "public RTControllerInterface" rtc_controllers/include` 와 `grep -n rtc_controller_interface rtc_controllers/package.xml` 이 둘 다 비어야 한다.
- **바인딩 계층에 인라인 법칙 사본이 없다** — 세는 기준은 "코어 대응물이 있는데 호출하지 않는 법칙" 이고, `grep -rn "cwiseProduct\|diagonal().array() +=\|LDLT<\|LLT<\|Jpinv\|pseudoInverse" integrated_bringup/src integrated_bringup/include` 가 0건이어야 한다. 호출자가 없는 코어는 위반이 아니다.
- **완료형은 바인딩 계층에 한정한다.** `rtc_tsid` 의 task 들이 가속도형 법칙을 자체적으로 쓰는 것은 동급 코어 간 중복 축이고, 그 수렴은 inert 리팩터가 아니라 법칙 변경이다 (E-9). "코어 간 법칙 중복이 없다" 를 완료형으로 쓰지 않는다.

이 규칙은 ARCH-7 과 별개다 — ARCH-7 은 exec 소유를, 이 규칙은 exec 가 없더라도 노드·구독을 만드는 것과 프레임워크 인터페이스를 상속하는 것을 금지한다. ARCH-7 의 standalone-node 예외는 컨트롤러에 적용되지 않는다. 위반이 필요해 보이면 `[CONCERN]` (E-1) 이다.

## Backend / Controller Layering

The backend ↔ controller boundary is governed by **responsibility**, not by data shape. The topic-ownership rule governs *who owns a ROS topic*; this rule governs *who computes a value*.

- **Backend = hardware-facing.** Every backend packs the raw values its hardware publishes into `DeviceStateCache`, filling **all stride slots the layout reserves** regardless of whether the active controller reads them. Unused slots are zero-filled (not skipped).
- **A backend that cannot fill a joint-position slot must say so — `hole_mask`.** Bit i set ⇒ slot i was not written by the most recent state message. A backend that never touches the field claims "no holes". Go through `WriteJointStateToCache`; anything that fills `positions` by hand owns the mask too.
- **Controller = behavior-facing.** Controllers read only the slots they consume. Derived quantities (`force_magnitude`, `in_contact`, `force_rate`, `slip_rate`) are computed inside the controller from raw inputs — they do not appear in `DeviceStateCache`.
- **Controller-owned publish = controller's derived view**, not a mirror of backend raw.
- **Controller command output stops at the actuation quantity, never the electrical quantity.** A controller emits position, velocity, or effort (torque) only; it never emits motor current. If a drive is current-controlled, the backend converts τ → I using the motor constants it owns. `rtc_msgs` / `rtc_base` carry no current-goal field.
- A new behavior signal is added in the controller, not the backend. A backend that fills only a subset of the layout is a legitimate sparse backend.

Crossing the layering boundary in code (a controller computing what the backend should have packed, or a backend computing a derived value the controller should own) is a `[CONCERN] Severity: Warning`.

## 게이트에 접을 것인가 — fold 는 방향이 두 개다

강한 답을 공용 게이트에 접으면 모든 소비자에 도달한다. 판정 기준은 **"도달 범위" 가 아니라 "그 강한 답이 이 소비자에게도 옳은가"** 다 — 아니면 접지 말고 소비자별로 남긴다 (일부 소비자만 필요로 하는 조건을 접으면 나머지에게 과잉거부다).

## 배치를 정하는 것이 예산이 아니라 좌표계일 수 있다

새 계산을 RT 경로의 어디에 둘지 정할 때, 예산을 재기 **전에** 그 경로의 좌표계가 하나인지 확인한다 — 안 그러면 정확한 숫자로 틀린 배치를 고른다.

## When Generalization Requires a Design Change

If you cannot satisfy all five principles with a local edit, STOP and:

1. Report a `[CONCERN] Severity: Warning` ([AGENTS.md](../AGENTS.md) §6 포맷)
2. Propose an interface refactor or dependency inversion as a separate task
3. Do NOT embed robot-specific logic in `rtc_*` "for now"
