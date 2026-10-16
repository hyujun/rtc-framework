# Design Principles — 근거 · 판정 사례 · 예시

> **이 문서는 헌법이 아니다.** 설계 원칙의 **규범** 은 [agent_docs/design-principles.md](../../agent_docs/design-principles.md) 가 갖는다 (P1~P5 의 정의도 거기다). 여기는 각 원칙의 근거, 그 판정이 내려진 실제 사례와 실측, 예시만 둔다. 규칙 문장은 여기 다시 적지 않는다 — 두 곳에 있던 규칙이 어긋난 것이 두 번이다 (2026-10-16 정리). 둘이 어긋나면 agent_docs 쪽이 옳다. 본문의 클래스명·파일 위치는 기록 시점의 것이다.

## Five Principles

규범: [design-principles.md](../../agent_docs/design-principles.md) §Five Principles

- **P4 가 따르는 인터페이스의 예** — Follow: `RTControllerInterface`, `TransportInterface`/`PacketCodec`, `InferenceEngine`, `TaskBase`/`ConstraintBase`, `PhaseManagerBase` (`rtc_mpc` FSM boundary; concrete FSMs live downstream).

## Boundary Rules (`rtc_*` vs integration packages)

규범: [design-principles.md](../../agent_docs/design-principles.md) §Boundary Rules

- **`rtc_inference_check` (#511) 가 ARCH-7 예외인 근거** — argv 로 받은 `.onnx` 경로와 shape 만 보고 표를 찍고 끝나는 일회성 프로세스다: 로봇을 모르고, RT 루프도 ROS 노드도 없으며, 어떤 launch 에도 등장하지 않는다. 런타임 정체성이 아니라 **개발 도구**라서 예외다 — 이 범주는 exec 를 만들되 *실행 중인 시스템의 일부가 되지 않는* 것을 뜻하고, 그 판정 기준은 "bringup chain 에 등장하는가" 다.

**ARCH-7 hook 의 근거 (#213)** — 이 규칙은 오랫동안 예외 서술 없이 "금지" 로만 적혀 있었고, 그 사이 7개 타깃이 아무 신호 없이 축적됐다. 그래서 hook 이 신규 `add_executable` **타깃 이름** 을 검사하게 됐다. 면제 경로와 마커는 [design-principles.md](../../agent_docs/design-principles.md) §Boundary Rules 가 갖는다.

## Controller-YAML Topics Are Controller-Owned (Phase 4 → issue #138)

규범: [design-principles.md](../../agent_docs/design-principles.md) §Controller-YAML Topics Are Controller-Owned

- **controller-owned 인 근거** — These carry external GUI / BT / planner traffic whose schema or QoS may differ per controller.

## `rtc_controllers` Controllers Are Pure Control Algorithms

규범: [design-principles.md](../../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms

### 금지되는 것

- **바인딩이 코어를 멤버로 드는 참조 형태** — `rtc_controllers` 의 `trajectory/*` · `grasp/*` 를 `integrated_bringup` 의 데모 컨트롤러가 멤버로 들고 쓰는 방식이 그 참조 형태다.

### 금지되지 않는 것

- **`rtc_base` 의존을 허용한 결정** — 코어의 **인자**는 프레임워크-중립 타입이지만, `rtc_base` 까지 끊는 "강" 안은 채택하지 않았다 (issue #236 D-A).

### 코어의 형태

- **법칙이 궤적 생성기를 소유하지 않는 근거** — `demo_joint_controller` 는 하나의 관절 법칙에 `JointSpaceTrajectory` 2개(arm+hand)를, `demo_wbc_controller` 는 3개를 붙인다. 궤적을 멤버로 든 법칙은 이 구성을 표현할 수 없다.
- **게인 위치 판정의 사례** — 자세 법칙의 레퍼런스 출처 차이(활성화 시점 측정값 / `safe_position`)는 바인딩 차이라 `q_ref` 인자로 수렴했다 (#236 S6). 반대로 CLIK 은 게인을 사영 *뒤에* 곱하고(`kp·(N·Δq)`) 나머지는 *앞에* 곱한다 — 대수적으로 같지만 비트로는 68% 가 다르다 (probe 실측, 코드에는 남아 있지 않다). 그 소비자를 같은 코어에 넣는 것은 비-inert 변경이다.
- **Params 중첩 판정의 사례** — 코어 `Params` POD 를 컨트롤러 `Gains` 에 중첩하는 것은 기본값이 같을 때만 중복 제거다. `ik_kp_*` 의 기본은 의도적으로 2.0, `TaskVelParams` 는 1.0 이라 중첩해도 두 번째 정의가 남아 이름만 바뀐다 (D-S5b). `admittance`·`inertia`·`TaskImpedanceController::Gains` 4배열(200/28/20/9)은 기본값이 같아 접혔고 (#236 S6b), 그때 매 tick positional 재조립 — 필드 *순서* 에 조용히 의존해 순서가 바뀌어도 컴파일되고 테스트도 통과하는 결함 — 도 사라졌다. 태스크 어드미턴스의 compliance §7.3 속도항은 `ν_ff` 자리만 다른 CLIK 과 같은 법칙이라 새 코어 없이 `task/task_vel_law.hpp` 를 재사용하는 것으로 판정됐다 (S5); 그 계약은 `test_admittance_task_vel_reuse.cpp` 가 든다.

### 부가 입력은 인자로 받는다

**setter 로 주입하는 근거** — 전송 계층(누가 센서를 읽어 넣어주는가)은 컨트롤러 밖의 관심사다.

### 완료 상태의 검증 기준 (수는 박제하지 않는다 — AP-DOC-1)

**호출자가 없는 코어가 위반이 아닌 근거** — 호출자가 없는 코어(`ComputeImpedanceForce` · `ComputeJointPdCommand` · `ComputePostureTorque` / `ComputePostureVelocity` · `ComputeTaskAcceleration`)는 위반이 아니다 — 그 법칙을 인라인으로 다시 쓰는 살아 있는 바인딩이 없으므로 "미수렴 사본" 이 아니라 "바인딩을 기다리는 코어" 다.

**완료형을 바인딩 계층에 한정하는 근거** — `rtc_tsid` 의 SE3 / object-SE3 / CoM / posture task 는 가속도형 법칙(`a_ff + kp ⊙ e + kd ⊙ ė`)을 자체로 쓰는 **동급 코어 간 중복** 축이다 — P5 가 "QP tasks, constraints" 를 배정한 자기 도메인이고 속도 오차 정의부터 다르다 (Jlog6 정확 미분 대 `ν_d − ν`). `ComputeTaskAcceleration` 으로의 수렴은 inert 리팩터가 아니라 **법칙 변경**이라 별도 판단이 필요하고 ([AGENTS.md](../../AGENTS.md) §6 E-9), 그래서 "코어 간 법칙 중복이 없다" 를 완료형으로 쓰지 않는다 — 쓰는 순간 문서-코드 불일치를 새로 만든다.

- 어댑터 클래스명(`ClikController` 등)은 추출 슬라이스 시점의 대상이고 지금 코드에 없다. 수렴 이력과 결정 로그는 [controllers-rationale.md](controllers-rationale.md) §알고리즘 코어 추출 이력 (#236) 과 issue #236 코멘트에 있다.

**근거**: 컨트롤러를 순수 알고리즘으로 유지하면 (a) 단위 테스트가 ROS 컨텍스트 없이 성립하고, (b) 같은 법칙이 sim / 실기 / 오프라인 재생에서 배선만 갈아끼워 재사용되며, (c) 배선 결정(QoS, 네임스페이스, 메시지 타입)이 robot bringup 한 곳에 모인다. 상속 금지가 추가된 근거는 (d) — 프레임워크 계약을 구현하는 순간 글루가 법칙과 같은 파일에 들어오고, 그 글루의 대부분은 컨트롤러마다 **동일한 boilerplate** 라서 구현체 수만큼 복제된다. 실제로 mailbox 스켈레톤은 그렇게 복제됐고 (#206), 그 복제본들의 검증 공백에서 결함이 반복해 나왔다. 컨트롤러에 구독을 넣으려는 충동은 대개 "이 입력을 어떻게 넣지?" 에서 나오는데 답은 setter 이지 구독이며, 인터페이스를 상속하려는 충동은 "CM 이 이걸 어떻게 부르지?" 에서 나오는데 답은 바인딩이지 상속이 아니다.

## Backend / Controller Layering

규범: [design-principles.md](../../agent_docs/design-principles.md) §Backend / Controller Layering

- **Backend 가 모든 stride slot 을 zero-fill 하는 근거** — logging / digital_twin / replay tools see a single SSoT for HW state and a backend swap does not silently change consumer semantics.
- **Controller 출력이 actuation quantity 에서 멈추는 근거 (2026-06-11 결정)** — 전류는 하드웨어 양이라 전류 제어 드라이브면 **backend** 가 자기 소유의 모터 상수(`k_t`, 기어비)로 τ → I 를 변환한다. 그래야 `rtc_msgs` (`JointCommand`) · `rtc_base` (`DeviceOutput`) 에 `goal_current` 가 없고 컨트롤러가 모터별 전기 모델을 갖지 않으며, 같은 컨트롤러가 토크 모드 sim 과 전류 모드 실기를 그대로 구동한다. 손 τ_ff (Stage C-3) 도 effort 로 표현한다 — 한때 가정한 `goal_current` ABI 변경은 이 근거로 철회됐다 ([invariants.md](../../agent_docs/invariants.md) E-3).

## 게이트에 접을 것인가 — fold 는 방향이 두 개다

규범: [design-principles.md](../../agent_docs/design-principles.md) §게이트에 접을 것인가 — fold 는 방향이 두 개다

"강한 답을 게이트에 접으면 모든 소비자에 도달한다" 는 #284 에서 옳았다 — hole 항을 `IsDeviceReadable` 에 접었고, 그래서 마스크를 채우지 않는 backend 가 조용히 통과하지 못한다. **같은 근거가 반대 결론을 내는 축이 있다**: lane 축에서 q 만 읽는 다수를 q̇ 구멍으로 함께 닫으면 과잉거부다 (#446).

## 배치를 정하는 것이 예산이 아니라 좌표계일 수 있다

규범: [design-principles.md](../../agent_docs/design-principles.md) §배치를 정하는 것이 예산이 아니라 좌표계일 수 있다

**좌표계가 배치를 정한 사례** — Layer 1 (momentum observer) 배치를 정한 것은 실측 µs 가 아니라 좌표계였다 — `ReducedDynamicsProvider` 가 M/h/g 만 덮고 Coriolis 에는 축약 경로가 없어서, 폐쇄 체인 위에서 재면 축약 M·g 와 open-chain C 가 섞인 값을 재게 된다.
