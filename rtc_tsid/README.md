# rtc_tsid

TSID (Task-Space Inverse Dynamics) QP 솔버 라이브러리와 velocity-level CLIK reference 생성기. **유형**: 라이브러리 패키지

## 개요

URDF 기반 매니퓰레이터의 전신 제어(Whole-Body Control)용 QP 를 구성하고 푼다. Weighted QP (WQP) 와 Hierarchical QP (HQP) formulation 을 Strategy 패턴으로 교체할 수 있고, 백엔드는 ProxSuite (ProxQP) 다. compute 경로는 사전 할당된 workspace 만 쓰며 (RT-1), 동역학 계산은 Pinocchio 공유 캐시 하나로 한다. phase 별 가중치·태스크 활성은 YAML preset 으로 바꾼다.

ROS 2 노드나 `RTControllerInterface` 구현이 아니라 정적 라이브러리다. 소비자는 [rtc_controllers](../rtc_controllers/README.md) 와 [integrated_bringup](../integrated_bringup/README.md) 의 컨트롤러다. `rtc_mpc` 에는 의존하지 않는다 — 둘을 잇는 것은 컨트롤러의 몫이다:

```
rtc_mpc::MPCSolutionManager.ComputeReference(...) -> (q_ref, v_ref, a_ff, u_fb)
        | 컨트롤러 코드가 주입
rtc::tsid::ControlReference.{q_des, v_des, a_des += u_fb}
        |
rtc::tsid::TSIDController::Compute(...)
```

`SE3Task` · `PostureTask` · `ForceTask` 의 `set_*_reference()` 는 RT-safe 라 매 tick 시간-가변 reference (MPC 보간 결과) 로 부를 수 있다.

## 빠른 시작

빌드·테스트는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 패키지만 빌드하려면 다음과 같다 (ProxSuite 는 `install.sh` 가 설치한다).

```bash
./build.sh --tests -p rtc_tsid
```

다른 패키지에서 쓰려면:

```cmake
find_package(rtc_tsid REQUIRED)
ament_target_dependencies(my_target rtc_tsid)
```

호출 순서는 `Init` (non-RT, `on_configure`) → 매 tick `PinocchioCache::Update()` → `Compute` (RT) 이다.

```cpp
rtc::tsid::TSIDController tsid;
tsid.Init(model, robot_info, yaml["tsid"]);               // 태스크·제약·solver 구성
tsid.ApplyPhasePreset("approach");                         // phase 전환
const rtc::tsid::CommandOutput out =
    tsid.Compute(state, ref, cache, contacts);             // out.tau, out.qp_converged
```

## 인터페이스

공개 헤더는 `include/rtc_tsid/` 아래에 있다. 시그니처·계약은 각 헤더의 주석이 SSoT 이고, 아래는 색인이다.

### TSIDController

`controller/tsid_controller.hpp`. `ControllerBase` 를 상속한 `final` 클래스다.

| 메서드 | 설명 |
|---|---|
| `Init(model, robot_info, config)` | Pinocchio 모델 + YAML 로 초기화 (non-RT, 잘못된 설정은 throw) |
| `Compute(state, ref, cache, contacts)` | QP 를 풀어 `CommandOutput` 반환 (`noexcept`) |
| `Reset()` · `Name()` | 내부 상태 초기화 · `"tsid"` |
| `ApplyPhasePreset(name)` | YAML 의 phase preset 적용 (`noexcept`) |
| `ActivateContact(idx)` · `DeactivateContact(idx)` | 접촉점 활성화·비활성화 (`noexcept`) |
| `Formulation()` | 내부 formulation 참조 |

`CommandOutput` 은 `tau` (구동 토크) · `a_opt` · `lambda_opt` (고정 크기, 비활성 slot ≈ 0) · `qp_converged` · `solve_time_us` · `solve_levels` (WQP 1, HQP N) · `qp_status` · `qp_iterations` · `qp_non_finite` 를 갖는다. 입력 타입은 `types/wbc_types.hpp`.

### Formulation

| Formulation | 설명 | solve_levels |
|---|---|---|
| WQP | 모든 태스크를 가중치로 합쳐 단일 QP 로 푼다 | 1 |
| HQP | 우선순위별 cascaded QP — 상위 레벨의 null-space 에서 하위를 최적화한다 | N |

`CreateFormulation()` (`formulation/formulation_factory.hpp`) 이 YAML `formulation_type` (`"wqp"` / `"hqp"`) 으로 고른다. 그 외 값은 throw 한다.

### 태스크

`TaskBase` 를 상속하며 `compute_residual()` 이 잔차와 Jacobian 을 낸다. 각 태스크는 `weight` · `priority` 를 갖는다.

| 태스크 | 설명 | residual_dim |
|---|---|---|
| `PostureTask` | 관절 자세 추종 | nv |
| `SE3Task` | TCP/EE pose 추종 (6D, 축 mask, `log3` 특이점 보호) | mask 활성 축 수 |
| `CoMTask` | CoM 위치 추종, `com_drift` 보상 | 3 |
| `ForceTask` | 접촉력 reference 추종 (active contact 만) | Σ active contact_dim |
| `MomentumTask` | angular momentum 정규화 또는 full momentum 추종 | 3 / 6 |
| `ContactConsistencyTask` | no-slip 접촉 가속도를 soft cost 로 (`ContactManagerConfig` 미주입 시 no-op) | Σ active contact_dim |
| `ObjectWrenchTask` | 접촉 합력 wrench 를 desired 로 | 6 / 0 |
| `InternalForceTask` | Null(G) 안의 squeeze force 를 desired 로 | n_λ / 0 |
| `ObjectSE3Task` | Object SE(3) pose 추종 (kinematic, `ObjectStateProvider` 필요) | mask 활성 축 수 / 0 |

접촉 활성화는 `ContactState::Entry` 의 `activation` ramp 로 부드럽게 전환된다. 비활성 접촉의 row 는 cost 와 제약 양쪽에서 빠진다 (`SetActivationTarget` · `kActivationDeadband`, `types/wbc_types.hpp`). Object 계열 태스크는 `contact/` 의 `ContactManager` · `GraspCache` · `ObjectStateProvider` 위에서 동작한다.

### 제약

| 제약 | 타입 | 설명 |
|---|---|---|
| `EOMConstraint` | 등식 | 운동 방정식 `M q̈ + h = Sᵀτ + J_cᵀλ`. mixed contact 면 `set_contact_manager` 필수 |
| `ContactConstraint` | 등식 | 접촉점 가속도 = 0 |
| `FrictionConeConstraint` | 부등식 | 쿨롱 마찰 원뿔 선형 근사 (point / surface) |
| `TorqueLimitConstraint` | 부등식 | URDF effort limit × YAML `tau_scale` ∈ (0, 1] (범위 위반 시 init throw) |
| `JointLimitConstraint` | 부등식 | 가속도 레벨 관절 한계 viability |

### ClikReferenceGenerator

`kinematics/clik_reference.hpp`. TSID 가속도 적분의 대안으로, 측정 상태에 anchor 한 velocity-level CLIK one-step reference `(q_ref, v_ref)` 를 단일 box-constrained QP 로 만든다. Robot-agnostic 이며 arm/hand velocity index 는 컨트롤러가 주입한다. 호출 형태는 단일 SE3 과제 · 위치 + 접근축 과제 · 다중 frame (`MultiFrameInput`) 세 가지다.

옵션 (속도·위치 box, 평활, twist feedforward, 가속 제약, 제동 거리 한계, 상대 과제, 자세 군), 진단 (`LastSolve()`), frame 규약, 입력 검증(NUM-7) 의 사양은 **전부 `clik_reference.hpp` 의 머리 주석과 `Config` 필드 주석**이 갖는다. 옵션은 기본 off 이고 off 에서는 기존 출력과 비트 일치한다 (`test_clik_golden`). 보조 헤더는 `kinematics/se3_error.hpp` (kinematics·dynamics 공유 SE(3) 오차, 식은 [L5](../docs/dynamic_catching/ref/L5_joint_cmd.md)) · `kinematics/relative_jacobian.hpp` 다.

### QPSolverWrapper

`solver/qp_solver_wrapper.hpp`. ProxSuite dense QP 래퍼로 `Init()` 에서 한 번 할당하고 `Solve()` 는 `operator new` · Eigen 할당이 없다 (ProxQP 내부의 C 수준 할당은 남는다). 비유한 해는 `converged = false` 로 보고하고 다음 solve 를 cold start 한다. `ResetWarmStart()` · `EqualityDual()` · `InequalityDual()` 과 opt-in 설정 (`update_preconditioner` · `dense_backend` · `eps_primal_inf`) 의 의미는 헤더 주석을 본다.

## 설정

YAML 은 소비 패키지가 갖는다 — 실제 사례는 [demo_wbc_controller.yaml](../integrated_bringup/config/ur5e_p1b/controllers/demo_wbc_controller.yaml) 의 `tsid:` 블록이다. 이 패키지는 설정 파일을 두지 않는다. 기본값은 코드가 아니라 그 파일이 갖는다.

### formulation 과 solver

```yaml
tsid:
  formulation_type: "wqp"     # 또는 "hqp"
  wqp:                        # HQP 는 hqp.solver_per_level (레벨마다 같은 키)
    solver:
      max_iter: 20            # 코드 기본값: WQP 20, HQP 10
      max_iter_in: 100        # inner loop 상한 — RT tick 의 무한 루프 방지
      eps_abs: 1.0e-6
      eps_rel: 0.0            # 0 → abs-only
      verbose: false          # per-iteration log (debug 전용)
```

5개 키는 `WQPFormulation::init` / `HQPFormulation::init` 이 읽고, 누락하면 코드 기본값을 쓴다.

### 태스크 키

태스크는 `tsid.tasks.<key>` 아래에 둔다. `SE3Task` 는 `name` 이 YAML key 와 같아야 phase preset 과 reference 갱신이 찾는다.

```yaml
se3_tcp:
  frame: "<tip_frame>"         # 제어 대상 frame (URDF frame 이름)
  base_frame: "<base_frame>"   # 필수 — 누락하면 init 이 std::runtime_error. "universe" 면 fast-path
  mask: [1, 1, 1, 1, 1, 1]     # [vx, vy, vz, wx, wy, wz]
  kp: [100, 100, 100, 50, 50, 50]
  kd: [20, 20, 20, 10, 10, 10]
  weight: 100.0
  priority: 0
```

`base_frame` 이 universe 가 아니면 오차 · `v_des` · `a_ff` · Jacobian 행이 모두 **base 축**이다 (base 는 한 tick 안에서 world 에 고정으로 본다). `CoMTask` 는 `kp` · `kd` (각 3) · `weight` · `priority`, `MomentumTask` 는 `mode` (`"angular_regularize"` / `"full_track"`) · `weight` · `priority` 를 받는다.

### 제약 키

```yaml
joint_limit:
  dt: 0.002              # 제어 주기 [s]
  position_margin: 0.05  # [rad]
  velocity_margin: 0.1   # [rad/s]
```

Floating-base 모델의 처음 6 DoF (free-flyer) 는 무제약이다. margin 은 관절별로 clamp 하므로 범위가 좁은 관절 (gripper finger) 에서도 안전하다. 접촉 활성화 ramp 시간의 기본값 `tsid.contacts_default_ramp_sec` 은 rtc_tsid 가 아니라 소비 컨트롤러가 읽는다.

## 디렉토리

`include/rtc_tsid/` 하위는 `core` · `controller` · `formulation` · `tasks` · `constraints` · `types` · `contact` · `kinematics` · `solver` 다.

```
rtc_tsid/
├── include/rtc_tsid/   # 공개 헤더 (하위: core, tasks, ...)
├── src/                # 구현
└── test/               # GTest (urdf/, golden/ fixture)
```

`CMakeLists.txt` 의 `add_tsid_gtest` 등록이 테스트 파일과 바이너리 대응의 SSoT 다. 한 바이너리를 여러 파일이 공유하므로 한 파일만 돌리려면 그 바이너리에 `--gtest_filter` 를 준다. 합성 17-DoF tree (`test/urdf/tree_17dof.urdf`) 는 어떤 로봇의 모델도 아니다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `Unknown formulation_type` | `formulation_type` 이 `wqp`/`hqp` 가 아니다 | YAML 값 수정 |
| `SE3Task` init 이 `std::runtime_error` | `base_frame` 누락 (strict) | `base_frame` 을 쓴다. world 기준이면 `"universe"` |
| 태스크가 reference 를 못 받고 identity 를 추종 | `name` 이 YAML key 와 다르다 | `name` 을 key 와 같게 |
| `qp_converged == false` | 해가 비유한이다 (ProxQP 가 SOLVED 여도 그렇게 보고하고, 다음 solve 는 warm start 없이 시작한다) | 입력 (`M` · `h` · `q` · `v` · 목표) 의 유한성 확인 |
| 거짓 `PRIMAL_INFEASIBLE` | 전처리기가 placeholder 스케일 | `QPSolverConfig::update_preconditioner` (헤더 주석) |

## 더 읽기

- CLIK 식·구조 결정: [L5 joint cmd](../docs/dynamic_catching/ref/L5_joint_cmd.md) · [다중 frame CLIK 정식화](../docs/dynamic_catching/ref/mpc_multiframe_clik_formulation.md)
- NUM-7 입력 유한성 가드의 근거·실측: [invariants-rationale.md](../docs/reference/invariants-rationale.md) §NUM-7 의 실측
- 테스트 명령·카운트 측정: [docs/testing.md](../docs/testing.md) §Test 측정
- 규칙: [invariants.md](../agent_docs/invariants.md) (RT · ARCH-1 · ARCH-2)

## 의존성

[package.xml](package.xml) · MIT. `pinocchio` (강체 동역학) · `proxsuite` (dense QP) · `rtc_math` (공유 SE(3) 오차) · `rtc_urdf_bridge` (`ReducedDynamicsProvider` 계약) · `yaml-cpp` · `eigen3_cmake_module`. `rtc_base` 는 test 전용이다 (`malloc_gate.hpp`).
