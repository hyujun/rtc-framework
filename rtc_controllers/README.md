# rtc_controllers

URDF 기반 매니퓰레이터용 제어 법칙·궤적·estimator 라이브러리. **유형**: 라이브러리 패키지 (정적 라이브러리 `librtc_controllers.a`, 노드 없음)

## 개요

관절/태스크 공간 제어 법칙, compliance 계열 법칙과 공용 커널, 적응형 PI 힘 제어 `GraspController`, 5차 다항식 궤적 생성기, in-thread estimator, 공 포구(`catching/`) 수치 코어를 **헤더 단위 함수·클래스**로 제공합니다. 노드·publisher·subscription 을 만들지 않고 `RTControllerInterface` 도 상속하지 않습니다. 법칙을 tick 마다 부르는 바인딩은 [integrated_bringup](../integrated_bringup/README.md) 같은 integration 패키지가 소유합니다 (경계 규범: [design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms).

### 사용 모델

- 이 패키지는 라이브러리 심볼만 제공하고 `RTC_REGISTER_CONTROLLER` 로 아무것도 등록하지 않습니다. 다운스트림 `<robot>_bringup` 이 바인딩 클래스를 만들어 이 패키지의 법칙을 멤버로 호출하고, 자체 `controller_registration.cpp` 에서 등록하며, 자체 `config/controllers/<subdir>/<key>.yaml` 을 가집니다.
- 바인딩은 configure 시 `params/` 의 `ParseXxxParams()` 로 YAML 을 한 번 파싱하고(비-RT, throw 가능), tick 에서 법칙 함수(`noexcept`, heap-free)를 부릅니다.
- `examples/controllers/` 의 YAML 은 `<robot>` placeholder 를 쓰는 **참고용**이라 그대로는 로드되지 않습니다 (의도된 동작). production YAML 은 `integrated_bringup/config/<robot>/controllers/` 에 두고 여기에는 새 파일을 추가하지 않습니다.
- 런타임 게인 파라미터 채널은 이 패키지에 없습니다. 게인 스냅샷(SeqLock)·타깃 전달(SPSC)은 바인딩과 `rtc_base` 몫입니다.

### 제어 법칙과 헤더

모든 경로는 `include/rtc_controllers/` 기준입니다. 법칙과 YAML 스키마 POD(`params/`)가 짝입니다.

| 계열 | 헤더 | 설명 |
|---|---|---|
| joint | `joint/joint_pd_law.hpp` · `params/joint_pd_params.hpp` | 관절 PD (+ 중력/코리올리 피드포워드, torque lane 한정) |
| joint | `joint/posture_law.hpp` | 영공간 posture 법칙 `τ₀ = Kp(q_null−q) − Kd q̇` |
| task | `task/task_vel_law.hpp` · `params/clik_params.hpp` | CLIK 태스크 속도 법칙 `K_p ⊙ e + ν_ff` (Position 출력) |
| task | `task/task_accel_law.hpp` · `params/osc_params.hpp` | OSC 태스크 가속도 법칙 (Torque 출력, task inertia Λ 사용) |
| compliance | `compliance/impedance_law.hpp` · `inertia_shaping.hpp` · `params/task_impedance_params.hpp` | 태스크 임피던스 (Jacobian-transpose, 선택적 관성 성형) |
| compliance | `compliance/admittance_integrator.hpp` · `params/task_admittance_params.hpp` | 태스크 어드미턴스 (힘 입력 → compliant frame 적분 → Position 출력) |
| compliance | `compliance/bandwidth_separation.hpp` · `params/cascaded_compliance_params.hpp` | 캐스케이드 (outer 어드미턴스 + inner 임피던스) 의 대역폭 분리 판정 |
| compliance | `compliance/differential_ik.hpp` · `task_dynamics.hpp` | 공용 커널: σ_min 적응형 DLS 미분 IK · Λ_S/h 조립 |
| compliance | `compliance/external_wrench.hpp` · `wrench_conditioning.hpp` · `wrench_pipeline.hpp` | 외부 F/T wrench 입력 계약 · 조건화 순서 · tick 파이프라인 |
| compliance | `compliance/safety_limiter.hpp` · `torque_estop.hpp` · `joint_command_tail.hpp` · `compliance_state_machine.hpp` | 출력 안전층 · torque E-STOP hold · 관절 명령 tail · 상태 머신 |
| grasp | `grasp/grasp_controller.hpp` · `grasp_types.hpp` · `grasp_state.hpp` | `GraspController` 적응형 PI 힘 제어 · 파라미터/FSM 타입 · tick 상태 POD |
| grasp | `grasp/pull_force_estimator.hpp` | 당김 힘 estimator |
| estimation | `estimation/momentum_observer.hpp` · `payload_estimator.hpp` · `inertial_estimator.hpp` | 모멘텀 observer 잔차 · payload 질량/COM 추정 · 관성 회귀 |
| trajectory | `trajectory/*.hpp` | 5차 다항식 궤적 ([아래](#궤적-생성기)) |
| inference | `inference/policy_io.hpp` · `reach_gate.hpp` · `params/policy_io_params.hpp` · `params/reach_gate_params.hpp` | 학습 정책 tensor marshalling · reach gate |
| 공용 | `gain_floor.hpp` | 비음수 게인 floor `FloorNonNegativeGain` (NUM-6) |

`catching/` 은 공 포구 플래너·감독자의 수치 코어입니다 (설계: [docs/dynamic_catching/](../docs/dynamic_catching/README.md)).

| 묶음 | 헤더 (`catching/`) |
|---|---|
| 데이터 계약 | `trajectory.hpp` · `time_types.hpp` · `planner_io.hpp` · `traj_ingress.hpp` · `traj_sampler.hpp` · `segment_ring.hpp` · `ball_node_samples.hpp` |
| 도달·포구 판정 | `catch_pose_ik.hpp` · `time_feasibility.hpp` · `reach_bound.hpp` · `rank_gates.hpp` · `unit_speed.hpp` · `plane_crossing.hpp` |
| 탐색 | `catch_search.hpp` · `grid_catch_search.hpp` · `nlp_catch_search.hpp` · `nlp_catch_screening.hpp` · `gamma_rollout.hpp` · `search_stats.hpp` |
| segment MPC | `mpc_segment_core*.hpp` · `mpc_segment_planner.hpp` · `mpc_docking_*.hpp` · `mpc_block_grid.hpp` · `jerk_segment.hpp` · `segment_planner.hpp` · `planner_cycle.hpp` · `node_follower.hpp` |
| 손·감독자 | `hand_sequencer.hpp` · `hand_capture.hpp` · `contact_debounce.hpp` · `decel_target.hpp` · `soft_catch.hpp` · `transition_table.hpp` · `joint_home.hpp` · `joint_stop.hpp` · `joint_groups.hpp` |
| 파라미터 | `catching_params.hpp` · `catch_pose_ik_params.hpp` · `planner_params.hpp` · `docking_params.hpp` · `docking_config.hpp` |
| 오프라인 도구 | `catch_pose_ik_batch.hpp` · `catch_gate_batch.hpp` · `catch_search_batch.hpp` |

compliance 3종(임피던스·어드미턴스·캐스케이드)의 법칙·부호 규약·wrench 계약·게인 단위는 [docs/compliance-conventions.md](docs/compliance-conventions.md) 가 SSoT 입니다. 게인 단위는 법칙마다 다릅니다 (CLIK 속도형 `[1/s]`, OSC 가속도형 `[1/s²]`, 임피던스 힘형).

## 빠른 시작

빌드·테스트는 [루트 README](../README.md#빠른-시작) 를 따릅니다. 이 패키지만 다시 빌드할 때는 `./build.sh --tests -p rtc_controllers`, 테스트는 `colcon test --packages-select rtc_controllers` 입니다 ([docs/testing.md](../docs/testing.md)).

소비 패키지의 `CMakeLists.txt` / `package.xml`:

```cmake
find_package(rtc_controllers REQUIRED)
ament_target_dependencies(my_binding rtc_controllers)
```

```xml
<depend>rtc_controllers</depend>
```

관절 PD 법칙을 tick 에서 부르는 최소 예 (position lane, 중력/코리올리 보상 끔):

```cpp
#include "rtc_controllers/joint/joint_pd_law.hpp"

std::array<double, 6> kp{200, 200, 150, 120, 120, 120}, kd{30, 30, 25, 20, 20, 20};
std::array<double, 6> q{}, ref_pos{}, ref_vel{}, prev_err{}, cmd{};  // 매 tick 갱신

const rtc::joint::JointPdGainsView gains{.kp = kp, .kd = kd};
const rtc::joint::JointPdInputs in{
    .q = q, .ref_pos = ref_pos, .ref_vel = ref_vel, .gravity = {}, .coriolis = {}};
rtc::joint::ComputeJointPdCommand(gains, in, /*dt=*/0.002, /*nq=*/6, /*nc0=*/6,
                                  rtc::CommandType::kPosition, prev_err, cmd);
```

## 인터페이스

### GraspController

가변 개수·가변 DoF 핸드를 위한 위치 제어 기반 적응형 PI 힘 제어기입니다. 스칼라 `s ∈ [0,1]` 로 finger 별 open/close 자세를 보간하고 외부 루프 PI 가 `s` 를 움직입니다. 직접 등록되지 않고 integration 패키지의 컨트롤러가 멤버로 소유합니다 (`grasp_controller_type: "force_pi"`).

| 호출 | 설명 |
|---|---|
| `Init(std::span<const FingerConfig>, const GraspParams&)` | non-RT. finger 수는 `min(configs.size(), kMaxGraspFingers)`, finger 별 DoF 는 `FingerConfig::dof` |
| `Update(std::span<const double> f_filtered, double dt) noexcept` | RT. finger 별 **필터된** 힘 크기 [N] → `GraspJointCommands`. 필터링은 호출자 몫 |
| `CommandGrasp(target_force)` · `CommandRelease()` | 다른 스레드에서 호출 가능 (atomic 플래그) |
| `Reset() noexcept` | FSM 을 `kIdle` 로 되돌림. 중단된 파지를 정리할 때 |
| `phase()` · `finger_states()` | 관측용 |

FSM(`GraspPhase`): `kIdle → kApproaching → kContact → kForceControl → kHolding → kReleasing → kIdle`. `thumb_finger_index` 가 가리키는 finger + 다른 finger 하나가 접촉하면 Contact 로 전이합니다. 슬립·힘 급감은 `kHolding` 에서 grip tightening 으로 처리합니다. 주요 `GraspParams` (기본값은 `grasp_types.hpp`):

| 키 | 기본 | 의미 |
|---|---|---|
| `f_target` · `f_contact_threshold` · `f_ramp_rate` | 2.0 N · 0.2 N · 1.0 N/s | 목표 파지력 · 접촉 감지 · 힘 레퍼런스 램프 |
| `Kp_base` · `Ki_base` · `integral_clamp` | 0.02 · 0.002 · 0.1 | PI 게인 · anti-windup |
| `approach_speed` · `release_speed` · `ds_max` | 0.2 · 0.3 · 0.05 [1/s] | `s` 변화율 |
| `delta_s_max` | 0.15 | 접촉 후 최대 변형 (이를 넘으면 closing 금지) |
| `thumb_finger_index` | 0 | Contact 전이가 필수로 요구하는 finger 슬롯 (호출자가 지정) |

적응 이득 스케줄링은 `K_est_max` 가 seed 와 같아 꺼져 있습니다. 튜닝 절차·근거는 [docs/grasp_tuning_guide.md](docs/grasp_tuning_guide.md) 입니다.

### 궤적 생성기

`trajectory/` 의 생성기는 모두 5차 다항식 기반이며 `compute(t)` 는 RT-safe 입니다. 시간은 `[0, T]` 로 클램프됩니다.

| 클래스 (`trajectory/`) | 공간 | 입력 | 연속성 |
|---|---|---|---|
| `QuinticPolynomial` (`trajectory_utils.hpp`) | 스칼라 | 경계 pos/vel/acc | C2 |
| `JointSpaceTrajectory<N>` (`joint_space_trajectory.hpp`) | 관절 | start/goal `State` + duration | C2 |
| `TaskSpaceTrajectory` (`task_space_trajectory.hpp`) | SE(3) | start/goal pose·velocity + duration | C2 |
| `QuinticBlendTrajectory<N>` (`quintic_blend_trajectory.hpp`) | 관절 | via-point (최대 `kMaxWaypoints = 32`), rest-to-rest | via-point 에서 C2 |
| `QuinticSplineTrajectory<N>` (`quintic_spline_trajectory.hpp`) | 관절 | via-point, natural 또는 clamped 경계 | 전역 C4 |
| `TaskSpaceBlendTrajectory` (`task_space_blend_trajectory.hpp`) | SE(3) | via-point, rest-to-rest | via-point 에서 C2 |
| `TaskSpaceSplineTrajectory` (`task_space_spline_trajectory.hpp`) | SE(3) | via-point, natural 또는 clamped 경계 | 전역 C4 |

SE(3) 생성기는 `log6` tangent space 에서 보간하고 `pose(t) = start * exp6(p(t))` 로 복원합니다. 스플라인은 `initialize()` 에서 선형 시스템을 풀므로(non-RT) via-point 수의 3제곱 비용이 듭니다. `State` 의 `velocity`·`acceleration` 은 LOCAL 표현이며, world-aligned 값은 `TaskSpaceTrajectory::State::AlignedVelocity()` · `AlignedClassicalAcceleration()` 입니다.

### 오프라인 CLI

모두 로봇을 모르는 판정 도구이며 모델·프레임·후보는 argv 로 받습니다. 옵션은 `--help` 로 확인합니다.

| 실행 파일 | 용도 |
|---|---|
| `catch_pose_ik_batch` | 포구 자세 IK 가능 여부 (catchability map) 판정 |
| `catch_gate_batch` | 도달 시간·γ 게이트 판정 (gate-catchable map) |
| `catch_search_batch` | 플래너 wake 의 catch search 를 오프라인으로 재생 |

```bash
ros2 run rtc_controllers catch_pose_ik_batch --help
```

## 설정

`examples/controllers/` 의 YAML 이 키·단위·기본값의 reference 입니다 (키는 컨트롤러 이름 아래에 둡니다. `topics:` 섹션은 base 의 `RTControllerInterface::LoadConfig()` 가 파싱합니다). 파서는 모르는 키를 조용히 무시하지만, 아는 키를 틀린 길이로 적으면 configure 가 실패합니다 (`double[3]` 게인은 정확히 3원소).

| 파일 (`examples/controllers/`) | 법칙 | 핵심 키 |
|---|---|---|
| `direct/joint_pd_controller.yaml` | 관절 PD (Torque) | `kp` `kd` `enable_gravity_compensation` `enable_coriolis_compensation` `trajectory_speed` `command_type` |
| `indirect/clik_controller.yaml` | CLIK (Position) | `kp_translation` `kp_rotation` `max_damping` `singularity_threshold` `null_kp` `control_6dof` |
| `direct/operational_space_controller.yaml` | OSC (Torque 고정) | `kp_pos` `kd_pos` `kp_rot` `kd_rot` `max_damping` `null_kp` `null_kd` `estop_damping` |

- 단일 `kp:` 배열이 아니라 CLIK 은 `kp_translation`/`kp_rotation`, OSC 는 `kp_pos`/`kd_pos`/`kp_rot`/`kd_rot` 입니다.
- 중력/코리올리 보상은 `command_type: torque` 에서만 허용됩니다 (비-torque 에서 켜면 거부).
- `max_damping`(λ_max) · `singularity_threshold`(σ₀) 는 σ_min 적응형 DLS 램프의 파라미터이고 로더가 floor 를 적용합니다 (NUM-1·2). 은퇴한 `damping` 키는 경고 후 무시됩니다.

## 디렉토리

```
rtc_controllers/
├── include/rtc_controllers/  -- 공개 헤더 (계열별)
├── src/                      -- params/ 파서 · 비헤더 구현
├── examples/controllers/     -- 참고용 YAML
├── docs/                     -- compliance · grasp 문서
└── test/                     -- gtest + 공용 fixture
```

`src/params/` 는 `ParseXxxParams()` 구현(YAML → POD)이고, `src/catching/` 의 `*_main.cpp` 3개가 오프라인 CLI 입니다.

## 더 읽기

- [docs/compliance-conventions.md](docs/compliance-conventions.md) — compliance 법칙·계약·부호 규약
- [docs/grasp_tuning_guide.md](docs/grasp_tuning_guide.md) — GraspController 튜닝
- [docs/dynamic_catching/](../docs/dynamic_catching/README.md) — 포구 설계·수식
- [design-principles.md](../agent_docs/design-principles.md) — 코어/바인딩 3계층 경계 · [invariants.md](../agent_docs/invariants.md) — RT/NUM 규칙
- [controllers-rationale.md](../docs/reference/controllers-rationale.md) — 코어 추출 근거
- 각 법칙의 수식·계약은 해당 헤더 상단 주석이 SSoT 입니다

## 의존성

[package.xml](package.xml): `rtc_base` `rtc_msgs` `rtc_math` `rtc_urdf_bridge` `rtc_tsid` (포구 QP) · Eigen3 · Pinocchio · yaml-cpp. `rtc_controller_interface` 는 의존하지 않습니다 (법칙은 프레임워크 계약을 모릅니다). 라이선스: MIT ([LICENSE](../LICENSE)).
