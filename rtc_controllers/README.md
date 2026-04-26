# rtc_controllers

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)
> 그래스프 튜닝: [docs/grasp_tuning_guide.md](docs/grasp_tuning_guide.md)

## 개요

RTC 프레임워크의 **내장 제어 알고리즘 구현체** 패키지입니다. `RTControllerInterface`를 상속하는 4개의 로봇 컨트롤러, 적응형 PI 힘 제어 그래스프 컨트롤러, 그리고 5차 다항식 기반 궤적 생성기(기본/블렌드/스플라인)를 제공합니다.

**컨트롤러 분류:**

| 컨트롤러 | 공간 | 출력 모드 | 카테고리 |
|---------|------|----------|----------|
| PController | 관절 공간 | Position (indirect) | `indirect/` |
| JointPDController | 관절 공간 | Torque (direct) | `direct/` |
| ClikController | 태스크 공간 | Position (indirect) | `indirect/` |
| OperationalSpaceController | 태스크 공간 | Torque (direct) | `direct/` |
| GraspController | 핸드 내부 | 적응형 PI 힘 제어 | `grasp/` |

---

## 패키지 구조

```
rtc_controllers/
├── CMakeLists.txt
├── package.xml
├── include/rtc_controllers/
│   ├── indirect/
│   │   ├── p_controller.hpp              -- 관절 공간 P 제어기
│   │   └── clik_controller.hpp           -- 태스크 공간 CLIK 제어기
│   ├── direct/
│   │   ├── joint_pd_controller.hpp       -- 관절 공간 PD + 동역학 보상
│   │   └── operational_space_controller.hpp -- 태스크 공간 OSC
│   ├── trajectory/
│   │   ├── trajectory_utils.hpp              -- 5차 다항식 궤적 (QuinticPolynomial)
│   │   ├── joint_space_trajectory.hpp        -- N-DOF 관절 공간 궤적 생성기
│   │   ├── task_space_trajectory.hpp         -- SE(3) 태스크 공간 궤적 생성기
│   │   ├── quintic_blend_trajectory.hpp      -- C2 via-point 블렌드 궤적
│   │   ├── quintic_spline_trajectory.hpp     -- C4 글로벌 스플라인 궤적
│   │   ├── task_space_blend_trajectory.hpp   -- SE(3) C2 via-point 블렌드
│   │   └── task_space_spline_trajectory.hpp  -- SE(3) C4 글로벌 스플라인
│   └── grasp/
│       ├── grasp_types.hpp                   -- 그래스프 상태 머신 타입/파라미터
│       └── grasp_controller.hpp              -- 적응형 PI 힘 제어 그래스프 컨트롤러
├── src/
│   ├── controller_registration.cpp           -- 4개 컨트롤러 자동 등록
│   └── controllers/
│       ├── indirect/
│       │   ├── p_controller.cpp
│       │   └── clik_controller.cpp
│       ├── direct/
│       │   ├── joint_pd_controller.cpp
│       │   └── operational_space_controller.cpp
│       └── grasp/
│           └── grasp_controller.cpp
└── config/controllers/
    ├── indirect/
    │   ├── p_controller.yaml             -- P 제어기 기본 설정
    │   └── clik_controller.yaml          -- CLIK 제어기 기본 설정
    └── direct/
        ├── joint_pd_controller.yaml      -- JointPD 제어기 기본 설정
        └── operational_space_controller.yaml -- OSC 제어기 기본 설정
```

---

## 컨트롤러 상세

### 1. PController (관절 공간 P 제어)

가장 단순한 비례 관절 위치 제어기입니다. 동역학 모델을 사용하지 않으며 빠른 응답이 필요한 기본 위치 제어에 적합합니다.

**제어 법칙:**

```
command[i] = current_pos[i] + kp[i] * (target[i] - current[i]) * dt
```

**파라미터:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp` | `double[6]` | `[120, 120, 100, 80, 80, 80]` | 관절별 비례 게인 |
| `command_type` | `string` | `"position"` | 출력 명령 타입 |

**특징:**
- 최소 계산량 (동역학 미사용)
- 위치 명령 출력 (증분 적분 방식)
- 출력 클램핑: `max_joint_velocity` (기본 2.0 rad/s)
- FK 계산으로 TCP 위치 진단 제공 (`actual_task_positions`)
- 궤적 생성기 미사용 (즉시 목표 추종)
- E-STOP: `estopped_` 원자적 플래그 기반, 활성화 시 현재 위치 유지 (hold position)
- 스레드 동기화 없음 (단일 스레드 전용)

```yaml
# config/controllers/indirect/p_controller.yaml
p_controller:
  kp: [120.0, 120.0, 100.0, 80.0, 80.0, 80.0]
  command_type: "position"
```

---

### 2. JointPDController (관절 공간 PD + 동역학 보상)

5차 다항식 궤적 생성과 선택적 중력/코리올리 보상을 포함하는 관절 공간 PD 토크 제어기입니다.

**제어 법칙:**

```
e[i]  = trajectory_position[i] - current_position[i]
de[i] = (e[i] - e_prev[i]) / dt

command[i] = kp[i] * e[i] + kd[i] * de[i]
           + ff_vel[i]                     (command_type != torque 일 때만)
           + g(q)[i]                       (enable_gravity_compensation = true)
           + C(q,v)*v[i]                   (enable_coriolis_compensation = true)
```

**파라미터:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp` | `double[6]` | `[200, 200, 150, 120, 120, 120]` | 비례 게인 |
| `kd` | `double[6]` | `[30, 30, 25, 20, 20, 20]` | 미분 게인 |
| `enable_gravity_compensation` | `bool` | `false` | 중력 보상 활성화 |
| `enable_coriolis_compensation` | `bool` | `false` | 코리올리 보상 활성화 |
| `trajectory_speed` | `double` | `1.0` | 궤적 최대 관절 속도 (rad/s) |
| `command_type` | `string` | `"torque"` | 출력 명령 타입 |

**알고리즘 흐름:**

1. `SetDeviceTarget()` -- 새 목표 수신 (mutex 보호)
2. 5차 다항식 궤적 초기화 (현재 -> 목표, duration = max_joint_distance / trajectory_speed)
3. Pinocchio로 FK, 중력 토크, 코리올리 행렬, 자코비안 계산 (`UpdateDynamics`)
4. 궤적 설정값 대비 PD 오차 계산 + 선택적 피드포워드/보상항
5. 명령 제한 적용 (토크 모드: max_joint_torque 기본 150 Nm / 위치 모드: max_joint_velocity 기본 2.0 rad/s)

**E-STOP:** `safe_position` (YAML 디바이스 설정에서 로드)으로 PD 제어를 통해 이동

```yaml
# config/controllers/direct/joint_pd_controller.yaml
joint_pd_controller:
  kp: [200.0, 200.0, 150.0, 120.0, 120.0, 120.0]
  kd: [30.0, 30.0, 25.0, 20.0, 20.0, 20.0]
  enable_gravity_compensation: false
  enable_coriolis_compensation: false
  trajectory_speed: 1.0
  command_type: "torque"
```

---

### 3. ClikController (태스크 공간 CLIK)

Closed-Loop Inverse Kinematics -- 감쇠 의사역행렬과 영공간 보조 태스크를 사용하는 태스크 공간 위치 제어기입니다. 3-DOF (위치만) 또는 6-DOF (위치+자세) 모드를 지원합니다.

**제어 법칙 (3-DOF 모드):**

```
pos_error  = traj_pos - FK(q)
J_pos      = J[0:3, :]                     (병진 자코비안, 3xnv)
J_pos^#    = J_pos^T (J_pos J_pos^T + lambda^2 I)^{-1}   (감쇠 의사역행렬, LDLT 분해)
N          = I - J_pos^# J_pos             (영공간 투영)

dq = kp * J_pos^# * pos_error + ff_vel
   + null_kp * N * (q_null - q)            (enable_null_space = true)

q_des += clamp(dq, +/-v_max) * dt          (trajectory 갱신 시 q_des = q_actual로 초기화)
q_cmd  = q_des
```

**제어 법칙 (6-DOF 모드):**

```
pos_error_6d[0:3] = traj_pos - FK(q)
pos_error_6d[3:6] = R_current * log6(T_current^{-1} * T_traj).angular   (SO(3) 로그)

J_full^#   = J_full^T (J_full J_full^T + lambda^2 I_6)^{-1}   (6x6 LDLT)

dq = kp * J_full^# * pos_error_6d + ff_vel_6d
q_des += clamp(dq, +/-v_max) * dt          (trajectory 갱신 시 q_des = q_actual로 초기화)
q_cmd  = q_des
```

**파라미터:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp` | `double[6]` | `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]` | 태스크 공간 비례 게인 |
| `damping` | `double` | `0.01` | 의사역행렬 감쇠 계수 (lambda) |
| `null_kp` | `double` | `0.5` | 영공간 보조 태스크 게인 |
| `enable_null_space` | `bool` | `true` | 영공간 관절 센터링 활성화 |
| `trajectory_speed` | `double` | `0.1` | 태스크 공간 궤적 최대 병진 속도 (m/s) |
| `control_6dof` | `bool` | `false` | 6-DOF (위치+자세) 제어 활성화 |
| `command_type` | `string` | `"position"` | 출력 명령 타입 |

**타겟 해석 방식:**

| 모드 | target[0:3] | target[3:6] |
|------|-------------|-------------|
| `control_6dof=false` | TCP 위치 (x, y, z) | 영공간 참조 관절 3-5 (rad) |
| `control_6dof=true` | TCP 위치 (x, y, z) | TCP 자세 (roll, pitch, yaw, ZYX) |

**핵심 기법:**
- LDLT 분해 -- 수치 안정적 의사역행렬 계산 (3x3 또는 6x6)
- SE(3) 궤적 보간 -- log6 기반 거리 계산 + TaskSpaceTrajectory
- Pinocchio 자코비안 -- `computeJointJacobians()` + `getJointJacobian(LOCAL_WORLD_ALIGNED)`
- 영공간 참조: `safe_position` (디바이스 설정에서 로드) 또는 기본값 `[0, -1.57, 1.57, -1.57, -1.57, 0]`

**E-STOP:** `safe_position`으로 관절 속도 제한 범위 내에서 위치 명령 이동

```yaml
# config/controllers/indirect/clik_controller.yaml
clik_controller:
  kp: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  damping: 0.01
  trajectory_speed: 0.1
  enable_null_space: true
  null_kp: 0.5
  control_6dof: false
  command_type: "position"
```

---

### 4. OperationalSpaceController (태스크 공간 6-DOF OSC)

전체 6-DOF 태스크 공간 PD 제어 + 피드포워드 궤적 속도 + 선택적 중력 보상을 포함하는 제어기입니다.

**제어 법칙:**

```
pos_error  = traj_pos - FK(q)
rot_error  = log3(R_traj * R_FK(q)^T)          (SO(3) 로그)
tcp_vel    = J * q_dot                          (현재 TCP 속도)

task_vel[0:3] = kp_pos * pos_error + traj_vel_lin - kd_pos * tcp_vel[0:3]
task_vel[3:6] = kp_rot * rot_error + traj_vel_ang - kd_rot * tcp_vel[3:6]

JJt    = J * J^T + lambda^2 I_6
J^#    = J^T * JJt^{-1}                        (PartialPivLU 분해)

dq     = J^# * task_vel  [+ g(q)]              (중력 보상, 선택)

q_cmd  = q + clamp(dq, +/-v_max) * dt
```

**파라미터:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp_pos` | `double[3]` | `[1.0, 1.0, 1.0]` | 위치 비례 게인 |
| `kd_pos` | `double[3]` | `[0.1, 0.1, 0.1]` | 위치 미분 게인 |
| `kp_rot` | `double[3]` | `[0.5, 0.5, 0.5]` | 자세 비례 게인 |
| `kd_rot` | `double[3]` | `[0.05, 0.05, 0.05]` | 자세 미분 게인 |
| `damping` | `double` | `0.01` | 의사역행렬 감쇠 계수 (lambda) |
| `enable_gravity_compensation` | `bool` | `false` | 중력 보상 활성화 |
| `trajectory_speed` | `double` | `0.1` | 위치 궤적 최대 병진 속도 (m/s) |
| `trajectory_angular_speed` | `double` | `0.5` | 자세 궤적 최대 회전 속도 (rad/s) |
| `command_type` | `string` | `"torque"` | 출력 명령 타입 |

**타겟 해석 방식:**
- `target[0:3]` = TCP 위치 (x, y, z) (m)
- `target[3:6]` = TCP 자세 (roll, pitch, yaw) (rad, ZYX 오일러)

**궤적 Duration 계산:**
```
duration = max(0.01, max(trans_dist / trajectory_speed, angular_dist / trajectory_angular_speed))
```

**핵심 기법:**
- PartialPivLU -- 고정 크기 6x6 LU 분해 (동적 할당 없음)
- SO(3) 로그 맵 -- 자세 오차를 축-각도(axis-angle)로 변환
- SE(3) 궤적 보간 -- 위치/자세 독립 속도 기반 궤적
- RPY -> 회전행렬 변환: ZYX 오일러 규약

**E-STOP:** `safe_position`으로 관절 속도 제한 범위 내에서 위치 명령 이동

```yaml
# config/controllers/direct/operational_space_controller.yaml
operational_space_controller:
  kp_pos: [1.0, 1.0, 1.0]
  kd_pos: [0.1, 0.1, 0.1]
  kp_rot: [0.5, 0.5, 0.5]
  kd_rot: [0.05, 0.05, 0.05]
  damping: 0.01
  enable_gravity_compensation: false
  trajectory_speed: 0.1
  trajectory_angular_speed: 0.5
  command_type: "torque"
```

---

### 5. GraspController (적응형 PI 힘 제어)

3-finger 핸드를 위한 위치 제어 기반 적응형 PI 힘 제어기입니다. 스칼라 그래스프 파라미터 `s in [0,1]`로 finger별 open/close 자세를 선형 보간하며, 외부 루프 PI 제어기와 온라인 강성(stiffness) 추정을 통해 목표 접촉력을 달성합니다. ROS2 독립적이며 `Init()` 호출 이후 RT-safe합니다.

> **내부 컨트롤러**: `RTC_REGISTER_CONTROLLER` 매크로로 직접 등록되지 않습니다. `DemoJointController`, `DemoTaskController`, `DemoWbcController` 등 상위 컨트롤러에서 `grasp_controller_type: "force_pi"` 설정으로 내부적으로 인스턴스화하여 사용합니다.

**상태 머신 (GraspPhase):**

```
  Idle ──[CommandGrasp()]──> Approaching ──[thumb+index 접촉]──> Contact
   ^                            |                                   |
   |                     [s=1.0, 미접촉]                    [settle_time 경과]
   |                            |                                   v
   +────────────────────────────+                            ForceControl
   ^                                                           |      |
   |                                                    [수렴]  |  [CommandRelease()]
   |                                                      v     |
   +──────[전원 open]───── Releasing <──[CommandRelease()]── Holding
```

| 단계 | 설명 |
|------|------|
| `kIdle` | 대기 상태, `s` 유지. `CommandGrasp()` 시 Approaching으로 전이 |
| `kApproaching` | `approach_speed` (1/s)로 `s`를 증가시키며 closing. `f_measured > f_contact_threshold`이면 접촉 감지. thumb(0)+index(1) 모두 접촉 시 Contact로 전이 |
| `kContact` | 안정화 대기 (`contact_settle_time` 초). 경과 후 ForceControl 진입 |
| `kForceControl` | PI 힘 제어 활성화. `f_desired`를 `f_ramp_rate` (N/s)로 `f_target`까지 램프. 모든 finger가 `settle_epsilon` 이내로 `settle_time` 이상 수렴하면 Holding 전이 |
| `kHolding` | 힘 유지 + 이상 감지. 슬립 (`df/dt < -df_slip_threshold`) 또는 힘 급감 시 grip tightening 적용 |
| `kReleasing` | `release_speed` (1/s)로 `s`를 감소. 모든 finger `s < 0.01`이면 Idle 복귀 |

**GraspParams (주요 파라미터):**

| 파라미터 | 타입 | 기본값 | 단위 | 설명 |
|---------|------|--------|------|------|
| `Kp_base` | `double` | `0.02` | 1/(N*s) | PI 비례 게인 기본값 |
| `Ki_base` | `double` | `0.002` | 1/(N*s^2) | PI 적분 게인 기본값 |
| `alpha_ema` | `double` | `0.95` | [0,1] | 강성 EMA 계수 (1에 가까울수록 느린 적응) |
| `beta` | `double` | `0.3` | -- | 적응 게인 감도 (높을수록 강한 물체에 게인 감소) |
| `f_contact_threshold` | `double` | `0.2` | N | 접촉 감지 힘 임계값 |
| `f_target` | `double` | `2.0` | N | 목표 파지력 |
| `f_ramp_rate` | `double` | `1.0` | N/s | 힘 레퍼런스 램프 속도 |
| `ds_max` | `double` | `0.05` | 1/s | 최대 ds/dt (s 변화율 제한) |
| `delta_s_max` | `double` | `0.15` | -- | 접촉 후 최대 변형 delta_s (변형 가드) |
| `integral_clamp` | `double` | `0.1` | -- | 적분기 포화 한계 |
| `approach_speed` | `double` | `0.2` | 1/s | Approaching 단계 ds/dt |
| `release_speed` | `double` | `0.3` | 1/s | Releasing 단계 ds/dt |
| `settle_epsilon` | `double` | `0.1` | N | 힘 수렴 판정 임계값 |
| `settle_time` | `double` | `0.3` | s | 수렴 유지 시간 |
| `contact_settle_time` | `double` | `0.1` | s | Contact 단계 안정화 대기 시간 |
| `df_slip_threshold` | `double` | `5.0` | N/s | 슬립 감지 df/dt 임계값 (음방향) |
| `grip_tightening_ratio` | `double` | `0.15` | -- | 슬립 시 힘 증가 비율 |
| `grip_decay_rate` | `double` | `0.1` | N/s | tightening 후 목표력으로 감쇄 속도 |
| `f_max_multiplier` | `double` | `2.0` | -- | 최대 허용 힘 = f_target * multiplier |
| `lpf_cutoff_hz` | `double` | `25.0` | Hz | Bessel 4차 LPF 차단 주파수 |
| `control_rate_hz` | `double` | `500.0` | Hz | 제어 루프 주파수 |

**Per-finger 제어 법칙 (적응형 PI):**

```
# 온라인 강성 추정 (EMA)
K_inst   = delta_f / delta_s                       (|delta_s| > 1e-6 일 때만)
K_est    = alpha_ema * K_est + (1 - alpha_ema) * K_inst   (K_inst > 0 일 때만)

# 적응 게인 스케줄링
gain_scale = 1.0 / (1.0 + beta * K_est)
Kp = Kp_base * gain_scale
Ki = Ki_base * gain_scale

# PI 제어 + anti-windup
e_f = f_desired - f_measured
integral_error += e_f * dt                         (integrator_frozen이 아닐 때)
integral_error  = clamp(integral_error, +/-integral_clamp)

ds = clamp(Kp * e_f + Ki * integral_error, +/-ds_max)

# 변형 가드 (deformation guard)
deformation = s - s_at_contact
if deformation >= delta_s_max:      ds = min(ds, 0)   # closing 금지
elif deformation >= 0.9*delta_s_max: ds *= remaining / (0.1*delta_s_max)   # 비례 감속

# 자세 보간
s += ds * dt
q[j] = (1 - s) * q_open[j] + s * q_close[j]       (j = 0..2, MCP_AA/MCP_FE/DIP_FE)
```

**RT 안전성:**

- `Update()` 메서드: `noexcept`, 매 제어 주기(500Hz) 호출
- 4차 Bessel LPF (`BesselFilterN<3>`)로 힘 신호 필터링 (RT 경로에서 동적 할당 없음)
- `CommandGrasp()` / `CommandRelease()`: `std::atomic<bool>` + `memory_order_release/acq_rel`로 크로스 스레드 명령 전달
- 모든 상태 (`FingerState`, `GraspParams`)는 `trivially_copyable` 구조체
- `Init()`은 non-RT (필터 계수 계산), 이후 모든 호출은 RT-safe
- 고정 크기 배열 (`std::array`) 사용, 동적 할당 없음

---

## 궤적 생성 (`trajectory/`)

모든 궤적 기반 컨트롤러(JointPD, CLIK, OSC)는 5차(quintic) 다항식 기반 궤적 생성기를 사용합니다.

### QuinticPolynomial (`trajectory_utils.hpp`)

시작/끝 위치, 속도, 가속도 경계 조건을 만족하는 5차 다항식입니다.

```
q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5

경계 조건: q(0) = q0, q_dot(0) = v0, q_ddot(0) = a0
          q(T) = qf, q_dot(T) = vf,  q_ddot(T) = af
```

> 시간 t는 `[0, T]` 범위로 클램핑되어 궤적 완료 후 오버슈트를 방지합니다.

**반환 구조체:** `TrajectoryState { pos, vel, acc }`

### JointSpaceTrajectory (`joint_space_trajectory.hpp`)

N-DOF 관절 공간 궤적 -- 각 관절에 독립 QuinticPolynomial을 적용합니다. `JointPDController`에서 사용합니다.

| 메서드 | 설명 |
|--------|------|
| `initialize(start_state, goal_state, duration)` | 시작/목표 상태 (위치, 속도, 가속도) 기반 궤적 초기화 |
| `compute(t)` | 시간 t에서의 위치/속도/가속도 반환 |
| `duration()` | 궤적 지속 시간 |

**Duration 계산 (JointPDController):** `max(0.01, max_joint_distance / trajectory_speed)`

### TaskSpaceTrajectory (`task_space_trajectory.hpp`)

SE(3) 태스크 공간 궤적 -- log6 기반 tangent space에서 6-DOF에 각각 QuinticPolynomial을 적용합니다. `ClikController`와 `OperationalSpaceController`에서 사용합니다.

| 메서드 | 설명 |
|--------|------|
| `initialize(start_pose, start_velocity, goal_pose, goal_velocity, duration)` | SE(3) 궤적 초기화 |
| `compute(t)` | 시간 t에서의 SE(3) pose, Motion velocity, Motion acceleration 반환 |
| `duration()` | 궤적 지속 시간 |

**보간 방식:**
1. `delta_X = log6(start_pose^{-1} * goal_pose)` -- tangent space 변위 계산
2. 각 6-DOF 성분에 QuinticPolynomial 적용 (시작 0 -> 목표 delta_X)
3. `pose(t) = start_pose * exp6(p(t))` -- SE(3) 지수 맵으로 복원
4. `velocity(t) = Jexp6(delta_X(t)) * v(t)` -- 우측 자코비안으로 속도 변환

### 다중 웨이포인트 궤적 (Multi-Waypoint Trajectories)

단일 시작-목표 궤적 외에, 2개 이상의 via-point를 경유하는 4가지 다중 웨이포인트 궤적 생성기를 제공합니다. 모든 궤적은 최대 `kMaxWaypoints = 32`개의 웨이포인트를 지원하며, `initialize()`는 non-RT, `compute()`는 RT-safe (`noexcept`)입니다.

#### QuinticBlendTrajectory (`quintic_blend_trajectory.hpp`)

N-DOF 관절 공간 다중 웨이포인트 블렌드 궤적입니다. 각 세그먼트를 5차 다항식으로 생성하며, via-point에서 **C2 연속성**(위치, 속도, 가속도 연속)을 보장합니다.

- **Via-point 속도 계산**: 인접 세그먼트 평균 속도 `v[i] = 0.5 * (v_prev + v_next)`
- **경계 조건**: Rest-to-rest (시작/끝 속도 = 0, 가속도 = 0)
- **세그먼트 탐색**: 선형 검색 (O(N), N <= 32)
- **웨이포인트 < 2**: 첫 번째 웨이포인트 위치에서 hold

```cpp
QuinticBlendTrajectory<6> traj;
traj.initialize(waypoints, num_waypoints);      // non-RT
auto state = traj.compute(time);                // RT-safe
// state.positions, state.velocities, state.accelerations
```

| 메서드 | 설명 |
|--------|------|
| `initialize(waypoints, num_waypoints)` | 웨이포인트 배열로 블렌드 궤적 초기화 (non-RT) |
| `compute(time)` | 시간 t에서 위치/속도/가속도 반환 (RT-safe, `noexcept`) |
| `duration()` | 전체 궤적 지속 시간 |
| `num_segments()` | 세그먼트 수 (= 웨이포인트 수 - 1) |

#### QuinticSplineTrajectory (`quintic_spline_trajectory.hpp`)

N-DOF 관절 공간 글로벌 스플라인 궤적입니다. 모든 내부 knot에서 **C4 연속성**(위치, 속도, 가속도, 저크, 스냅 연속)을 보장합니다.

- **풀이 방식**: 6*(N-1) 미지수 선형 시스템 (PartialPivLU 분해)
  - 조건: N개 위치 보간 + 4개 경계 조건 + 5*(N-2)개 내부 연속성 = 6*(N-1)
- **경계 조건**: Natural (vel=0, acc=0) 또는 Clamped (사용자 지정 시작/끝 속도, 가속도)
- **2-웨이포인트 특수 케이스**: 단일 세그먼트 직접 5차 다항식 (선형 시스템 없이)

```cpp
QuinticSplineTrajectory<6> traj;
traj.initialize(waypoints, num_waypoints);                   // natural
traj.initialize(waypoints, n, start_vel, start_acc,          // clamped
                end_vel, end_acc);
auto state = traj.compute(time);
```

| 메서드 | 설명 |
|--------|------|
| `initialize(waypoints, num_waypoints)` | Natural 스플라인 (시작/끝 vel=0, acc=0) |
| `initialize(waypoints, n, start_vel, start_acc, end_vel, end_acc)` | Clamped 스플라인 (사용자 경계 조건) |
| `compute(time)` | 시간 t에서 위치/속도/가속도 반환 (RT-safe, `noexcept`) |
| `duration()` | 전체 궤적 지속 시간 |

#### TaskSpaceBlendTrajectory (`task_space_blend_trajectory.hpp`)

SE(3) 태스크 공간 다중 웨이포인트 블렌드 궤적입니다. 각 세그먼트의 시작 pose 로컬 tangent space에서 5차 다항식 보간하며, via-point에서 **C2 연속성**을 보장합니다.

- **Via-point 속도**: 도착/출발 속도의 평균 (프레임 변환 적용: 도착 속도를 해당 웨이포인트 프레임으로 Adjoint 변환)
- **경계 조건**: Rest-to-rest (시작/끝 속도 = 0)
- **보간**: `pose(t) = start_pose[seg] * exp6(p(t))`, 속도 = `Jexp6(delta_X) * v(t)`

```cpp
TaskSpaceBlendTrajectory traj;
traj.initialize(waypoints, num_waypoints);
auto state = traj.compute(time);
// state.pose (SE3), state.velocity (Motion), state.acceleration (Motion)
```

| 메서드 | 설명 |
|--------|------|
| `initialize(waypoints, num_waypoints)` | SE(3) 웨이포인트로 블렌드 궤적 초기화 (non-RT) |
| `compute(time)` | 시간 t에서 SE(3) pose, Motion velocity/acceleration 반환 (RT-safe) |
| `duration()` | 전체 궤적 지속 시간 |

#### TaskSpaceSplineTrajectory (`task_space_spline_trajectory.hpp`)

SE(3) 태스크 공간 글로벌 스플라인 궤적으로 **C4 연속성**을 보장합니다. 첫 번째 웨이포인트 프레임을 공통 참조 프레임으로 사용하여 6D 선형 시스템을 풀고, 결과를 각 세그먼트의 로컬 tangent space 다항식으로 변환합니다.

- **풀이 방식**: 누적 tangent 벡터(`cum_pos[k] = log6(wp[0]^{-1} * wp[k])`)를 wp[0] 프레임에서 계산 -> 글로벌 6D 스플라인 시스템 풀이 (PartialPivLU) -> Adjoint 변환으로 per-segment 로컬 계수 추출
- **경계 조건**: Natural (vel=0, acc=0) 또는 Clamped (로컬 프레임에서 지정)
- **속도 변환**: 경계 속도는 `SE3::act()` / `SE3::actInv()`로 프레임 간 변환

```cpp
TaskSpaceSplineTrajectory traj;
traj.initialize(waypoints, num_waypoints);                   // natural
traj.initialize(waypoints, n, start_vel, start_acc,          // clamped
                end_vel, end_acc);
auto state = traj.compute(time);
```

| 메서드 | 설명 |
|--------|------|
| `initialize(waypoints, num_waypoints)` | Natural SE(3) 스플라인 (시작/끝 vel=0, acc=0) |
| `initialize(waypoints, n, start_vel, start_acc, end_vel, end_acc)` | Clamped SE(3) 스플라인 (로컬 프레임 경계 조건) |
| `compute(time)` | 시간 t에서 SE(3) pose, Motion velocity/acceleration 반환 (RT-safe) |
| `duration()` | 전체 궤적 지속 시간 |

#### 궤적 비교

| | QuinticBlend | QuinticSpline | TaskSpaceBlend | TaskSpaceSpline |
|---|---|---|---|---|
| **공간** | 관절 | 관절 | SE(3) 태스크 | SE(3) 태스크 |
| **연속성** | C2 | C4 | C2 | C4 |
| **초기화 비용** | O(N) | O(N^3) 선형 시스템 | O(N) | O(N^3) 선형 시스템 |
| **경계 조건** | Rest-to-rest only | Natural / Clamped | Rest-to-rest only | Natural / Clamped |
| **Via-point 속도** | 인접 평균 | 글로벌 최적 | 인접 평균 (프레임 변환) | 글로벌 최적 (프레임 변환) |
| **RT-safe compute** | O(DOF) | O(DOF) | O(1) (6-DOF 고정) | O(1) (6-DOF 고정) |
| **최대 웨이포인트** | 32 | 32 | 32 | 32 |

---

## 컨트롤러 등록

`rtc_controllers`의 4개 컨트롤러(`PController`, `JointPDController`, `ClikController`, `OperationalSpaceController`)는 **라이브러리 심볼로만 제공**되며 `ControllerRegistry`에 **자동 등록되지 않습니다**. 런타임에 선택 가능한 컨트롤러 집합은 각 로봇의 `<robot>_bringup` 패키지가 결정합니다 (예: `ur5e_bringup`은 `DemoJointController`, `DemoTaskController`, `DemoWbcController` 3종을 등록).

필요 시 downstream 패키지에서 다음과 같이 직접 등록할 수 있습니다.

```cpp
#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controllers/indirect/p_controller.hpp"

RTC_REGISTER_CONTROLLER(
    p_controller, "indirect/", "<your_bringup_pkg>",
    std::make_unique<rtc::PController>(urdf))
```

> `src/controller_registration.cpp`의 `ForceBuiltinControllerRegistration()`은 이제 no-op이며, 기존 `main()`들과의 링크 호환성을 위해서만 남아 있습니다.

---

## 컨트롤러 비교

| | PController | JointPDController | ClikController | OperationalSpaceController |
|---|---|---|---|---|
| **제어 공간** | 관절 | 관절 | 태스크 (3/6-DOF) | 태스크 (6-DOF) |
| **출력** | Position | Torque | Position | Torque |
| **궤적** | 없음 | JointSpace 5차 | TaskSpace SE(3) 5차 | TaskSpace SE(3) 5차 |
| **동역학** | FK만 | FK + G + C + J | FK + J | FK + J + G |
| **영공간** | N/A | N/A | 관절 센터링 (3-DOF만) | N/A (전체 6-DOF 사용) |
| **E-STOP** | 현재 위치 유지 (hold) | PD 기반 safe_position 이동 | safe_position 위치 명령 | safe_position 위치 명령 |
| **역행렬** | N/A | N/A | LDLT (3x3/6x6) | PartialPivLU (6x6) |
| **명령 제한** | velocity (기본 2.0 rad/s) | torque (기본 150 Nm) / velocity (기본 2.0 rad/s) | velocity (기본 2.0 rad/s) | velocity (기본 2.0 rad/s) |
| **계산량** | 최소 | 중간 | 중간 | 높음 |
| **스레드 안전** | 없음 | try_lock + atomic | try_lock + atomic | try_lock + atomic |

### 런타임 게인 레이아웃 (`UpdateGainsFromMsg` / `GetCurrentGains`)

| 컨트롤러 | 게인 레이아웃 | 수 |
|---------|-------------|---|
| **PController** | `[kp x 6]` | 6 |
| **JointPDController** | `[kp x 6, kd x 6, gravity(0/1), coriolis(0/1), trajectory_speed]` | 15 |
| **ClikController** | `[kp x 6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1)]` | 10 |
| **OperationalSpaceController** | `[kp_pos x 3, kd_pos x 3, kp_rot x 3, kd_rot x 3, damping, gravity(0/1), traj_speed, traj_ang_speed]` | 16 |

### RT 안전 보장 (전체 공통)

모든 컨트롤러의 RT 경로 메서드 (`Compute`, `SetDeviceTarget`, `InitializeHoldPosition`)는:
- `noexcept` 보장
- 생성자에서 모든 Pinocchio/Eigen 버퍼 사전 할당 (RT 경로에서 동적 할당 없음)
- `SetDeviceTarget()`은 `std::lock_guard` 사용, `Compute()`/`InitializeHoldPosition()`은 `std::try_to_lock` (RT 스레드 차단 불가)
- `new_target_` 플래그는 `memory_order_acquire/release` 원자적 동기화
- Eigen: `noalias()` 사용, 고정 크기 행렬(3x3, 6x6) 스택 할당
- **Gains SeqLock 스냅샷:** 모든 컨트롤러의 `gains_` 필드를 `rtc::SeqLock<Gains> gains_lock_`로 교체. RT 경로(`Compute()`)는 진입 시 `const auto gains = gains_lock_.Load()`로 전체 구조체를 단일 스냅샷으로 읽어, `UpdateGainsFromMsg()`가 aux 스레드에서 동시 실행되어도 한 틱 내 모든 필드(bool/배열/스칼라) 일관성 보장. Aux 스레드 writer(`LoadConfig`, `UpdateGainsFromMsg`, `set_gains`)는 Load→mutate→Store 패턴으로 torn-write 방지. Phase 1의 bool 플래그 스냅샷은 SeqLock에 흡수됨.
- **trajectory_speed 검증:** `LoadConfig()` 및 `UpdateGainsFromMsg()`에서 `trajectory_speed`/`trajectory_angular_speed`에 `std::max(1e-6, val)` 적용하여 0 또는 음수 값으로 인한 무한 궤적 duration 방지

---

## 설정 파일 (`config/`)

각 컨트롤러에 대응하는 YAML 설정 파일이 있으며, `LoadConfig()` 메서드에서 파싱합니다. 모든 설정 파일은 컨트롤러명을 최상위 키로 사용합니다.

| 파일 | 설명 |
|------|------|
| `config/controllers/indirect/p_controller.yaml` | P 제어기: kp 게인, command_type, 토픽 매핑 |
| `config/controllers/indirect/clik_controller.yaml` | CLIK 제어기: kp, damping, null_kp, 영공간/6DOF 설정, 토픽 매핑 |
| `config/controllers/direct/joint_pd_controller.yaml` | JointPD 제어기: kp/kd 게인, 중력/코리올리 보상, 궤적 속도, 토픽 매핑 |
| `config/controllers/direct/operational_space_controller.yaml` | OSC 제어기: 위치/자세 PD 게인, damping, 중력 보상, 궤적 속도, 토픽 매핑 |

각 YAML 파일은 `topics` 섹션에서 디바이스별 ROS2 토픽 구독/발행 매핑도 정의합니다. 이 매핑은 `RTControllerInterface::LoadConfig()`에서 공통 파싱됩니다.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_controller_interface` | 추상 컨트롤러 인터페이스 (`RTControllerInterface`) + 레지스트리 |
| `rtc_base` | 공유 데이터 타입 (`ControllerState`, `ControllerOutput`, `DeviceState`), 상수 (`kDefaultMaxJointVelocity`, `kDefaultMaxJointTorque`), RT-safe 유틸리티 (`utils/clamp_commands.hpp::ClampSymmetric`, `utils/device_passthrough.hpp::PassthroughSecondaryDevices`) |
| `rtc_msgs` | RTC 프레임워크 커스텀 ROS2 메시지 |
| `eigen` | 선형 대수 연산 (Eigen3) |
| `pinocchio` | 기구학/동역학 (FK, Jacobian, Gravity, Coriolis, SE3/SO3, exp/log) |
| `yaml-cpp` | YAML 설정 파싱 |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_controllers
source install/setup.bash
```

정적 라이브러리(`librtc_controllers.a`)가 생성됩니다. `POSITION_INDEPENDENT_CODE ON` 속성이 설정되어 공유 라이브러리에서 링크할 수 있습니다.

---

## 의존성 그래프 내 위치

```
rtc_base + pinocchio + yaml-cpp + eigen
    |
rtc_controller_interface  -- 추상 인터페이스 + 레지스트리
    |
rtc_controllers  -- 4개 내장 컨트롤러 구현
    ^
    |-- rtc_controller_manager  (컨트롤러 인스턴스화 + Compute 호출)
    |-- ur5e_bringup            (launch에서 컨트롤러 선택)
```

---

## 변경 내역

### v5.19.0 (Phase 1b)

| 영역 | 변경 내용 |
|------|----------|
| **스레드 안전 (SeqLock)** | 4개 컨트롤러(`PController`, `JointPDController`, `ClikController`, `OperationalSpaceController`) 모두 `Gains gains_` → `rtc::SeqLock<Gains> gains_lock_` 전환. RT 경로는 `Compute()` 진입 시 단일 `Load()` 스냅샷 사용, aux 스레드 writer는 Load/mutate/Store 패턴. Phase 1의 bool 플래그 스냅샷을 SeqLock으로 대체 — 전체 구조체 단위 일관성 보장. |
| **API 유지** | `set_gains`/`get_gains`/`GetCurrentGains` 시그니처 동일, 내부 구현만 SeqLock 경유로 변경. `set_kp()` (PController)도 Load/modify/Store 적용. |

### v5.18.0

| 영역 | 변경 내용 |
|------|----------|
| **스레드 안전** | `Compute()`에서 bool 플래그 (control_6dof, enable_null_space, enable_gravity/coriolis) 틱 시작 시 로컬 스냅샷 -- `UpdateGainsFromMsg()` 동시 실행 시 분기 일관성 보장 (v5.19.0에서 SeqLock으로 대체) |
| **RT 안전** | `InitializeHoldPosition()`에서 `std::lock_guard` → `std::try_to_lock` 변경 -- RT 경로 blocking lock 제거 (JointPD, CLIK, OSC) |
| **입력 검증** | `trajectory_speed`, `trajectory_angular_speed`에 `std::max(1e-6, val)` 클램프 적용 -- 0/음수 값 입력 시 무한 궤적 duration 방지 |
| **테스트** | `test_core_controllers.cpp` 추가 -- PController(10), JointPD(8), CLIK(8), OSC(7) = 33개 단위 테스트 |

### v5.17.0

| 영역 | 변경 내용 |
|------|----------|
| **JointPDController** | 피드포워드 속도 (`ff_vel`) 항 추가 -- 궤적 추적 정확도 향상 |
| **DeviceOutput 확장** | `trajectory_positions`/`trajectory_velocities` 필드 활용 -- 궤적 레퍼런스와 컨트롤러 타겟 분리 |
| **ControllerOutput 확장** | `task_goal_positions`, `trajectory_task_positions`/`velocities` 필드 활용 (CLIK/OSC) |
| **DeviceNameConfig** | `safe_position` 기반 E-STOP 위치 설정 (YAML 로드), `OnDeviceConfigsSet()` 오버라이드에서 max_velocity 캐싱 |

### v0.1.1

| 영역 | 변경 내용 |
|------|----------|
| **p_controller.hpp** | include 순서 수정 -- project header를 최상단으로 이동 (Google C++ style) |
| **joint_space_trajectory.hpp** | `static_assert(N > 0)` 추가 -- 템플릿 파라미터 컴파일 타임 검증 |

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
