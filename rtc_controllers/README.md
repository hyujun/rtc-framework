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

---

## 컨트롤러 등록

`src/controller_registration.cpp`에서 `RTC_REGISTER_CONTROLLER` 매크로로 4개 컨트롤러를 정적 초기화 시점에 자동 등록합니다.

```cpp
RTC_REGISTER_CONTROLLER(
    p_controller, "indirect/", "rtc_controllers",
    std::make_unique<rtc::PController>(urdf))

RTC_REGISTER_CONTROLLER(
    joint_pd_controller, "direct/", "rtc_controllers",
    std::make_unique<rtc::JointPDController>(urdf))

RTC_REGISTER_CONTROLLER(
    clik_controller, "indirect/", "rtc_controllers",
    std::make_unique<rtc::ClikController>(urdf, rtc::ClikController::Gains{}))

RTC_REGISTER_CONTROLLER(
    operational_space_controller, "direct/", "rtc_controllers",
    std::make_unique<rtc::OperationalSpaceController>(
        urdf, rtc::OperationalSpaceController::Gains{}))
```

> 정적 라이브러리에서 링커 스트립을 방지하려면 `rtc::ForceBuiltinControllerRegistration()`을 `main()`에서 호출해야 합니다.

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
- `SetDeviceTarget()`은 `std::lock_guard` 사용, `Compute()`는 `std::try_to_lock` (RT 스레드 차단 불가)
- `new_target_` 플래그는 `memory_order_acquire/release` 원자적 동기화
- Eigen: `noalias()` 사용, 고정 크기 행렬(3x3, 6x6) 스택 할당

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
| `rtc_base` | 공유 데이터 타입 (`ControllerState`, `ControllerOutput`, `DeviceState`), 상수 (`kDefaultMaxJointVelocity`, `kDefaultMaxJointTorque`) |
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
