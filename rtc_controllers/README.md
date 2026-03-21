# rtc_controllers

![version](https://img.shields.io/badge/version-v0.1.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **내장 제어 알고리즘 구현체** 패키지입니다. `RTControllerInterface`를 상속하는 4개의 컨트롤러와 5차 다항식 기반 궤적 생성기를 제공합니다.

**컨트롤러 분류:**

| 컨트롤러 | 공간 | 모드 | 카테고리 |
|---------|------|------|----------|
| PController | 관절 공간 | Position (indirect) | `indirect/` |
| JointPDController | 관절 공간 | Torque (direct) | `direct/` |
| ClikController | 태스크 공간 | Position (indirect) | `indirect/` |
| OperationalSpaceController | 태스크 공간 | Torque (direct) | `direct/` |

---

## 패키지 구조

```
rtc_controllers/
├── CMakeLists.txt
├── package.xml
├── include/rtc_controllers/
│   ├── indirect/
│   │   ├── p_controller.hpp              ← 관절 공간 P 제어기
│   │   └── clik_controller.hpp           ← 태스크 공간 CLIK 제어기
│   ├── direct/
│   │   ├── joint_pd_controller.hpp       ← 관절 공간 PD + 동역학 보상
│   │   └── operational_space_controller.hpp ← 태스크 공간 OSC
│   └── trajectory/
│       ├── trajectory_utils.hpp          ← 5차 다항식 궤적
│       ├── joint_space_trajectory.hpp    ← 관절 공간 궤적 생성기
│       └── task_space_trajectory.hpp     ← SE(3) 태스크 공간 궤적 생성기
├── src/
│   ├── controller_registration.cpp       ← 4개 컨트롤러 자동 등록
│   └── controllers/
│       ├── indirect/
│       │   ├── p_controller.cpp
│       │   └── clik_controller.cpp
│       └── direct/
│           ├── joint_pd_controller.cpp
│           └── operational_space_controller.cpp
└── config/controllers/
    ├── indirect/
    │   ├── p_controller.yaml             ← P 제어기 기본 설정
    │   └── clik_controller.yaml          ← CLIK 제어기 기본 설정
    └── direct/
        ├── joint_pd_controller.yaml      ← JointPD 제어기 기본 설정
        └── operational_space_controller.yaml ← OSC 제어기 기본 설정
```

---

## 컨트롤러 상세

### PController (관절 공간 P 제어)

가장 단순한 비례 관절 제어기입니다. 빠른 응답이 필요한 기본 위치 제어에 적합합니다.

**제어 법칙:**

```
command[i] = current_pos[i] + kp[i] × (target[i] - current[i]) × dt
```

**게인 구조체:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp` | `double[6]` | `[120, 120, 100, 80, 80, 80]` | 비례 게인 (관절별) |

**특징:**
- 최소 계산량 (동역학 미사용)
- 관절 속도 제한: ±2.0 rad/s
- Position 모드 출력 (incremental)
- FK 계산으로 TCP 위치 진단 제공

```yaml
# config/controllers/indirect/p_controller.yaml
p_controller:
  kp: [120.0, 120.0, 100.0, 80.0, 80.0, 80.0]
  command_type: "position"
```

---

### JointPDController (관절 공간 PD + 동역학 보상)

5차 궤적 생성과 선택적 중력/코리올리 보상을 포함하는 관절 공간 PD 토크 제어기입니다.

**제어 법칙:**

```
τ[i] = kp[i] × e[i] + kd[i] × ė[i]  [+ g(q)[i]]  [+ C(q,v)·v[i]]

여기서 e = trajectory_setpoint - current_position
      ė = (e - e_prev) / dt
```

**게인 구조체:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp` | `double[6]` | `[200, 200, 150, 120, 120, 120]` | 비례 게인 |
| `kd` | `double[6]` | `[30, 30, 25, 20, 20, 20]` | 미분 게인 |
| `enable_gravity_compensation` | `bool` | `false` | 중력 보상 활성화 |
| `enable_coriolis_compensation` | `bool` | `false` | 코리올리 보상 활성화 |
| `trajectory_speed` | `double` | `1.0` | 궤적 속도 (rad/s) |

**알고리즘 흐름:**

1. `SetRobotTarget()` → 새 목표 수신 (mutex 보호)
2. 5차 다항식 궤적 초기화 (현재 → 목표, duration = distance / speed)
3. Pinocchio로 순기구학, 중력 토크, 코리올리 행렬 계산
4. 궤적 설정값 대비 PD 오차 계산 + 피드포워드 보상
5. 토크 제한 (±150 Nm) 적용

**E-STOP:** 안전 위치 `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad로 이동

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

### ClikController (태스크 공간 CLIK)

Closed-Loop Inverse Kinematics — 감쇠 의사역행렬과 영공간 보조 태스크를 사용하는 태스크 공간 위치 제어기입니다.

**제어 법칙 (3-DOF 모드):**

```
pos_error  = p_des - FK(q)
J_pos      = J[0:3, :]
J_pos^#    = J_posᵀ (J_pos J_posᵀ + λ²I)⁻¹         ← 감쇠 의사역행렬
N          = I - J_pos^# J_pos                        ← 영공간 투영

dq = kp × J_pos^# × pos_error  +  null_kp × N × (q_null - q)
q_cmd = q + clamp(dq, ±v_max) × dt
```

**게인 구조체:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp` | `double[6]` | `[1.0, ..., 1.0]` | 태스크 공간 비례 게인 |
| `damping` | `double` | `0.01` | 의사역행렬 감쇠 계수 (λ²) |
| `null_kp` | `double` | `0.5` | 영공간 보조 태스크 게인 |
| `enable_null_space` | `bool` | `true` | 영공간 관절 센터링 활성화 |
| `trajectory_speed` | `double` | `0.1` | 태스크 공간 궤적 속도 (m/s) |
| `control_6dof` | `bool` | `false` | 6-DOF (위치+자세) 제어 활성화 |

**동작 모드:**

| 모드 | 입력 | 제어 |
|------|------|------|
| `control_6dof=false` | target[0:3]=xyz, target[3:6]=영공간 관절 | 3-DOF 위치 + 영공간 |
| `control_6dof=true` | target[0:3]=xyz, target[3:6]=rpy | 6-DOF 위치+자세 (영공간 없음) |

**핵심 기법:**
- LDLT 분해 — 수치 안정적 의사역행렬 계산
- SE(3) 궤적 보간 — log6 기반 거리 계산 + TaskSpaceTrajectory
- Pinocchio 자코비안 — `computeJointJacobians()` 활용

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

### OperationalSpaceController (태스크 공간 6-DOF OSC)

전체 6-DOF 태스크 공간 PD 제어 + 선택적 중력 보상을 포함하는 토크 제어기입니다.

**제어 법칙:**

```
pos_error  = p_des - FK(q)
rot_error  = log₃(R_des × R_FK(q)ᵀ)               ← SO(3) 로그

tcp_vel    = J × q̇

F[0:3] = kp_pos ⊙ pos_error  -  kd_pos ⊙ tcp_vel[0:3]
F[3:6] = kp_rot ⊙ rot_error  -  kd_rot ⊙ tcp_vel[3:6]

JJt    = J × Jᵀ + λ²I₆
J^#    = Jᵀ × JJt⁻¹                               ← 감쇠 의사역행렬

dq     = J^# × F  [+ g(q)]                         ← 중력 보상 (선택)
```

**게인 구조체:**

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `kp_pos` | `double[3]` | `[1.0, 1.0, 1.0]` | 위치 비례 게인 |
| `kd_pos` | `double[3]` | `[0.1, 0.1, 0.1]` | 위치 미분 게인 |
| `kp_rot` | `double[3]` | `[0.5, 0.5, 0.5]` | 자세 비례 게인 |
| `kd_rot` | `double[3]` | `[0.05, 0.05, 0.05]` | 자세 미분 게인 |
| `damping` | `double` | `0.01` | 의사역행렬 감쇠 계수 |
| `enable_gravity_compensation` | `bool` | `false` | 중력 보상 활성화 |
| `trajectory_speed` | `double` | `0.1` | 위치 궤적 속도 (m/s) |
| `trajectory_angular_speed` | `double` | `0.5` | 자세 궤적 속도 (rad/s) |

**핵심 기법:**
- PartialPivLU — 고정 크기 6×6 LU 분해 (동적 할당 없음)
- SO(3) 로그 맵 — 자세 오차를 축-각도(axis-angle)로 변환
- SE(3) 궤적 보간 — 위치/자세 독립 속도 기반 궤적

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

모든 컨트롤러는 5차(quintic) 다항식 기반 궤적 생성기를 사용합니다.

### QuinticPolynomial (`trajectory_utils.hpp`)

시작/끝 위치, 속도, 가속도 경계 조건을 만족하는 5차 다항식입니다.

```
q(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴ + c₅t⁵
```

### JointSpaceTrajectory (`joint_space_trajectory.hpp`)

N-DOF 관절 공간 궤적 — 각 관절에 독립 QuinticPolynomial을 적용합니다.

| 메서드 | 설명 |
|--------|------|
| `initialize(current, target, duration)` | 궤적 초기화 |
| `compute(t)` | 시간 t에서의 위치/속도/가속도 반환 |
| `duration()` | 궤적 지속 시간 |

### TaskSpaceTrajectory (`task_space_trajectory.hpp`)

SE(3) 태스크 공간 궤적 — 위치 3-DOF + 자세 3-DOF에 각각 QuinticPolynomial을 적용합니다.

| 메서드 | 설명 |
|--------|------|
| `initialize(current_pose, target_pose, duration)` | SE(3) 궤적 초기화 |
| `compute(t)` | 시간 t에서의 SE(3) pose, velocity, acceleration 반환 |
| `duration()` | 궤적 지속 시간 |

---

## 컨트롤러 등록

`src/controller_registration.cpp`에서 `RTC_REGISTER_CONTROLLER` 매크로로 4개 컨트롤러를 자동 등록합니다.

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
| **궤적** | 없음 | JointSpace | TaskSpace (SE3) | TaskSpace (SE3) |
| **동역학** | FK만 | FK + G + C | FK + J | FK + J + G |
| **영공간** | N/A | N/A | 관절 센터링 | N/A (전체 사용) |
| **E-STOP** | 미지원 | 안전 위치 이동 | 안전 위치 이동 | 안전 위치 이동 |
| **계산량** | 최소 | 중간 | 중간 | 높음 |

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_controller_interface` | 추상 컨트롤러 인터페이스 + 레지스트리 |
| `rtc_base` | 공유 데이터 타입, 상수 |
| `rtc_msgs` | 커스텀 ROS2 메시지 |
| `pinocchio` | 기구학/동역학 (FK, Jacobian, Gravity, Coriolis) |
| `yaml-cpp` | YAML 설정 파싱 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_controllers
source install/setup.bash
```

공유 라이브러리(`librtc_controllers.so`)가 생성됩니다.

---

## 의존성 그래프 내 위치

```
rtc_base + pinocchio + yaml-cpp
    ↓
rtc_controller_interface  ← 추상 인터페이스
    ↓
rtc_controllers  ← 4개 내장 컨트롤러 구현
    ↑
    ├── rtc_controller_manager  (컨트롤러 인스턴스화 + Compute 호출)
    └── ur5e_bringup            (launch에서 컨트롤러 선택)
```

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
