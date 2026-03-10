# ur5e_rt_controller

> **Note:** This package is part of the UR5e RT Controller workspace (v5.6.1). For full architecture details, installation instructions, and ROS 2 Jazzy compatibility, please refer to the [Root README](../README.md) and [Root CLAUDE.md](../CLAUDE.md).
UR5e 로봇 팔을 위한 **500Hz 실시간 위치 제어기** ROS2 패키지입니다. SCHED_FIFO 멀티스레드 아키텍처, 전략 패턴 기반 컨트롤러 교체, 런타임 컨트롤러 전환, 잠금-없는 로깅 인프라를 제공합니다.

## 개요

```
ur5e_rt_controller/
├── include/ur5e_rt_controller/
│   ├── rt_controller_interface.hpp          ← 추상 기반 클래스 (Strategy Pattern)
│   ├── controller_timing_profiler.hpp       ← 잠금-없는 Compute() 타이밍 프로파일러
│   ├── controllers/
│   │   ├── pd_controller.hpp                ← PD + E-STOP (기본값)
│   │   ├── p_controller.hpp                 ← 단순 P 제어기 (개발/테스트용)
│   │   ├── pinocchio_controller.hpp         ← 모델 기반 PD + 중력/코리올리 보상
│   │   ├── clik_controller.hpp              ← 폐루프 IK (데카르트 3-DOF)
│   │   └── operational_space_controller.hpp ← 전체 6-DOF 데카르트 PD + SO(3)
│   └── trajectory/                          ← 5차 궤적 생성 (v5.3.0+)
│       ├── trajectory_utils.hpp             ← QuinticPolynomial 스칼라 유틸리티
│       ├── task_space_trajectory.hpp        ← SE(3) 스플라인 (CLIK/OSC 사용)
│       └── joint_space_trajectory.hpp       ← 관절공간 N-DOF 스플라인
├── include/ur5e_rt_controller/
│   └── rt_controller_node.hpp               ← RtControllerNode 클래스 선언 (v5.5.0)
├── src/
│   ├── rt_controller_node.cpp               ← Controller Registry + 노드 구현 (v5.5.0)
│   ├── rt_controller_main.cpp               ← main() — executor/RT 스레드 (v5.5.0)
│   └── pd_controller.cpp                    ← PD 제어기 소스
├── config/
│   ├── ur5e_rt_controller.yaml              ← 제어기 파라미터
│   └── cyclone_dds.xml                      ← CycloneDDS 스레드 Core 0-1 제한
├── scripts/
│   └── setup_irq_affinity.sh                ← NIC IRQ → Core 0-1 고정 스크립트
└── launch/
    └── ur_control.launch.py                  ← 전체 시스템 (use_cpu_affinity 포함)
```

**의존성:**
- `ur5e_rt_base` — 공유 타입, 스레드 유틸리티, 로깅 인프라
- `rclcpp`, `std_msgs`, `sensor_msgs`, `realtime_tools`
- `pinocchio` (Pinocchio 기반 컨트롤러 사용 시)

---

## 아키텍처

### 전략 패턴 + 멀티스레드 실행기

```
RtControllerNode (ROS2 노드)
    │
    ├── rt_executor (Core 2, FIFO/90)     ← ControlLoop() 500Hz, CheckTimeouts() 50Hz
    ├── sensor_executor (Core 3, FIFO/70) ← /joint_states, /target_joint_positions, /hand/joint_states [전용]
    ├── log_executor (Core 4, OTHER/nice-5)← DataLogger CSV 기록 (SpscLogBuffer 드레인)
    └── aux_executor (Core 5, OTHER/0)    ← /system/estop_status 퍼블리시

HandUdpReceiver (별도 jthread)
    └── udp_recv (Core 5, FIFO/65)       ← UDP 패킷 수신 [sensor_io와 분리, v5.1.0]

    controller_ (RTControllerInterface)
        └── [교체 가능] PDController / PinocchioController / ClikController / OSController
```

`mlockall(MCL_CURRENT | MCL_FUTURE)` — 시작 시 페이지 폴트 방지

### 스레드 간 동기화

| 뮤텍스 | 보호 대상 | 사용 스레드 |
|--------|-----------|------------|
| `state_mutex_` | `/joint_states` 최신값 | sensor ↔ RT |
| `target_mutex_` | 목표 관절 위치 | sensor ↔ RT |
| `hand_mutex_` | 손 데이터 타임스탬프 | sensor ↔ RT |

---

## 컨트롤러 구현

### `PDController` (기본값)

관절 공간 PD 제어기. E-STOP 시 안전 위치로 이동합니다.

```
command[i] = Kp * e[i] + Kd * ė[i]
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `5.0` | 비례 게인 |
| `kd` | `0.5` | 미분 게인 |

- E-STOP 안전 위치: `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad
- 최대 관절 속도: `2.0 rad/s` (`kMaxJointVelocity`)

### `PController`

단순 비례 제어기 (E-STOP 없음, 개발/테스트용).

### `PinocchioController`

Pinocchio RNEA를 활용한 모델 기반 PD + 동역학 보상. **5차 관절공간 궤적 추종** (v5.6.1+).

```
command[i] = ff_vel[i] + Kp * e[i] + Kd * ė[i] + g(q)[i] [+ C(q,v)·v[i]]
```

여기서 `ff_vel` 및 목표 위치는 `JointSpaceTrajectory<6>`에서 실시간으로 계산됩니다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `[5.0×6]` | 비례 게인 |
| `kd` | `[0.5×6]` | 미분 게인 |
| `enable_gravity_compensation` | `true` | 중력 보상 활성화 |
| `enable_coriolis_compensation` | `false` | 코리올리 보상 활성화 |
| `trajectory_speed` | `1.0` | 궤적 이동 속도 상한 [rad/s] — 지속시간 = max_dist / speed |

- `UpdateGainsFromMsg` 레이아웃: `[kp×6, kd×6, gravity(0/1), coriolis(0/1), trajectory_speed]` (15개)
- 모든 Eigen 버퍼: 생성자에서 사전 할당 (500Hz 경로에서 힙 할당 없음)

### `ClikController`

폐루프 역기구학(CLIK). 감쇠 야코비안 유사역행렬 + 영공간 관절 센터링. **SE(3) 5차 궤적 추종** (v5.3.0+).

**목표 규약** (`/target_joint_positions`의 6개 값):
```
[x, y, z, null_q3, null_q4, null_q5]
 ─────────── ───────────────────────
 TCP 위치(m)  영공간 참조 관절 3–5 (rad)
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp` | `[1.0×3]` | 위치 비례 게인 |
| `damping` | `0.01` | 감쇠 유사역행렬 λ |
| `null_kp` | `0.5` | 영공간 관절 센터링 게인 |
| `enable_null_space` | `false` | 영공간 태스크 활성화 |
| `trajectory_speed` | `0.1` | TCP 이동 속도 상한 [m/s] |

- `UpdateGainsFromMsg` 레이아웃: `[kp×3, damping, null_kp, enable_null_space(0/1)]` (6개)

### `OperationalSpaceController`

전체 6-DOF 데카르트 PD 제어 (위치 + SO(3) 방향). Pinocchio `log3()` 사용. **SE(3) 5차 궤적 추종** (v5.6.1+).

**목표 규약** (`/target_joint_positions`의 6개 값):
```
[x, y, z, roll, pitch, yaw]
 ─────────── ─────────────
 TCP 위치(m)  ZYX 오일러 각 (rad)
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `kp_pos` | `[1.0×3]` | 위치 비례 게인 |
| `kd_pos` | `[0.1×3]` | 위치 미분 게인 |
| `kp_rot` | `[0.5×3]` | 회전 비례 게인 |
| `kd_rot` | `[0.05×3]` | 회전 미분 게인 |
| `damping` | `0.01` | 감쇠 유사역행렬 λ |
| `enable_gravity_compensation` | `false` | 중력 보상 활성화 |
| `trajectory_speed` | `0.1` | TCP 병진 이동 속도 상한 [m/s] |
| `trajectory_angular_speed` | `0.5` | TCP 회전 속도 상한 [rad/s] |

- `UpdateGainsFromMsg` 레이아웃: `[kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, gravity(0/1), traj_speed, traj_ang_speed]` (16개)

---

## ROS2 인터페이스

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR 드라이버 또는 시뮬레이터에서 6-DOF 위치/속도 |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 6개 목표값 (컨트롤러별 해석 다름) |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | UDP 수신기에서 11개 손 모터값 |
| `/rt_controller/controller_type` | `std_msgs/Int32` | 런타임 컨트롤러 전환 (0=P, 1=PD, 2=Pinocchio, 3=CLIK, 4=OSC) |
| `/rt_controller/controller_gains` | `std_msgs/Float64MultiArray` | 컨트롤러별 게인 동적 업데이트 |

### 퍼블리시 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 6개 로봇 위치 명령 (rad) |
| `/system/estop_status` | `std_msgs/Bool` | `true` = E-STOP 활성 |

---

## 설정

### `config/ur5e_rt_controller.yaml`

```yaml
controller:
  control_rate: 500.0        # Hz
  kp: 5.0                    # PD 비례 게인
  kd: 0.5                    # PD 미분 게인
  enable_logging: true
  log_path: "/tmp/ur5e_control_log.csv"

joint_limits:
  max_velocity: 2.0          # rad/s
  max_acceleration: 5.0      # rad/s²

estop:
  enable_estop: true
  robot_timeout_ms: 100.0    # /joint_states 갭이 이 값 초과 시 E-STOP
  hand_timeout_ms: 200.0     # /hand/joint_states 갭이 이 값 초과 시 E-STOP (0 = 비활성)
  safe_position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
```

---

## 실행

### 실제 로봇 (UR 드라이버 사용)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10
```

### 가상 하드웨어 (로봇 불필요)

```bash
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_ip` | `192.168.1.10` | UR 로봇 IP |
| `use_fake_hardware` | `false` | 가상 하드웨어 모드 |
| `use_cpu_affinity` | `true` | UR 드라이버 Core 0-1 taskset 자동 적용 (3초 후) |

런치 파일이 자동 설정하는 환경변수:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` → `config/cyclone_dds.xml` (DDS recv/send 스레드 Core 0-1 제한)

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_rt_base ur5e_rt_controller --symlink-install
source install/setup.bash
```

---

## 모니터링

```bash
# 제어 주기 확인 (약 500Hz)
ros2 topic hz /forward_position_controller/commands

# E-STOP 상태 확인
ros2 topic echo /system/estop_status

# 컨트롤러 목록 확인
ros2 control list_controllers -v

# RT 스레드 확인
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

### CSV 로그 분석

```python
import pandas as pd
df = pd.read_csv('/tmp/ur5e_control_log.csv')
print(df['compute_time_us'].describe())
print(f'P95: {df["compute_time_us"].quantile(0.95):.1f} us')
print(f'P99: {df["compute_time_us"].quantile(0.99):.1f} us')
print(f'Over 2ms: {(df["compute_time_us"] > 2000).mean()*100:.2f}%')
```

---

## 목표 위치 수동 퍼블리시

```bash
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
  "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

---

## 궤적 생성 서브시스템 (`trajectory/`)

v5.3.0에서 추가된 헤더-전용 5차 다항식 궤적 생성 라이브러리입니다. Pinocchio/CLIK/OSC 제어기에서 새 목표 수신 시 부드러운 이동을 자동으로 생성합니다.

### `QuinticPolynomial` (`trajectory/trajectory_utils.hpp`)

스칼라 5차 다항식 — 위치·속도·가속도 경계 조건 6개를 모두 만족하는 계수를 계산합니다.

```cpp
QuinticPolynomial poly;
poly.compute_coefficients(p0, v0, a0, pf, vf, af, T);  // (시작, 목표, 지속시간)
auto s = poly.compute(t);  // TrajectoryState{pos, vel, acc} — t를 [0,T]로 클램프
```

### `TaskSpaceTrajectory` (`trajectory/task_space_trajectory.hpp`)

SE(3) 공간에서의 5차 스플라인 궤적. Pinocchio `log6`/`exp6`를 사용하여 두 SE(3) 포즈 사이를 부드럽게 보간합니다.
**CLIK 및 OSC 제어기**에서 `SetRobotTarget()` 호출 시 자동으로 생성됩니다.

```cpp
// (CLIK/OSC 내부 사용 예)
traj_.initialize(start_pose, pinocchio::Motion::Zero(),
                 goal_pose,  pinocchio::Motion::Zero(), duration);
auto state = traj_.compute(trajectory_time_);  // State{pose, velocity, acceleration}
```

- **CLIK**: `duration = max(0.01, trans_dist / trajectory_speed)`
- **OSC** (v5.6.1+): `duration = max(0.01, max(trans_dist / traj_speed, ang_dist / traj_ang_speed))`

힙 할당 없음 (`QuinticPolynomial × 6`, 고정 크기 배열).

### `JointSpaceTrajectory<N>` (`trajectory/joint_space_trajectory.hpp`)

N-DOF 관절공간 5차 스플라인 (템플릿). N개 관절을 독립적으로 보간합니다.
**PinocchioController** (v5.6.1+)에서 사용됩니다.

```cpp
JointSpaceTrajectory<6> traj;
JointSpaceTrajectory<6>::State start{q0, v0, {}};
JointSpaceTrajectory<6>::State goal {qf, {}, {}};
traj.initialize(start, goal, duration);
auto s = traj.compute(t);  // State{positions, velocities, accelerations}
```

- **PinocchioController**: `duration = max(0.01, max_joint_dist / trajectory_speed)`

---

## 커스텀 컨트롤러 추가

v5.4.0부터 **Controller Registry** 패턴을 사용합니다. 새 컨트롤러를 추가할 때 수정해야 할 파일이 최소화되었습니다.

### 4단계 요약

**1. 헤더 작성** (`include/ur5e_rt_controller/controllers/my_controller.hpp`)

```cpp
#pragma once
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include <yaml-cpp/yaml.h>

namespace ur5e_rt_controller {
class MyController final : public RTControllerInterface {
public:
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;
  void SetRobotTarget(std::span<const double, kNumRobotJoints> target) noexcept override;
  void SetHandTarget(std::span<const double, kNumHandJoints> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override { return "MyController"; }

  void TriggerEstop() noexcept override;
  void ClearEstop()   noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  void LoadConfig(const YAML::Node & cfg) override;           // YAML 파싱 (noexcept 아님)
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
};
} // namespace ur5e_rt_controller
```

**2. 구현 작성** (`src/controllers/my_controller.cpp`) — `Compute()`, `LoadConfig()`, `UpdateGainsFromMsg()` 등 구현

**3. YAML 설정 추가** (`config/controllers/my_controller.yaml`)

```yaml
my_controller:
  kp: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
```

**4. Registry에 등록** (`src/rt_controller_node.cpp` → `MakeControllerEntries()`)

```cpp
{"my_controller", [](const std::string &) {
  return std::make_unique<urtc::MyController>();
}},
// ── Add new controllers here ──
```

> 자세한 전체 가이드는 [`docs/ADDING_CONTROLLER.md`](../docs/ADDING_CONTROLLER.md)를 참조하세요.

### RTControllerInterface 가상 메서드

| 메서드 | noexcept | 설명 |
|--------|----------|------|
| `Compute()` | ✓ | 500Hz RT 루프에서 호출 — 반드시 noexcept |
| `SetRobotTarget()` | ✓ | 목표 관절 위치 설정 |
| `SetHandTarget()` | ✓ | 손 목표 설정 |
| `Name()` | ✓ | 컨트롤러 이름 반환 |
| `TriggerEstop()` / `ClearEstop()` | ✓ | E-STOP 제어 |
| `LoadConfig()` | ✗ | YAML 파싱 (예외 발생 가능, 호출 측에서 try/catch) |
| `UpdateGainsFromMsg()` | ✓ | 센서 스레드에서 호출 — 반드시 noexcept |

---

## 성능 특성

| 지표 | v4.2.0 이전 | v4.2.0+ | 개선 |
|------|------------|---------|------|
| 제어 지터 | ~500μs | <50μs | 10배 |
| E-STOP 응답 | ~100ms | <20ms | 5배 |
| CPU 사용률 | ~30% | ~25% | -17% |
| 컨텍스트 스위치 | ~5000/s | ~1000/s | -80% |

### 지터 검증 (cyclictest)

```bash
sudo apt install rt-tests
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo \
    --interval=2000 --loops=100000 --affinity=2 --histogram=200
# 목표: 최대 지터 < 50μs
```

---

## RT 권한 요구사항

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필요
```

자세한 RT 튜닝 가이드는 `docs/RT_OPTIMIZATION.md`를 참조하세요.

---

## 라이선스

MIT License
