# ur5e_rt_controller

> **Note:** This package is part of the UR5e RT Controller workspace (v5.3.0). For full architecture details, installation instructions, and ROS 2 Jazzy compatibility, please refer to the [Root README](../README.md) and [Root CLAUDE.md](../CLAUDE.md).
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
├── src/
│   ├── custom_controller.cpp                ← 메인 500Hz 노드 (런타임 전환, v5.3.0)
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
CustomController (ROS2 노드)
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

Pinocchio RNEA를 활용한 모델 기반 PD + 동역학 보상.

```
command[i] = Kp * e[i] + Kd * ė[i] + g(q)[i] [+ C(q,v)·v[i]]
```

- 모든 Eigen 버퍼: 생성자에서 사전 할당 (500Hz 경로에서 힙 할당 없음)
- `enable_gravity_compensation`: 중력 보상 활성화
- `enable_coriolis_compensation`: 코리올리 보상 활성화

### `ClikController`

폐루프 역기구학(CLIK). 감쇠 야코비안 유사역행렬 + 영공간 관절 센터링.

**목표 규약** (`/target_joint_positions`의 6개 값):
```
[x, y, z, null_q3, null_q4, null_q5]
 ─────────── ───────────────────────
 TCP 위치(m)  영공간 참조 관절 3–5 (rad)
```

### `OperationalSpaceController`

전체 6-DOF 데카르트 PD 제어 (위치 + SO(3) 방향). Pinocchio `log3()` 사용.

**목표 규약** (`/target_joint_positions`의 6개 값):
```
[x, y, z, roll, pitch, yaw]
 ─────────── ─────────────
 TCP 위치(m)  ZYX 오일러 각 (rad)
```

---

## ROS2 인터페이스

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR 드라이버 또는 시뮬레이터에서 6-DOF 위치/속도 |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 6개 목표값 (컨트롤러별 해석 다름) |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | UDP 수신기에서 11개 손 모터값 |
| `/custom_controller/controller_type` | `std_msgs/Int32` | 런타임 컨트롤러 전환 (0=P, 1=PD, 2=Pinocchio, 3=CLIK, 4=OSC) |
| `/custom_controller/controller_gains` | `std_msgs/Float64MultiArray` | 컨트롤러별 게인 동적 업데이트 |

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
PID=$(pgrep -f custom_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
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

v5.3.0에서 추가된 헤더-전용 5차 다항식 궤적 생성 라이브러리입니다. CLIK/OSC 제어기에서 새 목표 수신 시 부드러운 이동을 자동으로 생성합니다.

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

속도 상한 계산: `duration = max(0.5, distance / 0.2m)` — 평균 이동 속도 최대 20cm/s.
힙 할당 없음 (`QuinticPolynomial × 6`, 고정 크기 배열).

### `JointSpaceTrajectory<N>` (`trajectory/joint_space_trajectory.hpp`)

N-DOF 관절공간 5차 스플라인 (템플릿). N개 관절을 독립적으로 보간합니다.

```cpp
JointSpaceTrajectory<6> traj;
JointSpaceTrajectory<6>::State start{q0, v0, {}};
JointSpaceTrajectory<6>::State goal {qf, {}, {}};
traj.initialize(start, goal, duration);
auto s = traj.compute(t);  // State{positions, velocities, accelerations}
```

---

## 커스텀 컨트롤러 추가

`RTControllerInterface`를 상속하여 `Compute()`, `SetRobotTarget()`, `SetHandTarget()`, `Name()`을 구현합니다. **모든 메서드는 반드시 `noexcept`여야 합니다** (500Hz 루프에서 예외 발생 시 프로세스 종료).

```cpp
class MyController : public ur5e_rt_controller::RTControllerInterface {
 public:
  [[nodiscard]] ControllerOutput Compute(
      const ControllerState& state) noexcept override;
  void SetRobotTarget(std::span<const double> target) noexcept override;
  void SetHandTarget(std::span<const double> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override {
    return "MyController";
  }
};
```

그 후 `custom_controller.cpp`에서 컨트롤러를 교체합니다:

```cpp
// 생성자 내 (약 40번 줄):
controller_(std::make_unique<MyController>())
```

헤더-전용 컨트롤러는 `CMakeLists.txt` 수정이 불필요합니다.

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
