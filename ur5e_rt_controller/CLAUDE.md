# CLAUDE.md — ur5e_rt_controller

> **Note:** This package is part of the UR5e RT Controller workspace (v5.2.2). Please refer to the [Root CLAUDE.md](../CLAUDE.md) for full workspace context, building instructions, and architecture overview.
500 Hz 실시간 위치 제어기 패키지. 스택의 핵심 실행 파일 `custom_controller` 을 빌드한다.

---

## 파일 구조 및 역할

```
src/
└── custom_controller.cpp        ← 유일한 실행 파일 (노드+main 전체)

include/ur5e_rt_controller/
├── rt_controller_interface.hpp  ← Strategy 추상 기반 클래스
├── controller_timing_profiler.hpp ← Compute() 시간 측정 (lock-free 히스토그램)
└── controllers/
    ├── pd_controller.hpp        ← 기본 PD + E-STOP (활성 기본값)
    ├── p_controller.hpp         ← 단순 P 제어기
    ├── pinocchio_controller.hpp ← PD + 중력/코리올리 보상 (Pinocchio)
    ├── clik_controller.hpp      ← 3-DOF Cartesian CLIK
    └── operational_space_controller.hpp ← 6-DOF Cartesian PD (OSC)

config/
├── ur5e_rt_controller.yaml      ← kp/kd/E-STOP/logging 파라미터
└── cyclone_dds.xml              ← DDS 스레드 Core 0-1 제한

scripts/
└── setup_irq_affinity.sh        ← NIC IRQ → Core 0-1 고정

launch/
├── ur_control.launch.py         ← 전체 시스템 (실제 로봇)
└── hand_udp.launch.py           ← 핸드 UDP 노드 전용
```

---

## custom_controller.cpp — 아키텍처

**4 SingleThreadedExecutor + 4 std::thread** 구조.

```
t_rt     (Core 2, FIFO 90) → rt_executor   → ControlLoop() 500Hz, CheckTimeouts() 50Hz
t_sensor (Core 3, FIFO 70) → sensor_executor → JointStateCallback, TargetCallback, HandStateCallback
t_log    (Core 4, nice -5) → log_executor   → drain_timer_ 100Hz (CSV 기록)
t_aux    (Core 5, OTHER 0) → aux_executor   → estop_pub_ 이벤트 드리븐
```

스레드 간 공유 데이터:
| 데이터 | 보호 뮤텍스 | 접근 스레드 |
|--------|------------|------------|
| `current_pos_/vel_` | `state_mutex_` | sensor(W), RT(R) |
| `target_snapshot_` | `target_mutex_` | sensor(W), RT(R) |
| 핸드 타임스탬프 | `hand_mutex_` | sensor(W), RT(R) |
| E-STOP 플래그 | `atomic<bool>` | RT(R), timeout(W) |

**mlockall(MCL_CURRENT | MCL_FUTURE)** — `rclcpp::init()` 이전에 호출.

---

## ControlLoop() — 500Hz RT 경로 요구사항

ControlLoop()는 **절대 블로킹하면 안 된다**:
- ❌ `std::mutex::lock()` (timeout 가능성) → `try_lock()` 사용
- ❌ `std::cout` / `RCLCPP_INFO` → `log_buffer_.Push()` 경유
- ❌ `new` / `delete` / STL 컨테이너 resize
- ❌ 파일 I/O — DataLogger는 log 스레드에서만 drain
- ✅ `atomic<bool>` 읽기 (memory_order_acquire)
- ✅ `log_buffer_.Push()` — lock-free, drop on full

---

## RTControllerInterface — 제어기 추가 방법

```cpp
// 1. include/ur5e_rt_controller/controllers/my_ctrl.hpp 생성
class MyController final : public RTControllerInterface {
 public:
  [[nodiscard]] ControllerOutput Compute(const ControllerState& s) noexcept override;
  void SetRobotTarget(std::span<const double, 6> t)  noexcept override;
  void SetHandTarget(std::span<const double, 11> t)  noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override { return "MyController"; }
};

// 2. custom_controller.cpp 생성자의 controller_ 초기화만 교체
// Pinocchio 기반이면 set_gains() 호출 제거 필요 (생성자 인자로 대체)
```

**모든 가상 메서드는 `noexcept` 필수** — 500Hz 콜백에서 예외 발생 시 프로세스 종료.

---

## 제어기별 타겟 입력 규칙

| 제어기 | `/target_joint_positions` 해석 |
|--------|-------------------------------|
| `PDController` / `PinocchioController` | 관절 각도 6개 (rad) |
| `ClikController` | `[x, y, z, null_q3, null_q4, null_q5]` (TCP 위치 m + null-space rad) |
| `OperationalSpaceController` | `[x, y, z, roll, pitch, yaw]` (m + rad ZYX) |

---

## ControllerTimingProfiler

`MeasuredCompute(controller, state)` → Compute() 래핑 + `steady_clock` 측정.
히스토그램: 0–2000µs, 100µs 버킷 21개. `Summary()` → 1000 반복마다 로그 권장.
`LogEntry.compute_time_us` → `SpscLogBuffer` → CSV에 기록됨.

---

## E-STOP 시스템

`CheckTimeouts()` (50Hz, RT 스레드):
- `/joint_states` 수신 공백 > `robot_timeout_ms` → `controller_->TriggerEstop()`
- `/hand/joint_states` 공백 > `hand_timeout_ms` → `controller_->SetHandEstop(true)`
- `hand_timeout_ms: 0` → 핸드 E-STOP 비활성화 (MuJoCo 시뮬 환경에서 사용)

`PDController::TriggerEstop()`:
- `estopped_` atomic flag set
- Compute() 내에서 target → `kSafePosition [0,-1.57,1.57,-1.57,-1.57,0]` 로 전환
- 출력 속도 `kMaxJointVelocity = 2.0 rad/s`로 클램프

---

## Pinocchio 제어기 공통 RT 안전성

- URDF 로드: 생성자에서 1회만 (`pinocchio::urdf::buildModel`)
- Eigen 버퍼: 생성자에서 전부 사전할당, `noalias()` 사용
- 행렬 분해: 고정-크기 타입 (`LDLT<Matrix3d>`, `PartialPivLU<Matrix<double,6,6>>`) → 스택 할당
- Pinocchio 헤더: 반드시 `#pragma GCC diagnostic push/pop`으로 감싸야 경고 무시됨

---

## 파라미터 (config/ur5e_rt_controller.yaml)

```yaml
controller:
  control_rate: 500.0   # Hz
  kp: 5.0
  kd: 0.5
  enable_logging: true

estop:
  enable_estop: true
  robot_timeout_ms: 100.0
  hand_timeout_ms: 200.0   # 0 = 비활성화
  safe_position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
```

MuJoCo 시뮬 시 `mujoco_simulator.yaml`에서 오버라이드:
```yaml
custom_controller:
  ros__parameters:
    enable_estop: false
    robot_timeout_ms: 10000.0
    hand_timeout_ms: 10000.0
```

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_rt_base ur5e_rt_controller --symlink-install
```

Pinocchio 제어기 사용 시: `sudo apt install ros-humble-pinocchio`

---

## 이 패키지를 수정할 때 주의사항

1. **새 제어기 헤더 추가** → CMakeLists 변경 불필요 (헤더-전용).
   Pinocchio 사용 시 이미 `target_link_libraries(custom_controller pinocchio::pinocchio)` 연결됨.

2. **ControlLoop() 수정** → RT 안전성 확인 필수. 뮤텍스 lock 시도 전에 `try_lock()` 사용.

3. **타이머 주기 변경** → `control_rate` 파라미터로 결정. 코드에 직접 하드코딩하지 말 것.

4. **콜백 그룹 추가** → 반드시 `MutuallyExclusive`로 생성하고 올바른 executor에 등록.

5. **`set_gains()` 호출** — `PDController` 전용. Pinocchio 제어기로 교체 시 제거 필수.
