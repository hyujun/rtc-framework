# Pinocchio Controller 구현 계획

## 목표

Pinocchio 라이브러리를 활용해 **모델 기반 제어기(Model-Based Controller)**를 RTControllerInterface에서 상속해 구현한다.
기존 PDController와 동일한 인터페이스를 유지하므로, `custom_controller.cpp`에서 한 줄만 바꿔 교체 가능하다.

---

## 제어 방식: 중력 보상 PD 제어 (Gravity-Compensated PD Control)

```
command[i] = Kp * e[i] + Kd * ė[i] + g(q)[i]
```

- `e[i]` = target - current (위치 오차)
- `ė[i]` = 오차 미분 (속도 기반)
- `g(q)` = Pinocchio로 계산한 **중력 보상 토크**

Pinocchio가 추가로 제공하는 기능:

| 기능 | Pinocchio API | 활용 |
|---|---|---|
| Forward Kinematics | `pinocchio::forwardKinematics()` | TCP 위치 계산 |
| Jacobian | `pinocchio::computeJointJacobian()` | 태스크 공간 정보 제공 |
| 중력 보상 | `pinocchio::computeGeneralizedGravity()` | 중력 토크 벡터 g(q) |
| 코리올리 | `pinocchio::computeCoriolisMatrix()` | 선택적 보상 |
| 역동역학 | `pinocchio::rnea()` | Computed Torque Control |

---

## 구현 범위 (3단계)

### Step 1 — 의존성 추가
**수정 파일:** `CMakeLists.txt`, `package.xml`

- `find_package(pinocchio REQUIRED)` 추가
- `custom_controller` 타겟에 `pinocchio::pinocchio` 링크
- package.xml에 `<depend>pinocchio</depend>` 추가

---

### Step 2 — PinocchioController 헤더 작성
**신규 파일:** `include/ur5e_rt_controller/controllers/pinocchio_controller.hpp`

#### 클래스 구조

```cpp
namespace ur5e_rt_controller {

class PinocchioController final : public RTControllerInterface {
public:
  struct Gains {
    double kp{5.0};
    double kd{0.5};
    bool enable_gravity_compensation{true};
    bool enable_coriolis_compensation{false};
  };

  // URDF 경로로 Pinocchio 모델 로드
  explicit PinocchioController(std::string_view urdf_path, Gains gains = {});

  // RTControllerInterface 구현 (모두 noexcept)
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;
  void SetRobotTarget(std::span<const double> target) noexcept override;
  void SetHandTarget(std::span<const double> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override;
  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // Pinocchio 전용 조회 (비-RT 경로)
  [[nodiscard]] std::array<double, 3> GetTcpPosition() const noexcept;
  [[nodiscard]] std::array<double, 6> GetGravityTorques() const noexcept;

private:
  pinocchio::Model model_;
  pinocchio::Data data_;

  Gains gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<double, kNumHandJoints>  hand_target_{};
  std::array<double, kNumRobotJoints> prev_error_{};
  std::array<double, kNumRobotJoints> gravity_torques_{};  // cache

  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  // 안전 위치 (E-STOP 시 이동)
  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0
  };
  static constexpr double kMaxJointVelocity{2.0};

  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;
  [[nodiscard]] std::array<double, kNumRobotJoints> ClampCommands(
    std::array<double, kNumRobotJoints> cmds) noexcept;
  void UpdateKinematics(const RobotState& robot) noexcept;
};

} // namespace ur5e_rt_controller
```

#### 핵심 구현 로직 (`Compute()`)

```cpp
ControllerOutput PinocchioController::Compute(const ControllerState& state) noexcept {
  if (estopped_) return ComputeEstop(state);

  // 1. Pinocchio 운동학/동역학 갱신
  UpdateKinematics(state.robot);

  // 2. 중력 보상 계산 (g(q))
  if (gains_.enable_gravity_compensation) {
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model_, data_, q_);
    for (int i = 0; i < kNumRobotJoints; ++i)
      gravity_torques_[i] = g[i];
  }

  // 3. PD 제어 + 중력 보상
  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    double e   = robot_target_[i] - state.robot.positions[i];
    double de  = (e - prev_error_[i]) / state.dt;
    output.robot_commands[i] = gains_.kp * e
                              + gains_.kd * de
                              + gravity_torques_[i];
    prev_error_[i] = e;
  }

  output.robot_commands = ClampCommands(output.robot_commands);
  return output;
}
```

---

### Step 3 — 사용 예시 주석 추가
**수정 파일:** `src/custom_controller.cpp`

```cpp
// Pinocchio 기반 제어기로 교체 예시:
// #include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"
// controller_(std::make_unique<urtc::PinocchioController>(
//   "/path/to/ur5e.urdf",
//   urtc::PinocchioController::Gains{.kp=5.0, .kd=0.5, .enable_gravity_compensation=true}
// ))
```

---

## 파일 변경 요약

| 파일 | 변경 종류 | 내용 |
|---|---|---|
| `CMakeLists.txt` | 수정 | pinocchio 의존성 추가 |
| `package.xml` | 수정 | pinocchio depend 추가 |
| `include/.../controllers/pinocchio_controller.hpp` | **신규** | 제어기 전체 구현 |
| `src/custom_controller.cpp` | 수정 | 사용 예시 주석 추가 |

---

## RTControllerInterface 준수 사항

- 모든 RT 경로 메서드: `noexcept` 필수
- Pinocchio 계산: Eigen 동적 할당 없이 `data_` 재사용 (RT-safe)
- E-STOP: `std::atomic<bool>` 사용 (mutex 없이 크로스-스레드 안전)
- 출력 클램핑: `±kMaxJointVelocity (2.0 rad/s)` 적용
- 모델 로드: 생성자에서만 수행 (RT 경로 외부)
