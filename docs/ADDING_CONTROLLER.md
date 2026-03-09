# 새 컨트롤러 추가 가이드

이 문서는 `ur5e_rt_controller` 패키지에 새 컨트롤러를 추가하는 방법을 단계별로 설명합니다.

---

## 개요

v5.4.0부터 **Controller Registry** 패턴을 사용합니다.
새 컨트롤러를 추가할 때 수정이 필요한 파일은 다음 세 가지뿐입니다.

| 단계 | 작업 | 파일 |
|:---:|---|---|
| 1 | 컨트롤러 헤더 작성 | `include/ur5e_rt_controller/controllers/my_controller.hpp` |
| 2 | 컨트롤러 구현 작성 | `src/controllers/my_controller.cpp` |
| 3 | YAML 설정 파일 작성 | `config/controllers/my_controller.yaml` |
| 4 | 레지스트리에 등록 | `src/rt_controller_node.cpp` (한 줄) |

> **CMakeLists 변경 필요 여부**
> - 헤더 전용(`.hpp`만): 불필요
> - `.cpp` 있음: `CMakeLists.txt`의 `target_sources`에 한 줄 추가

---

## 1단계 — 컨트롤러 헤더 작성

`include/ur5e_rt_controller/controllers/my_controller.hpp` 를 생성합니다.

```cpp
#pragma once
#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <array>
#include <span>
#include <string_view>

namespace ur5e_rt_controller
{

class MyController final : public RTControllerInterface
{
public:
  // ── Gains 구조체 ───────────────────────────────────────────────────────
  struct Gains
  {
    std::array<double, 6> kp{{5.0, 5.0, 5.0, 5.0, 5.0, 5.0}};
    // 필요한 파라미터를 추가하세요
  };

  MyController() noexcept;
  explicit MyController(Gains gains) noexcept;

  // ── RTControllerInterface 필수 구현 (모두 noexcept) ───────────────────
  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetRobotTarget(
    std::span<const double, kNumRobotJoints> target) noexcept override;

  void SetHandTarget(
    std::span<const double, kNumHandJoints> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "MyController";
  }

  // ── Controller Registry 훅 ────────────────────────────────────────────
  // gains layout: [kp×6]   ← 실제 레이아웃을 여기에 문서화하세요
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;

  // ── E-STOP (필요한 경우만 오버라이드) ────────────────────────────────
  // void TriggerEstop() noexcept override;
  // void ClearEstop()   noexcept override;
  // bool IsEstopped()   const noexcept override;

  // ── 게인 접근자 ───────────────────────────────────────────────────────
  void set_gains(Gains g) noexcept {gains_ = g;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

private:
  Gains  gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<double, kNumHandJoints>  hand_target_{};

  static constexpr double kMaxJointVelocity = 2.0;  // rad/s
};

}  // namespace ur5e_rt_controller
```

### 주의사항

| 규칙 | 이유 |
|---|---|
| `Compute()` 는 반드시 `noexcept` | 500 Hz RT 루프에서 예외 발생 시 프로세스 종료 |
| RT 경로에서 동적 메모리 할당 금지 | `std::vector`, `new` 사용 금지; 버퍼는 생성자에서 사전 할당 |
| Eigen 사용 시 `noalias()` 활용 | 임시 버퍼 할당 방지 |
| `std::atomic<bool>` 로 E-STOP 플래그 관리 | RT 스레드와 50 Hz 워치독 간 안전한 공유 |

---

## 2단계 — 컨트롤러 구현 작성

`src/controllers/my_controller.cpp` 를 생성합니다.

```cpp
#include "ur5e_rt_controller/controllers/my_controller.hpp"

#include <algorithm>  // std::copy, std::clamp

namespace ur5e_rt_controller
{

MyController::MyController() noexcept
: gains_(Gains{}) {}

MyController::MyController(Gains gains) noexcept
: gains_(gains) {}

ControllerOutput MyController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;
  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error = robot_target_[i] - state.robot.positions[i];
    output.robot_commands[i] = std::clamp(
      gains_.kp[i] * error, -kMaxJointVelocity, kMaxJointVelocity);
  }
  output.actual_target_positions = robot_target_;
  return output;
}

void MyController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), robot_target_.begin());
}

void MyController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), hand_target_.begin());
}

// ── Controller Registry 훅 ────────────────────────────────────────────────

void MyController::LoadConfig(const YAML::Node & cfg)
{
  // cfg는 YAML 파일의 "my_controller:" 하위 노드입니다.
  if (!cfg) {return;}
  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kp[i] = cfg["kp"][i].as<double>();
    }
  }
}

void MyController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp×6]
  if (gains.size() < 6) {return;}
  for (std::size_t i = 0; i < 6; ++i) {gains_.kp[i] = gains[i];}
}

}  // namespace ur5e_rt_controller
```

---

## 3단계 — YAML 설정 파일 작성

`config/controllers/my_controller.yaml` 을 생성합니다.

파일의 최상위 키는 반드시 컨트롤러의 `config_key` (4단계에서 등록할 이름)와 동일해야 합니다.

```yaml
my_controller:
  kp: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
  # 추가 파라미터...
```

YAML 파일이 없으면 경고 로그를 출력하고 기본값(`Gains{}`)이 사용됩니다.

---

## 4단계 — 레지스트리에 등록

`src/rt_controller_node.cpp` 의 `MakeControllerEntries()` 함수에 **한 줄**을 추가합니다.

```cpp
// rt_controller_node.cpp 상단 include 섹션에 추가
#include "ur5e_rt_controller/controllers/my_controller.hpp"
```

```cpp
std::vector<ControllerEntry> MakeControllerEntries()
{
  return {
    {"p_controller",     ...},
    {"pd_controller",    ...},
    {"pinocchio_controller", ...},
    {"clik_controller",  ...},
    {"operational_space_controller", ...},

    // ↓ 새 컨트롤러 등록
    {
      "my_controller",
      [](const std::string &) {return std::make_unique<urtc::MyController>();}
    },
  };
}
```

`config_key` 문자열은 YAML 파일명 스템(`my_controller.yaml`)과 일치해야 합니다.

---

## 5단계 — CMakeLists 수정 (`.cpp` 있는 경우)

`CMakeLists.txt` 에서 `target_sources` 에 추가합니다:

```cmake
target_sources(custom_controller PRIVATE
  src/controllers/p_controller.cpp
  src/controllers/pd_controller.cpp
  src/controllers/pinocchio_controller.cpp
  src/controllers/clik_controller.cpp
  src/controllers/operational_space_controller.cpp
  src/controllers/my_controller.cpp   # ← 추가
)
```

---

## 6단계 — 활성화

컨트롤러를 기본으로 시작하려면 `config/ur5e_rt_controller.yaml` 에서:

```yaml
/**:
  ros__parameters:
    initial_controller: "my_controller"  # config_key 또는 "MyController"
```

또는 런타임에 전환:

```bash
# 인덱스로 전환 (등록 순서 기준, 0부터 시작)
ros2 topic pub /custom_controller/controller_type std_msgs/msg/Int32 "data: 5"

# 게인 업데이트 (UpdateGainsFromMsg 레이아웃 참고)
ros2 topic pub /custom_controller/controller_gains std_msgs/msg/Float64MultiArray \
  "data: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]"
```

---

## Pinocchio 기반 컨트롤러 추가 시

URDF 파일이 필요한 컨트롤러는 팩토리 람다에서 `urdf_path` 인자를 사용합니다:

```cpp
#include "ur5e_rt_controller/controllers/my_pinocchio_controller.hpp"

// MakeControllerEntries() 내부:
{
  "my_pinocchio_controller",
  [](const std::string & p) {
    return std::make_unique<urtc::MyPinocchioController>(
      p, urtc::MyPinocchioController::Gains{});
  }
},
```

헤더에는 Pinocchio 경고 억제 pragma를 사용합니다:

```cpp
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/...>
#pragma GCC diagnostic pop
```

CMakeLists에서 Pinocchio 링크는 이미 `pinocchio::pinocchio`로 설정되어 있으므로
별도 수정 없이 사용할 수 있습니다.

---

## 기존 컨트롤러 레이아웃 참고

### `~/controller_gains` 토픽 레이아웃 요약

| 컨트롤러 | 인덱스 | gains 배열 레이아웃 |
|---|:---:|---|
| `PController` | 0 | `[kp×6]` (6개) |
| `PDController` | 1 | `[kp×6, kd×6]` (12개) |
| `PinocchioController` | 2 | `[kp×6, kd×6, grav, cori]` (14개) |
| `ClikController` | 3 | `[kp×3, damping, null_kp, enable_null]` (6개) |
| `OperationalSpaceController` | 4 | `[kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, grav]` (14개) |

인덱스는 `MakeControllerEntries()` 의 등록 순서에 따라 결정됩니다.

---

## 체크리스트

새 컨트롤러 추가 시 아래 항목을 확인하세요:

- [ ] `Compute()` 가 `noexcept` 로 선언되어 있다
- [ ] `Name()` 이 유일한 문자열을 반환한다
- [ ] `LoadConfig()` 에서 YAML 노드 없는 경우를 처리한다 (`if (!cfg) return;`)
- [ ] `UpdateGainsFromMsg()` 에서 배열 크기를 검증한다 (`if (gains.size() < N) return;`)
- [ ] RT 경로에 동적 메모리 할당이 없다
- [ ] `MakeControllerEntries()` 에 등록되어 있다
- [ ] YAML 파일의 최상위 키가 `config_key` 와 일치한다
- [ ] `.cpp` 파일이 있으면 `CMakeLists.txt` 에 추가했다
