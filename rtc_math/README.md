# rtc_math

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

## 개요

RTC 프레임워크의 **robot-agnostic 기하/제어 수학 라이브러리**입니다. Header-only,
**Eigen-only** 수치 코어로 SE(3) Lie-group 원시 연산과 task-space pose/velocity(twist)
error 정의를 제공합니다. 어떤 robot/URDF/joint 도 모르는 순수 수학 계층이며, URDF→모델
*구축*(`rtc_urdf_bridge`)이나 컨트롤러 *계약*(`rtc_controller_interface`)과 책임이 분리됩니다.

**설계 원칙:**
- **Eigen-only 코어** — `so3/se3/pose_error/velocity_error` 는 Eigen 만 의존. RT-safe
  (fixed-size, noexcept, zero-heap, no throw).
- **Pinocchio = optional adapter** — `pinocchio_adapter.hpp` 만 `pinocchio::SE3/Motion`
  오버로드를 제공하며, Pinocchio 가 있을 때만 컴파일/설치된다. 코어 재사용에 Pinocchio 불필요.
- **최저 의존 계층** — `rtc_base`/`rtc_msgs` 와 같은 independent 노드. 컨트롤러·솔버가
  하향 의존 (`rtc_controllers`, `rtc_tsid` 등).

## 구성

```
include/rtc_math/se3/
├── so3.hpp              # hat/vee, log3/exp3, leftJacobian(±inv), Jlog3
├── se3.hpp              # log6/exp6, adjoint, Jlog6 (analytic + numerical)
├── pose_error.hpp       # ErrorType(5 정의 + legacy) + computePoseError
├── velocity_error.hpp   # computeVelocityError / exactPoseErrorRate / transport
├── pinocchio_adapter.hpp# (optional) pinocchio::SE3/Motion 오버로드
└── README.md            # error 정의·규약·검증 스펙
examples/se3_error_compare.cpp   # error 정의 비교 실험 (S1–S5, CSV 출력; Eigen-only)
scripts/plot_se3_compare.py      # 비교 실험 plot (matplotlib)
test/test_se3_module.cpp         # Pinocchio 교차검증 + 유한차분 (Pinocchio 발견 시에만 빌드)
```

namespace 는 `rtc::math::se3`. 6D ordering 은 `[linear(3); angular(3)]`
(= `pinocchio::Motion`). error 정의·프레임·스케일·출처 표는
[se3/README.md](include/rtc_math/se3/README.md) 참조.

## 사용

```cpp
#include "rtc_math/se3/pose_error.hpp"
#include "rtc_math/se3/velocity_error.hpp"
namespace se3 = rtc::math::se3;

const Eigen::Matrix<double, 6, 1> e =
    se3::computePoseError(T, T_d, se3::ErrorType::SplitWorld);
```

비교 실험:

```bash
ros2 run rtc_math se3_error_compare /tmp/se3out
python3 $(ros2 pkg prefix rtc_math)/share/rtc_math/scripts/plot_se3_compare.py /tmp/se3out
```
