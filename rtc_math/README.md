# rtc_math

robot-agnostic 기하 · 제어 수학 라이브러리 (header-only, Eigen-only). **유형**: 라이브러리 패키지

## 개요

SE(3) Lie-group 원시 연산, task-space pose / velocity(twist) error 정의, wrench 변환, RPY → R 변환, approach-axis alignment error 를 제공한다. 어떤 robot · URDF · joint 도 모르는 순수 수학 계층이며, URDF → 모델 구축은 `rtc_urdf_bridge`, 컨트롤러 계약은 `rtc_controller_interface` 의 책임이다.

- **Eigen-only 코어** — RT-safe (fixed-size, `noexcept`, zero-heap, no throw)
- **Pinocchio 는 optional adapter** — `pinocchio_adapter.hpp` 만 `pinocchio::SE3/Motion` 오버로드를 제공하고, Pinocchio 가 있을 때만 컴파일 · 설치된다
- **최저 의존 계층** — `rtc_base` · `rtc_msgs` 와 같은 independent 패키지. `rtc_controllers` · `rtc_tsid` 등이 하향 의존한다 (소비자 예: [rtc_urdf_bridge](../rtc_urdf_bridge/README.md))

## 빠른 시작

빌드·테스트는 [루트 README](../README.md#빠른-시작) 를 따른다.

```bash
./build.sh --tests -p rtc_math
ros2 run rtc_math se3_error_compare /tmp/se3out
python3 $(ros2 pkg prefix rtc_math)/share/rtc_math/scripts/plot_se3_compare.py /tmp/se3out
```

다른 패키지에서 쓰려면 (`package.xml` 에 `<depend>rtc_math</depend>`):

```cmake
find_package(rtc_math REQUIRED)
ament_target_dependencies(my_target rtc_math)
```

```cpp
#include "rtc_math/se3/pose_error.hpp"
namespace se3 = rtc::math::se3;

const Eigen::Matrix<double, 6, 1> e =
    se3::computePoseError(T, T_d, se3::ErrorType::SplitWorld);
```

## 인터페이스

namespace 는 `rtc::math::se3`, 6D ordering 은 `[linear(3); angular(3)]` (= `pinocchio::Motion`). 정의 · 프레임 · 스케일 · 출처 표는 [se3/README.md](include/rtc_math/se3/README.md) 가 갖는다.

| 헤더 (`rtc_math/se3/`) | 내용 |
|---|---|
| `so3.hpp` | hat / vee, `log3` / `exp3`, left Jacobian, `Jlog3`, `RpyToRotationZyx` |
| `se3.hpp` | `log6` / `exp6`, adjoint, `Jlog6` (analytic + numerical) |
| `pose_error.hpp` | `ErrorType` (5 정의 + legacy), `computePoseError` |
| `velocity_error.hpp` | `computeVelocityError`, `exactPoseErrorRate`, twist transport |
| `wrench.hpp` | `transformWrench` (wrench 는 twist 와 다르게 변환) |
| `axis_align.hpp` | approach-axis alignment error · Jacobian · `ω` reference (2-DoF) |
| `pinocchio_adapter.hpp` | (optional) `pinocchio::SE3/Motion` 오버로드 |

## 디렉토리

```
rtc_math/
├── include/   헤더 (rtc_math/se3/)
├── examples/  error 정의 비교 실험 (S1-S5, CSV 출력)
├── scripts/   비교 실험 plot (matplotlib)
└── test/      gtest (Pinocchio 교차검증은 발견 시에만)
```

## 더 읽기

- [se3/README.md](include/rtc_math/se3/README.md) — error 정의 · 규약 · 선택 가이드 · 한계

## 의존성

Eigen3 (필수), Pinocchio (optional) — [package.xml](package.xml) · MIT
