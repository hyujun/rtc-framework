# SE(3) Pose/Twist Error 모듈 (`rtc::math::se3`)

SE(3) pose error 와 그에 **대응하는 velocity(twist) error** 를 계산하는 header-only, Eigen-only 수치 코어. **유형**: 라이브러리 패키지의 모듈

## 개요

[rtc_math](../../../README.md) 의 INTERFACE target 으로 export 되므로 `rtc_math` 에 의존하는 패키지는 헤더를 그대로 쓴다. 수치 코어는 Eigen 만 쓰고, Pinocchio 는 optional adapter (`pinocchio_adapter.hpp`) 와 테스트에서만 쓴다 — Pinocchio 가 발견될 때만 컴파일 · 설치된다.

### 규약

| 항목 | 선택 |
|---|---|
| Twist / 6D ordering | **`[linear(3); angular(3)]`** — `pinocchio::Motion` 과 같다. Modern Robotics · Featherstone 의 `[angular, linear]` 와 섞지 않는다 |
| Pose 타입 | 코어는 `Eigen::Isometry3d` (`T`: world ← frame), adapter 는 `pinocchio::SE3` |
| Rotation | 내부는 Hamilton quaternion. `log3` 은 quaternion + `atan2` (θ∈[0,π], θ≈0,π 에서 강건 — `acos` 미사용) |
| 단위 | SI (m, rad) |
| RT 안전 | 전부 `noexcept`, fixed-size Eigen, heap 없음, throw 없음 |

## 빠른 시작

```cpp
#include "rtc_math/se3/pose_error.hpp"
#include "rtc_math/se3/velocity_error.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"  // optional: pinocchio::SE3/Motion 오버로드
namespace se3 = rtc::math::se3;
```

테스트 (`test/test_se3_module.cpp` · `test/test_axis_align.cpp`) 는 `./build.sh --tests -p rtc_math` 로 빌드한다. 비교 실험은 [rtc_math README](../../../README.md#빠른-시작) 를 따른다.

## 인터페이스

### ErrorType — 정의 · 프레임 · 회전 스케일 · 출처

순수 회전 θ 일 때 회전 성분의 크기를 Scale 열에 적었다.

| `ErrorType` | Pose error | Frame | Scale | 출처 |
|---|---|---|---|---|
| `SplitWorld` | `[p_d−p ; log3(R_d Rᵀ)]` | world | θ | Caccavale 1999, Siciliano 2009 |
| `BodyLog6` | `log6(T⁻¹ T_d)` | body screw | θ | MLS 1994, Modern Robotics §11.3 |
| `SpatialLog6` | `log6(T_d T⁻¹)` | spatial screw | θ | MLS 1994 (`= Ad_T · BodyLog6`) |
| `SplitLee` | `[p_d−p ; ½(R_dRᵀ−RR_dᵀ)∨]` | world | sinθ | Lee et al. 2010 |
| `SplitQuat` | `[p_d−p ; 2·sign(w)·vec(q_d⊗q⁻¹)]` | world | 2sin(θ/2) | Nakanishi 2008 |
| `SplitBodyRot` | `[p_d−p ; log3(Rᵀ R_d)]` | base 위치 + **body** 회전 | θ | workspace legacy |

`isBodyFrame(t)` 는 `BodyLog6` 에서만 `true` 이다 (twist 가 전부 body frame).

### Velocity error ↔ Pinocchio frame 일치

twist `ν` 는 `T` 의 접공간, `ν_d` 는 `T_d` 의 접공간에 있어서 단순한 `ν_d − ν` 는 오차가 0 근처일 때만 맞다. `computeVelocityError` 는 type 별로 올바른 adjoint transport 를 적용한다 (Bullo & Murray 1999). **입력 frame 전제** — twist 가 다른 frame 이면 먼저 transport helper 를 적용한다.

| `ErrorType` | 입력 frame | `computeVelocityError` 반환 | Pinocchio `getFrameVelocity` |
|---|---|---|---|
| `SplitWorld` / `SplitLee` / `SplitQuat` | LOCAL_WORLD_ALIGNED | `ν_d − ν` (회전 성분은 1차) | `LOCAL_WORLD_ALIGNED` |
| `BodyLog6` | LOCAL | `Ad_{T⁻¹T_d}·ν_d − ν` | `LOCAL` |
| `SpatialLog6` | WORLD (spatial) | `ν_d − Ad_{T_d T⁻¹}·ν` | `WORLD` |
| `SplitBodyRot` | LWA 선형 + LOCAL 각 | `ν_d − ν` (1차) | 혼합 |

Transport helper 는 두 쌍이다. `twistWorldToLocal` / `twistLocalToWorld` (LWA↔LOCAL) 는 **회전만** `blockdiag(R,R)`, `twistLocalToSpatial` / `twistSpatialToLocal` (LOCAL↔WORLD) 는 **전체 adjoint** 이다. 둘을 혼동하는 것이 흔한 버그다.

`exactPoseErrorRate(T, T_d, ν, ν_d, type)` 는 Jacobian 을 포함한 **정확한** `ė` 를 반환한다 (`ν, ν_d` 는 LOCAL twist). 이방성 gain 보정과 유한차분 테스트의 oracle 로 쓴다. 스칼라 gain `k` 는 Ad transport 만 맞으면 큰 오차에서도 `ė = −k·e` 가 정확히 성립하고 (Modern Robotics §11.3), `Jlog` 보정은 **이방성 gain 행렬**에서만 필요하다: `e_ν^des ← (Jlog6)⁻¹·K·e`.

### Wrench transform (`wrench.hpp`)

wrench 는 twist 의 **dual** 이라 adjoint 로 변환하지 않는다. `T = ᴬT_B`, frame B 의 wrench `ᴮf` 에 대해 `transformWrench(T, f)` 는

`ᴬf = Ad_{ᴬT_B}^{-T}·ᴮf = adjoint(T.inverse()).transpose()·ᴮf`

를 반환한다 (`Ad_T^{-1} = Ad_{T^{-1}}` 로 계산하므로 6×6 역행렬 없음). ordering 은 `[force(3); torque(3)]` 로 twist 의 `[linear; angular]` 와 맞춘다. inverse-transpose 가 기계적 일률 `⟨ν, f⟩` 을 frame 불변으로 만든다 — `Ad_T^{T}` 를 쓰는 것이 전형적인 wrench 버그다. 방향은 `WrenchTransform.PowerDuality` 테스트가 고정한다.

### RPY → R (`so3.hpp`)

`RpyToRotationZyx(rpy)` 는 `R = Rz(yaw)·Ry(pitch)·Rx(roll)` (intrinsic Z-Y'-X'') 을 반환한다. 내부 수학은 quaternion 과 `log`/`exp` 로 하고, 메시지나 운영자가 `(x, y, z, r, p, y)` 를 쓰는 경계에서만 쓴다. 스칼라 3개가 아니라 `Vec3` 하나를 받는 것은 인접한 `double` 인자의 순서 바뀜을 막기 위해서다. gimbal lock 은 **역변환** (`R → rpy`) 의 문제이고 이 방향은 항상 잘 정의된다.

### Approach-axis alignment (`axis_align.hpp`)

body 의 한 축이 가리키는 방향만 구속하고 그 축 둘레 roll 은 자유로 두는 task 의 2-DoF 회전 오차. 단위축 `z` (현재) 와 `a_d` (목표)에 대해:

| 함수 | 반환 |
|---|---|
| `AxisAlignError(z, a_d, sin_eps)` | `e_a = θ·(z×a_d)/‖z×a_d‖`, `θ = atan2(‖z×a_d‖, zᵀa_d)`. `exp([e_a]×) z = a_d` 가 정확히 성립하고 `‖e_a‖ = θ` |
| `AxisAlignJacobian(z, a_d, sin_eps, jacobian_sin_floor)` | `ė_a = J_a ω` 의 `J_a` (ω 는 `z` 의 frame, `a_d` 고정) |
| `AxisAlignOmega(e_a, k_axis, w_max)` | `ω = k_axis·e_a` 를 `‖ω‖ ≤ w_max` 로 scale, `saturated` flag 포함 |

`z × a_d` 가 아니라 rotation vector 를 쓰는 이유는 `‖z×a_d‖ = sinθ` 가 180° 근처에서 0 이 되어 reference 가 거기서 멈추기 때문이다. `‖e_a‖ = θ` 는 연속이고 단조다. 모든 호출은 유한한 값을 반환하고 분기를 `AxisAlignRegion` 으로 알린다.

| Region | 조건 | `e_a` | `J_a` |
|---|---|---|---|
| `kAlignedDeadband` | `‖z×a_d‖ < sin_eps`, `c > 0` | 0 | 0 |
| `kAntiparallelDeadband` | `‖z×a_d‖ < sin_eps`, `c ≤ 0` | `π·u⊥` (`z` 에 수직인 고정 단위축, `z` 에 대해 결정적) | 0 |
| `kJacobianCapped` | `c < 0`, `sin_eps ≤ ‖z×a_d‖ < jacobian_sin_floor` | 정확 | floor 에서 평가한 `f`, `f'`: `‖J_a‖ ≲ π/floor`, floor 에서 연속 |
| `kInvalidInput` | 비유한 · 비단위 입력 (`|‖v‖−1| > 1e-6`), `sin_eps` / floor 가 [1e-12, 1) 밖 | 0 | 0 |

`J_a` 는 두 deadband 에서 `e_a` 가 상수라서 0 이다. 오차와 Jacobian 에 같은 `sin_eps` 를 넘겨 deadband 를 맞춘다. 기본값은 `sin_eps` 1e-6, `jacobian_sin_floor` 1e-3 (cap 은 ≈179.94° 이상에서만 시작). θ→π 에서 `J_a` 가 발산하는 것은 문제 자체의 성질 (회전축 미정의) 이라 숨기지 않고 cap 한다. 유도 · 구현 주의는 `axis_align.hpp` 헤더 주석이 갖는다.

### 정의 고르기

- **WBC / QP residual / impedance** 처럼 병진 · 회전 gain 을 따로 튜닝 → **`SplitWorld`** (SE(3) 에는 bi-invariant metric 이 없다, Park 1995)
- **Pinocchio 중심 geometric control** → **`BodyLog6`** (이방성 gain 이면 `+ Jlog6`)
- **큰 회전** → θ-선형 scale (`SplitWorld` / `BodyLog6`) 을 우선한다. `SplitLee` 는 θ→π 에서 멈춘다 (sinθ→0, 실험 S2)

### Limits (한계)

- SO(3) 위의 매끄러운 오차 기반 연속 feedback 은 기껏해야 almost-global 이다 (위상 장애, Bhat & Bernstein 2000)
- 이 모듈은 **velocity** 수준까지 정확하다. `log` 오차의 acceleration 수준 정확한 동역학에는 `J̇log` 항이 추가로 필요하다 (제공하지 않음)
- 같은 부류의 항이 하나 더 있다. LOCAL 오차율 `ė` 를 LWA 로 옮길 때 순간 회전 `R` 만 쓰면 frame 회전 결합 `blockdiag(Ṙ,Ṙ)·log6` 항이 빠진다 (`J̇log6` 누락의 SE(3) 대응). 이 근사는 표준 TSID 관행이고 LOCAL 오차에 대해서는 velocity 수준에서 정확하다. 이 항이 필요한 소비자 (예: `rtc_tsid` 의 `ComputeTaskVelocityError`) 는 acceleration 수준에서 근사임을 안다

## 더 읽기

- 검증: `test/test_se3_module.cpp` 와 `test/test_axis_align.cpp` 파일 헤더 (Pinocchio 교차검증은 `RTC_MATH_HAVE_PINOCCHIO` 로 gate)
- 비교 실험 S1–S5: `examples/se3_error_compare.cpp` 파일 헤더
- 참고문헌: Murray–Li–Sastry 1994 · Lynch & Park, *Modern Robotics* 2017 §11.3 · Bullo & Murray, *Automatica* 1999 · Caccavale et al., *IEEE T-RA* 1999 · Nakanishi et al., *IJRR* 2008 · Lee et al., *CDC* 2010 · Park, *ASME JMD* 1995 · Bhat & Bernstein, *SCL* 2000 · Solà et al., arXiv:1812.01537

## 의존성

Eigen3 (필수), Pinocchio (optional) — [package.xml](../../../package.xml)
