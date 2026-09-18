# L0 — Core: 공용 타입, 파라미터 검증, 공 동역학(시뮬레이션 fixture 전용)

- 브랜치: `feat/catching-L0-core` (기준: `TBD-GIT-01`)
- 패키지: `catching_core` (헤더 전용 + 파라미터 검증)
- 선행: 단계 W
- 산출물: `types.hpp`, `param_validation.hpp`, `ball_dynamics.hpp`(fixture 전용)

---

## 1. 범위 / 비범위

범위: 시간·좌표 기본 타입, YAML 파라미터 검증기(TBD 검출 포함), 그리고 **시뮬레이션 fixture용** 공 운동 모델과 수치 적분.

비범위:
- 공 상태 추정·예측 (vision 노드 소관).
- **메시지 정의.** v0.2의 `catching_msgs`(`BallState`, `BallModel`)는 폐기했다. 입력은 vision이 이미 발행하는 `sensor_msgs/PointCloud2`이고 제어 PC가 정의하지 않는다(마스터 §5).
- 실시간 경로의 궤적 전파 (제어 PC는 재전파하지 않는다, 마스터 §5.2).

**`ball_dynamics.hpp`의 위상 변경 `[확정]`.** v0.2에서 이 헤더는 "두 PC 공용 라이브러리"였고 `kModelVersion`으로 계약을 맞췄다. v0.3에서는 vision이 예측을 전담하므로 **실시간 제어 경로에서 쓰이지 않는다.** 남는 용도는 두 가지뿐이다.

1. L8 시뮬레이션 fixture: MuJoCo 참값에서 vision 역할을 대신하는 발행기를 만들 때(L8 §4.3).
2. L8 투척 생성기: 목표 지점에 도달하는 초기속도를 슈팅으로 푸는 데 상태전이행렬이 필요하다(L8 §4.2).

따라서 `kModelVersion`, 모델 버전 협상, 항력계수 $k$ 의 실시간 식별은 전부 삭제한다. $k$ 식별은 fixture를 실제 궤적에 맞출 때만 쓴다.

## 2. 코드 확인 게이트

단계 W에서 처리한다(`WORKSPACE_ANALYSIS.md` W1, W2, W6).

| ID | 확인 항목 | 방법 | 결과 기록 |
|---|---|---|---|
| G0-1 | SeqLock/SPSC 원시형 헤더 위치와 API (단일 writer 가정, reader 재시도 정책) | 소스 확인 (W2-2) | TBD-RTC-01 |
| G0-2 | 시간 타입 규약 (ns 정수 / double s, `rclcpp::Time` clock type) | 소스 확인 (W2-3) | TBD-RTC-02 |
| G0-3 | 기존 YAML 파라미터 로딩 패턴 (`generate_parameter_library` 사용 여부 등) | 소스 확인 (W2-6) | TBD-RTC-03 |
| G0-4 | 포구 코드를 새 패키지로 둘지 기존 패키지에 넣을지, 이름 확정 | 사용자 결정 (W1-5) | TBD-WS-02 |
| G0-5 | MJCF의 공 유체 모델 설정(항력 식이 본 모델과 다름을 전제) — fixture 한정 | MJCF 확인 (W6-3) | TBD-SIM-02 |

게이트 결과가 본 문서 가정과 다르면, 본 문서를 먼저 수정한 뒤 구현한다.

## 3. 참고자료

[R8] 공분산 전파·변분방정식, [R14] MuJoCo. RK4와 변분방정식은 수치해석 표준 내용(등급 a).

## 4. 수학적 이론

### 4.1 공 운동 모델 `[시뮬레이션 fixture 전용]`

**이 절은 실시간 제어 경로와 무관하다**(§1). L8의 vision 대역 발행기와 투척 생성기에만 쓴다.

상태 $x=[p;\,v;\,k]\in\mathbb R^7$ (`W` 기준). 이차 항력 모델:

$$\dot p=v,\qquad \dot v=g-k\Vert v\Vert v,\qquad \dot k=0,\qquad k=\frac{\rho C_dA}{2m}$$

- $k$는 공기밀도 $\rho$, 항력계수 $C_d$, 단면적 $A$, 질량 $m$에서 계산할 수 있으나, **시뮬레이션에서는 MJCF 유체 모델의 식이 다르므로 투척 데이터로 $k$를 최소제곱 식별한다** (§7 L0.3).
- 회전(Magnus) 효과는 모델에 넣지 않는다. 잔차는 vision PC 추정기의 공정잡음이 흡수한다고 가정한다(가정 명시).

### 4.2 Jacobian

$A=\partial f/\partial x$:

$$A=\begin{bmatrix}0&I_3&0\\0&-k\Big(\Vert v\Vert I_3+\dfrac{vv^\top}{\Vert v\Vert}\Big)&-\Vert v\Vert v\\0&0&0\end{bmatrix}$$

유도: $\partial(\Vert v\Vert v)/\partial v=\Vert v\Vert I+v\,\partial\Vert v\Vert/\partial v=\Vert v\Vert I+vv^\top/\Vert v\Vert$.

$\Vert v\Vert\to0$에서 $vv^\top/\Vert v\Vert$ 항이 정의되지 않으므로 **그 항에만** $\Vert v\Vert\leftarrow\max(\Vert v\Vert,v_\epsilon)$로 하한을 둔다. 비행 중 공 속도는 0이 되지 않지만(정점에서도 수평 성분 존재) 수직 투척을 위한 방어다.

$\partial\dot v/\partial k=-\Vert v\Vert v$ 와 $-k\Vert v\Vert I$ 항에는 clamp를 걸지 않는다. clamp를 걸면 $\Vert v\Vert<v_\epsilon$ 에서 $A$ 와 $f$ 가 서로 다른 모델이 되어 STM이 $v_\epsilon/\Vert v\Vert$ 배 틀린다(`test_l0.cpp` 테스트 5가 회귀 검사).

### 4.3 이산화: RK4와 상태전이행렬

상태는 고전 RK4로 적분한다(국소 오차 $O(h^5)$, 전역 $O(h^4)$).

공분산 전파(L2)에 필요한 상태전이행렬 $\Phi(t,t_0)=\partial x(t)/\partial x(t_0)$는 변분방정식

$$\dot\Phi=A(x(t))\,\Phi,\qquad \Phi(t_0)=I$$

을 **상태와 같은 RK4 단계**로 함께 적분한다. 상태만 RK4로 적분하고 $\Phi\approx I+Ah$로 근사하면 1차 정확도로 떨어진다.

### 4.4 Sanity check (구현 테스트로 고정)

1. $k=0$이면 $p(t)=p_0+v_0t+\tfrac12gt^2$와 일치 (특수해).
2. 역학적 에너지 $E=\tfrac12\Vert v\Vert^2-g^\top p$는 비증가: $\dot E=v^\top(g-k\Vert v\Vert v)-g^\top v=-k\Vert v\Vert^3\le0$.
3. 스텝을 절반으로 줄이면 전역 오차가 약 1/16 (4차 수렴).
4. $\Phi$와 중심 유한차분 Jacobian 일치.
5. (회귀) $\Vert v\Vert<v_\epsilon$ 에서 $\partial\dot v/\partial k$ 가 닫힌식 $-\Vert v\Vert v$ 와 일치. 유한차분은 $g=9.81$ 에 묻혀 자릿수 손실이 나므로 닫힌식과 직접 비교한다.
6. (회귀) `propagate()` 의 `truncated` 계약. $n=$`max_steps` 는 포화를 뜻하지 않는다(정확히 나누어떨어지는 경우가 있다).

`test_l0.cpp` 실행 결과(v0.2): 해석해 오차 $1.6\times10^{-14}$ m, 에너지 증가분 0, 오차비 15.25, STM 유한차분 차이 $2.8\times10^{-9}$ (0.5 s 구간, $h=2$ ms), 저속 $\partial A/\partial k$ 상대오차 0, `truncated` 계약 통과.

### 4.5 시간 표현 `[권장]`

epoch 기준 system time을 double 초로 두면 분해능이 약 $2\times10^{-7}$ s라 포구 용도에 충분하지만, 스냅샷에는 `int64_t` 나노초를 저장하고 계산 직전에 상대 시간 $\Delta t=(t-t_{meas})$만 double로 바꾼다. 누적 합산에서 반올림 오차가 쌓이지 않게 하기 위함이다.

## 5. C++ 구현

### 5.1 `ball_dynamics.hpp` (참조 구현, 검증 완료)

```cpp
#pragma once
// catching_core/ball_dynamics.hpp — **시뮬레이션 fixture 전용** (L0 §1).
// v0.3 에서 제어 PC 는 vision 예측을 재전파하지 않는다 (마스터 §5.2). 이 헤더는
// L8 의 vision 대역 발행기와 투척 생성기에서만 쓴다. 실시간 제어 경로에서 include 하지 말 것.
// 헤더 전용, 할당 없음, noexcept.
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

namespace catching::ball {

using State = Eigen::Matrix<double, 7, 1>;   // [p(3) m; v(3) m/s; k 1/m]
using Mat7  = Eigen::Matrix<double, 7, 7>;

struct Model {
  Eigen::Vector3d g{0.0, 0.0, -9.81};  // W [m/s^2]
  double v_eps{1e-3};                  // [m/s] |v| 하한: vv^T/|v| 항의 특이점 회피에만 사용
};

// 연속시간 벡터장 f(x). k는 상수 상태(dk/dt = 0).
[[nodiscard]] inline State f(const Model& m, const State& x) noexcept {
  const Eigen::Vector3d v = x.segment<3>(3);
  State dx;
  dx.segment<3>(0) = v;
  dx.segment<3>(3) = m.g - x(6) * v.norm() * v;
  dx(6) = 0.0;
  return dx;
}

// A = df/dx (연속시간 Jacobian)
//
// v_eps clamp 는 vv^T/|v| 항에만 적용한다. d(vdot)/dk = -|v| v 에는 clamp 를 걸지 않는다.
// (clamp 를 걸면 |v| < v_eps 에서 f() 와 A 가 서로 다른 모델이 되어 STM 이 v_eps/|v| 배 틀린다.)
[[nodiscard]] inline Mat7 jacobian(const Model& m, const State& x) noexcept {
  const Eigen::Vector3d v = x.segment<3>(3);
  const double v_true = v.norm();
  const double vn = std::max(v_true, m.v_eps);   // 특이점 방어용
  const double k = x(6);
  Mat7 A = Mat7::Zero();
  A.block<3, 3>(0, 3).setIdentity();
  A.block<3, 3>(3, 3) = -k * (v_true * Eigen::Matrix3d::Identity() + (v * v.transpose()) / vn);
  A.block<3, 1>(3, 6) = -v_true * v;             // f() 와 동일한 |v| 사용
  return A;
}

// RK4 1스텝 (상태만)
[[nodiscard]] inline State rk4(const Model& m, const State& x, double h) noexcept {
  const State k1 = f(m, x);
  const State k2 = f(m, x + 0.5 * h * k1);
  const State k3 = f(m, x + 0.5 * h * k2);
  const State k4 = f(m, x + h * k3);
  return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// RK4 1스텝 (상태 + 상태전이행렬 Phi). 변분방정식 dPhi/dt = A(x) Phi 를 같은 단계로 적분.
inline void rk4WithStm(const Model& m, State& x, Mat7& Phi, double h) noexcept {
  const State k1 = f(m, x);                    const Mat7 P1 = jacobian(m, x) * Phi;
  const State x2 = x + 0.5 * h * k1;           const State k2 = f(m, x2);
  const Mat7 P2 = jacobian(m, x2) * (Phi + 0.5 * h * P1);
  const State x3 = x + 0.5 * h * k2;           const State k3 = f(m, x3);
  const Mat7 P3 = jacobian(m, x3) * (Phi + 0.5 * h * P2);
  const State x4 = x + h * k3;                 const State k4 = f(m, x4);
  const Mat7 P4 = jacobian(m, x4) * (Phi + h * P3);
  x   += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  Phi += (h / 6.0) * (P1 + 2.0 * P2 + 2.0 * P3 + P4);
}

// propagate() 결과. steps == max_steps 는 포화를 뜻하지 않으므로(정확히 나누어떨어지는
// 경우가 있다) 호출자는 반드시 truncated 를 본다.
struct PropagateResult {
  int  steps{0};
  bool truncated{false};   // true: 스텝 상한 때문에 h > h_max 로 적분함
  double h_used{0.0};      // [s] 실제 사용한 스텝 크기
};

// [0, T] 전파. 부분 스텝 수 상한으로 최악 연산량을 고정한다.
inline PropagateResult propagate(const Model& m, State& x, double T,
                                 double h_max, int max_steps) noexcept {
  if (!(T > 0.0) || !(h_max > 0.0) || max_steps <= 0) return {};
  int n = static_cast<int>(std::ceil(T / h_max));
  const bool truncated = (n > max_steps);
  if (truncated) n = max_steps;
  const double h = T / n;
  for (int i = 0; i < n; ++i) x = rk4(m, x, h);
  return {n, truncated, h};
}

}  // namespace catching::ball
```

RT 규칙: 모든 함수 `noexcept`, 고정 크기 Eigen, 힙 할당 없음. `propagate()`는 스텝 수 상한으로 최악 연산량을 고정한다.

**`truncated` 계약.** v0.1은 반환값(스텝 수)만으로 상한 도달을 감지하게 했는데, $T=0.2$, `h_max=2 ms`, `max_steps=100` 이면 정확히 $n=100$ 이 나오면서도 포화가 아니다. 호출자가 $T>h_{max}\cdot max\_steps$ 를 따로 봐야 했다. v0.2는 이를 구조체 필드로 옮겼다. 참고로 이 문제에서 RK4의 정확도는 과분하다 — $T=0.5$ 를 $h=5$ ms로 잘라 적분해도 미세 스텝 대비 위치 오차가 $3.4\times10^{-12}$ m다. `truncated`는 정확도보다 **가정 위반 감지**(너무 오래된 스냅샷)를 위한 신호다.

### 5.2 `types.hpp`

```cpp
namespace catching {
using Nanoseconds = std::int64_t;               // system time 또는 sim time, 규약은 G0-2 결과를 따름

inline constexpr int kMaxArmDof     = 7;
inline constexpr int kMaxHandDof    = 16;
inline constexpr int kMaxFingertips = 4;
inline constexpr int kMaxTrajSamples = 512;      // L2 `traj::kMaxSamples` 와 같은 값. G2-1 실측 N 상한으로 함께 줄인다.

// 궤적 스냅샷은 traj::PredictedTrajectory (L2 traj_sampler.hpp) 를 그대로 쓴다.
// 계획용 공분산 버퍼 (non-RT 전용, L2 §4.5 의 1안).
// L0 은 L2 헤더에 의존하지 않는다 — 길이는 위 kMaxTrajSamples 로 두고,
// L2 쪽에서 static_assert(traj::kMaxSamples == kMaxTrajSamples) 로 묶는다.
struct CovBuffer {
  int n{0};
  std::array<Eigen::Matrix<double, 6, 6>, kMaxTrajSamples> S{};
};

// vision 메시지에 track_id 가 없어 L1 이 만들어내는 대체 식별자 (L1 §4.4).
struct TrackEpoch { std::uint32_t value{0}; };
}  // namespace catching
```

`PlanSnapshot`은 L3, `RobotSnapshot`은 L1, `PredictedTrajectory`는 L2에서 정의한다. v0.2의 `BallSnapshot`은 삭제했다.

### 5.3 `param_validation.hpp`

- 입력: YAML 로드 결과(구조체). 문자열 `"TBD"` 또는 NaN이 남은 필드를 목록으로 반환한다.
- 각 필드: `(name, value, lo, hi, unit)` 표에 따라 범위 검사.
- 결과는 `ValidationReport{bool armable; fixed-capacity list of failed keys}`. `armable=false`면 L7이 `ARMED` 진입을 거부한다.
- 로딩·검증은 configure 단계(non-RT)에서만 수행한다.

### 5.4 메시지 — 없음

v0.3에서 `catching_msgs`를 폐기했다. 입력은 vision의 `sensor_msgs/PointCloud2`(마스터 §5.1)이고, 파싱은 L1 §5.1이 맡는다. 제어 PC가 정의하는 ROS 인터페이스는 진단·이벤트용이 있다면 그것뿐이며, 그 필요 여부는 단계 W(W2-7 로깅 관례) 이후에 정한다.

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `core.ball.diameter` | double | m | `TBD` | 0.02–0.3 | TBD-BALL-01. L3 $d_{eff}$·L7 충격량에 쓰임 |
| `core.ball.mass` | double | kg | `TBD` | 0.005–1.0 | TBD-BALL-01. L7 §4.7 충격량 |
| `core.ball.restitution` | double | – | `TBD` | 0–1 | TBD-BALL-01. L3 §4.5 $d(1+1/e)$ |
| `sim.ball.gravity` | double[3] | m/s² | `[0, 0, -9.81]` | 크기 9.7–9.9 | fixture 전용 |
| `sim.ball.drag_k` | double | 1/m | `TBD` | 0–0.2 | fixture 전용, L0.3 식별값 |
| `sim.ball.v_eps` | double | m/s | 1e-3 | 1e-6–1e-1 | 특이점 방어 (fixture) |
| `sim.integrator.h_max` | double | s | 0.002 | 1e-4–0.01 | fixture 적분 스텝 |
| `sim.integrator.max_steps` | int | – | 100 | 1–500 | fixture 연산량 상한 |

공 물성(`core.ball.*`)은 제어 경로에도 필요하므로 남기고, 운동 모델 파라미터는 `sim.*`로 옮겼다.

## 7. 단위 기술 구현 순서

- **L0.1** `types.hpp`, `param_validation.hpp` + TBD 검출 테스트. 단계 W의 W2-3(시간 타입)·W2-6(파라미터 로딩) 결과를 반영한다.
- **L0.2** `ball_dynamics.hpp` 구현 + §4.4 테스트 6종 (참조: `test_l0.cpp`, 이식 시 GTest). **fixture 전용이므로 RT 게이트는 적용하지 않는다** — 다만 참조 구현이 이미 `noexcept`·무할당이라 그대로 둔다.
- **L0.3** 시뮬레이션 $k$ 식별 도구 (`tools/identify_drag.py`): MuJoCo에서 서로 다른 초기속도로 공을 던져 참값 궤적을 기록하고, $k$를 스칼라 최소제곱으로 추정. 산출값을 `sim.ball.drag_k`에 기록. **L8 착수 시점까지 미뤄도 된다.**

v0.2의 L0.1(`catching_msgs` 생성)은 삭제했다.

## 8. 디버깅 방법

- 해석해 대조 실패: `g` 부호·축 방향, 단위(deg/rad 혼동 아님을 확인), RK4 계수 오타 순으로 확인.
- STM 불일치: Jacobian의 $vv^\top/\Vert v\Vert$ 항 누락, $\partial a/\partial k$ 부호, 중간 단계 $\Phi$ 갱신식 확인.
- $k$ 식별 잔차가 큰 경우: MJCF 유체 모델이 속도의 1차 항(점성)을 포함하는지 확인. 포함되면 본 모델과 구조가 달라 잔차가 남는다. 이 경우 식별 결과와 잔차 크기를 기록하고 사용자에게 보고한다(모델 확장은 사용자 결정).

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G0-A | §4.4 여섯 테스트 통과 (해석해 오차 < 1e-9 m, 오차비 12–20, STM 차이 < 1e-6, 저속 ∂A/∂k 상대오차 < 1e-12, `truncated` 계약) | `[SIM-ANY]` |
| G0-B | 모든 함수 `noexcept`, 할당 0 (할당 카운터 훅 또는 RTC 검사 도구) | `[SIM-ANY]` |
| G0-C | TBD 필드가 있는 YAML에서 `armable=false`. `robot.hand.q_close != q_pre`(L6 §4.2 — 두 자세가 같으면 `T_close` 식별이 무의미) 검사 포함 | `[SIM-ANY]` |
| G0-D | 시뮬레이션 $k$ 식별 후 1 s 궤적 위치 RMS 잔차 기록 (합격 임계는 사용자 결정) | `[SIM-P1B]` |

## 10. 미확정 항목

TBD-BALL-01, TBD-SIM-02, TBD-WS-02, TBD-RTC-01~03. (`TBD-MSG-01`은 `catching_msgs` 폐기로 삭제.)
