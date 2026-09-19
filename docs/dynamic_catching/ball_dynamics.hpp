#pragma once
// catching_core/ball_dynamics.hpp — **시뮬레이션 fixture 전용** (L0 §1).
// v0.3 에서 제어 PC 는 vision 예측을 재전파하지 않는다 (마스터 §5.2). 이 헤더는
// L8 의 vision 대역 발행기와 투척 생성기에서만 쓴다. 실시간 제어 경로에서 include 하지 말 것.
// 헤더 전용, 할당 없음, noexcept.
#include <Eigen/Core>

#include <cmath>
#include <cstdint>

namespace catching::ball {

using State = Eigen::Matrix<double, 7, 1>;  // [p(3) m; v(3) m/s; k 1/m]
using Mat7 = Eigen::Matrix<double, 7, 7>;

struct Model {
  Eigen::Vector3d g{0.0, 0.0, -9.81};  // W [m/s^2]
  double v_eps{1e-3};  // [m/s] |v| 하한: vv^T/|v| 항의 특이점 회피에만 사용
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
  const double vn = std::max(v_true, m.v_eps);  // 특이점 방어용
  const double k = x(6);
  Mat7 A = Mat7::Zero();
  A.block<3, 3>(0, 3).setIdentity();
  A.block<3, 3>(3, 3) = -k * (v_true * Eigen::Matrix3d::Identity() + (v * v.transpose()) / vn);
  A.block<3, 1>(3, 6) = -v_true * v;  // f() 와 동일한 |v| 사용
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
  const State k1 = f(m, x);
  const Mat7 P1 = jacobian(m, x) * Phi;
  const State x2 = x + 0.5 * h * k1;
  const State k2 = f(m, x2);
  const Mat7 P2 = jacobian(m, x2) * (Phi + 0.5 * h * P1);
  const State x3 = x + 0.5 * h * k2;
  const State k3 = f(m, x3);
  const Mat7 P3 = jacobian(m, x3) * (Phi + 0.5 * h * P2);
  const State x4 = x + h * k3;
  const State k4 = f(m, x4);
  const Mat7 P4 = jacobian(m, x4) * (Phi + h * P3);
  x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  Phi += (h / 6.0) * (P1 + 2.0 * P2 + 2.0 * P3 + P4);
}

// propagate() 결과. steps == max_steps 는 포화를 뜻하지 않으므로(정확히 나누어떨어지는
// 경우가 있다) 호출자는 반드시 truncated 를 본다.
struct PropagateResult {
  int steps{0};
  bool truncated{false};  // true: 스텝 상한 때문에 h > h_max 로 적분함
  double h_used{0.0};     // [s] 실제 사용한 스텝 크기
};

// [0, T] 전파. 부분 스텝 수 상한으로 최악 연산량을 고정한다.
inline PropagateResult propagate(const Model& m, State& x, double T, double h_max,
                                 int max_steps) noexcept {
  if (!(T > 0.0) || !(h_max > 0.0) || max_steps <= 0)
    return {};
  int n = static_cast<int>(std::ceil(T / h_max));
  const bool truncated = (n > max_steps);
  if (truncated)
    n = max_steps;
  const double h = T / n;
  for (int i = 0; i < n; ++i)
    x = rk4(m, x, h);
  return {n, truncated, h};
}

}  // namespace catching::ball
