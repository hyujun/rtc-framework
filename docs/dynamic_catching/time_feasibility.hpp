#pragma once
// catching_planner/time_feasibility.hpp — 관절별 최소 도달시간 (속도·가속 한계, 목표 속도 0)
//                                          + gamma 창 부등식
#include <algorithm>
#include <cmath>

namespace catching::plan {

// 정지 상태에서 거리 D(>=0)를 이동해 정지하는 최소시간
[[nodiscard]] inline double tRest(double D, double w_max, double a_max) noexcept {
  if (D <= 0.0) return 0.0;
  const double w_peak = std::sqrt(a_max * D);
  return (w_peak <= w_max) ? 2.0 * std::sqrt(D / a_max) : D / w_max + w_max / a_max;
}

struct TMinResult {
  double t{0.0};
  bool   w0_clamped{false};   // |w0| > w_max 로 들어와 clamp 했음 → 반환값은 하한일 뿐
};

// (q0, w0) → (q1, 0) 최소시간.
//
// 전제: a_max > 0, w_max > 0. |w0| <= w_max 는 전제이지만 **검사한다**.
// |w0| > w_max 이면 초기 상태 자체가 속도 한계를 위반한 것이라 최소시간 문제가
// 정의되지 않는다(사다리꼴 분기에서 (w_max - w)/a < 0 인 음수 구간이 나온다).
// 이 경우 w0 를 clamp 하고 플래그를 세운다. 호출자는 플래그가 서면 해당 후보를
// 탈락시키거나(권장) 반환값을 하한으로만 쓴다.
[[nodiscard]] inline TMinResult tMinChecked(double q0, double w0, double q1,
                                            double w_max, double a_max) noexcept {
  TMinResult r{};
  if (!(a_max > 0.0) || !(w_max > 0.0)) return r;
  if (std::abs(w0) > w_max) { w0 = std::copysign(w_max, w0); r.w0_clamped = true; }

  const double d = q1 - q0;
  constexpr double eps = 1e-12;
  if (std::abs(d) < eps && std::abs(w0) < eps) return r;
  const double s = (std::abs(d) >= eps) ? std::copysign(1.0, d) : -std::copysign(1.0, w0);
  const double D = std::abs(d);
  const double w = s * w0;                                 // 목표 방향 속도 성분

  if (w < 0.0) {                                           // 반대 방향 이동 중: 먼저 정지
    r.t = -w / a_max + tRest(D + w * w / (2.0 * a_max), w_max, a_max);
    return r;
  }
  const double d_stop = w * w / (2.0 * a_max);
  if (d_stop > D) {                                        // 지나친 뒤 복귀
    r.t = w / a_max + tRest(d_stop - D, w_max, a_max);
    return r;
  }
  const double w_peak = std::sqrt(a_max * D + 0.5 * w * w);
  if (w_peak <= w_max) {                                   // 삼각
    r.t = (w_peak - w) / a_max + w_peak / a_max;
    return r;
  }
  const double cruise = D - (w_max * w_max - w * w) / (2.0 * a_max)
                          - w_max * w_max / (2.0 * a_max); // 사다리꼴
  r.t = (w_max - w) / a_max + w_max / a_max + cruise / w_max;
  return r;
}

// 편의 래퍼. 플래그를 버리므로 clamp 여부를 따로 봐야 하는 곳에서는 tMinChecked 를 쓸 것.
[[nodiscard]] inline double tMin(double q0, double w0, double q1,
                                 double w_max, double a_max) noexcept {
  return tMinChecked(q0, w0, q1, w_max, a_max).t;
}

// gamma 창: [g_min, g_max]
struct GammaWindow {
  double g_min{0.0}, g_max{0.0};
  [[nodiscard]] bool feasible() const noexcept { return g_min <= g_max; }
};

// L3 §4.5.
//   g_min = 1 - d_eff / (|v| * T_close_tot)          (손 폐쇄 하한)
//   g_max = min(v_dir_max, v_tcp_max) / |v|          (팔 속도 상한)
//
// v_tcp_max 는 L4 `reference.v_max` 와 같은 값이어야 한다. 이 인자를 빠뜨리면
// 계획이 통과시킨 gamma 가 L4 에서 속도 포화를 일으킨다.
// clamp 는 물리적 입력 범위(d_eff >= 0, v_dir_max >= 0, v_tcp_max >= 0)에서만
// 무clamp 판정과 동치다. 음수 v_dir_max 가 들어오면 clamp 가 g_max 를 0 으로 **올려**
// 판정을 뒤집으므로 검사한다. (v_dir_max 는 L3 §4.5 의 DLS 정규화식 결과라 구현·수치
// 문제로 음수가 나올 수 있다. tMinChecked 가 |w0| > w_max 를 검사하는 것과 같은 수준의 방어.)
[[nodiscard]] inline GammaWindow gammaWindow(double v_ball, double v_dir_max, double v_tcp_max,
                                             double d_eff, double t_close_total,
                                             bool* input_invalid = nullptr) noexcept {
  const bool bad = !(d_eff >= 0.0) || !(v_dir_max >= 0.0) || !(v_tcp_max >= 0.0)
                || !(t_close_total > 0.0) || !(v_ball >= 0.0);
  if (input_invalid) *input_invalid = bad;
  if (bad) return {1.0, 0.0};                         // 실현 불가로 보고 (feasible() == false)
  const double vb = std::max(v_ball, 1e-6);
  const double g_min = std::clamp(1.0 - d_eff / (vb * std::max(t_close_total, 1e-6)), 0.0, 1.0);
  const double g_max = std::clamp(std::min(v_dir_max, v_tcp_max) / vb, 0.0, 1.0);
  return {g_min, g_max};
}

// 창이 비는 임계 공 속력: |v| > v_reach + d_eff / T_close_tot 이면 어떤 gamma 도 불가.
// 경계(|v| == 상한)에서 gammaWindow().feasible() 과 1 ulp 로 엇갈릴 수 있으므로,
// 후보 게이트에는 둘 중 하나만 쓰고 여유(planner.gamma.margin)를 둔다.
[[nodiscard]] inline double maxCatchableSpeed(double v_dir_max, double v_tcp_max,
                                              double d_eff, double t_close_total) noexcept {
  return std::min(v_dir_max, v_tcp_max) + d_eff / std::max(t_close_total, 1e-6);
}

}  // namespace catching::plan
