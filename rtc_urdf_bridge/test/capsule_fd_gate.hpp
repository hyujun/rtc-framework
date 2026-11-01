// ── capsule_fd_gate.hpp — ∇_q d 의 중앙차분 oracle (test-only, 설치하지 않는다) ─────
//
// CapsuleWorkspace 의 기울기를 자코비안을 전혀 쓰지 않는 oracle — 거리의 중앙차분 — 로
// 대조한다. rtc_urdf_bridge 의 합성 모델과 integrated_bringup 의 실제 로봇 모델이 같은
// 게이트를 돌리므로 한 곳에 둔다 (test_urdf_path.hpp 와 같은 공유 방식: 상대 include).
//
// 제외 (세어 돌려준다): |d| < exclude (반지름 경계 — 미분 불연속), 두 축이 교차하는
// 자세 (축 사이 거리 < exclude — 최근접점 비유일). 침범 구간은 d ∈ [−(r1+r2)+exclude,
// −exclude]. 판정: ‖∇_FD − ∇‖∞ ≤ abs + rel·‖∇‖∞ (#642 Done when).
#pragma once

#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

namespace rtc::test {

struct CapsuleFdOptions {
  int samples{200};
  double step{1e-6};
  double abs_tol{1e-7};
  double rel_tol{1e-5};
  double exclude{1e-3};
  bool penetrating{false};  ///< 침범 구간만 (아니면 분리 구간만)
  unsigned seed{1};
  Eigen::VectorXd q_lo, q_hi;  ///< 표본 상자 (nq); 비어 있으면 [−1.2, 1.2]
  // 공 (with_ball 인 workspace): world 끝점과 반지름.
  bool with_ball{false};
  Eigen::Vector3d ball_p0{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ball_p1{Eigen::Vector3d::Zero()};
  double ball_radius{0.0};
};

struct CapsuleFdStats {
  int separated{0};
  int penetrating{0};
  int excluded_near_zero{0};
  int excluded_axis_cross{0};
  int excluded_out_of_range{0};
  double worst_ratio{0.0};
};

// 선분–선분 최소 거리 (Ericson 5.1.9) — coal 과 독립인 구현.
inline double SegmentDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                              const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);
  constexpr double kEps = 1e-30;
  double s = 0.0;
  double t = 0.0;
  if (a <= kEps && e <= kEps) {
    return (p1 - p2).norm();
  }
  if (a <= kEps) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEps) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      s = denom != 0.0 ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return ((p1 + d1 * s) - (p2 + d2 * t)).norm();
}

// spec capsule i 의 world 끝점 (BODY frame 경유 — workspace 와 독립인 경로).
inline void CapsuleWorldEndpoints(const pinocchio::Model& model, const pinocchio::Data& data,
                                  const rtc_urdf_bridge::CapsuleModel& spec, std::size_t i,
                                  Eigen::Vector3d& p0, Eigen::Vector3d& p1) {
  const rtc_urdf_bridge::CapsuleSpec& c = spec.Spec().capsules[i];
  const pinocchio::SE3& oMf = data.oMf[model.getFrameId(c.link, pinocchio::BODY)];
  p0 = oMf.act(c.p0);
  p1 = oMf.act(c.p1);
}

// 쌍 p 의 두 형상의 world 끝점과 반지름.
inline void PairWorldEndpoints(const pinocchio::Model& model, const pinocchio::Data& data,
                               const rtc_urdf_bridge::CapsuleModel& spec,
                               const rtc_urdf_bridge::CapsuleWorkspace& ws,
                               const CapsuleFdOptions& opt, std::size_t p, Eigen::Vector3d& a0,
                               Eigen::Vector3d& a1, Eigen::Vector3d& b0, Eigen::Vector3d& b1,
                               double& ra, double& rb) {
  const rtc_urdf_bridge::CapsulePairInfo& info = ws.Pair(p);
  if (info.is_ball) {
    a0 = opt.ball_p0;
    a1 = opt.ball_p1;
    ra = opt.ball_radius;
  } else {
    CapsuleWorldEndpoints(model, data, spec, info.capsule_a, a0, a1);
    ra = spec.Spec().capsules[info.capsule_a].radius;
  }
  CapsuleWorldEndpoints(model, data, spec, info.capsule_b, b0, b1);
  rb = spec.Spec().capsules[info.capsule_b].radius;
}

// FK (oMi · oMf) → 배치 → 공 → 거리. 기울기 없이.
inline void CapsuleDistancesAt(const pinocchio::Model& model, pinocchio::Data& data,
                               rtc_urdf_bridge::CapsuleWorkspace& ws, const CapsuleFdOptions& opt,
                               const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  ASSERT_TRUE(ws.UpdatePlacements(model, data));
  if (opt.with_ball) {
    ws.SetBall(opt.ball_p0, opt.ball_p1, opt.ball_radius);
  }
  EXPECT_EQ(ws.EvaluateDistances(), 0U);
}

// 모든 쌍 · 모든 표본의 FD 대조 (헤더 주석의 게이트).
inline CapsuleFdStats CheckCapsuleGradientsAgainstFd(const pinocchio::Model& model,
                                                     pinocchio::Data& data,
                                                     const rtc_urdf_bridge::CapsuleModel& spec,
                                                     rtc_urdf_bridge::CapsuleWorkspace& ws,
                                                     const CapsuleFdOptions& opt) {
  CapsuleFdStats st;
  const Eigen::Index nv = model.nv;
  EXPECT_EQ(model.nq, nv) << "이 게이트는 nq == nv 인 모델 (revolute · prismatic) 의 것";
  std::mt19937 rng(opt.seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const Eigen::VectorXd lo = opt.q_lo.size() == nv ? opt.q_lo : Eigen::VectorXd::Constant(nv, -1.2);
  const Eigen::VectorXd hi = opt.q_hi.size() == nv ? opt.q_hi : Eigen::VectorXd::Constant(nv, 1.2);
  const std::size_t np = ws.NumPairs();
  std::vector<Eigen::VectorXd> fd(np, Eigen::VectorXd::Zero(nv));
  std::vector<double> d_plus(np);
  std::vector<double> d_minus(np);
  for (int s = 0; s < opt.samples; ++s) {
    Eigen::VectorXd q(nv);
    for (Eigen::Index i = 0; i < nv; ++i) {
      q[i] = lo[i] + (hi[i] - lo[i]) * u(rng);
    }
    pinocchio::computeJointJacobians(model, data, q);  // FK + data.J
    pinocchio::updateFramePlacements(model, data);
    if (!ws.UpdatePlacements(model, data)) {
      ADD_FAILURE() << "지문 불일치";
      return st;
    }
    if (opt.with_ball) {
      ws.SetBall(opt.ball_p0, opt.ball_p1, opt.ball_radius);
    }
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
    if (!ws.EvaluateGradients(model, data)) {
      ADD_FAILURE() << "EvaluateGradients 가 거부했다";
      return st;
    }
    std::vector<double> d(np);
    std::vector<Eigen::VectorXd> g(np);
    std::vector<bool> use(np, true);
    for (std::size_t p = 0; p < np; ++p) {
      d[p] = ws.Distance(p);
      g[p] = ws.Gradient(p);
      Eigen::Vector3d a0;
      Eigen::Vector3d a1;
      Eigen::Vector3d b0;
      Eigen::Vector3d b1;
      double ra = 0.0;
      double rb = 0.0;
      PairWorldEndpoints(model, data, spec, ws, opt, p, a0, a1, b0, b1, ra, rb);
      const double axis = SegmentDistance(a0, a1, b0, b1);
      if (std::abs(d[p]) < opt.exclude) {
        use[p] = false;
        ++st.excluded_near_zero;
      } else if (axis < opt.exclude) {
        use[p] = false;
        ++st.excluded_axis_cross;
      } else if (opt.penetrating) {
        if (d[p] > 0.0 || d[p] < -(ra + rb) + opt.exclude) {
          use[p] = false;
          ++st.excluded_out_of_range;
        }
      } else if (d[p] < 0.0) {
        use[p] = false;
        ++st.excluded_out_of_range;
      }
    }
    for (Eigen::Index j = 0; j < nv; ++j) {
      Eigen::VectorXd qp = q;
      qp[j] += opt.step;
      CapsuleDistancesAt(model, data, ws, opt, qp);
      for (std::size_t p = 0; p < np; ++p) {
        d_plus[p] = ws.Distance(p);
      }
      qp[j] = q[j] - opt.step;
      CapsuleDistancesAt(model, data, ws, opt, qp);
      for (std::size_t p = 0; p < np; ++p) {
        d_minus[p] = ws.Distance(p);
      }
      for (std::size_t p = 0; p < np; ++p) {
        fd[p][j] = (d_plus[p] - d_minus[p]) / (2.0 * opt.step);
      }
    }
    for (std::size_t p = 0; p < np; ++p) {
      if (!use[p]) {
        continue;
      }
      (d[p] > 0.0 ? st.separated : st.penetrating) += 1;
      const double err = (fd[p] - g[p]).cwiseAbs().maxCoeff();
      const double tol = opt.abs_tol + opt.rel_tol * g[p].cwiseAbs().maxCoeff();
      st.worst_ratio = std::max(st.worst_ratio, err / tol);
      EXPECT_LE(err, tol) << "sample " << s << " pair " << p << " d=" << d[p]
                          << "\n  fd=" << fd[p].transpose() << "\n  an=" << g[p].transpose();
    }
  }
  return st;
}

inline void PrintCapsuleFdStats(const char* what, const CapsuleFdStats& st) {
  std::cout << "[FD " << what << "] separated=" << st.separated << " penetrating=" << st.penetrating
            << " excluded(|d|<tol)=" << st.excluded_near_zero
            << " excluded(axis cross)=" << st.excluded_axis_cross
            << " excluded(out of range)=" << st.excluded_out_of_range
            << " worst err/tol=" << st.worst_ratio << "\n";
}

}  // namespace rtc::test
