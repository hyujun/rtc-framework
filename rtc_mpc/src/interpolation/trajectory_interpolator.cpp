#include "rtc_mpc/interpolation/trajectory_interpolator.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>

namespace rtc::mpc {

namespace {

/// Cubic Hermite basis at α ∈ [0, 1] and its first two derivatives in α.
struct HermiteBasis {
  double h00, h10, h01, h11;
  double h00p, h10p, h01p, h11p;
  double h00pp, h10pp, h01pp, h11pp;

  explicit HermiteBasis(double a) noexcept {
    const double a2 = a * a;
    const double a3 = a2 * a;
    h00 = 2.0 * a3 - 3.0 * a2 + 1.0;
    h10 = a3 - 2.0 * a2 + a;
    h01 = -2.0 * a3 + 3.0 * a2;
    h11 = a3 - a2;
    h00p = 6.0 * a2 - 6.0 * a;
    h10p = 3.0 * a2 - 4.0 * a + 1.0;
    h01p = -6.0 * a2 + 6.0 * a;
    h11p = 3.0 * a2 - 2.0 * a;
    h00pp = 12.0 * a - 6.0;
    h10pp = 6.0 * a - 4.0;
    h01pp = -12.0 * a + 6.0;
    h11pp = 6.0 * a - 2.0;
  }
};

/// One Hermite row: positions `x0 → x1` with slopes `s0`, `s1` (per second)
/// over `dt`. Grouped so that α = 0 returns `x0` and `s0` exactly
/// (`h00 = h10p = 1`, every other weight 0 there).
struct HermiteRow {
  double x, xd, xdd;
};

HermiteRow EvalHermite(const HermiteBasis& b, double x0, double x1, double s0, double s1, double dt,
                       double inv_dt, double inv_dt2) noexcept {
  HermiteRow r{};
  r.x = (b.h00 * x0 + b.h01 * x1) + dt * (b.h10 * s0 + b.h11 * s1);
  r.xd = (b.h00p * x0 + b.h01p * x1) * inv_dt + (b.h10p * s0 + b.h11p * s1);
  r.xdd = (b.h00pp * x0 + b.h01pp * x1) * inv_dt2 + (b.h10pp * s0 + b.h11pp * s1) * inv_dt;
  return r;
}

/// The node spacing in integer nanoseconds, or 0 when it is not a positive,
/// finite number of them.
int64_t NodeSpacingNs(double dt_node) noexcept {
  if (!std::isfinite(dt_node)) {
    return 0;
  }
  // Bounded so that (horizon + 1) node spacings fit an int64 for any horizon.
  const double ns = std::round(dt_node * 1e9);
  return (ns >= 1.0 && ns <= 1e16) ? static_cast<int64_t>(ns) : 0;
}

/// A node's free-flyer orientation: pinocchio stores the quaternion xyzw,
/// Eigen's constructor takes wxyz.
Eigen::Quaterniond NodeOrientation(const std::array<double, kMaxNq>& q) noexcept {
  constexpr auto kX = static_cast<std::size_t>(kFreeFlyerQuatOffset);
  return Eigen::Quaterniond(q[kX + 3], q[kX], q[kX + 1], q[kX + 2]);
}

/// A node's base position slope in world: its LOCAL linear velocity rotated
/// by its (normalised) orientation.
Eigen::Vector3d WorldBaseSlope(const Eigen::Quaterniond& r,
                               const std::array<double, kMaxNv>& v) noexcept {
  return r.normalized().toRotationMatrix() * Eigen::Vector3d(v[0], v[1], v[2]);
}

}  // namespace

void TrajectoryInterpolator::Init(int nq, int nv, int n_contact_vars) {
  // Bounds check against the compile-time capacity of MPCSolution. These
  // are capacity limits, not per-robot limits, so they are runtime-invariant.
  if (nq < 0 || nq > kMaxNq || nv < 0 || nv > kMaxNv || n_contact_vars < 0 ||
      n_contact_vars > kMaxContactVars) {
    nq_ = 0;
    nv_ = 0;
    n_contact_vars_ = 0;
    has_solution_ = false;
    plain_layout_ = WbLayout{};
    return;
  }
  nq_ = nq;
  nv_ = nv;
  n_contact_vars_ = n_contact_vars;
  has_solution_ = false;
  plain_layout_ = WbLayout::FixedBase(nq, nv, n_contact_vars);
  if (!sol_) {
    sol_ = std::make_unique<MPCSolution>();
  }
}

bool TrajectoryInterpolator::SetSolution(const MPCSolution& sol, uint64_t receive_ns) noexcept {
  return SetSolutionWithOrigin(sol, static_cast<int64_t>(receive_ns));
}

bool TrajectoryInterpolator::SetSolutionWithOrigin(const MPCSolution& sol,
                                                   int64_t origin_ns) noexcept {
  // A plan that is not consumable does not evict the one being sampled;
  // the caller sees `false` and keeps ageing the current plan.
  if (!sol.IsValid() || !sol_) {
    return false;
  }
  *sol_ = sol;  // trivially copyable; compiler lowers to memcpy
  origin_ns_ = origin_ns;
  has_solution_ = true;
  return true;
}

SampleSpot TrajectoryInterpolator::Sample(int64_t now_ns, const WbLayout& layout, double* q,
                                          double* v, double* a, double* wrench,
                                          uint32_t& mask) const noexcept {
  SampleSpot spot{};
  // A refused Init leaves no dimensions: nothing to sample into.
  if (!has_solution_ || nq_ <= 0 || nv_ <= 0 || q == nullptr || v == nullptr || a == nullptr) {
    return spot;
  }
  const MPCSolution& s = *sol_;
  const int horizon = s.horizon_length;
  const int64_t dt_ns = NodeSpacingNs(s.dt_node);
  if (horizon <= 0 || horizon > kMaxHorizon || dt_ns == 0) {
    return spot;
  }
  const int nq = std::min(nq_, s.nq);
  const int nv = std::min(nv_, s.nv);
  const int nc = std::min(n_contact_vars_, s.n_contact_vars);
  const bool floating = layout.floating_base();
  if (floating && (layout.n_base_q != kFreeFlyerNq || layout.n_base_v != kFreeFlyerNv ||
                   nq < kFreeFlyerNq || nv < kFreeFlyerNv)) {
    return spot;
  }
  const int nbq = floating ? kFreeFlyerNq : 0;
  const int nbv = floating ? kFreeFlyerNv : 0;
  // A floating base pairs joint configuration and tangent entries one to one.
  if (floating && nq - nbq != nv - nbv) {
    return spot;
  }

  const int64_t tau = std::max<int64_t>(0, now_ns - origin_ns_);
  const int64_t horizon_ns = static_cast<int64_t>(horizon) * dt_ns;
  spot.tau_ns = tau;
  spot.dt_ns = dt_ns;

  // The wrench and the mode hold over an interval; the arrays have H rows.
  const auto hold_wrench = [&](int k) {
    const auto kk = static_cast<std::size_t>(k);
    mask = s.node_mode[kk];
    if (wrench == nullptr) {
      return;
    }
    for (int i = 0; i < nc; ++i) {
      wrench[i] = s.lambda_traj[kk][static_cast<std::size_t>(i)];
    }
    // Slots of contacts inactive at this node carry whatever the cost left.
    for (int c = 0; c < layout.n_contacts; ++c) {
      if (((mask >> c) & 1U) != 0U) {
        continue;
      }
      const int off = layout.contact_u_offset[static_cast<std::size_t>(c)];
      const int dim = layout.contact_dim[static_cast<std::size_t>(c)];
      for (int i = off; i < off + dim && i < nc; ++i) {
        wrench[i] = 0.0;
      }
    }
  };

  if (tau >= horizon_ns) {
    const auto last = static_cast<std::size_t>(horizon);
    for (int i = 0; i < nq; ++i) {
      q[i] = s.q_traj[last][static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < nv; ++i) {
      v[i] = s.v_traj[last][static_cast<std::size_t>(i)];
      a[i] = 0.0;
    }
    hold_wrench(horizon - 1);
    spot.valid = true;
    spot.beyond_horizon = true;
    spot.progress = 1.0;
    spot.node = horizon;
    return spot;
  }

  const int k = static_cast<int>(tau / dt_ns);
  const int64_t rem = tau - static_cast<int64_t>(k) * dt_ns;
  const double alpha = static_cast<double>(rem) / static_cast<double>(dt_ns);
  const HermiteBasis b(alpha);
  const double dt = static_cast<double>(dt_ns) * 1e-9;  // the spacing α is measured in
  const double inv_dt = 1.0 / dt;
  const double inv_dt2 = inv_dt * inv_dt;
  const auto kk = static_cast<std::size_t>(k);
  const auto kk1 = static_cast<std::size_t>(k + 1);
  const auto& q0 = s.q_traj[kk];
  const auto& q1 = s.q_traj[kk1];
  const auto& v0 = s.v_traj[kk];
  const auto& v1 = s.v_traj[kk1];

  // Joint rows: configuration entry nbq + i pairs with tangent entry nbv + i.
  const int nj_q = nq - nbq;
  const int nj_v = nv - nbv;
  const int n_coupled = std::min(nj_q, nj_v);
  for (int i = 0; i < n_coupled; ++i) {
    const auto iq = static_cast<std::size_t>(nbq + i);
    const auto iv = static_cast<std::size_t>(nbv + i);
    const HermiteRow r = EvalHermite(b, q0[iq], q1[iq], v0[iv], v1[iv], dt, inv_dt, inv_dt2);
    q[nbq + i] = r.x;
    v[nbv + i] = r.xd;
    a[nbv + i] = r.xdd;
  }
  // Fixed base only (a floating one was refused above): a configuration
  // entry without a velocity gets a linear position, a velocity entry
  // without a configuration entry zero.
  for (int i = n_coupled; i < nj_q; ++i) {
    const auto iq = static_cast<std::size_t>(nbq + i);
    q[nbq + i] = (1.0 - alpha) * q0[iq] + alpha * q1[iq];
  }
  for (int i = n_coupled; i < nj_v; ++i) {
    v[nbv + i] = 0.0;
    a[nbv + i] = 0.0;
  }

  if (floating) {
    constexpr int kQ = kFreeFlyerQuatOffset;
    constexpr auto kX = static_cast<std::size_t>(kQ);
    const Eigen::Quaterniond r0 = NodeOrientation(q0);
    const Eigen::Quaterniond r1 = NodeOrientation(q1);
    if (rem == 0) {
      // The node itself: no slerp, no renormalisation.
      for (int j = 0; j < kFreeFlyerQuatSize; ++j) {
        q[kQ + j] = q0[kX + static_cast<std::size_t>(j)];
      }
    } else {
      Eigen::Quaterniond r = r0.slerp(alpha, r1);
      r.normalize();
      q[kQ] = r.x();
      q[kQ + 1] = r.y();
      q[kQ + 2] = r.z();
      q[kQ + 3] = r.w();
    }
    // World-frame slopes of the base position: R · v_lin of each node.
    const Eigen::Vector3d pd0 = WorldBaseSlope(r0, v0);
    const Eigen::Vector3d pd1 = WorldBaseSlope(r1, v1);
    for (int j = 0; j < 3; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      q[j] = EvalHermite(b, q0[jj], q1[jj], pd0[j], pd1[j], dt, inv_dt, inv_dt2).x;
    }
    // LOCAL twist: linear; acceleration: the interval mean the plan ships.
    for (int j = 0; j < kFreeFlyerNv; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      v[j] = (1.0 - alpha) * v0[jj] + alpha * v1[jj];
      a[j] = s.a_ff_traj[kk][jj];
    }
  }

  hold_wrench(k);
  spot.valid = true;
  spot.progress = static_cast<double>(tau) / static_cast<double>(horizon_ns);
  spot.node = k;
  return spot;
}

void TrajectoryInterpolator::Interpolate(uint64_t now_ns, Eigen::Ref<Eigen::VectorXd> q_ref,
                                         Eigen::Ref<Eigen::VectorXd> v_ref,
                                         Eigen::Ref<Eigen::VectorXd> a_ff,
                                         Eigen::Ref<Eigen::VectorXd> lambda_ref,
                                         InterpMeta& meta_out) noexcept {
  meta_out.valid = false;
  meta_out.beyond_horizon = false;
  meta_out.progress = 0.0;
  if (!has_solution_ || q_ref.size() < std::min(nq_, sol_->nq) ||
      v_ref.size() < std::min(nv_, sol_->nv) || a_ff.size() < std::min(nv_, sol_->nv) ||
      lambda_ref.size() < std::min(n_contact_vars_, sol_->n_contact_vars)) {
    return;
  }
  uint32_t mask = 0;
  const SampleSpot spot = Sample(static_cast<int64_t>(now_ns), plain_layout_, q_ref.data(),
                                 v_ref.data(), a_ff.data(), lambda_ref.data(), mask);
  meta_out.valid = spot.valid;
  meta_out.beyond_horizon = spot.beyond_horizon;
  meta_out.progress = spot.progress;
}

double TrajectoryInterpolator::RemainingHorizonSec(uint64_t now_ns) const noexcept {
  if (!has_solution_ || sol_->dt_node <= 0.0 || sol_->horizon_length <= 0) {
    return 0.0;
  }
  const int64_t delta_ns = static_cast<int64_t>(now_ns) - origin_ns_;
  const double t_local = std::max<double>(0.0, static_cast<double>(delta_ns) * 1e-9);
  const double horizon_sec = static_cast<double>(sol_->horizon_length) * sol_->dt_node;
  const double remaining = horizon_sec - t_local;
  return remaining > 0.0 ? remaining : 0.0;
}

}  // namespace rtc::mpc
