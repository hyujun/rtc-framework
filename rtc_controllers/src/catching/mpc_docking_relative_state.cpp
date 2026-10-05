#include "rtc_controllers/catching/mpc_docking_relative_state.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rtc::catching {

namespace {

// Bisection steps of DockingTimingSigmaMax: the bracket halves in log space, so
// 200 steps exhaust a double's exponent range many times over.
constexpr int kSigmaMaxBisections = 200;

[[nodiscard]] double NormalPdf(double x) noexcept {
  return std::exp(-0.5 * x * x) / std::sqrt(2.0 * std::numbers::pi);
}

// √(max(V, 0) + ε²). V is a quadratic form of a covariance the caller checked
// finite and PSD to rounding; the max only absorbs that rounding.
[[nodiscard]] double RegularizedSigma(double variance, double eps_sigma) noexcept {
  return std::sqrt(std::max(variance, 0.0) + eps_sigma * eps_sigma);
}

}  // namespace

// ── Normal distribution ──────────────────────────────────────────────────────

double NormalCdf(double x) noexcept {
  return 0.5 * std::erfc(-x / std::numbers::sqrt2);
}

bool NormalQuantile(double p, double& z) noexcept {
  if (!std::isfinite(p) || !(p > 0.0) || !(p < 1.0)) {
    return false;
  }
  // Acklam's rational approximation (relative error < 1.2e-9), then one Halley
  // step on Φ(x) − p, which takes it to rounding.
  constexpr double a[] = {-3.969683028665376e+01, 2.209460984245205e+02,  -2.759285104469687e+02,
                          1.383577518672690e+02,  -3.066479806614716e+01, 2.506628277459239e+00};
  constexpr double b[] = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
                          6.680131188771972e+01, -1.328068155288572e+01};
  constexpr double c[] = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
                          -2.549732539343734e+00, 4.374664141464968e+00,  2.938163982698783e+00};
  constexpr double d[] = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
                          3.754408661907416e+00};
  constexpr double p_low = 0.02425;
  double x = 0.0;
  if (p < p_low) {
    const double q = std::sqrt(-2.0 * std::log(p));
    x = (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
        ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  } else if (p <= 1.0 - p_low) {
    const double q = p - 0.5;
    const double r = q * q;
    x = (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
        (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
  } else {
    const double q = std::sqrt(-2.0 * std::log(1.0 - p));
    x = -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
        ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  }
  const double pdf = NormalPdf(x);
  if (pdf > 0.0) {
    const double e = NormalCdf(x) - p;
    const double u = e / pdf;
    x -= u / (1.0 + 0.5 * x * u);
  }
  if (!std::isfinite(x)) {
    return false;
  }
  z = x;
  return true;
}

bool DockingTimingSigmaMax(double delta_lo, double delta_hi, double delta_0, double eps_t,
                           double& sigma_max) noexcept {
  if (!std::isfinite(delta_lo) || !std::isfinite(delta_hi) || !std::isfinite(delta_0) ||
      !std::isfinite(eps_t) || !(delta_lo < delta_0) || !(delta_0 < delta_hi) || !(eps_t > 0.0) ||
      !(eps_t < 1.0)) {
    return false;
  }
  const double hi_gap = delta_hi - delta_0;
  const double lo_gap = delta_lo - delta_0;  // < 0
  const double target = 1.0 - eps_t;
  const auto prob = [hi_gap, lo_gap](double sigma) noexcept {
    return NormalCdf(hi_gap / sigma) - NormalCdf(lo_gap / sigma);
  };
  // P(σ) falls from 1 (σ → 0) to 0 (σ → ∞). Bracket around the window width.
  const double width = delta_hi - delta_lo;
  double lo = width * 1e-6;
  double hi = width * 1e6;
  if (!(prob(lo) >= target) || !(prob(hi) <= target)) {
    return false;
  }
  for (int i = 0; i < kSigmaMaxBisections; ++i) {
    const double mid = std::sqrt(lo * hi);
    if (prob(mid) >= target) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  sigma_max = lo;  // the side that satisfies the probability
  return std::isfinite(sigma_max) && sigma_max > 0.0;
}

// ── Work-struct sizing (non-RT) ──────────────────────────────────────────────

void DockingFrameKinematics::Resize(Eigen::Index n) {
  j_p.setZero(3, n);
  j_w.setZero(3, n);
  dv_dq.setZero(3, n);
  dw_dq.setZero(3, n);
  j6.setZero(6, n);
  d6_dq.setZero(6, n);
  d6_dv.setZero(6, n);
  pv_dv.setZero(3, n);
}

void DockingRelativeState::Resize(Eigen::Index n) {
  dr_dq.setZero(3, n);
  dnu_dq.setZero(3, n);
}

void DockingScalar::Resize(Eigen::Index n) {
  value = 0.0;
  dq.setZero(n);
  dv.setZero(n);
}

void DockingImpact::Resize(Eigen::Index n) {
  beta.Resize(n);
  g_n.Resize(n);
  energy.Resize(n);
  impulse.Resize(n);
  root_energy.Resize(n);
}

void DockingImpactWork::Init(const pinocchio::Model& model) {
  const Eigen::Index n = model.nv;
  data = pinocchio::Data(model);
  kin_y.Resize(n);
  mass.setZero(n, n);
  llt = Eigen::LLT<Eigen::MatrixXd>(n);
  f.setZero(n);
  y.setZero(n);
  zero.setZero(n);
  dtau_dq.setZero(n, n);
  dtau_dv.setZero(n, n);
  dtau_da.setZero(n, n);
  dg_dq.setZero(n, n);
  j_c.setZero(3, n);
  dvc_dq.setZero(3, n);
}

void DockingManipulabilityWork::Init(const pinocchio::Model& model) {
  const Eigen::Index n = model.nv;
  data = pinocchio::Data(model);
  hessian = pinocchio::Data::Tensor3x(6, n, n);
  hessian.setZero();
  j6.setZero(6, n);
  j_bar.setZero(6, n);
  b.setZero(6, n);
}

// ── Kinematics ───────────────────────────────────────────────────────────────

bool ComputeDockingFrameKinematics(const pinocchio::Model& model, pinocchio::Data& data,
                                   pinocchio::FrameIndex frame,
                                   const Eigen::Ref<const Eigen::VectorXd>& q,
                                   const Eigen::Ref<const Eigen::VectorXd>& v,
                                   DockingFrameKinematics& out) noexcept {
  const Eigen::Index n = model.nv;
  if (model.nq != model.nv || frame >= model.frames.size() || q.size() != n || v.size() != n ||
      out.j_p.cols() != n || out.j_w.cols() != n || out.dv_dq.cols() != n ||
      out.dw_dq.cols() != n || out.d6_dq.cols() != n || out.d6_dv.cols() != n ||
      out.pv_dv.cols() != n) {
    return false;
  }
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v);
  const pinocchio::Frame& f = model.frames[frame];
  out.d6_dq.setZero();
  out.d6_dv.setZero();
  pinocchio::getFrameVelocityDerivatives(model, data, f.parentJoint, f.placement,
                                         pinocchio::LOCAL_WORLD_ALIGNED, out.d6_dq, out.d6_dv);
  out.dv_dq.setZero();
  out.pv_dv.setZero();
  pinocchio::getPointVelocityDerivatives(model, data, f.parentJoint, f.placement,
                                         pinocchio::LOCAL_WORLD_ALIGNED, out.dv_dq, out.pv_dv);
  // ∂(frame velocity)/∂q̇ IS the frame Jacobian.
  out.j_p = out.d6_dv.topRows<3>();
  out.j_w = out.d6_dv.bottomRows<3>();
  out.dw_dq = out.d6_dq.bottomRows<3>();
  const pinocchio::SE3 placement = data.oMi[f.parentJoint] * f.placement;
  out.p = placement.translation();
  out.R = placement.rotation();
  out.v.noalias() = out.j_p * v;
  out.w.noalias() = out.j_w * v;
  return true;
}

bool ComputeDockingRelativeState(const DockingFrameKinematics& kin, const Eigen::Vector3d& p_b,
                                 const Eigen::Vector3d& v_b, DockingRelativeState& out) noexcept {
  const Eigen::Index n = kin.j_p.cols();
  if (out.dr_dq.cols() != n || out.dnu_dq.cols() != n || kin.j_w.cols() != n ||
      kin.dv_dq.cols() != n || kin.dw_dq.cols() != n) {
    return false;
  }
  const Eigen::Matrix3d rt = kin.R.transpose();
  out.r_w = p_b - kin.p;
  out.r_h.noalias() = rt * out.r_w;
  const Eigen::Vector3d rel_w = v_b - kin.v - kin.w.cross(out.r_w);
  out.nu_h.noalias() = rt * rel_w;
  for (Eigen::Index j = 0; j < n; ++j) {
    const Eigen::Vector3d jp = kin.j_p.col(j);
    const Eigen::Vector3d jw = kin.j_w.col(j);
    const Eigen::Vector3d jw_h = rt * jw;
    const Eigen::Vector3d dvh = kin.dv_dq.col(j);
    const Eigen::Vector3d dwh = kin.dw_dq.col(j);
    out.dr_dq.col(j) = -(rt * jp) + out.r_h.cross(jw_h);
    const Eigen::Vector3d inner = -dvh + out.r_w.cross(dwh) + kin.w.cross(jp);
    out.dnu_dq.col(j) = out.nu_h.cross(jw_h) + rt * inner;
  }
  out.s = out.r_h.z();
  out.c = -out.nu_h.z();
  return true;
}

bool DockingRelativeAcceleration(const pinocchio::Model& model, pinocchio::Data& data,
                                 pinocchio::FrameIndex frame,
                                 const Eigen::Ref<const Eigen::VectorXd>& q,
                                 const Eigen::Ref<const Eigen::VectorXd>& v,
                                 const Eigen::Ref<const Eigen::VectorXd>& a,
                                 const Eigen::Vector3d& p_b, const Eigen::Vector3d& v_b,
                                 const Eigen::Vector3d& a_b, Eigen::Vector3d& a_rel_h) noexcept {
  const Eigen::Index n = model.nv;
  if (model.nq != model.nv || frame >= model.frames.size() || q.size() != n || v.size() != n ||
      a.size() != n) {
    return false;
  }
  pinocchio::forwardKinematics(model, data, q, v, a);
  const pinocchio::Frame& f = model.frames[frame];
  const pinocchio::SE3 placement = data.oMi[f.parentJoint] * f.placement;
  const pinocchio::Motion vel = pinocchio::getFrameVelocity(model, data, f.parentJoint, f.placement,
                                                            pinocchio::LOCAL_WORLD_ALIGNED);
  const pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
      model, data, f.parentJoint, f.placement, pinocchio::LOCAL_WORLD_ALIGNED);
  const Eigen::Vector3d w = vel.angular();
  const Eigen::Vector3d r_w = p_b - placement.translation();
  const Eigen::Vector3d rel_v = v_b - vel.linear();
  // ν^H = Rᵀ b, b = v_b − v_h − ω × r^W  ⇒  ν̇^H = Rᵀ (ḃ − ω × b).
  const Eigen::Vector3d b = rel_v - w.cross(r_w);
  const Eigen::Vector3d b_dot = a_b - acc.linear() - acc.angular().cross(r_w) - w.cross(rel_v);
  a_rel_h.noalias() = placement.rotation().transpose() * (b_dot - w.cross(b));
  return true;
}

// ── Approach rows ────────────────────────────────────────────────────────────

void DockingCorridorRow(const DockingRelativeState& rel, double s_ent, double r_ent,
                        double tan_theta, double s_c, DockingScalar& g, double& dg_ds_c) noexcept {
  const double gap = rel.s - s_ent;
  const bool open = gap >= 0.0;
  const double width = r_ent + (open ? gap : 0.0) * tan_theta + s_c;
  const double rho_x = rel.r_h.x();
  const double rho_y = rel.r_h.y();
  g.value = rho_x * rho_x + rho_y * rho_y - width * width;
  g.dq =
      (2.0 * rho_x) * rel.dr_dq.row(0).transpose() + (2.0 * rho_y) * rel.dr_dq.row(1).transpose();
  if (open) {
    g.dq -= (2.0 * width * tan_theta) * rel.dr_dq.row(2).transpose();
  }
  g.dv.setZero();
  dg_ds_c = -2.0 * width;
}

double DockingCorridorMinSlack(const DockingRelativeState& rel, double s_ent, double r_ent,
                               double tan_theta) noexcept {
  const double gap = rel.s - s_ent;
  const double width = r_ent + (gap >= 0.0 ? gap : 0.0) * tan_theta;
  const double need = rel.r_h.head<2>().norm() - width;
  return need > 0.0 ? need : 0.0;
}

void DockingEnvelopeRow(const DockingRelativeState& rel, double s_ent, double c_ent_max,
                        double a_brake, double s_v, DockingScalar& g) noexcept {
  const double gap = rel.s - s_ent;
  g.value = rel.c * rel.c - c_ent_max * c_ent_max - 2.0 * a_brake * gap - s_v;
  // c = −e₃ᵀ ν^H.
  g.dq = (-2.0 * rel.c) * rel.dnu_dq.row(2).transpose() -
         (2.0 * a_brake) * rel.dr_dq.row(2).transpose();
  g.dv = (-2.0 * rel.c) * rel.dr_dq.row(2).transpose();
}

double DockingEnvelopeMinSlack(const DockingRelativeState& rel, double s_ent, double c_ent_max,
                               double a_brake) noexcept {
  const double need = rel.c * rel.c - c_ent_max * c_ent_max - 2.0 * a_brake * (rel.s - s_ent);
  return need > 0.0 ? need : 0.0;
}

// ── Crossing plane ───────────────────────────────────────────────────────────

void ComputeDockingCrossing(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                            const Eigen::Matrix3d& sigma_p, double c_min, double eps_sigma,
                            DockingCrossing& out) noexcept {
  out.c_guarded = !(rel.c > c_min);
  out.c_tilde = out.c_guarded ? c_min : rel.c;
  // Π = I + ν e₃ᵀ / c̃.
  Eigen::Matrix3d pi = Eigen::Matrix3d::Identity();
  pi.col(2) += rel.nu_h / out.c_tilde;
  const Eigen::Matrix3d sigma_r_h = kin.R.transpose() * sigma_p * kin.R;
  const Eigen::Matrix3d crossed = pi * sigma_r_h * pi.transpose();
  out.sigma_rho = crossed.topLeftCorner<2, 2>();
  const Eigen::Vector3d d = kin.R.col(2);
  out.var_s = d.dot(sigma_p * d);
  out.sigma_s = RegularizedSigma(out.var_s, eps_sigma);
  out.sigma_t = out.sigma_s / out.c_tilde;
}

void DockingLateralChanceRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                             const Eigen::Matrix3d& sigma_p, const Eigen::Vector2d& a, double kappa,
                             double c_min, double eps_sigma, DockingScalar& h) noexcept {
  const bool guarded = !(rel.c > c_min);
  const double c_tilde = guarded ? c_min : rel.c;
  // g = Πᵀ E⊥ a = b + α e₃ with b = E⊥ a and α = νᵀ b / c̃.
  const Eigen::Vector3d b(a.x(), a.y(), 0.0);
  const double alpha = rel.nu_h.dot(b) / c_tilde;
  const Eigen::Vector3d g_hand(b.x(), b.y(), alpha);
  const Eigen::Vector3d w = kin.R * g_hand;
  const Eigen::Vector3d u = sigma_p * w;
  const double sigma = RegularizedSigma(w.dot(u), eps_sigma);
  h.value = a.x() * rel.r_h.x() + a.y() * rel.r_h.y() + kappa * sigma;

  // δα = dα_dν · δν^H; with c̃ = c (guard off) δc̃ = −e₃ᵀ δν adds (α / c̃) e₃ᵀ.
  Eigen::Vector3d dalpha_dnu = b / c_tilde;
  if (!guarded) {
    dalpha_dnu.z() += alpha / c_tilde;
  }
  // δ(variance) = 2 uᵀ δw,  δw = −[w]× J_ω δq + d δα.
  const Eigen::Vector3d d = kin.R.col(2);
  const Eigen::Vector3d wxu = w.cross(u);
  const double ud = u.dot(d);
  const double half_over_sigma = kappa / sigma;  // κ · (1 / 2σ) · 2
  h.dq = a.x() * rel.dr_dq.row(0).transpose() + a.y() * rel.dr_dq.row(1).transpose();
  // The scale goes INTO the 3-vector: noalias() covers a bare product only, a
  // scalar wrapped around it would build a run-time-sized temporary.
  const Eigen::Vector3d rot_term = half_over_sigma * wxu;
  const Eigen::Vector3d alpha_term = (half_over_sigma * ud) * dalpha_dnu;
  h.dq.noalias() += kin.j_w.transpose() * rot_term;
  h.dq.noalias() += rel.dnu_dq.transpose() * alpha_term;
  h.dv.noalias() = rel.dr_dq.transpose() * alpha_term;
}

void DockingTimingRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                      const Eigen::Matrix3d& sigma_p, double k_t, double eps_sigma,
                      DockingScalar& t) noexcept {
  const Eigen::Vector3d d = kin.R.col(2);
  const Eigen::Vector3d u = sigma_p * d;
  const double sigma_s = RegularizedSigma(d.dot(u), eps_sigma);
  t.value = rel.c * k_t - sigma_s;
  // δσ_s = (1/σ_s) uᵀ δd,  δd = −[d]× J_ω δq  ⇒  ∂σ_s/∂q = (d × u)ᵀ J_ω / σ_s.
  const Eigen::Vector3d dxu = d.cross(u) / sigma_s;
  t.dq = (-k_t) * rel.dnu_dq.row(2).transpose();
  t.dq.noalias() -= kin.j_w.transpose() * dxu;
  t.dv = (-k_t) * rel.dr_dq.row(2).transpose();
}

void DockingVelocitySigma(const DockingFrameKinematics& kin, const DockingCovariance6& sigma_b,
                          const Eigen::Vector3d& m_hand, double eps_sigma,
                          DockingScalar& sigma) noexcept {
  // mᵀ δν^H = ℓᵀ [δp_b; δv_b],  ℓ = [ω × m_w; m_w],  m_w = R m.
  const Eigen::Vector3d m_w = kin.R * m_hand;
  Eigen::Matrix<double, 6, 1> ell;
  ell.head<3>() = kin.w.cross(m_w);
  ell.tail<3>() = m_w;
  const Eigen::Matrix<double, 6, 1> s_ell = sigma_b * ell;
  const Eigen::Vector3d u_p = s_ell.head<3>();
  const Eigen::Vector3d u_v = s_ell.tail<3>();
  sigma.value = RegularizedSigma(ell.dot(s_ell), eps_sigma);
  // δ(variance) = 2 (m_w × u_p)ᵀ δω + 2 (m_w × y)ᵀ J_ω δq,  y = u_p × ω + u_v.
  const double inv = 1.0 / sigma.value;
  const Eigen::Vector3d mxup = inv * m_w.cross(u_p);
  const Eigen::Vector3d y = u_p.cross(kin.w) + u_v;
  const Eigen::Vector3d mxy = inv * m_w.cross(y);
  sigma.dq.noalias() = kin.dw_dq.transpose() * mxup;
  sigma.dq.noalias() += kin.j_w.transpose() * mxy;
  sigma.dv.noalias() = kin.j_w.transpose() * mxup;
}

void DockingAxialSpeedRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                          const DockingCovariance6& sigma_b, double kappa, double eps_sigma,
                          DockingScalar& row) noexcept {
  DockingVelocitySigma(kin, sigma_b, Eigen::Vector3d::UnitZ(), eps_sigma, row);
  row.value = rel.c + kappa * row.value;
  row.dq *= kappa;
  row.dq -= rel.dnu_dq.row(2).transpose();
  row.dv *= kappa;
  row.dv -= rel.dr_dq.row(2).transpose();
}

void DockingLateralSpeedRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                            const DockingCovariance6& sigma_b, const Eigen::Vector2d& u,
                            double kappa, double eps_sigma, DockingScalar& row) noexcept {
  const Eigen::Vector3d m(u.x(), u.y(), 0.0);
  DockingVelocitySigma(kin, sigma_b, m, eps_sigma, row);
  row.value = u.x() * rel.nu_h.x() + u.y() * rel.nu_h.y() + kappa * row.value;
  row.dq *= kappa;
  row.dq += u.x() * rel.dnu_dq.row(0).transpose() + u.y() * rel.dnu_dq.row(1).transpose();
  row.dv *= kappa;
  row.dv += u.x() * rel.dr_dq.row(0).transpose() + u.y() * rel.dr_dq.row(1).transpose();
}

// ── Impact ───────────────────────────────────────────────────────────────────

bool ComputeDockingImpact(const pinocchio::Model& model, pinocchio::FrameIndex frame,
                          const Eigen::Ref<const Eigen::VectorXd>& q,
                          const Eigen::Ref<const Eigen::VectorXd>& v,
                          const DockingFrameKinematics& kin, const Eigen::Vector3d& v_b,
                          const Eigen::Vector3d& p_c_hand, double m_ball, double restitution,
                          DockingImpactWork& work, DockingImpact& out) noexcept {
  const Eigen::Index n = model.nv;
  if (model.nq != model.nv || frame >= model.frames.size() || q.size() != n || v.size() != n ||
      kin.j_p.cols() != n || work.mass.rows() != n || work.f.size() != n ||
      out.beta.dq.size() != n || out.g_n.dq.size() != n || out.energy.dq.size() != n ||
      out.impulse.dq.size() != n || out.root_energy.dq.size() != n || !(m_ball > 0.0)) {
    return false;
  }
  const Eigen::Vector3d normal = kin.R.col(2);
  const Eigen::Vector3d lever = kin.R * p_c_hand;  // contact point from the frame origin, world

  // Contact Jacobian and the contact point's velocity derivative at (q, v):
  //   v_c = v_h + ω × lever,  δlever = −[lever]× J_ω δq.
  for (Eigen::Index j = 0; j < n; ++j) {
    const Eigen::Vector3d jp = kin.j_p.col(j);
    const Eigen::Vector3d jw = kin.j_w.col(j);
    const Eigen::Vector3d dvh = kin.dv_dq.col(j);
    const Eigen::Vector3d dwh = kin.dw_dq.col(j);
    work.j_c.col(j) = jp - lever.cross(jw);
    work.dvc_dq.col(j) = dvh - lever.cross(dwh) - kin.w.cross(lever.cross(jw));
  }
  const Eigen::Vector3d v_c = kin.v + kin.w.cross(lever);
  const Eigen::Vector3d rel_v = v_b - v_c;
  out.g_n.value = normal.dot(rel_v);
  // δn = −[n]× J_ω δq.
  const Eigen::Vector3d nxrel = normal.cross(rel_v);
  out.g_n.dq.noalias() = kin.j_w.transpose() * nxrel;
  out.g_n.dq.noalias() -= work.dvc_dq.transpose() * normal;
  out.g_n.dv.noalias() = work.j_c.transpose() * normal;
  out.g_n.dv = -out.g_n.dv;

  // β_h = fᵀ M⁻¹ f, f = J_cᵀ n.
  pinocchio::crba(model, work.data, q);
  work.mass = work.data.M.selfadjointView<Eigen::Upper>();
  work.llt.compute(work.mass);
  if (work.llt.info() != Eigen::Success) {
    return false;
  }
  work.f.noalias() = work.j_c.transpose() * normal;
  work.y = work.f;
  work.llt.solveInPlace(work.y);
  const double beta = work.f.dot(work.y);
  // LLT reports success on a NaN pivot; the value is the real check.
  if (!std::isfinite(beta) || !work.y.allFinite()) {
    return false;
  }

  // ∂β/∂q = 2 nᵀ ∂_q(J_c y)|_y + 2 (J_c y)ᵀ ∂_q n − yᵀ ∂_q(M y)|_y.
  if (!ComputeDockingFrameKinematics(model, work.data, frame, q, work.y, work.kin_y)) {
    return false;
  }
  const Eigen::Vector3d vcy = work.kin_y.v + work.kin_y.w.cross(lever);  // J_c y
  const Eigen::Vector3d nxvcy = normal.cross(vcy);
  for (Eigen::Index j = 0; j < n; ++j) {
    const Eigen::Vector3d jw = kin.j_w.col(j);
    const Eigen::Vector3d dvh = work.kin_y.dv_dq.col(j);
    const Eigen::Vector3d dwh = work.kin_y.dw_dq.col(j);
    const Eigen::Vector3d dvcy = dvh - lever.cross(dwh) - work.kin_y.w.cross(lever.cross(jw));
    out.beta.dq[j] = 2.0 * normal.dot(dvcy) + 2.0 * nxvcy.dot(jw);
  }
  work.dtau_dq.setZero();
  work.dtau_dv.setZero();
  work.dtau_da.setZero();
  pinocchio::computeRNEADerivatives(model, work.data, q, work.zero, work.y, work.dtau_dq,
                                    work.dtau_dv, work.dtau_da);
  work.dg_dq.setZero();
  pinocchio::computeGeneralizedGravityDerivatives(model, work.data, q, work.dg_dq);
  work.dtau_dq -= work.dg_dq;  // ∂_q(M y)|_y
  out.beta.dq.noalias() -= work.dtau_dq.transpose() * work.y;
  out.beta.dv.setZero();
  out.beta.value = beta;
  out.beta_h = beta;

  const double m_red = 1.0 / (1.0 / m_ball + beta);
  out.m_red = m_red;
  const double c_n = -out.g_n.value;
  const double dm = -m_red * m_red;  // ∂m_red/∂β
  out.energy.value = 0.5 * m_red * c_n * c_n;
  out.energy.dq = (0.5 * c_n * c_n * dm) * out.beta.dq - (m_red * c_n) * out.g_n.dq;
  out.energy.dv = (-m_red * c_n) * out.g_n.dv;
  const double gain = 1.0 + restitution;
  out.impulse.value = gain * m_red * c_n;
  out.impulse.dq = (gain * c_n * dm) * out.beta.dq - (gain * m_red) * out.g_n.dq;
  out.impulse.dv = (-gain * m_red) * out.g_n.dv;
  const double root = std::sqrt(0.5 * m_red);
  out.root_energy.value = root * c_n;
  out.root_energy.dq = (c_n * dm / (4.0 * root)) * out.beta.dq - root * out.g_n.dq;
  out.root_energy.dv = (-root) * out.g_n.dv;
  return true;
}

// ── Manipulability ───────────────────────────────────────────────────────────

bool ComputeDockingManipulability(const pinocchio::Model& model, pinocchio::FrameIndex frame,
                                  const Eigen::Ref<const Eigen::VectorXd>& q, double d_x_lin,
                                  double d_x_ang, const Eigen::Ref<const Eigen::VectorXd>& d_q,
                                  double delta, DockingManipulabilityWork& work, double& psi,
                                  Eigen::Ref<Eigen::VectorXd> grad) noexcept {
  const Eigen::Index n = model.nv;
  if (model.nq != model.nv || frame >= model.frames.size() || q.size() != n || d_q.size() != n ||
      grad.size() != n || work.j6.cols() != n || work.hessian.dimension(1) != n ||
      !(d_x_lin > 0.0) || !(d_x_ang > 0.0) || !(delta > 0.0)) {
    return false;
  }
  work.j6.setZero();
  pinocchio::computeFrameJacobian(model, work.data, q, frame, pinocchio::LOCAL_WORLD_ALIGNED,
                                  work.j6);
  pinocchio::computeJointKinematicHessians(model, work.data, q);
  const pinocchio::Frame& f = model.frames[frame];
  work.hessian.setZero();
  pinocchio::getFrameKinematicHessian(model, work.data, f.parentJoint, f.placement,
                                      pinocchio::LOCAL_WORLD_ALIGNED, work.hessian);
  const double inv_lin = 1.0 / d_x_lin;
  const double inv_ang = 1.0 / d_x_ang;
  for (Eigen::Index j = 0; j < n; ++j) {
    for (Eigen::Index i = 0; i < 6; ++i) {
      work.j_bar(i, j) = work.j6(i, j) * (i < 3 ? inv_lin : inv_ang) * d_q[j];
    }
  }
  Eigen::Matrix<double, 6, 6> a = delta * Eigen::Matrix<double, 6, 6>::Identity();
  a.noalias() += work.j_bar * work.j_bar.transpose();
  const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(a);
  if (llt.info() != Eigen::Success) {
    return false;
  }
  double log_det = 0.0;
  for (Eigen::Index i = 0; i < 6; ++i) {
    log_det += 2.0 * std::log(llt.matrixLLT()(i, i));
  }
  if (!std::isfinite(log_det)) {
    return false;
  }
  psi = -log_det;
  // ∂ψ/∂q_k = −2 tr(A⁻¹ ∂J̄_k J̄ᵀ) = −2 Σ_ij B_ij (∂J̄_k)_ij,  B = A⁻¹ J̄.
  work.b = work.j_bar;
  llt.solveInPlace(work.b);
  for (Eigen::Index k = 0; k < n; ++k) {
    double sum = 0.0;
    for (Eigen::Index j = 0; j < n; ++j) {
      for (Eigen::Index i = 0; i < 6; ++i) {
        sum += work.b(i, j) * (i < 3 ? inv_lin : inv_ang) * d_q[j] * work.hessian(i, j, k);
      }
    }
    grad[k] = -2.0 * sum;
  }
  return true;
}

}  // namespace rtc::catching
