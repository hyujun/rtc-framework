/// @file barrier_costs.cpp
/// @brief The piecewise-polynomial barrier and the joint-limit and
///        sphere-pair costs built on it, and the relaxed log barrier and the
///        contact wrench cone cost built on it (see the header).

#include "rtc_mpc/ocp/barrier_costs.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/skew.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace rtc::mpc {

// ── Barrier ─────────────────────────────────────────────────────────────────

double PolynomialBarrier::Value(double h) const noexcept {
  const double d = delta;
  if (h >= d) {
    return 0.0;
  }
  const double base = 0.5 * h * h - 0.5 * d * h + d * d / 6.0;
  if (h <= 0.0) {
    return mu * base;
  }
  return mu * (-h * h * h / (6.0 * d) + base);
}

double PolynomialBarrier::Derivative(double h) const noexcept {
  const double d = delta;
  if (h >= d) {
    return 0.0;
  }
  if (h <= 0.0) {
    return mu * (h - 0.5 * d);
  }
  return mu * (-h * h / (2.0 * d) + h - 0.5 * d);
}

double PolynomialBarrier::SecondDerivative(double h) const noexcept {
  const double d = delta;
  if (h >= d) {
    return 0.0;
  }
  if (h <= 0.0) {
    return mu;
  }
  return mu * (1.0 - h / d);
}

double RelaxedLogBarrier::Value(double h) const noexcept {
  if (h > delta) {
    return -mu * std::log(h);
  }
  const double r = (h - 2.0 * delta) / delta;
  return mu * (-std::log(delta) + 0.5 * r * r - 0.5);
}

double RelaxedLogBarrier::Derivative(double h) const noexcept {
  if (h > delta) {
    return -mu / h;
  }
  return mu * (h - 2.0 * delta) / (delta * delta);
}

double RelaxedLogBarrier::SecondDerivative(double h) const noexcept {
  if (h > delta) {
    return mu / (h * h);
  }
  return mu / (delta * delta);
}

// ── Joint limits ────────────────────────────────────────────────────────────

bool JointLimitBarrierCost::IsLimitedJoint(const pinocchio::Model& model,
                                           pinocchio::JointIndex j) noexcept {
  const auto& joint = model.joints[j];
  if (joint.nq() != 1 || joint.nv() != 1) {
    return false;
  }
  const double lo = model.lowerPositionLimit[joint.idx_q()];
  const double hi = model.upperPositionLimit[joint.idx_q()];
  return std::isfinite(lo) && std::isfinite(hi);
}

JointLimitBarrierCost::JointLimitBarrierCost(const Space& space, int nu,
                                             const pinocchio::Model& model,
                                             const PolynomialBarrier& barrier)
    : Base(space, nu), barrier_(barrier) {
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model.njoints); ++j) {
    if (!IsLimitedJoint(model, j)) {
      continue;
    }
    const auto& joint = model.joints[j];
    idx_q_.push_back(joint.idx_q());
    idx_v_.push_back(joint.idx_v());
    lower_.push_back(model.lowerPositionLimit[joint.idx_q()]);
    upper_.push_back(model.upperPositionLimit[joint.idx_q()]);
  }
}

int JointLimitBarrierCost::CountLimitedJoints(const pinocchio::Model& model) noexcept {
  int n = 0;
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model.njoints); ++j) {
    n += IsLimitedJoint(model, j) ? 1 : 0;
  }
  return n;
}

void JointLimitBarrierCost::evaluate(const ConstVectorRef& x, const ConstVectorRef& /*u*/,
                                     CostData& data) const {
  double v = 0.0;
  for (std::size_t i = 0; i < idx_q_.size(); ++i) {
    const double q = x[idx_q_[i]];
    v += barrier_.Value(q - lower_[i]) + barrier_.Value(upper_[i] - q);
  }
  data.value_ = v;
}

// Only the joint entries are ever written, by assignment: the rest of the
// gradient and Hessian keep the zeros the data was created with.
void JointLimitBarrierCost::computeGradients(const ConstVectorRef& x, const ConstVectorRef& /*u*/,
                                             CostData& data) const {
  for (std::size_t i = 0; i < idx_q_.size(); ++i) {
    const double q = x[idx_q_[i]];
    data.Lx_[idx_v_[i]] = barrier_.Derivative(q - lower_[i]) - barrier_.Derivative(upper_[i] - q);
  }
}

void JointLimitBarrierCost::computeHessians(const ConstVectorRef& x, const ConstVectorRef& /*u*/,
                                            CostData& data) const {
  for (std::size_t i = 0; i < idx_q_.size(); ++i) {
    const double q = x[idx_q_[i]];
    data.Lxx_(idx_v_[i], idx_v_[i]) =
        barrier_.SecondDerivative(q - lower_[i]) + barrier_.SecondDerivative(upper_[i] - q);
  }
}

// ── Sphere pairs ────────────────────────────────────────────────────────────

CollisionBarrierData::CollisionBarrierData(int ndx, int nu, const pinocchio::Model& model,
                                           int n_pairs)
    : Base(ndx, nu),
      pin_data(model),
      h(static_cast<std::size_t>(n_pairs), 0.0),
      grad_h(Eigen::MatrixXd::Zero(n_pairs, model.nv)),
      frame_jac(Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv)),
      point_a(Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, model.nv)),
      point_b(Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, model.nv)),
      scratch(Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, model.nv)) {}

CollisionBarrierCost::CollisionBarrierCost(const Space& space, const Ode& ode,
                                           std::vector<SpherePair> pairs,
                                           const PolynomialBarrier& barrier)
    : Base(space, ode.nu()),
      pairs_(std::move(pairs)),
      barrier_(barrier),
      contact_ids_(ode.contact_ids_),
      nq_(ode.pin_model_.nq),
      njoints_(ode.pin_model_.njoints),
      nframes_(ode.pin_model_.nframes) {}

bool CollisionBarrierCost::Bind(const Ode* ode) noexcept {
  if (ode != nullptr &&
      (ode->ndx() != ndx() || ode->nu() != nu || ode->contact_ids_ != contact_ids_ ||
       ode->pin_model_.nq != nq_ || ode->pin_model_.njoints != njoints_ ||
       ode->pin_model_.nframes != nframes_)) {
    return false;
  }
  ode_ = ode;
  return true;
}

namespace {

/// The world Jacobian (q tangent) of the point at @p offset in frame @p fid:
/// `R·(J_lin − [o]× J_ang)` from the LOCAL frame Jacobian. Needs the joint
/// Jacobians and the frame placement of the current q.
void PointJacobian(const pinocchio::Model& model, CollisionBarrierData& d,
                   pinocchio::FrameIndex fid, const Eigen::Vector3d& offset,
                   Eigen::Matrix<double, 3, Eigen::Dynamic>& out) noexcept {
  d.frame_jac.setZero();
  pinocchio::getFrameJacobian(model, d.pin_data, fid, pinocchio::LOCAL, d.frame_jac);
  const Eigen::Matrix3d& rot = d.pin_data.oMf[fid].rotation();
  const Eigen::Matrix3d skew_o = pinocchio::skew(offset);
  d.scratch.noalias() = d.frame_jac.topRows<3>();
  d.scratch.noalias() -= skew_o.lazyProduct(d.frame_jac.bottomRows<3>());
  out.noalias() = rot.lazyProduct(d.scratch);
}

}  // namespace

void CollisionBarrierCost::evaluate(const ConstVectorRef& x, const ConstVectorRef& /*u*/,
                                    CostData& data) const {
  auto& d = static_cast<CollisionBarrierData&>(data);
  d.evaluated = false;
  d.active = false;
  data.value_ = 0.0;
  if (ode_ == nullptr) {
    data.value_ = std::nan("");
    return;
  }
  d.evaluated = true;
  for (const bool s : ode_->contact_states_) {
    d.active = d.active || !s;
  }
  if (!d.active) {
    return;
  }
  const pinocchio::Model& model = ode_->pin_model_;
  pinocchio::forwardKinematics(model, d.pin_data, x.head(model.nq));
  double v = 0.0;
  for (std::size_t i = 0; i < pairs_.size(); ++i) {
    const SpherePair& p = pairs_[i];
    const pinocchio::SE3& ma = pinocchio::updateFramePlacement(model, d.pin_data, p.frame_a);
    const Eigen::Vector3d pa = ma.act(p.offset_a);
    const pinocchio::SE3& mb = pinocchio::updateFramePlacement(model, d.pin_data, p.frame_b);
    const Eigen::Vector3d pb = mb.act(p.offset_b);
    d.h[i] = (pa - pb).norm() - p.radius_a - p.radius_b;
    v += barrier_.Value(d.h[i]);
  }
  data.value_ = v;
}

void CollisionBarrierCost::computeGradients(const ConstVectorRef& x, const ConstVectorRef& /*u*/,
                                            CostData& data) const {
  auto& d = static_cast<CollisionBarrierData&>(data);
  const int nv = static_cast<int>(d.grad_h.cols());
  data.Lx_.head(nv).setZero();
  d.grad_h.setZero();
  if (ode_ == nullptr) {
    data.Lx_.head(nv).setConstant(std::nan(""));
    return;
  }
  if (!d.evaluated || !d.active) {
    return;
  }
  const pinocchio::Model& model = ode_->pin_model_;
  pinocchio::computeJointJacobians(model, d.pin_data, x.head(model.nq));
  for (std::size_t i = 0; i < pairs_.size(); ++i) {
    const SpherePair& p = pairs_[i];
    const double db = barrier_.Derivative(d.h[i]);
    const Eigen::Vector3d pa =
        pinocchio::updateFramePlacement(model, d.pin_data, p.frame_a).act(p.offset_a);
    const Eigen::Vector3d pb =
        pinocchio::updateFramePlacement(model, d.pin_data, p.frame_b).act(p.offset_b);
    const Eigen::Vector3d diff = pa - pb;
    const double dist = diff.norm();
    if (!(dist >= kMinDistance)) {
      continue;  // direction undefined: no gradient
    }
    const Eigen::Vector3d n = diff / dist;
    PointJacobian(model, d, p.frame_a, p.offset_a, d.point_a);
    PointJacobian(model, d, p.frame_b, p.offset_b, d.point_b);
    const auto row = static_cast<Eigen::Index>(i);
    d.grad_h.row(row).noalias() = n.transpose() * d.point_a;
    d.grad_h.row(row).noalias() -= n.transpose() * d.point_b;
    data.Lx_.head(nv).noalias() += db * d.grad_h.row(row).transpose();
  }
}

void CollisionBarrierCost::computeHessians(const ConstVectorRef& /*x*/, const ConstVectorRef& /*u*/,
                                           CostData& data) const {
  auto& d = static_cast<CollisionBarrierData&>(data);
  const int nv = static_cast<int>(d.grad_h.cols());
  data.Lxx_.topLeftCorner(nv, nv).setZero();
  if (ode_ == nullptr) {
    data.Lxx_.topLeftCorner(nv, nv).setConstant(std::nan(""));
    return;
  }
  if (!d.evaluated || !d.active) {
    return;
  }
  for (std::size_t i = 0; i < pairs_.size(); ++i) {
    const double ddb = barrier_.SecondDerivative(d.h[i]);
    if (ddb == 0.0) {
      continue;
    }
    const auto row = static_cast<Eigen::Index>(i);
    data.Lxx_.topLeftCorner(nv, nv).noalias() +=
        ddb * d.grad_h.row(row).transpose() * d.grad_h.row(row);
  }
}

std::shared_ptr<CollisionBarrierCost::CostData> CollisionBarrierCost::createData() const {
  if (ode_ == nullptr) {
    throw std::logic_error("CollisionBarrierCost::createData: Bind the stage's ODE first");
  }
  return std::make_shared<CollisionBarrierData>(ndx(), nu, ode_->pin_model_, n_pairs());
}

// ── Contact wrench cone ─────────────────────────────────────────────────────

ContactConeBarrierData::ContactConeBarrierData(
    int ndx, int nu, int nv, std::shared_ptr<aligator::StageFunctionDataTpl<double>> cone_data)
    : Base(ndx, nu),
      cone(std::move(cone_data)),
      jx(Eigen::VectorXd::Zero(nv)),
      jx_scaled(Eigen::VectorXd::Zero(nv)) {}

ContactConeBarrierCost::ContactConeBarrierCost(const Space& space,
                                               const ContactWrenchConeResidual& residual,
                                               const RelaxedLogBarrier& friction,
                                               const RelaxedLogBarrier& cop)
    : Base(space, residual.nu), residual_(residual), friction_(friction), cop_(cop) {}

void ContactConeBarrierCost::evaluate(const ConstVectorRef& x, const ConstVectorRef& u,
                                      CostData& data) const {
  auto& d = static_cast<ContactConeBarrierData&>(data);
  if (residual_.bound_ode() == nullptr) {
    data.value_ = std::nan("");
    return;
  }
  residual_.evaluate(x, u, *d.cone);
  const auto& rd = static_cast<const ContactWrenchConeData&>(*d.cone);
  const int rows = residual_.rows_per_contact();
  double v = 0.0;
  for (int c = 0; c < residual_.n_contacts(); ++c) {
    if (rd.stance[static_cast<std::size_t>(c)] == 0) {
      continue;
    }
    const Eigen::Index o = static_cast<Eigen::Index>(c) * rows;
    v += friction_.Value(-rd.value_[o]);
    for (int r = 1; r < rows; ++r) {
      v += cop_.Value(-rd.value_[o + r]);
    }
  }
  data.value_ = v;
}

// The residual's Jacobian rows are sparse by construction: the friction row
// has `∂/∂f` only (3 entries), a CoP row `∂/∂q` (first nv tangent entries)
// and `∂/∂[f; τ]` of its own contact (6 entries). The products below are
// written on those blocks, in fixed sizes or into the data's buffers.
void ContactConeBarrierCost::computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                                              CostData& data) const {
  auto& d = static_cast<ContactConeBarrierData&>(data);
  data.Lx_.setZero();
  data.Lu_.setZero();
  if (residual_.bound_ode() == nullptr) {
    data.Lx_.setConstant(std::nan(""));
    data.Lu_.setConstant(std::nan(""));
    return;
  }
  residual_.computeJacobians(x, u, *d.cone);
  const auto& rd = static_cast<const ContactWrenchConeData&>(*d.cone);
  const int rows = residual_.rows_per_contact();
  const int fs = residual_.force_size();
  const auto nv = static_cast<Eigen::Index>(d.jx.size());
  for (int c = 0; c < residual_.n_contacts(); ++c) {
    if (rd.stance[static_cast<std::size_t>(c)] == 0) {
      continue;
    }
    const Eigen::Index o = static_cast<Eigen::Index>(c) * rows;
    const Eigen::Index w = static_cast<Eigen::Index>(c) * fs;
    // ∇h = −∇c.
    const double gf = friction_.Derivative(-rd.value_[o]);
    data.Lu_.segment<3>(w).noalias() -= gf * rd.Ju_.row(o).segment<3>(w).transpose();
    for (int r = 1; r < rows; ++r) {
      const Eigen::Index i = o + r;
      const double g = cop_.Derivative(-rd.value_[i]);
      data.Lx_.head(nv).noalias() -= g * rd.Jx_.row(i).head(nv).transpose();
      data.Lu_.segment<6>(w).noalias() -= g * rd.Ju_.row(i).segment<6>(w).transpose();
    }
  }
}

void ContactConeBarrierCost::computeHessians(const ConstVectorRef& /*x*/, const ConstVectorRef& u,
                                             CostData& data) const {
  auto& d = static_cast<ContactConeBarrierData&>(data);
  data.Lxx_.setZero();
  data.Lxu_.setZero();
  data.Lux_.setZero();
  data.Luu_.setZero();
  if (residual_.bound_ode() == nullptr) {
    data.Lxx_.setConstant(std::nan(""));
    data.Luu_.setConstant(std::nan(""));
    return;
  }
  const auto& rd = static_cast<const ContactWrenchConeData&>(*d.cone);
  const int rows = residual_.rows_per_contact();
  const int fs = residual_.force_size();
  const auto nv = static_cast<Eigen::Index>(d.jx.size());
  const double eps = residual_.friction().regularization;
  for (int c = 0; c < residual_.n_contacts(); ++c) {
    if (rd.stance[static_cast<std::size_t>(c)] == 0) {
      continue;
    }
    const Eigen::Index o = static_cast<Eigen::Index>(c) * rows;
    const Eigen::Index w = static_cast<Eigen::Index>(c) * fs;
    // Friction: Gauss–Newton on ∂h/∂f, plus B'(h)·∇²h on (f_x, f_y).
    const double hf = -rd.value_[o];
    const Eigen::Vector3d jf = rd.Ju_.row(o).segment<3>(w).transpose();
    data.Luu_.block<3, 3>(w, w).noalias() += friction_.SecondDerivative(hf) * jf * jf.transpose();
    const Eigen::Vector2d fxy = u.segment<2>(w);
    const double s2 = fxy.squaredNorm() + eps;
    const double s = std::sqrt(s2);
    const Eigen::Matrix2d hess_sqrt =
        (Eigen::Matrix2d::Identity() - fxy * fxy.transpose() / s2) / s;
    data.Luu_.block<2, 2>(w, w).noalias() -= friction_.Derivative(hf) * hess_sqrt;
    // CoP: Gauss–Newton on ∂h/∂q and ∂h/∂[f; τ].
    for (int r = 1; r < rows; ++r) {
      const Eigen::Index i = o + r;
      const double g2 = cop_.SecondDerivative(-rd.value_[i]);
      d.jx = -rd.Jx_.row(i).head(nv).transpose();
      d.jx_scaled = g2 * d.jx;
      const Eigen::Matrix<double, 6, 1> ju = -rd.Ju_.row(i).segment<6>(w).transpose();
      data.Lxx_.topLeftCorner(nv, nv).noalias() += d.jx_scaled * d.jx.transpose();
      data.Lxu_.block(0, w, nv, 6).noalias() += d.jx_scaled * ju.transpose();
      data.Luu_.block<6, 6>(w, w).noalias() += g2 * ju * ju.transpose();
    }
  }
  data.Lux_ = data.Lxu_.transpose();
}

std::shared_ptr<ContactConeBarrierCost::CostData> ContactConeBarrierCost::createData() const {
  const Ode* ode = residual_.bound_ode();
  if (ode == nullptr) {
    throw std::logic_error("ContactConeBarrierCost::createData: Bind the stage's ODE first");
  }
  return std::make_shared<ContactConeBarrierData>(ndx(), nu, ode->pin_model_.nv,
                                                  residual_.createData());
}

}  // namespace rtc::mpc
