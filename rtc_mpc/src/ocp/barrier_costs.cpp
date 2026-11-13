/// @file barrier_costs.cpp
/// @brief The piecewise-polynomial barrier and the joint-limit and
///        sphere-pair costs built on it (see the header).

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

}  // namespace rtc::mpc
