/// @file contact_tracking_residual.cpp
/// @brief Value and analytic Jacobian of the contact tracking residual (see
///        the header for the rows, conventions and the binding contract).

#include "rtc_mpc/ocp/contact_tracking_residual.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/skew.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rtc::mpc {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// What `target` returns for a contact that does not exist.
const ContactTrackingTarget kNoTarget{};

/// The tilt `e = −c / N` of a frame whose z axis is @p a against the plane
/// normal @p n (`c = a × n`, `w = 1 + a·n`, `N = ‖(c, w)‖` floored at
/// `ContactTrackingResidual::kMinArcNorm`) and, when @p de_da is given,
/// `∂e/∂a` (zero change of `N` once floored).
void Tilt(const Eigen::Vector3d& a, const Eigen::Vector3d& n, Eigen::Vector3d& e,
          Eigen::Matrix3d* de_da) noexcept {
  const Eigen::Vector3d c = a.cross(n);
  const double w = 1.0 + a.dot(n);
  const double raw = std::sqrt(c.squaredNorm() + w * w);
  const bool floored = !(raw >= ContactTrackingResidual::kMinArcNorm);
  const double norm = floored ? ContactTrackingResidual::kMinArcNorm : raw;
  e = -c / norm;
  if (de_da == nullptr) {
    return;
  }
  // ∂c/∂a = −[n]×, ∂w/∂a = nᵀ.
  const Eigen::Matrix3d dc = -pinocchio::skew(n);
  Eigen::RowVector3d dnorm = Eigen::RowVector3d::Zero();
  if (!floored) {
    dnorm = (c.transpose() * dc + w * n.transpose()) / norm;
  }
  *de_da = -dc / norm + (c / (norm * norm)) * dnorm;
}

bool AllFinite(const ContactTrackingTarget& t) noexcept {
  return t.position.allFinite() && t.linear_velocity.allFinite() &&
         t.angular_velocity.allFinite() && t.linear_acceleration.allFinite() &&
         t.angular_acceleration.allFinite();
}

bool UnitNormal(const Eigen::Vector3d& n) noexcept {
  return n.allFinite() && std::fabs(n.squaredNorm() - 1.0) <= kSurfaceNormalTolerance;
}

bool ValidScale(double s) noexcept {
  return s >= 0.0 && s <= 1.0;
}

}  // namespace

ContactTrackingData::ContactTrackingData(int ndx, int nu, int nr,
                                         const KinodynamicDynamics::Ode& ode)
    : Base(ndx, nu, nr),
      pin_data(ode.pin_model_),
      ode_data(ode.createData()),
      vdot(Eigen::VectorXd::Zero(ode.pin_model_.nv)),
      scale_used(ode.contact_ids_.size(), 0.0),
      v_dq(Matrix6x::Zero(6, ode.pin_model_.nv)),
      v_dv(Matrix6x::Zero(6, ode.pin_model_.nv)),
      a_dq(Matrix6x::Zero(6, ode.pin_model_.nv)),
      a_dv(Matrix6x::Zero(6, ode.pin_model_.nv)),
      a_da(Matrix6x::Zero(6, ode.pin_model_.nv)),
      jw(Matrix3x::Zero(3, ode.pin_model_.nv)),
      lin(Matrix3x::Zero(3, ode.pin_model_.nv)),
      acc_jac(Matrix6x::Zero(6, ode.pin_model_.nv)) {}

ContactTrackingResidual::ContactTrackingResidual(const Ode& ode)
    : Base(ode.ndx(), ode.nu(), kRowsPerContact * static_cast<int>(ode.contact_ids_.size())),
      contact_ids_(ode.contact_ids_),
      nq_(ode.pin_model_.nq),
      njoints_(ode.pin_model_.njoints),
      nframes_(ode.pin_model_.nframes),
      targets_(ode.contact_ids_.size()),
      scales_(ode.contact_ids_.size(), 0.0) {}

bool ContactTrackingResidual::Bind(const Ode* ode) noexcept {
  if (ode != nullptr &&
      (ode->ndx() != ndx1 || ode->nu() != nu || ode->contact_ids_ != contact_ids_ ||
       ode->pin_model_.nq != nq_ || ode->pin_model_.njoints != njoints_ ||
       ode->pin_model_.nframes != nframes_)) {
    return false;
  }
  ode_ = ode;
  return true;
}

bool ContactTrackingResidual::SetTarget(int contact, const ContactTrackingTarget& target) noexcept {
  if (contact < 0 || contact >= n_contacts() || !AllFinite(target)) {
    return false;
  }
  targets_[static_cast<std::size_t>(contact)] = target;
  return true;
}

bool ContactTrackingResidual::SetScale(int contact, double scale) noexcept {
  if (contact < 0 || contact >= n_contacts() || !ValidScale(scale)) {
    return false;
  }
  scales_[static_cast<std::size_t>(contact)] = scale;
  return true;
}

bool ContactTrackingResidual::SetPlaneNormal(const Eigen::Vector3d& normal) noexcept {
  if (!UnitNormal(normal)) {
    return false;
  }
  normal_ = normal.normalized();
  return true;
}

bool ContactTrackingResidual::SetFromStage(const StageReference& stage,
                                           const SurfaceFrame& terrain) noexcept {
  const int nc = n_contacts();
  const Eigen::Vector3d normal(terrain.normal[0], terrain.normal[1], terrain.normal[2]);
  if (!UnitNormal(normal)) {
    return false;
  }
  // Check everything first: a refused node writes nothing.
  for (int k = 0; k < nc; ++k) {
    const auto i = static_cast<std::size_t>(k);
    const bool stance = ((stage.mode >> static_cast<unsigned>(k)) & 1U) != 0U;
    if (!stance && !ValidScale(stage.impact_proximity[i])) {
      return false;
    }
    for (std::size_t j = 0; j < 3; ++j) {
      if (!std::isfinite(stage.contact_pose[7 * i + j])) {
        return false;
      }
    }
    for (std::size_t j = 0; j < 6; ++j) {
      if (!std::isfinite(stage.contact_twist[6 * i + j]) ||
          !std::isfinite(stage.contact_acceleration[6 * i + j])) {
        return false;
      }
    }
  }
  for (int k = 0; k < nc; ++k) {
    const auto i = static_cast<std::size_t>(k);
    ContactTrackingTarget& t = targets_[i];
    t.position = Eigen::Map<const Eigen::Vector3d>(&stage.contact_pose[7 * i]);
    t.linear_velocity = Eigen::Map<const Eigen::Vector3d>(&stage.contact_twist[6 * i]);
    t.angular_velocity = Eigen::Map<const Eigen::Vector3d>(&stage.contact_twist[6 * i + 3]);
    t.linear_acceleration = Eigen::Map<const Eigen::Vector3d>(&stage.contact_acceleration[6 * i]);
    t.angular_acceleration =
        Eigen::Map<const Eigen::Vector3d>(&stage.contact_acceleration[6 * i + 3]);
    const bool stance = ((stage.mode >> static_cast<unsigned>(k)) & 1U) != 0U;
    scales_[i] = stance ? 0.0 : stage.impact_proximity[i];
  }
  normal_ = normal.normalized();
  return true;
}

double ContactTrackingResidual::scale(int contact) const noexcept {
  return contact >= 0 && contact < n_contacts() ? scales_[static_cast<std::size_t>(contact)] : kNaN;
}

const ContactTrackingTarget& ContactTrackingResidual::target(int contact) const noexcept {
  return contact >= 0 && contact < n_contacts() ? targets_[static_cast<std::size_t>(contact)]
                                                : kNoTarget;
}

void ContactTrackingResidual::evaluate(const ConstVectorRef& x, const ConstVectorRef& u,
                                       BaseData& data) const {
  auto& d = static_cast<ContactTrackingData&>(data);
  d.evaluated = false;
  if (ode_ == nullptr) {
    d.value_.setConstant(kNaN);
    return;
  }
  bool any = false;
  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    d.scale_used[k] = scales_[k];
    any = any || scales_[k] != 0.0;
  }
  d.value_.setZero();
  if (!any) {
    return;
  }

  const pinocchio::Model& model = ode_->pin_model_;
  const ConstVectorRef q = x.head(model.nq);
  const ConstVectorRef v = x.segment(model.nq, model.nv);
  ode_->forward(x, u, *d.ode_data);
  d.vdot = d.ode_data->xdot_.segment(model.nv, model.nv);
  pinocchio::forwardKinematics(model, d.pin_data, q, v, d.vdot);
  d.evaluated = true;

  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    const double s = d.scale_used[k];
    if (s == 0.0) {
      continue;
    }
    const pinocchio::FrameIndex fid = contact_ids_[k];
    const ContactTrackingTarget& t = targets_[k];
    const pinocchio::SE3& oMf = pinocchio::updateFramePlacement(model, d.pin_data, fid);
    const pinocchio::Motion vel =
        pinocchio::getFrameVelocity(model, d.pin_data, fid, pinocchio::LOCAL_WORLD_ALIGNED);
    const pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
        model, d.pin_data, fid, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector3d tilt;
    Tilt(oMf.rotation().col(2), normal_, tilt, nullptr);

    VectorRef r = d.value_.segment<kRowsPerContact>(static_cast<Eigen::Index>(k) * kRowsPerContact);
    r.segment<3>(0) = s * (oMf.translation() - t.position);
    r.segment<3>(3) = s * tilt;
    r.segment<3>(6) = s * (vel.linear() - t.linear_velocity);
    r.segment<3>(9) = s * (vel.angular() - t.angular_velocity);
    r.segment<3>(12) = s * (acc.linear() - t.linear_acceleration);
    r.segment<3>(15) = s * (acc.angular() - t.angular_acceleration);
  }
}

void ContactTrackingResidual::computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u,
                                               BaseData& data) const {
  auto& d = static_cast<ContactTrackingData&>(data);
  if (ode_ == nullptr) {
    d.Jx_.setConstant(kNaN);
    d.Ju_.setConstant(kNaN);
    return;
  }
  d.Jx_.setZero();
  d.Ju_.setZero();
  if (!d.evaluated) {
    return;  // every block was skipped by the evaluation
  }

  const pinocchio::Model& model = ode_->pin_model_;
  const int nv = model.nv;
  const ConstVectorRef q = x.head(model.nq);
  const ConstVectorRef v = x.segment(model.nq, nv);
  ode_->dForward(x, u, *d.ode_data);
  // ∂v̇/∂x and ∂v̇/∂u: the velocity-derivative rows of the ODE Jacobians.
  const ConstMatrixRef dvdot_dx = d.ode_data->Jx_.bottomRows(nv);
  const ConstMatrixRef dvdot_du = d.ode_data->Ju_.bottomRows(nv);
  pinocchio::computeForwardKinematicsDerivatives(model, d.pin_data, q, v, d.vdot);

  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    const double s = d.scale_used[k];
    if (s == 0.0) {
      continue;
    }
    const pinocchio::FrameIndex fid = contact_ids_[k];
    // The derivative routines write only the columns of the frame's support.
    d.v_dq.setZero();
    d.v_dv.setZero();
    d.a_dq.setZero();
    d.a_dv.setZero();
    d.a_da.setZero();
    // In LOCAL, then rotated to world axes: x_w = R·x_L, so
    // δx_w = R·δx_L − [x_w]× δθ with δθ = J_ω δq the world rotation.
    // (pinocchio's LOCAL_WORLD_ALIGNED derivatives disagree with central
    // differences of the LOCAL_WORLD_ALIGNED quantities in the q columns —
    // the base-translation columns of the linear rows come out non-zero.)
    pinocchio::getFrameAccelerationDerivatives(model, d.pin_data, fid, pinocchio::LOCAL, d.v_dq,
                                               d.v_dv, d.a_dq, d.a_dv, d.a_da);
    const Eigen::Matrix3d& rot = d.pin_data.oMf[fid].rotation();
    const pinocchio::Motion vel_l =
        pinocchio::getFrameVelocity(model, d.pin_data, fid, pinocchio::LOCAL);
    const pinocchio::Motion acc_l =
        pinocchio::getFrameAcceleration(model, d.pin_data, fid, pinocchio::LOCAL);
    const Eigen::Vector3d v_l = vel_l.linear();
    const Eigen::Vector3d w_l = vel_l.angular();
    const Eigen::Vector3d acc_cl = acc_l.linear() + w_l.cross(v_l);  // classical, LOCAL
    const Eigen::Matrix3d skew_v_w = pinocchio::skew(Eigen::Vector3d(rot * v_l));
    const Eigen::Matrix3d skew_w_w = pinocchio::skew(Eigen::Vector3d(rot * w_l));
    const Eigen::Matrix3d skew_a_w = pinocchio::skew(Eigen::Vector3d(rot * acc_cl));
    const Eigen::Matrix3d skew_alpha_w = pinocchio::skew(Eigen::Vector3d(rot * acc_l.angular()));
    const Eigen::Matrix3d skew_vl = pinocchio::skew(v_l);
    const Eigen::Matrix3d skew_wl = pinocchio::skew(w_l);
    d.jw.noalias() = rot.lazyProduct(d.v_dv.bottomRows<3>());

    const Eigen::Index o = static_cast<Eigen::Index>(k) * kRowsPerContact;
    MatrixRef jq = d.Jx_.block(o, 0, kRowsPerContact, nv);
    MatrixRef jv = d.Jx_.block(o, nv, kRowsPerContact, nv);

    // Position: the linear part of the frame Jacobian, on world axes.
    jq.middleRows<3>(0).noalias() = rot.lazyProduct(d.v_dv.topRows<3>());

    // Tilt: the frame z axis a = R·e_z turns with δθ, δa = δθ × a.
    const Eigen::Vector3d axis = rot.col(2);
    Eigen::Vector3d tilt;
    Eigen::Matrix3d de_da;
    Tilt(axis, normal_, tilt, &de_da);
    const Eigen::Matrix3d de_dtheta = -de_da * pinocchio::skew(axis);
    jq.middleRows<3>(3).noalias() = de_dtheta.lazyProduct(d.jw);

    // Velocity.
    jq.middleRows<3>(6).noalias() = rot.lazyProduct(d.v_dq.topRows<3>());
    jq.middleRows<3>(6).noalias() -= skew_v_w.lazyProduct(d.jw);
    jv.middleRows<3>(6).noalias() = rot.lazyProduct(d.v_dv.topRows<3>());
    jq.middleRows<3>(9).noalias() = rot.lazyProduct(d.v_dq.bottomRows<3>());
    jq.middleRows<3>(9).noalias() -= skew_w_w.lazyProduct(d.jw);
    jv.middleRows<3>(9).noalias() = rot.lazyProduct(d.v_dv.bottomRows<3>());

    // Linear acceleration at fixed v̇: the LOCAL classical acceleration
    // a_L + ω_L × v_L (δ(ω × v) = [ω]× δv − [v]× δω), rotated.
    d.lin.noalias() = d.a_dq.topRows<3>();
    d.lin.noalias() += skew_wl.lazyProduct(d.v_dq.topRows<3>());
    d.lin.noalias() -= skew_vl.lazyProduct(d.v_dq.bottomRows<3>());
    jq.middleRows<3>(12).noalias() = rot.lazyProduct(d.lin);
    jq.middleRows<3>(12).noalias() -= skew_a_w.lazyProduct(d.jw);
    d.lin.noalias() = d.a_dv.topRows<3>();
    d.lin.noalias() += skew_wl.lazyProduct(d.v_dv.topRows<3>());
    d.lin.noalias() -= skew_vl.lazyProduct(d.v_dv.bottomRows<3>());
    jv.middleRows<3>(12).noalias() = rot.lazyProduct(d.lin);

    // Angular acceleration at fixed v̇.
    jq.middleRows<3>(15).noalias() = rot.lazyProduct(d.a_dq.bottomRows<3>());
    jq.middleRows<3>(15).noalias() -= skew_alpha_w.lazyProduct(d.jw);
    jv.middleRows<3>(15).noalias() = rot.lazyProduct(d.a_dv.bottomRows<3>());

    // Through v̇(x, u): ∂a/∂v̇ is the frame Jacobian on world axes (the
    // classical term has no v̇).
    d.acc_jac.topRows<3>().noalias() = rot.lazyProduct(d.a_da.topRows<3>());
    d.acc_jac.bottomRows<3>().noalias() = rot.lazyProduct(d.a_da.bottomRows<3>());
    d.Jx_.middleRows<6>(o + 12).noalias() += d.acc_jac * dvdot_dx;
    d.Ju_.middleRows<6>(o + 12).noalias() = d.acc_jac * dvdot_du;

    if (s != 1.0) {
      d.Jx_.middleRows<kRowsPerContact>(o) *= s;
      d.Ju_.middleRows<kRowsPerContact>(o) *= s;
    }
  }
}

std::shared_ptr<ContactTrackingResidual::BaseData> ContactTrackingResidual::createData() const {
  if (ode_ == nullptr) {
    throw std::logic_error("ContactTrackingResidual::createData: Bind the stage's ODE first");
  }
  return std::make_shared<ContactTrackingData>(ndx1, nu, nr, *ode_);
}

}  // namespace rtc::mpc
