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
    : Base(ndx, nu, nr), kin(ode), scale_used(ode.contact_ids_.size(), 0.0) {}

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

bool ContactTrackingResidual::AcceptsStage(const StageReference& stage, const SurfaceFrame& terrain,
                                           int n_contacts) noexcept {
  if (n_contacts < 0 || n_contacts > kMaxContacts) {
    return false;
  }
  const Eigen::Vector3d normal(terrain.normal[0], terrain.normal[1], terrain.normal[2]);
  if (!UnitNormal(normal)) {
    return false;
  }
  for (int k = 0; k < n_contacts; ++k) {
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
  return true;
}

bool ContactTrackingResidual::SetFromStage(const StageReference& stage,
                                           const SurfaceFrame& terrain) noexcept {
  const int nc = n_contacts();
  // Check everything first: a refused node writes nothing.
  if (!AcceptsStage(stage, terrain, nc)) {
    return false;
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
  normal_ = Eigen::Vector3d(terrain.normal[0], terrain.normal[1], terrain.normal[2]).normalized();
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
  d.kin.Forward(*ode_, x, u);
  d.evaluated = true;

  FrameMotion f;
  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    const double s = d.scale_used[k];
    if (s == 0.0) {
      continue;
    }
    const ContactTrackingTarget& t = targets_[k];
    d.kin.FrameMotionOf(model, contact_ids_[k], f);
    Eigen::Vector3d tilt;
    FrameTilt(f.rotation.col(2), normal_, kMinArcNorm, tilt, nullptr);

    VectorRef r = d.value_.segment<kRowsPerContact>(static_cast<Eigen::Index>(k) * kRowsPerContact);
    r.segment<3>(0) = s * (f.position - t.position);
    r.segment<3>(3) = s * tilt;
    r.segment<3>(6) = s * (f.linear_velocity - t.linear_velocity);
    r.segment<3>(9) = s * (f.angular_velocity - t.angular_velocity);
    r.segment<3>(12) = s * (f.linear_acceleration - t.linear_acceleration);
    r.segment<3>(15) = s * (f.angular_acceleration - t.angular_acceleration);
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
  d.kin.Derivatives(*ode_, x, u);

  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    const double s = d.scale_used[k];
    if (s == 0.0) {
      continue;
    }
    const pinocchio::FrameIndex fid = contact_ids_[k];
    const Eigen::Index o = static_cast<Eigen::Index>(k) * kRowsPerContact;
    d.kin.FrameJacobian(model, fid, d.Jx_.middleRows<kRowsPerContact>(o),
                        d.Ju_.middleRows<kRowsPerContact>(o));

    // Tilt: the frame z axis a = R·e_z turns with δθ, δa = δθ × a.
    const Eigen::Vector3d axis = d.kin.pin_data.oMf[fid].rotation().col(2);
    Eigen::Vector3d tilt;
    Eigen::Matrix3d de_da;
    FrameTilt(axis, normal_, kMinArcNorm, tilt, &de_da);
    const Eigen::Matrix3d de_dtheta = -de_da * pinocchio::skew(axis);
    d.kin.lin = d.Jx_.block(o + 3, 0, 3, nv);  // J_ω
    d.Jx_.block(o + 3, 0, 3, nv).noalias() = de_dtheta.lazyProduct(d.kin.lin);

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
