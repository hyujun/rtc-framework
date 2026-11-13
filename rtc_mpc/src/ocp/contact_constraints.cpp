/// @file contact_constraints.cpp
/// @brief Values and analytic Jacobians of the contact equality and the
///        wrench cone residual (see the header for the rows).

#include "rtc_mpc/ocp/contact_constraints.hpp"

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

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace rtc::mpc {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

const ContactHeightTarget kNoTarget{};

/// Row indices in `ContactKinematics`' `[p; θ; v; ω; a; α]`.
constexpr Eigen::Index kP = 0;
constexpr Eigen::Index kTheta = 3;
constexpr Eigen::Index kV = 6;
constexpr Eigen::Index kW = 9;
constexpr Eigen::Index kA = 12;
constexpr Eigen::Index kAlpha = 15;

bool UnitNormal(const Eigen::Vector3d& n) noexcept {
  return n.allFinite() && std::fabs(n.squaredNorm() - 1.0) <= kSurfaceNormalTolerance;
}

/// Whether @p ode can stand in for the one a residual was made from.
bool SameStructure(const KinodynamicDynamics::Ode& ode, int ndx, int nu,
                   const std::vector<pinocchio::FrameIndex>& ids, int force_size, int nq,
                   int njoints, int nframes) noexcept {
  return ode.ndx() == ndx && ode.nu() == nu && ode.contact_ids_ == ids &&
         ode.force_size_ == force_size && ode.pin_model_.nq == nq &&
         ode.pin_model_.njoints == njoints && ode.pin_model_.nframes == nframes;
}

/// Copy the bound ODE's mode; whether any contact is in stance.
bool ReadMode(const KinodynamicDynamics::Ode& ode, std::vector<char>& stance) noexcept {
  bool any = false;
  for (std::size_t k = 0; k < stance.size(); ++k) {
    stance[k] = ode.contact_states_[k] ? 1 : 0;
    any = any || stance[k] != 0;
  }
  return any;
}

}  // namespace

// ── Equality ────────────────────────────────────────────────────────────────

ContactConstraintData::ContactConstraintData(int ndx, int nu, int nr,
                                             const KinodynamicDynamics::Ode& ode)
    : Base(ndx, nu, nr),
      kin(ode),
      stance(ode.contact_ids_.size(), 0),
      jx(Eigen::MatrixXd::Zero(kFrameKinematicsRows, ndx)),
      ju(Eigen::MatrixXd::Zero(kFrameKinematicsRows, nu)),
      tilt_jac(Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, ode.pin_model_.nv)) {}

ContactConstraintResidual::ContactConstraintResidual(const Ode& ode,
                                                     const FootConstraintGains& gains)
    : Base(ode.ndx(), ode.nu(),
           EqualityRows(ode.force_size_) * static_cast<int>(ode.contact_ids_.size())),
      contact_ids_(ode.contact_ids_),
      force_size_(ode.force_size_),
      nq_(ode.pin_model_.nq),
      njoints_(ode.pin_model_.njoints),
      nframes_(ode.pin_model_.nframes),
      gains_(gains),
      targets_(ode.contact_ids_.size()) {}

bool ContactConstraintResidual::Bind(const Ode* ode) noexcept {
  if (ode != nullptr &&
      !SameStructure(*ode, ndx1, nu, contact_ids_, force_size_, nq_, njoints_, nframes_)) {
    return false;
  }
  ode_ = ode;
  return true;
}

bool ContactConstraintResidual::AcceptsStage(const StageReference& stage,
                                             const SurfaceFrame& terrain, int n_contacts) noexcept {
  if (n_contacts < 0 || n_contacts > kMaxContacts) {
    return false;
  }
  if (!UnitNormal(Eigen::Vector3d(terrain.normal[0], terrain.normal[1], terrain.normal[2]))) {
    return false;
  }
  for (int k = 0; k < n_contacts; ++k) {
    const auto i = static_cast<std::size_t>(k);
    if (!std::isfinite(stage.contact_pose[7 * i + 2]) ||
        !std::isfinite(stage.contact_twist[6 * i + 2]) ||
        !std::isfinite(stage.contact_acceleration[6 * i + 2])) {
      return false;
    }
  }
  return true;
}

bool ContactConstraintResidual::SetFromStage(const StageReference& stage,
                                             const SurfaceFrame& terrain) noexcept {
  const int nc = n_contacts();
  if (!AcceptsStage(stage, terrain, nc)) {
    return false;
  }
  for (std::size_t i = 0; i < targets_.size(); ++i) {
    targets_[i].z = stage.contact_pose[7 * i + 2];
    targets_[i].vz = stage.contact_twist[6 * i + 2];
    targets_[i].az = stage.contact_acceleration[6 * i + 2];
  }
  normal_ = Eigen::Vector3d(terrain.normal[0], terrain.normal[1], terrain.normal[2]).normalized();
  return true;
}

const ContactHeightTarget& ContactConstraintResidual::target(int k) const noexcept {
  return k >= 0 && k < n_contacts() ? targets_[static_cast<std::size_t>(k)] : kNoTarget;
}

void ContactConstraintResidual::evaluate(const ConstVectorRef& x, const ConstVectorRef& u,
                                         BaseData& data) const {
  auto& d = static_cast<ContactConstraintData&>(data);
  d.evaluated = false;
  if (ode_ == nullptr) {
    d.value_.setConstant(kNaN);
    return;
  }
  static_cast<void>(ReadMode(*ode_, d.stance));
  const pinocchio::Model& model = ode_->pin_model_;
  d.kin.Forward(*ode_, x, u);
  d.evaluated = true;
  d.value_.setZero();

  const FootConstraintGains& g = gains_;
  const int rows = rows_per_contact();
  FrameMotion f;
  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    d.kin.FrameMotionOf(model, contact_ids_[k], f);
    const ContactHeightTarget& t = targets_[k];
    VectorRef r = d.value_.segment(static_cast<Eigen::Index>(k) * rows, rows);
    const double dz = f.position.z() - t.z;
    if (d.stance[k] != 0) {
      r[0] = g.linear_velocity_xy * f.linear_velocity.x() +
             g.linear_acceleration_xy * f.linear_acceleration.x();
      r[1] = g.linear_velocity_xy * f.linear_velocity.y() +
             g.linear_acceleration_xy * f.linear_acceleration.y();
      r[2] = g.position_z * dz + g.linear_velocity_z * f.linear_velocity.z() +
             g.linear_acceleration_z * f.linear_acceleration.z();
      if (force_size_ == 6) {
        Eigen::Vector3d tilt;
        FrameTilt(f.rotation.col(2), normal_, kMinArcNorm, tilt, nullptr);
        r.segment<3>(3) = g.orientation * tilt + g.angular_velocity * f.angular_velocity +
                          g.angular_acceleration * f.angular_acceleration;
      }
    } else {
      r[rows - 1] = g.position_z * dz + g.linear_velocity_z * (f.linear_velocity.z() - t.vz) +
                    g.linear_acceleration_z * (f.linear_acceleration.z() - t.az);
    }
  }
}

void ContactConstraintResidual::computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u,
                                                 BaseData& data) const {
  auto& d = static_cast<ContactConstraintData&>(data);
  if (ode_ == nullptr) {
    d.Jx_.setConstant(kNaN);
    d.Ju_.setConstant(kNaN);
    return;
  }
  d.Jx_.setZero();
  d.Ju_.setZero();
  if (!d.evaluated) {
    return;
  }
  // The rows follow the mode of the evaluation, the ODE derivatives the mode
  // the ODE has now: a mode switched in between would mix the two.
  for (std::size_t k = 0; k < d.stance.size(); ++k) {
    if ((d.stance[k] != 0) != static_cast<bool>(ode_->contact_states_[k])) {
      d.Jx_.setConstant(kNaN);
      d.Ju_.setConstant(kNaN);
      return;
    }
  }
  const pinocchio::Model& model = ode_->pin_model_;
  const int nv = model.nv;
  d.kin.Derivatives(*ode_, x, u);

  const FootConstraintGains& g = gains_;
  const int rows = rows_per_contact();
  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    const pinocchio::FrameIndex fid = contact_ids_[k];
    d.kin.FrameJacobian(model, fid, d.jx, d.ju);
    const Eigen::Index o = static_cast<Eigen::Index>(k) * rows;
    MatrixRef jx = d.Jx_.middleRows(o, rows);
    MatrixRef ju = d.Ju_.middleRows(o, rows);
    const Eigen::MatrixXd& kx = d.jx;
    const Eigen::MatrixXd& ku = d.ju;
    if (d.stance[k] != 0) {
      for (Eigen::Index i = 0; i < 2; ++i) {
        jx.row(i) =
            g.linear_velocity_xy * kx.row(kV + i) + g.linear_acceleration_xy * kx.row(kA + i);
        ju.row(i) =
            g.linear_velocity_xy * ku.row(kV + i) + g.linear_acceleration_xy * ku.row(kA + i);
      }
      jx.row(2) = g.position_z * kx.row(kP + 2) + g.linear_velocity_z * kx.row(kV + 2) +
                  g.linear_acceleration_z * kx.row(kA + 2);
      ju.row(2) = g.position_z * ku.row(kP + 2) + g.linear_velocity_z * ku.row(kV + 2) +
                  g.linear_acceleration_z * ku.row(kA + 2);
      if (force_size_ == 6) {
        // Tilt: the frame z axis a = R·e_z turns with δθ, δa = δθ × a.
        const Eigen::Vector3d axis = d.kin.pin_data.oMf[fid].rotation().col(2);
        Eigen::Vector3d tilt;
        Eigen::Matrix3d de_da;
        FrameTilt(axis, normal_, kMinArcNorm, tilt, &de_da);
        const Eigen::Matrix3d de_dtheta = g.orientation * (-de_da * pinocchio::skew(axis));
        d.tilt_jac.noalias() = de_dtheta.lazyProduct(kx.block(kTheta, 0, 3, nv));
        jx.middleRows<3>(3) = g.angular_velocity * kx.middleRows<3>(kW) +
                              g.angular_acceleration * kx.middleRows<3>(kAlpha);
        jx.block(3, 0, 3, nv) += d.tilt_jac;
        ju.middleRows<3>(3) = g.angular_velocity * ku.middleRows<3>(kW) +
                              g.angular_acceleration * ku.middleRows<3>(kAlpha);
      }
    } else {
      jx.row(rows - 1) = g.position_z * kx.row(kP + 2) + g.linear_velocity_z * kx.row(kV + 2) +
                         g.linear_acceleration_z * kx.row(kA + 2);
      ju.row(rows - 1) = g.position_z * ku.row(kP + 2) + g.linear_velocity_z * ku.row(kV + 2) +
                         g.linear_acceleration_z * ku.row(kA + 2);
    }
  }
}

std::shared_ptr<ContactConstraintResidual::BaseData> ContactConstraintResidual::createData() const {
  if (ode_ == nullptr) {
    throw std::logic_error("ContactConstraintResidual::createData: Bind the stage's ODE first");
  }
  return std::make_shared<ContactConstraintData>(ndx1, nu, nr, *ode_);
}

// ── Inequality ──────────────────────────────────────────────────────────────

ContactWrenchConeData::ContactWrenchConeData(int ndx, int nu, int nr,
                                             const KinodynamicDynamics::Ode& ode)
    : Base(ndx, nu, nr),
      pin_data(ode.pin_model_),
      stance(ode.contact_ids_.size(), 0),
      frame_jac(Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, ode.pin_model_.nv)),
      jw(Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, ode.pin_model_.nv)) {}

ContactWrenchConeResidual::ContactWrenchConeResidual(const Ode& ode, const FrictionCone& friction,
                                                     const ContactRectangle& rectangle)
    : Base(ode.ndx(), ode.nu(),
           InequalityRows(ode.force_size_) * static_cast<int>(ode.contact_ids_.size())),
      contact_ids_(ode.contact_ids_),
      force_size_(ode.force_size_),
      nq_(ode.pin_model_.nq),
      njoints_(ode.pin_model_.njoints),
      nframes_(ode.pin_model_.nframes),
      friction_(friction),
      rectangle_(rectangle) {}

bool ContactWrenchConeResidual::Bind(const Ode* ode) noexcept {
  if (ode != nullptr &&
      !SameStructure(*ode, ndx1, nu, contact_ids_, force_size_, nq_, njoints_, nframes_)) {
    return false;
  }
  ode_ = ode;
  return true;
}

void ContactWrenchConeResidual::evaluate(const ConstVectorRef& x, const ConstVectorRef& u,
                                         BaseData& data) const {
  auto& d = static_cast<ContactWrenchConeData&>(data);
  d.evaluated = false;
  if (ode_ == nullptr) {
    d.value_.setConstant(kNaN);
    return;
  }
  d.value_.setConstant(kInactiveValue);
  d.evaluated = true;
  if (!ReadMode(*ode_, d.stance)) {
    return;
  }
  const pinocchio::Model& model = ode_->pin_model_;
  if (force_size_ == 6) {
    pinocchio::forwardKinematics(model, d.pin_data, x.head(model.nq));
  }
  const int rows = rows_per_contact();
  const FrictionCone& fc = friction_;
  const ContactRectangle& rect = rectangle_;
  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    if (d.stance[k] == 0) {
      continue;
    }
    const Eigen::Index o = static_cast<Eigen::Index>(k) * rows;
    const Eigen::Index w = static_cast<Eigen::Index>(k) * force_size_;
    const Eigen::Vector3d f = u.segment<3>(w);
    d.value_[o] = std::sqrt(f.x() * f.x() + f.y() * f.y() + fc.regularization) -
                  fc.mu * (f.z() + fc.grip_force);
    if (force_size_ != 6) {
      continue;
    }
    const Eigen::Matrix3d& rot =
        pinocchio::updateFramePlacement(model, d.pin_data, contact_ids_[k]).rotation();
    const Eigen::Vector3d tau_l = rot.transpose() * Eigen::Vector3d(u.segment<3>(w + 3));
    const double fz_l = rot.col(2).dot(f);
    d.value_[o + 1] = -(tau_l.x() - rect.y_min * fz_l);
    d.value_[o + 2] = -(-tau_l.x() + rect.y_max * fz_l);
    d.value_[o + 3] = -(-tau_l.y() - rect.x_min * fz_l);
    d.value_[o + 4] = -(tau_l.y() + rect.x_max * fz_l);
  }
}

void ContactWrenchConeResidual::computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u,
                                                 BaseData& data) const {
  auto& d = static_cast<ContactWrenchConeData&>(data);
  if (ode_ == nullptr) {
    d.Jx_.setConstant(kNaN);
    d.Ju_.setConstant(kNaN);
    return;
  }
  d.Jx_.setZero();
  d.Ju_.setZero();
  if (!d.evaluated) {
    return;
  }
  bool any = false;
  for (const char s : d.stance) {
    any = any || s != 0;
  }
  if (!any) {
    return;
  }
  const pinocchio::Model& model = ode_->pin_model_;
  const int nv = model.nv;
  if (force_size_ == 6) {
    pinocchio::computeJointJacobians(model, d.pin_data, x.head(model.nq));
  }
  const int rows = rows_per_contact();
  const FrictionCone& fc = friction_;
  const ContactRectangle& rect = rectangle_;
  for (std::size_t k = 0; k < contact_ids_.size(); ++k) {
    if (d.stance[k] == 0) {
      continue;
    }
    const Eigen::Index o = static_cast<Eigen::Index>(k) * rows;
    const Eigen::Index w = static_cast<Eigen::Index>(k) * force_size_;
    const Eigen::Vector3d f = u.segment<3>(w);
    const double s = std::sqrt(f.x() * f.x() + f.y() * f.y() + fc.regularization);
    d.Ju_(o, w) = f.x() / s;
    d.Ju_(o, w + 1) = f.y() / s;
    d.Ju_(o, w + 2) = -fc.mu;
    if (force_size_ != 6) {
      continue;
    }
    const pinocchio::FrameIndex fid = contact_ids_[k];
    const Eigen::Matrix3d& rot = pinocchio::updateFramePlacement(model, d.pin_data, fid).rotation();
    const Eigen::Vector3d tau = u.segment<3>(w + 3);
    // h rows as a_τ·τ_L + b·F_z with τ_L = Rᵀτ, F_z = (Rᵀf)_z; c = −h.
    // ∂/∂u: ∂τ_L/∂τ = Rᵀ, ∂F_z/∂f = (R·e_z)ᵀ.
    // ∂/∂q: δ(Rᵀw) = Rᵀ[w]× δθ, δθ = J_ω δq (world rotation of the frame).
    const Eigen::RowVector3d tx = rot.col(0).transpose();
    const Eigen::RowVector3d ty = rot.col(1).transpose();
    const Eigen::RowVector3d nz = rot.col(2).transpose();
    const Eigen::Matrix3d m_tau = rot.transpose() * pinocchio::skew(tau);
    const Eigen::Matrix3d m_f = rot.transpose() * pinocchio::skew(f);
    // Rows: (a_x, a_y, b) of h = a_x·τ_x,L + a_y·τ_y,L + b·F_z.
    const std::array<Eigen::Vector3d, 4> coef = {{
        {1.0, 0.0, -rect.y_min},
        {-1.0, 0.0, rect.y_max},
        {0.0, -1.0, -rect.x_min},
        {0.0, 1.0, rect.x_max},
    }};
    d.frame_jac.setZero();
    pinocchio::getFrameJacobian(model, d.pin_data, fid, pinocchio::LOCAL, d.frame_jac);
    d.jw.noalias() = rot.lazyProduct(d.frame_jac.bottomRows<3>());
    for (std::size_t i = 0; i < coef.size(); ++i) {
      const Eigen::Vector3d& c = coef[i];
      const Eigen::Index row = o + 1 + static_cast<Eigen::Index>(i);
      d.Ju_.block<1, 3>(row, w + 3) = -(c.x() * tx + c.y() * ty);
      d.Ju_.block<1, 3>(row, w) = -c.z() * nz;
      const Eigen::RowVector3d dtheta =
          -(c.x() * m_tau.row(0) + c.y() * m_tau.row(1) + c.z() * m_f.row(2));
      d.Jx_.block(row, 0, 1, nv).noalias() = dtheta.lazyProduct(d.jw);
    }
  }
}

std::shared_ptr<ContactWrenchConeResidual::BaseData> ContactWrenchConeResidual::createData() const {
  if (ode_ == nullptr) {
    throw std::logic_error("ContactWrenchConeResidual::createData: Bind the stage's ODE first");
  }
  return std::make_shared<ContactWrenchConeData>(ndx1, nu, nr, *ode_);
}

}  // namespace rtc::mpc
