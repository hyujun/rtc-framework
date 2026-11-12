#include "rtc_tsid/constraints/contact_constraint.hpp"

#include "rtc_math/se3/pinocchio_adapter.hpp"
#include "rtc_math/se3/pose_error.hpp"
#include "rtc_tsid/core/yaml_vector.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::tsid {
namespace {

// A gain key: a scalar for every axis or exactly six values, finite and ≥ 0.
Eigen::Matrix<double, 6, 1> ReadGain(const YAML::Node& node, const char* key) {
  if (!node[key]) {
    return Eigen::Matrix<double, 6, 1>::Zero();
  }
  return ReadScalarOrVector(node, key, 6, 0.0, /*strict=*/false, "contact_accel.baumgarte");
}

}  // namespace

void ContactConstraint::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                             PinocchioCache& /*cache*/, const YAML::Node& constraint_config) {
  nv_ = robot_info.nv;
  baumgarte_ = false;
  use_terrain_ = false;
  kp_.setZero();
  kd_.setZero();
  if (constraint_config && constraint_config["baumgarte"]) {
    const YAML::Node b = constraint_config["baumgarte"];
    if (!b.IsMap()) {
      throw std::invalid_argument("contact_accel.baumgarte: must be a map {kp, kd}");
    }
    kp_ = ReadGain(b, "kp");
    kd_ = ReadGain(b, "kd");
    // All-zero gains: the plain rows, bit for bit, and no anchor.
    baumgarte_ = (kp_.array() != 0.0).any() || (kd_.array() != 0.0).any();
  }
  if (constraint_config && constraint_config["terrain_height"]) {
    terrain_height_ = constraint_config["terrain_height"].as<double>();
    if (!std::isfinite(terrain_height_)) {
      throw std::invalid_argument("contact_accel.terrain_height must be finite");
    }
    use_terrain_ = true;
  }
}

void ContactConstraint::SetContactManager(const ContactManagerConfig* manager) {
  manager_ = manager;
  slots_.assign(manager ? static_cast<std::size_t>(manager->max_contacts) : 0, Slot{});
}

void ContactConstraint::Reset() noexcept {
  for (Slot& s : slots_) {
    s.latched = false;
  }
}

int ContactConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  // Stage A-5a: fixed-dim QP — reserve one row block per contact slot
  // regardless of activation. Inactive contacts produce all-zero rows
  // (trivial 0=0 equality, no constraint on the QP).
  if (!manager_)
    return 0;
  return manager_->max_contact_vars;
}

int ContactConstraint::IneqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int ContactConstraint::MaxEqDim(const ContactManagerConfig& contacts) const noexcept {
  return contacts.max_contact_vars;
}

int ContactConstraint::MaxIneqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return 0;
}

void ContactConstraint::ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                                        const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                        Eigen::Ref<Eigen::MatrixXd> A_block,
                                        Eigen::Ref<Eigen::VectorXd> b_block) noexcept {
  if (!manager_ || slots_.size() < contacts.contacts.size())
    return;

  // Stage A-5a: fixed-dim equality block. Active contacts get the
  // Jc_i·a = -dotJc_i·v (+ a_c*) rows; inactive contacts get zero rows (left
  // as the caller's zero-init, so 0·a = 0 is trivially satisfied).
  int row = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager_->contacts[i].contact_dim;
    Slot& slot = slots_[i];
    const bool active = contacts.contacts[i].active && i < cache.contact_frames.size();

    if (!active) {
      slot.latched = false;  // the next activation latches a new anchor
      slot.a_star.setZero();
      row += cdim;
      continue;
    }

    const auto& fc = cache.contact_frames[i];

    // Jc_i · a = 0  →  A[row:row+cdim, 0:nv] = Jc_i[:cdim, :]
    A_block.block(row, 0, cdim, nv_) = fc.J.topRows(cdim);

    // b = -dJc_i · v (contact acceleration bias)
    b_block.segment(row, cdim) = -fc.dJv.head(cdim);

    if (baumgarte_) {
      if (!slot.latched) {
        // A non-finite pose is not an anchor: skip the term and wait.
        if (!fc.oMf.translation().allFinite() || !fc.oMf.rotation().allFinite()) {
          slot.a_star.setZero();
          row += cdim;
          continue;
        }
        slot.anchor = fc.oMf;
        if (use_terrain_) {
          slot.anchor.translation()[2] = terrain_height_;
        }
        slot.latched = true;
      }
      // e = [p − p*, log3(R R*ᵀ)] in world axes: minus rtc_math's SplitWorld
      // error (p* − p, log3(R* Rᵀ)).
      namespace se3 = rtc::math::se3;
      const se3::Vec6 e = -se3::computePoseError(se3::toIso3(fc.oMf), se3::toIso3(slot.anchor),
                                                 se3::ErrorType::SplitWorld);
      const Eigen::Matrix<double, 6, 1> v_c = fc.J * cache.v;
      slot.a_star = -(kp_.array() * e.array() + kd_.array() * v_c.array()).matrix();
      b_block.segment(row, cdim) += slot.a_star.head(cdim);
    }

    row += cdim;
  }
}

void ContactConstraint::ComputeInequality(const PinocchioCache& /*cache*/,
                                          const ContactState& /*contacts*/,
                                          const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                          Eigen::Ref<Eigen::MatrixXd> /*C_block*/,
                                          Eigen::Ref<Eigen::VectorXd> /*l_block*/,
                                          Eigen::Ref<Eigen::VectorXd> /*u_block*/) noexcept {}

}  // namespace rtc::tsid
