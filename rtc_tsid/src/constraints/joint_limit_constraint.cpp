#include "rtc_tsid/constraints/joint_limit_constraint.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::tsid {

void JointLimitConstraint::Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                                PinocchioCache& /*cache*/, const YAML::Node& constraint_config) {
  nv_ = robot_info.nv;
  n_base_ = robot_info.n_base_v;
  n_rows_ = nv_ - n_base_;

  // YAML config
  if (constraint_config) {
    dt_ = constraint_config["dt"].as<double>(0.002);
    margin_pos_ = constraint_config["position_margin"].as<double>(0.05);
    margin_vel_ = constraint_config["velocity_margin"].as<double>(0.1);
  }
  // Init is non-RT: reject what would make every box below meaningless.
  if (!(std::isfinite(dt_) && dt_ > 0.0)) {
    throw std::invalid_argument("joint_limit: dt must be finite and > 0");
  }
  if (!(std::isfinite(margin_pos_) && margin_pos_ >= 0.0) ||
      !(std::isfinite(margin_vel_) && margin_vel_ >= 0.0)) {
    throw std::invalid_argument(
        "joint_limit: position_margin / velocity_margin must be finite and >= 0");
  }

  // One row per joint after the root (the free-flyer has none): each must be
  // 1-DoF so its q entry and v entry pair up.
  if (robot_info.nv != model.nv || robot_info.nq != model.nq) {
    throw std::invalid_argument("joint_limit: RobotModelInfo does not match the model");
  }
  q_index_.assign(static_cast<size_t>(n_rows_), -1);
  // A root to skip ⇔ base rows (n_base_ > 0): one field decides both.
  const std::size_t first = n_base_ > 0 ? 2 : 1;
  for (std::size_t k = first; k < model.joints.size(); ++k) {
    const auto& j = model.joints[k];
    if (j.nq() != 1 || j.nv() != 1) {
      throw std::invalid_argument("joint_limit: joint '" + model.names[k] + "' (" + j.shortname() +
                                  ") is not 1-DoF");
    }
    const int row = j.idx_v() - n_base_;
    if (row < 0 || row >= n_rows_) {
      throw std::invalid_argument("joint_limit: joint velocity index outside the rows");
    }
    q_index_[static_cast<size_t>(row)] = j.idx_q();
  }
  for (const int qi : q_index_) {
    if (qi < 0) {
      throw std::invalid_argument("joint_limit: a velocity row has no 1-DoF joint");
    }
  }

  // Position limits + margin (per-joint margin clamping): a margin wider than
  // half the range is clamped to 0.
  q_min_.resize(n_rows_);
  q_max_.resize(n_rows_);
  for (int i = 0; i < n_rows_; ++i) {
    const int qi = q_index_[static_cast<size_t>(i)];
    const double ql = robot_info.q_lower(qi);
    const double qu = robot_info.q_upper(qi);
    // q_min_ <= q_max_ below relies on this (the collapse assumes at most
    // one side of a row can cross).
    if (!(std::isfinite(ql) && std::isfinite(qu) && ql <= qu)) {
      throw std::invalid_argument(
          "joint_limit: position limits must be finite with lower <= upper");
    }
    const double range = qu - ql;
    const double eff_margin = (range > 2.0 * margin_pos_) ? margin_pos_ : 0.0;
    q_min_(i) = ql + eff_margin;
    q_max_(i) = qu - eff_margin;
  }

  // Velocity limits + margin (per-joint margin clamping)
  v_min_.resize(n_rows_);
  v_max_limit_.resize(n_rows_);
  for (int i = 0; i < n_rows_; ++i) {
    const double vl = robot_info.v_max(n_base_ + i);
    // v_max == 0 is a locked joint and legitimate; negative or non-finite
    // would put vel_lb above vel_ub on every tick.
    if (!(std::isfinite(vl) && vl >= 0.0)) {
      throw std::invalid_argument("joint_limit: velocity limit must be finite and >= 0");
    }
    const double eff_margin = (vl > 2.0 * margin_vel_) ? margin_vel_ : 0.0;
    v_min_(i) = -vl + eff_margin;
    v_max_limit_(i) = vl - eff_margin;
  }

  // Workspace pre-allocate
  a_pos_lb_.setZero(n_rows_);
  a_pos_ub_.setZero(n_rows_);
  a_vel_lb_.setZero(n_rows_);
  a_vel_ub_.setZero(n_rows_);
}

int JointLimitConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int JointLimitConstraint::IneqDim(const ContactState& /*contacts*/) const noexcept {
  return n_rows_;
}

int JointLimitConstraint::MaxEqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return 0;
}

int JointLimitConstraint::MaxIneqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return n_rows_;
}

void JointLimitConstraint::ComputeEquality(const PinocchioCache& /*cache*/,
                                           const ContactState& /*contacts*/,
                                           const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                           Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
                                           Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {
  // No equality constraints
}

void JointLimitConstraint::ComputeInequality(const PinocchioCache& cache,
                                             const ContactState& /*contacts*/,
                                             const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                             Eigen::Ref<Eigen::MatrixXd> C_block,
                                             Eigen::Ref<Eigen::VectorXd> l_block,
                                             Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  const double dt2 = dt_ * dt_;

  // C row i = e_(n_base + i)
  for (int i = 0; i < n_rows_; ++i) {
    C_block(i, n_base_ + i) = 1.0;
  }

  int inverted = 0;

  for (int i = 0; i < n_rows_; ++i) {
    const double vi = cache.v(n_base_ + i);
    const double qi = cache.q(q_index_[static_cast<size_t>(i)]);

    // Position-based bounds: a ≥ 2·(q_min - q - v·dt) / dt²
    a_pos_lb_(i) = 2.0 * (q_min_(i) - qi - vi * dt_) / dt2;
    a_pos_ub_(i) = 2.0 * (q_max_(i) - qi - vi * dt_) / dt2;

    // Velocity-based bounds: a ≥ (v_min - v) / dt
    a_vel_lb_(i) = (v_min_(i) - vi) / dt_;
    a_vel_ub_(i) = (v_max_limit_(i) - vi) / dt_;

    // 최종: tightening
    l_block(i) = std::max(a_pos_lb_(i), a_vel_lb_(i));
    u_block(i) = std::min(a_pos_ub_(i), a_vel_ub_(i));

    // Inside the position-margin band the position bound crosses the opposite
    // velocity bound (l > u): one empty row makes the whole QP primal
    // infeasible (#551). Collapse the row onto the velocity bound it crossed —
    // the repair ClikReferenceGenerator::AssembleBox applies at velocity
    // level — so the box stays non-empty and the QP is asked to reverse the
    // joint toward the range at its velocity limit in one tick. Continuous at
    // the crossing (there u = pos_ub = vel_lb); rows that did not cross are
    // untouched. Both sides cannot cross at once since Init enforces
    // q_min_ <= q_max_. Non-finite q / v take the unrepaired path on purpose:
    // the row stays non-finite for the solve to fail on, instead of being
    // laundered into a finite box by std::max/min (NUM-7).
    if (std::isfinite(qi) && std::isfinite(vi) && l_block(i) > u_block(i)) {
      ++inverted;
      // Upper band: pos_ub < vel_lb. Lower band: pos_lb > vel_ub.
      const double pinned = (a_pos_ub_(i) < a_vel_lb_(i)) ? a_vel_lb_(i) : a_vel_ub_(i);
      l_block(i) = pinned;
      u_block(i) = pinned;
    }
  }
  inverted_count_ = inverted;
}

}  // namespace rtc::tsid
