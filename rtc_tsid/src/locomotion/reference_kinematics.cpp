#include "rtc_tsid/locomotion/reference_kinematics.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <stdexcept>

namespace rtc::tsid {

void ReferenceKinematics::Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                               const std::vector<pinocchio::FrameIndex>& frames, bool compute_com) {
  for (const auto f : frames) {
    if (f >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
      throw std::invalid_argument("ReferenceKinematics: a frame index is not in the model");
    }
  }
  model_ = &model;
  data_ = std::make_unique<pinocchio::Data>(model);
  frames_ = frames;
  oMf_.assign(frames.size(), pinocchio::SE3::Identity());
  twist_.assign(frames.size(), Vec6::Zero());
  acc_.assign(frames.size(), Vec6::Zero());
  oMf_next_ = oMf_;
  twist_next_ = twist_;
  acc_next_ = acc_;
  q_.setZero(model.nq);
  n_base_q_ = robot_info.floating_base ? robot_info.n_base_q : 0;
  compute_com_ = compute_com;
  valid_ = false;
}

bool ReferenceKinematics::Update(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                 const Eigen::VectorXd& a) noexcept {
  if (model_ == nullptr || q.size() != model_->nq || v.size() != model_->nv ||
      a.size() != model_->nv || !q.allFinite() || !v.allFinite() || !a.allFinite()) {
    return false;
  }
  q_ = q;
  if (n_base_q_ == 7) {  // free-flyer [p, q_xyzw]: normalise the quaternion
    const double n = q_.segment<4>(3).norm();
    if (!(std::isfinite(n) && n > 1e-9)) {  // an overflowing norm is not a unit quaternion
      return false;
    }
    q_.segment<4>(3) /= n;
  }
  pinocchio::forwardKinematics(*model_, *data_, q_, v, a);
  bool finite = true;
  for (std::size_t i = 0; i < frames_.size(); ++i) {
    oMf_next_[i] = pinocchio::updateFramePlacement(*model_, *data_, frames_[i]);
    const pinocchio::Motion vel =
        pinocchio::getFrameVelocity(*model_, *data_, frames_[i], pinocchio::LOCAL_WORLD_ALIGNED);
    const pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
        *model_, *data_, frames_[i], pinocchio::LOCAL_WORLD_ALIGNED);
    twist_next_[i].head<3>() = vel.linear();
    twist_next_[i].tail<3>() = vel.angular();
    acc_next_[i].head<3>() = acc.linear();
    acc_next_[i].tail<3>() = acc.angular();
    finite = finite && oMf_next_[i].translation().allFinite() &&
             oMf_next_[i].rotation().allFinite() && twist_next_[i].allFinite() &&
             acc_next_[i].allFinite();
  }
  if (compute_com_) {
    // The kinematics are those of the pass above (no second FK).
    pinocchio::centerOfMass(*model_, *data_, pinocchio::ACCELERATION, false);
    finite = finite && data_->com[0].allFinite() && data_->vcom[0].allFinite() &&
             data_->acom[0].allFinite();
  }
  if (!finite) {  // large finite input can overflow inside the kinematics
    return false;
  }
  oMf_.swap(oMf_next_);
  twist_.swap(twist_next_);
  acc_.swap(acc_next_);
  if (compute_com_) {
    com_ = data_->com[0];
    vcom_ = data_->vcom[0];
    acom_ = data_->acom[0];
  }
  valid_ = true;
  return true;
}

}  // namespace rtc::tsid
