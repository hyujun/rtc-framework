#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Forward kinematics of a REFERENCE (q_des, v_des, a_des) — not of the
// measured state — for a fixed set of frames: what a frame-level task is told
// to track when the reference is a whole-body configuration (an MPC plan).
//
// For each frame: the world placement oMf, the LOCAL_WORLD_ALIGNED twist
// [linear; angular] and the LOCAL_WORLD_ALIGNED *classical* acceleration
// (the a_ff an SE3Task adds to J·a + J̇v); optionally the centre of mass and
// its velocity / acceleration. The reference's base twist is pinocchio's
// LOCAL convention; the LOCAL → world transport closes here.
//
// Own pinocchio::Data (the measured-state cache is not touched — it holds the
// measured state's kinematics and its drift at a = 0, not a reference's). Update is
// RT-safe (no allocation, noexcept): sizes are checked before pinocchio is
// called, the free-flyer quaternion is normalised in an own copy, and a
// non-finite input, a quaternion whose norm is ≤ 1e-9 or not finite, or a
// result that came out non-finite returns false with the last result kept.
// Only the free-flyer root's quaternion is normalised (a model's other
// spherical joints are passed through).
// ────────────────────────────────────────────────
class ReferenceKinematics {
 public:
  using Vec6 = Eigen::Matrix<double, 6, 1>;

  // Init time (allocates): the frames to evaluate, in the order the accessors use.
  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
            const std::vector<pinocchio::FrameIndex>& frames, bool compute_com);

  [[nodiscard]] bool Update(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                            const Eigen::VectorXd& a) noexcept;

  [[nodiscard]] std::size_t size() const noexcept { return frames_.size(); }

  [[nodiscard]] const pinocchio::SE3& Placement(std::size_t i) const noexcept { return oMf_[i]; }

  [[nodiscard]] const Vec6& Twist(std::size_t i) const noexcept { return twist_[i]; }

  [[nodiscard]] const Vec6& Acceleration(std::size_t i) const noexcept { return acc_[i]; }

  [[nodiscard]] const Eigen::Vector3d& Com() const noexcept { return com_; }

  [[nodiscard]] const Eigen::Vector3d& ComVelocity() const noexcept { return vcom_; }

  [[nodiscard]] const Eigen::Vector3d& ComAcceleration() const noexcept { return acom_; }

  // true once an Update succeeded (the accessors hold its result).
  [[nodiscard]] bool valid() const noexcept { return valid_; }

 private:
  const pinocchio::Model* model_{nullptr};
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::FrameIndex> frames_;
  std::vector<pinocchio::SE3> oMf_;
  std::vector<Vec6> twist_;
  std::vector<Vec6> acc_;
  // The candidate result of an Update, swapped in once it is finite.
  std::vector<pinocchio::SE3> oMf_next_;
  std::vector<Vec6> twist_next_;
  std::vector<Vec6> acc_next_;
  Eigen::VectorXd q_;  // normalised copy of the reference configuration
  Eigen::Vector3d com_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vcom_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acom_{Eigen::Vector3d::Zero()};
  int n_base_q_{0};
  bool compute_com_{false};
  bool valid_{false};
};

}  // namespace rtc::tsid
