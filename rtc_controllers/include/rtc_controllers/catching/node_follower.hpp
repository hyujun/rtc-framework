// ── RT sampler of the segment MPC's joint nodes (MPC plan E1-F02, #628) ────────
//
// The planner publishes the stop segment as joint NODES only (MD-9) in a
// SegmentSnapshot (trajectory.hpp). This class is the RT side of that
// payload: at an instant on the lead axis it evaluates the joint reference
// q_ref, q̇_ref, q̈_ref in closed form between nodes (jerk_segment.hpp — exact
// for the MPC's own nodes, C² at every node) and derives the hand's target
// from it by forward kinematics:
//
//   T_d  = T_WC(q_ref)                   (catch frame placement, model world)
//   V_ff = ᵂJ_C(q_ref) q̇_ref             ([v; ω], world-aligned axes)
//
// V_ff comes from velocity forward kinematics (pinocchio getFrameVelocity,
// LOCAL_WORLD_ALIGNED), which IS ᵂJ_C q̇ without forming the Jacobian. The axes
// are the ones ClikReferenceGenerator's twist_ff and PositionAxisTarget use.
//
// WHAT THIS DOES NOT CLAIM (MD-30). With q_c = q_ref the CLIK's hand task is
// satisfied by v* = q̇_ref, and so is its posture term once it is given q̇_ref
// as well. The FK consistency above is what this sampler guarantees; feeding
// q_ref and q̇_ref to the CLIK's posture row (its goal and its velocity
// feed-forward, MD-36) is the RT caller's.
//
// WIRING. The catching controller under `planner.segment.mode: mpc`:
// Sample() on every tick that follows a segment — APPROACH through HOLD
// (E1-F09).
//
// RT CONTRACT. Init() is non-RT (copies nothing heavy: shares the model, owns
// its own pinocchio::Data, sizes every buffer). Sample() is noexcept,
// allocates nothing, and is fail-closed — on failure `out` is untouched. It
// checks the payload's SHAPE against its own (so it can never index out of
// bounds) but not node VALUES: the caller runs ValidateSegmentNodes once per new
// payload (by segment_seq), exactly as the jerk segment contract expects.
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>

namespace rtc::catching {

/// One evaluation of a stop segment. Joint arrays are DEVICE order, first
/// `nv` entries used.
struct SegmentNodeSample {
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  pinocchio::SE3 placement{pinocchio::SE3::Identity()};                    ///< T_WC(q_ref)
  Eigen::Matrix<double, 6, 1> twist{Eigen::Matrix<double, 6, 1>::Zero()};  ///< [v; ω] world axes
  double t_s{0.0};   ///< time since node 0 [s]
  bool held{false};  ///< past the last node: node N held (it is at rest)
};

class NodeTrajectoryFollower {
 public:
  NodeTrajectoryFollower() = default;

  /// @brief Bind the arm model (non-RT).
  /// @param arm the arm's hand-locked model in pinocchio velocity order
  ///        (the planner's catch sub-model), nq == nv ≤ kMaxSegmentNv
  /// @param frame the catch frame
  /// @param device_of_model `device_of_model[m]` = device index of model joint
  ///        m; a permutation of 0..nv−1
  /// @return false (and uninitialised) on a null or unsupported model, an
  ///         unknown frame, or a mapping that is not a permutation.
  [[nodiscard]] bool Init(std::shared_ptr<const pinocchio::Model> arm, pinocchio::FrameIndex frame,
                          std::span<const int> device_of_model);

  [[nodiscard]] bool Initialized() const noexcept { return model_ != nullptr; }

  [[nodiscard]] int Nv() const noexcept { return nv_; }

  /// @brief Evaluate the segment at `t_lead_ns` and run FK (RT-safe).
  /// @return false — `out` untouched — when uninitialised, when the payload's
  ///         shape does not match this arm or its capacities, or when
  ///         `t_lead_ns` is before node 0 (the switch rule keeps the current
  ///         plan until then; a query before node 0 is a caller bug).
  [[nodiscard]] bool Sample(const SegmentSnapshot& plan, std::int64_t t_lead_ns,
                            SegmentNodeSample& out) noexcept;

  /// @brief Joint-only evaluation, device order, no FK (RT-safe, stateless).
  /// Same shape checks and failure rule as Sample(); `held` reports t past
  /// node N. Used by the planner to read a published segment back. A segment
  /// with pre-catch nodes (n_pre > 0) is evaluated at dt_pre before its catch
  /// node and at dt from it on (SegmentNodeTimeNs, trajectory.hpp). One whose
  /// catch interval has its own length (dt_catch_ns ≠ 0) is evaluated in
  /// three parts — dt_pre up to node n_pre − 1, dt_catch from there to the
  /// catch node, dt from it on — and is refused when that length or node 0's
  /// instant is not what ValidateSegmentNodes accepts.
  [[nodiscard]] static bool SampleJoints(const SegmentSnapshot& plan, std::int64_t t_lead_ns,
                                         std::span<double> q, std::span<double> qd,
                                         std::span<double> qdd, bool* held = nullptr) noexcept;

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex frame_{0};
  int nv_{0};
  std::array<int, kMaxSegmentNv> device_of_model_{};
  Eigen::VectorXd q_model_;
  Eigen::VectorXd v_model_;
};

}  // namespace rtc::catching
