// ── RT sampler of the decel MPC's joint nodes (MPC plan E1-F02, #628) ────────
//
// The planner publishes the stop segment as joint NODES only (MD-9) in a
// DecelPlanSnapshot (trajectory.hpp). This class is the RT side of that
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
// satisfied by v* = q̇_ref, but the CLIK posture term is k_a(q_des − q) with
// no velocity feedforward. The FK consistency above is what this sampler
// guarantees; the RT caller feeds q̇_ref into the posture task by handing it
// q_ref + q̇_ref / k_a as the goal (MD-36).
//
// WIRING. The catching controller under `supervisor.decel.mode: mpc`:
// Sample() on every tick that follows a segment — APPROACH through HOLD
// (E1-F09) — and NodesInsideBox() once per admitted segment (MD-43).
//
// RT CONTRACT. Init() is non-RT (copies nothing heavy: shares the model, owns
// its own pinocchio::Data, sizes every buffer). Sample() is noexcept,
// allocates nothing, and is fail-closed — on failure `out` is untouched. It
// checks the payload's SHAPE against its own (so it can never index out of
// bounds) but not node VALUES: the caller runs ValidateDecelNodes once per new
// payload (by decel_seq), exactly as the jerk segment contract expects.
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
struct DecelNodeSample {
  std::array<double, kMaxDecelNv> q{};
  std::array<double, kMaxDecelNv> qd{};
  std::array<double, kMaxDecelNv> qdd{};
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
  ///        (the planner's catch sub-model), nq == nv ≤ kMaxDecelNv
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
  [[nodiscard]] bool Sample(const DecelPlanSnapshot& plan, std::int64_t t_lead_ns,
                            DecelNodeSample& out) noexcept;

  /// @brief Joint-only evaluation, device order, no FK (RT-safe, stateless).
  /// Same shape checks and failure rule as Sample(); `held` reports t past
  /// node N. Used by the planner to read a published segment back. A segment
  /// with pre-catch nodes (n_pre > 0) is evaluated at dt_pre before its catch
  /// node and at dt from it on (DecelNodeTimeNs, trajectory.hpp).
  [[nodiscard]] static bool SampleJoints(const DecelPlanSnapshot& plan, std::int64_t t_lead_ns,
                                         std::span<double> q, std::span<double> qd,
                                         std::span<double> qdd, bool* held = nullptr) noexcept;

  /// @brief Whether the catch frame at every node from `first_node` on lies in
  /// the axis-aligned box [lo, hi] of the model world (MD-43; RT-safe, one FK
  /// per node checked). With an `anchor`, the path is judged TRANSLATED so
  /// node `first_node` sits at the anchor — anchor + (p_k − p_first) — i.e.
  /// the stop's displacement from where it starts, placed at the catch point
  /// the planner checked its own stop from. `first_node` is 0 for a segment
  /// that starts at t_c or after it (n_pre 0) and the catch node (n_pre) for
  /// one that starts before, whose pre-catch nodes are the approach, not the
  /// stop.
  /// Between nodes is not checked. False when uninitialised, on a shape this
  /// arm cannot sample, on a `first_node` outside 0 .. n_nodes, or on the
  /// first node outside (NaN counts as outside); `first_outside` (if given)
  /// is that node, −1 when every checked node is inside.
  [[nodiscard]] bool NodesInsideBox(const DecelPlanSnapshot& plan, const std::array<double, 3>& lo,
                                    const std::array<double, 3>& hi,
                                    const std::array<double, 3>* anchor = nullptr,
                                    int* first_outside = nullptr, int first_node = 0) noexcept;

  /// @brief The catch frame's position at node `k` in the model world (RT-safe,
  /// one FK). Same shape checks as NodesInsideBox(); false — `p` untouched —
  /// on those or a `k` outside 0 .. n_nodes.
  [[nodiscard]] bool NodePosition(const DecelPlanSnapshot& plan, int k,
                                  std::array<double, 3>& p) noexcept;

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex frame_{0};
  int nv_{0};
  std::array<int, kMaxDecelNv> device_of_model_{};
  Eigen::VectorXd q_model_;
  Eigen::VectorXd v_model_;
};

}  // namespace rtc::catching
