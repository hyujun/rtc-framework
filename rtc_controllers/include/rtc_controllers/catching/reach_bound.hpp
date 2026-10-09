// ── How far the catch frame can ever be from the arm (L3 §4.1) ───────────────
//
// A necessary condition the searches test BEFORE the catch-pose IK: a target
// farther from the arm than its kinematics can ever put the catch frame cannot
// be reached, whatever the IK would find, so the IK is not run on it. The bound
// comes from the model alone — no key names it, no robot's number is written.
//
// THE BOUND. Along the kinematic chain from the first joint to the catch frame,
// pick one point c_i on every revolute joint's axis. Such a point is fixed in
// the link before the joint AND in the link after it, so the distance between
// consecutive points is a constant of the model, and by the triangle inequality
//
//     ‖p_frame − c_1‖ ≤ Σ_i ‖c_{i+1} − c_i‖ + ‖p_frame − c_n‖     for every q.
//
// ANY choice of points gives a valid bound; the one that minimises the sum is
// the tightest of the family and is found by iteratively reweighted least
// squares over where each point sits on its axis (a convex problem). The
// iteration only tightens: its start (the joint origins) is already a bound,
// so stopping it early never breaks the condition. A prismatic joint has no
// point fixed in both of its links; its origin stands for it and its stroke
// (the largest |q| its limits allow) is added to the sum. A chain that holds a
// joint of any other kind, or a prismatic joint without limits, gives no
// bound: the radius is infinite and the filter passes everything.
//
// The centre is c_1 — the chosen point on the first joint's axis — in the model
// world. Non-RT: computed once at configure time (allocates).
#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>

#include <cmath>
#include <limits>
#include <string>

namespace rtc::catching {

/// The reach bound of one frame of a model (header note).
struct ReachBound {
  Eigen::Vector3d centre{Eigen::Vector3d::Zero()};         ///< c_1 in the model world [m]
  double radius{std::numeric_limits<double>::infinity()};  ///< [m]; +∞ = no bound
  int joints{0};  ///< joints on the chain from the universe to the frame
  /// The joint that left the chain without a bound (`radius` infinite), or
  /// empty.
  std::string unbounded_by;

  [[nodiscard]] bool Bounded() const noexcept { return std::isfinite(radius); }
};

/// @brief The reach bound of `frame` (header note). `iterations` bounds the
///        reweighted least-squares passes; 0 keeps the joint origins.
/// @throws std::invalid_argument an unknown frame index
[[nodiscard]] ReachBound ComputeReachBound(const pinocchio::Model& model,
                                           pinocchio::FrameIndex frame, int iterations = 200);

/// @brief Whether `p` (model world) may be within the frame's reach: inside
///        the sphere of `bound.radius + tolerance` about its centre, or no
///        bound. A non-finite `p` is not within reach. RT-safe.
[[nodiscard]] inline bool WithinReach(const ReachBound& bound, const Eigen::Vector3d& p,
                                      double tolerance) noexcept {
  if (!bound.Bounded()) {
    return true;
  }
  const double d = (p - bound.centre).norm();
  return d <= bound.radius + tolerance;  // NaN fails the comparison: not within reach
}

}  // namespace rtc::catching
