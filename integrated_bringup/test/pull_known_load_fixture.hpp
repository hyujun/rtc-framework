// ── pull_known_load_fixture.hpp ──────────────────────────────────────────────
// A pinch grasp carrying a KNOWN external load, driven through the shipped
// iiwa7_leap profile and the production staging path (#177 crit#4, #469 S1).
//
// WHAT MAKES THIS A POSITIVE CONTROL. Quasi-static Newton on the grasped object
// fixes the sum of the finger-on-object forces at −L, where L is the total
// non-contact force the environment puts on it (an applied wrench plus the
// object's weight). The estimator's law is F̂ = −P∥(Σ c_i + m g), and the shipped
// profile sets `gravity_force: [0,0,0]`, so the answer is forced:
//
//     F̂ = −P∥(−L) = +P∥ L
//
// Every term on the right is chosen here, so sign, frame and magnitude are all
// pinned by one comparison. The simulator half of the same seam —
// rtc_mujoco_sim's test_contact_wrench_known_load.cpp — proves that the sim's
// fingertip lane really does carry that L, so the two compose.
//
// THREE THINGS ARE DELIBERATELY NOT AXIS-ALIGNED, and none of them is decoration:
// the pinch normal (2,1,2)/3 has no zero and no repeated component, the load
// (2,−5,3) has three distinct magnitudes, and the three fingertip rotations are
// distinct non-trivial rotations. Identity rotations or a single-axis load let a
// transposed, dropped or permuted frame reproduce the right answer, which is
// exactly the hole the pre-existing fixtures leave.
#pragma once

#include "integrated_bringup/support/pull_estimator_wiring.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

inline constexpr double kPullRateHz = 500.0;
inline constexpr double kPullDt = 1.0 / kPullRateHz;

/// The sim profile — the one whose pull_estimator block is fed by the
/// simulator's contact-wrench lane, and therefore the one carrying the sign
/// flip that turns env-on-link into finger-on-object.
inline constexpr const char* kPullProfile = "iiwa7_leap";

/// tree-model `leap` tip_links order (config/iiwa7_leap sim.yaml) — the
/// fingertip-slot order the demo controllers hand to the wiring at configure.
inline const std::vector<std::string> kPullTipLinks = {"thumb_tip_head", "index_tip_head",
                                                       "middle_tip_head", "ring_tip_head"};

/// Pinch axis. Exactly unit: |(2,1,2)| = 3.
inline const Eigen::Vector3d kPinchNormal = Eigen::Vector3d(2.0, 1.0, 2.0) / 3.0;

/// The known load: the total non-contact force the environment puts on the
/// grasped object, i.e. the L the simulator half measures.
inline const Eigen::Vector3d kLoad(2.0, -5.0, 3.0);

/// Expected estimate = P∥L = L − n(n·L), worked out by hand so the oracle is
/// not a second copy of the implementation's expression:
///   n·L   = (2·2 + 1·(−5) + 2·3) / 3 = 5/3
///   P∥L   = (2,−5,3) − (5/3)(2,1,2)/3 = (2 − 10/9, −5 − 5/9, 3 − 10/9)
inline const Eigen::Vector3d kExpectedInPlane(8.0 / 9.0, -50.0 / 9.0, 17.0 / 9.0);

/// Default grip along the pinch axis [N]. Clears the profile's 0.5 N contact
/// hysteresis at every tip with margin, keeps friction utilisation well under
/// the slip threshold, and stays far below the profile's 25 N saturation gate.
inline constexpr double kPullSqueezeN = 10.0;

/// The wiring reads only `.valid` and `.force[0..2]` off each controller's
/// private FingertipSensorData, so this stand-in is the whole contract. `float`,
/// like the real one — which is why callers use 1e-4 and not 1e-12.
struct FtSample {
  std::array<float, 3> force{};
  bool valid{false};
};

/// Three distinct, non-trivial fingertip orientations (the fourth tip is never
/// in the grasp). Identity rotations here would let a transposed or dropped R_i
/// pass unnoticed.
inline const std::array<Eigen::Matrix3d, 4> kPullTipRotations = {
    Eigen::Matrix3d(
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, -2.0).normalized()).toRotationMatrix()),
    Eigen::Matrix3d(
        Eigen::AngleAxisd(-1.1, Eigen::Vector3d(-2.0, 1.0, 2.0).normalized()).toRotationMatrix()),
    Eigen::Matrix3d(
        Eigen::AngleAxisd(2.0, Eigen::Vector3d(1.0, -1.0, 3.0).normalized()).toRotationMatrix()),
    Eigen::Matrix3d(Eigen::Matrix3d::Identity())};

/// Fingertip positions that make the OBSERVED pinch axis exactly kPinchNormal:
/// the wiring derives n from (centroid of the touching non-thumb tips − p_thumb),
/// so index and middle straddle a point 60 mm from the thumb along it. The
/// straddle direction is orthogonal to the axis — (2,1,2)·(1,−2,0) = 0 — so it
/// cancels out of the centroid exactly.
inline std::array<Eigen::Vector3d, 4> PullTipPositions() {
  const Eigen::Vector3d thumb(0.30, 0.02, 0.15);
  const Eigen::Vector3d centroid = thumb + 0.06 * kPinchNormal;
  const Eigen::Vector3d straddle = 0.02 * Eigen::Vector3d(1.0, -2.0, 0.0).normalized();
  return {thumb, centroid + straddle, centroid - straddle,
          Eigen::Vector3d(0.0, 0.0, 0.0)};  // ring: never touching, never read
}

/// Which fingertip slots have a usable pose. The ring is out of the grasp.
inline std::array<bool, 4> PullPoseValid() {
  return {true, true, true, false};
}

/// Parse the shipped profile as deployed. `config_dir` is the caller target's
/// RTC_DEMO_SHARED_CONFIG_DIR — passed rather than baked in so this header does
/// not depend on a macro every including target must remember to define.
inline DemoSharedConfig LoadShippedPullProfile(const std::string& config_dir) {
  const std::string path = config_dir + "/" + kPullProfile + "/controllers/demo_shared.yaml";
  const YAML::Node root = YAML::LoadFile(path);
  DemoSharedConfig cfg;
  ApplyDemoSharedConfig(root["demo_shared"], cfg);
  return cfg;
}

/// The finger-on-object forces of a three-finger pinch carrying `load`.
///
/// Newton fixes only their SUM (= −load); the split into a grip part and a
/// share of the load is this fixture's choice and any split clearing the gates
/// would do. The thumb pushes along +n and the two opposing tips share −n, so
/// the grip cancels out of the sum and only the load survives.
inline std::array<Eigen::Vector3d, 3> PullPinchForces(const Eigen::Vector3d& load,
                                                      double squeeze = kPullSqueezeN) {
  const Eigen::Vector3d share = load / 3.0;
  return {squeeze * kPinchNormal - share, -0.5 * squeeze * kPinchNormal - share,
          -0.5 * squeeze * kPinchNormal - share};
}

/// Turn those into what the SIM LANE publishes for them: env-on-link (the
/// negative of finger-on-object) in each fingertip's own frame. This is the
/// step the shipped `force_sign: -1.0` has to undo.
inline std::array<FtSample, 4> PullLaneSamples(const Eigen::Vector3d& load,
                                               double squeeze = kPullSqueezeN) {
  const std::array<Eigen::Vector3d, 3> contact = PullPinchForces(load, squeeze);
  std::array<FtSample, 4> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    const Eigen::Vector3d link = kPullTipRotations[i].transpose() * (-contact[i]);
    out[i].force = {static_cast<float>(link.x()), static_cast<float>(link.y()),
                    static_cast<float>(link.z())};
    out[i].valid = true;
  }
  out[3].valid = false;  // ring is not in this grasp
  return out;
}

/// Run `ticks` updates through the production staging path with the given lane.
///
/// Several ticks are needed whatever the assertion: the pinch axis comes from
/// the previous tick's touch hysteresis (a one-tick lag, by design) and
/// force_filtered sits behind the profile's 5 Hz Bessel.
inline const rtc::grasp::PullEstimate& RunPullTicks(PullEstimatorWiring& w,
                                                    const std::array<FtSample, 4>& samples,
                                                    bool grasp_detected, int ticks) {
  const std::array<Eigen::Vector3d, 4> positions = PullTipPositions();
  const std::array<bool, 4> pose_valid = PullPoseValid();
  rtc::grasp::PullEstimateData out{};
  for (int t = 0; t < ticks; ++t) {
    StageFkPullTickAndPublish<FtSample>(
        w, std::span<const FtSample>(samples), std::span<const Eigen::Matrix3d>(kPullTipRotations),
        std::span<const Eigen::Vector3d>(positions), std::span<const bool>(pose_valid),
        /*num_active_fingertips=*/4, grasp_detected, kPullDt, out);
  }
  return w.estimator->estimate();
}

/// The common case: settle the estimator on a lane carrying `load`.
inline const rtc::grasp::PullEstimate& RunPullLoad(PullEstimatorWiring& w,
                                                   const Eigen::Vector3d& load, bool grasp_detected,
                                                   int ticks, double squeeze = kPullSqueezeN) {
  return RunPullTicks(w, PullLaneSamples(load, squeeze), grasp_detected, ticks);
}

}  // namespace integrated_bringup::testfx
