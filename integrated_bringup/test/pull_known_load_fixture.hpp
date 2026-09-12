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
#include <cmath>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

inline constexpr double kPullRateHz = 500.0;
inline constexpr double kPullDt = 1.0 / kPullRateHz;

/// The sim profile — the one whose pull_estimator block is fed by the
/// simulator's contact-wrench lane. It used to carry a per-tip `force_sign:
/// -1.0` turning env-on-link into finger-on-object; the simulator now publishes
/// that convention itself, so what this profile pins is the ABSENCE of an
/// inversion.
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

/// Parse a shipped profile as deployed. `config_dir` is the caller target's
/// RTC_DEMO_SHARED_CONFIG_DIR — passed rather than baked in so this header does
/// not depend on a macro every including target must remember to define.
inline DemoSharedConfig LoadShippedPullProfile(const std::string& config_dir,
                                               const std::string& profile) {
  const std::string path = config_dir + "/" + profile + "/controllers/demo_shared.yaml";
  const YAML::Node root = YAML::LoadFile(path);
  DemoSharedConfig cfg;
  ApplyDemoSharedConfig(root["demo_shared"], cfg);
  return cfg;
}

inline DemoSharedConfig LoadShippedPullProfile(const std::string& config_dir) {
  return LoadShippedPullProfile(config_dir, kPullProfile);
}

/// One shipped robot profile that configures a pull estimator, plus the two
/// facts a caller needs to stand its wiring up.
///
/// `tip_links` is the tree-model fingertip order the demo controllers hand to
/// the wiring at configure; `expected_contacts` is how many of them the
/// profile's own `tip_names` block actually enrols (LEAP enrols its ring, the
/// two p1 hands do not). Both are spelled out rather than derived so that a
/// profile edit that renames a tip or drops a role fails a named assertion
/// instead of quietly changing which finger is under test.
struct PullProfileCase {
  const char* profile;
  std::vector<std::string> tip_links;
  int expected_contacts;
};

inline const std::vector<PullProfileCase>& ShippedPullProfiles() {
  static const std::vector<PullProfileCase> cases = {
      {"iiwa7_leap", {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}, 4},
      {"ur5e_p1a", {"thumb_tip_link", "index_tip_link", "middle_tip_link", "ring_tip_link"}, 3},
      {"ur5e_p1b",
       {"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket", "l_ring_tip_bracket"},
       3},
  };
  return cases;
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

/// A unit direction orthogonal to the pinch axis, used to tilt contact normals
/// off it by a known angle. n·e = 0 exactly: (2,1,2)·(0,2,-1) = 0.
inline const Eigen::Vector3d kPinchTiltDir = Eigen::Vector3d(0.0, 2.0, -1.0).normalized();

/// Finger-on-object forces for a PURE INTERNAL SQUEEZE whose contact normals are
/// tilted off the observed pinch axis by `tilt_rad` — no external load at all.
///
/// This is the construction #177 crit#6 (1) asks about. `alignment_error_rad`
/// declares that the true contact normals sit within delta of the plane normal
/// the estimator is handed, and bounds the resulting grip->in-plane leakage by
/// sum_i |f_n,i| sin(delta). A squeeze that is EXACTLY opposed leaks nothing
/// whatever delta is (the forces cancel in the sum), so it cannot test the
/// bound; what does is a squeeze whose two sides are tilted in OPPOSITE senses,
/// because then the cancellation is incomplete by a known amount:
///
///   thumb      = +S (cos d) n + S (sin d) e
///   two others = -S (cos d) n + S (sin d) e     (split as -0.5 S each)
///   sum        = 2 S sin(d) e,   which is entirely in-plane (e . n = 0)
///
/// and the normal forces the estimator accumulates are S cos d on the thumb and
/// 0.5 S cos d on each of the others, i.e. sum |f_n| = 2 S cos d. So
///
///   |F_hat| / sum|f_n| = tan(d)
///
/// exactly, independent of S — a closed form derived here on paper, not a
/// second copy of the estimator's expression. The test sweeps S and regresses,
/// which is the same slope #177 §S2 measures on hardware.
inline std::array<Eigen::Vector3d, 3> PullTiltedSqueezeForces(double squeeze, double tilt_rad) {
  const Eigen::Vector3d n_plus =
      std::cos(tilt_rad) * kPinchNormal + std::sin(tilt_rad) * kPinchTiltDir;
  const Eigen::Vector3d n_minus =
      std::cos(tilt_rad) * kPinchNormal - std::sin(tilt_rad) * kPinchTiltDir;
  return {squeeze * n_plus, -0.5 * squeeze * n_minus, -0.5 * squeeze * n_minus};
}

/// Turn those into what the SIM LANE publishes for them: link-on-environment —
/// the same sign as finger-on-object — resolved into each fingertip's own
/// frame. Only the frame changes here; there is no sign step left, because
/// rtc_mujoco_sim now publishes the convention the estimator sums (it used to
/// publish env-on-link, and the shipped profile pinned `force_sign: -1.0` to
/// undo it).
inline std::array<FtSample, 4> PullLaneSamplesFrom(const std::array<Eigen::Vector3d, 3>& contact) {
  std::array<FtSample, 4> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    const Eigen::Vector3d link = kPullTipRotations[i].transpose() * contact[i];
    out[i].force = {static_cast<float>(link.x()), static_cast<float>(link.y()),
                    static_cast<float>(link.z())};
    out[i].valid = true;
  }
  out[3].valid = false;  // ring is not in this grasp
  return out;
}

inline std::array<FtSample, 4> PullLaneSamples(const Eigen::Vector3d& load,
                                               double squeeze = kPullSqueezeN) {
  return PullLaneSamplesFrom(PullPinchForces(load, squeeze));
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
