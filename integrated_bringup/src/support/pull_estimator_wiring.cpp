#include "integrated_bringup/support/pull_estimator_wiring.hpp"

#include "rtc_urdf_bridge/types.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace integrated_bringup {

namespace {
/// Below this norm a direction vector is degenerate — normalizing it would
/// amplify noise or divide by zero. Mirrors the estimator core's own guard.
constexpr double kDegenerateNorm = 1e-9;
/// |cos| above this counts as "parallel" for the P-17 configure-time warning —
/// the residual perpendicular component is then below the estimator's own
/// basis-collapse threshold (1e-3), so the reference axis is unusable.
constexpr double kParallelCosMin = 1.0 - 1e-6;
}  // namespace

std::vector<std::string> ResolvePullTipLinks(const rtc_urdf_bridge::ModelConfig* sys_model,
                                             const std::string& secondary_device,
                                             std::size_t max_slots) {
  std::vector<std::string> links;
  if (sys_model == nullptr) {
    return links;
  }
  for (const auto& tm : sys_model->tree_models) {
    if (tm.name == secondary_device) {
      const std::size_t n_links = std::min(tm.tip_links.size(), max_slots);
      links.assign(tm.tip_links.begin(),
                   tm.tip_links.begin() + static_cast<std::ptrdiff_t>(n_links));
      break;
    }
  }
  return links;
}

void ConfigurePullEstimatorWiring(const DemoSharedConfig& cfg, double control_rate_hz,
                                  std::span<const std::string> link_names, PullEstimatorWiring& w) {
  w.estimator.reset();
  w.num_contacts = 0;
  w.slot.fill(-1);
  w.thumb_contact = -1;
  w.roles.clear();
  w.position_valid.fill(false);
  w.prev_grasp_detected = false;
  w.opposing_mask = 0;

  if (link_names.empty()) {
    // No FK-backed slots to wire against (hand-less / unit-test paths):
    // stay disabled rather than fail the whole controller configure.
    return;
  }

  BuildPullForceEstimator(cfg, control_rate_hz, w.estimator);
  if (!w.estimator) {
    return;
  }

  // Every throw below leaves the wiring disabled rather than "enabled with
  // zero contacts": w.num_contacts is still 0 here, so a caller that logged
  // and continued would otherwise drive Update() with an empty input span.
  const auto fail = [&w](const std::string& what) {
    w.estimator.reset();
    throw std::runtime_error(what);
  };

  const int n = w.estimator->num_contacts();
  int n_opposing_configured = 0;
  for (int k = 0; k < n; ++k) {
    const auto idx = static_cast<std::size_t>(k);
    const std::string& role = cfg.pull_tip_roles[idx];
    for (int j = 0; j < k; ++j) {
      if (cfg.pull_tip_roles[static_cast<std::size_t>(j)] == role) {
        // Roles key the per-tip config and pick the thumb, so a repeat resolves
        // both entries to the same link/slot and silently doubles one tip in
        // the sum — and a repeated "thumb" would leave the axis identically
        // zero (last-wins thumb vs. its own duplicate as the opposing tip).
        fail("pull_estimator: tips.tip_names has a repeated role '" + role +
             "' — each role names one tip and keys its own config block");
      }
    }
    const std::string& link = cfg.pull_tip_links[idx];
    int resolved = -1;
    for (std::size_t j = 0; j < link_names.size(); ++j) {
      if (link_names[j] == link) {
        resolved = static_cast<int>(j);
        break;
      }
    }
    if (resolved < 0) {
      // Silent mis-wiring would feed another fingertip's force/rotation into
      // the sum — hard-fail like the required_roles whitelist (P2 precedent).
      fail("pull_estimator: tips." + role + ".link '" + link +
           "' does not match any FK-backed fingertip link");
    }
    w.slot[idx] = resolved;
    if (role == "thumb") {
      w.thumb_contact = k;
    } else {
      ++n_opposing_configured;
    }
  }

  if (w.thumb_contact < 0) {
    // The per-contact gate normal is derived from pinch topology (thumb is the
    // opposing finger, ±plane normal), so a 'thumb' role is mandatory for any
    // plane_normal_source — not just pinch_geometry.
    fail(
        "pull_estimator: a 'thumb' role in tips.tip_names is required (per-contact normal is "
        "derived from the thumb-opposition axis)");
  }
  if (cfg.pull_plane_normal_source == PullPlaneNormalSource::kPinchGeometry &&
      n_opposing_configured == 0) {
    // Thumb alone can never form an opposing pair: the opposing set stays empty
    // forever, the pinch axis is degenerate and every tick decays to invalid.
    // Fail loudly instead of publishing a permanently dead estimator. A fixed
    // normal does not consult the opposing set, so it is exempt.
    fail(
        "pull_estimator: plane_normal_source 'pinch_geometry' needs at least one non-thumb tip "
        "in tips.tip_names to oppose the thumb");
  }

  // The fixed normal is copied straight from YAML and then divided by its own
  // norm every tick, so a non-finite or zero vector would turn every estimate
  // into a permanently invalid (or NaN) one at runtime with no configure-time
  // signal — the same silent-misconfiguration class as the link resolve above
  // (#234, cross-review §3). Only meaningful for the fixed source; pinch
  // geometry derives its own normal and ignores this field.
  if (cfg.pull_plane_normal_source == PullPlaneNormalSource::kFixed) {
    if (!cfg.pull_plane_normal.allFinite() || cfg.pull_plane_normal.norm() < kDegenerateNorm) {
      fail("pull_estimator: plane_normal_source 'fixed' requires a finite, non-zero plane_normal");
    }
  }

  w.num_contacts = n;
  w.roles.assign(cfg.pull_tip_roles.begin(),
                 cfg.pull_tip_roles.begin() + static_cast<std::ptrdiff_t>(n));
  w.normal_source = cfg.pull_plane_normal_source;
  w.fixed_normal = cfg.pull_plane_normal;
  w.use_baseline = cfg.pull_use_baseline_subtraction;
}

void LogPullEstimatorWiring(const rclcpp::Logger& logger, const PullEstimatorWiring& w,
                            const DemoSharedConfig& cfg) {
  if (!w.enabled()) {
    // Not an error: a hand-less variant, a disabled YAML block, or a model with
    // no tree-model match all land here deliberately. Say so out loud anyway —
    // the silent version is indistinguishable from "running but never valid".
    // Name the CSV explicitly: a missing pull_estimator.csv has three causes
    // (enabled=false / no tip links / the log-registration skip) and the INFO
    // used to mention none of them, so an empty session directory read as a
    // logging bug (#234 P-20).
    RCLCPP_INFO(logger,
                "[pull_estimator] disabled — block %s, %s. No pull estimate will be published and "
                "no pull_estimator.csv will be written.",
                cfg.has_pull_estimator_block ? "present" : "absent",
                cfg.pull_estimator_enabled ? "no FK-backed tip links resolved" : "enabled=false");
    return;
  }

  std::string tips;
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto idx = static_cast<std::size_t>(k);
    if (!tips.empty()) {
      tips += ", ";
    }
    tips += cfg.pull_tip_roles[idx];
    tips += "->";
    tips += cfg.pull_tip_links[idx];
    tips += "(slot ";
    tips += std::to_string(w.slot[idx]);
    tips += ")";
  }
  RCLCPP_INFO(logger,
              "[pull_estimator] enabled — %d contacts [%s], thumb=contact %d, normal_source=%s, "
              "baseline_subtraction=%s",
              w.num_contacts, tips.c_str(), w.thumb_contact,
              PullPlaneNormalSourceName(w.normal_source), w.use_baseline ? "on" : "off");

  // A fixed normal parallel to the in-plane reference is a silently degenerate
  // pair (#234 P-17): the reference projects to nothing, MakePlaneBasis falls
  // back to a world-axis seed, and force_inplane[0] stops meaning "along the
  // reference" without anything saying so. It is not an error — the estimate
  // stays correct and basis_source now reports SEED — but the shipped configs
  // all default both to +Z, so flipping plane_normal_source to "fixed" walks
  // straight into it.
  if (w.normal_source == PullPlaneNormalSource::kFixed) {
    const double n_norm = w.fixed_normal.norm();
    const double r_norm = cfg.pull_estimator_params.inplane_x_reference.norm();
    if (n_norm >= kDegenerateNorm && r_norm >= kDegenerateNorm) {
      const double cos_angle = std::abs(
          w.fixed_normal.dot(cfg.pull_estimator_params.inplane_x_reference) / (n_norm * r_norm));
      if (cos_angle > kParallelCosMin) {
        RCLCPP_WARN(logger,
                    "[pull_estimator] plane_normal is parallel to inplane_x_reference — the "
                    "in-plane x axis collapses and falls back to a world-axis seed, so "
                    "force_inplane[0] is no longer the reference-direction component. Set "
                    "inplane_x_reference to a direction lying in the plane.");
      }
    }
  }
}

void ResetPullEstimatorRtState(PullEstimatorWiring& w) noexcept {
  w.prev_grasp_detected = false;
  w.opposing_mask = 0;
  w.position_valid.fill(false);
  for (auto& in : w.inputs) {
    in.valid = false;
  }
  if (w.estimator) {
    w.estimator->ResetRtState();
  }
}

void StageEstopPullTick(PullEstimatorWiring& w, double dt,
                        rtc::grasp::PullEstimateData& out) noexcept {
  if (!w.enabled()) {
    return;
  }
  // Invalidate rather than re-stage: Update() with an all-invalid input span and
  // a zero normal is the estimator's own degenerate-geometry path, so the tick
  // decays through the same code the transient-contact-loss path uses. The
  // contact/touch latches clear with it, which is what a resumed loop should
  // find — ResetPullEstimatorRtState does the same on activate.
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto idx = static_cast<std::size_t>(k);
    w.inputs[idx].valid = false;
    w.inputs[idx].contact_normal.setZero();
    w.position_valid[idx] = false;
  }
  w.opposing_mask = 0;
  // The baseline edge detector is fed `false` so an E-STOP that spans a grasp
  // release/re-grasp cannot arm against a snapshot taken while stopped.
  w.prev_grasp_detected = false;
  const rtc::grasp::PullEstimate& est =
      w.estimator->Update(std::span<const rtc::grasp::PullContactInput>(
                              w.inputs.data(), static_cast<std::size_t>(w.num_contacts)),
                          Eigen::Vector3d::Zero(), dt);
  rtc::grasp::FillPullEstimateData(est, out);
}

const rtc::grasp::PullEstimate& UpdatePullEstimator(PullEstimatorWiring& w, bool grasp_detected,
                                                    double dt) noexcept {
  // ── Which tips actually oppose the thumb this tick ────────────────────────
  // The grasp shape is an observation, not a config constant: thumb+index,
  // thumb+middle, tripod and four-finger all fall out of the same rule.
  //
  // The selector is touch_active — a hysteresis on |f_obj|, which does not
  // depend on the plane normal. Selecting on contact_active instead would gate
  // the axis on f_n = -n.f_obj and thus on the axis itself; that loop has no
  // cold-start state and the force hysteresis cannot damp it, because its ON
  // and OFF tests would be evaluated against two different axes. Open-loop
  // selection costs a one-tick lag (touch_active reflects the previous
  // Update) and one invalid tick at grasp onset, and cannot deadlock.
  w.opposing_mask = 0;
  Eigen::Vector3d opposing_sum = Eigen::Vector3d::Zero();
  int n_opposing = 0;
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto idx = static_cast<std::size_t>(k);
    // touch_active is false whenever the sensor was invalid or saturated, so it
    // already subsumes inputs[k].valid; position_valid covers the geometry the
    // centroid needs. In the WBC consumer both derive from the TSID contact
    // cache (that is where fingertip poses come from there), in joint/task from
    // hand FK — the rule is the same, its inputs differ per controller.
    if (k == w.thumb_contact || !w.position_valid[idx] || !w.estimator->touch_active(k)) {
      continue;
    }
    opposing_sum += w.positions[idx];
    ++n_opposing;
    w.opposing_mask = static_cast<std::uint8_t>(w.opposing_mask | (1U << idx));
  }

  // Plane normal for this tick. Zero vector ⇒ Update flags the tick invalid
  // and applies bounded decay — the deliberate degenerate-geometry path, which
  // is also how "nothing is touching yet" is reported (no provisional axis is
  // invented, so no sample is published against a guessed plane).
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  if (w.normal_source == PullPlaneNormalSource::kFixed) {
    normal = w.fixed_normal;
  } else if (n_opposing > 0 && w.thumb_contact >= 0 &&
             w.position_valid[static_cast<std::size_t>(w.thumb_contact)]) {
    // Pinch geometry: n = (centroid of the opposing tips − p_thumb); Update
    // normalizes. A single opposing tip degrades to that tip's position.
    normal = opposing_sum / static_cast<double>(n_opposing) -
             w.positions[static_cast<std::size_t>(w.thumb_contact)];
  }

  // Per-contact gate/diagnostic normal = signed pinch-plane normal: the thumb's
  // outward object normal points opposite the opposing fingers'. This tracks the
  // grasp axis (FK), not a body-fixed fingertip axis — correct for hemispherical
  // tips whose contact point migrates. Degenerate normal (missing thumb/opposing
  // positions) ⇒ zero, and Update skips those contacts.
  const double gate_norm = normal.norm();
  const Eigen::Vector3d n_hat = (gate_norm >= kDegenerateNorm) ? Eigen::Vector3d(normal / gate_norm)
                                                               : Eigen::Vector3d::Zero();
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto idx = static_cast<std::size_t>(k);
    w.inputs[idx].contact_normal = (k == w.thumb_contact) ? Eigen::Vector3d(-n_hat) : n_hat;
  }

  // Baseline snapshot arms on grasp establishment (rising edge) — captures on
  // the next valid tick inside the estimator, removing in-plane gravity and
  // constant calibration residual. Re-grasping re-arms automatically.
  //
  // Armed on the edge itself, with no deferral: the estimator stores the
  // unprojected reference wrench and re-projects it onto whatever plane each
  // tick uses, so an axis that is still settling (or that later rotates with
  // the grasp shape) no longer bakes a stale plane into the snapshot. Holding
  // the arm instead would let it fire an unbounded time later, capturing a real
  // pull as the baseline.
  if (w.use_baseline && grasp_detected && !w.prev_grasp_detected) {
    w.estimator->ArmBaseline();
  }
  w.prev_grasp_detected = grasp_detected;

  return w.estimator->Update(std::span<const rtc::grasp::PullContactInput>(
                                 w.inputs.data(), static_cast<std::size_t>(w.num_contacts)),
                             normal, dt);
}

}  // namespace integrated_bringup
