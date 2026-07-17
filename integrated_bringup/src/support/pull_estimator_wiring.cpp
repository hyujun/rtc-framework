#include "integrated_bringup/support/pull_estimator_wiring.hpp"

#include "rtc_urdf_bridge/types.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace integrated_bringup {

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
  w.position_valid.fill(false);
  w.prev_grasp_detected = false;

  if (link_names.empty()) {
    // No FK-backed slots to wire against (hand-less / unit-test paths):
    // stay disabled rather than fail the whole controller configure.
    return;
  }

  BuildPullForceEstimator(cfg, control_rate_hz, w.estimator);
  if (!w.estimator) {
    return;
  }

  const int n = w.estimator->num_contacts();
  for (int k = 0; k < n; ++k) {
    const auto idx = static_cast<std::size_t>(k);
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
      throw std::runtime_error("pull_estimator: tips." + cfg.pull_tip_roles[idx] + ".link '" +
                               link + "' does not match any FK-backed fingertip link");
    }
    w.slot[idx] = resolved;
    if (cfg.pull_tip_roles[idx] == "thumb") {
      w.thumb_contact = k;
    }
  }

  if (w.thumb_contact < 0) {
    // The per-contact gate normal is derived from pinch topology (thumb is the
    // opposing finger, ±plane normal), so a 'thumb' role is mandatory for any
    // plane_normal_source — not just pinch_geometry.
    throw std::runtime_error(
        "pull_estimator: a 'thumb' role in tips.tip_names is required (per-contact normal is "
        "derived from the thumb-opposition axis)");
  }

  w.num_contacts = n;
  w.normal_source = cfg.pull_plane_normal_source;
  w.fixed_normal = cfg.pull_plane_normal;
  w.use_baseline = cfg.pull_use_baseline_subtraction;
}

const rtc::grasp::PullEstimate& UpdatePullEstimator(PullEstimatorWiring& w, bool grasp_detected,
                                                    double dt) noexcept {
  // Plane normal for this tick. Zero vector ⇒ Update flags the tick invalid
  // and applies bounded decay — the deliberate degenerate-geometry path.
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  if (w.normal_source == PullPlaneNormalSource::kFixed) {
    normal = w.fixed_normal;
  } else if (w.thumb_contact >= 0 && w.position_valid[static_cast<std::size_t>(w.thumb_contact)]) {
    // Pinch geometry: n = (midpoint of non-thumb tips − p_thumb); Update
    // normalizes. A single opposing tip degrades to that tip's position.
    Eigen::Vector3d opposing = Eigen::Vector3d::Zero();
    int n_opposing = 0;
    for (int k = 0; k < w.num_contacts; ++k) {
      const auto idx = static_cast<std::size_t>(k);
      if (k == w.thumb_contact || !w.position_valid[idx]) {
        continue;
      }
      opposing += w.positions[idx];
      ++n_opposing;
    }
    if (n_opposing > 0) {
      normal = opposing / static_cast<double>(n_opposing) -
               w.positions[static_cast<std::size_t>(w.thumb_contact)];
    }
  }

  // Per-contact gate/diagnostic normal = signed pinch-plane normal: the thumb's
  // outward object normal points opposite the opposing fingers'. This tracks the
  // grasp axis (FK), not a body-fixed fingertip axis — correct for hemispherical
  // tips whose contact point migrates. Degenerate normal (missing thumb/opposing
  // positions) ⇒ zero, and Update skips those contacts.
  const double gate_norm = normal.norm();
  const Eigen::Vector3d n_hat =
      (gate_norm >= 1e-9) ? Eigen::Vector3d(normal / gate_norm) : Eigen::Vector3d::Zero();
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto idx = static_cast<std::size_t>(k);
    w.inputs[idx].contact_normal = (k == w.thumb_contact) ? Eigen::Vector3d(-n_hat) : n_hat;
  }

  // Baseline snapshot arms on grasp establishment (rising edge) — captures on
  // the next valid tick inside the estimator, removing in-plane gravity and
  // constant calibration residual. Re-grasping re-arms automatically.
  if (w.use_baseline && grasp_detected && !w.prev_grasp_detected) {
    w.estimator->ArmBaseline();
  }
  w.prev_grasp_detected = grasp_detected;

  return w.estimator->Update(std::span<const rtc::grasp::PullContactInput>(
                                 w.inputs.data(), static_cast<std::size_t>(w.num_contacts)),
                             normal, dt);
}

}  // namespace integrated_bringup
