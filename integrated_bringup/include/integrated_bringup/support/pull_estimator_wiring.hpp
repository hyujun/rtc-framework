#ifndef UR5E_BRINGUP_SUPPORT_PULL_ESTIMATOR_WIRING_HPP_
#define UR5E_BRINGUP_SUPPORT_PULL_ESTIMATOR_WIRING_HPP_

// Shared wiring for the in-plane pull-force estimator (#167 P3) — the three
// demo controllers (joint / task / wbc) embed one PullEstimatorWiring each.
// Split of responsibilities:
//   ConfigurePullEstimatorWiring (non-RT, on_configure/LoadConfig): build the
//     estimator from DemoSharedConfig and resolve each configured tip link to
//     the caller's fingertip slot index (joint/task: tree-model tip_links
//     order; wbc: tsid contacts order — 1:1 with the sensor lanes either way).
//   Controller RT tick: stage inputs[k] / positions[k] / position_valid[k]
//     for [0, num_contacts) from its own force + FK (or contact-frame) caches.
//   UpdatePullEstimator (RT): derive the plane normal (fixed YAML vector or
//     pinch geometry), arm the baseline snapshot on a grasp_detected rising
//     edge, run Update.
//
// Pinch geometry is *observed*, not declared: the opposing set is the non-thumb
// tips whose contact is currently established (PullForceEstimator::
// contact_active, previous tick), so thumb+index / thumb+middle / tripod /
// four-finger all yield their own axis without a config switch. `tip_names`
// therefore lists every tip that can physically participate, not the tips a
// particular grasp is expected to use. See UpdatePullEstimator for the
// bootstrap fallback that keeps the first contact latchable.
//
// Output lands in the controllers' existing owned SeqLock PODs via
// rtc::grasp::FillPullEstimateData — no new topic / PublishRole (#167
// confirmed decision, E-11 avoided).

#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace rtc_urdf_bridge {
struct ModelConfig;
}  // namespace rtc_urdf_bridge

namespace integrated_bringup {

struct PullEstimatorWiring {
  // ── Fixed at configure (non-RT) ──
  std::unique_ptr<rtc::grasp::PullForceEstimator> estimator;
  int num_contacts{0};
  /// contact k → caller fingertip slot (index into link_names at configure).
  std::array<int, rtc::grasp::kMaxPullContacts> slot{};
  /// contact index (not slot) whose role is "thumb"; -1 when absent.
  int thumb_contact{-1};
  PullPlaneNormalSource normal_source{PullPlaneNormalSource::kFixed};
  Eigen::Vector3d fixed_normal{Eigen::Vector3d::UnitZ()};
  bool use_baseline{false};

  // ── RT staging (written by the controller each tick, read by Update) ──
  std::array<rtc::grasp::PullContactInput, rtc::grasp::kMaxPullContacts> inputs{};
  /// Per-contact fingertip position in the reference (robot base) frame —
  /// consumed only by the pinch-geometry normal.
  std::array<Eigen::Vector3d, rtc::grasp::kMaxPullContacts> positions{};
  std::array<bool, rtc::grasp::kMaxPullContacts> position_valid{};

  // ── RT state ──
  bool prev_grasp_detected{false};
  /// Baseline arm deferred to the first tick with a contact-derived axis: the
  /// grasp_detected edge (force-threshold, controller side) can land a tick or
  /// two before the estimator's hysteresis latches, and capturing the snapshot
  /// against the provisional bootstrap axis would bias every later sample.
  bool baseline_arm_pending{false};
  /// Bit k set = contact k joined the opposing set this tick (observed grasp
  /// shape; diagnostic + CSV). Zero while `opposing_fallback` is true.
  std::uint8_t opposing_mask{0};
  /// No contact was established yet, so the bootstrap fallback (all FK-valid
  /// non-thumb tips) supplied the axis. The estimate is still gated by the
  /// estimator's own validity rules; this only flags a provisional axis.
  bool opposing_fallback{false};

  [[nodiscard]] bool enabled() const noexcept { return estimator != nullptr; }
};

static_assert(rtc::grasp::kMaxPullContacts <= 8,
              "PullEstimatorWiring::opposing_mask is a uint8 bitmask over contacts");

// Push one pull-estimator log row from the wiring's latest estimate (#167).
// RT tick tail — noexcept, heap-free; a no-op when the CSV channel is
// unregistered or the estimator is disabled, so the three demo controllers
// carry one call instead of the copy-pasted guard + fill + Push block.
inline void PushPullEstimatorLog(rtc::LogHandle<PullEstimatorLogPod>& handle,
                                 const PullEstimatorWiring& wiring, double t_relative_s) noexcept {
  if (!handle || !wiring.enabled()) {
    return;
  }
  PullEstimatorLogPod pod{};
  FillPullEstimatorLogPod(wiring.estimator->estimate(), t_relative_s, pod);
  // Observed grasp shape lives in the wiring (the estimator core is told the
  // axis, not how it was chosen), so it is stamped here rather than in Fill.
  pod.opposing_mask = wiring.opposing_mask;
  pod.opposing_fallback = wiring.opposing_fallback;
  handle.Push(pod);
}

// Build the estimator via BuildPullForceEstimator and resolve tip links.
// `link_names[j]` names the fingertip link the caller services at slot j —
// pass only slots backed by real force/FK storage (joint/task cap at
// kNumFingertips). Resets `w` first; stays disabled (estimator == nullptr,
// no throw) when the YAML block is absent/disabled OR `link_names` is empty
// (no FK available — unit-test and hand-less paths). Throws
// std::runtime_error when a configured tip link resolves to no slot, when no
// "thumb" role is present, or when no non-thumb tip is configured (config
// wiring errors — non-RT path, propagates to CallbackReturn::FAILURE like
// BuildPullForceEstimator's std::invalid_argument).
void ConfigurePullEstimatorWiring(const DemoSharedConfig& cfg, double control_rate_hz,
                                  std::span<const std::string> link_names, PullEstimatorWiring& w);

// Per-tick RT update. Caller must have staged w.inputs / w.positions /
// w.position_valid for [0, w.num_contacts) and checked w.enabled().
// `grasp_detected` supplies the baseline arming edge (grasp establishment).
// noexcept, heap-free; a degenerate normal (thumb FK dropout, or no FK-valid
// opposing tip at all) yields an invalid tick with bounded decay inside Update.
const rtc::grasp::PullEstimate& UpdatePullEstimator(PullEstimatorWiring& w, bool grasp_detected,
                                                    double dt) noexcept;

// Resolve the hand fingertip tip-link names for ConfigurePullEstimatorWiring
// from the tree-model whose name == `secondary_device`, capped at `max_slots`
// (the caller's FK-slot capacity). Returns the tip_links-ordered link list, or
// empty when `sys_model` is null or no tree-model matches (hand-less / model-
// less paths → estimator stays disabled). Non-RT (on_configure / LoadConfig).
[[nodiscard]] std::vector<std::string> ResolvePullTipLinks(
    const rtc_urdf_bridge::ModelConfig* sys_model, const std::string& secondary_device,
    std::size_t max_slots);

// Stage one RT tick of the joint/task FK pull path, run the estimator, and
// mirror the result into `out`. `FtData` is each demo controller's private
// FingertipSensorData (only `.valid` + `.force[0..2]` are read), so this is a
// template rather than a concrete-typed free function. `w.slot[k]` indexes
// `fingertip_data` / `rotations` / `tip_positions` / `pose_valid` alike (tree-
// model tip_links order == fingertip sensor-lane order, Stage A-3 contract).
// Caller must gate on `w.enabled()`. noexcept, heap-free (RT tick path).
template <class FtData>
void StageFkPullTickAndPublish(PullEstimatorWiring& w, std::span<const FtData> fingertip_data,
                               std::span<const Eigen::Matrix3d> rotations,
                               std::span<const Eigen::Vector3d> tip_positions,
                               std::span<const bool> pose_valid, int num_active_fingertips,
                               bool grasp_detected, double dt,
                               rtc::grasp::PullEstimateData& out) noexcept {
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const auto s = static_cast<std::size_t>(w.slot[ki]);
    const auto& ft = fingertip_data[s];
    auto& in = w.inputs[ki];
    const bool sensor_ok = w.slot[ki] < num_active_fingertips && ft.valid &&
                           std::isfinite(ft.force[0]) && std::isfinite(ft.force[1]) &&
                           std::isfinite(ft.force[2]);
    const bool pose_ok = pose_valid[s];
    in.valid = sensor_ok && pose_ok;
    if (in.valid) {
      in.rotation = rotations[s];
      in.force = Eigen::Vector3d(static_cast<double>(ft.force[0]), static_cast<double>(ft.force[1]),
                                 static_cast<double>(ft.force[2]));
    }
    w.positions[ki] = tip_positions[s];
    w.position_valid[ki] = pose_ok;
  }
  const rtc::grasp::PullEstimate& est = UpdatePullEstimator(w, grasp_detected, dt);
  rtc::grasp::FillPullEstimateData(est, out);
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_PULL_ESTIMATOR_WIRING_HPP_
