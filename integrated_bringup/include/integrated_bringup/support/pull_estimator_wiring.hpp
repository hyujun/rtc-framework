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
//     pinch geometry n = (midpoint of non-thumb tips − p_thumb)), arm the
//     baseline snapshot on a grasp_detected rising edge, run Update.
// Output lands in the controllers' existing owned SeqLock PODs via
// rtc::grasp::FillPullEstimateData — no new topic / PublishRole (#167
// confirmed decision, E-11 avoided).

#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Core>

#include <array>
#include <memory>
#include <span>
#include <string>

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

  [[nodiscard]] bool enabled() const noexcept { return estimator != nullptr; }
};

// Build the estimator via BuildPullForceEstimator and resolve tip links.
// `link_names[j]` names the fingertip link the caller services at slot j —
// pass only slots backed by real force/FK storage (joint/task cap at
// kNumFingertips). Resets `w` first; stays disabled (estimator == nullptr,
// no throw) when the YAML block is absent/disabled OR `link_names` is empty
// (no FK available — unit-test and hand-less paths). Throws
// std::runtime_error when a configured tip link resolves to no slot or when
// pinch_geometry is selected without a "thumb" role (config wiring errors —
// non-RT path, propagates to CallbackReturn::FAILURE like
// BuildPullForceEstimator's std::invalid_argument).
void ConfigurePullEstimatorWiring(const DemoSharedConfig& cfg, double control_rate_hz,
                                  std::span<const std::string> link_names, PullEstimatorWiring& w);

// Per-tick RT update. Caller must have staged w.inputs / w.positions /
// w.position_valid for [0, w.num_contacts) and checked w.enabled().
// `grasp_detected` supplies the baseline arming edge (grasp establishment).
// noexcept, heap-free; a degenerate normal (missing thumb/opposing positions
// in pinch geometry) yields an invalid tick with bounded decay inside Update.
const rtc::grasp::PullEstimate& UpdatePullEstimator(PullEstimatorWiring& w, bool grasp_detected,
                                                    double dt) noexcept;

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_PULL_ESTIMATOR_WIRING_HPP_
