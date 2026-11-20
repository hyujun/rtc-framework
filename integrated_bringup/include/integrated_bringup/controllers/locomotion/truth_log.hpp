// The T-tier row of a locomotion_controller tick (LocomotionTruthLogPod) from
// the body group's lanes — what counts as truth, frames and signs are in
// logging/locomotion_truth_log_pod.hpp. A free function so the arithmetic
// (sign flip, CoP, freshness) is testable without a controller.
#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_TRUTH_LOG_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_TRUTH_LOG_HPP_

#include "integrated_bringup/logging/locomotion_truth_log_pod.hpp"
#include "rtc_base/types/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace integrated_bringup::locomotion {

/// The CoP needs a normal force above this share of the robot's weight: the
/// ratio τ / f_z is meaningless near lift-off, and a fraction rather than a
/// newton figure keeps one value for robots of other mass.
inline constexpr double kTruthCopMinWeightFraction = 0.03;

/// The last external wrench a simulator reported (LocomotionTruthLogPod, the
/// truth_ext_* block). Written by the subscription callback, read by the tick.
struct ExternalWrenchSample {
  std::array<double, 6> wrench{};  ///< on the robot [f; τ], world axes [N, N·m]
  std::int64_t recv_steady_ns{0};  ///< receive stamp, the tick's clock
  std::uint64_t sequence{0};       ///< messages received; 0 = none yet
};

static_assert(std::is_trivially_copyable_v<ExternalWrenchSample>,
              "ExternalWrenchSample travels through a TripleBuffer");

/// The tick's inputs besides the lanes.
struct TruthLogInputs {
  std::int64_t now_steady_ns{0};
  /// Freshness bound of both lanes — the state source's body_max_age (> 0).
  std::int64_t max_age_ns{0};
  /// Contacts of the model, in order; contact c is inference group c.
  std::size_t contacts{0};
  /// The body group's sensor_layout.inference_values_per_group.
  int stride{0};
  /// The body group's sensor_layout.has_native_moment.
  bool moment{false};
  /// Normal force at or below which a CoP is NaN [N]; not > 0 → every CoP NaN.
  double cop_min_fz{0.0};
  /// The external wrench lane, or null when the controller has no topic for
  /// it (the block is then not written and its bit never set).
  const ExternalWrenchSample* external{nullptr};
  /// Freshness bound of the external wrench lane (> 0).
  std::int64_t external_max_age_ns{0};
};

/// Fill every field of `pod` but the time and tick. A contact lane is read
/// only when it has one group per contact and room for the force (stride >= 4;
/// the moment needs >= 7 as well as the flag) — the same fit the diag's
/// meas_contact_valid requires — and a group that is not fresh is NaN, not its held value.
/// The external wrench is fresh when a message was received no longer than its
/// bound ago; a non-finite wrench is not fresh.
/// RT-safe: no allocation.
void FillLocomotionTruthLog(const rtc::DeviceState& body, const TruthLogInputs& in,
                            LocomotionTruthLogPod& pod) noexcept;

}  // namespace integrated_bringup::locomotion

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_TRUTH_LOG_HPP_
