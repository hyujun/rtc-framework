// ── What a state estimator reports about one tick, beside the state itself ────
//
// Carried in MeasuredRobotState (measured_state_source.hpp) and copied as is
// into the locomotion diagnostic row. Two parts:
//   - the estimator's inputs as this tick read them (`imu`, `imu_valid`,
//     `imu_age_ns`, `imu_sequence`, `skew_ns`, `planned_mask`) — every source
//     records them alike (RecordEstimatorInputs), truth included, so a run
//     driven by either source can be replayed through the estimator offline;
//   - the estimator's own record (everything else), which a source that
//     estimates nothing (truth) leaves at its defaults.
// Plain data: no Eigen, no estimator header, so the log pod can hold it.
//
// Slot i is the model's contact i (model.contacts order). The covariance
// diagonal follows the estimator's error order (contact_inekf.hpp: φ, v, p,
// d_0 … d_{n−1}, b_g, b_a), 15 + 3n values; the rest stay zero.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace integrated_bringup::locomotion {

inline constexpr std::size_t kMaxEstimatorSlots = 4;
/// Error-state rows before the slots (φ, v, p, then b_g, b_a after them).
inline constexpr std::size_t kEstimatorCovDiagBase = 15;
inline constexpr std::size_t kMaxEstimatorCovDiag = kEstimatorCovDiagBase + 3 * kMaxEstimatorSlots;

/// Why the estimator last restarted.
enum class EstimatorResetReason : std::uint8_t {
  kNone = 0,       ///< never reset
  kRequested = 1,  ///< MeasuredStateSource::Reset() (an activation)
  kGap = 2,  ///< IMU samples missed for longer than one step may span, or the sequence went back
  kStateMismatch = 3,  ///< the contact FSM and the filter disagreed on the anchors
  kRefused = 4,        ///< the filter refused max_refused_ticks steps in a row
  kGated = 5,          ///< every contact update gated for max_gated_s
  kNoAnchor = 6,       ///< no confirmed contact for max_no_anchor_s
};

/// Bits of EstimatorTelemetry::unobservable.
inline constexpr std::uint8_t kYawUnobservable = 1U << 0;
inline constexpr std::uint8_t kPositionUnobservable = 1U << 1;

/// @brief One tick of an estimator's own record.
struct EstimatorTelemetry {
  bool initialized{false};
  std::uint8_t reset_reason{0};   ///< EstimatorResetReason of the last reset
  std::uint32_t reset_count{0};   ///< resets since construction
  std::uint64_t imu_sequence{0};  ///< the primary IMU site's sequence read this tick (input)
  /// Sampling periods stepped this tick: ≥ 1 a step, 0 the sample repeated
  /// (held), −1 no step (missing IMU, a reset, not started).
  std::int32_t periods{-1};
  std::int64_t skew_ns{0};  ///< joint receipt − IMU receipt [ns] when imu_valid, else 0 (input)
  std::uint32_t hold_ticks{0};  ///< consecutive ticks on a repeated sample
  std::uint8_t step_status{0};  ///< the step's rtc::estimation::ContactFsmStatus (0 = ok / none)
  std::uint8_t core_status{0};  ///< the filter's last refusal (rtc::estimation::InekfStatus)
  bool report_valid{false};     ///< this tick's contact update ran
  std::uint8_t num_slots{0};
  std::uint32_t planned_mask{0};    ///< the contact prior of this tick, model contacts only (input)
  std::uint32_t estimated_mask{0};  ///< bit i: slot i in stance or unknown
  std::array<std::uint8_t, kMaxEstimatorSlots> mode{};    ///< rtc::estimation::ContactMode
  std::array<std::uint8_t, kMaxEstimatorSlots> reason{};  ///< its last transition's ContactReason
  std::array<std::uint8_t, kMaxEstimatorSlots> update{};  ///< ContactUpdateResult (report_valid)
  std::array<double, kMaxEstimatorSlots> evidence{};      ///< the FSM's contact score [0, 1]
  std::array<double, kMaxEstimatorSlots> speed{};         ///< the speed the FSM tested [m/s]
  std::array<double, kMaxEstimatorSlots> m2{};            ///< the update's NIS (report_valid)
  std::int32_t no_anchor_ticks{0};  ///< ticks since a contact was last confirmed
  std::int32_t gated_ticks{0};      ///< consecutive ticks whose update gated and used nothing
  std::int32_t refused_ticks{0};    ///< consecutive refused steps
  std::array<double, kMaxEstimatorCovDiag> p_diag{};  ///< diag of the error covariance
  std::uint8_t unobservable{0};  ///< kYawUnobservable | kPositionUnobservable when started
  /// The primary IMU site's raw sample (gyro, accel), site frame — as read, valid or
  /// not; zero when the site is not in the state.
  std::array<double, 6> imu{};
  /// The sample is usable: the site is in the state, its producer flags it valid,
  /// it has a receipt, and all six values are finite (a sequence that went back is
  /// the estimator's own verdict, not this one).
  bool imu_valid{false};
  /// now − the site's receipt [ns]; 0 for a receipt stamped after now, −1 when
  /// the site never reported or is not in the state.
  std::int64_t imu_age_ns{-1};
  /// This tick's support-height pin per slot (estimator.inekf.ground): its
  /// rtc::estimation::GroundUpdateResult (0 = none), innovation ν [m] and NIS.
  std::array<std::uint8_t, kMaxEstimatorSlots> ground{};
  std::array<double, kMaxEstimatorSlots> ground_innovation{};
  std::array<double, kMaxEstimatorSlots> ground_nis{};
  std::int32_t gated_pins{0};  ///< consecutive gated pins since the last applied one
};

static_assert(std::is_trivially_copyable_v<EstimatorTelemetry>,
              "EstimatorTelemetry travels inside MeasuredRobotState and the log pod");

}  // namespace integrated_bringup::locomotion
