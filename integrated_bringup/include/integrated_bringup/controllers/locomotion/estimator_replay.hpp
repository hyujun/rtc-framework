// ── The inekf state source replayed offline from a run's logs ────────────────
//
// A run of the locomotion controller logs, every tick, what the state source
// read (locomotion_diag.csv: the IMU sample, its validity, age and sequence,
// the joint / IMU skew, the contact prior, the joint velocities and the
// verdict; joint_ff.csv: the joint positions — the same `tick`). Every source
// records those inputs alike (RecordEstimatorInputs), so a run driven by
// either source can be replayed: each recorded tick is rebuilt into the
// ControllerState the source read (RebuildState) and handed to the very
// InekfStateSource::Update the controller runs — its cadence, hold, gap, skew,
// start gate and reset policy included — on the controller's own model
// (BuildBodyModel). Not a runtime source: an offline tool's core (the
// estimator_replay executable), allocation-free per tick but not RT-bound.
//
// Rebuilding a tick (the clock is synthetic: now = kReplayEpochNs + tick · T —
// only the differences of stamps reach the source):
//   IMU receipt   = now − est_imu_age_us (none when −1)
//   joint receipt = IMU receipt + est_skew_us when the sample is valid (exact),
//                   else now − source_age_us (the older of the joint and the
//                   other receipt the source folded in — approximate; counted)
//   body valid    = source_status ≠ body unreadable
//   planned mask  = est_planned_mask
// The source sees one device (the body) and no hands: it reads the body joints
// only, so the hands' absence changes nothing it computes.
//
// The log is 9 significant digits: a replay is not the online run bit for bit;
// it is the same computation on inputs rounded to the log's precision.
#pragma once

#include "integrated_bringup/controllers/locomotion/inekf_state_source.hpp"
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"
#include "rtc_base/types/types.hpp"

#include <pinocchio/multibody/fwd.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace integrated_bringup::locomotion {

/// The synthetic clock's origin [ns] (any positive value above every age).
inline constexpr int64_t kReplayEpochNs = 1'000'000'000'000;

/// @brief One recorded tick: what InekfStateSource::Update read on it.
struct ReplayInput {
  uint64_t tick{0};        ///< `tick` (the state's iteration)
  uint64_t tick_count{0};  ///< `tick_count` (1 = an activation's first tick: Reset())
  double t_relative_s{0.0};
  uint8_t source_status{0};
  uint64_t joint_sequence{0};               ///< `source_sequence`
  double source_age_us{-1.0};               ///< now − the state's stamp (may be < 0); −1 = none
  int num_joints{0};                        ///< body roster length
  std::array<double, kMaxStateJoints> q{};  ///< joint_ff `q_<j>`, roster order
  std::array<double, kMaxStateJoints> qd{};  ///< diag `v_<j>`, roster order
  std::array<double, 6> imu{};               ///< `imu_gx … imu_az`
  bool imu_valid{false};                     ///< `est_imu_valid`
  uint64_t imu_sequence{0};                  ///< `est_imu_sequence`
  double imu_age_us{-1.0};                   ///< `est_imu_age_us`
  double skew_us{0.0};                       ///< `est_skew_us`
  uint32_t planned_mask{0};                  ///< `est_planned_mask`
  uint64_t gap{0};  ///< ticks missing from the log right before this one (0 = contiguous)

  /// What the online source reported on this tick (diag), for the comparison.
  struct Online {
    uint8_t ctrl_mode{0};            ///< the controller's `mode`
    bool fault_latched{false};       ///< `fault_latched`
    std::array<double, 7> base_q{};  ///< `base_p*`, `base_q*`
    std::array<double, 6> base_v{};  ///< `base_v*`, `base_w*` (base frame)
    bool initialized{false};
    uint8_t reset_reason{0};
    uint32_t reset_count{0};
    int32_t periods{-1};
    uint32_t hold_ticks{0};
    uint8_t step_status{0};
    bool report_valid{false};
    uint32_t estimated_mask{0};
    std::array<uint8_t, kMaxEstimatorSlots> mode{};
    std::array<uint8_t, kMaxEstimatorSlots> update{};
  } online;
};

/// @brief The simulator's truth at one tick (locomotion_truth.csv), when it was fresh.
struct ReplayTruth {
  bool base_valid{false};                       ///< truth_mask bit 0
  std::array<double, 7> base_q{};               ///< p, q_xyzw (raw)
  std::array<double, 6> base_v{};               ///< v, ω — base frame
  std::array<double, kMaxEstimatorSlots> fz{};  ///< truth_grf_<c>_fz (floor on foot), NaN = stale
};

/// @brief Rebuild the ControllerState a recorded tick's source read (header).
/// @param body_channels  the body roster length (`in.num_joints` must equal it)
/// @param imu_site       the primary IMU site's index
/// @return true when the joint receipt is exact (from the IMU receipt and the
///         skew), false when it is approximated from source_age_us.
bool RebuildState(const ReplayInput& in, int imu_site, int64_t now_ns,
                  rtc::ControllerState& state) noexcept;

/// @brief What a replay was set up with.
struct ReplaySetup {
  InekfStateSource::Params params;                ///< imu_site, ages, the inekf config
  std::shared_ptr<const pinocchio::Model> model;  ///< the controller's BodyModel::reduced
  std::vector<std::string> contact_names;         ///< model.contacts order
  std::vector<std::string> contact_frames;
  std::vector<std::string> roster;  ///< the logged body joints, column order
};

/// @brief Steps the inekf source through recorded ticks (header).
class EstimatorReplay {
 public:
  /// @throws std::invalid_argument when a roster joint is not a model joint, a
  ///         model joint is not logged, or the source refuses its setup.
  explicit EstimatorReplay(const ReplaySetup& setup);
  ~EstimatorReplay();
  EstimatorReplay(const EstimatorReplay&) = delete;
  EstimatorReplay& operator=(const EstimatorReplay&) = delete;

  /// One recorded tick: Reset() first when it opens an activation (or is the
  /// first one stepped — a log that starts inside an activation then differs
  /// from the online run until its own next reset), rebuild,
  /// Update(). `truth` (optional) gives the contacts' true velocity.
  const MeasuredRobotState& Step(const ReplayInput& in, const ReplayTruth* truth);

  [[nodiscard]] const InekfStateSource& source() const noexcept { return *source_; }

  [[nodiscard]] const MeasuredRobotState& last() const noexcept { return out_; }

  /// Whether the last tick's joint receipt was exact (RebuildState).
  [[nodiscard]] bool last_joint_exact() const noexcept { return joint_exact_; }

  /// The last tick's contact velocities (world axes, LOCAL_WORLD_ALIGNED origin
  /// velocity of each contact frame) from the truth base and the logged joints;
  /// NaN without a valid truth row.
  [[nodiscard]] const std::array<std::array<double, 3>, kMaxEstimatorSlots>&
  truth_contact_velocity() const noexcept {
    return truth_vel_;
  }

  [[nodiscard]] const Eigen::Matrix3d& R_BI() const noexcept { return R_BI_; }

  [[nodiscard]] const Eigen::Vector3d& p_BI() const noexcept { return p_BI_; }

  [[nodiscard]] int num_contacts() const noexcept { return static_cast<int>(frames_.size()); }

 private:
  void TruthVelocities(const ReplayInput& in, const ReplayTruth* truth);

  ReplaySetup setup_;
  DeviceIndices indices_;
  std::unique_ptr<InekfStateSource> source_;
  rtc::ControllerState state_{};
  MeasuredRobotState out_{};
  bool joint_exact_{false};
  bool started_{false};  ///< a tick has been stepped (the first one resets)
  int64_t period_ns_{0};
  Eigen::Matrix3d R_BI_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_BI_{Eigen::Vector3d::Zero()};
  std::vector<std::size_t> frames_;        ///< contact frame ids in the model
  std::vector<int> roster_to_model_;       ///< roster channel → model joint index
  std::unique_ptr<pinocchio::Data> data_;  ///< the model's data (truth FK)
  std::array<std::array<double, 3>, kMaxEstimatorSlots> truth_vel_{};
};

/// @brief A run's recorded ticks, read from its logs (header).
struct ReplayLog {
  std::vector<std::string> roster;                ///< body joints, column order (`q_<j>` / `v_<j>`)
  std::vector<ReplayInput> ticks;                 ///< joined, in log order
  std::vector<std::optional<ReplayTruth>> truth;  ///< per tick; empty without a truth file
  std::size_t diag_rows{0};                       ///< the file's (every run)
  std::size_t unmatched{0};  ///< diag rows with no joint_ff row of their tick (the file's)
  std::size_t gaps{0};       ///< places where ticks are missing inside a run
  std::size_t missing_ticks{0};
  std::size_t runs{0};  ///< pieces where `tick` restarts (a new sim in one file)
  int run{0};           ///< the piece kept (kAllRuns = all)
};

/// @brief Read and join a session's logs. A diag row whose joint_ff row is
///        missing (a dropped row, or the drain tail) is dropped and counted;
///        `gap` on the next kept row counts every tick missing before it.
/// @param truth_path  empty = no truth
/// @throws std::runtime_error on a missing file or column, or a malformed row.
/// @param run  which piece to keep when `tick` restarts inside a file (a later
///             sim appended to a session folder an earlier one also wrote):
///             kLastRun (default), kAllRuns, or its index
/// @param contact_names  the model's contacts: the truth file's
///                       `truth_grf_<c>_fz` columns are read by these names
[[nodiscard]] ReplayLog LoadReplayLog(const std::string& diag_path,
                                      const std::string& joint_ff_path,
                                      const std::string& truth_path,
                                      const std::vector<std::string>& contact_names, int run = -1);

inline constexpr int kLastRun = -1;
inline constexpr int kAllRuns = -2;

/// @brief The replay CSV (17 significant digits): one row per replayed tick.
void WriteReplayHeader(std::ostream& os, const std::vector<std::string>& contact_names);
/// @param truth  the tick's truth row (nullptr = none)
void WriteReplayRow(std::ostream& os, const ReplayInput& in, const ReplayTruth* truth,
                    const EstimatorReplay& replay);

}  // namespace integrated_bringup::locomotion
