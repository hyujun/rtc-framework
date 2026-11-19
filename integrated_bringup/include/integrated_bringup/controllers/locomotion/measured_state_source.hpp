// ── The measured robot state the humanoid controller runs on, and where it comes
// from ─────────────────────────────────────────────────────────────────────────
//
// MeasuredRobotState is the controller's view of the robot at one tick: the
// full-model configuration (floating base + every actuated joint, hands included
// — kinematics needs the hand frames), its velocity, the contact prior, the
// external wrench at each end-effector, and the verdict on all of it. A source
// fills it from the tick's ControllerState; the controller never reads the
// device lanes itself.
//
//   q = [p_xyz, q_xyzw, joints...]   nq = 7 + num_joints  (pinocchio free-flyer order)
//   v = [v_xyz, ω_xyz, joints...]    nv = 6 + num_joints  (base part in the base frame)
//
// Joint slot a of q is q[7 + a] and of v is v[6 + a], with a the joint's index
// in the full model's joint order (JointOrder). Every actuated joint is assumed
// to be one-DoF (nq = nv per joint); the controller checks that at configure.
//
// MeasuredStateSource is the seam between "where the state comes from" and the
// control law: TruthStateSource below copies the simulator's ground truth, and
// InekfStateSource (inekf_state_source.hpp) estimates the base from the IMU and
// the leg kinematics. Which one runs is `estimator.source`, fixed at configure.
// Both run on the RT tick: Update() is noexcept and allocation-free. Each tick
// the caller hands the source a SourceTickInput (the contact prior it knows);
// a source that estimates also fills `estimator` (estimator_telemetry.hpp),
// truth leaves it zero. `state_epoch` is written on every Update, valid or
// not, and changes only when the source restarts its state (Reset(), or an
// estimator's own reset): a consumer that compares it tick to tick sees every
// discontinuity of the base.
//
// Verdict (truth). `status` is the first failure in this order: body joints
// unreadable, body joints stale, base missing, base stale, base pose invalid,
// non-finite value. `valid` is status == kOk. A lane that fails reads as the
// neutral state (zero joints, the identity base, zero velocity) and its
// valid_mask bit stays clear; with `valid` false, q / v are not a state —
// only the lanes valid_mask names hold a real sample. A lane stamped after the
// caller's `now` counts as age 0. A hand that is stale, has never
// reported or reports a non-finite value does NOT fail the state — its joints keep the last fresh
// value (0 until the first) with zero velocity, and only its bit in fresh_mask / valid_mask says
// so: the hands are held still and their frames only place the end-effectors.
#pragma once

#include "integrated_bringup/controllers/locomotion/estimator_telemetry.hpp"
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "integrated_bringup/controllers/locomotion/inekf_source_config.hpp"
#include "rtc_base/types/types.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace integrated_bringup::locomotion {

/// Actuated joints a state can hold, body and hands together.
inline constexpr std::size_t kMaxStateJoints = static_cast<std::size_t>(rtc::kMaxDeviceChannels);
/// Hand device groups a state can hold (every device but the body's).
inline constexpr std::size_t kMaxStateHands =
    static_cast<std::size_t>(rtc::ControllerState::kMaxDevices) - 1;
/// Contacts the prior mask can name.
inline constexpr int kMaxStateContacts = 32;
/// |‖q_base‖ − 1| a truth quaternion may carry and still be renormalised.
inline constexpr double kQuaternionNormTolerance = 1e-3;

/// fresh_mask / valid_mask bits.
inline constexpr uint32_t kBodyJointsBit = 1U << 0;
inline constexpr uint32_t kBaseBit = 1U << 1;

[[nodiscard]] constexpr uint32_t HandBit(std::size_t hand) noexcept {
  return 1U << (2U + static_cast<uint32_t>(hand));
}

enum class StateSourceKind : uint8_t {
  kTruth,      ///< a simulator's ground truth, copied
  kEstimated,  ///< an estimator's output
};

enum class StateSourceStatus : uint8_t {
  kOk = 0,
  kBodyUnreadable,  ///< body device not reported, too few channels, or a position / velocity hole
  kBodyStale,       ///< body joint lane older than the body age limit
  kBaseMissing,     ///< base lane never received, or its producer says invalid
  kBaseStale,       ///< base lane older than the body age limit
  kBaseInvalid,     ///< base quaternion norm off 1 by more than kQuaternionNormTolerance
  kNonFinite,       ///< a NaN / Inf in q or v
  kImuMissing,      ///< primary IMU site not reported, flagged invalid, non-finite, or its sequence
                    ///< went back
  kImuStale,        ///< the IMU sample repeated for more than the hold budget
  kImuSkew,         ///< joint and IMU receipts further apart than the invalid bound
  kNotInitialized,  ///< the estimator has not started (no still window yet, or just reset)
};

[[nodiscard]] const char* ToString(StateSourceStatus status) noexcept;

/// Where an end-effector's external wrench comes from. kUnknown: nobody knows it
/// (the controller treats it as absent); its wrench is zero.
enum class ExternalWrenchKind : uint8_t { kUnknown, kMeasured, kEstimated };

/// @brief The external wrench acting on one end-effector (an object held, a push).
struct ExternalWrench {
  ExternalWrenchKind kind{ExternalWrenchKind::kUnknown};
  std::array<double, 6> wrench{};  ///< [f; τ] [N, N·m] in the end-effector frame
  int64_t stamp_steady_ns{0};      ///< 0 = never
};

/// @brief One tick's measured state of the full model.
struct MeasuredRobotState {
  int nq{0};
  int nv{0};
  std::array<double, 7 + kMaxStateJoints> q{};
  std::array<double, 6 + kMaxStateJoints> v{};
  uint32_t contact_mask{0};  ///< bit c = contact c is closed (a prior or an estimate)
  std::array<ExternalWrench, kMaxEndEffectors> ee_wrench{};  ///< model.end_effectors order
  int64_t stamp_steady_ns{0};  ///< the older of the body joint and base receipts
  uint64_t sequence{0};        ///< body joint_sequence
  uint32_t model_epoch{0};     ///< which model layout q / v are in (source params)
  uint32_t state_epoch{0};     ///< bumped by every source Reset()
  uint32_t fresh_mask{0};      ///< lanes inside their age limit this tick
  uint32_t valid_mask{0};      ///< lanes holding a real sample (fresh now or held)
  StateSourceKind kind{StateSourceKind::kTruth};
  StateSourceStatus status{StateSourceStatus::kBodyUnreadable};
  bool valid{false};
  EstimatorTelemetry estimator{};  ///< zero from a source that estimates nothing
};

/// @brief What the caller knows on this tick that a source may use.
struct SourceTickInput {
  /// Bit c = contact c planned closed (the controller's mask as of the last tick).
  uint32_t planned_mask{0};
};

static_assert(std::is_trivially_copyable_v<MeasuredRobotState>,
              "MeasuredRobotState crosses SeqLock / SPSC boundaries");

/// @brief Where one device group's channels land in the state.
struct DeviceChannelMap {
  int device{-1};       ///< slot in ControllerState::devices
  int num_channels{0};  ///< roster length — the device must report at least this many
  std::array<int16_t, kMaxStateJoints> joint{};  ///< channel → joint index a
};

/// @brief The device layout of a state: built once at configure, read every tick.
struct DeviceIndices {
  DeviceChannelMap body;
  std::array<DeviceChannelMap, kMaxStateHands> hands{};
  int num_hands{0};
  int num_joints{0};  ///< actuated joints of the full model
};

/// @brief A hand group's device slot and roster, for @ref BuildDeviceIndices.
struct HandRoster {
  int device{-1};
  std::vector<std::string> joint_names;
};

/// @brief Map every roster channel to its joint in `joint_order` (configure time).
/// @param joint_order  the full model's @ref JointOrder (body and hands)
/// @throws std::invalid_argument naming `context` when `joint_order` exceeds
///         kMaxStateJoints, a device slot is out of range or used twice, there are
///         more hands than kMaxStateHands, a roster is empty or names a joint not
///         in `joint_order` or one already mapped, or a joint of `joint_order` is
///         on no roster.
[[nodiscard]] DeviceIndices BuildDeviceIndices(const std::vector<std::string>& joint_order,
                                               int body_device,
                                               const std::vector<std::string>& body_roster,
                                               const std::vector<HandRoster>& hands,
                                               const std::string& context);

/// @brief The `estimator` section of the controller config.
///
///   estimator:
///     source: truth          # truth | inekf
///     body_max_age_ms: 10.0  # body joint and base (truth) lanes, (0, 1000]
///     hand_max_age_ms: 50.0  # hand joint lanes, (0, 1000]
///     inekf: {...}           # inekf_source_config.hpp; required with source: inekf
struct EstimatorConfig {
  std::string source;
  double body_max_age_ms{0.0};
  double hand_max_age_ms{0.0};
  std::optional<InekfSourceConfig> inekf;  ///< parsed (and checked) whenever present
};

/// @throws std::invalid_argument on an unknown or missing key, a source other than
///         "truth" or "inekf", an age outside (0, 1000] ms, `inekf` missing with
///         source: inekf, or an `inekf` section ParseInekfSourceConfig refuses.
[[nodiscard]] EstimatorConfig ParseEstimatorConfig(const YAML::Node& estimator,
                                                   const std::string& context);

/// @brief The seam between where the measured state comes from and the control law.
class MeasuredStateSource {
 public:
  virtual ~MeasuredStateSource() = default;
  MeasuredStateSource() = default;
  MeasuredStateSource(const MeasuredStateSource&) = delete;
  MeasuredStateSource& operator=(const MeasuredStateSource&) = delete;
  MeasuredStateSource(MeasuredStateSource&&) = delete;
  MeasuredStateSource& operator=(MeasuredStateSource&&) = delete;

  /// Forget everything held across ticks and bump the state epoch. Allocation-free,
  /// but not synchronised with Update(): call it from the thread that runs
  /// Update(), or while Update() cannot run (activation).
  virtual void Reset() noexcept = 0;

  /// Fill `out` from this tick's `state`. RT: no allocation, no throw.
  /// @param tick           what the caller knows on this tick (SourceTickInput)
  /// @param now_steady_ns  the steady clock, read on this tick
  /// @return out.valid
  [[nodiscard]] virtual bool Update(const rtc::ControllerState& state, const DeviceIndices& indices,
                                    const SourceTickInput& tick, int64_t now_steady_ns,
                                    MeasuredRobotState& out) noexcept = 0;
};

/// @brief The hands' last fresh joints, held while a hand is stale (every source
///        treats the hands alike: they only place the end-effectors).
struct HeldHands {
  std::array<double, kMaxStateJoints> q{};
  uint32_t seen{0};  ///< HandBit(h) set once hand h has been fresh
};

/// @brief The joint lanes' part of a verdict.
struct JointLaneVerdict {
  bool body_present{false};  ///< the body device slot exists in the state
  bool body_unreadable{false};
  bool body_stale{false};
  bool non_finite{false};
};

/// @brief Read the body joint lanes and the hands into `out` — the part every
///        source shares (header: the hands' rule, the age rule). Writes the body
///        joints, `sequence` and `stamp_steady_ns` (the joint receipt), the hands,
///        and their fresh / valid bits; `out.q` / `out.v` must already hold the
///        neutral state. RT-safe.
[[nodiscard]] JointLaneVerdict ReadJointLanes(const rtc::ControllerState& state,
                                              const DeviceIndices& indices, int64_t now_steady_ns,
                                              int64_t body_max_age_ns, int64_t hand_max_age_ns,
                                              HeldHands& held, MeasuredRobotState& out) noexcept;

/// @brief Age of a lane's last receipt against `now`; negative when it never
///        arrived, 0 when it is stamped after `now`.
[[nodiscard]] int64_t LaneAgeNs(int64_t recv_steady_ns, int64_t now_steady_ns) noexcept;

/// @brief The simulator's ground truth: the body group's joint and floating-base
///        lanes and the hand groups' joint lanes, copied. Contacts are all closed
///        (a stance prior) and every external wrench is kUnknown.
class TruthStateSource final : public MeasuredStateSource {
 public:
  struct Params {
    int64_t body_max_age_ns{0};  ///< > 0
    int64_t hand_max_age_ns{0};  ///< > 0
    int num_contacts{0};         ///< [0, kMaxStateContacts]
    uint32_t model_epoch{0};
  };

  /// @throws std::invalid_argument on a non-positive age or a contact count out of range.
  explicit TruthStateSource(const Params& params);

  void Reset() noexcept override;
  /// `tick` is not read: truth knows no prior.
  [[nodiscard]] bool Update(const rtc::ControllerState& state, const DeviceIndices& indices,
                            const SourceTickInput& tick, int64_t now_steady_ns,
                            MeasuredRobotState& out) noexcept override;

 private:
  Params params_;
  uint32_t state_epoch_{0};
  HeldHands held_;
};

}  // namespace integrated_bringup::locomotion
