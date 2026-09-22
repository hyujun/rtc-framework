#ifndef INTEGRATED_BRINGUP_CONTROLLERS_CATCHING_TRAJ_INPUT_H_
#define INTEGRATED_BRINGUP_CONTROLLERS_CATCHING_TRAJ_INPUT_H_

// ── Vision ingress: PointCloud2 → TrajectorySnapshot (S5.2, L1 §5.1/§5.2) ────
//
// The ROS half of L1. It owns the wire format and nothing else: every decision
// it reaches — order, track epoch, validity — is delegated to
// `rtc::catching`, which has no ROS dependency and is tested against structs
// (D-1). What lives here is the part that cannot be: which bytes mean what.
//
// WHY A FIELD MAP RATHER THAN A STRUCT CAST. The publisher is a DEBUG topic of
// another repository (`ball_perception`'s sim estimator) and is explicitly not
// a stable ABI (W5-2). Its layout today is 384 bytes per point with known
// offsets, and reading it as a packed struct would work — right up until the
// day a field moves, at which point every number stays plausible and the ball
// is somewhere else. So fields are located BY NAME and their datatype and
// count are checked (L1 §4.6); offsets are read from the message.
//
// The map is rebuilt only when the layout hash changes, so the steady-state
// cost is a hash of the field array rather than a lookup per field per
// message. A layout change is reported: an offset-only change is accepted and
// followed, which is the point of parsing by name, but it is not something to
// discover silently.
//
// WHAT A HASH AND A NAME CANNOT CATCH is a change of MEANING — the same fields
// in the same places with a different unit, frame or sign convention. Nothing
// in this file can catch that; L1 §7's physical-consistency checks (S5.2c) are
// where that belongs.

#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <array>
#include <cstdint>
#include <string>

namespace integrated_bringup {

/// Why a message was refused. Every value is counted, and the counters are the
/// first thing to read when "all messages are stale": a lane that is being
/// refused looks exactly like a lane that is silent, from the RT side.
enum class CloudReject : std::uint8_t {
  kNone = 0,
  kBigEndian,       // byte-swapping is not implemented (and no publisher needs it)
  kShape,           // height != 1, or width outside [n_min, n_max]
  kSize,            // data / row_step disagree with point_step x width
  kMissingField,    // a required field is absent by name
  kFieldType,       // a required field has the wrong datatype or count
  kFieldBounds,     // a field would read past point_step
  kFrameId,         // frame_id is not the frame this controller was configured for
  kFutureStamp,     // the origin delay is more negative than future_tol allows
  kStampOverflow,   // the D-2 conversion did not fit in int64 (garbage stamp)
  kNotEvaluated,    // at least one point is not VALID (C-1)
  kStaleSequence,   // duplicate or overtaken snapshot_sequence within a track
  kInconsistentId,  // generation / sequence differ between points of one message
  kMalformed,       // failed the trajectory format check (finite, monotone, spacing)
};

[[nodiscard]] const char* CloudRejectName(CloudReject r) noexcept;

inline constexpr std::size_t kCloudRejectCount = 14;

/// One located field.
struct FieldSlot {
  std::uint32_t offset{0};
  bool found{false};
};

/// Where each required field lives in the current layout.
struct TrajFieldMap {
  /// x, y, z, vx, vy, vz, ax, ay, az — in that order, which is the order the
  /// snapshot's p/v/a triples are filled in.
  std::array<FieldSlot, 9> pva{};
  FieldSlot covariance{};
  FieldSlot snapshot_sequence{};
  FieldSlot generation{};
  FieldSlot horizon_ns{};
  FieldSlot validity{};
  std::uint32_t point_step{0};
  std::uint64_t layout_hash{0};
  bool ok{false};
};

/// Build (or rebuild) the field map from a message's `fields` array.
/// Checks presence by name, datatype, count, and that every field fits inside
/// `point_step`. Returns the refusal reason, `kNone` on success.
[[nodiscard]] CloudReject BuildFieldMap(const sensor_msgs::msg::PointCloud2& msg,
                                        TrajFieldMap& out) noexcept;

/// Everything the ingress needs from configuration. Resolved once at
/// configure from `CatchingParams` plus the controller's own YAML.
struct TrajInputConfig {
  std::int32_t n_min{2};
  std::int32_t n_max{rtc::catching::kCap};
  std::int64_t dt_min_ns{1'000'000};  // 1 ms structural floor; the real spacing gate is n_min
  std::int64_t future_tol_ns{0};
  std::int64_t horizon_min_ns{0};
  std::int64_t track_eval_offset_ns{0};
  double j_warn_m{-1.0};  // negative ⇒ no jump warning configured
  std::string expected_frame{"world"};
};

/// Diagnostics a message leaves behind whether or not it was accepted.
struct TrajInputDiagnostics {
  std::int64_t origin_delay_ns{0};  // recv_wall − stamp (L1 §4.1); published, never judged on
  std::int64_t horizon_ns{0};
  std::int32_t n{0};
  /// Identity of the last ACCEPTED message. Exposed because "exactly one of
  /// three queued predictions was delivered" is true of a queue that kept the
  /// oldest as well as one that kept the newest, and only the sequence tells
  /// them apart.
  std::uint64_t accepted_sequence{0};
  std::uint64_t accepted_generation{0};
  double jump_m{-1.0};  // distance to the previous prediction of the same track, −1 = not compared
  bool horizon_short{false};
  std::uint64_t layout_rebuilds{0};
};

/// Non-RT ingress. One instance per subscription, called only from the
/// controller's non-RT callback group.
class CatchingTrajInput {
 public:
  void Configure(const TrajInputConfig& cfg) noexcept;

  /// Decode and judge one message. On `kNone` the caller may publish `snap`
  /// (and `cov`) to their SeqLocks; on anything else both are untouched and
  /// the reason has been counted.
  ///
  /// `recv_steady` and `recv_wall_ns` must be sampled together, on arrival,
  /// before any work — they are the pair the D-2 conversion stands on.
  [[nodiscard]] CloudReject OnCloud(const sensor_msgs::msg::PointCloud2& msg,
                                    std::int64_t recv_steady_ns, std::int64_t recv_wall_ns,
                                    std::uint64_t activation_generation,
                                    rtc::catching::TrajectorySnapshot& snap,
                                    rtc::catching::CovarianceSnapshot& cov) noexcept;

  [[nodiscard]] std::uint64_t RejectCount(CloudReject r) const noexcept;

  [[nodiscard]] std::uint64_t AcceptCount() const noexcept { return accept_count_; }

  [[nodiscard]] const TrajInputDiagnostics& LastDiagnostics() const noexcept { return diag_; }

  [[nodiscard]] const TrajFieldMap& FieldMap() const noexcept { return map_; }

  /// Forget the accepted-sequence memory and the previous prediction. Called
  /// when the controller re-activates: a comparison against a trajectory from
  /// a previous activation is not a comparison the operator asked for.
  void Reset() noexcept;

 private:
  TrajInputConfig cfg_{};
  TrajFieldMap map_{};
  rtc::catching::IngressState order_{};
  /// The previous ACCEPTED snapshot, kept only to compute the jump diagnostic.
  rtc::catching::TrajectorySnapshot previous_{};
  TrajInputDiagnostics diag_{};
  std::array<std::uint64_t, kCloudRejectCount> rejects_{};
  std::uint64_t accept_count_{0};
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_CATCHING_TRAJ_INPUT_H_
