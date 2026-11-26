#ifndef RTC_MPC_REFERENCE_STAGE_REFERENCE_BUILDER_HPP_
#define RTC_MPC_REFERENCE_STAGE_REFERENCE_BUILDER_HPP_

/// @file stage_reference_builder.hpp
/// @brief Turns the gait scheduler's window, the measured state and the
///        filtered command into one @ref HorizonReference per MPC tick.
///
/// Owner: the MPC thread (SCHED_FIFO — RT path). `Init` is configure time
/// (it allocates its pinocchio Data and two horizon slots); `Build` and
/// `active` run on the owner thread, allocate nothing and throw nothing.
///
/// `Build` fills the back slot completely, then swaps the slot index, so
/// `active()` is always a whole horizon; a failed Build changes nothing
/// (`active()` · `reference_id` as they were). The reference `active()`
/// returns is valid until the next Build (it then names the other slot).
///
/// Per node `k` at `t_k = t0 + k·dt` (`t0` = the schedule's, `H = (n_nodes − 1)·dt`)
/// — or, built on a @ref NodeGrid, at `t_k = grid.Time(m + k)` with the window
/// starting at `grid.Time(m)`: every tick then gives a node its time from the
/// integer index, so an event on the grid falls on the same side of a node on
/// every tick (with `t0 + k·dt` the same instant rounds to either side from one
/// tick to the next, and the half-open mode lookup gives it either mode):
///  - **mode** = `SampleNodes` of the schedule (half-open at an event).
///  - **base** (three knots, linear between them): the measured planar pose
///    `(p, ψ)` from the snapshot's free-flyer, the measured world velocity
///    `v_m = (R·v_lin)_xy`, `ω_m = (R·ω)_z` (the full rotation, not yaw alone),
///    the commanded `v_c = R_z(ψ)·[v_x, v_y]`, `ω_c = ω_z` (0 when the command
///    is not valid); knot 1 at `f·H`: `p + f·H·(v_m + v_c)/2`,
///    `ψ + f·H·(ω_m + ω_c)/2`; knot 2 at `H`: `+ (1 − f)·H·v_c`, `+ (1 − f)·H·ω_c`
///    (`f` = `intermediate_fraction`). Height = the command's `base_height` at
///    every node, roll · pitch 0, the yaw interpolated unwrapped and turned
///    into `(0, 0, sin ψ/2, cos ψ/2)`. Velocity reference in LOCAL:
///    `v_lin,k = R_z(ψ_0 − ψ_k)·[v_x, v_y, 0]`, `ω = [0, 0, ω_z]` — the
///    commanded world velocity seen from the node's own yaw, so the pose and
///    velocity references agree.
///  - **joints**: `q_default` (bitwise), velocity 0; with arm swing on and the
///    gait not single-mode, `Δq_j = sign_j · gain · sin(2π(φ_k − phase_offset)) · v_x`
///    (`φ_k` = @ref GaitPhase at `t_k`) on every node but the terminal one,
///    the swung value clamped to the joint's position limits.
///  - **contacts**: x · y · yaw from forward kinematics of the snapshot (the
///    foot where it is), z from the swing plan (@ref BuildSwing on each swing
///    stretch of the window; a stretch already under way at `t0` is closed at
///    the lift-off @ref GaitScheduler::SwingStart reports — unless that is
///    clamped at the gait's take-over, when the open-start plan stands) or the terrain
///    height under that x · y in stance; twist · acceleration carry only ż · z̈.
///    With the foothold on (below) x · y · yaw are planned instead.
///  - **wrench**: every stance contact `F_z = m·|g| / n_stance`, else 0; a
///    flight node is all zero.
///
/// **Foothold** (`foothold.enabled`; off, the horizon is byte for byte the one
/// above). Each swing stretch `[t_lo, t_td)` of contact `c` in the horizon gets
/// a landing target from the base reference at touch-down:
/// `p_land = p_ref(t_td) + R_z(ψ_ref(t_td))·o_c + (T_st/2)·v_c`,
/// `ψ_land = ψ_ref(t_td) + ψ_c + (T_st/2)·ω_c` — `p_ref` · `ψ_ref` the base
/// knots above (continued past `H` at `v_c` · `ω_c`), `(o_c, ψ_c)` the planar
/// offset and yaw of the contact from the base at `q_default` (forward
/// kinematics at Init, base at the origin, level), `T_st` the stance after
/// touch-down: up to the contact's next lift-off in the window, else the gait
/// table's (@ref GaitScheduler::table — the stance run of the gait in force at
/// `t_td` from its phase there, one period when the contact never lifts). x ·
/// y · yaw run one cubic Hermite each from the lift-off pose (the previous
/// landing target, else the measured one), at rest, to the landing target at
/// rest; a stretch under way at `t0` starts at `t0` from the measured pose and
/// world velocity of the contact frame. The yaw target is taken within π of the
/// start. `contact_pose` x · y · yaw, `contact_twist` ẋ · ẏ · ω_z and
/// `contact_acceleration` ẍ · ÿ · α_z come from those curves; z is the swing
/// plan above. Stance nodes after a touch-down hold its landing target, a
/// contact in stance before its first lift-off the measured pose. Fallbacks: a
/// stretch whose touch-down is beyond the window (`open_end`) holds its
/// lift-off pose (twist 0); one too short to fit a curve
/// (`≤ kMinSwingDuration`) holds the landing target.
///
/// YAML schema (`reference:`, every key required; `q_default` must name
/// exactly the joints the caller lists, `joints` only those):
/// ```yaml
/// intermediate_fraction: 0.7   # in (0, 1) — where the averaged-velocity knot sits
/// q_default:                   # [rad or m] one entry per 1-DoF joint, inside its limits
///   <joint>: 0.0
/// arm_swing:
///   enabled: false
///   gain: 0.15                 # [rad·s/m] amplitude per m/s of commanded v_x
///   phase_offset: 0.15         # gait-phase offset of the sine
///   joints:                    # the swinging joints and their sign (±1); checked even when off
///     - {name: <joint>, sign: 1}
/// foothold:
///   enabled: false             # plan swing x · y · yaw to a landing target (file comment)
/// ```

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace rtc::mpc {

/// @brief Most swing stretches of one contact a window can hold.
inline constexpr std::size_t kMaxSwingsPerContact = (kMaxScheduleEvents + 2) / 2;

/// @brief Smallest node spacing @ref StageReferenceBuilder::Init accepts [s].
inline constexpr double kMinReferenceTimeStep = 1e-9;

/// @brief Failure modes of @ref LoadReferenceConfig, @ref StageReferenceBuilder::Init
///        and @ref StageReferenceBuilder::Build.
enum class ReferenceError {
  kNoError = 0,
  // loader
  kNotAMap,          ///< the section (or `q_default` · `arm_swing` · a joint entry) is not a map
  kUnknownKey,       ///< a key outside the schema
  kMissingKey,       ///< a schema key without a value
  kWrongType,        ///< a value of the wrong kind
  kNonFinite,        ///< a NaN / ±inf value
  kInvalidFraction,  ///< `intermediate_fraction ∉ (0, 1)`
  kInvalidSign,      ///< an arm-swing `sign ∉ {−1, 1}`
  kEmptyArmSwingJoints,  ///< `enabled` with no joints
  kUnknownJoint,         ///< a joint name not in the caller's list
  kMissingJoint,    ///< loader: a listed joint without a `q_default` entry; Init: a model joint
                    ///< without one
  kDuplicateJoint,  ///< a joint named twice in `arm_swing.joints`
  // Init
  kHandlerNotInitialised,
  kFixedBase,         ///< the model's root is not a free-flyer
  kUnsupportedJoint,  ///< a model joint that is not 1-DoF (after the root)
  kOutOfLimits,       ///< a `q_default` outside the model's position limits
  kCapacity,  ///< `n_nodes ∉ [2, kMaxHorizon + 1]`, `nq > kMaxNq`, `nv > kMaxNv`, or too many
              ///< contacts
  kInvalidTimeStep,     ///< `dt` not finite or < kMinReferenceTimeStep
  kInvalidSwingConfig,  ///< @ref ValidateSwingConfig refused
  kInvalidTerrain,      ///< @ref ValidateSurfaceFrame refused
  // Build
  kNotInitialised,
  kNoSchedule,        ///< the scheduler has not built a window (`schedule_id == 0`)
  kScheduleMismatch,  ///< the window's contact count is not the model's, or a mode bit beyond it
  kSnapshotMismatch,  ///< `nq` / `nv` differ, or the free-flyer quaternion is not unit
  kNonFiniteInput,    ///< a NaN / ±inf in the snapshot or the command, or a knot (or a foothold
                      ///< curve) that overflowed
  kWindowTooShort,    ///< a node falls outside the window (@ref SampleNodes)
  kBaseBelowTerrain,  ///< `base_height ≤` the terrain height under the base
  kSwingCapacity,     ///< more swing stretches than @ref kMaxSwingsPerContact
  kSwingFailed,       ///< @ref BuildSwing refused a stretch
  kOffGrid,           ///< the grid's spacing is not the builder's, or the window does not
                      ///< start at the grid node (@ref NodeGrid)
};

struct ArmSwingJoint {
  std::string name;
  double sign{1.0};
};

/// @brief The `reference:` section (configure time — strings allowed).
struct ReferenceConfig {
  std::vector<std::pair<std::string, double>> q_default;  ///< in YAML order
  double intermediate_fraction{0.0};
  bool arm_swing_enabled{false};
  double arm_swing_gain{0.0};
  double arm_swing_phase_offset{0.0};
  std::vector<ArmSwingJoint> arm_swing_joints;
  bool foothold_enabled{false};
};

/// @brief Parse the `reference:` map. `joint_names` are the joints `q_default`
///        must cover exactly and `arm_swing.joints` may name (checked even
///        when arm swing is off); limits are the model's, checked at Init.
///        `out` is written only on `kNoError`, which also implies
///        @ref ValidateReferenceConfig passed.
[[nodiscard]] ReferenceError LoadReferenceConfig(const YAML::Node& node,
                                                 std::span<const std::string> joint_names,
                                                 ReferenceConfig& out) noexcept;

/// @brief The value checks of the schema (fraction in (0, 1), finite gain and
///        offset, signs ±1, joints when enabled) — everything but names and limits.
[[nodiscard]] ReferenceError ValidateReferenceConfig(const ReferenceConfig& cfg) noexcept;

/// @brief The builder (file comment). Not copyable: it owns its slots.
class StageReferenceBuilder {
 public:
  StageReferenceBuilder() = default;
  StageReferenceBuilder(const StageReferenceBuilder&) = delete;
  StageReferenceBuilder& operator=(const StageReferenceBuilder&) = delete;

  /// @brief Bind to `handler`'s model (floating base; every joint after the
  ///        root 1-DoF, each with a `q_default` inside its limits — `q_default`
  ///        entries and arm-swing joints the model lacks are ignored, the
  ///        loader having checked their names) and size the horizon: `n_nodes` stages
  ///        (`2 … kMaxHorizon + 1`) at `dt`. The scheduler's window must
  ///        cover `(n_nodes − 1)·dt` (the caller's Init contract). Allocates;
  ///        on failure nothing changes. Both slots start zeroed.
  [[nodiscard]] ReferenceError Init(const RobotModelHandler& handler, const ReferenceConfig& cfg,
                                    const SwingConfig& swing, const SurfaceFrame& terrain,
                                    int n_nodes, double dt);

  /// @brief Build the next horizon from the scheduler's current window (its
  ///        `t0`), the snapshot and the command (file comment). On error
  ///        `active()` and `reference_id` are unchanged.
  [[nodiscard]] ReferenceError Build(const GaitScheduler& scheduler,
                                     const MPCStateSnapshot& snapshot,
                                     const FilteredCommand& cmd) noexcept;

  /// @brief As above, with node `k` at `grid.Time(first_index + k)`. The
  ///        scheduler's window must start at `grid.Time(first_index)` and
  ///        `grid.dt` must be the builder's (`kOffGrid` otherwise). The base
  ///        knots stay relative to node 0 (`k·dt`).
  [[nodiscard]] ReferenceError Build(const GaitScheduler& scheduler,
                                     const MPCStateSnapshot& snapshot, const FilteredCommand& cmd,
                                     const NodeGrid& grid, std::int64_t first_index) noexcept;

  /// @brief The last successfully built horizon (zeroed before the first).
  ///        Valid until the next Build.
  [[nodiscard]] const HorizonReference& active() const noexcept { return *slots_[active_]; }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

  [[nodiscard]] int n_nodes() const noexcept { return n_nodes_; }

 private:
  /// The body of both Builds; node times on @p grid when it is not null.
  [[nodiscard]] ReferenceError BuildImpl(const GaitScheduler& scheduler,
                                         const MPCStateSnapshot& snapshot,
                                         const FilteredCommand& cmd, const NodeGrid* grid,
                                         std::int64_t first_index) noexcept;

  /// One swing stretch's x · y · yaw (foothold on).
  struct Foothold {
    CubicSegment x{}, y{}, psi{};
    double land_x{0.0}, land_y{0.0}, land_psi{0.0};  ///< the target (or the held pose)
    bool curve{false};    ///< false: the stretch holds (land_x, land_y, land_psi)
    bool landing{false};  ///< it touches down in the window (stance after it holds the target)
  };

  /// A contact's planar pose at `q_default` relative to the base (Init).
  struct Nominal {
    double x{0.0}, y{0.0}, psi{0.0};
  };

  struct ArmJoint {
    int idx_q{0};
    double sign{1.0};
    double lower{0.0};  ///< position limits the swung value is clamped to
    double upper{0.0};
  };

  const RobotModelHandler* handler_{nullptr};
  std::unique_ptr<pinocchio::Data> data_{};
  std::array<std::unique_ptr<HorizonReference>, 2> slots_{};
  std::size_t active_{0};
  std::vector<double> q_default_{};     ///< nq, joint entries (base zero)
  std::vector<ArmJoint> arm_joints_{};  ///< empty when arm swing is off
  std::array<std::array<SwingTrajectory, kMaxSwingsPerContact>, kMaxContacts> swings_{};
  std::array<std::array<Foothold, kMaxSwingsPerContact>, kMaxContacts> footholds_{};
  std::array<Nominal, kMaxContacts> nominal_{};
  std::array<std::uint32_t, kMaxHorizon + 1> modes_{};
  SwingConfig swing_{};
  SurfaceFrame terrain_{};
  double dt_{0.0};
  double weight_{0.0};  ///< m·|g|
  double fraction_{0.0};
  double gain_{0.0};
  double phase_offset_{0.0};
  int n_nodes_{0};
  bool foothold_{false};
  bool initialized_{false};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_STAGE_REFERENCE_BUILDER_HPP_
