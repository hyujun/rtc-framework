#ifndef RTC_MPC_PHASE_LOCOMOTION_PHASE_MANAGER_HPP_
#define RTC_MPC_PHASE_LOCOMOTION_PHASE_MANAGER_HPP_

/// @file locomotion_phase_manager.hpp
/// @brief The phase manager of the kinodynamic OCP: each MPC tick it advances
///        the gait scheduler, builds the per-node horizon reference from the
///        measured state and the latest filtered command, and hands it to the
///        OCP through `PhaseContext::horizon`.
///
/// ## Per tick — `Update(state, q, v, sensor, tcp, t)`
///
///  1. the command: the latest @ref SetCommand (before the first, a
///     `FilteredCommand{}` — not valid, base height 0);
///  2. the node grid: anchored at the first tick's `t` (until the next
///     `Load`), node spacing `dt`; the tick runs at `t_q = Time(m)`, `m` the
///     grid cell holding `t` (@ref NodeIndexAt) — the horizon moves in whole
///     nodes whatever the MPC period;
///  3. `GaitScheduler::Advance(t_q, command, measured)`, `measured` the
///     snapshot's base velocity on the yaw-aligned axes (`v_x`, `v_y`) and its
///     world yaw rate (`omega_z`);
///  4. `StageReferenceBuilder::Build(scheduler, state, command, grid, m)` —
///     node `k` at `Time(m + k)`;
///  5. the context: `ocp_type` `"kinodynamic"`, `cost_config.horizon_length` /
///     `dt` the configured ones, `horizon` the builder's active slot,
///     `phase_changed` false (a contact mode change is not a topology change
///     for this OCP), `phase_id` the running gait's index, `phase_name` empty
///     (no per-tick string), `on_node_grid` true with `node_index` `m` and
///     `horizon_offset` `t − t_q`.
///
/// A failure of step 2 or 3 gives a context with a null `horizon` (the OCP
/// refuses it, so nothing is solved from a stale reference) and is kept in
/// @ref LastUpdateError. So does the overload without a snapshot (the base
/// class one): the builder needs the snapshot's provenance. The owner stores
/// the command filter's output (its rest base height included) with
/// @ref SetCommand before the thread starts, or the builder refuses the zero
/// base height.
///
/// ## YAML (`Load` / `Init`; every key required, an unknown key refused)
///
/// ```yaml
/// horizon_nodes: 32   # the OCP's node count (1 … kMaxHorizon)
/// dt: 0.035           # [s] the OCP's node spacing
/// gait: { … }         # LoadGaitTable — contact names given at construction
/// swing: { … }        # LoadSwingConfig
/// terrain: { … }      # LoadTerrainConfig
/// reference: { … }    # LoadReferenceConfig — the model's joints after the root
/// ```
///
/// ## Threads and real time
///
/// `Load` / `Init` are configure time (they allocate) and run before the MPC
/// thread starts. `Update` runs on the MPC thread (an RT path): it allocates,
/// throws and locks nothing. `SetCommand` may be called from one other thread
/// at a time (a seqlock slot: single writer, the reader never blocks it).
/// `Update`'s read of that slot retries while a store is in progress, so
/// the writer must never be scheduled on the MPC thread's core: preempted
/// mid-store by the higher-priority reader, it would stall the tick. Two
/// concurrent writers can tear the slot — wire one caller only (a timer in
/// a mutually exclusive callback group).
/// `RequestGait`, `SetPolicyEnabled`, `RequestReset` and `ForcePhase` may be
/// called from any thread (the scheduler's atomics, applied by the next
/// `Update`); `CurrentPhaseId`, `CurrentPhaseName` and `LastUpdateError` too.

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_manager_base.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace rtc::mpc {

/// @brief Failure modes of @ref LocomotionPhaseManager::Load and of its
///        `Update` (@ref LocomotionPhaseManager::LastUpdateError).
enum class LocomotionPhaseError {
  kNoError = 0,
  // Load
  kNotAMap,              ///< the node is not a map
  kUnknownKey,           ///< a key outside the schema
  kMissingKey,           ///< a schema key without a value
  kInvalidHorizon,       ///< `horizon_nodes` not an integer in `[1, kMaxHorizon]`, or `dt`
                         ///< not finite and positive
  kModelNotInitialised,  ///< the model handler is not initialised
  kContactNameMismatch,  ///< not one contact name per model contact
  kInvalidGait,          ///< `LoadGaitTable` refused `gait:`
  kInvalidSwing,         ///< `LoadSwingConfig` refused `swing:`
  kInvalidTerrain,       ///< `LoadTerrainConfig` refused `terrain:`
  kInvalidReference,     ///< `LoadReferenceConfig` refused `reference:`
  kSchedulerInitFailed,  ///< `GaitScheduler::Init` refused the table or horizon
  kReferenceInitFailed,  ///< `StageReferenceBuilder::Init` refused
  // Update
  kNotInitialised,        ///< Update before a successful Load
  kNoSnapshot,            ///< the overload without a snapshot was called
  kAdvanceFailed,         ///< `GaitScheduler::Advance` refused the tick
  kReferenceBuildFailed,  ///< `StageReferenceBuilder::Build` refused the tick
};

/// @brief The gait phase manager (file comment).
class LocomotionPhaseManager final : public PhaseManagerBase {
 public:
  /// @param model          initialised before `Load`; must outlive this manager
  /// @param contact_names  the gait table's contact names, one per model
  ///                       contact in the model's contact order (bit k)
  LocomotionPhaseManager(const RobotModelHandler& model, std::vector<std::string> contact_names);

  /// @brief Load the YAML (file comment) and set the scheduler and the builder
  ///        up. Allocates. On failure the manager is uninitialised.
  ///        Configure time only: a Load restarts the node grid (anchored
  ///        again at the next Update), so re-loading while a handler keeps
  ///        iterates solved on the old grid is not supported — rebuild or
  ///        reset the handler with it.
  [[nodiscard]] LocomotionPhaseError Load(const YAML::Node& cfg) noexcept;

  /// @brief `Load`, throwing `std::runtime_error` on any error (this entry
  ///        returns void).
  void Init(const YAML::Node& cfg) override;

  /// @brief The tick with the snapshot (file comment). Does not allocate.
  PhaseContext Update(const MPCStateSnapshot& state, const Eigen::VectorXd& q,
                      const Eigen::VectorXd& v, const Eigen::VectorXd& sensor,
                      const pinocchio::SE3& tcp, double t) override;

  /// @brief Without a snapshot there is no reference: the context's horizon
  ///        is null and the error `kNoSnapshot`.
  PhaseContext Update(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                      const Eigen::VectorXd& sensor, const pinocchio::SE3& tcp, double t) override;

  /// @brief Ignored: the command arrives through @ref SetCommand.
  void SetTaskTarget(const YAML::Node& target) override;

  /// @return the running gait's index (as of the last `Update`).
  [[nodiscard]] int CurrentPhaseId() const override;

  /// @return the running gait's name (allocates — not for the MPC thread).
  [[nodiscard]] std::string CurrentPhaseName() const override;

  /// @brief `RequestGait(phase_id)`; a negative id is dropped.
  void ForcePhase(int phase_id) override;

  // ── Any thread ──────────────────────────────────────────────────────────────

  /// @brief Store the command the next `Update` reads. One writer thread at a
  ///        time. Does not allocate.
  void SetCommand(const FilteredCommand& cmd) noexcept { command_slot_.Store(cmd); }

  /// @brief Forwarded to `GaitScheduler::RequestGait` (policy off, latched).
  void RequestGait(std::uint32_t index) noexcept { scheduler_.RequestGait(index); }

  void SetPolicyEnabled(bool enabled) noexcept { scheduler_.SetPolicyEnabled(enabled); }

  void RequestReset() noexcept { scheduler_.RequestReset(); }

  /// @return the outcome of the last `Update` (`kNotInitialised` before one).
  [[nodiscard]] LocomotionPhaseError LastUpdateError() const noexcept {
    return static_cast<LocomotionPhaseError>(last_update_error_.load(std::memory_order_relaxed));
  }

  // ── Owner thread (the MPC thread, or configure time) ───────────────────────

  [[nodiscard]] bool Initialised() const noexcept { return initialised_; }

  [[nodiscard]] const GaitScheduler& scheduler() const noexcept { return scheduler_; }

  [[nodiscard]] const StageReferenceBuilder& builder() const noexcept { return builder_; }

  /// @return the command the last `Update` read.
  [[nodiscard]] const FilteredCommand& command() const noexcept { return command_; }

  /// @return the measured velocity the last `Update` gave the scheduler.
  [[nodiscard]] const PlanarVelocity& measured() const noexcept { return measured_; }

  [[nodiscard]] int horizon_nodes() const noexcept { return horizon_nodes_; }

  [[nodiscard]] double dt() const noexcept { return dt_; }

  /// @return the node grid (its anchor valid once an `Update` ran after `Load`).
  [[nodiscard]] const NodeGrid& grid() const noexcept { return grid_; }

  /// @return the grid index of the last successful `Update`'s node 0.
  [[nodiscard]] std::int64_t node_index() const noexcept { return node_index_; }

  /// @return the snapshot's base velocity as the scheduler reads it: the
  ///        LOCAL linear velocity rotated to the yaw-aligned axes, and the
  ///        world z of the angular velocity. Zero for a snapshot whose base
  ///        quaternion is the zero vector.
  [[nodiscard]] static PlanarVelocity MeasuredPlanarVelocity(
      const MPCStateSnapshot& state) noexcept;

 private:
  [[nodiscard]] PhaseContext Context(const HorizonReference* horizon) const;
  [[nodiscard]] PhaseContext GridContext(const HorizonReference* horizon, std::int64_t index,
                                         double offset) const;

  void SetLastError(LocomotionPhaseError e) noexcept {
    last_update_error_.store(static_cast<int>(e), std::memory_order_relaxed);
  }

  const RobotModelHandler* model_;
  std::vector<std::string> contact_names_;
  std::vector<std::string> gait_names_{};

  GaitScheduler scheduler_{};
  StageReferenceBuilder builder_{};
  rtc::SeqLock<FilteredCommand> command_slot_{};
  FilteredCommand command_{};
  PlanarVelocity measured_{};

  int horizon_nodes_{0};
  double dt_{0.0};
  NodeGrid grid_{};
  std::int64_t node_index_{0};
  bool anchored_{false};  ///< the grid's anchor is set (first Update after Load)
  bool initialised_{false};

  std::atomic<std::uint32_t> current_gait_{0};
  std::atomic<int> last_update_error_{static_cast<int>(LocomotionPhaseError::kNotInitialised)};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_PHASE_LOCOMOTION_PHASE_MANAGER_HPP_
