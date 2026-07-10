#ifndef UR5E_BRINGUP_CONTROLLERS_WBC_GRASP_PHASE_MANAGER_HPP_
#define UR5E_BRINGUP_CONTROLLERS_WBC_GRASP_PHASE_MANAGER_HPP_

/// @file grasp_phase_manager.hpp
/// @brief Concrete `rtc::mpc::PhaseManagerBase` implementing a 5-state grasp
///        FSM for the UR5e + 10-DoF-hand bringup.
///
/// State graph (id → name, matching libstdc++ SSO budget):
///
///   0 idle ─► 1 approach ─► 2 closure ─► 3 hold ─► 4 release ─► 0 idle
///
/// Edge guards (standalone / guard-driven mode):
///
///   idle     ─► approach : `command == kApproach` AND a valid target set
///   approach ─► closure  : ‖tcp − pregrasp_pose‖ < approach_tolerance
///   closure  ─► hold     : Σ sensor[i] > force_threshold  (i ∈ frames)
///   hold     ─► release  : `command == kRelease`
///   release  ─► idle     : `command == kRelease` (hand open complete)
///   any      ─► idle     : `command == kAbort`
///   closure  ─► idle     : `failure_count >= max_failures`
///
/// The former dedicated `pre_grasp` fine-positioning state is folded into
/// `approach` (which drives to `pregrasp_pose`); residual travel to
/// `grasp_pose` happens under the contact_rich `closure` OCP. The `manipulate`
/// and `retreat` states were unused by every consumer and were removed.
///
/// All edges fire exactly once per tick; `PhaseContext::phase_changed` is
/// true on the transition tick.
///
/// Mirror mode (WBC bringup): when the WBC controller owns the FSM it calls
/// @ref ForcePhase on each of its own phase edges. The first `ForcePhase`
/// *latches* the manager (see @c mirror_latched_): subsequent ticks hold the
/// last forced id and suppress both the guard evaluation and the command bus,
/// so the manager stays a pure OCP-type / cost-table mirror of the WBC FSM
/// instead of self-advancing (e.g. a slow contact-forming closure will not
/// trip the standalone `max_failures` abort and flip the OCP back to
/// contact_light). @ref Load / @ref Init clear the latch.
///
/// Thread-safety contract (per `PhaseManagerBase`):
/// - `Update` runs on the MPC thread (RT-equivalent SCHED_FIFO).
/// - `SetTaskTarget` and `SetCommand` may come from any off-MPC thread
///   (topic callback, BT leaf, test driver). `SetTaskTarget` publishes the
///   target through @c target_seqlock_ (single-writer wait-free `rtc::SeqLock`
///   over @ref phase::GraspTargetPOD); the MPC reader is wait-free / lock-
///   free — see invariants.md RT-4.
/// - `ForcePhase` / `CurrentPhaseId` / `CurrentPhaseName` use atomics.
/// - Per-phase `PhaseCostConfig` / `ContactPlan` slots are built in `Init`
///   and stay `const` thereafter — `Update` reads them without locking.
///
/// Robot-agnostic notes:
/// - The manager does NOT hardcode UR5e joint counts or frame names. Contact
///   frames come from the `RobotModelHandler` injected at construction, and
///   the per-phase `q_posture_ref` / `F_target` sizes are enforced by
///   `PhaseCostConfig::LoadFromYaml` against that model.
/// - `ocp_type` per phase is YAML-driven ("contact_light" / "contact_rich").

#include "integrated_bringup/controllers/wbc/grasp_target.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/phase/phase_cost_config.hpp"
#include "rtc_mpc/phase/phase_manager_base.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <string>
#include <string_view>

namespace integrated_bringup::phase {

/// @brief Stable integer ids for the 5 grasp FSM states. Matches the index
///        into the `phases` YAML map loaded by @ref GraspPhaseManager::Init.
enum class GraspPhaseId : int {
  kIdle = 0,
  kApproach = 1,
  kClosure = 2,
  kHold = 3,
  kRelease = 4,
};

inline constexpr int kNumGraspPhases = 5;

/// @brief Failure modes for @ref GraspPhaseManager::Init.
enum class GraspPhaseInitError {
  kNoError = 0,
  kModelNotInitialised,  ///< RobotModelHandler unusable
  kInvalidYamlSchema,    ///< required key missing / wrong kind
  kMissingPhase,         ///< one of the 5 required phase entries absent
  kInvalidThreshold,     ///< approach_tolerance <= 0
                         ///< or force_threshold < 0 or max_failures <= 0
  kInvalidOcpType,       ///< per-phase ocp_type not in {contact_light,
                         ///< contact_rich}
  kInvalidContactIndex,  ///< active_contact_indices[i] out of model range
  kInvalidPhaseCost,     ///< PhaseCostConfig::LoadFromYaml rejected
};

/// @brief Transition thresholds + failure guard, loaded from YAML once.
struct GraspTransitionConfig {
  double approach_tolerance{0.05};  ///< m — APPROACH → CLOSURE guard (tcp-to-pregrasp)
  double force_threshold{0.5};      ///< N — Σ sensor for CLOSURE → HOLD
  int max_failures{50};             ///< consecutive CLOSURE ticks without force
};

/// @brief 5-state grasp FSM producing a `PhaseContext` per MPC tick.
class GraspPhaseManager final : public rtc::mpc::PhaseManagerBase {
 public:
  /// @param model  initialised handler; stays owned by the caller and must
  ///               outlive this manager (used only at @ref Init / @ref
  ///               SetTaskTarget time for dim validation + FK helpers).
  explicit GraspPhaseManager(const rtc::mpc::RobotModelHandler& model) noexcept;
  ~GraspPhaseManager() override = default;

  /// @brief Load thresholds + per-phase cost / contact plans from YAML.
  ///
  /// Expected schema (the caller passes the node rooted at the
  /// `grasp_phase_manager:` key):
  /// ```yaml
  /// transition:
  ///   approach_tolerance: 0.05
  ///   force_threshold: 0.5
  ///   max_failures: 50
  /// phases:
  ///   idle:        { ocp_type: "contact_light", active_contact_indices: [],
  ///                  cost: { ... PhaseCostConfig YAML ... } }
  ///   approach:    { ocp_type: "contact_light", active_contact_indices: [],
  ///                  cost: { ... } }
  ///   closure:     { ocp_type: "contact_rich",
  ///                  active_contact_indices: [0,1,2], cost: { ... } }
  ///   hold:        { ... }
  ///   release:     { ... }
  /// ```
  ///
  /// Errors leave the manager in the uninitialised state (`Initialised()
  /// == false`); callers inspect the returned enum to decide whether to
  /// abort before starting the MPC thread.
  [[nodiscard]] GraspPhaseInitError Load(const YAML::Node& cfg) noexcept;

  /// @brief PhaseManagerBase compatibility — delegates to @ref Load and
  ///        throws `std::runtime_error` on any error, since this entry is
  ///        only called off-RT during system init.
  void Init(const YAML::Node& cfg) override;

  rtc::mpc::PhaseContext Update(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                const Eigen::VectorXd& sensor, const pinocchio::SE3& tcp,
                                double t) override;

  /// @brief Update the grasp goal (and derived pre-grasp / approach-start
  ///        poses). Expected YAML:
  /// ```yaml
  /// grasp_translation: [x, y, z]
  /// grasp_rotation:    [qw, qx, qy, qz]  # unit quaternion (optional; if
  ///                                        absent, identity rotation)
  /// pregrasp_offset_local: [0, 0, 0.05]  # offset expressed in the grasp
  ///                                        frame (typically +Z above)
  /// ```
  /// Unknown / malformed payloads are logged to stderr and ignored — the
  /// FSM keeps the previous target.
  void SetTaskTarget(const YAML::Node& target) override;

  /// @brief Direct-object target setter used by tests and non-YAML wiring.
  void SetTaskTarget(const GraspTarget& target) noexcept;

  /// @brief External command bus. Idempotent; repeated sets overwrite.
  void SetCommand(GraspCommand cmd) noexcept;

  [[nodiscard]] int CurrentPhaseId() const override;
  [[nodiscard]] std::string CurrentPhaseName() const override;
  void ForcePhase(int phase_id) override;

  [[nodiscard]] bool Initialised() const noexcept { return initialised_; }

  // ── Test / diagnostic probes ────────────────────────────────────────────
  [[nodiscard]] bool HasTarget() const noexcept {
    return has_target_.load(std::memory_order_acquire);
  }

  [[nodiscard]] int FailureCount() const noexcept { return failure_count_; }

  /// @return canonical phase name for @p id; "" for invalid ids.
  [[nodiscard]] static std::string_view NameFor(int id) noexcept;

 private:
  struct PhaseSlot {
    std::string ocp_type{"contact_light"};
    rtc::mpc::PhaseCostConfig cost{};
    rtc::mpc::ContactPlan contact_plan{};
  };

  static constexpr int kNoForcedPhase = -1;

  [[nodiscard]] GraspPhaseInitError LoadTransition(const YAML::Node& cfg,
                                                   GraspTransitionConfig& out) const noexcept;
  [[nodiscard]] GraspPhaseInitError LoadPhases(
      const YAML::Node& phases_cfg, std::array<PhaseSlot, kNumGraspPhases>& out) const noexcept;

  [[nodiscard]] int EvaluateTransition(int current_id, const pinocchio::SE3& tcp,
                                       const Eigen::VectorXd& sensor, GraspCommand cmd) noexcept;
  [[nodiscard]] rtc::mpc::PhaseContext BuildContext(int id, bool changed) const;

  // Injected (non-owning) — must outlive this manager.
  const rtc::mpc::RobotModelHandler& model_;

  // Config (set in Load; const afterwards).
  GraspTransitionConfig thresholds_{};
  std::array<PhaseSlot, kNumGraspPhases> phases_{};
  bool initialised_{false};

  // Live state.
  std::atomic<int> current_id_{static_cast<int>(GraspPhaseId::kIdle)};
  std::atomic<int> forced_phase_id_{kNoForcedPhase};
  std::atomic<int> command_{static_cast<int>(GraspCommand::kNone)};
  // Set by the first ForcePhase; cleared only by Load/Init. While set, Update
  // holds the last forced id and skips guard + command evaluation (mirror
  // mode — the WBC FSM is authoritative). See the class-level "Mirror mode".
  std::atomic<bool> mirror_latched_{false};
  int failure_count_{0};  // MPC-thread only; plain int.

  // Grasp target — single-writer wait-free publication to the MPC reader.
  // `has_target_` separately tracks whether `target_seqlock_` carries a
  // meaningful payload (zero-initialised identity is treated as "no target").
  rtc::SeqLock<GraspTargetPOD> target_seqlock_{};
  std::atomic<bool> has_target_{false};
};

}  // namespace integrated_bringup::phase

#endif  // UR5E_BRINGUP_CONTROLLERS_WBC_GRASP_PHASE_MANAGER_HPP_
