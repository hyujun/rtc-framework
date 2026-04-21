#ifndef UR5E_BRINGUP_PHASE_GRASP_PHASE_MANAGER_HPP_
#define UR5E_BRINGUP_PHASE_GRASP_PHASE_MANAGER_HPP_

/// @file grasp_phase_manager.hpp
/// @brief Concrete `rtc::mpc::PhaseManagerBase` implementing an 8-state grasp
///        FSM for the UR5e + 10-DoF-hand bringup.
///
/// State graph (id → name, matching libstdc++ SSO budget):
///
///   0 idle       ─► 1 approach ─► 2 pre_grasp ─► 3 closure ─► 4 hold
///   7 release ◄─ 6 retreat    ◄─ 5 manipulate ◄─ 4 hold
///
/// Edge guards:
///
///   idle       ─► approach    : `command == kApproach` AND a valid target set
///   approach   ─► pre_grasp   : ‖tcp − pregrasp_pose‖ < approach_tolerance
///   pre_grasp  ─► closure     : ‖tcp − grasp_pose‖    < pregrasp_tolerance
///   closure    ─► hold        : Σ sensor[i] > force_threshold  (i ∈ frames)
///   hold       ─► manipulate  : `command == kManipulate`
///   manipulate ─► retreat     : `command == kRetreat`
///   retreat    ─► release     : ‖tcp − approach_start‖ < approach_tolerance
///   release    ─► idle        : `command == kRelease` (hand open complete)
///   any        ─► idle        : `command == kAbort`
///   closure    ─► idle        : `failure_count >= max_failures`
///
/// All edges fire exactly once per tick; `PhaseContext::phase_changed` is
/// true on the transition tick.
///
/// Thread-safety contract (per `PhaseManagerBase`):
/// - `Update` runs on the MPC thread.
/// - `SetTaskTarget` and `SetCommand` may come from any off-MPC thread
///   (topic callback, BT leaf, test driver); both serialise via
///   @c target_mutex_ (non-RT).
/// - `ForcePhase` / `CurrentPhaseId` / `CurrentPhaseName` use atomics.
/// - Per-phase `PhaseCostConfig` / `ContactPlan` slots are built in `Init`
///   and stay `const` thereafter — `Update` reads them without locking.
///
/// Robot-agnostic notes:
/// - The manager does NOT hardcode UR5e joint counts or frame names. Contact
///   frames come from the `RobotModelHandler` injected at construction, and
///   the per-phase `q_posture_ref` / `F_target` sizes are enforced by
///   `PhaseCostConfig::LoadFromYaml` against that model.
/// - `ocp_type` per phase is YAML-driven ("light_contact" / "contact_rich").

#include "ur5e_bringup/phase/grasp_target.hpp"

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
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <string_view>

namespace ur5e_bringup::phase {

/// @brief Stable integer ids for the 8 grasp FSM states. Matches the index
///        into the `phases` YAML map loaded by @ref GraspPhaseManager::Init.
enum class GraspPhaseId : int {
  kIdle = 0,
  kApproach = 1,
  kPreGrasp = 2,
  kClosure = 3,
  kHold = 4,
  kManipulate = 5,
  kRetreat = 6,
  kRelease = 7,
};

inline constexpr int kNumGraspPhases = 8;

/// @brief Failure modes for @ref GraspPhaseManager::Init.
enum class GraspPhaseInitError {
  kNoError = 0,
  kModelNotInitialised, ///< RobotModelHandler unusable
  kInvalidYamlSchema,   ///< required key missing / wrong kind
  kMissingPhase,        ///< one of the 8 required phase entries absent
  kInvalidThreshold,    ///< approach_tolerance / pregrasp_tolerance <= 0
                        ///< or force_threshold < 0 or max_failures <= 0
  kInvalidOcpType,      ///< per-phase ocp_type not in {light_contact,
                        ///< contact_rich}
  kInvalidContactIndex, ///< active_contact_indices[i] out of model range
  kInvalidPhaseCost,    ///< PhaseCostConfig::LoadFromYaml rejected
};

/// @brief Transition thresholds + failure guard, loaded from YAML once.
struct GraspTransitionConfig {
  double approach_tolerance{0.05}; ///< m — APPROACH → PRE_GRASP guard
  double pregrasp_tolerance{0.01}; ///< m — PRE_GRASP → CLOSURE guard
  double force_threshold{0.5};     ///< N — Σ sensor for CLOSURE → HOLD
  int max_failures{50};            ///< consecutive CLOSURE ticks without force
};

/// @brief 8-state grasp FSM producing a `PhaseContext` per MPC tick.
class GraspPhaseManager final : public rtc::mpc::PhaseManagerBase {
public:
  /// @param model  initialised handler; stays owned by the caller and must
  ///               outlive this manager (used only at @ref Init / @ref
  ///               SetTaskTarget time for dim validation + FK helpers).
  explicit GraspPhaseManager(const rtc::mpc::RobotModelHandler &model) noexcept;
  ~GraspPhaseManager() override = default;

  /// @brief Load thresholds + per-phase cost / contact plans from YAML.
  ///
  /// Expected schema (the caller passes the node rooted at the
  /// `grasp_phase_manager:` key):
  /// ```yaml
  /// transition:
  ///   approach_tolerance: 0.05
  ///   pregrasp_tolerance: 0.01
  ///   force_threshold: 0.5
  ///   max_failures: 50
  /// phases:
  ///   idle:        { ocp_type: "light_contact", active_contact_indices: [],
  ///                  cost: { ... PhaseCostConfig YAML ... } }
  ///   approach:    { ocp_type: "light_contact", active_contact_indices: [],
  ///                  cost: { ... } }
  ///   pre_grasp:   { ... }
  ///   closure:     { ocp_type: "contact_rich",
  ///                  active_contact_indices: [0,1,2], cost: { ... } }
  ///   hold:        { ... }
  ///   manipulate:  { ... }
  ///   retreat:     { ... }
  ///   release:     { ... }
  /// ```
  ///
  /// Errors leave the manager in the uninitialised state (`Initialised()
  /// == false`); callers inspect the returned enum to decide whether to
  /// abort before starting the MPC thread.
  [[nodiscard]] GraspPhaseInitError Load(const YAML::Node &cfg) noexcept;

  /// @brief PhaseManagerBase compatibility — delegates to @ref Load and
  ///        throws `std::runtime_error` on any error, since this entry is
  ///        only called off-RT during system init.
  void Init(const YAML::Node &cfg) override;

  rtc::mpc::PhaseContext Update(const Eigen::VectorXd &q,
                                const Eigen::VectorXd &v,
                                const Eigen::VectorXd &sensor,
                                const pinocchio::SE3 &tcp, double t) override;

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
  void SetTaskTarget(const YAML::Node &target) override;

  /// @brief Direct-object target setter used by tests and non-YAML wiring.
  void SetTaskTarget(const GraspTarget &target) noexcept;

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
  [[nodiscard]] GraspTransitionConfig Thresholds() const noexcept {
    return thresholds_;
  }

  /// @return canonical phase name for @p id; "" for invalid ids.
  [[nodiscard]] static std::string_view NameFor(int id) noexcept;

  /// @return ocp_type key for @p id (valid after @ref Load). "" on invalid.
  [[nodiscard]] std::string_view OcpTypeFor(int id) const noexcept;

private:
  struct PhaseSlot {
    std::string ocp_type{"light_contact"};
    rtc::mpc::PhaseCostConfig cost{};
    rtc::mpc::ContactPlan contact_plan{};
  };

  static constexpr int kNoForcedPhase = -1;

  [[nodiscard]] GraspPhaseInitError
  LoadTransition(const YAML::Node &cfg,
                 GraspTransitionConfig &out) const noexcept;
  [[nodiscard]] GraspPhaseInitError
  LoadPhases(const YAML::Node &phases_cfg,
             std::array<PhaseSlot, kNumGraspPhases> &out) const noexcept;

  [[nodiscard]] int EvaluateTransition(int current_id,
                                       const pinocchio::SE3 &tcp,
                                       const Eigen::VectorXd &sensor,
                                       GraspCommand cmd) noexcept;
  [[nodiscard]] rtc::mpc::PhaseContext BuildContext(int id, bool changed) const;

  // Injected (non-owning) — must outlive this manager.
  const rtc::mpc::RobotModelHandler &model_;

  // Config (set in Load; const afterwards).
  GraspTransitionConfig thresholds_{};
  std::array<PhaseSlot, kNumGraspPhases> phases_{};
  bool initialised_{false};

  // Live state.
  std::atomic<int> current_id_{static_cast<int>(GraspPhaseId::kIdle)};
  std::atomic<int> forced_phase_id_{kNoForcedPhase};
  std::atomic<int> command_{static_cast<int>(GraspCommand::kNone)};
  int failure_count_{0}; // MPC-thread only; plain int.

  // Grasp target — updated off-MPC-thread.
  mutable std::mutex target_mutex_{};
  GraspTarget target_{};
  std::atomic<bool> has_target_{false};
};

} // namespace ur5e_bringup::phase

#endif // UR5E_BRINGUP_PHASE_GRASP_PHASE_MANAGER_HPP_
