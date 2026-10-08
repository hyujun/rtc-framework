#pragma once

#include "rtc_math/se3/axis_align.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage C-1 / WBC Kinematic QP: ClikReferenceGenerator — velocity-level
// closed-loop IK (CLIK) producing a one-step (q_ref, v_ref) low-level command
// anchored at the measured state. The Kinematic WBC position backbone of the
// 2-QP (Kinematic CLIK-QP + Dynamic TSID-ID) reorganisation.
//
// Single weighted box-constrained QP (decision variable z = v ∈ Rᴺ, N == nv;
// the reduced "wbc" model contract nq == nv == n_actuated). The original
// 3-level damped-pseudo-inverse + nullspace projection is promoted to one
// proxsuite QP whose weighted cost soft-prioritises L1 ≫ L2 ≫ L3:
//   min_v  w_task·‖J_task·v − r_task‖²        (L1: TCP tracking, arm only)
//        + w_arm ·‖S_arm·v  − v_post_arm‖²    (L2: arm posture)
//        + w_hand·‖S_hand·v − v_post_hand‖²   (L3: hand posture)
//        + μ²·‖v‖²                            (damping, absorbs J♯ damping)
//   s.t.  lᵢ ≤ vᵢ ≤ uᵢ                        (per-joint velocity ∩ position box)
// with
//   J_task ∈ R⁶ˣᴺ : rf.J's arm columns (hand columns 0; an arm-tip frame's
//                   hand columns are ~0 by kinematics, zeroed for safety),
//   r_task = Kx ⊙ e_x (6-vec SE(3) error, base-frame aligned — see below),
//   S_arm / S_hand   : index-selection of the arm / hand velocity components,
//   v_post = K·(q_des − q) on the respective index set.
// Assembled as a fixed-dim QP (n_vars = N, n_eq = 0, n_ineq = N, C = Iₙ):
//   H = w_task·J_taskᵀJ_task + diag(w_arm@arm, w_hand@hand) + μ²·I,
//   g = −(w_task·J_taskᵀr_task + scatter(w_arm·v_post_arm, w_hand·v_post_hand)).
// w_task ≫ w_arm,w_hand ≫ μ² makes L2/L3 live in L1's residual (nullspace)
// space — a soft-priority approximation of the original hard nullspace
// projection (μ²/w_task reproduces the original damped-inverse damping).
//
// Box bounds (per velocity index i, dt = control period):
//   lᵢ = max(−v_limit, (q_minᵢ − qᵢ)/dt),  uᵢ = min(+v_limit, (q_maxᵢ − qᵢ)/dt).
// When already limit-violating (qᵢ out of [q_min,q_max]) the box can invert
// (lᵢ > uᵢ); it is forced feasible by lᵢ ← min(lᵢ, uᵢ) so only motion back
// toward the feasible set is allowed, and the collapsed value is re-clamped to
// [−v_limit, +v_limit] — recovery runs at the full limit but never past it, in
// either direction (the raw collapse is bounded below q_min and unbounded
// above q_max). v_limit ≤ 0 disables the velocity bound, which also leaves the
// collapsed recovery unbounded; an empty q_min/q_max disables the position
// bound (±∞).
//
// Error convention: the SHARED helper kinematics/se3_error.hpp
// (ComputeTaskPoseError) — identical to SE3Task / ObjectSE3Task (dynamics WBC),
// the U1 unification. It is the LWA-frame BodyLog6 error
// twistLocalToWorld(log6(T⁻¹T_d)), matching the registered-frame
// LOCAL_WORLD_ALIGNED Jacobian rf.J. rtc_math's quaternion+atan2 log3 is robust
// at θ=π (no explicit clamp). The velocity law stays first-order (U1):
// r_task = Kx ⊙ e_x — no Jlog6⁻¹ on this kinematic path. No separate FK —
// the caller registers tip/base frames on the shared PinocchioCache and
// calls cache.Update() once per tick.
//
// One-step target with selectable anchoring: q_ref = q_anchor + v_ref·dt.
//   reseed_anchor = true  → q_anchor = q_meas (measured anchoring; the velocity
//     correction rides on the freshly measured state — the classic closed-loop
//     CLIK form). This is the default and the post-failure / first-call value.
//   reseed_anchor = false → q_anchor = q_ref_prev (carry-forward: the desired
//     position integrates from the PREVIOUS desired, decoupled from per-tick
//     measured noise — mirrors DemoTaskController's desired_q_ integrator).
// The caller drives reseed_anchor from "a new goal arrived / phase re-entry"
// events so the command re-anchors to measured only at those edges. Only the
// integration anchor switches; the velocity solve (J_task, e_x, posture, box)
// stays measured-based, so v_ref remains a closed-loop correction. To bound the
// open-loop drift that carry-forward can accumulate when low-level tracking
// lags, anchor_drift_max clamps |q_ref_i − q_meas_i| per joint (≤ 0 → off).
//
// Requires nq == nv (reduced revolute/prismatic tree). Compute() returns
// false on precondition violation, QP non-convergence, or non-finite result;
// the caller holds last for that tick and must not consume the outputs (the
// failure branch leaves q_ref = q_meas, v_ref = 0, and forces the next call to
// re-anchor to measured).
//
// Acceleration constraint (Config::accel_constraint, dynamic_catching decision
// K). With v̇ ≈ (v − v_c)/dt every form is LINEAR in v, so each is a set of
// extra inequality rows below the box (C = [Iₙ; C_a]). v_c is the velocity the
// cache was evaluated at (cache.v — the command velocity under D-6, the same
// state h and J̇ are evaluated at); only its ARM entries enter, because the
// hand is commanded elsewhere and its acceleration is not this solve's:
//   kBox       — the per-joint window v_prev ± a_max·dt folded into the box
//                (the legacy option; no extra rows, bit-identical output);
//   kKinematic — the task acceleration J·v̇ + J̇·v_c of the rows the cost
//                tracks, |·| ≤ task_accel_max_{linear,angular} per row
//                (J̇·v_c = the registered frame's classical drift dJv). ONLY
//                those rows: null-space motion (e.g. roll about the approach
//                axis in the 5-row overload) has no acceleration bound but the
//                velocity box — a joint may still step to its velocity limit
//                in one tick. The dynamic form bounds every arm joint;
//   kDynamic   — the arm joint torque M_arm(q)·v̇_arm + h(q, v_c) on the ARM
//                indices, |·| ≤ eta_tau·tau_max (M and h are the cache's).
// Each row is scaled to unit norm (the feasible set is unchanged; rows of M/dt
// or J/dt are 10²–10⁵ against the box's 1, which made ProxQP report
// PRIMAL_INFEASIBLE on feasible problems). The rows are hard: when they cannot
// hold together with the velocity ∩ position box the call FAILS
// (LastSolve().accel_rows_violated, converged false) rather than return a
// command that breaks them — there is no per-joint resolution as the box has,
// because the rows couple joints. With rows, a failure and ResetAnchor() also
// drop the solver's warm start. M carries no rotor inertia (a URDF has none):
// the margin eta_tau < 1 is what covers it.
//
// Several frames in one solve (the MultiFrameInput overload, E2-F04). The cost
// above with a SUM of task terms — one per FrameTask, each an SE3 task (6
// rows) or a position + approach-axis task (3 + 2 rows) with its own gain,
// weight and feed-forward — and the posture term over Config::posture_groups
// instead of the fixed arm / hand pair:
//   min_v  Σ_k w_k·‖J_k·v − r_k‖² + Σ_g w_g·‖S_g·v − v_post,g‖² + μ²·‖v‖²
//   v_post,g = K_g·(q_des − q) + q̇_ff      (q̇_ff optional, on every group)
// The box, the acceleration constraint, the anchoring and the failure branch
// are the single-task ones. A task with base_frame_idx ≥ 0 is RELATIVE: its
// rows are the motion of the frame with respect to the base frame, in the
// base frame's axes,
//   J_rel,ang = R_bᵀ·(J_t,ang − J_b,ang)
//   J_rel,lin = R_bᵀ·(J_t,lin − J_b,lin + [p_t − p_b]ₓ·J_b,ang)
// (J_t / J_b the two frames' LOCAL_WORLD_ALIGNED Jacobians, R_b / p_b the
// base frame's pose), so the error — taken in the base frame — and the rows
// share their axes, and a joint upstream of both frames gets a zero column:
// it cannot be asked to serve that task. With one universe-base task the call
// reproduces the matching single-task overload bit for bit. That is NOT so
// for a registered base: the SE3 overload pairs its base-aligned error with
// the world-aligned Jacobian (exact only while the base frame's axes are the
// world's), where this overload uses J_rel.
//
// All allocations happen in Init(); Compute() is RT-safe and noexcept.
// ────────────────────────────────────────────────
class ClikReferenceGenerator {
 public:
  /// Which acceleration constraint the QP carries (see the file header).
  enum class AccelConstraint : std::uint8_t { kBox, kKinematic, kDynamic };

  struct Config {
    std::vector<int> arm_v_idx;   // arm velocity indices, Pinocchio order (≥1)
    std::vector<int> hand_v_idx;  // hand velocity indices (may be empty)
    double damping_sq{1e-4};      // μ² damping (H regularization, finite and > 0)
    double v_limit{1.5};          // per-joint |v_ref| clamp [rad/s], finite; ≤ 0 → off
    // Soft-priority weights (w_task ≫ w_arm,w_hand ≫ damping_sq). μ²/w_task is
    // the effective L1 damped-inverse damping — keep w_task=1 to reproduce the
    // legacy damped right-inverse at damping_sq.
    double w_task{1.0};   // L1 TCP tracking weight (finite and > 0)
    double w_arm{1e-2};   // L2 arm posture weight (finite and >= 0)
    double w_hand{1e-2};  // L3 hand posture weight (finite and >= 0)
    // Per-joint position limits [nv] for the position-aware velocity box
    // (lᵢ/uᵢ above). Empty → position bound disabled (velocity box only); both
    // sides must be empty or both full-nv. Every component must be FINITE and
    // ordered q_minᵢ ≤ q_maxᵢ (equality = a locked joint, allowed) — Init()
    // throws otherwise. ±inf is not an accepted "unbounded" encoding here:
    // leave the box empty instead. A caller deriving the box from a model that
    // reports unbounded joints (Pinocchio uses ±inf for continuous joints) must
    // therefore drop the box rather than forward the infinities. See Init().
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    // Anti-windup clamp on the carry-forward anchor: max |q_ref_i − q_meas_i|
    // [rad] the integrated desired may lead/lag the measured state. ≤ 0 → off.
    // Only bites in carry-forward mode (reseed_anchor=false) under tracking lag;
    // on a reseed tick the gap is just v_ref·dt.
    double anchor_drift_max{0.0};  // rad; finite; ≤ 0 → off
    // ProxQP outer-iteration cap (≥ 1). The default is the value CLIK always
    // used (QPSolverConfig), so an unset field keeps the legacy solve. Hitting
    // the cap is a failed Compute() with LastSolve().status =
    // PROXQP_MAX_ITER_REACHED (G5-C3).
    int max_iter{20};
    // Per-joint velocity limits [nv] (rad/s or m/s), each finite and > 0.
    // Empty → the scalar v_limit applies to every joint (legacy). When set it
    // replaces v_limit everywhere v_limit is used, including the re-clamp of a
    // collapsed position box.
    Eigen::VectorXd v_limit_per_joint;
    // Acceleration box [nv] (rad/s² or m/s²), each finite and > 0; empty → off.
    // Intersects the velocity ∩ position box with v_prev ± a_max·dt, v_prev =
    // the v_ref of the last successful Compute() (0 after Init / ResetAnchor and
    // after a failed call, whose output command is v_ref = 0).
    // When the intersection is empty the acceleration window wins: the joint
    // gets l = u = clamp(v*, v_prev − a·dt, v_prev + a·dt), v* the point of the
    // velocity ∩ position interval nearest v_prev, and LastSolve() raises
    // bound_conflict with the joint's bit in conflict_mask (L5 §4.3). Requires
    // nv ≤ 64 (the mask width). The same a_max must feed the planner's
    // reach-time check so the plan matches what CLIK executes (D-16).
    Eigen::VectorXd a_max;
    // Smoothing weight w_s ≥ 0 (finite): adds (w_s/2)·‖v − v_prev‖² to the
    // cost, pulling the command toward the previous one (L5 §4.3). 0 → off.
    // Keep w_s ≪ w_task like the posture weights, or it competes with tracking.
    double w_smooth{0.0};
    // Command-value evaluation (D-6, L5 §4.2). The caller passes a cache
    // updated at the COMMAND state q_c (its previous QRef() on the arm), not
    // at the measured state, so error, Jacobian and box are evaluated along
    // the commanded path and servo lag stays out of the loop. Then:
    //   - the anchor is cache.q every tick (reseed_anchor is ignored);
    //   - after the first successful call (and until ResetAnchor, failures
    //     included), cache.q on the arm indices must equal the previous
    //     QRef() (|Δ| ≤ 1e-12) — otherwise the call fails
    //     with LastSolve().command_mismatch (a wiring error: a measured q
    //     would silently turn this back into measured-state CLIK). Hand
    //     indices are not checked: the hand is commanded elsewhere (L6);
    //   - anchor_drift_max must be ≤ 0 (Init throws): there is no measured q
    //     to bound against; tracking is supervised outside (L7 TRACK_ERR).
    bool evaluate_at_command{false};
    // Weight of the two approach-axis rows of the position + axis Compute()
    // overload (L5 §4.3 w_a; finite and > 0). Unused by the SE3 overload.
    double w_axis{0.5};
    // The acceleration constraint (file header). kBox reads `a_max` above;
    // the other two refuse a set `a_max` (the constraint is ONE of the three)
    // and read their own fields, which kBox in turn refuses when set.
    AccelConstraint accel_constraint{AccelConstraint::kBox};
    // kKinematic: bound on each tracked task row's acceleration — the
    // position rows [m/s²] and the rotation / approach-axis rows [rad/s²].
    // Finite and > 0 when kKinematic, 0 otherwise.
    double task_accel_max_linear{0.0};
    double task_accel_max_angular{0.0};
    // kDynamic: torque limits [nv] (N·m or N), finite and > 0 on the arm
    // indices, finite and >= 0 elsewhere (read only on the arm); empty
    // otherwise. The bound is eta_tau·tau_max with eta_tau in (0, 1] — 0
    // (unset) otherwise, so a margin set for a form that is not selected is
    // refused rather than silently unused.
    Eigen::VectorXd tau_max;
    double eta_tau{0.0};
    // ── The multi-frame call (MultiFrameInput). Unset, every field below
    //    leaves the two single-task overloads as they were. ──
    // Most tasks one call may carry, 1..16. Sizes the per-task workspace and,
    // for kKinematic, the row block (6 rows per task).
    int max_frame_tasks{1};
    // Accept FrameTask::base_frame_idx >= 0, i.e. relative tasks (file
    // header). Off → such a task is refused at the call. Not available with
    // kKinematic — the rows' drift would need the base frame's terms too —
    // so Init() throws on the pair.
    bool relative_tasks{false};
    // Velocity indices the task rows of the multi-frame call may use; every
    // other column is zero. Empty → arm_v_idx, which is what the single-task
    // overloads always use. Range-checked, no duplicates. kDynamic bounds the
    // torque on arm_v_idx only: a task column outside it moves a joint whose
    // torque this solve does not bound.
    std::vector<int> task_v_idx;

    // Posture groups: disjoint velocity index sets, each with a weight (finite
    // and >= 0) and a gain of its own (SetPostureGroupGain). Empty → the two
    // groups {arm_v_idx, w_arm} and {hand_v_idx, w_hand}, in that order. When
    // set, w_arm / w_hand are not read, and SetPostureGains(ka, kh) addresses
    // groups 0 and 1. The groups serve every Compute() overload.
    struct PostureGroup {
      std::vector<int> v_idx;
      double weight{0.0};
    };

    std::vector<PostureGroup> posture_groups;
  };

  // Pre-allocates all workspaces and validates the config (indices in
  // [0, nv), no duplicates, arm/hand disjoint, damping_sq > 0, and a finite
  // ordered position box — NUM-7: the size checks alone are not a finiteness
  // gate, because Compute()'s std::max/std::min box assembly launders a
  // non-finite bound into the velocity limit and leaves the outputs finite).
  // Non-RT — call from on_configure; throws std::runtime_error on invalid
  // config.
  void Init(int nv, const Config& config);

  // Gains (RT-safe setters; the controller forwards its SeqLock'd gains).
  void SetTaskGain(const Eigen::Matrix<double, 6, 1>& kx) noexcept { kx_ = kx; }

  /// Gains of posture groups 0 and 1 — the arm and the hand unless
  /// Config::posture_groups says otherwise. A group that does not exist is
  /// skipped.
  void SetPostureGains(double ka, double kh) noexcept {
    if (!posture_.empty()) {
      posture_[0].gain = ka;
    }
    if (posture_.size() > 1) {
      posture_[1].gain = kh;
    }
  }

  /// Gain K_g [1/s] of posture group `g` (Config::posture_groups order).
  /// Returns false, changing nothing, when there is no such group.
  [[nodiscard]] bool SetPostureGroupGain(int g, double k) noexcept {
    if (g < 0 || static_cast<std::size_t>(g) >= posture_.size()) {
      return false;
    }
    posture_[static_cast<std::size_t>(g)].gain = k;
    return true;
  }

  /// Approach-axis gain K_a [1/s] of the position + axis overload (the
  /// position gain is SetTaskGain()'s linear part).
  void SetAxisGain(double ka) noexcept { k_axis_ = ka; }

  /// Target of the position + approach-axis task (dynamic_catching L5 §4.2),
  /// all in the base frame of the call. Roll about the axis is left free —
  /// the arm posture term resolves it.
  struct PositionAxisTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};             ///< frame origin [m]
    Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};                ///< desired frame +z, unit
    Eigen::Vector3d linear_velocity_ff{Eigen::Vector3d::Zero()};   ///< [m/s]
    Eigen::Vector3d angular_velocity_ff{Eigen::Vector3d::Zero()};  ///< [rad/s]
  };

  // One CLIK step anchored at the measured (cache.q, cache.v) of this tick.
  //   tcp_frame_idx / base_frame_idx : indices into cache.registered_frames
  //     (base_frame_idx < 0 → universe fast-path, like SE3Task).
  //   placement_des : desired TCP pose **in the base frame**
  //     (SetSe3Reference contract of SE3Task).
  //   q_posture_des : full [nq] desired posture; arm/hand entries are
  //     selected through the Config index sets.
  // Returns false on dimension mismatch / invalid frame index / dt ≤ 0 /
  // non-finite result — outputs must not be consumed in that case (the
  // finite-guard branch leaves q_ref = q_meas, v_ref = 0 as a safe value;
  // precondition branches leave stale values).
  // reseed_anchor: true = anchor q_ref at the measured state this tick (default,
  // closed-loop); false = carry forward from the previous q_ref. The first call
  // after Init (and the call after any failure) always re-anchors to measured
  // regardless, so the integrator never starts from an undefined anchor.
  // twist_ff (optional, D-5): desired TCP twist [linear; angular] in the same
  // axes as the pose error (base-frame aligned), added as
  // r_task = Kx ⊙ e_x + twist_ff. nullptr → the legacy law. A non-finite
  // twist_ff fails the call before the solve.
  [[nodiscard]] bool Compute(const PinocchioCache& cache, int tcp_frame_idx, int base_frame_idx,
                             const pinocchio::SE3& placement_des,
                             const Eigen::VectorXd& q_posture_des, double dt,
                             bool reseed_anchor = true,
                             const Eigen::Matrix<double, 6, 1>* twist_ff = nullptr) noexcept;

  // Position (3 rows) + approach axis (2 rows) step for a frame whose LOCAL
  // +z must point along target.axis, e.g. a catch frame (L5 §4.2):
  //   J_p = rf.J rows 0–2 (LOCAL_WORLD_ALIGNED),  r_p = Kx[0:3] ⊙ e_p + v_ff
  //   J_a = S·R_WCᵀ·rf.J rows 3–5,                r_a = S·R_WCᵀ·(K_a·e_a + ω_ff)
  // with S = [I₂ 0] (LOCAL x, y), e_a = rtc::math::se3::AxisAlignError(z_C, a)
  // and everything expressed in world-aligned axes: base_frame_idx only moves
  // the target into the world (unlike the SE3 overload, whose error stays in
  // base-aligned axes). Cost w_task‖J_p v − r_p‖² + w_axis‖J_a v − r_a‖² plus
  // the same posture, damping, smoothing and boxes as the SE3 overload.
  // Fails before the solve on a non-finite target or an axis that is not
  // unit (AxisAlignError invalid); LastAxisRegion() reports the branch.
  // qd_posture_ff (optional, [nv]): posture velocity feed-forward, added to
  // every posture group's reference — v_post = K·(q_des − q) + q̇_ff — so a
  // posture that MOVES (a planned joint trajectory) is tracked without the
  // lag a pure position reference leaves. nullptr → the law above, bit for
  // bit. A wrong size or a non-finite entry fails the call before the solve.
  [[nodiscard]] bool Compute(const PinocchioCache& cache, int frame_idx, int base_frame_idx,
                             const PositionAxisTarget& target, const Eigen::VectorXd& q_posture_des,
                             double dt, bool reseed_anchor = true,
                             const Eigen::VectorXd* qd_posture_ff = nullptr) noexcept;

  /// Which rows a FrameTask contributes.
  enum class TaskKind : std::uint8_t { kSe3, kPositionAxis };

  /// One frame task of the multi-frame Compute() (file header). The fields of
  /// the kind that is not selected are not read.
  struct FrameTask {
    TaskKind kind{TaskKind::kSe3};
    /// Index into cache.registered_frames of the frame the task moves.
    int frame_idx{-1};
    /// −1 → universe: the target is in the world and the rows are the
    /// frame's LOCAL_WORLD_ALIGNED ones. >= 0 → a RELATIVE task (needs
    /// Config::relative_tasks): target, feed-forward and rows are all in that
    /// registered frame. A base equal to frame_idx is refused.
    int base_frame_idx{-1};
    /// kSe3: desired frame pose in the base frame.
    pinocchio::SE3 placement_des{pinocchio::SE3::Identity()};
    /// kSe3, optional: the time derivative of placement_des — origin velocity
    /// and angular velocity [linear; angular] in the base frame's axes (the
    /// axes of the pose error), NOT a spatial or body twist. nullptr → none.
    const Eigen::Matrix<double, 6, 1>* twist_ff{nullptr};
    /// kPositionAxis: target in the base frame (its two feed-forwards too).
    PositionAxisTarget target;
    /// Task gain K [1/s]: kSe3 reads all six [linear; angular], kPositionAxis
    /// the linear three.
    Eigen::Matrix<double, 6, 1> gain{Eigen::Matrix<double, 6, 1>::Zero()};
    /// kPositionAxis: approach-axis gain K_a [1/s].
    double gain_axis{0.0};
    /// Weight of the task's rows (kSe3: all six; kPositionAxis: the three
    /// position rows). Finite and > 0.
    double weight{1.0};
    /// kPositionAxis: weight of the two approach-axis rows. Finite and > 0.
    double weight_axis{0.5};
    /// Cap on the feedback part of the reference, per block: ‖K_p ⊙ e_p‖ ≤
    /// fb_lin_max [m/s] and ‖K_o ⊙ e_o‖ (kPositionAxis: ‖K_a·e_a‖) ≤
    /// fb_ang_max [rad/s]. A block over its cap is scaled back onto it, which
    /// keeps its direction; the two blocks scale separately, so the 6D
    /// direction can change. 0 → off (the reference is K ⊙ e). The
    /// feed-forward is added AFTER the cap: the reference may exceed it by
    /// the feed-forward. A caller that expects rotation errors near π sets
    /// fb_ang_max — the rotation error's direction is discontinuous there
    /// (SolveDiagnostics::rot_near_pi), and the cap bounds what that flip
    /// can command.
    double fb_lin_max{0.0};
    double fb_ang_max{0.0};
  };

  /// Input of the multi-frame Compute().
  struct MultiFrameInput {
    /// 1 .. Config::max_frame_tasks tasks. Task 0 is the one TcpErrorNorm(),
    /// Manipulability() and — when it is a position + axis task —
    /// LastAxisRegion() / PositionErrorNorm() / AxisErrorAngle() report.
    std::span<const FrameTask> tasks;
    /// Desired posture [nq]; each posture group reads its own entries.
    const Eigen::VectorXd* q_posture_des{nullptr};
    /// Optional posture velocity feed-forward [nv] (see the axis overload).
    const Eigen::VectorXd* qd_posture_ff{nullptr};
    double dt{0.0};
    bool reseed_anchor{true};
  };

  // Several frame tasks in one solve (file header). Anchoring, the box, the
  // acceleration constraint and the failure branch are the single-task ones.
  // Refused BEFORE the solve, outputs untouched and
  // LastSolve().rejected_input set: a task count outside
  // [1, Config::max_frame_tasks]; a frame index out of range; a relative task
  // without Config::relative_tasks or with base == frame; a gain, weight or
  // feedback cap that is non-finite (weights must be > 0, caps >= 0); a
  // missing or mis-sized posture vector; a non-finite feed-forward
  // (twist_ff, the axis target's, qd_posture_ff) or axis-task position; an
  // approach axis that is not unit; dt not > 0.
  // A non-finite kSe3 placement_des is NOT refused there: like the SE3
  // overload it reaches the solve and takes the failure branch (q_ref =
  // cache.q, v_ref = 0, re-anchor), which is the safe output.
  [[nodiscard]] bool Compute(const PinocchioCache& cache, const MultiFrameInput& in) noexcept;

  /// Axis-alignment branch of the last position + axis Compute().
  [[nodiscard]] rtc::math::se3::AxisAlignRegion LastAxisRegion() const noexcept {
    return axis_region_;
  }

  /// ‖e_p‖ [m] and θ = ‖e_a‖ [rad] of the last position + axis Compute().
  [[nodiscard]] double PositionErrorNorm() const noexcept { return position_error_norm_; }

  [[nodiscard]] double AxisErrorAngle() const noexcept { return axis_error_angle_; }

  [[nodiscard]] const Eigen::VectorXd& QRef() const noexcept { return q_ref_; }

  [[nodiscard]] const Eigen::VectorXd& VRef() const noexcept { return v_ref_; }

  // √det(J_a·J_aᵀ + μ²·I) from the LDLT diagonal pivots of the last
  // Compute() — damped manipulability measure (≥ μ⁶ > 0 by construction)
  // for logging / A-B shadow diagnostics (Stage C-2 DeviceWbcLog).
  [[nodiscard]] double Manipulability() const noexcept { return manipulability_; }

  // ‖e_x‖ of the last Compute() (6D mixed: position [m] + rotation [rad]).
  [[nodiscard]] double TcpErrorNorm() const noexcept { return tcp_error_norm_; }

  /// Solver outcome of the last Compute() (dynamic_catching S2.2b, L5 §5.1).
  /// Diagnostic only — it never changes the outputs. Filled on every call;
  /// a call rejected by its preconditions leaves reached_solve = false.
  struct SolveDiagnostics {
    bool reached_solve{false};     ///< preconditions held and the QP was solved
    bool converged{false};         ///< QP converged with finite iterates
    bool non_finite{false};        ///< QP iterates or q_ref / v_ref were non-finite
    int status{-1};                ///< proxsuite::proxqp::QPSolverOutput (0 = SOLVED), −1 = none
    int iterations{0};             ///< ProxQP outer iterations
    double solve_time_us{0.0};     ///< wall time of the solve [µs]
    bool command_mismatch{false};  ///< evaluate_at_command: cache.q (arm) ≠ previous QRef()
    bool bound_conflict{false};    ///< acceleration box overrode velocity ∩ position
    std::uint64_t conflict_mask{0};  ///< bit i = velocity index i conflicted
    /// kKinematic / kDynamic: rows assembled this call, how many the solution
    /// sits on (within tolerance), and whether it broke one — then the call
    /// failed (the rows could not hold with the velocity ∩ position box) and
    /// `converged` is false although `status` may read SOLVED.
    int accel_rows{0};
    int accel_rows_binding{0};
    bool accel_rows_violated{false};
    /// The multi-frame Compute() only — the single-task overloads leave these
    /// at their defaults.
    int tasks{0};                ///< tasks of the call (0 when it was refused)
    bool rejected_input{false};  ///< refused before the solve by an input check
    /// Feedback cap active: bit 2k = task k's linear block, bit 2k + 1 its
    /// angular / approach-axis block.
    std::uint32_t fb_saturated{0};
    /// Bit k = task k's rotation (kSe3) or approach-axis (kPositionAxis)
    /// error is within 0.05 rad of π, where its direction is discontinuous.
    std::uint32_t rot_near_pi{0};
  };

  [[nodiscard]] const SolveDiagnostics& LastSolve() const noexcept { return last_solve_; }

  /// Forget the integration anchor and the previous velocity: the next
  /// Compute() re-anchors q_ref to cache.q and the acceleration box starts
  /// from v_prev = 0. Call at activation, re-arm and E-STOP release (L5 §4.2)
  /// — leaving v_prev from the previous run would make the first tick's
  /// acceleration box relative to a stale velocity. RT-safe.
  void ResetAnchor() noexcept {
    anchor_initialized_ = false;
    command_check_armed_ = false;
    v_prev_.setZero();
    if (n_accel_rows_max_ > 0) {
      qp_solver_.ResetWarmStart();  // a new trial's rows start cold (kBox: legacy)
    }
  }

 private:
  // Shared stages of Compute(). Each keeps the floating-point accumulation
  // order of the original single-function Compute(), which the golden-vector
  // regression (test/test_clik_golden.cpp) pins bit-for-bit.
  [[nodiscard]] bool PreconditionsHold(const PinocchioCache& cache, int tcp_frame_idx,
                                       int base_frame_idx, const Eigen::VectorXd& q_posture_des,
                                       double dt) const noexcept;
  // evaluate_at_command: cache.q on the arm equals the previous QRef().
  [[nodiscard]] bool CommandStateMatches(const PinocchioCache& cache) const noexcept;
  void ComputePostureReferences(const PinocchioCache& cache, const Eigen::VectorXd& q_posture_des,
                                const Eigen::VectorXd* qd_posture_ff) noexcept;
  // H/g must already hold the task terms.
  void AddPostureAndDamping() noexcept;
  void AssembleBox(const Eigen::VectorXd& q, double dt) noexcept;
  // kKinematic / kDynamic rows below the box (no-op for kBox). `axis_rows`
  // selects the position + axis overload's task rows (j_pos_ / j_axis_) over
  // the SE3 overload's (j_task_). Returns false on a non-finite row.
  [[nodiscard]] bool AssembleAccelRows(const PinocchioCache& cache,
                                       const PinocchioCache::RegisteredFrame& rf, double dt,
                                       bool axis_rows) noexcept;
  // The same for the multi-frame call: kKinematic stacks each task's rows.
  [[nodiscard]] bool AssembleAccelRows(const PinocchioCache& cache,
                                       std::span<const FrameTask> tasks, double dt) noexcept;
  // Pieces the two AssembleAccelRows share. The kKinematic writers put one
  // task's rows at `row0` of the accel block and return how many they wrote.
  [[nodiscard]] bool WriteDynamicRows(const PinocchioCache& cache, double inv_dt) noexcept;
  int WriteSe3AccelRows(int row0, const Eigen::MatrixXd& j6,
                        const PinocchioCache::RegisteredFrame& rf, const Eigen::VectorXd& v_c,
                        double inv_dt) noexcept;
  int WriteAxisAccelRows(int row0, const Eigen::MatrixXd& j_pos, const Eigen::MatrixXd& j_axis,
                         const PinocchioCache::RegisteredFrame& rf, const Eigen::VectorXd& v_c,
                         double inv_dt) noexcept;
  void WriteTaskAccelBounds(int row0, int rows, const Eigen::Matrix<double, 6, 1>& drift,
                            const Eigen::VectorXd& v_c) noexcept;
  // Leaves rows [rows, n_accel_rows_max_) inert, scales the assembled ones to
  // unit norm and reports whether they are finite.
  [[nodiscard]] bool FinishAccelRows(int rows) noexcept;
  // Input checks of the multi-frame call that need no kinematics.
  [[nodiscard]] bool MultiFrameInputHolds(const PinocchioCache& cache,
                                          const MultiFrameInput& in) const noexcept;
  // Error, reference and Jacobian rows of task k into task_rows_[k]. Returns
  // false on an approach axis that is not unit.
  [[nodiscard]] bool BuildTaskRows(const PinocchioCache& cache, const FrameTask& task,
                                   int k) noexcept;
  // Post-solve: every accel row holds at v (within tolerance); counts binding.
  [[nodiscard]] bool AccelRowsHold(const Eigen::VectorXd& v) noexcept;
  // The failure outputs: q_ref = cache.q, v_ref = 0, re-anchor, v_prev = 0,
  // and (with accel rows) a cold solver start. Returns false for the caller.
  bool Fail(const PinocchioCache& cache) noexcept;
  [[nodiscard]] bool SolveAndIntegrate(const PinocchioCache& cache, double dt,
                                       bool reseed_anchor) noexcept;

  int nv_{0};
  int n_arm_{0};
  std::vector<int> arm_v_idx_;
  std::vector<int> task_v_idx_;  // columns of the multi-frame task rows
  int max_frame_tasks_{1};
  bool relative_tasks_{false};
  double damping_sq_{1e-4};
  double v_limit_{1.5};
  Eigen::VectorXd v_limit_per_joint_;  // [nv] or empty (scalar v_limit_)
  Eigen::VectorXd a_max_;              // [nv] or empty (acceleration box off)
  Eigen::VectorXd v_prev_;             // [nv] v_ref of the last successful Compute()
  double w_smooth_{0.0};               // smoothing weight, 0 → off
  AccelConstraint accel_constraint_{AccelConstraint::kBox};
  double task_a_lin_{0.0};  // kKinematic row bounds
  double task_a_ang_{0.0};
  Eigen::VectorXd tau_bound_;        // [nv] eta_tau·tau_max (kDynamic)
  int n_accel_rows_max_{0};          // extra QP rows allocated below the box
  int n_accel_rows_{0};              // rows assembled by the current call
  bool evaluate_at_command_{false};  // cache.q is the command state q_c
  double w_axis_{0.5};               // approach-axis rows weight
  double k_axis_{0.0};               // approach-axis gain K_a
  rtc::math::se3::AxisAlignRegion axis_region_{rtc::math::se3::AxisAlignRegion::kInvalidInput};
  double position_error_norm_{0.0};
  double axis_error_angle_{0.0};
  Eigen::MatrixXd j_pos_;   // [3 × nv] position rows, arm columns
  Eigen::MatrixXd j_axis_;  // [2 × nv] approach-axis rows, arm columns
  double w_task_{1.0};
  Eigen::VectorXd q_min_;         // [nv] or empty (position box disabled)
  Eigen::VectorXd q_max_;         // [nv] or empty
  double anchor_drift_max_{0.0};  // carry-forward anti-windup clamp [rad], ≤0 → off
  // False until the first successful Compute(); forces a measured re-anchor on
  // the first call (and after any failure) so q_ref never integrates from a
  // stale/zero anchor.
  bool anchor_initialized_{false};
  // evaluate_at_command: the arm command-state check is active (see
  // CommandStateMatches). Survives failed calls; cleared by Init / ResetAnchor.
  bool command_check_armed_{false};

  // Task gain of the single-task overloads (L1).
  Eigen::Matrix<double, 6, 1> kx_{Eigen::Matrix<double, 6, 1>::Zero()};

  // Posture groups (L2 / L3: the arm and the hand unless configured).
  struct PostureGroupState {
    std::vector<int> v_idx;
    double weight{0.0};
    double gain{0.0};
    Eigen::VectorXd v_post;  // [v_idx.size()] posture velocity reference
  };

  std::vector<PostureGroupState> posture_;

  // Last-Compute diagnostics
  SolveDiagnostics last_solve_;
  double manipulability_{0.0};
  double tcp_error_norm_{0.0};

  // Outputs
  Eigen::VectorXd q_ref_;  // [nv] (nq == nv contract)
  Eigen::VectorXd v_ref_;  // [nv]

  // QP (decision variable v ∈ Rᴺ, N == nv; n_eq = 0, n_ineq = N box).
  QPSolverWrapper qp_solver_;
  QPData qp_data_;

  // Pre-allocated workspace
  Eigen::MatrixXd j_task_;              // [6 × nv] arm columns of rf.J (hand cols 0)
  Eigen::Matrix<double, 6, 1> e_x_;     // SE(3) error
  Eigen::Matrix<double, 6, 1> r_task_;  // Kx ⊙ e_x (L1 task-velocity reference)
  // Manipulability-only 6×6 (diag continuity; J♯ no longer used for the solve).
  Eigen::MatrixXd j_arm_;           // [6 × n_arm] gathered arm columns
  Eigen::Matrix<double, 6, 6> m6_;  // J_a·J_aᵀ + μ²·I
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt6_;

  // Per-task workspace of the multi-frame call. The matrices keep the shapes
  // and types the single-task overloads use (6 × nv / 3 × nv / 2 × nv,
  // dynamic), so one task accumulates into H and g through the very same
  // Eigen product path and the result is bit-identical to theirs.
  struct TaskRows {
    Eigen::MatrixXd j6;             // [6 × nv] kSe3 rows
    Eigen::MatrixXd j_pos;          // [3 × nv] kPositionAxis position rows
    Eigen::MatrixXd j_axis;         // [2 × nv] kPositionAxis axis rows
    Eigen::Matrix<double, 6, 1> e;  // kSe3 pose error
    Eigen::Matrix<double, 6, 1> r;  // kSe3 reference
    Eigen::Vector3d r_pos;          // kPositionAxis position reference
    Eigen::Vector3d w_ref_local;    // kPositionAxis axis reference (head<2> used)
  };

  std::vector<TaskRows> task_rows_;  // [max_frame_tasks]
};

}  // namespace rtc::tsid
