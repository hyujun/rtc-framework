// ── Catch-pose IK + catchability judgement (dynamic_catching S1.9) ──────────
// One call answers "can the arm meet a ball at p_c travelling along v̂, and how
// well-conditioned is the pose it would have to hold": a 5-row IK onto the
// catch frame, followed by the manipulability gate L3 §4.2 [확정 D-18] defines.
//
// The 5 rows are 3 translation (LOCAL_WORLD_ALIGNED) + 2 approach-axis
// (LOCAL x, y). Roll about the palm normal is deliberately NOT constrained —
// a ball does not care how the hand is spun about its own approach axis — so a
// 6-DoF arm keeps exactly one free degree of freedom at the solution.
//
// ── What this function is for, and what it is NOT ───────────────────────────
// It is the SINGLE function both the offline catchability map (S3.5a/b) and the
// runtime planner (S6.2) call. That is a hard requirement, not a convenience:
// the map's verdict is only meaningful if runtime reproduces it, so same seed +
// same inputs + same options must give a bit-identical q*. Everything here that
// looks over-specified — no warm start between calls, no internal state
// carried across Solve(), a deterministic finite-difference gradient — is in
// service of that (plan §11).
//
// It is NOT a controller. It runs on the planner thread, which may be
// SCHED_FIFO, so it obeys the RT path rules (RT-1 no heap, RT-2 no throw, RT-3
// no logging) after Resize() — but it is not a tick and has no notion of dt.
//
// ── Six places where the obvious reading is the wrong one ───────────────────
//
// 1. ρ WEIGHTS THE TASK, it does not scale the residual alone.
//    L3 §4.2 prints the update as Δq = Jᵀ(JJᵀ+λ²I)⁻¹·[−(p_C−p_c); ρ·S·e_a^C],
//    with ρ [m/rad] introduced as "the characteristic length that matches the
//    two blocks' units under a single λ". Applied to the residual only, that is
//    dimensionally inconsistent: the rotation rows of J·Δq are in rad while the
//    rotation rows of the residual would be in m, and ρ would act as a step
//    GAIN of 0.1 rather than a unit conversion — which also contradicts the
//    same section's statement that ω^L = e_a^C applied for unit time aligns the
//    axis EXACTLY in one step. Both properties hold, and ρ has its stated
//    units, only if ρ weights the task on both sides:
//
//        W = diag(1, 1, 1, ρ, ρ),   Δq = (W J₅)⁺ (W e),
//        e = [p_c − p_C ; S e_a^C]
//
//    Away from a singularity (λ² = 0) W cancels and the step is the exact
//    one-step alignment §4.2 describes; near one it is what decides how the
//    single λ splits damping between the metre block and the radian block,
//    which is the role §4.2 assigns it. This is a correction of the printed
//    formula, recorded in L3 §4.2.
//
// 2. THE MANIPULABILITY MEASURE IS TAKEN ON THE UNWEIGHTED J₅.
//    w₅ = √det(J₅J₅ᵀ) with J₅ exactly as §4.2 defines it — no W. The gate
//    threshold (`planner.catchability.manipulability_min.arm_5row` = 0.1,
//    provisional) was set against that mixed-unit definition, so weighting J
//    here would silently move the gate by ρ² while every threshold in YAML
//    stayed put. The weight belongs to the step, the measure belongs to the
//    gate, and they are not the same object.
//
// 3. THE TASK STEP IS A CONSTRAINED QP, NOT A DAMPED PSEUDO-INVERSE.
//    L3 §4.2 [확정 D-7d] said to reuse `DifferentialIk` for the whole update.
//    That is reversed (D-26): q̇_clik comes from
//
//        q̇_clik = argmin ½‖W(J₅q̇ − e)‖² + ½μ‖q̇‖²
//                  s.t. max(q_min−q, −dq_step_max) ≤ q̇ ≤ min(q_max−q, +dq_step_max)
//
//    so the joint limits and the step bound are CONSTRAINTS on the task step
//    rather than a clamp applied to it afterwards. `DifferentialIk` stays, but
//    only to form N — the projector belongs to the law, not to the solver.
//
//    The measurement behind the reversal (2 arms × 500 candidates, plan §4.4):
//    at μ = 1e-4 the QP accepts slightly more candidates than the DLS step
//    (99.2% vs 98.8% on the 6R, 98.8% vs 98.2% on the 7R) at the same w₅ and a
//    comparable residual, for ~22% more time per call. μ is not free: at
//    μ = 1e-8 the Hessian is near-singular, ProxQP fails to converge on most
//    iterations, and the whole thing falls back — which is why `mu` is
//    validated and its provisional default is recorded rather than guessed.
//
//    Two things the QP does NOT buy, stated because the plan expected them:
//    the limits still need a post-hoc clamp (note 4), and determinism needs an
//    explicit cold start (note 5).
//
// 4. THE CONSTRAINT BOUNDS q̇_clik, BUT q̇_d IS WHAT MOVES.
//    q̇_d = q̇_clik + q̇_n, and q̇_n is computed outside the QP, so the sum can
//    leave the box the QP respected. The ‖q̇_d‖∞ scaling and the clamp into the
//    joint limits therefore still run, exactly as they did for the DLS step —
//    "limits as constraints" is an improvement in how the task step is chosen,
//    not a replacement for bounding what is finally applied. Measured: the
//    fraction of accepted poses sitting on a bound is 1.0% (QP) vs 1.2% (DLS)
//    on the 6R and 4.7% vs 5.5% on the 7R — real, but small.
//
// 5. EVERY Solve() COLD-STARTS THE QP.
//    `QPSolverWrapper` warm-starts from the previous solve, which is right for
//    a controller and wrong here: consecutive Solve() calls are DIFFERENT
//    candidates, so a retained warm start would make each answer depend on
//    which candidate was tried first and the offline map would stop agreeing
//    with the runtime planner (plan §11). `ResetWarmStart()` (added to rtc_tsid
//    for this) is called once per Solve(); iterations WITHIN one Solve() do
//    warm-start from each other, which is deterministic and wanted.
//
// 6. A SINGULAR POSE DOES NOT SHOW UP AS `DifferentialIk::Compute → ok=false`.
//    J₅J₅ᵀ is positive semi-definite for any real J₅ and Eigen's LLT accepts a
//    singular one, so a rank-deficient pose returns ok = true with σ_min ≈ 0
//    (differential_ik.hpp #310). `ok = false` means one thing only: a non-finite
//    J. Rank deficiency is therefore caught where it is actually visible — in
//    the w computation, by a non-positive or non-finite LDLT pivot — and gets
//    its own reason code. Reading `ok` as "the pose is fine" would pass every
//    singular pose straight into the gate.
//
// ── Maximising w₅ in the null space ─────────────────────────────────────────
// [확정 D-18] said roll should be left to the seed and that choosing it by
// manipulability was out of v1 scope. That is reversed (user decision
// 2026-09-20, plan §1 결정 로그): the step carries a null-space ascent term on
// log w₅, so the free roll is spent on conditioning instead of on whatever the
// seed happened to leave.
//
//     q̇_sec  = k_manip·∇log w₅ + k_null·(q_n − q)     (the secondary task)
//     q̇_n    = N·q̇_sec                                (a velocity IN the null space)
//     q̇_d    = q̇_clik + q̇_n                           (what one iteration applies)
//
// The law is a sum of JOINT VELOCITIES, not one opaque joint step: CLIK's own
// output is q̇_clik, and the secondary task contributes q̇_n, which the projector
// has already made invisible to the primary task. Writing it as a single Δq
// hides exactly the priority N exists to enforce. One iteration integrates q̇_d
// over Δt = 1 — this is an offline root-finding iteration, not a servo tick, so
// `dq_step_max` bounds ‖q̇_d‖∞ per iteration and the unit step is the whole
// integrator.
//
// log w₅ rather than w₅: the gradient then grows as the pose approaches a
// singularity (∇log w = ∇w / w), which is where the push away from it is
// wanted, and the gain k_manip does not have to be retuned for the mixed-unit
// scale of w₅ itself.
//
// The ascent is LOCAL. It climbs to the nearest maximum on the IK branch the
// seed lands on; it does not search roll globally, and two seeds on different
// branches can legitimately return different q*. That is the price of the
// determinism requirement above, and it is why the seed is an input rather than
// something this function picks.
//
// k_manip = 0 recovers the original [확정 D-18] behaviour exactly, which is
// what the k_null-only baseline in the test suite measures against.
//
// q_n is the seed. L3 §4.2 names K_n(q_n − q) without ever defining q_n; with
// the seed being the wait pose (the one posture the arm is known to rest in),
// "stay near where you started" is the only posture reference this function has
// that does not invent a new YAML key. k_null defaults to 0, so the term is
// inert unless a caller asks for it.
#pragma once

#include "rtc_base/types/types.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cstdint>

namespace rtc::catching {

/// Which Jacobian the catchability gate is applied to
/// (`planner.catchability.definition`).
///
/// This selects the GATE only. The null-space ascent always climbs w₅,
/// whichever definition is active, so q* does not depend on this field — which
/// is what makes w₅ and w₆ at the returned q* two measurements of the same pose
/// and therefore comparable (plan §11 C-3). Under kArm6Row the gate is applied
/// to a quantity that was not itself maximised; that is intended, and is the
/// point of keeping both numbers.
enum class ManipDefinition : std::uint8_t {
  kArm5Row = 0,  ///< w₅ = √det(J₅J₅ᵀ), roll excluded — the default
  kArm6Row,      ///< w₆ = √det(J₆J₆ᵀ), roll included
};

/// Why a candidate was rejected. kNone is the only accepting value.
///
/// Ordered as the checks run, so a caller histogramming rejections reads the
/// gate order off the enum. Every one of these is a REJECTION: nothing here is
/// clamped into a plausible value and allowed through (NUM-7).
enum class CatchPoseReason : std::uint8_t {
  kNone = 0,           ///< accepted: position, axis and manipulability all met
  kOptionsInvalid,     ///< an option is non-finite or out of range
  kModelInvalid,       ///< nq ≠ nv, nv > kMaxRobotDOF, or the frame is unknown
  kSeedNonFinite,      ///< seed has a NaN/Inf, or its size ≠ nv
  kTargetNonFinite,    ///< p_c has a NaN/Inf
  kVelocityNonFinite,  ///< v̂ has a NaN/Inf
  kSpeedTooLow,        ///< ‖v̂‖ < v_eps — the approach axis is undefined
  kAxisAlignInvalid,   ///< AxisAlignError rejected its inputs (non-unit a^C)
  kJacobianNonFinite,  ///< FK/Jacobian produced a non-finite J (DifferentialIk ok=false)
  kQpFailed,           ///< the task-step QP did not converge (status/iterations reported)
  kNotConverged,       ///< max_iter reached without meeting eps_pos ∧ alpha_max
  kRankDeficient,      ///< J J ᵀ would not factor: a non-finite or non-positive pivot
  kBelowManipMin,      ///< w(definition) < manipulability_min
};

/// Tuning for one Solve(). Defaults mirror L3 §6 where a key exists there; the
/// rest are provisional and marked as such in the docs until S3.5a measures
/// them. No YAML parser yet (Q4): the planner binding that gets one later has
/// to produce exactly this struct.
struct CatchPoseIkOptions {
  // ── Acceptance (L3 §4.2) ──────────────────────────────────────────────────
  int max_iter{20};        ///< `planner.ik.max_iter`
  double eps_pos{0.002};   ///< `planner.ik.eps_pos` [m]
  double alpha_max{0.26};  ///< `planner.ik.alpha_max` [rad] — TBD in L3, provisional ≈15°

  // ── Step (L3 §4.2) ────────────────────────────────────────────────────────
  double rho{0.1};              ///< `planner.ik.rho` [m/rad] — task weight, see note 1 above
  double sigma0{1e-3};          ///< `planner.ik.sigma0` — §6.5 damping shell entry
  double lambda_max{1e-2};      ///< `planner.ik.lambda_max` — §6.5 max damping
  double dq_step_max{0.15};     ///< `planner.ik.dq_step_max` [rad] — bound on ‖q̇_d‖∞
  double mu{1e-4};              ///< `planner.ik.mu` — task-QP regularisation, see note 4
  double qp_eps_abs{1e-10};     ///< `planner.ik.qp_eps_abs` — QP absolute tolerance
  int qp_max_iter{50};          ///< `planner.ik.qp_max_iter` — QP iteration cap
  double k_null{0.0};           ///< `planner.ik.k_null` — posture pull toward the seed
  double k_manip{0.0};          ///< `planner.ik.k_manip` — null-space ascent gain on log w₅
  double manip_grad_tol{1e-4};  ///< `planner.ik.manip_grad_tol` — ascent stop on ‖N∇log w₅‖
  double fd_step{1e-5};         ///< central-difference step h [rad] for ∇log w₅

  // ── Input defence (NUM-7, L3 §4.2) ────────────────────────────────────────
  double v_eps{1e-6};  ///< `planner.ik.v_eps` [m/s] — floor on ‖v̂‖ before normalising

  // ── Gate (L3 §4.2 [확정 D-18]) ────────────────────────────────────────────
  ManipDefinition definition{ManipDefinition::kArm5Row};
  double manipulability_min{0.1};  ///< threshold for `definition`; mixed-unit, see note 2
};

/// Outcome of one Solve(). Fixed capacity, no heap, safe to copy on the RT path.
struct CatchPoseIkResult {
  bool accepted{false};
  CatchPoseReason reason{CatchPoseReason::kOptionsInvalid};

  /// q* — the joint vector of the LAST iterate that met both acceptance
  /// tolerances. Valid for `nv` entries whenever the IK itself converged, which
  /// is kNone, kBelowManipMin and kRankDeficient: in all three the pose exists
  /// and only the gate's verdict on it differs. Untouched for every other
  /// reason, because a q that never satisfied the task is not a pose.
  double q[rtc::kMaxRobotDOF]{};
  int nv{0};

  int iterations{0};      ///< iterations actually run
  double pos_error{0.0};  ///< ‖p_c − p_C‖ at q* [m]
  double theta{0.0};      ///< ‖e_a^C‖ at q* [rad] — the approach-axis cone angle
  double w5{0.0};         ///< √det(J₅J₅ᵀ) at q*, UNWEIGHTED (note 2)
  double w6{0.0};         ///< √det(J₆J₆ᵀ) at q* — recorded for C-3 even when unused
  bool w5_valid{false};   ///< the w₅ factorisation stayed finite and positive
  bool w6_valid{false};   ///< likewise for w₆

  /// False when the ascent was still improving w₅ at the iteration limit. NOT a
  /// rejection: a pose that meets the task is acceptable whether or not its
  /// conditioning had stopped climbing. It is the signal that max_iter is too
  /// small, which is exactly what G3-G is meant to measure.
  bool manip_converged{false};
  double manip_grad_norm{0.0};  ///< ‖N ∇log w₅‖ at the final iterate

  double sigma_min{0.0};  ///< σ_min(W J₅) at q* — §6.5 diagnostic for the projector N
  double lambda_sq{0.0};  ///< λ² applied when forming N at the final step
  int qp_status{-1};      ///< last task-QP solver status (0 = solved)
  int qp_iterations{0};   ///< last task-QP iteration count
  int qp_failures{0};     ///< 1 when the run ended on a failed QP (it fails closed)
};

namespace detail {

/// ½·log det(J Jᵀ) and whether the factorisation was usable.
///
/// Logs rather than the determinant itself: the ascent differentiates log w, and
/// a product of five or six pivots underflows to zero for a w that is small but
/// perfectly legitimate. `valid == false` is the rank-deficiency signal — see
/// note 3 in the file header for why it cannot come from DifferentialIk.
struct LogManip {
  double log_w{0.0};
  bool valid{false};
};

}  // namespace detail

/// Catch-pose IK and catchability gate for one candidate.
///
/// Pre-allocated once with Resize(); every Solve() after that is heap-free,
/// noexcept and silent. Holds no state between calls by design (determinism,
/// plan §11) — the buffers are scratch, never warm-start data.
class CatchPoseIk {
 public:
  /// Size the buffers for an nv-DoF arm model. Off-RT (lifecycle / map setup):
  /// this allocates and nothing else may run before it.
  ///
  /// @param nv joint count of the model Solve() will be given, ≤ kMaxRobotDOF.
  void Resize(int nv);

  /// Solve for the catch pose and judge it.
  ///
  /// @param model       arm sub-model handle (Q1: `GetReducedModel("arm")`, catch
  ///                    frame included). Mutated — its Data holds this call's FK.
  ///                    The handle is the caller's; it must not be shared with
  ///                    another thread for the duration.
  /// @param catch_frame frame id of the catch frame in `model`, local +z being
  ///                    the palm outward normal [확정 D-17].
  /// @param p_c         target position in MODEL WORLD coordinates [m] (Q2: any
  ///                    base→world transform is the caller's job).
  /// @param v_ball      ball velocity at the candidate instant, model world [m/s].
  ///                    The approach axis is a_d = −v_ball/‖v_ball‖; this takes
  ///                    the raw velocity so the ‖v̂‖ ≥ v_eps defence happens here
  ///                    rather than in each caller (NUM-7).
  /// @param q_seed      seed, nv entries — the wait pose [확정 D-18]. Also the
  ///                    posture reference q_n for the k_null term.
  /// @param opt         tuning; rejected as kOptionsInvalid if unusable.
  /// @return outcome; `accepted` iff `reason == kNone`.
  [[nodiscard]] CatchPoseIkResult Solve(rtc_urdf_bridge::RtModelHandle& model,
                                        pinocchio::FrameIndex catch_frame,
                                        const Eigen::Vector3d& p_c, const Eigen::Vector3d& v_ball,
                                        const Eigen::Ref<const Eigen::VectorXd>& q_seed,
                                        const CatchPoseIkOptions& opt) noexcept;

  [[nodiscard]] int nv() const noexcept { return nv_; }

 private:
  /// q̇_clik from the box-constrained task QP, written to qdot_clik_.
  /// Returns false if the solve did not converge or produced a non-finite
  /// step; `qp_status_` / `qp_iterations_` then carry the solver's own verdict.
  [[nodiscard]] bool QpTaskStep(const pinocchio::Model& pin,
                                const CatchPoseIkOptions& opt) noexcept;

  /// Clamp `q` into the model's own position limits, joint by joint.
  ///
  /// Static and taking the model because the limits are the MODEL's, never a
  /// constant in this file (ARCH-1) — the same function serves a 6-DoF and a
  /// 7-DoF arm with no branch on which robot it is.
  static void ClampToLimits(const pinocchio::Model& pin, Eigen::VectorXd& q) noexcept;

  /// Fill J6_ (6×nv) with [J^LWA_p ; J^L_ω] at `q`, and report finiteness.
  /// Both blocks come from the SAME q — two GetFrameJacobian calls against one
  /// ComputeJacobians, since the two reference frames differ only in how the
  /// rows are expressed.
  [[nodiscard]] bool StackJacobian(rtc_urdf_bridge::RtModelHandle& model,
                                   pinocchio::FrameIndex catch_frame,
                                   const Eigen::Ref<const Eigen::VectorXd>& q) noexcept;

  /// ½·log det(J₅J₅ᵀ) from the top 5 rows of J6_, fixed-size LDLT.
  [[nodiscard]] detail::LogManip LogW5() noexcept;

  /// ½·log det(J₆J₆ᵀ) from all 6 rows of J6_, fixed-size LDLT.
  [[nodiscard]] detail::LogManip LogW6() noexcept;

  /// Central-difference ∇log w₅ at `q`, written to grad_. 2·nv Jacobian
  /// evaluations, allocation-free. Returns false if any probe was invalid, in
  /// which case the caller drops the ascent term for this iteration rather than
  /// clamping it (NUM-7) — a dropped term is recorded in manip_grad_norm as 0.
  ///
  /// Leaves the model's Data holding the LAST PROBE's configuration, not q; the
  /// caller re-evaluates at q afterwards.
  [[nodiscard]] bool ManipGradient(rtc_urdf_bridge::RtModelHandle& model,
                                   pinocchio::FrameIndex catch_frame,
                                   const Eigen::Ref<const Eigen::VectorXd>& q, double h) noexcept;

  int nv_{0};

  // Kept for the NULL-SPACE PROJECTOR only. N = I − J⁺J is part of the update
  // law, not of the task-step solver, so it survives the switch to the QP —
  // `sigma0` / `lambda_max` now parameterise N and nothing else.
  compliance::DifferentialIk dls_;

  tsid::QPSolverWrapper qp_;
  tsid::QPData qp_data_;
  Eigen::MatrixXd JtJ_;  ///< nv×nv: J₅wᵀJ₅w, the task QP's Hessian before μ
  int qp_status_{-1};    ///< last task-QP status, surfaced in the result
  int qp_iterations_{0};

  Eigen::MatrixXd J6_;         ///< 6×nv: rows 0-2 LWA linear, rows 3-5 LOCAL angular
  Eigen::MatrixXd Jlocal_;     ///< 6×nv: the LOCAL extraction J6_ takes its rows 3-5 from
  Eigen::MatrixXd J5w_;        ///< 5×nv: the weighted task Jacobian W·J₅ handed to DLS
  Eigen::VectorXd e5w_;        ///< 5: the weighted residual W·e
  Eigen::VectorXd q_;          ///< nv: current iterate
  Eigen::VectorXd q_try_;      ///< nv: FD probe / trial configuration
  Eigen::VectorXd q_acc_;      ///< nv: last iterate that met both tolerances
  Eigen::VectorXd grad_;       ///< nv: ∇log w₅
  Eigen::VectorXd qdot_sec_;   ///< nv: secondary task k_manip·∇log w₅ + k_null·(q_n − q)
  Eigen::VectorXd qdot_clik_;  ///< nv: q̇_clik = (W J₅)⁺_λ (W e)
  Eigen::VectorXd qdot_n_;     ///< nv: q̇_n = N·q̇_sec — a velocity in the null space
  Eigen::VectorXd qdot_d_;     ///< nv: q̇_d = q̇_clik + q̇_n

  Eigen::Matrix<double, 5, 5> A5_{Eigen::Matrix<double, 5, 5>::Zero()};
  Eigen::Matrix<double, 6, 6> A6_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::LDLT<Eigen::Matrix<double, 5, 5>> ldlt5_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt6_;
};

}  // namespace rtc::catching
