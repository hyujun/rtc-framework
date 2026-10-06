// ── mpc_docking numeric core: approach, dock, stop — an SQP over ProxQP ────────
// (dynamic_catching E1-F13, #739; reference
//  docs/dynamic_catching/ref/ball_catching_inverse_dynamics_mpc.md §10, §17)
//
// The numeric core of the third segment planner and of the NLP catch search:
// one arm trajectory from "now" through the catch instant t_c to rest, with the
// ball's capture written in the hand's capture frame. It knows no ROS, no
// controller and no robot; nothing calls it yet but its tests (the planner that
// wraps it is E1-F16, the search that calls it per candidate E1-F14).
//
// ── Problem ───────────────────────────────────────────────────────────────────
// Decision variable: the block-constant joint jerk ũ (move blocking, shared by
// all joints), as in MpcSegmentCore — the states x_k = (q_k, q̇_k, q̈_k) are
// eliminated, x_k = Φ_k x_0 + Γ_k E ũ, with the same stage gains. The grid is
// anchored at the catch instant: n_pre intervals of Δ_a before the catch node
// k_c = n_pre, n_stop intervals of Δ_s after it, N = n_pre + n_stop; node N is
// at rest. With r^H, ν^H, s, ρ, c the ball in the capture frame
// (mpc_docking_relative_state.hpp), ℓ = s − s_ent, and A the approach nodes:
//
//   min   Δ_a Σ_{k<k_c} [ ‖τ_k‖²_Rτ + ‖q̈_k‖²_Ra + ‖u_k/u_s‖²_Rj
//                          + ‖q_k − q_nom‖²_Qq + w_m ψ_m(q_k) ]          motion
//       + Δ_a Σ_{k<k_c} ρ_T(t_k) [ ‖r^H_k − r_ref,k‖²_Qp + ‖ν^H_k − ν_ref‖²_Qv ]   near
//       + ‖ρ_kc − ρ_ref‖²_Qρf + ‖ν^H_kc − ν_ref‖²_Qνf + w_E E_n/E_ref   terminal
//       + Δ_a Σ_{k∈A} (λ_1ᵀ s_k + ‖s_k‖²_Λ2)                             slack
//       + Δ_s Σ_{k≥k_c} [ ‖u_k/u_s‖²_Rj,stop + w_⊥ ‖P_⊥(p_C(q_k) − p_line)‖² ]   stop
//   s.t.  q̇_N = q̈_N = 0
//         q_min ≤ q_k ≤ q_max,  |q̇_k| ≤ q̇_max,  [|q̈_k| ≤ q̈_max],  [|u_k| ≤ j_max]   k = 1..N
//         τ_lo ≤ RNEA(q_k, q̇_k, q̈_k) ≤ τ_hi                              k = 1..N
//         ℓ_k ≥ 0,  ‖ρ_k‖² ≤ (r_ent + ℓ_k⁺ tanθ + s_c,k)²,
//         c_k² ≤ c_ent,max² + 2 a_brake ℓ_k + s_v,k,  s ≥ 0               k ∈ A
//         ℓ_kc = 0                                                        entrance plane
//         a_iᵀρ + κ_i √(a_iᵀ Σ_ρ a_i + ε²) ≤ b_i                          lateral
//         c √(σ_max² − σ_τ²) ≥ σ_s                                        timing
//         c_min ≤ c ∓ κ_ν σ_c ≤ c_cap,max,  u_jᵀE⊥ᵀν + κ_ν σ_j ≤ v_⊥ cos(π/m)   V_cap
//         [g_n ≤ 0,  E_n ≤ E_max,  P_n ≤ P_max]                            impact
//
// ρ_T(t) = exp(−(t − t_c)²/2σ_T²), r_ref,k = r_ref + (t_c − t_k)(−ν_ref),
// r_ref = (ρ_ref, s_ent). [·] rows exist only when enabled. The cost is the
// reference's — no ½, running terms times the interval length — so J⋆ compares
// across candidates as the reference intends; the stop segment's terms are
// reported separately from the reference's.
//
// What is constant is not in the QP but IS in J⋆: node 0 is x_0, so its
// torque, acceleration, posture, ψ_m and near terms are constants of the
// candidate. The QP carries nodes 1..k_c−1 of the running sums (u_0 is a
// variable, so the jerk term covers stage 0 too) and J⋆ adds node 0. With a
// single pre-catch interval (k_c = 1) the QP therefore has no running terms
// and no approach rows: the problem is "dock, then stop".
//
// ── Method ────────────────────────────────────────────────────────────────────
// SQP with a Gauss–Newton Hessian (positive definite: every R_j > 0). The
// iterate is a point of the decision space, z = ũ/u_s; each iteration
// linearises the nonlinear rows at x̄ = x(z̄), solves ONE QP for the step d and
// backtracks on an ℓ₁ merit.
//
//  • Hard nonlinear rows are elastic. Each (row group, node) has one variable
//    e ≥ 0 shared by the group's rows there (an ℓ∞ penalty inside the group)
//    with the linear cost μ_G e. The solution counts as FEASIBLE only when the
//    rows, re-evaluated on the nonlinear model, hold to tol_violation — the
//    elastic is the diagnosis of an infeasible problem, never a relaxation
//    that gets published. Rows are normalised per group so that one tolerance
//    means something: torque by 1/τ_max; gap, entrance, lateral and timing in
//    metres; the velocity set in m/s; the impact rows by their thresholds
//    (g_n in m/s).
//  • The merit has the QP's own structure, φ = J + Σ_G μ_G Σ_nodes max_i viol_i
//    with the corridor slacks at their smallest feasible value, so the QP's
//    predicted decrease is evidence of descent. The Armijo test allows for
//    what the QP solution itself leaves of those rows (their residual times
//    μ, as measured on that solution): the accuracy a step can be judged to
//    is the solver's, and the KKT residual this core can reach is therefore
//    bounded below by about μ · solver.eps_abs. The penalty is exact only for
//    μ_G > Σ_{i∈G}|λ_i|; while an elastic stays positive the multipliers sit AT
//    μ and say nothing about how large μ should be. What does tell is the
//    QP's response: the penalties are grown geometrically (mu_growth, until
//    the largest reaches mu_max) and the QP solved again, and each step is
//    KEPT only if it buys linearised feasibility (mu_min_gain,
//    DockingPenaltyGrowthKept) — otherwise the rows cannot be met at this
//    linearisation, and a larger μ would only worsen the conditioning. ALL
//    penalties grow by the same factor: their
//    ratio is the caller's (mu_init), and it decides in which group an
//    infeasible problem leaves its residual.
//  • The starting point always satisfies the linear rows (box, terminal rest):
//    the caller's node trajectory projected onto the block jerk when that
//    satisfies them, otherwise an initialisation QP — nearest to a target under
//    exactly those rows. Line search keeps iterates in that convex set, so the
//    QP stays feasible with any trust region. The target is the caller's
//    trajectory, or the catch-node joint pose (the search's IK solution) with
//    the joint velocity that gives ν_ref there; it is never truncated to the
//    velocity limits first — the QP's rows do that.
//  • Guards (inactive wherever the constraint they protect holds): ℓ⁺ in the
//    corridor, c̃ = max(c, c_min) in the crossing-plane map, ε_σ under every
//    square root (mpc_docking_relative_state.hpp).
//  • An infeasible problem ends kInfeasible when, with an elastic left that a
//    larger penalty does not remove, the iterate is stationary OR the hard
//    rows' violation has stopped falling (stall_window, stall_reduction).
//  • A step that rests on the trust region (delta_tr) proves nothing: the
//    QP's multipliers are then not the problem's, and slow progress is the
//    cap's doing. While `step_capped`, neither kConverged nor the two
//    kInfeasible tests above can end the solve — a feasible problem under a
//    small δ ends kIterationLimit or kDeadline.
//  • max_iterations = 1 is one real-time iteration: a full step, no line
//    search. It reports `converged` only when the point it STARTED from
//    already meets the test (it then takes no step).
//  • The deadline is read between iterations (a QP in progress is bounded only
//    by the solver's own iteration cap). Past it, the last ACCEPTED iterate is
//    returned with its violations and KKT residual.
//
// ── The catch instant as a variable (catch_time_variable, E1-F14 #740) ────────
// A search that keeps one candidate per lattice cell wants the best catch
// instant INSIDE the cell. With `catch_time_variable` the core adds one
// variable, δt_c: the catch node is at t̂ + δt_c, t̂ the instant the caller's
// grid is anchored at. Off (the default), none of what follows exists — the
// dimensions, the code path and every number are those of the core without it.
//
//  • The grid stretches in ONE place. Nodes before the catch node stay where
//    they are; the interval that ends at the catch node has length
//    τ = Δ_a + δt_c; the nodes after it follow t̂ + δt_c at Δ_s. So x_0, node
//    0's instant and every pre-catch node are independent of δt_c, and
//      x_kc = cubic of interval k_c − 1 evaluated at τ (exact),
//      ∂x_kc/∂δt_c = f̄_c = (q̇_kc, q̈_kc, u_{kc−1}),
//      ∂x_k/∂δt_c = A(Δ_s)^{k−kc} f̄_c                              k > k_c.
//    Rows and costs on nodes before the catch node have no δt_c column — with
//    one exception in the COST: the near term's reference line
//    r_ref,k = r_ref + (t̂ + δt_c − t_k)(−ν_ref) moves with the catch instant.
//  • The QP's variables gain θ = δt_c/Δ_a next to the jerk step: [d | θ | …],
//    with the column of every row and cost term that moves with the catch
//    instant. The stage gains and the free response of nodes k ≥ k_c are
//    rebuilt at the δt_c of the point being evaluated.
//  • The ball at the catch node is read from the caller's prediction at
//    t̂ + δt_c (SampleBallNode — the interpolant every other consumer reads),
//    and its rows move at ∂/∂p_b · v̂_b + ∂/∂v_b · â_b. The covariance stays
//    the caller's (the candidate's own instant), as do the near weights
//    ρ_T(t_k), the approach set A and the running-cost weight Δ_a of the
//    stretched interval — its jerk is still u_{kc−1}, recovered over τ.
//  • Two row groups depend on δt_c through the stage gains: the box on nodes
//    k ≥ k_c and the terminal rest. They are elastic like every other hard
//    nonlinear row (one elastic per node for the box, one for the rest; μ in
//    mu_init_post_box / mu_init_terminal), enter the merit, and decide
//    `feasible` on the nonlinear model as before — so a trajectory brought
//    from another catch instant is a start, whatever it leaves of them. The
//    trust region on those nodes is a row of its own (it shared the box's).
//    Box rows before the catch node and the jerk box stay hard and linear.
//  • δt_c does NOT move inside a QP: θ's row pins it. A joint step in (d, θ)
//    was tried first and does not work — moving the catch instant with the
//    jerk held leaves the terminal rest and the entrance plane off by a term
//    of second order in the step (jerk change × time change) that the ℓ₁
//    merit, at penalties sized for infeasibility and not for this, weighs far
//    above what the objective gains: measured from a fixed-grid solution, a
//    20 ms step left 1.3 rad/s of terminal speed for a gain of 0.02 in J, and
//    no step above 0.1 µs passed the line search. So the two are nested:
//      inner  the SQP above at a fixed δt_c — all of it, unchanged;
//      outer  when the inner problem is settled (solved, or stationary with a
//             row it cannot meet), the pin's multiplier is −∂L/∂θ, which by
//             the envelope theorem is the derivative of the inner problem's
//             optimal value in θ. δt_c moves on it by a secant kept inside a
//             bracket of its sign change, by at most delta_t_step, to a whole
//             nanosecond inside the caller's box [delta_lo_ns, delta_hi_ns].
//    The solve ends kConverged when the inner problem is solved AND the
//    derivative is within the KKT tolerance — or points out of the box at its
//    end, or changes sign between two neighbouring nanoseconds. kkt_residual
//    covers both parts. One real-time iteration (max_iterations = 1) cannot
//    move the catch instant.
//  • The returned δt_c is a whole number of nanoseconds (the payload's and the
//    RT's resolution), and so is every instant the solve stood at.
//  • The cost gains c₁ δt_c + c₂ (δt_c − δ_ref)² (the caller's outer terms in
//    the catch instant). It is reported apart (cost.time) and is not in
//    cost.total; the merit and the QP carry it.
//
// ── Result ────────────────────────────────────────────────────────────────────
// Two flags, never merged: `feasible` (every hard row, re-evaluated on the
// nonlinear model at the returned iterate, holds to tol_violation) and
// `converged` (KKT at a feasible point). Whether a feasible but
// unconverged iterate may be published is the planner's decision. `reason`
// names why the solve ended; on kInfeasible `infeasible_group` is the group
// with the largest remaining violation.
//
// ── Contracts ─────────────────────────────────────────────────────────────────
//  • Init() is non-RT (copies the model, allocates everything). Solve() is
//    noexcept. It returns false — leaving the result's trajectory untouched
//    and every other field of the result reset, so that nothing of an earlier
//    solve stands next to this call's `reason` — only when it rejects the call
//    before any iterate exists (not initialised, bad sizes or values, x_0
//    outside the box, unusable ball input, no linear-feasible start).
//    Otherwise it returns true and the result holds the last accepted
//    iterate, whatever `reason` says.
//  • Heap: this core allocates nothing in Solve() — pinned stage by stage with
//    a C-level malloc gate (SetStageHook). ProxQP does allocate inside its own
//    update()/solve() (#654), which is why this core must not yet run on a
//    SCHED_FIFO planner thread on hardware: that waits for #654.
//  • One core per grid: the dimensions are fixed at Init (the solver's are).
//    Only n_pre ≥ 1 is solved here; a replan after the catch is the stop
//    problem MpcSegmentCore already solves.
//  • The model is the hand-locked arm in pinocchio velocity order (nq == nv);
//    its armature is ADDED to by the limits' armature.
//  • Limits are enforced at the nodes only.
#pragma once

#include "rtc_controllers/catching/ball_node_samples.hpp"
#include "rtc_controllers/catching/mpc_docking_relative_state.hpp"
#include "rtc_controllers/catching/mpc_segment_core.hpp"  // kMaxMpcNodes
#include "rtc_controllers/catching/trajectory.hpp"        // kMaxSegmentNodes, kMaxPlanNv
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace rtc::catching {

/// Longest stall window (iterations of violation history the core keeps).
inline constexpr int kDockingStallHistory = 16;
/// Faces of the lateral capture polygon a core can hold.
inline constexpr int kMaxDockingFaces = 8;
/// Faces of the inscribed polygon that bounds the lateral speed.
inline constexpr int kMaxDockingSpeedFaces = 16;

/// Row groups. The first kNumDockingElasticGroups carry an elastic variable
/// and a penalty μ_G; the last two are the linear rows (never relaxed).
enum class DockingRowGroup : std::uint8_t {
  kTorque = 0,   ///< τ_lo ≤ τ ≤ τ_hi, fraction of τ_max
  kGap,          ///< ℓ_k ≥ 0 on the approach nodes [m]
  kEntrance,     ///< ℓ_kc = 0 [m]
  kLateral,      ///< lateral chance rows [m]
  kTiming,       ///< timing row [m]
  kVelocitySet,  ///< tightened V_cap [m/s]
  kImpact,       ///< g_n [m/s], E/E_max, P/P_max
  /// q, q̇ (q̈, u) box — largest excess, in rad, rad/s (rad/s²; the jerk excess
  /// in units of u_s, the scale its rows are solved in).
  kBox,
  kTerminal,  ///< q̇_N = q̈_N = 0 [rad/s, rad/s²]
};
inline constexpr int kNumDockingElasticGroups = 7;
inline constexpr int kNumDockingRowGroups = 9;

enum class MpcDockingReason : std::uint8_t {
  kNone = 0,  ///< Init succeeded; Evaluate() ran (never a Solve outcome)
  // ── Init ──
  kNotInitialized,
  kParamsInvalid,
  kBlocksTooFew,
  kBlocksAcrossCatch,
  kLimitsInvalid,
  kModelUnsupported,
  kFrameUnknown,
  kTerminalRankDeficient,
  kTimingWindowInvalid,  ///< δ₀ not strictly inside the window, or σ_max ≤ σ_τ
  // ── Solve rejected before any iterate ──
  kDimMismatch,
  kNonFinite,
  kInputOutOfRange,
  kInitialStateOutsideBox,
  kBallInvalid,        ///< a node's ball mean is not valid, or lies past the prediction
  kCovarianceInvalid,  ///< chance rows on, catch-node covariance unusable or not PSD
  kTargetRequired,     ///< no initial trajectory and no catch-node target
  kLinearInfeasible,   ///< no start point satisfies box + terminal rest (solver certificate)
  // ── Solve ended with an iterate ──
  kConverged,  ///< KKT conditions met at a feasible point
  kIterationLimit,
  kDeadline,
  kInfeasible,        ///< stationary (or no descent) with a hard row still violated
  kLineSearchFailed,  ///< no acceptable step, hard rows hold
  /// A QP did not converge. From the main QP an iterate exists (Solve returns
  /// true); from the initialisation QP none does (Solve returns false).
  kQpFailed,
  kSolutionNonFinite,
};

/// @brief Stable name for logs and test messages.
[[nodiscard]] const char* MpcDockingReasonName(MpcDockingReason reason) noexcept;
[[nodiscard]] const char* DockingRowGroupName(DockingRowGroup group) noexcept;

/// @brief Whether a penalty growth step is kept (RT-safe). The rule Solve()
///        applies to each probe — a free function so that its test can drive
///        the cases a solve reaches rarely.
///
/// Kept when the larger penalty buys feasibility of the LINEARISED rows: the
/// QP's elastic total falls by `min_gain`, or — when a trust region caps what
/// one step can remove, so that the total barely moves — what the step removes
/// of the violation grows by `min_gain` and by more than `tol_violation`. A
/// step that adds violation counts as removing none.
/// @param violation      Σ over groups of the hard rows' violation at the iterate
/// @param elastic_before Σ of the QP's elastics at the smaller penalties
/// @param elastic_after  Σ of the QP's elastics at the grown ones
/// @param min_gain       params.mu_min_gain, in (0, 1)
/// @param tol_violation  params.tol_violation
[[nodiscard]] bool DockingPenaltyGrowthKept(double violation, double elastic_before,
                                            double elastic_after, double min_gain,
                                            double tol_violation) noexcept;

/// Tuning. Every field is validated by Init(). Vectors of size n may be left
/// empty for the default noted on each.
struct MpcDockingSegmentCoreParams {
  // ── Grid ──
  int n_pre{4};          ///< pre-catch intervals (≥ 1); k_c = n_pre
  double dt_pre{0.04};   ///< Δ_a [s]
  int n_stop{8};         ///< stop intervals (≥ 3)
  double dt_stop{0.05};  ///< Δ_s [s]
  int n_blocks{9};       ///< B ≤ kMaxSegmentNodes
  /// First B used, Σ = N. No block may span the catch node; at least 3 after it.
  std::array<int, kMaxSegmentNodes> block_sizes{1, 1, 1, 1, 1, 1, 2, 2, 2};
  double u_scale{1e3};  ///< jerk scale u_s [rad/s³]; the variable is ũ/u_s

  // ── Motion cost (reference §9.2), pre-catch stages ──
  Eigen::VectorXd r_tau;      ///< R_τ diag [1/(N·m)²]; empty = off
  Eigen::VectorXd r_acc;      ///< R_a diag [1/(rad/s²)²]; empty = off
  Eigen::VectorXd r_jerk;     ///< R_j diag on u/u_s, > 0; empty = all 1
  Eigen::VectorXd q_nom;      ///< nominal posture [rad]; read when w_q_nom is set
  Eigen::VectorXd w_q_nom;    ///< Q_q diag [1/rad²]; empty = off
  double w_manip{0.0};        ///< w_m; 0 = ψ_m is not computed
  double manip_d_lin{1.0};    ///< D_x translation scale [m]
  double manip_d_ang{1.0};    ///< D_x rotation scale [rad]
  Eigen::VectorXd manip_d_q;  ///< D_q diag; empty = all 1
  double manip_delta{1e-3};   ///< δ

  // ── Catch-vicinity and terminal cost (§9.3, §9.4) ──
  Eigen::Vector3d q_p{Eigen::Vector3d::Zero()};      ///< Q_p diag [1/m²]
  Eigen::Vector3d q_v{Eigen::Vector3d::Zero()};      ///< Q_v diag [(m/s)⁻²]
  double sigma_T{0.1};                               ///< σ_T [s], > 0
  Eigen::Vector2d rho_ref{Eigen::Vector2d::Zero()};  ///< ρ_ref [m]
  Eigen::Vector3d nu_ref{0.0, 0.0, -0.5};            ///< ν_ref [m/s]; −e₃ᵀν_ref > 0
  Eigen::Vector2d q_rho_f{Eigen::Vector2d::Zero()};  ///< Q_ρ,f diag [1/m²]
  Eigen::Vector3d q_nu_f{Eigen::Vector3d::Zero()};   ///< Q_ν,f diag [(m/s)⁻²]
  double w_impact{0.0};                              ///< w_E; 0 = the impact cost is off
  double e_ref{1.0};                                 ///< E_ref [J], > 0

  // ── Approach rows (§6.3, §6.4) ──
  double approach_window{0.15};  ///< nodes with 0 < t_c − t_k ≤ window form A [s]
  double s_ent{0.05};            ///< entrance plane offset s_ent [m]
  double r_ent{0.03};            ///< corridor radius at the entrance [m], > 0
  double tan_theta{0.3};         ///< corridor half-angle tangent, ≥ 0
  double c_ent_max{1.0};         ///< closing speed allowed at the entrance [m/s]
  double a_brake{5.0};           ///< relative braking [m/s²], ≥ 0
  /// Slack penalties; each slack needs λ₁ + λ₂ > 0 (a free slack would switch
  /// its row off).
  double lambda1_c{1.0};  ///< linear penalty on s_c [1/m]
  double lambda2_c{0.0};  ///< quadratic penalty on s_c [1/m²]
  double lambda1_v{1.0};  ///< linear penalty on s_v [(m/s)⁻²]
  double lambda2_v{0.0};  ///< quadratic penalty on s_v [(m/s)⁻⁴]

  // ── Capture set at the catch node (§6.1, §6.5, §8.3) ──
  /// false: the ball is treated as known exactly (Σ = 0) — the chance rows
  /// reduce to their deterministic rows tightened by κ ε_σ, the timing row is
  /// not built, and no covariance is required.
  bool chance{true};
  /// Lateral faces a_iᵀρ ≤ b_i in ball-centre coordinates, with UNIT normals
  /// a_i (Init rejects others — header of mpc_docking_relative_state.hpp).
  int n_faces{4};
  std::array<Eigen::Vector2d, kMaxDockingFaces> face_a{
      Eigen::Vector2d{1.0, 0.0}, Eigen::Vector2d{-1.0, 0.0}, Eigen::Vector2d{0.0, 1.0},
      Eigen::Vector2d{0.0, -1.0}};
  std::array<double, kMaxDockingFaces> face_b{0.03, 0.03, 0.03, 0.03};            ///< [m]
  std::array<double, kMaxDockingFaces> face_eps{0.0125, 0.0125, 0.0125, 0.0125};  ///< risk ε_i
  double c_min{0.2};                                                              ///< [m/s], > 0
  double c_cap_max{1.0};   ///< [m/s], > c_min
  double v_perp_max{0.3};  ///< [m/s], > 0
  int speed_faces{8};      ///< m of the inscribed polygon, 3..kMaxDockingSpeedFaces
  double eps_nu{0.01};     ///< risk of each velocity row → κ_ν
  double eps_sigma{1e-4};  ///< ε_σ under every square root [m or m/s], > 0

  // ── Timing row (§8.4) ──
  bool timing_row{true};  ///< built only with `chance`
  double delta_lo{0.0};   ///< closure window after the crossing [s]
  double delta_hi{0.03};
  double delta_0{0.015};  ///< nominal closure instant after the crossing [s]
  double eps_t{0.05};     ///< timing risk
  double sigma_tau{0.0};  ///< closure latency jitter [s], ≥ 0

  // ── Impact (§7) ──
  Eigen::Vector3d contact_point_hand{Eigen::Vector3d::Zero()};  ///< p_c in the capture frame [m]
  double m_ball{0.058};                                         ///< [kg], > 0
  double restitution{0.5};                                      ///< e ∈ [0, 1]
  double e_max{std::numeric_limits<double>::infinity()};        ///< E_max [J]; inf = row off
  double p_max{std::numeric_limits<double>::infinity()};        ///< P_max [N·s]; inf = row off

  // ── Stop segment ──
  Eigen::VectorXd r_jerk_stop;  ///< R_j,stop diag on u/u_s, > 0; empty = r_jerk
  double w_perp{0.0};           ///< stop-line weight [1/m²]; 0 = off

  // ── Rows ──
  bool accel_box{false};  ///< |q̈_k| ≤ q̈_max rows
  bool jerk_box{false};   ///< |u_k| ≤ j_max rows

  // ── The catch instant as a variable (header note) ──
  bool catch_time_variable{false};
  /// Most that δt_c moves in one iteration [s], > 0 (read with the flag).
  double delta_t_step{0.01};
  /// Penalties of the two groups that are elastic only with the flag: the box
  /// on nodes k ≥ k_c and the terminal rest. > 0, ≤ mu_max.
  double mu_init_post_box{1e2};
  double mu_init_terminal{1e2};

  // ── SQP ──
  int max_iterations{50};  ///< 1 = one real-time iteration
  double armijo_eta{1e-4};
  double backtrack_beta{0.5};
  int max_backtracks{12};
  /// Trust region |Δq_k| ≤ δ per step [rad]. A step resting on it cannot end
  /// the solve as converged or infeasible (header note).
  double delta_tr{std::numeric_limits<double>::infinity()};
  std::array<double, kNumDockingElasticGroups> mu_init{1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2};
  double mu_growth{10.0};  ///< > 1
  double mu_max{1e6};      ///< ≥ every mu_init
  /// A growth step is kept only if it lowers the QP's elastic total by at
  /// least this fraction, or raises by this fraction what the step removes of
  /// the linearised violation (the case under a trust region, where one step
  /// cannot remove much); otherwise it is undone (header note). In (0, 1).
  double mu_min_gain{0.1};
  /// Infeasibility by stall: with an elastic left in the QP, the solve ends
  /// kInfeasible once the hard rows' violation has fallen by less than
  /// stall_reduction (a fraction, in (0, 1)) over stall_window iterations
  /// (1..kDockingStallHistory).
  int stall_window{5};
  double stall_reduction{0.01};
  double tol_violation{1e-6};  ///< hard rows, in their group's unit
  double tol_kkt{1e-4};        ///< ‖∇L‖∞ ≤ tol_kkt · max(1, ‖∇J‖∞)
  double tol_complementarity{1e-5};
  /// Box and terminal-rest tolerance for accepting a projected start point.
  double tol_linear{1e-7};

  // ── Initialisation QP ──
  double init_w_q{1.0};            ///< weight of the position distance [1/rad²], > 0
  double init_w_v{0.05};           ///< weight of the velocity distance [(rad/s)⁻²], ≥ 0
  double init_pinv_damping{0.05};  ///< damping of J_p^# for the target velocity [m]

  /// Re-equilibrated every solve; PrimalDualLDLT (see QPSolverConfig). eps_rel
  /// is NOT zero: the elastic cost makes the multipliers as large as μ, and an
  /// absolute tolerance alone does not terminate on them.
  /// ProxQP's primal-infeasibility test is OFF (eps_primal_inf = 0): every QP
  /// here is feasible by construction (the start satisfies the linear rows and
  /// every nonlinear row has an elastic or a slack), and the test's approximate
  /// certificate fired on such QPs — at μ = 100 under a 0.05 rad trust region,
  /// and regularly from μ ≈ 1e4. The INITIALISATION QP does not take that
  /// setting: it is the one QP that can be infeasible, and it always runs with
  /// QPSolverConfig's default threshold.
  tsid::QPSolverConfig solver{.eps_abs = 1e-7,
                              .eps_rel = 1e-9,
                              .max_iter = 400,
                              .update_preconditioner = true,
                              .dense_backend = proxsuite::proxqp::DenseBackend::PrimalDualLDLT,
                              .eps_primal_inf = 0.0};
};

/// Joint limits in pinocchio velocity order (n each).
struct MpcDockingSegmentCoreLimits {
  Eigen::VectorXd q_min, q_max;  ///< [rad]; the box the nodes are held to
  Eigen::VectorXd qd_max;        ///< > 0 [rad/s]
  Eigen::VectorXd qdd_max;       ///< > 0 [rad/s²]; read with accel_box
  Eigen::VectorXd jerk_max;      ///< > 0 [rad/s³]; read with jerk_box
  Eigen::VectorXd tau_max;       ///< > 0 [N·m]: the normalisation of the torque rows
  /// Torque row bounds [N·m] (the caller's margin already applied);
  /// empty = −τ_max and +τ_max.
  Eigen::VectorXd tau_lo, tau_hi;
  Eigen::VectorXd armature;  ///< ≥ 0 [kg·m²], ADDED to the model's
};

struct MpcDockingSegmentCoreInput {
  Eigen::VectorXd q0, qd0, qdd0;  ///< x_0 (n each)
  /// The ball at nodes 0..k_c (index = node). Every mean must be valid and
  /// inside the prediction (`after_horizon` false — an extrapolated ball is
  /// rejected, kBallInvalid); only the catch node's covariance is read (with
  /// params.chance).
  std::array<BallNodeSample, kMaxMpcNodes + 1> ball{};
  /// Optional start trajectory, n × (N+1) nodes. It is PROJECTED onto the block
  /// jerk: the start is rebuilt from x_0 with, per block, the mean of the
  /// node-acceleration differences — so a solution of this core on the same
  /// grid is reproduced exactly, and of any other trajectory only its
  /// acceleration profile is taken. If that start breaks a linear row (it
  /// does when x_0 has moved off the trajectory's first node), the
  /// initialisation QP finds the start nearest to these nodes' q and q̇.
  Eigen::MatrixXd q_init, qd_init, qdd_init;
  bool initial_valid{false};
  /// Catch-node joint pose for the initialisation QP when there is no start
  /// trajectory (the search's IK solution), n.
  Eigen::VectorXd q_catch_target;
  bool catch_target_valid{false};
  /// Stop line (read when w_perp > 0): the hand stops on the line through
  /// p_line along the unit d_line.
  Eigen::Vector3d p_line{Eigen::Vector3d::Zero()};
  Eigen::Vector3d d_line{Eigen::Vector3d::UnitX()};
  /// Absolute instant on the core's clock [ns]; ≤ 0 = no deadline.
  std::int64_t deadline_ns{0};

  // ── Read only by a core with catch_time_variable ──
  /// The prediction the catch node's ball is read from (its mean; the
  /// covariance is ball[k_c]'s). Not owned; must outlive the call.
  const TrajectorySnapshot* prediction{nullptr};
  /// t̂: the catch node's instant on the ball axis at δt_c = 0 [ns].
  std::int64_t t_catch_ns{0};
  /// δt_c of the start point and its box [ns], lo ≤ start ≤ hi; the catch
  /// interval must keep a positive length at lo. The start trajectory (when
  /// there is one) is on the grid stretched by delta_start_ns.
  std::int64_t delta_start_ns{0};
  std::int64_t delta_lo_ns{0};
  std::int64_t delta_hi_ns{0};
  /// The cost's terms in the catch instant: c₁ δt_c + c₂ (δt_c − δ_ref)²
  /// (c₁ [1/s], c₂ ≥ 0 [1/s²], δ_ref [ns]).
  double time_c1{0.0};
  double time_c2{0.0};
  std::int64_t delta_ref_ns{0};
};

/// J at the returned iterate, by term (the reference's cost: no ½).
struct MpcDockingCost {
  double total{0.0};      ///< reference + stop
  double reference{0.0};  ///< motion + near + terminal + slack — the reference's J
  double stop{0.0};       ///< the stop segment's terms
  double tau{0.0}, acc{0.0}, jerk{0.0}, posture{0.0}, manip{0.0};  ///< motion
  double near{0.0};
  double terminal{0.0};  ///< ρ and ν terms of V_f
  double impact{0.0};    ///< w_E E_n / E_ref
  double slack{0.0};
  double stop_jerk{0.0}, stop_line{0.0};
  /// c₁ δt_c + c₂ (δt_c − δ_ref)² — NOT part of `total`; 0 without
  /// catch_time_variable.
  double time{0.0};
};

struct MpcDockingSegmentCoreResult {
  Eigen::MatrixXd q, qd, qdd;  ///< nodes, n × (N+1)
  Eigen::MatrixXd u;           ///< jerk of interval k, n × N [rad/s³]
  MpcDockingCost cost;
  /// Largest violation per group at the returned iterate, re-evaluated on the
  /// NONLINEAR model, in the group's unit (≥ 0).
  std::array<double, kNumDockingRowGroups> violation{};
  /// Largest elastic value per group in the last QP.
  std::array<double, kNumDockingElasticGroups> elastic{};
  std::array<double, kNumDockingElasticGroups> mu{};  ///< penalties at exit
  /// Corridor / envelope slack per approach node (index = position in A),
  /// the smallest feasible value at the returned iterate.
  std::array<double, kMaxMpcNodes> slack_c{};
  std::array<double, kMaxMpcNodes> slack_v{};
  /// ‖g + Cᵀλ + Aᵀy‖∞ on the jerk variables, last QP — without the trust
  /// region's own multipliers, which are not the problem's.
  double kkt_residual{0.0};
  double grad_norm{0.0};        ///< ‖∇J‖∞ on the jerk variables, same point
  double complementarity{0.0};  ///< max |λ_i · gap_i| of the last QP
  /// The last QP's step rests on the trust region (delta_tr): the multipliers
  /// of that bound are large enough to decide the stationarity test.
  bool step_capped{false};
  // ── Diagnostics at the catch node ──
  double c_catch{0.0};    ///< closing speed [m/s]
  double sigma_s{0.0};    ///< [m] (0 without `chance`)
  double sigma_t{0.0};    ///< [s]
  bool c_guarded{false};  ///< c ≤ c_min at the returned iterate
  /// ½‖E⊥ᵀ a_rel‖(κ σ_t)² over the smallest lateral margin — the size of the
  /// term the crossing-plane map drops. Read only when the flag is set (the
  /// margin is positive, c is not guarded, `chance` is on).
  double linearization_ratio{0.0};
  bool linearization_ratio_defined{false};
  double tau_ratio_max{0.0};  ///< max |τ/τ_max| over nodes 1..N (RNEA)
  int approach_nodes{0};      ///< |A|
  // ── With catch_time_variable (zero otherwise) ──
  /// δt_c of the returned iterate [ns] — the catch node is at t̂ + delta_ns.
  std::int64_t delta_ns{0};
  /// Largest elastic of the last QP and the penalty at exit, for the box on
  /// nodes k ≥ k_c and for the terminal rest.
  double elastic_post_box{0.0}, elastic_terminal{0.0};
  double mu_post_box{0.0}, mu_terminal{0.0};
  /// Iterations whose QP step rested on the trust region (delta_tr).
  int capped_iterations{0};
  /// Moves of the catch instant, and ∂L/∂θ at the returned one (θ = δt_c/Δ_a).
  int catch_time_steps{0};
  double catch_time_gradient{0.0};
  // ── Iteration record ──
  int iterations{0};     ///< SQP iterations (QP linearisations)
  int qp_solves{0};      ///< QPs solved, μ re-solves and cold retries included
  int qp_iterations{0};  ///< ProxQP iterations, summed
  int backtracks{0};     ///< step halvings, summed over iterations
  int mu_updates{0};     ///< QP re-solves after growing a penalty
  bool init_qp_used{false};
  int qp_status{-1};  ///< proxsuite QPSolverOutput of the last QP
  double start_us{0.0}, linearize_us{0.0}, assemble_us{0.0}, qp_us{0.0}, merit_us{0.0};
  double total_us{0.0};
  MpcDockingReason reason{MpcDockingReason::kNotInitialized};
  DockingRowGroup infeasible_group{DockingRowGroup::kTorque};  ///< read on kInfeasible
  bool feasible{false};
  bool converged{false};
};

/// Stages of Solve() that run outside the QP solver; the hook brackets each
/// (test seam for the per-stage allocation gate).
enum class MpcDockingStage : std::uint8_t {
  kStart = 0,  ///< input checks, start-point projection, init-QP assembly
  kLinearize,  ///< nonlinear evaluation with derivatives at the iterate
  kAssemble,   ///< QP matrices and bounds
  kPostQp,     ///< penalty update, KKT residual, complementarity
  kMerit,      ///< trial-point evaluation and step acceptance
  kFinish,     ///< final evaluation and the result
};

class MpcDockingSegmentCore {
 public:
  using ClockFn = std::int64_t (*)() noexcept;
  using StageHook = void (*)(MpcDockingStage stage, bool begin, void* user) noexcept;

  MpcDockingSegmentCore() = default;

  /// @brief Build the problem for one arm and one grid (non-RT).
  /// @param arm hand-locked arm model, pinocchio velocity order
  /// @param catch_frame the capture frame; its +z is the approach axis e₃
  /// @param clock steady clock for the deadline [ns]; nullptr = no deadline
  /// @return kNone on success; the core stays uninitialised otherwise.
  [[nodiscard]] MpcDockingReason Init(const pinocchio::Model& arm,
                                      pinocchio::FrameIndex catch_frame,
                                      const MpcDockingSegmentCoreParams& params,
                                      const MpcDockingSegmentCoreLimits& limits, ClockFn clock);

  /// @brief Replace the clock (non-RT; nullptr disables the deadline).
  void SetClock(ClockFn clock) noexcept { clock_ = clock; }

  /// @brief Bracket every stage outside the QP solver with `hook` (test seam;
  ///        nullptr removes it).
  void SetStageHook(StageHook hook, void* user) noexcept {
    stage_hook_ = hook;
    stage_user_ = user;
  }

  /// @brief Size a result / an input's matrices for this problem (non-RT).
  void ResizeResult(MpcDockingSegmentCoreResult& result) const;
  void ResizeInput(MpcDockingSegmentCoreInput& input) const;

  /// @brief Solve from input (RT-safe apart from the QP solver — header note).
  /// @return false when the call was rejected before any iterate existed
  ///         (`out`'s trajectory untouched); true otherwise — read
  ///         out.feasible, out.converged and out.reason.
  [[nodiscard]] bool Solve(const MpcDockingSegmentCoreInput& in,
                           MpcDockingSegmentCoreResult& out) noexcept;

  /// @brief Evaluate the caller's trajectory WITHOUT solving: the cost by term
  ///        and every hard row's violation on the nonlinear model (RT-safe).
  ///
  /// For judging a trajectory against a newer prediction, and for tests. The
  /// trajectory is `in`'s start trajectory projected onto the block jerk
  /// (exact for a trajectory this core produced on the same grid) — it is NOT
  /// required to satisfy the linear rows; their violation is reported.
  /// @return false when the input is rejected (`out`'s trajectory untouched);
  ///         true otherwise, with reason kNone, `converged` false and
  ///         `feasible` as evaluated.
  [[nodiscard]] bool Evaluate(const MpcDockingSegmentCoreInput& in,
                              MpcDockingSegmentCoreResult& out) noexcept;

  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

  [[nodiscard]] int Nv() const noexcept { return n_; }

  /// Whether the catch instant is a variable of this core (header note).
  [[nodiscard]] bool CatchTimeVariable() const noexcept { return tc_on_; }

  /// N = n_pre + n_stop: results hold N + 1 node columns.
  [[nodiscard]] int NumNodes() const noexcept { return n_nodes_; }

  /// The catch node k_c (= n_pre).
  [[nodiscard]] int CatchNode() const noexcept { return n_pre_; }

  [[nodiscard]] int NumBlocks() const noexcept { return n_blocks_; }

  /// Instant of node k relative to node 0 [s]; NaN outside [0, N].
  [[nodiscard]] double NodeTime(int k) const noexcept;
  /// Scaled stage gain ĝ_{m,k}[b] (m: 0 = q, 1 = q̇, 2 = q̈) — the same
  /// quantity as MpcSegmentCore::StageGain on the same grid.
  /// With catch_time_variable both are those of the point evaluated LAST (its
  /// δt_c), for the nodes k ≥ k_c; before any Solve, of δt_c = 0.
  [[nodiscard]] double StageGain(int m, int k, int b) const noexcept;

  /// |A| and its nodes (ascending); −1 outside.
  [[nodiscard]] int NumApproachNodes() const noexcept { return n_app_; }

  [[nodiscard]] int ApproachNode(int i) const noexcept;
  /// κ_i of lateral face i, κ_ν, and the timing row's σ_max (0 when the row is
  /// not built) — the constants Init derived from the risks.
  [[nodiscard]] double FaceKappa(int i) const noexcept;

  [[nodiscard]] double VelocityKappa() const noexcept { return kappa_nu_; }

  [[nodiscard]] double TimingSigmaMax() const noexcept { return sigma_max_; }

  /// The joint box the rows are built on and Solve checks the start state
  /// against (model order, as Init took it): q ∈ [lower, upper], |q̇| ≤ limit.
  /// A caller that screens or projects a start state reads the SAME numbers
  /// here rather than keeping its own copy of the limits. Empty before Init.
  [[nodiscard]] const Eigen::VectorXd& PositionLower() const noexcept { return q_lo_; }

  [[nodiscard]] const Eigen::VectorXd& PositionUpper() const noexcept { return q_hi_; }

  [[nodiscard]] const Eigen::VectorXd& VelocityLimit() const noexcept { return v_hi_; }

  /// |q̈| ≤ limit — meaningful only with HasAccelerationBox(); zeros otherwise.
  [[nodiscard]] const Eigen::VectorXd& AccelerationLimit() const noexcept { return a_hi_; }

  [[nodiscard]] bool HasAccelerationBox() const noexcept { return params_.accel_box; }

  /// Rank of the assembled terminal-equality matrix found at Init (2n).
  [[nodiscard]] int TerminalRank() const noexcept { return terminal_rank_; }

  // ── The last QP (diagnostics and tests) ──
  /// Variables: [d (n·B) | s_c (|A|) | s_v (|A|) | e]. d is the step of ũ/u_s.
  /// With catch_time_variable: [d | θ | s_c | s_v | e], θ the step of δt_c/Δ_a.
  [[nodiscard]] const tsid::QPData& LastQp() const noexcept { return qp_; }

  [[nodiscard]] const Eigen::VectorXd& LastQpSolution() const noexcept { return x_qp_; }

  [[nodiscard]] const Eigen::VectorXd& LastEqualityDual() const noexcept { return y_qp_; }

  [[nodiscard]] const Eigen::VectorXd& LastInequalityDual() const noexcept { return z_qp_; }

  [[nodiscard]] int NumJerkVariables() const noexcept { return nu_; }

  /// First inequality row and row count of each elastic group's rows; the
  /// rows of a disabled group are open (±inf).
  [[nodiscard]] int GroupRowBegin(DockingRowGroup group) const noexcept;
  [[nodiscard]] int GroupRowCount(DockingRowGroup group) const noexcept;

  // ── With catch_time_variable only (−1 / 0 otherwise) ──
  /// Column of θ in the last QP.
  [[nodiscard]] int CatchTimeColumn() const noexcept { return tc_on_ ? nu_ : -1; }

  /// The row that bounds θ (δt_c's box and the step limit).
  [[nodiscard]] int CatchTimeRow() const noexcept { return tc_on_ ? row_th_ : -1; }

  /// The elastic box rows of the nodes k ≥ k_c: `PostCatchBoxRowCount()` rows
  /// "… − e ≤ upper" then as many "… + e ≥ lower"; inside each half, node by
  /// node (k_c first), then q, q̇ (, q̈), then joint.
  [[nodiscard]] int PostCatchBoxRowBegin() const noexcept { return tc_on_ ? row_pbox_ : -1; }

  [[nodiscard]] int PostCatchBoxRowCount() const noexcept { return tc_on_ ? n_pbox_ : 0; }

  /// The elastic terminal-rest rows: 2n "… − e ≤ upper" (q̇_N then q̈_N), then
  /// 2n "… + e ≥ lower".
  [[nodiscard]] int TerminalRowBegin() const noexcept { return tc_on_ ? row_term_ : -1; }

 private:
  struct Evaluation {
    MpcDockingCost cost;
    std::array<double, kNumDockingRowGroups> viol_max{};
    std::array<double, kNumDockingElasticGroups> viol_sum{};  // Σ_nodes max_i viol
    double tau_ratio_max{0.0};
    // With catch_time_variable: the two groups that are elastic only then.
    double viol_post_box{0.0};  // Σ over nodes k ≥ k_c of the node's largest box excess
    double viol_terminal{0.0};  // the largest |q̇_N|, |q̈_N|
    bool ok{false};
  };

  void StageBegin(MpcDockingStage s) const noexcept {
    if (stage_hook_ != nullptr) {
      stage_hook_(s, true, stage_user_);
    }
  }

  void StageEnd(MpcDockingStage s) const noexcept {
    if (stage_hook_ != nullptr) {
      stage_hook_(s, false, stage_user_);
    }
  }

  [[nodiscard]] MpcDockingReason Prepare(const MpcDockingSegmentCoreInput& in,
                                         const MpcDockingSegmentCoreResult& out,
                                         bool need_initial) noexcept;
  static void ResetRecord(MpcDockingSegmentCoreResult& out) noexcept;
  void FreeResponse() noexcept;
  /// Put the grid, the gains, the free response and the catch node's ball at
  /// δt_c = `delta` [s] (catch_time_variable only). False when the ball cannot
  /// be read there.
  [[nodiscard]] bool SetCatchOffset(double delta) noexcept;
  [[nodiscard]] bool CatchTimeStep(double tol) noexcept;
  void RefreshInitQpGains() noexcept;
  void PostCatchSensitivity() noexcept;
  void AddTheta(double rate) noexcept;
  [[nodiscard]] double Penalty(int group) const noexcept;
  void ElasticOfNewGroups(double& post_box_max, double& post_box_sum, double& terminal_max,
                          double& terminal_sum) const noexcept;
  void TrajectoryFromZ(const Eigen::VectorXd& z) noexcept;
  void ProjectInitial(const MpcDockingSegmentCoreInput& in) noexcept;
  [[nodiscard]] bool LinearRowsHold() const noexcept;
  [[nodiscard]] MpcDockingReason RunInitQp(const MpcDockingSegmentCoreInput& in,
                                           MpcDockingSegmentCoreResult& out) noexcept;
  [[nodiscard]] bool EvaluateTrajectory(bool with_jacobians, Evaluation& ev) noexcept;
  void AssembleQp() noexcept;
  void MapRow(Eigen::Index k, const Eigen::VectorXd* a_q, const Eigen::VectorXd* a_v) noexcept;
  void AddScalarResidual(double weight, double residual) noexcept;
  void SetRow(Eigen::Index row, double lower, double upper) noexcept;
  void SetPenaltyGradient() noexcept;
  void ElasticByGroup(std::array<double, kNumDockingElasticGroups>& max_out,
                      std::array<double, kNumDockingElasticGroups>& sum_out) const noexcept;
  [[nodiscard]] MpcDockingReason RunQp(MpcDockingSegmentCoreResult& out) noexcept;
  [[nodiscard]] bool GrowPenalties() noexcept;
  void RestorePenalties() noexcept;
  void PostQp(MpcDockingSegmentCoreResult& out) noexcept;
  [[nodiscard]] double Merit(const Evaluation& ev) const noexcept;
  [[nodiscard]] double LinearizedDecrease(const Evaluation& ev) const noexcept;
  void Finish(const Evaluation& ev, MpcDockingReason reason,
              MpcDockingSegmentCoreResult& out) noexcept;

  bool initialized_{false};
  MpcDockingSegmentCoreParams params_;
  ClockFn clock_{nullptr};
  StageHook stage_hook_{nullptr};
  void* stage_user_{nullptr};

  int n_{0};
  int n_nodes_{0};
  int n_pre_{0};
  int n_blocks_{0};
  int n_app_{0};
  int nu_{0};         // n·B
  int n_elastic_{0};  // N + |A| + 5
  int nx_{0};         // nu + 2|A| + n_elastic
  int n_eq_{0};       // 2n
  int n_in_{0};
  int n_in_init_{0};  // the linear rows alone (the leading rows of the main QP)
  int terminal_rank_{0};
  // Variable offsets in x, and elastic indices inside the e block.
  int o_sc_{0}, o_sv_{0}, o_e_{0};
  int e_torque_{0}, e_gap_{0}, e_ent_{0}, e_lat_{0}, e_tim_{0}, e_vel_{0}, e_imp_{0};
  // First inequality row of each block.
  int row_q_{0}, row_v_{0}, row_a_{0}, row_u_{0}, row_tau_{0}, row_e_{0}, row_s_{0}, row_app_{0},
      row_ent_{0}, row_lat_{0}, row_tim_{0}, row_vel_{0}, row_imp_{0};
  // The catch instant as a variable. With tc_on_ false: nw_ = nu_, the counts
  // are 0 and none of the rows exist.
  bool tc_on_{false};
  int nw_{0};         // nu + 1: the smooth variables [d | θ]
  int n_eq_main_{0};  // the main QP's equality rows: 2n, or 0 (the rest is elastic)
  int n_post_{0};     // nodes k_c..N
  int n_pbox_{0};     // n_post · n · (2 or 3): one half of the elastic box rows
  int e_pbox_{0}, e_term_{0};
  int row_th_{0}, row_pbox_{0}, row_term_{0};
  bool torque_cost_{false}, acc_cost_{false}, posture_cost_{false}, near_cost_{false};
  bool manip_on_{false}, impact_rows_{false}, impact_cost_{false}, impact_on_{false};
  bool timing_on_{false}, perp_on_{false};

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex frame_{0};

  Eigen::VectorXd q_lo_, q_hi_, v_hi_, a_hi_, u_hi_, inv_tau_, tau_lo_, tau_hi_;
  Eigen::VectorXd r_tau_, r_tau2_, r_acc_, r_jerk_, r_jerk_stop_, w_q_nom_, q_nom_, manip_d_q_;
  std::array<int, kMaxMpcNodes> block_of_node_{};
  std::array<int, kMaxSegmentNodes> block_first_{};    // first interval of block b
  std::array<double, kMaxSegmentNodes> block_span_{};  // Σ_{k∈b} Δ_k [s]
  std::array<double, kMaxMpcNodes + 1> t_node_{};
  std::array<double, kMaxMpcNodes> dt_node_{};
  std::array<int, kMaxMpcNodes> app_node_{};
  std::array<double, kMaxMpcNodes + 1> near_weight_{};  // ρ_T(t_k)
  std::array<double, kMaxDockingFaces> kappa_face_{};
  std::array<Eigen::Vector2d, kMaxDockingSpeedFaces> speed_face_{};
  double kappa_nu_{0.0};
  double sigma_max_{0.0};
  double k_timing_{0.0};
  double speed_face_bound_{0.0};

  Eigen::MatrixXd gq_, gv_, ga_;  // (N+1) × B, scaled by u_scale
  Eigen::VectorXd jerk_diag_;     // the jerk cost's Hessian diagonal (nu)
  Eigen::MatrixXd h_const_;       // constant Hessian of the jerk variables (nu × nu)
  Eigen::MatrixXd h_init_all_, h_init_catch_;

  tsid::QPSolverWrapper solver_;
  tsid::QPSolverWrapper init_solver_;
  tsid::QPData qp_;
  tsid::QPData qp_init_;
  Eigen::VectorXd x_qp_, y_qp_, z_qp_;
  Eigen::VectorXd x_keep_, y_keep_, z_keep_;  // the solution before a penalty probe
  std::array<double, kNumDockingElasticGroups> mu_keep_{};
  double mu_post_box_{0.0}, mu_terminal_{0.0};  // the two groups' penalties
  double mu_post_box_keep_{0.0}, mu_terminal_keep_{0.0};
  double e_post_box_sum_{0.0}, e_terminal_sum_{0.0};  // Σ e, last QP
  double kkt_jerk_{0.0};      // ‖∇L‖∞ on the jerk variables, last QP (trust region removed)
  double reduced_grad_{0.0};  // ∂L/∂θ at the last QP's δt_c (catch_time_variable)
  std::vector<int> row_group_;  // per inequality row: its elastic group, or −1
  /// Per position row: which of its bounds the trust region set (kTrust* bits).
  std::vector<std::uint8_t> trust_side_;
  Eigen::VectorXd trust_grad_;  // their multipliers' part of Cᵀλ (nu)
  double qp_noise_{0.0};        // Σ μ_G · (row residual)⁺ of the last QP solution
  Eigen::VectorXd cx_, kkt_;    // C·x (n_in), ∇L (nx)
  std::array<double, kNumDockingElasticGroups> elastic_sum_{};  // Σ e per group, last QP
  bool solver_warm_{false};

  // Iterate and trajectory.
  Eigen::VectorXd q0_, v0_, a0_;
  Eigen::MatrixXd qf_, vf_, af_;  // free response
  Eigen::VectorXd z_, z_trial_, d_;
  Eigen::MatrixXd q_, v_, a_;  // trajectory of the point being evaluated
  std::array<double, kNumDockingElasticGroups> mu_{};
  std::array<double, kDockingStallHistory> violation_hist_{};  // ring, by iteration

  // The catch instant: the iterate's δt_c, that of the point last evaluated,
  // the trial's; the caller's box and cost terms [s]; the stretched interval.
  double delta_{0.0}, delta_eval_{0.0};
  double delta_lo_{0.0}, delta_hi_{0.0}, delta_ref_{0.0};
  std::int64_t delta_ns_{0}, delta_lo_ns_{0}, delta_hi_ns_{0};
  // The catch instant's search: the bracket of its derivative's sign change
  // (an end is "known" once the derivative was read there) and the last move.
  std::int64_t bracket_lo_ns_{0}, bracket_hi_ns_{0}, prev_ns_{0};
  bool bracket_lo_known_{false}, bracket_hi_known_{false}, have_prev_{false};
  double prev_grad_{0.0};
  double time_c1_{0.0}, time_c2_{0.0};
  double tau_catch_{0.0};
  const TrajectorySnapshot* prediction_{nullptr};
  std::int64_t t_catch_ns_{0};
  int ball_hint_{0};
  // ∂x_k/∂δt_c for k ≥ k_c (n × (N+1)), and the catch rows' rates along the ball.
  Eigen::MatrixXd sq_, sv_, sa_;
  Eigen::Vector3d r_rate_{Eigen::Vector3d::Zero()}, nu_rate_{Eigen::Vector3d::Zero()};
  std::array<double, kMaxDockingFaces> lateral_rate_{};
  std::array<double, kMaxDockingSpeedFaces> speed_rate_{};
  double timing_rate_{0.0}, axial_lo_rate_{0.0}, axial_hi_rate_{0.0};

  // Per-solve inputs.
  std::array<BallNodeSample, kMaxMpcNodes + 1> ball_{};
  Eigen::Matrix3d sigma_p_{Eigen::Matrix3d::Zero()};
  DockingCovariance6 sigma_b_{DockingCovariance6::Zero()};
  Eigen::Vector3d p_line_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d p_perp_{Eigen::Matrix3d::Identity()};

  // Evaluation workspace (values always; gradients when with_jacobians).
  DockingFrameKinematics kin_;
  std::array<DockingRelativeState, kMaxMpcNodes + 1> rel_;       // nodes 0..k_c
  Eigen::MatrixXd tau_;                                          // n × (N+1)
  Eigen::MatrixXd d_tau_;                                        // n × 3n(N+1)
  std::array<DockingScalar, kMaxMpcNodes> corridor_, envelope_;  // per approach index
  std::array<double, kMaxMpcNodes> corridor_ds_{};
  std::array<double, kMaxMpcNodes> slack_c_{}, slack_v_{};          // minimal, last evaluation
  std::array<double, kMaxMpcNodes> slack_c_cur_{}, slack_v_cur_{};  // … at the iterate
  std::array<DockingScalar, kMaxDockingFaces> lateral_;
  std::array<DockingScalar, kMaxDockingSpeedFaces> speed_;
  DockingScalar timing_, axial_lo_, axial_hi_;
  DockingImpact impact_;
  DockingImpactWork impact_work_;
  DockingManipulabilityWork manip_work_;
  Eigen::MatrixXd manip_grad_;  // n × (N+1), columns 0..k_c−1 used
  Eigen::MatrixXd perp_jac_;    // 3 × n(N+1): P_⊥ J_p per stop node
  Eigen::MatrixXd perp_res_;    // 3 × (N+1)
  Evaluation ev_cur_;

  // Assembly workspace.
  Eigen::MatrixXd t_node_jac_;  // n × nw: one node's torque Jacobian in [d | θ]
  Eigen::MatrixXd wt_;          // n × nw
  Eigen::VectorXd row_;         // nw: the row MapRow built
  Eigen::Index row_len_{0};     // its non-zero prefix
  Eigen::VectorXd aq_, av_;     // n
  Eigen::VectorXd work_n_, zero_n_, ones_n_;
  Eigen::MatrixXd j6_;
};

}  // namespace rtc::catching
