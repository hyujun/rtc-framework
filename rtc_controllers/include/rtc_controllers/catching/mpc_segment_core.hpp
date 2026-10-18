// ── MPC segment core: jerk-input condensed QP, APPROACH to the stop ─────────────
// (dynamic_catching MPC plan E1-F01 / #627 — the stop segment; E1-F07 / #660 —
// the pre-catch grid and the catch terms; formulation §1.1–§1.3, §1.6)
//
// The segment is not only the stop (MD-48): with n_pre > 0 the horizon
// starts before the catch. With the defaults (n_pre = 0, catch_terms = false)
// this is exactly the E1-F01 stop problem described first below.
//
// Replaces nothing yet — the closed-form DECEL (decel_target.hpp) and the
// QP-independent joint stop stay the safety nets (MD-11). This
// is the numeric core the planner thread calls through MpcSegmentPlanner
// (mpc_segment_planner.hpp, E1-F03); it knows no ROS, no controller, no robot.
//
// ── Problem ───────────────────────────────────────────────────────────────────
// State x_k = (q_k, q̇_k, q̈_k) ∈ R^{3n} at nodes k = 0..N spaced Δ, input the
// constant jerk u_k on [k, k+1]. x_0 is fixed (the stop starts there), so every
// state and torque row lives on k = 1..N. Condensing: x_k = Φ_k x_0 + Γ_k u,
// with move blocking u = E ũ (a block pattern {b_1..b_B}, Σb = N, shared by
// all joints) and a variable scale ũ = u_scale·z. With z ordered block-major
// (z_{b·n+j}) the triple integrator decouples per joint and every stage
// matrix is a Kronecker product: G_{m,k} = ĝ_{m,k}ᵀ ⊗ I_n for m ∈ {q, q̇, q̈},
// ĝ ∈ R^B. The solver works on z = [ũ/u_scale (nB); s (N)].
// Per joint group (E3-F02, params.n_groups): each group may have its own
// pattern. Joints whose groups share a pattern form one block class with the
// structure above; z holds the classes one after another (BlockLayout), and a
// term couples the classes through their own gains. One class — no groups, or
// groups of one pattern — is the problem above to the bit.
//
//   min  ½ Σ_k Σ_j R_j (u_kj/u_scale)²                        jerk
//      + ½ w_Δ Σ_k ‖q_k − q̄_k‖²                               stay near x̄
//      + ½ w_⊥ Σ_k ‖P_⊥(p_C(q̄_k) + J_C(q̄_k)(q_k − q̄_k) − p_c)‖²   optional
//      + ½ w_path Σ_k ‖P_k(p_C(q̄_k) + J_C(q̄_k)(q_k − q̄_k) − p_k)‖²  optional
//      + ½ Σ_k Σ_j w_vel,j q̇_kj²                              optional (E3-F02)
//      + ½ Σ_k Σ_j w_rest,j (q_kj − q_rest,j)²                 optional (E3-F02)
//      + ½ Σ_k W(k)/s² ‖A_k q̈_k + d_k‖²                      optional (E3-F02)
//      + ρ_τ Σ_k s_k                                          torque slack, per node
//   s.t. q̇_N = q̈_N = 0                                         (2n, hard)
//        max(q_min+m, q̄_k−δ) ≤ q_k ≤ min(q_max−m, q̄_k+δ)       (hard)
//        |q̇_k| ≤ η_v q̇_max                                     (hard)
//        |τ^lin_kj / τ_max,j| ≤ η_τ + s_k,  s_k ≥ 0  (every j; soft)
//
// τ^lin_k = τ̄_k + D_k (x_k − x̄_k), D_k = [∂τ/∂q  ∂τ/∂q̇  M] at the reference
// node x̄_k (pinocchio computeRNEADerivatives on the caller's hand-locked arm
// model with armature added). Torque rows and slack are scaled by 1/τ_max, so
// the slack is a DIMENSIONLESS fraction of τ_max — unlike the CLIK torque rows
// (unit-norm scaling), chosen here so the slack reads directly as "how far past
// the limit". There is ONE slack per node (#857): the node's joints share
// s_k, so where its rows bind s_k = max_j(|τ^lin_kj|/τ_max,j − η_τ)⁺ — the
// node's worst excess, an ℓ∞ penalty within the node (one slack per joint
// would pay the sum), and a joint below the worst may use η_τ + s_k. The
// penalty is exact (s = 0 whenever the hard problem is feasible) only while
// ρ_τ exceeds, on every node, the SUM of that node's torque-row multipliers
// Σ_j (λ⁺_kj + λ⁻_kj) — not each multiplier alone.
//
// ── Per-joint velocity and rest terms (E3-F02, #641; formulation §1.3) ────────
// w_vel and w_rest are weights per joint (the planner spreads a joint group's
// value over its joints), on every node k = 1..N, per node like w_Δ — no
// interval-length weight. Both are quadratic in z with a CONSTANT Hessian:
// that part is built at Init and enters H on every path, the w_delta_scale
// one (H = h_kin + h_wb + scale·h_delta) included; the pre-solve (a kinematic
// stop problem that only builds a reference) has neither. q_rest is read only
// where w_rest > 0. Off (empty, or all zero) the QP is the one without the
// term, bit for bit; the result's `whole_body` reports the terms' values at
// the solution.
//
// The centroidal angular-momentum rate (formulation §1.3, §1.4) is linearised
// at the reference like the torque rows: k̇_G,k ≈ A_k q̈_k + d_k with
// A_k = A^ω_G(q̄_k) (the angular rows of the centroidal momentum matrix) and
// the constant d_k = Ȧ^ω_G(q̄_k, q̄̇_k) q̄̇_k, both from one pinocchio dccrba at
// x̄_k on the core's own model. It reads EVERY joint of the model. Its weight
// is W(k)/s²: W = w_ang_pre on the nodes before the catch node, w_ang_post
// from it on (all of 1..N without a pre-catch grid), s = ang_scale [N·m] —
// per node, ½-weighted, like the terms above. The q / q̇ dependence of A_k
// beyond the reference (the first-order centroidal derivatives) is not
// modelled. The result's `whole_body` reports the term and max ‖k̇_G,k‖ over
// the nodes it weights, on this LINEAR model at the solution (no evaluation
// after the QP).
//
// ── The pre-catch grid and the catch terms (E1-F07) ───────────────────────────
// With n_pre > 0 the horizon has n_pre nodes of spacing Δ_a (dt_pre) BEFORE the
// catch node k_c = n_pre and n_nodes nodes of spacing Δ_s (dt) after it; the
// total is N = n_pre + n_nodes and everything above holds with that N (the
// terminal equality still sits at node N, the end of the stop). The grid is
// anchored at the catch instant t_c: node k_c is AT t_c. Differences:
//  • jerk cost is weighted by the interval length, ½ Σ_k (Δ_k/Δ_s) R (u_k/u_s)²,
//    so a mixed grid approximates the same time integral (formulation §1.2).
//    w_Δ and ρ_τ stay PER NODE, as the formulation writes them — on a mixed
//    grid the same value therefore regularises the coarse part less per second.
//  • a block may not cross the catch node, and at least 3 blocks must lie after
//    it (the terminal equality takes two; formulation §1.1).
//  • the stop-path term w_⊥ covers the stop segment only, nodes k_c..N: the
//    line through p_c is where the hand stops, not how it approaches.
//  • the closing-window path term w_path (#716, formulation §1.6) covers the
//    pre-catch nodes the caller marks (params.path_nodes, 1..n_pre − 1 only:
//    node 0 is x_0, and from k_c on w_⊥ holds the hand on the catch line),
//    each against ITS OWN line (p_k, d̂_k; P_k = I − d̂_k d̂_kᵀ) — the caller's
//    choice is the ball path near where the reference puts the hand at that
//    node. Same form as w_⊥, same storage (the two node sets are disjoint).
//    A marked node the input switches off (path_on) carries no term on that
//    solve. With w_path = 0 no path input is read and the QP is the one
//    without the term, bit for bit.
// With catch_terms the cost gains three terms AT the catch node, linearised at
// the reference x̄ (the same ½-weighted least-squares convention as above —
// formulation §1.3 puts them inside the same ½[…]):
//      ½ ‖p_C(q_kc) − p̂_b‖²_{W_p}                        position
//    + ½ w_a ‖e_a(q_kc)‖²                                 approach axis
//    + ½ ‖v_C − γ_ref v̂_b‖²_{W_v},  W_v = w_∥ d̂d̂ᵀ + w_⊥(I − d̂d̂ᵀ)   velocity
//    + ρ_v s_v,   |v̂_b − v_C|_i ≤ v_allow (1 + s_v),  s_v ≥ 0   (ρ_v > 0 only)
// with v_C = J_v(q̄) q̇_kc + H_v(q̄, q̄̇)(q_kc − q̄), H_v = ∂_q[J_v(q) q̄̇], d̂ the
// ball's direction of travel, and e_a rtc_math's axis-alignment error of the
// catch frame's +z against a_d (roll about the axis is free). The slack s_v is
// DIMENSIONLESS (a fraction of v_allow, like the torque slack) and its rows are
// always against v̂_b itself, not γ_ref v̂_b — with γ_ref < 1 the cost's own
// optimum sits (1 − γ_ref)‖v̂_b‖ off those rows, so s_v > 0 is then structural.
// The catch terms need a reference: the kinematic pre-solve below is a STOP
// problem and knows no catch point, so Solve() rejects reference_valid = false
// (kReferenceRequired) instead of linearising a reach around "stay where you
// are". Building the first reference (wait pose → the plan's posture at t_c)
// is the caller's job. It need not be inside the velocity box: the box rows
// and the trust region bend a reference that runs over it back (#819), as long
// as its area over the box stays within 2·delta_tr of reach.
// Limits are checked at the nodes only; between nodes of the coarse part the
// velocity is a quadratic and may overshoot the box by a little.
//
// ── Semantics a caller must know ──────────────────────────────────────────────
//  • The horizon IS the stopping time. The cost has no time term, so the
//    optimum always uses all of N·Δ: a longer horizon means a gentler but
//    longer stop, never an earlier one. Choosing N and Δ is the caller's
//    decision (`planner.segment.mpc.horizon`, MD-24), not this core's.
//  • Blocks: B ≥ 3 (after the catch node, when n_pre > 0 — the Init rank
//    self-test cannot see that rule, pre-catch blocks supply rank too; the
//    explicit count is the guard). The terminal equality has rank 2n for any B ≥ 2; B = 2
//    leaves no freedom (the terminal rows fix every block), so B ≥ 3 is the
//    freedom rule, rejected as kBlocksTooFew. The rank itself is re-checked on
//    the assembled matrix at Init as a self-test of the condensing.
//  • Terminal static torque: when the reference ends at rest, D_N reduces to
//    [∂g/∂q 0 M] and the terminal equality kills the M term, so node N's torque
//    rows ARE the static-torque check at the stop posture. Solve() therefore
//    requires a supplied reference to end at rest (kReferenceNotAtRest).
//  • No reference (reference_valid = false, e.g. the first cycle): a kinematic
//    pre-solve (no torque rows, no trust region, no w_Δ/w_⊥) produces x̄, then
//    the full problem is solved around it — two QPs that cycle only. The
//    pre-solve always starts cold (a new stop is a new problem); every other
//    QP warm-starts from the previous one.
//  • Entry state: x_0 outside the core's own box (q_0 within the margin m of a
//    limit, or |q̇_0| > η_v q̇_max) is rejected (kInitialStateOutsideBox), not
//    relaxed — node 1 could not satisfy the hard rows. Keeping x_0 inside is
//    the caller's policy (predict before entry, or fall back to closed form).
//  • x_0 more than δ from x̄_0 (it drifted between cycles), or a trust-region
//    row with lower > upper (x̄ left the box by more than δ), fails closed
//    (kTrustRegionConflict) before any QP runs; the caller re-solves with
//    reference_valid = false. "Cannot stop within limits" shows up as slack:
//    read slack_max / slack_terminal_max, the publish threshold is the
//    caller's (MD-11).
//
// ── Contracts ─────────────────────────────────────────────────────────────────
//  • Init() is non-RT (copies the model, allocates everything). Solve() is
//    noexcept and fail-closed — on ANY failure the result's trajectory,
//    slack, torque ratios and catch-node fields keep their previous values;
//    only reason, valid, qp_status, the iteration counts, presolved,
//    cold_retried and the timings change.
//  • A warm main QP that fails is solved once more from zero
//    (result.cold_retried): from iterates left by another problem ProxQP may
//    not converge within solver.max_iter on a feasible QP (and, under an
//    approximate infeasibility test, called such QPs infeasible — #880). A
//    QP that is really infeasible fails both runs, so it costs two. cold_start
//    on a new problem avoids paying for the failed run.
//  • Heap: this core allocates nothing in Solve(). Everything up to the QP
//    (linearisation, FK, condensing) is pinned by a C-level malloc gate; the
//    extraction after the QP shares its Solve with ProxQP, whose C mallocs
//    cannot be told apart from it, so it is pinned by the operator-new gate
//    only. ProxQP does allocate:
//    its public update() copies the vector arguments and solve() allocates a
//    few times per call (~7–18 C mallocs per Solve at n = 7). That is a
//    recorded RT-1 violation shared with every QPSolverWrapper user
//    (agent_docs/invariants.md §RT Path, #654; MD-22 / MD-23) — not a property
//    of this file.
//  • The model is the arm in pinocchio velocity order (nq == nv, revolute /
//    prismatic joints only). Mapping device order to that order is the
//    caller's job (the same rule CatchPoseIk documents).
//  • Result matrices are sized by ResizeResult() (non-RT); Solve() rejects a
//    wrongly sized result instead of resizing it.
#pragma once

#include "rtc_controllers/catching/mpc_block_grid.hpp"  // BlockLayout, BlockPattern
#include "rtc_controllers/catching/trajectory.hpp"      // kMaxSegmentNodes, kMaxPlanNv
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>

namespace rtc::catching {

/// Node capacity of the core: N = n_pre + n_nodes ≤ kMaxMpcNodes. Separate from
/// the payload's kMaxSegmentNodes (trajectory.hpp), which still bounds the stop
/// segment (n_nodes) and the number of blocks — what a published segment may
/// carry is the planner's decision, not this core's.
inline constexpr int kMaxMpcNodes = 32;

enum class MpcSegmentCoreReason : std::uint8_t {
  kNone = 0,
  kNotInitialized,
  kParamsInvalid,
  kBlocksTooFew,
  kLimitsInvalid,
  kModelUnsupported,
  kFrameUnknown,
  kTerminalRankDeficient,
  kDimMismatch,
  kNonFinite,
  kDirectionNotUnit,
  kInitialStateOutsideBox,
  kReferenceNotAtRest,
  kTrustRegionConflict,
  kPresolveFailed,
  kQpFailed,
  kSolutionNonFinite,
  // E1-F07 — appended, the values above are stable.
  kBlocksAcrossCatch,    ///< Init: a block spans the catch node
  kInputOutOfRange,      ///< W_p not symmetric PSD, γ_ref ∉ (0, 1], w_delta_scale ∉ [0, 1]
  kReferenceRequired,    ///< catch terms on and reference_valid = false
  kCatchAxisOutOfRange,  ///< the reference's axis error exceeds axis_theta_max
};

/// @brief Stable name for logs and test messages.
[[nodiscard]] const char* MpcSegmentCoreReasonName(MpcSegmentCoreReason reason) noexcept;

/// Tuning of the stop problem. Every field is validated by Init().
struct MpcSegmentCoreParams {
  int n_nodes{12};     ///< stop-segment nodes N_s (≤ kMaxSegmentNodes); N = n_pre + n_nodes
  double dt{0.05};     ///< Δ_s [s]; n_nodes·Δ_s is the stopping time (header note)
  int n_pre{0};        ///< pre-catch nodes (0 = the stop segment only); N ≤ kMaxMpcNodes
  double dt_pre{0.0};  ///< Δ_a [s], > 0 when n_pre > 0
  int n_blocks{6};     ///< B (3 ≤ B ≤ min(N, kMaxSegmentNodes))
  std::array<int, kMaxSegmentNodes> block_sizes{1, 1, 2, 2, 3, 3};  ///< first B used, Σ = N
  /// Per-group move blocking (E3-F02, formulation §1.1, MD-49). With n_groups
  /// ≤ 1 every joint follows n_blocks / block_sizes above. With 2..
  /// kMaxJointGroups, model joint j follows group_blocks[joint_group[j]] and
  /// n_blocks / block_sizes are not read; each partition a joint uses obeys
  /// the same rules (none across the catch node, three after it). Groups with
  /// the same partition share their variables' layout (BlockLayout) — so
  /// groups that all have one partition are the problem without groups, to
  /// the bit.
  int n_groups{0};
  std::array<std::int8_t, kMaxPlanNv> joint_group{};
  std::array<BlockPattern, kMaxJointGroups> group_blocks{};
  Eigen::VectorXd jerk_weight;  ///< R_j > 0 (n); empty = all 1
  /// Jerk scale [rad/s³]. NOT a pure preconditioner: the jerk cost is
  /// (u/u_scale)², so changing it re-weights jerk against w_Δ, w_⊥ and ρ_τ.
  double u_scale{1e3};
  double w_delta{1.0};  ///< pull toward the reference x̄ [1/rad²]
  /// Catch-frame off-line error weight [1/m²]; 0 = off. Large values raise
  /// H's condition number (more ProxQP iterations): the planner's parser
  /// bounds it where its cores still solve within solver.max_iter
  /// (kMpcSegmentStopPathWeightMax).
  double w_perp{0.0};
  /// Closing-window path weight [1/m²]; 0 = off (header note, #716). Bounded
  /// like w_perp.
  double w_path{0.0};
  /// The nodes the path term may sit on, indexed by node k. Only
  /// 1..n_pre − 1 may be set (Init: kParamsInvalid otherwise); read only with
  /// w_path > 0.
  std::array<bool, kMaxMpcNodes + 1> path_nodes{};
  /// Catch terms at node k_c = n_pre (header note). Needs n_pre ≥ 1; false
  /// ignores every catch field below and every catch input.
  bool catch_terms{false};
  double w_axis{0.0};    ///< approach-axis weight [1/rad²]; 0 = off
  double w_v_par{0.0};   ///< relative-velocity weight along the ball's travel [(m/s)⁻²]
  double w_v_perp{0.0};  ///< … and across it; both 0 = velocity term off
  /// Relative-velocity slack penalty; 0 = no slack variable and no rows (the
  /// QP keeps its dimensions). Like rho_tau: exact only above the rows'
  /// multipliers, and large values slow ProxQP.
  double rho_v{0.0};
  double v_rel_allow{0.0};  ///< per-axis |v̂_b − v_C| the hand absorbs [m/s], > 0 when rho_v > 0
  /// Largest axis error of the REFERENCE the axis term is linearised at [rad],
  /// in (0, π). Beyond it ‖J_a‖ grows like θ/sinθ and the linearisation radius
  /// falls below the trust region; Solve() fails closed instead.
  double axis_theta_max{1.5707963267948966};
  /// Torque slack penalty (per node); 0 = torque rows off. Exact only above
  /// every node's sum of torque-row multipliers (header note); larger values
  /// slow ProxQP (and, under an approximate infeasibility test, drew false
  /// infeasibility verdicts — solver.eps_primal_inf below). The E1-F01 sweep
  /// (#627) found 10 exact with one slack per joint and node; #857 records it
  /// for one per node.
  double rho_tau{10.0};
  double eta_v{0.95};    ///< velocity row fraction of q̇_max, (0, 1]
  double eta_tau{0.7};   ///< torque row fraction of τ_max, (0, 1]
  double m_q{0.05};      ///< position margin inside the limits [rad]
  double delta_tr{0.1};  ///< trust region half-width around q̄ [rad] (> 0, may be +inf)
  /// |q̇̄_N|, |q̈̄_N| bound for a supplied reference. Must exceed
  /// solver.eps_abs: a shifted previous solution is at rest only to eps_abs.
  double reference_rest_tol{1e-4};
  /// Re-equilibrated every solve; PrimalDualLDLT backend (see QPSolverConfig).
  /// eps_primal_inf 0: only an exact infeasibility certificate ends a solve
  /// early. ProxQP's approximate test (its default 1e-4) called feasible QPs
  /// of this core infeasible — every first solve it failed in the offline
  /// grid maps (5,452 of about 262,000, #880) and a vertical 7-joint arm's
  /// stop from rest at q = 0 (#857 D12) solve without it. An infeasible QP
  /// then runs to max_iter: 60 outer steps keep every converging first solve
  /// of those maps (the hardest takes 31 – 40; 30 cuts one) and hold an
  /// infeasible solve to about 0.1 s cold on a 7-joint arm (#880 D11).
  /// Designated so a field added to QPSolverConfig cannot shift these.
  tsid::QPSolverConfig solver{.max_iter = 60,
                              .update_preconditioner = true,
                              .dense_backend = proxsuite::proxqp::DenseBackend::PrimalDualLDLT,
                              .eps_primal_inf = 0.0};
  /// Joint velocity weight w_vel,j [(rad/s)⁻²] per joint (header note); empty
  /// or all zero = off. Each entry in [0, kWholeBodyWeightMax].
  Eigen::VectorXd w_vel;
  /// Rest weight w_rest,j [rad⁻²] per joint; empty or all zero = off. Each
  /// entry in [0, kWholeBodyWeightMax].
  Eigen::VectorXd w_rest;
  /// Rest posture [rad] (n), read where w_rest > 0 (finite there).
  Eigen::VectorXd q_rest;
  /// Angular-momentum rate weights W⁻ (nodes before the catch node) and W⁺
  /// (from it on), each in [0, kWholeBodyWeightMax]; both 0 = off.
  double w_ang_pre{0.0};
  double w_ang_post{0.0};
  /// s_k̇ [N·m], > 0 (read when the term is on): the rate the weight is
  /// measured against.
  double ang_scale{1.0};
  /// Test / benchmark only: rebuild every QP matrix densely each Solve
  /// (Γ_k products instead of the Kronecker form, nothing cached). Same
  /// problem, same answer to rounding — the oracle for the structured path
  /// and the "before" of its timing.
  bool reference_assembly{false};
};

/// Joint limits in pinocchio velocity order (n each).
struct MpcSegmentCoreLimits {
  Eigen::VectorXd q_min, q_max;  ///< [rad]; q_min ≤ q_max (equal allowed — a locked joint)
  Eigen::VectorXd qd_max;        ///< > 0 [rad/s]
  Eigen::VectorXd tau_max;       ///< > 0 [N·m]
  Eigen::VectorXd armature;      ///< ≥ 0 [kg·m²], ADDED to whatever armature the model carries
};

struct MpcSegmentCoreInput {
  Eigen::VectorXd q0, qd0, qdd0;  ///< x_0 (n each)
  /// Linearisation reference x̄ (n × (N+1)); read only when reference_valid.
  Eigen::MatrixXd q_ref, qd_ref, qdd_ref;
  bool reference_valid{false};
  Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};     ///< catch point [m]; read when w_⊥ > 0
  Eigen::Vector3d d_hat{Eigen::Vector3d::UnitX()};  ///< unit line direction; read when w_⊥ > 0
  /// Pull toward the reference: w_Δ is multiplied by this, in [0, 1] (the
  /// covariance-proportional schedule of formulation §1.3). Pass 0 on the first
  /// solve of a plan — the reference is then the caller's curve, not a previous
  /// solution worth staying near.
  double w_delta_scale{1.0};
  /// Start the main QP from x = y = z = 0. A supplied reference does not reset
  /// the warm start by itself; set this when the problem is a NEW one (a new
  /// plan, another core's grid point), where the last problem's iterates
  /// mislead ProxQP (it may not converge from them within solver.max_iter).
  /// Left false there, the solve still succeeds — a failed warm run is
  /// retried cold (result.cold_retried) — but pays for both runs.
  bool cold_start{false};
  // ── Catch inputs: read (and validated) only with params.catch_terms ────────
  Eigen::Vector3d p_b{
      Eigen::Vector3d::Zero()};  ///< predicted ball position at t_c [m], model world
  /// Position weight [1/m²], symmetric PSD (CatchPositionWeight builds it from
  /// a covariance). Zero = position term off.
  Eigen::Matrix3d w_p{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d a_d{Eigen::Vector3d::UnitZ()};  ///< desired approach axis (unit, world)
  Eigen::Vector3d v_b{Eigen::Vector3d::Zero()};   ///< predicted ball velocity at t_c [m/s]
  double gamma_ref{1.0};  ///< velocity cost target γ_ref·v̂_b, in (0, 1] (MD-53)
  // ── Closing-window path: read (and validated) only with params.w_path > 0,
  //    and only at a node params.path_nodes marks AND path_on turns on ──
  std::array<bool, kMaxMpcNodes + 1> path_on{};
  /// Column k: a point of node k's line [m] and its unit direction, model world.
  Eigen::Matrix<double, 3, kMaxMpcNodes + 1> path_p{
      Eigen::Matrix<double, 3, kMaxMpcNodes + 1>::Zero()};
  Eigen::Matrix<double, 3, kMaxMpcNodes + 1> path_d{
      Eigen::Matrix<double, 3, kMaxMpcNodes + 1>::Zero()};
};

/// The whole-body terms at the returned solution (E3-F02), each with the
/// cost's ½: their total and their sum over each joint group (index =
/// params.joint_group; group 0 holds every joint without groups).
struct MpcSegmentWholeBodyCost {
  double vel{0.0};
  double rest{0.0};
  std::array<double, kMaxJointGroups> vel_group{};
  std::array<double, kMaxJointGroups> rest_group{};
  double ang{0.0};           ///< ½ Σ_k W(k)/s² ‖k̇_G,k‖² (linear model)
  double ang_rate_max{0.0};  ///< max ‖k̇_G,k‖ over the weighted nodes [N·m] (linear model)
};

struct MpcSegmentCoreResult {
  Eigen::MatrixXd q, qd, qdd;  ///< nodes, n × (N+1)
  Eigen::MatrixXd u;           ///< jerk, n × N [rad/s³]
  Eigen::VectorXd slack;       ///< torque slack s_k, N (entry k−1 = node k), fraction of τ_max
  /// Linearised torque ratio τ^lin/τ_max (signed), n × N, column k−1 = node k.
  /// Only written when torque_evaluated.
  Eigen::MatrixXd tau_ratio;
  double slack_max{0.0};
  double slack_terminal_max{0.0};  ///< node N — the static-torque check at the stop posture
  double tau_ratio_max{0.0};       ///< max |τ^lin/τ_max| over nodes 1..N (linearised)
  bool torque_evaluated{false};    ///< false when ρ_τ = 0 (rows off, tau_ratio_max not computed)
  bool presolved{false};           ///< this Solve ran the kinematic pre-solve
  bool valid{false};
  MpcSegmentCoreReason reason{MpcSegmentCoreReason::kNotInitialized};
  int qp_status{-1};  ///< proxsuite QPSolverOutput of the last QP (0 = solved)
  int iterations{0};  ///< main QP, its last run (pre-solve iterations in presolve_iterations)
  /// The warm main QP failed and was solved again from zero (header note);
  /// solve_us covers both runs.
  bool cold_retried{false};
  int presolve_iterations{0};
  // ── Catch node, re-evaluated by FK at the SOLUTION (not the linear model) ──
  bool catch_evaluated{false};  ///< params.catch_terms; the fields below are then set
  Eigen::Vector3d catch_pos_err{Eigen::Vector3d::Zero()};  ///< p_C(q_kc) − p̂_b [m]
  double catch_axis_err{0.0};  ///< angle between the catch frame's +z and a_d [rad]
  Eigen::Vector3d catch_v_rel{Eigen::Vector3d::Zero()};  ///< v̂_b − J_v(q_kc) q̇_kc [m/s]
  double catch_gamma{0.0};  ///< v̂_bᵀ v_C / ‖v̂_b‖² (0 when the ball is at rest)
  double slack_v{0.0};      ///< s_v, fraction of v_rel_allow (linear model)
  /// Written by every valid Solve (zeros for the terms that are off).
  MpcSegmentWholeBodyCost whole_body;
  double presolve_us{0.0};
  double linearize_us{0.0};
  double condense_us{0.0};
  double solve_us{0.0};
};

class MpcSegmentCore {
 public:
  MpcSegmentCore() = default;

  /// @brief Build the problem for one arm (non-RT: copies the model, allocates).
  /// @param arm hand-locked arm model, pinocchio velocity order
  /// @param catch_frame frame for the w_⊥ term and the catch terms (validated
  ///        even when both are off); its +z is the approach axis
  /// @return kNone on success; the core stays uninitialised otherwise.
  [[nodiscard]] MpcSegmentCoreReason Init(const pinocchio::Model& arm,
                                          pinocchio::FrameIndex catch_frame,
                                          const MpcSegmentCoreParams& params,
                                          const MpcSegmentCoreLimits& limits);

  /// @brief Size a result for this problem (non-RT).
  void ResizeResult(MpcSegmentCoreResult& result) const;

  /// @brief Solve the stop problem from input.q0/qd0/qdd0 (RT-safe).
  /// @return result.valid. On failure the trajectory and slack are untouched.
  [[nodiscard]] bool Solve(const MpcSegmentCoreInput& in, MpcSegmentCoreResult& out) noexcept;

  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

  [[nodiscard]] int Nv() const noexcept { return n_; }

  /// The path term sits on node k on this solve (the last one, a failed one
  /// included: a solve that stopped before reading its inputs has none).
  [[nodiscard]] bool PathActive(int k) const noexcept {
    return k >= 0 && k <= kMaxMpcNodes && path_active_[static_cast<std::size_t>(k)];
  }

  /// The velocity / rest term is on (a positive weight on some joint).
  [[nodiscard]] bool VelocityTermActive() const noexcept { return vel_wb_on_; }

  [[nodiscard]] bool RestTermActive() const noexcept { return rest_on_; }

  [[nodiscard]] bool AngularMomentumTermActive() const noexcept { return ang_on_; }

  /// N = n_pre + n_nodes: results hold N + 1 node columns.
  [[nodiscard]] int NumNodes() const noexcept { return n_nodes_; }

  /// The catch node k_c (= n_pre; 0 for the stop segment alone).
  [[nodiscard]] int CatchNode() const noexcept { return n_pre_; }

  /// Instant of node k relative to node 0 [s]; NaN outside [0, N].
  [[nodiscard]] double NodeTime(int k) const noexcept;

  /// The effective position box the entry state and every node are held to
  /// [rad], model order: the limits pulled in by min(m_q, half the range), so
  /// a narrow or locked joint never inverts it. A caller that projects a state
  /// or a reference into the box reads it here instead of recomputing it.
  [[nodiscard]] const Eigen::VectorXd& PositionLow() const noexcept { return q_lo_; }

  [[nodiscard]] const Eigen::VectorXd& PositionHigh() const noexcept { return q_hi_; }

  /// B of block class 0 — every joint's, unless groups with different
  /// partitions split them (Layout()).
  [[nodiscard]] int NumBlocks() const noexcept { return n_blocks_; }

  /// Which block class each joint is in and where its jerk sits in z.
  [[nodiscard]] const BlockLayout& Layout() const noexcept { return layout_; }

  /// Rank of the assembled terminal-equality matrix found at Init (2n).
  [[nodiscard]] int TerminalRank() const noexcept { return terminal_rank_; }

  /// The last assembled full / pre-solve QP (diagnostics and tests).
  [[nodiscard]] const tsid::QPData& MainQp() const noexcept { return qp_main_; }

  [[nodiscard]] const tsid::QPData& PresolveQp() const noexcept { return qp_pre_; }

  /// The QP solution z of the last converged QP (tests; meaningless before one). A
  /// Solve() that fails after its pre-solve converged leaves the pre-solve's z here.
  [[nodiscard]] const Eigen::VectorXd& LastSolution() const noexcept { return z_; }

  /// Per-node scaled stage gains ĝ_{m,k}[b] (m: 0 = q, 1 = q̇, 2 = q̈) of
  /// block class `block_class` (Layout()).
  [[nodiscard]] double StageGain(int m, int k, int b, int block_class = 0) const noexcept;

 private:
  void FreeResponse() noexcept;
  void AssembleConstant(tsid::QPData& qp, bool main) noexcept;
  void AssembleConstantDense(tsid::QPData& qp, bool main) noexcept;
  void AssembleTorqueSlackConstant(tsid::QPData& qp) const noexcept;
  void AssembleSlackVConstant(tsid::QPData& qp) const noexcept;
  [[nodiscard]] bool Linearize() noexcept;
  void AssembleTorqueRows() noexcept;
  void AssembleTorqueRowsDense() noexcept;
  void AssemblePerp() noexcept;
  void AssemblePath() noexcept;
  [[nodiscard]] MpcSegmentCoreReason LinearizeCatch() noexcept;
  void AssembleCatch() noexcept;
  void AccumulateNodeTerm(Eigen::Index k, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a_q,
                          const Eigen::Matrix<double, 3, Eigen::Dynamic>* a_v,
                          const Eigen::Matrix3d& w, const Eigen::Vector3d& r) noexcept;
  void AccumulateNodeTermClasses(Eigen::Index k, bool with_v) noexcept;
  void AccumulateNodeTermDense(Eigen::Index k, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a_q,
                               const Eigen::Matrix<double, 3, Eigen::Dynamic>* a_v,
                               const Eigen::Matrix3d& w, const Eigen::Vector3d& r) noexcept;
  void AssembleSlackVRows() noexcept;
  [[nodiscard]] double AngularWeight(Eigen::Index k) const noexcept;
  void AssembleAngularMomentum() noexcept;
  void AccumulateAccelNodeTerm(Eigen::Index k, double w, const Eigen::Vector3d& r) noexcept;
  void EvaluateWholeBody(MpcSegmentCoreResult& out) const noexcept;
  void EvaluateCatch(MpcSegmentCoreResult& out) noexcept;
  [[nodiscard]] bool AssembleBounds(tsid::QPData& qp, bool main) noexcept;
  void AssembleGradient(tsid::QPData& qp, bool main) noexcept;
  [[nodiscard]] MpcSegmentCoreReason RunQp(tsid::QPData& qp, int& status, int& iterations) noexcept;
  void ResetSolver() noexcept;
  void TrajectoryFromZ() noexcept;

  [[nodiscard]] std::size_t ClassOf(Eigen::Index j) const noexcept {
    return static_cast<std::size_t>(layout_.class_of[static_cast<std::size_t>(j)]);
  }

  [[nodiscard]] Eigen::Index Col(Eigen::Index j, Eigen::Index b) const noexcept {
    return layout_.Col(static_cast<int>(j), static_cast<int>(b));
  }

  /// Column of the torque slack of node k0 + 1 (k0 = 0..N−1).
  [[nodiscard]] Eigen::Index SlackIdx(Eigen::Index k0) const noexcept { return nu_ + k0; }

  bool initialized_{false};
  MpcSegmentCoreParams params_;
  int n_{0};
  int n_nodes_{0};   // N = n_pre + stop nodes
  int n_pre_{0};     // k_c
  int n_blocks_{0};  // B of block class 0
  int nu_{0};        // Σ_c n_c·B_c (n·B with one class)
  int n_s_{0};       // torque slack variables: one per node, N (#857)
  int nz_{0};        // nu_ + n_s_ (+ 1 with the velocity slack)
  int n_eq_{0};      // 2n
  int n_in_{0};      // 4·n·N + n_s_ (+ 7 with the velocity slack)
  int sv_row_{0};    // first velocity-slack row, 4·n·N + n_s_
  int terminal_rank_{0};

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex frame_{0};

  Eigen::VectorXd q_lo_, q_hi_, v_hi_, inv_tau_;
  Eigen::VectorXd jerk_w_;

  // Stage structure per block class (BlockLayout): the joints of a class
  // share one block pattern, so its gains are scalars per (node, block). With
  // one class — no groups, or groups of one partition — this is the
  // structure every term was built on before groups (z_{b·n+j}), and each
  // assembly below keeps that code path for it.
  struct ClassStages {
    std::array<int, kMaxMpcNodes> block_of_node{};
    std::array<double, kMaxSegmentNodes> block_weight{};  // Σ_{k∈b} Δ_k/Δ_s
    // ĝ: (N+1) × B per derivative order, scaled by u_scale.
    Eigen::MatrixXd gq, gv, ga;
  };

  BlockLayout layout_;
  std::array<ClassStages, kMaxJointGroups> stage_;
  std::array<double, kMaxMpcNodes + 1> t_node_{};  // node instants from node 0 [s]
  std::array<double, kMaxMpcNodes> dt_node_{};     // Δ_k of interval [k, k+1]
  // Dense stage matrices [G_q; G_q̇; G_q̈] per node, stacked: 3n(N+1) × nB.
  Eigen::MatrixXd g_dense_;

  // Constant cost parts.
  Eigen::MatrixXd h_kin_;    // jerk only (pre-solve)
  Eigen::MatrixXd h_main_;   // jerk + w_Δ (full problem, before w_⊥ and the catch terms)
  Eigen::MatrixXd h_delta_;  // the w_Δ part alone at scale 1 (used when w_delta_scale ≠ 1)
  Eigen::MatrixXd h_wb_;     // the w_vel and w_rest parts (E3-F02); built only when one is on

  tsid::QPSolverWrapper solver_;
  tsid::QPData qp_pre_;
  tsid::QPData qp_main_;

  // Per-solve workspace (n × (N+1) unless noted).
  Eigen::VectorXd q0_, v0_, a0_;
  Eigen::MatrixXd qf_, vf_, af_;  // free response Φ_k x_0
  Eigen::MatrixXd qr_, vr_, ar_;  // reference x̄ in use
  Eigen::MatrixXd tau_bar_;       // τ̄_k
  Eigen::MatrixXd d_scaled_;      // n × 3n(N+1): rows scaled by 1/τ_max
  Eigen::VectorXd c_tilde_;       // n·N: scaled torque offset
  Eigen::MatrixXd j6_;            // 6 × n
  Eigen::Matrix3d p_perp_;
  Eigen::MatrixXd l_perp_;  // 3 × n
  Eigen::VectorXd r_perp_;  // 3
  Eigen::VectorXd work_n_, work_n2_, work_n3_, work_row_;
  Eigen::MatrixXd work_dg_;  // n × nB (dense assembly)
  Eigen::VectorXd z_;
  Eigen::MatrixXd q_out_, v_out_, a_out_;
  Eigen::VectorXd tau_lin_;  // n·N
  bool w_perp_on_{false};
  bool torque_on_{false};
  bool vel_wb_on_{false};  // w_vel > 0 on some joint
  bool rest_on_{false};    // w_rest > 0 on some joint
  bool ang_on_{false};     // w_ang_pre or w_ang_post > 0
  // A_k (3 × n per node, column block k) and d_k at the reference; node 0 unused.
  Eigen::Matrix<double, 3, Eigen::Dynamic> ag_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> dag_v_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> ag_k_;  // one node's A_k (workspace)
  Eigen::VectorXd w_vel_, w_rest_, q_rest_;  // n; zeros where off (q_rest_ 0 where w_rest = 0)
  // Closing-window path (#716): w_path > 0 and a node marked; this solve's
  // nodes (marked and switched on), whether any is, and their lines.
  bool path_on_{false};
  std::array<bool, kMaxMpcNodes + 1> path_active_{};
  bool path_solve_on_{false};
  Eigen::Matrix<double, 3, kMaxMpcNodes + 1> path_p_{
      Eigen::Matrix<double, 3, kMaxMpcNodes + 1>::Zero()};
  Eigen::Matrix<double, 3, kMaxMpcNodes + 1> path_d_{
      Eigen::Matrix<double, 3, kMaxMpcNodes + 1>::Zero()};
  Eigen::Vector3d p_c_{Eigen::Vector3d::Zero()};

  // Catch terms (E1-F07).
  bool catch_on_{false};
  bool vel_on_{false};       // w_v_par or w_v_perp > 0
  bool slack_v_on_{false};   // rho_v > 0: one more variable, seven more rows
  bool pos_on_{false};       // this solve's W_p ≠ 0
  bool h_modified_{false};   // qp_main_.H is not h_main_ (a term or a scale touched it)
  bool solver_warm_{false};  // the solver holds the iterates of a solved QP
  double w_delta_scale_{1.0};
  Eigen::Vector3d p_b_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a_d_{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d v_b_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d w_p_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d w_v_{Eigen::Matrix3d::Zero()};
  double gamma_ref_{1.0};
  // Linearisation at the catch node: residuals at z = 0 and their Jacobians.
  Eigen::Matrix<double, 3, Eigen::Dynamic> jv_c_, jw_c_, hv_c_, la_c_, dv_c_;  // 3 × n
  Eigen::Vector3d r_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_axis_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_c0_{Eigen::Vector3d::Zero()};  // v_C of the free response (z = 0)
  // AccumulateNodeTerm workspace.
  Eigen::Matrix<double, 3, Eigen::Dynamic> wa_q_, wa_v_;  // W·A
  Eigen::MatrixXd m_qq_, m_qv_, m_vv_, blk_;              // n × n
  Eigen::VectorXd c_q_, c_v_;                             // n
  Eigen::Matrix<double, 3, Eigen::Dynamic> l_dense_;      // 3 × nB (dense oracle)
  Eigen::Matrix<double, 3, Eigen::Dynamic> wl_dense_;     // 3 × nB
  Eigen::Matrix<double, 3, Eigen::Dynamic> perp_l_;       // 3 × n (one node's L_k)
};

/// @brief Position weight W_p = κ (Σ_p + σ_floor² I)⁻¹ from a predicted
///        position covariance, with its eigenvalues capped at w_max (RT-safe).
///
/// Σ_p is symmetrised first (an estimator's covariance is symmetric only to
/// rounding). An eigenvalue below zero — a covariance that is not PSD — counts
/// as zero, so the weight is never negative or infinite: each eigenvalue of W_p
/// is min(κ / (max(λ_i, 0) + σ_floor²), w_max). The result is exactly symmetric.
/// @param sigma_p     position covariance [m²]
/// @param kappa       dimensionless gain, > 0
/// @param sigma_floor tracking-error floor [m], > 0 (the weight's upper bound
///                    is κ/σ_floor² before w_max)
/// @param w_max       cap on the weight's eigenvalues [1/m²], > 0
/// @param[out] w_p    written only on success
/// @return false when an input is non-finite or a parameter is not positive.
[[nodiscard]] bool CatchPositionWeight(const Eigen::Matrix3d& sigma_p, double kappa,
                                       double sigma_floor, double w_max,
                                       Eigen::Matrix3d& w_p) noexcept;

}  // namespace rtc::catching
