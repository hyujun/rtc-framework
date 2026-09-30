// ── Decel MPC core: jerk-input condensed QP over the stop segment ─────────────
// (dynamic_catching MPC plan E1-F01 / #627; formulation §1.1–§1.3, §1.6)
//
// Replaces nothing yet — the closed-form DECEL (decel_target.hpp) and the
// QP-independent joint stop stay the safety nets (MPC_DUALARM_PLAN MD-11). This
// is the numeric core the planner thread will call (E1-F03); it knows no ROS,
// no controller, no robot.
//
// ── Problem ───────────────────────────────────────────────────────────────────
// State x_k = (q_k, q̇_k, q̈_k) ∈ R^{3n} at nodes k = 0..N spaced Δ, input the
// constant jerk u_k on [k, k+1]. x_0 is fixed (the stop starts there), so every
// state and torque row lives on k = 1..N. Condensing: x_k = Φ_k x_0 + Γ_k u,
// with move blocking u = E ũ (a block pattern {b_1..b_B}, Σb = N, shared by
// all joints) and a variable scale ũ = u_scale·z. With z ordered block-major
// (z_{b·n+j}) the triple integrator decouples per joint and every stage
// matrix is a Kronecker product: G_{m,k} = ĝ_{m,k}ᵀ ⊗ I_n for m ∈ {q, q̇, q̈},
// ĝ ∈ R^B. The solver works on z = [ũ/u_scale (nB); s (nN)].
//
//   min  ½ Σ_k Σ_j R_j (u_kj/u_scale)²                        jerk
//      + ½ w_Δ Σ_k ‖q_k − q̄_k‖²                               stay near x̄
//      + ½ w_⊥ Σ_k ‖P_⊥(p_C(q̄_k) + J_C(q̄_k)(q_k − q̄_k) − p_c)‖²   optional
//      + ρ_τ Σ s                                              torque slack
//   s.t. q̇_N = q̈_N = 0                                         (2n, hard)
//        max(q_min+m, q̄_k−δ) ≤ q_k ≤ min(q_max−m, q̄_k+δ)       (hard)
//        |q̇_k| ≤ η_v q̇_max                                     (hard)
//        |τ^lin_k / τ_max| ≤ η_τ + s_k,  s ≥ 0                  (soft)
//
// τ^lin_k = τ̄_k + D_k (x_k − x̄_k), D_k = [∂τ/∂q  ∂τ/∂q̇  M] at the reference
// node x̄_k (pinocchio computeRNEADerivatives on the caller's hand-locked arm
// model with armature added). Torque rows and slack are scaled by 1/τ_max, so
// the slack is a DIMENSIONLESS fraction of τ_max — unlike the CLIK torque rows
// (unit-norm scaling), chosen here so the slack reads directly as "how far past
// the limit". The penalty is exact (s = 0 whenever the hard problem is
// feasible) only while ρ_τ exceeds the torque rows' multipliers.
//
// ── Semantics a caller must know ──────────────────────────────────────────────
//  • The horizon IS the stopping time. The cost has no time term, so the
//    optimum always uses all of N·Δ: a longer horizon means a gentler but
//    longer stop, never an earlier one. Choosing N and Δ is the caller's
//    decision (E1-F03), not this core's.
//  • Blocks: B ≥ 3. The terminal equality has rank 2n for any B ≥ 2; B = 2
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
//    slack and torque ratios keep their previous values; only reason, valid,
//    qp_status, the iteration counts, presolved and the timings change.
//  • Heap: this core allocates nothing in Solve() (linearisation, FK,
//    condensing, extraction — pinned by a C-level malloc gate). ProxQP does:
//    its public update() copies the vector arguments and solve() allocates a
//    few times per call (~7–18 C mallocs per Solve at n = 7). That is a KNOWN
//    RT-1 gap shared with every QPSolverWrapper user, accepted for E1-F01 and
//    tracked as a follow-up issue — not a property of this file.
//  • The model is the arm in pinocchio velocity order (nq == nv, revolute /
//    prismatic joints only). Mapping device order to that order is the
//    caller's job (the same rule CatchPoseIk documents).
//  • Result matrices are sized by ResizeResult() (non-RT); Solve() rejects a
//    wrongly sized result instead of resizing it.
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>

namespace rtc::catching {

/// Node capacity of the stop segment. The E1-F02 plan payload carries the same
/// number of joint nodes and must use this constant, not its own.
inline constexpr int kMaxDecelNodes = 24;

enum class DecelMpcReason : std::uint8_t {
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
};

/// @brief Stable name for logs and test messages.
[[nodiscard]] const char* DecelMpcReasonName(DecelMpcReason reason) noexcept;

/// Tuning of the stop problem. Every field is validated by Init().
struct DecelMpcParams {
  int n_nodes{12};  ///< N (≤ kMaxDecelNodes)
  double dt{0.05};  ///< Δ [s]; N·Δ is the stopping time (header note)
  int n_blocks{6};  ///< B (3 ≤ B ≤ N)
  std::array<int, kMaxDecelNodes> block_sizes{1, 1, 2, 2, 3, 3};  ///< first B used, Σ = N
  Eigen::VectorXd jerk_weight;                                    ///< R_j > 0 (n); empty = all 1
  /// Jerk scale [rad/s³]. NOT a pure preconditioner: the jerk cost is
  /// (u/u_scale)², so changing it re-weights jerk against w_Δ, w_⊥ and ρ_τ.
  double u_scale{1e3};
  double w_delta{1.0};  ///< pull toward the reference x̄ [1/rad²]
  /// Catch-frame off-line error weight [1/m²]; 0 = off. Tested up to 1e4;
  /// large values raise H's condition number (more ProxQP iterations).
  double w_perp{0.0};
  /// Slack penalty; 0 = torque rows off. Exact only above the torque rows'
  /// multipliers; larger values slow ProxQP and trigger false infeasibility
  /// verdicts. 10 was exact on every sampled stop (E1-F01 sweep, #627).
  double rho_tau{10.0};
  double eta_v{0.95};               ///< velocity row fraction of q̇_max, (0, 1]
  double eta_tau{0.7};              ///< torque row fraction of τ_max, (0, 1]
  double m_q{0.05};                 ///< position margin inside the limits [rad]
  double delta_tr{0.1};             ///< trust region half-width around q̄ [rad] (> 0, may be +inf)
  double reference_rest_tol{1e-4};  ///< |q̇̄_N|, |q̈̄_N| bound for a supplied reference
  /// Re-equilibrated every solve; PrimalDualLDLT backend (see QPSolverConfig).
  tsid::QPSolverConfig solver{
      1e-6, 0.0, 200, 100, false, true, proxsuite::proxqp::DenseBackend::PrimalDualLDLT};
  /// Test / benchmark only: rebuild every QP matrix densely each Solve
  /// (Γ_k products instead of the Kronecker form, nothing cached). Same
  /// problem, same answer to rounding — the oracle for the structured path
  /// and the "before" of its timing.
  bool reference_assembly{false};
};

/// Joint limits in pinocchio velocity order (n each).
struct DecelMpcLimits {
  Eigen::VectorXd q_min, q_max;  ///< [rad]; q_min ≤ q_max (equal allowed — a locked joint)
  Eigen::VectorXd qd_max;        ///< > 0 [rad/s]
  Eigen::VectorXd tau_max;       ///< > 0 [N·m]
  Eigen::VectorXd armature;      ///< ≥ 0 [kg·m²], added to the model's diagonal inertia
};

struct DecelMpcInput {
  Eigen::VectorXd q0, qd0, qdd0;  ///< x_0 (n each)
  /// Linearisation reference x̄ (n × (N+1)); read only when reference_valid.
  Eigen::MatrixXd q_ref, qd_ref, qdd_ref;
  bool reference_valid{false};
  Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};     ///< catch point [m]; read when w_⊥ > 0
  Eigen::Vector3d d_hat{Eigen::Vector3d::UnitX()};  ///< unit line direction; read when w_⊥ > 0
};

struct DecelMpcResult {
  Eigen::MatrixXd q, qd, qdd;  ///< nodes, n × (N+1)
  Eigen::MatrixXd u;           ///< jerk, n × N [rad/s³]
  Eigen::MatrixXd slack;       ///< torque slack, n × N (column k−1 = node k), fraction of τ_max
  /// Linearised torque ratio τ^lin/τ_max (signed), n × N, column k−1 = node k.
  /// Only written when torque_evaluated.
  Eigen::MatrixXd tau_ratio;
  double slack_max{0.0};
  double slack_terminal_max{0.0};  ///< node N — the static-torque check at the stop posture
  double tau_ratio_max{0.0};       ///< max |τ^lin/τ_max| over nodes 1..N (linearised)
  bool torque_evaluated{false};    ///< false when ρ_τ = 0 (rows off, tau_ratio_max not computed)
  bool presolved{false};           ///< this Solve ran the kinematic pre-solve
  bool valid{false};
  DecelMpcReason reason{DecelMpcReason::kNotInitialized};
  int qp_status{-1};  ///< proxsuite QPSolverOutput of the last QP (0 = solved)
  int iterations{0};  ///< main QP (pre-solve iterations in presolve_iterations)
  int presolve_iterations{0};
  double presolve_us{0.0};
  double linearize_us{0.0};
  double condense_us{0.0};
  double solve_us{0.0};
};

class DecelMpc {
 public:
  DecelMpc() = default;

  /// @brief Build the problem for one arm (non-RT: copies the model, allocates).
  /// @param arm hand-locked arm model, pinocchio velocity order
  /// @param catch_frame frame for the w_⊥ term (validated even when w_⊥ = 0)
  /// @return kNone on success; the core stays uninitialised otherwise.
  [[nodiscard]] DecelMpcReason Init(const pinocchio::Model& arm, pinocchio::FrameIndex catch_frame,
                                    const DecelMpcParams& params, const DecelMpcLimits& limits);

  /// @brief Size a result for this problem (non-RT).
  void ResizeResult(DecelMpcResult& result) const;

  /// @brief Solve the stop problem from input.q0/qd0/qdd0 (RT-safe).
  /// @return result.valid. On failure the trajectory and slack are untouched.
  [[nodiscard]] bool Solve(const DecelMpcInput& in, DecelMpcResult& out) noexcept;

  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

  [[nodiscard]] int Nv() const noexcept { return n_; }

  [[nodiscard]] int NumNodes() const noexcept { return n_nodes_; }

  [[nodiscard]] int NumBlocks() const noexcept { return n_blocks_; }

  /// Rank of the assembled terminal-equality matrix found at Init (2n).
  [[nodiscard]] int TerminalRank() const noexcept { return terminal_rank_; }

  /// The last assembled full / pre-solve QP (diagnostics and tests).
  [[nodiscard]] const tsid::QPData& MainQp() const noexcept { return qp_main_; }

  [[nodiscard]] const tsid::QPData& PresolveQp() const noexcept { return qp_pre_; }

  /// Per-node scaled stage gains ĝ_{m,k}[b] (m: 0 = q, 1 = q̇, 2 = q̈).
  [[nodiscard]] double StageGain(int m, int k, int b) const noexcept;

 private:
  void FreeResponse() noexcept;
  void AssembleConstant(tsid::QPData& qp, bool main) noexcept;
  void AssembleConstantDense(tsid::QPData& qp, bool main) noexcept;
  [[nodiscard]] bool Linearize() noexcept;
  void AssembleTorqueRows() noexcept;
  void AssembleTorqueRowsDense() noexcept;
  void AssemblePerp() noexcept;
  [[nodiscard]] bool AssembleBounds(tsid::QPData& qp, bool main) noexcept;
  void AssembleGradient(tsid::QPData& qp, bool main) noexcept;
  [[nodiscard]] bool RunQp(tsid::QPData& qp, int& status, int& iterations) noexcept;
  void TrajectoryFromZ() noexcept;

  bool initialized_{false};
  DecelMpcParams params_;
  int n_{0};
  int n_nodes_{0};
  int n_blocks_{0};
  int nu_{0};    // n·B
  int nz_{0};    // n·(B + N)
  int n_eq_{0};  // 2n
  int n_in_{0};  // 5·n·N
  int terminal_rank_{0};

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex frame_{0};

  Eigen::VectorXd q_lo_, q_hi_, v_hi_, inv_tau_;
  Eigen::VectorXd jerk_w_;
  std::array<int, kMaxDecelNodes> block_of_node_{};

  // ĝ: (N+1) × B per derivative order, scaled by u_scale.
  Eigen::MatrixXd gq_, gv_, ga_;
  // Dense stage matrices [G_q; G_q̇; G_q̈] per node, stacked: 3n(N+1) × nB.
  Eigen::MatrixXd g_dense_;

  // Constant cost parts.
  Eigen::MatrixXd h_kin_;   // jerk only (pre-solve)
  Eigen::MatrixXd h_main_;  // jerk + w_Δ (full problem, before w_⊥)

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
  Eigen::MatrixXd ltl_;           // n × n
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
  Eigen::Vector3d p_c_{Eigen::Vector3d::Zero()};
};

}  // namespace rtc::catching
