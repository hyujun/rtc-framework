// ── mpc_docking: the ball in the capture frame, and every row built from it ────
// (dynamic_catching E1-F13, #739; reference
//  docs/dynamic_catching/ref/ball_catching_inverse_dynamics_mpc.md §5–§9, §17)
//
// One node's nonlinear outputs and their first derivatives with respect to the
// joint position q and velocity q̇, exposed as free functions so a test checks
// each derivative against a finite difference of the nonlinear output instead
// of only through a QP answer (the reason mpc_segment_core_torque.hpp exists).
// MpcDockingSegmentCore assembles its QP rows from exactly these.
//
// ── Frames ────────────────────────────────────────────────────────────────────
// The capture frame H is a frame of the arm model; R = R_WH, p_h its origin,
// e₃ its +z (the ball enters from +e₃ and travels toward −e₃), E⊥ = [e₁ e₂].
// Jacobians are LOCAL_WORLD_ALIGNED (world axes, origin at the frame):
// v_h = J_p q̇, ω_h = J_ω q̇, and a rotation perturbation is δR = [J_ω δq]× R.
// The ball (p_b, v_b) is a point of the MODEL world — it does not move with the
// hand, which is why the derivatives below are not those of a hand-fixed point.
//
//   r^W = p_b − p_h                       r^H = Rᵀ r^W
//   ν^H = ṙ^H = Rᵀ (v_b − v_h − ω_h × r^W)
//   s = e₃ᵀ r^H     ρ = E⊥ᵀ r^H     c = −e₃ᵀ ν^H   (closing speed, > 0 approaching)
//
//   ∂r^H/∂q  = −Rᵀ J_p + [r^H]× Rᵀ J_ω            (= ∂ν^H/∂q̇)
//   ∂ν^H/∂q  = [ν^H]× Rᵀ J_ω + Rᵀ(−∂_q v_h + [r^W]× ∂_q ω_h + [ω_h]× J_p)
//
// ∂_q v_h is pinocchio's POINT velocity derivative and ∂_q ω_h the angular rows
// of its frame velocity derivative, both LOCAL_WORLD_ALIGNED (the frame
// variant's linear rows lack the ω × v term a world-aligned point velocity
// carries; its angular rows do match — both pinned by the finite-difference
// suite).
//
// ── Uncertainty ───────────────────────────────────────────────────────────────
// Σ_p (3×3) and Σ_b (6×6 over [p; v]) are the ball's covariance at the node, in
// the model world. With the robot state held fixed:
//
//   crossing plane   Π = I + ν^H e₃ᵀ / c̃,   c̃ = max(c, c_min)
//                    Σ_ρ = E⊥ᵀ Π Σ^H_r Πᵀ E⊥,   Σ^H_r = Rᵀ Σ_p R
//                    σ_s² = dᵀ Σ_p d  (d = R e₃),   σ_t = σ_s / c̃
//   velocity         Var(mᵀ ν^H) = ℓᵀ Σ_b ℓ,   ℓ = [ω_h × Rm ; Rm]
//
// c̃ is a numerical guard: c ≥ c_min is itself a constraint the solver may
// violate on the way, and Π diverges at c → 0. Wherever that constraint holds
// (c > c_min) the guard is inactive and the expressions are the reference's.
// Every standard deviation is √(variance + ε_σ²): at zero variance the plain
// square root has no derivative.
//
// ── Contracts ─────────────────────────────────────────────────────────────────
// RT-safe: no heap after the work structs are sized (Resize / Init, non-RT),
// noexcept. The model must have nq == nv. Preconditions the caller owns (the
// core validates them once at Init): c_min > 0, ε_σ > 0, finite inputs, Σ
// symmetric. A function returning bool reports a size mismatch; the outputs
// are then unspecified.
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace rtc::catching {

using DockingMatrix3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using DockingMatrix6X = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using DockingCovariance6 = Eigen::Matrix<double, 6, 6>;

// ── Normal distribution helpers (used at Init to turn risks into κ) ────────────

/// @brief Standard normal CDF Φ(x).
[[nodiscard]] double NormalCdf(double x) noexcept;

/// @brief Standard normal quantile Φ⁻¹(p), accurate to rounding.
/// @param[out] z written only on success
/// @return false unless 0 < p < 1.
[[nodiscard]] bool NormalQuantile(double p, double& z) noexcept;

/// @brief Largest total timing standard deviation σ_max for which the closure,
///        nominally δ₀ after the crossing, falls inside [δ_lo, δ_hi] with
///        probability ≥ 1 − ε_t:
///
///          Φ((δ_hi − δ₀)/σ) − Φ((δ_lo − δ₀)/σ) = 1 − ε_t.
///
/// The left side decreases monotonically in σ when δ_lo < δ₀ < δ_hi, so the
/// root is unique. With δ₀ centred it is the reference's
/// (δ_hi − δ_lo) / (2 Φ⁻¹(1 − ε_t/2)).
/// @return false unless δ_lo < δ₀ < δ_hi (strict), 0 < ε_t < 1, all finite.
[[nodiscard]] bool DockingTimingSigmaMax(double delta_lo, double delta_hi, double delta_0,
                                         double eps_t, double& sigma_max) noexcept;

// ── Capture-frame kinematics at (q, q̇) ─────────────────────────────────────────

struct DockingFrameKinematics {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};      ///< p_h [m]
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};  ///< R_WH
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};      ///< v_h = J_p q̇ [m/s], world axes
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};      ///< ω_h = J_ω q̇ [rad/s], world axes
  DockingMatrix3X j_p, j_w;                        ///< 3 × n
  DockingMatrix3X dv_dq, dw_dq;                    ///< ∂_q v_h, ∂_q ω_h (3 × n)
  DockingMatrix6X d6_dq, d6_dv;                    ///< 6 × n scratch
  DockingMatrix3X pv_dv;                           ///< 3 × n scratch

  /// @brief Size every matrix for an n-joint arm (non-RT).
  void Resize(Eigen::Index n);
};

/// @brief FK, Jacobians and velocity derivatives of `frame` at (q, v).
/// @param data pinocchio workspace of `model`; overwritten (it holds the
///        forward-kinematics derivatives of (q, v) on return)
/// @return false on a size mismatch or an unknown frame.
[[nodiscard]] bool ComputeDockingFrameKinematics(const pinocchio::Model& model,
                                                 pinocchio::Data& data, pinocchio::FrameIndex frame,
                                                 const Eigen::Ref<const Eigen::VectorXd>& q,
                                                 const Eigen::Ref<const Eigen::VectorXd>& v,
                                                 DockingFrameKinematics& out) noexcept;

// ── Relative state ────────────────────────────────────────────────────────────

struct DockingRelativeState {
  Eigen::Vector3d r_w{Eigen::Vector3d::Zero()};   ///< r^W [m]
  Eigen::Vector3d r_h{Eigen::Vector3d::Zero()};   ///< r^H [m]
  Eigen::Vector3d nu_h{Eigen::Vector3d::Zero()};  ///< ν^H [m/s]
  DockingMatrix3X dr_dq;                          ///< ∂r^H/∂q = ∂ν^H/∂q̇ (3 × n)
  DockingMatrix3X dnu_dq;                         ///< ∂ν^H/∂q (3 × n)
  double s{0.0};                                  ///< e₃ᵀ r^H [m]
  double c{0.0};                                  ///< −e₃ᵀ ν^H [m/s]

  void Resize(Eigen::Index n);

  [[nodiscard]] Eigen::Vector2d Rho() const noexcept { return r_h.head<2>(); }
};

/// @brief r^H, ν^H and their derivatives for a ball at (p_b, v_b).
/// @return false when `kin` and `out` are not sized alike.
[[nodiscard]] bool ComputeDockingRelativeState(const DockingFrameKinematics& kin,
                                               const Eigen::Vector3d& p_b,
                                               const Eigen::Vector3d& v_b,
                                               DockingRelativeState& out) noexcept;

/// @brief Relative acceleration ν̇^H of the ball in the capture frame at
///        (q, q̇, q̈) — a diagnostic (the size of the term the crossing-plane
///        linearisation drops), not a constraint.
/// @param data overwritten (second-order forward kinematics of (q, v, a))
/// @return false on a size mismatch or an unknown frame.
[[nodiscard]] bool DockingRelativeAcceleration(
    const pinocchio::Model& model, pinocchio::Data& data, pinocchio::FrameIndex frame,
    const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& a, const Eigen::Vector3d& p_b,
    const Eigen::Vector3d& v_b, const Eigen::Vector3d& a_b, Eigen::Vector3d& a_rel_h) noexcept;

// ── Scalar rows: a value and its gradient in (q, q̇) ────────────────────────────

struct DockingScalar {
  double value{0.0};
  Eigen::VectorXd dq;  ///< ∂/∂q (n)
  Eigen::VectorXd dv;  ///< ∂/∂q̇ (n)

  void Resize(Eigen::Index n);
};

/// @brief Approach corridor g = ‖ρ‖² − w², w = r_ent + ℓ⁺ tanθ + s_c,
///        ℓ = s − s_ent, ℓ⁺ = max(ℓ, 0); feasible when g ≤ 0.
///
/// ℓ⁺ is a numerical guard: ℓ ≥ 0 is itself a constraint the solver may
/// violate on the way, and with ℓ < 0 the unguarded w can reach zero — then no
/// slack s_c ≥ 0 satisfies the linearised row. For ℓ ≥ 0 it is the reference's
/// row. ∂g/∂q̇ = 0.
/// @param[out] dg_ds_c ∂g/∂s_c = −2w
void DockingCorridorRow(const DockingRelativeState& rel, double s_ent, double r_ent,
                        double tan_theta, double s_c, DockingScalar& g, double& dg_ds_c) noexcept;

/// @brief Smallest s_c ≥ 0 that satisfies the corridor row at this state:
///        max(0, ‖ρ‖ − r_ent − ℓ⁺ tanθ).
[[nodiscard]] double DockingCorridorMinSlack(const DockingRelativeState& rel, double s_ent,
                                             double r_ent, double tan_theta) noexcept;

/// @brief Closing-speed envelope g = c² − c_ent,max² − 2 a_brake ℓ − s_v;
///        feasible when g ≤ 0. ∂g/∂s_v = −1.
void DockingEnvelopeRow(const DockingRelativeState& rel, double s_ent, double c_ent_max,
                        double a_brake, double s_v, DockingScalar& g) noexcept;

/// @brief Smallest s_v ≥ 0 that satisfies the envelope row at this state.
[[nodiscard]] double DockingEnvelopeMinSlack(const DockingRelativeState& rel, double s_ent,
                                             double c_ent_max, double a_brake) noexcept;

// ── Crossing-plane distribution (reference §8.2) ───────────────────────────────

struct DockingCrossing {
  Eigen::Matrix2d sigma_rho{Eigen::Matrix2d::Zero()};  ///< Σ_ρ [m²]
  double var_s{0.0};                                   ///< dᵀ Σ_p d [m²] (no ε_σ)
  double sigma_s{0.0};                                 ///< √(var_s + ε_σ²) [m]
  double sigma_t{0.0};                                 ///< σ_s / c̃ [s]
  double c_tilde{0.0};                                 ///< max(c, c_min) [m/s]
  bool c_guarded{false};  ///< c ≤ c_min: the guard is active (Π is not the reference's)
};

/// @brief Σ_ρ, σ_s and σ_t at this state (values only — the rows below carry
///        their own derivatives).
void ComputeDockingCrossing(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                            const Eigen::Matrix3d& sigma_p, double c_min, double eps_sigma,
                            DockingCrossing& out) noexcept;

/// @brief Lateral chance row h = aᵀρ + κ √(aᵀ Σ_ρ a + ε_σ²) for one face
///        normal a ∈ R²; the face is satisfied when h ≤ b.
///
/// The variance is evaluated as wᵀ Σ_p w with w = R Πᵀ E⊥ a — a quadratic form,
/// no factorisation, so a singular Σ_p is fine. The gradient carries the
/// dependence of Π on ν^H (through q and q̇) and of Σ^H_r on R.
void DockingLateralChanceRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                             const Eigen::Matrix3d& sigma_p, const Eigen::Vector2d& a, double kappa,
                             double c_min, double eps_sigma, DockingScalar& h) noexcept;

/// @brief Timing row t = c·k_t − σ_s(q), σ_s = √(dᵀ Σ_p d + ε_σ²); satisfied
///        when t ≥ 0. k_t = √(σ_max² − σ_τ²) is the caller's constant.
void DockingTimingRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                      const Eigen::Matrix3d& sigma_p, double k_t, double eps_sigma,
                      DockingScalar& t) noexcept;

/// @brief σ_m = √(Var(mᵀ ν^H) + ε_σ²) for a capture-frame direction m, with
///        its gradient (through R and ω_h).
void DockingVelocitySigma(const DockingFrameKinematics& kin, const DockingCovariance6& sigma_b,
                          const Eigen::Vector3d& m_hand, double eps_sigma,
                          DockingScalar& sigma) noexcept;

/// @brief Axial closing-speed row a = c + κ σ_c (σ_c along e₃). The lower
///        bound of V_cap passes κ = −κ_ν (a ≥ c_min), the upper κ = +κ_ν
///        (a ≤ c_cap,max).
void DockingAxialSpeedRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                          const DockingCovariance6& sigma_b, double kappa, double eps_sigma,
                          DockingScalar& row) noexcept;

/// @brief Lateral-speed face row a = uᵀ E⊥ᵀ ν^H + κ σ_u for one face normal
///        u ∈ R² of the inscribed polygon; satisfied when a ≤ v⊥,max cos(π/m).
void DockingLateralSpeedRow(const DockingFrameKinematics& kin, const DockingRelativeState& rel,
                            const DockingCovariance6& sigma_b, const Eigen::Vector2d& u,
                            double kappa, double eps_sigma, DockingScalar& row) noexcept;

// ── Impact (reference §7) ──────────────────────────────────────────────────────

/// Workspace of ComputeDockingImpact; Init is non-RT.
struct DockingImpactWork {
  pinocchio::Data data;          ///< its own Data: the impact calls set q̇ := y
  DockingFrameKinematics kin_y;  ///< kinematics at (q, y)
  Eigen::MatrixXd mass;          ///< M (n × n), symmetric
  Eigen::LLT<Eigen::MatrixXd> llt;
  Eigen::VectorXd f, y, zero;
  Eigen::MatrixXd dtau_dq, dtau_dv, dtau_da, dg_dq;  ///< n × n
  DockingMatrix3X j_c, dvc_dq;                       ///< 3 × n

  void Init(const pinocchio::Model& model);
};

struct DockingImpact {
  double beta_h{0.0};     ///< nᵀ J_c M⁻¹ J_cᵀ n [1/kg]
  double m_red{0.0};      ///< (1/m_b + β_h)⁻¹ [kg]
  DockingScalar beta;     ///< β_h and ∂β_h/∂q (dv = 0)
  DockingScalar g_n;      ///< nᵀ(v_b − J_c q̇) [m/s]; approaching when ≤ 0
  DockingScalar energy;   ///< E = ½ m_red c_n², c_n = −g_n [J]
  DockingScalar impulse;  ///< P = (1 + e) m_red c_n [N·s]
  /// √(m_red / 2) · c_n [√J]: the residual whose square is E — the
  /// Gauss–Newton form of the impact cost w_E E / E_ref.
  DockingScalar root_energy;

  void Resize(Eigen::Index n);
};

/// @brief Normal impact quantities at the capture frame's contact point
///        p_c = p_h + R p^H_c with normal n = R e₃, on the model's own inertia
///        (armature included — pass the hand-locked arm for the conservative
///        member of the reference's hierarchy).
///
/// β_h = fᵀ M⁻¹ f with f = J_cᵀ n, by Cholesky. Its gradient is
///   ∂β/∂q = 2 nᵀ ∂_q(J_c y)|_y + 2 (J_c y)ᵀ ∂_q n − yᵀ ∂_q(M y)|_y,  y = M⁻¹ f,
/// where ∂_q n = −[n]× J_ω and ∂_q(M y)|_y is the q-derivative of RNEA(q, 0, y)
/// minus the gravity derivative.
/// @param kin       kinematics at (q, v) (ComputeDockingFrameKinematics)
/// @param p_c_hand  contact point in the capture frame [m]
/// @param m_ball    ball mass [kg], > 0
/// @param restitution e ∈ [0, 1]
/// @return false on a size mismatch or when M is not positive definite.
[[nodiscard]] bool ComputeDockingImpact(const pinocchio::Model& model, pinocchio::FrameIndex frame,
                                        const Eigen::Ref<const Eigen::VectorXd>& q,
                                        const Eigen::Ref<const Eigen::VectorXd>& v,
                                        const DockingFrameKinematics& kin,
                                        const Eigen::Vector3d& v_b, const Eigen::Vector3d& p_c_hand,
                                        double m_ball, double restitution, DockingImpactWork& work,
                                        DockingImpact& out) noexcept;

// ── Manipulability regulariser (reference §9.2) ────────────────────────────────

/// Workspace of ComputeDockingManipulability; Init is non-RT.
struct DockingManipulabilityWork {
  pinocchio::Data data;
  pinocchio::Data::Tensor3x hessian;  ///< 6 × n × n, H(i, j, k) = ∂J_ij/∂q_k
  DockingMatrix6X j6, j_bar, b;       ///< 6 × n

  void Init(const pinocchio::Model& model);
};

/// @brief ψ_m = −log det(J̄ J̄ᵀ + δI), J̄ = D_x⁻¹ J D_q, and its exact gradient
///        (from the kinematic Hessian). J is the 6 × n frame Jacobian; ψ_m does
///        not depend on whether it is LOCAL or LOCAL_WORLD_ALIGNED, because
///        D_x scales translation and rotation each uniformly.
/// @param d_x_lin,d_x_ang task scales [m], [rad], > 0
/// @param d_q joint scales (n), > 0
/// @param delta δ > 0
/// @return false on a size mismatch or a failed factorisation.
[[nodiscard]] bool ComputeDockingManipulability(
    const pinocchio::Model& model, pinocchio::FrameIndex frame,
    const Eigen::Ref<const Eigen::VectorXd>& q, double d_x_lin, double d_x_ang,
    const Eigen::Ref<const Eigen::VectorXd>& d_q, double delta, DockingManipulabilityWork& work,
    double& psi, Eigen::Ref<Eigen::VectorXd> grad) noexcept;

}  // namespace rtc::catching
