#ifndef RTC_CONTROLLERS_ESTIMATION_CONTACT_INEKF_HPP_
#define RTC_CONTROLLERS_ESTIMATION_CONTACT_INEKF_HPP_

// ── Contact-aided right-invariant EKF core: IMU propagation + covariance ─────
//
// Robot-agnostic. No ROS, no model, no kinematics: contact points and their
// Jacobians arrive as arguments already expressed in the IMU frame.
//
// ### Frames and state
//
//   W  world (local, z up, gravity g = (0, 0, −g₀))
//   I  the primary IMU frame — the filter's body frame
//
//   X = (R, v, p, d_0 … d_{n−1}) ∈ SE_{2+n}(3),   θ = (b_g, b_a)
//     R    = R_WI                     rotation IMU → world
//     v, p = IMU origin velocity / position in W [m/s, m]
//     d_i  = contact anchor i in W [m]   (slot i, 0 ≤ i < n ≤ 4)
//     b_g  = gyro bias [rad/s], b_a = accel bias [m/s²], in I
//
// ### Measurement model (continuous)
//
//   ω_m = ω + b_g + n_g        a_m = Rᵀ(v̇ − g) + b_a + n_a
//   ḃ_g = n_bg   ḃ_a = n_ba   ḋ_i = R n_d,i
//
// n_g, n_a, n_bg, n_ba, n_d are independent white processes given by their
// CONTINUOUS densities (Config). A per-sample σ of a sensor sampled at period
// T_s corresponds to the density σ·√T_s — that conversion belongs to the caller.
//
// ### Error convention (right-invariant)
//
//   X = Exp(ξ)·X̂,   ζ = θ − θ̂,   e = [ξ_φ, ξ_v, ξ_p, ξ_d0 … ξ_d(n−1), ζ_g, ζ_a]
//
// dimension 15 + 3n, in exactly that order (SlotOffset() / BiasOffset()). An
// inactive slot keeps its rows and columns of P at +0.0 and its mean at 0.
//
//   Exp(ξ) = (exp([ξ_φ]×), J_l(ξ_φ)ξ_v, J_l(ξ_φ)ξ_p, J_l(ξ_φ)ξ_d…)
//   Ad(X)  : ξ_φ ↦ R ξ_φ,  ξ_c ↦ [c]× R ξ_φ + R ξ_c   for c ∈ {v, p, d_i}
//
// ξ_φ is a WORLD-frame rotation error, so a rotation of the world about the
// gravity axis is a symmetry of the error dynamics.
//
// ### Mean propagation (exact zero-order hold)
//
// With ω̄ = ω_m − b̂_g, ā = a_m − b̂_a held over the step and φ = ω̄·Δt:
//
//   R⁺ = R Γ₀(φ)
//   v⁺ = v + R Γ₁(φ) ā Δt + g Δt
//   p⁺ = p + v Δt + R Γ₂(φ) ā Δt² + ½ g Δt²
//   d⁺ = d,  θ̂⁺ = θ̂
//
//   Γ_m(φ) = Σ_k [φ]×ᵏ / (k + m)!  = I/m! + c₁(θ)[φ]× + c₂(θ)[φ]×²,
//   c₁ = Σ_j (−θ²)ʲ/(2j+m+1)!,  c₂ = Σ_j (−θ²)ʲ/(2j+m+2)!,  θ = ‖φ‖.
//
// The coefficients are a truncated power series, accurate to double precision
// only for θ ≤ kMaxRotationStep — which is why a step beyond it is refused
// rather than computed. Closed forms are avoided on purpose: Γ₂'s [φ]×²
// coefficient (θ² + 2cosθ − 2)/(2θ⁴) loses all digits near θ = 1e-4.
// R⁺ is re-orthonormalised by one Newton step R ← ½R(3I − RᵀR).
//
// ### Error transition Φ and process noise Q_d
//
// The LEFT-invariant error e_L = Ad(X̂)⁻¹ ξ obeys a linear time-invariant
// system while ω̄, ā are held (s = elapsed time, W = [ω̄]×):
//
//   ė_φ = −W e_φ − ζ_g          ė_v = −[ā]× e_φ − W e_v − ζ_a
//   ė_p = e_v − W e_p           ė_d = −W e_d          ζ̇ = 0
//
// whose transition Φ_L(u) over an elapsed time u has the closed blocks
// (E = Γ₀(ω̄u)ᵀ):
//
//   φφ = vv = pp = dd = E,  pv = uE,
//   vφ = −E[u Γ₁(ω̄u) ā]×,   pφ = −E[u² Γ₂(ω̄u) ā]×,
//   φ←b_g = −u Γ₁(ω̄u)ᵀ,     v←b_a = −u Γ₁(ω̄u)ᵀ,   p←b_a = −u² E Γ₂(ω̄u),
//   v←b_g = E ∫₀ᵘ f(s) ds,   p←b_g = E ∫₀ᵘ (u − s) f(s) ds,
//   f(s)  = [Γ₀(ω̄s) ā]× · s Γ₁(ω̄s)
//
// (the two integrals by 5-point Gauss–Legendre). The right-invariant
// transition is Φ = Ad(X̂⁺) Φ_L(Δt) Ad(X̂)⁻¹; its state–state part is the
// input-free exp(A Δt) = I + (v←φ: [g]×Δt, p←φ: ½[g]×Δt², p←v: IΔt), and only
// its bias columns need Ad(X̂⁺)·Φ_L(Δt)[:, θ].
//
//   Q_d = Ad(X̂⁺) [ ∫₀^Δt Φ_L(u) Q_c Φ_L(u)ᵀ du ] Ad(X̂⁺)ᵀ
//   Q_c = diag(σ_g² I, σ_a² I, 0, (s_i σ_d)² I …, σ_bg² I, σ_ba² I)
//
// with the outer integral by 5-point Gauss–Legendre and s_i ≥ 1 the slot's
// anchor-noise scale. No matrix exponential is evaluated on the tick.
//
//   P⁺ = Φ P Φᵀ + Q_d,   then P ← ½(P + Pᵀ) and a Cholesky test on the
//   active block; a failed test refuses the step.
//
// ### Initialisation (stationary, gravity aligned, configured heading)
//
// From the mean specific force ā and mean rate ω̄ of a window of duration T:
//   gate      |‖ā‖ − g₀| ≤ stationary_accel_tolerance,
//             ‖ω̄‖ ≤ stationary_gyro_tolerance (two independent tests)
//   roll  = atan2(ā_y, ā_z),  pitch = atan2(−ā_x, √(ā_y² + ā_z²)),
//   R₀    = Rz(yaw) Ry(pitch) Rx(roll),  so R₀ᵀ e_z = ā/‖ā‖
//   v = 0, θ̂ = 0, p = −R₀ · mean(r_i) over the given stance points (IMU frame),
//   i.e. the mean of the anchors those points would make is the origin.
//
// A horizontal accel bias is indistinguishable from tilt, so the attitude
// error is correlated with ζ_a. Writing the truth as R₀ evaluated at
// ā − (b_a + n̄), ξ_φ ≈ M (b_a + n̄) with the alignment Jacobian
//
//   M = −( Rz e_y ∂pitch/∂āᵀ + Rz Ry e_x ∂roll/∂āᵀ ),
//   ∂roll/∂ā  = (0, ā_z, −ā_y)/h²,  ∂pitch/∂ā = (−h, ā_x ā_y/h, ā_x ā_z/h)/‖ā‖²,
//   h = √(ā_y² + ā_z²)
//
// and, with S_b = σ_ba² I and S_n = σ_a²/T · I (the window mean's noise),
//
//   P₀[φ,φ]   = M (S_b + S_n) Mᵀ + σ_yaw² e_z e_zᵀ
//   P₀[φ,b_a] = M S_b
//   P₀[v,v] = σ_v² I, P₀[p,p] = σ_p² I, P₀[b_g,b_g] = σ_bg² I, P₀[b_a,b_a] = S_b.
//
// All three rows of M are kept: once the IMU is tilted, the world-z component
// of the attitude error also moves with b_a.
//
// M grows as 1/cos(pitch), and the prior's only margin along one attitude
// direction is σ_yaw² cos²(pitch): Initialize() refuses cos(pitch) < 0.1
// (|pitch| > 84.3°, gravity near the IMU x axis), where the configured ZYX
// heading stops being a usable initial condition.
//
// ### Contacts
//
// A contact point i has IMU-frame position r_i (forward kinematics) and joint
// Jacobian J_i = ∂r_i/∂q (3 × n_j, IMU frame, no base columns); its body-frame
// noise is Σ_i = σ_q² J_i J_iᵀ + σ_kin² I.
//
// Insert (slot inactive → active): d_i = p + R r_i. With X = Exp(ξ)X̂ the new
// error is ξ_d,i = ξ_p + R δr_i (the attitude error cancels — the point of
// the right-invariant form), so the slot's rows and columns copy those of p
// and P[d_i, d_i] = P[p, p] + R Σ_i Rᵀ.
//
// Remove (active → inactive): the slot's rows, columns and mean go to +0.0
// (marginalisation); every other entry is untouched. Its noise scale returns
// to 1.
//
// Update, for the requested active slots:
//
//   z_i = R̂ r_i + p̂ − d̂_i = H_i e + R̂ δr_i,   H_i = [0 0 −I … I(d_i) … 0 0]
//   N_i = R̂ Σ_i R̂ᵀ,   S_i = H_i P H_iᵀ + N_i,   m_i² = z_iᵀ S_i⁻¹ z_i
//
// A slot with m_i² > contact_gate_chi2 (computed before any update) is left
// out and reported, as is a slot inserted since the last Propagate() (its
// anchor covariance already holds that kinematic sample); the rest are
// stacked into one update:
//
//   S = H P Hᵀ + N,   K = P Hᵀ S⁻¹ (Cholesky solve),   δ = K z
//   X̂⁺ = Exp(δ_X) X̂,   θ̂⁺ = θ̂ + δ_θ,   P⁺ = (I − KH) P (I − KH)ᵀ + K N Kᵀ
//
// Exp here is rtc_math's exp3 / left Jacobian (an update step is not bounded
// by kMaxRotationStep). The reported innovation is the body-frame R̂ᵀ z_i.
// N is block diagonal: contacts driven by shared joints are treated as
// independent (their encoder-noise cross terms are not modelled).
//
// ### IMU → base
//
// BaseFromImu() turns the IMU state into the base pose and world velocity of a
// frame rigidly attached to the IMU (extrinsic as arguments): no angular
// acceleration is needed because the propagated point is the IMU itself.
//
// ### RT contract
//
// Init() is non-RT and throws std::invalid_argument. Every other member is
// noexcept, allocation-free and fixed-capacity; a call that returns a status
// other than kOk leaves the whole filter state bit-identical. After any
// failure, recovery is Reset() and a new Initialize().
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <span>

namespace rtc::estimation {

/// Contact-slot capacity. Storage is sized by it; arithmetic by the actual
/// slot count given to Init().
inline constexpr int kInekfMaxContacts = 4;
/// Error-state dimension at full capacity (15 + 3·kInekfMaxContacts).
inline constexpr int kInekfMaxDim = 15 + 3 * kInekfMaxContacts;
/// Largest ‖ω̄‖·Δt one Propagate() accepts [rad]. The Γ series is exact to
/// double precision up to here.
inline constexpr double kMaxRotationStep = 0.7;
/// Accepted Propagate() step range [s], both ends inclusive. Callers pass the
/// NOMINAL step (missed ticks × period), not a measured clock difference.
inline constexpr double kMinPropagationStep = 2e-4;
inline constexpr double kMaxPropagationStep = 2e-2;
/// Column capacity of a contact Jacobian (joints that move one contact point).
inline constexpr int kInekfMaxJacobianCols = 64;

/// Outcome of a filter call. Anything but kOk left the state untouched.
enum class InekfStatus : std::uint8_t {
  kOk = 0,
  kNotConfigured = 1,         ///< Called before a successful Init().
  kNotInitialized = 2,        ///< Propagate before Initialize(), or after Reset().
  kAlreadyInitialized = 3,    ///< Initialize() on a running filter (Reset() first).
  kNonFiniteInput = 4,        ///< Some input was NaN/Inf.
  kStepOutOfRange = 5,        ///< Δt outside [kMinPropagationStep, kMaxPropagationStep].
  kRotationOutOfRange = 6,    ///< ‖ω_m − b̂_g‖·Δt > kMaxRotationStep.
  kNonFiniteResult = 7,       ///< Finite inputs produced a non-finite mean or P.
  kNotPositiveDefinite = 8,   ///< The active block of P failed its Cholesky test.
  kAccelNotStationary = 9,    ///< |‖ā‖ − g₀| above the stationary tolerance.
  kGyroNotStationary = 10,    ///< ‖ω̄‖ above the stationary tolerance.
  kAlignmentSingular = 11,    ///< |pitch| > 84.3° (gravity near the IMU x axis).
  kBadWindow = 12,            ///< Window duration not positive.
  kTooManyStancePoints = 13,  ///< More stance points than Init()'s slot count.
  kBadSlot = 14,              ///< Slot index outside [0, num_slots()).
  kBadNoiseScale = 15,        ///< Anchor-noise scale below 1.
  kSlotActive = 16,           ///< InsertContact on a slot that is already active.
  kSlotInactive = 17,         ///< RemoveContact, or a requested update, on an inactive slot.
  kSizeMismatch = 18,         ///< UpdateContacts spans not num_slots() long.
};

/// Γ_M(φ) = Σ_k [φ]×ᵏ/(k+M)! for M ∈ {0, 1, 2}, by its power series. Valid
/// for ‖φ‖ ≤ kMaxRotationStep only (outside, the truncation is not bounded). Γ₀
/// is the rotation exp([φ]×) and Γ₁ the SO(3) left Jacobian.
template <int M>
[[nodiscard]] Eigen::Matrix3d GammaSeries(const Eigen::Vector3d& phi) noexcept;
extern template Eigen::Matrix3d GammaSeries<0>(const Eigen::Vector3d& phi) noexcept;
extern template Eigen::Matrix3d GammaSeries<1>(const Eigen::Vector3d& phi) noexcept;
extern template Eigen::Matrix3d GammaSeries<2>(const Eigen::Vector3d& phi) noexcept;

/// Running mean of IMU samples for the stationary initialisation window.
/// Compensated (Neumaier) sums, so the mean of a long window is not limited by
/// summation order. Non-finite samples are refused, not folded in.
class StationaryImuAccumulator {
 public:
  /// Fold one sample in. Returns false (and changes nothing) if either vector
  /// has a non-finite element.
  [[nodiscard]] bool Add(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) noexcept;
  /// Start a new window.
  void Reset() noexcept;

  [[nodiscard]] std::int64_t count() const noexcept { return count_; }

  /// Mean of the accepted samples; zero while count() == 0.
  [[nodiscard]] Eigen::Vector3d MeanGyro() const noexcept { return MeanAt(0); }

  [[nodiscard]] Eigen::Vector3d MeanAccel() const noexcept { return MeanAt(3); }

 private:
  std::array<double, 6> sum_{};
  [[nodiscard]] Eigen::Vector3d MeanAt(std::size_t offset) const noexcept;

  std::array<double, 6> comp_{};
  std::int64_t count_{0};
};

/// IMU-frame forward kinematics of one contact point.
struct ContactKinematics {
  using Jacobian =
      Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor, 3, kInekfMaxJacobianCols>;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  ///< r_i in I [m]
  /// ∂r_i/∂q in I [m/rad]; 0 … kInekfMaxJacobianCols columns (the storage
  /// capacity — a wider assignment is the caller's error and is not checked).
  Jacobian jacobian{3, 0};
  bool use{false};  ///< UpdateContacts only: include this slot
};

/// Why a slot did or did not enter the last UpdateContacts().
enum class ContactUpdateResult : std::uint8_t {
  kInactive = 0,      ///< Slot inactive and not requested.
  kNotRequested = 1,  ///< Active, `use` = false: anchor kept, no update.
  kUsed = 2,          ///< Entered the stacked update.
  kGated = 3,         ///< m² above contact_gate_chi2: left out.
  /// Inserted since the last Propagate(): its anchor covariance already holds
  /// this sample's kinematic noise, so using it again would count it twice.
  kInsertedThisStep = 4,
};

/// Per-slot outcome of UpdateContacts(); written only when it returns kOk.
struct ContactUpdateReport {
  Eigen::Vector3d innovation{Eigen::Vector3d::Zero()};  ///< R̂ᵀ z_i [m], requested slots
  double mahalanobis2{0.0};  ///< m_i² before the update, requested slots
  ContactUpdateResult result{ContactUpdateResult::kInactive};
};

/// Base pose and world velocity derived from the IMU state.
struct BaseState {
  Eigen::Matrix3d R_WB{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_WB{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_WB{Eigen::Vector3d::Zero()};
};

/// Contact-aided right-invariant EKF over SE_{2+n}(3) with augmented IMU bias.
/// See the file preamble for every equation, frame and sign.
class ContactInekf {
 public:
  using CovMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor,
                                  kInekfMaxDim, kInekfMaxDim>;

  /// Noise model, priors and gates. Configuration, never robot constants.
  /// Every field must be > 0 with a finite, normal square.
  struct Config {
    double gravity{0.0};                     ///< g₀ [m/s²]; world gravity is (0, 0, −g₀).
    double gyro_noise_density{0.0};          ///< σ_g  [rad/s/√Hz]
    double accel_noise_density{0.0};         ///< σ_a  [m/s²/√Hz]
    double gyro_bias_walk{0.0};              ///< σ_bg [rad/s²/√Hz]
    double accel_bias_walk{0.0};             ///< σ_ba-walk [m/s³/√Hz]
    double contact_noise_density{0.0};       ///< σ_d  anchor random walk [m/√s]
    double joint_position_sigma{0.0};        ///< σ_q  encoder noise for contact updates [rad]
    double kinematic_sigma{0.0};             ///< σ_kin model error of a contact point [m]
    double contact_gate_chi2{0.0};           ///< χ²(3) threshold for a contact innovation
    double prior_velocity_sigma{0.0};        ///< σ_v   [m/s]
    double prior_position_sigma{0.0};        ///< σ_p   [m]
    double prior_yaw_sigma{0.0};             ///< σ_yaw [rad], about world z
    double prior_gyro_bias_sigma{0.0};       ///< σ_bg0 [rad/s]
    double prior_accel_bias_sigma{0.0};      ///< σ_ba  [m/s²]
    double stationary_accel_tolerance{0.0};  ///< max |‖ā‖ − g₀| [m/s²]
    double stationary_gyro_tolerance{0.0};   ///< max ‖ω̄‖ [rad/s]
  };

  /// Unconfigured, with every state member zeroed.
  ContactInekf() noexcept { Reset(); }

  /// Validate the configuration and size the state for `num_slots` contact
  /// slots (non-RT). Leaves the filter uninitialised.
  /// @throws std::invalid_argument on a field that is non-finite, ≤ 0, or whose
  ///         square underflows or overflows, or
  ///         num_slots outside [0, kInekfMaxContacts].
  void Init(const Config& config, int num_slots);

  /// Start the filter from a stationary window (see preamble §Initialisation).
  ///   mean_gyro, mean_accel — window means in I [rad/s, m/s²]
  ///   window_duration      — T [s] of that window (sets the mean's noise)
  ///   yaw                  — configured ZYX heading of R_WI [rad]
  ///   stance_points        — contact points in I [m] that fix the origin;
  ///                          no anchors are created here
  [[nodiscard]] InekfStatus Initialize(const Eigen::Vector3d& mean_gyro,
                                       const Eigen::Vector3d& mean_accel, double window_duration,
                                       double yaw,
                                       std::span<const Eigen::Vector3d> stance_points) noexcept;

  /// One exact-ZOH step with raw IMU samples in I and the nominal step Δt [s].
  [[nodiscard]] InekfStatus Propagate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel,
                                      double dt) noexcept;

  /// Scale slot i's anchor random walk by `scale` ≥ 1 (σ_d → scale·σ_d) from
  /// the next Propagate() on; (scale·σ_d)² must stay finite. Persists until
  /// changed, RemoveContact() or Reset().
  [[nodiscard]] InekfStatus SetContactNoiseScale(int slot, double scale) noexcept;

  /// Anchor slot `slot` at the contact point `kin` (position + Jacobian; `use`
  /// is ignored). The slot must be inactive.
  [[nodiscard]] InekfStatus InsertContact(int slot, const ContactKinematics& kin) noexcept;

  /// Marginalise slot `slot` out. The slot must be active.
  [[nodiscard]] InekfStatus RemoveContact(int slot) noexcept;

  /// One stacked contact update. `kinematics` and `report` are num_slots()
  /// long; slot i enters if kinematics[i].use (it must then be active and
  /// finite). Slots gated by χ² are left out and reported; if none remain the
  /// state is unchanged and the call still returns kOk.
  [[nodiscard]] InekfStatus UpdateContacts(std::span<const ContactKinematics> kinematics,
                                           std::span<ContactUpdateReport> report) noexcept;

  /// Drop the state, slot mask and noise scales; keep the configuration.
  void Reset() noexcept;

  // ── Layout ────────────────────────────────────────────────────────────────
  /// Row of ξ_d,i in e / P.
  [[nodiscard]] static constexpr int SlotOffset(int slot) noexcept { return 9 + 3 * slot; }

  /// Row of ζ_g in e / P (ζ_a follows at +3).
  [[nodiscard]] constexpr int BiasOffset() const noexcept { return 9 + 3 * num_slots_; }

  /// 15 + 3·num_slots(), or 0 before Init().
  [[nodiscard]] int dim() const noexcept { return num_slots_ < 0 ? 0 : 15 + 3 * num_slots_; }

  [[nodiscard]] int num_slots() const noexcept { return num_slots_ < 0 ? 0 : num_slots_; }

  // ── State ─────────────────────────────────────────────────────────────────
  [[nodiscard]] bool configured() const noexcept { return num_slots_ >= 0; }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

  [[nodiscard]] const Eigen::Matrix3d& rotation() const noexcept { return R_; }  ///< R_WI

  /// R_WI as a unit quaternion (Hamilton).
  [[nodiscard]] Eigen::Quaterniond orientation() const noexcept;

  [[nodiscard]] const Eigen::Vector3d& velocity() const noexcept { return v_; }  ///< in W

  [[nodiscard]] const Eigen::Vector3d& position() const noexcept { return p_; }  ///< in W

  [[nodiscard]] const Eigen::Vector3d& gyro_bias() const noexcept { return bg_; }

  [[nodiscard]] const Eigen::Vector3d& accel_bias() const noexcept { return ba_; }

  /// ω_m − b̂_g of the last accepted Propagate() (Initialize: the window mean) [rad/s, in I].
  [[nodiscard]] const Eigen::Vector3d& angular_velocity() const noexcept { return omega_; }

  /// Anchor of slot i in W; zero while inactive. Out-of-range i reads slot 0.
  [[nodiscard]] const Eigen::Vector3d& contact_position(int slot) const noexcept;
  [[nodiscard]] bool contact_active(int slot) const noexcept;
  [[nodiscard]] double contact_noise_scale(int slot) const noexcept;

  /// Error covariance, dim() × dim(), in the order of the preamble.
  [[nodiscard]] const CovMatrix& covariance() const noexcept { return P_; }

  /// Base pose / world velocity of a frame B rigidly attached to the IMU.
  ///   R_BI, p_BI — IMU pose in B (the extrinsic)
  ///   omega_I    — angular velocity in I (angular_velocity())
  /// R_WB = R_WI R_BIᵀ,  p_WB = p_WI − R_WB p_BI,  v_WB = v_WI + R_WI (ω_I × p_IB),
  /// p_IB = −R_BIᵀ p_BI.
  [[nodiscard]] static BaseState BaseFromImu(const Eigen::Matrix3d& R_WI,
                                             const Eigen::Vector3d& p_WI,
                                             const Eigen::Vector3d& v_WI,
                                             const Eigen::Vector3d& omega_I,
                                             const Eigen::Matrix3d& R_BI,
                                             const Eigen::Vector3d& p_BI) noexcept;

 private:
  friend struct ContactInekfTestAccess;

  [[nodiscard]] bool SlotInRange(int slot) const noexcept {
    return slot >= 0 && slot < num_slots();
  }

  struct Mean {
    Eigen::Matrix3d R;
    Eigen::Vector3d v;
    Eigen::Vector3d p;
  };

  /// The exact-ZOH mean map with bias-corrected inputs (no range checks).
  [[nodiscard]] static Mean PropagateMean(const Eigen::Matrix3d& R, const Eigen::Vector3d& v,
                                          const Eigen::Vector3d& p, const Eigen::Vector3d& w,
                                          const Eigen::Vector3d& a, double dt,
                                          double gravity) noexcept;

  /// Φ_L(u) into `out` (dim() × dim()) for held ω̄, ā.
  void LeftTransition(const Eigen::Vector3d& w, const Eigen::Vector3d& a, double u,
                      CovMatrix& out) const noexcept;
  /// Ad(X) of (R, v, p, d…) extended by I on θ, into `out`.
  void Adjoint(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p,
               CovMatrix& out) const noexcept;
  /// Cholesky test of the active rows/columns of `P`.
  [[nodiscard]] bool ActiveBlockPositiveDefinite(const CovMatrix& P) noexcept;
  /// Inactive slot rows/columns → +0.0, then exact symmetrisation.
  void Tidy(CovMatrix& P) const noexcept;

  Config config_{};
  int num_slots_{-1};
  bool initialized_{false};

  Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, kInekfMaxContacts> d_;
  std::array<bool, kInekfMaxContacts> active_{};
  std::array<bool, kInekfMaxContacts> inserted_{};  // since the last Propagate()
  std::array<double, kInekfMaxContacts> noise_scale_{1.0, 1.0, 1.0, 1.0};
  CovMatrix P_;

  // Scratch, fully rewritten by every call that reads it. phi_ / qd_ hold the
  // last COMPUTED step's Φ and Q_d — a refused step included.
  CovMatrix phi_;
  CovMatrix qd_;
  CovMatrix ad_;
  CovMatrix left_;
  CovMatrix work_;
  CovMatrix next_;
  CovMatrix packed_;
  Eigen::LLT<CovMatrix> llt_;

  // Update scratch: stacked rows ≤ 3·kInekfMaxContacts. h_ / gain_ hold the
  // last computed update's H and K.
  static constexpr int kMaxRows = 3 * kInekfMaxContacts;
  using RowsMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, kMaxRows, kMaxRows>;
  using HMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, kMaxRows,
                                kInekfMaxDim>;
  using GainMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor,
                                   kInekfMaxDim, kMaxRows>;
  using RowsVector = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, kMaxRows, 1>;
  HMatrix h_;
  GainMatrix gain_;
  GainMatrix pht_;
  GainMatrix kn_;
  HMatrix kt_;
  RowsMatrix s_;
  RowsMatrix n_;
  RowsVector z_;
  Eigen::LLT<RowsMatrix> s_llt_;
};

}  // namespace rtc::estimation

#endif  // RTC_CONTROLLERS_ESTIMATION_CONTACT_INEKF_HPP_
