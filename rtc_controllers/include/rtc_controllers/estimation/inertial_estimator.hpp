#ifndef RTC_CONTROLLERS_ESTIMATION_INERTIAL_ESTIMATOR_HPP_
#define RTC_CONTROLLERS_ESTIMATION_INERTIAL_ESTIMATOR_HPP_

// ── Layer 2B payload inertial regression (#135 / #455) ──────────────────────
#include "rtc_controllers/estimation/momentum_observer.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <span>

namespace rtc::estimation {

/// Why an inertial estimate is not being reported. Mirrored by
/// rtc_msgs/PayloadEstimate.msg INERTIAL_* constants — never renumber.
///
/// Same split as PayloadEstimator: gates whose evidence does not reach this
/// class arrive through Hold(), the rest it decides from what it can see.
enum class InertialInvalidReason : std::uint8_t {
  kNone = 0,             ///< Valid; the estimate advanced this tick.
  kNotInitialized = 1,   ///< Update()/Hold() before a successful Init().
  kHeld = 2,             ///< Caller's gate was closed with no more specific reason.
  kShortInput = 3,       ///< `r` shorter than nv(), or Y had the wrong shape.
  kNonFiniteInput = 4,   ///< Some element of Y or r was NaN/Inf.
  kUpstreamInvalid = 5,  ///< The residual this regression consumes was not valid.
  /// The accumulated regressor still has rank < 4. This is the NORMAL state
  /// after a re-seed, not an error: one pose can never identify the 4 set (see
  /// the class preamble), so the estimator reports nothing until the arm has
  /// visited poses that span the missing direction.
  kInsufficientPoseDiversity = 6,
  /// Y's inertia columns were non-negligible, i.e. the arm was NOT quasi-static.
  /// Fitting 4 parameters to data that carries 6 more would dump the rotational
  /// torque into m̂ and ĉ. #135 AC 10 is exactly this refusal.
  kDynamicExcitation = 7,
  kSolverFailed = 8,     ///< The accumulated normal equations would not factor.
  kNonPositiveMass = 9,  ///< [C3] m̂ ≤ min_mass — not a physically realizable body.
  kComOutOfBounds = 10,  ///< [C3] |ĉ| beyond max_com_offset — outside the payload envelope.
};

/// One accepted inertial estimate plus the diagnostics that justify it.
struct InertialEstimate {
  double mass{0.0};                      ///< m̂ [kg].
  std::array<double, 3> first_moment{};  ///< m̂·ĉ [kg·m], payload frame, LOCAL.
  std::array<double, 3> com{};           ///< ĉ [m] — first_moment / m̂, gated by min_mass.
  /// σ_min of the accumulated 4×4 normal matrix. This is the pose-diversity
  /// meter: it sits at ~0 while the arm holds one pose and climbs as it visits
  /// others. The gate is stated on this rather than on a tick count because
  /// "moved for N ticks" and "moved somewhere that helps" are different facts.
  double param_sigma_min{0.0};
  std::uint8_t param_rank{0};  ///< Numerical rank of that matrix, 0..4.
  double fit_error{0.0};       ///< ‖Y₄φ̂ − r‖∞ [N·m] on the current tick.
  InertialInvalidReason reason{InertialInvalidReason::kNotInitialized};
  bool valid{false};
};

/// Recursive least squares for the OBSERVABLE part of a payload's inertial
/// parameter set, under the quasi-static gate Layer 2A already runs on.
///
/// ### Why four parameters and not ten
///
/// The full set is φ_L = [m, m·c, I(6)]. Under gravity alone the regressor's
/// six inertia columns are **identically zero** — not small, zero — because a
/// body's rotational inertia enters the dynamics only through `I·a + v×I·v`.
/// Stacking poses does not help: 60 poses give rank 4, not 10. So this class
/// fits the four it can see and never claims the six it cannot. Identifying
/// those needs angular-acceleration excitation, which is a different gate and a
/// different lane (#455 out of scope). RtModelHandle still computes all ten
/// columns, and its oracle test exercises them — the API is ready for that lane
/// even though this estimator is not it.
///
/// ### Why it must accumulate rather than solve per tick
///
/// A SINGLE pose gives rank exactly 3, with σ₄ exactly 0, on every robot and
/// every pose measured. The reason is structural: a gravity-loaded body applies
/// f = m·ᵂg and τ = (m·c) × ᵂg at the frame, and that cross product annihilates
/// the component of m·c along ᵂg. Only re-expressing gravity in a rotated frame
/// — i.e. moving the arm — reveals it. Layer 2A escapes this because it fits a
/// wrench through Jᵀ, not the parameters behind it.
///
/// So this class carries normal equations across ticks with a forgetting
/// factor, and refuses to report until σ_min says the missing direction has
/// actually been filled. A per-tick least squares here would return a finite,
/// smooth, wrong ĉ.
///
/// ### Coordinate order
///
/// Y's ROWS and r's ENTRIES must index the same velocity space; this class
/// never learns which one, exactly as PayloadEstimator does not. The caller
/// owns the reorder contract. Y's COLUMNS are pinocchio's
/// `Inertia::toDynamicParameters()` order, and `first_moment` comes back in the
/// payload frame those columns were built in.
///
/// RT contract: Init() is non-RT and may throw; Update() / Hold() / ResetRtState()
/// are noexcept, heap-free, fixed-size only.
class InertialEstimator {
 public:
  /// Thresholds. Configuration, never robot constants (ARCH-1).
  struct Config {
    /// Per-tick decay λ ∈ (0, 1] applied to the accumulator before each update.
    /// 1.0 means "never forget", which is right only if the payload cannot
    /// change during a run. Below 1 the estimator tracks a payload that is
    /// picked up and put down, at the cost of needing pose diversity again
    /// after each change.
    double forgetting_factor{0.0};
    /// σ_min of the 4×4 normal matrix below this reports
    /// kInsufficientPoseDiversity. This is the gate that makes the estimate
    /// honest — without it the solve still returns a number, just an arbitrary
    /// one along the unobserved direction.
    double min_param_sigma{0.0};
    /// m̂ at or below this reports kNonPositiveMass [kg] ([C3]). A negative mass
    /// is not a small error, it is evidence the residual is not a payload.
    double min_mass{0.0};
    /// |ĉ| above this reports kComOutOfBounds [m] ([C3]). The 4-parameter
    /// reduction of the full LMI consistency condition: with I unobserved the
    /// only realizability statements left are m > 0 and a bounded lever arm.
    double max_com_offset{0.0};
    /// An entry of Y's inertia columns above this means the tick was not
    /// quasi-static → kDynamicExcitation. Not zero, because Y is built from
    /// measured q̇/q̈ that are never exactly zero.
    double max_inertia_column{0.0};
  };

  InertialEstimator() = default;

  /// Validate thresholds and size every buffer (non-RT).
  /// @throws std::invalid_argument on nv outside [1, kMaxObserverDof], a
  ///         forgetting factor outside (0, 1], or any other threshold that is
  ///         non-positive or non-finite. Non-positive is rejected rather than
  ///         read as "disable this gate" — a disabled gate and a passing one
  ///         look identical downstream.
  void Init(int nv, const Config& config);

  /// Fold one tick into the accumulator and re-solve (RT, noexcept, heap-free).
  ///
  ///   Y — payload regressor, nv×10, from RtModelHandle::GetPayloadRegressor()
  ///   r — joint-space residual [N·m], nv entries, SAME order as Y's rows
  ///
  /// Y is taken at full width even though only the first four columns are
  /// fitted: the remaining six are what this class INSPECTS to decide whether
  /// the tick was quasi-static at all. A rejected tick leaves both the
  /// accumulator and the last estimate untouched.
  void Update(const Eigen::Ref<const Eigen::MatrixXd>& Y, std::span<const double> r) noexcept;

  /// Declare this tick unusable without offering inputs (RT). Carries which of
  /// the caller's gates closed. Does NOT decay the accumulator — a closed gate
  /// is an absence of evidence, not evidence the payload changed.
  void Hold(InertialInvalidReason reason) noexcept;

  /// Drop the accumulator and the latched estimate so a resumed loop cannot
  /// report a payload identified before a gap. Init-time state is kept.
  void ResetRtState() noexcept;

  [[nodiscard]] const InertialEstimate& estimate() const noexcept { return estimate_; }

  [[nodiscard]] bool valid() const noexcept { return reason_ == InertialInvalidReason::kNone; }

  [[nodiscard]] InertialInvalidReason invalid_reason() const noexcept { return reason_; }

  [[nodiscard]] int nv() const noexcept { return nv_; }

 private:
  /// Number of fitted parameters — [m, m·c_x, m·c_y, m·c_z].
  static constexpr int kParams = 4;

  void Reject(InertialInvalidReason reason) noexcept;

  int nv_{0};
  Config config_{};
  bool initialized_{false};
  InertialInvalidReason reason_{InertialInvalidReason::kNotInitialized};
  InertialEstimate estimate_{};

  // Accumulated normal equations: A_ = Σ λ^k Y₄ᵀY₄, b_ = Σ λ^k Y₄ᵀr. Fixed 4×4,
  // so the whole RLS state is 20 doubles regardless of DOF.
  Eigen::Matrix<double, kParams, kParams> A_{Eigen::Matrix<double, kParams, kParams>::Zero()};
  Eigen::Matrix<double, kParams, 1> b_{Eigen::Matrix<double, kParams, 1>::Zero()};
  Eigen::Matrix<double, kParams, 1> phi_{Eigen::Matrix<double, kParams, 1>::Zero()};

  Eigen::VectorXd r_buf_;    ///< r gathered into an Eigen vector (nv)
  Eigen::VectorXd fit_buf_;  ///< Y₄φ̂ (nv), for the reconstruction diagnostic
};

}  // namespace rtc::estimation

#endif  // RTC_CONTROLLERS_ESTIMATION_INERTIAL_ESTIMATOR_HPP_
