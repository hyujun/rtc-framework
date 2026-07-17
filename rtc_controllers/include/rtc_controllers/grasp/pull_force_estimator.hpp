#ifndef RTC_CONTROLLERS_GRASP_PULL_FORCE_ESTIMATOR_HPP_
#define RTC_CONTROLLERS_GRASP_PULL_FORCE_ESTIMATOR_HPP_

#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_controllers/grasp/grasp_state.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <Eigen/Core>

#include <array>
#include <span>

namespace rtc::grasp {

/// Upper bound on pull-estimation contacts — mirrors kMaxGraspFingers so all
/// per-contact storage stays fixed-size (RT no-alloc). Actual contact count is
/// a runtime value supplied to Init().
inline constexpr int kMaxPullContacts = kMaxGraspFingers;

// ── Per-contact configuration (Init 시 고정, non-RT) ─────────────────────────

/// Per-fingertip calibration + contact model for in-plane pull estimation.
///
/// Force input contract (#167, SSoT: rtc_msgs/msg/FingertipSensor.msg):
/// fingertip *link* frame, [N], env-on-fingertip sign. `force_sign = -1`
/// (default) converts to the finger-on-object forces the estimator sums.
/// For the current UDP-hand/MuJoCo backends the sensor extrinsic is baked
/// into the publisher, so `force_calibration = I`, `force_bias = 0` defaults
/// hold; the fields exist for future sensors whose payload is not yet
/// link-frame-calibrated.
struct PullContactConfig {
  /// C_i — axis permutation / scale / cross-axis calibration (link frame).
  Eigen::Matrix3d force_calibration{Eigen::Matrix3d::Identity()};
  /// b_i — additive bias removed before calibration [N].
  Eigen::Vector3d force_bias{Eigen::Vector3d::Zero()};
  /// Multiplied after calibration. -1 converts the wire contract
  /// (env-on-fingertip) to finger-on-object.
  double force_sign{-1.0};
  /// mu_i — Coulomb friction coefficient for slip-ratio diagnostics.
  double friction_coeff{0.7};
  /// Contact hysteresis on normal force f_n [N]: ON above on-threshold,
  /// OFF below off-threshold (on > off > 0 enforced at Init).
  double contact_on_threshold{0.5};
  double contact_off_threshold{0.2};
  /// Per-axis |raw force| at/above this is treated as saturated: the contact
  /// is gated out of the sum and PullEstimate::any_saturated is set [N].
  double force_saturation{50.0};
  /// Required-role mask: if true this contact must be active (post-hysteresis)
  /// for the estimate to be valid — e.g. thumb in any pinch mode. Guards the
  /// "index+middle without thumb still counts 2 contacts" false-valid.
  bool required{false};
};

// ── Estimator parameters ─────────────────────────────────────────────────────

struct PullEstimatorParams {
  /// Low-pass (Bessel biquad) on the in-plane estimate.
  double filter_cutoff_hz{5.0};
  double sample_rate_hz{500.0};
  /// Minimum active contacts for a valid estimate (>= 1).
  int min_valid_contacts{2};
  /// Bounded decay time constant applied to the published estimate on invalid
  /// ticks (contact loss / degenerate normal) instead of a hard zero [s].
  double decay_time_constant_s{0.1};
  /// max friction utilization >= this ⇒ slip_risk.
  double slip_risk_threshold{1.0};
  /// delta-theta for the alignment-leakage upper bound
  /// sum_i |f_n,i| * sin(delta_theta) [rad].
  double alignment_error_rad{0.035};
  /// m·g of the object expressed in the common reference frame [N]. Zero
  /// (default) when using baseline subtraction or a gravity-orthogonal plane.
  Eigen::Vector3d gravity_force{Eigen::Vector3d::Zero()};
};

// ── Per-tick input (RT) ──────────────────────────────────────────────────────

/// One fingertip's measurement for this tick. Same order as Init() configs.
struct PullContactInput {
  /// R_i(q) — fingertip link frame → common reference frame (FK rotation).
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  /// Raw force, link frame, env-on-fingertip sign (wire contract) [N].
  Eigen::Vector3d force{Eigen::Vector3d::Zero()};
  /// n_i — outward object normal at the contact, in the *common reference*
  /// frame (already FK-resolved by the caller), pointing from the object
  /// surface into the finger. Compression then has f_n = -n·f_obj > 0. Drives
  /// the contact hysteresis and slip/leakage diagnostics only — NOT the pull
  /// estimate, which projects on the plane normal. For a pinch the caller sets
  /// it from FK geometry (±plane normal), so it tracks the grasp axis rather
  /// than a body-fixed fingertip axis — correct for hemispherical tips whose
  /// contact point migrates. Degenerate/non-finite ⇒ this contact is skipped.
  Eigen::Vector3d contact_normal{Eigen::Vector3d::Zero()};
  /// Controller-side gate: finite + fresh sensor. false ⇒ this contact is
  /// skipped and its hysteresis state resets (invalid sensor policy).
  bool valid{false};
};

// ── Output ───────────────────────────────────────────────────────────────────

/// Estimate + diagnostics for one tick. Not a SeqLock POD — Eigen members are
/// not trivially copyable; controllers mirror what they publish into their
/// own POD (SeqLock pattern) off this struct.
struct PullEstimate {
  /// -P∥(Σ f_obj + m·g) (baseline-subtracted when armed), pre-filter [N].
  Eigen::Vector3d force_raw{Eigen::Vector3d::Zero()};
  /// Low-passed force_raw; decayed on invalid ticks [N].
  Eigen::Vector3d force_filtered{Eigen::Vector3d::Zero()};
  /// B^T * force_filtered — 2D coordinates in the plane basis [N].
  Eigen::Vector2d force_inplane{Eigen::Vector2d::Zero()};
  /// |force_filtered| [N].
  double magnitude{0.0};
  /// d^T * force_filtered when a pull direction is set, else 0 [N].
  double directional{0.0};
  /// max_i |f_t,i| / (mu_i f_n,i + eps) over active contacts.
  double max_friction_utilization{0.0};
  /// sum_i |f_n,i| * sin(alignment_error_rad) — grip→in-plane leakage bound [N].
  double leakage_bound{0.0};
  int valid_contact_count{0};
  /// All gates passed: >= min_valid_contacts, all required roles active,
  /// finite plane normal.
  bool valid{false};
  bool slip_risk{false};
  /// Some contact was gated out for per-axis raw saturation this tick.
  bool any_saturated{false};
  /// A baseline snapshot is currently being subtracted.
  bool baseline_applied{false};
};

// ── Estimator ────────────────────────────────────────────────────────────────

/// Quasi-static in-plane external pull-force estimator (#167).
///
///   F̂ = -P∥ ( Σ_i sign_i · R_i · C_i (f_i^raw - b_i) + m·g ),  P∥ = I - n nᵀ
///
/// Robot-agnostic math core: consumes per-contact FK rotations and raw forces
/// the caller gathers — no Pinocchio, no ROS, no robot facts (ARCH-1). The
/// plane normal is a per-tick input so callers choose its source (fixed object
/// pose / pinch geometry / vision). Internal grip (opposing squeeze) cancels
/// in the sum automatically; out-of-plane wrench components are *not*
/// observable from a flat pinch and are deliberately not produced.
///
/// RT contract: Init() is non-RT and may throw; Update() is noexcept,
/// heap-free, fixed-size only. Observability/condition diagnostics that need
/// SVD live off-RT (issue #167) — not here.
class PullForceEstimator {
 public:
  PullForceEstimator() = default;

  /// Validate configs/params and precompute filter coefficients (non-RT).
  /// Throws std::invalid_argument on: empty/oversized configs, on <= off or
  /// off <= 0 hysteresis, non-positive saturation/friction/decay/min-contacts,
  /// or bad filter rates.
  void Init(std::span<const PullContactConfig> configs, const PullEstimatorParams& params);

  /// Per-tick update on the RT thread. `inputs` follows Init() config order
  /// (extra entries ignored, missing treated invalid). `plane_normal` is the
  /// object-plane normal in the common reference frame (normalized here;
  /// degenerate ⇒ invalid tick with bounded decay). Returns the stored
  /// estimate (also via estimate()).
  const PullEstimate& Update(std::span<const PullContactInput> inputs,
                             const Eigen::Vector3d& plane_normal, double dt) noexcept;

  /// Capture the in-plane sum as baseline on the next *valid* tick and
  /// subtract it thereafter — removes in-plane gravity and constant
  /// calibration residual. Re-arm after large object re-orientation.
  void ArmBaseline() noexcept { baseline_armed_ = true; }

  void ClearBaseline() noexcept {
    baseline_armed_ = false;
    baseline_captured_ = false;
    baseline_.setZero();
  }

  /// Optional known pull direction (reference frame) for the directional
  /// scalar output — more noise-robust than |F̂| in gauge experiments.
  /// Normalized here; zero/degenerate disables the output.
  void SetPullDirection(const Eigen::Vector3d& direction) noexcept;

  [[nodiscard]] const PullEstimate& estimate() const noexcept { return estimate_; }

  [[nodiscard]] int num_contacts() const noexcept { return num_contacts_; }

  /// Post-hysteresis contact state (logging / tests). OOB ⇒ false.
  [[nodiscard]] bool contact_active(int i) const noexcept {
    return i >= 0 && i < num_contacts_ && contact_active_[static_cast<std::size_t>(i)];
  }

 private:
  /// Deterministic in-plane basis B = [e_x e_y] for a unit normal.
  static void MakePlaneBasis(const Eigen::Vector3d& n, Eigen::Vector3d& e_x,
                             Eigen::Vector3d& e_y) noexcept;

  // ── Fixed at Init ──
  std::array<PullContactConfig, kMaxPullContacts> configs_{};
  PullEstimatorParams params_{};
  int num_contacts_{0};
  bool initialized_{false};
  double sin_alignment_error_{0.0};

  // ── RT state ──
  BesselFilterN<3> filter_;
  std::array<bool, kMaxPullContacts> contact_active_{};
  Eigen::Vector3d filtered_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d baseline_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pull_direction_{Eigen::Vector3d::Zero()};
  bool pull_direction_set_{false};
  bool baseline_armed_{false};
  bool baseline_captured_{false};
  PullEstimate estimate_{};
};

/// Mirror a PullEstimate into the SeqLock-compatible POD embedded in the
/// controller-owned state PODs (GraspStateData / WbcStateData). RT-safe:
/// noexcept, fixed-size copies only.
void FillPullEstimateData(const PullEstimate& in, PullEstimateData& out) noexcept;

}  // namespace rtc::grasp

#endif  // RTC_CONTROLLERS_GRASP_PULL_FORCE_ESTIMATOR_HPP_
