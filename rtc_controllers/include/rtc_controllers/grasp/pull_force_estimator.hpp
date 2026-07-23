#ifndef RTC_CONTROLLERS_GRASP_PULL_FORCE_ESTIMATOR_HPP_
#define RTC_CONTROLLERS_GRASP_PULL_FORCE_ESTIMATOR_HPP_

#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_controllers/grasp/grasp_state.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <span>

namespace rtc::grasp {

/// Upper bound on pull-estimation contacts — mirrors kMaxGraspFingers so all
/// per-contact storage stays fixed-size (RT no-alloc). Actual contact count is
/// a runtime value supplied to Init().
inline constexpr int kMaxPullContacts = kMaxGraspFingers;

static_assert(kMaxPullContacts <= 8,
              "PullEstimate::contact_mask / touch_mask are uint8 bitmasks over contacts");

// ── Self-description enums (#234 P-5 / P-11) ────────────────────────────────

/// Which rule produced the in-plane basis on a given tick. Published so a
/// consumer can tell a physically named axis pair from a fallback one:
/// force_inplane means "(along the configured reference, then n x that)" only
/// under kReference. Values are wire constants — never renumber.
enum class PullBasisSource : std::uint8_t {
  kNone = 0,       ///< No basis this tick (invalid / degenerate normal).
  kReference = 1,  ///< inplane_x_reference projected into the plane.
  kCarry = 2,      ///< Previous e_x re-projected (reference collapsed).
  kSeed = 3,       ///< World-axis seed (no usable carry either).
};

/// Why a tick was not valid. `kNone` iff PullEstimate::valid. Reported in
/// priority order (the first gate that failed), so a consumer distinguishes
/// "the geometry is degenerate" from "the thumb dropped out" from "not enough
/// tips" — the three collapse to the same valid=false today, which is what
/// makes a joint-vs-WBC trace comparison unreadable (#234 P-11).
/// Values are wire constants — never renumber.
enum class PullInvalidReason : std::uint8_t {
  kNone = 0,                    ///< Valid tick.
  kNotInitialized = 1,          ///< Update() before a successful Init().
  kDegenerateNormal = 2,        ///< plane_normal zero / non-finite this tick.
  kRequiredContactMissing = 3,  ///< A `required` contact was not active.
  kInsufficientContacts = 4,    ///< Active count below min_valid_contacts.
};

// ── Per-contact configuration (Init 시 고정, non-RT) ─────────────────────────

/// Per-fingertip calibration + contact model for in-plane pull estimation.
///
/// Force input contract (#167): fingertip *link* frame, [N], **finger-on-object**
/// sign — the force the fingertip exerts on the grasped object, which is what
/// the estimator sums. Every fingertip *sensor* publisher reports that sign
/// (rtc_msgs/msg/FingertipSensor.msg: proto-1b firmware, the 1a ONNX path), so
/// `force_sign = +1` (default) passes the input through unchanged.
///
/// Verified on hardware 2026-07-22 (thumb+middle plate pinch, p1b): with
/// R_link(q) applied, the thumb's force pointed along the thumb→middle pinch
/// axis and the middle's back along it (145.8 deg apart) — each tip pushing
/// into the object. Under the previous `-1` default this inverted every gate
/// (f_n = -n.f_obj < 0), so a solid two-finger grasp published an all-zero
/// estimate. `force_sign = -1.0` inverts an env-on-fingertip source instead —
/// which is why it is per contact rather than global: a robot without fingertip
/// sensors may be fed from a lane with the opposite convention (e.g. a
/// simulator's contact wrench, which follows the ROS wrench convention of
/// reporting the load *on* the link), and pins -1.0 in its own config.
///
/// For the current UDP-hand/MuJoCo backends the sensor extrinsic is baked
/// into the publisher, so `force_calibration = I`, `force_bias = 0` defaults
/// hold; the fields exist for future sensors whose payload is not yet
/// link-frame-calibrated.
struct PullContactConfig {
  /// C_i — axis permutation / scale / cross-axis calibration (link frame).
  Eigen::Matrix3d force_calibration{Eigen::Matrix3d::Identity()};
  /// b_i — additive bias removed before calibration [N].
  Eigen::Vector3d force_bias{Eigen::Vector3d::Zero()};
  /// Multiplied after calibration. +1 (default) = the input is already
  /// finger-on-object; -1 converts an env-on-fingertip publisher
  /// (rtc_msgs/FingertipSensor.msg wire sign) into it.
  double force_sign{1.0};
  /// mu_i — Coulomb friction coefficient for slip-ratio diagnostics.
  double friction_coeff{0.7};
  /// Contact hysteresis on normal force f_n [N]: ON above on-threshold,
  /// OFF below off-threshold (on > off > 0 enforced at Init).
  ///
  /// These also drive the *touch* hysteresis (on |f_obj| instead of f_n — see
  /// PullForceEstimator::touch_active) unless the touch pair below is set.
  /// Sharing them makes the touch set a superset of the contact set, since
  /// |f_obj| >= |f_n|: "this tip is pressing on something" (axis-independent)
  /// vs "this tip is pressing along the plane normal" (the stricter gate on
  /// the sum).
  double contact_on_threshold{0.5};
  double contact_off_threshold{0.2};
  /// Optional independent hysteresis for the *touch* gate on |f_obj| [N].
  /// Zero (default) = unset ⇒ the contact pair above is reused, which is the
  /// historical behaviour and preserves `touch ⊇ contact` exactly.
  ///
  /// Set them when the two gates need different sensitivity (#234 P-16): touch
  /// selects the pinch axis, contact admits a tip into the force sum, and on a
  /// hand with large shear a tip can read as "touching" — dragging the axis
  /// around — while contributing nothing to the sum. Raising only the touch
  /// pair keeps such a tip out of the axis.
  ///
  /// Init enforces on > off > 0 on the pair, and that both or neither are set.
  /// Note that raising touch above contact can invert `touch ⊇ contact`; the
  /// wiring only uses touch to *select* the axis, so this costs axis
  /// sensitivity, not correctness of the sum.
  double touch_on_threshold{0.0};
  double touch_off_threshold{0.0};
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
  /// Reference direction for the in-plane x axis, common reference frame.
  /// force_inplane[0] is then the component along this direction projected into
  /// the plane, and [1] the remaining one (e_y = n x e_x) — so with the default
  /// (+Z base = anti-gravity for an upright base) the pair reads as
  /// (vertical, horizontal) instead of two axes that only a plot can interpret.
  /// Set to the actual anti-gravity direction if the robot base is not upright.
  /// Zero disables the reference and falls back to basis continuity carry.
  Eigen::Vector3d inplane_x_reference{Eigen::Vector3d::UnitZ()};
};

// ── Per-tick input (RT) ──────────────────────────────────────────────────────

/// One fingertip's measurement for this tick. Same order as Init() configs.
struct PullContactInput {
  /// R_i(q) — fingertip link frame → common reference frame (FK rotation).
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  /// Raw force, link frame, finger-on-object sign [N] (see
  /// PullContactConfig::force_sign for env-on-fingertip publishers).
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
  /// B^T * force_filtered — 2D coordinates in the plane basis [N]. Only
  /// interpretable together with `plane_normal` + `basis_x` + `basis_source`
  /// below, which name the basis this tick actually used (#234 P-5).
  Eigen::Vector2d force_inplane{Eigen::Vector2d::Zero()};
  /// n̂ used this tick, common reference frame. Zero only when there was no
  /// usable normal at all (invalid_reason == kDegenerateNormal /
  /// kNotInitialized); an otherwise-invalid tick still reports the plane it
  /// gated against.
  Eigen::Vector3d plane_normal{Eigen::Vector3d::Zero()};
  /// e_x of the in-plane basis, common reference frame; e_y = n̂ x e_x, so the
  /// pair (plane_normal, basis_x) determines B completely. Zero when no basis
  /// was produced. Together with force_inplane this makes the 2-D pair
  /// invertible back to the 3-D in-plane force without knowing the estimator's
  /// config or its per-tick fallback branch.
  Eigen::Vector3d basis_x{Eigen::Vector3d::Zero()};
  /// Which MakePlaneBasis branch produced `basis_x`.
  PullBasisSource basis_source{PullBasisSource::kNone};
  /// First gate that failed; kNone iff `valid`.
  PullInvalidReason invalid_reason{PullInvalidReason::kNone};
  /// Bit k = contact k passed the *contact* hysteresis this tick, i.e. it is
  /// in the force sum. This is the set `valid_contact_count` counts — the
  /// wiring's opposing_mask is built from touch_mask instead and is a
  /// different set (#234 P-12).
  std::uint8_t contact_mask{0};
  /// Bit k = contact k passed the *touch* hysteresis this tick (|f_obj|,
  /// axis-independent). Includes the thumb, unlike the wiring's opposing_mask,
  /// so "no tip touching" and "thumb geometry invalid" are distinguishable
  /// (#234 P-14).
  std::uint8_t touch_mask{0};
  /// |force_filtered| [N].
  double magnitude{0.0};
  /// d^T * force_filtered when a pull direction is set, else 0 [N].
  double directional{0.0};
  /// max_i |f_t,i| / (mu_i f_n,i + eps) over active contacts.
  double max_friction_utilization{0.0};
  /// sum_i |f_n,i| * sin(alignment_error_rad) — grip→in-plane leakage bound [N].
  double leakage_bound{0.0};
  int valid_contact_count{0};
  /// **Pull-vector validity only** (#234 P-7): >= min_valid_contacts, all
  /// required roles active, finite plane normal. It gates force_raw /
  /// force_filtered / force_inplane / magnitude / directional — the quantities
  /// that need the plane. It does NOT gate the per-contact friction
  /// diagnostics below, which are physically well defined on any active
  /// contact whether or not an in-plane vector could be formed.
  bool valid{false};
  /// max_friction_utilization >= slip_risk_threshold on some active contact.
  /// Evaluated outside the `valid` gate — a required-role dropout must not
  /// silence a tip that is visibly sliding. False whenever no contact is
  /// active, since the utilization is then vacuously zero.
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
  /// off <= 0 hysteresis (contact pair, and the touch pair when set), a
  /// half-set touch pair, non-positive saturation/friction/decay/min-contacts,
  /// `min_valid_contacts` above the configured contact count (#234 P-6 — it
  /// would otherwise configure cleanly and run permanently invalid),
  /// `force_sign` outside {+1, -1}, any non-finite scalar or vector, or bad
  /// filter rates.
  void Init(std::span<const PullContactConfig> configs, const PullEstimatorParams& params);

  /// Per-tick update on the RT thread. `inputs` follows Init() config order
  /// (extra entries ignored, missing treated invalid). `plane_normal` is the
  /// object-plane normal in the common reference frame (normalized here;
  /// degenerate ⇒ invalid tick with bounded decay). Returns the stored
  /// estimate (also via estimate()).
  ///
  /// Invalid ticks decay the published force by exp(-dt / decay_time_constant)
  /// rather than zeroing it. `dt <= 0` is treated as a full collapse (factor 0,
  /// hard zero) — a non-advancing clock carries no information about how much
  /// the estimate should have decayed, and holding the last value instead would
  /// let a stalled caller publish a stale force indefinitely.
  ///
  /// The first valid tick after any invalid stretch reseeds the output filter
  /// with that tick's raw estimate (#234 P-4). Without it the filter would
  /// resume from the delay line it held before the gap and snap the output back
  /// to the pre-gap force in one sample, undoing the decay the invalid ticks
  /// published — the decay would apply on the way down but not on the way up.
  /// The cost is that the first sample after a gap is unfiltered; the filter
  /// resumes smoothing from there.
  const PullEstimate& Update(std::span<const PullContactInput> inputs,
                             const Eigen::Vector3d& plane_normal, double dt) noexcept;

  /// Capture the reference wrench as baseline on the next *valid* tick and
  /// subtract it thereafter — removes in-plane gravity and constant
  /// calibration residual. Re-arm after large object re-orientation.
  ///
  /// What is stored is the *unprojected* total (Σ f_obj + m·g), not its
  /// in-plane projection, so the subtraction is re-projected onto whatever
  /// plane the current tick uses. A grasp-shape change that rotates the plane
  /// therefore carries the baseline with it instead of leaving a constant
  /// residual in the old plane (the snapshot models a fixed 3-D force —
  /// gravity plus calibration offset — whose in-plane part is plane-dependent).
  void ArmBaseline() noexcept { baseline_armed_ = true; }

  void ClearBaseline() noexcept {
    baseline_armed_ = false;
    baseline_captured_ = false;
    baseline_ref_.setZero();
  }

  /// Drop every per-tick latch so a resumed control loop cannot inherit state
  /// from before the gap: contact/touch hysteresis, the output filter, the
  /// plane-basis continuity carry and the baseline. Init-time state (configs,
  /// params, filter coefficients) is kept — call this from on_activate, not
  /// ClearBaseline alone. noexcept, heap-free.
  void ResetRtState() noexcept;

  /// Optional known pull direction (reference frame) for the directional
  /// scalar output — more noise-robust than |F̂| in gauge experiments.
  /// Normalized here; zero/degenerate disables the output.
  void SetPullDirection(const Eigen::Vector3d& direction) noexcept;

  [[nodiscard]] const PullEstimate& estimate() const noexcept { return estimate_; }

  [[nodiscard]] int num_contacts() const noexcept { return num_contacts_; }

  /// Post-hysteresis contact state — f_n above this contact's hysteresis with
  /// the plane normal *this tick* supplied. OOB ⇒ false.
  ///
  /// Diagnostic only. Callers deriving the plane normal from contact topology
  /// must use touch_active() instead: contact_active depends on the normal, so
  /// feeding it back into the normal closes a loop the hysteresis cannot damp
  /// (the ON and OFF tests would be evaluated against different axes).
  [[nodiscard]] bool contact_active(int i) const noexcept {
    return i >= 0 && i < num_contacts_ && contact_active_[static_cast<std::size_t>(i)];
  }

  /// Post-hysteresis *touch* state: |f_obj| above this contact's hysteresis,
  /// evaluated on the last Update(). OOB ⇒ false.
  ///
  /// Independent of the plane normal by construction, so a caller may use it to
  /// choose the normal (which tips define the pinch geometry) without creating
  /// a feedback loop. False whenever the sensor was invalid or saturated, so it
  /// already subsumes the caller's own force-validity gate. Reflects the tick
  /// that just ran — a caller staging the next tick sees a one-tick lag, which
  /// costs one invalid tick at grasp onset and cannot deadlock.
  [[nodiscard]] bool touch_active(int i) const noexcept {
    return i >= 0 && i < num_contacts_ && touch_active_[static_cast<std::size_t>(i)];
  }

 private:
  /// In-plane basis B = [e_x e_y] for a unit normal. e_x is the configured
  /// inplane_x_reference projected into the plane, which makes the reported 2-D
  /// coordinates physically named (anti-gravity, then n x e_x) and a pure
  /// function of n — no path dependence. When the plane turns edge-on to that
  /// reference the projection collapses, so the basis falls back to continuity
  /// carry (previous e_x re-projected) and then to a world-axis seed.
  ///
  /// Returns which of those three branches produced e_x, so the tick can say
  /// on the wire whether force_inplane[0] really is the named axis
  /// (kReference) or a path-dependent fallback. kNone ⇒ e_x/e_y are zero.
  [[nodiscard]] PullBasisSource MakePlaneBasis(const Eigen::Vector3d& n, Eigen::Vector3d& e_x,
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
  std::array<bool, kMaxPullContacts> touch_active_{};
  Eigen::Vector3d filtered_{Eigen::Vector3d::Zero()};
  /// Unprojected (Σ f_obj + m·g) captured when the baseline armed; re-projected
  /// onto the current plane every tick rather than stored pre-projected.
  Eigen::Vector3d baseline_ref_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pull_direction_{Eigen::Vector3d::Zero()};
  /// Previous in-plane e_x, carried forward to keep the basis continuous.
  Eigen::Vector3d prev_e_x_{Eigen::Vector3d::Zero()};
  bool prev_e_x_valid_{false};
  /// Set by every invalid tick, cleared by the valid tick that reseeds the
  /// output filter (#234 P-4). While it is set, `filtered_` is being driven by
  /// the bounded decay and the filter's delay line no longer describes it — so
  /// the next valid sample must seed the filter rather than continue from a
  /// tail the world has moved on from.
  bool filter_reseed_pending_{false};
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
