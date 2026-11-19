#ifndef RTC_CONTROLLERS_ESTIMATION_CONTACT_FSM_HPP_
#define RTC_CONTROLLERS_ESTIMATION_CONTACT_FSM_HPP_

// ── Per-slot contact state machine + the tick sequencer that drives the
//    contact-aided InEKF core with it ───────────────────────────────────────
//
// Robot-agnostic. No ROS, no model, no planner: the planned contact bits, the
// IMU-frame forward kinematics (r_i, J_i) and the joint-rate term of each
// contact point arrive as arguments. A slot is one contact point 0 … n−1
// (n ≤ kInekfMaxContacts); nothing here knows what a foot, a side or a floor is.
//
// ### Units, frames, symbols
//
//   W, I         world and IMU frame, as in contact_inekf.hpp
//   R̂, v̂, ω̂      the core's R_WI, IMU velocity in W [m/s] and ω_m − b̂_g in I
//                [rad/s], read right after this tick's Propagate()
//   r_i, J_i     contact point i in I [m] and ∂r_i/∂q [m/rad]
//   ṙ_i          joint-rate term J_i q̇ in I [m/s]
//   P_i          planned contact bit of slot i (bit i of the planned mask)
//   k            sampling periods elapsed since the previous sample (≥ 1);
//                the core propagates the NOMINAL step k·T, T = Config::period [s]
//   m_i²         the core's innovation NIS of slot i, computed on the prior P
//                (ContactUpdateReport::mahalanobis2)
//
// ### Predicted contact-point velocity and the speed tested
//
//   u_i = v̂ + R̂ (ω̂ × r_i + ṙ_i)                                   [m/s, in W]
//
// is the world velocity the filter predicts for the body-fixed point r_i. A
// point that holds on the ground has u_i ≈ 0. The speed a slot is tested with
// is, decided once per tick from the modes the tick starts with:
//
//   some slot in stance or unknown:   s_i = ‖u_i‖
//   none (no confirmed anchor):       s_i = max_{j ≠ i, P_j, u_j finite} ‖u_i − u_j‖,
//                                     s_i = 0 when no such j exists
//
// Without a confirmed anchor v̂ is dead reckoning; its error is common to every
// u_i and cancels in the difference, so a filter that drifted while nothing
// held the ground can still re-acquire. A single planned slot is exempt.
// u_i, r_i, J_i or ṙ_i non-finite makes the slot INVALID for that tick.
//
// ### Streaks and dwells
//
// Each slot keeps streaks of consecutive samples; a streak adds k ticks per
// sample and is cleared by a sample that breaks it:
//
//   R   P_i off                      L   s_i ≤ v_enter        H   s_i > v_slip
//   N+  last report high             N−  last report low      A   ticks since insertion
//                                                                 (every sample, valid or not)
//
// where a report is HIGH if it was gated, m² > χ²_hi or m² not finite, LOW if used with
// m² ≤ χ²_lo, and NEUTRAL if used in between (N+ and N− both clear). A tick
// without a report (inactive, not requested, inserted this step, a refused
// update) leaves N+ and N− as they are. A dwell of d seconds is the tick count
// n = ⌈d / T⌉ ≥ 1 (Init, with a 1e-12 relative allowance so d = mT gives m),
// and a streak MEETS n when it spans ≥ n ticks over ≥ min(n, 2) samples — one
// late sample of k ≥ n ticks alone never satisfies a dwell of two ticks or more.
//
// Two thresholds per signal give the hysteresis: v_enter < v_slip [m/s] and
// χ²_lo < χ²_hi (Init refuses equal or inverted pairs).
//
// ### Transition table
//
// Evaluated once per tick in PreUpdate, rows of a mode top to bottom, first
// match wins (priority: release > slip > NIS > promotion / recovery). Every
// transition clears all streaks of the slot and records one reason.
//
//   from       condition                                  to         core action        reason
//   swing      P_i ∧ L ≥ n_enter                          candidate  InsertContact      kTouchdown
//   candidate  R ≥ n_release                              swing      RemoveContact      kReleased
//   candidate  H ≥ n_slip                                 swing      RemoveContact kRejectedSpeed
//   candidate  last report high                           swing      RemoveContact kRejectedNis
//   candidate  A ≥ max(n_confirm, 2) ∧ L ≥ n_confirm      stance     —                  kConfirmed
//              ∧ last report low
//   stance     R ≥ n_release                              swing      RemoveContact      kReleased
//   stance     H ≥ n_slip                                 slip       RemoveContact      kSlip
//   stance     N+ ≥ n_nis                                 unknown    noise scale s_u    kNisHigh
//   unknown    R ≥ n_release                              swing      RemoveContact      kReleased
//   unknown    H ≥ n_slip                                 slip       RemoveContact      kSlip
//   unknown    N− ≥ n_recover ∧ L ≥ n_recover             stance     noise scale 1      kRecovered
//   slip       R ≥ n_release                              swing      —                  kReleased
//   slip       P_i ∧ L ≥ n_enter                          candidate  InsertContact      kTouchdown
//   candidate  (the core refused the insertion)           swing      — kInsertRefused
//
// The slot's request to the update is  use_i = valid_i ∧ mode ∈ {candidate,
// stance, unknown}  (the modes that hold an anchor). An INVALID slot keeps its
// mode, clears L, H, N+ and N−, still advances R and is tested against the
// release rows only.
//
// Consequences of the core contract this table is built on:
//  * A candidate takes part in the update from the tick after its insertion,
//    at noise scale 1 (the insertion tick reports kInsertedThisStep).
//  * The first report after an insertion comes from the next tick's update and
//    is read the tick after that: candidate → stance is no earlier than
//    insertion + 2 ticks, hence max(n_confirm, 2).
//  * RemoveContact returns the core's noise scale to 1, so the FSM never
//    remembers a scale: it sets s_u on every entry to unknown and 1 on recovery.
//
// ### Evidence (diagnostic only — it decides no transition)
//
//   g(s) = clamp((v_slip − s) / (v_slip − v_enter), 0, 1)
//   h    = 1 (no report) · 0 (gated) · clamp((χ²_hi − m²) / (χ²_hi − χ²_lo), 0, 1) (used)
//   q_i  = P_i · g(s_i) · h,     c_i ← c_i + (1 − exp(−kT / τ_e)) (q_i − c_i),   c_i ∈ [0, 1]
//
// c is a smoothed score, NOT a calibrated probability. An invalid slot holds c.
//
// ### Tick order (ContactInekfSequencer::Step)
//
//   1. Propagate(gyro, accel, kT). Refused → return kPropagateRefused: FSM, core
//      and counters keep their values, this tick reports no action and no
//      update. kT above the core's kMaxPropagationStep is always refused — a
//      gap that long is the caller's to handle (Reset).
//   2. u_i from the propagated state (above).
//   3. PreUpdate: every transition and core action of this tick, decided from
//      the PREVIOUS tick's reports only.
//   4. Core actions per slot: RemoveContact, InsertContact (refused → the slot
//      reverts to swing, kInsertRefused), SetContactNoiseScale.
//   5. Check anchor-holding mode ⇔ core contact_active for every slot. A
//      mismatch, or a refused Remove / scale, returns kStateMismatch without an
//      update; recovery is Reset() and a new Initialize().
//   6. UpdateContacts with use_i. Refused → no report this tick, kUpdateRefused.
//   7. PostUpdate stores the reports for the NEXT tick's PreUpdate. Nothing in
//      this tick's decision can depend on this tick's update.
//
// Outputs for the caller's recovery policy: ticks since some slot was last in
// stance or unknown (0 while one is), and consecutive ticks whose update gated
// a slot and used none (a tick that used a slot clears it; a tick with neither
// leaves it). Both add k per sample and saturate.
//
// ### Declared limits
//
//  * The tested point is body-fixed: rolling or curved contact moves it and
//    reads as slip.
//  * A slow slip below v_slip during stance is left to the accumulated NIS
//    (on the order of 100 ms at 0.1 m/s) and may not be detected at all.
//  * A slip below v_slip but within measurement noise of it (ṙ noise plus v̂
//    error) can still read as slip: the hysteresis margin is v_slip − speed,
//    not v_slip − v_enter.
//  * In single support v̂ absorbs part of a slip, so it is under-detected.
//  * Unknown re-fits the anchor under an inflated random walk; its NIS
//    recovery test has little power, so recovery is decided by speed + dwell,
//    and a creeping point can cycle stance → unknown → stance.
//  * Without a confirmed anchor a planned slot that is not yet down (plan
//    ahead of contact) makes the others' spread large and holds back their
//    re-acquisition until it lands or its plan bit drops. Conversely, points
//    that move TOGETHER with the plan ahead of contact (a lowered or landing
//    body) show no spread and anchor in the air; above v_slip they then cycle
//    stance ↔ slip after touchdown.
//  * A measurement outlier the filter model does not cover (an acceleration
//    step that is not motion) and a long stretch with no anchor are filter-level
//    failures: the outputs above are the caller's cue to Reset().
//
// ### RT contract
//
// Init() is non-RT and throws std::invalid_argument. Every other member is
// noexcept, allocation-free and fixed-capacity.
#include "rtc_controllers/estimation/contact_inekf.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <span>

namespace rtc::estimation {

/// Estimated mode of one contact slot.
enum class ContactMode : std::uint8_t {
  kSwing = 0,      ///< No anchor; not in contact.
  kCandidate = 1,  ///< Anchor inserted, waiting for confirmation.
  kStance = 2,     ///< Confirmed contact.
  kSlip = 3,       ///< Contact point moving: anchor dropped, waiting to re-acquire.
  kUnknown = 4,    ///< NIS out of family: anchor kept under an inflated random walk.
};

/// Why a slot made its last transition.
enum class ContactReason : std::uint8_t {
  kNone = 0,           ///< No transition since Init / Reset.
  kTouchdown = 1,      ///< Planned and slow for the enter dwell.
  kConfirmed = 2,      ///< Slow for the confirm dwell with a low report.
  kReleased = 3,       ///< Planned off for the release dwell.
  kRejectedSpeed = 4,  ///< Candidate fast for the slip dwell.
  kRejectedNis = 5,    ///< Candidate's report high.
  kSlip = 6,           ///< Stance / unknown fast for the slip dwell.
  kNisHigh = 7,        ///< Stance's reports high for the NIS dwell.
  kRecovered = 8,      ///< Unknown low and slow for the recover dwell.
  kInsertRefused = 9,  ///< The core refused the insertion.
};

/// Outcome of a FSM / sequencer call. Anything but kOk is described per member.
enum class ContactFsmStatus : std::uint8_t {
  kOk = 0,
  kNotConfigured = 1,     ///< Before a successful Init().
  kNotInitialized = 2,    ///< Core not initialised (before Initialize(), after Reset()).
  kBadPeriods = 3,        ///< Step count k < 1.
  kSizeMismatch = 4,      ///< An input span is not num_slots() long.
  kPropagateRefused = 5,  ///< The core refused Propagate(); nothing changed.
  kUpdateRefused = 6,     ///< The core refused UpdateContacts(); no report this tick.
  kStateMismatch = 7,     ///< FSM anchors and core slots disagree; Reset() to recover.
};

/// Pure decision: planned bits, contact-point velocities and the previous
/// reports in, per-slot core actions out. Owns no filter.
class ContactFsm {
 public:
  /// Thresholds and times. Configuration, never robot constants: no field has
  /// a default, every one must be finite and > 0.
  struct Config {
    double period{0.0};                  ///< T [s], one sampling period
    double enter_speed{0.0};             ///< v_enter [m/s]
    double slip_speed{0.0};              ///< v_slip [m/s], > v_enter
    double nis_low{0.0};                 ///< χ²_lo, NIS recovery threshold
    double nis_high{0.0};                ///< χ²_hi, NIS exit threshold, > χ²_lo
    double enter_dwell{0.0};             ///< [s] swing / slip → candidate
    double confirm_dwell{0.0};           ///< [s] candidate → stance
    double release_dwell{0.0};           ///< [s] planned off → swing
    double slip_dwell{0.0};              ///< [s] fast → slip / reject
    double nis_dwell{0.0};               ///< [s] high NIS → unknown
    double recover_dwell{0.0};           ///< [s] unknown → stance
    double evidence_time_constant{0.0};  ///< τ_e [s]
    double unknown_noise_scale{0.0};     ///< s_u ≥ 1, anchor random-walk scale in unknown
  };

  /// What the sequencer does to the core for one slot this tick.
  struct SlotAction {
    bool insert{false};     ///< InsertContact
    bool remove{false};     ///< RemoveContact
    bool set_scale{false};  ///< SetContactNoiseScale(noise_scale)
    double noise_scale{1.0};
    bool use{false};  ///< request the slot in UpdateContacts
  };

  /// Validate and size for `num_slots` slots (non-RT); leaves every slot in swing.
  /// @throws std::invalid_argument on a field non-finite or ≤ 0, v_enter ≥ v_slip,
  ///         χ²_lo ≥ χ²_hi, s_u < 1, a period outside the core's step range, a
  ///         dwell above 10⁶ periods, or num_slots outside [1, kInekfMaxContacts].
  void Init(const Config& config, int num_slots);

  /// Every slot to swing, streaks / evidence / stored reports cleared.
  void Reset() noexcept;

  /// Decide this tick (table above). `contact_velocity[i]` is u_i in W (non-finite
  /// = invalid slot); `periods` is k. Writes `actions` (num_slots() long).
  /// Refusals (not configured, k < 1, size) change nothing.
  [[nodiscard]] ContactFsmStatus PreUpdate(int periods, std::uint32_t planned_mask,
                                           std::span<const Eigen::Vector3d> contact_velocity,
                                           std::span<SlotAction> actions) noexcept;

  /// The core refused this tick's insertion of `slot`: back to swing.
  void RevertInsert(int slot) noexcept;

  /// Store this tick's reports for the next PreUpdate. `report` is read only
  /// when `valid` (an update that returned kOk); otherwise every slot has none.
  void PostUpdate(std::span<const ContactUpdateReport> report, bool valid) noexcept;

  [[nodiscard]] bool configured() const noexcept { return num_slots_ > 0; }

  [[nodiscard]] int num_slots() const noexcept { return num_slots_; }

  /// T [s] (0 before Init).
  [[nodiscard]] double period() const noexcept { return config_.period; }

  /// Out-of-range slots read slot 0.
  [[nodiscard]] ContactMode mode(int slot) const noexcept { return At(slot).mode; }

  [[nodiscard]] ContactReason reason(int slot) const noexcept { return At(slot).reason; }

  [[nodiscard]] double evidence(int slot) const noexcept { return At(slot).evidence; }

  /// The speed s_i of the last PreUpdate [m/s] (NaN while invalid).
  [[nodiscard]] double tested_speed(int slot) const noexcept { return At(slot).speed; }

  /// True while the mode holds a core anchor (candidate, stance, unknown).
  [[nodiscard]] static constexpr bool HoldsAnchor(ContactMode m) noexcept {
    return m == ContactMode::kCandidate || m == ContactMode::kStance || m == ContactMode::kUnknown;
  }

  /// True while the mode is a confirmed anchor (stance, unknown).
  [[nodiscard]] static constexpr bool Confirmed(ContactMode m) noexcept {
    return m == ContactMode::kStance || m == ContactMode::kUnknown;
  }

 private:
  friend struct ContactFsmTestAccess;

  struct Streak {
    int ticks{0};
    int samples{0};
  };

  enum class Stored : std::uint8_t { kNone, kLow, kNeutral, kHigh };

  struct Slot {
    ContactMode mode{ContactMode::kSwing};
    ContactReason reason{ContactReason::kNone};
    Streak release;
    Streak low_speed;
    Streak high_speed;
    Streak nis_high;
    Streak nis_low;
    int age{0};  // ticks since insertion
    Stored last{Stored::kNone};
    double last_m2{0.0};
    double evidence{0.0};
    double speed{0.0};
  };

  [[nodiscard]] const Slot& At(int slot) const noexcept {
    return slots_[slot >= 0 && slot < num_slots_ ? static_cast<std::size_t>(slot) : 0U];
  }

  [[nodiscard]] static bool Meets(const Streak& s, int n) noexcept;
  static void ClearSignalStreaks(Slot& s) noexcept;
  void Enter(Slot& s, ContactMode to, ContactReason why) noexcept;
  void Decide(Slot& s, bool planned, bool valid, int k, SlotAction& a) noexcept;

  Config config_{};
  int num_slots_{0};

  struct Dwells {  // tick counts of the configured dwells
    int enter{1};
    int confirm{1};
    int release{1};
    int slip{1};
    int nis{1};
    int recover{1};
  };

  Dwells n_{};
  std::array<Slot, kInekfMaxContacts> slots_{};
};

/// One sample for ContactInekfSequencer::Step(). Spans are num_slots() long.
struct ContactStepInput {
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};    ///< raw gyro sample in I [rad/s]
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};   ///< raw specific force in I [m/s²]
  std::span<const ContactKinematics> kinematics;    ///< r_i, J_i in I (`use` ignored)
  std::span<const Eigen::Vector3d> joint_velocity;  ///< ṙ_i = J_i q̇ in I [m/s]
  std::uint32_t planned_mask{0};                    ///< bit i: slot i planned in contact
  int periods{1};  ///< k ≥ 1 sampling periods since the last sample
};

/// The core and the FSM composed in the tick order of the preamble. The one
/// place that order is implemented — online and replay callers both call Step().
class ContactInekfSequencer {
 public:
  /// Validate both configurations and size for `num_slots` slots (non-RT).
  /// Leaves the core uninitialised and every slot in swing.
  /// @throws std::invalid_argument from either Init, or when (s_u·σ_d)² is not finite.
  void Init(const ContactInekf::Config& core, const ContactFsm::Config& fsm, int num_slots);

  /// Core and FSM back to their post-Init state (configuration kept).
  void Reset() noexcept;

  /// ContactInekf::Initialize on the core; creates no anchor (every slot
  /// stays in swing until the table inserts it).
  [[nodiscard]] InekfStatus Initialize(const Eigen::Vector3d& mean_gyro,
                                       const Eigen::Vector3d& mean_accel, double window_duration,
                                       double yaw,
                                       std::span<const Eigen::Vector3d> stance_points) noexcept;

  /// One sample. On a refusal before PreUpdate nothing changes.
  [[nodiscard]] ContactFsmStatus Step(const ContactStepInput& in) noexcept;

  [[nodiscard]] const ContactInekf& core() const noexcept { return core_; }

  [[nodiscard]] const ContactFsm& fsm() const noexcept { return fsm_; }

  [[nodiscard]] int num_slots() const noexcept { return fsm_.num_slots(); }

  /// Status of the last refused core call (kOk when the last Step had none).
  [[nodiscard]] InekfStatus core_status() const noexcept { return core_status_; }

  /// This tick's report of `slot` (valid only when report_valid()).
  [[nodiscard]] const ContactUpdateReport& report(int slot) const noexcept;

  [[nodiscard]] bool report_valid() const noexcept { return report_valid_; }

  /// Core action applied to `slot` this tick.
  [[nodiscard]] const ContactFsm::SlotAction& action(int slot) const noexcept;

  /// Ticks since some slot was last in stance or unknown (0 while one is).
  [[nodiscard]] int ticks_without_confirmed_anchor() const noexcept { return no_anchor_ticks_; }

  /// Consecutive ticks whose update gated a slot and used none.
  [[nodiscard]] int consecutive_gated_ticks() const noexcept { return gated_ticks_; }

 private:
  friend struct ContactFsmTestAccess;

  [[nodiscard]] std::size_t Index(int slot) const noexcept {
    return slot >= 0 && slot < num_slots() ? static_cast<std::size_t>(slot) : 0U;
  }

  void FinishTick(int periods) noexcept;

  ContactInekf core_;
  ContactFsm fsm_;
  InekfStatus core_status_{InekfStatus::kOk};
  bool report_valid_{false};
  int no_anchor_ticks_{0};
  int gated_ticks_{0};
  std::array<ContactKinematics, kInekfMaxContacts> kin_{};
  std::array<ContactUpdateReport, kInekfMaxContacts> report_{};
  std::array<ContactFsm::SlotAction, kInekfMaxContacts> actions_{};
  std::array<Eigen::Vector3d, kInekfMaxContacts> velocity_{};
};

}  // namespace rtc::estimation

#endif  // RTC_CONTROLLERS_ESTIMATION_CONTACT_FSM_HPP_
