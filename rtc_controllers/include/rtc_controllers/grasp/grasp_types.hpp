#ifndef RTC_CONTROLLERS_GRASP_GRASP_TYPES_HPP_
#define RTC_CONTROLLERS_GRASP_GRASP_TYPES_HPP_

#include <array>
#include <cstdint>

namespace rtc::grasp {

/// Upper bound on the number of force-controlled fingers. The *actual* finger
/// count is a runtime value (GraspController::Init(std::span<const FingerConfig>),
/// GraspJointCommands::num_fingers). Kept as a compile-time cap so all
/// per-finger storage stays fixed-size (RT no-alloc); 8 mirrors the repo-wide
/// fingertip cap (kMaxFingertips / kMaxWbcFingertips / kMaxForceContacts).
static constexpr int kMaxGraspFingers = 8;
/// Upper bound on DoF per finger. Fingers may have *different* DoF counts
/// (e.g. thumb:4, index:3, middle:2, ring:1); the per-finger actual count
/// lives in FingerConfig::dof / GraspJointCommands::dof[f]. Loops iterate to
/// the runtime count, never this cap.
static constexpr int kMaxDoFPerFinger = 8;

// ── State Machine ────────────────────────────────────────────────────────────

enum class GraspPhase : uint8_t {
  kIdle,         // 대기 상태, s 유지
  kApproaching,  // 일정 속도로 closing (position ramp); s=1.0 에서 접촉을 계속 기다린다
  kContact,      // 전 finger 접촉 대기 + 안정화
  kForceControl,  // PI force regulation 활성화 + force ramp
  kHolding,       // Force 유지 + anomaly monitoring
  kReleasing      // 역방향 position ramp
};

// ── Per-finger configuration (생성 시 고정) ─────────────────────────────────

struct FingerConfig {
  int dof{0};  // actual DoF of this finger (≤ kMaxDoFPerFinger)
  std::array<double, kMaxDoFPerFinger> q_open{};   // fully open joint angles [rad]
  std::array<double, kMaxDoFPerFinger> q_close{};  // fully closed joint angles [rad]
};

// ── Grasp parameters ─────────────────────────────────────────────────────────

struct GraspParams {
  // PI gains
  double Kp_base{0.02};   // [1/(N*s)] proportional gain base
  double Ki_base{0.002};  // [1/(N*s^2)] integral gain base

  // Adaptive gain scheduling (stiffness EMA)
  double alpha_ema{0.95};  // stiffness EMA coefficient [0,1]
  // beta is the maximum-loop-gain handle, not a free knob. The closed loop
  // converges at lambda = K*Kp_base/(1 + beta*K), which saturates at
  // Kp_base/beta as the contact stiffness K grows, so beta alone sets the
  // fastest achievable settling: tau_min = beta/Kp_base. 0.03 (tau_min 1.5 s)
  // is the value that keeps K in [10, 200] inside a 10 s grasp budget and is
  // what this should become the moment adaptation is switched on.
  //
  // It stays at 0.3 while K_est_max pins the estimate at its 1.0 seed (see
  // below), because pinned the whole schedule collapses to the constant
  // 1/(1 + beta) and that constant IS the deployed behaviour: 1/1.3 = 0.769.
  // Moving beta alone would therefore change the gain of every grasp on real
  // hardware by 26% while buying nothing, since no adaptation is running.
  // beta and K_est_max are released together, in #426.
  double beta{0.3};        // adaptive gain sensitivity
  // Upper bound on a stiffness sample and on the running estimate, and — at
  // this default — the switch that keeps adaptation OFF.
  //
  // 1.0 equals the seed K_contact_est starts from, so the estimate can never
  // rise and gain_scale is the constant 1/(1 + beta). That reproduces the
  // behaviour this controller shipped with while the estimator was inert, bit
  // for bit, and it is deliberately the DEFAULT rather than only a deployed
  // override: a config that omits the key must not silently switch on a
  // feature that is known to fail on hardware.
  //
  // Why it fails: K_inst = delta_f/delta_s divides by an increment that goes to
  // zero as the loop converges, so a force step made of sensor ripple reads as
  // an arbitrarily stiff object, and the resulting state is absorbing — the
  // collapsed gain_scale shrinks ds, which shrinks delta_s, so no corrective
  // sample is ever taken. Measured against the pinned behaviour at 500 Hz with
  // 2 mN of 25 Hz-filtered ripple, K in [10, 50] went from 3.3-9.3 s to
  // 10-30 s, i.e. past the behaviour tree's 10 s budget, while pinned stayed
  // flat at 3.3-9.3 s across 0-20 mN. The estimator needs a different
  // estimator, not a different constant: see grasp_tuning_guide.md 6.6 and
  // #426. Raising this (400 gives 2x the documented [10, 200] design range) is
  // what turns adaptation back on, and it belongs with the beta retune above.
  double K_est_max{1.0};

  // Force thresholds
  double f_contact_threshold{0.2};  // [N] contact detection threshold
  double f_target{2.0};             // [N] target grip force
  double f_ramp_rate{1.0};          // [N/s] force reference ramp rate

  // Rate & saturation limits
  double ds_max{0.05};         // max ds/dt [1/s]
  double delta_s_max{0.15};    // max deformation delta_s after contact
  double integral_clamp{0.1};  // integrator saturation

  // State machine timing
  double approach_speed{0.2};  // [1/s] approaching ds/dt
  double release_speed{0.3};   // [1/s] releasing ds/dt
  // settle_epsilon / settle_time are the tolerance and dwell of the ForceControl
  // -> Holding promotion, and they are measured on the error against f_target —
  // NOT against the mid-ramp reference. UpdateForceControl will not start the
  // dwell until the ramp has reached active_target_force_, so these two need no
  // relation to f_ramp_rate. They did before: the dwell used to run during the
  // ramp, so any (settle_time < 2*settle_epsilon / f_ramp_rate) promoted while
  // the reference was merely passing through the measured force, and Holding —
  // which has no ramp — then froze the reference there. Measured on a deployed
  // set (settle_epsilon 0.5, f_ramp_rate 2.0, settle_time 0.3), that landed
  // Holding at f_desired 0.60-0.68 N against a 2.0 N target, every grasp.
  double settle_epsilon{0.1};       // [N] force convergence threshold
  double settle_time{0.3};          // [s] convergence hold time
  double contact_settle_time{0.1};  // [s] Contact phase dwell

  // Anomaly detection
  double df_slip_threshold{5.0};     // [N/s] slip df/dt threshold (negative direction)
  double f_slip_fraction{0.5};       // [0,1] grip is "lost" below f_target * this
  double grip_tightening_rate{0.5};  // [N/s] reference increase rate while grip is lost
  double grip_decay_rate{0.1};       // [N/s] force decay rate toward target after tightening
  double f_max_multiplier{2.0};      // max force = f_target * multiplier

  // NOTE: there is no filter cutoff here. Update() consumes an ALREADY FILTERED
  // force — see GraspController::Update. The fields were removed rather than
  // deprecated on purpose: the span signature cannot express the change, so a
  // compile error at the assignment is the only reliable notice a caller gets.
  //
  // grip_tightening_rate replaced grip_tightening_ratio for the same reason, and
  // the rename is deliberate rather than a value change: the old field was
  // applied ONCE PER TICK as f_desired *= (1 + ratio), so the grip force a slip
  // produced was a function of control_rate — 0.15 reached the f_max_multiplier
  // cap in ~11 ticks, i.e. 22 ms at 500 Hz and 11 ms at 1 kHz. A rate in N/s is
  // rate-independent and symmetric with grip_decay_rate. Keeping the old name
  // would have let a YAML carrying 0.15 be silently reinterpreted as 0.15 N/s.
};

// ── Per-finger runtime state ─────────────────────────────────────────────────

struct FingerState {
  double s{0.0};               // grasp parameter [0, 1]
  double s_at_contact{0.0};    // s value at contact detection
  // s as of the end of the previous tick. Written after the FSM has advanced s,
  // so that during a tick it still holds s_{t-2} while s holds s_{t-1}: the
  // stiffness estimator needs that pair, since it divides the force step
  // f_t - f_{t-1} by the s increment that caused it. Latching it at tick entry
  // instead makes it equal to s and the estimator silently stops updating.
  double s_prev{0.0};          // previous step s (for stiffness estimation)
  double f_desired{0.0};       // current force reference [N]
  double f_measured{0.0};      // filtered force [N]
  double f_prev{0.0};          // previous step force [N]
  double integral_error{0.0};  // PI integrator
  double K_contact_est{1.0};   // estimated contact stiffness [N/delta_s]
  bool contact_detected{false};
  bool integrator_frozen{false};
};

// ── Output ───────────────────────────────────────────────────────────────────

struct GraspJointCommands {
  int num_fingers{0};                       // fingers actually populated
  std::array<int, kMaxGraspFingers> dof{};  // per-finger DoF (valid for f < num_fingers)
  // q[f][0 .. dof[f]) are the joint commands for finger f; entries beyond
  // dof[f] and fingers beyond num_fingers are left zero.
  std::array<std::array<double, kMaxDoFPerFinger>, kMaxGraspFingers> q{};
};

}  // namespace rtc::grasp

#endif  // RTC_CONTROLLERS_GRASP_GRASP_TYPES_HPP_
