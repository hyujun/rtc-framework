#ifndef RTC_CONTROLLERS_GRASP_GRASP_TYPES_HPP_
#define RTC_CONTROLLERS_GRASP_GRASP_TYPES_HPP_

#include <array>
#include <cstdint>

namespace rtc::grasp {

/// Number of fingers under force control (thumb, index, middle).
static constexpr int kNumGraspFingers = 3;
/// Degrees of freedom per finger (MCP_AA, MCP_FE, DIP_FE).
static constexpr int kDoFPerFinger = 3;

// ── State Machine ────────────────────────────────────────────────────────────

enum class GraspPhase : uint8_t {
  kIdle,          // 대기 상태, s 유지
  kApproaching,   // 일정 속도로 closing (position ramp)
  kContact,       // 전 finger 접촉 대기 + 안정화
  kForceControl,  // PI force regulation 활성화 + force ramp
  kHolding,       // Force 유지 + anomaly monitoring
  kReleasing      // 역방향 position ramp
};

// ── Per-finger configuration (생성 시 고정) ─────────────────────────────────

struct FingerConfig {
  std::array<double, kDoFPerFinger> q_open{};   // fully open joint angles [rad]
  std::array<double, kDoFPerFinger> q_close{};  // fully closed joint angles [rad]
};

// ── Grasp parameters ─────────────────────────────────────────────────────────

struct GraspParams {
  // PI gains
  double Kp_base{0.02};               // [1/(N*s)] proportional gain base
  double Ki_base{0.002};              // [1/(N*s^2)] integral gain base

  // Adaptive gain scheduling (stiffness EMA)
  double alpha_ema{0.95};             // stiffness EMA coefficient [0,1]
  double beta{0.3};                   // adaptive gain sensitivity

  // Force thresholds
  double f_contact_threshold{0.2};    // [N] contact detection threshold
  double f_target{2.0};               // [N] target grip force
  double f_ramp_rate{1.0};            // [N/s] force reference ramp rate

  // Rate & saturation limits
  double ds_max{0.05};                // max ds/dt [1/s]
  double delta_s_max{0.15};           // max deformation delta_s after contact
  double integral_clamp{0.1};         // integrator saturation

  // State machine timing
  double approach_speed{0.2};         // [1/s] approaching ds/dt
  double release_speed{0.3};          // [1/s] releasing ds/dt
  double settle_epsilon{0.1};         // [N] force convergence threshold
  double settle_time{0.3};            // [s] convergence hold time
  double contact_settle_time{0.1};    // [s] Contact phase dwell

  // Anomaly detection
  double df_slip_threshold{5.0};      // [N/s] slip df/dt threshold (negative direction)
  double grip_tightening_ratio{0.15}; // force increase ratio on slip
  double grip_decay_rate{0.1};          // [N/s] force decay rate toward target after tightening
  double f_max_multiplier{2.0};       // max force = f_target * multiplier

  // Filter
  double lpf_cutoff_hz{25.0};        // Bessel LPF cutoff [Hz]
  double control_rate_hz{500.0};      // control loop rate [Hz]
};

// ── Per-finger runtime state ─────────────────────────────────────────────────

struct FingerState {
  double s{0.0};                      // grasp parameter [0, 1]
  double s_at_contact{0.0};           // s value at contact detection
  double s_prev{0.0};                 // previous step s (for stiffness estimation)
  double f_desired{0.0};              // current force reference [N]
  double f_measured{0.0};             // filtered force [N]
  double f_prev{0.0};                 // previous step force [N]
  double integral_error{0.0};         // PI integrator
  double K_contact_est{1.0};          // estimated contact stiffness [N/delta_s]
  bool   contact_detected{false};
  bool   integrator_frozen{false};
};

// ── Output ───────────────────────────────────────────────────────────────────

struct GraspJointCommands {
  std::array<std::array<double, kDoFPerFinger>, kNumGraspFingers> q{};
};

}  // namespace rtc::grasp

#endif  // RTC_CONTROLLERS_GRASP_GRASP_TYPES_HPP_
