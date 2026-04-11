#include "rtc_controllers/grasp/grasp_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::grasp {

// ═══════════════════════════════════════════════════════════════════════════════
// Init
// ═══════════════════════════════════════════════════════════════════════════════

void GraspController::Init(
  const std::array<FingerConfig, kNumGraspFingers>& configs,
  const GraspParams& params)
{
  configs_ = configs;
  params_ = params;
  active_target_force_ = params_.f_target;

  force_filter_.Init(params_.lpf_cutoff_hz, params_.control_rate_hz);
  force_filter_.Reset();

  ResetFingers();
  phase_ = GraspPhase::kIdle;
  initialized_ = true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Command interface
// ═══════════════════════════════════════════════════════════════════════════════

void GraspController::CommandGrasp(double target_force) noexcept
{
  if (target_force > 0.0) {
    active_target_force_ = target_force;
  }
  grasp_requested_.store(true, std::memory_order_release);
}

void GraspController::CommandRelease() noexcept
{
  release_requested_.store(true, std::memory_order_release);
}

void GraspController::set_target_force(double f) noexcept
{
  if (f > 0.0) {
    active_target_force_ = f;
  }
}

void GraspController::set_params(const GraspParams& params) noexcept
{
  params_ = params;
  // LPF 재계산은 Init에서만 — RT path에서 Init 호출 불가
  // cutoff가 변경되면 다음 Init에서 반영
}

// ═══════════════════════════════════════════════════════════════════════════════
// Update (RT-safe, 매 제어 주기 호출)
// ═══════════════════════════════════════════════════════════════════════════════

GraspJointCommands GraspController::Update(
  std::span<const double, kNumGraspFingers> f_raw, double dt) noexcept
{
  GraspJointCommands output{};

  if (!initialized_ || dt <= 0.0) {
    // 미초기화 또는 잘못된 dt — 현재 posture 유지
    for (int f = 0; f < kNumGraspFingers; ++f) {
      output.q[static_cast<std::size_t>(f)] =
        InterpolatePosture(configs_[static_cast<std::size_t>(f)],
                           fingers_[static_cast<std::size_t>(f)].s);
    }
    return output;
  }

  // ── 1. Force filtering ──────────────────────────────────────────────────
  const std::array<double, kNumGraspFingers> raw_arr{f_raw[0], f_raw[1], f_raw[2]};
  const auto filtered = force_filter_.Apply(raw_arr);
  for (int f = 0; f < kNumGraspFingers; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];
    fs.f_prev = fs.f_measured;
    fs.s_prev = fs.s;
    fs.f_measured = filtered[static_cast<std::size_t>(f)];
  }

  // ── 2. State machine update ─────────────────────────────────────────────
  switch (phase_) {
    case GraspPhase::kIdle:         UpdateIdle();                break;
    case GraspPhase::kApproaching:  UpdateApproaching(dt);       break;
    case GraspPhase::kContact:      UpdateContact(dt);           break;
    case GraspPhase::kForceControl: UpdateForceControl(dt);      break;
    case GraspPhase::kHolding:      UpdateHolding(dt);           break;
    case GraspPhase::kReleasing:    UpdateReleasing(dt);         break;
  }

  // ── 3. Compute joint commands from s ────────────────────────────────────
  for (int f = 0; f < kNumGraspFingers; ++f) {
    const auto idx = static_cast<std::size_t>(f);
    output.q[idx] = InterpolatePosture(configs_[idx], fingers_[idx].s);
  }

  return output;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Phase updates
// ═══════════════════════════════════════════════════════════════════════════════

void GraspController::UpdateIdle() noexcept
{
  if (grasp_requested_.exchange(false, std::memory_order_acq_rel)) {
    ResetFingers();
    force_filter_.Reset();
    phase_ = GraspPhase::kApproaching;
  }
  // release_requested 무시 (이미 Idle)
  release_requested_.store(false, std::memory_order_relaxed);
}

void GraspController::UpdateApproaching(double dt) noexcept
{
  for (int f = 0; f < kNumGraspFingers; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];

    if (!fs.contact_detected) {
      // 일정 속도로 closing
      fs.s += params_.approach_speed * dt;
      fs.s = std::min(fs.s, 1.0);

      // 접촉 감지
      if (fs.f_measured > params_.f_contact_threshold) {
        fs.contact_detected = true;
        fs.s_at_contact = fs.s;
        fs.f_prev = fs.f_measured;
        fs.K_contact_est = 1.0;
        fs.integral_error = 0.0;
        fs.integrator_frozen = false;
      }
    }
    // 접촉한 finger는 s 고정 (대기)
  }

  // Contact 전이: thumb(0) + index(1) 두 손가락만 접촉하면 충분.
  // middle(2) 은 접촉 여부와 무관하게 grasp 진행을 막지 않는다.
  if (fingers_[0].contact_detected && fingers_[1].contact_detected) {
    contact_settle_timer_ = 0.0;
    phase_ = GraspPhase::kContact;
    return;
  }

  // Grasp 실패: thumb 또는 index 가 s=1.0 까지 닫혔는데도 접촉 못 함
  for (int f = 0; f < 2; ++f) {
    const auto& fs = fingers_[static_cast<std::size_t>(f)];
    if (!fs.contact_detected && fs.s >= 1.0) {
      phase_ = GraspPhase::kIdle;
      return;
    }
  }
}

void GraspController::UpdateContact(double dt) noexcept
{
  // 안정화 대기
  contact_settle_timer_ += dt;

  if (contact_settle_timer_ >= params_.contact_settle_time) {
    // ForceControl 진입 준비: f_desired를 0에서 시작 (ramp)
    for (int f = 0; f < kNumGraspFingers; ++f) {
      auto& fs = fingers_[static_cast<std::size_t>(f)];
      fs.f_desired = 0.0;
      fs.integral_error = 0.0;
      fs.integrator_frozen = false;
    }
    force_settle_timer_ = 0.0;
    phase_ = GraspPhase::kForceControl;
  }
}

void GraspController::UpdateForceControl(double dt) noexcept
{
  bool all_settled = true;

  for (int f = 0; f < kNumGraspFingers; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];

    if (!fs.contact_detected) continue;

    // Force reference ramp
    fs.f_desired = std::min(fs.f_desired + params_.f_ramp_rate * dt,
                            active_target_force_);

    // Adaptive PI → ds
    double ds = ComputeAdaptivePI(f, dt);

    // Deformation guard
    ApplyDeformationGuard(f, ds);

    // s 갱신
    fs.s += ds * dt;
    fs.s = std::clamp(fs.s, 0.0, 1.0);

    // 수렴 판정
    if (std::abs(fs.f_desired - fs.f_measured) > params_.settle_epsilon) {
      all_settled = false;
    }
  }

  if (all_settled) {
    force_settle_timer_ += dt;
    if (force_settle_timer_ >= params_.settle_time) {
      phase_ = GraspPhase::kHolding;
    }
  } else {
    force_settle_timer_ = 0.0;
  }

  // Release 요청 처리
  if (release_requested_.exchange(false, std::memory_order_acq_rel)) {
    phase_ = GraspPhase::kReleasing;
  }
}

void GraspController::UpdateHolding(double dt) noexcept
{
  for (int f = 0; f < kNumGraspFingers; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];

    if (!fs.contact_detected) continue;

    // Force control 유지
    double ds = ComputeAdaptivePI(f, dt);
    ApplyDeformationGuard(f, ds);
    fs.s += ds * dt;
    fs.s = std::clamp(fs.s, 0.0, 1.0);

    // Force anomaly detection
    if (dt > 0.0) {
      const double df_dt = (fs.f_measured - fs.f_prev) / dt;
      if (df_dt < -params_.df_slip_threshold ||
          fs.f_measured < active_target_force_ * 0.5) {
        // Grip tightening
        fs.f_desired = std::min(
          fs.f_desired * (1.0 + params_.grip_tightening_ratio),
          active_target_force_ * params_.f_max_multiplier);
        fs.integrator_frozen = false;  // 추가 closing 허용
      }
    }
  }

  // Release 요청 처리
  if (release_requested_.exchange(false, std::memory_order_acq_rel)) {
    phase_ = GraspPhase::kReleasing;
  }
}

void GraspController::UpdateReleasing(double dt) noexcept
{
  bool all_open = true;

  for (int f = 0; f < kNumGraspFingers; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];

    fs.s -= params_.release_speed * dt;
    fs.s = std::max(fs.s, 0.0);

    if (fs.s > 0.01) {
      all_open = false;
    }
  }

  if (all_open) {
    ResetFingers();
    phase_ = GraspPhase::kIdle;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════════

std::array<double, kDoFPerFinger> GraspController::InterpolatePosture(
  const FingerConfig& cfg, double s) noexcept
{
  const double sc = std::clamp(s, 0.0, 1.0);
  std::array<double, kDoFPerFinger> q{};
  for (int j = 0; j < kDoFPerFinger; ++j) {
    const auto idx = static_cast<std::size_t>(j);
    q[idx] = (1.0 - sc) * cfg.q_open[idx] + sc * cfg.q_close[idx];
  }
  return q;
}

double GraspController::ComputeAdaptivePI(int finger, double dt) noexcept
{
  auto& fs = fingers_[static_cast<std::size_t>(finger)];

  const double e_f = fs.f_desired - fs.f_measured;

  // ── Online stiffness estimation (EMA) ─────────────────────────────────
  const double delta_f = fs.f_measured - fs.f_prev;
  const double delta_s = fs.s - fs.s_prev;
  if (std::abs(delta_s) > kDeltaSEpsilon) {
    const double K_inst = delta_f / delta_s;
    if (K_inst > 0.0) {
      fs.K_contact_est = params_.alpha_ema * fs.K_contact_est
                       + (1.0 - params_.alpha_ema) * K_inst;
    }
  }

  // ── Adaptive gain scheduling ──────────────────────────────────────────
  const double gain_scale = 1.0 / (1.0 + params_.beta * fs.K_contact_est);
  const double Kp = params_.Kp_base * gain_scale;
  const double Ki = params_.Ki_base * gain_scale;

  // ── PI with anti-windup ───────────────────────────────────────────────
  if (!fs.integrator_frozen) {
    fs.integral_error += e_f * dt;
    fs.integral_error = std::clamp(fs.integral_error,
                                   -params_.integral_clamp,
                                    params_.integral_clamp);
  }

  double ds = Kp * e_f + Ki * fs.integral_error;
  ds = std::clamp(ds, -params_.ds_max, params_.ds_max);

  return ds;
}

void GraspController::ApplyDeformationGuard(int finger, double& ds) noexcept
{
  auto& fs = fingers_[static_cast<std::size_t>(finger)];
  const double deformation = fs.s - fs.s_at_contact;
  const double remaining = params_.delta_s_max - deformation;

  if (remaining <= 0.0) {
    // 한계 도달: closing 완전 금지, opening만 허용
    ds = std::min(ds, 0.0);
    fs.integrator_frozen = true;
  } else if (remaining < params_.delta_s_max * 0.1) {
    // 한계 근접: 비례 감속
    if (ds > 0.0) {
      ds *= remaining / (params_.delta_s_max * 0.1);
    }
    fs.integrator_frozen = true;
  }
  // else: 여유 있음 — deformation guard에서는 integrator 해제하지 않음
  // (integrator_frozen은 PI 로직에서 별도로 관리)
}

void GraspController::ResetFingers() noexcept
{
  for (int f = 0; f < kNumGraspFingers; ++f) {
    fingers_[static_cast<std::size_t>(f)] = FingerState{};
  }
  contact_settle_timer_ = 0.0;
  force_settle_timer_ = 0.0;
}

}  // namespace rtc::grasp
