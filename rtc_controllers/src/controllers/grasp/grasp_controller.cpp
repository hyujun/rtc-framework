#include "rtc_controllers/grasp/grasp_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::grasp {

// ═══════════════════════════════════════════════════════════════════════════════
// Init
// ═══════════════════════════════════════════════════════════════════════════════

void GraspController::Init(std::span<const FingerConfig> configs, const GraspParams& params) {
  num_fingers_ = std::min(static_cast<int>(configs.size()), kMaxGraspFingers);
  configs_ = {};
  for (int f = 0; f < num_fingers_; ++f) {
    configs_[static_cast<std::size_t>(f)] = configs[static_cast<std::size_t>(f)];
  }
  params_ = params;
  active_target_force_ = params_.f_target;

  ResetFingers();
  phase_ = GraspPhase::kIdle;
  initialized_ = true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Command interface
// ═══════════════════════════════════════════════════════════════════════════════

void GraspController::CommandGrasp(double target_force) noexcept {
  if (target_force > 0.0) {
    active_target_force_ = target_force;
  }
  grasp_requested_.store(true, std::memory_order_release);
}

void GraspController::CommandRelease() noexcept {
  release_requested_.store(true, std::memory_order_release);
}

void GraspController::Reset() noexcept {
  phase_ = GraspPhase::kIdle;
  grasp_requested_.store(false, std::memory_order_release);
  // The release flag is defence-in-depth rather than a live hazard on its own:
  // UpdateIdle() already drops it unconditionally ("이미 Idle"), so a lone stale
  // RELEASE cannot act. What it covers is the ordering — UpdateIdle() takes a
  // pending GRASP first, so a pair armed while another law owned the hand would
  // enter kApproaching still carrying the release, and abort the new grasp on the
  // next tick.
  release_requested_.store(false, std::memory_order_release);
  // Same as Init(), and for the same reason: the per-finger s / force history
  // describes an object this controller may no longer be touching.
  ResetFingers();
}

void GraspController::set_target_force(double f) noexcept {
  if (f > 0.0) {
    active_target_force_ = f;
  }
}

void GraspController::set_params(const GraspParams& params) noexcept {
  params_ = params;
  // 모든 필드가 즉시 발효한다. 예외였던 lpf_cutoff_hz 는 이제 이 구조체에 없다
  // (필터가 상류로 이동) — 그 필드만 다음 Init 까지 반영되지 않는 비대칭이
  // 사라졌다.
}

// ═══════════════════════════════════════════════════════════════════════════════
// Update (RT-safe, 매 제어 주기 호출)
// ═══════════════════════════════════════════════════════════════════════════════

GraspJointCommands GraspController::Update(std::span<const double> f_filtered, double dt) noexcept {
  GraspJointCommands output{};
  // s as it stands on entry, staged so that s_prev can be published *after* the
  // FSM has moved s — see the write below the switch for why it cannot be
  // latched here alongside f_prev.
  std::array<double, kMaxGraspFingers> s_at_tick_start{};
  output.num_fingers = num_fingers_;
  for (int f = 0; f < num_fingers_; ++f) {
    output.dof[static_cast<std::size_t>(f)] = configs_[static_cast<std::size_t>(f)].dof;
  }

  if (!initialized_ || dt <= 0.0) {
    // 미초기화 또는 잘못된 dt — 현재 posture 유지
    for (int f = 0; f < num_fingers_; ++f) {
      output.q[static_cast<std::size_t>(f)] = InterpolatePosture(
          configs_[static_cast<std::size_t>(f)], fingers_[static_cast<std::size_t>(f)].s);
    }
    return output;
  }

  // ── 1. Latch this tick's force ──────────────────────────────────────────
  // Already filtered by the caller — see Update()'s contract. Missing entries
  // are treated as 0 (RT-safe: no throw on short span).
  for (int f = 0; f < num_fingers_; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];
    fs.f_prev = fs.f_measured;
    s_at_tick_start[static_cast<std::size_t>(f)] = fs.s;
    // Estimator diagnostics are per-tick, not sticky: ComputeAdaptivePI runs
    // only in kForceControl / kHolding, so without this a logger reading them
    // on an Approaching tick would report the last force-control sample as if
    // it were fresh.
    fs.delta_s_used = 0.0;
    fs.delta_f_used = 0.0;
    fs.K_inst_raw = 0.0;
    fs.estimate_updated = false;
    fs.f_measured = (static_cast<std::size_t>(f) < f_filtered.size())
                        ? f_filtered[static_cast<std::size_t>(f)]
                        : 0.0;
  }

  // ── 2. State machine update ─────────────────────────────────────────────
  fingers_reset_this_tick_ = false;
  switch (phase_) {
    case GraspPhase::kIdle:
      UpdateIdle();
      break;
    case GraspPhase::kApproaching:
      UpdateApproaching(dt);
      break;
    case GraspPhase::kContact:
      UpdateContact(dt);
      break;
    case GraspPhase::kForceControl:
      UpdateForceControl(dt);
      break;
    case GraspPhase::kHolding:
      UpdateHolding(dt);
      break;
    case GraspPhase::kReleasing:
      UpdateReleasing(dt);
      break;
  }

  // ── 3. Compute joint commands from s ────────────────────────────────────
  for (int f = 0; f < num_fingers_; ++f) {
    const auto idx = static_cast<std::size_t>(f);
    // s_prev is published here, not in the latch loop above. Latched next to
    // f_prev it equalled s for the whole tick, so the stiffness estimator's
    // delta_s = s - s_prev was identically zero, its |delta_s| > epsilon guard
    // never passed, K_contact_est stayed pinned at the 1.0 seed and gain_scale
    // degenerated to the constant 1/(1 + beta) — the adaptation never ran.
    // Writing it here leaves the value observers read unchanged (s_prev is
    // still s_{t-1} once the tick returns) while giving ComputeAdaptivePI the
    // increment s_{t-1} - s_{t-2}, which is the one that produced the force
    // step f_t - f_{t-1} it is divided into.
    // ...unless the FSM reset the fingers during this very tick. ResetFingers()
    // zeroes the whole FingerState, s included, so restoring the pre-reset s
    // into s_prev here would leave s = 0 beside s_prev = the s the finger held
    // before the reset. Anything differencing the pair — which is how every
    // plant model in the tests derives finger velocity — then reads a spurious
    // -5 1/s for one tick after a release completes. It fires on the
    // Releasing->Idle and Idle->Approaching transitions, and "ResetFingers
    // means the finger state is zero" is the property that function exists for.
    if (!fingers_reset_this_tick_) {
      fingers_[idx].s_prev = s_at_tick_start[idx];
    }
    output.q[idx] = InterpolatePosture(configs_[idx], fingers_[idx].s);
  }

  return output;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Phase updates
// ═══════════════════════════════════════════════════════════════════════════════

void GraspController::UpdateIdle() noexcept {
  if (grasp_requested_.exchange(false, std::memory_order_acq_rel)) {
    ResetFingers();
    phase_ = GraspPhase::kApproaching;
  }
  // release_requested 무시 (이미 Idle)
  release_requested_.store(false, std::memory_order_relaxed);
}

void GraspController::UpdateApproaching(double dt) noexcept {
  // Release 를 이 phase 의 다른 무엇보다 먼저 본다 — kForceControl / kHolding 이
  // 함수 끝에서 확인하는 것과 반대다. 그 둘과 달리 여기에는 경쟁하는 전이
  // (kContact) 가 있어서, 끝에 두면 접촉이 성립한 tick 의 early return 이
  // 플래그를 건너뛴다. 요청이 소실되지는 않지만 (다음 소비자는 kForceControl
  // 이다) 그 사이 contact dwell 만큼 손은 계속 닫힌다. 먼저 소비하면 "RELEASE
  // 는 지금 닫기를 멈춘다" 가 phase 와 무관하게 성립한다.
  if (release_requested_.exchange(false, std::memory_order_acq_rel)) {
    phase_ = GraspPhase::kReleasing;
    return;
  }

  for (int f = 0; f < num_fingers_; ++f) {
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

  // Contact 전이: thumb + 접촉한 아무 손가락 하나 (#432).
  //
  // 이전 규칙은 인덱스 0·1 두 손가락(관례상 thumb + index)을 요구했다. 물체가
  // 그 쌍이 아닌 곳에 물리면 다른 손가락이 아무리 세게 눌려도 FSM 이 진행하지
  // 않는다. 2026-08-13 실기(p1b, f_contact_threshold 0.8 N)에서 그대로
  // 발현했다: 물체가 thumb-middle 사이에 물려 max force 가 thumb 0.983 N /
  // middle 1.359 N (둘 다 문턱 초과) 인데 index 는 0.446 N 에 그쳤고, 58036
  // tick(116 s) 내내 approaching 이었다 — contact / force_control / holding 이
  // 전부 0%. 같은 런에서 thumb ∧ middle 의 contact_detected 는 tick 35269 부터
  // 11825 tick 성립해 있었으므로, 규칙만 바꿔도 그 시점에 래치된다.
  //
  // thumb 은 필수로 남는다 — 대향 없이 같은 방향 손가락들만 닿은 것은 파지가
  // 아니다 (ApproachHoldsWhenThumbCannotContact 가 고정한다). 어느 슬롯이
  // thumb 인지는 config 가 정한다 (params_.thumb_finger_index): FingerConfig
  // 는 geometry 만 들고 있어 이 패키지 안에서는 알 수 없다.
  //
  // 관측된 대향 집합(PullForceEstimator 의 touch_mask, 배선의 opposing_mask)을
  // 주입하는 안은 검토 후 채택하지 않았다. contact_detected 는 이 함수
  // 안에서만 세워지므로 전이 시점에 안 걸린 손가락은 이 파지 동안 force
  // control 대상이 아니고(UpdateForceControl 의 early continue), 따라서 관측
  // 쌍이 contact_detected 를 대체하면 파트너가 서보되지 않는 파지가 나온다.
  // 대체하지 않으면 마스크는 부분집합 필터로만 작동해 이 규칙과 같은 tick 에
  // 래치한다 — 위 실기 런에서 실측으로 확인했다. 게다가 그 마스크는 ring 을
  // 보지 못한다 (tsid.contacts 제약으로 tips.tip_names 에 없다).
  //
  // 손가락이 2개 미만이면 파트너가 존재하지 않으므로 가용한 손가락 전부의
  // 접촉을 요구한다 (이전 규칙과 동일).
  const int thumb = thumb_finger();
  bool pair_in_contact = false;
  if (num_fingers_ >= 2) {
    if (fingers_[static_cast<std::size_t>(thumb)].contact_detected) {
      for (int f = 0; f < num_fingers_; ++f) {
        if (f != thumb && fingers_[static_cast<std::size_t>(f)].contact_detected) {
          pair_in_contact = true;
          break;
        }
      }
    }
  } else if (num_fingers_ == 1) {
    pair_in_contact = fingers_[0].contact_detected;
  }
  if (pair_in_contact) {
    contact_settle_timer_ = 0.0;
    phase_ = GraspPhase::kContact;
    return;
  }

  // s 가 1.0 에 도달했는데 접촉이 없어도 kIdle 로 낙하하지 않는다 — 손은 완전히
  // 닫힌 자세에서 대기하고 접촉 판정은 위 루프에서 매 tick 계속 돈다. 접촉은
  // 시간 제한이 아니라 사건이고 힘은 늦게 들어올 수 있다: 물체가 미끄러져 들어
  // 오거나, 팔이 아직 움직이는 중이거나, fingertip lane 이 뒤늦게 살아나는
  // 경우다. 늦은 접촉이 잡히면 s_at_contact = 1.0 이 되고, s 는 이미 1.0 에
  // clamp 돼 있으므로 이어지는 force control 은 여는 방향으로만 권한을 갖는다
  // (deformation guard 의 여유는 남지만 s 상한이 먼저 막는다).
  //
  // 대가는 이 phase 가 스스로 끝나지 않는다는 것이다. 나가는 길은 접촉 또는
  // RELEASE 뿐이며, 그동안 phase 미러가 non-Idle 로 남아 grasp_controller_type
  // 전환도 막힌다 (GraspModeChangeRejectReason 의 quiet gate). 위쪽 RELEASE
  // 소비가 그 유일한 탈출구이므로 두 변경은 한 세트다.
}

void GraspController::UpdateContact(double dt) noexcept {
  // 안정화 대기
  contact_settle_timer_ += dt;

  if (contact_settle_timer_ >= params_.contact_settle_time) {
    // ForceControl 진입 준비: f_desired를 0에서 시작 (ramp)
    for (int f = 0; f < num_fingers_; ++f) {
      auto& fs = fingers_[static_cast<std::size_t>(f)];
      fs.f_desired = 0.0;
      fs.integral_error = 0.0;
      fs.integrator_frozen = false;
    }
    force_settle_timer_ = 0.0;
    phase_ = GraspPhase::kForceControl;
  }
}

void GraspController::UpdateForceControl(double dt) noexcept {
  bool all_settled = true;

  for (int f = 0; f < num_fingers_; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];

    if (!fs.contact_detected)
      continue;

    // Force reference ramp
    fs.f_desired = std::min(fs.f_desired + params_.f_ramp_rate * dt, active_target_force_);

    // Adaptive PI → ds
    double ds = ComputeAdaptivePI(f, dt);

    // Deformation guard
    ApplyDeformationGuard(f, ds);

    // s 갱신
    fs.s += ds * dt;
    fs.s = std::clamp(fs.s, 0.0, 1.0);

    // 수렴 판정 — 램프가 목표에 닿기 전에는 절대 성립하지 않는다.
    //
    // 오차를 램프 중인 f_desired 에 대고 재면, 판정하는 것이 "목표힘에 수렴했다"
    // 가 아니라 "램프가 지금 측정힘 근처를 지나간다" 가 된다. 램프는 폭 2*eps 인
    // 창을 2*eps/f_ramp_rate 초에 통과하므로, settle_time 이 그보다 짧으면 승급은
    // 램프 도중에 일어나고 — kHolding 에는 램프가 없으므로 — 기준값이 거기서
    // 얼어붙는다. 실제로 배포된 조합(eps 0.5, rate 2.0, settle 0.3)에서 통과에
    // 0.5 s 가 걸려 0.3 s 를 넘으므로 이 조건은 예외가 아니라 상시였다: 실측
    // kHolding 진입 f_desired 0.60~0.68 N (목표 2.0 N), 물체 강성 무관.
    //
    // f_desired 는 std::min 으로 정확히 active_target_force_ 에 안착하므로
    // 부등호 비교로 충분하다 (epsilon 불필요).
    if (fs.f_desired < active_target_force_ ||
        std::abs(fs.f_desired - fs.f_measured) > params_.settle_epsilon) {
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

void GraspController::UpdateHolding(double dt) noexcept {
  for (int f = 0; f < num_fingers_; ++f) {
    auto& fs = fingers_[static_cast<std::size_t>(f)];

    if (!fs.contact_detected)
      continue;

    // Gradual force decay toward active_target_force_ after tightening
    if (fs.f_desired > active_target_force_) {
      fs.f_desired = std::max(fs.f_desired - params_.grip_decay_rate * dt, active_target_force_);
    }

    // Force control 유지
    double ds = ComputeAdaptivePI(f, dt);
    ApplyDeformationGuard(f, ds);
    fs.s += ds * dt;
    fs.s = std::clamp(fs.s, 0.0, 1.0);

    // Force anomaly detection.
    //
    // 두 조건의 성격이 다르다: df/dt 는 사건(슬립의 순간)이고, f_slip_fraction
    // 비교는 상태(지금 grip 을 잃고 있다)다. 후자는 조건이 참인 동안 매 tick
    // 재발화하므로 **보정량은 반드시 rate 여야 한다** — per-tick 비율이었을 때
    // grip force 가 control_rate 의 함수가 됐다 (0.15/tick 이면 500 Hz 에서
    // 22 ms 만에 f_max_multiplier 상한 도달). 이제 grip_decay_rate 와 같은
    // 단위·같은 축이며, 둘의 비가 곧 조임/풀림의 비대칭이다.
    if (dt > 0.0) {
      const double df_dt = (fs.f_measured - fs.f_prev) / dt;
      if (df_dt < -params_.df_slip_threshold ||
          fs.f_measured < active_target_force_ * params_.f_slip_fraction) {
        fs.f_desired = std::min(fs.f_desired + params_.grip_tightening_rate * dt,
                                active_target_force_ * params_.f_max_multiplier);
        // 여기서 integrator_frozen 을 풀지 않는다. 동결의 유일한 권한은 이 tick
        // 앞에서 이미 돈 ApplyDeformationGuard 이고, 그것이 얼렸다는 것은 "더
        // 닫지 말라" 는 뜻이다. 풀어주면 guard 가 얼리고 이 분기가 녹이는 교착이
        // 매 tick 반복된다 — 실측: 연체(K=3)에서 f_desired 가 상한 4.0 N 에
        // 영구 고정된 채 Δs 는 delta_s_max 에 붙어 힘은 0.85 N 에서 멈췄고,
        // 그 상태로 빠져나오지 못했다. guard 는 여유가 회복되면 스스로 푼다.
      }
    }
  }

  // Release 요청 처리
  if (release_requested_.exchange(false, std::memory_order_acq_rel)) {
    phase_ = GraspPhase::kReleasing;
  }
}

void GraspController::UpdateReleasing(double dt) noexcept {
  bool all_open = true;

  for (int f = 0; f < num_fingers_; ++f) {
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

std::array<double, kMaxDoFPerFinger> GraspController::InterpolatePosture(const FingerConfig& cfg,
                                                                         double s) noexcept {
  const double sc = std::clamp(s, 0.0, 1.0);
  std::array<double, kMaxDoFPerFinger> q{};
  const int dof = std::min(cfg.dof, kMaxDoFPerFinger);
  for (int j = 0; j < dof; ++j) {
    const auto idx = static_cast<std::size_t>(j);
    q[idx] = (1.0 - sc) * cfg.q_open[idx] + sc * cfg.q_close[idx];
  }
  return q;
}

double GraspController::ComputeAdaptivePI(int finger, double dt) noexcept {
  auto& fs = fingers_[static_cast<std::size_t>(finger)];

  const double e_f = fs.f_desired - fs.f_measured;

  // ── Online stiffness estimation (EMA) ─────────────────────────────────
  const double delta_f = fs.f_measured - fs.f_prev;
  const double delta_s = fs.s - fs.s_prev;
  // Diagnostics (#428): stored where the estimator reads them, because the pair
  // is not recoverable from a post-Update log — see FingerState.
  fs.delta_s_used = delta_s;
  fs.delta_f_used = delta_f;
  if (std::abs(delta_s) > kDeltaSEpsilon) {
    // Clamped before it enters the EMA. delta_s goes to zero as the loop
    // converges, so on a noisy force lane K_inst is a ratio of two quantities
    // that are both sensor ripple, and one unbounded sample is enough to sink
    // gain_scale for the rest of the grasp with no path back (a collapsed gain
    // shrinks ds, which shrinks delta_s, which stops any corrective sample).
    // The clamp bounds that failure; it does not remove it — at 500 Hz the
    // per-tick force step is 0.4-1.4 mN against ~1.4 mN of filtered ripple, so
    // the single-tick estimate is noise over noise on real hardware. See
    // grasp_tuning_guide.md 6.6.
    const double K_inst = delta_f / delta_s;
    fs.K_inst_raw = K_inst;
    if (K_inst > 0.0) {
      const double K_bounded = std::min(K_inst, params_.K_est_max);
      fs.K_contact_est =
          params_.alpha_ema * fs.K_contact_est + (1.0 - params_.alpha_ema) * K_bounded;
      fs.estimate_updated = true;
    }
  }

  // ── Adaptive gain scheduling ──────────────────────────────────────────
  const double gain_scale = 1.0 / (1.0 + params_.beta * fs.K_contact_est);
  const double Kp = params_.Kp_base * gain_scale;
  const double Ki = params_.Ki_base * gain_scale;

  // ── PI with anti-windup ───────────────────────────────────────────────
  if (!fs.integrator_frozen) {
    fs.integral_error += e_f * dt;
    fs.integral_error =
        std::clamp(fs.integral_error, -params_.integral_clamp, params_.integral_clamp);
  }

  double ds = Kp * e_f + Ki * fs.integral_error;
  ds = std::clamp(ds, -params_.ds_max, params_.ds_max);

  return ds;
}

void GraspController::ApplyDeformationGuard(int finger, double& ds) noexcept {
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
  } else {
    // 여유 충분: 인테그레이터 동결 해제 (deformation guard에 의한 동결만 해제)
    fs.integrator_frozen = false;
  }
}

void GraspController::ResetFingers() noexcept {
  for (int f = 0; f < num_fingers_; ++f) {
    fingers_[static_cast<std::size_t>(f)] = FingerState{};
  }
  // Tell the s_prev write at the end of Update() to stand down for this tick —
  // see there for why restoring a pre-reset s would undo part of this reset.
  fingers_reset_this_tick_ = true;
  contact_settle_timer_ = 0.0;
  force_settle_timer_ = 0.0;
}

}  // namespace rtc::grasp
