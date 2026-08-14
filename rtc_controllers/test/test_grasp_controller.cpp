#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>

using namespace rtc::grasp;

namespace {
// This fixture exercises the historical 3-identical-fingers × 3-DoF layout
// (assm_v1). The controller itself is DoF-agnostic; GraspControllerRaggedTest
// below covers the variable per-finger DoF path (thumb:4/index:3/middle:2/ring:1).
constexpr int kNumFingers = 3;
constexpr int kDoF = 3;
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// Test fixture with Kelvin-Voigt contact model
// ═══════════════════════════════════════════════════════════════════════════════

class GraspControllerTest : public ::testing::Test {
 protected:
  static constexpr double kDt = 0.002;           // 500 Hz
  // Not constexpr: tests that sweep object stiffness drive this fixture rather
  // than standing up a private copy of the plant and tuning beside it.
  double obj_stiffness_ = 20.0;  // [N/delta_s]
  static constexpr double kObjDamping = 1.0;     // [N*s/delta_s]

  void SetUp() override {
    // 3 identical fingers: open at 0, close at ~60 deg per joint
    for (auto& fc : finger_configs_) {
      fc.dof = kDoF;
      fc.q_open = {0.0, 0.0, 0.0};
      fc.q_close = {0.524, 1.047, 0.785};
    }

    params_.f_contact_threshold = 0.2;
    params_.f_target = 2.0;
    params_.f_ramp_rate = 10.0;    // fast ramp for tests
    params_.approach_speed = 0.5;  // faster approach for tests
    params_.release_speed = 0.5;
    params_.settle_epsilon = 0.1;
    params_.settle_time = 0.05;  // short settle for tests
    params_.contact_settle_time = 0.02;
    params_.ds_max = 0.5;  // generous rate limit for tests
    params_.delta_s_max = 0.3;
    params_.integral_clamp = 0.5;
    params_.Kp_base = 0.05;
    params_.Ki_base = 0.01;
    params_.alpha_ema = 0.95;
    // Tracks the deployed value, and deployed means adaptation is pinned:
    // K_est_max defaults to the 1.0 seed, so gain_scale is the constant
    // 1/(1 + beta). This battery is FSM coverage and should run the schedule
    // real robots run; the estimator mechanism is exercised by
    // GraspStiffnessEstimationTest, which opts in explicitly.
    params_.beta = 0.3;
    params_.df_slip_threshold = 5.0;
    params_.f_slip_fraction = 0.5;
    params_.grip_tightening_rate = 0.5;  // [N/s]
    params_.grip_decay_rate = 0.1;       // [N/s] — 짝이므로 테스트도 명시한다
    params_.f_max_multiplier = 2.0;

    controller_.Init(finger_configs_, params_);
  }

  // Kelvin-Voigt contact model: force = K*max(delta_s,0) + D*max(ds/dt,0)
  double SimulateContactForce(double s, double s_at_contact, double ds_dt) const {
    const double delta_s = s - s_at_contact;
    if (delta_s <= 0.0)
      return 0.0;
    return obj_stiffness_ * delta_s + kObjDamping * std::max(ds_dt, 0.0);
  }

  // Run simulation loop, returns forces at each step.
  // contact_s: the s value where object is located (contact starts here)
  void RunUntilPhase(GraspPhase target_phase, int max_steps = 50000) {
    for (int i = 0; i < max_steps; ++i) {
      // Compute forces based on current finger s values
      std::array<double, kNumFingers> forces{};
      const auto& states = controller_.finger_states();
      for (int f = 0; f < kNumFingers; ++f) {
        const auto idx = static_cast<std::size_t>(f);
        const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
        forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
      }

      (void)controller_.Update(std::span<const double, 3>(forces), kDt);

      if (controller_.phase() == target_phase)
        return;
    }
  }

  // Drive the loop with only the fingers marked in `can_contact` able to feel
  // the object; every other finger reads 0 N forever (unreachable object).
  // Stops early once the FSM leaves kApproaching, and returns the phase it
  // ended in — the transition-rule tests below differ only in that mask.
  GraspPhase RunWithContactMask(const std::array<bool, kNumFingers>& can_contact, int max_steps) {
    for (int i = 0; i < max_steps; ++i) {
      std::array<double, kNumFingers> forces{};
      const auto& states = controller_.finger_states();
      for (int f = 0; f < kNumFingers; ++f) {
        const auto idx = static_cast<std::size_t>(f);
        if (!can_contact[idx]) {
          continue;
        }
        const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
        forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
      }
      (void)controller_.Update(std::span<const double, 3>(forces), kDt);
      if (controller_.phase() != GraspPhase::kApproaching &&
          controller_.phase() != GraspPhase::kIdle) {
        break;
      }
    }
    return controller_.phase();
  }

  // Run N steps with force feedback
  void RunSteps(int n) {
    for (int i = 0; i < n; ++i) {
      std::array<double, kNumFingers> forces{};
      const auto& states = controller_.finger_states();
      for (int f = 0; f < kNumFingers; ++f) {
        const auto idx = static_cast<std::size_t>(f);
        const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
        forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
      }
      (void)controller_.Update(std::span<const double, 3>(forces), kDt);
    }
  }

  std::array<FingerConfig, kNumFingers> finger_configs_{};
  GraspParams params_{};
  GraspController controller_;
  double contact_s_{0.3};  // object contact at s=0.3
};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. State Machine Transition Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, InitialPhaseIsIdle) {
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);
}

TEST_F(GraspControllerTest, IdleToApproachingOnCommand) {
  controller_.CommandGrasp();
  std::array<double, 3> zero{0.0, 0.0, 0.0};
  (void)controller_.Update(std::span<const double, 3>(zero), kDt);
  EXPECT_EQ(controller_.phase(), GraspPhase::kApproaching);
}

TEST_F(GraspControllerTest, ApproachingDetectsContact) {
  controller_.CommandGrasp();

  // Run until approaching, then simulate until contact
  RunUntilPhase(GraspPhase::kContact);
  EXPECT_EQ(controller_.phase(), GraspPhase::kContact);

  // Verify all fingers detected contact
  for (int f = 0; f < kNumFingers; ++f) {
    EXPECT_TRUE(controller_.finger_states()[static_cast<std::size_t>(f)].contact_detected);
    EXPECT_NEAR(controller_.finger_states()[static_cast<std::size_t>(f)].s_at_contact, contact_s_,
                0.05);  // within tolerance of approach speed * dt
  }
}

TEST_F(GraspControllerTest, ContactToForceControlAfterDwell) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kForceControl);
  EXPECT_EQ(controller_.phase(), GraspPhase::kForceControl);
}

TEST_F(GraspControllerTest, ForceControlToHolding) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kHolding);
}

// 수렴 판정은 램프 중인 기준값이 아니라 f_target 에 대고 재야 한다.
//
// 측정힘을 **상수로 고정해 플랜트를 루프에서 뺀다**. fixture 의 Kelvin-Voigt
// 모델을 끼우면 접촉 동역학이 오차를 흔들어 dwell 타이머를 계속 리셋해버려,
// 파라미터가 조기 승급 영역에 있어도 그 경로를 타지 않는다 (실측: 회귀를 심어도
// green). 상수 힘이면 남는 동역학은 램프뿐이라 판정이 결정적이다.
//
// 여기서 파라미터를 배포 ur5e_p1a 비율로 바꾸는 이유도 같다 — 조기 승급은
// `settle_time < 2*settle_epsilon / f_ramp_rate` 일 때만 일어나고, fixture
// 기본값(0.05 vs 0.02)은 안전한 쪽, 배포값(0.3 vs 0.5)은 위험한 쪽이다.
TEST_F(GraspControllerTest, HoldingIsNotEnteredWhileForceIsShortOfTarget) {
  params_.f_ramp_rate = 2.0;
  params_.settle_epsilon = 0.5;
  params_.settle_time = 0.3;
  ASSERT_LT(params_.settle_time, 2.0 * params_.settle_epsilon / params_.f_ramp_rate)
      << "이 파라미터 조합은 조기 승급 영역이 아니라 회귀를 잡지 못한다";
  controller_.Init(finger_configs_, params_);
  controller_.CommandGrasp();

  // 목표(2.0)에 settle_epsilon(0.5)보다 멀리 떨어진 상수 힘 — 정의상 미수렴이다.
  // 램프는 이 값을 지나쳐 2.0 까지 올라가므로, 판정을 램프 기준값에 대고 재던
  // 옛 코드는 지나가는 도중에 승급했다 (f_desired ≈ 1.1 에서).
  constexpr double kHeldForce = 1.0;
  std::array<double, kNumFingers> forces{};
  forces.fill(kHeldForce);
  ASSERT_GT(std::abs(params_.f_target - kHeldForce), params_.settle_epsilon);

  for (int i = 0; i < static_cast<int>(5.0 / kDt); ++i) {
    (void)controller_.Update(std::span<const double, 3>(forces), kDt);
  }

  EXPECT_EQ(controller_.phase(), GraspPhase::kForceControl)
      << "미수렴 상태에서 Holding 으로 승급했다 (phase=" << static_cast<int>(controller_.phase())
      << ")";
  EXPECT_DOUBLE_EQ(controller_.finger_states()[0].f_desired, params_.f_target)
      << "램프는 목표까지 올라와 있어야 한다";
}

TEST_F(GraspControllerTest, HoldingToReleasingOnCommand) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  controller_.CommandRelease();
  RunSteps(1);
  EXPECT_EQ(controller_.phase(), GraspPhase::kReleasing);
}

TEST_F(GraspControllerTest, ReleasingToIdleOnComplete) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  controller_.CommandRelease();

  RunUntilPhase(GraspPhase::kIdle, 50000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);

  // All fingers should be reset
  for (int f = 0; f < kNumFingers; ++f) {
    EXPECT_NEAR(controller_.finger_states()[static_cast<std::size_t>(f)].s, 0.0, 0.02);
  }
}

// s=1.0 까지 닫혔는데 접촉이 없으면 kApproaching 에서 **대기**한다. 이전 spec 은
// 여기서 kIdle 로 낙하했으나, 그건 접촉을 시간 제한처럼 다루는 것이었다 — 힘은
// 늦게 들어올 수 있고 (ApproachAcceptsContactAfterSaturating), 낙하는 호출측의
// `phase() != kIdle` 게이트를 통해 손 명령을 한 tick 만에 trajectory 로 되돌려
// 놓았다. 나가는 길은 접촉 또는 RELEASE (ReleaseFromApproaching) 뿐이다.
TEST_F(GraspControllerTest, ApproachHoldsAtFullCloseWhenNoContact) {
  // Set contact_s very high (object unreachable before s=1.0)
  contact_s_ = 1.5;

  controller_.CommandGrasp();
  // approach_speed 0.5 → s 0→1 에 2 s = 1000 tick. 그 2배를 돌려 포화를 확실히 한다.
  RunSteps(2000);

  EXPECT_EQ(controller_.phase(), GraspPhase::kApproaching);
  for (int f = 0; f < kNumFingers; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    EXPECT_DOUBLE_EQ(fs.s, 1.0);
    EXPECT_FALSE(fs.contact_detected);
  }
}

// 위 대기가 존재하는 이유: 완전히 닫힌 뒤에 들어온 힘도 접촉으로 잡혀야 한다.
TEST_F(GraspControllerTest, ApproachAcceptsContactAfterSaturating) {
  contact_s_ = 1.5;
  controller_.CommandGrasp();
  RunSteps(2000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kApproaching);

  // 물체가 뒤늦게 손 안으로 들어옴 — s 는 이미 1.0 이므로 힘만 올라온다.
  std::array<double, kNumFingers> forces{};
  forces.fill(params_.f_contact_threshold * 2.0);
  (void)controller_.Update(std::span<const double, 3>(forces), kDt);

  EXPECT_NE(controller_.phase(), GraspPhase::kApproaching);
  for (int f = 0; f < 2; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    EXPECT_TRUE(fs.contact_detected);
    // 포화 지점이 곧 접촉 지점이므로 이후 force control 은 닫는 여유가 없다.
    EXPECT_DOUBLE_EQ(fs.s_at_contact, 1.0);
  }
}

// RELEASE 는 kApproaching 에서도 즉시 소비된다. 이전에는 kForceControl /
// kHolding 만 소비했으므로 접촉에 실패한 grasp 는 release 로 빠져나올 수 없었고,
// BT 의 ForcePIRelease (phase==idle 대기) 가 타임아웃까지 매달렸다.
TEST_F(GraspControllerTest, ReleaseFromApproaching) {
  contact_s_ = 1.5;  // 접촉 없이 approach 중
  controller_.CommandGrasp();
  RunSteps(300);
  ASSERT_EQ(controller_.phase(), GraspPhase::kApproaching);

  controller_.CommandRelease();
  RunSteps(1);
  EXPECT_EQ(controller_.phase(), GraspPhase::kReleasing);

  RunUntilPhase(GraspPhase::kIdle, 50000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);
}

// Approaching → Contact 전이는 thumb + 접촉한 아무 손가락 하나를 요구한다.
// middle(2) 가 영영 접촉하지 못해도 thumb+index 가 물었으면 grasp 는 진행된다.
// (규칙이 인덱스 0·1 고정이던 시절에도 통과하던 단언이다 — #432 가 바꾼 것은
// 이 케이스가 아니라 아래 ...OnThumbPlusMiddle... 케이스다.)
TEST_F(GraspControllerTest, ApproachingTransitionsWithoutMiddleContact) {
  controller_.CommandGrasp();

  // thumb/index 는 contact_s_=0.3 에서 접촉, middle 은 영영 접촉 안 함.
  for (int i = 0; i < 50000; ++i) {
    std::array<double, kNumFingers> forces{};
    const auto& states = controller_.finger_states();
    for (int f = 0; f < 2; ++f) {  // thumb, index 만 force 생성
      const auto idx = static_cast<std::size_t>(f);
      const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
      forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
    }
    forces[2] = 0.0;  // middle: 항상 0
    (void)controller_.Update(std::span<const double, 3>(forces), kDt);

    if (controller_.phase() == GraspPhase::kContact ||
        controller_.phase() == GraspPhase::kForceControl ||
        controller_.phase() == GraspPhase::kHolding) {
      break;
    }
  }

  EXPECT_NE(controller_.phase(), GraspPhase::kIdle);
  EXPECT_NE(controller_.phase(), GraspPhase::kApproaching);

  EXPECT_TRUE(controller_.finger_states()[0].contact_detected);
  EXPECT_TRUE(controller_.finger_states()[1].contact_detected);
  EXPECT_FALSE(controller_.finger_states()[2].contact_detected);
}

// thumb 또는 index 가 s=1.0 까지 닫혀도 접촉 못 하면 grasp 는 진행되지 않는다 —
// 다만 kIdle 로 낙하하는 대신 kApproaching 에서 대기한다 (위 두 테스트와 같은
// spec). 이미 접촉한 index/middle 은 자기 접촉 지점에 동결된 채 남는다.
// (middle 의 s=1.0 도달은 애초에 실패 트리거가 아니다.)
TEST_F(GraspControllerTest, ApproachHoldsWhenThumbCannotContact) {
  controller_.CommandGrasp();

  // thumb 만 unreachable, index/middle 은 정상 접촉 가능.
  for (int i = 0; i < 3000; ++i) {
    std::array<double, kNumFingers> forces{};
    const auto& states = controller_.finger_states();
    // thumb (0): 항상 0 (unreachable object)
    forces[0] = 0.0;
    // index (1), middle (2): 정상 접촉
    for (int f = 1; f < kNumFingers; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
      forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
    }
    (void)controller_.Update(std::span<const double, 3>(forces), kDt);
  }

  EXPECT_EQ(controller_.phase(), GraspPhase::kApproaching);
  EXPECT_FALSE(controller_.finger_states()[0].contact_detected);
  EXPECT_DOUBLE_EQ(controller_.finger_states()[0].s, 1.0);
  EXPECT_TRUE(controller_.finger_states()[1].contact_detected);
  EXPECT_TRUE(controller_.finger_states()[2].contact_detected);
}

// #432 회귀: 물체가 thumb-index 가 **아닌** 쌍에 물려도 전이해야 한다.
//
// 2026-08-13 실기(p1b)에서 물체가 thumb-middle 사이에 물렸고 index 는
// f_contact_threshold 의 56%(0.446 / 0.8 N)에 그쳤다. 당시 규칙은 인덱스 0·1 을
// 요구했으므로 thumb 0.983 N · middle 1.359 N 이 둘 다 문턱을 넘고도 58036
// tick(116 s) 내내 approaching 이었다 — contact / force_control / holding 전부
// 0%. 규칙을 kPrimaryContacts = min(2, num_fingers_) 로 되돌리면 이 테스트가
// red 가 된다.
TEST_F(GraspControllerTest, ApproachingTransitionsOnThumbPlusMiddleWhenIndexNeverContacts) {
  controller_.CommandGrasp();
  const auto phase = RunWithContactMask({true, false, true}, 50000);

  EXPECT_NE(phase, GraspPhase::kApproaching);
  EXPECT_NE(phase, GraspPhase::kIdle);
  EXPECT_TRUE(controller_.finger_states()[0].contact_detected);
  EXPECT_FALSE(controller_.finger_states()[1].contact_detected);
  EXPECT_TRUE(controller_.finger_states()[2].contact_detected);
}

// 파트너는 여전히 필수다 — thumb 하나만 닿은 것은 파지가 아니다. 규칙을
// "thumb 이 닿으면 전이" 로 약화시키면 red. (대향 쪽 절반, 즉 thumb 이 빠진
// 경우는 ApproachHoldsWhenThumbCannotContact 가 본다.)
TEST_F(GraspControllerTest, ApproachHoldsWhenOnlyTheThumbContacts) {
  controller_.CommandGrasp();
  const auto phase = RunWithContactMask({true, false, false}, 3000);

  EXPECT_EQ(phase, GraspPhase::kApproaching);
  EXPECT_TRUE(controller_.finger_states()[0].contact_detected);
  EXPECT_FALSE(controller_.finger_states()[1].contact_detected);
  EXPECT_FALSE(controller_.finger_states()[2].contact_detected);
}

// thumb 슬롯은 config 가 정한다 (#432). 같은 손을 thumb = 슬롯 2 로 선언하면
// 전이 판정의 필수 손가락도 따라 옮겨간다 — 슬롯 0 은 특별하지 않다.
TEST_F(GraspControllerTest, ThumbSlotFollowsConfiguredIndex) {
  params_.thumb_finger_index = 2;
  controller_.Init(finger_configs_, params_);

  // 선언된 thumb(2) 이 빠진 쌍 — 슬롯 0·1 이 둘 다 물어도 전이 금지.
  controller_.CommandGrasp();
  EXPECT_EQ(RunWithContactMask({true, true, false}, 3000), GraspPhase::kApproaching);

  // 선언된 thumb(2) + 파트너 0 — 전이해야 한다.
  controller_.Reset();
  controller_.CommandGrasp();
  const auto phase = RunWithContactMask({true, false, true}, 50000);
  EXPECT_NE(phase, GraspPhase::kApproaching);
  EXPECT_NE(phase, GraspPhase::kIdle);
}

// 범위 밖 인덱스는 관례(슬롯 0)로 낙하한다. set_params() 는 noexcept 라 값을
// 거부할 자리가 없고, 조용히 도달 불가능해진 Contact phase 가 더 나쁜 실패다.
TEST_F(GraspControllerTest, OutOfRangeThumbIndexFallsBackToSlotZero) {
  params_.thumb_finger_index = 7;  // num_fingers_ = 3 의 밖
  controller_.Init(finger_configs_, params_);

  // fallback thumb(0) 이 빠지면 전이 금지.
  controller_.CommandGrasp();
  EXPECT_EQ(RunWithContactMask({false, true, true}, 3000), GraspPhase::kApproaching);

  // fallback thumb(0) + 파트너 2 — 전이해야 한다.
  controller_.Reset();
  controller_.CommandGrasp();
  const auto phase = RunWithContactMask({true, false, true}, 50000);
  EXPECT_NE(phase, GraspPhase::kApproaching);
  EXPECT_NE(phase, GraspPhase::kIdle);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. Force Control Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, ForceConvergesToTarget) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // In Holding, forces should be near target
  for (int f = 0; f < kNumFingers; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    EXPECT_NEAR(fs.f_measured, params_.f_target, params_.settle_epsilon * 2)
        << "Finger " << f << " force not converged";
  }
}

TEST_F(GraspControllerTest, OvershootWithinBounds) {
  controller_.CommandGrasp();

  double max_force_seen = 0.0;
  for (int i = 0; i < 100000; ++i) {
    std::array<double, kNumFingers> forces{};
    const auto& states = controller_.finger_states();
    for (int f = 0; f < kNumFingers; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
      forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
      if (forces[idx] > max_force_seen)
        max_force_seen = forces[idx];
    }
    (void)controller_.Update(std::span<const double, 3>(forces), kDt);

    if (controller_.phase() == GraspPhase::kHolding)
      break;
  }

  // Overshoot should be < 20% of target
  EXPECT_LT(max_force_seen, params_.f_target * 1.2)
      << "Force overshoot exceeded 20%: " << max_force_seen;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. Deformation Guard Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, DeformationGuardClampsS) {
  // Very high target force to push deformation to limit
  params_.f_target = 100.0;
  params_.f_ramp_rate = 1000.0;
  controller_.Init(finger_configs_, params_);

  controller_.CommandGrasp(100.0);
  RunUntilPhase(GraspPhase::kForceControl, 50000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kForceControl);

  // Run many steps of force control
  RunSteps(5000);

  // s should not exceed s_at_contact + delta_s_max
  for (int f = 0; f < kNumFingers; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    EXPECT_LE(fs.s, fs.s_at_contact + params_.delta_s_max + 0.01)
        << "Finger " << f << " exceeded deformation limit";
  }
}

TEST_F(GraspControllerTest, DeformationGuardFreezesIntegrator) {
  // Push close to deformation limit
  params_.f_target = 100.0;
  params_.f_ramp_rate = 1000.0;
  params_.delta_s_max = 0.05;  // very small
  controller_.Init(finger_configs_, params_);

  controller_.CommandGrasp(100.0);
  RunUntilPhase(GraspPhase::kForceControl, 50000);
  RunSteps(5000);

  // At least one finger should have frozen integrator
  bool any_frozen = false;
  for (int f = 0; f < kNumFingers; ++f) {
    if (controller_.finger_states()[static_cast<std::size_t>(f)].integrator_frozen) {
      any_frozen = true;
    }
  }
  EXPECT_TRUE(any_frozen) << "No finger had integrator frozen at deformation limit";
}

TEST_F(GraspControllerTest, DeformationGuardProportionalDeceleration) {
  // Set delta_s_max small and check s stays close to limit
  params_.f_target = 50.0;
  params_.f_ramp_rate = 500.0;
  params_.delta_s_max = 0.08;
  controller_.Init(finger_configs_, params_);

  controller_.CommandGrasp(50.0);
  RunUntilPhase(GraspPhase::kForceControl, 50000);
  RunSteps(3000);

  for (int f = 0; f < kNumFingers; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    const double deformation = fs.s - fs.s_at_contact;
    // Should be at or near the limit, not wildly overshooting
    EXPECT_LE(deformation, params_.delta_s_max + 0.01)
        << "Finger " << f << " deformation exceeded limit";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. Force Anomaly Detection Tests
// ═══════════════════════════════════════════════════════════════════════════════

// Grip tightening 은 **rate** 다. 힘을 잃은 동안 기준값은 (tightening − decay)
// N/s 로 오르며, 그 값은 tick 수가 아니라 경과 시간이 정한다.
TEST_F(GraspControllerTest, AnomalyDetectionRaisesForceDesiredAtConfiguredRate) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  const double f_desired_before = controller_.finger_states()[0].f_desired;

  // Simulate sudden force drop (object slipping) by feeding zero forces
  constexpr double kDropSeconds = 1.0;
  const int steps = static_cast<int>(kDropSeconds / kDt);
  std::array<double, 3> zero_force{0.0, 0.0, 0.0};
  for (int i = 0; i < steps; ++i) {
    (void)controller_.Update(std::span<const double, 3>(zero_force), kDt);
  }

  // 첫 tick 은 f_desired == target 이라 decay 가 걸리지 않는다 — 1 tick 분의
  // decay 만큼 오차가 남으므로 tolerance 는 그보다 넉넉히 잡는다.
  const double expected =
      f_desired_before + (params_.grip_tightening_rate - params_.grip_decay_rate) * kDropSeconds;
  EXPECT_NEAR(controller_.finger_states()[0].f_desired, expected, 0.01)
      << "grip tightening 이 rate 가 아니다 (per-tick 이면 상한까지 튄다)";
}

// 같은 결함의 직접 고정: per-tick 비율이었을 때 tightening 총량은 control_rate
// 에 비례했다 (500 Hz 와 1 kHz 가 2배 차이). rate 로 바뀐 지금 같은 경과 시간은
// 주기와 무관하게 같은 상승을 낸다.
TEST_F(GraspControllerTest, GripTighteningIsIndependentOfControlRate) {
  auto rise_after_one_second_of_zero_force = [this](double dt) {
    GraspController c;
    c.Init(finger_configs_, params_);
    c.CommandGrasp();
    // 이 헬퍼는 자체 dt 로 접촉 모델을 돌려야 하므로 fixture 루프를 쓰지 않는다.
    for (int i = 0; i < 400000; ++i) {
      std::array<double, kNumFingers> forces{};
      const auto& st = c.finger_states();
      for (int f = 0; f < kNumFingers; ++f) {
        const auto idx = static_cast<std::size_t>(f);
        forces[idx] =
            SimulateContactForce(st[idx].s, contact_s_, (st[idx].s - st[idx].s_prev) / dt);
      }
      (void)c.Update(std::span<const double, 3>(forces), dt);
      if (c.phase() == GraspPhase::kHolding)
        break;
    }
    EXPECT_EQ(c.phase(), GraspPhase::kHolding) << "dt=" << dt << " 에서 Holding 미도달";
    const double before = c.finger_states()[0].f_desired;
    std::array<double, 3> zero_force{0.0, 0.0, 0.0};
    for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) {
      (void)c.Update(std::span<const double, 3>(zero_force), dt);
    }
    return c.finger_states()[0].f_desired - before;
  };

  const double rise_500hz = rise_after_one_second_of_zero_force(0.002);
  const double rise_1khz = rise_after_one_second_of_zero_force(0.001);
  EXPECT_NEAR(rise_500hz, rise_1khz, 0.005)
      << "500 Hz " << rise_500hz << " vs 1 kHz " << rise_1khz << " — control_rate 의존이 남아 있다";
}

TEST_F(GraspControllerTest, AnomalyDetectionRespectsMaxMultiplier) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // 상한까지 확실히 밀어붙인다: 순 상승률 (0.5 − 0.1) N/s 로 target 2.0 →
  // cap 4.0 에 5 s 필요하므로 20 s 를 돌린다. 짧게 돌리면 상한 근처에 가지도
  // 않은 채 EXPECT_LE 가 공허하게 통과한다.
  std::array<double, 3> zero_force{0.0, 0.0, 0.0};
  for (int i = 0; i < static_cast<int>(20.0 / kDt); ++i) {
    (void)controller_.Update(std::span<const double, 3>(zero_force), kDt);
  }

  const double max_allowed = params_.f_target * params_.f_max_multiplier;
  for (int f = 0; f < kNumFingers; ++f) {
    const double fd = controller_.finger_states()[static_cast<std::size_t>(f)].f_desired;
    EXPECT_LE(fd, max_allowed + 0.01) << "Finger " << f << " f_desired exceeded max multiplier";
    // 공허 통과 가드 — 상한에 실제로 도달했는지 확인해야 위 단언이 의미를 갖는다.
    EXPECT_NEAR(fd, max_allowed, 0.01) << "Finger " << f << " 가 상한까지 오르지 못했다";
  }
}

// deformation guard 가 얼린 integrator 를 anomaly 분기가 매 tick 녹이던 교착의
// 회귀 방지. guard 가 한계에 닿은 뒤에는 힘을 잃은 상태가 계속돼도 integrator 는
// 얼어 있어야 한다 (닫는 권한은 guard 만 갖는다).
TEST_F(GraspControllerTest, AnomalyDoesNotThawDeformationGuardFreeze) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // 힘을 0 으로 떨어뜨려 tightening 을 계속 발화시키면 s 가 guard 한계까지 간다.
  std::array<double, 3> zero_force{0.0, 0.0, 0.0};
  for (int i = 0; i < static_cast<int>(60.0 / kDt); ++i) {
    (void)controller_.Update(std::span<const double, 3>(zero_force), kDt);
  }

  const auto& fs = controller_.finger_states()[0];
  ASSERT_GE(fs.s - fs.s_at_contact, params_.delta_s_max - 1e-9) << "guard 한계에 도달하지 못했다";
  EXPECT_TRUE(fs.integrator_frozen) << "guard 가 얼린 integrator 를 anomaly 분기가 녹였다";
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. Force input pass-through (was: LPF Tests)
// ═══════════════════════════════════════════════════════════════════════════════
//
// SPEC CHANGE. LPFConvergesOnDCInput / LPFAttenuatesHighFrequency lived here and
// are gone, in two pieces:
//
//   * the FILTER LAWS they asserted (DC convergence, 200 Hz attenuation, step
//     overshoot, reset) are pinned at the primitive level in
//     rtc_base/test/test_filters.cpp — those two tests were duplicating them
//     through a controller that happened to own a BesselFilterN;
//   * the WIRING they implied — "something filters the force this law servos
//     on" — moved with the filter itself to the controller that owns the three
//     axes, and is pinned in integrated_bringup/test/test_demo_joint_controller
//     (RotatingHighFrequencyForceIsConstantInMagnitudeButFiltersAway,
//     TheTwoBanksHaveIndependentCutoffs).
//
// What belongs here now is the new contract: the force arrives filtered and this
// class must not alter it. That is the assertion a caller can be broken by.

TEST_F(GraspControllerTest, MeasuredForceIsTheCallerSInputVerbatim) {
  // No CommandGrasp: the Idle→Approaching transition runs ResetFingers(), which
  // zeroes f_measured for that one tick (unchanged behaviour, and not what this
  // test is about). The force latch itself is unconditional at the top of
  // Update(), so Idle is the cleanest place to observe it.
  std::array<double, 3> force{1.5, 1.5, 1.5};
  (void)controller_.Update(std::span<const double, 3>(force), kDt);

  // No settling loop on purpose: with no filter of its own, one tick is enough,
  // and a filter reintroduced here would need many. The absence of the loop is
  // the assertion.
  for (int f = 0; f < kNumFingers; ++f) {
    EXPECT_DOUBLE_EQ(controller_.finger_states()[static_cast<std::size_t>(f)].f_measured, 1.5)
        << "Finger " << f << ": Update() must not transform the force it is given";
  }

  // …and it tracks a step immediately, in both directions.
  std::array<double, 3> lower{0.25, 0.25, 0.25};
  (void)controller_.Update(std::span<const double, 3>(lower), kDt);
  for (int f = 0; f < kNumFingers; ++f) {
    EXPECT_DOUBLE_EQ(controller_.finger_states()[static_cast<std::size_t>(f)].f_measured, 0.25)
        << "Finger " << f;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. Edge Cases
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, JointCommandsAreInterpolated) {
  // CommandGrasp → first Update transitions Idle→Approaching (s still 0)
  // Second Update increments s in Approaching phase
  controller_.CommandGrasp();
  std::array<double, 3> zero{0.0, 0.0, 0.0};
  (void)controller_.Update(std::span<const double, 3>(zero), kDt);  // Idle→Approaching
  auto output =
      controller_.Update(std::span<const double, 3>(zero), kDt);  // s += approach_speed*dt

  const double expected_s = params_.approach_speed * kDt;
  for (int f = 0; f < kNumFingers; ++f) {
    const auto idx = static_cast<std::size_t>(f);
    for (int j = 0; j < kDoF; ++j) {
      const auto jidx = static_cast<std::size_t>(j);
      const double expected_q = (1.0 - expected_s) * finger_configs_[idx].q_open[jidx] +
                                expected_s * finger_configs_[idx].q_close[jidx];
      EXPECT_NEAR(output.q[idx][jidx], expected_q, 1e-6) << "Finger " << f << " joint " << j;
    }
  }
}

TEST_F(GraspControllerTest, ReleaseFromForceControl) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kForceControl, 50000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kForceControl);

  controller_.CommandRelease();
  RunSteps(1);
  EXPECT_EQ(controller_.phase(), GraspPhase::kReleasing);

  RunUntilPhase(GraspPhase::kIdle, 50000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 7. Variable per-finger DoF (proto_1b: thumb:4 / index:3 / middle:2 / ring:1)
// ═══════════════════════════════════════════════════════════════════════════════

// The controller must handle fingers with *different* DoF counts. This exercises
// Init(span), the ragged GraspJointCommands output, and per-finger interpolation.
TEST(GraspControllerRaggedTest, HandlesHeterogeneousFingerDoF) {
  constexpr double kDt = 0.002;
  constexpr int kDofs[4] = {4, 3, 2, 1};  // thumb, index, middle, ring

  std::array<FingerConfig, 4> configs{};
  for (std::size_t f = 0; f < configs.size(); ++f) {
    configs[f].dof = kDofs[f];
    // q_open = 0, q_close[j] = 1.0 for the finger's active joints (rest 0).
    for (int j = 0; j < kDofs[f]; ++j) {
      configs[f].q_close[static_cast<std::size_t>(j)] = 1.0;
    }
  }

  GraspParams params{};
  params.approach_speed = 0.5;

  GraspController controller;
  controller.Init(std::span<const FingerConfig>(configs), params);

  // finger_states() spans the configured finger count.
  EXPECT_EQ(controller.finger_states().size(), 4u);

  // Idle → Approaching, then one step advances s = approach_speed * dt.
  controller.CommandGrasp();
  std::array<double, 4> zero_force{0.0, 0.0, 0.0, 0.0};
  (void)controller.Update(std::span<const double>(zero_force), kDt);  // Idle→Approaching
  const auto out = controller.Update(std::span<const double>(zero_force), kDt);

  ASSERT_EQ(out.num_fingers, 4);
  const double expected_s = params.approach_speed * kDt;
  for (std::size_t f = 0; f < 4; ++f) {
    EXPECT_EQ(out.dof[f], kDofs[f]) << "finger " << f << " dof mismatch";
    // Active joints interpolate to expected_s; joints beyond this finger's
    // DoF must stay exactly zero even though the storage is kMaxDoFPerFinger.
    for (int j = 0; j < kMaxDoFPerFinger; ++j) {
      const double expected_q = (j < kDofs[f]) ? expected_s : 0.0;
      EXPECT_NEAR(out.q[f][static_cast<std::size_t>(j)], expected_q, 1e-6)
          << "finger " << f << " joint " << j;
    }
  }
}

// A short f_raw span (fewer entries than fingers) must not throw / read OOB —
// missing entries are treated as zero force (RT-safe degradation).
TEST(GraspControllerRaggedTest, ShortForceSpanIsZeroPadded) {
  constexpr double kDt = 0.002;
  std::array<FingerConfig, 4> configs{};
  for (auto& c : configs) {
    c.dof = 2;
    c.q_close = {1.0, 1.0};
  }
  GraspParams params{};

  GraspController controller;
  controller.Init(std::span<const FingerConfig>(configs), params);
  controller.CommandGrasp();

  std::array<double, 2> short_force{0.0, 0.0};  // only 2 of 4 fingers
  const auto out = controller.Update(std::span<const double>(short_force), kDt);
  EXPECT_EQ(out.num_fingers, 4);  // no crash, all fingers still reported
}

// ═══════════════════════════════════════════════════════════════════════════════
// 8. Online stiffness estimation (K_contact_est)
// ═══════════════════════════════════════════════════════════════════════════════

namespace {

// These tests sweep object stiffness and tuning together, so they drive a
// controller directly rather than through the fixture's phase helpers. The
// configs and params below are the fixture's, factored out so the two cannot
// drift; only the knobs each test varies are parameters.
constexpr double kRigDt = 0.002;
constexpr double kRigContactS = 0.3;
constexpr double kRigDamping = 1.0;
constexpr std::size_t kRigFingers = 3;

std::array<FingerConfig, kRigFingers> RigConfigs() {
  std::array<FingerConfig, kRigFingers> cfgs{};
  for (auto& fc : cfgs) {
    fc.dof = 3;
    fc.q_open = {0.0, 0.0, 0.0};
    fc.q_close = {0.524, 1.047, 0.785};
  }
  return cfgs;
}

// Same tuning as GraspControllerTest::SetUp, with beta/alpha_ema exposed and
// adaptation switched ON. These tests exercise the estimator itself, so they
// opt out of the shipped pin (K_est_max == the 1.0 seed) that keeps
// K_contact_est frozen everywhere else — see grasp_types.hpp.
//
// Un-pinned is a characterisation rig, not a deployment target: #426 retired
// adaptation (grasp_tuning_guide.md 6.9) and nothing is going to ship this
// configuration. The tests remain because the code they cover remains, and a
// clamp/EMA that is never exercised is a clamp/EMA nobody can refactor safely.
GraspParams RigParams(double beta, double alpha_ema) {
  GraspParams p{};
  p.K_est_max = 400.0;
  p.f_contact_threshold = 0.2;
  p.f_target = 2.0;
  p.f_ramp_rate = 10.0;
  p.approach_speed = 0.5;
  p.release_speed = 0.5;
  p.settle_epsilon = 0.1;
  p.settle_time = 0.05;
  p.contact_settle_time = 0.02;
  p.ds_max = 0.5;
  p.delta_s_max = 0.3;
  p.integral_clamp = 0.5;
  p.Kp_base = 0.05;
  p.Ki_base = 0.01;
  p.alpha_ema = alpha_ema;
  p.beta = beta;
  return p;
}

// A mirror of the force_pi_grasp block deployed to ur5e_p1a / ur5e_p1b. It is a
// copy, so it does not track the YAML automatically — it is here because the
// budget assertion below is a claim about the *deployed* design point, and the
// rig tuning above (2.5x Kp, 5x ramp) would make that claim vacuous.
GraspParams DeployedParams() {
  GraspParams p{};
  p.Kp_base = 0.02;
  p.Ki_base = 0.002;
  p.alpha_ema = 0.95;
  p.beta = 0.3;
  p.K_est_max = 1.0;
  p.f_contact_threshold = 0.8;
  p.f_target = 2.0;
  p.f_ramp_rate = 2.0;
  p.f_max_multiplier = 2.0;
  p.ds_max = 0.05;
  p.delta_s_max = 0.15;
  p.integral_clamp = 0.1;
  p.approach_speed = 0.4;
  p.release_speed = 0.3;
  p.contact_settle_time = 0.1;
  p.settle_epsilon = 0.5;
  p.settle_time = 0.3;
  return p;
}

// One tick against a Kelvin-Voigt object of stiffness true_K, using the same
// f = K*max(delta_s,0) + D*max(ds/dt,0) model as the fixture.
void PlantStep(GraspController& controller, double true_K) {
  std::array<double, kRigFingers> forces{};
  const auto& states = controller.finger_states();
  for (std::size_t f = 0; f < kRigFingers; ++f) {
    const double ds_dt = (states[f].s - states[f].s_prev) / kRigDt;
    const double delta_s = states[f].s - kRigContactS;
    forces[f] =
        (delta_s <= 0.0) ? 0.0 : true_K * delta_s + kRigDamping * std::max(ds_dt, 0.0);
  }
  (void)controller.Update(std::span<const double, 3>(forces), kRigDt);
}

// Ticks spent before kHolding, or max_steps if it is never reached.
int DriveToHolding(GraspController& controller, double true_K, int max_steps) {
  for (int i = 0; i < max_steps; ++i) {
    PlantStep(controller, true_K);
    if (controller.phase() == GraspPhase::kHolding)
      return i + 1;
  }
  return max_steps;
}

// Ticks until finger 0's estimate first reaches frac*true_K, or -1 if never.
int TicksToEstimateFraction(double alpha_ema, double true_K, double frac, int max_steps) {
  GraspController c;
  const auto cfgs = RigConfigs();
  c.Init(cfgs, RigParams(0.03, alpha_ema));
  c.CommandGrasp();
  for (int i = 0; i < max_steps; ++i) {
    PlantStep(c, true_K);
    if (c.finger_states()[0].K_contact_est >= frac * true_K)
      return i;
  }
  return -1;
}

}  // namespace

// The estimator was inert from the first commit: s_prev was latched at tick
// entry next to f_prev, so ComputeAdaptivePI's delta_s = s - s_prev was
// identically zero, the |delta_s| > kDeltaSEpsilon guard never passed, and
// K_contact_est stayed at its 1.0 seed for the life of the grasp. gain_scale
// was therefore the constant 1/(1 + beta) and the controller was fixed-gain
// de-rating, not adaptive. Nothing read the estimate — not GraspState, not a
// log line, not a test — which is why 27 tests passed over it.
TEST(GraspStiffnessEstimationTest, EstimateTracksTrueContactStiffness) {
  for (const double true_K : {20.0, 200.0}) {
    GraspController c;
    const auto cfgs = RigConfigs();
    c.Init(cfgs, RigParams(0.03, 0.95));
    c.CommandGrasp();
    DriveToHolding(c, true_K, 100000);
    ASSERT_EQ(c.phase(), GraspPhase::kHolding) << "true_K=" << true_K;

    const double est = c.finger_states()[0].K_contact_est;
    EXPECT_GT(est, 1.0) << "estimate never left its seed, true_K=" << true_K;
    EXPECT_NEAR(est, true_K, 0.1 * true_K) << "true_K=" << true_K;
  }
}

// alpha_ema is the estimator's smoothing coefficient, so a smaller value must
// reach a given fraction of the true stiffness in fewer ticks. While delta_s
// was pinned at zero this parameter had no observable effect at all.
TEST(GraspStiffnessEstimationTest, AlphaEmaSetsEstimatorTimeConstant) {
  constexpr double kTrueK = 200.0;
  const int slow = TicksToEstimateFraction(0.95, kTrueK, 0.9, 100000);
  const int fast = TicksToEstimateFraction(0.5, kTrueK, 0.9, 100000);

  ASSERT_GE(slow, 0) << "alpha_ema=0.95 never reached 0.9*K";
  ASSERT_GE(fast, 0) << "alpha_ema=0.5 never reached 0.9*K";
  EXPECT_LT(fast, slow) << "fast=" << fast << " slow=" << slow;
}

// The shipped configuration keeps adaptation pinned, and that pin is the whole
// reason this tuning is safe to deploy. K_est_max equals the seed, so
// K_contact_est cannot rise, gain_scale is the constant 1/(1 + beta), and the
// loop is therefore indifferent to force-sensor noise. Unpinned it is not:
// measured at 2 mN of 25 Hz-filtered ripple the same grasps ran 10-30 s against
// a 10 s budget. This test drives the deployed parameters through a noisy force
// lane and asserts both halves — the estimate never leaves its seed, and the
// grasp still lands inside the budget.
TEST(GraspStiffnessEstimationTest, DeployedTuningIsPinnedAndNoiseImmune) {
  constexpr double kFc = 25.0;
  const double a = kRigDt / (1.0 / (2.0 * M_PI * kFc) + kRigDt);
  const double lpf_gain = std::sqrt(a / (2.0 - a));

  for (const double sigma : {0.0, 0.002, 0.02}) {
    for (const double true_K : {10.0, 20.0, 50.0, 200.0}) {
      GraspController c;
      const auto cfgs = RigConfigs();
      c.Init(cfgs, DeployedParams());
      c.CommandGrasp();

      std::mt19937 rng(4242);
      std::normal_distribution<double> nd(0.0, sigma > 0.0 ? sigma / lpf_gain : 0.0);
      double noise = 0.0;
      int ticks = 100000;
      for (int i = 0; i < 100000; ++i) {
        if (sigma > 0.0) noise += a * (nd(rng) - noise);
        std::array<double, kRigFingers> forces{};
        const auto& st = c.finger_states();
        for (std::size_t f = 0; f < kRigFingers; ++f) {
          const double ds_dt = (st[f].s - st[f].s_prev) / kRigDt;
          const double d = st[f].s - kRigContactS;
          forces[f] =
              (d <= 0.0) ? 0.0 : true_K * d + kRigDamping * std::max(ds_dt, 0.0) + noise;
        }
        (void)c.Update(std::span<const double, 3>(forces), kRigDt);
        for (std::size_t f = 0; f < kRigFingers; ++f) {
          ASSERT_LE(c.finger_states()[f].K_contact_est, 1.0)
              << "estimate left its seed: sigma=" << sigma << " K=" << true_K;
        }
        if (c.phase() == GraspPhase::kHolding) {
          ticks = i + 1;
          break;
        }
      }
      ASSERT_EQ(c.phase(), GraspPhase::kHolding) << "sigma=" << sigma << " K=" << true_K;
      EXPECT_LT(static_cast<double>(ticks) * kRigDt, 10.0)
          << "sigma=" << sigma << " K=" << true_K;
    }
  }
}

// beta caps the loop gain rather than merely scaling it: lambda =
// K*Kp_base/(1 + beta*K) saturates at Kp_base/beta, so tau_min = beta/Kp_base
// bounds the settling time no matter how stiff the object is. Pinned, that
// schedule degenerates to the constant 1/(1 + beta) and this test measures what
// the robots actually run — permanently, since #426 retired adaptation
// (grasp_tuning_guide.md 6.9). The bound is why neither value may move alone:
// 0.3 with a live estimator puts every grasp past the 10 s budget, and 0.03
// while pinned changes every grasp's gain by 26% for no adaptation. Both stay.
TEST(GraspStiffnessEstimationTest, DeployedTuningReachesHoldWithinBudget) {
  for (const double true_K : {10.0, 20.0, 50.0, 100.0, 200.0}) {
    GraspController c;
    const auto cfgs = RigConfigs();
    c.Init(cfgs, DeployedParams());
    c.CommandGrasp();
    const int ticks = DriveToHolding(c, true_K, 100000);
    ASSERT_EQ(c.phase(), GraspPhase::kHolding) << "true_K=" << true_K;
    EXPECT_LT(static_cast<double>(ticks) * kRigDt, 10.0) << "true_K=" << true_K;
  }
}

// Regression guard, not a bug detector: this held before the fix too. s_prev is
// the ds/dt source for every plant model in this file, so the fix had to move
// only *when* it is written, never what a caller reads. Once a tick returns,
// s_prev is still exactly the s the previous tick returned.
TEST(GraspStiffnessEstimationTest, SPrevStillHoldsThePreviousTicksS) {
  GraspController c;
  const auto cfgs = RigConfigs();
  c.Init(cfgs, RigParams(0.03, 0.95));
  c.CommandGrasp();

  std::array<double, kRigFingers> s_at_last_return{};
  bool have_previous = false;
  for (int i = 0; i < 4000; ++i) {
    PlantStep(c, 20.0);
    const auto& states = c.finger_states();
    if (have_previous) {
      for (std::size_t f = 0; f < kRigFingers; ++f) {
        ASSERT_DOUBLE_EQ(states[f].s_prev, s_at_last_return[f])
            << "finger " << f << " tick " << i;
      }
    }
    for (std::size_t f = 0; f < kRigFingers; ++f) {
      s_at_last_return[f] = states[f].s;
    }
    have_previous = true;
  }
}

// The estimate is bounded on purpose. K_inst = delta_f/delta_s divides by an
// increment that vanishes as the loop converges, so on a noisy force lane both
// operands are sensor ripple and a single sample can name an arbitrarily stiff
// object. That state does not recover by itself: the collapsed gain_scale
// shrinks ds, which shrinks delta_s, which stops any corrective sample being
// taken. Unbounded, 20 mN of 25 Hz-filtered ripple measured 3364 here.
TEST(GraspStiffnessEstimationTest, EstimateStaysWithinItsClamp) {
  // The regime #426 considered and discarded: adaptation on, beta retuned with
  // it. Kept as coverage because the clamp still has to bound the estimator
  // code that remains — see RigParams and grasp_tuning_guide.md 6.9.
  GraspParams p = DeployedParams();
  p.beta = 0.03;
  p.K_est_max = 400.0;
  GraspController c;
  const auto cfgs = RigConfigs();
  c.Init(cfgs, p);
  c.CommandGrasp();
  DriveToHolding(c, 20.0, 100000);

  // Ripple through the same 1-pole 25 Hz filter the deployed force lane uses,
  // scaled so what the controller sees has std == sigma.
  constexpr double kFc = 25.0;
  const double a = kRigDt / (1.0 / (2.0 * M_PI * kFc) + kRigDt);
  const double lpf_gain = std::sqrt(a / (2.0 - a));
  std::mt19937 rng(20260812);
  std::normal_distribution<double> nd(0.0, 0.02 / lpf_gain);
  double noise = 0.0;
  for (int i = 0; i < 5000; ++i) {
    noise += a * (nd(rng) - noise);
    std::array<double, kRigFingers> forces{};
    const auto& st = c.finger_states();
    for (std::size_t f = 0; f < kRigFingers; ++f) {
      const double ds_dt = (st[f].s - st[f].s_prev) / kRigDt;
      const double d = st[f].s - kRigContactS;
      forces[f] = (d <= 0.0) ? 0.0 : 20.0 * d + kRigDamping * std::max(ds_dt, 0.0) + noise;
    }
    (void)c.Update(std::span<const double, 3>(forces), kRigDt);
    for (std::size_t f = 0; f < kRigFingers; ++f) {
      ASSERT_LE(c.finger_states()[f].K_contact_est, p.K_est_max)
          << "finger " << f << " tick " << i;
    }
  }
}

// ResetFingers() zeroes the whole FingerState. The s_prev write after the FSM
// must not put the pre-reset s back, or s = 0 sits beside a non-zero s_prev and
// anything differencing them - every plant model in this file - sees a spurious
// finger velocity for one tick after a release completes.
TEST(GraspStiffnessEstimationTest, ResetFingersLeavesNoStaleSPrev) {
  GraspController c;
  const auto cfgs = RigConfigs();
  c.Init(cfgs, RigParams(0.03, 0.95));
  c.CommandGrasp();
  DriveToHolding(c, 20.0, 100000);
  c.CommandRelease();

  bool reached_idle = false;
  for (int i = 0; i < 100000; ++i) {
    PlantStep(c, 20.0);
    if (c.phase() == GraspPhase::kIdle) {
      reached_idle = true;
      break;
    }
  }
  ASSERT_TRUE(reached_idle);
  for (std::size_t f = 0; f < kRigFingers; ++f) {
    EXPECT_DOUBLE_EQ(c.finger_states()[f].s, 0.0) << "finger " << f;
    EXPECT_DOUBLE_EQ(c.finger_states()[f].s_prev, 0.0)
        << "finger " << f << ": reset left s_prev holding the pre-reset s";
  }
}
