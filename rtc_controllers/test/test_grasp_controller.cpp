#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

using namespace rtc::grasp;

namespace {
// This fixture exercises the historical 3-identical-fingers × 3-DoF layout
// (assm_v1). The controller itself is DoF-agnostic; RaggedFingerLayout below
// covers the variable per-finger DoF path (thumb:4/index:3/middle:2/ring:1).
constexpr int kNumFingers = 3;
constexpr int kDoF = 3;
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// Test fixture with Kelvin-Voigt contact model
// ═══════════════════════════════════════════════════════════════════════════════

class GraspControllerTest : public ::testing::Test {
 protected:
  static constexpr double kDt = 0.002;           // 500 Hz
  static constexpr double kObjStiffness = 20.0;  // [N/delta_s]
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
    params_.beta = 0.3;
    params_.df_slip_threshold = 5.0;
    params_.grip_tightening_ratio = 0.15;
    params_.f_max_multiplier = 2.0;

    controller_.Init(finger_configs_, params_);
  }

  // Kelvin-Voigt contact model: force = K*max(delta_s,0) + D*max(ds/dt,0)
  double SimulateContactForce(double s, double s_at_contact, double ds_dt) const {
    const double delta_s = s - s_at_contact;
    if (delta_s <= 0.0)
      return 0.0;
    return kObjStiffness * delta_s + kObjDamping * std::max(ds_dt, 0.0);
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

// Approaching → Contact 전이는 thumb(0) + index(1) 기준만 본다.
// middle(2) 가 영영 접촉하지 못해도 grasp 는 진행되어야 한다.
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

TEST_F(GraspControllerTest, AnomalyDetectionIncreasesForceDesired) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // Record f_desired before anomaly
  const double f_desired_before = controller_.finger_states()[0].f_desired;

  // Simulate sudden force drop (object slipping) by feeding zero forces
  std::array<double, 3> zero_force{0.0, 0.0, 0.0};
  for (int i = 0; i < 10; ++i) {
    (void)controller_.Update(std::span<const double, 3>(zero_force), kDt);
  }

  // f_desired should have increased due to grip tightening
  const double f_desired_after = controller_.finger_states()[0].f_desired;
  EXPECT_GT(f_desired_after, f_desired_before) << "f_desired did not increase after force anomaly";
}

TEST_F(GraspControllerTest, AnomalyDetectionRespectsMaxMultiplier) {
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // Feed zero forces for many steps to trigger repeated tightening
  std::array<double, 3> zero_force{0.0, 0.0, 0.0};
  for (int i = 0; i < 500; ++i) {
    (void)controller_.Update(std::span<const double, 3>(zero_force), kDt);
  }

  // f_desired should not exceed f_target * f_max_multiplier
  const double max_allowed = params_.f_target * params_.f_max_multiplier;
  for (int f = 0; f < kNumFingers; ++f) {
    EXPECT_LE(controller_.finger_states()[static_cast<std::size_t>(f)].f_desired,
              max_allowed + 0.01)
        << "Finger " << f << " f_desired exceeded max multiplier";
  }
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
