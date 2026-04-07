#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>

#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

using namespace rtc::grasp;

// ═══════════════════════════════════════════════════════════════════════════════
// Test fixture with Kelvin-Voigt contact model
// ═══════════════════════════════════════════════════════════════════════════════

class GraspControllerTest : public ::testing::Test {
protected:
  static constexpr double kDt = 0.002;         // 500 Hz
  static constexpr double kObjStiffness = 20.0; // [N/delta_s]
  static constexpr double kObjDamping = 1.0;    // [N*s/delta_s]

  void SetUp() override
  {
    // 3 identical fingers: open at 0, close at ~60 deg per joint
    for (auto& fc : finger_configs_) {
      fc.q_open  = {0.0, 0.0, 0.0};
      fc.q_close = {0.524, 1.047, 0.785};
    }

    params_.f_contact_threshold = 0.2;
    params_.f_target = 2.0;
    params_.f_ramp_rate = 10.0;   // fast ramp for tests
    params_.approach_speed = 0.5;  // faster approach for tests
    params_.release_speed = 0.5;
    params_.settle_epsilon = 0.1;
    params_.settle_time = 0.05;    // short settle for tests
    params_.contact_settle_time = 0.02;
    params_.ds_max = 0.5;         // generous rate limit for tests
    params_.delta_s_max = 0.3;
    params_.integral_clamp = 0.5;
    params_.Kp_base = 0.05;
    params_.Ki_base = 0.01;
    params_.alpha_ema = 0.95;
    params_.beta = 0.3;
    params_.lpf_cutoff_hz = 100.0; // high cutoff for tests (minimal filtering)
    params_.control_rate_hz = 500.0;
    params_.df_slip_threshold = 5.0;
    params_.grip_tightening_ratio = 0.15;
    params_.f_max_multiplier = 2.0;

    controller_.Init(finger_configs_, params_);
  }

  // Kelvin-Voigt contact model: force = K*max(delta_s,0) + D*max(ds/dt,0)
  double SimulateContactForce(double s, double s_at_contact, double ds_dt) const
  {
    const double delta_s = s - s_at_contact;
    if (delta_s <= 0.0) return 0.0;
    return kObjStiffness * delta_s + kObjDamping * std::max(ds_dt, 0.0);
  }

  // Run simulation loop, returns forces at each step.
  // contact_s: the s value where object is located (contact starts here)
  void RunUntilPhase(GraspPhase target_phase, int max_steps = 50000)
  {
    for (int i = 0; i < max_steps; ++i) {
      // Compute forces based on current finger s values
      std::array<double, kNumGraspFingers> forces{};
      const auto& states = controller_.finger_states();
      for (int f = 0; f < kNumGraspFingers; ++f) {
        const auto idx = static_cast<std::size_t>(f);
        const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
        forces[idx] = SimulateContactForce(
          states[idx].s, contact_s_, ds_dt);
      }

      (void)controller_.Update(std::span<const double, 3>(forces), kDt);

      if (controller_.phase() == target_phase) return;
    }
  }

  // Run N steps with force feedback
  void RunSteps(int n)
  {
    for (int i = 0; i < n; ++i) {
      std::array<double, kNumGraspFingers> forces{};
      const auto& states = controller_.finger_states();
      for (int f = 0; f < kNumGraspFingers; ++f) {
        const auto idx = static_cast<std::size_t>(f);
        const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
        forces[idx] = SimulateContactForce(
          states[idx].s, contact_s_, ds_dt);
      }
      (void)controller_.Update(std::span<const double, 3>(forces), kDt);
    }
  }

  std::array<FingerConfig, kNumGraspFingers> finger_configs_{};
  GraspParams params_{};
  GraspController controller_;
  double contact_s_{0.3};  // object contact at s=0.3
};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. State Machine Transition Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, InitialPhaseIsIdle)
{
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);
}

TEST_F(GraspControllerTest, IdleToApproachingOnCommand)
{
  controller_.CommandGrasp();
  std::array<double, 3> zero{0.0, 0.0, 0.0};
  (void)controller_.Update(std::span<const double, 3>(zero), kDt);
  EXPECT_EQ(controller_.phase(), GraspPhase::kApproaching);
}

TEST_F(GraspControllerTest, ApproachingDetectsContact)
{
  controller_.CommandGrasp();

  // Run until approaching, then simulate until contact
  RunUntilPhase(GraspPhase::kContact);
  EXPECT_EQ(controller_.phase(), GraspPhase::kContact);

  // Verify all fingers detected contact
  for (int f = 0; f < kNumGraspFingers; ++f) {
    EXPECT_TRUE(controller_.finger_states()[static_cast<std::size_t>(f)].contact_detected);
    EXPECT_NEAR(controller_.finger_states()[static_cast<std::size_t>(f)].s_at_contact,
                contact_s_, 0.05);  // within tolerance of approach speed * dt
  }
}

TEST_F(GraspControllerTest, ContactToForceControlAfterDwell)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kForceControl);
  EXPECT_EQ(controller_.phase(), GraspPhase::kForceControl);
}

TEST_F(GraspControllerTest, ForceControlToHolding)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kHolding);
}

TEST_F(GraspControllerTest, HoldingToReleasingOnCommand)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  controller_.CommandRelease();
  RunSteps(1);
  EXPECT_EQ(controller_.phase(), GraspPhase::kReleasing);
}

TEST_F(GraspControllerTest, ReleasingToIdleOnComplete)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  controller_.CommandRelease();

  RunUntilPhase(GraspPhase::kIdle, 50000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);

  // All fingers should be reset
  for (int f = 0; f < kNumGraspFingers; ++f) {
    EXPECT_NEAR(controller_.finger_states()[static_cast<std::size_t>(f)].s, 0.0, 0.02);
  }
}

TEST_F(GraspControllerTest, GraspFailsWhenNoContact)
{
  // Set contact_s very high (object unreachable before s=1.0)
  contact_s_ = 1.5;

  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kIdle, 50000);
  // Should return to Idle because s maxed out without contact
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. Force Control Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, ForceConvergesToTarget)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // In Holding, forces should be near target
  for (int f = 0; f < kNumGraspFingers; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    EXPECT_NEAR(fs.f_measured, params_.f_target, params_.settle_epsilon * 2)
      << "Finger " << f << " force not converged";
  }
}

TEST_F(GraspControllerTest, OvershootWithinBounds)
{
  controller_.CommandGrasp();

  double max_force_seen = 0.0;
  for (int i = 0; i < 100000; ++i) {
    std::array<double, kNumGraspFingers> forces{};
    const auto& states = controller_.finger_states();
    for (int f = 0; f < kNumGraspFingers; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const double ds_dt = (states[idx].s - states[idx].s_prev) / kDt;
      forces[idx] = SimulateContactForce(states[idx].s, contact_s_, ds_dt);
      if (forces[idx] > max_force_seen) max_force_seen = forces[idx];
    }
    (void)controller_.Update(std::span<const double, 3>(forces), kDt);

    if (controller_.phase() == GraspPhase::kHolding) break;
  }

  // Overshoot should be < 20% of target
  EXPECT_LT(max_force_seen, params_.f_target * 1.2)
    << "Force overshoot exceeded 20%: " << max_force_seen;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. Deformation Guard Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, DeformationGuardClampsS)
{
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
  for (int f = 0; f < kNumGraspFingers; ++f) {
    const auto& fs = controller_.finger_states()[static_cast<std::size_t>(f)];
    EXPECT_LE(fs.s, fs.s_at_contact + params_.delta_s_max + 0.01)
      << "Finger " << f << " exceeded deformation limit";
  }
}

TEST_F(GraspControllerTest, DeformationGuardFreezesIntegrator)
{
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
  for (int f = 0; f < kNumGraspFingers; ++f) {
    if (controller_.finger_states()[static_cast<std::size_t>(f)].integrator_frozen) {
      any_frozen = true;
    }
  }
  EXPECT_TRUE(any_frozen) << "No finger had integrator frozen at deformation limit";
}

TEST_F(GraspControllerTest, DeformationGuardProportionalDeceleration)
{
  // Set delta_s_max small and check s stays close to limit
  params_.f_target = 50.0;
  params_.f_ramp_rate = 500.0;
  params_.delta_s_max = 0.08;
  controller_.Init(finger_configs_, params_);

  controller_.CommandGrasp(50.0);
  RunUntilPhase(GraspPhase::kForceControl, 50000);
  RunSteps(3000);

  for (int f = 0; f < kNumGraspFingers; ++f) {
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

TEST_F(GraspControllerTest, AnomalyDetectionIncreasesForceDesired)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kHolding, 100000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kHolding);

  // Record f_desired before anomaly
  const double f_desired_before =
    controller_.finger_states()[0].f_desired;

  // Simulate sudden force drop (object slipping) by feeding zero forces
  std::array<double, 3> zero_force{0.0, 0.0, 0.0};
  for (int i = 0; i < 10; ++i) {
    (void)controller_.Update(std::span<const double, 3>(zero_force), kDt);
  }

  // f_desired should have increased due to grip tightening
  const double f_desired_after =
    controller_.finger_states()[0].f_desired;
  EXPECT_GT(f_desired_after, f_desired_before)
    << "f_desired did not increase after force anomaly";
}

TEST_F(GraspControllerTest, AnomalyDetectionRespectsMaxMultiplier)
{
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
  for (int f = 0; f < kNumGraspFingers; ++f) {
    EXPECT_LE(controller_.finger_states()[static_cast<std::size_t>(f)].f_desired,
              max_allowed + 0.01)
      << "Finger " << f << " f_desired exceeded max multiplier";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. LPF Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, LPFConvergesOnDCInput)
{
  // Use lower cutoff to see filter effect
  params_.lpf_cutoff_hz = 25.0;
  controller_.Init(finger_configs_, params_);

  // Feed constant force
  std::array<double, 3> dc_force{1.5, 1.5, 1.5};
  for (int i = 0; i < 500; ++i) {  // 1 second at 500Hz
    (void)controller_.Update(std::span<const double, 3>(dc_force), kDt);
  }

  // Filtered force should converge to DC value
  for (int f = 0; f < kNumGraspFingers; ++f) {
    EXPECT_NEAR(controller_.finger_states()[static_cast<std::size_t>(f)].f_measured,
                1.5, 0.01)
      << "Finger " << f << " LPF did not converge on DC input";
  }
}

TEST_F(GraspControllerTest, LPFAttenuatesHighFrequency)
{
  params_.lpf_cutoff_hz = 25.0;
  controller_.Init(finger_configs_, params_);

  // Feed high-frequency sinusoidal noise (200 Hz, well above 25 Hz cutoff)
  double max_filtered = 0.0;
  for (int i = 0; i < 1000; ++i) {
    const double t = static_cast<double>(i) * kDt;
    const double noise = 1.0 * std::sin(2.0 * std::numbers::pi * 200.0 * t);
    std::array<double, 3> noisy_force{noise, noise, noise};
    (void)controller_.Update(std::span<const double, 3>(noisy_force), kDt);

    // Track max filtered force after settling (skip first 100 samples)
    if (i > 100) {
      const double f_mag = std::abs(
        controller_.finger_states()[0].f_measured);
      if (f_mag > max_filtered) max_filtered = f_mag;
    }
  }

  // 200 Hz should be heavily attenuated by 25 Hz 4th-order Bessel
  // Attenuation at 200/25 = 8x ratio, 4th order ~ -80 dB/decade beyond cutoff
  EXPECT_LT(max_filtered, 0.05)
    << "High-frequency noise not sufficiently attenuated: " << max_filtered;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. Edge Cases
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(GraspControllerTest, JointCommandsAreInterpolated)
{
  // CommandGrasp → first Update transitions Idle→Approaching (s still 0)
  // Second Update increments s in Approaching phase
  controller_.CommandGrasp();
  std::array<double, 3> zero{0.0, 0.0, 0.0};
  (void)controller_.Update(std::span<const double, 3>(zero), kDt);  // Idle→Approaching
  auto output = controller_.Update(std::span<const double, 3>(zero), kDt);  // s += approach_speed*dt

  const double expected_s = params_.approach_speed * kDt;
  for (int f = 0; f < kNumGraspFingers; ++f) {
    const auto idx = static_cast<std::size_t>(f);
    for (int j = 0; j < kDoFPerFinger; ++j) {
      const auto jidx = static_cast<std::size_t>(j);
      const double expected_q =
        (1.0 - expected_s) * finger_configs_[idx].q_open[jidx]
        + expected_s * finger_configs_[idx].q_close[jidx];
      EXPECT_NEAR(output.q[idx][jidx], expected_q, 1e-6)
        << "Finger " << f << " joint " << j;
    }
  }
}

TEST_F(GraspControllerTest, ReleaseFromForceControl)
{
  controller_.CommandGrasp();
  RunUntilPhase(GraspPhase::kForceControl, 50000);
  ASSERT_EQ(controller_.phase(), GraspPhase::kForceControl);

  controller_.CommandRelease();
  RunSteps(1);
  EXPECT_EQ(controller_.phase(), GraspPhase::kReleasing);

  RunUntilPhase(GraspPhase::kIdle, 50000);
  EXPECT_EQ(controller_.phase(), GraspPhase::kIdle);
}
