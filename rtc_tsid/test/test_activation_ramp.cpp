// Stage A-5b: ContactState activation ramp + deadband behaviour.

#include "rtc_tsid/types/wbc_types.hpp"

#include <gtest/gtest.h>

namespace rtc::tsid {
namespace {

constexpr double kDt = 1.0 / 500.0;  // 500 Hz RT tick

class ActivationRampTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mgr_.contacts.resize(1);
    mgr_.contacts[0].contact_dim = 3;
    mgr_.max_contacts = 1;
    mgr_.max_contact_vars = 3;

    cs_.Init(1);
    // Start fully inactive (s_i = 0, active = false) for the rising-ramp tests.
    cs_.contacts[0].activation = 0.0;
    cs_.contacts[0].activation_target = 0.0;
    cs_.contacts[0].active = false;
  }

  ContactManagerConfig mgr_;
  ContactState cs_;
};

// Linear ramp toward target = 1 over t_ramp = 0.05 s. Convergence monotonic,
// terminal value exactly clamped to 1.0, never exceeds 1.0.
TEST_F(ActivationRampTest, MonotonicConvergence) {
  cs_.SetActivationTarget(0, 1.0, 0.05);

  double prev_s = cs_.contacts[0].activation;
  int monotonic_violations = 0;
  for (int k = 0; k < 100; ++k) {
    cs_.UpdateActivation(kDt);
    const double s = cs_.contacts[0].activation;
    EXPECT_LE(s, 1.0 + 1e-12);
    EXPECT_GE(s, 0.0 - 1e-12);
    if (s + 1e-12 < prev_s) {
      ++monotonic_violations;
    }
    prev_s = s;
  }
  EXPECT_EQ(monotonic_violations, 0);
  // After 100 ticks (200 ms) at 50 ms ramp, target reached and clamped.
  EXPECT_NEAR(cs_.contacts[0].activation, 1.0, 1e-12);
  EXPECT_TRUE(cs_.contacts[0].active);
}

// Jump-reduction: compare effective force-task row magnitude on the first
// post-transition tick under (a) instant on (t_ramp ≈ 0) vs (b) 50 ms ramp.
// √s scaling means a 50 ms ramp gives s = dt/t_ramp = 0.04 on tick 1 →
// √0.04 = 0.2 row magnitude, vs 1.0 for instant. ≥50% reduction trivially
// holds (5× reduction).
TEST_F(ActivationRampTest, JumpReductionFiftyPercent) {
  // Scenario A: instant on (t_ramp = 0).
  cs_.SetActivationTarget(0, 1.0, 0.0);
  cs_.UpdateActivation(kDt);
  const double s_instant = cs_.contacts[0].activation;

  // Scenario B: 50 ms ramp from s=0.
  cs_.contacts[0].activation = 0.0;
  cs_.contacts[0].active = false;
  cs_.SetActivationTarget(0, 1.0, 0.05);
  cs_.UpdateActivation(kDt);
  const double s_ramp = cs_.contacts[0].activation;

  // Effective task row magnitude scales as √s.
  const double mag_instant = std::sqrt(s_instant);
  const double mag_ramp = std::sqrt(s_ramp);

  EXPECT_NEAR(s_instant, 1.0, 1e-12) << "instant ramp must snap to target";
  EXPECT_LT(s_ramp, 0.1) << "ramped s on tick 1 must be ≪ 1";
  EXPECT_LT(mag_ramp, 0.5 * mag_instant) << "ramp must reduce first-tick magnitude by ≥ 50 %";
}

// Deadband: as s_i crosses kActivationDeadband (1e-3), the legacy
// `active : bool` flips automatically inside UpdateActivation. Downstream
// `if (!active)` paths key off this flip.
TEST_F(ActivationRampTest, DeadbandTransitionsActiveFlag) {
  // Starts at s = 0, active = false.
  EXPECT_FALSE(cs_.contacts[0].active);

  // Rising: 0 → 1 over 0.1 s. Active must flip true once s ≥ deadband.
  cs_.SetActivationTarget(0, 1.0, 0.1);

  bool flipped_true = false;
  for (int k = 0; k < 100; ++k) {
    cs_.UpdateActivation(kDt);
    if (cs_.contacts[0].active) {
      flipped_true = true;
      EXPECT_GE(cs_.contacts[0].activation, kActivationDeadband - 1e-15);
      break;
    }
  }
  EXPECT_TRUE(flipped_true);

  // Saturate to s = 1.
  for (int k = 0; k < 200; ++k) {
    cs_.UpdateActivation(kDt);
  }
  EXPECT_TRUE(cs_.contacts[0].active);

  // Falling: 1 → 0. Active must flip false once s drops below deadband.
  cs_.SetActivationTarget(0, 0.0, 0.05);
  bool flipped_false = false;
  for (int k = 0; k < 200; ++k) {
    cs_.UpdateActivation(kDt);
    if (!cs_.contacts[0].active) {
      flipped_false = true;
      EXPECT_LT(cs_.contacts[0].activation, kActivationDeadband + 1e-15);
      break;
    }
  }
  EXPECT_TRUE(flipped_false);
}

// SetActivationTarget out-of-range idx and target are clamped/no-op.
TEST_F(ActivationRampTest, SetActivationTargetClamping) {
  cs_.SetActivationTarget(-1, 1.0, 0.05);  // no-op
  cs_.SetActivationTarget(99, 1.0, 0.05);  // no-op
  EXPECT_DOUBLE_EQ(cs_.contacts[0].activation_target, 0.0);

  // target > 1 clamps to 1; target < 0 clamps to 0.
  cs_.SetActivationTarget(0, 5.0, 0.05);
  EXPECT_DOUBLE_EQ(cs_.contacts[0].activation_target, 1.0);
  cs_.SetActivationTarget(0, -1.0, 0.05);
  EXPECT_DOUBLE_EQ(cs_.contacts[0].activation_target, 0.0);
}

}  // namespace
}  // namespace rtc::tsid
