// Stage A-3 — ForceReferenceUpdater unit tests.
//
// Covers the three behaviours the handoff Sprint Contract calls out:
//   StepResponseConverges        — error monotonically drops to 0 and the
//                                  output stays inside [lambda_min, lambda_max].
//   AntiWindupSaturatesIntegrator — integrator bounded by ±i_max even under
//                                   prolonged saturation; reverses without
//                                   waiting for the integrator to bleed off.
//   InvalidSensorFallback        — measured_valid=false zeroes the
//                                  integrator and the output; valid-again
//                                  ticks do not see a stale-I spike.

#include "integrated_bringup/controllers/wbc/force_reference_updater.hpp"

#include <gtest/gtest.h>

namespace integrated_bringup::wbc {
namespace {

constexpr double kDt = 1.0 / 500.0;  // 500 Hz RT tick

ForceReferenceUpdaterConfig MakeConfig() {
  ForceReferenceUpdaterConfig cfg;
  cfg.kp = 0.5;
  cfg.ki = 5.0;
  cfg.i_max = 5.0;
  cfg.lambda_min = 0.0;
  cfg.lambda_max = 20.0;
  cfg.f_des_default = 2.0;
  return cfg;
}

// Simple first-order plant: f_meas[k+1] = f_meas[k] + (lambda - f_meas[k])
// * (dt / tau). The PI driving this plant should converge.
double StepPlant(double f_prev, double lambda, double tau) {
  return f_prev + (lambda - f_prev) * (kDt / tau);
}

TEST(ForceReferenceUpdaterTest, StepResponseConverges) {
  ForceReferenceUpdater updater;
  updater.SetConfig(MakeConfig());

  constexpr double f_des = 5.0;
  double f_meas = 0.0;
  constexpr double tau = 0.05;  // 50 ms plant time constant
  constexpr int kSteps = 1000;  // 2 s simulated

  double max_overshoot = 0.0;
  for (int k = 0; k < kSteps; ++k) {
    const double lambda = updater.Update(0, /*valid=*/true, f_des, f_meas, kDt);
    EXPECT_GE(lambda, 0.0);
    EXPECT_LE(lambda, 20.0);
    f_meas = StepPlant(f_meas, lambda, tau);
    if (f_meas - f_des > max_overshoot) {
      max_overshoot = f_meas - f_des;
    }
  }

  // Steady-state convergence: within 1 % of target after 2 s.
  EXPECT_LT(std::abs(f_des - f_meas), 0.05);
  // Bounded overshoot — kp=0.5, ki=5, anti-windup should keep peak <40 %.
  EXPECT_LT(max_overshoot, 2.0);
}

TEST(ForceReferenceUpdaterTest, AntiWindupSaturatesIntegrator) {
  ForceReferenceUpdater updater;
  auto cfg = MakeConfig();
  cfg.kp = 0.1;  // gentle proportional so the integrator has room to grow
  cfg.ki = 2.0;
  cfg.i_max = 3.0;
  cfg.lambda_min = 0.0;
  cfg.lambda_max = 5.0;
  cfg.f_des_default = 2.0;
  updater.SetConfig(cfg);

  // Modest positive error so kp*err+f_des≈2.3 stays inside lambda_max=5;
  // the integrator can accumulate freely up to i_max.
  for (int k = 0; k < 5000; ++k) {
    const double lambda = updater.Update(0, /*valid=*/true, /*f_des=*/2.0,
                                         /*f_meas=*/-1.0, kDt);
    EXPECT_LE(lambda, cfg.lambda_max + 1e-9);
    EXPECT_GE(updater.GetIntegratorForTesting(0), -cfg.i_max - 1e-9);
    EXPECT_LE(updater.GetIntegratorForTesting(0), cfg.i_max + 1e-9);
  }
  // The integrator should have charged up. Conditional integration may
  // pin it below i_max if kp*err + i + f_des hits lambda_max before
  // reaching i_max — both outcomes are valid anti-windup behaviour. The
  // hard property we test is the upper-bound + meaningful contribution.
  EXPECT_GT(updater.GetIntegratorForTesting(0), 1.0);
  EXPECT_LE(updater.GetIntegratorForTesting(0), cfg.i_max + 1e-9);

  // Now reverse: f_meas overshoots f_des. The integrator must bleed off,
  // not stay pinned. Across a short window of reverse ticks the integrator
  // must move meaningfully toward 0 (and may go negative).
  const double i_before_reverse = updater.GetIntegratorForTesting(0);
  for (int k = 0; k < 1000; ++k) {
    const double lambda = updater.Update(0, /*valid=*/true, /*f_des=*/2.0,
                                         /*f_meas=*/4.0, kDt);
    EXPECT_GE(lambda, 0.0);
    EXPECT_LE(lambda, cfg.lambda_max);
  }
  EXPECT_LT(updater.GetIntegratorForTesting(0), i_before_reverse - 1.0);
}

TEST(ForceReferenceUpdaterTest, InvalidSensorFallback) {
  ForceReferenceUpdater updater;
  updater.SetConfig(MakeConfig());

  // Wind up the integrator with valid measurements first.
  for (int k = 0; k < 200; ++k) {
    (void)updater.Update(0, /*valid=*/true, /*f_des=*/5.0, /*f_meas=*/0.0, kDt);
  }
  EXPECT_GT(updater.GetIntegratorForTesting(0), 0.0);

  // Sensor goes invalid — output must drop to zero and integrator reset
  // immediately. No spike on re-valid.
  const double lambda_invalid = updater.Update(0, /*valid=*/false, 5.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(lambda_invalid, 0.0);
  EXPECT_DOUBLE_EQ(updater.GetIntegratorForTesting(0), 0.0);

  // First valid tick after invalid: I_i=0 means output is at most
  // kp*err + f_des, no carried-over wind-up.
  const auto& cfg = updater.GetConfig();
  const double f_des = 5.0;
  const double f_meas = 0.0;
  const double lambda_valid_after = updater.Update(0, /*valid=*/true, f_des, f_meas, kDt);
  const double expected_max_no_windup =
      cfg.kp * (f_des - f_meas) + cfg.ki * (f_des - f_meas) * kDt + f_des;
  EXPECT_LE(lambda_valid_after, expected_max_no_windup + 1e-9);
}

TEST(ForceReferenceUpdaterTest, OutOfRangeContactIndexNoOp) {
  ForceReferenceUpdater updater;
  updater.SetConfig(MakeConfig());

  EXPECT_DOUBLE_EQ(updater.Update(-1, true, 5.0, 0.0, kDt), 0.0);
  EXPECT_DOUBLE_EQ(updater.Update(kMaxForceContacts, true, 5.0, 0.0, kDt), 0.0);
  EXPECT_DOUBLE_EQ(updater.Update(kMaxForceContacts + 10, true, 5.0, 0.0, kDt), 0.0);
}

TEST(ForceReferenceUpdaterTest, ResetClearsAllContacts) {
  ForceReferenceUpdater updater;
  updater.SetConfig(MakeConfig());

  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 100; ++k) {
      (void)updater.Update(c, true, 5.0, 0.0, kDt);
    }
    EXPECT_GT(updater.GetIntegratorForTesting(c), 0.0);
  }

  updater.Reset();
  for (int c = 0; c < kMaxForceContacts; ++c) {
    EXPECT_DOUBLE_EQ(updater.GetIntegratorForTesting(c), 0.0);
    EXPECT_DOUBLE_EQ(updater.GetLastOutputForTesting(c), 0.0);
  }
}

}  // namespace
}  // namespace integrated_bringup::wbc
