// ── test_pull_estimator_leakage_bound.cpp ────────────────────────────────────
// #177 crit#6 (1): does the estimator's own math honour the grip->in-plane
// leakage bound its shipped config DECLARES?
//
// WHAT THE CRITERION IS, AND WHAT IT IS NOT. `alignment_error_rad` (delta,
// shipped 0.035 rad in all three profiles) is the estimator's statement that
// the true contact normals sit within delta of the plane normal it is handed,
// and it publishes `leakage_bound = sum_i |f_n,i| sin(delta)` every tick from
// exactly that expression (pull_force_estimator.cpp). So asserting
// "leakage_bound <= sum|f_n| sin delta" is an IDENTITY and green means nothing.
// The criterion #177 §S2 actually gates on is the regression
//
//     d |F_hat| / d sum|f_n|  <=  sin(alignment_error_rad)
//
// i.e. sweep grip force with the true external load held at a known constant,
// and check how much of the extra grip leaks into the in-plane estimate. The
// two sides of that inequality come from different code: the left from the
// projection P_par of the summed contact forces, the right from the per-contact
// normal projections. Nothing forces them to agree.
//
// WHY THIS IS A UNIT TEST AND NOT A SIM RUN. crit#6 (2) -- whether THIS hand's
// physical pinch really stays within 0.035 rad -- is out of scope (#177 comment
// 5644776857); (1) is a property of the math, and crit#4 already established
// the known-load positive control on the same axis. The MuJoCo route was tried
// and does not close it: on a weightless free disk the restoring spring BECOMES
// the true load (measured 0 -> 1.09 N across one ladder) and without the spring
// the disk cannot be grasped at all (measured 0.00 N through full closure), so
// "true load held constant" and "grip force sweepable" are not simultaneously
// obtainable in that fixture (2026-09-12; .claude/skills/verify/SKILL.md p1b).
//
// THE ORACLE IS ON PAPER, not a second copy of the implementation. An exactly
// opposed squeeze cancels in the sum and leaks nothing for ANY delta, so it
// cannot test the bound. Tilting the two sides of the pinch in OPPOSITE senses
// by d leaves a residual that is computable by hand (see PullTiltedSqueezeForces):
//
//     |F_hat| = 2 S sin(d),   sum|f_n| = 2 S cos(d)   =>   slope = tan(d)
//
// independent of the squeeze S. So the sweep must recover tan(d) — a far
// stronger assertion than the inequality alone, because a wrong projection, a
// dropped rotation or a mis-signed contact normal all move it.
//
// The negative control below is what keeps the inequality from being vacuous:
// with d deliberately set above the declared delta, the same sweep must EXCEED
// the bound. A test that cannot fail is not a sensor.
// ─────────────────────────────────────────────────────────────────────────────
#include "pull_known_load_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::ConfigurePullEstimatorWiring;
using integrated_bringup::DemoSharedConfig;
using integrated_bringup::PullEstimatorWiring;
namespace fx = integrated_bringup::testfx;

// Past the one-tick axis lag and well past the profiles' 5 Hz output filter at
// 500 Hz, so every rung is read at its settled value.
constexpr int kSettleTicks = 1500;

// The squeeze ladder [N]. Chosen against the tightest shipped gates rather than
// the loosest: every rung clears ur5e_p1b's 0.5 N contact hysteresis on the
// weakest tip (0.5 S cos d >= 1 N at the bottom rung) and the top rung stays
// under its 15 N per-axis saturation, which is the lowest of the three.
constexpr std::array<double, 5> kSqueezeLadderN = {2.0, 4.0, 6.0, 8.0, 10.0};

// The premise gate #177 §S2 puts on a hardware session, applied to this sweep
// too: with no grip spread a slope is fitting something other than leakage.
constexpr double kMinSumFnSpreadN = 2.0;

struct Rung {
  double sum_fn;     // as the runbook reads it: leakage_bound / sin(delta)
  double magnitude;  // |F_hat|
  std::uint8_t contact_mask;
  int contact_count;
  bool valid;
};

/// Ordinary least squares slope of y on x, with an intercept.
double FitSlope(const std::vector<double>& x, const std::vector<double>& y) {
  const auto n = static_cast<double>(x.size());
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    sx += x[i];
    sy += y[i];
    sxx += x[i] * x[i];
    sxy += x[i] * y[i];
  }
  return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}

class ShippedPullLeakage : public ::testing::TestWithParam<fx::PullProfileCase> {
 protected:
  void SetUp() override {
    const fx::PullProfileCase& p = GetParam();
    cfg_ = fx::LoadShippedPullProfile(RTC_DEMO_SHARED_CONFIG_DIR, p.profile);
    ASSERT_NO_THROW(ConfigurePullEstimatorWiring(cfg_, fx::kPullRateHz,
                                                 std::span<const std::string>(p.tip_links), w_));
    ASSERT_TRUE(w_.enabled()) << "the shipped " << p.profile << " profile disabled the estimator";
    ASSERT_EQ(w_.num_contacts, p.expected_contacts)
        << p.profile << ": tip_names enrols a different number of tips than this case expects";
    // The thumb must be contact 0: the tilted construction below puts the +n
    // side on it, and the wiring gives only the thumb the -n contact normal.
    ASSERT_EQ(w_.thumb_contact, 0) << p.profile;
    ASSERT_EQ(w_.normal_source, integrated_bringup::PullPlaneNormalSource::kPinchGeometry)
        << p.profile << ": this sweep drives the plane normal through fingertip geometry";
    delta_ = cfg_.pull_estimator_params.alignment_error_rad;
    ASSERT_GT(delta_, 0.0) << p.profile << ": no leakage bound is declared at all";
  }

  /// Run the squeeze ladder at a given true normal misalignment.
  std::vector<Rung> Sweep(double tilt_rad) {
    std::vector<Rung> rungs;
    rungs.reserve(kSqueezeLadderN.size());
    for (const double squeeze : kSqueezeLadderN) {
      const std::array<fx::FtSample, 4> samples =
          fx::PullLaneSamplesFrom(fx::PullTiltedSqueezeForces(squeeze, tilt_rad));
      // Copied, not bound by reference: RunPullTicks returns a reference into
      // the estimator (which outlives this), but taking it from a call whose
      // arguments include temporaries trips -Wdangling-reference.
      const rtc::grasp::PullEstimate est =
          fx::RunPullTicks(w_, samples, /*grasp_detected=*/false, kSettleTicks);
      rungs.push_back(Rung{est.leakage_bound / std::sin(delta_), est.magnitude, est.contact_mask,
                           est.valid_contact_count, est.valid});
    }
    return rungs;
  }

  DemoSharedConfig cfg_;
  PullEstimatorWiring w_;
  double delta_{0.0};
};

// ── 1. The declared bound holds, and the slope is the one on paper ──────────
TEST_P(ShippedPullLeakage, GripLeaksNoFasterThanTheProfileDeclares) {
  const std::string profile = GetParam().profile;
  // Half the declared bound: comfortably inside it, and expressed relative to
  // the shipped value so that raising or lowering delta moves the test with it
  // instead of leaving a stale literal behind.
  const double tilt = delta_ / 2.0;
  const std::vector<Rung> rungs = Sweep(tilt);

  std::vector<double> x;
  std::vector<double> y;
  for (std::size_t i = 0; i < rungs.size(); ++i) {
    const Rung& r = rungs[i];
    ASSERT_TRUE(r.valid) << profile << " rung " << i << ": the estimate was not valid";
    EXPECT_EQ(r.contact_mask, rungs[0].contact_mask)
        << profile << " rung " << i
        << ": the pinch set changed mid-ladder, which makes the fit a geometry change and "
           "not a leakage measurement (the same premise #177 §S2 gates a session on)";
    // sum|f_n| = 2 S cos(d), derived in the fixture. Checking it here also
    // validates the substitution the hardware runbook makes when it reads grip
    // force off the estimator as leakage_bound / sin(delta).
    const double expected_sum_fn = 2.0 * kSqueezeLadderN[i] * std::cos(tilt);
    EXPECT_NEAR(r.sum_fn, expected_sum_fn, 1e-5 * expected_sum_fn) << profile << " rung " << i;
    x.push_back(r.sum_fn);
    y.push_back(r.magnitude);
  }

  const double spread = x.back() - x.front();
  ASSERT_GE(spread, kMinSumFnSpreadN)
      << profile << ": the ladder produced only " << spread
      << " N of grip spread, so the slope below would be fitting noise";

  const double slope = FitSlope(x, y);
  const double bound = std::sin(delta_);

  // The strong form: the slope is tan(d) exactly, from the hand derivation.
  EXPECT_NEAR(slope, std::tan(tilt), 1e-5)
      << profile
      << ": the measured leakage slope is not the one the geometry implies — "
         "a wrong projection plane, a dropped fingertip rotation or a mis-signed "
         "contact normal all land here";

  // The criterion itself.
  EXPECT_LE(slope, bound)
      << profile << ": d|F_hat|/d sum|f_n| = " << slope << " exceeds the declared bound sin("
      << delta_ << ") = " << bound
      << ". That does not mean the estimator is broken; it means alignment_error_rad "
         "understates the leakage this math produces and must be raised to at least "
      << std::asin(std::min(std::abs(slope), 1.0)) << " rad.";
}

// ── 2. Negative control — the gate can fail ────────────────────────────────
//
// Without this the test above is unfalsifiable: an estimator that returned a
// constant zero in-plane force would pass it. Here the true misalignment is put
// ABOVE what the profile declares, and the same sweep must breach the bound.
TEST_P(ShippedPullLeakage, AMisalignmentBeyondTheDeclarationIsCaught) {
  const std::string profile = GetParam().profile;
  const double tilt = 3.0 * delta_;
  const std::vector<Rung> rungs = Sweep(tilt);

  std::vector<double> x;
  std::vector<double> y;
  for (const Rung& r : rungs) {
    ASSERT_TRUE(r.valid) << profile;
    x.push_back(r.sum_fn);
    y.push_back(r.magnitude);
  }
  const double slope = FitSlope(x, y);
  EXPECT_NEAR(slope, std::tan(tilt), 1e-5) << profile;
  EXPECT_GT(slope, std::sin(delta_))
      << profile
      << ": a pinch misaligned by three times the declared delta did NOT breach the "
         "declared bound — the gate above cannot fail, so it is not measuring anything";
}

INSTANTIATE_TEST_SUITE_P(EveryShippedProfile, ShippedPullLeakage,
                         ::testing::ValuesIn(fx::ShippedPullProfiles()),
                         [](const ::testing::TestParamInfo<fx::PullProfileCase>& i) {
                           return std::string(i.param.profile);
                         });

}  // namespace
