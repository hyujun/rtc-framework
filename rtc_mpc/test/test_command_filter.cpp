// CommandFilter (reference/command_filter.hpp): the W2-F01 [SPRINT] PR A gates.
//
// The step gate is judged against the closed-form step response of the analog
// 4th-order Bessel low-pass (prototype poles from scipy.signal.besselap(4,
// norm='mag') × 2π f_c, residue sum) — not against rtc::BesselFilterN's own
// constants — sampled half a period late: the bilinear transform's
// trapezoidal rule puts the discrete step half a sample behind the analog one,
// leaving an O(h²) gap. A first-order EMA, a 2× cutoff or a Butterworth miss
// the band by ≥ 0.3. Every refusal is next to an accepted twin.
#include "rtc_mpc/reference/command_filter.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::mpc::CommandFilter;
using rtc::mpc::CommandFilterConfig;
using rtc::mpc::CommandFilterError;
using rtc::mpc::FilteredCommand;
using rtc::mpc::kCommandAxes;
using rtc::mpc::RawVelocityCommand;

constexpr double kCutoffHz = 2.4;
constexpr std::array<double, kCommandAxes> kUMax = {2.0, 1.2, 1.0, 0.1};
constexpr double kInf = std::numeric_limits<double>::infinity();
const double kNaN = std::numeric_limits<double>::quiet_NaN();

CommandFilterConfig Config(double fs) {
  CommandFilterConfig c;
  c.sample_rate_hz = fs;
  c.cutoff_hz = kCutoffHz;
  c.u_max = kUMax;
  c.accel_max = {0.0, 0.0, 0.0, 0.0};
  c.t_stale = 0.5;
  c.base_height = 0.6;
  c.h_min = 0.5;
  c.h_max = 1.0;
  return c;
}

CommandFilter Made(const CommandFilterConfig& c) {
  CommandFilter f;
  EXPECT_EQ(f.Init(c), CommandFilterError::kNoError);
  return f;
}

std::int64_t PeriodNs(double fs) {
  return std::llround(1e9 / fs);
}

// u = [v_x, v_y, ω_z, ḣ] → the Twist-shaped raw command.
RawVelocityCommand Raw(const std::array<double, kCommandAxes>& u) {
  RawVelocityCommand r;
  r.linear_x = u[0];
  r.linear_y = u[1];
  r.angular_z = u[2];
  r.linear_z = u[3];
  return r;
}

// Every axis at `s` × its bound.
std::array<double, kCommandAxes> Scaled(double s) {
  std::array<double, kCommandAxes> u{};
  for (std::size_t i = 0; i < kCommandAxes; ++i) {
    u[i] = s * kUMax[i];
  }
  return u;
}

std::int64_t Ns(double seconds) {
  return std::llround(seconds * 1e9);
}

std::array<double, 3> Velocity(const FilteredCommand& o) {
  return {o.v_x, o.v_y, o.omega_z};
}

// Closed-form step response of the analog 4th-order Bessel (−3 dB at f_c).
double AnalyticBesselStep(double t, double fc) {
  using C = std::complex<double>;
  const double w = 2.0 * std::numbers::pi * fc;
  const std::array<C, 4> p = {C(-0.9952087643502724, 1.2571057394546652) * w,
                              C(-0.9952087643502724, -1.2571057394546652) * w,
                              C(-1.3700678305514433, 0.41024971749375183) * w,
                              C(-1.3700678305514433, -0.41024971749375183) * w};
  C k(1.0, 0.0);
  for (const C& pi : p) {
    k *= -pi;
  }
  C y(1.0, 0.0);
  for (std::size_t i = 0; i < p.size(); ++i) {
    C den = p[i];
    for (std::size_t j = 0; j < p.size(); ++j) {
      if (j != i) {
        den *= p[i] - p[j];
      }
    }
    y += k / den * std::exp(p[i] * t);
  }
  return y.real();
}

// ── The oracle itself ────────────────────────────────────────────────────────

TEST(CommandFilterOracle, AnalyticStepMatchesTheScipyReferencePoints) {
  EXPECT_NEAR(AnalyticBesselStep(0.09, kCutoffHz), 0.193758, 1e-5);
  EXPECT_NEAR(AnalyticBesselStep(0.15, kCutoffHz), 0.583595, 1e-5);
  EXPECT_NEAR(AnalyticBesselStep(0.29, kCutoffHz), 1.003780, 1e-5);
  double y_max = 0.0;
  double t_max = 0.0;
  for (int i = 0; i <= 100000; ++i) {
    const double t = 1e-5 * i;
    const double y = AnalyticBesselStep(t, kCutoffHz);
    if (y > y_max) {
      y_max = y;
      t_max = t;
    }
  }
  EXPECT_NEAR(y_max, 1.008354, 1e-5);
  EXPECT_NEAR(t_max, 0.320, 1e-3);
}

// ── Step response ────────────────────────────────────────────────────────────

TEST(CommandFilter, StepTracksTheAnalyticBesselStepAtEachRate) {
  const std::array<double, 3> rates = {50.0, 100.0, 200.0};
  const std::array<double, 3> bands = {0.010, 0.003, 0.001};
  for (std::size_t r = 0; r < rates.size(); ++r) {
    const double fs = rates[r];
    SCOPED_TRACE("f_s = " + std::to_string(fs));
    CommandFilter f = Made(Config(fs));
    const std::int64_t period = PeriodNs(fs);
    const std::array<double, kCommandAxes> u =
        Scaled(0.5);  // the final clamp cannot hide the overshoot
    const int n = static_cast<int>(3.0 * fs);
    std::array<double, 3> err{};
    std::array<double, 3> peak{};
    std::array<double, 3> late{};
    for (int k = 0; k < n; ++k) {
      const std::int64_t now = k * period;
      const auto v = Velocity(f.Step(Raw(u), now, now));
      for (std::size_t i = 0; i < 3; ++i) {
        const double y = v[i] / u[i];
        if (k < static_cast<int>(fs)) {  // the first second
          const double ya = AnalyticBesselStep((k + 0.5) / fs, kCutoffHz);
          err[i] = std::max(err[i], std::abs(y - ya));
        }
        peak[i] = std::max(peak[i], y);
        if (k >= static_cast<int>(2.0 * fs)) {
          late[i] = std::max(late[i], std::abs(y - 1.0));
        }
      }
    }
    for (std::size_t i = 0; i < 3; ++i) {
      SCOPED_TRACE("axis " + std::to_string(i));
      EXPECT_LE(err[i], bands[r]);
      EXPECT_GE(peak[i] - 1.0, 0.005);  // the lower bound refuses an EMA
      EXPECT_LE(peak[i] - 1.0, 0.015);
      EXPECT_LE(late[i], 1e-9);
    }
  }
}

// ── Finite gate · clamp ──────────────────────────────────────────────────────

// Axis i in 0.5 s block b sees kind (b + i) mod 6, so every axis meets every
// kind while the others are elsewhere.
double HostileSample(std::size_t kind, double u_max, std::mt19937_64& rng) {
  switch (kind) {
    case 0:
      return u_max;
    case 1:
      return -3.0 * u_max;
    case 2:
      return kInf;
    case 3:
      return kNaN;
    case 4:
      return std::uniform_real_distribution<double>(-5.0 * u_max, 5.0 * u_max)(rng);
    default:
      return -kInf;
  }
}

TEST(CommandFilter, NonFiniteAxesHoldTheirLastValueAndEveryOutputStaysInBounds) {
  const double fs = 100.0;
  const CommandFilterConfig cfg = Config(fs);
  CommandFilter hostile = Made(cfg);
  CommandFilter twin = Made(cfg);  // fed the held value where `hostile` sees a non-finite one
  std::mt19937_64 rng(7);
  std::array<double, kCommandAxes> held{};
  const std::int64_t period = PeriodNs(fs);
  const int n = static_cast<int>(6.0 * fs);  // 12 blocks: every kind twice per axis
  std::uint64_t expected_nonfinite = 0;
  for (int k = 0; k < n; ++k) {
    const auto block = static_cast<std::size_t>(k / static_cast<int>(0.5 * fs));
    std::array<double, kCommandAxes> u{};
    std::array<double, kCommandAxes> u_twin{};
    bool any = false;
    for (std::size_t i = 0; i < kCommandAxes; ++i) {
      u[i] = HostileSample((block + i) % 6, kUMax[i], rng);
      if (std::isfinite(u[i])) {
        held[i] = u[i];
      } else {
        any = true;
      }
      u_twin[i] = held[i];
    }
    expected_nonfinite += any ? 1U : 0U;
    const std::int64_t now = k * period;
    const FilteredCommand a = hostile.Step(Raw(u), now, now);
    const FilteredCommand b = twin.Step(Raw(u_twin), now, now);
    const auto va = Velocity(a);
    const auto vb = Velocity(b);
    for (std::size_t i = 0; i < 3; ++i) {
      ASSERT_TRUE(std::isfinite(va[i])) << "tick " << k;
      ASSERT_LE(std::abs(va[i]), kUMax[i]) << "tick " << k << " axis " << i;
      ASSERT_EQ(va[i], vb[i]) << "tick " << k << " axis " << i;
    }
    ASSERT_TRUE(std::isfinite(a.base_height));
    ASSERT_GE(a.base_height, cfg.h_min);
    ASSERT_LE(a.base_height, cfg.h_max);
    ASSERT_EQ(a.base_height, b.base_height) << "tick " << k;
    ASSERT_EQ(a.nonfinite_count, expected_nonfinite) << "tick " << k;
    ASSERT_EQ(b.nonfinite_count, 0U);
  }
}

TEST(CommandFilter, AnOversizedStepIsClampedBeforeTheFilter) {
  for (const double fs : {50.0, 100.0, 200.0}) {
    SCOPED_TRACE("f_s = " + std::to_string(fs));
    CommandFilter f = Made(Config(fs));
    const std::int64_t period = PeriodNs(fs);
    const std::array<double, kCommandAxes> u = Scaled(10.0);
    double t50 = -1.0;
    for (int k = 0; k < static_cast<int>(fs) && t50 < 0.0; ++k) {
      const std::int64_t now = k * period;
      if (f.Step(Raw(u), now, now).v_x >= 0.5 * kUMax[0]) {
        t50 = k / fs;
      }
    }
    EXPECT_NEAR(t50, 0.14, 0.02);  // without the pre-clamp: 0.06
  }
}

// ── Slew ─────────────────────────────────────────────────────────────────────

TEST(CommandFilter, SlewBoundsTheFilterInputAndOffPassesTheFullJump) {
  const double fs = 100.0;
  const std::int64_t period = PeriodNs(fs);
  CommandFilterConfig on = Config(fs);
  for (std::size_t i = 0; i < kCommandAxes; ++i) {
    on.accel_max[i] = 0.1 * kUMax[i] * fs;  // a_max / f_s = 0.1 · u_max
  }
  CommandFilter slewed = Made(on);
  CommandFilter plain = Made(Config(fs));
  std::array<double, kCommandAxes> prev_in{};
  std::array<double, kCommandAxes> prev_plain_in{};
  std::array<double, 3> prev_out{};
  double max_out_step_ratio = 0.0;
  bool full_jump_seen = false;
  for (int k = 0; k < static_cast<int>(4.0 * fs); ++k) {
    const double sign = (k / static_cast<int>(0.5 * fs)) % 2 == 0 ? 1.0 : -1.0;
    std::array<double, kCommandAxes> u{};
    for (std::size_t i = 0; i < kCommandAxes; ++i) {
      u[i] = sign * kUMax[i];
    }
    const std::int64_t now = k * period;
    const auto v = Velocity(slewed.Step(Raw(u), now, now));
    (void)plain.Step(Raw(u), now, now);
    for (std::size_t i = 0; i < kCommandAxes; ++i) {
      const double step = on.accel_max[i] / fs;
      ASSERT_LE(std::abs(slewed.limited_input()[i] - prev_in[i]), step + 1e-12);
      const double plain_step = std::abs(plain.limited_input()[i] - prev_plain_in[i]);
      if (k > 0 && plain_step != 0.0) {
        ASSERT_EQ(plain_step, 2.0 * kUMax[i]);  // off: the stage passes the whole jump
        full_jump_seen = true;
      }
      if (i < 3) {
        max_out_step_ratio = std::max(max_out_step_ratio, std::abs(v[i] - prev_out[i]) / step);
        prev_out[i] = v[i];
      }
      prev_in[i] = slewed.limited_input()[i];
      prev_plain_in[i] = plain.limited_input()[i];
    }
  }
  EXPECT_TRUE(full_jump_seen);
  EXPECT_LE(max_out_step_ratio, 1.02);
}

TEST(CommandFilter, SlewLimitMovesAtMostOneStepAndZeroIsOff) {
  EXPECT_DOUBLE_EQ(rtc::mpc::SlewLimit(0.0, 1.0, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(rtc::mpc::SlewLimit(0.0, -1.0, 0.1), -0.1);
  EXPECT_DOUBLE_EQ(rtc::mpc::SlewLimit(0.0, 0.05, 0.1), 0.05);
  EXPECT_DOUBLE_EQ(rtc::mpc::SlewLimit(0.0, 1.0, 0.0), 1.0);
}

// ── Stale ────────────────────────────────────────────────────────────────────

TEST(CommandFilter, StaleInputDecaysToRestAndAFreshStampResumesTracking) {
  const double fs = 100.0;
  const CommandFilterConfig cfg = Config(fs);
  CommandFilter f = Made(cfg);
  const std::int64_t period = PeriodNs(fs);
  std::array<double, kCommandAxes> u = Scaled(0.5);
  u[3] = 0.0;  // height held still here; its stale path has its own test
  int k = 0;
  for (; k < static_cast<int>(3.0 * fs); ++k) {
    (void)f.Step(Raw(u), k * period, k * period);  // settled at 0.5 · u_max
  }
  const std::int64_t t_f = (k - 1) * period;  // the last fresh stamp
  const std::int64_t t_stale = Ns(cfg.t_stale);
  std::uint64_t stale_ticks = 0;
  for (; k < static_cast<int>(5.0 * fs); ++k) {
    const std::int64_t now = k * period;
    const FilteredCommand& o = f.Step(Raw(u), t_f, now);
    const auto v = Velocity(o);
    if (now - t_f > t_stale) {
      ++stale_ticks;
    }
    ASSERT_EQ(o.stale_count, stale_ticks) << "tick " << k;
    for (std::size_t i = 0; i < 3; ++i) {
      if (now <= t_f + t_stale) {  // equality is still fresh
        ASSERT_NEAR(v[i], u[i], 1e-9) << "tick " << k << " axis " << i;
      }
      if (now >= t_f + t_stale + Ns(0.30)) {
        ASSERT_LE(std::abs(v[i]), 0.01 * kUMax[i]) << "tick " << k << " axis " << i;
      }
    }
  }
  ASSERT_GT(stale_ticks, 0U);
  // A fresh stamp: the filter input jumps back the same tick and the output rises.
  const double before = f.output().v_x;
  const std::int64_t now = k * period;
  const FilteredCommand& o = f.Step(Raw(u), now, now);
  EXPECT_EQ(f.limited_input()[0], u[0]);
  EXPECT_GT(o.v_x, before);
  EXPECT_EQ(o.stale_count, stale_ticks);
  EXPECT_TRUE(o.valid);
}

TEST(CommandFilter, AStampFromTheFutureIsStaleBeyondTheTimeout) {
  const double fs = 100.0;
  CommandFilter f = Made(Config(fs));
  const std::int64_t t_stale = Ns(0.5);
  const std::int64_t now = 10 * Ns(1.0);
  const RawVelocityCommand r = Raw({1.0, 0.0, 0.0, 0.0});
  EXPECT_TRUE(f.Step(r, now + t_stale, now).fresh);  // a clock ahead by t_stale: still fresh
  EXPECT_FALSE(f.Step(r, now + t_stale + 1, now).fresh);
  EXPECT_FALSE(f.Step(r, std::numeric_limits<std::int64_t>::min(), now).fresh);  // overflow
  EXPECT_EQ(f.output().stale_count, 2U);
  EXPECT_EQ(f.limited_input()[0], 0.0);
}

TEST(CommandFilter, ANonFiniteValueIsNotCountedWhileTheInputIsStale) {
  CommandFilter f = Made(Config(100.0));
  RawVelocityCommand r = Raw({kNaN, 0.0, 0.0, 0.0});
  const FilteredCommand& o = f.Step(r, 0, Ns(1.0));  // 1 s old
  EXPECT_FALSE(o.fresh);
  EXPECT_EQ(o.stale_count, 1U);
  EXPECT_EQ(o.nonfinite_count, 0U);
  EXPECT_EQ(f.Step(r, Ns(1.0), Ns(1.0)).nonfinite_count, 1U);
}

TEST(CommandFilter, AStaleHeightRateStopsTheHeight) {
  const double fs = 100.0;
  CommandFilter f = Made(Config(fs));
  const std::int64_t period = PeriodNs(fs);
  const std::array<double, kCommandAxes> u = {0.0, 0.0, 0.0, 0.1};
  int k = 0;
  for (; k < static_cast<int>(2.0 * fs); ++k) {
    (void)f.Step(Raw(u), k * period, k * period);
  }
  const std::int64_t t_f = (k - 1) * period;
  const std::int64_t t_onset = t_f + Ns(0.5);  // last fresh tick
  double h_onset = 0.0;
  double h_2s = 0.0;
  double h_3s = 0.0;
  for (; k < static_cast<int>(6.0 * fs); ++k) {
    const std::int64_t now = k * period;
    const double h = f.Step(Raw(u), t_f, now).base_height;
    if (now == t_onset) {
      h_onset = h;
    }
    if (now == t_onset + Ns(2.0)) {
      h_2s = h;
    }
    if (now == t_onset + Ns(3.0)) {
      h_3s = h;
    }
  }
  EXPECT_NEAR(h_2s - h_onset, 0.014, 0.001);  // 0.1 · group delay (0.140 s)
  EXPECT_NEAR(h_3s, h_2s, 1e-12);
}

// ── Height ───────────────────────────────────────────────────────────────────

TEST(CommandFilter, HeightIntegratesTheFilteredRate) {
  for (const double fs : {50.0, 100.0, 200.0}) {
    SCOPED_TRACE("f_s = " + std::to_string(fs));
    // ḣ = 0.1 at half scale, as in the step gate: at ḣ == hdot_max the final
    // clamp cuts the ~0.8 % Bessel overshoot and with it 7e-5 m of the 0.2 m.
    CommandFilterConfig cfg = Config(fs);
    cfg.u_max[3] = 0.2;
    CommandFilter f = Made(cfg);
    const std::int64_t period = PeriodNs(fs);
    const double h0 = f.output().base_height;
    double h_2s = 0.0;
    const int n2 = static_cast<int>(2.0 * fs);
    for (int k = 0; k < static_cast<int>(3.0 * fs); ++k) {
      const std::array<double, kCommandAxes> u = {0.0, 0.0, 0.0, k < n2 ? 0.1 : 0.0};
      const double h = f.Step(Raw(u), k * period, k * period).base_height;
      ASSERT_GE(h, cfg.h_min);
      ASSERT_LE(h, cfg.h_max);
      if (k == n2 - 1) {
        h_2s = h;
      }
    }
    EXPECT_NEAR(h_2s - h0, 0.186, 0.001);  // 0.2 − 0.1 · t_gd; a raw-rate integral gives 0.200
    EXPECT_NEAR(f.output().base_height - h0, 0.2, 1e-6);
  }
}

TEST(CommandFilter, HeightSaturatesExactlyAndDoesNotWindUp) {
  const double fs = 100.0;
  const CommandFilterConfig cfg = Config(fs);
  const std::int64_t period = PeriodNs(fs);
  CommandFilter f = Made(cfg);
  int k = 0;
  auto step = [&](double hdot) {
    const std::array<double, kCommandAxes> u = {0.0, 0.0, 0.0, hdot};
    const double h = f.Step(Raw(u), k * period, k * period).base_height;
    ++k;
    return h;
  };
  double h = 0.0;
  for (int i = 0; i < static_cast<int>(10.0 * fs); ++i) {
    h = step(-0.1);
  }
  EXPECT_EQ(h, cfg.h_min);
  for (int i = 0; i < static_cast<int>(10.0 * fs); ++i) {
    h = step(0.1);
  }
  EXPECT_EQ(h, cfg.h_max);

  // From h_max − 0.05: push until h_max, 1 s more, then down for 0.5 s.
  ASSERT_EQ(f.Reset(cfg.h_max - 0.05), CommandFilterError::kNoError);
  k = 0;
  while (step(0.1) < cfg.h_max) {
  }
  for (int i = 0; i < static_cast<int>(1.0 * fs); ++i) {
    (void)step(0.1);
  }
  for (int i = 0; i < static_cast<int>(0.5 * fs); ++i) {
    h = step(-0.1);
  }
  EXPECT_LE(h, cfg.h_max - 0.03);  // a wound-up integrator is still at h_max
}

TEST(CommandFilter, ResetRefusesAHeightOutsideTheRange) {
  const CommandFilterConfig cfg = Config(100.0);
  CommandFilter f = Made(cfg);
  EXPECT_EQ(f.Reset(cfg.h_min), CommandFilterError::kNoError);
  EXPECT_EQ(f.Reset(cfg.h_max), CommandFilterError::kNoError);
  EXPECT_EQ(f.Reset(0.7), CommandFilterError::kNoError);
  EXPECT_EQ(f.Reset(cfg.h_min - 1e-9), CommandFilterError::kHeightOutOfRange);
  EXPECT_EQ(f.Reset(cfg.h_max + 1e-9), CommandFilterError::kHeightOutOfRange);
  EXPECT_EQ(f.Reset(kNaN), CommandFilterError::kHeightOutOfRange);
  EXPECT_EQ(f.output().base_height, 0.7);  // a refusal changes nothing
  CommandFilter fresh;
  EXPECT_EQ(fresh.Reset(0.7), CommandFilterError::kNotInitialised);
}

TEST(CommandFilter, ResetStartsFromRest) {
  const double fs = 100.0;
  CommandFilter f = Made(Config(fs));
  const std::int64_t period = PeriodNs(fs);
  const std::array<double, kCommandAxes> u = {1.0, -0.5, 0.3, 0.05};
  for (int k = 0; k < 50; ++k) {
    RawVelocityCommand r = Raw(u);
    r.angular_x = k % 2 == 0 ? kNaN : 0.0;
    (void)f.Step(r, k % 3 == 0 ? 0 : k * period, k * period);
  }
  ASSERT_EQ(f.Reset(0.75), CommandFilterError::kNoError);
  const FilteredCommand& r0 = f.output();
  EXPECT_EQ(r0.sequence, 0U);
  EXPECT_FALSE(r0.valid);
  EXPECT_EQ(r0.nonfinite_count + r0.stale_count + r0.ignored_axis_count, 0U);

  // First Step(0) after Reset: exactly rest (bitwise), the height h0.
  const std::int64_t later = 1000 * period;
  const FilteredCommand& s = f.Step(Raw({0.0, 0.0, 0.0, 0.0}), 0, later);  // stale
  EXPECT_EQ(s.v_x, 0.0);
  EXPECT_EQ(s.v_y, 0.0);
  EXPECT_EQ(s.omega_z, 0.0);
  EXPECT_EQ(s.base_height, 0.75);
  EXPECT_EQ(s.sequence, 1U);
  EXPECT_FALSE(s.valid);  // nothing fresh yet
  EXPECT_TRUE(f.Step(Raw({0.0, 0.0, 0.0, 0.0}), later, later + period).valid);
}

TEST(CommandFilter, IgnoredAxesAreCountedAndChangeNoOutput) {
  const double fs = 100.0;
  CommandFilter with = Made(Config(fs));
  CommandFilter without = Made(Config(fs));
  const std::int64_t period = PeriodNs(fs);
  const std::array<double, kCommandAxes> u = {0.8, -0.4, 0.5, 0.05};
  const int n = 200;
  for (int k = 0; k < n; ++k) {
    RawVelocityCommand r = Raw(u);
    r.angular_x = k % 3 == 0 ? kNaN : 0.7;
    r.angular_y = -kInf;
    const FilteredCommand a = with.Step(r, k * period, k * period);
    const FilteredCommand b = without.Step(Raw(u), k * period, k * period);
    ASSERT_EQ(a.v_x, b.v_x);
    ASSERT_EQ(a.v_y, b.v_y);
    ASSERT_EQ(a.omega_z, b.omega_z);
    ASSERT_EQ(a.base_height, b.base_height);
    ASSERT_EQ(a.nonfinite_count, 0U);
  }
  EXPECT_EQ(with.output().ignored_axis_count, static_cast<std::uint64_t>(n));
  EXPECT_EQ(without.output().ignored_axis_count, 0U);
}

TEST(CommandFilter, StepBeforeInitIsInert) {
  CommandFilter f;
  const FilteredCommand& o = f.Step(Raw({1.0, 1.0, 1.0, 1.0}), 0, 0);
  EXPECT_EQ(o.sequence, 0U);
  EXPECT_EQ(o.v_x, 0.0);
  EXPECT_FALSE(f.initialized());
}

// ── YAML ─────────────────────────────────────────────────────────────────────

constexpr const char* kYaml = R"(
sample_rate_hz: 100.0
cutoff_hz: 2.4
v_max: [2.0, 1.2, 1.0]
hdot_max: 0.1
accel_max: [0.0, 0.0, 0.0, 0.0]
t_stale: 0.5
base_height: 0.8
h_min: 0.5
h_max: 1.0
)";

CommandFilterError LoadWith(const std::string& key, const std::string& value) {
  YAML::Node n = YAML::Load(kYaml);
  if (!key.empty()) {
    n[key] = YAML::Load(value);
  }
  CommandFilterConfig c;
  return rtc::mpc::LoadCommandFilterConfig(n, c);
}

TEST(CommandFilterYaml, LoadsTheSchemaIntoTheConfig) {
  CommandFilterConfig c;
  ASSERT_EQ(rtc::mpc::LoadCommandFilterConfig(YAML::Load(kYaml), c), CommandFilterError::kNoError);
  EXPECT_EQ(c.sample_rate_hz, 100.0);
  EXPECT_EQ(c.cutoff_hz, 2.4);
  EXPECT_EQ(c.u_max, kUMax);
  EXPECT_EQ(c.t_stale, 0.5);
  EXPECT_EQ(c.base_height, 0.8);
  EXPECT_EQ(c.h_min, 0.5);
  EXPECT_EQ(c.h_max, 1.0);
}

TEST(CommandFilterYaml, RefusesEachBadValueNextToAnAcceptedTwin) {
  using E = CommandFilterError;

  struct Case {
    const char* key;
    const char* bad;
    E error;
    const char* good;
  };

  const std::vector<Case> cases = {
      {"cutoff_hz", "0.0", E::kInvalidCutoff, "0.01"},
      {"cutoff_hz", "-1.0", E::kInvalidCutoff, "2.4"},
      {"cutoff_hz", "50.0", E::kInvalidCutoff, "49.9"},
      {"sample_rate_hz", "0.0", E::kInvalidSampleRate, "5.0"},
      {"sample_rate_hz", "-100.0", E::kInvalidSampleRate, "100.0"},
      {"v_max", "[2.0, -0.1, 1.0]", E::kNegativeLimit, "[2.0, 0.0, 1.0]"},
      {"hdot_max", "-0.1", E::kNegativeLimit, "0.0"},
      {"accel_max", "[0.0, 0.0, 0.0, -1.0]", E::kNegativeLimit, "[0.0, 0.0, 0.0, 1.0]"},
      {"t_stale", "0.0", E::kInvalidStaleTimeout, "0.01"},  // ≥ one period (f_s 100)
      {"t_stale", "0.0099", E::kInvalidStaleTimeout, "1e6"},
      {"t_stale", "1.0000001e6", E::kInvalidStaleTimeout, "0.5"},
      {"h_min", "1.1", E::kInvalidHeightRange, "0.8"},  // h_min == base_height passes
      {"base_height", "1.01", E::kInvalidHeightRange, "1.0"},
      {"base_height", "0.49", E::kInvalidHeightRange, "0.5"},
      {"h_max", ".nan", E::kNonFinite, "1.0"},
      {"cutoff_hz", ".inf", E::kNonFinite, "2.4"},
      {"v_max", "[2.0, .nan, 1.0]", E::kNonFinite, "[2.0, 1.2, 1.0]"},
      {"v_max", "[2.0, 1.2]", E::kWrongType, "[2.0, 1.2, 1.0]"},
      {"accel_max", "0.0", E::kWrongType, "[0.0, 0.0, 0.0, 0.0]"},
      {"t_stale", "fast", E::kWrongType, "0.5"},
      {"h_max", "[1.0]", E::kWrongType, "1.0"},
      {"t_stale", "~", E::kMissingKey, "0.5"},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(std::string(c.key) + ": " + c.bad + " / " + c.good);
    EXPECT_EQ(LoadWith(c.key, c.bad), c.error);
    EXPECT_EQ(LoadWith(c.key, c.good), E::kNoError);
  }
}

TEST(CommandFilterYaml, PinnedHeightIsAccepted) {
  YAML::Node n = YAML::Load(kYaml);
  n["h_min"] = 0.8;
  n["h_max"] = 0.8;
  CommandFilterConfig c;
  EXPECT_EQ(rtc::mpc::LoadCommandFilterConfig(n, c), CommandFilterError::kNoError);
}

TEST(CommandFilterYaml, RefusesAnUnknownKeyAMissingKeyAndANonMap) {
  YAML::Node unknown = YAML::Load(kYaml);
  unknown["cutof_hz"] = 2.4;
  YAML::Node missing = YAML::Load(kYaml);
  missing.remove("h_max");
  CommandFilterConfig c;
  c.cutoff_hz = 9.0;
  EXPECT_EQ(rtc::mpc::LoadCommandFilterConfig(unknown, c), CommandFilterError::kUnknownKey);
  EXPECT_EQ(rtc::mpc::LoadCommandFilterConfig(missing, c), CommandFilterError::kMissingKey);
  EXPECT_EQ(rtc::mpc::LoadCommandFilterConfig(YAML::Load("[1, 2]"), c),
            CommandFilterError::kNotAMap);
  EXPECT_EQ(rtc::mpc::LoadCommandFilterConfig(YAML::Node(), c), CommandFilterError::kNotAMap);
  EXPECT_EQ(c.cutoff_hz, 9.0);  // a refusal leaves `out` alone
}

TEST(CommandFilterYaml, InitRefusesAnInvalidConfigAndKeepsTheFilter) {
  const CommandFilterConfig good = Config(100.0);
  CommandFilter f = Made(good);
  CommandFilterConfig bad = good;
  bad.cutoff_hz = 60.0;
  EXPECT_EQ(f.Init(bad), CommandFilterError::kInvalidCutoff);
  EXPECT_EQ(f.config().cutoff_hz, good.cutoff_hz);
  CommandFilter fresh;
  EXPECT_EQ(fresh.Init(bad), CommandFilterError::kInvalidCutoff);
  EXPECT_FALSE(fresh.initialized());
}

TEST(CommandFilterYaml, TheShippedTemplateLoads) {
  const YAML::Node root =
      YAML::LoadFile(std::string(RTC_MPC_CONFIG_DIR) + "/reference_default.yaml");
  CommandFilterConfig c;
  EXPECT_EQ(rtc::mpc::LoadCommandFilterConfig(root["command"], c), CommandFilterError::kNoError);
  CommandFilter f;
  EXPECT_EQ(f.Init(c), CommandFilterError::kNoError);
}

}  // namespace
