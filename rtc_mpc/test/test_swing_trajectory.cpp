// Swing trajectory · impact proximity · SurfaceFrame (reference/): the W2-F02
// [SPRINT] PR A gates.
//
// The oracle is the cubic Hermite basis on each segment's four boundary values
// (h00·p0 + h10·Δt·v0 + h01·p1 + h11·Δt·v1), not the implementation's
// coefficient formula, and the registered extremes come from the pre-registered
// simulation (the original coefficients evaluated in Python). Two fixtures run
// everything: the core one (terrain 0.137, v_td −0.03 — every term non-zero)
// and the shipped one (terrain 0, v_td 0). Every refusal sits next to an
// accepted twin, and a refused load leaves the config untouched.
#include "rtc_mpc/reference/surface_frame.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using rtc::mpc::BuildSwing;
using rtc::mpc::FlatSurface;
using rtc::mpc::HeightAt;
using rtc::mpc::LoadSwingConfig;
using rtc::mpc::LoadTerrainConfig;
using rtc::mpc::SurfaceFrame;
using rtc::mpc::SurfaceFrameError;
using rtc::mpc::SwingConfig;
using rtc::mpc::SwingError;
using rtc::mpc::SwingInterval;
using rtc::mpc::SwingSample;
using rtc::mpc::SwingTrajectory;

constexpr double kTol = 1e-12;
constexpr double kInf = std::numeric_limits<double>::infinity();
const double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kLo = 1.0;  // every swing starts here

double Before(double t) {
  return std::nextafter(t, -kInf);
}

struct Fixture {
  const char* name;
  SwingConfig cfg;
  double terrain;
};

SwingConfig Config(double v_td) {
  SwingConfig c;
  c.height = 0.08;
  c.lift_off_velocity = 0.05;
  c.touch_down_velocity = v_td;
  c.touch_down_offset = -0.001;
  c.time_scale = 0.4;
  c.impact_proximity.mid = 0.005;
  c.impact_proximity.lift_off_velocity = -0.15;
  c.impact_proximity.touch_down_velocity = 0.3;
  return c;
}

const std::vector<Fixture>& Fixtures() {
  static const std::vector<Fixture> f = {{"core", Config(-0.03), 0.137},
                                         {"shipped", Config(0.0), 0.0}};
  return f;
}

SwingInterval Interval(double lo, double td, bool open_start = false, bool open_end = false) {
  SwingInterval iv;
  iv.lift_off = lo;
  iv.touch_down = td;
  iv.open_start = open_start;
  iv.open_end = open_end;
  return iv;
}

SwingTrajectory Built(const Fixture& f, const SwingInterval& iv) {
  SwingTrajectory s = SwingTrajectory::Stance(f.terrain);
  EXPECT_EQ(BuildSwing(iv, f.cfg, f.terrain, s), SwingError::kNoError);
  return s;
}

// The expected apex of a closed swing, in the implementation's expression order.
double ClosedApex(const Fixture& f, double s) {
  return std::min(f.terrain, f.terrain + f.cfg.touch_down_offset) + s * f.cfg.height;
}

double TouchDownZ(const Fixture& f) {
  return f.terrain + f.cfg.touch_down_offset;
}

// ── Closed swings ─────────────────────────────────────────────────────────────

TEST(SwingTrajectory, ClosedSwingBoundaryValues) {
  for (const Fixture& f : Fixtures()) {
    for (const double T : {0.6, 0.2}) {
      SCOPED_TRACE(std::string(f.name) + " T=" + std::to_string(T));
      const double td = kLo + T;
      const double t_mid = 0.5 * (kLo + td);
      const SwingTrajectory s = Built(f, Interval(kLo, td));
      const double scale = std::min(1.0, T / 0.4);
      EXPECT_NEAR(s.scaling(), scale, kTol);
      EXPECT_EQ(s.scaling(), T >= 0.4 ? 1.0 : s.scaling());  // s == 1 exactly when T ≥ time_scale

      const SwingSample lo = s.Evaluate(kLo);
      EXPECT_EQ(std::memcmp(&lo.z, &f.terrain, sizeof(double)), 0)
          << "z(t_lo) must be c0 = terrain";
      EXPECT_NEAR(lo.z_dot, scale * 0.05, kTol);

      const SwingSample mid = s.Evaluate(t_mid);
      const SwingSample mid_before = s.Evaluate(Before(t_mid));
      EXPECT_NEAR(mid.z, ClosedApex(f, scale), kTol);
      EXPECT_NEAR(mid_before.z, mid.z, kTol);
      EXPECT_NEAR(mid.z_dot, 0.0, kTol);
      EXPECT_NEAR(mid_before.z_dot, 0.0, kTol);

      const SwingSample td_before = s.Evaluate(Before(td));
      EXPECT_NEAR(td_before.z, TouchDownZ(f), kTol);
      EXPECT_NEAR(td_before.z_dot, scale * f.cfg.touch_down_velocity, kTol);
    }
  }
}

// Registered (pre-registration sim, core fixture): the extremes over the swing
// and the acceleration on either side of the apex (the right segment starts at
// −5.1333 / −23.7, the left ends at −4.9333 / −22.9 — a continuous z̈ is not a
// goal, but the values are pinned so a `<` → `≤` branch at t_mid is red).
TEST(SwingTrajectory, ClosedSwingRegisteredExtremes) {
  const Fixture& f = Fixtures()[0];

  struct Registered {
    double T, max_rate, max_acc, acc_mid_before, acc_mid;
  };

  const Registered table[] = {{0.6, 0.392649, 5.133233, -4.9333333333, -5.1333333333},
                              {0.2, 0.596274, 23.699529, -22.9, -23.7}};
  for (const Registered& r : table) {
    SCOPED_TRACE(r.T);
    const double td = kLo + r.T;
    const SwingTrajectory s = Built(f, Interval(kLo, td));
    double max_rate = 0.0, max_acc = 0.0;
    const int n = 200000;
    for (int i = 0; i <= n; ++i) {
      const double t = kLo + (td - 1e-12 - kLo) * i / n;
      const SwingSample p = s.Evaluate(t);
      max_rate = std::max(max_rate, std::fabs(p.z_dot));
      max_acc = std::max(max_acc, std::fabs(p.z_ddot));
    }
    EXPECT_NEAR(max_rate, r.max_rate, 1e-3);
    EXPECT_NEAR(max_acc, r.max_acc, 1e-3);
    const double t_mid = 0.5 * (kLo + td);
    EXPECT_NEAR(s.Evaluate(Before(t_mid)).z_ddot, r.acc_mid_before, 1e-6);
    EXPECT_NEAR(s.Evaluate(t_mid).z_ddot, r.acc_mid, 1e-6);
  }
}

// Hermite basis on the four boundary values of each segment — independent of
// the coefficient formula. z̈ is also checked against a central difference of ż.
TEST(SwingTrajectory, MatchesTheHermiteBasisOracle) {
  struct Knot {
    double t, p, v;
  };

  auto hermite = [](const Knot& a, const Knot& b, double t, double& p, double& v, double& acc) {
    const double dt = b.t - a.t;
    const double tau = (t - a.t) / dt;
    const double tau2 = tau * tau, tau3 = tau2 * tau;
    const double h00 = 2 * tau3 - 3 * tau2 + 1, h10 = tau3 - 2 * tau2 + tau;
    const double h01 = -2 * tau3 + 3 * tau2, h11 = tau3 - tau2;
    p = h00 * a.p + h10 * dt * a.v + h01 * b.p + h11 * dt * b.v;
    const double d00 = 6 * tau2 - 6 * tau, d10 = 3 * tau2 - 4 * tau + 1;
    const double d01 = -6 * tau2 + 6 * tau, d11 = 3 * tau2 - 2 * tau;
    v = (d00 * a.p + d10 * dt * a.v + d01 * b.p + d11 * dt * b.v) / dt;
    const double e00 = 12 * tau - 6, e10 = 6 * tau - 4, e01 = -12 * tau + 6, e11 = 6 * tau - 2;
    acc = (e00 * a.p + e10 * dt * a.v + e01 * b.p + e11 * dt * b.v) / (dt * dt);
  };
  for (const Fixture& f : Fixtures()) {
    for (const double T : {0.6, 0.2}) {
      SCOPED_TRACE(std::string(f.name) + " T=" + std::to_string(T));
      const double td = kLo + T;
      const double t_mid = 0.5 * (kLo + td);
      const double sc = std::min(1.0, T / 0.4);
      const SwingTrajectory s = Built(f, Interval(kLo, td));
      const Knot k_lo{kLo, f.terrain, sc * f.cfg.lift_off_velocity};
      const Knot k_mid{t_mid, ClosedApex(f, sc), 0.0};
      const Knot k_td{td, TouchDownZ(f), sc * f.cfg.touch_down_velocity};
      for (int seg = 0; seg < 2; ++seg) {
        const Knot& a = seg == 0 ? k_lo : k_mid;
        const Knot& b = seg == 0 ? k_mid : k_td;
        for (int i = 0; i < 1000; ++i) {
          // Inside the open segment, 2e-6 away from its ends so the central
          // difference (h = 1e-6) stays in it.
          const double t = a.t + 2e-6 + (b.t - a.t - 4e-6) * i / 999.0;
          double p = 0, v = 0, acc = 0;
          hermite(a, b, t, p, v, acc);
          const SwingSample got = s.Evaluate(t);
          ASSERT_NEAR(got.z, p, kTol) << "t = " << t;
          ASSERT_NEAR(got.z_dot, v, kTol) << "t = " << t;
          ASSERT_NEAR(got.z_ddot, acc, 1e-9) << "t = " << t;
          const double h = 1e-6;
          const double fd = (s.Evaluate(t + h).z_dot - s.Evaluate(t - h).z_dot) / (2 * h);
          ASSERT_NEAR(got.z_ddot, fd, 1e-5) << "t = " << t;
        }
      }
    }
  }
}

// WG-2: z is continuous at lift-off and jumps by exactly the touch-down offset
// at touch-down (stance = terrain); ż jumps by the designed end rates. With
// v_lo = v_td = offset = 0 both ends are C¹.
TEST(SwingTrajectory, ContinuityAtTheSwingEnds) {
  for (const Fixture& f : Fixtures()) {
    for (const double T : {0.6, 0.2}) {
      SCOPED_TRACE(std::string(f.name) + " T=" + std::to_string(T));
      const double td = kLo + T;
      const double sc = std::min(1.0, T / 0.4);
      const SwingTrajectory s = Built(f, Interval(kLo, td));
      const SwingSample stance_before = s.Evaluate(Before(kLo));
      const SwingSample lo = s.Evaluate(kLo);
      EXPECT_EQ(stance_before.z, f.terrain);
      EXPECT_EQ(stance_before.z_dot, 0.0);
      EXPECT_NEAR(lo.z - stance_before.z, 0.0, kTol);
      EXPECT_NEAR(lo.z_dot - stance_before.z_dot, sc * f.cfg.lift_off_velocity, kTol);
      const SwingSample td_before = s.Evaluate(Before(td));
      const SwingSample td_at = s.Evaluate(td);
      EXPECT_EQ(td_at.z, f.terrain);
      EXPECT_EQ(td_at.z_dot, 0.0);
      EXPECT_NEAR(td_at.z - td_before.z, -f.cfg.touch_down_offset, kTol);  // +0.001
      EXPECT_NEAR(td_at.z_dot - td_before.z_dot, -sc * f.cfg.touch_down_velocity, kTol);
      EXPECT_EQ(td_at.impact, 1.0);
    }
  }
  Fixture c1 = Fixtures()[0];
  c1.cfg.lift_off_velocity = 0.0;
  c1.cfg.touch_down_velocity = 0.0;
  c1.cfg.touch_down_offset = 0.0;
  for (const double T : {0.6, 0.2}) {
    const double td = kLo + T;
    const SwingTrajectory s = Built(c1, Interval(kLo, td));
    for (const double t : {kLo, td}) {
      const SwingSample a = s.Evaluate(Before(t));
      const SwingSample b = s.Evaluate(t);
      EXPECT_NEAR(a.z, b.z, kTol) << "t = " << t;
      EXPECT_NEAR(a.z_dot, b.z_dot, kTol) << "t = " << t;
    }
  }
}

// ── Open cases · stance ───────────────────────────────────────────────────────

TEST(SwingTrajectory, OpenEndKeepsTheUnscaledLiftOffAndParksAtTheApex) {
  const Fixture& f = Fixtures()[0];
  const double t_end = kLo + 0.2;
  const SwingTrajectory s = Built(f, Interval(kLo, t_end, false, true));
  EXPECT_EQ(s.Evaluate(kLo).z, f.terrain);
  EXPECT_NEAR(s.Evaluate(kLo).z_dot, 0.05, kTol);  // not scaled by 0.5
  EXPECT_NEAR(s.Evaluate(Before(t_end)).z, f.terrain + 0.08, kTol);
  EXPECT_NEAR(s.Evaluate(Before(t_end)).z_dot, 0.0, kTol);
  const double t_mid = 0.5 * (kLo + t_end);
  EXPECT_NEAR(s.Evaluate(t_mid).z, f.terrain + 0.08, kTol);
  EXPECT_NEAR(s.Evaluate(t_mid).z_dot, 0.0, kTol);
  double z_max = -kInf, t_max = 0.0;
  for (int i = 0; i <= 2000; ++i) {
    const double t = kLo + 0.2 * i / 2000.0;
    const double z = s.Evaluate(t).z;
    if (z > z_max) {
      z_max = z;
      t_max = t;
    }
  }
  EXPECT_NEAR(t_max, t_mid, 1e-3);
  // An open end holds the curve past the window end (no stance drop).
  EXPECT_NEAR(s.Evaluate(t_end).z, f.terrain + 0.08, kTol);
  EXPECT_NEAR(s.Evaluate(t_end + 1.0).z, f.terrain + 0.08, kTol);
  EXPECT_NEAR(s.Evaluate(kLo).impact, 1.0, kTol);
  EXPECT_NEAR(s.Evaluate(kLo).impact_dot, -0.15, kTol);
  EXPECT_NEAR(s.Evaluate(Before(t_end)).impact, 0.005, kTol);
  EXPECT_NEAR(s.Evaluate(Before(t_end)).impact_dot, 0.0, kTol);
}

TEST(SwingTrajectory, OpenStartDescendsFromTheApexUnscaled) {
  const Fixture& f = Fixtures()[0];
  const double td = kLo + 0.2;
  const SwingTrajectory s = Built(f, Interval(kLo, td, true, false));
  const double apex = (f.terrain + f.cfg.touch_down_offset) + 0.08;  // 0.216
  EXPECT_NEAR(s.Evaluate(kLo).z, apex, kTol);
  EXPECT_NEAR(s.Evaluate(kLo).z_dot, 0.0, kTol);
  EXPECT_NEAR(s.Evaluate(Before(td)).z, TouchDownZ(f), kTol);
  EXPECT_NEAR(s.Evaluate(Before(td)).z_dot, -0.03, kTol);  // not scaled by 0.5
  EXPECT_EQ(s.Evaluate(td).z, f.terrain);                  // touch-down is stance
  EXPECT_NEAR(s.Evaluate(kLo).impact, 0.005, kTol);
  EXPECT_NEAR(s.Evaluate(kLo).impact_dot, 0.0, kTol);
  EXPECT_NEAR(s.Evaluate(Before(td)).impact, 1.0, kTol);
  EXPECT_NEAR(s.Evaluate(Before(td)).impact_dot, 0.3, kTol);
}

TEST(SwingTrajectory, BothOpenIsTheConstantApexAndStanceIsTheTerrain) {
  const Fixture& f = Fixtures()[0];
  const double t_end = kLo + 0.2;
  const SwingTrajectory s = Built(f, Interval(kLo, t_end, true, true));
  const double apex = (f.terrain + f.cfg.touch_down_offset) + 0.08;
  for (int i = 0; i <= 100; ++i) {
    const double t = kLo + 0.2 * i / 100.0;
    const SwingSample p = s.Evaluate(t);
    ASSERT_NEAR(p.z, apex, 1e-15) << t;
    ASSERT_NEAR(p.z_dot, 0.0, 1e-15) << t;
    ASSERT_NEAR(p.z_ddot, 0.0, 1e-15) << t;
    ASSERT_NEAR(p.impact, 0.005, 1e-15) << t;
    ASSERT_NEAR(p.impact_dot, 0.0, 1e-15) << t;
  }
  const SwingTrajectory stance = SwingTrajectory::Stance(f.terrain);
  EXPECT_FALSE(stance.swinging());
  for (const double t : {-1.0, 0.0, kLo, 1e6, kNaN, kInf, -kInf}) {
    const SwingSample p = stance.Evaluate(t);
    EXPECT_EQ(std::memcmp(&p.z, &f.terrain, sizeof(double)), 0);
    EXPECT_EQ(p.z_dot, 0.0);
    EXPECT_EQ(p.z_ddot, 0.0);
    EXPECT_EQ(p.impact, 1.0);
    EXPECT_EQ(p.impact_dot, 0.0);
  }
}

// End rates steep enough to overshoot the cubic are not refused by the
// loader; the sample is clamped to the weight's range instead.
TEST(SwingTrajectory, ImpactProximityIsClampedToTheUnitInterval) {
  Fixture f = Fixtures()[0];
  f.cfg.impact_proximity.lift_off_velocity = -30.0;
  f.cfg.impact_proximity.touch_down_velocity = 30.0;
  const double td = kLo + 0.4;
  const SwingTrajectory s = Built(f, Interval(kLo, td));
  double lo = 1.0, hi = 0.0;
  for (int i = 0; i <= 4000; ++i) {
    const double t = kLo + 0.4 * i / 4000.0;
    const double w = s.Evaluate(t).impact;
    lo = std::min(lo, w);
    hi = std::max(hi, w);
  }
  EXPECT_EQ(lo, 0.0);  // the clamp bites: the unclamped cubic goes below 0 (next)
  EXPECT_EQ(hi, 1.0);
  // The left impact segment by the Hermite basis: (1, −30) → (0.005, 0) over
  // 0.2 s dives below 0, and the sample's rate is 0 while it is clamped.
  double raw_min = 1.0;
  bool rate_zero_while_clamped = true;
  for (int i = 0; i <= 2000; ++i) {
    const double tau = i / 2000.0;
    const double tau2 = tau * tau, tau3 = tau2 * tau;
    const double h00 = 2 * tau3 - 3 * tau2 + 1, h10 = tau3 - 2 * tau2 + tau;
    const double h01 = -2 * tau3 + 3 * tau2;
    const double raw = h00 * 1.0 + h10 * 0.2 * -30.0 + h01 * 0.005;
    raw_min = std::min(raw_min, raw);
    const SwingSample p = s.Evaluate(kLo + 0.2 * tau);
    if (raw < 0.0 && p.impact_dot != 0.0) {
      rate_zero_while_clamped = false;
    }
  }
  EXPECT_LT(raw_min, 0.0);
  EXPECT_TRUE(rate_zero_while_clamped);
}

TEST(SwingTrajectory, OpenFlagsAreReported) {
  const Fixture& f = Fixtures()[0];
  EXPECT_FALSE(Built(f, Interval(kLo, kLo + 0.2)).open_start());
  EXPECT_TRUE(Built(f, Interval(kLo, kLo + 0.2, true, false)).open_start());
  EXPECT_FALSE(Built(f, Interval(kLo, kLo + 0.2, true, false)).open_end());
  EXPECT_TRUE(Built(f, Interval(kLo, kLo + 0.2, false, true)).open_end());
}

TEST(SwingTrajectory, ClosedImpactProximityScalesTheEndRatesNotTheMid) {
  for (const Fixture& f : Fixtures()) {
    SCOPED_TRACE(f.name);
    const double td = kLo + 0.2;
    const SwingTrajectory s = Built(f, Interval(kLo, td));
    EXPECT_NEAR(s.Evaluate(kLo).impact, 1.0, kTol);
    EXPECT_NEAR(s.Evaluate(kLo).impact_dot, 0.5 * -0.15, kTol);
    EXPECT_NEAR(s.Evaluate(0.5 * (kLo + td)).impact, 0.005, kTol);
    EXPECT_NEAR(s.Evaluate(0.5 * (kLo + td)).impact_dot, 0.0, kTol);
    EXPECT_NEAR(s.Evaluate(Before(td)).impact, 1.0, kTol);
    EXPECT_NEAR(s.Evaluate(Before(td)).impact_dot, 0.5 * 0.3, kTol);
    const SwingTrajectory s6 = Built(f, Interval(kLo, kLo + 0.6));
    EXPECT_NEAR(s6.Evaluate(kLo).impact_dot, -0.15, kTol);
    EXPECT_NEAR(s6.Evaluate(Before(kLo + 0.6)).impact_dot, 0.3, kTol);
  }
}

// ── NUM-2 ─────────────────────────────────────────────────────────────────────

TEST(SwingTrajectory, DegenerateAndNonFiniteIntervalsAreRefusedAndEvaluateStaysFinite) {
  const Fixture& f = Fixtures()[0];
  // Lift-off at 0 so the duration is the literal (1 + 1e-9) − 1 is not 1e-9).
  for (const double T : {0.0, 1e-9}) {
    SwingTrajectory s = Built(f, Interval(kLo, kLo + 0.3));
    EXPECT_EQ(BuildSwing(Interval(0.0, T), f.cfg, f.terrain, s), SwingError::kDegenerateInterval);
    EXPECT_TRUE(s.swinging());  // untouched: still the 0.3 s swing
    EXPECT_NEAR(s.touch_down(), kLo + 0.3, 0.0);
    const SwingSample p = s.Evaluate(kLo + 0.15);
    EXPECT_TRUE(std::isfinite(p.z) && std::isfinite(p.z_dot) && std::isfinite(p.z_ddot));
  }
  SwingTrajectory ok = SwingTrajectory::Stance(f.terrain);
  EXPECT_EQ(BuildSwing(Interval(0.0, 2e-9), f.cfg, f.terrain, ok), SwingError::kNoError);
  EXPECT_TRUE(std::isfinite(ok.Evaluate(1e-9).z_ddot));
  for (const double bad : {kNaN, kInf, -kInf}) {
    SwingTrajectory s = SwingTrajectory::Stance(f.terrain);
    EXPECT_EQ(BuildSwing(Interval(kLo, bad), f.cfg, f.terrain, s), SwingError::kNonFinite);
    EXPECT_EQ(BuildSwing(Interval(bad, kLo), f.cfg, f.terrain, s), SwingError::kNonFinite);
    EXPECT_EQ(BuildSwing(Interval(kLo, kLo + 0.3), f.cfg, bad, s), SwingError::kNonFinite);
    EXPECT_FALSE(s.swinging());
  }
  const SwingTrajectory s = Built(f, Interval(kLo, kLo + 0.3));
  for (const double t : {kNaN, kInf, -kInf}) {
    const SwingSample p = s.Evaluate(t);
    EXPECT_TRUE(std::isfinite(p.z) && std::isfinite(p.z_dot) && std::isfinite(p.z_ddot) &&
                std::isfinite(p.impact) && std::isfinite(p.impact_dot))
        << t;
  }
  EXPECT_EQ(s.Evaluate(kNaN).z, f.terrain);  // stance
  // Also on an open end, where the curve otherwise holds past t_end.
  const SwingTrajectory open = Built(f, Interval(kLo, kLo + 0.2, false, true));
  for (const double t : {kNaN, kInf, -kInf}) {
    EXPECT_EQ(open.Evaluate(t).z, f.terrain) << t;
    EXPECT_EQ(open.Evaluate(t).impact, 1.0) << t;
  }
  // A stretch of one ulp at a large t passes the absolute bound but its
  // midpoint rounds onto an end: refused, not a zero-length segment.
  {
    const double big = 1e8;
    const double next = std::nextafter(big, kInf);
    ASSERT_GT(next - big, rtc::mpc::kMinSwingDuration);
    SwingTrajectory s1 = SwingTrajectory::Stance(f.terrain);
    EXPECT_EQ(BuildSwing(Interval(big, next), f.cfg, f.terrain, s1),
              SwingError::kDegenerateInterval);
    EXPECT_FALSE(s1.swinging());
    const double four = std::nextafter(std::nextafter(std::nextafter(next, kInf), kInf), kInf);
    EXPECT_EQ(BuildSwing(Interval(big, four), f.cfg, f.terrain, s1), SwingError::kNoError);
    EXPECT_TRUE(std::isfinite(s1.Evaluate(0.5 * (big + four)).z_ddot));
  }
  // A finite config whose coefficients overflow is refused, not NaN.
  {
    Fixture huge = f;
    huge.cfg.height = 1e308;
    SwingTrajectory s2 = SwingTrajectory::Stance(f.terrain);
    EXPECT_EQ(BuildSwing(Interval(kLo, kLo + 0.3), huge.cfg, f.terrain, s2),
              SwingError::kCoefficientOverflow);
    EXPECT_FALSE(s2.swinging());
  }
  static_assert(noexcept(s.Evaluate(0.0)));
  static_assert(
      noexcept(BuildSwing(SwingInterval{}, SwingConfig{}, 0.0, std::declval<SwingTrajectory&>())));
}

// ── SurfaceFrame ──────────────────────────────────────────────────────────────

TEST(SurfaceFrame, PlaneHeightMatchesThePlaneEquationAndFlatIsConstant) {
  SurfaceFrame plane;
  plane.point = {0.0, 0.0, 0.137};
  const double n = std::sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.95);
  plane.normal = {0.1 / n, 0.2 / n, std::sqrt(0.95) / n};
  ASSERT_EQ(rtc::mpc::ValidateSurfaceFrame(plane), SurfaceFrameError::kNoError);
  for (const double x : {-1.0, 0.0, 0.37}) {
    for (const double y : {-0.5, 0.0, 2.0}) {
      // n·(r − p) = 0 ⇒ z = p_z − (n_x x + n_y y) / n_z
      const double z = 0.137 - (plane.normal[0] * x + plane.normal[1] * y) / plane.normal[2];
      EXPECT_NEAR(HeightAt(plane, x, y), z, kTol);
      const double residual = plane.normal[0] * x + plane.normal[1] * y +
                              plane.normal[2] * (HeightAt(plane, x, y) - 0.137);
      EXPECT_NEAR(residual, 0.0, kTol);
    }
  }
  const SurfaceFrame flat = FlatSurface(0.137);
  ASSERT_EQ(rtc::mpc::ValidateSurfaceFrame(flat), SurfaceFrameError::kNoError);
  for (const double x : {-10.0, 0.0, 3.5}) {
    EXPECT_EQ(HeightAt(flat, x, -x), 0.137);
  }
}

TEST(SurfaceFrame, ValidationRefusalsHaveAcceptedTwins) {
  auto with_normal = [](double nx, double nz) {
    SurfaceFrame s;
    const double n = std::sqrt(nx * nx + nz * nz);
    s.normal = {nx / n, 0.0, nz / n};
    return s;
  };
  EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(with_normal(1.0, 1e-9)),
            SurfaceFrameError::kDegenerateNormal);
  EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(with_normal(1.0, 0.0)),
            SurfaceFrameError::kDegenerateNormal);
  EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(with_normal(0.0, -1.0)),
            SurfaceFrameError::kDegenerateNormal);
  EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(with_normal(1.0, 2e-9)), SurfaceFrameError::kNoError);
  SurfaceFrame not_unit = FlatSurface(0.0);
  not_unit.normal = {0.0, 0.0, 1.0 + 1e-4};
  EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(not_unit), SurfaceFrameError::kNotUnitNormal);
  for (const double bad : {kNaN, kInf}) {
    SurfaceFrame s = FlatSurface(0.0);
    s.point[1] = bad;
    EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(s), SurfaceFrameError::kNonFinite);
    SurfaceFrame n = FlatSurface(0.0);
    n.normal[0] = bad;
    EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(n), SurfaceFrameError::kNonFinite);
  }
}

// ── YAML ──────────────────────────────────────────────────────────────────────

constexpr const char* kSwingYaml = R"(
height: 0.08
lift_off_velocity: 0.05
touch_down_velocity: 0.0
touch_down_offset: -0.001
time_scale: 0.4
impact_proximity:
  mid: 0.005
  lift_off_velocity: -0.15
  touch_down_velocity: 0.3
)";

// kSwingYaml with one `key: value` line replaced (the key's first occurrence
// at the given indent).
std::string With(const std::string& line, const std::string& replacement) {
  std::string y = kSwingYaml;
  const std::size_t pos = y.find(line);
  EXPECT_NE(pos, std::string::npos) << line;
  return y.replace(pos, line.size(), replacement);
}

SwingConfig Sentinel() {
  SwingConfig c;
  c.height = -123.0;
  return c;
}

bool Untouched(const SwingConfig& c) {
  return c.height == -123.0;
}

TEST(SwingConfigYaml, TheTemplateLoadsWithTheExpectedValues) {
  SwingConfig c = Sentinel();
  ASSERT_EQ(LoadSwingConfig(YAML::Load(kSwingYaml), c), SwingError::kNoError);
  EXPECT_EQ(c.height, 0.08);
  EXPECT_EQ(c.lift_off_velocity, 0.05);
  EXPECT_EQ(c.touch_down_velocity, 0.0);
  EXPECT_EQ(c.touch_down_offset, -0.001);
  EXPECT_EQ(c.time_scale, 0.4);
  EXPECT_EQ(c.impact_proximity.mid, 0.005);
  EXPECT_EQ(c.impact_proximity.lift_off_velocity, -0.15);
  EXPECT_EQ(c.impact_proximity.touch_down_velocity, 0.3);
  SurfaceFrame terrain;
  terrain.point[2] = -1.0;
  ASSERT_EQ(LoadTerrainConfig(YAML::Load("height: 0.137"), terrain), SurfaceFrameError::kNoError);
  EXPECT_EQ(terrain.point[2], 0.137);
  EXPECT_EQ(terrain.normal[2], 1.0);
}

TEST(SwingConfigYaml, RefusalsHaveAcceptedTwinsAndLeaveTheConfigUntouched) {
  struct Case {
    std::string yaml;
    SwingError expected;
  };

  std::vector<Case> cases = {
      {std::string(kSwingYaml) + "hieght: 0.1\n", SwingError::kUnknownKey},
      {std::string(kSwingYaml) + "  middle: 0.1\n", SwingError::kUnknownKey},  // nested
      {With("height: 0.08", "height: 0.0"), SwingError::kInvalidHeight},
      {With("height: 0.08", "height: -0.01"), SwingError::kInvalidHeight},
      {With("time_scale: 0.4", "time_scale: 0.0"), SwingError::kInvalidTimeScale},
      {With("time_scale: 0.4", "time_scale: -1.0"), SwingError::kInvalidTimeScale},
      {With("  mid: 0.005", "  mid: -0.001"), SwingError::kInvalidImpactMid},
      {With("  mid: 0.005", "  mid: 1.001"), SwingError::kInvalidImpactMid},
      {With("time_scale: 0.4", ""), SwingError::kMissingKey},
      {With("  mid: 0.005", ""), SwingError::kMissingKey},
      {With("height: 0.08", "height: [0.08]"), SwingError::kWrongType},
      {With("impact_proximity:", "impact_proximity: 0.3\nignored:"), SwingError::kUnknownKey},
      {"- 1\n- 2\n", SwingError::kNotAMap},
  };
  const char* lines[] = {"height: 0.08",
                         "lift_off_velocity: 0.05",
                         "touch_down_velocity: 0.0",
                         "touch_down_offset: -0.001",
                         "time_scale: 0.4",
                         "  mid: 0.005",
                         "  lift_off_velocity: -0.15",
                         "  touch_down_velocity: 0.3"};
  for (const char* line : lines) {
    const std::string key(line, std::strlen(line) - std::strlen(std::strrchr(line, ':')));
    cases.push_back({With(line, key + ": .nan"), SwingError::kNonFinite});
    cases.push_back({With(line, key + ": .inf"), SwingError::kNonFinite});
  }
  for (const Case& c : cases) {
    SCOPED_TRACE(c.yaml);
    SwingConfig cfg = Sentinel();
    EXPECT_EQ(LoadSwingConfig(YAML::Load(c.yaml), cfg), c.expected);
    EXPECT_TRUE(Untouched(cfg));
  }
  // Accepted twins at the boundaries.
  for (const std::string& ok :
       {With("  mid: 0.005", "  mid: 0.0"), With("  mid: 0.005", "  mid: 1.0"),
        With("touch_down_offset: -0.001", "touch_down_offset: 0.0"),
        With("touch_down_velocity: 0.0", "touch_down_velocity: 0.0"),
        With("height: 0.08", "height: 1.0e-3"), With("time_scale: 0.4", "time_scale: 1.0e-3")}) {
    SCOPED_TRACE(ok);
    SwingConfig cfg = Sentinel();
    EXPECT_EQ(LoadSwingConfig(YAML::Load(ok), cfg), SwingError::kNoError);
    EXPECT_FALSE(Untouched(cfg));
  }
  SwingConfig missing_map = Sentinel();
  EXPECT_EQ(LoadSwingConfig(YAML::Node(), missing_map), SwingError::kNotAMap);
  EXPECT_TRUE(Untouched(missing_map));
}

// The shipped template (config/reference_default.yaml), not an inline copy.
TEST(SwingConfigYaml, TheConfigTemplateFileLoads) {
  const YAML::Node root =
      YAML::LoadFile(std::string(RTC_MPC_CONFIG_DIR) + "/reference_default.yaml");
  SwingConfig swing = Sentinel();
  EXPECT_EQ(LoadSwingConfig(root["swing"], swing), SwingError::kNoError);
  EXPECT_EQ(swing.height, 0.08);
  SurfaceFrame terrain = FlatSurface(-1.0);
  EXPECT_EQ(LoadTerrainConfig(root["terrain"], terrain), SurfaceFrameError::kNoError);
  EXPECT_EQ(rtc::mpc::ValidateSurfaceFrame(terrain), SurfaceFrameError::kNoError);
}

TEST(SwingConfigYaml, TerrainRefusalsHaveAcceptedTwins) {
  SurfaceFrame t = FlatSurface(-1.0);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("height: .nan"), t), SurfaceFrameError::kNonFinite);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("height: .inf"), t), SurfaceFrameError::kNonFinite);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("height: -.inf"), t), SurfaceFrameError::kNonFinite);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("height: 0.0\nslope: 0.1"), t),
            SurfaceFrameError::kUnknownKey);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("slope: 0.1"), t), SurfaceFrameError::kUnknownKey);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("{}"), t), SurfaceFrameError::kMissingKey);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("height: [0.0]"), t), SurfaceFrameError::kWrongType);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("0.0"), t), SurfaceFrameError::kNotAMap);
  EXPECT_EQ(t.point[2], -1.0);
  EXPECT_EQ(LoadTerrainConfig(YAML::Load("height: -0.2"), t), SurfaceFrameError::kNoError);
  EXPECT_EQ(t.point[2], -0.2);
}

}  // namespace
