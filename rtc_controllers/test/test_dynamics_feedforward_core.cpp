// ── Dynamics feedforward law + schema ───────────────────────────────────────
// Pins the contract of joint/dynamics_feedforward_law.hpp and its parser:
//   - the law: gain scaling, the per-channel magnitude clamp and its bit mask,
//     the all-or-nothing non-finite rule, the tail policy (zero past the model
//     terms / past the bound span), the Coriolis and inertia slots being
//     unread at gain 0 and sharing the one clamp when read, and
//     allocation-freedom on the tick (RT-1);
//   - the schema: `max_abs` is required and bounded, `gain` and `inertia_gain`
//     are bounded, the block itself is required, and absent optional keys keep
//     the seed.
//
// Each clamp / bound case has a partner just inside the bound, so a deleted
// check fails here rather than passing on both sides.
#include "rtc_controllers/joint/dynamics_feedforward_law.hpp"
#include "rtc_controllers/params/dynamics_feedforward_params.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using rtc::joint::ComputeDynamicsFeedforward;
using rtc::joint::DynamicsFeedforwardGainsView;
using rtc::joint::DynamicsFeedforwardInputs;
using rtc::params::DynamicsFeedforwardParams;
using rtc::params::kDynamicsFeedforwardGainMax;
using rtc::params::ParseDynamicsFeedforwardParams;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// ═══════════════════════════════════════════════════════════════════════════
// The law
// ═══════════════════════════════════════════════════════════════════════════

TEST(DynamicsFeedforwardLaw, ScalesGravityByTheGainAndKeepsTheSign) {
  const std::array<double, 3> g = {0.08, -0.04, 0.0};
  const std::array<double, 3> lim = {1.0, 1.0, 1.0};
  std::array<double, 3> tau{};
  const auto r =
      ComputeDynamicsFeedforward({.gravity_gain = 0.5, .max_abs = lim}, {.gravity = g}, 3, tau);
  EXPECT_TRUE(r.finite);
  EXPECT_EQ(r.clamped, 0u);
  EXPECT_EQ(r.written, 3u);
  EXPECT_DOUBLE_EQ(tau[0], 0.04);
  EXPECT_DOUBLE_EQ(tau[1], -0.02);
  EXPECT_DOUBLE_EQ(tau[2], 0.0);
}

TEST(DynamicsFeedforwardLaw, ClampsByMagnitudePerChannelAndFlagsTheBit) {
  // Channel 0 just inside its bound, channel 1 just over (negative), channel 2
  // far over: only 1 and 2 are clamped, and each keeps its sign.
  const std::array<double, 3> g = {0.0999, -0.1001, 5.0};
  const std::array<double, 3> lim = {0.1, 0.1, 0.1};
  std::array<double, 3> tau{};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 3, tau);
  EXPECT_TRUE(r.finite);
  EXPECT_EQ(r.clamped, 0b110u);
  EXPECT_DOUBLE_EQ(tau[0], 0.0999);
  EXPECT_DOUBLE_EQ(tau[1], -0.1);
  EXPECT_DOUBLE_EQ(tau[2], 0.1);
}

TEST(DynamicsFeedforwardLaw, PerChannelBoundsAreIndependent) {
  const std::array<double, 2> g = {0.5, 0.5};
  const std::array<double, 2> lim = {0.2, 1.0};
  std::array<double, 2> tau{};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 2, tau);
  EXPECT_EQ(r.clamped, 0b01u);
  EXPECT_DOUBLE_EQ(tau[0], 0.2);
  EXPECT_DOUBLE_EQ(tau[1], 0.5);
}

TEST(DynamicsFeedforwardLaw, ANegativeBoundReadsAsZero) {
  const std::array<double, 1> g = {0.3};
  const std::array<double, 1> lim = {-0.1};
  std::array<double, 1> tau = {7.0};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 1, tau);
  EXPECT_EQ(r.clamped, 0b1u);
  EXPECT_DOUBLE_EQ(tau[0], 0.0);
}

TEST(DynamicsFeedforwardLaw, NonFiniteGravityZeroesEveryChannelNotJustTheBadOne) {
  // NaN at channel 2: channels 0 and 1 would have been finite and in range. The
  // all-or-nothing rule retracts them too, and the clamp mask stays empty.
  const std::array<double, 4> g = {0.05, 0.05, kNaN, 0.05};
  const std::array<double, 4> lim = {1.0, 1.0, 1.0, 1.0};
  std::array<double, 4> tau = {9.0, 9.0, 9.0, 9.0};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 4, tau);
  EXPECT_FALSE(r.finite);
  EXPECT_EQ(r.clamped, 0u);
  for (double t : tau) {
    EXPECT_DOUBLE_EQ(t, 0.0);
  }
}

TEST(DynamicsFeedforwardLaw, NonFiniteGainOrBoundIsAlsoAllOrNothing) {
  const std::array<double, 2> g = {0.05, 0.05};
  std::array<double, 2> tau{};
  {
    const std::array<double, 2> lim = {1.0, 1.0};
    const auto r =
        ComputeDynamicsFeedforward({.gravity_gain = kInf, .max_abs = lim}, {.gravity = g}, 2, tau);
    EXPECT_FALSE(r.finite);
    EXPECT_DOUBLE_EQ(tau[0], 0.0);
    EXPECT_DOUBLE_EQ(tau[1], 0.0);
  }
  {
    const std::array<double, 2> lim = {1.0, kNaN};
    const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 2, tau);
    EXPECT_FALSE(r.finite);
    EXPECT_DOUBLE_EQ(tau[0], 0.0);
  }
}

TEST(DynamicsFeedforwardLaw, TailPastTheModelTermsIsZero) {
  // A 4-channel device whose model covers 2 joints: channels 2 and 3 get 0
  // even though the output buffer held garbage.
  const std::array<double, 2> g = {0.05, -0.05};
  const std::array<double, 4> lim = {1.0, 1.0, 1.0, 1.0};
  std::array<double, 4> tau = {9.0, 9.0, 9.0, 9.0};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 4, tau);
  EXPECT_TRUE(r.finite);
  EXPECT_EQ(r.written, 2u);
  EXPECT_DOUBLE_EQ(tau[0], 0.05);
  EXPECT_DOUBLE_EQ(tau[1], -0.05);
  EXPECT_DOUBLE_EQ(tau[2], 0.0);
  EXPECT_DOUBLE_EQ(tau[3], 0.0);
}

TEST(DynamicsFeedforwardLaw, TailPastTheBoundSpanIsZero) {
  const std::array<double, 3> g = {0.05, 0.05, 0.05};
  const std::array<double, 1> lim = {1.0};
  std::array<double, 3> tau = {9.0, 9.0, 9.0};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 3, tau);
  EXPECT_EQ(r.written, 1u);
  EXPECT_DOUBLE_EQ(tau[0], 0.05);
  EXPECT_DOUBLE_EQ(tau[1], 0.0);
  EXPECT_DOUBLE_EQ(tau[2], 0.0);
}

TEST(DynamicsFeedforwardLaw, OutputSpanBoundsNc) {
  const std::array<double, 4> g = {0.1, 0.1, 0.1, 0.1};
  const std::array<double, 4> lim = {1.0, 1.0, 1.0, 1.0};
  std::array<double, 2> tau{};
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, 4, tau);
  EXPECT_EQ(r.written, 2u);
  EXPECT_DOUBLE_EQ(tau[0], 0.1);
  EXPECT_DOUBLE_EQ(tau[1], 0.1);
}

TEST(DynamicsFeedforwardLaw, CoriolisSlotIsUnreadAtGainZeroAndReadOtherwise) {
  const std::array<double, 2> g = {0.1, 0.1};
  const std::array<double, 2> c = {kNaN, 0.2};  // NaN in slot 0: only matters when read
  const std::array<double, 2> lim = {1.0, 1.0};
  std::array<double, 2> tau{};
  {
    // Gain 0, EMPTY span: the slot must neither shrink n nor be read.
    const auto r = ComputeDynamicsFeedforward({.coriolis_gain = 0.0, .max_abs = lim},
                                              {.gravity = g, .coriolis = {}}, 2, tau);
    EXPECT_TRUE(r.finite);
    EXPECT_EQ(r.written, 2u);
    EXPECT_DOUBLE_EQ(tau[1], 0.1);
  }
  {
    // Gain 0, a span WITH a NaN: still unread.
    const auto r = ComputeDynamicsFeedforward({.coriolis_gain = 0.0, .max_abs = lim},
                                              {.gravity = g, .coriolis = c}, 2, tau);
    EXPECT_TRUE(r.finite);
  }
  {
    // Gain set: the slot is read, so the NaN retracts everything.
    const auto r = ComputeDynamicsFeedforward({.coriolis_gain = 1.0, .max_abs = lim},
                                              {.gravity = g, .coriolis = c}, 2, tau);
    EXPECT_FALSE(r.finite);
  }
  {
    // Gain set, finite span: the term is added with its gain.
    const std::array<double, 2> c_ok = {0.3, 0.2};
    const auto r = ComputeDynamicsFeedforward({.coriolis_gain = 0.5, .max_abs = lim},
                                              {.gravity = g, .coriolis = c_ok}, 2, tau);
    EXPECT_TRUE(r.finite);
    EXPECT_DOUBLE_EQ(tau[0], 0.1 + 0.15);
    EXPECT_DOUBLE_EQ(tau[1], 0.1 + 0.1);
  }
}

TEST(DynamicsFeedforwardLaw, InertiaSlotIsUnreadAtGainZeroAndReadOtherwise) {
  const std::array<double, 2> g = {0.1, 0.1};
  const std::array<double, 2> m = {kNaN, 0.2};  // NaN in slot 0: only matters when read
  const std::array<double, 2> lim = {1.0, 1.0};
  std::array<double, 2> tau{};
  {
    // Gain 0, EMPTY span: the slot neither shrinks n nor is read.
    const auto r = ComputeDynamicsFeedforward({.inertia_gain = 0.0, .max_abs = lim},
                                              {.gravity = g, .inertia = {}}, 2, tau);
    EXPECT_TRUE(r.finite);
    EXPECT_EQ(r.written, 2u);
    EXPECT_DOUBLE_EQ(tau[0], 0.1);
    EXPECT_DOUBLE_EQ(tau[1], 0.1);
  }
  {
    // Gain 0, a span WITH a NaN: still unread.
    const auto r = ComputeDynamicsFeedforward({.inertia_gain = 0.0, .max_abs = lim},
                                              {.gravity = g, .inertia = m}, 2, tau);
    EXPECT_TRUE(r.finite);
  }
  {
    // Gain set: the slot is read, so the NaN retracts every channel.
    const auto r = ComputeDynamicsFeedforward({.inertia_gain = 1.0, .max_abs = lim},
                                              {.gravity = g, .inertia = m}, 2, tau);
    EXPECT_FALSE(r.finite);
    EXPECT_DOUBLE_EQ(tau[0], 0.0);
    EXPECT_DOUBLE_EQ(tau[1], 0.0);
  }
  {
    // A non-finite inertia GAIN is all-or-nothing too.
    const std::array<double, 2> m_ok = {0.3, 0.2};
    const auto r = ComputeDynamicsFeedforward({.inertia_gain = kInf, .max_abs = lim},
                                              {.gravity = g, .inertia = m_ok}, 2, tau);
    EXPECT_FALSE(r.finite);
  }
  {
    // Gain set, finite span: the term is added with its gain, and a short span
    // bounds n like any read span (channel 1 is the tail).
    const std::array<double, 1> m_short = {0.3};
    const auto r = ComputeDynamicsFeedforward({.inertia_gain = 0.5, .max_abs = lim},
                                              {.gravity = g, .inertia = m_short}, 2, tau);
    EXPECT_TRUE(r.finite);
    EXPECT_EQ(r.written, 1u);
    EXPECT_DOUBLE_EQ(tau[0], 0.1 + 0.15);
    EXPECT_DOUBLE_EQ(tau[1], 0.0);
  }
}

TEST(DynamicsFeedforwardLaw, EveryTermSharesTheOneClampOnTheSum) {
  // Each term alone is inside the bound; their sum is not (channel 0), and
  // terms of opposite sign cancel before the clamp (channel 1).
  const std::array<double, 2> g = {0.08, 0.08};
  const std::array<double, 2> c = {0.04, 0.0};
  const std::array<double, 2> m = {0.04, -0.12};
  const std::array<double, 2> lim = {0.15, 0.15};
  std::array<double, 2> tau{};
  const auto r = ComputeDynamicsFeedforward(
      {.gravity_gain = 1.0, .coriolis_gain = 1.0, .inertia_gain = 1.0, .max_abs = lim},
      {.gravity = g, .coriolis = c, .inertia = m}, 2, tau);
  EXPECT_TRUE(r.finite);
  EXPECT_EQ(r.clamped, 0b01u);
  EXPECT_DOUBLE_EQ(tau[0], 0.15);
  EXPECT_DOUBLE_EQ(tau[1], 0.08 - 0.12);
}

TEST(DynamicsFeedforwardLaw, ClampMaskBitsFollowChannelIndexUpToSixtyFour) {
  constexpr std::size_t kN = rtc::kMaxDeviceChannels;  // 64
  std::vector<double> g(kN, 1.0);
  std::vector<double> lim(kN, 0.5);
  std::vector<double> tau(kN, 0.0);
  const auto r = ComputeDynamicsFeedforward({.max_abs = lim}, {.gravity = g}, kN, tau);
  EXPECT_EQ(r.clamped, ~std::uint64_t{0});
  EXPECT_DOUBLE_EQ(tau[kN - 1], 0.5);
}

// RT-1. The gate counts `operator new` (alloc_gate.hpp); the law is Eigen-free
// so that is the sensor it needs.
TEST(DynamicsFeedforwardLaw, ComputeIsAllocationFree) {
  const std::array<double, 10> g = {0.07, -0.08, 0.004, 0.005, 0.003,
                                    0.04, 0.006, 0.038, 0.005, 0.03};
  const std::array<double, 10> c = {0.001, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 10> m = {0.002, -0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 10> lim = {0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15};
  std::array<double, 10> tau{};
  std::size_t new_calls = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    for (int it = 0; it < 1000; ++it) {
      (void)ComputeDynamicsFeedforward(
          {.gravity_gain = 1.0, .coriolis_gain = 1.0, .inertia_gain = 1.0, .max_abs = lim},
          {.gravity = g, .coriolis = c, .inertia = m}, 10, tau);
    }
    new_calls = heap_gate.count();
  }
  EXPECT_EQ(new_calls, 0u) << "the feedforward law allocated on the RT path (RT-1)";
}

// ═══════════════════════════════════════════════════════════════════════════
// The schema
// ═══════════════════════════════════════════════════════════════════════════

TEST(DynamicsFeedforwardParams, ParsesEveryKey) {
  DynamicsFeedforwardParams p;
  ParseDynamicsFeedforwardParams(
      YAML::Load("enabled: false\ngain: 0.8\ninertia_gain: 0.6\nmax_abs: 0.12"), "gravity_ff", p);
  EXPECT_FALSE(p.enabled);
  EXPECT_DOUBLE_EQ(p.gravity_gain, 0.8);
  EXPECT_DOUBLE_EQ(p.inertia_gain, 0.6);
  EXPECT_DOUBLE_EQ(p.max_abs, 0.12);
}

TEST(DynamicsFeedforwardParams, AbsentOptionalKeysKeepTheSeed) {
  DynamicsFeedforwardParams p;
  p.enabled = false;
  p.gravity_gain = 0.3;
  p.inertia_gain = 0.7;
  ParseDynamicsFeedforwardParams(YAML::Load("max_abs: 0.2"), "gravity_ff", p);
  EXPECT_FALSE(p.enabled);
  EXPECT_DOUBLE_EQ(p.gravity_gain, 0.3);
  EXPECT_DOUBLE_EQ(p.inertia_gain, 0.7);
  // And the default seed is the term OFF: a block without the key ships gravity alone.
  EXPECT_DOUBLE_EQ(DynamicsFeedforwardParams{}.inertia_gain, 0.0);
  EXPECT_DOUBLE_EQ(p.max_abs, 0.2);
}

TEST(DynamicsFeedforwardParams, MaxAbsIsRequired) {
  DynamicsFeedforwardParams p;
  EXPECT_THROW(
      ParseDynamicsFeedforwardParams(YAML::Load("enabled: true\ngain: 1.0"), "gravity_ff", p),
      std::runtime_error);
}

TEST(DynamicsFeedforwardParams, TheBlockItselfIsRequired) {
  DynamicsFeedforwardParams p;
  const YAML::Node cfg = YAML::Load("other: 1");
  EXPECT_THROW(ParseDynamicsFeedforwardParams(cfg["gravity_ff"], "gravity_ff", p),
               std::runtime_error);
  // A default-constructed node (what an absent config file yields) is refused
  // the same way — it is Null, not a map.
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Node{}, "gravity_ff", p), std::runtime_error);
  // And a scalar where a map is expected.
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("0.1"), "gravity_ff", p),
               std::runtime_error);
}

// Only the schema's keys: a misspelt or renamed key (the gravity gain was
// `gain`) would leave its term at the seed without a word. The Coriolis gain
// is refused too — the law has the slot, the schema grows the key with a
// binding that ships the term. The check runs before any value is read.
TEST(DynamicsFeedforwardParams, AKeyOutsideTheSchemaIsRefused) {
  for (const char* key : {"gain", "coriolis_gain", "max_torque"}) {
    DynamicsFeedforwardParams p;
    const std::string yaml = std::string(key) + ": 0.5\nmax_abs: 0.1";
    try {
      ParseDynamicsFeedforwardParams(YAML::Load(yaml), "dynamics_feedforward", p);
      ADD_FAILURE() << key << ": expected a throw";
    } catch (const std::runtime_error& e) {
      const std::string want = std::string("dynamics_feedforward.") + key + " is not a key";
      EXPECT_NE(std::string(e.what()).find(want), std::string::npos) << e.what();
    }
  }
  // Ahead of the max_abs check: the unknown key is named even with no bound.
  DynamicsFeedforwardParams p;
  try {
    ParseDynamicsFeedforwardParams(YAML::Load("gain: 1.0"), "k", p);
    ADD_FAILURE() << "expected a throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("k.gain is not a key"), std::string::npos) << e.what();
  }
  // Every key of the schema together is accepted.
  EXPECT_NO_THROW(ParseDynamicsFeedforwardParams(
      YAML::Load("enabled: true\ngravity_gain: 0.8\ninertia_gain: 0.6\nmax_abs: 0.12"), "k", p));
  EXPECT_DOUBLE_EQ(p.gravity_gain, 0.8);
}

TEST(DynamicsFeedforwardParams, MaxAbsMustBePositiveAndFinite) {
  DynamicsFeedforwardParams p;
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("max_abs: 0.0"), "k", p),
               std::runtime_error);
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("max_abs: -0.1"), "k", p),
               std::runtime_error);
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("max_abs: .nan"), "k", p),
               std::runtime_error);
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("max_abs: .inf"), "k", p),
               std::runtime_error);
  EXPECT_NO_THROW(ParseDynamicsFeedforwardParams(YAML::Load("max_abs: 1.0e-6"), "k", p));
}

TEST(DynamicsFeedforwardParams, GainIsBoundedOnBothSides) {
  DynamicsFeedforwardParams p;
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("gain: -0.01\nmax_abs: 0.1"), "k", p),
               std::runtime_error);
  const std::string over =
      "gain: " + std::to_string(kDynamicsFeedforwardGainMax + 0.01) + "\nmax_abs: 0.1";
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load(over), "k", p), std::runtime_error);
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load("gain: .nan\nmax_abs: 0.1"), "k", p),
               std::runtime_error);
  EXPECT_NO_THROW(ParseDynamicsFeedforwardParams(YAML::Load("gain: 0.0\nmax_abs: 0.1"), "k", p));
  EXPECT_DOUBLE_EQ(p.gravity_gain, 0.0);
  const std::string edge =
      "gain: " + std::to_string(kDynamicsFeedforwardGainMax) + "\nmax_abs: 0.1";
  EXPECT_NO_THROW(ParseDynamicsFeedforwardParams(YAML::Load(edge), "k", p));
}

TEST(DynamicsFeedforwardParams, InertiaGainIsBoundedOnBothSides) {
  DynamicsFeedforwardParams p;
  EXPECT_THROW(
      ParseDynamicsFeedforwardParams(YAML::Load("inertia_gain: -0.01\nmax_abs: 0.1"), "k", p),
      std::runtime_error);
  const std::string over =
      "inertia_gain: " + std::to_string(kDynamicsFeedforwardGainMax + 0.01) + "\nmax_abs: 0.1";
  EXPECT_THROW(ParseDynamicsFeedforwardParams(YAML::Load(over), "k", p), std::runtime_error);
  EXPECT_THROW(
      ParseDynamicsFeedforwardParams(YAML::Load("inertia_gain: .nan\nmax_abs: 0.1"), "k", p),
      std::runtime_error);
  const std::string edge =
      "inertia_gain: " + std::to_string(kDynamicsFeedforwardGainMax) + "\nmax_abs: 0.1";
  EXPECT_NO_THROW(ParseDynamicsFeedforwardParams(YAML::Load(edge), "k", p));
  EXPECT_DOUBLE_EQ(p.inertia_gain, kDynamicsFeedforwardGainMax);
  try {
    ParseDynamicsFeedforwardParams(YAML::Load("inertia_gain: 3.0\nmax_abs: 0.1"), "gravity_ff", p);
    FAIL() << "expected a throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("gravity_ff.inertia_gain"), std::string::npos) << e.what();
  }
}

TEST(DynamicsFeedforwardParams, ErrorNamesTheBlockKey) {
  DynamicsFeedforwardParams p;
  try {
    ParseDynamicsFeedforwardParams(YAML::Load("gain: 1.0"), "gravity_ff", p);
    FAIL() << "expected a throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("gravity_ff.max_abs"), std::string::npos) << e.what();
  }
}

}  // namespace
