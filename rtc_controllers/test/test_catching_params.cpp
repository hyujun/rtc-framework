// ── Catching parameter validator (S1.7, G0-C) ────────────────────────────────
//
// Every case below is pinned to a specific reason code and key (never just
// `armable == false`), so a case that goes red for the wrong reason is
// visible rather than masquerading as the intended failure.
//
// NOTE on the ω·h boundary (G0-C: "ωh≥0.828(100·500·5000 Hz 각각)"). L4 §6
// bounds `reference.omega` to [1, 25] rad/s. At every control_rate in the
// framework's own [100, 5000] Hz range, s = omega*h stays far below the
// 0.8284271 discrete-stability limit for any omega in that range (max s is
// 25 * (1/100) = 0.25). Reaching the instability boundary at 100/500/5000 Hz
// therefore requires an omega outside L4 §6's declared range — the
// DiscretizationBoundary test below does that deliberately and checks only
// the discretization-specific reason code, ignoring the (expected,
// independent) range violation that also fires. This is a doc-precision
// finding reported alongside this change, not a bug in the validator: the
// two checks are orthogonal by design and G0-C's boundary requirement is
// about the formula, not about staying inside L4 §6's tuning range.
//
// NOTE on D-9 ("v_tcp_max mismatch"). CATCHING_MASTER §6 / L3 §4.5 fixed
// v_tcp_max = eta_v * reference.v_max as a VALUE COMPUTED AT THE gammaWindow
// CALL SITE, not a second stored YAML key — the old "same value" row it
// replaced is exactly what S0.3 removed (no more paired keys to disagree).
// There is therefore nothing to "mismatch": the D-9 cross-constraint reduces
// to the eta_v range check below (0 < eta_v <= 1), which this suite tests at
// both ends and at the inclusive upper boundary.

#include "rtc_controllers/catching/catching_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using rtc::catching::CatchingParams;
using rtc::catching::CatchingValidationReason;
using rtc::catching::CatchingValidationReport;
using rtc::catching::ParseCatchingParams;
using rtc::catching::ValidateCatchingParams;

constexpr double kControlRateHz = 500.0;  // repo default (rtc::kDefaultControlRateHz)

// Fully resolved, non-provisional, in-range config — every check below
// mutates one field of this baseline rather than restating the whole tree.
constexpr const char* kValidYaml = R"(
reference:
  omega: 10.0
  zeta: 1.0
  v_max: 2.0
  a_max: 5.0
planner:
  gamma:
    eta_v: 0.9
  catchability:
    manipulability_min:
      arm_5row: 0.1
      provisional: false
supervisor:
  decel:
    a_dec: 3.0
core:
  ball:
    diameter: 0.065
    mass: 0.058
    restitution: 0.8
    provisional: false
sim:
  ball:
    drag_k: 0.02
robot:
  hand:
    provisional: false
    rho_eps: 0.02
    q_pre: [0.0, 0.0, 0.0]
    q_close: [0.5, 0.5, 0.5]
    caging_mask: [true, true, true]
)";

YAML::Node ValidRoot() {
  return YAML::Load(kValidYaml);
}

bool ReportHasFailure(const CatchingValidationReport& r, CatchingValidationReason reason,
                      std::string_view key, int index = -1) {
  for (std::size_t i = 0; i < r.failure_count; ++i) {
    const auto& e = r.failures[i];
    if (e.reason == reason && key == e.key && (index < 0 || e.index == index)) {
      return true;
    }
  }
  return false;
}

bool ReportHasWarning(const CatchingValidationReport& r, CatchingValidationReason reason,
                      std::string_view key) {
  for (std::size_t i = 0; i < r.warning_count; ++i) {
    const auto& e = r.warnings[i];
    if (e.reason == reason && key == e.key) {
      return true;
    }
  }
  return false;
}

void ExpectRejectMentioning(const YAML::Node& node, std::string_view needle) {
  try {
    static_cast<void>(ParseCatchingParams(node));
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(needle), std::string::npos)
        << "rejected, but for a different reason.\n  expected substring: " << needle
        << "\n  actual: " << what;
    return;
  }
  ADD_FAILURE() << "expected ParseCatchingParams to refuse this config, but it parsed";
}

// ── Clean baseline ──────────────────────────────────────────────────────────

TEST(CatchingParams, FullyValidYamlArmsCleanlyInSim) {
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/false);
  EXPECT_TRUE(r.armable);
  EXPECT_EQ(r.failure_count, 0u);
  EXPECT_EQ(r.warning_count, 0u);
}

TEST(CatchingParams, FullyValidYamlArmsCleanlyOnRealArm) {
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/true);
  EXPECT_TRUE(r.armable);
  EXPECT_EQ(r.failure_count, 0u);
  EXPECT_EQ(r.warning_count, 0u);
}

// ── G0-C: active-config TBD blocks arming, inactive-config TBD passes ───────

TEST(CatchingParams, ActiveConfigTbdBlocksArming) {
  YAML::Node root = ValidRoot();
  root["core"]["ball"]["diameter"] = "TBD";
  const CatchingParams p = ParseCatchingParams(root);

  // core.ball.* is active in both configurations (L0 §6).
  const CatchingValidationReport sim = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(sim.armable);
  EXPECT_TRUE(
      ReportHasFailure(sim, CatchingValidationReason::kActiveConfigTbd, "core.ball.diameter"));

  const CatchingValidationReport real = ValidateCatchingParams(p, kControlRateHz, true);
  EXPECT_FALSE(real.armable);
  EXPECT_TRUE(
      ReportHasFailure(real, CatchingValidationReason::kActiveConfigTbd, "core.ball.diameter"));
}

TEST(CatchingParams, InactiveConfigTbdPasses) {
  YAML::Node root = ValidRoot();
  root["sim"]["ball"]["drag_k"] = "TBD";
  const CatchingParams p = ParseCatchingParams(root);

  // sim.ball.drag_k is fixture-only: active in sim, inactive on the real arm.
  const CatchingValidationReport sim = ValidateCatchingParams(p, kControlRateHz, /*real=*/false);
  EXPECT_FALSE(sim.armable);
  EXPECT_TRUE(ReportHasFailure(sim, CatchingValidationReason::kActiveConfigTbd, "sim.ball.drag_k"));

  const CatchingValidationReport real = ValidateCatchingParams(p, kControlRateHz, /*real=*/true);
  EXPECT_TRUE(real.armable);
  EXPECT_FALSE(
      ReportHasFailure(real, CatchingValidationReason::kActiveConfigTbd, "sim.ball.drag_k"));
}

// ── L6 §4.2: q_close != q_pre on caging joints ──────────────────────────────

TEST(CatchingParams, HandCagingGapEqualToPreFails) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_close"][0] = 0.0;  // == q_pre[0], caging_mask[0] == true
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kHandCagingGapTooSmall,
                               "robot.hand.q_close", 0));
}

TEST(CatchingParams, HandCagingGapIgnoredOffCagingMask) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_close"][1] = 0.0;        // == q_pre[1] ...
  root["robot"]["hand"]["caging_mask"][1] = false;  // ... but not a caging joint
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(r.armable);
  EXPECT_FALSE(ReportHasFailure(r, CatchingValidationReason::kHandCagingGapTooSmall,
                                "robot.hand.q_close", 1));
}

// ── D-9: 0 < eta_v <= 1 ──────────────────────────────────────────────────────

TEST(CatchingParams, EtaVZeroFails) {
  YAML::Node root = ValidRoot();
  root["planner"]["gamma"]["eta_v"] = 0.0;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kEtaVOutOfRange, "planner.gamma.eta_v"));
}

TEST(CatchingParams, EtaVAboveOneFails) {
  YAML::Node root = ValidRoot();
  root["planner"]["gamma"]["eta_v"] = 1.5;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kEtaVOutOfRange, "planner.gamma.eta_v"));
}

TEST(CatchingParams, EtaVExactlyOnePasses) {
  YAML::Node root = ValidRoot();
  root["planner"]["gamma"]["eta_v"] = 1.0;  // upper bound is inclusive (D-9: 0 < eta_v <= 1)
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(
      ReportHasFailure(r, CatchingValidationReason::kEtaVOutOfRange, "planner.gamma.eta_v"));
}

// ── L7 §4.3: a_dec <= reference.a_max ────────────────────────────────────────

TEST(CatchingParams, DecelExceedingAMaxFails) {
  YAML::Node root = ValidRoot();
  root["supervisor"]["decel"]["a_dec"] = 10.0;  // > a_max (5.0)
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kDecelExceedsAMax, "supervisor.decel.a_dec"));
}

TEST(CatchingParams, DecelEqualToAMaxPasses) {
  YAML::Node root = ValidRoot();
  root["supervisor"]["decel"]["a_dec"] = 5.0;  // == a_max, inclusive bound
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(
      ReportHasFailure(r, CatchingValidationReason::kDecelExceedsAMax, "supervisor.decel.a_dec"));
}

// ── L4 §4.4: reference.zeta must be exactly 1 in v1 ─────────────────────────

TEST(CatchingParams, ZetaNotOneFails) {
  YAML::Node root = ValidRoot();
  root["reference"]["zeta"] = 0.9;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kZetaNotCriticallyDamped, "reference.zeta"));
}

// ── control_rate range [100, 5000] Hz ────────────────────────────────────────

TEST(CatchingParams, ControlRateBelowRangeFails) {
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  const CatchingValidationReport r = ValidateCatchingParams(p, 99.0, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kControlRateOutOfRange, "control_rate"));
}

TEST(CatchingParams, ControlRateAboveRangeFails) {
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  const CatchingValidationReport r = ValidateCatchingParams(p, 5001.0, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kControlRateOutOfRange, "control_rate"));
}

TEST(CatchingParams, ControlRateAtBoundsPasses) {
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  EXPECT_FALSE(ReportHasFailure(ValidateCatchingParams(p, 100.0, false),
                                CatchingValidationReason::kControlRateOutOfRange, "control_rate"));
  EXPECT_FALSE(ReportHasFailure(ValidateCatchingParams(p, 5000.0, false),
                                CatchingValidationReason::kControlRateOutOfRange, "control_rate"));
}

// ── L4 §4.7: s = omega*h discrete stability boundary, per control_rate ──────
// See the file-header NOTE: reaching s >= 0.8284271 at these rates needs an
// omega outside L4 §6's [1, 25] range, so a reference.omega range violation
// is also expected here — only the discretization-specific code is asserted.

class CatchingParamsDiscretization : public ::testing::TestWithParam<double> {};

TEST_P(CatchingParamsDiscretization, JustBelowLimitPasses) {
  const double rate_hz = GetParam();
  const double h = 1.0 / rate_hz;
  constexpr double kLimit = 0.8284271247461903;
  YAML::Node root = ValidRoot();
  root["reference"]["omega"] = (kLimit - 1e-4) / h;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, rate_hz, false);
  EXPECT_FALSE(
      ReportHasFailure(r, CatchingValidationReason::kUnstableDiscretization, "reference.omega"));
}

TEST_P(CatchingParamsDiscretization, AtOrAboveLimitFails) {
  const double rate_hz = GetParam();
  const double h = 1.0 / rate_hz;
  constexpr double kLimit = 0.8284271247461903;
  YAML::Node root = ValidRoot();
  root["reference"]["omega"] = (kLimit + 1e-4) / h;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, rate_hz, false);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kUnstableDiscretization, "reference.omega"));
}

INSTANTIATE_TEST_SUITE_P(AtEachRepoRate, CatchingParamsDiscretization,
                         ::testing::Values(100.0, 500.0, 5000.0));

TEST(CatchingParams, DiscretizationAccuracyWarningInRange) {
  // rate 100 Hz, omega 10 rad/s (both in-range): s = 0.1, over the 0.05
  // accuracy recommendation but far under the 0.828 stability limit.
  YAML::Node root = ValidRoot();
  root["reference"]["omega"] = 10.0;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, 100.0, false);
  EXPECT_TRUE(r.armable);
  EXPECT_TRUE(
      ReportHasWarning(r, CatchingValidationReason::kDiscretizationAccuracy, "reference.omega"));
  EXPECT_FALSE(
      ReportHasFailure(r, CatchingValidationReason::kUnstableDiscretization, "reference.omega"));
}

// ── L0 §5.3: provisional warns in sim, blocks the real arm ──────────────────

TEST(CatchingParams, ProvisionalBallSimWarnsRealBlocks) {
  YAML::Node root = ValidRoot();
  root["core"]["ball"]["provisional"] = true;
  const CatchingParams p = ParseCatchingParams(root);

  const CatchingValidationReport sim = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(sim.armable);
  EXPECT_TRUE(ReportHasWarning(sim, CatchingValidationReason::kProvisionalWarning, "core.ball"));

  const CatchingValidationReport real = ValidateCatchingParams(p, kControlRateHz, true);
  EXPECT_FALSE(real.armable);
  EXPECT_TRUE(ReportHasFailure(real, CatchingValidationReason::kProvisionalOnRealArm, "core.ball"));
}

TEST(CatchingParams, ProvisionalCatchabilitySimWarnsRealBlocks) {
  YAML::Node root = ValidRoot();
  root["planner"]["catchability"]["manipulability_min"]["provisional"] = true;
  const CatchingParams p = ParseCatchingParams(root);

  const CatchingValidationReport sim = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(sim.armable);
  EXPECT_TRUE(ReportHasWarning(sim, CatchingValidationReason::kProvisionalWarning,
                               "planner.catchability.manipulability_min"));

  const CatchingValidationReport real = ValidateCatchingParams(p, kControlRateHz, true);
  EXPECT_FALSE(real.armable);
  EXPECT_TRUE(ReportHasFailure(real, CatchingValidationReason::kProvisionalOnRealArm,
                               "planner.catchability.manipulability_min"));
}

TEST(CatchingParams, ProvisionalHandSimWarnsRealBlocks) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["provisional"] = true;
  const CatchingParams p = ParseCatchingParams(root);

  const CatchingValidationReport sim = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(sim.armable);
  EXPECT_TRUE(ReportHasWarning(sim, CatchingValidationReason::kProvisionalWarning, "robot.hand"));

  const CatchingValidationReport real = ValidateCatchingParams(p, kControlRateHz, true);
  EXPECT_FALSE(real.armable);
  EXPECT_TRUE(
      ReportHasFailure(real, CatchingValidationReason::kProvisionalOnRealArm, "robot.hand"));
}

// ── ParseCatchingParams rejection paths ──────────────────────────────────────

TEST(CatchingParams, RejectsNonMapRoot) {
  ExpectRejectMentioning(YAML::Load("[1, 2, 3]"), "must be a map");
}

TEST(CatchingParams, RejectsMismatchedHandArrayLengths) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_close"] = YAML::Load("[0.1, 0.2]");  // q_pre still has 3 entries
  ExpectRejectMentioning(root, "same length");
}

TEST(CatchingParams, RejectsGarbageTbdField) {
  YAML::Node root = ValidRoot();
  root["reference"]["v_max"] = "not-a-number-or-tbd";
  ExpectRejectMentioning(root, "'v_max'");
}

TEST(CatchingParams, RejectsHandArrayLongerThanCapacity) {
  YAML::Node root = ValidRoot();
  std::string long_seq = "[";
  for (std::size_t i = 0; i < rtc::catching::kMaxHandDof + 1; ++i) {
    long_seq += (i == 0 ? "" : ", ") + std::to_string(0.01 * static_cast<double>(i));
  }
  long_seq += "]";
  root["robot"]["hand"]["q_pre"] = YAML::Load(long_seq);
  root["robot"]["hand"]["q_close"] = YAML::Load(long_seq);
  ExpectRejectMentioning(root, "kMaxHandDof");
}

TEST(CatchingParams, RejectsNonMapSection) {
  YAML::Node root = ValidRoot();
  root["planner"] = 5;
  ExpectRejectMentioning(root, "section 'planner'");
}

TEST(CatchingParams, RejectsEmptyHandArrays) {
  // An empty profile would parse as "resolved, 0 joints" and the caging-gap
  // check would cover nothing — the validator would then report armable.
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_pre"] = YAML::Load("[]");
  root["robot"]["hand"]["q_close"] = YAML::Load("[]");
  root["robot"]["hand"].remove("caging_mask");
  ExpectRejectMentioning(root, "must not be empty");
}

// ── Absent sections: defaults, never a foreign exception type ────────────────

TEST(CatchingParams, RealArmConfigMayOmitSimSection) {
  // sim.* is inactive on the real arm (G0-C); omitting the tree must parse
  // and arm, not throw YAML::InvalidNode from subscripting a missing node.
  YAML::Node root = ValidRoot();
  root.remove("sim");
  CatchingParams p;
  ASSERT_NO_THROW(p = ParseCatchingParams(root));
  EXPECT_TRUE(p.sim_ball_drag_k.tbd);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/true);
  EXPECT_TRUE(r.armable);
  EXPECT_EQ(r.failure_count, 0u);
}

TEST(CatchingParams, EveryOtherAbsentSectionParsesButBlocksRealArm) {
  // Each non-sim section removed in turn: parsing must not throw anything,
  // and the real-arm config must refuse to arm — every one of these sections
  // holds a TBD default or a provisional flag defaulting to true. (In sim an
  // absent `planner:` does arm: its values have doc defaults, L3 §6, and the
  // provisional flag only warns there.)
  for (const char* key : {"reference", "planner", "supervisor", "core", "robot"}) {
    YAML::Node root = ValidRoot();
    ASSERT_TRUE(root[key]) << key << " missing from the baseline YAML";
    root.remove(key);
    CatchingParams p;
    try {
      p = ParseCatchingParams(root);
    } catch (const std::exception& e) {
      ADD_FAILURE() << "absent '" << key << "' threw: " << e.what();
      continue;
    }
    const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/true);
    EXPECT_FALSE(r.armable) << "absent '" << key << "' still armed on the real arm";
  }
}

}  // namespace
