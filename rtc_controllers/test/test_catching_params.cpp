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
using rtc::catching::CheckCatchFrameProvisional;
using rtc::catching::kCatchFrameProvisionalKey;
using rtc::catching::ParseCatchingParams;
using rtc::catching::ValidateCatchingParams;

constexpr double kControlRateHz = 500.0;  // repo default (rtc::kDefaultControlRateHz)

// Fully resolved, non-provisional, in-range config — every check below
// mutates one field of this baseline rather than restating the whole tree.
//
// The `io:` / `prediction:` sections joined it with S5.2 and
// `joint_cmd:` / `robot.arm:` / the two `supervisor:` keys with S5.3, as each
// step made its own keys active. The baseline grows with the schema; the
// assertions on it (armable, zero failures, zero warnings) are unchanged,
// which is what keeps "a clean config arms cleanly" meaning the same thing
// before and after.
constexpr const char* kValidYaml = R"(
reference:
  provisional: false
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
  track_err_abort: 0.3
  n_qp: 3
joint_cmd:
  K_p: 20.0
  K_a: 8.0
  K_n: 1.0
  w_task: 1.0
  w_a: 0.5
  w_arm: 0.01
  w_smooth: 0.001
  damping_sq: 0.0001
  qp:
    max_iter: 20
  lag:
    T_arm: 0.0
    lead_enable: false
io:
  n_min: 12
  t_stale: 0.10
  future_tol: 0.001
  horizon_min: 0.51
  track:
    eval_offset: 0.05
    j_warn: 0.05
prediction:
  dt_expected: 0.05
core:
  ball:
    diameter: 0.065
    mass: 0.058
    restitution: 0.8
    provisional: false
sim:
  ball:
    drag_k: 0.02
  io:
    future_tol: 0.1
robot:
  arm:
    limit_margin: 0.05
  hand:
    provisional: false
    rho_eps: 0.02
    q_open: [-0.2, -0.2, -0.2]
    q_pre: [0.0, 0.0, 0.0]
    q_close: [0.5, 0.5, 0.5]
    caging_mask: [true, true, true]
    eta_close: 0.9
    T_close_e2e: 0.15
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

// ── L6 §5.1/§6: q_open, eta_close, T_close_e2e (S4.1) ───────────────────────
//
// The three fields S4a added to the profile. Each is active in both
// configurations and each carries its own report line: a profile that is
// half-drafted must say which key is missing, not fail as one lump.

TEST(CatchingParams, HandQOpenAbsentBlocksArmingWithoutHidingThePair) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"].remove("q_open");
  const CatchingParams p = ParseCatchingParams(root);
  EXPECT_TRUE(p.hand.q_open_tbd);
  EXPECT_FALSE(p.hand.tbd) << "q_pre/q_close were given: the pair must still read as resolved";

  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_open"));
  EXPECT_FALSE(
      ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_pre/q_close"));
}

TEST(CatchingParams, HandQOpenLiteralTbdBlocksArming) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_open"] = "TBD";
  const CatchingParams p = ParseCatchingParams(root);
  EXPECT_TRUE(p.hand.q_open_tbd);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_open"));
}

TEST(CatchingParams, HandProfileTbdReportsThePairNotQOpen) {
  // With the pair itself TBD the joint count is unknown, so q_open cannot be
  // length-checked: exactly one failure, naming the pair.
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_pre"] = "TBD";
  root["robot"]["hand"]["q_close"] = "TBD";
  root["robot"]["hand"].remove("caging_mask");
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_pre/q_close"));
  EXPECT_FALSE(
      ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_open"));
}

TEST(CatchingParams, HandQOpenIsReadInJointOrder) {
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  ASSERT_EQ(p.hand.dof, 3);
  EXPECT_FALSE(p.hand.q_open_tbd);
  for (int i = 0; i < p.hand.dof; ++i) {
    EXPECT_DOUBLE_EQ(p.hand.q_open[static_cast<std::size_t>(i)], -0.2) << "joint " << i;
  }
}

TEST(CatchingParams, RejectsQOpenLengthMismatch) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_open"] = YAML::Load("[0.1, 0.2]");  // the pair has 3 entries
  ExpectRejectMentioning(root, "robot.hand.q_open must have the same length");
}

TEST(CatchingParams, RejectsQOpenScalarThatIsNotTbd) {
  // A typo'd scalar must be refused, not silently read as "still TBD".
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["q_open"] = 0.3;
  ExpectRejectMentioning(root, "robot.hand.q_open must be a sequence");
}

TEST(CatchingParams, HandEtaCloseTbdBlocksArming) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["eta_close"] = "TBD";
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.eta_close"));
}

TEST(CatchingParams, HandEtaCloseOutOfRangeFails) {
  for (const double eta : {0.49, 1.01}) {
    YAML::Node root = ValidRoot();
    root["robot"]["hand"]["eta_close"] = eta;
    const CatchingParams p = ParseCatchingParams(root);
    const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
    EXPECT_FALSE(r.armable) << "eta_close = " << eta;
    EXPECT_TRUE(
        ReportHasFailure(r, CatchingValidationReason::kRangeViolation, "robot.hand.eta_close"))
        << "eta_close = " << eta;
  }
}

TEST(CatchingParams, HandEtaCloseAtBoundsPasses) {
  // L6 §6 gives the range as 0.5–1, inclusive at both ends.
  for (const double eta : {0.5, 1.0}) {
    YAML::Node root = ValidRoot();
    root["robot"]["hand"]["eta_close"] = eta;
    const CatchingParams p = ParseCatchingParams(root);
    const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
    EXPECT_TRUE(r.armable) << "eta_close = " << eta;
  }
}

TEST(CatchingParams, HandTCloseE2eTbdBlocksArming) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["T_close_e2e"] = "TBD";
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "robot.hand.T_close_e2e"));
}

TEST(CatchingParams, HandTCloseE2eNegativeFails) {
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["T_close_e2e"] = -0.01;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(
      ReportHasFailure(r, CatchingValidationReason::kRangeViolation, "robot.hand.T_close_e2e"));
}

TEST(CatchingParams, HandTCloseE2eZeroPasses) {
  // L6 §6 bounds it at >= 0; zero is a degenerate but in-range identification.
  YAML::Node root = ValidRoot();
  root["robot"]["hand"]["T_close_e2e"] = 0.0;
  const CatchingParams p = ParseCatchingParams(root);
  const CatchingValidationReport r = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(r.armable);
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

TEST(CatchingParams, ProvisionalReferenceSimWarnsRealBlocks) {
  // Same rule, fourth flag (L4 §6). It exists because `reference.a_max` is
  // deferred to the D-16 revision while `v_max` is decided, and one TBD in a
  // consumed key refuses the configure in sim — which takes every controller
  // on the robot down with it, because CM latches `bring_up_failed` on any
  // controller's failure. The shipped ur5e_p1b sim did exactly that on
  // 2026-09-22 before this flag existed.
  YAML::Node root = ValidRoot();
  root["reference"]["provisional"] = true;
  const CatchingParams p = ParseCatchingParams(root);
  EXPECT_TRUE(p.reference_provisional);

  const CatchingValidationReport sim = ValidateCatchingParams(p, kControlRateHz, false);
  EXPECT_TRUE(sim.armable);
  EXPECT_TRUE(ReportHasWarning(sim, CatchingValidationReason::kProvisionalWarning, "reference"));

  const CatchingValidationReport real = ValidateCatchingParams(p, kControlRateHz, true);
  EXPECT_FALSE(real.armable);
  EXPECT_TRUE(ReportHasFailure(real, CatchingValidationReason::kProvisionalOnRealArm, "reference"));
}

TEST(CatchingParams, ReferenceProvisionalDefaultsToTrueWhenTheKeyIsAbsent) {
  // Same polarity as every other invented provisional key: a profile that says
  // nothing is NOT cleared. The opposite default would make an omission read
  // as a clearance, which is the one direction that fails open on hardware.
  YAML::Node root = ValidRoot();
  root["reference"].remove("provisional");
  EXPECT_TRUE(ParseCatchingParams(root).reference_provisional);
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

// ── io / prediction: the vision ingress (S5.2) ──────────────────────────────

TEST(CatchingParams, IoNMinMustBeResolvedAndWholeAndWithinCapacity) {
  {  // absent → active TBD, not a silent default
    YAML::Node root = ValidRoot();
    root["io"].remove("n_min");
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_FALSE(r.armable);
    EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "io.n_min"));
  }
  {  // a fraction of a point is a malformed key, refused at parse time rather
     // than truncated — a count is not a value to round.
    YAML::Node root = ValidRoot();
    root["io"]["n_min"] = 10.5;
    ExpectRejectMentioning(root, "whole number");
  }
  {  // above the snapshot capacity the requirement can never be met, and the
    // lane would close permanently with the rejection counter blaming vision.
    YAML::Node root = ValidRoot();
    root["io"]["n_min"] = 41;  // kCap is 40
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_FALSE(r.armable);
    EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kRangeViolation, "io.n_min"));
  }
}

TEST(CatchingParams, IoNMinMustBeAbleToCoverTheRequiredHorizon) {
  // Both keys are individually in range here; only the PAIR is wrong. Nothing
  // else catches it, and the symptom would be messages accepted on count and
  // then rejected on horizon — which reads as a vision fault rather than as
  // the configuration mistake it is.
  YAML::Node root = ValidRoot();
  root["io"]["horizon_min"] = 0.51;
  root["io"]["n_min"] = 6;
  const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
  EXPECT_FALSE(r.armable);
  EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kRangeViolation, "io.n_min"));
}

TEST(CatchingParams, TheHorizonGateCountsINTERVALSNotSamples) {
  // n samples spaced dt apart span (n-1)*dt. The off-by-one is worth its own
  // case because it is invisible in the shipped numbers: 0.51 s at 0.05 s
  // spacing "needs 11" by the wrong arithmetic and 12 by the right one, and 11
  // produces a 0.50 s window that trips the horizon warning on every minimal
  // message — the exact symptom this gate claims to prevent (found by
  // /code-review, 2026-09-22).
  YAML::Node root = ValidRoot();
  root["io"]["horizon_min"] = 0.51;
  root["prediction"]["dt_expected"] = 0.05;

  root["io"]["n_min"] = 11;
  const auto eleven = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
  EXPECT_FALSE(eleven.armable) << "11 points span 0.50 s, not 0.51";
  EXPECT_TRUE(ReportHasFailure(eleven, CatchingValidationReason::kRangeViolation, "io.n_min"));

  root["io"]["n_min"] = 12;
  const auto twelve = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
  EXPECT_TRUE(twelve.armable);
}

TEST(CatchingParams, IoStalenessAndOffsetRangesAreChecked) {
  {
    YAML::Node root = ValidRoot();
    root["io"]["t_stale"] = 0.5;  // range is [0.02, 0.2]
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kRangeViolation, "io.t_stale"));
  }
  {
    YAML::Node root = ValidRoot();
    root["io"]["track"]["eval_offset"] = 1.0;  // range is [0, 0.3]
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_TRUE(
        ReportHasFailure(r, CatchingValidationReason::kRangeViolation, "io.track.eval_offset"));
  }
}

TEST(CatchingParams, JWarnIsDiagnosticAndDoesNotBlockArming) {
  // Nothing decides on it — it is the threshold of a printed warning. Gating
  // arming on it would make an operator resolve a number to get a message
  // they may not want.
  YAML::Node root = ValidRoot();
  root["io"]["track"].remove("j_warn");
  const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
  EXPECT_TRUE(r.armable);
  EXPECT_EQ(r.failure_count, 0u);
}

TEST(CatchingParams, TheSimFutureToleranceIsAnOverlayNotASecondRange) {
  // A-S5-2. The two numbers are two orders of magnitude apart because they
  // measure different things: the sim ball lane's stamps ride the SIM time
  // axis and legitimately lead wall by the in-flight phase error, while a
  // camera stamps at capture and may lead wall only by the clock-sync error.
  const CatchingParams p = ParseCatchingParams(ValidRoot());
  EXPECT_NEAR(rtc::catching::EffectiveFutureTol(p, /*real_arm=*/false).value, 0.1, 1e-12);
  EXPECT_NEAR(rtc::catching::EffectiveFutureTol(p, /*real_arm=*/true).value, 0.001, 1e-12);

  // The sim value would be far out of range for the shared key, and it is not
  // judged against it — that is the whole reason the override exists.
  const auto sim = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/false);
  EXPECT_TRUE(sim.armable);
  EXPECT_EQ(sim.failure_count, 0u);
}

TEST(CatchingParams, ASimConfigWithNoOverrideInheritsTheStrictTolerance) {
  // The fail-closed direction: an absent override must not read as "no limit".
  YAML::Node root = ValidRoot();
  root["sim"].remove("io");
  const CatchingParams p = ParseCatchingParams(root);
  EXPECT_TRUE(p.sim_io_future_tol.tbd);
  EXPECT_NEAR(rtc::catching::EffectiveFutureTol(p, /*real_arm=*/false).value, 0.001, 1e-12);
  const auto r = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/false);
  EXPECT_TRUE(r.armable) << "an absent sim override is not a failure";
}

TEST(CatchingParams, TheSimOverrideIsNotJudgedOnARealArmConfig) {
  // It is inactive there, like sim.ball.drag_k: a hardware bring-up loading
  // the same file must not be blocked by a key that only sim reads.
  YAML::Node root = ValidRoot();
  root["sim"]["io"]["future_tol"] = 9.0;  // out of range even for the sim key
  const CatchingParams p = ParseCatchingParams(root);
  const auto real = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/true);
  EXPECT_TRUE(real.armable);
  const auto sim = ValidateCatchingParams(p, kControlRateHz, /*real_arm=*/false);
  EXPECT_FALSE(sim.armable);
  EXPECT_TRUE(
      ReportHasFailure(sim, CatchingValidationReason::kRangeViolation, "sim.io.future_tol"));
}

// ── joint_cmd / robot.arm / supervisor: the CLIK step (S5.3) ────────────────

TEST(CatchingParams, ClikWeightsMustKeepTheirOrdering) {
  // Each weight below is individually in range. The SET is wrong, and the
  // result is a controller that tracks its posture and treats the catch point
  // as a suggestion — plausible motion, missed ball, and nothing in any log
  // that says why. Three range checks cannot see this.
  {
    YAML::Node root = ValidRoot();
    root["joint_cmd"]["w_arm"] = 2.0;  // above w_task (1.0) and w_a (0.5)
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_FALSE(r.armable);
    EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kWeightOrdering, "joint_cmd.w_arm"));
  }
  {
    // The regulariser must stay below the posture term, or the damping is what
    // decides the redundant degree of freedom.
    YAML::Node root = ValidRoot();
    root["joint_cmd"]["damping_sq"] = 0.5;  // above w_arm (0.01)
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_FALSE(r.armable);
    EXPECT_TRUE(
        ReportHasFailure(r, CatchingValidationReason::kWeightOrdering, "joint_cmd.damping_sq"));
  }
  {
    // And the ordering check does not fire on the shipped ordering.
    const auto r = ValidateCatchingParams(ParseCatchingParams(ValidRoot()), kControlRateHz, false);
    EXPECT_TRUE(r.armable);
  }
}

TEST(CatchingParams, TheSupervisorKeysTheJointLayerReportsToAreActive) {
  // L7 owns both keys, but the joint command layer is what reports to them, so
  // they become active with it. A TBD `track_err_abort` would mean the tracking
  // watchdog has no threshold — the one thing standing between a diverging
  // command and the hardware.
  {
    YAML::Node root = ValidRoot();
    root["supervisor"].remove("track_err_abort");
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_FALSE(r.armable);
    EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd,
                                 "supervisor.track_err_abort"));
  }
  {
    YAML::Node root = ValidRoot();
    root["supervisor"].remove("n_qp");
    const auto r = ValidateCatchingParams(ParseCatchingParams(root), kControlRateHz, false);
    EXPECT_FALSE(r.armable);
    EXPECT_TRUE(ReportHasFailure(r, CatchingValidationReason::kActiveConfigTbd, "supervisor.n_qp"));
  }
}

TEST(CatchingParams, TheQpIterationCapMustBeAtLeastOne) {
  YAML::Node root = ValidRoot();
  root["joint_cmd"]["qp"]["max_iter"] = 0;
  // A zero cap is refused at PARSE time (a count must be positive), which is
  // the stricter of the two answers and the one that names the key.
  ExpectRejectMentioning(root, "whole number");
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

// ── Catch frame provisional (D-17, S2.3a) ───────────────────────────────────
// The flag comes from the robot config's urdf.extra_frames, not from the
// catching: section; the same L0 §5.3 rule applies — sim warns, real arm blocks.
TEST(CatchFrameProvisional, SimWarnsRealArmBlocks) {
  CatchingValidationReport sim;
  CheckCatchFrameProvisional(sim, /*catch_frame_provisional=*/true, /*real_arm_config=*/false);
  EXPECT_TRUE(sim.armable);
  EXPECT_EQ(sim.failure_count, 0U);
  ASSERT_EQ(sim.warning_count, 1U);
  EXPECT_EQ(sim.warnings[0].reason, CatchingValidationReason::kProvisionalWarning);
  EXPECT_STREQ(sim.warnings[0].key, kCatchFrameProvisionalKey);

  CatchingValidationReport real;
  CheckCatchFrameProvisional(real, true, true);
  EXPECT_FALSE(real.armable);
  ASSERT_EQ(real.failure_count, 1U);
  EXPECT_EQ(real.failures[0].reason, CatchingValidationReason::kProvisionalOnRealArm);
  EXPECT_STREQ(real.failures[0].key, kCatchFrameProvisionalKey);

  for (const bool real_arm : {false, true}) {
    CatchingValidationReport confirmed;
    CheckCatchFrameProvisional(confirmed, false, real_arm);
    EXPECT_TRUE(confirmed.armable);
    EXPECT_EQ(confirmed.failure_count, 0U);
    EXPECT_EQ(confirmed.warning_count, 0U);
  }
}

// It adds to an existing report instead of replacing it: an already
// non-armable report stays non-armable when the frame is confirmed.
TEST(CatchFrameProvisional, AppendsToExistingReport) {
  CatchingValidationReport report;
  report.armable = false;
  report.failure_count = 1;
  CheckCatchFrameProvisional(report, false, true);
  EXPECT_FALSE(report.armable);
  EXPECT_EQ(report.failure_count, 1U);
  CheckCatchFrameProvisional(report, true, true);
  EXPECT_EQ(report.failure_count, 2U);
}

}  // namespace
