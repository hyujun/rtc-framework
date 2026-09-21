// ── Catch-pose IK schema (dynamic_catching S3.5a) ────────────────────────────
//
// The suite is built around three questions the parser can only answer wrongly
// in silence:
//
// 1. DOES THE KEY REACH THE STRUCT? Every case that sets a field sets it to a
//    value DIFFERENT from the in-code default, and asserts that difference
//    first (`ASSERT_NE` against `CatchPoseIkOptions{}`). A case that writes a
//    field's own default passes with the assignment deleted, which is the
//    failure mode these cases exist to exclude — and the ASSERT_NE means a
//    future default change that collides with a chosen value goes red here
//    rather than turning the case vacuous.
//
// 2. IS IT WIRED TO THE RIGHT KEY? One case sets EVERY key at once to mutually
//    distinct values. Per-field cases cannot see a crossed pair (`mu` reading
//    `rho`'s node looks fine when only `rho` is set); with all of them set at
//    once, a crossed pair puts the other field's number in place.
//
// 3. IS A TYPO DEFAULTED? `planner.ik` is owned whole by this parser, so an
//    unknown key there is refused. That is the one behaviour a permissive
//    parser would get backwards, so it is asserted directly, and its converse
//    too: the retired keys L3 §6 names are REPORTED, not refused.
//
// DocDefaultsAgreeWithStructDefaults is the only case that hardcodes numbers.
// It is the doc-vs-code sensor: the literals are L3 §6's 기본값 column, so if
// either side moves, that case — and not some downstream tuning surprise —
// reports it.

#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catching_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using rtc::catching::ActiveManipulabilityMin;
using rtc::catching::CatchPoseIkConfig;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CatchPoseIkRetiredKeys;
using rtc::catching::ManipDefinition;
using rtc::catching::ParseCatchPoseIkParams;

const CatchPoseIkOptions kDefault{};

/// `catching:` root carrying `planner.ik` as an (initially empty) map.
YAML::Node IkRoot() {
  YAML::Node root = YAML::Load("planner:\n  ik: {}\n");
  return root;
}

/// `catching:` root carrying `planner.catchability` as an empty map.
YAML::Node CatchabilityRoot() {
  return YAML::Load("planner:\n  catchability: {}\n");
}

void ExpectRejectMentioning(const YAML::Node& node, std::string_view needle) {
  try {
    static_cast<void>(ParseCatchPoseIkParams(node));
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(needle), std::string::npos)
        << "rejected, but for a different reason.\n  expected substring: " << needle
        << "\n  actual: " << what;
    return;
  }
  ADD_FAILURE() << "expected ParseCatchPoseIkParams to refuse this config, but it parsed";
}

/// Field-by-field (never memcmp) equality with the in-code defaults.
void ExpectAllDefaults(const CatchPoseIkConfig& cfg, const char* what) {
  const CatchPoseIkOptions& o = cfg.options;
  SCOPED_TRACE(what);
  EXPECT_EQ(o.max_iter, kDefault.max_iter);
  EXPECT_DOUBLE_EQ(o.eps_pos, kDefault.eps_pos);
  EXPECT_DOUBLE_EQ(o.alpha_max, kDefault.alpha_max);
  EXPECT_DOUBLE_EQ(o.rho, kDefault.rho);
  EXPECT_DOUBLE_EQ(o.sigma0, kDefault.sigma0);
  EXPECT_DOUBLE_EQ(o.lambda_max, kDefault.lambda_max);
  EXPECT_DOUBLE_EQ(o.dq_step_max, kDefault.dq_step_max);
  EXPECT_DOUBLE_EQ(o.mu, kDefault.mu);
  EXPECT_DOUBLE_EQ(o.qp_eps_abs, kDefault.qp_eps_abs);
  EXPECT_EQ(o.qp_max_iter, kDefault.qp_max_iter);
  EXPECT_DOUBLE_EQ(o.k_null, kDefault.k_null);
  EXPECT_DOUBLE_EQ(o.k_manip, kDefault.k_manip);
  EXPECT_DOUBLE_EQ(o.manip_grad_tol, kDefault.manip_grad_tol);
  EXPECT_DOUBLE_EQ(o.fd_step, kDefault.fd_step);  // no YAML key: struct default only
  EXPECT_DOUBLE_EQ(o.v_eps, kDefault.v_eps);
  EXPECT_EQ(o.definition, kDefault.definition);
  EXPECT_DOUBLE_EQ(o.manipulability_min, kDefault.manipulability_min);

  // The TBD record: alpha_max and arm_6row are open in L3 §6, arm_5row is not.
  EXPECT_TRUE(cfg.alpha_max.tbd);
  EXPECT_FALSE(cfg.manipulability_min_arm_5row.tbd);
  EXPECT_DOUBLE_EQ(cfg.manipulability_min_arm_5row.value, kDefault.manipulability_min);
  EXPECT_TRUE(cfg.manipulability_min_arm_6row.tbd);
  EXPECT_TRUE(cfg.manipulability_min_provisional);  // fail-closed
}

// ── Table of every double-valued planner.ik key ─────────────────────────────
// `distinct` is in range and deliberately unequal to the default.
struct DoubleField {
  const char* key;
  double distinct;
  double (*get)(const CatchPoseIkOptions&);
};

const DoubleField kDoubleFields[] = {
    {"eps_pos", 0.005, +[](const CatchPoseIkOptions& o) { return o.eps_pos; }},
    {"rho", 0.375, +[](const CatchPoseIkOptions& o) { return o.rho; }},
    {"sigma0", 0.0025, +[](const CatchPoseIkOptions& o) { return o.sigma0; }},
    {"lambda_max", 0.05, +[](const CatchPoseIkOptions& o) { return o.lambda_max; }},
    {"dq_step_max", 0.0625, +[](const CatchPoseIkOptions& o) { return o.dq_step_max; }},
    {"mu", 0.0003, +[](const CatchPoseIkOptions& o) { return o.mu; }},
    {"qp_eps_abs", 1e-9, +[](const CatchPoseIkOptions& o) { return o.qp_eps_abs; }},
    {"k_null", 0.75, +[](const CatchPoseIkOptions& o) { return o.k_null; }},
    {"k_manip", 1.5, +[](const CatchPoseIkOptions& o) { return o.k_manip; }},
    {"manip_grad_tol", 0.0075, +[](const CatchPoseIkOptions& o) { return o.manip_grad_tol; }},
    {"v_eps", 0.0001, +[](const CatchPoseIkOptions& o) { return o.v_eps; }},
};

struct IntField {
  const char* key;
  int distinct;
  int (*get)(const CatchPoseIkOptions&);
};

const IntField kIntFields[] = {
    {"max_iter", 37, +[](const CatchPoseIkOptions& o) { return o.max_iter; }},
    {"qp_max_iter", 123, +[](const CatchPoseIkOptions& o) { return o.qp_max_iter; }},
};

// ── Defaults ────────────────────────────────────────────────────────────────

TEST(CatchPoseIkParams, AbsentPlannerSectionGivesStructDefaults) {
  ExpectAllDefaults(ParseCatchPoseIkParams(YAML::Load("{}")), "empty catching: root");
}

TEST(CatchPoseIkParams, EmptyIkSectionGivesStructDefaults) {
  ExpectAllDefaults(ParseCatchPoseIkParams(IkRoot()), "planner.ik: {}");
}

TEST(CatchPoseIkParams, NullIkSectionGivesStructDefaults) {
  // `ik:` with nothing under it is a null node, not a map — it must read as
  // "absent", not throw from being subscripted.
  ExpectAllDefaults(ParseCatchPoseIkParams(YAML::Load("planner:\n  ik:\n")), "planner.ik: null");
}

TEST(CatchPoseIkParams, UnrelatedPlannerKeysAreIgnored) {
  // planner.* at large belongs to other consumers (L3 §6 lists ~30 keys): this
  // parser must not police it, only planner.ik.
  YAML::Node root = YAML::Load(R"(
planner:
  budget_s: 0.01
  n_settle: 3
  hand:
    d_eff: 0.095
)");
  ExpectAllDefaults(ParseCatchPoseIkParams(root), "foreign planner keys present");
}

TEST(CatchPoseIkParams, DocDefaultsAgreeWithStructDefaults) {
  // The L3 §6 기본값 column, hardcoded on purpose: this is the doc-vs-code
  // sensor. `alpha_max` IS here now — the discrepancy this parser surfaced
  // (doc `TBD` vs a live struct default) was reconciled toward the code on
  // 2026-09-21, so L3 §6 records `0.26 (provisional)` and the number is
  // pinnable. It remains a `TbdDouble` in the config because what is still
  // open is the evidence that closes it (the S3.5a map's `theta` distribution),
  // not the value — that openness is asserted separately as a flag below.
  EXPECT_DOUBLE_EQ(kDefault.alpha_max, 0.26);
  EXPECT_EQ(kDefault.max_iter, 20);
  EXPECT_DOUBLE_EQ(kDefault.eps_pos, 0.002);
  EXPECT_DOUBLE_EQ(kDefault.rho, 0.1);
  EXPECT_DOUBLE_EQ(kDefault.sigma0, 1e-3);
  EXPECT_DOUBLE_EQ(kDefault.lambda_max, 1e-2);
  EXPECT_DOUBLE_EQ(kDefault.dq_step_max, 0.15);
  EXPECT_DOUBLE_EQ(kDefault.mu, 1e-4);
  EXPECT_DOUBLE_EQ(kDefault.qp_eps_abs, 1e-10);
  EXPECT_EQ(kDefault.qp_max_iter, 50);
  EXPECT_DOUBLE_EQ(kDefault.k_null, 0.0);
  EXPECT_DOUBLE_EQ(kDefault.k_manip, 0.0);
  EXPECT_DOUBLE_EQ(kDefault.manip_grad_tol, 1e-4);
  EXPECT_DOUBLE_EQ(kDefault.v_eps, 1e-6);
  EXPECT_EQ(kDefault.definition, ManipDefinition::kArm5Row);
  EXPECT_DOUBLE_EQ(kDefault.manipulability_min, 0.1);  // = the arm_5row row
}

// ── Every parsed field reaches the struct ───────────────────────────────────

TEST(CatchPoseIkParams, EveryDoubleFieldReachesTheStruct) {
  for (const DoubleField& f : kDoubleFields) {
    SCOPED_TRACE(f.key);
    ASSERT_NE(f.distinct, f.get(kDefault)) << "case value equals the default: it proves nothing";
    YAML::Node root = IkRoot();
    root["planner"]["ik"][f.key] = f.distinct;
    const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
    EXPECT_DOUBLE_EQ(f.get(cfg.options), f.distinct);
  }
}

TEST(CatchPoseIkParams, EveryIntFieldReachesTheStruct) {
  for (const IntField& f : kIntFields) {
    SCOPED_TRACE(f.key);
    ASSERT_NE(f.distinct, f.get(kDefault)) << "case value equals the default: it proves nothing";
    YAML::Node root = IkRoot();
    root["planner"]["ik"][f.key] = f.distinct;
    const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
    EXPECT_EQ(f.get(cfg.options), f.distinct);
  }
}

TEST(CatchPoseIkParams, AllFieldsAtOnceAreNotCrossWired) {
  // Mutually distinct values for every key at once: a key read through the
  // wrong node shows up as another field's number.
  YAML::Node root = IkRoot();
  for (const DoubleField& f : kDoubleFields) {
    root["planner"]["ik"][f.key] = f.distinct;
  }
  for (const IntField& f : kIntFields) {
    root["planner"]["ik"][f.key] = f.distinct;
  }
  root["planner"]["ik"]["alpha_max"] = 0.4;
  root["planner"]["catchability"]["definition"] = "arm_6row";
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = 0.42;
  root["planner"]["catchability"]["manipulability_min"]["arm_6row"] = 0.77;
  root["planner"]["catchability"]["manipulability_min"]["provisional"] = false;

  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
  for (const DoubleField& f : kDoubleFields) {
    SCOPED_TRACE(f.key);
    EXPECT_DOUBLE_EQ(f.get(cfg.options), f.distinct);
  }
  for (const IntField& f : kIntFields) {
    SCOPED_TRACE(f.key);
    EXPECT_EQ(f.get(cfg.options), f.distinct);
  }
  EXPECT_DOUBLE_EQ(cfg.options.alpha_max, 0.4);
  EXPECT_FALSE(cfg.alpha_max.tbd);
  EXPECT_EQ(cfg.options.definition, ManipDefinition::kArm6Row);
  EXPECT_DOUBLE_EQ(cfg.manipulability_min_arm_5row.value, 0.42);
  EXPECT_DOUBLE_EQ(cfg.manipulability_min_arm_6row.value, 0.77);
  EXPECT_DOUBLE_EQ(cfg.options.manipulability_min, 0.77) << "the active definition's row";
  EXPECT_FALSE(cfg.manipulability_min_provisional);

  // fd_step has no key in L3 §6 and must have stayed put.
  EXPECT_DOUBLE_EQ(cfg.options.fd_step, kDefault.fd_step);
}

TEST(CatchPoseIkParams, FdStepHasNoYamlKey) {
  // The one struct field this schema deliberately does not expose: naming it
  // is a typo, not a tuning knob.
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["fd_step"] = 1e-4;
  ExpectRejectMentioning(root, "unknown key 'planner.ik.fd_step'");
}

// ── alpha_max: the TBD convention ───────────────────────────────────────────

TEST(CatchPoseIkParams, AlphaMaxAbsentIsTbdAndKeepsTheProvisionalDefault) {
  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(IkRoot());
  EXPECT_TRUE(cfg.alpha_max.tbd);
  EXPECT_TRUE(std::isnan(cfg.alpha_max.value));
  EXPECT_DOUBLE_EQ(cfg.options.alpha_max, kDefault.alpha_max)
      << "absent must resolve to the struct default, not to NaN";
}

TEST(CatchPoseIkParams, AlphaMaxLiteralTbdIsTbdAndKeepsTheProvisionalDefault) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["alpha_max"] = "TBD";
  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
  EXPECT_TRUE(cfg.alpha_max.tbd) << "'TBD' must not silently become a number";
  EXPECT_DOUBLE_EQ(cfg.options.alpha_max, kDefault.alpha_max);
}

TEST(CatchPoseIkParams, AlphaMaxNumberResolvesAndReachesTheStruct) {
  ASSERT_NE(0.4, kDefault.alpha_max);
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["alpha_max"] = 0.4;
  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
  EXPECT_FALSE(cfg.alpha_max.tbd);
  EXPECT_DOUBLE_EQ(cfg.alpha_max.value, 0.4);
  EXPECT_DOUBLE_EQ(cfg.options.alpha_max, 0.4);
}

TEST(CatchPoseIkParams, AlphaMaxAcceptsBothEndsOfItsRange) {
  // L3 §6 gives 0–π/2, both inclusive.
  for (const double a : {0.0, 1.5707963267948966}) {
    YAML::Node root = IkRoot();
    root["planner"]["ik"]["alpha_max"] = a;
    const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
    EXPECT_DOUBLE_EQ(cfg.options.alpha_max, a);
  }
}

TEST(CatchPoseIkParams, AlphaMaxAboveHalfPiRejected) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["alpha_max"] = 1.6;  // > π/2
  ExpectRejectMentioning(root, "'planner.ik.alpha_max'");
}

// ── The catchability gate: definition selects the row ───────────────────────

TEST(CatchPoseIkParams, Arm5RowThresholdReachesTheStruct) {
  ASSERT_NE(0.42, kDefault.manipulability_min);
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = 0.42;
  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
  EXPECT_DOUBLE_EQ(cfg.options.manipulability_min, 0.42);
  EXPECT_DOUBLE_EQ(ActiveManipulabilityMin(cfg).value, 0.42);
}

TEST(CatchPoseIkParams, DefinitionArm6RowSelectsItsOwnRow) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["definition"] = "arm_6row";
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = 0.42;
  root["planner"]["catchability"]["manipulability_min"]["arm_6row"] = 0.77;
  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
  EXPECT_EQ(cfg.options.definition, ManipDefinition::kArm6Row);
  EXPECT_DOUBLE_EQ(cfg.options.manipulability_min, 0.77)
      << "the arm_5row number must not leak into an arm_6row gate";
}

TEST(CatchPoseIkParams, DefinitionArm5RowIsAcceptedExplicitly) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["definition"] = "arm_5row";
  const CatchPoseIkConfig cfg = ParseCatchPoseIkParams(root);
  EXPECT_EQ(cfg.options.definition, ManipDefinition::kArm5Row);
  EXPECT_DOUBLE_EQ(cfg.options.manipulability_min, kDefault.manipulability_min);
}

TEST(CatchPoseIkParams, TbdActiveThresholdLeavesTheOptionsUnusable) {
  // L3 §6's arm_6row default IS TBD, and there is no provisional stand-in in a
  // different unit to fall back on: the threshold must reach Solve() as
  // non-finite so it refuses the options rather than gating on the arm_5row
  // number. Both routes to that state are checked.
  YAML::Node by_definition = CatchabilityRoot();
  by_definition["planner"]["catchability"]["definition"] = "arm_6row";
  const CatchPoseIkConfig a = ParseCatchPoseIkParams(by_definition);
  EXPECT_TRUE(ActiveManipulabilityMin(a).tbd);
  EXPECT_FALSE(std::isfinite(a.options.manipulability_min));

  YAML::Node by_literal = CatchabilityRoot();
  by_literal["planner"]["catchability"]["manipulability_min"]["arm_5row"] = "TBD";
  const CatchPoseIkConfig b = ParseCatchPoseIkParams(by_literal);
  EXPECT_TRUE(b.manipulability_min_arm_5row.tbd);
  EXPECT_FALSE(std::isfinite(b.options.manipulability_min));
}

TEST(CatchPoseIkParams, ProvisionalFlagIsRead) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["provisional"] = false;
  EXPECT_FALSE(ParseCatchPoseIkParams(root).manipulability_min_provisional);
}

TEST(CatchPoseIkParams, RejectsUnknownDefinition) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["definition"] = "arm_7row";
  ExpectRejectMentioning(root, "'planner.catchability.definition'");
}

TEST(CatchPoseIkParams, RejectsNonScalarDefinition) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["definition"] = YAML::Load("[arm_5row]");
  ExpectRejectMentioning(root, "'planner.catchability.definition'");
}

TEST(CatchPoseIkParams, RejectsNegativeThreshold) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = -0.1;
  ExpectRejectMentioning(root, "'planner.catchability.manipulability_min.arm_5row'");
}

TEST(CatchPoseIkParams, RejectsNonNumericThreshold) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["arm_6row"] = "lowish";
  ExpectRejectMentioning(root, "'planner.catchability.manipulability_min.arm_6row'");
}

// ── Typo protection (the behaviour a permissive parser gets backwards) ──────

TEST(CatchPoseIkParams, RejectsUnknownKeyUnderPlannerIk) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["eps_position"] = 0.005;  // plausible misspelling of eps_pos
  ExpectRejectMentioning(root, "unknown key 'planner.ik.eps_position'");
}

TEST(CatchPoseIkParams, RejectsUnknownKeyEvenWhenEveryRealKeyIsPresent) {
  YAML::Node root = IkRoot();
  for (const DoubleField& f : kDoubleFields) {
    root["planner"]["ik"][f.key] = f.distinct;
  }
  root["planner"]["ik"]["rhoo"] = 0.2;
  ExpectRejectMentioning(root, "unknown key 'planner.ik.rhoo'");
}

TEST(CatchPoseIkParams, RetiredKeysAreReportedNotRejected) {
  // L3 §6: `lambda` and `manip_min` were removed in v0.5. A deployed config may
  // still carry them; their meaning is known, so they are migration reports.
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["lambda"] = 0.05;
  root["planner"]["ik"]["manip_min"] = 0.1;
  root["planner"]["ik"]["rho"] = 0.375;

  CatchPoseIkRetiredKeys retired;
  CatchPoseIkConfig cfg;
  ASSERT_NO_THROW(cfg = ParseCatchPoseIkParams(root, &retired));
  EXPECT_TRUE(retired.lambda);
  EXPECT_TRUE(retired.manip_min);
  EXPECT_DOUBLE_EQ(cfg.options.rho, 0.375) << "the live keys still parse alongside them";
  // The retired values are NOT smuggled into the live parameters.
  EXPECT_DOUBLE_EQ(cfg.options.lambda_max, kDefault.lambda_max);
  EXPECT_DOUBLE_EQ(cfg.options.manipulability_min, kDefault.manipulability_min);
}

TEST(CatchPoseIkParams, RetiredKeysReportFalseWhenAbsent) {
  CatchPoseIkRetiredKeys retired;
  retired.lambda = true;  // must be cleared by the call, not OR-ed into
  retired.manip_min = true;
  static_cast<void>(ParseCatchPoseIkParams(IkRoot(), &retired));
  EXPECT_FALSE(retired.lambda);
  EXPECT_FALSE(retired.manip_min);
}

TEST(CatchPoseIkParams, RetiredPointerIsOptional) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["lambda"] = 0.05;
  EXPECT_NO_THROW(static_cast<void>(ParseCatchPoseIkParams(root)));
}

// ── Out-of-range, per the L3 §6 범위 column ─────────────────────────────────

TEST(CatchPoseIkParams, RejectsOutOfRangeDoubles) {
  struct Case {
    const char* key;
    double value;
  };

  // rho is bounded on BOTH sides (0.01–1); the ">0" and "≥0" rows are checked
  // at the boundary they actually differ on.
  const Case cases[] = {
      {"rho", 2.0},          {"rho", 0.005},      {"mu", 0.0},          {"mu", -1e-4},
      {"sigma0", 0.0},       {"qp_eps_abs", 0.0}, {"dq_step_max", 0.0}, {"v_eps", 0.0},
      {"lambda_max", -1e-3}, {"k_null", -0.5},    {"k_manip", -1.0},    {"manip_grad_tol", -1e-6},
      {"eps_pos", 0.0},      {"eps_pos", -0.002},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(std::string(c.key) + " = " + std::to_string(c.value));
    YAML::Node root = IkRoot();
    root["planner"]["ik"][c.key] = c.value;
    ExpectRejectMentioning(root, std::string("'planner.ik.") + c.key + "'");
  }
}

TEST(CatchPoseIkParams, AcceptsTheInclusiveEndsOfBoundedRanges) {
  // The complement of the case above: the bounds L3 §6 states as inclusive
  // must parse, or the range check is one-sided in the wrong direction.
  struct Case {
    const char* key;
    double value;
  };

  const Case cases[] = {
      {"rho", 0.01},   {"rho", 1.0},     {"lambda_max", 0.0},
      {"k_null", 0.0}, {"k_manip", 0.0}, {"manip_grad_tol", 0.0},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(std::string(c.key) + " = " + std::to_string(c.value));
    YAML::Node root = IkRoot();
    root["planner"]["ik"][c.key] = c.value;
    EXPECT_NO_THROW(static_cast<void>(ParseCatchPoseIkParams(root)));
  }
}

TEST(CatchPoseIkParams, RejectsOutOfRangeIntegers) {
  struct Case {
    const char* key;
    int value;
  };

  const Case cases[] = {
      {"max_iter", 0}, {"max_iter", -5}, {"max_iter", 101}, {"qp_max_iter", 0}, {"qp_max_iter", -1},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(std::string(c.key) + " = " + std::to_string(c.value));
    YAML::Node root = IkRoot();
    root["planner"]["ik"][c.key] = c.value;
    ExpectRejectMentioning(root, std::string("'planner.ik.") + c.key + "'");
  }
}

TEST(CatchPoseIkParams, AcceptsIntegerRangeEnds) {
  for (const int v : {1, 100}) {
    YAML::Node root = IkRoot();
    root["planner"]["ik"]["max_iter"] = v;
    EXPECT_EQ(ParseCatchPoseIkParams(root).options.max_iter, v);
  }
}

// ── Wrong YAML type ─────────────────────────────────────────────────────────

TEST(CatchPoseIkParams, RejectsStringWhereADoubleIsExpected) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["rho"] = "fast";
  ExpectRejectMentioning(root, "'planner.ik.rho' must be a number");
}

TEST(CatchPoseIkParams, RejectsSequenceWhereAScalarIsExpected) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["rho"] = YAML::Load("[0.1, 0.2]");
  ExpectRejectMentioning(root, "must be a number, got a sequence");
}

TEST(CatchPoseIkParams, RejectsMapWhereAScalarIsExpected) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["v_eps"] = YAML::Load("{value: 1e-6}");
  ExpectRejectMentioning(root, "must be a number, got a map");
}

TEST(CatchPoseIkParams, RejectsSequenceForTheTbdScalar) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["alpha_max"] = YAML::Load("[0.26]");
  ExpectRejectMentioning(root, "must be a number or the literal 'TBD', got a sequence");
}

TEST(CatchPoseIkParams, RejectsNonIntegerWhereAnIntIsExpected) {
  // 3.5 must not truncate to 3: an iteration cap the config did not ask for.
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["max_iter"] = 3.5;
  ExpectRejectMentioning(root, "'planner.ik.max_iter' must be an integer");
}

TEST(CatchPoseIkParams, RejectsStringWhereAnIntIsExpected) {
  YAML::Node root = IkRoot();
  root["planner"]["ik"]["qp_max_iter"] = "fifty";
  ExpectRejectMentioning(root, "'planner.ik.qp_max_iter' must be an integer");
}

TEST(CatchPoseIkParams, RejectsNonBoolProvisional) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["provisional"] = "maybe";
  ExpectRejectMentioning(root, "must be a bool");
}

// ── Malformed structure ─────────────────────────────────────────────────────

TEST(CatchPoseIkParams, RejectsNonMapRoot) {
  ExpectRejectMentioning(YAML::Load("[1, 2, 3]"), "must be a map");
}

TEST(CatchPoseIkParams, RejectsNonMapIkSection) {
  ExpectRejectMentioning(YAML::Load("planner:\n  ik: 0.5\n"), "section 'planner.ik' must be a map");
}

TEST(CatchPoseIkParams, RejectsNonMapPlannerSection) {
  ExpectRejectMentioning(YAML::Load("planner: 5\n"), "section 'planner' must be a map");
}

TEST(CatchPoseIkParams, RejectsNonMapCatchabilitySection) {
  ExpectRejectMentioning(YAML::Load("planner:\n  catchability: 5\n"),
                         "section 'planner.catchability' must be a map");
}

TEST(CatchPoseIkParams, RejectsNonMapManipulabilityMinSection) {
  ExpectRejectMentioning(YAML::Load("planner:\n  catchability:\n    manipulability_min: 0.1\n"),
                         "section 'planner.catchability.manipulability_min' must be a map");
}

// ── One config load feeds both parsers ──────────────────────────────────────

TEST(CatchPoseIkParams, SharesTheArm5RowKeyWithTheG0CValidator) {
  // The one key both schemas read. They must agree on it, or the map's gate
  // and the arming check would disagree about the same YAML.
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = 0.42;
  const CatchPoseIkConfig ik = ParseCatchPoseIkParams(root);
  const rtc::catching::CatchingParams g0c = rtc::catching::ParseCatchingParams(root);
  ASSERT_FALSE(g0c.planner_catchability_manip_min_arm5row.tbd);
  EXPECT_DOUBLE_EQ(ik.manipulability_min_arm_5row.value,
                   g0c.planner_catchability_manip_min_arm5row.value);
  EXPECT_DOUBLE_EQ(ik.options.manipulability_min, g0c.planner_catchability_manip_min_arm5row.value);
}

// The two parsers share how the key is READ and differ, by design, in how an
// out-of-range value is REPORTED (catch_pose_ik_params.hpp §Scope). Both halves
// are pinned here so the documented difference cannot drift.

TEST(CatchPoseIkParams, SharedKeyIsReadTheSameWayByBothParsers) {
  // Every spelling the shared reader classifies, and the record each yields.
  const struct {
    const char* spelling;
    bool tbd;
    double value;  // compared only when resolved
  } kCases[] = {
      {"0.42", false, 0.42}, {"0", false, 0.0},   {"1e-3", false, 1e-3},
      {"TBD", true, 0.0},    {".nan", true, 0.0}, {".inf", true, 0.0},
  };

  for (const auto& c : kCases) {
    SCOPED_TRACE(c.spelling);
    const YAML::Node root = YAML::Load(
        std::string("planner:\n  catchability:\n    manipulability_min:\n      arm_5row: ") +
        c.spelling + "\n");
    const rtc::catching::TbdDouble ik = ParseCatchPoseIkParams(root).manipulability_min_arm_5row;
    const rtc::catching::TbdDouble g0c =
        rtc::catching::ParseCatchingParams(root).planner_catchability_manip_min_arm5row;
    EXPECT_EQ(ik.tbd, c.tbd);
    EXPECT_EQ(g0c.tbd, c.tbd);
    if (!c.tbd) {
      EXPECT_DOUBLE_EQ(ik.value, c.value);
      EXPECT_DOUBLE_EQ(g0c.value, c.value);
    }
  }

  // Absent: the same default from both.
  const YAML::Node absent = CatchabilityRoot();
  const rtc::catching::TbdDouble ik = ParseCatchPoseIkParams(absent).manipulability_min_arm_5row;
  const rtc::catching::TbdDouble g0c =
      rtc::catching::ParseCatchingParams(absent).planner_catchability_manip_min_arm5row;
  EXPECT_FALSE(ik.tbd);
  EXPECT_FALSE(g0c.tbd);
  EXPECT_DOUBLE_EQ(ik.value, g0c.value);

  // Neither a number nor `TBD`: refused by both, with the same exception type.
  const YAML::Node words =
      YAML::Load("planner:\n  catchability:\n    manipulability_min:\n      arm_5row: lowish\n");
  EXPECT_THROW((void)ParseCatchPoseIkParams(words), std::invalid_argument);
  EXPECT_THROW((void)rtc::catching::ParseCatchingParams(words), std::invalid_argument);
}

TEST(CatchPoseIkParams, SharedKeyOutOfRangeIsThrownHereAndReportedThere) {
  YAML::Node root = CatchabilityRoot();
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = -0.1;

  // This parser: `options` must be ready for Solve, so it refuses outright.
  ExpectRejectMentioning(root, "'planner.catchability.manipulability_min.arm_5row'");

  // The G0-C parser: the value PARSES, as written...
  rtc::catching::CatchingParams g0c;
  ASSERT_NO_THROW(g0c = rtc::catching::ParseCatchingParams(root));
  EXPECT_FALSE(g0c.planner_catchability_manip_min_arm5row.tbd);
  EXPECT_DOUBLE_EQ(g0c.planner_catchability_manip_min_arm5row.value, -0.1);

  // ...and the validator is what refuses it, as a range violation on the key.
  const rtc::catching::CatchingValidationReport report =
      rtc::catching::ValidateCatchingParams(g0c, /*control_rate_hz=*/500.0,
                                            /*real_arm_config=*/false);
  EXPECT_FALSE(report.armable);
  bool reported = false;
  for (std::size_t i = 0; i < report.failure_count; ++i) {
    const rtc::catching::CatchingValidationEntry& e = report.failures[i];
    if (e.reason == rtc::catching::CatchingValidationReason::kRangeViolation &&
        std::string_view(e.key) == "planner.catchability.manipulability_min.arm_5row") {
      reported = true;
    }
  }
  EXPECT_TRUE(reported) << "the out-of-range threshold was parsed but never reported";

  // Control: the same report for an in-range value does NOT carry that entry,
  // so the assertion above is about -0.1 and not about the rest of the config.
  root["planner"]["catchability"]["manipulability_min"]["arm_5row"] = 0.1;
  const rtc::catching::CatchingValidationReport ok =
      rtc::catching::ValidateCatchingParams(rtc::catching::ParseCatchingParams(root), 500.0, false);
  for (std::size_t i = 0; i < ok.failure_count; ++i) {
    EXPECT_FALSE(
        ok.failures[i].reason == rtc::catching::CatchingValidationReason::kRangeViolation &&
        std::string_view(ok.failures[i].key) == "planner.catchability.manipulability_min.arm_5row");
  }
}

}  // namespace
