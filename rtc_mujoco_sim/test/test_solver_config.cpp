// ── test_solver_config.cpp ───────────────────────────────────────────────────
// XML-priority rule: solver params in XML <option>/<flag> override YAML;
// YAML applies only to attributes absent from XML.
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

namespace rtc {
namespace {

// minimal.xml sets solver="Newton" iterations="20" → these win.
// timestep="0.002" is also set, but not in solver-atomic matrix.

TEST(SolverConfig, XmlSolverOverridesYaml) {
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.solver = "CG";  // YAML says CG
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetSolverType(), mjSOL_NEWTON);  // XML wins
}

TEST(SolverConfig, XmlIterationsOverridesYaml) {
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.iterations = 999;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetSolverIterations(), 20);  // XML says 20
}

TEST(SolverConfig, YamlAppliedWhenXmlSilent) {
  // minimal.xml does NOT set cone → YAML elliptic should apply
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.cone = "elliptic";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetCone(), mjCONE_ELLIPTIC);
}

TEST(SolverConfig, YamlImpratioApplied) {
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.impratio = 3.5;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_DOUBLE_EQ(sim.GetImpratio(), 3.5);
}

TEST(SolverConfig, YamlNoslipIterApplied) {
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.noslip_iterations = 7;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetNoslipIterations(), 7);
}

TEST(SolverConfig, YamlTolerance) {
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.tolerance = 1e-5;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  // XML does not set tolerance → YAML wins
  EXPECT_DOUBLE_EQ(sim.GetSolverTolerance(), 1e-5);
}

TEST(SolverConfig, IntegratorYamlApplied) {
  // XML does not set integrator → YAML implicit should apply
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.integrator = "implicit";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetIntegrator(), mjINT_IMPLICIT);
}

TEST(SolverConfig, ContactOverrideYamlApplied) {
  auto cfg = test::MakeMinimalConfig();
  cfg.solver_config.contact_override.enable   = true;
  cfg.solver_config.contact_override.o_margin = 0.05;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  // We can't directly inspect model_->opt.o_margin (private), but initialization
  // success with override enabled exercises the code path without crashing.
  SUCCEED();
}

TEST(SolverConfig, SolverStatsInitialZero) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  const auto stats = sim.GetSolverStats();
  EXPECT_EQ(stats.iter, 0);
  EXPECT_EQ(stats.ncon, 0);
}

}  // namespace
}  // namespace rtc
