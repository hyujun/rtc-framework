/// @file test_kinodynamic_alloc_gate.cpp
/// @brief Heap allocations of the kinodynamic mode path: `SetMode`, `OdeOf`
///        and the accessors allocate nothing; the library's `forward` /
///        `dForward` allocation counts are recorded and must be the same on
///        consecutive calls (library allocations are recorded, not gated).
///
/// Both counters are armed: the operator-new counter (C++ allocations) and
/// the C-level malloc gate (any library). malloc_gate.hpp may be included in
/// exactly one translation unit per binary.

#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#pragma GCC diagnostic pop

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::KinodynamicDynamics;
using tu::AllocCounter;
using Ode = KinodynamicDynamics::Ode;

/// Allocations of one call of @p fn: {operator new, C malloc family}.
template <typename Fn>
std::pair<std::int64_t, std::size_t> Count(Fn&& fn) {
  std::size_t c = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    fn();
    c = gate.count();
  }
  AllocCounter::Disarm();
  return {AllocCounter::AllocCount(), c};
}

class KinodynamicAllocGate : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(dyn_.Init(handler_), rtc::mpc::KinodynamicError::kNoError);
    oracle_.emplace(model_,
                    std::vector<pinocchio::FrameIndex>{model_.getFrameId("left_sole"),
                                                       model_.getFrameId("right_sole")},
                    6);
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<tu::KinoOracle> oracle_;
};

TEST_F(KinodynamicAllocGate, PositiveControlsAreCounted) {
  const auto [n_data, m_data] = Count([&] { const pinocchio::Data data(model_); });
  EXPECT_GT(m_data, 0U) << "the malloc gate missed pinocchio::Data";
  const std::optional<Ode> ode = dyn_.MakeOde(0b11);
  ASSERT_TRUE(ode.has_value());
  const auto [n_cd, m_cd] = Count([&] { auto d = ode->createData(); });
  EXPECT_GT(n_cd, 0) << "the operator-new counter missed createData";
  EXPECT_GT(m_cd, 0U) << "the malloc gate missed createData";
  (void)n_data;
}

TEST_F(KinodynamicAllocGate, ModeSwitchAndStageLookupAllocateNothing) {
  using Integrator = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
  using Stage = aligator::StageModelTpl<double>;
  const aligator::CostStackTpl<double> cost(*dyn_.space(), dyn_.nu());
  Stage stage(cost, Integrator(dyn_.MakeOde(0b00).value(), 0.02));
  Ode* in_stage = KinodynamicDynamics::OdeOf(stage);
  ASSERT_NE(in_stage, nullptr);

  int sink = 0;
  const auto [n, m] = Count([&] {
    for (int i = 0; i < 1000; ++i) {
      Ode* o = KinodynamicDynamics::OdeOf(stage);
      sink += dyn_.SetMode(*o, static_cast<std::uint32_t>(i % 4)) ? 1 : 0;
      sink += dyn_.SetMode(*o, 7U) ? 1 : 0;  // refused: no write
      sink += dyn_.n_contacts() + dyn_.force_size() + dyn_.nu() + static_cast<int>(dyn_.n_modes()) +
              static_cast<int>(dyn_.space()->nx());
    }
  });
  EXPECT_EQ(n, 0);
  EXPECT_EQ(m, 0U);
  EXPECT_GT(sink, 0);
}

// The library's own allocations, recorded per call; a steady count across
// consecutive calls means nothing grows with use.
TEST_F(KinodynamicAllocGate, LibraryEvaluationAllocationsAreRecordedAndSteady) {
  std::mt19937 rng(20261114U);
  const tu::KinoSample s = oracle_->Draw(rng, 0b11);
  const std::optional<Ode> ode = dyn_.MakeOde(0b11);
  ASSERT_TRUE(ode.has_value());
  auto data = ode->createData();
  ode->forward(s.x, s.u, *data);  // warm-up
  ode->dForward(s.x, s.u, *data);

  const auto f1 = Count([&] { ode->forward(s.x, s.u, *data); });
  const auto f2 = Count([&] { ode->forward(s.x, s.u, *data); });
  const auto d1 = Count([&] { ode->dForward(s.x, s.u, *data); });
  const auto d2 = Count([&] { ode->dForward(s.x, s.u, *data); });
  EXPECT_EQ(f1, f2);
  EXPECT_EQ(d1, d2);
  RecordProperty("forward_new", std::to_string(f1.first));
  RecordProperty("forward_malloc", std::to_string(f1.second));
  RecordProperty("dforward_new", std::to_string(d1.first));
  RecordProperty("dforward_malloc", std::to_string(d1.second));
  std::printf("per call: forward new %lld malloc %zu, dForward new %lld malloc %zu\n",
              static_cast<long long>(f1.first), f1.second, static_cast<long long>(d1.first),
              d1.second);
}

}  // namespace
