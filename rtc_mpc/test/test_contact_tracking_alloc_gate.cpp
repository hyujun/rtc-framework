/// @file test_contact_tracking_alloc_gate.cpp
/// @brief Heap allocations of the contact tracking residual on the public G1:
///        `evaluate` and `computeJacobians` make no C++ allocation, and their C
///        allocations are exactly those of the ODE calls they make (`forward`,
///        `dForward` — the library's own, recorded per call and steady), so
///        the residual's code and its kinematics add none; the setters
///        allocate nothing.
///
/// Both counters are armed: the operator-new counter (C++ allocations) and
/// the C-level malloc gate (any library). malloc_gate.hpp may be included in
/// exactly one translation unit per binary.

#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/contact_tracking_residual.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::ContactTrackingResidual;
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

class ContactTrackingAllocGate : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(dyn_.Init(handler_), rtc::mpc::KinodynamicError::kNoError);
    oracle_.emplace(model_, tu::G1SoleFrames(model_), 6);
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<tu::KinoOracle> oracle_;
};

TEST_F(ContactTrackingAllocGate, PositiveControlsAreCounted) {
  const Ode ode = dyn_.MakeOde(0b11).value();
  ContactTrackingResidual r(ode);
  ASSERT_TRUE(r.Bind(&ode));
  const auto [n, m] = Count([&] { auto d = r.createData(); });
  EXPECT_GT(n, 0) << "the operator-new counter missed createData";
  EXPECT_GT(m, 0U) << "the malloc gate missed createData";
}

// Every contact tracked, for each dynamics mode: the call allocates no C++
// memory, and its C allocations are the bound ODE's, call for call.
TEST_F(ContactTrackingAllocGate, EvaluationAllocatesOnlyWhatTheOdeDoes) {
  std::mt19937 rng(20261114U);
  for (std::uint32_t mode = 0; mode < dyn_.n_modes(); ++mode) {
    const tu::KinoSample s = oracle_->Draw(rng, mode);
    const Ode ode = dyn_.MakeOde(mode).value();
    ContactTrackingResidual r(ode);
    ASSERT_TRUE(r.Bind(&ode));
    for (int k = 0; k < r.n_contacts(); ++k) {
      ASSERT_TRUE(r.SetScale(k, 0.5));
    }
    auto d = r.createData();
    r.evaluate(s.x, s.u, *d);  // warm-up
    r.computeJacobians(s.x, s.u, *d);

    const auto e1 = Count([&] { r.evaluate(s.x, s.u, *d); });
    const auto e2 = Count([&] { r.evaluate(s.x, s.u, *d); });
    const auto j1 = Count([&] {
      r.evaluate(s.x, s.u, *d);
      r.computeJacobians(s.x, s.u, *d);
    });
    const auto j2 = Count([&] {
      r.evaluate(s.x, s.u, *d);
      r.computeJacobians(s.x, s.u, *d);
    });
    // Baseline: the same ODE calls on a data of their own.
    auto od = ode.createData();
    ode.forward(s.x, s.u, *od);
    ode.dForward(s.x, s.u, *od);
    const auto ode_f = Count([&] { ode.forward(s.x, s.u, *od); });
    const auto ode_fd = Count([&] {
      ode.forward(s.x, s.u, *od);
      ode.dForward(s.x, s.u, *od);
    });
    EXPECT_EQ(e1.second, ode_f.second) << "mode " << mode;
    EXPECT_EQ(j1.second, ode_fd.second) << "mode " << mode;
    EXPECT_EQ(e1.first, 0) << "mode " << mode;
    EXPECT_EQ(j1.first, 0) << "mode " << mode;
    EXPECT_EQ(e1, e2) << "mode " << mode;
    EXPECT_EQ(j1, j2) << "mode " << mode;
    const std::string tag = "_mode" + std::to_string(mode);
    RecordProperty("evaluate_malloc" + tag, std::to_string(e1.second));
    RecordProperty("evaluate_jacobians_malloc" + tag, std::to_string(j1.second));
    std::printf("mode %u per call: evaluate malloc %zu, evaluate + computeJacobians malloc %zu\n",
                mode, e1.second, j1.second);
  }
}

// Untracked (every scale 0): neither call evaluates the ODE, so nothing at all.
TEST_F(ContactTrackingAllocGate, UntrackedEvaluationAllocatesNothing) {
  std::mt19937 rng(7U);
  const tu::KinoSample s = oracle_->Draw(rng, 0b11);
  const Ode ode = dyn_.MakeOde(0b11).value();
  ContactTrackingResidual r(ode);
  ASSERT_TRUE(r.Bind(&ode));
  auto d = r.createData();
  const auto c = Count([&] {
    r.evaluate(s.x, s.u, *d);
    r.computeJacobians(s.x, s.u, *d);
  });
  EXPECT_EQ(c.first, 0);
  EXPECT_EQ(c.second, 0U);
}

TEST_F(ContactTrackingAllocGate, SettersAndBindingAllocateNothing) {
  const Ode ode = dyn_.MakeOde(0b01).value();
  ContactTrackingResidual r(ode);
  rtc::mpc::StageReference stage{};
  stage.mode = 0b01;
  stage.impact_proximity[1] = 0.4;
  const rtc::mpc::SurfaceFrame flat = rtc::mpc::FlatSurface(0.0);
  int sink = 0;
  const auto c = Count([&] {
    for (int i = 0; i < 1000; ++i) {
      sink += r.Bind(&ode) ? 1 : 0;
      sink += r.SetScale(i % 2, 0.25) ? 1 : 0;
      sink += r.SetTarget(i % 2, rtc::mpc::ContactTrackingTarget{}) ? 1 : 0;
      sink += r.SetPlaneNormal(Eigen::Vector3d::UnitZ()) ? 1 : 0;
      sink += r.SetFromStage(stage, flat) ? 1 : 0;
    }
  });
  EXPECT_EQ(c.first, 0);
  EXPECT_EQ(c.second, 0U);
  EXPECT_EQ(sink, 5000);
}

}  // namespace
