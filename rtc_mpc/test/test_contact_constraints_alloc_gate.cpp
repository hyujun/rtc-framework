/// @file test_contact_constraints_alloc_gate.cpp
/// @brief Heap allocations of the contact constraints and the barrier costs
///        on the public G1: the wrench cone residual, its barrier cost (every
///        mode), the joint-limit and the sphere-pair barriers (active)
///        allocate nothing at all; the
///        contact equality makes no C++ allocation and its C allocations are
///        exactly those of the ODE calls it makes (`forward`, `dForward` —
///        the library's own, measured on the same ODE and steady call to
///        call); `SetFromStage` allocates nothing.
///
/// Both counters are armed: the operator-new counter (C++ allocations) and
/// the C-level malloc gate (any library). malloc_gate.hpp may be included in
/// exactly one translation unit per binary.

#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/barrier_costs.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
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
#include <pinocchio/algorithm/frames.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
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
using rtc::mpc::CollisionBarrierCost;
using rtc::mpc::ContactConeBarrierCost;
using rtc::mpc::ContactConstraintResidual;
using rtc::mpc::ContactRectangle;
using rtc::mpc::ContactWrenchConeResidual;
using rtc::mpc::FootConstraintGains;
using rtc::mpc::FrictionCone;
using rtc::mpc::JointLimitBarrierCost;
using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::PolynomialBarrier;
using rtc::mpc::RelaxedLogBarrier;
using rtc::mpc::SpherePair;
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

FootConstraintGains Gains() {
  FootConstraintGains g;
  g.position_z = 100.0;
  g.orientation = 80.0;
  g.linear_velocity_xy = 20.0;
  g.linear_velocity_z = 10.0;
  g.angular_velocity = 20.0;
  return g;
}

FrictionCone Friction() {
  FrictionCone f;
  f.mu = 0.4;
  f.regularization = 25.0;
  return f;
}

ContactRectangle Rectangle() {
  ContactRectangle r;
  r.x_min = -0.09;
  r.x_max = 0.09;
  r.y_min = -0.03;
  r.y_max = 0.03;
  return r;
}

class ContactConstraintsAllocGate : public ::testing::Test {
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

TEST_F(ContactConstraintsAllocGate, PositiveControlsAreCounted) {
  const Ode ode = dyn_.MakeOde(0b11).value();
  ContactWrenchConeResidual r(ode, Friction(), Rectangle());
  ASSERT_TRUE(r.Bind(&ode));
  const auto [n, m] = Count([&] { auto d = r.createData(); });
  EXPECT_GT(n, 0) << "the operator-new counter missed createData";
  EXPECT_GT(m, 0U) << "the malloc gate missed createData";
}

// For each mode: the inequality allocates nothing; the equality no C++
// memory, and C memory call for call with the bound ODE.
TEST_F(ContactConstraintsAllocGate, ConstraintEvaluationAllocatesOnlyWhatTheOdeDoes) {
  std::mt19937 rng(20261114U);
  for (std::uint32_t mode = 0; mode < dyn_.n_modes(); ++mode) {
    const tu::KinoSample s = oracle_->Draw(rng, mode);
    const Ode ode = dyn_.MakeOde(mode).value();
    ContactConstraintResidual eq(ode, Gains());
    ContactWrenchConeResidual ineq(ode, Friction(), Rectangle());
    ASSERT_TRUE(eq.Bind(&ode));
    ASSERT_TRUE(ineq.Bind(&ode));
    auto de = eq.createData();
    auto di = ineq.createData();
    eq.evaluate(s.x, s.u, *de);  // warm-up
    eq.computeJacobians(s.x, s.u, *de);
    ineq.evaluate(s.x, s.u, *di);
    ineq.computeJacobians(s.x, s.u, *di);

    const auto i1 = Count([&] { ineq.evaluate(s.x, s.u, *di); });
    const auto ij = Count([&] {
      ineq.evaluate(s.x, s.u, *di);
      ineq.computeJacobians(s.x, s.u, *di);
    });
    EXPECT_EQ(i1.first, 0) << "mode " << mode;
    EXPECT_EQ(i1.second, 0U) << "mode " << mode;
    EXPECT_EQ(ij.first, 0) << "mode " << mode;
    EXPECT_EQ(ij.second, 0U) << "mode " << mode;

    const auto e1 = Count([&] { eq.evaluate(s.x, s.u, *de); });
    const auto e2 = Count([&] { eq.evaluate(s.x, s.u, *de); });
    const auto j1 = Count([&] {
      eq.evaluate(s.x, s.u, *de);
      eq.computeJacobians(s.x, s.u, *de);
    });
    const auto j2 = Count([&] {
      eq.evaluate(s.x, s.u, *de);
      eq.computeJacobians(s.x, s.u, *de);
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
    EXPECT_EQ(e1.first, 0) << "mode " << mode;
    EXPECT_EQ(j1.first, 0) << "mode " << mode;
    EXPECT_EQ(e1.second, ode_f.second) << "mode " << mode;
    EXPECT_EQ(j1.second, ode_fd.second) << "mode " << mode;
    EXPECT_EQ(e1, e2) << "mode " << mode;
    EXPECT_EQ(j1, j2) << "mode " << mode;
    const std::string tag = "_mode" + std::to_string(mode);
    RecordProperty("equality_evaluate_malloc" + tag, std::to_string(e1.second));
    RecordProperty("equality_evaluate_jacobians_malloc" + tag, std::to_string(j1.second));
    std::printf(
        "mode %u per call: equality evaluate malloc %zu (ODE forward %zu), evaluate + "
        "computeJacobians malloc %zu (ODE forward + dForward %zu); inequality %zu / %zu\n",
        mode, e1.second, ode_f.second, j1.second, ode_fd.second, i1.second, ij.second);
  }
}

TEST_F(ContactConstraintsAllocGate, SetFromStageAllocatesNothing) {
  const Ode ode = dyn_.MakeOde(0b01).value();
  ContactConstraintResidual eq(ode, Gains());
  rtc::mpc::StageReference stage{};
  stage.contact_pose[2] = 0.1;
  stage.contact_twist[6 + 2] = -0.2;
  const rtc::mpc::SurfaceFrame flat = rtc::mpc::FlatSurface(0.0);
  int sink = 0;
  const auto c = Count([&] {
    for (int i = 0; i < 1000; ++i) {
      sink += eq.Bind(&ode) ? 1 : 0;
      sink += eq.SetFromStage(stage, flat) ? 1 : 0;
      sink += ContactConstraintResidual::AcceptsStage(stage, flat, eq.n_contacts()) ? 1 : 0;
    }
  });
  EXPECT_EQ(c.first, 0);
  EXPECT_EQ(c.second, 0U);
  EXPECT_EQ(sink, 3000);
}

TEST_F(ContactConstraintsAllocGate, BarrierCostsAllocateNothing) {
  const auto& space = *dyn_.space();
  std::mt19937 rng(7U);
  const tu::KinoSample s = oracle_->Draw(rng, 0b01);

  // Joint limits, with every limited joint inside the band.
  const JointLimitBarrierCost jl(space, dyn_.nu(), model_, PolynomialBarrier{1200.0, 0.1});
  Eigen::VectorXd x = s.x;
  for (int j = 1; j < model_.njoints; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    if (model_.nqs[ju] == 1 && model_.nvs[ju] == 1) {
      x[model_.idx_qs[ju]] = model_.lowerPositionLimit[model_.idx_qs[ju]] + 0.05;
    }
  }
  auto dj = jl.createData();
  jl.evaluate(x, s.u, *dj);
  jl.computeGradients(x, s.u, *dj);
  jl.computeHessians(x, s.u, *dj);
  ASSERT_GT(dj->value_, 0.0);
  const auto cj = Count([&] {
    jl.evaluate(x, s.u, *dj);
    jl.computeGradients(x, s.u, *dj);
    jl.computeHessians(x, s.u, *dj);
  });
  EXPECT_EQ(cj.first, 0);
  EXPECT_EQ(cj.second, 0U);

  // Sphere pairs in a swing mode, radii that put each pair in the band.
  const Ode ode = dyn_.MakeOde(0b01).value();
  pinocchio::Data pd(model_);
  pinocchio::framesForwardKinematics(model_, pd, s.x.head(model_.nq));
  std::vector<SpherePair> pairs;
  const std::vector<std::pair<const char*, const char*>> names{
      {"left_sole", "right_sole"}, {"left_knee_link", "right_knee_link"}};
  for (const auto& [a, b] : names) {
    SpherePair p;
    p.frame_a = model_.getFrameId(a);
    p.frame_b = model_.getFrameId(b);
    p.offset_a = Eigen::Vector3d(0.02, 0.0, 0.01);
    p.offset_b = Eigen::Vector3d(-0.01, 0.02, 0.0);
    const double dist =
        (pd.oMf[p.frame_a].act(p.offset_a) - pd.oMf[p.frame_b].act(p.offset_b)).norm();
    p.radius_a = std::max(0.0, 0.5 * (dist - 0.01));
    p.radius_b = p.radius_a;
    pairs.push_back(p);
  }
  CollisionBarrierCost cb(space, ode, pairs, PolynomialBarrier{1200.0, 0.04});
  ASSERT_TRUE(cb.Bind(&ode));
  auto dc = cb.createData();
  cb.evaluate(s.x, s.u, *dc);
  cb.computeGradients(s.x, s.u, *dc);
  cb.computeHessians(s.x, s.u, *dc);
  ASSERT_GT(dc->value_, 0.0);
  ASSERT_FALSE(dc->Lxx_.isZero());
  const auto cc = Count([&] {
    cb.evaluate(s.x, s.u, *dc);
    cb.computeGradients(s.x, s.u, *dc);
    cb.computeHessians(s.x, s.u, *dc);
  });
  EXPECT_EQ(cc.first, 0);
  EXPECT_EQ(cc.second, 0U);
  std::printf(
      "barriers per call: joint limit new %lld malloc %zu; sphere pairs new %lld malloc %zu\n",
      static_cast<long long>(cj.first), cj.second, static_cast<long long>(cc.first), cc.second);
}

// The wrench cone barrier in every mode (both contacts' rows, one, none).
TEST_F(ContactConstraintsAllocGate, ContactConeBarrierAllocatesNothing) {
  const auto& space = *dyn_.space();
  std::mt19937 rng(20261114U);
  for (std::uint32_t mode = 0; mode < dyn_.n_modes(); ++mode) {
    const tu::KinoSample s = oracle_->Draw(rng, mode);
    const Ode ode = dyn_.MakeOde(mode).value();
    const ContactWrenchConeResidual res(ode, Friction(), Rectangle());
    ContactConeBarrierCost cost(space, res, RelaxedLogBarrier{0.2, 5.0},
                                RelaxedLogBarrier{0.6, 0.03});
    ASSERT_TRUE(cost.Bind(&ode));
    auto d = cost.createData();
    cost.evaluate(s.x, s.u, *d);
    cost.computeGradients(s.x, s.u, *d);
    cost.computeHessians(s.x, s.u, *d);
    ASSERT_EQ(d->value_ != 0.0, mode != 0U) << "mode " << mode;
    const auto c = Count([&] {
      cost.evaluate(s.x, s.u, *d);
      cost.computeGradients(s.x, s.u, *d);
      cost.computeHessians(s.x, s.u, *d);
    });
    EXPECT_EQ(c.first, 0) << "mode " << mode;
    EXPECT_EQ(c.second, 0U) << "mode " << mode;
    std::printf("contact cone barrier mode %u per call: new %lld malloc %zu\n", mode,
                static_cast<long long>(c.first), c.second);
  }
}

}  // namespace
