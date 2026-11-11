// RT-1 for TSIDController::Compute on a floating base: everything our code
// does on the tick — the formulation's assembly and the τ recovery — makes no
// heap allocation. ProxQP's own allocations inside the solve are a recorded,
// accepted violation (invariants.md §RT Path, known violation 4): counted and
// recorded here, not asserted to be zero.
//
// The gate is rtc_base's C-level malloc gate (it defines malloc in this
// executable, so it sees allocations made inside librtc_tsid, pinocchio and
// ProxQP too) — hence a binary of its own.
//
//   * assembly: a controller whose last constraint asks for one row more than
//     it reported stops after assembling everything else (kRowBudgetExceeded,
//     no solve) — that whole tick must allocate nothing;
//   * τ recovery: two identical controllers, run identically, then one
//     measures its formulation's Solve alone and the other a full Compute —
//     the same count means the recovery around the solve adds nothing.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include "g1_fixture.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_tsid/constraints/contact_constraint.hpp"
#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

#include <memory>
#include <string>
#include <string_view>

namespace rtc::tsid {
namespace {

// Asks for one inequality row more than it reports (the row-budget trip).
class OneRowTooMany final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "one_row_too_many"; }

  void Init(const pinocchio::Model& /*model*/, const RobotModelInfo& /*robot_info*/,
            PinocchioCache& /*cache*/, const YAML::Node& /*constraint_config*/) override {}

  [[nodiscard]] int EqDim(const ContactState& /*contacts*/) const noexcept override { return 0; }

  [[nodiscard]] int IneqDim(const ContactState& /*contacts*/) const noexcept override { return 1; }

  [[nodiscard]] int MaxEqDim(const ContactManagerConfig& /*contacts*/) const noexcept override {
    return 0;
  }

  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig& /*contacts*/) const noexcept override {
    return 0;
  }

  void ComputeEquality(const PinocchioCache& /*cache*/, const ContactState& /*contacts*/,
                       const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                       Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
                       Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept override {}

  void ComputeInequality(const PinocchioCache& /*cache*/, const ContactState& /*contacts*/,
                         const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                         Eigen::Ref<Eigen::MatrixXd> /*C_block*/,
                         Eigen::Ref<Eigen::VectorXd> /*l_block*/,
                         Eigen::Ref<Eigen::VectorXd> /*u_block*/) noexcept override {}
};

YAML::Node ControllerYaml() {
  YAML::Node cfg;
  cfg["formulation_type"] = "wqp";
  for (const char* foot : {test::kG1LeftFoot, test::kG1RightFoot}) {
    YAML::Node c;
    c["name"] = foot;
    c["frame"] = foot;
    c["type"] = "surface";
    c["mu"] = 0.7;
    c["n_faces"] = 4;
    c["patch_half_length_x"] = 0.12;
    c["patch_half_length_y"] = 0.05;
    c["torsional_friction_coeff"] = 0.5;
    cfg["contacts"].push_back(c);
  }
  return cfg;
}

// A G1 TSID stack on both feet: posture + eom, contact, friction, torque and
// joint limits; the state, cache and reference it is run with.
struct Rig {
  explicit Rig(bool trip_row_budget) {
    model = test::SharedG1Model();
    info.Build(*model, YAML::Node{});
    const YAML::Node cfg = ControllerYaml();
    mgr.Load(cfg, *model);
    ctrl.Init(*model, info, cfg);

    auto& f = ctrl.Formulation();
    auto posture = std::make_unique<PostureTask>();
    YAML::Node pcfg;
    pcfg["kp"] = 100.0;
    pcfg["kd"] = 20.0;
    posture->Init(*model, info, cache, pcfg);
    f.AddTask(std::move(posture));
    auto eom = std::make_unique<EomConstraint>();
    eom->Init(*model, info, cache, YAML::Node{});
    eom->SetContactManager(&mgr);
    f.AddConstraint(std::move(eom));
    auto cc = std::make_unique<ContactConstraint>();
    cc->Init(*model, info, cache, YAML::Node{});
    cc->SetContactManager(&mgr);
    f.AddConstraint(std::move(cc));
    auto fc = std::make_unique<FrictionConeConstraint>();
    fc->Init(*model, info, cache, YAML::Node{});
    fc->SetContactManager(&mgr);
    f.AddConstraint(std::move(fc));
    auto tl = std::make_unique<TorqueLimitConstraint>();
    tl->Init(*model, info, cache, YAML::Node{});
    tl->SetContactManager(&mgr);
    f.AddConstraint(std::move(tl));
    auto jl = std::make_unique<JointLimitConstraint>();
    jl->Init(*model, info, cache, YAML::Node{});
    f.AddConstraint(std::move(jl));
    if (trip_row_budget) {
      f.AddConstraint(std::make_unique<OneRowTooMany>());
    }

    cache.Init(model, ContactFrameIds(mgr));
    contacts.Init(mgr.max_contacts);
    contacts.SeedNormals(mgr);
    for (auto& c : contacts.contacts) {
      c.active = true;
    }
    contacts.RecomputeActive(mgr);

    state.q = pinocchio::neutral(*model);
    state.q(2) = 0.8;
    state.v = Eigen::VectorXd::Zero(model->nv);
    state.v(6 + 3) = 0.2;
    cache.Update(state.q, state.v);
    ref.Init(info.nq, info.nv, info.n_actuated, mgr.max_contact_vars);
    ref.q_des = state.q;
    ref.q_des.tail(info.n_actuated).array() += 0.05;
  }

  std::shared_ptr<const pinocchio::Model> model;
  RobotModelInfo info;
  ContactManagerConfig mgr;
  TSIDController ctrl;
  PinocchioCache cache;
  ContactState contacts;
  ControlState state;
  ControlReference ref;
};

// Contract item 4 of the gate: it sees an allocation made inside a library.
TEST(TsidComputeMallocTest, TheGateSeesAllocationsMadeInsideALibrary) {
  const auto model = test::SharedG1Model();
  std::size_t count = 0;
  {
    rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data data(*model);
    count = gate.count();
  }
  EXPECT_GT(count, 0U);
}

TEST(TsidComputeMallocTest, AssemblingTheQpAllocatesNothing) {
  Rig rig(/*trip_row_budget=*/true);
  for (int i = 0; i < 3; ++i) {
    (void)rig.ctrl.Compute(rig.state, rig.ref, rig.cache, rig.contacts);
  }
  std::size_t count = 0;
  int status = 0;
  {
    rtc::testing::ScopedMallocGate gate;
    status = rig.ctrl.Compute(rig.state, rig.ref, rig.cache, rig.contacts).qp_status;
    count = gate.count();
  }
  EXPECT_EQ(status, kRowBudgetExceeded) << "premise: every row assembled, then no solve";
  EXPECT_EQ(count, 0U);
}

TEST(TsidComputeMallocTest, TorqueRecoveryAroundTheSolveAllocatesNothing) {
  Rig a(/*trip_row_budget=*/false);
  Rig b(/*trip_row_budget=*/false);
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(a.ctrl.Compute(a.state, a.ref, a.cache, a.contacts).qp_converged);
    ASSERT_TRUE(b.ctrl.Compute(b.state, b.ref, b.cache, b.contacts).qp_converged);
  }

  std::size_t solve_only = 0;
  {
    rtc::testing::ScopedMallocGate gate;
    (void)a.ctrl.Formulation().Solve(a.cache, a.ref, a.contacts, a.info);
    solve_only = gate.count();
  }
  std::size_t compute = 0;
  bool converged = false;
  {
    rtc::testing::ScopedMallocGate gate;
    converged = b.ctrl.Compute(b.state, b.ref, b.cache, b.contacts).qp_converged;
    compute = gate.count();
  }
  ::testing::Test::RecordProperty("proxqp_allocations_per_solve", std::to_string(solve_only));
  EXPECT_TRUE(converged);
  EXPECT_EQ(compute, solve_only) << "the τ recovery around the solve allocated";
}

}  // namespace
}  // namespace rtc::tsid
