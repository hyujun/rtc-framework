// The QP row budget: WQP / HQP size their constraint rows from the MaxEqDim /
// MaxIneqDim of the constraints added to them, and an active constraint whose
// rows do not fit is reported (rows_dropped, kRowBudgetExceeded) instead of
// being left out of a QP that is then solved as if it were whole.
//
// The free-flyer fixture is the public 29-DoF floating G1 URDF from
// robot_descriptions (ament lookup): nv 29, 23 actuated. Its joint_limit rows
// (nv) exceed the actuated count by more than the fixed slack the budget used
// to carry, which is the case that dropped the torque limits before.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "g1_fixture.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/constraints/contact_constraint.hpp"
#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

#include <memory>
#include <string>
#include <string_view>

namespace rtc::tsid {
namespace {

ContactConfig MakeContact(const pinocchio::Model& model, const std::string& frame, int dim,
                          int faces) {
  ContactConfig c;
  c.name = frame;
  c.frame_name = frame;
  c.frame_id = static_cast<int>(model.getFrameId(frame));
  c.contact_dim = dim;
  c.friction_coeff = 0.5;
  c.friction_faces = faces;
  if (dim == 6) {
    c.patch_half_length_x = 0.12;
    c.patch_half_length_y = 0.05;
    c.torsional_friction_coeff = 0.5;
  }
  return c;
}

void Finish(ContactManagerConfig& mgr) {
  mgr.max_contacts = static_cast<int>(mgr.contacts.size());
  mgr.max_contact_vars = 0;
  for (const auto& c : mgr.contacts) {
    mgr.max_contact_vars += c.contact_dim;
  }
}

// A test constraint that reports a smaller Max*Dim than the rows it asks for.
class UnderReportedConstraint final : public ConstraintBase {
 public:
  UnderReportedConstraint(int eq, int ineq, int max_eq, int max_ineq)
      : eq_(eq), ineq_(ineq), max_eq_(max_eq), max_ineq_(max_ineq) {}

  [[nodiscard]] std::string_view Name() const noexcept override { return "under_reported"; }

  void Init(const pinocchio::Model& /*model*/, const RobotModelInfo& /*robot_info*/,
            PinocchioCache& /*cache*/, const YAML::Node& /*constraint_config*/) override {}

  [[nodiscard]] int EqDim(const ContactState& /*contacts*/) const noexcept override { return eq_; }

  [[nodiscard]] int IneqDim(const ContactState& /*contacts*/) const noexcept override {
    return ineq_;
  }

  [[nodiscard]] int MaxEqDim(const ContactManagerConfig& /*contacts*/) const noexcept override {
    return max_eq_;
  }

  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig& /*contacts*/) const noexcept override {
    return max_ineq_;
  }

  void ComputeEquality(const PinocchioCache& /*cache*/, const ContactState& /*contacts*/,
                       const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                       Eigen::Ref<Eigen::MatrixXd> A_block,
                       Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept override {
    A_block.setZero();
    for (int i = 0; i < A_block.rows() && i < A_block.cols(); ++i) {
      A_block(i, i) = 1.0;
    }
  }

  void ComputeInequality(const PinocchioCache& /*cache*/, const ContactState& /*contacts*/,
                         const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                         Eigen::Ref<Eigen::MatrixXd> C_block, Eigen::Ref<Eigen::VectorXd> l_block,
                         Eigen::Ref<Eigen::VectorXd> u_block) noexcept override {
    C_block.setZero();
    for (int i = 0; i < C_block.rows() && i < C_block.cols(); ++i) {
      C_block(i, i) = 1.0;
    }
    l_block.setConstant(-1.0);
    u_block.setConstant(1.0);
  }

 private:
  int eq_, ineq_, max_eq_, max_ineq_;
};

template <typename C>
std::unique_ptr<C> MakeConstraint(const pinocchio::Model& model, const RobotModelInfo& info,
                                  PinocchioCache& cache, const ContactManagerConfig* mgr) {
  auto c = std::make_unique<C>();
  c->Init(model, info, cache, YAML::Node{});
  if constexpr (requires { c->SetContactManager(mgr); }) {
    c->SetContactManager(mgr);
  }
  return c;
}

std::unique_ptr<PostureTask> MakePosture(const pinocchio::Model& model, const RobotModelInfo& info,
                                         PinocchioCache& cache) {
  auto posture = std::make_unique<PostureTask>();
  YAML::Node tcfg;
  tcfg["kp"] = 100.0;
  tcfg["kd"] = 20.0;
  tcfg["weight"] = 1.0;
  posture->Init(model, info, cache, tcfg);
  return posture;
}

// ── Max*Dim of each constraint = its dimension with every contact active ────

class MaxDimTest : public test::PandaTest {
 protected:
  void SetUp() override {
    test::PandaTest::SetUp();
    // Two point contacts (8 faces) and one surface contact (4 faces): rows
    // per contact 9, 9 and 4 + 1 + 6 = 11; contact vars 3 + 3 + 6.
    mgr_.contacts.push_back(MakeContact(*model_, "panda_link5", 3, 8));
    mgr_.contacts.push_back(MakeContact(*model_, "panda_link6", 3, 8));
    mgr_.contacts.push_back(MakeContact(*model_, "panda_link7", 6, 4));
    Finish(mgr_);
    cache_.Init(model_, ContactFrameIds(mgr_));
    cs_.Init(mgr_.max_contacts);
    for (auto& c : cs_.contacts) {
      c.active = true;
    }
    cs_.RecomputeActive(mgr_);
  }

  ContactManagerConfig mgr_;
  PinocchioCache cache_;
  ContactState cs_;
};

TEST_F(MaxDimTest, EachConstraintReportsItsAllActiveDimension) {
  const auto fc = MakeConstraint<FrictionConeConstraint>(*model_, robot_info_, cache_, &mgr_);
  EXPECT_EQ(fc->MaxIneqDim(mgr_), 9 + 9 + 11);
  EXPECT_EQ(fc->MaxIneqDim(mgr_), fc->IneqDim(cs_));
  EXPECT_EQ(fc->MaxEqDim(mgr_), 0);

  const auto cc = MakeConstraint<ContactConstraint>(*model_, robot_info_, cache_, &mgr_);
  EXPECT_EQ(cc->MaxEqDim(mgr_), 12);
  EXPECT_EQ(cc->MaxEqDim(mgr_), cc->EqDim(cs_));
  EXPECT_EQ(cc->MaxIneqDim(mgr_), 0);

  const auto jl = MakeConstraint<JointLimitConstraint>(*model_, robot_info_, cache_, &mgr_);
  EXPECT_EQ(jl->MaxIneqDim(mgr_), robot_info_.nv);
  EXPECT_EQ(jl->MaxEqDim(mgr_), 0);

  const auto tl = MakeConstraint<TorqueLimitConstraint>(*model_, robot_info_, cache_, &mgr_);
  EXPECT_EQ(tl->MaxIneqDim(mgr_), robot_info_.n_actuated);
  EXPECT_EQ(tl->MaxEqDim(mgr_), 0);

  // Fixed base: no unactuated rows.
  const auto eom = MakeConstraint<EomConstraint>(*model_, robot_info_, cache_, &mgr_);
  EXPECT_EQ(eom->MaxEqDim(mgr_), 0);
  EXPECT_EQ(eom->MaxIneqDim(mgr_), 0);
}

// ── Free-flyer G1: the rows the old fixed budget could not hold ─────────────

class G1RowBudgetTest : public ::testing::TestWithParam<const char*> {
 protected:
  void SetUp() override {
    model_ = test::SharedG1Model();
    robot_info_.Build(*model_, YAML::Node{});
    mgr_.contacts.push_back(MakeContact(*model_, "left_ankle_roll_link", 6, 4));
    mgr_.contacts.push_back(MakeContact(*model_, "right_ankle_roll_link", 6, 4));
    Finish(mgr_);
    cache_.Init(model_, ContactFrameIds(mgr_));
    cs_.Init(mgr_.max_contacts);
    cs_.SeedNormals(mgr_);
    for (auto& c : cs_.contacts) {
      c.active = true;
    }
    cs_.RecomputeActive(mgr_);
    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(model_->nv);
    cache_.Update(q_, v_);
    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, mgr_.max_contact_vars);
    ref_.q_des = q_;
    ref_.v_des = v_;
    ref_.a_des.setZero(robot_info_.nv);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig mgr_;
  PinocchioCache cache_;
  ContactState cs_;
  ControlReference ref_;
  Eigen::VectorXd q_, v_;
};

TEST_P(G1RowBudgetTest, EveryRowOfEveryConstraintIsInTheQp) {
  ASSERT_EQ(model_->nv, 29);
  ASSERT_TRUE(robot_info_.floating_base);
  ASSERT_EQ(robot_info_.n_actuated, 23);

  YAML::Node fcfg;
  fcfg["formulation_type"] = GetParam();
  auto f = CreateFormulation(*model_, robot_info_, mgr_, fcfg);
  ASSERT_NE(f, nullptr);
  f->AddTask(MakePosture(*model_, robot_info_, cache_));
  // The shipped registration order: eom → joint_limit → friction → torque.
  f->AddConstraint(MakeConstraint<EomConstraint>(*model_, robot_info_, cache_, &mgr_));
  f->AddConstraint(MakeConstraint<ContactConstraint>(*model_, robot_info_, cache_, &mgr_));
  f->AddConstraint(MakeConstraint<JointLimitConstraint>(*model_, robot_info_, cache_, &mgr_));
  f->AddConstraint(MakeConstraint<FrictionConeConstraint>(*model_, robot_info_, cache_, &mgr_));
  f->AddConstraint(MakeConstraint<TorqueLimitConstraint>(*model_, robot_info_, cache_, &mgr_));

  const auto& r = f->Solve(cache_, ref_, cs_, robot_info_);
  EXPECT_EQ(r.rows_dropped, 0);
  EXPECT_NE(r.status, kRowBudgetExceeded);
  EXPECT_EQ(r.n_eq_used, 6 + 12);              // unactuated + 2 surface contacts
  EXPECT_EQ(r.n_ineq_used, 23 + 2 * 11 + 23);  // joint_limit (actuated only) + friction + torque
}

INSTANTIATE_TEST_SUITE_P(Formulations, G1RowBudgetTest, ::testing::Values("wqp", "hqp"));

// ── A constraint asking for more rows than it reported is not solved around ─

class UnderReportTest : public test::PandaNoContactTest,
                        public ::testing::WithParamInterface<const char*> {
 protected:
  void SetUp() override {
    test::PandaNoContactTest::SetUp();
    const Eigen::VectorXd q = pinocchio::neutral(*model_);
    cache_.Update(q, Eigen::VectorXd::Zero(model_->nv));
    ref_.q_des = q;
  }
};

TEST_P(UnderReportTest, DroppedRowsAreReportedAndNothingIsSolved) {
  YAML::Node fcfg;
  fcfg["formulation_type"] = GetParam();
  auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, fcfg);
  ASSERT_NE(f, nullptr);
  f->AddTask(MakePosture(*model_, robot_info_, cache_));
  // The budget is shared: the torque limits, added first, take their rows;
  // a constraint that reports 2 eq / 3 ineq but asks for 4 / 5 then fits in
  // neither block.
  f->AddConstraint(
      MakeConstraint<TorqueLimitConstraint>(*model_, robot_info_, cache_, &contact_cfg_));
  f->AddConstraint(std::make_unique<UnderReportedConstraint>(4, 5, 2, 3));

  const auto& r = f->Solve(cache_, ref_, contacts_, robot_info_);
  EXPECT_EQ(r.rows_dropped, 4 + 5);
  EXPECT_EQ(r.status, kRowBudgetExceeded);
  EXPECT_FALSE(r.converged);
  EXPECT_EQ(r.iterations, 0);
  EXPECT_EQ(r.levels_solved, 0);
  EXPECT_EQ(r.n_eq_used, 0);
  EXPECT_EQ(r.n_ineq_used, robot_info_.n_actuated);
}

// Control: the same constraint reporting its true size solves.
TEST_P(UnderReportTest, TheSameRowsReportedInFullSolve) {
  YAML::Node fcfg;
  fcfg["formulation_type"] = GetParam();
  auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, fcfg);
  ASSERT_NE(f, nullptr);
  f->AddTask(MakePosture(*model_, robot_info_, cache_));
  f->AddConstraint(std::make_unique<UnderReportedConstraint>(4, 5, 4, 5));
  f->AddConstraint(
      MakeConstraint<TorqueLimitConstraint>(*model_, robot_info_, cache_, &contact_cfg_));

  const auto& r = f->Solve(cache_, ref_, contacts_, robot_info_);
  EXPECT_EQ(r.rows_dropped, 0);
  EXPECT_TRUE(r.converged) << "status " << r.status;
  EXPECT_EQ(r.n_eq_used, 4);
  EXPECT_EQ(r.n_ineq_used, 5 + robot_info_.n_actuated);
}

INSTANTIATE_TEST_SUITE_P(Formulations, UnderReportTest, ::testing::Values("wqp", "hqp"));

// ── The solver settings survive the re-size AddConstraint does ──────────────

class SolverConfigReplayTest : public UnderReportTest {
 protected:
  // Inner (Newton) iterations of one solve of a posture task pulled away from
  // the current pose under the torque and joint limits.
  int InnerIterations(const YAML::Node& fcfg) {
    auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, fcfg);
    f->AddTask(MakePosture(*model_, robot_info_, cache_));
    f->AddConstraint(MakeConstraint<JointLimitConstraint>(*model_, robot_info_, cache_, nullptr));
    f->AddConstraint(
        MakeConstraint<TorqueLimitConstraint>(*model_, robot_info_, cache_, &contact_cfg_));
    ref_.q_des = pinocchio::neutral(*model_);
    ref_.q_des.array() += 0.5;
    return f->Solve(cache_, ref_, contacts_, robot_info_).iterations;
  }
};

TEST_P(SolverConfigReplayTest, IterationCapsFromYamlStillBind) {
  const std::string type = GetParam();
  const std::string block = type == "wqp" ? "solver" : "solver_per_level";
  YAML::Node capped;
  capped["formulation_type"] = type;
  capped[type][block]["max_iter"] = 1;
  capped[type][block]["max_iter_in"] = 1;
  YAML::Node free;
  free["formulation_type"] = type;

  // ProxQP counts the inner step that tests convergence after the cap, so a
  // 1 × 1 cap reports up to 2; a cap lost in the re-size would solve to the
  // default caps (20 / 10 × 100) and match the uncapped count.
  const int uncapped = InnerIterations(free);
  const int with_caps = InnerIterations(capped);
  RecordProperty("uncapped_inner_iterations", std::to_string(uncapped));
  ASSERT_GT(uncapped, 2) << "control: the problem must need more inner iterations than the cap";
  EXPECT_LE(with_caps, 2);
  EXPECT_LT(with_caps, uncapped);
}

INSTANTIATE_TEST_SUITE_P(Formulations, SolverConfigReplayTest, ::testing::Values("wqp", "hqp"));

}  // namespace
}  // namespace rtc::tsid
