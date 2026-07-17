// Stage A-5a: fixed-dim QP refactor regression tests.
//
// Verifies the central property of the refactor: contact on/off transitions
// no longer change the QP variable dimension. The solver sees the same
// problem size each tick; inactive contacts' λ slots are picked as ≈ 0 by
// the solver via Hessian regularization (no task or constraint references
// those columns when the contact is inactive).

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/contact_constraint.hpp"
#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class FixedDimQpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    // 2 point contacts on panda links → max_contact_vars = 6.
    mgr_.contacts.resize(2);
    mgr_.contacts[0].name = "ee0";
    mgr_.contacts[0].frame_name = "panda_link7";
    mgr_.contacts[0].frame_id = static_cast<int>(model_->getFrameId("panda_link7"));
    mgr_.contacts[0].contact_dim = 3;
    mgr_.contacts[0].friction_coeff = 0.7;
    mgr_.contacts[0].friction_faces = 4;
    mgr_.contacts[1].name = "ee1";
    mgr_.contacts[1].frame_name = "panda_link8";
    mgr_.contacts[1].frame_id = static_cast<int>(model_->getFrameId("panda_link8"));
    mgr_.contacts[1].contact_dim = 3;
    mgr_.contacts[1].friction_coeff = 0.7;
    mgr_.contacts[1].friction_faces = 4;
    mgr_.max_contacts = 2;
    mgr_.max_contact_vars = 6;

    cache_.Init(model_, rtc::tsid::ContactFrameIds(mgr_));
    cs_.Init(2);
    cs_.SeedNormals(mgr_);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig mgr_;
  PinocchioCache cache_;
  ContactState cs_;
};

// Stage A-5a: WQPFormulation::Solve uses n_vars = nv + max_contact_vars
// regardless of activation. Toggle contacts and confirm the solver receives
// the same x_opt dimension on every tick.
TEST_F(FixedDimQpTest, NVarsConstantAcrossContactToggles) {
  YAML::Node fcfg;
  fcfg["formulation_type"] = "wqp";
  auto formulation = CreateFormulation(*model_, robot_info_, mgr_, fcfg);
  ASSERT_NE(formulation, nullptr);

  // Posture-only task is enough to exercise the solve path.
  auto posture = std::make_unique<PostureTask>();
  YAML::Node tcfg;
  tcfg["kp"] = 100.0;
  tcfg["kd"] = 20.0;
  tcfg["weight"] = 1.0;
  posture->Init(*model_, robot_info_, cache_, tcfg);
  formulation->AddTask(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);

  ControlReference ref;
  ref.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, mgr_.max_contact_vars);
  ref.q_des = q;
  ref.v_des = v;
  ref.a_des.setZero(robot_info_.nv);

  const int expected_n_vars = robot_info_.nv + mgr_.max_contact_vars;

  // Scenario 1: both contacts inactive.
  cs_.contacts[0].active = false;
  cs_.contacts[1].active = false;
  cs_.RecomputeActive(mgr_);
  cache_.Update(q, v);
  const auto& r0 = formulation->Solve(cache_, ref, cs_, robot_info_);
  ASSERT_TRUE(r0.converged);
  const Eigen::Index x_size_0 = r0.x_opt.size();

  // Scenario 2: contact 0 active.
  cs_.contacts[0].active = true;
  cs_.contacts[1].active = false;
  cs_.RecomputeActive(mgr_);
  cache_.Update(q, v);
  const auto& r1 = formulation->Solve(cache_, ref, cs_, robot_info_);
  ASSERT_TRUE(r1.converged);

  // Scenario 3: both contacts active.
  cs_.contacts[0].active = true;
  cs_.contacts[1].active = true;
  cs_.RecomputeActive(mgr_);
  cache_.Update(q, v);
  const auto& r2 = formulation->Solve(cache_, ref, cs_, robot_info_);
  ASSERT_TRUE(r2.converged);

  // x_opt size must be identical across the three scenarios.
  EXPECT_EQ(r0.x_opt.size(), x_size_0);
  EXPECT_EQ(r1.x_opt.size(), x_size_0);
  EXPECT_EQ(r2.x_opt.size(), x_size_0);
  EXPECT_GE(x_size_0, expected_n_vars);  // ≥ since formulation pre-allocates max
}

// Stage A-5a: inactive contact's λ slot picks ≈ 0 by Hessian regularization.
// No task / constraint references those columns when the contact is inactive,
// so the unique minimizer of the diagonal-regularized cost is zero.
TEST_F(FixedDimQpTest, InactiveLambdaSlotIsZero) {
  YAML::Node fcfg;
  fcfg["formulation_type"] = "wqp";
  auto formulation = CreateFormulation(*model_, robot_info_, mgr_, fcfg);
  ASSERT_NE(formulation, nullptr);

  // Posture task — purely on a, no λ coupling.
  auto posture = std::make_unique<PostureTask>();
  YAML::Node tcfg;
  tcfg["kp"] = 100.0;
  tcfg["kd"] = 20.0;
  tcfg["weight"] = 1.0;
  posture->Init(*model_, robot_info_, cache_, tcfg);
  formulation->AddTask(std::move(posture));

  // EoM constraint binds tau via λ; active contact's λ does contribute,
  // inactive does not.
  auto eom = std::make_unique<EomConstraint>();
  YAML::Node econfig;
  eom->Init(*model_, robot_info_, cache_, econfig);
  eom->SetContactManager(&mgr_);
  formulation->AddConstraint(std::move(eom));

  // FrictionCone for the active contact — inactive contact's cone rows are
  // skipped, so its λ slot is free.
  auto fc = std::make_unique<FrictionConeConstraint>();
  YAML::Node ccfg;
  fc->Init(*model_, robot_info_, cache_, ccfg);
  fc->SetContactManager(&mgr_);
  formulation->AddConstraint(std::move(fc));

  // ContactConstraint — active contact must satisfy Jc·a = -dotJc·v.
  auto cc = std::make_unique<ContactConstraint>();
  YAML::Node cccfg;
  cc->Init(*model_, robot_info_, cache_, cccfg);
  cc->SetContactManager(&mgr_);
  formulation->AddConstraint(std::move(cc));

  // Activate contact 0, leave contact 1 inactive.
  cs_.contacts[0].active = true;
  cs_.contacts[1].active = false;
  cs_.RecomputeActive(mgr_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ControlReference ref;
  ref.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, mgr_.max_contact_vars);
  ref.q_des = q;
  ref.v_des = v;
  ref.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->Solve(cache_, ref, cs_, robot_info_);
  ASSERT_TRUE(result.converged);

  // Inactive contact 1's λ slot occupies columns nv+3 .. nv+5.
  const int nv = robot_info_.nv;
  const Eigen::Vector3d lambda_inactive = result.x_opt.segment(nv + 3, 3);
  // Hessian regularization alone (1e-8) drives this to numerical zero
  // since no task/constraint touches those columns.
  EXPECT_LT(lambda_inactive.norm(), 1e-4)
      << "Inactive contact's λ slot must be ≈ 0 (got " << lambda_inactive.transpose() << ")";
}

// Stage A-5a: ContactConstraint::EqDim returns the full max_contact_vars
// regardless of activation count.
TEST_F(FixedDimQpTest, ContactConstraintEqDimIsConstant) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.Init(*model_, robot_info_, cache_, cfg);
  cc.SetContactManager(&mgr_);

  // All-inactive.
  cs_.contacts[0].active = false;
  cs_.contacts[1].active = false;
  cs_.RecomputeActive(mgr_);
  EXPECT_EQ(cc.EqDim(cs_), mgr_.max_contact_vars);

  // One active.
  cs_.contacts[0].active = true;
  cs_.RecomputeActive(mgr_);
  EXPECT_EQ(cc.EqDim(cs_), mgr_.max_contact_vars);

  // All-active.
  cs_.contacts[1].active = true;
  cs_.RecomputeActive(mgr_);
  EXPECT_EQ(cc.EqDim(cs_), mgr_.max_contact_vars);
}

}  // namespace
}  // namespace rtc::tsid
