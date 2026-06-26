#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "alloc_counter.hpp"
#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/contact/object_state_provider.hpp"
#include "rtc_tsid/tasks/object_se3_task.hpp"
#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class ObjectSE3TaskTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(robot_info_.nv);

    // Default object pose at world origin, identity orientation.
    object_frame_.p_w = Eigen::Vector3d::Zero();
    object_frame_.R_w = Eigen::Matrix3d::Identity();
    provider_.SetPose(object_frame_);
    provider_.SetTwist(Eigen::Matrix<double, 6, 1>::Zero());
    provider_.SetValid(true);
  }

  void Configure(const std::vector<std::pair<std::string, int>>& contacts) {
    contact_cfg_ = ContactManagerConfig{};
    contact_cfg_.contacts.resize(contacts.size());
    int total = 0;
    for (size_t i = 0; i < contacts.size(); ++i) {
      auto& c = contact_cfg_.contacts[i];
      c.name = "c" + std::to_string(i);
      c.frame_name = contacts[i].first;
      c.frame_id = static_cast<int>(model_->getFrameId(c.frame_name));
      c.contact_dim = contacts[i].second;
      total += c.contact_dim;
    }
    contact_cfg_.max_contacts = static_cast<int>(contacts.size());
    contact_cfg_.max_contact_vars = total;

    cache_.Init(model_, contact_cfg_);

    contacts_.Init(static_cast<int>(contacts.size()));
    for (auto& e : contacts_.contacts) {
      e.active = true;
    }
    contacts_.RecomputeActive(contact_cfg_);
    cache_.Update(q_, v_, contacts_);

    mgr_.Init(contact_cfg_, robot_info_.nv);

    const int n_active = mgr_.ActiveLambdaDim(contacts_);
    G_.setZero(6, n_active);
    if (n_active > 0) {
      mgr_.ComputeGraspMatrix(cache_, contacts_, object_frame_, G_);
    }
    grasp_cache_.Init(contact_cfg_.max_contact_vars);
    grasp_cache_.Compute(G_, n_active);
  }

  void WireTask(ObjectSE3Task& task) {
    YAML::Node cfg;
    task.Init(*model_, robot_info_, cache_, cfg);
    task.SetContactManager(&contact_cfg_);
    task.SetContactManagerRuntime(&mgr_);
    task.SetGraspCache(&grasp_cache_);
    task.SetObjectStateProvider(&provider_);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ContactManager mgr_;
  GraspCache grasp_cache_;
  ObjectFrame object_frame_;
  IdentityObjectStateProvider provider_;
  Eigen::MatrixXd G_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
};

// Case 1: J is [6 × n_vars] with left nv columns equal to (Gᵀ)⁺·J_c_stack
// and right max_contact_vars columns equal to 0. Three non-coplanar
// fingertip contacts → full-rank G.
TEST_F(ObjectSE3TaskTest, DimensionalityCheck) {
  Configure({{"panda_link3", 3}, {"panda_link5", 3}, {"panda_link7", 3}});
  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 9);
  ASSERT_EQ(grasp_cache_.Rank(), 6);

  ObjectSE3Task task;
  WireTask(task);

  // Reference at current pose → zero error / zero feedforward.
  task.SetSe3Reference(pinocchio::SE3::Identity());
  Eigen::Matrix<double, 6, 1> kp = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> kd = Eigen::Matrix<double, 6, 1>::Zero();
  task.SetGains(kp, kd);

  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 6);

  const int nv = robot_info_.nv;
  const int n_vars = nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  // Build reference J_obj = (Gᵀ)⁺ · J_c_stack directly.
  Eigen::MatrixXd jc_stack(9, nv);
  jc_stack.setZero();
  mgr_.StackContactJacobians(cache_, contacts_, jc_stack);
  const Eigen::MatrixXd GTpinv_full = grasp_cache_.GTPinv();
  const Eigen::MatrixXd J_obj_ref = GTpinv_full * jc_stack;

  const auto J_left = J.leftCols(nv);
  EXPECT_LT((J_left - J_obj_ref).norm(), 1e-10) << "left nv block ≠ (Gᵀ)⁺·J_c_stack";

  const auto J_right = J.rightCols(contact_cfg_.max_contact_vars);
  EXPECT_LT(J_right.norm(), 1e-15) << "right lambda block must be zero (kinematic task)";
}

// Case 2: r = a_obj_des - (Gᵀ)⁺ · (J̇_c·v)_stack. Random v injects a
// non-zero J̇_c·v bias; we set Kp/Kd to zero and a_ff to a known constant,
// then verify r matches a_ff - (Gᵀ)⁺·dJv_stack via direct computation.
TEST_F(ObjectSE3TaskTest, ReferenceFormula) {
  Configure({{"panda_link3", 3}, {"panda_link5", 3}, {"panda_link7", 3}});

  // Inject a non-zero velocity so dJv is non-trivial.
  v_.setRandom();
  cache_.Update(q_, v_, contacts_);
  // Re-run grasp pieces because contact frames moved (oMf is the same since
  // q didn't change, but dJv changed).
  const int n_active = mgr_.ActiveLambdaDim(contacts_);
  G_.setZero(6, n_active);
  mgr_.ComputeGraspMatrix(cache_, contacts_, object_frame_, G_);
  grasp_cache_.Compute(G_, n_active);

  ObjectSE3Task task;
  WireTask(task);

  Eigen::Matrix<double, 6, 1> a_ff;
  a_ff << 0.1, -0.2, 0.3, 0.05, -0.05, 0.02;
  task.SetSe3Reference(pinocchio::SE3::Identity(), Eigen::Matrix<double, 6, 1>::Zero(), a_ff);
  task.SetGains(Eigen::Matrix<double, 6, 1>::Zero(), Eigen::Matrix<double, 6, 1>::Zero());

  task.UpdateResidualDim(contacts_);
  ASSERT_EQ(task.ResidualDim(), 6);

  const int nv = robot_info_.nv;
  const int n_vars = nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  // Direct reference computation.
  Eigen::VectorXd dJv_stack(n_active);
  dJv_stack.setZero();
  mgr_.StackContactJDotV(cache_, contacts_, dJv_stack);
  const Eigen::MatrixXd GTpinv = grasp_cache_.GTPinv();
  const Eigen::Matrix<double, 6, 1> r_ref = a_ff - GTpinv * dJv_stack;

  EXPECT_LT((r - r_ref).norm(), 1e-5) << "r ≠ a_ff - (Gᵀ)⁺·dJv_stack (damped pinv tolerance)";
}

// Case 3: With zero active contacts, ResidualDim() == 0 and ComputeResidual
// is a no-op (J/r untouched). Mirrors Object/Internal force task gates.
TEST_F(ObjectSE3TaskTest, ActiveCountGate) {
  Configure({{"panda_link5", 3}});
  // Deactivate the only contact.
  contacts_.contacts[0].active = false;
  contacts_.RecomputeActive(contact_cfg_);
  cache_.Update(q_, v_, contacts_);

  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 0);

  ObjectSE3Task task;
  WireTask(task);
  task.SetSe3Reference(pinocchio::SE3::Identity());

  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 0) << "no active contacts → task must skip";

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(1, n_vars);
  Eigen::VectorXd r(1);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);
  EXPECT_LT(J.norm(), 1e-15);
  EXPECT_LT(r.norm(), 1e-15);
}

// Case 4: Watchdog gate — provider.IsValid()==false forces ResidualDim=0
// even when contacts are fully active. Flipping back to valid restores
// the residual.
TEST_F(ObjectSE3TaskTest, ProviderWatchdog) {
  Configure({{"panda_link3", 3}, {"panda_link5", 3}, {"panda_link7", 3}});
  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 9);

  ObjectSE3Task task;
  WireTask(task);
  task.SetSe3Reference(pinocchio::SE3::Identity());

  // Stale pose: provider reports invalid.
  provider_.SetValid(false);
  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 0) << "invalid provider must skip task";

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(1, n_vars);
  Eigen::VectorXd r(1);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);
  EXPECT_LT(J.norm(), 1e-15);
  EXPECT_LT(r.norm(), 1e-15);

  // Recovery: provider valid again → ResidualDim restored.
  provider_.SetValid(true);
  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 6);
}

// ── RT zero-alloc guard for the shared-helper pose/velocity error path ───────
// Same BodyLog6 + Jlog6 helper as SE3Task, here through the object task's
// grasp-projected residual. Nonzero v, v_obj, v_des and combined
// translation+rotation error exercise the full path.
TEST_F(ObjectSE3TaskTest, ComputeIsAllocationFree) {
  Configure({{"panda_link3", 3}, {"panda_link5", 3}, {"panda_link7", 3}});
  v_.setConstant(0.1);  // nonzero velocity → dJv + velocity-error path
  cache_.Update(q_, v_, contacts_);
  const int n_active = mgr_.ActiveLambdaDim(contacts_);
  G_.setZero(6, n_active);
  mgr_.ComputeGraspMatrix(cache_, contacts_, object_frame_, G_);
  grasp_cache_.Compute(G_, n_active);

  ObjectSE3Task task;
  WireTask(task);
  provider_.SetTwist(Eigen::Matrix<double, 6, 1>::Constant(0.02));  // nonzero v_obj
  const pinocchio::SE3 des(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                           Eigen::Vector3d(0.1, -0.05, 0.2));
  task.SetSe3Reference(des, Eigen::Matrix<double, 6, 1>::Constant(0.05));
  task.SetGains(Eigen::Matrix<double, 6, 1>::Constant(50.0),
                Eigen::Matrix<double, 6, 1>::Constant(5.0));
  task.UpdateResidualDim(contacts_);

  const int nv = robot_info_.nv;
  const int n_vars = nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;

  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);  // warm-up

  test::AllocCounter::Arm();
  for (int i = 0; i < 100; ++i) {
    task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);
  }
  test::AllocCounter::Disarm();

  EXPECT_EQ(test::AllocCounter::alloc_count.load(), 0)
      << "ObjectSE3Task::ComputeResidual must be allocation-free in the RT path";
}

}  // namespace
}  // namespace rtc::tsid
