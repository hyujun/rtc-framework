#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/tasks/internal_force_task.hpp"
#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class InternalForceTaskTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(robot_info_.nv);
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

    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg_));

    contacts_.Init(static_cast<int>(contacts.size()));
    for (auto& e : contacts_.contacts) {
      e.active = true;
    }
    contacts_.RecomputeActive(contact_cfg_);
    cache_.Update(q_, v_);

    mgr_.Init(contact_cfg_, robot_info_.nv);

    // Compute G + run GraspCache once for the test scenario.
    const int n_active = mgr_.ActiveLambdaDim(contacts_);
    G_.setZero(6, n_active);
    if (n_active > 0) {
      mgr_.ComputeGraspMatrix(cache_, contacts_, object_, G_);
    }
    grasp_cache_.Init(contact_cfg_.max_contact_vars);
    grasp_cache_.Compute(G_, n_active);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ContactManager mgr_;
  GraspCache grasp_cache_;
  ObjectFrame object_;
  Eigen::MatrixXd G_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
};

// Case 1: 2 contacts at the *same* Panda frame → G has two identical 6×3
// column blocks → rank_G = 3, n_λ = 6, nullity = 3. The task must
// activate (ResidualDim == 6) and the projected reference must satisfy
// G · r = 0 (lives entirely in Null(G)).
TEST_F(InternalForceTaskTest, RankBasedNullity_DegenerateGrasp) {
  // Reuse the same Panda frame for both contacts → guaranteed collinear.
  Configure({{"panda_link5", 3}, {"panda_link5", 3}});
  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 6);
  ASSERT_EQ(grasp_cache_.Rank(), 3);

  InternalForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);
  task.SetGraspCache(&grasp_cache_);

  // Squeeze reference: opposing forces on the two contacts (will project
  // partly into Null(G), partly out).
  Eigen::VectorXd lambda_des(contact_cfg_.max_contact_vars);
  lambda_des << 1.0, 0.0, 5.0, -1.0, 0.0, 5.0;  // mixed: anti-x + symmetric-z
  task.SetSqueezeReference(lambda_des);

  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 6);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  // G · r ≈ 0 (the projected reference lives entirely in Null(G)).
  EXPECT_LT((G_ * r).norm(), 1e-6) << "projected reference leaks into Range(Gᵀ)";

  // r itself must be non-trivial (the anti-symmetric part survives).
  EXPECT_GT(r.norm(), 1e-3);

  // J has identity in the (row, λ-slot) positions: 6×6 identity in the
  // right block (both contacts active, all slots used).
  const int nv = robot_info_.nv;
  Eigen::MatrixXd J_lambda = J.rightCols(contact_cfg_.max_contact_vars);
  EXPECT_LT((J_lambda - Eigen::MatrixXd::Identity(6, 6)).norm(), 1e-15);
  EXPECT_LT(J.leftCols(nv).norm(), 1e-15);
}

// Case 2: 3 non-coplanar fingertip contacts → n_λ=9, rank_G=6, nullity=3.
// Task active, ResidualDim=9, projected reference lives in Null(G).
TEST_F(InternalForceTaskTest, FullRankFullRowGrasp_NullityThree) {
  Configure({{"panda_link3", 3}, {"panda_link5", 3}, {"panda_link7", 3}});
  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 9);
  ASSERT_EQ(grasp_cache_.Rank(), 6);

  InternalForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);
  task.SetGraspCache(&grasp_cache_);

  Eigen::VectorXd lambda_des(contact_cfg_.max_contact_vars);
  lambda_des << 0.5, 0.2, 4.0, -0.3, 0.1, 4.5, 0.0, -0.4, 3.8;
  task.SetSqueezeReference(lambda_des);

  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 9);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(9, n_vars);
  Eigen::VectorXd r(9);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  // The projected reference must lie in Null(G) up to damping tolerance.
  EXPECT_LT((G_ * r).norm(), 1e-4)
      << "‖G·r‖ = " << (G_ * r).norm() << " (Tikhonov-damped projector)";

  // Right block identity, left block zero.
  Eigen::MatrixXd J_lambda = J.rightCols(contact_cfg_.max_contact_vars);
  EXPECT_LT((J_lambda - Eigen::MatrixXd::Identity(9, 9)).norm(), 1e-15);
  EXPECT_LT(J.leftCols(robot_info_.nv).norm(), 1e-15);
}

// Case 3: Single point contact (n_λ=3, rank_G=3, nullity=0). v2 rule says
// skip via rank-based test; ResidualDim() must be 0 (not the naive "n_λ
// ≤ 6 → skip" path which would also skip here but for the wrong reason).
TEST_F(InternalForceTaskTest, SingleContact_NullityZero_Skip) {
  Configure({{"panda_link5", 3}});
  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 3);
  ASSERT_EQ(grasp_cache_.Rank(), 3);

  InternalForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);
  task.SetGraspCache(&grasp_cache_);

  Eigen::VectorXd lambda_des(contact_cfg_.max_contact_vars);
  lambda_des << 1.0, 0.0, 5.0;
  task.SetSqueezeReference(lambda_des);

  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 0) << "nullity=0 must trigger task skip";

  // ComputeResidual is a no-op when ResidualDim==0; J/r untouched.
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

// Case 4: Projection idempotency — r computed by the task must equal
// P_N · λ_des_active (active-packed), and applying P_N to r again is a
// no-op (within damping tolerance).
TEST_F(InternalForceTaskTest, ReferenceIsProjected) {
  Configure({{"panda_link5", 3}, {"panda_link5", 3}});
  ASSERT_EQ(grasp_cache_.Rank(), 3);

  InternalForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);
  task.SetGraspCache(&grasp_cache_);

  Eigen::VectorXd lambda_des(contact_cfg_.max_contact_vars);
  lambda_des << 2.0, -1.0, 3.0, 0.5, 1.5, 3.0;
  task.SetSqueezeReference(lambda_des);

  task.UpdateResidualDim(contacts_);
  ASSERT_EQ(task.ResidualDim(), 6);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  // Reference r should equal P_N · λ_des_active by direct computation.
  // Active-packing for two active contacts at slots 0..2 / 3..5 leaves
  // the order identical to lambda_des.
  const Eigen::MatrixXd P_N = grasp_cache_.ProjN();
  const Eigen::VectorXd r_ref = P_N * lambda_des;
  EXPECT_LT((r - r_ref).norm(), 1e-12) << "r != P_N · λ_des";

  // Idempotency: P_N · r ≈ r (P_N · P_N = P_N up to damping).
  const Eigen::VectorXd r_double = P_N * r;
  EXPECT_LT((r_double - r).norm(), 1e-5) << "P_N idempotency broken";
}

}  // namespace
}  // namespace rtc::tsid
