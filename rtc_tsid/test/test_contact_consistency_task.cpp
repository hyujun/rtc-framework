#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/contact_consistency_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class ContactConsistencyTaskTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    // 1 point contact on panda_hand
    YAML::Node contact_yaml;
    YAML::Node c1;
    c1["name"] = "ee";
    c1["frame"] = "panda_hand";
    c1["type"] = "point";
    c1["friction_coeff"] = 0.5;
    contact_yaml["contacts"].push_back(c1);

    contact_cfg_.Load(contact_yaml, *model_);
    cache_.Init(model_, contact_cfg_);

    contacts_.Init(contact_cfg_.max_contacts);

    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated,
              contact_cfg_.max_contact_vars);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

// A-2 (guide §3): residual rows = J_c rows, r = -dotJ_c·v exactly; λ columns are zero.
TEST_F(ContactConsistencyTaskTest, ResidualMatchesFormula) {
  // Random non-zero q, v so dotJ_c·v is non-trivial.
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  for (int i = 0; i < q.size(); ++i)
    q(i) += 0.1 * (i + 1);
  Eigen::VectorXd v = Eigen::VectorXd::Random(robot_info_.nv);

  contacts_.contacts[0].active = true;
  contacts_.RecomputeActive(contact_cfg_);
  cache_.Update(q, v, contacts_);

  ContactConsistencyTask task;
  YAML::Node cfg;
  cfg["weight"] = 2.5;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  EXPECT_EQ(task.Weight(), 2.5);

  const int cdim = contact_cfg_.contacts[0].contact_dim;
  ASSERT_EQ(cdim, 3);
  const int n_vars = robot_info_.nv + contacts_.active_contact_vars;
  ASSERT_EQ(n_vars, robot_info_.nv + cdim);

  // Pre-zero buffers (formulation does this for us at runtime).
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(cdim, n_vars);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(cdim);
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

  // ResidualDim refreshed inside ComputeResidual.
  EXPECT_EQ(task.ResidualDim(), cdim);

  const auto& Jc = cache_.contact_frames[0].J;     // [6 × nv]
  const auto& dJv = cache_.contact_frames[0].dJv;  // [6]

  // a-block: J[:, 0:nv] == J_c[:cdim, :]
  EXPECT_LT((J.leftCols(robot_info_.nv) - Jc.topRows(cdim)).norm(), 1e-12);
  // λ-block: J[:, nv:] all zero (no force coupling).
  EXPECT_NEAR(J.rightCols(cdim).norm(), 0.0, 1e-15);
  // r == -dotJ_c · v
  EXPECT_LT((r + dJv.head(cdim)).norm(), 1e-12);

  // Quadratic-form sanity: cost = ‖J·z - r‖² with z=[a; λ] equals
  // ‖J_c·a + dotJ_c·v‖² (λ drops out, sign matches J_block - r_block).
  Eigen::VectorXd a = Eigen::VectorXd::Random(robot_info_.nv);
  Eigen::VectorXd lambda = Eigen::VectorXd::Random(cdim);
  Eigen::VectorXd z(n_vars);
  z.head(robot_info_.nv) = a;
  z.tail(cdim) = lambda;
  const Eigen::VectorXd residual = J * z - r;
  const Eigen::VectorXd direct = Jc.topRows(cdim) * a + dJv.head(cdim);
  EXPECT_LT((residual - direct).norm(), 1e-12);
}

// A-2 (guide §3): contact inactive → residual_dim=0 → no QP contribution.
TEST_F(ContactConsistencyTaskTest, InactiveContactZeroResidualDim) {
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);

  // Contact inactive (default).
  contacts_.RecomputeActive(contact_cfg_);
  EXPECT_EQ(contacts_.active_contact_vars, 0);
  cache_.Update(q, v, contacts_);

  ContactConsistencyTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  // Tiny buffer (active=0 → formulation will allocate 0 rows; we mirror that).
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, robot_info_.nv);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(1);
  task.ComputeResidual(cache_, ref_, contacts_, robot_info_.nv, J, r);

  EXPECT_EQ(task.ResidualDim(), 0);
  // Untouched buffers — no writes when contact set is empty.
  EXPECT_NEAR(J.norm(), 0.0, 1e-15);
  EXPECT_NEAR(r.norm(), 0.0, 1e-15);
}

// A-2 (guide §3): manager 미설정 시 안전 fallback — w_out 효과 0, buffer 미터치.
TEST_F(ContactConsistencyTaskTest, ManagerNotSet_NoOp) {
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Random(robot_info_.nv);

  contacts_.contacts[0].active = true;
  contacts_.RecomputeActive(contact_cfg_);
  cache_.Update(q, v, contacts_);

  ContactConsistencyTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  // *Intentionally not* calling SetContactManager.

  const int cdim = contact_cfg_.contacts[0].contact_dim;
  const int n_vars = robot_info_.nv + cdim;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(cdim, n_vars);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(cdim);

  // 호출은 안전해야 하고 buffer 는 그대로여야 한다 (early-return on !manager_).
  EXPECT_NO_THROW(task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r));
  EXPECT_NEAR(J.norm(), 0.0, 1e-15);
  EXPECT_NEAR(r.norm(), 0.0, 1e-15);
}

}  // namespace
}  // namespace rtc::tsid
