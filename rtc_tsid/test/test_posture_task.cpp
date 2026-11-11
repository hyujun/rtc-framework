#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "g1_fixture.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace rtc::tsid {
namespace {

class PostureTaskTest : public test::PandaNoContactTest {};

TEST_F(PostureTaskTest, InitAndDimension) {
  PostureTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.ResidualDim(), robot_info_.nv);
  EXPECT_EQ(task.Name(), "posture");
  EXPECT_TRUE(task.IsActive());
  EXPECT_DOUBLE_EQ(task.Weight(), 1.0);
  EXPECT_EQ(task.Priority(), 0);
}

TEST_F(PostureTaskTest, ComputeResidualAtRest) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 10.0;
  cfg["kd"] = 1.0;
  task.Init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ref_.q_des = q;  // same as current → error = 0
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

  // J should be identity in the a columns
  EXPECT_TRUE(J.isApprox(Eigen::MatrixXd::Identity(n_vars, n_vars)));

  // r should be zero (q_des == q, v_des == v, a_ff == 0)
  EXPECT_LT(r.norm(), 1e-10);
}

TEST_F(PostureTaskTest, ComputeResidualWithError) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 10.0;
  cfg["kd"] = 2.0;
  task.Init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // Set desired = current + offset
  ref_.q_des = q;
  ref_.q_des.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.1);
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

  // r = Kp * (q_des - q) + Kd * (0 - 0) + 0 = 10 * 0.1 = 1.0
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_NEAR(r(i), 1.0, 1e-10);
  }
}

TEST_F(PostureTaskTest, LocalReference) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 5.0;
  cfg["kd"] = 1.0;
  task.Init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // Set local reference (overrides ControlReference)
  Eigen::VectorXd q_des = q;
  q_des.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.2);
  ASSERT_TRUE(task.SetReference(q_des, v, Eigen::VectorXd::Zero(robot_info_.nv)));

  // ControlReference with different values (should be ignored)
  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

  // r = 5.0 * 0.2 = 1.0 (using local ref, not ControlReference)
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_NEAR(r(i), 1.0, 1e-10);
  }
}

TEST_F(PostureTaskTest, WeightAndPriority) {
  PostureTask task;
  YAML::Node cfg;
  cfg["weight"] = 2.5;
  cfg["priority"] = 3;
  task.Init(*model_, robot_info_, cache_, cfg);

  EXPECT_DOUBLE_EQ(task.Weight(), 2.5);
  EXPECT_EQ(task.Priority(), 3);

  task.SetWeight(0.5);
  task.SetPriority(1);
  EXPECT_DOUBLE_EQ(task.Weight(), 0.5);
  EXPECT_EQ(task.Priority(), 1);
}

// ── Floating base (G1 free-flyer, 23 actuated) ───────────────────────────────

class FloatingPostureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = test::SharedG1Model();
    info_.Build(*model_, YAML::Node{});
    cache_.Init(model_, {});
    contacts_.Init(0);
    // ControlReference::Init leaves q_des all zero — a zero base quaternion.
    ref_.Init(info_.nq, info_.nv, info_.n_actuated, 0);
    std::srand(7);
    q_ = pinocchio::randomConfiguration(*model_, -Eigen::VectorXd::Ones(model_->nq),
                                        Eigen::VectorXd::Ones(model_->nq));
    v_ = Eigen::VectorXd::Random(model_->nv);
    ref_.q_des = pinocchio::neutral(*model_);
    ref_.q_des.tail(info_.n_actuated).setConstant(0.2);
    ref_.v_des = Eigen::VectorXd::Random(model_->nv);
    ref_.a_des = Eigen::VectorXd::Random(model_->nv);
    cache_.Update(q_, v_);
  }

  Eigen::VectorXd Residual(PostureTask& task) {
    const int nv = info_.nv;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(nv, nv);
    Eigen::VectorXd r = Eigen::VectorXd::Zero(nv);
    task.ComputeResidual(cache_, ref_, contacts_, nv, J, r);
    EXPECT_TRUE(J.isIdentity());
    return r;
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo info_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
  Eigen::VectorXd q_, v_;
};

// Actuated rows: Kp·(q_des.tail − q.tail) + Kd·(v_des − v) + a_ff. Base rows:
// no Kp term, default Kd 0 → a_ff alone.
TEST_F(FloatingPostureTest, ActuatedErrorIsTheJointDifferenceAndTheBaseHasNoGain) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 100.0;  // scalar: actuated entries only
  cfg["kd"] = 20.0;
  task.Init(*model_, info_, cache_, cfg);
  const Eigen::VectorXd r = Residual(task);

  const int na = info_.n_actuated;
  const Eigen::VectorXd expected_act = 100.0 * (ref_.q_des.tail(na) - q_.tail(na)) +
                                       20.0 * (ref_.v_des.tail(na) - v_.tail(na)) +
                                       ref_.a_des.tail(na);
  EXPECT_LT((r.tail(na) - expected_act).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((r.head(6) - ref_.a_des.head(6)).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_TRUE(r.allFinite()) << "a zero reference quaternion must not reach the residual";

  // Control: the actuated position error is actually there.
  EXPECT_GT(
      (r.tail(na) - (20.0 * (ref_.v_des.tail(na) - v_.tail(na)) + ref_.a_des.tail(na))).norm(),
      1.0);
}

// Moving the base (pose and quaternion) leaves the residual unchanged.
TEST_F(FloatingPostureTest, BasePoseDoesNotEnterTheResidual) {
  PostureTask task;
  task.Init(*model_, info_, cache_, YAML::Node{});
  const Eigen::VectorXd r0 = Residual(task);
  q_.head(3) += Eigen::Vector3d(0.3, -0.2, 0.1);
  q_.segment(3, 4) = Eigen::Quaterniond(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY())).coeffs();
  cache_.Update(q_, v_);
  EXPECT_EQ(Residual(task), r0);
}

// An nv-long list sets the base Kd too.
TEST_F(FloatingPostureTest, ListGainsReachTheBaseKd) {
  PostureTask task;
  YAML::Node cfg;
  for (int i = 0; i < info_.nv; ++i) {
    cfg["kd"].push_back(3.0);
  }
  task.Init(*model_, info_, cache_, cfg);
  const Eigen::VectorXd r = Residual(task);
  const Eigen::VectorXd expected_base =
      3.0 * (ref_.v_des.head(6) - v_.head(6)) + ref_.a_des.head(6);
  EXPECT_LT((r.head(6) - expected_base).cwiseAbs().maxCoeff(), 1e-12);
}

// An na-long list sets the actuated rows only; any other length throws.
TEST_F(FloatingPostureTest, AnActuatedLengthListSkipsTheBaseRows) {
  PostureTask task;
  YAML::Node cfg;
  for (int i = 0; i < info_.n_actuated; ++i) {
    cfg["kd"].push_back(3.0);
  }
  task.Init(*model_, info_, cache_, cfg);
  const Eigen::VectorXd r = Residual(task);
  const int na = info_.n_actuated;
  EXPECT_LT((r.head(6) - ref_.a_des.head(6)).cwiseAbs().maxCoeff(), 1e-15) << "base Kd stays 0";
  const Eigen::VectorXd expected_act = 10.0 * (ref_.q_des.tail(na) - q_.tail(na)) +
                                       3.0 * (ref_.v_des.tail(na) - v_.tail(na)) +
                                       ref_.a_des.tail(na);
  EXPECT_LT((r.tail(na) - expected_act).cwiseAbs().maxCoeff(), 1e-12);

  YAML::Node bad;
  for (int i = 0; i < na - 1; ++i) {
    bad["kp"].push_back(1.0);
  }
  PostureTask other;
  EXPECT_THROW(other.Init(*model_, info_, cache_, bad), std::invalid_argument);
}

// A floating base whose q and v do not pair joint by joint is refused (here
// a RobotModelInfo claiming one more q entry than a 1-DoF chain has).
TEST_F(FloatingPostureTest, AFloatingBaseWithUnpairedJointsIsRefused) {
  RobotModelInfo unpaired = info_;
  unpaired.nq += 1;
  PostureTask task;
  EXPECT_THROW(task.Init(*model_, unpaired, cache_, YAML::Node{}), std::invalid_argument);
  EXPECT_NO_THROW(task.Init(*model_, info_, cache_, YAML::Node{}));  // control
}

// SetGains: nv entries each, finite, or refused and the previous gains stay.
TEST_F(FloatingPostureTest, SetGainsRefusesWrongSizes) {
  PostureTask task;
  task.Init(*model_, info_, cache_, YAML::Node{});
  const Eigen::VectorXd r0 = Residual(task);
  const Eigen::VectorXd ones = Eigen::VectorXd::Ones(info_.nv);
  EXPECT_FALSE(task.SetGains(Eigen::VectorXd::Ones(info_.n_actuated), ones));
  Eigen::VectorXd nan = ones;
  nan(2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(task.SetGains(ones, nan));
  EXPECT_EQ(Residual(task), r0);
  EXPECT_TRUE(task.SetGains(ones, ones));
  EXPECT_NE(Residual(task), r0);
}

// SetReference: nq / nv / nv sizes and finite values, or refused and the
// previous reference stays.
TEST_F(FloatingPostureTest, SetReferenceRefusesWrongSizesAndNonFiniteValues) {
  PostureTask task;
  task.Init(*model_, info_, cache_, YAML::Node{});
  const Eigen::VectorXd zv = Eigen::VectorXd::Zero(info_.nv);
  Eigen::VectorXd q_des = pinocchio::neutral(*model_);
  q_des.tail(info_.n_actuated).setConstant(-0.1);
  ASSERT_TRUE(task.SetReference(q_des, zv, zv));
  const Eigen::VectorXd r_set = Residual(task);

  EXPECT_FALSE(task.SetReference(q_des.head(info_.nv), zv, zv));  // nv-long q
  Eigen::VectorXd bad = q_des;
  bad(10) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(task.SetReference(bad, zv, zv));
  Eigen::VectorXd bad_v = zv;
  bad_v(3) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(task.SetReference(q_des, bad_v, zv));
  EXPECT_EQ(Residual(task), r_set);
}

}  // namespace
}  // namespace rtc::tsid
