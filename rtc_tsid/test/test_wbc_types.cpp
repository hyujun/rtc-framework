#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/types/wbc_types.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

// ──────────────────────────────────────────────
// RobotModelInfo
// ──────────────────────────────────────────────
class RobotModelInfoTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pinocchio::urdf::buildModel(kPandaUrdf, model_);
  }
  pinocchio::Model model_;
};

TEST_F(RobotModelInfoTest, PandaFixedBase) {
  RobotModelInfo info;
  YAML::Node config;
  info.build(model_, config);

  EXPECT_EQ(info.nq, model_.nq);
  EXPECT_EQ(info.nv, model_.nv);
  EXPECT_FALSE(info.floating_base);
  EXPECT_EQ(info.n_actuated, info.nv);

  // S = Identity for fixed-base
  EXPECT_EQ(info.S.rows(), info.n_actuated);
  EXPECT_EQ(info.S.cols(), info.nv);
  EXPECT_TRUE(info.S.isApprox(Eigen::MatrixXd::Identity(info.nv, info.nv)));

  EXPECT_EQ(info.tau_max.size(), info.n_actuated);
  EXPECT_EQ(info.tau_min.size(), info.n_actuated);
  EXPECT_EQ(info.q_upper.size(), info.nq);
  EXPECT_EQ(info.q_lower.size(), info.nq);
  EXPECT_EQ(info.v_max.size(), info.nv);
}

TEST_F(RobotModelInfoTest, FloatingBaseOverride) {
  RobotModelInfo info;
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = 7;
  info.build(model_, config);

  EXPECT_TRUE(info.floating_base);
  EXPECT_EQ(info.n_actuated, 7);
  EXPECT_EQ(info.S.rows(), 7);
  EXPECT_EQ(info.S.cols(), info.nv);
  EXPECT_TRUE(info.S.rightCols(7).isApprox(Eigen::MatrixXd::Identity(7, 7)));
  EXPECT_TRUE(info.S.leftCols(info.nv - 7).isZero());
}

// ──────────────────────────────────────────────
// ContactManagerConfig
// ──────────────────────────────────────────────
TEST_F(RobotModelInfoTest, ContactManagerEmpty) {
  ContactManagerConfig mgr;
  YAML::Node config;
  mgr.load(config, model_);

  EXPECT_EQ(mgr.max_contacts, 0);
  EXPECT_EQ(mgr.max_contact_vars, 0);
}

TEST_F(RobotModelInfoTest, ContactManagerWithContacts) {
  std::string frame_name;
  for (size_t i = 0; i < model_.frames.size(); ++i) {
    if (model_.frames[i].name.find("link") != std::string::npos) {
      frame_name = model_.frames[i].name;
      break;
    }
  }
  ASSERT_FALSE(frame_name.empty());

  YAML::Node config;
  YAML::Node contact;
  contact["name"] = "test_contact";
  contact["frame"] = frame_name;
  contact["type"] = "point";
  contact["friction_coeff"] = 0.5;
  contact["friction_faces"] = 8;
  config["contacts"].push_back(contact);

  ContactManagerConfig mgr;
  mgr.load(config, model_);

  EXPECT_EQ(mgr.max_contacts, 1);
  EXPECT_EQ(mgr.max_contact_vars, 3);
  EXPECT_EQ(mgr.contacts[0].contact_dim, 3);
  EXPECT_DOUBLE_EQ(mgr.contacts[0].friction_coeff, 0.5);
  EXPECT_EQ(mgr.contacts[0].friction_faces, 8);
}

// ──────────────────────────────────────────────
// ContactState
// ──────────────────────────────────────────────
TEST(ContactStateTest, InitAndRecompute) {
  ContactManagerConfig mgr;
  mgr.max_contacts = 3;
  mgr.contacts.resize(3);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[1].contact_dim = 6;
  mgr.contacts[2].contact_dim = 3;

  ContactState cs;
  cs.init(3);
  EXPECT_EQ(cs.active_count, 0);
  EXPECT_EQ(cs.active_contact_vars, 0);

  cs.contacts[0].active = true;
  cs.contacts[2].active = true;
  cs.recompute_active(mgr);
  EXPECT_EQ(cs.active_count, 2);
  EXPECT_EQ(cs.active_contact_vars, 6);
}

// ──────────────────────────────────────────────
// PinocchioCache
// ──────────────────────────────────────────────
class PinocchioCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.init(model_, contact_cfg);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  PinocchioCache cache_;
};

TEST_F(PinocchioCacheTest, InitBufferSizes) {
  EXPECT_EQ(cache_.M.rows(), model_->nv);
  EXPECT_EQ(cache_.M.cols(), model_->nv);
  EXPECT_EQ(cache_.h.size(), model_->nv);
  EXPECT_EQ(cache_.g.size(), model_->nv);
  EXPECT_EQ(cache_.q.size(), model_->nq);
  EXPECT_EQ(cache_.v.size(), model_->nv);
}

TEST_F(PinocchioCacheTest, UpdateAtNeutralConfig) {
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v_vec = Eigen::VectorXd::Zero(model_->nv);
  ContactState cs;
  cache_.update(q, v_vec, cs);

  // M 대칭
  EXPECT_TRUE(cache_.M.isApprox(cache_.M.transpose(), 1e-10));
  // v=0 → h = g
  EXPECT_TRUE(cache_.h.isApprox(cache_.g, 1e-10));
  // 중력 존재
  EXPECT_GT(cache_.g.norm(), 0.0);
  EXPECT_GT(cache_.M.norm(), 0.0);
}

TEST_F(PinocchioCacheTest, RegisterFrame) {
  pinocchio::FrameIndex fid = 0;
  for (size_t i = 0; i < model_->frames.size(); ++i) {
    if (model_->frames[i].name.find("link") != std::string::npos) {
      fid = static_cast<pinocchio::FrameIndex>(i);
      break;
    }
  }

  int idx = cache_.register_frame("test_frame", fid);
  EXPECT_GE(idx, 0);
  EXPECT_EQ(static_cast<int>(cache_.registered_frames.size()), 1);

  // 중복 등록 → 같은 인덱스
  int idx2 = cache_.register_frame("dup", fid);
  EXPECT_EQ(idx, idx2);

  // Update 후 Jacobian 계산됨
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v_vec = Eigen::VectorXd::Zero(model_->nv);
  ContactState cs;
  cache_.update(q, v_vec, cs);

  EXPECT_EQ(cache_.registered_frames[0].J.rows(), 6);
  EXPECT_EQ(cache_.registered_frames[0].J.cols(), model_->nv);

  // update 이후 등록 시도 실패
  int idx3 = cache_.register_frame("late", fid);
  EXPECT_EQ(idx3, -1);
}

TEST_F(PinocchioCacheTest, MassMatrixSymmetryRandom) {
  Eigen::VectorXd q = pinocchio::randomConfiguration(*model_);
  Eigen::VectorXd v_vec = Eigen::VectorXd::Random(model_->nv) * 0.1;
  ContactState cs;
  cache_.update(q, v_vec, cs);

  Eigen::MatrixXd diff = cache_.M - cache_.M.transpose();
  EXPECT_LT(diff.norm(), 1e-10);
}

// ──────────────────────────────────────────────
// ControlReference / CommandOutput
// ──────────────────────────────────────────────
TEST(IOTest, ControlReferenceInit) {
  ControlReference ref;
  ref.init(9, 9, 9, 6);
  EXPECT_EQ(ref.q_des.size(), 9);
  EXPECT_EQ(ref.a_des.size(), 9);
  EXPECT_EQ(ref.lambda_des.size(), 6);
}

TEST(IOTest, CommandOutputInit) {
  CommandOutput out;
  out.init(9, 7, 6);
  EXPECT_EQ(out.tau.size(), 7);
  EXPECT_EQ(out.a_opt.size(), 9);
  EXPECT_EQ(out.lambda_opt.size(), 6);
  EXPECT_FALSE(out.qp_converged);
}

}  // namespace
}  // namespace rtc::tsid
