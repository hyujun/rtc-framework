#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

// Cross-product / skew helper for hand-rolled reference G computation.
Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  S(0, 1) = -v.z();
  S(0, 2) = v.y();
  S(1, 0) = v.z();
  S(1, 2) = -v.x();
  S(2, 0) = -v.y();
  S(2, 1) = v.x();
  return S;
}

// Pick a list of distinct named frames from the Panda model to use as
// contact points. Returns first N matching "linkK" frames in order.
std::vector<std::string> PickFrames(const pinocchio::Model& model,
                                    const std::vector<std::string>& wanted) {
  std::vector<std::string> out;
  for (const auto& w : wanted) {
    for (size_t i = 0; i < model.frames.size(); ++i) {
      if (model.frames[i].name == w) {
        out.push_back(w);
        break;
      }
    }
  }
  return out;
}

class GraspMatrixTest : public ::testing::Test {
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

  // Configure contacts with the given (frame_name, cdim) pairs.
  void ConfigureContacts(const std::vector<std::pair<std::string, int>>& contacts) {
    contact_cfg_ = ContactManagerConfig{};
    contact_cfg_.contacts.resize(contacts.size());
    int total = 0;
    for (size_t i = 0; i < contacts.size(); ++i) {
      auto& c = contact_cfg_.contacts[i];
      c.name = "c" + std::to_string(i);
      c.frame_name = contacts[i].first;
      c.frame_id = static_cast<int>(model_->getFrameId(c.frame_name));
      c.contact_dim = contacts[i].second;
      c.friction_coeff = 0.7;
      c.friction_faces = 4;
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
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ContactManager mgr_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
};

// Case 1: single point contact, object at world origin → G is the
// canonical [I_3; skew(p_ci)] block. Tests the trivial baseline.
TEST_F(GraspMatrixTest, IdentityFrame_PointContact_Trivial) {
  // Panda has link3..link7; pick one with a non-trivial world placement.
  auto frames = PickFrames(*model_, {"panda_link5"});
  ASSERT_EQ(frames.size(), 1u);

  ConfigureContacts({{frames[0], 3}});

  ObjectFrame obj;  // origin at world (0,0,0), identity rotation
  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 3);

  Eigen::MatrixXd G(6, 3);
  G.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G);

  const Eigen::Vector3d p_ci = cache_.contact_frames[0].oMf.translation();
  Eigen::Matrix<double, 6, 3> G_ref = Eigen::Matrix<double, 6, 3>::Zero();
  G_ref.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  G_ref.bottomLeftCorner<3, 3>() = Skew(p_ci);

  EXPECT_LT((G - G_ref).norm(), 1e-12);
}

// Case 2: two point contacts forming a pinch — verify column blocks land
// at the expected offsets and that the lever-arm uses (p_ci - p_o), not
// p_ci. Picks a non-origin ObjectFrame to exercise p_o subtraction.
TEST_F(GraspMatrixTest, TwoFinger_PinchGrasp_RankFour) {
  auto frames = PickFrames(*model_, {"panda_link3", "panda_link5"});
  ASSERT_EQ(frames.size(), 2u);

  ConfigureContacts({{frames[0], 3}, {frames[1], 3}});

  ObjectFrame obj;
  obj.p_w = Eigen::Vector3d{0.1, -0.05, 0.3};

  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 6);

  Eigen::MatrixXd G(6, 6);
  G.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G);

  // Verify each 6×3 block against reference.
  for (int i = 0; i < 2; ++i) {
    const Eigen::Vector3d p_ci = cache_.contact_frames[static_cast<size_t>(i)].oMf.translation();
    Eigen::Matrix<double, 6, 3> B_ref = Eigen::Matrix<double, 6, 3>::Zero();
    B_ref.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
    B_ref.bottomLeftCorner<3, 3>() = Skew(p_ci - obj.p_w);

    const Eigen::MatrixXd B = G.block(0, 3 * i, 6, 3);
    EXPECT_LT((B - B_ref).norm(), 1e-12) << "block " << i;
  }

  // Two point contacts: the force part (top 3 rows) has columns [I_3 | I_3]
  // which has rank 3. The full G has rank ≤ min(6, 6) = 6; for typical
  // non-collinear positions the actual rank is ≥ 4 (force span 3 + at
  // least one independent moment direction perpendicular to the line
  // connecting the contact points). We assert rank ≥ 4 as the documented
  // "pinch grasp" lower bound (cf. Stage B-3 RankBasedNullity rationale).
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(G);
  const auto& sv = svd.singularValues();
  int rank = 0;
  for (int k = 0; k < sv.size(); ++k) {
    if (sv(k) > 1e-9)
      ++rank;
  }
  EXPECT_GE(rank, 4);
}

// Case 3: three non-coplanar point contacts → full row-rank 6 (object is
// fully restrained at force/moment level). This is the typical 3-finger
// fingertip grasp targeted by Stage A-1+ (ur5e_hand thumb/index/middle).
TEST_F(GraspMatrixTest, ThreeFinger_NonCoplanar_RankSix) {
  auto frames = PickFrames(*model_, {"panda_link3", "panda_link5", "panda_link7"});
  ASSERT_EQ(frames.size(), 3u);

  ConfigureContacts({{frames[0], 3}, {frames[1], 3}, {frames[2], 3}});

  ObjectFrame obj;
  obj.p_w = Eigen::Vector3d{0.2, 0.1, 0.5};

  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 9);

  Eigen::MatrixXd G(6, 9);
  G.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(G);
  const auto& sv = svd.singularValues();
  // Smallest of 6 singular values should be well above zero for a
  // non-coplanar 3-fingertip grasp.
  EXPECT_GT(sv(5), 1e-6) << "G singular values: " << sv.transpose();

  // Confirm each block matches Skew(p_ci - p_o) in the moment rows.
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d p_ci = cache_.contact_frames[static_cast<size_t>(i)].oMf.translation();
    const Eigen::Matrix3d S_ref = Skew(p_ci - obj.p_w);
    const Eigen::Matrix3d S_got = G.block(3, 3 * i, 3, 3);
    EXPECT_LT((S_got - S_ref).norm(), 1e-12) << "moment block " << i;
  }
}

// Case 4: single surface contact (cdim=6) → 6×6 block must equal the
// Adjoint-dual transport AdjointDualWorldAligned(p_ci, p_o). Also
// verifies the helper directly against an explicit construction.
TEST_F(GraspMatrixTest, SurfaceContact_AdjointDual) {
  auto frames = PickFrames(*model_, {"panda_link5"});
  ASSERT_EQ(frames.size(), 1u);

  ConfigureContacts({{frames[0], 6}});

  ObjectFrame obj;
  obj.p_w = Eigen::Vector3d{-0.05, 0.15, 0.2};

  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 6);

  Eigen::MatrixXd G(6, 6);
  G.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G);

  const Eigen::Vector3d p_ci = cache_.contact_frames[0].oMf.translation();
  const Eigen::Matrix<double, 6, 6> Ad_ref = AdjointDualWorldAligned(p_ci, obj.p_w);

  // (a) ContactManager-produced G matches the helper.
  EXPECT_LT((G - Ad_ref).norm(), 1e-12);

  // (b) Helper structure: top-left I, top-right 0, bottom-right I,
  //     bottom-left skew(p_ci - p_o). Use parenthesized templated
  //     corner accessors so the comma inside the angle brackets does not
  //     break the EXPECT_* preprocessor macros.
  const Eigen::Matrix3d top_left = Ad_ref.topLeftCorner<3, 3>();
  const Eigen::Matrix3d top_right = Ad_ref.topRightCorner<3, 3>();
  const Eigen::Matrix3d bot_right = Ad_ref.bottomRightCorner<3, 3>();
  const Eigen::Matrix3d bot_left = Ad_ref.bottomLeftCorner<3, 3>();
  EXPECT_LT((top_left - Eigen::Matrix3d::Identity()).norm(), 1e-15);
  EXPECT_LT(top_right.norm(), 1e-15);
  EXPECT_LT((bot_right - Eigen::Matrix3d::Identity()).norm(), 1e-15);
  EXPECT_LT((bot_left - Skew(p_ci - obj.p_w)).norm(), 1e-15);

  // (c) StackContactJacobians shape sanity — top-6 rows of contact Jac.
  const int nv = robot_info_.nv;
  Eigen::MatrixXd Jc(6, nv);
  Jc.setZero();
  mgr_.StackContactJacobians(cache_, contacts_, Jc);
  EXPECT_LT((Jc - cache_.contact_frames[0].J.topRows(6)).norm(), 1e-15);
}

// Bonus: inactive contact column-skip behavior. Activate only the middle
// contact and verify ActiveLambdaDim + G column count + block content.
TEST_F(GraspMatrixTest, InactiveContactsSkipColumns) {
  auto frames = PickFrames(*model_, {"panda_link3", "panda_link5", "panda_link7"});
  ASSERT_EQ(frames.size(), 3u);

  ConfigureContacts({{frames[0], 3}, {frames[1], 3}, {frames[2], 3}});

  // Deactivate 0 and 2, keep only 1.
  contacts_.contacts[0].active = false;
  contacts_.contacts[2].active = false;
  contacts_.RecomputeActive(contact_cfg_);

  ASSERT_EQ(mgr_.ActiveLambdaDim(contacts_), 3);

  ObjectFrame obj;
  Eigen::MatrixXd G(6, 3);
  G.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G);

  const Eigen::Vector3d p_ci = cache_.contact_frames[1].oMf.translation();
  Eigen::Matrix<double, 6, 3> B_ref = Eigen::Matrix<double, 6, 3>::Zero();
  B_ref.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  B_ref.bottomLeftCorner<3, 3>() = Skew(p_ci);
  EXPECT_LT((G - B_ref).norm(), 1e-12);
}

}  // namespace
}  // namespace rtc::tsid
