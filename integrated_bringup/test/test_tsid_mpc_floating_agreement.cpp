// ── rtc_tsid and rtc_mpc read the same free-flyer the same way ──────────────
//
// The WBC stack hands rtc_mpc's floating state ([p, q_xyzw, q_j] and a LOCAL
// base twist, WbLayout) to rtc_tsid's PinocchioCache, and both packages decide
// "floating" on their own. This package is the first that sees both, so the
// agreement is held here:
//
//   * rtc::mpc::IsFloatingBaseModel == RobotModelInfo::Build().floating_base
//     on the public G1 with a free-flyer root (true), with no root joint
//     (false), and with a translation + spherical composite root — nq 7 / nv 6
//     like a free-flyer, which the old dimension rule took for one;
//   * a state composed by WbLayout and given to PinocchioCache::Update puts the
//     base where the pose says and moves it with the twist it was given.
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-spherical.hpp>
#include <pinocchio/multibody/joint/joint-translation.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

namespace {

std::string G1Urdf() {
  return ament_index_cpp::get_package_share_directory("robot_descriptions") +
         "/robots/g1/urdf/g1_29dof_floating.urdf";
}

std::shared_ptr<pinocchio::Model> FreeFlyerG1() {
  auto m = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(G1Urdf(), pinocchio::JointModelFreeFlyer(), *m);
  return m;
}

std::shared_ptr<pinocchio::Model> FixedG1() {
  auto m = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(G1Urdf(), *m);
  return m;
}

std::shared_ptr<pinocchio::Model> CompositeRootG1() {
  pinocchio::JointModelComposite root;
  root.addJoint(pinocchio::JointModelTranslation());
  root.addJoint(pinocchio::JointModelSpherical());
  auto m = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(G1Urdf(), root, *m);
  return m;
}

bool TsidFloating(const pinocchio::Model& m) {
  rtc::tsid::RobotModelInfo info;
  info.Build(m, YAML::Node{});
  return info.floating_base;
}

// The rule rtc_tsid used before: the root's dimensions.
bool DimensionRule(const pinocchio::Model& m) {
  return m.joints.size() > 1 && m.joints[1].nq() == 7 && m.joints[1].nv() == 6;
}

TEST(TsidMpcFloatingAgreement, BothPackagesCallTheSameModelsFloating) {
  const auto ff = FreeFlyerG1();
  const auto fixed = FixedG1();
  const auto composite = CompositeRootG1();

  EXPECT_TRUE(rtc::mpc::IsFloatingBaseModel(*ff));
  EXPECT_TRUE(TsidFloating(*ff));
  EXPECT_FALSE(rtc::mpc::IsFloatingBaseModel(*fixed));
  EXPECT_FALSE(TsidFloating(*fixed));
  EXPECT_FALSE(rtc::mpc::IsFloatingBaseModel(*composite));
  EXPECT_FALSE(TsidFloating(*composite));

  // Control: the composite root is the case the dimension rule gets wrong.
  ASSERT_TRUE(DimensionRule(*composite));
  EXPECT_NE(DimensionRule(*composite), rtc::mpc::IsFloatingBaseModel(*composite));
}

TEST(TsidMpcFloatingAgreement, AComposedStatePlacesAndMovesTheBaseInTheCache) {
  const auto model = FreeFlyerG1();
  const auto layout = rtc::mpc::WbLayout::FromModel(*model, 0);
  ASSERT_TRUE(layout.floating_base());
  ASSERT_EQ(layout.n_base_q, 7);
  ASSERT_EQ(layout.n_base_v, 6);

  // Base pose [p, q_xyzw]: a rotation about a skew axis, so R, Rᵀ and I differ.
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(0.9, Eigen::Vector3d(1, 2, -1).normalized()));
  Eigen::VectorXd pose(7);
  pose << 0.4, -0.3, 0.8, rot.x(), rot.y(), rot.z(), rot.w();
  const Eigen::VectorXd q_j = pinocchio::neutral(*model).tail(model->nq - 7);
  Eigen::VectorXd q(model->nq);
  ASSERT_TRUE(layout.ComposeQ(pose, q_j, q));

  // LOCAL base twist [v; ω] in the base frame.
  Eigen::VectorXd twist(6);
  twist << 0.5, -0.2, 0.1, 0.3, 0.7, -0.4;
  const Eigen::VectorXd v_j = Eigen::VectorXd::Zero(model->nv - 6);
  Eigen::VectorXd v(model->nv);
  ASSERT_TRUE(layout.ComposeV(twist, v_j, v));

  rtc::tsid::PinocchioCache cache;
  cache.Init(model, {});
  cache.Update(q, v);

  const pinocchio::SE3& oMb = cache.data.oMi[1];
  EXPECT_LT((oMb.translation() - pose.head<3>()).norm(), 1e-12);
  EXPECT_TRUE(oMb.rotation().isApprox(rot.toRotationMatrix(), 1e-12));
  // data.v[1] is the base's spatial velocity in its own frame: the LOCAL twist.
  EXPECT_LT((cache.data.v[1].linear() - twist.head<3>()).norm(), 1e-12);
  EXPECT_LT((cache.data.v[1].angular() - twist.tail<3>()).norm(), 1e-12);
  // Control: in world axes the same twist is R·v, which is not v here.
  EXPECT_GT((rot.toRotationMatrix() * twist.head<3>() - twist.head<3>()).norm(),
            0.1 * twist.head<3>().norm());
}

}  // namespace
