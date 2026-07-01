// ── test_constraint_builder — tree load / frame lookup / endpoint transform /
//    builder (Phase 3·4·5·6) ────────────────────────────────────────────────
#include "rtc_urdf_bridge/constraint_builder.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "test_urdf_path.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <stdexcept>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

namespace {
pinocchio::Model LoadTreeModel() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(TestUrdfPath("four_bar_tree.urdf"), model);
  return model;
}

rub::ClosedChainInfo FrameSpec(bool is_6d) {
  rub::ClosedChainInfo cc;
  cc.name = "fourbar_loop_0";
  cc.frame_1 = "c1";
  cc.frame_2 = "c2";
  cc.is_6d = is_6d;
  cc.reference_frame = rub::LoopReferenceFrame::kLocal;
  return cc;
}
}  // namespace

// ── Phase 3: 표준 URDF spanning-tree 정상 build (nq/nv 예상값) ────────────────
TEST(ConstraintBuilder, TreeModelHasExpectedDof) {
  const pinocchio::Model model = LoadTreeModel();
  // 4 revolute (joint_a, joint_ab, joint_c, joint_cd), fixed base, 2 fixed mount.
  EXPECT_EQ(model.nq, 4);
  EXPECT_EQ(model.nv, 4);
  EXPECT_TRUE(model.existJointName("joint_a"));
  EXPECT_TRUE(model.existJointName("joint_cd"));
}

// ── Phase 4: closure frame 이 model 에 존재 ──────────────────────────────────
TEST(ConstraintBuilder, ClosureFramesExist) {
  const pinocchio::Model model = LoadTreeModel();
  EXPECT_TRUE(model.existFrame("c1"));
  EXPECT_TRUE(model.existFrame("c2"));
  EXPECT_FALSE(model.existFrame("nonexistent_frame"));
}

// ── Phase 5: endpoint transform — joint_M_closure 수동 계산 대조 ──────────────
// c1 은 c1_mount(fixed, origin [0.3,0,0]) 로 link_b 에 붙음 → parentJoint = joint_ab,
// frames["c1"].placement = SE3([0.3,0,0]). frame-first 경로는 이를 그대로 사용.
TEST(ConstraintBuilder, EndpointTransformFrameFirstMatchesManual) {
  const pinocchio::Model model = LoadTreeModel();
  const rub::ResolvedEndpoint ep =
      rub::ResolveEndpoint(model, "c1", "", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  EXPECT_EQ(ep.joint_id, model.getJointId("joint_ab"));
  EXPECT_NEAR(ep.joint_M_closure.translation().x(), 0.3, 1e-9);
  EXPECT_NEAR(ep.joint_M_closure.translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(ep.joint_M_closure.translation().z(), 0.0, 1e-9);
  EXPECT_TRUE(ep.joint_M_closure.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

// ── Phase 5b: link+origin fallback — frames[link].placement 합성 검증 ────────
// c2 를 link_d + origin[0,0,0] 로 지정하면 c2_mount(identity) 결과와 동일해야 한다.
// (link BODY frame placement 는 identity → 합성이 origin 을 그대로 반영)
TEST(ConstraintBuilder, EndpointTransformLinkFallbackComposesPlacement) {
  const pinocchio::Model model = LoadTreeModel();
  const Eigen::Vector3d xyz(0.05, -0.02, 0.01);
  const rub::ResolvedEndpoint ep =
      rub::ResolveEndpoint(model, "", "link_d", xyz, Eigen::Vector3d::Zero());

  EXPECT_EQ(ep.joint_id, model.getJointId("joint_cd"));
  // link_d BODY frame placement 는 identity 이므로 joint_M_closure = origin.
  EXPECT_TRUE(ep.joint_M_closure.translation().isApprox(xyz, 1e-9));
}

// ── Phase 4b: 없는 frame/link → 명확한 예외 ──────────────────────────────────
TEST(ConstraintBuilder, ThrowsOnMissingFrame) {
  const pinocchio::Model model = LoadTreeModel();
  EXPECT_THROW(
      rub::ResolveEndpoint(model, "ghost", "", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      std::runtime_error);
  EXPECT_THROW(rub::ResolveEndpoint(model, "", "ghost_link", Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::Zero()),
               std::runtime_error);
}

// ── Phase 6: builder — 3D → size 3, 6D → size 6 ──────────────────────────────
TEST(ConstraintBuilder, Contact3dHasSizeThree) {
  const pinocchio::Model model = LoadTreeModel();
  const pinocchio::RigidConstraintModel cm = rub::BuildRigidConstraint(model, FrameSpec(false));
  EXPECT_EQ(cm.type, pinocchio::CONTACT_3D);
  EXPECT_EQ(cm.name, "fourbar_loop_0");
  const std::vector<pinocchio::RigidConstraintModel> v{cm};
  EXPECT_EQ(rub::TotalConstraintDim(v), 3);
}

TEST(ConstraintBuilder, Contact6dHasSizeSix) {
  const pinocchio::Model model = LoadTreeModel();
  const pinocchio::RigidConstraintModel cm = rub::BuildRigidConstraint(model, FrameSpec(true));
  EXPECT_EQ(cm.type, pinocchio::CONTACT_6D);
  const std::vector<pinocchio::RigidConstraintModel> v{cm};
  EXPECT_EQ(rub::TotalConstraintDim(v), 6);
}

// ── Baumgarte: Kp>0 대입 확인 ─────────────────────────────────────────────────
TEST(ConstraintBuilder, BaumgarteAppliedWhenKpPositive) {
  const pinocchio::Model model = LoadTreeModel();
  rub::ClosedChainInfo cc = FrameSpec(false);
  cc.baumgarte_kp = 20.0;
  cc.baumgarte_kd = 5.0;
  const pinocchio::RigidConstraintModel cm = rub::BuildRigidConstraint(model, cc);
  EXPECT_DOUBLE_EQ(cm.baumgarte_corrector_parameters().Kp, 20.0);
  EXPECT_DOUBLE_EQ(cm.baumgarte_corrector_parameters().Kd, 5.0);
}

// ── reference_frame == WORLD → 미지원 예외 ───────────────────────────────────
TEST(ConstraintBuilder, ThrowsOnWorldReferenceFrame) {
  const pinocchio::Model model = LoadTreeModel();
  rub::ClosedChainInfo cc = FrameSpec(false);
  cc.reference_frame = rub::LoopReferenceFrame::kWorld;
  EXPECT_THROW(rub::BuildRigidConstraint(model, cc), std::runtime_error);
}
