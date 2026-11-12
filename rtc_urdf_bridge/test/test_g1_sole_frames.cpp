// G1 sole contact frames (W1-F03, WD-31): the floating G1 URDF carries `left_sole` /
// `right_sole` as bare fixed links on `*_ankle_roll_link`, at the sole's bottom face.
//
// This is the asset PR's own sensor (AP-PROC-7): the frames are checked on the two URDFs the
// repo loads — the plant URDF and the g1_loco CM xacro that includes it — through the same
// builder the controllers use. The offset literal below is the URDF's, so this test pins the
// URDF against itself; that the point is the MJCF `*_sole_ft_site` is the pair gate's job
// (`model_pairs.yaml` g1 pair `tip_frames`, rtc_tools test_real_model_pairs.py, left side —
// the right side is held to it here by the left == right assertion).
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

namespace rub = rtc_urdf_bridge;

namespace {

constexpr double kTol = 1e-12;

std::shared_ptr<const pinocchio::Model> BuildFloating(const std::string& relative) {
  rub::ModelConfig cfg;
  cfg.urdf_path =
      (std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_descriptions")) /
       relative)
          .string();
  cfg.root_joint_type = "floating";
  return rub::PinocchioModelBuilder(cfg).GetFullModel();
}

// The sole's placement in its ankle_roll link's frame, after checking the frame chain
// sole (body) -> sole_joint (fixed) -> ankle_roll_link (body), all on ankle_roll_joint.
pinocchio::SE3 SoleInAnkle(const pinocchio::Model& m, const std::string& side) {
  const std::string sole = side + "_sole";
  const std::string ankle = side + "_ankle_roll_link";
  EXPECT_TRUE(m.existFrame(sole, pinocchio::BODY)) << sole;
  EXPECT_TRUE(m.existFrame(side + "_sole_joint", pinocchio::FIXED_JOINT)) << side;
  EXPECT_TRUE(m.existFrame(ankle, pinocchio::BODY)) << ankle;
  if (!m.existFrame(sole, pinocchio::BODY) || !m.existFrame(ankle, pinocchio::BODY)) {
    return pinocchio::SE3::Identity();
  }
  const auto& f_sole = m.frames[m.getFrameId(sole, pinocchio::BODY)];
  const auto& f_fixed = m.frames[f_sole.parentFrame];
  const auto ankle_id = m.getFrameId(ankle, pinocchio::BODY);
  EXPECT_EQ(f_fixed.name, side + "_sole_joint");
  EXPECT_EQ(f_fixed.parentFrame, ankle_id) << "the sole's parent link is not " << ankle;
  EXPECT_EQ(m.names[f_sole.parentJoint], side + "_ankle_roll_joint");
  return m.frames[ankle_id].placement.actInv(f_sole.placement);
}

class G1SoleFrames : public ::testing::TestWithParam<std::string> {};

TEST_P(G1SoleFrames, EachSoleIsTheAnkleRollBottomFacePointWithTheAnkleAxes) {
  const auto model = BuildFloating(GetParam());
  ASSERT_NE(model, nullptr);
  const pinocchio::SE3 left = SoleInAnkle(*model, "left");
  const pinocchio::SE3 right = SoleInAnkle(*model, "right");

  EXPECT_LE((left.translation() - Eigen::Vector3d(0.035, 0.0, -0.035)).norm(), kTol)
      << left.translation().transpose();
  EXPECT_LE((left.rotation() - Eigen::Matrix3d::Identity()).norm(), kTol);
  // Left == right: the pair gate compares only the left sole against the MJCF.
  EXPECT_LE((right.translation() - left.translation()).norm(), kTol)
      << right.translation().transpose();
  EXPECT_LE((right.rotation() - left.rotation()).norm(), kTol);
}

INSTANTIATE_TEST_SUITE_P(
    PlantAndCmModel, G1SoleFrames,
    ::testing::Values("robots/g1/urdf/g1_29dof_floating.urdf",
                      "robots/g1/urdf/g1_29dof_floating_with_p1b_surrogate.urdf.xacro"),
    [](const ::testing::TestParamInfo<std::string>& info) {
      return std::filesystem::path(info.param).stem().stem().string();
    });

// Fixed children add no DoF: the plant keeps nq 30 / nv 29 (free-flyer 7 / 6 + 23 actuated).
TEST(G1SoleFrames, TheSolesAddNoDegreeOfFreedom) {
  const auto model = BuildFloating("robots/g1/urdf/g1_29dof_floating.urdf");
  ASSERT_NE(model, nullptr);
  EXPECT_EQ(model->nq, 30);
  EXPECT_EQ(model->nv, 29);
  EXPECT_EQ(model->njoints, 25);  // universe + free-flyer + 23 revolute
}

}  // namespace
