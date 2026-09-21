// ── Shipped catch frames on the real robot models (dynamic_catching S2.3a) ───
// Reads urdf.extra_frames from the SHIPPED robot configs (not a copy), builds
// the same model topology the RT node builds, and checks:
//   - the catch frame exists in every derived model at the declared placement
//     relative to its parent (full / sub / tree / actuated),
//   - its +z is the palm's outward normal: flexing every non-arm joint moves
//     the fingertip centroid toward +z (the FK argument the axis proposal
//     rests on, plan §10),
//   - it is still provisional (D-17: the user confirms in sim, S2.3b sets the
//     pocket offset),
//   - the model dimensions the plan quotes (ur5e_p1b full nv 26 / actuated 16).
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math/rpy.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

const std::string kConfigDir = RTC_BRINGUP_CONFIG_DIR;

struct RobotCase {
  const char* label;
  const char* config_rel;   // shipped robot config holding urdf.extra_frames
  const char* urdf_rel;     // robot_descriptions share-relative
  const char* closure_rel;  // nullptr → plain URDF
  const char* arm_sub;
  const char* arm_root;
  const char* arm_tip;
  const char* hand_tree;
  const char* hand_root;
  std::array<const char*, 4> tips;
  int arm_dof;
};

const RobotCase kUr5eP1b{
    "ur5e_p1b",
    "ur5e_p1b/_base.yaml",
    "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
    "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml",
    "ur5e",
    "base",
    "tool0",
    "p1b",
    "base_adapter",
    {"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket", "l_ring_tip_bracket"},
    6};

const RobotCase kIiwa7Leap{"iiwa7_leap",
                           "iiwa7_leap/sim.yaml",
                           "robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
                           nullptr,
                           "iiwa7",
                           "link_0",
                           "ee_link",
                           "leap",
                           "base",
                           {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"},
                           7};

std::vector<rub::ExtraFrameConfig> ShippedExtraFrames(const RobotCase& rc) {
  const YAML::Node root = YAML::LoadFile(kConfigDir + "/" + rc.config_rel);
  const YAML::Node frames = root["/**"]["ros__parameters"]["urdf"]["extra_frames"];
  std::vector<rub::ExtraFrameConfig> out;
  if (!frames) {
    return out;
  }
  for (const auto& kv : frames) {
    rub::ExtraFrameConfig ef;
    ef.name = kv.first.as<std::string>();
    ef.parent = kv.second["parent"].as<std::string>();
    const auto xyz = kv.second["xyz"].as<std::vector<double>>();
    const auto rpy = kv.second["rpy"].as<std::vector<double>>();
    ef.xyz = Eigen::Vector3d(xyz.at(0), xyz.at(1), xyz.at(2));
    ef.rpy = Eigen::Vector3d(rpy.at(0), rpy.at(1), rpy.at(2));
    ef.provisional = kv.second["provisional"] ? kv.second["provisional"].as<bool>() : true;
    out.push_back(ef);
  }
  return out;
}

rub::ModelConfig MakeConfig(const RobotCase& rc) {
  const std::string share = ament_index_cpp::get_package_share_directory("robot_descriptions");
  rub::ModelConfig cfg;
  cfg.urdf_path = share + "/" + rc.urdf_rel;
  cfg.root_joint_type = "fixed";
  if (rc.closure_rel != nullptr) {
    cfg.closure_yaml_path = share + "/" + rc.closure_rel;
  }
  cfg.sub_models.push_back({rc.arm_sub, rc.arm_root, rc.arm_tip});
  const std::vector<std::string> tips(rc.tips.begin(), rc.tips.end());
  cfg.tree_models.push_back({rc.hand_tree, rc.hand_root, tips});
  cfg.tree_models.push_back({"wbc", rc.arm_root, tips});
  cfg.extra_frames = ShippedExtraFrames(rc);
  return cfg;
}

const rub::ExtraFrameConfig* FindCatchFrame(const rub::ModelConfig& cfg) {
  for (const auto& ef : cfg.extra_frames) {
    if (ef.name == "catch_frame") {
      return &ef;
    }
  }
  return nullptr;
}

void CheckRobot(const RobotCase& rc) {
  const rub::ModelConfig cfg = MakeConfig(rc);
  const rub::ExtraFrameConfig* cf = FindCatchFrame(cfg);
  ASSERT_NE(cf, nullptr) << rc.label << ": no urdf.extra_frames.catch_frame in " << rc.config_rel;
  // SPEC CHANGE 2026-09-21 (PROC-6): this asserted `provisional == true`, as a
  // tripwire against shipping a confirmed-looking catch frame before S2.3b had
  // produced an offset and the user had checked it in sim. Both happened — the
  // offset is the S4.5 measured catch point (plan §10) and the user confirmed the
  // rendered frame — so the tripwire now points the other way: what ships must be
  // the confirmed state. A new robot profile copied from these, or a revert to the
  // placeholder, goes red here and has to be thought about rather than inherited.
  EXPECT_FALSE(cf->provisional)
      << rc.label << ": catch frame back to provisional — S2.3b and the user check are done";

  const rub::PinocchioModelBuilder builder(cfg);
  std::vector<std::pair<std::string, std::shared_ptr<const pinocchio::Model>>> models = {
      {"full", builder.GetFullModel()},
      {"sub", builder.GetReducedModel(rc.arm_sub)},
      {"tree:hand", builder.GetTreeModel(rc.hand_tree)},
      {"tree:wbc", builder.GetTreeModel("wbc")},
  };
  if (auto actuated = builder.GetActuatedModel()) {
    models.emplace_back("actuated", actuated);
  }

  const pinocchio::SE3 expected(pinocchio::rpy::rpyToMatrix(cf->rpy.x(), cf->rpy.y(), cf->rpy.z()),
                                cf->xyz);
  for (const auto& [label, model] : models) {
    ASSERT_TRUE(model->existFrame("catch_frame")) << rc.label << " " << label;
    pinocchio::Data data(*model);
    pinocchio::framesForwardKinematics(*model, data, pinocchio::neutral(*model));
    const pinocchio::SE3 rel =
        data.oMf[model->getFrameId(cf->parent)].actInv(data.oMf[model->getFrameId("catch_frame")]);
    EXPECT_TRUE(rel.isApprox(expected, 1e-12)) << rc.label << " " << label;
  }

  // +z is the palm's outward normal: bending every hand joint from straight
  // (q = 0) toward the limit FARTHER from 0 moves the fingertip centroid toward
  // catch-frame +z. "Farther limit", not "upper": P1b flexion runs negative
  // ([-π/2, 0]), LEAP positive, and interpolating lower → upper would measure
  // extension on one hand and flexion on the other. Checked at 25 % and 50 %
  // only — beyond that the spanning-tree P1b fingers (loop-passive joints moved
  // independently) curl back and the centroid is no longer monotone.
  const pinocchio::Model& full = *builder.GetFullModel();
  pinocchio::Data data(full);
  auto centroid_in_catch = [&](double frac) {
    Eigen::VectorXd q = pinocchio::neutral(full);
    for (int j = rc.arm_dof; j < full.nq; ++j) {
      const double lo = full.lowerPositionLimit(j);
      const double hi = full.upperPositionLimit(j);
      if (std::isfinite(lo) && std::isfinite(hi)) {
        q(j) = frac * (std::abs(lo) > std::abs(hi) ? lo : hi);
      }
    }
    pinocchio::framesForwardKinematics(full, data, q);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const char* tip : rc.tips) {
      c += data.oMf[full.getFrameId("catch_frame")].actInv(
          data.oMf[full.getFrameId(tip)].translation());
    }
    return Eigen::Vector3d(c / static_cast<double>(rc.tips.size()));
  };
  const double z0 = centroid_in_catch(0.0).z();
  for (const double frac : {0.25, 0.5}) {
    const double dz = centroid_in_catch(frac).z() - z0;
    ::testing::Test::RecordProperty(
        std::string(rc.label) + "_flex" + std::to_string(frac) + "_dz_m", std::to_string(dz));
    EXPECT_GT(dz, 0.02) << rc.label << " at " << frac
                        << ": catch frame +z does not face the flexing fingers";
  }
}

}  // namespace

TEST(CatchFrameModels, Ur5eP1b) {
  CheckRobot(kUr5eP1b);
  // Plan §2 / L5 §5.1: full tree nv 26 (UR5e 6 + P1b 20 revolute), actuated 16.
  const rub::PinocchioModelBuilder builder(MakeConfig(kUr5eP1b));
  EXPECT_EQ(builder.GetFullModel()->nv, 26);
  EXPECT_EQ(builder.GetFullModel()->nq, 26);
  ASSERT_NE(builder.GetActuatedModel(), nullptr);
  EXPECT_EQ(builder.GetActuatedModel()->nv, 16);
}

TEST(CatchFrameModels, Iiwa7Leap) {
  CheckRobot(kIiwa7Leap);
  // Recorded for plan §2 (no closure sidecar → no actuated model).
  const rub::PinocchioModelBuilder builder(MakeConfig(kIiwa7Leap));
  RecordProperty("iiwa7_leap_full_nv", builder.GetFullModel()->nv);
  RecordProperty("iiwa7_leap_wbc_nv", builder.GetTreeModel("wbc")->nv);
  EXPECT_EQ(builder.GetActuatedModel(), nullptr);
  EXPECT_EQ(builder.GetFullModel()->nv, 23);  // iiwa7 7 + LEAP 16
}
