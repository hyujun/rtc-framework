// ── Shipped catch frames on the real robot models ────────────────────────────
// Reads urdf.extra_frames from the SHIPPED robot configs (not a copy), builds
// the same model topology the RT node builds, and checks:
//   - the catch frame exists in every derived model at the declared placement
//     relative to its parent (full / sub / tree / actuated),
//   - its +z is the palm's outward normal: flexing every non-arm joint moves
//     the fingertip centroid toward +z (the FK argument the axis proposal
//     rests on),
//   - it is not provisional (the user confirmed it in sim),
//   - the model dimensions (iiwa7_leap full nv 23).
//
// Only robots whose model this repository owns are listed. ur5e_p1b ships a
// catch frame too, but its model belongs to `hand_description`: this
// repository loads it and does not verify it (#682).
// That the declared frame reaches every derived model is this test's concern
// only for iiwa7_leap; the mechanism itself is rtc_urdf_bridge's
// test_extra_frames.
//
// g1_p1b is here for ONE thing this repository does own (#640): the
// profile's own declaration of two planner models — the search chain
// `sub_models.g1_catch` and the device tree `tree_models.g1` — and that the
// catch frame is the same frame in both. Its model is `hand_description`'s
// too, so that case skips where the package is absent (optional_package.hpp).
// The checks of the hand (the four fingertips, `provisional == false`) are NOT
// made for it: the frame is still provisional on that robot.
#include "optional_package.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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
  // This once asserted `provisional == true`, as a tripwire against shipping a
  // confirmed-looking catch frame before the offset was measured and the user
  // had checked it in sim. Both happened, so the tripwire now points the other
  // way: what ships must be the confirmed state. A new robot profile copied from
  // these, or a revert to the placeholder, goes red here and has to be thought
  // about rather than inherited.
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

TEST(CatchFrameModels, Iiwa7Leap) {
  CheckRobot(kIiwa7Leap);
  // Recorded as properties (no closure sidecar → no actuated model).
  const rub::PinocchioModelBuilder builder(MakeConfig(kIiwa7Leap));
  RecordProperty("iiwa7_leap_full_nv", builder.GetFullModel()->nv);
  RecordProperty("iiwa7_leap_wbc_nv", builder.GetTreeModel("wbc")->nv);
  EXPECT_EQ(builder.GetActuatedModel(), nullptr);
  EXPECT_EQ(builder.GetFullModel()->nv, 23);  // iiwa7 7 + LEAP 16
}

// ── g1_p1b: one catch frame, two planner models ──────────────────────────────

// The profile's `urdf:` block as a ModelConfig: its package, files, sub-models,
// tree models and extra frames, as shipped.
rub::ModelConfig ShippedG1Config() {
  const YAML::Node urdf =
      YAML::LoadFile(kConfigDir + "/g1_p1b/_base.yaml")["/**"]["ros__parameters"]["urdf"];
  // The literal is what the fixture gate reads; the profile must agree with it.
  EXPECT_EQ(urdf["package"].as<std::string>(), "hand_description");
  const std::string share = ament_index_cpp::get_package_share_directory("hand_description");
  rub::ModelConfig cfg;
  cfg.urdf_path = share + "/" + urdf["path"].as<std::string>();
  cfg.root_joint_type = urdf["root_joint_type"].as<std::string>();
  cfg.closure_yaml_path = share + "/" + urdf["closure_path"].as<std::string>();
  for (const auto& kv : urdf["sub_models"]) {
    cfg.sub_models.push_back({kv.first.as<std::string>(), kv.second["root_link"].as<std::string>(),
                              kv.second["tip_link"].as<std::string>()});
  }
  for (const auto& kv : urdf["tree_models"]) {
    cfg.tree_models.push_back({kv.first.as<std::string>(), kv.second["root_link"].as<std::string>(),
                               kv.second["tip_links"].as<std::vector<std::string>>()});
  }
  RobotCase rc{};
  rc.label = "g1_p1b";
  rc.config_rel = "g1_p1b/_base.yaml";
  cfg.extra_frames = ShippedExtraFrames(rc);
  return cfg;
}

// The search runs on `g1_catch` (pelvis → the right palm, 10 joints) and the
// segments on `g1` (17). What makes the two one problem: the catch frame is in
// both, fixed to the chain's tip, at the same place for the same joint angles —
// and in the tree it does not move with a joint the chain lacks.
TEST(CatchFrameModels, G1P1bTheSearchChainAndTheDeviceTreeCarryTheSameCatchFrame) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  const rub::ModelConfig cfg = ShippedG1Config();
  const rub::ExtraFrameConfig* cf = FindCatchFrame(cfg);
  ASSERT_NE(cf, nullptr) << "no urdf.extra_frames.catch_frame in g1_p1b/_base.yaml";
  const auto chain_cfg = std::find_if(cfg.sub_models.begin(), cfg.sub_models.end(),
                                      [](const auto& sm) { return sm.name == "g1_catch"; });
  ASSERT_NE(chain_cfg, cfg.sub_models.end()) << "no urdf.sub_models.g1_catch";
  // The search's end-effector frame is the chain's tip, and the catch frame
  // is declared on it (what a controller's configure checks).
  EXPECT_EQ(cf->parent, chain_cfg->tip_link);

  const rub::PinocchioModelBuilder builder(cfg);
  const std::shared_ptr<const pinocchio::Model> chain = builder.GetReducedModel("g1_catch");
  const std::shared_ptr<const pinocchio::Model> tree = builder.GetTreeModel("g1");
  ASSERT_TRUE(chain);
  ASSERT_TRUE(tree);
  EXPECT_EQ(chain->nv, 10);  // waist 3 + right arm 7
  EXPECT_EQ(tree->nv, 17);   // + left arm 7
  ASSERT_EQ(chain->nq, chain->nv);
  ASSERT_EQ(tree->nq, tree->nv);

  // The declared placement on the parent, in every derived model.
  const pinocchio::SE3 expected(pinocchio::rpy::rpyToMatrix(cf->rpy.x(), cf->rpy.y(), cf->rpy.z()),
                                cf->xyz);
  for (const auto& [label, model] :
       std::vector<std::pair<std::string, std::shared_ptr<const pinocchio::Model>>>{
           {"full", builder.GetFullModel()}, {"sub:g1_catch", chain}, {"tree:g1", tree}}) {
    ASSERT_TRUE(model->existFrame("catch_frame")) << label;
    ASSERT_TRUE(model->existFrame(cf->parent)) << label;
    pinocchio::Data data(*model);
    pinocchio::framesForwardKinematics(*model, data, pinocchio::neutral(*model));
    const pinocchio::SE3 rel =
        data.oMf[model->getFrameId(cf->parent)].actInv(data.oMf[model->getFrameId("catch_frame")]);
    EXPECT_TRUE(rel.isApprox(expected, 1e-12)) << label;
  }

  // Every joint of the chain is a joint of the tree; the tree's index of each.
  std::vector<int> tree_of_chain(static_cast<std::size_t>(chain->nv), -1);
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(chain->njoints);
       ++jid) {
    ASSERT_TRUE(tree->existJointName(chain->names[jid])) << chain->names[jid];
    tree_of_chain[static_cast<std::size_t>(chain->joints[jid].idx_v())] =
        tree->joints[tree->getJointId(chain->names[jid])].idx_v();
  }
  std::vector<bool> in_chain(static_cast<std::size_t>(tree->nv), false);
  for (const int t : tree_of_chain) {
    ASSERT_GE(t, 0);
    in_chain[static_cast<std::size_t>(t)] = true;
  }

  // The same joint angles: the same frame placement in the world, the same
  // Jacobian in the chain's joints — and zero columns for the others, whatever
  // angle THEY are at (the chain locks them at the model's reference pose).
  Eigen::VectorXd q_tree(tree->nv);
  for (int j = 0; j < tree->nv; ++j) {
    const double lo = tree->lowerPositionLimit[j];
    const double hi = tree->upperPositionLimit[j];
    q_tree[j] = lo + (hi - lo) * (0.3 + 0.4 * std::fabs(std::sin(1.7 * j + 0.4)));
  }
  Eigen::VectorXd q_chain(chain->nv);
  for (int m = 0; m < chain->nv; ++m) {
    q_chain[m] = q_tree[tree_of_chain[static_cast<std::size_t>(m)]];
  }
  pinocchio::Data d_tree(*tree);
  pinocchio::Data d_chain(*chain);
  const pinocchio::FrameIndex f_tree = tree->getFrameId("catch_frame");
  const pinocchio::FrameIndex f_chain = chain->getFrameId("catch_frame");
  Eigen::MatrixXd j_tree = Eigen::MatrixXd::Zero(6, tree->nv);
  Eigen::MatrixXd j_chain = Eigen::MatrixXd::Zero(6, chain->nv);
  pinocchio::computeFrameJacobian(*tree, d_tree, q_tree, f_tree, pinocchio::LOCAL_WORLD_ALIGNED,
                                  j_tree);
  pinocchio::computeFrameJacobian(*chain, d_chain, q_chain, f_chain, pinocchio::LOCAL_WORLD_ALIGNED,
                                  j_chain);
  pinocchio::framesForwardKinematics(*tree, d_tree, q_tree);
  pinocchio::framesForwardKinematics(*chain, d_chain, q_chain);
  EXPECT_TRUE(d_tree.oMf[f_tree].isApprox(d_chain.oMf[f_chain], 1e-12));
  for (int m = 0; m < chain->nv; ++m) {
    EXPECT_LT((j_tree.col(tree_of_chain[static_cast<std::size_t>(m)]) - j_chain.col(m)).norm(),
              1e-12)
        << chain->names[static_cast<std::size_t>(m) + 1];
    EXPECT_GT(j_chain.col(m).norm(), 1e-3) << "a chain joint that does not move the frame";
  }
  int outside = 0;
  for (int j = 0; j < tree->nv; ++j) {
    if (!in_chain[static_cast<std::size_t>(j)]) {
      ++outside;
      EXPECT_EQ(j_tree.col(j).norm(), 0.0) << "tree joint " << j << " moves the catch frame";
    }
  }
  EXPECT_EQ(outside, 7);  // the left arm
  RecordProperty("g1_p1b_search_chain_nv", chain->nv);
  RecordProperty("g1_p1b_segment_tree_nv", tree->nv);
}
