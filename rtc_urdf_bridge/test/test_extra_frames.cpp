// ── ModelConfig::extra_frames (dynamic_catching S2.3a, D-10/D-17) ────────────
// The builder adds YAML-declared frames to the full model; every derived model
// (sub / tree / actuated) is reduced from it and must carry the frame at the
// same placement relative to its parent. Invalid declarations fail loudly.
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math/rpy.hpp>

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

namespace {

// four_bar_tree + sidecar: joint_a actuated, joint_ab/joint_c/joint_cd passive.
// link_b hangs off joint_ab, which the actuated model locks and the branch2
// tree (base_link → c2) does not contain — so the frame must be re-parented
// in those models and still sit at the same place.
rub::ModelConfig FourBarWithFrames() {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("four_bar_tree.urdf");
  cfg.closure_yaml_path = TestUrdfPath("four_bar.closure.yaml");
  cfg.sub_models.push_back({"branch1", "base_link", "c1"});
  cfg.tree_models.push_back({"branch2", "base_link", {"c2"}});

  rub::ExtraFrameConfig catch_frame;
  catch_frame.name = "catch_frame";
  catch_frame.parent = "link_b";
  catch_frame.xyz = Eigen::Vector3d(0.01, -0.02, 0.03);
  catch_frame.rpy = Eigen::Vector3d(M_PI, 0.2, -0.4);
  cfg.extra_frames.push_back(catch_frame);

  rub::ExtraFrameConfig tool;
  tool.name = "tool_offset";
  // c1 is the fixture's only frame with a non-identity joint-relative
  // placement (+0.3 m on joint_ab); a builder that drops the parent frame's
  // own placement would pass on every other parent.
  tool.parent = "c1";
  tool.xyz = Eigen::Vector3d(0.0, 0.0, 0.05);
  tool.rpy = Eigen::Vector3d(0.0, 0.5, 0.0);
  tool.provisional = false;
  cfg.extra_frames.push_back(tool);
  return cfg;
}

// Placement of `frame` relative to `parent` in `model` at a random q.
pinocchio::SE3 RelativePlacement(const pinocchio::Model& model, const std::string& parent,
                                 const std::string& frame) {
  pinocchio::Data data(model);
  const Eigen::VectorXd q = pinocchio::randomConfiguration(model, -Eigen::VectorXd::Ones(model.nq),
                                                           Eigen::VectorXd::Ones(model.nq));
  pinocchio::framesForwardKinematics(model, data, q);
  return data.oMf[model.getFrameId(parent)].actInv(data.oMf[model.getFrameId(frame)]);
}

}  // namespace

TEST(ExtraFrames, PresentInEveryModelWithTheSameParentRelativePlacement) {
  const rub::ModelConfig cfg = FourBarWithFrames();
  const rub::PinocchioModelBuilder builder(cfg);

  std::vector<std::pair<std::string, std::shared_ptr<const pinocchio::Model>>> models = {
      {"full", builder.GetFullModel()},
      {"sub", builder.GetReducedModel("branch1")},
      {"tree", builder.GetTreeModel("branch2")},
      {"actuated", builder.GetActuatedModel()},
  };
  for (const auto& ef : cfg.extra_frames) {
    const pinocchio::SE3 expected(pinocchio::rpy::rpyToMatrix(ef.rpy.x(), ef.rpy.y(), ef.rpy.z()),
                                  ef.xyz);
    for (const auto& [label, model] : models) {
      ASSERT_NE(model, nullptr) << label;
      ASSERT_TRUE(model->existFrame(ef.name)) << ef.name << " missing in " << label;
      EXPECT_EQ(model->frames[model->getFrameId(ef.name)].type, pinocchio::OP_FRAME);
      const pinocchio::SE3 rel = RelativePlacement(*model, ef.parent, ef.name);
      EXPECT_TRUE(rel.isApprox(expected, 1e-12)) << ef.name << " in " << label << ":\n"
                                                 << rel << "\nexpected\n"
                                                 << expected;
    }
  }
  // The re-parenting case really happened: joint_ab is locked in the actuated
  // model, so the frame there cannot hang off joint_ab.
  const auto actuated = builder.GetActuatedModel();
  EXPECT_FALSE(actuated->existJointName("joint_ab"));
}

// Frames are appended: every frame the URDF produced keeps its index, and
// the builder's inertias are untouched (the frame carries no inertia).
TEST(ExtraFrames, ExistingFrameIdsAndInertiasUnchanged) {
  rub::ModelConfig plain = FourBarWithFrames();
  plain.extra_frames.clear();
  const rub::PinocchioModelBuilder without(plain);
  const rub::PinocchioModelBuilder with(FourBarWithFrames());

  const auto& a = *without.GetFullModel();
  const auto& b = *with.GetFullModel();
  ASSERT_EQ(b.nframes, a.nframes + 2);
  for (int i = 0; i < a.nframes; ++i) {
    EXPECT_EQ(a.frames[static_cast<std::size_t>(i)].name,
              b.frames[static_cast<std::size_t>(i)].name)
        << i;
  }
  ASSERT_EQ(a.inertias.size(), b.inertias.size());
  for (std::size_t j = 0; j < a.inertias.size(); ++j) {
    EXPECT_TRUE(a.inertias[j].isApprox(b.inertias[j], 0.0)) << j;
  }
  EXPECT_EQ(a.nv, b.nv);
}

TEST(ExtraFrames, InvalidDeclarationsFailConstruction) {
  auto expect_throw = [](const std::string& why, auto mutate) {
    rub::ModelConfig cfg = FourBarWithFrames();
    mutate(cfg);
    EXPECT_THROW(rub::PinocchioModelBuilder{cfg}, std::runtime_error) << why;
  };
  expect_throw("missing parent", [](rub::ModelConfig& c) { c.extra_frames[0].parent = "nope"; });
  expect_throw("name of an existing link",
               [](rub::ModelConfig& c) { c.extra_frames[0].name = "link_a"; });
  expect_throw("duplicate within the list",
               [](rub::ModelConfig& c) { c.extra_frames[1].name = c.extra_frames[0].name; });
  expect_throw("empty name", [](rub::ModelConfig& c) { c.extra_frames[0].name.clear(); });
  expect_throw("NaN offset", [](rub::ModelConfig& c) {
    c.extra_frames[0].xyz.x() = std::numeric_limits<double>::quiet_NaN();
  });
  expect_throw("inf rpy", [](rub::ModelConfig& c) {
    c.extra_frames[0].rpy.z() = std::numeric_limits<double>::infinity();
  });
}

// ── LoadModelConfig: same map schema as the robot config ────────────────────

namespace {
std::string WriteTempYaml(const std::string& name, const std::string& content) {
  const std::filesystem::path p = std::filesystem::path(::testing::TempDir()) / name;
  std::ofstream(p) << content;
  return p.string();
}
}  // namespace

TEST(ExtraFrames, LoadModelConfigParsesMapSchema) {
  const std::string path = WriteTempYaml("rub_extra_frames.yaml", R"(
urdf_path: /abs/robot.urdf
extra_frames:
  catch_frame:
    parent: palm
    xyz: [0.0, 0.01, 0.02]
    rpy: [3.141592653589793, 0, 0]
  marker:
    parent: tool0
    xyz: [1, 2, 3]
    rpy: [0.0, 0.0, 0.0]
    provisional: false
)");
  const rub::ModelConfig cfg = rub::PinocchioModelBuilder::LoadModelConfig(path);
  ASSERT_EQ(cfg.extra_frames.size(), 2u);
  const rub::ExtraFrameConfig* c = nullptr;
  const rub::ExtraFrameConfig* m = nullptr;
  for (const auto& ef : cfg.extra_frames) {
    (ef.name == "catch_frame" ? c : m) = &ef;
  }
  ASSERT_NE(c, nullptr);
  ASSERT_NE(m, nullptr);
  EXPECT_EQ(c->parent, "palm");
  EXPECT_TRUE(c->xyz.isApprox(Eigen::Vector3d(0.0, 0.01, 0.02)));
  EXPECT_DOUBLE_EQ(c->rpy.x(), M_PI);
  EXPECT_TRUE(c->provisional) << "provisional defaults to true (fail-closed)";
  EXPECT_EQ(m->parent, "tool0");
  EXPECT_TRUE(m->xyz.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0))) << "integer lists accepted";
  EXPECT_FALSE(m->provisional);
}

TEST(ExtraFrames, LoadModelConfigRejectsIncompleteEntries) {
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"no_parent", "extra_frames:\n  f:\n    xyz: [0, 0, 0]\n    rpy: [0, 0, 0]\n"},
      {"no_xyz", "extra_frames:\n  f:\n    parent: a\n    rpy: [0, 0, 0]\n"},
      {"short_rpy", "extra_frames:\n  f:\n    parent: a\n    xyz: [0, 0, 0]\n    rpy: [0, 0]\n"},
      {"list_form", "extra_frames:\n  - {name: f, parent: a, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n"},
  };
  for (const auto& [name, body] : bad) {
    const std::string path = WriteTempYaml("rub_extra_bad_" + name + ".yaml", body);
    EXPECT_THROW((void)rub::PinocchioModelBuilder::LoadModelConfig(path), std::exception) << name;
  }
}
