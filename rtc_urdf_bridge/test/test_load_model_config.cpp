// ── PinocchioModelBuilder::LoadModelConfig + YAML-path ctor + closed-chain ──
// Covers the YAML→ModelConfig parser (all field families), the std::string_view
// constructor that drives it, and RegisterClosedChainConstraints /
// MakeReferenceConfig build paths that no other test exercises.
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace rub = rtc_urdf_bridge;

using rtc::test::TestUrdfPath;

static std::string WriteTempYaml(const std::string& name, const std::string& content) {
  std::filesystem::path p = std::filesystem::path(::testing::TempDir()) / name;
  std::ofstream(p) << content;
  return p.string();
}

// ═══════════════════════════════════════════════════════════════════════════════
// LoadModelConfig — 전 필드 파싱
// ═══════════════════════════════════════════════════════════════════════════════

TEST(LoadModelConfig, ParsesAllFields) {
  const std::string yaml = R"(
urdf_path: /abs/robot.urdf
xacro_args:
  hand: leap
  side: right
root_joint_type: floating
sub_models:
  - {name: arm, root_link: base_link, tip_link: tool_link}
tree_models:
  - {name: hand, root_link: palm, tip_links: [t1, t2, t3]}
closed_chains:
  - name: loop1
    link_a: link_b
    link_b: link_c
    contact_type: 3D
    offset_a: {xyz: [1.0, 2.0, 3.0], rpy: [0.1, 0.2, 0.3]}
    offset_b: {xyz: [4.0, 5.0, 6.0], rpy: [0.4, 0.5, 0.6]}
    baumgarte_kp: 10.0
    baumgarte_kd: 1.0
passive_joints: [pj1, pj2]
yaml_passive_override: true
lock_reference_config:
  joint_1: 0.5
  joint_2: -0.3
)";
  std::string path = WriteTempYaml("rub_full.yaml", yaml);
  auto cfg = rub::PinocchioModelBuilder::LoadModelConfig(path);

  // absolute urdf_path → unchanged
  EXPECT_EQ(cfg.urdf_path, "/abs/robot.urdf");

  EXPECT_EQ(cfg.xacro_args.size(), 2u);
  EXPECT_EQ(cfg.xacro_args.at("hand"), "leap");
  EXPECT_EQ(cfg.xacro_args.at("side"), "right");

  EXPECT_EQ(cfg.root_joint_type, "floating");

  ASSERT_EQ(cfg.sub_models.size(), 1u);
  EXPECT_EQ(cfg.sub_models[0].name, "arm");
  EXPECT_EQ(cfg.sub_models[0].root_link, "base_link");
  EXPECT_EQ(cfg.sub_models[0].tip_link, "tool_link");

  ASSERT_EQ(cfg.tree_models.size(), 1u);
  EXPECT_EQ(cfg.tree_models[0].name, "hand");
  EXPECT_EQ(cfg.tree_models[0].tip_links.size(), 3u);
  EXPECT_EQ(cfg.tree_models[0].tip_links[2], "t3");

  ASSERT_EQ(cfg.closed_chains.size(), 1u);
  const auto& cc = cfg.closed_chains[0];
  EXPECT_EQ(cc.name, "loop1");
  EXPECT_EQ(cc.link_a, "link_b");
  EXPECT_EQ(cc.link_b, "link_c");
  EXPECT_FALSE(cc.is_6d);  // contact_type "3D" → CONTACT_3D
  EXPECT_DOUBLE_EQ(cc.offset_a_xyz.x(), 1.0);
  EXPECT_DOUBLE_EQ(cc.offset_a_xyz.z(), 3.0);
  EXPECT_DOUBLE_EQ(cc.offset_a_rpy.y(), 0.2);
  EXPECT_DOUBLE_EQ(cc.offset_b_xyz.y(), 5.0);
  EXPECT_DOUBLE_EQ(cc.offset_b_rpy.z(), 0.6);
  EXPECT_DOUBLE_EQ(cc.baumgarte_kp, 10.0);
  EXPECT_DOUBLE_EQ(cc.baumgarte_kd, 1.0);

  ASSERT_EQ(cfg.passive_joints.size(), 2u);
  EXPECT_EQ(cfg.passive_joints[1], "pj2");
  EXPECT_TRUE(cfg.yaml_passive_override);

  EXPECT_EQ(cfg.lock_reference_config.size(), 2u);
  EXPECT_DOUBLE_EQ(cfg.lock_reference_config.at("joint_1"), 0.5);
  EXPECT_DOUBLE_EQ(cfg.lock_reference_config.at("joint_2"), -0.3);
}

TEST(LoadModelConfig, MinimalYamlUsesDefaults) {
  std::string path = WriteTempYaml("rub_min.yaml", "urdf_path: /abs/only.urdf\n");
  auto cfg = rub::PinocchioModelBuilder::LoadModelConfig(path);

  EXPECT_EQ(cfg.urdf_path, "/abs/only.urdf");
  EXPECT_EQ(cfg.root_joint_type, "fixed");  // default
  EXPECT_TRUE(cfg.sub_models.empty());
  EXPECT_TRUE(cfg.tree_models.empty());
  EXPECT_TRUE(cfg.closed_chains.empty());
  EXPECT_TRUE(cfg.passive_joints.empty());
  EXPECT_FALSE(cfg.yaml_passive_override);
  EXPECT_TRUE(cfg.lock_reference_config.empty());
}

TEST(LoadModelConfig, RelativeUrdfPathResolvedAgainstYamlDir) {
  std::string path = WriteTempYaml("rub_rel.yaml", "urdf_path: sub/robot.urdf\n");
  auto cfg = rub::PinocchioModelBuilder::LoadModelConfig(path);

  std::filesystem::path expected =
      std::filesystem::path(::testing::TempDir()) / "sub" / "robot.urdf";
  EXPECT_EQ(cfg.urdf_path, expected.string());
}

TEST(LoadModelConfig, ContactTypeDefaultsTo6D) {
  // contact_type absent AND contact_type "6D" both → is_6d true.
  const std::string yaml = R"(
urdf_path: /abs/r.urdf
closed_chains:
  - {name: a, link_a: x, link_b: y}
  - {name: b, link_a: x, link_b: y, contact_type: 6D}
)";
  std::string path = WriteTempYaml("rub_contact.yaml", yaml);
  auto cfg = rub::PinocchioModelBuilder::LoadModelConfig(path);

  ASSERT_EQ(cfg.closed_chains.size(), 2u);
  EXPECT_TRUE(cfg.closed_chains[0].is_6d);  // absent → default true
  EXPECT_TRUE(cfg.closed_chains[1].is_6d);  // "6D" != "3D" → true
}

TEST(LoadModelConfig, MissingFileThrows) {
  EXPECT_THROW(rub::PinocchioModelBuilder::LoadModelConfig("/no/such/file.yaml"),
               std::exception);
}

// ═══════════════════════════════════════════════════════════════════════════════
// YAML-path 생성자 (string_view ctor) — end-to-end
// ═══════════════════════════════════════════════════════════════════════════════

TEST(YamlPathConstructor, BuildsModelFromYaml) {
  const std::string yaml = "urdf_path: " + TestUrdfPath("serial_6dof.urdf") +
                           "\nroot_joint_type: fixed\n"
                           "sub_models:\n  - {name: arm, root_link: base_link, tip_link: tool_link}\n";
  std::string path = WriteTempYaml("rub_serial.yaml", yaml);

  rub::PinocchioModelBuilder builder(std::string_view{path});
  EXPECT_EQ(builder.GetFullModel()->nq, 6);
  EXPECT_EQ(builder.GetConfig().sub_models.size(), 1u);
  EXPECT_EQ(builder.GetReducedModel("arm")->nq, 6);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 폐쇄 체인 제약 등록 (RegisterClosedChainConstraints)
// ═══════════════════════════════════════════════════════════════════════════════

// four_bar.urdf's loop links are not part of Pinocchio's spanning tree, so the
// constraint is registered between two real bodies of the serial arm instead —
// geometrically artificial but sufficient to exercise the registration path.
TEST(ClosedChainRegistration, RegistersConstraintForValidLinks) {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("serial_6dof.urdf");
  cfg.root_joint_type = "fixed";
  rub::ClosedChainInfo cc;
  cc.name = "artificial_loop";
  cc.link_a = "link_2";
  cc.link_b = "link_4";
  cc.is_6d = true;
  cc.baumgarte_kp = 10.0;
  cc.baumgarte_kd = 1.0;
  cfg.closed_chains.push_back(cc);

  rub::PinocchioModelBuilder builder(cfg);
  EXPECT_EQ(builder.GetConstraintModels().size(), 1u);
}

TEST(ClosedChainRegistration, ThrowsForUnknownLink) {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("serial_6dof.urdf");
  cfg.root_joint_type = "fixed";
  rub::ClosedChainInfo cc;
  cc.name = "bad";
  cc.link_a = "nonexistent_a";
  cc.link_b = "link_2";
  cfg.closed_chains.push_back(cc);

  EXPECT_THROW({ rub::PinocchioModelBuilder builder(cfg); }, std::runtime_error);
}

// ═══════════════════════════════════════════════════════════════════════════════
// lock_reference_config (MakeReferenceConfig) — 잠금 관절 기준값
// ═══════════════════════════════════════════════════════════════════════════════

TEST(LockReferenceConfig, BuildsReducedModelWithReferenceOverride) {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("serial_6dof.urdf");
  cfg.root_joint_type = "fixed";
  // "partial" locks joint_1 and joint_6 (outside link_2 → link_5).
  cfg.sub_models.push_back({"partial", "link_2", "link_5"});
  cfg.lock_reference_config["joint_1"] = 0.5;       // valid joint → override branch
  cfg.lock_reference_config["does_not_exist"] = 1.0;  // missing → skip branch

  rub::PinocchioModelBuilder builder(cfg);
  EXPECT_EQ(builder.GetReducedModel("partial")->nq, 3);
}
