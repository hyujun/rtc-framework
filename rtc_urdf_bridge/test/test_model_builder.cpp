// ── PinocchioModelBuilder 테스트 ─────────────────────────────────────────────
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace rub = rtc_urdf_bridge;

static std::string TestUrdfPath(const std::string & filename)
{
  std::filesystem::path p(__FILE__);
  return (p.parent_path() / "urdf" / filename).string();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Serial arm — ModelConfig 직접 전달
// ═══════════════════════════════════════════════════════════════════════════════

class ModelBuilderSerialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("serial_6dof.urdf");
    cfg.root_joint_type = "fixed";
    cfg.sub_models.push_back({"arm", "base_link", "tool_link"});
    cfg.sub_models.push_back({"partial", "link_2", "link_5"});
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
  }
  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
};

TEST_F(ModelBuilderSerialTest, FullModelDimensions)
{
  auto model = builder_->GetFullModel();
  // 6 revolute joints → nq=6, nv=6
  EXPECT_EQ(model->nq, 6);
  EXPECT_EQ(model->nv, 6);
}

TEST_F(ModelBuilderSerialTest, FullSubModelMatchesFull)
{
  auto arm = builder_->GetReducedModel("arm");
  // arm: base_link → tool_link, 모든 6개 관절 포함 → nq=6
  EXPECT_EQ(arm->nq, 6);
  EXPECT_EQ(arm->nv, 6);
}

TEST_F(ModelBuilderSerialTest, PartialSubModelReduced)
{
  auto partial = builder_->GetReducedModel("partial");
  // partial: link_2 → link_5, 3개 관절 → nq=3
  EXPECT_EQ(partial->nq, 3);
  EXPECT_EQ(partial->nv, 3);
}

TEST_F(ModelBuilderSerialTest, SubModelNamesReturned)
{
  auto names = builder_->GetSubModelNames();
  EXPECT_EQ(names.size(), 2u);
}

TEST_F(ModelBuilderSerialTest, InvalidSubModelThrows)
{
  EXPECT_THROW(builder_->GetReducedModel("nonexistent"), std::out_of_range);
}

TEST_F(ModelBuilderSerialTest, AnalyzerAccessible)
{
  const auto & analyzer = builder_->GetAnalyzer();
  EXPECT_EQ(analyzer.GetRootLinkName(), "base_link");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tree hand — 트리모델 빌드
// ═══════════════════════════════════════════════════════════════════════════════

class ModelBuilderTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("tree_hand.urdf");
    cfg.root_joint_type = "fixed";

    rub::TreeModelConfig tc;
    tc.name = "hand";
    tc.root_link = "palm_link";
    tc.tip_links = {"thumb_tip", "index_tip", "middle_tip", "ring_tip"};
    cfg.tree_models.push_back(std::move(tc));

    cfg.sub_models.push_back({"thumb", "palm_link", "thumb_tip"});

    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
  }
  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
};

TEST_F(ModelBuilderTreeTest, FullModelDimensions)
{
  auto model = builder_->GetFullModel();
  // 10 revolute joints
  EXPECT_EQ(model->nq, 10);
  EXPECT_EQ(model->nv, 10);
}

TEST_F(ModelBuilderTreeTest, TreeModelIsFullHand)
{
  auto hand = builder_->GetTreeModel("hand");
  // 모든 10개 관절 포함 (잠글 관절 없음)
  EXPECT_EQ(hand->nq, 10);
}

TEST_F(ModelBuilderTreeTest, ThumbSubModel)
{
  auto thumb = builder_->GetReducedModel("thumb");
  // thumb: 3 joints
  EXPECT_EQ(thumb->nq, 3);
}

TEST_F(ModelBuilderTreeTest, InvalidTreeModelThrows)
{
  EXPECT_THROW(builder_->GetTreeModel("nonexistent"), std::out_of_range);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mimic arm — mimic 관절 잠금
// ═══════════════════════════════════════════════════════════════════════════════

class ModelBuilderMimicTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("arm_with_mimic.urdf");
    cfg.root_joint_type = "fixed";
    // finger_right_joint은 mimic → 자동으로 passive에 추가됨
    cfg.sub_models.push_back({"arm", "base_link", "link_5"});
    cfg.sub_models.push_back({"arm_gripper", "base_link", "finger_left"});
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
  }
  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
};

TEST_F(ModelBuilderMimicTest, FullModelHasMimicJoint)
{
  auto model = builder_->GetFullModel();
  // 5 arm + finger_left + finger_right = 7 nq (all prismatic/revolute = 1 each)
  EXPECT_EQ(model->nq, 7);
}

TEST_F(ModelBuilderMimicTest, ArmSubModelExcludesGripper)
{
  auto arm = builder_->GetReducedModel("arm");
  // arm: 5 joints (finger 관절은 체인 외부 → 잠금)
  EXPECT_EQ(arm->nq, 5);
}

TEST_F(ModelBuilderMimicTest, ArmGripperExcludesMimicJoint)
{
  auto arm_gripper = builder_->GetReducedModel("arm_gripper");
  // 5 arm + finger_left = 6, finger_right는 mimic→passive→잠금
  EXPECT_EQ(arm_gripper->nq, 6);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 에러 케이스
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ModelBuilderErrorTest, EmptyConfigThrows)
{
  rub::ModelConfig cfg;  // urdf_path도 xml도 없음
  EXPECT_THROW({ rub::PinocchioModelBuilder builder(cfg); }, std::runtime_error);
}

TEST(ModelBuilderErrorTest, BadUrdfPathThrows)
{
  rub::ModelConfig cfg;
  cfg.urdf_path = "/nonexistent/path/robot.urdf";
  EXPECT_THROW({ rub::PinocchioModelBuilder builder(cfg); }, std::runtime_error);
}
