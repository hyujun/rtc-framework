// ── KinematicChainExtractor 테스트 ───────────────────────────────────────────
#include "rtc_urdf_bridge/kinematic_chain_extractor.hpp"
#include "rtc_urdf_bridge/urdf_analyzer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>

namespace rub = rtc_urdf_bridge;

static std::string TestUrdfPath(const std::string & filename)
{
  std::filesystem::path p(__FILE__);
  return (p.parent_path() / "urdf" / filename).string();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Serial arm — 서브모델 추출
// ═══════════════════════════════════════════════════════════════════════════════

class ChainExtractorSerialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<rub::UrdfAnalyzer>(TestUrdfPath("serial_6dof.urdf"));
    extractor_ = std::make_unique<rub::KinematicChainExtractor>(*analyzer_);
  }
  std::unique_ptr<rub::UrdfAnalyzer> analyzer_;
  std::unique_ptr<rub::KinematicChainExtractor> extractor_;
};

TEST_F(ChainExtractorSerialTest, FullChainExtraction)
{
  auto sub = extractor_->ExtractSubModel("full", "base_link", "tool_link");
  EXPECT_EQ(sub.name, "full");
  EXPECT_EQ(sub.root_link, "base_link");
  EXPECT_EQ(sub.tip_link, "tool_link");
  // 6 actuated joints (tool_joint은 fixed이므로 제외)
  EXPECT_EQ(sub.joint_names.size(), 6u);
  // 7 all joints (tool_joint 포함)
  EXPECT_EQ(sub.all_joint_names.size(), 7u);
  // 8 links
  EXPECT_EQ(sub.link_names.size(), 8u);
}

TEST_F(ChainExtractorSerialTest, PartialChainExtraction)
{
  auto sub = extractor_->ExtractSubModel("partial", "link_2", "link_5");
  // link_2 → link_3 → link_4 → link_5: 3 actuated joints
  EXPECT_EQ(sub.joint_names.size(), 3u);
  EXPECT_EQ(sub.link_names.size(), 4u);
}

TEST_F(ChainExtractorSerialTest, JointsToLockPartial)
{
  auto sub = extractor_->ExtractSubModel("mid", "link_2", "link_5");
  auto to_lock = extractor_->ComputeJointsToLock(sub);
  // 전체 6개 중 3개 사용 → 3개 잠금
  EXPECT_EQ(to_lock.size(), 3u);
}

TEST_F(ChainExtractorSerialTest, JointsToLockFullChain)
{
  auto sub = extractor_->ExtractSubModel("full", "base_link", "tool_link");
  auto to_lock = extractor_->ComputeJointsToLock(sub);
  // 전체 6개 모두 사용 → 0개 잠금
  EXPECT_EQ(to_lock.size(), 0u);
}

TEST_F(ChainExtractorSerialTest, InvalidLinkThrows)
{
  EXPECT_THROW(
    extractor_->ExtractSubModel("bad", "base_link", "nonexistent"),
    std::out_of_range);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tree hand — 트리모델 추출
// ═══════════════════════════════════════════════════════════════════════════════

class ChainExtractorTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<rub::UrdfAnalyzer>(TestUrdfPath("tree_hand.urdf"));
    extractor_ = std::make_unique<rub::KinematicChainExtractor>(*analyzer_);
  }
  std::unique_ptr<rub::UrdfAnalyzer> analyzer_;
  std::unique_ptr<rub::KinematicChainExtractor> extractor_;
};

TEST_F(ChainExtractorTreeTest, FullTreeExtraction)
{
  std::vector<std::string> tips = {"thumb_tip", "index_tip", "middle_tip", "ring_tip"};
  auto tree = extractor_->ExtractTreeModel("hand", "palm_link", tips);

  EXPECT_EQ(tree.name, "hand");
  EXPECT_EQ(tree.root_link, "palm_link");
  EXPECT_EQ(tree.tip_links.size(), 4u);
  // thumb(3) + index(3) + middle(2) + ring(2) = 10 actuated joints
  EXPECT_EQ(tree.joint_names.size(), 10u);
}

TEST_F(ChainExtractorTreeTest, BranchingPointDetection)
{
  std::vector<std::string> tips = {"thumb_tip", "index_tip", "middle_tip", "ring_tip"};
  auto tree = extractor_->ExtractTreeModel("hand", "palm_link", tips);

  // palm_link에서 4개 finger가 분기 → palm_link가 분기점
  EXPECT_FALSE(tree.branching_points.empty());
  auto it = std::find(tree.branching_points.begin(), tree.branching_points.end(), "palm_link");
  EXPECT_NE(it, tree.branching_points.end()) << "palm_link가 분기점이어야 합니다";
}

TEST_F(ChainExtractorTreeTest, SingleFingerSubModel)
{
  auto sub = extractor_->ExtractSubModel("thumb", "palm_link", "thumb_tip");
  EXPECT_EQ(sub.joint_names.size(), 3u);  // thumb_joint_1, 2, 3
}

TEST_F(ChainExtractorTreeTest, JointsToLockSingleFinger)
{
  auto sub = extractor_->ExtractSubModel("thumb", "palm_link", "thumb_tip");
  auto to_lock = extractor_->ComputeJointsToLock(sub);
  // 전체 10개 중 thumb 3개 사용 → 7개 잠금
  EXPECT_EQ(to_lock.size(), 7u);
}

TEST_F(ChainExtractorTreeTest, InvalidTipThrows)
{
  std::vector<std::string> tips = {"thumb_tip", "nonexistent_tip"};
  EXPECT_THROW(
    extractor_->ExtractTreeModel("bad", "palm_link", tips),
    std::out_of_range);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mimic arm — 서브모델 추출
// ═══════════════════════════════════════════════════════════════════════════════

class ChainExtractorMimicTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<rub::UrdfAnalyzer>(TestUrdfPath("arm_with_mimic.urdf"));
    extractor_ = std::make_unique<rub::KinematicChainExtractor>(*analyzer_);
  }
  std::unique_ptr<rub::UrdfAnalyzer> analyzer_;
  std::unique_ptr<rub::KinematicChainExtractor> extractor_;
};

TEST_F(ChainExtractorMimicTest, ArmOnlySubModel)
{
  auto sub = extractor_->ExtractSubModel("arm", "base_link", "link_5");
  EXPECT_EQ(sub.joint_names.size(), 5u);
}

TEST_F(ChainExtractorMimicTest, ArmToFingerSubModel)
{
  auto sub = extractor_->ExtractSubModel("arm_finger", "base_link", "finger_left");
  // joint_1~5 + finger_left_joint = 6 actuated
  EXPECT_EQ(sub.joint_names.size(), 6u);
}
