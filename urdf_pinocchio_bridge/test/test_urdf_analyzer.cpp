// ── UrdfAnalyzer 테스트 ──────────────────────────────────────────────────────
#include "urdf_pinocchio_bridge/urdf_analyzer.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace upb = urdf_pinocchio_bridge;

// 테스트 URDF 경로 헬퍼
static std::string TestUrdfPath(const std::string & filename)
{
  // 빌드 디렉토리에서 소스 위치 추정
  std::filesystem::path p(__FILE__);
  return (p.parent_path() / "urdf" / filename).string();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Serial 6-DoF arm 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class UrdfAnalyzerSerialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<upb::UrdfAnalyzer>(TestUrdfPath("serial_6dof.urdf"));
  }
  std::unique_ptr<upb::UrdfAnalyzer> analyzer_;
};

TEST_F(UrdfAnalyzerSerialTest, RootLinkDetection)
{
  EXPECT_EQ(analyzer_->GetRootLinkName(), "base_link");
  EXPECT_GE(analyzer_->GetRootIndex(), 0);
}

TEST_F(UrdfAnalyzerSerialTest, LinkCount)
{
  // base + link_1~6 + tool_link = 8
  EXPECT_EQ(analyzer_->GetNumLinks(), 8u);
}

TEST_F(UrdfAnalyzerSerialTest, JointCount)
{
  // joint_1~6 + tool_joint = 7
  EXPECT_EQ(analyzer_->GetNumJoints(), 7u);
}

TEST_F(UrdfAnalyzerSerialTest, ActuatedJoints)
{
  const auto & actuated = analyzer_->GetActuatedJointNames();
  EXPECT_EQ(actuated.size(), 6u);
  // 모든 revolute 관절이 actuated
  for (const auto & j : actuated) {
    EXPECT_EQ(analyzer_->GetJointType(j), upb::UrdfJointType::kRevolute);
  }
}

TEST_F(UrdfAnalyzerSerialTest, FixedJoints)
{
  const auto & fixed = analyzer_->GetFixedJointNames();
  EXPECT_EQ(fixed.size(), 1u);
  EXPECT_EQ(fixed[0], "tool_joint");
}

TEST_F(UrdfAnalyzerSerialTest, NoPassiveJoints)
{
  // 모든 관절에 transmission 있음
  EXPECT_TRUE(analyzer_->GetPassiveJoints().empty());
}

TEST_F(UrdfAnalyzerSerialTest, NoMimicJoints)
{
  EXPECT_TRUE(analyzer_->GetMimicJoints().empty());
}

TEST_F(UrdfAnalyzerSerialTest, FindPathBaseToTool)
{
  auto path = analyzer_->FindPath("base_link", "tool_link");
  // base → link_1 → ... → link_6 → tool_link = 8 링크
  EXPECT_EQ(path.size(), 8u);
  // 시작은 base_link
  EXPECT_EQ(analyzer_->GetLinkNodes()[static_cast<std::size_t>(path.front())].link_name,
            "base_link");
  // 끝은 tool_link
  EXPECT_EQ(analyzer_->GetLinkNodes()[static_cast<std::size_t>(path.back())].link_name,
            "tool_link");
}

TEST_F(UrdfAnalyzerSerialTest, FindPathPartial)
{
  auto path = analyzer_->FindPath("link_2", "link_5");
  // link_2 → link_3 → link_4 → link_5 = 4 링크
  EXPECT_EQ(path.size(), 4u);
}

TEST_F(UrdfAnalyzerSerialTest, LCA)
{
  // serial chain에서 LCA(link_2, link_5) = link_2
  int lca = analyzer_->FindLCA("link_2", "link_5");
  EXPECT_EQ(analyzer_->GetLinkNodes()[static_cast<std::size_t>(lca)].link_name, "link_2");
}

TEST_F(UrdfAnalyzerSerialTest, JointMeta)
{
  const auto & meta = analyzer_->GetJointMeta("joint_3");
  EXPECT_EQ(meta.parent_link, "link_2");
  EXPECT_EQ(meta.child_link, "link_3");
  EXPECT_EQ(meta.type, upb::UrdfJointType::kRevolute);
  EXPECT_NEAR(meta.upper, 3.14159, 1e-4);
}

TEST_F(UrdfAnalyzerSerialTest, InvalidLinkThrows)
{
  EXPECT_THROW((void)analyzer_->GetLinkIndex("nonexistent_link"), std::out_of_range);
}

TEST_F(UrdfAnalyzerSerialTest, InvalidJointThrows)
{
  EXPECT_THROW((void)analyzer_->GetJointType("nonexistent_joint"), std::out_of_range);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tree hand 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class UrdfAnalyzerTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<upb::UrdfAnalyzer>(TestUrdfPath("tree_hand.urdf"));
  }
  std::unique_ptr<upb::UrdfAnalyzer> analyzer_;
};

TEST_F(UrdfAnalyzerTreeTest, RootLink)
{
  EXPECT_EQ(analyzer_->GetRootLinkName(), "palm_link");
}

TEST_F(UrdfAnalyzerTreeTest, ActuatedJointCount)
{
  // thumb(3) + index(3) + middle(2) + ring(2) = 10
  EXPECT_EQ(analyzer_->GetActuatedJointNames().size(), 10u);
}

TEST_F(UrdfAnalyzerTreeTest, LCABetweenFingerTips)
{
  // thumb_tip과 index_tip의 LCA는 palm_link
  int lca = analyzer_->FindLCA("thumb_tip", "index_tip");
  EXPECT_EQ(analyzer_->GetLinkNodes()[static_cast<std::size_t>(lca)].link_name, "palm_link");
}

TEST_F(UrdfAnalyzerTreeTest, PathPalmToThumb)
{
  auto path = analyzer_->FindPath("palm_link", "thumb_tip");
  // palm → thumb_proximal → thumb_middle → thumb_tip = 4
  EXPECT_EQ(path.size(), 4u);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Four-bar linkage 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class UrdfAnalyzerFourBarTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<upb::UrdfAnalyzer>(TestUrdfPath("four_bar.urdf"));
  }
  std::unique_ptr<upb::UrdfAnalyzer> analyzer_;
};

TEST_F(UrdfAnalyzerFourBarTest, ClosedChainDetection)
{
  auto chains = analyzer_->DetectClosedChains();
  // <loop_joint> 태그 + heuristic (loop_closure_joint)
  EXPECT_GE(chains.size(), 1u);
}

TEST_F(UrdfAnalyzerFourBarTest, ClosedChainFromCustomTag)
{
  auto chains = analyzer_->DetectClosedChains();
  bool found_tag = false;
  for (const auto & cc : chains) {
    if (cc.name == "four_bar_loop") {
      found_tag = true;
      EXPECT_EQ(cc.link_a, "link_b");
      EXPECT_EQ(cc.link_b, "link_c");
      EXPECT_TRUE(cc.is_6d);
    }
  }
  EXPECT_TRUE(found_tag) << "<loop_joint> 태그에서 폐쇄 체인이 감지되어야 합니다";
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mimic gripper 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class UrdfAnalyzerMimicTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<upb::UrdfAnalyzer>(TestUrdfPath("arm_with_mimic.urdf"));
  }
  std::unique_ptr<upb::UrdfAnalyzer> analyzer_;
};

TEST_F(UrdfAnalyzerMimicTest, MimicJointDetection)
{
  const auto & mimics = analyzer_->GetMimicJoints();
  ASSERT_EQ(mimics.size(), 1u);
  EXPECT_EQ(mimics[0].joint_name, "finger_right_joint");
  EXPECT_EQ(mimics[0].mimicked_joint, "finger_left_joint");
  EXPECT_DOUBLE_EQ(mimics[0].multiplier, -1.0);
  EXPECT_DOUBLE_EQ(mimics[0].offset, 0.0);
}

TEST_F(UrdfAnalyzerMimicTest, PassiveVsActuated)
{
  // finger_right_joint: mimic이므로 passive에는 포함되지 않음 (mimic은 별도)
  // transmission 없는 non-mimic non-fixed 관절 → passive
  // 여기서는 모든 non-mimic 관절에 transmission이 있으므로 passive = 0
  EXPECT_TRUE(analyzer_->GetPassiveJoints().empty());
}

TEST_F(UrdfAnalyzerMimicTest, JointTypes)
{
  EXPECT_EQ(analyzer_->GetJointType("finger_left_joint"), upb::UrdfJointType::kPrismatic);
  EXPECT_EQ(analyzer_->GetJointType("finger_right_joint"), upb::UrdfJointType::kPrismatic);
}

// ═══════════════════════════════════════════════════════════════════════════════
// XML 문자열 입력 테스트
// ═══════════════════════════════════════════════════════════════════════════════

TEST(UrdfAnalyzerXmlTest, FromXmlString)
{
  const char * xml = R"(
    <robot name="tiny">
      <link name="world"/>
      <link name="body">
        <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
      </link>
      <joint name="j1" type="revolute">
        <parent link="world"/><child link="body"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>
  )";

  upb::UrdfAnalyzer analyzer(xml, upb::UrdfAnalyzer::FromXmlTag{});
  EXPECT_EQ(analyzer.GetRootLinkName(), "world");
  EXPECT_EQ(analyzer.GetNumLinks(), 2u);
  EXPECT_EQ(analyzer.GetActuatedJointNames().size(), 1u);
}

TEST(UrdfAnalyzerXmlTest, EmptyXmlThrows)
{
  EXPECT_THROW(
    upb::UrdfAnalyzer("", upb::UrdfAnalyzer::FromXmlTag{}),
    std::runtime_error);
}

TEST(UrdfAnalyzerXmlTest, MalformedXmlThrows)
{
  EXPECT_THROW(
    upb::UrdfAnalyzer("<robot><broken", upb::UrdfAnalyzer::FromXmlTag{}),
    std::runtime_error);
}
