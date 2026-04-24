// ── Joint classification 테스트 (JointRole / PassiveSubtype) ─────────────────
#include "rtc_urdf_bridge/urdf_analyzer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

std::string TestUrdfPath(const std::string &filename) {
  std::filesystem::path p(__FILE__);
  return (p.parent_path() / "urdf" / filename).string();
}

bool Contains(const std::vector<std::string> &vec, const std::string &name) {
  return std::find(vec.begin(), vec.end(), name) != vec.end();
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// Serial arm — 모든 non-fixed 관절은 active, fixed는 fixed
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointClassificationSerial, AllNonFixedAreActive) {
  rub::UrdfAnalyzer analyzer(TestUrdfPath("serial_6dof.urdf"));

  EXPECT_EQ(analyzer.GetActiveJointNames().size(), 6u);
  EXPECT_TRUE(analyzer.GetPassiveJointNames().empty());
  EXPECT_EQ(analyzer.GetFixedJointNames().size(), 1u);
  EXPECT_EQ(analyzer.GetNonFixedJointNames().size(), 6u);

  for (const auto &j : analyzer.GetActiveJointNames()) {
    const auto &meta = analyzer.GetJointMeta(j);
    EXPECT_EQ(meta.role, rub::JointRole::kActive);
    EXPECT_EQ(meta.passive_subtype, rub::PassiveSubtype::kNone);
    EXPECT_TRUE(meta.has_physics);
    EXPECT_TRUE(meta.has_limit_tag);
  }
}

TEST(JointClassificationSerial, FixedJointRole) {
  rub::UrdfAnalyzer analyzer(TestUrdfPath("serial_6dof.urdf"));
  EXPECT_EQ(analyzer.GetJointRole("tool_joint"), rub::JointRole::kFixed);
  EXPECT_EQ(analyzer.GetPassiveSubtype("tool_joint"),
            rub::PassiveSubtype::kNone);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mimic — slave joint은 kPassive/kMimic
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointClassificationMimic, MimicIsPassiveMimic) {
  rub::UrdfAnalyzer analyzer(TestUrdfPath("arm_with_mimic.urdf"));

  EXPECT_EQ(analyzer.GetJointRole("finger_right_joint"),
            rub::JointRole::kPassive);
  EXPECT_EQ(analyzer.GetPassiveSubtype("finger_right_joint"),
            rub::PassiveSubtype::kMimic);
  EXPECT_EQ(analyzer.GetJointRole("finger_left_joint"),
            rub::JointRole::kActive);

  const auto mimics =
      analyzer.GetPassiveJointNamesOfSubtype(rub::PassiveSubtype::kMimic);
  ASSERT_EQ(mimics.size(), 1u);
  EXPECT_EQ(mimics[0], "finger_right_joint");

  // active 목록에는 mimic이 없어야 함
  EXPECT_FALSE(Contains(analyzer.GetActiveJointNames(), "finger_right_joint"));
  // passive 목록에는 mimic이 있어야 함
  EXPECT_TRUE(Contains(analyzer.GetPassiveJointNames(), "finger_right_joint"));
  // non-fixed에는 둘 다 있어야 함
  EXPECT_TRUE(Contains(analyzer.GetNonFixedJointNames(), "finger_right_joint"));
  EXPECT_TRUE(Contains(analyzer.GetNonFixedJointNames(), "finger_left_joint"));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Closed-chain — 4-bar linkage
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointClassificationClosedChain, FourBarLoopJointsArePassive) {
  rub::UrdfAnalyzer analyzer(TestUrdfPath("four_bar.urdf"));

  // 4-bar: joint_a, joint_ab, joint_c 모두 closed-chain 경로(link_b↔link_c)에
  // 포함되므로 kPassive/kClosedChain으로 분류된다.
  const auto closed =
      analyzer.GetPassiveJointNamesOfSubtype(rub::PassiveSubtype::kClosedChain);
  EXPECT_GE(closed.size(), 1u);

  for (const auto &j : closed) {
    EXPECT_EQ(analyzer.GetJointRole(j), rub::JointRole::kPassive);
    EXPECT_EQ(analyzer.GetPassiveSubtype(j), rub::PassiveSubtype::kClosedChain);
  }

  // loop_closure_joint은 fixed 타입이므로 분류상 kFixed
  EXPECT_EQ(analyzer.GetJointRole("loop_closure_joint"),
            rub::JointRole::kFixed);
}

// ═══════════════════════════════════════════════════════════════════════════════
// YAML hint — hint로 지정된 관절은 kPassive/kFree (closed-chain 없는 경우)
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointClassificationHint, HintMakesJointPassiveFree) {
  rub::UrdfAnalyzer analyzer(TestUrdfPath("serial_6dof.urdf"),
                             std::vector<std::string>{"joint_3", "joint_5"});

  EXPECT_EQ(analyzer.GetJointRole("joint_3"), rub::JointRole::kPassive);
  EXPECT_EQ(analyzer.GetPassiveSubtype("joint_3"), rub::PassiveSubtype::kFree);
  EXPECT_EQ(analyzer.GetJointRole("joint_5"), rub::JointRole::kPassive);
  EXPECT_EQ(analyzer.GetPassiveSubtype("joint_5"), rub::PassiveSubtype::kFree);

  // 나머지는 여전히 active
  EXPECT_EQ(analyzer.GetActiveJointNames().size(), 4u);
  EXPECT_FALSE(Contains(analyzer.GetActiveJointNames(), "joint_3"));
  EXPECT_FALSE(Contains(analyzer.GetActiveJointNames(), "joint_5"));
}

TEST(JointClassificationHint, UnknownHintNameIsIgnored) {
  // 존재하지 않는 관절 이름은 경고만 내고 skip — 예외 없음
  EXPECT_NO_THROW(
      (void)rub::UrdfAnalyzer(TestUrdfPath("serial_6dof.urdf"),
                              std::vector<std::string>{"no_such_joint"}));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Physics 판정 — <limit effort> 없으면 active이지만 warning (예외는 아님)
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointClassificationPhysics, MissingLimitClassifiedActiveNoThrow) {
  // <limit> 태그 아예 없는 URDF — analyzer 생성은 성공, joint는 active,
  // has_physics == false
  const char *xml = R"(
    <robot name="no_limit">
      <link name="world"/>
      <link name="body"/>
      <joint name="j1" type="revolute">
        <parent link="world"/><child link="body"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
  )";
  EXPECT_NO_THROW(
      (void)rub::UrdfAnalyzer(xml, rub::UrdfAnalyzer::FromXmlTag{}));

  rub::UrdfAnalyzer analyzer(xml, rub::UrdfAnalyzer::FromXmlTag{});
  EXPECT_EQ(analyzer.GetJointRole("j1"), rub::JointRole::kActive);
  const auto &meta = analyzer.GetJointMeta("j1");
  EXPECT_FALSE(meta.has_physics);
  EXPECT_FALSE(meta.has_limit_tag);
}

TEST(JointClassificationPhysics, ZeroEffortFailsPhysics) {
  const char *xml = R"(
    <robot name="zero_effort">
      <link name="world"/>
      <link name="body"/>
      <joint name="j1" type="revolute">
        <parent link="world"/><child link="body"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="0" velocity="1"/>
      </joint>
    </robot>
  )";
  rub::UrdfAnalyzer analyzer(xml, rub::UrdfAnalyzer::FromXmlTag{});
  const auto &meta = analyzer.GetJointMeta("j1");
  EXPECT_TRUE(meta.has_limit_tag);
  EXPECT_FALSE(meta.has_physics);
  // 분류는 여전히 active (warning만)
  EXPECT_EQ(meta.role, rub::JointRole::kActive);
}

TEST(JointClassificationPhysics, ContinuousSkipsVelocityCheck) {
  // continuous는 position limit 없이도 physics 인정 (effort>0만 요구)
  const char *xml = R"(
    <robot name="continuous">
      <link name="world"/>
      <link name="body"/>
      <joint name="j1" type="continuous">
        <parent link="world"/><child link="body"/>
        <axis xyz="0 0 1"/>
        <limit effort="10" velocity="1"/>
      </joint>
    </robot>
  )";
  rub::UrdfAnalyzer analyzer(xml, rub::UrdfAnalyzer::FromXmlTag{});
  const auto &meta = analyzer.GetJointMeta("j1");
  EXPECT_TRUE(meta.has_physics);
  EXPECT_EQ(meta.role, rub::JointRole::kActive);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Enum string 변환
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointClassificationStrings, RoleAndSubtypeToString) {
  EXPECT_STREQ(rub::JointRoleToString(rub::JointRole::kActive), "active");
  EXPECT_STREQ(rub::JointRoleToString(rub::JointRole::kPassive), "passive");
  EXPECT_STREQ(rub::JointRoleToString(rub::JointRole::kFixed), "fixed");
  EXPECT_STREQ(rub::PassiveSubtypeToString(rub::PassiveSubtype::kMimic),
               "mimic");
  EXPECT_STREQ(rub::PassiveSubtypeToString(rub::PassiveSubtype::kClosedChain),
               "closed_chain");
  EXPECT_STREQ(rub::PassiveSubtypeToString(rub::PassiveSubtype::kFree), "free");
  EXPECT_STREQ(rub::PassiveSubtypeToString(rub::PassiveSubtype::kNone), "none");
}
