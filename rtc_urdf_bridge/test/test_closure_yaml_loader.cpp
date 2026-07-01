// ── test_closure_yaml_loader — Extended-URDF sidecar 파서 (Phase 2) ──────────
#include "rtc_urdf_bridge/closure_yaml_loader.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

// ── frame-first 스키마 정확히 파싱 ────────────────────────────────────────────
TEST(ClosureYamlLoader, ParsesFrameFirstSchema) {
  const std::string yaml = R"(
name: four_bar
closures:
  - name: fourbar_loop_0
    type: contact_3d
    frame_1: c1
    frame_2: c2
    reference_frame: LOCAL
    baumgarte: { Kp: 20.0, Kd: 5.0 }
actuation:
  actuated_joints: [joint_a]
joint_substitutions: []
)";
  const rub::ClosureSpec spec = rub::ParseClosureYaml(yaml);

  EXPECT_EQ(spec.name, "four_bar");
  ASSERT_EQ(spec.closures.size(), 1u);

  const auto& cc = spec.closures.front();
  EXPECT_EQ(cc.name, "fourbar_loop_0");
  EXPECT_FALSE(cc.is_6d);  // contact_3d (기본)
  EXPECT_EQ(cc.Type(), rub::LoopConstraintType::kContact3d);
  EXPECT_TRUE(cc.UsesFrameRefs());
  EXPECT_EQ(cc.frame_1, "c1");
  EXPECT_EQ(cc.frame_2, "c2");
  EXPECT_EQ(cc.reference_frame, rub::LoopReferenceFrame::kLocal);
  EXPECT_DOUBLE_EQ(cc.baumgarte_kp, 20.0);
  EXPECT_DOUBLE_EQ(cc.baumgarte_kd, 5.0);

  ASSERT_EQ(spec.actuated_joints.size(), 1u);
  EXPECT_EQ(spec.actuated_joints.front(), "joint_a");
}

// ── link+origin fallback 스키마 ───────────────────────────────────────────────
TEST(ClosureYamlLoader, ParsesLinkOriginFallback) {
  const std::string yaml = R"(
name: fb
closures:
  - name: loop0
    type: contact_6d
    parent_link: link_a
    child_link: link_b
    parent_origin: { xyz: [0.03, 0, 0], rpy: [0, 0, 0] }
    child_origin:  { xyz: [0.04, 0, 0.01], rpy: [0, 1.5707963, 0] }
    reference_frame: LOCAL_WORLD_ALIGNED
)";
  const rub::ClosureSpec spec = rub::ParseClosureYaml(yaml);
  ASSERT_EQ(spec.closures.size(), 1u);
  const auto& cc = spec.closures.front();

  EXPECT_TRUE(cc.is_6d);
  EXPECT_FALSE(cc.UsesFrameRefs());
  EXPECT_EQ(cc.link_a, "link_a");
  EXPECT_EQ(cc.link_b, "link_b");
  EXPECT_DOUBLE_EQ(cc.offset_a_xyz.x(), 0.03);
  EXPECT_DOUBLE_EQ(cc.offset_b_xyz.z(), 0.01);
  EXPECT_NEAR(cc.offset_b_rpy.y(), 1.5707963, 1e-6);
  EXPECT_EQ(cc.reference_frame, rub::LoopReferenceFrame::kLocalWorldAligned);
}

// ── baumgarte Kd 미지정 → 임계감쇠 근처 (2√Kp) 자동 ──────────────────────────
TEST(ClosureYamlLoader, FillsCriticalDampingWhenKdOmitted) {
  const std::string yaml = R"(
closures:
  - name: loop0
    type: contact_3d
    frame_1: a
    frame_2: b
    baumgarte: { Kp: 25.0 }
)";
  const rub::ClosureSpec spec = rub::ParseClosureYaml(yaml);
  ASSERT_EQ(spec.closures.size(), 1u);
  EXPECT_DOUBLE_EQ(spec.closures.front().baumgarte_kp, 25.0);
  EXPECT_DOUBLE_EQ(spec.closures.front().baumgarte_kd, 2.0 * std::sqrt(25.0));  // = 10
}

// ── type 필수 필드 누락 → throw ───────────────────────────────────────────────
TEST(ClosureYamlLoader, ThrowsWhenTypeMissing) {
  const std::string yaml = R"(
closures:
  - name: loop0
    frame_1: a
    frame_2: b
)";
  EXPECT_THROW(rub::ParseClosureYaml(yaml), std::runtime_error);
}

// ── endpoint 미지정 (frame 도 link 도 없음) → throw ──────────────────────────
TEST(ClosureYamlLoader, ThrowsWhenEndpointMissing) {
  const std::string yaml = R"(
closures:
  - name: loop0
    type: contact_3d
)";
  EXPECT_THROW(rub::ParseClosureYaml(yaml), std::runtime_error);
}

// ── 유효하지 않은 type 문자열 → throw ────────────────────────────────────────
TEST(ClosureYamlLoader, ThrowsOnInvalidType) {
  const std::string yaml = R"(
closures:
  - name: loop0
    type: contact_9d
    frame_1: a
    frame_2: b
)";
  EXPECT_THROW(rub::ParseClosureYaml(yaml), std::runtime_error);
}

// ── 실제 sidecar 파일 로드 (four_bar.closure.yaml) ───────────────────────────
TEST(ClosureYamlLoader, LoadsRealSidecarFile) {
  const rub::ClosureSpec spec = rub::LoadClosureYaml(TestUrdfPath("four_bar.closure.yaml"));
  EXPECT_EQ(spec.name, "four_bar_tree");
  ASSERT_EQ(spec.closures.size(), 1u);
  EXPECT_EQ(spec.closures.front().frame_1, "c1");
  EXPECT_EQ(spec.closures.front().frame_2, "c2");
  EXPECT_FALSE(spec.closures.front().is_6d);
  ASSERT_EQ(spec.actuated_joints.size(), 1u);
  EXPECT_EQ(spec.actuated_joints.front(), "joint_a");
}
