// ── test_pinocchio_builder_closure — Extended-URDF sidecar 를 PinocchioModelBuilder
//    경로로 로드 (ModelConfig::closure_yaml_path). CM 이 bring-up 시 밟는 경로. ──────
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

namespace {
rub::ModelConfig FourBarConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("four_bar_tree.urdf");
  // 픽스처 파일명(four_bar.closure.yaml)은 URDF stem(four_bar_tree)과 다르므로 명시 경로.
  cfg.closure_yaml_path = TestUrdfPath("four_bar.closure.yaml");
  return cfg;
}
}  // namespace

// ── closure_yaml_path 설정 시 sidecar 로부터 constraints/actuated/q_ref 채워짐 ──────
TEST(PinocchioBuilderClosure, ExtendedUrdfPopulatesClosure) {
  const rub::PinocchioModelBuilder builder(FourBarConfig());

  // four_bar sidecar: 1 loop (contact_3d), 1 actuated joint (joint_a).
  ASSERT_EQ(builder.GetConstraintModels().size(), 1u);
  EXPECT_EQ(builder.GetClosureActuatedJointIds().size(), 1u);

  const auto full = builder.GetFullModel();
  ASSERT_NE(full, nullptr);
  EXPECT_EQ(builder.GetClosureReferenceConfig().size(), full->nq);

  // four_bar 는 nominal 조립 상태로 설계 → projection 수렴하지만, neutral 근방 평면
  // 4-bar 는 대칭 특이형상이라 singular 로 표시된다 (converged 와 singular 는 별개 신호).
  EXPECT_TRUE(builder.IsClosureReferenceConverged());
  EXPECT_TRUE(builder.IsClosureReferenceSingular());
}

// ── closure_yaml_path 미설정(plain URDF)이면 closure 결과가 비어 있고 model 은 정상 ──
TEST(PinocchioBuilderClosure, PlainUrdfLeavesClosureEmpty) {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("four_bar_tree.urdf");
  // closure_yaml_path 비워둠 → 순수 spanning-tree 로만 빌드.

  const rub::PinocchioModelBuilder builder(cfg);

  EXPECT_TRUE(builder.GetConstraintModels().empty());
  EXPECT_TRUE(builder.GetClosureActuatedJointIds().empty());
  EXPECT_EQ(builder.GetClosureReferenceConfig().size(), 0);
  EXPECT_FALSE(builder.IsClosureReferenceConverged());
  EXPECT_FALSE(builder.IsClosureReferenceSingular());

  const auto full = builder.GetFullModel();
  ASSERT_NE(full, nullptr);
  EXPECT_GT(full->nq, 0);
}
