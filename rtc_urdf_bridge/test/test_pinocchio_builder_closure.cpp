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

// arm + closed-chain hand 를 xacro <xacro:include> 로 병합한 픽스처. hand 는
// four_bar_tree(spanning-tree) 이고 loop 은 standalone hand 기준으로 작성된
// four_bar.closure.yaml sidecar 다. auto-derive(a_b.closure.yaml) 가 아니라
// hand 이름 sidecar 를 명시 지정하는 사용자 시나리오와 동일.
rub::ModelConfig CombinedArmHandConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("arm_with_four_bar_hand.urdf.xacro");
  cfg.root_joint_type = "fixed";
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

// ── 사용자 시나리오: arm + hand 를 xacro 로 병합해도 standalone hand 의 closure
//    sidecar 가 병합 모델에서 resolve 되어 constraint 가 build 되는가. ──────────────
TEST(PinocchioBuilderClosure, CombinedXacroResolvesHandClosure) {
  const rub::PinocchioModelBuilder builder(CombinedArmHandConfig());

  const auto full = builder.GetFullModel();
  ASSERT_NE(full, nullptr);

  // (1) 병합이 실제로 일어났음: arm 관절이 full 모델에 존재하고 nq = arm(1) + hand(4).
  //     standalone four_bar 는 nq=4 이므로 arm_joint 존재가 병합의 직접 증거.
  EXPECT_TRUE(full->existJointName("arm_joint"));
  EXPECT_EQ(full->nq, 5);

  // (2) standalone hand 이름 기준 sidecar 가 병합 모델에서 그대로 resolve:
  //     frame c1/c2 → constraint 1개, joint_a → actuated joint 1개.
  ASSERT_EQ(builder.GetConstraintModels().size(), 1u);
  ASSERT_EQ(builder.GetClosureActuatedJointIds().size(), 1u);

  // constraint endpoint 는 hand loop 의 서로 다른 두 관절(joint_ab / joint_cd)에
  // 매달린다 — arm prefix 없이 이름이 보존됐다는 표시. id 는 full 모델 범위 내.
  const auto& c = builder.GetConstraintModels().front();
  EXPECT_NE(c.joint1_id, c.joint2_id);
  EXPECT_GT(c.joint1_id, 0u);
  EXPECT_GT(c.joint2_id, 0u);
  EXPECT_LT(c.joint1_id, static_cast<pinocchio::JointIndex>(full->njoints));
  EXPECT_LT(c.joint2_id, static_cast<pinocchio::JointIndex>(full->njoints));

  // actuated joint(joint_a) 가 full 모델에 존재.
  EXPECT_TRUE(full->existJointName("joint_a"));

  // (3) q_ref 는 병합 full 모델 차원. loop 은 arm(loop 밖)과 무관하므로 수렴/특이
  //     성질은 standalone four_bar 와 동일해야 한다 (병합 불변): converged + singular.
  EXPECT_EQ(builder.GetClosureReferenceConfig().size(), full->nq);
  EXPECT_TRUE(builder.IsClosureReferenceConverged());
  EXPECT_TRUE(builder.IsClosureReferenceSingular());
}
