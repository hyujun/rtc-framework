// ── 관성 게이트를 저장소 실모델에 발사한다 ───────────────────────────────────
//
// 합성 픽스처(test_inertial_validation.cpp)는 *게이트의 로직*만 검증한다. 이
// 파일이 없으면 게이트는 자기가 겨냥한 대상에 한 번도 발사되지 않는다 —
// `rtc_tools/test/test_real_model_pairs.py` 가 `compare_mjcf_urdf` 에 대해
// 같은 이유로 존재하는 것과 같은 축이다.
//
// **schunk_svh_hand_{left,right} 는 negative fixture 다.** 통과 대상이 아니라
// *거부 대상*이며, 이는 의도된 상태다 (#316):
//   - `{left,right}_hand_p` 는 mass 0.024 kg 에 주모멘트 (9e-7, 2e-6, 3e-6) 로
//     9e-7 + 2e-6 = 2.9e-6 < 3e-6 — 삼각부등식을 3.3% 위반한다.
//   - movable body 13 개가 composite mass 0 이고, 그중 8 개는 mimic 이 아니라
//     **독립 구동 관절**이다 → M(q) 가 자기 actuated DoF 위에서 특이하다.
// 이 URDF 의 관성값을 지어내 고치지 않는 것은 #413 이 세운 규율이다 (실기 측정
// 없이 placeholder 를 물리값으로 확정하지 않는다). schunk 는 config/launch 어디
// 에서도 로드되지 않는 data-only 자산이라 운영 파급이 없고, 그래서 합성 위반보다
// 강한 **실세계 positive control** 로 쓴다 — 내가 만든 픽스처는 자기 oracle 이
// 되기 쉽지만 이건 아니다.
#include "rtc_urdf_bridge/inertial_validation.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

std::string ModelPath(const std::string& relative) {
  return (std::filesystem::path(
              ament_index_cpp::get_package_share_directory("robot_descriptions")) /
          relative)
      .string();
}

/// 런타임 소비자를 가진 실모델 전부 — model_pairs.yaml 8 쌍의 URDF + 그 밖의
/// 소비 모델 3 종. 로드 게이트는 *열리는 모든 URDF* 에 걸리므로 pairs 목록보다
/// 넓은 것이 맞다.
const std::vector<std::string>& CleanModels() {
  static const std::vector<std::string> kModels = {
      "robots/ur5e/urdf/ur5e.urdf",
      "robots/iiwa7/urdf/iiwa7.urdf",
      "robots/leap_hand/urdf/leap_hand_left.urdf",
      "robots/leap_hand/urdf/leap_hand_right.urdf",
      "robots/iiwa7_leap/urdf/iiwa7_with_leap_left.urdf.xacro",
      "robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
      "robots/assm_v1/urdf/hand.urdf.xacro",
      "robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf.xacro",
      "robots/panda/urdf/panda.urdf",
      "robots/ur5e_p1b/urdf/proto_1b.urdf",
      "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
  };
  return kModels;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 통과 축 — 게이트가 실제로 쓰이는 모델을 막지 않는가
// ═══════════════════════════════════════════════════════════════════════════

class RealModelInertialGate : public ::testing::TestWithParam<std::string> {};

TEST_P(RealModelInertialGate, LoadsWithoutInertialViolation) {
  rub::ModelConfig cfg;
  cfg.urdf_path = ModelPath(GetParam());
  ASSERT_TRUE(std::filesystem::exists(cfg.urdf_path)) << cfg.urdf_path;

  std::unique_ptr<rub::PinocchioModelBuilder> builder;
  ASSERT_NO_THROW(builder = std::make_unique<rub::PinocchioModelBuilder>(cfg)) << GetParam();

  const auto& report = builder->GetInertialReport();
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
  // 무경고 통과 — V5 레인도 비어야 한다.
  EXPECT_TRUE(report.degenerate.empty()) << rub::DescribeInertialViolations(report.degenerate);
  EXPECT_TRUE(builder->IsFullModelDynamicsCapable());
}

INSTANTIATE_TEST_SUITE_P(AllConsumedModels, RealModelInertialGate,
                         ::testing::ValuesIn(CleanModels()),
                         [](const ::testing::TestParamInfo<std::string>& info) {
                           std::string name =
                               std::filesystem::path(info.param).stem().stem().string();
                           std::replace(name.begin(), name.end(), '.', '_');
                           return name;
                         });

// ═══════════════════════════════════════════════════════════════════════════
// 거부 축 — schunk SVH (실세계 negative fixture)
// ═══════════════════════════════════════════════════════════════════════════

class SchunkNegativeFixture : public ::testing::TestWithParam<const char*> {};

TEST_P(SchunkNegativeFixture, RejectedForTriangleInequalityAtDistalLink) {
  const std::string path =
      ModelPath(std::string("robots/schunk_hand/urdf/") + GetParam() + ".urdf");
  ASSERT_TRUE(std::filesystem::exists(path)) << path;

  // 빌더 경로: V6 가 있으므로 로드가 실패해야 한다.
  rub::ModelConfig cfg;
  cfg.urdf_path = path;
  EXPECT_THROW(rub::PinocchioModelBuilder{cfg}, std::runtime_error);

  // 직접 경로: throw 뒤에 가려진 판정 내용을 들여다본다.
  pinocchio::Model model;
  pinocchio::urdf::buildModel(path, model);
  const auto report = rub::ValidateInertias(model);

  ASSERT_EQ(report.fatal.size(), 1u) << rub::DescribeInertialViolations(report.fatal);
  const auto& bad = report.fatal[0];
  EXPECT_EQ(bad.defect, rub::InertialDefect::kTriangleInequality);
  EXPECT_NE(bad.body_name.find("hand_p"), std::string::npos) << bad.body_name;
  EXPECT_NEAR(bad.mass, 0.024, 1e-12);
  EXPECT_NEAR(bad.principal_moments[0], 9e-7, 1e-13);
  EXPECT_NEAR(bad.principal_moments[1], 2e-6, 1e-13);
  EXPECT_NEAR(bad.principal_moments[2], 3e-6, 1e-13);
  // -1/30 — 부동소수 잡음이 아니라 손으로 적힌 값의 3.3% 위반이다.
  EXPECT_NEAR(bad.margin, -1.0 / 30.0, 1e-9);
}

TEST_P(SchunkNegativeFixture, ReportsThirteenMasslessMovableBodies) {
  const std::string path =
      ModelPath(std::string("robots/schunk_hand/urdf/") + GetParam() + ".urdf");
  pinocchio::Model model;
  pinocchio::urdf::buildModel(path, model);
  const auto report = rub::ValidateInertias(model);

  EXPECT_EQ(report.degenerate.size(), 13u) << rub::DescribeInertialViolations(report.degenerate);
  for (const auto& v : report.degenerate) {
    EXPECT_EQ(v.defect, rub::InertialDefect::kMasslessMovableBody);
    EXPECT_EQ(v.mass, 0.0);
  }
  // fixed joint 흡수가 이것을 구제하지 못한다는 점이 핵심이다 — composite 를
  // 검사하는데도 남는다.
  EXPECT_EQ(model.nv, 20);
}

INSTANTIATE_TEST_SUITE_P(BothHands, SchunkNegativeFixture,
                         ::testing::Values("schunk_svh_hand_left", "schunk_svh_hand_right"),
                         [](const ::testing::TestParamInfo<const char*>& info) {
                           return std::string(info.param);
                         });
