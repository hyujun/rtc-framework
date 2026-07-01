// ── test_mjcf_comparison — Extended-URDF vs MJCF 교차검증 (Phase 11) ──────────
//
// [Pinocchio 4.0 MJCF 파서 한계 — §6/§1b-5 보고]
//   (1) <equality><connect> body 해석이 compiled 파서에서 "Model does not have any
//       body named ..." 예외를 던진다 (fixture 문제 아님).
//   (2) worldbody 의 두 번째 이후 child branch 를 파싱하지 않는다 — 2-branch 4-bar 를
//       넣어도 첫 branch(link_a→link_b) 만 model 에 들어온다 (nq=2).
//   → 두 branch 를 요구하는 loop 자체를 이 파서로 round-trip 할 수 없다. 따라서 완전한
//     loop 교차검증(closure error/qdd/lambda)은 URDF 경로 단독으로 수행하고
//     (test_loop_closed_chain 이 담당), 여기서는 파서가 실제로 읽는 branch 에 한해
//     "URDF vs MJCF 의 joint/frame 규약이 일치하는가"(핵심 위험)를 검증한다.
//
// 검증: 동일 (joint_a, joint_ab) 형상에서 link_b BODY frame 의 world placement 가
//       URDF 모델과 MJCF 모델에서 일치 → 두 엔진 규약 동등.
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "test_urdf_path.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

namespace {
pinocchio::Model LoadMjcfModel() {
  pinocchio::Model model;
  pinocchio::mjcf::buildModel(TestUrdfPath("four_bar.xml"), model);
  return model;
}

pinocchio::Model LoadUrdfModel() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(TestUrdfPath("four_bar_tree.urdf"), model);
  return model;
}

Eigen::VectorXd MakeMappedConfig(const pinocchio::Model& model,
                                 const std::vector<std::pair<std::string, double>>& joint_values) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  for (const auto& [name, value] : joint_values) {
    if (!model.existJointName(name)) {
      continue;
    }
    q[model.idx_qs[model.getJointId(name)]] = value;  // hinge/revolute: nq_j == 1
  }
  return q;
}

pinocchio::SE3 FramePlacement(const pinocchio::Model& model, pinocchio::Data& data,
                              const std::string& frame_name, const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  return data.oMf[model.getFrameId(frame_name)];
}
}  // namespace

// ── 파서가 읽는 branch: joint/frame 이름이 URDF 와 공유됨 ─────────────────────
TEST(MjcfComparison, ParsedBranchSharesNames) {
  const pinocchio::Model urdf = LoadUrdfModel();
  const pinocchio::Model mjcf = LoadMjcfModel();

  // 첫 branch 만 파싱됨 (파서 한계). 그래도 이름 mapping 은 성립해야 한다.
  for (const auto& jn : {"joint_a", "joint_ab"}) {
    EXPECT_TRUE(mjcf.existJointName(jn)) << jn;
    EXPECT_TRUE(urdf.existJointName(jn)) << jn;
  }
  EXPECT_TRUE(mjcf.existFrame("link_b"));
  EXPECT_TRUE(urdf.existFrame("link_b"));
}

// ── 핵심: 동일 형상에서 link_b world placement 가 두 엔진에서 일치 ────────────
TEST(MjcfComparison, LinkBPlacementAgreesAcrossEngines) {
  const pinocchio::Model urdf = LoadUrdfModel();
  const pinocchio::Model mjcf = LoadMjcfModel();
  pinocchio::Data urdf_data(urdf);
  pinocchio::Data mjcf_data(mjcf);

  // 파서가 읽는 관절만 구동 (branch 2 는 URDF 에서 0 유지 → link_b 에 영향 없음).
  const std::vector<std::pair<std::string, double>> jv{{"joint_a", 0.4}, {"joint_ab", -0.3}};
  const pinocchio::SE3 urdf_link_b =
      FramePlacement(urdf, urdf_data, "link_b", MakeMappedConfig(urdf, jv));
  const pinocchio::SE3 mjcf_link_b =
      FramePlacement(mjcf, mjcf_data, "link_b", MakeMappedConfig(mjcf, jv));

  EXPECT_TRUE(urdf_link_b.translation().isApprox(mjcf_link_b.translation(), 1e-9))
      << "urdf=" << urdf_link_b.translation().transpose()
      << " mjcf=" << mjcf_link_b.translation().transpose();
  EXPECT_TRUE(urdf_link_b.rotation().isApprox(mjcf_link_b.rotation(), 1e-9));
}

// ── nominal q=0: link_b world placement 도 일치 ──────────────────────────────
TEST(MjcfComparison, LinkBPlacementAgreesAtNominal) {
  const pinocchio::Model urdf = LoadUrdfModel();
  const pinocchio::Model mjcf = LoadMjcfModel();
  pinocchio::Data urdf_data(urdf);
  pinocchio::Data mjcf_data(mjcf);

  const pinocchio::SE3 a = FramePlacement(urdf, urdf_data, "link_b", pinocchio::neutral(urdf));
  const pinocchio::SE3 b = FramePlacement(mjcf, mjcf_data, "link_b", pinocchio::neutral(mjcf));
  EXPECT_TRUE(a.isApprox(b, 1e-9));
}
