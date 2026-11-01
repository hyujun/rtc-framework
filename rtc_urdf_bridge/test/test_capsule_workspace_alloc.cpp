// ── test_capsule_workspace_alloc — 노드당 평가 경로의 C malloc 0 (RT-1) ───────────
//
// MPC 계획기 노드와 CLIK tick 이 부르는 경로 — computeJointJacobians · UpdatePlacements ·
// SetBall · EvaluateDistances · EvaluateGradients — 를 17 관절 (손가락 잠근 축소 모델) ·
// 자기충돌 쌍 30 · 공 쌍 3 으로 돌린다. coal 의 거리 함수와 pinocchio 의 Jacobian 추출은 다른
// 공유 라이브러리에서 컴파일됐으므로 operator-new 카운터로는 보이지 않는다 — rtc_base 의
// C malloc gate 가 필요하다 (헤더 계약 참조). 양성 대조: pinocchio::Data 생성이 잡히는지.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"
#include "test_urdf_path.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;
using rtc::testing::ScopedMallocGate;

namespace {

struct Capsule {
  const char* name;
  const char* link;
  double p1x, p1y, p1z;
  double radius;
};

const std::vector<Capsule> kCapsules = {
    {"torso", "torso_link", 0.0, 0.0, 0.25, 0.08},
    {"l_shoulder", "left_shoulder_roll_link", 0.0, 0.0, -0.08, 0.05},
    {"l_upper", "left_shoulder_yaw_link", 0.012, 0.0, -0.12, 0.04},
    {"l_fore", "left_elbow_link", 0.1, 0.0, 0.0, 0.035},
    {"l_hand", "left_wrist_yaw_link", 0.05, 0.0, 0.0, 0.03},
    {"r_shoulder", "right_shoulder_roll_link", 0.0, 0.0, -0.08, 0.05},
    {"r_upper", "right_shoulder_yaw_link", 0.012, 0.0, -0.13, 0.04},
    {"r_fore", "right_elbow_link", 0.11, 0.0, 0.0, 0.035},
    {"r_hand", "hand_base", 0.06, 0.0, 0.0, 0.03},
};

std::string SpecYaml(std::size_t n_pairs) {
  std::string y = "collision_capsules:\n  capsules:\n";
  for (const Capsule& c : kCapsules) {
    y += "    - {name: " + std::string(c.name) + ", link: " + c.link + ", p0: [0, 0, 0], p1: [" +
         std::to_string(c.p1x) + ", " + std::to_string(c.p1y) + ", " + std::to_string(c.p1z) +
         "], radius: " + std::to_string(c.radius) + "}\n";
  }
  y += "  pairs:\n";
  std::size_t n = 0;
  for (std::size_t i = 0; i < kCapsules.size() && n < n_pairs; ++i) {
    for (std::size_t j = i + 1; j < kCapsules.size() && n < n_pairs; ++j) {
      y += std::string("    - [") + kCapsules[i].name + ", " + kCapsules[j].name + "]\n";
      ++n;
    }
  }
  y += "  ball_capsules: [l_upper, l_fore, l_hand]\n";
  return y;
}

}  // namespace

TEST(CapsuleWorkspaceAlloc, PerNodeEvaluationDoesNotMalloc) {
  pinocchio::Model full;
  pinocchio::urdf::buildModel(TestUrdfPath("dual_arm_trunk_hand.urdf"), full);
  std::vector<pinocchio::JointIndex> lock;
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full.njoints); ++j) {
    if (full.names[j].rfind("finger_", 0) == 0) {
      lock.push_back(j);
    }
  }
  const pinocchio::Model model = pinocchio::buildReducedModel(full, lock, pinocchio::neutral(full));
  ASSERT_EQ(model.nv, 17);
  pinocchio::Data data(model);

  rub::CapsuleWorkspace ws;
  rub::CapsuleWorkspace::BindOptions opt;
  opt.with_ball = true;
  opt.catch_frame = model.getFrameId("catch_frame", pinocchio::BODY);
  ASSERT_EQ(ws.Bind(rub::CapsuleModel::ParseYaml(SpecYaml(30), "alloc"), model, opt),
            rub::CapsuleBindStatus::kOk)
      << ws.BindError();
  ASSERT_EQ(ws.NumSelfPairs(), 30U);
  ASSERT_EQ(ws.NumBallPairs(), 3U);

  std::mt19937 rng(642);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  std::vector<Eigen::VectorXd> qs;
  for (int t = 0; t < 50; ++t) {
    Eigen::VectorXd q(model.nq);
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      q[i] = u(rng);
    }
    qs.push_back(q);
  }
  const Eigen::Vector3d p0(0.25, 0.25, 0.95);
  const Eigen::Vector3d p1(0.15, 0.35, 0.85);

  const auto tick = [&](const Eigen::VectorXd& q) {
    pinocchio::computeJointJacobians(model, data, q);
    ASSERT_TRUE(ws.UpdatePlacements(model, data));
    ws.SetBall(p0, p1, 0.035);
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
    ASSERT_TRUE(ws.EvaluateGradients(model, data));
  };
  tick(qs[0]);  // warm-up

  // 양성 대조: 라이브러리 안의 할당이 이 gate 에 보인다.
  {
    ScopedMallocGate gate;
    pinocchio::Data probe(model);
    ASSERT_GT(gate.count(), 0U) << "gate 가 라이브러리 할당을 보지 못한다 — 0 은 뜻이 없다";
  }

  ScopedMallocGate gate;
  for (const Eigen::VectorXd& q : qs) {
    tick(q);
  }
  EXPECT_EQ(gate.count(), 0U) << "노드당 평가 경로가 " << gate.count() << " 회 malloc (RT-1)";
}
