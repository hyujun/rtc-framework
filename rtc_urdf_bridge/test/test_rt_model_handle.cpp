// ── RtModelHandle 테스트 ────────────────────────────────────────────────────
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace rub = rtc_urdf_bridge;

static std::string TestUrdfPath(const std::string & filename)
{
  std::filesystem::path p(__FILE__);
  return (p.parent_path() / "urdf" / filename).string();
}

// NaN/Inf 검사 헬퍼
static bool HasNanOrInf(const Eigen::Ref<const Eigen::VectorXd> & v)
{
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (std::isnan(v[i]) || std::isinf(v[i])) return true;
  }
  return false;
}

static bool MatrixHasNanOrInf(const Eigen::Ref<const Eigen::MatrixXd> & m)
{
  for (Eigen::Index i = 0; i < m.rows(); ++i) {
    for (Eigen::Index j = 0; j < m.cols(); ++j) {
      if (std::isnan(m(i, j)) || std::isinf(m(i, j))) return true;
    }
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Serial arm 핸들 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class RtModelHandleSerialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("serial_6dof.urdf");
    cfg.root_joint_type = "fixed";
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
    handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetFullModel());
  }
  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
  std::unique_ptr<rub::RtModelHandle> handle_;
};

TEST_F(RtModelHandleSerialTest, Dimensions)
{
  EXPECT_EQ(handle_->nq(), 6);
  EXPECT_EQ(handle_->nv(), 6);
}

TEST_F(RtModelHandleSerialTest, ForwardKinematicsNeutral)
{
  std::vector<double> q(6, 0.0);
  handle_->ComputeForwardKinematics(q);

  auto tool_id = handle_->GetFrameId("tool_link");
  EXPECT_GT(tool_id, 0u);

  auto pos = handle_->GetFramePosition(tool_id);
  EXPECT_FALSE(std::isnan(pos.x()));
  EXPECT_FALSE(std::isnan(pos.y()));
  EXPECT_FALSE(std::isnan(pos.z()));
  // Z 좌표는 양수여야 함 (팔이 위를 향함)
  EXPECT_GT(pos.z(), 0.0);
}

TEST_F(RtModelHandleSerialTest, JacobianComputation)
{
  std::vector<double> q(6, 0.0);
  handle_->ComputeForwardKinematics(q);
  handle_->ComputeJacobians(q);

  auto tool_id = handle_->GetFrameId("tool_link");
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
  handle_->GetFrameJacobian(tool_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

  EXPECT_FALSE(MatrixHasNanOrInf(J));
  // 자코비안이 완전 영행렬이면 안 됨
  EXPECT_GT(J.norm(), 1e-10);
}

TEST_F(RtModelHandleSerialTest, InverseDynamicsGravity)
{
  // Z축 회전 관절이므로 non-zero config에서 중력 토크 확인
  std::vector<double> q = {0.0, 0.5, -0.3, 0.2, -0.1, 0.4};
  std::vector<double> v(6, 0.0);
  std::vector<double> a(6, 0.0);
  handle_->ComputeInverseDynamics(q, v, a);

  auto tau = handle_->GetTau();
  EXPECT_FALSE(HasNanOrInf(tau));
  // RNEA with v=0, a=0 → gravity compensation 토크 반환
  // (값 자체가 0일 수도 있으나 NaN/Inf 아님 확인이 핵심)
}

TEST_F(RtModelHandleSerialTest, ForwardDynamics)
{
  std::vector<double> q(6, 0.0);
  std::vector<double> v(6, 0.0);
  std::vector<double> tau(6, 0.0);
  handle_->ComputeForwardDynamics(q, v, tau);

  auto ddq = handle_->GetDdq();
  EXPECT_FALSE(HasNanOrInf(ddq));
}

TEST_F(RtModelHandleSerialTest, NonLinearEffects)
{
  std::vector<double> q(6, 0.0);
  std::vector<double> v(6, 0.0);
  handle_->ComputeNonLinearEffects(q, v);

  auto nle = handle_->GetNonLinearEffects();
  EXPECT_FALSE(HasNanOrInf(nle));
}

TEST_F(RtModelHandleSerialTest, MassMatrix)
{
  std::vector<double> q(6, 0.0);
  handle_->ComputeMassMatrix(q);

  auto M = handle_->GetMassMatrix();
  EXPECT_FALSE(MatrixHasNanOrInf(M));
  EXPECT_EQ(M.rows(), 6);
  EXPECT_EQ(M.cols(), 6);

  // 대칭 확인
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < i; ++j) {
      EXPECT_NEAR(M(i, j), M(j, i), 1e-12);
    }
  }

  // 양의 정부호 확인 (대각 원소 양수)
  for (int i = 0; i < 6; ++i) {
    EXPECT_GT(M(i, i), 0.0);
  }
}

TEST_F(RtModelHandleSerialTest, MoveConstruct)
{
  auto handle2 = std::move(*handle_);
  EXPECT_EQ(handle2.nq(), 6);

  std::vector<double> q(6, 0.0);
  handle2.ComputeForwardKinematics(q);
  auto pos = handle2.GetFramePosition(handle2.GetFrameId("tool_link"));
  EXPECT_FALSE(std::isnan(pos.z()));
}

TEST_F(RtModelHandleSerialTest, NonexistentFrameReturnsZero)
{
  auto fid = handle_->GetFrameId("nonexistent_frame");
  EXPECT_EQ(fid, 0u);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sub-model FK vs Full model FK 일치 테스트
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RtModelHandleFKConsistency, SubModelVsFullModel)
{
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("serial_6dof.urdf");
  cfg.root_joint_type = "fixed";
  cfg.sub_models.push_back({"arm", "base_link", "tool_link"});
  rub::PinocchioModelBuilder builder(cfg);

  // full model handle
  rub::RtModelHandle full_handle(builder.GetFullModel());
  // sub-model handle (arm = 전체 체인이므로 동일한 결과 기대)
  rub::RtModelHandle arm_handle(builder.GetReducedModel("arm"));

  // 임의의 설정값
  std::vector<double> q = {0.1, -0.3, 0.5, -0.2, 0.4, -0.1};

  full_handle.ComputeForwardKinematics(q);
  arm_handle.ComputeForwardKinematics(q);

  auto full_tool_id = full_handle.GetFrameId("tool_link");
  auto arm_tool_id = arm_handle.GetFrameId("tool_link");

  auto full_pos = full_handle.GetFramePosition(full_tool_id);
  auto arm_pos = arm_handle.GetFramePosition(arm_tool_id);

  EXPECT_NEAR(full_pos.x(), arm_pos.x(), 1e-10);
  EXPECT_NEAR(full_pos.y(), arm_pos.y(), 1e-10);
  EXPECT_NEAR(full_pos.z(), arm_pos.z(), 1e-10);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tree hand 핸들 테스트
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RtModelHandleTreeTest, MultipleTipFK)
{
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("tree_hand.urdf");
  cfg.root_joint_type = "fixed";
  rub::PinocchioModelBuilder builder(cfg);

  rub::RtModelHandle handle(builder.GetFullModel());
  EXPECT_EQ(handle.nq(), 10);

  std::vector<double> q(10, 0.3);
  handle.ComputeForwardKinematics(q);

  // 각 finger tip의 위치가 유효해야 함
  for (const auto & tip : {"thumb_tip", "index_tip", "middle_tip", "ring_tip"}) {
    auto fid = handle.GetFrameId(tip);
    EXPECT_GT(fid, 0u) << tip << " 프레임을 찾을 수 없습니다";
    auto pos = handle.GetFramePosition(fid);
    EXPECT_FALSE(std::isnan(pos.x())) << tip;
    EXPECT_FALSE(std::isnan(pos.y())) << tip;
    EXPECT_FALSE(std::isnan(pos.z())) << tip;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Joint reorder 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class RtModelHandleReorderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("tree_hand.urdf");
    cfg.root_joint_type = "fixed";
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
  }
  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
};

TEST_F(RtModelHandleReorderTest, ReorderedFKMatchesDirect)
{
  // tree_hand.urdf: palm_link → thumb(3), index(3), middle(2), ring(2)
  // Pinocchio 알파벳 DFS 순서: index(3), middle(2), ring(2), thumb(3)
  // 외부 순서 (thumb first):
  std::vector<std::string> external_order = {
    "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
    "index_joint_1", "index_joint_2", "index_joint_3",
    "middle_joint_1", "middle_joint_2",
    "ring_joint_1", "ring_joint_2"
  };

  // 외부 순서로 thumb만 0.5, 0.3, 0.2 설정
  std::vector<double> external_q = {
    0.5, 0.3, 0.2,    // thumb
    0.0, 0.0, 0.0,    // index
    0.0, 0.0,          // middle
    0.0, 0.0           // ring
  };

  // (1) Reorder handle
  rub::RtModelHandle reorder_handle(builder_->GetFullModel());
  ASSERT_TRUE(reorder_handle.SetJointOrder(external_order));
  EXPECT_TRUE(reorder_handle.HasJointReorder());
  reorder_handle.ComputeForwardKinematics(external_q);

  // (2) Direct handle — Pinocchio 순서로 수동 배치
  // Pinocchio 순서: index(0,1,2), middle(3,4), ring(5,6), thumb(7,8,9)
  rub::RtModelHandle direct_handle(builder_->GetFullModel());
  auto pin_names = direct_handle.GetPinocchioJointNames();
  ASSERT_EQ(pin_names.size(), 10u);

  // Pinocchio 순서에 맞게 q 벡터 구성
  std::vector<double> pinocchio_q(10, 0.0);
  // thumb 값을 Pinocchio 인덱스에 배치
  for (std::size_t i = 0; i < pin_names.size(); ++i) {
    if (pin_names[i] == "thumb_joint_1") pinocchio_q[i] = 0.5;
    if (pin_names[i] == "thumb_joint_2") pinocchio_q[i] = 0.3;
    if (pin_names[i] == "thumb_joint_3") pinocchio_q[i] = 0.2;
  }
  direct_handle.ComputeForwardKinematics(pinocchio_q);

  // (3) 모든 fingertip 위치가 일치
  for (const auto & tip : {"thumb_tip", "index_tip", "middle_tip", "ring_tip"}) {
    auto fid_r = reorder_handle.GetFrameId(tip);
    auto fid_d = direct_handle.GetFrameId(tip);
    ASSERT_GT(fid_r, 0u) << tip;
    auto pos_r = reorder_handle.GetFramePosition(fid_r);
    auto pos_d = direct_handle.GetFramePosition(fid_d);
    EXPECT_NEAR(pos_r.x(), pos_d.x(), 1e-10) << tip;
    EXPECT_NEAR(pos_r.y(), pos_d.y(), 1e-10) << tip;
    EXPECT_NEAR(pos_r.z(), pos_d.z(), 1e-10) << tip;
  }
}

TEST_F(RtModelHandleReorderTest, SetJointOrderReturnsFalseForBadName)
{
  rub::RtModelHandle handle(builder_->GetFullModel());
  std::vector<std::string> bad_names = {"thumb_joint_1", "nonexistent_joint"};
  EXPECT_FALSE(handle.SetJointOrder(bad_names));
  EXPECT_FALSE(handle.HasJointReorder());
}

TEST_F(RtModelHandleReorderTest, IdentityOrderSkipsReorder)
{
  rub::RtModelHandle handle(builder_->GetFullModel());
  // Pinocchio 내부 순서와 동일한 순서 전달
  auto pin_names = handle.GetPinocchioJointNames();
  ASSERT_TRUE(handle.SetJointOrder(pin_names));
  // identity이므로 reorder 비활성
  EXPECT_FALSE(handle.HasJointReorder());
}

TEST_F(RtModelHandleReorderTest, ReorderedJacobianMatchesDirect)
{
  std::vector<std::string> external_order = {
    "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
    "index_joint_1", "index_joint_2", "index_joint_3",
    "middle_joint_1", "middle_joint_2",
    "ring_joint_1", "ring_joint_2"
  };

  std::vector<double> external_q = {
    0.5, 0.3, 0.2, 0.1, -0.1, 0.4,
    0.2, -0.1, 0.3, 0.1
  };

  // (1) Reorder handle
  rub::RtModelHandle reorder_handle(builder_->GetFullModel());
  ASSERT_TRUE(reorder_handle.SetJointOrder(external_order));
  reorder_handle.ComputeJacobians(external_q);

  // (2) Direct handle — Pinocchio 순서 q
  rub::RtModelHandle direct_handle(builder_->GetFullModel());
  auto pin_names = direct_handle.GetPinocchioJointNames();
  std::vector<double> pinocchio_q(10, 0.0);
  // 이름 기반 매핑
  std::unordered_map<std::string, double> name_val;
  for (std::size_t i = 0; i < external_order.size(); ++i) {
    name_val[external_order[i]] = external_q[i];
  }
  for (std::size_t i = 0; i < pin_names.size(); ++i) {
    pinocchio_q[i] = name_val[pin_names[i]];
  }
  direct_handle.ComputeJacobians(pinocchio_q);

  // (3) thumb_tip Jacobian 비교
  auto fid_r = reorder_handle.GetFrameId("thumb_tip");
  auto fid_d = direct_handle.GetFrameId("thumb_tip");
  Eigen::MatrixXd J_r = Eigen::MatrixXd::Zero(6, 10);
  Eigen::MatrixXd J_d = Eigen::MatrixXd::Zero(6, 10);
  reorder_handle.GetFrameJacobian(fid_r, pinocchio::LOCAL_WORLD_ALIGNED, J_r);
  direct_handle.GetFrameJacobian(fid_d, pinocchio::LOCAL_WORLD_ALIGNED, J_d);

  // Jacobian 값이 동일 (Pinocchio 내부 순서로 출력됨)
  for (Eigen::Index r = 0; r < 6; ++r) {
    for (Eigen::Index c = 0; c < 10; ++c) {
      EXPECT_NEAR(J_r(r, c), J_d(r, c), 1e-10)
        << "row=" << r << " col=" << c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mimic 헬퍼 테스트
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RtModelHandleMimicTest, ComputeMimicPosition)
{
  // q_mimic = -1.0 * 0.5 + 0.0 = -0.5
  double result = rub::RtModelHandle::ComputeMimicPosition(0.5, -1.0, 0.0);
  EXPECT_DOUBLE_EQ(result, -0.5);

  // q_mimic = 2.0 * 0.3 + 0.1 = 0.7
  result = rub::RtModelHandle::ComputeMimicPosition(0.3, 2.0, 0.1);
  EXPECT_DOUBLE_EQ(result, 0.7);
}
