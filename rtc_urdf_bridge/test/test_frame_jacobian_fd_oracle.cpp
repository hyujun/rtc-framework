// ── GetFrameJacobian 행/열 순서 독립 oracle (#316 V9) ───────────────────────
//
// RtModelHandle::GetFrameJacobian 의 공개 계약은 셋이다:
//   rows 0..2 = linear · rows 3..5 = angular · columns = Pinocchio velocity order.
// 단 linear 이 **무엇의** 선속도인지는 ref_frame 이 정한다 — LOCAL/LWA 는 프레임
// 원점의 속도지만 WORLD 는 world 원점에 놓인 spatial twist 다 (아래
// WorldJacobianIsSpatialTwistAtOrigin 이 그 셋째를 고정한다).
//
// test_rt_model_handle.cpp 의 기존 커버리지는 이 중 어느 것도 고정하지 못한다:
//   • JacobianComputation      → 유한성 + `norm() > 0` 뿐. 블록을 맞바꿔도 노름은
//                                불변이므로 row-swap 에 완전히 둔감하다.
//   • ReorderedJacobianMatchesDirect → **자기일치**다. 두 handle 이 같은 추출기를
//                                통과하므로 그 추출기가 행을 뒤집어도 양쪽이 똑같이
//                                뒤집혀 green 으로 남는다.
// 즉 "테스트가 있다" 가 "이 축이 검사된다" 를 뜻하지 않았다.
//
// 여기서는 자코비안을 **전혀 쓰지 않는** oracle 로 대조한다: 프레임 SE3 의 중심
// 차분. 참조 twist 가 FK 경로에서만 나오므로 추출기 안의 행·열 치환이 상쇄될 수
// 없다. 오배선은 NaN 도 발산도 아닌 "유한하고 매끄럽게 틀린 값" 이라 이런 외부
// oracle 이 아니면 어떤 게이트에도 안 걸린다.
//
// 각 테스트는 자기 판별력을 실행 시점에 재검사한다 (positive control) — 픽스처
// 기하가 바뀌어 linear ≡ angular 가 되거나 device 순서가 Pinocchio 순서와 우연히
// 같아지면, 대조는 green 인데 mutation 은 안 잡히는 상태가 되기 때문이다.

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "test_urdf_path.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/skew.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

using rtc::test::TestUrdfPath;

namespace {

// 중심차분 스텝. 오차 예산 = 절단 O(h²·f''') ≈ 2e-13 + 반올림 O(eps·|f|/h) ≈ 6e-11
// → 실제 편차는 1e-10 수준이다. kFdTol 은 그보다 세 자릿수 위, 그러나 행 교환이
// 만드는 편차(O(0.1~1))보다는 여섯 자릿수 아래에 둔다.
constexpr double kFdStep = 1e-6;
constexpr double kFdTol = 1e-7;

// 대수적 항등식 대조용 (FD 가 아니므로 반올림만 허용한다).
constexpr double kExactTol = 1e-12;

// 두 행 블록이 kFdTol 안에서 구별 가능해야 대조가 row-swap 을 잡는다.
// 이 여유(1e3×)는 판별력이 사라진 픽스처를 green 이 아니라 red 로 만든다.
constexpr double kBlockGapFloor = 1e3 * kFdTol;

struct Twist {
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

// q 의 slot `slot` 만 ±h 흔들었을 때의 프레임 배치 두 개.
// GetFramePlacement 는 handle 내부 data_ 로의 **참조**를 돌려주므로 값으로
// 복사해 둔다 — 참조로 들고 있으면 두 번째 FK 가 첫 결과를 덮어 두 배치가
// 같아지고, 그러면 모든 차분이 0 이 되어 테스트가 조용히 공허해진다.
//
// **사후조건**: 반환 시 handle 의 FK 상태를 q 로 되돌려 놓는다. 안 그러면 oMf 는
// q−h, data_.J 는 (직전 ComputeJacobians 의) q 인 혼성 상태가 남아, 이 헬퍼를
// GetFrameJacobian **앞에** 부르는 테스트가 O(dJ/dq·h)≈1e-6 만큼 틀린 LWA
// 자코비안을 받는다 — kFdTol 의 열 배라 잡히긴 하나 마진이 얇다.
void PerturbedPlacements(rub::RtModelHandle& handle, const std::vector<double>& q, std::size_t slot,
                         double h, pinocchio::FrameIndex fid, pinocchio::SE3& m_plus,
                         pinocchio::SE3& m_minus) {
  std::vector<double> q_pert = q;
  q_pert[slot] = q[slot] + h;
  handle.ComputeForwardKinematics(q_pert);
  m_plus = handle.GetFramePlacement(fid);

  q_pert[slot] = q[slot] - h;
  handle.ComputeForwardKinematics(q_pert);
  m_minus = handle.GetFramePlacement(fid);

  handle.ComputeForwardKinematics(q);
}

// LOCAL twist 참조: log6(M₋⁻¹·M₊) / 2h.
// oMf(q ± h·e_j) ≈ oMf(q)·exp6(±h·J_local.col(j)) 이므로 M₋⁻¹·M₊ ≈ exp6(2h·col_j).
Twist FdLocalTwist(rub::RtModelHandle& handle, const std::vector<double>& q, std::size_t slot,
                   pinocchio::FrameIndex fid, double h) {
  pinocchio::SE3 m_plus;
  pinocchio::SE3 m_minus;
  PerturbedPlacements(handle, q, slot, h, fid, m_plus, m_minus);

  const pinocchio::Motion delta = pinocchio::log6(m_minus.actInv(m_plus));
  Twist t;
  t.linear = delta.linear() / (2.0 * h);
  t.angular = delta.angular() / (2.0 * h);
  return t;
}

// LOCAL_WORLD_ALIGNED twist 참조: 프레임 원점의 world 속도 + world 각속도.
//   linear  = (p₊ − p₋) / 2h
//   angular = log3(R₊·R₋ᵀ) / 2h   (Ṙ = ω× R → R₊R₋ᵀ ≈ exp3(2h·ω))
Twist FdWorldAlignedTwist(rub::RtModelHandle& handle, const std::vector<double>& q,
                          std::size_t slot, pinocchio::FrameIndex fid, double h) {
  pinocchio::SE3 m_plus;
  pinocchio::SE3 m_minus;
  PerturbedPlacements(handle, q, slot, h, fid, m_plus, m_minus);

  Twist t;
  t.linear = (m_plus.translation() - m_minus.translation()) / (2.0 * h);
  t.angular = pinocchio::log3(m_plus.rotation() * m_minus.rotation().transpose()) / (2.0 * h);
  return t;
}

using FdTwistFn = Twist (*)(rub::RtModelHandle&, const std::vector<double>&, std::size_t,
                            pinocchio::FrameIndex, double);

// 판별력 positive control: linear 블록과 angular 블록이 구별 가능한가.
// 이 여유가 없으면 아래 원소별 대조는 row-swap 아래서도 green 으로 남는다.
void ExpectRowBlockSwapWouldBeCaught(const Eigen::MatrixXd& J, const char* what) {
  const double gap = (J.topRows(3) - J.bottomRows(3)).cwiseAbs().maxCoeff();
  EXPECT_GT(gap, kBlockGapFloor)
      << what << ": linear/angular 블록 차가 " << gap << " 로 판별 하한(" << kBlockGapFloor
      << ") 에 붙어 있다 — 이 픽스처에서는 행 교환이 대조를 통과한다. 기하를 바꿔 대칭을 깰 것.";
}

// ref_frame 하나에 대한 전체 대조. ref_frame 을 늘려도 호출 한 줄이면 되도록
// 참조 생성기를 함수 포인터로 받는다.
void ExpectJacobianMatchesFd(rub::RtModelHandle& handle, const std::vector<double>& q,
                             pinocchio::FrameIndex fid, pinocchio::ReferenceFrame ref_frame,
                             FdTwistFn fd, const char* what) {
  handle.ComputeJacobians(q);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(q.size()));
  handle.GetFrameJacobian(fid, ref_frame, J);

  for (std::size_t j = 0; j < q.size(); ++j) {
    const Twist ref = fd(handle, q, j, fid, kFdStep);
    const auto c = static_cast<Eigen::Index>(j);
    for (Eigen::Index r = 0; r < 3; ++r) {
      EXPECT_NEAR(J(r, c), ref.linear[r], kFdTol) << what << " linear row=" << r << " col=" << j;
      EXPECT_NEAR(J(r + 3, c), ref.angular[r], kFdTol)
          << what << " angular row=" << (r + 3) << " col=" << j;
    }
  }

  ExpectRowBlockSwapWouldBeCaught(J, what);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// prismatic + revolute 혼합 픽스처
// ═══════════════════════════════════════════════════════════════════════════════

class FrameJacobianFdOracleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("mixed_prismatic_revolute.urdf");
    cfg.root_joint_type = "fixed";
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
    handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetFullModel());

    fid_ = handle_->GetFrameId("tool_link");
    ASSERT_GT(fid_, 0u) << "tool_link 프레임이 없다";

    // 아래 대조는 "q slot j ↔ v column j" 를 전제한다. 픽스처에 continuous
    // (nq=2) 나 다-DoF 관절이 들어오면 그 전제가 깨지므로 여기서 막는다.
    ASSERT_EQ(handle_->nq(), handle_->nv()) << "픽스처는 1-DoF 관절만 가져야 한다";
    ASSERT_EQ(handle_->nv(), kNv);
  }

  // Pinocchio v-공간에서 관절 `name` 의 열 인덱스. 열 순서는 handle 이 아니라
  // Model 의 성질이므로, 같은 Model 로 만든 다른 handle 에도 그대로 쓴다.
  // 이름이 없으면 nullopt 다 — `idx_vs[getJointId(...)]` 는 Release 에서 bounds
  // assert 가 빠져 njoints 를 그대로 인덱싱하므로, 픽스처를 rename 하면 깨끗한
  // red 가 아니라 범위 밖 읽기(UB)가 된다.
  [[nodiscard]] std::optional<Eigen::Index> VelocityColumnOf(const std::string& name) const {
    const auto& model = handle_->GetModel();
    if (!model.existJointName(name)) {
      ADD_FAILURE() << name << ": 픽스처에 그런 관절이 없다 (rename 됐는가?)";
      return std::nullopt;
    }
    return static_cast<Eigen::Index>(model.idx_vs[model.getJointId(name)]);
  }

  static constexpr int kNv = 5;

  // 특이자세를 피하고 모든 열에 부하를 싣는 자세 (관절 한계 안).
  const std::vector<double> q_{0.37, 0.11, -0.62, -0.05, 0.84};

  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
  std::unique_ptr<rub::RtModelHandle> handle_;
  pinocchio::FrameIndex fid_{0};
};

TEST_F(FrameJacobianFdOracleTest, LocalJacobianMatchesCentralDifference) {
  ExpectJacobianMatchesFd(*handle_, q_, fid_, pinocchio::LOCAL, &FdLocalTwist, "LOCAL");
}

TEST_F(FrameJacobianFdOracleTest, WorldAlignedJacobianMatchesCentralDifference) {
  ExpectJacobianMatchesFd(*handle_, q_, fid_, pinocchio::LOCAL_WORLD_ALIGNED, &FdWorldAlignedTwist,
                          "LOCAL_WORLD_ALIGNED");
}

// 두 ref_frame 이 실제로 다른 값을 내야 위 두 테스트가 서로 다른 계약을 검사한다.
// tool_joint 의 rpy 를 지우면 이 단언이 먼저 죽는다.
TEST_F(FrameJacobianFdOracleTest, LocalAndWorldAlignedActuallyDiffer) {
  handle_->ComputeJacobians(q_);
  Eigen::MatrixXd J_local = Eigen::MatrixXd::Zero(6, kNv);
  Eigen::MatrixXd J_lwa = Eigen::MatrixXd::Zero(6, kNv);
  handle_->GetFrameJacobian(fid_, pinocchio::LOCAL, J_local);
  handle_->GetFrameJacobian(fid_, pinocchio::LOCAL_WORLD_ALIGNED, J_lwa);

  EXPECT_GT((J_local - J_lwa).cwiseAbs().maxCoeff(), kBlockGapFloor)
      << "LOCAL 과 LWA 가 같다 — tool 프레임이 world 와 정렬돼 두 대조가 같은 것을 본다";
}

// WORLD 는 셋 중 유일하게 rows 0..2 가 **프레임 원점 속도가 아니다** — world
// 원점에 놓인 body-fixed 점의 속도 v_O = ṗ + p × ω 다. 저장소 안에 WORLD 호출자는
// 없지만 Doxygen 이 세 ref_frame 을 약속하는 이상 계약은 셋 다 고정한다.
// 참조는 WORLD 경로를 쓰지 않고 조립한다: FD 로 이미 고정된 LWA + FK 원점 p.
TEST_F(FrameJacobianFdOracleTest, WorldJacobianIsSpatialTwistAtOrigin) {
  handle_->ComputeJacobians(q_);
  Eigen::MatrixXd J_world = Eigen::MatrixXd::Zero(6, kNv);
  Eigen::MatrixXd J_lwa = Eigen::MatrixXd::Zero(6, kNv);
  handle_->GetFrameJacobian(fid_, pinocchio::WORLD, J_world);
  handle_->GetFrameJacobian(fid_, pinocchio::LOCAL_WORLD_ALIGNED, J_lwa);

  const Eigen::Vector3d p = handle_->GetFramePlacement(fid_).translation();
  Eigen::MatrixXd expected = J_lwa;
  expected.topRows(3) += pinocchio::skew(p) * J_lwa.bottomRows(3);

  EXPECT_LT((J_world - expected).cwiseAbs().maxCoeff(), kExactTol)
      << "WORLD 자코비안이 v_O = ṗ + p × ω 규약과 다르다";

  // 판별력: p × ω 항이 0 이면 이 대조는 WORLD 를 LWA 와 구별하지 못한다
  // (프레임 원점이 world 원점에 오거나 각속도 열이 전부 0 인 픽스처).
  EXPECT_GT((J_world.topRows(3) - J_lwa.topRows(3)).cwiseAbs().maxCoeff(), kBlockGapFloor)
      << "WORLD 와 LWA 의 linear 블록이 같다 — 픽스처가 p × ω 를 0 으로 만든다";
}

// prismatic 열의 angular 블록은 정확히 0, linear 블록은 단위 축이다. 이 비대칭이
// 행 순서를 가장 날카롭게 고정한다 — 블록을 맞바꾸면 0 이어야 할 세 행에 단위
// 벡터가 앉는다.
TEST_F(FrameJacobianFdOracleTest, PrismaticColumnsHaveZeroAngularBlock) {
  handle_->ComputeJacobians(q_);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, kNv);
  handle_->GetFrameJacobian(fid_, pinocchio::LOCAL, J);

  for (const auto* name : {"pris_x", "pris_diag"}) {
    const auto c = VelocityColumnOf(name);
    ASSERT_TRUE(c.has_value()) << name;
    EXPECT_LT(J.col(*c).tail<3>().cwiseAbs().maxCoeff(), kFdTol)
        << name << ": prismatic 열의 angular 블록(rows 3..5)이 0 이 아니다";
    EXPECT_NEAR(J.col(*c).head<3>().norm(), 1.0, 1e-9)
        << name << ": prismatic 열의 linear 블록(rows 0..2)은 단위 축이어야 한다";
  }
}

// 회전 열의 angular 블록은 단위 축이고 linear 블록은 0 이 아니다 (프레임이 축
// 위에 있지 않다는 뜻). 후자가 0 이면 그 열에서는 행 교환이 안 잡힌다.
TEST_F(FrameJacobianFdOracleTest, RevoluteColumnsCarryBothBlocks) {
  handle_->ComputeJacobians(q_);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, kNv);
  handle_->GetFrameJacobian(fid_, pinocchio::LOCAL, J);

  for (const auto* name : {"rev_yaw", "rev_pitch", "rev_roll"}) {
    const auto c = VelocityColumnOf(name);
    ASSERT_TRUE(c.has_value()) << name;
    EXPECT_NEAR(J.col(*c).tail<3>().norm(), 1.0, 1e-9)
        << name << ": revolute 열의 angular 블록은 단위 축이어야 한다";
    EXPECT_GT(J.col(*c).head<3>().norm(), kBlockGapFloor)
        << name
        << ": revolute 열의 linear 블록이 0 이다 — 프레임이 회전축 위에 있어 "
           "이 열에서는 행 교환이 관측되지 않는다";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 열 순서 계약: 출력은 device 순서가 아니라 Pinocchio v-공간 순서다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(FrameJacobianFdOracleTest, ColumnsStayInPinocchioOrderUnderDeviceReorder) {
  // device 순서를 Pinocchio 순서의 역순으로 잡아 **대칭을 깬다** — 둘이 우연히
  // 같으면 이 테스트는 열 순서를 전혀 고정하지 못한 채 green 이 된다.
  rub::RtModelHandle handle(builder_->GetFullModel());
  std::vector<std::string> device_names = handle.GetPinocchioJointNames();
  std::reverse(device_names.begin(), device_names.end());
  ASSERT_TRUE(handle.SetJointOrder(device_names));
  ASSERT_TRUE(handle.HasJointReorder()) << "device 순서가 Pinocchio 순서와 같다 — 대칭 미파괴";

  // device 순서 q. 값이 slot 마다 달라야 gather 오배선이 값 차이로 드러난다.
  const std::vector<double> device_q{0.84, -0.05, -0.62, 0.11, 0.37};
  handle.ComputeJacobians(device_q);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, kNv);
  handle.GetFrameJacobian(fid_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  bool any_column_moved = false;

  for (std::size_t slot = 0; slot < device_names.size(); ++slot) {
    // device slot 을 흔든다 → 그 효과는 **Pinocchio 열** 에 나타나야 한다.
    const Twist ref = FdWorldAlignedTwist(handle, device_q, slot, fid_, kFdStep);
    const auto pin_col = VelocityColumnOf(device_names[slot]);
    ASSERT_TRUE(pin_col.has_value()) << device_names[slot];
    if (*pin_col != static_cast<Eigen::Index>(slot)) {
      any_column_moved = true;
    }

    for (Eigen::Index r = 0; r < 3; ++r) {
      EXPECT_NEAR(J(r, *pin_col), ref.linear[r], kFdTol)
          << "device slot=" << slot << " → pinocchio col=" << *pin_col << " linear row=" << r;
      EXPECT_NEAR(J(r + 3, *pin_col), ref.angular[r], kFdTol)
          << "device slot=" << slot << " → pinocchio col=" << *pin_col
          << " angular row=" << (r + 3);
    }
  }

  EXPECT_TRUE(any_column_moved) << "치환이 항등이라 device/Pinocchio 순서를 구별할 수 없다";
  ExpectRowBlockSwapWouldBeCaught(J, "device-reorder LWA");
}
