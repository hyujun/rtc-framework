// ── test_wbc_reduced_dynamics_provider — closed-chain 축약 동역학 주입 (#120) ──
//   WbcReducedDynamicsProvider 의 신규 로직 검증: (1) cache 좌표 q/v → 핸들 q_a/v_a 브릿지 +
//   축약 M/h/g scatter 가 non-RT RtClosedChainHandle 과 값 일치(crank_rocker 비특이), (2) 이름 기반
//   좌표 정렬 게이트(정렬 성공 활성 / 미매칭 비활성), (3) held tick → last-good hold, (4) 구속
//   없음/최초 tick → open-chain fallback(false). (축약 동역학 수학 자체는 rtc_urdf_bridge 에서
//   검증.)
#include "closure_test_fixtures.hpp"  // rtc_urdf_bridge/test (include dir via CMake)
#include "integrated_bringup/support/wbc_reduced_dynamics_provider.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
namespace ib = integrated_bringup;

namespace {

// crank_rocker: full model 에서 j_crank 만 남기고 나머지 movable 을 seed 형상에서 잠근 control
// (actuated 등가) 모델을 만든다. GetActuatedModel 이 사이드카 actuated 를 남기고 loop-passive 를
// 잠그는 것과 동형.
pinocchio::Model MakeControlModel(const pinocchio::Model& full, const Eigen::VectorXd& seed,
                                  const std::string& keep) {
  std::vector<pinocchio::JointIndex> to_lock;
  for (int j = 1; j < full.njoints; ++j) {
    const auto jidx = static_cast<std::size_t>(j);
    if (full.nvs[jidx] > 0 && full.names[jidx] != keep) {
      to_lock.push_back(static_cast<pinocchio::JointIndex>(j));
    }
  }
  pinocchio::Model reduced;
  pinocchio::buildReducedModel(full, to_lock, seed, reduced);
  return reduced;
}

}  // namespace

// ── (1) cache 좌표 브릿지 + scatter 가 non-RT 핸들과 값 일치 (crank_rocker 비특이) ──
TEST(WbcReducedDynamicsProvider, ScatterMatchesConvergedHandle) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);

  // loop-consistent seed (crank=0.2 비특이).
  rub::ClosedChainHandle seedh(full, ccm.constraints, ccm.actuated_joint_ids, {});
  ASSERT_TRUE(seedh.Update(std::vector<double>{0.2}).converged);
  const Eigen::VectorXd seed = seedh.GetFullConfiguration();

  // 참조: 동일 파라미터 RtClosedChainHandle 로 축약 M/h/g.
  const double vval = 0.31;
  rub::RtClosedChainHandle ref(full, ccm.constraints, ccm.actuated_joint_ids, seed, 2);
  ASSERT_FALSE(ref.Update(std::vector<double>{0.2}).singular);
  ASSERT_FALSE(ref.UpdateDynamics(std::vector<double>{vval}).held);

  // control(actuated 등가) 모델 = j_crank 만 남김.
  const pinocchio::Model control = MakeControlModel(*full, seed, "j_crank");
  ASSERT_EQ(control.nv, 1) << "crank_rocker control 모델은 1-DoF (j_crank)";

  ib::WbcReducedDynamicsProvider provider;
  ASSERT_TRUE(provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
  ASSERT_EQ(provider.n_a(), 1);

  // cache 좌표 q/v (control 모델) 구성: j_crank 슬롯에 0.2 / vval.
  const auto jid = control.getJointId("j_crank");
  Eigen::VectorXd q = pinocchio::neutral(control);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(control.nv);
  q[static_cast<Eigen::Index>(control.idx_qs[jid])] = 0.2;
  v[static_cast<Eigen::Index>(control.idx_vs[jid])] = vval;

  // open-chain M/h/g 를 sentinel 로 채워, provider 가 실제로 덮는지 확인.
  Eigen::MatrixXd M = Eigen::MatrixXd::Constant(control.nv, control.nv, -99.0);
  Eigen::VectorXd h = Eigen::VectorXd::Constant(control.nv, -99.0);
  Eigen::VectorXd g = Eigen::VectorXd::Constant(control.nv, -99.0);

  ASSERT_TRUE(provider.FillReducedDynamics(q, v, M, h, g));

  const int vk = static_cast<int>(control.idx_vs[jid]);
  EXPECT_NEAR(M(vk, vk), ref.GetMassMatrix()(0, 0), 1e-9) << "M scatter";
  EXPECT_NEAR(h[vk], ref.GetNonLinearEffects()(0), 1e-9) << "h scatter";
  EXPECT_NEAR(g[vk], ref.GetGeneralizedGravity()(0), 1e-9) << "g scatter";
  EXPECT_GT(std::abs(M(vk, vk)), 1e-6) << "sentinel 이 실제 축약값으로 덮여야 한다";
}

// ── (2) 이름 기반 좌표 정렬 게이트: 매칭 성공→활성, 미매칭→비활성(fallback) ──────
TEST(WbcReducedDynamicsProvider, AlignmentGate) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle seedh(full, ccm.constraints, ccm.actuated_joint_ids, {});
  seedh.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = seedh.GetFullConfiguration();

  // 매칭: control 에 j_crank 존재 → 활성.
  {
    const pinocchio::Model control = MakeControlModel(*full, seed, "j_crank");
    ib::WbcReducedDynamicsProvider provider;
    EXPECT_TRUE(provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
    EXPECT_TRUE(provider.active());
  }
  // 미매칭: control nv 는 1 이지만 독립 관절(j_crank)이 없는 모델 → 정렬 실패 → 비활성.
  //   (control 로 crank 대신 다른 1-DoF 만 남긴 모델을 쓰면 n_a==nv 여도 이름 매칭이 실패한다.)
  {
    // full 에서 j_crank 를 잠그고 다른 movable 1개만 남긴다.
    std::string other;
    for (int j = 1; j < full->njoints; ++j) {
      const auto jidx = static_cast<std::size_t>(j);
      if (full->nvs[jidx] > 0 && full->names[jidx] != "j_crank") {
        other = full->names[jidx];
        break;
      }
    }
    ASSERT_FALSE(other.empty());
    const pinocchio::Model control = MakeControlModel(*full, seed, other);
    if (control.nv == 1) {  // 남긴 관절이 단일-DoF 일 때만 유효한 미매칭 시나리오
      ib::WbcReducedDynamicsProvider provider;
      EXPECT_FALSE(
          provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
      EXPECT_FALSE(provider.active());
      EXPECT_EQ(provider.missing_joint(), "j_crank") << "매칭 실패 관절 보고";
    }
  }
}

// ── (3) held tick(비유한 q) → 직전 유효(last-good) 축약값 유지 ────────────────────
TEST(WbcReducedDynamicsProvider, HeldHoldsLastGood) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle seedh(full, ccm.constraints, ccm.actuated_joint_ids, {});
  seedh.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = seedh.GetFullConfiguration();
  const pinocchio::Model control = MakeControlModel(*full, seed, "j_crank");

  ib::WbcReducedDynamicsProvider provider;
  ASSERT_TRUE(provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
  const auto jid = control.getJointId("j_crank");
  const int qi = static_cast<int>(control.idx_qs[jid]);
  const int vk = static_cast<int>(control.idx_vs[jid]);

  Eigen::VectorXd q = pinocchio::neutral(control);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(control.nv);
  q[qi] = 0.2;
  v[vk] = 0.1;
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(control.nv, control.nv);
  Eigen::VectorXd h = Eigen::VectorXd::Zero(control.nv);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(control.nv);
  ASSERT_TRUE(provider.FillReducedDynamics(q, v, M, h, g));
  const double good_M = M(vk, vk);

  // 비유한 q → 핸들 held → last-good 유지 (open-chain 로 안 떨어짐).
  Eigen::MatrixXd M2 = Eigen::MatrixXd::Constant(control.nv, control.nv, -99.0);
  Eigen::VectorXd h2 = Eigen::VectorXd::Constant(control.nv, -99.0);
  Eigen::VectorXd g2 = Eigen::VectorXd::Constant(control.nv, -99.0);
  q[qi] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(provider.FillReducedDynamics(q, v, M2, h2, g2)) << "held 여도 last-good 로 true";
  EXPECT_NEAR(M2(vk, vk), good_M, 1e-12) << "held tick 은 직전 유효 M 유지";
}

// ── (4) 최초 tick held / 구속 없음 → open-chain fallback (false, 미변경) ──────────
TEST(WbcReducedDynamicsProvider, InactiveAndFirstTickFallback) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);

  // 구속 없음 → 비활성 → FillReducedDynamics 는 false (open-chain).
  {
    const pinocchio::Model control = *full;
    ib::WbcReducedDynamicsProvider provider;
    EXPECT_FALSE(provider.Configure(full, /*constraints=*/{}, /*actuated=*/{}, {}, control));
    Eigen::MatrixXd M = Eigen::MatrixXd::Constant(full->nv, full->nv, 7.0);
    Eigen::VectorXd h = Eigen::VectorXd::Constant(full->nv, 7.0);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(full->nv, 7.0);
    EXPECT_FALSE(provider.FillReducedDynamics(pinocchio::neutral(*full),
                                              Eigen::VectorXd::Zero(full->nv), M, h, g));
    EXPECT_EQ(M(0, 0), 7.0) << "비활성이면 M 미변경 (open-chain 유지)";
  }

  // 활성이지만 최초 tick 이 held(비유한 q) → last-good 없음 → false (open-chain).
  {
    rub::ClosedChainHandle seedh(full, ccm.constraints, ccm.actuated_joint_ids, {});
    seedh.Update(std::vector<double>{0.2});
    const Eigen::VectorXd seed = seedh.GetFullConfiguration();
    const pinocchio::Model control = MakeControlModel(*full, seed, "j_crank");
    ib::WbcReducedDynamicsProvider provider;
    ASSERT_TRUE(provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
    const auto jid = control.getJointId("j_crank");
    Eigen::VectorXd q = pinocchio::neutral(control);
    q[static_cast<Eigen::Index>(control.idx_qs[jid])] = std::numeric_limits<double>::quiet_NaN();
    Eigen::MatrixXd M = Eigen::MatrixXd::Constant(control.nv, control.nv, 7.0);
    Eigen::VectorXd h = Eigen::VectorXd::Constant(control.nv, 7.0);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(control.nv, 7.0);
    EXPECT_FALSE(provider.FillReducedDynamics(q, Eigen::VectorXd::Zero(control.nv), M, h, g));
    EXPECT_EQ(M(0, 0), 7.0) << "최초 held → open-chain 유지 (미변경)";
  }
}
