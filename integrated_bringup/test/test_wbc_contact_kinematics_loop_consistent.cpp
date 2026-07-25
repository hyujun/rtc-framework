// ── test_wbc_contact_kinematics_loop_consistent — Phase ③ contact frame J·oMf 격상 ──
//   WbcReducedDynamicsProvider::FillReducedFrameKinematics 검증: closed-chain 활성 시 loop-하류
//   contact frame 의 J·oMf·dJv 를 loop-consistent 값으로 덮되 frozen-loop(open-chain) 은 안 쓴다.
//   (1) 하류 frame override 가 non-RT 수렴 RtClosedChainHandle 과 일치 + frozen-loop 대비 유의미
//   차이, (2) dJv 가 loop-consistent drift 와 일치 (L2-exact, #173), (3) held tick → J/oMf/dJv
//   3값 last-good hold (L1, open-chain 미주입), (4) 구속 없음(serial) → override 없음
//   (byte-for-byte). 통합 경로는 PinocchioCache::Update 로 재현한다.
#include "closure_test_fixtures.hpp"  // rtc_urdf_bridge/test (include dir via CMake)
#include "integrated_bringup/support/wbc_reduced_dynamics_provider.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
namespace ib = integrated_bringup;

namespace {

// crank_rocker: j_crank 만 남기고 나머지 movable 을 seed 형상에서 잠근 control(actuated 등가) 모델.
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

// crank=seed_crank 에서 loop-consistent seed 를 만든다.
Eigen::VectorXd LoopConsistentSeed(const std::shared_ptr<pinocchio::Model>& full,
                                   const rub::ClosedChainModel& ccm, double seed_crank) {
  rub::ClosedChainHandle seedh(full, ccm.constraints, ccm.actuated_joint_ids, {});
  seedh.Update(std::vector<double>{seed_crank});
  return seedh.GetFullConfiguration();
}

// [#248] seed(0.2)→eval(0.3) 은 Δ=0.1 rad 로 RtClosedChainHandle 의 tick 당 seed 증분 상한
// (0.05)을 넘는다 → 첫 tick 은 클램프되어 held 다. 프로덕션 RT loop 과 동일하게 held 가 풀릴
// 때까지 tick 을 돌려 평가점에 도달시킨다. `ref` 와 provider 내부 핸들은 seed·K·입력열이
// 같으므로 **같은 tick 수**를 돌려야 두 사영이 동일 형상에서 비교된다 (고정 K 잔차까지 일치).
constexpr int kWalkInTicks = 8;  // ⌈0.1/0.05⌉=2 + settle

}  // namespace

// ── (1) 하류 contact frame override 가 수렴 핸들과 일치 + frozen-loop 대비 유의미 차이 ─────
//   seed 는 crank=0.2 loop-consistent, 평가는 crank=0.3 (seed 이탈) → frozen-loop 근사가 갈라진다.
TEST(WbcContactKinematicsLoopConsistent, DownstreamOverrideMatchesConvergedNotFrozen) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);
  const double seed_crank = 0.2;
  const double eval_crank = 0.3;
  const double crank_vel = 0.37;
  const Eigen::VectorXd seed = LoopConsistentSeed(full, ccm, seed_crank);

  const pinocchio::Model control = MakeControlModel(*full, seed, "j_crank");
  ASSERT_EQ(control.nv, 1);
  ASSERT_TRUE(control.existFrame("c1")) << "c1 contact frame 이 control 모델에 보존돼야 한다";
  const pinocchio::FrameIndex c1_ctrl = control.getFrameId("c1");

  // 참조: 동일 파라미터 RtClosedChainHandle 로 loop-consistent J·oMf (full 모델).
  rub::RtClosedChainHandle ref(full, ccm.constraints, ccm.actuated_joint_ids, seed, 2);
  rub::RtClosedChainHandle::Status st_ref{};
  for (int t = 0; t < kWalkInTicks; ++t) {  // seed 증분 클램프 walk-in (#248)
    st_ref = ref.Update(std::vector<double>{eval_crank});
  }
  ASSERT_FALSE(st_ref.held) << kWalkInTicks << " tick 뒤에도 held — walk-in 미완료";
  ASSERT_FALSE(st_ref.singular);
  const pinocchio::FrameIndex c1_full = ref.GetFrameId("c1");
  ASSERT_NE(c1_full, static_cast<pinocchio::FrameIndex>(0));
  ASSERT_TRUE(ref.IsFrameDownstreamOfLoop(c1_full)) << "c1 은 loop-하류";
  Eigen::MatrixXd J_ref(6, 1);
  ref.GetFrameJacobian(c1_full, pinocchio::LOCAL_WORLD_ALIGNED, J_ref);
  const pinocchio::SE3 oMf_ref = ref.GetFramePlacement(c1_full);
  // 같은 tick 의 UpdateDynamics 로 loop-consistent drift (provider 경로와 동일 순서).
  ASSERT_FALSE(ref.UpdateDynamics(std::vector<double>{crank_vel}).held);
  Eigen::Matrix<double, 6, 1> dJv_ref;
  ref.GetFrameClassicalAccelerationDrift(c1_full, pinocchio::LOCAL_WORLD_ALIGNED, dJv_ref);

  // provider 구성 (M/h/g + contact frame 매핑).
  ib::WbcReducedDynamicsProvider provider;
  ASSERT_TRUE(provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
  provider.ConfigureContactFrames(control, std::vector<pinocchio::FrameIndex>{c1_ctrl});

  // 통합 경로: PinocchioCache(control 모델) 로 open-chain 계산 → provider override.
  auto control_ptr = std::make_shared<pinocchio::Model>(control);
  rub::PinocchioCache cache;
  cache.Init(control_ptr, std::vector<pinocchio::FrameIndex>{c1_ctrl});

  const auto jid = control.getJointId("j_crank");
  Eigen::VectorXd q = pinocchio::neutral(control);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(control.nv);
  q[static_cast<Eigen::Index>(control.idx_qs[jid])] = eval_crank;
  v[static_cast<Eigen::Index>(control.idx_vs[jid])] = crank_vel;

  // provider 미배선 → open-chain(frozen-loop) 스냅샷.
  cache.Update(q, v);
  const Eigen::MatrixXd J_open = cache.contact_frames[0].J;
  const pinocchio::SE3 oMf_open = cache.contact_frames[0].oMf;
  ASSERT_GT(cache.contact_frames[0].dJv.norm(), 1e-9) << "open-chain dJv 는 v≠0 에서 비영";

  // provider 배선 → 하류 frame override. provider 내부 핸들도 `ref` 와 같은 이유로 walk-in 이
  // 필요하며, 동일 tick 수를 돌려야 두 사영이 같은 형상에서 비교된다 (#248).
  cache.reduced_provider = &provider;
  for (int t = 0; t < kWalkInTicks; ++t) {
    cache.Update(q, v);
  }
  const Eigen::MatrixXd J_after = cache.contact_frames[0].J;
  const pinocchio::SE3 oMf_after = cache.contact_frames[0].oMf;

  // control.nv==1 → cache_v_idx 순열이 항등(단일 열) → J_after == J_ref 직접 비교.
  EXPECT_LT((J_after - J_ref).norm(), 1e-9) << "override J 가 수렴 핸들과 일치";
  EXPECT_LT((oMf_after.translation() - oMf_ref.translation()).norm(), 1e-9) << "override oMf 일치";
  EXPECT_LT((oMf_after.rotation() - oMf_ref.rotation()).norm(), 1e-9) << "override oMf 회전 일치";

  // frozen-loop 대비 유의미 차이 (정량화). seed 이탈 crank 에서 loop 이 갈라진다.
  const double dJ = (J_after - J_open).norm();
  const double dp = (oMf_after.translation() - oMf_open.translation()).norm();
  std::cout << "[ Phase3   ] frozen-loop vs loop-consistent @crank=" << eval_crank
            << " (seed=" << seed_crank << "): Δ‖J‖=" << dJ << ", Δp=" << dp << " m\n";
  EXPECT_GT(dJ, 1e-4) << "loop-consistent J 가 frozen-loop 과 유의미하게 다름 (Δ‖J‖=" << dJ << ")";
  EXPECT_GT(dp, 1e-4) << "loop-consistent oMf 가 frozen-loop 과 유의미하게 다름 (Δp=" << dp << ")";

  // (2) L2-exact (#173): 하류 frame dJv 는 loop-consistent drift J̇_a·v_a 로 주입.
  //   (L2-zero 잠정안의 dJv=0 assertion 교체 — spec 변경, issue #173 본문.)
  EXPECT_GT(dJv_ref.norm(), 1e-9) << "v≠0 → loop-consistent drift 비영 (공허 통과 방지)";
  EXPECT_LT((cache.contact_frames[0].dJv - dJv_ref).norm(), 1e-9)
      << "하류 dJv 가 수렴 핸들 drift 와 일치 (L2-exact)";
}

// ── (3) held tick(비유한 q) → 직전 유효 loop-consistent J/oMf/dJv 3값 유지 (L1+F3) ──────
TEST(WbcContactKinematicsLoopConsistent, HeldHoldsLastGoodNotFrozen) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = LoopConsistentSeed(full, ccm, 0.2);
  const pinocchio::Model control = MakeControlModel(*full, seed, "j_crank");
  const pinocchio::FrameIndex c1_ctrl = control.getFrameId("c1");

  ib::WbcReducedDynamicsProvider provider;
  ASSERT_TRUE(provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control));
  provider.ConfigureContactFrames(control, std::vector<pinocchio::FrameIndex>{c1_ctrl});

  auto control_ptr = std::make_shared<pinocchio::Model>(control);
  rub::PinocchioCache cache;
  cache.Init(control_ptr, std::vector<pinocchio::FrameIndex>{c1_ctrl});
  cache.reduced_provider = &provider;

  const auto jid = control.getJointId("j_crank");
  const int qi = static_cast<int>(control.idx_qs[jid]);
  Eigen::VectorXd q = pinocchio::neutral(control);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(control.nv);
  q[qi] = 0.3;
  v[static_cast<Eigen::Index>(control.idx_vs[jid])] = 0.37;  // 비영 → dJv last-good 이 비자명

  // fresh tick → last-good 갱신 (J/oMf/dJv 3값 snapshot).
  // seed(0.2)→0.3 은 tick 당 seed 증분 상한을 넘으므로 walk-in 이 끝나야 fresh 가 된다 (#248).
  for (int t = 0; t < kWalkInTicks; ++t) {
    cache.Update(q, v);
  }
  const Eigen::MatrixXd J_good = cache.contact_frames[0].J;
  const pinocchio::SE3 oMf_good = cache.contact_frames[0].oMf;
  const Eigen::Matrix<double, 6, 1> dJv_good = cache.contact_frames[0].dJv;
  ASSERT_GT(dJv_good.norm(), 1e-9) << "비영 v → fresh dJv 비영 (hold 검증이 비자명하도록)";

  // 비유한 q → 핸들 held → last-good 유지 (open-chain frozen-loop 로 안 떨어짐).
  q[qi] = std::numeric_limits<double>::quiet_NaN();
  cache.Update(q, v);
  // open-chain 계산은 NaN 이 섞이지만 override 가 last-good 로 덮으므로 유한·직전값.
  EXPECT_TRUE(cache.contact_frames[0].J.allFinite()) << "held tick J 유한 (NaN 누출 없음)";
  EXPECT_LT((cache.contact_frames[0].J - J_good).norm(), 1e-12) << "held → 직전 유효 J 유지";
  EXPECT_LT((cache.contact_frames[0].oMf.translation() - oMf_good.translation()).norm(), 1e-12)
      << "held → 직전 유효 oMf 유지";
  EXPECT_TRUE(cache.contact_frames[0].dJv.allFinite()) << "held tick dJv 유한 (NaN 누출 없음)";
  EXPECT_LT((cache.contact_frames[0].dJv - dJv_good).norm(), 1e-12)
      << "held → 직전 유효 dJv 유지 (J/oMf/dJv 3값 snapshot 단위, #173 F3)";
}

// ── (4) 구속 없음(serial) → override 없음, contact frame byte-for-byte 미변경 ───────────
TEST(WbcContactKinematicsLoopConsistent, SerialNoOverrideByteForByte) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);

  // 구속 없이 Configure → 비활성.
  ib::WbcReducedDynamicsProvider provider;
  EXPECT_FALSE(provider.Configure(full, /*constraints=*/{}, /*actuated=*/{}, {}, *full));
  provider.ConfigureContactFrames(*full,
                                  std::vector<pinocchio::FrameIndex>{full->getFrameId("c1")});

  // sentinel contact frame 을 직접 구성해 FillReducedFrameKinematics 가 안 건드리는지 확인.
  std::vector<rub::PinocchioCache::FrameCache> frames(1);
  frames[0].frame_id = full->getFrameId("c1");
  frames[0].J = Eigen::MatrixXd::Constant(6, full->nv, -99.0);
  frames[0].oMf = pinocchio::SE3::Identity();
  frames[0].oMf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  frames[0].dJv.setConstant(-7.0);
  const Eigen::MatrixXd J_before = frames[0].J;

  const Eigen::VectorXd q = pinocchio::neutral(*full);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(full->nv);
  EXPECT_FALSE(provider.FillReducedFrameKinematics(q, v, frames)) << "비활성 → override 없음";
  EXPECT_EQ((frames[0].J - J_before).norm(), 0.0) << "serial → J byte-for-byte 미변경";
  EXPECT_EQ(frames[0].oMf.translation(), Eigen::Vector3d(1.0, 2.0, 3.0)) << "oMf 미변경";
  EXPECT_EQ(frames[0].dJv.sum(), -7.0 * 6.0) << "dJv 미변경";
}
