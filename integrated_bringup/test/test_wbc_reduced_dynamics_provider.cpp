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

// ── (#175) 사영 view — fingertip FK 가 같은 tick 의 사영을 재사용하기 위한 창구 ────
//   아래 세 테스트가 P1 의 계약이다: (5) view 가 provider 사영과 같은 형상을 준다,
//   (6) dynamics 사유 held 가 운동학 status 를 오염시키지 않는다, (7) 사영 실행이 세어진다.

namespace {

/// crank_rocker 고정물 + 활성 provider 한 벌. 신규 테스트 3개가 같은 배선을 쓰므로 묶는다.
struct CrankRockerProviderFixture {
  rub::ClosedChainModel ccm{rtc::test::CrankRocker()};
  std::shared_ptr<pinocchio::Model> full{std::make_shared<pinocchio::Model>(ccm.model)};
  Eigen::VectorXd seed;
  pinocchio::Model control;
  ib::WbcReducedDynamicsProvider provider;
  int qi{0};
  int vk{0};

  CrankRockerProviderFixture() {
    rub::ClosedChainHandle seedh(full, ccm.constraints, ccm.actuated_joint_ids, {});
    seedh.Update(std::vector<double>{0.2});
    seed = seedh.GetFullConfiguration();
    control = MakeControlModel(*full, seed, "j_crank");
    const auto jid = control.getJointId("j_crank");
    qi = static_cast<int>(control.idx_qs[jid]);
    vk = static_cast<int>(control.idx_vs[jid]);
  }

  [[nodiscard]] bool Configure() {
    return provider.Configure(full, ccm.constraints, ccm.actuated_joint_ids, seed, control);
  }

  /// q_a=crank, v_a=vel 로 한 tick 돌린다 (M/h/g 는 버린다 — 여기 관심사는 사영/뷰다).
  bool Tick(double crank, double vel) {
    Eigen::VectorXd q = pinocchio::neutral(control);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(control.nv);
    q[qi] = crank;
    v[vk] = vel;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(control.nv, control.nv);
    Eigen::VectorXd h = Eigen::VectorXd::Zero(control.nv);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(control.nv);
    return provider.FillReducedDynamics(q, v, M, h, g);
  }
};

}  // namespace

// ── (5) view 는 비활성이면 nullptr, 활성이면 provider 가 방금 돌린 사영 형상 ──────
TEST(WbcReducedDynamicsProvider, ProjectionViewExposesTheSameProjection) {
  // 비활성(구속 없음) → nullptr. 소비자는 이것으로 configure-time fallback 을 판정한다.
  {
    const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
    auto full = std::make_shared<pinocchio::Model>(ccm.model);
    const pinocchio::Model control = *full;
    ib::WbcReducedDynamicsProvider provider;
    EXPECT_FALSE(provider.Configure(full, /*constraints=*/{}, /*actuated=*/{}, {}, control));
    EXPECT_EQ(provider.projection(), nullptr) << "비활성 provider 는 사영을 노출하지 않는다";
  }

  CrankRockerProviderFixture fx;
  ASSERT_TRUE(fx.Configure());
  ASSERT_NE(fx.provider.projection(), nullptr);

  const double vval = 0.31;
  ASSERT_TRUE(fx.Tick(0.2, vval));

  // 동일 파라미터 참조 핸들 — 같은 seed·같은 q_a 이므로 결과는 **비트 동일**이어야 한다.
  // (같은 코드 경로·같은 연산 순서. 근사 비교로 두면 view 가 엉뚱한 tick 의 형상을 줘도 통과한다.)
  rub::RtClosedChainHandle ref(fx.full, fx.ccm.constraints, fx.ccm.actuated_joint_ids, fx.seed, 2);
  ASSERT_FALSE(ref.Update(std::vector<double>{0.2}).held);
  ASSERT_FALSE(ref.UpdateDynamics(std::vector<double>{vval}).held);

  const auto fid = static_cast<pinocchio::FrameIndex>(fx.full->frames.size() - 1);
  const pinocchio::SE3& seen = fx.provider.projection()->GetFramePlacement(fid);
  const pinocchio::SE3& want = ref.GetFramePlacement(fid);
  EXPECT_EQ(seen.translation(), want.translation()) << "view placement 는 참조 사영과 비트 동일";
  EXPECT_EQ(seen.rotation(), want.rotation());

  // 사영된 full configuration 도 같아야 한다 (view 가 다른 tick 의 형상을 들고 있지 않다는 증거).
  EXPECT_EQ(fx.provider.projection()->GetFullConfiguration(), ref.GetFullConfiguration());
}

// ── (6) 유한 q + 비유한 v: 공용 status 는 held 여도 **운동학** status 는 성공 ─────
//   이것이 #175 의 D4 다. fingertip pose 는 dynamics 사유로 버려지면 안 된다 — 핸들이 같은
//   q_full_ 로 2차 FK 를 복원하므로 그 tick 의 placement 는 유효하기 때문.
TEST(WbcReducedDynamicsProvider, KinematicStatusSurvivesNonFiniteVelocity) {
  CrankRockerProviderFixture fx;
  ASSERT_TRUE(fx.Configure());

  EXPECT_FALSE(fx.Tick(0.2, std::numeric_limits<double>::quiet_NaN()))
      << "최초 tick 의 dynamics 가 held → last-good 없음 → open-chain fallback";

  ASSERT_NE(fx.provider.projection(), nullptr);
  EXPECT_TRUE(fx.provider.projection()->GetStatus().held)
      << "공용 status 는 UpdateDynamics 사유로 held";
  EXPECT_FALSE(fx.provider.kinematic_status().held)
      << "운동학 사영 자체는 성공했다 — 스냅샷이 dynamics hold 를 먹으면 fingertip 이 죽는다";
  EXPECT_TRUE(std::isfinite(fx.provider.kinematic_status().closure_error))
      << "held 로 세팅되는 inf closure_error 가 스냅샷에 새면 안 된다";

  // 그리고 placement 는 실제로 이번 q 의 것이다 — 운동학만 돌린 참조와 비트 동일.
  rub::RtClosedChainHandle ref(fx.full, fx.ccm.constraints, fx.ccm.actuated_joint_ids, fx.seed, 2);
  ASSERT_FALSE(ref.Update(std::vector<double>{0.2}).held);
  const auto fid = static_cast<pinocchio::FrameIndex>(fx.full->frames.size() - 1);
  EXPECT_EQ(fx.provider.projection()->GetFramePlacement(fid).translation(),
            ref.GetFramePlacement(fid).translation())
      << "NaN v tick 에도 fingertip 이 읽을 placement 는 이번 tick 형상";
}

// ── (7) 사영 실행 카운터 — "이번 tick 에 사영이 돌았는가" 의 증거 ─────────────────
TEST(WbcReducedDynamicsProvider, ProjectionSeqCountsEveryProjection) {
  CrankRockerProviderFixture fx;
  ASSERT_TRUE(fx.Configure());
  EXPECT_EQ(fx.provider.projection_seq(), 0U) << "Configure 는 카운터를 되돌린다";

  ASSERT_TRUE(fx.Tick(0.2, 0.1));
  EXPECT_EQ(fx.provider.projection_seq(), 1U) << "tick 당 사영 1회";
  ASSERT_TRUE(fx.Tick(0.21, 0.1));
  EXPECT_EQ(fx.provider.projection_seq(), 2U);

  // held tick 도 사영은 돌았다 (핸들 Update 는 호출된다) — 소비자는 seq 증가만으로 fresh 를
  // 결론지으면 안 되고, 입력 provenance 를 따로 봐야 한다 (#175 D5).
  ASSERT_TRUE(fx.Tick(std::numeric_limits<double>::quiet_NaN(), 0.1));
  EXPECT_EQ(fx.provider.projection_seq(), 3U) << "held tick 도 사영 실행은 1회로 센다";

  // 비활성 provider 는 사영하지 않으므로 증가하지 않는다.
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto full = std::make_shared<pinocchio::Model>(ccm.model);
  const pinocchio::Model control = *full;
  ib::WbcReducedDynamicsProvider inactive;
  EXPECT_FALSE(inactive.Configure(full, /*constraints=*/{}, /*actuated=*/{}, {}, control));
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(full->nv, full->nv);
  Eigen::VectorXd h = Eigen::VectorXd::Zero(full->nv);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(full->nv);
  EXPECT_FALSE(inactive.FillReducedDynamics(pinocchio::neutral(*full),
                                            Eigen::VectorXd::Zero(full->nv), M, h, g));
  EXPECT_EQ(inactive.projection_seq(), 0U) << "비활성은 사영하지 않는다";
}
