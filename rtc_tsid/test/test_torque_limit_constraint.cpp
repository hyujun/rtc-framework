#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/torque_limit_constraint.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

// Fixture: build the Panda model once per test in SetUp(), shared via model_ /
// info_. Each TEST_F gets a fresh fixture instance, so behaviour is identical
// to the previous per-TEST inline build — only the boilerplate is removed.
class TorqueLimitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model_);
    YAML::Node config;
    info_.Build(*model_, config);
  }

  std::shared_ptr<pinocchio::Model> model_;
  RobotModelInfo info_;
};

TEST_F(TorqueLimitTest, Dimensions) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  mgr.max_contact_vars = 0;

  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(0);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  tl.Init(*model, info, cache, cfg);

  EXPECT_EQ(tl.EqDim(cs), 0);
  EXPECT_EQ(tl.IneqDim(cs), info.n_actuated);
}

TEST_F(TorqueLimitTest, BoundsConsistency) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(0);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  tl.Init(*model, info, cache, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.Update(q, v, cs);

  const int n_vars = info.nv;
  const int n_ineq = tl.IneqDim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();

  tl.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  // C = S·M (should be non-zero)
  EXPECT_GT(C.norm(), 0.0);

  // l < u (torque limits: tau_min < tau_max assumed)
  for (int i = 0; i < n_ineq; ++i) {
    EXPECT_LT(l(i), u(i)) << "Joint " << i;
  }
}

// A-1: YAML tau_scale 적용 — bound 가 정확히 tau_scale 배가 되는지.
TEST_F(TorqueLimitTest, TauScaleAppliesMargin) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(0);

  // 첫째: tau_scale=1.0 (default) — baseline bound 기록
  TorqueLimitConstraint tl1;
  YAML::Node cfg1;
  tl1.Init(*model, info, cache, cfg1);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.Update(q, v, cs);

  const int n_vars = info.nv;
  const int n_ineq = tl1.IneqDim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l1(n_ineq), u1(n_ineq);
  C.setZero();
  l1.setZero();
  u1.setZero();
  tl1.ComputeInequality(cache, cs, info, n_vars, C, l1, u1);

  // 둘째: tau_scale=0.5 — bound 가 정확히 절반 magnitude 가 되는지.
  // l = 0.5·τ_min - S·h,  u = 0.5·τ_max - S·h.
  // baseline (scale=1): l = τ_min - S·h, u = τ_max - S·h.
  // → diff_l = l(0.5) - l(1) = -0.5·τ_min, diff_u = u(0.5) - u(1) = -0.5·τ_max.
  TorqueLimitConstraint tl2;
  YAML::Node cfg2;
  cfg2["tau_scale"] = 0.5;
  tl2.Init(*model, info, cache, cfg2);

  Eigen::VectorXd l2(n_ineq), u2(n_ineq);
  C.setZero();
  l2.setZero();
  u2.setZero();
  tl2.ComputeInequality(cache, cs, info, n_vars, C, l2, u2);

  // tau_min < 0, tau_max > 0 가정 (URDF effort_limit 의 부호).
  // diff = (scale - 1) · tau_lim. scale=0.5 → diff_l = -0.5·tau_min (positive),
  // diff_u = -0.5·tau_max (negative).
  for (int i = 0; i < n_ineq; ++i) {
    EXPECT_NEAR(l2(i) - l1(i), -0.5 * info.tau_min(i), 1e-9) << "Joint " << i;
    EXPECT_NEAR(u2(i) - u1(i), -0.5 * info.tau_max(i), 1e-9) << "Joint " << i;
  }
}

// A-1 (guide §2): No-contact 시 inequality matrix·boundary 가 직접 계산식과 정확히
// 일치하는지 — random a 100 개에 대해 (C·z)_τ + S·h = S·M·a, l + S·h = τ_min,
// u + S·h = τ_max.
TEST_F(TorqueLimitTest, NoContact_MatchesDirectComputation) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  mgr.max_contact_vars = 0;
  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(0);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  tl.Init(*model, info, cache, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.Update(q, v, cs);

  const int n_vars = info.nv;  // no contact → n_vars = nv
  const int n_ineq = tl.IneqDim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  tl.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  const Eigen::VectorXd Sh = info.S * cache.h;
  const Eigen::MatrixXd SM = info.S * cache.M;

  // l + S·h ≈ τ_min,  u + S·h ≈ τ_max
  EXPECT_LT((l + Sh - info.tau_min).norm(), 1e-9);
  EXPECT_LT((u + Sh - info.tau_max).norm(), 1e-9);

  // C·a ≈ S·M·a for random a (100 samples).
  std::srand(42);
  for (int trial = 0; trial < 100; ++trial) {
    Eigen::VectorXd a = Eigen::VectorXd::Random(info.nv);
    const Eigen::VectorXd Cz = C * a;
    const Eigen::VectorXd direct = SM * a;
    EXPECT_LT((Cz - direct).norm(), 1e-9) << "trial=" << trial;
  }
}

// A-1 (guide §2): Contact active 시 -S·Jcᵀ block 이 직접 계산과 일치 — random
// (a, λ) 100 개에 대해 C·z + S·h ≈ S·(M·a + h - Jcᵀ·λ).
TEST_F(TorqueLimitTest, WithContact_MatchesDirectComputation) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  // Find link7 frame for point contact.
  std::string frame_name;
  for (size_t i = 0; i < model->frames.size(); ++i) {
    if (model->frames[i].name.find("link7") != std::string::npos) {
      frame_name = model->frames[i].name;
      break;
    }
  }
  ASSERT_FALSE(frame_name.empty());

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].name = "ee";
  mgr.contacts[0].frame_name = frame_name;
  mgr.contacts[0].frame_id = static_cast<int>(model->getFrameId(frame_name));
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.7;
  mgr.contacts[0].friction_faces = 4;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  tl.Init(*model, info, cache, cfg);
  tl.SetContactManager(&mgr);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.Update(q, v, cs);

  const int cdim = mgr.contacts[0].contact_dim;
  const int n_vars = info.nv + cs.active_contact_vars;
  ASSERT_EQ(n_vars, info.nv + cdim);

  const int n_ineq = tl.IneqDim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  tl.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  const Eigen::VectorXd Sh = info.S * cache.h;
  const Eigen::MatrixXd SM = info.S * cache.M;
  const Eigen::MatrixXd SJcT =
      info.S * cache.contact_frames[0].J.topRows(cdim).transpose();  // [na × cdim]

  // C[:, nv:nv+cdim] ≈ -S·Jcᵀ.
  EXPECT_LT((C.rightCols(cdim) + SJcT).norm(), 1e-9);

  std::srand(123);
  for (int trial = 0; trial < 100; ++trial) {
    Eigen::VectorXd a = Eigen::VectorXd::Random(info.nv);
    Eigen::VectorXd lambda = Eigen::VectorXd::Random(cdim);
    Eigen::VectorXd z(n_vars);
    z.head(info.nv) = a;
    z.tail(cdim) = lambda;

    const Eigen::VectorXd Cz = C * z;
    // direct: S·(M·a + h - Jcᵀ·λ) - S·h = S·M·a - S·Jcᵀ·λ
    const Eigen::VectorXd direct = SM * a - SJcT * lambda;
    EXPECT_LT((Cz - direct).norm(), 1e-9) << "trial=" << trial;
  }
}

// A-1 (guide §2): tau_scale 을 매우 작게 잡으면 feasible (a, λ) 영역이 좁아진다
// — bound width (u - l) 가 tau_scale 에 비례해서 작아지는지 직접 확인. matrix-level
// 검증 (QP solver 미사용).
TEST_F(TorqueLimitTest, TightBoundsReduceFeasibleRegion) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(0);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.Update(q, v, cs);

  // tau_scale=1.0 baseline.
  TorqueLimitConstraint tl_baseline;
  YAML::Node cfg_baseline;
  tl_baseline.Init(*model, info, cache, cfg_baseline);
  const int n_ineq = tl_baseline.IneqDim(cs);
  Eigen::MatrixXd C1(n_ineq, info.nv);
  Eigen::VectorXd l1(n_ineq), u1(n_ineq);
  C1.setZero();
  l1.setZero();
  u1.setZero();
  tl_baseline.ComputeInequality(cache, cs, info, info.nv, C1, l1, u1);

  // tau_scale=0.01 — 100배 tight.
  TorqueLimitConstraint tl_tight;
  YAML::Node cfg_tight;
  cfg_tight["tau_scale"] = 0.01;
  tl_tight.Init(*model, info, cache, cfg_tight);
  Eigen::MatrixXd C2(n_ineq, info.nv);
  Eigen::VectorXd l2(n_ineq), u2(n_ineq);
  C2.setZero();
  l2.setZero();
  u2.setZero();
  tl_tight.ComputeInequality(cache, cs, info, info.nv, C2, l2, u2);

  // C matrix 는 tau_scale 과 무관 (S·M 만 들어감).
  EXPECT_LT((C2 - C1).norm(), 1e-12);

  // Width: (u - l) = (τ_max - τ_min)·tau_scale. tight 는 baseline 의 1% 여야 한다.
  for (int i = 0; i < n_ineq; ++i) {
    const double w1 = u1(i) - l1(i);
    const double w2 = u2(i) - l2(i);
    ASSERT_GT(w1, 0.0) << "Joint " << i;
    EXPECT_NEAR(w2 / w1, 0.01, 1e-9) << "Joint " << i;
  }
}

// A-1 (guide §2): tau_scale=1.0 (default) 면 bound 가 raw URDF effort_limit 그대로 —
// "unconstrained baseline" 과 등가. baseline 보다 더 큰 magnitude 의 a 가 통과할 여지
// 가 있는지 확인.
TEST_F(TorqueLimitTest, LooseBoundsMatchRawEffortLimits) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  PinocchioCache cache;
  cache.Init(model, mgr);
  ContactState cs;
  cs.Init(0);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.Update(q, v, cs);

  TorqueLimitConstraint tl;
  YAML::Node cfg;  // tau_scale 미설정 → 1.0 default.
  tl.Init(*model, info, cache, cfg);

  const int n_ineq = tl.IneqDim(cs);
  Eigen::MatrixXd C(n_ineq, info.nv);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  tl.ComputeInequality(cache, cs, info, info.nv, C, l, u);

  const Eigen::VectorXd Sh = info.S * cache.h;
  // raw bound: l + S·h = τ_min, u + S·h = τ_max.
  for (int i = 0; i < n_ineq; ++i) {
    EXPECT_NEAR(l(i) + Sh(i), info.tau_min(i), 1e-9) << "Joint " << i;
    EXPECT_NEAR(u(i) + Sh(i), info.tau_max(i), 1e-9) << "Joint " << i;
  }

  // tau_scale=0.5 와 비교 — loose (1.0) 의 width 가 정확히 2배.
  TorqueLimitConstraint tl_half;
  YAML::Node cfg_half;
  cfg_half["tau_scale"] = 0.5;
  tl_half.Init(*model, info, cache, cfg_half);
  Eigen::MatrixXd Ch(n_ineq, info.nv);
  Eigen::VectorXd lh(n_ineq), uh(n_ineq);
  Ch.setZero();
  lh.setZero();
  uh.setZero();
  tl_half.ComputeInequality(cache, cs, info, info.nv, Ch, lh, uh);

  for (int i = 0; i < n_ineq; ++i) {
    const double w_loose = u(i) - l(i);
    const double w_half = uh(i) - lh(i);
    EXPECT_NEAR(w_loose / w_half, 2.0, 1e-9) << "Joint " << i;
  }
}

// A-1: tau_scale 범위 위반 시 init throw.
TEST_F(TorqueLimitTest, TauScaleOutOfRangeThrows) {
  auto& model = model_;
  RobotModelInfo& info = info_;
  ContactManagerConfig mgr;
  PinocchioCache cache;
  cache.Init(model, mgr);

  // > 1: reject
  {
    YAML::Node cfg;
    cfg["tau_scale"] = 1.5;
    TorqueLimitConstraint tl;
    EXPECT_THROW(tl.Init(*model, info, cache, cfg), std::runtime_error);
  }
  // 0: reject (boundary)
  {
    YAML::Node cfg;
    cfg["tau_scale"] = 0.0;
    TorqueLimitConstraint tl;
    EXPECT_THROW(tl.Init(*model, info, cache, cfg), std::runtime_error);
  }
  // < 0: reject
  {
    YAML::Node cfg;
    cfg["tau_scale"] = -0.5;
    TorqueLimitConstraint tl;
    EXPECT_THROW(tl.Init(*model, info, cache, cfg), std::runtime_error);
  }
  // = 1.0: accept (boundary)
  {
    YAML::Node cfg;
    cfg["tau_scale"] = 1.0;
    TorqueLimitConstraint tl;
    EXPECT_NO_THROW(tl.Init(*model, info, cache, cfg));
  }
}

}  // namespace
}  // namespace rtc::tsid
