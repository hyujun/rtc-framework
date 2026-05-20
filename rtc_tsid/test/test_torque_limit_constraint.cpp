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

TEST(TorqueLimitTest, Dimensions) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(kPandaUrdf, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.build(*model, config);

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  mgr.max_contact_vars = 0;

  PinocchioCache cache;
  cache.init(model, mgr);
  ContactState cs;
  cs.init(0);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  tl.init(*model, info, cache, cfg);

  EXPECT_EQ(tl.eq_dim(cs), 0);
  EXPECT_EQ(tl.ineq_dim(cs), info.n_actuated);
}

TEST(TorqueLimitTest, BoundsConsistency) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(kPandaUrdf, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.build(*model, config);

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  PinocchioCache cache;
  cache.init(model, mgr);
  ContactState cs;
  cs.init(0);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  tl.init(*model, info, cache, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.update(q, v, cs);

  const int n_vars = info.nv;
  const int n_ineq = tl.ineq_dim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();

  tl.compute_inequality(cache, cs, info, n_vars, C, l, u);

  // C = S·M (should be non-zero)
  EXPECT_GT(C.norm(), 0.0);

  // l < u (torque limits: tau_min < tau_max assumed)
  for (int i = 0; i < n_ineq; ++i) {
    EXPECT_LT(l(i), u(i)) << "Joint " << i;
  }
}

// A-1: YAML tau_scale 적용 — bound 가 정확히 tau_scale 배가 되는지.
TEST(TorqueLimitTest, TauScaleAppliesMargin) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(kPandaUrdf, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.build(*model, config);

  ContactManagerConfig mgr;
  mgr.max_contacts = 0;
  PinocchioCache cache;
  cache.init(model, mgr);
  ContactState cs;
  cs.init(0);

  // 첫째: tau_scale=1.0 (default) — baseline bound 기록
  TorqueLimitConstraint tl1;
  YAML::Node cfg1;
  tl1.init(*model, info, cache, cfg1);

  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  cache.update(q, v, cs);

  const int n_vars = info.nv;
  const int n_ineq = tl1.ineq_dim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l1(n_ineq), u1(n_ineq);
  C.setZero();
  l1.setZero();
  u1.setZero();
  tl1.compute_inequality(cache, cs, info, n_vars, C, l1, u1);

  // 둘째: tau_scale=0.5 — bound 가 정확히 절반 magnitude 가 되는지.
  // l = 0.5·τ_min - S·h,  u = 0.5·τ_max - S·h.
  // baseline (scale=1): l = τ_min - S·h, u = τ_max - S·h.
  // → diff_l = l(0.5) - l(1) = -0.5·τ_min, diff_u = u(0.5) - u(1) = -0.5·τ_max.
  TorqueLimitConstraint tl2;
  YAML::Node cfg2;
  cfg2["tau_scale"] = 0.5;
  tl2.init(*model, info, cache, cfg2);

  Eigen::VectorXd l2(n_ineq), u2(n_ineq);
  C.setZero();
  l2.setZero();
  u2.setZero();
  tl2.compute_inequality(cache, cs, info, n_vars, C, l2, u2);

  // tau_min < 0, tau_max > 0 가정 (URDF effort_limit 의 부호).
  // diff = (scale - 1) · tau_lim. scale=0.5 → diff_l = -0.5·tau_min (positive),
  // diff_u = -0.5·tau_max (negative).
  for (int i = 0; i < n_ineq; ++i) {
    EXPECT_NEAR(l2(i) - l1(i), -0.5 * info.tau_min(i), 1e-9) << "Joint " << i;
    EXPECT_NEAR(u2(i) - u1(i), -0.5 * info.tau_max(i), 1e-9) << "Joint " << i;
  }
}

// A-1: tau_scale 범위 위반 시 init throw.
TEST(TorqueLimitTest, TauScaleOutOfRangeThrows) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(kPandaUrdf, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.build(*model, config);
  ContactManagerConfig mgr;
  PinocchioCache cache;
  cache.init(model, mgr);

  // > 1: reject
  {
    YAML::Node cfg;
    cfg["tau_scale"] = 1.5;
    TorqueLimitConstraint tl;
    EXPECT_THROW(tl.init(*model, info, cache, cfg), std::runtime_error);
  }
  // 0: reject (boundary)
  {
    YAML::Node cfg;
    cfg["tau_scale"] = 0.0;
    TorqueLimitConstraint tl;
    EXPECT_THROW(tl.init(*model, info, cache, cfg), std::runtime_error);
  }
  // < 0: reject
  {
    YAML::Node cfg;
    cfg["tau_scale"] = -0.5;
    TorqueLimitConstraint tl;
    EXPECT_THROW(tl.init(*model, info, cache, cfg), std::runtime_error);
  }
  // = 1.0: accept (boundary)
  {
    YAML::Node cfg;
    cfg["tau_scale"] = 1.0;
    TorqueLimitConstraint tl;
    EXPECT_NO_THROW(tl.init(*model, info, cache, cfg));
  }
}

}  // namespace
}  // namespace rtc::tsid
