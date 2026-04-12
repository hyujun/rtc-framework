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

const std::string kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

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

}  // namespace
}  // namespace rtc::tsid
