// The floating-base WQP against independent oracles — pre-registered gates
// (#1012 [SPRINT] PR B; tolerances fixed before the first run, not after).
//
// The G1 free-flyer stands on both feet (surface contacts, hard contact
// constraint) with posture, EoM, friction, torque and joint limits. N = 200
// seeded samples: base position uniform in a 2 m box, base rotation uniform on
// SO(3), joints uniform inside their limits less margin + 0.05, velocities
// non-zero. Every sample is a cold start (a fresh controller). Only converged
// samples are graded; the others are counted and reported, never passed.
//
//   1. EoM base rows: rnea(q, v, a).head(6) − (Jcᵀλ).head(6) (RNEA is
//      independent of the CRBA mass matrix the QP uses) within
//      2·eps_abs + 64·nv·ε·S, S = ‖M‖∞‖a‖∞ + ‖h‖∞ + ‖Jcᵀλ‖∞.
//      Controls: a 1e-4 change of a fails it; without the EoM constraint the
//      residual is far above it; at eps_abs 1e-9 the same formula still holds.
//   2. Contact acceleration: ‖J_c a + J̇_c v‖∞ ≤ 2·eps_abs per foot.
//   3. τ against RNEA with λ applied as external forces: actuated rows within
//      max(1e-10, 1e-12·S); λ enters as world-axis forces at the frame
//      origin, so this also pins λ's convention.
//   4. With torque limits scaled down until they bind: violation ≤ eps_abs,
//      and some row binds.
//   5. Free-flyer velocity convention: the joint-1 LOCAL_WORLD_ALIGNED twist
//      is blockdiag(R, R)·v.head(6) within 1e-12·max(1, ‖v‖) (v is the LOCAL
//      twist); control ‖R·v − v‖ ≥ 0.1‖v‖ on most samples.
//   6. The contact frames' oMf after the cache update equal forward
//      kinematics (pinocchio 4.1 fills them as a side effect of the frame
//      Jacobian).

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

#include "g1_fixture.hpp"
#include "rtc_tsid/constraints/contact_constraint.hpp"
#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

constexpr int kSamples = 200;
constexpr double kEps = std::numeric_limits<double>::epsilon();
constexpr double kMargin = 0.05;  // joint_limit position_margin

struct Options {
  double eps_abs{1e-6};
  bool eom{true};
  double tau_scale{1.0};
  int max_iter{200};
};

YAML::Node ControllerYaml(const Options& o) {
  YAML::Node cfg;
  cfg["formulation_type"] = "wqp";
  // Iteration caps raised so the tolerance, not the cap, ends the solve.
  cfg["wqp"]["solver"]["eps_abs"] = o.eps_abs;
  cfg["wqp"]["solver"]["max_iter"] = o.max_iter;
  cfg["wqp"]["solver"]["max_iter_in"] = 500;
  for (const char* foot : {test::kG1LeftFoot, test::kG1RightFoot}) {
    YAML::Node c;
    c["name"] = foot;
    c["frame"] = foot;
    c["type"] = "surface";
    c["mu"] = 0.7;
    c["n_faces"] = 4;
    c["patch_half_length_x"] = 0.12;
    c["patch_half_length_y"] = 0.05;
    c["torsional_friction_coeff"] = 0.5;
    cfg["contacts"].push_back(c);
  }
  return cfg;
}

struct Sample {
  Eigen::VectorXd q, v, q_des;
};

// Shoemake's uniform quaternion from three uniforms.
Eigen::Quaterniond UniformRotation(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const double u1 = u(rng), u2 = u(rng), u3 = u(rng);
  const double a = std::sqrt(1.0 - u1), b = std::sqrt(u1);
  return {a * std::sin(2 * M_PI * u2), a * std::cos(2 * M_PI * u2), b * std::sin(2 * M_PI * u3),
          b * std::cos(2 * M_PI * u3)};
}

std::vector<Sample> Samples(const pinocchio::Model& model, const RobotModelInfo& info) {
  std::mt19937 rng(20261111);
  std::uniform_real_distribution<double> pm(-1.0, 1.0);
  std::vector<Sample> out;
  const int na = info.n_actuated;
  for (int s = 0; s < kSamples; ++s) {
    Sample x;
    x.q = pinocchio::neutral(model);
    x.q.head(3) << pm(rng), pm(rng), pm(rng);
    x.q.segment(3, 4) = UniformRotation(rng).coeffs();
    for (int i = 0; i < na; ++i) {
      const double lo = info.q_lower(7 + i) + kMargin + 0.05;
      const double hi = info.q_upper(7 + i) - kMargin - 0.05;
      const double t = 0.5 * (pm(rng) + 1.0);
      x.q(7 + i) = lo < hi ? lo + t * (hi - lo) : 0.5 * (info.q_lower(7 + i) + info.q_upper(7 + i));
    }
    pinocchio::normalize(model, x.q);
    x.v = Eigen::VectorXd::Zero(model.nv);
    for (int i = 0; i < model.nv; ++i) {
      x.v(i) = 0.5 * pm(rng);
    }
    x.q_des = x.q;
    for (int i = 0; i < na; ++i) {
      x.q_des(7 + i) += 0.1 * pm(rng);
    }
    out.push_back(x);
  }
  return out;
}

// A fresh G1 controller (cold start) for one sample.
struct Rig {
  Rig(const Options& o, const Sample& x) {
    model = test::SharedG1Model();
    info.Build(*model, YAML::Node{});
    const YAML::Node cfg = ControllerYaml(o);
    mgr.Load(cfg, *model);
    ctrl.Init(*model, info, cfg);
    auto& f = ctrl.Formulation();
    auto posture = std::make_unique<PostureTask>();
    YAML::Node pcfg;
    pcfg["kp"] = 100.0;
    pcfg["kd"] = 20.0;
    posture->Init(*model, info, cache, pcfg);
    f.AddTask(std::move(posture));
    if (o.eom) {
      auto eom = std::make_unique<EomConstraint>();
      eom->Init(*model, info, cache, YAML::Node{});
      eom->SetContactManager(&mgr);
      f.AddConstraint(std::move(eom));
    }
    auto cc = std::make_unique<ContactConstraint>();
    cc->Init(*model, info, cache, YAML::Node{});
    cc->SetContactManager(&mgr);
    f.AddConstraint(std::move(cc));
    auto fc = std::make_unique<FrictionConeConstraint>();
    fc->Init(*model, info, cache, YAML::Node{});
    fc->SetContactManager(&mgr);
    f.AddConstraint(std::move(fc));
    auto tl = std::make_unique<TorqueLimitConstraint>();
    YAML::Node tcfg;
    tcfg["tau_scale"] = o.tau_scale;
    tl->Init(*model, info, cache, tcfg);
    tl->SetContactManager(&mgr);
    f.AddConstraint(std::move(tl));
    auto jl = std::make_unique<JointLimitConstraint>();
    YAML::Node jcfg;
    jcfg["position_margin"] = kMargin;
    jl->Init(*model, info, cache, jcfg);
    f.AddConstraint(std::move(jl));

    cache.Init(model, ContactFrameIds(mgr));
    contacts.Init(mgr.max_contacts);
    contacts.SeedNormals(mgr);
    for (auto& c : contacts.contacts) {
      c.active = true;
    }
    contacts.RecomputeActive(mgr);
    state.q = x.q;
    state.v = x.v;
    cache.Update(x.q, x.v);
    ref.Init(info.nq, info.nv, info.n_actuated, mgr.max_contact_vars);
    ref.q_des = x.q_des;
  }

  const CommandOutput& Solve() { return ctrl.Compute(state, ref, cache, contacts); }

  // Σ_i J_i[0:6]ᵀ λ_i (both feet are surface contacts).
  [[nodiscard]] Eigen::VectorXd JcTLambda(const Eigen::VectorXd& lambda) const {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(info.nv);
    for (int i = 0; i < 2; ++i) {
      out.noalias() += cache.contact_frames[static_cast<size_t>(i)].J.topRows(6).transpose() *
                       lambda.segment(6 * i, 6);
    }
    return out;
  }

  [[nodiscard]] double Scale(const Eigen::VectorXd& a, const Eigen::VectorXd& jtl) const {
    const double m_inf = cache.M.cwiseAbs().rowwise().sum().maxCoeff();
    return m_inf * a.lpNorm<Eigen::Infinity>() + cache.h.lpNorm<Eigen::Infinity>() +
           jtl.lpNorm<Eigen::Infinity>();
  }

  // rnea(q, v, a).head(6) − (Jcᵀλ).head(6)
  [[nodiscard]] double BaseResidual(const Eigen::VectorXd& a, const Eigen::VectorXd& jtl) const {
    pinocchio::Data d(*model);
    const Eigen::VectorXd tau = pinocchio::rnea(*model, d, state.q, state.v, a);
    return (tau.head(6) - jtl.head(6)).lpNorm<Eigen::Infinity>();
  }

  // rnea(q, v, a, f_ext(λ)): λ_i = [force; moment] in world axes at the
  // contact frame origin, applied ON the robot.
  [[nodiscard]] Eigen::VectorXd RneaWithContactForces(const Eigen::VectorXd& a,
                                                      const Eigen::VectorXd& lambda) const {
    pinocchio::Data d(*model);
    pinocchio::forwardKinematics(*model, d, state.q);
    pinocchio::updateFramePlacements(*model, d);
    std::vector<pinocchio::Force> fext(static_cast<size_t>(model->njoints),
                                       pinocchio::Force::Zero());
    for (int i = 0; i < 2; ++i) {
      const auto fid =
          static_cast<pinocchio::FrameIndex>(mgr.contacts[static_cast<size_t>(i)].frame_id);
      const auto parent = model->frames[fid].parentJoint;
      const Eigen::Vector3d p = d.oMf[fid].translation();
      const Eigen::Vector3d f = lambda.segment<3>(6 * i);
      const Eigen::Vector3d m = lambda.segment<3>(6 * i + 3);
      const pinocchio::Force world(f, m + p.cross(f));  // about the world origin
      fext[parent] += d.oMi[parent].actInv(world);
    }
    pinocchio::Data d2(*model);
    return pinocchio::rnea(*model, d2, state.q, state.v, a, fext);
  }

  std::shared_ptr<const pinocchio::Model> model;
  RobotModelInfo info;
  ContactManagerConfig mgr;
  TSIDController ctrl;
  PinocchioCache cache;
  ContactState contacts;
  ControlState state;
  ControlReference ref;
};

double P99(std::vector<double> v) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const auto k = static_cast<size_t>(std::ceil(0.99 * static_cast<double>(v.size()))) - 1;
  return v[std::min(k, v.size() - 1)];
}

struct Counters {
  int converged{0};
  int not_converged{0};
  int max_iter{0};

  void Record(const CommandOutput& out) {
    if (out.qp_converged) {
      ++converged;
    } else {
      ++not_converged;
      max_iter += out.qp_status == 1 ? 1 : 0;
    }
  }

  void Report(const std::string& tag) const {
    ::testing::Test::RecordProperty(tag + "_converged", std::to_string(converged));
    ::testing::Test::RecordProperty(tag + "_not_converged", std::to_string(not_converged));
    ::testing::Test::RecordProperty(tag + "_max_iter", std::to_string(max_iter));
    // Grading only converged samples must not let a regression that stops
    // the QP converging pass green. Floor added after the first run (which
    // converged 199–200 of 200), well below it; not one of the pre-registered
    // tolerances.
    EXPECT_GE(converged, 0.9 * (converged + not_converged)) << tag << ": convergence rate";
  }
};

class FloatingWqpGates : public ::testing::Test {
 protected:
  void SetUp() override {
    RobotModelInfo info;
    info.Build(*test::SharedG1Model(), YAML::Node{});
    samples_ = Samples(*test::SharedG1Model(), info);
  }

  std::vector<Sample> samples_;
};

// Gates 1, 2, 3 and the 1e-4 control of gate 1.
TEST_F(FloatingWqpGates, EomContactAndTorqueMatchRnea) {
  const Options o;
  Counters n;
  std::vector<double> eom_ratio, contact_ratio, tau_ratio;
  int perturbation_caught = 0;
  for (const auto& x : samples_) {
    Rig rig(o, x);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    const Eigen::VectorXd a = out.a_opt;
    const Eigen::VectorXd lambda = out.lambda_opt;
    const Eigen::VectorXd jtl = rig.JcTLambda(lambda);
    const double S = rig.Scale(a, jtl);
    const int nv = rig.info.nv;
    const double tol1 = 2.0 * o.eps_abs + 64.0 * nv * kEps * S;

    // 1. EoM base rows.
    const double e1 = rig.BaseResidual(a, jtl);
    eom_ratio.push_back(e1 / tol1);
    EXPECT_LE(e1, tol1);
    Eigen::VectorXd a_bad = a;
    a_bad(0) += 1e-4;
    perturbation_caught += rig.BaseResidual(a_bad, jtl) > tol1 ? 1 : 0;

    // 2. Contact acceleration.
    for (int i = 0; i < 2; ++i) {
      const auto& fc = rig.cache.contact_frames[static_cast<size_t>(i)];
      const double e2 = (fc.J * a + fc.dJv).lpNorm<Eigen::Infinity>();
      contact_ratio.push_back(e2 / (2.0 * o.eps_abs));
      EXPECT_LE(e2, 2.0 * o.eps_abs) << "foot " << i;
    }

    // 3. τ against RNEA with the contact forces as external forces.
    const Eigen::VectorXd tau_ext = rig.RneaWithContactForces(a, lambda);
    const double tol3 = std::max(1e-10, 1e-12 * S);
    const double e3 = (tau_ext.tail(rig.info.n_actuated) - out.tau).lpNorm<Eigen::Infinity>();
    tau_ratio.push_back(e3 / tol3);
    EXPECT_LE(e3, tol3);
    EXPECT_LE(tau_ext.head(6).lpNorm<Eigen::Infinity>(), tol1) << "base rows: gate 1 again";
  }
  n.Report("main");
  RecordProperty("eom_ratio_max",
                 std::to_string(*std::max_element(eom_ratio.begin(), eom_ratio.end())));
  RecordProperty("eom_ratio_p99", std::to_string(P99(eom_ratio)));
  RecordProperty("contact_ratio_p99", std::to_string(P99(contact_ratio)));
  RecordProperty("tau_ratio_max",
                 std::to_string(*std::max_element(tau_ratio.begin(), tau_ratio.end())));
  RecordProperty("tau_ratio_p99", std::to_string(P99(tau_ratio)));
  ASSERT_GT(n.converged, 0) << "no converged sample: every gate above is vacuous";
  EXPECT_EQ(perturbation_caught, n.converged) << "control: a 1e-4 change of a must fail gate 1";
}

// Gate 1 control: without the EoM constraint the base residual is far above.
TEST_F(FloatingWqpGates, WithoutTheEomConstraintTheBaseResidualIsFarAboveTheGate) {
  Options o;
  o.eom = false;
  Counters n;
  double min_ratio = std::numeric_limits<double>::infinity();
  for (int s = 0; s < 20; ++s) {
    Rig rig(o, samples_[static_cast<size_t>(s)]);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    const Eigen::VectorXd jtl = rig.JcTLambda(out.lambda_opt);
    const double tol1 = 2.0 * o.eps_abs + 64.0 * rig.info.nv * kEps * rig.Scale(out.a_opt, jtl);
    min_ratio = std::min(min_ratio, rig.BaseResidual(out.a_opt, jtl) / tol1);
  }
  n.Report("no_eom");
  RecordProperty("no_eom_min_ratio", std::to_string(min_ratio));
  ASSERT_GT(n.converged, 0);
  EXPECT_GT(min_ratio, 100.0);
}

// Gate 1 control: the residual follows eps_abs down to 1e-9.
TEST_F(FloatingWqpGates, AtATighterToleranceTheSameFormulaHolds) {
  Options o;
  o.eps_abs = 1e-9;
  o.max_iter = 2000;  // the tighter tolerance needs more outer steps
  Counters n;
  double max_ratio = 0.0;
  for (int s = 0; s < 50; ++s) {
    Rig rig(o, samples_[static_cast<size_t>(s)]);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    const Eigen::VectorXd jtl = rig.JcTLambda(out.lambda_opt);
    const double tol1 = 2.0 * o.eps_abs + 64.0 * rig.info.nv * kEps * rig.Scale(out.a_opt, jtl);
    const double e = rig.BaseResidual(out.a_opt, jtl);
    max_ratio = std::max(max_ratio, e / tol1);
    EXPECT_LE(e, tol1);
  }
  n.Report("eps_1e-9");
  RecordProperty("eps_1e-9_max_ratio", std::to_string(max_ratio));
  ASSERT_GT(n.converged, 0);
}

// Gate 4: torque limits that bind.
TEST_F(FloatingWqpGates, ScaledTorqueLimitsBindAndHold) {
  Options o;
  o.tau_scale = 0.1;
  Counters n;
  int binding = 0;
  double max_violation = 0.0;
  for (const auto& x : samples_) {
    Rig rig(o, x);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    const Eigen::VectorXd hi = o.tau_scale * rig.info.tau_max;
    const Eigen::VectorXd lo = o.tau_scale * rig.info.tau_min;
    for (int i = 0; i < rig.info.n_actuated; ++i) {
      const double viol = std::max(out.tau(i) - hi(i), lo(i) - out.tau(i));
      max_violation = std::max(max_violation, viol);
      binding += viol > -1e-6 ? 1 : 0;
    }
  }
  n.Report("tau_scaled");
  RecordProperty("tau_scaled_binding_rows", std::to_string(binding));
  RecordProperty("tau_scaled_max_violation", std::to_string(max_violation));
  ASSERT_GT(n.converged, 0);
  EXPECT_LE(max_violation, o.eps_abs);
  EXPECT_GT(binding, 0) << "control: the scaled limits must bind somewhere";
}

// Gates 5 and 6: the free-flyer velocity convention and the contact oMf.
TEST_F(FloatingWqpGates, BaseTwistIsLocalAndContactPlacementsAreFresh) {
  const auto model = test::SharedG1Model();
  ContactManagerConfig mgr = test::G1FeetContacts(*model);
  PinocchioCache cache;
  cache.Init(model, ContactFrameIds(mgr));
  int discriminating = 0;
  for (const auto& x : samples_) {
    // 5. joint-1 LWA twist = blockdiag(R, R)·v.head(6).
    pinocchio::Data d(*model);
    pinocchio::computeJointJacobians(*model, d, x.q);
    pinocchio::Data::Matrix6x J1 = pinocchio::Data::Matrix6x::Zero(6, model->nv);
    pinocchio::getJointJacobian(*model, d, 1, pinocchio::LOCAL_WORLD_ALIGNED, J1);
    const Eigen::Matrix3d R = Eigen::Quaterniond(x.q.segment<4>(3)).toRotationMatrix();
    const Eigen::Matrix<double, 6, 1> twist = J1 * x.v;
    const double tol = 1e-12 * std::max(1.0, x.v.norm());
    EXPECT_LE((twist.head<3>() - R * x.v.head<3>()).lpNorm<Eigen::Infinity>(), tol);
    EXPECT_LE((twist.tail<3>() - R * x.v.segment<3>(3)).lpNorm<Eigen::Infinity>(), tol);
    discriminating += (R * x.v.head<3>() - x.v.head<3>()).norm() >= 0.1 * x.v.head<3>().norm();

    // 6. oMf after the cache update = forward kinematics.
    cache.Update(x.q, x.v);
    pinocchio::Data fk(*model);
    pinocchio::forwardKinematics(*model, fk, x.q);
    for (int i = 0; i < 2; ++i) {
      const auto fid =
          static_cast<pinocchio::FrameIndex>(mgr.contacts[static_cast<size_t>(i)].frame_id);
      const pinocchio::SE3 oMf =
          fk.oMi[model->frames[fid].parentJoint] * model->frames[fid].placement;
      EXPECT_TRUE(cache.contact_frames[static_cast<size_t>(i)].oMf.isApprox(oMf, 1e-12)) << i;
    }
  }
  RecordProperty("twist_discriminating_samples", std::to_string(discriminating));
  EXPECT_GE(discriminating, static_cast<int>(0.9 * kSamples))
      << "control: the rotations must move the twist for the check to discriminate";
}

// Surface contacts on feet in every orientation: the QP's λ, read in each
// foot's own axes (t1 = foot x projected ⊥ the world-up contact normal, or its
// y when x is within ~6° of it), keeps f_n ≥ 0 and the CoP / yaw moment
// inside the foot's 0.12 × 0.05 rectangle, within 10·eps_abs.
TEST_F(FloatingWqpGates, EachFootHoldsItsCopRectangleInItsOwnAxes) {
  const Options o;
  Counters n;
  double worst = -std::numeric_limits<double>::infinity();
  int loaded = 0;
  for (const auto& x : samples_) {
    Rig rig(o, x);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    for (int i = 0; i < 2; ++i) {
      const Eigen::Vector3d nz = Eigen::Vector3d::UnitZ();
      const Eigen::Matrix3d oRf = rig.cache.contact_frames[static_cast<size_t>(i)].oMf.rotation();
      Eigen::Matrix3d R_c;
      Eigen::Vector3d t1 = oRf.col(0) - nz * oRf.col(0).dot(nz);
      if (t1.norm() >= 0.1) {
        t1.normalize();
      } else {
        t1 = (oRf.col(1) - nz * oRf.col(1).dot(nz)).normalized().cross(nz);
      }
      R_c << t1, nz.cross(t1), nz;
      const Eigen::Vector3d f = R_c.transpose() * out.lambda_opt.segment<3>(6 * i);
      const Eigen::Vector3d m = R_c.transpose() * out.lambda_opt.segment<3>(6 * i + 3);
      const double fn = f.z();
      loaded += fn > 1.0 ? 1 : 0;
      for (const double excess : {-fn, std::abs(m.y()) - 0.12 * fn, std::abs(m.x()) - 0.05 * fn,
                                  std::abs(m.z()) - 0.5 * fn}) {
        worst = std::max(worst, excess);
        EXPECT_LE(excess, 10.0 * o.eps_abs) << "foot " << i;
      }
    }
  }
  n.Report("cop");
  RecordProperty("cop_worst_excess", std::to_string(worst));
  RecordProperty("cop_loaded_feet", std::to_string(loaded));
  ASSERT_GT(n.converged, 0);
  EXPECT_GT(loaded, 0) << "control: some foot must carry load for the check to bind";
}

}  // namespace
}  // namespace rtc::tsid
