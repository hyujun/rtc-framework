// The floating-base WQP rig the gates and the bounds suites share: the G1
// free-flyer on both feet (surface contacts, hard contact constraint) with
// posture, EoM, friction, torque and joint limits; N = 200 seeded cold-start
// samples (base position uniform in a 2 m box, rotation uniform on SO(3),
// joints uniform inside their limits less margin + 0.05, velocities
// non-zero); the convergence counter with its 0.9 floor; the solution digest.
// Pre-registered numbers live in the suites, not here. Not installed.
#pragma once

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
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
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::tsid::test {

constexpr int kSamples = 200;
constexpr double kEps = std::numeric_limits<double>::epsilon();
constexpr double kMargin = 0.05;  // joint_limit position_margin

struct Options {
  double eps_abs{1e-6};
  bool eom{true};
  double tau_scale{1.0};
  int max_iter{200};
  // Extra keys of the three constraints the bounds suite exercises (merged
  // over the defaults the gates use): `contact_accel` (baumgarte ·
  // terrain_height), `joint_limit` (acceleration_max), `torque_limit`
  // (rate_max · dt). Empty = the gates' configuration.
  YAML::Node contact_accel;
  YAML::Node joint_limit_yaml;
  YAML::Node torque_limit_yaml;
  bool second_contact_active{true};  // contacts[1].active at construction
  // Which of the inequality constraints the rig carries, and the formulation.
  bool friction{true};
  bool torque_limit{true};
  bool joint_limit{true};
  const char* formulation{"wqp"};
};

inline YAML::Node ControllerYaml(const Options& o) {
  YAML::Node cfg;
  cfg["formulation_type"] = o.formulation;
  // Iteration caps raised so the tolerance, not the cap, ends the solve.
  YAML::Node solver =
      std::string(o.formulation) == "hqp" ? cfg["hqp"]["solver_per_level"] : cfg["wqp"]["solver"];
  solver["eps_abs"] = o.eps_abs;
  solver["max_iter"] = o.max_iter;
  solver["max_iter_in"] = 500;
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
inline Eigen::Quaterniond UniformRotation(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const double u1 = u(rng), u2 = u(rng), u3 = u(rng);
  const double a = std::sqrt(1.0 - u1), b = std::sqrt(u1);
  return {a * std::sin(2 * M_PI * u2), a * std::cos(2 * M_PI * u2), b * std::sin(2 * M_PI * u3),
          b * std::cos(2 * M_PI * u3)};
}

inline std::vector<Sample> Samples(const pinocchio::Model& model, const RobotModelInfo& info) {
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
    cc->Init(*model, info, cache, o.contact_accel);
    cc->SetContactManager(&mgr);
    contact_accel = cc.get();
    f.AddConstraint(std::move(cc));
    if (o.friction) {
      auto fc = std::make_unique<FrictionConeConstraint>();
      fc->Init(*model, info, cache, YAML::Node{});
      fc->SetContactManager(&mgr);
      f.AddConstraint(std::move(fc));
    }
    if (o.torque_limit) {
      auto tl = std::make_unique<TorqueLimitConstraint>();
      YAML::Node tcfg = YAML::Clone(o.torque_limit_yaml);
      tcfg["tau_scale"] = o.tau_scale;
      tl->Init(*model, info, cache, tcfg);
      tl->SetContactManager(&mgr);
      torque_limit = tl.get();
      f.AddConstraint(std::move(tl));
    }
    if (o.joint_limit) {
      auto jl = std::make_unique<JointLimitConstraint>();
      YAML::Node jcfg = YAML::Clone(o.joint_limit_yaml);
      jcfg["position_margin"] = kMargin;
      jl->Init(*model, info, cache, jcfg);
      joint_limit = jl.get();
      f.AddConstraint(std::move(jl));
    }

    cache.Init(model, ContactFrameIds(mgr));
    contacts.Init(mgr.max_contacts);
    contacts.SeedNormals(mgr);
    for (auto& c : contacts.contacts) {
      c.active = true;
    }
    contacts.contacts[1].active = o.second_contact_active;
    contacts.RecomputeActive(mgr);
    state.q = x.q;
    state.v = x.v;
    cache.Update(x.q, x.v);
    ref.Init(info.nq, info.nv, info.n_actuated, mgr.max_contact_vars);
    ref.q_des = x.q_des;
  }

  const CommandOutput& Solve() { return ctrl.Compute(state, ref, cache, contacts); }

  // The three constraints the bounds suite drives (owned by the formulation).
  ContactConstraint* contact_accel{nullptr};
  JointLimitConstraint* joint_limit{nullptr};
  TorqueLimitConstraint* torque_limit{nullptr};

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

inline double P99(std::vector<double> v) {
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

// FNV-1a over the bytes of every converged sample's (a_opt, lambda_opt, tau),
// in sample order: one number that changes when any solution bit changes.
// Recorded (not asserted) so a branch can be compared with main in a PR body —
// a term that is off by default must leave it unchanged.
inline std::uint64_t Fnv1a(std::uint64_t h, const Eigen::VectorXd& v) {
  const auto* bytes = reinterpret_cast<const unsigned char*>(v.data());
  for (std::size_t i = 0; i < static_cast<std::size_t>(v.size()) * sizeof(double); ++i) {
    h ^= bytes[i];
    h *= 1099511628211ULL;
  }
  return h;
}

// Asks for one inequality row more than it reports (the row-budget trip).
class OneRowTooMany final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "one_row_too_many"; }

  void Init(const pinocchio::Model&, const RobotModelInfo&, PinocchioCache&,
            const YAML::Node&) override {}

  [[nodiscard]] int EqDim(const ContactState&) const noexcept override { return 0; }

  [[nodiscard]] int IneqDim(const ContactState&) const noexcept override { return 1; }

  [[nodiscard]] int MaxEqDim(const ContactManagerConfig&) const noexcept override { return 0; }

  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig&) const noexcept override { return 0; }

  void ComputeEquality(const PinocchioCache&, const ContactState&, const RobotModelInfo&, int,
                       Eigen::Ref<Eigen::MatrixXd>, Eigen::Ref<Eigen::VectorXd>) noexcept override {
  }

  void ComputeInequality(const PinocchioCache&, const ContactState&, const RobotModelInfo&, int,
                         Eigen::Ref<Eigen::MatrixXd>, Eigen::Ref<Eigen::VectorXd>,
                         Eigen::Ref<Eigen::VectorXd>) noexcept override {}
};

}  // namespace rtc::tsid::test
