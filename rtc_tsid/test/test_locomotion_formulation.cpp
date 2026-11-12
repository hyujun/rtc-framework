// The W6-F01 [SPRINT] v3-B PR B gates (#1039): the humanoid whole-body QP
// assembled by LocomotionFormulationBuilder on the public G1 (sole frames),
// against independent oracles — RNEA with the contact forces as external
// forces, pinocchio's centre-of-mass acceleration, a minimum-norm balance
// force, direct pinocchio frame kinematics and central differences. The K /
// A_sim / R_sim numbers were registered from the probe before these tests.
#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include "loco_wbc_fixture.hpp"
#include "rtc_tsid/kinematics/se3_error.hpp"
#include "rtc_tsid/tasks/com_task.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace rtc::tsid {
namespace {

using namespace rtc::tsid::test;  // NOLINT — the shared rig

constexpr double kEpsAbs = 1e-6;
constexpr double kEps = std::numeric_limits<double>::epsilon();
// Registered (v3-B, probe 2026-11-12).
constexpr double kKa = 15.0;
constexpr double kKlambda = 5400.0;
constexpr double kAsim = 2.6e-2;
constexpr double kTol1 = 1e-9;
constexpr double kRsim = 4.4e-4;

std::vector<Eigen::VectorXd> StanceSamples(const pinocchio::Model& m) {
  std::vector<Eigen::VectorXd> out = {QStand(m)};
  std::mt19937 rng(20261112);
  std::uniform_real_distribution<double> pm(-0.02, 0.02);
  for (int s = 0; s < 20; ++s) {
    Eigen::VectorXd q = QStand(m);
    for (int i = 7; i < m.nq; ++i) {
      q(i) += pm(rng);
    }
    out.push_back(q);
  }
  return out;
}

// Minimum-norm λ of the base EOM rows at a = 0: Jcᵀ(base rows) λ = h_base.
Eigen::VectorXd LambdaBal(const LocoRig& r) {
  Eigen::MatrixXd JT(6, 12);
  for (int c = 0; c < 2; ++c) {
    JT.block(0, 6 * c, 6, 6) =
        r.cache.contact_frames[static_cast<size_t>(c)].J.topRows(6).leftCols(6).transpose();
  }
  return JT.completeOrthogonalDecomposition().solve(r.cache.h.head(6));
}

using Vec6 = Eigen::Matrix<double, 6, 1>;

// A frame's placement, LWA twist [lin; ang] and LWA classical acceleration
// at (q, v, a), from pinocchio directly — the oracle the tests hold the
// builder and ReferenceKinematics to.
struct FrameMotion {
  pinocchio::SE3 M;
  Vec6 v;
  Vec6 a;
};

FrameMotion FrameOracle(const pinocchio::Model& model, const Eigen::VectorXd& q,
                        const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                        pinocchio::FrameIndex f) {
  pinocchio::Data d(model);
  pinocchio::forwardKinematics(model, d, q, v, a);
  FrameMotion out;
  out.M = pinocchio::updateFramePlacement(model, d, f);
  const pinocchio::Motion tw =
      pinocchio::getFrameVelocity(model, d, f, pinocchio::LOCAL_WORLD_ALIGNED);
  const pinocchio::Motion ac =
      pinocchio::getFrameClassicalAcceleration(model, d, f, pinocchio::LOCAL_WORLD_ALIGNED);
  out.v << tw.linear(), tw.angular();
  out.a << ac.linear(), ac.angular();
  return out;
}

// The twist as a central difference of the placement along q ⊕ ±h v.
Vec6 PlacementFd(const pinocchio::Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                 pinocchio::FrameIndex f) {
  const double h = 1e-6;
  auto place = [&](double t) {
    pinocchio::Data d(model);
    pinocchio::framesForwardKinematics(model, d, pinocchio::integrate(model, q, t * v));
    return d.oMf[f];
  };
  const pinocchio::SE3 Mp = place(h), Mm = place(-h);
  Vec6 fd;
  fd.head<3>() = (Mp.translation() - Mm.translation()) / (2 * h);
  fd.tail<3>() = pinocchio::log3(Mp.rotation() * Mm.rotation().transpose()) / (2 * h);
  return fd;
}

// A task's row (J, r) on a rig's state.
struct TaskRow {
  Eigen::MatrixXd J;
  Eigen::VectorXd r;
};

TaskRow Row(TaskBase& task, const LocoRig& rig) {
  const int n = rig.info.nv + rig.b.Contacts().max_contact_vars;
  TaskRow row{Eigen::MatrixXd::Zero(task.ResidualDim(), n),
              Eigen::VectorXd::Zero(task.ResidualDim())};
  task.ComputeResidual(rig.cache, rig.ref, rig.contacts, n, row.J, row.r);
  return row;
}

double RelDiff(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
  return (x - y).lpNorm<Eigen::Infinity>() / std::max(1.0, y.lpNorm<Eigen::Infinity>());
}

// rnea(q, v, a, f_ext(λ)) — λ_i on world axes at the contact frame origin.
Eigen::VectorXd RneaWithContactForces(const LocoRig& r, const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                                      const Eigen::VectorXd& lambda) {
  const auto& model = *r.model;
  pinocchio::Data d(model);
  pinocchio::forwardKinematics(model, d, q);
  pinocchio::updateFramePlacements(model, d);
  std::vector<pinocchio::Force> fext(static_cast<size_t>(model.njoints), pinocchio::Force::Zero());
  for (int i = 0; i < 2; ++i) {
    const auto fid = static_cast<pinocchio::FrameIndex>(
        r.b.Contacts().contacts[static_cast<size_t>(i)].frame_id);
    const auto parent = model.frames[fid].parentJoint;
    const Eigen::Vector3d p = d.oMf[fid].translation();
    const Eigen::Vector3d f = lambda.segment<3>(6 * i);
    const Eigen::Vector3d mo = lambda.segment<3>(6 * i + 3);
    fext[parent] += d.oMi[parent].actInv(pinocchio::Force(f, mo + p.cross(f)));
  }
  pinocchio::Data d2(model);
  return pinocchio::rnea(model, d2, q, v, a, fext);
}

// ── Static stance ─────────────────────────────────────────────────────────────

TEST(LocomotionFormulation, StandingAtTheBalanceForceTheQpHoldsStill) {
  int converged = 0;
  for (const Eigen::VectorXd& q : StanceSamples(*SharedG1Model())) {
    LocoRig r;
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(r.model->nv);
    r.Set(q, v);
    const Eigen::VectorXd lb = LambdaBal(r);
    r.ref.lambda_des = lb;
    const CommandOutput& o = r.Solve();
    ASSERT_TRUE(o.qp_converged);
    ASSERT_TRUE(o.tau_valid);
    ++converged;
    EXPECT_LE(o.a_opt.lpNorm<Eigen::Infinity>(), kKa * kEpsAbs);
    EXPECT_LE((o.lambda_opt - lb).lpNorm<Eigen::Infinity>(), kKlambda * kEpsAbs);
    // Σ f_z = m (g + c̈_z(a)) — the identity of the base EOM's vertical row.
    pinocchio::Data d(*r.model);
    pinocchio::centerOfMass(*r.model, d, q, v, o.a_opt);
    const double m = pinocchio::computeTotalMass(*r.model);
    EXPECT_LE(std::fabs(o.lambda_opt(2) + o.lambda_opt(8) - m * (9.81 + d.acom[0](2))), kTol1);
    // τ against RNEA with λ as external forces (the gates' gate 3).
    const Eigen::VectorXd tau = RneaWithContactForces(r, q, v, o.a_opt, o.lambda_opt);
    const double S =
        r.cache.M.cwiseAbs().rowwise().sum().maxCoeff() * o.a_opt.lpNorm<Eigen::Infinity>() +
        r.cache.h.lpNorm<Eigen::Infinity>() + 300.0;
    EXPECT_LE((tau.tail(r.info.n_actuated) - o.tau).lpNorm<Eigen::Infinity>(),
              std::max(1e-10, 1e-12 * S));
    EXPECT_LE(tau.head(6).lpNorm<Eigen::Infinity>(), 2.0 * kEpsAbs + 64.0 * r.info.nv * kEps * S);
  }
  EXPECT_EQ(converged, 21);
}

TEST(LocomotionFormulation, StandingAtHalfTheWeightPerFootStaysNearlyStill) {
  for (const Eigen::VectorXd& q : StanceSamples(*SharedG1Model())) {
    LocoRig r;
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(r.model->nv);
    r.Set(q, v);
    r.WeightOn({0, 1});
    const CommandOutput& o = r.Solve();
    const double m = pinocchio::computeTotalMass(*r.model);
    ASSERT_TRUE(o.tau_valid);
    EXPECT_LE(o.a_opt.lpNorm<Eigen::Infinity>(), kAsim);
    pinocchio::Data d(*r.model);
    pinocchio::centerOfMass(*r.model, d, q, v, o.a_opt);
    EXPECT_LE(std::fabs(o.lambda_opt(2) + o.lambda_opt(8) - m * (9.81 + d.acom[0](2))), kTol1);
  }
}

// ── Swing masking ─────────────────────────────────────────────────────────────

TEST(LocomotionFormulation, ASwingFootLeavesTheContactRowsAndItsTaskTakesOver) {
  LocoRig r;
  const Eigen::VectorXd q = QStand(*r.model);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(r.model->nv);
  r.Set(q, v);
  const CommandOutput& both = r.Solve();
  ASSERT_TRUE(both.tau_valid);
  const int ineq_both = both.n_ineq_used;
  EXPECT_FALSE(r.b.FootTask(0)->IsActive());
  EXPECT_FALSE(r.b.FootTask(1)->IsActive());

  r.Release(1);
  r.WeightOn({0});
  const CommandOutput& o = r.Solve();
  ASSERT_TRUE(o.tau_valid);
  EXPECT_TRUE(r.b.FootTask(1)->IsActive());  // the same tick
  EXPECT_FALSE(r.b.FootTask(0)->IsActive());
  EXPECT_EQ(o.n_eq_used, 18);                // fixed-dim rows: masking zeroes, not removes
  EXPECT_EQ(o.n_ineq_used, ineq_both - 11);  // one surface contact's friction rows
  EXPECT_LE(o.lambda_opt.segment(6, 6).lpNorm<Eigen::Infinity>(), kEpsAbs);
  // The contact rows directly: the swing foot's six are zero, the stance foot's are Jc.
  const int n_vars = r.info.nv + 12;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, n_vars);
  Eigen::VectorXd bb = Eigen::VectorXd::Zero(12);
  r.b.ContactRows()->ComputeEquality(r.cache, r.contacts, r.info, n_vars, A, bb);
  EXPECT_EQ(A.middleRows(6, 6).lpNorm<Eigen::Infinity>(), 0.0);
  EXPECT_EQ(bb.segment(6, 6).lpNorm<Eigen::Infinity>(), 0.0);
  EXPECT_EQ((A.block(0, 0, 6, r.info.nv) - r.cache.contact_frames[0].J.topRows(6))
                .lpNorm<Eigen::Infinity>(),
            0.0);
  // τ does not depend on the swing slot's force reference: two cold rigs
  // that differ only there pose the same QP.
  LocoRig c0, c1;
  for (LocoRig* c : {&c0, &c1}) {
    c->Set(q, v);
    c->Release(1);
    c->ref.lambda_des = r.ref.lambda_des;
  }
  c1.ref.lambda_des.segment(6, 6) << 40, -20, 300, 5, -5, 1;
  const Eigen::VectorXd tau0 = c0.Solve().tau;
  const CommandOutput& o1 = c1.Solve();
  ASSERT_TRUE(o1.tau_valid);
  EXPECT_EQ((o1.tau - tau0).lpNorm<Eigen::Infinity>(), 0.0);
  // t_ramp from the YAML reaches the contact state.
  YAML::Node ramp = YAML::Load(kLocoWbq);
  ramp["contact"]["t_ramp"] = 0.05;
  LocoRig rr(ramp);
  EXPECT_EQ(rr.contacts.contacts[0].t_ramp_sec, 0.05);
  EXPECT_EQ(rr.b.RampTime(), 0.05);
}

// ── Swing tracking ────────────────────────────────────────────────────────────

Eigen::VectorXd LiftRightFoot(const pinocchio::Model& m, Eigen::VectorXd q) {
  q(m.joints[m.getJointId("right_hip_pitch_joint")].idx_q()) -= 0.1;
  q(m.joints[m.getJointId("right_knee_joint")].idx_q()) += 0.2;
  return q;
}

TEST(LocomotionFormulation, TheSwingFootTaskTracksTheReferenceKinematics) {
  LocoRig r;
  const Eigen::VectorXd q = QStand(*r.model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(r.model->nv);
  v(6 + 9) = 0.3;  // a moving right knee: twist and drift non-zero
  r.Set(q, v);
  r.Release(1);
  r.ref.q_des = LiftRightFoot(*r.model, q);
  r.ref.v_des = v;
  r.ref.v_des(6 + 9) = 0.5;
  r.ref.a_des.setZero();
  r.ref.a_des(6 + 9) = 2.0;
  ASSERT_TRUE(r.b.UpdateReferences(r.ref, r.contacts));
  // (i) the task row: r = a_ref + kp⊙e + kd⊙ė − J̇v with a_ref, the reference's
  // classical LWA acceleration, from pinocchio directly.
  const FrameMotion des = FrameOracle(*r.model, r.ref.q_des, r.ref.v_des, r.ref.a_des,
                                      r.model->getFrameId("right_sole"));
  const auto& fc = r.cache.contact_frames[1];
  const Vec6 nu = fc.J * r.cache.v;
  const Vec6 e = ComputeTaskPoseError(fc.oMf, des.M);
  const Vec6 edot = ComputeTaskVelocityError(fc.oMf, des.M, nu, des.v);
  const Vec6 expect = des.a + 400.0 * e + 40.0 * edot - fc.dJv;
  EXPECT_LE(RelDiff(Row(*r.b.FootTask(1), r).r, expect), 1e-12);
  EXPECT_GT(des.a.lpNorm<Eigen::Infinity>(), 1e-2);  // a_ff is not zero here
}

TEST(LocomotionFormulation, AHeavyFootTaskReachesItsTargetAndOffItDoesNot) {
  YAML::Node wide = YAML::Load(kLocoWbq);
  wide["contact"]["friction_coeff"] = 2.0;
  wide["contact"]["patch_half_length_x"] = 0.5;
  wide["contact"]["patch_half_length_y"] = 0.5;
  wide["tasks"]["feet"]["weight"] = 1e6;
  double res[2] = {0.0, 0.0};
  for (int on = 1; on >= 0; --on) {
    LocoRig s(wide);
    const Eigen::VectorXd q = QStand(*s.model);
    s.Set(q, Eigen::VectorXd::Zero(s.model->nv));
    s.Release(1);
    s.WeightOn({0});
    s.ref.q_des = LiftRightFoot(*s.model, q);
    if (on == 0) {
      s.b.FootTask(1)->SetWeight(0.0);
    }
    const CommandOutput& o = s.Solve();
    ASSERT_TRUE(o.tau_valid);
    const TaskRow row = Row(*s.b.FootTask(1), s);
    Eigen::VectorXd x(row.J.cols());
    x << o.a_opt, o.lambda_opt;
    res[on] = (row.J * x - row.r).lpNorm<Eigen::Infinity>();
  }
  EXPECT_LE(res[1], kRsim);
  EXPECT_GE(res[0], 100.0 * kRsim);
}

// ── Reference kinematics · torso ──────────────────────────────────────────────

TEST(ReferenceKinematics, MatchesPinocchioAndCentralDifferences) {
  LocoRig r;
  const auto& model = *r.model;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> pm(-1.0, 1.0);
  int power = 0, samples = 0;
  for (int s = 0; s < 40; ++s) {
    Eigen::VectorXd q = QStand(model);
    // yaw π/2 · pitch 0.2 base, joints moved
    const Eigen::Quaterniond base(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
    q.segment<4>(3) = base.coeffs();
    for (int i = 7; i < model.nq; ++i) {
      q(i) += 0.2 * pm(rng);
    }
    Eigen::VectorXd v(model.nv), a(model.nv);
    for (int i = 0; i < model.nv; ++i) {
      v(i) = 0.5 * pm(rng);
      a(i) = 2.0 * pm(rng);
    }
    v(5) = 0.5;  // ω_z
    ReferenceKinematics rk;
    const std::vector<pinocchio::FrameIndex> frames = {model.getFrameId("right_sole"),
                                                       model.getFrameId("torso_link")};
    rk.Init(model, r.info, frames, true);
    ASSERT_TRUE(rk.Update(q, v, a));
    for (std::size_t k = 0; k < frames.size(); ++k) {
      // Wiring: pinocchio directly.
      const FrameMotion o = FrameOracle(model, q, v, a, frames[k]);
      EXPECT_TRUE(rk.Placement(k).isApprox(o.M, 1e-12));
      EXPECT_LE((rk.Twist(k) - o.v).lpNorm<Eigen::Infinity>(), 1e-12);
      EXPECT_LE((rk.Twist(k) - PlacementFd(model, q, v, frames[k])).lpNorm<Eigen::Infinity>(), 1e-6)
          << "frame " << k;
      // Classical acceleration against the derivative of the LWA twist along
      // the trajectory (q ⊕ t v + ½t² a, v + t a), Richardson-bounded.
      const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);
      auto twist_at = [&](double t) {
        return FrameOracle(model, pinocchio::integrate(model, q, t * v + 0.5 * t * t * a),
                           v + t * a, zero, frames[k])
            .v;
      };
      const double hh = 1e-4;
      const Eigen::Matrix<double, 6, 1> fd_h = (twist_at(hh) - twist_at(-hh)) / (2 * hh);
      const Eigen::Matrix<double, 6, 1> fd_h2 = (twist_at(hh / 2) - twist_at(-hh / 2)) / hh;
      const double w = twist_at(hh).lpNorm<Eigen::Infinity>();
      const double bound = 2.0 * (fd_h - fd_h2).lpNorm<Eigen::Infinity>() + 16.0 * kEps * w / hh;
      EXPECT_LE((rk.Acceleration(k) - fd_h2).lpNorm<Eigen::Infinity>(), bound + 1e-9)
          << "frame " << k;
      // Power: the classical term ω × v the spatial convention would miss.
      const Eigen::Vector3d wxv = o.v.tail<3>().cross(o.v.head<3>());
      ++samples;
      power += wxv.norm() > 10.0 * (bound + 1e-9);
    }
  }
  EXPECT_GE(power, static_cast<int>(0.95 * samples));
}

TEST(ReferenceKinematics, RefusesBadInputAndNormalisesTheQuaternion) {
  LocoRig r;
  const auto& model = *r.model;
  ReferenceKinematics rk;
  rk.Init(model, r.info, {model.getFrameId("torso_link")}, false);
  Eigen::VectorXd q = QStand(model);
  // A rotated base: scaling the identity quaternion only scales w, which the
  // rotation-matrix formula drops — that would hide a missing normalisation.
  q.segment<4>(3) =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())).coeffs();
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  ASSERT_TRUE(rk.Update(q, v, v));
  const pinocchio::SE3 good = rk.Placement(0);
  EXPECT_FALSE(rk.Update(q.head(model.nq - 1), v, v));  // wrong size
  EXPECT_FALSE(rk.Update(q, v.head(model.nv - 1), v));
  Eigen::VectorXd bad = q;
  bad(9) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rk.Update(bad, v, v));
  Eigen::VectorXd zero_quat = q;
  zero_quat.segment<4>(3).setZero();
  EXPECT_FALSE(rk.Update(zero_quat, v, v));
  Eigen::VectorXd tiny_quat = q;  // finite but too small to trust its direction
  tiny_quat.segment<4>(3) *= 1e-12;
  EXPECT_FALSE(rk.Update(tiny_quat, v, v));
  EXPECT_TRUE(rk.Placement(0).isApprox(good, 0.0));  // the last result is kept
  Eigen::VectorXd scaled = q;
  scaled.segment<4>(3) *= 0.9;  // not unit: normalised in the kinematics' own copy
  ASSERT_TRUE(rk.Update(scaled, v, v));
  EXPECT_TRUE(rk.Placement(0).isApprox(good, 1e-12));
}

TEST(LocomotionFormulation, TheTorsoTaskGetsTheReferenceTwistOfAYawedPitchedBase) {
  LocoRig r;
  Eigen::VectorXd q = QStand(*r.model);
  const Eigen::Quaterniond base(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  q.segment<4>(3) = base.coeffs();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(r.model->nv);
  v(5) = 0.5;  // ω_z (LOCAL)
  v(0) = 0.3;
  r.Set(q, v);
  ASSERT_TRUE(r.b.UpdateReferences(r.ref, r.contacts));
  const auto& rk = r.b.Kinematics();
  const std::size_t t = r.b.NumFeet();
  const pinocchio::FrameIndex torso = r.model->getFrameId("torso_link");
  EXPECT_LE((rk.Twist(t) - PlacementFd(*r.model, q, v, torso)).lpNorm<Eigen::Infinity>(), 1e-6);
  EXPECT_GT(std::fabs(rk.Twist(t)(1)), 0.1);  // the LOCAL x velocity became world y (yaw π/2)
  // The task row: the same as a twin whose torso reference is set from
  // pinocchio directly.
  LocoRig o;
  o.Set(q, v);
  ASSERT_TRUE(o.b.UpdateReferences(o.ref, o.contacts));
  const FrameMotion des = FrameOracle(*o.model, q, v, o.ref.a_des, torso);
  o.b.TorsoTask()->SetSe3Reference(des.M, des.v, des.a);
  const Eigen::VectorXd r1 = Row(*r.b.TorsoTask(), r).r;
  EXPECT_LE(RelDiff(r1, Row(*o.b.TorsoTask(), o).r), 1e-12);
  // Off-reference the row is not trivially equal: a zero twist moves it.
  o.b.TorsoTask()->SetSe3Reference(des.M, Vec6::Zero(), des.a);
  const Eigen::VectorXd r2 = Row(*o.b.TorsoTask(), o).r;
  EXPECT_GT((r1 - r2).lpNorm<Eigen::Infinity>(), 1e-3);
}

TEST(LocomotionFormulation, AComTaskGetsTheReferenceCentreOfMass) {
  EXPECT_EQ(LocoRig{}.b.Com(), nullptr);  // optional: absent without the key
  YAML::Node wbq = YAML::Load(kLocoWbq);
  wbq["tasks"]["com"] = YAML::Load("{kp: [30, 30, 30], kd: [8, 8, 8], weight: 5.0}");
  LocoRig r(wbq);
  ASSERT_NE(r.b.Com(), nullptr);
  const Eigen::VectorXd q = QStand(*r.model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(r.model->nv);
  v(0) = 0.2;
  r.Set(q, v);
  r.ref.q_des = LiftRightFoot(*r.model, q);
  r.ref.v_des(6 + 9) = 0.4;
  r.ref.a_des(6 + 9) = 1.5;
  ASSERT_TRUE(r.b.UpdateReferences(r.ref, r.contacts));
  // The oracle: a CoMTask given pinocchio's reference centre of mass directly.
  pinocchio::Data d(*r.model);
  pinocchio::centerOfMass(*r.model, d, r.ref.q_des, r.ref.v_des, r.ref.a_des);
  EXPECT_LE((r.b.Kinematics().Com() - d.com[0]).lpNorm<Eigen::Infinity>(), 1e-12);
  PinocchioCache open;
  open.Init(r.model, {});
  CoMTask oracle;
  oracle.Init(*r.model, r.info, open, wbq["tasks"]["com"]);
  oracle.SetComReference(d.com[0], d.vcom[0], d.acom[0]);
  EXPECT_LE(RelDiff(Row(*r.b.Com(), r).r, Row(oracle, r).r), 1e-12);
  EXPECT_GT(d.acom[0].norm(), 1e-3);  // the feed-forward is not zero here
  EXPECT_TRUE(r.Solve().tau_valid);
}

// ── Robot-agnostic: the contact set is the spec's ─────────────────────────────

TEST(LocomotionFormulation, TheContactSetIsWhatTheSpecSays) {
  const YAML::Node wbq = YAML::Load(kLocoWbq);
  {
    LocomotionModelSpec rev = LocoSpec();
    std::swap(rev.contacts[0], rev.contacts[1]);
    LocoRig r(wbq, rev);
    EXPECT_EQ(r.b.NumFeet(), 2U);
    EXPECT_EQ(r.b.Contacts().contacts[0].frame_name, "right_sole");
    r.Set(QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
    r.Release(0);  // contact 0 is now the right foot
    (void)r.Solve();
    EXPECT_TRUE(r.b.FootTask(0)->IsActive());
    EXPECT_EQ(r.b.FootTask(0)->Name(), "foot_foot_b");
  }
  {
    LocomotionModelSpec three = LocoSpec();
    three.contacts.push_back({"hand", "left_wrist_yaw_link", 3});
    LocoRig r(wbq, three);
    EXPECT_EQ(r.b.NumFeet(), 3U);
    EXPECT_EQ(r.b.Contacts().max_contact_vars, 15);
    EXPECT_EQ(r.b.Contacts().contacts[2].contact_dim, 3);
  }
  {
    LocomotionModelSpec one = LocoSpec();
    one.contacts.pop_back();
    LocoRig r(wbq, one);
    EXPECT_EQ(r.b.NumFeet(), 1U);
    EXPECT_EQ(r.b.Contacts().max_contact_vars, 6);
  }
}

// ── The two bugs this path guards (#1040, #1041) ──────────────────────────────

TEST(LocomotionFormulation, TheForceTaskHasItsRowsOnTheTouchDownTick) {
  LocoRig r;
  r.Set(QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.Release(1);
  (void)r.Solve();
  EXPECT_EQ(r.b.Force()->ResidualDim(), 6);
  // Touch-down: both contacts on this tick — the rows are there before the solve.
  r.contacts.contacts[1].active = true;
  r.contacts.contacts[1].activation = 1.0;
  r.contacts.contacts[1].activation_target = 1.0;
  r.contacts.RecomputeActive(r.b.Contacts());
  ASSERT_TRUE(r.b.UpdateReferences(r.ref, r.contacts));
  EXPECT_EQ(r.b.Force()->ResidualDim(), r.contacts.active_contact_vars);
  EXPECT_EQ(r.b.Force()->ResidualDim(), 12);
}

TEST(LocomotionFormulation, ALockedCacheIsRefused) {
  LocoRig ok;  // twin: an unlocked cache builds
  EXPECT_EQ(ok.b.NumFeet(), 2U);
  auto model = SharedG1Model();
  RobotModelInfo info;
  info.Build(*model, YAML::Node{});
  PinocchioCache cache;
  InitUnlockedCache(cache, model);
  cache.Update(QStand(*model), Eigen::VectorXd::Zero(model->nv));  // locks registration
  LocomotionFormulationBuilder b;
  EXPECT_THROW(b.Init(*model, info, cache, LocoSpec(), YAML::Load(kLocoWbq), kLocoDt),
               std::invalid_argument);
  // A cache whose contact frames are not the spec's, in its order.
  PinocchioCache swapped;
  std::vector<pinocchio::FrameIndex> ids =
      LocomotionFormulationBuilder::ContactFrameIds(*model, LocoSpec());
  std::swap(ids[0], ids[1]);
  swapped.Init(model, ids);
  LocomotionFormulationBuilder bs;
  EXPECT_THROW(bs.Init(*model, info, swapped, LocoSpec(), YAML::Load(kLocoWbq), kLocoDt),
               std::invalid_argument);
  // And SE3Task itself refuses to register on a locked cache (#1041).
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "torso_link";
  cfg["base_frame"] = "universe";
  EXPECT_THROW(task.Init(*model, info, cache, cfg), std::runtime_error);
  PinocchioCache open;
  open.Init(model, {});
  SE3Task twin;
  EXPECT_NO_THROW(twin.Init(*model, info, open, cfg));
}

TEST(LocomotionFormulation, YamlRefusalsHaveAnAcceptedTwin) {
  EXPECT_NO_THROW(LocoRig{});
  auto refused = [](const std::function<void(YAML::Node&)>& edit) {
    YAML::Node n = YAML::Load(kLocoWbq);
    edit(n);
    EXPECT_THROW(LocoRig{n}, std::invalid_argument);
  };
  refused([](YAML::Node& n) { n["solve"] = 1; });  // unknown key
  refused([](YAML::Node& n) { n["tasks"]["arms"] = YAML::Load("{}"); });
  refused([](YAML::Node& n) { n["tasks"].remove("feet"); });
  refused([](YAML::Node& n) { n["constraints"].remove("contact"); });
  refused([](YAML::Node& n) { n["contact"]["mu"] = 0.4; });  // not the canonical key
  refused([](YAML::Node& n) { n["contact_overrides"]["foot_z"]["friction_coeff"] = 0.5; });
  refused([](YAML::Node& n) { n["formulation_type"] = "lqp"; });
  refused([](YAML::Node& n) { n["contact"]["t_ramp"] = -1.0; });
  refused([](YAML::Node& n) { n["contact_overrides"]["foot_b"]["t_ramp"] = 0.05; });  // one ramp
  refused([](YAML::Node& n) { n["tasks"]["feet"]["frame"] = "left_sole"; });  // the builder's key
  refused([](YAML::Node& n) { n["tasks"]["torso"] = YAML::Load("[0, 0, 0, 1, 1, 1]"); });
  refused([](YAML::Node& n) { n["constraints"].remove("eom"); });
  YAML::Node no_limits = YAML::Load(kLocoWbq);  // the optional constraints are optional
  no_limits["constraints"].remove("joint_limit");
  no_limits["constraints"].remove("torque_limit");
  EXPECT_NO_THROW(LocoRig{no_limits});
  YAML::Node over = YAML::Load(kLocoWbq);
  over["contact_overrides"]["foot_b"]["friction_coeff"] = 0.6;
  LocoRig r(over);
  EXPECT_EQ(r.b.Contacts().contacts[1].friction_coeff, 0.6);
  EXPECT_EQ(r.b.Contacts().contacts[0].friction_coeff, 0.4);
  for (const double period : {0.0, -kLocoDt, std::numeric_limits<double>::quiet_NaN()}) {
    auto model = SharedG1Model();
    RobotModelInfo info;
    info.Build(*model, YAML::Node{});
    PinocchioCache cache;
    InitUnlockedCache(cache, model);
    LocomotionFormulationBuilder b;
    EXPECT_THROW(b.Init(*model, info, cache, LocoSpec(), YAML::Load(kLocoWbq), period),
                 std::invalid_argument);
  }
  LocomotionModelSpec dim5 = LocoSpec();
  dim5.contacts[0].dim = 5;
  EXPECT_THROW((LocoRig{YAML::Load(kLocoWbq), dim5}), std::invalid_argument);
}

// The built terms point into the builder: it cannot be copied or moved.
static_assert(!std::is_copy_constructible_v<LocomotionFormulationBuilder>);
static_assert(!std::is_move_constructible_v<LocomotionFormulationBuilder>);
static_assert(!std::is_move_assignable_v<LocomotionFormulationBuilder>);

TEST(LocomotionFormulation, ARefusedReferenceSkipsTheSolveAndKeepsTheLastTorque) {
  LocoRig r;
  r.Set(QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  const CommandOutput& ok = r.Solve();
  ASSERT_TRUE(ok.tau_valid);  // the twin: a usable reference solves
  const Eigen::VectorXd tau = ok.tau;
  r.ref.q_des.segment<4>(3).setZero();  // a zero quaternion
  const CommandOutput& o = r.Solve();
  EXPECT_FALSE(o.tau_valid);
  EXPECT_FALSE(o.qp_converged);
  EXPECT_EQ(o.qp_status, -1);
  EXPECT_EQ(o.tau, tau);
  // Never a valid reference: the very first tick is refused too.
  LocoRig f;
  f.Set(QStand(*f.model), Eigen::VectorXd::Zero(f.model->nv));
  f.ref.q_des(9) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(f.Solve().tau_valid);
  EXPECT_FALSE(f.b.Kinematics().valid());
}

TEST(LocomotionFormulation, AReInitDropsTheTermsOfTheLastOne) {
  YAML::Node with_com = YAML::Load(kLocoWbq);
  with_com["tasks"]["com"] = YAML::Load("{kp: [30, 30, 30], kd: [8, 8, 8], weight: 5.0}");
  LocoRig r(with_com);
  ASSERT_NE(r.b.Com(), nullptr);
  PinocchioCache fresh;
  InitUnlockedCache(fresh, r.model);
  r.b.Init(*r.model, r.info, fresh, LocoSpec(), YAML::Load(kLocoWbq), kLocoDt);
  EXPECT_EQ(r.b.Com(), nullptr);
  EXPECT_EQ(r.b.NumFeet(), 2U);
}

TEST(ReferenceKinematics, AnOverflowIsRefusedNotNormalisedAway) {
  LocoRig r;
  const auto& model = *r.model;
  ReferenceKinematics rk;
  rk.Init(model, r.info, {model.getFrameId("right_sole")}, true);
  const Eigen::VectorXd q = QStand(model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);
  ASSERT_TRUE(rk.Update(q, zero, zero));
  const pinocchio::SE3 good = rk.Placement(0);
  Eigen::VectorXd huge_quat = q;
  huge_quat.segment<4>(3) *= 1e300;  // finite, but its norm overflows
  EXPECT_FALSE(rk.Update(huge_quat, zero, zero));
  Eigen::VectorXd v = zero;
  v.segment<3>(3).setConstant(1e200);  // finite input, ω × v overflows
  v(6 + 9) = 1e200;
  Eigen::VectorXd q_moved = q;  // another pose: a partial overwrite would show
  q_moved(7 + 9) += 0.3;
  EXPECT_FALSE(rk.Update(q_moved, v, zero));
  EXPECT_TRUE(rk.Placement(0).isApprox(good, 0.0));
  EXPECT_TRUE(rk.Twist(0).allFinite());
  EXPECT_TRUE(rk.ComAcceleration().allFinite());
}

}  // namespace
}  // namespace rtc::tsid
