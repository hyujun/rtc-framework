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
//   7. Contact acceleration by second-order forward kinematics at the QP's a
//      (#1017): forwardKinematics(q, v, a) then the foot frame's classical
//      acceleration (LOCAL_WORLD_ALIGNED), ‖·‖∞ ≤ 2·eps_abs + 64·nv·ε·S7,
//      S7 = ‖J_c‖∞‖a‖∞ + ‖J̇_c v‖∞. Not independent of the cache's dJv (the
//      same pinocchio routine at a = 0): it checks that J_c·a + dJv is that
//      routine's acceleration at a, i.e. that J_c and dJv are one convention.
//   8. The independent oracle: central differences of the frame velocity
//      along q(±h) = q ⊕ (±h·v + ½h²·a), v(±h) = v ± h·a, h = 1e-4 — no
//      Jacobian, no acceleration routine: ‖FD_{h/2}‖∞ ≤ 2·eps_abs
//      + 2·‖FD_h − FD_{h/2}‖∞ + 16·ε·‖w‖∞/h (Richardson estimate of the
//      truncation, roundoff of the quotient). Power: ‖J̇_c v‖∞ — what a zeroed
//      dJv removes — exceeds tol7 on both feet of ≥ 95 % of the converged
//      samples; control: a zeroed dJv in PinocchioCache (mutation) fails both.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>
#pragma GCC diagnostic pop

#include "g1_wqp_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

using namespace rtc::tsid::test;  // NOLINT — the shared rig

class FloatingWqpGates : public ::testing::Test {
 protected:
  void SetUp() override {
    RobotModelInfo info;
    info.Build(*test::SharedG1Model(), YAML::Node{});
    samples_ = Samples(*test::SharedG1Model(), info);
  }

  std::vector<Sample> samples_;
};

// The solution digest of the 200 samples under the shipped defaults (see Fnv1a).
TEST_F(FloatingWqpGates, SolutionDigestOfTheDefaults) {
  const Options o;
  std::uint64_t digest = 14695981039346656037ULL;
  int converged = 0;
  for (const auto& x : samples_) {
    Rig rig(o, x);
    const CommandOutput& out = rig.Solve();
    if (!out.qp_converged) {
      continue;
    }
    ++converged;
    digest = Fnv1a(Fnv1a(Fnv1a(digest, out.a_opt), out.lambda_opt), out.tau);
  }
  char hex[32];
  std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(digest));
  RecordProperty("solution_digest", hex);
  RecordProperty("solution_digest_converged", std::to_string(converged));
  std::printf("solution_digest %s over %d converged samples\n", hex, converged);
  EXPECT_GE(converged, 0.9 * kSamples);
}

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
  ASSERT_GT(n.converged, 0) << "no converged sample: every gate above is vacuous";
  RecordProperty("eom_ratio_max",
                 std::to_string(*std::max_element(eom_ratio.begin(), eom_ratio.end())));
  RecordProperty("eom_ratio_p99", std::to_string(P99(eom_ratio)));
  RecordProperty("contact_ratio_p99", std::to_string(P99(contact_ratio)));
  RecordProperty("tau_ratio_max",
                 std::to_string(*std::max_element(tau_ratio.begin(), tau_ratio.end())));
  RecordProperty("tau_ratio_p99", std::to_string(P99(tau_ratio)));
  EXPECT_EQ(perturbation_caught, n.converged) << "control: a 1e-4 change of a must fail gate 1";
}

// Gates 7 and 8: the contact acceleration against second-order kinematics at
// the QP's a (7) and against finite differences that use neither the Jacobian
// nor an acceleration routine (8); same samples, same rig as gate 2.
TEST_F(FloatingWqpGates, ContactAccelerationMatchesKinematicOracles) {
  const Options o;
  constexpr double kH = 1e-4;
  Counters n;
  std::vector<double> r7, r8;
  int powered_samples = 0;
  for (const auto& x : samples_) {
    Rig rig(o, x);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    const Eigen::VectorXd& a = out.a_opt;
    const pinocchio::Model& model = *rig.model;
    pinocchio::Data d(model);
    pinocchio::forwardKinematics(model, d, rig.state.q, rig.state.v, a);
    pinocchio::Data scratch(model);  // the finite-difference evaluations
    int powered_feet = 0;
    for (int i = 0; i < 2; ++i) {
      const auto fid =
          static_cast<pinocchio::FrameIndex>(rig.mgr.contacts[static_cast<size_t>(i)].frame_id);
      const auto& fc = rig.cache.contact_frames[static_cast<size_t>(i)];

      // 7. Second-order forward kinematics at the QP's a.
      const double e7 =
          pinocchio::getFrameClassicalAcceleration(model, d, fid, pinocchio::LOCAL_WORLD_ALIGNED)
              .toVector()
              .lpNorm<Eigen::Infinity>();
      const double djv_inf = fc.dJv.lpNorm<Eigen::Infinity>();
      const double S7 =
          fc.J.cwiseAbs().rowwise().sum().maxCoeff() * a.lpNorm<Eigen::Infinity>() + djv_inf;
      const double tol7 = 2.0 * o.eps_abs + 64.0 * rig.info.nv * kEps * S7;
      r7.push_back(e7 / tol7);
      EXPECT_LE(e7, tol7) << "foot " << i;
      powered_feet += djv_inf > tol7 ? 1 : 0;

      // 8. Central differences of the frame velocity along the constant-a arc.
      const auto frame_velocity = [&](double t) -> Eigen::Matrix<double, 6, 1> {
        const Eigen::VectorXd dq = t * rig.state.v + 0.5 * t * t * a;
        const Eigen::VectorXd q_t = pinocchio::integrate(model, rig.state.q, dq);
        const Eigen::VectorXd v_t = rig.state.v + t * a;
        pinocchio::forwardKinematics(model, scratch, q_t, v_t);
        return pinocchio::getFrameVelocity(model, scratch, fid, pinocchio::LOCAL_WORLD_ALIGNED)
            .toVector();
      };
      const auto fd = [&](double step) -> Eigen::Matrix<double, 6, 1> {
        return (frame_velocity(step) - frame_velocity(-step)) / (2.0 * step);
      };
      const Eigen::Matrix<double, 6, 1> fd_h = fd(kH);
      const Eigen::Matrix<double, 6, 1> fd_h2 = fd(0.5 * kH);
      // w at (q, v): d holds it (forwardKinematics with a fills the velocities).
      const double w_inf =
          pinocchio::getFrameVelocity(model, d, fid, pinocchio::LOCAL_WORLD_ALIGNED)
              .toVector()
              .lpNorm<Eigen::Infinity>();
      const double e8 = fd_h2.lpNorm<Eigen::Infinity>();
      const double tol8 = 2.0 * o.eps_abs + 2.0 * (fd_h - fd_h2).lpNorm<Eigen::Infinity>() +
                          16.0 * kEps * w_inf / kH;
      r8.push_back(e8 / tol8);
      EXPECT_LE(e8, tol8) << "foot " << i;
    }
    powered_samples += powered_feet == 2 ? 1 : 0;
  }
  n.Report("oracle");
  ASSERT_GT(n.converged, 0) << "no converged sample: the gates above are vacuous";
  RecordProperty("classical_ratio_max", std::to_string(*std::max_element(r7.begin(), r7.end())));
  RecordProperty("classical_ratio_p99", std::to_string(P99(r7)));
  RecordProperty("fd_ratio_max", std::to_string(*std::max_element(r8.begin(), r8.end())));
  RecordProperty("fd_ratio_p99", std::to_string(P99(r8)));
  RecordProperty("powered_samples", std::to_string(powered_samples));
  EXPECT_GE(powered_samples, static_cast<int>(std::ceil(0.95 * n.converged)))
      << "power: ‖J̇_c v‖∞ must sit above tol7 on both feet for a zeroed dJv to be caught";
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
