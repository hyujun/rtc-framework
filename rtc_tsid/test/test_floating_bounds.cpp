// The W6-F01 [SPRINT] v3 PR A gates (#1039): on the gates' G1 rig, the terms
// the locomotion QP adds to the existing constraints — the Baumgarte contact
// target with its latched anchor, the joint acceleration box, the torque-rate
// box, the valid-torque flag and the solver keys — each against an
// independent oracle, with the shipped defaults pinned bit for bit (every new
// term off = the old bytes). Numbers (a_max = 30 + i, δ = {0: +3, 7: −4,
// 14: +5}, r·dt = 2, the gains) are the v3 registration from the C++ probe.
#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include "g1_wqp_fixture.hpp"
#include "rtc_tsid/formulation/hqp_formulation.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

#include <cstring>
#include <stdexcept>

namespace rtc::tsid {
namespace {

using namespace rtc::tsid::test;  // NOLINT — the shared rig

constexpr double kInf = std::numeric_limits<double>::infinity();
const double kNaN = std::numeric_limits<double>::quiet_NaN();

YAML::Node BaumgarteYaml(bool zero = false) {
  YAML::Node n;
  if (zero) {
    n["baumgarte"]["kp"] = 0.0;
    n["baumgarte"]["kd"] = 0.0;
  } else {
    n["baumgarte"]["kp"] = YAML::Load("[11, 13, 17, 19, 23, 29]");
    n["baumgarte"]["kd"] = YAML::Load("[2, 3, 5, 7, 11, 13]");
  }
  return n;
}

Eigen::Matrix<double, 6, 1> Gains(bool kp) {
  Eigen::Matrix<double, 6, 1> g;
  if (kp) {
    g << 11, 13, 17, 19, 23, 29;
  } else {
    g << 2, 3, 5, 7, 11, 13;
  }
  return g;
}

// The contact constraint's rows, assembled directly (the caller zero-inits).
struct Rows {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

Rows ContactRows(Rig& rig) {
  const int n_vars = rig.info.nv + rig.mgr.max_contact_vars;
  Rows r{Eigen::MatrixXd::Zero(rig.mgr.max_contact_vars, n_vars),
         Eigen::VectorXd::Zero(rig.mgr.max_contact_vars)};
  rig.contact_accel->ComputeEquality(rig.cache, rig.contacts, rig.info, n_vars, r.A, r.b);
  return r;
}

bool SameBytes(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), static_cast<size_t>(a.size()) * sizeof(double)) == 0;
}

class FloatingBounds : public ::testing::Test {
 protected:
  void SetUp() override {
    info_.Build(*SharedG1Model(), YAML::Node{});
    samples_ = Samples(*SharedG1Model(), info_);
  }

  RobotModelInfo info_;
  std::vector<Sample> samples_;
};

// ── Defaults unchanged ────────────────────────────────────────────────────────

// Without a `baumgarte:` key, and with explicit zero gains, the contact rows
// are exactly J and −J̇v (memcmp), nothing is latched, and the solution bytes
// equal the plain rig's; absent acceleration_max / rate_max leave the row
// counts and the solution bytes as they were.
TEST_F(FloatingBounds, EveryNewTermOffLeavesTheOldBytes) {
  for (const auto& x : samples_) {
    Rig plain(Options{}, x);
    Options zero;
    zero.contact_accel = BaumgarteYaml(/*zero=*/true);
    Rig zeroed(zero, x);
    const CommandOutput& op = plain.Solve();
    const CommandOutput& oz = zeroed.Solve();
    EXPECT_FALSE(plain.contact_accel->BaumgarteEnabled());
    EXPECT_FALSE(zeroed.contact_accel->BaumgarteEnabled());
    EXPECT_FALSE(zeroed.contact_accel->AnchorLatched(0));
    EXPECT_FALSE(plain.joint_limit->AccelerationMaxEnabled());
    EXPECT_FALSE(plain.torque_limit->RateMaxEnabled());
    for (Rig* rig : {&plain, &zeroed}) {
      const Rows r = ContactRows(*rig);
      for (int c = 0; c < 2; ++c) {
        const auto& fc = rig->cache.contact_frames[static_cast<size_t>(c)];
        const Eigen::MatrixXd J = fc.J.topRows(6);
        const Eigen::VectorXd minus_dJv = -fc.dJv.head(6);
        const Eigen::MatrixXd A = r.A.block(6 * c, 0, 6, rig->info.nv);
        const Eigen::VectorXd b = r.b.segment(6 * c, 6);
        ASSERT_EQ(std::memcmp(A.data(), J.data(), sizeof(double) * 6 * rig->info.nv), 0);
        ASSERT_TRUE(SameBytes(b, minus_dJv));
      }
    }
    ASSERT_EQ(op.qp_converged, oz.qp_converged);
    if (op.qp_converged) {
      ASSERT_TRUE(SameBytes(op.a_opt, oz.a_opt));
      ASSERT_TRUE(SameBytes(op.lambda_opt, oz.lambda_opt));
      ASSERT_TRUE(SameBytes(op.tau, oz.tau));
      ASSERT_TRUE(op.tau_valid);
      ASSERT_EQ(op.n_ineq_used, oz.n_ineq_used);
    }
    // The row budget is the same: no term adds rows.
    ASSERT_EQ(plain.joint_limit->MaxIneqDim(plain.mgr), info_.n_actuated);
    ASSERT_EQ(plain.torque_limit->MaxIneqDim(plain.mgr), info_.n_actuated);
  }
}

// ── Baumgarte ─────────────────────────────────────────────────────────────────

// After the latch the base moves (−0.01, 0.02, −0.01) and yaws 0.1 (the
// velocities are the sample's): a_c* must equal −Kp⊙e − Kd⊙v_c with e and
// v_c from pinocchio directly (log3, getFrameVelocity), and the QP's contact
// acceleration must reproduce a_c*.
TEST_F(FloatingBounds, BaumgarteTargetMatchesPinocchioAndTheSolveReachesIt) {
  Options o;
  o.contact_accel = BaumgarteYaml();
  Counters n;
  int yawed = 0;
  for (const auto& x : samples_) {
    Rig rig(o, x);
    ASSERT_TRUE(rig.contact_accel->BaumgarteEnabled());
    (void)rig.Solve();  // latches both feet where they are
    std::vector<pinocchio::SE3> anchor = {rig.contact_accel->Anchor(0),
                                          rig.contact_accel->Anchor(1)};
    // Move the base and keep the sample's velocity.
    Eigen::VectorXd q = x.q;
    q.head(3) += Eigen::Vector3d(-0.01, 0.02, -0.01);
    const Eigen::Quaterniond yaw(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond qb(q(6), q(3), q(4), q(5));
    const Eigen::Quaterniond qn = (yaw * qb).normalized();
    q.segment(3, 4) << qn.x(), qn.y(), qn.z(), qn.w();
    rig.state.q = q;
    rig.cache.Update(q, x.v);
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    for (int c = 0; c < 2; ++c) {
      ASSERT_TRUE(rig.contact_accel->AnchorLatched(static_cast<size_t>(c)));
      ASSERT_TRUE(rig.contact_accel->Anchor(static_cast<size_t>(c))
                      .isApprox(anchor[static_cast<size_t>(c)], 0.0));
    }
    // Oracle: pinocchio frame placement and LWA velocity at (q, v).
    pinocchio::Data d(*rig.model);
    pinocchio::forwardKinematics(*rig.model, d, q, x.v);
    pinocchio::updateFramePlacements(*rig.model, d);
    for (int c = 0; c < 2; ++c) {
      const auto fid =
          static_cast<pinocchio::FrameIndex>(rig.mgr.contacts[static_cast<size_t>(c)].frame_id);
      const pinocchio::SE3& oMf = d.oMf[fid];
      const pinocchio::SE3& A = anchor[static_cast<size_t>(c)];
      Eigen::Matrix<double, 6, 1> e;
      e.head<3>() = oMf.translation() - A.translation();
      e.tail<3>() = pinocchio::log3(oMf.rotation() * A.rotation().transpose());
      const pinocchio::Motion vm =
          pinocchio::getFrameVelocity(*rig.model, d, fid, pinocchio::LOCAL_WORLD_ALIGNED);
      Eigen::Matrix<double, 6, 1> vc;
      vc << vm.linear(), vm.angular();
      const Eigen::Matrix<double, 6, 1> expect =
          -(Gains(true).array() * e.array() + Gains(false).array() * vc.array());
      const double scale = (Gains(true).array() * e.array()).matrix().norm() +
                           (Gains(false).array() * vc.array()).matrix().norm();
      const double tol = 1e-12 * std::max(1.0, scale);
      const Eigen::Matrix<double, 6, 1>& got =
          rig.contact_accel->TargetAcceleration(static_cast<size_t>(c));
      ASSERT_LE((got - expect).lpNorm<Eigen::Infinity>(), tol) << "contact " << c;
      // Every axis carries error (the displacement is three-axis, the yaw one).
      ASSERT_GT(e.head<3>().cwiseAbs().minCoeff(), 1e-6);
      ASSERT_GT(e.tail<3>().lpNorm<Eigen::Infinity>(), 1e-3);
      const double foot_yaw = std::atan2(oMf.rotation()(1, 0), oMf.rotation()(0, 0));
      yawed += std::fabs(foot_yaw) > 0.1;
      if (!out.qp_converged) {
        continue;
      }
      const auto& fc = rig.cache.contact_frames[static_cast<size_t>(c)];
      const Eigen::VectorXd res = fc.J * out.a_opt + fc.dJv - got;
      const double S =
          fc.J.cwiseAbs().rowwise().sum().maxCoeff() * out.a_opt.lpNorm<Eigen::Infinity>() +
          fc.dJv.lpNorm<Eigen::Infinity>() + got.lpNorm<Eigen::Infinity>();
      const double tol_res = 2.0 * o.eps_abs + 64.0 * rig.info.nv * kEps * S;
      ASSERT_LE(res.lpNorm<Eigen::Infinity>(), tol_res) << "contact " << c;
    }
  }
  n.Report("baumgarte");
  EXPECT_GT(yawed, kSamples);  // most feet are yawed: the rotation error is not axis-aligned
}

TEST_F(FloatingBounds, TheAnchorLatchesOnTheActivationEdgeAndStaysUntilResetOrRelease) {
  const Sample& x = samples_[3];
  Options o;
  o.contact_accel = BaumgarteYaml();
  o.second_contact_active = false;
  Rig rig(o, x);
  (void)rig.Solve();
  EXPECT_TRUE(rig.contact_accel->AnchorLatched(0));   // active from Init: first Compute
  EXPECT_FALSE(rig.contact_accel->AnchorLatched(1));  // inactive: nothing
  // The activation edge, with the ramp only 0.02 in: latched on that tick.
  rig.contacts.contacts[1].active = true;
  rig.contacts.contacts[1].activation = 0.0;
  rig.contacts.contacts[1].activation_target = 1.0;
  rig.contacts.contacts[1].t_ramp_sec = 0.1;
  rig.contacts.UpdateActivation(0.002);
  rig.contacts.RecomputeActive(rig.mgr);
  ASSERT_NEAR(rig.contacts.contacts[1].activation, 0.02, 1e-12);
  (void)rig.Solve();
  ASSERT_TRUE(rig.contact_accel->AnchorLatched(1));
  const pinocchio::SE3 a1 = rig.contact_accel->Anchor(1);
  EXPECT_TRUE(a1.isApprox(rig.cache.contact_frames[1].oMf, 0.0));
  // A changed configuration (the second foot's own leg) does not move the anchor (bytes).
  Eigen::VectorXd q = x.q;
  q(7 + 9) += 0.05;  // a joint of the leg contact 1 hangs from (its foot moves)
  rig.cache.Update(q, x.v);
  ASSERT_FALSE(rig.cache.contact_frames[1].oMf.isApprox(a1, 1e-9));
  (void)rig.Solve();
  EXPECT_EQ(std::memcmp(rig.contact_accel->Anchor(1).translation().data(), a1.translation().data(),
                        3 * sizeof(double)),
            0);
  EXPECT_EQ(std::memcmp(rig.contact_accel->Anchor(1).rotation().data(), a1.rotation().data(),
                        9 * sizeof(double)),
            0);
  // Release and re-activate: a new anchor at the new pose.
  rig.contacts.contacts[1].active = false;
  rig.contacts.RecomputeActive(rig.mgr);
  (void)rig.Solve();
  EXPECT_FALSE(rig.contact_accel->AnchorLatched(1));
  rig.contacts.contacts[1].active = true;
  rig.contacts.RecomputeActive(rig.mgr);
  (void)rig.Solve();
  ASSERT_TRUE(rig.contact_accel->AnchorLatched(1));
  EXPECT_FALSE(rig.contact_accel->Anchor(1).isApprox(a1, 1e-9));
  EXPECT_TRUE(rig.contact_accel->Anchor(1).isApprox(rig.cache.contact_frames[1].oMf, 0.0));
  // A phase preset turning the constraint off and on re-latches (nothing
  // watched the feet while it was off): the anchor is where the foot is now.
  const pinocchio::SE3 a0 = rig.contact_accel->Anchor(0);
  PhasePreset off;
  off.constraint_presets.push_back({"contact_accel", false});
  rig.ctrl.ApplyPhasePreset(off);
  (void)rig.Solve();
  Eigen::VectorXd moved = x.q;
  moved(7 + 3) -= 0.04;  // contact 0's leg moved while the constraint was off
  rig.cache.Update(moved, x.v);
  PhasePreset on;
  on.constraint_presets.push_back({"contact_accel", true});
  rig.ctrl.ApplyPhasePreset(on);
  EXPECT_FALSE(rig.contact_accel->AnchorLatched(0));
  (void)rig.Solve();
  EXPECT_FALSE(rig.contact_accel->Anchor(0).isApprox(a0, 1e-9));
  EXPECT_TRUE(rig.contact_accel->Anchor(0).isApprox(rig.cache.contact_frames[0].oMf, 0.0));
  rig.cache.Update(x.q, x.v);
  // Reset does too: the next Compute latches where the foot is now.
  rig.ctrl.Reset();
  EXPECT_FALSE(rig.contact_accel->AnchorLatched(0));
  q = x.q;
  q(7 + 3) += 0.05;  // contact 0's leg
  rig.cache.Update(q, x.v);
  (void)rig.Solve();
  ASSERT_TRUE(rig.contact_accel->AnchorLatched(0));
  EXPECT_TRUE(rig.contact_accel->Anchor(0).isApprox(rig.cache.contact_frames[0].oMf, 0.0));
}

// A non-finite pose on the edge tick is not an anchor: the latch waits.
TEST_F(FloatingBounds, ANonFinitePoseIsNotLatched) {
  const Sample& x = samples_[8];
  Options o;
  o.contact_accel = BaumgarteYaml();
  o.second_contact_active = false;
  Rig rig(o, x);
  (void)rig.Solve();
  rig.contacts.contacts[1].active = true;
  rig.contacts.RecomputeActive(rig.mgr);
  rig.cache.contact_frames[1].oMf.translation()[0] = kNaN;
  (void)rig.Solve();
  EXPECT_FALSE(rig.contact_accel->AnchorLatched(1));
  EXPECT_EQ(rig.contact_accel->TargetAcceleration(1).lpNorm<Eigen::Infinity>(), 0.0);
  rig.cache.Update(x.q, x.v);  // a finite pose again: latched now
  (void)rig.Solve();
  EXPECT_TRUE(rig.contact_accel->AnchorLatched(1));
  EXPECT_TRUE(rig.contact_accel->Anchor(1).translation().allFinite());
}

TEST_F(FloatingBounds, TerrainHeightReplacesTheLatchedZ) {
  const Sample& x = samples_[5];
  Options o;
  o.contact_accel = BaumgarteYaml();
  o.contact_accel["terrain_height"] = 0.3;
  Rig rig(o, x);
  (void)rig.Solve();
  for (int c = 0; c < 2; ++c) {
    const auto& fc = rig.cache.contact_frames[static_cast<size_t>(c)];
    EXPECT_EQ(rig.contact_accel->Anchor(static_cast<size_t>(c)).translation()[2], 0.3);
    const double e_z = fc.oMf.translation()[2] - 0.3;
    // a_c*_z = −kp_z·e_z − kd_z·v_z
    const Eigen::Matrix<double, 6, 1> vc = fc.J * rig.cache.v;
    EXPECT_NEAR(rig.contact_accel->TargetAcceleration(static_cast<size_t>(c))(2),
                -17.0 * e_z - 5.0 * vc(2),
                1e-12 * std::max(1.0, std::fabs(17.0 * e_z) + std::fabs(5.0 * vc(2))));
  }
}

TEST_F(FloatingBounds, APointContactUsesTheFirstThreeRowsAndGains) {
  const Sample& x = samples_[7];
  Options o;
  o.contact_accel = BaumgarteYaml();
  Rig rig(o, x);
  // Re-declare the second foot as a point contact for this rig only.
  rig.mgr.contacts[1].contact_dim = 3;
  rig.mgr.max_contact_vars = 9;
  Rows r{Eigen::MatrixXd::Zero(12, rig.info.nv + 12), Eigen::VectorXd::Zero(12)};
  rig.contact_accel->ComputeEquality(rig.cache, rig.contacts, rig.info, rig.info.nv + 12, r.A, r.b);
  const auto& fc = rig.cache.contact_frames[1];
  const Eigen::Matrix<double, 6, 1>& a_star = rig.contact_accel->TargetAcceleration(1);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(r.b(6 + i), -fc.dJv(i) + a_star(i), 1e-12 * std::max(1.0, std::fabs(a_star(i))));
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_EQ(r.b(6 + i), 0.0);  // beyond the point contact's rows: untouched
    EXPECT_EQ(r.A.row(6 + i).lpNorm<Eigen::Infinity>(), 0.0);
  }
}

TEST_F(FloatingBounds, BaumgarteYamlRefusalsHaveAcceptedTwins) {
  const Sample& x = samples_[0];
  auto init = [&](const YAML::Node& n) {
    Options o;
    o.contact_accel = n;
    Rig rig(o, x);
    return rig.contact_accel->BaumgarteEnabled();
  };
  EXPECT_TRUE(init(BaumgarteYaml()));
  EXPECT_FALSE(init(BaumgarteYaml(true)));
  YAML::Node five;
  five["baumgarte"]["kp"] = YAML::Load("[1, 2, 3, 4, 5]");
  EXPECT_THROW((void)init(five), std::invalid_argument);
  YAML::Node negative;
  negative["baumgarte"]["kd"] = -1.0;
  EXPECT_THROW((void)init(negative), std::invalid_argument);
  YAML::Node nan;
  nan["baumgarte"]["kp"] = YAML::Load(".nan");
  EXPECT_THROW((void)init(nan), std::invalid_argument);
  YAML::Node terrain;
  terrain["terrain_height"] = YAML::Load(".inf");
  EXPECT_THROW((void)init(terrain), std::invalid_argument);
  YAML::Node scalar;
  scalar["baumgarte"]["kp"] = 3.0;
  EXPECT_TRUE(init(scalar));
}

// ── Acceleration box ──────────────────────────────────────────────────────────

Options Boxed() {
  Options o;
  YAML::Node am(YAML::NodeType::Sequence);
  for (int i = 0; i < kG1Actuated; ++i) {
    am.push_back(30.0 + i);
  }
  o.joint_limit_yaml["acceleration_max"] = am;
  return o;
}

// A ±500 rad/s² posture demand (q_des = q ± 5, alternating) so the box binds.
Sample Demanding(const Sample& x, const RobotModelInfo& info) {
  Sample d = x;
  for (int i = 0; i < info.n_actuated; ++i) {
    d.q_des(7 + i) = x.q(7 + i) + (i % 2 == 0 ? 5.0 : -5.0) - 0.2 * x.v(6 + i);
  }
  return d;
}

TEST_F(FloatingBounds, AccelerationMaxBindsWhereTheControlViolates) {
  const Options boxed = Boxed();
  Counters n;
  int ctrl_violating = 0, bind_up = 0, bind_lo = 0;
  for (const auto& x : samples_) {
    const Sample d = Demanding(x, info_);
    Rig ctrl(Options{}, d);
    const CommandOutput& oc = ctrl.Solve();
    if (oc.qp_converged) {
      bool v = false;
      for (int i = 0; i < info_.n_actuated; ++i) {
        v = v || std::fabs(oc.a_opt(6 + i)) > 30.0 + i;
      }
      ctrl_violating += v;
    }
    Rig rig(boxed, d);
    ASSERT_TRUE(rig.joint_limit->AccelerationMaxEnabled());
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    bool any = false;
    for (int i = 0; i < info_.n_actuated; ++i) {
      const double m = 30.0 + i;
      const double a = out.a_opt(6 + i);
      ASSERT_LE(std::fabs(a), m + boxed.eps_abs + 4.0 * kEps * m) << "joint " << i;
      if (a >= m - 1e-6) {
        ++bind_up;
        any = true;
      }
      if (a <= -m + 1e-6) {
        ++bind_lo;
        any = true;
      }
    }
    ASSERT_TRUE(any);
  }
  n.Report("accel_box");
  EXPECT_GE(ctrl_violating, 1);
  EXPECT_GE(bind_up, 1);
  EXPECT_GE(bind_lo, 1);
}

TEST_F(FloatingBounds, AccelerationMaxRowsAndTheirCorners) {
  const Sample& x = samples_[11];
  Rig rig(Boxed(), x);
  const int n_vars = rig.info.nv + rig.mgr.max_contact_vars;
  const int na = rig.info.n_actuated;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(na, n_vars);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(na), u = Eigen::VectorXd::Zero(na);
  rig.joint_limit->ComputeInequality(rig.cache, rig.contacts, rig.info, n_vars, C, l, u);
  for (int i = 0; i < na; ++i) {
    for (int j = 0; j < n_vars; ++j) {
      ASSERT_EQ(C(i, j), j == rig.info.n_base_v + i ? 1.0 : 0.0) << i << " " << j;
    }
    ASSERT_GE(l(i), -(30.0 + i));
    ASSERT_LE(u(i), 30.0 + i);
    ASSERT_LE(l(i), u(i));
  }
  EXPECT_EQ(rig.joint_limit->CutCount(), 0);
  // NUM-7: a non-finite velocity row stays non-finite (l first: max keeps NaN).
  Rig nan(Boxed(), x);
  nan.cache.v(rig.info.n_base_v + 2) = kNaN;
  nan.cache.v(rig.info.n_base_v + 5) = kInf;  // an infinite row is not laundered either
  nan.joint_limit->ComputeInequality(nan.cache, nan.contacts, nan.info, n_vars, C, l, u);
  EXPECT_TRUE(std::isnan(l(2)));
  EXPECT_TRUE(std::isnan(u(2)));
  EXPECT_FALSE(std::isfinite(l(5)) && std::isfinite(u(5)));
  EXPECT_EQ(nan.joint_limit->CutCount(), 0);
  // A collapsed row (joint inside the position band, moving out) pinned beyond
  // a_max is cut to the box edge: with a tiny a_max every pinned row is cut.
  Options tiny;
  tiny.joint_limit_yaml["acceleration_max"] = 1.0;
  Rig band(tiny, x);
  Eigen::VectorXd q = x.q;
  q(7 + 3) = band.info.q_upper(7 + 3) - 0.5 * kMargin;  // inside the margin band
  Eigen::VectorXd v = x.v;
  v(6 + 3) = 0.0;
  band.cache.Update(q, v);
  band.joint_limit->ComputeInequality(band.cache, band.contacts, band.info, n_vars, C, l, u);
  EXPECT_GE(band.joint_limit->InvertedCount(), 1);
  EXPECT_GE(band.joint_limit->CutCount(), 1);
  EXPECT_EQ(l(3), u(3));
  EXPECT_LE(std::fabs(l(3)), 1.0);
  // Refusals with accepted twins.
  Options bad;
  bad.joint_limit_yaml["acceleration_max"] = 0.0;
  EXPECT_THROW((void)Rig(bad, x), std::invalid_argument);
  bad.joint_limit_yaml["acceleration_max"] = YAML::Load("[1, 2, 3]");
  EXPECT_THROW((void)Rig(bad, x), std::invalid_argument);
  Options ok;
  ok.joint_limit_yaml["acceleration_max"] = 1e-3;
  EXPECT_NO_THROW((void)Rig(ok, x));
}

// ── Torque rate ───────────────────────────────────────────────────────────────

Options Rated() {
  Options o;
  o.torque_limit_yaml["rate_max"] = 1000.0;
  o.torque_limit_yaml["dt"] = 0.002;
  return o;
}

Eigen::VectorXd Delta(const Eigen::VectorXd& tau_u) {
  Eigen::VectorXd t = tau_u;
  t(0) += 3.0;
  t(7) -= 4.0;
  t(14) += 5.0;
  return t;
}

TEST_F(FloatingBounds, TorqueRateBindsAroundThePreviousTorque) {
  const Options rated = Rated();
  Counters n;
  for (const auto& x : samples_) {
    Rig plain(Options{}, x);
    const CommandOutput& ou = plain.Solve();
    if (!ou.qp_converged) {
      continue;
    }
    Rig rig(rated, x);
    ASSERT_TRUE(rig.torque_limit->RateMaxEnabled());
    ASSERT_FALSE(rig.torque_limit->HasPreviousTorque());
    // Injected previous torque: τ_u + δ on three joints.
    rig.torque_limit->OnTorqueApplied(Delta(ou.tau));
    ASSERT_TRUE(rig.torque_limit->HasPreviousTorque());
    const CommandOutput& out = rig.Solve();
    n.Record(out);
    if (!out.qp_converged) {
      continue;
    }
    const Eigen::VectorXd tau_prev = Delta(ou.tau);
    const double S =
        rig.cache.M.cwiseAbs().rowwise().sum().maxCoeff() * out.a_opt.lpNorm<Eigen::Infinity>() +
        rig.cache.h.lpNorm<Eigen::Infinity>() + out.tau.lpNorm<Eigen::Infinity>();
    const double tol = rated.eps_abs + 64.0 * rig.info.nv * kEps * S;
    bool any = false;
    for (int i = 0; i < info_.n_actuated; ++i) {
      const double d = std::fabs(out.tau(i) - tau_prev(i));
      // A rate box beyond the effort limit is pinned to it (the plain solution
      // already sat on that limit): then τ is the limit, not within ±2.
      const double lim_lo = info_.tau_min(i), lim_hi = info_.tau_max(i);
      if (tau_prev(i) - 2.0 > lim_hi) {
        ASSERT_LE(std::fabs(out.tau(i) - lim_hi), tol) << "joint " << i;
        continue;
      }
      if (tau_prev(i) + 2.0 < lim_lo) {
        ASSERT_LE(std::fabs(out.tau(i) - lim_lo), tol) << "joint " << i;
        continue;
      }
      ASSERT_LE(d, 2.0 + tol) << "joint " << i;
      any = any || std::fabs(d - 2.0) <= 1e-5;
    }
    ASSERT_TRUE(any);
    ASSERT_GT((out.tau - ou.tau).lpNorm<Eigen::Infinity>(), 1.0);
  }
  n.Report("torque_rate");
}

TEST_F(FloatingBounds, TheFirstTickHasNoRateBoundAndOnlyValidTicksUpdateIt) {
  const Sample& x = samples_[2];
  Rig plain(Options{}, x);
  Rig rig(Rated(), x);
  const int n_vars = rig.info.nv + rig.mgr.max_contact_vars;
  const int na = rig.info.n_actuated;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(na, n_vars), Cp = C;
  Eigen::VectorXd l(na), u(na), lp(na), up(na);
  rig.torque_limit->ComputeInequality(rig.cache, rig.contacts, rig.info, n_vars, C, l, u);
  plain.torque_limit->ComputeInequality(plain.cache, plain.contacts, plain.info, n_vars, Cp, lp,
                                        up);
  EXPECT_TRUE(SameBytes(l, lp));  // no previous torque: the effort limits alone
  EXPECT_TRUE(SameBytes(u, up));
  const CommandOutput& o1 = rig.Solve();
  ASSERT_TRUE(o1.tau_valid);
  ASSERT_TRUE(rig.torque_limit->HasPreviousTorque());
  const Eigen::VectorXd first = o1.tau;
  EXPECT_TRUE(SameBytes(rig.torque_limit->PreviousTorque(), first));
  // A non-converged tick (q_des NaN → non-finite iterates) applies nothing.
  rig.ref.q_des(8) = kNaN;
  const CommandOutput& o2 = rig.Solve();
  EXPECT_FALSE(o2.tau_valid);
  EXPECT_TRUE(o2.qp_non_finite);
  EXPECT_TRUE(SameBytes(o2.tau, first));  // the last valid torque stays
  EXPECT_TRUE(SameBytes(rig.torque_limit->PreviousTorque(), first));
  rig.ref.q_des = x.q_des;
  const CommandOutput& o3 = rig.Solve();
  ASSERT_TRUE(o3.tau_valid);
  // The rate bound was taken from the first valid torque.
  for (int i = 0; i < na; ++i) {
    ASSERT_LE(std::fabs(o3.tau(i) - first(i)), 2.0 + 1e-5) << i;
  }
  // Reset forgets it.
  rig.ctrl.Reset();
  EXPECT_FALSE(rig.torque_limit->HasPreviousTorque());
  EXPECT_FALSE(rig.ctrl.Compute(rig.state, rig.ref, rig.cache, rig.contacts).tau_valid ==
               false);  // a plain tick again valid
  // A preset turning torque_limit off turns the rate off with it (no rows).
  Rig off(Rated(), x);
  const CommandOutput& ob = off.Solve();
  ASSERT_TRUE(ob.tau_valid);
  const int rows_on = ob.n_ineq_used;
  PhasePreset preset;
  preset.constraint_presets.push_back({"torque_limit", false});
  off.ctrl.ApplyPhasePreset(preset);
  const CommandOutput& oc = off.Solve();
  EXPECT_EQ(oc.n_ineq_used, rows_on - na);
  // dt is read and checked; a τ_prev beyond the effort limits is pinned.
  EXPECT_EQ(rig.torque_limit->Dt(), 0.002);
  Options bad_dt = Rated();
  bad_dt.torque_limit_yaml["dt"] = 0.0;
  EXPECT_THROW((void)Rig(bad_dt, x), std::invalid_argument);
  Options bad_rate = Rated();
  bad_rate.torque_limit_yaml["rate_max"] = -1.0;
  EXPECT_THROW((void)Rig(bad_rate, x), std::invalid_argument);
  Rig pinned(Rated(), x);
  Eigen::VectorXd nan_tau = Eigen::VectorXd::Constant(na, kNaN);
  pinned.torque_limit->OnTorqueApplied(nan_tau);
  EXPECT_FALSE(pinned.torque_limit->HasPreviousTorque());  // a non-finite torque is ignored
  Eigen::VectorXd beyond = Eigen::VectorXd::Constant(na, 1e6);
  pinned.torque_limit->OnTorqueApplied(beyond);
  pinned.torque_limit->ComputeInequality(pinned.cache, pinned.contacts, pinned.info, n_vars, C, l,
                                         u);
  EXPECT_EQ(pinned.torque_limit->ClampedCount(), na);
  for (int i = 0; i < na; ++i) {
    ASSERT_EQ(l(i), u(i));
  }
}

// ── Failure modes · CommandOutput ─────────────────────────────────────────────

class Contradiction final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "contradiction"; }

  void Init(const pinocchio::Model&, const RobotModelInfo&, PinocchioCache&,
            const YAML::Node&) override {}

  [[nodiscard]] int EqDim(const ContactState&) const noexcept override { return 2; }

  [[nodiscard]] int IneqDim(const ContactState&) const noexcept override { return 0; }

  [[nodiscard]] int MaxEqDim(const ContactManagerConfig&) const noexcept override { return 2; }

  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig&) const noexcept override { return 0; }

  void ComputeEquality(const PinocchioCache&, const ContactState&, const RobotModelInfo&, int,
                       Eigen::Ref<Eigen::MatrixXd> A,
                       Eigen::Ref<Eigen::VectorXd> b) noexcept override {
    A(0, 6) = 1.0;
    b(0) = 0.0;
    A(1, 6) = 1.0;
    b(1) = 1.0;
  }

  void ComputeInequality(const PinocchioCache&, const ContactState&, const RobotModelInfo&, int,
                         Eigen::Ref<Eigen::MatrixXd>, Eigen::Ref<Eigen::VectorXd>,
                         Eigen::Ref<Eigen::VectorXd>) noexcept override {}
};

TEST_F(FloatingBounds, ANonValidTickKeepsTheLastValidOutputsAndReportsWhy) {
  const Sample& x = samples_[4];
  // One valid tick first, then each deterministic failure on a rig of its own.
  Rig rig(Options{}, x);
  const CommandOutput& ok = rig.Solve();
  ASSERT_TRUE(ok.tau_valid);
  const Eigen::VectorXd tau = ok.tau, a = ok.a_opt, lambda = ok.lambda_opt;
  EXPECT_EQ(ok.n_eq_used, 18);  // EoM base 6 + two 6-D contacts
  {
    auto& f = rig.ctrl.Formulation();
    int ineq = 0;
    for (const char* name : {"friction_cone", "torque_limit", "joint_limit"}) {
      ConstraintBase* c = f.GetConstraint(name);
      ASSERT_NE(c, nullptr) << name;
      ineq += c->IneqDim(rig.contacts);
    }
    EXPECT_EQ(ok.n_ineq_used, ineq);  // Σ IneqDim of the active constraints
  }
  EXPECT_GE(ok.outer_iterations, 1);
  EXPECT_EQ(ok.rows_dropped, 0);
  // (a) a contradictory equality → PRIMAL_INFEASIBLE (2)
  rig.ctrl.Formulation().AddConstraint(std::make_unique<Contradiction>());
  const CommandOutput& o1 = rig.Solve();
  EXPECT_FALSE(o1.tau_valid);
  EXPECT_FALSE(o1.qp_converged);
  EXPECT_EQ(o1.qp_status, 2);
  EXPECT_TRUE(SameBytes(o1.tau, tau));
  EXPECT_TRUE(SameBytes(o1.a_opt, a));
  EXPECT_TRUE(SameBytes(o1.lambda_opt, lambda));
  // (b) q_des NaN → non-finite iterates
  Rig nan(Options{}, x);
  const CommandOutput& n0 = nan.Solve();
  ASSERT_TRUE(n0.tau_valid);
  const Eigen::VectorXd tau_n = n0.tau;
  nan.ref.q_des(8) = kNaN;
  const CommandOutput& n1 = nan.Solve();
  EXPECT_FALSE(n1.tau_valid);
  EXPECT_TRUE(n1.qp_non_finite);
  EXPECT_TRUE(SameBytes(n1.tau, tau_n));
  // (c) a row too many → kRowBudgetExceeded (−2), nothing solved
  Rig budget(Options{}, x);
  const CommandOutput& b0 = budget.Solve();
  ASSERT_TRUE(b0.tau_valid);
  const Eigen::VectorXd tau_b = b0.tau;
  budget.ctrl.Formulation().AddConstraint(std::make_unique<OneRowTooMany>());
  const CommandOutput& b1 = budget.Solve();
  EXPECT_FALSE(b1.tau_valid);
  EXPECT_EQ(b1.qp_status, kRowBudgetExceeded);
  EXPECT_EQ(b1.rows_dropped, 1);
  EXPECT_TRUE(SameBytes(b1.tau, tau_b));
  // (d) max_iter 1 on a cold start → MAX_ITER_REACHED (1), the premise of the
  // registered max_iter-hit path
  Options one;
  one.max_iter = 1;
  Rig cold(one, x);
  const CommandOutput& c1 = cold.Solve();
  ASSERT_EQ(c1.qp_status, 1);
  EXPECT_FALSE(c1.tau_valid);
  EXPECT_EQ(c1.tau.lpNorm<Eigen::Infinity>(), 0.0);  // nothing valid yet: zero
  // The five SolveResult fields reach CommandOutput through the formulation.
  const SolveResult& sr = rig.ctrl.Formulation().Solve(rig.cache, rig.ref, rig.contacts, rig.info);
  const CommandOutput& again = rig.Solve();
  EXPECT_EQ(again.n_eq_used, sr.n_eq_used);
  EXPECT_EQ(again.n_ineq_used, sr.n_ineq_used);
  EXPECT_EQ(again.rows_dropped, sr.rows_dropped);
  EXPECT_EQ(again.outer_iterations, sr.outer_iterations);
  EXPECT_EQ(again.qp_status, sr.status);
}

// A solve that converges but whose torque recovery overflows (the solve does
// not read h; τ = S(M a + h − Jᵀλ) does): reported as a non-finite tick, the
// outputs stay the last valid ones.
TEST_F(FloatingBounds, AConvergedSolveWithANonFiniteTorqueIsNotValid) {
  Options o;  // no EoM, no torque limit: the QP never reads h, the recovery does
  o.eom = false;
  o.torque_limit = false;
  Rig rig(o, samples_[9]);
  const CommandOutput& ok = rig.Solve();
  ASSERT_TRUE(ok.tau_valid);
  const Eigen::VectorXd tau = ok.tau;
  rig.cache.h(7) = kInf;  // the recovery overflows, the solve does not see it
  const CommandOutput& bad = rig.Solve();
  EXPECT_FALSE(bad.tau_valid);
  EXPECT_FALSE(bad.qp_converged);
  EXPECT_TRUE(bad.qp_non_finite);
  EXPECT_TRUE(SameBytes(bad.tau, tau));
}

// ── Solver keys ───────────────────────────────────────────────────────────────

TEST_F(FloatingBounds, SolverKeysReachTheConfigAndBadOnesAreRefused) {
  const QPSolverConfig d{};
  YAML::Node n;
  n["max_iter"] = 33;
  n["max_iter_in"] = 77;
  n["eps_abs"] = 2e-7;
  n["eps_rel"] = 1e-3;
  n["verbose"] = true;
  n["update_preconditioner"] = true;
  n["eps_primal_inf"] = 5e-5;
  for (const char* backend : {"Automatic", "PrimalDualLDLT", "PrimalLDLT"}) {
    n["dense_backend"] = backend;
    const QPSolverConfig c = ParseQPSolverConfig(n, d);
    EXPECT_EQ(c.max_iter, 33);
    EXPECT_EQ(c.max_iter_in, 77);
    EXPECT_EQ(c.eps_abs, 2e-7);
    EXPECT_EQ(c.eps_rel, 1e-3);
    EXPECT_TRUE(c.verbose);
    EXPECT_TRUE(c.update_preconditioner);
    EXPECT_EQ(c.eps_primal_inf, 5e-5);
    const auto expect =
        std::string(backend) == "Automatic"        ? proxsuite::proxqp::DenseBackend::Automatic
        : std::string(backend) == "PrimalDualLDLT" ? proxsuite::proxqp::DenseBackend::PrimalDualLDLT
                                                   : proxsuite::proxqp::DenseBackend::PrimalLDLT;
    EXPECT_EQ(c.dense_backend, expect);
  }
  // Missing keys keep the defaults handed in.
  const QPSolverConfig kept = ParseQPSolverConfig(YAML::Load("{}"), d);
  EXPECT_EQ(kept.max_iter, d.max_iter);
  EXPECT_EQ(kept.dense_backend, d.dense_backend);
  EXPECT_EQ(ParseQPSolverConfig(YAML::Node(), d).eps_primal_inf, d.eps_primal_inf);
  YAML::Node bad;
  bad["dense_backend"] = "Fast";
  EXPECT_THROW((void)ParseQPSolverConfig(bad, d), std::invalid_argument);
  YAML::Node neg;
  neg["eps_primal_inf"] = -1e-4;
  EXPECT_THROW((void)ParseQPSolverConfig(neg, d), std::invalid_argument);
  EXPECT_THROW((void)ParseQPSolverConfig(YAML::Load("200"), d), std::invalid_argument);
  YAML::Node typo;
  typo["eps_abs"] = "1e-6x";
  EXPECT_THROW((void)ParseQPSolverConfig(typo, d), std::exception);  // does not convert
  YAML::Node zero_iter;
  zero_iter["max_iter"] = 0;
  EXPECT_THROW((void)ParseQPSolverConfig(zero_iter, d), std::invalid_argument);
  YAML::Node nan_eps;
  nan_eps["eps_abs"] = YAML::Load(".nan");
  EXPECT_THROW((void)ParseQPSolverConfig(nan_eps, d), std::invalid_argument);
  // Through the formulations, with their own defaults.
  const Sample& x = samples_[0];
  YAML::Node wq = ControllerYaml(Options{});
  wq["wqp"]["solver"]["dense_backend"] = "PrimalDualLDLT";
  wq["wqp"]["solver"]["update_preconditioner"] = true;
  WQPFormulation wqp;
  RobotModelInfo info;
  info.Build(*SharedG1Model(), YAML::Node{});
  ContactManagerConfig mgr;
  mgr.Load(wq, *SharedG1Model());
  wqp.Init(*SharedG1Model(), info, mgr, wq);
  EXPECT_EQ(wqp.SolverConfig().dense_backend, proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
  EXPECT_TRUE(wqp.SolverConfig().update_preconditioner);
  EXPECT_EQ(wqp.SolverConfig().max_iter, 200);
  EXPECT_EQ(wqp.SolverConfig().max_iter_in, 500);
  YAML::Node hq = ControllerYaml(Options{});
  hq["formulation_type"] = "hqp";
  hq["hqp"]["solver_per_level"]["eps_primal_inf"] = 0.0;
  HQPFormulation hqp;
  hqp.Init(*SharedG1Model(), info, mgr, hq);
  EXPECT_EQ(hqp.SolverConfig().max_iter, 10);  // the in-map HQP fallback, not the WQP's
  EXPECT_EQ(hqp.SolverConfig().eps_primal_inf, 0.0);
  YAML::Node no_map = ControllerYaml(Options{});
  no_map["formulation_type"] = "hqp";
  HQPFormulation bare;
  bare.Init(*SharedG1Model(), info, mgr, no_map);
  EXPECT_EQ(bare.SolverConfig().max_iter, QPSolverConfig{}.max_iter);  // 20, as before
  hq["hqp"]["solver_per_level"]["dense_backend"] = "Sparse";
  HQPFormulation refused;
  EXPECT_THROW(refused.Init(*SharedG1Model(), info, mgr, hq), std::invalid_argument);
  // The member order rtc_controllers' designated initialisers rely on.
  const QPSolverConfig ordered{.eps_abs = 1e-6,
                               .eps_rel = 0.0,
                               .max_iter = 20,
                               .max_iter_in = 100,
                               .verbose = false,
                               .update_preconditioner = false,
                               .dense_backend = proxsuite::proxqp::DenseBackend::Automatic,
                               .eps_primal_inf = 1e-4};
  EXPECT_EQ(ordered.max_iter_in, 100);
  (void)x;
}

// The HQP: the torque is the last level's solution; a converged stack is valid.
TEST_F(FloatingBounds, TheHqpTorqueIsValidWhenItsLevelsConverge) {
  Options o;
  o.formulation = "hqp";
  Rig rig(o, samples_[6]);
  const CommandOutput& out = rig.Solve();
  ASSERT_TRUE(out.qp_converged);
  EXPECT_TRUE(out.tau_valid);
  EXPECT_EQ(out.solve_levels, 1);
  EXPECT_EQ(out.n_eq_used, 18);
  rig.ctrl.Reset();  // propagates: no throw, the next tick is valid again
  EXPECT_TRUE(rig.Solve().tau_valid);
}

}  // namespace
}  // namespace rtc::tsid
