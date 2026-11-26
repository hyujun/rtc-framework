// The QP that WQPFormulation assembles through the shortcuts
// (TaskBase::AccumulateCost for the selection tasks, the row copies of
// TorqueLimitConstraint when S = [0 | I]) against the dense assembly they
// replace — ComputeResidual + w·JᵀJ / w·Jᵀr, and the three S·X products —
// rebuilt here. Two implementations compared byte for byte, not values pinned:
// a change to a task's cost changes both sides.
//
// The comparison is of bytes where the compiler does not fuse a multiply into
// an add (x86-64 without FMA — the shipped build); elsewhere `w * (d * d)`
// added to H may round once where the product kernel rounds twice, so there
// the mismatches are recorded, not asserted.

#include "loco_gait.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

#if defined(__x86_64__) && !defined(__FMA__)
constexpr bool kBitsComparable = true;
#else
constexpr bool kBitsComparable = false;
#endif

// Entries whose bytes differ (a −0 against a +0 counts, a NaN against the same
// NaN does not).
template <typename A, typename B>
int DifferentBits(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return -1;
  }
  int n = 0;
  for (Eigen::Index j = 0; j < a.cols(); ++j) {
    for (Eigen::Index i = 0; i < a.rows(); ++i) {
      const double x = a(i, j);
      const double y = b(i, j);
      n += std::memcmp(&x, &y, sizeof(double)) != 0 ? 1 : 0;
    }
  }
  return n;
}

// Where bytes are not comparable the two sides still agree to rounding; the
// entries whose bytes differ are counted for the record.
int g_recorded_mismatches = 0;

template <typename A, typename B>
void ExpectSameBits(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b,
                    const std::string& what) {
  const int n = DifferentBits(a, b);
  ASSERT_GE(n, 0) << what << ": shapes differ";
  if (kBitsComparable) {
    EXPECT_EQ(n, 0) << what;
    return;
  }
  g_recorded_mismatches += n;
  const double scale = std::max(1.0, a.cwiseAbs().maxCoeff());
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), 1e-12 * scale) << what;
}

// ── The dense assembly (what Solve did for every task) ──────────────────────

struct DenseCost {
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
};

// `rdims`: the residual dimension Solve read from each task — before it called
// into the task.
DenseCost DenseCostOf(const std::vector<std::unique_ptr<TaskBase>>& tasks,
                      const std::vector<int>& rdims, const PinocchioCache& cache,
                      const ControlReference& ref, const ContactState& contacts, int n_vars) {
  DenseCost out{Eigen::MatrixXd::Zero(n_vars, n_vars), Eigen::VectorXd::Zero(n_vars)};
  int max_rdim = n_vars;
  for (const int d : rdims) {
    max_rdim = std::max(max_rdim, d);
  }
  Eigen::MatrixXd J_workspace = Eigen::MatrixXd::Zero(max_rdim, n_vars);
  Eigen::VectorXd r_workspace = Eigen::VectorXd::Zero(max_rdim);
  auto H = out.H.topLeftCorner(n_vars, n_vars);
  auto g_vec = out.g.head(n_vars);
  for (std::size_t k = 0; k < tasks.size(); ++k) {
    TaskBase& task = *tasks[k];
    if (!task.IsActive()) {
      continue;
    }
    const int rdim = rdims[k];
    auto J_view = J_workspace.topLeftCorner(rdim, n_vars);
    auto r_view = r_workspace.head(rdim);
    J_view.setZero();
    r_view.setZero();
    task.ComputeResidual(cache, ref, contacts, n_vars, J_view, r_view);
    const double w = task.Weight();
    H.noalias() += w * J_view.transpose() * J_view;
    g_vec.noalias() -= w * J_view.transpose() * r_view;
  }
  for (int i = 0; i < n_vars; ++i) {
    out.H(i, i) += 1e-8;
  }
  return out;
}

std::vector<int> ResidualDims(const std::vector<std::unique_ptr<TaskBase>>& tasks) {
  std::vector<int> out;
  out.reserve(tasks.size());
  for (const auto& t : tasks) {
    out.push_back(t->ResidualDim());
  }
  return out;
}

struct DenseTorque {
  Eigen::MatrixXd C;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
};

// The torque-limit rows through the products (no rate bound).
DenseTorque DenseTorqueOf(const RobotModelInfo& info, const PinocchioCache& cache,
                          const ContactManagerConfig& manager, const ContactState& contacts,
                          double tau_scale, int n_vars) {
  const int na = info.n_actuated;
  const int nv = info.nv;
  DenseTorque out{Eigen::MatrixXd::Zero(na, n_vars), Eigen::VectorXd(na), Eigen::VectorXd(na)};
  Eigen::MatrixXd SM(na, nv);
  SM.noalias() = info.S * cache.M;
  out.C.leftCols(nv) = SM;
  int lambda_offset = 0;
  for (std::size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager.contacts[i].contact_dim;
    if (contacts.contacts[i].active && i < cache.contact_frames.size()) {
      const auto& Jc = cache.contact_frames[i].J;
      out.C.block(0, nv + lambda_offset, na, cdim).noalias() =
          -info.S * Jc.topRows(cdim).transpose();
    }
    lambda_offset += cdim;
  }
  Eigen::VectorXd Sh(na);
  Sh.noalias() = info.S * cache.h;
  out.l = tau_scale * info.tau_min - Sh;
  out.u = tau_scale * info.tau_max - Sh;
  return out;
}

// ── On the locomotion rig ───────────────────────────────────────────────────

WQPFormulation& Wqp(test::LocoRig& rig) {
  return dynamic_cast<WQPFormulation&>(rig.b.Controller().Formulation());
}

YAML::Node ShippedPolicyYaml() {
  YAML::Node n = YAML::Load(test::kLocoWbq);
  n["solver"]["warm_start"] = "previous_result_reset_mu";
  n["constraints"]["contact"]["inactive_lambda"] = "zero";
  return n;
}

// The QP of the rig's last Solve against the dense assembly of the same tick.
// `rdims` empty: the dimensions as they are now (the builder brings the force
// task's up to date before Solve, so they are the ones Solve read).
void ExpectAssembledAsDense(test::LocoRig& rig, const std::string& what,
                            std::vector<int> rdims = {}) {
  WQPFormulation& wqp = Wqp(rig);
  const QPData& qp = wqp.LastQpForTesting();
  const int n_vars = qp.n_vars;
  if (rdims.empty()) {
    rdims = ResidualDims(wqp.TasksForTesting());
  }
  const DenseCost cost =
      DenseCostOf(wqp.TasksForTesting(), rdims, rig.cache, rig.ref, rig.contacts, n_vars);
  ExpectSameBits(qp.H, cost.H, what + ": H");
  ExpectSameBits(qp.g, cost.g, what + ": g");

  const auto [row, dim] = wqp.IneqRowsForTesting("torque_limit");
  ASSERT_EQ(dim, rig.info.n_actuated) << what;
  const DenseTorque tq =
      DenseTorqueOf(rig.info, rig.cache, rig.b.Contacts(), rig.contacts, 1.0, n_vars);
  ExpectSameBits(qp.C.middleRows(row, dim), tq.C, what + ": C (torque rows)");
  ExpectSameBits(qp.l.segment(row, dim), tq.l, what + ": l (torque rows)");
  ExpectSameBits(qp.u.segment(row, dim), tq.u, what + ": u (torque rows)");
}

// Two cycles of the walking schedule — double support, either foot released,
// every support change — under the default and the shipped solver policy,
// without and with state noise.
TEST(QpAssemblyTest, TheGaitTraceIsAssembledAsTheDenseProductsWould) {
  g_recorded_mismatches = 0;
  int ticks = 0;
  int single_support = 0;
  int transitions = 0;
  for (const bool shipped : {false, true}) {
    for (const double noise : {0.0, 1e-3}) {
      test::LocoRig rig(shipped ? ShippedPolicyYaml() : YAML::Load(test::kLocoWbq));
      test::LocoGait gait(*rig.model, noise);
      for (int t = 0; t < 2 * test::LocoGait::kCycle; ++t) {
        gait.Apply(t, rig);
        rig.Solve();
        ExpectAssembledAsDense(rig, "tick " + std::to_string(t));
        ++ticks;
        single_support += rig.contacts.active_count == 1 ? 1 : 0;
        transitions += test::LocoGait::IsTransition(t) ? 1 : 0;
        if (::testing::Test::HasFailure()) {
          return;  // one tick's report is enough
        }
      }
    }
  }
  EXPECT_EQ(ticks, 4 * 2 * test::LocoGait::kCycle);
  EXPECT_GT(single_support, 0);
  EXPECT_GT(transitions, 0);
  RecordProperty("bits_comparable", kBitsComparable ? "1" : "0");
  RecordProperty("recorded_mismatches", std::to_string(g_recorded_mismatches));
}

// A contact mid-ramp scales its force rows by √activation: the entry is no
// longer 1, so the order of the two multiplications shows.
TEST(QpAssemblyTest, APartlyActiveContactIsAssembledAsDense) {
  test::LocoRig rig;
  rig.Set(test::QStand(*rig.model), Eigen::VectorXd::Zero(rig.info.nv));
  rig.WeightOn({0, 1});
  int differing = 0;
  for (const double s : {0.3, 0.77, 1.0 / 3.0, 0.123456789}) {
    rig.contacts.contacts[0].activation = s;
    rig.contacts.contacts[1].activation = 1.0 - s;
    rig.Solve();
    ExpectAssembledAsDense(rig, "activation " + std::to_string(s));
    // The case has to tell the two orders apart for some entry.
    const double d = std::sqrt(s);
    const double w = Wqp(rig).GetTask("force")->Weight();
    differing += (w * (d * d) != (w * d) * d) ? 1 : 0;
  }
  EXPECT_GT(differing, 0) << "no activation here separates w·(d·d) from (w·d)·d";
}

// Weights and activity changed between ticks reach the shortcut as they reach
// the products.
TEST(QpAssemblyTest, WeightAndActivityChangesAreAssembledAsDense) {
  test::LocoRig rig;
  rig.Set(test::QStand(*rig.model), Eigen::VectorXd::Zero(rig.info.nv));
  rig.WeightOn({0, 1});
  WQPFormulation& wqp = Wqp(rig);
  TaskBase* posture = wqp.GetTask("posture");
  TaskBase* force = wqp.GetTask("force");
  ASSERT_NE(posture, nullptr);
  ASSERT_NE(force, nullptr);

  rig.Solve();
  ExpectAssembledAsDense(rig, "as built");
  posture->SetWeight(3.7);
  force->SetWeight(0.013);
  rig.Solve();
  ExpectAssembledAsDense(rig, "new weights");
  force->SetActive(false);
  rig.Solve();
  ExpectAssembledAsDense(rig, "force off");
  const int nv = rig.info.nv;
  EXPECT_EQ(wqp.LastQpForTesting().H(nv + 2, nv + 2), 1e-8) << "an inactive task added cost";
  force->SetActive(true);
  posture->SetActive(false);
  rig.Release(1);
  rig.WeightOn({0});
  rig.Solve();
  ExpectAssembledAsDense(rig, "posture off, one foot");
}

// Point contacts: three λ per slot, so the column offsets differ from the
// surface rig's.
TEST(QpAssemblyTest, PointContactsAreAssembledAsDense) {
  const LocomotionModelSpec spec{{{"foot_a", "left_sole", 3}, {"foot_b", "right_sole", 3}},
                                 "torso_link"};
  test::LocoRig rig(YAML::Load(test::kLocoWbq), spec);
  rig.Set(test::QStand(*rig.model), Eigen::VectorXd::Zero(rig.info.nv));
  rig.ref.lambda_des.setZero();
  rig.ref.lambda_des(2) = 170.0;
  rig.ref.lambda_des(5) = 171.0;
  rig.Solve();
  ExpectAssembledAsDense(rig, "two point contacts");
  rig.Release(0);
  rig.Solve();
  ExpectAssembledAsDense(rig, "slot 0 released");
  const int nv = rig.info.nv;
  const QPData& qp = Wqp(rig).LastQpForTesting();
  EXPECT_EQ(qp.H(nv + 2, nv + 2), 1e-8) << "the released slot's λ carries force cost";
  EXPECT_GT(qp.H(nv + 5, nv + 5), 1e-8) << "the standing slot's λ lost its force cost";
}

// Solve reads the force task's residual dimension before the task updates it.
// An owner that does not bring it up to date first (the builder does) gets the
// rows the stale dimension leaves — the shortcut keeps exactly those.
TEST(QpAssemblyTest, AStaleForceDimensionKeepsTheRowsTheDenseViewKept) {
  test::LocoRig rig;
  rig.Set(test::QStand(*rig.model), Eigen::VectorXd::Zero(rig.info.nv));
  rig.Release(1);
  rig.WeightOn({0});
  rig.Solve();  // the force task now holds 6

  // Both feet again, straight through the formulation: 12 active λ, 6 rows.
  auto& c = rig.contacts.contacts[1];
  c.active = true;
  c.activation = 1.0;
  c.activation_target = 1.0;
  rig.contacts.RecomputeActive(rig.b.Contacts());
  rig.WeightOn({0, 1});
  WQPFormulation& wqp = Wqp(rig);
  const std::vector<int> rdims = ResidualDims(wqp.TasksForTesting());
  ASSERT_EQ(wqp.GetTask("force")->ResidualDim(), 6);
  ASSERT_EQ(rig.contacts.active_contact_vars, 12);
  (void)wqp.Solve(rig.cache, rig.ref, rig.contacts, rig.info);
  ExpectAssembledAsDense(rig, "stale dimension", rdims);
  const int nv = rig.info.nv;
  const QPData& qp = wqp.LastQpForTesting();
  EXPECT_GT(qp.H(nv + 5, nv + 5), 1e-8);
  EXPECT_EQ(qp.H(nv + 6, nv + 6), 1e-8) << "a row past the dimension Solve read was added";
}

// A force reference shorter than a contact's slice: the dense path leaves that
// contact's residual at the caller's 0 (the rows still cost w·λ²).
TEST(QpAssemblyTest, AShortForceReferenceLeavesAZeroResidual) {
  test::LocoRig rig;
  rig.Set(test::QStand(*rig.model), Eigen::VectorXd::Zero(rig.info.nv));
  rig.WeightOn({0, 1});
  const double share = rig.ref.lambda_des(2);
  ASSERT_GT(share, 100.0);
  rig.ref.lambda_des.conservativeResize(9);  // slot 0 whole, slot 1 cut
  rig.Solve();
  ExpectAssembledAsDense(rig, "short reference");
  const int nv = rig.info.nv;
  const QPData& qp = Wqp(rig).LastQpForTesting();
  EXPECT_NE(qp.g(nv + 2), 0.0);
  EXPECT_EQ(qp.g(nv + 8), 0.0) << "slot 1 read a reference it does not have";
}

// A non-finite reference, activation or weight: the dense products spread it over all
// of g (0 × NaN) and the solver's iterates went non-finite — the tick was
// refused. Left in one λ column, ProxQP returns its previous (finite) iterate
// as solved. Solve therefore spreads it again; the next finite tick solves.
TEST(QpAssemblyTest, ANonFiniteForceTermRefusesTheTickAndTheNextOneSolves) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const int which : {0, 1, 2, 3}) {
    test::LocoRig rig;
    rig.Set(test::QStand(*rig.model), Eigen::VectorXd::Zero(rig.info.nv));
    rig.WeightOn({0, 1});
    ASSERT_TRUE(rig.Solve().tau_valid);

    TaskBase* weighted = Wqp(rig).GetTask(which == 2 ? "posture" : "force");
    const double weight = weighted->Weight();
    if (which == 0) {
      rig.ref.lambda_des(2) = nan;
    } else if (which == 1) {
      rig.contacts.contacts[0].activation = nan;
    } else {
      weighted->SetWeight(nan);
    }
    const CommandOutput& bad = rig.Solve();
    EXPECT_FALSE(bad.tau_valid) << "case " << which;
    EXPECT_TRUE(bad.qp_non_finite) << "case " << which;
    // As the dense products left it: the whole of g, not one column.
    EXPECT_TRUE(Wqp(rig).LastQpForTesting().g.array().isNaN().all()) << "case " << which;
    EXPECT_TRUE(bad.tau.allFinite()) << "case " << which;

    rig.WeightOn({0, 1});
    rig.contacts.contacts[0].activation = 1.0;
    weighted->SetWeight(weight);
    const CommandOutput& good = rig.Solve();
    EXPECT_TRUE(good.tau_valid) << "case " << which;
    EXPECT_TRUE(good.qp_converged) << "case " << which;
  }
}

// ── Tasks and the torque limit on a fixed base ──────────────────────────────

class FixedBaseAssemblyTest : public test::PandaNoContactTest {};

// The posture task alone, S = I: both paths through a formulation of one task
// and the torque limit.
TEST_F(FixedBaseAssemblyTest, AFixedBaseIsAssembledAsDense) {
  g_recorded_mismatches = 0;
  WQPFormulation wqp;
  YAML::Node cfg;
  cfg["formulation_type"] = "wqp";
  wqp.Init(*model_, robot_info_, contact_cfg_, cfg);
  auto posture = std::make_unique<PostureTask>();
  YAML::Node pcfg;
  pcfg["kp"] = 100.0;
  pcfg["kd"] = 20.0;
  pcfg["weight"] = 0.37;
  posture->Init(*model_, robot_info_, cache_, pcfg);
  wqp.AddTask(std::move(posture));
  auto tl = std::make_unique<TorqueLimitConstraint>();
  tl->Init(*model_, robot_info_, cache_, YAML::Node{});
  wqp.AddConstraint(std::move(tl));

  const int nv = robot_info_.nv;
  for (int k = 0; k < 20; ++k) {
    const Eigen::VectorXd q = 0.5 * (robot_info_.q_lower + robot_info_.q_upper) +
                              0.1 * Eigen::VectorXd::LinSpaced(nv, -1.0, 1.0) * std::sin(0.7 * k);
    const Eigen::VectorXd v = 0.3 * Eigen::VectorXd::LinSpaced(nv, 1.0, -1.0) * std::cos(0.4 * k);
    cache_.Update(q, v);
    ref_.q_des = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
    ref_.v_des.setZero();
    ref_.a_des.setConstant(0.01 * k);
    (void)wqp.Solve(cache_, ref_, contacts_, robot_info_);

    const QPData& qp = wqp.LastQpForTesting();
    const DenseCost cost = DenseCostOf(wqp.TasksForTesting(), ResidualDims(wqp.TasksForTesting()),
                                       cache_, ref_, contacts_, nv);
    ExpectSameBits(qp.H, cost.H, "H");
    ExpectSameBits(qp.g, cost.g, "g");
    const DenseTorque tq = DenseTorqueOf(robot_info_, cache_, contact_cfg_, contacts_, 1.0, nv);
    const auto [row, dim] = wqp.IneqRowsForTesting("torque_limit");
    ASSERT_EQ(dim, robot_info_.n_actuated);
    ExpectSameBits(qp.C.middleRows(row, dim), tq.C, "C");
    ExpectSameBits(qp.l.segment(row, dim), tq.l, "l");
    ExpectSameBits(qp.u.segment(row, dim), tq.u, "u");
  }
  RecordProperty("recorded_mismatches", std::to_string(g_recorded_mismatches));
}

// The torque limit with a contact on a model this small: Eigen evaluates
// −S·Jcᵀ coefficient-wise there, which leaves −0 where the row copy leaves +0.
// The rows are the same numbers.
TEST_F(FixedBaseAssemblyTest, ASmallModelsContactColumnsAreTheSameNumbers) {
  ContactManagerConfig mgr;
  ContactConfig cc;
  cc.name = "tip";
  cc.frame_name = "panda_link8";
  ASSERT_TRUE(model_->existFrame(cc.frame_name));
  cc.frame_id = static_cast<int>(model_->getFrameId(cc.frame_name));
  cc.contact_dim = 3;
  mgr.contacts.push_back(cc);
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;
  PinocchioCache cache;
  cache.Init(model_, ContactFrameIds(mgr));
  ContactState contacts;
  contacts.Init(1);
  contacts.contacts[0].active = true;
  contacts.RecomputeActive(mgr);

  TorqueLimitConstraint tl;
  YAML::Node cfg;
  cfg["tau_scale"] = 0.8;
  tl.Init(*model_, robot_info_, cache, cfg);
  tl.SetContactManager(&mgr);

  const int nv = robot_info_.nv;
  const int na = robot_info_.n_actuated;
  const int n_vars = nv + 3;
  const Eigen::VectorXd q = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
  cache.Update(q, 0.2 * Eigen::VectorXd::Ones(nv));
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(na, n_vars);
  Eigen::VectorXd l(na);
  Eigen::VectorXd u(na);
  tl.ComputeInequality(cache, contacts, robot_info_, n_vars, C, l, u);
  const DenseTorque tq = DenseTorqueOf(robot_info_, cache, mgr, contacts, 0.8, n_vars);
  EXPECT_TRUE((C.array() == tq.C.array()).all());
  EXPECT_GT(C.rightCols(3).cwiseAbs().maxCoeff(), 0.0);
  ExpectSameBits(C.leftCols(nv), tq.C.leftCols(nv), "S·M");
  ExpectSameBits(l, tq.l, "l");
  ExpectSameBits(u, tq.u, "u");
}

// The products accumulate from +0, so a −0 in M or h comes out as +0 (and a
// −0 effort limit then keeps its sign in l). The row copies do the same.
TEST_F(FixedBaseAssemblyTest, NegativeZerosComeOutAsTheProductsLeaveThem) {
  RobotModelInfo info = robot_info_;
  info.tau_min(2) = -0.0;
  TorqueLimitConstraint tl;
  tl.Init(*model_, info, cache_, YAML::Node{});
  const int nv = info.nv;
  const int na = info.n_actuated;
  cache_.Update(0.5 * (info.q_lower + info.q_upper), 0.2 * Eigen::VectorXd::Ones(nv));
  cache_.M(1, 3) = -0.0;
  cache_.M(3, 1) = -0.0;
  cache_.h(2) = -0.0;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(na, nv);
  Eigen::VectorXd l(na);
  Eigen::VectorXd u(na);
  tl.ComputeInequality(cache_, contacts_, info, nv, C, l, u);
  const DenseTorque tq = DenseTorqueOf(info, cache_, contact_cfg_, contacts_, 1.0, nv);
  ExpectSameBits(C, tq.C, "C");
  ExpectSameBits(l, tq.l, "l");
  ExpectSameBits(u, tq.u, "u");
  EXPECT_FALSE(std::signbit(C(1, 3)));
  EXPECT_TRUE(std::signbit(l(2))) << "−0 − (+0) is −0";
}

// An S that is not [0 | I] (two actuators swapped) takes the products.
TEST_F(FixedBaseAssemblyTest, AnotherSelectionMatrixTakesTheProducts) {
  RobotModelInfo info = robot_info_;
  info.S.row(1).swap(info.S.row(4));
  TorqueLimitConstraint tl;
  tl.Init(*model_, info, cache_, YAML::Node{});

  const int nv = info.nv;
  const int na = info.n_actuated;
  cache_.Update(0.5 * (info.q_lower + info.q_upper), 0.2 * Eigen::VectorXd::Ones(nv));
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(na, nv);
  Eigen::VectorXd l(na);
  Eigen::VectorXd u(na);
  tl.ComputeInequality(cache_, contacts_, info, nv, C, l, u);
  const DenseTorque tq = DenseTorqueOf(info, cache_, contact_cfg_, contacts_, 1.0, nv);
  EXPECT_EQ(DifferentBits(C, tq.C), 0);
  EXPECT_EQ(DifferentBits(l, tq.l), 0);
  EXPECT_EQ(DifferentBits(u, tq.u), 0);
  // And it is not the row copy: row 1 is M's row 4.
  EXPECT_EQ(DifferentBits(C.row(1), cache_.M.row(4)), 0);
  EXPECT_NE(DifferentBits(C.row(1), cache_.M.row(1)), 0);
}

}  // namespace
}  // namespace rtc::tsid
