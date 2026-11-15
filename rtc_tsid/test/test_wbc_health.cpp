// The solve margins (WbcMargins, from ConstraintBase::MinSlack over the rows
// the QP assembled) and the fail-safe classification (SolveHealth) of the
// locomotion whole-body QP:
//
//   * every margin is the smallest slack of its kind of row, recomputed here
//     from the assembled rows and the published solution;
//   * a margin goes to ~0 where its rows bind (a reduced torque envelope, a
//     tight acceleration box, a force demand outside the friction pyramid) and
//     stays positive standing still; at the pyramid's corner the row slack is
//     ~0 while the cone the pyramid approximates is violated — the margin is
//     the QP's own rows;
//   * SolveHealth over deterministic tick sequences, and through the builder:
//     a refused reference and a contradictory QP are bad ticks, Fault is
//     reached and left as configured, the torque is untouched.

#include "loco_gait.hpp"
#include "loco_wbc_fixture.hpp"
#include "rtc_tsid/controller/solve_health.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

using test::LocoRig;

constexpr double kEpsAbs = 1e-6;  // the rig's solver.eps_abs
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

const WQPFormulation& Wqp(LocoRig& r) {
  return dynamic_cast<const WQPFormulation&>(r.b.Controller().Formulation());
}

// The margins recomputed from the last assembled QP and the published x: the
// rows in the builder's constraint order (friction cone, joint limit, torque
// limit), each friction block per active contact 4 faces + 1 unilateral, then
// (surface contacts) 4 CoP and 2 yaw rows. Also the rounding bound the comparison is held to.
struct Oracle {
  WbcMargins m;
  double tol{0.0};
};

Oracle Recompute(LocoRig& r, const Eigen::VectorXd& x) {
  const QPData& qp = Wqp(r).LastQpForTesting();
  auto& f = r.b.Controller().Formulation();
  const int n_fric = f.GetConstraint("friction_cone")->IneqDim(r.contacts);
  const int n_joint = f.GetConstraint("joint_limit")->IneqDim(r.contacts);
  const int n_torque = f.GetConstraint("torque_limit")->IneqDim(r.contacts);
  EXPECT_EQ(n_fric + n_joint + n_torque, qp.n_ineq) << "premise: the row layout";
  Oracle o;
  o.m.ClearSlack();
  double bound = 0.0;
  double c_norm = 0.0;
  const auto slack = [&](int i) {
    const double cx = qp.C.row(i).dot(x);
    for (const double b : {qp.l(i), qp.u(i)}) {
      if (std::abs(b) < 1e9) {
        bound = std::max(bound, std::abs(b));
      }
    }
    c_norm = std::max(c_norm, qp.C.row(i).cwiseAbs().sum());
    return std::min(qp.u(i) - cx, cx - qp.l(i));
  };
  int row = 0;
  for (std::size_t c = 0; c < r.contacts.contacts.size(); ++c) {
    if (!r.contacts.contacts[c].active) {
      continue;
    }
    for (int k = 0; k < 5; ++k) {
      o.m.friction_n = std::min(o.m.friction_n, slack(row++));
    }
    if (r.b.Contacts().contacts[c].contact_dim != 6) {
      continue;  // a point contact: faces and the unilateral row only
    }
    for (int k = 0; k < 4; ++k) {
      o.m.cop_nm = std::min(o.m.cop_nm, slack(row++));
    }
    for (int k = 0; k < 2; ++k) {
      o.m.yaw_nm = std::min(o.m.yaw_nm, slack(row++));
    }
  }
  EXPECT_EQ(row, n_fric);
  for (int k = 0; k < n_joint; ++k) {
    o.m.joint_accel_box = std::min(o.m.joint_accel_box, slack(row++));
  }
  for (int k = 0; k < n_torque; ++k) {
    o.m.torque_nm = std::min(o.m.torque_nm, slack(row++));
  }
  const double eps = std::numeric_limits<double>::epsilon();
  o.tol =
      64.0 * static_cast<double>(x.size()) * eps * (c_norm * x.lpNorm<Eigen::Infinity>() + bound);
  return o;
}

void ExpectMarginsMatch(LocoRig& r, const CommandOutput& out, const char* where) {
  SCOPED_TRACE(where);
  ASSERT_TRUE(out.tau_valid);
  const WbcMargins& m = r.b.Margins();
  EXPECT_FALSE(m.stale);
  const Oracle o = Recompute(r, Wqp(r).SolverForTesting().Primal());
  EXPECT_LE(std::abs(m.friction_n - o.m.friction_n), o.tol);
  EXPECT_LE(std::abs(m.cop_nm - o.m.cop_nm), o.tol);
  EXPECT_LE(std::abs(m.yaw_nm - o.m.yaw_nm), o.tol);
  EXPECT_LE(std::abs(m.torque_nm - o.m.torque_nm), o.tol);
  EXPECT_LE(std::abs(m.joint_accel_box - o.m.joint_accel_box), o.tol);
  EXPECT_EQ(std::memcmp(&m.eom_residual_inf, &out.eom_residual_inf, sizeof(double)), 0);
}

// ── Margins ───────────────────────────────────────────────────────────────────

TEST(WbcMargins, StandingStillTheMarginsArePositiveAndAreTheRowsSlack) {
  LocoRig r;
  EXPECT_TRUE(r.b.Margins().stale) << "no tick yet";
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.WeightOn({0, 1});
  const CommandOutput& out = r.Solve();
  ExpectMarginsMatch(r, out, "standing");
  const WbcMargins& m = r.b.Margins();
  for (const double v : {m.friction_n, m.cop_nm, m.torque_nm, m.joint_accel_box}) {
    EXPECT_GT(v, 0.0);
    EXPECT_TRUE(std::isfinite(v));
  }
  // torsional_friction_coeff 0: the two yaw rows hold m_z = 0, so their slack
  // is 0 to the solver tolerance on every tick.
  ASSERT_EQ(r.b.Contacts().contacts[0].torsional_friction_coeff, 0.0);
  EXPECT_LE(std::abs(m.yaw_nm), 10.0 * kEpsAbs);
  EXPECT_EQ(r.b.Health().State(), SolveHealthState::kNominal);
}

// Over a gait window with state noise (each support set, both transitions)
// every tick's margins are its rows' slack; a released foot's rows are gone.
TEST(WbcMargins, OverAGaitEveryTicksMarginsAreItsRowsSlack) {
  YAML::Node n = YAML::Load(test::kLocoWbq);
  n["solver"]["warm_start"] = "previous_result_reset_mu";
  n["constraints"]["contact"]["inactive_lambda"] = "zero";
  LocoRig r(n);
  test::LocoGait gait(*r.model, 1e-3);
  int checked = 0;
  for (int t = test::LocoGait::kSlot1Off - 20; t < test::LocoGait::kSlot1Off + 20; ++t) {
    gait.Apply(t, r);
    const CommandOutput& out = r.Solve();
    ExpectMarginsMatch(r, out, ("tick " + std::to_string(t)).c_str());
    ++checked;
  }
  EXPECT_EQ(checked, 40);
}

// Mixed contacts (a point foot and a surface foot): with the point foot
// released, the surface foot's rows are read as its own — a released contact
// takes no rows, so the next one's kinds stay aligned.
TEST(WbcMargins, WithMixedContactsAReleasedContactTakesNoRows) {
  LocomotionModelSpec spec = test::LocoSpec();
  spec.contacts[0].dim = 3;
  LocoRig r(YAML::Load(test::kLocoWbq), spec);
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.ref.lambda_des.setZero();
  r.ref.lambda_des(3 + 2) = 300.0;
  ExpectMarginsMatch(r, r.Solve(), "both");
  r.Release(0);
  ExpectMarginsMatch(r, r.Solve(), "point foot released");
  EXPECT_TRUE(std::isfinite(r.b.Margins().cop_nm)) << "premise: the surface foot's rows";
}

// All contacts released: the friction cone writes no rows, and its MinSlack
// on an empty block leaves every contact margin at +inf.
TEST(WbcMargins, WithEveryContactReleasedTheContactMarginsAreNone) {
  LocoRig r;
  r.Release(0);
  r.Release(1);
  const auto* cone = r.b.Controller().Formulation().GetConstraint("friction_cone");
  ASSERT_EQ(cone->IneqDim(r.contacts), 0);
  WbcMargins m;
  const auto n_vars = Wqp(r).LastQpForTesting().C.cols();
  const Eigen::MatrixXd C(0, n_vars);
  const Eigen::VectorXd b(0);
  const Eigen::VectorXd x = Eigen::VectorXd::Zero(n_vars);
  cone->MinSlack(r.contacts, C, b, b, x, m);
  EXPECT_EQ(m.friction_n, kInf);
  EXPECT_EQ(m.cop_nm, kInf);
  EXPECT_EQ(m.yaw_nm, kInf);
}

// Binding rows: a torque envelope too small to hold the robot pins the
// torque margin at ~0; a small acceleration box against a far posture target
// pins the joint margin.
TEST(WbcMargins, ABindingTorqueEnvelopeOrAccelerationBoxPinsItsMarginAtZero) {
  {
    YAML::Node n = YAML::Load(test::kLocoWbq);
    n["constraints"]["torque_limit"]["tau_scale"] = 0.05;
    LocoRig r(n);
    r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
    r.WeightOn({0, 1});
    const CommandOutput& out = r.Solve();
    ExpectMarginsMatch(r, out, "torque");
    EXPECT_LE(std::abs(r.b.Margins().torque_nm), 10.0 * kEpsAbs);
  }
  {
    YAML::Node n = YAML::Load(test::kLocoWbq);
    n["constraints"]["joint_limit"]["acceleration_max"] = 0.5;
    LocoRig r(n);
    const Eigen::VectorXd q = test::QStand(*r.model);
    r.Set(q, Eigen::VectorXd::Zero(r.model->nv));
    r.WeightOn({0, 1});
    r.ref.q_des(7 + 15) += 0.5;  // an arm joint far from its target
    const CommandOutput& out = r.Solve();
    ExpectMarginsMatch(r, out, "joint");
    EXPECT_LE(std::abs(r.b.Margins().joint_accel_box), 10.0 * kEpsAbs);
  }
}

// A force demand outside the pyramid in both tangential directions puts the
// solution on a corner: two face rows bind (row slack ~0) while the friction
// cone μ f_n − ‖f_t‖ the pyramid circumscribes is violated by far more than
// the solver tolerance — a margin taken from the cone would report a violation
// of the rows the QP actually enforced.
TEST(WbcMargins, AtThePyramidsCornerTheFrictionMarginIsTheRowsNotTheCones) {
  YAML::Node n = YAML::Load(test::kLocoWbq);
  n["tasks"]["force"]["weight"] = 1e2;
  LocoRig r(n);
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.WeightOn({0, 1});
  for (const int c : {0, 1}) {
    r.ref.lambda_des(6 * c + 0) = 400.0;
    r.ref.lambda_des(6 * c + 1) = 400.0;
  }
  const CommandOutput& out = r.Solve();
  ExpectMarginsMatch(r, out, "corner");
  const double mu = r.b.Contacts().contacts[0].friction_coeff;
  double cone = kInf;
  for (const int c : {0, 1}) {
    const Eigen::Vector3d f = out.lambda_opt.segment<3>(6 * c);
    cone = std::min(cone, mu * f.z() - f.head<2>().norm());
  }
  EXPECT_GE(r.b.Margins().friction_n, -10.0 * kEpsAbs);
  EXPECT_LE(r.b.Margins().friction_n, 10.0 * kEpsAbs) << "premise: a face row binds";
  EXPECT_LT(cone, -1.0) << "premise: the corner is outside the cone";
}

// The HQP gathers the same kinds from its shared rows.
TEST(WbcMargins, TheHqpReportsTheSameKindsOfMargin) {
  YAML::Node n = YAML::Load(test::kLocoWbq);
  n["formulation_type"] = "hqp";
  LocoRig r(n);
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.WeightOn({0, 1});
  ASSERT_TRUE(r.Solve().tau_valid);
  const WbcMargins& m = r.b.Margins();
  EXPECT_FALSE(m.stale);
  for (const double v : {m.friction_n, m.cop_nm, m.torque_nm, m.joint_accel_box}) {
    EXPECT_GT(v, 0.0);
    EXPECT_TRUE(std::isfinite(v));
  }
  EXPECT_LE(std::abs(m.yaw_nm), 10.0 * kEpsAbs);  // m_z = 0 rows (μ_τ 0)
}

// ── SolveHealth, deterministic sequences ──────────────────────────────────────

WbcMargins Good() {
  WbcMargins m;
  m.ClearSlack();
  m.friction_n = m.cop_nm = m.yaw_nm = m.torque_nm = m.joint_accel_box = 1.0;
  m.eom_residual_inf = 1e-9;
  m.stale = false;
  return m;
}

TEST(SolveHealth, TheDefaultsDegradeOnABadTickNeverFaultAndRecoverOnAGoodOne) {
  SolveHealth h;
  EXPECT_EQ(h.State(), SolveHealthState::kNominal);
  EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kNominal);
  for (int k = 0; k < 1000; ++k) {
    EXPECT_EQ(h.Update(false, Good()), SolveHealthState::kDegraded);
  }
  EXPECT_EQ(h.BadRun(), 1000);
  EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kNominal);
  EXPECT_EQ(h.BadRun(), 0);
}

TEST(SolveHealth, MoreBadTicksThanAllowedFaultAndOnlyARunOfGoodOnesRecovers) {
  SolveHealthConfig c;
  c.max_invalid_ticks = 3;
  c.recover_valid_ticks = 5;
  SolveHealth h;
  h.Configure(c);
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(h.Update(false, Good()), SolveHealthState::kDegraded) << k;
  }
  EXPECT_EQ(h.Update(false, Good()), SolveHealthState::kFault);
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kFault) << k;
  }
  EXPECT_EQ(h.Update(false, Good()), SolveHealthState::kFault) << "a bad tick restarts the run";
  EXPECT_EQ(h.GoodRun(), 0);
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kFault) << k;
  }
  EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kNominal);
  // Degraded also needs the run of good ticks.
  EXPECT_EQ(h.Update(false, Good()), SolveHealthState::kDegraded);
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kDegraded) << k;
  }
  EXPECT_EQ(h.Update(true, Good()), SolveHealthState::kNominal);
  // max_invalid_ticks 0: the first bad tick is a Fault.
  c.max_invalid_ticks = 0;
  h.Configure(c);
  EXPECT_EQ(h.Update(false, Good()), SolveHealthState::kFault);
  h.Reset();
  EXPECT_EQ(h.State(), SolveHealthState::kNominal);
  EXPECT_EQ(h.BadRun(), 0);
  EXPECT_EQ(h.GoodRun(), 0);
}

// Margin floors apply to valid ticks only; off, they still refuse NaN.
TEST(SolveHealth, AMarginBelowItsFloorIsABadTickAndANanMarginAlwaysIs) {
  SolveHealth defaults;
  WbcMargins m = Good();
  m.torque_nm = kNan;
  EXPECT_EQ(defaults.Update(true, m), SolveHealthState::kDegraded) << "off still refuses NaN";
  m = Good();
  m.friction_n = kInf;  // no friction rows
  EXPECT_EQ(defaults.Update(true, m), SolveHealthState::kNominal);
  m.friction_n = -1e3;  // violated, but no floor set
  EXPECT_EQ(defaults.Update(true, m), SolveHealthState::kNominal);

  SolveHealthConfig c;
  c.min_friction_margin = 1.0;
  c.min_cop_margin = 0.1;
  c.min_yaw_margin = 0.1;
  c.min_torque_margin = 2.0;
  c.min_joint_accel_margin = 0.5;
  c.max_eom_residual = 1e-6;
  for (int field = 0; field < 6; ++field) {
    SolveHealth h;
    h.Configure(c);
    WbcMargins ok = Good();
    ok.friction_n = 1.5;
    ok.torque_nm = 2.5;
    EXPECT_EQ(h.Update(true, ok), SolveHealthState::kNominal) << field;
    WbcMargins low = ok;
    double* f[] = {&low.friction_n, &low.cop_nm,          &low.yaw_nm,
                   &low.torque_nm,  &low.joint_accel_box, &low.eom_residual_inf};
    *f[field] = field == 5 ? 2e-6 : 0.05;
    EXPECT_EQ(h.Update(true, low), SolveHealthState::kDegraded) << field;
    // An invalid tick is bad whatever its (stale) margins say.
    h.Reset();
    EXPECT_EQ(h.Update(false, ok), SolveHealthState::kDegraded) << field;
  }
}

// ── Through the builder ───────────────────────────────────────────────────────

// An equality pair no x satisfies: every solve fails (status 1 or 2).
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

YAML::Node WithHealth(const char* health) {
  YAML::Node n = YAML::Load(test::kLocoWbq);
  n["health"] = YAML::Load(health);
  return n;
}

bool SameBytes(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

// A contradictory QP fails every tick: Fault after max_invalid_ticks + 1, the
// torque and the margins those of the last valid tick (margins marked stale),
// and the builder keeps ticking.
TEST(SolveHealthBuilder, AContradictoryQpReachesFaultAndTheTorqueIsUntouched) {
  LocoRig r(WithHealth("{max_invalid_ticks: 2, recover_valid_ticks: 3}"));
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.WeightOn({0, 1});
  const CommandOutput& ok = r.Solve();
  ASSERT_TRUE(ok.tau_valid);
  const Eigen::VectorXd tau = ok.tau;
  const WbcMargins last = r.b.Margins();
  r.b.Controller().Formulation().AddConstraint(std::make_unique<Contradiction>());
  for (int k = 0; k < 10; ++k) {
    const CommandOutput& o = r.Solve();
    EXPECT_FALSE(o.tau_valid);
    EXPECT_NE(o.qp_status, 0);
    EXPECT_TRUE(SameBytes(o.tau, tau)) << k;
    EXPECT_EQ(r.b.Health().State(), k < 2 ? SolveHealthState::kDegraded : SolveHealthState::kFault)
        << k;
    EXPECT_TRUE(r.b.Margins().stale);
    EXPECT_EQ(r.b.Margins().torque_nm, last.torque_nm);
  }
  r.b.Reset();
  EXPECT_EQ(r.b.Health().State(), SolveHealthState::kNominal);
  EXPECT_TRUE(r.b.Margins().stale);
}

// A refused reference (SkipTick) is a bad tick too; good ticks recover.
TEST(SolveHealthBuilder, ARefusedReferenceIsABadTick) {
  LocoRig r(WithHealth("{max_invalid_ticks: 1}"));
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.WeightOn({0, 1});
  ASSERT_TRUE(r.Solve().tau_valid);
  const Eigen::VectorXd q_des = r.ref.q_des;
  r.ref.q_des.segment<4>(3).setZero();  // a zero quaternion: refused
  EXPECT_FALSE(r.Solve().tau_valid);
  EXPECT_EQ(r.b.Health().State(), SolveHealthState::kDegraded);
  EXPECT_FALSE(r.Solve().tau_valid);
  EXPECT_EQ(r.b.Health().State(), SolveHealthState::kFault);
  r.ref.q_des = q_des;
  EXPECT_TRUE(r.Solve().tau_valid);
  EXPECT_EQ(r.b.Health().State(), SolveHealthState::kNominal);
  EXPECT_FALSE(r.b.Margins().stale);
}

// A floor from the YAML acts on the builder's own margins.
TEST(SolveHealthBuilder, AMarginFloorFromTheYamlActsOnTheTicksMargins) {
  YAML::Node n = WithHealth("{min_torque_margin: 1.0}");
  n["constraints"]["torque_limit"]["tau_scale"] = 0.05;  // the torque rows bind
  LocoRig r(n);
  r.Set(test::QStand(*r.model), Eigen::VectorXd::Zero(r.model->nv));
  r.WeightOn({0, 1});
  ASSERT_TRUE(r.Solve().tau_valid);
  EXPECT_LT(r.b.Margins().torque_nm, 1.0);
  EXPECT_EQ(r.b.Health().State(), SolveHealthState::kDegraded);
}

TEST(SolveHealthBuilder, EveryHealthKeyReachesTheConfigAndBadOnesAreRefused) {
  LocoRig r(WithHealth(R"({max_invalid_ticks: 4, recover_valid_ticks: 7,
      min_friction_margin: 1.5, min_cop_margin: 0.25, min_yaw_margin: 0.125,
      min_torque_margin: 3.0, min_joint_accel_margin: -0.5, max_eom_residual: 1.0e-5})"));
  const SolveHealthConfig& c = r.b.Health().Config();
  EXPECT_EQ(c.max_invalid_ticks, 4);
  EXPECT_EQ(c.recover_valid_ticks, 7);
  EXPECT_EQ(c.min_friction_margin, 1.5);
  EXPECT_EQ(c.min_cop_margin, 0.25);
  EXPECT_EQ(c.min_yaw_margin, 0.125);
  EXPECT_EQ(c.min_torque_margin, 3.0);
  EXPECT_EQ(c.min_joint_accel_margin, -0.5);
  EXPECT_EQ(c.max_eom_residual, 1.0e-5);
  const SolveHealthConfig d = LocoRig{}.b.Health().Config();
  EXPECT_EQ(d.max_invalid_ticks, -1);
  EXPECT_EQ(d.recover_valid_ticks, 1);
  EXPECT_EQ(d.min_torque_margin, -kInf);
  EXPECT_EQ(d.max_eom_residual, kInf);
  for (const char* bad :
       {"{max_invalid_ticks: -1}", "{recover_valid_ticks: 0}", "{min_torque_margin: .inf}",
        "{min_cop_margin: .nan}", "{max_eom_residual: 0.0}", "{max_eom_residual: -1.0}",
        "{max_invalid_tick: 3}", "{max_invalid_ticks: three}", "[1, 2]", "3"}) {
    EXPECT_THROW(LocoRig{WithHealth(bad)}, std::invalid_argument) << bad;
  }
}

}  // namespace
}  // namespace rtc::tsid
