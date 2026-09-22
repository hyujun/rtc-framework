// dynamic_catching S3.5b — the gate batch behind the gate-catchable map.
//
// What is pinned here is the PLUMBING, not the gate mathematics (that is
// test_catching_time_feasibility.cpp): that each column is the runtime
// function's own double, that a verdict does not depend on what ran before it,
// that every constant reaches the gate it belongs to, and that an unusable
// input is an error or a fail-closed row — never a plausible map.
#include "rtc_controllers/catching/catch_gate_batch.hpp"
#include "rtc_controllers/catching/time_feasibility.hpp"
#include "rtc_controllers/catching/time_types.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace rc = rtc::catching;

constexpr int kNv = 3;

// Every per-joint value differs, and no constant equals another, so a swapped
// index or a constant wired to the wrong gate cannot cancel out.
rc::GateSettings Settings() {
  rc::GateSettings s;
  s.qdot_max = {2.0, 3.0, 5.0};
  s.qddot_max = {4.0, 9.0, 25.0};
  s.eta_v = 0.8;
  s.v_max = 3.5;
  s.d_eff = 0.09;
  s.t_close_total = 0.12;
  s.gamma_margin = 0.1;
  s.a_dec = 6.0;
  s.first_plan_s = 0.24;
  s.t_arm_s = 0.05;
  s.t_margin_s = 0.03;
  return s;
}

Eigen::VectorXd Wait() {
  Eigen::VectorXd q(kNv);
  q << 0.1, -0.4, 0.7;
  return q;
}

// A candidate every gate passes under Settings(): short moves, a slow ball,
// and a q̇ᵘ that reaches unit speed exactly along v̂.
rc::GateCandidate Passing() {
  rc::GateCandidate c;
  c.id = 7;
  c.t_c_s = 0.9;
  c.p_c = Eigen::Vector3d(0.4, -0.2, 0.8);
  c.v_ball = Eigen::Vector3d(1.2, 0.0, -0.9);  // 1.5 m/s
  c.q_star = Wait() + Eigen::Vector3d(0.10, -0.15, 0.20);
  c.qdot_u = Eigen::Vector3d(0.5, -0.8, 1.0);
  c.jp_qdot_u = c.v_ball / c.v_ball.norm();
  return c;
}

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    out.push_back(line.substr(start, comma == std::string::npos ? comma : comma - start));
    if (comma == std::string::npos) {
      return out;
    }
    start = comma + 1;
  }
}

std::string Cell(const rc::GateRow& row, const std::string& column) {
  const auto header = Split(rc::GateCsvHeader());
  const auto cells = Split(rc::GateCsvRow(row));
  EXPECT_EQ(header.size(), cells.size());
  const auto it = std::find(header.begin(), header.end(), column);
  EXPECT_NE(it, header.end()) << column;
  if (it == header.end() || header.size() != cells.size()) {
    return {};
  }
  return cells.at(static_cast<std::size_t>(std::distance(header.begin(), it)));
}

bool BitEqual(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

std::string CandidateCsv(const std::vector<rc::GateCandidate>& cs) {
  std::ostringstream os;
  os.precision(17);
  // Deliberately NOT the order the parser lists them in.
  os << "qu2,qs0,v_z,id,p_c_y,jpu_x,t_c_s,qs2,qu0,p_c_x,v_x,qs1,jpu_z,p_c_z,qu1,v_y,jpu_y,seed_"
        "id\n";
  for (const auto& c : cs) {
    os << c.qdot_u(2) << ',' << c.q_star(0) << ',' << c.v_ball(2) << ',' << c.id << ',' << c.p_c(1)
       << ',' << c.jp_qdot_u(0) << ',' << c.t_c_s << ',' << c.q_star(2) << ',' << c.qdot_u(0) << ','
       << c.p_c(0) << ',' << c.v_ball(0) << ',' << c.q_star(1) << ',' << c.jp_qdot_u(2) << ','
       << c.p_c(2) << ',' << c.qdot_u(1) << ',' << c.v_ball(1) << ',' << c.jp_qdot_u(1) << ','
       << c.seed_id << '\n';
  }
  return os.str();
}

}  // namespace

TEST(CatchGateBatch, PassingCandidatePassesEveryGate) {
  const rc::GateRow row = rc::JudgeGates(Passing(), Wait(), Settings());
  EXPECT_EQ(row.reason, rc::GateReason::kNone);
  EXPECT_TRUE(row.reach_ok);
  EXPECT_TRUE(row.gamma_ok);
  EXPECT_TRUE(row.stop_gamma_min.valid);
  EXPECT_LT(row.window.g_min, row.window.g_max);
}

TEST(CatchGateBatch, VerdictsAreTheRuntimeFunctionsOwnValues) {
  const rc::GateSettings s = Settings();
  const rc::GateCandidate c = Passing();
  const Eigen::VectorXd wait = Wait();
  const rc::GateRow row = rc::JudgeGates(c, wait, s);

  std::vector<double> w_plan = s.qdot_max;
  for (double& w : w_plan) {
    w *= s.eta_v;
  }
  const std::vector<double> w0(kNv, 0.0);
  const rc::TMinResult reach =
      rc::MaxJointTMin({wait.data(), kNv}, w0, {c.q_star.data(), kNv}, w_plan, s.qddot_max);
  EXPECT_TRUE(BitEqual(row.reach.t, reach.t));
  // lead = t_c − (first_plan + T_arm); the margin is the gate's, not the lead's.
  EXPECT_NEAR(row.lead_s, c.t_c_s - s.first_plan_s - s.t_arm_s, 1e-9);

  const double speed = c.v_ball.norm();
  const rc::DirectionalSpeed dir =
      rc::DirectionalSpeedMax(c.v_ball / speed, c.jp_qdot_u, {c.qdot_u.data(), kNv}, w_plan);
  EXPECT_TRUE(BitEqual(row.direction.v_dir_max, dir.v_dir_max));
  // q̇ᵘ·(w_plan)⁻¹ peaks on joint 1: 0.8 / (0.8·3.0). A swapped limit index
  // would pick 0.5/(0.8·2.0) or 1.0/(0.8·5.0) instead.
  EXPECT_NEAR(dir.v_dir_max, 1.0 / (0.8 / 2.4), 1e-12);

  const double v_tcp = rc::PlanningTcpSpeed(s.eta_v, s.v_max);
  const rc::GammaWindow win =
      rc::ComputeGammaWindow(speed, dir.v_dir_max, v_tcp, s.d_eff, s.t_close_total);
  EXPECT_TRUE(BitEqual(row.window.g_min, win.g_min));
  EXPECT_TRUE(BitEqual(row.window.g_max, win.g_max));
  EXPECT_TRUE(BitEqual(row.v_tcp_plan, v_tcp));
  EXPECT_TRUE(BitEqual(row.max_catchable,
                       rc::MaxCatchableSpeed(dir.v_dir_max, v_tcp, s.d_eff, s.t_close_total)));

  const rc::StoppingReservation stop = rc::StoppingPoint(c.p_c, c.v_ball, win.g_min, s.a_dec);
  EXPECT_TRUE(BitEqual(row.stop_gamma_min.distance, stop.distance));
  EXPECT_TRUE(row.stop_gamma_min.p_stop.isApprox(stop.p_stop, 0.0));
  EXPECT_GT(row.stop_gamma_max.distance, row.stop_gamma_min.distance);
}

TEST(CatchGateBatch, CsvColumnsRoundTripTheDoublesExactly) {
  const rc::GateRow row = rc::JudgeGates(Passing(), Wait(), Settings());
  EXPECT_TRUE(BitEqual(std::stod(Cell(row, "t_min_s")), row.reach.t));
  EXPECT_TRUE(BitEqual(std::stod(Cell(row, "v_dir_max")), row.direction.v_dir_max));
  EXPECT_TRUE(BitEqual(std::stod(Cell(row, "g_min")), row.window.g_min));
  EXPECT_TRUE(BitEqual(std::stod(Cell(row, "g_max")), row.window.g_max));
  EXPECT_TRUE(BitEqual(std::stod(Cell(row, "stop_gmin_z")), row.stop_gamma_min.p_stop.z()));
  EXPECT_EQ(Cell(row, "accepted"), "1");
  EXPECT_EQ(Cell(row, "reason_name"), "none");
  EXPECT_EQ(Cell(row, "id"), "7");
}

TEST(CatchGateBatch, ReachGateStopsALateCandidateAndOnlyThatGate) {
  rc::GateCandidate c = Passing();
  c.q_star(0) = Wait()(0) + 2.5;  // joint 0 is the slowest: ω 1.6, a 4
  const rc::GateRow row = rc::JudgeGates(c, Wait(), Settings());
  EXPECT_EQ(row.reason, rc::GateReason::kReachTime);
  EXPECT_FALSE(row.reach_ok);
  EXPECT_TRUE(row.gamma_ok) << "each gate reports its own verdict, not the first failure's";
  // trapezoid: D/ω + ω/a with the PLANNING speed η_v·q̇_max = 1.6
  EXPECT_NEAR(row.reach.t, 2.5 / 1.6 + 1.6 / 4.0, 1e-12);
}

TEST(CatchGateBatch, TimeMarginIsPartOfTheReachGate) {
  rc::GateSettings s = Settings();
  rc::GateCandidate c = Passing();
  const double t_min = rc::JudgeGates(c, Wait(), s).reach.t;
  c.t_c_s = s.first_plan_s + s.t_arm_s + t_min + 0.5 * s.t_margin_s;
  EXPECT_EQ(rc::JudgeGates(c, Wait(), s).reason, rc::GateReason::kReachTime);
  s.t_margin_s = 0.0;
  EXPECT_NE(rc::JudgeGates(c, Wait(), s).reason, rc::GateReason::kReachTime);
}

TEST(CatchGateBatch, GammaGateStopsAFastBallAndHonoursTheSpeedMargin) {
  const rc::GateSettings s = Settings();
  rc::GateCandidate c = Passing();
  const rc::GateRow slow = rc::JudgeGates(c, Wait(), s);
  // Put the ball just inside the margin below max_catchable: the window itself
  // is still open, the margin is what turns it away.
  const double speed = slow.max_catchable - 0.5 * s.gamma_margin;
  c.v_ball *= speed / c.v_ball.norm();
  const rc::GateRow row = rc::JudgeGates(c, Wait(), s);
  EXPECT_TRUE(row.window.Feasible());
  EXPECT_FALSE(row.gamma_ok);
  EXPECT_EQ(row.reason, rc::GateReason::kGammaWindowEmpty);
}

TEST(CatchGateBatch, EtaVAppliesToTheJointLimitsAsWellAsTheTcpLimit) {
  rc::GateSettings s = Settings();
  const rc::GateRow a = rc::JudgeGates(Passing(), Wait(), s);
  s.eta_v = 0.4;
  const rc::GateRow b = rc::JudgeGates(Passing(), Wait(), s);
  EXPECT_NEAR(b.direction.v_dir_max, 0.5 * a.direction.v_dir_max, 1e-12);
  EXPECT_NEAR(b.v_tcp_plan, 0.5 * a.v_tcp_plan, 1e-12);
  EXPECT_GT(b.reach.t, a.reach.t - 1e-15);
}

TEST(CatchGateBatch, UnusableInputsFailClosedNotOpen) {
  const rc::GateSettings s = Settings();
  rc::GateCandidate still = Passing();
  still.v_ball.setZero();
  EXPECT_EQ(rc::JudgeGates(still, Wait(), s).reason, rc::GateReason::kGammaInvalid);

  rc::GateCandidate nan_pose = Passing();
  nan_pose.q_star(1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(rc::JudgeGates(nan_pose, Wait(), s).reason, rc::GateReason::kReachInvalid);

  rc::GateSettings bad = s;
  bad.qddot_max[2] = 0.0;  // JudgeGates itself must not read this as "instant"
  EXPECT_EQ(rc::JudgeGates(Passing(), Wait(), bad).reason, rc::GateReason::kReachInvalid);
}

TEST(CatchGateBatch, ReasonIsTheFirstFailureInPlannerOrder) {
  // Late AND without a usable γ input: the planner drops it at the reach gate,
  // and the histogram must say so rather than credit the later gate.
  rc::GateCandidate c = Passing();
  c.q_star(0) = Wait()(0) + 2.5;
  c.jp_qdot_u(0) = std::numeric_limits<double>::quiet_NaN();
  const rc::GateRow row = rc::JudgeGates(c, Wait(), Settings());
  EXPECT_TRUE(row.direction.input_invalid);
  EXPECT_FALSE(row.gamma_ok);
  EXPECT_EQ(row.reason, rc::GateReason::kReachTime);

  rc::GateCandidate on_time = Passing();
  on_time.jp_qdot_u(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(rc::JudgeGates(on_time, Wait(), Settings()).reason, rc::GateReason::kGammaInvalid);
}

TEST(CatchGateBatch, SettingsAreValidatedUpFront) {
  const std::map<int, Eigen::VectorXd> seeds{{0, Wait()}};
  const std::vector<rc::GateCandidate> cs{Passing()};
  EXPECT_NO_THROW((void)rc::RunGateBatch(cs, seeds, Settings()));

  const auto rejects = [&](auto mutate) {
    rc::GateSettings s = Settings();
    mutate(s);
    EXPECT_FALSE(rc::ValidateGateSettings(s, kNv).empty());
    EXPECT_THROW((void)rc::RunGateBatch(cs, seeds, s), std::invalid_argument);
  };
  rejects([](rc::GateSettings& s) { s.qdot_max.pop_back(); });
  rejects([](rc::GateSettings& s) { s.qddot_max[1] = -1.0; });
  rejects([](rc::GateSettings& s) { s.eta_v = 1.2; });
  rejects([](rc::GateSettings& s) { s.eta_v = 0.0; });
  rejects([](rc::GateSettings& s) { s.v_max = 0.0; });
  rejects([](rc::GateSettings& s) { s.t_close_total = 0.0; });
  rejects([](rc::GateSettings& s) { s.a_dec = std::numeric_limits<double>::infinity(); });
  rejects([](rc::GateSettings& s) { s.d_eff = -0.01; });
  rejects([](rc::GateSettings& s) { s.first_plan_s = std::nan(""); });
}

TEST(CatchGateBatch, MissingSeedAndWrongWidthAreErrors) {
  std::map<int, Eigen::VectorXd> seeds{{0, Wait()}};
  rc::GateCandidate other_seed = Passing();
  other_seed.seed_id = 3;
  EXPECT_THROW((void)rc::RunGateBatch({other_seed}, seeds, Settings()), std::invalid_argument);

  rc::GateCandidate narrow = Passing();
  narrow.q_star.conservativeResize(2);
  EXPECT_THROW((void)rc::RunGateBatch({narrow}, seeds, Settings()), std::invalid_argument);
  EXPECT_THROW((void)rc::RunGateBatch({Passing()}, {}, Settings()), std::invalid_argument);
}

TEST(CatchGateBatch, WaitPoseIsTakenFromTheCandidatesOwnSeed) {
  Eigen::VectorXd far = Wait();
  far(0) += 3.0;
  const std::map<int, Eigen::VectorXd> seeds{{0, Wait()}, {1, far}};
  rc::GateCandidate near_c = Passing();
  rc::GateCandidate far_c = Passing();
  far_c.id = 8;
  far_c.seed_id = 1;
  const auto rows = rc::RunGateBatch({near_c, far_c}, seeds, Settings());
  EXPECT_EQ(rows.at(0).reason, rc::GateReason::kNone);
  EXPECT_EQ(rows.at(1).reason, rc::GateReason::kReachTime);
}

TEST(CatchGateBatch, VerdictDoesNotDependOnBatchOrder) {
  rc::GateCandidate late = Passing();
  late.id = 1;
  late.q_star(0) += 2.5;
  rc::GateCandidate fast = Passing();
  fast.id = 2;
  fast.v_ball *= 4.0;
  const std::vector<rc::GateCandidate> forward{Passing(), late, fast};
  const std::vector<rc::GateCandidate> backward{fast, late, Passing()};
  const std::map<int, Eigen::VectorXd> seeds{{0, Wait()}};
  const auto a = rc::RunGateBatch(forward, seeds, Settings());
  const auto b = rc::RunGateBatch(backward, seeds, Settings());
  ASSERT_EQ(a.size(), 3U);
  ASSERT_EQ(b.size(), 3U);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(rc::GateCsvRow(a[i]), rc::GateCsvRow(b[2 - i]));
  }
  EXPECT_EQ(a[2].reason, rc::GateReason::kGammaWindowEmpty);
}

TEST(CatchGateBatch, CandidateCsvTakesColumnOrderFromTheHeader) {
  rc::GateCandidate c = Passing();
  c.seed_id = 4;
  std::istringstream in(CandidateCsv({c}));
  const auto parsed = rc::ParseGateCandidateCsv(in, kNv);
  ASSERT_EQ(parsed.size(), 1U);
  const rc::GateCandidate& p = parsed.front();
  EXPECT_EQ(p.id, c.id);
  EXPECT_EQ(p.seed_id, 4);
  EXPECT_TRUE(BitEqual(p.t_c_s, c.t_c_s));
  EXPECT_TRUE(p.p_c.isApprox(c.p_c, 0.0));
  EXPECT_TRUE(p.v_ball.isApprox(c.v_ball, 0.0));
  EXPECT_TRUE(p.jp_qdot_u.isApprox(c.jp_qdot_u, 0.0));
  EXPECT_TRUE(p.q_star.isApprox(c.q_star, 0.0));
  EXPECT_TRUE(p.qdot_u.isApprox(c.qdot_u, 0.0));
  // q* and q̇ᵘ are distinct vectors in the fixture, so a qs/qu mix-up shows.
  EXPECT_FALSE(p.q_star.isApprox(p.qdot_u, 1e-3));
}

TEST(CatchGateBatch, CandidateCsvRejectsWhatItCannotRead) {
  const auto parse = [](const std::string& text, int nv) {
    std::istringstream in(text);
    return rc::ParseGateCandidateCsv(in, nv);
  };
  const std::string good = CandidateCsv({Passing()});
  EXPECT_NO_THROW((void)parse(good, kNv));
  EXPECT_THROW((void)parse("", kNv), std::invalid_argument);
  EXPECT_THROW((void)parse(good, kNv + 1), std::invalid_argument) << "a posture column is missing";
  EXPECT_THROW((void)parse(good, kNv - 1), std::invalid_argument) << "an extra posture column";
  std::string nan_cell = good;
  nan_cell.replace(nan_cell.rfind("0.9"), 3, "nan");
  EXPECT_THROW((void)parse(nan_cell, kNv), std::invalid_argument);
  EXPECT_THROW((void)parse(good + "1,2,3\n", kNv), std::invalid_argument) << "ragged row";
}
