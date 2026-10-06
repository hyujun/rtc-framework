// NLP catch search (E1-F14 #740; reference §9.5, §11, §12.7, §17.11, §17.12).
//
// What is tested, and against what:
//   1. screening — the IK verdict against CatchPoseIk called directly; the
//      reach and closing-speed conditions against the reference's formulas
//      written out here on scalars (the pure functions' own boundaries are
//      test_catching_nlp_catch_screening.cpp's).
//   2. the choice — against an EXHAUSTIVE solve this file runs itself, on its
//      own cores: every candidate of the window, no screening, no budget.
//   3. order independence — the same wake with its solves permuted, compared
//      bit for bit.
//   4. validity — a solve past its share, a chance-row violation, an
//      unconverged iterate: none is chosen, each has its reason.
//   5. reasons — every value of NlpReject from an input that produces it.
//   6. the wakes after the RT follows a plan — the start state on the reported
//      segment, the re-solve of the followed candidate, the switch term's
//      anchor, the decision.
//   7. the lattice and the arm grid; what Configure refuses.
//   8. allocation — a C-level malloc gate over whole wakes, the QP solvers
//      bracketed out and the cores' own stages gated again inside them.
//
// THE THROW. A ball on a straight line down the capture axis of the wait pose:
// it is on that pose's entrance plane at one instant, and every other
// candidate's catch pose is the wait pose shifted along the axis. Candidates
// are therefore cheap or expensive by how far the arm must move — a spread of
// costs with one clear minimum, which is what a test of "the minimum is
// chosen" needs. The arm is a 6R with a wrist offset and its device order is a
// non-identity permutation of the model's, so a device/model mix-up moves the
// answer.
//
// THE CLOCK steps on every read. A wake's timing — how many solves fit, which
// solve passes its deadline — is then a function of how many times the clock
// was read, not of the host, and can be set by the step.
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/nlp_catch_screening.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_controllers/testing/planner_trace_digest.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using rtc::catching::BallNodeSample;
using rtc::catching::BallTime;
using rtc::catching::CatchPoseIk;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CatchPoseIkResult;
using rtc::catching::CatchPoseReason;
using rtc::catching::CatchSolution;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::DockingRowGroup;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::kNlpRejectCount;
using rtc::catching::Mode;
using rtc::catching::MpcDockingReason;
using rtc::catching::MpcDockingReasonName;
using rtc::catching::MpcDockingSegmentCore;
using rtc::catching::MpcDockingSegmentCoreInput;
using rtc::catching::MpcDockingSegmentCoreParams;
using rtc::catching::MpcDockingSegmentCoreResult;
using rtc::catching::MpcDockingStage;
using rtc::catching::NlpCatchSearch;
using rtc::catching::NlpCatchSearchConstants;
using rtc::catching::NlpCatchSearchModel;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::NlpPlanReason;
using rtc::catching::NlpReachLimit;
using rtc::catching::NlpReject;
using rtc::catching::NlpRejectName;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::NowReal;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanReason;
using rtc::catching::PlanSnapshot;
using rtc::catching::ReportedSegments;
using rtc::catching::SampleBallNode;
using rtc::catching::SearchStats;
using rtc::catching::SegmentSnapshot;
using rtc::catching::SwitchDecision;
using rtc::catching::TrajectorySnapshot;
using rtc::catching::ValidateSegmentNodes;
using rtc::testing::BitsEqual;
namespace fx = rtc::testing::mpc_segment_core;
namespace dk = rtc::testing::mpc_docking;
using Candidate = NlpCatchSearch::CandidateRecord;

static_assert(noexcept(std::declval<NlpCatchSearch&>().Plan(
    std::declval<const TrajectorySnapshot&>(), std::declval<const CovarianceSnapshot&>(), true,
    std::declval<const PlannerRtState&>(), std::declval<const ReportedSegments&>(), NowReal{},
    std::declval<SearchStats&>())));
static_assert(noexcept(std::declval<const NlpCatchSearch&>().Solution()));
static_assert(noexcept(std::declval<NlpCatchSearch&>().ResetTrial()));

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 10'000 * kMs;
constexpr std::uint64_t kActivation = 3;
constexpr std::uint64_t kTrack = 7;
constexpr double kInf = std::numeric_limits<double>::infinity();

// ── The clock ────────────────────────────────────────────────────────────────

std::atomic<std::int64_t> g_clock{0};
std::atomic<std::int64_t> g_step{1000};  // ns per read

std::int64_t StepClock() noexcept {
  const std::int64_t step = g_step.load(std::memory_order_relaxed);
  return g_clock.fetch_add(step, std::memory_order_relaxed) + step;
}

void SetClockStep(std::int64_t step_ns) {
  g_clock.store(0);
  g_step.store(step_ns);
}

[[nodiscard]] std::int64_t Ns(double s) {
  return static_cast<std::int64_t>(std::llround(s * 1e9));
}

[[nodiscard]] double Sec(std::int64_t ns) {
  return static_cast<double>(ns) / 1e9;
}

// ── The rig ──────────────────────────────────────────────────────────────────

// model joint m is the arm device's joint kDeviceOfModel[m].
constexpr std::array<int, 6> kDeviceOfModel{2, 0, 1, 5, 3, 4};

template <typename V>
[[nodiscard]] std::array<double, 6> ToDevice(const V& q_model) {
  std::array<double, 6> out{};
  for (std::size_t m = 0; m < 6; ++m) {
    out[static_cast<std::size_t>(kDeviceOfModel[m])] = q_model[static_cast<Eigen::Index>(m)];
  }
  return out;
}

struct Rig {
  rtc::testing::Arm arm = rtc::testing::Arm6R();
  dk::Rig dock = dk::MakeRig(fx::Synthetic6R());
  NlpCatchSearchModel model{};
  NlpCatchSearchConstants constants{};
  NlpCatchSearchParams params{};
  CatchPoseIkOptions ik{};
  NlpCatchSearch search;
  Eigen::VectorXd q_wait;  // model order

  Rig() {
    q_wait = dock.arm.q_nominal;
    model.arm = arm.model;
    model.handle = arm.handle.get();
    model.catch_frame = arm.frame;
    model.nv = arm.nv;
    for (std::size_t m = 0; m < 6; ++m) {
      model.device_of_model[m] = kDeviceOfModel[m];
    }
    constants.t_arm_s = 0.0;
    constants.control_dt = 0.002;
    constants.t_close_e2e = 0.1;

    // The lattice: one candidate every 40 ms. The arm grid: 50 ms intervals,
    // 2..8 before the catch — so two candidates in a row often share a core.
    params.cand_dt = 0.04;
    params.t_lead_min = 0.1;
    params.t_max = 0.4;
    params.cand_capacity = 16;
    params.catch_box.set = true;
    params.catch_box.min = {-5.0, -5.0, -5.0};
    params.catch_box.max = {5.0, 5.0, 5.0};
    params.wait_pose_n = arm.nv;
    const std::array<double, 6> wait_dev = ToDevice(q_wait);
    std::copy(wait_dev.begin(), wait_dev.end(), params.wait_pose.begin());
    params.n_pre_min = 2;
    params.n_pre_max = 8;
    params.dt_pre = 0.05;
    params.n_stop = 7;
    params.dt_stop = 0.05;
    params.n_stop_blocks = 4;
    params.stop_block_sizes = {1, 1, 2, 3};
    // With the 1 µs clock step below a wake spends microseconds: the budget
    // never cuts unless a test makes the step large.
    params.budget_s = 0.04;
    params.solve_budget_s = 0.004;
    params.start_lead_s = 0.004;
    params.max_solves = 8;
    params.w_time = 0.0;
    params.w_switch = 0.0;
    params.t_ref_s = 0.5;
    params.rank_w_q = 1.0;
    params.core = dock.params;
    params.limits = dock.limits;
    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
  }

  [[nodiscard]] bool Configure(std::string* error = nullptr) {
    SetClockStep(1000);
    return search.Configure(model, constants, params, ik, &StepClock, error);
  }

  // The arm at rest on its wait pose, following no plan.
  [[nodiscard]] PlannerRtState RestingRt(std::int64_t now) const {
    PlannerRtState rt = rtc::testing::TrackingRtState(kActivation, ToDevice(q_wait));
    rt.rt_iteration = 500 + static_cast<std::uint64_t>((now - kNow) / kMs);
    rt.rt_state_ns = now - kMs;
    return rt;
  }

  // The arm following plan `t_c_ns` (its command is not read by a solve).
  [[nodiscard]] PlannerRtState FollowingRt(std::int64_t now, std::int64_t t_c_ns) const {
    PlannerRtState rt = RestingRt(now);
    rt.mode = static_cast<std::uint8_t>(Mode::kApproach);
    rt.plan_active = true;
    rt.plan_id = 4;
    rt.plan_t_c_ns = t_c_ns;
    return rt;
  }

  // t_0 of a wake at `now`: when a segment published by it can start.
  [[nodiscard]] std::int64_t StartInstant(std::int64_t now) const {
    return now + Ns(constants.t_arm_s) + Ns(params.budget_s) + Ns(params.start_lead_s);
  }
};

// ── Throws ───────────────────────────────────────────────────────────────────

struct Throw {
  TrajectorySnapshot traj;
  CovarianceSnapshot cov;
  Eigen::Vector3d v_hat{Eigen::Vector3d::Zero()};
};

// The ball comes down the wait pose's capture axis at `speed` and is on its
// entrance plane `t_star_s` after kNow — `lateral` off the axis there, in the
// capture frame's x and y [m]. 20 samples, `spacing_ns` apart, from kNow.
[[nodiscard]] Throw AxisThrow(const Rig& rig, double t_star_s, double speed = 0.8,
                              double sigma = 0.002, std::uint64_t seq = 1,
                              std::uint64_t gen = kTrack,
                              const Eigen::Vector2d& lateral = Eigen::Vector2d::Zero(),
                              std::int64_t spacing_ns = 50 * kMs) {
  const dk::HandState h = dk::HandAt(rig.dock, rig.q_wait, Eigen::VectorXd::Zero(rig.arm.nv));
  const Eigen::Vector3d v_b = -speed * h.R.col(2);
  const Eigen::Vector3d p_e =
      h.p + h.R * Eigen::Vector3d(lateral.x(), lateral.y(), rig.params.core.s_ent);
  Throw t;
  t.traj = rtc::testing::LineTrajectory(p_e - v_b * t_star_s, v_b, kNow, spacing_ns, 20, 0, seq,
                                        gen, kActivation, kNow - 5 * kMs);
  t.cov = rtc::testing::IsotropicCovariance(t.traj, sigma);
  t.v_hat = v_b / speed;
  return t;
}

// The same line 4 cm and 3 cm off the capture axis: every candidate needs the
// arm to move sideways, so a solution is a real motion and a segment the RT
// follows has a velocity.
[[nodiscard]] Throw OffAxisThrow(const Rig& rig, double t_star_s, std::uint64_t seq = 1) {
  return AxisThrow(rig, t_star_s, 0.8, 0.002, seq, kTrack, Eigen::Vector2d(0.04, -0.03));
}

// A covariance that is wide ACROSS the ball's travel and narrow along it: the
// lateral chance rows see it, the timing row (σ along the approach axis) and
// the screening's closing-speed window do not.
[[nodiscard]] CovarianceSnapshot LateralCovariance(const Throw& t, double sigma_lateral,
                                                   double sigma_axial, double sigma_v) {
  CovarianceSnapshot c{};
  c.valid = true;
  c.n = t.traj.n;
  c.token = t.traj.token;
  const Eigen::Matrix3d along = t.v_hat * t.v_hat.transpose();
  const Eigen::Matrix3d sigma_p =
      sigma_lateral * sigma_lateral * (Eigen::Matrix3d::Identity() - along) +
      sigma_axial * sigma_axial * along;
  for (int k = 0; k < c.n; ++k) {
    auto& e = c.c[static_cast<std::size_t>(k)];
    e.fill(0.0);
    for (int r = 0; r < 3; ++r) {
      for (int q = 0; q < 3; ++q) {
        e[static_cast<std::size_t>(r * 6 + q)] = sigma_p(r, q);
      }
      e[static_cast<std::size_t>((r + 3) * 6 + r + 3)] = sigma_v * sigma_v;
    }
  }
  return c;
}

const ReportedSegments& NoSegments() {
  static const ReportedSegments none{};
  return none;
}

// ── Reading a wake ───────────────────────────────────────────────────────────

struct Wake {
  PlanSnapshot plan{};
  SearchStats stats{};
};

[[nodiscard]] Wake RunWake(Rig& rig, const Throw& t, const PlannerRtState& rt,
                           const ReportedSegments& arm, std::int64_t now, bool cov_matched = true) {
  Wake w;
  w.plan = rig.search.Plan(t.traj, t.cov, cov_matched, rt, arm, NowReal{now}, w.stats);
  return w;
}

[[nodiscard]] std::string Table(const NlpCatchSearch& s, const SearchStats& stats) {
  std::ostringstream os;
  os << "\nwake: reason " << NlpRejectName(stats.nlp.reason) << ", lattice " << stats.nlp.n_lattice
     << ", in window " << stats.n_in_window << ", screened " << stats.nlp.n_screened << ", solved "
     << stats.nlp.n_solved << ", valid " << stats.nlp.n_valid << "\n";
  for (const Candidate& c : s.Candidates()) {
    os << "  i " << c.index << " lead " << c.lead_s << " n_pre " << c.n_pre << " wait "
       << c.wait_ns / kMs << " ms: " << NlpRejectName(c.reject) << ", rank " << c.rank << ", core "
       << MpcDockingReasonName(c.core_reason) << ", it " << c.iterations << ", J " << c.j_reference
       << " + stop " << c.j_stop << ", phi " << c.phi << ", worst "
       << rtc::catching::DockingRowGroupName(c.worst_group) << " " << c.worst_violation
       << ", start " << static_cast<int>(c.start) << " from " << c.start_index << ", src "
       << c.source_seq << "\n";
  }
  return os.str();
}

[[nodiscard]] const Candidate* Find(const NlpCatchSearch& s, std::int64_t index) {
  for (const Candidate& c : s.Candidates()) {
    if (c.index == index) {
      return &c;
    }
  }
  return nullptr;
}

[[nodiscard]] int Count(const NlpCatchSearch& s, NlpReject why) {
  int n = 0;
  for (const Candidate& c : s.Candidates()) {
    n += c.reject == why ? 1 : 0;
  }
  return n;
}

[[nodiscard]] std::set<NlpReject> Reasons(const NlpCatchSearch& s) {
  std::set<NlpReject> out;
  for (const Candidate& c : s.Candidates()) {
    out.insert(c.reject);
  }
  return out;
}

// The lattice index of the instant `ms` after kNow, for a search whose first
// wake was at kNow (the rig's lattice: 40 ms).
[[nodiscard]] constexpr std::int64_t IndexAt(std::int64_t ms) {
  return ms / 40;
}

// Node k of a segment's node array, in MODEL order.
[[nodiscard]] std::array<double, 6> NodeModelOrder(const std::array<double, 200>& nodes, int k) {
  std::array<double, 6> out{};
  for (std::size_t m = 0; m < 6; ++m) {
    out[m] = nodes[static_cast<std::size_t>(k * kMaxSegmentNv + kDeviceOfModel[m])];
  }
  return out;
}

static_assert(sizeof(SegmentSnapshot::q) == 200 * sizeof(double));

// ── The oracle: the candidates of a wake, solved one by one on cores built
//    here ─────────────────────────────────────────────────────────────────────

[[nodiscard]] MpcDockingSegmentCoreParams CoreParamsFor(const NlpCatchSearchParams& p, int n_pre) {
  MpcDockingSegmentCoreParams cp = p.core;
  cp.n_pre = n_pre;
  cp.dt_pre = p.dt_pre;
  cp.n_stop = p.n_stop;
  cp.dt_stop = p.dt_stop;
  cp.n_blocks = n_pre + p.n_stop_blocks;
  cp.block_sizes.fill(1);
  for (int b = 0; b < p.n_stop_blocks; ++b) {
    cp.block_sizes[static_cast<std::size_t>(n_pre + b)] =
        p.stop_block_sizes[static_cast<std::size_t>(b)];
  }
  return cp;
}

struct OracleCandidate {
  std::int64_t index{0};
  std::int64_t t_c_ns{0};
  std::int64_t t_s_ns{0};
  int n_pre{0};
  bool solved{false};
  bool valid{false};  // an iterate, every hard row within tolerance, converged
  MpcDockingReason reason{MpcDockingReason::kNone};
  double j_reference{0.0}, j_stop{0.0};
  double phi_reference{0.0};  // J⋆ + J_time + J_switch
  double phi_total{0.0};      // the same with the stop part's cost added
  bool ik_accepted{false};
  CatchPoseReason ik_reason{CatchPoseReason::kNone};
  std::array<double, 6> q_ik{};
};

// Every candidate with t_c − t_0 in [t_lead_min, t_max], from a start state the
// caller gives per candidate (model order). `t_c_prev_ns` < 0: no switch term.
template <typename StartFn>
[[nodiscard]] std::vector<OracleCandidate> Exhaustive(const Rig& rig, const Throw& t,
                                                      std::int64_t t_ref_ns, std::int64_t now,
                                                      std::int64_t t_c_prev_ns, StartFn&& start) {
  std::vector<OracleCandidate> out;
  const NlpCatchSearchParams& p = rig.params;
  const std::int64_t t_0 = rig.StartInstant(now);
  const std::int64_t h = Ns(p.cand_dt);
  const std::int64_t dt = Ns(p.dt_pre);
  rtc::testing::Arm ik_arm = rtc::testing::Arm6R();
  CatchPoseIk ik;
  ik.Resize(ik_arm.nv);
  for (std::int64_t i = -1000; i <= 1000; ++i) {
    const std::int64_t t_c = t_ref_ns + i * h;
    const std::int64_t lead = t_c - t_0;
    if (lead < Ns(p.t_lead_min) || lead > Ns(p.t_max)) {
      continue;
    }
    OracleCandidate c;
    c.index = i;
    c.t_c_ns = t_c;
    c.n_pre = static_cast<int>(lead / dt);
    c.t_s_ns = t_c - c.n_pre * dt;
    MpcDockingSegmentCore core;
    const MpcDockingReason why =
        core.Init(*rig.arm.model, rig.arm.frame, CoreParamsFor(p, c.n_pre), p.limits, nullptr);
    EXPECT_EQ(why, MpcDockingReason::kNone) << MpcDockingReasonName(why);
    MpcDockingSegmentCoreInput in;
    MpcDockingSegmentCoreResult res;
    core.ResizeInput(in);
    core.ResizeResult(res);
    int hint = 0;
    for (int k = 0; k <= c.n_pre; ++k) {
      in.ball[static_cast<std::size_t>(k)] =
          SampleBallNode(t.traj, &t.cov, true, BallTime{c.t_s_ns + k * dt}, hint);
    }
    const BallNodeSample& ball = in.ball[static_cast<std::size_t>(c.n_pre)];
    const Eigen::Vector3d v_hat = ball.v.normalized();
    const CatchPoseIkResult r = ik.Solve(*ik_arm.handle, ik_arm.frame,
                                         ball.p + p.core.s_ent * v_hat, ball.v, rig.q_wait, rig.ik);
    c.ik_accepted = r.accepted;
    c.ik_reason = r.reason;
    if (r.accepted) {
      for (std::size_t m = 0; m < 6; ++m) {
        c.q_ik[m] = r.q[m];
        in.q_catch_target[static_cast<Eigen::Index>(m)] = r.q[m];
      }
      Eigen::VectorXd q0(6);
      Eigen::VectorXd qd0(6);
      Eigen::VectorXd qdd0(6);
      if (start(c, q0, qd0, qdd0)) {
        in.q0 = q0;
        in.qd0 = qd0;
        in.qdd0 = qdd0;
        in.catch_target_valid = true;
        in.initial_valid = false;
        in.p_line = ball.p;
        in.d_line = v_hat;
        in.deadline_ns = 0;
        c.solved = core.Solve(in, res);
        c.reason = res.reason;
        if (c.solved) {
          c.valid = res.feasible && res.converged;
          c.j_reference = res.cost.reference;
          c.j_stop = res.cost.stop;
          const double j_time = p.w_time * Sec(lead) / p.t_ref_s;
          const double d = t_c_prev_ns < 0 ? 0.0 : Sec(t_c - t_c_prev_ns) / p.t_ref_s;
          const double j_switch = p.w_switch * d * d;
          c.phi_reference = res.cost.reference + j_time + j_switch;
          c.phi_total = c.phi_reference + res.cost.stop;
        }
      }
    }
    out.push_back(c);
  }
  return out;
}

[[nodiscard]] const OracleCandidate* ArgMin(const std::vector<OracleCandidate>& all,
                                            double OracleCandidate::*key) {
  const OracleCandidate* best = nullptr;
  for (const OracleCandidate& c : all) {
    if (c.valid && (best == nullptr || c.*key < best->*key)) {
      best = &c;
    }
  }
  return best;
}

// The arm at rest on the wait pose.
[[nodiscard]] auto RestStart(const Rig& rig) {
  return [&rig](const OracleCandidate&, Eigen::VectorXd& q0, Eigen::VectorXd& qd0,
                Eigen::VectorXd& qdd0) {
    q0 = rig.q_wait;
    qd0.setZero();
    qdd0.setZero();
    return true;
  };
}

// The arm on `seg` at the candidate's node-0 instant.
[[nodiscard]] auto SegmentStart(const SegmentSnapshot& seg) {
  return [&seg](const OracleCandidate& c, Eigen::VectorXd& q0, Eigen::VectorXd& qd0,
                Eigen::VectorXd& qdd0) {
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    if (!NodeTrajectoryFollower::SampleJoints(seg, c.t_s_ns, q, qd, qdd)) {
      return false;
    }
    for (std::size_t m = 0; m < 6; ++m) {
      const auto d = static_cast<std::size_t>(kDeviceOfModel[m]);
      const auto mi = static_cast<Eigen::Index>(m);
      q0[mi] = q[d];
      qd0[mi] = qd[d];
      qdd0[mi] = qdd[d];
    }
    return true;
  };
}

// A first wake at kNow on the off-axis throw. Returns the chosen candidate's
// solution as the segment the RT then follows — a segment that moves.
struct Adopted {
  Throw ball;
  Wake first;
  SegmentSnapshot followed{};
  std::int64_t t_c_ns{0};
  std::int64_t index{0};
  std::int64_t wait_ns{0};  // t_s − t_0 of the chosen candidate at the first wake
};

[[nodiscard]] std::unique_ptr<Adopted> AdoptFirst(Rig& rig, double t_star_s = 0.28) {
  auto a = std::make_unique<Adopted>();
  a->ball = OffAxisThrow(rig, t_star_s);
  a->first = RunWake(rig, a->ball, rig.RestingRt(kNow), NoSegments(), kNow);
  EXPECT_TRUE(a->first.plan.valid) << Table(rig.search, a->first.stats);
  const CatchSolution* sol = rig.search.Solution();
  EXPECT_NE(sol, nullptr);
  if (sol != nullptr) {
    a->followed = sol->seg;
    a->followed.segment_seq = 11;
    a->followed.plan_id = 4;
    a->followed.publish_ns = kNow + kMs;
  }
  a->t_c_ns = a->first.plan.t_c_ns;
  a->index = a->first.stats.nlp.chosen_index;
  const Candidate* chosen = Find(rig.search, a->index);
  a->wait_ns = chosen != nullptr ? chosen->wait_ns : 0;
  return a;
}

[[nodiscard]] std::unique_ptr<ReportedSegments> Following(const SegmentSnapshot& seg) {
  auto r = std::make_unique<ReportedSegments>();
  r->has_following = true;
  r->following = seg;
  return r;
}

// ── 1. Screening ─────────────────────────────────────────────────────────────

TEST(NlpCatchSearchScreening, TheIkVerdictIsCatchPoseIksOwn) {
  // Three throws: one every in-window candidate's IK converges on, one whose
  // late candidates are out of the arm's reach, one refused by the
  // catchability gate.
  struct Case {
    const char* name;
    double speed;
    double manip_min;
    bool expect_accept, expect_ik, expect_manip;
  };

  for (const Case tc : {Case{"reachable", 0.8, 0.0, true, false, false},
                        Case{"a fast ball leaves the workspace", 6.0, 0.0, true, true, false},
                        Case{"catchability gate", 0.8, 1e6, false, false, true}}) {
    SCOPED_TRACE(tc.name);
    auto rig = std::make_unique<Rig>();
    rig->ik.manipulability_min = tc.manip_min;
    // Nothing but the IK may remove a candidate before it runs, and nothing
    // after it matters here.
    rig->params.t_max = 0.4;
    std::string err;
    ASSERT_TRUE(rig->Configure(&err)) << err;
    const Throw ball = AxisThrow(*rig, 0.16, tc.speed);
    const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
    const auto oracle = Exhaustive(*rig, ball, kNow, kNow, -1, [](auto&&...) { return false; });
    ASSERT_GE(oracle.size(), 6U);
    int accepted = 0;
    int ik_fail = 0;
    int manip_fail = 0;
    for (const OracleCandidate& o : oracle) {
      const Candidate* c = Find(rig->search, o.index);
      ASSERT_NE(c, nullptr) << o.index;
      ASSERT_TRUE(c->ik_run) << o.index << Table(rig->search, w.stats);
      EXPECT_EQ(c->ik_reason, o.ik_reason) << o.index;
      if (o.ik_accepted) {
        ++accepted;
        for (std::size_t m = 0; m < 6; ++m) {
          EXPECT_TRUE(BitsEqual(c->q_ik[m], o.q_ik[m])) << o.index << " joint " << m;
        }
        EXPECT_NE(c->reject, NlpReject::kIk);
        EXPECT_NE(c->reject, NlpReject::kManipulability);
      } else if (o.ik_reason == CatchPoseReason::kBelowManipMin) {
        ++manip_fail;
        EXPECT_EQ(c->reject, NlpReject::kManipulability) << o.index;
      } else {
        ++ik_fail;
        EXPECT_EQ(c->reject, NlpReject::kIk) << o.index;
      }
    }
    EXPECT_EQ(accepted > 0, tc.expect_accept) << Table(rig->search, w.stats);
    EXPECT_EQ(ik_fail > 0, tc.expect_ik) << Table(rig->search, w.stats);
    EXPECT_EQ(manip_fail > 0, tc.expect_manip) << Table(rig->search, w.stats);
    EXPECT_EQ(w.stats.n_ik, static_cast<std::uint16_t>(oracle.size()));
  }
}

// The reference's S4 for one joint, on scalars.
[[nodiscard]] bool ReachOk(double q_c, double q_0, double v_0, double v_max, double a_max,
                           bool accel, double T) {
  const bool by_velocity = std::fabs(q_c - q_0) <= v_max * T;
  const bool by_acceleration = !accel || std::fabs(q_c - q_0 - v_0 * T) <= 0.5 * a_max * T * T;
  return by_velocity && by_acceleration;
}

// S4 as the reference writes it, for every candidate the IK accepted: first
// failing joint and which condition, with T the time the arm MOVES.
void ExpectReachIsTheFormula(const Rig& rig, const SearchStats& stats, int& passed, int& failed) {
  const NlpCatchSearchParams& p = rig.params;
  for (const Candidate& c : rig.search.Candidates()) {
    if (!c.ik_run || c.ik_reason != CatchPoseReason::kNone) {
      continue;
    }
    const double t_move = c.n_pre * p.dt_pre;
    NlpReachLimit limit = NlpReachLimit::kNone;
    int joint = -1;
    for (int m = 0; m < 6 && joint < 0; ++m) {
      const auto u = static_cast<std::size_t>(m);
      const double dq = c.q_ik[u] - c.q0[u];
      if (!(std::fabs(dq) <= p.limits.qd_max[m] * t_move)) {
        limit = NlpReachLimit::kVelocity;
        joint = m;
      } else if (p.core.accel_box && !(std::fabs(dq - c.qd0[u] * t_move) <=
                                       0.5 * p.limits.qdd_max[m] * t_move * t_move)) {
        limit = NlpReachLimit::kAcceleration;
        joint = m;
      }
    }
    EXPECT_EQ(c.reach_limit, limit) << c.index << Table(rig.search, stats);
    EXPECT_EQ(c.reach_joint, joint) << c.index;
    EXPECT_EQ(c.reject == NlpReject::kReach, joint >= 0) << c.index;
    (joint >= 0 ? failed : passed) += 1;
  }
}

TEST(NlpCatchSearchScreening, ReachIsTheReferencesConditionOverTheTimeTheArmMoves) {
  // From rest. One joint's velocity limit is then set BETWEEN what the
  // condition needs over the moving time n_pre·Δ_a and what it would need over
  // the whole lead t_c − t_0: with the right time the candidate is unreachable,
  // with the lead it would pass.
  auto probe = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(probe->Configure(&err)) << err;
  const Throw ball = AxisThrow(*probe, 0.28);
  const Wake w0 = RunWake(*probe, ball, probe->RestingRt(kNow), NoSegments(), kNow);
  // A candidate that waits before it moves and needs a real joint motion.
  const Candidate* pick = nullptr;
  int joint = 0;
  for (const Candidate& c : probe->search.Candidates()) {
    if (c.reject != NlpReject::kNone || c.wait_ns < 20 * kMs) {
      continue;
    }
    for (int m = 0; m < 6; ++m) {
      const auto u = static_cast<std::size_t>(m);
      if (std::fabs(c.q_ik[u] - c.q0[u]) > 0.02) {
        pick = &c;
        joint = m;
      }
    }
  }
  ASSERT_NE(pick, nullptr) << Table(probe->search, w0.stats);
  const double dq = std::fabs(pick->q_ik[static_cast<std::size_t>(joint)] -
                              pick->q0[static_cast<std::size_t>(joint)]);
  const double t_move = pick->n_pre * probe->params.dt_pre;
  const double t_lead = pick->lead_s;
  ASSERT_GT(t_lead, t_move + 0.015);
  const std::int64_t index = pick->index;

  auto rig = std::make_unique<Rig>();
  rig->params.limits.qd_max[joint] = dq / (0.5 * (t_move + t_lead));
  ASSERT_FALSE(ReachOk(dq, 0.0, 0.0, rig->params.limits.qd_max[joint], 0.0, false, t_move));
  ASSERT_TRUE(ReachOk(dq, 0.0, 0.0, rig->params.limits.qd_max[joint], 0.0, false, t_lead));
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  const Candidate* c = Find(rig->search, index);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->reject, NlpReject::kReach) << Table(rig->search, w.stats);
  EXPECT_EQ(c->reach_limit, NlpReachLimit::kVelocity);
  EXPECT_EQ(c->reach_joint, joint);
  int passed = 0;
  int failed = 0;
  ExpectReachIsTheFormula(*rig, w.stats, passed, failed);
  EXPECT_GT(passed, 0) << "no candidate on the passing side";
  EXPECT_GT(failed, 0) << "no candidate on the failing side";
}

TEST(NlpCatchSearchScreening, ReachFromAMovingArmCountsItsStartVelocity) {
  // After adoption the arm is on its segment: x_0 has a velocity. With the
  // acceleration box on, one joint's limit is set BETWEEN what the condition
  // needs with that start velocity and what it would need from rest, so that
  // the v_0·T term alone decides the verdict.
  auto probe = std::make_unique<Rig>();
  probe->params.core.accel_box = true;
  probe->params.limits.qdd_max = Eigen::VectorXd::Constant(6, 1e4);
  std::string err;
  ASSERT_TRUE(probe->Configure(&err)) << err;
  const auto adopted = AdoptFirst(*probe, 0.20);
  ASSERT_TRUE(adopted->first.plan.valid);
  const std::int64_t now = kNow + 66 * kMs;
  const auto arm = Following(adopted->followed);
  const Wake w0 =
      RunWake(*probe, adopted->ball, probe->FollowingRt(now, adopted->t_c_ns), *arm, now);

  struct Pick {
    std::int64_t index;
    int joint;
    double need_moving, need_rest, t_move;
  };

  // The candidate and joint on which the two needs differ most.
  Pick pick{0, -1, 0.0, 0.0, 0.0};
  double best_ratio = 1.0;
  for (const Candidate& c : probe->search.Candidates()) {
    if (!c.ik_run || c.ik_reason != CatchPoseReason::kNone) {
      continue;
    }
    const double T = c.n_pre * probe->params.dt_pre;
    for (int m = 0; m < 6; ++m) {
      const auto u = static_cast<std::size_t>(m);
      const double dq = c.q_ik[u] - c.q0[u];
      const double moving = std::fabs(dq - c.qd0[u] * T) / (0.5 * T * T);
      const double rest = std::fabs(dq) / (0.5 * T * T);
      const double lo = std::min(moving, rest);
      const double hi = std::max(moving, rest);
      if (lo > 0.05 && hi / lo > best_ratio) {
        best_ratio = hi / lo;
        pick = {c.index, m, moving, rest, T};
      }
    }
  }
  ASSERT_GE(pick.joint, 0) << Table(probe->search, w0.stats);
  ASSERT_GT(best_ratio, 1.2) << "no candidate whose start velocity matters"
                             << Table(probe->search, w0.stats);

  auto rig = std::make_unique<Rig>();
  rig->params.core.accel_box = true;
  rig->params.limits.qdd_max = Eigen::VectorXd::Constant(6, 1e4);
  const double a_max = 0.5 * (pick.need_moving + pick.need_rest);
  rig->params.limits.qdd_max[pick.joint] = a_max;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  // The same lattice (anchored by a first wake at kNow, whatever it chooses
  // under the tightened limit) and the same reported segment as the probe's
  // wake: the same candidates from the same start states.
  static_cast<void>(RunWake(*rig, adopted->ball, rig->RestingRt(kNow), NoSegments(), kNow));
  ASSERT_EQ(rig->search.LatticeAnchorNs(), probe->search.LatticeAnchorNs());
  const Wake w = RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
  const Candidate* c = Find(rig->search, pick.index);
  ASSERT_NE(c, nullptr);
  const auto u = static_cast<std::size_t>(pick.joint);
  ASSERT_GT(std::fabs(c->qd0[u]), 1e-3) << "the joint does not move at node 0";
  const bool with_v0 = ReachOk(c->q_ik[u], c->q0[u], c->qd0[u], kInf, a_max, true, pick.t_move);
  const bool from_rest = ReachOk(c->q_ik[u], c->q0[u], 0.0, kInf, a_max, true, pick.t_move);
  ASSERT_NE(with_v0, from_rest) << Table(rig->search, w.stats);
  EXPECT_EQ(with_v0, pick.need_moving < pick.need_rest);
  EXPECT_EQ(c->reject == NlpReject::kReach, !with_v0) << Table(rig->search, w.stats);
  if (!with_v0) {
    EXPECT_EQ(c->reach_limit, NlpReachLimit::kAcceleration);
    EXPECT_EQ(c->reach_joint, pick.joint);
  } else {
    EXPECT_EQ(c->reach_limit, NlpReachLimit::kNone);
  }
  int passed = 0;
  int failed = 0;
  ExpectReachIsTheFormula(*rig, w.stats, passed, failed);
  EXPECT_GT(passed + failed, 3);
}

// m_red at `q` from the mass matrix and the frame's Jacobian, computed here:
// (1/m_b + nᵀ J M⁻¹ Jᵀ n)⁻¹ with n the capture axis and the contact point the
// frame origin.
[[nodiscard]] double ReducedMass(const Rig& rig, const std::array<double, kMaxSegmentNv>& q_model,
                                 double m_ball) {
  const pinocchio::Model& model = rig.dock.model;  // armature included
  pinocchio::Data data(model);
  Eigen::VectorXd q(6);
  for (int m = 0; m < 6; ++m) {
    q[m] = q_model[static_cast<std::size_t>(m)];
  }
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
  Eigen::MatrixXd j6 = Eigen::MatrixXd::Zero(6, 6);
  pinocchio::computeFrameJacobian(model, data, q, rig.dock.arm.frame,
                                  pinocchio::LOCAL_WORLD_ALIGNED, j6);
  const Eigen::Vector3d n = data.oMf[rig.dock.arm.frame].rotation().col(2);
  const Eigen::VectorXd f = j6.topRows(3).transpose() * n;
  const double beta = f.dot(data.M.ldlt().solve(f));
  return 1.0 / (1.0 / m_ball + beta);
}

TEST(NlpCatchSearchScreening, TheClosingSpeedWindowIsTheReferencesAtTheIkPose) {
  // The window's two inner edges, recomputed from the IK pose: σ_s from the
  // approach axis there and Σ_p at the catch instant, m_red from the mass
  // matrix. Then each edge is moved until the window closes.
  struct Case {
    const char* name;
    double sigma_axial;  // ball position std along its travel [m]
    double e_max, p_max;
    bool expect_empty;
  };

  for (const Case tc :
       {Case{"both rows open", 0.002, 0.5, 0.5, false},
        Case{"timing: the ball's instant is too uncertain", 0.02, kInf, kInf, true},
        Case{"impact: the energy limit is below any catch", 0.002, 1e-4, kInf, true},
        Case{"impact: the impulse limit is", 0.002, kInf, 1e-3, true}}) {
    SCOPED_TRACE(tc.name);
    auto rig = std::make_unique<Rig>();
    rig->params.core.e_max = tc.e_max;
    rig->params.core.p_max = tc.p_max;
    std::string err;
    ASSERT_TRUE(rig->Configure(&err)) << err;
    Throw ball = AxisThrow(*rig, 0.28);
    ball.cov = LateralCovariance(ball, 0.002, tc.sigma_axial, 0.002);
    const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
    const MpcDockingSegmentCoreParams& cp = rig->params.core;
    const double sigma_max = rig->search.Core(rig->params.n_pre_min)->TimingSigmaMax();
    ASSERT_GT(sigma_max, cp.sigma_tau);
    int judged = 0;
    for (const Candidate& c : rig->search.Candidates()) {
      // Reached the window: the IK accepted and the pose is reachable.
      if (!c.ik_run || c.ik_reason != CatchPoseReason::kNone ||
          c.reach_limit != NlpReachLimit::kNone || c.reject == NlpReject::kReach) {
        continue;
      }
      ++judged;
      int hint = 0;
      const BallNodeSample b = SampleBallNode(ball.traj, &ball.cov, true, BallTime{c.t_c_ns}, hint);
      Eigen::VectorXd q(6);
      for (int m = 0; m < 6; ++m) {
        q[m] = c.q_ik[static_cast<std::size_t>(m)];
      }
      const dk::HandState h = dk::HandAt(rig->dock, q, Eigen::VectorXd::Zero(6));
      const Eigen::Vector3d d = h.R.col(2);
      const double sigma_s =
          std::sqrt(d.dot(b.cov.topLeftCorner<3, 3>() * d) + cp.eps_sigma * cp.eps_sigma);
      const double c_t_lo =
          sigma_s / std::sqrt(sigma_max * sigma_max - cp.sigma_tau * cp.sigma_tau);
      const double m_red = ReducedMass(*rig, c.q_ik, cp.m_ball);
      const double c_n_hi =
          std::min(std::sqrt(2.0 * cp.e_max / m_red), cp.p_max / ((1.0 + cp.restitution) * m_red));
      const double lo = std::max(cp.c_min, c_t_lo);
      const double hi = std::min(cp.c_cap_max, c_n_hi);
      EXPECT_NEAR(c.speed_lo, lo, 1e-12 * std::max(1.0, lo)) << c.index;
      EXPECT_NEAR(c.speed_hi, hi, 1e-9 * std::max(1.0, hi)) << c.index;
      EXPECT_EQ(c.reject == NlpReject::kSpeedWindow, lo > hi) << c.index;
      EXPECT_EQ(lo > hi, tc.expect_empty) << c.index << " lo " << lo << " hi " << hi;
    }
    EXPECT_GE(judged, 3) << Table(rig->search, w.stats);
  }
}

// ── 2. The choice ────────────────────────────────────────────────────────────

void ExpectTheChoiceIsTheExhaustiveArgMin(const Rig& rig, const Wake& w,
                                          const std::vector<OracleCandidate>& oracle) {
  int valid = 0;
  for (const OracleCandidate& o : oracle) {
    valid += o.valid ? 1 : 0;
    const Candidate* c = Find(rig.search, o.index);
    ASSERT_NE(c, nullptr) << o.index;
    EXPECT_EQ(c->n_pre, o.n_pre) << o.index;
    EXPECT_EQ(c->t_s_ns, o.t_s_ns) << o.index;
    // What the search called valid, the exhaustive solve calls valid — and
    // nothing it removed before solving turns out to have a valid solution.
    EXPECT_EQ(c->reject == NlpReject::kNone, o.valid)
        << o.index << " core " << MpcDockingReasonName(o.reason) << Table(rig.search, w.stats);
    if (c->reject == NlpReject::kNone && o.valid) {
      EXPECT_TRUE(BitsEqual(c->phi, o.phi_reference))
          << o.index << " " << c->phi << " vs " << o.phi_reference;
      EXPECT_TRUE(BitsEqual(c->j_reference, o.j_reference)) << o.index;
      EXPECT_TRUE(BitsEqual(c->j_stop, o.j_stop)) << o.index;
    }
  }
  ASSERT_GE(oracle.size(), 6U);
  ASSERT_GE(valid, 3) << Table(rig.search, w.stats);
  const OracleCandidate* best = ArgMin(oracle, &OracleCandidate::phi_reference);
  ASSERT_NE(best, nullptr);
  ASSERT_TRUE(w.plan.valid) << Table(rig.search, w.stats);
  EXPECT_EQ(w.stats.nlp.chosen_index, best->index) << Table(rig.search, w.stats);
  EXPECT_EQ(w.plan.t_c_ns, best->t_c_ns);
  EXPECT_TRUE(BitsEqual(w.plan.score, best->phi_reference));
  EXPECT_TRUE(BitsEqual(w.stats.nlp.chosen_phi, best->phi_reference));
  // The minimum is a minimum by a margin no solver tolerance explains.
  for (const OracleCandidate& o : oracle) {
    if (o.valid && o.index != best->index) {
      EXPECT_GT(o.phi_reference - best->phi_reference, 1e-4) << o.index;
    }
  }
}

TEST(NlpCatchSearchChoice, FromRestItIsTheExhaustiveSolvesArgMin) {
  auto rig = std::make_unique<Rig>();
  // The catch box ends between the candidates at +400 and +440 ms.
  const Throw probe_ball = AxisThrow(*rig, 0.28);
  int hint = 0;
  const BallNodeSample at_420 =
      SampleBallNode(probe_ball.traj, &probe_ball.cov, true, BallTime{kNow + 420 * kMs}, hint);
  const BallNodeSample at_wait =
      SampleBallNode(probe_ball.traj, &probe_ball.cov, true, BallTime{kNow + 280 * kMs}, hint);
  // A box around the ball's line, cut by a plane across it at the +420 ms point.
  for (int a = 0; a < 3; ++a) {
    const auto u = static_cast<std::size_t>(a);
    const double lo = std::min(at_wait.p[a], at_420.p[a]);
    const double hi = std::max(at_wait.p[a], at_420.p[a]);
    const bool toward_min = at_420.p[a] < at_wait.p[a];
    rig->params.catch_box.min[u] = toward_min ? lo : lo - 1.0;
    rig->params.catch_box.max[u] = toward_min ? hi + 1.0 : hi;
  }
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Wake w = RunWake(*rig, probe_ball, rig->RestingRt(kNow), NoSegments(), kNow);
  // Three kinds of removal besides the valid ones.
  const std::set<NlpReject> reasons = Reasons(rig->search);
  EXPECT_TRUE(reasons.contains(NlpReject::kLeadShort)) << Table(rig->search, w.stats);
  EXPECT_TRUE(reasons.contains(NlpReject::kWorkspace)) << Table(rig->search, w.stats);
  EXPECT_TRUE(reasons.contains(NlpReject::kReach)) << Table(rig->search, w.stats);
  // The oracle knows no catch box: take the candidates the box removed out.
  std::vector<OracleCandidate> oracle =
      Exhaustive(*rig, probe_ball, kNow, kNow, -1, RestStart(*rig));
  std::erase_if(oracle, [&](const OracleCandidate& o) {
    const Candidate* c = Find(rig->search, o.index);
    return c != nullptr && c->reject == NlpReject::kWorkspace;
  });
  ExpectTheChoiceIsTheExhaustiveArgMin(*rig, w, oracle);
  EXPECT_EQ(w.stats.decision, SwitchDecision::kNoCurrent);
  EXPECT_TRUE(w.stats.publish);
  EXPECT_EQ(w.stats.nlp.reason, NlpReject::kNone);
  EXPECT_EQ(w.plan.reason, PlanReason::kNone);
}

TEST(NlpCatchSearchChoice, FromAMovingArmItIsTheExhaustiveSolvesArgMin) {
  // The RT follows a segment — the first wake's solution for a candidate that
  // needs motion — and the prediction has moved: another snapshot, the ball
  // 60 ms later on the same line. A search with NO memory (a fresh object) so
  // that every solve starts from its IK pose, as the oracle's do.
  auto first = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(first->Configure(&err)) << err;
  const auto adopted = AdoptFirst(*first, 0.20);
  ASSERT_TRUE(adopted->first.plan.valid);

  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw moved = AxisThrow(*rig, 0.26, 0.8, 0.002, /*seq=*/2);
  const std::int64_t now = kNow + 33 * kMs;
  const auto arm = Following(adopted->followed);
  const Wake w = RunWake(*rig, moved, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
  // The lattice of a fresh search is anchored at ITS first wake.
  ASSERT_EQ(rig->search.LatticeAnchorNs(), now);
  const auto oracle = Exhaustive(*rig, moved, now, now, -1, SegmentStart(adopted->followed));
  ExpectTheChoiceIsTheExhaustiveArgMin(*rig, w, oracle);
  // The start really moved: a candidate's x_0 is not the wait pose at rest.
  bool moving = false;
  for (const Candidate& c : rig->search.Candidates()) {
    for (std::size_t m = 0; m < 6; ++m) {
      moving = moving || std::fabs(c.qd0[m]) > 1e-3;
    }
    if (c.reject == NlpReject::kNone) {
      EXPECT_EQ(c.source_seq, 11U);
    }
  }
  EXPECT_TRUE(moving) << Table(rig->search, w.stats);
}

TEST(NlpCatchSearchChoice, TheStopPartsCostIsRecordedAndNotChosenOn) {
  // Two candidates and a time weight chosen so that A wins on the reference's
  // cost J⋆ + J_time while B would win if the stop part's cost were added.
  auto probe = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(probe->Configure(&err)) << err;
  const Throw ball = AxisThrow(*probe, 0.28);
  const auto base = Exhaustive(*probe, ball, kNow, kNow, -1, RestStart(*probe));
  // B: the free candidate (the ball comes to the wait pose). A: an earlier one
  // that needs motion, hence a stop afterwards.
  const OracleCandidate* b = ArgMin(base, &OracleCandidate::phi_reference);
  ASSERT_NE(b, nullptr);
  const OracleCandidate* a = nullptr;
  for (const OracleCandidate& o : base) {
    if (o.valid && o.t_c_ns < b->t_c_ns && o.j_stop - b->j_stop > 1e-3 &&
        (a == nullptr || o.t_c_ns > a->t_c_ns)) {
      a = &o;
    }
  }
  ASSERT_NE(a, nullptr);
  const double d_stop = a->j_stop - b->j_stop;
  const double d_ref = a->j_reference - b->j_reference;
  const double d_lead = Sec(b->t_c_ns - a->t_c_ns);
  // Φ_A − Φ_B = d_ref − w_T·d_lead/T_ref := −½ d_stop.
  auto rig = std::make_unique<Rig>();
  rig->params.w_time = (d_ref + 0.5 * d_stop) * rig->params.t_ref_s / d_lead;
  ASSERT_GT(rig->params.w_time, 0.0);
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  const auto oracle = Exhaustive(*rig, ball, kNow, kNow, -1, RestStart(*rig));
  const OracleCandidate* by_reference = ArgMin(oracle, &OracleCandidate::phi_reference);
  const OracleCandidate* by_total = ArgMin(oracle, &OracleCandidate::phi_total);
  ASSERT_NE(by_reference, nullptr);
  ASSERT_NE(by_total, nullptr);
  ASSERT_NE(by_reference->index, by_total->index)
      << "the stop part's cost does not change the order here";
  ASSERT_TRUE(w.plan.valid) << Table(rig->search, w.stats);
  EXPECT_EQ(w.stats.nlp.chosen_index, by_reference->index) << Table(rig->search, w.stats);
  EXPECT_TRUE(BitsEqual(w.plan.score, by_reference->phi_reference));
  // Recorded, both parts.
  EXPECT_TRUE(BitsEqual(w.stats.nlp.chosen_j_reference, by_reference->j_reference));
  EXPECT_TRUE(BitsEqual(w.stats.nlp.chosen_j_stop, by_reference->j_stop));
  EXPECT_GT(w.stats.nlp.chosen_j_stop, 0.0);
  EXPECT_DOUBLE_EQ(w.stats.nlp.chosen_j_time,
                   rig->params.w_time * Sec(by_reference->t_c_ns - rig->StartInstant(kNow)) /
                       rig->params.t_ref_s);
  const CatchSolution* sol = rig->search.Solution();
  ASSERT_NE(sol, nullptr);
  EXPECT_TRUE(BitsEqual(sol->cost_reference, by_reference->j_reference));
  EXPECT_TRUE(BitsEqual(sol->cost_stop, by_reference->j_stop));
}

TEST(NlpCatchSearchChoice, ThePlanAndTheSolutionDescribeTheChosenCandidate) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.20);
  const PlannerRtState rt = rig->RestingRt(kNow);
  const Wake w = RunWake(*rig, ball, rt, NoSegments(), kNow);
  ASSERT_TRUE(w.plan.valid) << Table(rig->search, w.stats);
  const Candidate* c = Find(rig->search, w.stats.nlp.chosen_index);
  ASSERT_NE(c, nullptr);
  int hint = 0;
  const BallNodeSample b = SampleBallNode(ball.traj, &ball.cov, true, BallTime{c->t_c_ns}, hint);
  // The plan: the ball at the catch instant, the approach axis against it.
  EXPECT_EQ(w.plan.t_c_ns, c->t_c_ns);
  EXPECT_EQ(w.plan.t_cmd_ns, c->t_c_ns - Ns(rig->constants.t_close_e2e));
  for (int a = 0; a < 3; ++a) {
    const auto u = static_cast<std::size_t>(a);
    EXPECT_TRUE(BitsEqual(w.plan.p_c[u], b.p[a]));
    EXPECT_TRUE(BitsEqual(w.plan.v_c[u], b.v[a]));
    EXPECT_NEAR(w.plan.a_d[u], -ball.v_hat[a], 1e-12);
  }
  EXPECT_EQ(w.plan.nv, 6);
  EXPECT_EQ(w.plan.token.generation, kTrack);
  EXPECT_EQ(w.plan.token.snapshot_sequence, 1U);
  EXPECT_EQ(w.plan.token.activation_generation, kActivation);
  EXPECT_EQ(w.plan.rt_iteration, rt.rt_iteration);
  EXPECT_EQ(w.plan.rt_state_ns, rt.rt_state_ns);
  EXPECT_GT(w.plan.w5, 0.0);
  EXPECT_GT(w.plan.w6, 0.0);
  // … of the pose the plan carries — the SOLVED catch pose, not the IK pose
  // the solve was aimed at: CatchPoseIk's own numbers at q_star.
  {
    Eigen::VectorXd q_star(6);
    Eigen::VectorXd q_ik(6);
    for (int m = 0; m < 6; ++m) {
      q_star[m] =
          w.plan.q_star[static_cast<std::size_t>(kDeviceOfModel[static_cast<std::size_t>(m)])];
      q_ik[m] = c->q_ik[static_cast<std::size_t>(m)];
    }
    CatchPoseIk probe;
    probe.Resize(6);
    double w5 = 0.0;
    double w6 = 0.0;
    ASSERT_TRUE(probe.Manipulability(*rig->arm.handle, rig->arm.frame, q_star, w5, w6));
    EXPECT_TRUE(BitsEqual(w.plan.w5, w5)) << w.plan.w5 << " vs " << w5;
    EXPECT_TRUE(BitsEqual(w.plan.w6, w6)) << w.plan.w6 << " vs " << w6;
    // The two poses are different poses here, with different numbers.
    EXPECT_GT((q_star - q_ik).cwiseAbs().maxCoeff(), 1e-4);
    EXPECT_GT(std::fabs(w.plan.w5 - c->w5), 1e-6 * c->w5) << w.plan.w5 << " vs " << c->w5;
  }
  EXPECT_NEAR(w.plan.sigma_c, 0.002, 2e-4);  // √λ_max(Σ_p) of an isotropic 2 mm
  EXPECT_NEAR(w.plan.dp_impact, rig->params.core.m_ball * 0.8, 0.05);
  EXPECT_GT(w.stats.chosen_lead_s, 0.0);
  EXPECT_DOUBLE_EQ(w.stats.chosen_lead_s, Sec(c->t_c_ns - kNow));
  EXPECT_DOUBLE_EQ(w.stats.nlp.chosen_lead_s, Sec(c->t_c_ns - rig->StartInstant(kNow)));
  EXPECT_EQ(w.stats.nlp.chosen_n_pre, c->n_pre);
  EXPECT_EQ(w.stats.nlp.chosen_source_seq, 0U);

  // The solution: a segment the RT's own validator and sampler accept, on the
  // candidate's grid, from the start state, in DEVICE order.
  const CatchSolution* sol = rig->search.Solution();
  ASSERT_NE(sol, nullptr);
  const SegmentSnapshot& seg = sol->seg;
  EXPECT_TRUE(ValidateSegmentNodes(seg));
  EXPECT_EQ(seg.t_c_ns, c->t_c_ns);
  EXPECT_EQ(seg.t0_ns, c->t_s_ns);
  EXPECT_EQ(seg.n_pre, c->n_pre);
  EXPECT_EQ(seg.n_nodes, c->n_pre + rig->params.n_stop);
  EXPECT_EQ(seg.dt_pre_ns, 50 * kMs);
  EXPECT_EQ(seg.dt_ns, 50 * kMs);
  EXPECT_EQ(seg.nv, 6);
  EXPECT_EQ(seg.rt_iteration, rt.rt_iteration);
  EXPECT_EQ(seg.token.generation, kTrack);
  EXPECT_EQ(seg.plan_id, 0U);
  EXPECT_EQ(seg.segment_seq, 0U);
  // Nothing in the node arrays but this solution: the entries no joint or
  // node of it uses are zero, not an earlier solve's.
  for (int k = 0; k <= rtc::catching::kMaxSegmentNodes; ++k) {
    for (int d = 0; d < kMaxSegmentNv; ++d) {
      if (k > seg.n_nodes || d >= seg.nv) {
        const auto e = static_cast<std::size_t>(k * kMaxSegmentNv + d);
        ASSERT_EQ(seg.q[e], 0.0) << k << " " << d;
        ASSERT_EQ(seg.qd[e], 0.0) << k << " " << d;
        ASSERT_EQ(seg.qdd[e], 0.0) << k << " " << d;
      }
    }
  }
  EXPECT_FALSE(seg.x0_clamped);
  EXPECT_TRUE(sol->feasible);
  EXPECT_TRUE(sol->converged);
  EXPECT_EQ(sol->source_seq, 0U);
  const std::array<double, 6> node0 = NodeModelOrder(seg.q, 0);
  const std::array<double, 6> catch_node = NodeModelOrder(seg.q, c->n_pre);
  double moved = 0.0;
  for (std::size_t m = 0; m < 6; ++m) {
    EXPECT_TRUE(BitsEqual(node0[m], rig->q_wait[static_cast<Eigen::Index>(m)])) << m;
    EXPECT_TRUE(
        BitsEqual(w.plan.q_star[static_cast<std::size_t>(kDeviceOfModel[m])], catch_node[m]))
        << m;
    moved = std::max(moved, std::fabs(catch_node[m] - node0[m]));
  }
  EXPECT_GT(moved, 1e-3) << "the chosen candidate needs no motion: the order is not tested";
  // At the catch node the frame is where the capture puts it: the ball on the
  // entrance plane, s_ent up the capture axis.
  Eigen::VectorXd q_c(6);
  Eigen::VectorXd v_c(6);
  const std::array<double, 6> catch_v = NodeModelOrder(seg.qd, c->n_pre);
  for (int m = 0; m < 6; ++m) {
    q_c[m] = catch_node[static_cast<std::size_t>(m)];
    v_c[m] = catch_v[static_cast<std::size_t>(m)];
  }
  const dk::Relative rel = dk::RelativeAt(dk::HandAt(rig->dock, q_c, v_c), b.p, b.v);
  EXPECT_NEAR(rel.s, rig->params.core.s_ent, 1e-6);
  // A wake that chooses nothing has no solution.
  Throw gone = ball;
  gone.traj.valid = false;
  const Wake none =
      RunWake(*rig, gone, rig->RestingRt(kNow + 33 * kMs), NoSegments(), kNow + 33 * kMs);
  EXPECT_FALSE(none.plan.valid);
  EXPECT_EQ(rig->search.Solution(), nullptr);
}

// ── 3. Order independence ────────────────────────────────────────────────────

// Everything a wake produced that is not a duration.
[[nodiscard]] std::uint64_t WakeDigest(const NlpCatchSearch& s, const Wake& w) {
  rtc::testing::ValueDigest h;
  rtc::testing::AddPlan(h, w.plan);
  h.Add(w.stats.n_in_window);
  h.Add(w.stats.n_ik);
  h.Add(w.stats.n_pass);
  h.Add(w.stats.budget_hit);
  h.Add(w.stats.chosen_score);
  h.Add(w.stats.decision);
  h.Add(w.stats.publish);
  h.Add(w.stats.nlp.reason);
  h.Add(w.stats.nlp.n_screened);
  h.Add(w.stats.nlp.n_solved);
  h.Add(w.stats.nlp.n_valid);
  h.Add(w.stats.nlp.rejects);
  h.Add(w.stats.nlp.chosen_index);
  h.Add(w.stats.nlp.chosen_phi);
  for (const Candidate& c : s.Candidates()) {
    h.Add(c.index);
    h.Add(c.n_pre);
    h.Add(c.reject);
    h.Add(c.rank);
    h.Add(c.q0);
    h.Add(c.qd0);
    h.Add(c.qdd0);
    h.Add(c.start);
    h.Add(c.start_index);
    h.Add(c.core_reason);
    h.Add(c.solved);
    h.Add(c.feasible);
    h.Add(c.converged);
    h.Add(c.iterations);
    h.Add(c.j_reference);
    h.Add(c.j_stop);
    h.Add(c.j_time);
    h.Add(c.j_switch);
    h.Add(c.phi);
    h.Add(c.worst_group);
    h.Add(c.worst_violation);
    h.Add(c.c_catch);
    // What the NEXT wake would start this candidate from.
    const SegmentSnapshot* m = s.RememberedSolution(c.index);
    h.Add(m != nullptr);
    if (m != nullptr) {
      rtc::testing::AddSegment(h, *m);
    }
  }
  const CatchSolution* sol = s.Solution();
  h.Add(sol != nullptr);
  if (sol != nullptr) {
    rtc::testing::AddSegment(h, sol->seg);
    h.Add(sol->cost_reference);
    h.Add(sol->cost_stop);
  }
  return h.Value();
}

struct OrderRun {
  std::uint64_t digest{0};
  int solved{0};
  std::string table;
  std::vector<NlpReject> rejects;
  std::vector<NlpCatchSearch::Start> starts;
  std::vector<int> n_pre;
};

// Wake 1 at kNow in rank order, then wake 2 at +41 ms with its solves in
// `order` (empty = rank order). At +41 ms the candidates at +440 and +480 ms
// are both on the 7-interval core, and only the first was solved by wake 1.
[[nodiscard]] OrderRun TwoWakes(std::span<const int> order, std::int64_t step2_ns,
                                double solve_budget_s) {
  auto rig = std::make_unique<Rig>();
  rig->params.solve_budget_s = solve_budget_s;
  std::string err;
  EXPECT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  const Wake w1 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  EXPECT_TRUE(w1.plan.valid);
  rig->search.SetEvaluationOrderForTesting(order);
  SetClockStep(step2_ns);
  const std::int64_t now = kNow + 41 * kMs;
  const Wake w2 = RunWake(*rig, ball, rig->RestingRt(now), NoSegments(), now);
  OrderRun r;
  r.digest = WakeDigest(rig->search, w2);
  r.solved = w2.stats.nlp.n_solved;
  r.table = Table(rig->search, w2.stats);
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.rank >= 0) {
      r.rejects.push_back(c.reject);
      r.starts.push_back(c.start);
      r.n_pre.push_back(c.n_pre);
    }
  }
  return r;
}

TEST(NlpCatchSearchOrder, PermutingTheSolvesChangesNothing) {
  struct Clock {
    const char* name;
    std::int64_t step_ns;
    double solve_budget_s;
    bool expect_deadline;
  };

  // 1 µs per read against a 4 ms share: no solve nears it. 100 µs per read
  // against a 0.4 ms share: a solve that needs more than four iterations runs
  // past it, one that restarts from its own solution does not — and the wake's
  // whole budget (40 ms) still holds every solve after the screening's reads.
  for (const Clock clock : {Clock{"no solve is cut", 1000, 0.004, false},
                            Clock{"some solves pass their deadline", 100'000, 0.0004, true}}) {
    SCOPED_TRACE(clock.name);
    const OrderRun base = TwoWakes({}, clock.step_ns, clock.solve_budget_s);
    const int n = base.solved;
    ASSERT_GE(n, 5) << base.table;
    // The wake is the one the test is about: two candidates on one core, one
    // of them with its own memory and one without, and — on the slow clock —
    // both kinds of outcome.
    bool shared = false;
    for (std::size_t i = 0; i < base.n_pre.size(); ++i) {
      for (std::size_t j = 0; j < base.n_pre.size(); ++j) {
        shared = shared || (base.n_pre[i] == base.n_pre[j] &&
                            base.starts[i] == NlpCatchSearch::Start::kSameCandidate &&
                            base.starts[j] == NlpCatchSearch::Start::kNeighbour);
      }
    }
    ASSERT_TRUE(shared) << base.table;
    const bool any_deadline =
        std::count(base.rejects.begin(), base.rejects.end(), NlpReject::kDeadline) > 0;
    const bool any_valid =
        std::count(base.rejects.begin(), base.rejects.end(), NlpReject::kNone) > 0;
    ASSERT_EQ(any_deadline, clock.expect_deadline) << base.table;
    ASSERT_TRUE(any_valid) << base.table;

    std::vector<int> order(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      order[static_cast<std::size_t>(i)] = i;
    }
    // Reversed, rotated, and neighbours swapped: every same-core pair is
    // solved in both orders somewhere in these.
    std::vector<std::vector<int>> orders;
    orders.emplace_back(order.rbegin(), order.rend());
    for (int shift : {1, n / 2}) {
      std::vector<int> o = order;
      std::rotate(o.begin(), o.begin() + shift, o.end());
      orders.push_back(o);
    }
    std::vector<int> swapped = order;
    for (std::size_t i = 0; i + 1 < swapped.size(); i += 2) {
      std::swap(swapped[i], swapped[i + 1]);
    }
    orders.push_back(swapped);
    for (const std::vector<int>& o : orders) {
      const OrderRun r = TwoWakes(o, clock.step_ns, clock.solve_budget_s);
      EXPECT_EQ(r.digest, base.digest)
          << "order starting " << o[0] << "," << o[1] << base.table << r.table;
    }
    // The seam is live: an order of another length is not applied, and the
    // same wake is what comes out.
    const std::array<int, 2> short_order{1, 0};
    EXPECT_EQ(TwoWakes(short_order, clock.step_ns, clock.solve_budget_s).digest, base.digest);
  }
}

TEST(NlpCatchSearchOrder, ANewTracksFirstWakeIsAFreshSearchsFirstWake) {
  // The cores' inputs and results outlive a wake: after two wakes on one ball
  // they hold a start trajectory, its flag, a catch-node target and a result.
  // A new track starts with no memory, so every solve of its first wake starts
  // from the IK pose — and must be the solve a search that has never run
  // makes. Anything of the earlier wakes still read from a shared buffer (a
  // start-trajectory flag left on, say) shows as a difference.
  const std::int64_t later = kNow + 200 * kMs;
  const auto first_wake_of_the_new_track = [&](bool used_before, std::string& table) {
    auto rig = std::make_unique<Rig>();
    std::string err;
    EXPECT_TRUE(rig->Configure(&err)) << err;
    if (used_before) {
      const Throw old_ball = OffAxisThrow(*rig, 0.20);
      const Wake w1 = RunWake(*rig, old_ball, rig->RestingRt(kNow), NoSegments(), kNow);
      EXPECT_TRUE(w1.plan.valid);
      const std::int64_t now = kNow + 41 * kMs;
      const Wake w2 = RunWake(*rig, old_ball, rig->RestingRt(now), NoSegments(), now);
      // The buffers were left by solves that started from memory.
      int from_memory = 0;
      for (const Candidate& c : rig->search.Candidates()) {
        from_memory += c.start == NlpCatchSearch::Start::kSameCandidate ||
                               c.start == NlpCatchSearch::Start::kNeighbour
                           ? 1
                           : 0;
      }
      EXPECT_GE(from_memory, 4) << Table(rig->search, w2.stats);
    }
    // Another ball: a new track generation, another line, a later instant.
    Throw ball = AxisThrow(*rig, 0.28, 0.8, 0.002, /*seq=*/1, /*gen=*/kTrack + 1);
    for (auto& s : ball.traj.s) {
      s.t_ns += 200 * kMs;
    }
    const Wake w = RunWake(*rig, ball, rig->RestingRt(later), NoSegments(), later);
    table = Table(rig->search, w.stats);
    EXPECT_TRUE(w.plan.valid) << table;
    EXPECT_EQ(rig->search.LatticeAnchorNs(), later);
    int solved = 0;
    for (const Candidate& c : rig->search.Candidates()) {
      if (c.solved) {
        ++solved;
        EXPECT_EQ(c.start, NlpCatchSearch::Start::kIkTarget) << c.index;
      }
    }
    EXPECT_GE(solved, 4) << table;
    return WakeDigest(rig->search, w);
  };
  std::string fresh_table;
  std::string used_table;
  const std::uint64_t fresh = first_wake_of_the_new_track(false, fresh_table);
  EXPECT_EQ(first_wake_of_the_new_track(true, used_table), fresh) << fresh_table << used_table;
}

// ── 4. Validity ──────────────────────────────────────────────────────────────

TEST(NlpCatchSearchValidity, ASolvePastItsShareIsNotAValidCandidate) {
  // The same wake on two clocks, with a share of 0.4 ms per solve. At 1 µs a
  // read no solve nears it and the cold solves are valid; at 100 µs a read
  // every solve of more than four iterations runs past its own share and is
  // refused for that alone — the wake's whole budget still holds them all.
  const auto build = [] {
    auto rig = std::make_unique<Rig>();
    rig->params.solve_budget_s = 0.0004;
    return rig;
  };
  auto fast_rig = build();
  auto slow_rig = build();
  std::string err;
  ASSERT_TRUE(fast_rig->Configure(&err)) << err;
  ASSERT_TRUE(slow_rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*fast_rig, 0.28);
  SetClockStep(1000);
  const Wake wf = RunWake(*fast_rig, ball, fast_rig->RestingRt(kNow), NoSegments(), kNow);
  SetClockStep(100'000);
  const Wake ws = RunWake(*slow_rig, ball, slow_rig->RestingRt(kNow), NoSegments(), kNow);
  ASSERT_TRUE(wf.plan.valid) << Table(fast_rig->search, wf.stats);
  ASSERT_EQ(wf.stats.nlp.n_solved, ws.stats.nlp.n_solved) << Table(slow_rig->search, ws.stats);
  int cut = 0;
  for (const Candidate& f : fast_rig->search.Candidates()) {
    const Candidate* s = Find(slow_rig->search, f.index);
    ASSERT_NE(s, nullptr);
    if (f.reject != NlpReject::kNone) {
      continue;
    }
    // More than four iterations is more than four reads of a 100 µs clock
    // against a 0.4 ms share.
    if (f.iterations > 4) {
      ++cut;
      EXPECT_EQ(s->reject, NlpReject::kDeadline) << f.index << Table(slow_rig->search, ws.stats);
      EXPECT_EQ(s->core_reason, MpcDockingReason::kDeadline);
      EXPECT_GE(s->solve_ns, 400'000);
      EXPECT_TRUE(s->solved);
    } else {
      EXPECT_EQ(s->reject, NlpReject::kNone) << f.index << Table(slow_rig->search, ws.stats);
      EXPECT_TRUE(BitsEqual(s->phi, f.phi));
    }
  }
  ASSERT_GT(cut, 0) << "no valid solve of this wake is long enough to be cut"
                    << Table(fast_rig->search, wf.stats);
  // A cut candidate is not chosen even when it was the minimum.
  const Candidate* fast_best = Find(fast_rig->search, wf.stats.nlp.chosen_index);
  ASSERT_NE(fast_best, nullptr);
  if (fast_best->iterations > 4) {
    EXPECT_NE(ws.stats.nlp.chosen_index, wf.stats.nlp.chosen_index);
  }
  EXPECT_GT(ws.stats.nlp.solve_ns_max, 400'000);
  EXPECT_LE(wf.stats.nlp.solve_ns_max, 400'000);
}

// Advances the clock once, inside the first solve a wake runs.
struct ClockJump {
  std::int64_t jump_ns{0};
  bool done{false};
};

void JumpInTheFirstSolve(MpcDockingStage stage, bool begin, void* user) noexcept {
  auto* j = static_cast<ClockJump*>(user);
  if (!j->done && begin && stage == MpcDockingStage::kStart) {
    j->done = true;
    g_clock.fetch_add(j->jump_ns, std::memory_order_relaxed);
  }
}

TEST(NlpCatchSearchValidity, ACandidateTheRtCanNoLongerReadFromItsStartIsNotValid) {
  // t_0 stands the wake's budget after `now`, and a candidate's node 0 its
  // `wait` after t_0. The first solve of the wake takes `jump` — far over its
  // share, as a QP in progress can (the core reads its deadline between
  // iterations only) — so the wake ends `jump − budget` LATE. Every other
  // solve is inside its own share and is exactly the solve of an undisturbed
  // wake; but a segment whose node 0 has less wait than the wake is late by
  // reaches the RT after the instant it starts at.
  const std::int64_t budget_ns = Ns(Rig().params.budget_s);
  const std::int64_t share_ns = Ns(Rig().params.solve_budget_s);
  const auto run = [](Rig& rig, ClockJump& jump) {
    std::string err;
    EXPECT_TRUE(rig.Configure(&err)) << err;
    rig.search.SetCoreStageHookForTesting(&JumpInTheFirstSolve, &jump);
    const Throw ball = AxisThrow(rig, 0.28);
    return RunWake(rig, ball, rig.RestingRt(kNow), NoSegments(), kNow);
  };
  auto base = std::make_unique<Rig>();
  ClockJump none{0, false};
  const Wake wb = run(*base, none);
  ASSERT_TRUE(wb.plan.valid) << Table(base->search, wb.stats);
  // The undisturbed wake is far inside one millisecond of this clock.
  ASSERT_LT(wb.stats.search_ns, kMs);

  // ── 30 ms late: the candidates with less wait than that are lost ──
  {
    const std::int64_t late_ns = 30 * kMs;
    auto rig = std::make_unique<Rig>();
    ClockJump jump{budget_ns + late_ns, false};
    const Wake w = run(*rig, jump);
    ASSERT_TRUE(jump.done);
    ASSERT_EQ(w.stats.nlp.n_solved, wb.stats.nlp.n_solved) << Table(rig->search, w.stats);
    int lost = 0;
    int kept = 0;
    const Candidate* best = nullptr;
    for (const Candidate& f : base->search.Candidates()) {
      const Candidate* s = Find(rig->search, f.index);
      ASSERT_NE(s, nullptr);
      ASSERT_EQ(s->wait_ns, f.wait_ns);
      if (f.reject != NlpReject::kNone) {
        EXPECT_FALSE(s->late) << f.index;
        continue;
      }
      if (f.rank == 0) {
        // The solve the time went into: past its own share.
        EXPECT_EQ(s->reject, NlpReject::kDeadline) << Table(rig->search, w.stats);
        EXPECT_FALSE(s->late);
        EXPECT_GT(s->solve_ns, share_ns);
        continue;
      }
      // Its own solve is the undisturbed one, inside its share.
      EXPECT_LE(s->solve_ns, share_ns) << f.index;
      EXPECT_TRUE(s->feasible && s->converged) << f.index;
      EXPECT_TRUE(BitsEqual(s->phi, f.phi)) << f.index;
      ASSERT_TRUE(f.wait_ns <= late_ns || f.wait_ns >= late_ns + kMs)
          << "a wait within the wake's own duration of the lateness: " << f.wait_ns;
      if (f.wait_ns <= late_ns) {
        ++lost;
        EXPECT_EQ(s->reject, NlpReject::kDeadline) << f.index << Table(rig->search, w.stats);
        EXPECT_TRUE(s->late) << f.index;
        // What it ended on is still a start for the next wake.
        EXPECT_NE(rig->search.RememberedSolution(f.index), nullptr) << f.index;
      } else {
        ++kept;
        EXPECT_EQ(s->reject, NlpReject::kNone) << f.index << Table(rig->search, w.stats);
        EXPECT_FALSE(s->late) << f.index;
        if (best == nullptr || s->phi < best->phi) {
          best = s;  // lattice order: the smaller index stays on a tie
        }
      }
    }
    ASSERT_GT(lost, 0) << "no valid candidate starts early enough to be lost"
                       << Table(base->search, wb.stats);
    ASSERT_GT(kept, 0) << "no valid candidate starts late enough to be kept"
                       << Table(base->search, wb.stats);
    EXPECT_EQ(w.stats.nlp.n_valid, kept);
    ASSERT_TRUE(w.plan.valid) << Table(rig->search, w.stats);
    ASSERT_NE(best, nullptr);
    EXPECT_EQ(w.stats.nlp.chosen_index, best->index);
    ASSERT_NE(rig->search.Solution(), nullptr);
    EXPECT_GE(rig->search.Solution()->seg.t0_ns - rig->search.LastStartInstantNs(), late_ns + kMs);
  }

  // ── 60 ms late: later than any node 0 (the waits are under one 50 ms interval) ──
  {
    auto rig = std::make_unique<Rig>();
    ClockJump jump{budget_ns + 60 * kMs, false};
    const Wake w = run(*rig, jump);
    EXPECT_FALSE(w.plan.valid) << Table(rig->search, w.stats);
    EXPECT_EQ(w.stats.nlp.reason, NlpReject::kDeadline);
    EXPECT_EQ(w.plan.reason, PlanReason::kBudgetExceeded);
    EXPECT_EQ(w.stats.nlp.n_valid, 0);
    EXPECT_EQ(rig->search.Solution(), nullptr);
    int late = 0;
    for (const Candidate& c : rig->search.Candidates()) {
      late += c.late ? 1 : 0;
      EXPECT_NE(c.reject, NlpReject::kNone) << c.index;
    }
    EXPECT_GE(late, 3) << Table(rig->search, w.stats);
  }
}

TEST(NlpCatchSearchValidity, AChanceRowViolationIsNotChosenThoughItsCostIsTheLowest) {
  // The covariance is wide (laterally) at the two samples around the best
  // candidate's catch instant and narrow everywhere else: that candidate's
  // solution breaks the lateral chance rows and nothing else.
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  Throw ball = AxisThrow(*rig, 0.28);
  const Wake clean = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  ASSERT_TRUE(clean.plan.valid);
  const std::int64_t best = clean.stats.nlp.chosen_index;
  ASSERT_EQ(best, IndexAt(280));
  const double best_phi = clean.stats.nlp.chosen_phi;

  const CovarianceSnapshot wide = LateralCovariance(ball, 0.03, 0.001, 0.002);
  ball.cov = LateralCovariance(ball, 0.002, 0.001, 0.002);
  // +280 ms is nearest the sample at +300 ms (index 6); the candidates at
  // +240 and +320 ms are nearest +250 and +300/+350.
  ball.cov.c[6] = wide.c[6];
  rig->search.ResetTrial();
  const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  const Candidate* c = Find(rig->search, best);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->reject, NlpReject::kChance) << Table(rig->search, w.stats);
  EXPECT_TRUE(c->solved);
  EXPECT_FALSE(c->feasible);
  EXPECT_TRUE(c->worst_group == DockingRowGroup::kLateral ||
              c->worst_group == DockingRowGroup::kTiming ||
              c->worst_group == DockingRowGroup::kVelocitySet)
      << rtc::catching::DockingRowGroupName(c->worst_group);
  EXPECT_GT(c->worst_violation, rig->params.core.tol_violation);
  // Its reference cost is still the lowest of the wake's...
  ASSERT_TRUE(w.plan.valid) << Table(rig->search, w.stats);
  const Candidate* chosen = Find(rig->search, w.stats.nlp.chosen_index);
  ASSERT_NE(chosen, nullptr);
  EXPECT_NE(chosen->index, best);
  EXPECT_LT(c->j_reference + c->j_time + c->j_switch, chosen->phi);
  EXPECT_LT(best_phi, chosen->phi);
  // ...and the candidates beside it, whose covariance is another sample's,
  // are untouched.
  EXPECT_GE(w.stats.nlp.n_valid, 3) << Table(rig->search, w.stats);
}

TEST(NlpCatchSearchValidity, AFeasibleIterateThatDidNotConvergeIsNotChosen) {
  // Two SQP iterations: a candidate that needs more ends on a point where the
  // hard rows may hold, and it is still no solution.
  auto rig = std::make_unique<Rig>();
  rig->params.core.max_iterations = 2;
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  int unconverged = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.reject == NlpReject::kUnconverged) {
      ++unconverged;
      EXPECT_TRUE(c.solved);
      EXPECT_TRUE(c.feasible) << c.index;
      EXPECT_FALSE(c.converged);
      EXPECT_EQ(c.core_reason, MpcDockingReason::kIterationLimit);
      EXPECT_LE(c.worst_violation, rig->params.core.tol_violation);
      EXPECT_NE(c.index, w.stats.nlp.chosen_index);
    }
    if (c.reject == NlpReject::kNone) {
      EXPECT_TRUE(c.converged);
    }
  }
  EXPECT_GT(unconverged, 0) << Table(rig->search, w.stats);
  // With room to converge the same candidates are valid.
  auto roomy = std::make_unique<Rig>();
  ASSERT_TRUE(roomy->Configure(&err)) << err;
  const Wake wr = RunWake(*roomy, ball, roomy->RestingRt(kNow), NoSegments(), kNow);
  EXPECT_EQ(Count(roomy->search, NlpReject::kUnconverged), 0);
  EXPECT_GT(wr.stats.nlp.n_valid, w.stats.nlp.n_valid);
}

// ── 5. Reasons ───────────────────────────────────────────────────────────────

TEST(NlpCatchSearchReasons, ThePublishedReasonIsATableOverEveryValue) {
  const std::array<std::pair<NlpReject, PlanReason>, kNlpRejectCount> table{{
      {NlpReject::kNone, PlanReason::kNone},
      {NlpReject::kLeadShort, PlanReason::kHorizonShort},
      {NlpReject::kBallInvalid, PlanReason::kInputNonFinite},
      {NlpReject::kWorkspace, PlanReason::kStoppingDistance},
      {NlpReject::kCovariance, PlanReason::kUncertainty},
      {NlpReject::kNoSource, PlanReason::kLimitsInvalid},
      {NlpReject::kIk, PlanReason::kIkFailed},
      {NlpReject::kManipulability, PlanReason::kManipulability},
      {NlpReject::kReach, PlanReason::kReachTime},
      {NlpReject::kSpeedWindow, PlanReason::kGammaWindow},
      {NlpReject::kNotRanked, PlanReason::kBudgetExceeded},
      {NlpReject::kDeadline, PlanReason::kBudgetExceeded},
      {NlpReject::kSolverRejected, PlanReason::kLimitsInvalid},
      {NlpReject::kHardRow, PlanReason::kRollout},
      {NlpReject::kChance, PlanReason::kUncertainty},
      {NlpReject::kUnconverged, PlanReason::kRollout},
      {NlpReject::kNoCandidate, PlanReason::kHorizonShort},
      {NlpReject::kNotAtRest, PlanReason::kLimitsInvalid},
      {NlpReject::kRtInvalid, PlanReason::kLimitsInvalid},
  }};
  std::set<std::string> names;
  for (std::size_t i = 0; i < table.size(); ++i) {
    // The table is in enum order and has no hole.
    EXPECT_EQ(static_cast<std::size_t>(table[i].first), i);
    EXPECT_EQ(NlpPlanReason(table[i].first), table[i].second) << NlpRejectName(table[i].first);
    names.insert(NlpRejectName(table[i].first));
  }
  EXPECT_EQ(names.size(), kNlpRejectCount) << "two reasons share a name";
  EXPECT_FALSE(names.contains("unknown"));
}

// One wake of a rig the caller edited, on a throw the caller edited.
struct ReasonCase {
  std::unique_ptr<Rig> rig = std::make_unique<Rig>();
  Throw ball;
  Wake w;

  template <typename EditRig, typename EditThrow>
  ReasonCase(EditRig&& edit_rig, EditThrow&& edit_throw, bool cov_matched = true) {
    edit_rig(*rig);
    std::string err;
    EXPECT_TRUE(rig->Configure(&err)) << err;
    ball = AxisThrow(*rig, 0.28);
    edit_throw(*rig, ball);
    w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow, cov_matched);
  }
};

const auto kNoEdit = [](auto&&...) {};

// A wake chose nothing: its reason, what is published for it, and that the
// reason is the farthest any candidate got.
void ExpectNoPlan(const ReasonCase& rc, NlpReject why) {
  EXPECT_FALSE(rc.w.plan.valid) << Table(rc.rig->search, rc.w.stats);
  EXPECT_EQ(rc.w.stats.nlp.reason, why) << Table(rc.rig->search, rc.w.stats);
  EXPECT_EQ(rc.w.plan.reason, NlpPlanReason(why));
  EXPECT_EQ(rc.rig->search.Solution(), nullptr);
  // The RT follows nothing: "no plan" is published.
  EXPECT_TRUE(rc.w.stats.publish);
  EXPECT_EQ(rc.w.stats.decision, SwitchDecision::kNoCurrent);
  for (const Candidate& c : rc.rig->search.Candidates()) {
    EXPECT_LE(static_cast<int>(c.reject), static_cast<int>(why)) << c.index;
  }
}

TEST(NlpCatchSearchReasons, EveryScreeningReasonComesFromAnInputThatPassedTheChecksBeforeIt) {
  {
    SCOPED_TRACE("lead_short: every lattice instant of the horizon is nearer than the lead");
    const ReasonCase rc([](Rig& r) { r.params.t_lead_min = 0.4; }, kNoEdit);
    // Only the instant at exactly t_0 + 0.4 s could pass; none is there.
    ExpectNoPlan(rc, NlpReject::kLeadShort);
    EXPECT_GE(rc.w.stats.nlp.n_lattice, 9);
    EXPECT_EQ(rc.w.stats.n_in_window, 0);
    EXPECT_EQ(rc.w.stats.n_ik, 0);
  }
  {
    SCOPED_TRACE("ball_invalid: the prediction ends before the candidates' instants");
    const ReasonCase rc(kNoEdit, [](const Rig&, Throw& t) {
      t.traj.n = 3;  // samples at +0, +50, +100 ms; t_0 is at +44 ms
      t.cov.n = 3;
    });
    ExpectNoPlan(rc, NlpReject::kBallInvalid);
    EXPECT_GT(rc.w.stats.n_in_window, 0);
    EXPECT_EQ(Count(rc.rig->search, NlpReject::kBallInvalid), rc.w.stats.n_in_window);
    EXPECT_EQ(rc.w.stats.n_ik, 0);
  }
  {
    SCOPED_TRACE("ball_invalid: a ball that does not move has no approach axis");
    const ReasonCase rc(kNoEdit, [](const Rig&, Throw& t) {
      for (auto& s : t.traj.s) {
        s.v = {0.0, 0.0, 0.0};
        s.p = t.traj.s[0].p;
      }
    });
    ExpectNoPlan(rc, NlpReject::kBallInvalid);
    EXPECT_GT(rc.w.stats.n_in_window, 0);
  }
  {
    SCOPED_TRACE("workspace: the catch box holds no point of the ball's line");
    const ReasonCase rc(
        [](Rig& r) {
          r.params.catch_box.min = {40.0, 40.0, 40.0};
          r.params.catch_box.max = {41.0, 41.0, 41.0};
        },
        kNoEdit);
    ExpectNoPlan(rc, NlpReject::kWorkspace);
    EXPECT_EQ(Count(rc.rig->search, NlpReject::kWorkspace), rc.w.stats.n_in_window);
    EXPECT_EQ(rc.w.stats.n_ik, 0);
  }
  {
    SCOPED_TRACE("covariance: the covariance in the box is another snapshot's");
    const ReasonCase rc(kNoEdit, kNoEdit, /*cov_matched=*/false);
    ExpectNoPlan(rc, NlpReject::kCovariance);
    EXPECT_EQ(Count(rc.rig->search, NlpReject::kCovariance), rc.w.stats.n_in_window);
    EXPECT_EQ(rc.w.stats.n_ik, 0);
    // Without chance rows the same wake needs no covariance.
    const ReasonCase exact([](Rig& r) { r.params.core.chance = false; }, kNoEdit, false);
    EXPECT_TRUE(exact.w.plan.valid) << Table(exact.rig->search, exact.w.stats);
    EXPECT_EQ(exact.w.plan.sigma_c, 0.0);
  }
  {
    SCOPED_TRACE("ik: the ball passes where the arm cannot put the frame");
    const ReasonCase rc(kNoEdit, [](const Rig&, Throw& t) {
      for (auto& s : t.traj.s) {
        s.p[0] += 3.0;  // metres away from the arm
      }
    });
    ExpectNoPlan(rc, NlpReject::kIk);
    EXPECT_EQ(Count(rc.rig->search, NlpReject::kIk), rc.w.stats.n_in_window);
    EXPECT_EQ(rc.w.stats.n_ik, rc.w.stats.n_in_window);
  }
  {
    SCOPED_TRACE("manipulability: the catchability gate refuses every pose");
    const ReasonCase rc([](Rig& r) { r.ik.manipulability_min = 1e6; }, kNoEdit);
    ExpectNoPlan(rc, NlpReject::kManipulability);
    for (const Candidate& c : rc.rig->search.Candidates()) {
      if (c.reject == NlpReject::kManipulability) {
        EXPECT_EQ(c.ik_reason, CatchPoseReason::kBelowManipMin);
      }
    }
  }
  {
    SCOPED_TRACE("reach: no joint may move faster than a crawl");
    const ReasonCase rc([](Rig& r) { r.params.limits.qd_max = Eigen::VectorXd::Constant(6, 1e-3); },
                        [](const Rig& r, Throw& t) { t = AxisThrow(r, 0.10); });
    ExpectNoPlan(rc, NlpReject::kReach);
    for (const Candidate& c : rc.rig->search.Candidates()) {
      if (c.reject == NlpReject::kReach) {
        EXPECT_EQ(c.ik_reason, CatchPoseReason::kNone);
        EXPECT_EQ(c.reach_limit, NlpReachLimit::kVelocity);
      }
    }
    EXPECT_EQ(Count(rc.rig->search, NlpReject::kReach), rc.w.stats.n_in_window);
  }
  {
    SCOPED_TRACE("speed_window: the impact limit is below the slowest allowed catch");
    const ReasonCase rc([](Rig& r) { r.params.core.e_max = 1e-6; }, kNoEdit);
    ExpectNoPlan(rc, NlpReject::kSpeedWindow);
    int n = 0;
    for (const Candidate& c : rc.rig->search.Candidates()) {
      if (c.reject == NlpReject::kSpeedWindow) {
        ++n;
        EXPECT_EQ(c.reach_limit, NlpReachLimit::kNone);
        EXPECT_GT(c.speed_lo, c.speed_hi);
      }
    }
    EXPECT_GT(n, 0);
  }
}

TEST(NlpCatchSearchReasons, EverySolveStageReasonComesFromAnInputThatReachedTheSolve) {
  {
    SCOPED_TRACE("not_ranked: the budget holds two solves");
    const ReasonCase rc([](Rig& r) { r.params.max_solves = 2; }, kNoEdit);
    EXPECT_TRUE(rc.w.plan.valid);
    EXPECT_EQ(rc.w.stats.nlp.n_solved, 2);
    // `max_solves` is a cap the wake was configured with, not the budget
    // running out: the flag stays for the time budget (the next case).
    EXPECT_FALSE(rc.w.stats.budget_hit);
    EXPECT_EQ(Count(rc.rig->search, NlpReject::kNotRanked), rc.w.stats.nlp.n_screened - 2);
    for (const Candidate& c : rc.rig->search.Candidates()) {
      EXPECT_EQ(c.reject == NlpReject::kNotRanked, c.rank >= 2) << c.index;
      if (c.rank >= 0 && c.rank < 2) {
        EXPECT_TRUE(c.solved);
      }
    }
    // The rank is J_time + J_switch + the proxy, best first: here the squared
    // joint distance of the IK pose from the start.
    const auto all = rc.rig->search.Candidates();
    for (const Candidate& a : all) {
      for (const Candidate& b : all) {
        if (a.rank >= 0 && b.rank >= 0 && a.rank < b.rank) {
          EXPECT_LE(a.rank_key, b.rank_key);
        }
      }
      if (a.rank >= 0) {
        double dq_sq = 0.0;
        for (std::size_t m = 0; m < 6; ++m) {
          dq_sq += (a.q_ik[m] - a.q0[m]) * (a.q_ik[m] - a.q0[m]);
        }
        EXPECT_DOUBLE_EQ(a.rank_key, dq_sq) << a.index;
      }
    }
  }
  {
    SCOPED_TRACE("not_ranked: the screening used the whole budget");
    auto rig = std::make_unique<Rig>();
    std::string err;
    ASSERT_TRUE(rig->Configure(&err)) << err;
    const Throw ball = AxisThrow(*rig, 0.28);
    SetClockStep(5 * kMs);  // sixteen IK timings alone are 80 ms of a 40 ms budget
    const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
    EXPECT_FALSE(w.plan.valid);
    EXPECT_EQ(w.stats.nlp.reason, NlpReject::kNotRanked) << Table(rig->search, w.stats);
    EXPECT_EQ(w.plan.reason, PlanReason::kBudgetExceeded);
    EXPECT_EQ(w.stats.nlp.n_solved, 0);
    EXPECT_GT(w.stats.nlp.n_screened, 0);
    EXPECT_TRUE(w.stats.budget_hit);
    EXPECT_GT(w.stats.nlp.screen_ns, 40 * kMs);
  }
  {
    SCOPED_TRACE("hard_row: the torque rows allow less than gravity needs");
    const ReasonCase rc(
        [](Rig& r) {
          r.params.limits.tau_lo = Eigen::VectorXd::Constant(6, -1e-3);
          r.params.limits.tau_hi = Eigen::VectorXd::Constant(6, 1e-3);
        },
        kNoEdit);
    ExpectNoPlan(rc, NlpReject::kHardRow);
    int n = 0;
    for (const Candidate& c : rc.rig->search.Candidates()) {
      if (c.reject == NlpReject::kHardRow) {
        ++n;
        EXPECT_TRUE(c.solved);
        EXPECT_FALSE(c.feasible);
        EXPECT_EQ(c.worst_group, DockingRowGroup::kTorque);
        EXPECT_GT(c.worst_violation, rc.rig->params.core.tol_violation);
      }
    }
    EXPECT_GT(n, 0);
  }
  {
    SCOPED_TRACE("no_candidate: the lattice is coarser than the horizon");
    const ReasonCase rc(
        [](Rig& r) {
          r.params.cand_dt = 1.0;  // instants at kNow and kNow + 1 s; the window ends at +444 ms
        },
        kNoEdit);
    ExpectNoPlan(rc, NlpReject::kNoCandidate);
    EXPECT_EQ(rc.w.stats.nlp.n_lattice, 0);
    EXPECT_TRUE(rc.rig->search.Candidates().empty());
  }
}

TEST(NlpCatchSearchReasons, AReportTheSearchCannotPlanFromIsItsOwnReason) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  const auto expect = [&](const PlannerRtState& rt, NlpReject why, const char* what) {
    SCOPED_TRACE(what);
    const Wake w = RunWake(*rig, ball, rt, NoSegments(), kNow);
    EXPECT_FALSE(w.plan.valid);
    EXPECT_EQ(w.stats.nlp.reason, why);
    EXPECT_EQ(w.plan.reason, PlanReason::kLimitsInvalid);
    EXPECT_EQ(w.stats.nlp.n_lattice, 0) << "the wake went on to look at candidates";
    EXPECT_TRUE(rig->search.Candidates().empty());
    EXPECT_EQ(rig->search.Solution(), nullptr);
  };
  const PlannerRtState good = rig->RestingRt(kNow);
  ASSERT_TRUE(RunWake(*rig, ball, good, NoSegments(), kNow).plan.valid);
  PlannerRtState rt = good;
  rt.valid = false;
  expect(rt, NlpReject::kRtInvalid, "no report yet");
  rt = good;
  rt.nv = 5;
  expect(rt, NlpReject::kRtInvalid, "another arm's width");
  rt = good;
  rt.rt_state_ns = kNow - 51 * kMs;
  expect(rt, NlpReject::kRtInvalid, "a report older than the bound");
  rt = good;
  rt.rt_state_ns = kNow + 1;
  expect(rt, NlpReject::kRtInvalid, "a report from the future");
  rt = good;
  rt.q_cmd[3] = std::numeric_limits<double>::quiet_NaN();
  expect(rt, NlpReject::kRtInvalid, "a NaN command");
  rt = good;
  rt.qd_cmd[1] = std::numeric_limits<double>::infinity();
  expect(rt, NlpReject::kRtInvalid, "an infinite command velocity");
  // Inside the bounds: exactly the oldest report allowed.
  rt = good;
  rt.rt_state_ns = kNow - 50 * kMs;
  EXPECT_TRUE(RunWake(*rig, ball, rt, NoSegments(), kNow).plan.valid);

  // A command that moves is no start for a first plan.
  rt = good;
  rt.qd_cmd[2] = 2e-3;  // rest_tol is 1e-3
  expect(rt, NlpReject::kNotAtRest, "the command moves and no plan is followed");
  rt.qd_cmd[2] = 1e-3;
  EXPECT_TRUE(RunWake(*rig, ball, rt, NoSegments(), kNow).plan.valid);
}

// A reported segment that is at rest on the wait pose up to `jump_ns` and from
// there on has one joint a hair under its upper limit, moving up at 0.9 of its
// speed limit: a state inside the box that no trajectory of the box can leave.
[[nodiscard]] SegmentSnapshot SegmentThatBecomesUnstoppable(const Rig& rig, std::int64_t jump_ns,
                                                            int joint) {
  SegmentSnapshot seg{};
  seg.valid = true;
  seg.segment_seq = 21;
  seg.plan_id = 4;
  seg.nv = 6;
  seg.n_pre = 4;
  seg.n_nodes = 11;
  seg.dt_pre_ns = 50 * kMs;
  seg.dt_ns = 50 * kMs;
  seg.k0 = 0;
  seg.t0_ns = jump_ns - 2 * seg.dt_pre_ns;  // the jump is on node 2
  seg.t_c_ns = seg.t0_ns + seg.n_pre * seg.dt_pre_ns;
  const std::array<double, 6> wait = ToDevice(rig.q_wait);
  const auto dev = static_cast<std::size_t>(kDeviceOfModel[static_cast<std::size_t>(joint)]);
  for (int k = 0; k <= seg.n_nodes; ++k) {
    for (std::size_t d = 0; d < 6; ++d) {
      seg.q[static_cast<std::size_t>(k) * kMaxSegmentNv + d] = wait[d];
    }
    if (k >= 2) {
      const auto e = static_cast<std::size_t>(k) * kMaxSegmentNv + dev;
      seg.q[e] = rig.params.limits.q_max[joint] - 1e-4;
      seg.qd[e] = 0.9 * rig.params.limits.qd_max[joint];
    }
  }
  return seg;
}

TEST(NlpCatchSearchReasons, ASolveRefusedBeforeAnyIterateIsItsOwnReasonAndLeavesNothingBehind) {
  // Two candidates in a row share a core; node 0 of the first is before the
  // reported segment's jump and node 0 of the second after it. The second's
  // start cannot be stopped inside the box: the core refuses it before any
  // iterate exists, and its RESULT buffer still holds the first one's
  // trajectory. Whichever of the two is solved first, the wake is the same.
  constexpr int kJoint = 1;
  const auto run = [&](std::span<const int> order, std::string& table) {
    auto rig = std::make_unique<Rig>();
    // The box's upper face on that joint, just above the wait pose.
    rig->params.limits.q_max[kJoint] = rig->q_wait[kJoint] + 0.02;
    std::string err;
    EXPECT_TRUE(rig->Configure(&err)) << err;
    const Throw ball = AxisThrow(*rig, 0.28);
    // The lattice is anchored at kNow by a first wake.
    const Wake first = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
    EXPECT_TRUE(first.plan.valid) << Table(rig->search, first.stats);
    // What each candidate is remembered by before the wake under test.
    const auto remembered = [&rig](std::int64_t index) -> std::uint64_t {
      const SegmentSnapshot* seg = rig->search.RememberedSolution(index);
      rtc::testing::ValueDigest h;
      h.Add(seg != nullptr);
      if (seg != nullptr) {
        rtc::testing::AddSegment(h, *seg);
      }
      return h.Value();
    };
    std::vector<std::pair<std::int64_t, std::uint64_t>> before;
    for (const Candidate& c : rig->search.Candidates()) {
      before.emplace_back(c.index, remembered(c.index));
    }
    rig->search.SetEvaluationOrderForTesting(order);
    // The jump 25 ms after t_0: node 0 of a candidate that waits less is
    // before it.
    const auto arm =
        Following(SegmentThatBecomesUnstoppable(*rig, rig->StartInstant(kNow) + 25 * kMs, kJoint));
    const Wake w = RunWake(*rig, ball, rig->FollowingRt(kNow, kNow + 280 * kMs), *arm, kNow);
    table = Table(rig->search, w.stats);
    int refused = 0;
    int solved = 0;
    bool shared = false;
    for (const Candidate& c : rig->search.Candidates()) {
      if (c.rank < 0) {
        continue;
      }
      const bool after_jump = c.wait_ns >= 25 * kMs;
      if (after_jump) {
        ++refused;
        EXPECT_EQ(c.reject, NlpReject::kSolverRejected) << c.index << table;
        EXPECT_FALSE(c.solved);
        EXPECT_EQ(c.core_reason, MpcDockingReason::kLinearInfeasible)
            << MpcDockingReasonName(c.core_reason);
        // Nothing of another candidate's solve is reported as its own.
        EXPECT_EQ(c.iterations, 0);
        EXPECT_EQ(c.j_reference, 0.0);
        EXPECT_FALSE(c.feasible);
        EXPECT_FALSE(c.converged);
        // ...and nothing of this wake is remembered for it: what the first
        // wake left is still there, untouched.
        for (const auto& [index, digest] : before) {
          if (index == c.index) {
            EXPECT_EQ(remembered(index), digest) << index;
          }
        }
        // It passed everything before the solve.
        EXPECT_EQ(c.reach_limit, NlpReachLimit::kNone);
        EXPECT_EQ(c.source_seq, 21U);
      } else {
        ++solved;
        EXPECT_TRUE(c.solved) << c.index << table;
        EXPECT_NE(c.reject, NlpReject::kSolverRejected);
      }
      for (const Candidate& other : rig->search.Candidates()) {
        shared = shared || (other.rank >= 0 && other.n_pre == c.n_pre &&
                            (other.wait_ns >= 25 * kMs) != after_jump);
      }
    }
    EXPECT_GT(refused, 0) << table;
    EXPECT_GT(solved, 0) << table;
    EXPECT_TRUE(shared) << "no core has both a refused and a solved candidate" << table;
    EXPECT_EQ(w.plan.valid, w.stats.nlp.n_valid > 0);
    return std::make_pair(WakeDigest(rig->search, w), static_cast<int>(w.stats.nlp.n_solved));
  };
  std::string table;
  const auto base = run({}, table);
  const int n = base.second;
  ASSERT_GE(n, 4) << table;
  std::vector<int> reversed(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    reversed[static_cast<std::size_t>(i)] = n - 1 - i;
  }
  std::string reversed_table;
  EXPECT_EQ(run(reversed, reversed_table).first, base.first) << table << reversed_table;
}

TEST(NlpCatchSearchReasons, AnIterateTheSolverFailedOnIsTheSolversRefusalAndIsForgotten) {
  // The timing row is off, so that the screening's closing-speed window does
  // not read the covariance and the solves do.
  auto rig = std::make_unique<Rig>();
  rig->params.core.timing_row = false;
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  const Wake w1 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  ASSERT_TRUE(w1.plan.valid) << Table(rig->search, w1.stats);
  // The same throw under a covariance that is finite and positive — and whose
  // chance rows are numbers no QP is solved with. The core returns the iterate
  // it stood on with the SOLVER's reason; its row values say nothing.
  Throw broken = ball;
  for (int k = 0; k < broken.cov.n; ++k) {
    auto& e = broken.cov.c[static_cast<std::size_t>(k)];
    e.fill(0.0);
    for (std::size_t r = 0; r < 6; ++r) {
      e[r * 6 + r] = 1e100;
    }
  }
  const Wake w2 = RunWake(*rig, broken, rig->RestingRt(kNow), NoSegments(), kNow);
  EXPECT_FALSE(w2.plan.valid) << Table(rig->search, w2.stats);
  EXPECT_EQ(w2.stats.nlp.reason, NlpReject::kSolverRejected) << Table(rig->search, w2.stats);
  EXPECT_EQ(w2.plan.reason, PlanReason::kLimitsInvalid);
  EXPECT_EQ(rig->search.Solution(), nullptr);
  int failed = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.rank < 0) {
      continue;
    }
    ++failed;
    EXPECT_TRUE(c.solved) << c.index;  // an iterate came back
    EXPECT_TRUE(c.core_reason == MpcDockingReason::kQpFailed ||
                c.core_reason == MpcDockingReason::kSolutionNonFinite)
        << c.index << " " << MpcDockingReasonName(c.core_reason);
    // Not a row's refusal, though the iterate's chance rows read as violated.
    EXPECT_EQ(c.reject, NlpReject::kSolverRejected) << c.index;
    EXPECT_FALSE(c.feasible);
    // It started from the first wake's solution — and that is dropped.
    EXPECT_EQ(c.start, NlpCatchSearch::Start::kSameCandidate) << c.index;
    EXPECT_EQ(rig->search.RememberedSolution(c.index), nullptr) << c.index;
  }
  ASSERT_GE(failed, 4) << Table(rig->search, w2.stats);
  EXPECT_EQ(Count(rig->search, NlpReject::kChance) + Count(rig->search, NlpReject::kHardRow), 0);
  // The next wake starts each of them from the IK pose, as a first wake does,
  // and ends where the first wake ended.
  const Wake w3 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  ASSERT_TRUE(w3.plan.valid) << Table(rig->search, w3.stats);
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.solved) {
      EXPECT_EQ(c.start, NlpCatchSearch::Start::kIkTarget) << c.index;
    }
  }
  EXPECT_EQ(w3.stats.nlp.chosen_index, w1.stats.nlp.chosen_index);
  EXPECT_TRUE(BitsEqual(w3.stats.nlp.chosen_phi, w1.stats.nlp.chosen_phi));
}

// ── 6. The wakes after the RT follows a plan ─────────────────────────────────

TEST(NlpCatchSearchFollowing, WithoutAReportedSegmentThereIsNothingToStartFrom) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const auto adopted = AdoptFirst(*rig, 0.20);
  ASSERT_TRUE(adopted->first.plan.valid);
  const std::int64_t now = kNow + 33 * kMs;
  // The RT follows a plan and the planner holds no segment of it.
  const Wake w =
      RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), NoSegments(), now);
  EXPECT_FALSE(w.plan.valid);
  EXPECT_EQ(w.stats.nlp.reason, NlpReject::kNoSource);
  // Nothing is published: the RT keeps what it follows.
  EXPECT_FALSE(w.stats.publish);
  EXPECT_EQ(w.stats.decision, SwitchDecision::kHeldNoCandidate);
  EXPECT_EQ(rig->search.Solution(), nullptr);

  // A PENDING segment alone, not yet due at some candidates' node 0: those
  // candidates have nothing to start from, the later ones start on it.
  auto pending = std::make_unique<ReportedSegments>();
  pending->has_pending = true;
  pending->pending = adopted->followed;
  pending->pending.segment_seq = 12;
  const std::int64_t t_0 = rig->StartInstant(now);
  // Node 0 moved 20 ms into the candidates' node-0 instants [t_0, t_0 + 50 ms).
  const std::int64_t shift = t_0 + 20 * kMs - pending->pending.t0_ns;
  pending->pending.t0_ns += shift;
  pending->pending.t_c_ns += shift;
  const Wake w2 =
      RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), *pending, now);
  int no_source = 0;
  int started = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.reject == NlpReject::kLeadShort) {
      continue;
    }
    if (c.t_s_ns < pending->pending.t0_ns) {
      ++no_source;
      EXPECT_EQ(c.reject, NlpReject::kNoSource) << c.index << Table(rig->search, w2.stats);
      EXPECT_FALSE(c.ik_run);
    } else {
      ++started;
      EXPECT_NE(c.reject, NlpReject::kNoSource) << c.index;
      EXPECT_EQ(c.source_seq, 12U) << c.index;
    }
  }
  EXPECT_GT(no_source, 0) << Table(rig->search, w2.stats);
  EXPECT_GT(started, 0) << Table(rig->search, w2.stats);
}

TEST(NlpCatchSearchFollowing, TheStartStateIsTheReportedSegmentAtNodeZero) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  // A solution long enough to have nodes left once the RT is on it.
  const auto adopted = AdoptFirst(*rig, 0.36);
  ASSERT_TRUE(adopted->first.plan.valid);
  ASSERT_GE(adopted->followed.n_pre, 4);
  // A wake by which the followed segment has started: its t_0 is 70 ms past
  // that segment's node 0.
  const std::int64_t now = adopted->followed.t0_ns + 70 * kMs - (rig->StartInstant(kNow) - kNow);
  const std::int64_t t_0 = rig->StartInstant(now);
  ASSERT_EQ(t_0, adopted->followed.t0_ns + 70 * kMs);

  // Two reported segments that differ everywhere: the followed one, and a
  // pending one — the same trajectory with every joint 5 mrad off — whose
  // node 0 lies INSIDE the range of the candidates' node-0 instants, so that
  // the rule picks one for some candidates and the other for the rest.
  auto arm = Following(adopted->followed);
  arm->has_pending = true;
  arm->pending = adopted->followed;
  arm->pending.segment_seq = 12;
  for (double& q : arm->pending.q) {
    q += 5e-3;
  }
  // Its grid restarted some intervals later: node 0 on the followed one's
  // first node after t_0.
  const int drop = static_cast<int>((t_0 - arm->pending.t0_ns) / arm->pending.dt_pre_ns) + 1;
  ASSERT_GE(drop, 1);
  ASSERT_GT(arm->pending.n_pre, drop);
  const auto shift_nodes = [&](std::array<double, 200>& a) {
    std::copy(a.begin() + drop * kMaxSegmentNv, a.end(), a.begin());
  };
  shift_nodes(arm->pending.q);
  shift_nodes(arm->pending.qd);
  shift_nodes(arm->pending.qdd);
  arm->pending.n_pre -= drop;
  arm->pending.n_nodes -= drop;
  arm->pending.t0_ns += drop * arm->pending.dt_pre_ns;
  ASSERT_TRUE(ValidateSegmentNodes(arm->pending));
  ASSERT_GT(arm->pending.t0_ns, t_0);
  ASSERT_LT(arm->pending.t0_ns, t_0 + 50 * kMs);

  const Wake w = RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
  int on_pending = 0;
  int on_following = 0;
  int at_a_node = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.reject == NlpReject::kLeadShort) {
      continue;
    }
    // MD-58: the pending one from its node 0 on.
    const bool pending = c.t_s_ns >= arm->pending.t0_ns;
    const SegmentSnapshot& src = pending ? arm->pending : arm->following;
    (pending ? on_pending : on_following) += 1;
    EXPECT_EQ(c.source_seq, src.segment_seq) << c.index << Table(rig->search, w.stats);
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(src, c.t_s_ns, q, qd, qdd));
    for (std::size_t m = 0; m < 6; ++m) {
      const auto d = static_cast<std::size_t>(kDeviceOfModel[m]);
      EXPECT_TRUE(BitsEqual(c.q0[m], q[d])) << c.index << " joint " << m;
      EXPECT_TRUE(BitsEqual(c.qd0[m], qd[d])) << c.index << " joint " << m;
      EXPECT_TRUE(BitsEqual(c.qdd0[m], qdd[d])) << c.index << " joint " << m;
    }
    EXPECT_FALSE(c.x0_clamped) << c.index;
    // A node-0 instant ON a node of the source: the stored column itself.
    const std::int64_t since = c.t_s_ns - src.t0_ns;
    if (since % src.dt_pre_ns == 0 && since / src.dt_pre_ns <= src.n_pre) {
      ++at_a_node;
      const auto k = static_cast<int>(since / src.dt_pre_ns);
      const std::array<double, 6> node = NodeModelOrder(src.q, k);
      const std::array<double, 6> node_v = NodeModelOrder(src.qd, k);
      for (std::size_t m = 0; m < 6; ++m) {
        EXPECT_NEAR(c.q0[m], node[m], 1e-13) << c.index;
        EXPECT_NEAR(c.qd0[m], node_v[m], 1e-12) << c.index;
      }
    }
  }
  EXPECT_GT(on_pending, 0) << Table(rig->search, w.stats);
  EXPECT_GT(on_following, 0) << Table(rig->search, w.stats);
  EXPECT_GT(at_a_node, 0) << "no candidate's node 0 falls on a node of its source";

  // What the command and the followed segment disagree by is recorded: the
  // command is the segment at the instant the RT read it for, one joint 3 mrad
  // and 20 mrad/s off.
  PlannerRtState rt = rig->FollowingRt(now, adopted->t_c_ns);
  // The followed segment's node 0 is ahead of this report; one whose node 0
  // has passed is what the RT samples.
  auto running = Following(adopted->followed);
  const std::int64_t back = running->following.t0_ns - (rt.rt_state_ns - 10 * kMs);
  running->following.t0_ns -= back;
  running->following.t_c_ns -= back;
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(
      running->following, rt.rt_state_ns + Ns(rig->constants.control_dt), q, qd, qdd));
  for (std::size_t d = 0; d < 6; ++d) {
    rt.q_cmd[d] = q[d];
    rt.qd_cmd[d] = qd[d];
  }
  rt.q_cmd[4] += 3e-3;
  rt.qd_cmd[1] -= 2e-2;
  rt.plan_t_c_ns = running->following.t_c_ns;
  const Wake gap = RunWake(*rig, adopted->ball, rt, *running, now);
  EXPECT_NEAR(gap.stats.nlp.cmd_gap_q, 3e-3, 1e-12);
  EXPECT_NEAR(gap.stats.nlp.cmd_gap_qd, 2e-2, 1e-12);
  // Following nothing, there is no gap to report.
  EXPECT_TRUE(std::isnan(adopted->first.stats.nlp.cmd_gap_q));
  EXPECT_TRUE(std::isnan(adopted->first.stats.nlp.cmd_gap_qd));
}

TEST(NlpCatchSearchFollowing, AStartStateOutsideTheBoxIsProjectedAndMarked) {
  // The reported segment was solved in a wider box than this search's cores
  // have: its state at node 0 is outside, and a solve refuses such a start.
  auto wide = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(wide->Configure(&err)) << err;
  const auto adopted = AdoptFirst(*wide, 0.20);
  ASSERT_TRUE(adopted->first.plan.valid);
  const std::int64_t now = kNow + 99 * kMs;

  // The box's upper position face is put just BELOW where the followed
  // segment has joint 0 over the next 50 ms, and its velocity face below the
  // speed joint 0 has there.
  const std::int64_t t_0 = wide->StartInstant(now);
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(adopted->followed, t_0 + 25 * kMs, q, qd, qdd));
  int joint = 0;
  for (int m = 1; m < 6; ++m) {
    if (std::fabs(qd[static_cast<std::size_t>(kDeviceOfModel[static_cast<std::size_t>(m)])]) >
        std::fabs(qd[static_cast<std::size_t>(kDeviceOfModel[static_cast<std::size_t>(joint)])])) {
      joint = m;
    }
  }
  const auto dev = static_cast<std::size_t>(kDeviceOfModel[static_cast<std::size_t>(joint)]);
  ASSERT_GT(std::fabs(qd[dev]), 0.05) << "the followed segment does not move";

  auto rig = std::make_unique<Rig>();
  const double v_face = 0.5 * std::fabs(qd[dev]);
  rig->params.limits.qd_max[joint] = v_face;
  const bool upward = qd[dev] > 0.0;
  // A position face the moving joint has already passed.
  if (upward) {
    rig->params.limits.q_max[joint] = wide->q_wait[joint] + 0.25 * (q[dev] - wide->q_wait[joint]);
  } else {
    rig->params.limits.q_min[joint] = wide->q_wait[joint] + 0.25 * (q[dev] - wide->q_wait[joint]);
  }
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const auto arm = Following(adopted->followed);
  const Wake w = RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
  int clamped = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.reject == NlpReject::kLeadShort || c.source_seq == 0) {
      continue;
    }
    ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(adopted->followed, c.t_s_ns, q, qd, qdd));
    bool outside = false;
    for (std::size_t m = 0; m < 6; ++m) {
      const auto d = static_cast<std::size_t>(kDeviceOfModel[m]);
      const auto mi = static_cast<Eigen::Index>(m);
      const double q_in =
          std::clamp(q[d], rig->params.limits.q_min[mi], rig->params.limits.q_max[mi]);
      const double v_in =
          std::clamp(qd[d], -rig->params.limits.qd_max[mi], rig->params.limits.qd_max[mi]);
      outside = outside || q_in != q[d] || v_in != qd[d];
      EXPECT_TRUE(BitsEqual(c.q0[m], q_in)) << c.index << " joint " << m;
      EXPECT_TRUE(BitsEqual(c.qd0[m], v_in)) << c.index << " joint " << m;
      EXPECT_TRUE(BitsEqual(c.qdd0[m], qdd[d])) << c.index << " joint " << m;
    }
    EXPECT_EQ(c.x0_clamped, outside) << c.index;
    clamped += outside ? 1 : 0;
    // The solve was not refused for its start.
    EXPECT_NE(c.core_reason, MpcDockingReason::kInitialStateOutsideBox) << c.index;
  }
  EXPECT_GT(clamped, 0) << Table(rig->search, w.stats);
  if (w.plan.valid) {
    const Candidate* c = Find(rig->search, w.stats.nlp.chosen_index);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(rig->search.Solution()->seg.x0_clamped, c->x0_clamped);
    EXPECT_EQ(w.stats.nlp.chosen_x0_clamped, c->x0_clamped);
  }
}

// max |q_a(k) − q_b(k + shift)| over the nodes of `a`, both in device order.
[[nodiscard]] double TailDifference(const SegmentSnapshot& a, const SegmentSnapshot& b, int shift) {
  double worst = 0.0;
  for (int k = 0; k <= a.n_nodes; ++k) {
    for (int d = 0; d < a.nv; ++d) {
      const auto ia = static_cast<std::size_t>(k * kMaxSegmentNv + d);
      const auto ib = static_cast<std::size_t>((k + shift) * kMaxSegmentNv + d);
      worst = std::max(worst, std::fabs(a.q[ia] - b.q[ib]));
    }
  }
  return worst;
}

TEST(NlpCatchSearchFollowing, TheFollowedCandidatesResolveIsTheRestOfItsSolution) {
  // The prediction has not changed and the RT follows the first wake's
  // solution. Solved again from that solution's own state, on the same core
  // (the same number of pre-catch intervals) and on a shorter one (one
  // interval has passed), the followed candidate's trajectory is the part of
  // the solution that is left.
  struct Later {
    const char* name;
    int dropped;
  };

  for (const Later later : {Later{"the same grid", 0}, Later{"one interval has passed", 1},
                            Later{"two intervals have passed", 2}}) {
    SCOPED_TRACE(later.name);
    auto rig = std::make_unique<Rig>();
    std::string err;
    ASSERT_TRUE(rig->Configure(&err)) << err;
    // A candidate that needs motion, far enough ahead to lose two intervals.
    const auto adopted = AdoptFirst(*rig, 0.36);
    ASSERT_TRUE(adopted->first.plan.valid);
    const SegmentSnapshot before = adopted->followed;
    double motion = 0.0;
    for (int d = 0; d < 6; ++d) {
      motion = std::max(
          motion, std::fabs(before.q[static_cast<std::size_t>(before.n_pre * kMaxSegmentNv + d)] -
                            before.q[static_cast<std::size_t>(d)]));
    }
    ASSERT_GT(motion, 5e-3) << "the followed solution does not move";
    // The candidate keeps its grid while t_0 has moved by no more than its
    // wait, and loses one interval for each 50 ms beyond that.
    const std::int64_t after_ns =
        later.dropped == 0 ? adopted->wait_ns / 2
                           : adopted->wait_ns + (later.dropped - 1) * 50 * kMs + 20 * kMs;
    const std::int64_t now = kNow + after_ns;
    const auto arm = Following(adopted->followed);
    const Wake w = RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
    const Candidate* c = Find(rig->search, adopted->index);
    ASSERT_NE(c, nullptr);
    ASSERT_EQ(c->reject, NlpReject::kNone) << Table(rig->search, w.stats);
    ASSERT_EQ(c->start, NlpCatchSearch::Start::kSameCandidate);
    ASSERT_EQ(before.n_pre - c->n_pre, later.dropped) << Table(rig->search, w.stats);
    EXPECT_EQ(c->source_seq, 11U);
    const SegmentSnapshot* after = rig->search.RememberedSolution(adopted->index);
    ASSERT_NE(after, nullptr);
    ASSERT_EQ(after->t_c_ns, before.t_c_ns);
    ASSERT_EQ(after->n_nodes, before.n_nodes - later.dropped);
    const double diff = TailDifference(*after, before, later.dropped);
    RecordProperty(std::string("tail_max_dq_dropped_") + std::to_string(later.dropped),
                   std::to_string(static_cast<long long>(std::llround(diff * 1e18))) + "e-18");
    RecordProperty(std::string("tail_iterations_dropped_") + std::to_string(later.dropped),
                   c->iterations);
    EXPECT_LT(diff, 1e-9) << "iterations " << c->iterations << Table(rig->search, w.stats);
    // The catch instant in force stays: the same candidate, refreshed.
    ASSERT_TRUE(w.plan.valid);
    EXPECT_EQ(w.stats.nlp.chosen_index, adopted->index) << Table(rig->search, w.stats);
    EXPECT_EQ(w.plan.t_c_ns, adopted->t_c_ns);
    EXPECT_EQ(w.stats.decision, SwitchDecision::kRefreshed);
    EXPECT_TRUE(w.stats.publish);
  }
}

TEST(NlpCatchSearchFollowing, TheSwitchTermIsAnchoredOnThePlanTheRtFollows) {
  auto rig = std::make_unique<Rig>();
  rig->params.w_switch = 3.0;
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  // No choice yet, no plan followed: the term is zero for every candidate.
  const Wake w1 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  ASSERT_TRUE(w1.plan.valid);
  for (const Candidate& c : rig->search.Candidates()) {
    EXPECT_EQ(c.j_switch, 0.0) << c.index;
  }
  const std::int64_t chosen = w1.plan.t_c_ns;
  const SegmentSnapshot seg = rig->search.Solution()->seg;
  const auto term = [&](std::int64_t t_c, std::int64_t anchor) {
    const double d = Sec(t_c - anchor) / rig->params.t_ref_s;
    return rig->params.w_switch * d * d;
  };
  // Still no plan followed: anchored on what this search chose last.
  const std::int64_t now2 = kNow + 33 * kMs;
  const Wake w2 = RunWake(*rig, ball, rig->RestingRt(now2), NoSegments(), now2);
  int judged = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.rank >= 0) {
      ++judged;
      EXPECT_DOUBLE_EQ(c.j_switch, term(c.t_c_ns, chosen)) << c.index;
    }
  }
  EXPECT_GE(judged, 4);
  static_cast<void>(w2);
  // The RT follows ANOTHER plan than the one last chosen — 120 ms later: the
  // anchor is the followed one's catch instant.
  const std::int64_t followed = chosen + 120 * kMs;
  ASSERT_NE(followed, rig->search.Candidates()[0].t_c_ns);
  const std::int64_t now3 = kNow + 66 * kMs;
  const auto arm = Following(seg);
  arm->following.segment_seq = 11;
  const Wake w3 = RunWake(*rig, ball, rig->FollowingRt(now3, followed), *arm, now3);
  judged = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.rank >= 0) {
      ++judged;
      EXPECT_DOUBLE_EQ(c.j_switch, term(c.t_c_ns, followed)) << c.index;
      EXPECT_NE(c.j_switch, term(c.t_c_ns, chosen)) << c.index;
      if (c.reject == NlpReject::kNone) {
        EXPECT_DOUBLE_EQ(c.phi, c.j_reference + c.j_time + c.j_switch);
      }
    }
  }
  EXPECT_GE(judged, 4) << Table(rig->search, w3.stats);
  // The term is part of the rank as well.
  for (const Candidate& a : rig->search.Candidates()) {
    for (const Candidate& b : rig->search.Candidates()) {
      if (a.rank >= 0 && b.rank >= 0 && a.rank < b.rank) {
        EXPECT_LE(a.rank_key, b.rank_key);
      }
    }
  }
  // A trial reset forgets the last choice.
  rig->search.ResetTrial();
  const Wake w4 = RunWake(*rig, ball, rig->RestingRt(now3), NoSegments(), now3);
  ASSERT_TRUE(w4.plan.valid);
  for (const Candidate& c : rig->search.Candidates()) {
    EXPECT_EQ(c.j_switch, 0.0) << c.index;
  }
}

TEST(NlpCatchSearchFollowing, TheDecisionComparesCatchInstantsToTheNanosecond) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const auto adopted = AdoptFirst(*rig, 0.20);
  ASSERT_TRUE(adopted->first.plan.valid);
  EXPECT_EQ(adopted->first.stats.decision, SwitchDecision::kNoCurrent);
  const std::int64_t now = kNow + 10 * kMs;
  const auto arm = Following(adopted->followed);
  // The followed plan's catch instant IS the chosen one's.
  const Wake same = RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
  ASSERT_TRUE(same.plan.valid);
  ASSERT_EQ(same.plan.t_c_ns, adopted->t_c_ns);
  EXPECT_EQ(same.stats.decision, SwitchDecision::kRefreshed);
  EXPECT_TRUE(same.stats.publish);
  // One nanosecond apart is another catch instant.
  for (const std::int64_t off : {std::int64_t{1}, std::int64_t{-1}, 20 * kMs}) {
    const Wake other =
        RunWake(*rig, adopted->ball, rig->FollowingRt(now, adopted->t_c_ns + off), *arm, now);
    ASSERT_TRUE(other.plan.valid);
    ASSERT_EQ(other.plan.t_c_ns, adopted->t_c_ns);
    EXPECT_EQ(other.stats.decision, SwitchDecision::kReplaced) << off;
    EXPECT_TRUE(other.stats.publish);
  }
  // Nothing valid while a plan is followed: hold — publish nothing.
  Throw gone = adopted->ball;
  for (auto& s : gone.traj.s) {
    s.p[0] += 3.0;
  }
  const Wake held = RunWake(*rig, gone, rig->FollowingRt(now, adopted->t_c_ns), *arm, now);
  EXPECT_FALSE(held.plan.valid);
  EXPECT_EQ(held.stats.decision, SwitchDecision::kHeldNoCandidate);
  EXPECT_FALSE(held.stats.publish);
  EXPECT_EQ(held.stats.nlp.reason, NlpReject::kIk);
  EXPECT_EQ(rig->search.Solution(), nullptr);
}

TEST(NlpCatchSearchFollowing, AnUncertainBallIsRefusedByItsChanceRowsAndASharperOneAccepted) {
  // The ball's VELOCITY is uncertain (0.5 m/s on every axis) and its position
  // is not: the screening's closing-speed window, which reads the position
  // spread along the approach axis, stays open, every candidate is solved, and
  // every solution breaks the terminal velocity set's chance rows — whatever
  // the arm does, since no pose makes the ball's speed better known. The
  // samples are ON the lattice instants, so that no position spread grows out
  // of the velocity's between a sample and a candidate.
  for (const bool following : {false, true}) {
    SCOPED_TRACE(following ? "the RT follows a plan" : "no plan followed");
    auto rig = std::make_unique<Rig>();
    std::string err;
    ASSERT_TRUE(rig->Configure(&err)) << err;
    const Throw ball = AxisThrow(*rig, 0.28, 0.8, 0.002, 1, kTrack, Eigen::Vector2d(0.04, -0.03),
                                 /*spacing_ns=*/40 * kMs);
    const Wake first = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
    ASSERT_TRUE(first.plan.valid) << Table(rig->search, first.stats);
    auto arm = Following(rig->search.Solution()->seg);
    arm->following.segment_seq = 11;
    const std::int64_t now = kNow + 33 * kMs;
    const PlannerRtState rt =
        following ? rig->FollowingRt(now, first.plan.t_c_ns) : rig->RestingRt(now);
    const ReportedSegments& reported = following ? *arm : NoSegments();
    Throw unsure = ball;
    unsure.cov = LateralCovariance(unsure, 0.002, 0.001, /*sigma_v=*/0.5);
    const Wake w = RunWake(*rig, unsure, rt, reported, now);
    EXPECT_FALSE(w.plan.valid) << Table(rig->search, w.stats);
    EXPECT_EQ(w.stats.nlp.reason, NlpReject::kChance) << Table(rig->search, w.stats);
    EXPECT_EQ(w.plan.reason, PlanReason::kUncertainty);
    EXPECT_GE(w.stats.nlp.n_solved, 3) << Table(rig->search, w.stats);
    EXPECT_EQ(Count(rig->search, NlpReject::kChance), w.stats.nlp.n_solved)
        << Table(rig->search, w.stats);
    EXPECT_EQ(Count(rig->search, NlpReject::kSpeedWindow), 0);
    for (const Candidate& c : rig->search.Candidates()) {
      if (c.reject == NlpReject::kChance) {
        EXPECT_EQ(c.worst_group, DockingRowGroup::kVelocitySet) << c.index;
        EXPECT_GT(c.worst_violation, 0.1) << c.index;  // m/s — no tolerance's doing
      }
    }
    // Nothing is published over a followed plan; "no plan" otherwise.
    EXPECT_EQ(w.stats.publish, !following);
    EXPECT_EQ(w.stats.decision,
              following ? SwitchDecision::kHeldNoCandidate : SwitchDecision::kNoCurrent);
    // What each refused solve ended on is remembered …
    std::set<std::int64_t> refused;
    for (const Candidate& c : rig->search.Candidates()) {
      if (c.reject == NlpReject::kChance) {
        EXPECT_NE(rig->search.RememberedSolution(c.index), nullptr) << c.index;
        refused.insert(c.index);
      }
    }
    // The same throw at the same instant, its velocity known ten times better.
    Throw sharp = ball;
    sharp.cov = LateralCovariance(sharp, 0.002, 0.001, /*sigma_v=*/0.05);
    const Wake ok = RunWake(*rig, sharp, rt, reported, now);
    EXPECT_TRUE(ok.plan.valid) << Table(rig->search, ok.stats);
    EXPECT_EQ(Count(rig->search, NlpReject::kChance), 0);
    EXPECT_TRUE(ok.stats.publish);
    // … and is where the next wake starts that candidate: a start from an
    // iterate the rows refused does not keep the candidate refused once the
    // rows can be met.
    int from_refused = 0;
    for (const Candidate& c : rig->search.Candidates()) {
      if (c.solved && refused.count(c.index) != 0) {
        EXPECT_EQ(c.start, NlpCatchSearch::Start::kSameCandidate) << c.index;
        EXPECT_EQ(c.reject, NlpReject::kNone) << c.index << Table(rig->search, ok.stats);
        ++from_refused;
      }
    }
    EXPECT_GE(from_refused, 3) << Table(rig->search, ok.stats);
    EXPECT_EQ(refused.count(ok.stats.nlp.chosen_index), 1U);
  }
}

TEST(NlpCatchSearchFollowing, ALaterallyUncertainBallIsRefusedAndTheReasonFollowsTheResidual) {
  // The POSITION is uncertain across the ball's travel (3 cm against a 3 cm
  // capture half-width) and sharp along it. No candidate is valid, and the
  // largest violation of every solution is a lateral chance row. But the arm
  // CAN shrink that row a little — by turning the capture plane against the
  // spread — and a solve that ends infeasible leaves what it broke on the way
  // there: the reason reads `chance` where only chance rows are left violated
  // and `hard_row` where an arm row is too. How many of each is recorded.
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  Throw ball = AxisThrow(*rig, 0.28);
  ball.cov = LateralCovariance(ball, 0.03, 0.001, 0.002);
  const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  EXPECT_FALSE(w.plan.valid) << Table(rig->search, w.stats);
  EXPECT_EQ(Count(rig->search, NlpReject::kSpeedWindow), 0) << Table(rig->search, w.stats);
  EXPECT_GE(w.stats.nlp.n_solved, 4);
  EXPECT_EQ(w.stats.nlp.n_valid, 0);
  const int chance = Count(rig->search, NlpReject::kChance);
  const int hard_row = Count(rig->search, NlpReject::kHardRow);
  EXPECT_EQ(chance + hard_row, w.stats.nlp.n_solved) << Table(rig->search, w.stats);
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.solved) {
      EXPECT_EQ(c.worst_group, DockingRowGroup::kLateral) << c.index;
      EXPECT_GT(c.worst_violation, 0.02) << c.index;  // metres
    }
  }
  EXPECT_TRUE(w.stats.nlp.reason == NlpReject::kChance ||
              w.stats.nlp.reason == NlpReject::kHardRow);
  RecordProperty("lateral_sigma_3cm_solved", w.stats.nlp.n_solved);
  RecordProperty("lateral_sigma_3cm_reason_chance", chance);
  RecordProperty("lateral_sigma_3cm_reason_hard_row", hard_row);
  // Ten times sharper, the same throw is caught.
  ball.cov = LateralCovariance(ball, 0.003, 0.001, 0.002);
  rig->search.ResetTrial();
  const Wake ok = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  EXPECT_TRUE(ok.plan.valid) << Table(rig->search, ok.stats);
}

// ── 7. The lattice, the grid, Configure ──────────────────────────────────────

TEST(NlpCatchSearchLattice, EveryCandidateOfTheWindowStartsLessThanOneIntervalAfterT0) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  std::set<std::int64_t> waits;
  std::set<int> counts;
  std::int64_t anchor = 0;
  for (int wake = 0; wake < 12; ++wake) {
    const std::int64_t now = kNow + wake * 7 * kMs;
    const Wake w = RunWake(*rig, ball, rig->RestingRt(now), NoSegments(), now);
    if (wake == 0) {
      anchor = rig->search.LatticeAnchorNs();
      EXPECT_EQ(anchor, kNow);
    }
    // The anchor is the trial's, not the wake's.
    EXPECT_EQ(rig->search.LatticeAnchorNs(), anchor);
    const std::int64_t t_0 = rig->StartInstant(now);
    EXPECT_EQ(rig->search.LastStartInstantNs(), t_0);
    int in_window = 0;
    std::int64_t previous = 0;
    for (const Candidate& c : rig->search.Candidates()) {
      // On the lattice, in order, ahead of t_0 and inside the horizon.
      EXPECT_EQ(c.t_c_ns, anchor + c.index * 40 * kMs);
      EXPECT_GT(c.t_c_ns, t_0);
      EXPECT_LE(c.t_c_ns, t_0 + 400 * kMs);
      EXPECT_GT(c.t_c_ns, previous);
      previous = c.t_c_ns;
      EXPECT_EQ(c.reject == NlpReject::kLeadShort, c.t_c_ns - t_0 < 100 * kMs) << c.index;
      if (c.reject == NlpReject::kLeadShort) {
        continue;
      }
      ++in_window;
      EXPECT_EQ(c.wait_ns, c.t_s_ns - t_0);
      EXPECT_GE(c.wait_ns, 0) << c.index;
      EXPECT_LT(c.wait_ns, 50 * kMs) << c.index;
      EXPECT_EQ(c.t_s_ns, c.t_c_ns - c.n_pre * 50 * kMs);
      EXPECT_GE(c.n_pre, rig->params.n_pre_min);
      EXPECT_LE(c.n_pre, rig->params.n_pre_max);
      waits.insert(c.wait_ns);
      counts.insert(c.n_pre);
    }
    EXPECT_EQ(in_window, w.stats.n_in_window);
    EXPECT_GE(in_window, 7);
  }
  // The wakes covered the grid: many phases of the wait, every core.
  EXPECT_GE(waits.size(), 20U);
  // 2..7 intervals (8 only at a lead of exactly 0.4 s).
  EXPECT_GE(counts.size(), 6U);

  // A new track is a new attempt: the lattice is anchored again, and nothing
  // of the old track's memory is used.
  const std::int64_t now = kNow + 100 * kMs;
  const Throw other = AxisThrow(*rig, 0.38, 0.8, 0.002, /*seq=*/1, /*gen=*/kTrack + 1);
  const Wake w = RunWake(*rig, other, rig->RestingRt(now), NoSegments(), now);
  EXPECT_EQ(rig->search.LatticeAnchorNs(), now);
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.rank >= 0 && c.solved) {
      EXPECT_EQ(c.start, NlpCatchSearch::Start::kIkTarget) << c.index;
    }
  }
  EXPECT_TRUE(w.plan.valid);
  // And a reset likewise.
  rig->search.ResetTrial();
  const std::int64_t later = now + 21 * kMs;
  static_cast<void>(RunWake(*rig, other, rig->RestingRt(later), NoSegments(), later));
  EXPECT_EQ(rig->search.LatticeAnchorNs(), later);
}

TEST(NlpCatchSearchLattice, TheStartPointComesFromThePreviousWakesMemory) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Throw ball = AxisThrow(*rig, 0.28);
  const Wake w1 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  for (const Candidate& c : rig->search.Candidates()) {
    if (c.solved) {
      EXPECT_EQ(c.start, NlpCatchSearch::Start::kIkTarget) << c.index;
      EXPECT_NE(rig->search.RememberedSolution(c.index), nullptr) << c.index;
    } else {
      EXPECT_EQ(rig->search.RememberedSolution(c.index), nullptr) << c.index;
    }
  }
  const std::int64_t entering = rig->search.Candidates().back().index + 1;
  // 41 ms later one more lattice instant has entered the horizon.
  const std::int64_t now = kNow + 41 * kMs;
  const Wake w2 = RunWake(*rig, ball, rig->RestingRt(now), NoSegments(), now);
  int own = 0;
  int cold_again = 0;
  for (const Candidate& c : rig->search.Candidates()) {
    if (!c.solved) {
      continue;
    }
    if (c.index == entering) {
      // A new candidate: the nearest remembered one, stretched in time.
      EXPECT_EQ(c.start, NlpCatchSearch::Start::kNeighbour) << Table(rig->search, w2.stats);
      EXPECT_EQ(c.start_index, entering - 1);
      EXPECT_EQ(c.reject, NlpReject::kNone);
    } else {
      EXPECT_EQ(c.start, NlpCatchSearch::Start::kSameCandidate) << c.index;
      EXPECT_EQ(c.start_index, c.index);
      ++own;
      cold_again += c.iterations > 1 ? 1 : 0;
    }
  }
  EXPECT_GE(own, 4) << Table(rig->search, w2.stats);
  // A restart from its own solution on an unchanged grid converges at once.
  EXPECT_LT(cold_again, own) << "no restart took a single iteration";
  ASSERT_NE(Find(rig->search, entering), nullptr) << Table(rig->search, w2.stats);
  static_cast<void>(w1);
}

TEST(NlpCatchSearchConfigure, RefusesAGridThatDoesNotCoverTheCandidateWindow) {
  const auto refuses = [](auto&& edit, const char* fragment) {
    auto rig = std::make_unique<Rig>();
    edit(*rig);
    std::string err;
    EXPECT_FALSE(rig->Configure(&err)) << fragment;
    EXPECT_FALSE(rig->search.Configured());
    EXPECT_NE(err.find(fragment), std::string::npos)
        << "'" << err << "' lacks '" << fragment << "'";
    // An unconfigured search answers "no plan" and holds no solution.
    const Throw ball = AxisThrow(*rig, 0.28);
    const Wake w = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
    EXPECT_FALSE(w.plan.valid);
    EXPECT_EQ(rig->search.Solution(), nullptr);
  };
  // n_pre_max·Δ_a < T_max: a far candidate would wait before it moves.
  refuses([](Rig& r) { r.params.t_max = 0.401; }, "does not cover the candidate window");
  refuses([](Rig& r) { r.params.n_pre_max = 7; }, "does not cover the candidate window");
  // t_lead_min < n_pre_min·Δ_a: a near candidate would have no grid.
  refuses([](Rig& r) { r.params.t_lead_min = 0.099; }, "shorter than n_pre_min * dt_pre");
  refuses([](Rig& r) { r.params.n_pre_min = 3; }, "shorter than n_pre_min * dt_pre");
  // Exactly covering is accepted — the comparison is on integer nanoseconds.
  {
    auto rig = std::make_unique<Rig>();
    rig->params.t_max = 0.4;
    rig->params.t_lead_min = 0.1;
    std::string err;
    EXPECT_TRUE(rig->Configure(&err)) << err;
    EXPECT_TRUE(rig->search.Configured());
  }
  refuses([](Rig& r) { r.params.cand_capacity = 10; }, "cand_capacity");  // needs 11
  refuses([](Rig& r) { r.params.cand_dt = 0.0; }, "non-finite or out of range");
  refuses([](Rig& r) { r.params.max_solves = 0; }, "max_solves");
  // A share the budget does not hold once: no wake could run a solve.
  refuses([](Rig& r) { r.params.solve_budget_s = 0.0401; }, "solve_budget_s is over budget_s");
  refuses([](Rig& r) { r.params.max_solves = 33; }, "max_solves");
  refuses([](Rig& r) { r.params.t_ref_s = 0.0; }, "non-finite or out of range");
  refuses([](Rig& r) { r.params.w_switch = -1.0; }, "non-finite or out of range");
  refuses([](Rig& r) { r.params.catch_box.set = false; }, "catch_box");
  refuses([](Rig& r) { r.params.wait_pose_n = 5; }, "wait_pose");
  refuses([](Rig& r) { r.params.stop_block_sizes = {1, 1, 2, 2}; }, "do not add up");
  refuses([](Rig& r) { r.params.n_stop_blocks = 2; }, "at least 3 move blocks");
  refuses(
      [](Rig& r) {
        r.params.n_pre_max = 18;  // 18 + 7 nodes: more than a published segment holds
        r.params.t_max = 0.4;
      },
      "n_pre_max + n_stop");
  refuses([](Rig& r) { r.model.device_of_model[1] = 2; }, "permutation");
  refuses([](Rig& r) { r.model.handle = nullptr; }, "no arm model");
  // An IK handle on another arm than the cores' model.
  {
    rtc::testing::Arm other = rtc::testing::Arm7R();
    refuses([&other](Rig& r) { r.model.handle = other.handle.get(); }, "IK handle");
  }
  // What a core refuses is reported with the core's own reason.
  refuses([](Rig& r) { r.params.core.c_cap_max = 0.1; }, "refused its grid or parameters");
}

TEST(NlpCatchSearchConfigure, BuildsOneCoreForEveryPreCatchCountOnTheSharedParameters) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  for (int n_pre = rig->params.n_pre_min; n_pre <= rig->params.n_pre_max; ++n_pre) {
    ASSERT_NE(rig->search.Core(n_pre), nullptr) << n_pre;
    const MpcDockingSegmentCore& core = *rig->search.Core(n_pre);
    EXPECT_TRUE(core.IsInitialized());
    EXPECT_EQ(core.CatchNode(), n_pre);
    EXPECT_EQ(core.NumNodes(), n_pre + 7);
    // One block per pre-catch interval, the stop part's four.
    EXPECT_EQ(core.NumBlocks(), n_pre + 4);
    EXPECT_DOUBLE_EQ(core.NodeTime(n_pre), n_pre * 0.05);
    EXPECT_DOUBLE_EQ(core.NodeTime(n_pre + 7), n_pre * 0.05 + 0.35);
    // The reach condition reads this box.
    EXPECT_EQ(core.VelocityLimit(), rig->params.limits.qd_max);
    EXPECT_EQ(core.PositionLower(), rig->params.limits.q_min);
    EXPECT_EQ(core.PositionUpper(), rig->params.limits.q_max);
    EXPECT_FALSE(core.HasAccelerationBox());
  }
  // No core outside the configured counts — and none at all on a search that
  // was never configured or whose Configure failed.
  EXPECT_EQ(rig->search.Core(rig->params.n_pre_min - 1), nullptr);
  EXPECT_EQ(rig->search.Core(rig->params.n_pre_max + 1), nullptr);
  {
    NlpCatchSearch never;
    EXPECT_EQ(never.Core(rig->params.n_pre_min), nullptr);
    auto broken = std::make_unique<Rig>();
    broken->params.core.c_cap_max = 0.1;  // a core refuses it
    ASSERT_FALSE(broken->Configure(&err));
    EXPECT_EQ(broken->search.Core(broken->params.n_pre_min), nullptr);
  }
  // A second Configure is a new search: the first wake is a first wake.
  const Throw ball = AxisThrow(*rig, 0.28);
  const Wake w1 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  const std::uint64_t d1 = WakeDigest(rig->search, w1);
  ASSERT_TRUE(rig->Configure(&err)) << err;
  const Wake w2 = RunWake(*rig, ball, rig->RestingRt(kNow), NoSegments(), kNow);
  EXPECT_EQ(WakeDigest(rig->search, w2), d1);
}

TEST(NlpCatchSearchMonitor, ReportsThePositionSpreadAtTheFollowedCatchInstant) {
  auto rig = std::make_unique<Rig>();
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  Throw ball = AxisThrow(*rig, 0.28);
  ball.cov = LateralCovariance(ball, 0.004, 0.001, 0.0);
  SearchStats stats;
  stats.publish = true;
  rig->search.Monitor(ball.traj, ball.cov, true, rig->FollowingRt(kNow, kNow + 300 * kMs), stats);
  EXPECT_FALSE(stats.publish);
  EXPECT_NEAR(stats.sigma_l, 0.004, 1e-9);  // the sample AT +300 ms: no propagation
  EXPECT_FALSE(stats.nlp.ran);
  // Nothing followed, or a covariance that is not this prediction's: unknown.
  rig->search.Monitor(ball.traj, ball.cov, true, rig->RestingRt(kNow), stats);
  EXPECT_TRUE(std::isnan(stats.sigma_l));
  rig->search.Monitor(ball.traj, ball.cov, false, rig->FollowingRt(kNow, kNow + 300 * kMs), stats);
  EXPECT_TRUE(std::isnan(stats.sigma_l));
  EXPECT_FALSE(stats.publish);
}

// ── 8. Allocation ────────────────────────────────────────────────────────────

struct GateLog {
  int solver_calls{0};
  int stages{0};
};

// The gate is armed over the whole Plan (depth 1). A QP-solving call suspends
// it; inside a core's Solve each stage that is NOT the QP arms it again.
void SolverHook(bool begin, void* user) noexcept {
  auto* log = static_cast<GateLog*>(user);
  if (begin) {
    --rtc::testing::detail::MallocGateDepth();
    ++log->solver_calls;
  } else {
    ++rtc::testing::detail::MallocGateDepth();
  }
}

void StageHook(MpcDockingStage, bool begin, void* user) noexcept {
  auto* log = static_cast<GateLog*>(user);
  if (begin) {
    ++rtc::testing::detail::MallocGateDepth();
    ++log->stages;
  } else {
    --rtc::testing::detail::MallocGateDepth();
  }
}

TEST(NlpCatchSearchAllocation, AWakeAllocatesNothingOutsideTheQpSolvers) {
  // Positive control: the gate counts an allocation made inside a library.
  {
    const rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data probe(*fx::Synthetic6R().model);
    ASSERT_GT(gate.count(), 0U) << "the malloc gate does not see library allocations";
  }
  auto rig = std::make_unique<Rig>();
  // Every optional term of the screening and of the cores on.
  rig->params.rank_w_manip = 0.1;
  rig->params.w_time = 0.2;
  rig->params.w_switch = 0.5;
  rig->params.core.w_manip = 0.02;
  rig->params.core.w_impact = 0.5;
  rig->params.core.e_ref = 0.05;
  rig->params.core.e_max = 0.5;
  rig->params.core.p_max = 0.5;
  rig->params.core.w_perp = 50.0;
  std::string err;
  ASSERT_TRUE(rig->Configure(&err)) << err;
  GateLog log;
  rig->search.SetSolverHookForTesting(&SolverHook, &log);
  rig->search.SetCoreStageHookForTesting(&StageHook, &log);
  const Throw ball = AxisThrow(*rig, 0.20);
  const Throw far = AxisThrow(*rig, 0.28, 6.0, 0.002, /*seq=*/3);
  auto arm = std::make_unique<ReportedSegments>();

  // Everything a wake reads is built before the gate; the wakes run inside it.
  struct Step {
    const Throw* ball;
    bool following;
    std::int64_t now;
  };

  // A first wake, a second one from memory with a candidate entering, wakes on
  // the reported segment (set after the first), a wake whose screening removes
  // candidates for several reasons, and one the report is unusable for.
  const std::array<Step, 6> steps{{{&ball, false, kNow},
                                   {&ball, false, kNow + 41 * kMs},
                                   {&ball, true, kNow + 66 * kMs},
                                   {&ball, true, kNow + 99 * kMs},
                                   {&far, false, kNow + 120 * kMs},
                                   {&far, true, kNow + 130 * kMs}}};
  std::array<Wake, 6> wakes{};
  std::array<PlannerRtState, 6> reports{};
  std::int64_t followed_t_c = 0;
  std::size_t counted = 0;
  std::array<int, 6> solved{};
  std::set<NlpCatchSearch::Start> starts;
  std::set<NlpReject> reasons;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    reports[i] = steps[i].following ? rig->FollowingRt(steps[i].now, followed_t_c)
                                    : rig->RestingRt(steps[i].now);
    SearchStats& stats = wakes[i].stats;
    PlanSnapshot& plan = wakes[i].plan;
    const ReportedSegments& reported = steps[i].following ? *arm : NoSegments();
    {
      rtc::testing::detail::MallocGateCount() = 0;
      ++rtc::testing::detail::MallocGateDepth();
      plan = rig->search.Plan(steps[i].ball->traj, steps[i].ball->cov, true, reports[i], reported,
                              NowReal{steps[i].now}, stats);
      --rtc::testing::detail::MallocGateDepth();
      counted += rtc::testing::detail::MallocGateCount();
    }
    ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
    solved[i] = stats.nlp.n_solved;
    for (const Candidate& c : rig->search.Candidates()) {
      starts.insert(c.start);
      reasons.insert(c.reject);
    }
    if (i == 0) {
      ASSERT_TRUE(plan.valid) << Table(rig->search, stats);
      arm->has_following = true;
      arm->following = rig->search.Solution()->seg;
      arm->following.segment_seq = 11;
      followed_t_c = plan.t_c_ns;
    }
  }
  EXPECT_EQ(counted, 0U) << "Plan allocated outside the QP solvers";
  // The gated wakes did the work the claim is about.
  EXPECT_GT(solved[0], 3);
  EXPECT_GT(solved[1], 3);
  EXPECT_GT(solved[2], 3);
  EXPECT_TRUE(starts.contains(NlpCatchSearch::Start::kIkTarget));
  EXPECT_TRUE(starts.contains(NlpCatchSearch::Start::kSameCandidate));
  EXPECT_TRUE(starts.contains(NlpCatchSearch::Start::kNeighbour));
  EXPECT_TRUE(reasons.contains(NlpReject::kNone));
  EXPECT_TRUE(reasons.contains(NlpReject::kLeadShort));
  EXPECT_GE(reasons.size(), 4U);
  EXPECT_TRUE(wakes[2].plan.valid || wakes[3].plan.valid);
  EXPECT_GT(log.solver_calls, 30);
  EXPECT_GT(log.stages, 50);
  // The other three entry points, and a reset between trials.
  {
    const rtc::testing::ScopedMallocGate gate;
    SearchStats stats;
    rig->search.Monitor(ball.traj, ball.cov, true, reports[2], stats);
    rig->search.NotePublished(wakes[0].plan);
    rig->search.ResetTrial();
    static_cast<void>(rig->search.Solution());
    EXPECT_EQ(gate.count(), 0U);
  }
}

}  // namespace
