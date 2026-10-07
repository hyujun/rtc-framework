// E1-F08 (#661): the APPROACH–stop pair and replans as PlannerCycle runs them
// (planner_cycle.hpp "THE MPC SEGMENT PLANNER'S PART") — the real search on a real
// 6-joint arm, the MPC segment planner behind it, on the REAL steady clock (the
// budgets are measured), with the test as the RT stand-in. RUN_SERIAL: a
// loaded host stretches the solves the budgets judge. The planner's own
// decisions are test_catching_approach_planner.cpp; the core alone is E1-F07's
// test_catching_mpc_segment_core_approach.cpp.
//   pair        APairIsPublishedTogetherSegmentFirst, AWithheldSegmentWithholdsThePlan,
//               ANewerSnapshotOfTheSameTrackStillPublishes, RightAfterAPairTheSearchWaits
//   replans     OnceFollowingTheSearchStopsAndEverySegmentStartsOnTheReport,
//               AnotherTracksBallIsNotTheFollowedPlansTarget,
//               WithoutAReportNothingIsReplanned
//   replacement ApproachCycleReplacement.* — the search's other plan goes out
//               as a pair beside the followed plan's segments; every following
//               wake searches first, then pairs or replans (E1-F17 #743; on
//               the settable clock)
//   lifetime    ATrialResetWithdrawsTheSegment,
//               WithoutASegmentBoxThePlanIsPublishedAlone
//
//   stop line   TheStopCoresTakeTheFollowedSegmentsLineThroughAWholeCatch — the one
//               test here on a SETTABLE clock (FakeTime): it asserts values,
//               not budgets, so no solve duration may decide it
//   bit for bit AWholeCatchOnThePinnedClockIsUnchangedBitForBit — every wake's
//               record and both boxes, digested; a refactor of what the cycle
//               calls must leave the four numbers alone (E1-F12 #738)
//
// The RT stand-in reports the segment it holds pending and then follows, as
// the RT does under `mode: mpc` (E1-F09); WithoutAReportNothingIsReplanned
// pins an RT that reports neither.
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/planner_trace_digest.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using rtc::catching::CycleOutcome;
using rtc::catching::Mode;
using rtc::catching::PlannerCycleRecord;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::SegmentKind;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentSnapshot;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kTArm = 50 * kMs;  // the sim profile catch_lead_on's T_arm
constexpr std::int64_t kH = 2 * kMs;
constexpr std::uint64_t kActivation = 3;
constexpr std::uint64_t kTrack = 7;

// The suite's clock: the real steady clock, unless a test pins it (FakeTime).
// A rig built with `fake_clock` hands the same function to the cycle, the
// search and the MPC segment planner, so on a pinned clock every budget and lead is
// measured against the test's own time — a solve takes zero of it.
std::atomic<std::int64_t> g_fake_now{0};  // 0 = not pinned

std::int64_t SuiteClock() noexcept {
  const std::int64_t pinned = g_fake_now.load();
  return pinned != 0 ? pinned : rtc::SteadyNowNs();
}

std::int64_t Now() {
  return SuiteClock();
}

struct FakeTime {
  explicit FakeTime(std::int64_t start) { g_fake_now.store(start); }

  ~FakeTime() { g_fake_now.store(0); }

  FakeTime(const FakeTime&) = delete;
  FakeTime& operator=(const FakeTime&) = delete;

  void Advance(std::int64_t ns) { g_fake_now.fetch_add(ns); }
};

struct Boxes {
  rtc::SeqLock<rtc::catching::TrajectorySnapshot> traj;
  rtc::SeqLock<rtc::catching::CovarianceSnapshot> cov;
  rtc::SeqLock<PlannerRtState> rt;
  rtc::SeqLock<PlanSnapshot> plan;
  rtc::SeqLock<SegmentSnapshot> segment;
};

// The RT side under `mode: mpc`: adopt the first plan in the box, hold
// each newer segment of that plan pending until its node 0, then follow it.
// Mode by time: APPROACH, COMMITTED from t_c − T_freeze, DECEL from t_c.
struct RtStandIn {
  std::vector<double> wait_pose;  // device order
  bool adopt{true};               // false: never takes a plan
  bool report_segment{true};      // false: an RT that reports no segment
  // The newest segment it takes: a later one is published and never followed
  // (the RT's switch gate refused it).
  std::uint32_t take_up_to_seq{std::numeric_limits<std::uint32_t>::max()};
  std::int64_t t_freeze_ns{200 * kMs};
  std::uint64_t track{kTrack};  // the track it consumed last
  std::uint32_t reset_epoch{1};
  bool following{false};
  std::uint32_t plan_id{0};
  std::int64_t t_c{0};
  std::uint32_t last_seq{0};
  bool pending{false};
  std::uint32_t pending_seq{0};
  std::int64_t pending_t0{0};
  bool active{false};
  std::uint32_t active_seq{0};

  PlannerRtState Tick(Boxes& b, std::int64_t now) {
    if (adopt && !following) {
      const PlanSnapshot p = b.plan.Load();
      if (p.valid) {
        following = true;
        plan_id = p.plan_id;
        t_c = p.t_c_ns;
      }
    }
    if (following && report_segment) {
      const SegmentSnapshot d = b.segment.Load();
      if (d.valid && d.plan_id == plan_id && d.segment_seq > last_seq &&
          d.segment_seq <= take_up_to_seq) {
        last_seq = d.segment_seq;
        pending = true;
        pending_seq = d.segment_seq;
        pending_t0 = d.t0_ns;
      }
      if (pending && now + kTArm + kH >= pending_t0) {
        active = true;
        active_seq = pending_seq;
        pending = false;
      }
    }
    PlannerRtState s = rtc::testing::TrackingRtState(kActivation, wait_pose);
    s.rt_iteration = static_cast<std::uint64_t>(now / kH);
    s.rt_state_ns = now;
    s.reset_epoch = reset_epoch;
    s.track_seen = true;
    s.track_generation = track;
    if (following) {
      const std::int64_t lead = t_c - now;
      s.mode = static_cast<std::uint8_t>(lead <= 0             ? Mode::kDecel
                                         : lead <= t_freeze_ns ? Mode::kCommitted
                                                               : Mode::kApproach);
      s.plan_active = true;
      s.plan_id = plan_id;
      s.plan_t_c_ns = t_c;
      s.segment_pending = pending;
      s.segment_pending_seq = pending ? pending_seq : 0;
      s.segment_active = active;
      s.segment_seq = active ? active_seq : 0;
    }
    b.rt.Store(s);
    return s;
  }
};

struct Rig {
  std::shared_ptr<const pinocchio::Model> model;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  pinocchio::FrameIndex frame{0};
  int nv{0};
  Eigen::VectorXd q_ref;  // the catch posture (model = device order here)
  Eigen::Vector3d p_c;
  Eigen::Vector3d v_ball;
  Boxes boxes;
  rtc::catching::PlannerCycle cycle;
  RtStandIn rt;
  rtc::catching::PlannerParams params;
  std::int64_t traj_first_ns{0};
  // What the cycle was configured with, kept for Reconfigure().
  rtc::catching::PlannerCycleIo io{};
  rtc::catching::GridCatchSearchModel search_model{};
  rtc::catching::GridCatchSearchConstants search_consts{};
  rtc::catching::CatchPoseIkOptions ik_options{};
  rtc::catching::MpcSegmentPlannerModel mpc_segment_model{};
  rtc::catching::MpcSegmentPlannerConstants mpc_segment_consts{};
  // The clock the objects are built with — the cycle's own, which installing
  // them hands them again.
  rtc::catching::CatchSearch::ClockFn clock{&rtc::SteadyNowNs};
  // The planner InstallPlanners last installed, for the tests that read what
  // its cores were handed. Owned by the cycle: good until the next
  // InstallPlanners, ClearSegmentPlanner or InstallSegmentPlanner.
  const rtc::catching::MpcSegmentPlanner* mpc_segment_planner{nullptr};

  explicit Rig(double catch_err_max = 0.02, bool bind_segment = true, double w_perp = 0.0,
               bool fake_clock = false) {
    rtc_urdf_bridge::ModelConfig config;
    config.urdf_path = std::string(RTC_TEST_ROBOT_DESCRIPTIONS_DIR) + "/ur5e/urdf/ur5e.urdf";
    config.root_joint_type = "fixed";
    rtc_urdf_bridge::PinocchioModelBuilder builder(config);
    model = builder.GetFullModel();
    handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model);
    frame = model->getFrameId("tool0");
    nv = model->nv;
    q_ref.resize(nv);
    q_ref << 0.2, -1.4, 1.1, -1.9, -1.6, 0.1;
    pinocchio::Data data(*model);
    pinocchio::forwardKinematics(*model, data, q_ref);
    pinocchio::updateFramePlacement(*model, data, frame);
    p_c = data.oMf[frame].translation();
    // A slow ball along the frame's −z: every candidate on the line is a
    // short reach, so the first segment is about the pair, not the reach.
    v_ball = -1.0 * data.oMf[frame].rotation().col(2);
    for (int j = 0; j < nv; ++j) {
      rt.wait_pose.push_back(q_ref[j] + (j % 2 == 0 ? 0.03 : -0.03));
    }

    params.enabled = true;
    params.wait_pose_n = nv;
    for (int j = 0; j < nv; ++j) {
      params.wait_pose[static_cast<std::size_t>(j)] = rt.wait_pose[static_cast<std::size_t>(j)];
    }
    params.t_freeze = 0.2;
    // As a profile without the key gives it: the search goes on while a plan
    // is followed, until the plan freezes.
    params.t_stop_plan = params.t_freeze;
    params.slice_t_lead_min = 0.6;
    params.slice_t_max = 0.8;
    params.n_settle = 0;
    params.d_eff = 0.2;
    params.r_cap = 0.03;
    params.max_ik = 8;
    params.budget_s = 0.05;
    params.catch_box.set = true;
    params.catch_box.min = {-5.0, -5.0, -5.0};
    params.catch_box.max = {5.0, 5.0, 5.0};
    auto& d = params.mpc_segment;
    d.n_nodes = 7;
    d.dt_s = 0.05;
    d.blocks = {1, 1, 2, 3};
    d.n_blocks = 4;
    d.k_max = 2;
    d.n_pre_max = 6;
    d.dt_pre_s = 0.1;
    d.horizon_explicit = true;
    // Generous: a loaded host must not turn this suite into a budget test.
    d.budget_first_s = 0.1;
    d.budget_replan_s = 0.1;
    d.catch_pos_err_max = catch_err_max;
    d.w_perp = w_perp;
    cycle.Configure(params);
    if (fake_clock) {
      clock = &SuiteClock;
      cycle.SetClock(clock);
    }

    rtc::catching::GridCatchSearchModel pm;
    pm.handle = handle.get();
    pm.catch_frame = frame;
    pm.nv = nv;
    rtc::catching::MpcSegmentPlannerModel dm;
    dm.arm = model;
    dm.catch_frame = frame;
    dm.nv = nv;
    const std::array<double, 6> qd_max{2.0, 2.0, 3.0, 3.0, 3.0, 3.0};
    const std::array<double, 6> tau_max{150.0, 150.0, 150.0, 28.0, 28.0, 28.0};
    for (int j = 0; j < nv; ++j) {
      const auto u = static_cast<std::size_t>(j);
      pm.device_of_model[u] = j;
      pm.qdot_max[u] = qd_max[u];
      pm.qddot_max[u] = 20.0;
      dm.device_of_model[u] = j;
      dm.q_min[u] = model->lowerPositionLimit[j];
      dm.q_max[u] = model->upperPositionLimit[j];
      dm.qdot_max[u] = qd_max[u];
      dm.tau_max[u] = tau_max[u];
    }
    pm.accel_box = true;
    rtc::catching::GridCatchSearchConstants pc;
    pc.eta_v = 0.9;
    pc.v_max = 3.0;
    pc.a_dec = 10.0;
    pc.t_arm_s = static_cast<double>(kTArm) * 1e-9;
    pc.t_close_lead = 0.1;
    pc.t_close_total = 0.101;
    pc.ball_mass = 0.057;
    pc.ref_omega = 10.0;
    pc.ref_zeta = 1.0;
    pc.ref_a_max = 21.0;
    pc.control_dt = 0.002;
    rtc::catching::CatchPoseIkOptions ik;
    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
    rtc::catching::MpcSegmentPlannerConstants dc;
    dc.eta_v = 0.9;
    dc.t_arm_s = pc.t_arm_s;
    dc.control_dt = 0.002;
    dc.v_eps = 1e-6;
    io = rtc::catching::PlannerCycleIo{&boxes.traj, &boxes.cov, &boxes.rt, &boxes.plan,
                                       bind_segment ? &boxes.segment : nullptr};
    EXPECT_TRUE(cycle.Bind(io));
    search_model = pm;
    search_consts = pc;
    ik_options = ik;
    mpc_segment_model = dm;
    mpc_segment_consts = dc;
    InstallPlanners();
  }

  // Build the search and the MPC segment planner from what the rig holds and
  // install them in the cycle, each in place of the one there.
  void InstallPlanners() {
    auto search =
        rtc::catching::MakeGridCatchSearch(search_model, search_consts, params, ik_options, clock);
    EXPECT_NE(search, nullptr);
    cycle.InstallSearch(std::move(search));
    std::string err;
    auto planner = rtc::catching::MakeMpcSegmentPlanner(mpc_segment_model, mpc_segment_consts,
                                                        params.mpc_segment, clock, &err);
    EXPECT_NE(planner, nullptr) << err;
    mpc_segment_planner = planner.get();
    cycle.InstallSegmentPlanner(std::move(planner));
  }

  // Configure again a cycle that has already run, with what it was built
  // with. The search is configured over the one in place (no ClearSearch) and
  // the MPC segment planner after a ClearSegmentPlanner, which is how the controller's
  // configure reaches it.
  void Reconfigure() {
    cycle.Configure(params);
    EXPECT_TRUE(cycle.Bind(io));
    cycle.ClearSegmentPlanner();
    EXPECT_FALSE(cycle.SegmentPlannerConfigured());
    InstallPlanners();
  }

  // The ball passes the catch point 0.7 s after the first snapshot; later
  // snapshots of the same track predict the same ball (only the sequence
  // moves), so a replan's ball target is the plan's.
  void StartTrajectory() {
    traj_first_ns = Now();
    StoreTrajectory(1);
  }

  void StoreTrajectory(std::uint64_t seq, std::uint64_t gen = kTrack) {
    const auto t = rtc::testing::LineTrajectory(p_c, v_ball, traj_first_ns, 50 * kMs, 20, 14, seq,
                                                gen, kActivation, Now() - 5 * kMs);
    boxes.cov.Store(rtc::testing::IsotropicCovariance(t, 0.004));
    boxes.traj.Store(t);
  }

  PlannerCycleRecord Wake() {
    const std::int64_t now = Now();
    rt.Tick(boxes, now);
    return cycle.Run(rtc::catching::NowReal{Now()});
  }
};

std::string Why(const PlannerCycleRecord& r) {
  return std::string(rtc::catching::CycleOutcomeName(r.outcome)) + " / decel " +
         SegmentOutcomeName(r.segment.outcome) + " / " + r.segment.core_reason_name;
}

// ── The pair ─────────────────────────────────────────────────────────────────

struct OrderProbe {
  Boxes* boxes{nullptr};
  int calls{0};
  bool segment_already_there{false};
  bool plan_not_yet_there{false};
  std::uint32_t expected_plan_id{0};
};

void ProbeOrder(void* ctx) noexcept {
  auto* p = static_cast<OrderProbe*>(ctx);
  ++p->calls;
  const SegmentSnapshot d = p->boxes->segment.Load();
  const PlanSnapshot plan = p->boxes->plan.Load();
  p->segment_already_there = d.valid && d.plan_id == p->expected_plan_id;
  p->plan_not_yet_there = !plan.valid || plan.plan_id != p->expected_plan_id;
}

TEST(ApproachCycle, APairIsPublishedTogetherSegmentFirst) {
  auto r = std::make_unique<Rig>();
  r->rt.adopt = false;
  r->StartTrajectory();
  OrderProbe probe;
  probe.boxes = &r->boxes;
  probe.expected_plan_id = r->cycle.LastPlanId() + 1;
  r->cycle.SetPairStoreHookForTesting(&ProbeOrder, &probe);
  const PlannerCycleRecord rec = r->Wake();
  r->cycle.SetPairStoreHookForTesting(nullptr, nullptr);
  ASSERT_EQ(rec.outcome, CycleOutcome::kPublished) << Why(rec);
  ASSERT_TRUE(rec.plan_valid);
  EXPECT_TRUE(rec.search_valid);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kPublished);
  EXPECT_EQ(rec.segment.kind, SegmentKind::kFirst);
  EXPECT_TRUE(rec.segment.cold_start);
  // Between the two stores the segment is there and the plan is not.
  EXPECT_EQ(probe.calls, 1);
  EXPECT_TRUE(probe.segment_already_there);
  EXPECT_TRUE(probe.plan_not_yet_there);
  const PlanSnapshot plan = r->boxes.plan.Load();
  const SegmentSnapshot seg = r->boxes.segment.Load();
  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(seg.valid);
  EXPECT_EQ(plan.plan_id, rec.plan_id);
  EXPECT_EQ(seg.plan_id, plan.plan_id);
  EXPECT_EQ(seg.t_c_ns, plan.t_c_ns);
  EXPECT_EQ(seg.publish_ns, plan.publish_ns);
  EXPECT_EQ(seg.publish_ns, rec.publish_ns);
  EXPECT_GT(seg.n_pre, 0);
  EXPECT_EQ(seg.t0_ns, plan.t_c_ns - seg.n_pre * seg.dt_pre_ns);
  EXPECT_TRUE(rtc::catching::ValidateSegmentNodes(seg));
  // Recorded for the sim comparison: the search plus the first solve.
  std::printf("[ record ] pair wake: search %.2f ms, first solve %.2f ms, n_pre %d\n",
              static_cast<double>(rec.search.search_ns) * 1e-6,
              static_cast<double>(rec.segment.solve_ns) * 1e-6, seg.n_pre);
}

TEST(ApproachCycle, AWithheldSegmentWithholdsThePlan) {
  auto r = std::make_unique<Rig>(/*catch_err_max=*/1e-9);
  r->rt.adopt = false;
  r->StartTrajectory();
  const PlannerCycleRecord rec = r->Wake();
  EXPECT_EQ(rec.outcome, CycleOutcome::kHeld) << Why(rec);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kCatchError);
  // The search found a plan; the pair was not published. The two flags are
  // what tells this wake from one whose search found nothing.
  EXPECT_TRUE(rec.search_valid);
  EXPECT_FALSE(rec.plan_valid);
  EXPECT_FALSE(r->boxes.plan.Load().valid);
  EXPECT_FALSE(r->boxes.segment.Load().valid);
  EXPECT_EQ(r->cycle.LastPlanId(), 0U);
  EXPECT_EQ(r->cycle.LastSegmentSeq(), 0U);
}

struct TrajSwap {
  Rig* rig{nullptr};
  std::uint64_t seq{0};
  std::uint64_t gen{kTrack};
};

void StoreNewerTrajectory(void* ctx) noexcept {
  auto* s = static_cast<TrajSwap*>(ctx);
  s->rig->StoreTrajectory(s->seq, s->gen);
}

TEST(ApproachCycle, ANewerSnapshotOfTheSameTrackStillPublishes) {
  // The first solve can outlast a trajectory period: a newer snapshot of the
  // SAME track landing during it must not drop the pair — demanding the same
  // snapshot would drop every one. A new track does.
  auto r = std::make_unique<Rig>();
  r->rt.adopt = false;
  r->StartTrajectory();
  TrajSwap swap{r.get(), 2, kTrack};
  r->cycle.SetPostSegmentHookForTesting(&StoreNewerTrajectory, &swap);
  PlannerCycleRecord rec = r->Wake();
  EXPECT_EQ(rec.outcome, CycleOutcome::kPublished) << Why(rec);
  EXPECT_TRUE(r->boxes.segment.Load().valid);

  auto q = std::make_unique<Rig>();
  q->rt.adopt = false;
  q->StartTrajectory();
  TrajSwap other{q.get(), 2, kTrack + 1};
  q->cycle.SetPostSegmentHookForTesting(&StoreNewerTrajectory, &other);
  rec = q->Wake();
  EXPECT_EQ(rec.outcome, CycleOutcome::kSuperseded) << Why(rec);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kSuperseded);
  EXPECT_FALSE(q->boxes.plan.Load().valid);
  EXPECT_FALSE(q->boxes.segment.Load().valid);
  r->cycle.SetPostSegmentHookForTesting(nullptr, nullptr);
  q->cycle.SetPostSegmentHookForTesting(nullptr, nullptr);
}

TEST(ApproachCycle, RightAfterAPairTheSearchWaits) {
  auto r = std::make_unique<Rig>();
  r->rt.adopt = false;
  r->StartTrajectory();
  const PlannerCycleRecord first = r->Wake();
  ASSERT_EQ(first.outcome, CycleOutcome::kPublished) << Why(first);
  // An RT state that cannot have seen the pair yet (≤ publish + 3h): no search.
  PlannerRtState s = r->boxes.rt.Load();
  s.rt_state_ns = first.publish_ns + 3 * kH;
  r->boxes.rt.Store(s);
  PlannerCycleRecord rec = r->cycle.Run(rtc::catching::NowReal{Now()});
  EXPECT_EQ(rec.outcome, CycleOutcome::kIdle);
  EXPECT_EQ(rec.search.n_ik, 0);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kOff);
  EXPECT_EQ(r->cycle.LastPlanId(), first.plan_id);
  // One that postdates it (the RT did not take the plan): the search resumes.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  rec = r->Wake();
  EXPECT_NE(rec.outcome, CycleOutcome::kIdle) << Why(rec);
  EXPECT_GT(rec.search.n_ik, 0);
}

// ── Replans ──────────────────────────────────────────────────────────────────

TEST(ApproachCycle, WhileFollowingTheSearchGoesOnUnpublishedAndEverySegmentStartsOnTheReport) {
  auto r = std::make_unique<Rig>();
  r->StartTrajectory();
  const PlannerCycleRecord first = r->Wake();
  ASSERT_EQ(first.outcome, CycleOutcome::kPublished) << Why(first);
  const std::int64_t t_c = r->boxes.plan.Load().t_c_ns;
  int replans = 0;
  int same = 0;
  int advance = 0;
  int stop = 0;
  int searched = 0;
  int searched_after_freeze = 0;
  int held_replacements = 0;
  std::int32_t last_k = first.segment.k;
  std::uint64_t seq = 1;
  // Wake every ~15 ms through APPROACH, COMMITTED and DECEL to the end of the
  // replan window, with a new trajectory snapshot every other wake.
  while (Now() < t_c + 4 * 50 * kMs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    if (seq % 2 == 0) {
      r->StoreTrajectory(seq + 1);
    }
    ++seq;
    const std::int64_t wake_ns = Now();
    const PlannerCycleRecord rec = r->Wake();
    const bool ran = rec.search.n_ik > 0;
    searched += ran ? 1 : 0;
    // The search stops where the plan freezes (t_stop_plan = T_freeze here):
    // a wake inside the freeze window is the replan alone.
    searched_after_freeze += ran && t_c - wake_ns <= r->rt.t_freeze_ns ? 1 : 0;
    held_replacements += rec.outcome == CycleOutcome::kHeldReplaceUnsupported ? 1 : 0;
    // Whatever the search found, the RT's plan is the one it took: nothing is
    // stored over it, and its id never moves.
    EXPECT_NE(rec.outcome, CycleOutcome::kPublished) << "a second plan while following";
    EXPECT_EQ(r->boxes.plan.Load().t_c_ns, t_c);
    EXPECT_EQ(r->boxes.plan.Load().plan_id, first.plan_id);
    if (rec.segment.outcome != SegmentOutcome::kPublished) {
      continue;
    }
    ++replans;
    // Every published replan started on a segment the RT reported (path (i)).
    EXPECT_TRUE(rec.segment.x0_from_segment);
    EXPECT_NE(rec.segment.source_seq, 0U);
    // A new grid point or core is a new problem; the same point is not.
    if (rec.segment.kind == SegmentKind::kSame) {
      ++same;
      EXPECT_FALSE(rec.segment.cold_start);
    } else {
      EXPECT_TRUE(rec.segment.cold_start) << rtc::catching::SegmentKindName(rec.segment.kind);
      advance += rec.segment.kind == SegmentKind::kAdvance ? 1 : 0;
      stop += rec.segment.kind == SegmentKind::kStop ? 1 : 0;
    }
    EXPECT_GE(rec.segment.k, last_k) << "the grid point never moves back";
    last_k = rec.segment.k;
  }
  EXPECT_GT(searched, 0) << "the search stopped as soon as the RT followed a plan";
  EXPECT_EQ(searched_after_freeze, 0) << "the search ran inside the freeze window";
  EXPECT_GT(replans, 0);
  EXPECT_GT(same, 0);
  EXPECT_GT(advance, 0);
  EXPECT_GT(stop, 0);
  std::printf(
      "[ record ] replans %d: same %d, advance %d, stop %d; searches while following "
      "%d (%d would have replaced the plan)\n",
      replans, same, advance, stop, searched, held_replacements);
}

// The whole catch's time and prediction, in one place: the stop-line test
// asserts values on it and the digest test pins every bit of it, so the two
// must run the SAME catch. 15 ms per wake; on every other wake vision has
// published a newer snapshot whose prediction drifted by 0.36 mm and 4 mrad;
// the catch runs to the end of the replan window.
struct DriftingCatch {
  Eigen::Vector3d p0;
  Eigen::Vector3d v0;
  Eigen::Vector3d side;
  int update{0};
  std::uint64_t seq{1};

  explicit DriftingCatch(const Rig& r) : p0(r.p_c), v0(r.v_ball), side(r.v_ball.unitOrthogonal()) {}

  // Whether a wake still belongs to the catch whose instant is `t_c`.
  [[nodiscard]] static bool Running(std::int64_t t_c) { return Now() < t_c + 4 * 50 * kMs; }

  // Move to the next wake's instant and store what vision published by then.
  void Step(Rig& r, FakeTime& time) {
    time.Advance(15 * kMs);
    if (seq % 2 == 0) {
      ++update;
      const double a = 0.004 * update;
      r.p_c = p0 + update * Eigen::Vector3d(0.0003, -0.0002, 0.0);
      r.v_ball = v0.norm() * (std::cos(a) * v0.normalized() + std::sin(a) * side);
      r.StoreTrajectory(seq + 1);
    }
    ++seq;
  }
};

TEST(ApproachCycle, TheStopCoresTakeTheFollowedSegmentsLineThroughAWholeCatch) {
  // `cost.w_perp` > 0 (#698): a stop core solves on the line of the segment
  // the RT follows. Which segment that is, and whether a segment has a line at
  // all, hangs on the cycle's own publish (NotePublished) — so a whole catch
  // is run through the cycle: the pair, re-solves and grid advances against a
  // prediction that DRIFTS (every published catch-core segment has a line of
  // its own), then the stop cores. Each stop core must have received exactly
  // the line of the segment it started from.
  //  • RT takes every segment: the stop cores inherit down the chain.
  //  • RT takes only the first (its switch gate refuses the rest): later
  //    lines are published and never followed, and every stop core stays on
  //    the FIRST segment's line.
  // On a pinned clock: the test steps time itself, a solve takes none of it,
  // and no budget, lead or sleep depends on the host.
  struct Line {
    Eigen::Vector3d p;
    Eigen::Vector3d d;
  };

  constexpr std::int64_t kStart = 1'727'000'000'000'000'000LL;
  for (const bool rt_takes_replans : {true, false}) {
    SCOPED_TRACE(rt_takes_replans ? "the RT takes every segment" : "the RT keeps the first");
    FakeTime time(kStart);
    auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_segment=*/true,
                                   /*w_perp=*/2000.0, /*fake_clock=*/true);
    ASSERT_TRUE(r->cycle.SegmentPlannerConfigured());
    const rtc::catching::MpcSegmentPlanner& mpc_segment_planner = *r->mpc_segment_planner;
    DriftingCatch drift(*r);
    r->StartTrajectory();
    const PlannerCycleRecord first = r->Wake();
    ASSERT_EQ(first.outcome, CycleOutcome::kPublished) << Why(first);
    ASSERT_EQ(first.segment.kind, SegmentKind::kFirst);
    const PlanSnapshot plan = r->boxes.plan.Load();
    const std::int64_t t_c = plan.t_c_ns;
    if (!rt_takes_replans) {
      r->rt.take_up_to_seq = first.segment.segment_seq;
    }

    std::map<std::uint32_t, Line> line_of;  // published segment → its line
    // The first segment's line is the PLAN's catch point along its ball.
    {
      const auto& in = mpc_segment_planner.ApproachCoreInput(-first.segment.k);
      const Eigen::Vector3d p(plan.p_c[0], plan.p_c[1], plan.p_c[2]);
      const Eigen::Vector3d v(plan.v_c[0], plan.v_c[1], plan.v_c[2]);
      EXPECT_EQ(in.p_c, p);
      EXPECT_LT((in.d_hat - v.normalized()).norm(), 1e-12);
      line_of[first.segment.segment_seq] = Line{in.p_c, in.d_hat};
    }
    Line newest_catch = line_of[first.segment.segment_seq];
    std::set<std::int32_t> stop_points;
    int stops = 0;
    int stops_off_the_newest_line = 0;
    int catch_segments = 1;
    while (DriftingCatch::Running(t_c)) {
      drift.Step(*r, time);
      const PlannerCycleRecord rec = r->Wake();
      const bool stop = rec.segment.kind == SegmentKind::kStop;
      EXPECT_FALSE(stop && rec.segment.outcome == SegmentOutcome::kNoBall)
          << "a stop grid point whose source had no line";
      if (rec.segment.outcome != SegmentOutcome::kPublished) {
        continue;
      }
      ASSERT_EQ(line_of.count(rec.segment.source_seq), 1U) << rec.segment.source_seq;
      if (!stop) {
        // A catch-core segment: the line of THIS wake's ball at t_c — the
        // trajectory in the box, evaluated here from what the rig stored.
        const auto& in = mpc_segment_planner.ApproachCoreInput(-rec.segment.k);
        const double dt = static_cast<double>(t_c - (r->traj_first_ns + 14 * 50 * kMs)) / 1e9;
        EXPECT_LT((in.p_c - (r->p_c + r->v_ball * dt)).norm(), 1e-9);
        EXPECT_LT((in.d_hat - r->v_ball.normalized()).norm(), 1e-9);
        newest_catch = Line{in.p_c, in.d_hat};
        line_of[rec.segment.segment_seq] = newest_catch;
        ++catch_segments;
        continue;
      }
      const Line& followed = line_of[rec.segment.source_seq];
      const auto& in = mpc_segment_planner.StopCoreInput(rec.segment.k);
      EXPECT_EQ(in.p_c, followed.p) << "stop k " << rec.segment.k;
      EXPECT_EQ(in.d_hat, followed.d) << "stop k " << rec.segment.k;
      line_of[rec.segment.segment_seq] = followed;  // a stop segment inherits
      stop_points.insert(rec.segment.k);
      ++stops;
      stops_off_the_newest_line += (in.p_c - newest_catch.p).norm() > 1e-4 ? 1 : 0;
      if (!rt_takes_replans) {
        EXPECT_EQ(rec.segment.source_seq, first.segment.segment_seq);
      }
    }
    // Every stop grid point of the replan window was solved (k_max 2).
    EXPECT_EQ(stop_points, (std::set<std::int32_t>{0, 1, 2}));
    ASSERT_GT(catch_segments, 3);  // the drift did produce other lines
    if (!rt_takes_replans) {
      // The discriminating case: the newest PUBLISHED catch-core line is not
      // the followed one, and no stop core took it.
      EXPECT_GT((newest_catch.p - line_of[first.segment.segment_seq].p).norm(), 1e-3);
      EXPECT_EQ(stops_off_the_newest_line, stops);
    }
    std::printf(
        "[ record ] w_perp on, RT %s: %d catch-core segments, %d stop segments, %d of "
        "them off the newest published line\n",
        rt_takes_replans ? "takes all" : "keeps the first", catch_segments, stops,
        stops_off_the_newest_line);
  }
}

// ── The whole catch, bit for bit (E1-F12 #738) ───────────────────────────────

// One catch as the stop-line test above runs it (DriftingCatch) — the pair,
// re-solves and grid advances against a drifting prediction, the freeze, the
// stop cores —
// with every wake's record and what the two boxes hold after it digested
// (planner_trace_digest.hpp: values, field by field).
struct CatchTrace {
  std::uint64_t digest{0};
  int wakes{0};
  int searches{0};       // wakes whose search ran an IK
  int monitor_wakes{0};  // the RT in COMMITTED
  int decel_wakes{0};    // the RT in DECEL
  int plans{0};          // published
  int segments{0};       // published
  int stop_segments{0};
};

CatchTrace RunCatch(Rig& r, FakeTime& time, bool rt_takes_replans) {
  CatchTrace t;
  rtc::testing::ValueDigest h;
  const auto note = [&](const PlannerCycleRecord& rec) {
    rtc::testing::AddCycleRecord(h, rec);
    rtc::testing::AddPlan(h, r.boxes.plan.Load());
    rtc::testing::AddSegment(h, r.boxes.segment.Load());
    ++t.wakes;
    t.searches += rec.search.n_ik > 0 ? 1 : 0;
    t.monitor_wakes += static_cast<Mode>(rec.mode) == Mode::kCommitted ? 1 : 0;
    t.decel_wakes += static_cast<Mode>(rec.mode) == Mode::kDecel ? 1 : 0;
    t.plans += rec.outcome == CycleOutcome::kPublished && rec.plan_valid ? 1 : 0;
    const bool segment = rec.segment.outcome == SegmentOutcome::kPublished;
    t.segments += segment ? 1 : 0;
    t.stop_segments += segment && rec.segment.kind == SegmentKind::kStop ? 1 : 0;
  };
  DriftingCatch drift(r);
  r.StartTrajectory();
  const PlannerCycleRecord first = r.Wake();
  note(first);
  EXPECT_EQ(first.outcome, CycleOutcome::kPublished) << Why(first);
  const std::int64_t t_c = r.boxes.plan.Load().t_c_ns;
  if (!rt_takes_replans) {
    r.rt.take_up_to_seq = first.segment.segment_seq;
  }
  while (first.outcome == CycleOutcome::kPublished && DriftingCatch::Running(t_c)) {
    drift.Step(r, time);
    note(r.Wake());
  }
  // Back to the first prediction, for a rig that runs another catch.
  r.p_c = drift.p0;
  r.v_ball = drift.v0;
  t.digest = h.Value();
  return t;
}

// What happened to the rig before the catch that is digested.
enum class RigHistory : std::uint8_t {
  kBuilt = 0,          // nothing: a rig just built
  kResetReconfigured,  // a catch, then a trial reset and a re-configure
  kReconfigured,       // then another catch and a re-configure with NO trial reset
};

/// RunCatch's digests on the rig below: [RT takes every segment, RT keeps the
/// first] × RigHistory. Retaken when the search began to run while a plan is
/// followed (E1-F16 #742; the rig stops it at T_freeze): every wake of that
/// stretch now records a search where it recorded none. That is the ONLY thing
/// that moved them — with the search stopped at the first followed plan
/// (`t_stop_plan` left unset) the same code gives the six numbers they
/// replace, 0x74b66752c2b6a927, 0x47a6055faba0f13a, 0xf994bd4e88fc8518 /
/// 0xb7132fe2c7363186, 0xb945046f48c1f383, 0x231a91137a556e3c, bit for bit
/// (measured). Those were taken on the code BEFORE the search and the MPC
/// segment planner moved behind CatchSearch / SegmentPlanner (E1-F12 #738),
/// when the cycle held both by value and a re-configure re-used the same two
/// objects, so the chain back to that code is unbroken.
/// The two re-configured columns are the cycle-level half of "building NEW
/// objects on a re-configure changes nothing" — the last one without a trial
/// reset in between. They are NOT what shows that Configure alone clears what
/// a catch left behind: this catch's trace does not depend on that (measured —
/// with both Configure-time resets removed all six numbers stay). That half is
/// pinned on the two classes directly: AReconfiguredSearchIsANewOne
/// (test_catching_grid_catch_search.cpp) and AReconfiguredPlannerIsANewOne
/// (test_catching_approach_planner.cpp).
///
/// Numbers to COMPARE AGAINST, not a claim that these segments are the right
/// ones — the rule of kNormalTrialCommandDigest. They include the QP's
/// iterates, pinocchio's kinematics and libm's sin / cos, so they are tied to
/// where they were taken: this host's CPU, GCC on x86-64 with the repo's flags
/// (no -march, -ffast-math or -ffp-contract), and the installed Eigen,
/// pinocchio and ProxSuite. Two consequences:
///  - a change meant to alter what a wake publishes or records — the search,
///    the cores, the cycle's order — replaces them in a commit of its own that
///    says why (PROC-6); a refactor that is not meant to must leave them alone;
///  - red on ANOTHER host, or right after one of those libraries was upgraded,
///    with no change to this package, is the environment and not a regression:
///    take the same commit's numbers on the recording host before concluding
///    anything, and replace them the same way if the environment is what moved.
constexpr std::array<std::array<std::uint64_t, 3>, 2> kWholeCatchDigest{{
    {{0x98d32eb27cf530f5ULL, 0x65c28c576069d158ULL, 0x849daff1596f26acULL}},
    {{0x44e5a74f2f581dfcULL, 0xa3bab7d8edad0ec5ULL, 0x7f81e44215a9e1ecULL}},
}};

TEST(ApproachCycle, AWholeCatchOnThePinnedClockIsUnchangedBitForBit) {
  constexpr std::int64_t kStart = 1'727'000'000'000'000'000LL;
  for (const bool rt_takes_replans : {true, false}) {
    SCOPED_TRACE(rt_takes_replans ? "the RT takes every segment" : "the RT keeps the first");
    const auto& expected = kWholeCatchDigest[rt_takes_replans ? 0U : 1U];
    FakeTime time(kStart);
    auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_segment=*/true,
                                   /*w_perp=*/2000.0, /*fake_clock=*/true);
    for (const RigHistory history :
         {RigHistory::kBuilt, RigHistory::kResetReconfigured, RigHistory::kReconfigured}) {
      const char* const name = history == RigHistory::kBuilt               ? "built"
                               : history == RigHistory::kResetReconfigured ? "reset_reconfigured"
                                                                           : "reconfigured";
      SCOPED_TRACE(name);
      if (history != RigHistory::kBuilt) {
        // The catch before is over: the RT drops its plan, and the plan box no
        // longer holds the one it followed (the RT refuses that one by age and
        // reset floor; the stand-in has neither check). With a trial reset the
        // RT also moves its reset epoch, and the cycle's first wake drops both
        // objects' per-trial memory itself; without one, only the re-configure
        // stands between the last catch and this one.
        time.Advance(500 * kMs);
        RtStandIn next;
        next.wait_pose = r->rt.wait_pose;
        next.reset_epoch =
            r->rt.reset_epoch + (history == RigHistory::kResetReconfigured ? 1U : 0U);
        r->rt = next;
        r->boxes.plan.Store(PlanSnapshot{});
        r->Reconfigure();
      }
      const CatchTrace t = RunCatch(*r, time, rt_takes_replans);
      // The trace went through every kind of wake the cycle has.
      EXPECT_EQ(t.plans, 1);
      EXPECT_GT(t.searches, 1) << "the search stopped as soon as the RT followed a plan";
      EXPECT_GT(t.monitor_wakes, 0);
      EXPECT_GT(t.decel_wakes, 0);
      EXPECT_GT(t.segments, 3);
      EXPECT_GT(t.stop_segments, 0);
      std::array<char, 32> hex{};
      std::snprintf(hex.data(), hex.size(), "0x%016llx", static_cast<unsigned long long>(t.digest));
      const std::string key = std::string("whole_catch_digest_") +
                              (rt_takes_replans ? "takes_all_" : "keeps_first_") + name;
      RecordProperty(key, hex.data());
      std::printf("[ record ] %s: %s — %d wakes (%d COMMITTED, %d DECEL), %d segments (%d stop)\n",
                  key.c_str(), hex.data(), t.wakes, t.monitor_wakes, t.decel_wakes, t.segments,
                  t.stop_segments);
      EXPECT_EQ(t.digest, expected[static_cast<std::size_t>(history)])
          << "the whole-catch trace changed: digest " << hex.data()
          << ". If the change is meant to alter what a wake publishes or records, replace the "
             "constant in its own commit with the reason; otherwise the refactor is not "
             "behaviour-preserving. (Another host or upgraded libraries: see kWholeCatchDigest.)";
    }
  }
}

TEST(ApproachCycle, AnotherTracksBallIsNotTheFollowedPlansTarget) {
  // The RT keeps a followed plan when vision starts another track, and from
  // then on reports THAT track as the one it consumed last. The trajectory box
  // holds the other ball too — it must not become this catch's target.
  auto r = std::make_unique<Rig>();
  r->StartTrajectory();
  ASSERT_EQ(r->Wake().outcome, CycleOutcome::kPublished);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  PlannerCycleRecord rec = r->Wake();
  ASSERT_EQ(rec.segment.outcome, SegmentOutcome::kPublished) << Why(rec);
  EXPECT_EQ(r->boxes.segment.Load().token.generation, kTrack);
  const std::uint32_t seq = r->cycle.LastSegmentSeq();
  r->rt.track = kTrack + 1;
  r->StoreTrajectory(50, kTrack + 1);
  for (int i = 0; i < 4; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    rec = r->Wake();
    ASSERT_LT(rec.segment.k, 0) << "the wakes are meant to fall before the catch";
    EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kNoBall) << Why(rec);
  }
  EXPECT_EQ(r->cycle.LastSegmentSeq(), seq);
  // The plan's own track again: the replans resume.
  r->StoreTrajectory(51, kTrack);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  rec = r->Wake();
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kPublished) << Why(rec);
}

TEST(ApproachCycle, WithoutAReportNothingIsReplanned) {
  // An RT that took the plan but reports no segment — neither pending nor
  // followed (it dropped the one it had): the planner finds no source, and
  // does not infer one (MD-58).
  auto r = std::make_unique<Rig>();
  r->rt.report_segment = false;
  r->StartTrajectory();
  ASSERT_EQ(r->Wake().outcome, CycleOutcome::kPublished);
  const std::uint32_t seq = r->cycle.LastSegmentSeq();
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const PlannerCycleRecord rec = r->Wake();
    EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kNotFollowed) << Why(rec);
  }
  EXPECT_EQ(r->cycle.LastSegmentSeq(), seq);
}

// ── A followed plan is replaced (E1-F17 #743) ────────────────────────────────
//
// The real search and the real segment planner behind the cycle, on a pinned
// clock (FakeTime): values are asserted, not budgets. In this rig the search's
// lead window is 0.6 – 0.8 s, so the plan the RT follows leaves it a tenth of a
// second after the first pair and the search chooses a later sample — another
// plan. RtStandIn does not model a replacement (it follows the first plan and
// ignores a segment of any other): to the cycle it is an RT that dropped every
// pair, and the two reports it never gives are built here by hand.

constexpr std::int64_t kPinnedStart = 1'727'000'000'000'000'000LL;

// Whether a wake published a plan with its first segment.
[[nodiscard]] bool PublishedPair(const PlannerCycleRecord& rec) {
  return rec.outcome == CycleOutcome::kPublished && rec.plan_valid &&
         rec.segment.outcome == SegmentOutcome::kPublished &&
         rec.segment.kind == SegmentKind::kFirst;
}

// 15 ms later, with a newer snapshot of the same prediction: the next wake.
PlannerCycleRecord NextWake(Rig& r, FakeTime& time, std::uint64_t& seq) {
  time.Advance(15 * kMs);
  r.StoreTrajectory(++seq);
  return r.Wake();
}

// What the cycle has stored so far.
struct StoreCounts {
  std::uint64_t plans{0};
  std::uint64_t segments{0};

  explicit StoreCounts(const Rig& r)
      : plans(r.boxes.plan.sequence()), segments(r.boxes.segment.sequence()) {}

  friend bool operator==(const StoreCounts&, const StoreCounts&) = default;
};

TEST(ApproachCycleReplacement, TheSearchsOtherPlanGoesOutAsAPairBesideTheFollowedPlansSegments) {
  FakeTime time(kPinnedStart);
  auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_segment=*/true, /*w_perp=*/0.0,
                                 /*fake_clock=*/true);
  const rtc::catching::MpcSegmentPlanner& planner = *r->mpc_segment_planner;
  r->StartTrajectory();
  const PlannerCycleRecord first = r->Wake();
  ASSERT_TRUE(PublishedPair(first)) << Why(first);
  const PlanSnapshot plan1 = r->boxes.plan.Load();

  // Wake until the search replaces the plan the RT follows.
  OrderProbe probe;
  probe.boxes = &r->boxes;
  probe.expected_plan_id = first.plan_id + 1;
  r->cycle.SetPairStoreHookForTesting(&ProbeOrder, &probe);
  std::uint64_t seq = 1;
  PlannerCycleRecord rec{};
  int wakes = 0;
  do {
    rec = NextWake(*r, time, seq);
    ++wakes;
  } while (wakes < 20 && !PublishedPair(rec));
  r->cycle.SetPairStoreHookForTesting(nullptr, nullptr);
  ASSERT_TRUE(PublishedPair(rec)) << "no replacement in " << wakes << " wakes: " << Why(rec);
  const std::int64_t now = Now();
  const PlannerRtState report = r->boxes.rt.Load();  // what the wake started from
  ASSERT_TRUE(report.plan_active);
  ASSERT_EQ(report.plan_id, plan1.plan_id);
  ASSERT_EQ(static_cast<Mode>(report.mode), Mode::kApproach);

  // The search ran first and its verdict was another plan.
  EXPECT_GT(rec.search.n_ik, 0);
  EXPECT_EQ(rec.search.decision, rtc::catching::SwitchDecision::kReplaced);
  EXPECT_TRUE(rec.search_valid);
  EXPECT_EQ(rec.replacement.outcome, SegmentOutcome::kOff);
  // A pair: a new plan id, one stamp, the segment stored first.
  const PlanSnapshot plan2 = r->boxes.plan.Load();
  const SegmentSnapshot seg2 = r->boxes.segment.Load();
  ASSERT_TRUE(plan2.valid);
  ASSERT_TRUE(seg2.valid);
  EXPECT_EQ(rec.plan_id, plan1.plan_id + 1);
  EXPECT_EQ(plan2.plan_id, rec.plan_id);
  EXPECT_NE(plan2.t_c_ns, plan1.t_c_ns);
  EXPECT_EQ(seg2.plan_id, plan2.plan_id);
  EXPECT_EQ(seg2.t_c_ns, plan2.t_c_ns);
  EXPECT_EQ(seg2.segment_seq, rec.segment.segment_seq);
  EXPECT_EQ(seg2.publish_ns, plan2.publish_ns);
  EXPECT_EQ(seg2.publish_ns, rec.publish_ns);
  EXPECT_EQ(rec.publish_ns, now);
  EXPECT_EQ(probe.calls, 1);
  EXPECT_TRUE(probe.segment_already_there);
  EXPECT_TRUE(probe.plan_not_yet_there);
  // On the NEW plan's grid, before the followed plan freezes, and readable.
  EXPECT_GT(seg2.n_pre, 0);
  EXPECT_EQ(seg2.t0_ns, plan2.t_c_ns - seg2.n_pre * seg2.dt_pre_ns);
  EXPECT_LT(seg2.t0_ns, plan1.t_c_ns - r->rt.t_freeze_ns);
  EXPECT_GT(plan2.t_c_ns - rec.publish_ns, r->rt.t_freeze_ns);
  EXPECT_TRUE(planner.StartsInTime(rec.publish_ns, seg2.t0_ns));
  EXPECT_TRUE(rtc::catching::ValidateSegmentNodes(seg2));
  // Its first segment starts on the segment the RT reported for node 0 — the
  // arm is moving along the followed plan — and says which.
  EXPECT_TRUE(rec.segment.x0_from_segment);
  ASSERT_NE(rec.segment.source_seq, 0U);
  EXPECT_EQ(rec.segment.source_seq, planner.SourceSeq(report, seg2.t0_ns));
  auto reported = std::make_unique<rtc::catching::ReportedSegments>();
  planner.Reported(report, *reported);
  const SegmentSnapshot* const source = rtc::catching::SourceSegmentAt(*reported, seg2.t0_ns);
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->segment_seq, rec.segment.source_seq);
  EXPECT_EQ(source->plan_id, plan1.plan_id);
  std::array<double, rtc::catching::kMaxSegmentNv> q{};
  std::array<double, rtc::catching::kMaxSegmentNv> qd{};
  std::array<double, rtc::catching::kMaxSegmentNv> qdd{};
  ASSERT_TRUE(rtc::catching::NodeTrajectoryFollower::SampleJoints(*source, seg2.t0_ns, q, qd, qdd));
  if (!rec.segment.x0_clamped) {
    for (int j = 0; j < r->nv; ++j) {
      const auto u = static_cast<std::size_t>(j);
      EXPECT_NEAR(seg2.q[u], q[u], 1e-9) << j;
      EXPECT_NEAR(seg2.qd[u], qd[u], 1e-6) << j;
    }
  }

  // The followed plan's segments are still the planner's: the RT's report of
  // the old plan finds its source and its track as before the pair.
  std::uint64_t track = 0;
  EXPECT_TRUE(planner.FollowedTrack(report, track));
  EXPECT_EQ(track, kTrack);
  EXPECT_EQ(planner.SourceSeq(report, seg2.t0_ns), rec.segment.source_seq);
  // And the report of an RT that HOLDS the pair names both plans' segments.
  PlannerRtState held = report;
  held.plan_pending = true;
  held.plan_pending_id = plan2.plan_id;
  held.plan_pending_t_c_ns = plan2.t_c_ns;
  held.segment_pending = true;
  held.segment_pending_seq = seg2.segment_seq;
  EXPECT_EQ(planner.SourceSeq(held, seg2.t0_ns), seg2.segment_seq);
  planner.Reported(held, *reported);
  ASSERT_TRUE(reported->has_pending);
  EXPECT_EQ(reported->pending.plan_id, plan2.plan_id);
  if (report.segment_active) {
    ASSERT_TRUE(reported->has_following);
    EXPECT_EQ(reported->following.plan_id, plan1.plan_id);
  }

  // Right after the pair nothing is stored: the RT's report (still the old
  // plan, nothing held) cannot show yet what it did with it.
  {
    const StoreCounts before(*r);
    const PlannerCycleRecord quiet = r->Wake();  // the same instant: the report is the stamp's
    EXPECT_EQ(quiet.outcome, CycleOutcome::kIdle) << Why(quiet);
    EXPECT_EQ(quiet.search.n_ik, 0);
    EXPECT_EQ(quiet.segment.outcome, SegmentOutcome::kOff);
    EXPECT_EQ(StoreCounts(*r), before);
  }
  // The RT holds the pair until its node 0: a wake runs neither the search nor
  // a replan.
  {
    time.Advance(15 * kMs);
    r->StoreTrajectory(++seq);
    held.rt_state_ns = Now();
    held.rt_iteration = static_cast<std::uint64_t>(Now() / kH);
    r->boxes.rt.Store(held);
    const StoreCounts before(*r);
    const PlannerCycleRecord quiet = r->cycle.Run(rtc::catching::NowReal{Now()});
    EXPECT_EQ(quiet.outcome, CycleOutcome::kHeld) << Why(quiet);
    EXPECT_EQ(quiet.search.n_ik, 0);
    EXPECT_FALSE(quiet.search_valid);
    EXPECT_EQ(quiet.segment.outcome, SegmentOutcome::kOff);
    EXPECT_EQ(StoreCounts(*r), before);
  }
  // The RT dropped the pair: its report is the old plan again with nothing
  // held (what RtStandIn reports), and the search may replace it again — under
  // a new id.
  wakes = 0;
  do {
    rec = NextWake(*r, time, seq);
    ++wakes;
  } while (wakes < 10 && !PublishedPair(rec));
  ASSERT_TRUE(PublishedPair(rec)) << "no second replacement in " << wakes << " wakes: " << Why(rec);
  EXPECT_EQ(rec.plan_id, plan2.plan_id + 1);
  EXPECT_EQ(r->boxes.plan.Load().plan_id, plan2.plan_id + 1);
  EXPECT_EQ(r->boxes.segment.Load().plan_id, plan2.plan_id + 1);
  EXPECT_TRUE(rec.segment.x0_from_segment);
}

// What the cycle's two hooks saw of one wake: the search's end, and each
// publishable segment solve's end.
struct WakeOrder {
  int searches{0};
  int solves_before_the_search{0};
  int solves_after_the_search{0};
};

// What the following wakes of one APPROACH came to.
struct FollowingWakes {
  int searched{0};
  int pairs{0};
  int dropped{0};  // at the pair's re-check
  int replans_behind_a_search{0};
  int held_by_the_freeze{0};  // of those: a replacement too late to start
  int withheld{0};            // of those: a replacement whose first solve was withheld
};

// Run the rig's catch from its first pair to the followed plan's freeze, a
// wake every 15 ms, and hold EVERY wake that searched to the rule: it ends in
// exactly one of a pair (published, or dropped at its re-check) or the replan
// of the followed plan, and whichever solve it ran came AFTER the search. A
// replacement is held without a solve exactly when no first segment could
// start before the followed plan freezes.
FollowingWakes RunFollowingApproach(Rig& r, FakeTime& time) {
  FollowingWakes n;
  const rtc::catching::MpcSegmentPlanner& planner = *r.mpc_segment_planner;
  r.StartTrajectory();
  const PlannerCycleRecord first = r.Wake();
  EXPECT_TRUE(PublishedPair(first)) << Why(first);
  if (!PublishedPair(first)) {
    return n;
  }
  const std::int64_t t_c = r.boxes.plan.Load().t_c_ns;
  const std::int64_t followed_freeze = t_c - r.rt.t_freeze_ns;
  WakeOrder order;
  r.cycle.SetPostSearchHookForTesting(
      [](void* user) noexcept { ++static_cast<WakeOrder*>(user)->searches; }, &order);
  r.cycle.SetPostSegmentHookForTesting(
      [](void* user) noexcept {
        auto* o = static_cast<WakeOrder*>(user);
        ++(o->searches > 0 ? o->solves_after_the_search : o->solves_before_the_search);
      },
      &order);
  std::uint64_t seq = 1;
  std::uint32_t last_plan_id = first.plan_id;
  while (Now() < followed_freeze) {
    order = WakeOrder{};
    const PlannerCycleRecord rec = NextWake(r, time, seq);
    const std::int64_t now = Now();
    if (static_cast<Mode>(rec.mode) != Mode::kApproach || order.searches == 0) {
      continue;
    }
    ++n.searched;
    SCOPED_TRACE("wake " + std::to_string((now - kPinnedStart) / kMs) + " ms: " + Why(rec));
    EXPECT_EQ(order.searches, 1);
    EXPECT_EQ(order.solves_before_the_search, 0) << "a segment was solved before the search";
    EXPECT_LE(order.solves_after_the_search, 1);
    const bool replace = rec.search_valid && rec.search.publish &&
                         rec.search.decision == rtc::catching::SwitchDecision::kReplaced;
    if (rec.outcome == CycleOutcome::kPublished) {
      ++n.pairs;
      EXPECT_TRUE(replace);
      EXPECT_TRUE(PublishedPair(rec));
      EXPECT_EQ(rec.plan_id, last_plan_id + 1);
      last_plan_id = rec.plan_id;
      EXPECT_EQ(rec.replacement.outcome, SegmentOutcome::kOff);
      EXPECT_EQ(r.boxes.segment.Load().plan_id, rec.plan_id);
      continue;
    }
    if (rec.outcome == CycleOutcome::kSuperseded) {
      ++n.dropped;
      EXPECT_TRUE(replace);
      EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kSuperseded);
      EXPECT_EQ(rec.segment.kind, SegmentKind::kFirst);
      continue;
    }
    // Nothing of another plan went out: the followed plan's segment was
    // replanned (the replan may itself be withheld — it ran).
    EXPECT_EQ(rec.outcome, CycleOutcome::kHeld);
    EXPECT_EQ(r.boxes.plan.Load().plan_id, last_plan_id);
    EXPECT_NE(rec.segment.kind, SegmentKind::kFirst);
    EXPECT_NE(rec.segment.outcome, SegmentOutcome::kOff);
    ++n.replans_behind_a_search;
    if (!replace) {
      EXPECT_EQ(rec.replacement.outcome, SegmentOutcome::kOff);
      continue;
    }
    // The search chose another plan and no pair came of it: the first solve
    // withheld it (its account is kept), or — and only then is there no
    // account — no first segment could start before the followed plan freezes.
    const bool too_late = !(planner.EarliestFirstStartNs(now) < followed_freeze);
    EXPECT_EQ(rec.replacement.outcome == SegmentOutcome::kOff, too_late);
    n.held_by_the_freeze += too_late ? 1 : 0;
    n.withheld += too_late ? 0 : 1;
    if (!too_late) {
      EXPECT_EQ(rec.replacement.kind, SegmentKind::kFirst);
    }
  }
  r.cycle.SetPostSearchHookForTesting(nullptr, nullptr);
  r.cycle.SetPostSegmentHookForTesting(nullptr, nullptr);
  std::printf(
      "[ record ] following wakes that searched %d: %d pairs, %d dropped at the re-check, %d "
      "replans behind the search (%d of them a replacement too late to start, %d a withheld "
      "first solve)\n",
      n.searched, n.pairs, n.dropped, n.replans_behind_a_search, n.held_by_the_freeze, n.withheld);
  return n;
}

TEST(ApproachCycleReplacement, AFollowingWakeSearchesFirstThenPairsOrReplans) {
  FakeTime time(kPinnedStart);
  auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_segment=*/true, /*w_perp=*/0.0,
                                 /*fake_clock=*/true);
  const FollowingWakes n = RunFollowingApproach(*r, time);
  EXPECT_GT(n.searched, 0);
  EXPECT_GT(n.pairs, 0) << "the catch never replaced its plan: the pair path was not run";
  EXPECT_GT(n.replans_behind_a_search, n.held_by_the_freeze + n.withheld)
      << "no wake whose search kept the plan: the plain replan behind a search was not run";
}

TEST(ApproachCycleReplacement, AReplacementThatCouldNotStartBeforeTheFreezeIsNotSolvedFor) {
  // The same catch with a first-solve budget of 0.3 s: a first segment starts
  // 0.354 s after a wake at the earliest, so from 0.146 s into the catch no
  // replacement could start before the followed plan freezes (0.5 s) — while
  // the search goes on replacing the plan for a while longer. Those wakes run
  // no first solve and replan the followed plan's segment.
  FakeTime time(kPinnedStart);
  auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_segment=*/true, /*w_perp=*/0.0,
                                 /*fake_clock=*/true);
  r->params.mpc_segment.budget_first_s = 0.3;
  r->Reconfigure();
  ASSERT_EQ(r->mpc_segment_planner->EarliestFirstStartNs(kPinnedStart),
            kPinnedStart + kTArm + 300 * kMs + 2 * kH);
  const FollowingWakes n = RunFollowingApproach(*r, time);
  EXPECT_GT(n.searched, 0);
  EXPECT_GT(n.held_by_the_freeze, 0)
      << "no replacement fell between the first-solve lead and the freeze";
}

// ── Lifetime ─────────────────────────────────────────────────────────────────

TEST(ApproachCycle, ATrialResetWithdrawsTheSegment) {
  // The planner is the segment box's only writer: the ended trial's segment
  // must not stay in it (the RT's plan match refuses it as well).
  auto r = std::make_unique<Rig>();
  r->StartTrajectory();
  ASSERT_EQ(r->Wake().outcome, CycleOutcome::kPublished);
  ASSERT_TRUE(r->boxes.segment.Load().valid);
  PlannerRtState s = r->boxes.rt.Load();
  s.reset_epoch = 2;
  s.mode = static_cast<std::uint8_t>(Mode::kIdle);
  s.plan_active = false;
  s.rt_state_ns = Now();
  r->boxes.rt.Store(s);
  const PlannerCycleRecord rec = r->cycle.Run(rtc::catching::NowReal{Now()});
  EXPECT_TRUE(rec.reset_seen);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kOff);
  EXPECT_FALSE(r->boxes.segment.Load().valid) << "the ended trial's segment stays in the box";
}

TEST(ApproachCycle, WithoutASegmentBoxThePlanIsPublishedAlone) {
  // No fifth box (a binding without the segment lane): the MPC segment planner is
  // configured but solves nothing, and the search's plan goes out by itself.
  auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_segment=*/false);
  ASSERT_TRUE(r->cycle.SegmentPlannerConfigured());
  r->rt.adopt = false;
  r->StartTrajectory();
  const PlannerCycleRecord rec = r->Wake();
  ASSERT_EQ(rec.outcome, CycleOutcome::kPublished) << Why(rec);
  EXPECT_TRUE(rec.plan_valid);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kOff);
  EXPECT_TRUE(r->boxes.plan.Load().valid);
  EXPECT_EQ(r->boxes.segment.sequence(), 0U);
  EXPECT_EQ(r->cycle.LastSegmentSeq(), 0U);
  r->cycle.ClearSegmentPlanner();
  EXPECT_FALSE(r->cycle.SegmentPlannerConfigured());
}

}  // namespace
