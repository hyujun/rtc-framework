// E1-F08 (#661): the APPROACH–stop pair and replans as PlannerCycle runs them
// (planner_cycle.hpp "THE DECEL PLANNER'S PART") — the real search on a real
// 6-joint arm, the decel planner behind it, on the REAL steady clock (the
// budgets are measured), with the test as the RT stand-in. RUN_SERIAL: a
// loaded host stretches the solves the budgets judge. The planner's own
// decisions are test_catching_approach_planner.cpp; the core alone is E1-F07's
// test_catching_decel_mpc_approach.cpp.
//   pair        APairIsPublishedTogetherSegmentFirst, AWithheldSegmentWithholdsThePlan,
//               ANewerSnapshotOfTheSameTrackStillPublishes, RightAfterAPairTheSearchWaits
//   replans     OnceFollowingTheSearchStopsAndEverySegmentStartsOnTheReport,
//               AnotherTracksBallIsNotTheFollowedPlansTarget,
//               WithoutAReportNothingIsReplanned
//   lifetime    ATrialResetWithdrawsTheSegment,
//               WithoutADecelBoxThePlanIsPublishedAlone
//
// The RT stand-in reports the segment it holds pending and then follows, as
// the RT does under `mode: mpc` (E1-F09); WithoutAReportNothingIsReplanned
// pins an RT that reports neither.
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controllers/catching/decel_planner.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/planner_search_fixture.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace {

using rtc::catching::CycleOutcome;
using rtc::catching::DecelKind;
using rtc::catching::DecelOutcome;
using rtc::catching::DecelOutcomeName;
using rtc::catching::DecelPlanSnapshot;
using rtc::catching::Mode;
using rtc::catching::PlannerCycleRecord;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kTArm = 50 * kMs;  // the sim profile catch_lead_on's T_arm
constexpr std::int64_t kH = 2 * kMs;
constexpr std::uint64_t kActivation = 3;
constexpr std::uint64_t kTrack = 7;

std::int64_t Now() {
  return rtc::SteadyNowNs();
}

struct Boxes {
  rtc::SeqLock<rtc::catching::TrajectorySnapshot> traj;
  rtc::SeqLock<rtc::catching::CovarianceSnapshot> cov;
  rtc::SeqLock<PlannerRtState> rt;
  rtc::SeqLock<PlanSnapshot> plan;
  rtc::SeqLock<DecelPlanSnapshot> decel;
};

// The RT side under `mode: mpc`: adopt the first plan in the box, hold
// each newer segment of that plan pending until its node 0, then follow it.
// Mode by time: APPROACH, COMMITTED from t_c − T_freeze, DECEL from t_c.
struct RtStandIn {
  std::vector<double> wait_pose;  // device order
  bool adopt{true};               // false: never takes a plan
  bool report_decel{true};        // false: an RT that reports no segment
  std::int64_t t_freeze_ns{200 * kMs};
  std::uint64_t track{kTrack};  // the track it consumed last
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
    if (following && report_decel) {
      const DecelPlanSnapshot d = b.decel.Load();
      if (d.valid && d.plan_id == plan_id && d.decel_seq > last_seq) {
        last_seq = d.decel_seq;
        pending = true;
        pending_seq = d.decel_seq;
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
    s.reset_epoch = 1;
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
      s.decel_pending = pending;
      s.decel_pending_seq = pending ? pending_seq : 0;
      s.decel_active = active;
      s.decel_seq = active ? active_seq : 0;
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

  explicit Rig(double catch_err_max = 0.02, bool bind_decel = true) {
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
    auto& d = params.decel;
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
    cycle.Configure(params);

    rtc::catching::PlannerModel pm;
    pm.handle = handle.get();
    pm.catch_frame = frame;
    pm.nv = nv;
    rtc::catching::DecelPlannerModel dm;
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
    rtc::catching::PlannerConstants pc;
    pc.eta_v = 0.9;
    pc.v_max = 3.0;
    pc.a_dec = 10.0;
    pc.t_arm_s = static_cast<double>(kTArm) * 1e-9;
    pc.t_close_e2e = 0.1;
    pc.t_close_total = 0.101;
    pc.ball_mass = 0.057;
    pc.ref_omega = 10.0;
    pc.ref_zeta = 1.0;
    pc.ref_a_max = 21.0;
    pc.control_dt = 0.002;
    rtc::catching::CatchPoseIkOptions ik;
    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
    rtc::catching::DecelPlannerConstants dc;
    dc.eta_v = 0.9;
    dc.t_arm_s = pc.t_arm_s;
    dc.control_dt = 0.002;
    dc.v_eps = 1e-6;
    rtc::catching::PlannerCycleIo io{&boxes.traj, &boxes.cov, &boxes.rt, &boxes.plan,
                                     bind_decel ? &boxes.decel : nullptr};
    EXPECT_TRUE(cycle.Bind(io));
    EXPECT_TRUE(cycle.ConfigureSearch(pm, pc, ik));
    std::string err;
    EXPECT_TRUE(cycle.ConfigureDecel(dm, dc, &err)) << err;
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
         DecelOutcomeName(r.decel.outcome) + " / " +
         rtc::catching::DecelMpcReasonName(r.decel.core_reason);
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
  const DecelPlanSnapshot d = p->boxes->decel.Load();
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
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kPublished);
  EXPECT_EQ(rec.decel.kind, DecelKind::kFirst);
  EXPECT_TRUE(rec.decel.cold_start);
  // Between the two stores the segment is there and the plan is not.
  EXPECT_EQ(probe.calls, 1);
  EXPECT_TRUE(probe.segment_already_there);
  EXPECT_TRUE(probe.plan_not_yet_there);
  const PlanSnapshot plan = r->boxes.plan.Load();
  const DecelPlanSnapshot seg = r->boxes.decel.Load();
  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(seg.valid);
  EXPECT_EQ(plan.plan_id, rec.plan_id);
  EXPECT_EQ(seg.plan_id, plan.plan_id);
  EXPECT_EQ(seg.t_c_ns, plan.t_c_ns);
  EXPECT_EQ(seg.publish_ns, plan.publish_ns);
  EXPECT_EQ(seg.publish_ns, rec.publish_ns);
  EXPECT_GT(seg.n_pre, 0);
  EXPECT_EQ(seg.t0_ns, plan.t_c_ns - seg.n_pre * seg.dt_pre_ns);
  EXPECT_TRUE(rtc::catching::ValidateDecelNodes(seg));
  // Recorded for the sim comparison: the search plus the first solve.
  std::printf("[ record ] pair wake: search %.2f ms, first solve %.2f ms, n_pre %d\n",
              static_cast<double>(rec.search.search_ns) * 1e-6,
              static_cast<double>(rec.decel.solve_ns) * 1e-6, seg.n_pre);
}

TEST(ApproachCycle, AWithheldSegmentWithholdsThePlan) {
  auto r = std::make_unique<Rig>(/*catch_err_max=*/1e-9);
  r->rt.adopt = false;
  r->StartTrajectory();
  const PlannerCycleRecord rec = r->Wake();
  EXPECT_EQ(rec.outcome, CycleOutcome::kHeld) << Why(rec);
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kCatchError);
  // The search found a plan; the pair was not published. The two flags are
  // what tells this wake from one whose search found nothing.
  EXPECT_TRUE(rec.search_valid);
  EXPECT_FALSE(rec.plan_valid);
  EXPECT_FALSE(r->boxes.plan.Load().valid);
  EXPECT_FALSE(r->boxes.decel.Load().valid);
  EXPECT_EQ(r->cycle.LastPlanId(), 0U);
  EXPECT_EQ(r->cycle.LastDecelSeq(), 0U);
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
  r->cycle.SetPostDecelHookForTesting(&StoreNewerTrajectory, &swap);
  PlannerCycleRecord rec = r->Wake();
  EXPECT_EQ(rec.outcome, CycleOutcome::kPublished) << Why(rec);
  EXPECT_TRUE(r->boxes.decel.Load().valid);

  auto q = std::make_unique<Rig>();
  q->rt.adopt = false;
  q->StartTrajectory();
  TrajSwap other{q.get(), 2, kTrack + 1};
  q->cycle.SetPostDecelHookForTesting(&StoreNewerTrajectory, &other);
  rec = q->Wake();
  EXPECT_EQ(rec.outcome, CycleOutcome::kSuperseded) << Why(rec);
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kSuperseded);
  EXPECT_FALSE(q->boxes.plan.Load().valid);
  EXPECT_FALSE(q->boxes.decel.Load().valid);
  r->cycle.SetPostDecelHookForTesting(nullptr, nullptr);
  q->cycle.SetPostDecelHookForTesting(nullptr, nullptr);
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
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kOff);
  EXPECT_EQ(r->cycle.LastPlanId(), first.plan_id);
  // One that postdates it (the RT did not take the plan): the search resumes.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  rec = r->Wake();
  EXPECT_NE(rec.outcome, CycleOutcome::kIdle) << Why(rec);
  EXPECT_GT(rec.search.n_ik, 0);
}

// ── Replans ──────────────────────────────────────────────────────────────────

TEST(ApproachCycle, OnceFollowingTheSearchStopsAndEverySegmentStartsOnTheReport) {
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
  std::int32_t last_k = first.decel.k;
  std::uint64_t seq = 1;
  // Wake every ~15 ms through APPROACH, COMMITTED and DECEL to the end of the
  // replan window, with a new trajectory snapshot every other wake.
  while (Now() < t_c + 4 * 50 * kMs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    if (seq % 2 == 0) {
      r->StoreTrajectory(seq + 1);
    }
    ++seq;
    const PlannerCycleRecord rec = r->Wake();
    searched += rec.search.n_ik > 0 ? 1 : 0;
    EXPECT_NE(rec.outcome, CycleOutcome::kPublished) << "a second plan while following";
    if (rec.decel.outcome != DecelOutcome::kPublished) {
      continue;
    }
    ++replans;
    // Every published replan started on a segment the RT reported (path (i)).
    EXPECT_TRUE(rec.decel.from_segment);
    EXPECT_NE(rec.decel.source_seq, 0U);
    // A new grid point or core is a new problem; the same point is not.
    if (rec.decel.kind == DecelKind::kSame) {
      ++same;
      EXPECT_FALSE(rec.decel.cold_start);
    } else {
      EXPECT_TRUE(rec.decel.cold_start) << rtc::catching::DecelKindName(rec.decel.kind);
      advance += rec.decel.kind == DecelKind::kAdvance ? 1 : 0;
      stop += rec.decel.kind == DecelKind::kStop ? 1 : 0;
    }
    EXPECT_GE(rec.decel.k, last_k) << "the grid point never moves back";
    last_k = rec.decel.k;
  }
  EXPECT_EQ(searched, 0) << "the search ran while the RT followed a plan";
  EXPECT_GT(replans, 0);
  EXPECT_GT(same, 0);
  EXPECT_GT(advance, 0);
  EXPECT_GT(stop, 0);
  std::printf("[ record ] replans %d: same %d, advance %d, stop %d\n", replans, same, advance,
              stop);
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
  ASSERT_EQ(rec.decel.outcome, DecelOutcome::kPublished) << Why(rec);
  EXPECT_EQ(r->boxes.decel.Load().token.generation, kTrack);
  const std::uint32_t seq = r->cycle.LastDecelSeq();
  r->rt.track = kTrack + 1;
  r->StoreTrajectory(50, kTrack + 1);
  for (int i = 0; i < 4; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    rec = r->Wake();
    ASSERT_LT(rec.decel.k, 0) << "the wakes are meant to fall before the catch";
    EXPECT_EQ(rec.decel.outcome, DecelOutcome::kNoBall) << Why(rec);
  }
  EXPECT_EQ(r->cycle.LastDecelSeq(), seq);
  // The plan's own track again: the replans resume.
  r->StoreTrajectory(51, kTrack);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  rec = r->Wake();
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kPublished) << Why(rec);
}

TEST(ApproachCycle, WithoutAReportNothingIsReplanned) {
  // An RT that took the plan but reports no segment — neither pending nor
  // followed (it dropped the one it had): the planner finds no source, and
  // does not infer one (MD-58).
  auto r = std::make_unique<Rig>();
  r->rt.report_decel = false;
  r->StartTrajectory();
  ASSERT_EQ(r->Wake().outcome, CycleOutcome::kPublished);
  const std::uint32_t seq = r->cycle.LastDecelSeq();
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const PlannerCycleRecord rec = r->Wake();
    EXPECT_EQ(rec.decel.outcome, DecelOutcome::kNotFollowed) << Why(rec);
  }
  EXPECT_EQ(r->cycle.LastDecelSeq(), seq);
}

// ── Lifetime ─────────────────────────────────────────────────────────────────

TEST(ApproachCycle, ATrialResetWithdrawsTheSegment) {
  // The planner is the decel box's only writer: the ended trial's segment
  // must not stay in it (the RT's plan match refuses it as well).
  auto r = std::make_unique<Rig>();
  r->StartTrajectory();
  ASSERT_EQ(r->Wake().outcome, CycleOutcome::kPublished);
  ASSERT_TRUE(r->boxes.decel.Load().valid);
  PlannerRtState s = r->boxes.rt.Load();
  s.reset_epoch = 2;
  s.mode = static_cast<std::uint8_t>(Mode::kIdle);
  s.plan_active = false;
  s.rt_state_ns = Now();
  r->boxes.rt.Store(s);
  const PlannerCycleRecord rec = r->cycle.Run(rtc::catching::NowReal{Now()});
  EXPECT_TRUE(rec.reset_seen);
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kOff);
  EXPECT_FALSE(r->boxes.decel.Load().valid) << "the ended trial's segment stays in the box";
}

TEST(ApproachCycle, WithoutADecelBoxThePlanIsPublishedAlone) {
  // No fifth box (a binding without the decel lane): the decel planner is
  // configured but solves nothing, and the search's plan goes out by itself.
  auto r = std::make_unique<Rig>(/*catch_err_max=*/0.02, /*bind_decel=*/false);
  ASSERT_TRUE(r->cycle.DecelConfigured());
  r->rt.adopt = false;
  r->StartTrajectory();
  const PlannerCycleRecord rec = r->Wake();
  ASSERT_EQ(rec.outcome, CycleOutcome::kPublished) << Why(rec);
  EXPECT_TRUE(rec.plan_valid);
  EXPECT_EQ(rec.decel.outcome, DecelOutcome::kOff);
  EXPECT_TRUE(r->boxes.plan.Load().valid);
  EXPECT_EQ(r->boxes.decel.sequence(), 0U);
  EXPECT_EQ(r->cycle.LastDecelSeq(), 0U);
  r->cycle.ClearDecel();
  EXPECT_FALSE(r->cycle.DecelConfigured());
}

}  // namespace
