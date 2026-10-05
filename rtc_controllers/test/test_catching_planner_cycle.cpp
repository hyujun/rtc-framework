// ── Planner data contract + one planner wake (dynamic_catching S6-A) ────────
//
// What this suite pins, and why each block is separate:
//
//   1. The RT admission rule (L3 §5.2 (a)-(f)). Each refusal is produced by a
//      plan that passes every EARLIER check, so a case cannot pass because an
//      earlier check happened to refuse it for another reason.
//   2. The activity predicate. Enumerated over every mode, because the point
//      of a predicate over an ordering test is that inserting a mode cannot
//      silently change what the planner does in the others.
//   3. `planner.*` parsing — defaults when absent, refusal (never a default)
//      when present and malformed.
//   4. The cycle's provenance handling (G3-L, planner side): what it publishes,
//      what it refuses to publish, and that ids are monotone per publish.
//   5. The RT contract of one wake (G3-K): no heap, no Eigen allocation.
//   6. The two interfaces (E1-F12 #738): with a fake CatchSearch and a fake
//      SegmentPlanner installed, what a wake calls, in what order and with
//      what — so the wake is shown to go through the interfaces alone.
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using rtc::catching::ActivityFor;
using rtc::catching::AdmittedPlan;
using rtc::catching::BallPrediction;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::CycleOutcome;
using rtc::catching::JudgePlan;
using rtc::catching::Mode;
using rtc::catching::NowReal;
using rtc::catching::ParsePlannerParams;
using rtc::catching::PlanAdmissionContext;
using rtc::catching::PlannerActivity;
using rtc::catching::PlannerCycle;
using rtc::catching::PlannerCycleIo;
using rtc::catching::PlannerCycleRecord;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanRefusal;
using rtc::catching::PlanSnapshot;
using rtc::catching::SearchStats;
using rtc::catching::SegmentKind;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;
using rtc::catching::TrajectorySnapshot;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::uint64_t kActivation = 5;
constexpr std::uint64_t kTrack = 42;

// ── 1. RT admission ─────────────────────────────────────────────────────────

PlanSnapshot AdmissiblePlan() {
  PlanSnapshot p{};
  p.valid = true;
  p.token.activation_generation = kActivation;
  p.token.generation = kTrack;
  p.plan_id = 9;
  p.publish_ns = 1000 * kMs;
  return p;
}

PlanAdmissionContext Context() {
  PlanAdmissionContext c{};
  c.activation_generation = kActivation;
  c.track_seen = true;
  c.track_generation = kTrack;
  c.now = NowReal{1050 * kMs};
  c.max_age_ns = 100 * kMs;
  c.reset_floor_ns = 900 * kMs;
  return c;
}

TEST(PlanAdmission, APlanPassingEveryCheckIsAdmitted) {
  EXPECT_EQ(JudgePlan(AdmissiblePlan(), Context(), AdmittedPlan{}), PlanRefusal::kNone);
}

TEST(PlanAdmission, EachCheckRefusesOnItsOwn) {
  // (a) the writer says there is no plan.
  {
    auto p = AdmissiblePlan();
    p.valid = false;
    EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kInvalid);
  }
  // (b) D-23: a plan from the previous activation — the deactivate/Pause race
  // publishes exactly this.
  {
    auto p = AdmissiblePlan();
    p.token.activation_generation = kActivation - 1;
    EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kActivation);
  }
  // (c) computed from another ball.
  {
    auto p = AdmissiblePlan();
    p.token.generation = kTrack + 1;
    EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kTrack);
  }
  // (c') and the RT has not consumed any trajectory yet this trial — there is
  // no track to match, so nothing matches it.
  {
    auto c = Context();
    c.track_seen = false;
    EXPECT_EQ(JudgePlan(AdmissiblePlan(), c, {}), PlanRefusal::kTrack);
  }
  // (d) the plan the RT already took.
  EXPECT_EQ(JudgePlan(AdmissiblePlan(), Context(), AdmittedPlan{true, 9}), PlanRefusal::kRepeat);
  // (e) too old, never published, and from the future.
  {
    auto p = AdmissiblePlan();
    p.publish_ns = Context().now.ns - Context().max_age_ns - 1;
    EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kAged);
    p.publish_ns = 0;
    EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kAged);
    p.publish_ns = Context().now.ns + 1;
    EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kAged);
  }
  // (f) published before the RT's last reset — the E-STOP case, where the
  // activation generation did not move.
  {
    auto c = Context();
    c.reset_floor_ns = AdmissiblePlan().publish_ns + 1;
    EXPECT_EQ(JudgePlan(AdmissiblePlan(), c, {}), PlanRefusal::kBeforeReset);
  }
}

TEST(PlanAdmission, APlanWhoseCatchIsInsideTheFreezeWindowIsTooLate) {
  // (g), S7: the boundary is exclusive — t_c − now == T_freeze would commit on
  // the tick the plan is taken.
  auto c = Context();
  c.t_freeze_ns = 360 * kMs;
  auto p = AdmissiblePlan();
  p.t_c_ns = c.now.ns + c.t_freeze_ns;
  EXPECT_EQ(JudgePlan(p, c, {}), PlanRefusal::kTooLate);
  p.t_c_ns = c.now.ns - 1;  // already past
  EXPECT_EQ(JudgePlan(p, c, {}), PlanRefusal::kTooLate);
  p.t_c_ns = c.now.ns + c.t_freeze_ns + 1;
  EXPECT_EQ(JudgePlan(p, c, {}), PlanRefusal::kNone);
  // A profile with no freeze window does not judge it at all.
  c.t_freeze_ns = 0;
  p.t_c_ns = 0;
  EXPECT_EQ(JudgePlan(p, c, {}), PlanRefusal::kNone);
}

TEST(PlanAdmission, TheAgeBoundIsInclusiveAndADifferentIdIsNew) {
  auto p = AdmissiblePlan();
  p.publish_ns = Context().now.ns - Context().max_age_ns;
  EXPECT_EQ(JudgePlan(p, Context(), {}), PlanRefusal::kNone);
  // Same token, new id: a re-plan from the same trajectory IS a new plan.
  EXPECT_EQ(JudgePlan(AdmissiblePlan(), Context(), AdmittedPlan{true, 8}), PlanRefusal::kNone);
}

// ── 2. Activity predicate ───────────────────────────────────────────────────

TEST(PlannerActivityPredicate, SearchesOnlyInTrackingAndApproach) {
  const std::array<std::pair<Mode, PlannerActivity>, rtc::catching::kNumModes> expected{{
      {Mode::kIdle, PlannerActivity::kIdle},
      {Mode::kArmed, PlannerActivity::kIdle},
      {Mode::kTracking, PlannerActivity::kSearch},
      {Mode::kApproach, PlannerActivity::kSearch},
      {Mode::kCommitted, PlannerActivity::kMonitor},
      {Mode::kClosing, PlannerActivity::kMonitor},
      // MPC E1-F03 (MD-29): DECEL hosts the decel MPC's post-catch replans.
      {Mode::kDecel, PlannerActivity::kDecel},
      {Mode::kHold, PlannerActivity::kIdle},
      {Mode::kRetreat, PlannerActivity::kIdle},
      {Mode::kAbortSafe, PlannerActivity::kIdle},
      {Mode::kFault, PlannerActivity::kIdle},
  }};
  for (const auto& [mode, activity] : expected) {
    EXPECT_EQ(ActivityFor(mode), activity) << "mode " << static_cast<int>(mode);
  }
}

// ── 3. planner.* parsing ────────────────────────────────────────────────────

TEST(PlannerParams, AnAbsentSectionYieldsTheDocumentedDefaults) {
  const auto p = ParsePlannerParams(YAML::Load("io: {t_stale: 0.1}"));
  EXPECT_FALSE(p.enabled);
  EXPECT_DOUBLE_EQ(p.wake_timeout_s, 0.05);  // decision H
  EXPECT_DOUBLE_EQ(p.budget_s, 0.020);       // R-2
  EXPECT_EQ(p.wait_pose_n, 0);
}

TEST(PlannerParams, ReadsEveryKey) {
  const auto p = ParsePlannerParams(YAML::Load(R"(
planner:
  enabled: true
  wake_timeout_s: 0.04
  budget_s: 0.015
  wait_pose: [0.1, -0.2, 0.3]
  ik: {max_iter: 40}
)"));
  EXPECT_TRUE(p.enabled);
  EXPECT_DOUBLE_EQ(p.wake_timeout_s, 0.04);
  EXPECT_DOUBLE_EQ(p.budget_s, 0.015);
  ASSERT_EQ(p.wait_pose_n, 3);
  EXPECT_DOUBLE_EQ(p.wait_pose[0], 0.1);
  EXPECT_DOUBLE_EQ(p.wait_pose[1], -0.2);
  EXPECT_DOUBLE_EQ(p.wait_pose[2], 0.3);
}

TEST(PlannerParams, AMalformedKeyIsRefusedRatherThanDefaulted) {
  for (const char* bad : {
           "planner: 3",
           "planner: {enabled: maybe}",
           "planner: {wake_timeout_s: TBD}",
           "planner: {wake_timeout_s: 0.001}",
           "planner: {wake_timeout_s: 0.6}",
           "planner: {budget_s: 0.0005}",
           "planner: {budget_s: 0.06}",
           "planner: {budget_s: .nan}",
           "planner: {wait_pose: []}",
           "planner: {wait_pose: 0.3}",
           "planner: {wait_pose: [0.1, x]}",
           "planner: {wait_pose: [0.1, .inf]}",
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
  // The boundaries themselves are inside the range.
  EXPECT_NO_THROW(static_cast<void>(
      ParsePlannerParams(YAML::Load("planner: {wake_timeout_s: 0.005, budget_s: 0.05}"))));
}

TEST(PlannerParams, AWaitPoseLongerThanThePlanCapacityIsRefused) {
  std::string pose = "planner: {wait_pose: [";
  for (std::size_t i = 0; i <= rtc::catching::kMaxPlanNv; ++i) {
    pose += (i == 0 ? "0.0" : ", 0.0");
  }
  pose += "]}";
  EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(pose))), std::invalid_argument);
}

TEST(PlannerParams, ReadsTheSearchKeys) {
  const auto p = ParsePlannerParams(YAML::Load(R"(
planner:
  sub_model: "arm_catch"
  max_ik: 5
  n_settle: 2
  slice: {dt: 0.05, t_lead_min: 0.3, t_max: 0.9}
  time: {margin: 0.02}
  unc: {kappa_sigma: 0.4}
  gamma: {margin: 0.2, eta_v: 0.9}
  budget: {n_sigma: 2.5, sigma_trk: 0.001, clock_err: 0.002}
  hand: {d_eff: 0.28, r_cap: 0.024, provisional: true}
  switch: {delta_J: 0.2, eta_jump: 0.4}
  freeze: {T_freeze: 0.36}
  score: {w_sigma: 2, w_t: 3, w_q: 0.5, w_late: 0.1, w_gamma: 4, penalty: 20}
  workspace: {catch_box: {min: [0.1, -0.3, 0.2], max: [1.0, 0.3, 0.9]}}
)"));
  EXPECT_EQ(p.sub_model, "arm_catch");
  EXPECT_EQ(p.max_ik, 5);
  EXPECT_EQ(p.n_settle, 2);
  EXPECT_DOUBLE_EQ(p.slice_t_lead_min, 0.3);
  EXPECT_DOUBLE_EQ(p.LeadMin(), 0.3);
  EXPECT_DOUBLE_EQ(p.slice_t_max, 0.9);
  EXPECT_DOUBLE_EQ(p.time_margin, 0.02);
  EXPECT_DOUBLE_EQ(p.kappa_sigma, 0.4);
  EXPECT_DOUBLE_EQ(p.gamma_margin, 0.2);
  EXPECT_DOUBLE_EQ(p.n_sigma, 2.5);
  EXPECT_DOUBLE_EQ(p.sigma_trk, 0.001);
  EXPECT_DOUBLE_EQ(p.clock_err, 0.002);
  EXPECT_DOUBLE_EQ(p.d_eff, 0.28);
  EXPECT_DOUBLE_EQ(p.r_cap, 0.024);
  EXPECT_DOUBLE_EQ(p.switch_delta_j, 0.2);
  EXPECT_DOUBLE_EQ(p.switch_eta_jump, 0.4);
  EXPECT_DOUBLE_EQ(p.t_freeze, 0.36);
  EXPECT_DOUBLE_EQ(p.score.w_sigma, 2.0);
  EXPECT_DOUBLE_EQ(p.score.penalty, 20.0);
  ASSERT_TRUE(p.catch_box.set);
  EXPECT_TRUE(p.catch_box.Contains(0.5, 0.0, 0.5));
  EXPECT_FALSE(p.catch_box.Contains(0.5, 0.31, 0.5));
  EXPECT_EQ(p.wait_pose_source, rtc::catching::PlannerParams::WaitPoseSource::kYaml)
      << "absent = the YAML pose";
}

TEST(PlannerParams, TheWaitPoseSourceIsYamlOrCurrentAndNothingElse) {
  // S8-I: `current` adopts the arm's switched-in pose; anything but the two
  // spellings is refused rather than silently read as the default.
  EXPECT_EQ(ParsePlannerParams(YAML::Load("planner: {wait_pose_source: yaml}")).wait_pose_source,
            rtc::catching::PlannerParams::WaitPoseSource::kYaml);
  EXPECT_EQ(ParsePlannerParams(YAML::Load("planner: {wait_pose_source: current}")).wait_pose_source,
            rtc::catching::PlannerParams::WaitPoseSource::kCurrent);
  for (const char* bad :
       {"planner: {wait_pose_source: Current}", "planner: {wait_pose_source: 1}",
        "planner: {wait_pose_source: ''}", "planner: {wait_pose_source: [yaml]}"}) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
}

TEST(PlannerParams, ADecisionLeftOutOrTbdIsUnsetNotDefaulted) {
  // T_freeze, catch_box, d_eff/r_cap are decisions (L3 §6 TBD): absent or TBD
  // must read as UNSET so the binding parks, never as a plausible number.
  for (const char* yaml : {"planner: {enabled: true}",
                           "planner: {freeze: {T_freeze: TBD}, workspace: {catch_box: TBD}, "
                           "hand: {d_eff: TBD, r_cap: TBD}}"}) {
    const auto p = ParsePlannerParams(YAML::Load(yaml));
    EXPECT_TRUE(std::isnan(p.t_freeze)) << yaml;
    EXPECT_TRUE(std::isnan(p.LeadMin())) << yaml << ": t_lead_min falls back to T_freeze";
    EXPECT_FALSE(p.catch_box.set) << yaml;
    EXPECT_TRUE(std::isnan(p.d_eff)) << yaml;
    EXPECT_TRUE(std::isnan(p.r_cap)) << yaml;
    EXPECT_TRUE(p.sub_model.empty()) << yaml;
  }
}

TEST(PlannerParams, AMalformedSearchKeyIsRefused) {
  for (const char* bad : {
           "planner: {sub_model: TBD}",
           "planner: {sub_model: ''}",
           "planner: {max_ik: 0}",
           "planner: {max_ik: 41}",
           "planner: {slice: {t_max: 2.0}}",
           "planner: {slice: 3}",
           "planner: {freeze: {T_freeze: -0.1}}",
           "planner: {workspace: {catch_box: {min: [0, 0, 0]}}}",
           "planner: {workspace: {catch_box: {min: [1, 0, 0], max: [0, 1, 1]}}}",
           "planner: {workspace: {catch_box: {min: [0, 0], max: [1, 1, 1]}}}",
           "planner: {score: {penalty: -1}}",
           "planner: {switch: {eta_jump: 0}}",
           "planner: {switch: {eta_jump: 1.5}}",
           // Retired by the acceleration budget (decision ⑥): refused, never
           // silently run on the eta_jump default.
           "planner: {switch: {e_jump_max: 0.01}}",
           "planner: {switch: {ed_jump_max: 0.05}}",
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
}

// `SwitchStep` evaluates a ramp per sample on the planner thread for every switch
// check: the count is bounded above (kSwitchSamplesMax), refused by key name.
TEST(PlannerParams, SwitchSamplesIsBoundedAboveByName) {
  const auto message = [](int samples) -> std::string {
    try {
      static_cast<void>(ParsePlannerParams(
          YAML::Load("planner: {switch: {samples: " + std::to_string(samples) + "}}")));
    } catch (const std::invalid_argument& e) {
      return e.what();
    }
    return {};
  };
  EXPECT_EQ(ParsePlannerParams(YAML::Load("planner: {switch: {samples: " +
                                          std::to_string(rtc::catching::kSwitchSamplesMax) + "}}"))
                .switch_samples,
            rtc::catching::kSwitchSamplesMax);
  const std::string why = message(rtc::catching::kSwitchSamplesMax + 1);
  ASSERT_FALSE(why.empty()) << "samples = kSwitchSamplesMax + 1 was accepted";
  EXPECT_NE(why.find("'planner.switch.samples'"), std::string::npos) << why;
  EXPECT_FALSE(message(1000000).empty());
}

// ── 4. The cycle ────────────────────────────────────────────────────────────

std::int64_t FixedClock() noexcept {
  return 2000 * kMs;
}

struct Boxes {
  rtc::SeqLock<TrajectorySnapshot> traj{};
  rtc::SeqLock<CovarianceSnapshot> cov{};
  rtc::SeqLock<PlannerRtState> rt{};
  rtc::SeqLock<PlanSnapshot> plan{};

  PlannerCycleIo Io() { return {&traj, &cov, &rt, &plan}; }
};

PlannerRtState RtIn(Mode mode, std::uint32_t reset_epoch = 1) {
  PlannerRtState s{};
  s.valid = true;
  s.activation_generation = kActivation;
  s.rt_iteration = 77;
  s.rt_state_ns = 1990 * kMs;
  s.reset_epoch = reset_epoch;
  s.mode = static_cast<std::uint8_t>(mode);
  return s;
}

TrajectorySnapshot Traj(std::uint64_t sequence, std::uint64_t activation = kActivation) {
  TrajectorySnapshot t{};
  t.valid = true;
  t.n = 12;
  t.token.activation_generation = activation;
  t.token.generation = kTrack;
  t.token.snapshot_sequence = sequence;
  t.token.traj_recv_ns = 1980 * kMs;
  return t;
}

CovarianceSnapshot Cov(std::uint64_t sequence) {
  CovarianceSnapshot c{};
  c.valid = true;
  c.n = 12;
  c.token = Traj(sequence).token;
  return c;
}

// Heap-allocated: three snapshot-sized members would blow a test's stack frame
// for no reason.
struct CycleRig {
  Boxes boxes;
  PlannerCycle cycle;

  CycleRig() {
    EXPECT_TRUE(cycle.Bind(boxes.Io()));
    cycle.SetClock(&FixedClock);
  }
};

TEST(PlannerCycleRun, AnUnboundOrStatelessCycleDoesNothing) {
  auto rig = std::make_unique<CycleRig>();
  PlannerCycle unbound;
  EXPECT_FALSE(unbound.Bind(PlannerCycleIo{}));
  EXPECT_EQ(unbound.Run(NowReal{1}).outcome, CycleOutcome::kIdle);
  // Bound, but the RT has not stored its first state.
  EXPECT_EQ(rig->cycle.Run(NowReal{1}).outcome, CycleOutcome::kIdle);
  EXPECT_FALSE(rig->boxes.plan.Load().valid);
  EXPECT_EQ(rig->cycle.LastPlanId(), 0U);
}

TEST(PlannerCycleRun, PublishesNothingInAModeWithNothingToPlanFor) {
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.traj.Store(Traj(3));
  for (const Mode m : {Mode::kIdle, Mode::kArmed, Mode::kCommitted, Mode::kRetreat,
                       Mode::kAbortSafe, Mode::kFault}) {
    rig->boxes.rt.Store(RtIn(m));
    EXPECT_EQ(rig->cycle.Run(NowReal{1}).outcome, CycleOutcome::kIdle)
        << "mode " << static_cast<int>(m);
  }
  EXPECT_EQ(rig->cycle.LastPlanId(), 0U) << "a non-search mode published";
  EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
}

TEST(PlannerCycleRun, AMonitorWakeWithoutASearchRecordsNothingToPublish) {
  // COMMITTED with no search configured: there is no followed plan of ours to
  // read σ_ℓ at, and the record says so the way a configured search does —
  // nothing to publish, σ_ℓ unknown.
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kCommitted));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  const auto rec = rig->cycle.Run(NowReal{1});
  EXPECT_EQ(rec.outcome, CycleOutcome::kIdle);
  EXPECT_FALSE(rec.search.publish);
  EXPECT_TRUE(std::isnan(rec.search.sigma_l));
  EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
}

TEST(PlannerCycleRun, NoTrajectoryOfThisActivationIsNoInput) {
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  EXPECT_EQ(rig->cycle.Run(NowReal{1}).outcome, CycleOutcome::kNoInput);
  // D-23: a trajectory the ingress stamped with the PREVIOUS activation.
  rig->boxes.traj.Store(Traj(3, kActivation - 1));
  EXPECT_EQ(rig->cycle.Run(NowReal{1}).outcome, CycleOutcome::kNoInput);
  EXPECT_EQ(rig->cycle.LastPlanId(), 0U);
}

TEST(PlannerCycleRun, ASearchWakePublishesTheStubsNoPlanWithFullProvenance) {
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));

  const auto rec = rig->cycle.Run(NowReal{1995 * kMs});
  ASSERT_EQ(rec.outcome, CycleOutcome::kPublished);
  EXPECT_TRUE(rec.cov_matched);
  EXPECT_EQ(rec.plan_id, 1U);
  EXPECT_FALSE(rec.plan_valid) << "S6-A's search is a stub";
  EXPECT_FALSE(rec.search_valid) << "a published \"no plan\" is not a valid search";
  EXPECT_EQ(rec.snapshot_sequence, 3U);
  EXPECT_EQ(rec.wake_ns, 1995 * kMs);
  EXPECT_EQ(rec.publish_ns, FixedClock());

  const PlanSnapshot p = rig->boxes.plan.Load();
  EXPECT_FALSE(p.valid);
  EXPECT_EQ(p.plan_id, 1U);
  EXPECT_EQ(p.publish_ns, FixedClock());
  EXPECT_EQ(p.token.activation_generation, kActivation);
  EXPECT_EQ(p.token.generation, kTrack);
  EXPECT_EQ(p.token.snapshot_sequence, 3U);
  EXPECT_EQ(p.rt_iteration, 77U);
  EXPECT_EQ(p.rt_state_ns, 1990 * kMs);

  // A second wake on the SAME trajectory is a new plan: the id moves.
  const auto again = rig->cycle.Run(NowReal{1996 * kMs});
  ASSERT_EQ(again.outcome, CycleOutcome::kPublished);
  EXPECT_EQ(again.plan_id, 2U);
  EXPECT_EQ(rig->boxes.plan.Load().plan_id, 2U);
}

TEST(PlannerCycleRun, ACovarianceOfAnotherSnapshotIsNotPaired) {
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kApproach));
  rig->boxes.traj.Store(Traj(4));
  rig->boxes.cov.Store(Cov(3));  // N-1: the mix L3 §5.2 names
  const auto rec = rig->cycle.Run(NowReal{1});
  EXPECT_EQ(rec.outcome, CycleOutcome::kPublished);
  EXPECT_FALSE(rec.cov_matched);
}

struct SupersedeContext {
  Boxes* boxes{nullptr};
  TrajectorySnapshot next{};
  PlannerRtState rt_next{};
  bool bump_traj{false};
  bool bump_reset{false};
  bool bump_activation{false};
};

void Supersede(void* raw) noexcept {
  auto* ctx = static_cast<SupersedeContext*>(raw);
  if (ctx->bump_traj) {
    ctx->boxes->traj.Store(ctx->next);
  }
  if (ctx->bump_reset || ctx->bump_activation) {
    ctx->boxes->rt.Store(ctx->rt_next);
  }
}

TEST(PlannerCycleRun, AnythingThatMovesDuringTheSearchDropsThePublish) {
  // G3-L, planner side: a newer snapshot during compute, a reset during
  // compute, and an activation boundary during compute. Each must drop the
  // publish — and must not consume a plan id, so the ids the RT sees stay a
  // record of what was PUBLISHED.
  for (int which = 0; which < 3; ++which) {
    auto rig = std::make_unique<CycleRig>();
    rig->boxes.rt.Store(RtIn(Mode::kTracking));
    rig->boxes.traj.Store(Traj(3));
    auto ctx = std::make_unique<SupersedeContext>();
    ctx->boxes = &rig->boxes;
    ctx->next = Traj(4);
    ctx->rt_next = RtIn(Mode::kTracking, /*reset_epoch=*/2);
    ctx->bump_traj = which == 0;
    ctx->bump_reset = which == 1;
    if (which == 2) {
      ctx->rt_next = RtIn(Mode::kTracking);
      ctx->rt_next.activation_generation = kActivation + 1;
      ctx->bump_activation = true;
    }
    rig->cycle.SetPostSearchHookForTesting(&Supersede, ctx.get());
    const auto rec = rig->cycle.Run(NowReal{1});
    EXPECT_EQ(rec.outcome, CycleOutcome::kSuperseded) << "case " << which;
    EXPECT_EQ(rig->cycle.LastPlanId(), 0U) << "case " << which;
    EXPECT_EQ(rig->boxes.plan.sequence(), 0U) << "case " << which << ": something was stored";
  }
}

TEST(PlannerCycleRun, AResetIsSeenOnceEvenInAModeWithNothingToPlan) {
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kIdle, /*reset_epoch=*/1));
  EXPECT_TRUE(rig->cycle.Run(NowReal{1}).reset_seen);
  EXPECT_FALSE(rig->cycle.Run(NowReal{2}).reset_seen) << "the same reset reported twice";
  rig->boxes.rt.Store(RtIn(Mode::kIdle, /*reset_epoch=*/2));
  EXPECT_TRUE(rig->cycle.Run(NowReal{3}).reset_seen);
}

TEST(PlannerCycleRun, EveryPublishedPlanIsRefusedByTheRtWhileTheSearchIsAStub) {
  // The end-to-end half of "wiring the thread changes no behaviour": whatever
  // S6-A publishes, the RT's admission rule refuses as `kInvalid`.
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  rig->boxes.traj.Store(Traj(3));
  ASSERT_EQ(rig->cycle.Run(NowReal{1}).outcome, CycleOutcome::kPublished);
  PlanAdmissionContext c{};
  c.activation_generation = kActivation;
  c.track_seen = true;
  c.track_generation = kTrack;
  c.now = NowReal{FixedClock()};
  c.max_age_ns = 100 * kMs;
  EXPECT_EQ(JudgePlan(rig->boxes.plan.Load(), c, {}), PlanRefusal::kInvalid);
}

// ── 5. RT contract (G3-K, stub part) ────────────────────────────────────────

TEST(PlannerCycleRun, OneWakeAllocatesNothing) {
  auto rig = std::make_unique<CycleRig>();
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  // Warm once outside the gate, the way the thread's first wake would be.
  ASSERT_EQ(rig->cycle.Run(NowReal{1}).outcome, CycleOutcome::kPublished);

  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  CycleOutcome outcome = CycleOutcome::kIdle;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 100; ++i) {
      outcome = rig->cycle.Run(NowReal{2 + i}).outcome;
    }
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_EQ(outcome, CycleOutcome::kPublished)
      << "the gated loop did not exercise the publish path";
  EXPECT_EQ(heap, 0U);
  EXPECT_EQ(eigen, 0U);
}

// ── 6. The two interfaces (E1-F12 #738) ─────────────────────────────────────
//
// The fakes answer what the real implementations cannot — a catch point no arm
// reaches, a control period of one second, a segment that "starts in time"
// with its node 0 at instant 0 — so a wake that consulted a concrete type
// instead of the installed object would publish something else, or nothing.
// Every call goes into one log, the cycle's clock reads included: the order of
// a wake is asserted, not inferred.

enum class Call : std::uint8_t {
  kClock,  // the cycle's own clock (the fakes read none)
  kSearchPlan,
  kSearchMonitor,
  kSearchNotePublished,
  kSearchResetTrial,
  kSetClock,
  kSegmentResetTrial,
  kPlanFirst,
  kReplan,
  kSegmentNotePublished,
  kFollowedTrack,
  kStartsInTime,
  kControlDtNs,
  kSourceSeq,
};

std::ostream& operator<<(std::ostream& os, Call c) {
  static constexpr std::array<const char*, 14> kNames{"Clock",
                                                      "Search.Plan",
                                                      "Search.Monitor",
                                                      "Search.NotePublished",
                                                      "Search.ResetTrial",
                                                      "Segment.SetClock",
                                                      "Segment.ResetTrial",
                                                      "Segment.PlanFirst",
                                                      "Segment.Replan",
                                                      "Segment.NotePublished",
                                                      "Segment.FollowedTrack",
                                                      "Segment.StartsInTime",
                                                      "Segment.ControlDtNs",
                                                      "Segment.SourceSeq"};
  return os << kNames.at(static_cast<std::size_t>(c));
}

using Calls = std::vector<Call>;

// Fixed storage: the fakes log inside the allocation gate too.
struct CallLog {
  std::array<Call, 32> calls{};
  std::size_t n{0};
  bool overflow{false};

  void Add(Call c) noexcept {
    if (n < calls.size()) {
      calls[n++] = c;
    } else {
      overflow = true;
    }
  }

  void Clear() noexcept {
    n = 0;
    overflow = false;
  }

  [[nodiscard]] Calls Taken() const { return Calls(calls.begin(), calls.begin() + n); }
};

CallLog g_calls;

// A clock that moves on every read, so WHEN the cycle reads it shows in what
// it stamps, and each read is in the log.
constexpr std::int64_t kStepBase = 5000 * kMs;
std::int64_t g_step_now = kStepBase;

std::int64_t StepClock() noexcept {
  g_calls.Add(Call::kClock);
  g_step_now += kMs;
  return g_step_now;
}

constexpr std::int64_t kFakeTc = kStepBase + 10'000 * kMs;
constexpr std::array<double, 3> kFakeCatchPoint{111.0, -222.0, 333.0};
constexpr std::uint16_t kFakeIkCount = 4242;
constexpr double kFakeSigmaL = 0.125;
constexpr std::int64_t kFakeControlDtNs = 1000 * kMs;  // the real one is 2 ms
constexpr std::uint32_t kFakeSourceSeq = 5;

class FakeSearch final : public rtc::catching::CatchSearch {
 public:
  [[nodiscard]] PlanSnapshot Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt, NowReal now,
                                  SearchStats& stats) noexcept override {
    g_calls.Add(Call::kSearchPlan);
    plan_sequence = traj.token.snapshot_sequence;
    plan_cov_sequence = cov.token.snapshot_sequence;
    plan_cov_matched = cov_matched;
    plan_now_ns = now.ns;
    stats = SearchStats{};
    stats.n_ik = kFakeIkCount;
    stats.publish = publish;
    PlanSnapshot p{};
    p.token = traj.token;
    p.token.activation_generation = rt.activation_generation;
    p.rt_iteration = rt.rt_iteration;
    p.rt_state_ns = rt.rt_state_ns;
    p.valid = valid;
    p.t_c_ns = kFakeTc;
    p.p_c = kFakeCatchPoint;
    return p;
  }

  void Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov, bool cov_matched,
               const PlannerRtState& rt, SearchStats& stats) const noexcept override {
    g_calls.Add(Call::kSearchMonitor);
    monitor_sequence = traj.token.snapshot_sequence;
    monitor_cov_sequence = cov.token.snapshot_sequence;
    monitor_cov_matched = cov_matched;
    monitor_plan_id = rt.plan_id;
    stats = SearchStats{};
    stats.publish = false;
    stats.sigma_l = kFakeSigmaL;
  }

  void NotePublished(const PlanSnapshot& plan) noexcept override {
    g_calls.Add(Call::kSearchNotePublished);
    noted_plan_id = plan.plan_id;
    noted_publish_ns = plan.publish_ns;
    noted_valid = plan.valid;
  }

  void ResetTrial() noexcept override { g_calls.Add(Call::kSearchResetTrial); }

  // What it answers.
  bool valid{true};
  bool publish{true};
  // What it was handed.
  std::uint64_t plan_sequence{0};
  std::uint64_t plan_cov_sequence{0};
  bool plan_cov_matched{false};
  std::int64_t plan_now_ns{0};
  mutable std::uint64_t monitor_sequence{0};
  mutable std::uint64_t monitor_cov_sequence{0};
  mutable bool monitor_cov_matched{false};
  mutable std::uint32_t monitor_plan_id{0};
  std::uint32_t noted_plan_id{0};
  std::int64_t noted_publish_ns{0};
  bool noted_valid{false};
};

class FakeSegmentPlanner final : public rtc::catching::SegmentPlanner {
 public:
  void SetClock(ClockFn clock) noexcept override {
    g_calls.Add(Call::kSetClock);
    clock_seen = clock;
  }

  void ResetTrial() noexcept override { g_calls.Add(Call::kSegmentResetTrial); }

  [[nodiscard]] bool PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                               const BallPrediction& ball, SegmentSnapshot& out,
                               SegmentRecord& rec) noexcept override {
    g_calls.Add(Call::kPlanFirst);
    static_cast<void>(rt);
    NoteBall(ball);
    first_plan_id = plan.plan_id;
    first_p_c = plan.p_c;
    Fill(plan.plan_id, plan.t_c_ns, out, rec);
    rec.kind = SegmentKind::kFirst;
    return first_ok;
  }

  [[nodiscard]] bool Replan(const PlannerRtState& rt, const BallPrediction& ball,
                            SegmentSnapshot& out, SegmentRecord& rec) noexcept override {
    g_calls.Add(Call::kReplan);
    NoteBall(ball);
    Fill(rt.plan_id, rt.plan_t_c_ns, out, rec);
    rec.kind = SegmentKind::kAdvance;
    // The one record field the cycle reads back (SegmentPlanner::Replan): the
    // seq this solve started from, which the re-check compares with SourceSeq.
    rec.source_seq = kFakeSourceSeq;
    return replan_ok;
  }

  void NotePublished(const SegmentSnapshot& p) noexcept override {
    g_calls.Add(Call::kSegmentNotePublished);
    noted_seq = p.segment_seq;
    noted_publish_ns = p.publish_ns;
  }

  [[nodiscard]] bool FollowedTrack(const PlannerRtState& rt,
                                   std::uint64_t& generation) const noexcept override {
    g_calls.Add(Call::kFollowedTrack);
    static_cast<void>(rt);
    generation = followed_generation;
    return followed;
  }

  [[nodiscard]] bool StartsInTime(std::int64_t publish_ns,
                                  std::int64_t t0_ns) const noexcept override {
    g_calls.Add(Call::kStartsInTime);
    starts_publish_ns = publish_ns;
    starts_t0_ns = t0_ns;
    return starts_in_time;
  }

  [[nodiscard]] std::int64_t ControlDtNs() const noexcept override {
    g_calls.Add(Call::kControlDtNs);
    return kFakeControlDtNs;
  }

  [[nodiscard]] std::uint32_t SourceSeq(const PlannerRtState& rt,
                                        std::int64_t t_eff_ns) const noexcept override {
    g_calls.Add(Call::kSourceSeq);
    static_cast<void>(rt);
    source_t_eff_ns = t_eff_ns;
    return source_seq;
  }

  // What it answers.
  bool first_ok{true};
  bool replan_ok{true};
  bool followed{true};
  std::uint64_t followed_generation{kTrack};
  bool starts_in_time{true};
  std::uint32_t source_seq{kFakeSourceSeq};
  // What it was handed.
  ClockFn clock_seen{nullptr};
  bool ball_empty{true};
  bool ball_cov_matched{false};
  std::uint64_t ball_sequence{0};
  std::uint64_t ball_cov_sequence{0};
  std::uint32_t first_plan_id{0};
  std::array<double, 3> first_p_c{};
  std::uint32_t noted_seq{0};
  std::int64_t noted_publish_ns{0};
  mutable std::int64_t starts_publish_ns{0};
  mutable std::int64_t starts_t0_ns{-1};
  mutable std::int64_t source_t_eff_ns{-1};

 private:
  void NoteBall(const BallPrediction& ball) noexcept {
    ball_empty = ball.Empty();
    ball_cov_matched = ball.cov_matched;
    ball_sequence = ball.Empty() ? 0 : ball.traj->token.snapshot_sequence;
    ball_cov_sequence = ball.Empty() ? 0 : ball.cov->token.snapshot_sequence;
  }

  // A segment whose node 0 is at instant 0: the real planner's StartsInTime
  // refuses it at any publish stamp.
  static void Fill(std::uint32_t plan_id, std::int64_t t_c_ns, SegmentSnapshot& out,
                   SegmentRecord& rec) noexcept {
    rec = SegmentRecord{};
    rec.outcome = SegmentOutcome::kReady;
    out = SegmentSnapshot{};
    out.valid = true;
    out.plan_id = plan_id;
    out.t_c_ns = t_c_ns;
    out.t0_ns = 0;
    out.token.generation = kTrack;
  }
};

PlannerRtState Following(Mode mode, std::uint32_t plan_id) {
  PlannerRtState s = RtIn(mode);
  s.plan_active = true;
  s.plan_id = plan_id;
  s.plan_t_c_ns = kFakeTc;
  return s;
}

struct FakeRig {
  Boxes boxes;
  rtc::SeqLock<SegmentSnapshot> segment_box{};
  PlannerCycle cycle;
  FakeSearch* search{nullptr};           // owned by the cycle
  FakeSegmentPlanner* segment{nullptr};  // owned by the cycle
  PlannerCycleRecord rec{};

  FakeRig() {
    PlannerCycleIo io = boxes.Io();
    io.segment = &segment_box;
    EXPECT_TRUE(cycle.Bind(io));
    EXPECT_FALSE(cycle.SearchConfigured());
    EXPECT_FALSE(cycle.SegmentPlannerConfigured());
    auto s = std::make_unique<FakeSearch>();
    auto p = std::make_unique<FakeSegmentPlanner>();
    search = s.get();
    segment = p.get();
    cycle.InstallSearch(std::move(s));
    cycle.InstallSegmentPlanner(std::move(p));
    EXPECT_TRUE(cycle.SearchConfigured());
    EXPECT_TRUE(cycle.SegmentPlannerConfigured());
    g_calls.Clear();
    g_step_now = kStepBase;
    cycle.SetClock(&StepClock);
    // The cycle's first sight of the RT is a reset (epoch 0 → 1): taken here,
    // in a mode with nothing to plan, so each test's wakes start clean.
    boxes.rt.Store(RtIn(Mode::kIdle));
    static_cast<void>(cycle.Run(NowReal{1}));
  }

  Calls Wake(std::int64_t wake_ns = 1995 * kMs) {
    g_calls.Clear();
    rec = cycle.Run(NowReal{wake_ns});
    EXPECT_FALSE(g_calls.overflow);
    return g_calls.Taken();
  }

  // A search wake that publishes the pair; returns its stamp.
  std::int64_t PublishPair(std::uint64_t sequence = 3) {
    boxes.rt.Store(RtIn(Mode::kTracking));
    boxes.traj.Store(Traj(sequence));
    boxes.cov.Store(Cov(sequence));
    static_cast<void>(Wake());
    EXPECT_EQ(rec.outcome, CycleOutcome::kPublished);
    EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kPublished);
    return rec.publish_ns;
  }
};

TEST(PlannerCycleInterfaces, InstalledIsConfiguredAndTheClockReachesTheSegmentPlannerOnly) {
  auto rig = std::make_unique<FakeRig>();
  // The rig's SetClock, then its first wake's reset: nothing else was called
  // — a search has no clock to be handed.
  EXPECT_EQ(rig->segment->clock_seen, &StepClock);
  EXPECT_EQ(g_calls.Taken(),
            (Calls{Call::kSetClock, Call::kSearchResetTrial, Call::kSegmentResetTrial}));
  // A planner installed AFTER the cycle's SetClock is handed that clock on the
  // way in: whichever order the two calls come in, its solves are timed on the
  // axis the cycle stamps.
  {
    auto later = std::make_unique<FakeSegmentPlanner>();
    FakeSegmentPlanner* const raw = later.get();
    g_calls.Clear();
    rig->cycle.InstallSegmentPlanner(std::move(later));
    rig->segment = raw;
    EXPECT_EQ(raw->clock_seen, &StepClock);
    EXPECT_EQ(g_calls.Taken(), (Calls{Call::kSetClock}));
  }
  // "A segment planner is installed" does not make it a MpcSegmentPlanner:
  // MpcSegmentPlannerForDiagnostics() answers an unconfigured one, as a cycle without its own
  // always has.
  EXPECT_TRUE(rig->cycle.SegmentPlannerConfigured());
  EXPECT_FALSE(rig->cycle.MpcSegmentPlannerForDiagnostics().Configured());
  // Installing nothing is the cleared state: the wake is the S6-A stub again,
  // and reads the clock once, for the "no plan" it publishes alone.
  rig->cycle.InstallSegmentPlanner(nullptr);
  rig->cycle.InstallSearch(nullptr);
  rig->segment = nullptr;
  rig->search = nullptr;
  EXPECT_FALSE(rig->cycle.SegmentPlannerConfigured());
  EXPECT_FALSE(rig->cycle.SearchConfigured());
  EXPECT_FALSE(rig->cycle.MpcSegmentPlannerForDiagnostics().Configured());
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  EXPECT_EQ(rig->Wake(), (Calls{Call::kClock}));
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
  EXPECT_FALSE(rig->rec.plan_valid);
  EXPECT_FALSE(rig->segment_box.Load().valid);
}

TEST(PlannerCycleInterfaces, ASearchWakePublishesThePairThroughBothInterfaces) {
  for (const bool matched : {true, false}) {
    SCOPED_TRACE(matched ? "the covariance is the trajectory's" : "another snapshot's covariance");
    auto rig = std::make_unique<FakeRig>();
    rig->boxes.rt.Store(RtIn(Mode::kTracking));
    rig->boxes.traj.Store(Traj(3));
    rig->boxes.cov.Store(Cov(matched ? 3 : 2));
    // The order of a pair wake: search, first solve, ONE clock read, the
    // "can the RT still read it" question at that stamp, then both are told.
    EXPECT_EQ(rig->Wake(),
              (Calls{Call::kSearchPlan, Call::kPlanFirst, Call::kClock, Call::kStartsInTime,
                     Call::kSearchNotePublished, Call::kSegmentNotePublished}));
    const std::int64_t stamp = kStepBase + kMs;  // the first read of the clock
    const PlannerCycleRecord& rec = rig->rec;
    EXPECT_EQ(rec.outcome, CycleOutcome::kPublished);
    EXPECT_TRUE(rec.plan_valid);
    EXPECT_TRUE(rec.search_valid);
    EXPECT_EQ(rec.plan_id, 1U);
    EXPECT_EQ(rec.cov_matched, matched);
    EXPECT_EQ(rec.publish_ns, stamp);
    EXPECT_EQ(rec.search.n_ik, kFakeIkCount) << "the record is not the installed search's";
    EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kPublished);
    EXPECT_EQ(rec.segment.kind, SegmentKind::kFirst);
    EXPECT_EQ(rec.segment.segment_seq, 1U);
    EXPECT_EQ(rec.segment.publish_ns, stamp);
    // The search was handed this wake's inputs.
    EXPECT_EQ(rig->search->plan_sequence, 3U);
    EXPECT_EQ(rig->search->plan_cov_sequence, matched ? 3U : 2U);
    EXPECT_EQ(rig->search->plan_cov_matched, matched);
    EXPECT_EQ(rig->search->plan_now_ns, 1995 * kMs);
    // The segment planner was handed the plan under the id it is published
    // with, and the prediction it was searched on — the pairing flag with it.
    EXPECT_EQ(rig->segment->first_plan_id, 1U);
    EXPECT_EQ(rig->segment->first_p_c, kFakeCatchPoint);
    EXPECT_FALSE(rig->segment->ball_empty);
    EXPECT_EQ(rig->segment->ball_sequence, 3U);
    EXPECT_EQ(rig->segment->ball_cov_sequence, matched ? 3U : 2U);
    EXPECT_EQ(rig->segment->ball_cov_matched, matched);
    // Its answer decided the publish: node 0 at instant 0 is "in time" only
    // because the installed planner said so.
    EXPECT_EQ(rig->segment->starts_publish_ns, stamp);
    EXPECT_EQ(rig->segment->starts_t0_ns, 0);
    // Both boxes hold what the fakes produced, under one stamp.
    const PlanSnapshot plan = rig->boxes.plan.Load();
    const SegmentSnapshot seg = rig->segment_box.Load();
    ASSERT_TRUE(plan.valid);
    EXPECT_EQ(plan.p_c, kFakeCatchPoint);
    EXPECT_EQ(plan.plan_id, 1U);
    EXPECT_EQ(plan.publish_ns, stamp);
    ASSERT_TRUE(seg.valid);
    EXPECT_EQ(seg.plan_id, 1U);
    EXPECT_EQ(seg.segment_seq, 1U);
    EXPECT_EQ(seg.publish_ns, stamp);
    // And both were told what was stored.
    EXPECT_EQ(rig->search->noted_plan_id, 1U);
    EXPECT_EQ(rig->search->noted_publish_ns, stamp);
    EXPECT_TRUE(rig->search->noted_valid);
    EXPECT_EQ(rig->segment->noted_seq, 1U);
    EXPECT_EQ(rig->segment->noted_publish_ns, stamp);
  }
}

TEST(PlannerCycleInterfaces, TheInstalledObjectsAnswersDecideWhatASearchWakePublishes) {
  const auto search_wake = [](FakeRig& rig) {
    rig.boxes.rt.Store(RtIn(Mode::kTracking));
    rig.boxes.traj.Store(Traj(3));
    rig.boxes.cov.Store(Cov(3));
    return rig.Wake();
  };
  {
    // The search says "publish nothing": the segment planner is not asked.
    auto rig = std::make_unique<FakeRig>();
    rig->search->publish = false;
    EXPECT_EQ(search_wake(*rig), (Calls{Call::kSearchPlan}));
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
  }
  {
    // "No plan" goes out alone, and the search is told about it.
    auto rig = std::make_unique<FakeRig>();
    rig->search->valid = false;
    EXPECT_EQ(search_wake(*rig),
              (Calls{Call::kSearchPlan, Call::kClock, Call::kSearchNotePublished}));
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
    EXPECT_FALSE(rig->rec.plan_valid);
    EXPECT_FALSE(rig->search->noted_valid);
    EXPECT_FALSE(rig->segment_box.Load().valid);
  }
  {
    // A withheld first segment withholds the plan: no clock read, no store,
    // nobody told.
    auto rig = std::make_unique<FakeRig>();
    rig->segment->first_ok = false;
    EXPECT_EQ(search_wake(*rig), (Calls{Call::kSearchPlan, Call::kPlanFirst}));
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    EXPECT_TRUE(rig->rec.search_valid);
    EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
    EXPECT_FALSE(rig->segment_box.Load().valid);
    EXPECT_EQ(rig->cycle.LastPlanId(), 0U);
  }
  {
    // A segment the RT could no longer read drops the pair at the re-check.
    auto rig = std::make_unique<FakeRig>();
    rig->segment->starts_in_time = false;
    EXPECT_EQ(search_wake(*rig),
              (Calls{Call::kSearchPlan, Call::kPlanFirst, Call::kClock, Call::kStartsInTime}));
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kSuperseded);
    EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kSuperseded);
    EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
    EXPECT_FALSE(rig->segment_box.Load().valid);
  }
}

TEST(PlannerCycleInterfaces, AFollowedPlansWakeHandsOverThatPlansBallOrNone) {
  const Calls replan_published{Call::kFollowedTrack, Call::kReplan,
                               Call::kClock,         Call::kSourceSeq,
                               Call::kStartsInTime,  Call::kSegmentNotePublished};
  auto rig = std::make_unique<FakeRig>();
  const std::int64_t pair_stamp = rig->PublishPair();
  // APPROACH, the box holds a newer snapshot of the plan's track: no search,
  // the ball is handed over, and its pairing flag is THIS wake's reading — the
  // record's cov_matched belongs to the search path and stays false.
  rig->boxes.rt.Store(Following(Mode::kApproach, 1));
  rig->boxes.traj.Store(Traj(4));
  rig->boxes.cov.Store(Cov(4));
  EXPECT_EQ(rig->Wake(), replan_published);
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kIdle) << "a replan changed the wake's outcome (MD-29)";
  EXPECT_FALSE(rig->rec.cov_matched);
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
  EXPECT_EQ(rig->rec.segment.segment_seq, 2U);
  EXPECT_FALSE(rig->segment->ball_empty);
  EXPECT_EQ(rig->segment->ball_sequence, 4U);
  EXPECT_TRUE(rig->segment->ball_cov_matched);
  const std::int64_t stamp = pair_stamp + kMs;  // the wake's one read
  EXPECT_EQ(rig->rec.segment.publish_ns, stamp);
  EXPECT_EQ(rig->segment->starts_publish_ns, stamp);
  EXPECT_EQ(rig->segment->source_t_eff_ns, 0) << "SourceSeq is asked at the segment's node 0";
  EXPECT_EQ(rig->segment->noted_seq, 2U);
  EXPECT_EQ(rig->segment_box.Load().segment_seq, 2U);

  // Another snapshot's covariance: still the ball, flagged unmatched.
  rig->boxes.cov.Store(Cov(3));
  EXPECT_EQ(rig->Wake(), replan_published);
  EXPECT_FALSE(rig->segment->ball_empty);
  EXPECT_EQ(rig->segment->ball_cov_sequence, 3U);
  EXPECT_FALSE(rig->segment->ball_cov_matched);
  rig->boxes.cov.Store(Cov(4));

  // The trajectory in the box is not the followed plan's track — by the
  // segment planner's account of that plan, not by rt.track_generation: no
  // ball. Still a replan; what to do without one is the planner's call.
  rig->segment->followed_generation = kTrack + 1;
  EXPECT_EQ(rig->Wake(), replan_published);
  EXPECT_TRUE(rig->segment->ball_empty);
  rig->segment->followed_generation = kTrack;
  rig->segment->followed = false;
  EXPECT_EQ(rig->Wake(), replan_published);
  EXPECT_TRUE(rig->segment->ball_empty);
  rig->segment->followed = true;

  // COMMITTED: σ_ℓ is recorded first (Monitor, with this wake's pairing),
  // then the same replan with the ball.
  rig->boxes.rt.Store(Following(Mode::kCommitted, 1));
  Calls committed{Call::kSearchMonitor};
  committed.insert(committed.end(), replan_published.begin(), replan_published.end());
  EXPECT_EQ(rig->Wake(), committed);
  EXPECT_EQ(rig->search->monitor_sequence, 4U);
  EXPECT_TRUE(rig->search->monitor_cov_matched);
  EXPECT_EQ(rig->search->monitor_plan_id, 1U);
  EXPECT_EQ(rig->rec.search.sigma_l, kFakeSigmaL);
  EXPECT_FALSE(rig->segment->ball_empty);
  EXPECT_TRUE(rig->segment->ball_cov_matched);

  // DECEL: after the catch no ball is read — the view is EMPTY although the
  // box still holds the followed track, and the followed track is not asked.
  rig->boxes.rt.Store(Following(Mode::kDecel, 1));
  EXPECT_EQ(rig->Wake(), (Calls{Call::kReplan, Call::kClock, Call::kSourceSeq, Call::kStartsInTime,
                                Call::kSegmentNotePublished}));
  EXPECT_TRUE(rig->segment->ball_empty);
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
  const std::uint32_t last_seq = rig->cycle.LastSegmentSeq();

  // The re-check's two questions, each refusing on its own; neither stores
  // nor tells the planner.
  rig->segment->source_seq = kFakeSourceSeq + 1;  // the RT reports another source now
  EXPECT_EQ(rig->Wake(), (Calls{Call::kReplan, Call::kClock, Call::kSourceSeq}));
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kSuperseded);
  rig->segment->source_seq = kFakeSourceSeq;
  rig->segment->starts_in_time = false;
  EXPECT_EQ(rig->Wake(),
            (Calls{Call::kReplan, Call::kClock, Call::kSourceSeq, Call::kStartsInTime}));
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kSuperseded);
  rig->segment->starts_in_time = true;
  // A solve that is not publishable: the clock is not read at all.
  rig->segment->replan_ok = false;
  EXPECT_EQ(rig->Wake(), (Calls{Call::kReplan}));
  EXPECT_EQ(rig->cycle.LastSegmentSeq(), last_seq);
  EXPECT_EQ(rig->segment_box.Load().segment_seq, last_seq);
}

TEST(PlannerCycleInterfaces, ATrialResetReachesBothBeforeAnythingElse) {
  auto rig = std::make_unique<FakeRig>();
  static_cast<void>(rig->PublishPair());
  ASSERT_TRUE(rig->segment_box.Load().valid);
  rig->boxes.rt.Store(RtIn(Mode::kTracking, /*reset_epoch=*/2));
  // The new trial's first wake: both forget, then it is an ordinary search
  // wake — the "right after a pair" wait went with the trial (no ControlDtNs).
  EXPECT_EQ(rig->Wake(),
            (Calls{Call::kSearchResetTrial, Call::kSegmentResetTrial, Call::kSearchPlan,
                   Call::kPlanFirst, Call::kClock, Call::kStartsInTime, Call::kSearchNotePublished,
                   Call::kSegmentNotePublished}));
  EXPECT_TRUE(rig->rec.reset_seen);
  rig->boxes.rt.Store(RtIn(Mode::kIdle, /*reset_epoch=*/3));
  EXPECT_EQ(rig->Wake(), (Calls{Call::kSearchResetTrial, Call::kSegmentResetTrial}));
  EXPECT_FALSE(rig->segment_box.Load().valid) << "the ended trial's segment stays in the box";
}

TEST(PlannerCycleInterfaces, RightAfterAPairTheSearchWaitsTheInstalledPlannersPeriods) {
  // Three control periods OF THE INSTALLED PLANNER: one second each here. On
  // the real planner's 2 ms the first wake below would already search.
  auto rig = std::make_unique<FakeRig>();
  const std::int64_t stamp = rig->PublishPair();
  PlannerRtState s = RtIn(Mode::kTracking);
  s.rt_state_ns = stamp + 3 * kFakeControlDtNs;
  rig->boxes.rt.Store(s);
  EXPECT_EQ(rig->Wake(), (Calls{Call::kControlDtNs}));
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kIdle);
  s.rt_state_ns = stamp + 3 * kFakeControlDtNs + 1;
  rig->boxes.rt.Store(s);
  const Calls calls = rig->Wake();
  ASSERT_GE(calls.size(), 2U);
  EXPECT_EQ(calls[0], Call::kControlDtNs);
  EXPECT_EQ(calls[1], Call::kSearchPlan);
}

TEST(PlannerCycleInterfaces, AWakeThroughTheInterfacesAllocatesNothing) {
  // G3-K for the paths OneWakeAllocatesNothing does not reach — it runs the
  // stub and binds no decel box: the pair, the three kinds of replan and a
  // trial reset, with implementations installed. What an implementation
  // allocates is its own suite's (AFullSearchAllocatesNothing, the decel
  // planner's malloc gates); this is the cycle's part.
  auto rig = std::make_unique<FakeRig>();
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  static_cast<void>(rig->PublishPair());  // warm, outside the gate

  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  int pairs = 0;
  int replans = 0;
  int resets = 0;
  bool overflow = false;
  constexpr int kRounds = 50;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < kRounds; ++i) {
      const auto epoch = static_cast<std::uint32_t>(10 + i);
      g_calls.Clear();
      rig->boxes.rt.Store(RtIn(Mode::kTracking, epoch));
      PlannerCycleRecord rec = rig->cycle.Run(NowReal{2 + i});
      resets += rec.reset_seen ? 1 : 0;
      pairs += rec.segment.outcome == SegmentOutcome::kPublished && rec.plan_valid ? 1 : 0;
      const std::uint32_t plan_id = rig->cycle.LastPlanId();
      for (const Mode m : {Mode::kApproach, Mode::kCommitted, Mode::kDecel}) {
        PlannerRtState s = Following(m, plan_id);
        s.reset_epoch = epoch;
        rig->boxes.rt.Store(s);
        rec = rig->cycle.Run(NowReal{2 + i});
        replans += rec.segment.outcome == SegmentOutcome::kPublished ? 1 : 0;
      }
      overflow = overflow || g_calls.overflow;
    }
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_EQ(resets, kRounds) << "the gated loop did not exercise the reset path";
  EXPECT_EQ(pairs, kRounds) << "the gated loop did not exercise the pair path";
  EXPECT_EQ(replans, 3 * kRounds) << "the gated loop did not exercise the replan paths";
  EXPECT_FALSE(overflow);
  EXPECT_EQ(heap, 0U);
  EXPECT_EQ(eigen, 0U);
}

}  // namespace
