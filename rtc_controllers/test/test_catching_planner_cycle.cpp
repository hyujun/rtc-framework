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
#include "rtc_controllers/catching/catching_params.hpp"
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
#include <fstream>
#include <iterator>
#include <limits>
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
using rtc::catching::CatchSolution;
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
using rtc::catching::PlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanRefusal;
using rtc::catching::PlanSnapshot;
using rtc::catching::ReplaceStep;
using rtc::catching::ReportedSegments;
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
      // MPC E1-F03 (MD-29): DECEL hosts the segment MPC's post-catch replans.
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
  wait_pose: [0.1, -0.2, 0.3]
  search:
    grid:
      budget_s: 0.015
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
           "planner: {search: {grid: {budget_s: 0.0005}}}",
           "planner: {search: {grid: {budget_s: 0.06}}}",
           "planner: {search: {grid: {budget_s: .nan}}}",
           "planner: {wait_pose: []}",
           "planner: {wait_pose: 0.3}",
           "planner: {wait_pose: [0.1, x]}",
           "planner: {wait_pose: [0.1, .inf]}",
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
  // The boundaries themselves are inside the range.
  EXPECT_NO_THROW(static_cast<void>(ParsePlannerParams(
      YAML::Load("planner: {wake_timeout_s: 0.005, search: {grid: {budget_s: 0.05}}}"))));
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
  freeze: {T_freeze: 0.36}
  search:
    grid:
      max_ik: 5
      n_settle: 2
      slice: {dt: 0.05, t_lead_min: 0.3, t_max: 0.9}
      time: {margin: 0.02}
      unc: {kappa_sigma: 0.4}
      gamma: {margin: 0.2, eta_v: 0.9}
      budget: {n_sigma: 2.5, sigma_trk: 0.001, clock_err: 0.002}
      hand: {d_eff: 0.28, r_cap: 0.024, provisional: true}
      switch: {delta_J: 0.2, eta_jump: 0.4}
      score: {w_sigma: 2, w_t: 3, w_q: 0.5, w_late: 0.1, w_gamma: 4, penalty: 20}
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
  // T_freeze, d_eff/r_cap are decisions (L3 §6 TBD): absent or TBD must read
  // as UNSET so the binding parks, never as a plausible number.
  for (const char* yaml : {"planner: {enabled: true}",
                           "planner: {freeze: {T_freeze: TBD}, search: {grid: {hand: {d_eff: TBD, "
                           "r_cap: TBD}}}}"}) {
    const auto p = ParsePlannerParams(YAML::Load(yaml));
    EXPECT_TRUE(std::isnan(p.t_freeze)) << yaml;
    EXPECT_TRUE(std::isnan(p.LeadMin())) << yaml << ": t_lead_min falls back to T_freeze";
    EXPECT_TRUE(std::isnan(p.d_eff)) << yaml;
    EXPECT_TRUE(std::isnan(p.r_cap)) << yaml;
    EXPECT_TRUE(p.sub_model.empty()) << yaml;
  }
}

TEST(PlannerParams, AMalformedSearchKeyIsRefused) {
  for (const char* bad : {
           "planner: {sub_model: TBD}",
           "planner: {sub_model: ''}",
           "planner: {search: {grid: {max_ik: 0}}}",
           "planner: {search: {grid: {max_ik: 41}}}",
           "planner: {search: {grid: {slice: {t_max: 2.0}}}}",
           "planner: {search: {grid: {slice: 3}}}",
           "planner: {freeze: {T_freeze: -0.1}}",
           "planner: {search: {grid: {score: {penalty: -1}}}}",
           "planner: {search: {grid: {switch: {eta_jump: 0}}}}",
           "planner: {search: {grid: {switch: {eta_jump: 1.5}}}}",
           // Retired by the acceleration budget (decision ⑥): refused, never
           // silently run on the eta_jump default.
           "planner: {search: {grid: {switch: {e_jump_max: 0.01}}}}",
           "planner: {search: {grid: {switch: {ed_jump_max: 0.05}}}}",
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
}

// MD-94: the search does not judge where the catch point is. The removed
// `workspace` map is not read and does not make the parser throw, whatever is
// in it — the binding parks on the key (kRemovedCatchingKeys), which keeps the
// robot up where a throw here would fail the whole configure.
TEST(PlannerParams, TheRemovedWorkspaceMapIsNotReadAndIsReportedAsRemoved) {
  const rtc::catching::PlannerParams defaults = ParsePlannerParams(YAML::Load("planner: {}"));
  for (const char* workspace :
       {"{catch_box: {min: [0.1, -0.3, 0.2], max: [1.0, 0.3, 0.9]}}", "{catch_box: TBD}",
        "{catch_box: {min: [1, 0, 0], max: [0, 1, 1]}}", "{}", "3"}) {
    const std::string yaml =
        std::string("planner: {search: {grid: {workspace: ") + workspace + "}}}";
    const YAML::Node tree = YAML::Load(yaml);
    rtc::catching::PlannerParams p;
    ASSERT_NO_THROW(p = ParsePlannerParams(tree)) << yaml;
    EXPECT_EQ(p.max_ik, defaults.max_ik) << yaml;
    const auto removed = rtc::catching::FindRemovedCatchingKeys(tree);
    ASSERT_EQ(removed.size(), 1U) << yaml;
    EXPECT_STREQ(removed[0], "planner.search.grid.workspace") << yaml;
  }
}

// `SwitchStep` evaluates a ramp per sample on the planner thread for every switch
// check: the count is bounded above (kSwitchSamplesMax), refused by key name.
TEST(PlannerParams, SwitchSamplesIsBoundedAboveByName) {
  const auto message = [](int samples) -> std::string {
    try {
      static_cast<void>(ParsePlannerParams(YAML::Load(
          "planner: {search: {grid: {switch: {samples: " + std::to_string(samples) + "}}}}")));
    } catch (const std::invalid_argument& e) {
      return e.what();
    }
    return {};
  };
  EXPECT_EQ(
      ParsePlannerParams(YAML::Load("planner: {search: {grid: {switch: {samples: " +
                                    std::to_string(rtc::catching::kSwitchSamplesMax) + "}}}}"))
          .switch_samples,
      rtc::catching::kSwitchSamplesMax);
  const std::string why = message(rtc::catching::kSwitchSamplesMax + 1);
  ASSERT_FALSE(why.empty()) << "samples = kSwitchSamplesMax + 1 was accepted";
  EXPECT_NE(why.find("'planner.search.grid.switch.samples'"), std::string::npos) << why;
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
constexpr std::int64_t kFakeReplanBudgetNs = 7 * kMs;  // nobody's budget.replan_s
constexpr std::int64_t kFakeFirstLeadNs = 300 * kMs;   // nor anybody's first-solve lead
// What crosses between the two fakes through the cycle. None of it is what a
// real search or planner produces: joint angles of hundreds of radians, a
// negative cost, segments numbered far above anything the cycle has stored.
constexpr std::uint32_t kFakePendingSeq = 70'001;
constexpr std::uint32_t kFakeFollowingSeq = 70'002;
constexpr double kFakePendingQ = 611.0;
constexpr double kFakeFollowingQ = -612.0;
constexpr std::uint32_t kFakeSolutionSourceSeq = 80'001;
constexpr double kFakeSolutionQ = 813.0;
constexpr double kFakeSolutionCost = -814.0;

class FakeSearch final : public rtc::catching::CatchSearch {
 public:
  [[nodiscard]] PlanSnapshot Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt,
                                  const ReportedSegments& arm, NowReal now,
                                  std::int64_t budget_cap_ns,
                                  SearchStats& stats) noexcept override {
    g_calls.Add(Call::kSearchPlan);
    plan_budget_cap_ns = budget_cap_ns;
    plan_rt_iteration = rt.rt_iteration;
    // Read under the flags only: a snapshot whose flag is false is not written.
    arm_has_pending = arm.has_pending;
    arm_has_following = arm.has_following;
    arm_pending_seq = arm.has_pending ? arm.pending.segment_seq : 0;
    arm_following_seq = arm.has_following ? arm.following.segment_seq : 0;
    arm_pending_q = arm.has_pending ? arm.pending.q[0] : 0.0;
    arm_following_q = arm.has_following ? arm.following.q[0] : 0.0;
    plan_sequence = traj.token.snapshot_sequence;
    plan_cov_sequence = cov.token.snapshot_sequence;
    plan_cov_matched = cov_matched;
    plan_now_ns = now.ns;
    stats = SearchStats{};
    stats.n_ik = kFakeIkCount;
    stats.publish = publish;
    stats.decision = decision;
    PlanSnapshot p{};
    p.token = traj.token;
    p.token.activation_generation = rt.activation_generation;
    p.rt_iteration = rt.rt_iteration;
    p.rt_state_ns = rt.rt_state_ns;
    p.valid = valid;
    p.t_c_ns = t_c_ns;
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

  // Not in the call log: the order tests assert the wake's calls, and the
  // clock is handed over on the configure path. A null clock leaves the one
  // held, as the contract says.
  void SetClock(ClockFn clock) noexcept override {
    ++set_clock_calls;
    if (clock != nullptr) {
      clock_seen = clock;
    }
  }

  // Not in the call log, as FakeSegmentPlanner::Reported is not: the order
  // tests assert the wake's calls as they were before the two crossed.
  [[nodiscard]] const CatchSolution* Solution() const noexcept override {
    ++solution_calls;
    return has_solution ? &solution : nullptr;
  }

  FakeSearch() {
    solution.seg.valid = true;
    solution.seg.t_c_ns = kFakeTc;
    solution.seg.q[0] = kFakeSolutionQ;
    solution.source_seq = kFakeSolutionSourceSeq;
    solution.cost_reference = kFakeSolutionCost;
  }

  // What it answers.
  bool valid{true};
  bool publish{true};
  bool has_solution{true};
  // The catch instant of the plan it answers with.
  std::int64_t t_c_ns{kFakeTc};
  // The switching verdict it records (the default is SearchStats' own).
  rtc::catching::SwitchDecision decision{rtc::catching::SwitchDecision::kNoCurrent};
  CatchSolution solution{};
  // What it was handed.
  ClockFn clock_seen{nullptr};
  int set_clock_calls{0};
  bool arm_has_pending{false};
  bool arm_has_following{false};
  std::uint32_t arm_pending_seq{0};
  std::uint32_t arm_following_seq{0};
  double arm_pending_q{0.0};
  double arm_following_q{0.0};
  mutable int solution_calls{0};
  std::uint64_t plan_sequence{0};
  std::uint64_t plan_cov_sequence{0};
  bool plan_cov_matched{false};
  std::int64_t plan_now_ns{0};
  std::int64_t plan_budget_cap_ns{-1};
  std::uint64_t plan_rt_iteration{0};
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
                               const BallPrediction& ball, const CatchSolution* solution,
                               SegmentSnapshot& out, SegmentRecord& rec) noexcept override {
    g_calls.Add(Call::kPlanFirst);
    first_rt_iteration = rt.rt_iteration;
    first_rt_plan_active = rt.plan_active;
    NoteBall(ball);
    first_solution = solution;
    first_solution_source_seq = solution != nullptr ? solution->source_seq : 0;
    first_solution_q = solution != nullptr ? solution->seg.q[0] : 0.0;
    first_solution_cost = solution != nullptr ? solution->cost_reference : 0.0;
    first_plan_id = plan.plan_id;
    first_p_c = plan.p_c;
    Fill(plan.plan_id, plan.t_c_ns, out, rec);
    rec.kind = SegmentKind::kFirst;
    out.t0_ns = first_t0_ns;
    if (rt.plan_active) {
      // The first segment of a replacement starts on a reported segment, and
      // says which (SegmentPlanner::PlanFirst): the cycle reads it back.
      rec.source_seq = kFakeSourceSeq;
      rec.x0_from_segment = true;
    }
    if (!first_ok) {
      rec.outcome = first_refusal;
    }
    return first_ok;
  }

  [[nodiscard]] bool Replan(const PlannerRtState& rt, const BallPrediction& ball,
                            SegmentSnapshot& out, SegmentRecord& rec) noexcept override {
    g_calls.Add(Call::kReplan);
    replan_rt_iteration = rt.rt_iteration;
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

  // Not in the call log, like Reported below: the order tests assert the
  // wake's solves, stores and clock reads.
  [[nodiscard]] std::int64_t ReplanBudgetNs() const noexcept override {
    ++replan_budget_calls;
    return replan_budget_ns;
  }

  [[nodiscard]] std::int64_t EarliestFirstStartNs(std::int64_t now_ns) const noexcept override {
    ++earliest_calls;
    earliest_now_ns = now_ns;
    return now_ns + first_lead_ns;
  }

  // Not in the call log (see FakeSearch::Solution); counted here.
  void Reported(const PlannerRtState& rt, ReportedSegments& out) const noexcept override {
    ++reported_calls;
    reported_rt_iteration = rt.rt_iteration;
    out.has_pending = report_pending;
    out.has_following = report_following;
    if (report_pending) {
      out.pending = SegmentSnapshot{};
      out.pending.segment_seq = kFakePendingSeq;
      out.pending.q[0] = kFakePendingQ;
    }
    if (report_following) {
      out.following = SegmentSnapshot{};
      out.following.segment_seq = kFakeFollowingSeq;
      out.following.q[0] = kFakeFollowingQ;
    }
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
  // What a withheld first segment says, node 0 of the first segment it fills
  // (0: see Fill), its replan budget, and how far after `now` a first segment
  // can start at the earliest.
  SegmentOutcome first_refusal{SegmentOutcome::kSolveFailed};
  std::int64_t first_t0_ns{0};
  std::int64_t replan_budget_ns{kFakeReplanBudgetNs};
  std::int64_t first_lead_ns{kFakeFirstLeadNs};
  // On a first search wake only a fake can report segments (the real planner
  // has published none yet); on a following wake a real one does too.
  bool report_pending{true};
  bool report_following{true};
  // What it was handed.
  const CatchSolution* first_solution{nullptr};
  std::uint32_t first_solution_source_seq{0};
  double first_solution_q{0.0};
  double first_solution_cost{0.0};
  mutable int reported_calls{0};
  mutable std::uint64_t reported_rt_iteration{0};
  mutable int replan_budget_calls{0};
  mutable int earliest_calls{0};
  mutable std::int64_t earliest_now_ns{0};
  std::uint64_t first_rt_iteration{0};
  bool first_rt_plan_active{false};
  std::uint64_t replan_rt_iteration{0};
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

TEST(PlannerCycleInterfaces, InstalledIsConfiguredAndTheClockReachesBothInterfaces) {
  auto rig = std::make_unique<FakeRig>();
  // The rig's SetClock, then its first wake's reset: the one logged call is
  // the segment planner's (the search's clock hand-over is not in the log).
  EXPECT_EQ(rig->segment->clock_seen, &StepClock);
  EXPECT_EQ(rig->search->clock_seen, &StepClock);
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
  EXPECT_TRUE(rig->cycle.SegmentPlannerConfigured());
  // Installing nothing is the cleared state: the wake is the S6-A stub again,
  // and reads the clock once, for the "no plan" it publishes alone.
  rig->cycle.InstallSegmentPlanner(nullptr);
  rig->cycle.InstallSearch(nullptr);
  rig->segment = nullptr;
  rig->search = nullptr;
  EXPECT_FALSE(rig->cycle.SegmentPlannerConfigured());
  EXPECT_FALSE(rig->cycle.SearchConfigured());
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  EXPECT_EQ(rig->Wake(), (Calls{Call::kClock}));
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
  EXPECT_FALSE(rig->rec.plan_valid);
  EXPECT_FALSE(rig->segment_box.Load().valid);
}

TEST(PlannerCycleInterfaces, InstallSearchAndSetClockBothReachTheInstalledSearch) {
  auto rig = std::make_unique<FakeRig>();
  // The rig installed the search before its SetClock: the SetClock reached it.
  ASSERT_NE(rig->search, nullptr);
  EXPECT_EQ(rig->search->clock_seen, &StepClock);
  // A later SetClock replaces what the search holds.
  rig->cycle.SetClock(&FixedClock);
  EXPECT_EQ(rig->search->clock_seen, &FixedClock);
  // A search installed AFTER the cycle's SetClock is handed that clock on the
  // way in, once, whichever order the two calls come in.
  auto later = std::make_unique<FakeSearch>();
  FakeSearch* const raw = later.get();
  EXPECT_TRUE(raw->clock_seen == nullptr);
  rig->cycle.InstallSearch(std::move(later));
  rig->search = raw;
  EXPECT_EQ(raw->clock_seen, &FixedClock);
  EXPECT_EQ(raw->set_clock_calls, 1);
  rig->cycle.SetClock(&StepClock);
  EXPECT_EQ(raw->clock_seen, &StepClock);
  EXPECT_EQ(raw->set_clock_calls, 2);
  // Installing nothing hands nothing, and SetClock with no search is fine.
  rig->cycle.InstallSearch(nullptr);
  rig->search = nullptr;
  rig->cycle.SetClock(&FixedClock);
  EXPECT_FALSE(rig->cycle.SearchConfigured());
}

// ── While a plan is followed the search goes on (E1-F16) ─────────────────────

// The cycle's parameters with the search allowed to run while a plan is
// followed: until the catch instant is `t_stop_plan_s` away.
PlannerParams FollowingSearchParams(double t_stop_plan_s) {
  PlannerParams params{};
  params.t_freeze = 0.1;
  params.t_stop_plan = t_stop_plan_s;
  return params;
}

[[nodiscard]] int IndexOf(const Calls& calls, Call what) {
  for (std::size_t i = 0; i < calls.size(); ++i) {
    if (calls[i] == what) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

TEST(PlannerCycleFollowing, TheSearchOfAFollowingWakeIsHandedTheSegmentsTheRtReports) {
  using rtc::catching::SwitchDecision;

  struct Case {
    const char* what;
    bool publish;
    bool valid;
    SwitchDecision decision;
  };

  // What the search answers. What the wake then does with each answer — the
  // pair, or the replan behind the search — is PlannerCycleReplacement's; here:
  // whatever the answer, the search was handed the RT's segments.
  const Case cases[] = {
      {"another plan is better", true, true, SwitchDecision::kReplaced},
      {"the followed plan, refreshed", true, true, SwitchDecision::kRefreshed},
      {"kept by hysteresis", false, true, SwitchDecision::kHeldHysteresis},
      {"no candidate this wake", true, false, SwitchDecision::kReplaced},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    auto rig = std::make_unique<FakeRig>();
    rig->cycle.Configure(FollowingSearchParams(0.5));
    rig->search->publish = c.publish;
    rig->search->valid = c.valid;
    rig->search->decision = c.decision;
    rig->boxes.rt.Store(Following(Mode::kApproach, 9));
    rig->boxes.traj.Store(Traj(3));
    rig->boxes.cov.Store(Cov(3));
    // 10 s before the followed catch instant: far outside t_stop_plan.
    const Calls calls = rig->Wake(kFakeTc - 10'000 * kMs);
    ASSERT_GE(IndexOf(calls, Call::kSearchPlan), 0);
    // The search was handed the segments the RT reports — the ones a
    // candidate's arm motion has to start on.
    EXPECT_TRUE(rig->search->arm_has_following);
    EXPECT_EQ(rig->search->arm_following_seq, kFakeFollowingSeq);
    EXPECT_EQ(rig->search->arm_following_q, kFakeFollowingQ);
    EXPECT_TRUE(rig->search->arm_has_pending);
    EXPECT_EQ(rig->search->arm_pending_seq, kFakePendingSeq);
  }
}

TEST(PlannerCycleFollowing, TheSearchStopsAtTStopPlanBeforeTheFirstFollowedCatchInstant) {
  const double t_stop_s = 0.5;
  const std::int64_t t_stop_ns = 500 * kMs;
  const auto searched_at = [](std::int64_t wake_ns, Mode mode, double t_stop_plan) {
    auto rig = std::make_unique<FakeRig>();
    rig->cycle.Configure(FollowingSearchParams(t_stop_plan));
    rig->boxes.rt.Store(Following(mode, 9));
    rig->boxes.traj.Store(Traj(3));
    rig->boxes.cov.Store(Cov(3));
    const Calls calls = rig->Wake(wake_ns);
    return IndexOf(calls, Call::kSearchPlan) >= 0;
  };
  // Both sides of the boundary: the search runs while t_c − now is ABOVE
  // t_stop_plan, and a wake exactly on it is the replan alone.
  EXPECT_TRUE(searched_at(kFakeTc - t_stop_ns - 1, Mode::kApproach, t_stop_s));
  EXPECT_FALSE(searched_at(kFakeTc - t_stop_ns, Mode::kApproach, t_stop_s));
  EXPECT_FALSE(searched_at(kFakeTc - t_stop_ns + 1, Mode::kApproach, t_stop_s));
  // COMMITTED ends it whatever the instant: the wake monitors, it does not plan.
  EXPECT_FALSE(searched_at(kFakeTc - 10'000 * kMs, Mode::kCommitted, t_stop_s));
  // A t_stop_plan nobody set is "never": parameters no profile filled in keep
  // the wake a replan alone.
  EXPECT_FALSE(searched_at(kFakeTc - 10'000 * kMs, Mode::kApproach,
                           std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(searched_at(kFakeTc - 10'000 * kMs, Mode::kApproach, 0.0));
}

TEST(PlannerCycleFollowing, TheStopIsMeasuredFromTheFirstPlanFollowedSinceTheReset) {
  auto rig = std::make_unique<FakeRig>();
  rig->cycle.Configure(FollowingSearchParams(0.5));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  // The first followed catch instant is kFakeTc: a wake 1 s before it searches.
  rig->boxes.rt.Store(Following(Mode::kApproach, 9));
  EXPECT_GE(IndexOf(rig->Wake(kFakeTc - 1000 * kMs), Call::kSearchPlan), 0);
  // Should the RT then report a LATER catch instant for the plan it follows,
  // the stop does not move out with it: 0.3 s before the first one is inside
  // t_stop_plan, although 1.3 s before the reported one.
  PlannerRtState later = Following(Mode::kApproach, 9);
  later.plan_t_c_ns = kFakeTc + 1000 * kMs;
  rig->boxes.rt.Store(later);
  EXPECT_EQ(IndexOf(rig->Wake(kFakeTc - 300 * kMs), Call::kSearchPlan), -1);
  // A reset forgets it: the next followed plan's instant is the first again.
  PlannerRtState reset = Following(Mode::kApproach, 10);
  reset.reset_epoch = 2;
  reset.plan_t_c_ns = kFakeTc + 1000 * kMs;
  rig->boxes.rt.Store(reset);
  EXPECT_GE(IndexOf(rig->Wake(kFakeTc - 300 * kMs), Call::kSearchPlan), 0);
}

TEST(PlannerCycleFollowing, AFollowingWakeEndsInAReplanOrAPairWhateverTheTrajectoryDid) {
  // No trajectory of this activation: the search has no input — and the
  // followed plan's segment is replanned all the same.
  {
    auto rig = std::make_unique<FakeRig>();
    rig->cycle.Configure(FollowingSearchParams(0.5));
    rig->boxes.rt.Store(Following(Mode::kApproach, 9));
    const Calls calls = rig->Wake(kFakeTc - 10'000 * kMs);
    EXPECT_GE(IndexOf(calls, Call::kReplan), 0);
    EXPECT_EQ(IndexOf(calls, Call::kSearchPlan), -1);
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kNoInput);
    EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
  }
  // The trajectory moved during the search: a newer snapshot of the same
  // track. A wake with no plan followed would be superseded by it; a following
  // wake is not — "superseded" counts what a publish re-check dropped (MD-29),
  // and neither of the two things this wake goes on to is dropped for a newer
  // snapshot of the track.
  for (const bool replace : {false, true}) {
    SCOPED_TRACE(replace ? "the search chose another plan" : "the search kept the plan");
    auto rig = std::make_unique<FakeRig>();
    rig->cycle.Configure(FollowingSearchParams(0.5));
    rig->search->decision = replace ? rtc::catching::SwitchDecision::kReplaced
                                    : rtc::catching::SwitchDecision::kRefreshed;
    rig->boxes.rt.Store(Following(Mode::kApproach, 9));
    rig->boxes.traj.Store(Traj(3));
    rig->boxes.cov.Store(Cov(3));

    struct Ctx {
      Boxes* boxes;
    } ctx{&rig->boxes};

    rig->cycle.SetPostSearchHookForTesting(
        [](void* user) noexcept { static_cast<Ctx*>(user)->boxes->traj.Store(Traj(4)); }, &ctx);
    const Calls calls = rig->Wake(kFakeTc - 10'000 * kMs);
    const int search = IndexOf(calls, Call::kSearchPlan);
    ASSERT_GE(search, 0);
    EXPECT_EQ(rig->rec.search.decision, rig->search->decision);
    // Either way the solve behind the search ran on the prediction the search
    // read, not on the one that landed meanwhile.
    EXPECT_EQ(rig->segment->ball_sequence, 3U);
    if (replace) {
      // The pair: its re-check asks for the same TRACK, not the same snapshot.
      EXPECT_GT(IndexOf(calls, Call::kPlanFirst), search);
      EXPECT_EQ(IndexOf(calls, Call::kReplan), -1);
      EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
      EXPECT_NE(rig->boxes.plan.sequence(), 0U);
      EXPECT_EQ(rig->boxes.plan.Load().plan_id, rig->rec.plan_id);
    } else {
      // The replan, behind the search.
      EXPECT_GT(IndexOf(calls, Call::kReplan), search);
      EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
      EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
      EXPECT_EQ(IndexOf(calls, Call::kPlanFirst), -1);
    }
  }
}

// A wake in which the RT follows NO plan is timed from the wake — as a
// following wake's search is (PlannerCycleReplacement): the search is the
// first thing either runs.
TEST(PlannerCycleFollowing, ASearchWithNoPlanFollowedIsStillTimedFromTheWake) {
  auto rig = std::make_unique<FakeRig>();
  rig->cycle.Configure(FollowingSearchParams(0.5));
  rig->boxes.rt.Store(RtIn(Mode::kTracking));
  rig->boxes.traj.Store(Traj(3));
  rig->boxes.cov.Store(Cov(3));
  const std::int64_t wake_ns = kFakeTc - 10'000 * kMs;
  static_cast<void>(rig->Wake(wake_ns));
  EXPECT_EQ(rig->search->plan_now_ns, wake_ns);
}

TEST(PlannerParams, TheSearchStopsWhereThePlanFreezesUnlessTheProfileSaysEarlier) {
  // Absent: T_freeze. Written: its own value. Neither: unset, like T_freeze.
  EXPECT_DOUBLE_EQ(
      ParsePlannerParams(YAML::Load("planner: {freeze: {T_freeze: 0.36}}")).t_stop_plan, 0.36);
  const auto p =
      ParsePlannerParams(YAML::Load("planner: {freeze: {T_freeze: 0.36, t_stop_plan: 0.5}}"));
  EXPECT_DOUBLE_EQ(p.t_freeze, 0.36);
  EXPECT_DOUBLE_EQ(p.t_stop_plan, 0.5);
  EXPECT_TRUE(std::isnan(ParsePlannerParams(YAML::Load("planner: {enabled: true}")).t_stop_plan));
  EXPECT_TRUE(std::isnan(PlannerParams{}.t_stop_plan));
  // Below T_freeze it still PARSES: that the pair is in order is the binding's
  // check, which parks and names both. A malformed number is refused here.
  EXPECT_DOUBLE_EQ(
      ParsePlannerParams(YAML::Load("planner: {freeze: {T_freeze: 0.36, t_stop_plan: 0.2}}"))
          .t_stop_plan,
      0.2);
  for (const char* bad : {"planner: {freeze: {T_freeze: 0.36, t_stop_plan: -1.0}}",
                          "planner: {freeze: {T_freeze: 0.36, t_stop_plan: soon}}",
                          "planner: {freeze: {T_freeze: 0.36, t_stop_plan: TBD}}",
                          "planner: {freeze: {T_freeze: 0.36, t_stop_plan: 9.0}}"}) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
}

TEST(PlannerCycleInterfaces, TheCycleNamesNoImplementation) {
  // What a wake calls is the two interfaces. Which search and which segment
  // planner stand behind them is the configure path's knowledge alone: a
  // cycle that included one of their headers could reach around the interface
  // without a test noticing, so the files themselves are held to it —
  // comments too, which is where the next include starts.
  for (const char* file :
       {"include/rtc_controllers/catching/planner_cycle.hpp", "src/catching/planner_cycle.cpp"}) {
    const std::string path = std::string(RTC_CONTROLLERS_SOURCE_DIR) + "/" + file;
    std::ifstream in(path);
    ASSERT_TRUE(in.is_open()) << path;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    ASSERT_FALSE(text.empty()) << path;
    for (const char* name : {"grid_catch_search", "mpc_segment_planner", "nlp_catch_search",
                             "GridCatchSearch", "MpcSegmentPlanner", "NlpCatchSearch"}) {
      EXPECT_EQ(text.find(name), std::string::npos) << file << " names " << name;
    }
  }
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

// E1-F14 (#740): what the search and the segment planner hand each other goes
// through the cycle untouched, in both directions, and the wake's order of
// calls is the one the tests above assert — neither crossing is in that log.
TEST(PlannerCycleInterfaces, TheSolutionAndTheReportedSegmentsCrossTheCycleUntouched) {
  const auto search_wake = [](FakeRig& rig) {
    rig.boxes.rt.Store(RtIn(Mode::kTracking));
    rig.boxes.traj.Store(Traj(3));
    rig.boxes.cov.Store(Cov(3));
    return rig.Wake();
  };
  const Calls pair{Call::kSearchPlan,   Call::kPlanFirst,           Call::kClock,
                   Call::kStartsInTime, Call::kSearchNotePublished, Call::kSegmentNotePublished};
  {
    SCOPED_TRACE("both reported, a solution");
    auto rig = std::make_unique<FakeRig>();
    EXPECT_EQ(search_wake(*rig), pair);
    // segment planner → search: the planner is asked once, with this wake's
    // RT report, and the search sees exactly what it answered.
    EXPECT_EQ(rig->segment->reported_calls, 1);
    EXPECT_EQ(rig->segment->reported_rt_iteration, RtIn(Mode::kTracking).rt_iteration);
    EXPECT_TRUE(rig->search->arm_has_pending);
    EXPECT_TRUE(rig->search->arm_has_following);
    EXPECT_EQ(rig->search->arm_pending_seq, kFakePendingSeq);
    EXPECT_EQ(rig->search->arm_following_seq, kFakeFollowingSeq);
    EXPECT_EQ(rig->search->arm_pending_q, kFakePendingQ);
    EXPECT_EQ(rig->search->arm_following_q, kFakeFollowingQ);
    // search → segment planner: the search's own object, not a copy the cycle
    // made and not null.
    EXPECT_EQ(rig->search->solution_calls, 1);
    EXPECT_EQ(rig->segment->first_solution, &rig->search->solution);
    EXPECT_EQ(rig->segment->first_solution_source_seq, kFakeSolutionSourceSeq);
    EXPECT_EQ(rig->segment->first_solution_q, kFakeSolutionQ);
    EXPECT_EQ(rig->segment->first_solution_cost, kFakeSolutionCost);
  }
  {
    SCOPED_TRACE("only the followed one, no solution");
    auto rig = std::make_unique<FakeRig>();
    rig->segment->report_pending = false;
    rig->search->has_solution = false;
    EXPECT_EQ(search_wake(*rig), pair);
    EXPECT_FALSE(rig->search->arm_has_pending);
    EXPECT_TRUE(rig->search->arm_has_following);
    EXPECT_EQ(rig->search->arm_following_seq, kFakeFollowingSeq);
    EXPECT_EQ(rig->segment->first_solution, nullptr);
  }
  {
    SCOPED_TRACE("only the pending one");
    auto rig = std::make_unique<FakeRig>();
    rig->segment->report_following = false;
    EXPECT_EQ(search_wake(*rig), pair);
    EXPECT_TRUE(rig->search->arm_has_pending);
    EXPECT_FALSE(rig->search->arm_has_following);
    EXPECT_EQ(rig->search->arm_pending_seq, kFakePendingSeq);
  }
  {
    // A wake whose search holds back asks for no solution: nothing is
    // published for it to go with.
    SCOPED_TRACE("the search publishes nothing");
    auto rig = std::make_unique<FakeRig>();
    rig->search->publish = false;
    EXPECT_EQ(search_wake(*rig), (Calls{Call::kSearchPlan}));
    EXPECT_EQ(rig->segment->reported_calls, 1);
    EXPECT_TRUE(rig->search->arm_has_following);
    EXPECT_EQ(rig->search->solution_calls, 0);
  }
  {
    // Without a segment planner there is nobody to report a segment — and the
    // cycle's scratch still holds the two an earlier wake was handed.
    SCOPED_TRACE("the segment planner is removed after a wake that reported both");
    auto rig = std::make_unique<FakeRig>();
    EXPECT_EQ(search_wake(*rig), pair);
    ASSERT_TRUE(rig->search->arm_has_pending);
    ASSERT_TRUE(rig->search->arm_has_following);
    rig->cycle.InstallSegmentPlanner(nullptr);
    rig->segment = nullptr;
    EXPECT_EQ(search_wake(*rig),
              (Calls{Call::kSearchPlan, Call::kClock, Call::kSearchNotePublished}));
    EXPECT_FALSE(rig->search->arm_has_pending);
    EXPECT_FALSE(rig->search->arm_has_following);
  }
}

TEST(PlannerCycleInterfaces, AWakeThroughTheInterfacesAllocatesNothing) {
  // G3-K for the paths OneWakeAllocatesNothing does not reach — it runs the
  // stub and binds no segment box: the pair, the three kinds of replan and a
  // trial reset, with implementations installed. What an implementation
  // allocates is its own suite's (AFullSearchAllocatesNothing, the MPC segment
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
  // The gated pairs carried both crossings (E1-F14): two reported segments in
  // (copied into the cycle's scratch), the search's solution out.
  EXPECT_EQ(rig->segment->reported_calls, 1 + kRounds);
  EXPECT_EQ(rig->search->arm_pending_seq, kFakePendingSeq);
  EXPECT_EQ(rig->search->arm_following_q, kFakeFollowingQ);
  EXPECT_EQ(rig->search->solution_calls, 1 + kRounds);
  EXPECT_EQ(rig->segment->first_solution, &rig->search->solution);
}

// ── A followed plan is replaced: the replacement pair (E1-F17 #743) ──────────
//
// A wake while the RT follows a plan runs the search FIRST and then one of two
// things: the search chose another plan → its first segment is solved from the
// moving arm and the two go out as a pair; anything else → the followed plan's
// segment is replanned, on the RT's report read again. Observed through the
// fakes' call log (the order), their records of what they were handed, and the
// boxes.

using rtc::catching::SwitchDecision;

// The first wake's instant: 10 s before the followed catch instant, far
// outside t_stop_plan.
constexpr std::int64_t kFollowWake = kFakeTc - 10'000 * kMs;
constexpr double kFreezeS = 0.1;
constexpr std::int64_t kFreezeNs = 100 * kMs;

// A rig whose RT follows plan 1 — the pair the cycle itself published — in
// APPROACH, and whose search answers "another plan" on the next wake.
struct ReplacingRig {
  std::unique_ptr<FakeRig> rig = std::make_unique<FakeRig>();
  std::int64_t first_stamp{0};

  explicit ReplacingRig(PlannerParams params = FollowingSearchParams(0.5)) {
    rig->cycle.Configure(params);
    first_stamp = rig->PublishPair();
    rig->search->decision = SwitchDecision::kReplaced;
    rig->boxes.rt.Store(Following(Mode::kApproach, 1));
    rig->boxes.traj.Store(Traj(4));
    rig->boxes.cov.Store(Cov(4));
  }

  FakeRig* operator->() const { return rig.get(); }
};

const Calls kReplacementPairCalls{Call::kSearchPlan,          Call::kClock,
                                  Call::kPlanFirst,           Call::kClock,
                                  Call::kSourceSeq,           Call::kStartsInTime,
                                  Call::kSearchNotePublished, Call::kSegmentNotePublished};
const Calls kReplanBehindSearchCalls{
    Call::kSearchPlan,   Call::kFollowedTrack,       Call::kReplan, Call::kClock, Call::kSourceSeq,
    Call::kStartsInTime, Call::kSegmentNotePublished};

// What the two boxes and the cycle's counters hold, to show that a wake stored
// nothing.
struct Stored {
  std::uint64_t plan_stores{0};
  std::uint64_t segment_stores{0};
  std::uint32_t last_plan_id{0};
  std::uint32_t last_segment_seq{0};

  explicit Stored(FakeRig& rig)
      : plan_stores(rig.boxes.plan.sequence()),
        segment_stores(rig.segment_box.sequence()),
        last_plan_id(rig.cycle.LastPlanId()),
        last_segment_seq(rig.cycle.LastSegmentSeq()) {}

  friend bool operator==(const Stored&, const Stored&) = default;
};

struct PairProbe {
  FakeRig* rig{nullptr};
  int calls{0};
  std::uint32_t segment_plan_id{0};
  std::uint32_t plan_plan_id{0};
};

TEST(PlannerCycleReplacement, AnotherPlanGoesOutAsAPairSegmentFirstAndNothingIsReplanned) {
  ReplacingRig rig;
  PairProbe probe{rig.rig.get()};
  rig->cycle.SetPairStoreHookForTesting(
      [](void* user) noexcept {
        auto* p = static_cast<PairProbe*>(user);
        ++p->calls;
        p->segment_plan_id = p->rig->segment_box.Load().plan_id;
        p->plan_plan_id = p->rig->boxes.plan.Load().plan_id;
      },
      &probe);
  const std::int64_t clock_before = g_step_now;
  // The search, then — its verdict being another plan — the first-solve bound
  // read on the clock, the first solve, ONE stamp, the two re-check questions
  // a replan's re-check asks too, and both told. No replan: the followed
  // plan's segment is not touched on a wake that replaces the plan.
  EXPECT_EQ(rig->Wake(kFollowWake), kReplacementPairCalls);
  const std::int64_t stamp = clock_before + 2 * kMs;  // the wake's second read
  const PlannerCycleRecord& rec = rig->rec;
  EXPECT_EQ(rec.outcome, CycleOutcome::kPublished);
  EXPECT_TRUE(rec.plan_valid);
  EXPECT_TRUE(rec.search_valid);
  EXPECT_EQ(rec.search.decision, SwitchDecision::kReplaced);
  EXPECT_EQ(rec.plan_id, 2U) << "a replacement carries a new plan id";
  EXPECT_EQ(rec.publish_ns, stamp);
  EXPECT_EQ(rec.segment.outcome, SegmentOutcome::kPublished);
  EXPECT_EQ(rec.segment.kind, SegmentKind::kFirst);
  EXPECT_EQ(rec.segment.segment_seq, 2U);
  EXPECT_EQ(rec.segment.publish_ns, stamp);
  EXPECT_EQ(rec.segment.source_seq, kFakeSourceSeq);
  EXPECT_EQ(rec.replacement.outcome, SegmentOutcome::kOff);
  EXPECT_EQ(rec.replace_step, ReplaceStep::kPublished);
  // The search ran first, so it is timed from the wake, on the wake's report
  // and with the segments the RT reports.
  EXPECT_EQ(rig->search->plan_now_ns, kFollowWake);
  EXPECT_EQ(rig->search->plan_sequence, 4U);
  EXPECT_TRUE(rig->search->arm_has_following);
  EXPECT_EQ(rig->search->arm_following_seq, kFakeFollowingSeq);
  // The first solve was handed the wake's report — the one the search solved
  // on — the plan under its new id, the search's solution and its prediction.
  EXPECT_TRUE(rig->segment->first_rt_plan_active);
  EXPECT_EQ(rig->segment->first_rt_iteration, Following(Mode::kApproach, 1).rt_iteration);
  EXPECT_EQ(rig->segment->first_plan_id, 2U);
  EXPECT_EQ(rig->segment->first_solution, &rig->search->solution);
  EXPECT_FALSE(rig->segment->ball_empty);
  EXPECT_EQ(rig->segment->ball_sequence, 4U);
  // The earliest a first segment could start was asked at the wake's first
  // clock read, and the re-check's questions at the stamp and at node 0.
  EXPECT_EQ(rig->segment->earliest_calls, 1);
  EXPECT_EQ(rig->segment->earliest_now_ns, clock_before + kMs);
  EXPECT_EQ(rig->segment->starts_publish_ns, stamp);
  EXPECT_EQ(rig->segment->source_t_eff_ns, 0);
  // Segment first: between the two stores the segment box already holds the
  // replacement's segment and the plan box still the followed plan.
  EXPECT_EQ(probe.calls, 1);
  EXPECT_EQ(probe.segment_plan_id, 2U);
  EXPECT_EQ(probe.plan_plan_id, 1U);
  // Both boxes hold the pair, under one stamp and one plan id; both were told.
  const PlanSnapshot plan = rig->boxes.plan.Load();
  const SegmentSnapshot seg = rig->segment_box.Load();
  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(seg.valid);
  EXPECT_EQ(plan.plan_id, 2U);
  EXPECT_EQ(seg.plan_id, 2U);
  EXPECT_EQ(seg.segment_seq, 2U);
  EXPECT_EQ(plan.publish_ns, stamp);
  EXPECT_EQ(seg.publish_ns, stamp);
  EXPECT_EQ(rig->search->noted_plan_id, 2U);
  EXPECT_EQ(rig->search->noted_publish_ns, stamp);
  EXPECT_EQ(rig->segment->noted_seq, 2U);
  EXPECT_EQ(rig->segment->noted_publish_ns, stamp);
  EXPECT_EQ(rig->cycle.LastPlanId(), 2U);
}

TEST(PlannerCycleReplacement, EveryOtherVerdictReplansBehindTheSearchOnTheReportReadAgain) {
  struct Case {
    const char* what;
    bool publish;
    bool valid;
    SwitchDecision decision;
  };

  const Case cases[] = {
      {"the followed plan, refreshed", true, true, SwitchDecision::kRefreshed},
      {"kept by hysteresis", false, true, SwitchDecision::kHeldHysteresis},
      {"another plan, but the search holds it back", false, true, SwitchDecision::kReplaced},
      {"no candidate this wake", true, false, SwitchDecision::kReplaced},
      {"no candidate, held", false, false, SwitchDecision::kHeldNoCandidate},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    ReplacingRig rig;
    rig->search->publish = c.publish;
    rig->search->valid = c.valid;
    rig->search->decision = c.decision;

    // The RT ticks while the search runs: the same plan, a later report.
    struct Ctx {
      FakeRig* rig;
    } ctx{rig.rig.get()};

    rig->cycle.SetPostSearchHookForTesting(
        [](void* user) noexcept {
          PlannerRtState later = Following(Mode::kApproach, 1);
          later.rt_iteration += 1;
          later.rt_state_ns += 2 * kMs;
          static_cast<Ctx*>(user)->rig->boxes.rt.Store(later);
        },
        &ctx);
    const Stored before(*rig.rig);
    // The search first, then the replan — never the other way round, and no
    // first solve.
    EXPECT_EQ(rig->Wake(kFollowWake), kReplanBehindSearchCalls);
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    EXPECT_EQ(rig->rec.search.decision, c.decision);
    EXPECT_EQ(rig->rec.search_valid, c.valid);
    EXPECT_FALSE(rig->rec.plan_valid);
    EXPECT_EQ(rig->rec.publish_ns, 0);
    EXPECT_EQ(rig->rec.replacement.outcome, SegmentOutcome::kOff);
    EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kNone) << "no replacement was attempted";
    // The replan's is the one store of the wake.
    EXPECT_EQ(rig->boxes.plan.sequence(), before.plan_stores);
    EXPECT_EQ(rig->cycle.LastPlanId(), before.last_plan_id);
    EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
    EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kAdvance);
    EXPECT_EQ(rig->segment_box.Load().segment_seq, before.last_segment_seq + 1);
    // The search was handed the wake's report; the replan the one read after.
    const std::uint64_t wake_iteration = Following(Mode::kApproach, 1).rt_iteration;
    EXPECT_EQ(rig->search->plan_rt_iteration, wake_iteration);
    EXPECT_EQ(rig->segment->replan_rt_iteration, wake_iteration + 1);
    EXPECT_EQ(rig->search->plan_now_ns, kFollowWake);
  }
}

TEST(PlannerCycleReplacement, AReportThatMovedDuringTheSearchLeavesNothingToReplan) {
  // What the RT may report by the time the search is done. The replan is of
  // the plan the wake started on: another trial, another plan, or a
  // replacement the RT holds by now, and there is nothing of it to replan.
  struct Case {
    const char* what;
    void (*change)(PlannerRtState&);
    bool replans;
  };

  const Case cases[] = {
      {"the same plan, a later tick", [](PlannerRtState& s) { s.rt_iteration += 1; }, true},
      {"committed meanwhile",
       [](PlannerRtState& s) { s.mode = static_cast<std::uint8_t>(Mode::kCommitted); }, true},
      {"a trial reset", [](PlannerRtState& s) { s.reset_epoch += 1; }, false},
      {"another activation", [](PlannerRtState& s) { s.activation_generation += 1; }, false},
      {"no plan followed", [](PlannerRtState& s) { s.plan_active = false; }, false},
      {"another plan followed", [](PlannerRtState& s) { s.plan_id += 1; }, false},
      {"another catch instant", [](PlannerRtState& s) { s.plan_t_c_ns += 1; }, false},
      {"a replacement held", [](PlannerRtState& s) { s.plan_pending = true; }, false},
      {"an invalid report", [](PlannerRtState& s) { s.valid = false; }, false},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    ReplacingRig rig;
    rig->search->decision = SwitchDecision::kRefreshed;

    struct Ctx {
      FakeRig* rig;
      void (*change)(PlannerRtState&);
    } ctx{rig.rig.get(), c.change};

    rig->cycle.SetPostSearchHookForTesting(
        [](void* user) noexcept {
          auto* x = static_cast<Ctx*>(user);
          PlannerRtState s = Following(Mode::kApproach, 1);
          x->change(s);
          x->rig->boxes.rt.Store(s);
        },
        &ctx);
    const Stored before(*rig.rig);
    const Calls calls = rig->Wake(kFollowWake);
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    if (c.replans) {
      EXPECT_EQ(calls, kReplanBehindSearchCalls);
      EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
    } else {
      EXPECT_EQ(calls, (Calls{Call::kSearchPlan}));
      EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kOff);
      EXPECT_EQ(Stored(*rig.rig), before);
    }
  }
}

TEST(PlannerCycleReplacement, WithNothingToSearchOnTheFollowedSegmentIsStillReplanned) {
  // No trajectory of this activation: no search — and the replan runs, without
  // a ball (the followed track is not even asked: the box holds none).
  ReplacingRig rig;
  rig->boxes.traj.Store(Traj(5, kActivation - 1));
  const Stored before(*rig.rig);
  EXPECT_EQ(rig->Wake(kFollowWake), (Calls{Call::kReplan, Call::kClock, Call::kSourceSeq,
                                           Call::kStartsInTime, Call::kSegmentNotePublished}));
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kNoInput);
  EXPECT_EQ(rig->boxes.plan.sequence(), before.plan_stores);
  EXPECT_EQ(rig->cycle.LastPlanId(), before.last_plan_id);
  EXPECT_TRUE(rig->segment->ball_empty);
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
  EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kAdvance);
}

// What lands between the first solve and the pair's re-check.
struct RecheckCase {
  const char* what;
  // Before the wake: what the installed objects answer.
  void (*arrange)(FakeRig&);
  // Between the solve and the re-check: what the boxes hold by then.
  void (*land)(FakeRig&);
  bool published;
};

void Nothing(FakeRig& rig) {
  static_cast<void>(rig);
}

void StoreRt(FakeRig& rig, void (*change)(PlannerRtState&)) {
  PlannerRtState s = Following(Mode::kApproach, 1);
  change(s);
  rig.boxes.rt.Store(s);
}

TEST(PlannerCycleReplacement, EachRecheckConditionAloneDropsThePairAndStoresNothing) {
  const RecheckCase cases[] = {
      // The control, and what the re-check lets through.
      {"nothing moved", &Nothing, &Nothing, true},
      {"a newer snapshot of the same track", &Nothing,
       [](FakeRig& rig) {
         rig.boxes.traj.Store(Traj(9));
         rig.boxes.cov.Store(Cov(9));
       },
       true},
      {"the same plan, a later tick", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.rt_iteration += 1; }); }, true},
      // The trajectory.
      {"another track", &Nothing,
       [](FakeRig& rig) {
         TrajectorySnapshot t = Traj(9);
         t.token.generation = kTrack + 1;
         rig.boxes.traj.Store(t);
       },
       false},
      {"another activation's trajectory", &Nothing,
       [](FakeRig& rig) { rig.boxes.traj.Store(Traj(9, kActivation + 1)); }, false},
      // The RT's report.
      {"a trial reset", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.reset_epoch += 1; }); }, false},
      {"another activation", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.activation_generation += 1; }); },
       false},
      {"an invalid report", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.valid = false; }); }, false},
      {"no plan followed any more", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.plan_active = false; }); }, false},
      {"another plan followed", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.plan_id += 1; }); }, false},
      {"another catch instant followed", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.plan_t_c_ns += 1; }); }, false},
      {"committed meanwhile", &Nothing,
       [](FakeRig& rig) {
         StoreRt(rig,
                 [](PlannerRtState& s) { s.mode = static_cast<std::uint8_t>(Mode::kCommitted); });
       },
       false},
      {"a replacement already held", &Nothing,
       [](FakeRig& rig) { StoreRt(rig, [](PlannerRtState& s) { s.plan_pending = true; }); }, false},
      // The segment the solve started on is no longer the one reported.
      {"another source segment reported",
       [](FakeRig& rig) { rig.segment->source_seq = kFakeSourceSeq + 1; }, &Nothing, false},
      // The RT could not take it any more.
      {"the segment can no longer be read",
       [](FakeRig& rig) { rig.segment->starts_in_time = false; }, &Nothing, false},
  };
  for (const RecheckCase& c : cases) {
    SCOPED_TRACE(c.what);
    ReplacingRig rig;
    c.arrange(*rig.rig);

    struct Ctx {
      FakeRig* rig;
      void (*land)(FakeRig&);
    } ctx{rig.rig.get(), c.land};

    rig->cycle.SetPostSegmentHookForTesting(
        [](void* user) noexcept {
          auto* x = static_cast<Ctx*>(user);
          x->land(*x->rig);
        },
        &ctx);
    const Stored before(*rig.rig);
    const Calls calls = rig->Wake(kFollowWake);
    EXPECT_GE(IndexOf(calls, Call::kPlanFirst), 0);
    // Published or dropped, a wake whose first segment came out publishable
    // does not replan: a segment of the followed plan would land in the one
    // slot the pair's segment is in.
    EXPECT_EQ(IndexOf(calls, Call::kReplan), -1);
    if (c.published) {
      EXPECT_EQ(calls, kReplacementPairCalls);
      EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
      EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
      EXPECT_EQ(rig->boxes.plan.Load().plan_id, 2U);
      EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kPublished);
      continue;
    }
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kSuperseded);
    EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kSuperseded);
    EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kSuperseded);
    EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kFirst);
    EXPECT_FALSE(rig->rec.plan_valid);
    EXPECT_EQ(rig->rec.publish_ns, 0);
    EXPECT_EQ(Stored(*rig.rig), before);
    EXPECT_EQ(IndexOf(calls, Call::kSearchNotePublished), -1);
    EXPECT_EQ(IndexOf(calls, Call::kSegmentNotePublished), -1);
    EXPECT_EQ(rig->boxes.plan.Load().plan_id, 1U);
    EXPECT_EQ(rig->segment_box.Load().plan_id, 1U);
  }
}

TEST(PlannerCycleReplacement, TheFreezeBoundsOfThePairsRecheckAreOnTheNanosecond) {
  // Three instants the re-check holds against T_freeze, each on both sides of
  // its bound and each with the others far inside.
  //  • The NEW catch instant: more than T_freeze after the publish stamp.
  //  • The new segment's node 0 against the FOLLOWED plan's freeze,
  //    t_c − T_freeze: before it — the RT switches at node 0 and takes no
  //    replacement once the plan it follows is frozen.
  //  • Node 0 against the NEW plan's catch instant: more than T_freeze before
  //    it — the RT drops a held pair whose catch instant is inside the freeze
  //    window when it gets to the switch.
  // (The planner's bound on where a first segment can start is put long
  // before the wake: the pre-solve check asks the last two questions of that
  // bound, and every case here has to get through it to the re-check.)
  const auto run = [](std::int64_t new_t_c_offset_from_stamp, std::int64_t node0_ns,
                      double t_freeze_s, bool node0_is_before_the_new_t_c = false) {
    PlannerParams params = FollowingSearchParams(0.5);
    params.t_freeze = t_freeze_s;
    ReplacingRig rig(params);
    rig->segment->first_lead_ns = -1'000'000 * kMs;
    // The stamp is the wake's second clock read with a freeze window (the
    // first asks where a first segment could start), its first without one.
    const bool freeze = t_freeze_s > 0.0;
    const std::int64_t stamp = g_step_now + (freeze ? 2 : 1) * kMs;
    rig->search->t_c_ns = stamp + new_t_c_offset_from_stamp;
    rig->segment->first_t0_ns =
        node0_is_before_the_new_t_c ? rig->search->t_c_ns - node0_ns : node0_ns;
    const Stored before(*rig.rig);
    const Calls calls = rig->Wake(kFollowWake);
    EXPECT_GE(IndexOf(calls, Call::kPlanFirst), 0) << "held before the solve: not the re-check";
    EXPECT_EQ(rig->rec.publish_ns == stamp, rig->rec.outcome == CycleOutcome::kPublished);
    if (rig->rec.outcome != CycleOutcome::kPublished) {
      EXPECT_EQ(rig->rec.outcome, CycleOutcome::kSuperseded);
      EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kSuperseded);
      EXPECT_EQ(Stored(*rig.rig), before);
    }
    return rig->rec.outcome == CycleOutcome::kPublished;
  };
  const std::int64_t near = 5000 * kMs;   // a new catch instant before the followed one
  const std::int64_t far = 15'000 * kMs;  // and one after it
  const std::int64_t followed_freeze = kFakeTc - kFreezeNs;
  EXPECT_TRUE(run(near, 0, kFreezeS));
  EXPECT_TRUE(run(far, 0, kFreezeS));
  // The new catch instant against the stamp.
  EXPECT_TRUE(run(kFreezeNs + 1, 0, kFreezeS));
  EXPECT_FALSE(run(kFreezeNs, 0, kFreezeS));
  EXPECT_FALSE(run(kFreezeNs - 1, 0, kFreezeS));
  EXPECT_FALSE(run(-1, 0, kFreezeS)) << "a catch instant already past";
  // Node 0 against the followed plan's freeze (the new catch instant far
  // after it).
  EXPECT_TRUE(run(far, followed_freeze - 1, kFreezeS));
  EXPECT_FALSE(run(far, followed_freeze, kFreezeS));
  EXPECT_FALSE(run(far, followed_freeze + 1, kFreezeS));
  // Node 0 against the new plan's catch instant (both far before the followed
  // plan's freeze): what is left at node 0 has to be MORE than T_freeze.
  EXPECT_TRUE(run(near, kFreezeNs + 1, kFreezeS, /*node0_is_before_the_new_t_c=*/true));
  EXPECT_FALSE(run(near, kFreezeNs, kFreezeS, /*node0_is_before_the_new_t_c=*/true));
  EXPECT_FALSE(run(near, kFreezeNs - 1, kFreezeS, /*node0_is_before_the_new_t_c=*/true));
  EXPECT_FALSE(run(near, 0, kFreezeS, /*node0_is_before_the_new_t_c=*/true))
      << "node 0 at the catch instant";
  EXPECT_FALSE(run(near, -kMs, kFreezeS, /*node0_is_before_the_new_t_c=*/true))
      << "node 0 after the catch instant";
  // Without a freeze window (T_freeze unset) none of the three has a bound:
  // the new catch instant only has to be ahead of the stamp.
  const double unset = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(run(1, followed_freeze + 1, unset));
  EXPECT_TRUE(run(near, 0, unset, /*node0_is_before_the_new_t_c=*/true));
  EXPECT_FALSE(run(0, 0, unset));
}

TEST(PlannerCycleReplacement, APairThatCouldNotStartBeforeTheFollowedPlanFreezesIsNotSolvedFor) {
  // Before the first solve: where could a first segment start at the earliest
  // (the planner's own bound, asked at the cycle's clock)? Not before the
  // followed plan's freeze → no solve; the followed plan goes on and its
  // segment is replanned. (The new plan's catch instant is 5 s after the
  // followed one's: its own bound on that start is far inside.)
  const std::int64_t followed_freeze = kFakeTc - kFreezeNs;
  for (const std::int64_t margin_ns : {std::int64_t{1}, std::int64_t{0}, -kMs}) {
    SCOPED_TRACE(margin_ns);
    ReplacingRig rig;
    rig->search->t_c_ns = kFakeTc + 5000 * kMs;
    const std::int64_t asked_at = g_step_now + kMs;  // the wake's first clock read
    // The earliest start is `margin_ns` BEFORE the followed plan's freeze.
    rig->segment->first_lead_ns = followed_freeze - margin_ns - asked_at;
    const Stored before(*rig.rig);
    const Calls calls = rig->Wake(kFollowWake);
    EXPECT_EQ(rig->segment->earliest_calls, 1);
    EXPECT_EQ(rig->segment->earliest_now_ns, asked_at);
    if (margin_ns > 0) {
      EXPECT_EQ(calls, kReplacementPairCalls);
      EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
      EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kPublished);
      continue;
    }
    Calls expected{Call::kSearchPlan, Call::kClock};
    expected.insert(expected.end(), kReplanBehindSearchCalls.begin() + 1,
                    kReplanBehindSearchCalls.end());
    EXPECT_EQ(calls, expected);
    EXPECT_EQ(IndexOf(calls, Call::kPlanFirst), -1);
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    EXPECT_EQ(rig->rec.search.decision, SwitchDecision::kReplaced);
    EXPECT_TRUE(rig->rec.search_valid);
    EXPECT_EQ(rig->rec.replacement.outcome, SegmentOutcome::kOff) << "no solve ran";
    EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kTooLateFollowed);
    EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
    EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kAdvance);
    EXPECT_EQ(rig->boxes.plan.sequence(), before.plan_stores);
    EXPECT_EQ(rig->cycle.LastPlanId(), before.last_plan_id);
  }
  // Without a freeze window there is nothing to be before: the bound is not
  // asked, and the pair is solved for.
  PlannerParams params = FollowingSearchParams(0.5);
  params.t_freeze = std::numeric_limits<double>::quiet_NaN();
  ReplacingRig rig(params);
  rig->segment->first_lead_ns = 20'000 * kMs;
  const Calls calls = rig->Wake(kFollowWake);
  EXPECT_EQ(rig->segment->earliest_calls, 0);
  EXPECT_GE(IndexOf(calls, Call::kPlanFirst), 0);
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
}

TEST(PlannerCycleReplacement, APairWhoseCatchInstantWouldBeFrozenAtItsEarliestStartIsNotSolvedFor) {
  // The other question asked of the planner's bound before the first solve.
  // The RT switches to a held pair at its first segment's node 0 and drops it
  // there when the NEW catch instant is within T_freeze: a pair whose segment
  // cannot start more than T_freeze before its own catch instant would be held
  // by the RT until node 0 — nothing published meanwhile — and then dropped.
  // It is not solved for; the followed plan's segment is replanned. (The
  // followed plan's freeze is seconds away: that bound is far inside.)
  for (const std::int64_t margin_ns : {std::int64_t{1}, std::int64_t{0}, -kMs}) {
    SCOPED_TRACE(margin_ns);
    ReplacingRig rig;
    const std::int64_t asked_at = g_step_now + kMs;  // the wake's first clock read
    const std::int64_t earliest = asked_at + kFakeFirstLeadNs;
    ASSERT_LT(earliest, kFakeTc - kFreezeNs - 1000 * kMs);
    // At its earliest start the new plan has T_freeze + `margin_ns` left.
    rig->search->t_c_ns = earliest + kFreezeNs + margin_ns;
    const Stored before(*rig.rig);
    const Calls calls = rig->Wake(kFollowWake);
    EXPECT_EQ(rig->segment->earliest_calls, 1);
    EXPECT_EQ(rig->segment->earliest_now_ns, asked_at);
    if (margin_ns > 0) {
      EXPECT_EQ(calls, kReplacementPairCalls);
      EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
      EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kPublished);
      continue;
    }
    Calls expected{Call::kSearchPlan, Call::kClock};
    expected.insert(expected.end(), kReplanBehindSearchCalls.begin() + 1,
                    kReplanBehindSearchCalls.end());
    EXPECT_EQ(calls, expected);
    EXPECT_EQ(IndexOf(calls, Call::kPlanFirst), -1);
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    EXPECT_EQ(rig->rec.search.decision, SwitchDecision::kReplaced);
    EXPECT_TRUE(rig->rec.search_valid);
    EXPECT_EQ(rig->rec.replacement.outcome, SegmentOutcome::kOff) << "no solve ran";
    EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kTooLateNew);
    EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
    EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kAdvance);
    EXPECT_EQ(rig->boxes.plan.sequence(), before.plan_stores);
    EXPECT_EQ(rig->cycle.LastPlanId(), before.last_plan_id);
  }
}

TEST(PlannerCycleReplacement, AWithheldFirstSegmentKeepsThePlanItsAccountAndReplans) {
  ReplacingRig rig;
  rig->segment->first_ok = false;
  rig->segment->first_refusal = SegmentOutcome::kCatchError;

  struct Ctx {
    FakeRig* rig;
  } ctx{rig.rig.get()};

  rig->cycle.SetPostSearchHookForTesting(
      [](void* user) noexcept {
        PlannerRtState later = Following(Mode::kApproach, 1);
        later.rt_iteration += 1;
        static_cast<Ctx*>(user)->rig->boxes.rt.Store(later);
      },
      &ctx);
  const Stored before(*rig.rig);
  // The search, the bound, the first solve — withheld: no stamp is read for
  // it — and then the replan of the followed plan.
  Calls expected{Call::kSearchPlan, Call::kClock, Call::kPlanFirst};
  expected.insert(expected.end(), kReplanBehindSearchCalls.begin() + 1,
                  kReplanBehindSearchCalls.end());
  EXPECT_EQ(rig->Wake(kFollowWake), expected);
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
  EXPECT_TRUE(rig->rec.search_valid);
  EXPECT_FALSE(rig->rec.plan_valid);
  EXPECT_EQ(rig->rec.search.decision, SwitchDecision::kReplaced);
  // The withheld solve's account is kept beside the replan's.
  EXPECT_EQ(rig->rec.replacement.outcome, SegmentOutcome::kCatchError);
  EXPECT_EQ(rig->rec.replace_step, ReplaceStep::kWithheld);
  EXPECT_EQ(rig->rec.replacement.kind, SegmentKind::kFirst);
  EXPECT_EQ(rig->rec.replacement.source_seq, kFakeSourceSeq);
  EXPECT_TRUE(rig->rec.replacement.x0_from_segment);
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
  EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kAdvance);
  // No plan went out, no id was used; the one store is the replan's segment.
  EXPECT_EQ(rig->boxes.plan.sequence(), before.plan_stores);
  EXPECT_EQ(rig->cycle.LastPlanId(), before.last_plan_id);
  EXPECT_EQ(rig->boxes.plan.Load().plan_id, 1U);
  EXPECT_EQ(rig->segment_box.Load().plan_id, 1U);
  EXPECT_EQ(rig->segment_box.Load().segment_seq, before.last_segment_seq + 1);
  // The first solve ran on the wake's report, the replan on the one read
  // after the search.
  const std::uint64_t wake_iteration = Following(Mode::kApproach, 1).rt_iteration;
  EXPECT_EQ(rig->segment->first_rt_iteration, wake_iteration);
  EXPECT_EQ(rig->segment->replan_rt_iteration, wake_iteration + 1);
  // A wake that withholds neither: the account is empty again.
  rig->segment->first_ok = true;
  rig->cycle.SetPostSearchHookForTesting(nullptr, nullptr);
  rig->boxes.rt.Store(Following(Mode::kApproach, 1));
  static_cast<void>(rig->Wake(kFollowWake));
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
  EXPECT_EQ(rig->rec.replacement.outcome, SegmentOutcome::kOff);
}

TEST(PlannerCycleReplacement, AReplanIsDroppedAtItsRecheckWhenTheRtHoldsAReplacementByThen) {
  // The RT can take a pair late — after the wait behind the pair has run out
  // — so a wake that read "nothing held" can find a replacement held by the
  // time its replan is solved. That replan is of the plan the RT is about to
  // leave, and the RT takes nothing until it has switched or dropped the
  // pair: it is not stored. Behind a search, and on the replan-alone wake.
  struct Ctx {
    FakeRig* rig;
    bool held;
  };

  for (const std::int64_t wake_ns : {kFollowWake, kFakeTc - 300 * kMs}) {
    const bool searches = wake_ns == kFollowWake;
    for (const bool held : {true, false}) {
      SCOPED_TRACE(std::string(searches ? "behind a search" : "the replan alone") +
                   (held ? ", a replacement held by the re-check" : ", nothing held (control)"));
      ReplacingRig rig;
      rig->search->decision = SwitchDecision::kRefreshed;
      Ctx ctx{rig.rig.get(), held};
      rig->cycle.SetPostSegmentHookForTesting(
          [](void* user) noexcept {
            auto* x = static_cast<Ctx*>(user);
            PlannerRtState s = Following(Mode::kApproach, 1);
            s.rt_iteration += 1;
            s.plan_pending = x->held;
            s.plan_pending_id = 2;
            s.plan_pending_t_c_ns = kFakeTc + 40 * kMs;
            x->rig->boxes.rt.Store(s);
          },
          &ctx);
      const Stored before(*rig.rig);
      const Calls calls = rig->Wake(wake_ns);
      Calls expected = searches ? Calls{Call::kSearchPlan} : Calls{};
      const Calls solved{Call::kFollowedTrack, Call::kReplan, Call::kClock};
      expected.insert(expected.end(), solved.begin(), solved.end());
      EXPECT_EQ(rig->rec.outcome, searches ? CycleOutcome::kHeld : CycleOutcome::kIdle);
      EXPECT_EQ(rig->rec.segment.kind, SegmentKind::kAdvance);
      if (held) {
        // Dropped before either question of the re-check is asked of the
        // planner; nothing stored, nobody told.
        EXPECT_EQ(calls, expected);
        EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kSuperseded);
        EXPECT_EQ(Stored(*rig.rig), before);
      } else {
        const Calls stored{Call::kSourceSeq, Call::kStartsInTime, Call::kSegmentNotePublished};
        expected.insert(expected.end(), stored.begin(), stored.end());
        EXPECT_EQ(calls, expected);
        EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kPublished);
        EXPECT_EQ(rig->segment_box.Load().segment_seq, before.last_segment_seq + 1);
      }
    }
  }
}

TEST(PlannerCycleReplacement, WhileTheRtHoldsAReplacementAWakeRunsNothing) {
  // The RT took the pair and holds it until its first segment starts: it takes
  // nothing else until then, so neither a search nor a replan has anywhere to
  // go — before t_stop_plan and after it.
  for (const std::int64_t wake_ns : {kFollowWake, kFakeTc - 300 * kMs}) {
    SCOPED_TRACE(wake_ns - kFakeTc);
    ReplacingRig rig;
    PlannerRtState held = Following(Mode::kApproach, 1);
    held.plan_pending = true;
    held.plan_pending_id = 7;
    held.plan_pending_t_c_ns = kFakeTc + 40 * kMs;
    held.segment_pending = true;
    held.segment_pending_seq = 2;
    rig->boxes.rt.Store(held);
    const Stored before(*rig.rig);
    EXPECT_EQ(rig->Wake(wake_ns), (Calls{}));
    EXPECT_EQ(rig->rec.outcome, CycleOutcome::kHeld);
    EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kOff);
    EXPECT_FALSE(rig->rec.search_valid);
    EXPECT_EQ(Stored(*rig.rig), before);
    // The same report without the held replacement is an ordinary wake again.
    held.plan_pending = false;
    rig->boxes.rt.Store(held);
    const Calls calls = rig->Wake(wake_ns);
    EXPECT_FALSE(calls.empty());
    EXPECT_NE(Stored(*rig.rig), before);
  }
}

TEST(PlannerCycleReplacement, RightAfterAReplacementPairNothingIsStoredUntilTheRtCanShowIt) {
  // The segment box has one slot. Until the RT's report can show what it did
  // with the pair, a replan of the plan it still follows would land on top of
  // the replacement's first segment — and another replacement on top of this
  // one.
  ReplacingRig rig;
  ASSERT_EQ(rig->Wake(kFollowWake), kReplacementPairCalls);
  const std::int64_t stamp = rig->rec.publish_ns;
  PlannerRtState s = Following(Mode::kApproach, 1);  // still on the old plan, nothing held
  s.rt_state_ns = stamp + 3 * kFakeControlDtNs;
  rig->boxes.rt.Store(s);
  const Stored before(*rig.rig);
  EXPECT_EQ(rig->Wake(kFollowWake), (Calls{Call::kControlDtNs}));
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kIdle);
  EXPECT_EQ(rig->rec.segment.outcome, SegmentOutcome::kOff);
  EXPECT_EQ(Stored(*rig.rig), before);
  // The replan-alone wake (inside t_stop_plan) waits as well.
  EXPECT_EQ(rig->Wake(kFakeTc - 300 * kMs), (Calls{Call::kControlDtNs}));
  EXPECT_EQ(Stored(*rig.rig), before);
  // A report that already follows the pair's plan has shown it: its segment is
  // replanned at once, and nobody's period is asked.
  PlannerRtState switched = Following(Mode::kApproach, 2);
  switched.rt_state_ns = stamp + 1;
  rig->boxes.rt.Store(switched);
  rig->search->decision = SwitchDecision::kRefreshed;
  EXPECT_EQ(rig->Wake(kFollowWake), kReplanBehindSearchCalls);
  // One tick later than the three periods, still on the old plan with nothing
  // held: the RT did not take the pair, and the wake is an ordinary one.
  s.rt_state_ns = stamp + 3 * kFakeControlDtNs + 1;
  rig->boxes.rt.Store(s);
  const Calls calls = rig->Wake(kFollowWake);
  ASSERT_GE(calls.size(), 2U);
  EXPECT_EQ(calls[0], Call::kControlDtNs);
  EXPECT_EQ(calls[1], Call::kSearchPlan);
}

TEST(PlannerCycleReplacement, AfterTheRtDropsAPairTheNextWakeMayReplaceAgainUnderANewId) {
  ReplacingRig rig;
  ASSERT_EQ(rig->Wake(kFollowWake), kReplacementPairCalls);
  ASSERT_EQ(rig->rec.plan_id, 2U);
  const std::int64_t stamp = rig->rec.publish_ns;
  // The RT dropped the pair (its switch gate refused it, say): its report is
  // the old plan again with nothing held, and postdates the pair.
  PlannerRtState s = Following(Mode::kApproach, 1);
  s.rt_state_ns = stamp + 3 * kFakeControlDtNs + 1;
  rig->boxes.rt.Store(s);
  Calls expected{Call::kControlDtNs};
  expected.insert(expected.end(), kReplacementPairCalls.begin(), kReplacementPairCalls.end());
  EXPECT_EQ(rig->Wake(kFollowWake), expected);
  EXPECT_EQ(rig->rec.outcome, CycleOutcome::kPublished);
  EXPECT_EQ(rig->rec.plan_id, 3U);
  EXPECT_EQ(rig->rec.segment.segment_seq, 3U);
  EXPECT_EQ(rig->boxes.plan.Load().plan_id, 3U);
  EXPECT_EQ(rig->segment_box.Load().plan_id, 3U);
  EXPECT_EQ(rig->segment->first_plan_id, 3U);
}

TEST(PlannerCycleReplacement, TheFirstFollowedCatchInstantStillStopsTheSearchAfterAReplacement) {
  // t_stop_plan is measured back from the FIRST catch instant followed since
  // the reset. The RT switching to a replacement with a later catch instant
  // does not move it out.
  ReplacingRig rig;
  ASSERT_EQ(rig->Wake(kFollowWake), kReplacementPairCalls);
  PlannerRtState switched = Following(Mode::kApproach, 2);
  switched.plan_t_c_ns = kFakeTc + 1000 * kMs;
  rig->boxes.rt.Store(switched);
  // 0.3 s before the first followed instant: inside t_stop_plan (0.5 s),
  // although 1.3 s before the replacement's.
  const Calls calls = rig->Wake(kFakeTc - 300 * kMs);
  EXPECT_EQ(IndexOf(calls, Call::kSearchPlan), -1);
  EXPECT_GE(IndexOf(calls, Call::kReplan), 0);
}

TEST(PlannerCycleReplacement, TheSearchOfAFollowingWakeIsCappedByThePeriodLessTheReplanBudget) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto cap_of = [](double dt_expected_s, bool following) {
    PlannerParams params = FollowingSearchParams(0.5);
    params.dt_expected = dt_expected_s;
    ReplacingRig rig(params);
    if (!following) {
      // The RT follows nothing (it did not take the first pair) and its report
      // postdates that pair: an ordinary search wake.
      PlannerRtState s = RtIn(Mode::kTracking);
      s.rt_state_ns = rig.first_stamp + 3 * kFakeControlDtNs + 1;
      rig->boxes.rt.Store(s);
    }
    rig->search->plan_budget_cap_ns = -1;
    const Calls calls = rig->Wake(kFollowWake);
    EXPECT_GE(IndexOf(calls, Call::kSearchPlan), 0);
    // The replan budget is asked for on a following wake only.
    EXPECT_EQ(rig->segment->replan_budget_calls > 0,
              following && std::isfinite(dt_expected_s) && dt_expected_s > 0.0);
    return rig->search->plan_budget_cap_ns;
  };
  // One prediction period less the installed planner's replan budget (7 ms).
  EXPECT_EQ(cap_of(0.05, true), 50 * kMs - kFakeReplanBudgetNs);
  EXPECT_EQ(cap_of(0.02, true), 20 * kMs - kFakeReplanBudgetNs);
  EXPECT_EQ(cap_of(0.007 + 1e-9, true), 1);
  // Nothing left, or nothing known: no cap (0), never a negative one.
  EXPECT_EQ(cap_of(0.007, true), 0);
  EXPECT_EQ(cap_of(0.005, true), 0);
  EXPECT_EQ(cap_of(0.0, true), 0);
  EXPECT_EQ(cap_of(-0.05, true), 0);
  EXPECT_EQ(cap_of(nan, true), 0);
  EXPECT_EQ(cap_of(std::numeric_limits<double>::infinity(), true), 0);
  // A wake with no plan followed has no replan behind its search: no cap,
  // whatever the period.
  EXPECT_EQ(cap_of(0.05, false), 0);
  EXPECT_EQ(cap_of(nan, false), 0);
}

TEST(PlannerCycleReplacement, AReplacementWakeAllocatesNothing) {
  // The cycle's part of the three kinds of following wake a replacement adds:
  // the pair, the withheld pair with its replan, and the wake the RT holds a
  // replacement on. (What an implementation allocates is its own suite's.)
  ReplacingRig rig;
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  int pairs = 0;
  int withheld = 0;
  int held = 0;
  bool overflow = false;
  constexpr int kRounds = 30;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < kRounds; ++i) {
      // The RT follows the plan the cycle published last, its report well
      // after that pair.
      PlannerRtState s = Following(Mode::kApproach, rig->cycle.LastPlanId());
      s.rt_state_ns = g_step_now + 4 * kFakeControlDtNs;
      rig->boxes.rt.Store(s);
      g_calls.Clear();
      rig->segment->first_ok = true;
      PlannerCycleRecord rec = rig->cycle.Run(NowReal{kFollowWake});
      pairs += rec.outcome == CycleOutcome::kPublished && rec.plan_valid ? 1 : 0;
      s.plan_id = rig->cycle.LastPlanId();
      s.rt_state_ns = g_step_now + 4 * kFakeControlDtNs;
      rig->boxes.rt.Store(s);
      g_calls.Clear();
      rig->segment->first_ok = false;
      rec = rig->cycle.Run(NowReal{kFollowWake});
      withheld += rec.outcome == CycleOutcome::kHeld &&
                          rec.replacement.outcome != SegmentOutcome::kOff &&
                          rec.segment.outcome == SegmentOutcome::kPublished
                      ? 1
                      : 0;
      s.plan_pending = true;
      rig->boxes.rt.Store(s);
      g_calls.Clear();
      rec = rig->cycle.Run(NowReal{kFollowWake});
      held +=
          rec.outcome == CycleOutcome::kHeld && rec.segment.outcome == SegmentOutcome::kOff ? 1 : 0;
      overflow = overflow || g_calls.overflow;
    }
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_EQ(pairs, kRounds) << "the gated loop did not exercise the replacement pair";
  EXPECT_EQ(withheld, kRounds) << "the gated loop did not exercise the withheld pair";
  EXPECT_EQ(held, kRounds) << "the gated loop did not exercise the held wake";
  EXPECT_FALSE(overflow);
  EXPECT_EQ(heap, 0U);
  EXPECT_EQ(eigen, 0U);
}

}  // namespace
