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
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using rtc::catching::ActivityFor;
using rtc::catching::AdmittedPlan;
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
using rtc::catching::PlannerRtState;
using rtc::catching::PlanRefusal;
using rtc::catching::PlanSnapshot;
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
      {Mode::kDecel, PlannerActivity::kIdle},
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
  switch: {delta_J: 0.2, e_jump_max: 0.02, ed_jump_max: 0.1}
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
  EXPECT_DOUBLE_EQ(p.switch_e_jump_max, 0.02);
  EXPECT_DOUBLE_EQ(p.switch_ed_jump_max, 0.1);
  EXPECT_DOUBLE_EQ(p.t_freeze, 0.36);
  EXPECT_DOUBLE_EQ(p.score.w_sigma, 2.0);
  EXPECT_DOUBLE_EQ(p.score.penalty, 20.0);
  ASSERT_TRUE(p.catch_box.set);
  EXPECT_TRUE(p.catch_box.Contains(0.5, 0.0, 0.5));
  EXPECT_FALSE(p.catch_box.Contains(0.5, 0.31, 0.5));
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
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
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

}  // namespace
