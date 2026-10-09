// ── S6-A: the planner thread and the RT plan lane ───────────────────────────
//
// Two halves, because they fail for different reasons.
//
// THE THREAD (no controller). The wake source is an eventfd, and what G3-L
// asks of it is coalescing: any number of signals between two wakes is ONE
// wake, and that wake reads the newest snapshot. Also: a signal raised during
// the wake that sees a trial reset is kept for the next wake (it belongs to the
// new trial), and Join does not wait out the wake timeout.
//
// THE LANE (real UR5e+P1b model, CM's configure path). The RT loads the plan
// box every tick and adopts a plan only when JudgePlan admits it. The cases
// drive the refusals that matter for safety through the CONTROLLER, not just
// the pure function: a plan from the previous activation (the deactivate /
// Pause race of D-23), a plan published before an E-STOP reset (which moves
// no generation), and the oracle — which since S6-A reaches the law through
// the same box, so its regression suites now exercise the adoption path.

#include "catching_cloud_fixture.hpp"
#include "catching_planner_fixture.hpp"
#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/catching/planner_thread.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>

namespace {

using integrated_bringup::CatchingPlannerThread;
using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::SharedCatchFrameBuilder;
using integrated_bringup::testfx::TrackingYaml;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::CycleOutcome;
using rtc::catching::Mode;
using rtc::catching::PlannerCycle;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanRefusal;
using rtc::catching::PlanSnapshot;
using rtc::catching::TrajectorySnapshot;

using namespace std::chrono_literals;

/// Collects the text of every WARN-or-worse log line while alive, chained to the
/// previous handler so a local run still shows the controller's log. The
/// observable of "configure does not warn about X" — the WARN is only a log.
class WarnCapture {
 public:
  WarnCapture() {
    Lines().clear();
    Previous() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&WarnCapture::Handler);
  }

  ~WarnCapture() { rcutils_logging_set_output_handler(Previous()); }

  WarnCapture(const WarnCapture&) = delete;
  WarnCapture& operator=(const WarnCapture&) = delete;

  /// Whether a captured line contains `needle`.
  [[nodiscard]] static bool Contains(const std::string& needle) {
    const std::lock_guard<std::mutex> lock(Mutex());
    return std::any_of(Lines().begin(), Lines().end(), [&](const std::string& line) {
      return line.find(needle) != std::string::npos;
    });
  }

 private:
  static void Handler(const rcutils_log_location_t* location, int severity, const char* name,
                      rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    if (severity >= RCUTILS_LOG_SEVERITY_WARN) {
      va_list sizing;
      va_copy(sizing, *args);
      const int n = std::vsnprintf(nullptr, 0, format, sizing);
      va_end(sizing);
      if (n > 0) {
        std::string text(static_cast<std::size_t>(n) + 1, '\0');
        va_list copy;
        va_copy(copy, *args);
        std::vsnprintf(text.data(), text.size(), format, copy);
        va_end(copy);
        text.resize(static_cast<std::size_t>(n));
        const std::lock_guard<std::mutex> lock(Mutex());
        Lines().push_back(std::move(text));
      }
    }
    if (Previous() != nullptr) {
      va_list forward;
      va_copy(forward, *args);
      Previous()(location, severity, name, timestamp, format, &forward);
      va_end(forward);
    }
  }

  static std::vector<std::string>& Lines() {
    static std::vector<std::string> lines;
    return lines;
  }

  static std::mutex& Mutex() {
    static std::mutex m;
    return m;
  }

  static rcutils_logging_output_handler_t& Previous() {
    static rcutils_logging_output_handler_t prev = nullptr;
    return prev;
  }
};

class RclcppScope : public ::testing::Environment {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

const ::testing::Environment* const kRclcpp =
    ::testing::AddGlobalTestEnvironment(new RclcppScope);  // NOLINT

/// An unpinned, SCHED_OTHER config: the thread tests are about the wake
/// source, not placement, and must not depend on this host's RT permissions.
rtc::ThreadConfig PlainThread() {
  return rtc::ThreadConfig{-1, SCHED_OTHER, 0, 0, "plan_test"};
}

template <typename Pred>
bool WaitUntil(Pred pred, std::chrono::milliseconds limit = 2000ms) {
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

// ── The thread ──────────────────────────────────────────────────────────────

struct ThreadRig {
  rtc::SeqLock<TrajectorySnapshot> traj{};
  rtc::SeqLock<CovarianceSnapshot> cov{};
  rtc::SeqLock<PlannerRtState> rt{};
  rtc::SeqLock<PlanSnapshot> plan{};
  PlannerCycle cycle;
  CatchingPlannerThread::TimingBuffer timing{};
  CatchingPlannerThread::EventQueue events{};
  int fd{-1};

  ThreadRig() {
    fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    EXPECT_GE(fd, 0);
    EXPECT_TRUE(cycle.Bind({&traj, &cov, &rt, &plan}));
  }

  ~ThreadRig() {
    if (fd >= 0) {
      ::close(fd);
    }
  }

  void Tracking(std::uint32_t reset_epoch = 1) {
    PlannerRtState s{};
    s.valid = true;
    s.activation_generation = 3;
    s.reset_epoch = reset_epoch;
    s.mode = static_cast<std::uint8_t>(Mode::kTracking);
    rt.Store(s);
  }

  void Trajectory(std::uint64_t sequence) {
    TrajectorySnapshot t{};
    t.valid = true;
    t.n = 8;
    t.token.activation_generation = 3;
    t.token.generation = 42;
    t.token.snapshot_sequence = sequence;
    t.token.traj_recv_ns = 1;
    traj.Store(t);
  }
};

TEST(CatchingPlannerThread, ABurstOfSignalsIsOneWakeThatReadsTheNewestSnapshot) {
  auto rig = std::make_unique<ThreadRig>();
  rig->Tracking();
  // Five trajectories, each "published" with its own signal, all before the
  // thread gets to run: the counter holds 5, one read drains it.
  for (std::uint64_t seq = 1; seq <= 5; ++seq) {
    rig->Trajectory(seq);
    ASSERT_TRUE(CatchingPlannerThread::Signal(rig->fd));
  }
  // A long timeout, so every wake inside the observation window is a signal.
  CatchingPlannerThread thread(rig->cycle, rig->fd, /*wake_timeout_s=*/0.5, rig->timing,
                               rig->events);
  thread.StartWith(PlainThread());
  ASSERT_TRUE(WaitUntil([&] { return thread.PublishedCount() >= 1; }));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(thread.SignalWakeCount(), 1U) << "five signals were not coalesced into one wake";
  EXPECT_EQ(thread.LastRecord().snapshot_sequence, 5U) << "the wake did not read the newest";
  EXPECT_EQ(thread.PublishedCount(), 1U);
}

TEST(CatchingPlannerThread, EachSignalAfterAWakeIsItsOwnWakeAndNeverMoreThanTheSignals) {
  auto rig = std::make_unique<ThreadRig>();
  rig->Tracking();
  rig->Trajectory(1);
  CatchingPlannerThread thread(rig->cycle, rig->fd, 0.5, rig->timing, rig->events);
  thread.StartWith(PlainThread());
  constexpr int kSignals = 10;
  for (int i = 0; i < kSignals; ++i) {
    rig->Trajectory(static_cast<std::uint64_t>(i + 2));
    ASSERT_TRUE(CatchingPlannerThread::Signal(rig->fd));
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(WaitUntil([&] { return thread.LastRecord().snapshot_sequence == kSignals + 1; }));
  EXPECT_LE(thread.SignalWakeCount(), static_cast<std::uint64_t>(kSignals));
  EXPECT_GE(thread.SignalWakeCount(), 1U);
}

TEST(CatchingPlannerThread, TheTimeoutWakesThePlannerWithoutASignal) {
  auto rig = std::make_unique<ThreadRig>();
  rig->Tracking();
  rig->Trajectory(1);
  CatchingPlannerThread thread(rig->cycle, rig->fd, /*wake_timeout_s=*/0.01, rig->timing,
                               rig->events);
  thread.StartWith(PlainThread());
  ASSERT_TRUE(WaitUntil([&] { return thread.TimeoutWakeCount() >= 3; }));
  EXPECT_EQ(thread.SignalWakeCount(), 0U);
  // A timeout wake re-plans against the current RT state: it publishes.
  EXPECT_GE(thread.PublishedCount(), 3U);
}

struct ResetDuringSearch {
  int fd{-1};
  // Written on the planner thread, read on the test thread.
  std::atomic<bool> fired{false};
};

void SignalDuringSearch(void* raw) noexcept {
  auto* ctx = static_cast<ResetDuringSearch*>(raw);
  if (!ctx->fired.exchange(true)) {
    static_cast<void>(CatchingPlannerThread::Signal(ctx->fd));
  }
}

TEST(CatchingPlannerThread, ASignalRaisedDuringAResetWakeIsNotLost) {
  // L7 §4.8's "drain the wake signal on re-arm" is satisfied by the wait
  // itself: every signal raised BEFORE a wake is consumed by that wake's read.
  // A signal raised DURING the wake that first sees a reset belongs to the new
  // trial (a trajectory accepted after the reset), and dropping it would delay
  // that trial's first plan by a whole wake timeout — /code-review 2026-09-23
  // found an earlier version doing exactly that. It must cause the next wake.
  auto rig = std::make_unique<ThreadRig>();
  rig->Tracking(/*reset_epoch=*/7);
  rig->Trajectory(1);
  ResetDuringSearch ctx;
  ctx.fd = rig->fd;
  rig->cycle.SetPostSearchHookForTesting(&SignalDuringSearch, &ctx);
  CatchingPlannerThread thread(rig->cycle, rig->fd, 0.5, rig->timing, rig->events);
  ASSERT_TRUE(CatchingPlannerThread::Signal(rig->fd));  // the wake that sees the reset
  thread.StartWith(PlainThread());
  ASSERT_TRUE(WaitUntil([&] { return thread.ResetSeenCount() == 1; }));
  ASSERT_TRUE(ctx.fired.load());
  // Well inside the 0.5 s timeout: only the surviving signal can cause this.
  EXPECT_TRUE(WaitUntil([&] { return thread.SignalWakeCount() == 2; }, 200ms))
      << "the signal raised during the reset wake was dropped";
  EXPECT_EQ(thread.TimeoutWakeCount(), 0U);
}

TEST(CatchingPlannerThread, JoinDoesNotWaitOutTheTimeout) {
  auto rig = std::make_unique<ThreadRig>();
  auto thread =
      std::make_unique<CatchingPlannerThread>(rig->cycle, rig->fd, 0.5, rig->timing, rig->events);
  thread->StartWith(PlainThread());
  ASSERT_TRUE(WaitUntil([&] { return thread->Running(); }));
  std::this_thread::sleep_for(20ms);  // well inside a poll
  const auto t0 = std::chrono::steady_clock::now();
  thread.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 200ms);
}

TEST(CatchingPlannerThread, APausedThreadStopsPublishingAfterAtMostOneWake) {
  auto rig = std::make_unique<ThreadRig>();
  rig->Tracking();
  rig->Trajectory(1);
  CatchingPlannerThread thread(rig->cycle, rig->fd, 0.005, rig->timing, rig->events);
  thread.StartWith(PlainThread());
  ASSERT_TRUE(WaitUntil([&] { return thread.PublishedCount() >= 3; }));
  thread.Pause();
  // One wake may already be past the pause check (PeriodicRtThread::Pause does
  // not stop an iteration in flight — the reason D-23 exists).
  std::this_thread::sleep_for(30ms);
  const auto settled = thread.PublishedCount();
  std::this_thread::sleep_for(60ms);
  EXPECT_EQ(thread.PublishedCount(), settled) << "a paused planner kept publishing";
  thread.Resume();
  EXPECT_TRUE(WaitUntil([&] { return thread.PublishedCount() > settled; }));
}

TEST(PlannerEventsCsv, EveryRowHasTheHeadersWidthAndIdleWakesAreSkipped) {
  std::ostringstream header;
  integrated_bringup::WritePlannerEventsHeader(header);
  rtc::catching::PlannerCycleRecord rec{};
  rec.outcome = CycleOutcome::kPublished;
  rec.traj_recv_ns = 100;
  rec.publish_ns = 2'000'100;
  rec.search.chosen_rank_mask = rtc::catching::kRankReach | rtc::catching::kRankGamma;
  std::ostringstream row;
  integrated_bringup::WritePlannerEventsRow(row, rec);
  const auto columns = [](const std::string& s) { return std::count(s.begin(), s.end(), ',') + 1; };
  EXPECT_EQ(columns(header.str()), columns(row.str()));
  EXPECT_NE(row.str().find(",2,published,"), std::string::npos) << "latency 2 ms: " << row.str();
  EXPECT_TRUE(integrated_bringup::PlannerEventWorthRecording(rec));
  rtc::catching::PlannerCycleRecord idle{};
  EXPECT_FALSE(integrated_bringup::PlannerEventWorthRecording(idle));
  idle.reset_seen = true;
  EXPECT_TRUE(integrated_bringup::PlannerEventWorthRecording(idle));
}

TEST(PlannerEventsCsv, ASegmentStepEarnsARowOnlyWhenItDidSomething) {
  // MPC E1-F03: the segment columns are appended (readers select by name), and
  // a wake whose segment step only waited does not earn a row on its own.
  using rtc::catching::SegmentOutcome;
  std::ostringstream header;
  integrated_bringup::WritePlannerEventsHeader(header);
  EXPECT_NE(header.str().find(",max_catchable,segment_outcome,"), std::string::npos);
  // E1-F05: the search's own validity beside the published plan's, and the
  // E1-F08 record after the E1-F03 columns.
  EXPECT_NE(header.str().find(",plan_valid,search_valid,plan_reason,"), std::string::npos);
  EXPECT_NE(header.str().find(",segment_tau_ratio_max,segment_kind,"), std::string::npos);
  EXPECT_NE(header.str().find(",segment_catch_v_rel,segment_slack_v,"), std::string::npos);
  rtc::catching::PlannerCycleRecord rec{};
  for (const SegmentOutcome waited :
       {SegmentOutcome::kOff, SegmentOutcome::kUpToDate, SegmentOutcome::kPastReplanWindow}) {
    rec.segment.outcome = waited;
    EXPECT_FALSE(integrated_bringup::PlannerEventWorthRecording(rec))
        << rtc::catching::SegmentOutcomeName(waited);
  }
  for (const SegmentOutcome acted :
       {SegmentOutcome::kPublished, SegmentOutcome::kSlack, SegmentOutcome::kSolveFailed,
        SegmentOutcome::kBudget, SegmentOutcome::kSuperseded, SegmentOutcome::kNoState}) {
    rec.segment.outcome = acted;
    EXPECT_TRUE(integrated_bringup::PlannerEventWorthRecording(rec))
        << rtc::catching::SegmentOutcomeName(acted);
  }
  rec.segment.outcome = SegmentOutcome::kPublished;
  std::ostringstream row;
  integrated_bringup::WritePlannerEventsRow(row, rec);
  const auto columns = [](const std::string& s) { return std::count(s.begin(), s.end(), ',') + 1; };
  EXPECT_EQ(columns(header.str()), columns(row.str()));
  EXPECT_NE(row.str().find(",published,"), std::string::npos) << row.str();
  // A default record's kind is written by name, its uncomputed values as nan.
  // (The row goes on after `segment_source_seq` — E1-F18 appended columns; the
  // test below reads those by name.)
  EXPECT_NE(row.str().find(",none,0,0,0,0,nan,nan,nan,nan,nan,nan,nan,nan,nan,0,nan,0,"),
            std::string::npos)
      << row.str();
}

// E1-F18: the NLP search's block, the docking core's block and the replacement
// columns, read back by NAME — the way every reader selects them.
TEST(PlannerEventsCsv, TheNlpDockingAndReplacementColumnsCarryTheRecordByName) {
  const auto split = [](std::string s) {
    while (!s.empty() && s.back() == '\n') {
      s.pop_back();
    }
    std::vector<std::string> out;
    std::istringstream in(s);
    for (std::string cell; std::getline(in, cell, ',');) {
      out.push_back(cell);
    }
    return out;
  };
  std::ostringstream header;
  integrated_bringup::WritePlannerEventsHeader(header);
  const std::vector<std::string> names = split(header.str());
  EXPECT_EQ(names.back(), "replacement_solve_us");
  // No name twice: a reader that selects by name would take one of them.
  EXPECT_EQ(std::set<std::string>(names.begin(), names.end()).size(), names.size());
  const auto row_of = [&](const rtc::catching::PlannerCycleRecord& rec) {
    std::ostringstream row;
    integrated_bringup::WritePlannerEventsRow(row, rec);
    const std::vector<std::string> cells = split(row.str());
    EXPECT_EQ(cells.size(), names.size());
    std::map<std::string, std::string> out;
    for (std::size_t i = 0; i < std::min(cells.size(), names.size()); ++i) {
      out[names[i]] = cells[i];
    }
    return out;
  };

  // A record nothing filled: no NLP search ran, no docking solve, no replacement.
  rtc::catching::PlannerCycleRecord rec{};
  auto row = row_of(rec);
  EXPECT_EQ(row["nlp_ran"], "0");
  EXPECT_EQ(row["nlp_reason"], "off") << "not `none`: that is a wake that chose a plan";
  EXPECT_EQ(row["nlp_n_lattice"], "0");
  EXPECT_EQ(row["nlp_phi"], "nan");
  EXPECT_EQ(row["nlp_solve_us_max"], "nan");
  EXPECT_EQ(row["segment_qp_solves"], "0");
  EXPECT_EQ(row["segment_qp_us"], "nan");
  EXPECT_EQ(row["segment_viol_torque"], "nan");
  EXPECT_EQ(row["segment_viol_terminal"], "nan");
  EXPECT_EQ(row["segment_elastic_impact"], "nan");
  EXPECT_EQ(row["segment_infeasible_group"], "none");
  EXPECT_EQ(row["segment_chance_lateral"], "nan");
  EXPECT_EQ(row["replace_step"], "none");
  EXPECT_EQ(row["replacement_outcome"], "off");
  EXPECT_EQ(row["replacement_core_reason"], "none");

  // An NLP wake that chose nothing: the reason and the counts are written, the
  // chosen candidate's fields are not numbers.
  rec.search.nlp.ran = true;
  rec.search.nlp.reason = rtc::catching::NlpReject::kSpeedWindow;
  rec.search.nlp.n_lattice = 18;
  rec.search.nlp.n_screened = 3;
  rec.search.nlp.rejects[static_cast<std::size_t>(rtc::catching::NlpReject::kSpeedWindow)] = 15;
  rec.search.nlp.rejects[static_cast<std::size_t>(rtc::catching::NlpReject::kUnconverged)] = 2;
  rec.search.nlp.chosen_phi = 7.0;  // stale: not a chosen candidate's
  rec.search.nlp.screen_ns = 1'500'000;
  rec.search.nlp.solve_ns_max = 12'000'000;
  row = row_of(rec);
  EXPECT_EQ(row["nlp_ran"], "1");
  EXPECT_EQ(row["nlp_reason"], "speed_window");
  EXPECT_EQ(row["nlp_n_lattice"], "18");
  EXPECT_EQ(row["nlp_n_screened"], "3");
  EXPECT_EQ(row["nlp_rej_speed_window"], "15");
  EXPECT_EQ(row["nlp_rej_unconverged"], "2");
  EXPECT_EQ(row["nlp_rej_follow_window"], "0");
  EXPECT_EQ(row["nlp_phi"], "nan");
  EXPECT_EQ(row["nlp_screen_us"], "1500");
  EXPECT_EQ(row["nlp_solve_us_max"], "12000");
  // … and one that chose a plan.
  rec.search.nlp.reason = rtc::catching::NlpReject::kNone;
  rec.search.nlp.chosen_phi = 7.0;
  rec.search.nlp.chosen_j_reference = 5.5;
  rec.search.nlp.chosen_n_pre = 3;
  rec.search.nlp.chosen_lead_s = 0.25;
  // Integers are written exactly, not through the six digits of a double.
  rec.search.nlp.chosen_index = 3305956301;
  rec.search.nlp.chosen_delta_ns = -3456789;
  rec.search.nlp.screen_ns = 1'234'567'000;
  row = row_of(rec);
  EXPECT_EQ(row["nlp_index"], "3305956301");
  EXPECT_EQ(row["nlp_delta_ns"], "-3456789");
  EXPECT_EQ(row["nlp_screen_us"], "1234567");
  EXPECT_EQ(row["nlp_reason"], "none");
  EXPECT_EQ(row["nlp_phi"], "7");
  EXPECT_EQ(row["nlp_j_reference"], "5.5");
  EXPECT_EQ(row["nlp_n_pre"], "3");
  EXPECT_EQ(row["nlp_lead_s"], "0.25");
  EXPECT_EQ(row["nlp_cells_from_anchor"], "nan") << "the RT follows no plan: no anchor";
  rec.search.nlp.follow_anchor_set = true;
  rec.search.nlp.follow_anchor_index = 3305956299;
  rec.search.nlp.chosen_cells_from_anchor = 2;
  rec.search.nlp.chosen_ns_from_first = 123456789;
  row = row_of(rec);
  EXPECT_EQ(row["nlp_follow_anchor_index"], "3305956299");
  EXPECT_EQ(row["nlp_cells_from_anchor"], "2");
  EXPECT_EQ(row["nlp_ns_from_first"], "123456789");

  // A docking solve that ended infeasible, and a replacement that was withheld.
  auto& d = rec.segment.docking;
  d.ran = true;
  d.qp_solves = 20;
  d.qp_iterations = 400;
  d.qp_us = 9500.0;
  d.kkt_residual = 0.5;
  d.infeasible_group_name = "lateral";
  d.violation[3] = 0.02;  // lateral
  d.violation[8] = 0.25;  // terminal: the last group
  d.elastic[6] = 0.125;   // impact: the last elastic group
  d.c_catch = 0.75;
  d.c_guarded = true;
  d.sigma_s = 0.03;
  d.sigma_t = 0.04;
  d.lateral_margin = -0.02;
  d.timing_margin = std::numeric_limits<double>::quiet_NaN();
  rec.replace_step = rtc::catching::ReplaceStep::kWithheld;
  rec.replacement.outcome = rtc::catching::SegmentOutcome::kBudget;
  rec.replacement.core_reason_name = "deadline";
  rec.replacement.iterations = 9;
  rec.replacement.solve_ns = 35'050'000;
  row = row_of(rec);
  EXPECT_EQ(row["segment_qp_solves"], "20");
  EXPECT_EQ(row["segment_qp_iterations"], "400");
  EXPECT_EQ(row["segment_qp_us"], "9500");
  EXPECT_EQ(row["segment_kkt_residual"], "0.5");
  EXPECT_EQ(row["segment_infeasible_group"], "lateral");
  EXPECT_EQ(row["segment_viol_lateral"], "0.02");
  EXPECT_EQ(row["segment_viol_torque"], "0");
  EXPECT_EQ(row["segment_viol_terminal"], "0.25");
  EXPECT_EQ(row["segment_elastic_impact"], "0.125");
  EXPECT_EQ(row["segment_c_catch"], "0.75");
  EXPECT_EQ(row["segment_c_guarded"], "1");
  EXPECT_EQ(row["segment_sigma_s"], "0.03");
  EXPECT_EQ(row["segment_sigma_t"], "0.04");
  EXPECT_EQ(row["segment_chance_lateral"], "-0.02");
  EXPECT_EQ(row["segment_chance_timing"], "nan") << "a row the problem lacks";
  EXPECT_EQ(row["replace_step"], "withheld");
  EXPECT_EQ(row["replacement_outcome"], "budget");
  EXPECT_EQ(row["replacement_core_reason"], "deadline");
  EXPECT_EQ(row["replacement_iterations"], "9");
  EXPECT_EQ(row["replacement_solve_us"], "35050");

  // The header names the row groups in the docking core's order: the record's
  // arrays are indexed by it.
  const auto at = [&](const std::string& name) {
    return std::find(names.begin(), names.end(), name) - names.begin();
  };
  for (int g = 0; g < rtc::catching::kNumDockingRowGroups; ++g) {
    const auto group = static_cast<rtc::catching::DockingRowGroup>(g);
    const std::string name = rtc::catching::DockingRowGroupName(group);
    EXPECT_EQ(at("segment_viol_" + name), at("segment_viol_torque") + g) << name;
    if (g < rtc::catching::kNumDockingElasticGroups) {
      EXPECT_EQ(at("segment_elastic_" + name), at("segment_elastic_torque") + g) << name;
    }
  }
  // … and the nlp_rej_* columns by NlpRejectName.
  for (std::size_t i = 0; i < integrated_bringup::kNlpRejectColumns.size(); ++i) {
    const std::string name = rtc::catching::NlpRejectName(integrated_bringup::kNlpRejectColumns[i]);
    EXPECT_EQ(at("nlp_rej_" + name), at("nlp_rej_follow_window") + static_cast<long>(i)) << name;
  }
}

// ── The lane ────────────────────────────────────────────────────────────────

class CatchingPlanLaneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_plan_lane_" +
                                                              std::to_string(++counter_));
    builder_ = SharedCatchFrameBuilder();
    topic_ = "/test_catching_plan_lane/prediction_" + std::to_string(counter_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->remove_node(node_->get_node_base_interface());
    }
    pub_.reset();
    executor_.reset();
    ctrl_.reset();
    node_.reset();
  }

  /// CM's configure path on the tracking suite's profile. `oracle` false
  /// leaves the box without a writer, so a case can play the planner.
  void BringUp(bool oracle, bool planner = false,
               const std::function<void(YAML::Node&)>& tweak = nullptr) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(sim_axis_ ? SimAxisConfigs()
                                          : integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    YAML::Node yaml = YAML::Load(
        TrackingYaml(topic_, Eigen::Vector3d(0.5, 0.2, 0.4), Eigen::Vector3d::UnitZ(), 0.0, 1.0));
    yaml["diagnostic"]["oracle_plan"]["enabled"] = oracle;
    yaml["catching"]["planner"]["enabled"] = planner;
    if (planner) {
      // The shipped ur5e_p1b planner values (S6-B). The fixture's model config
      // mirrors the shipped robot config, `ur5e_catch` included (R-3).
      YAML::Node pl = yaml["catching"]["planner"];
      pl["sub_model"] = "ur5e_catch";
      // NOT the shipped wait pose: from S7 the supervisor HOMES to it, and
      // this fixture holds the measured arm at kUr5eHome without a servo, so
      // the homing could never arrive. TrackingYaml's wait pose is that home
      // (the arm waits where it is), which is also the pose the reachable-
      // ball cases below build their ball from.
      pl["freeze"]["T_freeze"] = 0.36;
      pl["search"]["grid"]["hand"]["d_eff"] = 0.2815;
      pl["search"]["grid"]["hand"]["r_cap"] = 0.024;
      pl["wake_timeout_s"] = 0.02;
      pl["search"]["grid"]["n_settle"] = 1;
      // Cleared like every other provisional flag in TrackingYaml: this
      // fixture is judged on the REAL-ARM axis (its devices declare no
      // backend), where a provisional planner block parks the controller.
      pl["provisional"] = false;
    }
    if (tweak) {
      tweak(yaml);
    }
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_FALSE(ctrl_->IsSimOnlyDisabled());
    ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
    if (!executor_) {
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_->get_node_base_interface());
      rclcpp::QoS qos{rclcpp::KeepLast(1)};
      qos.best_effort();
      pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);
    }
    state_ = MakeState();
  }

  /// The fixture's devices as the sim runs them (backend mujoco_native): the
  /// SIM axis, the only one a docking function runs on (#654 parks it on a
  /// real arm).
  static std::map<std::string, rtc::DeviceNameConfig> SimAxisConfigs() {
    auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
    for (auto& entry : configs) {
      rtc::DeviceBackendBinding backend;
      backend.type = integrated_bringup::kCatchingSimBackendType;
      entry.second.backend = backend;
    }
    return configs;
  }

  static ControllerState MakeState() {
    ControllerState state{};
    state.num_devices = 2;
    state.dt = kDt;
    auto& a = state.devices[0];
    a.num_channels = kUr5eArmDof;
    a.valid = true;
    for (int i = 0; i < kUr5eArmDof; ++i) {
      a.positions[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    auto& h = state.devices[1];
    h.num_channels = kP1bHandDof;
    h.valid = true;
    return state;
  }

  void Publish(std::uint64_t sequence, std::uint64_t generation = 42) {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = static_cast<std::uint32_t>(cloud_n_);
    spec.sequence = sequence;
    spec.generation = generation;
    if (ball_set_) {
      spec.p0 = ball_p0_;
      spec.vel = ball_vel_;
      if (ball_cov_) {
        spec.cov_diag = *ball_cov_;
      }
      if (ball_advances_) {
        // A ball that is flying: every message starts where the previous
        // one's ball has got to, instead of restarting at p0.
        const auto steady = std::chrono::steady_clock::now();
        if (!ball_first_pub_) {
          ball_first_pub_ = steady;
        }
        const double elapsed = std::chrono::duration<double>(steady - *ball_first_pub_).count();
        for (std::size_t i = 0; i < 3; ++i) {
          spec.p0[i] += ball_vel_[i] * elapsed;
        }
      }
    }
    auto msg = integrated_bringup::testing::MakeCloud(spec);
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const std::int64_t wall =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - 5'000'000;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
    for (int i = 0; i < 8; ++i) {
      executor_->spin_some(2ms);
    }
  }

  ControllerOutput Tick() {
    state_.iteration += 1;
    state_.t_relative_s = static_cast<double>(state_.iteration) * kDt;
    ControllerOutput out = ctrl_->Compute(state_);
    if (servo_) {
      // A perfect servo on both devices: measured q of the next tick = this
      // tick's command. Off by default — the cases above hold the arm still.
      for (int d = 0; d < 2; ++d) {
        const auto& o = out.devices[static_cast<std::size_t>(d)];
        auto& dev = state_.devices[static_cast<std::size_t>(d)];
        for (int i = 0; i < o.num_channels; ++i) {
          dev.positions[static_cast<std::size_t>(i)] = o.commands[static_cast<std::size_t>(i)];
        }
      }
    }
    return out;
  }

  /// Configure a fresh controller on the planner profile plus `tweak`, with
  /// `configs` as its devices, and report what configure decided.
  struct ConfigureVerdict {
    DemoCatchingController::CallbackReturn ret{DemoCatchingController::CallbackReturn::ERROR};
    bool parked{false};
    integrated_bringup::CatchingParkReason reason{integrated_bringup::CatchingParkReason::kNone};
  };

  ConfigureVerdict ConfigureOnly(bool planner, const std::function<void(YAML::Node&)>& tweak,
                                 std::map<std::string, rtc::DeviceNameConfig> configs =
                                     integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs()) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(configs);
    YAML::Node yaml = ConfigureOnlyYaml(planner);
    if (tweak) {
      tweak(yaml);
    }
    const rclcpp_lifecycle::State prev;
    ConfigureVerdict v;
    v.ret = ctrl_->on_configure(prev, node_, yaml);
    v.parked = ctrl_->IsSimOnlyDisabled();
    v.reason = ctrl_->GetParkReason();
    return v;
  }

  /// The shipped APPROACH–stop grid (MPC MD-54): 7 x 0.05 s after the catch,
  /// up to 6 x 0.1 s before it. `mode: mpc` needs the pre-catch part (MD-45):
  /// the RT takes a plan only with a segment that starts before t_c.
  static void ApproachGrid(YAML::Node& y) {
    YAML::Node d = y["catching"]["planner"]["segment"]["mpc"];
    d["horizon"]["n_nodes"] = 7;
    d["horizon"]["dt_s"] = 0.05;
    d["horizon"]["blocks"] = std::vector<int>{1, 1, 2, 3};
    d["replan"]["k_max"] = 2;
    d["approach"]["n_pre_max"] = 6;
  }

  /// ConfigureOnly's profile: mode mpc, the segment MPC (with its approach grid)
  /// as `planner` says.
  YAML::Node ConfigureOnlyYaml(bool planner) const {
    YAML::Node yaml = YAML::Load(
        TrackingYaml(topic_, Eigen::Vector3d(0.5, 0.2, 0.4), Eigen::Vector3d::UnitZ(), 0.0, 1.0));
    yaml["diagnostic"]["oracle_plan"]["enabled"] = !planner;
    YAML::Node pl = yaml["catching"]["planner"];
    pl["enabled"] = planner;
    pl["sub_model"] = "ur5e_catch";
    pl["freeze"]["T_freeze"] = 0.36;
    pl["search"]["grid"]["hand"]["d_eff"] = 0.2815;
    pl["search"]["grid"]["hand"]["r_cap"] = 0.024;
    pl["provisional"] = false;
    if (planner) {
      ApproachGrid(yaml);
    }
    yaml["catching"]["planner"]["segment"]["mode"] = "mpc";
    return yaml;
  }

  /// The real-clock closed loop of the planner in the loop, with the segment
  /// planner `docking` selects (the two TEST_Fs below).
  void RealClockPairCase(bool docking);

  /// Tick with a fresh prediction until the supervisor reaches `mode` (or the
  /// tick budget runs out). Returns whether it did.
  bool TickUntil(Mode mode, int budget = 200) {
    for (int t = 0; t < budget; ++t) {
      if (t % 10 == 0) {
        Publish(next_seq_++);
      }
      static_cast<void>(Tick());
      if (ctrl_->GetMode() == mode) {
        return true;
      }
    }
    return false;
  }

  /// A plan the RT would admit right now, for the track the fixture publishes.
  PlanSnapshot AdmissiblePlan(std::uint32_t id) const {
    const PlannerRtState rt = ctrl_->GetPlannerRtState();
    PlanSnapshot p{};
    p.valid = true;
    p.token.activation_generation = rt.activation_generation;
    p.token.generation = 42;
    p.plan_id = id;
    p.p_c = {0.5, 0.2, 0.4};
    p.a_d = {0.0, 0.0, 1.0};
    p.publish_ns = rtc::SteadyNowNs();
    p.gamma_t0_ns = p.publish_ns;
    p.t_c_ns = p.publish_ns + 1'000'000'000;
    p.gamma_t1_ns = p.t_c_ns;
    return p;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string topic_;
  ControllerState state_{};
  std::uint64_t next_seq_{1};
  /// Tick() servos both devices to their commands (MPC E1-F04's real-clock
  /// case needs the arm to reach DECEL; every other case holds it still).
  bool servo_{false};
  /// The ball the published prediction carries. Default: the decode suite's
  /// diagonal, nowhere near the arm — there is no catch point on it.
  bool ball_set_{false};
  std::array<double, 3> ball_p0_{};
  std::array<double, 3> ball_vel_{};
  int cloud_n_{8};
  /// BringUp's devices run on the sim axis (SimAxisConfigs).
  bool sim_axis_{false};
  /// The prediction's covariance diagonal; unset keeps the cloud fixture's
  /// (5 mm)² — wider than the docking hand's lateral capture set.
  std::optional<std::array<double, 6>> ball_cov_;
  /// The ball moves between messages (p0 is its position at the first one);
  /// off, every message restarts the ball at p0 (the cases that only need a
  /// catchable prediction).
  bool ball_advances_{false};
  std::optional<std::chrono::steady_clock::time_point> ball_first_pub_;
  static inline int counter_ = 0;
};

TEST_F(CatchingPlanLaneTest, TheOracleReachesTheLawThroughThePlanBox) {
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/true));
  ASSERT_TRUE(TickUntil(Mode::kApproach)) << "the oracle plan was never adopted";
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 1U);
  const PlanSnapshot followed = ctrl_->GetFollowedPlanForTesting();
  EXPECT_TRUE(followed.valid);
  EXPECT_EQ(followed.plan_id, ctrl_->GetPublishedPlan().plan_id)
      << "the followed plan is not the one the box delivered";
  // Once following, the oracle stops writing and the box holds the adopted
  // plan: every later tick judges it a repeat, never a second adoption.
  static_cast<void>(Tick());
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kRepeat);
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 1U);
}

TEST_F(CatchingPlanLaneTest, ThePlannerStateIsStoredEveryTickFromThatTick) {
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/true));
  for (int t = 0; t < 5; ++t) {
    static_cast<void>(Tick());
    const PlannerRtState s = ctrl_->GetPlannerRtState();
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.rt_iteration, state_.iteration) << "not stored on tick " << t;
    EXPECT_EQ(s.mode, static_cast<std::uint8_t>(ctrl_->GetMode()));
    EXPECT_EQ(s.nv, kUr5eArmDof);
  }
  ASSERT_TRUE(TickUntil(Mode::kApproach));
  // A law tick: the reference block and the command are live.
  static_cast<void>(Tick());
  const PlannerRtState s = ctrl_->GetPlannerRtState();
  EXPECT_TRUE(s.plan_active);
  EXPECT_EQ(s.plan_id, ctrl_->GetFollowedPlanForTesting().plan_id);
  EXPECT_TRUE(s.cmd_seeded);
  EXPECT_TRUE(s.ref_valid);
  EXPECT_TRUE(s.track_seen);
  EXPECT_EQ(s.track_generation, 42U);
}

TEST_F(CatchingPlanLaneTest, APlanFromThePreviousActivationIsNeverConsumed) {
  // G3-L / D-23: the planner's Pause does not stop a wake in flight, so a plan
  // can land after on_deactivate. It must not be followed after re-activation.
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false));
  static_cast<void>(Tick());
  const PlanSnapshot stale = AdmissiblePlan(1);  // stamped with THIS activation
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ctrl_->PlanBoxForTesting().Store(stale);  // "published after deactivate"
  ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));

  ASSERT_TRUE(TickUntil(Mode::kTracking));
  for (int t = 0; t < 20; ++t) {
    static_cast<void>(Tick());
    ASSERT_NE(ctrl_->GetMode(), Mode::kApproach) << "a previous-activation plan was followed";
  }
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kActivation);
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 0U);
  EXPECT_EQ(ctrl_->GetLastReason(), rtc::catching::Reason::kNoCatchablePlan);

  // Positive control: the same plan stamped with the current activation IS
  // taken — so the refusal above was the generation, not something else.
  ctrl_->PlanBoxForTesting().Store(AdmissiblePlan(2));
  static_cast<void>(Tick());
  EXPECT_EQ(ctrl_->GetMode(), Mode::kApproach);
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 1U);
}

TEST_F(CatchingPlanLaneTest, APlanPublishedBeforeAnEstopResetIsNotTakenAfterIt) {
  // JudgePlan (f). An E-STOP resets the trial without moving the activation
  // generation, so (b) cannot catch this plan — only the reset floor can.
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false));
  ASSERT_TRUE(TickUntil(Mode::kArmed));
  ctrl_->PlanBoxForTesting().Store(AdmissiblePlan(1));  // ARMED: judged, not adopted
  std::this_thread::sleep_for(2ms);
  ctrl_->TriggerEstop();
  static_cast<void>(Tick());
  ctrl_->ClearEstop();
  static_cast<void>(Tick());
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  ASSERT_TRUE(TickUntil(Mode::kTracking, 60)) << "precondition: re-armed inside the age bound";
  static_cast<void>(Tick());
  EXPECT_NE(ctrl_->GetMode(), Mode::kApproach);
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kBeforeReset);
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 0U);
}

TEST_F(CatchingPlanLaneTest, WithNoCatchablePointThePlannerKeepsTrackingOnNoCatchablePlan) {
  // S6-A wrote this case against the stub search. Since S6-B the search is
  // real, and the default prediction (the decode suite's diagonal) carries no
  // catch point: every candidate is outside the catch box or the horizon. The
  // outcome the RT sees is the same as the stub's — "no plan", refused as
  // invalid, TRACKING self-looping on NO_CATCHABLE_PLAN — now for a reason.
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false, /*planner=*/true));
  ASSERT_TRUE(ctrl_->IsGridCatchSearchConfigured()) << "the real model gave the search no model";
  const CatchingPlannerThread* thread = ctrl_->GetPlannerThread();
  ASSERT_NE(thread, nullptr);
  EXPECT_TRUE(thread->Running());
  EXPECT_FALSE(thread->Paused());
  ASSERT_TRUE(TickUntil(Mode::kTracking));
  // Keep ticking while vision publishes, so the planner wakes on real signals.
  for (int t = 0; t < 100; ++t) {
    if (t % 10 == 0) {
      Publish(next_seq_++);
    }
    static_cast<void>(Tick());
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_GT(thread->SignalWakeCount(), 0U) << "the subscription never woke the planner";
  EXPECT_GT(thread->PublishedCount(), 0U);
  const PlanSnapshot published = ctrl_->GetPublishedPlan();
  EXPECT_FALSE(published.valid);
  EXPECT_EQ(published.token.generation, 42U);
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kInvalid);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kTracking);
  EXPECT_EQ(ctrl_->GetLastReason(), rtc::catching::Reason::kNoCatchablePlan);
  // §13 S6: the tick record (hence the state message) says WHY — the
  // planner's first bottleneck — with the planner's id, and no plan.
  const auto record = ctrl_->GetLastTickRecord();
  EXPECT_FALSE(record.plan_valid);
  EXPECT_GT(record.plan_id, 0U);
  EXPECT_NE(record.plan_reason, static_cast<std::uint8_t>(rtc::catching::PlanReason::kNone));

  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(thread->Paused());
}

TEST_F(CatchingPlanLaneTest, ThePlannerFindsAReachableCatchPointAndTheRtFollowsIt) {
  // End to end on the real UR5e+P1b model (S6-B): a ball whose prediction
  // passes through a point the catch frame can reach, facing the palm. The
  // planner's search must publish a valid plan for it and the RT must adopt
  // it (TRACKING → APPROACH) through the same admission path the oracle uses.
  const pinocchio::SE3 pose = [&] {
    CatchFrameOracle oracle(*builder_);
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    return oracle.PoseAt(
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names, home,
        kUr5eArmDof);
  }();
  // Ball moving INTO the palm (against the catch frame's +z) at 1.5 m/s,
  // passing 3 cm in front of the current catch point 0.5 s after the stamp.
  const Eigen::Vector3d z = pose.rotation().col(2);
  const Eigen::Vector3d target = pose.translation() + 0.03 * z;
  const Eigen::Vector3d vel = -1.5 * z;
  const Eigen::Vector3d p0 = target - 0.5 * vel;
  ball_set_ = true;
  ball_p0_ = {p0.x(), p0.y(), p0.z()};
  ball_vel_ = {vel.x(), vel.y(), vel.z()};
  cloud_n_ = 20;  // 0.95 s of prediction: the slice window is [T_freeze, 0.95]

  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false, /*planner=*/true));
  ASSERT_TRUE(ctrl_->IsGridCatchSearchConfigured());
  bool approached = false;
  for (int t = 0; t < 600 && !approached; ++t) {
    if (t % 10 == 0) {
      Publish(next_seq_++);
    }
    static_cast<void>(Tick());
    std::this_thread::sleep_for(1ms);
    approached = ctrl_->GetMode() == Mode::kApproach;
  }
  const auto record = ctrl_->GetPlannerThread()->LastRecord();
  ASSERT_TRUE(approached) << "no plan was adopted; last planner record: outcome "
                          << rtc::catching::CycleOutcomeName(record.outcome) << ", reason "
                          << static_cast<int>(record.reason) << ", in window "
                          << record.search.n_in_window << ", IK " << record.search.n_ik
                          << ", passed " << record.search.n_pass;
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 1U);
  const PlanSnapshot followed = ctrl_->GetFollowedPlanForTesting();
  EXPECT_TRUE(followed.valid);
  EXPECT_EQ(followed.token.generation, 42U);
  // The chosen catch point is ON the predicted line (a vision sample), inside
  // the catch box, with the approach axis against the ball.
  const Eigen::Vector3d p_c(followed.p_c[0], followed.p_c[1], followed.p_c[2]);
  const Eigen::Vector3d off = p_c - p0;
  EXPECT_LT((off - off.dot(vel.normalized()) * vel.normalized()).norm(), 1e-9);
  const Eigen::Vector3d a_d(followed.a_d[0], followed.a_d[1], followed.a_d[2]);
  EXPECT_NEAR(a_d.dot(z), 1.0, 1e-9);
}

// ── The segment MPC on the planner thread (MPC E1-F03, #629) ──────────────────

TEST_F(CatchingPlanLaneTest, TheMpcSegmentPlannerIsParkedWithoutThePlanner) {
  ctrl_ = std::make_unique<DemoCatchingController>("");
  ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
  ctrl_->SetSharedModelBuilder(builder_);
  ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
  YAML::Node yaml = YAML::Load(
      TrackingYaml(topic_, Eigen::Vector3d(0.5, 0.2, 0.4), Eigen::Vector3d::UnitZ(), 0.0, 1.0));
  yaml["catching"]["planner"]["enabled"] = false;
  yaml["diagnostic"]["oracle_plan"]["enabled"] = false;  // no other writer of the segment box
  // MD-44: the mpc segment planner's keys are read only under the law that follows them.
  yaml["catching"]["planner"]["segment"]["mode"] = "mpc";
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
  EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kSegmentModeUnmet);
  EXPECT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
}

TEST_F(CatchingPlanLaneTest, TheMpcSegmentTorqueBoxAndSlackMustFitTheCliksTorqueBox) {
  // MD-33: η'_τ + slack_max ≤ joint_cmd.eta_tau under the dynamic CLIK form.
  ctrl_ = std::make_unique<DemoCatchingController>("");
  ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
  ctrl_->SetSharedModelBuilder(builder_);
  ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
  YAML::Node yaml = YAML::Load(
      TrackingYaml(topic_, Eigen::Vector3d(0.5, 0.2, 0.4), Eigen::Vector3d::UnitZ(), 0.0, 1.0));
  yaml["diagnostic"]["oracle_plan"]["enabled"] = false;  // one writer for the plan box
  YAML::Node pl = yaml["catching"]["planner"];
  pl["enabled"] = true;
  pl["segment"]["mpc"]["eta_tau"] = 0.75;
  pl["segment"]["mpc"]["publish"]["slack_max"] = 0.1;
  yaml["catching"]["joint_cmd"]["accel_constraint"] = "dynamic";
  yaml["catching"]["joint_cmd"]["eta_tau"] = 0.8;
  yaml["catching"]["planner"]["segment"]["mode"] = "mpc";  // MD-44, as above
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
  EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kMpcSegmentInvalid);
}

TEST_F(CatchingPlanLaneTest, AFittingMpcSegmentTorqueBoxConfiguresUnderTheDynamicClik) {
  // The passing side of MD-33's configure check: 0.7 + 0.1 ≤ 0.8.
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false, /*planner=*/true, [](YAML::Node& y) {
    ApproachGrid(y);                                      // MD-45
    y["catching"]["planner"]["segment"]["mode"] = "mpc";  // MD-44
    y["catching"]["joint_cmd"]["accel_constraint"] = "dynamic";
    y["catching"]["joint_cmd"]["eta_tau"] = 0.8;
  }));
  EXPECT_TRUE(ctrl_->IsSegmentPlannerConfigured());
}

// ── planner.segment.mode (MPC E1-F04, MD-34 · MD-42 · MD-44) ────────────────

TEST_F(CatchingPlanLaneTest, EachMissingMpcPrerequisiteParksTheController) {
  using integrated_bringup::CatchingParkReason;
  using Return = DemoCatchingController::CallbackReturn;
  const auto expect_park = [this](const char* what, bool planner,
                                  const std::function<void(YAML::Node&)>& tweak,
                                  std::map<std::string, rtc::DeviceNameConfig> configs =
                                      integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs()) {
    const ConfigureVerdict v = ConfigureOnly(planner, tweak, std::move(configs));
    EXPECT_EQ(v.ret, Return::SUCCESS) << what << ": a profile mistake parks, it does not fail";
    EXPECT_TRUE(v.parked) << what;
    EXPECT_EQ(v.reason, CatchingParkReason::kSegmentModeUnmet) << what;
    const rclcpp_lifecycle::State prev;
    EXPECT_EQ(ctrl_->on_activate(prev), Return::FAILURE) << what;
  };
  // The complete profile configures, on both writers of the plan box.
  for (const bool planner : {true, false}) {
    const ConfigureVerdict ok = ConfigureOnly(planner, nullptr);
    ASSERT_EQ(ok.ret, Return::SUCCESS);
    EXPECT_FALSE(ok.parked) << "planner " << planner << ": reason " << static_cast<int>(ok.reason);
    EXPECT_EQ(ctrl_->GetSegmentMode(), rtc::catching::CatchingSegmentMode::kMpc);
    EXPECT_EQ(ctrl_->IsSegmentPlannerConfigured(), planner);
  }
  expect_park("K_n = 0", true, [](YAML::Node& y) { y["catching"]["joint_cmd"]["K_n"] = 0.0; });
  expect_park("eta_v = 1", true, [](YAML::Node& y) {
    y["catching"]["planner"]["search"]["grid"]["gamma"]["eta_v"] = 1.0;
    y["catching"]["planner"]["segment"]["mpc"]["eta_v"] = 1.0;  // the key the mpc gate reads
  });
  // MD-45, MD-70: a plan goes out only with a segment that starts before
  // t_c, so without a pre-catch grid there is no MPC segment planner to build.
  expect_park("an MPC segment planner without the pre-catch grid", true, [](YAML::Node& y) {
    y["catching"]["planner"]["segment"]["mpc"]["approach"]["n_pre_max"] = 0;
  });
  expect_park("no planner and no oracle", false,
              [](YAML::Node& y) { y["diagnostic"]["oracle_plan"]["enabled"] = false; });
  expect_park("no catch sub-model", false,
              [](YAML::Node& y) { y["catching"]["planner"]["sub_model"] = "no_such_model"; });
  auto slow = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
  slow.at("ur5e").joint_limits->max_velocity[3] = 0.0;
  expect_park("an arm joint without max_velocity", false, nullptr, slow);
}

TEST_F(CatchingPlanLaneTest, EachMissingMpcDockingPrerequisiteParksAndNamesItsOwnKey) {
  // SegmentModeUnmet's docking branch (E1-F17): the same three prerequisites
  // the mpc test above parks on, read from the keys of the planner the arm
  // follows — and the log says which key. Under the shipped docking design
  // (the hand's capture set and the planner's fragment) the profile itself
  // configures, on both writers of the plan box.
  using integrated_bringup::CatchingParkReason;
  using Return = DemoCatchingController::CallbackReturn;
  const auto docking = [](YAML::Node& y) {
    integrated_bringup::testfx::ApplyShippedDocking(y);
    y["catching"]["planner"]["segment"]["mode"] = "mpc_docking";
  };
  for (const bool planner : {true, false}) {
    const WarnCapture log;
    const ConfigureVerdict ok = ConfigureOnly(planner, docking, SimAxisConfigs());
    ASSERT_EQ(ok.ret, Return::SUCCESS);
    EXPECT_FALSE(ok.parked) << "planner " << planner << ": reason " << static_cast<int>(ok.reason);
    EXPECT_EQ(ctrl_->GetSegmentMode(), rtc::catching::CatchingSegmentMode::kMpcDocking);
    EXPECT_EQ(ctrl_->IsSegmentPlannerConfigured(), planner);
    EXPECT_EQ(ctrl_->GetMpcDockingSegmentPlannerForTesting() != nullptr, planner);
  }
  const auto expect_park = [&](const char* what, bool planner, const char* said,
                               const std::function<void(YAML::Node&)>& more) {
    const WarnCapture log;
    const ConfigureVerdict v = ConfigureOnly(
        planner,
        [&](YAML::Node& y) {
          docking(y);
          more(y);
        },
        SimAxisConfigs());
    EXPECT_EQ(v.ret, Return::SUCCESS) << what << ": a profile mistake parks, it does not fail";
    EXPECT_TRUE(v.parked) << what;
    EXPECT_EQ(v.reason, CatchingParkReason::kSegmentModeUnmet) << what;
    EXPECT_TRUE(WarnCapture::Contains(said)) << what << ": the log does not say '" << said << "'";
    const rclcpp_lifecycle::State prev;
    EXPECT_EQ(ctrl_->on_activate(prev), Return::FAILURE) << what;
  };
  // eta_v >= 1 cannot reach SegmentModeUnmet's docking branch: the planner's
  // own parse refuses the key first, and that is a configure FAILURE, not a
  // park (the mpc key is not parsed that way). The branch's eta_v line is
  // unreachable through this key.
  {
    const WarnCapture log;
    const ConfigureVerdict v = ConfigureOnly(
        true,
        [&](YAML::Node& y) {
          docking(y);
          y["catching"]["planner"]["segment"]["mpc_docking"]["eta_v"] = 1.0;
        },
        SimAxisConfigs());
    EXPECT_EQ(v.ret, Return::FAILURE);
    EXPECT_TRUE(WarnCapture::Contains("planner.segment.mpc_docking.eta_v"));
  }
  // The admission age bound is 50 ms (kSegmentAdmissionMaxAgeNs): a replan
  // budget plus three control periods that reaches it would refuse the
  // segments that wait in the box for the pending slot.
  expect_park("replan_s + 3 periods at the age bound", true,
              "planner.segment.mpc_docking.budget.replan_s", [](YAML::Node& y) {
                y["catching"]["planner"]["segment"]["mpc_docking"]["budget"]["replan_s"] = 0.05;
              });
  expect_park("no planner and no oracle", false, "no segment planner runs",
              [](YAML::Node& y) { y["diagnostic"]["oracle_plan"]["enabled"] = false; });
}

TEST_F(CatchingPlanLaneTest, AMalformedSegmentModeOrSwitchMarginFailsTheConfigure) {
  using Return = DemoCatchingController::CallbackReturn;
  EXPECT_EQ(ConfigureOnly(
                true, [](YAML::Node& y) { y["catching"]["planner"]["segment"]["mode"] = "mcp"; })
                .ret,
            Return::FAILURE);
  EXPECT_EQ(ConfigureOnly(true,
                          [](YAML::Node& y) {
                            y["catching"]["planner"]["segment"]["mpc"]["switch_margin"] = 0.0;
                          })
                .ret,
            Return::FAILURE);
}

TEST_F(CatchingPlanLaneTest, ClosedFormBuildsNoMpcSegmentCoresEvenWhenTheyAreEnabled) {
  // MD-44: the planner runs the closed form's part only.
  for (const char* mode : {"closed_form", ""}) {
    const ConfigureVerdict v = ConfigureOnly(true, [mode](YAML::Node& y) {
      if (*mode == '\0') {
        y["catching"]["planner"]["segment"].remove("mode");
      } else {
        y["catching"]["planner"]["segment"]["mode"] = mode;
      }
    });
    ASSERT_EQ(v.ret, DemoCatchingController::CallbackReturn::SUCCESS) << mode;
    EXPECT_FALSE(v.parked) << mode;
    EXPECT_EQ(ctrl_->GetSegmentMode(), rtc::catching::CatchingSegmentMode::kClosedForm) << mode;
    EXPECT_FALSE(ctrl_->IsSegmentPlannerConfigured()) << "mode '" << mode << "'";
  }
}

TEST_F(CatchingPlanLaneTest, ClosedFormDoesNotParkOnAnMpcSegmentSettingItNeverReads) {
  // MD-44 (/code-review 2026-09-30): under closed_form no MPC segment core is built,
  // so an MPC segment torque box that would not fit the CLIK's is not a profile
  // mistake there — only the WARN that the MPC is not built. Under mpc the
  // same setting parks.
  const auto misfit = [](const char* mode) {
    return [mode](YAML::Node& y) {
      y["catching"]["planner"]["segment"]["mode"] = mode;
      y["catching"]["planner"]["segment"]["mpc"]["eta_tau"] = 0.75;
      y["catching"]["planner"]["segment"]["mpc"]["publish"]["slack_max"] = 0.1;
      y["catching"]["joint_cmd"]["accel_constraint"] = "dynamic";
      y["catching"]["joint_cmd"]["eta_tau"] = 0.8;
    };
  };
  const ConfigureVerdict closed = ConfigureOnly(true, misfit("closed_form"));
  ASSERT_EQ(closed.ret, DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_FALSE(closed.parked) << "park reason " << static_cast<int>(closed.reason);
  EXPECT_FALSE(ctrl_->IsSegmentPlannerConfigured());

  const ConfigureVerdict mpc = ConfigureOnly(true, misfit("mpc"));
  ASSERT_EQ(mpc.ret, DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(mpc.parked);
  EXPECT_EQ(mpc.reason, integrated_bringup::CatchingParkReason::kMpcSegmentInvalid);
}

TEST_F(CatchingPlanLaneTest, TheLawIsChosenByPlannerSegmentModeAlone) {
  // MPC MD-91: there is no `enabled` switch — the planner solves the segment
  // MPC exactly when `planner.segment.mode` is mpc.
  using integrated_bringup::CatchingParkReason;
  using Return = DemoCatchingController::CallbackReturn;

  // closed_form: no MPC segment core, and no WARN about the segment MPC either.
  {
    const WarnCapture warns;
    const ConfigureVerdict v = ConfigureOnly(
        true, [](YAML::Node& y) { y["catching"]["planner"]["segment"]["mode"] = "closed_form"; });
    ASSERT_EQ(v.ret, Return::SUCCESS);
    EXPECT_FALSE(v.parked);
    EXPECT_FALSE(ctrl_->IsSegmentPlannerConfigured());
    EXPECT_FALSE(WarnCapture::Contains("segment.mpc")) << "closed_form must not warn";
    EXPECT_FALSE(WarnCapture::Contains("not built")) << "closed_form must not warn";
  }
  // mpc + planner on: the MPC segment planner runs.
  {
    const ConfigureVerdict v = ConfigureOnly(true, nullptr);
    ASSERT_EQ(v.ret, Return::SUCCESS);
    EXPECT_FALSE(v.parked) << "reason " << static_cast<int>(v.reason);
    EXPECT_TRUE(ctrl_->IsSegmentPlannerConfigured());
  }
  // mpc + planner off + the oracle plan profile: configures (a test writes
  // the box), with no MPC segment planner of its own.
  {
    const ConfigureVerdict v = ConfigureOnly(false, nullptr);
    ASSERT_EQ(v.ret, Return::SUCCESS);
    EXPECT_FALSE(v.parked) << "reason " << static_cast<int>(v.reason);
    EXPECT_FALSE(ctrl_->IsSegmentPlannerConfigured());
  }
  // mpc + planner off + no oracle: nothing writes a segment, so it parks
  // with the prerequisite reason — never the MPC-segment-config one.
  {
    const ConfigureVerdict v = ConfigureOnly(
        false, [](YAML::Node& y) { y["diagnostic"]["oracle_plan"]["enabled"] = false; });
    ASSERT_EQ(v.ret, Return::SUCCESS);
    EXPECT_TRUE(v.parked);
    EXPECT_EQ(v.reason, CatchingParkReason::kSegmentModeUnmet);
  }
}

TEST_F(CatchingPlanLaneTest, ALeftoverDecelMpcKeyParksUnderEitherMode) {
  // `planner.decel_mpc` moved to `planner.segment.mpc` (#711) and is no longer
  // read. A profile that still writes it — its long-removed `enabled` switch
  // included, whatever the value — would run the shipped MPC values under its
  // own name, so it parks under either mode and the ERROR names both paths.
  using integrated_bringup::CatchingParkReason;
  using Return = DemoCatchingController::CallbackReturn;
  for (const char* mode : {"mpc", "closed_form"}) {
    for (const int stale : {0, 1}) {
      const std::string tag =
          std::string(mode) + ", " + (stale == 1 ? "enabled: true" : "enabled: false");
      const WarnCapture logs;
      const ConfigureVerdict v = ConfigureOnly(true, [&](YAML::Node& y) {
        y["catching"]["planner"]["segment"]["mode"] = mode;
        y["catching"]["planner"]["decel_mpc"]["enabled"] = stale == 1;
      });
      ASSERT_EQ(v.ret, Return::SUCCESS) << tag;
      EXPECT_TRUE(v.parked) << tag;
      EXPECT_EQ(v.reason, CatchingParkReason::kRemovedKey) << tag;
      EXPECT_TRUE(WarnCapture::Contains("'catching.planner.decel_mpc' was renamed")) << tag;
      EXPECT_TRUE(WarnCapture::Contains("'catching.planner.segment.mpc'")) << tag;
      EXPECT_FALSE(ctrl_->IsSegmentPlannerConfigured()) << tag;
    }
  }
}

// ── #711: a function is handed its own keys ─────────────────────────────────

namespace {

/// `root` with the dotted `path` set to `value` (maps created on the way).
void SetCatchingPath(YAML::Node root, const std::string& path, double value) {
  const std::size_t dot = path.find('.');
  if (dot == std::string::npos) {
    root[path] = value;
    return;
  }
  SetCatchingPath(root[path.substr(0, dot)], path.substr(dot + 1), value);
}

}  // namespace

TEST_F(CatchingPlanLaneTest, EachFunctionIsHandedItsOwnKeysAndNoOtherFunctions) {
  // The search and the mpc segment planner each read their numbers from their
  // own map, and two of those numbers used to be one key for both. Moving one
  // key must move the constant of the function that owns it, and nothing else
  // either function was handed — a read wired to the other function's key, or
  // to the closed_form law's, shows as the wrong column moving (or none).
  enum Seen : std::size_t {
    kGridEtaV,
    kGridVMax,
    kGridADec,
    kGridOmega,
    kGridAMax,
    kMpcEtaV,
    kMpcVEps,
    kGateEtaV,
    kSeenCount
  };

  const std::array<const char*, kSeenCount> names{
      "search eta_v",     "search v_max", "search a_dec", "search ref omega",
      "search ref a_max", "mpc eta_v",    "mpc v_eps",    "switch gate eta_v"};
  using Values = std::array<double, kSeenCount>;
  // Mode mpc (the fixture's): a copy that differs from the law's value only
  // warns there, so every function is built and can be read.
  const auto seen_with = [this](const std::function<void(YAML::Node&)>& tweak, Values& out) {
    const ConfigureVerdict v = ConfigureOnly(true, tweak);
    if (v.ret != DemoCatchingController::CallbackReturn::SUCCESS || v.parked ||
        !ctrl_->IsSegmentPlannerConfigured()) {
      return false;
    }
    const auto& grid = ctrl_->GetGridCatchSearchConstantsForTesting();
    const auto& mpc = ctrl_->GetMpcSegmentPlannerConstantsForTesting();
    out = {grid.eta_v,     grid.v_max, grid.a_dec, grid.ref_omega,
           grid.ref_a_max, mpc.eta_v,  mpc.v_eps,  ctrl_->GetSegmentEtaVForTesting()};
    return true;
  };
  Values base{};
  ASSERT_TRUE(seen_with(nullptr, base));

  struct Case {
    const char* path;  // under catching:
    double value;
    std::vector<Seen> moves;
  };

  const std::vector<Case> cases{
      {"planner.search.grid.gamma.eta_v", 0.8, {kGridEtaV}},
      {"planner.search.grid.reference.v_max", 2.5, {kGridVMax}},
      {"planner.search.grid.stop.a_dec", 8.0, {kGridADec}},
      {"planner.search.grid.reference.omega", 12.0, {kGridOmega}},
      {"planner.search.grid.reference.a_max", 25.0, {kGridAMax}},
      {"planner.segment.mpc.eta_v", 0.7, {kMpcEtaV, kGateEtaV}},
      {"planner.segment.mpc.v_eps", 1.0e-3, {kMpcVEps}},
      // The keys these used to be read from reach neither function now.
      {"planner.search.grid.ik.v_eps", 2.0e-3, {}},
      {"reference.v_max", 2.5, {}},
      {"reference.omega", 12.0, {}},
      {"reference.a_max", 25.0, {}},
      {"supervisor.decel.a_dec", 8.0, {}},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.path);
    Values got{};
    ASSERT_TRUE(
        seen_with([&c](YAML::Node& y) { SetCatchingPath(y["catching"], c.path, c.value); }, got));
    for (std::size_t i = 0; i < kSeenCount; ++i) {
      const bool moves =
          std::find(c.moves.begin(), c.moves.end(), static_cast<Seen>(i)) != c.moves.end();
      if (moves) {
        EXPECT_EQ(got[i], c.value) << names[i];
        EXPECT_NE(base[i], c.value) << names[i] << ": the baseline already has this value";
      } else {
        EXPECT_EQ(got[i], base[i]) << names[i] << " moved";
      }
    }
  }
}

TEST_F(CatchingPlanLaneTest, ASearchCopyThatDiffersParksUnderClosedFormAndOnlyWarnsUnderMpc) {
  // The search rolls a candidate out on its own copies of the closed_form
  // law's values. Under closed_form the arm follows that law, so a copy that
  // differs ranks candidates by a motion the arm will not make: park, naming
  // the pair. Under mpc they are two functions' values: a warning, then run.
  // (zeta has no second legal value — the validator refuses anything but 1.)
  using integrated_bringup::CatchingParkReason;
  using Return = DemoCatchingController::CallbackReturn;

  struct Pair {
    const char* copy;
    const char* source;
    double other;  // legal for both keys, and not the fixture's value
  };

  for (const Pair& pair : {Pair{"planner.search.grid.reference.v_max", "reference.v_max", 2.5},
                           Pair{"planner.search.grid.reference.omega", "reference.omega", 12.0},
                           Pair{"planner.search.grid.reference.a_max", "reference.a_max", 25.0},
                           Pair{"planner.search.grid.stop.a_dec", "supervisor.decel.a_dec", 8.0}}) {
    const std::string differs =
        std::string("'catching.") + pair.copy + "' differs from 'catching." + pair.source + "'";
    for (const bool move_copy : {true, false}) {
      const char* moved = move_copy ? pair.copy : pair.source;
      for (const char* mode : {"closed_form", "mpc"}) {
        SCOPED_TRACE(std::string(moved) + " moved, " + mode);
        const WarnCapture logs;
        const ConfigureVerdict v = ConfigureOnly(true, [&](YAML::Node& y) {
          y["catching"]["planner"]["segment"]["mode"] = mode;
          SetCatchingPath(y["catching"], moved, pair.other);
        });
        ASSERT_EQ(v.ret, Return::SUCCESS);
        EXPECT_TRUE(WarnCapture::Contains(differs));
        if (std::string(mode) == "closed_form") {
          EXPECT_TRUE(v.parked);
          EXPECT_EQ(v.reason, CatchingParkReason::kSearchCopyDiffers);
          EXPECT_TRUE(WarnCapture::Contains("DISABLED: " + differs));
        } else {
          EXPECT_FALSE(v.parked) << "reason " << static_cast<int>(v.reason);
          EXPECT_TRUE(ctrl_->IsSegmentPlannerConfigured());
          EXPECT_FALSE(WarnCapture::Contains("DISABLED"));
        }
      }
    }
    // With the planner off nothing reads the copies: no park, no word.
    const WarnCapture logs;
    const ConfigureVerdict off = ConfigureOnly(false, [&](YAML::Node& y) {
      y["catching"]["planner"]["segment"]["mode"] = "closed_form";
      SetCatchingPath(y["catching"], pair.copy, pair.other);
    });
    ASSERT_EQ(off.ret, Return::SUCCESS) << pair.copy;
    EXPECT_NE(off.reason, CatchingParkReason::kSearchCopyDiffers) << pair.copy;
    EXPECT_FALSE(WarnCapture::Contains("differs from")) << pair.copy;
  }
  // Positive control: equal copies say nothing under either mode.
  for (const char* mode : {"closed_form", "mpc"}) {
    const WarnCapture logs;
    const ConfigureVerdict v = ConfigureOnly(
        true, [mode](YAML::Node& y) { y["catching"]["planner"]["segment"]["mode"] = mode; });
    ASSERT_EQ(v.ret, Return::SUCCESS) << mode;
    EXPECT_FALSE(v.parked) << mode << ": reason " << static_cast<int>(v.reason);
    EXPECT_FALSE(WarnCapture::Contains("differs from")) << mode;
  }
}

TEST_F(CatchingPlanLaneTest, AnMpcEtaVThatDiffersFromTheSearchsWarnsUnderMpcOnly) {
  using Return = DemoCatchingController::CallbackReturn;
  const std::string needle =
      "'catching.planner.segment.mpc.eta_v' (0.7) differs from "
      "'catching.planner.search.grid.gamma.eta_v' (0.9)";
  for (const char* mode : {"mpc", "closed_form"}) {
    const WarnCapture logs;
    const ConfigureVerdict v = ConfigureOnly(true, [mode](YAML::Node& y) {
      y["catching"]["planner"]["segment"]["mode"] = mode;
      y["catching"]["planner"]["segment"]["mpc"]["eta_v"] = 0.7;
    });
    ASSERT_EQ(v.ret, Return::SUCCESS) << mode;
    EXPECT_FALSE(v.parked) << mode << ": reason " << static_cast<int>(v.reason);
    // closed_form builds no mpc segment planner: its eta_v is read by nothing.
    EXPECT_EQ(WarnCapture::Contains(needle), std::string(mode) == "mpc") << mode;
  }
}

TEST_F(CatchingPlanLaneTest, AReconfigureToClosedFormClearsTheMpcSegmentPlannersBox) {
  // The box getters report THIS configuration (/code-review 2026-09-30): a
  // closed_form re-configure builds no MPC segment planner, so the box is zero.
  ASSERT_EQ(ConfigureOnly(true, nullptr).ret, DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_TRUE(ctrl_->IsSegmentPlannerConfigured());
  ASSERT_NE(ctrl_->GetMpcSegmentPlannerQMaxForTesting()[0], 0.0);
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  YAML::Node yaml = ConfigureOnlyYaml(true);
  yaml["catching"]["planner"]["segment"]["mode"] = "closed_form";
  ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl_->IsSegmentPlannerConfigured());
  for (int d = 0; d < kUr5eArmDof; ++d) {
    const auto u = static_cast<std::size_t>(d);
    EXPECT_EQ(ctrl_->GetMpcSegmentPlannerQMinForTesting()[u], 0.0) << "joint " << d;
    EXPECT_EQ(ctrl_->GetMpcSegmentPlannerQMaxForTesting()[u], 0.0) << "joint " << d;
  }
}

TEST_F(CatchingPlanLaneTest, TheMpcSegmentPlannersBoxSitsInsideTheCliksMarginedBox) {
  // MD-42. The fixture's elbow limit (±3.14) is inside the URDF's (±π), so the
  // CLIK's margined box (±3.09) is the binding side there — the case the
  // shipped iiwa7 A7 is on (device 3.0543 < URDF 3.05433).
  const ConfigureVerdict v = ConfigureOnly(true, nullptr);
  ASSERT_EQ(v.ret, DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_FALSE(v.parked);
  ASSERT_TRUE(ctrl_->IsSegmentPlannerConfigured());
  const auto& lo = ctrl_->GetMarginedArmQMinForTesting();
  const auto& hi = ctrl_->GetMarginedArmQMaxForTesting();
  ASSERT_EQ(static_cast<int>(lo.size()), kUr5eArmDof);
  for (int d = 0; d < kUr5eArmDof; ++d) {
    const auto u = static_cast<std::size_t>(d);
    EXPECT_GE(ctrl_->GetMpcSegmentPlannerQMinForTesting()[u], lo[u]) << "joint " << d;
    EXPECT_LE(ctrl_->GetMpcSegmentPlannerQMaxForTesting()[u], hi[u]) << "joint " << d;
  }
  EXPECT_DOUBLE_EQ(ctrl_->GetMpcSegmentPlannerQMaxForTesting()[2], 3.14 - 0.05);
  EXPECT_DOUBLE_EQ(ctrl_->GetMpcSegmentPlannerQMinForTesting()[2], -3.14 + 0.05);
}

void CatchingPlanLaneTest::RealClockPairCase(bool docking) {
  // MPC E1-F09 (#662), planner in the loop, under mpc and (E1-F17) the real
  // MpcDockingSegmentPlanner: a reachable throw, the
  // APPROACH–stop grid on, the arm servoed so the trial runs to its
  // end. The planner publishes the plan TOGETHER with its first segment — one
  // publish_ns, the segment starting before t_c — and the RT takes the two on
  // one tick. From there it reports the segment it holds, the planner replans
  // from that report alone (MD-58), and the arm follows the planner's
  // segments from APPROACH through DECEL into HOLD. Structural claims — which
  // segment was taken and followed, and that the planner always found its
  // source — because a loaded host moves every timing here.
  const pinocchio::SE3 pose = [&] {
    CatchFrameOracle oracle(*builder_);
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    return oracle.PoseAt(
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names, home,
        kUr5eArmDof);
  }();
  const Eigen::Vector3d z = pose.rotation().col(2);
  // mpc: the throw of the case's origin — 1.5 m/s down the approach axis, the
  // catch point 3 cm out, reached 0.5 s after every message.
  Eigen::Vector3d target = pose.translation() + 0.03 * z;
  Eigen::Vector3d vel = -1.5 * z;
  Eigen::Vector3d p0 = target - 0.5 * vel;
  if (docking) {
    // The docking planner runs on the sim axis only (#654).
    sim_axis_ = true;
    // mpc_docking: the ball the HAND'S capture set holds — a closing speed in
    // the identified band (robot.hand.docking.speed, c 0.5 - 0.6 m/s; 1.5 m/s
    // is outside it, the planner would refuse every candidate) and a line
    // that crosses the entrance plane (s_ent) at the lateral set's reference
    // point rho_ref, whose faces are millimetres wide. It flies: the planner
    // replans against a ball whose crossing time must not slide with every
    // message. Its prediction is as tight as that set needs (the cloud's own
    // (5 mm)² would leave the chance rows no lateral room).
    const YAML::Node hand = YAML::LoadFile(
        ament_index_cpp::get_package_share_directory("integrated_bringup") +
        "/config/ur5e_p1b/controllers/demo_catching_controller.yaml")["demo_catching_controller"]
                                                                     ["catching"]["robot"]["hand"]
                                                                     ["docking"];
    const double s_ent = hand["s_ent"].as<double>();
    const std::vector<double> rho_ref = hand["lateral"]["rho_ref"].as<std::vector<double>>();
    target = pose.translation() + pose.rotation() * Eigen::Vector3d(rho_ref[0], rho_ref[1], s_ent);
    vel = -0.55 * z;
    p0 = target - 1.2 * vel;
    ball_advances_ = true;
    ball_cov_ = std::array<double, 6>{1e-10, 1e-10, 1e-10, 1e-10, 1e-10, 1e-10};
  }
  ball_set_ = true;
  ball_p0_ = {p0.x(), p0.y(), p0.z()};
  ball_vel_ = {vel.x(), vel.y(), vel.z()};
  cloud_n_ = docking ? 40 : 20;
  servo_ = true;
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false, /*planner=*/true, [docking](YAML::Node& y) {
    // Head-room for a loaded host: this case is about what is published,
    // taken and followed, not how fast. The replan budget stays inside the
    // admission age bound (replan_s + 3 ticks < 50 ms, or configure parks).
    y["catching"]["planner"]["search"]["grid"]["budget_s"] = 0.03;
    if (docking) {
      // The shipped docking design, with the fixture's budgets: the shipped
      // budget.first_s (0.035 s) is below a docking first solve on a loaded
      // host (in the #743 sim runs none finished inside it, and none inside
      // 0.080 s either — the times recorded there are where the deadline
      // cut the solve), so it is 0.15 s — a first segment's node 0 is
      // first_s + 2h past the wake, well inside the 1.2 s flight. replan_s
      // 0.04 is as for mpc.
      integrated_bringup::testfx::ApplyShippedDocking(y);
      YAML::Node d = y["catching"]["planner"]["segment"]["mpc_docking"];
      d["budget"]["first_s"] = 0.15;
      d["budget"]["replan_s"] = 0.04;
      y["catching"]["planner"]["segment"]["mode"] = "mpc_docking";
    } else {
      YAML::Node d = y["catching"]["planner"]["segment"]["mpc"];
      ApproachGrid(y);
      d["budget"]["first_s"] = 0.05;
      d["budget"]["replan_s"] = 0.04;
      y["catching"]["planner"]["segment"]["mode"] = "mpc";
    }
    // The shipped CLIK form (MD-7): the MPC bounds acceleration by torque
    // rows, so the CLIK that executes its segments must too — under the
    // fixture's constant box the command falls behind the segment.
    y["catching"]["joint_cmd"]["accel_constraint"] = "dynamic";
    y["catching"]["joint_cmd"]["eta_tau"] = 0.8;
  }));
  ASSERT_TRUE(ctrl_->IsSegmentPlannerConfigured());
  ASSERT_EQ(ctrl_->GetSegmentMode(), docking ? rtc::catching::CatchingSegmentMode::kMpcDocking
                                             : rtc::catching::CatchingSegmentMode::kMpc);
  using Event = integrated_bringup::CatchingDiagLogPod::SegmentEvent;
  bool approached = false;
  bool pair_taken = false;
  int not_followed = 0;
  int not_followed_while_waiting = 0;
  int replans_with_a_source = 0;
  int switches = 0;
  int gate_refused = 0;
  double rho_max = 0.0;
  double rho_refused_max = 0.0;
  std::set<Mode> followed_in;
  std::set<std::uint32_t> followed_seqs;
  std::array<int, 11> events{};  // tick records per SegmentEvent, for the failure message
  Mode last = Mode::kIdle;
  integrated_bringup::CatchingDiagLogPod end_record{};
  auto next = std::chrono::steady_clock::now();
  for (int t = 0; t < 2500; ++t) {
    if (t % 10 == 0 && last != Mode::kDecel && last != Mode::kHold) {
      Publish(next_seq_++);
    }
    const Mode before = ctrl_->GetMode();
    static_cast<void>(Tick());
    last = ctrl_->GetMode();
    const auto record = ctrl_->GetLastTickRecord();
    const rtc::catching::PlannerRtState rt = ctrl_->GetPlannerRtState();
    end_record = record;
    if (const auto ev = static_cast<std::size_t>(record.segment_event); ev < events.size()) {
      ++events[ev];
    }
    if (!approached && last == Mode::kApproach) {
      approached = true;
      // The tick that took the plan took its first segment with it.
      const PlanSnapshot plan = ctrl_->GetFollowedPlanForTesting();
      const rtc::catching::SegmentSnapshot seg = ctrl_->GetPendingSegmentForTesting();
      EXPECT_EQ(before, Mode::kTracking);
      EXPECT_EQ(record.segment_event, Event::kAdmitted);
      EXPECT_TRUE(ctrl_->HasPendingSegmentForTesting());
      EXPECT_TRUE(rt.segment_pending);
      EXPECT_EQ(rt.segment_pending_seq, seg.segment_seq);
      EXPECT_FALSE(rt.segment_active);
      EXPECT_TRUE(seg.valid);
      EXPECT_EQ(seg.plan_id, plan.plan_id);
      EXPECT_EQ(seg.t_c_ns, plan.t_c_ns);
      EXPECT_EQ(seg.publish_ns, plan.publish_ns) << "the pair carries one publish stamp";
      EXPECT_EQ(seg.token.generation, plan.token.generation);
      EXPECT_GT(seg.n_pre, 0);
      EXPECT_LT(seg.t0_ns, plan.t_c_ns);
      EXPECT_TRUE(rtc::catching::ValidateSegmentNodes(seg));
      pair_taken = seg.valid && seg.plan_id == plan.plan_id;
    }
    if (approached && (last == Mode::kApproach || last == Mode::kCommitted ||
                       last == Mode::kClosing || last == Mode::kDecel)) {
      // The planner's newest wake WHILE the RT holds a segment of its plan.
      const auto wake = ctrl_->GetPlannerThread()->LastRecord();
      const bool replan = wake.segment.kind != rtc::catching::SegmentKind::kFirst &&
                          wake.segment.outcome != rtc::catching::SegmentOutcome::kOff;
      const bool no_source =
          replan && wake.segment.outcome == rtc::catching::SegmentOutcome::kNotFollowed;
      // A docking replan during the first segment's wait: its budget
      // (replan_s) is shorter than the first segment's (first_s, which has to
      // cover a cold solve), so the first lattice point it reaches can lie
      // before the waiting segment's node 0 — nothing the RT reports covers
      // it yet. Counted apart and reported; every other wake has to find its
      // source.
      if (docking && no_source && rt.segment_pending && !rt.segment_active) {
        ++not_followed_while_waiting;
      } else {
        not_followed += no_source ? 1 : 0;
      }
      replans_with_a_source += replan && wake.segment.source_seq != 0 ? 1 : 0;
      EXPECT_TRUE(rt.segment_pending || rt.segment_active)
          << "the RT follows a plan and reports no segment, mode " << static_cast<int>(last);
    }
    if (record.segment_event == Event::kSwitched) {
      ++switches;
      rho_max = std::max(rho_max, record.segment_rho);
    }
    if (record.segment_event == Event::kGateRefused) {
      ++gate_refused;
      rho_refused_max = std::max(rho_refused_max, record.segment_rho);
    }
    if (record.segment_following) {
      followed_in.insert(before);
      followed_seqs.insert(record.segment_seq);
      EXPECT_TRUE(rt.segment_active);
      EXPECT_EQ(rt.segment_seq, record.segment_seq);
      EXPECT_FALSE(record.ref_valid) << "the soft-catch reference ran while a segment was followed";
    }
    if (last == Mode::kAbortSafe || last == Mode::kRetreat || last == Mode::kHold) {
      break;
    }
    next += std::chrono::microseconds(2000);
    std::this_thread::sleep_until(next);
  }
  const auto planner = ctrl_->GetPlannerThread()->LastRecord();
  std::ostringstream counts;
  for (std::size_t e = 1; e < events.size(); ++e) {
    counts << e << ":" << events[e] << " ";
  }
  ASSERT_TRUE(approached) << "no plan was adopted; last planner record: outcome "
                          << rtc::catching::CycleOutcomeName(planner.outcome) << ", decel "
                          << rtc::catching::SegmentOutcomeName(planner.segment.outcome) << " / "
                          << planner.segment.core_reason_name << "; RT segment refusal "
                          << static_cast<int>(end_record.segment_refusal);
  EXPECT_TRUE(pair_taken);
  ASSERT_EQ(last, Mode::kHold) << "the trial did not reach HOLD: mode " << static_cast<int>(last)
                               << ", reason " << static_cast<int>(ctrl_->GetLastReason())
                               << ", segment events " << counts.str() << ", last event "
                               << static_cast<int>(end_record.segment_event) << ", gate rho "
                               << end_record.segment_rho << " (joint "
                               << end_record.segment_gate_joint << "), last segment step "
                               << rtc::catching::SegmentOutcomeName(planner.segment.outcome)
                               << " / " << planner.segment.core_reason_name;
  // Followed from APPROACH to the stop — DECEL is the same chain going on.
  // (Where node 0 falls — APPROACH or just past the freeze — is the throw's
  // lead; before it the command is held with the segment reported pending.)
  EXPECT_GE(followed_in.count(Mode::kApproach) + followed_in.count(Mode::kCommitted), 1U)
      << "events " << counts.str();
  EXPECT_EQ(followed_in.count(Mode::kClosing), 1U);
  EXPECT_EQ(followed_in.count(Mode::kDecel), 1U);
  EXPECT_GE(switches, 1);
  EXPECT_GE(followed_seqs.size(), 1U);
  // The planner replanned from what the RT reported, every time.
  EXPECT_EQ(not_followed, 0) << "a replan wake found no source while the RT held a segment";
  EXPECT_GT(replans_with_a_source, 0) << "no replan wake was sampled while the plan was followed";
  // The stop's end is fixed (MD-21): t_c + N_s·Δ_s, whichever segment ends it.
  const rtc::catching::SegmentSnapshot final_seg = ctrl_->GetFollowedSegmentForTesting();
  const PlanSnapshot plan = ctrl_->GetFollowedPlanForTesting();
  ASSERT_TRUE(final_seg.valid);
  EXPECT_EQ(rtc::catching::SegmentNodeTimeNs(final_seg, final_seg.n_nodes),
            plan.t_c_ns + 7 * 50'000'000LL);
  // The solves ran on the planner thread: still exactly one mpc_main (no
  // thread of its own — E-7).
  EXPECT_EQ(
      integrated_bringup::testfx::ThreadsNamed(rtc::SelectThreadConfigs().mpc.main.name).size(),
      1U);
  std::ostringstream os;
  os << "switches " << switches << ", segments followed " << followed_seqs.size()
     << ", gate refusals " << gate_refused << " (max rho " << rho_refused_max
     << "), max switch rho " << rho_max << ", replan wakes with a source " << replans_with_a_source
     << ", replan wakes before node 0 with no source yet " << not_followed_while_waiting
     << "; events " << counts.str();
  RecordProperty("real_clock_closed_loop", os.str());
  std::printf("[ MEASURED ] %s\n", os.str().c_str());
}

TEST_F(CatchingPlanLaneTest, OnTheRealClockTheRtTakesThePairAndFollowsThePlannersSegments) {
  RealClockPairCase(/*docking=*/false);
}

TEST_F(CatchingPlanLaneTest,
       OnTheRealClockTheRtTakesTheDockingPlannersPairAndFollowsItsSegmentsToTheHold) {
  RealClockPairCase(/*docking=*/true);
}

// ── Vision world → model world (L3 §4.2, S6-C sim finding) ──────────────────

class CatchingVisionFrameTest : public CatchingPlanLaneTest {
 protected:
  /// The reachable-throw case of the test above, with the prediction written
  /// in the VISION world the sim publishes in. On ur5e_p1b the model root is
  /// `base_link`, a 180° turn about z from `base`, and `base` is the sim world
  /// (§11): p_world = (−x, −y, z) of the model-world point.
  void SetUpWorldFrameBall() {
    CatchFrameOracle oracle(*builder_);
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    const pinocchio::SE3 pose = oracle.PoseAt(
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names, home,
        kUr5eArmDof);
    const Eigen::Vector3d z = pose.rotation().col(2);
    vel_model_ = -1.5 * z;
    p0_model_ = pose.translation() + 0.03 * z - 0.5 * vel_model_;
    const auto to_world = [](const Eigen::Vector3d& m) {
      return Eigen::Vector3d(-m.x(), -m.y(), m.z());
    };
    const Eigen::Vector3d p0_w = to_world(p0_model_);
    const Eigen::Vector3d vel_w = to_world(vel_model_);
    ball_set_ = true;
    ball_p0_ = {p0_w.x(), p0_w.y(), p0_w.z()};
    ball_vel_ = {vel_w.x(), vel_w.y(), vel_w.z()};
    cloud_n_ = 20;
  }

  bool RunUntilApproach() {
    for (int t = 0; t < 600; ++t) {
      if (t % 10 == 0) {
        Publish(next_seq_++);
      }
      static_cast<void>(Tick());
      std::this_thread::sleep_for(1ms);
      if (ctrl_->GetMode() == Mode::kApproach) {
        return true;
      }
    }
    return false;
  }

  Eigen::Vector3d p0_model_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_model_{Eigen::Vector3d::Zero()};
};

TEST_F(CatchingVisionFrameTest, AWorldFramePredictionReachesThePlannerInModelCoordinates) {
  SetUpWorldFrameBall();
  // The shipped ur5e_p1b keys: vision is measured against `base`, and
  // base_T_world is the identity the sim measured (§11).
  ASSERT_NO_FATAL_FAILURE(BringUp(false, true, [](YAML::Node& y) {
    y["catching"]["io"]["arm_base_frame"] = "base";
    y["catching"]["io"]["base_T_world"]["yaw_deg"] = 0.0;
    y["catching"]["io"]["base_T_world"]["translation"] = std::vector<double>{0.0, 0.0, 0.0};
  }));
  const auto& cfg = ctrl_->GetTrajInputConfig();
  ASSERT_TRUE(cfg.to_model);
  const std::array<double, 9> rz180{-1, 0, 0, 0, -1, 0, 0, 0, 1};
  for (std::size_t k = 0; k < 9; ++k) {
    EXPECT_NEAR(cfg.r_model_world[k], rz180[k], 1e-12) << k;
  }
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR(cfg.t_model_world[k], 0.0, 1e-12) << k;
  }

  ASSERT_TRUE(RunUntilApproach()) << "the world-frame prediction produced no plan";
  const PlanSnapshot followed = ctrl_->GetFollowedPlanForTesting();
  const Eigen::Vector3d p_c(followed.p_c[0], followed.p_c[1], followed.p_c[2]);
  const Eigen::Vector3d off = p_c - p0_model_;
  const Eigen::Vector3d dir = vel_model_.normalized();
  EXPECT_LT((off - off.dot(dir) * dir).norm(), 1e-9) << "p_c is not on the MODEL-world line";
}

TEST_F(CatchingVisionFrameTest, WithoutTheFrameKeysTheSamePredictionIsBehindTheArm) {
  // The premise of the case above, and the S6-C sim symptom: the same
  // world-frame numbers read as model coordinates put every candidate on the
  // far side of the robot, and no plan is ever adopted.
  SetUpWorldFrameBall();
  ASSERT_NO_FATAL_FAILURE(BringUp(false, true));
  EXPECT_FALSE(ctrl_->GetTrajInputConfig().to_model);
  EXPECT_FALSE(RunUntilApproach());
}

TEST_F(CatchingVisionFrameTest, AFrameTheModelLacksRefusesTheConfiguration) {
  ctrl_ = std::make_unique<DemoCatchingController>("");
  ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
  ctrl_->SetSharedModelBuilder(builder_);
  ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
  YAML::Node yaml = YAML::Load(
      TrackingYaml(topic_, Eigen::Vector3d(0.5, 0.2, 0.4), Eigen::Vector3d::UnitZ(), 0.0, 1.0));
  yaml["catching"]["io"]["arm_base_frame"] = "no_such_frame";
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl_->on_configure(prev, node_, yaml),
            DemoCatchingController::CallbackReturn::FAILURE);
}

// ── The thread lives for one configuration (/code-review 2026-09-23) ────────

TEST(CatchingPlannerLifecycle, ACleanupJoinsThePlannerAndAnOracleReconfigureHasNoSecondWriter) {
  // A thread that survived on_cleanup used to be RESUMED by the next
  // activation whatever the new configuration said — with the oracle enabled
  // instead, the plan box then had two writers, and interleaved SeqLock
  // stores can leave its sequence odd and hang the RT tick's Load forever.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_planner_reconf");
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(integrated_bringup::testfx::PlannerSimDevices());
  const rclcpp_lifecycle::State prev;
  using integrated_bringup::testfx::PlannerMinimalYaml;

  ASSERT_EQ(ctrl.on_configure(prev, node, YAML::Load(PlannerMinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  const auto* first = ctrl.GetPlannerThread();
  ASSERT_NE(first, nullptr);
  ASSERT_EQ(ctrl.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.GetPlannerThread(), nullptr) << "the thread outlived its configuration";

  // Oracle only: the RT is the box's one writer, and no planner may resume.
  ASSERT_EQ(ctrl.on_configure(prev, node, YAML::Load(PlannerMinimalYaml(false, true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_FALSE(ctrl.IsSimOnlyDisabled());
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.GetPlannerThread(), nullptr) << "a planner runs beside the oracle";
  ASSERT_EQ(ctrl.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);

  // And back to the planner: a FRESH thread under the new configuration.
  ASSERT_EQ(ctrl.on_configure(prev, node, YAML::Load(PlannerMinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_NE(ctrl.GetPlannerThread(), nullptr);
  EXPECT_TRUE(ctrl.GetPlannerThread()->Running());
  EXPECT_FALSE(ctrl.GetPlannerThread()->Paused());
  ASSERT_EQ(ctrl.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

}  // namespace
