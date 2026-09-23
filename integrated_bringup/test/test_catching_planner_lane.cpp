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
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace {

using integrated_bringup::CatchingPlannerThread;
using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
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
  CatchingPlannerThread thread(rig->cycle, rig->fd, /*wake_timeout_s=*/0.5, rig->timing);
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
  CatchingPlannerThread thread(rig->cycle, rig->fd, 0.5, rig->timing);
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
  CatchingPlannerThread thread(rig->cycle, rig->fd, /*wake_timeout_s=*/0.01, rig->timing);
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
  CatchingPlannerThread thread(rig->cycle, rig->fd, 0.5, rig->timing);
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
  auto thread = std::make_unique<CatchingPlannerThread>(rig->cycle, rig->fd, 0.5, rig->timing);
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
  CatchingPlannerThread thread(rig->cycle, rig->fd, 0.005, rig->timing);
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

// ── The lane ────────────────────────────────────────────────────────────────

class CatchingPlanLaneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_plan_lane_" +
                                                              std::to_string(++counter_));
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(MakeConfigWithCatchFrame());
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
  void BringUp(bool oracle, bool planner = false) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    YAML::Node yaml = YAML::Load(
        TrackingYaml(topic_, Eigen::Vector3d(0.5, 0.2, 0.4), Eigen::Vector3d::UnitZ(), 0.0, 1.0));
    yaml["diagnostic"]["oracle_plan"]["enabled"] = oracle;
    yaml["catching"]["planner"]["enabled"] = planner;
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
    spec.n = 8;
    spec.sequence = sequence;
    spec.generation = generation;
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
    return ctrl_->Compute(state_);
  }

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

TEST_F(CatchingPlanLaneTest, WithThePlannerEnabledTheStubKeepsTrackingOnNoCatchablePlan) {
  // Wiring the thread changes no behaviour at S6-A: the stub publishes
  // "no plan", the RT refuses it as invalid, and TRACKING self-loops exactly as
  // it did before a planner existed.
  ASSERT_NO_FATAL_FAILURE(BringUp(/*oracle=*/false, /*planner=*/true));
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

  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(thread->Paused());
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
