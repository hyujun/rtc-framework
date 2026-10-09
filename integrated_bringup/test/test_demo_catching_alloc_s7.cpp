// ── DemoCatchingController RT-1 zero-allocation gate, ARMED modes (S7, G7-D) ──
//
// test_demo_catching_alloc.cpp covers the disarmed hold and step paths on a
// fixture with no model. This binary covers what S7 added to the tick: the
// joint-space homing, the tracking law, the freeze, the hand sequencer, the
// decelerating target, the hold, the return and the stop — one gated tick in
// EVERY mode a trial passes through, on the real UR5e+P1b model.
//
// Its own binary because alloc_gate.hpp installs a replacement global
// `operator new` — one TU per binary (alloc_gate.hpp contract item 1).
//
// WHAT THIS GATE SEES. Global `operator new`, across translation units, so it
// covers the real Compute() in the integrated_bringup library. It does NOT see
// Eigen's aligned_malloc (the CLIK's own allocation-freedom is gated in
// rtc_tsid); a positive control shows the gate is armed. The per-tick worst
// execution time is recorded alongside (G7-D asks for both).
//
// The predictions are published OUTSIDE the gate — DDS allocates on publish
// (a known, accepted RT exception that is not this tick's) — and so is the
// executor spin that delivers them.
//
// TIME IS STEPPED where the case allows: the controller reads
// FakeSteadyClock (SetClockForTesting), advanced one control period per tick
// instead of slept through. The fault-latch case keeps the real steady clock,
// so the production clock read stays inside a gated Compute().

#include "catching_cloud_fixture.hpp"
#include "catching_segment_fixture.hpp"
#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "shipped_catching_bringup_fixture.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::FakeSteadyClock;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::SharedCatchFrameBuilder;
using integrated_bringup::testfx::TrackingYaml;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::catching::Mode;

using namespace std::chrono_literals;

TEST(DemoCatchingAllocS7, TheGateSeesAnAllocation) {
  // Positive control: an allocation inside the gate is counted.
  rtc::testing::ScopedAllocGate gate;
  auto* leak = new int(7);  // NOLINT(cppcoreguidelines-owning-memory)
  EXPECT_GT(gate.count(), 0U);
  delete leak;  // NOLINT(cppcoreguidelines-owning-memory)
}

/// A ball that is `vel` at `p_ref` on the steady axis' `t_ref_ns` (E1-F17: the
/// close re-timing needs one that crosses the catch frame's close plane).
struct BallLine {
  std::array<double, 3> p_ref{};
  std::int64_t t_ref_ns{0};
  std::array<double, 3> vel{};
};

class DemoCatchingAllocS7Test : public ::testing::Test {
 protected:
  void SetUp() override {
    FakeSteadyClock::Restart();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_alloc_s7_test");
    builder_ = SharedCatchFrameBuilder();
    CatchFrameOracle oracle(*builder_);
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    start_ = oracle.PoseAt(
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names, home,
        kUr5eArmDof);
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

  /// `s8c`: the hand-joint capture witness on (the hand device given its
  /// max_torque) and a short RETREAT release timeout (#537 S8-C).
  void BringUp(bool s8c = false, const std::function<void(YAML::Node&)>& tweak = nullptr) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    if (!real_clock_) {
      ctrl_->SetClockForTesting(&FakeSteadyClock::Now);
    }
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
    if (s8c) {
      rtc::DeviceJointLimits limits;
      limits.max_torque = std::vector<double>(kP1bHandDof, 3.0);
      configs.at("p1b").joint_limits = limits;
    }
    ctrl_->SetDeviceNameConfigs(configs);
    const Eigen::Vector3d p_c = start_.translation() + Eigen::Vector3d(0.04, 0.03, 0.02);
    YAML::Node yaml = YAML::Load(
        TrackingYaml(topic_, p_c, start_.rotation().col(2), /*gamma_f=*/0.3, /*t_c=*/0.7));
    // A short hold, so RETREAT is reached inside a short run.
    yaml["catching"]["robot"]["hand"]["T_hold"] = 0.02;
    if (s8c) {
      YAML::Node hand = yaml["catching"]["robot"]["hand"];
      hand["T_hold"] = 0.2;
      hand["T_release_timeout"] = 0.35;
      hand["capture"]["rho_min"] = 0.2;
      hand["capture"]["rho_max"] = 0.9;
      hand["capture"]["effort_frac_min"] = 0.5;
      hand["capture"]["t_persist"] = 0.05;
      hand["capture"]["provisional"] = false;  // this fixture is judged a real-arm config
    }
    if (tweak) {
      tweak(yaml);
    }
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_TRUE(ctrl_->AreTrialsEnabled());
    ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl_->GetPlannerThread(), nullptr)
        << "precondition: the planner wakes on the real clock (SetClockForTesting)";
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    state_ = ControllerState{};
    state_.num_devices = 2;
    state_.dt = kDt;
    auto& a = state_.devices[0];
    a.num_channels = kUr5eArmDof;
    a.valid = true;
    for (int i = 0; i < kUr5eArmDof; ++i) {
      // Off the wait pose by more than pose_tol (0.02), so the first armed
      // ticks HOME.
      a.positions[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)] + 0.05;
    }
    auto& h = state_.devices[1];
    h.num_channels = kP1bHandDof;
    h.valid = true;
  }

  void Publish(std::uint64_t sequence) {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = 16;
    spec.sequence = sequence;
    spec.generation = 42;
    const Eigen::Vector3d p = start_.translation();
    spec.p0 = {p.x(), p.y(), p.z()};
    spec.vel = {0.0, 0.0, 0.0};
    if (ball_) {
      // A ball on a straight line (see ball_), as the origin stamp the
      // message will carry puts it.
      spec.n = 20;
      const double since =
          static_cast<double>(FakeSteadyClock::Now() - 5'000'000 - ball_->t_ref_ns) * 1e-9;
      for (std::size_t i = 0; i < 3; ++i) {
        spec.p0[i] = ball_->p_ref[i] + ball_->vel[i] * since;
        spec.vel[i] = ball_->vel[i];
      }
    }
    auto msg = integrated_bringup::testing::MakeCloud(spec);
    const std::int64_t wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count() -
                              5'000'000;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
    for (int i = 0; i < 8; ++i) {
      executor_->spin_some(2ms);
    }
  }

  /// One tick with a perfect servo on both devices. `gated` wraps ONLY the
  /// Compute() call. With `stall_in_hold_`, a hand joint stops part-way in
  /// Hold and pushes; with `freeze_hand_in_retreat_`, the hand stops following
  /// its command from RETREAT on (the release never arrives).
  void Tick(bool gated, std::uint64_t& allocations, double& tick_us) {
    state_.iteration += 1;
    state_.t_relative_s = static_cast<double>(state_.iteration) * kDt;
    ControllerOutput out;
    const auto t0 = std::chrono::steady_clock::now();
    if (gated) {
      rtc::testing::ScopedAllocGate gate;
      out = ctrl_->Compute(state_);
      allocations = gate.count();
    } else {
      out = ctrl_->Compute(state_);
      allocations = 0;
    }
    tick_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    const bool hand_frozen = freeze_hand_in_retreat_ && ctrl_->GetMode() == Mode::kRetreat;
    for (int d = 0; d < 2; ++d) {
      const auto& o = out.devices[static_cast<std::size_t>(d)];
      auto& dev = state_.devices[static_cast<std::size_t>(d)];
      if (d == 1 && hand_frozen) {
        continue;
      }
      for (int i = 0; i < o.num_channels; ++i) {
        dev.positions[static_cast<std::size_t>(i)] = o.commands[static_cast<std::size_t>(i)];
      }
    }
    if (stall_in_hold_) {
      const auto& h = ctrl_->GetHandOutputForTesting();
      const bool holding = h.active && h.phase == rtc::catching::HandPhase::kHold;
      auto& hand = state_.devices[1];
      if (holding) {
        hand.positions[5] = 0.3;  // ρ 0.6 of the fixture's 0 → 0.5 travel
      }
      hand.efforts[5] = holding ? 2.7 : 0.0;  // 0.9 of max_torque, toward q_close
    }
  }

  /// One control period: stepped on the fake clock, slept on the real one.
  void Advance() {
    if (real_clock_) {
      std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
    } else {
      FakeSteadyClock::Step();
    }
  }

  /// Unset: the fixture's ball at rest at the hand's start pose.
  std::optional<BallLine> ball_;

  bool stall_in_hold_{false};
  bool freeze_hand_in_retreat_{false};
  /// Set by a case before BringUp reads it.
  bool real_clock_{false};

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string topic_{"/test_catching_alloc_s7/prediction"};
  pinocchio::SE3 start_{pinocchio::SE3::Identity()};
  ControllerState state_{};
};

/// What the replacement-and-M2 gate needs of a robot's bring-up (the two
/// fixtures below fill it in): the controller, its node, the independent FK of
/// the catch frame, and the fixture's own publish / gated tick / clock step.
struct GatedRig {
  DemoCatchingController* ctrl{nullptr};
  rclcpp_lifecycle::LifecycleNode* node{nullptr};
  CatchFrameOracle* oracle{nullptr};
  std::vector<std::string> arm_names;
  int arm_dof{0};
  std::function<void(std::uint64_t)> publish;
  std::function<void(std::uint64_t&, double&)> tick;  // one GATED Compute
  std::function<void()> advance;
  std::function<void(const BallLine&)> set_ball;
};

void RunGatedReplacementCase(const GatedRig& rig, const std::string& segment_mode) {
  // E1-F17 (2f, 3d): TRACKING → pair → following → a replacement pair (the
  // segment box, then the plan box — MD-56) → its admission tick, the waiting
  // ticks, the plan-switch tick → COMMITTED, where a ball that crosses the
  // close plane after t_c makes RetimeClose solve on every new snapshot and
  // move the close → HOLD. Every Compute is gated. The box writes (this test
  // plays the planner) are outside it.
  constexpr double kTcOffsetS = 0.7;  // the oracle's t_c − now
  constexpr double kBump = 0.08;
  constexpr std::int64_t kShiftNs = 20'000'000;  // the replacement's t_c − the first plan's
  constexpr std::int64_t kCrossNs = 25'000'000;  // the ball's crossing, after the new t_c
  using integrated_bringup::testfx::kApproachDtPreNs;
  using integrated_bringup::testfx::kApproachNPre;
  using Event = integrated_bringup::CatchingDiagLogPod::SegmentEvent;
  const std::int64_t lead_ns =
      std::llround(rig.node->get_parameter("hand.T_close_lead_from_t_c").as_double() * 1e9);
  const auto stamp = [&rig](rtc::catching::SegmentSnapshot& seg, std::uint32_t seq) {
    seg.token.activation_generation = rig.ctrl->GetPlannerRtState().activation_generation;
    seg.token.generation = 42;
    seg.publish_ns = FakeSteadyClock::Now();
    seg.rt_state_ns = FakeSteadyClock::Now() - integrated_bringup::testfx::kDtNs;
    seg.segment_seq = seq;
  };
  const auto first_segment = [&rig](std::uint32_t plan_id, std::int64_t t_c, double bump) {
    const rtc::catching::PlannerRtState rt = rig.ctrl->GetPlannerRtState();
    std::array<double, rtc::catching::kMaxSegmentNv> q{};
    const std::array<double, rtc::catching::kMaxSegmentNv> rest{};
    for (int i = 0; i < rig.arm_dof; ++i) {
      q[static_cast<std::size_t>(i)] = rt.q_cmd[static_cast<std::size_t>(i)];
    }
    return integrated_bringup::testfx::MakeApproachSegment(
        q, rest, rig.arm_dof, plan_id, t_c, kApproachNPre, t_c - kApproachNPre * kApproachDtPreNs,
        bump);
  };
  // The catch frame on a segment at an instant: the ball's line is put on the
  // plane s = 0 of the hand there, 1 m/s down its z axis.
  const auto ball_through_hand = [&](const rtc::catching::SegmentSnapshot& seg, std::int64_t t_ns) {
    std::array<double, rtc::catching::kMaxSegmentNv> q{};
    std::array<double, rtc::catching::kMaxSegmentNv> qd{};
    std::array<double, rtc::catching::kMaxSegmentNv> qdd{};
    EXPECT_TRUE(rtc::catching::NodeTrajectoryFollower::SampleJoints(seg, t_ns, q, qd, qdd));
    std::array<double, 64> wide{};
    std::copy(q.begin(), q.begin() + rig.arm_dof, wide.begin());
    const pinocchio::SE3 pose = rig.oracle->PoseAt(rig.arm_names, wide, rig.arm_dof);
    const Eigen::Vector3d v = -1.0 * pose.rotation().col(2);
    BallLine b;
    b.t_ref_ns = t_ns;
    for (std::size_t k = 0; k < 3; ++k) {
      b.p_ref[k] = pose.translation()[static_cast<long>(k)];
      b.vel[k] = v[static_cast<long>(k)];
    }
    return b;
  };

  rtc::catching::SegmentSnapshot base{};
  rtc::catching::PlanSnapshot new_plan{};
  rtc::catching::SegmentSnapshot replacement{};
  bool replacement_written = false;
  std::int64_t t_c_new = 0;
  std::int64_t instant_first = 0;  // the first COMMITTED tick's
  std::int64_t instant_last = 0;
  int retimes = 0;
  std::set<std::uint32_t> events;
  std::set<Mode> followed_in;
  std::uint64_t sequence = 1;
  Mode prev = rig.ctrl->GetMode();
  bool saw_switch_after_pair = false;
  double us_pair = 0.0;
  double us_wait = 0.0;
  double us_switch = 0.0;
  double us_retime = 0.0;
  double us_worst = 0.0;
  for (int t = 0; t < 4000; ++t) {
    const Mode mode = rig.ctrl->GetMode();
    if (t % 15 == 0 &&
        (mode == Mode::kIdle || mode == Mode::kArmed || mode == Mode::kTracking ||
         mode == Mode::kApproach || mode == Mode::kCommitted || mode == Mode::kClosing)) {
      rig.publish(sequence++);
    }
    if (mode == Mode::kTracking) {
      base = first_segment(rig.ctrl->GetPublishedPlan().plan_id + 1,
                           FakeSteadyClock::Now() + static_cast<std::int64_t>(kTcOffsetS * 1e9),
                           kBump);
      stamp(base, 1);
      rig.ctrl->SegmentBoxForTesting().Store(base);
    } else if (!replacement_written && rig.ctrl->IsFollowingSegmentForTesting()) {
      // The replacement pair: the next plan, its catch instant a little later,
      // and its first segment from the command the RT holds (at rest — the arm
      // is still in the old segment's first interval).
      new_plan = rig.ctrl->GetFollowedPlanForTesting();
      new_plan.plan_id = rig.ctrl->GetPublishedPlan().plan_id + 1;
      new_plan.t_c_ns += kShiftNs;
      new_plan.t_cmd_ns = new_plan.t_c_ns;
      new_plan.gamma_t1_ns = new_plan.t_c_ns;
      new_plan.publish_ns = FakeSteadyClock::Now();
      t_c_new = new_plan.t_c_ns;
      replacement = first_segment(new_plan.plan_id, t_c_new, kBump);
      stamp(replacement, 2);
      rig.ctrl->SegmentBoxForTesting().Store(replacement);
      rig.ctrl->PlanBoxForTesting().Store(new_plan);
      // The ball that will cross the close plane 25 ms after the new t_c.
      rig.set_ball(ball_through_hand(replacement, t_c_new + kCrossNs));
      replacement_written = true;
    }
    std::uint64_t allocations = 0;
    double us = 0.0;
    rig.tick(allocations, us);
    EXPECT_EQ(allocations, 0U) << "mode " << static_cast<int>(mode) << " (the tick's START mode)";
    const auto record = rig.ctrl->GetLastTickRecord();
    events.insert(static_cast<std::uint32_t>(record.segment_event));
    const bool segment_tick = mode == Mode::kTracking || mode == Mode::kApproach ||
                              mode == Mode::kCommitted || mode == Mode::kClosing ||
                              mode == Mode::kDecel || mode == Mode::kHold;
    if (segment_tick) {
      us_worst = std::max(us_worst, us);
    }
    if (record.segment_event == Event::kPairAdmitted) {
      us_pair = std::max(us_pair, us);
    } else if (record.segment_event == Event::kPlanSwitched) {
      us_switch = std::max(us_switch, us);
      saw_switch_after_pair = true;
    } else if (mode == Mode::kApproach && rig.ctrl->HasPendingPlanForTesting()) {
      us_wait = std::max(us_wait, us);
    }
    if (rig.ctrl->GetMode() == Mode::kCommitted) {
      const std::int64_t instant = rig.ctrl->GetCommittedCloseInstantNsForTesting();
      if (instant_first == 0) {
        instant_first = instant;
      }
      if (instant != instant_last && instant_last != 0) {
        ++retimes;
        us_retime = std::max(us_retime, us);
      }
      instant_last = instant;
    }
    if (record.segment_following) {
      followed_in.insert(mode);
    }
    if (rig.ctrl->GetMode() == Mode::kRetreat && prev == Mode::kHold) {
      break;
    }
    prev = rig.ctrl->GetMode();
    rig.advance();
  }
  EXPECT_TRUE(replacement_written) << "the first segment was never followed";
  EXPECT_TRUE(events.count(static_cast<std::uint32_t>(Event::kPairAdmitted)) == 1)
      << "the replacement pair was never admitted on a gated tick";
  EXPECT_TRUE(saw_switch_after_pair) << "the plan switch was never gated";
  EXPECT_EQ(rig.ctrl->GetPlanReplacedCount(), 1U);
  // The gate proves nothing about M2 unless the solve ran and moved the close:
  // the committed instant started at t_c − lead and ended near the ball's
  // crossing less the lead (under mpc_docking the entrance plane is crossed
  // earlier than the catch frame's origin).
  EXPECT_NE(instant_first, 0);
  EXPECT_GE(retimes, 1) << "the close instant never moved: RetimeClose never solved";
  EXPECT_GT(std::abs(instant_last - (t_c_new - lead_ns)), 4'000'000)
      << "the close moved by less than two ticks";
  // … and stayed inside the solve's window (± one pre-catch interval of the
  // committed t_c). Where exactly depends on the robot's entrance plane: the
  // crossing of the plane s_ent moves with it (the leap hand's is 49 mm out).
  EXPECT_LT(std::abs(instant_last - (t_c_new - lead_ns)), kApproachDtPreNs);
  for (const Mode m :
       {Mode::kApproach, Mode::kCommitted, Mode::kClosing, Mode::kDecel, Mode::kHold}) {
    EXPECT_EQ(followed_in.count(m), 1U)
        << "no gated tick followed a segment in mode " << static_cast<int>(m);
  }
  std::printf(
      "[ MEASURED ] %s replacement + M2 worst tick [us]: pair admission %.1f, wait %.1f, "
      "plan switch %.1f, retimed %.1f, any %.1f; close moved %.2f ms (%d re-times)\n",
      segment_mode.c_str(), us_pair, us_wait, us_switch, us_retime, us_worst,
      static_cast<double>(instant_last - (t_c_new - lead_ns)) * 1e-6, retimes);
  ::testing::Test::RecordProperty("worst_us_pair_admission", static_cast<int>(us_pair));
  ::testing::Test::RecordProperty("worst_us_plan_switch", static_cast<int>(us_switch));
  ::testing::Test::RecordProperty("worst_us_retimed_tick", static_cast<int>(us_retime));
  ::testing::Test::RecordProperty("worst_us_any_tick", static_cast<int>(us_worst));
}

TEST_F(DemoCatchingAllocS7Test, EveryModeOfATrialTicksWithoutAllocating) {
  ASSERT_NO_FATAL_FAILURE(BringUp());
  // Every tick is gated, and a mode counts as measured once a tick STARTED
  // in it. (ARMED lasts a single tick when a ball is already in view, so
  // "two ticks per mode" would demand a trial shape the fixture does not
  // have.) Edge actions — the freeze, DECEL entry, RETREAT's release — run on
  // the tick that LEAVES a mode, which is gated too.
  std::set<Mode> measured;
  std::vector<int> path;
  std::vector<int> path_reason;
  double worst_us = 0.0;
  std::uint64_t sequence = 1;
  Mode prev = ctrl_->GetMode();
  for (int t = 0; t < 4000 && measured.size() < 10; ++t) {
    // Publish on a fixed cadence while the ball is in flight — up to t_c, i.e.
    // through CLOSING. A lane that went quiet at the freeze would age past
    // io.t_stale + stale_committed_max_s before t_c on this plan (COMMITTED +
    // CLOSING is 0.36 s) and end the trial on BALL_STALE_LONG, which is the
    // controller being right, not this case measuring anything.
    const Mode mode = ctrl_->GetMode();
    if (t % 15 == 0 &&
        (mode == Mode::kIdle || mode == Mode::kArmed || mode == Mode::kTracking ||
         mode == Mode::kApproach || mode == Mode::kCommitted || mode == Mode::kClosing)) {
      Publish(sequence++);
    }
    std::uint64_t allocations = 0;
    double us = 0.0;
    Tick(/*gated=*/true, allocations, us);
    worst_us = std::max(worst_us, us);
    EXPECT_EQ(allocations, 0U) << "mode " << static_cast<int>(mode) << " (the tick's START mode)";
    // Count gated ticks by the mode the tick started in.
    measured.insert(mode);
    if (path.empty() || path.back() != static_cast<int>(ctrl_->GetMode())) {
      path.push_back(static_cast<int>(ctrl_->GetMode()));
      path_reason.push_back(static_cast<int>(ctrl_->GetLastReason()));
    }
    if (ctrl_->GetMode() == Mode::kArmed && prev == Mode::kRetreat) {
      break;  // one full trial
    }
    prev = ctrl_->GetMode();
    Advance();
  }
  // The trial passed through every mode of the nominal cycle.
  for (const Mode m :
       {Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
        Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat}) {
    std::string seen;
    for (std::size_t k = 0; k < path.size(); ++k) {
      seen += std::to_string(path[k]) + "(" + std::to_string(path_reason[k]) + ") ";
    }
    EXPECT_TRUE(measured.count(m) == 1)
        << "mode " << static_cast<int>(m) << " was never gated; path: " << seen;
  }
  RecordProperty("worst_tick_us", static_cast<int>(worst_us));
}

TEST_F(DemoCatchingAllocS7Test, TheHandWitnessAndTheReleaseTimeoutTickWithoutAllocating) {
  // #537 S8-C: the same trial with the hand-joint witness evaluated on every
  // Hold tick (a stalled, pushing finger — the full per-joint evaluation, not
  // its early exit) and a hand that never comes back to q_pre, so RETREAT ends
  // on the release timeout. Every tick is gated.
  ASSERT_NO_FATAL_FAILURE(BringUp(/*s8c=*/true));
  stall_in_hold_ = true;
  freeze_hand_in_retreat_ = true;
  std::uint64_t sequence = 1;
  bool blocked_seen = false;
  bool timed_out = false;
  Mode prev = ctrl_->GetMode();
  for (int t = 0; t < 5000; ++t) {
    const Mode mode = ctrl_->GetMode();
    if (t % 15 == 0 &&
        (mode == Mode::kIdle || mode == Mode::kArmed || mode == Mode::kTracking ||
         mode == Mode::kApproach || mode == Mode::kCommitted || mode == Mode::kClosing)) {
      Publish(sequence++);
    }
    std::uint64_t allocations = 0;
    double us = 0.0;
    Tick(/*gated=*/true, allocations, us);
    EXPECT_EQ(allocations, 0U) << "mode " << static_cast<int>(mode) << " (the tick's START mode)";
    blocked_seen =
        blocked_seen || (mode == Mode::kHold && ctrl_->GetLastTickRecord().hand_blocked_s > 0.0);
    if (prev == Mode::kRetreat && ctrl_->GetMode() == Mode::kIdle) {
      timed_out = ctrl_->GetLastReason() == rtc::catching::Reason::kHandTimeout;
      break;
    }
    prev = ctrl_->GetMode();
    Advance();
  }
  EXPECT_TRUE(blocked_seen) << "the witness never evaluated a stalled joint — not measured";
  EXPECT_TRUE(timed_out) << "RETREAT did not end on the release timeout — not measured";
  // (No fingertip lane in this fixture: the verdict is Undetermined whatever
  // the hand says — the scenario suite owns the verdict; this case, the heap.)
}

TEST_F(DemoCatchingAllocS7Test, TheFaultLatchAndBothFaultResetAnswersTickWithoutAllocating) {
  // #537 S9b: n_qp = 1 and a degenerate axis latch the fault on the first law
  // tick; FAULT then runs, a reset is refused (the measured arm moves) and one
  // is accepted. Every tick is gated. On the real clock (file header).
  real_clock_ = true;
  ASSERT_NO_FATAL_FAILURE(BringUp(false, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["a_d"] = YAML::Load("[0.0, 0.0, 0.0]");
    y["catching"]["supervisor"]["n_qp"] = 1;
  }));
  std::uint64_t sequence = 1;
  std::uint64_t allocations = 0;
  double us = 0.0;
  bool faulted = false;
  for (int t = 0; t < 1500 && !faulted; ++t) {
    if (t % 15 == 0) {
      Publish(sequence++);
    }
    Tick(/*gated=*/true, allocations, us);
    EXPECT_EQ(allocations, 0U) << "mode " << static_cast<int>(ctrl_->GetMode());
    faulted = ctrl_->GetMode() == Mode::kFault;
    Advance();
  }
  ASSERT_TRUE(faulted) << "precondition: the fault never latched";
  state_.devices[0].velocities[0] = 0.5;  // the measured arm moves: refused
  ctrl_->ResetFault();
  Tick(/*gated=*/true, allocations, us);
  EXPECT_EQ(allocations, 0U) << "the refused reset";
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  ASSERT_EQ(ctrl_->GetFaultResetRefusedCount(), 1U) << "precondition: the refusal path ran";
  state_.devices[0].velocities[0] = 0.0;
  ctrl_->ResetFault();
  Tick(/*gated=*/true, allocations, us);
  EXPECT_EQ(allocations, 0U) << "the accepted reset";
  EXPECT_FALSE(ctrl_->HasLatchedFault()) << "precondition: the accept path ran";
}

TEST_F(DemoCatchingAllocS7Test, TheJointSpaceStopTicksWithoutAllocating) {
  // ABORT_SAFE: a degenerate approach axis makes CLIK refuse, and the stop
  // runs on the carried command without the QP.
  ASSERT_NO_FATAL_FAILURE(BringUp());
  bool approaching = false;
  std::uint64_t sequence = 1;
  for (int t = 0; t < 400 && !approaching; ++t) {
    if (t % 15 == 0) {
      Publish(sequence++);
    }
    std::uint64_t allocations = 0;
    double us = 0.0;
    Tick(/*gated=*/false, allocations, us);
    Advance();
    approaching = ctrl_->GetMode() == Mode::kApproach;
  }
  ASSERT_TRUE(approaching) << "precondition: the approach never started";
  // Displace the measured arm far enough to trip TRACK_ERR → ABORT_SAFE.
  for (int i = 0; i < kUr5eArmDof; ++i) {
    state_.devices[0].positions[static_cast<std::size_t>(i)] += 1.0;
  }
  std::uint64_t allocations = 0;
  double us = 0.0;
  Tick(/*gated=*/true, allocations, us);
  EXPECT_EQ(allocations, 0U);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kAbortSafe)
      << "reason " << static_cast<int>(ctrl_->GetLastReason());
  Tick(/*gated=*/true, allocations, us);  // the ramp's steady tick
  EXPECT_EQ(allocations, 0U);
}

/// The segment-following tick under each segment planner (E1-F17): the RT's
/// lane is one code path for `mpc` and `mpc_docking`, so the gate is run on
/// both.
class DemoCatchingAllocS7SegmentTest : public DemoCatchingAllocS7Test,
                                       public ::testing::WithParamInterface<const char*> {};

TEST_P(DemoCatchingAllocS7SegmentTest, TheMpcTicksFromThePairToTheHoldWithoutAllocating) {
  // MPC E1-F09 (planner.segment.mode mpc; E1-F17: and mpc_docking): the segment lane's Load and
  // judge in TRACKING and from APPROACH on, the stop part's node-wise catch-box FK, the pair's
  // adoption, the wait before node 0, a same-node-0 replacement, the first switch and two replan
  // switches with their gates, the segment sample and the posture feedforward in every mode that
  // follows — every tick gated. The box writes (this test plays the planner) are outside the gate.
  constexpr double kTcOffsetS = 0.7;  // BringUp's oracle t_c − now
  constexpr double kBump = 0.08;      // rad/s: 1.6 rad/s² at the stop, inside the D-16 box
  using integrated_bringup::testfx::kApproachDtPreNs;
  using integrated_bringup::testfx::kApproachNPre;
  using Event = integrated_bringup::CatchingDiagLogPod::SegmentEvent;
  const std::string segment_mode = GetParam();
  ASSERT_NO_FATAL_FAILURE(BringUp(false, [&segment_mode](YAML::Node& y) {
    if (segment_mode == "mpc_docking") {
      // The shipped docking design the RT's lane reads (the close lead, the
      // switch margin, eta_v); the planner itself does not run here.
      integrated_bringup::testfx::ApplyShippedDocking(y);
    }
    y["catching"]["planner"]["segment"]["mode"] = segment_mode;
    y["catching"]["planner"]["sub_model"] = "ur5e_catch";
    y["catching"]["planner"]["search"]["grid"]["workspace"]["catch_box"]["min"] =
        std::vector<double>{-2.0, -2.0, -2.0};
    y["catching"]["planner"]["search"]["grid"]["workspace"]["catch_box"]["max"] =
        std::vector<double>{2.0, 2.0, 2.0};
  }));
  const auto stamp = [this](rtc::catching::SegmentSnapshot& seg, std::uint32_t seq) {
    seg.token.activation_generation = ctrl_->GetPlannerRtState().activation_generation;
    seg.token.generation = 42;  // Publish()'s track
    seg.publish_ns = FakeSteadyClock::Now();
    seg.rt_state_ns = FakeSteadyClock::Now() - integrated_bringup::testfx::kDtNs;
    seg.segment_seq = seq;
  };
  // The first segment of plan (id, t_c) from the pose the RT reports, at rest.
  const auto first_segment = [this](std::uint32_t plan_id, std::int64_t t_c, double bump) {
    const rtc::catching::PlannerRtState rt = ctrl_->GetPlannerRtState();
    std::array<double, kUr5eArmDof> q{};
    const std::array<double, kUr5eArmDof> rest{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      q[static_cast<std::size_t>(i)] = rt.q_cmd[static_cast<std::size_t>(i)];
    }
    return integrated_bringup::testfx::MakeApproachSegment(
        q, rest, kUr5eArmDof, plan_id, t_c, kApproachNPre, t_c - kApproachNPre * kApproachDtPreNs,
        bump);
  };
  rtc::catching::SegmentSnapshot base{};  // the trajectory the replans are cut from
  int adopted_at = -1;
  bool replacement_written = false;
  bool pre_replan_written = false;
  bool post_replan_written = false;
  bool replaced = false;
  std::set<std::uint32_t> switched;
  std::set<Mode> followed_in;
  std::uint64_t sequence = 1;
  Mode prev = ctrl_->GetMode();
  // Reported, not judged: the worst tick of each kind on this host. The v1
  // trial above records its own worst tick (worst_tick_us).
  double us_pair = 0.0;    // the tick that took the plan and its segment (+ the stop part's FK)
  double us_wait = 0.0;    // APPROACH ticks holding the command before node 0
  double us_admit = 0.0;   // ticks that admitted or replaced a segment (+ the FK, MD-43)
  double us_switch = 0.0;  // ticks that switched segments (sample + gate)
  double us_follow = 0.0;  // other ticks that followed a segment
  double us_worst = 0.0;   // every mpc tick from TRACKING to the end of HOLD
  for (int t = 0; t < 4000; ++t) {
    const Mode mode = ctrl_->GetMode();
    if (t % 15 == 0 &&
        (mode == Mode::kIdle || mode == Mode::kArmed || mode == Mode::kTracking ||
         mode == Mode::kApproach || mode == Mode::kCommitted || mode == Mode::kClosing)) {
      Publish(sequence++);
    }
    if (mode == Mode::kTracking) {
      // The pair: the first segment of the plan the oracle stores on this tick.
      base = first_segment(ctrl_->GetPublishedPlan().plan_id + 1,
                           FakeSteadyClock::Now() + static_cast<std::int64_t>(kTcOffsetS * 1e9),
                           kBump);
      stamp(base, 1);
      ctrl_->SegmentBoxForTesting().Store(base);
    } else if (adopted_at >= 0 && !replacement_written && t == adopted_at + 5) {
      // The same node 0 solved again (MD-58): replaces the waiting segment.
      base = first_segment(base.plan_id, base.t_c_ns, 0.9 * kBump);
      stamp(base, 2);
      ctrl_->SegmentBoxForTesting().Store(base);
      replacement_written = true;
    } else if (ctrl_->IsFollowingSegmentForTesting() && !pre_replan_written) {
      rtc::catching::SegmentSnapshot replan =
          integrated_bringup::testfx::ShiftSegment(base, kApproachNPre - 1);
      stamp(replan, 3);
      ctrl_->SegmentBoxForTesting().Store(replan);
      pre_replan_written = true;
    } else if (mode == Mode::kDecel && !post_replan_written) {
      rtc::catching::SegmentSnapshot replan =
          integrated_bringup::testfx::ShiftSegment(base, kApproachNPre + 1);
      stamp(replan, 4);
      ctrl_->SegmentBoxForTesting().Store(replan);
      post_replan_written = true;
    }
    std::uint64_t allocations = 0;
    double us = 0.0;
    Tick(/*gated=*/true, allocations, us);
    EXPECT_EQ(allocations, 0U) << "mode " << static_cast<int>(mode) << " (the tick's START mode)";
    const auto record = ctrl_->GetLastTickRecord();
    const bool mpc_tick = mode == Mode::kTracking || mode == Mode::kApproach ||
                          mode == Mode::kCommitted || mode == Mode::kClosing ||
                          mode == Mode::kDecel || mode == Mode::kHold;
    if (mpc_tick) {
      us_worst = std::max(us_worst, us);
    }
    if (mode == Mode::kTracking && ctrl_->GetMode() == Mode::kApproach) {
      adopted_at = t;
      us_pair = std::max(us_pair, us);
      EXPECT_EQ(record.segment_event, Event::kAdmitted);
    } else if (record.segment_event == Event::kAdmitted ||
               record.segment_event == Event::kReplaced) {
      us_admit = std::max(us_admit, us);
      replaced = replaced || record.segment_event == Event::kReplaced;
    } else if (record.segment_event == Event::kSwitched) {
      us_switch = std::max(us_switch, us);
      switched.insert(record.segment_seq);
    } else if (record.segment_following) {
      us_follow = std::max(us_follow, us);
    } else if (mode == Mode::kApproach && ctrl_->HasPendingSegmentForTesting()) {
      us_wait = std::max(us_wait, us);
    }
    if (record.segment_following) {
      followed_in.insert(mode);
    }
    if (ctrl_->GetMode() == Mode::kRetreat && prev == Mode::kHold) {
      break;
    }
    prev = ctrl_->GetMode();
    Advance();
  }
  EXPECT_GE(adopted_at, 0) << "the pair was never taken";
  EXPECT_TRUE(replaced) << "the same-node-0 replacement was never gated";
  EXPECT_EQ(switched, (std::set<std::uint32_t>{2U, 3U, 4U}))
      << "the first switch and the two replan switches were not all gated";
  for (const Mode m :
       {Mode::kApproach, Mode::kCommitted, Mode::kClosing, Mode::kDecel, Mode::kHold}) {
    EXPECT_EQ(followed_in.count(m), 1U)
        << "no gated tick followed a segment in mode " << static_cast<int>(m);
  }
  std::printf(
      "[ MEASURED ] %s worst tick [us]: pair %.1f, wait %.1f, admission %.1f, switch %.1f, "
      "follow %.1f, any %.1f\n",
      segment_mode.c_str(), us_pair, us_wait, us_admit, us_switch, us_follow, us_worst);
  RecordProperty("worst_us_pair", static_cast<int>(us_pair));
  RecordProperty("worst_us_wait", static_cast<int>(us_wait));
  RecordProperty("worst_us_admission", static_cast<int>(us_admit));
  RecordProperty("worst_us_switch", static_cast<int>(us_switch));
  RecordProperty("worst_us_follow", static_cast<int>(us_follow));
  RecordProperty("worst_us_mpc_tick", static_cast<int>(us_worst));
}

TEST_P(DemoCatchingAllocS7SegmentTest,
       TheReplacementPairTheSwitchAndTheRetimedCloseTickWithoutAllocating) {
  const std::string segment_mode = GetParam();
  ASSERT_NO_FATAL_FAILURE(BringUp(false, [&segment_mode](YAML::Node& y) {
    if (segment_mode == "mpc_docking") {
      integrated_bringup::testfx::ApplyShippedDocking(y);
    }
    y["catching"]["planner"]["segment"]["mode"] = segment_mode;
    y["catching"]["planner"]["sub_model"] = "ur5e_catch";
    y["catching"]["planner"]["search"]["grid"]["workspace"]["catch_box"]["min"] =
        std::vector<double>{-2.0, -2.0, -2.0};
    y["catching"]["planner"]["search"]["grid"]["workspace"]["catch_box"]["max"] =
        std::vector<double>{2.0, 2.0, 2.0};
  }));
  CatchFrameOracle oracle(*builder_);
  GatedRig rig;
  rig.ctrl = ctrl_.get();
  rig.node = node_.get();
  rig.oracle = &oracle;
  rig.arm_names =
      integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names;
  rig.arm_dof = kUr5eArmDof;
  rig.publish = [this](std::uint64_t seq) { Publish(seq); };
  rig.tick = [this](std::uint64_t& allocations, double& us) {
    Tick(/*gated=*/true, allocations, us);
  };
  rig.advance = [this] { Advance(); };
  rig.set_ball = [this](const BallLine& b) { ball_ = b; };
  RunGatedReplacementCase(rig, segment_mode);
}

INSTANTIATE_TEST_SUITE_P(SegmentPlanners, DemoCatchingAllocS7SegmentTest,
                         ::testing::Values("mpc", "mpc_docking"),
                         [](const ::testing::TestParamInfo<const char*>& info) {
                           return std::string(info.param) == "mpc_docking" ? "MpcDocking" : "Mpc";
                         });

/// The second robot of the replacement-and-M2 gate (decision D7): the shipped
/// iiwa7_leap profile on the sim axis, brought up as the CM brings it up
/// (shipped_catching_bringup_fixture.hpp), with the planner off and the oracle
/// plan on — the oracle writes the plan of a catch point at the hand's start
/// pose, this test the segment box. A 7-joint arm, a 16-joint hand.
class DemoCatchingAllocS7LeapTest : public ::testing::TestWithParam<const char*> {
 protected:
  static constexpr const char* kProfile = "iiwa7_leap";

  void SetUp() override {
    FakeSteadyClock::Restart();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_alloc_s7_leap_test");
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

  void BringUp(const std::string& segment_mode) {
    YAML::Node yaml =
        integrated_bringup::testfx::ShippedControllerNode(kProfile, "demo_catching_controller");
    auto configs = integrated_bringup::testfx::ShippedSimConfigs(kProfile, yaml);
    ASSERT_EQ(configs.size(), 2U);
    arm_names_ = configs.at("iiwa7").joint_state_names;
    ASSERT_EQ(arm_names_.size(), 7U);
    const YAML::Node planner = yaml["catching"]["planner"];
    wait_pose_ = planner["wait_pose"].as<std::vector<double>>();
    const YAML::Node hand = yaml["catching"]["robot"]["hand"];
    q_pre_ = hand["q_pre"].as<std::vector<double>>();
    builder_ = integrated_bringup::testfx::ShippedModelBuilder(kProfile);
    oracle_ = std::make_unique<CatchFrameOracle>(*builder_);
    std::array<double, 64> wait{};
    for (std::size_t i = 0; i < wait_pose_.size(); ++i) {
      wait[i] = wait_pose_[i];
    }
    start_ = oracle_->PoseAt(arm_names_, wait, 7);
    // The catch point is where the hand already is, the approach axis its own.
    const Eigen::Vector3d p_c = start_.translation();
    const Eigen::Vector3d a_d = start_.rotation().col(2);
    YAML::Node oracle_plan = yaml["diagnostic"]["oracle_plan"];
    oracle_plan["enabled"] = true;
    oracle_plan["p_c"] = std::vector<double>{p_c.x(), p_c.y(), p_c.z()};
    oracle_plan["a_d"] = std::vector<double>{a_d.x(), a_d.y(), a_d.z()};
    oracle_plan["t_c_offset_s"] = 0.7;
    oracle_plan["gamma_f"] = 0.3;
    yaml["catching"]["planner"]["enabled"] = false;
    yaml["catching"]["planner"]["segment"]["mode"] = segment_mode;
    yaml["catching"]["io"]["traj_topic"] = topic_;
    // This test writes its ball in the MODEL frame (the catch frame's own FK),
    // so the vision frame has to be that frame: the shipped base_T_world is the
    // shipped SCENE's (the arm on a pedestal), and through it the ball this
    // test aims at the hand would arrive that much lower and never cross the
    // close plane.
    yaml["catching"]["io"]["base_T_world"]["translation"] = std::vector<double>{0.0, 0.0, 0.0};
    yaml["catching"]["planner"]["search"]["grid"]["workspace"]["catch_box"]["min"] =
        std::vector<double>{-3.0, -3.0, -3.0};
    yaml["catching"]["planner"]["search"]["grid"]["workspace"]["catch_box"]["max"] =
        std::vector<double>{3.0, 3.0, 3.0};

    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetClockForTesting(&FakeSteadyClock::Now);
    integrated_bringup::testfx::BringUpShipped(*ctrl_, kProfile, configs);
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_FALSE(ctrl_->IsSimOnlyDisabled())
        << "parked: reason " << static_cast<int>(ctrl_->GetParkReason());
    ASSERT_TRUE(ctrl_->AreTrialsEnabled());
    ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    state_ = ControllerState{};
    state_.num_devices = 2;
    state_.dt = kDt;
    auto& a = state_.devices[0];
    a.num_channels = 7;
    a.valid = true;
    for (std::size_t i = 0; i < wait_pose_.size(); ++i) {
      a.positions[i] = wait_pose_[i];  // at the wait pose: no homing
    }
    auto& h = state_.devices[1];
    h.num_channels = static_cast<int>(q_pre_.size());
    h.valid = true;
    for (std::size_t i = 0; i < q_pre_.size(); ++i) {
      h.positions[i] = q_pre_[i];  // at q_pre, at rest
    }
  }

  void Publish(std::uint64_t sequence) {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = 16;
    spec.sequence = sequence;
    spec.generation = 42;
    const Eigen::Vector3d p = start_.translation();
    spec.p0 = {p.x(), p.y(), p.z()};
    spec.vel = {0.0, 0.0, 0.0};
    if (ball_) {
      const double since =
          static_cast<double>(FakeSteadyClock::Now() - 5'000'000 - ball_->t_ref_ns) * 1e-9;
      for (std::size_t i = 0; i < 3; ++i) {
        spec.p0[i] = ball_->p_ref[i] + ball_->vel[i] * since;
        spec.vel[i] = ball_->vel[i];
      }
    }
    auto msg = integrated_bringup::testing::MakeCloud(spec);
    const std::int64_t wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count() -
                              5'000'000;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
    for (int i = 0; i < 8; ++i) {
      executor_->spin_some(2ms);
    }
  }

  /// One GATED tick with a perfect servo on both devices.
  void Tick(std::uint64_t& allocations, double& tick_us) {
    state_.iteration += 1;
    state_.t_relative_s = static_cast<double>(state_.iteration) * kDt;
    ControllerOutput out;
    const auto t0 = std::chrono::steady_clock::now();
    {
      rtc::testing::ScopedAllocGate gate;
      out = ctrl_->Compute(state_);
      allocations = gate.count();
    }
    tick_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    for (int d = 0; d < 2; ++d) {
      const auto& o = out.devices[static_cast<std::size_t>(d)];
      auto& dev = state_.devices[static_cast<std::size_t>(d)];
      for (int i = 0; i < o.num_channels; ++i) {
        dev.positions[static_cast<std::size_t>(i)] = o.commands[static_cast<std::size_t>(i)];
      }
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<CatchFrameOracle> oracle_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string topic_{"/test_catching_alloc_s7_leap/prediction"};
  std::vector<std::string> arm_names_;
  std::vector<double> wait_pose_;
  std::vector<double> q_pre_;
  pinocchio::SE3 start_{pinocchio::SE3::Identity()};
  std::optional<BallLine> ball_;
  ControllerState state_{};
};

TEST_P(DemoCatchingAllocS7LeapTest,
       TheReplacementPairTheSwitchAndTheRetimedCloseTickWithoutAllocatingOnTheLeapArm) {
  const std::string segment_mode = GetParam();
  ASSERT_NO_FATAL_FAILURE(BringUp(segment_mode));
  GatedRig rig;
  rig.ctrl = ctrl_.get();
  rig.node = node_.get();
  rig.oracle = oracle_.get();
  rig.arm_names = arm_names_;
  rig.arm_dof = 7;
  rig.publish = [this](std::uint64_t seq) { Publish(seq); };
  rig.tick = [this](std::uint64_t& allocations, double& us) { Tick(allocations, us); };
  rig.advance = [] { FakeSteadyClock::Step(); };
  rig.set_ball = [this](const BallLine& b) { ball_ = b; };
  RunGatedReplacementCase(rig, segment_mode);
}

INSTANTIATE_TEST_SUITE_P(SegmentPlanners, DemoCatchingAllocS7LeapTest,
                         ::testing::Values("mpc", "mpc_docking"),
                         [](const ::testing::TestParamInfo<const char*>& info) {
                           return std::string(info.param) == "mpc_docking" ? "MpcDocking" : "Mpc";
                         });

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
