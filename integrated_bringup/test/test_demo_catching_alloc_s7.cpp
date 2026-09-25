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

#include "catching_cloud_fixture.hpp"
#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
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
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
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

class DemoCatchingAllocS7Test : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_alloc_s7_test");
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(MakeConfigWithCatchFrame());
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
  void BringUp(bool s8c = false) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
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
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
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

  bool stall_in_hold_{false};
  bool freeze_hand_in_retreat_{false};

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string topic_{"/test_catching_alloc_s7/prediction"};
  pinocchio::SE3 start_{pinocchio::SE3::Identity()};
  ControllerState state_{};
};

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
    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
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
    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
  }
  EXPECT_TRUE(blocked_seen) << "the witness never evaluated a stalled joint — not measured";
  EXPECT_TRUE(timed_out) << "RETREAT did not end on the release timeout — not measured";
  // (No fingertip lane in this fixture: the verdict is Undetermined whatever
  // the hand says — the scenario suite owns the verdict; this case, the heap.)
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
    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
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

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
