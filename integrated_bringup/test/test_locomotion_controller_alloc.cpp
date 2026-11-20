// locomotion_controller's RT tick allocates nothing on any of its paths:
// nominal (the QP), degraded (a short source drop), fault (damping), the reset
// and E-STOP services (the hold's waiting ticks, the clear's guard refusals,
// the first passing tick), and a re-latch; with the MPC, the stand on its plan
// and the fall back to the latched pose; with the inekf source, its start,
// hold, restart and timeout in the tick. Own binary: the gate replaces the global
// `operator new` (alloc_gate.hpp contract item 1).
//
// WHAT THIS GATE SEES: global `operator new` across translation units, so the
// QP solver's and pinocchio's heap use counts too. It does not see Eigen's
// aligned_malloc, which none of the fixed-size types here reach.
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <sensor_msgs/msg/joy.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <thread>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig

class LocomotionAlloc : public LocomotionControllerTest {
 protected:
  // Ticks one call under the gate; the stamps are written before it.
  std::size_t GatedTick(bool stale) {
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = stale ? now - 1'000'000'000 : now;
    state_.devices[kBody].base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    rtc::testing::ScopedAllocGate gate;
    static_cast<void>(ctrl_->Compute(state_));
    return gate.count();
  }

  // Gated ticks at about the control rate (the MPC's plans and their age are
  // wall time) until `done` or 1000 ticks; the most any tick allocated goes
  // into `worst`.
  void GatedTickUntil(const std::function<bool()>& done, std::size_t& worst) {
    for (int k = 0; k < 1000 && !done(); ++k) {
      worst = std::max(worst, GatedTick(false));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
};

TEST_F(LocomotionAlloc, EveryTickPathAllocatesNothing) {
  BringUp();
  Tick();  // warm the solver and the latches outside the gate
  Tick();
  std::size_t worst = 0;
  for (int k = 0; k < 50; ++k) {  // nominal: the QP
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n + 3; ++k) {  // degraded, then the latch and damping
    worst = std::max(worst, GatedTick(true));
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  ctrl_->ResetFault();                        // off-RT hook, outside the gate
  worst = std::max(worst, GatedTick(false));  // the reset and the re-latch
  ASSERT_EQ(LastMode(), Mode::kNominal);
  ctrl_->TriggerEstop();
  for (int k = 0; k < 10; ++k) {  // the hold: waiting, models updated, logs written
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  // The clear onto a pitched base: the guard refuses, the controller waits;
  // then the base comes up and the first passing tick latches.
  auto& pose = state_.devices[kBody].base_pose;
  pose[4] = std::sin(0.5 * 0.7);
  pose[6] = std::cos(0.5 * 0.7);
  ctrl_->ClearEstop();
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  ASSERT_NE(ctrl_->LastDiag().pose_guard_mask, 0U);
  pose[4] = 0.0;
  pose[6] = 1.0;
  worst = std::max(worst, GatedTick(false));  // the guard passes: the latch
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(worst, 0U);
}

// The MPC path: the state write, the sample, the plausibility check, the plan
// copy and the contact mask, refused plans answered by the plan before them,
// then the plan lost and the pose re-latched. The
// plans come from a stub on the MPC thread (the gate counts the ticking thread
// only).
TEST_F(LocomotionAlloc, TheMpcPathAllocatesNothing) {
  Configure(MpcConfig(), rclcpp::NodeOptions{}, "test_locomotion_mpc_alloc");
  StandingStub* const s = SpawnStandingStub(*ctrl_);
  ASSERT_NE(s, nullptr);
  Activate();
  MakeStanding();
  // Until the first stand, outside the gate (the plan arrives from another thread).
  ASSERT_TRUE(TickUntil(
      [this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kStand; }));
  ASSERT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kStand);
  std::size_t worst = 0;
  for (int k = 0; k < 30; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kStand);
  s->joint_offset = 1e3;  // every new plan refused: held, then aged out (still: re-latched)
  GatedTickUntil(
      [this] {
        return ctrl_->LastDiag().plan_holds > 0 &&
               ctrl_->Source() == LocomotionController::ReferenceSource::kIdle;
      },
      worst);
  EXPECT_GT(ctrl_->LastDiag().plan_holds, 0U);
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kIdle);
  s->joint_offset = 0.0;
  ASSERT_TRUE(TickUntil(
      [this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kStand; }));
  s->fail = true;  // the plan ages out (max_policy_age_s): the fall back and the re-latch
  GatedTickUntil([this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kIdle; },
                 worst);
  for (int k = 0; k < 20; ++k) {  // the re-latched pose, still under the gate
    worst = std::max(worst, GatedTick(false));
  }
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kIdle);
  EXPECT_EQ(worst, 0U);
}

// The inekf source in the tick: the start on the activation tick, nominal
// steps, a held then stale IMU (degraded, the latch), the reset and the
// re-latch, a restart of the estimate after the latch (the epoch fault), the
// E-STOP's waiting ticks, and an estimate that cannot start (the timeout).
TEST_F(LocomotionAlloc, TheInekfSourcePathsAllocateNothing) {
  YAML::Node cfg = YAML::Clone(ShippedConfig());
  cfg["estimator"]["source"] = "inekf";
  Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_inekf_alloc");
  Activate();
  MakeStanding();
  // Every lane stamped now; the IMU sequence advanced by k (0 = the sample repeated).
  const auto gated = [this](uint64_t k) {
    const int64_t now = NowNs();
    state_.iteration += 1;
    auto& body = state_.devices[kBody];
    body.joint_recv_steady_ns = now;
    body.base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    for (int site = 0; site < body.num_imu_sites; ++site) {
      body.imu_sequence[static_cast<std::size_t>(site)] += k;
      body.imu_recv_steady_ns[static_cast<std::size_t>(site)] = now;
    }
    rtc::testing::ScopedAllocGate gate;
    static_cast<void>(ctrl_->Compute(state_));
    return gate.count();
  };
  std::size_t worst = gated(1);  // the activation tick: the start and the latch
  ASSERT_EQ(LastMode(), Mode::kNominal);
  for (int k = 0; k < 50; ++k) {
    worst = std::max(worst, gated(1));
  }
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k < n + 6; ++k) {  // held, stale, degraded, the latch
    worst = std::max(worst, gated(0));
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  ctrl_->ResetFault();
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, gated(1));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  worst = std::max(worst, gated(11));  // a gap: the estimate restarts, the epoch fault
  ASSERT_EQ(LastMode(), Mode::kFault);
  ctrl_->ResetFault();
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, gated(1));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  ctrl_->TriggerEstop();
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, gated(1));
  }
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  state_.devices[kBody].imu[0][0] = 0.1;  // shaking: a restart cannot start again
  worst = std::max(worst, gated(11));
  ctrl_->ClearEstop();
  for (int k = 0; k < 300 && !ctrl_->HasLatchedFault(); ++k) {  // the start timeout
    worst = std::max(worst, gated(1));
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  EXPECT_EQ(worst, 0U);
}

// The T-tier log on the tick: bound handle, the moment lane, both soles and
// the base fresh, then each lane stale — the fill and the push allocate
// nothing (the file opens on the drain timer, off the tick).
TEST_F(LocomotionAlloc, TheTruthLogPathAllocatesNothing) {
  const YAML::Node cfg = ShippedConfig();
  ConfigureWithDevices(cfg, ShippedLaneDevices(cfg), rclcpp::NodeOptions{},
                       "test_locomotion_truth_alloc");
  ASSERT_FALSE(integrated_bringup::LocomotionControllerTestAccess::LogPath(
                   *ctrl_, integrated_bringup::kLocomotionTruthLogInstance)
                   .empty());
  Activate();
  MakeStanding();
  auto& body = state_.devices[kBody];
  body.num_inference_groups = 2;
  const auto stamp_contacts = [&body](int64_t at) {
    for (std::size_t c = 0; c < 2; ++c) {
      body.inference_enable[c] = true;
      body.inference_recv_steady_ns[c] = at;
      body.inference_data[c * 7 + 3] = -170.0F;
      body.inference_data[c * 7 + 5] = 1.0F;
    }
  };
  stamp_contacts(NowNs());
  Tick();  // warm outside the gate
  Tick();
  std::size_t worst = 0;
  for (int k = 0; k < 40; ++k) {
    // Fresh soles on even ticks, stale (a second old) on odd ones.
    stamp_contacts(k % 2 == 0 ? NowNs() : NowNs() - 1'000'000'000);
    worst = std::max(worst, GatedTick(false));
    ASSERT_NE(ctrl_->LastTruthLog().truth_mask, 0U);
  }
  EXPECT_EQ(worst, 0U);
}

// The external wrench lane: the subscription callback (the message is built
// before the gate, as the executor hands it over) and the tick that copies it
// into the truth log.
TEST_F(LocomotionAlloc, TheExternalWrenchPathAllocatesNothing) {
  YAML::Node cfg = YAML::Clone(ShippedConfig());
  cfg["external_wrench"]["topic"] = "sim/external_wrench";
  ConfigureWithDevices(cfg, ShippedLaneDevices(cfg), rclcpp::NodeOptions{},
                       "test_locomotion_ext_alloc");
  ASSERT_TRUE(ctrl_->ExternalWrenchSubscribed());
  Activate();
  MakeStanding();
  geometry_msgs::msg::WrenchStamped msg;
  msg.wrench.force.x = 40.0;
  integrated_bringup::LocomotionControllerTestAccess::OnExternalWrench(*ctrl_, msg);
  Tick();  // warm outside the gate
  Tick();
  std::size_t worst = 0;
  for (int k = 0; k < 40; ++k) {
    {
      rtc::testing::ScopedAllocGate gate;
      integrated_bringup::LocomotionControllerTestAccess::OnExternalWrench(*ctrl_, msg);
      worst = std::max(worst, gate.count());
    }
    worst = std::max(worst, GatedTick(false));
    ASSERT_NE(
        ctrl_->LastTruthLog().truth_mask & integrated_bringup::LocomotionTruthLogPod::kExternalBit,
        0U);
  }
  EXPECT_EQ(worst, 0U);
}

// velocity_command.type joy: the tick under the Joy configuration, and the Joy
// subscription callback itself (the mapping is a fixed table from configure;
// the message is built before the gate, as the executor hands it over).
TEST_F(LocomotionAlloc, TheJoyCommandPathAllocatesNothing) {
  YAML::Node cfg = MpcConfig();
  SetJoyCommand(cfg);
  Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_joy_alloc");
  ASSERT_TRUE(ctrl_->VelocityCommandSubscribed());
  StandingStub* const s = SpawnStandingStub(*ctrl_);
  ASSERT_NE(s, nullptr);
  Activate();
  MakeStanding();
  ASSERT_TRUE(TickUntil(
      [this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kStand; }));
  sensor_msgs::msg::Joy pressed;
  pressed.axes.assign(6, 0.0F);
  pressed.buttons.assign(21, 0);
  pressed.axes[1] = 0.25F;
  pressed.buttons[9] = 1;
  sensor_msgs::msg::Joy unreadable = pressed;
  unreadable.axes[1] = std::nanf("");
  sensor_msgs::msg::Joy short_one;
  short_one.axes.assign(2, 0.0F);
  std::size_t worst_callback = 0;
  std::size_t worst_tick = 0;
  for (int k = 0; k < 30; ++k) {
    const auto& msg = k % 3 == 0 ? pressed : (k % 3 == 1 ? unreadable : short_one);
    {
      rtc::testing::ScopedAllocGate gate;
      integrated_bringup::LocomotionControllerTestAccess::OnJoy(*ctrl_, msg);
      worst_callback = std::max(worst_callback, gate.count());
    }
    worst_tick = std::max(worst_tick, GatedTick(false));
  }
  EXPECT_GT(
      integrated_bringup::LocomotionControllerTestAccess::RawCommand(*ctrl_).source.joy_rejects,
      0U);
  EXPECT_EQ(worst_callback, 0U);
  EXPECT_EQ(worst_tick, 0U);
}

TEST(LocomotionAllocGate, PositiveControlTheGateIsArmed) {
  rtc::testing::ScopedAllocGate gate;
  std::vector<double> v;
  v.reserve(64);
  EXPECT_GT(gate.count(), 0U) << "the gate is inert — every zero above proves nothing";
}

}  // namespace
