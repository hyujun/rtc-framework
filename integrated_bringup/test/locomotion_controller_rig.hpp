// Shared bring-up for the locomotion_controller tests: the installed g1_loco
// profile on the CM's own model, a standing robot, ticks with fresh or stale
// lanes. Header-only so each test binary (the alloc gate needs its own) uses
// the same rig.
#ifndef INTEGRATED_BRINGUP_TEST_LOCOMOTION_CONTROLLER_RIG_HPP_
#define INTEGRATED_BRINGUP_TEST_LOCOMOTION_CONTROLLER_RIG_HPP_

#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "shipped_catching_bringup_fixture.hpp"
#include "shipped_config_test_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace loco_rig {

namespace fx = integrated_bringup::testfx;
namespace loco = integrated_bringup::locomotion;
using integrated_bringup::LocomotionController;
using integrated_bringup::LocomotionControllerConfig;
using integrated_bringup::ParseLocomotionControllerConfig;
using Mode = LocomotionController::Mode;
using Cause = LocomotionController::FaultCause;
using CallbackReturn = LocomotionController::CallbackReturn;

constexpr const char* kProfile = "g1_loco";
constexpr const char* kKey = "locomotion_controller";
constexpr int kBody = 0;
constexpr int kHand = 1;

inline int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// The installed configuration with the MPC off: the rig's tests are the
// latched-pose controller's, and an MPC thread would make their outputs depend
// on when its plan arrives. The MPC tests turn it on themselves.
inline YAML::Node ShippedConfig() {
  YAML::Node cfg = fx::ShippedControllerNode(kProfile, kKey);
  if (cfg["mpc"]) {
    cfg["mpc"]["enabled"] = false;
  }
  return cfg;
}

inline std::map<std::string, rtc::DeviceNameConfig> ShippedDevices(const YAML::Node& cfg) {
  auto configs = fx::ShippedSimConfigs(kProfile, cfg);
  for (auto& [group, device] : configs) {
    rtc::DeviceBackendBinding backend;
    backend.type = "mujoco_native";
    device.backend = backend;
  }
  return configs;
}

// The same with the body group's shipped sensor_layout (ShippedDevices carries
// none): the contact lane's stride and its moment flag, read from the profile.
inline std::map<std::string, rtc::DeviceNameConfig> ShippedLaneDevices(const YAML::Node& cfg) {
  auto devices = ShippedDevices(cfg);
  const YAML::Node sl = fx::ShippedSimParams(kProfile)["devices"]["g1"]["sensor_layout"];
  rtc::DeviceSensorLayout layout;
  layout.inference_values_per_group = sl["inference_values_per_group"].as<int>();
  layout.has_native_moment = sl["has_native_moment"].as<bool>();
  devices.at("g1").sensor_layout = layout;
  return devices;
}

// The stand keyframe: demo_joint's safe pose (body roster order).
inline std::vector<double> StandPose() {
  return fx::ShippedControllerNode(kProfile, "demo_joint_controller")["estop"]["arm_safe_position"]
      .as<std::vector<double>>();
}

// `base` spoiled by `spoil` is refused by the config parser (`what` names the case).
// `velocity_command` as the shipped g1_loco joy overlay sets it (type joy on
// g1/joy, left stick v_x / v_y, right stick X yaw, LB deadman).
inline void SetJoyCommand(YAML::Node& cfg) {
  cfg["velocity_command"]["topic"] = "g1/joy";
  cfg["velocity_command"]["type"] = "joy";
  cfg["velocity_command"]["joy"]["linear_x"] = "lefty";
  cfg["velocity_command"]["joy"]["linear_y"] = "leftx";
  cfg["velocity_command"]["joy"]["angular_z"] = "rightx";
  cfg["velocity_command"]["joy"]["deadman"] = "leftshoulder";
}

inline void ExpectRefused(const YAML::Node& base, const std::function<void(YAML::Node&)>& spoil,
                          const char* what) {
  YAML::Node cfg = YAML::Clone(base);
  spoil(cfg);
  EXPECT_THROW(static_cast<void>(ParseLocomotionControllerConfig(cfg)), std::invalid_argument)
      << what;
}

class LocomotionControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // The CM's bring-up order: model + builder + rate, PreConfigure, devices,
  // on_configure (Configure), on_activate.
  void BringUp(YAML::Node cfg = ShippedConfig()) {
    Configure(cfg);
    Activate();
    MakeStanding();
  }

  // A controller with the CM's model, builder and rate, before its YAML.
  static std::unique_ptr<LocomotionController> MakeController() {
    auto ctrl = std::make_unique<LocomotionController>("");
    ctrl->SetSystemModelConfig(fx::ShippedModelConfig(kProfile));
    ctrl->SetSharedModelBuilder(fx::ShippedModelBuilder(kProfile));
    ctrl->SetControlRate(fx::kShippedProfileControlRateHz);
    return ctrl;
  }

  void Configure(YAML::Node cfg, rclcpp::NodeOptions opts = rclcpp::NodeOptions{},
                 const char* node_name = "test_locomotion_controller") {
    ConfigureWithDevices(cfg, ShippedDevices(cfg), opts, node_name);
  }

  // The same with device configs of the test's own (a roster changed from the shipped one).
  void ConfigureWithDevices(YAML::Node cfg,
                            const std::map<std::string, rtc::DeviceNameConfig>& devices,
                            rclcpp::NodeOptions opts = rclcpp::NodeOptions{},
                            const char* node_name = "test_locomotion_controller") {
    cfg_ = cfg;
    devices_ = devices;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, "", opts);
    ctrl_ = MakeController();
    ASSERT_EQ(ctrl_->PreConfigure(node_, cfg_), CallbackReturn::SUCCESS);
    ctrl_->SetDeviceNameConfigs(devices_);
    ASSERT_EQ(ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg_), CallbackReturn::SUCCESS)
        << ctrl_->ConfigError();
  }

  // Ticks at about the control rate until `done` or the timeout (for state
  // another thread produces).
  bool TickUntil(const std::function<bool()>& done,
                 std::chrono::milliseconds timeout = std::chrono::seconds(3)) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      last_out_ = Tick();
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  void TickFor(int n) {
    for (int k = 0; k < n; ++k) {
      last_out_ = Tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void Activate() {
    ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  void Deactivate() {
    ASSERT_EQ(ctrl_->on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  // The robot standing on both soles at the stand pose, at rest, every lane
  // fresh — the IMU sites (model.imu.sites: the pelvis and torso frames) read
  // what a still IMU reads, no rate and the reaction to gravity in their frame.
  void MakeStanding() {
    const pinocchio::Model& full = ctrl_->FullModel();
    const auto& idx = ctrl_->Indices();
    const std::vector<double> pose = StandPose();
    ASSERT_EQ(static_cast<int>(pose.size()), idx.body.num_channels);
    Eigen::VectorXd q = pinocchio::neutral(full);
    for (int c = 0; c < idx.body.num_channels; ++c) {
      q(7 + idx.body.joint[static_cast<std::size_t>(c)]) = pose[static_cast<std::size_t>(c)];
    }
    pinocchio::Data data(full);
    pinocchio::framesForwardKinematics(full, data, q);
    double z_min = std::numeric_limits<double>::infinity();
    for (const char* sole : {"left_sole", "right_sole"}) {
      z_min = std::min(z_min, data.oMf[full.getFrameId(sole)].translation().z());
    }
    state_ = rtc::ControllerState{};
    state_.num_devices = 2;
    state_.dt = 1.0 / fx::kShippedProfileControlRateHz;
    auto& body = state_.devices[kBody];
    body.valid = true;
    body.num_channels = idx.body.num_channels;
    for (std::size_t c = 0; c < pose.size(); ++c) {
      body.positions[c] = pose[c];
    }
    body.base_pose = {0.0, 0.0, -z_min, 0.0, 0.0, 0.0, 1.0};
    body.base_valid = true;
    q.head<3>() = Eigen::Vector3d(0.0, 0.0, -z_min);
    pinocchio::framesForwardKinematics(full, data, q);
    const Eigen::Vector3d up_g(0.0, 0.0, full.gravity.linear().norm());
    body.num_imu_sites = 2;
    const char* sites[] = {"imu_in_pelvis", "imu_in_torso"};
    for (std::size_t s = 0; s < 2; ++s) {
      const Eigen::Vector3d f = data.oMf[full.getFrameId(sites[s])].rotation().transpose() * up_g;
      body.imu[s] = {0.0, 0.0, 0.0, f.x(), f.y(), f.z()};
      body.imu_valid[s] = true;
    }
    auto& hand = state_.devices[kHand];
    hand.valid = true;
    hand.num_channels = idx.hands[0].num_channels;
  }

  // The IMU lane of this tick: every site a new sample stamped `now`.
  void StampImu(int64_t now) {
    auto& body = state_.devices[kBody];
    for (int s = 0; s < body.num_imu_sites; ++s) {
      body.imu_sequence[static_cast<std::size_t>(s)] += 1;
      body.imu_recv_steady_ns[static_cast<std::size_t>(s)] = now;
    }
  }

  // One tick with every lane stamped now.
  rtc::ControllerOutput Tick() {
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = now;
    state_.devices[kBody].base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    StampImu(now);
    return ctrl_->Compute(state_);
  }

  // One tick with the body joint lane a second old (the source fails it).
  rtc::ControllerOutput StaleTick() {
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = now - 1'000'000'000;
    state_.devices[kBody].base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    StampImu(now);
    return ctrl_->Compute(state_);
  }

  Mode LastMode() const { return static_cast<Mode>(ctrl_->LastDiag().mode); }

  // The damping law on the raw body lane, bit for bit: target = the measured
  // q, feedforward = −damping_kd·v clamped to the roster's max_torque (±inf
  // without one), a velocity read as 0 when its lane is unreadable or the
  // value is not finite (so −0.0), the shipped gains on the wire.
  void ExpectDamping(const rtc::ControllerOutput& out) const {
    const auto& body = out.devices[kBody];
    const auto& dev = state_.devices[kBody];
    const auto rt = cfg_["fault"]["damping_kd"].as<std::vector<double>>();
    const auto& limits = devices_.at("g1").joint_limits;
    const int n = static_cast<int>(rt.size());
    const bool v_ok = rtc::IsLaneReadable(dev, rtc::StateLane::kVelocity, n);
    ASSERT_EQ(body.num_channels, n);
    for (std::size_t i = 0; i < rt.size(); ++i) {
      const double lim = limits && !limits->max_torque.empty()
                             ? limits->max_torque[i]
                             : std::numeric_limits<double>::infinity();
      const double v = v_ok && std::isfinite(dev.velocities[i]) ? dev.velocities[i] : 0.0;
      const double ff = std::clamp(-rt[i] * v, -lim, lim);
      EXPECT_EQ(body.commands[i], dev.positions[i]) << i;
      EXPECT_EQ(std::bit_cast<std::uint64_t>(body.feedforward[i]), std::bit_cast<std::uint64_t>(ff))
          << i << ": " << body.feedforward[i] << " vs " << ff;
    }
    ExpectShippedGains(out);
  }

  void ExpectShippedGains(const rtc::ControllerOutput& out) const {
    const auto& body = out.devices[kBody];
    const auto kp = cfg_["pd_feedforward"]["kp"].as<std::vector<double>>();
    const auto kd = cfg_["pd_feedforward"]["kd"].as<std::vector<double>>();
    EXPECT_TRUE(body.has_pd_gains);
    for (std::size_t i = 0; i < kp.size(); ++i) {
      EXPECT_EQ(body.pd_kp[i], kp[i]) << i;
      EXPECT_EQ(body.pd_kd[i], kd[i]) << i;
    }
  }

  YAML::Node cfg_;
  std::map<std::string, rtc::DeviceNameConfig> devices_;  ///< what Configure gave the controller
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<LocomotionController> ctrl_;
  rtc::ControllerState state_{};
  rtc::ControllerOutput last_out_{};
};

}  // namespace loco_rig

#endif  // INTEGRATED_BRINGUP_TEST_LOCOMOTION_CONTROLLER_RIG_HPP_
