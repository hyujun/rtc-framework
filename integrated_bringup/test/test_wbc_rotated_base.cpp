// ── DemoWbcController on a base turned against the world (#779) ──────────────
//
// UR's root_link "base" is Rz(π) from base_link, so on ur5e_p1b every task
// quantity in the base points the other way in the world in x and y. The real
// controller on the shipped ur5e_p1b config, closed loop: a task goal 3 cm
// along +x of the base moves the TCP along +x of the base, and GRASP reaches
// kClosure — the approach → closure guard compares the TCP with the goal in
// the base (ComputeTcpError). The ramp feed-forward's axes are judged in
// rtc_controllers' test_task_space_trajectory.
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <memory>
#include <span>
#include <string>

#ifndef RTC_WBC_CONFIG_DIR
#error "RTC_WBC_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::DemoWbcController;
using integrated_bringup::WbcPhase;
using rtc::ControllerOutput;
using rtc::ControllerState;
namespace fx = integrated_bringup::testfx;

// ── The controller on ur5e_p1b ──────────────────────────────────────────────

YAML::Node ShippedWbcConfig(const char* robot) {
  const std::string path =
      std::string(RTC_WBC_CONFIG_DIR) + "/" + robot + "/controllers/demo_wbc_controller.yaml";
  YAML::Node root = YAML::LoadFile(path);
  return root["demo_wbc_controller"];
}

class WbcRotatedBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctrl_ = std::make_unique<DemoWbcController>("");
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("wbc_rotated_base_test");
    ctrl_->SetSystemModelConfig(fx::SharedUr5eP1bModelConfig());
    ctrl_->SetSharedModelBuilder(fx::SharedUr5eP1bBuilder());
    ctrl_->SetControlRate(1.0 / fx::kUr5eDt);
    const YAML::Node cfg = ShippedWbcConfig("ur5e_p1b");
    ASSERT_TRUE(cfg);
    ctrl_->LoadConfig(cfg);
    ctrl_->SetDeviceNameConfigs(fx::MakeUr5eP1bDeviceConfigs());
    ASSERT_EQ(ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg),
              DemoWbcController::CallbackReturn::SUCCESS);
    state_ = fx::MakeUr5eP1bState();
    last_out_ = ctrl_->Compute(state_);
  }

  void TearDown() override {
    ctrl_.reset();
    node_.reset();
  }

  /// Closed loop: the plant tracks the command on both devices.
  void RunTicks(int n) {
    for (int i = 0; i < n; ++i) {
      state_.iteration += 1;
      last_out_ = ctrl_->Compute(state_);
      for (int d = 0; d < 2; ++d) {
        const auto& cmd = last_out_.devices[static_cast<std::size_t>(d)];
        auto& dev = state_.devices[static_cast<std::size_t>(d)];
        if (cmd.num_channels == dev.num_channels) {
          for (int j = 0; j < cmd.num_channels; ++j) {
            dev.positions[static_cast<std::size_t>(j)] = cmd.commands[static_cast<std::size_t>(j)];
          }
        }
      }
    }
  }

  [[nodiscard]] Eigen::Vector3d TcpInBase() const {
    return {last_out_.actual_task_positions[0], last_out_.actual_task_positions[1],
            last_out_.actual_task_positions[2]};
  }

  std::unique_ptr<DemoWbcController> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  ControllerState state_{};
  ControllerOutput last_out_{};
};

TEST_F(WbcRotatedBaseTest, ATaskGoalAlongTheBaseXMovesThereAndGraspCloses) {
  RunTicks(300);  // closed-chain walk-in, idle hold
  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kIdle);
  ASSERT_TRUE(last_out_.arm_tip_pose_valid);
  const Eigen::Vector3d start = TcpInBase();

  std::array<double, 6> task6{};
  for (std::size_t i = 0; i < task6.size(); ++i) {
    task6[i] = last_out_.actual_task_positions[i];
  }
  const Eigen::Vector3d offset(0.03, 0.0, 0.0);
  task6[0] += offset.x();
  // The operator sequence of the sim check: a joint goal (here the measured
  // arm, so it moves nothing — kIdle → kApproach needs one), then the task
  // goal, which kIdle already tracks, then GRASP.
  std::array<double, fx::kUr5eArmDof> joints{};
  for (std::size_t i = 0; i < joints.size(); ++i) {
    joints[i] = state_.devices[0].positions[i];
  }
  ctrl_->SetDeviceTarget(0, std::span<const double>(joints));
  RunTicks(1);
  ctrl_->SetDeviceTaskTarget(0, std::span<const double>(task6));

  // First 50 ms (25 ticks at 500 Hz): the TCP moves along +x of the base.
  RunTicks(25);
  const Eigen::Vector3d moved = TcpInBase() - start;
  ASSERT_GT(moved.norm(), 1e-5) << "the TCP has to move for its direction to be judged";
  const double cos = moved.dot(offset) / (moved.norm() * offset.norm());
  RecordProperty("cos_first_50ms", std::to_string(cos));
  EXPECT_GT(cos, 0.9) << "moved " << moved.transpose();

  // GRASP: approach ramps to the commanded pose and ends in closure once the
  // TCP is within epsilon_pregrasp (5 mm) of the goal IN THE BASE.
  ctrl_->SetGraspCmdForTesting(1);
  bool closed = false;
  for (int k = 0; k < 3000 && !closed; ++k) {
    RunTicks(1);
    closed = ctrl_->GetPhaseForTesting() == WbcPhase::kClosure;
  }
  EXPECT_TRUE(closed) << "phase " << static_cast<int>(ctrl_->GetPhaseForTesting())
                      << ", TCP-goal (base) " << (TcpInBase() - (start + offset)).norm();
  EXPECT_LT((TcpInBase() - (start + offset)).norm(), 0.006);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
