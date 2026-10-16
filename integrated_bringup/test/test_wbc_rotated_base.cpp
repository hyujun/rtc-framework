// ── DemoWbcController on a base turned against the world (#779) ──────────────
//
// UR's root_link "base" is Rz(π) from base_link, so on ur5e_p1b every task
// quantity in the base points the other way in the world in x and y. Two
// suites:
//
//  1. LocalToAlignedFeedForward (se3_feedforward.hpp) against finite
//     differences of a TaskSpaceTrajectory: the trajectory reports LOCAL
//     motions, SE3Task takes origin velocity and classical acceleration in the
//     base's axes.
//  2. The real controller on the shipped ur5e_p1b config, closed loop: a task
//     goal 3 cm along +x of the base moves the TCP along +x of the base, and
//     GRASP reaches kClosure — the approach → closure guard compares the TCP
//     with the goal in the base (ComputeTcpError).
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/controllers/wbc/se3_feedforward.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#ifndef RTC_WBC_CONFIG_DIR
#error "RTC_WBC_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::DemoWbcController;
using integrated_bringup::WbcPhase;
using rtc::ControllerOutput;
using rtc::ControllerState;
namespace fx = integrated_bringup::testfx;

// ── 1. Feed-forward axes ────────────────────────────────────────────────────

TEST(WbcSe3FeedForward, MatchesFiniteDifferencesOfTheTrajectory) {
  // A segment that turns about two axes while it moves, so v_b, ω_b and the
  // ω_b × v_b term are all non-zero mid-way.
  const pinocchio::SE3 start(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                             Eigen::Vector3d(0.3, -0.2, 0.5));
  const pinocchio::SE3 goal((Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()))
                                .toRotationMatrix(),
                            Eigen::Vector3d(0.6, 0.1, 0.2));
  rtc::trajectory::TaskSpaceTrajectory traj;
  traj.initialize(start, pinocchio::Motion::Zero(), goal, pinocchio::Motion::Zero(), 1.0);

  constexpr double kH = 1e-5;
  for (const double t : {0.3, 0.5, 0.7}) {
    const auto s = traj.compute(t, kH);
    const auto sm = traj.compute(t - kH, kH);
    const auto sp = traj.compute(t + kH, kH);
    const auto ff =
        integrated_bringup::wbc::LocalToAlignedFeedForward(s.pose, s.velocity, s.acceleration);

    // Origin velocity / acceleration by central differences of p(t).
    const Eigen::Vector3d p_dot = (sp.pose.translation() - sm.pose.translation()) / (2.0 * kH);
    const Eigen::Vector3d p_ddot =
        (sp.pose.translation() - 2.0 * s.pose.translation() + sm.pose.translation()) / (kH * kH);
    // Angular velocity in the pose's frame of reference, and its derivative.
    auto omega = [](const pinocchio::SE3& a, const pinocchio::SE3& b, double dt) {
      return Eigen::Vector3d(a.rotation() *
                             pinocchio::log3(a.rotation().transpose() * b.rotation()) / dt);
    };
    const Eigen::Vector3d w = omega(sm.pose, sp.pose, 2.0 * kH);
    const auto smm = traj.compute(t - 2.0 * kH, kH);
    const auto spp = traj.compute(t + 2.0 * kH, kH);
    const Eigen::Vector3d w_dot =
        (omega(s.pose, spp.pose, 2.0 * kH) - omega(smm.pose, s.pose, 2.0 * kH)) / (2.0 * kH);

    ASSERT_GT(s.velocity.linear().norm(), 1e-2) << t;
    ASSERT_GT(s.velocity.angular().cross(s.velocity.linear()).norm(), 1e-3) << t;
    EXPECT_LT((ff.velocity.head<3>() - p_dot).norm(), 1e-6) << t;
    EXPECT_LT((ff.velocity.tail<3>() - w).norm(), 1e-6) << t;
    EXPECT_LT((ff.acceleration.head<3>() - p_ddot).norm(), 1e-3) << t;
    EXPECT_LT((ff.acceleration.tail<3>() - w_dot).norm(), 1e-3) << t;
  }
}

// ── 2. The controller on ur5e_p1b ───────────────────────────────────────────

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
