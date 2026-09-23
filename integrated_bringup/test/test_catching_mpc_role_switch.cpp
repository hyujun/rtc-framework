// ── R-1: the catching planner shares the `mpc` layout role (E-7 decision J) ──
//
// The planner takes DemoWbc's MPC slot and its thread name instead of a layout
// role of its own. That is only safe if a controller switch leaves exactly ONE
// of the two solver threads running on that core — the controller manager
// keeps one controller active and deactivates the previous one, and each
// controller pauses its solver on deactivate. This suite drives that switch
// the way CM does (deactivate the old, then activate the new) in one process
// and reads the answer from the kernel, not from the controllers' own claims:
//
//   * how many threads are named `mpc_main` (/proc/self/task/*/comm),
//   * where each may run (sched_getaffinity) against the role's slot,
//   * which of them is paused, from each thread object,
//   * the scheduling policy, when this host lets the process set it.
//
// `verify_rt_runtime.sh` is NOT a substitute: it keys threads by name and keeps
// one TID per name, so with two `mpc_main` threads it checks only one of them.
//
// Policy on a development host. Setting SCHED_FIFO needs CAP_SYS_NICE /
// RLIMIT_RTPRIO; without it ApplyThreadConfig leaves the thread on SCHED_OTHER
// and carries on (its documented behaviour). The case then RECORDS
// `NOT_EVALUATED(EPERM)` instead of passing silently or skipping — the policy
// half of R-1 is judged on the control PC (S6-D).

#include "catching_planner_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "rtc_base/threading/thread_utils.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <sched.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::DemoWbcController;
using integrated_bringup::testfx::AllowedCpus;
using integrated_bringup::testfx::PlannerMinimalYaml;
using integrated_bringup::testfx::PlannerSimDevices;
using integrated_bringup::testfx::ThreadsNamed;
using integrated_bringup::testfx::WaitForThreadsNamed;
using rtc::RTControllerInterface;

rclcpp_lifecycle::State Inactive() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

class RclcppScope : public ::testing::Environment {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

const ::testing::Environment* const kRclcpp =
    ::testing::AddGlobalTestEnvironment(new RclcppScope);  // NOLINT

/// The DemoWbc MPC path, driven exactly as test_wbc_layout_profile_gate does:
/// shipped config, MPC on, arm-only URDF (the tsid block names hand frames).
std::unique_ptr<DemoWbcController> MakeWbc(std::string_view profile) {
  auto ctrl = std::make_unique<DemoWbcController>(RTC_UR5E_URDF_PATH);
  YAML::Node cfg = YAML::LoadFile(RTC_WBC_CONFIG_PATH)["demo_wbc_controller"];
  cfg.remove("tsid");
  cfg["mpc"]["enabled"] = true;
  ctrl->LoadConfig(cfg);
  ctrl->SetLayoutProfile(profile);
  return ctrl;
}

struct Catching {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::unique_ptr<DemoCatchingController> ctrl;
};

Catching MakeCatching(const std::string& node_name, std::string_view profile) {
  Catching c;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rt_layout_profile", std::string(profile))});
  c.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  c.ctrl = std::make_unique<DemoCatchingController>("");
  c.ctrl->SetDeviceNameConfigs(PlannerSimDevices());
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(c.ctrl->on_configure(prev, c.node, YAML::Load(PlannerMinimalYaml(true))),
            RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(c.ctrl->IsSimOnlyDisabled());
  return c;
}

/// The logical CPU the `mpc` role's slot resolves to on this host, or -1 when
/// the role is not pinned.
int RoleLogicalCpu() {
  const int slot = rtc::SelectThreadConfigs().mpc.main.cpu_core;
  return slot < 0 ? -1 : rtc::SlotToLogicalCpu(slot);
}

/// Affinity verdict for one thread: pinned to exactly the role's CPU. When the
/// role's CPU is not in this process's own mask (a cset shield or a test
/// runner's cpuset), ApplyThreadConfig CANNOT pin there — that is a property
/// of the host, recorded rather than failed.
void ExpectPinnedToRole(pid_t tid, const char* who) {
  const int cpu = RoleLogicalCpu();
  if (cpu < 0) {
    ::testing::Test::RecordProperty(std::string(who) + "_affinity", "NOT_EVALUATED(unpinned role)");
    return;
  }
  const auto process_cpus = AllowedCpus(0);
  if (std::find(process_cpus.begin(), process_cpus.end(), cpu) == process_cpus.end()) {
    ::testing::Test::RecordProperty(std::string(who) + "_affinity",
                                    "NOT_EVALUATED(role cpu outside process cpuset)");
    return;
  }
  EXPECT_EQ(AllowedCpus(tid), std::vector<int>{cpu})
      << who << " (tid " << tid << ") is not pinned to the mpc role's CPU " << cpu;
}

/// Policy verdict: the role's policy and priority when the host allowed the
/// thread to take it, NOT_EVALUATED(EPERM) when it did not.
void ExpectRolePolicy(pid_t tid, const char* who) {
  const auto& role = rtc::SelectThreadConfigs().mpc.main;
  const int policy = sched_getscheduler(tid);
  if (role.sched_policy == SCHED_FIFO && policy != SCHED_FIFO) {
    ::testing::Test::RecordProperty(std::string(who) + "_policy", "NOT_EVALUATED(EPERM)");
    return;
  }
  EXPECT_EQ(policy, role.sched_policy) << who;
  if (policy == SCHED_FIFO) {
    sched_param param{};
    ASSERT_EQ(sched_getparam(tid, &param), 0);
    EXPECT_EQ(param.sched_priority, role.sched_priority) << who;
  }
  ::testing::Test::RecordProperty(std::string(who) + "_policy", "EVALUATED");
}

TEST(CatchingMpcRoleSwitch, TheTwoSolverThreadsShareTheRoleAndOnlyOneRunsAcrossASwitch) {
  const std::string name = rtc::SelectThreadConfigs().mpc.main.name;
  ASSERT_EQ(name, "mpc_main") << "precondition: the role this suite is about";
  ASSERT_TRUE(ThreadsNamed(name).empty()) << "precondition: no mpc_main before the first spawn";

  // ── wbc active: one mpc_main, running ────────────────────────────────────
  auto wbc = MakeWbc(DemoWbcController::kDefaultLayoutProfile);
  ASSERT_NE(wbc->on_activate(Inactive()), RTControllerInterface::CallbackReturn::FAILURE);
  ASSERT_NE(wbc->GetMpcThread(), nullptr);
  const auto after_wbc = WaitForThreadsNamed(name, 1);
  ASSERT_EQ(after_wbc.size(), 1U) << "the MPC thread did not appear as mpc_main";
  const pid_t wbc_tid = after_wbc.front();
  EXPECT_FALSE(wbc->GetMpcThread()->Paused());
  ExpectPinnedToRole(wbc_tid, "wbc_mpc");
  ExpectRolePolicy(wbc_tid, "wbc_mpc");

  // ── switch wbc → catching, in CM's order: deactivate, then activate ──────
  ASSERT_EQ(wbc->on_deactivate(Inactive()), RTControllerInterface::CallbackReturn::SUCCESS);
  auto catching = MakeCatching("r1_switch_catching", DemoWbcController::kDefaultLayoutProfile);
  ASSERT_EQ(catching.ctrl->on_activate(Inactive()), RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_NE(catching.ctrl->GetPlannerThread(), nullptr);
  const auto after_switch = WaitForThreadsNamed(name, 2);
  ASSERT_EQ(after_switch.size(), 2U) << "the planner did not appear as a second mpc_main";
  const pid_t planner_tid =
      after_switch.front() == wbc_tid ? after_switch.back() : after_switch.front();
  ASSERT_NE(planner_tid, wbc_tid);

  EXPECT_TRUE(wbc->GetMpcThread()->Paused()) << "the deactivated controller's solver still runs";
  EXPECT_TRUE(catching.ctrl->GetPlannerThread()->Running());
  EXPECT_FALSE(catching.ctrl->GetPlannerThread()->Paused());
  ExpectPinnedToRole(planner_tid, "planner");
  ExpectRolePolicy(planner_tid, "planner");
  // Same core for both — the premise of reusing the role rather than adding
  // one. Only meaningful when the role is pinned: two unpinned threads share
  // the process mask and would compare equal for no reason.
  if (RoleLogicalCpu() >= 0) {
    EXPECT_EQ(AllowedCpus(planner_tid), AllowedCpus(wbc_tid));
  }

  // ── and back: catching → wbc ──────────────────────────────────────────────
  ASSERT_EQ(catching.ctrl->on_deactivate(Inactive()),
            RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_NE(wbc->on_activate(Inactive()), RTControllerInterface::CallbackReturn::FAILURE);
  EXPECT_TRUE(catching.ctrl->GetPlannerThread()->Paused());
  EXPECT_FALSE(wbc->GetMpcThread()->Paused());
  EXPECT_EQ(ThreadsNamed(name).size(), 2U) << "a switch spawned or lost a solver thread";

  // Teardown order mirrors a process exit: the planner joins in its
  // controller's destructor.
  ASSERT_EQ(wbc->on_deactivate(Inactive()), RTControllerInterface::CallbackReturn::SUCCESS);
  catching.ctrl.reset();
  EXPECT_EQ(WaitForThreadsNamed(name, 1).size(), 1U) << "the planner thread outlived its owner";
}

TEST(CatchingMpcRoleSwitch, UnderTheMpcOffProfileThePlannerNeverSpawns) {
  // Mirrors WbcLayoutProfileGate.RefusesMpcEnabledConfigUnderTheOptOutProfile
  // for the second tenant of the role. The count is taken before and after so
  // a thread left by another case cannot mask a spawn here.
  const std::string name = rtc::SelectThreadConfigs().mpc.main.name;
  const auto before = ThreadsNamed(name).size();
  auto catching = MakeCatching("r1_mpc_off_catching", DemoWbcController::kMpcOffLayoutProfile);
  EXPECT_EQ(catching.ctrl->on_activate(Inactive()), RTControllerInterface::CallbackReturn::FAILURE);
  EXPECT_EQ(catching.ctrl->GetPlannerThread(), nullptr);
  EXPECT_EQ(ThreadsNamed(name).size(), before);
}

}  // namespace
