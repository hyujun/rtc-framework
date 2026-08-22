// ── DemoComplianceController ≡ DemoTaskController (#469 S2) ──────────────────
//
// Until the §7 admittance law lands (S3), the compliance controller has no
// behaviour of its own: it is DemoTaskController with renamed identifiers, and
// "identical output on identical input" is its ENTIRE specification. This file
// is the only place that specification is checked.
//
// WHY IT HAS TO EXIST AT ALL. Adding a controller to this package breaks
// nothing. Every per-controller suite here — the device-readability gate, the
// gate-closure diagnostic, the activation-generation gate, the momentum-observer
// embedding, the shipped-config lint — enumerates the three shipped controller
// names by hand, so a fourth is not covered, not counted, and not failed. A
// rename that silently changed which YAML key is read, a source file left out of
// CMakeLists, a constant that kept pointing at the sibling's value: all of those
// ship green. The 941 tests that passed when this controller was added executed
// none of its lines.
//
// WHY NOT DUPLICATE THOSE FIVE SUITES INSTEAD. Because equivalence composes with
// them for free. Each of those suites asserts, absolutely, that DemoTaskController
// does the right thing in scenario S; this file asserts that DemoCompliance-
// Controller does the SAME thing in scenario S. The conjunction is the absolute
// property, for every S this file replays — which is why the tick programs below
// are not a happy path but a transcription of those suites' scenarios: a narrow
// device, an invalid device, an E-STOP ramp, a target applied mid-flight, a
// fingertip in contact.
//
// The composition is only valid while the copy is exact, so it expires the
// moment S3 gives this controller a law of its own. AT THAT POINT: keep
// ShippedConfigsAreTheSameConfig for the blocks S3 does not touch, re-scope the
// behavioural half to "equivalent when the wrench source is withheld" (the
// admittance integrator is a no-op on a zero wrench, so that case must stay
// bit-identical), and give the compliance controller its own entries in the five
// suites above for everything else. Deleting this file instead would leave the
// copy unobserved again.
//
// COMPARISON IS BITWISE, not near-equal. Both controllers run the same code on
// the same input, so every double must match to the last bit; a tolerance would
// silently absorb exactly the small divergence a bad rename produces. Bitwise
// also makes NaN == NaN, which matters because "both produce NaN here" is a
// legitimate equivalence and `==` would call it a difference.
// ─────────────────────────────────────────────────────────────────────────────
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::DemoComplianceController;
using integrated_bringup::DemoTaskController;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::DeviceOutput;

using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kHandDof;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::SetFingertipForce;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;

// Tripwire for SameOutput()'s hand-written member list — see the test that
// reads it. Updated together with that list, never on its own.
constexpr std::size_t kExpectedControllerOutputSize = 29656;

// The three shipped profiles. Only iiwa7_leap gets the behavioural half — it is
// the profile the URDF fixture models — so the other two are held by the config
// comparison alone. That asymmetry is deliberate and stated rather than hidden:
// a ur5e-shaped fixture would be a second model harness for a controller that
// does not yet have a law to exercise.
constexpr std::array<const char*, 3> kProfiles = {"iiwa7_leap", "ur5e_p1a", "ur5e_p1b"};

YAML::Node ShippedControllerNode(const std::string& profile, const std::string& key) {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/controllers/" + key + ".yaml";
  const YAML::Node root = YAML::LoadFile(path);
  EXPECT_TRUE(root[key]) << path << ": missing top-level key '" << key << "'";
  return root[key];
}

// Give the controller the profile's shared block.
//
// In production the CM hands it over by declaring `config_variant` on the
// per-controller LifecycleNode, which LoadConfig reads to find
// config/<variant>/controllers/demo_shared.yaml. This fixture builds no node, so
// that lookup falls back to the legacy flat path, finds nothing, and the grasp
// controller, virtual TCP and pull-estimator lanes all stay at their built-in
// defaults — i.e. out of the comparison, which is where the copy is least
// observed. LoadConfig applies the controller node's OWN keys through the same
// ApplyDemoSharedConfig entry point (that is the documented per-controller
// override channel), so merging the shipped block in here delivers exactly what
// the node parameter would have. Existing keys win, since an override is
// precisely what they are.
void MergeShippedShared(YAML::Node& cfg, const std::string& profile) {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/controllers/demo_shared.yaml";
  const YAML::Node shared = YAML::LoadFile(path)["demo_shared"];
  // Throws rather than ASSERT_: an ASSERT_ here would return from THIS function
  // and leave the caller comparing two equally-unmerged controllers, which is
  // green for the wrong reason.
  if (!shared || !shared.IsMap()) {
    throw std::runtime_error(path + ": missing 'demo_shared' map");
  }
  for (auto it = shared.begin(); it != shared.end(); ++it) {
    const auto key = it->first.as<std::string>();
    if (!cfg[key]) {
      cfg[key] = it->second;
    }
  }
}

// ── Bitwise field-by-field output comparison ─────────────────────────────────

bool SameBits(double a, double b) {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

template <std::size_t N>
::testing::AssertionResult SameArray(const std::array<double, N>& a, const std::array<double, N>& b,
                                     const char* what) {
  for (std::size_t i = 0; i < N; ++i) {
    if (!SameBits(a[i], b[i])) {
      return ::testing::AssertionFailure()
             << what << "[" << i << "]: task=" << a[i] << " compliance=" << b[i];
    }
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult SamePose(const rtc::Pose& a, const rtc::Pose& b, const char* what) {
  for (std::size_t i = 0; i < a.position.size(); ++i) {
    if (!SameBits(a.position[i], b.position[i])) {
      return ::testing::AssertionFailure() << what << ".position[" << i << "]";
    }
  }
  for (std::size_t i = 0; i < a.quaternion.size(); ++i) {
    if (!SameBits(a.quaternion[i], b.quaternion[i])) {
      return ::testing::AssertionFailure() << what << ".quaternion[" << i << "]";
    }
  }
  return ::testing::AssertionSuccess();
}

// Every field of DeviceOutput, named. A loop over raw bytes would be shorter and
// would also compare padding, which is uninitialised and free to differ.
::testing::AssertionResult SameDevice(const DeviceOutput& a, const DeviceOutput& b, int d) {
  const std::string p = "devices[" + std::to_string(d) + "].";
  if (a.num_channels != b.num_channels) {
    return ::testing::AssertionFailure()
           << p << "num_channels: task=" << a.num_channels << " compliance=" << b.num_channels;
  }
  if (a.goal_type != b.goal_type) {
    return ::testing::AssertionFailure() << p << "goal_type";
  }
  if (a.command_type != b.command_type) {
    return ::testing::AssertionFailure() << p << "command_type";
  }
  for (const auto& [lhs, rhs, name] :
       {std::tuple{&a.commands, &b.commands, "commands"},
        std::tuple{&a.goal_positions, &b.goal_positions, "goal_positions"},
        std::tuple{&a.target_positions, &b.target_positions, "target_positions"},
        std::tuple{&a.target_velocities, &b.target_velocities, "target_velocities"},
        std::tuple{&a.trajectory_positions, &b.trajectory_positions, "trajectory_positions"},
        std::tuple{&a.trajectory_velocities, &b.trajectory_velocities, "trajectory_velocities"},
        std::tuple{&a.feedforward, &b.feedforward, "feedforward"}}) {
    const auto r = SameArray(*lhs, *rhs, (p + name).c_str());
    if (!r) {
      return r;
    }
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult SameOutput(const ControllerOutput& a, const ControllerOutput& b) {
  if (a.num_devices != b.num_devices) {
    return ::testing::AssertionFailure()
           << "num_devices: task=" << a.num_devices << " compliance=" << b.num_devices;
  }
  if (a.valid != b.valid) {
    return ::testing::AssertionFailure() << "valid: task=" << a.valid << " compliance=" << b.valid;
  }
  if (a.command_type != b.command_type) {
    return ::testing::AssertionFailure() << "command_type";
  }
  // Every slot, not just num_devices of them: a controller that writes a device
  // it does not declare is a divergence worth seeing.
  for (int d = 0; d < ControllerOutput::kMaxDevices; ++d) {
    const auto r = SameDevice(a.devices[static_cast<std::size_t>(d)],
                              b.devices[static_cast<std::size_t>(d)], d);
    if (!r) {
      return r;
    }
  }
  for (const auto& [lhs, rhs, name] :
       {std::tuple{&a.actual_task_positions, &b.actual_task_positions, "actual_task_positions"},
        std::tuple{&a.task_goal_positions, &b.task_goal_positions, "task_goal_positions"},
        std::tuple{&a.trajectory_task_positions, &b.trajectory_task_positions,
                   "trajectory_task_positions"},
        std::tuple{&a.trajectory_task_velocities, &b.trajectory_task_velocities,
                   "trajectory_task_velocities"}}) {
    const auto r = SameArray(*lhs, *rhs, name);
    if (!r) {
      return r;
    }
  }
  if (a.arm_tip_pose_valid != b.arm_tip_pose_valid) {
    return ::testing::AssertionFailure() << "arm_tip_pose_valid";
  }
  if (a.virtual_tcp_pose_valid != b.virtual_tcp_pose_valid) {
    return ::testing::AssertionFailure() << "virtual_tcp_pose_valid";
  }
  {
    const auto r = SamePose(a.arm_tip_pose, b.arm_tip_pose, "arm_tip_pose");
    if (!r) {
      return r;
    }
  }
  {
    const auto r = SamePose(a.virtual_tcp_pose, b.virtual_tcp_pose, "virtual_tcp_pose");
    if (!r) {
      return r;
    }
  }
  for (std::size_t i = 0; i < a.task_link_poses.size(); ++i) {
    if (a.task_link_pose_valid[i] != b.task_link_pose_valid[i]) {
      return ::testing::AssertionFailure() << "task_link_pose_valid[" << i << "]";
    }
    const auto r = SamePose(a.task_link_poses[i], b.task_link_poses[i],
                            ("task_link_poses[" + std::to_string(i) + "]").c_str());
    if (!r) {
      return r;
    }
  }
  return ::testing::AssertionSuccess();
}

// ── Tick programs ────────────────────────────────────────────────────────────

// One tick's worth of stimulus. Both controllers receive the identical struct,
// so a scenario is data rather than two hand-written call sequences that could
// drift apart and make the comparison vacuous.
struct Tick {
  int arm_channels{kArmDof};  ///< < kArmDof exercises the device-readability gate
  bool arm_valid{true};       ///< false exercises the F5 device-validity gate
  bool hand_valid{true};
  bool estop{false};
  double fingertip_fz{0.0};           ///< > 0 puts a finger in contact
  std::vector<double> arm_target{};   ///< non-empty → SetDeviceTarget(0, ·)
  std::vector<double> hand_target{};  ///< non-empty → SetDeviceTarget(1, ·)
};

// Apply the controller-level half of a tick. Templated because the two classes
// share no base that exposes these — which is itself the point of the copy.
template <class Ctrl>
void ApplyTick(Ctrl& ctrl, const Tick& t) {
  if (t.estop != ctrl.IsEstopped()) {
    if (t.estop) {
      ctrl.TriggerEstop();
    } else {
      ctrl.ClearEstop();
    }
  }
  if (!t.arm_target.empty()) {
    ctrl.SetDeviceTarget(0, std::span<const double>(t.arm_target));
  }
  if (!t.hand_target.empty()) {
    ctrl.SetDeviceTarget(1, std::span<const double>(t.hand_target));
  }
}

// Build the state for a tick, carrying `arm_meas` forward as the closed-loop
// measurement. The loop is closed from the TASK controller's command: both
// controllers are fed the same state either way, and any tick where that is not
// true has already failed the comparison.
ControllerState MakeTickState(const Tick& t, const std::array<double, kArmDof>& arm_meas,
                              std::uint64_t iteration) {
  ControllerState state = MakeIiwa7LeapState();
  state.iteration = iteration;
  state.t_relative_s = static_cast<double>(iteration) * kDt;

  auto& dev0 = state.devices[0];
  dev0.num_channels = t.arm_channels;
  dev0.valid = t.arm_valid;
  // Only the reported channels carry a measurement — an unreported joint must
  // read back as a finite 0.0, which is what makes the narrow-device tick a real
  // gate exercise instead of a full-width tick wearing a smaller number.
  dev0.positions = {};
  for (int i = 0; i < t.arm_channels && i < kArmDof; ++i) {
    dev0.positions[static_cast<std::size_t>(i)] = arm_meas[static_cast<std::size_t>(i)];
  }

  state.devices[1].valid = t.hand_valid;
  if (t.fingertip_fz != 0.0) {
    for (int f = 0; f < 3; ++f) {
      SetFingertipForce(state, f, static_cast<float>(t.fingertip_fz));
    }
  }
  return state;
}

template <class Ctrl>
std::unique_ptr<Ctrl> BringUp(const YAML::Node& cfg) {
  auto ctrl = std::make_unique<Ctrl>("", typename Ctrl::Gains{});
  ctrl->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
  return ctrl;
}

// Drive both controllers through the same program and compare every tick.
void RunAndCompare(const std::vector<Tick>& program, const char* what) {
  const std::string profile = "iiwa7_leap";
  YAML::Node task_cfg = ShippedControllerNode(profile, "demo_task_controller");
  YAML::Node comp_cfg = ShippedControllerNode(profile, "demo_compliance_controller");
  MergeShippedShared(task_cfg, profile);
  MergeShippedShared(comp_cfg, profile);

  auto task = BringUp<DemoTaskController>(task_cfg);
  auto comp = BringUp<DemoComplianceController>(comp_cfg);

  std::array<double, kArmDof> arm_meas{};
  for (int i = 0; i < kArmDof; ++i) {
    arm_meas[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }

  for (std::size_t k = 0; k < program.size(); ++k) {
    const Tick& t = program[k];
    ApplyTick(*task, t);
    ApplyTick(*comp, t);

    const ControllerState state = MakeTickState(t, arm_meas, static_cast<std::uint64_t>(k) + 1);
    const ControllerOutput a = task->Compute(state);
    const ControllerOutput b = comp->Compute(state);

    EXPECT_TRUE(SameOutput(a, b)) << what << ": diverged at tick " << k;
    if (::testing::Test::HasFailure()) {
      return;  // one divergence is the finding; the rest is noise
    }

    // Close the loop so the arm actually travels — a program that never moves
    // compares two controllers holding still, which almost nothing can break.
    for (int i = 0; i < kArmDof && i < a.devices[0].num_channels; ++i) {
      arm_meas[static_cast<std::size_t>(i)] = a.devices[0].commands[static_cast<std::size_t>(i)];
    }
  }
}

std::vector<Tick> Repeat(Tick t, int n) {
  return std::vector<Tick>(static_cast<std::size_t>(n), t);
}

void Append(std::vector<Tick>& dst, const std::vector<Tick>& src) {
  dst.insert(dst.end(), src.begin(), src.end());
}

// ── 1. The shipped configs are one config ───────────────────────────────────
//
// This is the half that covers ur5e_p1a and ur5e_p1b, and the half that fails
// when someone tunes a gain in one file and not the other. Comparing the parsed
// node rather than the file text is deliberate: the two files differ in comments
// and in the top-level key by design, and neither of those is configuration.
TEST(ComplianceTaskEquivalence, ShippedConfigsAreTheSameConfig) {
  for (const char* profile : kProfiles) {
    const YAML::Node task = ShippedControllerNode(profile, "demo_task_controller");
    const YAML::Node comp = ShippedControllerNode(profile, "demo_compliance_controller");
    EXPECT_EQ(YAML::Dump(comp), YAML::Dump(task))
        << profile
        << ": demo_compliance_controller.yaml has drifted from its sibling. Until #469 S3 the "
           "two must stay identical below the top-level key — if this is the S3 divergence, "
           "re-scope this test to the blocks S3 does not own instead of deleting it.";
  }
}

// ── 2. The comparator sees the whole struct ─────────────────────────────────
//
// SameOutput names every field by hand, so a field added to ControllerOutput
// would be compared by nobody and this file would quietly get weaker. There is
// no way to ask C++ for "did I cover every member", so the proxy is the size:
// when this fails, add the new field to SameOutput and then update the constant.
TEST(ComplianceTaskEquivalence, OutputStructHasNotGrownPastTheComparator) {
  EXPECT_EQ(sizeof(ControllerOutput), kExpectedControllerOutputSize)
      << "rtc::ControllerOutput changed size — SameOutput() in this file compares members by "
         "hand and may now be missing one.";
}

// ── 3. Behavioural equivalence, scenario by scenario ────────────────────────

TEST(ComplianceTaskEquivalence, NominalTrackingIsIdentical) {
  std::vector<Tick> program = Repeat(Tick{}, 20);
  // A task-space goal mid-flight: the arm has to actually go somewhere, or the
  // CLIK / trajectory / virtual-TCP paths never run.
  program[2].arm_target = {0.45, 0.10, 0.55, 0.0, 0.0, 0.0};
  program[2].hand_target = std::vector<double>(kHandDof, 0.3);
  Append(program, Repeat(Tick{}, 80));
  RunAndCompare(program, "nominal tracking");
}

TEST(ComplianceTaskEquivalence, NarrowArmDeviceIsRefusedIdentically) {
  std::vector<Tick> program = Repeat(Tick{}, 10);  // seed from a readable device first
  Tick narrow;
  narrow.arm_channels = kArmDof - 1;
  Append(program, Repeat(narrow, 10));
  Append(program, Repeat(Tick{}, 10));  // and the recovery is identical too
  RunAndCompare(program, "narrow arm device");
}

TEST(ComplianceTaskEquivalence, InvalidDevicesAreRefusedIdentically) {
  std::vector<Tick> program = Repeat(Tick{}, 10);
  Tick arm_out;
  arm_out.arm_valid = false;
  Append(program, Repeat(arm_out, 5));
  Tick hand_out;
  hand_out.hand_valid = false;
  Append(program, Repeat(hand_out, 5));
  Append(program, Repeat(Tick{}, 10));
  RunAndCompare(program, "invalid devices");
}

TEST(ComplianceTaskEquivalence, EstopRampAndReleaseAreIdentical) {
  std::vector<Tick> program = Repeat(Tick{}, 10);
  program[2].arm_target = {0.45, 0.10, 0.55, 0.0, 0.0, 0.0};
  Tick stopped;
  stopped.estop = true;
  Append(program, Repeat(stopped, 40));  // long enough for the safe-position ramp to move
  Append(program, Repeat(Tick{}, 20));   // and for the release to be identical
  RunAndCompare(program, "estop ramp");
}

TEST(ComplianceTaskEquivalence, FingertipContactIsSeenIdentically) {
  std::vector<Tick> program = Repeat(Tick{}, 10);
  program[2].hand_target = std::vector<double>(kHandDof, 0.6);
  Tick touching;
  touching.fingertip_fz = 4.0;  // past the profile's contact and force thresholds
  touching.hand_target = {};
  Append(program, Repeat(touching, 40));
  RunAndCompare(program, "fingertip contact");
}

}  // namespace
