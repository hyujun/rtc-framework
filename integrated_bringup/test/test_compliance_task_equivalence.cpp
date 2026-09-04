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
// #469 S3 HAS NOW HAPPENED, and this file was re-scoped rather than deleted —
// the copy would otherwise go unobserved again. What survives:
//
//   * the config comparison, minus the blocks and the three renamed gains the
//     compliance controller owns, each of which has its own assertion here;
//   * the behavioural comparison, narrowed to "WHILE THE WRENCH IS WITHHELD".
//     The admittance integrator is a no-op on a zero wrench and the tick skips
//     the pose composition entirely when the law contributes nothing, so those
//     programs must still be bit-identical — and RunAndCompare now ASSERTS the
//     withholding rather than relying on a fixture that happens not to produce
//     a pull estimate. Without that assertion this file would go quietly
//     vacuous the day someone gives the fixture a grasp.
//
// What the law itself does lives in test_compliance_admittance_coupling.cpp;
// the five per-controller suites got their own demo_compliance entries in the
// commit before this one.
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
#include "integrated_bringup/logging/compliance_diag_log_pod.hpp"
#include "integrated_bringup/support/compliance_wrench_source.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_controllers/params/task_admittance_params.hpp"
#include "shipped_config_test_fixture.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::ApplyDemoSharedConfig;
using integrated_bringup::ComplianceWrenchSource;
using integrated_bringup::DemoComplianceController;
using integrated_bringup::DemoSharedConfig;
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
using integrated_bringup::testfx::MergeShippedShared;
using integrated_bringup::testfx::ResolvedShared;
using integrated_bringup::testfx::SetFingertipForce;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;
using integrated_bringup::testfx::ShippedControllerNode;

// Tripwire for SameOutput()'s hand-written member list — see the test that
// reads it. Updated together with that list, never on its own.
constexpr std::size_t kExpectedControllerOutputSize = 29656;

// The three shipped profiles. Only iiwa7_leap gets the behavioural half — it is
// the profile the URDF fixture models — so the other two are held by the config
// comparison alone. That asymmetry is deliberate and stated rather than hidden:
// a ur5e-shaped fixture would be a second model harness for a controller that
// does not yet have a law to exercise.
constexpr std::array<const char*, 3> kProfiles = {"iiwa7_leap", "ur5e_p1a", "ur5e_p1b"};

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
  // ACTIVATE, or every `arm_target` in the programs below is silently dropped by
  // the #196 activation-generation gate (a target queued while Inactive is
  // stale by definition) and the "nominal tracking" comparison is two arms
  // holding still. Found by #469 S3's coupling fixture, which needed the arm to
  // actually travel and did not get it.
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  EXPECT_EQ(ctrl->on_activate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
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

    // The precondition this whole comparison now rests on (#469 S3). These
    // programs drive no grasp, so the pull adapter withholds and the compliance
    // lane integrates nothing — which is the ONLY regime in which the two
    // controllers are still expected to agree. If this fires, the fixture has
    // grown a wrench source and the program belongs in the coupling suite, not
    // in a looser tolerance here.
    const auto probe = comp->GetComplianceProbeForTesting();
    ASSERT_FALSE(probe.status.valid)
        << what << ": tick " << k
        << " produced an external wrench — equivalence with demo_task is not claimed there";
    ASSERT_TRUE(probe.deviation.isZero(0.0))
        << what << ": tick " << k << " moved the compliant frame";

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
// Keys the compliance controller owns outright, excluded from the equality
// above and covered instead by the two tests after it. An exclusion with no
// replacement assertion is a hole, which is why the list lives three lines from
// the tests that fill it.
constexpr std::array<const char*, 3> kComplianceOwnedKeys = {
    "external_wrench",  // no sibling equivalent — the whole point of the controller
    "pull_estimator",   // per-profile override, see the D-A8 test below
    "stiffness",        // §7.2 K_p^a — D-A3, see the hand-guiding test below
};

// The task gains, which the two files now SPELL differently (#469 D-A13): the
// compliance controller uses the §7 schema's names because
// ParseTaskAdmittanceParams reads its node and would otherwise parse a second,
// unread copy of the same gain. Same physical quantity, same value — so these
// are excluded from the textual comparison and re-asserted pairwise below,
// which keeps the property that mattered (tune one file, the other fails).
struct RenamedGain {
  const char* compliance;  ///< §7 spelling, what demo_compliance_controller.yaml says
  const char* task;        ///< CLIK spelling, what demo_task_controller.yaml says
};

constexpr std::array<RenamedGain, 3> kRenamedGains = {{
    {"ik_kp_pos", "kp_translation"},
    {"ik_kp_rot", "kp_rotation"},
    {"nullspace_kp", "null_kp"},
}};

// The compliance controller owns ONE ENTRY inside `logs:` (#469 S4), not the
// whole key. Excluding `logs` outright would be the easy move and would stop
// comparing the other seven channels — the shipped instance names, the pull and
// momentum lanes — which is most of what this equality is for. So only the
// owned entry is dropped, and the test below asserts it is present here and
// absent from the sibling.
YAML::Node WithoutComplianceDiagLogEntry(const YAML::Node& logs) {
  YAML::Node out(YAML::NodeType::Sequence);
  for (const auto& entry : logs) {
    if (entry["msg_type"] && entry["msg_type"].as<std::string>() ==
                                 std::string(integrated_bringup::kComplianceDiagLogMsgType)) {
      continue;
    }
    out.push_back(entry);
  }
  return out;
}

// `which` selects which half of each pair to strip, because the key to remove
// differs per file — stripping both names from both nodes would hide a
// compliance file that still carried the old spelling.
YAML::Node WithoutComplianceOwnedKeys(const YAML::Node& node, bool is_compliance) {
  YAML::Node out = YAML::Clone(node);
  for (const char* key : kComplianceOwnedKeys) {
    out.remove(key);
  }
  for (const auto& g : kRenamedGains) {
    out.remove(is_compliance ? g.compliance : g.task);
  }
  if (is_compliance && out["logs"] && out["logs"].IsSequence()) {
    out["logs"] = WithoutComplianceDiagLogEntry(out["logs"]);
  }
  return out;
}

TEST(ComplianceTaskEquivalence, ShippedConfigsAreTheSameConfig) {
  for (const char* profile : kProfiles) {
    const YAML::Node task = ShippedControllerNode(profile, "demo_task_controller");
    const YAML::Node comp = ShippedControllerNode(profile, "demo_compliance_controller");
    EXPECT_EQ(YAML::Dump(WithoutComplianceOwnedKeys(comp, /*is_compliance=*/true)),
              YAML::Dump(WithoutComplianceOwnedKeys(task, /*is_compliance=*/false)))
        << profile
        << ": demo_compliance_controller.yaml has drifted from its sibling outside the blocks it "
           "owns. Until #469 S3 gives it a law the rest must stay identical — if this IS the S3 "
           "divergence, move the newly-owned block into kComplianceOwnedKeys and give it its own "
           "assertion, rather than deleting this test.";
  }
}

// ── 1b. What the excluded keys must say ─────────────────────────────────────

// The renamed half of the exclusion. Both directions are asserted: the §7 name
// must be present on the compliance side (a file that kept the old spelling
// would leave the gain at the parser's default and this test would be the only
// place that noticed), the CLIK name must be ABSENT there (both spellings in one
// file is exactly the parsed-but-unread trap D-A13 exists to prevent), and the
// values must match — which is what still fails when someone tunes one file.
TEST(ComplianceTaskEquivalence, RenamedGainsCarryTheSiblingsValues) {
  for (const char* profile : kProfiles) {
    const YAML::Node task = ShippedControllerNode(profile, "demo_task_controller");
    const YAML::Node comp = ShippedControllerNode(profile, "demo_compliance_controller");
    for (const auto& g : kRenamedGains) {
      ASSERT_TRUE(comp[g.compliance])
          << profile << ": demo_compliance_controller.yaml is missing '" << g.compliance
          << "' — the §7 parser would fall back to its default and nothing else would say so";
      EXPECT_FALSE(comp[g.task]) << profile << ": demo_compliance_controller.yaml still carries '"
                                 << g.task << "', which this controller does not read (#469 D-A13)";
      ASSERT_TRUE(task[g.task]) << profile << ": demo_task_controller.yaml lost '" << g.task << "'";
      EXPECT_EQ(YAML::Dump(comp[g.compliance]), YAML::Dump(task[g.task]))
          << profile << ": '" << g.compliance << "' has drifted from the sibling's '" << g.task
          << "'";
    }
  }
}

// The `logs:` half of the exclusion (#469 S4). Both directions again: the entry
// must be here (without it the channel registers on no profile and the S5
// hardware session ships with no compliance_diag.csv at all — a failure whose
// only symptom is an absent file), and it must NOT be on the sibling, which has
// no §7 law to describe and whose LoadConfig rejects the msg_type outright.
TEST(ComplianceTaskEquivalence, EveryProfileOwnsTheDiagLaneAndTheSiblingDoesNot) {
  for (const char* profile : kProfiles) {
    const YAML::Node comp = ShippedControllerNode(profile, "demo_compliance_controller");
    ASSERT_TRUE(comp["logs"] && comp["logs"].IsSequence()) << profile;
    int found = 0;
    for (const auto& entry : comp["logs"]) {
      if (entry["msg_type"] && entry["msg_type"].as<std::string>() ==
                                   std::string(integrated_bringup::kComplianceDiagLogMsgType)) {
        ++found;
        EXPECT_EQ(entry["instance"].as<std::string>(),
                  std::string(integrated_bringup::kComplianceDiagLogInstance))
            << profile << ": the registration branch gates on this exact instance string";
      }
    }
    EXPECT_EQ(found, 1) << profile
                        << ": demo_compliance_controller.yaml must carry exactly one "
                           "ComplianceDiagLog entry";

    const YAML::Node task = ShippedControllerNode(profile, "demo_task_controller");
    for (const auto& entry : task["logs"]) {
      EXPECT_NE(entry["msg_type"].as<std::string>(),
                std::string(integrated_bringup::kComplianceDiagLogMsgType))
          << profile << ": demo_task_controller has no §7 law to diagnose";
    }
  }
}

TEST(ComplianceTaskEquivalence, EveryProfileNamesTheOnlyImplementedWrenchSource) {
  for (const char* profile : kProfiles) {
    const YAML::Node comp = ShippedControllerNode(profile, "demo_compliance_controller");
    ASSERT_TRUE(comp["external_wrench"] && comp["external_wrench"]["source"])
        << profile << ": required 'external_wrench.source' is missing — LoadConfig throws on this";
    EXPECT_EQ(comp["external_wrench"]["source"].as<std::string>(),
              std::string(integrated_bringup::kPullEstimatorSourceName))
        << profile;
  }
  // The task controller must NOT have grown one: a wrench source on a
  // controller with no compliance law would be configuration nobody reads.
  for (const char* profile : kProfiles) {
    EXPECT_FALSE(ShippedControllerNode(profile, "demo_task_controller")["external_wrench"])
        << profile << ": demo_task_controller has no compliance law to feed";
  }
}

// #469 D-A8. The pull estimate this controller will integrate must be a DELTA,
// on every profile, however that profile configures the measurement lane for the
// other three controllers.
//
// p1b is the case that forced the decision: its shared block ships
// `use_baseline_subtraction: false` deliberately (it mirrors the control PC, and
// #177 crit#4's plate rung needs the plate's weight to read as pull). Absolute-
// with-offset is right for a measurement and wrong for this law — with K_p^a = 0
// a constant offset integrates to constant velocity, so the compliant frame
// walks to the displacement limit and stays there. The controller's own YAML
// overrides it; demo_shared.yaml is untouched and the other three controllers
// keep the setting they ship with.
//
// Asserted on the RESOLVED config rather than on p1b's override text, so a
// fourth profile added later with the baseline off fails here instead of
// silently inheriting the wrong regime.
TEST(ComplianceTaskEquivalence, EveryProfileGivesTheComplianceLaneABaselinedEstimate) {
  for (const char* profile : kProfiles) {
    const DemoSharedConfig comp = ResolvedShared(profile, "demo_compliance_controller");
    EXPECT_TRUE(comp.pull_use_baseline_subtraction)
        << profile
        << ": the compliance controller resolves to an absolute-with-offset pull estimate. Add "
           "`pull_estimator: {use_baseline_subtraction: true}` to its YAML — do NOT flip the "
           "shared block, which the other three controllers read.";
    // With the baseline on, a gravity model would subtract the object weight
    // twice (the snapshot already removed it).
    EXPECT_TRUE(comp.pull_estimator_params.gravity_force.isZero(0.0))
        << profile << ": gravity_force is non-zero while the baseline snapshot is on";
  }
}

// #469 D-A3 and D-A5, asserted on the RESOLVED §7 parse rather than on the YAML
// text. Both are decisions about WHICH LAW RUNS, and both shipped for a while as
// core defaults that read as plausible tunings — so a profile that merely omits
// the key has to fail here instead of quietly inheriting the rejected behaviour.
//
// K_p^a = 0 IS THE LAW, not a gain. With a spring the steady state is a position
// (the default K_p = 200 N/m puts a 1.4 N pull 7 mm from X_d, which on hardware
// reads as "the controller did nothing"); with K_p = 0 it is a velocity, and
// losing the grasp leaves the arm standing rather than snapping home. The
// default is the wrong one of the two and nothing else in the build said so.
//
// bias_calibration_samples = 0 IS AN INTERLOCK, not a sample count. The pull
// baseline asserted directly above already removes the bias, so the default 100
// removes it twice — and it also makes `bias_calibrated` conditional on the
// source delivering 100 samples before it goes stale, which a grasp-gated source
// owes nobody. When it does not, the §10.7 ramp is suspended AND
// BIAS_CALIBRATING re-arms every tick: measured on p1b 2026-09-04 (`260904_1023`)
// as alpha pinned at 0 for a whole 16.8 s activation with a live source.
TEST(ComplianceTaskEquivalence, EveryProfileShipsTheHandGuidingLawAndOneBiasRemoval) {
  for (const char* profile : kProfiles) {
    const YAML::Node node = ShippedControllerNode(profile, "demo_compliance_controller");

    // PRESENCE ON THE TEXT, VALUE ON THE PARSE — and neither substitutes for the
    // other. `ParseTaskAdmittanceParams`'s loaders return early on an absent
    // node, so the value assertions below read a DEFAULT just as happily as a
    // shipped one. They catch an omission today only because two of the three
    // defaults (K_p = 200, 100 samples) happen to BE the rejected values — the
    // moment either moves to what D-A3/D-A5 want, a profile that drops the line
    // passes here in silence, and `ShippedConfigsAreTheSameConfig` cannot catch
    // it either because both keys sit in kComplianceOwnedKeys. `payload_mass`
    // has no such luck at all: its default already equals the shipped value, so
    // this is the ONLY assertion in the build that can tell the key from its
    // absence, and it is what makes writing it down worth the config surface.
    ASSERT_TRUE(node["stiffness"])
        << profile
        << ": no `stiffness` key — K_p^a is whatever the core default is, and D-A3 is "
           "a decision about which law runs, not a tuning to inherit";
    ASSERT_TRUE(node["external_wrench"])
        << profile << ": no `external_wrench` block — the source is required with no default";
    ASSERT_TRUE(node["external_wrench"]["bias_calibration_samples"])
        << profile << ": no `external_wrench.bias_calibration_samples` key (D-A5)";
    ASSERT_TRUE(node["external_wrench"]["payload_mass"])
        << profile << ": no `external_wrench.payload_mass` key (D-A5)";

    rtc::params::TaskAdmittanceParams params;
    rtc::params::TaskAdmittanceConfig config;
    rtc::params::ParseTaskAdmittanceParams(node, params, config);

    for (std::size_t i = 0; i < params.admittance.stiffness.size(); ++i) {
      EXPECT_EQ(params.admittance.stiffness[i], 0.0)
          << profile << ": K_p^a[" << i
          << "] is non-zero — D-A3 is hand-guiding, and the core default is a spring";
    }
    EXPECT_EQ(config.wrench.bias_samples, 0)
        << profile
        << ": external_wrench.bias_calibration_samples must be 0 (D-A5). The pull baseline "
           "already removes the bias, and a grasp-gated source cannot promise N samples before "
           "it goes stale — when it does not, the §10.7 ramp never leaves alpha = 0.";
    EXPECT_EQ(config.wrench.payload_mass, 0.0)
        << profile << ": this wrench measures a force on the grasped object, not a tool load";

    // The sibling must not have grown a §7.2 gain: it has no admittance law, so
    // a `stiffness` key there would be parsed by nobody — the same
    // written-and-never-read trap D-A13 covers for the renamed CLIK gains.
    EXPECT_FALSE(ShippedControllerNode(profile, "demo_task_controller")["stiffness"])
        << profile << ": demo_task_controller has no §7.2 virtual dynamics to gain";
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

// ── 4. external_wrench.source, through the real configure path ──────────────
//
// The three tests above read the YAML. These run it through LoadConfig, which is
// what actually decides whether a bring-up survives — a shipped file that parses
// as text but throws at configure would pass every assertion above.

// iiwa7_leap only, because BringUp needs a model and the fixture builds one
// robot. That is not a coverage gap for the parse — the code is profile-blind —
// but it IS one for "this file configures": the ur5e profiles are held by
// EveryProfileNamesTheOnlyImplementedWrenchSource, which reads their YAML but
// never runs LoadConfig over it.
TEST(ComplianceWrenchSourceConfig, TheShippedProfileConfiguresToThePullEstimator) {
  YAML::Node cfg = ShippedControllerNode("iiwa7_leap", "demo_compliance_controller");
  MergeShippedShared(cfg, "iiwa7_leap");
  std::unique_ptr<DemoComplianceController> ctrl;
  ASSERT_NO_THROW(ctrl = BringUp<DemoComplianceController>(cfg));
  EXPECT_EQ(ctrl->GetWrenchSourceForTesting(), ComplianceWrenchSource::kPullEstimator);
}

TEST(ComplianceWrenchSourceConfig, AMissingSourceRefusesToConfigure) {
  YAML::Node cfg = ShippedControllerNode("iiwa7_leap", "demo_compliance_controller");
  MergeShippedShared(cfg, "iiwa7_leap");
  cfg.remove("external_wrench");
  // Not "returns a default": there is no correct default for which measurement
  // drives an arm, and a silent one would be indistinguishable from a config
  // that named the source and got it.
  EXPECT_THROW(BringUp<DemoComplianceController>(cfg), std::runtime_error);
}

TEST(ComplianceWrenchSourceConfig, AnUnimplementedSourceRefusesToConfigure) {
  YAML::Node cfg = ShippedControllerNode("iiwa7_leap", "demo_compliance_controller");
  MergeShippedShared(cfg, "iiwa7_leap");
  // The value that matters: momentum_observer is a REAL source in the design
  // (#469 S6) with no adapter behind it yet. It must be refused exactly like a
  // typo rather than accepted into a lane that produces nothing.
  cfg["external_wrench"]["source"] = "momentum_observer";
  EXPECT_THROW(BringUp<DemoComplianceController>(cfg), std::runtime_error);

  cfg["external_wrench"]["source"] = "pull_estimatorr";
  EXPECT_THROW(BringUp<DemoComplianceController>(cfg), std::runtime_error);
}

// The §7 schema is parsed off this SAME node from #469 S2's successor commit on,
// and nothing in the tick reads the result yet — so this is the only assertion
// that `ParseTaskAdmittanceParams` is called at all. Deleting the call leaves
// every other test in this file green.
//
// The positive control runs FIRST and is an ASSERT: three EXPECT_THROWs on a
// config that never configured would pass for a reason that has nothing to do
// with the schema.
TEST(ComplianceWrenchSourceConfig, AMalformedAdmittanceSchemaRefusesToConfigure) {
  YAML::Node base = ShippedControllerNode("iiwa7_leap", "demo_compliance_controller");
  MergeShippedShared(base, "iiwa7_leap");
  ASSERT_NO_THROW(BringUp<DemoComplianceController>(base))
      << "the shipped profile must configure — every rejection below is measured against it";

  {
    // §7.2: the law divides by the virtual inertia, so a non-positive entry is
    // not a soft setting. Chosen over a mis-shaped sequence because a length
    // error would also be caught by yaml-cpp's own conversion.
    YAML::Node cfg = YAML::Clone(base);
    cfg["desired_inertia"] = std::vector<double>{1.0, 1.0, 0.0, 1.0, 1.0, 1.0};
    EXPECT_THROW(BringUp<DemoComplianceController>(cfg), std::runtime_error);
  }
  {
    // §7.1: admittance takes force as its INPUT, so `enabled: false` is a
    // rejection and not a fallback. This key lives in the very `external_wrench`
    // map the source does — which is the reason that block is SHARED with the
    // core rather than owned by the binding, and the reason `source` must not be
    // "cleaned up" for not appearing in the core schema.
    YAML::Node cfg = YAML::Clone(base);
    cfg["external_wrench"]["enabled"] = false;
    EXPECT_THROW(BringUp<DemoComplianceController>(cfg), std::runtime_error);
  }
  {
    // The §7 parser forces a position command lane. Note what this NARROWS: the
    // binding's own reader maps any non-"torque" string to kPosition and never
    // throws, so before the parser was wired this controller silently accepted
    // `command_type: torque`. It no longer does — deliberately, since the
    // admittance law has no torque lane — and no shipped profile used it.
    YAML::Node cfg = YAML::Clone(base);
    cfg["command_type"] = "torque";
    EXPECT_THROW(BringUp<DemoComplianceController>(cfg), std::runtime_error);
  }
}

}  // namespace
