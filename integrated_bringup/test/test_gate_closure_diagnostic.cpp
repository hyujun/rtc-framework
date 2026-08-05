// ── F5 gate closure diagnostic, at the binding (issues #307, #284) ───────────
//
// A gate closed by WIDTH stays closed for the life of the process and its only
// other trace is an absence — the arm-tip frame stops appearing in
// /<key>/transforms, which does not distinguish "gate closed" from "controller
// inactive" from "TF slot unset". §3.7 of rtc_controllers/docs/
// compliance-conventions.md owns the contract; this file pins that all three
// shipped bindings actually emit it, and that they agree on WHEN.
//
// #284 added a THIRD closure cause — wide enough, but with slots the model
// reads that this message never wrote — and it inherits the same problem for
// the same reason: `valid` is true, the width message stays silent, and the
// only symptom is the same absence. So it is pinned here on the same terms.
// The base package tests the predicates themselves; what cannot be tested there
// is that each binding hands them the right device and the right width, which
// is the half a copy-paste between three near-identical blocks gets wrong.
//
// WHY THIS IS ITS OWN TEST BINARY. RCLCPP_*_THROTTLE keeps its state in a
// static at the macro's expansion point, so the window is per-process and per
// call site — not per controller instance. test_device_readability_gate.cpp
// drives narrow devices through the same call sites many times, which would
// consume the window before any assertion here ran, and a suppressed line is
// indistinguishable from a line that was never written. A separate executable
// is a separate process, so every window below starts fresh.
//
// AND WHY ITS TESTS ARE SPLIT ACROSS TWO ctest REGISTRATIONS. hand_dof_ has two
// sources and the message names a different fix for each, but both variants
// leave the same three call sites — so pinning both needs two processes, not
// two cases. CMakeLists.txt registers this one executable twice, filtered on
// `*DeclaredWidth*` and its COMPLEMENT, so the partition is total: a test whose
// name matches neither intent still runs in the second registration rather than
// silently never running at all.
//
// WHY ONE TEST PER BINDING RATHER THAN ONE PER CLAIM. The window is a property
// of the PROCESS, so ordering that lives across gtest cases is not ordering the
// suite controls: a silence assertion in case N is only honest if no earlier
// case fired that same call site, and gtest is free to reorder cases. Putting
// the whole sequence in one body makes the ordering structural — every silence
// assertion below sits on a window this process has provably not touched, and
// each call site fires exactly once per run. That also makes the suite safe
// under --gtest_shuffle, which the previous split was not.
//
// WHAT THIS SHAPE STILL CANNOT DO: --gtest_repeat. The bindings' log_clock_ is
// an RCL_STEADY_TIME clock (demo_joint_controller.hpp and siblings), so nothing
// in-process can advance past kThrottleSlowMs, and a second run of the same
// body would find every window already consumed. Repetition needs a new
// process, not a new iteration.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace {

using integrated_bringup::DemoJointController;
using integrated_bringup::DemoTaskController;
using integrated_bringup::DemoWbcController;
using rtc::ControllerState;

constexpr int kArmDof = 6;
constexpr int kHandChannels = 10;
constexpr int kNarrowArm = 4;
constexpr int kNarrowHand = 8;
constexpr double kDt = 0.002;

// The holed slots (#284). DIFFERENT per axis on purpose: the slot number is the
// one datum the operator acts on, and it is the only thing distinguishing "the
// hand branch read devices[1] against hand_dof_" from a copy-paste that reached
// for the arm's device or the arm's width. Both sit inside their axis' model
// width, so the width term stays satisfied and only the hole term fires.
constexpr int kArmHoleSlot = 2;
constexpr int kHandHoleSlot = 7;

double MeasuredArm(std::size_t i) {
  return 0.11 + 0.07 * static_cast<double>(i);
}

double MeasuredHand(std::size_t i) {
  return -0.23 - 0.05 * static_cast<double>(i);
}

double SafeArm(std::size_t i) {
  return 1.30 - 0.09 * static_cast<double>(i);
}

// Only the reported channels carry a measurement — the rest keep DeviceState's
// zero-initialisation, which is what the hazard looks like (an unreported joint
// reads back as a finite 0.0). `valid` is a separate axis on purpose: the whole
// point of this suite is that the two gate terms are diagnosed differently.
ControllerState MakeState(int arm_channels, int hand_channels = kHandChannels,
                          bool arm_valid = true) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = arm_channels;
  dev0.valid = arm_valid;
  for (std::size_t i = 0; i < static_cast<std::size_t>(arm_channels); ++i) {
    dev0.positions[i] = MeasuredArm(i);
  }

  auto& dev1 = state.devices[1];
  dev1.num_channels = hand_channels;
  dev1.valid = true;
  for (std::size_t i = 0; i < static_cast<std::size_t>(hand_channels); ++i) {
    dev1.positions[i] = MeasuredHand(i);
  }
  return state;
}

// The same tick, plus a record that one slot the model reads was not written by
// this message (issue #284). Kept apart from MakeState so the width axis stays
// untouched: the hole cause requires `num_channels >= model_dim`, so a state
// that closes the gate this way is one the WIDTH site provably cannot fire on,
// and the two causes' call sites stay independently observable in one process.
ControllerState WithHole(ControllerState state, std::size_t dev_idx, int slot) {
  state.devices[dev_idx].hole_mask = uint64_t{1} << slot;
  return state;
}

std::string MinimalJointYaml() {
  std::string yaml = "arm_dof: " + std::to_string(kArmDof) + "\nestop:\n  arm_safe_position: [";
  for (int i = 0; i < kArmDof; ++i) {
    yaml += (i ? ", " : "") + std::to_string(SafeArm(static_cast<std::size_t>(i)));
  }
  yaml += "]\nfsm:\n  contact_stop_release_eps: 0.005\n  contact_stop_lpf_cutoff_hz: 20.0\n";
  return yaml + "command_type: \"position\"\n";
}

std::string MinimalTaskYaml() {
  std::string yaml = "arm_dof: " + std::to_string(kArmDof) + "\nestop:\n  arm_safe_position: [";
  for (int i = 0; i < kArmDof; ++i) {
    yaml += (i ? ", " : "") + std::to_string(SafeArm(static_cast<std::size_t>(i)));
  }
  yaml += "]\nfsm:\n  pi_rotation_margin: 0.15\n  contact_stop_release_eps: 0.005\n";
  return yaml;
}

// Two topic groups so the bindings resolve a SECONDARY device name — without
// one, OnDeviceConfigsSet has nothing to look the hand config up by and
// hand_dof_ falls back to the device's first report no matter what the map
// below declares.
std::string TopicsTwoGroups() {
  return "topics:\n"
         "  arm:\n    subscribe:\n      - {topic: \"target\", role: \"target\"}\n"
         "  hand:\n    subscribe:\n      - {topic: \"hand_target\", role: \"target\"}\n";
}

// The declared-width shape: the hand group names its kHandChannels joints, so
// hand_dof_ comes from config and the shortfall below is a config mismatch the
// operator can edit. This is the production shape — every shipped hand group
// (_base.yaml for both ur5e variants, iiwa7_leap/sim.yaml) declares them.
std::map<std::string, rtc::DeviceNameConfig> MakeHandDeclaringConfigs() {
  std::map<std::string, rtc::DeviceNameConfig> configs;

  rtc::DeviceNameConfig arm;
  arm.device_name = "arm";
  for (int i = 0; i < kArmDof; ++i) {
    arm.joint_state_names.push_back("a" + std::to_string(i));
  }
  configs["arm"] = std::move(arm);

  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  for (int i = 0; i < kHandChannels; ++i) {
    hand.joint_state_names.push_back("h" + std::to_string(i));
  }
  configs["hand"] = std::move(hand);
  return configs;
}

// ── Log capture ─────────────────────────────────────────────────────────────
//
// The diagnostic's only observable IS the log line — that is the whole point of
// the issue, and adding a counter to the controllers purely so a test could
// read it would put test-only state on the RT path. So the assertion goes to
// the sink the message actually reaches. Same shape as
// rtc_controller_manager/test/test_command_type_gate.cpp; the handler is
// process-global, which this binary can afford because it is a single-TU
// island (see the file header).
class LogSink {
 public:
  static void Install() {
    Clear();
    Previous() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&LogSink::Handler);
  }

  static void Restore() {
    if (Previous() != nullptr) {
      rcutils_logging_set_output_handler(Previous());
      Previous() = nullptr;
    }
  }

  static void Clear() {
    const std::lock_guard<std::mutex> lock(Mutex());
    Lines().clear();
  }

  // Captured WARN lines containing every needle. Substring matching keeps the
  // assertions off wording that carries no contract — but each needle used
  // below is a fact the operator has to receive, not decoration.
  static std::vector<std::string> WarningsMatching(const std::vector<std::string>& needles) {
    const std::lock_guard<std::mutex> lock(Mutex());
    std::vector<std::string> hits;
    for (const auto& [sev, text] : Lines()) {
      if (sev != RCUTILS_LOG_SEVERITY_WARN) {
        continue;
      }
      bool all = true;
      for (const auto& needle : needles) {
        if (text.find(needle) == std::string::npos) {
          all = false;
          break;
        }
      }
      if (all) {
        hits.push_back(text);
      }
    }
    return hits;
  }

 private:
  static constexpr std::size_t kBufferSize = 2048;

  static void Handler(const rcutils_log_location_t* /*location*/, int severity,
                      const char* /*name*/, rcutils_time_point_value_t /*timestamp*/,
                      const char* format, va_list* args) {
    // Value-initialised, not merely declared: vsnprintf truncates and
    // NUL-terminates on overflow, but on an ENCODING failure it returns
    // negative and need not write a terminator at all, and std::string(buffer)
    // would then read past the array. Zeroing degrades that to an empty
    // capture, which is what test_command_type_gate.cpp does for the same
    // reason.
    char buffer[kBufferSize]{};
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    const std::lock_guard<std::mutex> lock(Mutex());
    Lines().emplace_back(severity, std::string(buffer));
  }

  static std::vector<std::pair<int, std::string>>& Lines() {
    static std::vector<std::pair<int, std::string>> lines;
    return lines;
  }

  static std::mutex& Mutex() {
    static std::mutex m;
    return m;
  }

  static rcutils_logging_output_handler_t& Previous() {
    static rcutils_logging_output_handler_t previous = nullptr;
    return previous;
  }
};

// The needles every binding's arm message must carry: both widths, and the KEY
// to edit. #340 made the declared width a required YAML key, which is what lets
// the message name the fix instead of only the mismatch.
//
// The key needle is the QUOTED form. Bare "arm_dof" would be satisfied by the
// "arm_dof=6" needle above it — the whole "fix the 'arm_dof' key ..." clause
// could be deleted and this vector would still match, which is exactly the
// vacuity this file previously shipped.
std::vector<std::string> ArmNeedles() {
  return {"num_channels=" + std::to_string(kNarrowArm), "arm_dof=" + std::to_string(kArmDof),
          "'arm_dof'"};
}

// The hand carries the same pair, but NOT the same prescription: hand_dof_ has
// two sources and only one of them has a key the operator can edit. So the
// message's tail is the part under test here, and each variant is pinned by a
// phrase the other one does not contain — "fix the hand group's" vs "narrowed
// mid-session". A binding that emitted one tail for both sources would satisfy
// one of these vectors and fail the other's absence check.
std::vector<std::string> HandNeedlesFromDeclaredWidth() {
  return {"num_channels=" + std::to_string(kNarrowHand),
          "hand_dof=" + std::to_string(kHandChannels), "fix the hand group's 'joint_state_names'"};
}

std::vector<std::string> HandNeedlesFromDeviceReport() {
  return {"num_channels=" + std::to_string(kNarrowHand),
          "hand_dof=" + std::to_string(kHandChannels), "narrowed mid-session"};
}

// The hole axis (#284). Three claims per needle set, and each is separately
// falsifiable: WHICH slot (a wrong device or a wrong width in the FirstHoleSlot
// call changes it), that the width pair is quoted even though neither width is
// at fault (without it the operator cannot tell this apart from the width
// message), and the KEY — `joint_command_names`, because that is the reference
// list BuildJointStateReorder is actually built against and therefore what
// defines the slot index being reported.
//
// Unlike the hand's WIDTH message this prescription does NOT vary with the
// source of hand_dof_, and that is deliberate rather than an oversight: a hole
// is always "the message carried no joint of that name", so the fix is always
// to reconcile the declared names with the publisher's. The declared-width
// process below asserts the identical text to pin that non-variation.
std::vector<std::string> ArmHoleNeedles() {
  return {"arm device slot " + std::to_string(kArmHoleSlot),
          "num_channels=" + std::to_string(kArmDof), "arm_dof=" + std::to_string(kArmDof),
          "'joint_command_names'"};
}

std::vector<std::string> HandHoleNeedles() {
  return {"hand device slot " + std::to_string(kHandHoleSlot),
          "num_channels=" + std::to_string(kHandChannels),
          "hand_dof=" + std::to_string(kHandChannels), "'joint_command_names'"};
}

// Any hole diagnostic on either axis. Used where the claim is "not on this
// axis" — the axis word is what a binding that passed the wrong device would
// get wrong, so matching only "device slot" would let that through.
std::vector<std::string> AnyHoleDiagnostic() {
  return {"was never written although"};
}

// Any F5 diagnostic at all, regardless of axis or values. Used for the silent
// cases, where the claim is that NOTHING was said.
std::vector<std::string> AnyDiagnostic() {
  return {"F5 gate closed"};
}

// The whole per-binding sequence, in the one order that keeps every assertion
// honest. Templated rather than virtual-dispatched because the three bindings
// share no base that exposes Compute() the way this needs it, and because the
// point is to run the IDENTICAL sequence against each — a divergence between
// them is the failure mode #307's fourth acceptance criterion names.
template <typename Controller>
void ExerciseBothAxes(Controller& ctrl) {
  {
    // Resolves hand_dof_ (and, where YAML was bypassed, arm_dof_) from the
    // first readable tick — without it hand_dof_ stays 0 and the hand gate
    // cannot close. Both windows are untouched here, so this silence is real.
    SCOPED_TRACE("step 1 — a fully readable tick says nothing");
    static_cast<void>(ctrl.Compute(MakeState(kArmDof)));
    ASSERT_TRUE(LogSink::WarningsMatching(AnyDiagnostic()).empty());
  }
  {
    // The arm is narrow AND unreported: the width term is true and the validity
    // term is false, so this tick is refused by the half that must NOT be
    // reported. A full-width invalid device would not test that at all — the
    // width term alone already decides there, and the `dev.valid` term could be
    // deleted from IsGateClosedByWidth with this suite still green.
    //
    // In production CM's startup gate means a binding never sees an unreported
    // device (§3.7); a fixture that bypasses YAML does, and diagnosing it here
    // would duplicate CM's init timeout and fire on every such fixture.
    SCOPED_TRACE("step 2 — an unreported device is the startup shape, not a misconfiguration");
    static_cast<void>(ctrl.Compute(MakeState(kNarrowArm, kHandChannels, /*arm_valid=*/false)));
    ASSERT_TRUE(LogSink::WarningsMatching(AnyDiagnostic()).empty());
  }
  {
    // First and only fire of this binding's arm site in this process.
    SCOPED_TRACE("step 3 — the arm width shortfall names both widths and the key to fix");
    static_cast<void>(ctrl.Compute(MakeState(kNarrowArm)));
    EXPECT_EQ(LogSink::WarningsMatching(ArmNeedles()).size(), 1u);
  }
  {
    // The arm is full width again, so this fires the hand site only. No device
    // config was supplied above, so hand_dof_ came from the device's own first
    // report — the source with no key to edit, and the one this fixture is
    // therefore entitled to assert.
    SCOPED_TRACE("step 4 — the secondary axis is diagnosed like the arm, with its own key (#291)");
    static_cast<void>(ctrl.Compute(MakeState(kArmDof, kNarrowHand)));
    EXPECT_EQ(LogSink::WarningsMatching(HandNeedlesFromDeviceReport()).size(), 1u);
    EXPECT_TRUE(LogSink::WarningsMatching(HandNeedlesFromDeclaredWidth()).empty())
        << "sent the operator to a key this configuration never declared";
  }
  {
    // The gate's THIRD closure cause (#284), and the reason it needs a binding
    // test of its own rather than the base predicate tests it already has:
    // those prove the predicate, not that these three bindings call it with the
    // right device and the right width. A holed tick has no other trace — the
    // width message does not fire, the device is `valid`, and the arm simply
    // stops appearing — so a binding that never wired it up looks identical to
    // one that did until an operator is staring at a silent arm.
    //
    // Both hole sites are untouched at this point, so the hand silence below is
    // a real observation rather than a consumed window.
    SCOPED_TRACE("step 5 — a hole inside a wide-enough arm names the slot, not the width");
    static_cast<void>(ctrl.Compute(WithHole(MakeState(kArmDof), 0, kArmHoleSlot)));
    EXPECT_EQ(LogSink::WarningsMatching(ArmHoleNeedles()).size(), 1u);
    EXPECT_EQ(LogSink::WarningsMatching(AnyHoleDiagnostic()).size(), 1u)
        << "the arm's hole was also reported on another axis";
  }
  {
    // The hand's hole site, at a DIFFERENT slot from the arm's. A binding that
    // reached for devices[0], or for arm_dof_, on this axis either reports slot
    // 2 or reports nothing at all — both fail the needle, and neither would be
    // caught by a fixture that used one slot number for both axes.
    SCOPED_TRACE("step 6 — the secondary axis reports its own slot (#291 + #284)");
    static_cast<void>(ctrl.Compute(WithHole(MakeState(kArmDof), 1, kHandHoleSlot)));
    EXPECT_EQ(LogSink::WarningsMatching(HandHoleNeedles()).size(), 1u);
    EXPECT_EQ(LogSink::WarningsMatching(AnyHoleDiagnostic()).size(), 2u)
        << "one hole per axis, each from its own call site";
  }
}

// The other source (#307). Hand axis only: the arm's prescription is
// unconditional, so re-asserting it here would pin nothing new — and it would
// need a second fire of the arm site, which this process cannot observe.
template <typename Controller>
void ExerciseDeclaredHandWidth(Controller& ctrl) {
  {
    SCOPED_TRACE("step 1 — a fully readable tick says nothing");
    static_cast<void>(ctrl.Compute(MakeState(kArmDof)));
    ASSERT_TRUE(LogSink::WarningsMatching(AnyDiagnostic()).empty());
  }
  {
    SCOPED_TRACE("step 2 — a declared width names the key that declares it");
    static_cast<void>(ctrl.Compute(MakeState(kArmDof, kNarrowHand)));
    EXPECT_EQ(LogSink::WarningsMatching(HandNeedlesFromDeclaredWidth()).size(), 1u);
    EXPECT_TRUE(LogSink::WarningsMatching(HandNeedlesFromDeviceReport()).empty())
        << "blamed the backend for a width this configuration declared itself";
  }
  {
    // The hole prescription does NOT branch on where hand_dof_ came from, and
    // this is the assertion that makes that a decision rather than an omission:
    // the needles are byte-identical to the ones ExerciseBothAxes uses on a
    // config that declared nothing. The width axis two lines above is the
    // counterexample in the same file — it DOES branch, because only one of its
    // two sources has a key to edit. A hole always has one: the slot index is
    // an index into the declared reference list, so that list is always the
    // thing to reconcile.
    SCOPED_TRACE("step 3 — the hole prescription is the same whichever key declared the width");
    static_cast<void>(ctrl.Compute(WithHole(MakeState(kArmDof), 1, kHandHoleSlot)));
    EXPECT_EQ(LogSink::WarningsMatching(HandHoleNeedles()).size(), 1u);
  }
}

class GateDiagnosticTest : public ::testing::Test {
 protected:
  void SetUp() override { LogSink::Install(); }

  void TearDown() override { LogSink::Restore(); }
};

TEST_F(GateDiagnosticTest, DemoJointReportsEveryClosureCauseOnBothAxes) {
  DemoJointController ctrl{""};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(MinimalJointYaml())));
  ExerciseBothAxes(ctrl);
}

TEST_F(GateDiagnosticTest, DemoTaskReportsEveryClosureCauseOnBothAxes) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(MinimalTaskYaml())));
  ExerciseBothAxes(ctrl);
}

// No LoadConfig here, mirroring test_device_readability_gate.cpp: this binding
// resolves arm_dof_ from the first readable tick when YAML is bypassed. Step 1
// therefore carries an extra claim for this binding — arm_dof_ is still 0 while
// the diagnostic runs, so a helper that reported on `num_channels < 0` would
// surface as a step-1 failure here and nowhere else.
TEST_F(GateDiagnosticTest, DemoWbcReportsEveryClosureCauseOnBothAxes) {
  DemoWbcController ctrl{""};
  ExerciseBothAxes(ctrl);
}

// ── The declared-width source, in its own process ────────────────────────────
//
// Everything below runs under the `*DeclaredWidth*` ctest registration (see
// CMakeLists.txt). It has to: these tests fire the SAME three hand call sites
// as the cases above, and a call site can only be observed firing once per
// process. Sharing a process would leave whichever half ran second asserting
// against a consumed window.

class GateDiagnosticDeclaredWidthTest : public ::testing::Test {
 protected:
  void SetUp() override { LogSink::Install(); }

  void TearDown() override { LogSink::Restore(); }
};

TEST_F(GateDiagnosticDeclaredWidthTest, DemoJointNamesTheKeyThatDeclaredTheWidth) {
  DemoJointController ctrl{""};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(MinimalJointYaml() + TopicsTwoGroups())));
  ctrl.SetDeviceNameConfigs(MakeHandDeclaringConfigs());
  ExerciseDeclaredHandWidth(ctrl);
}

TEST_F(GateDiagnosticDeclaredWidthTest, DemoTaskNamesTheKeyThatDeclaredTheWidth) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(MinimalTaskYaml() + TopicsTwoGroups())));
  ctrl.SetDeviceNameConfigs(MakeHandDeclaringConfigs());
  ExerciseDeclaredHandWidth(ctrl);
}

// Topics only, mirroring test_device_readability_gate.cpp's wbc hand fixture:
// this binding needs the secondary group name to look the hand config up, but
// not the arm_dof key — the arm axis is not under test here.
TEST_F(GateDiagnosticDeclaredWidthTest, DemoWbcNamesTheKeyThatDeclaredTheWidth) {
  DemoWbcController ctrl{""};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(TopicsTwoGroups())));
  ctrl.SetDeviceNameConfigs(MakeHandDeclaringConfigs());
  ExerciseDeclaredHandWidth(ctrl);
}

}  // namespace
