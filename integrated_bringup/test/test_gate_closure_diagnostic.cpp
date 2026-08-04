// ── F5 gate closure diagnostic, at the binding (issue #307) ──────────────────
//
// A gate closed by WIDTH stays closed for the life of the process and its only
// other trace is an absence — the arm-tip frame stops appearing in
// /<key>/transforms, which does not distinguish "gate closed" from "controller
// inactive" from "TF slot unset". §3.7 of rtc_controllers/docs/
// compliance-conventions.md owns the contract; this file pins that all three
// shipped bindings actually emit it, and that they agree on WHEN.
//
// WHY THIS IS ITS OWN TEST BINARY. RCLCPP_*_THROTTLE keeps its state in a
// static at the macro's expansion point, so the window is per-process and per
// call site — not per controller instance. test_device_readability_gate.cpp
// drives narrow devices through the same call sites many times, which would
// consume the window before any assertion here ran, and a suppressed line is
// indistinguishable from a line that was never written. A separate executable
// is a separate process, so every window below starts fresh.
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
#include <cstdio>
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

std::vector<std::string> HandNeedles() {
  return {"num_channels=" + std::to_string(kNarrowHand),
          "hand_dof=" + std::to_string(kHandChannels), "joint_state_names"};
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
    // The arm is full width again, so this fires the hand site only.
    SCOPED_TRACE("step 4 — the secondary axis is diagnosed like the arm, with its own key (#291)");
    static_cast<void>(ctrl.Compute(MakeState(kArmDof, kNarrowHand)));
    EXPECT_EQ(LogSink::WarningsMatching(HandNeedles()).size(), 1u);
  }
}

class GateDiagnosticTest : public ::testing::Test {
 protected:
  void SetUp() override { LogSink::Install(); }

  void TearDown() override { LogSink::Restore(); }
};

TEST_F(GateDiagnosticTest, DemoJointReportsTheWidthShortfallOnBothAxes) {
  DemoJointController ctrl{""};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(MinimalJointYaml())));
  ExerciseBothAxes(ctrl);
}

TEST_F(GateDiagnosticTest, DemoTaskReportsTheWidthShortfallOnBothAxes) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  ASSERT_NO_THROW(ctrl.LoadConfig(YAML::Load(MinimalTaskYaml())));
  ExerciseBothAxes(ctrl);
}

// No LoadConfig here, mirroring test_device_readability_gate.cpp: this binding
// resolves arm_dof_ from the first readable tick when YAML is bypassed. Step 1
// therefore carries an extra claim for this binding — arm_dof_ is still 0 while
// the diagnostic runs, so a helper that reported on `num_channels < 0` would
// surface as a step-1 failure here and nowhere else.
TEST_F(GateDiagnosticTest, DemoWbcReportsTheWidthShortfallOnBothAxes) {
  DemoWbcController ctrl{""};
  ExerciseBothAxes(ctrl);
}

}  // namespace
