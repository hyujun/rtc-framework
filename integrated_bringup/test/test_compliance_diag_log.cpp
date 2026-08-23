// ── The §7 diagnostics lane, as this binding writes it (#469 S4) ─────────────
//
// S3 gave demo_compliance a law and no way to see it. This file asserts the
// CHANNEL — that a row exists per tick, that it survives the ticks the law does
// not run, that it carries the parameters a stored row needs to be readable,
// and above all that its columns describe ONE tick rather than two.
//
// WHY A BOUND HANDLE AND A REAL FILE. The unit fixtures build through
// LoadConfig, which leaves every production log handle unbound — and a row
// assertion written against that state passes with the push deleted outright
// (#424, where exactly that shape let a `fill()` deletion survive its own
// test). So the channel is registered here against a test-owned
// ControllerLogSet and read back off disk: the assertions below fail if the
// push, the registration or the writer goes away.
//
// THE ONE-TICK SKEW IS THE POINT OF THE LAST TEST. The wrench is published from
// ComputeSecondary and consumed by the NEXT tick's ComputeControl, so at the
// push tail — which runs after both — the controller's live `invalid_reason`
// already belongs to a sample the law has not seen. The row must carry the
// verdict that gated ITS OWN wrench, which is why every field is staged inside
// ComputeControl. That test drives the two apart deliberately and pins which
// one lands in the file.
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/logging/compliance_diag_log_pod.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "shipped_config_test_fixture.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

using integrated_bringup::ComplianceDiagLogPod;
using integrated_bringup::DemoComplianceController;
using rtc::ControllerOutput;
using rtc::ControllerState;

using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::MergeShippedShared;
using integrated_bringup::testfx::SetFingertipForce;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;
using integrated_bringup::testfx::ShippedControllerNode;

constexpr const char* kProfile = "iiwa7_leap";

// ── Session dir + CSV reader ────────────────────────────────────────────────
// Same shape test_momentum_observer_embedding uses; kept local because the two
// suites are separate binaries and a shared header would couple their fixtures.

class ScopedSessionDir {
 public:
  ScopedSessionDir() {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = fs::temp_directory_path() / "rtc_compliance_diag_test";
    fs::create_directories(base);
    dir_ = base / ("s_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) & 0xFFFFFFFFU));
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  ~ScopedSessionDir() {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_value_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  ScopedSessionDir(const ScopedSessionDir&) = delete;
  ScopedSessionDir& operator=(const ScopedSessionDir&) = delete;

 private:
  fs::path dir_;
  bool had_prev_{false};
  std::string prev_value_;
};

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

struct CsvFile {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;

  [[nodiscard]] std::size_t Column(const std::string& name) const {
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) {
        return i;
      }
    }
    return header.size();
  }

  /// Same cell as At(), narrowed to float. The POD stores these as float and
  /// the logger prints float::max_digits10 digits, so the round trip is exact —
  /// comparing at double width would fail on the digits the file never had.
  [[nodiscard]] float AtF(std::size_t row, const std::string& name) const {
    return static_cast<float>(At(row, name));
  }

  [[nodiscard]] double At(std::size_t row, const std::string& name) const {
    const std::size_t c = Column(name);
    EXPECT_LT(c, header.size()) << "missing column " << name;
    if (c >= header.size() || row >= rows.size()) {
      return 0.0;
    }
    EXPECT_EQ(rows[row].size(), header.size())
        << "row " << row << " does not match the header width";
    return std::stod(rows[row][c]);
  }
};

CsvFile ReadCsv(const fs::path& path) {
  CsvFile out;
  std::ifstream in(path);
  std::string line;
  if (!std::getline(in, line)) {
    return out;
  }
  out.header = SplitCsv(line);
  while (std::getline(in, line)) {
    if (!line.empty()) {
      out.rows.push_back(SplitCsv(line));
    }
  }
  return out;
}

// ── Bring-up ────────────────────────────────────────────────────────────────

YAML::Node ComplianceConfig() {
  YAML::Node cfg = ShippedControllerNode(kProfile, "demo_compliance_controller");
  MergeShippedShared(cfg, kProfile);
  return cfg;
}

std::unique_ptr<DemoComplianceController> BringUp() {
  auto ctrl = std::make_unique<DemoComplianceController>("", DemoComplianceController::Gains{});
  ctrl->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(ComplianceConfig());
  ctrl->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  EXPECT_EQ(ctrl->on_activate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  return ctrl;
}

struct DiagChannel {
  rtc::LogHandle<ComplianceDiagLogPod> handle;
  fs::path path;
};

struct LogEntry {
  std::string msg_type;
  std::string instance;
};

/// Register the channel the way on_configure does — through the shared helper,
/// on the shipped msg_type/instance pair. Going through RegisterControllerLogs
/// rather than log_set.RegisterLog directly is deliberate: it puts the
/// registration branch and its `compliance_diag_enabled` gate under test too,
/// so deleting either shows up here as an unbound handle.
DiagChannel BindDiagChannel(rtc::ControllerLogSet& log_set, bool enabled = true) {
  const std::vector<LogEntry> entries{
      {std::string(integrated_bringup::kComplianceDiagLogMsgType),
       std::string(integrated_bringup::kComplianceDiagLogInstance)}};
  integrated_bringup::LogRegistrationContext ctx{
      .logger = rclcpp::get_logger("compliance_diag_test"),
      .log_set = log_set,
      .compliance_diag_enabled = enabled,
  };
  auto reg = integrated_bringup::RegisterControllerLogs(entries, ctx);
  EXPECT_EQ(reg.status, integrated_bringup::LogRegistrationStatus::kSuccess);
  DiagChannel out;
  out.handle = std::move(reg.handles.compliance_diag);
  for (const auto& ch : log_set.Channels()) {
    if (ch.first == integrated_bringup::kComplianceDiagLogInstance) {
      out.path = ch.second;
    }
  }
  return out;
}

/// Drives the controller the way production does, closing the joint loop so the
/// arm actually travels. `PublishExternalWrenchForTesting` is the S3 fixture's
/// seam and is used for the same reason: the source adapter has its own suite,
/// and what is asserted here is the LANE, not the estimate.
struct Driver {
  DemoComplianceController* ctrl;
  /// Drained every `kDrainEvery` ticks, the way the controller's own 100 ms
  /// timer does. NOT optional: the SPSC ring holds 512 rows, so a program long
  /// enough to matter silently loses its tail without this — and the row a test
  /// then reads as "the last tick" is whichever tick filled the ring.
  rtc::ControllerLogSet* log_set = nullptr;
  ControllerState state = MakeIiwa7LeapState();
  std::array<double, kArmDof> arm_meas = kArmHome;
  std::uint64_t iteration = 0;
  Eigen::Vector3d apply_point{Eigen::Vector3d::Zero()};

  static constexpr int kDrainEvery = 50;

  DemoComplianceController::ComplianceProbe Step(const rtc::compliance::Wrench6* w) {
    for (int i = 0; i < kArmDof; ++i) {
      state.devices[0].positions[static_cast<std::size_t>(i)] =
          arm_meas[static_cast<std::size_t>(i)];
    }
    state.iteration = ++iteration;
    state.t_relative_s = static_cast<double>(iteration) * kDt;
    if (w != nullptr) {
      ctrl->PublishExternalWrenchForTesting(*w, apply_point);
    }
    const ControllerOutput out = ctrl->Compute(state);
    const auto probe = ctrl->GetComplianceProbeForTesting();
    if (log_set != nullptr && iteration % kDrainEvery == 0) {
      log_set->DrainAll();
    }
    apply_point = probe.task_origin;
    for (int i = 0; i < kArmDof && i < out.devices[0].num_channels; ++i) {
      arm_meas[static_cast<std::size_t>(i)] = out.devices[0].commands[static_cast<std::size_t>(i)];
    }
    return probe;
  }

  DemoComplianceController::ComplianceProbe Run(int n, const rtc::compliance::Wrench6& w) {
    DemoComplianceController::ComplianceProbe probe;
    for (int k = 0; k < n; ++k) {
      probe = Step(&w);
    }
    return probe;
  }
};

// A pinch: thumb + index carrying a squeeze force. Enough for the shipped
// `required_roles: [thumb]` + `min_valid_contacts: 2` gates to be satisfiable,
// which is what makes the estimator's verdict CHANGE when the force goes away —
// and a verdict that never changes cannot pin a one-tick skew.
void StagePinch(ControllerState& state, float fz) {
  // DeriveFingertipCounts reads this, and the whole fingertip decode loop is
  // bounded by it — leave it 0 (the bare fixture's value) and every force
  // written below is silently never read, which is exactly how this test first
  // asserted a verdict that never moved.
  state.devices[1].num_inference_groups = 4;
  SetFingertipForce(state, 0, fz);  // thumb — the shipped `required_roles`
  SetFingertipForce(state, 1, fz);  // index
  SetFingertipForce(state, 2, fz);  // middle — min_valid_contacts is 2, plus margin
}

void ClearPinch(ControllerState& state) {
  for (int f = 0; f < 3; ++f) {
    SetFingertipForce(state, f, 0.0F);
  }
}

constexpr int kSettleTicks = 400;  // 100 bias samples + the 0.5 s ramp, with slack

// ── The channel itself ──────────────────────────────────────────────────────

TEST(ComplianceDiagLog, EveryTickIsARowAndTheHeaderMatchesTheRowWidth) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"compliance_diag_rows"};
  auto ctrl = BringUp();
  auto ch = BindDiagChannel(log_set);
  ASSERT_TRUE(ch.handle) << "the compliance_diag channel did not bind";
  ctrl->SetComplianceDiagLogHandleForTesting(std::move(ch.handle));

  Driver d{ctrl.get(), &log_set};
  constexpr int kTicks = 120;
  d.Run(kTicks, rtc::compliance::Wrench6{});
  log_set.DrainAll();

  EXPECT_EQ(log_set.TotalDropCount(), 0U)
      << "the ring overflowed — the row count below would be measuring the ring, not the push";
  const CsvFile csv = ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), static_cast<std::size_t>(kTicks))
      << "one row per tick is the whole contract — a gap must mean a dropped row and nothing else";
  for (std::size_t i = 0; i < csv.rows.size(); ++i) {
    EXPECT_EQ(csv.rows[i].size(), csv.header.size())
        << "row " << i << " is not the header's width (#440)";
    EXPECT_EQ(csv.At(i, "tick"), static_cast<double>(i + 1)) << "tick column is not the RT tick";
  }
  // The fingerprint plot_rtc_log keys on. Asserted here rather than only in the
  // python suite so a rename of the POD's columns cannot pass C++ review while
  // silently unclassifying every stored file.
  EXPECT_LT(csv.Column("x_tilde_x"), csv.header.size());
}

TEST(ComplianceDiagLog, AHeldTickIsAZeroedValidZeroRowRatherThanAGap) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"compliance_diag_held"};
  auto ctrl = BringUp();
  auto ch = BindDiagChannel(log_set);
  ASSERT_TRUE(ch.handle);
  ctrl->SetComplianceDiagLogHandleForTesting(std::move(ch.handle));

  Driver d{ctrl.get(), &log_set};
  constexpr int kLive = 60;
  constexpr int kHeld = 20;
  // A force large enough that the deviation is unmistakably non-zero, so the
  // held rows below are asserted against a lane that actually had something in
  // it — zeros that were always zero would prove nothing.
  const rtc::compliance::Wrench6 push{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  d.Run(kSettleTicks, rtc::compliance::Wrench6{});
  const auto live = d.Run(kLive, push);
  ASSERT_TRUE(live.engaged) << "the arm lane never ran — the held rows would prove nothing";
  ASSERT_GT(live.deviation.head<3>().norm(), 1e-4) << "no deviation to lose";

  ctrl->TriggerEstop();
  d.Run(kHeld, push);
  log_set.DrainAll();

  EXPECT_EQ(log_set.TotalDropCount(), 0U) << "the ring overflowed";
  const CsvFile csv = ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), static_cast<std::size_t>(kSettleTicks + kLive + kHeld))
      << "E-STOP ticks stopped producing rows — a gap the reader cannot distinguish from a drop";
  const std::size_t last_live = static_cast<std::size_t>(kSettleTicks + kLive) - 1;
  EXPECT_EQ(csv.At(last_live, "valid"), 1.0);
  EXPECT_GT(std::abs(csv.At(last_live, "wrench_fx")), 1.0) << "the driven tick logged no wrench";

  for (std::size_t i = last_live + 1; i < csv.rows.size(); ++i) {
    EXPECT_EQ(csv.At(i, "valid"), 0.0) << "row " << i << " claims the law ran on an E-STOP tick";
    // Zeroed, not frozen. A frozen wrench on a halted tick reads as a live
    // measurement — the §10.6 last-value hold this lane exists to make visible.
    EXPECT_EQ(csv.At(i, "wrench_fx"), 0.0) << "row " << i << " froze the last wrench";
    EXPECT_EQ(csv.At(i, "x_tilde_x"), 0.0) << "row " << i << " froze the last deviation";
    EXPECT_EQ(csv.At(i, "alpha"), 0.0) << "row " << i << " froze the activation ramp";
  }
}

TEST(ComplianceDiagLog, TheParameterSnapshotRidesEveryRow) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"compliance_diag_params"};
  auto ctrl = BringUp();
  auto ch = BindDiagChannel(log_set);
  ASSERT_TRUE(ch.handle);
  ctrl->SetComplianceDiagLogHandleForTesting(std::move(ch.handle));

  Driver d{ctrl.get(), &log_set};
  d.Run(30, rtc::compliance::Wrench6{});
  log_set.DrainAll();

  // The oracle is the controller's OWN parsed params, not a copy of the YAML
  // numbers: a snapshot that silently logged defaults would match a hardcoded
  // expectation only by luck, and hardcoding would break every time the shipped
  // profile is retuned. What is asserted is that the row reports what the law
  // used.
  const auto& p = ctrl->GetAdmittanceParamsForTesting().admittance;
  const CsvFile csv = ReadCsv(ch.path);
  ASSERT_FALSE(csv.rows.empty());
  static constexpr const char* kAxes[] = {"x", "y", "z", "rx", "ry", "rz"};
  for (std::size_t row : {std::size_t{0}, csv.rows.size() - 1}) {
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_FLOAT_EQ(csv.AtF(row, std::string("kp_") + kAxes[i]),
                      static_cast<float>(p.stiffness[i]));
      EXPECT_FLOAT_EQ(csv.AtF(row, std::string("kd_") + kAxes[i]),
                      static_cast<float>(p.damping[i]));
      EXPECT_FLOAT_EQ(csv.AtF(row, std::string("md_") + kAxes[i]),
                      static_cast<float>(p.inertia[i]));
    }
    // Without the bound, a flat x_tilde cannot be told from one pinned against
    // the §7.5 box — the reading this whole column set exists to make possible.
    EXPECT_FLOAT_EQ(csv.AtF(row, "max_disp_lin"), static_cast<float>(p.max_displacement_lin));
    EXPECT_FLOAT_EQ(csv.AtF(row, "max_disp_ang"), static_cast<float>(p.max_displacement_ang));
  }
  EXPECT_GT(csv.At(0, "max_disp_lin"), 0.0) << "the shipped profile disabled the bound";
}

TEST(ComplianceDiagLog, EveryRowCarriesTheVerdictThatGatedItsOwnWrench) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"compliance_diag_skew"};
  auto ctrl = BringUp();
  auto ch = BindDiagChannel(log_set);
  ASSERT_TRUE(ch.handle);
  ctrl->SetComplianceDiagLogHandleForTesting(std::move(ch.handle));

  Driver d{ctrl.get(), &log_set};
  const rtc::compliance::Wrench6 zero{};

  // Free hand → pinch → free hand, so the estimator's verdict moves at least
  // twice. WHICH tick it moves on is deliberately not predicted: the pinch
  // normal is derived from the previous tick's touch set, so the verdict lags
  // the staged force by a tick or two of its own. Pinning a single boundary
  // would be pinning that lag, which is the estimator's business, not this
  // lane's. What IS this lane's business holds on every tick, whatever the lag:
  // the row written on tick k carries the verdict that was in force when tick
  // k's law consumed its wrench — i.e. the one ComputeSecondary left on tick
  // k-1 (D-A14).
  std::vector<unsigned> reason_after_tick;
  std::vector<int> quality_after_tick;
  const auto drive = [&](int n) {
    for (int k = 0; k < n; ++k) {
      const auto pr = d.Step(&zero);
      reason_after_tick.push_back(static_cast<unsigned>(pr.invalid_reason));
      quality_after_tick.push_back(pr.quality_low ? 1 : 0);
    }
  };

  drive(40);
  StagePinch(d.state, 3.0F);
  drive(80);
  ClearPinch(d.state);
  drive(40);

  const std::set<unsigned> distinct(reason_after_tick.begin(), reason_after_tick.end());
  ASSERT_GT(distinct.size(), 1U)
      << "the estimator's verdict never moved — a shifted trace and an unshifted one are the "
         "same file, so this test would pass with the skew reintroduced";

  log_set.DrainAll();
  const CsvFile csv = ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), reason_after_tick.size());

  for (std::size_t k = 1; k < csv.rows.size(); ++k) {
    EXPECT_EQ(static_cast<unsigned>(csv.At(k, "invalid_reason")), reason_after_tick[k - 1])
        << "row " << k << " (tick " << k + 1
        << ") carries the verdict computed AFTER its own wrench was consumed — reading the members "
           "at the push tail instead of staging them in ComputeControl is exactly this failure, "
           "and it is the #425 latch order one layer up";
    EXPECT_EQ(static_cast<int>(csv.At(k, "quality_low")), quality_after_tick[k - 1])
        << "row " << k << ": quality_low is skewed the same way";
  }
  // The first row has no predecessor: it carries the member's initial value,
  // which is what a controller that has never run ComputeSecondary has.
  EXPECT_EQ(csv.At(0, "invalid_reason"), 0.0);
}

TEST(ComplianceDiagLog, TheConsumedWrenchAndTheDeviationLandOnTheSameRow) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"compliance_diag_values"};
  auto ctrl = BringUp();
  auto ch = BindDiagChannel(log_set);
  ASSERT_TRUE(ch.handle);
  ctrl->SetComplianceDiagLogHandleForTesting(std::move(ch.handle));

  Driver d{ctrl.get(), &log_set};
  const rtc::compliance::Wrench6 push{{12.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  d.Run(kSettleTicks, rtc::compliance::Wrench6{});
  const auto probe = d.Run(600, push);
  log_set.DrainAll();

  ASSERT_TRUE(probe.engaged);
  ASSERT_EQ(probe.alpha, 1.0) << "the §10.7 ramp never completed";

  const CsvFile csv = ReadCsv(ch.path);
  ASSERT_FALSE(csv.rows.empty());
  const std::size_t last = csv.rows.size() - 1;
  // The row is the probe, not a re-derivation of it: same tick, same values,
  // through float. Anything else means the staging block and the law disagree
  // about which tick they are on.
  EXPECT_FLOAT_EQ(csv.AtF(last, "wrench_fx"), static_cast<float>(probe.wrench_lwa[0]));
  EXPECT_FLOAT_EQ(csv.AtF(last, "x_tilde_x"), static_cast<float>(probe.deviation[0]));
  EXPECT_FLOAT_EQ(csv.AtF(last, "nu_c_x"), static_cast<float>(probe.velocity[0]));
  EXPECT_FLOAT_EQ(csv.AtF(last, "alpha"), static_cast<float>(probe.alpha));
  EXPECT_EQ(csv.At(last, "fsm_state"), static_cast<double>(static_cast<unsigned>(probe.state)));
  EXPECT_EQ(csv.At(last, "wrench_source"), 0.0) << "pull_estimator is the only source today";
  EXPECT_FLOAT_EQ(csv.AtF(last, "task_origin_x"), static_cast<float>(probe.task_origin[0]));
  EXPECT_GT(std::abs(csv.At(last, "wrench_fx")), 1.0) << "the driven force never reached the row";
}

}  // namespace
