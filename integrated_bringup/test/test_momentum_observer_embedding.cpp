// ── #135 Layer 1b: the momentum observer inside the production controllers ───
//
// Layer 1 proved the residual math and the coordinate contract against a real
// arm sub-model (test_momentum_observer / test_momentum_observer_wiring). What
// is new here is EMBEDDING: three controllers each build one wiring from a
// `momentum_observer` YAML block, tick it before their control law, hold it on
// E-STOP, and push a row to momentum_observer.csv every tick.
//
// The CSV is the surface these cases exercise, and every controller-level case
// below goes through a real RegisterControllerLogs registration and reads the
// file back. Asserting on the wiring alone would leave the whole
// push/registration path untested, and a row-count assertion against an unbound
// handle passes with the push deleted outright.
//
// The residual has a second observable — `rtc_msgs/PayloadEstimate`, added with
// Layer 2A (#135 D12). Its FILL is owned by test_payload_estimate_topic.cpp and
// the shared row by test_momentum_observer_wiring.cpp §MomentumObserverChannels.
// What belongs here instead is the lifecycle GATE that decides whether that
// publisher is built at all — `if (momentum_wiring_.enabled())` in each
// controller's on_configure. That is an embedding property, not a wire property,
// and it had no observable on either side: with the observer off nothing is
// created, and the CSV lane binds through a different path, so a gate that
// leaked or over-gated looked identical from every other fixture (#454, from
// #135's non-AC residue).
//
// So the CSV cases stop at SetDeviceNameConfigs — the hook that builds the
// wiring — with no ROS node, and the §MomentumObserverLifecycleGate cases at the
// bottom go one step further and run on_configure on a real LifecycleNode. Only
// those need DDS, which is why this target claims the package ROS_DOMAIN_ID.

#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/momentum_observer_wiring.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace rub = rtc_urdf_bridge;

using integrated_bringup::BuildMomentumObserverWiring;
using integrated_bringup::MomentumObserverParams;
using integrated_bringup::MomentumObserverWiring;
using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;

/// The arm device's joint order, exactly as MakeIiwa7LeapDeviceConfigs states
/// it — which is what the controllers hand the wiring.
std::vector<std::string> ArmJointNames() {
  return MakeIiwa7LeapDeviceConfigs().at("iiwa7").joint_state_names;
}

std::shared_ptr<const pinocchio::Model> ArmModel() {
  return SharedIiwa7LeapBuilder()->GetReducedModel("iiwa7");
}

// ── Part A: the params → wiring rules ────────────────────────────────────────

MomentumObserverWiring BuildFrom(const MomentumObserverParams& p) {
  MomentumObserverWiring w;
  BuildMomentumObserverWiring(p, ArmModel(), ArmJointNames(), 0, w);
  return w;
}

TEST(MomentumObserverBuild, StaysDisabledWithoutAnEnabledBlock) {
  // No block at all — the shipped state of any variant that did not opt in.
  EXPECT_FALSE(BuildFrom(MomentumObserverParams{}).enabled());

  MomentumObserverParams off;
  off.has_block = true;
  off.enabled = false;
  EXPECT_FALSE(BuildFrom(off).enabled());

  // Block present and on, but nothing to observe: disabled rather than thrown,
  // because a caller with no arm sub-model is not misconfigured.
  MomentumObserverParams on;
  on.has_block = true;
  {
    MomentumObserverWiring w;
    EXPECT_NO_THROW(BuildMomentumObserverWiring(on, nullptr, ArmJointNames(), 0, w));
    EXPECT_FALSE(w.enabled());
  }
  {
    MomentumObserverWiring w;
    EXPECT_NO_THROW(BuildMomentumObserverWiring(on, ArmModel(), {}, 0, w));
    EXPECT_FALSE(w.enabled());
  }
}

TEST(MomentumObserverBuild, ExpandsGainsAndRejectsAWrongLengthList) {
  MomentumObserverParams p;
  p.has_block = true;

  // Empty → the documented default, broadcast.
  {
    const auto w = BuildFrom(p);
    ASSERT_TRUE(w.enabled());
    EXPECT_EQ(w.dof, kArmDof);
    EXPECT_EQ(w.observer.dof(), kArmDof);
  }
  // One entry → broadcast.
  p.gains = {25.0};
  EXPECT_TRUE(BuildFrom(p).enabled());
  // Exactly dof → used as given.
  p.gains.assign(static_cast<std::size_t>(kArmDof), 25.0);
  EXPECT_TRUE(BuildFrom(p).enabled());

  // Anything else is refused rather than padded or truncated: either would put
  // the tail joints on a gain no config asked for, and the residual it produces
  // looks perfectly reasonable.
  p.gains.assign(static_cast<std::size_t>(kArmDof) - 1, 25.0);
  EXPECT_THROW(BuildFrom(p), std::invalid_argument);
  p.gains.assign(static_cast<std::size_t>(kArmDof) + 1, 25.0);
  EXPECT_THROW(BuildFrom(p), std::invalid_argument);

  // A non-positive gain is the observer's own rejection reaching this far.
  p.gains.assign(static_cast<std::size_t>(kArmDof), 25.0);
  p.gains[2] = 0.0;
  EXPECT_THROW(BuildFrom(p), std::invalid_argument);
}

// ── The CSV surface ──────────────────────────────────────────────────────────

class ScopedSessionDir {
 public:
  ScopedSessionDir() {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = fs::temp_directory_path() / "rtc_momentum_observer_test";
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

/// g(q) for the arm sub-model, in the arm DEVICE order — what a resting arm's
/// joint torque sensor reads while holding this posture with nothing in its
/// hand. Feeding this into the effort lane is the no-load oracle: the observer
/// must then report ~nothing.
std::vector<double> ArmGravityInDeviceOrder(std::span<const double> q_dev) {
  rub::RtModelHandle h(ArmModel());
  const auto names = ArmJointNames();
  EXPECT_TRUE(h.SetJointOrder(names)) << "arm device joint names do not resolve on the sub-model";
  h.ComputeGeneralizedGravity(q_dev);
  std::vector<double> g(static_cast<std::size_t>(kArmDof), 0.0);
  h.ReorderOutput(h.GetGeneralizedGravity(), std::span<double>(g.data(), g.size()));
  return g;
}

/// Give the arm device a clean, fully-readable state holding kArmHome against
/// gravity. Velocities stay zero: this is the quasi-static no-load case.
void StageNoLoadArmState(rtc::ControllerState& state) {
  auto& dev = state.devices[0];
  dev.valid = true;
  dev.hole_mask = 0;
  dev.velocity_hole_mask = 0;
  dev.effort_hole_mask = 0;
  const std::vector<double> q(kArmHome.begin(), kArmHome.end());
  const std::vector<double> g = ArmGravityInDeviceOrder(q);
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = g[u];
  }
}

/// Register the momentum channel exactly the way on_configure does — through
/// RegisterControllerLogs, so the msg_type dispatch, the enable gate and the
/// shared header/row column count are all on the tested path.
struct MomentumChannel {
  rtc::LogHandle<integrated_bringup::MomentumObserverLogPod> handle;
  fs::path path;
};

struct LogEntry {
  std::string msg_type;
  std::string instance;
};

MomentumChannel BindMomentumChannel(rtc::ControllerLogSet& log_set, bool enabled) {
  const std::vector<LogEntry> entries{
      {std::string(integrated_bringup::kMomentumObserverLogMsgType),
       std::string(integrated_bringup::kMomentumObserverLogInstance)}};
  integrated_bringup::LogRegistrationContext ctx{
      .logger = rclcpp::get_logger("momentum_observer_test"),
      .log_set = log_set,
      .momentum_observer_enabled = enabled,
      .momentum_observer_joint_names = enabled ? ArmJointNames() : std::vector<std::string>{},
  };
  auto reg = integrated_bringup::RegisterControllerLogs(entries, ctx);
  EXPECT_EQ(reg.status, integrated_bringup::LogRegistrationStatus::kSuccess);
  MomentumChannel out;
  out.handle = std::move(reg.handles.momentum_observer);
  for (const auto& ch : log_set.Channels()) {
    if (ch.first == integrated_bringup::kMomentumObserverLogInstance) {
      out.path = ch.second;
    }
  }
  return out;
}

// ── Part B: each production controller, end to end ───────────────────────────

// Required keys only, plus the momentum_observer block. The block rides the
// CONTROLLER yaml rather than demo_shared.yaml because ApplyDemoSharedConfig
// overlays the controller node onto the shared defaults — the same path the
// shipped per-variant demo_shared.yaml takes, minus the file lookup a
// node-less fixture cannot do.
const char* const kMomentumBlock = R"(
momentum_observer:
  enabled: true
  gains: 40.0
)";

const char* const kJointYaml = R"(
arm_dof: 7
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

const char* const kTaskYaml = R"(
arm_dof: 7
kp_translation: [5.0, 5.0, 5.0]
kp_rotation: [2.0, 2.0, 2.0]
singularity_threshold: 0.02
max_damping: 0.05
null_kp: 0.5
enable_null_space: true
control_6dof: false
trajectory_speed: 0.5
trajectory_angular_speed: 2.0
hand_trajectory_speed: 3.0
max_traj_velocity: 1.0
max_traj_angular_velocity: 4.0
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  pi_rotation_margin: 0.15
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

// demo_compliance's config is demo_task's plus the blocks it owns outright, so
// it is DERIVED rather than transcribed: what this file asserts is the observer
// embedding, and two hand-maintained YAMLs would eventually differ in the arm
// posture instead — which surfaces here as a residual that will not settle.
// NOTE for #469 S3: when the compliance binding switches its task gains to the
// core spelling (ik_kp_pos / ik_kp_rot), this derivation has to translate them,
// or the inherited kp_translation lines go unread and silently take defaults.
const std::string kComplianceYaml =
    std::string(kTaskYaml) + "external_wrench:\n  source: \"pull_estimator\"\n";

const char* const kWbcYaml = R"(
arm_dof: 7
command_type: "position"
arm_trajectory_speed: 0.5
hand_trajectory_speed: 1.0
arm_max_traj_velocity: 1.71
hand_max_traj_velocity: 4.0
tcp_trajectory_speed: 0.1
tcp_trajectory_angular_speed: 0.5
tcp_max_traj_velocity: 0.5
tcp_max_traj_angular_velocity: 1.0
pi_rotation_margin: 0.15
integration:
  position_margin: 0.02
  velocity_scale: 0.95
  force_rate_alpha: 0.1
clik:
  damping_sq: 1.0e-4
  v_limit: 1.5
  kx_pos: 5.0
  kx_rot: 2.0
  ka: 1.0
  kh: 1.0
  anchor_drift_max: 0.5
fsm:
  epsilon_pregrasp: 0.005
  force_contact_threshold: 0.2
  min_contacts_for_hold: 2
  slip_rate_threshold: 5.0
  deformation_threshold: 0.015
  max_qp_fail_before_fallback: 5
  approach_speed: 0.5
  release_ramp_sec: 0.03
estop:
  arm_safe_position: [0.0, 0.7, 0.0, -1.4, 0.0, 0.7, 0.0]
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

/// Bring one controller up the way CM does, short of the node: system model →
/// shared builder → control rate → LoadConfig → SetDeviceNameConfigs (the hook
/// that builds the observer wiring).
/// DemoTaskController and its #469 rename-copy are the ones whose constructor
/// also takes a Gains struct; the default-constructed one is what their own URDF
/// fixtures use.
template <class Ctrl>
std::unique_ptr<Ctrl> MakeController() {
  return std::make_unique<Ctrl>("");
}

template <>
std::unique_ptr<integrated_bringup::DemoTaskController>
MakeController<integrated_bringup::DemoTaskController>() {
  return std::make_unique<integrated_bringup::DemoTaskController>(
      "", integrated_bringup::DemoTaskController::Gains{});
}

template <>
std::unique_ptr<integrated_bringup::DemoComplianceController>
MakeController<integrated_bringup::DemoComplianceController>() {
  return std::make_unique<integrated_bringup::DemoComplianceController>(
      "", integrated_bringup::DemoComplianceController::Gains{});
}

template <class Ctrl>
std::unique_ptr<Ctrl> BringUp(const std::string& yaml) {
  auto ctrl = MakeController<Ctrl>();
  ctrl->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(YAML::Load(yaml));
  ctrl->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
  return ctrl;
}

/// Drive `ctrl` through `n_normal` no-load ticks, then `n_estop` E-STOP ticks,
/// and hand back the parsed CSV. `ctrl` must already have the channel bound.
template <class Ctrl>
CsvFile RunAndRead(Ctrl& ctrl, rtc::ControllerLogSet& log_set, const fs::path& path, int n_normal,
                   int n_estop) {
  rtc::ControllerState state = MakeIiwa7LeapState();
  StageNoLoadArmState(state);
  for (int k = 0; k < n_normal; ++k) {
    state.iteration += 1;
    state.t_relative_s = static_cast<double>(state.iteration) * kDt;
    (void)ctrl.Compute(state);
  }
  ctrl.TriggerEstop();
  for (int k = 0; k < n_estop; ++k) {
    state.iteration += 1;
    state.t_relative_s = static_cast<double>(state.iteration) * kDt;
    (void)ctrl.Compute(state);
  }
  log_set.DrainAll();
  return ReadCsv(path);
}

/// ‖r‖∞ column of a row.
double InfNorm(const CsvFile& csv, std::size_t row) {
  const std::size_t c = csv.Column("residual_inf_norm");
  EXPECT_LT(c, csv.header.size()) << "residual_inf_norm column missing";
  return std::stod(csv.rows[row][c]);
}

int ValidFlag(const CsvFile& csv, std::size_t row) {
  const std::size_t c = csv.Column("valid");
  EXPECT_LT(c, csv.header.size()) << "valid column missing";
  return std::stoi(csv.rows[row][c]);
}

std::vector<double> ResidualRow(const CsvFile& csv, std::size_t row) {
  std::vector<double> out;
  for (const auto& name : ArmJointNames()) {
    const std::size_t c = csv.Column("r_" + name);
    EXPECT_LT(c, csv.header.size()) << "missing column r_" << name;
    out.push_back(std::stod(csv.rows[row][c]));
  }
  return out;
}

/// The shared body of the three controller cases. Everything asserted here is
/// a property of the EMBEDDING, not of the observer math (that has its own
/// file): a row per tick, convergence to ~zero under no load, and an E-STOP
/// tick that freezes rather than zeroes.
template <class Ctrl>
void CheckEmbedding(const std::string& yaml, const char* who) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{std::string("momentum_embed_") + who};

  auto ctrl = BringUp<Ctrl>(yaml + kMomentumBlock);
  ASSERT_TRUE(ctrl->MomentumObserverConfigErrorForTesting().empty())
      << who << ": " << ctrl->MomentumObserverConfigErrorForTesting();

  auto ch = BindMomentumChannel(log_set, /*enabled=*/true);
  ASSERT_TRUE(ch.handle) << who << ": momentum channel did not bind";
  ctrl->SetMomentumObserverLogHandleForTesting(std::move(ch.handle));

  constexpr int kNormal = 400;  // 0.8 s at 2 ms — 32 time constants at K_I = 40
  constexpr int kEstop = 5;
  const CsvFile csv = RunAndRead(*ctrl, log_set, ch.path, kNormal, kEstop);

  // One row per tick, header and row the same width — the failure mode a
  // separately-sized header and row produced once already (#440).
  ASSERT_EQ(csv.rows.size(), static_cast<std::size_t>(kNormal + kEstop)) << who;
  for (const auto& row : csv.rows) {
    ASSERT_EQ(row.size(), csv.header.size()) << who << ": row/header width mismatch";
  }
  // Columns are named after the ARM device's joints, in its order.
  for (const auto& name : ArmJointNames()) {
    EXPECT_LT(csv.Column("r_" + name), csv.header.size()) << who << ": missing r_" << name;
  }

  // The last steady tick: the arm holds its own weight and nothing else, so the
  // residual must be ~0. g(q) at this posture is tens of N·m, so a dropped or
  // sign-flipped term cannot hide inside this bound.
  const std::size_t last_normal = static_cast<std::size_t>(kNormal) - 1;
  EXPECT_EQ(ValidFlag(csv, last_normal), 1) << who << ": steady no-load tick was not valid";
  EXPECT_LT(InfNorm(csv, last_normal), 1e-6)
      << who << ": no-load residual did not settle (" << InfNorm(csv, last_normal) << " N.m)";

  // E-STOP holds: valid goes false and the residual is FROZEN, not zeroed. A
  // zero would assert "no external torque" on a tick nobody measured.
  const std::vector<double> frozen = ResidualRow(csv, last_normal);
  for (std::size_t k = 0; k < static_cast<std::size_t>(kEstop); ++k) {
    const std::size_t row = static_cast<std::size_t>(kNormal) + k;
    EXPECT_EQ(ValidFlag(csv, row), 0) << who << ": E-STOP tick reported a valid residual";
    EXPECT_EQ(ResidualRow(csv, row), frozen) << who << ": E-STOP tick moved the residual";
  }
}

TEST(MomentumObserverEmbedding, JointControllerLogsAConvergedResidualAndHoldsOnEstop) {
  CheckEmbedding<integrated_bringup::DemoJointController>(kJointYaml, "joint");
}

TEST(MomentumObserverEmbedding, TaskControllerLogsAConvergedResidualAndHoldsOnEstop) {
  CheckEmbedding<integrated_bringup::DemoTaskController>(kTaskYaml, "task");
}

TEST(MomentumObserverEmbedding, WbcControllerLogsAConvergedResidualAndHoldsOnEstop) {
  CheckEmbedding<integrated_bringup::DemoWbcController>(kWbcYaml, "wbc");
}

// #469 S2 shipped demo_compliance as a rename-copy, so today this asserts what
// the task case above does. It is enumerated by hand because S3 puts an
// admittance law between the model and the arm command, and the observer's
// residual is computed from the commanded torque — a lane that stops being the
// task binding's is a lane this case is the only one watching.
TEST(MomentumObserverEmbedding, ComplianceControllerLogsAConvergedResidualAndHoldsOnEstop) {
  CheckEmbedding<integrated_bringup::DemoComplianceController>(kComplianceYaml, "compliance");
}

// The gate the whole layer rests on (#446 / AC5): a torque lane with a hole in
// it must HOLD, not read the missing slot as 0 N·m. Driven through the
// controller so the tick's gate call is the one under test, and observed on the
// CSV so a deleted gate shows up as valid=1 rows instead of nothing.
TEST(MomentumObserverEmbedding, AHoledEffortLaneHoldsTheResidualThroughTheController) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"momentum_embed_gate"};

  auto ctrl =
      BringUp<integrated_bringup::DemoJointController>(std::string(kJointYaml) + kMomentumBlock);
  ASSERT_TRUE(ctrl->MomentumObserverConfigErrorForTesting().empty());
  auto ch = BindMomentumChannel(log_set, /*enabled=*/true);
  ASSERT_TRUE(ch.handle);
  ctrl->SetMomentumObserverLogHandleForTesting(std::move(ch.handle));

  rtc::ControllerState state = MakeIiwa7LeapState();
  StageNoLoadArmState(state);
  // Load the arm so the residual is unmistakably non-zero before the gate
  // closes — freezing a zero would prove nothing.
  const std::vector<double> tau_ext{2.0, -3.0, 1.5, 0.5, -0.25, 0.75, -0.5};
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].efforts[static_cast<std::size_t>(i)] -= tau_ext[static_cast<std::size_t>(i)];
  }

  constexpr int kOpen = 400;
  constexpr int kClosed = 20;
  for (int k = 0; k < kOpen; ++k) {
    state.iteration += 1;
    state.t_relative_s = static_cast<double>(state.iteration) * kDt;
    (void)ctrl->Compute(state);
  }
  // One slot of the effort lane goes missing — everything else stays perfect.
  state.devices[0].effort_hole_mask = 1ULL << 3;
  for (int k = 0; k < kClosed; ++k) {
    state.iteration += 1;
    state.t_relative_s = static_cast<double>(state.iteration) * kDt;
    (void)ctrl->Compute(state);
  }
  log_set.DrainAll();

  const CsvFile csv = ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), static_cast<std::size_t>(kOpen + kClosed));

  const std::size_t last_open = static_cast<std::size_t>(kOpen) - 1;
  ASSERT_EQ(ValidFlag(csv, last_open), 1);
  const std::vector<double> converged = ResidualRow(csv, last_open);
  // The known external torque came back out, in the arm device's joint order.
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(converged[static_cast<std::size_t>(i)], tau_ext[static_cast<std::size_t>(i)], 1e-6)
        << "joint " << i;
  }

  const std::vector<double> frozen = converged;
  for (std::size_t k = 0; k < static_cast<std::size_t>(kClosed); ++k) {
    const std::size_t row = static_cast<std::size_t>(kOpen) + k;
    EXPECT_EQ(ValidFlag(csv, row), 0) << "holed effort lane still produced a valid residual";
    EXPECT_EQ(ResidualRow(csv, row), frozen) << "held tick moved the residual";
  }
}

// A controller whose variant carries no `momentum_observer` block must not pay
// for the observer, and must not produce the file either. The registration is
// asked with the SAME enable gate on_configure uses, so a block-less variant
// that somehow registered would show up here.
TEST(MomentumObserverEmbedding, NoBlockMeansNoObserverAndNoFile) {
  ScopedSessionDir session;
  rtc::ControllerLogSet log_set{"momentum_embed_off"};

  auto ctrl = BringUp<integrated_bringup::DemoJointController>(kJointYaml);
  ASSERT_TRUE(ctrl->MomentumObserverConfigErrorForTesting().empty());

  auto ch = BindMomentumChannel(log_set, /*enabled=*/false);
  EXPECT_FALSE(ch.handle) << "the channel registered on a variant with no observer";
  EXPECT_TRUE(log_set.empty());

  // The controller still ticks; it just has nothing to say about the residual.
  rtc::ControllerState state = MakeIiwa7LeapState();
  StageNoLoadArmState(state);
  for (int k = 0; k < 10; ++k) {
    state.iteration += 1;
    (void)ctrl->Compute(state);
  }
}

// A gain list the arm's dof cannot accept must FAIL the configure rather than
// leave an observer that silently never runs. OnDeviceConfigsSet cannot throw
// through CM, so the error is latched and on_configure is what refuses.
TEST(MomentumObserverEmbedding, ABadGainListIsLatchedAsAConfigureError) {
  auto ctrl = BringUp<integrated_bringup::DemoJointController>(std::string(kJointYaml) + R"(
momentum_observer:
  enabled: true
  gains: [10.0, 10.0]
)");
  EXPECT_FALSE(ctrl->MomentumObserverConfigErrorForTesting().empty())
      << "a 2-entry gain list was accepted for a 7-DoF arm";
  EXPECT_NE(ctrl->MomentumObserverConfigErrorForTesting().find("momentum_observer.gains"),
            std::string::npos)
      << ctrl->MomentumObserverConfigErrorForTesting();
}

}  // namespace

// ── §MomentumObserverLifecycleGate — the on_configure branch (#454, from #135) ─
//
// `if (momentum_wiring_.enabled()) SetupPayloadEstimatePublisher(...)` decides
// whether the residual reaches the wire. Both sides matter and neither had an
// observable: a gate stuck OPEN publishes an all-zero estimate that a consumer
// cannot distinguish from "no external wrench", and a gate stuck CLOSED takes
// the residual off the wire while every CSV assertion in this file still passes.
//
// These run the full production bring-up plus on_configure on a real
// LifecycleNode — that call is where the branch lives, and the fixtures above
// deliberately stop before it.
namespace {

/// BringUp + on_configure on a real node. Returns the controller; the node is
/// handed back through `node` because the publisher's lifetime is tied to it.
template <class Ctrl>
std::unique_ptr<Ctrl> ConfigureOnNode(const std::string& yaml, const char* node_name,
                                      rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);
  node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, "", opts);

  auto ctrl = BringUp<Ctrl>(yaml);
  const YAML::Node cfg = YAML::Load(yaml);
  const auto rc = ctrl->on_configure(rclcpp_lifecycle::State{}, node, cfg);
  EXPECT_EQ(rc, Ctrl::CallbackReturn::SUCCESS)
      << node_name << ": on_configure failed — the gate assertion below would be vacuous";
  return ctrl;
}

class MomentumObserverLifecycleGate : public ::testing::Test {
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

  /// Both sides of the branch for one controller, in one case: the same
  /// bring-up, differing only in the `momentum_observer` block. Asserting them
  /// together is what makes each meaningful — a publisher that is always absent
  /// would satisfy the disabled case alone, and one that is always present would
  /// satisfy the enabled case alone.
  template <class Ctrl>
  void CheckGate(const std::string& yaml, const char* who) {
    rclcpp_lifecycle::LifecycleNode::SharedPtr on_node;
    auto enabled = ConfigureOnNode<Ctrl>(yaml + kMomentumBlock,
                                         (std::string("mo_gate_on_") + who).c_str(), on_node);
    ASSERT_TRUE(enabled->MomentumObserverConfigErrorForTesting().empty())
        << who << ": " << enabled->MomentumObserverConfigErrorForTesting();
    EXPECT_TRUE(enabled->HasPayloadEstimatePublisherForTesting())
        << who
        << ": observer enabled but no PayloadEstimate publisher — the residual "
           "never reaches the wire";

    rclcpp_lifecycle::LifecycleNode::SharedPtr off_node;
    auto disabled =
        ConfigureOnNode<Ctrl>(yaml, (std::string("mo_gate_off_") + who).c_str(), off_node);
    ASSERT_TRUE(disabled->MomentumObserverConfigErrorForTesting().empty())
        << who << ": " << disabled->MomentumObserverConfigErrorForTesting();
    EXPECT_FALSE(disabled->HasPayloadEstimatePublisherForTesting())
        << who
        << ": no observer, yet a PayloadEstimate publisher exists — it would "
           "advertise an estimate nothing computes";
  }
};

}  // namespace

TEST_F(MomentumObserverLifecycleGate, JointControllerPublishesOnlyWithTheObserverOn) {
  CheckGate<integrated_bringup::DemoJointController>(kJointYaml, "joint");
}

TEST_F(MomentumObserverLifecycleGate, TaskControllerPublishesOnlyWithTheObserverOn) {
  CheckGate<integrated_bringup::DemoTaskController>(kTaskYaml, "task");
}

TEST_F(MomentumObserverLifecycleGate, WbcControllerPublishesOnlyWithTheObserverOn) {
  CheckGate<integrated_bringup::DemoWbcController>(kWbcYaml, "wbc");
}

TEST_F(MomentumObserverLifecycleGate, ComplianceControllerPublishesOnlyWithTheObserverOn) {
  CheckGate<integrated_bringup::DemoComplianceController>(kComplianceYaml, "compliance");
}
