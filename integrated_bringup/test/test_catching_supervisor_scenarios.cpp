// ── The S7 supervisor, scenario by scenario, closed loop (L7 §9, G7-A/B/G) ──
//
// WHAT IS UNDER TEST. The whole trial cycle of DemoCatchingController — IDLE
// homing, ARMED, TRACKING, APPROACH, the freeze (COMMITTED), the hand close
// (CLOSING), DECEL at t_c, HOLD, RETREAT and the re-arm — driven through the
// real Compute() on the shipped ur5e_p1b model, with the oracle plan
// (`diagnostic.oracle_plan`) or a hand-stored plan box as the planner. Each
// case asserts the MODE SEQUENCE the controller walked (consecutive duplicates
// collapsed) against L7 §9's table, plus the reason / outcome that names the
// scenario.
//
// THE PLANT. A perfect arm servo (measured q of tick n+1 = command of tick n),
// or arm_lag_fixture's pure delay for the T_arm ≠ 0 case; a perfect HAND servo
// (the hand's measured q is its last command, velocity 0) unless a case
// freezes the hand on purpose; and injected fingertip samples (one new sample
// per tick, 7-value stride, force in slots 1..3) for the contact lane.
//
// REAL TIME IS REAL, as in test_catching_tracking: the controller reads the
// steady clock, so every tick sleeps one control period. Timing assertions are
// written against steady stamps taken immediately before and after each
// Compute(), so they bound the controller's own clock read rather than
// assuming a perfect tick grid.
//
// This file adds no production seam: every observation is an existing getter.

#include "arm_lag_fixture.hpp"
#include "catching_cloud_fixture.hpp"
#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/catching/hand_sequencer.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::TrackingYaml;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::catching::HandPhase;
using rtc::catching::Mode;
using rtc::catching::Outcome;
using rtc::catching::PlanRefusal;
using rtc::catching::PlanSnapshot;
using rtc::catching::Reason;

using namespace std::chrono_literals;

constexpr std::int64_t kMsNs = 1'000'000;
constexpr std::int64_t kHNs = 2 * kMsNs;            // state.dt, the tick the sequencer rounds to
constexpr std::int64_t kTCloseE2eNs = 280 * kMsNs;  // TrackingYaml robot.hand.T_close_e2e
constexpr double kPoseTol = 0.02;                   // supervisor.ready.pose_tol default
constexpr int kTips = 4;                            // the fixture hand's sensor_names
constexpr std::size_t kTipStride = 7;               // no sensor_layout → the 7-value union

const char* ModeName(Mode m) {
  switch (m) {
    case Mode::kIdle:
      return "Idle";
    case Mode::kArmed:
      return "Armed";
    case Mode::kTracking:
      return "Tracking";
    case Mode::kApproach:
      return "Approach";
    case Mode::kCommitted:
      return "Committed";
    case Mode::kClosing:
      return "Closing";
    case Mode::kDecel:
      return "Decel";
    case Mode::kHold:
      return "Hold";
    case Mode::kRetreat:
      return "Retreat";
    case Mode::kAbortSafe:
      return "AbortSafe";
    case Mode::kFault:
      return "Fault";
  }
  return "?";
}

const char* ReasonName(Reason r) {
  switch (r) {
    case Reason::kNone:
      return "None";
    case Reason::kBallStale:
      return "BallStale";
    case Reason::kBallStaleCommitted:
      return "BallStaleCommitted";
    case Reason::kBallStaleLong:
      return "BallStaleLong";
    case Reason::kTrackChanged:
      return "TrackChanged";
    case Reason::kHorizonExtrap:
      return "HorizonExtrap";
    case Reason::kPredInconsistent:
      return "PredInconsistent";
    case Reason::kNoCatchablePlan:
      return "NoCatchablePlan";
    case Reason::kPlanInvalid:
      return "PlanInvalid";
    case Reason::kQpFailed:
      return "QpFailed";
    case Reason::kRefSaturated:
      return "RefSaturated";
    case Reason::kJointConflict:
      return "JointConflict";
    case Reason::kTrackErr:
      return "TrackErr";
    case Reason::kAbortEscalated:
      return "AbortEscalated";
    case Reason::kEstop:
      return "Estop";
    case Reason::kFaultReset:
      return "FaultReset";
    case Reason::kSpeedScaling:
      return "SpeedScaling";
    case Reason::kClockUnhealthy:
      return "ClockUnhealthy";
    case Reason::kParamsTbd:
      return "ParamsTbd";
    case Reason::kHandTimeout:
      return "HandTimeout";
    case Reason::kTipStale:
      return "TipStale";
  }
  return "?";
}

const char* PhaseName(HandPhase p) {
  switch (p) {
    case HandPhase::kOpen:
      return "Open";
    case HandPhase::kPreshape:
      return "Preshape";
    case HandPhase::kClose:
      return "Close";
    case HandPhase::kHold:
      return "Hold";
    case HandPhase::kRelease:
      return "Release";
  }
  return "?";
}

const char* OutcomeName(Outcome o) {
  switch (o) {
    case Outcome::kNone:
      return "None";
    case Outcome::kCaptured:
      return "Captured";
    case Outcome::kMissed:
      return "Missed";
    case Outcome::kUndetermined:
      return "Undetermined";
    case Outcome::kAborted:
      return "Aborted";
  }
  return "?";
}

std::string SeqString(const std::vector<Mode>& seq) {
  std::string s;
  for (std::size_t i = 0; i < seq.size(); ++i) {
    s += (i == 0 ? "" : " -> ");
    s += ModeName(seq[i]);
  }
  return s;
}

/// The D-16 acceleration box, read from the file the controller reads — the
/// bound every joint-space ramp must respect per tick.
std::vector<double> DerivedQddMax() {
  const std::string path = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/ur5e_p1b/derived_accel_limits.yaml";
  const YAML::Node y = YAML::LoadFile(path);
  return y["derived_accel_limits"]["ur5e"]["qdd_max"].as<std::vector<double>>();
}

/// What one tick left behind.
struct TickRec {
  Mode mode{Mode::kIdle};
  Reason reason{Reason::kNone};
  bool hand_active{false};
  HandPhase phase{HandPhase::kOpen};
  bool hand_timeout{false};
  Outcome outcome{Outcome::kNone};
  bool homing{false};
  double track_err{0.0};
  bool armed_latch{false};
  PlanRefusal refusal{PlanRefusal::kNone};
  std::uint64_t admitted{0};
  std::uint32_t plan_id{0};
  std::int64_t plan_t_c_ns{0};
  std::int64_t before_ns{0};                 // steady, immediately before Compute()
  std::int64_t after_ns{0};                  // steady, immediately after Compute()
  std::array<double, kUr5eArmDof> q_meas{};  // the measured arm this tick was given
  std::array<double, kUr5eArmDof> q_out{};   // the arm command it wrote
  bool q_out_valid{false};
  std::array<double, kUr5eArmDof> q_cmd{};   // carried command (GetArmCommandForTesting)
  std::array<double, kUr5eArmDof> qd_cmd{};  // carried velocity
  bool ref_valid{false};
  std::array<double, 3> ref_e{};
  std::array<double, 3> ref_ed{};
  std::array<bool, kTips> tip_contact{};
  std::array<bool, kTips> tip_fresh{};
};

class SupervisorScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_supervisor_scenarios");
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(MakeConfigWithCatchFrame());
    oracle_ = std::make_unique<CatchFrameOracle>(*builder_);
    arm_names_ =
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names;
    topic_ = "/test_catching_supervisor_scenarios/prediction";
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    start_pose_ = oracle_->PoseAt(arm_names_, home, kUr5eArmDof);
    log_.reserve(8000);
  }

  void TearDown() override {
    if (executor_) {
      executor_->remove_node(node_->get_node_base_interface());
    }
    pub_.reset();
    executor_.reset();
    ctrl_.reset();
    node_.reset();
  }

  /// A catch point a few centimetres from where the hand starts, on the
  /// starting approach axis — a short, unsaturated approach and a short return.
  Eigen::Vector3d NearPc(double scale = 1.0) const {
    return start_pose_.translation() + scale * Eigen::Vector3d(0.03, 0.02, 0.02);
  }

  Eigen::Vector3d StartAxis() const { return start_pose_.rotation().col(2); }

  /// TrackingYaml plus `T_hold` shortened to 0.02 s (applied BEFORE `tweak`,
  /// so a case can set its own). Arms the controller unless `arm` is false.
  void BringUp(const Eigen::Vector3d& p_c, const Eigen::Vector3d& a_d, double gamma_f,
               double t_c_offset_s, const std::function<void(YAML::Node&)>& tweak = nullptr,
               const std::array<double, kUr5eArmDof>& start_arm = kUr5eHome, bool arm = true) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    YAML::Node yaml = YAML::Load(TrackingYaml(topic_, p_c, a_d, gamma_f, t_c_offset_s));
    yaml["catching"]["robot"]["hand"]["T_hold"] = 0.02;
    if (tweak) {
      tweak(yaml);
    }
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_FALSE(ctrl_->IsSimOnlyDisabled()) << "precondition: the profile parked (reason "
                                             << static_cast<int>(ctrl_->GetParkReason()) << ")";
    ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_TRUE(ctrl_->AreTrialsEnabled()) << "precondition: the S7 supervisor is not wired";
    if (arm) {
      SetArmed(true);
    }
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    state_ = MakeState(start_arm);
    initial_mode_ = ctrl_->GetMode();
  }

  void SetArmed(bool on) {
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, on));
  }

  static ControllerState MakeState(const std::array<double, kUr5eArmDof>& arm) {
    ControllerState state{};
    state.num_devices = 2;
    state.dt = kDt;
    auto& a = state.devices[0];
    a.num_channels = kUr5eArmDof;
    a.valid = true;
    for (int i = 0; i < kUr5eArmDof; ++i) {
      a.positions[static_cast<std::size_t>(i)] = arm[static_cast<std::size_t>(i)];
    }
    auto& h = state.devices[1];
    h.num_channels = kP1bHandDof;
    h.valid = true;  // at q_pre (zeros), at rest
    return state;
  }

  void PublishNow() {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = 8;
    spec.sequence = seq_++;
    spec.generation = generation_;
    auto msg = integrated_bringup::testing::MakeCloud(spec);
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const std::int64_t wall =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - 5'000'000;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
    for (int i = 0; i < 8; ++i) {
      executor_->spin_some(2ms);
    }
    last_pub_ns_ = rtc::SteadyNowNs();
  }

  // ── Fingertips ────────────────────────────────────────────────────────────

  /// This tick's force per fingertip: the bias everywhere, plus a contact
  /// force on `contact_tips_` while the ball is (physically) in the hand —
  /// from DECEL (t_c) through HOLD, and through a RETREAT / ABORT_SAFE that
  /// still has the hand closed. Decided from the mode the LAST tick left.
  void UpdateTipForces() {
    const Mode m = ctrl_->GetMode();
    const auto& h = ctrl_->GetHandOutputForTesting();
    const bool holding = h.active && h.phase == HandPhase::kHold;
    const bool in_hand =
        ball_in_hand_ && (m == Mode::kDecel || m == Mode::kHold ||
                          ((m == Mode::kRetreat || m == Mode::kAbortSafe) && holding));
    for (int g = 0; g < kTips; ++g) {
      const auto u = static_cast<std::size_t>(g);
      tip_force_[u] = kTipBias;
      if (in_hand && contact_tips_[u]) {
        tip_force_[u] += kTipContact;
      }
    }
  }

  /// One NEW sample per fingertip per tick. A tip whose stamp is frozen keeps
  /// its old receive instant while its value (and sequence) still change: the
  /// fresh-looking OLD sample G7-G's negative control needs.
  void InjectTips() {
    auto& h = state_.devices[1];
    h.num_inference_groups = kTips;
    const std::int64_t now = rtc::SteadyNowNs();
    for (int g = 0; g < kTips; ++g) {
      const auto u = static_cast<std::size_t>(g);
      h.inference_enable[u] = true;
      if (!tip_stamp_frozen_[u]) {
        h.inference_recv_steady_ns[u] = now;
      }
      h.inference_sequence[u] += 1;
      const auto base = u * kTipStride;
      for (std::size_t k = 0; k < kTipStride; ++k) {
        h.inference_data[base + k] = 0.0F;
      }
      for (std::size_t k = 0; k < 3; ++k) {
        h.inference_data[base + 1 + k] = static_cast<float>(tip_force_[u][static_cast<long>(k)]);
      }
    }
  }

  // ── The loop ──────────────────────────────────────────────────────────────

  void Tick() {
    if (pre_tick_) {
      pre_tick_();
    }
    if (publishing_ && (last_pub_ns_ == 0 || rtc::SteadyNowNs() - last_pub_ns_ >= 30 * kMsNs)) {
      PublishNow();
    }
    if (tips_enabled_) {
      UpdateTipForces();
      InjectTips();
    }
    state_.iteration += 1;
    state_.t_relative_s = static_cast<double>(state_.iteration) * kDt;

    TickRec rec;
    for (int i = 0; i < kUr5eArmDof; ++i) {
      rec.q_meas[static_cast<std::size_t>(i)] =
          state_.devices[0].positions[static_cast<std::size_t>(i)];
    }
    rec.before_ns = rtc::SteadyNowNs();
    const ControllerOutput out = ctrl_->Compute(state_);
    rec.after_ns = rtc::SteadyNowNs();

    rec.mode = ctrl_->GetMode();
    rec.reason = ctrl_->GetLastReason();
    const auto& hand = ctrl_->GetHandOutputForTesting();
    rec.hand_active = hand.active;
    rec.phase = hand.phase;
    rec.hand_timeout = hand.timeout;
    rec.outcome = ctrl_->GetOutcomeForTesting();
    rec.homing = ctrl_->IsHomingForTesting();
    rec.track_err = ctrl_->GetTrackError();
    rec.armed_latch = ctrl_->IsArmRequested();
    rec.refusal = ctrl_->GetLastPlanRefusal();
    rec.admitted = ctrl_->GetPlanAdmittedCount();
    rec.plan_id = ctrl_->GetFollowedPlanForTesting().plan_id;
    rec.plan_t_c_ns = ctrl_->GetFollowedPlanForTesting().t_c_ns;
    const auto& qc = ctrl_->GetArmCommandForTesting();
    const auto& qdc = ctrl_->GetArmVelocityCommandForTesting();
    for (int i = 0; i < kUr5eArmDof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      rec.q_cmd[u] = qc[u];
      rec.qd_cmd[u] = qdc[u];
    }
    const auto record = ctrl_->GetLastTickRecord();
    rec.ref_valid = record.ref_valid;
    rec.ref_e = record.ref_e;
    rec.ref_ed = record.ref_ed;
    for (int g = 0; g < kTips; ++g) {
      const auto u = static_cast<std::size_t>(g);
      rec.tip_contact[u] = record.tip_contact[u];
      rec.tip_fresh[u] = record.tip_fresh[u];
    }

    // ── The plant ───────────────────────────────────────────────────────────
    std::array<double, kUr5eArmDof> cmd{};
    if (out.devices[0].num_channels >= kUr5eArmDof) {
      rec.q_out_valid = true;
      for (int i = 0; i < kUr5eArmDof; ++i) {
        const auto u = static_cast<std::size_t>(i);
        cmd[u] = out.devices[0].commands[u];
        rec.q_out[u] = cmd[u];
      }
    } else {
      for (int i = 0; i < kUr5eArmDof; ++i) {
        cmd[static_cast<std::size_t>(i)] = state_.devices[0].positions[static_cast<std::size_t>(i)];
      }
    }
    std::array<double, kUr5eArmDof> measured = cmd;
    if (lag_) {
      measured = lag_->Step(cmd);
    }
    for (int i = 0; i < kUr5eArmDof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      state_.devices[0].positions[u] = measured[u];
    }
    if (servo_hand_ && out.devices[1].num_channels >= kP1bHandDof) {
      for (int i = 0; i < kP1bHandDof; ++i) {
        const auto u = static_cast<std::size_t>(i);
        state_.devices[1].positions[u] = out.devices[1].commands[u];
      }
    }
    log_.push_back(rec);
    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
  }

  void Ticks(int n) {
    for (int i = 0; i < n; ++i) {
      Tick();
    }
  }

  /// Tick until `done()` holds after a tick; false after `max_ticks`.
  bool TickUntil(const std::function<bool()>& done, int max_ticks) {
    for (int i = 0; i < max_ticks; ++i) {
      Tick();
      if (done()) {
        return true;
      }
    }
    return false;
  }

  bool TickUntilMode(Mode m, int max_ticks) {
    return TickUntil([this, m] { return ctrl_->GetMode() == m; }, max_ticks);
  }

  /// ARMED without a ball for `ticks` ticks: the contact baseline is learned
  /// here (hand at q_pre, at rest) — n_baseline_min is 20 samples.
  void LearnBaselineInArmed(int ticks = 30) {
    const bool was = publishing_;
    publishing_ = false;
    Ticks(ticks);
    ASSERT_EQ(ctrl_->GetMode(), Mode::kArmed) << Transitions();
    publishing_ = was;
  }

  // ── Reading the log ──────────────────────────────────────────────────────

  std::vector<Mode> ModeSeq(std::size_t from = 0) const {
    std::vector<Mode> seq;
    seq.push_back(from == 0 ? initial_mode_ : log_[from - 1].mode);
    for (std::size_t i = from; i < log_.size(); ++i) {
      if (log_[i].mode != seq.back()) {
        seq.push_back(log_[i].mode);
      }
    }
    return seq;
  }

  /// Index of the n-th (0-based) tick that ENTERED `m`, or -1.
  int Entry(Mode m, int nth = 0, std::size_t from = 0) const {
    Mode prev = from == 0 ? initial_mode_ : log_[from - 1].mode;
    int seen = 0;
    for (std::size_t i = from; i < log_.size(); ++i) {
      if (log_[i].mode == m && prev != m) {
        if (seen == nth) {
          return static_cast<int>(i);
        }
        ++seen;
      }
      prev = log_[i].mode;
    }
    return -1;
  }

  int CountTicks(const std::function<bool(const TickRec&)>& pred, std::size_t from = 0,
                 std::size_t to = static_cast<std::size_t>(-1)) const {
    int n = 0;
    for (std::size_t i = from; i < log_.size() && i < to; ++i) {
      n += pred(log_[i]) ? 1 : 0;
    }
    return n;
  }

  double Ms(std::int64_t ns) const {
    return log_.empty() ? 0.0 : static_cast<double>(ns - log_.front().before_ns) * 1e-6;
  }

  std::string Line(std::size_t i) const {
    const auto& r = log_[i];
    std::ostringstream os;
    os.precision(4);
    os << "  #" << i << " t=" << std::fixed << Ms(r.before_ns) << "ms " << ModeName(r.mode) << " ("
       << ReasonName(r.reason) << ") hand=" << (r.hand_active ? PhaseName(r.phase) : "latch")
       << (r.hand_timeout ? "[timeout]" : "") << " outcome=" << OutcomeName(r.outcome)
       << " refusal=" << static_cast<int>(r.refusal) << " track_err=" << r.track_err << " |qd|inf=";
    double qd = 0.0;
    for (double v : r.qd_cmd) {
      qd = std::max(qd, std::abs(v));
    }
    os << qd << " tips(c/f)=";
    for (int g = 0; g < kTips; ++g) {
      os << r.tip_contact[static_cast<std::size_t>(g)] << r.tip_fresh[static_cast<std::size_t>(g)]
         << ' ';
    }
    os << '\n';
    return os.str();
  }

  /// Every mode change, with the reason that took it — the failure message.
  std::string Transitions() const {
    std::ostringstream os;
    os << "sequence: " << SeqString(ModeSeq()) << '\n';
    Mode prev = initial_mode_;
    for (std::size_t i = 0; i < log_.size(); ++i) {
      if (log_[i].mode != prev) {
        os << Line(i);
        prev = log_[i].mode;
      }
    }
    return os.str();
  }

  std::string Window(int center, int radius) const {
    std::string s;
    if (center < 0) {
      return s;
    }
    const int lo = std::max(0, center - radius);
    const int hi = std::min(static_cast<int>(log_.size()) - 1, center + radius);
    for (int i = lo; i <= hi; ++i) {
      s += Line(static_cast<std::size_t>(i));
    }
    return s;
  }

  void ExpectSeq(const std::vector<Mode>& expected, std::size_t from = 0) const {
    EXPECT_EQ(SeqString(ModeSeq(from)), SeqString(expected)) << Transitions();
  }

  bool ArmMeasuredAtWait(const TickRec& r) const {
    for (int i = 0; i < kUr5eArmDof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      if (!(std::abs(r.q_meas[u] - kUr5eHome[u]) <= kPoseTol)) {
        return false;
      }
    }
    return true;
  }

  /// DECEL at t_c on the LEAD axis (R-DECEL-ENTRY): the tick before did not
  /// see now_lead ≥ t_c and the entry tick did. Bounded by the stamps around
  /// the controller's own clock read, so it is exact rather than grid-based.
  void ExpectDecelAtTc(std::int64_t t_arm_ns = 0) const {
    const int d = Entry(Mode::kDecel);
    ASSERT_GT(d, 0) << Transitions();
    const auto& prev = log_[static_cast<std::size_t>(d - 1)];
    const auto& entry = log_[static_cast<std::size_t>(d)];
    const std::int64_t t_c = prev.plan_t_c_ns;
    ASSERT_GT(t_c, 0);
    EXPECT_LT(prev.before_ns + t_arm_ns, t_c)
        << "DECEL entered late: the previous tick already had now_lead >= t_c (late by "
        << static_cast<double>(prev.before_ns + t_arm_ns - t_c) * 1e-6 << " ms)\n"
        << Window(d, 3);
    EXPECT_GE(entry.after_ns + t_arm_ns, t_c) << "DECEL entered before t_c\n" << Window(d, 3);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<CatchFrameOracle> oracle_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<std::string> arm_names_;
  std::string topic_;
  pinocchio::SE3 start_pose_{pinocchio::SE3::Identity()};
  ControllerState state_{};
  Mode initial_mode_{Mode::kIdle};
  std::vector<TickRec> log_;

  // Ball lane.
  bool publishing_{true};
  std::uint64_t seq_{1};
  std::uint64_t generation_{42};
  std::int64_t last_pub_ns_{0};

  // Plant.
  bool servo_hand_{true};
  std::optional<integrated_bringup::testing::ArmLagPlant<kUr5eArmDof>> lag_;
  std::function<void()> pre_tick_;

  // Fingertips.
  const Eigen::Vector3d kTipBias{0.10, -0.05, 0.20};
  const Eigen::Vector3d kTipContact{0.0, 0.0, 1.5};  // far above f_min = 0.2 N
  bool tips_enabled_{false};
  bool ball_in_hand_{false};
  std::array<bool, kTips> contact_tips_{true, true, true, false};
  std::array<bool, kTips> tip_stamp_frozen_{};
  std::array<Eigen::Vector3d, kTips> tip_force_{};
};

// ════════════════════════════════════════════════════════════════════════════
// L7 §9 — the scenario table
// ════════════════════════════════════════════════════════════════════════════

TEST_F(SupervisorScenarioTest, NormalTrialIsCapturedAndReArms) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();

  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  ExpectDecelAtTc();

  // G7-B: the DECEL entry step is τ = 0 from the reference's own state, so the
  // reference error it records is zero.
  const int d = Entry(Mode::kDecel);
  ASSERT_GT(d, 0);
  const auto& entry = log_[static_cast<std::size_t>(d)];
  ASSERT_TRUE(entry.ref_valid) << "the DECEL entry tick did not step the reference";
  const double e = Eigen::Vector3d(entry.ref_e[0], entry.ref_e[1], entry.ref_e[2]).norm();
  const double ed = Eigen::Vector3d(entry.ref_ed[0], entry.ref_ed[1], entry.ref_ed[2]).norm();
  RecordProperty("decel_entry_ref_e_nm", static_cast<int>(e * 1e9));
  EXPECT_LT(e, 1e-9) << "|e| at DECEL entry " << e;
  EXPECT_LT(ed, 1e-9) << "|ė| at DECEL entry " << ed;

  // Q12: Captured keeps the hand closed until the arm is back, then releases.
  const int r = Entry(Mode::kRetreat);
  const int armed = Entry(Mode::kArmed, 1);
  ASSERT_GT(r, 0);
  ASSERT_GT(armed, r);
  int release = -1;
  for (int i = r; i < armed; ++i) {
    const auto& t = log_[static_cast<std::size_t>(i)];
    if (t.phase != HandPhase::kHold) {
      release = i;
      break;
    }
  }
  ASSERT_GT(release, r) << "the hand did not hold through the start of RETREAT\n" << Window(r, 3);
  EXPECT_EQ(log_[static_cast<std::size_t>(release)].phase, HandPhase::kRelease);
  EXPECT_TRUE(ArmMeasuredAtWait(log_[static_cast<std::size_t>(release)]))
      << "the hand released before the arm was back at the wait pose\n"
      << Window(release, 2);
  // Re-armed at the wait pose, hand at q_pre.
  const auto& back = log_[static_cast<std::size_t>(armed)];
  EXPECT_TRUE(ArmMeasuredAtWait(back));
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(back.q_cmd[static_cast<std::size_t>(i)], kUr5eHome[static_cast<std::size_t>(i)],
                1e-12)
        << "joint " << i;
  }
  for (int i = 0; i < kP1bHandDof; ++i) {
    EXPECT_NEAR(state_.devices[1].positions[static_cast<std::size_t>(i)], 0.0, 1e-9);
  }
}

TEST_F(SupervisorScenarioTest, ActivatedOutsideTheWaitPoseHomesThenArms) {
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;  // ≈ 3.4°
  off[2] -= 0.05;
  off[4] += 0.04;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, nullptr, off));
  publishing_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 800)) << "deadlock: never left IDLE\n" << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed});
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.homing; }), 10)
      << "IDLE never homed (a Q13 skip from outside the pose would be wrong)";
  const auto& qc = ctrl_->GetArmCommandForTesting();
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(qc[static_cast<std::size_t>(i)], kUr5eHome[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i << ": the carried command did not end on the wait pose";
  }
  EXPECT_FALSE(ctrl_->IsHomingForTesting());
  // The hand opens to q_open while homing and waits at q_pre after (Q4).
  EXPECT_EQ(ctrl_->GetHandOutputForTesting().phase, HandPhase::kPreshape);
}

TEST_F(SupervisorScenarioTest, AMissedBallIsJudgedMissedAndReleasesAtOnce) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = false;  // the fingertips see only their bias
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  const int r = static_cast<int>(log_.size()) - 1;
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kMissed) << Transitions();
  // Q12: Missed releases on RETREAT entry, before the return.
  EXPECT_EQ(log_[static_cast<std::size_t>(r)].phase, HandPhase::kRelease) << Window(r, 3);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
}

TEST_F(SupervisorScenarioTest, StaleBeforeTheFreezeRetreatsOnBallStale) {
  // t_c 1 s out: the approach lasts 0.64 s, far longer than t_stale = 0.2 s.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.0));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  PublishNow();
  publishing_ = false;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() != Mode::kApproach; }, 400))
      << Transitions();
  EXPECT_EQ(ctrl_->GetMode(), Mode::kRetreat) << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kBallStale) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kAborted);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq(
      {Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kRetreat, Mode::kArmed});
}

TEST_F(SupervisorScenarioTest, AShortStaleAfterTheFreezeIsOnlyRecorded) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 400)) << Transitions();
  // A 0.24 s gap: past t_stale (0.2) and short of t_stale + stale_committed_max
  // (0.3), so the lane is stale for ~40 ms and never long-stale.
  PublishNow();
  publishing_ = false;
  const std::int64_t gap_start = rtc::SteadyNowNs();
  pre_tick_ = [this, gap_start] {
    if (!publishing_ && rtc::SteadyNowNs() - gap_start >= 240 * kMsNs) {
      publishing_ = true;
    }
  };
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1000)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_GT(CountTicks([](const TickRec& t) {
              return (t.mode == Mode::kCommitted || t.mode == Mode::kClosing) &&
                     t.reason == Reason::kBallStaleCommitted;
            }),
            0)
      << "BallStaleCommitted was never recorded\n"
      << Transitions();
  ExpectDecelAtTc();
}

TEST_F(SupervisorScenarioTest, ALongStaleAfterTheFreezeAbortsOnBallStaleLong) {
  // With the shipped T_freeze (0.36) COMMITTED lasts only T_freeze − T_close_e2e
  // = 80 ms, shorter than the 0.1 s the long-stale bound adds on top of
  // t_stale, so a lane that goes quiet in COMMITTED is long-stale only in
  // CLOSING. The freeze is widened (0.8 s, window 0.52 s) so the abort the
  // table names — COMMITTED → ABORT_SAFE — is reachable.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.1, [](YAML::Node& y) {
    y["catching"]["planner"]["freeze"]["T_freeze"] = 0.8;
  }));
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 400)) << Transitions();
  publishing_ = false;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() != Mode::kCommitted; }, 400))
      << Transitions();
  EXPECT_EQ(ctrl_->GetMode(), Mode::kAbortSafe) << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kBallStaleLong) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 800)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kAbortSafe, Mode::kRetreat});
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kAborted);
}

TEST_F(SupervisorScenarioTest, WithNoCatchablePlanTrackingStaysAndRecordsIt) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["enabled"] = false;  // no planner either: the box stays empty
  }));
  Ticks(80);
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking});
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kNoCatchablePlan);
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kInvalid);
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 0U);
}

TEST_F(SupervisorScenarioTest, SaturationBeforeTheFreezeRetreatsOnRefSaturated) {
  // A far target saturates the reference from its first step; five saturated
  // ticks in a row give it up while the approach still has ~0.6 s to run.
  const Eigen::Vector3d far = start_pose_.translation() + Eigen::Vector3d(0.45, -0.35, 0.30);
  ASSERT_NO_FATAL_FAILURE(BringUp(far, StartAxis(), 0.0, 1.0, [](YAML::Node& y) {
    y["catching"]["supervisor"]["sat_ticks"] = 5;
  }));
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() == Mode::kRetreat; }, 400))
      << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kRefSaturated) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kRetreat});
}

TEST_F(SupervisorScenarioTest, SaturationAfterTheFreezeAbortsOnRefSaturated) {
  // The streak straddles the freeze: t_c is 0.4 s out when the plan is taken,
  // so APPROACH lasts ~40 ms (< 40 ticks), and the 40-tick streak a far target
  // builds from its first step completes inside COMMITTED (80 ms long).
  const Eigen::Vector3d far = start_pose_.translation() + Eigen::Vector3d(0.45, -0.35, 0.30);
  ASSERT_NO_FATAL_FAILURE(BringUp(far, StartAxis(), 0.0, 0.4, [](YAML::Node& y) {
    y["catching"]["supervisor"]["sat_ticks"] = 40;
  }));
  ASSERT_TRUE(TickUntil(
      [this] {
        const Mode m = ctrl_->GetMode();
        return m == Mode::kAbortSafe || m == Mode::kRetreat || m == Mode::kClosing;
      },
      400))
      << Transitions();
  EXPECT_EQ(ctrl_->GetMode(), Mode::kAbortSafe) << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kRefSaturated) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kAbortSafe});
}

TEST_F(SupervisorScenarioTest, QpFailuresAbortAndTheThirdLatchesAFaultThatResetClears) {
  // A plan with a degenerate approach axis fails CLIK on its first law tick.
  // Each retry needs a NEW ball (Q15), and n_qp = 3 in a row latches the fault.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["a_d"] = YAML::Load("[0.0, 0.0, 0.0]");
  }));
  Mode last = ctrl_->GetMode();
  ASSERT_TRUE(TickUntil(
      [this, &last] {
        const Mode m = ctrl_->GetMode();
        if (m == Mode::kAbortSafe && last != Mode::kAbortSafe) {
          ++generation_;  // the thrower retries with a new ball
        }
        last = m;
        return m == Mode::kFault;
      },
      600))
      << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kFault});
  for (int k = 0; k < 3; ++k) {
    const int a = Entry(Mode::kAbortSafe, k);
    ASSERT_GE(a, 0);
    EXPECT_EQ(log_[static_cast<std::size_t>(a)].reason, Reason::kQpFailed) << Window(a, 2);
  }
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kAbortEscalated);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 3U);

  // Fault reset → IDLE, disarmed (no automatic resume).
  const std::size_t mark = log_.size();
  ctrl_->ResetFault();
  Ticks(30);
  ExpectSeq({Mode::kFault, Mode::kIdle}, mark);
  EXPECT_EQ(log_[mark].reason, Reason::kFaultReset) << Window(static_cast<int>(mark), 2);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_FALSE(ctrl_->IsArmRequested());
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle);
}

TEST_F(SupervisorScenarioTest, EstopGoesToIdleAndTheClearDoesNotResume) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 1.0));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  // Moving, and already further from the wait pose than pose_tol, so the
  // re-arm below cannot take the Q13 skip.
  ASSERT_TRUE(TickUntil(
      [this] {
        double d = 0.0;
        for (int i = 0; i < kUr5eArmDof; ++i) {
          const auto u = static_cast<std::size_t>(i);
          d = std::max(d, std::abs(state_.devices[0].positions[u] - kUr5eHome[u]));
        }
        return d > 2.0 * kPoseTol;
      },
      300))
      << Transitions();
  ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();

  const std::size_t trig = log_.size();
  ctrl_->TriggerEstop();
  Ticks(20);
  EXPECT_EQ(log_[trig].mode, Mode::kIdle) << Window(static_cast<int>(trig), 2);
  EXPECT_EQ(log_[trig].reason, Reason::kEstop);
  EXPECT_FALSE(ctrl_->IsArmRequested());

  const std::size_t clear = log_.size();
  ctrl_->ClearEstop();
  Ticks(100);  // the ball is still being published
  ExpectSeq({Mode::kApproach, Mode::kIdle}, trig);
  EXPECT_FALSE(ctrl_->IsArmRequested()) << "the clear re-armed the controller";
  // q_c = q_meas after the clear: the command is the measured pose, and it
  // stays there (nothing moves a disarmed IDLE).
  for (std::size_t i = clear; i < log_.size(); ++i) {
    ASSERT_TRUE(log_[i].q_out_valid);
    for (int j = 0; j < kUr5eArmDof; ++j) {
      const auto u = static_cast<std::size_t>(j);
      ASSERT_DOUBLE_EQ(log_[i].q_out[u], log_[clear].q_meas[u])
          << "tick " << i << " joint " << j << ": the arm moved after the clear";
    }
  }
  // Re-arming deliberately starts from homing (the arm stopped off the pose).
  const std::size_t rearm = log_.size();
  SetArmed(true);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.homing; }, rearm), 0);
}

TEST_F(SupervisorScenarioTest, AnUnarmableProfileParksAndAnUnarmedControllerStaysIdle) {
  // (1) A consumed value still TBD: configure succeeds PARKED and activation is
  // refused, so there is no active-but-unarmable controller to tick (armable_
  // is false only when on_configure never ran — the unit suite's fixture).
  {
    DemoCatchingController parked{""};
    parked.SetSystemModelConfig(MakeConfigWithCatchFrame());
    parked.SetSharedModelBuilder(builder_);
    parked.SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    YAML::Node yaml = YAML::Load(TrackingYaml(topic_, NearPc(), StartAxis(), 0.0, 0.6));
    yaml["catching"]["robot"]["hand"]["T_close_e2e"] = "TBD";
    const rclcpp_lifecycle::State prev;
    auto park_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_supervisor_park");
    ASSERT_EQ(parked.on_configure(prev, park_node, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    EXPECT_TRUE(parked.IsSimOnlyDisabled());
    EXPECT_NE(parked.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  }
  // (2) Active and not armed: IDLE forever, on the documented kParamsTbd reuse.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, nullptr, kUr5eHome,
                                  /*arm=*/false));
  Ticks(60);
  ExpectSeq({Mode::kIdle});
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kParamsTbd);
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.reason != Reason::kParamsTbd; }), 0);
}

// ════════════════════════════════════════════════════════════════════════════
// S7 additions (L7 §4.8, the S7.2 driver rules)
// ════════════════════════════════════════════════════════════════════════════

TEST_F(SupervisorScenarioTest, DisarmDuringTheReturnRampsTheArmDownInIdle) {
  const std::vector<double> qdd = DerivedQddMax();
  ASSERT_EQ(static_cast<int>(qdd.size()), kUr5eArmDof);
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 0.6));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  // Wait for the RETURN (not the stop): the carried speed rising past 0.05.
  double prev_speed = 0.0;
  ASSERT_TRUE(TickUntil(
      [this, &prev_speed] {
        double s = 0.0;
        for (double v : ctrl_->GetArmVelocityCommandForTesting()) {
          s = std::max(s, std::abs(v));
        }
        const bool rising = s > prev_speed;
        prev_speed = s;
        return ctrl_->GetMode() == Mode::kRetreat && rising && s > 0.05;
      },
      800))
      << "the return never reached speed\n"
      << Transitions();
  const std::size_t last_retreat = log_.size() - 1;
  SetArmed(false);
  ASSERT_TRUE(TickUntil(
      [this] {
        for (double v : ctrl_->GetArmVelocityCommandForTesting()) {
          if (v != 0.0) {
            return false;
          }
        }
        return true;
      },
      600))
      << Transitions();
  Ticks(5);
  ExpectSeq({Mode::kRetreat, Mode::kIdle}, last_retreat);
  const std::size_t first_idle = last_retreat + 1;
  ASSERT_EQ(log_[first_idle].mode, Mode::kIdle) << Window(static_cast<int>(first_idle), 2);
  EXPECT_EQ(log_[first_idle].reason, Reason::kParamsTbd);
  double speed0 = 0.0;
  for (double v : log_[first_idle].qd_cmd) {
    speed0 = std::max(speed0, std::abs(v));
  }
  EXPECT_GT(speed0, 0.0) << "the command stopped in one tick — a velocity step, not a ramp";
  int ramp_ticks = 0;
  for (std::size_t i = first_idle; i < log_.size(); ++i) {
    bool moving = false;
    for (int j = 0; j < kUr5eArmDof; ++j) {
      const auto u = static_cast<std::size_t>(j);
      const double dqd = std::abs(log_[i].qd_cmd[u] - log_[i - 1].qd_cmd[u]);
      EXPECT_LE(dqd, qdd[u] * kDt * (1.0 + 1e-9) + 1e-12)
          << "tick " << i << " joint " << j << ": |Δq̇| beyond q̈_max·dt";
      EXPECT_LE(std::abs(log_[i].qd_cmd[u]), std::abs(log_[i - 1].qd_cmd[u]) + 1e-12)
          << "tick " << i << " joint " << j << ": the stop sped a joint up";
      moving = moving || log_[i].qd_cmd[u] != 0.0;
    }
    ramp_ticks += moving ? 1 : 0;
  }
  EXPECT_GT(ramp_ticks, 1) << "the ramp took a single tick";
  RecordProperty("disarm_ramp_ticks", ramp_ticks);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle);
}

TEST_F(SupervisorScenarioTest, AStaleSpanningTcmdToTcStillDeceleratesAtTc) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  ASSERT_TRUE(TickUntilMode(Mode::kClosing, 600)) << Transitions();
  Ticks(5);
  // The last prediction ~t_cmd + 10 ms: at t_c it is ~0.27 s old — stale
  // (> 0.2) through the end of CLOSING, never long-stale (> 0.3).
  PublishNow();
  publishing_ = false;
  pre_tick_ = [this] {
    if (!publishing_ && ctrl_->GetMode() == Mode::kHold) {
      publishing_ = true;
    }
  };
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 800)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_GT(CountTicks([](const TickRec& t) {
              return t.mode == Mode::kClosing && t.reason == Reason::kBallStaleCommitted;
            }),
            0)
      << "the gap never went stale inside CLOSING — the case did not test anything\n"
      << Transitions();
  ExpectDecelAtTc();
}

TEST_F(SupervisorScenarioTest, AHandTimeoutDuringHoldDoesNotDelayTheEndOfHold) {
  // The hand is not servoed: ρ stays 0, so Close ends on T_close_timeout
  // (2·T_close_e2e = 0.56 s after the close), which lands inside a 0.5 s HOLD.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["catching"]["robot"]["hand"]["T_hold"] = 0.5;
  }));
  servo_hand_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat});
  const int h = Entry(Mode::kHold);
  const int r = Entry(Mode::kRetreat);
  ASSERT_GT(h, 0);
  ASSERT_GT(r, h);
  // The timeout happened INSIDE HOLD…
  int timeout_in_hold = -1;
  for (int i = h; i < r; ++i) {
    if (log_[static_cast<std::size_t>(i)].hand_timeout) {
      timeout_in_hold = i;
      break;
    }
  }
  ASSERT_GT(timeout_in_hold, h) << "precondition: the hand timed out outside HOLD\n"
                                << Transitions();
  // …and HOLD still ended T_hold after it began. hold_entry lies in
  // [before_h, after_h]; the tick before RETREAT saw now − hold_entry < T_hold
  // and the RETREAT tick saw ≥ T_hold, which the stamps bound exactly. The
  // upper bound on the end allows 10 ms of host scheduling.
  constexpr std::int64_t kTHoldNs = 500 * kMsNs;
  const auto& hold = log_[static_cast<std::size_t>(h)];
  const auto& last_hold = log_[static_cast<std::size_t>(r - 1)];
  const auto& retreat = log_[static_cast<std::size_t>(r)];
  EXPECT_LT(last_hold.before_ns - hold.after_ns, kTHoldNs) << "HOLD ran past T_hold";
  EXPECT_GE(retreat.after_ns - hold.before_ns, kTHoldNs) << "HOLD ended before T_hold";
  EXPECT_LT(retreat.before_ns - hold.after_ns, kTHoldNs + 10 * kMsNs)
      << "HOLD ended " << static_cast<double>(retreat.before_ns - hold.after_ns) * 1e-6
      << " ms after it began";
}

TEST_F(SupervisorScenarioTest, ATrackErrAbortReturnsAndReArms) {
  // The plain TRACK_ERR abort (no contact, before the freeze): APPROACH →
  // ABORT_SAFE → RETREAT → ARMED. The measured arm is 0.6 rad off for ONE
  // tick; from the next tick the servo is perfect again, so the return has no
  // tracking error of its own.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.0));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  Ticks(20);
  state_.devices[0].positions[1] += 0.6;
  const std::size_t kick = log_.size();
  Ticks(1);
  ASSERT_EQ(log_[kick].mode, Mode::kAbortSafe) << Window(static_cast<int>(kick), 3);
  EXPECT_EQ(log_[kick].reason, Reason::kTrackErr);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 400)) << Transitions();
  const bool rearmed = TickUntilMode(Mode::kArmed, 400);
  const int r = Entry(Mode::kRetreat);
  EXPECT_TRUE(rearmed) << "RETREAT never re-armed after a TRACK_ERR abort\n" << Window(r, 6);
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed});
}

TEST_F(SupervisorScenarioTest, AnAbortAfterConfirmedContactKeepsTheBallUntilTheReturn) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["catching"]["robot"]["hand"]["T_hold"] = 0.3;
  }));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kHold, 1000)) << Transitions();
  Ticks(3);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kHold) << Transitions();
  const auto& now_rec = log_.back();
  ASSERT_GE(std::count(now_rec.tip_contact.begin(), now_rec.tip_contact.end(), true), 2)
      << "precondition: contact is confirmed before the abort\n"
      << Window(static_cast<int>(log_.size()) - 1, 3);
  // TRACK_ERR: the measured arm the next tick sees is 0.6 rad off its command
  // (> track_err_abort 0.5) — one tick only; the servo puts it back after.
  state_.devices[0].positions[1] += 0.6;
  const std::size_t kick = log_.size();
  Ticks(1);
  EXPECT_EQ(log_[kick].mode, Mode::kAbortSafe) << Window(static_cast<int>(kick), 3);
  EXPECT_EQ(log_[kick].reason, Reason::kTrackErr);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 600)) << Transitions();
  const int r = static_cast<int>(log_.size()) - 1;
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kAborted);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kAbortSafe, Mode::kRetreat,
             Mode::kArmed});
  // Q14: the hand stays closed through the return, and lets go only at the pose.
  int release = -1;
  for (std::size_t i = static_cast<std::size_t>(r); i < log_.size(); ++i) {
    if (log_[i].phase != HandPhase::kHold) {
      release = static_cast<int>(i);
      break;
    }
  }
  ASSERT_GT(release, r) << "the hand let go on RETREAT entry (Q14)\n" << Window(r, 3);
  EXPECT_EQ(log_[static_cast<std::size_t>(release)].phase, HandPhase::kRelease);
  EXPECT_TRUE(ArmMeasuredAtWait(log_[static_cast<std::size_t>(release)]))
      << "released before the arm was back\n"
      << Window(release, 2);
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kAborted)
      << "the verdict must survive the re-arm (reset table: outcome_ exempt from R)";
}

TEST_F(SupervisorScenarioTest, TheHandClosesAtTcmdOnTheRealAxisUnderAnArmLag) {
  constexpr int kDelayTicks = 25;  // 50 ms
  constexpr std::int64_t kTArmNs = kDelayTicks * kHNs;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["catching"]["joint_cmd"]["lag"]["T_arm"] = static_cast<double>(kTArmNs) * 1e-9;
    y["catching"]["joint_cmd"]["lag"]["lead_enable"] = true;  // else now_lead = now
  }));
  lag_.emplace(kDelayTicks, kUr5eHome);
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 2000)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  ExpectDecelAtTc(kTArmNs);

  // The close: the first tick with now ≥ t_cmd − h/2 on the REAL axis. A
  // lead-axis close would be T_arm = 50 ms early and fail the first bound.
  int c = -1;
  for (std::size_t i = 0; i < log_.size(); ++i) {
    if (log_[i].hand_active && log_[i].phase == HandPhase::kClose) {
      c = static_cast<int>(i);
      break;
    }
  }
  ASSERT_GT(c, 0) << Transitions();
  const auto& at = log_[static_cast<std::size_t>(c)];
  const auto& before = log_[static_cast<std::size_t>(c - 1)];
  // R-CLOSE: COMMITTED → CLOSING on the close_issued cue. The sequencer runs
  // AFTER the decision in Compute(), so the cue is read one tick later: the
  // mode may lag the wire by one tick, never more.
  const int closing = Entry(Mode::kClosing);
  EXPECT_TRUE(closing == c || closing == c + 1)
      << "CLOSING entered at #" << closing << ", the close went out at #" << c << '\n'
      << Window(c, 3);
  RecordProperty("closing_lag_ticks", closing - c);
  const std::int64_t t_cmd = at.plan_t_c_ns - kTCloseE2eNs;
  EXPECT_GE(at.after_ns, t_cmd - kHNs / 2) << "closed early (lead axis?)";
  EXPECT_LT(before.before_ns, t_cmd - kHNs / 2) << "closed late: an earlier tick was due";
  // |now_close − t_cmd| ≤ h/2 + slack, the slack being how much longer than h
  // this tick's interval actually was (sleep + compute on a non-RT host).
  const std::int64_t interval = at.before_ns - before.before_ns;
  const std::int64_t slack = std::max<std::int64_t>(0, interval - kHNs);
  const std::int64_t err = at.before_ns - t_cmd;
  RecordProperty("close_error_us", static_cast<int>(err / 1000));
  RecordProperty("close_tick_interval_us", static_cast<int>(interval / 1000));
  EXPECT_LE(std::abs(err), kHNs / 2 + slack + 200'000)
      << "close error " << static_cast<double>(err) * 1e-6 << " ms, interval "
      << static_cast<double>(interval) * 1e-6 << " ms";
}

TEST_F(SupervisorScenarioTest, TwoConsecutiveTrialsShareNothingButThePose) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 600)) << Transitions();
  const std::uint32_t first_plan = ctrl_->GetFollowedPlanForTesting().plan_id;
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 1U);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1000)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  // Back at the wait pose with the hand at q_pre BEFORE the second throw.
  const auto& rearm = log_.back();
  EXPECT_TRUE(ArmMeasuredAtWait(rearm));
  for (int i = 0; i < kP1bHandDof; ++i) {
    EXPECT_NEAR(state_.devices[1].positions[static_cast<std::size_t>(i)], 0.0, 1e-9);
  }
  // Q15: the same ball keeps being published; ARMED does not start on it. The
  // baseline is re-learned meanwhile (ResetForRearm emptied it).
  const std::size_t mark = log_.size();
  Ticks(40);
  ExpectSeq({Mode::kArmed}, mark);
  ++generation_;  // the second throw
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 600)) << Transitions();
  const std::uint32_t second_plan = ctrl_->GetFollowedPlanForTesting().plan_id;
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 2U) << "one admission per trial";
  EXPECT_NE(second_plan, first_plan) << "the second trial followed the first trial's plan";
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1000)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed,
             Mode::kTracking, Mode::kApproach, Mode::kCommitted, Mode::kClosing, Mode::kDecel,
             Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 2U);
}

TEST_F(SupervisorScenarioTest, AnImmediateReThrowRefusesThePlansOfTheAbortedTrial) {
  // Oracle off: this test is the planner, writing the box itself.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["enabled"] = false;
  }));
  const auto plan_for = [this](std::uint64_t generation, std::uint32_t id,
                               std::int64_t publish_ns) {
    PlanSnapshot p{};
    p.valid = true;
    p.token.activation_generation = ctrl_->GetPlannerRtState().activation_generation;
    p.token.generation = generation;
    p.plan_id = id;
    const Eigen::Vector3d pc = NearPc();
    const Eigen::Vector3d ad = StartAxis();
    p.p_c = {pc.x(), pc.y(), pc.z()};
    p.a_d = {ad.x(), ad.y(), ad.z()};
    p.publish_ns = publish_ns;
    p.gamma_t0_ns = publish_ns;
    p.t_c_ns = publish_ns + 2'000'000'000;
    p.t_cmd_ns = p.t_c_ns;
    p.gamma_t1_ns = p.t_c_ns;
    return p;
  };
  ASSERT_TRUE(TickUntilMode(Mode::kTracking, 100)) << Transitions();
  ctrl_->PlanBoxForTesting().Store(plan_for(42, 1, rtc::SteadyNowNs()));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 5)) << Transitions();
  Ticks(20);
  // The re-throw: a new ball, which ends the attempt (TRACK_CHANGED).
  generation_ = 43;
  PublishNow();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 5)) << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kTrackChanged);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  const std::int64_t before_rearm = log_.back().before_ns;
  ASSERT_TRUE(TickUntilMode(Mode::kTracking, 20)) << Transitions();

  // (1) The box still holds the aborted trial's plan: refused (another track).
  Ticks(3);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kTracking) << Transitions();
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kTrack);
  // (2) A plan for the NEW ball, published before the re-arm: refused by the
  // reset floor (JudgePlan (f)), not by its age.
  ctrl_->PlanBoxForTesting().Store(plan_for(43, 2, before_rearm));
  Ticks(3);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kTracking) << Transitions();
  EXPECT_EQ(ctrl_->GetLastPlanRefusal(), PlanRefusal::kBeforeReset);
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kNoCatchablePlan);
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 1U);
  // Positive control: the same plan published now is taken.
  ctrl_->PlanBoxForTesting().Store(plan_for(43, 3, rtc::SteadyNowNs()));
  Ticks(1);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 2U);
  EXPECT_EQ(ctrl_->GetFollowedPlanForTesting().plan_id, 3U);
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kRetreat,
             Mode::kArmed, Mode::kTracking, Mode::kApproach});
}

TEST_F(SupervisorScenarioTest, AStaleFingertipIsNeverContactAndMakesTheOutcomeUndetermined) {
  // G7-G. Fingertip 0 keeps delivering NEW samples carrying a contact force,
  // but its receive stamp stops moving at the close: an old sample dressed as
  // new. Fingertip 1 is a genuine contact. m_min = 2, so without the freshness
  // gate the two would make a Captured.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  contact_tips_ = {true, true, false, false};
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kClosing, 600)) << Transitions();
  tip_stamp_frozen_[0] = true;
  const std::size_t frozen = log_.size();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1000)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat});
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kUndetermined) << Transitions();
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.tip_contact[0]; }), 0)
      << "the stale fingertip was judged in contact";
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.tip_contact[1]; }), 0)
      << "positive control: the fresh fingertip never confirmed contact";
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.reason == Reason::kTipStale; }, frozen), 0)
      << "TIP_STALE was never recorded\n"
      << Transitions();
  // Undetermined from HOLD keeps the hand closed into RETREAT (Q12).
  EXPECT_EQ(log_.back().phase, HandPhase::kHold);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
