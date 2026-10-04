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
// TIME IS STEPPED, NOT SLEPT. The controller reads its clock through
// SetClockForTesting, and this file hands it a fake one that advances one
// control period per tick; every stamp the file takes (fingertip receipt,
// plan publish, the before/after pair around Compute()) reads the same clock.
// Timing assertions are written against the before/after pair, so they bound
// the controller's own clock read rather than assuming a tick grid — on the
// fake clock the pair collapses onto that read, which makes them exact.
// The *RealClock fixtures run representative cases on the real steady clock,
// sleeping one period per tick, so the production clock path stays covered.
//
// The one production seam is the clock; every observation is an existing
// getter.

#include "arm_lag_fixture.hpp"
#include "catching_cloud_fixture.hpp"
#include "catching_decel_segment_fixture.hpp"
#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/catching/decel_target.hpp"
#include "rtc_controllers/catching/hand_sequencer.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::AccelBoxFlag;
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::FakeSteadyClock;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::SharedCatchFrameBuilder;
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
constexpr std::int64_t kHNs = 2 * kMsNs;  // state.dt, the tick the sequencer rounds to
static_assert(kHNs == integrated_bringup::testfx::kDtNs, "the fake clock steps state.dt");
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

/// The acceleration box (`robot.arm.qdd_max`), read from the shipped profile the
/// controller reads — the bound every joint-space ramp must respect per tick.
std::vector<double> DerivedQddMax() {
  return integrated_bringup::testfx::ShippedQddMax();
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
  std::array<double, kUr5eArmDof> q_cmd{};     // carried command (GetArmCommandForTesting)
  std::array<double, kUr5eArmDof> qd_cmd{};    // carried velocity
  std::array<double, kP1bHandDof> hand_out{};  // the hand command it wrote
  bool hand_out_valid{false};
  bool ref_valid{false};
  std::array<double, 3> ref_e{};
  std::array<double, 3> ref_ed{};
  std::array<bool, kTips> tip_contact{};
  std::array<bool, kTips> tip_fresh{};
  bool hand_blocked{false};                       // D-S8-8 (b): the stall has run unbroken
  std::uint8_t outcome_source{0};                 // 0 none, 1 fingertips, 2 hand, 3 both
  std::uint64_t iteration{0};                     // the state.iteration this tick was given
  double t_relative_s{0.0};                       // … and its t_relative_s
  integrated_bringup::CatchingDiagLogPod body{};  // the row this tick published (G8-H)
  // What the tick told the planner about the segments it holds (PlannerRtState).
  bool rt_decel_active{false};
  std::uint32_t rt_decel_seq{0};
  bool rt_decel_pending{false};
  std::uint32_t rt_decel_pending_seq{0};
};

class SupervisorScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // A real instant, so stamps look like the ones the controller sees in the
    // field; from here on only Tick() moves it.
    FakeSteadyClock::Restart();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_supervisor_scenarios");
    builder_ = SharedCatchFrameBuilder();
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

  /// The clock the controller reads (file header).
  [[nodiscard]] std::int64_t Now() const noexcept {
    return real_clock_ ? rtc::SteadyNowNs() : FakeSteadyClock::Now();
  }

  /// TrackingYaml plus `T_hold` shortened to 0.02 s (applied BEFORE `tweak`,
  /// so a case can set its own). Arms the controller unless `arm` is false.
  void BringUp(const Eigen::Vector3d& p_c, const Eigen::Vector3d& a_d, double gamma_f,
               double t_c_offset_s, const std::function<void(YAML::Node&)>& tweak = nullptr,
               const std::array<double, kUr5eArmDof>& start_arm = kUr5eHome, bool arm = true) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    if (!real_clock_) {
      ctrl_->SetClockForTesting(&FakeSteadyClock::Now);
    }
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
    if (hand_max_torque_) {
      // Only the torque scale: the empty position/velocity lists keep their
      // fallbacks, so nothing else about the hand changes.
      rtc::DeviceJointLimits limits;
      limits.max_torque = *hand_max_torque_;
      configs.at("p1b").joint_limits = limits;
    }
    ctrl_->SetDeviceNameConfigs(configs);
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
    ASSERT_EQ(ctrl_->GetPlannerThread(), nullptr)
        << "precondition: the planner wakes on the real clock (SetClockForTesting)";
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
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - stamp_age_ns_;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
    for (int i = 0; i < 8; ++i) {
      executor_->spin_some(2ms);
    }
    last_pub_ns_ = Now();
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
    const std::int64_t now = Now();
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
    if (publishing_ && (last_pub_ns_ == 0 || Now() - last_pub_ns_ >= 30 * kMsNs)) {
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
    rec.before_ns = Now();
    const ControllerOutput out = ctrl_->Compute(state_);
    rec.after_ns = Now();

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
    rec.hand_blocked = record.hand_blocked_s > 0.0;
    rec.outcome_source = record.outcome_source;
    rec.iteration = state_.iteration;
    rec.t_relative_s = state_.t_relative_s;
    rec.body = record;
    const rtc::catching::PlannerRtState rt = ctrl_->GetPlannerRtState();
    rec.rt_decel_active = rt.decel_active;
    rec.rt_decel_seq = rt.decel_seq;
    rec.rt_decel_pending = rt.decel_pending;
    rec.rt_decel_pending_seq = rt.decel_pending_seq;

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
    if (out.devices[1].num_channels >= kP1bHandDof) {
      rec.hand_out_valid = true;
      for (int i = 0; i < kP1bHandDof; ++i) {
        const auto u = static_cast<std::size_t>(i);
        rec.hand_out[u] = out.devices[1].commands[u];
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
    if (hand_stall_) {
      // One finger stopped on the ball while the sequencer holds q_close: it
      // stays short of its command and pushes (D-S8-8 (b)). Outside Hold it
      // follows the servo and carries no load.
      auto& d = state_.devices[1];
      const auto j = static_cast<std::size_t>(hand_stall_->joint);
      const bool holding = hand.active && hand.phase == HandPhase::kHold;
      if (holding) {
        d.positions[j] = hand_stall_->q;
      }
      d.efforts[j] = holding ? hand_stall_->effort : 0.0;
    }
    log_.push_back(rec);
    if (real_clock_) {
      std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
    } else {
      FakeSteadyClock::Step();
    }
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

  /// The RETREAT release rule (#537 S7, 2026-09-24, replacing Q12/Q14's
  /// outcome split): a hand that closed stays closed through the whole return
  /// and lets go only once the arm is back at the wait pose — WHATEVER the
  /// verdict. `r` is the RETREAT entry; the log must already run to the re-arm.
  void ExpectHeldUntilTheWaitPose(int r) const {
    ASSERT_GT(r, 0);
    int release = -1;
    for (std::size_t i = static_cast<std::size_t>(r); i < log_.size(); ++i) {
      if (log_[i].phase != HandPhase::kHold) {
        release = static_cast<int>(i);
        break;
      }
    }
    ASSERT_GT(release, r) << "the hand let go on RETREAT entry\n" << Window(r, 3);
    EXPECT_EQ(log_[static_cast<std::size_t>(release)].phase, HandPhase::kRelease);
    EXPECT_TRUE(ArmMeasuredAtWait(log_[static_cast<std::size_t>(release)]))
        << "the hand released before the arm was back at the wait pose\n"
        << Window(release, 2);
  }

  /// NormalTrialIsCapturedAndReArms, shared by both clocks. `tweak` is the
  /// profile change a case runs the same trial under (none by default).
  void NormalTrialCase(const std::function<void(YAML::Node&)>& tweak = nullptr) {
    ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, tweak));
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

    // The hand keeps the ball until the arm is back, then releases.
    const int r = Entry(Mode::kRetreat);
    const int armed = Entry(Mode::kArmed, 1);
    ASSERT_GT(r, 0);
    ASSERT_GT(armed, r);
    ASSERT_NO_FATAL_FAILURE(ExpectHeldUntilTheWaitPose(r));
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

  /// #749: `t` is a tick that ended in a stop (ABORT_SAFE, or RETREAT's stop
  /// stage) after its law had already stepped the command. The command that
  /// left it must be ONE step of the joint-space ramp from the command the
  /// tick started with — not the ramp's step on top of the law's.
  ///
  /// Two assertions, because they fail differently. The bound is the contract:
  /// no joint moves further than it did the tick before plus what q̈max allows
  /// in one tick. The equality names the mechanism: the ramp
  /// (JointSpaceDecelStep, the function the stop itself runs) applied to the
  /// carried command of tick t − 1 reproduces what went out. Both are trivially
  /// true for an arm at rest, hence the precondition.
  void ExpectTheStopTakesOverInOneStep(std::size_t t, const char* row) {
    ASSERT_GE(t, 2U);
    if (log_.size() == t + 1) {
      Ticks(1);  // the tick after, for the record only
    }
    const TickRec& entry = log_[t];
    const TickRec& before = log_[t - 1];
    const TickRec& before2 = log_[t - 2];
    ASSERT_TRUE(entry.q_out_valid && before.q_out_valid && before2.q_out_valid)
        << Window(static_cast<int>(t), 3);
    ASSERT_TRUE(before.body.clik_ran) << "precondition: the law was not running before the stop\n"
                                      << Window(static_cast<int>(t), 3);
    ASSERT_TRUE(entry.body.clik_ran)
        << "precondition: the law did not step the command on the tick the stop took over\n"
        << Window(static_cast<int>(t), 3);
    const std::vector<double> qdd = DerivedQddMax();
    ASSERT_GE(qdd.size(), static_cast<std::size_t>(kUr5eArmDof));

    std::array<double, kUr5eArmDof> q = before.q_cmd;
    std::array<double, kUr5eArmDof> qd = before.qd_cmd;
    // The box the controller's own ramp clamps to, so a stop that reaches it
    // is the same step here.
    const std::vector<double>& lower = ctrl_->GetArmPositionBoxLowerForTesting();
    const std::vector<double>& upper = ctrl_->GetArmPositionBoxUpperForTesting();
    ASSERT_GE(lower.size(), static_cast<std::size_t>(kUr5eArmDof));
    ASSERT_GE(upper.size(), static_cast<std::size_t>(kUr5eArmDof));
    const auto ramp =
        rtc::catching::JointSpaceDecelStep(q, qd, qdd, lower, upper, kUr5eArmDof, kDt);
    ASSERT_TRUE(ramp.valid);

    double moved = 0.0;
    double step_entry = 0.0;
    double step_after = 0.0;
    double worst = 0.0;
    for (int j = 0; j < kUr5eArmDof; ++j) {
      const auto u = static_cast<std::size_t>(j);
      const double last = std::abs(before.q_out[u] - before2.q_out[u]);
      const double step = std::abs(entry.q_out[u] - before.q_out[u]);
      const double allowed = last + qdd[u] * kDt * kDt;
      moved = std::max(moved, last);
      step_entry = std::max(step_entry, step);
      worst = std::max(worst, step / allowed);
      if (log_[t + 1].q_out_valid) {
        step_after = std::max(step_after, std::abs(log_[t + 1].q_out[u] - entry.q_out[u]));
      }
      EXPECT_LE(step, allowed + 1e-12)
          << row << ": joint " << j << " moved " << step << " rad on the tick the stop took over, "
          << last << " rad the tick before\n"
          << Window(static_cast<int>(t), 2);
      EXPECT_DOUBLE_EQ(entry.q_out[u], q[u])
          << row << ": joint " << j << " is not one ramp step from the command the tick began with";
      EXPECT_DOUBLE_EQ(entry.qd_cmd[u], qd[u]) << row << ": joint " << j << " (velocity)";
    }
    std::printf(
        "[ MEASURED ] #749 %s: max |dq| [rad] before %.3e, on the stop's first tick %.3e, "
        "after %.3e; step / (before + qdd*h^2) = %.3f\n",
        row, moved, step_entry, step_after, worst);
    ASSERT_GT(moved, kMovingStep) << row
                                  << ": the command was (nearly) at rest — the tick would "
                                     "pass whatever it did. The scenario must stop a "
                                     "moving arm.";
  }

  /// One tick of TRACK_ERR (see ATrackErrAbortReturnsAndReArms); the tick.
  std::size_t KickTrackErr() {
    state_.devices[0].positions[1] += kTrackErrKick;
    const std::size_t kick = log_.size();
    Ticks(1);
    return kick;
  }

  /// Tick `t` took the edge `from` → `to` for `reason`.
  void ExpectEdgeAt(std::size_t t, Mode from, Reason reason, Mode to = Mode::kAbortSafe) const {
    ASSERT_GT(t, 0U);
    ASSERT_LT(t, log_.size());
    ASSERT_EQ(log_[t - 1].mode, from) << Window(static_cast<int>(t), 3);
    ASSERT_EQ(log_[t].mode, to) << Window(static_cast<int>(t), 3);
    ASSERT_EQ(log_[t].reason, reason) << Window(static_cast<int>(t), 3);
  }

  /// A measured-position offset well past `track_err_abort`.
  static constexpr double kTrackErrKick = 0.6;

  /// A per-tick step above which a doubled step breaks the bound above:
  /// 2·Δ − q̈max·h² > Δ + q̈max·h² needs Δ > 2·q̈max·h² = 1.6e-5 rad on the
  /// shipped box. 0.02 rad/s.
  static constexpr double kMovingStep = 4e-5;

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
  /// Set by the *RealClock fixtures' constructors, before BringUp reads it.
  bool real_clock_{false};

  // Ball lane.
  bool publishing_{true};
  std::uint64_t seq_{1};
  std::uint64_t generation_{42};
  std::int64_t last_pub_ns_{0};
  /// How old the origin stamp is on receipt. Past the cloud's 0.35 s window
  /// every sample is behind the tick while the message is still fresh.
  std::int64_t stamp_age_ns_{5 * kMsNs};

  // Plant.
  bool servo_hand_{true};

  struct HandStall {
    int joint{0};
    double q{0.0};       // where the finger stops
    double effort{0.0};  // what it pushes with, toward q_close
  };

  std::optional<HandStall> hand_stall_;
  /// The hand device's joint_limits.max_torque (none by default, as shipped
  /// in the fixture): the capture witness's torque scale.
  std::optional<std::vector<double>> hand_max_torque_;
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
  NormalTrialCase();
}

/// Representative cases on the real steady clock (file header).
class SupervisorScenarioRealClockTest : public SupervisorScenarioTest {
 protected:
  SupervisorScenarioRealClockTest() { real_clock_ = true; }
};

TEST_F(SupervisorScenarioRealClockTest, NormalTrialIsCapturedAndReArms) {
  NormalTrialCase();
}

// ── MPC E1-F04 step 0: which reference instant the v1 command carries ───────

/// FNV-1a over every tick's mode and the arm / hand commands it wrote — the
/// digest a behaviour-preserving refactor must leave unchanged.
std::uint64_t CommandTraceDigest(const std::vector<TickRec>& log) {
  std::uint64_t h = 1469598103934665603ULL;
  const auto mix = [&h](const void* p, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 1099511628211ULL;
    }
  };
  for (const auto& r : log) {
    const auto mode = static_cast<std::uint8_t>(r.mode);
    mix(&mode, sizeof(mode));
    if (r.q_out_valid) {
      mix(r.q_out.data(), sizeof(r.q_out));
    }
    mix(r.qd_cmd.data(), sizeof(r.qd_cmd));
    if (r.hand_out_valid) {
      mix(r.hand_out.data(), sizeof(r.hand_out));
    }
  }
  return h;
}

/// CommandTraceDigest of NormalTrialCase(): the closed-form law, on this
/// fixture's profile, bit for bit. The value is the same in a Release and in a
/// Debug build (measured on GCC / x86-64 when the assertion was added) — the
/// repo sets no -march, -ffast-math or -ffp-contract, so the optimiser has no
/// licence to change a double.
///
/// It is a number to COMPARE AGAINST, not a claim that these commands are the
/// right ones. A change that is meant to alter the commands (the law tick, the
/// CLIK's numerics, the fixture's profile) replaces it in a commit of its own
/// that says why (PROC-6); a refactor that is not meant to must leave it alone.
constexpr std::uint64_t kNormalTrialCommandDigest = 0xf7cede9905b97416ULL;

TEST_F(SupervisorScenarioTest, TheNormalTrialCommandTraceIsUnchanged) {
  // The trace is deterministic on the fake clock — γ = 0 keeps the ball's wall
  // stamps out of the reference.
  ASSERT_NO_FATAL_FAILURE(NormalTrialCase());
  const std::uint64_t digest = CommandTraceDigest(log_);
  std::ostringstream hex;
  hex << std::hex << digest;
  RecordProperty("normal_trial_command_digest", hex.str());
  EXPECT_EQ(digest, kNormalTrialCommandDigest)
      << "the NormalTrial command trace changed: digest " << hex.str()
      << ". If the change is meant to alter the commands, replace the constant in its own "
         "commit with the reason; otherwise the refactor is not behaviour-preserving.";
}

TEST_F(SupervisorScenarioTest, TheV1CommandsTimeOffsetFromItsReferenceIsMeasured) {
  // MD-40, recorded rather than judged. The reference's output on tick n is
  // the reference for t_n + h; this measures which instant of it the command
  // leaving tick n sits at, along the reference velocity:
  //   label_n = h + (FK(q_n) − x(t_n + h))·ẋ / |ẋ|².
  // A first-order model of the CLIK (fixed point 2h) does not describe it:
  // the error starts at 0 when the reference is seeded at the arm, needs
  // 1/(hK_p) = 25 ticks to settle, and the posture row (w_arm · K_a) pulls the
  // task back by more than hẋ on this approach. The decel convention does not
  // rest on this number — while the RT follows a segment every target comes
  // from q_ref(s), so the command is q_ref(s + h) by construction, and that is
  // what the MPC follow scenarios assert. Under the shipped torque rows: the
  // fixture's derived box (2.03 rad/s²) binds on this approach and the arm
  // then lags by ẋ/K_p, which would measure the box.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["catching"]["joint_cmd"]["accel_constraint"] = "dynamic";
  }));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kHold, 1500)) << Transitions();

  constexpr double h = kDt;
  const int a = Entry(Mode::kApproach);
  const int d = Entry(Mode::kDecel);
  ASSERT_GT(a, 0);
  ASSERT_GT(d, a);
  std::array<double, 64> q{};
  std::vector<double> labels;
  double peak_speed = 0.0;
  double peak_label = 0.0;
  for (int n = a; n < d; ++n) {
    const auto& r = log_[static_cast<std::size_t>(n)];
    if (!(r.q_out_valid && r.body.ref_valid)) {
      continue;
    }
    for (int i = 0; i < kUr5eArmDof; ++i) {
      q[static_cast<std::size_t>(i)] = r.q_out[static_cast<std::size_t>(i)];
    }
    const Eigen::Vector3d fk = oracle_->PoseAt(arm_names_, q, kUr5eArmDof).translation();
    const Eigen::Vector3d x(r.body.ref_x[0], r.body.ref_x[1], r.body.ref_x[2]);  // x(t_n + h)
    const Eigen::Vector3d v(r.body.ref_xd[0], r.body.ref_xd[1], r.body.ref_xd[2]);
    const double speed = v.norm();
    if (speed < 0.02) {
      continue;
    }
    const double label = h + (fk - x).dot(v) / (speed * speed);
    labels.push_back(label);
    if (speed > peak_speed) {
      peak_speed = speed;
      peak_label = label;
    }
  }
  ASSERT_GT(labels.size(), 20U) << "too few moving ticks to measure on";
  std::sort(labels.begin(), labels.end());
  std::ostringstream os;
  os.precision(3);
  os << "at peak speed " << peak_label / h << " h (|x_dot| " << peak_speed << " m/s), range ["
     << labels.front() / h << ", " << labels.back() / h << "] h, median "
     << labels[labels.size() / 2] / h << " h over " << labels.size() << " ticks";
  RecordProperty("v1_command_time_label", os.str());
  std::printf("[ MEASURED ] v1 command time label: %s\n", os.str().c_str());
  EXPECT_TRUE(std::isfinite(peak_label)) << os.str();
}

// ════════════════════════════════════════════════════════════════════════════
// MPC E1-F09 — the arm follows the planner's segments from APPROACH to the end
// of the stop (mode mpc, MD-44 · MD-45)
// ════════════════════════════════════════════════════════════════════════════
//
// The oracle profile: no planner thread, so these cases are the decel box's
// one writer. In TRACKING they write the first segment of the plan the oracle
// stores on the same tick — the PAIR a planner publishes (MD-56) — built from
// the arm state the RT last reported, at rest (catching_decel_segment_
// fixture.hpp). Replans are the same trajectory from a later node, or one
// built from the command of the tick that takes it, which is what a planner
// with a perfect initial-state prediction would have published.

using DecelEvent = integrated_bringup::CatchingDiagLogPod::DecelEvent;
using integrated_bringup::testfx::kApproachDtNs;
using integrated_bringup::testfx::kApproachDtPreNs;
using integrated_bringup::testfx::kApproachNPre;
using integrated_bringup::testfx::kApproachNStop;
using rtc::catching::CatchingDecelMode;
using rtc::catching::DecelPlanSnapshot;
using rtc::catching::DecelRefusal;

class DecelMpcScenarioTest : public SupervisorScenarioTest {
 protected:
  static constexpr double kBump = 0.3;       // rad/s — the approach's velocity step
  static constexpr double kTcOffsetS = 0.6;  // the oracle's t_c − now

  /// mode mpc on the scenario profile: the catch sub-model the sampler binds
  /// to, a wide catch box (the search's key — the RT does not read it, MD-73),
  /// and the shipped torque rows (the fixture's derived box, 2.03 rad/s²,
  /// would cap the follow's step).
  static void MpcProfile(YAML::Node& y) {
    y["catching"]["supervisor"]["decel"]["mode"] = "mpc";
    y["catching"]["planner"]["sub_model"] = "ur5e_catch";
    y["catching"]["planner"]["workspace"]["catch_box"]["min"] =
        std::vector<double>{-2.0, -2.0, -2.0};
    y["catching"]["planner"]["workspace"]["catch_box"]["max"] = std::vector<double>{2.0, 2.0, 2.0};
    y["catching"]["joint_cmd"]["accel_constraint"] = "dynamic";
  }

  void BringUpMpc(const std::function<void(YAML::Node&)>& extra = nullptr) {
    ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, kTcOffsetS, [extra](YAML::Node& y) {
      MpcProfile(y);
      if (extra) {
        extra(y);
      }
    }));
    ASSERT_EQ(ctrl_->GetDecelMode(), CatchingDecelMode::kMpc);
    tips_enabled_ = true;
    ball_in_hand_ = true;
    ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  }

  /// The carried command, device order, in the 64-wide form the oracle takes.
  std::array<double, 64> Wide(const std::array<double, kUr5eArmDof>& a) const {
    std::array<double, 64> w{};
    std::copy(a.begin(), a.end(), w.begin());
    return w;
  }

  /// The planner's stamps on a segment written now: this activation, the
  /// plan's track, published this instant from the RT state of the tick
  /// before.
  void Stamp(DecelPlanSnapshot& seg, std::uint32_t seq) const {
    seg.token.activation_generation = ctrl_->GetPlannerRtState().activation_generation;
    seg.token.generation = generation_;
    seg.publish_ns = Now();
    seg.rt_state_ns = Now() - kHNs;
    seg.decel_seq = seq;
  }

  /// The first segment of plan (`plan_id`, `t_c`): from the arm the RT last
  /// reported — the measured pose before a command exists, the carried one
  /// after (PlannerRtState::q_cmd) — at rest.
  DecelPlanSnapshot FirstSegment(std::uint32_t plan_id, std::int64_t t_c_ns) const {
    const rtc::catching::PlannerRtState rt = ctrl_->GetPlannerRtState();
    std::array<double, kUr5eArmDof> q{};
    const std::array<double, kUr5eArmDof> rest{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      q[static_cast<std::size_t>(i)] = rt.q_cmd[static_cast<std::size_t>(i)];
    }
    return integrated_bringup::testfx::MakeApproachSegment(
        q, rest, kUr5eArmDof, plan_id, t_c_ns, kApproachNPre,
        t_c_ns - kApproachNPre * kApproachDtPreNs, kBump);
  }

  /// Play the planner's pair: on every TRACKING tick, before it runs, store
  /// the first segment of the plan the oracle is about to store on that tick
  /// (its next id, t_c one offset from the tick's clock read). `mutate` spoils
  /// it; `write` false leaves the box alone (a plan without its segment).
  void WritePair(const std::function<void(DecelPlanSnapshot&)>& mutate = nullptr,
                 bool write = true) {
    pre_tick_ = [this, mutate, write] {
      if (ctrl_->GetMode() != Mode::kTracking) {
        return;
      }
      const std::uint32_t plan_id = ctrl_->GetPublishedPlan().plan_id + 1;
      const std::int64_t t_c = Now() + static_cast<std::int64_t>(kTcOffsetS * 1e9);
      first_seg_ = FirstSegment(plan_id, t_c);
      Stamp(first_seg_, pair_seq_);
      if (mutate) {
        mutate(first_seg_);
      }
      if (write) {
        ctrl_->DecelBoxForTesting().Store(first_seg_);
      }
      pair_tick_ = static_cast<int>(log_.size());
    };
  }

  /// A pair the RT took: APPROACH, with `first_seg_` the segment it holds.
  void TakeThePair(const std::function<void(DecelPlanSnapshot&)>& mutate = nullptr) {
    WritePair(mutate);
    ASSERT_TRUE(TickUntilMode(Mode::kApproach, 1500)) << Transitions();
    ASSERT_EQ(static_cast<int>(log_.size()) - 1, pair_tick_)
        << "the pair was not taken on the tick it came";
    ASSERT_TRUE(ctrl_->HasPendingDecelForTesting()) << Transitions();
    pre_tick_ = nullptr;
  }

  /// … and followed: the first tick after the switch at node 0.
  void FollowThePair() {
    ASSERT_NO_FATAL_FAILURE(TakeThePair());
    ASSERT_TRUE(TickUntil([this] { return ctrl_->IsFollowingDecelForTesting(); }, 400))
        << Transitions();
  }

  /// Tick until the NEXT tick's sample instant (its now + h) reaches `t_ns` —
  /// i.e. stop with that tick still to run.
  bool TickToJustBefore(std::int64_t t_ns, int max_ticks = 1500) {
    for (int i = 0; i < max_ticks; ++i) {
      if (Now() + kHNs >= t_ns) {
        return true;
      }
      Tick();
    }
    return false;
  }

  /// The tick whose record carries `event`, from `from` on; -1 when none.
  int EventTick(DecelEvent event, std::size_t from = 0) const {
    for (std::size_t i = from; i < log_.size(); ++i) {
      if (log_[i].body.decel_event == event) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  int CountEvent(DecelEvent event, std::size_t from = 0) const {
    return CountTicks([event](const TickRec& t) { return t.body.decel_event == event; }, from);
  }

  /// A pair the RT does not take: it stays in TRACKING with "no plan", the
  /// plan itself admissible, and the lane's record says why the segment was
  /// not.
  void ExpectThePairRefused(DecelRefusal refusal, DecelEvent event = DecelEvent::kNone) {
    ASSERT_TRUE(TickUntilMode(Mode::kTracking, 1500)) << Transitions();
    Ticks(60);
    EXPECT_EQ(ctrl_->GetMode(), Mode::kTracking) << Transitions();
    EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 0U);
    EXPECT_EQ(CountTicks([](const TickRec& t) { return t.mode == Mode::kApproach; }), 0);
    const auto& r = log_.back();
    EXPECT_EQ(r.reason, Reason::kNoCatchablePlan);
    EXPECT_EQ(r.refusal, PlanRefusal::kNone) << "the plan was admissible; only its segment was not";
    EXPECT_TRUE(r.body.decel_judged) << "the lane did not judge the pair";
    EXPECT_EQ(static_cast<DecelRefusal>(r.body.decel_refusal), refusal)
        << "refusal " << static_cast<int>(r.body.decel_refusal);
    EXPECT_EQ(r.body.decel_event, event) << "event " << static_cast<int>(r.body.decel_event);
    EXPECT_FALSE(ctrl_->HasPendingDecelForTesting());
    EXPECT_FALSE(r.rt_decel_pending);
    EXPECT_FALSE(r.body.ref_valid) << "the soft-catch reference ran under mpc";
  }

  /// The tick the pair was last written before (the adoption tick, when it
  /// was taken), and the segment written.
  int pair_tick_{-1};
  std::uint32_t pair_seq_{1};
  DecelPlanSnapshot first_seg_{};
};

TEST_F(DecelMpcScenarioTest, TheRtTakesThePairAndFollowsItFromApproachToTheRearm) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  const DecelPlanSnapshot seg = first_seg_;
  // HOLD takes no new segment (MD-38): one written on its entry is not followed.
  ASSERT_TRUE(TickUntilMode(Mode::kHold, 1500)) << Transitions();
  DecelPlanSnapshot late = integrated_bringup::testfx::ShiftSegment(seg, kApproachNPre + 2);
  Stamp(late, 2);
  ctrl_->DecelBoxForTesting().Store(late);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 400)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();

  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured)
      << Transitions();
  ExpectDecelAtTc();

  // ── The adoption tick took the plan and its segment together ──
  const int a = Entry(Mode::kApproach);
  const auto& adopt = log_[static_cast<std::size_t>(a)];
  EXPECT_EQ(adopt.body.decel_event, DecelEvent::kAdmitted);
  EXPECT_EQ(static_cast<DecelRefusal>(adopt.body.decel_refusal), DecelRefusal::kNone);
  EXPECT_EQ(adopt.plan_id, seg.plan_id);
  EXPECT_EQ(adopt.plan_t_c_ns, seg.t_c_ns) << "the fixture mis-predicted the oracle's t_c";

  // ── Until node 0 the command is held and the segment is reported pending ──
  const int sw = EventTick(DecelEvent::kSwitched);
  ASSERT_GT(sw, a) << Transitions();
  ASSERT_GT(sw - a, 50) << "node 0 came too soon to see the wait";
  EXPECT_GE(log_[static_cast<std::size_t>(sw)].before_ns + kHNs, seg.t0_ns)
      << "switched before node 0";
  EXPECT_LT(log_[static_cast<std::size_t>(sw) - 1].before_ns + kHNs, seg.t0_ns)
      << "switched later than the first due tick";
  for (int i = a; i < sw; ++i) {
    const auto& r = log_[static_cast<std::size_t>(i)];
    ASSERT_EQ(r.mode, Mode::kApproach) << Window(i, 2);
    ASSERT_FALSE(r.body.decel_following) << Window(i, 2);
    ASSERT_TRUE(r.rt_decel_pending) << "the waiting segment is not reported\n" << Window(i, 2);
    ASSERT_EQ(r.rt_decel_pending_seq, 1U);
    ASSERT_FALSE(r.rt_decel_active);
    for (int j = 0; j < kUr5eArmDof; ++j) {
      const auto u = static_cast<std::size_t>(j);
      ASSERT_EQ(r.q_cmd[u], adopt.q_cmd[u]) << "the command moved before node 0, joint " << j;
      ASSERT_EQ(r.qd_cmd[u], 0.0) << "joint " << j;
    }
  }

  // ── G7-B′ (MD-39 (1)): node 0 is the command the arm holds, so the switch is
  // continuous to rounding ──
  const auto& entry = log_[static_cast<std::size_t>(sw)];
  const auto& before = log_[static_cast<std::size_t>(sw - 1)];
  EXPECT_TRUE(entry.body.decel_following);
  EXPECT_EQ(entry.body.decel_seq, 1U);
  EXPECT_LT(entry.body.decel_dq_max, 1e-9);
  EXPECT_LT(entry.body.decel_dqd_max, 1e-9);
  EXPECT_LT(entry.body.decel_rho, 1e-6);
  {
    const Eigen::Vector3d p_c =
        oracle_->PoseAt(arm_names_, Wide(before.q_cmd), kUr5eArmDof).translation();
    const Eigen::Vector3d p_d(entry.body.decel_p_d[0], entry.body.decel_p_d[1],
                              entry.body.decel_p_d[2]);
    EXPECT_LT((p_d - p_c).norm(), 1e-9) << "‖p_d − FK(q_c)‖";
  }

  // ── From the switch to the re-arm: one segment, reported as followed in
  // every mode that follows it; the soft-catch reference never runs ──
  const int d = Entry(Mode::kDecel);
  const int h = Entry(Mode::kHold);
  const int ret = Entry(Mode::kRetreat);
  ASSERT_GT(d, sw);
  ASSERT_GT(h, d);
  ASSERT_GT(ret, h);
  EXPECT_NE(log_[static_cast<std::size_t>(d)].body.decel_event, DecelEvent::kSwitched)
      << "the DECEL entry is the same segment going on, not a switch";
  for (int i = sw; i < ret; ++i) {
    const auto& r = log_[static_cast<std::size_t>(i)];
    ASSERT_TRUE(r.body.decel_following) << Window(i, 2);
    ASSERT_EQ(r.body.decel_seq, 1U) << "another segment was followed\n" << Window(i, 2);
    ASSERT_TRUE(r.rt_decel_active)
        << "the followed segment is not reported in " << ModeName(r.mode) << '\n'
        << Window(i, 2);
    ASSERT_EQ(r.rt_decel_seq, 1U);
    ASSERT_FALSE(r.rt_decel_pending) << Window(i, 2);
  }
  for (int i = static_cast<int>(Entry(Mode::kTracking)); i < ret; ++i) {
    ASSERT_FALSE(log_[static_cast<std::size_t>(i)].body.ref_valid)
        << "an mpc tick stepped the soft-catch reference\n"
        << Window(i, 2);
  }
  // The modes it was followed in.
  for (const Mode m :
       {Mode::kApproach, Mode::kCommitted, Mode::kClosing, Mode::kDecel, Mode::kHold}) {
    EXPECT_GT(CountTicks([m](const TickRec& t) { return t.mode == m && t.rt_decel_active; }), 0)
        << "no tick followed the segment in " << ModeName(m);
  }
  // The DECEL entry carries the reference on: one tick of travel, no step.
  {
    const auto& e = log_[static_cast<std::size_t>(d)];
    const auto& p = log_[static_cast<std::size_t>(d - 1)];
    const Eigen::Vector3d step(e.body.decel_p_d[0] - p.body.decel_p_d[0],
                               e.body.decel_p_d[1] - p.body.decel_p_d[1],
                               e.body.decel_p_d[2] - p.body.decel_p_d[2]);
    const Eigen::Vector3d v(p.body.decel_v_ff[0], p.body.decel_v_ff[1], p.body.decel_v_ff[2]);
    ASSERT_GT(v.norm(), 0.01) << "the arm is at rest at t_c: the entry would be continuous anyway";
    EXPECT_LT((step - kDt * v).norm(), 0.1 * kDt * v.norm())
        << "|Δp_d − h·v_ff| at the DECEL entry";
  }
  // After the stop: nothing held, nothing reported, the lane silent.
  for (std::size_t i = static_cast<std::size_t>(sw) + 1; i < log_.size(); ++i) {
    const Mode start = log_[i - 1].mode;  // the mode the tick STARTED in
    if (start == Mode::kHold || start == Mode::kRetreat) {
      ASSERT_FALSE(log_[i].body.decel_judged) << "the lane ran in " << ModeName(start) << '\n'
                                              << Window(static_cast<int>(i), 2);
    }
    if (log_[i].mode == Mode::kRetreat || log_[i].mode == Mode::kArmed) {
      ASSERT_FALSE(log_[i].rt_decel_active) << Window(static_cast<int>(i), 2);
      ASSERT_FALSE(log_[i].rt_decel_pending) << Window(static_cast<int>(i), 2);
    }
  }

  // ── MD-40: following, the command leaving tick n is the segment at
  // now_lead + 2h (sampled at now_lead + h). Judged on the catch frame, where
  // the CLIK tracks with K_p and the twist feedforward; measured against the
  // neighbours h and 3h so the offset is resolved, not just bounded. The
  // joint-space gap is recorded only: the redundant direction is held by the
  // posture row alone (w_arm, K_n = 1), which the smoothing term drags ──
  double err_p[3] = {0.0, 0.0, 0.0};
  double err_q[3] = {0.0, 0.0, 0.0};
  double speed_peak = 0.0;
  int n_follow = 0;
  for (std::size_t i = static_cast<std::size_t>(sw) + 1; i < log_.size(); ++i) {
    const auto& r = log_[i];
    if (!r.body.decel_following || r.body.decel_held) {
      continue;
    }
    ++n_follow;
    const Eigen::Vector3d p_out =
        oracle_->PoseAt(arm_names_, Wide(r.q_out), kUr5eArmDof).translation();
    for (int k = 0; k < 3; ++k) {
      std::array<double, rtc::catching::kMaxDecelNv> q{};
      std::array<double, rtc::catching::kMaxDecelNv> qd{};
      std::array<double, rtc::catching::kMaxDecelNv> qdd{};
      ASSERT_TRUE(rtc::catching::NodeTrajectoryFollower::SampleJoints(
          seg, r.before_ns + (k + 1) * kHNs, q, qd, qdd));
      std::array<double, 64> qw{};
      std::array<double, 64> qdw{};
      for (int j = 0; j < kUr5eArmDof; ++j) {
        const auto u = static_cast<std::size_t>(j);
        err_q[k] = std::max(err_q[k], std::abs(r.q_out[u] - q[u]));
        qw[u] = q[u];
        qdw[u] = qd[u];
      }
      err_p[k] = std::max(
          err_p[k], (p_out - oracle_->PoseAt(arm_names_, qw, kUr5eArmDof).translation()).norm());
      if (k == 1) {
        speed_peak = std::max(speed_peak,
                              oracle_->LinearVelocityAt(arm_names_, qw, qdw, kUr5eArmDof).norm());
      }
    }
  }
  std::ostringstream os;
  os.precision(3);
  os << "over " << n_follow << " ticks, max |FK(q_out) - FK(q_ref(t))| [m]: t = now+h " << err_p[0]
     << ", now+2h " << err_p[1] << ", now+3h " << err_p[2] << " (h*|p_dot|max " << kDt * speed_peak
     << "); joint max [rad]: " << err_q[0] << " / " << err_q[1] << " / " << err_q[2];
  RecordProperty("decel_follow_time_label", os.str());
  std::printf("[ MEASURED ] %s\n", os.str().c_str());
  ASSERT_GT(n_follow, 100);
  // 2h is the label: at least twice as close as either neighbour. What is left
  // is the feedback loop's lag through the step's acceleration phases, not a
  // time offset — it is the same on both sides of 2h, which an offset would
  // not be.
  EXPECT_LT(err_p[1], 0.5 * std::min(err_p[0], err_p[2])) << os.str();
  EXPECT_LT(err_p[1], 0.4 * kDt * speed_peak) << os.str();
}

// ── The pair: a plan is taken with its first segment, or not at all ─────────

TEST_F(DecelMpcScenarioTest, APlanWithoutItsSegmentIsNotTaken) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair(nullptr, /*write=*/false);
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kInvalid));
}

TEST_F(DecelMpcScenarioTest, APlanWithAnAgedSegmentIsNotTaken) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair([this](DecelPlanSnapshot& s) { s.publish_ns = Now() - 60 * kMsNs; });
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kAged));
}

TEST_F(DecelMpcScenarioTest, APlanWithAnotherPlansSegmentIsNotTaken) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair([](DecelPlanSnapshot& s) { s.plan_id += 1; });
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kPlan));
}

TEST_F(DecelMpcScenarioTest, APlanWithASegmentForAnotherTrackIsNotTaken) {
  // The segment carries the PLAN's track; one solved for another ball is not
  // this plan's, whatever its id says.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair([](DecelPlanSnapshot& s) { s.token.generation += 1; });
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kPlan));
}

TEST_F(DecelMpcScenarioTest, APlanWithAMalformedSegmentIsNotTaken) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair([](DecelPlanSnapshot& s) {
    s.q[static_cast<std::size_t>(3 * rtc::catching::kMaxDecelNv + 2)] =
        std::numeric_limits<double>::quiet_NaN();
  });
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kMalformed));
}

TEST_F(DecelMpcScenarioTest, APlanWithASegmentOfAnotherJointCountIsNotTaken) {
  // Well-formed in itself — every used node entry finite, the last at rest —
  // and not this arm's: the sampler is bound to the arm's joints and could
  // never evaluate it. Taken with its plan, the trial would enter APPROACH
  // and abort at node 0; refused here, the pair is simply not there.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair([](DecelPlanSnapshot& s) { s.nv -= 1; });
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kMalformed));
}

TEST_F(DecelMpcScenarioTest, APlanWithASegmentPredictedBeforeTheResetIsNotTaken) {
  // MD-37: published after the floor, predicted from an RT state before it.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  WritePair([](DecelPlanSnapshot& s) { s.rt_state_ns = 1; });
  ASSERT_NO_FATAL_FAILURE(ExpectThePairRefused(DecelRefusal::kBeforeReset));
}

// ── The RT does not judge where the stop ends (MD-73) ───────────────────────
// catch_box is the planner search's: it judges the catch point and the stop
// point of the plans it publishes. The RT takes a segment on JudgeDecelPlan
// and the switch gate alone. Under a catch box whose ceiling is below the hand
// no node of any segment is inside it — the pair is taken and a replan is
// followed all the same. (What these assert is the adoption itself: no event
// code is left to count — a check brought back under any name fails them by
// refusing the pair or the replan.)

class DecelMpcNoCatchBoxCheckTest : public DecelMpcScenarioTest {
 protected:
  void BringUpUnderABoxThatHoldsNoNode() {
    ASSERT_NO_FATAL_FAILURE(BringUpMpc([this](YAML::Node& y) {
      const double z = start_pose_.translation().z() - 0.2;
      y["catching"]["planner"]["workspace"]["catch_box"]["max"] = std::vector<double>{2.0, 2.0, z};
    }));
  }
};

TEST_F(DecelMpcNoCatchBoxCheckTest, APairWhoseStopLeavesTheCatchBoxIsTaken) {
  ASSERT_NO_FATAL_FAILURE(BringUpUnderABoxThatHoldsNoNode());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

TEST_F(DecelMpcNoCatchBoxCheckTest, AReplanWhoseStopLeavesTheCatchBoxIsFollowed) {
  ASSERT_NO_FATAL_FAILURE(BringUpUnderABoxThatHoldsNoNode());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kDecel, 1500)) << Transitions();
  // The same trajectory from one stop node on: a post-catch replan.
  DecelPlanSnapshot replan =
      integrated_bringup::testfx::ShiftSegment(first_seg_, kApproachNPre + 1);
  Stamp(replan, 2);
  ctrl_->DecelBoxForTesting().Store(replan);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 600)) << Transitions();
  EXPECT_GT(
      CountTicks([](const TickRec& t) { return t.body.decel_following && t.body.decel_seq == 2U; }),
      0)
      << "the replan was not followed\n"
      << Transitions();
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

// ── Nothing to follow is ABORT_SAFE, from APPROACH on (MD-44) ───────────────

TEST_F(DecelMpcScenarioTest, AFirstSegmentPastTheSwitchGateAbortsInApproach) {
  // MD-39 (2): 0.05 rad off on one joint — K_p·Δq = 1 rad/s against a
  // headroom of (1 − 0.9)·2 = 0.2 rad/s. The pair is admissible (admission
  // does not compare node 0 with the command); the gate at node 0 refuses it,
  // and with no segment behind it the trial aborts where it stands.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair([](DecelPlanSnapshot& s) {
    for (int k = 0; k <= s.n_nodes; ++k) {
      s.q[static_cast<std::size_t>(k * rtc::catching::kMaxDecelNv)] += 0.05;
    }
  }));
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 2500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed});
  const int ab = Entry(Mode::kAbortSafe);
  const auto& r = log_[static_cast<std::size_t>(ab)];
  EXPECT_EQ(r.reason, Reason::kParamsTbd) << Window(ab, 2);
  EXPECT_EQ(r.body.decel_event, DecelEvent::kGateRefused);
  EXPECT_EQ(r.body.decel_gate_joint, 0);
  EXPECT_NEAR(r.body.decel_rho, 5.0, 1e-6);
  EXPECT_FALSE(r.body.decel_following);
  EXPECT_FALSE(r.body.ref_valid) << "the soft-catch reference ran on an mpc abort";
  EXPECT_FALSE(r.rt_decel_pending) << "the refused segment is still reported";
  EXPECT_FALSE(r.rt_decel_active);
  // The abort is the tick node 0 came due, not a later one.
  EXPECT_GE(r.before_ns + kHNs, first_seg_.t0_ns);
  EXPECT_LT(log_[static_cast<std::size_t>(ab) - 1].before_ns + kHNs, first_seg_.t0_ns);
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.body.decel_following; }), 0);
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kAborted);
}

TEST_F(DecelMpcScenarioTest, AClikFailureWhileFollowingIsAnAbort) {
  // A hand reading the solve cannot use (NaN) fails the CLIK on the next
  // mpc tick: the same kQpFailed → ABORT_SAFE as the soft-catch law's.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  Ticks(5);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();
  ASSERT_TRUE(ctrl_->IsFollowingDecelForTesting());
  servo_hand_ = false;
  state_.devices[1].positions[0] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 20)) << Transitions();
  const int a = Entry(Mode::kAbortSafe);
  EXPECT_TRUE(log_[static_cast<std::size_t>(a)].reason == Reason::kQpFailed ||
              log_[static_cast<std::size_t>(a)].reason == Reason::kJointConflict)
      << Window(a, 2);
  EXPECT_FALSE(ctrl_->IsFollowingDecelForTesting()) << "ABORT_SAFE entry drops the segment";
  EXPECT_FALSE(log_[static_cast<std::size_t>(a)].rt_decel_active);
}

TEST_F(DecelMpcScenarioTest, ABallThatGoesStaleEndsTheApproachAsBefore) {
  // The ball lane's reasons are the supervisor's whichever law runs: a
  // prediction that stops arriving ends the approach by BALL_STALE → RETREAT,
  // and the segments go with the plan.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  publishing_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 400)) << Transitions();
  const int r = Entry(Mode::kRetreat);
  EXPECT_EQ(log_[static_cast<std::size_t>(r)].reason, Reason::kBallStale) << Window(r, 2);
  EXPECT_EQ(log_[static_cast<std::size_t>(r) - 1].mode, Mode::kApproach) << Window(r, 2);
  EXPECT_FALSE(ctrl_->IsFollowingDecelForTesting());
  EXPECT_FALSE(ctrl_->HasPendingDecelForTesting());
  EXPECT_FALSE(log_[static_cast<std::size_t>(r)].rt_decel_active);
  EXPECT_FALSE(log_[static_cast<std::size_t>(r)].rt_decel_pending);
}

TEST_F(DecelMpcScenarioTest, AnotherPlanInApproachIsNotTaken) {
  // MD-57: under mpc the followed plan is not replaced — its segments belong
  // to it. A plan that reaches the box all the same (admissible, outside the
  // freeze window) is left there and the trial runs on the one it took.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  const PlanSnapshot followed = ctrl_->GetFollowedPlanForTesting();
  PlanSnapshot other = followed;
  other.plan_id = followed.plan_id + 1;
  other.p_c[0] += 0.02;
  other.publish_ns = Now();
  ctrl_->PlanBoxForTesting().Store(other);
  Ticks(5);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();
  EXPECT_EQ(log_.back().refusal, PlanRefusal::kNone) << "the other plan was not even admissible";
  EXPECT_EQ(ctrl_->GetFollowedPlanForTesting().plan_id, followed.plan_id);
  EXPECT_EQ(ctrl_->GetPlanReplacedCount(), 0U);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat});
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

// ── The pending slot (MD-37, MD-58) ─────────────────────────────────────────

TEST_F(DecelMpcScenarioTest, ANewerSegmentForTheSameNodeZeroReplacesTheWaitingOne) {
  // The same grid point solved again on a newer prediction: here, the same
  // start with a smaller velocity step.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  Ticks(10);
  ASSERT_FALSE(ctrl_->IsFollowingDecelForTesting()) << "node 0 came before the replacement";
  std::array<double, kUr5eArmDof> q0{};
  const std::array<double, kUr5eArmDof> rest{};
  for (int i = 0; i < kUr5eArmDof; ++i) {
    q0[static_cast<std::size_t>(i)] = first_seg_.q[static_cast<std::size_t>(i)];
  }
  DecelPlanSnapshot again = integrated_bringup::testfx::MakeApproachSegment(
      q0, rest, kUr5eArmDof, first_seg_.plan_id, first_seg_.t_c_ns, kApproachNPre, first_seg_.t0_ns,
      0.5 * kBump);
  ASSERT_EQ(again.t0_ns, first_seg_.t0_ns);
  Stamp(again, 2);
  ctrl_->DecelBoxForTesting().Store(again);
  const std::size_t at = log_.size();
  Tick();
  EXPECT_EQ(log_[at].body.decel_event, DecelEvent::kReplaced) << Window(static_cast<int>(at), 2);
  EXPECT_TRUE(log_[at].rt_decel_pending);
  EXPECT_EQ(log_[at].rt_decel_pending_seq, 2U);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(
      CountTicks([](const TickRec& t) { return t.body.decel_following && t.body.decel_seq != 2U; }),
      0)
      << "the replaced segment was followed";
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.body.decel_following; }), 100);
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

TEST_F(DecelMpcScenarioTest, ASegmentForALaterNodeZeroWaitsInTheBoxUntilTheSlotIsFree) {
  // MD-37: the next grid point's segment arrives while the slot still holds
  // the one before it. It is left in the box — taking it would leave the
  // instant in between nothing to follow — and admitted once the slot clears.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  ASSERT_TRUE(TickToJustBefore(first_seg_.t0_ns - 20 * kMsNs)) << Transitions();
  DecelPlanSnapshot next = integrated_bringup::testfx::ShiftSegment(first_seg_, 1);
  Stamp(next, 2);
  ctrl_->DecelBoxForTesting().Store(next);
  const std::size_t from = log_.size();
  ASSERT_TRUE(TickUntil(
      [this] {
        return ctrl_->IsFollowingDecelForTesting() &&
               ctrl_->GetFollowedDecelForTesting().decel_seq == 2U;
      },
      400))
      << Transitions();
  const int sw1 = EventTick(DecelEvent::kSwitched, from);
  const int adm = EventTick(DecelEvent::kAdmitted, from);
  ASSERT_GT(sw1, static_cast<int>(from)) << Transitions();
  EXPECT_EQ(adm, sw1 + 1) << "admitted on the first tick the slot was free";
  for (int i = static_cast<int>(from); i < sw1; ++i) {
    const auto& r = log_[static_cast<std::size_t>(i)];
    ASSERT_EQ(r.body.decel_event, DecelEvent::kDeferred) << Window(i, 2);
    ASSERT_EQ(r.rt_decel_pending_seq, 1U) << "the waiting segment was overwritten";
  }
  EXPECT_EQ(log_[static_cast<std::size_t>(sw1)].body.decel_seq, 1U);
  EXPECT_EQ(log_[static_cast<std::size_t>(adm)].rt_decel_pending_seq, 2U);
  EXPECT_EQ(log_[static_cast<std::size_t>(adm)].rt_decel_seq, 1U);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

TEST_F(DecelMpcScenarioTest, ASegmentThatWaitsPastTheAgeBoundIsNeverTaken) {
  // The age bound is read when the segment is admitted, and a deferred one is
  // judged again every tick: left waiting for longer than the bound, it is
  // refused as aged and the followed segment goes on.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  ASSERT_TRUE(TickToJustBefore(first_seg_.t0_ns - 80 * kMsNs)) << Transitions();
  DecelPlanSnapshot next = integrated_bringup::testfx::ShiftSegment(first_seg_, 1);
  Stamp(next, 2);
  ctrl_->DecelBoxForTesting().Store(next);
  const std::size_t from = log_.size();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_GT(CountEvent(DecelEvent::kDeferred, from), 10);
  EXPECT_GT(CountTicks(
                [](const TickRec& t) {
                  return t.body.decel_judged &&
                         static_cast<DecelRefusal>(t.body.decel_refusal) == DecelRefusal::kAged;
                },
                from),
            0);
  EXPECT_EQ(
      CountTicks([](const TickRec& t) { return t.body.decel_following && t.body.decel_seq != 1U; }),
      0)
      << "the aged segment was followed";
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

// ── Replans (MD-38, MD-39) ──────────────────────────────────────────────────

TEST_F(DecelMpcScenarioTest, AReplanBuiltFromTheMovingCommandSwitchesWithoutAStep) {
  // G7-B′ on a moving arm (MD-39 (1)): one pre-catch interval before t_c the
  // arm is cruising on the first segment. A replan whose node 0 is made from
  // the command the switch tick starts from is continuous to rounding — in
  // the joints and in the hand's pose and twist.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  const std::int64_t t0 = first_seg_.t_c_ns - kApproachDtPreNs;
  ASSERT_TRUE(TickToJustBefore(t0)) << Transitions();
  ASSERT_TRUE(ctrl_->IsFollowingDecelForTesting());
  std::array<double, kUr5eArmDof> q{};
  std::array<double, kUr5eArmDof> qd{};
  const auto& qc = ctrl_->GetArmCommandForTesting();
  const auto& qdc = ctrl_->GetArmVelocityCommandForTesting();
  double speed = 0.0;
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    q[u] = qc[u];
    qd[u] = qdc[u];
    speed = std::max(speed, std::abs(qd[u]));
  }
  ASSERT_GT(speed, 0.5 * kBump) << "the arm is not moving: the switch would be continuous anyway";
  DecelPlanSnapshot replan = integrated_bringup::testfx::MakeApproachSegment(
      q, qd, kUr5eArmDof, first_seg_.plan_id, first_seg_.t_c_ns, 1, Now() + kHNs, kBump);
  ASSERT_EQ(replan.t0_ns, t0);
  Stamp(replan, 2);
  ctrl_->DecelBoxForTesting().Store(replan);
  const std::size_t at = log_.size();
  Tick();
  const auto& sw = log_[at];
  const auto& before = log_[at - 1];
  ASSERT_EQ(sw.body.decel_event, DecelEvent::kSwitched) << Window(static_cast<int>(at), 2);
  EXPECT_EQ(sw.body.decel_seq, 2U);
  EXPECT_EQ(sw.mode, Mode::kClosing) << "the switch was meant to fall before t_c";
  const Eigen::Vector3d p_c =
      oracle_->PoseAt(arm_names_, Wide(before.q_cmd), kUr5eArmDof).translation();
  const Eigen::Vector3d v_c =
      oracle_->LinearVelocityAt(arm_names_, Wide(before.q_cmd), Wide(before.qd_cmd), kUr5eArmDof);
  const Eigen::Vector3d p_d(sw.body.decel_p_d[0], sw.body.decel_p_d[1], sw.body.decel_p_d[2]);
  const Eigen::Vector3d v_ff(sw.body.decel_v_ff[0], sw.body.decel_v_ff[1], sw.body.decel_v_ff[2]);
  EXPECT_LT((p_d - p_c).norm(), 1e-9) << "‖p_d − FK(q_c)‖";
  EXPECT_LT((v_ff - v_c).norm(), 1e-9) << "‖V_ff − J q̇_c‖";
  EXPECT_LT(sw.body.decel_dq_max, 1e-9);
  EXPECT_LT(sw.body.decel_dqd_max, 1e-9);
  EXPECT_LT(sw.body.decel_rho, 1e-6);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

TEST_F(DecelMpcScenarioTest, AReplanOnTheSameTrajectoryIsTakenAtItsNodeZero) {
  // Before the catch (an APPROACH–stop segment from a later node) and after it
  // (a stop-only segment at grid point 1): each waits for its node 0 and is
  // taken on the first tick that is due, the gate barely used.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  DecelPlanSnapshot pre = integrated_bringup::testfx::ShiftSegment(first_seg_, kApproachNPre - 1);
  ASSERT_EQ(pre.n_pre, 1);
  Stamp(pre, 2);
  ctrl_->DecelBoxForTesting().Store(pre);
  ASSERT_TRUE(TickUntilMode(Mode::kDecel, 1500)) << Transitions();
  DecelPlanSnapshot post = integrated_bringup::testfx::ShiftSegment(first_seg_, kApproachNPre + 1);
  ASSERT_EQ(post.n_pre, 0);
  ASSERT_EQ(post.k0, 1);
  Stamp(post, 3);
  ctrl_->DecelBoxForTesting().Store(post);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 600)) << Transitions();
  for (const DecelPlanSnapshot* s : {&pre, &post}) {
    int sw = -1;
    for (std::size_t i = 0; i < log_.size(); ++i) {
      if (log_[i].body.decel_event == DecelEvent::kSwitched &&
          log_[i].body.decel_seq == s->decel_seq) {
        sw = static_cast<int>(i);
        break;
      }
    }
    ASSERT_GT(sw, 0) << "segment " << s->decel_seq << " was not taken\n" << Transitions();
    const auto& r = log_[static_cast<std::size_t>(sw)];
    EXPECT_EQ(r.body.decel_k0, s->k0);
    EXPECT_GE(r.before_ns + kHNs, s->t0_ns) << "switched before its node 0";
    EXPECT_LT(log_[static_cast<std::size_t>(sw) - 1].before_ns + kHNs, s->t0_ns)
        << "switched later than the first due tick";
    EXPECT_LT(r.body.decel_rho, 0.2) << "the same trajectory, followed: the gate is barely used";
    ASSERT_GT(r.body.decel_dqd_max + r.body.decel_dq_max, 0.0);
  }
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

TEST_F(DecelMpcScenarioTest, AReplanPastTheGateIsDroppedAndTheFollowedSegmentGoesOn) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  DecelPlanSnapshot replan =
      integrated_bringup::testfx::ShiftSegment(first_seg_, kApproachNPre - 1);
  for (int k = 0; k <= replan.n_nodes; ++k) {
    replan.q[static_cast<std::size_t>(k * rtc::catching::kMaxDecelNv)] += 0.05;
  }
  Stamp(replan, 2);
  ctrl_->DecelBoxForTesting().Store(replan);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  EXPECT_EQ(CountEvent(DecelEvent::kGateRefused), 1) << Transitions();
  const int g = EventTick(DecelEvent::kGateRefused);
  ASSERT_GT(g, 0);
  EXPECT_FALSE(log_[static_cast<std::size_t>(g)].rt_decel_pending)
      << "the refused replan is still reported";
  EXPECT_TRUE(log_[static_cast<std::size_t>(g)].rt_decel_active);
  EXPECT_EQ(
      CountTicks([](const TickRec& t) { return t.body.decel_following && t.body.decel_seq != 1U; }),
      0)
      << "the refused replan was followed";
  EXPECT_EQ(log_[static_cast<std::size_t>(Entry(Mode::kRetreat))].outcome, Outcome::kCaptured);
}

// ── E-STOP and reset (MD-35, E-8) ───────────────────────────────────────────

class DecelMpcEstopTest : public DecelMpcScenarioTest {
 protected:
  /// The E-STOP on the next tick, with whatever the RT holds: everything it
  /// holds is gone, the report with it; then, re-armed on a new ball with the
  /// stopped trial's segment still in the box, no trial takes it.
  void StopAndExpectNothingCarriesOver() {
    const std::size_t trig = log_.size();
    ctrl_->TriggerEstop();
    Tick();
    EXPECT_EQ(log_[trig].mode, Mode::kIdle) << Window(static_cast<int>(trig), 2);
    EXPECT_FALSE(ctrl_->IsFollowingDecelForTesting()) << "E-8: the stop kept the followed segment";
    EXPECT_FALSE(ctrl_->HasPendingDecelForTesting()) << "E-8: the stop kept the waiting segment";
    EXPECT_FALSE(log_[trig].rt_decel_active);
    EXPECT_FALSE(log_[trig].rt_decel_pending);
    EXPECT_FALSE(log_[trig].body.decel_following);
    Ticks(10);
    ctrl_->ClearEstop();
    Ticks(5);
    ++generation_;
    const std::size_t second = log_.size();
    SetArmed(true);
    // The oracle offers a new plan every TRACKING tick; the box still holds
    // the stopped trial's segment (another plan, and past every floor).
    ASSERT_TRUE(TickUntilMode(Mode::kTracking, 2500)) << Transitions();
    Ticks(60);
    EXPECT_EQ(ctrl_->GetMode(), Mode::kTracking) << Transitions();
    EXPECT_EQ(CountTicks([](const TickRec& t) { return t.mode == Mode::kApproach; }, second), 0)
        << "the stopped trial's segment let the next plan in";
    EXPECT_EQ(CountTicks([](const TickRec& t) { return t.body.decel_following; }, second), 0);
    EXPECT_TRUE(log_.back().body.decel_judged);
    EXPECT_NE(static_cast<DecelRefusal>(log_.back().body.decel_refusal), DecelRefusal::kNone);
    // … and a pair of its own starts the next trial as usual.
    pair_seq_ = 7;
    ASSERT_NO_FATAL_FAILURE(TakeThePair());
    ASSERT_TRUE(TickUntil([this] { return ctrl_->IsFollowingDecelForTesting(); }, 400))
        << Transitions();
    EXPECT_EQ(ctrl_->GetFollowedDecelForTesting().decel_seq, 7U);
  }
};

TEST_F(DecelMpcEstopTest, AnEstopWhileTheFirstSegmentWaitsDropsIt) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(TakeThePair());
  Ticks(10);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach);
  ASSERT_TRUE(ctrl_->HasPendingDecelForTesting());
  ASSERT_FALSE(ctrl_->IsFollowingDecelForTesting());
  ASSERT_NO_FATAL_FAILURE(StopAndExpectNothingCarriesOver());
}

TEST_F(DecelMpcEstopTest, AnEstopWhileFollowingInApproachDropsTheSegment) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  Ticks(10);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach);
  ASSERT_TRUE(ctrl_->IsFollowingDecelForTesting());
  ASSERT_NO_FATAL_FAILURE(StopAndExpectNothingCarriesOver());
}

TEST_F(DecelMpcEstopTest, AnEstopInDecelDropsTheFollowedAndTheWaitingSegment) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kDecel, 1500)) << Transitions();
  // A replan for a later grid point is waiting when the stop lands.
  DecelPlanSnapshot post = integrated_bringup::testfx::ShiftSegment(first_seg_, kApproachNPre + 2);
  Stamp(post, 2);
  ctrl_->DecelBoxForTesting().Store(post);
  Ticks(3);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kDecel);
  ASSERT_TRUE(ctrl_->IsFollowingDecelForTesting());
  ASSERT_TRUE(ctrl_->HasPendingDecelForTesting()) << Transitions();
  ASSERT_NO_FATAL_FAILURE(StopAndExpectNothingCarriesOver());
}

TEST_F(DecelMpcEstopTest, AnEstopPairBetweenTwoTicksDropsTheSegment) {
  // Trigger and clear both land before the next tick: the epoch moved, so the
  // tick resets the trial — the segment with it — though no tick saw the stop.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  Ticks(10);
  ASSERT_TRUE(ctrl_->IsFollowingDecelForTesting());
  ctrl_->TriggerEstop();
  ctrl_->ClearEstop();
  const std::size_t pair = log_.size();
  Tick();
  EXPECT_FALSE(ctrl_->IsFollowingDecelForTesting()) << Window(static_cast<int>(pair), 2);
  EXPECT_FALSE(log_[pair].body.decel_following);
  EXPECT_FALSE(log_[pair].rt_decel_active);
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() == Mode::kIdle; }, 1500)) << Transitions();
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.body.decel_following; }, pair), 0);
}

TEST_F(DecelMpcScenarioTest, ClosedFormIsTheDefaultAndNeverReadsTheBox) {
  // "기본값 동등" (a): the key absent and closed_form written out drive the
  // NormalTrial to the same command digest, with an admissible pair in the box
  // on the tick the plan is taken — which closed_form never loads.
  ASSERT_NO_FATAL_FAILURE(NormalTrialCase());
  const std::uint64_t absent = CommandTraceDigest(log_);
  EXPECT_EQ(ctrl_->GetDecelMode(), CatchingDecelMode::kClosedForm);

  TearDown();
  log_.clear();
  seq_ = 1;
  last_pub_ns_ = 0;
  pair_tick_ = -1;
  SetUp();
  WritePair();
  ASSERT_NO_FATAL_FAILURE(NormalTrialCase(
      [](YAML::Node& y) { y["catching"]["supervisor"]["decel"]["mode"] = "closed_form"; }));
  ASSERT_GE(pair_tick_, 0) << "the box was never written";
  EXPECT_EQ(CommandTraceDigest(log_), absent) << "closed_form written out changed the trial";
  EXPECT_EQ(CountTicks([](const TickRec& t) {
              return t.body.decel_judged || t.body.decel_following ||
                     t.body.decel_event != DecelEvent::kNone || t.rt_decel_active ||
                     t.rt_decel_pending;
            }),
            0)
      << "closed_form touched the decel lane";
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

TEST_F(SupervisorScenarioTest, ASwitchedInPoseIsTheWaitPoseAndNeedsNoHoming) {
  // S8-I (`planner.wait_pose_source: current`): activated OFF the YAML pose,
  // the arm's own pose is the wait pose — IDLE → ARMED without one homing tick,
  // the carried command stays where the arm was, and the diag says adopted.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;
  off[2] -= 0.05;
  off[4] += 0.04;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off));
  publishing_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 800)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed});
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.homing; }), 0)
      << "homing to the pose the arm is already at";
  EXPECT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_TRUE(ctrl_->GetLastTickRecord().wait_pose_adopted);
  // The Q13 skip holds the arm where it is (the hold latch, not the homing
  // law — the carried command is seeded later, by the first plan), so the
  // OUTPUT never leaves the switched-in pose.
  const ControllerOutput held = ctrl_->Compute(state_);
  ASSERT_EQ(held.devices[0].num_channels, kUr5eArmDof);
  const auto wp = ctrl_->GetWaitPoseForTesting();
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_NEAR(held.devices[0].commands[u], off[u], 1e-9)
        << "joint " << i << ": the output left the switched-in pose";
    EXPECT_NEAR(wp[u], off[u], 1e-12) << "joint " << i;
  }
  EXPECT_EQ(ctrl_->GetHandOutputForTesting().phase, HandPhase::kPreshape);
}

TEST_F(SupervisorScenarioTest, TheAdoptedWaitPoseSurvivesAnEstopAndIsRetakenOnActivation) {
  // S8-I, the three edges of "once per activation": an E-STOP keeps the
  // adopted pose (where the arm stopped is not a wait pose), a new activation
  // adopts wherever the arm is then, and a pose outside the margined joint box
  // is refused in favour of the YAML pose.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;
  off[2] -= 0.05;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off));
  publishing_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 800)) << Transitions();
  ASSERT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting());
  const std::uint32_t seq1 = ctrl_->GetLastTickRecord().wait_pose_adopt_seq;

  // E-STOP with the arm moved: still the first pose.
  std::array<double, kUr5eArmDof> moved = off;
  moved[1] += 0.1;
  ctrl_->TriggerEstop();
  ctrl_->ClearEstop();
  state_ = MakeState(moved);
  Ticks(3);
  EXPECT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_adopt_seq, seq1);
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                off[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i << ": an E-STOP re-adopted";
  }

  // A new activation adopts where the arm is now.
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  Ticks(2);
  ASSERT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_adopt_seq, seq1 + 1);
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                moved[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i << ": re-activation kept the old pose";
  }

  // Outside the margined joint box: refused, the YAML pose stands.
  std::array<double, kUr5eArmDof> outside = off;
  outside[2] = 100.0;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  state_ = MakeState(outside);
  Ticks(2);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_reason,
            integrated_bringup::CatchingDiagLogPod::WaitPoseRefusal::kOutsideBox);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_joint, 2);
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                kUr5eHome[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i << ": a refused pose must leave the YAML pose in force";
  }
}

TEST_F(SupervisorScenarioTest, ATrialFromASwitchedInPoseReturnsToIt) {
  // S8-I: RETREAT homes to the ADOPTED pose, not the YAML's — a full cycle from
  // an off-YAML switch-in re-arms where it started.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;
  off[2] -= 0.05;
  off[4] += 0.04;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  const int armed = Entry(Mode::kArmed, 1);
  ASSERT_GT(armed, 0);
  const auto& back = log_[static_cast<std::size_t>(armed)];
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_NEAR(back.q_cmd[u], off[u], 1e-12) << "joint " << i
                                              << ": RETREAT did not return to "
                                                 "the adopted wait pose";
  }
  EXPECT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting()) << "a re-arm is not a new activation";
}

TEST_F(SupervisorScenarioTest, AMovingArmDefersTheAdoptionUntilItRests) {
  // S8-I (2026-09-27 /code-review): a switch made while the previous
  // controller's motion is still running must not adopt a pose in passing.
  // Unarmed, the decision waits for rest and takes the pose the arm rests at.
  std::array<double, kUr5eArmDof> passing = kUr5eHome;
  passing[1] += 0.05;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, passing,
      /*arm=*/false));
  publishing_ = false;
  state_.devices[0].velocities[1] = 0.5;  // far above the homing arrival tolerance
  Ticks(5);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting()) << "adopted a pose the arm was passing";
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 0U) << "deferring is not refusing";

  std::array<double, kUr5eArmDof> rest = kUr5eHome;
  rest[1] += 0.12;
  state_ = MakeState(rest);
  Ticks(1);
  ASSERT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting());
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                rest[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i;
  }
}

TEST_F(SupervisorScenarioTest, ArmingWhileTheArmMovesRefusesTheSwitchedInPose) {
  // Homing needs its target on the tick the operator arms: an arm still moving
  // then gets the YAML pose, once, and a later rest does not re-decide.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off));
  publishing_ = false;
  state_.devices[0].velocities[0] = 0.5;
  Ticks(2);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_reason,
            integrated_bringup::CatchingDiagLogPod::WaitPoseRefusal::kMoving);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_joint, 0);
  state_ = MakeState(off);
  Ticks(3);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting()) << "one decision per activation";
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                kUr5eHome[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i;
  }
}

TEST_F(SupervisorScenarioTest, ASwitchInUnderAnEstopIsNotAdopted) {
  // Where the arm was STOPPED is not where the operator put it: an activation
  // whose deciding tick is under an E-STOP keeps the YAML pose, also after the
  // stop clears.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[2] -= 0.05;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off,
      /*arm=*/false));
  publishing_ = false;
  ctrl_->TriggerEstop();
  Ticks(2);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_reason,
            integrated_bringup::CatchingDiagLogPod::WaitPoseRefusal::kEstop);
  ctrl_->ClearEstop();
  Ticks(3);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
}

TEST_F(SupervisorScenarioTest, AMissedBallIsJudgedMissedAndTheHandStillWaitsForTheWaitPose) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = false;  // the fingertips see only their bias
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  const int r = static_cast<int>(log_.size()) - 1;
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kMissed) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kArmed});
  // A Missed verdict is only what the fingertips saw: a ball lying on the
  // links or the palm reads Missed too (sim 260923_2336, 1 of 25). So Missed
  // does not open the hand either — it opens at the wait pose.
  ASSERT_NO_FATAL_FAILURE(ExpectHeldUntilTheWaitPose(r));
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
  const std::int64_t gap_start = Now();
  pre_tick_ = [this, gap_start] {
    if (!publishing_ && Now() - gap_start >= 240 * kMsNs) {
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

// ── A reference step the generator refuses while stopping (#718) ────────────
//
// DECEL and HOLD do not count saturation: a saturated reference while stopping
// is the stop taking longer. A step the generator REFUSES is not that — there
// is no reference that tick and the arm command is not written. The supervisor
// used to pass over both alike, so the trial went on to HOLD and to a verdict
// with the arm command left where the refusal began.
//
// The refusal is injected through the tick's own `dt`: the generator rejects a
// step that is not positive. The shipped RT loop cannot produce one (its dt is
// 1 / control_rate), which is why this is a line of defence rather than a
// failure seen in a trial. The other way to a refusal — a generator whose own
// state went non-finite — cannot be reached from outside the controller.
class RefusedStopReferenceTest : public SupervisorScenarioTest {
 protected:
  void RefuseOneStepIn(Mode stage) {
    ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
    tips_enabled_ = true;
    ball_in_hand_ = true;
    ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
    ASSERT_TRUE(TickUntilMode(stage, 1500)) << Transitions();
    ASSERT_TRUE(log_.back().ref_valid) << "precondition: the law was stepping the reference";

    // The tick that ENTERED the stage was evaluated by the stage before it, so
    // the very next one is the stage's own evaluation. It has to be that one
    // for DECEL: on this profile the virtual target stops at once and DECEL
    // lasts a single tick.
    state_.dt = 0.0;
    const std::size_t refused = log_.size();
    Ticks(1);
    state_.dt = kDt;

    EXPECT_FALSE(log_[refused].ref_valid) << "the injected step was not refused";
    EXPECT_EQ(log_[refused].mode, Mode::kAbortSafe) << Window(static_cast<int>(refused), 3);
    EXPECT_EQ(log_[refused].reason, Reason::kParamsTbd);
    // The stop is the joint-space ramp, which needs neither the reference nor
    // CLIK, and the attempt ends without a verdict of its own.
    ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 600)) << Transitions();
    EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kAborted) << Transitions();
    ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  }
};

TEST_F(RefusedStopReferenceTest, InDecelItAbortsOnParamsTbd) {
  ASSERT_NO_FATAL_FAILURE(RefuseOneStepIn(Mode::kDecel));
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kAbortSafe, Mode::kRetreat, Mode::kArmed});
}

TEST_F(RefusedStopReferenceTest, InHoldItAbortsOnParamsTbd) {
  ASSERT_NO_FATAL_FAILURE(RefuseOneStepIn(Mode::kHold));
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kAbortSafe, Mode::kRetreat,
             Mode::kArmed});
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

// ── E-STOP in every stage of an attempt (S9a, D-S9-A/B/C) ───────────────────
//
// The case above stops an APPROACH. These stop the stages after it, where the
// hand is doing something: the reaction is the SAME in all of them (D-S9-B) —
// IDLE on the stop tick, disarmed, no resume on the clear — and the hand is not
// opened and not squeezed but held where it was measured (D-S9-A). In HOLD and
// RETREAT that pose is the closed one, which is the documented cost: under the
// CM's hold the position servo has no gap left to push with, so the ball goes.
//
// WHAT THIS DOES NOT SEE. These are the CONTROLLER's outputs. Under a real
// E-STOP the CM discards them and writes each device's measured position
// itself (rt_controller_node_rt_loop.cpp, BuildHoldOutput); that substitution
// has its own tests in rtc_controller_manager, and the sim run of S9a is what
// sees both together. What the controller's side has to guarantee is that the
// value it would fall back to is the same pose, so a CM that stopped
// substituting would not turn the stop into a motion.
//
// THE MEASUREMENT IS MOVED OFF THE LAST COMMAND BEFORE THE STOP TICK. The
// fixture's plant is a perfect servo — its next measurement is whatever it
// was just commanded — so without this a controller that REPLAYED its last
// command would be indistinguishable from one that held the MEASUREMENT, and
// D-S9-A is about the measurement. The stop tick is given a pose a few mrad
// away from the last command, and the snapshot compared against is that pose,
// taken before the tick (never read back from the tick that follows).

class EstopInStageTest : public SupervisorScenarioTest {
 protected:
  static constexpr double kHandOffset = 0.01;  // rad; + so q_pre = 0 stays inside the box
  static constexpr double kArmOffset = 0.002;  // rad; far below track_err_abort

  void StopIn(Mode stage) {
    ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
    tips_enabled_ = true;
    ball_in_hand_ = true;
    ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
    ASSERT_TRUE(TickUntilMode(stage, 1500)) << "never reached " << ModeName(stage) << '\n'
                                            << Transitions();
    const Outcome before = ctrl_->GetOutcomeForTesting();

    std::array<double, kP1bHandDof> hand_meas{};
    for (int i = 0; i < kP1bHandDof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      state_.devices[1].positions[u] += kHandOffset;
      hand_meas[u] = state_.devices[1].positions[u];
      ASSERT_NE(hand_meas[u], log_.back().hand_out[u]) << "precondition: measurement = command";
    }
    for (int i = 0; i < kUr5eArmDof; ++i) {
      state_.devices[0].positions[static_cast<std::size_t>(i)] += kArmOffset;
    }
    const std::size_t trig = log_.size();
    ctrl_->TriggerEstop();
    Ticks(20);
    const std::size_t clear = log_.size();
    ctrl_->ClearEstop();
    Ticks(100);  // the ball is still being published

    // D-S9-B: one reaction, on the stop tick.
    EXPECT_EQ(log_[trig].mode, Mode::kIdle) << Window(static_cast<int>(trig), 2);
    EXPECT_EQ(log_[trig].reason, Reason::kEstop) << Window(static_cast<int>(trig), 2);
    EXPECT_FALSE(log_[trig].armed_latch) << "the stop tick left the controller armed";
    for (std::size_t i = trig; i < clear; ++i) {
      ASSERT_EQ(log_[i].reason, Reason::kEstop) << Window(static_cast<int>(i), 2);
    }
    // P-1 (c) / D-S9-C: the clear does not resume anything.
    ExpectSeq({stage, Mode::kIdle}, trig);
    EXPECT_FALSE(ctrl_->IsArmRequested()) << "the clear re-armed the controller";
    // An attempt still in progress ends Aborted, and says so for the whole
    // stop. One already judged is NOT rewritten as Aborted (L7 §4.1): its
    // verdict went out on the RETREAT entry tick, which is where the trial
    // runner reads it, and the stop is not a second opinion on the catch.
    // (The CLEAR is a reset too and puts the field back to None — the next
    // attempt starts with no verdict — so the stop window is where it is read.)
    for (std::size_t i = trig; i < clear; ++i) {
      if (before == Outcome::kNone) {
        ASSERT_EQ(log_[i].outcome, Outcome::kAborted) << Window(static_cast<int>(i), 2);
      } else {
        ASSERT_NE(log_[i].outcome, Outcome::kAborted) << "the stop rewrote a judged attempt\n"
                                                      << Window(static_cast<int>(i), 2);
      }
    }

    // D-S9-A: the hand stays where it was measured — through the stop and after
    // the clear, until an operator re-arms.
    for (std::size_t i = trig; i < log_.size(); ++i) {
      ASSERT_TRUE(log_[i].hand_out_valid) << "tick " << i << " silenced the hand";
      for (int j = 0; j < kP1bHandDof; ++j) {
        const auto u = static_cast<std::size_t>(j);
        ASSERT_DOUBLE_EQ(log_[i].hand_out[u], hand_meas[u])
            << "tick " << i << " hand joint " << j << ": the hand moved "
            << (i < clear ? "during the stop" : "after the clear");
      }
    }
    // The arm likewise: the pose the stop tick measured, never a step away.
    for (std::size_t i = trig; i < log_.size(); ++i) {
      ASSERT_TRUE(log_[i].q_out_valid) << "tick " << i << " silenced the arm";
      for (int j = 0; j < kUr5eArmDof; ++j) {
        const auto u = static_cast<std::size_t>(j);
        ASSERT_DOUBLE_EQ(log_[i].q_out[u], log_[trig].q_meas[u])
            << "tick " << i << " arm joint " << j << ": the arm moved "
            << (i < clear ? "during the stop" : "after the clear");
      }
    }
    stop_hand_ = hand_meas;
    stop_tick_ = trig;
  }

  /// A stop in a stage where the hand has closed on the ball: the hand was
  /// commanded q_close (TrackingYaml: 0.5 on every joint) up to the stop, and
  /// the pose it is held at is the closed one as measured — not q_pre.
  void ExpectHeldClosed() const {
    ASSERT_GT(stop_tick_, 0U);
    const auto& last = log_[stop_tick_ - 1];
    for (int j = 0; j < kP1bHandDof; ++j) {
      const auto u = static_cast<std::size_t>(j);
      EXPECT_NEAR(last.hand_out[u], 0.5, 1e-9)
          << "hand joint " << j << " was not commanded closed when the stop landed";
      EXPECT_NEAR(stop_hand_[u], 0.5 + kHandOffset, 1e-9) << "hand joint " << j;
    }
  }

  std::array<double, kP1bHandDof> stop_hand_{};
  std::size_t stop_tick_{0};
};

TEST_F(EstopInStageTest, InCommittedTheArmAndHandHoldAndTheClearDoesNotResume) {
  ASSERT_NO_FATAL_FAILURE(StopIn(Mode::kCommitted));
}

TEST_F(EstopInStageTest, InClosingTheArmAndHandHoldAndTheClearDoesNotResume) {
  ASSERT_NO_FATAL_FAILURE(StopIn(Mode::kClosing));
}

TEST_F(EstopInStageTest, InDecelTheArmAndHandHoldAndTheClearDoesNotResume) {
  ASSERT_NO_FATAL_FAILURE(StopIn(Mode::kDecel));
}

TEST_F(EstopInStageTest, InHoldTheClosedHandIsHeldAndTheClearDoesNotResume) {
  ASSERT_NO_FATAL_FAILURE(StopIn(Mode::kHold));
  ExpectHeldClosed();
}

TEST_F(EstopInStageTest, InRetreatTheClosedHandIsHeldAndTheVerdictIsKept) {
  ASSERT_NO_FATAL_FAILURE(StopIn(Mode::kRetreat));
  ExpectHeldClosed();
  const int r = Entry(Mode::kRetreat);
  ASSERT_GT(r, 0);
  EXPECT_EQ(log_[static_cast<std::size_t>(r)].outcome, Outcome::kCaptured)
      << "the verdict the runner reads is not on the RETREAT entry tick\n"
      << Window(r, 2);
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

TEST_F(SupervisorScenarioTest, AServoStillLaggingAfterATrackErrAbortDoesNotCycleTheAbort) {
  // The measured arm stays 0.6 rad off its command for 60 ticks, well past the
  // stop. RETREAT waits in its stop for the servo instead of starting the
  // return, tripping TRACK_ERR and going round ABORT_SAFE ↔ RETREAT.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.0));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  Ticks(20);
  int lagging = 60;
  pre_tick_ = [this, &lagging] {
    if (lagging > 0) {
      state_.devices[0].positions[1] += 0.6;
      --lagging;
    }
  };
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 5)) << Transitions();
  const bool rearmed = TickUntilMode(Mode::kArmed, 800);
  pre_tick_ = nullptr;
  EXPECT_TRUE(rearmed) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed});
}

TEST_F(SupervisorScenarioTest, AnAbortDuringTheReturnKeepsTheJudgedVerdict) {
  // HOLD judged the attempt Captured; a TRACK_ERR during the return goes
  // RETREAT → ABORT_SAFE → RETREAT. That second RETREAT entry must not
  // rewrite the verdict as Aborted: the attempt ended at HOLD.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  // The return, not the stop: the carried speed RISING past 0.05 (the stop
  // only brings the HOLD's residual speed down, so the first tick seen here
  // is never "rising").
  double prev_speed = std::numeric_limits<double>::infinity();
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
  // RETREAT judges the error its motion stage measured on the tick before,
  // so the abort lands one tick after the kick.
  state_.devices[0].positions[1] += 0.6;
  const std::size_t kick = log_.size();
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 3)) << Window(static_cast<int>(kick), 3);
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kTrackErr);
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 600)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured)
      << "an abort after the verdict rewrote it\n"
      << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured);
}

TEST_F(SupervisorScenarioTest, ADisarmInTrackingWithAnAlignedArmLeavesAbortSafe) {
  // The arm starts at the wait pose, so IDLE takes the Q13 skip and nothing
  // seeds the arm command before APPROACH. A disarm in TRACKING goes to
  // ABORT_SAFE with no command to ramp; its exit must still open.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["enabled"] = false;  // TRACKING, and no plan to leave it
  }));
  ASSERT_TRUE(TickUntilMode(Mode::kTracking, 200)) << Transitions();
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.homing; }), 0)
      << "precondition: the Q13 skip, not a homing";
  SetArmed(false);
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 5)) << Transitions();
  EXPECT_TRUE(TickUntilMode(Mode::kIdle, 200)) << "stranded in ABORT_SAFE\n" << Transitions();
  ExpectSeq(
      {Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kAbortSafe, Mode::kRetreat, Mode::kIdle});
}

TEST_F(SupervisorScenarioTest, AnArmGateClosedMidHomingIsNotATrackingError) {
  // Mid-homing the arm reads as unreadable for 10 ticks, its position slots
  // zeroed. Those slots are not a measurement: the homing neither disarms on
  // TRACK_ERR nor stops, and arms once the gate reopens.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;
  off[2] -= 0.05;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, nullptr, off));
  publishing_ = false;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->IsHomingForTesting(); }, 50)) << Transitions();
  Ticks(10);
  std::array<double, kUr5eArmDof> saved{};
  int closed = 10;
  pre_tick_ = [this, &saved, &closed] {
    auto& a = state_.devices[0];
    if (closed == 10) {
      for (int i = 0; i < kUr5eArmDof; ++i) {
        saved[static_cast<std::size_t>(i)] = a.positions[static_cast<std::size_t>(i)];
      }
    }
    if (closed > 0) {
      a.valid = false;
      a.positions.fill(0.0);
      --closed;
    } else if (closed == 0) {
      a.valid = true;
      for (int i = 0; i < kUr5eArmDof; ++i) {
        a.positions[static_cast<std::size_t>(i)] = saved[static_cast<std::size_t>(i)];
      }
      --closed;
    }
  };
  Ticks(12);
  pre_tick_ = nullptr;
  EXPECT_TRUE(ctrl_->IsArmRequested()) << "a closed gate disarmed the homing\n" << Transitions();
  EXPECT_NE(ctrl_->GetLastReason(), Reason::kTrackErr);
  EXPECT_TRUE(TickUntilMode(Mode::kArmed, 800)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed});
}

// An abort in HOLD, after the close: the verdict is Aborted, and the hand
// keeps what it closed on until the arm is back — with or without confirmed
// contact. Q14 made the contact the condition; the 2026-09-24 rule does not
// ask (the fingertips' "no contact" is not "no ball"), so both runs below
// must hold to the wait pose.
class AbortInHoldTest : public SupervisorScenarioTest {
 protected:
  void AbortInHoldThenReturn(bool ball_in_hand) {
    ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
      y["catching"]["robot"]["hand"]["T_hold"] = 0.3;
    }));
    tips_enabled_ = true;
    ball_in_hand_ = ball_in_hand;
    ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
    ASSERT_TRUE(TickUntilMode(Mode::kHold, 1000)) << Transitions();
    Ticks(3);
    ASSERT_EQ(ctrl_->GetMode(), Mode::kHold) << Transitions();
    const auto& now_rec = log_.back();
    const auto confirmed = std::count(now_rec.tip_contact.begin(), now_rec.tip_contact.end(), true);
    if (ball_in_hand) {
      ASSERT_GE(confirmed, 2) << "precondition: contact is confirmed before the abort\n"
                              << Window(static_cast<int>(log_.size()) - 1, 3);
    } else {
      ASSERT_EQ(confirmed, 0) << "precondition: no fingertip is in contact\n"
                              << Window(static_cast<int>(log_.size()) - 1, 3);
    }
    // TRACK_ERR: the measured arm the next tick sees is 0.6 rad off its
    // command (> track_err_abort 0.5) — one tick only; the servo puts it back.
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
    ASSERT_NO_FATAL_FAILURE(ExpectHeldUntilTheWaitPose(r));
    EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kAborted)
        << "the verdict must survive the re-arm (reset table: outcome_ exempt from R)";
  }
};

TEST_F(AbortInHoldTest, AnAbortAfterConfirmedContactKeepsTheBallUntilTheReturn) {
  AbortInHoldThenReturn(true);
}

TEST_F(AbortInHoldTest, AnAbortWithNoContactStillKeepsTheHandClosedUntilTheReturn) {
  AbortInHoldThenReturn(false);
}

// ── #749: the stop takes over in one step ───────────────────────────────────
//
// Three reasons are known only after the law has integrated the command:
// TRACK_ERR (judged on the new command), REF_SATURATED (reported after the
// solve) and BALL_STALE_LONG (judged after a healthy law tick). On that tick
// the supervisor enters a mode whose motion is the joint-space ramp, and the
// ramp used to step the command the law had just stepped — two steps in one
// tick, about twice the previous step (1.9 – 2.4 times, measured on every row
// below; #749). One case per row of the transition table that can be reached
// that way, under each form of the CLIK's acceleration bound (kinematic,
// dynamic).

class StopEntryTest : public SupervisorScenarioTest,
                      public ::testing::WithParamInterface<const char*> {
 protected:
  /// A `sat_ticks` no streak in these cases reaches (the key must be positive).
  static constexpr int kNoSaturationAbort = 100000;

  std::function<void(YAML::Node&)> Form(
      const std::function<void(YAML::Node&)>& more = nullptr) const {
    const std::string form = GetParam();
    return [form, more](YAML::Node& y) {
      y["catching"]["joint_cmd"]["accel_constraint"] = form;
      if (form == "kinematic") {
        // The form's own keys, at the validator's ceilings (no shipped value).
        y["catching"]["joint_cmd"]["task_accel_max_linear"] = 500.0;
        y["catching"]["joint_cmd"]["task_accel_max_angular"] = 500.0;
      }
      if (more) {
        more(y);
      }
    };
  }

  /// G5-B's far target: the reference saturates from its first step and the
  /// arm is still moving at every instant these cases stop it.
  Eigen::Vector3d FarPc() const {
    return start_pose_.translation() + Eigen::Vector3d(0.45, -0.35, 0.30);
  }

  static void NoSaturationAbort(YAML::Node& y) {
    y["catching"]["supervisor"]["sat_ticks"] = kNoSaturationAbort;
  }
};

TEST_P(StopEntryTest, ATrackErrInApproach) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.0, Form()));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  Ticks(20);
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kApproach, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "APPROACH TRACK_ERR -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ATrackErrInCommitted) {
  // t_c 0.45 s out: APPROACH lasts 90 ms and the freeze finds the arm moving.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.45, Form()));
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 400)) << Transitions();
  Ticks(2);
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kCommitted, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "COMMITTED TRACK_ERR -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ATrackErrInClosing) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.4, Form()));
  ASSERT_TRUE(TickUntilMode(Mode::kClosing, 400)) << Transitions();
  Ticks(2);
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kClosing, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "CLOSING TRACK_ERR -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ATrackErrOnTheDecelEntryTick) {
  // The tick that would enter DECEL (now_lead >= t_c, still CLOSING): the law
  // it runs is the DECEL entry step, and CLOSING answers its TRACK_ERR.
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.4, Form(NoSaturationAbort)));
  ASSERT_TRUE(TickUntilMode(Mode::kClosing, 400)) << Transitions();
  std::size_t kick = 0;
  pre_tick_ = [this, &kick] {
    if (kick == 0 && ctrl_->GetMode() == Mode::kClosing &&
        Now() >= ctrl_->GetFollowedPlanForTesting().t_c_ns) {
      state_.devices[0].positions[1] += kTrackErrKick;
      kick = log_.size();
    }
  };
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() != Mode::kClosing; }, 400))
      << Transitions();
  pre_tick_ = nullptr;
  ASSERT_GT(kick, 0U) << Transitions();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kClosing, Reason::kTrackErr));
  ASSERT_LT(log_[kick - 1].before_ns, log_[kick - 1].plan_t_c_ns)
      << "the kick came after the DECEL entry tick";
  ExpectTheStopTakesOverInOneStep(kick, "CLOSING (DECEL entry step) TRACK_ERR -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ATrackErrInDecel) {
  // The arm is still moving at t_c, so DECEL lasts more than its entry tick.
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.4, Form(NoSaturationAbort)));
  ASSERT_TRUE(TickUntilMode(Mode::kDecel, 400)) << Transitions();
  Ticks(2);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kDecel) << Transitions();
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kDecel, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "DECEL TRACK_ERR -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ATrackErrInHold) {
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.4, Form([](YAML::Node& y) {
                                    NoSaturationAbort(y);
                                    y["catching"]["robot"]["hand"]["T_hold"] = 0.3;
                                  })));
  ASSERT_TRUE(TickUntilMode(Mode::kHold, 1000)) << Transitions();
  Ticks(3);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kHold) << Transitions();
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kHold, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "HOLD TRACK_ERR -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ASaturationInApproach) {
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 1.0, Form([](YAML::Node& y) {
                                    y["catching"]["supervisor"]["sat_ticks"] = 50;
                                  })));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 400)) << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(
      ExpectEdgeAt(entry, Mode::kApproach, Reason::kRefSaturated, Mode::kRetreat));
  ExpectTheStopTakesOverInOneStep(entry, "APPROACH REF_SATURATED -> RETREAT");
}

TEST_P(StopEntryTest, ASaturationInCommitted) {
  // SaturationAfterTheFreezeAbortsOnRefSaturated's timing: the 40-tick streak
  // completes inside COMMITTED.
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.4, Form([](YAML::Node& y) {
                                    y["catching"]["supervisor"]["sat_ticks"] = 40;
                                  })));
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 400)) << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kCommitted, Reason::kRefSaturated));
  ExpectTheStopTakesOverInOneStep(entry, "COMMITTED REF_SATURATED -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ASaturationInClosing) {
  // APPROACH is 20 ticks and COMMITTED 40: a 70-tick streak completes in CLOSING.
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.4, Form([](YAML::Node& y) {
                                    y["catching"]["supervisor"]["sat_ticks"] = 70;
                                  })));
  ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 400)) << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kClosing, Reason::kRefSaturated));
  ExpectTheStopTakesOverInOneStep(entry, "CLOSING REF_SATURATED -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ALongStaleInCommitted) {
  // COMMITTED from 50 ms to 570 ms; the lane goes quiet at the freeze and is
  // long-stale 0.22 s later, with the arm still on its way.
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.85, Form([](YAML::Node& y) {
                                    NoSaturationAbort(y);
                                    y["catching"]["planner"]["freeze"]["T_freeze"] = 0.8;
                                    y["catching"]["supervisor"]["stale_committed_max_s"] = 0.02;
                                  })));
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 400)) << Transitions();
  publishing_ = false;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() != Mode::kCommitted; }, 400))
      << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kCommitted, Reason::kBallStaleLong));
  ExpectTheStopTakesOverInOneStep(entry, "COMMITTED BALL_STALE_LONG -> ABORT_SAFE");
}

TEST_P(StopEntryTest, ALongStaleInClosing) {
  // The lane goes quiet at the freeze (40 ms); long-stale 0.3 s later is
  // inside CLOSING (120 ms – 400 ms).
  ASSERT_NO_FATAL_FAILURE(BringUp(FarPc(), StartAxis(), 0.0, 0.4, Form(NoSaturationAbort)));
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 400)) << Transitions();
  publishing_ = false;
  ASSERT_TRUE(TickUntil(
      [this] {
        const Mode m = ctrl_->GetMode();
        return m != Mode::kCommitted && m != Mode::kClosing;
      },
      400))
      << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kClosing, Reason::kBallStaleLong));
  ExpectTheStopTakesOverInOneStep(entry, "CLOSING BALL_STALE_LONG -> ABORT_SAFE");
}

// The normal end of a trial (HOLD → RETREAT once T_hold is over) is not in the
// table above: it is not an abort, the law's step stays, and the command is
// pinned as it is by kNormalTrialCommandDigest (L7 §4.1). What that leaves is
// bounded here at the hold the parser defaults to and the shipped YAML does not
// override: by then the command has settled, and the law's step plus the
// ramp's is inside what one tick allows (measured 1.5e-7 rad the tick before
// and on it, under both forms).
TEST_P(StopEntryTest, TheNormalEndOfHoldStaysInsideTheStepBoundAtTheShippedHold) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, Form([](YAML::Node& y) {
                                    y["catching"]["robot"]["hand"]["T_hold"] = 0.5;
                                  })));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kHold, Reason::kNone, Mode::kRetreat));
  ASSERT_GE(entry, 2U);
  ASSERT_TRUE(log_[entry].body.clik_ran) << "precondition: HOLD's law did not run on its last tick";
  const std::vector<double> qdd = DerivedQddMax();
  ASSERT_GE(qdd.size(), static_cast<std::size_t>(kUr5eArmDof));
  double step_before = 0.0;
  double step_entry = 0.0;
  for (int j = 0; j < kUr5eArmDof; ++j) {
    const auto u = static_cast<std::size_t>(j);
    const double last = std::abs(log_[entry - 1].q_out[u] - log_[entry - 2].q_out[u]);
    const double step = std::abs(log_[entry].q_out[u] - log_[entry - 1].q_out[u]);
    step_before = std::max(step_before, last);
    step_entry = std::max(step_entry, step);
    EXPECT_LE(step, last + qdd[u] * kDt * kDt + 1e-12)
        << "joint " << j << " moved " << step << " rad on the tick HOLD ended, " << last
        << " rad the tick before\n"
        << Window(static_cast<int>(entry), 2);
  }
  std::printf(
      "[ MEASURED ] #749 HOLD (T_hold 0.5 s over) -> RETREAT, %s: max |dq| [rad] before %.3e, on "
      "the tick HOLD ended %.3e\n",
      GetParam(), step_before, step_entry);
}

INSTANTIATE_TEST_SUITE_P(Forms, StopEntryTest, ::testing::Values("kinematic", "dynamic"));

/// The same rows under the mpc law (the segment follower), where TRACK_ERR and
/// BALL_STALE_LONG are the two reasons that follow the command — it has no
/// reference generator, so nothing saturates. The first segment is at rest
/// until t_c − 0.3 s and moves from there to t_c + 0.1 s.
class StopEntryMpcTest : public DecelMpcScenarioTest {
 protected:
  /// A freeze and a close short enough that APPROACH and COMMITTED still hold
  /// the arm while the segment moves it: APPROACH to t_c − 0.15 s, COMMITTED
  /// to t_c − 0.10 s.
  static void LateFreeze(YAML::Node& y) {
    y["catching"]["robot"]["hand"]["T_close_e2e"] = 0.10;
    y["catching"]["planner"]["freeze"]["T_freeze"] = 0.15;
  }
};

TEST_F(StopEntryMpcTest, ATrackErrInApproach) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc(LateFreeze));
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickToJustBefore(first_seg_.t_c_ns - 200 * kMsNs)) << Transitions();
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kApproach, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "mpc APPROACH TRACK_ERR -> ABORT_SAFE");
}

TEST_F(StopEntryMpcTest, ATrackErrInCommitted) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc(LateFreeze));
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 1500)) << Transitions();
  Ticks(5);
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kCommitted, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "mpc COMMITTED TRACK_ERR -> ABORT_SAFE");
}

TEST_F(StopEntryMpcTest, ATrackErrInClosing) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickToJustBefore(first_seg_.t_c_ns - 100 * kMsNs)) << Transitions();
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kClosing, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "mpc CLOSING TRACK_ERR -> ABORT_SAFE");
}

TEST_F(StopEntryMpcTest, ATrackErrOnTheDecelEntryTick) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kClosing, 1500)) << Transitions();
  std::size_t kick = 0;
  pre_tick_ = [this, &kick] {
    if (kick == 0 && ctrl_->GetMode() == Mode::kClosing && Now() >= first_seg_.t_c_ns) {
      state_.devices[0].positions[1] += kTrackErrKick;
      kick = log_.size();
    }
  };
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() != Mode::kClosing; }, 400))
      << Transitions();
  pre_tick_ = nullptr;
  ASSERT_GT(kick, 0U) << Transitions();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kClosing, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "mpc CLOSING (DECEL entry step) TRACK_ERR -> ABORT_SAFE");
}

TEST_F(StopEntryMpcTest, ATrackErrInDecel) {
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kDecel, 1500)) << Transitions();
  Ticks(3);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kDecel) << Transitions();
  const std::size_t kick = KickTrackErr();
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(kick, Mode::kDecel, Reason::kTrackErr));
  ExpectTheStopTakesOverInOneStep(kick, "mpc DECEL TRACK_ERR -> ABORT_SAFE");
}

TEST_F(StopEntryMpcTest, ALongStaleInClosing) {
  // Quiet from the freeze (t_c − 0.36 s): long-stale 0.3 s later, in CLOSING,
  // with the segment cruising.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc());
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 1500)) << Transitions();
  publishing_ = false;
  ASSERT_TRUE(TickUntil(
      [this] {
        const Mode m = ctrl_->GetMode();
        return m != Mode::kCommitted && m != Mode::kClosing;
      },
      400))
      << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kClosing, Reason::kBallStaleLong));
  ExpectTheStopTakesOverInOneStep(entry, "mpc CLOSING BALL_STALE_LONG -> ABORT_SAFE");
}

TEST_F(StopEntryMpcTest, ALongStaleInCommitted) {
  // COMMITTED from t_c − 0.36 s to t_c − 0.10 s; quiet from the freeze,
  // long-stale 0.22 s later (t_c − 0.14 s), the segment near its cruise speed.
  ASSERT_NO_FATAL_FAILURE(BringUpMpc([](YAML::Node& y) {
    y["catching"]["robot"]["hand"]["T_close_e2e"] = 0.10;
    y["catching"]["supervisor"]["stale_committed_max_s"] = 0.02;
  }));
  ASSERT_NO_FATAL_FAILURE(FollowThePair());
  ASSERT_TRUE(TickUntilMode(Mode::kCommitted, 1500)) << Transitions();
  publishing_ = false;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetMode() != Mode::kCommitted; }, 400))
      << Transitions();
  const std::size_t entry = log_.size() - 1;
  ASSERT_NO_FATAL_FAILURE(ExpectEdgeAt(entry, Mode::kCommitted, Reason::kBallStaleLong));
  ExpectTheStopTakesOverInOneStep(entry, "mpc COMMITTED BALL_STALE_LONG -> ABORT_SAFE");
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
  ctrl_->PlanBoxForTesting().Store(plan_for(42, 1, Now()));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 5)) << Transitions();
  Ticks(20);
  // The re-throw: a new ball, which ends the attempt (TRACK_CHANGED).
  generation_ = 43;
  PublishNow();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 5)) << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kTrackChanged);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  // The re-arm tick takes the reset floor from its own clock read; on the
  // stepped clock that read IS the tick's before stamp, so "before the re-arm"
  // is one nanosecond earlier (on the real clock the gap was the stamp-to-read
  // latency).
  const std::int64_t before_rearm = log_.back().before_ns - 1;
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
  ctrl_->PlanBoxForTesting().Store(plan_for(43, 3, Now()));
  Ticks(1);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();
  EXPECT_EQ(ctrl_->GetPlanAdmittedCount(), 2U);
  EXPECT_EQ(ctrl_->GetFollowedPlanForTesting().plan_id, 3U);
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kRetreat,
             Mode::kArmed, Mode::kTracking, Mode::kApproach});
}

// ════════════════════════════════════════════════════════════════════════════
// #537 S8-C — the hand-joint capture witness (D-S8-8 (b)) and RETREAT's
// release timeout (D-S8-6)
// ════════════════════════════════════════════════════════════════════════════

constexpr double kHandTauMax = 3.0;

/// A capture block the fixture hand (q_pre 0 → q_close 0.5) can satisfy, with
/// a HOLD long enough to hold t_persist.
void CaptureTweak(YAML::Node& y) {
  YAML::Node hand = y["catching"]["robot"]["hand"];
  hand["T_hold"] = 0.3;
  YAML::Node cap = hand["capture"];
  cap["rho_min"] = 0.2;
  cap["rho_max"] = 0.9;
  cap["effort_frac_min"] = 0.5;
  cap["t_persist"] = 0.1;
  // This fixture declares no backend, so the configure is judged a REAL-ARM
  // one — where the default `provisional: true` parks the controller (the
  // effort lane of a real hand need not be a torque).
  cap["provisional"] = false;
}

TEST_F(SupervisorScenarioTest, AStalledHandCapturesABallTheFingertipsMissed) {
  // The ball rests on the links: no fingertip sees it, but a finger stopped at
  // ρ 0.6 pushing at 0.9·max_torque. The verdict is Captured — by the hand.
  hand_max_torque_ = std::vector<double>(kP1bHandDof, kHandTauMax);
  hand_stall_ = HandStall{5, 0.6 * 0.5, 0.9 * kHandTauMax};
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, CaptureTweak));
  tips_enabled_ = true;
  ball_in_hand_ = false;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  EXPECT_EQ(log_.back().outcome_source, 2) << "the verdict must name the hand as its witness";
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.mode == Mode::kHold && t.hand_blocked; }),
            0);
  // The release rule does not read the verdict: back at the wait pose, re-armed.
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
}

TEST_F(SupervisorScenarioTest, AFingerStoppedWithoutPushingIsNotACapture) {
  // Negative control of the case above: the same stall, no load on it.
  hand_max_torque_ = std::vector<double>(kP1bHandDof, kHandTauMax);
  hand_stall_ = HandStall{5, 0.6 * 0.5, 0.0};
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, CaptureTweak));
  tips_enabled_ = true;
  ball_in_hand_ = false;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kMissed) << Transitions();
  EXPECT_EQ(log_.back().outcome_source, 0);
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.hand_blocked; }), 0);
}

TEST_F(SupervisorScenarioTest, AStallShorterThanTPersistIsNotACapture) {
  // The finger stops only in the last ~0.1 s of a 0.3 s HOLD, against a
  // t_persist of 0.25 s: blocked at the end, but not for long enough.
  hand_max_torque_ = std::vector<double>(kP1bHandDof, kHandTauMax);
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    CaptureTweak(y);
    y["catching"]["robot"]["hand"]["capture"]["t_persist"] = 0.25;
  }));
  tips_enabled_ = true;
  ball_in_hand_ = false;
  std::int64_t hold_seen_ns = 0;
  pre_tick_ = [this, &hold_seen_ns] {
    if (ctrl_->GetMode() != Mode::kHold) {
      return;
    }
    const std::int64_t now = Now();
    if (hold_seen_ns == 0) {
      hold_seen_ns = now;
    }
    if (!hand_stall_ && now - hold_seen_ns >= 200 * kMsNs) {
      hand_stall_ = HandStall{5, 0.6 * 0.5, 0.9 * kHandTauMax};
    }
  };
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  const int r = Entry(Mode::kRetreat);
  ASSERT_GT(r, 0);
  EXPECT_TRUE(log_[static_cast<std::size_t>(r - 1)].hand_blocked)
      << "precondition: the hand was blocked when HOLD ended";
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kMissed) << Transitions();
  EXPECT_EQ(log_.back().outcome_source, 0);
}

TEST_F(SupervisorScenarioTest, FingertipsAndAStalledHandAreRecordedAsBoth) {
  hand_max_torque_ = std::vector<double>(kP1bHandDof, kHandTauMax);
  hand_stall_ = HandStall{5, 0.6 * 0.5, 0.9 * kHandTauMax};
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, CaptureTweak));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kCaptured) << Transitions();
  EXPECT_EQ(log_.back().outcome_source, 3);
}

TEST_F(SupervisorScenarioTest, TheHandWitnessDoesNotLiftAStaleFingertipLane) {
  // G7-G with a stalled hand: a lane that could not judge stays Undetermined —
  // the hand adds evidence, it does not stand in for a stale fingertip.
  hand_max_torque_ = std::vector<double>(kP1bHandDof, kHandTauMax);
  hand_stall_ = HandStall{5, 0.6 * 0.5, 0.9 * kHandTauMax};
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, CaptureTweak));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  contact_tips_ = {true, true, false, false};
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kClosing, 600)) << Transitions();
  tip_stamp_frozen_[0] = true;
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1000)) << Transitions();
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kUndetermined) << Transitions();
  EXPECT_GT(CountTicks([](const TickRec& t) { return t.hand_blocked; }), 0)
      << "precondition: the hand witness held";
}

TEST_F(SupervisorScenarioTest, ACaptureBlockWithoutTheHandTorqueLimitsParksTheTrials) {
  // Asked for, but unusable: the controller is PARKED (A-S5-12 — the robot
  // comes up, this controller refuses to activate) rather than silently
  // judging by the fingertips alone.
  ctrl_ = std::make_unique<DemoCatchingController>("");
  ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
  ctrl_->SetSharedModelBuilder(builder_);
  ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
  YAML::Node yaml = YAML::Load(TrackingYaml(topic_, NearPc(), StartAxis(), 0.0, 0.6));
  CaptureTweak(yaml);
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
  EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kSupervisorUnset);
  EXPECT_FALSE(ctrl_->AreTrialsEnabled());
  EXPECT_NE(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

TEST_F(SupervisorScenarioTest, AReleaseTimeoutNotAboveTheCloseTimeParksTheTrials) {
  // An explicit T_release_timeout at or below T_close_e2e (0.28 here) would
  // time out every release and disarm after every catch. The validator's line
  // for the key is only a warning at configure (it shares T_close_e2e's
  // exemption from the consumed gate), so the supervisor refuses it itself.
  for (const double t : {0.28, 0.1}) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    YAML::Node yaml = YAML::Load(TrackingYaml(topic_, NearPc(), StartAxis(), 0.0, 0.6));
    yaml["catching"]["robot"]["hand"]["T_release_timeout"] = t;
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS)
        << t;
    EXPECT_TRUE(ctrl_->IsSimOnlyDisabled()) << t;
    EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kSupervisorUnset)
        << t;
    EXPECT_FALSE(ctrl_->AreTrialsEnabled()) << t;
  }
}

TEST_F(SupervisorScenarioTest, AHandThatNeverReachesQPreEndsTheReturnInIdleDisarmed) {
  // D-S8-6 (a): the hand freezes the moment it is released, so it never gets
  // back to q_pre. RETREAT waits T_release_timeout, then ends in IDLE,
  // disarmed on that same tick — and stays there until the operator re-arms.
  // Just above the fixture's T_close_e2e (0.28), which the validator requires.
  constexpr std::int64_t kTimeoutNs = 350 * kMsNs;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["catching"]["robot"]["hand"]["T_release_timeout"] = 0.35;
  }));
  // Frozen from RETREAT entry: RETREAT does not move the hand until the
  // release, and a servo still running on the release tick would put it at
  // q_pre before the controller ever waited.
  pre_tick_ = [this] {
    if (ctrl_->GetMode() == Mode::kRetreat) {
      servo_hand_ = false;
    }
  };
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kIdle, 1500)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kIdle});
  const int idle = Entry(Mode::kIdle);
  ASSERT_GT(idle, 0);
  const auto& entry = log_[static_cast<std::size_t>(idle)];
  EXPECT_EQ(entry.reason, Reason::kHandTimeout) << Window(idle, 2);
  EXPECT_FALSE(entry.armed_latch) << "the timeout must disarm on its own tick";
  // The clock started at the release: the first RETREAT tick in Release.
  int release = -1;
  for (int i = Entry(Mode::kRetreat); i < idle; ++i) {
    if (log_[static_cast<std::size_t>(i)].phase == HandPhase::kRelease) {
      release = i;
      break;
    }
  }
  ASSERT_GT(release, 0) << Transitions();
  const auto& rel = log_[static_cast<std::size_t>(release)];
  EXPECT_GE(entry.after_ns - rel.before_ns, kTimeoutNs) << "RETREAT gave up early";
  EXPECT_LT(entry.before_ns - rel.after_ns, kTimeoutNs + 20 * kMsNs)
      << "RETREAT gave up " << static_cast<double>(entry.before_ns - rel.after_ns) * 1e-6
      << " ms after the release";
  // HOLD judged the attempt; the timeout does not rewrite it as an abort.
  EXPECT_NE(ctrl_->GetOutcomeForTesting(), Outcome::kAborted);

  // A hand that settles later does not re-arm by itself...
  pre_tick_ = nullptr;
  servo_hand_ = true;
  Ticks(150);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle) << Transitions();
  EXPECT_FALSE(ctrl_->IsArmRequested());
  // ...and the operator's re-arm brings the cycle back.
  SetArmed(true);
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
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
  // Undetermined from HOLD keeps the hand closed into RETREAT, like every verdict.
  EXPECT_EQ(log_.back().phase, HandPhase::kHold);
}

// ════════════════════════════════════════════════════════════════════════════
// G8-H — every branch that decides to do less publishes THIS tick's body
// ════════════════════════════════════════════════════════════════════════════
//
// PROC-7 (L8 §9, G8-H). Compute() has one exit and default-constructs the
// record at the top of every tick, so a branch cannot skip the publish — but
// that is a property of today's layout, and S8-E counted only two of the nine
// branches (E-STOP, stale input: test_demo_catching_controller.cpp's
// `DemoCatchingRecord`) as pinned by a test. These are the other seven, each
// driven through the real supervisor rather than asserted from the layout.
//
// Each case checks two things on the branch's tick: the IDENTITY fields say it
// is this tick's row (a republished previous row and a skipped tick look the
// same downstream otherwise), and the BLOCK the branch is about says what this
// tick found — including, where the branch computes less, that the block is
// empty rather than the previous tick's.

class TickBodyTest : public SupervisorScenarioTest {
 protected:
  /// The row the tick at `i` published names that tick and its decision.
  void ExpectThisTicksBody(std::size_t i, const char* branch) const {
    ASSERT_LT(i, log_.size());
    const auto& r = log_[i];
    EXPECT_EQ(r.body.tick, r.iteration) << branch << ": the row is not from this tick\n"
                                        << Window(static_cast<int>(i), 2);
    EXPECT_DOUBLE_EQ(r.body.t_relative_s, r.t_relative_s) << branch;
    EXPECT_EQ(r.body.mode, static_cast<std::uint8_t>(r.mode)) << branch;
    EXPECT_EQ(r.body.reason, static_cast<std::uint8_t>(r.reason)) << branch;
  }

  /// The first log index at or after `from` whose tick satisfies `pred`.
  int First(const std::function<bool(const TickRec&)>& pred, std::size_t from = 0) const {
    for (std::size_t i = from; i < log_.size(); ++i) {
      if (pred(log_[i])) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }
};

TEST_F(TickBodyTest, AGenerationMismatchTickCarriesTheSnapshotItRefused) {
  // A snapshot received under the PREVIOUS activation is still in the box when
  // the controller is re-activated (D-23: the subscription outlives the
  // activation). The first tick after must judge it stale on the generation
  // alone — it is younger than t_stale — and say so in its own row.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  ASSERT_TRUE(TickUntilMode(Mode::kTracking, 200)) << Transitions();
  const std::uint32_t old_gen = ctrl_->ActivationGeneration();
  ASSERT_EQ(log_.back().body.input_activation_generation, old_gen) << "precondition";
  ASSERT_FALSE(log_.back().body.input_stale) << "precondition: the lane was fresh";

  publishing_ = false;
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_NE(ctrl_->ActivationGeneration(), old_gen) << "precondition: the generation moved";
  const std::size_t t = log_.size();
  Ticks(1);

  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(t, "generation mismatch"));
  const auto& b = log_[t].body;
  EXPECT_TRUE(b.input_valid) << "the refused snapshot is a valid one";
  EXPECT_EQ(b.input_activation_generation, old_gen) << "the row does not name what it refused";
  EXPECT_GE(b.input_age_s, 0.0);
  EXPECT_LT(b.input_age_s, 0.2) << "precondition: the snapshot was still inside t_stale";
  EXPECT_TRUE(b.input_stale) << "a previous activation's snapshot was not refused";
  EXPECT_EQ(log_[t].mode, Mode::kIdle) << "an activation starts in IDLE";
  EXPECT_FALSE(b.plan_valid);
  EXPECT_FALSE(b.ref_valid);
}

TEST_F(TickBodyTest, AHorizonExtrapTickCarriesTheExhaustedWindow) {
  // No other test reaches HORIZON_EXTRAP. Fresh messages (received now) whose
  // every sample is already behind the tick: the window is exhausted while the
  // lane is not stale, which is the one combination that names this reason.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  stamp_age_ns_ = 400 * kMsNs;  // > the 0.35 s the fixture cloud spans
  last_pub_ns_ = 0;             // the next tick publishes one
  const std::size_t from = log_.size();
  ASSERT_TRUE(TickUntil([this] { return ctrl_->GetLastReason() == Reason::kHorizonExtrap; }, 50))
      << Transitions();
  const auto t = static_cast<std::size_t>(log_.size() - 1);

  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(t, "horizon exhausted"));
  const auto& b = log_[t].body;
  EXPECT_TRUE(b.input_valid);
  EXPECT_FALSE(b.input_stale) << "the reason should have been BALL_STALE, not HORIZON_EXTRAP";
  EXPECT_TRUE(b.input_expired);
  EXPECT_NEAR(b.input_horizon_s, 0.35, 1e-6) << "the row does not carry the window it judged";
  EXPECT_EQ(log_[t].mode, Mode::kRetreat) << "{APPROACH, HORIZON_EXTRAP} → RETREAT";
  ExpectSeq({Mode::kApproach, Mode::kRetreat}, from);
}

TEST_F(TickBodyTest, ANoPlanTickCarriesTheBallAndAnEmptyPlan) {
  // TRACKING with a fresh ball and no plan anywhere (no planner, no oracle):
  // the tick saw the ball, found nothing to follow, and its row says both.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["enabled"] = false;
  }));
  Ticks(80);
  const auto t = static_cast<std::size_t>(log_.size() - 1);
  ASSERT_EQ(log_[t].mode, Mode::kTracking) << Transitions();
  ASSERT_EQ(log_[t].reason, Reason::kNoCatchablePlan);

  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(t, "no plan"));
  const auto& b = log_[t].body;
  EXPECT_TRUE(b.input_valid);
  EXPECT_FALSE(b.input_stale);
  EXPECT_EQ(b.input_generation, generation_);
  EXPECT_FALSE(b.plan_valid);
  EXPECT_EQ(b.plan_id, 0U);
  EXPECT_DOUBLE_EQ(b.plan_p_c[0], 0.0);
  EXPECT_FALSE(b.ref_valid) << "no plan, yet the row carries a reference";
  EXPECT_FALSE(b.clik_ran);
}

TEST_F(TickBodyTest, AnAbortSafeTickCarriesTheRampAndNoReference) {
  // After the entry, ABORT_SAFE runs the joint-space ramp and nothing else: no
  // reference, no CLIK. Its rows carry the ramp's command of THAT tick and an
  // empty reference/CLIK block, not the last law tick's.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.0));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  Ticks(20);
  ASSERT_TRUE(log_.back().body.ref_valid) << "precondition: the law was running";
  state_.devices[0].positions[1] += 0.6;  // one tick of TRACK_ERR (see ATrackErrAbort…)
  const std::size_t kick = log_.size();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 400)) << Transitions();
  ASSERT_EQ(log_[kick].mode, Mode::kAbortSafe) << Window(static_cast<int>(kick), 3);

  int n = 0;
  for (std::size_t i = kick + 1; i < log_.size() && log_[i].mode == Mode::kAbortSafe; ++i) {
    ++n;
    ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(i, "ABORT_SAFE"));
    const auto& b = log_[i].body;
    ASSERT_FALSE(b.ref_valid) << "tick " << i << " carries a reference ABORT_SAFE did not compute";
    ASSERT_FALSE(b.clik_ran) << "tick " << i;
    for (int j = 0; j < kUr5eArmDof; ++j) {
      const auto u = static_cast<std::size_t>(j);
      ASSERT_DOUBLE_EQ(b.q_cmd[u], log_[i].q_out[u])
          << "tick " << i << " joint " << j << ": the row's command is not the one that went out";
    }
  }
  ASSERT_GT(n, 0) << "precondition: the abort ended on its entry tick\n" << Transitions();
  const int last = static_cast<int>(kick) + n;
  // The edge is taken at the NEXT tick's head (EvaluateReason reads
  // abort_stopped_), so the stop is recorded on the last ABORT_SAFE tick.
  EXPECT_TRUE(log_[static_cast<std::size_t>(last)].body.abort_stopped)
      << "the last ABORT_SAFE tick does not say the arm stopped\n"
      << Window(last, 2);
}

TEST_F(TickBodyTest, AHandStageEarlyReturnEmptiesTheHandBlock) {
  // A disarm in ARMED: the hand stage returns early (disarmed IDLE — the latch
  // takes the hand), so the row that says IDLE must not still carry ARMED's
  // sequencer phase.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 200)) << Transitions();
  Ticks(5);
  ASSERT_TRUE(log_.back().body.hand_phase_valid) << "precondition: the sequencer had the hand";
  SetArmed(false);
  const std::size_t t = log_.size();
  Ticks(1);

  ASSERT_EQ(log_[t].mode, Mode::kIdle) << Window(static_cast<int>(t), 2);
  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(t, "hand stage early return"));
  const auto& b = log_[t].body;
  EXPECT_FALSE(b.hand_phase_valid) << "the row carries a sequencer phase the tick did not run";
  EXPECT_DOUBLE_EQ(b.hand_rho, 0.0);
  EXPECT_FALSE(b.hand_timeout);
  EXPECT_EQ(b.hand_stalled_n, 0);
  EXPECT_TRUE(std::isnan(b.hand_effort_frac)) << "a witness the tick did not compute";
}

TEST_F(TickBodyTest, AHandTimeoutTickIsTheDisarmingTicksRow) {
  // The HAND_TIMEOUT a test can reach is RETREAT's release timeout (D-S8-6 (a),
  // the AHandThatNeverReachesQPre… setup): CLOSING/DECEL record the sequencer's
  // close timeout only if it fires inside them, and it cannot — it is
  // 2·T_close_e2e after the close, and CLOSING is T_close_e2e long. The edge
  // disarms and ends in IDLE on one tick, so that tick's row must say IDLE,
  // disarmed, and the latch holding the hand — not the Release it left.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["catching"]["robot"]["hand"]["T_release_timeout"] = 0.35;
  }));
  pre_tick_ = [this] {
    if (ctrl_->GetMode() == Mode::kRetreat) {
      servo_hand_ = false;
    }
  };
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kIdle, 1500)) << Transitions();
  const int t = First([](const TickRec& r) { return r.reason == Reason::kHandTimeout; });
  ASSERT_GE(t, 1) << "HAND_TIMEOUT was never recorded\n" << Transitions();

  const auto u = static_cast<std::size_t>(t);
  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(u, "HAND_TIMEOUT"));
  ASSERT_EQ(log_[u - 1].body.hand_phase, static_cast<std::uint8_t>(HandPhase::kRelease))
      << "precondition: the tick before was waiting on the release";
  const auto& b = log_[u].body;
  EXPECT_EQ(log_[u].mode, Mode::kIdle);
  EXPECT_FALSE(b.armed) << "the row does not show the disarm its own tick did";
  EXPECT_FALSE(b.hand_phase_valid) << "the row carries the Release the tick left";
  EXPECT_FALSE(b.hand_timeout);
}

TEST_F(TickBodyTest, TheHandWitnessIsThisTicksAndEmptyOnceTheHandLetsGo) {
  // D-S8-8 (b): the hand-joint capture witness is computed only while the hand
  // is in Hold. Its rows carry this tick's reading while it is; the first tick
  // after the release carries NONE — NaN, not the last stall — because a
  // witness the tick did not compute must not read as one it did.
  hand_max_torque_ = std::vector<double>(kP1bHandDof, kHandTauMax);
  hand_stall_ = HandStall{5, 0.6 * 0.5, 0.9 * kHandTauMax};
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, CaptureTweak));
  tips_enabled_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();  // through the release

  const int s = First([](const TickRec& r) { return r.body.hand_stalled_n > 0; });
  ASSERT_GE(s, 0) << "precondition: the witness never saw the stall\n" << Transitions();
  const auto su = static_cast<std::size_t>(s);
  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(su, "hand witness"));
  EXPECT_EQ(log_[su].phase, HandPhase::kHold);
  EXPECT_TRUE(std::isfinite(log_[su].body.hand_effort_frac));
  EXPECT_NEAR(log_[su].body.hand_effort_frac, 0.9, 1e-6) << "not this tick's effort";

  const int rel = First([](const TickRec& r) { return r.phase == HandPhase::kRelease; }, su);
  ASSERT_GT(rel, s) << "precondition: the hand never released\n" << Transitions();
  const auto ru = static_cast<std::size_t>(rel);
  ASSERT_NO_FATAL_FAILURE(ExpectThisTicksBody(ru, "hand released"));
  EXPECT_EQ(log_[ru].body.hand_stalled_n, 0) << "the release row carries the last stall";
  EXPECT_TRUE(std::isnan(log_[ru].body.hand_effort_frac))
      << "the release row carries a witness it did not compute";
  EXPECT_DOUBLE_EQ(log_[ru].body.hand_blocked_s, 0.0);
}

// ════════════════════════════════════════════════════════════════════════════
// #537 S9b — the FAULT extension (D-S9-D1 / D2 / D3 / K / L)
// ════════════════════════════════════════════════════════════════════════════
//
// What each decision adds, and the case that pins it:
//   D1  a stop (ABORT_SAFE ramp, RETREAT stop) or a return that runs past its
//       deadline latches a fault — ABORT_ESCALATED → FAULT — and FAULT brings
//       the carried command to rest inside the acceleration box;
//   K   RETREAT does not move an arm it cannot read: the return is ramped to
//       rest and held, its deadline keeps running, and a readable arm resumes;
//   D2  n_qp counts TRIALS ended by a CLIK failure — a failure after good
//       solves counts, a verdict at HOLD ends the run, nothing else does;
//   D3  a fault reset is refused until the arm is at rest (command AND
//       measurement, velocity lane vouched for);
//   L   a reset under an E-STOP drops the latch at once and leaves FAULT on
//       the clear.
// The within-deadline counterparts of D1 are the existing cases that abort,
// lag or return and re-arm: ATrackErrAbort…, AServoStillLagging… (the 60-tick
// lag), and every trial that reaches ARMED again — all run with the default
// deadlines, unchanged.

using FaultCause = integrated_bringup::CatchingDiagLogPod::FaultCause;
using FaultResetRefusal = integrated_bringup::CatchingDiagLogPod::FaultResetRefusal;

class FaultExtensionTest : public SupervisorScenarioTest {
 protected:
  static double Speed(const TickRec& r) {
    double s = 0.0;
    for (double v : r.qd_cmd) {
      s = std::max(s, std::abs(v));
    }
    return s;
  }

  /// First tick at or after `from` satisfying `pred`, or -1.
  int First(const std::function<bool(const TickRec&)>& pred, std::size_t from = 0) const {
    for (std::size_t i = from; i < log_.size(); ++i) {
      if (pred(log_[i])) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  static std::function<void(YAML::Node&)> Deadlines(double stop_s, double return_s) {
    return [stop_s, return_s](YAML::Node& y) {
      y["catching"]["supervisor"]["deadline"]["stop_s"] = stop_s;
      y["catching"]["supervisor"]["deadline"]["return_s"] = return_s;
    };
  }

  static void OracleOff(YAML::Node& y) { y["diagnostic"]["oracle_plan"]["enabled"] = false; }

  /// A plan for `generation`, published now, catching at NearPc() `t_c_s`
  /// from now. `degenerate_axis` zeroes the approach axis — the plan CLIK
  /// cannot solve (QpFailuresAbortAndTheThirdLatches… uses the same, through
  /// the oracle).
  PlanSnapshot PlanFor(std::uint64_t generation, double t_c_s, bool degenerate_axis) {
    PlanSnapshot p{};
    const std::int64_t now = Now();
    p.valid = true;
    p.token.activation_generation = ctrl_->GetPlannerRtState().activation_generation;
    p.token.generation = generation;
    p.plan_id = next_plan_id_++;
    const Eigen::Vector3d pc = NearPc();
    const Eigen::Vector3d ad = degenerate_axis ? Eigen::Vector3d::Zero() : StartAxis();
    p.p_c = {pc.x(), pc.y(), pc.z()};
    p.a_d = {ad.x(), ad.y(), ad.z()};
    p.publish_ns = now;
    p.gamma_t0_ns = now;
    p.t_c_ns = now + static_cast<std::int64_t>(t_c_s * 1e9);
    p.t_cmd_ns = p.t_c_ns;
    p.gamma_t1_ns = p.t_c_ns;
    return p;
  }

  /// From ARMED on the current ball: a trial that the solver ends AFTER a run
  /// of good solves — a good plan, `good_ticks` law ticks, then a replacement
  /// plan (§4.7) with an axis CLIK cannot solve. Ends on the ABORT_SAFE entry.
  void MidTrialClikFailure(int good_ticks = 20) {
    ASSERT_TRUE(TickUntilMode(Mode::kTracking, 200)) << Transitions();
    ctrl_->PlanBoxForTesting().Store(PlanFor(generation_, 2.0, false));
    ASSERT_TRUE(TickUntilMode(Mode::kApproach, 5)) << Transitions();
    const std::size_t approach = log_.size() - 1;
    Ticks(good_ticks);
    ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();
    ASSERT_GT(CountTicks([](const TickRec& r) { return r.body.clik_converged; }, approach),
              good_ticks / 2)
        << "precondition: the trial was not solving before the failure";
    ctrl_->PlanBoxForTesting().Store(PlanFor(generation_, 2.0, true));
    ASSERT_TRUE(TickUntilMode(Mode::kAbortSafe, 5)) << Transitions();
    ASSERT_EQ(ctrl_->GetLastReason(), Reason::kQpFailed) << Transitions();
  }

  /// In RETREAT: tick until the RETURN moves — the carried speed RISING (the
  /// stop only brings a residual speed down), as AnAbortDuringTheReturn… does.
  bool TickUntilTheReturnMoves(int max_ticks) {
    double prev = std::numeric_limits<double>::infinity();
    return TickUntil(
        [this, &prev] {
          const double s = Speed(log_.back());
          const bool rising = s > prev;
          prev = s;
          return ctrl_->GetMode() == Mode::kRetreat && rising;
        },
        max_ticks);
  }

  /// AnAbortSafeRampPastTheStopDeadlineFaultsAndStillStops, shared by both
  /// clocks.
  void StopDeadlineCase() {
    ASSERT_NO_FATAL_FAILURE(FaultByTheStopDeadline());
    ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
               Mode::kFault});
    const int f = Entry(Mode::kFault);
    ASSERT_GT(f, 0);
    const auto fu = static_cast<std::size_t>(f);
    EXPECT_EQ(log_[fu].reason, Reason::kAbortEscalated);
    EXPECT_EQ(log_[fu].body.fault_cause, FaultCause::kStopDeadline);
    EXPECT_TRUE(log_[fu].body.fault_latched);
    EXPECT_GT(Speed(log_[fu - 1]), 0.0)
        << "precondition: the ramp had not finished at the deadline";
    // FAULT does not drop the stop the deadline interrupted: it finishes it.
    Ticks(100);
    ExpectRampedToRestFrom(fu, "FAULT after a late ramp");
    EXPECT_EQ(ctrl_->GetMode(), Mode::kFault);
    EXPECT_TRUE(ctrl_->HasLatchedFault());
    EXPECT_FALSE(ctrl_->IsArmRequested()) << "a latched fault must disarm";
    EXPECT_EQ(log_.back().body.fault_cause, FaultCause::kStopDeadline) << "the cause is a state";
  }

  /// FAULT by the stop deadline, from an ABORT_SAFE whose ramp needs more than
  /// the one tick a sub-tick deadline allows. Leaves the log at the FAULT entry.
  void FaultByTheStopDeadline() {
    ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(3.0), StartAxis(), 0.0, 1.0, Deadlines(0.001, 5.0)));
    ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
    const std::vector<double> qdd = DerivedQddMax();
    ASSERT_TRUE(TickUntil(
        [this, &qdd] {
          const auto& v = ctrl_->GetArmVelocityCommandForTesting();
          for (int j = 0; j < kUr5eArmDof; ++j) {
            const auto u = static_cast<std::size_t>(j);
            if (std::abs(v[u]) > 2.0 * qdd[u] * kDt) {
              return true;
            }
          }
          return false;
        },
        300))
        << "precondition: the approach never got fast enough to need a two-tick stop\n"
        << Transitions();
    ASSERT_EQ(ctrl_->GetMode(), Mode::kApproach) << Transitions();
    state_.devices[0].positions[1] += 0.6;  // one tick of TRACK_ERR (see ATrackErrAbort…)
    ASSERT_TRUE(TickUntilMode(Mode::kFault, 20)) << Transitions();
  }

  /// From `from` on, the carried command only decelerates, inside the D-16
  /// box (no one-tick velocity step), and ends at rest, holding still.
  void ExpectRampedToRestFrom(std::size_t from, const char* what) const {
    ASSERT_GT(from, 0U);
    const std::vector<double> qdd = DerivedQddMax();
    for (std::size_t i = from; i < log_.size(); ++i) {
      for (int j = 0; j < kUr5eArmDof; ++j) {
        const auto u = static_cast<std::size_t>(j);
        const double dv = log_[i].qd_cmd[u] - log_[i - 1].qd_cmd[u];
        ASSERT_LE(std::abs(dv), qdd[u] * kDt + 1e-9)
            << what << ": tick " << i << " joint " << j << " steps its velocity by " << dv << '\n'
            << Window(static_cast<int>(i), 2);
        ASSERT_LE(std::abs(log_[i].qd_cmd[u]), std::abs(log_[i - 1].qd_cmd[u]) + 1e-12)
            << what << ": tick " << i << " joint " << j << " speeds up\n"
            << Window(static_cast<int>(i), 2);
      }
    }
    const int rest = First([](const TickRec& r) { return Speed(r) == 0.0; }, from);
    ASSERT_GE(rest, 0) << what << ": the command never came to rest\n" << Transitions();
    for (std::size_t i = static_cast<std::size_t>(rest); i < log_.size(); ++i) {
      for (int j = 0; j < kUr5eArmDof; ++j) {
        const auto u = static_cast<std::size_t>(j);
        ASSERT_EQ(log_[i].q_cmd[u], log_[static_cast<std::size_t>(rest)].q_cmd[u])
            << what << ": tick " << i << " joint " << j << " moves after the stop";
      }
    }
  }

  std::uint32_t next_plan_id_{1};
};

// ── D-S9-D1: the motion deadlines ───────────────────────────────────────────

TEST_F(FaultExtensionTest, AnAbortSafeRampPastTheStopDeadlineFaultsAndStillStops) {
  StopDeadlineCase();
}

/// A deadline case on the real steady clock (file header): the fault latch's
/// timing through the production clock path.
class FaultExtensionRealClockTest : public FaultExtensionTest {
 protected:
  FaultExtensionRealClockTest() { real_clock_ = true; }
};

TEST_F(FaultExtensionRealClockTest, AnAbortSafeRampPastTheStopDeadlineFaultsAndStillStops) {
  StopDeadlineCase();
}

TEST_F(FaultExtensionTest, ARetreatStopWhoseServoNeverCatchesUpFaultsAtTheStopDeadline) {
  // AServoStillLagging… recovers inside the default deadline. This servo never
  // does: RETREAT waits in its stop for a catch-up that does not come, and the
  // stop deadline ends the wait in FAULT — without starting the return, which
  // would drive an arm that is not following.
  constexpr double kStopS = 0.3;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 1.0, Deadlines(kStopS, 5.0)));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 200)) << Transitions();
  Ticks(20);
  pre_tick_ = [this] { state_.devices[0].positions[1] += 0.6; };
  ASSERT_TRUE(TickUntilMode(Mode::kFault, 800)) << Transitions();
  pre_tick_ = nullptr;
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kFault});
  const int r = Entry(Mode::kRetreat);
  const int f = Entry(Mode::kFault);
  ASSERT_GT(r, 0);
  ASSERT_GT(f, r);
  const auto ru = static_cast<std::size_t>(r);
  const auto fu = static_cast<std::size_t>(f);
  EXPECT_EQ(log_[fu].reason, Reason::kAbortEscalated);
  EXPECT_EQ(log_[fu].body.fault_cause, FaultCause::kStopDeadline);
  // On time: the RETREAT entry tick's clock read is inside [before, after] of
  // that tick, the FAULT tick's inside its own.
  const double waited = static_cast<double>(log_[fu].after_ns - log_[ru].before_ns) * 1e-9;
  EXPECT_GE(waited, kStopS) << Window(f, 2);
  EXPECT_LT(waited, kStopS + 0.1) << Window(f, 2);
  // No return was started: the command stood still through the whole RETREAT.
  for (std::size_t i = ru; i < fu; ++i) {
    ASSERT_EQ(Speed(log_[i]), 0.0) << "RETREAT moved an arm that never caught up\n"
                                   << Window(static_cast<int>(i), 2);
  }
}

TEST_F(FaultExtensionTest, AReturnPastItsDeadlineFaultsAndTheReturnStops) {
  // A caught ball's return (NearPc(2.0): a few tenths of a second) against a
  // 50 ms return deadline. A latch during RETREAT stops the return: FAULT
  // ramps the command to rest where it is, short of the wait pose.
  constexpr double kReturnS = 0.05;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 0.6, Deadlines(2.0, kReturnS)));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilMode(Mode::kFault, 400)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kCommitted,
             Mode::kClosing, Mode::kDecel, Mode::kHold, Mode::kRetreat, Mode::kFault});
  const int f = Entry(Mode::kFault);
  const auto fu = static_cast<std::size_t>(f);
  EXPECT_EQ(log_[fu].reason, Reason::kAbortEscalated);
  EXPECT_EQ(log_[fu].body.fault_cause, FaultCause::kReturnDeadline);
  EXPECT_GT(Speed(log_[fu - 1]), 0.0) << "precondition: the return was under way";
  Ticks(100);
  ExpectRampedToRestFrom(fu, "FAULT during the return");
  EXPECT_FALSE(ArmMeasuredAtWait(log_.back())) << "the return went on after the latch";
  EXPECT_FALSE(ctrl_->IsArmRequested());
}

TEST_F(FaultExtensionTest, AnEstopPairBetweenTwoTicksDoesNotTimeTheStopOnTheReturnsClock) {
  // A trigger→clear pair that lands between two ticks is seen as one epoch
  // move: the reset puts RETREAT back to its stop stage while the mode stays
  // RETREAT. That stop must be timed from the reset, not from when the return
  // began — else a stop deadline shorter than the return so far reads it as
  // overdue on its first tick (2026-09-27 /code-review).
  // stop_s 0.15: above RETREAT's own stop here (the HOLD leaves ~0.1 rad/s on
  // the command, ~0.05 s to ramp out at the fixture's 2.03 rad/s² box), below
  // how far into the return the pair lands.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 0.6, Deadlines(0.15, 5.0)));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilTheReturnMoves(300)) << Transitions();
  const std::int64_t return_seen = log_.back().before_ns;
  Ticks(85);
  ASSERT_GT(log_.back().after_ns - return_seen, 150 * kMsNs) << "precondition: past stop_s";
  ASSERT_EQ(ctrl_->GetMode(), Mode::kRetreat) << "precondition: still returning\n" << Transitions();
  ctrl_->TriggerEstop();
  ctrl_->ClearEstop();
  const std::size_t pair = log_.size();
  Ticks(50);
  EXPECT_EQ(CountTicks([](const TickRec& r) { return r.mode == Mode::kFault; }, pair), 0)
      << "a false stop-deadline FAULT\n"
      << Window(static_cast<int>(pair), 3);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle) << "the pair disarms (P-1 (c))\n" << Transitions();
  EXPECT_FALSE(ctrl_->IsArmRequested());
}

// ── D-S9-K: an arm the controller cannot read ───────────────────────────────

TEST_F(FaultExtensionTest, AnArmUnreadableDuringTheReturnIsStoppedAndFaultsAtTheDeadline) {
  constexpr double kReturnS = 0.3;
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 0.6, Deadlines(2.0, kReturnS)));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilTheReturnMoves(300)) << Transitions();
  // Slot 0 never written: the gate closes (a hole, not a stale value).
  state_.devices[0].hole_mask = 1;
  const std::size_t cut = log_.size();
  ASSERT_TRUE(TickUntilMode(Mode::kFault, 400)) << Transitions();
  const int f = Entry(Mode::kFault, 0, cut);
  ASSERT_GT(f, static_cast<int>(cut));
  const auto fu = static_cast<std::size_t>(f);
  EXPECT_EQ(log_[fu - 1].mode, Mode::kRetreat);
  EXPECT_EQ(log_[fu].reason, Reason::kAbortEscalated);
  EXPECT_EQ(log_[fu].body.fault_cause, FaultCause::kReturnDeadline);
  Ticks(20);
  // From the cut on: ramped to rest inside the box and held — no return motion
  // onto an arm nobody can see, in RETREAT or after it.
  ExpectRampedToRestFrom(cut, "RETREAT with the arm unreadable");
  const int rest = First([](const TickRec& r) { return Speed(r) == 0.0; }, cut);
  EXPECT_LT(rest, f) << "the command was still moving when the deadline fired";
}

TEST_F(FaultExtensionTest, AnArmReadableAgainInsideTheDeadlineResumesTheReturn) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(2.0), StartAxis(), 0.0, 0.6, Deadlines(2.0, 3.0)));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  ASSERT_TRUE(TickUntilTheReturnMoves(300)) << Transitions();
  state_.devices[0].hole_mask = 1;
  const std::size_t cut = log_.size();
  Ticks(60);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kRetreat) << Transitions();
  const std::size_t back = log_.size();
  ExpectRampedToRestFrom(cut, "RETREAT with the arm unreadable");
  EXPECT_EQ(Speed(log_[back - 1]), 0.0) << "held at rest while unreadable";
  EXPECT_FALSE(ArmMeasuredAtWait(log_[back - 1])) << "precondition: the return was not over";
  state_.devices[0].hole_mask = 0;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
  EXPECT_GT(CountTicks([](const TickRec& r) { return Speed(r) > 0.0; }, back), 0)
      << "the return did not resume";
  EXPECT_TRUE(ArmMeasuredAtWait(log_.back()));
  EXPECT_EQ(CountTicks([](const TickRec& r) { return r.mode == Mode::kFault; }), 0)
      << Transitions();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
}

// ── D-S9-D2: n_qp counts trials ─────────────────────────────────────────────

TEST_F(FaultExtensionTest, ThreeTrialsInARowThatTheSolverEndsMidTrialLatchAFault) {
  // Each trial solves for a while first. A per-SOLVE streak (the old count)
  // was reset by those good solves and never reached n_qp = 3 here.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, OracleOff));
  for (int k = 0; k < 3; ++k) {
    ASSERT_NO_FATAL_FAILURE(MidTrialClikFailure()) << "trial " << k;
    EXPECT_EQ(log_.back().body.qp_fail_streak, k + 1) << "trial " << k;
    if (k < 2) {
      ++generation_;  // the thrower retries with a new ball (Q15)
      ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();
    }
  }
  ASSERT_TRUE(TickUntilMode(Mode::kFault, 5)) << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kRetreat, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kFault});
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kAbortEscalated);
  EXPECT_EQ(log_.back().body.fault_cause, FaultCause::kQpFailures);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
}

TEST_F(FaultExtensionTest, OnlyAVerdictOrAFaultResetEndsTheRunOfSolverEndedTrials) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, OracleOff));
  const auto streak = [this] { return log_.back().body.qp_fail_streak; };

  // 1 — the solver ends it: 1.
  ASSERT_NO_FATAL_FAILURE(MidTrialClikFailure());
  EXPECT_EQ(streak(), 1);
  ++generation_;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();

  // 2 — another reason ends it (a new ball, TRACK_CHANGED): unchanged.
  ASSERT_TRUE(TickUntilMode(Mode::kTracking, 200)) << Transitions();
  ctrl_->PlanBoxForTesting().Store(PlanFor(generation_, 2.0, false));
  ASSERT_TRUE(TickUntilMode(Mode::kApproach, 5)) << Transitions();
  Ticks(10);
  ++generation_;
  PublishNow();
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 5)) << Transitions();
  EXPECT_EQ(ctrl_->GetLastReason(), Reason::kTrackChanged);
  EXPECT_EQ(streak(), 1) << "an abort the solver did not cause moved the count";
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();

  // An E-STOP and its clear: unchanged (the stop says nothing about the solver).
  ctrl_->TriggerEstop();
  Ticks(5);
  EXPECT_EQ(streak(), 1) << "the E-STOP reset cleared the count";
  ctrl_->ClearEstop();
  Ticks(5);
  EXPECT_EQ(streak(), 1) << "the clear cleared the count";
  // The E-STOP reset forgot the last trial's ball (R-TRACK memory), so the new
  // ball must be in the box BEFORE the re-arm, or TRACKING starts on the old one.
  ++generation_;
  PublishNow();
  SetArmed(true);

  // 3 — the solver again: 2.
  ASSERT_NO_FATAL_FAILURE(MidTrialClikFailure());
  EXPECT_EQ(streak(), 2);
  ++generation_;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();

  // 4 — a trial that reaches HOLD's verdict (Undetermined: no fingertips): 0.
  ASSERT_TRUE(TickUntilMode(Mode::kTracking, 200)) << Transitions();
  ctrl_->PlanBoxForTesting().Store(PlanFor(generation_, 0.6, false));
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  EXPECT_EQ(log_[log_.size() - 2].mode, Mode::kHold) << "precondition: the trial reached HOLD";
  EXPECT_EQ(ctrl_->GetOutcomeForTesting(), Outcome::kUndetermined);
  EXPECT_EQ(streak(), 0) << "a verdict did not end the run";
  ++generation_;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 1500)) << Transitions();

  // 5 — the solver: 1, not 3 — the verdict broke the run, so no fault.
  ASSERT_NO_FATAL_FAILURE(MidTrialClikFailure());
  EXPECT_EQ(streak(), 1);
  Ticks(5);
  EXPECT_FALSE(ctrl_->HasLatchedFault()) << Transitions();
  EXPECT_EQ(CountTicks([](const TickRec& r) { return r.mode == Mode::kFault; }), 0);
}

// ── D-S9-D3: no fault reset while the arm moves ─────────────────────────────

TEST_F(FaultExtensionTest, AFaultResetIsRefusedUntilTheArmIsAtRest) {
  ASSERT_NO_FATAL_FAILURE(FaultByTheStopDeadline());
  const auto reset_once = [this] {
    ctrl_->ResetFault();
    const std::size_t t = log_.size();
    Ticks(1);
    return t;
  };

  // (1) The command is still ramping to rest.
  ASSERT_GT(Speed(log_.back()), 0.0) << "precondition: FAULT entered with the ramp unfinished";
  std::size_t t = reset_once();
  EXPECT_EQ(log_[t].body.fault_reset_refused, FaultResetRefusal::kCommandMoving)
      << Window(static_cast<int>(t), 1);
  EXPECT_EQ(log_[t].mode, Mode::kFault);
  EXPECT_NE(log_[t].reason, Reason::kFaultReset);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  EXPECT_EQ(ctrl_->GetFaultResetRefusedCount(), 1U);
  Ticks(1);
  EXPECT_EQ(log_.back().body.fault_reset_refused, FaultResetRefusal::kNone)
      << "a refusal is its own tick's, and it is not queued";
  EXPECT_TRUE(ctrl_->HasLatchedFault()) << "a refused reset was acted on later";
  ASSERT_TRUE(TickUntil([this] { return Speed(log_.back()) == 0.0; }, 500)) << Transitions();

  // (2) The command is at rest, the measured arm is not.
  state_.devices[0].velocities[2] = 0.05;  // > homing.qd_tol (0.02)
  t = reset_once();
  EXPECT_EQ(log_[t].body.fault_reset_refused, FaultResetRefusal::kArmMoving);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  state_.devices[0].velocities[2] = 0.0;

  // (3) The velocity lane is not vouched for: fail-closed.
  state_.devices[0].velocity_hole_mask = 1;
  t = reset_once();
  EXPECT_EQ(log_[t].body.fault_reset_refused, FaultResetRefusal::kVelocityUnreadable);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  state_.devices[0].velocity_hole_mask = 0;
  EXPECT_EQ(ctrl_->GetFaultResetRefusedCount(), 3U);

  // (4) At rest: IDLE, disarmed, the cause gone with the latch.
  t = reset_once();
  Ticks(2);
  EXPECT_EQ(log_[t].body.fault_reset_refused, FaultResetRefusal::kNone);
  EXPECT_EQ(log_[t].mode, Mode::kIdle) << Window(static_cast<int>(t), 2);
  EXPECT_EQ(log_[t].reason, Reason::kFaultReset);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(log_.back().body.fault_cause, FaultCause::kNone);
  EXPECT_FALSE(ctrl_->IsArmRequested()) << "no automatic resume (P-1 (c))";
  EXPECT_EQ(ctrl_->GetFaultResetRefusedCount(), 3U);
}

// ── D-S9-L and a latch that reached IDLE ────────────────────────────────────

TEST_F(FaultExtensionTest, AFaultResetUnderAnEstopShowsFaultUntilTheClearThenIdle) {
  ASSERT_NO_FATAL_FAILURE(FaultByTheStopDeadline());
  ASSERT_TRUE(TickUntil([this] { return Speed(log_.back()) == 0.0; }, 500)) << Transitions();
  ctrl_->TriggerEstop();
  Ticks(3);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kFault) << "P-1 (d): the stop keeps FAULT";
  ctrl_->ResetFault();
  const std::size_t reset = log_.size();
  Ticks(10);
  EXPECT_FALSE(ctrl_->HasLatchedFault()) << "the latch goes down on the reset's own tick";
  EXPECT_FALSE(log_[reset].body.fault_latched);
  for (std::size_t i = reset; i < log_.size(); ++i) {
    ASSERT_EQ(log_[i].mode, Mode::kFault) << "ESTOP outranks the reset (R-PREC)\n"
                                          << Window(static_cast<int>(i), 2);
    ASSERT_EQ(log_[i].reason, Reason::kEstop);
  }
  ctrl_->ClearEstop();
  const std::size_t clear = log_.size();
  Ticks(3);
  EXPECT_EQ(log_[clear].mode, Mode::kIdle) << Window(static_cast<int>(clear), 2);
  EXPECT_EQ(log_[clear].reason, Reason::kFaultReset);
  EXPECT_FALSE(ctrl_->IsArmRequested());
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kFault, Mode::kIdle});
}

TEST_F(FaultExtensionTest, AResetBetweenTheLatchAndItsEscalationDoesNotResumeTrials) {
  // The n_qp latch rises inside the law tick, after that tick's reset service.
  // A reset that lands before the next tick clears it while the mode is still
  // ABORT_SAFE (no FAULT to leave). The controller must end disarmed, not
  // carry the abort on through RETREAT into ARMED and a new trial.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["a_d"] = YAML::Load("[0.0, 0.0, 0.0]");
    y["catching"]["supervisor"]["n_qp"] = 1;
  }));
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }, 600)) << Transitions();
  ASSERT_EQ(ctrl_->GetMode(), Mode::kAbortSafe) << "precondition: the latch tick";
  EXPECT_FALSE(ctrl_->IsArmRequested()) << "the latch tick did not disarm";
  ctrl_->ResetFault();
  const std::size_t reset = log_.size();
  Ticks(1);
  ASSERT_FALSE(ctrl_->HasLatchedFault()) << "precondition: the reset was taken in ABORT_SAFE";
  ASSERT_EQ(log_[reset].mode, Mode::kAbortSafe) << Window(static_cast<int>(reset), 2);
  ++generation_;  // a fresh ball is in view: nothing but the arm latch stops a new trial
  Ticks(400);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle) << Transitions();
  EXPECT_FALSE(ctrl_->IsArmRequested());
  EXPECT_EQ(CountTicks([](const TickRec& r) { return r.mode == Mode::kArmed; }, reset), 0)
      << "the abort resumed into a new trial after the reset\n"
      << Transitions();
}

TEST_F(FaultExtensionTest, AFaultLatchedIntoIdleByAnEstopIsResetInIdleAndArmsAgain) {
  // n_qp = 1: the first trial the solver ends latches the fault on the tick it
  // enters ABORT_SAFE. An E-STOP that lands before the next tick sends
  // ABORT_SAFE to IDLE with the latch still up — no FAULT mode to leave.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6, [](YAML::Node& y) {
    y["diagnostic"]["oracle_plan"]["a_d"] = YAML::Load("[0.0, 0.0, 0.0]");
    y["catching"]["supervisor"]["n_qp"] = 1;
  }));
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }, 600)) << Transitions();
  ASSERT_EQ(ctrl_->GetMode(), Mode::kAbortSafe) << "precondition: the latch tick";
  ctrl_->TriggerEstop();
  Ticks(3);
  ASSERT_EQ(ctrl_->GetMode(), Mode::kIdle) << Transitions();
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  ctrl_->ClearEstop();
  SetArmed(true);  // the operator tries: the latch disarms again on every tick
  Ticks(20);
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle);
  EXPECT_FALSE(ctrl_->IsArmRequested()) << "armed with a fault latched";
  ctrl_->ResetFault();
  const std::size_t t = log_.size();
  Ticks(3);
  EXPECT_EQ(log_[t].mode, Mode::kIdle);
  EXPECT_EQ(log_[t].reason, Reason::kFaultReset) << Window(static_cast<int>(t), 2);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_FALSE(ctrl_->IsArmRequested());
  SetArmed(true);
  EXPECT_TRUE(TickUntilMode(Mode::kArmed, 1500))
      << "the operator could not re-arm after the reset\n"
      << Transitions();
  ExpectSeq({Mode::kIdle, Mode::kArmed, Mode::kTracking, Mode::kApproach, Mode::kAbortSafe,
             Mode::kIdle, Mode::kArmed});
}

// ── pre-S10 R3 (#537 Q4 · Q5): what parks a real-arm configuration ──────────
//
// The park's observable beyond the verdict IS the configure log: the operator
// learns WHICH value is not cleared from that line and nowhere else (the park
// reason is one enum value for every consumed key). So the assertion goes to
// the sink the line reaches — same shape as test_gate_closure_diagnostic.cpp;
// the handler is process-global, which a single-TU binary can afford.
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

  /// Captured lines of `severity` containing every needle.
  static std::vector<std::string> Matching(int severity, const std::vector<std::string>& needles) {
    const std::lock_guard<std::mutex> lock(Mutex());
    std::vector<std::string> hits;
    for (const auto& [sev, text] : Lines()) {
      if (sev != severity) {
        continue;
      }
      const bool all = std::all_of(needles.begin(), needles.end(), [&text](const auto& needle) {
        return text.find(needle) != std::string::npos;
      });
      if (all) {
        hits.push_back(text);
      }
    }
    return hits;
  }

 private:
  static void Handler(const rcutils_log_location_t* /*location*/, int severity,
                      const char* /*name*/, rcutils_time_point_value_t /*timestamp*/,
                      const char* format, va_list* args) {
    // Value-initialised: on an encoding failure vsnprintf need not terminate.
    char buffer[2048]{};
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

class SafetyGateParkTest : public SupervisorScenarioTest {
 protected:
  void SetUp() override {
    SupervisorScenarioTest::SetUp();
    LogSink::Install();
  }

  void TearDown() override {
    LogSink::Restore();
    SupervisorScenarioTest::TearDown();
  }

  /// The fixture profile — every park key cleared — with `tweak` applied,
  /// configured on the real-arm axis (no backend declared) or the sim one.
  void Configure(const std::function<void(YAML::Node&)>& tweak, bool sim) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetClockForTesting(&FakeSteadyClock::Now);
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
    if (sim) {
      rtc::DeviceBackendBinding backend;
      backend.type = integrated_bringup::kCatchingSimBackendType;
      for (auto& [name, cfg] : configs) {
        cfg.backend = backend;
      }
    }
    ctrl_->SetDeviceNameConfigs(configs);
    YAML::Node yaml = YAML::Load(TrackingYaml(topic_, NearPc(), StartAxis(), 0.0, 0.6));
    if (tweak) {
      tweak(yaml);
    }
    LogSink::Clear();
    ASSERT_EQ(ctrl_->on_configure(prev_, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS)
        << "a park is a SUCCESSFUL configure: the robot must still come up";
    ASSERT_EQ(ctrl_->IsRealArmConfig(), !sim) << "precondition: the axis this case is judged on";
  }

  void ExpectParked(const std::vector<std::string>& needles) {
    EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
    EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kConsumedValues);
    EXPECT_FALSE(LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, needles).empty())
        << "the configure log must name the value that parked it";
    EXPECT_NE(ctrl_->on_activate(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
  }

  void ExpectActivates() {
    EXPECT_FALSE(ctrl_->IsSimOnlyDisabled())
        << "parked (reason " << static_cast<int>(ctrl_->GetParkReason()) << ")";
    EXPECT_TRUE(ctrl_->AreTrialsEnabled());
    ASSERT_EQ(ctrl_->on_activate(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
    EXPECT_EQ(ctrl_->on_deactivate(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
  }

  static std::function<void(YAML::Node&)> AccelBox(AccelBoxFlag flag) {
    return [flag](YAML::Node& y) {
      YAML::Node arm = y["catching"]["robot"]["arm"];
      switch (flag) {
        case AccelBoxFlag::kCleared:
          arm["qdd_provisional"] = false;
          break;
        case AccelBoxFlag::kProvisional:
          arm["qdd_provisional"] = true;
          break;
        case AccelBoxFlag::kAbsent:
          arm.remove("qdd_provisional");
          break;
      }
    };
  }

  static void ProvisionalLag(YAML::Node& y) {
    y["catching"]["joint_cmd"]["lag"]["provisional"] = true;
  }

  const rclcpp_lifecycle::State prev_;
};

TEST_F(SafetyGateParkTest, TheFixtureProfileWithEveryParkKeyClearedActivatesOnTheRealArm) {
  // Positive control for every park below: same fixture, nothing provisional.
  ASSERT_NO_FATAL_FAILURE(Configure(nullptr, /*sim=*/false));
  ExpectActivates();
}

TEST_F(SafetyGateParkTest, AReconfigureWithoutABoxDoesNotKeepTheLastOnes) {
  // #609: the box DATA was cleared per configure, its PATH was not — so a
  // profile that names no box silently ran on the one the last configure
  // named. No box is a supervisor park (nothing to ramp within).
  ASSERT_NO_FATAL_FAILURE(Configure(AccelBox(AccelBoxFlag::kCleared), /*sim=*/false));
  ASSERT_FALSE(ctrl_->IsSimOnlyDisabled()) << "precondition: the first profile activates";
  ASSERT_EQ(ctrl_->on_cleanup(prev_), DemoCatchingController::CallbackReturn::SUCCESS);

  YAML::Node yaml = YAML::Load(TrackingYaml(topic_, NearPc(), StartAxis(), 0.0, 0.6));
  yaml["catching"]["robot"]["arm"].remove("qdd_max");
  ASSERT_EQ(ctrl_->on_configure(prev_, node_, yaml),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl_->IsSimOnlyDisabled()) << "ran on the box of a profile it no longer has";
  EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kSupervisorUnset);
  EXPECT_NE(ctrl_->on_activate(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
}

// ── the acceleration box's keys: `robot.arm.qdd_max` · `qdd_provisional` ─────

TEST_F(SafetyGateParkTest, AProvisionalQddBoxParksTheRealArm) {
  // Q4: the box is reviewed but still provisional — the real arm's is not
  // identified. Every ramp and the CLIK box run on it. Same park, same reason
  // code, and the log names the key.
  ASSERT_NO_FATAL_FAILURE(Configure(AccelBox(AccelBoxFlag::kProvisional), /*sim=*/false));
  ExpectParked({"robot.arm.qdd_provisional"});
}

TEST_F(SafetyGateParkTest, AQddBoxWithoutTheFlagIsProvisional) {
  // Fail-closed: a profile that does not say is not a profile that was cleared.
  ASSERT_NO_FATAL_FAILURE(Configure(AccelBox(AccelBoxFlag::kAbsent), /*sim=*/false));
  ExpectParked({"robot.arm.qdd_provisional"});
}

TEST_F(SafetyGateParkTest, AQddProvisionalThatIsNotABoolIsProvisional) {
  ASSERT_NO_FATAL_FAILURE(
      Configure([](YAML::Node& y) { y["catching"]["robot"]["arm"]["qdd_provisional"] = "cleared"; },
                /*sim=*/false));
  ExpectParked({"robot.arm.qdd_provisional"});
}

TEST_F(SafetyGateParkTest, AProvisionalQddBoxOnlyWarnsInSim) {
  ASSERT_NO_FATAL_FAILURE(Configure(AccelBox(AccelBoxFlag::kProvisional), /*sim=*/true));
  EXPECT_FALSE(LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {"robot.arm.qdd_provisional"}).empty());
  ExpectActivates();
}

TEST_F(SafetyGateParkTest, AClearedQddBoxIsTheValueTheProfileNames) {
  // The positive control of the three below: the box that loads is the one the
  // fixture wrote, so a parked case is parked by ITS defect and not by a box
  // that never loaded.
  ASSERT_NO_FATAL_FAILURE(Configure(AccelBox(AccelBoxFlag::kCleared), /*sim=*/false));
  ExpectActivates();
  EXPECT_FALSE(LogSink::Matching(RCUTILS_LOG_SEVERITY_INFO, {"acceleration box"}).empty());
}

struct BadQddBox {
  const char* name;
  std::function<void(YAML::Node&)> tweak;
};

TEST_F(SafetyGateParkTest, AMissingShortOrNonPositiveQddBoxParksAsNoBox) {
  // A bad box is not a configure failure but an empty box: the same
  // supervisor park as a profile with no box at all (nothing to ramp within).
  const std::vector<BadQddBox> cases = {
      {"absent", [](YAML::Node& y) { y["catching"]["robot"]["arm"].remove("qdd_max"); }},
      {"short",
       [](YAML::Node& y) {
         YAML::Node box(YAML::NodeType::Sequence);
         for (int i = 0; i < kUr5eArmDof - 1; ++i) {
           box.push_back(2.0);
         }
         y["catching"]["robot"]["arm"]["qdd_max"] = box;
       }},
      {"zero entry", [](YAML::Node& y) { y["catching"]["robot"]["arm"]["qdd_max"][2] = 0.0; }},
      {"negative entry", [](YAML::Node& y) { y["catching"]["robot"]["arm"]["qdd_max"][0] = -1.0; }},
      {"not a sequence", [](YAML::Node& y) { y["catching"]["robot"]["arm"]["qdd_max"] = 2.0; }},
      {"non-numeric entry",
       [](YAML::Node& y) { y["catching"]["robot"]["arm"]["qdd_max"][1] = "fast"; }},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    ASSERT_NO_FATAL_FAILURE(Configure(c.tweak, /*sim=*/false));
    EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
    EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kSupervisorUnset);
    EXPECT_NE(ctrl_->on_activate(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl_->on_cleanup(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
  }
}

TEST_F(SafetyGateParkTest, ARemovedAccelLimitsKeyParksNamingTheNewKeys) {
  // An old overlay that set one of the three would otherwise be ignored and the
  // run would silently fall back to the shipped box. Not a configure FAILURE
  // (CM would then refuse every controller on the robot): the controller parks
  // with its own reason, in sim and on a real arm alike, and the ERROR names
  // the removed key and the keys that replace it.
  for (const bool sim : {false, true}) {
    for (const char* key : {"accel_limits_path", "accel_limits_package", "accel_limits_group"}) {
      SCOPED_TRACE(std::string(key) + (sim ? " (sim)" : " (real arm)"));
      ASSERT_NO_FATAL_FAILURE(Configure(
          [key](YAML::Node& y) {
            y["catching"]["robot"]["arm"][key] = "config/ur5e_p1b/derived_accel_limits.yaml";
          },
          sim));
      EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
      EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kRemovedKey);
      EXPECT_FALSE(
          LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR,
                            {std::string("catching.robot.arm.") + key, "catching.robot.arm.qdd_max",
                             "catching.robot.arm.qdd_provisional"})
              .empty())
          << "the ERROR must name the removed key and both keys to use";
      EXPECT_EQ(ctrl_->on_activate(prev_), DemoCatchingController::CallbackReturn::FAILURE);
      ASSERT_EQ(ctrl_->on_cleanup(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
    }
    // Positive control: the same profile without the key is not parked for it.
    SCOPED_TRACE(sim ? "no key (sim)" : "no key (real arm)");
    ASSERT_NO_FATAL_FAILURE(Configure(AccelBox(AccelBoxFlag::kCleared), sim));
    EXPECT_NE(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kRemovedKey);
    ExpectActivates();
  }
}

// ── #712: the CLIK's acceleration form has no default, and `box` is gone ─────
//
// Both cases carry `eta_tau`, the dynamic form's key, as every shipped profile
// does. A profile that lost only its form line (or had it set to `box`) used
// to be REFUSED at parse for that key — a configure FAILURE, so every
// controller on the robot down — where these ask for a park.

TEST_F(SafetyGateParkTest, AProfileThatNamesNoClikFormParksNamingTheKey) {
  // The key has no default: no form is run on a profile's behalf. It parks the
  // way any consumed value left open does, on both axes.
  for (const bool sim : {false, true}) {
    SCOPED_TRACE(sim ? "sim" : "real arm");
    ASSERT_NO_FATAL_FAILURE(Configure(
        [](YAML::Node& y) {
          YAML::Node joint_cmd = y["catching"]["joint_cmd"];
          ASSERT_TRUE(joint_cmd.remove("accel_constraint"))
              << "precondition: the fixture names a form";
          joint_cmd["eta_tau"] = 0.8;
        },
        sim));
    ExpectParked({"joint_cmd.accel_constraint"});
    ASSERT_EQ(ctrl_->on_cleanup(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
    // Positive control: the same profile with its form line is not parked.
    ASSERT_NO_FATAL_FAILURE(
        Configure([](YAML::Node& y) { y["catching"]["joint_cmd"]["eta_tau"] = 0.8; }, sim));
    ExpectActivates();
  }
}

TEST_F(SafetyGateParkTest, TheRemovedBoxFormParksNamingTheValueAndTheFormsLeft) {
  // An old overlay that selects `box` must not read as a profile that merely
  // left the key out: it parks under the removed-key reason, and the ERROR
  // names the value and what to write instead.
  for (const bool sim : {false, true}) {
    SCOPED_TRACE(sim ? "sim" : "real arm");
    ASSERT_NO_FATAL_FAILURE(Configure(
        [](YAML::Node& y) {
          y["catching"]["joint_cmd"]["accel_constraint"] = "box";
          y["catching"]["joint_cmd"]["eta_tau"] = 0.8;
        },
        sim));
    EXPECT_TRUE(ctrl_->IsSimOnlyDisabled());
    EXPECT_EQ(ctrl_->GetParkReason(), integrated_bringup::CatchingParkReason::kRemovedKey);
    EXPECT_FALSE(
        LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR,
                          {"catching.joint_cmd.accel_constraint: box", "kinematic or dynamic"})
            .empty())
        << "the ERROR must name the removed value and the forms that are left";
    EXPECT_EQ(ctrl_->on_activate(prev_), DemoCatchingController::CallbackReturn::FAILURE);
    ASSERT_EQ(ctrl_->on_cleanup(prev_), DemoCatchingController::CallbackReturn::SUCCESS);
  }
}

TEST_F(SafetyGateParkTest, AProvisionalArmLagParksTheRealArm) {
  // Q5: T_arm is not identified on the real arm. Parked whatever its value and
  // with the lead compensation off, as the fixture has it.
  ASSERT_NO_FATAL_FAILURE(Configure(ProvisionalLag, /*sim=*/false));
  ExpectParked({"joint_cmd.lag"});
}

TEST_F(SafetyGateParkTest, AProvisionalArmLagOnlyWarnsInSim) {
  ASSERT_NO_FATAL_FAILURE(Configure(ProvisionalLag, /*sim=*/true));
  EXPECT_FALSE(LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {"joint_cmd.lag"}).empty());
  ExpectActivates();
}

// ── pre-S10 R3 (#537 Q9 · Q16): a velocity nobody vouches for ───────────────
//
// The position lane stays readable in every case here: what is missing is the
// velocity lane's word, and the slots then hold whatever was there (zeros in
// this fixture — exactly the reading that would pass every "at rest" check).

TEST_F(SupervisorScenarioTest, AnArmWhoseVelocityIsUnreadableIsNotAtTheWaitPose) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  state_.devices[0].velocity_hole_mask = 1;
  Ticks(20);
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.mode != Mode::kIdle; }), 0)
      << "armed on a velocity lane with a hole\n"
      << Transitions();
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_NEAR(log_.back().q_cmd[u], kUr5eHome[u], 1e-12)
        << "joint " << i << ": the arm must keep being held at the wait pose";
    EXPECT_EQ(log_.back().qd_cmd[u], 0.0) << "joint " << i;
  }
  // Positive control: the same arm, vouched for, arms.
  state_.devices[0].velocity_hole_mask = 0;
  EXPECT_TRUE(TickUntilMode(Mode::kArmed, 20)) << Transitions();
}

TEST_F(SupervisorScenarioTest, AHandWhoseVelocityIsUnreadableIsNotSettled) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  state_.devices[1].velocity_hole_mask = 1ULL << 3;
  Ticks(20);
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.mode != Mode::kIdle; }), 0)
      << "armed on a hand whose rest nobody vouches for\n"
      << Transitions();
  state_.devices[1].velocity_hole_mask = 0;
  EXPECT_TRUE(TickUntilMode(Mode::kArmed, 20)) << Transitions();
}

// #606: the same question on the two paths that read the hand sequencer's
// `at_target` instead of HandSettledAtPre — the re-arm out of RETREAT and the
// contact baseline.

TEST_F(SupervisorScenarioTest, AHandWhoseVelocityIsUnreadableDoesNotReArmAfterATrial) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  ASSERT_TRUE(TickUntilMode(Mode::kHold, 1500)) << Transitions();
  // The hole opens while the hand holds the ball. The close is over, so what
  // is left to judge with a velocity is the release.
  state_.devices[1].velocity_hole_mask = 1ULL << 3;
  ASSERT_TRUE(TickUntilMode(Mode::kRetreat, 1500)) << Transitions();
  const std::size_t retreat = log_.size();
  // The way out is the release timeout (D-S8-6 (a)), into a disarmed IDLE —
  // never the re-arm. The plant has long put the hand at q_pre by then.
  ASSERT_TRUE(TickUntil(
      [this] { return !log_.empty() && log_.back().reason == Reason::kHandTimeout; }, 3000))
      << "RETREAT neither re-armed nor timed out\n"
      << Transitions();
  EXPECT_EQ(CountTicks([](const TickRec& t) { return t.mode == Mode::kArmed; }, retreat), 0)
      << "re-armed on a hand whose rest nobody vouches for\n"
      << Transitions();
  EXPECT_EQ(CountTicks(
                [](const TickRec& t) {
                  return t.mode == Mode::kRetreat && t.hand_active &&
                         t.phase == HandPhase::kPreshape;
                },
                retreat),
            0)
      << "the sequencer called the hand settled at q_pre\n"
      << Transitions();
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle) << Transitions();
}

TEST_F(SupervisorScenarioTest, AHandWhoseVelocityIsUnreadableLearnsNoContactBaseline) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  tips_enabled_ = true;
  ball_in_hand_ = true;
  publishing_ = false;
  ASSERT_TRUE(TickUntilMode(Mode::kArmed, 20)) << Transitions();
  // Armed on a readable hand (it could not have armed otherwise); the hole
  // opens on the next tick and stays through the wait.
  const int before = ctrl_->GetTipBaselineCountForTesting();
  state_.devices[1].velocity_hole_mask = 1ULL << 3;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  // One sample at most: the contact lane runs before the hand stage and reads
  // LAST tick's `at_target`, so the first tick of the hole is still judged on
  // the tick before it — which was vouched for. 30 ticks would have learned 30.
  EXPECT_LE(ctrl_->GetTipBaselineCountForTesting(), before + 1)
      << "a baseline was learned from a hand whose rest nobody vouches for";
  // Positive control: the same wait, vouched for, learns it — counted from
  // where the holed wait left off, on the fingertip that learned LEAST.
  const int holed = ctrl_->GetTipBaselineCountForTesting();
  state_.devices[1].velocity_hole_mask = 0;
  ASSERT_NO_FATAL_FAILURE(LearnBaselineInArmed());
  EXPECT_GE(ctrl_->GetTipBaselineMinCountForTesting() - holed, 20);
}

TEST_F(SupervisorScenarioTest, AnUnreadableArmVelocityDefersTheAdoptionUntilItIsReadable) {
  // Unarmed, like a moving arm: a hole that lasts a tick must not cost the one
  // decision an activation gets.
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[1] += 0.05;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off,
      /*arm=*/false));
  publishing_ = false;
  state_.devices[0].velocity_hole_mask = 1;
  Ticks(5);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting()) << "adopted a pose at an unknown velocity";
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 0U) << "deferring is not refusing";
  state_.devices[0].velocity_hole_mask = 0;
  Ticks(1);
  ASSERT_TRUE(ctrl_->IsWaitPoseAdoptedForTesting());
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                off[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i;
  }
}

TEST_F(SupervisorScenarioTest, ArmingWhileTheArmVelocityIsUnreadableRefusesTheSwitchedInPose) {
  std::array<double, kUr5eArmDof> off = kUr5eHome;
  off[0] += 0.06;
  ASSERT_NO_FATAL_FAILURE(BringUp(
      NearPc(), StartAxis(), 0.0, 0.6,
      [](YAML::Node& y) { y["catching"]["planner"]["wait_pose_source"] = "current"; }, off));
  publishing_ = false;
  state_.devices[0].velocity_hole_mask = 1ULL << 2;
  Ticks(2);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting());
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_reason,
            integrated_bringup::CatchingDiagLogPod::WaitPoseRefusal::kVelocityUnreadable);
  EXPECT_EQ(ctrl_->GetLastTickRecord().wait_pose_refuse_joint, -1)
      << "the lane is refused, not a joint of it";
  state_.devices[0].velocity_hole_mask = 0;
  Ticks(3);
  EXPECT_FALSE(ctrl_->IsWaitPoseAdoptedForTesting()) << "one decision per activation";
  EXPECT_EQ(ctrl_->GetWaitPoseRefusedCount(), 1U);
  for (int i = 0; i < kUr5eArmDof; ++i) {
    EXPECT_NEAR(ctrl_->GetWaitPoseForTesting()[static_cast<std::size_t>(i)],
                kUr5eHome[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i << ": a refused pose must leave the YAML pose in force";
  }
}

// The operator's only sign of the above: nothing transitions, so the publish
// thread says it — once when the lane closes, once when it opens.
class VelocityLaneReportTest : public SupervisorScenarioTest {
 protected:
  void SetUp() override {
    SupervisorScenarioTest::SetUp();
    LogSink::Install();
  }

  void TearDown() override {
    LogSink::Restore();
    SupervisorScenarioTest::TearDown();
  }

  /// Ticks, then one pass of the publish thread's body.
  void TicksThenPublish(int n) {
    Ticks(n);
    ctrl_->PublishNonRtSnapshot(rtc::PublishSnapshot{});
  }

  static std::size_t Warned(const char* axis) {
    return LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN,
                             {std::string(axis) + " velocity lane UNREADABLE"})
        .size();
  }
};

TEST_F(VelocityLaneReportTest, AnUnreadableVelocityLaneIsWarnedOncePerAxisAndPerEpisode) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  LogSink::Clear();
  // Negative control: readable lanes say nothing.
  TicksThenPublish(3);
  EXPECT_EQ(Warned("arm"), 0U);
  EXPECT_EQ(Warned("hand"), 0U);

  state_.devices[0].velocity_hole_mask = 1ULL << 4;
  TicksThenPublish(2);
  TicksThenPublish(2);
  EXPECT_EQ(Warned("arm"), 1U) << "one WARN per episode, not per publish";
  EXPECT_EQ(Warned("hand"), 0U);
  EXPECT_FALSE(LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN,
                                 {"arm velocity lane UNREADABLE", "ARMED", "fault reset"})
                   .empty())
      << "the line must say what the hole blocks";

  state_.devices[0].velocity_hole_mask = 0;
  state_.devices[1].velocity_hole_mask = 1;
  TicksThenPublish(2);
  EXPECT_EQ(Warned("hand"), 1U);
  EXPECT_FALSE(
      LogSink::Matching(RCUTILS_LOG_SEVERITY_INFO, {"arm velocity lane readable again"}).empty());

  // A second episode on the arm is a second WARN.
  state_.devices[0].velocity_hole_mask = 1;
  TicksThenPublish(2);
  EXPECT_EQ(Warned("arm"), 2U);
}

TEST_F(VelocityLaneReportTest, AClosedPositionGateIsNotReportedAsAVelocityHole) {
  // The gate's own diagnostic owns that case; two lines for one cause would
  // send the operator after the wrong lane.
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  LogSink::Clear();
  state_.devices[0].valid = false;
  state_.devices[0].velocity_hole_mask = 1;
  TicksThenPublish(3);
  EXPECT_EQ(Warned("arm"), 0U);
}

// #610: the flag is "positions readable AND velocity lane holed", so a position
// gate that closes DURING an episode used to read as the episode ending.

TEST_F(VelocityLaneReportTest, AClosingPositionGateDoesNotEndAVelocityEpisode) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  LogSink::Clear();
  state_.devices[0].velocity_hole_mask = 1;
  TicksThenPublish(2);
  ASSERT_EQ(Warned("arm"), 1U);

  // The device drops out altogether; the velocity lane is as holed as it was.
  state_.devices[0].valid = false;
  TicksThenPublish(3);
  EXPECT_TRUE(
      LogSink::Matching(RCUTILS_LOG_SEVERITY_INFO, {"arm velocity lane readable again"}).empty())
      << "reported a recovery nobody observed";

  // Back, and still holed: the same episode, not a second WARN.
  state_.devices[0].valid = true;
  TicksThenPublish(3);
  EXPECT_EQ(Warned("arm"), 1U);

  // Positive control: the real recovery is reported.
  state_.devices[0].velocity_hole_mask = 0;
  TicksThenPublish(3);
  EXPECT_FALSE(
      LogSink::Matching(RCUTILS_LOG_SEVERITY_INFO, {"arm velocity lane readable again"}).empty());
}

TEST_F(VelocityLaneReportTest, ANewActivationWarnsAgainAboutAHoleThatIsStillThere) {
  ASSERT_NO_FATAL_FAILURE(BringUp(NearPc(), StartAxis(), 0.0, 0.6));
  publishing_ = false;
  LogSink::Clear();
  state_.devices[1].velocity_hole_mask = 1;
  TicksThenPublish(2);
  ASSERT_EQ(Warned("hand"), 1U);

  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_->on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  LogSink::Clear();
  TicksThenPublish(2);
  EXPECT_EQ(Warned("hand"), 1U) << "the operator of this activation was never told";
  EXPECT_TRUE(
      LogSink::Matching(RCUTILS_LOG_SEVERITY_INFO, {"hand velocity lane readable again"}).empty());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
