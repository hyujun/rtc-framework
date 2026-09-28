// ── The CM's two latch services against the catching controller (S9a) ──────
//
// WHAT IS UNDER TEST. /rtc_cm/clear_estop and /rtc_cm/reset_fault, called over
// the wire, on a real RtControllerNode whose ACTIVE controller is the real
// DemoCatchingController — and what the catching controller does with each.
// The CM's own suites (rtc_controller_manager/test/test_clear_estop_service,
// test_reset_fault_service) pin the replies against mocks whose hooks ARE the
// state; the catching suites (test_catching_supervisor_scenarios) pin the
// supervisor against direct hook calls. Neither shows the seam between them:
// that a CM-level clear or reset, answered "ok", leaves the catching
// controller in the state its P-1 contract promises. That seam is what the
// operator GUI (S9) will drive, so it is what this file pins:
//   - an E-STOP latched by the CM's own trigger path reaches the catching
//     controller, which goes to IDLE with reason Estop and is disarmed ON THE
//     TICK that sees it, while the CM substitutes its hold (Phase 2c);
//   - clear_estop refuses an empty acknowledgement and the refusal quotes the
//     latched reason in a form a caller can extract and echo back (the GUI's
//     two-step); the echoed clear propagates ClearEstop, and the controller
//     stays IDLE and disarmed — no automatic resume — until the operator
//     re-arms through `catching.enable`;
//   - clear_estop does not clear the catching fault latch, and says so;
//   - reset_fault refuses a wrong or missing name, clears the latch to IDLE
//     (reason FaultReset) without re-arming, does not clear the global E-STOP
//     and says so, and a FAULT is never promoted to a global E-STOP.
// Nothing here changes production code: these are today's behaviours (P-1
// contract, D-S9-A/B/C/E1) written down before the GUI builds on them.
//
// HOW THE CM IS STOOD UP. Not through on_configure: that needs a variant
// config tree, registry-built device backends and a robot description, none of
// which is what this file is about. Instead the same friend the CM's own
// service suites use (`rtc::ControllerLifecycleTestAccess`, redefined here —
// each gtest binary is its own program, so the redefinition is ODR-safe, and
// ARCH-4 keeps rtc_controller_manager/test/ out of reach) hosts a catching
// controller brought up the way test_catching_supervisor_scenarios brings it
// up, installs two position-holding stub backends, and runs the CM's REAL tick
// body (ControlLoop: device read → Compute → validation → E-STOP substitution
// → WriteCommand → tick counter) on a test thread. The services wait on that
// tick counter, so the replies are the ones a running CM gives.
//
// Why a test thread rather than StartRtLoop: the PeriodicRtThread adds the
// overrun watchdog, which on a loaded CI host would latch a second,
// unrelated E-STOP ("consecutive_overrun") into the middle of a case about
// E-STOP reasons. The tick body is the thing under test; its scheduler is not.
// The thread ticks at a QUARTER period on an absolute schedule for the reason
// test_reset_fault_service gives: the services' deadlines are measured in
// periods, and a stand-in slower than its own period would eat their margin.
//
// THE ONE INJECTED PRECONDITION — the catching FAULT. The only production
// cause of the fault latch is three consecutive CLIK-failure trials
// (QpFailuresAbortAndTheThirdLatchesAFaultThatResetClears), which needs a
// vision stream, the oracle plan and several full trial cycles — seconds of
// real time per case, and none of it about the services. So the latch is
// injected through the controller's test-only friend, in the least invasive
// form that still exercises the production edge: with the tick thread JOINED
// (so no tick is in flight), the probe puts the controller exactly where
// NoteLawVerdict leaves it on the third failure — mode ABORT_SAFE with
// `fault_latched_` raised — and the NEXT real tick takes the table's own
// ABORT_SAFE → FAULT edge on kAbortEscalated. Everything after that (the
// disarm, the reset, the E-STOP interplay) is production code on production
// ticks.
//
// Wall time: under a second for the suite (measured ~0.45 s); the model build
// is shared by every case.

#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "ur5e_p1b_test_fixture.hpp"
#include <rtc_msgs/srv/clear_estop.hpp>
#include <rtc_msgs/srv/reset_fault.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace rtc {

// Friend bridge — the name is fixed by the friend declaration in
// rt_controller_node.hpp. Only what this file needs: host one controller with
// its device slots, run one tick, and read the E-STOP latch and counters.
class ControllerLifecycleTestAccess {
 public:
  // Mirrors what the CM's bring-up leaves behind for one controller: the
  // topic config the controller parsed (its group order IS the device order
  // Compute() indexes), a slot mapping over that order, and the name / key
  // lookup the services resolve through.
  static void HostController(RtControllerNode& node, std::unique_ptr<RTControllerInterface> ctrl,
                             const std::string& config_key) {
    const TopicConfig tc = ctrl->GetTopicConfig();
    RtControllerNode::ControllerSlotMapping mapping;
    mapping.num_groups = static_cast<int>(tc.groups.size());
    for (int i = 0; i < mapping.num_groups; ++i) {
      mapping.slots[static_cast<std::size_t>(i)] = i;
    }
    const std::string name(ctrl->Name());
    node.controllers_.clear();
    node.controllers_.push_back(std::move(ctrl));
    node.controller_states_ = std::vector<std::atomic<int>>(1);
    node.controller_topic_configs_ = {tc};
    node.controller_slot_mappings_ = {mapping};
    node.controller_types_ = {config_key};
    node.controller_name_to_idx_.clear();
    node.controller_name_to_idx_[name] = 0;
    node.controller_name_to_idx_[config_key] = 0;
  }

  static void InjectBackend(RtControllerNode& node, std::size_t slot,
                            std::unique_ptr<DeviceBackend> backend) {
    node.backends_[slot] = std::move(backend);
  }

  // What on_configure does once the backends exist: ask each one which
  // command types it honours. Without it every slot's mask is empty and the
  // tick would reject — and eventually E-STOP on — every output.
  static void CacheCommandTypeMasks(RtControllerNode& node) { node.CacheSlotCommandTypeMasks(); }

  // What ActivateController leaves behind for the CM (the controller's own
  // on_activate is driven by the fixture, as in the catching suites).
  static void MarkActive(RtControllerNode& node, int idx) {
    node.active_controller_idx_.store(idx, std::memory_order_release);
    node.controller_states_[static_cast<std::size_t>(idx)].store(1, std::memory_order_release);
  }

  static void BringServicesOnline(RtControllerNode& node) {
    if (!node.cb_group_nrt_callback_) {
      node.cb_group_nrt_callback_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    node.CreateServices();
  }

  // One full CM tick — the production body, including the tick counter the
  // services wait on.
  static void Tick(RtControllerNode& node) { node.ControlLoop(); }

  // The CM's own trigger path: latch, reason, and propagation to every
  // controller's TriggerEstop / SetHandEstop.
  static void TriggerGlobalEstop(RtControllerNode& node, std::string_view reason) {
    node.TriggerGlobalEstop(reason);
  }

  static bool IsEstopped(const RtControllerNode& node) { return node.IsGlobalEstopped(); }

  static std::uint64_t RtTickCount(const RtControllerNode& node) { return node.RtTickCount(); }

  static std::uint64_t EstopSubstituted(const RtControllerNode& node) {
    return node.EstopSubstitutedOutputCount();
  }

  static std::uint64_t RejectedOutputs(const RtControllerNode& node) {
    return node.RejectedOutputCount();
  }

  static std::uint64_t RejectedCommandTypes(const RtControllerNode& node) {
    return node.RejectedCommandTypeCount();
  }

  static std::chrono::microseconds ControlPeriod(const RtControllerNode& node) {
    return node.ControlPeriod();
  }
};

}  // namespace rtc

namespace integrated_bringup {

/// The controller's test-only friend (declared in its header; the reset-table
/// suite defines its own in its own binary). Here it does ONE thing: put the
/// controller where the third CLIK failure leaves it — see the file header.
/// Call only with no tick in flight.
class DemoCatchingControllerResetProbe {
 public:
  static void LeaveAsAfterThirdClikFailure(DemoCatchingController& c) {
    c.mode_ = rtc::catching::Mode::kAbortSafe;
    c.fault_latched_.store(true, std::memory_order_release);
  }
};

}  // namespace integrated_bringup

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::DemoCatchingControllerResetProbe;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::SharedCatchFrameBuilder;
using integrated_bringup::testfx::TrackingYaml;
using rtc::catching::Mode;
using rtc::catching::Reason;
using Access = rtc::ControllerLifecycleTestAccess;

using namespace std::chrono_literals;

// A quote INSIDE the reason on purpose: that is where "the text between the
// last two quotes" and the GUI's prefix rule disagree, so it keeps the parse
// below honest about which contract it pins.
constexpr const char* kEstopReason = "s9a 'operator' stop";
constexpr const char* kConfigKey = "demo_catching_controller";

// A device that has always just reported the same pose: the arm at the
// fixture's home (which the profile also names as the wait pose, so arming
// takes the Q13 skip), the hand at q_pre. Commands are counted, not obeyed —
// the cases are about latches, not about motion.
class HeldPoseBackend : public rtc::DeviceBackend {
 public:
  HeldPoseBackend(const double* q, int n) : n_(n) { std::copy(q, q + n, q_.begin()); }

  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/,
                 const rtc::DeviceBackendConfig& /*config*/,
                 rclcpp::CallbackGroup::SharedPtr /*group*/) override {}

  void Activate() override {}

  void Deactivate() override {}

  bool ReadState(rtc::DeviceStateCache& cache) noexcept override {
    cache.num_channels = n_;
    std::copy(q_.begin(), q_.begin() + n_, cache.positions.begin());
    cache.valid = true;
    return true;
  }

  void WriteCommand(const rtc::PublishSnapshot::GroupCommandSlot& /*slot*/,
                    rtc::CommandType /*ct*/) noexcept override {
    writes_.fetch_add(1, std::memory_order_relaxed);
  }

  void WriteSafeCommand() noexcept override {
    safe_writes_.fetch_add(1, std::memory_order_relaxed);
  }

  std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return std::chrono::steady_clock::now();
  }

 private:
  std::array<double, rtc::kMaxDeviceChannels> q_{};
  int n_{0};
  std::atomic<std::uint64_t> writes_{0};
  std::atomic<std::uint64_t> safe_writes_{0};
};

/// What one CM tick left behind, read on the tick thread right after it.
struct TickObs {
  Mode mode{Mode::kIdle};
  Reason reason{Reason::kNone};
  bool armed{false};             // the controller's arm latch, NOT the parameter
  bool fault_latched{false};     // the catching fault latch
  bool ctrl_estopped{false};     // the controller's own E-STOP request flag
  bool cm_estopped{false};       // the CM's global latch
  std::uint64_t estop_ticks{0};  // controller ticks run with the E-STOP raised
};

std::string Describe(const TickObs& o) {
  std::ostringstream os;
  os << "mode=" << static_cast<int>(o.mode) << " reason=" << static_cast<int>(o.reason)
     << " armed=" << o.armed << " latched=" << o.fault_latched << " ctrl_estop=" << o.ctrl_estopped
     << " cm_estop=" << o.cm_estopped << " estop_ticks=" << o.estop_ticks;
  return os.str();
}

/// The latched reason, lifted out of clear_estop's empty-ack refusal by the
/// SAME rule the operator GUI uses (demo_gui/latch_clear.py,
/// parse_estop_reason): the refusal is this fixed prefix, the reason, and a
/// closing quote with nothing after it. Empty when the message is not that
/// refusal. Kept identical on purpose — this seam test is only worth having if
/// it pins the contract the GUI actually relies on, so a reworded refusal
/// breaks here as well as in the GUI's own tests.
std::string ParseEstopReason(const std::string& msg) {
  static const std::string kPrefix =
      "reason_ack is required \u2014 echo the latched reason to confirm: '";
  if (msg.size() <= kPrefix.size() || msg.compare(0, kPrefix.size(), kPrefix) != 0 ||
      msg.back() != '\'') {
    return {};
  }
  return msg.substr(kPrefix.size(), msg.size() - kPrefix.size() - 1);
}

class CatchingCmServicesTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    builder_ = SharedCatchFrameBuilder();
  }

  static void TearDownTestSuite() {
    builder_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    // ── The catching controller, as the scenario suite brings it up ──
    ctrl_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_cm_services_ctrl");
    auto ctrl = std::make_unique<DemoCatchingController>("");
    ctrl->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl->SetSharedModelBuilder(builder_);
    const auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
    ctrl->SetDeviceNameConfigs(configs);
    // A catch point beside the start pose; no ball is ever published, so it
    // only has to be a value the profile validates, not one a trial reaches.
    integrated_bringup::testfx::CatchFrameOracle oracle(*builder_);
    std::array<double, 64> home{};
    std::copy(kUr5eHome.begin(), kUr5eHome.end(), home.begin());
    const auto start = oracle.PoseAt(configs.at("ur5e").joint_state_names, home, kUr5eArmDof);
    const Eigen::Vector3d p_c = start.translation() + Eigen::Vector3d(0.03, 0.02, 0.02);
    const YAML::Node yaml = YAML::Load(TrackingYaml("/test_catching_cm_services/prediction", p_c,
                                                    start.rotation().col(2), 0.0, 0.6));
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl->on_configure(prev, ctrl_node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_FALSE(ctrl->IsSimOnlyDisabled()) << "precondition: the profile parked (reason "
                                            << static_cast<int>(ctrl->GetParkReason()) << ")";
    ASSERT_EQ(ctrl->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_TRUE(ctrl->AreTrialsEnabled()) << "precondition: the S7 supervisor is not wired";
    ctrl_ = ctrl.get();

    // ── The CM hosting it ──
    node_ = std::make_shared<RtControllerNode>("test_catching_cm_services_node");
    Access::HostController(*node_, std::move(ctrl), kConfigKey);
    const std::array<double, kP1bHandDof> hand_q_pre{};
    Access::InjectBackend(*node_, 0,
                          std::make_unique<HeldPoseBackend>(kUr5eHome.data(), kUr5eArmDof));
    Access::InjectBackend(*node_, 1,
                          std::make_unique<HeldPoseBackend>(hand_q_pre.data(), kP1bHandDof));
    Access::CacheCommandTypeMasks(*node_);
    Access::MarkActive(*node_, 0);
    Access::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_catching_cm_services_client");
    clear_client_ = client_node_->create_client<rtc_msgs::srv::ClearEstop>("/rtc_cm/clear_estop");
    reset_client_ = client_node_->create_client<rtc_msgs::srv::ResetFault>("/rtc_cm/reset_fault");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    ASSERT_TRUE(clear_client_->wait_for_service(2s)) << "clear_estop did not appear";
    ASSERT_TRUE(reset_client_->wait_for_service(2s)) << "reset_fault did not appear";

    tick_period_ = Access::ControlPeriod(*node_) / 4;
    history_.reserve(1 << 16);
    StartTicking();
    ASSERT_TRUE(WaitTicks(10)) << "the CM tick never ran";
  }

  void TearDown() override {
    StopTicking();
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (executor_) {
      executor_->remove_node(client_node_);
      executor_->remove_node(node_->get_node_base_interface());
    }
    if (node_) {
      // Fixture sanity, checked last: a controller output the CM rejected
      // would have been held — and escalated to its own E-STOP — for a reason
      // no case here is about.
      EXPECT_EQ(Access::RejectedOutputs(*node_), 0U);
      EXPECT_EQ(Access::RejectedCommandTypes(*node_), 0U);
    }
    executor_.reset();
    node_.reset();  // owns — and destroys — the catching controller
    ctrl_ = nullptr;
    ctrl_node_.reset();
  }

  // ── The stand-in RT thread ────────────────────────────────────────────────

  void StartTicking() {
    ticking_.store(true, std::memory_order_release);
    tick_thread_ = std::thread([this]() {
      auto next = std::chrono::steady_clock::now();
      while (ticking_.load(std::memory_order_acquire)) {
        Access::Tick(*node_);
        TickObs o;
        o.mode = ctrl_->GetMode();
        o.reason = ctrl_->GetLastReason();
        o.armed = ctrl_->IsArmRequested();
        o.fault_latched = ctrl_->HasLatchedFault();
        o.ctrl_estopped = ctrl_->IsEstopped();
        o.cm_estopped = Access::IsEstopped(*node_);
        o.estop_ticks = ctrl_->GetEstopTickCount();
        {
          std::lock_guard<std::mutex> lock(history_mutex_);
          history_.push_back(o);
        }
        next += tick_period_;
        std::this_thread::sleep_until(next);
      }
    });
  }

  void StopTicking() {
    ticking_.store(false, std::memory_order_release);
    if (tick_thread_.joinable()) {
      tick_thread_.join();
    }
  }

  std::size_t HistorySize() {
    std::lock_guard<std::mutex> lock(history_mutex_);
    return history_.size();
  }

  std::vector<TickObs> HistoryFrom(std::size_t mark) {
    std::lock_guard<std::mutex> lock(history_mutex_);
    return {history_.begin() + static_cast<std::ptrdiff_t>(std::min(mark, history_.size())),
            history_.end()};
  }

  bool WaitFor(const std::function<bool()>& pred, std::chrono::milliseconds timeout = 3s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return pred();
  }

  bool WaitTicks(std::uint64_t n) {
    const std::uint64_t target = Access::RtTickCount(*node_) + n;
    return WaitFor([this, target] { return Access::RtTickCount(*node_) >= target; });
  }

  // ── Operator actions ──────────────────────────────────────────────────────

  void SetArmed(bool on) {
    ctrl_node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, on));
  }

  bool ArmParameter() {
    return ctrl_node_->get_parameter(integrated_bringup::kCatchingEnableParam).as_bool();
  }

  // The clear_estop deadline is tens of periods; the client's has to outlast
  // it or a slow reply would read as a hung service.
  rtc_msgs::srv::ClearEstop::Response::SharedPtr CallClear(const std::string& ack) {
    auto req = std::make_shared<rtc_msgs::srv::ClearEstop::Request>();
    req->reason_ack = ack;
    auto fut = clear_client_->async_send_request(req);
    if (fut.wait_for(10s) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  rtc_msgs::srv::ResetFault::Response::SharedPtr CallReset(const std::string& name) {
    auto req = std::make_shared<rtc_msgs::srv::ResetFault::Request>();
    req->controller_name = name;
    auto fut = reset_client_->async_send_request(req);
    if (fut.wait_for(10s) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  /// Arm through the operator channel and wait for the supervisor to take it.
  void ArmAndWaitForArmed() {
    SetArmed(true);
    ASSERT_TRUE(WaitFor([this] { return ctrl_->GetMode() == Mode::kArmed; }))
        << "precondition: the controller never armed (mode " << static_cast<int>(ctrl_->GetMode())
        << ")";
  }

  /// The injected precondition (file header): the third CLIK failure's state,
  /// then the production ABORT_SAFE → FAULT edge on a real tick.
  void LatchCatchingFault() {
    StopTicking();  // joined: no tick is in flight while the probe writes
    const std::size_t mark = HistorySize();
    DemoCatchingControllerResetProbe::LeaveAsAfterThirdClikFailure(*ctrl_);
    StartTicking();
    ASSERT_TRUE(WaitFor([this] { return ctrl_->GetMode() == Mode::kFault; }))
        << "the latched fault did not escalate to FAULT (mode "
        << static_cast<int>(ctrl_->GetMode()) << ")";
    ASSERT_TRUE(ctrl_->HasLatchedFault());
    // Read off the EDGE tick, not the getter: FAULT answers kNone on every
    // later tick (it holds), so the escalation reason is one tick wide.
    const auto after = HistoryFrom(mark);
    ASSERT_FALSE(after.empty());
    EXPECT_EQ(after.front().mode, Mode::kFault) << Describe(after.front());
    EXPECT_EQ(after.front().reason, Reason::kAbortEscalated)
        << "FAULT was reached by some edge other than the escalation: " << Describe(after.front());
  }

  static inline std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ctrl_node_;
  DemoCatchingController* ctrl_{nullptr};  // owned by node_
  std::shared_ptr<RtControllerNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<rtc_msgs::srv::ClearEstop>::SharedPtr clear_client_;
  rclcpp::Client<rtc_msgs::srv::ResetFault>::SharedPtr reset_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::thread tick_thread_;
  std::atomic<bool> ticking_{false};
  std::chrono::microseconds tick_period_{500};
  std::mutex history_mutex_;
  std::vector<TickObs> history_;
};

// ── clear_estop ─────────────────────────────────────────────────────────────

// The GUI's whole E-STOP recovery, end to end: the stop reaches the catching
// controller through the CM, the first clear call hands back the reason, the
// second clears with it — and the catching controller does not resume.
TEST_F(CatchingCmServicesTest, ClearEstopNeedsTheEchoedReasonAndDoesNotResumeCatching) {
  ASSERT_NO_FATAL_FAILURE(ArmAndWaitForArmed());
  ASSERT_TRUE(ctrl_->IsArmRequested());
  const std::uint64_t estop_ticks_before = ctrl_->GetEstopTickCount();

  // ── The stop, through the CM's own trigger path ──
  const std::size_t trig = HistorySize();
  Access::TriggerGlobalEstop(*node_, kEstopReason);
  // P-1 (a): the hook only raised a request — the flag is up at once, and
  // nothing else has moved until a tick acts on it.
  EXPECT_TRUE(ctrl_->IsEstopped()) << "the CM did not propagate TriggerEstop";
  ASSERT_TRUE(WaitFor(
      [this, estop_ticks_before] { return ctrl_->GetEstopTickCount() > estop_ticks_before; }));
  ASSERT_TRUE(WaitTicks(20));

  // The FIRST tick that ran with the stop raised already shows IDLE / Estop /
  // disarmed — the stop and the disarm land on that tick, not later (P-1 (b),
  // D-S9-A). Every tick after it, while the latch is up, says the same.
  const auto during = HistoryFrom(trig);
  const auto first = std::find_if(during.begin(), during.end(), [&](const TickObs& o) {
    return o.estop_ticks > estop_ticks_before;
  });
  ASSERT_NE(first, during.end());
  for (auto it = first; it != during.end(); ++it) {
    ASSERT_EQ(it->mode, Mode::kIdle) << Describe(*it);
    ASSERT_EQ(it->reason, Reason::kEstop) << Describe(*it);
    ASSERT_FALSE(it->armed) << "a stopped controller is still armed: " << Describe(*it);
  }
  // And the CM, not the controller, is what the actuators obey meanwhile.
  EXPECT_GT(Access::EstopSubstituted(*node_), 0U) << "Phase 2c never substituted the hold";

  // ── First call: no acknowledgement → refused, and told what to echo ──
  const auto refused = CallClear("");
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->ok) << refused->message;
  // The GUI's two-step depends on this being EXTRACTABLE, not just present.
  const std::string echoed = ParseEstopReason(refused->message);
  EXPECT_EQ(echoed, kEstopReason) << "reason not extractable from: " << refused->message;
  EXPECT_TRUE(Access::IsEstopped(*node_)) << "a refused clear lowered the latch";
  EXPECT_TRUE(ctrl_->IsEstopped()) << "a refused clear reached the controller";

  // ── Second call: the echoed reason clears ──
  const auto cleared = CallClear(echoed);
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->ok) << cleared->message;
  EXPECT_NE(cleared->message.find(kEstopReason), std::string::npos) << cleared->message;
  EXPECT_EQ(cleared->message.find("latched fault"), std::string::npos)
      << "the reply warned about a fault that is not latched: " << cleared->message;
  EXPECT_FALSE(Access::IsEstopped(*node_));
  EXPECT_FALSE(ctrl_->IsEstopped()) << "the CM did not propagate ClearEstop";

  // ── No automatic resume (P-1 (c), D-S9-B) ──
  const std::size_t after_clear = HistorySize();
  const std::uint64_t substituted = Access::EstopSubstituted(*node_);
  ASSERT_TRUE(WaitTicks(100));
  for (const TickObs& o : HistoryFrom(after_clear)) {
    ASSERT_FALSE(o.cm_estopped) << Describe(o);
    ASSERT_EQ(o.mode, Mode::kIdle) << "the clear resumed the supervisor: " << Describe(o);
    ASSERT_FALSE(o.armed) << "the clear re-armed the controller: " << Describe(o);
  }
  EXPECT_EQ(Access::EstopSubstituted(*node_), substituted)
      << "the CM kept substituting after the clear";
  // The tick lowered the arm LATCH, not the parameter: the two disagree now,
  // which is exactly the "disarmed out from under the operator" state the
  // header documents for IsArmRequested().
  EXPECT_TRUE(ArmParameter());
  EXPECT_FALSE(ctrl_->IsArmRequested());

  // ── Only a deliberate re-arm resumes ──
  ASSERT_NO_FATAL_FAILURE(ArmAndWaitForArmed());
}

// P-1 (d) seen from the CM: clearing the global latch must not launder the
// catching controller's own, and the operator has to be told the arm is still
// held for another reason.
TEST_F(CatchingCmServicesTest, ClearEstopLeavesTheCatchingFaultLatchedAndSaysSo) {
  ASSERT_NO_FATAL_FAILURE(LatchCatchingFault());
  Access::TriggerGlobalEstop(*node_, kEstopReason);
  ASSERT_TRUE(WaitTicks(20));
  ASSERT_TRUE(ctrl_->HasLatchedFault()) << "the E-STOP cleared the fault latch";

  const auto resp = CallClear(kEstopReason);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->ok) << resp->message;
  EXPECT_NE(resp->message.find("still has a latched fault"), std::string::npos)
      << "the reply did not mention the catching fault still latched: " << resp->message;
  EXPECT_NE(resp->message.find(std::string(ctrl_->Name())), std::string::npos) << resp->message;
  EXPECT_FALSE(Access::IsEstopped(*node_));

  const std::size_t mark = HistorySize();
  ASSERT_TRUE(WaitTicks(50));
  EXPECT_TRUE(ctrl_->HasLatchedFault()) << "clear_estop cleared the controller fault latch";
  for (const TickObs& o : HistoryFrom(mark)) {
    ASSERT_TRUE(o.fault_latched) << Describe(o);
    ASSERT_EQ(o.mode, Mode::kFault)
        << "the E-STOP clear moved the supervisor out of FAULT: " << Describe(o);
    ASSERT_FALSE(o.armed) << Describe(o);
  }
}

// ── reset_fault ─────────────────────────────────────────────────────────────

// The name is the operator confirmation (ResetFault.srv): a missing or wrong
// one is refused and the latch stays up across further ticks.
TEST_F(CatchingCmServicesTest, ResetFaultRefusesAWrongOrMissingNameAndKeepsTheLatch) {
  ASSERT_NO_FATAL_FAILURE(LatchCatchingFault());
  const std::string name(ctrl_->Name());

  const auto empty = CallReset("");
  ASSERT_NE(empty, nullptr);
  EXPECT_FALSE(empty->ok) << "an unnamed request cleared the fault";
  EXPECT_NE(empty->message.find("required"), std::string::npos) << empty->message;
  EXPECT_NE(empty->message.find(name), std::string::npos)
      << "the refusal did not name the active controller: " << empty->message;

  const auto wrong = CallReset("DemoJointController");
  ASSERT_NE(wrong, nullptr);
  EXPECT_FALSE(wrong->ok) << "a request naming another controller cleared the fault";
  EXPECT_NE(wrong->message.find("not the active controller"), std::string::npos) << wrong->message;

  const std::size_t mark = HistorySize();
  ASSERT_TRUE(WaitTicks(20));
  for (const TickObs& o : HistoryFrom(mark)) {
    ASSERT_TRUE(o.fault_latched) << "a refused request cleared the latch: " << Describe(o);
    ASSERT_EQ(o.mode, Mode::kFault) << Describe(o);
  }
}

// The happy path, and the two "no" rules around it: the reset does not re-arm
// (an operator who set `catching.enable` while faulted must not find the
// controller armed the moment the fault clears), and a FAULT never became a
// global E-STOP in the first place (D-S9-E1).
TEST_F(CatchingCmServicesTest, ResetFaultReturnsTheCatchingControllerToIdleDisarmed) {
  ASSERT_NO_FATAL_FAILURE(ArmAndWaitForArmed());
  ASSERT_NO_FATAL_FAILURE(LatchCatchingFault());
  EXPECT_FALSE(ctrl_->IsArmRequested()) << "the fault latch did not disarm";

  // The operator re-arms while faulted; the tick lowers it again.
  SetArmed(true);
  ASSERT_TRUE(WaitFor([this] { return !ctrl_->IsArmRequested(); }))
      << "a faulted controller accepted the arm request";
  ASSERT_TRUE(WaitTicks(20));

  // FAULT is controller-local: nothing escalated it to the CM's latch, so the
  // CM never substituted a hold for it.
  EXPECT_FALSE(Access::IsEstopped(*node_)) << "a catching FAULT raised the global E-STOP";
  EXPECT_EQ(Access::EstopSubstituted(*node_), 0U);

  const std::size_t mark = HistorySize();
  const auto resp = CallReset(std::string(ctrl_->Name()));
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->ok) << resp->message;
  EXPECT_NE(resp->message.find("fault latch cleared"), std::string::npos) << resp->message;
  EXPECT_EQ(resp->message.find("E-STOP"), std::string::npos)
      << "the reply warned about an E-STOP that is not latched: " << resp->message;
  EXPECT_FALSE(ctrl_->HasLatchedFault());

  ASSERT_TRUE(WaitTicks(50));
  const auto after = HistoryFrom(mark);
  // The tick that cleared the latch is the tick that left FAULT, on FaultReset
  // — the one tick that reason is visible; every later tick reports why IDLE
  // is staying IDLE (not armed) instead.
  const auto edge =
      std::find_if(after.begin(), after.end(), [](const TickObs& o) { return !o.fault_latched; });
  ASSERT_NE(edge, after.end());
  EXPECT_EQ(edge->mode, Mode::kIdle) << Describe(*edge);
  EXPECT_EQ(edge->reason, Reason::kFaultReset) << Describe(*edge);
  for (auto it = edge; it != after.end(); ++it) {
    ASSERT_FALSE(it->fault_latched) << Describe(*it);
    ASSERT_EQ(it->mode, Mode::kIdle) << "the reset resumed the supervisor: " << Describe(*it);
    ASSERT_FALSE(it->armed) << "the reset re-armed the controller: " << Describe(*it);
  }
  EXPECT_TRUE(ArmParameter()) << "the parameter the operator set is untouched";
  EXPECT_FALSE(Access::IsEstopped(*node_));
}

// The other half of E-8 separation: the fault reset goes through during a
// global E-STOP, clears only the controller's latch, and the reply says the
// arm is still stopped. The reset is not lost to the stop either — once the
// E-STOP clears, the controller leaves FAULT on FaultReset, still disarmed.
TEST_F(CatchingCmServicesTest, ResetFaultDuringAGlobalEstopClearsOnlyTheFaultLatch) {
  ASSERT_NO_FATAL_FAILURE(LatchCatchingFault());
  Access::TriggerGlobalEstop(*node_, kEstopReason);
  ASSERT_TRUE(WaitTicks(20));
  ASSERT_TRUE(ctrl_->HasLatchedFault());

  const auto resp = CallReset(std::string(ctrl_->Name()));
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->ok) << "a global E-STOP blocked the fault reset: " << resp->message;
  EXPECT_NE(resp->message.find("global E-STOP still latched"), std::string::npos)
      << "the reply did not say the arm is still stopped: " << resp->message;
  EXPECT_FALSE(ctrl_->HasLatchedFault()) << "the fault latch was not cleared";

  // The global latch — and the CM's hold — are untouched.
  const std::uint64_t substituted = Access::EstopSubstituted(*node_);
  ASSERT_TRUE(WaitTicks(20));
  EXPECT_TRUE(Access::IsEstopped(*node_)) << "reset_fault cleared the global E-STOP";
  EXPECT_TRUE(ctrl_->IsEstopped());
  EXPECT_GT(Access::EstopSubstituted(*node_), substituted) << "the CM stopped holding the arm";
  EXPECT_FALSE(ctrl_->IsArmRequested());

  // Now clear the stop: nothing is latched any more, so the reply has no
  // fault note, and the deferred FaultReset edge lands.
  const std::size_t clear_mark = HistorySize();
  const auto cleared = CallClear(kEstopReason);
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->ok) << cleared->message;
  EXPECT_EQ(cleared->message.find("latched fault"), std::string::npos) << cleared->message;
  ASSERT_TRUE(WaitTicks(50));
  const auto after = HistoryFrom(clear_mark);
  const auto edge = std::find_if(after.begin(), after.end(), [](const TickObs& o) {
    return o.mode == Mode::kIdle && o.reason == Reason::kFaultReset;
  });
  EXPECT_NE(edge, after.end()) << "the reset was lost to the E-STOP: FAULT never left on "
                                  "FaultReset after the clear";
  for (const TickObs& o : after) {
    ASSERT_FALSE(o.armed) << Describe(o);
    ASSERT_FALSE(o.fault_latched) << Describe(o);
  }
  EXPECT_EQ(ctrl_->GetMode(), Mode::kIdle);
}

}  // namespace
