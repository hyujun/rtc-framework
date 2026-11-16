// ── The CM's E-STOP surface over the two g1_loco controllers ─────────────────
//
// WHAT IS UNDER TEST. The operator stop and the post-clear policy the g1_loco
// sim ships (`estop_on_clear: initial_controller`), on a real RtControllerNode
// hosting the two controllers that profile instantiates — the shipped
// demo_joint_controller and the locomotion controller (its MPC off) — with the
// CM's own tick body on a stand-in RT thread and the services called over the
// wire. The CM's suites pin each piece against stubs; what only this seam
// shows is the sequence an operator sees on the humanoid: the locomotion
// controller is active, the stop holds every slot at the latched pose with no
// gains on the wire (so the simulator keeps the LAST gains it was sent), the
// verified clear switches back to demo_joint_controller, whose first tick puts
// the shipped scene-servo gains back on the wire, and the locomotion
// controller is left paused.
//
// MEASURED, NOT ASSUMED. Two numbers the design documents only as estimates:
// the verification window in ticks (one watchdog period plus the two-tick
// proof), and the ticks between the window closing and the post-clear switch
// landing, during which the PREVIOUS controller's output reaches the wire. The
// second is a known gap, bounded here.
//
// Not a claim about the locomotion controller's own behaviour under E-STOP or
// at re-entry (its gtests own that): its output is only classified by the
// gains it declares, so this file holds whatever that controller does.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "locomotion_controller_rig.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_msgs/srv/clear_estop.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {

// Friend bridge — the name is fixed by the friend declaration in
// rt_controller_node.hpp (each gtest binary is its own program, so this
// redefinition is ODR-safe).
class ControllerLifecycleTestAccess {
 public:
  // What the CM's bring-up leaves behind for several controllers over the
  // same device slots: each controller's topic config (its group order IS the
  // device order Compute() indexes), a slot mapping over that order with the
  // capability bits the backends declare, and the name / key lookup.
  static void HostControllers(RtControllerNode& node,
                              std::vector<std::unique_ptr<RTControllerInterface>> ctrls,
                              const std::vector<std::string>& config_keys,
                              const std::vector<std::uint16_t>& slot_caps) {
    node.controllers_.clear();
    node.controller_topic_configs_.clear();
    node.controller_slot_mappings_.clear();
    node.controller_types_.clear();
    node.controller_name_to_idx_.clear();
    node.controller_states_ = std::vector<std::atomic<int>>(ctrls.size());
    node.slot_to_capability_ = slot_caps;
    for (std::size_t i = 0; i < ctrls.size(); ++i) {
      const TopicConfig tc = ctrls[i]->GetTopicConfig();
      RtControllerNode::ControllerSlotMapping mapping;
      mapping.num_groups = static_cast<int>(tc.groups.size());
      for (int g = 0; g < mapping.num_groups; ++g) {
        const auto ug = static_cast<std::size_t>(g);
        mapping.slots[ug] = g;
        mapping.capabilities[ug] = slot_caps[ug];
      }
      node.controller_name_to_idx_[std::string(ctrls[i]->Name())] = static_cast<int>(i);
      node.controller_name_to_idx_[config_keys[i]] = static_cast<int>(i);
      node.controller_topic_configs_.push_back(tc);
      node.controller_slot_mappings_.push_back(mapping);
      node.controller_types_.push_back(config_keys[i]);
      node.controllers_.push_back(std::move(ctrls[i]));
    }
  }

  static void InjectBackend(RtControllerNode& node, std::size_t slot,
                            std::unique_ptr<DeviceBackend> backend) {
    node.backends_[slot] = std::move(backend);
  }

  static void CacheCommandTypeMasks(RtControllerNode& node) { node.CacheSlotCommandTypeMasks(); }

  static void MarkActive(RtControllerNode& node, int idx) {
    node.active_controller_idx_.store(idx, std::memory_order_release);
    node.controller_states_[static_cast<std::size_t>(idx)].store(1, std::memory_order_release);
  }

  static void BringServicesOnline(RtControllerNode& node) {
    if (!node.cb_group_nrt_callback_) {
      node.cb_group_nrt_callback_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    if (!node.active_ctrl_name_pub_) {
      node.active_ctrl_name_pub_ =
          node.create_publisher<std_msgs::msg::String>("/rtc_cm/active_controller_name", 1);
    }
    node.CreateFixedSafetyPublishers();
    node.CreateServices();
  }

  static void SetEstopOnClear(RtControllerNode& node, RtControllerNode::EstopOnClear policy) {
    node.estop_on_clear_ = policy;
  }

  static void SetInitialControllerIdx(RtControllerNode& node, int idx) {
    node.initial_controller_idx_ = idx;
  }

  static void Tick(RtControllerNode& node) { node.ControlLoop(); }

  static bool CallSwitch(RtControllerNode& node, const std::string& name, std::string& message) {
    return node.SwitchActiveController(name, message);
  }

  static int GetActiveIdx(const RtControllerNode& node) {
    return node.active_controller_idx_.load(std::memory_order_acquire);
  }

  static int GetControllerState(const RtControllerNode& node, std::size_t idx) {
    return node.controller_states_[idx].load(std::memory_order_acquire);
  }

  static bool IsEstopped(const RtControllerNode& node) { return node.IsGlobalEstopped(); }

  static bool IsHoldActive(const RtControllerNode& node) { return node.IsEstopHoldActive(); }

  static std::uint64_t VerifyWindowTicks(const RtControllerNode& node) {
    return node.EstopVerifyWindowTicks();
  }

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

  static std::string GlobalEstopReason(const RtControllerNode& node) {
    return node.GlobalEstopReason();
  }

  static constexpr std::string_view kOperatorEstopReason = RtControllerNode::kOperatorEstopReason;
  using EstopOnClear = RtControllerNode::EstopOnClear;
};

}  // namespace rtc

namespace {

namespace fx = integrated_bringup::testfx;
using integrated_bringup::DemoJointController;
using integrated_bringup::LocomotionController;
using Access = rtc::ControllerLifecycleTestAccess;
using namespace std::chrono_literals;

constexpr const char* kProfile = "g1_loco";
constexpr const char* kJointKey = "demo_joint_controller";
constexpr const char* kLocoKey = "locomotion_controller";
constexpr int kBody = 0;
constexpr int kHand = 1;

/// What one WriteCommand carried, copied out on the tick thread.
struct WireWrite {
  int num_channels{0};
  rtc::CommandType ct{rtc::CommandType::kPosition};
  bool has_pd_gains{false};
  std::array<double, rtc::kMaxDeviceChannels> commands{};
  std::array<double, rtc::kMaxDeviceChannels> feedforward{};
  std::array<double, rtc::kMaxDeviceChannels> pd_kp{};
  std::array<double, rtc::kMaxDeviceChannels> pd_kd{};
};

/// A device standing still at a fixed pose, every lane fresh at each read.
/// The body slot also carries the floating-base ground truth the locomotion
/// controller's state source needs.
class StandingBackend : public rtc::DeviceBackend {
 public:
  StandingBackend(std::vector<double> q, bool floating, double base_z)
      : q_(std::move(q)), floating_(floating), base_z_(base_z) {}

  void Configure(rclcpp_lifecycle::LifecycleNode*, const rtc::DeviceBackendConfig&,
                 rclcpp::CallbackGroup::SharedPtr) override {}

  void Activate() override {}

  void Deactivate() override {}

  bool ReadState(rtc::DeviceStateCache& cache) noexcept override {
    const auto now = loco_rig::NowNs();
    cache.num_channels = static_cast<int>(q_.size());
    std::copy(q_.begin(), q_.end(), cache.positions.begin());
    cache.hole_mask = 0;
    cache.valid = true;
    cache.joint_recv_steady_ns = now;
    cache.joint_sequence = ++seq_;
    return true;
  }

  bool HasFloatingBase() const noexcept override { return floating_; }

  void ReadBaseState(rtc::DeviceStateCache& cache) noexcept override {
    const auto now = loco_rig::NowNs();
    cache.base_pose = {0.0, 0.0, base_z_, 0.0, 0.0, 0.0, 1.0};
    cache.base_twist = {};
    cache.base_recv_steady_ns = now;
    cache.base_sequence = seq_;
    cache.base_valid = true;
  }

  bool AcceptsCommandType(rtc::CommandType ct) const noexcept override {
    return ct == rtc::CommandType::kPosition || ct == rtc::CommandType::kPdFeedforward;
  }

  void WriteCommand(const rtc::PublishSnapshot::GroupCommandSlot& slot,
                    rtc::CommandType ct) noexcept override {
    WireWrite w;
    w.num_channels = slot.num_channels;
    w.ct = ct;
    w.has_pd_gains = slot.has_pd_gains;
    const auto n =
        static_cast<std::size_t>(std::clamp(slot.num_channels, 0, rtc::kMaxDeviceChannels));
    std::copy_n(slot.commands.data(), n, w.commands.data());
    std::copy_n(slot.feedforward.data(), n, w.feedforward.data());
    std::copy_n(slot.pd_kp.data(), n, w.pd_kp.data());
    std::copy_n(slot.pd_kd.data(), n, w.pd_kd.data());
    last_ = w;  // tick thread only; read by the tick thread right after the tick
    writes_.fetch_add(1, std::memory_order_release);
  }

  void WriteSafeCommand() noexcept override {
    safe_writes_.fetch_add(1, std::memory_order_relaxed);
  }

  std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return std::chrono::steady_clock::now();
  }

  [[nodiscard]] const WireWrite& Last() const noexcept { return last_; }

  [[nodiscard]] int Writes() const noexcept { return writes_.load(std::memory_order_acquire); }

  [[nodiscard]] int SafeWrites() const noexcept {
    return safe_writes_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] const std::vector<double>& Pose() const noexcept { return q_; }

 private:
  std::vector<double> q_;
  bool floating_;
  double base_z_;
  std::uint64_t seq_{0};
  WireWrite last_{};
  std::atomic<int> writes_{0};
  std::atomic<int> safe_writes_{0};
};

/// Whose command the body slot carried on one tick, told by the gains it
/// declared: the hold declares none; the two controllers declare their own.
enum class Wire : std::uint8_t { kHold, kJoint, kLoco, kOther };

class LocoEstopOnClearTest : public ::testing::Test {
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

  void SetUp() override {
    // ── The two controllers, as the CM brings them up ──
    auto builder = fx::ShippedModelBuilder(kProfile);
    const auto model = fx::ShippedModelConfig(kProfile);

    joint_cfg_ = fx::ShippedControllerNode(kProfile, kJointKey);
    fx::MergeShippedShared(joint_cfg_, kProfile);
    joint_node_ = MakeNode("loco_estop_on_clear_joint");
    auto joint = std::make_unique<DemoJointController>("");
    joint->SetSystemModelConfig(model);
    joint->SetSharedModelBuilder(builder);
    joint->SetControlRate(fx::kShippedProfileControlRateHz);
    ASSERT_EQ(joint->PreConfigure(joint_node_, joint_cfg_),
              DemoJointController::CallbackReturn::SUCCESS);
    joint->SetDeviceNameConfigs(loco_rig::ShippedDevices(joint_cfg_));
    ASSERT_EQ(joint->on_configure(rclcpp_lifecycle::State{}, joint_node_, joint_cfg_),
              DemoJointController::CallbackReturn::SUCCESS);
    joint_kp_ = joint_cfg_["pd_feedforward"]["kp"].as<std::vector<double>>();
    joint_kd_ = joint_cfg_["pd_feedforward"]["kd"].as<std::vector<double>>();

    loco_cfg_ = loco_rig::ShippedConfig();  // MPC off
    loco_node_ = MakeNode("loco_estop_on_clear_loco");
    auto loco = std::make_unique<LocomotionController>("");
    loco->SetSystemModelConfig(model);
    loco->SetSharedModelBuilder(builder);
    loco->SetControlRate(fx::kShippedProfileControlRateHz);
    ASSERT_EQ(loco->PreConfigure(loco_node_, loco_cfg_),
              LocomotionController::CallbackReturn::SUCCESS);
    loco->SetDeviceNameConfigs(loco_rig::ShippedDevices(loco_cfg_));
    ASSERT_EQ(loco->on_configure(rclcpp_lifecycle::State{}, loco_node_, loco_cfg_),
              LocomotionController::CallbackReturn::SUCCESS)
        << loco->ConfigError();
    loco_kp_ = loco_cfg_["pd_feedforward"]["kp"].as<std::vector<double>>();
    ASSERT_NE(loco_kp_.at(0), joint_kp_.at(0)) << "the two controllers must be told apart by gain";

    // The stand pose and the pelvis height that puts both soles on the floor,
    // from the locomotion controller's own full model (the rig's MakeStanding).
    const std::vector<double> pose = loco_rig::StandPose();
    const pinocchio::Model& full = loco->FullModel();
    const auto& idx = loco->Indices();
    ASSERT_EQ(static_cast<int>(pose.size()), idx.body.num_channels);
    Eigen::VectorXd q = pinocchio::neutral(full);
    for (int c = 0; c < idx.body.num_channels; ++c) {
      q(7 + idx.body.joint[static_cast<std::size_t>(c)]) = pose[static_cast<std::size_t>(c)];
    }
    pinocchio::Data data(full);
    pinocchio::framesForwardKinematics(full, data, q);
    double z_min = std::numeric_limits<double>::infinity();
    for (const char* sole : {"left_sole", "right_sole"}) {
      z_min = std::min(z_min, data.oMf[full.getFrameId(sole)].translation().z());
    }
    const std::vector<double> hand_pose(static_cast<std::size_t>(idx.hands[0].num_channels), 0.0);

    // demo_joint is the initial controller: active before the loop runs.
    ASSERT_EQ(joint->on_activate(rclcpp_lifecycle::State{}),
              DemoJointController::CallbackReturn::SUCCESS);
    joint_ = joint.get();
    loco_ = loco.get();

    // ── The CM hosting both ──
    node_ = std::make_shared<RtControllerNode>("test_loco_estop_on_clear_node");
    std::vector<std::unique_ptr<rtc::RTControllerInterface>> ctrls;
    ctrls.push_back(std::move(joint));
    ctrls.push_back(std::move(loco));
    const auto joint_state = static_cast<std::uint16_t>(rtc::DeviceCapability::kJointState);
    const auto floating = static_cast<std::uint16_t>(rtc::DeviceCapability::kFloatingBase);
    Access::HostControllers(*node_, std::move(ctrls), {kJointKey, kLocoKey},
                            {static_cast<std::uint16_t>(joint_state | floating), joint_state});
    auto body = std::make_unique<StandingBackend>(pose, true, -z_min);
    auto hand = std::make_unique<StandingBackend>(hand_pose, false, 0.0);
    body_ = body.get();
    hand_ = hand.get();
    Access::InjectBackend(*node_, kBody, std::move(body));
    Access::InjectBackend(*node_, kHand, std::move(hand));
    Access::CacheCommandTypeMasks(*node_);
    Access::MarkActive(*node_, 0);
    Access::SetEstopOnClear(*node_, Access::EstopOnClear::kInitialController);
    Access::SetInitialControllerIdx(*node_, 0);
    Access::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_loco_estop_on_clear_client");
    trigger_ = client_node_->create_client<std_srvs::srv::Trigger>("/rtc_cm/trigger_estop");
    clear_ = client_node_->create_client<rtc_msgs::srv::ClearEstop>("/rtc_cm/clear_estop");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    ASSERT_TRUE(trigger_->wait_for_service(2s));
    ASSERT_TRUE(clear_->wait_for_service(2s));

    StartTicking();
    ASSERT_TRUE(WaitFor([this] { return Count(Wire::kJoint) >= 10; }, 3s))
        << "demo_joint_controller never reached the wire";
  }

  void TearDown() override {
    StopTicking();
    if (executor_) {
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      while (!executor_->is_spinning() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
      }
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
      // Fixture sanity: an output the CM rejected would have been held — and
      // escalated — for a reason no case here is about.
      EXPECT_EQ(Access::RejectedOutputs(*node_), 0U);
      EXPECT_EQ(Access::RejectedCommandTypes(*node_), 0U);
    }
    executor_.reset();
    node_.reset();  // owns — and destroys — both controllers
    joint_ = nullptr;
    loco_ = nullptr;
  }

  static rclcpp_lifecycle::LifecycleNode::SharedPtr MakeNode(const char* name) {
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, "", opts);
  }

  // ── The stand-in RT thread: the CM tick body at the control period ──
  void StartTicking() {
    ticking_.store(true, std::memory_order_release);
    tick_thread_ = std::thread([this]() {
      const auto period = Access::ControlPeriod(*node_);
      auto next = std::chrono::steady_clock::now();
      while (ticking_.load(std::memory_order_acquire)) {
        const int writes_before = body_->Writes();
        Access::Tick(*node_);
        const Wire body = body_->Writes() == writes_before ? Wire::kOther : Classify(body_->Last());
        const bool estopped = Access::IsEstopped(*node_);
        const bool hold = Access::IsHoldActive(*node_);  // latch OR window
        const int idx = Access::GetActiveIdx(*node_);
        counts_[static_cast<std::size_t>(body)].fetch_add(1, std::memory_order_relaxed);
        // A controller command on the wire while the hold was active after it.
        if (hold && (body == Wire::kLoco || body == Wire::kJoint)) {
          violations_.fetch_add(1, std::memory_order_relaxed);
        }
        if (clear_started_.load(std::memory_order_acquire)) {
          // The window: held with the latch already down.
          if (body == Wire::kHold && !estopped && hold) {
            window_held_.fetch_add(1, std::memory_order_relaxed);
          }
          // The gap: hold released, switch not yet landed.
          if (!hold && idx != 0) {
            release_to_switch_.fetch_add(1, std::memory_order_relaxed);
            if (body == Wire::kLoco) {
              loco_after_release_.fetch_add(1, std::memory_order_relaxed);
            }
          }
        }
        next += period;
        // A stall (a slow QP, a preempted thread) is not caught up on with a
        // burst of back-to-back ticks: that would compress the window into no
        // wall time and distort the tick counts measured below.
        const auto now = std::chrono::steady_clock::now();
        if (next < now - period) {
          next = now;
        }
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

  Wire Classify(const WireWrite& w) const {
    if (!w.has_pd_gains) {
      return Wire::kHold;
    }
    if (w.pd_kp[0] == joint_kp_[0]) {
      return Wire::kJoint;
    }
    if (w.pd_kp[0] == loco_kp_[0]) {
      return Wire::kLoco;
    }
    return Wire::kOther;
  }

  int Count(Wire w) { return counts_[static_cast<std::size_t>(w)].load(std::memory_order_relaxed); }

  static bool WaitFor(const std::function<bool()>& pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return pred();
  }

  std_srvs::srv::Trigger::Response::SharedPtr CallTrigger() {
    auto fut = trigger_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (fut.wait_for(10s) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  rtc_msgs::srv::ClearEstop::Response::SharedPtr CallClear(const std::string& ack,
                                                           std::chrono::milliseconds* elapsed) {
    auto req = std::make_shared<rtc_msgs::srv::ClearEstop::Request>();
    req->reason_ack = ack;
    const auto t0 = std::chrono::steady_clock::now();
    auto fut = clear_->async_send_request(req);
    if (fut.wait_for(10s) != std::future_status::ready) {
      return nullptr;
    }
    auto resp = fut.get();
    if (elapsed) {
      *elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0);
    }
    return resp;
  }

  YAML::Node joint_cfg_;
  YAML::Node loco_cfg_;
  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;
  std::vector<double> loco_kp_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr joint_node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr loco_node_;
  DemoJointController* joint_{nullptr};
  LocomotionController* loco_{nullptr};
  std::shared_ptr<RtControllerNode> node_;
  StandingBackend* body_{nullptr};
  StandingBackend* hand_{nullptr};
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_;
  rclcpp::Client<rtc_msgs::srv::ClearEstop>::SharedPtr clear_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread tick_thread_;
  std::atomic<bool> ticking_{false};
  // Per-tick tallies kept on the tick thread; the measured numbers are read
  // once the ticking has stopped.
  std::array<std::atomic<int>, 4> counts_{};
  std::atomic<bool> clear_started_{false};
  std::atomic<int> violations_{0};
  std::atomic<int> window_held_{0};
  std::atomic<int> release_to_switch_{0};
  std::atomic<int> loco_after_release_{0};
};

TEST_F(LocoEstopOnClearTest, StopUnderLocomotionHoldsThenTheClearHandsBackToDemoJoint) {
  // ── Operator switches to the locomotion controller ──
  std::string msg;
  ASSERT_TRUE(Access::CallSwitch(*node_, kLocoKey, msg)) << msg;
  ASSERT_TRUE(WaitFor([this] { return Count(Wire::kLoco) >= 5; }, 3s))
      << "the locomotion controller never reached the wire";
  ASSERT_EQ(Access::RejectedOutputs(*node_), 0U);
  ASSERT_EQ(Access::RejectedCommandTypes(*node_), 0U);

  // ── Operator stop ──
  const auto stop = CallTrigger();
  ASSERT_NE(stop, nullptr);
  ASSERT_TRUE(stop->success) << stop->message;
  ASSERT_TRUE(WaitFor([this] { return Access::EstopSubstituted(*node_) > 3U; }, 3s));
  {
    // The hold on the wire: the controller's global type on every slot, the
    // latched pose, no feedforward, no gains — the simulator keeps the last
    // gains it was given, which were the locomotion controller's.
    StopTicking();
    const WireWrite body = body_->Last();
    const WireWrite hand = hand_->Last();
    EXPECT_EQ(body.ct, rtc::CommandType::kPdFeedforward);
    EXPECT_EQ(hand.ct, rtc::CommandType::kPdFeedforward) << "the per-device override is dropped";
    EXPECT_FALSE(body.has_pd_gains);
    EXPECT_FALSE(hand.has_pd_gains);
    ASSERT_EQ(body.num_channels, static_cast<int>(body_->Pose().size()));
    for (int c = 0; c < body.num_channels; ++c) {
      const auto uc = static_cast<std::size_t>(c);
      EXPECT_EQ(body.commands[uc], body_->Pose()[uc]) << c;  // bit-exact latch
      EXPECT_EQ(body.feedforward[uc], 0.0) << c;
    }
    for (int c = 0; c < hand.num_channels; ++c) {
      const auto uc = static_cast<std::size_t>(c);
      EXPECT_EQ(hand.commands[uc], hand_->Pose()[uc]) << c;
    }
    EXPECT_EQ(body_->SafeWrites(), 0);
    StartTicking();
  }

  // ── Verified clear → back to demo_joint ──
  clear_started_.store(true, std::memory_order_release);
  std::chrono::milliseconds elapsed{0};
  const auto cleared = CallClear(std::string(Access::kOperatorEstopReason), &elapsed);
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->ok) << cleared->message;
  EXPECT_EQ(cleared->message.rfind("post-clear switch to 'DemoJointController': ok; ", 0), 0U)
      << cleared->message;
  RecordProperty("clear_plus_switch_ms", static_cast<int>(elapsed.count()));

  EXPECT_EQ(Access::GetActiveIdx(*node_), 0);
  EXPECT_EQ(Access::GetControllerState(*node_, 0), 1);
  EXPECT_EQ(Access::GetControllerState(*node_, 1), 0);
  EXPECT_FALSE(loco_->MpcManager().Enabled()) << "the locomotion controller was not paused";

  // demo_joint's first ticks put the shipped scene-servo gains back on the wire.
  const int joint_before = Count(Wire::kJoint);
  ASSERT_TRUE(WaitFor([this, joint_before] { return Count(Wire::kJoint) >= joint_before + 5; }, 3s))
      << "demo_joint_controller did not resume on the wire";
  StopTicking();
  {
    const WireWrite body = body_->Last();
    ASSERT_TRUE(body.has_pd_gains);
    ASSERT_EQ(body.num_channels, static_cast<int>(joint_kp_.size()));
    for (std::size_t i = 0; i < joint_kp_.size(); ++i) {
      EXPECT_EQ(body.pd_kp[i], joint_kp_[i]) << i;
      EXPECT_EQ(body.pd_kd[i], joint_kd_[i]) << i;
    }
  }

  // ── The two measured numbers ──
  // The window: ticks held with the latch already down — one watchdog period
  // plus the two-tick proof, give or take the tick on which the service
  // lowered the latch.
  const auto window_ticks = static_cast<std::uint64_t>(window_held_.load());
  const auto expected = Access::VerifyWindowTicks(*node_);
  RecordProperty("window_ticks", static_cast<int>(window_ticks));
  EXPECT_TRUE(window_ticks == expected || window_ticks == expected - 1U)
      << "window " << window_ticks << " ticks, expected " << expected << " or " << expected - 1U;

  // The gap: ticks whose wire carried the PREVIOUS controller's (locomotion)
  // command after the hold released and before the switch landed.
  RecordProperty("loco_ticks_after_release", loco_after_release_.load());
  RecordProperty("ticks_release_to_switch", release_to_switch_.load());
  EXPECT_LE(loco_after_release_.load(), 8)
      << "the previous controller drove the wire for " << loco_after_release_.load()
      << " ticks after the window closed";
  // Never a controller command on the wire while the hold was active.
  EXPECT_EQ(violations_.load(), 0);
  EXPECT_FALSE(Access::IsEstopped(*node_));
}

// The `resume` policy (the default; the humanoid sims do not ship it): the
// verified clear hands the wire straight back to the locomotion controller,
// which passes its pose guard on the standing robot and declares its own
// gains again on the first tick after the window.
TEST_F(LocoEstopOnClearTest, UnderResumeTheClearGivesTheWireBackToLocomotionWithItsGains) {
  Access::SetEstopOnClear(*node_, Access::EstopOnClear::kResume);
  std::string msg;
  ASSERT_TRUE(Access::CallSwitch(*node_, kLocoKey, msg)) << msg;
  ASSERT_TRUE(WaitFor([this] { return Count(Wire::kLoco) >= 5; }, 3s));

  const auto stop = CallTrigger();
  ASSERT_NE(stop, nullptr);
  ASSERT_TRUE(stop->success) << stop->message;
  ASSERT_TRUE(WaitFor([this] { return Access::EstopSubstituted(*node_) > 3U; }, 3s));
  const int loco_held = Count(Wire::kLoco);

  clear_started_.store(true, std::memory_order_release);
  const auto cleared = CallClear(std::string(Access::kOperatorEstopReason), nullptr);
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->ok) << cleared->message;
  EXPECT_EQ(cleared->message.find("post-clear switch"), std::string::npos) << cleared->message;
  EXPECT_EQ(Access::GetActiveIdx(*node_), 1) << "no switch under resume";
  ASSERT_TRUE(WaitFor([this, loco_held] { return Count(Wire::kLoco) >= loco_held + 5; }, 3s))
      << "the locomotion controller did not come back on the wire";
  StopTicking();
  const WireWrite body = body_->Last();
  ASSERT_TRUE(body.has_pd_gains);
  ASSERT_EQ(body.num_channels, static_cast<int>(loco_kp_.size()));
  const auto loco_kd = loco_cfg_["pd_feedforward"]["kd"].as<std::vector<double>>();
  for (std::size_t i = 0; i < loco_kp_.size(); ++i) {
    EXPECT_EQ(body.pd_kp[i], loco_kp_[i]) << i;
    EXPECT_EQ(body.pd_kd[i], loco_kd[i]) << i;
  }
  EXPECT_EQ(static_cast<LocomotionController::Mode>(loco_->LastDiag().mode),
            LocomotionController::Mode::kNominal);
  EXPECT_EQ(loco_->LastDiag().pose_guard_mask, 0U);
  EXPECT_EQ(loco_->LastDiag().fault_count, 0U);

  const auto window_ticks = static_cast<std::uint64_t>(window_held_.load());
  const auto expected = Access::VerifyWindowTicks(*node_);
  RecordProperty("window_ticks", static_cast<int>(window_ticks));
  EXPECT_TRUE(window_ticks == expected || window_ticks == expected - 1U)
      << "window " << window_ticks << " ticks, expected " << expected << " or " << expected - 1U;
  EXPECT_EQ(violations_.load(), 0);
  EXPECT_FALSE(Access::IsEstopped(*node_));
}

}  // namespace
