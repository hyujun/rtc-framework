#include "integrated_bringup/controllers/hand_joint_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "integrated_bringup/support/model_config_lookup.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_controllers/joint/dynamics_feedforward_law.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace rub = rtc_urdf_bridge;
using rtc::ControllerOutput;
using rtc::ControllerState;

namespace {

/// Peak velocity of a rest-to-rest quintic over Δq in time T is 15/8 · Δq / T.
constexpr double kQuinticPeakVelocityFactor = 15.0 / 8.0;

[[noreturn]] void Fail(const std::string& what) {
  throw std::runtime_error("hand_joint_controller: " + what);
}

double RequireDouble(const YAML::Node& cfg, const char* key) {
  if (!cfg[key]) {
    Fail(std::string("required key '") + key + "' is missing");
  }
  const double v = cfg[key].as<double>();
  if (!std::isfinite(v)) {
    Fail(std::string("'") + key + "' must be finite");
  }
  return v;
}

}  // namespace

// ── Config ──────────────────────────────────────────────────────────────────

HandJointMode ParseHandJointMode(std::string_view text) {
  for (std::size_t i = 0; i < kHandJointModeNames.size(); ++i) {
    if (text == kHandJointModeNames[i]) {
      return static_cast<HandJointMode>(i);
    }
  }
  std::string msg = "hand_joint_controller: 'mode' must be one of";
  for (const auto name : kHandJointModeNames) {
    msg += " \"" + std::string(name) + "\"";
  }
  throw std::runtime_error(msg + " (got \"" + std::string(text) + "\")");
}

std::string_view HandJointModeName(HandJointMode mode) noexcept {
  const auto i = static_cast<std::size_t>(mode);
  return i < kHandJointModeNames.size() ? kHandJointModeNames[i] : kHandJointModeNames[0];
}

HandJointConfig ParseHandJointConfig(const YAML::Node& cfg) {
  if (!cfg || !cfg.IsMap()) {
    Fail("the controller configuration is missing or not a map");
  }
  HandJointConfig out;
  out.trajectory_speed = RequireDouble(cfg, "trajectory_speed");
  if (!(out.trajectory_speed > 0.0)) {
    Fail("'trajectory_speed' must be > 0 (rad/s)");
  }
  out.max_traj_velocity = RequireDouble(cfg, "max_traj_velocity");
  if (!(out.max_traj_velocity >= out.trajectory_speed)) {
    Fail("'max_traj_velocity' must be >= trajectory_speed (rad/s)");
  }
  if (cfg["closure_error_threshold"]) {
    out.closure_error_threshold = cfg["closure_error_threshold"].as<double>();
    if (!(out.closure_error_threshold > 0.0) || !std::isfinite(out.closure_error_threshold)) {
      Fail("'closure_error_threshold' must be finite and > 0 (m)");
    }
  }
  // The lane this controller exists for. A position command would carry no
  // torque, and a torque command has no servo under it on this wire.
  if (!cfg["command_type"]) {
    Fail("required key 'command_type' is missing (must be \"pd_feedforward\")");
  }
  if (cfg["command_type"].as<std::string>() != "pd_feedforward") {
    Fail(
        "'command_type' must be \"pd_feedforward\" — this controller's output is a position "
        "reference plus a feedforward torque");
  }
  rtc::params::ParseDynamicsFeedforwardParams(cfg[std::string(kHandJointFfBlockKey)],
                                              kHandJointFfBlockKey, out.ff);
  // Optional; the shipped YAML spells it out. Off-whitelist throws (naming
  // the three values) — a typo must not fall back to tracking silently.
  if (cfg["mode"]) {
    // A key with no value parses as "" and is refused with the whitelist.
    out.mode =
        ParseHandJointMode(cfg["mode"].IsScalar() ? cfg["mode"].as<std::string>() : std::string());
  }
  return out;
}

// ── Construction ────────────────────────────────────────────────────────────

HandJointController::HandJointController(std::string_view urdf_path)
    : logger_(rclcpp::get_logger("integrated_bringup.hand_joint_controller")),
      urdf_path_(urdf_path) {
  // The model comes from the system model config the controller manager
  // injects (urdf.tree_models); the registry's per-controller URDF path is
  // kept only as the last-resort fallback InitModel describes.
}

HandJointController::~HandJointController() = default;

void HandJointController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);  // parses `topics:`
  cfg_loaded_ = false;
  cfg_ = ParseHandJointConfig(cfg);

  parsed_log_entries_.clear();
  if (cfg["logs"]) {
    if (!cfg["logs"].IsSequence()) {
      Fail("'logs' must be a sequence");
    }
    for (const auto& entry : cfg["logs"]) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        Fail("each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      // Closed set: a typo is a hard failure at parse time.
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != kJointFeedforwardLogMsgType) {
        Fail("unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
  // ONE device group: the hand. A second group would have no lane here.
  if (topic_config_.groups.size() != 1) {
    Fail("'topics' must name exactly one device group (the hand)");
  }

  Gains gains;
  gains.trajectory_speed = cfg_.trajectory_speed;
  gains.max_traj_velocity = cfg_.max_traj_velocity;
  gains.ff_enabled = cfg_.ff.enabled;
  gains.ff_gain = cfg_.ff.gravity_gain;
  gains.inertia_gain = cfg_.ff.inertia_gain;
  gains.mode = cfg_.mode;
  gains_lock_.Store(gains);

  InitModel();
  cfg_loaded_ = true;
  // Device configs may already be here (a config loaded a second time, after
  // them); otherwise OnDeviceConfigsSet wires the model when they arrive.
  if (GetDeviceNameConfig(GetPrimaryDeviceName()) != nullptr) {
    config_error_ = WireDevice();
  } else {
    config_error_ = "device configs have not arrived yet";
  }
}

void HandJointController::InitModel() {
  hand_handle_.reset();
  closed_handle_.reset();
  builder_.reset();
  root_link_.clear();
  tip_links_.clear();
  num_tips_ = 0;

  const auto* sys_cfg = GetSystemModelConfig();
  const auto group = GetPrimaryDeviceName();
  const auto* tm = sys_cfg ? FindTreeModel(*sys_cfg, group) : nullptr;
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty() || tm == nullptr) {
    Fail("the system model config declares no `urdf.tree_models." + group +
         "` — this controller's model is the tree model named after its device group");
  }
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rub::PinocchioModelBuilder>(*sys_cfg);
  }
  root_link_ = tm->root_link;
  tip_links_ = tm->tip_links;
  if (root_link_.empty()) {
    Fail("`urdf.tree_models." + group + ".root_link` is empty — the fingertip poses need a frame");
  }
  num_tips_ = std::min(tip_links_.size(), kHandJointMaxTips);

  hand_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetTreeModel(group));
  if (hand_handle_->nv() > kHandJointMaxDof) {
    Fail("tree model `" + group + "` has nv=" + std::to_string(hand_handle_->nv()) +
         " > capacity " + std::to_string(kHandJointMaxDof));
  }

  // Extended URDF: the loop closures of the WHOLE model. The handle is built
  // here (no device facts needed); the q_a bridge waits for the device roster.
  const auto& constraints = builder_->GetConstraintModels();
  if (!constraints.empty()) {
    try {
      closed_handle_ = std::make_unique<rub::RtClosedChainHandle>(
          builder_->GetFullModel(), constraints, builder_->GetClosureActuatedJointIds(),
          builder_->GetClosureReferenceConfig());
    } catch (const std::exception& e) {
      Fail(std::string("closed-chain handle construction failed (ill-posed closure): ") + e.what());
    }
    q_a_ = Eigen::VectorXd::Zero(closed_handle_->nv_independent());
  }
}

std::string HandJointController::WireDevice() {
  const auto group = GetPrimaryDeviceName();
  const auto* dev = GetDeviceNameConfig(group);
  if (dev == nullptr) {
    return "device group '" + group + "' has no device config";
  }
  joint_names_ = dev->joint_state_names;
  dof_ = static_cast<int>(joint_names_.size());
  if (dof_ <= 0) {
    return "device group '" + group + "' declares no joint_state_names";
  }
  if (dof_ > kHandJointMaxDof) {
    return "device group '" + group + "' has " + std::to_string(dof_) + " joints > capacity " +
           std::to_string(kHandJointMaxDof);
  }
  q_dev_ = Eigen::VectorXd::Zero(dof_);
  // A reconfigure with another roster must not inherit the old matrix.
  have_m_ = false;
  const Eigen::Index n_model = closed_handle_ != nullptr
                                   ? static_cast<Eigen::Index>(closed_handle_->nv_independent())
                                   : static_cast<Eigen::Index>(dof_);
  qdd_model_ = Eigen::VectorXd::Zero(n_model);
  tau_model_ = Eigen::VectorXd::Zero(n_model);

  // Position limits (device, else the model's; the fallback bounds below are
  // wide on purpose — a hand without limits in its YAML is clamped by the
  // model, not by a number invented here).
  std::array<std::vector<double>, ControllerState::kMaxDevices> lo{};
  std::array<std::vector<double>, ControllerState::kMaxDevices> hi{};
  std::array<std::vector<double>, ControllerState::kMaxDevices> vmax{};
  LoadDeviceLimitsFromConfig(lo, hi, vmax, -std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(), cfg_.max_traj_velocity);
  for (int i = 0; i < dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_min_[ui] = ui < lo[0].size() ? lo[0][ui] : -std::numeric_limits<double>::infinity();
    q_max_[ui] = ui < hi[0].size() ? hi[0][ui] : std::numeric_limits<double>::infinity();
    // Per-joint velocity cap of a goal trajectory: the device rating where
    // one is declared (and finite, > 0), never above the YAML cap.
    double v = cfg_.max_traj_velocity;
    if (ui < vmax[0].size() && std::isfinite(vmax[0][ui]) && vmax[0][ui] > 0.0) {
      v = std::min(v, vmax[0][ui]);
    }
    v_max_[ui] = v;
  }

  // Torque bound per joint: the YAML bound, tightened by the device rating
  // where one is declared and positive. The YAML key is required so a
  // placeholder rating is never the only bound.
  const std::vector<double>* rating = (dev->joint_limits && !dev->joint_limits->max_torque.empty())
                                          ? &dev->joint_limits->max_torque
                                          : nullptr;
  for (int i = 0; i < dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    double b = cfg_.ff.max_abs;
    if (rating != nullptr && ui < rating->size() && std::isfinite((*rating)[ui]) &&
        (*rating)[ui] > 0.0) {
      b = std::min(b, (*rating)[ui]);
    }
    ff_bound_[ui] = b;
  }

  if (closed_handle_ != nullptr) {
    // q_a bridge by NAME: every independent joint of the projection must be a
    // joint of this device. A device joint that is not independent (none on
    // the shipped hand) simply gets no model term (ff 0, tail policy).
    const std::vector<std::string> indep = closed_handle_->GetIndependentJointNames();
    bridge_.assign(indep.size(), -1);
    for (std::size_t k = 0; k < indep.size(); ++k) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), indep[k]);
      if (it == joint_names_.end()) {
        return "closed-chain independent joint '" + indep[k] +
               "' is not a joint of device group '" + group +
               "' — the loop cannot be projected from this group";
      }
      bridge_[k] = static_cast<int>(it - joint_names_.begin());
    }
    const auto res = closed_fk_.ConfigureBorrowed(
        *closed_handle_, std::span<const std::string>(tip_links_.data(), num_tips_), root_link_,
        cfg_.closure_error_threshold);
    LogHandFkWiring(logger_, "[hand_joint]", res, closed_fk_.missing_joint());
    if (res != HandFkWiringResult::kActive) {
      return "closed-chain fingertip FK could not be wired (root '" + root_link_ +
             "' or a tip link is not a frame of the model)";
    }
  } else {
    // Plain model: the device order IS the model's input order from here on.
    if (!hand_handle_->SetJointOrder(joint_names_)) {
      return "device group '" + group + "': joint_state_names do not cover the tree model `" +
             group +
             "` exactly once (a loop-passive joint left in a plain tree is the usual cause)";
    }
    if (hand_handle_->nv() != dof_) {
      return "device group '" + group + "' has " + std::to_string(dof_) +
             " joints but its tree model has nv=" + std::to_string(hand_handle_->nv());
    }
    root_fid_ = hand_handle_->GetFrameId(root_link_);
    if (root_fid_ == 0) {
      return "tree model root '" + root_link_ + "' is not a frame of the model";
    }
    for (std::size_t f = 0; f < num_tips_; ++f) {
      tip_fid_[f] = hand_handle_->GetFrameId(tip_links_[f]);
      if (tip_fid_[f] == 0) {
        return "tip link '" + tip_links_[f] + "' is not a frame of the model";
      }
    }
  }
  return {};
}

void HandJointController::OnDeviceConfigsSet() {
  if (!cfg_loaded_) {
    config_error_ = "the controller configuration has not been loaded";
    return;
  }
  config_error_ = WireDevice();
  if (!config_error_.empty()) {
    RCLCPP_ERROR(logger_, "[hand_joint] %s", config_error_.c_str());
  }
}

// ── Targets ─────────────────────────────────────────────────────────────────

void HandJointController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

void HandJointController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                             bool is_task) noexcept {
  // Also while a re-seed is pending: the seed would overwrite the goal, so it
  // is counted as refused rather than accepted and lost. A non-tracking mode
  // refuses too (the reference is the measurement there) — unlike E-STOP,
  // whose goals never reach this function (Compute discards them).
  if (device_idx != 0 || is_task || !seeded_ || need_reseed_.load(std::memory_order_relaxed) ||
      estop_active_ || gains_tick_.mode != HandJointMode::kTracking ||
      static_cast<int>(values.size()) != dof_) {
    goal_rejects_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  for (const double v : values) {
    if (!std::isfinite(v)) {
      goal_rejects_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
  // The speed never exceeds the (read-only) cap, whatever the runtime
  // parameter was set to — the parameter callback refuses that too, but the
  // tick is the bound that holds (NUM floor rule: loader AND tick).
  const double speed = std::clamp(gains_tick_.trajectory_speed, kHandJointMinSpeed,
                                  std::max(kHandJointMinSpeed, gains_tick_.max_traj_velocity));
  rtc::trajectory::JointSpaceTrajectory<kHandJointMaxDof>::State start{};
  rtc::trajectory::JointSpaceTrajectory<kHandJointMaxDof>::State goal{};
  double max_dist = 0.0;
  // Rest-to-rest from the CURRENT reference: a goal that lands mid-motion
  // restarts the quintic with zero start velocity (the reference velocity
  // steps to 0 for one tick, which a position servo takes), so the 15/8 peak
  // factor below is exact and the per-joint cap holds on a retarget too.
  double duration = max_dist / speed;
  for (int i = 0; i < dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    goal_[ui] = std::clamp(values[ui], q_min_[ui], q_max_[ui]);
    start.positions[ui] = q_ref_[ui];
    start.velocities[ui] = 0.0;
    goal.positions[ui] = goal_[ui];
    const double dist = std::abs(goal_[ui] - q_ref_[ui]);
    max_dist = std::max(max_dist, dist);
    // Extended until this joint's peak velocity (15/8 · Δq / T) is within
    // its cap (device rating, never above the YAML cap).
    const double cap =
        std::max(kHandJointMinSpeed, std::min(v_max_[ui], gains_tick_.max_traj_velocity));
    duration = std::max(duration, kQuinticPeakVelocityFactor * dist / cap);
  }
  duration = std::max(duration, max_dist / speed);
  duration = std::max(duration, GetDefaultDt());
  traj_.initialize(start, goal, duration);
  traj_time_ = 0.0;
  traj_active_ = true;
}

void HandJointController::ResetTargetInitialization() noexcept {
  need_reseed_.store(true, std::memory_order_relaxed);
}

// ── E-STOP ──────────────────────────────────────────────────────────────────

void HandJointController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void HandJointController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool HandJointController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

// ── RT tick ─────────────────────────────────────────────────────────────────

ControllerOutput HandJointController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("HandJointController::Compute");
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  gains_tick_ = gains_lock_.Load();
  const bool was_estopped = estop_active_;
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (was_estopped && !estop_active_) {
    // release: the reference restarts from the measurement
    need_reseed_.store(true, std::memory_order_relaxed);
  }
  // Mode edge, judged here and not in AdvanceReference (which returns early
  // on an unreadable tick and would miss or double-consume the edge): back to
  // tracking re-seeds, the goal becomes "here".
  if (mode_prev_ != HandJointMode::kTracking && gains_tick_.mode == HandJointMode::kTracking) {
    need_reseed_.store(true, std::memory_order_relaxed);
  }
  mode_prev_ = gains_tick_.mode;
  readable_ = state.num_devices > 0 && rtc::IsDeviceReadable(state.devices[0], dof_);
  if (state.num_devices > 0 && rtc::IsGateClosedByWidth(state.devices[0], dof_)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, logging::kThrottleSlowMs,
                         "F5 gate closed: hand device reports num_channels=%d < dof=%d — the "
                         "hand is silenced every tick; fix joint_state_names or the backend's "
                         "reported channel count",
                         state.devices[0].num_channels, dof_);
  }

  // Goals: applied only on a tick the reference is live; otherwise dropped
  // (E-STOP) so they do not land on the tick the hand resumes. A non-tracking
  // mode drains them and ApplyPendingTarget refuses each (counted).
  if (estop_active_) {
    DiscardPendingTargets();
  } else {
    (void)DrainPendingTargets();
  }

  EvaluateModel(state);
  AdvanceReference(state, dt);
  ComputeFeedforward();

  ControllerOutput out{};
  FillOutput(state, out);
  FillTickRecord(state);
  PushLogs(state, out);
  return out;
}

void HandJointController::EvaluateModel(const ControllerState& state) noexcept {
  model_fresh_ = false;
  if (dof_ <= 0 || !readable_) {
    return;  // hold: g_dev_ / fingertip caches keep their last valid values
  }
  const auto& dev = state.devices[0];
  if (closed_handle_ != nullptr) {
    for (std::size_t k = 0; k < bridge_.size(); ++k) {
      q_a_[static_cast<Eigen::Index>(k)] = dev.positions[static_cast<std::size_t>(bridge_[k])];
    }
    const rub::RtClosedChainHandle::Status st = closed_handle_->Update(
        std::span<const double>(q_a_.data(), static_cast<std::size_t>(q_a_.size())));
    // Snapshot BEFORE UpdateDynamics: that call may set `held` for a dynamics
    // reason while this tick's placement is still valid (#175).
    kin_status_ = st;
    // Fingertip FK on the same projection (borrowed mode): fresh = the
    // projection ran this tick on this tick's measurement (readable_ above).
    closed_fk_.UpdateFromProjection(*closed_handle_, /*projection_fresh=*/!st.held, st);
    if (st.held || st.singular) {
      return;  // last-good torque stays
    }
    const rub::RtClosedChainHandle::Status dst = closed_handle_->UpdateDynamics({});
    if (dst.held) {
      return;
    }
    const Eigen::Ref<const Eigen::VectorXd> g_a = closed_handle_->GetGeneralizedGravity();
    for (std::size_t k = 0; k < bridge_.size(); ++k) {
      g_dev_[static_cast<std::size_t>(bridge_[k])] = g_a[static_cast<Eigen::Index>(k)];
    }
    // M_a comes with g_a from the same UpdateDynamics, and the handle keeps
    // it through a later held / singular tick, as it keeps g_a.
    have_m_ = true;
    have_g_ = true;
    model_fresh_ = true;
    return;
  }
  // Plain model: the handle takes q in device order (SetJointOrder) and
  // returns g in pinocchio v-order; scatter it back to device order.
  for (int i = 0; i < dof_; ++i) {
    q_dev_[i] = dev.positions[static_cast<std::size_t>(i)];
  }
  const std::span<const double> q_span(q_dev_.data(), static_cast<std::size_t>(dof_));
  hand_handle_->ComputeForwardKinematics(q_span);
  hand_handle_->ComputeGeneralizedGravity(q_span);
  hand_handle_->ReorderOutput(hand_handle_->GetGeneralizedGravity(),
                              std::span<double>(g_dev_.data(), static_cast<std::size_t>(dof_)));
  // CRBA only on a tick that can use it: the term is on and a goal is being
  // followed (a goal drained this tick has already set traj_active_). At rest
  // q̈_ref is 0 and the gravity-only lane pays nothing; the flag says the
  // matrix is THIS tick's, so one from another pose is never read.
  have_m_ = gains_tick_.inertia_gain != 0.0 && traj_active_;
  if (have_m_) {
    hand_handle_->ComputeMassMatrix(q_span);
  }
  have_g_ = true;
  model_fresh_ = true;
}

void HandJointController::AdvanceReference(const ControllerState& state, double dt) noexcept {
  // Every branch but a running quintic leaves the reference at rest — a
  // frozen one included, so an outage never carries the last acceleration.
  std::fill(qdd_ref_.begin(), qdd_ref_.end(), 0.0);
  if (dof_ <= 0 || !readable_) {
    return;  // frozen: last reference, trajectory clock stopped
  }
  const auto& dev = state.devices[0];
  if (estop_active_ || gains_tick_.mode != HandJointMode::kTracking) {
    // E-STOP: the measured pose is commanded (the controller manager
    // substitutes its own hold on the wire anyway). damping /
    // damping_feedforward: the measured pose IS the command.
    HoldAtMeasurement(state);
    return;
  }
  // exchange, not load + store: an on_activate store landing between the two
  // would be erased (it would ask for what this tick does anyway, but the
  // consume should not be able to lose a request).
  if (need_reseed_.exchange(false, std::memory_order_relaxed) || !seeded_) {
    for (int i = 0; i < dof_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_ref_[ui] = std::clamp(dev.positions[ui], q_min_[ui], q_max_[ui]);
      qd_ref_[ui] = 0.0;
      goal_[ui] = q_ref_[ui];
    }
    traj_active_ = false;
    traj_time_ = 0.0;
    seeded_ = true;
    return;
  }
  if (!traj_active_) {
    return;
  }
  traj_time_ += dt;
  if (traj_time_ >= traj_.duration()) {
    traj_active_ = false;
    for (int i = 0; i < dof_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_ref_[ui] = goal_[ui];
      qd_ref_[ui] = 0.0;
    }
    return;
  }
  const auto s = traj_.compute(traj_time_);
  for (int i = 0; i < dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_ref_[ui] = std::clamp(s.positions[ui], q_min_[ui], q_max_[ui]);
    qd_ref_[ui] = s.velocities[ui];
    qdd_ref_[ui] = s.accelerations[ui];
  }
}

void HandJointController::HoldAtMeasurement(const ControllerState& state) noexcept {
  const auto& dev = state.devices[0];
  // Not clamped to the position limits: a finger pushed past one is not
  // pulled back (that would be a servo error, not a hold). The re-seed on
  // the return to tracking clamps, as every tracking reference does.
  for (int i = 0; i < dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_ref_[ui] = dev.positions[ui];
    qd_ref_[ui] = 0.0;
    goal_[ui] = q_ref_[ui];
  }
  traj_active_ = false;
  traj_time_ = 0.0;
  // A hold is a seed: a controller activated in a non-tracking mode has a
  // valid output from its first readable tick (an invalid one for 0.1 s is a
  // global E-STOP by the controller manager). The pending re-seed is spent
  // here; the mode edge / E-STOP release re-arm it.
  seeded_ = true;
  (void)need_reseed_.exchange(false, std::memory_order_relaxed);
}

void HandJointController::ComputeFeedforward() noexcept {
  clamped_mask_ = 0;
  ff_finite_ = false;
  const auto n = static_cast<std::size_t>(std::max(dof_, 0));
  // One OR chain, appended to: `damping` zeroes the lane like the switch does;
  // `damping_feedforward` leaves it to the switch and the E-STOP.
  std::fill_n(tau_m_.begin(), n, 0.0);
  inertia_gain_applied_ = 0.0;
  if (estop_active_ || !have_g_ || !gains_tick_.ff_enabled ||
      gains_tick_.mode == HandJointMode::kDamping) {
    std::fill_n(tau_ff_.begin(), n, 0.0);
    return;
  }
  // Inertia term: the handle's M times this tick's q̈_ref. No matrix → the
  // term is off this tick (gravity alone). At rest the product is 0 and is
  // not formed.
  const double k_m = have_m_ ? gains_tick_.inertia_gain : 0.0;
  inertia_gain_applied_ = k_m;
  if (k_m != 0.0 && traj_active_) {
    ComputeInertiaTerm(n);
  }
  const rtc::joint::DynamicsFeedforwardGainsView k{.gravity_gain = gains_tick_.ff_gain,
                                                   .coriolis_gain = 0.0,
                                                   .inertia_gain = k_m,
                                                   .max_abs = {ff_bound_.data(), n}};
  const rtc::joint::DynamicsFeedforwardInputs in{
      .gravity = {g_dev_.data(), n}, .coriolis = {}, .inertia = {tau_m_.data(), n}};
  const auto r = rtc::joint::ComputeDynamicsFeedforward(k, in, n, {tau_ff_.data(), n});
  ff_finite_ = r.finite;
  clamped_mask_ = r.clamped;
}

void HandJointController::ComputeInertiaTerm(std::size_t n) noexcept {
  if (closed_handle_ != nullptr) {
    // Device → independent order by the name bridge, M_a · q̈, and back.
    for (std::size_t k = 0; k < bridge_.size(); ++k) {
      qdd_model_[static_cast<Eigen::Index>(k)] = qdd_ref_[static_cast<std::size_t>(bridge_[k])];
    }
    tau_model_.noalias() = closed_handle_->GetMassMatrix() * qdd_model_;
    for (std::size_t k = 0; k < bridge_.size(); ++k) {
      tau_m_[static_cast<std::size_t>(bridge_[k])] = tau_model_[static_cast<Eigen::Index>(k)];
    }
    return;
  }
  // Plain: the handle's own maps — ReorderInput scatters device order into
  // v order, ReorderOutput gathers v order back into device order.
  hand_handle_->ReorderInput(std::span<const double>(qdd_ref_.data(), n), qdd_model_);
  tau_model_.noalias() = hand_handle_->GetMassMatrix() * qdd_model_;
  hand_handle_->ReorderOutput(tau_model_, std::span<double>(tau_m_.data(), n));
}

void HandJointController::FillOutput(const ControllerState& state, ControllerOutput& out) noexcept {
  out.num_devices = 1;
  out.command_type = rtc::CommandType::kPdFeedforward;
  out.valid = seeded_ && dof_ > 0;
  auto& d = out.devices[0];
  d.num_channels = dof_;
  d.goal_type = rtc::GoalType::kJoint;
  for (int i = 0; i < dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    d.commands[ui] = q_ref_[ui];
    d.goal_positions[ui] = goal_[ui];
    d.target_positions[ui] = goal_[ui];
    d.trajectory_positions[ui] = q_ref_[ui];
    d.trajectory_velocities[ui] = qd_ref_[ui];
    d.feedforward[ui] = tau_ff_[ui];
  }
  // Fingertip poses, root-relative, for TF (AppendHandTipSlots group 0).
  for (std::size_t f = 0; f < num_tips_; ++f) {
    pinocchio::SE3 pose;
    bool ok = false;
    if (closed_handle_ != nullptr) {
      ok = closed_fk_.GetFingertipHandRootPose(f, pose);
    } else if (readable_ || seeded_) {
      const pinocchio::SE3& root = hand_handle_->GetFramePlacement(root_fid_);
      pose = root.actInv(hand_handle_->GetFramePlacement(tip_fid_[f]));
      ok = seeded_;
    }
    out.task_link_pose_valid[f] = ok;
    if (ok) {
      const Eigen::Quaterniond q(pose.rotation());
      out.task_link_poses[f].position = {pose.translation().x(), pose.translation().y(),
                                         pose.translation().z()};
      out.task_link_poses[f].quaternion = {q.w(), q.x(), q.y(), q.z()};
    }
  }
  (void)state;
}

void HandJointController::FillTickRecord(const ControllerState& state) noexcept {
  tick_.t_relative_s = state.t_relative_s;
  tick_.tick = state.iteration;
  tick_.estop = estop_active_;
  tick_.readable = readable_;
  tick_.seeded = seeded_;
  tick_.traj_active = traj_active_;
  tick_.ff_enabled = gains_tick_.ff_enabled;
  tick_.mode = static_cast<std::uint8_t>(gains_tick_.mode);
  tick_.ff_finite = ff_finite_;
  tick_.model_fresh = model_fresh_;
  tick_.held = kin_status_.held;
  tick_.held_ticks = static_cast<std::uint32_t>(std::max(0, kin_status_.held_ticks));
  tick_.singular = kin_status_.singular;
  tick_.closure_error = kin_status_.closure_error;
  tick_.clamped_mask = clamped_mask_;
  tick_.inertia_gain = inertia_gain_applied_;
  tick_.num_joints = static_cast<std::uint8_t>(std::clamp(dof_, 0, kHandJointMaxDof));
  const auto n = static_cast<std::size_t>(tick_.num_joints);
  for (std::size_t i = 0; i < n; ++i) {
    tick_.q[i] = state.num_devices > 0 ? state.devices[0].positions[i] : 0.0;
    tick_.q_ref[i] = q_ref_[i];
    tick_.g[i] = g_dev_[i];
    tick_.tau_ff[i] = tau_ff_[i];
    tick_.qdd_ref[i] = qdd_ref_[i];
    tick_.tau_inertia[i] = inertia_gain_applied_ * tau_m_[i];
  }
}

void HandJointController::PushLogs(const ControllerState& state,
                                   const ControllerOutput& out) noexcept {
  if (state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, out, /*device_idx=*/0, pod);
    state_log_handle_.Push(pod);
  }
  if (ff_log_handle_) {
    ff_log_handle_.Push(tick_);
  }
}

void HandJointController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

}  // namespace integrated_bringup
