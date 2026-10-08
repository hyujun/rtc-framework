// ── DemoDualArmController: configuration, runtime construction, ingress ─────
//
// Everything here is non-RT (config parsing, model and solve construction) or
// a wait-free hook (the target lanes, the E-STOP and fault requests). The tick
// is in compute.cpp.

#include "integrated_bringup/controllers/demo_dualarm_controller.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "integrated_bringup/support/hand_fk_wiring.hpp"
#include "integrated_bringup/support/model_config_lookup.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup {

namespace {

constexpr const char* kKey = "demo_dualarm_controller";
/// A locked joint's velocity bound: the hand group's joints are decision
/// variables of the solve (the task frames may hang off them) that this
/// controller does not command through it.
constexpr double kLockedJointVelocity = 1e-9;

[[noreturn]] void Fail(const std::string& what) {
  throw std::runtime_error(std::string(kKey) + ": " + what);
}

YAML::Node Require(const YAML::Node& parent, const char* key, const std::string& where) {
  if (!parent || !parent.IsMap() || !parent[key]) {
    Fail("missing required key '" + where + key + "'");
  }
  return parent[key];
}

double RequireDouble(const YAML::Node& parent, const char* key, const std::string& where) {
  const YAML::Node node = Require(parent, key, where);
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const std::exception&) {
    Fail("'" + where + key + "' is not a number");
  }
  if (!std::isfinite(value)) {
    Fail("'" + where + key + "' is not finite");
  }
  return value;
}

std::string RequireString(const YAML::Node& parent, const char* key, const std::string& where) {
  const YAML::Node node = Require(parent, key, where);
  // yaml-cpp reads an empty value (`key:`) as the string "null", which would
  // go on to be looked up as a frame or a joint of that name.
  if (node.IsNull()) {
    Fail("'" + where + key + "' is empty");
  }
  std::string value;
  try {
    value = node.as<std::string>();
  } catch (const std::exception&) {
    Fail("'" + where + key + "' is not a string");
  }
  return value;
}

void RequireRange(bool ok, const std::string& key, const char* rule) {
  if (!ok) {
    Fail("'" + key + "' must be " + rule);
  }
}

}  // namespace

// ── YAML ────────────────────────────────────────────────────────────────────

DualArmConfig ParseDualArmConfig(const YAML::Node& cfg) {
  if (!cfg || !cfg.IsMap()) {
    Fail("no configuration node — this controller has no defaults");
  }
  DualArmConfig out;

  // Position commands only. A key rather than an assumption: a config that
  // asks for torque must be told, not silently run as position.
  if (const std::string command_type = RequireString(cfg, "command_type", "");
      command_type != "position") {
    Fail("'command_type' is '" + command_type + "' — this controller commands positions only");
  }

  const YAML::Node clik = Require(cfg, "clik", "");
  out.damping_sq = RequireDouble(clik, "damping_sq", "clik.");
  RequireRange(out.damping_sq > 0.0, "clik.damping_sq", "> 0");
  out.w_smooth = RequireDouble(clik, "w_smooth", "clik.");
  RequireRange(out.w_smooth >= 0.0, "clik.w_smooth", ">= 0");
  {
    const YAML::Node qp = Require(clik, "qp", "clik.");
    const double max_iter = RequireDouble(qp, "max_iter", "clik.qp.");
    RequireRange(max_iter >= 1.0 && max_iter <= 1000.0 && max_iter == std::floor(max_iter),
                 "clik.qp.max_iter", "an integer in [1, 1000]");
    out.max_iter = static_cast<int>(max_iter);
  }
  {
    // A key although only one value is accepted: a relative task cannot be
    // paired with the kinematic form, and running with no acceleration
    // constraint at all is a different controller. The key keeps the choice
    // visible in the file instead of implied by the code.
    const std::string form = RequireString(clik, "accel_constraint", "clik.");
    if (form != "dynamic") {
      Fail("'clik.accel_constraint' is '" + form +
           "' — only 'dynamic' is supported (the tasks are relative to a base frame, which the "
           "kinematic form does not take)");
    }
  }
  out.eta_tau = RequireDouble(clik, "eta_tau", "clik.");
  RequireRange(out.eta_tau > 0.0 && out.eta_tau <= kDualArmEtaTauMax, "clik.eta_tau",
               "in (0, 1.2]");
  out.limit_margin = RequireDouble(clik, "limit_margin", "clik.");
  RequireRange(out.limit_margin >= 0.0, "clik.limit_margin", ">= 0");
  out.joint_velocity_max = RequireDouble(clik, "joint_velocity_max", "clik.");
  RequireRange(out.joint_velocity_max > 0.0, "clik.joint_velocity_max", "> 0");
  {
    const YAML::Node brake = Require(clik, "brake", "clik.");
    const YAML::Node enabled = Require(brake, "enabled", "clik.brake.");
    try {
      out.brake_enabled = enabled.as<bool>();
    } catch (const std::exception&) {
      Fail("'clik.brake.enabled' is not a boolean");
    }
    out.brake_margin = RequireDouble(brake, "margin", "clik.brake.");
    RequireRange(out.brake_margin > 0.0 && out.brake_margin <= 1.0, "clik.brake.margin",
                 "in (0, 1]");
  }

  // ── target frames ─────────────────────────────────────────────────────────
  {
    const YAML::Node frames = Require(clik, "target_frames", "clik.");
    if (!frames.IsSequence()) {
      Fail("'clik.target_frames' must be a sequence of frame names");
    }
    std::set<std::string> seen;
    for (const auto& node : frames) {
      const auto name = node.as<std::string>();
      if (name.empty() || !seen.insert(name).second) {
        Fail("'clik.target_frames' has an empty or repeated name ('" + name + "')");
      }
      out.target_frames.push_back(name);
    }
    if (out.target_frames.size() > kDualArmMaxTargetFrames) {
      Fail("'clik.target_frames' lists " + std::to_string(out.target_frames.size()) +
           " frames, capacity " + std::to_string(kDualArmMaxTargetFrames));
    }
  }

  // ── tasks ─────────────────────────────────────────────────────────────────
  {
    const YAML::Node tasks = Require(clik, "tasks", "clik.");
    if (!tasks.IsSequence() || tasks.size() == 0) {
      Fail("'clik.tasks' must be a non-empty sequence");
    }
    if (tasks.size() > kDualArmMaxTasks) {
      Fail("'clik.tasks' lists " + std::to_string(tasks.size()) + " tasks, capacity " +
           std::to_string(kDualArmMaxTasks));
    }
    std::set<std::string> names;
    std::size_t k = 0;
    for (const auto& node : tasks) {
      const std::string where = "clik.tasks[" + std::to_string(k++) + "].";
      DualArmConfig::Task task;
      task.name = RequireString(node, "name", where);
      if (!IsTopicToken(task.name)) {
        Fail("'" + where + "name' ('" + task.name +
             "') must match [A-Za-z][A-Za-z0-9_]* — it names the goal topic");
      }
      if (!names.insert(task.name).second) {
        Fail("'" + where + "name' repeats '" + task.name + "'");
      }
      task.frame = RequireString(node, "frame", where);
      task.base_frame = RequireString(node, "base_frame", where);
      if (task.frame.empty() || task.base_frame.empty()) {
        // No "empty means world": the base frame is always a named frame of the
        // model, so that what a goal is relative to is written in the file.
        Fail("'" + where + "frame' and '" + where + "base_frame' must both name a frame");
      }
      if (task.frame == task.base_frame) {
        Fail("'" + where + "frame' and 'base_frame' are the same frame ('" + task.frame + "')");
      }
      const std::string kind = RequireString(node, "kind", where);
      if (kind != "se3") {
        Fail("'" + where + "kind' is '" + kind + "' — only 'se3' is supported");
      }
      task.gain_linear = RequireDouble(node, "gain_linear", where);
      task.gain_angular = RequireDouble(node, "gain_angular", where);
      RequireRange(task.gain_linear >= 0.0 && task.gain_angular >= 0.0, where + "gain_*", ">= 0");
      task.weight = RequireDouble(node, "weight", where);
      RequireRange(task.weight > 0.0, where + "weight", "> 0");
      task.fb_lin_max = RequireDouble(node, "fb_lin_max", where);
      task.fb_ang_max = RequireDouble(node, "fb_ang_max", where);
      RequireRange(task.fb_lin_max >= 0.0 && task.fb_ang_max >= 0.0, where + "fb_*_max", ">= 0");
      out.tasks.push_back(std::move(task));
    }
  }

  // ── posture groups ────────────────────────────────────────────────────────
  {
    const YAML::Node groups = Require(clik, "posture_groups", "clik.");
    if (!groups.IsSequence() || groups.size() == 0) {
      Fail("'clik.posture_groups' must be a non-empty sequence");
    }
    if (groups.size() > kDualArmMaxPostureGroups) {
      Fail("'clik.posture_groups' lists " + std::to_string(groups.size()) + " groups, capacity " +
           std::to_string(kDualArmMaxPostureGroups));
    }
    std::set<std::string> names;
    std::set<std::string> joints;
    std::size_t g = 0;
    for (const auto& node : groups) {
      const std::string where = "clik.posture_groups[" + std::to_string(g++) + "].";
      DualArmConfig::PostureGroup group;
      group.name = RequireString(node, "name", where);
      if (!IsTopicToken(group.name) || !names.insert(group.name).second) {
        Fail("'" + where + "name' ('" + group.name +
             "') must be a unique [A-Za-z][A-Za-z0-9_]* token — it names a parameter");
      }
      const YAML::Node list = Require(node, "joints", where);
      if (!list.IsSequence() || list.size() == 0) {
        Fail("'" + where + "joints' must be a non-empty sequence");
      }
      for (const auto& j : list) {
        const auto joint = j.as<std::string>();
        if (!joints.insert(joint).second) {
          Fail("'" + where + "joints' lists '" + joint +
               "', which is already in a posture group (the groups are disjoint)");
        }
        group.joints.push_back(joint);
      }
      group.weight = RequireDouble(node, "weight", where);
      RequireRange(group.weight >= 0.0, where + "weight", ">= 0");
      group.gain = RequireDouble(node, "gain", where);
      RequireRange(group.gain >= 0.0, where + "gain", ">= 0");
      out.posture_groups.push_back(std::move(group));
    }
  }

  // ── trajectory ────────────────────────────────────────────────────────────
  {
    const YAML::Node traj = Require(cfg, "trajectory", "");
    const auto speed = [&traj](const char* key) {
      const double value = RequireDouble(traj, key, "trajectory.");
      RequireRange(value > 0.0, std::string("trajectory.") + key, "> 0");
      return std::max(kDualArmMinSpeed, value);
    };
    out.linear_speed = speed("linear_speed");
    out.angular_speed = speed("angular_speed");
    out.linear_speed_max = speed("linear_speed_max");
    out.angular_speed_max = speed("angular_speed_max");
    out.hand_speed = speed("hand_speed");
    out.hand_speed_max = speed("hand_speed_max");
  }

  // ── fault ─────────────────────────────────────────────────────────────────
  {
    const YAML::Node fault = Require(cfg, "fault", "");
    const double ticks = RequireDouble(fault, "max_qp_fail_ticks", "fault.");
    RequireRange(ticks >= 1.0 && ticks <= 1.0e6 && ticks == std::floor(ticks),
                 "fault.max_qp_fail_ticks", "an integer >= 1");
    out.max_qp_fail_ticks = static_cast<int>(ticks);
    out.track_err_max = RequireDouble(fault, "track_err_max", "fault.");
    RequireRange(out.track_err_max >= 0.0, "fault.track_err_max", ">= 0 (0 = off)");
  }
  return out;
}

// ── Construction ────────────────────────────────────────────────────────────

DemoDualArmController::DemoDualArmController(std::string_view /*urdf_path*/)
    : logger_(rclcpp::get_logger("integrated_bringup.demo_dualarm_controller")) {
  // The model comes from the system model config the controller manager
  // injects (urdf.tree_models); the registry's per-controller URDF path is not
  // read.
  target_frame_idx_.fill(-1);
}

DemoDualArmController::~DemoDualArmController() = default;

void DemoDualArmController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);  // parses `topics:`
  cfg_loaded_ = false;
  cfg_ = ParseDualArmConfig(cfg);

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
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != kDualArmDiagLogMsgType) {
        Fail("unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
  if (topic_config_.groups.empty() || topic_config_.groups.size() > kMaxOwnedGroups) {
    Fail("'topics' must name one or two device groups (the body group first, then the hand)");
  }

  Gains gains;
  for (std::size_t k = 0; k < cfg_.tasks.size(); ++k) {
    gains.task_gain_linear[k] = cfg_.tasks[k].gain_linear;
    gains.task_gain_angular[k] = cfg_.tasks[k].gain_angular;
  }
  for (std::size_t g = 0; g < cfg_.posture_groups.size(); ++g) {
    gains.posture_gain[g] = cfg_.posture_groups[g].gain;
  }
  gains.linear_speed = cfg_.linear_speed;
  gains.angular_speed = cfg_.angular_speed;
  gains.hand_speed = cfg_.hand_speed;
  gains_lock_.Store(gains);
  cfg_loaded_ = true;

  // The controller manager loads the config BEFORE it sends the device configs
  // (the hook below then builds the runtime). A caller that goes the other way
  // round — device configs first, or a config loaded a second time — would
  // otherwise keep a runtime built from the previous config.
  if (!device_name_configs_.empty()) {
    config_error_ = BuildRuntime();
  } else {
    config_error_ = "no device configuration yet";
  }
}

void DemoDualArmController::OnDeviceConfigsSet() {
  // This hook cannot refuse a configure, so the verdict is latched and
  // on_configure reads it.
  config_error_ = cfg_loaded_ ? BuildRuntime()
                              : std::string("the controller configuration has not been loaded");
  if (!config_error_.empty()) {
    RCLCPP_ERROR(logger_, "[dualarm] %s", config_error_.c_str());
  }
}

double DemoDualArmController::GainUpperBound() const noexcept {
  return 1.0 / GetDefaultDt();
}

// ── Runtime construction ────────────────────────────────────────────────────

void DemoDualArmController::ResetRuntime() noexcept {
  clik_ready_ = false;
  meas_ready_ = false;
  seeded_ = false;
  need_reseed_ = true;
  hand_seeded_ = false;
  num_tasks_ = 0;
  num_posture_groups_ = 0;
  body_dof_ = 0;
  hand_dof_ = 0;
  full_dof_ = 0;
  has_tip_ = false;
  num_fingertips_ = 0;
  use_hand_root_frame_ = false;
  hand_handle_.reset();
  meas_data_.reset();
  target_frame_idx_.fill(-1);
  target_frame_names_.clear();
  task_names_.clear();
  fingertip_names_.clear();
  fingertip_frame_ids_.fill(0);
}

std::string DemoDualArmController::BuildRuntime() {
  ResetRuntime();
  try {
    if (std::string error = SetupModel(); !error.empty()) {
      return error;
    }
    if (std::string error = SetupFrames(); !error.empty()) {
      return error;
    }
    if (std::string error = SetupClik(); !error.empty()) {
      return error;
    }
    if (std::string error = SetupMeasuredKinematics(); !error.empty()) {
      return error;
    }
  } catch (const std::exception& e) {
    clik_ready_ = false;
    return std::string("runtime construction failed: ") + e.what();
  }
  clik_ready_ = true;
  RCLCPP_INFO(logger_,
              "[dualarm] ready: %zu task(s), %zu posture group(s), body %d + hand %d joints, "
              "eta_tau %.2f, braking bound %s",
              num_tasks_, num_posture_groups_, body_dof_, hand_dof_, cfg_.eta_tau,
              cfg_.brake_enabled ? "on" : "off");
  return {};
}

std::string DemoDualArmController::SetupModel() {
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty()) {
    return "no system model: the top-level `urdf:` section is required";
  }
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(*sys_cfg);
  }

  const std::string primary = GetPrimaryDeviceName();
  const std::string secondary = GetSecondaryDeviceName();
  const auto* body_cfg = GetDeviceNameConfig(primary);
  if (body_cfg == nullptr || body_cfg->joint_state_names.empty()) {
    return "body device group '" + primary + "' has no joint_state_names";
  }
  const auto* hand_cfg = secondary.empty() ? nullptr : GetDeviceNameConfig(secondary);
  if (!secondary.empty() && hand_cfg == nullptr) {
    return "hand device group '" + secondary + "' has no device configuration";
  }
  body_joint_names_ = body_cfg->joint_state_names;
  hand_joint_names_ =
      hand_cfg != nullptr ? hand_cfg->joint_state_names : std::vector<std::string>{};
  hand_motor_names_ =
      hand_cfg != nullptr ? hand_cfg->motor_state_names : std::vector<std::string>{};
  if (body_joint_names_.size() > static_cast<std::size_t>(kDualArmMaxBodyDof) ||
      hand_joint_names_.size() > static_cast<std::size_t>(kDualArmMaxHandDof)) {
    return "a device group is wider than this controller's buffers (body " +
           std::to_string(body_joint_names_.size()) + "/" + std::to_string(kDualArmMaxBodyDof) +
           ", hand " + std::to_string(hand_joint_names_.size()) + "/" +
           std::to_string(kDualArmMaxHandDof) + ")";
  }
  {
    std::set<std::string> seen;
    for (const auto* names : {&body_joint_names_, &hand_joint_names_}) {
      for (const auto& name : *names) {
        if (!seen.insert(name).second) {
          return "joint '" + name + "' is listed twice across the device groups";
        }
      }
    }
  }
  body_dof_ = static_cast<int>(body_joint_names_.size());
  hand_dof_ = static_cast<int>(hand_joint_names_.size());
  full_dof_ = body_dof_ + hand_dof_;

  // A fresh cache on every build: frame registration locks at the first
  // Update(), and a rebuilt runtime registers its frames again.
  combined_cache_ = std::make_unique<CombinedModelCache>();
  if (!combined_cache_->InitModel(*builder_, /*contact_frame_ids=*/{}, "[dualarm]", logger_)) {
    return "combined model/cache init failed";
  }
  combined_cache_->BuildReorderMap(&body_joint_names_,
                                   hand_cfg != nullptr ? &hand_joint_names_ : nullptr, full_dof_,
                                   "[dualarm]", logger_);
  if (!combined_cache_->reorder_valid() || !combined_cache_->model()) {
    return "device joint names do not all resolve on the control model";
  }
  const pinocchio::Model& model = *combined_cache_->model();
  // The solve integrates q with v and takes its position box per velocity
  // index, so the two must have one dimension.
  if (model.nq != model.nv) {
    return "control model has nq " + std::to_string(model.nq) + " != nv " +
           std::to_string(model.nv) + " — a model of 1-DoF joints is required";
  }
  // Every joint of the model has to be on a device: the position box is
  // all-or-nothing, and a joint with no device has no limits to put in it —
  // and no measurement to evaluate the model at.
  if (model.nv != full_dof_) {
    return "control model has " + std::to_string(model.nv) + " joints, the device groups drive " +
           std::to_string(full_dof_);
  }
  q_meas_ = Eigen::VectorXd::Zero(model.nq);
  v_meas_ = Eigen::VectorXd::Zero(model.nv);
  q_eval_ = Eigen::VectorXd::Zero(model.nq);
  v_eval_ = Eigen::VectorXd::Zero(model.nv);
  q_posture_des_ = Eigen::VectorXd::Zero(model.nq);
  return {};
}

std::string DemoDualArmController::SetupFrames() {
  const pinocchio::Model& model = *combined_cache_->model();
  auto& cache = combined_cache_->cache();
  const auto* sys_cfg = GetSystemModelConfig();

  // One registration per distinct frame name. Looked up by NAME on the model:
  // frame id 0 is a real frame here (the model's universe), so an id is never
  // used as a "not found" marker.
  std::vector<std::pair<std::string, int>> registered;
  std::string error;
  const auto resolve = [&](const std::string& name, const std::string& what) -> int {
    for (const auto& [known, idx] : registered) {
      if (known == name) {
        return idx;
      }
    }
    if (!model.existFrame(name)) {
      error = what + " '" + name + "' is not a frame of the control model";
      return -1;
    }
    const int idx = cache.RegisterFrame(name, model.getFrameId(name));
    if (idx < 0) {
      error = what + " '" + name + "' could not be registered on the cache";
      return -1;
    }
    registered.emplace_back(name, idx);
    if (sys_cfg != nullptr) {
      for (const auto& extra : sys_cfg->extra_frames) {
        if (extra.name == name && extra.provisional) {
          RCLCPP_WARN(logger_,
                      "[dualarm] %s '%s' is a PROVISIONAL extra frame: its placement is a "
                      "placeholder, so poses commanded through it are too",
                      what.c_str(), name.c_str());
        }
      }
    }
    return idx;
  };

  num_tasks_ = cfg_.tasks.size();
  task_names_.clear();
  for (std::size_t k = 0; k < num_tasks_; ++k) {
    const auto& task_cfg = cfg_.tasks[k];
    TaskRt& task = tasks_[k];
    task = TaskRt{};
    task.frame_idx = resolve(task_cfg.frame, "task '" + task_cfg.name + "': frame");
    if (task.frame_idx < 0) {
      return error;
    }
    task.base_idx = resolve(task_cfg.base_frame, "task '" + task_cfg.name + "': base_frame");
    if (task.base_idx < 0) {
      return error;
    }
    task.frame_fid = model.getFrameId(task_cfg.frame);
    task.base_fid = model.getFrameId(task_cfg.base_frame);
    // Whatever the ingress holds predates this runtime (its frame slot may not
    // exist any more): it is already "seen".
    task.seen_sequence = ingress_[k].box.Load().sequence;
    task.weight = task_cfg.weight;
    task.fb_lin_max = task_cfg.fb_lin_max;
    task.fb_ang_max = task_cfg.fb_ang_max;
    task_names_.push_back(task_cfg.name);
  }

  target_frame_names_ = cfg_.target_frames;
  target_frame_idx_.fill(-1);
  for (std::size_t i = 0; i < target_frame_names_.size(); ++i) {
    target_frame_idx_[i] = resolve(target_frame_names_[i], "clik.target_frames entry");
    if (target_frame_idx_[i] < 0) {
      return error;
    }
  }
  return {};
}

std::string DemoDualArmController::SetupClik() {
  using Clik = rtc::tsid::ClikReferenceGenerator;
  const pinocchio::Model& model = *combined_cache_->model();
  const int nv = model.nv;
  const auto& map = combined_cache_->ext_to_pin_v_map();
  const std::string primary = GetPrimaryDeviceName();
  const auto* body_cfg = GetDeviceNameConfig(primary);
  const auto* hand_cfg = hand_dof_ > 0 ? GetDeviceNameConfig(GetSecondaryDeviceName()) : nullptr;

  // The discrete loop: K·h <= 1 for every gain the YAML carries. Checked here
  // rather than at parse time because the control rate is the manager's.
  const double k_max = GainUpperBound();
  for (const auto& task : cfg_.tasks) {
    if (task.gain_linear > k_max || task.gain_angular > k_max) {
      return "task '" + task.name + "': a gain exceeds 1/dt = " + std::to_string(k_max) + " 1/s";
    }
  }
  for (const auto& group : cfg_.posture_groups) {
    if (group.gain > k_max) {
      return "posture group '" + group.name + "': gain exceeds 1/dt = " + std::to_string(k_max) +
             " 1/s";
    }
  }

  Clik::Config cfg;
  BuildArmHandVelocityIndexSets(body_dof_, full_dof_, nv, map, cfg.arm_v_idx, cfg.hand_v_idx);
  if (static_cast<int>(cfg.arm_v_idx.size()) != body_dof_) {
    return "the body group's joints did not all get a model index";
  }
  cfg.damping_sq = cfg_.damping_sq;
  cfg.w_smooth = cfg_.w_smooth;
  cfg.max_iter = cfg_.max_iter;
  cfg.v_limit = cfg_.joint_velocity_max;
  cfg.evaluate_at_command = true;
  cfg.anchor_drift_max = 0.0;
  cfg.relative_tasks = true;
  cfg.max_frame_tasks = static_cast<int>(num_tasks_);
  // task_v_idx stays empty: the task rows may use the body group's columns.

  // ── posture groups ────────────────────────────────────────────────────────
  num_posture_groups_ = cfg_.posture_groups.size();
  for (const auto& group_cfg : cfg_.posture_groups) {
    Clik::Config::PostureGroup group;
    group.weight = group_cfg.weight;
    for (const auto& joint : group_cfg.joints) {
      const auto it = std::find(body_joint_names_.begin(), body_joint_names_.end(), joint);
      if (it == body_joint_names_.end()) {
        return "posture group '" + group_cfg.name + "': joint '" + joint +
               "' is not a joint of device group '" + primary + "'";
      }
      const auto ext = static_cast<std::size_t>(std::distance(body_joint_names_.begin(), it));
      group.v_idx.push_back(map[ext]);
    }
    cfg.posture_groups.push_back(std::move(group));
  }

  // ── boxes ─────────────────────────────────────────────────────────────────
  if (body_cfg == nullptr || !body_cfg->joint_limits.has_value()) {
    return "body device group '" + primary + "' has no joint_limits";
  }
  if (hand_dof_ > 0 && (hand_cfg == nullptr || !hand_cfg->joint_limits.has_value())) {
    return "hand device group has no joint_limits (the position box covers every model joint)";
  }
  cfg.q_min = Eigen::VectorXd::Zero(nv);
  cfg.q_max = Eigen::VectorXd::Zero(nv);
  cfg.v_limit_per_joint = Eigen::VectorXd::Zero(nv);
  cfg.tau_max = Eigen::VectorXd::Zero(nv);  // hand entries stay 0: the rows are the body's

  const auto fill = [&](const rtc::DeviceNameConfig& dev, int dof, int ext_base,
                        bool is_body) -> std::string {
    const auto& lim = *dev.joint_limits;
    const auto n = static_cast<std::size_t>(dof);
    if (lim.position_lower.size() < n || lim.position_upper.size() < n) {
      return "device group '" + dev.device_name + "': joint_limits position bounds cover " +
             std::to_string(std::min(lim.position_lower.size(), lim.position_upper.size())) +
             " of " + std::to_string(n) + " joints";
    }
    if (is_body && (lim.max_velocity.size() < n || lim.max_torque.size() < n)) {
      return "device group '" + dev.device_name +
             "': joint_limits.max_velocity and max_torque are required for every joint";
    }
    for (std::size_t i = 0; i < n; ++i) {
      const std::string joint = "joint '" + dev.joint_state_names[i] + "'";
      const double lower = lim.position_lower[i];
      const double upper = lim.position_upper[i];
      if (!std::isfinite(lower) || !std::isfinite(upper) || !(lower <= upper)) {
        return joint + ": position limits are not a finite ordered pair";
      }
      const int pv = map[static_cast<std::size_t>(ext_base) + i];
      if (is_body) {
        // The backend clamps to the device limits, so a box that reached them
        // would let the solve return a value the backend then changes. A
        // margin that leaves no interval is a configuration error, not
        // something to shrink quietly.
        const double lo = lower + cfg_.limit_margin;
        const double hi = upper - cfg_.limit_margin;
        if (!(lo < hi)) {
          return joint + ": clik.limit_margin " + std::to_string(cfg_.limit_margin) +
                 " rad leaves no range inside [" + std::to_string(lower) + ", " +
                 std::to_string(upper) + "]";
        }
        const double v_max = lim.max_velocity[i];
        const double tau = lim.max_torque[i];
        if (!std::isfinite(v_max) || !(v_max > 0.0) || !std::isfinite(tau) || !(tau > 0.0)) {
          return joint + ": max_velocity and max_torque must be finite and > 0";
        }
        cfg.q_min[pv] = lo;
        cfg.q_max[pv] = hi;
        cfg.v_limit_per_joint[pv] = std::min(v_max, cfg_.joint_velocity_max);
        // The core takes a margin in (0, 1]; the product is what its rows read.
        cfg.tau_max[pv] = cfg_.eta_tau * tau;
        body_q_min_[i] = lo;
        body_q_max_[i] = hi;
        body_v_max_[i] = cfg.v_limit_per_joint[pv];
      } else {
        // No margin on the hand: its joints rest ON a limit, and a margin
        // would put the resting pose outside the box on every tick.
        cfg.q_min[pv] = lower;
        cfg.q_max[pv] = upper;
        cfg.v_limit_per_joint[pv] = kLockedJointVelocity;
        hand_q_min_[i] = lower;
        hand_q_max_[i] = upper;
      }
    }
    return {};
  };
  if (std::string error = fill(*body_cfg, body_dof_, 0, /*is_body=*/true); !error.empty()) {
    return error;
  }
  if (hand_cfg != nullptr) {
    if (std::string error = fill(*hand_cfg, hand_dof_, body_dof_, /*is_body=*/false);
        !error.empty()) {
      return error;
    }
  }

  cfg.accel_constraint = Clik::AccelConstraint::kDynamic;
  cfg.eta_tau = 1.0;
  cfg.brake_from_torque = cfg_.brake_enabled;
  cfg.brake_margin = cfg_.brake_enabled ? cfg_.brake_margin : 1.0;

  clik_.Init(nv, cfg);  // throws on a config it refuses; BuildRuntime reports it
  // After Init: before it only groups 0 and 1 exist to take a gain.
  for (std::size_t g = 0; g < num_posture_groups_; ++g) {
    if (!clik_.SetPostureGroupGain(static_cast<int>(g), cfg_.posture_groups[g].gain)) {
      return "posture group '" + cfg_.posture_groups[g].name + "' was not created by the solve";
    }
  }
  return {};
}

std::string DemoDualArmController::SetupMeasuredKinematics() {
  const pinocchio::Model& model = *combined_cache_->model();
  const auto* sys_cfg = GetSystemModelConfig();
  const std::string primary = GetPrimaryDeviceName();
  const std::string secondary = GetSecondaryDeviceName();
  const auto* body_cfg = GetDeviceNameConfig(primary);
  const auto* hand_cfg = secondary.empty() ? nullptr : GetDeviceNameConfig(secondary);

  // The frame the published transforms are expressed in: the body group's
  // root link, as the device config or its tree model names it.
  root_link_name_.clear();
  if (body_cfg != nullptr && body_cfg->urdf) {
    root_link_name_ = body_cfg->urdf->root_link;
  }
  if (root_link_name_.empty() && sys_cfg != nullptr) {
    if (const auto* tree = FindTreeModel(*sys_cfg, primary)) {
      root_link_name_ = tree->root_link;
    }
  }
  if (root_link_name_.empty() || !model.existFrame(root_link_name_)) {
    return "body device group '" + primary + "': root link '" + root_link_name_ +
           "' is not a frame of the control model (urdf.tree_models." + primary + ".root_link)";
  }
  root_fid_ = model.getFrameId(root_link_name_);
  meas_data_ = std::make_unique<pinocchio::Data>(model);

  // The hand: mounted on the root link of the hand group's tree model, which
  // is also the "arm tip" the other demo controllers publish.
  const auto* hand_tree = sys_cfg != nullptr ? FindTreeModel(*sys_cfg, secondary) : nullptr;
  tip_link_name_.clear();
  has_tip_ = false;
  if (hand_tree != nullptr && !hand_tree->root_link.empty()) {
    if (!model.existFrame(hand_tree->root_link)) {
      return "hand tree root link '" + hand_tree->root_link +
             "' is not a frame of the control model";
    }
    tip_link_name_ = hand_tree->root_link;
    tip_fid_ = model.getFrameId(tip_link_name_);
    has_tip_ = true;

    hand_handle_ =
        std::make_unique<rtc_urdf_bridge::RtModelHandle>(builder_->GetTreeModel(secondary));
    // Closed-chain FK first: it decides whether the serial handle is read at
    // all (a hand with loop closures is not fully on its serial tree).
    std::vector<std::vector<std::string>> device_joint_names = {body_joint_names_,
                                                                hand_joint_names_};
    const auto wiring = closed_hand_fk_.Configure(
        builder_->GetFullModel(), builder_->GetConstraintModels(),
        builder_->GetClosureActuatedJointIds(), builder_->GetClosureReferenceConfig(),
        device_joint_names, hand_tree->tip_links, hand_tree->root_link);
    LogHandFkWiring(logger_, "[dualarm]", wiring, closed_hand_fk_.missing_joint());
    if (std::string error =
            InstallHandJointOrder(hand_handle_.get(), hand_cfg, closed_hand_fk_.active());
        !error.empty()) {
      return "hand FK wiring failed: " + error;
    }
    hand_root_frame_id_ = hand_handle_->GetFrameId(hand_tree->root_link);
    use_hand_root_frame_ = hand_root_frame_id_ != 0;
    num_fingertips_ = std::min(hand_tree->tip_links.size(), kDualArmMaxFingertips);
    fingertip_names_.assign(
        hand_tree->tip_links.begin(),
        hand_tree->tip_links.begin() + static_cast<std::ptrdiff_t>(num_fingertips_));
    for (std::size_t f = 0; f < num_fingertips_; ++f) {
      fingertip_frame_ids_[f] = hand_handle_->GetFrameId(hand_tree->tip_links[f]);
    }
    hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());
  }
  if (num_fingertips_ + num_tasks_ > static_cast<std::size_t>(rtc::kMaxTaskLinks)) {
    return "fingertips (" + std::to_string(num_fingertips_) + ") + tasks (" +
           std::to_string(num_tasks_) + ") exceed the transform slots (" +
           std::to_string(rtc::kMaxTaskLinks) + ")";
  }
  meas_ready_ = true;
  return {};
}

// ── Target lanes (non-RT producers, RT consumer) ────────────────────────────

void DemoDualArmController::CountGroupGoalReject(const char* reason) noexcept {
  group_goal_rejects_.fetch_add(1, std::memory_order_relaxed);
  if (node_) {
    // Non-RT: both callers run on the node's default callback group.
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "group goal refused: %s", reason);
  }
}

void DemoDualArmController::SetDeviceTarget(int device_idx,
                                            std::span<const double> target) noexcept {
  const int dof = device_idx == kDualArmBodyDeviceIdx   ? body_dof_
                  : device_idx == kDualArmHandDeviceIdx ? hand_dof_
                                                        : 0;
  if (dof <= 0 || static_cast<int>(target.size()) != dof) {
    CountGroupGoalReject("a joint goal must carry exactly the group's joints");
    return;
  }
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

void DemoDualArmController::SetDeviceTaskTarget(int /*device_idx*/,
                                                std::span<const double> /*task6*/) noexcept {
  CountGroupGoalReject(
      "a task goal on a device-group topic — task goals go to <task>/task_goal, one per frame");
}

TaskGoalReject DemoDualArmController::DeliverTaskGoal(
    std::size_t task, const rtc_msgs::msg::RobotTarget& msg) noexcept {
  if (task >= num_tasks_) {
    return TaskGoalReject::kUnknownFrame;
  }
  return ::integrated_bringup::DeliverTaskGoal(msg, target_frame_names_, ActivationGeneration(),
                                               ingress_[task]);
}

// ── E-STOP and fault requests ───────────────────────────────────────────────
//
// Every body is a store. The base warns that an override must not branch on
// the global latch (the manager propagates both directions while it reads
// true); none of these reads it.

void DemoDualArmController::TriggerEstop() noexcept {
  // The flag FIRST: the flag alone stops the solve, so no tick that could have
  // seen the request runs one.
  estop_requested_.store(true, std::memory_order_release);
  // The epoch moves on the trigger as well as on the clear: a trigger → clear
  // pair landing between two ticks leaves the flag false, and without the
  // epoch the tick would carry its command on from before the stop.
  estop_epoch_.fetch_add(1, std::memory_order_release);
}

void DemoDualArmController::ClearEstop() noexcept {
  // The epoch FIRST, the reverse of TriggerEstop. A tick that lands between
  // the two stores then reads "still stopped, and an edge is pending" and
  // re-seeds once, on the tick that reads the flag down. With the flag first
  // it would re-seed on that tick, take a goal, and re-seed again on the next
  // one when the epoch arrived — dropping the goal with no counter moved.
  estop_epoch_.fetch_add(1, std::memory_order_release);
  estop_requested_.store(false, std::memory_order_release);
  // fault_latched_ is NOT cleared here: a global clear must not release a
  // controller fault.
}

bool DemoDualArmController::IsEstopped() const noexcept {
  return estop_requested_.load(std::memory_order_acquire);
}

void DemoDualArmController::ResetFault() noexcept {
  fault_reset_epoch_.fetch_add(1, std::memory_order_release);
}

bool DemoDualArmController::HasLatchedFault() const noexcept {
  return fault_latched_.load(std::memory_order_acquire);
}

}  // namespace integrated_bringup
