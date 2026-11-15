// locomotion_controller: construction, the two models and the QP (Pass 1), the
// device layout (Pass 3), and the cross-thread hooks. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace rub = rtc_urdf_bridge;
namespace loco = locomotion;

namespace {

constexpr const char* kContext = "locomotion_controller";

[[noreturn]] void Fail(const std::string& what) {
  throw std::invalid_argument(std::string(kContext) + ": " + what);
}

}  // namespace

LocomotionController::LocomotionController(std::string_view urdf_path)
    : logger_(rclcpp::get_logger("integrated_bringup.locomotion_controller")),
      urdf_path_(urdf_path) {}

LocomotionController::~LocomotionController() {
  // Before any member goes: the MPC thread borrows the model handler, the
  // phase manager it owns runs on it, and it writes the plan sampler.
  StopMpcThread();
}

// ── Pass 1: the YAML and the system model ────────────────────────────────────

void LocomotionController::LoadConfig(const YAML::Node& cfg) {
  StopMpcThread();  // a thread of an earlier configuration runs on what is rebuilt here
  RTControllerInterface::LoadConfig(cfg);  // parses `topics:`
  cfg_loaded_ = false;
  cfg_ = ParseLocomotionControllerConfig(cfg);

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
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != kJointFeedforwardLogMsgType &&
          e.msg_type != kLocomotionDiagLogMsgType) {
        Fail("unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }

  // The device slots are the `topics:` groups in order: the body, then the hands.
  std::vector<std::string> expected{cfg_.model.body_group};
  expected.insert(expected.end(), cfg_.model.hand_groups.begin(), cfg_.model.hand_groups.end());
  std::vector<std::string> groups;
  for (const auto& [name, unused] : topic_config_.groups) {
    static_cast<void>(unused);
    groups.push_back(name);
  }
  if (groups != expected) {
    std::string want;
    for (const auto& g : expected) {
      want += (want.empty() ? "" : ", ") + g;
    }
    Fail("'topics' must name the device groups [" + want +
         "] in that order (model.body_group, then model.hand_groups)");
  }

  InitModels();
  BuildMpc(cfg);
  cfg_loaded_ = true;
  config_error_ = GetDeviceNameConfig(cfg_.model.body_group) != nullptr
                      ? WireDevices()
                      : "device configs have not arrived yet";
}

void LocomotionController::InitModels() {
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty()) {
    Fail("no system model config — this controller's model is the profile's `urdf:` model");
  }
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rub::PinocchioModelBuilder>(*sys_cfg);
  }
  full_model_ = builder_->GetActuatedModel();
  if (!full_model_) {
    full_model_ = builder_->GetFullModel();
  }
  if (!full_model_ || !rub::IsFreeFlyerRoot(*full_model_)) {
    Fail("the system model has no free-flyer root — a floating-base controller needs one");
  }
  loco::ValidateAgainstModel(cfg_.model, *full_model_, std::string(kContext) + ".model");

  full_joint_order_ = loco::JointOrder(*full_model_);
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(full_model_->njoints);
       ++j) {
    if (full_model_->nqs[j] != 1 || full_model_->nvs[j] != 1) {
      Fail("joint '" + full_model_->names[j] +
           "' is not one-DoF — only revolute / prismatic "
           "actuated joints are supported");
    }
  }

  // The body joints are its tree model's; everything else actuated is a hand's,
  // locked at neutral in the dynamics model.
  const auto body_tree = builder_->GetTreeModel(cfg_.model.body_group);
  if (!body_tree) {
    Fail("the system model declares no `urdf.tree_models." + cfg_.model.body_group + "`");
  }
  body_joint_names_ = loco::JointOrder(*body_tree);
  const std::set<std::string> body_set(body_joint_names_.begin(), body_joint_names_.end());
  std::vector<pinocchio::JointIndex> lock;
  for (const auto& name : full_joint_order_) {
    if (body_set.count(name) == 0) {
      lock.push_back(full_model_->getJointId(name));
    }
  }
  if (lock.empty() != cfg_.model.hand_groups.empty()) {
    Fail(lock.empty() ? "model.hand_groups names hands but the model has no joint off the body"
                      : "the model has joints off the body tree but model.hand_groups is empty");
  }
  reduced_model_ = std::make_shared<const pinocchio::Model>(
      pinocchio::buildReducedModel(*full_model_, lock, pinocchio::neutral(*full_model_)));
  if (loco::JointOrder(*reduced_model_) != body_joint_names_) {
    Fail("the dynamics model (hands locked) does not have the body tree's joint order");
  }
  if (body_joint_names_.size() > kLocomotionMaxBodyJoints) {
    Fail("the body has " + std::to_string(body_joint_names_.size()) + " joints, more than " +
         std::to_string(kLocomotionMaxBodyJoints));
  }
  nb_ = body_joint_names_.size();
  std::map<std::string, int> full_index;
  for (std::size_t a = 0; a < full_joint_order_.size(); ++a) {
    full_index[full_joint_order_[a]] = static_cast<int>(a);
  }
  reduced_to_full_.clear();
  for (const auto& name : body_joint_names_) {
    reduced_to_full_.push_back(full_index.at(name));
  }

  rtc::tsid::LocomotionModelSpec spec;
  contact_names_.clear();
  contact_frames_full_.clear();
  for (const auto& c : cfg_.model.contacts) {
    spec.contacts.push_back({c.name, c.frame, c.dim});
    contact_names_.push_back(c.name);
    contact_frames_full_.push_back(full_model_->getFrameId(c.frame));
  }
  spec.torso_frame = cfg_.model.torso_frame;
  all_contacts_mask_ = (1U << static_cast<unsigned>(contact_names_.size())) - 1U;
  contact_mask_ = all_contacts_mask_;
  if (spec.contacts.size() > LocomotionDiagLogPod::kMaxContacts) {
    Fail("more contacts than the diagnostic log holds (" +
         std::to_string(LocomotionDiagLogPod::kMaxContacts) + ")");
  }

  info_ = rtc::tsid::RobotModelInfo{};
  info_.Build(*reduced_model_, YAML::Node{});
  cache_ = rtc::tsid::PinocchioCache{};
  cache_.Init(reduced_model_,
              rtc::tsid::LocomotionFormulationBuilder::ContactFrameIds(*reduced_model_, spec));
  qp_.Init(*reduced_model_, info_, cache_, spec, cfg_.whole_body_qp, GetDefaultDt());
  contacts_ = qp_.MakeContactState();
  ref_.Init(info_.nq, info_.nv, info_.n_actuated, qp_.Contacts().max_contact_vars);
  qp_state_.q = pinocchio::neutral(*reduced_model_);
  qp_state_.v = Eigen::VectorXd::Zero(reduced_model_->nv);
  full_data_ = std::make_unique<pinocchio::Data>(*full_model_);
  q_full_ = pinocchio::neutral(*full_model_);
  total_mass_ = pinocchio::computeTotalMass(*full_model_);
  ++model_epoch_;
}

// ── Pass 3: the device rosters ───────────────────────────────────────────────

std::string LocomotionController::WireDevices() {
  try {
    const std::string ctx = std::string(kContext) + ".model";
    loco::ValidateDevices(
        cfg_.model, body_joint_names_,
        [this](const std::string& g) { return GetDeviceNameConfig(g); }, ctx);
    const auto* body = GetDeviceNameConfig(cfg_.model.body_group);
    const auto& roster = body->joint_state_names;
    if (cfg_.pd_kp.size() != nb_ || cfg_.pd_kd.size() != nb_ || cfg_.damping_kd.size() != nb_) {
      return "pd_feedforward.kp / kd and fault.damping_kd need one gain per body joint (" +
             std::to_string(nb_) + "), got " + std::to_string(cfg_.pd_kp.size()) + " / " +
             std::to_string(cfg_.pd_kd.size()) + " / " + std::to_string(cfg_.damping_kd.size());
    }
    std::vector<loco::HandRoster> hands;
    hand_joint_names_.clear();
    for (std::size_t h = 0; h < cfg_.model.hand_groups.size(); ++h) {
      const auto* hand = GetDeviceNameConfig(cfg_.model.hand_groups[h]);
      hands.push_back({static_cast<int>(h + 1), hand->joint_state_names});
      hand_joint_names_.push_back(hand->joint_state_names);
    }
    indices_ = loco::BuildDeviceIndices(full_joint_order_, /*body_device=*/0, roster, hands, ctx);
    // The damping clamp: the roster's max_torque, one positive finite value per
    // joint, or none at all (then the damping is unclamped).
    tau_max_.fill(std::numeric_limits<double>::infinity());
    if (body->joint_limits && !body->joint_limits->max_torque.empty()) {
      const auto& lim = body->joint_limits->max_torque;
      if (lim.size() != nb_) {
        return "devices." + cfg_.model.body_group + ".joint_limits.max_torque has " +
               std::to_string(lim.size()) + " values for " + std::to_string(nb_) + " joints";
      }
      for (std::size_t i = 0; i < nb_; ++i) {
        if (!(std::isfinite(lim[i]) && lim[i] > 0.0)) {
          return "devices." + cfg_.model.body_group + ".joint_limits.max_torque[" +
                 std::to_string(i) + "] must be finite and > 0";
        }
        tau_max_[i] = lim[i];
      }
    }
    for (std::size_t i = 0; i < nb_; ++i) {
      kp_[i] = cfg_.pd_kp[i];
      kd_[i] = cfg_.pd_kd[i];
      damping_kd_[i] = cfg_.damping_kd[i];
    }
    loco::TruthStateSource::Params p;
    p.body_max_age_ns = static_cast<int64_t>(cfg_.estimator.body_max_age_ms * 1e6);
    p.hand_max_age_ns = static_cast<int64_t>(cfg_.estimator.hand_max_age_ms * 1e6);
    p.num_contacts = static_cast<int>(cfg_.model.contacts.size());
    p.model_epoch = model_epoch_;
    source_ = std::make_unique<loco::TruthStateSource>(p);
  } catch (const std::exception& e) {
    return e.what();
  }
  return {};
}

void LocomotionController::OnDeviceConfigsSet() {
  if (!cfg_loaded_) {
    config_error_ = "the controller configuration has not been loaded";
    return;
  }
  config_error_ = WireDevices();
}

std::optional<pinocchio::SE3> LocomotionController::FullFramePlacement(
    std::string_view frame) const {
  if (!full_model_ || !full_data_) {
    return std::nullopt;
  }
  const std::string name(frame);
  if (!full_model_->existFrame(name)) {
    return std::nullopt;
  }
  return full_data_->oMf[full_model_->getFrameId(name)];
}

// ── Cross-thread hooks (wait-free; consumed by the next tick) ────────────────

void LocomotionController::TriggerEstop() noexcept {
  estop_requested_.store(true, std::memory_order_release);
  estop_epoch_.fetch_add(1, std::memory_order_acq_rel);
}

void LocomotionController::ClearEstop() noexcept {
  estop_requested_.store(false, std::memory_order_release);
  estop_epoch_.fetch_add(1, std::memory_order_acq_rel);
}

bool LocomotionController::IsEstopped() const noexcept {
  return estop_requested_.load(std::memory_order_acquire);
}

void LocomotionController::ResetFault() noexcept {
  fault_reset_epoch_.fetch_add(1, std::memory_order_release);
}

bool LocomotionController::HasLatchedFault() const noexcept {
  return fault_latched_.load(std::memory_order_acquire);
}

void LocomotionController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

}  // namespace integrated_bringup
