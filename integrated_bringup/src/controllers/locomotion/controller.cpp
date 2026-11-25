// locomotion_controller: construction, the two models and the QP (Pass 1), the
// device layout (Pass 3), and the cross-thread hooks. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
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
// The pose guard may refuse the measured pose this long before the fault
// latches (header): long enough for the robot to come to rest after a switch
// or a clear, short enough that a robot that is down is held, not stood up.
constexpr double kPoseGuardSettleS = 0.5;

[[noreturn]] void Fail(const std::string& what) {
  throw std::invalid_argument(std::string(kContext) + ": " + what);
}

}  // namespace

LocomotionController::LocomotionController(std::string_view urdf_path)
    : logger_(rclcpp::get_logger("integrated_bringup.locomotion_controller")),
      urdf_path_(urdf_path) {}

LocomotionController::~LocomotionController() {
  // Before any member goes: the MPC thread borrows the model handler, the
  // phase manager it owns runs on it (and the command timer writes that phase
  // manager), and it writes the plan sampler.
  StopMpcThread();
}

// ── Pass 1: the YAML and the system model ────────────────────────────────────

void LocomotionController::LoadConfig(const YAML::Node& cfg) {
  StopMpcThread();  // a thread of an earlier configuration runs on what is rebuilt here
  RTControllerInterface::LoadConfig(cfg);  // parses `topics:`
  cfg_loaded_ = false;
  cfg_ = ParseLocomotionControllerConfig(cfg);

  if (const std::string reason =
          ParseLogEntries(cfg,
                          {"rtc_msgs/DeviceStateLog", kJointFeedforwardLogMsgType,
                           kLocomotionDiagLogMsgType, kLocomotionTruthLogMsgType},
                          parsed_log_entries_);
      !reason.empty()) {
    Fail(reason);
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
  // The servo target's constants (header, "The servo target"): the share of the
  // old target kept on each tick, and the step it is propagated by.
  target_dt_ = GetDefaultDt();
  if (!(std::isfinite(target_dt_) && target_dt_ > 0.0)) {
    Fail("the control rate gives no tick period");
  }
  target_wbc_ = cfg_.pd_target_wbc;
  const double tc = cfg_.pd_target_time_constant_s;
  target_keep_ = tc > 0.0 ? std::exp(-target_dt_ / tc) : 0.0;
  // The pose guard's constants, once: the tilt limit as a cosine and the
  // settle time in ticks of the control rate (header).
  cos_tilt_ = std::cos(cfg_.pose_guard_base_tilt_rad);
  guard_settle_ticks_ =
      static_cast<uint32_t>(std::max<long>(1, std::lround(kPoseGuardSettleS / GetDefaultDt())));
  init_timeout_ticks_ = std::numeric_limits<std::uint32_t>::max();
  if (cfg_.estimator.inekf) {
    const auto& inekf = *cfg_.estimator.inekf;
    // The IMU noise is stated per sample of the control tick: one IMU sample a tick.
    const double dt = GetDefaultDt();
    if (!(std::abs(inekf.imu.sample_period_s - dt) <= 1e-9 * dt)) {
      Fail("estimator.inekf.imu.sample_period_s is " + std::to_string(inekf.imu.sample_period_s) +
           " s but the control period is " + std::to_string(dt) + " s");
    }
    if (cfg_.estimator.source == "inekf") {
      init_timeout_ticks_ =
          static_cast<uint32_t>(std::max<long>(1, std::lround(inekf.init.timeout_s / dt)));
    }
  }
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
  loco::BodyModel body;
  try {
    body = loco::BuildBodyModel(*full_model_, *body_tree);
  } catch (const std::invalid_argument& e) {
    Fail(e.what());
  }
  if (body.locked == cfg_.model.hand_groups.empty()) {
    Fail(body.locked ? "the model has joints off the body tree but model.hand_groups is empty"
                     : "model.hand_groups names hands but the model has no joint off the body");
  }
  reduced_model_ = body.reduced;
  body_joint_names_ = std::move(body.joint_names);
  if (body_joint_names_.size() > kLocomotionMaxBodyJoints) {
    Fail("the body has " + std::to_string(body_joint_names_.size()) + " joints, more than " +
         std::to_string(kLocomotionMaxBodyJoints));
  }
  nb_ = body_joint_names_.size();
  reduced_to_full_ = std::move(body.reduced_to_full);
  for (std::size_t j = 0; j < nb_; ++j) {
    joint_lo_[j] = reduced_model_->lowerPositionLimit(static_cast<Eigen::Index>(7 + j));
    joint_hi_[j] = reduced_model_->upperPositionLimit(static_cast<Eigen::Index>(7 + j));
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
  if (cfg_.estimator.inekf) {
    // The estimator's model binding, checked here once (the source builds its own).
    try {
      const loco::ImuFrameKinematics probe(reduced_model_, ContactFrameNames(),
                                           cfg_.estimator.inekf->imu_frame);
      static_cast<void>(probe);
    } catch (const std::invalid_argument& e) {
      Fail(std::string("estimator.inekf: ") + e.what());
    }
  }
  total_mass_ = pinocchio::computeTotalMass(*full_model_);
  weight_ = total_mass_ * full_model_->gravity.linear().norm();
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
    contact_force_stride_ =
        body->sensor_layout ? body->sensor_layout->inference_values_per_group : 0;
    contact_moment_lane_ = body->sensor_layout && body->sensor_layout->has_native_moment;
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
    const auto body_age_ns = static_cast<int64_t>(cfg_.estimator.body_max_age_ms * 1e6);
    const auto hand_age_ns = static_cast<int64_t>(cfg_.estimator.hand_max_age_ms * 1e6);
    // Both sources read the primary IMU site — inekf steps on it, truth records it
    // for an offline replay. Its index is its place in model.imu.sites.
    const auto& imu = cfg_.model.imu;
    const auto imu_site = static_cast<int>(
        std::find(imu.sites.begin(), imu.sites.end(), imu.primary_site) - imu.sites.begin());
    if (cfg_.estimator.source == "inekf") {
      loco::InekfStateSource::Params p;
      p.body_max_age_ns = body_age_ns;
      p.hand_max_age_ns = hand_age_ns;
      p.model_epoch = model_epoch_;
      p.imu_site = imu_site;
      p.config = *cfg_.estimator.inekf;
      source_ = std::make_unique<loco::InekfStateSource>(
          p,
          std::make_unique<loco::ImuFrameKinematics>(reduced_model_, ContactFrameNames(),
                                                     p.config.imu_frame),
          reduced_to_full_);
    } else {
      loco::TruthStateSource::Params p;
      p.body_max_age_ns = body_age_ns;
      p.hand_max_age_ns = hand_age_ns;
      p.num_contacts = static_cast<int>(cfg_.model.contacts.size());
      p.model_epoch = model_epoch_;
      p.imu_site = imu_site;
      source_ = std::make_unique<loco::TruthStateSource>(p);
    }
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

std::vector<std::string> LocomotionController::ContactFrameNames() const {
  std::vector<std::string> frames;
  for (const auto& c : cfg_.model.contacts) {
    frames.push_back(c.frame);
  }
  return frames;
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
//
// The E-STOP word: bit 0 is the flag, the rest counts the edges. An edge is
// one CAS that flips the bit and adds 2, so the tick's single load sees the
// flag and the count of the same instant; a repeated trigger (or clear) is a
// no-op and counts no edge. The count wraps harmlessly: the tick compares the
// word for inequality, never for order.

void LocomotionController::TriggerEstop() noexcept {
  uint32_t v = estop_state_.load(std::memory_order_acquire);
  do {
    if ((v & 1U) != 0) {
      return;
    }
  } while (!estop_state_.compare_exchange_weak(v, (v | 1U) + 2U, std::memory_order_acq_rel,
                                               std::memory_order_acquire));
}

void LocomotionController::ClearEstop() noexcept {
  uint32_t v = estop_state_.load(std::memory_order_acquire);
  do {
    if ((v & 1U) == 0) {
      return;
    }
  } while (!estop_state_.compare_exchange_weak(v, (v & ~1U) + 2U, std::memory_order_acq_rel,
                                               std::memory_order_acquire));
}

bool LocomotionController::IsEstopped() const noexcept {
  return (estop_state_.load(std::memory_order_acquire) & 1U) != 0;
}

void LocomotionController::ResetFault() noexcept {
  fault_reset_epoch_.fetch_add(1, std::memory_order_release);
}

bool LocomotionController::HasLatchedFault() const noexcept {
  return fault_latched_.load(std::memory_order_acquire);
}

}  // namespace integrated_bringup
