// ── DemoInferenceController: lifecycle ───────────────────────────────────────
// Everything expensive happens here and nowhere else: model load, tensor
// allocation, shape validation and warmup are all inside `engine_->Init()`,
// and every frame the tick reads is resolved to an index here — which is why
// the tick can be allocation-free.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_inference/inference_types.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace integrated_bringup {

namespace {

/// Check a posture against a device's position band.
///
/// This is not defensive padding — it is the check that catches the specific
/// trap this robot ships with. `ur5e_bt_coordinator/config/poses_p1b.yaml`
/// declares "close" postures that are POSITIVE on six joints whose upper bound
/// is 0.0 (those joints flex negative), so copying them here would produce a
/// "closed" hand that the command clamp quietly returns to open. A posture
/// outside the band cannot be detected later: by the time the tail has clamped
/// it, the value looks like a perfectly ordinary command.
bool PostureWithinLimits(const std::vector<double>& posture, const std::vector<double>& lower,
                         const std::vector<double>& upper, std::string& offender) {
  for (std::size_t i = 0; i < posture.size(); ++i) {
    if (!std::isfinite(posture[i])) {
      offender = "joint " + std::to_string(i) + " is not finite";
      return false;
    }
    if (i < lower.size() && posture[i] < lower[i]) {
      offender = "joint " + std::to_string(i) + " = " + std::to_string(posture[i]) +
                 " is below its lower limit " + std::to_string(lower[i]);
      return false;
    }
    if (i < upper.size() && posture[i] > upper[i]) {
      offender = "joint " + std::to_string(i) + " = " + std::to_string(posture[i]) +
                 " is above its upper limit " + std::to_string(upper[i]);
      return false;
    }
  }
  return true;
}

}  // namespace

RTControllerInterface::CallbackReturn DemoInferenceController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    if (!engine_) {
      RCLCPP_ERROR(logger_, "[inference] no inference engine was injected");
      return CallbackReturn::FAILURE;
    }

    // Checked BEFORE the schema, because the schema's own role table matches
    // `device:` against these two group names — with no secondary group it
    // would report "device 'p1b' is not one of ('ur5e', '')", which names the
    // symptom rather than the missing group.
    const auto secondary = GetSecondaryDeviceName();
    if (secondary.empty() || hand_dof_ <= 0) {
      RCLCPP_ERROR(logger_,
                   "[inference] a second device group (the hand) is required — this controller "
                   "drives an arm and a hand together");
      return CallbackReturn::FAILURE;
    }

    // Pass 3: the schema half that needs the device rosters the CM injected
    // between LoadConfig and here.
    ApplyIoSchema(yaml);

    // ── Kinematic model, link table, frames ────────────────────────────────
    // Built only when a pose is observed (or the object pose has to be moved
    // between frames), so a policy that observes joints and forces alone does
    // not pay for a URDF parse — and does not fail configure on a robot with no
    // system URDF.
    if (!ConfigureKinematics()) {
      return CallbackReturn::FAILURE;
    }

    // ── Object pose subscription ───────────────────────────────────────────
    if (wants_object_) {
      if (!node) {
        RCLCPP_ERROR(logger_,
                     "[inference] the object pose is observed but there is no node to "
                     "subscribe with");
        return CallbackReturn::FAILURE;
      }
      // ARCH-6: depth 1. This is a "latest wins" lane — an older object pose is
      // never more useful than the newest one, and queueing would hand the tick
      // a backlog to walk.
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.best_effort();
      object_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
          object_topic_, qos,
          [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { OnObjectTransforms(*msg); });
      RCLCPP_INFO(logger_,
                  "[inference] object pose lane: %s (match %s '%s', %s = URDF '%s' → '%s', "
                  "timeout %.3f s)",
                  object_topic_.c_str(), object_match_prefix_ ? "prefix" : "exact",
                  object_frame_match_.c_str(), object_source_frame_id_.c_str(),
                  object_source_frame_link_.c_str(), policy_frame_.c_str(), object_timeout_sec_);
    }

    // ── Hand posture width vs the device that will execute it ──────────────
    // Only when a posture_scalar role asked for the blend (#511 D-9). A policy
    // that commands the hand joints directly never reads these two lists, and
    // failing configure over them was a bring-up refusal with nothing wrong.
    if (hand_role_ == PolicyOutputRole::kPostureScalar) {
      if (posture_open_.size() != static_cast<std::size_t>(hand_dof_)) {
        RCLCPP_ERROR(logger_, "[inference] hand_posture has %zu joints but device '%s' declares %d",
                     posture_open_.size(), secondary.c_str(), hand_dof_);
        return CallbackReturn::FAILURE;
      }

      // Device index 1 is the hand: the tail's bounds are indexed by
      // `topic_config_.groups` position, which is the same order the CM fills
      // `ControllerState::devices`.
      std::string offender;
      if (!PostureWithinLimits(posture_open_, device_position_lower_[1], device_position_upper_[1],
                               offender)) {
        RCLCPP_ERROR(logger_, "[inference] hand_posture.open is outside the joint limits: %s",
                     offender.c_str());
        return CallbackReturn::FAILURE;
      }
      if (!PostureWithinLimits(posture_close_, device_position_lower_[1], device_position_upper_[1],
                               offender)) {
        RCLCPP_ERROR(logger_,
                     "[inference] hand_posture.close is outside the joint limits: %s. NOTE: on "
                     "this hand several joints flex NEGATIVE (upper limit 0), so a posture "
                     "copied from a degrees-based pose file will look closed and clamp open",
                     offender.c_str());
        return CallbackReturn::FAILURE;
      }
    }

    // ── Inference engine ───────────────────────────────────────────────────
    // The path is expanded here and not in LoadConfig so an unset variable can
    // still become the documented hold mode: a checkout that ships the config
    // but not the policy file is the ordinary state of every machine but the
    // one the policy was copied to (D4 — the model stays out of the repo).
    std::string missing_var;
    const bool expanded =
        model_path_raw_.empty() || ExpandModelPath(model_path_raw_, model_path_, missing_var);
    if (model_path_raw_.empty()) {
      model_path_.clear();
    }
    if (!expanded || model_path_.empty()) {
      if (!allow_missing_model_) {
        if (!expanded) {
          RCLCPP_ERROR(logger_,
                       "[inference] inference.model_path '%s' needs ${%s}, which is not set. "
                       "Export it, or set inference.allow_missing_model: true to bring the wiring "
                       "up in a hold-position mode with no policy",
                       model_path_raw_.c_str(), missing_var.c_str());
        } else {
          RCLCPP_ERROR(logger_,
                       "[inference] inference.model_path is empty. Set it, or set "
                       "inference.allow_missing_model: true to bring the wiring up in a "
                       "hold-position mode with no policy");
        }
        return CallbackReturn::FAILURE;
      }
      hold_mode_ = true;
      model_path_.clear();
      if (!expanded) {
        RCLCPP_WARN(logger_,
                    "[inference] NO POLICY LOADED: inference.model_path '%s' needs ${%s}, which "
                    "is not set (allow_missing_model). Every tick holds the position latched at "
                    "activation; this controller commands no motion until the variable is set",
                    model_path_raw_.c_str(), missing_var.c_str());
      } else {
        RCLCPP_WARN(logger_,
                    "[inference] NO POLICY LOADED (allow_missing_model). Every tick holds the "
                    "position latched at activation; this controller commands no motion until "
                    "inference.model_path is set");
      }
    } else {
      rtc::ModelConfig mc;
      mc.model_path = model_path_;
      mc.optimized_model_path = optimized_model_path_;
      // Straight copy — the schema layer already speaks the engine's shape. Both
      // sides carry the .onnx tensor NAMES (the schema makes them mandatory,
      // #511 D-1), which is what puts the engine in by-name binding: a re-export
      // that reordered two tensors of the same shape then fails to load instead
      // of loading cleanly and computing the wrong thing.
      mc.inputs.reserve(io_.inputs.size());
      for (const auto& tensor : io_.inputs) {
        mc.inputs.push_back({tensor.name, tensor.shape});
      }
      mc.outputs.reserve(io_.outputs.size());
      for (const auto& tensor : io_.outputs) {
        mc.outputs.push_back({tensor.name, tensor.shape});
      }
      mc.intra_op_threads = intra_op_threads_;
      // A path that is SET but does not load is a failure even under
      // allow_missing_model: the operator pointed at a file, and a typo in that
      // path must not look like "no policy on this machine".
      engine_->Init(mc);

      // The gate that makes a missing ONNX Runtime loud. The stub engine's
      // Init() is a no-op that does not throw and its Run() returns false, so
      // without this the controller would configure cleanly and then hold
      // position forever with nothing in the log to say why.
      if (!engine_->is_initialized()) {
        RCLCPP_ERROR(logger_,
                     "[inference] the inference engine did not initialise. This is what a build "
                     "without ONNX Runtime looks like (the stub engine's Init() is a silent "
                     "no-op); check that rtc_inference found onnxruntime at build time");
        return CallbackReturn::FAILURE;
      }
      hold_mode_ = false;
      RCLCPP_INFO(logger_,
                  "[inference] policy loaded: %s (decimation %d, %zu input / %zu output "
                  "tensor(s), %zu recurrent link(s))",
                  model_path_.c_str(), io_.decimation, io_.inputs.size(), io_.outputs.size(),
                  io_.recurrent_links.size());
    }
  } catch (const std::exception& e) {
    // Includes the engine's own shape validation, which throws when the .onnx
    // on disk does not match the shapes the YAML declares — the single most
    // useful failure this controller can produce after a retrain.
    RCLCPP_ERROR(logger_, "[inference] configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

bool DemoInferenceController::ConfigureKinematics() {
  namespace rub = rtc_urdf_bridge;
  has_closed_links_ = false;
  closed_fk_fresh_ = false;
  policy_frame_idx_ = -1;
  hand_root_idx_ = -1;
  pf_from_src_ = pinocchio::SE3::Identity();

  // Nothing observed in space — and an object lane whose messages are already
  // in the policy frame needs no model to say so.
  const bool object_needs_model = wants_object_ && !object_source_frame_link_.empty() &&
                                  object_source_frame_link_ != policy_frame_;
  if (links_.empty() && !object_needs_model) {
    return true;
  }

  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty()) {
    RCLCPP_ERROR(logger_,
                 "[inference] a pose is observed in '%s' but no system URDF is available to "
                 "resolve it",
                 policy_frame_.c_str());
    return false;
  }

  // Prefer the builder RtControllerNode already parsed — the URDF is parsed
  // once per bring-up, not once per controller.
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rub::PinocchioModelBuilder>(*sys_cfg);
  }

  if (!combined_cache_.InitModel(*builder_, /*contact_frame_ids=*/{}, "[inference]", logger_)) {
    RCLCPP_ERROR(logger_, "[inference] combined model init failed");
    return false;
  }
  const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
  const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
  combined_cache_.BuildReorderMap(arm_cfg ? &arm_cfg->joint_state_names : nullptr,
                                  hand_cfg ? &hand_cfg->joint_state_names : nullptr,
                                  arm_dof_ + hand_dof_, "[inference]", logger_);

  const auto& model = combined_cache_.model();
  if (!model) {
    RCLCPP_ERROR(logger_, "[inference] no combined model to resolve frames against");
    return false;
  }
  if (!model->existFrame(policy_frame_)) {
    RCLCPP_ERROR(logger_, "[inference] inference.policy_frame '%s' does not exist in the model",
                 policy_frame_.c_str());
    return false;
  }

  // ── Object source frame → policy frame ─────────────────────────────────
  // Static by requirement: the transform is applied in the subscription
  // callback, off the tick, so a source frame that moves with the joints (a
  // wrist camera) would be applied at a configuration the tick never agreed
  // to. Two frames on the same joint are rigidly attached, and their relative
  // placement is then a model constant — no FK, no configuration.
  if (object_needs_model) {
    if (!model->existFrame(object_source_frame_link_)) {
      RCLCPP_ERROR(logger_,
                   "[inference] inference.object_pose.source_frame_link '%s' does not exist in "
                   "the model",
                   object_source_frame_link_.c_str());
      return false;
    }
    const auto& pf = model->frames[model->getFrameId(policy_frame_)];
    const auto& src = model->frames[model->getFrameId(object_source_frame_link_)];
    if (pf.parentJoint != src.parentJoint) {
      RCLCPP_ERROR(logger_,
                   "[inference] object_pose.source_frame_link '%s' and policy_frame '%s' are not "
                   "rigidly attached (a joint moves between them) — the object pose is "
                   "transformed off the tick, so the transform between them must be constant",
                   object_source_frame_link_.c_str(), policy_frame_.c_str());
      return false;
    }
    pf_from_src_ = pf.placement.actInv(src.placement);
  }

  if (links_.empty()) {
    return true;
  }

  policy_frame_idx_ = combined_cache_.cache().RegisterFrame("inference_policy_frame",
                                                            model->getFrameId(policy_frame_));
  if (policy_frame_idx_ < 0) {
    RCLCPP_ERROR(logger_, "[inference] policy frame registration failed (cache locked)");
    return false;
  }

  // ── Where each link's pose comes from ──────────────────────────────────
  // Decided from the TOPOLOGY, per link. The cache's model locks every
  // loop-passive joint at zero, so a link downstream of one — a fingertip on a
  // linkage finger — gets a pose from it that is finite, smooth and wrong by
  // centimetres. Those go through the closed-chain projection; everything else
  // (a palm, an arm link) stays on the cache, where it is exact.
  const auto full = builder_->GetFullModel();
  const auto& constraints = builder_->GetConstraintModels();
  const auto& actuated = builder_->GetClosureActuatedJointIds();
  std::vector<std::string> closed_names;
  for (std::size_t i = 0; i < links_.size(); ++i) {
    auto& slot = links_[i];
    if (!model->existFrame(slot.name)) {
      RCLCPP_ERROR(logger_, "[inference] observed link '%s' does not exist in the model",
                   slot.name.c_str());
      return false;
    }
    const bool downstream =
        !constraints.empty() && full && full->existFrame(slot.name) &&
        rub::IsFrameDownstreamOfLoop(*full, actuated, full->getFrameId(slot.name));
    if (downstream) {
      if (closed_names.size() >= ClosedChainHandFk::kMaxFingertips) {
        RCLCPP_ERROR(logger_,
                     "[inference] more than %zu observed links are downstream of a loop; the "
                     "closed-chain hand FK serves at most that many",
                     ClosedChainHandFk::kMaxFingertips);
        return false;
      }
      slot.closed_tip = static_cast<int>(closed_names.size());
      closed_names.push_back(slot.name);
      continue;
    }
    slot.cache_idx = combined_cache_.cache().RegisterFrame("inference_link_" + std::to_string(i),
                                                           model->getFrameId(slot.name));
    if (slot.cache_idx < 0) {
      RCLCPP_ERROR(logger_, "[inference] frame registration for '%s' failed (cache locked)",
                   slot.name.c_str());
      return false;
    }
  }

  if (!closed_names.empty()) {
    // The projection reports fingertips relative to the hand root, and the
    // cache supplies the hand root relative to the policy frame — the root is
    // upstream of every loop, so its cache pose is exact.
    std::string hand_root;
    const auto secondary = GetSecondaryDeviceName();
    for (const auto& tm : sys_cfg->tree_models) {
      if (tm.name == secondary) {
        hand_root = tm.root_link;
        break;
      }
    }
    if (hand_root.empty() || !model->existFrame(hand_root)) {
      RCLCPP_ERROR(logger_,
                   "[inference] links downstream of a loop need the hand root "
                   "(urdf.tree_models.%s.root_link), which is %s",
                   secondary.c_str(), hand_root.empty() ? "not declared" : "not in the model");
      return false;
    }
    hand_root_idx_ =
        combined_cache_.cache().RegisterFrame("inference_hand_root", model->getFrameId(hand_root));
    if (hand_root_idx_ < 0) {
      RCLCPP_ERROR(logger_, "[inference] hand root registration failed (cache locked)");
      return false;
    }

    std::vector<std::vector<std::string>> dev_names;
    dev_names.push_back(arm_cfg ? arm_cfg->joint_state_names : std::vector<std::string>{});
    dev_names.push_back(hand_cfg ? hand_cfg->joint_state_names : std::vector<std::string>{});
    const auto res =
        closed_fk_.Configure(full, constraints, actuated, builder_->GetClosureReferenceConfig(),
                             dev_names, closed_names, hand_root, kClosureErrorThreshold);
    LogHandFkWiring(logger_, "[inference]", res, closed_fk_.missing_joint());
    if (res != HandFkWiringResult::kActive) {
      // No serial fallback for these links: the serial pose is exactly the
      // wrong answer this branch exists to avoid.
      RCLCPP_ERROR(logger_,
                   "[inference] %zu observed link(s) are downstream of a loop but the closed-chain "
                   "hand FK did not activate — their serial pose would be wrong",
                   closed_names.size());
      return false;
    }
    has_closed_links_ = true;
  }

  RCLCPP_INFO(logger_, "[inference] %zu observed link(s) in '%s' (%zu closed-chain)", links_.size(),
              policy_frame_.c_str(), closed_names.size());
  return true;
}

void DemoInferenceController::OnObjectTransforms(const tf2_msgs::msg::TFMessage& msg) noexcept {
  // Non-RT (controller LifecycleNode default callback group). Allocation and
  // string comparison are fine here; the tick only reads the SeqLock.
  ObjectPoseSample sample;
  int matches = 0;
  bool frame_mismatch = false;
  for (const auto& tf : msg.transforms) {
    if (tf.header.frame_id != object_source_frame_id_) {
      // Checked, not assumed. A publisher quoting the same message type in a
      // different frame is the real-hardware failure mode this key exists for,
      // and every value it produces would still look like a valid pose.
      frame_mismatch = true;
      continue;
    }
    const auto& child = tf.child_frame_id;
    const bool hit = object_match_prefix_ ? (child.rfind(object_frame_match_, 0) == 0)
                                          : (child == object_frame_match_);
    if (!hit) {
      continue;
    }
    ++matches;
    sample.position = {tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z};
    sample.orientation_xyzw = {tf.transform.rotation.x, tf.transform.rotation.y,
                               tf.transform.rotation.z, tf.transform.rotation.w};
  }

  // Exactly one, or nothing. The sim publishes every non-parked free body in a
  // single message, so "take the first match" would silently start tracking a
  // different object the day the scene gains one — and every downstream value
  // would stay finite and plausible.
  sample.valid = (matches == 1);

  // Deliver in the policy frame. `object_source_frame_id_` fixed what came in;
  // `pf_from_src_` (identity when the two frames are the same) moves it.
  if (sample.valid) {
    const Eigen::Vector3d p_src(sample.position[0], sample.position[1], sample.position[2]);
    const Eigen::Quaterniond q_src(sample.orientation_xyzw[3], sample.orientation_xyzw[0],
                                   sample.orientation_xyzw[1], sample.orientation_xyzw[2]);
    const pinocchio::SE3 src_obj(q_src.normalized().toRotationMatrix(), p_src);
    const pinocchio::SE3 pf_obj = pf_from_src_ * src_obj;
    const Eigen::Quaterniond q_pf(pf_obj.rotation());
    sample.position = {pf_obj.translation().x(), pf_obj.translation().y(),
                       pf_obj.translation().z()};
    sample.orientation_xyzw = {q_pf.x(), q_pf.y(), q_pf.z(), q_pf.w()};
  }

  if (frame_mismatch) {
    object_frame_mismatch_.store(true, std::memory_order_relaxed);
  }
  sample.sequence = ++object_seq_written_;
  object_pose_lock_.Store(sample);
}

RTControllerInterface::CallbackReturn DemoInferenceController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // Base first: it bumps the activation generation and resets target
  // initialisation, which is what stops anything stale from the inactive
  // window being applied on the first tick back.
  const auto ret = RTControllerInterface::on_activate(prev);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // A policy action from before the gap describes a robot state that no longer
  // exists, so the first tick after activation must hold and re-evaluate
  // rather than resume.
  have_action_ = false;
  hold_latched_ = false;
  have_readable_ = false;
  cmd_base_valid_ = false;  // the first command steps from the measured position
  tick_ = 0;
  inference_count_ = 0;
  last_tick_held_ = true;
  last_scalar_clamped_ = false;
  warned_scalar_range_ = false;
  // Recurrent state too, and UNCONDITIONALLY — `reset_after_hold_sec` governs
  // holds during a run, not the gap across a deactivation. Whatever the state
  // described, the robot has been out of this controller's hands since (#511
  // D-3). Applied on the first evaluation, because the state lives in the
  // engine's buffers and those are only safe to touch on the tick.
  hold_elapsed_sec_ = 0.0;
  recurrent_reset_pending_ = true;
  warned_state_reset_ = false;
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoInferenceController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  have_action_ = false;
  return RTControllerInterface::on_deactivate(prev);
}

}  // namespace integrated_bringup
