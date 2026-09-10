// ── DemoInferenceController: lifecycle ───────────────────────────────────────
// Everything expensive happens here and nowhere else: model load, tensor
// allocation, shape validation and warmup are all inside `engine_->Init()`,
// which is why the tick can be allocation-free.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_inference/inference_types.hpp"

#include <Eigen/Geometry>

#include <algorithm>
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

    // Pass 3: the schema half that needs the device rosters the CM injected
    // between LoadConfig and here.
    ApplyIoSchema(yaml);

    // ── Kinematic model + palm frame ───────────────────────────────────────
    // Only built when a pose feature actually asks for it, so a policy that
    // observes joints and forces alone does not pay for a URDF parse — and,
    // more usefully, does not fail configure on a robot with no system URDF.
    const bool needs_palm =
        std::any_of(feature_kinds_.begin(), feature_kinds_.end(), [](PolicyFeature k) {
          return k == PolicyFeature::kPalmPosition || k == PolicyFeature::kPalmOrientationXyzw;
        });
    if (needs_palm) {
      if (!ConfigurePalmFk()) {
        return CallbackReturn::FAILURE;
      }
    }

    // ── Object pose subscription ───────────────────────────────────────────
    const bool needs_object =
        std::any_of(feature_kinds_.begin(), feature_kinds_.end(), [](PolicyFeature k) {
          return k == PolicyFeature::kObjectPosition || k == PolicyFeature::kObjectOrientationXyzw;
        });
    if (needs_object) {
      if (!node) {
        RCLCPP_ERROR(logger_,
                     "[inference] an object pose feature is configured but there is no node to "
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
      RCLCPP_INFO(logger_, "[inference] object pose lane: %s (match %s '%s', timeout %.3f s)",
                  object_topic_.c_str(), object_match_prefix_ ? "prefix" : "exact",
                  object_frame_match_.c_str(), object_timeout_sec_);
    }

    // ── Hand posture width vs the device that will execute it ──────────────
    const auto secondary = GetSecondaryDeviceName();
    if (secondary.empty() || hand_dof_ <= 0) {
      RCLCPP_ERROR(logger_,
                   "[inference] a second device group (the hand) is required — the posture "
                   "scalar has nothing to drive");
      return CallbackReturn::FAILURE;
    }
    if (posture_open_.size() != static_cast<std::size_t>(hand_dof_)) {
      RCLCPP_ERROR(logger_, "[inference] hand_posture has %zu joints but device '%s' declares %d",
                   posture_open_.size(), secondary.c_str(), hand_dof_);
      return CallbackReturn::FAILURE;
    }

    // Device index 1 is the hand: the tail's bounds are indexed by
    // `topic_config_.groups` position, which is the same order the CM fills
    // `ControllerState::devices`.
    {
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
    if (model_path_.empty()) {
      if (!allow_missing_model_) {
        RCLCPP_ERROR(logger_,
                     "[inference] inference.model_path is empty. Set it, or set "
                     "inference.allow_missing_model: true to bring the wiring up in a "
                     "hold-position mode with no policy");
        return CallbackReturn::FAILURE;
      }
      hold_mode_ = true;
      RCLCPP_WARN(logger_,
                  "[inference] NO POLICY LOADED (allow_missing_model). Every tick holds the "
                  "position latched at activation; this controller commands no motion until "
                  "inference.model_path is set");
    } else {
      rtc::ModelConfig mc;
      mc.model_path = model_path_;
      mc.optimized_model_path = optimized_model_path_;
      // Bridge to the engine's N-tensor ModelConfig while this controller's own
      // schema is still single-input and nameless. Names are left empty, which
      // asks the engine for POSITIONAL binding — the same behaviour this had
      // before. #511 P3/P4 replaces `io_` with a per-tensor, named schema and
      // this collapses into a direct copy.
      mc.inputs = {rtc::TensorSpec{"", io_.input_shape}};
      mc.outputs.reserve(io_.output_shapes.size());
      for (const auto& shape : io_.output_shapes) {
        mc.outputs.push_back({"", shape});
      }
      mc.intra_op_threads = intra_op_threads_;
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
      RCLCPP_INFO(logger_, "[inference] policy loaded: %s (decimation %d, %zu-element input)",
                  model_path_.c_str(), io_.decimation, io_.InputNumel());
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

bool DemoInferenceController::ConfigurePalmFk() {
  namespace rub = rtc_urdf_bridge;
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty()) {
    RCLCPP_ERROR(logger_,
                 "[inference] a palm pose feature is configured but no system URDF is available");
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
    RCLCPP_ERROR(logger_, "[inference] no combined model to resolve '%s' against",
                 palm_link_.c_str());
    return false;
  }
  if (!model->existFrame(palm_link_)) {
    // The failure the shipped default used to produce: `hand_base_link` is a
    // leap-hand link and does not exist on this hand at all.
    RCLCPP_ERROR(logger_, "[inference] palm link '%s' does not exist in the model",
                 palm_link_.c_str());
    return false;
  }
  palm_frame_idx_ =
      combined_cache_.cache().RegisterFrame("inference_palm", model->getFrameId(palm_link_));
  if (palm_frame_idx_ < 0) {
    RCLCPP_ERROR(logger_, "[inference] palm frame registration failed (cache locked)");
    return false;
  }

  // Base frame: without it the cache returns the WORLD-tip pose, and this
  // controller's own `world` is defined by `base_pose_in_world` — mixing the
  // two would double-count the mounting rotation.
  if (!sys_cfg->sub_models.empty()) {
    const auto& root = sys_cfg->sub_models.front().root_link;
    if (!root.empty() && model->existFrame(root)) {
      arm_base_frame_idx_ =
          combined_cache_.cache().RegisterFrame("inference_base", model->getFrameId(root));
    }
  }
  if (arm_base_frame_idx_ < 0) {
    RCLCPP_ERROR(logger_,
                 "[inference] arm root frame could not be registered; a palm pose without an "
                 "explicit base frame would be quoted in the model's own world");
    return false;
  }

  RCLCPP_INFO(logger_, "[inference] palm FK: %s in %s frame", palm_link_.c_str(),
              palm_frame_ == PoseFrame::kWorld ? "world" : "base");
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

  // Deliver in the frame the policy asked for. `object_source_frame_id_` fixes
  // what came in; `object_frame_` fixes what goes out.
  if (sample.valid && object_frame_ == PoseFrame::kBase) {
    const Eigen::Vector3d p_world(sample.position[0], sample.position[1], sample.position[2]);
    const Eigen::Quaterniond q_world(sample.orientation_xyzw[3], sample.orientation_xyzw[0],
                                     sample.orientation_xyzw[1], sample.orientation_xyzw[2]);
    const pinocchio::SE3 world_obj(q_world.normalized().toRotationMatrix(), p_world);
    const pinocchio::SE3 base_obj = world_from_base_.actInv(world_obj);
    const Eigen::Quaterniond q_base(base_obj.rotation());
    sample.position = {base_obj.translation().x(), base_obj.translation().y(),
                       base_obj.translation().z()};
    sample.orientation_xyzw = {q_base.x(), q_base.y(), q_base.z(), q_base.w()};
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
  tick_ = 0;
  inference_count_ = 0;
  last_tick_held_ = true;
  last_scalar_clamped_ = false;
  warned_scalar_range_ = false;
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoInferenceController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  have_action_ = false;
  return RTControllerInterface::on_deactivate(prev);
}

}  // namespace integrated_bringup
