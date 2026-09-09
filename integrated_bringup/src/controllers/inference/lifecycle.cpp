// ── DemoInferenceController: lifecycle ───────────────────────────────────────
// Everything expensive happens here and nowhere else: model load, tensor
// allocation, shape validation and warmup are all inside `engine_->Init()`,
// which is why the tick can be allocation-free.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_inference/inference_types.hpp"

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
                  "[inference] NO POLICY LOADED (allow_missing_model). Every tick will hold the "
                  "measured joint positions; this controller commands nothing until "
                  "inference.model_path is set");
    } else {
      rtc::ModelConfig mc;
      mc.model_path = model_path_;
      mc.optimized_model_path = optimized_model_path_;
      mc.input_shape = io_.input_shape;
      mc.output_shapes = io_.output_shapes;
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
