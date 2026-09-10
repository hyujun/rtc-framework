// ── DemoInferenceController: the RT tick ─────────────────────────────────────
//
// RT path. No allocation, no throw, no logging except throttled one-liners with
// plain scalars. Every buffer is a fixed-size member sized at configure.
//
// Tick shape:
//   1. hold-or-run decision (validity gates, then decimation)
//   2. pack observation → engine->Run() → unpack heads
//   3. blend the hand posture, bound both devices, emit
//
// The order matters: validity is judged BEFORE the decimation counter is
// consulted, so a tick that would have skipped inference anyway still refuses
// to replay a stale action over an unreadable robot.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_controllers/compliance/joint_command_tail.hpp"
#include "rtc_controllers/inference/policy_io.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

namespace integrated_bringup {

namespace {

/// Fingertip force lane geometry. The hand's inference lane packs
/// `inference_values_per_group` floats per sensor group and the mujoco/udp
/// backends both write fx/fy/fz into slots 1..3 of that stride (slot 0 is the
/// contact flag, 4..6 the direction unit vector). Only the magnitude is wanted
/// here, and a magnitude is frame-invariant — which is why this feature needs
/// no reference-frame configuration while the palm pose does.
constexpr int kForceSlotBegin = 1;
constexpr int kForceSlotCount = 3;

}  // namespace

bool DemoInferenceController::PalmPose(std::array<double, 3>& position,
                                       std::array<double, 4>& orientation_xyzw) noexcept {
  if (palm_frame_idx_ < 0 || arm_base_frame_idx_ < 0) {
    return false;
  }
  // T_base_palm. The cache gates this itself: an unregistered frame, a stale
  // reorder map or an un-updated cache all return Identity rather than a stale
  // oMf, and Identity here is indistinguishable from a real pose — so the
  // finite check below is not the whole guard, `reorder_valid()` is.
  if (!combined_cache_.reorder_valid()) {
    return false;
  }
  const pinocchio::SE3 base_palm =
      combined_cache_.ArmTcpPoseFromCache(palm_frame_idx_, arm_base_frame_idx_);
  const pinocchio::SE3 pose =
      (palm_frame_ == PoseFrame::kWorld) ? (world_from_base_ * base_palm) : base_palm;

  const auto& t = pose.translation();
  if (!t.allFinite()) {
    return false;
  }
  // Explicit type, not `auto`: an Eigen expression bound to `auto` here would
  // alias the SE3's storage (RT-5).
  const Eigen::Quaterniond q(pose.rotation());
  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
      !std::isfinite(q.w())) {
    return false;
  }
  position = {t.x(), t.y(), t.z()};
  // Serialisation order is x, y, z, w — the policy's convention, and NOT
  // Eigen's constructor order (w first), which is the easy way to ship a
  // rotation that is wrong in a way every value still looks normal.
  orientation_xyzw = {q.x(), q.y(), q.z(), q.w()};
  return true;
}

bool DemoInferenceController::PackObservation(const ControllerState& state,
                                              std::span<const std::span<float>> bufs) noexcept {
  if (state.num_devices < 2) {
    return false;
  }
  const auto& arm = state.devices[0];
  const auto& hand = state.devices[1];

  // F5: the shared gate that decides whether devices[0]'s positions may be used
  // as this tick's joint state at all. `num_channels` is the wire width and does
  // not answer this — a message can be wide enough while leaving holes behind a
  // reorder map, and those slots still hold the previous tick's values.
  if (!rtc::IsDeviceReadable(arm, arm_dof_) || !rtc::IsDeviceReadable(hand, hand_dof_)) {
    return false;
  }

  for (std::size_t f = 0; f < feature_kinds_.size(); ++f) {
    // One flat walk over every tensor's features. The segment names its own
    // tensor, so the routing is a table lookup rather than a nested loop with
    // its own counters — and an out-of-range tensor is a hold, not a fold onto
    // tensor 0, because folding would write the right values into the wrong
    // model input and leave the right one holding the previous tick.
    const auto& seg = flat_segments_[f];
    if (seg.tensor < 0 || static_cast<std::size_t>(seg.tensor) >= bufs.size()) {
      return false;
    }
    const std::span<float> buf = bufs[static_cast<std::size_t>(seg.tensor)];
    switch (feature_kinds_[f]) {
      case PolicyFeature::kArmPosition: {
        for (int i = 0; i < arm_dof_; ++i) {
          scratch_measured_[static_cast<std::size_t>(i)] =
              arm.positions[static_cast<std::size_t>(i)];
        }
        if (!rtc::inference::PackSegment(
                buf, seg,
                std::span<const double>(scratch_measured_.data(),
                                        static_cast<std::size_t>(arm_dof_)))) {
          return false;
        }
        break;
      }
      case PolicyFeature::kHandPosition: {
        for (int i = 0; i < hand_dof_; ++i) {
          scratch_measured_[static_cast<std::size_t>(i)] =
              hand.positions[static_cast<std::size_t>(i)];
        }
        if (!rtc::inference::PackSegment(
                buf, seg,
                std::span<const double>(scratch_measured_.data(),
                                        static_cast<std::size_t>(hand_dof_)))) {
          return false;
        }
        break;
      }
      case PolicyFeature::kFingertipForceNorm: {
        // A stale group contributes 0, by the decision recorded in the spec:
        // "no contact" and "no reading" are the same observation to this
        // policy, and holding the last force instead would let a dropped lane
        // keep reporting a grasp that ended.
        const int stride = fingertip_stride_;
        for (int g = 0; g < num_fingertips_; ++g) {
          double norm = 0.0;
          const bool fresh =
              (g < rtc::kMaxSensorGroups) && hand.inference_enable[static_cast<std::size_t>(g)];
          if (fresh && stride >= kForceSlotBegin + kForceSlotCount) {
            double sum_sq = 0.0;
            for (int c = 0; c < kForceSlotCount; ++c) {
              const int idx = (g * stride) + kForceSlotBegin + c;
              if (idx < 0 || idx >= rtc::kMaxInferenceValues) {
                sum_sq = 0.0;
                break;
              }
              const double v =
                  static_cast<double>(hand.inference_data[static_cast<std::size_t>(idx)]);
              sum_sq += v * v;
            }
            norm = std::isfinite(sum_sq) ? std::sqrt(sum_sq) : 0.0;
          }
          scratch_force_norm_[static_cast<std::size_t>(g)] = norm;
        }
        if (!rtc::inference::PackSegment(
                buf, seg,
                std::span<const double>(scratch_force_norm_.data(),
                                        static_cast<std::size_t>(num_fingertips_)))) {
          return false;
        }
        break;
      }
      case PolicyFeature::kPalmPosition:
      case PolicyFeature::kPalmOrientationXyzw: {
        std::array<double, 3> p{};
        std::array<double, 4> q{};
        if (!PalmPose(p, q)) {
          return false;
        }
        const bool want_position = (feature_kinds_[f] == PolicyFeature::kPalmPosition);
        const std::span<const double> src = want_position
                                                ? std::span<const double>(p.data(), p.size())
                                                : std::span<const double>(q.data(), q.size());
        if (!rtc::inference::PackSegment(buf, seg, src)) {
          return false;
        }
        break;
      }
      case PolicyFeature::kObjectPosition:
      case PolicyFeature::kObjectOrientationXyzw: {
        // `object_valid_this_tick_` was decided once at the top of Compute so
        // both halves of the pose come from the SAME sample — reading the
        // SeqLock per feature could straddle a callback and pair a position
        // with the next message's orientation.
        if (!object_valid_this_tick_) {
          return false;
        }
        const bool want_position = (feature_kinds_[f] == PolicyFeature::kObjectPosition);
        const std::span<const double> src =
            want_position ? std::span<const double>(object_this_tick_.position.data(), 3)
                          : std::span<const double>(object_this_tick_.orientation_xyzw.data(), 4);
        if (!rtc::inference::PackSegment(buf, seg, src)) {
          return false;
        }
        break;
      }
    }
  }

  // Per tensor, after every segment of that tensor is in place: the lane is
  // indexed against its OWN tensor, and applying it to a partially packed
  // buffer would normalise this tick's values together with whatever the
  // untouched elements still held.
  for (std::size_t t = 0; t < bufs.size() && t < io_.inputs.size(); ++t) {
    rtc::inference::ApplyAffine(bufs[t], io_.inputs[t].offset, io_.inputs[t].scale);
  }
  return true;
}

void DemoInferenceController::HoldPosition(const ControllerState& state,
                                           ControllerOutput& out) noexcept {
  // Every hold path in Compute() funnels through here, which is why the hold
  // clock lives here rather than being re-armed at each of the six early
  // returns. Arming a flag instead of resetting on the spot keeps the hold path
  // from touching the engine at all (#511 D-3).
  hold_elapsed_sec_ += state.dt;
  if (reset_after_hold_sec_ >= 0.0 && hold_elapsed_sec_ >= reset_after_hold_sec_) {
    recurrent_reset_pending_ = true;
  }

  // Latch on entry. See the member declaration for why commanding the measured
  // position every tick is not a hold: it is a zero-stiffness follower, and the
  // arm sags.
  if (!hold_latched_) {
    if (have_readable_) {
      hold_arm_ = last_readable_arm_;
      hold_hand_ = last_readable_hand_;
    } else {
      // Nothing readable has ever arrived — the only position available is
      // whatever this tick reports. Better than commanding zero.
      for (int i = 0; i < arm_dof_ && i < rtc::kMaxDeviceChannels; ++i) {
        hold_arm_[static_cast<std::size_t>(i)] =
            state.devices[0].positions[static_cast<std::size_t>(i)];
      }
      for (int i = 0; i < hand_dof_ && i < rtc::kMaxDeviceChannels; ++i) {
        hold_hand_[static_cast<std::size_t>(i)] =
            state.devices[1].positions[static_cast<std::size_t>(i)];
      }
    }
    hold_latched_ = true;
  }

  out.num_devices = std::min(state.num_devices, ControllerOutput::kMaxDevices);
  for (int d = 0; d < out.num_devices; ++d) {
    const auto& dev = state.devices[static_cast<std::size_t>(d)];
    auto& dst = out.devices[static_cast<std::size_t>(d)];
    dst.num_channels = dev.num_channels;
    const double* src = (d == 0) ? hold_arm_.data() : (d == 1) ? hold_hand_.data() : nullptr;
    const int latched = (d == 0) ? arm_dof_ : (d == 1) ? hand_dof_ : 0;
    for (int c = 0; c < dev.num_channels && c < rtc::kMaxDeviceChannels; ++c) {
      // A channel past the latched width (a third device, or a wire wider than
      // the roster) falls back to its measured value: there is no latched
      // number for it, and inventing one would be worse than following.
      const double q = (src != nullptr && c < latched) ? src[static_cast<std::size_t>(c)]
                                                       : dev.positions[static_cast<std::size_t>(c)];
      dst.commands[static_cast<std::size_t>(c)] = q;
      dst.target_positions[static_cast<std::size_t>(c)] = q;
      dst.target_velocities[static_cast<std::size_t>(c)] = 0.0;
    }
  }
}

void DemoInferenceController::BoundDeviceCommand(int device_idx, std::span<const double> measured,
                                                 std::span<double> command, double dt) noexcept {
  // The §7.3 tail integrates a VELOCITY from a base, so the absolute target the
  // policy produced is expressed as the velocity that would reach it in one
  // tick. What comes back is the same value after the mandated
  // clamp-then-rate-rebound order, which is why this binding does not do either
  // step itself.
  const auto n = command.size();
  std::array<double, kMaxArmDof + kMaxHandDof> q{};
  std::array<double, kMaxArmDof + kMaxHandDof> dq{};
  const auto capped = std::min(n, q.size());
  for (std::size_t i = 0; i < capped; ++i) {
    q[i] = measured[i];
    dq[i] = (dt > 0.0) ? ((command[i] - measured[i]) / dt) : 0.0;
  }

  rtc::compliance::JointCommandBounds bounds;
  const auto idx = static_cast<std::size_t>(device_idx);
  if (idx < device_position_lower_.size()) {
    bounds.lower = device_position_lower_[idx];
    bounds.upper = device_position_upper_[idx];
    bounds.max_velocity = device_max_velocity_[idx];
  }
  bounds.margin = joint_limit_margin_;

  static_cast<void>(rtc::compliance::IntegrateAndBoundJointCommand(
      std::span<double>(q.data(), capped), std::span<double>(dq.data(), capped), capped, dt,
      bounds));

  for (std::size_t i = 0; i < capped; ++i) {
    command[i] = q[i];
  }
}

ControllerOutput DemoInferenceController::Compute(const ControllerState& state) noexcept {
  ControllerOutput out;
  out.command_type = command_type_;
  out.valid = true;

  const double dt = state.dt;
  ++tick_;

  // ── Object pose snapshot ──────────────────────────────────────────────────
  // Read ONCE per tick, before anything consumes it, so position and
  // orientation are guaranteed to be halves of the same message.
  //
  // Age is accumulated in `dt` rather than read off a clock: no `now()` on the
  // RT path, and it stays on the simulation's own time axis, which is the axis
  // the policy's cadence is defined in.
  {
    const ObjectPoseSample sample = object_pose_lock_.Load();
    if (sample.sequence != last_object_seq_seen_) {
      last_object_seq_seen_ = sample.sequence;
      object_age_sec_ = 0.0;
      object_this_tick_ = sample;
      object_ever_seen_ = true;
    } else {
      object_age_sec_ += (dt > 0.0) ? dt : 0.0;
    }
    object_valid_this_tick_ =
        object_ever_seen_ && object_this_tick_.valid && (object_age_sec_ <= object_timeout_sec_);
  }

  if (object_frame_mismatch_.load(std::memory_order_relaxed) && !warned_object_frame_) {
    warned_object_frame_ = true;
    RCLCPP_WARN(logger_,
                "[inference] object pose transforms arrived in an unexpected frame and were "
                "ignored — check inference.object_pose.source_frame_id against the publisher");
  }

  // One-shot, argument-free: within the RT logging carve-out, and the operator
  // otherwise has no way to distinguish "this controller ignores joint goals"
  // from "this controller is broken".
  if (external_target_seen_.load(std::memory_order_relaxed) && !warned_external_target_) {
    warned_external_target_ = true;
    RCLCPP_WARN(logger_,
                "[inference] an external joint goal arrived and was ignored — the policy owns "
                "the command stream while this controller is active");
  }

  // ── Hold gates ────────────────────────────────────────────────────────────
  // Anything that makes the observation or the action untrustworthy lands the
  // whole robot on its measured position. Never one device only: the policy
  // reasons about arm and hand together, so half an action is not a smaller
  // version of the right thing.
  const bool structurally_ready =
      !hold_mode_ && engine_ && arm_dof_ > 0 && hand_dof_ > 0 && state.num_devices >= 2;

  if (!structurally_ready) {
    HoldPosition(state, out);
    last_tick_held_ = true;
    return out;
  }

  const bool devices_readable = rtc::IsDeviceReadable(state.devices[0], arm_dof_) &&
                                rtc::IsDeviceReadable(state.devices[1], hand_dof_);

  // FK is read from the cache in PackObservation, so the scatter+Update has to
  // have run for THIS tick first. ExtractFullState carries the same F5 gate
  // internally, so an unreadable arm leaves the cache holding the previous
  // configuration rather than a half-written one.
  if (devices_readable) {
    for (int i = 0; i < arm_dof_; ++i) {
      last_readable_arm_[static_cast<std::size_t>(i)] =
          state.devices[0].positions[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < hand_dof_; ++i) {
      last_readable_hand_[static_cast<std::size_t>(i)] =
          state.devices[1].positions[static_cast<std::size_t>(i)];
    }
    have_readable_ = true;
  }

  if (devices_readable && palm_frame_idx_ >= 0) {
    combined_cache_.ExtractFullState(state, arm_dof_, hand_dof_);
    combined_cache_.Update();
  }
  if (!devices_readable) {
    // Judged before decimation on purpose: replaying the held action over a
    // robot whose state we cannot read would keep driving toward a target
    // derived from a configuration that may no longer be true.
    have_action_ = false;
    HoldPosition(state, out);
    last_tick_held_ = true;
    return out;
  }

  // ── Decimation ────────────────────────────────────────────────────────────
  // Phase against the ABSOLUTE tick counter, not a countdown: after a hold the
  // policy re-runs immediately (there is no action to replay) and then falls
  // back onto the same grid, so a transient bad tick cannot permanently shift
  // the cadence the policy was trained at. `1 % decim` makes decimation == 1
  // mean "every tick" without a special case.
  const auto decim = static_cast<std::uint64_t>((io_.decimation > 0) ? io_.decimation : 1);
  const bool run_policy = !have_action_ || ((tick_ % decim) == (1U % decim));

  if (run_policy) {
    // Every declared input tensor, checked before anything is written. ONNX
    // Runtime allocates all of them, so a tensor this binding failed to fill
    // would not be absent — it would be whatever that allocation held, observed
    // by the policy as if it were this tick's robot.
    //
    // Stack array, fixed capacity, bounded by `kMaxInputTensors` at configure:
    // no allocation on the tick (RT-1).
    std::array<std::span<float>, kMaxInputTensors> in_bufs{};
    const std::size_t n_tensors = io_.inputs.size();
    bool tensors_ok = (n_tensors <= in_bufs.size()) &&
                      (static_cast<std::size_t>(engine_->num_inputs(0)) == n_tensors);
    for (std::size_t t = 0; tensors_ok && t < n_tensors; ++t) {
      float* buf = engine_->input_buffer(0, static_cast<int>(t));
      const std::size_t size = engine_->input_size(0, static_cast<int>(t));
      if (buf == nullptr || size != io_.inputs[t].Numel()) {
        tensors_ok = false;
        break;
      }
      in_bufs[t] = std::span<float>(buf, size);
    }
    if (!tensors_ok) {
      have_action_ = false;
      HoldPosition(state, out);
      last_tick_held_ = true;
      return out;
    }

    // ── Recurrent state: reset before the observation goes in ──────────────
    // Armed by activation or by a hold that outlasted `reset_after_hold_sec_`.
    // Zeroing here rather than after the run means the very step that resumes
    // already runs on the fresh state instead of one step later.
    if (recurrent_reset_pending_) {
      for (const auto& link : io_.recurrent_links) {
        const auto t = static_cast<std::size_t>(link.input_tensor);
        if (t < n_tensors) {
          std::fill(in_bufs[t].begin(), in_bufs[t].end(), 0.0F);
        }
      }
      recurrent_reset_pending_ = false;
    }

    if (!PackObservation(state, std::span<const std::span<float>>(in_bufs.data(), n_tensors))) {
      have_action_ = false;
      HoldPosition(state, out);
      last_tick_held_ = true;
      return out;
    }

    ++inference_count_;
    if (!engine_->Run()) {
      // A failed Run() leaves the output buffer holding the PREVIOUS result, so
      // the return value is the only thing that distinguishes a fresh action
      // from a stale one. Dropping the held action here is what stops that
      // stale buffer from being replayed for the next `decimation` ticks.
      have_action_ = false;
      HoldPosition(state, out);
      last_tick_held_ = true;
      return out;
    }

    // ── Unpack: arm target ─────────────────────────────────────────────────
    const auto& arm_slice = io_.output_features[static_cast<std::size_t>(arm_target_idx_)].slice;
    const auto& hand_slice = io_.output_features[static_cast<std::size_t>(hand_command_idx_)].slice;

    bool ok = arm_slice.count == arm_dof_;
    if (ok) {
      ok = rtc::inference::UnpackSlice(
          std::span<double>(scratch_head_.data(), static_cast<std::size_t>(arm_slice.count)),
          arm_slice, engine_->output_buffer(0, arm_slice.tensor),
          engine_->output_size(0, arm_slice.tensor));
    }
    if (ok) {
      for (int i = 0; i < arm_dof_; ++i) {
        const double v = scratch_head_[static_cast<std::size_t>(i)];
        if (!std::isfinite(v)) {
          ok = false;
          break;
        }
        arm_action_[static_cast<std::size_t>(i)] = v;
      }
    }

    // ── Unpack: hand command ───────────────────────────────────────────────
    // Which of the two shapes arrives is a configure-time fact, so the branch
    // is on a resolved enum and not on anything the tick has to discover.
    if (ok && hand_role_ == PolicyOutputRole::kJointTarget) {
      // Same width guard the arm path carries, and for a sharper reason here:
      // `scratch_head_` was just written with the ARM's targets, and
      // `UnpackSlice` cannot catch a short slice because the span handed to it
      // is sized by that same `count` — its bounds check compares the slice
      // against itself. A `count` below `hand_dof_` would therefore leave the
      // tail of this read holding arm joint angles, finite and inside the
      // hand's limits, with nothing downstream to object. `parameters.cpp`
      // refuses that config at configure time; this keeps the two lanes
      // symmetric rather than resting the hand's correctness on a check the
      // arm did not consider sufficient for itself.
      ok = hand_slice.count == hand_dof_;
      if (ok) {
        ok = rtc::inference::UnpackSlice(
            std::span<double>(scratch_head_.data(), static_cast<std::size_t>(hand_slice.count)),
            hand_slice, engine_->output_buffer(0, hand_slice.tensor),
            engine_->output_size(0, hand_slice.tensor));
      }
      for (int i = 0; ok && i < hand_dof_; ++i) {
        const double v = scratch_head_[static_cast<std::size_t>(i)];
        if (!std::isfinite(v)) {
          ok = false;
          break;
        }
        hand_action_[static_cast<std::size_t>(i)] = v;
      }
    } else if (ok) {
      double scalar = 0.0;
      ok = rtc::inference::UnpackSlice(std::span<double>(&scalar, 1), hand_slice,
                                       engine_->output_buffer(0, hand_slice.tensor),
                                       engine_->output_size(0, hand_slice.tensor));
      if (ok) {
        const auto blend = rtc::inference::BlendPosture(
            std::span<double>(hand_action_.data(), static_cast<std::size_t>(hand_dof_)),
            posture_open_, posture_close_, scalar);
        ok = blend.valid;
        last_scalar_clamped_ = blend.clamped;
        if (blend.clamped && !warned_scalar_range_) {
          // One-shot, plain scalars only: an out-of-range scalar means the
          // model's normalisation and this YAML disagree, and that is worth
          // exactly one line rather than one per tick.
          warned_scalar_range_ = true;
          RCLCPP_WARN(logger_,
                      "[inference] posture scalar %.3f is outside [0,1] and was clamped — the "
                      "policy's output normalisation likely disagrees with hand_posture",
                      scalar);
        }
      }
    }

    if (!ok) {
      have_action_ = false;
      HoldPosition(state, out);
      last_tick_held_ = true;
      return out;
    }
    // ── Recurrent feedback ─────────────────────────────────────────────────
    // Deliberately AFTER the `ok` gate, which already means "the whole action
    // was accepted". A partial action is not a smaller version of the right
    // thing, and neither is the state that produced it — advancing the state on
    // a rejected step would keep the policy's memory moving while the robot
    // stood still (#511 C-5, no new branch needed).
    //
    // Engine buffer → engine buffer: the state never passes through this
    // controller's scratch, so a hidden layer of any width costs it zero bytes.
    for (const auto& link : io_.recurrent_links) {
      const auto in_t = static_cast<std::size_t>(link.input_tensor);
      if (in_t >= n_tensors) {
        continue;
      }
      if (!rtc::inference::CopyFiniteChecked(in_bufs[in_t],
                                             engine_->output_buffer(0, link.output_tensor),
                                             engine_->output_size(0, link.output_tensor)) &&
          !warned_state_reset_) {
        // One-shot, plain integer: a non-finite state is a fault, and the
        // alternative to zeroing it is a permanent silent hold (C-4).
        warned_state_reset_ = true;
        RCLCPP_WARN(logger_,
                    "[inference] recurrent state tensor %d was not finite and has been RESET to "
                    "zero. Freezing it instead would have made every later step non-finite with "
                    "no way back short of re-activation",
                    link.input_tensor);
      }
    }

    have_action_ = true;
    hold_elapsed_sec_ = 0.0;
    hold_latched_ = false;  // a fresh action supersedes the latch
  }

  // ── Emit ──────────────────────────────────────────────────────────────────
  out.num_devices = std::min(state.num_devices, ControllerOutput::kMaxDevices);

  std::array<double, kMaxArmDof> arm_cmd{};
  std::array<double, kMaxHandDof> hand_cmd{};
  std::array<double, kMaxArmDof> arm_meas{};
  std::array<double, kMaxHandDof> hand_meas{};
  for (int i = 0; i < arm_dof_; ++i) {
    arm_cmd[static_cast<std::size_t>(i)] = arm_action_[static_cast<std::size_t>(i)];
    arm_meas[static_cast<std::size_t>(i)] = state.devices[0].positions[static_cast<std::size_t>(i)];
  }
  for (int i = 0; i < hand_dof_; ++i) {
    hand_cmd[static_cast<std::size_t>(i)] = hand_action_[static_cast<std::size_t>(i)];
    hand_meas[static_cast<std::size_t>(i)] =
        state.devices[1].positions[static_cast<std::size_t>(i)];
  }

  BoundDeviceCommand(0,
                     std::span<const double>(arm_meas.data(), static_cast<std::size_t>(arm_dof_)),
                     std::span<double>(arm_cmd.data(), static_cast<std::size_t>(arm_dof_)), dt);
  BoundDeviceCommand(1,
                     std::span<const double>(hand_meas.data(), static_cast<std::size_t>(hand_dof_)),
                     std::span<double>(hand_cmd.data(), static_cast<std::size_t>(hand_dof_)), dt);

  auto& arm_out = out.devices[0];
  arm_out.num_channels = arm_dof_;
  for (int i = 0; i < arm_dof_; ++i) {
    arm_out.commands[static_cast<std::size_t>(i)] = arm_cmd[static_cast<std::size_t>(i)];
    arm_out.target_positions[static_cast<std::size_t>(i)] =
        arm_action_[static_cast<std::size_t>(i)];
  }
  auto& hand_out = out.devices[1];
  hand_out.num_channels = hand_dof_;
  for (int i = 0; i < hand_dof_; ++i) {
    hand_out.commands[static_cast<std::size_t>(i)] = hand_cmd[static_cast<std::size_t>(i)];
    hand_out.target_positions[static_cast<std::size_t>(i)] =
        hand_action_[static_cast<std::size_t>(i)];
  }

  last_tick_held_ = false;
  return out;
}

}  // namespace integrated_bringup
