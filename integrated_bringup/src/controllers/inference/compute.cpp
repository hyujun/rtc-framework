// ── DemoInferenceController: the RT tick ─────────────────────────────────────
//
// RT path. No allocation, no throw, no logging except one-shot lines with plain
// scalars. Every buffer is a fixed-size member sized at configure.
//
// Tick shape (`ComputeCommand`):
//   1. hold-or-run decision (validity gates, then decimation)
//   2. seed/reset recurrent state → pack observation → engine->Run() → unpack
//   3. bound both devices against the previous command, emit
// `Compute` wraps it and pushes the tick's CSV rows once, after whichever
// early return was taken.
//
// The order matters: validity is judged BEFORE the decimation counter is
// consulted, so a tick that would have skipped inference anyway still refuses
// to replay a stale action over an unreadable robot.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
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

double DemoInferenceController::GroupForceNorm(const rtc::DeviceState& hand, int g) const noexcept {
  // A stale group contributes 0, by the decision recorded in the spec: "no
  // contact" and "no reading" are the same observation to this policy, and
  // holding the last force instead would let a dropped lane keep reporting a
  // grasp that ended.
  const int stride = fingertip_stride_;
  if (g < 0 || g >= rtc::kMaxSensorGroups || !hand.inference_enable[static_cast<std::size_t>(g)] ||
      stride < kForceSlotBegin + kForceSlotCount) {
    return 0.0;
  }
  double sum_sq = 0.0;
  for (int c = 0; c < kForceSlotCount; ++c) {
    const int idx = (g * stride) + kForceSlotBegin + c;
    if (idx < 0 || idx >= rtc::kMaxInferenceValues) {
      return 0.0;
    }
    const auto v = static_cast<double>(hand.inference_data[static_cast<std::size_t>(idx)]);
    sum_sq += v * v;
  }
  return std::isfinite(sum_sq) ? std::sqrt(sum_sq) : 0.0;
}

bool DemoInferenceController::ComputeLinkPoses() noexcept {
  // The cache gates itself — an unregistered frame, a stale reorder map or an
  // un-updated cache all answer Identity, which is indistinguishable from a
  // real pose — so `reorder_valid()` is part of this guard, not the finite
  // check below.
  pack_failure_ = InferenceHoldReason::kLinkPose;
  if (policy_frame_idx_ < 0 || !combined_cache_.reorder_valid()) {
    return false;
  }
  pinocchio::SE3 pf_hand_root = pinocchio::SE3::Identity();
  if (has_closed_links_) {
    // The wrapper keeps serving its last good fingertip pose through a held or
    // unconverged tick. That is the right answer for a TF display and the wrong
    // one for an observation: the policy would see the fingers where they were.
    if (!closed_fk_fresh_ || hand_root_idx_ < 0) {
      pack_failure_ = InferenceHoldReason::kClosedChain;
      return false;
    }
    pf_hand_root = combined_cache_.ArmTcpPoseFromCache(hand_root_idx_, policy_frame_idx_);
  }
  for (std::size_t i = 0; i < links_.size() && i < link_pos_.size(); ++i) {
    const auto& slot = links_[i];
    pinocchio::SE3 pose = pinocchio::SE3::Identity();
    if (slot.closed_tip >= 0) {
      pinocchio::SE3 root_tip = pinocchio::SE3::Identity();
      if (!closed_fk_.GetFingertipHandRootPose(static_cast<std::size_t>(slot.closed_tip),
                                               root_tip)) {
        pack_failure_ = InferenceHoldReason::kClosedChain;
        return false;
      }
      pose = pf_hand_root * root_tip;
    } else if (slot.cache_idx >= 0) {
      pose = combined_cache_.ArmTcpPoseFromCache(slot.cache_idx, policy_frame_idx_);
    } else {
      return false;
    }
    // Explicit types, not `auto`: an Eigen expression bound to `auto` would
    // alias the SE3's storage (RT-5).
    const Eigen::Vector3d t = pose.translation();
    const Eigen::Quaterniond q(pose.rotation());
    if (!t.allFinite() || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        !std::isfinite(q.w())) {
      return false;
    }
    link_pos_[i] = {t.x(), t.y(), t.z()};
    // Serialisation order is x, y, z, w — the policy's convention, and NOT
    // Eigen's constructor order (w first), which is the easy way to ship a
    // rotation that is wrong in a way every value still looks normal.
    link_quat_[i] = {q.x(), q.y(), q.z(), q.w()};
  }
  return true;
}

bool DemoInferenceController::ComputeReachPhase(const ControllerState& state,
                                                double& phase) noexcept {
  if (!reach_enabled_ || !object_valid_this_tick_ || reach_tips_ <= 0 || state.num_devices < 2) {
    return false;
  }
  const auto& hand = state.devices[1];
  const auto n = static_cast<std::size_t>(reach_tips_);
  std::array<double, std::size_t{3} * kMaxReachTips> tips{};
  std::array<double, kMaxReachTips> forces{};
  for (std::size_t i = 0; i < n; ++i) {
    const int slot = reach_link_slot_[i];
    if (slot < 0 || static_cast<std::size_t>(slot) >= links_.size()) {
      return false;
    }
    const auto& p = link_pos_[static_cast<std::size_t>(slot)];
    tips[3 * i] = p[0];
    tips[(3 * i) + 1] = p[1];
    tips[(3 * i) + 2] = p[2];
    forces[i] = GroupForceNorm(hand, reach_force_group_[i]);
  }

  // Advance a COPY: the trigger counts policy steps whose action was taken, so
  // it is committed alongside the recurrent feedback, not here.
  rtc::inference::ReachHoldState next = reach_state_;
  const bool grasped = rtc::inference::IsGraspedByForce(std::span<const double>(forces.data(), n),
                                                        reach_.force_threshold, reach_.min_fingers);
  const bool hold =
      rtc::inference::UpdateReachHold(next, grasped, reach_.hold_on_steps, reach_.hold_off_steps);
  const double distance = rtc::inference::MeanTipDistance(
      std::span<const double>(tips.data(), 3 * n),
      std::span<const double, 3>(object_this_tick_.position),
      std::span<const double, 4>(object_this_tick_.orientation_xyzw),
      std::span<const double>(reach_contacts_.data(), 3 * n));
  const double gate =
      rtc::inference::ReachGate(rtc::inference::ReachProximity(distance, reach_.tip_std), hold);
  if (!std::isfinite(gate)) {
    return false;
  }
  reach_state_pending_ = next;
  reach_phase_pending_ = gate;
  tip_distance_pending_ = distance;
  reach_pending_ = true;
  phase = gate;
  return true;
}

bool DemoInferenceController::ExtractFeature(PolicyFeature kind, int arg,
                                             const ControllerState& state,
                                             std::span<double> out) noexcept {
  const auto& arm = state.devices[0];
  const auto& hand = state.devices[1];
  const auto arm_n = static_cast<std::size_t>(arm_dof_);
  const auto hand_n = static_cast<std::size_t>(hand_dof_);
  switch (kind) {
    case PolicyFeature::kArmPosition:
      if (out.size() != arm_n) {
        return false;
      }
      for (std::size_t i = 0; i < arm_n; ++i) {
        out[i] = (arm_sign_[i] * arm.positions[i]) + arm_offset_[i];
      }
      return true;
    case PolicyFeature::kHandPosition:
      if (out.size() != hand_n) {
        return false;
      }
      for (std::size_t i = 0; i < hand_n; ++i) {
        out[i] = (hand_sign_[i] * hand.positions[i]) + hand_offset_[i];
      }
      return true;
    case PolicyFeature::kArmVelocity:
      // The position gate says nothing about this lane (#446), and a driver
      // that omits `velocity` leaves it zero-initialised — a robot at rest, as
      // far as the policy could tell.
      if (out.size() != arm_n || !rtc::IsLaneReadable(arm, rtc::StateLane::kVelocity, arm_dof_)) {
        return false;
      }
      for (std::size_t i = 0; i < arm_n; ++i) {
        out[i] = arm_sign_[i] * arm.velocities[i];
      }
      return true;
    case PolicyFeature::kHandVelocity:
      if (out.size() != hand_n ||
          !rtc::IsLaneReadable(hand, rtc::StateLane::kVelocity, hand_dof_)) {
        return false;
      }
      for (std::size_t i = 0; i < hand_n; ++i) {
        out[i] = hand_sign_[i] * hand.velocities[i];
      }
      return true;
    case PolicyFeature::kFingertipForceNorm: {
      const auto n = static_cast<std::size_t>(num_fingertips_);
      if (out.size() != n) {
        return false;
      }
      for (std::size_t g = 0; g < n; ++g) {
        out[g] = GroupForceNorm(hand, static_cast<int>(g));
      }
      return true;
    }
    case PolicyFeature::kGroupForceNorm:
      if (out.size() != 1) {
        return false;
      }
      out[0] = GroupForceNorm(hand, arg);
      return true;
    case PolicyFeature::kLinkPosition:
    case PolicyFeature::kLinkOrientationXyzw: {
      if (arg < 0 || static_cast<std::size_t>(arg) >= links_.size()) {
        return false;
      }
      const auto a = static_cast<std::size_t>(arg);
      if (kind == PolicyFeature::kLinkPosition) {
        if (out.size() != 3) {
          return false;
        }
        std::copy(link_pos_[a].begin(), link_pos_[a].end(), out.begin());
      } else {
        if (out.size() != 4) {
          return false;
        }
        std::copy(link_quat_[a].begin(), link_quat_[a].end(), out.begin());
      }
      return true;
    }
    case PolicyFeature::kReachPhase: {
      double phase = 0.0;
      if (out.size() != 1 || !ComputeReachPhase(state, phase)) {
        return false;
      }
      out[0] = phase;
      return true;
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
      if (kind == PolicyFeature::kObjectPosition) {
        if (out.size() != 3) {
          return false;
        }
        std::copy(object_this_tick_.position.begin(), object_this_tick_.position.end(),
                  out.begin());
      } else {
        if (out.size() != 4) {
          return false;
        }
        std::copy(object_this_tick_.orientation_xyzw.begin(),
                  object_this_tick_.orientation_xyzw.end(), out.begin());
      }
      return true;
    }
  }
  return false;
}

bool DemoInferenceController::PackObservation(const ControllerState& state,
                                              std::span<const std::span<float>> bufs) noexcept {
  pack_failure_ = InferenceHoldReason::kUnreadable;
  if (state.num_devices < 2) {
    return false;
  }
  // F5: the shared gate that decides whether the positions may be used as this
  // tick's joint state at all. `num_channels` is the wire width and does not
  // answer this — a message can be wide enough while leaving holes behind a
  // reorder map, and those slots still hold the previous tick's values.
  if (!rtc::IsDeviceReadable(state.devices[0], arm_dof_) ||
      !rtc::IsDeviceReadable(state.devices[1], hand_dof_)) {
    return false;
  }
  reach_pending_ = false;  // this attempt's trigger advance, if any, starts fresh

  // ── 1. Filler ─────────────────────────────────────────────────────────────
  // Before the features, so an element no feature covers holds its declared
  // value rather than whatever the allocation or an earlier attempt left there.
  // Recurrent tensors are skipped: their contents ARE the state.
  for (std::size_t t = 0; t < bufs.size() && t < io_.inputs.size(); ++t) {
    const auto& spec = io_.inputs[t];
    if (spec.recurrent) {
      continue;
    }
    rtc::inference::ApplyFill(bufs[t], spec.constant ? std::span<const float>(spec.values)
                                                     : std::span<const float>(spec.fill));
  }

  // ── 2. Link poses, once per evaluation ────────────────────────────────────
  if (!links_.empty() && !ComputeLinkPoses()) {
    return false;
  }

  // ── 3. Features ───────────────────────────────────────────────────────────
  for (std::size_t f = 0; f < feature_kinds_.size(); ++f) {
    // One flat walk over every tensor's features. The segment names its own
    // tensor, so the routing is a table lookup rather than a nested loop with
    // its own counters — and an out-of-range tensor is a hold, not a fold onto
    // tensor 0, because folding would write the right values into the wrong
    // model input and leave the right one holding the previous tick.
    const auto& seg = flat_segments_[f];
    pack_failure_ = InferenceHoldReason::kFeature;
    if (seg.tensor < 0 || static_cast<std::size_t>(seg.tensor) >= bufs.size() || seg.count <= 0 ||
        static_cast<std::size_t>(seg.count) > scratch_measured_.size()) {
      return false;
    }
    const std::span<double> values(scratch_measured_.data(), static_cast<std::size_t>(seg.count));
    const PolicyFeature kind = feature_kinds_[f];
    if (!ExtractFeature(kind, feature_args_[f], state, values)) {
      // The object lane is the one input with its own freshness rule, and the
      // reach gate depends on it, so a stale object is named as such rather
      // than folded into "some feature".
      const bool object_fed = kind == PolicyFeature::kObjectPosition ||
                              kind == PolicyFeature::kObjectOrientationXyzw ||
                              kind == PolicyFeature::kReachPhase;
      if (object_fed && !object_valid_this_tick_) {
        pack_failure_ = InferenceHoldReason::kObject;
      }
      return false;
    }
    if (!rtc::inference::PackSegment(bufs[static_cast<std::size_t>(seg.tensor)], seg, values)) {
      return false;
    }
  }

  // ── 4. Affine, per tensor, after every segment of that tensor is in place ──
  // Recurrent and constant tensors carry no lane (the parser refuses one), so
  // this is a no-op for them.
  for (std::size_t t = 0; t < bufs.size() && t < io_.inputs.size(); ++t) {
    rtc::inference::ApplyAffine(bufs[t], io_.inputs[t].offset, io_.inputs[t].scale);
  }
  return true;
}

void DemoInferenceController::HoldPosition(const ControllerState& state, ControllerOutput& out,
                                           InferenceHoldReason reason) noexcept {
  // Every hold path in Compute() funnels through here, which is why the hold
  // clock lives here rather than being re-armed at each early return. Arming a
  // flag instead of resetting on the spot keeps the hold path from touching
  // the engine at all (#511 D-3). The same funnel is what lets every hold carry
  // a reason: there is no early return that could skip naming one.
  last_hold_reason_ = reason;
  const auto r = static_cast<std::size_t>(reason);
  if (r < hold_counts_.size()) {
    ++hold_counts_[r];
  }
  hold_elapsed_sec_ += state.dt;
  if (reset_after_hold_sec_ >= 0.0 && hold_elapsed_sec_ >= reset_after_hold_sec_) {
    recurrent_reset_pending_ = true;
  }
  // The command stream restarts from the measured position when a policy action
  // is next accepted — see `cmd_base_valid_`.
  cmd_base_valid_ = false;

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
      dst.goal_positions[static_cast<std::size_t>(c)] = q;
      dst.target_positions[static_cast<std::size_t>(c)] = q;
      dst.target_velocities[static_cast<std::size_t>(c)] = 0.0;
    }
  }
}

void DemoInferenceController::BoundDeviceCommand(int device_idx, std::span<const double> base,
                                                 std::span<double> command, double dt) noexcept {
  // The §7.3 tail integrates a VELOCITY from a base, so the absolute target the
  // policy produced is expressed as the velocity that would reach it in one
  // tick. What comes back is the same value after the mandated
  // clamp-then-rate-rebound order, which is why this binding does not do either
  // step itself.
  const auto n = command.size();
  std::array<double, kMaxArmDof + kMaxHandDof> q{};
  std::array<double, kMaxArmDof + kMaxHandDof> dq{};
  const auto capped = std::min({n, base.size(), q.size()});
  for (std::size_t i = 0; i < capped; ++i) {
    q[i] = base[i];
    dq[i] = (dt > 0.0) ? ((command[i] - base[i]) / dt) : 0.0;
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

ControllerOutput DemoInferenceController::ComputeCommand(const ControllerState& state) noexcept {
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
  // whole robot on its latched position. Never one device only: the policy
  // reasons about arm and hand together, so half an action is not a smaller
  // version of the right thing.
  const bool structurally_ready =
      !hold_mode_ && engine_ && arm_dof_ > 0 && hand_dof_ > 0 && state.num_devices >= 2;

  if (!structurally_ready) {
    HoldPosition(state, out, InferenceHoldReason::kNotReady);
    last_tick_held_ = true;
    return out;
  }

  const bool devices_readable = rtc::IsDeviceReadable(state.devices[0], arm_dof_) &&
                                rtc::IsDeviceReadable(state.devices[1], hand_dof_);

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

  // Kinematics on EVERY readable tick, not only on policy steps. The cache is
  // read in PackObservation, so it has to hold this tick's configuration; the
  // closed-chain projection additionally clamps its seed increment per update
  // (NUM-5), so running it only every N-th tick would turn ordinary motion into
  // a walk-in and hold the policy for no reason. ExtractFullState carries the
  // same F5 gate internally.
  closed_fk_fresh_ = false;
  if (devices_readable && !links_.empty()) {
    combined_cache_.ExtractFullState(state, arm_dof_, hand_dof_);
    combined_cache_.Update();
    if (has_closed_links_) {
      closed_fk_.Update(state);
      const auto st = closed_fk_.status();
      closed_fk_fresh_ = !st.held && !st.singular && std::isfinite(st.closure_error) &&
                         st.closure_error < kClosureErrorThreshold;
      closed_fk_status_ = st;
      closed_fk_ran_ = true;
      // Publish only. A projection that never converges has to be said out
      // loud, but not from here (RT-3) — the non-RT poll reads this.
      closed_chain_held_ticks_.store(st.held_ticks, std::memory_order_relaxed);
    }
  }
  if (!devices_readable) {
    // Judged before decimation on purpose: replaying the held action over a
    // robot whose state we cannot read would keep driving toward a target
    // derived from a configuration that may no longer be true.
    have_action_ = false;
    HoldPosition(state, out, InferenceHoldReason::kUnreadable);
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
      HoldPosition(state, out, InferenceHoldReason::kTensorMismatch);
      last_tick_held_ = true;
      return out;
    }

    // ── Recurrent state: reset before the observation goes in ──────────────
    // Armed by activation or by a hold that outlasted `reset_after_hold_sec_`,
    // and re-applied on every attempt until an action is accepted, so a seed
    // read off the joints describes the step that actually runs. A seeded
    // tensor starts from its feature (an integrator whose state is the current
    // joint position must not start from zero — that would command every joint
    // toward zero); the rest start from zero. The reach trigger is part of the
    // same episode state and resets with it.
    if (recurrent_reset_pending_) {
      for (const auto& seed : seeds_) {
        const auto t = static_cast<std::size_t>(seed.tensor);
        if (t >= n_tensors) {
          continue;
        }
        const std::span<float> dst = in_bufs[t];
        if (!seed.has_seed) {
          std::fill(dst.begin(), dst.end(), 0.0F);
          continue;
        }
        const auto w = static_cast<std::size_t>(seed.width);
        if (w != dst.size() || w > scratch_measured_.size() ||
            !ExtractFeature(seed.kind, seed.arg, state,
                            std::span<double>(scratch_measured_.data(), w))) {
          have_action_ = false;
          HoldPosition(state, out, InferenceHoldReason::kSeed);
          last_tick_held_ = true;
          return out;
        }
        for (std::size_t i = 0; i < w; ++i) {
          dst[i] = static_cast<float>(scratch_measured_[i]);
        }
      }
      reach_state_ = {};
    }

    if (!PackObservation(state, std::span<const std::span<float>>(in_bufs.data(), n_tensors))) {
      have_action_ = false;
      HoldPosition(state, out, pack_failure_);
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
      HoldPosition(state, out, InferenceHoldReason::kRunFailed);
      last_tick_held_ = true;
      return out;
    }

    // ── Unpack: arm target ─────────────────────────────────────────────────
    // A named head is gathered by name into device joint order; an unnamed
    // one is the positional slice. Either way the values are in the POLICY's
    // joint convention and cross back through it before they become commands.
    const auto& arm_slice = io_.output_features[static_cast<std::size_t>(arm_target_idx_)].slice;
    const auto& hand_slice = io_.output_features[static_cast<std::size_t>(hand_command_idx_)].slice;

    bool ok = arm_slice.count == arm_dof_;
    if (ok) {
      const std::span<double> dst(scratch_head_.data(), static_cast<std::size_t>(arm_dof_));
      const float* src = engine_->output_buffer(0, arm_slice.tensor);
      const std::size_t size = engine_->output_size(0, arm_slice.tensor);
      ok = arm_out_idx_.empty() ? rtc::inference::UnpackSlice(dst, arm_slice, src, size)
                                : rtc::inference::UnpackIndexed(dst, arm_out_idx_, src, size);
    }
    if (ok) {
      for (int i = 0; i < arm_dof_; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const double v = scratch_head_[k];
        if (!std::isfinite(v)) {
          ok = false;
          break;
        }
        arm_action_[k] = arm_sign_[k] * (v - arm_offset_[k]);
      }
    }

    // ── Unpack: hand command ───────────────────────────────────────────────
    // Which of the two shapes arrives is a configure-time fact, so the branch
    // is on a resolved enum and not on anything the tick has to discover.
    if (ok && hand_role_ == PolicyOutputRole::kJointTarget) {
      // Same width guard the arm path carries, and for a sharper reason here:
      // `scratch_head_` was just written with the ARM's targets, and the
      // unpack cannot catch a short slice because the span handed to it is
      // sized by that same `count`. A `count` below `hand_dof_` would leave the
      // tail of this read holding arm joint angles, finite and inside the
      // hand's limits. `parameters.cpp` refuses that config; this keeps the
      // two lanes symmetric.
      ok = hand_slice.count == hand_dof_;
      if (ok) {
        const std::span<double> dst(scratch_head_.data(), static_cast<std::size_t>(hand_dof_));
        const float* src = engine_->output_buffer(0, hand_slice.tensor);
        const std::size_t size = engine_->output_size(0, hand_slice.tensor);
        ok = hand_out_idx_.empty() ? rtc::inference::UnpackSlice(dst, hand_slice, src, size)
                                   : rtc::inference::UnpackIndexed(dst, hand_out_idx_, src, size);
      }
      for (int i = 0; ok && i < hand_dof_; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const double v = scratch_head_[k];
        if (!std::isfinite(v)) {
          ok = false;
          break;
        }
        hand_action_[k] = hand_sign_[k] * (v - hand_offset_[k]);
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
      HoldPosition(state, out, InferenceHoldReason::kOutput);
      last_tick_held_ = true;
      return out;
    }
    // ── Recurrent feedback ─────────────────────────────────────────────────
    // Deliberately AFTER the `ok` gate, which already means "the whole action
    // was accepted". A partial action is not a smaller version of the right
    // thing, and neither is the state that produced it — advancing the state on
    // a rejected step would keep the policy's memory moving while the robot
    // stood still (#511 C-5). The reach trigger is committed here for the same
    // reason.
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
    if (reach_pending_) {
      reach_state_ = reach_state_pending_;
      last_reach_phase_ = reach_phase_pending_;
      last_tip_distance_ = tip_distance_pending_;
      reach_pending_ = false;
    }
    recurrent_reset_pending_ = false;

    have_action_ = true;
    policy_step_this_tick_ = true;
    hold_elapsed_sec_ = 0.0;
    hold_latched_ = false;  // a fresh action supersedes the latch
  }

  // ── Emit ──────────────────────────────────────────────────────────────────
  out.num_devices = std::min(state.num_devices, ControllerOutput::kMaxDevices);

  // The rate bound steps from the PREVIOUS command (D2) — see `cmd_base_valid_`
  // for why not from the measurement, and why the stream re-seeds from the
  // measurement after activation and after a hold.
  std::array<double, kMaxArmDof> arm_cmd{};
  std::array<double, kMaxHandDof> hand_cmd{};
  std::array<double, kMaxArmDof> arm_base{};
  std::array<double, kMaxHandDof> hand_base{};
  for (int i = 0; i < arm_dof_; ++i) {
    const auto k = static_cast<std::size_t>(i);
    arm_cmd[k] = arm_action_[k];
    arm_base[k] = cmd_base_valid_ ? last_cmd_arm_[k] : state.devices[0].positions[k];
  }
  for (int i = 0; i < hand_dof_; ++i) {
    const auto k = static_cast<std::size_t>(i);
    hand_cmd[k] = hand_action_[k];
    hand_base[k] = cmd_base_valid_ ? last_cmd_hand_[k] : state.devices[1].positions[k];
  }

  BoundDeviceCommand(0,
                     std::span<const double>(arm_base.data(), static_cast<std::size_t>(arm_dof_)),
                     std::span<double>(arm_cmd.data(), static_cast<std::size_t>(arm_dof_)), dt);
  BoundDeviceCommand(1,
                     std::span<const double>(hand_base.data(), static_cast<std::size_t>(hand_dof_)),
                     std::span<double>(hand_cmd.data(), static_cast<std::size_t>(hand_dof_)), dt);
  last_cmd_arm_ = arm_cmd;
  last_cmd_hand_ = hand_cmd;
  cmd_base_valid_ = true;

  // `goal_positions` carries the policy's own target — the value the command
  // is rate-bounded toward — so the state log and GUI show both where the
  // policy wants the joint and where this tick is sending it.
  auto& arm_out = out.devices[0];
  arm_out.num_channels = arm_dof_;
  for (int i = 0; i < arm_dof_; ++i) {
    const auto k = static_cast<std::size_t>(i);
    arm_out.commands[k] = arm_cmd[k];
    arm_out.goal_positions[k] = arm_action_[k];
    arm_out.target_positions[k] = arm_action_[k];
  }
  auto& hand_out = out.devices[1];
  hand_out.num_channels = hand_dof_;
  for (int i = 0; i < hand_dof_; ++i) {
    const auto k = static_cast<std::size_t>(i);
    hand_out.commands[k] = hand_cmd[k];
    hand_out.goal_positions[k] = hand_action_[k];
    hand_out.target_positions[k] = hand_action_[k];
  }

  last_tick_held_ = false;
  return out;
}

ControllerOutput DemoInferenceController::Compute(const ControllerState& state) noexcept {
  last_hold_reason_ = InferenceHoldReason::kNone;
  policy_step_this_tick_ = false;
  closed_fk_ran_ = false;
  ControllerOutput out = ComputeCommand(state);
  PushLogs(state, out);
  return out;
}

void DemoInferenceController::PushLogs(const ControllerState& state,
                                       const ControllerOutput& out) noexcept {
  if (primary_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, out, 0, pod);
    primary_state_log_handle_.Push(pod);
  }
  if (secondary_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, out, 1, pod);
    secondary_state_log_handle_.Push(pod);
  }
  if (!inference_diag_log_handle_) {
    return;
  }

  InferenceDiagLogPod pod{};
  pod.t_relative_s = state.t_relative_s;
  pod.iteration = state.iteration;
  pod.held = last_tick_held_;
  pod.hold_reason = static_cast<std::uint8_t>(last_hold_reason_);
  pod.policy_step = policy_step_this_tick_;
  pod.inference_count = inference_count_;

  pod.reach_phase = last_reach_phase_;
  pod.tip_distance = last_tip_distance_;
  pod.reach_hold = reach_state_.hold;

  pod.object_valid = object_valid_this_tick_;
  if (object_ever_seen_) {
    pod.object_age_s = object_age_sec_;
    if (object_this_tick_.valid) {
      pod.object_position = object_this_tick_.position;
    }
  }

  if (closed_fk_ran_) {
    pod.closed_held = closed_fk_status_.held;
    pod.closed_held_ticks = closed_fk_status_.held_ticks;
    pod.closed_singular = closed_fk_status_.singular;
    pod.closure_error = closed_fk_status_.closure_error;
  }

  if (!last_tick_held_ && state.num_devices >= 1) {
    double lag = 0.0;
    for (int i = 0; i < arm_dof_; ++i) {
      const auto k = static_cast<std::size_t>(i);
      lag = std::max(lag, std::abs(arm_action_[k] - state.devices[0].positions[k]));
    }
    pod.arm_lag_max = lag;
  }

  if (state.num_devices >= 2) {
    const auto n =
        std::min(static_cast<std::size_t>(std::max(reach_tips_, 0)), InferenceDiagLogPod::kMaxTips);
    pod.num_tips = static_cast<std::uint8_t>(n);
    for (std::size_t i = 0; i < n; ++i) {
      pod.tip_force[i] = GroupForceNorm(state.devices[1], reach_force_group_[i]);
    }
  }
  inference_diag_log_handle_.Push(pod);
}

}  // namespace integrated_bringup
