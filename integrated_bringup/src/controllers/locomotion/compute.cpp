// locomotion_controller: the RT tick. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion/pose_guard.hpp"
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_controller_interface/device_readability.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace integrated_bringup {

namespace loco = locomotion;
using rtc::ControllerOutput;
using rtc::ControllerState;

namespace {

// The lever rule's bounds (LatchReference): two contacts closer than this
// share the weight half and half; a centre of pressure is placed at most this
// far from its contact's origin per axis.
constexpr double kLeverMinSeparationM = 0.05;
constexpr double kCopOffsetMaxM = 0.1;

}  // namespace

ControllerOutput LocomotionController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("LocomotionController::Compute");
  const auto t0 = std::chrono::steady_clock::now();
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count();

  ServiceRequests();
  ++tick_count_;
  last_solve_ = nullptr;
  ReadState(state, now_ns);
  if (mpc_enabled_) {
    wbc_ref_.valid = false;  // a tick that does not sample the plan logs none
  }

  if (fault_latched_tick_) {
    mode_ = Mode::kFault;
  } else {
    // The E-STOP (header): either edge restarted the reference, and nothing
    // latches one while the stop holds — so the tick below waits, solves
    // nothing and sends the MPC nothing, while the state, the models and the
    // logs still run.
    if (source_ok_ && !reference_latched_ && !estop_active_) {
      TryLatchReference(state);
    }
    if (mpc_enabled_ && source_ok_ && reference_latched_) {
      // The MPC's clock: the tick count, so it runs with the control loop. The
      // MPC gets a state only while a reference is latched (the reset, the
      // E-STOP edges and the activation restart its stream).
      mpc_t_ns_ =
          static_cast<std::uint64_t>(state.iteration) * static_cast<std::uint64_t>(tick_dt_ns_);
      WriteMpcState();
      FollowPlan(state);  // may latch the fault (a plan lost off a still double support)
    }
    if (fault_latched_tick_) {
      mode_ = Mode::kFault;
    } else if (!reference_latched_) {
      mode_ = Mode::kWaiting;
    } else if (source_bad_ticks_ > static_cast<uint32_t>(cfg_.max_source_bad_ticks)) {
      Latch(FaultCause::kSource);
    } else if (source_ok_) {
      Solve();
    } else {
      // A short run of bad state: hold the last valid torque, as on a missed solve.
      mode_ = Mode::kDegraded;
      tau_ = tau_prev_;
    }
  }

  ControllerOutput out{};
  FillOutput(state, out);
  const double tick_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  FillLogs(state, out, now_ns, tick_us);
  return out;
}

void LocomotionController::ServiceRequests() noexcept {
  const uint32_t generation = ActivationGeneration();
  if (!activation_seen_ || generation != serviced_generation_) {
    // A fresh start from wherever the robot is — and the latch goes with the
    // old session (header). A reset request from before this activation was
    // made to a controller that was not ticking: it is not carried in.
    serviced_generation_ = generation;
    activation_seen_ = true;
    tick_count_ = 0;
    RestartReference();
    if (source_) {
      source_->Reset();
    }
    ClearLatch();
    serviced_reset_epoch_ = fault_reset_epoch_.load(std::memory_order_acquire);
    serviced_estop_state_ = estop_state_.load(std::memory_order_acquire);
  }

  const uint32_t reset_epoch = fault_reset_epoch_.load(std::memory_order_acquire);
  if (reset_epoch != serviced_reset_epoch_) {
    serviced_reset_epoch_ = reset_epoch;
    if (fault_latched_tick_) {
      ClearLatch();
      RestartReference();
    }
  }

  // One load: the flag and the edge count of the same instant (header).
  const uint32_t estop_state = estop_state_.load(std::memory_order_acquire);
  estop_active_ = (estop_state & 1U) != 0;
  if (estop_state != serviced_estop_state_) {
    // Either edge: the robot was not following this controller in between, so
    // the reference starts again from where the robot is.
    serviced_estop_state_ = estop_state;
    RestartReference();
  }
}

void LocomotionController::RestartReference() noexcept {
  reference_latched_ = false;
  hand_hold_mask_ = 0;
  source_bad_ticks_ = 0;
  guard_fail_ticks_ = 0;
  pose_guard_mask_ = 0;
  tau_prev_.fill(0.0);
  tau_.fill(0.0);
  qp_.Reset();
  if (mpc_enabled_) {
    RestartMpcStream();
  }
}

void LocomotionController::ClearLatch() noexcept {
  fault_latched_tick_ = false;
  fault_cause_.store(FaultCause::kNone, std::memory_order_relaxed);
  fault_latched_.store(false, std::memory_order_release);
}

void LocomotionController::ReadState(const ControllerState& state, int64_t now_ns) noexcept {
  source_ok_ = source_ != nullptr && source_->Update(state, indices_, now_ns, measured_);
  if (source_ok_ && measured_.model_epoch != model_epoch_) {
    source_ok_ = false;
    Latch(FaultCause::kModelEpoch);
  }
  source_bad_ticks_ = source_ok_ ? 0 : source_bad_ticks_ + 1;
  if (!source_ok_) {
    return;
  }
  // The dynamics model's state: the base, then the body joints (hands locked).
  for (Eigen::Index i = 0; i < 7; ++i) {
    qp_state_.q(i) = measured_.q[static_cast<std::size_t>(i)];
  }
  for (Eigen::Index i = 0; i < 6; ++i) {
    qp_state_.v(i) = measured_.v[static_cast<std::size_t>(i)];
  }
  for (std::size_t j = 0; j < nb_; ++j) {
    const auto a = static_cast<std::size_t>(reduced_to_full_[j]);
    qp_state_.q(static_cast<Eigen::Index>(7 + j)) = measured_.q[7 + a];
    qp_state_.v(static_cast<Eigen::Index>(6 + j)) = measured_.v[6 + a];
  }
  UpdateModels();
}

void LocomotionController::UpdateModels() noexcept {
  // Full model: every frame at the measured state, hands included.
  for (Eigen::Index i = 0; i < q_full_.size(); ++i) {
    q_full_(i) = measured_.q[static_cast<std::size_t>(i)];
  }
  pinocchio::centerOfMass(*full_model_, *full_data_, q_full_, false);
  pinocchio::updateFramePlacements(*full_model_, *full_data_);
  // Reduced model: the dynamics the QP reads.
  cache_.Update(qp_state_.q, qp_state_.v);
}

std::uint32_t LocomotionController::GuardPose(bool stance) noexcept {
  // Only called with a valid state: measured_ is this tick's sample and the
  // full model's frames are at it.
  std::array<double, LocomotionDiagLogPod::kMaxContacts> z{};
  const std::size_t nc = ContactHeights(z);
  loco::PoseGuardInput in;
  in.base_valid = (measured_.valid_mask & loco::kBaseBit) != 0;
  in.q_xyzw = {measured_.q[3], measured_.q[4], measured_.q[5], measured_.q[6]};
  in.v_base = {measured_.v[0], measured_.v[1], measured_.v[2]};
  in.contact_z = z.data();
  in.num_contacts = static_cast<int>(nc);
  loco::PoseGuardLimits lim;
  lim.cos_tilt = cos_tilt_;
  lim.check_stance = stance;
  lim.max_spread_m = kRelatchFootSpreadM;
  lim.max_speed = kRelatchBaseSpeed;
  pose_guard_mask_ = loco::EvaluatePoseGuard(in, lim);
  return pose_guard_mask_;
}

void LocomotionController::TryLatchReference(const ControllerState& state) noexcept {
  if (GuardPose(/*stance=*/false) == 0) {
    LatchReference(state);
    return;
  }
  // Refused: wait in damping and ask again next tick; a robot that is still
  // down after the settle time is held by the fault latch (header).
  if (++guard_fail_ticks_ >= guard_settle_ticks_) {
    Latch(FaultCause::kPoseGuard);
  }
}

void LocomotionController::LatchReference(const ControllerState& state) noexcept {
  static_cast<void>(state);
  ref_.q_des = qp_state_.q;
  ref_.v_des.setZero();
  ref_.a_des.setZero();
  ref_.tau_ff.setZero();
  ref_.lambda_des.setZero();
  // The static wrench of the measured pose. Two contacts: the weight splits by
  // the lever rule along the line between them at the CoM's projection, and
  // each surface contact's centre of pressure sits off its origin by the CoM's
  // offset from that line — so the forces and moments together balance gravity
  // at the CoM and the QP has nothing to trade against its acceleration. Any
  // other count: an equal share of the weight, no moment. (z slot = the normal
  // force of a point or a surface contact; moments in world axes at the frame.)
  // Bounded: contacts closer than kLeverMinSeparationM split the weight half
  // and half (the projection onto a near-zero line is noise), and the CoP sits
  // at most kCopOffsetMaxM off its origin per axis — a sunk or shifted pose
  // gets a finite, foot-sized moment the QP can trade, never one outside the
  // sole. Every value here is finite by the state source's contract (a state
  // with a non-finite q is not ok) and the guard's (base and contact heights).
  const double weight = weight_;
  const std::size_t nc = cfg_.model.contacts.size();
  std::array<double, LocomotionDiagLogPod::kMaxContacts> share{};
  Eigen::Vector2d cop_offset = Eigen::Vector2d::Zero();
  if (nc == 2) {
    const Eigen::Vector2d c = full_data_->com[0].head<2>();
    const Eigen::Vector2d p0 = full_data_->oMf[contact_frames_full_[0]].translation().head<2>();
    const Eigen::Vector2d p1 = full_data_->oMf[contact_frames_full_[1]].translation().head<2>();
    const Eigen::Vector2d d = p1 - p0;
    const double len2 = d.squaredNorm();
    const double t = len2 >= kLeverMinSeparationM * kLeverMinSeparationM
                         ? std::clamp((c - p0).dot(d) / len2, 0.0, 1.0)
                         : 0.5;
    share[0] = (1.0 - t) * weight;
    share[1] = t * weight;
    cop_offset = (c - (p0 + t * d)).cwiseMax(-kCopOffsetMaxM).cwiseMin(kCopOffsetMaxM);
  } else {
    for (std::size_t i = 0; i < nc; ++i) {
      share[i] = weight / static_cast<double>(std::max<std::size_t>(1, nc));
    }
  }
  Eigen::Index offset = 0;
  for (std::size_t i = 0; i < nc; ++i) {
    ref_.lambda_des(offset + 2) = share[i];
    if (cfg_.model.contacts[i].dim == 6) {
      // τ = r × (0, 0, f): r = the CoP's offset from the frame origin.
      ref_.lambda_des(offset + 3) = cop_offset.y() * share[i];
      ref_.lambda_des(offset + 4) = -cop_offset.x() * share[i];
    }
    offset += cfg_.model.contacts[i].dim;
  }
  // The hands are not touched: FillHands holds each from its own first
  // readable tick (header), so the body's latch never moves a hand.
  tau_prev_.fill(0.0);
  qp_.Reset();
  reference_latched_ = true;
}

void LocomotionController::Solve() noexcept {
  const rtc::tsid::CommandOutput& out = qp_.Compute(qp_state_, ref_, cache_, contacts_);
  last_solve_ = &out;
  if (out.tau_valid) {
    for (std::size_t i = 0; i < nb_; ++i) {
      tau_[i] = out.tau(static_cast<Eigen::Index>(i));
    }
    tau_prev_ = tau_;
    mode_ = Mode::kNominal;
  } else {
    tau_ = tau_prev_;
    mode_ = Mode::kDegraded;
  }
  if (qp_.Health().State() == rtc::tsid::SolveHealthState::kFault) {
    Latch(FaultCause::kSolveHealth);
  }
}

void LocomotionController::Latch(FaultCause cause) noexcept {
  if (!fault_latched_tick_) {
    fault_latched_tick_ = true;
    fault_cause_.store(cause, std::memory_order_relaxed);
    ++fault_count_;
    fault_latched_.store(true, std::memory_order_release);
  }
  mode_ = Mode::kFault;
}

// ── Output ──────────────────────────────────────────────────────────────────

void LocomotionController::FillOutput(const ControllerState& state,
                                      ControllerOutput& out) noexcept {
  out.num_devices = state.num_devices;
  out.command_type = rtc::CommandType::kPdFeedforward;
  out.valid = true;
  auto& body = out.devices[static_cast<std::size_t>(indices_.body.device)];
  body.goal_type = rtc::GoalType::kJoint;
  body.has_pd_gains = true;
  for (std::size_t i = 0; i < nb_; ++i) {
    body.pd_kp[i] = kp_[i];
    body.pd_kd[i] = kd_[i];
  }

  const bool commanding = mode_ == Mode::kNominal || mode_ == Mode::kDegraded;
  if (commanding) {
    body.num_channels = static_cast<int>(nb_);
    bool finite = true;
    for (std::size_t i = 0; i < nb_; ++i) {
      const double q_cmd = ref_.q_des(static_cast<Eigen::Index>(7 + i));
      body.commands[i] = q_cmd;
      body.feedforward[i] = tau_[i];
      finite = finite && std::isfinite(q_cmd) && std::isfinite(tau_[i]);
    }
    if (!finite) {
      Latch(FaultCause::kNonFinite);
      FillDamping(state, body);
    }
  } else {
    FillDamping(state, body);
  }
  for (int i = 0; i < body.num_channels; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    body.goal_positions[ui] = body.commands[ui];
    body.target_positions[ui] = body.commands[ui];
    body.trajectory_positions[ui] = body.commands[ui];
    body.trajectory_velocities[ui] = 0.0;
  }
  FillHands(state, out);
}

void LocomotionController::FillDamping(const ControllerState& state,
                                       rtc::DeviceOutput& body) noexcept {
  const auto slot = static_cast<std::size_t>(indices_.body.device);
  const int n = static_cast<int>(nb_);
  if (state.num_devices <= static_cast<int>(slot) ||
      !rtc::IsDeviceReadable(state.devices[slot], n)) {
    body.num_channels = 0;  // nothing honest to command: the backend's safe output
    return;
  }
  const auto& dev = state.devices[slot];
  const bool v_ok = rtc::IsLaneReadable(dev, rtc::StateLane::kVelocity, n);
  body.num_channels = n;
  for (std::size_t i = 0; i < nb_; ++i) {
    const double v = v_ok && std::isfinite(dev.velocities[i]) ? dev.velocities[i] : 0.0;
    const double ff = std::clamp(-damping_kd_[i] * v, -tau_max_[i], tau_max_[i]);
    tau_[i] = ff;
    body.commands[i] = dev.positions[i];
    body.feedforward[i] = ff;
  }
}

void LocomotionController::FillHands(const ControllerState& state, ControllerOutput& out) noexcept {
  for (int h = 0; h < indices_.num_hands; ++h) {
    const auto& map = indices_.hands[static_cast<std::size_t>(h)];
    const auto slot = static_cast<std::size_t>(map.device);
    if (static_cast<int>(slot) >= state.num_devices) {
      continue;
    }
    auto& hand = out.devices[slot];
    hand.command_type = rtc::CommandType::kPosition;
    hand.goal_type = rtc::GoalType::kJoint;
    const auto& dev = state.devices[slot];
    const uint32_t bit = 1U << static_cast<uint32_t>(h);
    if ((hand_hold_mask_ & bit) == 0) {
      if (!rtc::IsDeviceReadable(dev, map.num_channels)) {
        hand.num_channels = 0;  // nothing honest to hold yet
        continue;
      }
      // The first readable tick after a restart: hold here — whether or not
      // the body's reference is latched yet (header).
      for (int c = 0; c < map.num_channels; ++c) {
        const auto uc = static_cast<std::size_t>(c);
        const auto a = static_cast<std::size_t>(map.joint[uc]);
        hand_hold_[a] = mpc_enabled_ ? q_default_full_[a] : dev.positions[uc];
      }
      hand_hold_mask_ |= bit;
    }
    hand.num_channels = map.num_channels;
    for (int c = 0; c < map.num_channels; ++c) {
      const auto uc = static_cast<std::size_t>(c);
      const auto a = static_cast<std::size_t>(map.joint[uc]);
      const double target = hand_hold_[a];
      hand.commands[uc] = target;
      hand.goal_positions[uc] = target;
      hand.target_positions[uc] = target;
      hand.trajectory_positions[uc] = target;
    }
  }
}

// ── Logs ────────────────────────────────────────────────────────────────────

void LocomotionController::FillLogs(const ControllerState& state, const ControllerOutput& out,
                                    int64_t now_ns, double tick_us) noexcept {
  const auto slot = static_cast<std::size_t>(indices_.body.device);
  const auto& dev = state.devices[slot];
  const auto& body = out.devices[slot];
  const auto nj = std::min(nb_, LocomotionDiagLogPod::kMaxJoints);

  LocomotionDiagLogPod& d = diag_;
  d.t_relative_s = state.t_relative_s;
  d.tick = state.iteration;
  d.tick_count = tick_count_;
  d.mode = static_cast<std::uint8_t>(mode_);
  d.estop = estop_active_;
  d.fault_cause = static_cast<std::uint8_t>(fault_cause_.load(std::memory_order_relaxed));
  d.fault_latched = fault_latched_tick_;
  d.fault_count = fault_count_;
  d.pose_guard_mask = pose_guard_mask_;
  d.source_status = static_cast<std::uint8_t>(measured_.status);
  d.source_fresh_mask = measured_.fresh_mask;
  d.source_valid_mask = measured_.valid_mask;
  d.source_bad_ticks = source_bad_ticks_;
  d.source_sequence = measured_.sequence;
  d.source_age_us = measured_.stamp_steady_ns > 0
                        ? static_cast<double>(now_ns - measured_.stamp_steady_ns) * 1e-3
                        : -1.0;
  const auto& health = qp_.Health();
  d.health = static_cast<std::uint8_t>(health.State());
  d.health_bad_run = static_cast<std::uint32_t>(std::max(0, health.BadRun()));
  const auto* s = last_solve_;
  d.tau_valid = s != nullptr && s->tau_valid;
  d.qp_converged = s != nullptr && s->qp_converged;
  d.qp_status = s != nullptr ? s->qp_status : -1;
  d.qp_iterations = s != nullptr ? s->qp_iterations : 0;
  d.rows_dropped = s != nullptr ? s->rows_dropped : 0;
  d.solve_time_us = s != nullptr ? s->solve_time_us : 0.0;
  d.wbc_tick_us = tick_us;
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  d.qp_pri_res = s != nullptr ? s->qp_pri_res : kNaN;
  d.qp_dua_res = s != nullptr ? s->qp_dua_res : kNaN;
  const auto& m = qp_.Margins();
  d.eom_residual_inf = d.tau_valid ? m.eom_residual_inf : kNaN;
  d.margin_friction_n = m.friction_n;
  d.margin_cop_nm = m.cop_nm;
  d.margin_yaw_nm = m.yaw_nm;
  d.margin_torque_nm = m.torque_nm;
  d.margin_joint_accel = m.joint_accel_box;
  d.margins_stale = m.stale || !d.tau_valid;
  for (std::size_t i = 0; i < 3; ++i) {
    d.base_p[i] = measured_.q[i];
    d.ref_base_p[i] = ref_.q_des.size() > 2 ? ref_.q_des(static_cast<Eigen::Index>(i)) : 0.0;
    d.com[i] = full_data_->com[0](static_cast<Eigen::Index>(i));
  }
  d.ref_source = static_cast<std::uint8_t>(Source());
  const bool plan = mpc_enabled_ && wbc_ref_.valid;
  d.plan_id = plan ? wbc_ref_.plan_id : 0;
  d.policy_age_us = plan ? static_cast<double>(static_cast<std::int64_t>(
                               mpc_t_ns_ - wbc_ref_.observation_stamp_ns)) *
                               1e-3
                         : -1.0;
  d.mpc_converged = plan && wbc_ref_.converged;
  d.mpc_initial_solve = plan && wbc_ref_.initial_solve;
  d.mpc_solve_us = plan ? static_cast<double>(wbc_ref_.solve_duration_ns) * 1e-3 : 0.0;
  d.schedule_epoch = plan ? wbc_ref_.schedule_epoch : 0;
  d.contact_mask = contact_mask_;
  d.plausibility_rejects = plausibility_rejects_.load(std::memory_order_relaxed);
  d.u_fb_norm = u_fb_norm_;
  d.mpc_state_epoch = mpc_state_epoch_;
  for (std::size_t i = 0; i < 4; ++i) {
    d.base_q_xyzw[i] = measured_.q[3 + i];
  }
  for (std::size_t i = 0; i < 6; ++i) {
    d.base_v[i] = measured_.v[i];
    d.a_opt_base[i] = s != nullptr ? s->a_opt(static_cast<Eigen::Index>(i)) : 0.0;
  }
  d.num_contacts = static_cast<std::uint8_t>(contact_frames_full_.size());
  Eigen::Index offset = 0;
  for (std::size_t c = 0; c < contact_frames_full_.size(); ++c) {
    const auto& p = full_data_->oMf[contact_frames_full_[c]].translation();
    for (std::size_t k = 0; k < 3; ++k) {
      d.foot_p[c * 3 + k] = p(static_cast<Eigen::Index>(k));
    }
    const int dim = cfg_.model.contacts[c].dim;
    for (int k = 0; k < 6; ++k) {
      d.lambda[c * 6 + static_cast<std::size_t>(k)] =
          (s != nullptr && k < dim) ? s->lambda_opt(offset + k) : 0.0;
    }
    offset += dim;
  }
  // The command timer's last step (a new one when it published since the last
  // tick, else the one already held).
  static_cast<void>(command_log_.TryAcquireLatest());
  const LocomotionCommandLogSample& cmd = command_log_.Current();
  d.cmd_vx = cmd.v_x;
  d.cmd_vy = cmd.v_y;
  d.cmd_wz = cmd.omega_z;
  d.cmd_h = cmd.base_height;
  d.cmd_valid = cmd.fresh;
  d.gait_latest = cmd.gait;
  for (std::size_t i = 0; i < 3; ++i) {
    d.ref_base_v[i] = plan ? wbc_ref_.v_ref[i] : kNaN;
  }
  // Contact c is group c of the body's inference lane: fz is value 3 of the group.
  const std::size_t nc = std::min(contact_frames_full_.size(), LocomotionDiagLogPod::kMaxContacts);
  const auto stride = static_cast<std::size_t>(std::max(contact_force_stride_, 0));
  const bool lane = stride >= 4 && dev.num_inference_groups == static_cast<int>(nc);
  d.meas_contact_valid = 0;
  for (std::size_t c = 0; c < nc; ++c) {
    const std::size_t at = c * stride + 3;
    const bool fresh = lane && at < dev.inference_data.size() && dev.inference_enable[c];
    d.meas_fz[c] = fresh ? static_cast<double>(dev.inference_data[at]) : kNaN;
    if (fresh) {
      d.meas_contact_valid |= 1U << static_cast<std::uint32_t>(c);
    }
  }
  double servo2 = 0.0;
  double ff2 = 0.0;
  d.num_joints = static_cast<std::uint8_t>(nj);
  for (std::size_t i = 0; i < nj; ++i) {
    const double q = dev.positions[i];
    const double v = dev.velocities[i];
    const double cmd = body.num_channels > 0 ? body.commands[i] : q;
    const double ff = body.num_channels > 0 ? body.feedforward[i] : 0.0;
    const double servo = kp_[i] * (cmd - q) - kd_[i] * v;
    d.v[i] = v;
    d.tau_servo[i] = servo;
    servo2 += servo * servo;
    ff2 += ff * ff;
  }
  d.servo_norm = std::sqrt(servo2);
  d.ff_norm = std::sqrt(ff2);

  JointFeedforwardLogPod& j = joint_log_;
  j.t_relative_s = state.t_relative_s;
  j.tick = state.iteration;
  j.estop = estop_active_;
  j.readable = rtc::IsDeviceReadable(dev, static_cast<int>(nb_));
  j.seeded = reference_latched_;
  j.ff_enabled = true;
  j.mode = static_cast<std::uint8_t>(mode_);
  j.ff_finite = std::all_of(body.feedforward.begin(), body.feedforward.begin() + nj,
                            [](double x) { return std::isfinite(x); });
  j.model_fresh = source_ok_;
  j.num_joints = static_cast<std::uint8_t>(nj);
  for (std::size_t i = 0; i < nj; ++i) {
    j.q[i] = dev.positions[i];
    j.q_ref[i] = body.num_channels > 0 ? body.commands[i] : dev.positions[i];
    j.g[i] = cache_.g.size() > static_cast<Eigen::Index>(6 + i)
                 ? cache_.g(static_cast<Eigen::Index>(6 + i))
                 : 0.0;
    j.tau_ff[i] = body.num_channels > 0 ? body.feedforward[i] : 0.0;
    j.qdd_ref[i] = s != nullptr ? s->a_opt(static_cast<Eigen::Index>(6 + i)) : 0.0;
    j.tau_inertia[i] = 0.0;
  }

  for (std::size_t g = 0; g < state_log_handles_.size(); ++g) {
    if (state_log_handles_[g]) {
      DeviceStateLogPod pod{};
      FillDeviceStateLogPod(state, out, g, pod);
      state_log_handles_[g].Push(pod);
    }
  }
  if (joint_log_handle_) {
    joint_log_handle_.Push(joint_log_);
  }
  if (diag_log_handle_) {
    diag_log_handle_.Push(diag_);
  }
  status_lock_.Store(diag_);  // the status topic's record
}

}  // namespace integrated_bringup
