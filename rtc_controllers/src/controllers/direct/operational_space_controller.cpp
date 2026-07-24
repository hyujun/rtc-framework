// ── Includes: project header first, then C++ stdlib
// ────────────────────────────
#include "rtc_controllers/direct/operational_space_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace rtc {

// ── Constructor ─────────────────────────────────────────────────────────────

OperationalSpaceController::OperationalSpaceController(std::string_view urdf_path, Gains gains)
    : gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";

  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  InitFromModel(builder.GetFullModel());
}

void OperationalSpaceController::InitFromModel(std::shared_ptr<const pinocchio::Model> model) {
  model_ptr_ = std::move(model);
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);

  // Default: use the last frame in the model as tip
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);

  const int nv = handle_->nv();

  // Pre-allocate all Eigen buffers + Cholesky factors to their final sizes so
  // the RT path never allocates. nv-dependent buffers must be re-sized whenever
  // the model changes (e.g. submodel selection — #172 A1). OSC has no fixed
  // kMaxRobotDOF cap (all buffers are dynamic), so no capacity check here.
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  Jt_ = Eigen::MatrixXd::Zero(nv, 6);
  M_ = Eigen::MatrixXd::Zero(nv, nv);
  MinvJt_ = Eigen::MatrixXd::Zero(nv, 6);
  h_ = Eigen::VectorXd::Zero(nv);
  tau_out_ = Eigen::VectorXd::Zero(nv);
  gravity_estop_ = Eigen::VectorXd::Zero(nv);
  grav_dev_ = Eigen::VectorXd::Zero(nv);
  qdot_dev_ = Eigen::VectorXd::Zero(nv);
  tau_dev_ = Eigen::VectorXd::Zero(nv);
  JbarT_ = Eigen::MatrixXd::Zero(6, nv);
  NT_ = Eigen::MatrixXd::Zero(nv, nv);
  tau0_ = Eigen::VectorXd::Zero(nv);
  tau0_dev_ = Eigen::VectorXd::Zero(nv);
  null_tmp_ = Eigen::VectorXd::Zero(nv);
  LambdaInv_.setZero();
  task_err_.setZero();
  a_task_.setZero();
  F_.setZero();
  tcp_vel_.setZero();

  // Pre-size the Cholesky factorisations (allocates storage once, here).
  llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv);
}

void OperationalSpaceController::MaybeSelectSubModel() {
  const auto* sys = GetSystemModelConfig();
  if (sys == nullptr || sys->urdf_path.empty() || sys->sub_models.empty()) {
    return;  // no system topology → keep the full model built in the ctor
  }
  const auto primary = GetPrimaryDeviceName();
  if (primary.empty()) {
    return;
  }
  const bool matches =
      std::any_of(sys->sub_models.begin(), sys->sub_models.end(),
                  [&](const rtc_urdf_bridge::SubModelConfig& sm) { return sm.name == primary; });
  if (!matches) {
    return;  // primary device is not a declared sub_model → keep full model
  }
  auto builder = GetSharedModelBuilder();
  if (!builder) {
    builder = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(*sys);
  }
  try {
    InitFromModel(builder->GetReducedModel(primary));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
        rclcpp::get_logger("OperationalSpaceController"),
        "[OperationalSpaceController] reduced submodel '%s' unavailable (%s) — keeping full model",
        primary.c_str(), e.what());
  }
}

void OperationalSpaceController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) {
        tip_frame_id_ = fid;
      }
    }
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
      max_joint_torque_ = cfg->joint_limits->max_torque;
    }
    if (!cfg->safe_position.empty()) {
      safe_position_ = cfg->safe_position;
    }
    // Register device→model joint reorder (#172 A2): the model consumes
    // device-order q/v and τ is scattered back to device channel order.
    // Identity order → HasJointReorder()==false → zero-overhead passthrough.
    const auto js = static_cast<int>(cfg->joint_state_names.size());
    const int nv = handle_->nv();
    if (js == nv) {
      if (!handle_->SetJointOrder(cfg->joint_state_names)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("OperationalSpaceController"),
            "[OperationalSpaceController] primary device '%s' joint_state_names not all in model — "
            "reorder disabled, falling back to positional (URDF-order) consumption",
            primary.c_str());
      }
    } else if (js > 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OperationalSpaceController"),
          "[OperationalSpaceController] primary device '%s' joint_state_names size=%d != model DOF "
          "nv=%d — FK/dynamics state mapping will be inconsistent",
          primary.c_str(), js, nv);
    }
  }
  if (max_joint_velocity_.empty()) {
    max_joint_velocity_.assign(kMaxDeviceChannels, kDefaultMaxJointVelocity);
  }
  if (max_joint_torque_.empty()) {
    max_joint_torque_.assign(kMaxDeviceChannels, kDefaultMaxJointTorque);
  }
  if (safe_position_.empty()) {
    safe_position_.assign(kMaxDeviceChannels, 0.0);
  }
  // The torque E-STOP hold (#184) maps max_joint_torque_ as an exactly nv-sized
  // Eigen vector; a config supplying fewer than nv entries would read OOB. Grow
  // (never shrink) so ClampSymmetric's existing per-index default fallback on the
  // main path is unaffected.
  if (const auto nvz = static_cast<std::size_t>(handle_->nv()); max_joint_torque_.size() < nvz) {
    max_joint_torque_.resize(nvz, kDefaultMaxJointTorque);
  }
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput OperationalSpaceController::Compute(const ControllerState& state) noexcept {
  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free). Read once
  // and threaded into ComputeEstop too (the hold path needs estop_damping) — no
  // second SeqLock load on the E-STOP branch.
  const auto gains = gains_lock_.Load();

  if (estopped_.load(std::memory_order_acquire)) {
    // Drain & discard any targets queued during the global E-STOP. Compute()
    // short-circuits here BEFORE the normal drain (below), so without this the
    // SPSC queue backs up; and since ClearEstop() does not bump the activation
    // generation, those stale pre-E-STOP commands would pass IsCurrentGeneration
    // on the first recovery tick and overwrite the measured-pose re-seed. The RT
    // thread is the sole SPSC consumer, so draining here keeps the single-
    // consumer invariant.
    PendingTarget discarded{};
    while (pending_targets_.Pop(discarded)) {
      // discard: a command issued during E-STOP must not survive recovery
    }
    return ComputeEstop(state, gains);
  }

  const int nv = handle_->nv();

  // ── Step 1: copy joint state into buffers ───────────────────────────────
  const auto& dev0 = state.devices[0];
  std::array<double, kMaxDeviceChannels> q_buf{};
  std::array<double, kMaxDeviceChannels> v_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    v_buf[ui] = dev0.velocities[ui];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  std::span<const double> v_span(v_buf.data(), static_cast<std::size_t>(nv));

  // ── Step 2: FK + full Jacobian ────────────────────────────────────────────
  // ComputeJacobians performs FK internally and updates frame placements.
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);

  // ── Step 3: current task-space velocity  tcp_vel = J * dq ────────────────
  Eigen::Map<const Eigen::VectorXd> v_eigen(v_buf.data(), nv);
  tcp_vel_.noalias() = J_full_ * v_eigen;

  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);

  // ── Target slot maintenance (RT thread is the single SeqLock writer) ──
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;

  if (!target_initialized_.load(std::memory_order_acquire)) {
    // Seed pose_target / goal_rot / goal_t from current TCP.
    const Eigen::Vector3d rpy_init = pinocchio::rpy::matrixToRpy(tcp.rotation());
    slot.pose_target[0] = tcp.translation()[0];
    slot.pose_target[1] = tcp.translation()[1];
    slot.pose_target[2] = tcp.translation()[2];
    slot.pose_target[3] = rpy_init[0];
    slot.pose_target[4] = rpy_init[1];
    slot.pose_target[5] = rpy_init[2];
    std::memcpy(slot.goal_rot.data(), tcp.rotation().data(), sizeof(slot.goal_rot));
    std::memcpy(slot.goal_t.data(), tcp.translation().data(), sizeof(slot.goal_t));
    goal_pose_ = tcp;
    for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
      const auto& dev = state.devices[d];
      if (!dev.valid) {
        continue;
      }
      const std::size_t nch = std::min(static_cast<std::size_t>(dev.num_channels),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i) {
        slot.targets[d][i] = dev.positions[i];
      }
    }
    trajectory_.initialize(tcp, pinocchio::Motion::Zero(), tcp, pinocchio::Motion::Zero(), 0.01);
    trajectory_time_ = 0.0;
    has_pending_segment_ = false;
    new_target_pending_ = false;
    target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
  } else {
    // Reader-side: restore RT working SE3 from the slot's flat doubles so
    // diagnostic-echo paths (task_goal_positions below) see a consistent
    // rotation matrix every tick.
    std::memcpy(goal_pose_.rotation().data(), slot.goal_rot.data(), sizeof(slot.goal_rot));
    std::memcpy(goal_pose_.translation().data(), slot.goal_t.data(), sizeof(slot.goal_t));
  }

  // Drain pending targets pushed by off-RT SetDeviceTarget callers.
  PendingTarget pending{};
  while (pending_targets_.Pop(pending)) {
    // Drop targets queued before the current activation (#196 §3) — they were
    // addressed to a controller that is no longer the one running.
    if (!IsCurrentGeneration(pending.generation)) {
      continue;
    }
    const auto didx = static_cast<std::size_t>(pending.device_idx);
    if (didx >= ControllerState::kMaxDevices) {
      continue;
    }
    if (didx == 0) {
      const std::size_t pose_n =
          std::min(static_cast<std::size_t>(pending.num_values), std::size_t{6});
      for (std::size_t i = 0; i < pose_n; ++i) {
        slot.pose_target[i] = pending.values[i];
      }
      if (pose_n >= 6) {
        const Eigen::Vector3d translation(pending.values[0], pending.values[1], pending.values[2]);
        const Eigen::Matrix3d rotation =
            RpyToMatrix(pending.values[3], pending.values[4], pending.values[5]);
        std::memcpy(slot.goal_rot.data(), rotation.data(), sizeof(slot.goal_rot));
        std::memcpy(slot.goal_t.data(), translation.data(), sizeof(slot.goal_t));
        goal_pose_.translation() = translation;
        goal_pose_.rotation() = rotation;
        new_target_pending_ = true;
      }
    } else {
      const std::size_t nch = std::min(static_cast<std::size_t>(pending.num_values),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i) {
        slot.targets[didx][i] = pending.values[i];
      }
    }
    slot_dirty = true;
  }

  if (slot_dirty) {
    target_seqlock_.Store(slot);
  }

  // ── Step 3.5: initialise trajectory on new target (after FK) ─────────────
  if (new_target_pending_) {
    {
      const Eigen::Vector3d start_pos = tcp.translation();
      const Eigen::Vector3d goal_pos = goal_pose_.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      // Duration from translational trajectory_speed and velocity limit.
      // Quintic rest-to-rest peak velocity = (15/8) * dist / T.
      const double T_speed_trans = trans_dist / gains.trajectory_speed;
      const double T_vel_trans =
          (gains.max_traj_velocity > 0.0) ? (1.875 * trans_dist / gains.max_traj_velocity) : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      // Angular distance via AngleAxisd (stable at θ → π, unlike log3)
      constexpr double kPiSafetyMargin = 0.15;  // rad ≈ 8.6°
      const Eigen::AngleAxisd aa(tcp.rotation().transpose() * goal_pose_.rotation());
      const double angular_dist = aa.angle();  // always in [0, π]
      const Eigen::Vector3d rot_axis = aa.axis();

      const double T_speed_rot = angular_dist / gains.trajectory_angular_speed;
      const double T_vel_rot = (gains.max_traj_angular_velocity > 0.0)
                                   ? (1.875 * angular_dist / gains.max_traj_angular_velocity)
                                   : 0.0;
      duration = std::max({duration, T_speed_rot, T_vel_rot});

      const bool split_trajectory = (angular_dist > M_PI - kPiSafetyMargin);

      if (split_trajectory) {
        // ── π-rotation defense: split into 2 rest-to-rest segments ──
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid =
            tcp.rotation() * Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

        pinocchio::SE3 mid_pose;
        mid_pose.translation() = 0.5 * (start_pos + goal_pos);
        mid_pose.rotation() = R_mid;

        // Segment 1: start → mid (half distances)
        const double half_trans = trans_dist * 0.5;
        const double T1_speed_t = half_trans / gains.trajectory_speed;
        const double T1_vel_t =
            (gains.max_traj_velocity > 0.0) ? (1.875 * half_trans / gains.max_traj_velocity) : 0.0;
        const double T1_speed_r = half_angle / gains.trajectory_angular_speed;
        const double T1_vel_r = (gains.max_traj_angular_velocity > 0.0)
                                    ? (1.875 * half_angle / gains.max_traj_angular_velocity)
                                    : 0.0;
        const double dur1 = std::max({0.01, T1_speed_t, T1_vel_t, T1_speed_r, T1_vel_r});

        trajectory_.initialize(tcp, pinocchio::Motion::Zero(), mid_pose, pinocchio::Motion::Zero(),
                               dur1);
        pending_goal_pose_ = goal_pose_;
        pending_duration_ = dur1;  // symmetric split
        has_pending_segment_ = true;
      } else {
        trajectory_.initialize(tcp, pinocchio::Motion::Zero(), goal_pose_,
                               pinocchio::Motion::Zero(), duration);
        has_pending_segment_ = false;
      }

      trajectory_time_ = 0.0;
      new_target_pending_ = false;
    }
  }

  traj_state_ = trajectory_.compute(trajectory_time_, dt);
  trajectory_time_ += dt;

  // ── Segment transition (π-rotation defense) ────────────────────────────
  if (has_pending_segment_ && trajectory_time_ >= trajectory_.duration()) {
    pinocchio::SE3 mid_pose = trajectory_.compute(trajectory_.duration()).pose;
    trajectory_.initialize(mid_pose, pinocchio::Motion::Zero(), pending_goal_pose_,
                           pinocchio::Motion::Zero(), pending_duration_);
    trajectory_time_ = 0.0;
    has_pending_segment_ = false;
  }

  // ── Step 4: 6D pose error w.r.t. trajectory setpoint ─────────────────────
  // SplitWorld = [p_d−p ; log3(R_d Rᵀ)] (base position + world-frame rotation),
  // computed via rtc_math (rtc::math::se3).
  task_err_ = rtc::math::se3::computePoseError(tcp, traj_state_.pose,
                                               rtc::math::se3::ErrorType::SplitWorld);
  const Eigen::Vector3d pos_err = task_err_.head<3>();
  const Eigen::Vector3d rot_err = task_err_.tail<3>();
  tcp_position_ = {tcp.translation()[0], tcp.translation()[1], tcp.translation()[2]};

  // Cache for diagnostics (non-RT reads via pose_error())
  for (int i = 0; i < 6; ++i) {
    pose_error_cache_[static_cast<std::size_t>(i)] = task_err_[i];
  }

  // ── Step 5: desired task-space acceleration (task PD + feedforward) ────────
  //   a_task = kp·e_pose + kd·(ẋ_d − ẋ) + a_ff
  const Eigen::Vector3d kp_p(gains.kp_pos[0], gains.kp_pos[1], gains.kp_pos[2]);
  const Eigen::Vector3d kd_p(gains.kd_pos[0], gains.kd_pos[1], gains.kd_pos[2]);
  const Eigen::Vector3d kp_r(gains.kp_rot[0], gains.kp_rot[1], gains.kp_rot[2]);
  const Eigen::Vector3d kd_r(gains.kd_rot[0], gains.kd_rot[1], gains.kd_rot[2]);

  a_task_.head<3>() = kp_p.cwiseProduct(pos_err) +
                      kd_p.cwiseProduct(traj_state_.velocity.linear() - tcp_vel_.head<3>()) +
                      traj_state_.acceleration.linear();
  a_task_.tail<3>() = kp_r.cwiseProduct(rot_err) +
                      kd_r.cwiseProduct(traj_state_.velocity.angular() - tcp_vel_.tail<3>()) +
                      traj_state_.acceleration.angular();

  // ── Step 6: joint-space dynamics  M(q),  h = C(q,v)·v + g(q) ──────────────
  // GetMassMatrix() already returns a full symmetric M (the handle symmetrises
  // CRBA's upper triangle internally); the mirror below is a cheap defensive
  // no-op that keeps this correct even if that handle contract ever changes.
  handle_->ComputeMassMatrix(q_span);
  M_ = handle_->GetMassMatrix();
  M_.triangularView<Eigen::StrictlyLower>() = M_.triangularView<Eigen::StrictlyUpper>().transpose();
  handle_->ComputeNonLinearEffects(q_span, v_span);
  h_ = handle_->GetNonLinearEffects();

  // ── Step 7: task inertia inverse  Λ⁻¹ = J M⁻¹ Jᵀ + λ²I ────────────────────
  // In-place Cholesky solves keep the RT path allocation-free. M is provably SPD
  // for a physical fixed-base manipulator — that assumption is load-bearing: the
  // λ² floor regularises Λ⁻¹ (kinematic/Jacobian singularities) but NOT M itself
  // (an ill-conditioned URDF inertia would need M-side regularisation). Both
  // factorisations are checked via .info(); on failure we fall back to a safe
  // gravity/Coriolis-hold torque (τ = h) rather than emit NaN (issue #172 review).
  bool dyn_ok = true;
  llt_M_.compute(M_);
  dyn_ok = dyn_ok && (llt_M_.info() == Eigen::Success);

  if (dyn_ok) {
    Jt_.noalias() = J_full_.transpose();
    MinvJt_ = Jt_;
    llt_M_.solveInPlace(MinvJt_);  // MinvJt_ = M⁻¹ Jᵀ   (nv×6)

    LambdaInv_.noalias() = J_full_ * MinvJt_;
    // Floor λ at the point of use so the singularity guard holds regardless of
    // how the gains were set (LoadConfig floors too, but set_gains()/the ctor
    // default bypass it) — NUM-1.
    const double lambda = std::max(1e-4, gains.damping);
    LambdaInv_.diagonal().array() += lambda * lambda;
    llt6_.compute(LambdaInv_);
    dyn_ok = (llt6_.info() == Eigen::Success);
  }

  if (dyn_ok) {
    // ── Step 8: task force  F = Λ a_task,  joint torque  τ = Jᵀ F + h ───────
    F_ = a_task_;
    llt6_.solveInPlace(F_);  // solve Λ⁻¹ F = a_task  ⇒  F = Λ a_task
    tau_out_.noalias() = Jt_ * F_;
    tau_out_ += h_;

    // ── Step 9: dynamically-consistent null-space posture task ──────────────
    // Redundancy resolution: project τ₀ through Nᵀ = I − Jᵀ J̄ᵀ (J̄ᵀ = Λ J M⁻¹).
    // Only meaningful for a genuinely redundant arm (nv > 6 task DOF): for nv==6
    // Nᵀ ≈ 0 (contributes nothing), and for nv < 6 the task is over-determined so
    // J M⁻¹ Jᵀ is rank-deficient and Nᵀ is NOT small — running the posture task
    // there would inject τ₀ into the primary Cartesian task (issue #172). Gate on
    // redundancy so a reduced-DOF arm on default null_kd is not coupled.
    if ((gains.null_kp != 0.0 || gains.null_kd != 0.0) && nv > 6) {
      JbarT_ = MinvJt_.transpose();  // (M⁻¹ Jᵀ)ᵀ = J M⁻¹   (6×nv)
      llt6_.solveInPlace(JbarT_);    // J̄ᵀ = Λ J M⁻¹
      NT_.setIdentity();
      NT_.noalias() -= Jt_ * JbarT_;  // Nᵀ = I − Jᵀ J̄ᵀ
      for (int i = 0; i < nv; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        const double q_ref = (ui < safe_position_.size()) ? safe_position_[ui] : 0.0;
        tau0_dev_[i] = gains.null_kp * (q_ref - q_buf[ui]) - gains.null_kd * v_buf[ui];
      }
      // safe_position_/q_buf/v_buf are device-order; gather the posture torque to
      // Pinocchio order before the null-space projection Nᵀ (Pinocchio) (#172 A2).
      // Identity order → memcpy (tau0_dev_ ≡ tau0_), so non-reordered arms are
      // byte-for-byte unchanged.
      handle_->ReorderInput(std::span<const double>(tau0_dev_.data(), static_cast<std::size_t>(nv)),
                            tau0_);
      null_tmp_.noalias() = NT_ * tau0_;
      tau_out_ += null_tmp_;
    }
  } else {
    // Degenerate M / Λ⁻¹: hold against gravity + Coriolis only (finite, safe).
    tau_out_ = h_;
  }

  // ── Step 10: emit torque command (N·m), clamp to per-joint torque limits ──
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;

  const int ncmd = std::min(nc0, nv);
  // tau_out_ is Pinocchio-order; scatter to device channel order so channel i
  // drives its own joint (#172 A2). Identity order → memcpy (unchanged).
  handle_->ReorderOutput(tau_out_,
                         std::span<double>(out0.commands.data(), static_cast<std::size_t>(ncmd)));
  rtc::utils::ClampSymmetric(out0.commands, nc0, std::span<const double>(max_joint_torque_),
                             kDefaultMaxJointTorque);

  for (std::size_t i = 0; i < 6; ++i) {
    out0.target_positions[i] = slot.pose_target[i];
    out0.goal_positions[i] = slot.pose_target[i];
  }

  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  Eigen::Vector3d rpy_current = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy_current[0];
  output.actual_task_positions[4] = rpy_current[1];
  output.actual_task_positions[5] = rpy_current[2];

  // Task-space goal target
  {
    Eigen::Vector3d goal_rpy = pinocchio::rpy::matrixToRpy(goal_pose_.rotation());
    output.task_goal_positions[0] = slot.pose_target[0];
    output.task_goal_positions[1] = slot.pose_target[1];
    output.task_goal_positions[2] = slot.pose_target[2];
    output.task_goal_positions[3] = goal_rpy[0];
    output.task_goal_positions[4] = goal_rpy[1];
    output.task_goal_positions[5] = goal_rpy[2];
  }

  // Task-space trajectory reference
  {
    const Eigen::Vector3d traj_pos = traj_state_.pose.translation();
    Eigen::Vector3d traj_rpy = pinocchio::rpy::matrixToRpy(traj_state_.pose.rotation());
    output.trajectory_task_positions[0] = traj_pos[0];
    output.trajectory_task_positions[1] = traj_pos[1];
    output.trajectory_task_positions[2] = traj_pos[2];
    output.trajectory_task_positions[3] = traj_rpy[0];
    output.trajectory_task_positions[4] = traj_rpy[1];
    output.trajectory_task_positions[5] = traj_rpy[2];

    output.trajectory_task_velocities[0] = traj_state_.velocity.linear()[0];
    output.trajectory_task_velocities[1] = traj_state_.velocity.linear()[1];
    output.trajectory_task_velocities[2] = traj_state_.velocity.linear()[2];
    output.trajectory_task_velocities[3] = traj_state_.velocity.angular()[0];
    output.trajectory_task_velocities[4] = traj_state_.velocity.angular()[1];
    output.trajectory_task_velocities[5] = traj_state_.velocity.angular()[2];
  }

  output.command_type = command_type_;
  return output;
}

void OperationalSpaceController::SetDeviceTarget(int device_idx,
                                                 std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) {
    return;
  }
  PendingTarget pending{};
  pending.device_idx = device_idx;
  pending.generation = ActivationGeneration();
  const std::size_t nch = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  pending.num_values = static_cast<int>(nch);
  for (std::size_t i = 0; i < nch; ++i) {
    pending.values[i] = target[i];
  }
  // Off-RT marshal — the RT thread drains pending_targets_ inside Compute()
  // and is the SOLE writer of target_seqlock_.
  (void)pending_targets_.Push(pending);
}

std::string_view OperationalSpaceController::Name() const noexcept {
  return "OperationalSpaceController";
}

void OperationalSpaceController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void OperationalSpaceController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  // Force the next Compute() to re-seed pose target + trajectory from the
  // current state (single-writer-safe: RT thread itself re-stores the slot).
  target_initialized_.store(false, std::memory_order_release);
}

bool OperationalSpaceController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void OperationalSpaceController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput OperationalSpaceController::ComputeEstop(const ControllerState& state,
                                                          const Gains& gains) noexcept {
  // Torque E-STOP hold (E-8, #184): τ = ĝ(q) − D·q̇, clamped per joint to ±τ_max.
  // ĝ(q) holds the arm against gravity; −D·q̇ bleeds residual kinetic energy to
  // rest. Replaces the pre-#184 position-scale slew, which emitted a joint-angle
  // magnitude on the kTorque command (a torque backend applied radians as N·m).
  // Coriolis is deliberately omitted (matches TaskImpedanceController::ComputeEstop
  // and the shared compliance::GravityCompDampedHold helper): at the low speeds a
  // safety stop targets, C(q,v)·v is small and the damping term dominates. This is
  // NOT merged with any global-E-STOP latch — estopped_ is the controller-local
  // flag, cleared only by ClearEstop() (no auto-recovery).
  const int nv = handle_->nv();
  const auto& dev0 = state.devices[0];

  // q (device order) for ĝ(q); q̇ read straight into device-order buffer.
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    qdot_dev_(i) = dev0.velocities[ui];
  }
  handle_->ComputeGeneralizedGravity(
      std::span<const double>(q_buf.data(), static_cast<std::size_t>(nv)));
  gravity_estop_ = handle_->GetGeneralizedGravity();  // Pinocchio order
  handle_->ReorderOutput(gravity_estop_,
                         std::span<double>(grav_dev_.data(), static_cast<std::size_t>(nv)));

  // max_joint_torque_ is grown to ≥ nv in OnDeviceConfigsSet, so this map is valid.
  Eigen::Map<Eigen::VectorXd> t_max(max_joint_torque_.data(), nv);
  compliance::GravityCompDampedHold(tau_dev_.head(nv), grav_dev_.head(nv), qdot_dev_.head(nv),
                                    gains.estop_damping, t_max);

  // A held tick is a discontinuity — force the next active tick to re-seed the
  // pose target + trajectory from the measured state (as ClearEstop also does).
  target_initialized_.store(false, std::memory_order_release);

  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  const int ncmd = std::min(nc0, nv);
  for (int i = 0; i < ncmd; ++i) {
    out0.commands[static_cast<std::size_t>(i)] = tau_dev_(i);
  }
  // Belt-and-braces bound on the torque command (the helper already clamps to the
  // same envelope; this also covers channels beyond nv on multi-device states).
  rtc::utils::ClampSymmetric(out0.commands, nc0, std::span<const double>(max_joint_torque_),
                             kDefaultMaxJointTorque);
  output.command_type = command_type_;
  return output;
}

Eigen::Matrix3d OperationalSpaceController::RpyToMatrix(double roll, double pitch,
                                                        double yaw) noexcept {
  // ZYX Euler convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// ── Controller registry hooks ────────────────────────────────────────────────

void OperationalSpaceController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  // A1 (#172): switch to the primary device's reduced submodel if one is declared
  // in the injected system model config (topic_config_ is populated by the base
  // LoadConfig above). All later Compute() dynamics use the submodel.
  MaybeSelectSubModel();
  if (!cfg) {
    return;
  }
  auto g = gains_lock_.Load();
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr) {
    if (n && n.IsSequence() && n.size() == 3) {
      for (std::size_t i = 0; i < 3; ++i) {
        arr[i] = n[i].as<double>();
      }
    }
  };
  load3(cfg["kp_pos"], g.kp_pos);
  load3(cfg["kd_pos"], g.kd_pos);
  load3(cfg["kp_rot"], g.kp_rot);
  load3(cfg["kd_rot"], g.kd_rot);
  if (cfg["damping"]) {
    // Floor λ > 0 (NUM-1/NUM-4): λ² regularises Λ⁻¹ at kinematic singularities.
    // A zero/negative damping would remove that guard and admit NaN torque.
    g.damping = std::max(1e-4, cfg["damping"].as<double>());
  }
  // Dynamically-consistent null-space posture gains (only bite when nv > 6).
  if (cfg["null_kp"]) {
    g.null_kp = cfg["null_kp"].as<double>();
  }
  if (cfg["null_kd"]) {
    g.null_kd = cfg["null_kd"].as<double>();
  }
  if (cfg["estop_damping"]) {
    // D ≥ 0: the torque E-STOP hold (#184) subtracts D·q̇ to bleed kinetic energy;
    // a negative D would inject energy (destabilising a safety stop).
    g.estop_damping = std::max(0.0, cfg["estop_damping"].as<double>());
  }
  if (cfg["enable_gravity_compensation"]) {
    // Deprecated: torque OSC always compensates g(q)+C·v. Parsed for YAML
    // back-compat but has no effect on the control law.
    g.enable_gravity_compensation = cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["trajectory_angular_speed"]) {
    g.trajectory_angular_speed = std::max(1e-6, cfg["trajectory_angular_speed"].as<double>());
  }
  if (cfg["max_traj_velocity"]) {
    g.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    g.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }
  // OSC is a torque controller (operational-space law outputs N·m). Reject any
  // other command_type fail-fast instead of silently mislabelling the output.
  // Position/velocity task control belongs to ClikController. Validate BEFORE
  // committing the parsed gains, so a rejected reconfigure does not mutate the
  // live RT gains (issue #172 — non-atomic config application).
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque") {
      throw std::runtime_error("OperationalSpaceController: command_type must be 'torque' (got '" +
                               s + "'); use ClikController for position/velocity task control");
    }
  }
  gains_lock_.Store(g);
  command_type_ = CommandType::kTorque;
}

}  // namespace rtc
