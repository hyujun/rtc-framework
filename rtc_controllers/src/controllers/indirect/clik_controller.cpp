// ── Includes: project header first, then C++ stdlib
// ────────────────────────────
#include "rtc_controllers/indirect/clik_controller.hpp"

#include "rtc_base/utils/device_passthrough.hpp"
#include "rtc_controllers/joint/posture_law.hpp"
#include "rtc_controllers/task/task_vel_law.hpp"
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

ClikController::ClikController(std::string_view urdf_path, Gains gains) : gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";

  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  InitFromModel(builder.GetFullModel());
}

void ClikController::InitFromModel(std::shared_ptr<const pinocchio::Model> model) {
  model_ptr_ = std::move(model);
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);

  // Default: use the last frame in the model as tip
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);

  const int nv = handle_->nv();
  // Fail-fast capacity check at model-load time (off-RT): TargetSlot::null_target
  // is kMaxRobotDOF-wide, so a larger model would overrun it in Compute.
  // Enforcing it here guarantees the invariant even when LoadConfig is never
  // called or is handed a null node (issue #172).
  if (nv > kMaxRobotDOF) {
    throw std::runtime_error(
        "ClikController: model DOF nv=" + std::to_string(nv) +
        " exceeds fixed capacity kMaxRobotDOF=" + std::to_string(kMaxRobotDOF));
  }

  // Pre-allocate all Eigen buffers to their final sizes (nv-dependent → must be
  // re-sized whenever the model changes, e.g. submodel selection — #172 A1).
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  J_pos_ = Eigen::MatrixXd::Zero(3, nv);
  dq_ = Eigen::VectorXd::Zero(nv);
  desired_q_ = Eigen::VectorXd::Zero(nv);
  traj_dq_ = Eigen::VectorXd::Zero(nv);
  null_dq_dev_ = Eigen::VectorXd::Zero(nv);
  qdot_null_ = Eigen::VectorXd::Zero(nv);
  pos_error_ = Eigen::Vector3d::Zero();
  pos_error_6d_ = Eigen::Matrix<double, 6, 1>::Zero();

  // Both task dimensions are sized here, off-RT: `control_6dof` is a SeqLock
  // gain, so which one a tick uses is not known until that tick reads it.
  ik_6d_.Resize(nv, 6);
  ik_3d_.Resize(nv, 3);
}

void ClikController::MaybeSelectSubModel() {
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
    RCLCPP_ERROR(rclcpp::get_logger("ClikController"),
                 "[ClikController] reduced submodel '%s' unavailable (%s) — keeping full model",
                 primary.c_str(), e.what());
  }
}

void ClikController::OnDeviceConfigsSet() {
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
    }
    if (!cfg->safe_position.empty()) {
      safe_position_ = cfg->safe_position;
      // Seed the off-RT null-space init vector; the RT thread copies it
      // into the TargetSlot on the first Compute() tick.
      for (std::size_t i = 0; i < std::min(cfg->safe_position.size(), null_target_init_.size());
           ++i) {
        null_target_init_[i] = cfg->safe_position[i];
      }
    }
    // Register device→model joint reorder (#172 A2): the model consumes
    // device-order q and dq/traj_dq are scattered back to device channel order.
    // Identity order → HasJointReorder()==false → zero-overhead passthrough.
    const auto js = static_cast<int>(cfg->joint_state_names.size());
    const int nv = handle_->nv();
    if (js == nv) {
      if (!handle_->SetJointOrder(cfg->joint_state_names)) {
        RCLCPP_ERROR(rclcpp::get_logger("ClikController"),
                     "[ClikController] primary device '%s' joint_state_names not all in model — "
                     "reorder disabled, falling back to positional (URDF-order) consumption",
                     primary.c_str());
      }
    } else if (js > 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ClikController"),
                   "[ClikController] primary device '%s' joint_state_names size=%d != model DOF "
                   "nv=%d — FK/Jacobian state mapping will be inconsistent",
                   primary.c_str(), js, nv);
    }
  }
  if (max_joint_velocity_.empty()) {
    max_joint_velocity_.assign(kMaxDeviceChannels, kDefaultMaxJointVelocity);
  }
  if (safe_position_.empty()) {
    safe_position_.assign(kMaxDeviceChannels, 0.0);
  }
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput ClikController::Compute(const ControllerState& state) noexcept {
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const int nv = handle_->nv();

  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  const bool use_6dof = gains.control_6dof;
  const bool use_null_space = gains.enable_null_space;

  // ── Step 1: copy joint state into q buffer ──────────────────────────────
  const auto& dev0 = state.devices[0];
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    q_buf[static_cast<std::size_t>(i)] = dev0.positions[static_cast<std::size_t>(i)];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));

  // ── Step 2: FK + Jacobians ───────────────────────────────────────────────
  // ComputeJacobians performs FK internally and updates frame placements.
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  // Translational Jacobian: rows 0..2
  J_pos_.noalias() = J_full_.topRows(3);

  // ── Step 3: Cartesian position/orientation error ──────────────────────────
  const pinocchio::SE3& tcp_pose = handle_->GetFramePlacement(tip_frame_id_);
  const Eigen::Vector3d tcp = tcp_pose.translation();

  // ── Target slot maintenance (RT thread is the single SeqLock writer) ──
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;

  if (!target_initialized_.load(std::memory_order_acquire)) {
    slot.tcp_target = {tcp[0], tcp[1], tcp[2]};
    std::memcpy(slot.tcp_target_rot.data(), tcp_pose.rotation().data(),
                sizeof(slot.tcp_target_rot));
    std::memcpy(slot.tcp_target_t.data(), tcp_pose.translation().data(), sizeof(slot.tcp_target_t));
    for (int i = 0; i < nv; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      // Prefer YAML safe_position via null_target_init_; fall back to the
      // current joint pose if no safe_position was supplied.
      slot.null_target[idx] =
          (null_target_init_[idx] != 0.0) ? null_target_init_[idx] : dev0.positions[idx];
    }
    for (std::size_t didx = 1; didx < static_cast<std::size_t>(state.num_devices); ++didx) {
      const auto& dev = state.devices[didx];
      if (!dev.valid) {
        continue;
      }
      const std::size_t nch = std::min(static_cast<std::size_t>(dev.num_channels),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i) {
        slot.targets[didx][i] = dev.positions[i];
      }
    }
    // Seed RT-thread working SE3 + integration state.
    tcp_target_pose_ = tcp_pose;
    for (int i = 0; i < nv; ++i) {
      desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
    }
    trajectory_.initialize(tcp_pose, pinocchio::Motion::Zero(), tcp_pose, pinocchio::Motion::Zero(),
                           0.01);
    trajectory_time_ = 0.0;
    has_pending_segment_ = false;
    new_target_pending_ = false;
    target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
  } else {
    // Restore RT-thread working SE3 from POD storage so diagnostic echoes
    // see a consistent rotation matrix every tick.
    std::memcpy(tcp_target_pose_.rotation().data(), slot.tcp_target_rot.data(),
                sizeof(slot.tcp_target_rot));
    std::memcpy(tcp_target_pose_.translation().data(), slot.tcp_target_t.data(),
                sizeof(slot.tcp_target_t));
  }

  // Drain SPSC entries from off-RT SetDeviceTarget.
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
      if (gains_lock_.Load().control_6dof) {
        if (pending.num_values >= 6) {
          slot.tcp_target[0] = pending.values[0];
          slot.tcp_target[1] = pending.values[1];
          slot.tcp_target[2] = pending.values[2];
          Eigen::AngleAxisd rollAngle(pending.values[3], Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd pitchAngle(pending.values[4], Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd yawAngle(pending.values[5], Eigen::Vector3d::UnitZ());
          const Eigen::Quaternion<double> qrot = yawAngle * pitchAngle * rollAngle;
          const Eigen::Matrix3d rotation = qrot.matrix();
          const Eigen::Vector3d translation(pending.values[0], pending.values[1],
                                            pending.values[2]);
          std::memcpy(slot.tcp_target_rot.data(), rotation.data(), sizeof(slot.tcp_target_rot));
          std::memcpy(slot.tcp_target_t.data(), translation.data(), sizeof(slot.tcp_target_t));
          tcp_target_pose_.translation() = translation;
          tcp_target_pose_.rotation() = rotation;
          new_target_pending_ = true;
        }
      } else {
        const auto nv_u = static_cast<std::size_t>(handle_->nv());
        const std::size_t pn = std::min(static_cast<std::size_t>(pending.num_values), nv_u);
        for (std::size_t i = 0; i < std::min(pn, std::size_t{3}); ++i) {
          slot.tcp_target[i] = pending.values[i];
        }
        for (std::size_t i = 3; i < pn; ++i) {
          slot.null_target[i] = pending.values[i];
        }
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

  if (new_target_pending_) {
    {
      pinocchio::SE3 start_pose = tcp_pose;
      pinocchio::SE3 goal_pose;

      if (use_6dof) {
        goal_pose = tcp_target_pose_;
      } else {
        goal_pose = start_pose;  // keep current rotation
        goal_pose.translation() =
            Eigen::Vector3d(slot.tcp_target[0], slot.tcp_target[1], slot.tcp_target[2]);
      }

      const Eigen::Vector3d start_pos = start_pose.translation();
      const Eigen::Vector3d goal_pos = goal_pose.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      // Duration from translational trajectory_speed and velocity limit.
      // Quintic rest-to-rest peak velocity = (15/8) * dist / T.
      const double T_speed_trans = trans_dist / gains.trajectory_speed;
      const double T_vel_trans =
          (gains.max_traj_velocity > 0.0) ? (1.875 * trans_dist / gains.max_traj_velocity) : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      // Angular distance via AngleAxisd (stable at θ → π, unlike log3)
      constexpr double kPiSafetyMargin = 0.15;  // rad ≈ 8.6°
      double angular_dist = 0.0;
      Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ();  // fallback
      bool split_trajectory = false;

      if (use_6dof) {
        const Eigen::AngleAxisd aa(start_pose.rotation().transpose() * goal_pose.rotation());
        angular_dist = aa.angle();  // always in [0, π]
        rot_axis = aa.axis();

        const double T_speed_rot = angular_dist / gains.trajectory_angular_speed;
        const double T_vel_rot = (gains.max_traj_angular_velocity > 0.0)
                                     ? (1.875 * angular_dist / gains.max_traj_angular_velocity)
                                     : 0.0;
        duration = std::max({duration, T_speed_rot, T_vel_rot});

        split_trajectory = (angular_dist > M_PI - kPiSafetyMargin);
      }

      if (split_trajectory) {
        // ── π-rotation defense: split into 2 rest-to-rest segments ──
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid =
            start_pose.rotation() * Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

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

        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), mid_pose,
                               pinocchio::Motion::Zero(), dur1);
        pending_goal_pose_ = goal_pose;
        pending_duration_ = dur1;  // symmetric split
        has_pending_segment_ = true;
      } else {
        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), goal_pose,
                               pinocchio::Motion::Zero(), duration);
        has_pending_segment_ = false;
      }

      trajectory_time_ = 0.0;
      // Initialize desired_q_ from current actual joint positions
      for (int i = 0; i < nv; ++i) {
        desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
      }
      new_target_pending_ = false;
    }
  }

  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
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

  tcp_position_ = {tcp[0], tcp[1], tcp[2]};

  // BodyLog6 = log6(T_cur⁻¹ T_d): body-frame screw error (preserves the
  // position-rotation coupling), via rtc_math. The error is returned in the
  // LOCAL frame; J_full_ is expressed in LOCAL_WORLD_ALIGNED, so transport
  // LOCAL → LWA (rotation-only blockdiag(R,R), NOT the full adjoint).
  const rtc::math::se3::Iso3 tcp_iso = rtc::math::se3::toIso3(tcp_pose);
  const rtc::math::se3::Vec6 e_body = rtc::math::se3::computePoseError(
      tcp_iso, rtc::math::se3::toIso3(traj_state_.pose), rtc::math::se3::ErrorType::BodyLog6);
  pos_error_6d_ = rtc::math::se3::twistLocalToWorld(tcp_iso, e_body);

  const Eigen::Vector3d p_err = pos_error_6d_.head<3>();

  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = p_err[i];
  }

  // Feedforward: trajectory local → world-aligned via R_trajectory (not
  // R_current). The transport stays HERE with the trajectory sample it belongs
  // to; the law takes an already world-aligned twist (#236 S3a). Computed ONCE
  // per tick — the primary task and the feedforward-only logging lane below both
  // read it off the same unchanged traj_state_, so the rotation products are not
  // repeated on the RT path.
  const Eigen::Matrix3d& R_traj = traj_state_.pose.rotation();
  const Eigen::Vector3d nu_ff_lin = R_traj * traj_state_.velocity.linear();

  // ── Step 4, 5 & 6: DLS pseudoinverse, primary task, null-space task ───────
  // #236 S3b (issue #258): these two branches used to inline a constant-λ DLS
  // each — LDLT of J Jᵀ + λ²I, the inverse materialised, then Jᵀ·(J Jᵀ)⁻¹. They
  // are now compliance::DifferentialIk, the same helper TaskAdmittanceController
  // uses. Three deliberate changes, only the first of which is a law change
  // (D-S2b, option (b) — the convergence is the point of the migration):
  //   • λ² is σ_min-adaptive (§6.5) instead of a constant damping². Away from
  //     singularities the damping is now exactly 0.
  //   • LLT of a solve, not LDLT of a materialised inverse, at dynamic rather
  //     than fixed size. Pure reassociation: max relative difference ≤ 1.7e-12.
  //   • the posture gain multiplies BEFORE the projection (N·(kp·Δq)) rather
  //     than after (kp·(N·Δq)). Algebraically identical, bitwise not — a probe
  //     measured 68.36% of doubles differing at the last bits. Multiplying first
  //     is what lets the posture term be joint::ComputePostureVelocity, the same
  //     law the other four consumers call; #236 S6 measured this difference and
  //     deferred the choice to this slice rather than pre-empt it.
  // `ok == false` means J⁺/N are STALE (a non-finite J, i.e. a NaN joint state
  // reaching FK). This controller integrates its output into a position command,
  // so a stale inverse would be latched forever: hold instead, exactly as
  // TaskAdmittanceController does. The inline LDLT never checked .info() at all.
  //
  //     The trailing traj_dq_ assignment in each branch is the feedforward-only
  //     lane (for logging): the same ν_ff through J⁺, with no P term.
  const bool nullspace_active = use_null_space && !use_6dof;
  if (nullspace_active) {
    // Posture task — joint/posture_law.hpp (#236 S6). The P form is its OWN core
    // function, not the PD one with K_d = 0: that reduction flips the sign of a
    // zero, so it is not bitwise inert. Reference and measurement are both
    // device-order here, so the law runs in device order and the gather to
    // Pinocchio order happens on the RESULT — the projector N is Pinocchio-
    // ordered (#172 A2). Identity order → memcpy (unchanged).
    const auto nvu = static_cast<std::size_t>(nv);
    joint::ComputePostureVelocity(
        gains.null_kp,
        joint::PostureInputs{{slot.null_target.data(), slot.null_target.size()},
                             {dev0.positions.data(), dev0.positions.size()},
                             {}},
        nvu, {null_dq_dev_.data(), nvu});
    handle_->ReorderInput(std::span<const double>(null_dq_dev_.data(), nvu), qdot_null_);
  } else {
    qdot_null_.setZero();
  }

  // Floor λ_max at the point of use so the singularity guard holds regardless of
  // how the gains were set (LoadConfig floors too, but set_gains() bypasses
  // it) — NUM-1.
  const double max_damping = std::max(compliance::kMinMaxDamping, gains.max_damping);
  if (use_6dof) {
    const compliance::DifferentialIk::Result r =
        ik_6d_.Compute(J_full_, gains.singularity_threshold, max_damping);

    Eigen::Matrix<double, 6, 1> nu_ff_6d;
    nu_ff_6d.head<3>() = nu_ff_lin;
    nu_ff_6d.tail<3>() = R_traj * traj_state_.velocity.angular();

    const Eigen::Matrix<double, 6, 1> task_vel_6d = rtc::task::ComputeTaskVelocity(
        rtc::task::TaskVelParams{gains.kp_translation, gains.kp_rotation}, pos_error_6d_, nu_ff_6d);

    if (r.ok) {
      // No posture term in this mode — the gate above is `!use_6dof`, so
      // qdot_null_ is identically zero here. The two-argument overload is what
      // the pre-migration 6-DOF branch was (its null-space block sat outside the
      // branch entirely); routing it through the three-argument form added an
      // nv×nv product against a zero vector on every tick, forever.
      ik_6d_.Solve(task_vel_6d, dq_);
      traj_dq_.noalias() = ik_6d_.PseudoInverse() * nu_ff_6d;
    } else {
      dq_.setZero();
      traj_dq_.setZero();
    }
  } else {
    // 3D version
    const compliance::DifferentialIk::Result r =
        ik_3d_.Compute(J_pos_, gains.singularity_threshold, max_damping);

    const Eigen::Vector3d task_vel =
        rtc::task::ComputeTranslationVelocity(gains.kp_translation, pos_error_, nu_ff_lin);

    if (r.ok) {
      ik_3d_.Solve(task_vel, qdot_null_, dq_);
      traj_dq_.noalias() = ik_3d_.PseudoInverse() * nu_ff_lin;
    } else {
      dq_.setZero();
      traj_dq_.setZero();
    }
  }

  // ── Step 7: Clamp joint velocity and integrate ────────────────────────────
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;
  // Model-dimension bound (issue #172 OOB): dq_/desired_q_/traj_dq_ are nv-sized
  // and null_target is kMaxRobotDOF-wide, but nc0 (device channel count) is
  // bounded only by kMaxDeviceChannels. Indexing them past nv/kMaxRobotDOF is a
  // heap/stack overrun. Channels in [nq, nc0) have no CLIK joint solution and
  // hold their current position.
  const std::size_t nq =
      std::min(static_cast<std::size_t>(nc0), static_cast<std::size_t>(handle_->nv()));

  // dq_/traj_dq_ are Pinocchio-order (Jpinv_ has Pinocchio columns); scatter to
  // device channel order before clamping (limits are device-order) and integrating
  // (#172 A2). desired_q_ stays device-order throughout (seeded from device
  // positions, never fed back into the model), so no reorder on the integrator.
  // Identity order → memcpy (unchanged).
  handle_->ReorderOutput(dq_, std::span<double>(out0.target_velocities.data(), nq));
  ClampVelocity(out0.target_velocities, nc0);
  handle_->ReorderOutput(traj_dq_, std::span<double>(out0.trajectory_velocities.data(), nq));

  for (std::size_t i = 0; i < nq; ++i) {
    desired_q_[static_cast<Eigen::Index>(i)] += out0.target_velocities[i] * dt;
    out0.commands[i] = desired_q_[static_cast<Eigen::Index>(i)];
    out0.trajectory_positions[i] = desired_q_[static_cast<Eigen::Index>(i)];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out0.target_positions[i] = traj_state_.pose.translation()[static_cast<Eigen::Index>(i)];
    // goal_positions: task-space goal in [0..2], null-space goal in [3..nq)
    out0.goal_positions[i] = slot.tcp_target[i];
  }
  for (std::size_t i = 3; i < nq; ++i) {
    out0.target_positions[i] = slot.null_target[i];
    out0.goal_positions[i] = slot.null_target[i];
  }
  // Channels past the model DOF (nc0 > nv) hold current position.
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = dev0.positions[i];
    out0.trajectory_positions[i] = dev0.positions[i];
  }
  for (std::size_t i = std::max<std::size_t>(nq, 3); i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = dev0.positions[i];
    out0.goal_positions[i] = dev0.positions[i];
  }

  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  const pinocchio::SE3& tcp_current = handle_->GetFramePlacement(tip_frame_id_);
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp_current.rotation());
  output.actual_task_positions[0] = tcp_current.translation().x();
  output.actual_task_positions[1] = tcp_current.translation().y();
  output.actual_task_positions[2] = tcp_current.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  // Task-space goal target
  output.task_goal_positions[0] = slot.tcp_target[0];
  output.task_goal_positions[1] = slot.tcp_target[1];
  output.task_goal_positions[2] = slot.tcp_target[2];
  if (use_6dof) {
    Eigen::Vector3d goal_rpy = pinocchio::rpy::matrixToRpy(tcp_target_pose_.rotation());
    output.task_goal_positions[3] = goal_rpy[0];
    output.task_goal_positions[4] = goal_rpy[1];
    output.task_goal_positions[5] = goal_rpy[2];
  } else {
    output.task_goal_positions[3] = rpy[0];
    output.task_goal_positions[4] = rpy[1];
    output.task_goal_positions[5] = rpy[2];
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

void ClikController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
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

std::string_view ClikController::Name() const noexcept {
  return "ClikController";
}

void ClikController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void ClikController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool ClikController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void ClikController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput ClikController::ComputeEstop(const ControllerState& state) noexcept {
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim =
        (i < max_joint_velocity_.size()) ? max_joint_velocity_[i] : kDefaultMaxJointVelocity;
    const double sp = (i < safe_position_.size()) ? safe_position_[i] : 0.0;
    out0.commands[i] = dev0.positions[i] + std::clamp(sp - dev0.positions[i], -lim, lim) *
                                               ((state.dt > 0.0) ? state.dt : GetDefaultDt());
  }
  return output;
}

void ClikController::ClampVelocity(std::array<double, kMaxDeviceChannels>& dq,
                                   int n) const noexcept {
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lim =
        (i < max_joint_velocity_.size()) ? max_joint_velocity_[i] : kDefaultMaxJointVelocity;
    dq[i] = std::clamp(dq[i], -lim, lim);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void ClikController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  // A1 (#172): switch to the primary device's reduced submodel if one is declared
  // in the injected system model config (topic_config_ is populated by the base
  // LoadConfig above). All later Compute() FK/Jacobian use the submodel.
  MaybeSelectSubModel();
  if (!cfg) {
    return;
  }

  // nv <= kMaxRobotDOF is already guaranteed by the constructor's fail-fast
  // capacity check.
  auto g = gains_lock_.Load();

  // CLIK gains — translation / rotation separated
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr) {
    if (n && n.IsSequence() && n.size() >= 3) {
      for (std::size_t i = 0; i < 3; ++i) {
        arr[i] = n[i].as<double>();
      }
    }
  };
  load3(cfg["kp_translation"], g.kp_translation);
  load3(cfg["kp_rotation"], g.kp_rotation);

  if (cfg["max_damping"]) {
    // Floor the damped-least-squares λ_max so a zero/negative value cannot remove
    // the singularity guard (NUM-1); a singular J Jᵀ would otherwise yield a NaN
    // joint-velocity command. `max_damping` replaced the constant-λ `damping` key
    // in #236 S3b — same role in the law, now the ceiling of the §6.5 ramp rather
    // than a constant, and the same key the other three task-space controllers
    // already use. Mirrors the OperationalSpaceController floor.
    g.max_damping = std::max(compliance::kMinMaxDamping, cfg["max_damping"].as<double>());
  }
  if (cfg["singularity_threshold"]) {
    // Floor σ₀ > 0 (NUM-2). AdaptiveDampingSquared short-circuits to λ² = 0 when
    // σ₀ ≤ 0, so a zero here removes the §6.5 ramp entirely instead of narrowing
    // it — and with λ² ≡ 0 a singular pose makes DifferentialIk report !ok, which
    // this controller answers by holding at zero velocity every tick. Clamped the
    // same way by the other three task-space controllers.
    g.singularity_threshold =
        std::max(compliance::kMinSigma0, cfg["singularity_threshold"].as<double>());
  }
  if (cfg["damping"]) {
    // Retired in #236 S3b (#258). LoadConfig ignores unknown keys, so without
    // this an existing deployment config would keep parsing clean while its
    // singularity behaviour silently changed. Warned and ignored rather than
    // mapped onto max_damping: a constant λ and the ceiling of a σ_min-adaptive
    // ramp are not the same quantity, so any mapping would be a guess.
    RCLCPP_WARN(rclcpp::get_logger("ClikController"),
                "[ClikController] 'damping' is retired (#236 S3b) and IGNORED — use 'max_damping' "
                "(λ_max, default 0.05) and 'singularity_threshold' (σ₀, default 0.02); see "
                "rtc_controllers/README.md");
  }
  if (cfg["null_kp"]) {
    g.null_kp = cfg["null_kp"].as<double>();
  }
  if (cfg["enable_null_space"]) {
    g.enable_null_space = cfg["enable_null_space"].as<bool>();
  }
  if (cfg["control_6dof"]) {
    g.control_6dof = cfg["control_6dof"].as<bool>();
  }

  // Trajectory speed
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["trajectory_angular_speed"]) {
    g.trajectory_angular_speed = std::max(1e-6, cfg["trajectory_angular_speed"].as<double>());
  }

  // Trajectory velocity limits
  if (cfg["max_traj_velocity"]) {
    g.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    g.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }

  gains_lock_.Store(g);

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

}  // namespace rtc
