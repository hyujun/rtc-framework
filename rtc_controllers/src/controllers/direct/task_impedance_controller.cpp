// ── Includes: project header first, then C++ stdlib ─────────────────────────
#include "rtc_controllers/direct/task_impedance_controller.hpp"

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
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace rtc {

namespace {
// DEGRADED → RUNNING recovery dwell (§10.6 default). Not a per-robot tuning knob
// in slice 1 — promoted to a Gains field only if a deployment needs it.
constexpr double kDegradedRecoveryTime = 0.5;  // s
}  // namespace

// ── Constructor ─────────────────────────────────────────────────────────────

TaskImpedanceController::TaskImpedanceController(std::string_view urdf_path, Gains gains,
                                                 TaskSelection selection)
    : selection_(selection),
      m_(selection == TaskSelection::kTranslationOnly ? 3 : 6),
      gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  InitFromModel(builder.GetFullModel());
}

void TaskImpedanceController::InitFromModel(std::shared_ptr<const pinocchio::Model> model) {
  model_ptr_ = std::move(model);
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);

  const int nv = handle_->nv();
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  J_S_ = Eigen::MatrixXd::Zero(m_, nv);
  M_ = Eigen::MatrixXd::Zero(nv, nv);
  gravity_ = Eigen::VectorXd::Zero(nv);
  tau_ = Eigen::VectorXd::Zero(nv);
  tau_posture_dev_ = Eigen::VectorXd::Zero(nv);
  tau_posture_ = Eigen::VectorXd::Zero(nv);
  tau_null_ = Eigen::VectorXd::Zero(nv);
  tcp_vel_ = Eigen::VectorXd::Zero(6);
  q_null_ = Eigen::VectorXd::Zero(nv);
  tau_dev_ = Eigen::VectorXd::Zero(nv);
  tau_prev_dev_ = Eigen::VectorXd::Zero(nv);
  q_dev_ = Eigen::VectorXd::Zero(nv);
  qdot_dev_ = Eigen::VectorXd::Zero(nv);
  grav_dev_ = Eigen::VectorXd::Zero(nv);
  llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv);
  dyn_.Resize(nv, m_);

  // Size the per-joint limit vectors to nv up front with inert defaults so the
  // safety layer has valid bounds even if OnDeviceConfigsSet is never called
  // (e.g. a direct unit-test construction). OnDeviceConfigsSet overwrites/pads.
  const auto nvz = static_cast<std::size_t>(nv);
  max_joint_torque_.assign(nvz, kDefaultMaxJointTorque);
  max_joint_velocity_.assign(nvz, kDefaultMaxJointVelocity);
  position_lower_.assign(nvz, -1e9);
  position_upper_.assign(nvz, 1e9);
}

void TaskImpedanceController::MaybeSelectSubModel() {
  const auto* sys = GetSystemModelConfig();
  if (sys == nullptr || sys->urdf_path.empty() || sys->sub_models.empty())
    return;
  const auto primary = GetPrimaryDeviceName();
  if (primary.empty())
    return;
  const bool matches =
      std::any_of(sys->sub_models.begin(), sys->sub_models.end(),
                  [&](const rtc_urdf_bridge::SubModelConfig& sm) { return sm.name == primary; });
  if (!matches)
    return;
  auto builder = GetSharedModelBuilder();
  if (!builder)
    builder = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(*sys);
  try {
    InitFromModel(builder->GetReducedModel(primary));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("TaskImpedanceController"),
                 "[TaskImpedanceController] reduced submodel '%s' unavailable (%s) — keeping full",
                 primary.c_str(), e.what());
  }
}

void TaskImpedanceController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0)
        tip_frame_id_ = fid;
    }
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
      max_joint_torque_ = cfg->joint_limits->max_torque;
      position_lower_ = cfg->joint_limits->position_lower;
      position_upper_ = cfg->joint_limits->position_upper;
    }
    const auto js = static_cast<int>(cfg->joint_state_names.size());
    const int nv = handle_->nv();
    if (js == nv) {
      if (!handle_->SetJointOrder(cfg->joint_state_names))
        RCLCPP_ERROR(rclcpp::get_logger("TaskImpedanceController"),
                     "[TaskImpedanceController] primary device '%s' joint_state_names not all in "
                     "model — reorder disabled",
                     primary.c_str());
    } else if (js > 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("TaskImpedanceController"),
          "[TaskImpedanceController] primary device '%s' joint_state_names size=%d != nv=%d",
          primary.c_str(), js, nv);
    }
  }
  // Guarantee every limit vector is exactly nv-sized so the RT safety maps have
  // valid bounds (config may supply fewer/none; a short vector would read OOB).
  // Absent position limits → a wide band so the §5.3 repulsive term stays inert.
  const auto nvz = static_cast<std::size_t>(handle_->nv());
  auto pad = [nvz](std::vector<double>& v, double fill) { v.resize(nvz, fill); };
  pad(max_joint_velocity_, kDefaultMaxJointVelocity);
  pad(max_joint_torque_, kDefaultMaxJointTorque);
  pad(position_lower_, -1e9);
  pad(position_upper_, 1e9);
}

// ── RTControllerInterface ────────────────────────────────────────────────────

ControllerOutput TaskImpedanceController::Compute(const ControllerState& state) noexcept {
  Diagnostics diag;
  diag.estopped = estopped_.load(std::memory_order_acquire);

  // A latched controller-local SAFE_STOP is cleared only by ResetFault(), never
  // by ClearEstop (E-8). Consume the request edge before stepping the machine.
  if (reset_fault_requested_.exchange(false, std::memory_order_acq_rel))
    sm_.ResetFault();

  // Surface the latched controller-local FSM state (a SAFE_STOP/DEGRADED latch
  // persists across a global E-STOP) so the estop early-return below publishes
  // the real state instead of a spurious HOLDING. The normal control path
  // recomputes diag.state after Step().
  diag.state = static_cast<std::uint8_t>(sm_.state());

  // One SeqLock read per RT tick, shared by every path: the global-E-STOP hold
  // and the SAFE_STOP hold both consume it through ComputeEstop below.
  const auto gains = gains_lock_.Load();

  if (diag.estopped) {
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
    return ComputeEstop(state, /*control_valid=*/false, diag, gains);
  }

  const int nv = handle_->nv();
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  // ── Copy joint state (device order) ─────────────────────────────────────
  const auto& dev0 = state.devices[0];
  std::array<double, kMaxDeviceChannels> q_buf{};
  std::array<double, kMaxDeviceChannels> v_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    v_buf[ui] = dev0.velocities[ui];
    q_dev_(i) = dev0.positions[ui];
    qdot_dev_(i) = dev0.velocities[ui];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));

  // ── FK + Jacobian + current task twist ──────────────────────────────────
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  Eigen::Map<const Eigen::VectorXd> v_eigen(v_buf.data(), nv);
  tcp_vel_.noalias() = J_full_ * v_eigen;
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);

  // ── Target slot: seed X_d/q_null from measured on (re)activation (§10.7) ──
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;
  bool just_seeded = false;
  if (!target_initialized_.load(std::memory_order_acquire)) {
    goal_pose_ = tcp;
    std::memcpy(slot.goal_rot.data(), tcp.rotation().data(), sizeof(slot.goal_rot));
    std::memcpy(slot.goal_t.data(), tcp.translation().data(), sizeof(slot.goal_t));
    for (int i = 0; i < nv; ++i)
      q_null_(i) = q_dev_(i);   // posture target = measured (device order)
    activation_elapsed_ = 0.0;  // restart the gain ramp
    saturation_elapsed_ = 0.0;
    just_seeded = true;  // seed the rate-limit history to this tick's own
                         // command below, so activation is not slew-limited
    for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
      const auto& dev = state.devices[d];
      if (!dev.valid)
        continue;
      const std::size_t nch = std::min(static_cast<std::size_t>(dev.num_channels),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i)
        slot.targets[d][i] = dev.positions[i];
    }
    target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
  } else {
    std::memcpy(goal_pose_.rotation().data(), slot.goal_rot.data(), sizeof(slot.goal_rot));
    std::memcpy(goal_pose_.translation().data(), slot.goal_t.data(), sizeof(slot.goal_t));
  }

  // Drain off-RT targets (device 0 = SE3 pose; others = joint passthrough).
  PendingTarget pending{};
  while (pending_targets_.Pop(pending)) {
    if (!IsCurrentGeneration(pending.generation))
      continue;
    const auto didx = static_cast<std::size_t>(pending.device_idx);
    if (didx >= ControllerState::kMaxDevices)
      continue;
    if (didx == 0 && pending.num_values >= 6) {
      const Eigen::Vector3d p(pending.values[0], pending.values[1], pending.values[2]);
      const Eigen::Matrix3d R = (Eigen::AngleAxisd(pending.values[5], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(pending.values[4], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(pending.values[3], Eigen::Vector3d::UnitX()))
                                    .toRotationMatrix();
      goal_pose_.translation() = p;
      goal_pose_.rotation() = R;
      std::memcpy(slot.goal_rot.data(), R.data(), sizeof(slot.goal_rot));
      std::memcpy(slot.goal_t.data(), p.data(), sizeof(slot.goal_t));
    } else if (didx > 0) {
      const std::size_t nch = std::min(static_cast<std::size_t>(pending.num_values),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i)
        slot.targets[didx][i] = pending.values[i];
    }
    slot_dirty = true;
  }
  if (slot_dirty)
    target_seqlock_.Store(slot);

  // ── Pose / velocity error (SplitWorld = LOCAL_WORLD_ALIGNED) ─────────────
  const Eigen::Matrix<double, 6, 1> e =
      rtc::math::se3::computePoseError(tcp, goal_pose_, rtc::math::se3::ErrorType::SplitWorld);
  // SplitWorld velocity error is ν_d − ν (LWA); the desired task velocity ν_d is
  // 0 for a static compliance setpoint, so ė = −ν = −J·q̇. (computeVelocityError
  // returns exactly this for Split types and takes no SE3 overload — inline it.)
  const Eigen::Matrix<double, 6, 1> edot = -tcp_vel_;

  for (int i = 0; i < 6; ++i)
    diag.pose_error[static_cast<std::size_t>(i)] = e(i);

  // Gain ramp α (§10.7): 0→1 over activation_ramp_time; ≤0 disables.
  const double ramp = gains.activation_ramp_time;
  const double alpha = (ramp <= 0.0) ? 1.0 : std::min(1.0, activation_elapsed_ / ramp);

  // ── Selection: J_S = S·J (linear rows first in [linear;angular]) ─────────
  // f_task is fixed-size (stack) so Compute stays heap-free; only head(m_) used.
  J_S_ = J_full_.topRows(m_);
  Eigen::Matrix<double, 6, 1> f_task = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < m_; ++i) {
    const double kp = (i < 3) ? gains.kp_pos[static_cast<std::size_t>(i)]
                              : gains.kp_rot[static_cast<std::size_t>(i - 3)];
    const double kd = (i < 3) ? gains.kd_pos[static_cast<std::size_t>(i)]
                              : gains.kd_rot[static_cast<std::size_t>(i - 3)];
    f_task(i) = alpha * (kp * e(i) + kd * edot(i));  // +K_p·e (sign per §6.2)
  }

  // ── Joint-space gravity ĝ(q) (always needed: comp + E-STOP hold) ─────────
  handle_->ComputeGeneralizedGravity(q_span);
  gravity_ = handle_->GetGeneralizedGravity();

  // ── Task torque τ = Jᵀ Sᵀ f_task  (Λ NOT used ⇒ singularity-free) ────────
  tau_.noalias() = J_S_.transpose() * f_task.head(m_);

  // ── Nullspace posture task (only when redundant: nv > m) ─────────────────
  // M(q) and its Cholesky feed ONLY the dynamically-consistent nullspace
  // projector — the Jacobian-transpose task law never touches M. Compute and
  // fault-check them ONLY when the nullspace task is live; otherwise a benign
  // M-factorization hiccup would spuriously latch SAFE_STOP on a robot that
  // never uses M at all (e.g. UR5e, nv == m ⇒ nullspace always inactive).
  bool dyn_ok = true;
  double sigma_min = std::numeric_limits<double>::infinity();
  double lambda_sq = 0.0;
  const bool nullspace_active =
      (nv > m_) && (gains.nullspace_kp != 0.0 || gains.nullspace_kd != 0.0);
  if (nullspace_active) {
    handle_->ComputeMassMatrix(q_span);
    M_ = handle_->GetMassMatrix();
    M_.triangularView<Eigen::StrictlyLower>() =
        M_.triangularView<Eigen::StrictlyUpper>().transpose();
    llt_M_.compute(M_);
    if (llt_M_.info() != Eigen::Success) {
      dyn_ok = false;
    } else {
      const auto r = dyn_.Compute(J_S_, llt_M_, gains.singularity_threshold, gains.max_damping);
      dyn_ok = r.ok;
      sigma_min = r.sigma_min;
      lambda_sq = r.lambda_sq;
      if (dyn_ok) {
        for (int i = 0; i < nv; ++i)
          tau_posture_dev_(i) =
              gains.nullspace_kp * (q_null_(i) - q_dev_(i)) - gains.nullspace_kd * qdot_dev_(i);
        // Posture is device-order; gather to Pinocchio order before the (Pinocchio)
        // projector Nᵀ. Identity order → memcpy (unchanged).
        handle_->ReorderInput(
            std::span<const double>(tau_posture_dev_.data(), static_cast<std::size_t>(nv)),
            tau_posture_);
        dyn_.ProjectNullspace(tau_posture_, tau_null_);
        tau_.noalias() += alpha * tau_null_;
      }
    }
  }

  // Gravity compensation is NEVER ramped (the arm must not sag on activation).
  if (dyn_ok) {
    tau_ += gravity_;
  } else {
    tau_ = gravity_;  // degenerate M/Λ: gravity hold (finite, safe)
  }

  // ── Scatter to device order, then the §10.5 safety layer ─────────────────
  // All limit vectors are guaranteed nv-sized (InitFromModel + OnDeviceConfigsSet).
  const auto nvz = static_cast<std::size_t>(nv);
  handle_->ReorderOutput(tau_, std::span<double>(tau_dev_.data(), nvz));
  // On the activation/seed tick, start the rate-limit history AT this tick's own
  // command so activation does not slew-limit the initial gravity-comp load.
  if (just_seeded)
    tau_prev_dev_ = tau_dev_;
  Eigen::Map<Eigen::VectorXd> q_lo(position_lower_.data(), nv);
  Eigen::Map<Eigen::VectorXd> q_hi(position_upper_.data(), nv);
  Eigen::Map<Eigen::VectorXd> t_max(max_joint_torque_.data(), nv);
  const auto safety = compliance::ApplySafetyLayer(
      tau_dev_, tau_prev_dev_, q_dev_, qdot_dev_, q_lo, q_hi, t_max, gains.joint_limit_margin,
      gains.joint_limit_kp, gains.joint_limit_kd, gains.max_torque_rate, dt);

  // ── Fault evaluation → state machine (§10.6) ─────────────────────────────
  saturation_elapsed_ = safety.saturated ? (saturation_elapsed_ + dt) : 0.0;
  compliance::ComplianceFaults faults;
  faults.nan_inf = !safety.finite || !dyn_ok;
  // Bound only the REGULATED task DoF: under TRANSLATION_ONLY (m_=3) the rotation
  // rows of `e` are left to the soft nullspace posture task and may drift by
  // design, so folding them into the norm would spuriously latch SAFE_STOP even
  // while translation tracking is perfect. e.head(m_) == the full 6D norm for
  // FULL_SE3 (m_=6), so this is a no-op there.
  faults.pose_error_exceeded = e.head(m_).norm() > gains.pose_error_limit;
  faults.sigma_below_critical = nullspace_active && (sigma_min < gains.singularity_critical);
  faults.saturation_persist = saturation_elapsed_ > gains.saturation_persist_time;
  faults.sigma_below_threshold = nullspace_active && (sigma_min < gains.singularity_threshold);
  const bool ramp_done = (alpha >= 1.0);
  const auto cstate = sm_.Step(faults, ramp_done, dt, kDegradedRecoveryTime);

  diag.state = static_cast<std::uint8_t>(cstate);
  diag.sigma_min = sigma_min;
  diag.lambda_sq = lambda_sq;
  diag.saturated = safety.saturated;
  diag.rate_limited = safety.rate_limited;
  diag.nullspace_active = nullspace_active && dyn_ok;

  // A latched SAFE_STOP (or a non-finite/degenerate tick) hands off to the torque
  // E-STOP hold instead of emitting the raw command.
  if (sm_.in_safe_stop() || !safety.finite || !dyn_ok)
    return ComputeEstop(state, /*control_valid=*/false, diag, gains);

  activation_elapsed_ += dt;  // advance the ramp only on a clean control tick
  diag.control_valid = true;

  // ── Emit ─────────────────────────────────────────────────────────────────
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;
  const int ncmd = std::min(nc0, nv);
  for (int i = 0; i < ncmd; ++i)
    out0.commands[static_cast<std::size_t>(i)] = tau_dev_(i);
  rtc::utils::ClampSymmetric(out0.commands, nc0, std::span<const double>(max_joint_torque_),
                             kDefaultMaxJointTorque);
  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.command_type = command_type_;

  diag_lock_.Store(diag);
  return output;
}

ControllerOutput TaskImpedanceController::ComputeEstop(const ControllerState& state,
                                                       bool control_valid, const Diagnostics& diag,
                                                       const Gains& gains) noexcept {
  // Torque E-STOP hold (E-8): τ = ĝ(q) − D·q̇, clamped to ±τ_max. Replaces the
  // #184 position-slew pattern; the same helper #184 should migrate onto.
  // Gains are read once by the caller (Compute) and threaded in — no second
  // SeqLock load on the hold path.
  const int nv = handle_->nv();
  const auto& dev0 = state.devices[0];
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    qdot_dev_(i) = dev0.velocities[ui];
  }
  handle_->ComputeGeneralizedGravity(
      std::span<const double>(q_buf.data(), static_cast<std::size_t>(nv)));
  gravity_ = handle_->GetGeneralizedGravity();
  handle_->ReorderOutput(gravity_,
                         std::span<double>(grav_dev_.data(), static_cast<std::size_t>(nv)));
  Eigen::Map<Eigen::VectorXd> t_max(max_joint_torque_.data(), nv);  // nv-sized (guaranteed)
  compliance::GravityCompDampedHold(tau_dev_.head(nv), grav_dev_.head(nv), qdot_dev_.head(nv),
                                    gains.estop_damping, t_max);
  // A held tick is a discontinuity — force the next active tick to re-seed and
  // re-ramp from the measured state.
  target_initialized_.store(false, std::memory_order_release);

  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  const int ncmd = std::min(nc0, nv);
  for (int i = 0; i < ncmd; ++i)
    out0.commands[static_cast<std::size_t>(i)] = tau_dev_(i);
  rtc::utils::ClampSymmetric(out0.commands, nc0, std::span<const double>(max_joint_torque_),
                             kDefaultMaxJointTorque);
  output.command_type = command_type_;

  Diagnostics d = diag;
  d.control_valid = control_valid;  // false: the pose-error/σ fields are stale
  diag_lock_.Store(d);
  return output;
}

void TaskImpedanceController::SetDeviceTarget(int device_idx,
                                              std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices)
    return;
  PendingTarget pending{};
  pending.device_idx = device_idx;
  pending.generation = ActivationGeneration();
  const std::size_t nch = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  pending.num_values = static_cast<int>(nch);
  for (std::size_t i = 0; i < nch; ++i)
    pending.values[i] = target[i];
  (void)pending_targets_.Push(pending);
}

std::string_view TaskImpedanceController::Name() const noexcept {
  return "TaskImpedanceController";
}

void TaskImpedanceController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void TaskImpedanceController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  target_initialized_.store(false, std::memory_order_release);
  // NOTE: does NOT clear a controller-local SAFE_STOP — that needs ResetFault().
}

bool TaskImpedanceController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void TaskImpedanceController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Controller registry hooks ────────────────────────────────────────────────

void TaskImpedanceController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  MaybeSelectSubModel();
  if (!cfg) {
    // Even without YAML, enforce the §6.1 selection/nullspace invariant.
    const auto g0 = gains_lock_.Load();
    if (selection_ == TaskSelection::kTranslationOnly && g0.nullspace_kp == 0.0)
      throw std::runtime_error(
          "TaskImpedanceController: TRANSLATION_ONLY requires nullspace_stiffness > 0 (§6.1) — "
          "orientation is otherwise uncontrolled");
    return;
  }
  auto g = gains_lock_.Load();
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr) {
    if (n && n.IsSequence() && n.size() == 3)
      for (std::size_t i = 0; i < 3; ++i)
        arr[i] = n[i].as<double>();
  };
  load3(cfg["kp_pos"], g.kp_pos);
  load3(cfg["kd_pos"], g.kd_pos);
  load3(cfg["kp_rot"], g.kp_rot);
  load3(cfg["kd_rot"], g.kd_rot);
  if (cfg["nullspace_stiffness"])
    g.nullspace_kp = cfg["nullspace_stiffness"].as<double>();
  if (cfg["nullspace_damping"])
    g.nullspace_kd = cfg["nullspace_damping"].as<double>();
  if (cfg["singularity_threshold"])
    g.singularity_threshold = std::max(1e-6, cfg["singularity_threshold"].as<double>());
  if (cfg["singularity_critical"])
    g.singularity_critical = std::max(0.0, cfg["singularity_critical"].as<double>());
  if (cfg["max_damping"])
    g.max_damping = std::max(0.0, cfg["max_damping"].as<double>());
  if (cfg["joint_limit_margin"])
    g.joint_limit_margin = cfg["joint_limit_margin"].as<double>();
  if (cfg["joint_limit_stiffness"])
    g.joint_limit_kp = cfg["joint_limit_stiffness"].as<double>();
  if (cfg["joint_limit_damping"])
    g.joint_limit_kd = cfg["joint_limit_damping"].as<double>();
  if (cfg["max_torque_rate"])
    g.max_torque_rate = cfg["max_torque_rate"].as<double>();
  if (cfg["pose_error_limit"])
    g.pose_error_limit = cfg["pose_error_limit"].as<double>();
  if (cfg["activation_ramp_time"])
    g.activation_ramp_time = cfg["activation_ramp_time"].as<double>();
  if (cfg["estop_damping"])
    g.estop_damping = std::max(0.0, cfg["estop_damping"].as<double>());

  // §6.4 nullspace damping floor K_dⁿ ≥ 2√K_pⁿ (also allows K_pⁿ=0 pure damping).
  if (g.nullspace_kp > 0.0)
    g.nullspace_kd = std::max(g.nullspace_kd, 2.0 * std::sqrt(g.nullspace_kp));

  // §6.1: TRANSLATION_ONLY leaves orientation to the nullspace, so zero nullspace
  // stiffness would let it drift uncontrolled — a configure error, not a warning.
  if (selection_ == TaskSelection::kTranslationOnly && g.nullspace_kp == 0.0)
    throw std::runtime_error(
        "TaskImpedanceController: TRANSLATION_ONLY requires nullspace_stiffness > 0 (§6.1)");

  // Torque-only (MuJoCo backend scope). Reject other command types fail-fast,
  // BEFORE committing gains, so a rejected reconfigure never mutates live gains.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque")
      throw std::runtime_error("TaskImpedanceController: command_type must be 'torque' (got '" + s +
                               "')");
  }
  gains_lock_.Store(g);
  command_type_ = CommandType::kTorque;
}

}  // namespace rtc
