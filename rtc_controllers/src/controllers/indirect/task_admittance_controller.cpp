// ── Includes: project header first, then C++ stdlib ─────────────────────────
#include "rtc_controllers/indirect/task_admittance_controller.hpp"

#include "rtc_base/utils/device_passthrough.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
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

namespace {
// DEGRADED → RUNNING recovery dwell (§10.6 default), same value the impedance
// controller uses. Promoted to a Gains field only if a deployment needs it.
constexpr double kDegradedRecoveryTime = 0.5;  // s
// NUM-1: the DLS damping guard must not be removable from YAML. λ_max = 0 makes
// J J^T singular at a rank-deficient pose, the Cholesky fails and the IK stops
// producing a command at exactly the configuration it exists to survive.
constexpr double kMinMaxDamping = 1e-4;
}  // namespace

// ── Constructor ─────────────────────────────────────────────────────────────

TaskAdmittanceController::TaskAdmittanceController(std::string_view urdf_path, Gains gains)
    : gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  InitFromModel(builder.GetFullModel());
  // Configure here as well as in LoadConfig: BesselFilterN returns 0 for EVERY
  // sample before Init() (all coefficients are zero) and says nothing about it,
  // so a controller constructed directly — as the unit tests do — would
  // otherwise silently zero the whole wrench path, i.e. never move.
  wrench_.Configure(compliance::WrenchConditioningConfig{}, 1.0 / GetDefaultDt());
}

TaskAdmittanceController::TaskAdmittanceController(std::string_view urdf_path)
    : TaskAdmittanceController(urdf_path, Gains{}) {}

void TaskAdmittanceController::InitFromModel(std::shared_ptr<const pinocchio::Model> model) {
  model_ptr_ = std::move(model);
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);

  const int nv = handle_->nv();
  // No kMaxRobotDOF capacity check here, deliberately (the same call the OSC
  // makes). This controller owns no kMaxRobotDOF-wide storage: every work buffer
  // below is a dynamic Eigen type sized to nv, and the only fixed arrays it has
  // — estop_hold_, TargetSlot::targets — are indexed by device CHANNEL and
  // bounded by kMaxDeviceChannels, which is a different and much larger
  // constant. A check here would therefore guard nothing while doing real harm:
  // it runs against the SYSTEM model (the constructor is handed
  // builder.GetFullModel()), and MaybeSelectSubModel() reduces to the primary
  // device only later in LoadConfig — so an nv=14 dual-arm URDF threw at
  // construction, failing the factory and on_configure, for a model this
  // controller never indexes with. TaskImpedanceController has no such check on
  // the same URDF.

  J_full_ = Eigen::MatrixXd::Zero(kTaskDim, nv);
  dq_ = Eigen::VectorXd::Zero(nv);
  dq_dev_ = Eigen::VectorXd::Zero(nv);
  qdot_null_ = Eigen::VectorXd::Zero(nv);
  qdot_null_dev_ = Eigen::VectorXd::Zero(nv);
  q_null_ = Eigen::VectorXd::Zero(nv);
  q_dev_ = Eigen::VectorXd::Zero(nv);
  desired_q_ = Eigen::VectorXd::Zero(nv);
  q_base_ = Eigen::VectorXd::Zero(nv);
  ik_.Resize(nv, kTaskDim);

  // Gravity comes from the model, never a hard-coded 9.81 down −Z: the URDF
  // decides which axis gravity points along, and the payload compensation term
  // would silently point the wrong way in a non-Z-gravity model.
  gravity_world_ = model_ptr_->gravity.linear();
  ResolveSensorFrame();  // the model just moved — any resolved index is stale

  // Inert defaults so the RT path has valid bounds even if OnDeviceConfigsSet is
  // never called (e.g. a direct unit-test construction).
  const auto nvz = static_cast<std::size_t>(nv);
  max_joint_velocity_.assign(nvz, kDefaultMaxJointVelocity);
  position_lower_.assign(nvz, -1e9);
  position_upper_.assign(nvz, 1e9);
}

pinocchio::FrameIndex TaskAdmittanceController::LookupSensorFrame(const std::string& name) const {
  // Empty name ⇒ the wrench is expressed in the TIP body frame. That is NOT an
  // identity transform into LWA: the Ad^{-T} still rotates by R_tip.
  if (name.empty())
    return tip_frame_id_;
  const auto fid = handle_->GetFrameId(name);
  if (fid == 0)  // 0 == universe == "not found" (RtModelHandle contract)
    throw std::runtime_error("TaskAdmittanceController: sensor_frame '" + name +
                             "' is not a frame of the loaded model");
  return fid;
}

void TaskAdmittanceController::ResolveSensorFrame() {
  sensor_frame_id_ = LookupSensorFrame(sensor_frame_name_);
}

void TaskAdmittanceController::SetExternalWrench(
    std::span<const double, compliance::kWrenchDim> wrench) noexcept {
  if (!wrench_enabled_)
    return;
  wrench_.Publish(wrench);
}

void TaskAdmittanceController::MaybeSelectSubModel() {
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
    RCLCPP_ERROR(rclcpp::get_logger("TaskAdmittanceController"),
                 "[TaskAdmittanceController] reduced submodel '%s' unavailable (%s) — keeping full",
                 primary.c_str(), e.what());
  }
}

void TaskAdmittanceController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0)
        tip_frame_id_ = fid;
    }
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
      position_lower_ = cfg->joint_limits->position_lower;
      position_upper_ = cfg->joint_limits->position_upper;
    }
    const auto js = static_cast<int>(cfg->joint_state_names.size());
    const int nv = handle_->nv();
    if (js == nv) {
      if (!handle_->SetJointOrder(cfg->joint_state_names))
        RCLCPP_ERROR(rclcpp::get_logger("TaskAdmittanceController"),
                     "[TaskAdmittanceController] primary device '%s' joint_state_names not all in "
                     "model — reorder disabled",
                     primary.c_str());
    } else if (js > 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("TaskAdmittanceController"),
          "[TaskAdmittanceController] primary device '%s' joint_state_names size=%d != nv=%d",
          primary.c_str(), js, nv);
    }
  }
  // Guarantee every limit vector is exactly nv-sized so the RT maps have valid
  // bounds. Absent position limits → a wide band so the q_cmd clamp stays inert.
  const auto nvz = static_cast<std::size_t>(handle_->nv());
  auto pad = [nvz](std::vector<double>& v, double fill) { v.resize(nvz, fill); };
  pad(max_joint_velocity_, kDefaultMaxJointVelocity);
  pad(position_lower_, -1e9);
  pad(position_upper_, 1e9);
  // tip_frame_id_ may have just moved to the configured tip_link, and the
  // default sensor frame IS the tip — re-resolve so the wrench transform does
  // not keep pointing at the model's last frame.
  ResolveSensorFrame();
}

// ── RTControllerInterface ────────────────────────────────────────────────────

ControllerOutput TaskAdmittanceController::Compute(const ControllerState& state) noexcept {
  Diagnostics diag;
  diag.estopped = estopped_.load(std::memory_order_acquire);

  // A latched controller-local SAFE_STOP is cleared only by ResetFault(), never
  // by ClearEstop (E-8). Consume the request edge before stepping the machine.
  if (reset_fault_requested_.exchange(false, std::memory_order_acq_rel))
    sm_.ResetFault();

  // Retire a hold latched before an activation / E-STOP-clear / fault-reset
  // boundary. Consumed HERE, ahead of every ComputeEstop() call site below, so
  // the first held tick after the boundary re-latches at the measured pose
  // instead of commanding wherever the arm was last time (E-8).
  if (estop_hold_invalidate_.exchange(false, std::memory_order_acq_rel))
    estop_hold_valid_ = false;

  // Surface the latched FSM state so an early return publishes the real state
  // instead of a spurious HOLDING.
  diag.state = static_cast<std::uint8_t>(sm_.state());

  const auto gains = gains_lock_.Load();

  if (diag.estopped) {
    // Drain & discard targets queued during the global E-STOP: ClearEstop() does
    // not bump the activation generation, so they would pass IsCurrentGeneration
    // on the first recovery tick and overwrite the measured-pose re-seed. The RT
    // thread is the sole SPSC consumer, so draining here keeps that invariant.
    PendingTarget discarded{};
    while (pending_targets_.Pop(discarded)) {
      // discard: a command issued during E-STOP must not survive recovery
    }
    return ComputeEstop(state, gains, /*control_valid=*/false, diag);
  }

  const int nv = handle_->nv();
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  // ── Copy joint state (device order) ───────────────────────────────────────
  const auto& dev0 = state.devices[0];
  // Everything below is evaluated at q read from this device. Before the
  // backend's first state arrives (`valid == false`), or on a device narrower
  // than the model, the unread channels read as a default-constructed 0 — FK and
  // the Jacobian are then evaluated at the ZERO configuration and out0.commands
  // comes out as a full-arm move to the origin, with no fault raised because
  // every number involved is perfectly finite. There is no honest substitute for
  // an unknown joint position, so the tick emits NO command (the CM's own
  // BuildHoldOutput uses the same zero-length-command idiom for exactly this)
  // and reports DEGRADED.
  if (!dev0.valid || dev0.num_channels < nv)
    return ComputeNoJointState(state, gains, diag);
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    q_dev_(i) = dev0.positions[ui];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));

  // ── FK + Jacobian ─────────────────────────────────────────────────────────
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);

  // ── Target slot: seed X_d / q_null from measured on (re)activation (§10.7) ─
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;
  if (!target_initialized_.load(std::memory_order_acquire)) {
    goal_pose_ = tcp;
    std::memcpy(slot.goal_rot.data(), tcp.rotation().data(), sizeof(slot.goal_rot));
    std::memcpy(slot.goal_t.data(), tcp.translation().data(), sizeof(slot.goal_t));
    for (int i = 0; i < nv; ++i) {
      q_null_(i) = q_dev_(i);
      desired_q_(i) = q_dev_(i);
    }
    // The compliant frame collapses onto X_d: activation must not inherit a
    // deviation (or a velocity) accrued while the controller was not running,
    // which would step the arm on the first tick.
    integrator_.Reset();
    activation_elapsed_ = 0.0;
    saturation_elapsed_ = 0.0;
    if (wrench_enabled_ && wrench_.ResetForActivation()) {
      // §3.2.1 "on_activate 시 자동 1회". The state machine refuses this while a
      // SAFE_STOP is latched, so re-seeding cannot launder a fault (E-8).
      sm_.BeginBiasCalibration();
      bias_gate_ = false;
    }
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

  // Gain ramp α (§10.7): applied to the WRENCH, which is the only discontinuous
  // input at activation — X_c starts at X_d = the measured pose, so the position
  // command itself is already continuous and must not be ramped.
  const double ramp = gains.activation_ramp_time;
  const double alpha = (ramp <= 0.0) ? 1.0 : std::min(1.0, activation_elapsed_ / ramp);

  // ── External wrench f_ext in LWA at the tip (§3.2.1, §10.6) ───────────────
  Eigen::Matrix<double, 6, 1> f_ext = Eigen::Matrix<double, 6, 1>::Zero();
  if (wrench_enabled_) {
    const pinocchio::SE3& sensor = handle_->GetFramePlacement(sensor_frame_id_);
    compliance::WrenchPipelineStatus ws;
    const Eigen::Matrix<double, 6, 1> f_lwa =
        wrench_.Update(sensor.rotation(), sensor.translation(), tcp.translation(), gravity_world_,
                       dt, gains.wrench, ws);
    if (ws.begin_bias_calibration)
      sm_.BeginBiasCalibration();
    bias_gate_ = ws.bias_gate_released;
    for (int i = 0; i < 6; ++i)
      diag.wrench_lwa[static_cast<std::size_t>(i)] = f_lwa(i);  // pre-ramp: the physical estimate
    diag.wrench_valid = ws.valid;
    diag.wrench_age = ws.age;
    diag.wrench_fade = ws.fade;
    diag.wrench_rejected = ws.rejected_samples;
    diag.wrench_stale = ws.stale;
    diag.bias_calibrated = ws.bias_calibrated;
    diag.in_contact = ws.in_contact;
    f_ext = alpha * f_lwa;
  } else {
    bias_gate_ = true;
  }

  // ── §7.2 compliant frame ──────────────────────────────────────────────────
  const compliance::AdmittanceStatus ast = integrator_.Step(gains.admittance, f_ext, dt);
  compliant_pose_.rotation() = integrator_.rotation() * goal_pose_.rotation();
  compliant_pose_.translation() = goal_pose_.translation() + integrator_.translation();

  const Eigen::Matrix<double, 6, 1> x_dev = integrator_.deviation();
  const Eigen::Matrix<double, 6, 1>& nu_c = integrator_.velocity();
  for (int i = 0; i < 6; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    diag.compliant_deviation[ui] = x_dev(i);
    diag.compliant_velocity[ui] = nu_c(i);
  }
  diag.displacement_limited = ast.displacement_limited;
  diag.velocity_limited = ast.velocity_limited;

  // ── §7.3 differential IK onto X_c ─────────────────────────────────────────
  // e runs CURRENT → COMPLIANT (SplitWorld = LWA), so the correction is +K·e —
  // the same §1.3 sign convention the impedance law uses. Note this is the
  // OPPOSITE direction to the §7.2 deviation x̃_c, which runs desired → compliant.
  const Eigen::Matrix<double, 6, 1> e =
      rtc::math::se3::computePoseError(tcp, compliant_pose_, rtc::math::se3::ErrorType::SplitWorld);
  for (int i = 0; i < 6; ++i)
    diag.pose_error[static_cast<std::size_t>(i)] = e(i);

  Eigen::Matrix<double, 6, 1> nu = nu_c;
  for (int i = 0; i < 3; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    nu(i) += gains.ik_kp_pos[ui] * e(i);
    nu(i + 3) += gains.ik_kp_rot[ui] * e(i + 3);
  }

  const bool nullspace_active = (nv > kTaskDim) && (gains.nullspace_kp != 0.0);
  if (nullspace_active) {
    for (int i = 0; i < nv; ++i)
      qdot_null_dev_(i) = gains.nullspace_kp * (q_null_(i) - q_dev_(i));
    // Posture is device-order; gather to Pinocchio order before the (Pinocchio)
    // projector N. Identity order → memcpy (unchanged).
    handle_->ReorderInput(
        std::span<const double>(qdot_null_dev_.data(), static_cast<std::size_t>(nv)), qdot_null_);
  } else {
    qdot_null_.setZero();
  }

  // Floor λ_max at the point of use so the singularity guard holds regardless of
  // how the gains were set (LoadConfig floors too, but set_gains() bypasses it) — NUM-1.
  const double max_damping = std::max(kMinMaxDamping, gains.max_damping);
  const compliance::DifferentialIk::Result ik =
      ik_.Compute(J_full_, gains.singularity_threshold, max_damping);
  if (ik.ok) {
    ik_.Solve(nu, qdot_null_, dq_);
  } else {
    dq_.setZero();  // J⁺/N are stale — hold rather than command garbage
  }
  diag.sigma_min = ik.sigma_min;
  diag.lambda_sq = ik.lambda_sq;
  diag.nullspace_active = nullspace_active && ik.ok;

  // ── Scatter to device order, clamp velocity, integrate to a position ───────
  const auto nvz = static_cast<std::size_t>(nv);
  handle_->ReorderOutput(dq_, std::span<double>(dq_dev_.data(), nvz));
  bool vel_clamped = false;
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lim =
        (ui < max_joint_velocity_.size()) ? max_joint_velocity_[ui] : kDefaultMaxJointVelocity;
    if (dq_dev_(i) > lim) {
      dq_dev_(i) = lim;
      vel_clamped = true;
    } else if (dq_dev_(i) < -lim) {
      dq_dev_(i) = -lim;
      vel_clamped = true;
    }
  }

  // §7.3 MUST — both integration bases implemented and selectable.
  for (int i = 0; i < nv; ++i) {
    q_base_(i) = gains.integrate_from_measured ? q_dev_(i) : desired_q_(i);
    desired_q_(i) = q_base_(i) + dq_dev_(i) * dt;
  }
  // Joint-limit clamp on the POSITION command. The torque-domain repulsive
  // potential of compliance/safety_limiter.hpp does not transfer: a position
  // command outside the mechanical range is not a soft push, it is a request the
  // backend will either reject or drive into a hard stop.
  //
  // ...and then the RATE is re-imposed, because the clamp above can WIDEN the
  // step it was handed. This controller replaced compliance::ApplySafetyLayer's
  // bounded repulsion + max_torque_rate with a hard clamp and kept no rate bound
  // to go with it: with the arm sitting inside the joint_limit_margin band, the
  // clamp moves q_cmd to the band edge in a single tick regardless of how small
  // the IK step was — a `joint_limit_margin` of 0.08 rad is an 0.08 rad jump. The
  // one monitor that could have caught it, faults.command_divergence, is gated on
  // `!integrate_from_measured` and that flag defaults to true.
  //
  // Order matters: clamp first, then re-bound the rate. Bounding first and
  // clamping second is what leaves the clamp's own correction unbounded, which
  // is the defect. The consequence is that q_cmd may stay transiently inside the
  // margin band while it slews out of it — the band is a soft margin, and
  // arriving at its edge at a bounded rate is the behaviour it is asking for.
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lo = position_lower_[ui] + gains.joint_limit_margin;
    const double hi = position_upper_[ui] - gains.joint_limit_margin;
    if (lo < hi)
      desired_q_(i) = std::clamp(desired_q_(i), lo, hi);

    const double lim =
        (ui < max_joint_velocity_.size()) ? max_joint_velocity_[ui] : kDefaultMaxJointVelocity;
    const double step = std::abs(lim) * dt;
    const double bounded = std::clamp(desired_q_(i), q_base_(i) - step, q_base_(i) + step);
    if (bounded != desired_q_(i)) {
      desired_q_(i) = bounded;
      vel_clamped = true;
    }
    // Keep the reported joint velocity the one actually commanded: emitting a
    // target_velocity the emitted position contradicts is how a rate breach
    // stays invisible to whoever is watching the lane for exactly that.
    dq_dev_(i) = (desired_q_(i) - q_base_(i)) / dt;
  }
  diag.joint_velocity_limited = vel_clamped;

  double divergence = 0.0;
  for (int i = 0; i < nv; ++i) {
    const double d = desired_q_(i) - q_dev_(i);
    divergence += d * d;
  }
  divergence = std::sqrt(divergence);
  diag.command_divergence = divergence;

  // ── Fault evaluation → state machine (§10.6) ──────────────────────────────
  const bool finite = desired_q_.allFinite() && dq_dev_.allFinite() && e.allFinite() && ast.finite;
  saturation_elapsed_ = vel_clamped ? (saturation_elapsed_ + dt) : 0.0;
  compliance::ComplianceFaults faults;
  faults.nan_inf = !finite || !ik.ok;
  faults.pose_error_exceeded = e.norm() > gains.pose_error_limit;
  faults.sigma_below_critical = ik.sigma_min < gains.singularity_critical;
  faults.sigma_below_threshold = ik.sigma_min < gains.singularity_threshold;
  faults.saturation_persist = saturation_elapsed_ > gains.saturation_persist_time;
  // Wrench loss DEGRADES, never latches (§10.6: the middle state between normal
  // and fatal). The fade has already ramped f_ext to zero, so the compliant
  // frame returns to X_d under K_p — or holds where it is when K_p = 0, which is
  // the intended hand-guiding behaviour, not a failure to stop.
  faults.wrench_timeout = diag.wrench_stale;
  // §7.3 MUST — only the self-integrating mode can wind away from the arm; in
  // the measured-base mode q_cmd is re-anchored every tick by construction.
  faults.command_divergence =
      !gains.integrate_from_measured && (divergence > gains.command_divergence_limit);

  const bool ramp_done = (alpha >= 1.0);
  const auto cstate =
      sm_.Step(faults, ramp_done, dt, kDegradedRecoveryTime, bias_gate_, diag.in_contact);
  diag.state = static_cast<std::uint8_t>(cstate);

  if (sm_.in_safe_stop() || !finite || !ik.ok)
    return ComputeEstop(state, gains, /*control_valid=*/false, diag);

  activation_elapsed_ += dt;  // advance the ramp only on a clean control tick
  diag.control_valid = true;
  // Arm the E-STOP latch for the NEXT held tick. Deliberately not cleared in the
  // re-seed block above: a latched SAFE_STOP forces a re-seed every tick, so
  // clearing it there would re-latch the hold at the freshly measured position
  // on every one of them — a "hold" that follows a backdriven arm, which is
  // exactly the failure the latch exists to prevent. The boundaries that DO
  // retire the latch go through InvalidateEstopHold() and are consumed at the
  // top of this function.
  estop_hold_valid_ = false;

  // ── Emit ──────────────────────────────────────────────────────────────────
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;
  const auto nq = std::min(static_cast<std::size_t>(nc0), nvz);
  for (std::size_t i = 0; i < nq; ++i) {
    const auto ei = static_cast<Eigen::Index>(i);
    out0.commands[i] = desired_q_(ei);
    out0.target_velocities[i] = dq_dev_(ei);
    out0.trajectory_positions[i] = desired_q_(ei);
  }
  // Channels past the model DOF (nc0 > nv) hold their current position.
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = dev0.positions[i];
    out0.trajectory_positions[i] = dev0.positions[i];
  }
  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  // Both lanes are 6-wide (x,y,z,r,p,y) and every consumer reads all six —
  // device_state_log_pod / pod_fill emit them straight to CSV. Filling only the
  // translation on a FULL_SE3 controller means every orientation experiment logs
  // "no rotation" rather than "not measured", which is the one reading an
  // operator cannot tell apart from a real result. ZYX Euler at the boundary, as
  // p_controller and the OSC already do.
  const Eigen::Vector3d rpy_actual = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy_actual.x();
  output.actual_task_positions[4] = rpy_actual.y();
  output.actual_task_positions[5] = rpy_actual.z();
  // The compliant frame is the thing this controller actually commands, so it is
  // what the task-goal lane must show — X_d alone would look motionless under a
  // sustained push, which is the one situation an operator is watching for.
  const Eigen::Vector3d rpy_goal = pinocchio::rpy::matrixToRpy(compliant_pose_.rotation());
  output.task_goal_positions[0] = compliant_pose_.translation().x();
  output.task_goal_positions[1] = compliant_pose_.translation().y();
  output.task_goal_positions[2] = compliant_pose_.translation().z();
  output.task_goal_positions[3] = rpy_goal.x();
  output.task_goal_positions[4] = rpy_goal.y();
  output.task_goal_positions[5] = rpy_goal.z();
  output.command_type = command_type_;

  diag_lock_.Store(diag);
  return output;
}

ControllerOutput TaskAdmittanceController::ComputeEstop(const ControllerState& state,
                                                        const Gains& gains, bool control_valid,
                                                        const Diagnostics& diag) noexcept {
  const auto& dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const auto nch =
      std::min(static_cast<std::size_t>(nc0), static_cast<std::size_t>(kMaxDeviceChannels));
  if (!estop_hold_valid_) {
    for (std::size_t i = 0; i < nch; ++i) {
      estop_hold_[i] = dev0.positions[i];
      // Seed the rate base at the same pose: the FIRST held tick after a
      // (re)latch must be able to emit the latch itself, not slew up to it.
      estop_last_command_[i] = dev0.positions[i];
    }
    estop_hold_valid_ = true;
  }

  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  out0.num_channels = nc0;
  for (std::size_t i = 0; i < nch; ++i) {
    double cmd = estop_hold_[i];
    // The same joint-limit clamp the live path applies (§ q_cmd clamp): an arm
    // that was already outside its mechanical range when the stop fired must not
    // have that pose LATCHED and re-commanded forever.
    if (i < position_lower_.size() && i < position_upper_.size()) {
      const double lo = position_lower_[i] + gains.joint_limit_margin;
      const double hi = position_upper_[i] - gains.joint_limit_margin;
      if (lo < hi)
        cmd = std::clamp(cmd, lo, hi);
    }
    // ...and the correction that clamp asks for is a MOTION, so it is rate-bound
    // like every other motion this controller emits. Against the last emitted
    // command, not q_meas — see estop_last_command_. A steady hold is unaffected
    // (the command does not change, so the bound never binds).
    const double lim =
        (i < max_joint_velocity_.size()) ? max_joint_velocity_[i] : kDefaultMaxJointVelocity;
    const double step = std::abs(lim) * dt;
    cmd = std::clamp(cmd, estop_last_command_[i] - step, estop_last_command_[i] + step);
    estop_last_command_[i] = cmd;
    out0.commands[i] = cmd;
    out0.trajectory_positions[i] = cmd;
  }
  output.command_type = command_type_;

  // A held tick is a discontinuity — force the next active tick to re-seed X_d,
  // the compliant frame and the command integrator from the measured state.
  target_initialized_.store(false, std::memory_order_release);

  Diagnostics d = diag;
  d.control_valid = control_valid;  // false: the pose-error / σ fields are stale
  diag_lock_.Store(d);
  return output;
}

ControllerOutput TaskAdmittanceController::ComputeNoJointState(const ControllerState& state,
                                                               const Gains& gains,
                                                               Diagnostics& diag) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  compliance::ComplianceFaults faults;
  faults.device_state_invalid = true;
  // The same ramp predicate the clean path derives from alpha. Not fabricated as
  // `true`: a controller that has never had a readable joint state has not
  // finished activating, and §10.6's lattice honours a degrade cause at the
  // HOLDING→RUNNING edge. A device that fails mid-run is already RUNNING, so it
  // degrades on this tick regardless.
  const double ramp = gains.activation_ramp_time;
  const bool ramp_done = (ramp <= 0.0) || (activation_elapsed_ >= ramp);
  diag.state = static_cast<std::uint8_t>(
      sm_.Step(faults, ramp_done, dt, kDegradedRecoveryTime, bias_gate_, false));

  ControllerOutput output;
  output.num_devices = state.num_devices;
  // Zero-length = "no update" to every backend. Secondary devices keep their
  // own passthrough: a hand does not stop being commandable because the arm's
  // state went missing.
  output.devices[0].num_channels = 0;
  rtc::utils::PassthroughSecondaryDevices(state, output, target_seqlock_.Load().targets);
  output.command_type = command_type_;

  // Same discontinuity contract a held tick has: whatever X_d / q_null / command
  // integrator state exists was seeded from a joint state this tick could not
  // read, so the next controllable tick re-seeds from measurement. The hold latch
  // goes with it — it would otherwise describe a pose read while the device was
  // untrustworthy.
  target_initialized_.store(false, std::memory_order_release);
  estop_hold_valid_ = false;

  diag.control_valid = false;
  diag_lock_.Store(diag);
  return output;
}

void TaskAdmittanceController::SetDeviceTarget(int device_idx,
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
  // Off-RT marshal — the RT thread drains pending_targets_ inside Compute() and
  // is the SOLE writer of target_seqlock_.
  (void)pending_targets_.Push(pending);
}

std::string_view TaskAdmittanceController::Name() const noexcept {
  return "TaskAdmittanceController";
}

void TaskAdmittanceController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void TaskAdmittanceController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  target_initialized_.store(false, std::memory_order_release);
  // The arm may have been jogged while the stop was asserted, so the hold latch
  // is retired here too — otherwise a re-trigger before the next clean tick
  // (E-STOP cleared, arm moved, E-STOP hit again) would command the pre-clear
  // pose in one step. Note this clears the HOLD, not the fault.
  InvalidateEstopHold();
  // NOTE: does NOT clear a controller-local SAFE_STOP — that needs ResetFault().
}

bool TaskAdmittanceController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void TaskAdmittanceController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Controller registry hooks ────────────────────────────────────────────────

void TaskAdmittanceController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  MaybeSelectSubModel();
  if (!cfg)
    return;

  auto g = gains_lock_.Load();

  // Shape-checked exactly like load6 below. Silently ignoring a mis-shaped node
  // is worse here than anywhere else in this function: the header documents
  // `ik_kp_*: 0` as the way to reproduce §7.3's pure-feedforward law literally,
  // so a scalar `ik_kp_pos: 0.0` — the obvious way to write that — was dropped
  // and the default 2.0 kept, which means the one experiment the knob exists for
  // silently ran as a CLIK variant, with nothing in the diagnostics to say so.
  // No scalar broadcast (D5): a convenience shorthand would re-introduce the
  // same "quietly a different value" failure in a new shape.
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr, const char* what) {
    if (!n)
      return;
    if (!n.IsSequence() || n.size() != 3)
      throw std::runtime_error(std::string("TaskAdmittanceController: ") + what +
                               " must be a 3-entry sequence [x,y,z]");
    for (std::size_t i = 0; i < 3; ++i)
      arr[i] = n[i].as<double>();
  };
  auto load6 = [](const YAML::Node& n, std::array<double, 6>& arr, const char* what,
                  bool require_positive) {
    if (!n)
      return;
    if (!n.IsSequence() || n.size() != 6)
      throw std::runtime_error(std::string("TaskAdmittanceController: ") + what +
                               " must be a 6-entry sequence [x,y,z,rx,ry,rz]");
    for (std::size_t i = 0; i < 6; ++i) {
      const double v = n[i].as<double>();
      // NUM-2: Λ_d is inverted every tick. A zero or negative entry is not a
      // soft-clamped tuning mistake, it is a divide.
      if (require_positive && !(v > 0.0))
        throw std::runtime_error(std::string("TaskAdmittanceController: ") + what +
                                 " entries must be > 0");
      if (!require_positive && v < 0.0)
        throw std::runtime_error(std::string("TaskAdmittanceController: ") + what +
                                 " entries must be >= 0");
      arr[i] = v;
    }
  };

  // ── §7.2 virtual dynamics ────────────────────────────────────────────────
  load6(cfg["desired_inertia"], g.admittance.inertia, "desired_inertia", true);
  load6(cfg["damping"], g.admittance.damping, "damping", false);
  load6(cfg["stiffness"], g.admittance.stiffness, "stiffness", false);

  // ── §7.4 contact-stability floor: [translation kg, rotation kg·m²] ───────
  if (const YAML::Node& n = cfg["min_desired_inertia"]; n) {
    if (!n.IsSequence() || n.size() != 2)
      throw std::runtime_error(
          "TaskAdmittanceController: min_desired_inertia must be [translation, rotation]");
    g.admittance.min_inertia_lin = std::max(0.0, n[0].as<double>());
    g.admittance.min_inertia_ang = std::max(0.0, n[1].as<double>());
  }

  // ── §7.5 workspace / velocity bounds ─────────────────────────────────────
  if (const YAML::Node& n = cfg["max_compliant_displacement"]; n) {
    if (!n.IsSequence() || n.size() != 2)
      throw std::runtime_error(
          "TaskAdmittanceController: max_compliant_displacement must be [metres, radians]");
    g.admittance.max_displacement_lin = n[0].as<double>();
    g.admittance.max_displacement_ang = n[1].as<double>();
  }
  if (cfg["max_compliant_linear_velocity"])
    g.admittance.max_velocity_lin = cfg["max_compliant_linear_velocity"].as<double>();
  if (cfg["max_compliant_angular_velocity"])
    g.admittance.max_velocity_ang = cfg["max_compliant_angular_velocity"].as<double>();
  // Separate from the two above on purpose — see AdmittanceParams. Not floored
  // here: the floor lives at the point of use so set_gains() cannot bypass it
  // (NUM-1), and clamping here as well would only hide a bad YAML value from
  // whoever reads the gains back.
  if (cfg["max_return_linear_velocity"])
    g.admittance.max_return_velocity_lin = cfg["max_return_linear_velocity"].as<double>();
  if (cfg["max_return_angular_velocity"])
    g.admittance.max_return_velocity_ang = cfg["max_return_angular_velocity"].as<double>();
  if (const YAML::Node& n = cfg["barrier_stiffness"]; n) {
    if (!n.IsSequence() || n.size() != 2)
      throw std::runtime_error(
          "TaskAdmittanceController: barrier_stiffness must be [linear, angular]");
    g.admittance.barrier_stiffness_lin = std::max(0.0, n[0].as<double>());
    g.admittance.barrier_stiffness_ang = std::max(0.0, n[1].as<double>());
  }

  // ── §7.3 IK ──────────────────────────────────────────────────────────────
  load3(cfg["ik_kp_pos"], g.ik_kp_pos, "ik_kp_pos");
  load3(cfg["ik_kp_rot"], g.ik_kp_rot, "ik_kp_rot");
  if (cfg["nullspace_kp"])
    g.nullspace_kp = cfg["nullspace_kp"].as<double>();
  if (cfg["integrate_from_measured"])
    g.integrate_from_measured = cfg["integrate_from_measured"].as<bool>();
  if (cfg["singularity_threshold"])
    g.singularity_threshold = std::max(1e-6, cfg["singularity_threshold"].as<double>());
  if (cfg["singularity_critical"])
    g.singularity_critical = std::max(0.0, cfg["singularity_critical"].as<double>());
  if (cfg["max_damping"])
    g.max_damping = std::max(kMinMaxDamping, cfg["max_damping"].as<double>());

  // ── Safety / activation ──────────────────────────────────────────────────
  // Strictly positive, and rejected rather than clamped (D6). `e.norm() > limit`
  // is evaluated every tick against a CRITICAL fault, so a 0 or negative bound
  // makes that comparison true forever: SAFE_STOP latches on the first tick, and
  // `ComplianceFaults::pose_error_exceeded` carries no cause field to point at
  // the config. The `<= 0 disables` idiom the §7.5 bounds use is deliberately
  // NOT offered — this is the guard, and a way to switch it off would make a
  // mis-configuration look like a legitimate setting.
  if (const YAML::Node& n = cfg["pose_error_limit"]; n) {
    const double v = n.as<double>();
    if (!(v > 0.0))
      throw std::runtime_error("TaskAdmittanceController: pose_error_limit must be > 0");
    g.pose_error_limit = v;
  }
  if (cfg["command_divergence_limit"])
    g.command_divergence_limit = std::max(0.0, cfg["command_divergence_limit"].as<double>());
  if (cfg["joint_limit_margin"])
    g.joint_limit_margin = std::max(0.0, cfg["joint_limit_margin"].as<double>());
  if (cfg["activation_ramp_time"])
    g.activation_ramp_time = cfg["activation_ramp_time"].as<double>();
  if (cfg["saturation_persist_time"])
    g.saturation_persist_time = std::max(0.0, cfg["saturation_persist_time"].as<double>());

  // ── External wrench source (§3.2.1) — REQUIRED here (§7.1, axis A ≠ NONE) ─
  bool wrench_enabled = true;
  std::string sensor_frame;
  compliance::WrenchConditioningConfig wc;
  if (const YAML::Node& ew = cfg["external_wrench"]; ew && ew.IsMap()) {
    if (ew["enabled"])
      wrench_enabled = ew["enabled"].as<bool>();
    if (ew["sensor_frame"])
      sensor_frame = ew["sensor_frame"].as<std::string>();
    auto load_w6 = [](const YAML::Node& n, compliance::Wrench6& arr, const char* what) {
      if (!n)
        return;
      if (!n.IsSequence() || n.size() != 6)
        throw std::runtime_error(std::string("TaskAdmittanceController: ") + what +
                                 " must be a 6-entry sequence [fx,fy,fz,tx,ty,tz]");
      for (std::size_t i = 0; i < 6; ++i)
        arr[i] = n[i].as<double>();
    };
    load_w6(ew["deadband"], wc.deadband, "external_wrench.deadband");
    load_w6(ew["max"], wc.max_abs, "external_wrench.max");
    if (ew["payload_mass"])
      wc.payload_mass = ew["payload_mass"].as<double>();
    if (const YAML::Node& com = ew["payload_com"]; com && com.IsSequence()) {
      if (com.size() != 3)
        throw std::runtime_error(
            "TaskAdmittanceController: external_wrench.payload_com must have 3 entries");
      for (std::size_t i = 0; i < 3; ++i)
        wc.payload_com(static_cast<Eigen::Index>(i)) = com[i].as<double>();
    }
    if (ew["filter_enabled"])
      wc.filter_enabled = ew["filter_enabled"].as<bool>();
    if (ew["filter_cutoff_force"])
      wc.filter_cutoff_force_hz = ew["filter_cutoff_force"].as<double>();
    if (ew["filter_cutoff_torque"])
      wc.filter_cutoff_torque_hz = ew["filter_cutoff_torque"].as<double>();
    if (ew["bias_calibration_samples"])
      wc.bias_samples = ew["bias_calibration_samples"].as<int>();
    if (ew["timeout"])
      g.wrench.timeout = std::max(0.0, ew["timeout"].as<double>());
    if (ew["fadeout_time"])
      g.wrench.fadeout_time = std::max(0.0, ew["fadeout_time"].as<double>());
    if (ew["contact_threshold"])
      g.wrench.contact_threshold = std::max(0.0, ew["contact_threshold"].as<double>());
    if (ew["contact_release_ratio"])
      // Clamped strictly below 1 so the ⇄ transition always keeps a hysteresis
      // band (§10.6 MUST): equal thresholds chatter at the boundary.
      g.wrench.contact_release_ratio =
          std::clamp(ew["contact_release_ratio"].as<double>(), 0.0, 0.99);
  }

  // §7.1: admittance takes force as its INPUT. Without a wrench source the
  // compliant frame never leaves X_d and the controller degenerates into an
  // expensive position hold — a configure error, not a fallback (unlike the
  // impedance controller, where A=NONE is a first-class law).
  if (!wrench_enabled)
    throw std::runtime_error(
        "TaskAdmittanceController: external_wrench.enabled must be true (§7.1 — admittance "
        "requires axis A != NONE)");

  // Position-only output (D14 — no CommandType::kVelocity). Rejected fail-fast
  // BEFORE committing gains so a bad reconfigure never mutates live state.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "position")
      throw std::runtime_error("TaskAdmittanceController: command_type must be 'position' (got '" +
                               s + "')");
  }

  // Last two things that can fail. Resolve the frame into a LOCAL first: a bad
  // sensor_frame must not leave the controller half-reconfigured. (The model may
  // have been swapped for a reduced one by MaybeSelectSubModel above, so the
  // lookup has to run against the model actually in use.)
  const pinocchio::FrameIndex sensor_id = LookupSensorFrame(sensor_frame);
  wrench_.Configure(wc, 1.0 / GetDefaultDt());  // validates, then commits

  // Commit. Nothing below throws.
  sensor_frame_name_ = sensor_frame;
  sensor_frame_id_ = sensor_id;
  wrench_enabled_ = wrench_enabled;
  gains_lock_.Store(g);
  command_type_ = CommandType::kPosition;
}

}  // namespace rtc
