// ── Includes: project header first, then C++ stdlib ─────────────────────────
#include "rtc_controllers/direct/cascaded_compliance_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
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
// DEGRADED → RUNNING recovery dwell (§10.6 default), the same value the other
// two compliance controllers use.
constexpr double kDegradedRecoveryTime = 0.5;  // s
// NUM-1: the DLS damping guard must not be removable. λ_max = 0 leaves Λ_S⁻¹
// undamped at a rank-deficient pose, so the Cholesky fails and the nullspace
// projector dies at exactly the configuration it exists to survive. Floored at
// the point of USE, not only in LoadConfig — set_gains() bypasses configure.
constexpr double kMinMaxDamping = 1e-4;
}  // namespace

// ── Constructor ─────────────────────────────────────────────────────────────

CascadedComplianceController::CascadedComplianceController(std::string_view urdf_path, Gains gains)
    : gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  InitFromModel(builder.GetFullModel());
  // Configure here as well as in LoadConfig: BesselFilterN returns 0 for EVERY
  // sample before Init() (all coefficients are zero) and says nothing about it,
  // so a controller constructed directly — as the unit tests do — would silently
  // zero the whole wrench path, i.e. an outer loop that never moves.
  wrench_.Configure(compliance::WrenchConditioningConfig{}, 1.0 / GetDefaultDt());
}

CascadedComplianceController::CascadedComplianceController(std::string_view urdf_path)
    : CascadedComplianceController(urdf_path, Gains{}) {}

void CascadedComplianceController::InitFromModel(std::shared_ptr<const pinocchio::Model> model) {
  // Build and VALIDATE against the incoming model before any member moves. The
  // sensor-frame lookup is the one step here that can throw, and the only caller
  // that catches (MaybeSelectSubModel) logs "keeping full" — a claim the previous
  // order could not honour: model_ptr_/handle_ were replaced first, so a throw
  // left the reduced model live with the PREVIOUS model's limit-vector lengths
  // and a stale sensor_frame_id_, i.e. the RT tick transforming the wrench by the
  // wrong link's rotation and lever arm, with no fault raised.
  //
  // Structural, not a live bug fix: GetReducedModel currently carries the locked
  // subtree's FRAMES over, so a sensor_frame that resolved against the full model
  // still resolves against the reduced one and this lookup does not actually
  // throw today (verified — a `sensor_frame: finger_left` reduction succeeds).
  // The ordering is what keeps that an implementation detail of the builder
  // rather than a silent correctness dependency of this controller.
  auto handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model);
  const auto tip = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  const pinocchio::FrameIndex sensor = LookupSensorFrame(*handle, tip, sensor_frame_name_);

  // ── Commit ────────────────────────────────────────────────────────────────
  model_ptr_ = std::move(model);
  handle_ = std::move(handle);
  tip_frame_id_ = tip;
  sensor_frame_id_ = sensor;  // the model just moved — any resolved index is stale

  // No kMaxRobotDOF capacity check, deliberately — the same call the OSC and both
  // sibling compliance controllers make. Every work buffer here is a dynamic
  // Eigen type sized to nv; the only fixed arrays (TargetSlot::targets) are
  // indexed by device CHANNEL and bounded by kMaxDeviceChannels. A check would
  // also run against the SYSTEM model (the constructor gets GetFullModel()) and
  // reject an nv=14 dual-arm URDF before MaybeSelectSubModel() reduces it.
  const int nv = handle_->nv();
  J_full_ = Eigen::MatrixXd::Zero(kTaskDim, nv);
  M_ = Eigen::MatrixXd::Zero(nv, nv);
  gravity_ = Eigen::VectorXd::Zero(nv);
  tau_ = Eigen::VectorXd::Zero(nv);
  tau_posture_dev_ = Eigen::VectorXd::Zero(nv);
  tau_posture_ = Eigen::VectorXd::Zero(nv);
  tau_null_ = Eigen::VectorXd::Zero(nv);
  tcp_vel_ = Eigen::VectorXd::Zero(kTaskDim);
  qdot_ = Eigen::VectorXd::Zero(nv);
  q_null_ = Eigen::VectorXd::Zero(nv);
  tau_dev_ = Eigen::VectorXd::Zero(nv);
  tau_prev_dev_ = Eigen::VectorXd::Zero(nv);
  q_dev_ = Eigen::VectorXd::Zero(nv);
  qdot_dev_ = Eigen::VectorXd::Zero(nv);
  grav_dev_ = Eigen::VectorXd::Zero(nv);
  llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv);
  dyn_.Resize(nv, kTaskDim);

  // Gravity comes from the model, never a hard-coded 9.81 down −Z: the URDF
  // decides which axis gravity points along, and the payload compensation term
  // would silently point the wrong way in a non-Z-gravity model.
  gravity_world_ = model_ptr_->gravity.linear();

  // Inert defaults so the RT path has valid bounds even if OnDeviceConfigsSet is
  // never called (e.g. a direct unit-test construction).
  const auto nvz = static_cast<std::size_t>(nv);
  max_joint_torque_.assign(nvz, kDefaultMaxJointTorque);
  max_joint_velocity_.assign(nvz, kDefaultMaxJointVelocity);
  position_lower_.assign(nvz, -1e9);
  position_upper_.assign(nvz, 1e9);
}

pinocchio::FrameIndex CascadedComplianceController::LookupSensorFrame(
    const rtc_urdf_bridge::RtModelHandle& handle, pinocchio::FrameIndex tip,
    const std::string& name) {
  // Empty name ⇒ the wrench is expressed in the TIP body frame. That is NOT an
  // identity transform into LWA: the Ad^{-T} still rotates by R_tip.
  if (name.empty())
    return tip;
  const auto fid = handle.GetFrameId(name);
  if (fid == 0)  // 0 == universe == "not found" (RtModelHandle contract)
    throw std::runtime_error("CascadedComplianceController: sensor_frame '" + name +
                             "' is not a frame of the loaded model");
  return fid;
}

pinocchio::FrameIndex CascadedComplianceController::LookupSensorFrame(
    const std::string& name) const {
  return LookupSensorFrame(*handle_, tip_frame_id_, name);
}

void CascadedComplianceController::ResolveSensorFrame() {
  sensor_frame_id_ = LookupSensorFrame(sensor_frame_name_);
}

void CascadedComplianceController::SetExternalWrench(
    std::span<const double, compliance::kWrenchDim> wrench) noexcept {
  if (!wrench_enabled_)
    return;
  wrench_.Publish(wrench);
}

void CascadedComplianceController::MaybeSelectSubModel() {
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
    RCLCPP_ERROR(
        rclcpp::get_logger("CascadedComplianceController"),
        "[CascadedComplianceController] reduced submodel '%s' unavailable (%s) — keeping full",
        primary.c_str(), e.what());
  }
}

void CascadedComplianceController::OnDeviceConfigsSet() {
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
        RCLCPP_ERROR(rclcpp::get_logger("CascadedComplianceController"),
                     "[CascadedComplianceController] primary device '%s' joint_state_names not all "
                     "in model — reorder disabled",
                     primary.c_str());
    } else if (js > 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("CascadedComplianceController"),
          "[CascadedComplianceController] primary device '%s' joint_state_names size=%d != nv=%d",
          primary.c_str(), js, nv);
    }
  }
  // Guarantee every limit vector is exactly nv-sized so the RT safety maps have
  // valid bounds. Absent position limits → a wide band so §5.3 stays inert.
  const auto nvz = static_cast<std::size_t>(handle_->nv());
  auto pad = [nvz](std::vector<double>& v, double fill) { v.resize(nvz, fill); };
  pad(max_joint_velocity_, kDefaultMaxJointVelocity);
  pad(max_joint_torque_, kDefaultMaxJointTorque);
  pad(position_lower_, -1e9);
  pad(position_upper_, 1e9);
  // tip_frame_id_ may have just moved to the configured tip_link, and the
  // default sensor frame IS the tip — re-resolve so the wrench transform does
  // not keep pointing at the model's last frame.
  ResolveSensorFrame();
}

// ── §7.6 MUST-1 bandwidth separation ─────────────────────────────────────────

void CascadedComplianceController::EvaluateBandwidthSeparation(const Gains& gains) noexcept {
  // ω_i = √(K_p^i / Λ_S)  (inner, per task axis, at the seeding pose)
  // ω_a = √(K_p^a / Λ_d)  (outer, per task axis)
  // The comparison is per-axis and the reported figure is the WORST axis: a
  // cascade separated on five axes and coupled on the sixth is coupled.
  const Eigen::MatrixXd& lambda_s = dyn_.LambdaS();
  double worst = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kTaskDim; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double kp_a = gains.admittance.stiffness[ui];
    // K_p^a = 0 is hand-guiding: the outer loop has no restoring frequency to be
    // separated FROM, so the axis carries no ratio rather than an infinite one
    // that would silently dominate the min in the other direction.
    if (!(kp_a > 0.0))
      continue;
    // The same floor Step()/Energy() apply, so the ratio cannot disagree with the
    // Λ_d the integrator actually runs (NUM-2: Λ_d is inverted every tick).
    const double lambda_d =
        std::max(gains.admittance.inertia[ui],
                 (i < 3) ? gains.admittance.min_inertia_lin : gains.admittance.min_inertia_ang);
    const double lambda_i = lambda_s(i, i);
    const double kp_i = (i < 3) ? gains.impedance.kp_pos[ui] : gains.impedance.kp_rot[ui - 3];
    // Λ_d ≤ 0 or a non-positive Λ_S diagonal means the RATIO cannot be formed —
    // a degenerate seeding pose or a gain the floor above should have caught —
    // so the axis carries no verdict.
    if (!(lambda_d > 0.0) || !(lambda_i > 0.0))
      continue;
    // K_p^i = 0 is NOT "not evaluable": ω_i = 0 against a positive ω_a is the
    // WORST possible separation, an inner loop with no restoring bandwidth under
    // an outer loop that has one. Skipping it removed the single most coupled
    // axis from a min() whose whole job is to report the most coupled axis, and
    // `inner.kp_pos: [0,0,0]` passes LoadConfig (>= 0), so the flag read "fine"
    // for exactly the configuration it exists to catch.
    if (!(kp_i > 0.0)) {
      worst = 0.0;
      continue;
    }
    const double omega_a = std::sqrt(kp_a / lambda_d);
    const double omega_i = std::sqrt(kp_i / lambda_i);
    if (!(omega_a > 0.0))
      continue;
    worst = std::min(worst, omega_i / omega_a);
  }
  bandwidth_ratio_ = worst;
  // Diagnostic, never a fault (D20): a sluggish inner loop is a tuning statement
  // about two gain sets, and latching SAFE_STOP for it would stop a robot that
  // is merely soft. RT ticks cannot log (RT-3), hence a flag.
  bandwidth_ratio_low_ = std::isfinite(worst) && (worst < gains.min_bandwidth_ratio);
}

// ── RTControllerInterface ────────────────────────────────────────────────────

ControllerOutput CascadedComplianceController::Compute(const ControllerState& state) noexcept {
  Diagnostics diag;
  diag.estopped = estopped_.load(std::memory_order_acquire);

  // A latched controller-local SAFE_STOP is cleared only by ResetFault(), never
  // by ClearEstop (E-8). Consume the request edge before stepping the machine.
  if (reset_fault_requested_.exchange(false, std::memory_order_acq_rel))
    sm_.ResetFault();

  // Surface the latched FSM state so an early return publishes the real state
  // instead of a spurious HOLDING.
  diag.state = static_cast<std::uint8_t>(sm_.state());
  diag.bandwidth_ratio = bandwidth_ratio_;
  diag.bandwidth_ratio_low = bandwidth_ratio_low_;

  // One SeqLock read per RT tick, shared by every path.
  const auto gains = gains_lock_.Load();

  if (diag.estopped) {
    DrainPendingTargets();
    return ComputeEstop(state, /*control_valid=*/false, diag, gains);
  }

  const int nv = handle_->nv();
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  // ── Copy joint state (device order) ───────────────────────────────────────
  const auto& dev0 = state.devices[0];
  // Everything below is evaluated at q read from this device (F5): before the
  // backend's first state arrives, or on a device narrower than the model, the
  // unread channels read as a default-constructed 0, FK/Jacobian run at the ZERO
  // configuration, and the emitted torque is a full-arm pull toward the origin —
  // with no fault raised, because every number involved is finite.
  if (!dev0.valid || dev0.num_channels < nv)
    return ComputeNoJointState(state, gains, diag);

  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    q_dev_(i) = dev0.positions[ui];
    qdot_dev_(i) = dev0.velocities[ui];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));

  // ── FK + Jacobian + current task twist ν ──────────────────────────────────
  // ComputeJacobians gathers q into Pinocchio order internally, so J_full_ has
  // PINOCCHIO column order. q̇ therefore has to be gathered the same way before
  // the product: the device-order vector would pair each column with another
  // joint's velocity, which is a permuted ν — a wrong TCP velocity and a wrong
  // −K_d^i·ν damping torque, with nothing non-finite to fault on. Identity order
  // → memcpy, so this costs nothing on the common path.
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  handle_->ReorderInput(std::span<const double>(qdot_dev_.data(), static_cast<std::size_t>(nv)),
                        qdot_);
  tcp_vel_.noalias() = J_full_ * qdot_;
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);

  // ── Target slot: seed X_d / q_null from measured on (re)activation (§10.7) ─
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;
  bool just_seeded = false;
  if (!target_initialized_.load(std::memory_order_acquire)) {
    goal_pose_ = tcp;
    std::memcpy(slot.goal_rot.data(), tcp.rotation().data(), sizeof(slot.goal_rot));
    std::memcpy(slot.goal_t.data(), tcp.translation().data(), sizeof(slot.goal_t));
    for (int i = 0; i < nv; ++i)
      q_null_(i) = q_dev_(i);  // posture target = measured (device order)
    // The compliant frame collapses onto X_d at rest: an activation must not
    // inherit a deviation (or a velocity) accrued while the controller was not
    // running, which would command a torque toward a frame the arm never left.
    integrator_.Reset();
    activation_elapsed_ = 0.0;  // restart the inner (α_track) ramp
    wrench_elapsed_ = 0.0;      // ...and the outer one, which re-arms below
    saturation_elapsed_ = 0.0;
    tau_prev_dev_.setZero();
    // The wrench path restarts with everything else — including DISOWNING the
    // sample in the slot, so a producer that died before deactivation cannot come
    // back as fresh data on the first tick of this activation (§10.6). The state
    // machine refuses BIAS_CALIBRATING while a SAFE_STOP is latched, so re-seeding
    // cannot launder a fault (E-8).
    // No `bias_gate_ = false` here: this branch requires wrench_enabled_, so the
    // OUTER block below unconditionally assigns bias_gate_ = ws.bias_gate_released
    // on this very tick with no return in between. Writing it here reads like the
    // latch that HOLDS BIAS_CALIBRATING, is not one, and no test can tell.
    if (wrench_enabled_ && wrench_.ResetForActivation())
      sm_.BeginBiasCalibration();
    just_seeded = true;
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

  // Drain off-RT targets (device 0 = SE3 pose X_d; others = joint passthrough).
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

  // Gain ramps (§10.7): 0→1 over activation_ramp_time; ≤0 disables. TWO clocks
  // over one duration — α_track from the seeding tick for the inner loop,
  // α_wrench from the first tick the pipeline emits a force for the outer one.
  // See the class comment: a shared clock is consumed by BIAS_CALIBRATING and
  // hands the outer loop a step, and freezing the shared clock instead would
  // leave the arm on gravity comp alone for the same interval.
  const double ramp = gains.activation_ramp_time;
  const double alpha = (ramp <= 0.0) ? 1.0 : std::min(1.0, activation_elapsed_ / ramp);
  const double alpha_wrench = (ramp <= 0.0) ? 1.0 : std::min(1.0, wrench_elapsed_ / ramp);

  // ── OUTER: external wrench → compliant frame (X_c, ν_c) ───────────────────
  Eigen::Matrix<double, 6, 1> f_ext = Eigen::Matrix<double, 6, 1>::Zero();
  // "The pipeline is emitting" — a sample has arrived AND the §3.2.1 average is
  // committed. Until then Update() returns zero by construction, so advancing
  // α_wrench would spend the ramp on a force that does not exist yet.
  bool wrench_live = true;
  if (wrench_enabled_) {
    const pinocchio::SE3& sensor = handle_->GetFramePlacement(sensor_frame_id_);
    compliance::WrenchPipelineStatus ws;
    const Eigen::Matrix<double, 6, 1> f_lwa =
        wrench_.Update(sensor.rotation(), sensor.translation(), tcp.translation(), gravity_world_,
                       dt, gains.wrench, ws);
    // Routed out of the pipeline rather than done inside it so the E-8 SAFE_STOP
    // latch (which refuses this transition) stays the state machine's business.
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
    wrench_live = ws.valid && ws.bias_calibrated;
    f_ext = alpha_wrench * f_lwa;
  } else {
    bias_gate_ = true;
  }

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

  // ── INNER: §6.2 impedance tracking (X_c, ν_c) ─────────────────────────────
  // e runs CURRENT → COMPLIANT (SplitWorld = LWA), so the force is +K_p·e. Note
  // this is the OPPOSITE direction to the outer deviation x̃_c, which runs
  // desired → compliant (compliance-conventions.md §3.3 — the two live one line
  // apart here and slice 2 already shipped one sign defect at this seam).
  //
  // ν_d = ν_c, NOT zero: the inner loop must track the compliant frame's motion,
  // not damp it. The measured wrench does NOT appear — the outer loop already
  // consumed it, and consuming it again here is §7.6 MUST-4's double count
  // (structurally impossible: there is no inertia-shaping term to put it in).
  const Eigen::Matrix<double, 6, 1> e =
      rtc::math::se3::computePoseError(tcp, compliant_pose_, rtc::math::se3::ErrorType::SplitWorld);
  for (int i = 0; i < 6; ++i)
    diag.pose_error[static_cast<std::size_t>(i)] = e(i);

  const Eigen::Matrix<double, 6, 1> f_task =
      compliance::ComputeImpedanceForce(gains.impedance, e, tcp_vel_, nu_c, alpha, kTaskDim);

  // ── Joint-space gravity ĝ(q) (always needed: comp + E-STOP hold) ──────────
  handle_->ComputeGeneralizedGravity(q_span);
  gravity_ = handle_->GetGeneralizedGravity();

  // ── Task dynamics Λ_S / Nᵀ ────────────────────────────────────────────────
  // Needed by the nullspace projector, and ONCE more on the seeding tick for the
  // §7.6 MUST-1 ratio (D20 — Λ_S(q₀) does not exist at configure time). The
  // inner law itself is Jacobian-transpose and never forms Λ, so it stays free of
  // task-space singularities (§6.5) and the σ faults below follow the nullspace,
  // not the seed-tick diagnostic: a rank-deficient seeding pose must cost the
  // bandwidth REPORT, not the activation.
  bool dyn_ok = true;
  double sigma_min = std::numeric_limits<double>::infinity();
  double lambda_sq = 0.0;
  const bool nullspace_active =
      (nv > kTaskDim) && (gains.nullspace_kp != 0.0 || gains.nullspace_kd != 0.0);
  // Re-evaluate the MUST-1 ratio whenever the gains it compares have CHANGED, not
  // only on activation: set_gains() / a re-LoadConfig write a new POD into the
  // SeqLock, and the published figure would otherwise keep describing the gain
  // set that was just retired — a "3.4, separated" reading for a cascade that is
  // no longer tuned that way.
  const bool bw_pending = bandwidth_eval_pending_.exchange(false, std::memory_order_acq_rel);
  const bool bw_evaluate = just_seeded || bw_pending;
  if (bw_evaluate) {
    // Clear BEFORE attempting: if the Cholesky below fails at this pose the
    // controller must publish "not evaluable" (∞, flag clear) rather than the
    // previous activation's number under the new gains.
    bandwidth_ratio_ = std::numeric_limits<double>::infinity();
    bandwidth_ratio_low_ = false;
  }
  if (nullspace_active || bw_evaluate) {
    handle_->ComputeMassMatrix(q_span);
    M_ = handle_->GetMassMatrix();
    M_.triangularView<Eigen::StrictlyLower>() =
        M_.triangularView<Eigen::StrictlyUpper>().transpose();
    llt_M_.compute(M_);
    if (llt_M_.info() != Eigen::Success) {
      if (nullspace_active)
        dyn_ok = false;
      if (bw_evaluate)
        bandwidth_eval_pending_.store(true, std::memory_order_release);  // retry next tick
    } else {
      // NUM-1 at the point of use: LoadConfig floors max_damping too, but
      // set_gains() writes the POD straight into the SeqLock and bypasses it.
      const compliance::TaskDynamics::Result r =
          dyn_.Compute(J_full_, llt_M_, gains.singularity_threshold,
                       std::max(kMinMaxDamping, gains.max_damping));
      if (bw_evaluate && r.ok)
        EvaluateBandwidthSeparation(gains);
      else if (bw_evaluate)
        bandwidth_eval_pending_.store(true, std::memory_order_release);  // retry next tick
      if (nullspace_active) {
        dyn_ok = r.ok;
        sigma_min = r.sigma_min;
        lambda_sq = r.lambda_sq;
        if (dyn_ok) {
          for (int i = 0; i < nv; ++i)
            tau_posture_dev_(i) =
                gains.nullspace_kp * (q_null_(i) - q_dev_(i)) - gains.nullspace_kd * qdot_dev_(i);
          // Posture is device-order; gather to Pinocchio order before the
          // (Pinocchio) projector Nᵀ. Identity order → memcpy (unchanged).
          handle_->ReorderInput(
              std::span<const double>(tau_posture_dev_.data(), static_cast<std::size_t>(nv)),
              tau_posture_);
          dyn_.ProjectNullspace(tau_posture_, tau_null_);
        }
      }
    }
  }
  diag.bandwidth_ratio = bandwidth_ratio_;
  diag.bandwidth_ratio_low = bandwidth_ratio_low_;

  // ── Task torque τ = Jᵀ f_task + α Nᵀ τ_posture + ĝ ────────────────────────
  tau_.noalias() = J_full_.transpose() * f_task;
  if (dyn_ok && nullspace_active)
    tau_.noalias() += alpha * tau_null_;
  // Gravity compensation is NEVER ramped (the arm must not sag on activation).
  if (dyn_ok) {
    tau_ += gravity_;
  } else {
    tau_ = gravity_;  // degenerate M/Λ: gravity hold (finite, safe)
  }

  // ── Scatter to device order, then the §10.5 safety layer ──────────────────
  const auto nvz = static_cast<std::size_t>(nv);
  handle_->ReorderOutput(tau_, std::span<double>(tau_dev_.data(), nvz));
  // On the activation/seed tick, start the rate-limit history AT this tick's own
  // command so activation is not slew-limited against a stale history.
  if (just_seeded)
    tau_prev_dev_ = tau_dev_;
  Eigen::Map<Eigen::VectorXd> q_lo(position_lower_.data(), nv);
  Eigen::Map<Eigen::VectorXd> q_hi(position_upper_.data(), nv);
  Eigen::Map<Eigen::VectorXd> t_max(max_joint_torque_.data(), nv);
  const auto safety = compliance::ApplySafetyLayer(
      tau_dev_, tau_prev_dev_, q_dev_, qdot_dev_, q_lo, q_hi, t_max, gains.joint_limit_margin,
      gains.joint_limit_kp, gains.joint_limit_kd, gains.max_torque_rate, dt);

  // ── Fault evaluation → state machine (§10.6) ──────────────────────────────
  saturation_elapsed_ = safety.saturated ? (saturation_elapsed_ + dt) : 0.0;
  compliance::ComplianceFaults faults;
  faults.nan_inf = !safety.finite || !dyn_ok || !ast.finite || !e.allFinite();
  faults.pose_error_exceeded = e.norm() > gains.pose_error_limit;
  // The σ faults follow the nullspace gate for the reason above: Λ_S is the only
  // consumer, and the Jacobian-transpose inner law has no singularity exposure.
  faults.sigma_below_critical = nullspace_active && (sigma_min < gains.singularity_critical);
  faults.sigma_below_threshold = nullspace_active && (sigma_min < gains.singularity_threshold);
  faults.saturation_persist = saturation_elapsed_ > gains.saturation_persist_time;
  // Wrench loss DEGRADES, never latches (§10.6: the middle state between normal
  // and fatal). The fade has already ramped f_ext to zero, so the compliant frame
  // returns to X_d under K_p^a — or stays where it is when K_p^a = 0, which is
  // the intended hand-guiding behaviour and not a failure to stop.
  faults.wrench_timeout = diag.wrench_stale;
  // HOLDING→RUNNING follows α_track alone, NOT α_wrench: with no producer yet,
  // α_wrench never advances (there is nothing to ramp in), and gating the FSM on
  // it would park a controller whose tracking loop is fully engaged in HOLDING
  // for as long as the F/T driver is late. BIAS_CALIBRATING already covers the
  // "wrench not usable yet" state, via bias_gate_.
  const bool ramp_done = (alpha >= 1.0);
  const auto cstate =
      sm_.Step(faults, ramp_done, dt, kDegradedRecoveryTime, bias_gate_, diag.in_contact);

  diag.state = static_cast<std::uint8_t>(cstate);
  diag.sigma_min = sigma_min;
  diag.lambda_sq = lambda_sq;
  diag.saturated = safety.saturated;
  diag.rate_limited = safety.rate_limited;
  diag.nullspace_active = nullspace_active && dyn_ok;

  // A latched SAFE_STOP (or a non-finite / degenerate tick) hands off to the
  // torque E-STOP hold instead of emitting the raw command.
  if (sm_.in_safe_stop() || !safety.finite || !dyn_ok)
    return ComputeEstop(state, /*control_valid=*/false, diag, gains);

  // Advance the ramps only on a clean control tick. α_wrench additionally waits
  // for the pipeline to have something to ramp IN — otherwise the whole ramp is
  // spent while f_ext ≡ 0 and the first real sample arrives at full gain.
  activation_elapsed_ += dt;
  if (wrench_live)
    wrench_elapsed_ += dt;
  diag.control_valid = true;

  // ── Emit ──────────────────────────────────────────────────────────────────
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;
  const int ncmd = std::min(nc0, nv);
  for (int i = 0; i < ncmd; ++i)
    out0.commands[static_cast<std::size_t>(i)] = tau_dev_(i);
  // Channels past the model DOF (nc0 > nv) get zero torque, not a stale value.
  rtc::utils::ClampSymmetric(out0.commands, nc0, std::span<const double>(max_joint_torque_),
                             kDefaultMaxJointTorque);
  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  // Both task lanes are 6-wide (x,y,z,r,p,y) and every consumer reads all six
  // (device_state_log_pod / pod_fill emit them straight to CSV). The GOAL lane is
  // the COMPLIANT frame, not X_d: X_c is what this controller actually commands,
  // and X_d alone looks motionless under a sustained push — the one situation an
  // operator is watching for. ZYX Euler at the boundary.
  const Eigen::Vector3d rpy_actual = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy_actual.x();
  output.actual_task_positions[4] = rpy_actual.y();
  output.actual_task_positions[5] = rpy_actual.z();
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

ControllerOutput CascadedComplianceController::ComputeEstop(const ControllerState& state,
                                                            bool control_valid,
                                                            const Diagnostics& diag,
                                                            const Gains& gains) noexcept {
  const auto& dev0 = state.devices[0];
  const int nv = handle_->nv();
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  output.command_type = command_type_;

  // Without a readable joint state there is no ĝ(q) to hold with. Zero torque is
  // the honest command: a torque backend with no drive signal is a free joint,
  // which is what an unknown configuration already is.
  if (!dev0.valid || dev0.num_channels < nv) {
    target_initialized_.store(false, std::memory_order_release);
    Diagnostics d = diag;
    d.control_valid = false;
    diag_lock_.Store(d);
    return output;
  }

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

  const int ncmd = std::min(nc0, nv);
  for (int i = 0; i < ncmd; ++i)
    out0.commands[static_cast<std::size_t>(i)] = tau_dev_(i);
  rtc::utils::ClampSymmetric(out0.commands, nc0, std::span<const double>(max_joint_torque_),
                             kDefaultMaxJointTorque);

  // A held tick is a discontinuity — force the next active tick to re-seed X_d,
  // the compliant frame, the ramp and the rate-limit history from measurement.
  // That re-seed is also what retires any compliant-frame deviation accrued
  // before the hold, so there is no separate latch to invalidate here (the
  // position-domain hold of TaskAdmittanceController needs one; ĝ(q) − D·q̇ does
  // not remember a pose).
  target_initialized_.store(false, std::memory_order_release);

  Diagnostics d = diag;
  d.control_valid = control_valid;  // false: the pose-error / σ fields are stale
  diag_lock_.Store(d);
  return output;
}

void CascadedComplianceController::DrainPendingTargets() noexcept {
  PendingTarget discarded{};
  while (pending_targets_.Pop(discarded)) {
    // discard: a command issued while held must not survive recovery
  }
}

ControllerOutput CascadedComplianceController::ComputeNoJointState(const ControllerState& state,
                                                                   const Gains& gains,
                                                                   Diagnostics& diag) noexcept {
  // Same contract as the E-STOP path: this tick forces a re-seed below, so any
  // target queued while the device was unreadable must not outlive the outage.
  // The producer keeps publishing through a backend dropout, so without this the
  // depth-4 queue holds the OLDEST commands of the outage (SpscQueue drops the
  // newest when full) and the first recovered tick jumps to one of them instead
  // of the measured pose — braked only by max_torque_rate.
  DrainPendingTargets();
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
  // Zero-length = "no update" to every backend. Secondary devices keep their own
  // passthrough: a hand does not stop being commandable because the arm's state
  // went missing.
  output.devices[0].num_channels = 0;
  rtc::utils::PassthroughSecondaryDevices(state, output, target_seqlock_.Load().targets);
  output.command_type = command_type_;

  // Whatever X_d / X_c / posture state exists was seeded from a joint state this
  // tick could not read, so the next controllable tick re-seeds from measurement.
  target_initialized_.store(false, std::memory_order_release);

  diag.control_valid = false;
  diag_lock_.Store(diag);
  return output;
}

void CascadedComplianceController::SetDeviceTarget(int device_idx,
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

std::string_view CascadedComplianceController::Name() const noexcept {
  return "CascadedComplianceController";
}

void CascadedComplianceController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void CascadedComplianceController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  target_initialized_.store(false, std::memory_order_release);
  // NOTE: does NOT clear a controller-local SAFE_STOP — that needs ResetFault().
}

bool CascadedComplianceController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void CascadedComplianceController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Controller registry hooks ────────────────────────────────────────────────

void CascadedComplianceController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  MaybeSelectSubModel();
  if (!cfg)
    return;

  auto g = gains_lock_.Load();

  // Shape-checked, never silently ignored (F7, PR #256): a mis-shaped node that
  // is dropped leaves the DEFAULT gain running under a config file that says
  // otherwise, and nothing downstream can tell the two apart. No scalar
  // broadcast (D5) — a shorthand would re-introduce "quietly a different value"
  // in a new shape.
  // Every scalar read goes through this. NaN and ±inf are NOT tuning mistakes
  // that a clamp can absorb: `std::max(0.0, NaN)` returns 0.0 — silently a
  // different value, the exact failure mode the shape check above exists to
  // prevent — and an infinite gain reaches the admittance state on the first
  // tick, so the controller latches SAFE_STOP with `nan_inf` and nothing points
  // at the config line that caused it.
  auto num = [](const YAML::Node& n, const char* what) {
    const double v = n.as<double>();
    if (!std::isfinite(v))
      throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                               " must be a finite number");
    return v;
  };
  // A section that is ABSENT is legal (the defaults apply, which is the
  // documented behaviour); a section that is PRESENT but not a map is a config
  // error. Silently skipping the latter leaves the defaults running under a file
  // that says otherwise — the F7 failure class, one level up from the leaves.
  auto require_map = [](const YAML::Node& n, const char* what) {
    if (n && !n.IsMap())
      throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                               " must be a map of keys");
  };
  auto load3 = [&num](const YAML::Node& n, std::array<double, 3>& arr, const char* what,
                      bool require_non_negative) {
    if (!n)
      return;
    if (!n.IsSequence() || n.size() != 3)
      throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                               " must be a 3-entry sequence [x,y,z]");
    for (std::size_t i = 0; i < 3; ++i) {
      const double v = num(n[i], what);
      if (require_non_negative && !(v >= 0.0))
        throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                                 " entries must be >= 0");
      arr[i] = v;
    }
  };
  auto load6 = [&num](const YAML::Node& n, std::array<double, 6>& arr, const char* what,
                      bool require_positive) {
    if (!n)
      return;
    if (!n.IsSequence() || n.size() != 6)
      throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                               " must be a 6-entry sequence [x,y,z,rx,ry,rz]");
    for (std::size_t i = 0; i < 6; ++i) {
      // NaN is rejected by `num` above, not here: `v < 0.0` is FALSE for NaN, so
      // the non-negative branch would pass it straight into the admittance state.
      const double v = num(n[i], what);
      // NUM-2: Λ_d is inverted every tick. A zero or negative entry is not a
      // soft-clamped tuning mistake, it is a divide.
      if (require_positive && !(v > 0.0))
        throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                                 " entries must be > 0");
      if (!require_positive && v < 0.0)
        throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                                 " entries must be >= 0");
      arr[i] = v;
    }
  };

  // ── OUTER loop (§7.2 / §7.4 / §7.5) ──────────────────────────────────────
  // Nested under `outer:` / `inner:` rather than flattened: both loops have a
  // stiffness and a damping, and a flat `stiffness:` key would be the single
  // most consequential ambiguity in this file — the outer one defines how the
  // robot yields to force, the inner one only how tightly it tracks.
  require_map(cfg["outer"], "outer");
  if (const YAML::Node& o = cfg["outer"]; o) {
    load6(o["desired_inertia"], g.admittance.inertia, "outer.desired_inertia", true);
    load6(o["damping"], g.admittance.damping, "outer.damping", false);
    load6(o["stiffness"], g.admittance.stiffness, "outer.stiffness", false);
    if (const YAML::Node& n = o["min_desired_inertia"]; n) {
      if (!n.IsSequence() || n.size() != 2)
        throw std::runtime_error(
            "CascadedComplianceController: outer.min_desired_inertia must be [translation, "
            "rotation]");
      g.admittance.min_inertia_lin = std::max(0.0, num(n[0], "outer.min_desired_inertia"));
      g.admittance.min_inertia_ang = std::max(0.0, num(n[1], "outer.min_desired_inertia"));
    }
    if (const YAML::Node& n = o["max_compliant_displacement"]; n) {
      if (!n.IsSequence() || n.size() != 2)
        throw std::runtime_error(
            "CascadedComplianceController: outer.max_compliant_displacement must be [metres, "
            "radians]");
      g.admittance.max_displacement_lin = num(n[0], "outer.max_compliant_displacement");
      g.admittance.max_displacement_ang = num(n[1], "outer.max_compliant_displacement");
    }
    if (o["max_compliant_linear_velocity"])
      g.admittance.max_velocity_lin =
          num(o["max_compliant_linear_velocity"], "outer.max_compliant_linear_velocity");
    if (o["max_compliant_angular_velocity"])
      g.admittance.max_velocity_ang =
          num(o["max_compliant_angular_velocity"], "outer.max_compliant_angular_velocity");
    // Separate keys from the two above on purpose (PR #256 F2) — see
    // AdmittanceParams. Not floored here: the floor lives at the point of use so
    // set_gains() cannot bypass it (NUM-1).
    if (o["max_return_linear_velocity"])
      g.admittance.max_return_velocity_lin =
          num(o["max_return_linear_velocity"], "outer.max_return_linear_velocity");
    if (o["max_return_angular_velocity"])
      g.admittance.max_return_velocity_ang =
          num(o["max_return_angular_velocity"], "outer.max_return_angular_velocity");
    if (const YAML::Node& n = o["barrier_stiffness"]; n) {
      if (!n.IsSequence() || n.size() != 2)
        throw std::runtime_error(
            "CascadedComplianceController: outer.barrier_stiffness must be [linear, angular]");
      g.admittance.barrier_stiffness_lin = std::max(0.0, num(n[0], "outer.barrier_stiffness"));
      g.admittance.barrier_stiffness_ang = std::max(0.0, num(n[1], "outer.barrier_stiffness"));
    }
  }

  // ── INNER loop (§6.2) ────────────────────────────────────────────────────
  require_map(cfg["inner"], "inner");
  if (const YAML::Node& in = cfg["inner"]; in) {
    load3(in["kp_pos"], g.impedance.kp_pos, "inner.kp_pos", true);
    load3(in["kd_pos"], g.impedance.kd_pos, "inner.kd_pos", true);
    load3(in["kp_rot"], g.impedance.kp_rot, "inner.kp_rot", true);
    load3(in["kd_rot"], g.impedance.kd_rot, "inner.kd_rot", true);
  }

  // ── Nullspace / DLS / safety / activation ────────────────────────────────
  if (cfg["nullspace_stiffness"])
    g.nullspace_kp = std::max(0.0, num(cfg["nullspace_stiffness"], "nullspace_stiffness"));
  if (cfg["nullspace_damping"])
    g.nullspace_kd = std::max(0.0, num(cfg["nullspace_damping"], "nullspace_damping"));
  if (cfg["singularity_threshold"])
    g.singularity_threshold =
        std::max(1e-6, num(cfg["singularity_threshold"], "singularity_threshold"));
  if (cfg["singularity_critical"])
    g.singularity_critical =
        std::max(0.0, num(cfg["singularity_critical"], "singularity_critical"));
  if (cfg["max_damping"])
    g.max_damping = std::max(kMinMaxDamping, num(cfg["max_damping"], "max_damping"));
  if (cfg["joint_limit_margin"])
    g.joint_limit_margin = std::max(0.0, num(cfg["joint_limit_margin"], "joint_limit_margin"));
  if (cfg["joint_limit_stiffness"])
    g.joint_limit_kp = std::max(0.0, num(cfg["joint_limit_stiffness"], "joint_limit_stiffness"));
  if (cfg["joint_limit_damping"])
    g.joint_limit_kd = std::max(0.0, num(cfg["joint_limit_damping"], "joint_limit_damping"));
  // F8 — every threshold that a first tick compares against must be incapable of
  // latching SAFE_STOP by being 0 or negative.
  //
  // max_torque_rate is a rate BOUND: 0 would freeze the command at the
  // rate-limit history forever (the arm stops responding while every fault stays
  // clear), so it is rejected rather than clamped.
  if (const YAML::Node& n = cfg["max_torque_rate"]; n) {
    const double v = num(n, "max_torque_rate");
    if (!(v > 0.0))
      throw std::runtime_error("CascadedComplianceController: max_torque_rate must be > 0");
    g.max_torque_rate = v;
  }
  // pose_error_limit is compared every tick against a CRITICAL fault, so a 0 or
  // negative bound makes `e.norm() > limit` true forever: SAFE_STOP latches on
  // the first tick and `pose_error_exceeded` carries no cause field to point at
  // the config. The `<= 0 disables` idiom is deliberately NOT offered — this is
  // the guard (D6).
  if (const YAML::Node& n = cfg["pose_error_limit"]; n) {
    const double v = num(n, "pose_error_limit");
    if (!(v > 0.0))
      throw std::runtime_error("CascadedComplianceController: pose_error_limit must be > 0");
    g.pose_error_limit = v;
  }
  if (cfg["activation_ramp_time"])
    g.activation_ramp_time = num(cfg["activation_ramp_time"], "activation_ramp_time");
  if (cfg["estop_damping"])
    g.estop_damping = std::max(0.0, num(cfg["estop_damping"], "estop_damping"));
  // saturation_persist_time == 0 would degrade on the FIRST saturated tick.
  // That is a legitimate (if twitchy) setting — DEGRADED is recoverable and
  // carries no latch — so it is clamped at 0 rather than rejected.
  if (cfg["saturation_persist_time"])
    g.saturation_persist_time =
        std::max(0.0, num(cfg["saturation_persist_time"], "saturation_persist_time"));
  // 0 silences the §7.6 MUST-1 flag; negative is meaningless, not a disable.
  if (cfg["min_bandwidth_ratio"])
    g.min_bandwidth_ratio = std::max(0.0, num(cfg["min_bandwidth_ratio"], "min_bandwidth_ratio"));

  // ── External wrench source (§3.2.1) — REQUIRED (the outer loop's input) ───
  bool wrench_enabled = true;
  std::string sensor_frame;
  compliance::WrenchConditioningConfig wc;
  require_map(cfg["external_wrench"], "external_wrench");
  if (const YAML::Node& ew = cfg["external_wrench"]; ew) {
    if (ew["enabled"])
      wrench_enabled = ew["enabled"].as<bool>();
    if (ew["sensor_frame"])
      sensor_frame = ew["sensor_frame"].as<std::string>();
    auto load_w6 = [&num](const YAML::Node& n, compliance::Wrench6& arr, const char* what) {
      if (!n)
        return;
      if (!n.IsSequence() || n.size() != 6)
        throw std::runtime_error(std::string("CascadedComplianceController: ") + what +
                                 " must be a 6-entry sequence [fx,fy,fz,tx,ty,tz]");
      for (std::size_t i = 0; i < 6; ++i)
        arr[i] = num(n[i], what);
    };
    load_w6(ew["deadband"], wc.deadband, "external_wrench.deadband");
    load_w6(ew["max"], wc.max_abs, "external_wrench.max");
    if (ew["payload_mass"])
      wc.payload_mass = num(ew["payload_mass"], "external_wrench.payload_mass");
    if (const YAML::Node& com = ew["payload_com"]; com && com.IsSequence()) {
      if (com.size() != 3)
        throw std::runtime_error(
            "CascadedComplianceController: external_wrench.payload_com must have 3 entries");
      for (std::size_t i = 0; i < 3; ++i)
        wc.payload_com(static_cast<Eigen::Index>(i)) = num(com[i], "external_wrench.payload_com");
    }
    if (ew["filter_enabled"])
      wc.filter_enabled = ew["filter_enabled"].as<bool>();
    if (ew["filter_cutoff_force"])
      wc.filter_cutoff_force_hz =
          num(ew["filter_cutoff_force"], "external_wrench.filter_cutoff_force");
    if (ew["filter_cutoff_torque"])
      wc.filter_cutoff_torque_hz =
          num(ew["filter_cutoff_torque"], "external_wrench.filter_cutoff_torque");
    if (ew["bias_calibration_samples"])
      wc.bias_samples = ew["bias_calibration_samples"].as<int>();
    if (ew["timeout"])
      g.wrench.timeout = std::max(0.0, num(ew["timeout"], "external_wrench.timeout"));
    if (ew["fadeout_time"])
      g.wrench.fadeout_time =
          std::max(0.0, num(ew["fadeout_time"], "external_wrench.fadeout_time"));
    if (ew["contact_threshold"])
      g.wrench.contact_threshold =
          std::max(0.0, num(ew["contact_threshold"], "external_wrench.contact_threshold"));
    if (ew["contact_release_ratio"])
      // Clamped strictly below 1 so the ⇄ transition always keeps a hysteresis
      // band (§10.6 MUST): equal thresholds chatter at the boundary.
      g.wrench.contact_release_ratio = std::clamp(
          num(ew["contact_release_ratio"], "external_wrench.contact_release_ratio"), 0.0, 0.99);
  }

  // §7.6: the cascade exists to turn a MEASURED force into motion. With no
  // source the compliant frame never leaves X_d and the controller is a §6.2
  // impedance controller with extra steps — one that already exists and is
  // better tested. A configure error, not a fallback.
  if (!wrench_enabled)
    throw std::runtime_error(
        "CascadedComplianceController: external_wrench.enabled must be true (§7.6 — the outer "
        "admittance loop has no input otherwise; use TaskImpedanceController for the A=NONE law)");

  // Torque output only. Rejected fail-fast BEFORE committing gains so a bad
  // reconfigure never mutates live state.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque")
      throw std::runtime_error(
          "CascadedComplianceController: command_type must be 'torque' (got '" + s + "')");
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
  // Same reason as set_gains(): a reconfigure retires the published MUST-1 ratio.
  bandwidth_eval_pending_.store(true, std::memory_order_release);
  command_type_ = CommandType::kTorque;
}

}  // namespace rtc
