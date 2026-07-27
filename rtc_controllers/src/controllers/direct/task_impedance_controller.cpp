// ── Includes: project header first, then C++ stdlib ─────────────────────────
#include "rtc_controllers/direct/task_impedance_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"
#include "rtc_controllers/joint/posture_law.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"
#include "rtc_math/se3/wrench.hpp"

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
  // Configure the conditioner here as well as in LoadConfig: BesselFilterN
  // returns 0 for EVERY sample before Init() (all coefficients are zero) and
  // says nothing about it, so a controller constructed directly — as the unit
  // tests do — would otherwise silently zero the whole wrench path.
  wrench_.Configure(compliance::WrenchConditioningConfig{}, 1.0 / GetDefaultDt());
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
  qdot_ = Eigen::VectorXd::Zero(nv);
  q_null_ = Eigen::VectorXd::Zero(nv);
  tau_dev_ = Eigen::VectorXd::Zero(nv);
  tau_prev_dev_ = Eigen::VectorXd::Zero(nv);
  q_dev_ = Eigen::VectorXd::Zero(nv);
  qdot_dev_ = Eigen::VectorXd::Zero(nv);
  grav_dev_ = Eigen::VectorXd::Zero(nv);
  llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv);
  dyn_.Resize(nv, m_);

  // Gravity comes from the model, never a hard-coded 9.81 down −Z: this package
  // already tests a non-Z-gravity URDF, and the payload compensation term would
  // silently point the wrong way there.
  gravity_world_ = model_ptr_->gravity.linear();
  // The model just changed, so any previously resolved frame index is stale.
  ResolveSensorFrame();

  // Size the per-joint limit vectors to nv up front with inert defaults so the
  // safety layer has valid bounds even if OnDeviceConfigsSet is never called
  // (e.g. a direct unit-test construction). OnDeviceConfigsSet overwrites/pads.
  const auto nvz = static_cast<std::size_t>(nv);
  max_joint_torque_.assign(nvz, kDefaultMaxJointTorque);
  max_joint_velocity_.assign(nvz, kDefaultMaxJointVelocity);
  position_lower_.assign(nvz, -1e9);
  position_upper_.assign(nvz, 1e9);
}

pinocchio::FrameIndex TaskImpedanceController::LookupSensorFrame(const std::string& name) const {
  // Empty name ⇒ the wrench is expressed in the TIP body frame. Note that this
  // is NOT an identity transform into LWA: the Ad^{-T} still rotates by R_tip.
  if (name.empty())
    return tip_frame_id_;
  const auto fid = handle_->GetFrameId(name);
  if (fid == 0)  // 0 == universe == "not found" (RtModelHandle contract)
    throw std::runtime_error("TaskImpedanceController: sensor_frame '" + name +
                             "' is not a frame of the loaded model");
  return fid;
}

void TaskImpedanceController::ResolveSensorFrame() {
  sensor_frame_id_ = LookupSensorFrame(sensor_frame_name_);
}

void TaskImpedanceController::SetExternalWrench(
    std::span<const double, compliance::kWrenchDim> wrench) noexcept {
  if (!wrench_enabled_)
    return;
  wrench_.Publish(wrench);
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
  // tip_frame_id_ may have just moved to the configured tip_link, and the
  // default sensor frame IS the tip — re-resolve so the wrench transform does
  // not keep pointing at the model's last frame.
  ResolveSensorFrame();
}

// ── External wrench: read → condition → Ad^{-T} → fade (§3.2.1, §10.6) ───────

Eigen::Matrix<double, 6, 1> TaskImpedanceController::UpdateExternalWrench(
    const pinocchio::SE3& tcp, double dt, const Gains& gains, Diagnostics& diag) noexcept {
  Eigen::Matrix<double, 6, 1> f_lwa = Eigen::Matrix<double, 6, 1>::Zero();
  diag.wrench_fade = 1.0;
  if (!wrench_enabled_) {
    bias_gate_ = true;  // nothing to calibrate ⇒ never gate the FSM on it
    diag.bias_calibrated = true;
    return f_lwa;
  }

  // The whole per-tick sequence — read → bias two-latch → condition → Ad^{-T}
  // into LWA at the tip → staleness fade → contact hysteresis — lives in
  // compliance/wrench_pipeline.hpp, shared with TaskAdmittanceController rather
  // than kept as a second copy here (#236 D22). The copy this replaced predated
  // the helper and had already drifted: its (re)activation reset re-dated the
  // sample in the slot instead of disowning it, so a dead producer's last
  // reading came back as FRESH on the first tick after re-activation — the exact
  // opposite of §10.6's "an expired wrench goes to zero, never held".
  const pinocchio::SE3& sensor = handle_->GetFramePlacement(sensor_frame_id_);
  compliance::WrenchPipelineParams params;
  params.timeout = gains.wrench_timeout;
  params.fadeout_time = gains.wrench_fadeout_time;
  params.contact_threshold = gains.contact_threshold;
  params.contact_release_ratio = gains.contact_release_ratio;

  compliance::WrenchPipelineStatus ws;
  f_lwa = wrench_.Update(sensor.rotation(), sensor.translation(), tcp.translation(), gravity_world_,
                         dt, params, ws);
  // Routed out of the pipeline rather than done inside it so the E-8 SAFE_STOP
  // latch (which refuses this transition) stays the state machine's business.
  if (ws.begin_bias_calibration)
    sm_.BeginBiasCalibration();
  bias_gate_ = ws.bias_gate_released;

  diag.wrench_valid = ws.valid;
  diag.wrench_age = ws.age;
  diag.wrench_fade = ws.fade;
  diag.wrench_stale = ws.stale;
  diag.wrench_rejected = ws.rejected_samples;
  diag.bias_calibrated = ws.bias_calibrated;
  diag.in_contact = ws.in_contact;
  for (int i = 0; i < 6; ++i)
    diag.wrench_lwa[static_cast<std::size_t>(i)] = f_lwa(i);
  return f_lwa;
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
    DrainPendingTargets();
    return ComputeEstop(state, /*control_valid=*/false, diag, gains);
  }

  const int nv = handle_->nv();
  // The task dimension of THIS tick. Identical to m_ — the ctor fixes m_ at 3 or
  // 6 from the selection and it is const — so this clamp can never bite. It is a
  // WARNING FIX, not a bound anyone relies on: GCC cannot see m_'s range across
  // the inlined §6.3 helper, and without the local the fixed 6-vector heads below
  // (f_cmd.head, e.head) draw a false -Warray-bounds on their vectorised load.
  // Read it as such — an out-of-range m_ would be a construction bug, and
  // truncating it here would hide that rather than diagnose it.
  const int m = std::clamp(m_, 0, 6);
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  // ── Copy joint state (device order) ─────────────────────────────────────
  const auto& dev0 = state.devices[0];
  // Everything below is evaluated at q read from this device (F5): on a device
  // narrower than the model, or one that has not reported, the unread channels
  // read as a default-constructed 0, FK/Jacobian run at the ZERO configuration,
  // and the emitted torque is a full-arm pull toward the origin — with no fault
  // raised, because every number involved is finite.
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

  // ── FK + Jacobian + current task twist ──────────────────────────────────
  // ComputeJacobians gathers q into Pinocchio order internally, so J_full_ has
  // PINOCCHIO column order. q̇ therefore has to be gathered the same way before
  // the product: the device-order vector would pair each column with another
  // joint's velocity, which is a permuted task twist — a wrong −K_d·ν damping
  // torque, with nothing non-finite to fault on. Identity order → memcpy.
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  handle_->ReorderInput(std::span<const double>(qdot_dev_.data(), static_cast<std::size_t>(nv)),
                        qdot_);
  tcp_vel_.noalias() = J_full_ * qdot_;
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
    // The wrench path restarts with everything else: an activation must not
    // inherit an age accrued while the controller was not running, nor a bias /
    // filter history from a previous grasp, nor the SAMPLE ITSELF — the pipeline
    // DISOWNS it (Invalidate) rather than re-dating it, so a producer that died
    // before deactivation cannot come back as fresh data on the first tick of the
    // next activation. With a source configured this enters BIAS_CALIBRATING
    // (§3.2.1 "on_activate 시 자동 1회"); the state machine refuses the transition
    // while a SAFE_STOP is latched, so re-seeding cannot launder a fault (E-8).
    // No `bias_gate_ = false` here: this branch requires wrench_enabled_, so
    // UpdateExternalWrench() below unconditionally assigns bias_gate_ =
    // ws.bias_gate_released on this very tick with no return in between. Writing
    // it here reads like the latch that HOLDS BIAS_CALIBRATING, is not one, and
    // no test can tell.
    if (wrench_enabled_ && wrench_.ResetForActivation())
      sm_.BeginBiasCalibration();
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
  // returns exactly this for Split types and takes no SE3 overload.) The §7.6
  // cascade is the caller that passes a MOVING ν_d — see impedance_law.hpp.
  const Eigen::Matrix<double, 6, 1> nu_d = Eigen::Matrix<double, 6, 1>::Zero();

  for (int i = 0; i < 6; ++i)
    diag.pose_error[static_cast<std::size_t>(i)] = e(i);

  // Gain ramp α (§10.7): 0→1 over activation_ramp_time; ≤0 disables.
  const double ramp = gains.activation_ramp_time;
  const double alpha = (ramp <= 0.0) ? 1.0 : std::min(1.0, activation_elapsed_ / ramp);

  // ── Selection: J_S = S·J (linear rows first in [linear;angular]) ─────────
  // f_task is fixed-size (stack) so Compute stays heap-free; only head(m) used.
  // The §6.2 law itself lives in compliance/impedance_law.hpp — shared with the
  // §7.6 cascade rather than copied into it (#236 D18, P5).
  J_S_ = J_full_.topRows(m);
  const Eigen::Matrix<double, 6, 1> f_task =
      compliance::ComputeImpedanceForce(gains.impedance, e, tcp_vel_, nu_d, alpha, m);

  // ── Joint-space gravity ĝ(q) (always needed: comp + E-STOP hold) ─────────
  handle_->ComputeGeneralizedGravity(q_span);
  gravity_ = handle_->GetGeneralizedGravity();

  // ── External wrench f_ext in LWA (zero when no source — §6.2 path intact) ─
  // Ramped by α like the rest of the task torque so activation stays continuous
  // (gravity comp is the only never-ramped term — the arm must not sag).
  const Eigen::Matrix<double, 6, 1> f_ext = alpha * UpdateExternalWrench(tcp, dt, gains, diag);

  // ── Task dynamics Λ_S / Nᵀ ───────────────────────────────────────────────
  // M(q) and its Cholesky were previously gated on the nullspace task alone
  // (PR #241 F2): the Jacobian-transpose law never touches M, so a benign
  // M-factorization hiccup must not latch SAFE_STOP on a non-redundant arm that
  // never uses M at all (nv == m ⇒ nullspace always inactive). §6.3 needs Λ_S
  // regardless of redundancy, so the gate is WIDENED here — not reverted: when
  // neither the nullspace nor inertia shaping is live, F2's narrowing still
  // holds exactly.
  bool dyn_ok = true;
  double sigma_min = std::numeric_limits<double>::infinity();
  double lambda_sq = 0.0;
  // NUM-1 at the point of use (#277): LoadConfig floors both posture gains, but
  // set_gains() writes the POD straight into the SeqLock and bypasses it. Before
  // the gate rather than only on the law's arguments — the gate tests `!= 0.0`,
  // so flooring the law alone would hold the gate open on a negative gain and
  // drag the whole Λ_S/Nᵀ block in (`need_task_dynamics` below) to project an
  // all-zero τ₀. A negative gain must behave exactly as 0 does: gate closed.
  // `FloorPostureGain` and not `std::max(0.0, ·)` — the latter returns 0.0 for
  // NaN, which would close the gate on a corrupt gain instead of letting it
  // reach the law and latch `faults.nan_inf` through `safety.finite` below.
  const double nullspace_kp = joint::FloorPostureGain(gains.nullspace_kp);
  const double nullspace_kd = joint::FloorPostureGain(gains.nullspace_kd);
  const bool nullspace_active = (nv > m) && (nullspace_kp != 0.0 || nullspace_kd != 0.0);
  const bool inertia_shaping = (formulation_ == Formulation::kInertiaShaping);
  const bool need_task_dynamics = nullspace_active || inertia_shaping;
  // f_cmd is the bracketed task force of §6.2 / §6.3; only head(m) is used and
  // it is fixed-size (stack) so Compute stays heap-free.
  Eigen::Matrix<double, 6, 1> f_cmd = f_task;
  if (need_task_dynamics) {
    handle_->ComputeMassMatrix(q_span);
    M_ = handle_->GetMassMatrix();
    M_.triangularView<Eigen::StrictlyLower>() =
        M_.triangularView<Eigen::StrictlyUpper>().transpose();
    llt_M_.compute(M_);
    if (llt_M_.info() != Eigen::Success) {
      dyn_ok = false;
    } else {
      // NUM-1 at the point of use: LoadConfig floors max_damping too, but
      // set_gains() writes the POD straight into the SeqLock and bypasses it.
      const compliance::TaskDynamics::Result r =
          dyn_.Compute(J_S_, llt_M_, gains.singularity_threshold,
                       std::max(compliance::kMinMaxDamping, gains.max_damping));
      dyn_ok = r.ok;
      sigma_min = r.sigma_min;
      lambda_sq = r.lambda_sq;
      if (dyn_ok && inertia_shaping) {
        // §6.3 itself is compliance/inertia_shaping.hpp (#236 S4) — the binding
        // hands it Λ_S rather than the TaskDynamics that produced it, so the law
        // stays free of the S2b convergence decision. Λ_S is valid here by the
        // dyn_ok gate, which is that helper's precondition.
        const compliance::InertiaShapingResult shaped =
            compliance::ComputeShapedTaskForce(gains.inertia, dyn_.LambdaS(), f_task, f_ext, m);
        f_cmd = shaped.f_cmd;
        diag.inertia_solve_failed = shaped.solve_failed;
        diag.inertia_clamped = shaped.clamped;
      }
      if (dyn_ok && nullspace_active) {
        // Posture task — joint/posture_law.hpp (#236 S6). This loop and the
        // cascade's were character-for-character identical, which is the ARCH-3
        // trigger the extraction resolves; the reference is an argument so the
        // measured seed here and the OSC's configured posture are one law.
        const auto nvu = static_cast<std::size_t>(nv);
        joint::ComputePostureTorque(
            nullspace_kp, nullspace_kd,
            joint::PostureInputs{
                {q_null_.data(), nvu}, {q_dev_.data(), nvu}, {qdot_dev_.data(), nvu}},
            nvu, {tau_posture_dev_.data(), nvu});
        // Posture is device-order; gather to Pinocchio order before the (Pinocchio)
        // projector Nᵀ. Identity order → memcpy (unchanged).
        handle_->ReorderInput(
            std::span<const double>(tau_posture_dev_.data(), static_cast<std::size_t>(nv)),
            tau_posture_);
        dyn_.ProjectNullspace(tau_posture_, tau_null_);
      }
    }
  }

  // ── Task torque τ = Jᵀ Sᵀ f_cmd ──────────────────────────────────────────
  // Under kJacobianTranspose f_cmd == f_task and Λ is never formed, so the law
  // stays free of task-space singularities (§6.5). Under kInertiaShaping f_cmd
  // carries the Λ_S Λ_d⁻¹ factor and that immunity is gone — which is exactly why
  // the σ_min faults below are re-armed for it.
  tau_.noalias() = J_S_.transpose() * f_cmd.head(m);
  if (dyn_ok && nullspace_active)
    tau_.noalias() += alpha * tau_null_;

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
  // Bound only the REGULATED task DoF: under TRANSLATION_ONLY (m = 3) the rotation
  // rows of `e` are left to the soft nullspace posture task and may drift by
  // design, so folding them into the norm would spuriously latch SAFE_STOP even
  // while translation tracking is perfect. e.head(m) == the full 6D norm for
  // FULL_SE3 (m = 6), so this is a no-op there.
  faults.pose_error_exceeded = e.head(m).norm() > gains.pose_error_limit;
  // The σ_min faults follow the SAME widened gate as Λ_S itself: §6.5's
  // singularity exposure is a property of USING Λ_S, and §6.3 uses it whether or
  // not the robot is redundant. Under kJacobianTranspose the F2 narrowing is
  // untouched — Λ is never formed there, so there is nothing to blow up.
  faults.sigma_below_critical = need_task_dynamics && (sigma_min < gains.singularity_critical);
  faults.saturation_persist = saturation_elapsed_ > gains.saturation_persist_time;
  faults.sigma_below_threshold = need_task_dynamics && (sigma_min < gains.singularity_threshold);
  // Wrench loss degrades, never latches (§10.6: the middle state between normal
  // and fatal). The fade above has already ramped f_ext to zero, so DEGRADED
  // here means "running the A=NONE law with reduced trust", not "stopped".
  faults.wrench_timeout = diag.wrench_stale;
  // §6.1 at RUNTIME. LoadConfig refuses TRANSLATION_ONLY with a non-positive
  // K_pⁿ, but set_gains() writes the POD straight into the SeqLock and never
  // passes configure, so the one mode that hands orientation to the posture task
  // could reach a tick with the posture gate closed and nothing would say so —
  // the floor above is what makes a negative gain read as zero here, and zero is
  // precisely the value §6.1 exists to reject. Read from the FLOORED gain, not
  // from `nullspace_active`, so a non-redundant arm (nv ≤ m, gate closed for a
  // reason configure already accepts) is not accused of a gain fault.
  faults.posture_authority_lost =
      (selection_ == TaskSelection::kTranslationOnly) && !(nullspace_kp > 0.0);
  const bool ramp_done = (alpha >= 1.0);
  const auto cstate =
      sm_.Step(faults, ramp_done, dt, kDegradedRecoveryTime, bias_gate_, diag.in_contact);

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

  // Both lanes are 6-wide (x,y,z,r,p,y) and every consumer reads all six —
  // device_state_log_pod / pod_fill emit them straight to CSV. Filling only the
  // translation logs "no rotation" where the honest reading is "not measured",
  // the one reading an operator cannot tell apart from a real result (§3.5 of
  // docs/compliance-conventions.md; 43fd96e closed the same defect on the
  // admittance controller). ZYX Euler at the boundary, as §10 Style requires.
  const Eigen::Vector3d rpy_actual = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy_actual.x();
  output.actual_task_positions[4] = rpy_actual.y();
  output.actual_task_positions[5] = rpy_actual.z();
  // The goal lane carries X_d. Admittance/cascade report the COMPLIANT frame X_c
  // for the same underlying reason — the lane must show the thing the controller
  // actually commands — and this controller has no X_c: §6.2/§6.3 regulate toward
  // the setpoint itself. Leaving the lane at zero while claiming goal_type ==
  // kTask reads as "commanding the world origin at identity attitude", which is a
  // pose the arm can be nowhere near. Under TRANSLATION_ONLY the rotation of X_d
  // is still what was commanded (the law simply does not regulate it) — `m_` is a
  // controller configuration, not a property of the goal being reported.
  const Eigen::Vector3d rpy_goal = pinocchio::rpy::matrixToRpy(goal_pose_.rotation());
  output.task_goal_positions[0] = goal_pose_.translation().x();
  output.task_goal_positions[1] = goal_pose_.translation().y();
  output.task_goal_positions[2] = goal_pose_.translation().z();
  output.task_goal_positions[3] = rpy_goal.x();
  output.task_goal_positions[4] = rpy_goal.y();
  output.task_goal_positions[5] = rpy_goal.z();
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

  // F5 on the hold path too: ĝ(q) − D·q̇ is only a hold if q and q̇ are real.
  // With an unreadable device the unread channels are 0, so this would emit the
  // gravity load of the ZERO configuration and call it a hold. There is no
  // honest torque here, and the answer is the same one ComputeNoJointState
  // gives — a zero-length "no update" rather than a fabricated value. Note the
  // alternative that was rejected (#236 E-8): emitting nc0 zeros is a REAL
  // 0 N·m command, which on a torque-mode arm is a drop, not a stop.
  if (!dev0.valid || dev0.num_channels < nv) {
    ControllerOutput no_state_output;
    no_state_output.num_devices = state.num_devices;
    no_state_output.devices[0].num_channels = 0;
    no_state_output.command_type = command_type_;
    target_initialized_.store(false, std::memory_order_release);
    Diagnostics dn = diag;
    dn.control_valid = false;
    diag_lock_.Store(dn);
    return no_state_output;
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

void TaskImpedanceController::DrainPendingTargets() noexcept {
  PendingTarget discarded{};
  while (pending_targets_.Pop(discarded)) {
    // discard: a command issued while held must not survive recovery
  }
}

ControllerOutput TaskImpedanceController::ComputeNoJointState(const ControllerState& state,
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
  //
  // Why this stays a ride-through instead of escalating on a timer (#236 E-8,
  // decided against options (a)/(b)): the realistic failure — a backend that
  // stops publishing mid-run — never reaches this gate at all. `valid` latches
  // true on the first state message and is never cleared, so a dropout leaves
  // the stamp stale, not the flag, and CM's 50 Hz device watchdog trips a global
  // E-STOP after `device_timeout` (1 s on real HW), after which CM substitutes
  // BuildHoldOutput wholesale. What actually reaches this gate is a device
  // narrower than the model — a startup configuration fault, where the backend
  // has published nothing yet, so "no update" means the drive was never
  // commanded rather than "hold the last torque". A timer here would turn that
  // into "drop the arm N seconds after every misconfigured startup".
  output.devices[0].num_channels = 0;
  rtc::utils::PassthroughSecondaryDevices(state, output, target_seqlock_.Load().targets);
  output.command_type = command_type_;

  // Whatever X_d / posture state exists was seeded from a joint state this tick
  // could not read, so the next controllable tick re-seeds from measurement.
  target_initialized_.store(false, std::memory_order_release);

  diag.control_valid = false;
  diag_lock_.Store(diag);
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
    // Even without YAML this is a CONFIGURE, so it runs the same three steps the
    // YAML path does — floor, §6.4, §6.1 — on the gains it has. NUM-6's loader
    // half is "regardless of whether the key is present", and an absent NODE is
    // the widest case of that: these gains come from the constructor or a
    // previous set_gains(), i.e. exactly the two paths the floor exists for. The
    // floor is also STORED and not just evaluated for the guard — otherwise
    // get_gains() would keep reporting a gain Compute() refuses to run, on the
    // one path where nothing else rewrites the POD.
    auto g0 = gains_lock_.Load();
    g0.nullspace_kp = joint::FloorPostureGain(g0.nullspace_kp);
    g0.nullspace_kd = joint::FloorPostureGain(g0.nullspace_kd);
    if (g0.nullspace_kp > 0.0)
      g0.nullspace_kd = std::max(g0.nullspace_kd, 2.0 * std::sqrt(g0.nullspace_kp));
    gains_lock_.Store(g0);
    if (selection_ == TaskSelection::kTranslationOnly && !(g0.nullspace_kp > 0.0))
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
  load3(cfg["kp_pos"], g.impedance.kp_pos);
  load3(cfg["kd_pos"], g.impedance.kd_pos);
  load3(cfg["kp_rot"], g.impedance.kp_rot);
  load3(cfg["kd_rot"], g.impedance.kd_rot);
  if (cfg["nullspace_stiffness"])
    g.nullspace_kp = cfg["nullspace_stiffness"].as<double>();
  if (cfg["nullspace_damping"])
    g.nullspace_kd = cfg["nullspace_damping"].as<double>();
  if (cfg["singularity_threshold"])
    g.singularity_threshold =
        std::max(compliance::kMinSigma0, cfg["singularity_threshold"].as<double>());
  if (cfg["singularity_critical"])
    g.singularity_critical = std::max(0.0, cfg["singularity_critical"].as<double>());
  if (cfg["max_damping"])
    g.max_damping = std::max(compliance::kMinMaxDamping, cfg["max_damping"].as<double>());
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

  // Posture gains floored at 0 (#277), HERE — after the parse and before the two
  // rules below, because the order is what makes those two mean what they read
  // as, and unconditionally rather than inside the `if (cfg[…])` above, because
  // when the key is ABSENT the value still arrives from the constructor or a
  // previous set_gains() and both rules must judge the gain Compute() will run:
  //   • §6.4's correction is `if (kp > 0.0)`, so a negative K_pⁿ skipped the
  //     damping floor entirely — the configuration that most needs damping was
  //     the one that never got it.
  //   • §6.1's guard tests `kp == 0.0` while its message claims `> 0`, so a
  //     negative K_pⁿ walked straight through the check that exists precisely
  //     because TRANSLATION_ONLY leaves orientation to the null space.
  // Compute() floors again at the point of use because set_gains() bypasses
  // configure entirely (NUM-1) — the same treatment max_damping gets. The two
  // rules below do NOT get that second half; see §6.4's note.
  g.nullspace_kp = joint::FloorPostureGain(g.nullspace_kp);
  g.nullspace_kd = joint::FloorPostureGain(g.nullspace_kd);

  // §6.4 nullspace damping floor K_dⁿ ≥ 2√K_pⁿ (also allows K_pⁿ=0 pure damping).
  //
  // CONFIGURE-TIME ONLY, deliberately — σ₀'s side of the NUM-1 split, not
  // λ_max's. This is not a bound on the gain being read but a REWRITE of a
  // different gain, so putting it on the tick would (a) make get_gains() and
  // Compute() disagree about K_dⁿ for every positive K_pⁿ and (b) oblige every
  // literal oracle of this posture block to mirror a configure-time rule. It
  // also buys less than the floor does: `set_gains({kp, kd < 2√kp})` leaves an
  // under-damped null space — oscillatory, bounded, and visible — where an
  // unfloored negative gain leaves a DIVERGENT one that Nᵀ hides. If a caller
  // ever needs the correction at runtime, the honest fix is to apply it where
  // the POD is installed (set_gains), not to re-derive it every tick.
  if (g.nullspace_kp > 0.0)
    g.nullspace_kd = std::max(g.nullspace_kd, 2.0 * std::sqrt(g.nullspace_kp));

  // §6.1: TRANSLATION_ONLY leaves orientation to the nullspace, so zero nullspace
  // stiffness would let it drift uncontrolled — a configure error, not a warning.
  // `!(kp > 0.0)` and not `kp == 0.0`: the message claims `> 0`, and after the
  // floor above the only remaining way to fail it while comparing equal to
  // nothing is a NaN gain, which is exactly the value that must not configure.
  if (selection_ == TaskSelection::kTranslationOnly && !(g.nullspace_kp > 0.0))
    throw std::runtime_error(
        "TaskImpedanceController: TRANSLATION_ONLY requires nullspace_stiffness > 0 (§6.1)");

  // ── Axis A: formulation (§6.2 vs §6.3) ─────────────────────────────────────
  Formulation formulation = Formulation::kJacobianTranspose;
  if (cfg["formulation"]) {
    const auto s = cfg["formulation"].as<std::string>();
    if (s == "jacobian_transpose")
      formulation = Formulation::kJacobianTranspose;
    else if (s == "inertia_shaping")
      formulation = Formulation::kInertiaShaping;
    else
      throw std::runtime_error(
          "TaskImpedanceController: formulation must be 'jacobian_transpose' or 'inertia_shaping' "
          "(got '" +
          s + "')");
  }

  // §5.2 MUST — inertia shaping stays OFF unless asked for, and Λ_d defaults to
  // Λ_S (neutral) unless a desired inertia is actually supplied.
  if (const YAML::Node& di = cfg["desired_inertia"]; di && di.IsSequence()) {
    if (di.size() != 6)
      throw std::runtime_error("TaskImpedanceController: desired_inertia must have 6 entries");
    for (std::size_t i = 0; i < 6; ++i) {
      const double v = di[i].as<double>();
      // NUM-2: Λ_d⁻¹ is formed every tick — a zero or negative desired inertia
      // is not a soft-clamped tuning mistake, it is a non-SPD Λ_d.
      if (!(v > 0.0))
        throw std::runtime_error(
            "TaskImpedanceController: desired_inertia entries must be > 0 (Λ_d must be SPD)");
      g.inertia.desired_inertia[i] = v;
    }
    g.inertia.desired_inertia_natural = false;
  }
  if (cfg["max_inertia_ratio"])
    g.inertia.max_inertia_ratio = std::max(1.0, cfg["max_inertia_ratio"].as<double>());

  // ── External wrench source (§3.2.1). Disabled ⇒ A=NONE, slice-1 behaviour ──
  bool wrench_enabled = false;
  std::string sensor_frame;
  compliance::WrenchConditioningConfig wc;
  if (const YAML::Node& ew = cfg["external_wrench"]; ew && ew.IsMap()) {
    if (ew["enabled"])
      wrench_enabled = ew["enabled"].as<bool>();
    if (ew["sensor_frame"])
      sensor_frame = ew["sensor_frame"].as<std::string>();
    auto load6 = [](const YAML::Node& n, compliance::Wrench6& arr, const char* what) {
      if (!n)
        return;
      if (!n.IsSequence() || n.size() != 6)
        throw std::runtime_error(std::string("TaskImpedanceController: ") + what +
                                 " must be a 6-entry sequence [fx,fy,fz,tx,ty,tz]");
      for (std::size_t i = 0; i < 6; ++i)
        arr[i] = n[i].as<double>();
    };
    load6(ew["deadband"], wc.deadband, "external_wrench.deadband");
    load6(ew["max"], wc.max_abs, "external_wrench.max");
    if (ew["payload_mass"])
      wc.payload_mass = ew["payload_mass"].as<double>();
    if (const YAML::Node& com = ew["payload_com"]; com && com.IsSequence()) {
      if (com.size() != 3)
        throw std::runtime_error(
            "TaskImpedanceController: external_wrench.payload_com must have 3 entries");
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
      g.wrench_timeout = std::max(0.0, ew["timeout"].as<double>());
    if (ew["fadeout_time"])
      g.wrench_fadeout_time = std::max(0.0, ew["fadeout_time"].as<double>());
    if (ew["contact_threshold"])
      g.contact_threshold = std::max(0.0, ew["contact_threshold"].as<double>());
    if (ew["contact_release_ratio"])
      // Clamped strictly below 1 so the ⇄ transition always has hysteresis
      // (§10.6 MUST): equal thresholds chatter at the boundary.
      g.contact_release_ratio = std::clamp(ew["contact_release_ratio"].as<double>(), 0.0, 0.99);
  }

  // Torque-only (MuJoCo backend scope). Reject other command types fail-fast,
  // BEFORE committing gains, so a rejected reconfigure never mutates live gains.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque")
      throw std::runtime_error("TaskImpedanceController: command_type must be 'torque' (got '" + s +
                               "')");
  }

  // Last two things that can fail. Resolve the frame into a LOCAL first: a bad
  // sensor_frame must not leave the controller half-reconfigured. (The model may
  // have been swapped for a reduced one by MaybeSelectSubModel at the top of
  // this function, so the lookup has to run against the model actually in use.)
  const pinocchio::FrameIndex sensor_id = LookupSensorFrame(sensor_frame);
  wrench_.Configure(wc, 1.0 / GetDefaultDt());  // validates, then commits

  // Commit. Nothing below throws, so a rejected reconfigure leaves the live
  // gains and the live wrench path exactly as they were.
  sensor_frame_name_ = sensor_frame;
  sensor_frame_id_ = sensor_id;
  wrench_enabled_ = wrench_enabled;
  formulation_ = formulation;
  gains_lock_.Store(g);
  command_type_ = CommandType::kTorque;
}

}  // namespace rtc
