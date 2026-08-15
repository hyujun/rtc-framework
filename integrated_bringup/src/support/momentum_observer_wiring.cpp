#include "integrated_bringup/support/momentum_observer_wiring.hpp"

#include "rtc_controller_interface/device_readability.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

bool MomentumObserverInputsReadable(const rtc::DeviceState& dev, int dof) noexcept {
  return rtc::IsLaneReadable(dev, rtc::StateLane::kPosition, dof) &&
         rtc::IsLaneReadable(dev, rtc::StateLane::kVelocity, dof) &&
         rtc::IsLaneReadable(dev, rtc::StateLane::kEffort, dof);
}

void ConfigureMomentumObserverWiring(std::shared_ptr<const pinocchio::Model> arm_model,
                                     std::span<const std::string> arm_joint_names,
                                     int device_index, std::span<const double> gains,
                                     MomentumObserverWiring& w) {
  // Disable first: every throw below must leave a wiring that cannot be ticked,
  // including a re-configure that fails after a successful one.
  w.configured = false;
  w.handle.reset();

  if (arm_model == nullptr) {
    throw std::invalid_argument("ConfigureMomentumObserverWiring: null arm model");
  }
  if (device_index < 0 || device_index >= rtc::ControllerState::kMaxDevices) {
    throw std::invalid_argument("ConfigureMomentumObserverWiring: device_index out of range: " +
                                std::to_string(device_index));
  }

  const auto dof = static_cast<int>(arm_joint_names.size());
  if (dof != arm_model->nv) {
    throw std::invalid_argument(
        "ConfigureMomentumObserverWiring: arm_joint_names size (" + std::to_string(dof) +
        ") != arm model nv (" + std::to_string(arm_model->nv) +
        ") — the observer works in the model's velocity space");
  }

  auto handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(std::move(arm_model));

  // Pin the device order. Without this the reorder falls back to memcpy, which
  // is right only when the two orders coincide — see the header preamble.
  if (!handle->SetJointOrder(arm_joint_names)) {
    std::string names;
    for (const auto& n : arm_joint_names) {
      if (!names.empty())
        names += ", ";
      names += n;
    }
    throw std::invalid_argument(
        "ConfigureMomentumObserverWiring: SetJointOrder failed — the arm model does not carry "
        "every configured joint name in [" +
        names + "]");
  }

  // Init() before latching anything else: it validates the gains and throws.
  w.observer.Init(dof, gains);

  w.handle = std::move(handle);
  w.dof = dof;
  w.device_index = device_index;

  w.v_pin.setZero(dof);
  w.tau_pin.setZero(dof);
  w.p_pin.setZero(dof);
  w.ctv_pin.setZero(dof);
  w.residual_device.assign(static_cast<std::size_t>(dof), 0.0);

  w.configured = true;
}

namespace {

/// Non-RT. Arms the Layer 2A estimator on an already-configured wiring.
///
/// Every rejection path here leaves payload estimation OFF rather than throwing,
/// with one exception: a `frame` that the model does not carry throws, because
/// that is a typo in a key the user explicitly set, and silently degrading to
/// "residual only" would look identical to not having asked for it.
void ConfigurePayloadEstimator(const PayloadEstimatorParams& pp,
                               const pinocchio::Model& model,
                               const std::vector<double>& gains, MomentumObserverWiring& w) {
  if (!w.configured || !pp.has_block || !pp.enabled || pp.frame.empty()) {
    return;
  }
  // GetFrameId returns 0 (universe) for a name the model does not carry — a
  // SILENT failure, and one that would leave the estimator projecting onto the
  // world frame's Jacobian (all zeros) for the life of the run.
  const pinocchio::FrameIndex fid = w.handle->GetFrameId(pp.frame);
  if (fid == 0) {
    throw std::invalid_argument(
        "demo_shared: momentum_observer.payload_estimator.frame '" + pp.frame +
        "' is not a frame of the arm sub-model");
  }

  rtc::estimation::PayloadEstimator::Config c;
  c.sigma0 = pp.sigma0;
  c.lambda_max = pp.lambda_max;
  c.min_sigma = pp.min_sigma;
  c.max_fit_error = pp.max_fit_error;
  c.min_gravity = pp.min_gravity;
  w.payload.Init(w.dof, c);  // throws on any unusable threshold

  w.payload_frame = fid;
  w.max_arm_velocity = pp.max_arm_velocity;
  w.max_peripheral_velocity = pp.max_peripheral_velocity;
  w.settle_time_constants = pp.settle_time_constants;
  w.min_gain = *std::min_element(gains.begin(), gains.end());
  // ᵂg FROM THE MODEL, never a hard-coded 9.81·−Z (ARCH-1): the URDF decides
  // both the axis and the magnitude, and [MASS-A] divides by ‖ᵂg‖² so both
  // matter. Latched here because a Model's gravity cannot change at runtime.
  w.gravity_world = model.gravity.linear();
  w.J_payload = Eigen::MatrixXd::Zero(6, w.dof);
  w.payload_configured = true;
}

/// Largest |q̇| across every device OTHER than the observed one.
///
/// Stated as "the other devices" rather than "the hand" on purpose: the
/// observer runs on an arm sub-model whose remaining joints are pinned at a
/// reference posture (#135 D3), so ANY articulated body sharing the robot
/// injects reaction torques the arm sub-model must call external. Naming the
/// hand would encode this robot's layout into the gate (ARCH-1) and would miss
/// a third device on the next one.
[[nodiscard]] double PeripheralSpeed(const rtc::ControllerState& state, int observed) noexcept {
  double worst = 0.0;
  for (int d = 0; d < state.num_devices; ++d) {
    if (d == observed) {
      continue;
    }
    const rtc::DeviceState& dev = state.devices[static_cast<std::size_t>(d)];
    for (int i = 0; i < dev.num_channels; ++i) {
      worst = std::max(worst, std::abs(dev.velocities[static_cast<std::size_t>(i)]));
    }
  }
  return worst;
}

/// RT. The quasi-static gates, then [WRENCH-A]/[MASS-A]. Called only on a tick
/// whose residual advanced, with `r` in PINOCCHIO order.
///
/// Each gate reports ITSELF rather than a generic "held": which assumption
/// failed is the difference between "move the hand out of the way" and "the
/// pose is singular", and a single invalid flag would collapse the two.
void UpdatePayloadEstimate(const rtc::ControllerState& state, MomentumObserverWiring& w,
                           std::span<const double> r, int n) noexcept {
  using rtc::estimation::PayloadInvalidReason;

  // Settle gate. The residual restarts at zero after any re-seed and converges
  // over ~1/K_I, so an early read is a measurement of the filter, not the load.
  // Expressed in time constants so it tracks the configured gain and dt rather
  // than a tick count that silently means something else at another rate.
  const double taus = static_cast<double>(w.observer.ticks_since_seed()) * w.min_gain * state.dt;
  if (taus < w.settle_time_constants) {
    w.payload.Hold(PayloadInvalidReason::kNotConverged);
    return;
  }

  const rtc::DeviceState& dev = state.devices[static_cast<std::size_t>(w.device_index)];
  double arm_speed = 0.0;
  for (int i = 0; i < n; ++i) {
    arm_speed = std::max(arm_speed, std::abs(dev.velocities[static_cast<std::size_t>(i)]));
  }
  if (arm_speed > w.max_arm_velocity) {
    // [WRENCH-A] drops M q̈ entirely; under motion the residual carries
    // acceleration and unmodelled joint-level terms that no wrench explains.
    w.payload.Hold(PayloadInvalidReason::kArmMoving);
    return;
  }
  if (PeripheralSpeed(state, w.device_index) > w.max_peripheral_velocity) {
    // #135 D14: the hand's joints are PINNED in the arm sub-model, so its real
    // motion enters the residual as an external torque by construction and is
    // indistinguishable from a payload. Measured at ~6x the arm-motion term.
    w.payload.Hold(PayloadInvalidReason::kHandMoving);
    return;
  }

  // J in the same Pinocchio order as `r`. ComputeJacobians overwrites the
  // handle's data, which is safe HERE and only here: the mass / Coriolis /
  // gravity Refs taken above were consumed by observer.Update() before this
  // call, so nothing still points into what this overwrites (RT-5).
  const std::span<const double> q(dev.positions.data(), static_cast<std::size_t>(n));
  w.handle->ComputeJacobians(q);
  w.handle->GetFrameJacobian(w.payload_frame, pinocchio::LOCAL_WORLD_ALIGNED, w.J_payload);

  w.payload.Update(w.J_payload, r, w.gravity_world);
}

}  // namespace

void BuildMomentumObserverWiring(const MomentumObserverParams& params,
                                 std::shared_ptr<const pinocchio::Model> arm_model,
                                 std::span<const std::string> arm_joint_names, int device_index,
                                 MomentumObserverWiring& w) {
  w = MomentumObserverWiring{};

  if (!params.has_block || !params.enabled) {
    return;
  }
  // Model-less / hand-less callers (unit tests, a variant with no arm sub-model)
  // stay disabled rather than throwing: there is nothing misconfigured about a
  // caller that has no arm to observe.
  if (arm_model == nullptr || arm_joint_names.empty()) {
    return;
  }

  const auto dof = static_cast<int>(arm_joint_names.size());
  const auto& g = params.gains;
  std::vector<double> gains;
  if (g.empty()) {
    gains.assign(static_cast<std::size_t>(dof), kDefaultMomentumObserverGain);
  } else if (g.size() == 1) {
    gains.assign(static_cast<std::size_t>(dof), g.front());
  } else if (g.size() == static_cast<std::size_t>(dof)) {
    gains = g;
  } else {
    throw std::invalid_argument(
        "demo_shared: momentum_observer.gains has " + std::to_string(g.size()) +
        " entries — expected 1 (broadcast) or " + std::to_string(dof) + " (arm dof)");
  }

  // Keep a handle on the model: the payload step needs its gravity, and the
  // move below would otherwise consume the only reference.
  const auto model_for_payload = arm_model;
  ConfigureMomentumObserverWiring(std::move(arm_model), arm_joint_names, device_index, gains, w);

  ConfigurePayloadEstimator(params.payload, *model_for_payload, gains, w);
}

void LogMomentumObserverWiring(const rclcpp::Logger& logger, const MomentumObserverWiring& w,
                               const MomentumObserverParams& params) {
  if (!w.enabled()) {
    const char* why = !params.has_block
                          ? "no 'momentum_observer' block in demo_shared.yaml"
                          : (!params.enabled ? "block present but enabled: false"
                                             : "no arm sub-model to observe");
    RCLCPP_INFO(logger,
                "[momentum_observer] disabled — %s. No momentum_observer.csv will be written.",
                why);
    return;
  }
  std::string gains;
  for (const auto& v : params.gains) {
    if (!gains.empty()) {
      gains += ", ";
    }
    gains += std::to_string(v);
  }
  if (gains.empty()) {
    gains = std::to_string(kDefaultMomentumObserverGain) + " (default)";
  }
  RCLCPP_INFO(logger,
              "[momentum_observer] enabled — device %d, dof %d, K_I [%s]. Residual r is logged to "
              "momentum_observer.csv; held/E-STOP ticks appear as valid=0 rows.",
              w.device_index, w.dof, gains.c_str());
}

void ResetMomentumObserverRtState(MomentumObserverWiring& w) noexcept {
  w.observer.ResetRtState();
  std::fill(w.residual_device.begin(), w.residual_device.end(), 0.0);
  if (w.payload_configured)
    w.payload.ResetRtState();
}

void HoldMomentumObserver(MomentumObserverWiring& w) noexcept {
  if (!w.configured)
    return;
  w.observer.Hold();
  // The payload estimate is downstream of a residual that just froze, so it
  // must freeze with it — otherwise an E-STOP tick would keep republishing the
  // last payload as if it had been measured on this tick.
  if (w.payload_configured)
    w.payload.Hold(rtc::estimation::PayloadInvalidReason::kObserverInvalid);
}

bool UpdateMomentumObserver(const rtc::ControllerState& state,
                            MomentumObserverWiring& w) noexcept {
  if (!w.configured || w.handle == nullptr)
    return false;
  if (w.device_index >= state.num_devices)
    return false;

  const rtc::DeviceState& dev = state.devices[static_cast<std::size_t>(w.device_index)];
  const int n = w.dof;

  if (!MomentumObserverInputsReadable(dev, n)) {
    // Through HoldMomentumObserver, not w.observer.Hold() directly: a closed
    // lane gate freezes the residual one branch EARLIER than the observer's own
    // rejection below, so it cannot ride along with the payload hold down
    // there. Holding only the observer here left payload_valid standing on a
    // tick that produced no new residual — the estimate would keep publishing
    // as if it had just been measured. One call site so the two hold paths
    // cannot diverge again.
    HoldMomentumObserver(w);
    return false;
  }

  const std::span<const double> q(dev.positions.data(), static_cast<std::size_t>(n));
  const std::span<const double> v(dev.velocities.data(), static_cast<std::size_t>(n));
  const std::span<const double> tau(dev.efforts.data(), static_cast<std::size_t>(n));

  // Device order in — these three reorder internally.
  w.handle->ComputeMassMatrix(q);
  w.handle->ComputeCoriolisMatrix(q, v);
  w.handle->ComputeGeneralizedGravity(q);

  // Device order → Pinocchio order, so the products below are formed inside one
  // coordinate system (see the header's coordinate contract).
  w.handle->ReorderInput(v, w.v_pin);
  w.handle->ReorderInput(tau, w.tau_pin);

  // Explicit Eigen::Ref types rather than `auto`: an expression template bound
  // to auto would defer evaluation past the next Compute* call that overwrites
  // the handle's data (RT-5).
  const Eigen::Ref<const Eigen::MatrixXd> mass = w.handle->GetMassMatrix();
  const Eigen::Ref<const Eigen::MatrixXd> coriolis = w.handle->GetCoriolisMatrix();
  const Eigen::Ref<const Eigen::VectorXd> gravity = w.handle->GetGeneralizedGravity();

  // p = M q̇ and C^T q̇ — noalias into preallocated scratch, so no temporary is
  // materialised on the RT path. C^T, not C: C^T v != C v, which is also why
  // the cheaper h = C v + g cannot stand in for this term.
  w.p_pin.noalias() = mass * w.v_pin;
  w.ctv_pin.noalias() = coriolis.transpose() * w.v_pin;

  const auto un = static_cast<std::size_t>(n);
  w.observer.Update(std::span<const double>(w.p_pin.data(), un),
                    std::span<const double>(w.ctv_pin.data(), un),
                    std::span<const double>(gravity.data(), un),
                    std::span<const double>(w.tau_pin.data(), un), state.dt);

  if (!w.observer.valid()) {
    if (w.payload_configured)
      w.payload.Hold(rtc::estimation::PayloadInvalidReason::kObserverInvalid);
    return false;
  }

  // The observer ran in Pinocchio order, so its own output still is — this is
  // the ONLY point where the residual and a GetFrameJacobian share one order.
  const std::span<const double> r = w.observer.residual();
  const Eigen::Map<const Eigen::VectorXd> r_pin(r.data(), n);

  // ── Layer 2A (#135): payload estimate, before the reorder below ───────────
  if (w.payload_configured) {
    UpdatePayloadEstimate(state, w, r, n);
  }

  // Back to the caller's order — the residual is a joint-space quantity and its
  // consumers index it by device slot.
  w.handle->ReorderOutput(r_pin, std::span<double>(w.residual_device.data(), un));
  return true;
}

}  // namespace integrated_bringup
