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

  ConfigureMomentumObserverWiring(std::move(arm_model), arm_joint_names, device_index, gains, w);
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
}

void HoldMomentumObserver(MomentumObserverWiring& w) noexcept {
  if (!w.configured)
    return;
  w.observer.Hold();
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
    w.observer.Hold();
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

  if (!w.observer.valid())
    return false;

  // Back to the caller's order — the residual is a joint-space quantity and its
  // consumers index it by device slot.
  const std::span<const double> r = w.observer.residual();
  const Eigen::Map<const Eigen::VectorXd> r_pin(r.data(), n);
  w.handle->ReorderOutput(r_pin, std::span<double>(w.residual_device.data(), un));
  return true;
}

}  // namespace integrated_bringup
