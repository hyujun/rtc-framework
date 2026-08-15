#include "integrated_bringup/support/momentum_observer_wiring.hpp"

#include "rtc_controller_interface/device_readability.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace integrated_bringup {

bool MomentumObserverInputsReadable(const rtc::DeviceState& dev, int dof) noexcept {
  return rtc::IsLaneReadable(dev, rtc::StateLane::kPosition, dof) &&
         rtc::IsLaneReadable(dev, rtc::StateLane::kVelocity, dof) &&
         rtc::IsLaneReadable(dev, rtc::StateLane::kEffort, dof);
}

void ConfigureMomentumObserverWiring(rtc_urdf_bridge::RtModelHandle& handle, int device_index,
                                     int dof, std::span<const double> gains,
                                     MomentumObserverWiring& w) {
  if (device_index < 0 || device_index >= rtc::ControllerState::kMaxDevices) {
    throw std::invalid_argument("ConfigureMomentumObserverWiring: device_index out of range: " +
                                std::to_string(device_index));
  }
  if (dof != handle.nv()) {
    throw std::invalid_argument("ConfigureMomentumObserverWiring: dof (" + std::to_string(dof) +
                                ") != handle.nv() (" + std::to_string(handle.nv()) +
                                ") — the observer works in the model's velocity space");
  }

  // Init() first: it validates the gains and throws, and leaving the wiring
  // unconfigured on that path keeps a half-built observer from being ticked.
  w.observer.Init(dof, gains);

  w.handle = &handle;
  w.dof = dof;
  w.device_index = device_index;

  w.v_pin.setZero(dof);
  w.tau_pin.setZero(dof);
  w.p_pin.setZero(dof);
  w.ctv_pin.setZero(dof);
  w.residual_device.assign(static_cast<std::size_t>(dof), 0.0);

  w.configured = true;
}

void ResetMomentumObserverRtState(MomentumObserverWiring& w) noexcept {
  w.observer.ResetRtState();
  std::fill(w.residual_device.begin(), w.residual_device.end(), 0.0);
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
