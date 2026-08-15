#ifndef INTEGRATED_BRINGUP_SUPPORT_MOMENTUM_OBSERVER_WIRING_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_MOMENTUM_OBSERVER_WIRING_HPP_

// Wiring for the generalized-momentum observer (#135 Layer 1) — the layer that
// turns a DeviceState plus a Pinocchio handle into the four nv-vectors
// rtc::estimation::MomentumObserver consumes. Split of responsibilities:
//   ConfigureMomentumObserverWiring (non-RT, on_configure): latch the handle and
//     the device slot, size every scratch buffer, Init() the observer.
//   UpdateMomentumObserver (RT): ask the lane gate, assemble p / C^T v / g /
//     tau_m in ONE joint order, run the observer, publish the residual back in
//     device order.
//   ResetMomentumObserverRtState (non-RT, on_activate): drop the latches.
//
// THIS FILE OWNS THE COORDINATE CONTRACT, and it is the whole reason the
// observer takes vectors instead of matrices. RtModelHandle is asymmetric:
// Compute*(q, v, ...) take DEVICE order and reorder internally, while
// GetMassMatrix() / GetCoriolisMatrix() / GetGeneralizedGravity() hand back
// PINOCCHIO order. So `GetMassMatrix() * dev.velocities` is a genuine order mix
// that yields a finite, smooth, wrong momentum — no NaN, no closed gate, and a
// payload estimate downstream that is stable and reproducible and false. Every
// product below is therefore formed after ReorderInput() has gathered the
// device-order lane into Pinocchio order, the observer runs entirely in
// Pinocchio order, and ReorderOutput() puts the residual back at the end.
//
// THE MODEL SURFACE IS THE ARM SUB-MODEL (open chain), not a WBC tree. #135's
// [MO-1] is arm dynamics, and ReducedDynamicsProvider covers only M/h/g — there
// is no reduced path for the Coriolis matrix, so on a closed-chain tree the
// cached reduced M/g would meet an open-chain C inside [MO-3a] and the residual
// would be quietly wrong in exactly the way described above.
//
// THE LANE GATE IS HERE, NOT IN THE OBSERVER, because rtc::IsLaneReadable lives
// in rtc_controller_interface and rtc_controllers does not depend on it. The
// observer receives the verdict as Update() vs Hold().

#include "rtc_controllers/estimation/momentum_observer.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include "rtc_base/types/types.hpp"

#include <Eigen/Core>

#include <span>
#include <vector>

namespace integrated_bringup {

/// Every joint-space lane this observer reads, asked as one question (#446).
///
/// All three, because [MO-3a] reads all three: q feeds M / C / g, q̇ feeds the
/// momentum and the Coriolis product, and tau_m is the measured torque. Asking
/// only about efforts would leave the other two behind a gate no one opened,
/// which is the shape #446 exists to retire.
[[nodiscard]] bool MomentumObserverInputsReadable(const rtc::DeviceState& dev, int dof) noexcept;

/// Per-controller observer state. Scratch is Pinocchio-order and sized once.
struct MomentumObserverWiring {
  rtc::estimation::MomentumObserver observer;
  /// Non-owning. The arm sub-model handle the controller already builds.
  rtc_urdf_bridge::RtModelHandle* handle{nullptr};
  int dof{0};
  int device_index{0};
  bool configured{false};

  // ── Pinocchio-order scratch (preallocated; RT touches these, never resizes) ──
  Eigen::VectorXd v_pin;
  Eigen::VectorXd tau_pin;
  Eigen::VectorXd p_pin;
  Eigen::VectorXd ctv_pin;
  /// Residual re-expressed in the caller's device order.
  std::vector<double> residual_device;
};

/// Non-RT. Throws std::invalid_argument on a null handle, a device index outside
/// [0, ControllerState::kMaxDevices), a dof that does not match the handle's nv,
/// or gains the observer rejects.
///
/// `dof` must equal handle.nv(): the observer works in the model's velocity
/// space, so a mismatch is a configuration error rather than something to
/// truncate — truncating would silently observe a subset of the arm and report
/// the rest of it as external torque.
void ConfigureMomentumObserverWiring(rtc_urdf_bridge::RtModelHandle& handle, int device_index,
                                     int dof, std::span<const double> gains,
                                     MomentumObserverWiring& w);

/// RT. Returns true iff the residual advanced this tick.
///
/// A closed gate routes to MomentumObserver::Hold(), which freezes the residual
/// rather than zeroing it — a zero would assert "no external torque", which is
/// precisely the fabricated measurement the gate exists to prevent.
bool UpdateMomentumObserver(const rtc::ControllerState& state,
                            MomentumObserverWiring& w) noexcept;

/// Non-RT (on_activate). Drops every per-tick latch; keeps Init-time state.
void ResetMomentumObserverRtState(MomentumObserverWiring& w) noexcept;

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_MOMENTUM_OBSERVER_WIRING_HPP_
