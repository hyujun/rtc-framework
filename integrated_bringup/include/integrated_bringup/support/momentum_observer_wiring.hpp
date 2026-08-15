#ifndef INTEGRATED_BRINGUP_SUPPORT_MOMENTUM_OBSERVER_WIRING_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_MOMENTUM_OBSERVER_WIRING_HPP_

// Wiring for the generalized-momentum observer (#135 Layer 1/1b) — the layer
// that turns a DeviceState plus an arm sub-model into the four nv-vectors
// rtc::estimation::MomentumObserver consumes. Split of responsibilities:
//   ConfigureMomentumObserverWiring (non-RT, on_configure): build the observer's
//     OWN model handle, pin the joint order on it, size every scratch buffer,
//     Init() the observer.
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
// THE HANDLE IS OWNED, NOT BORROWED (#135 Layer 1b, D11), and both halves of
// that decision are load-bearing:
//
//   1. THE REORDER MAP HAS TO BE SET BY SOMEBODY. A handle whose
//      SetJointOrder() was never called falls back to memcpy, which is correct
//      only when the device order happens to equal the model's internal order.
//      No demo controller calls SetJointOrder on its arm handle (only on the
//      hand handle), so borrowing one would quietly turn the contract above
//      into an unverified assumption about the URDF — and the failure it
//      protects against is by construction invisible. Configure() therefore
//      pins the order here and throws when the names do not resolve.
//   2. PINOCCHIO Data IS NOT RE-ENTRANT WITHIN A TICK. The controllers use
//      their arm handle for FK and read the result back a few statements later
//      (e.g. the WBC TCP goal). Running crba / computeCoriolisMatrix on that
//      same handle in between would overwrite the data those readers are still
//      holding. A separate handle removes the ordering coupling entirely, and
//      costs one arm-sized pinocchio::Data — sharing would have saved nothing
//      else, since the observer issues its own Compute* calls regardless.
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
//
// tau_m UNIT CONTRACT (D10 clause 1): DeviceState::efforts is JOINT TORQUE in
// N·m. A backend that measures motor current converts at the backend boundary
// and keeps the raw current in DeviceState::motor_efforts. Nothing downstream
// of here can check that — a lane in the wrong unit is fresh, full and wrong.

#include "integrated_bringup/logging/momentum_observer_log_pod.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controllers/estimation/momentum_observer.hpp"
#include "rtc_controllers/estimation/payload_estimator.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include "rtc_base/types/types.hpp"

#include <rclcpp/logger.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
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
  /// Owned arm sub-model handle with the device joint order pinned on it —
  /// see the header preamble for why this is not the controller's arm handle.
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
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

  /// True once Configure has run. False is the shipped state on a variant whose
  /// demo_shared.yaml carries no `momentum_observer` block — the controllers
  /// gate every observer call on it, so a disabled wiring costs one branch per
  /// tick and nothing else. Mirrors PullEstimatorWiring::enabled().
  [[nodiscard]] bool enabled() const noexcept { return configured; }

  /// Residual r [N·m] in DEVICE order, dof() entries. Meaningful only while
  /// observer.valid(); frozen at its last accepted value otherwise.
  [[nodiscard]] std::span<const double> residual() const noexcept {
    return {residual_device.data(), residual_device.size()};
  }

  // ── Layer 2A: payload estimation (#135) ────────────────────────────────────
  //
  // IT LIVES HERE BECAUSE THE JOINT ORDER DOES. PayloadEstimator needs J and r
  // in one order, and this struct is where the two orders are known to differ:
  // GetFrameJacobian hands back PINOCCHIO-order columns while `residual()`
  // above is DEVICE order. Running the estimator from a controller off
  // `residual()` would therefore pair mismatched orders and produce a finite,
  // smooth, wrong wrench — the same failure mode the header preamble opens
  // with, one layer up. UpdateMomentumObserver runs it against the
  // Pinocchio-order residual before ReorderOutput, and the wrench that comes
  // out is Cartesian, so no joint order survives into the result at all.
  rtc::estimation::PayloadEstimator payload;
  bool payload_configured{false};
  pinocchio::FrameIndex payload_frame{0};
  double max_arm_velocity{0.0};
  double max_peripheral_velocity{0.0};
  double settle_time_constants{0.0};
  /// Slowest observer gain [1/s] — the settle gate is stated in time constants
  /// and the slowest joint is the one that decides when the residual is ready.
  double min_gain{0.0};
  /// ᵂg from the arm model, latched at configure (the model cannot change).
  Eigen::Vector3d gravity_world{Eigen::Vector3d::Zero()};
  /// 6×nv payload-frame Jacobian, preallocated — RT writes it every tick.
  Eigen::MatrixXd J_payload;

  [[nodiscard]] bool payload_enabled() const noexcept {
    return configured && payload_configured;
  }
};

/// Non-RT. Builds the wiring's own RtModelHandle over `arm_model` and pins
/// `arm_joint_names` on it as the device order.
///
/// Throws std::invalid_argument on a null model, a device index outside
/// [0, ControllerState::kMaxDevices), an arm_joint_names size that does not
/// match the model's nv, a joint name the model does not carry, or gains the
/// observer rejects. Leaves `w` disabled (configured == false) on every throw
/// path, so a caller that downgrades the throw to a warning cannot then tick a
/// half-built observer.
///
/// The dof is `arm_joint_names.size()` and it must equal the model's nv: the
/// observer works in the model's velocity space, so a mismatch is a
/// configuration error rather than something to truncate — truncating would
/// silently observe a subset of the arm and report the rest of it as external
/// torque.
void ConfigureMomentumObserverWiring(std::shared_ptr<const pinocchio::Model> arm_model,
                                     std::span<const std::string> arm_joint_names,
                                     int device_index, std::span<const double> gains,
                                     MomentumObserverWiring& w);

/// RT. Returns true iff the residual advanced this tick.
///
/// A closed gate routes to MomentumObserver::Hold(), which freezes the residual
/// rather than zeroing it — a zero would assert "no external torque", which is
/// precisely the fabricated measurement the gate exists to prevent. A disabled
/// wiring returns false without touching anything.
bool UpdateMomentumObserver(const rtc::ControllerState& state,
                            MomentumObserverWiring& w) noexcept;

/// RT. Declare this tick's inputs unusable without offering any — the E-STOP
/// path, and any tick where the controller skips its control law.
///
/// The residual freezes and the observer arms a reseed, so resuming does not
/// bill the momentum change accumulated during the gap as an external torque.
/// A row logged from this tick therefore carries valid=0 and last tick's
/// residual, which is what "the observer stopped looking" should read like.
/// No-op on a disabled wiring.
///
/// The Layer 2A payload estimate goes down with it (kObserverInvalid). It is a
/// function of `r`, so a tick that did not advance `r` carries no new payload
/// evidence and leaving it valid would republish the last estimate as if it had
/// just been measured. This is also why UpdateMomentumObserver's lane gate
/// calls THIS rather than w.observer.Hold(): both hold paths must stay one
/// call site or they drift apart, which they did once.
void HoldMomentumObserver(MomentumObserverWiring& w) noexcept;

/// Non-RT (on_activate). Drops every per-tick latch; keeps Init-time state.
void ResetMomentumObserverRtState(MomentumObserverWiring& w) noexcept;

/// Build the wiring from `cfg` (non-RT, on_configure / LoadConfig) — the entry
/// point the three demo controllers call, so the gain expansion and the
/// enablement rule live in one place instead of three.
///
/// Resets `w` first and leaves it DISABLED (no throw) when the
/// `momentum_observer` block is absent or `enabled: false`, or when
/// `arm_model` is null / `arm_joint_names` is empty (model-less and arm-less
/// unit-test paths). Otherwise expands `params.gains` — empty means
/// kDefaultMomentumObserverGain, one entry broadcasts, dof entries are used as
/// given — and forwards to ConfigureMomentumObserverWiring.
///
/// Throws std::invalid_argument on a gain list that is neither 1 nor dof long,
/// and propagates everything ConfigureMomentumObserverWiring throws (non-RT
/// path → CallbackReturn::FAILURE). A wrong-length list is rejected rather
/// than padded because padding picks a gain for the tail joints that no config
/// asked for, and the residual it produces looks perfectly reasonable.
void BuildMomentumObserverWiring(const MomentumObserverParams& params,
                                 std::shared_ptr<const pinocchio::Model> arm_model,
                                 std::span<const std::string> arm_joint_names, int device_index,
                                 MomentumObserverWiring& w);

/// One-shot INFO describing what the wiring resolved to — enabled/disabled, the
/// dof and gains, and the CSV file that will or will not appear. Call once from
/// on_configure after BuildMomentumObserverWiring (non-RT: it formats a
/// string). The observer path is otherwise completely silent, so a wiring that
/// stayed disabled by design would be indistinguishable at runtime from one
/// that is running but never validating.
void LogMomentumObserverWiring(const rclcpp::Logger& logger, const MomentumObserverWiring& w,
                               const MomentumObserverParams& params);

/// Push one observer log row from the wiring's current residual (#135 Layer 1b).
/// RT tick tail — noexcept, heap-free; a no-op when the CSV channel is
/// unregistered or the wiring is disabled, so the three demo controllers carry
/// one call instead of a copy-pasted guard + fill + Push block.
///
/// Call it on EVERY tick, including held and E-STOP ones. A row is what makes
/// "the observer stopped looking" visible: it carries valid=0, the reason, and
/// the frozen residual. Skipping those ticks would leave gaps in the file that
/// are indistinguishable from dropped rows, which is the ambiguity #234 P-20
/// had to remove from the pull channel.
inline void PushMomentumObserverLog(rtc::LogHandle<MomentumObserverLogPod>& handle,
                                    const MomentumObserverWiring& w, double t_relative_s,
                                    std::uint64_t tick) noexcept {
  if (!handle || !w.enabled()) {
    return;
  }
  MomentumObserverLogPod pod{};
  pod.t_relative_s = t_relative_s;
  pod.tick = tick;
  const std::size_t n =
      std::min(w.residual_device.size(), MomentumObserverLogPod::kMaxArmJoints);
  double inf_norm = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double r = w.residual_device[i];
    pod.residual[i] = r;
    inf_norm = std::max(inf_norm, std::fabs(r));
  }
  pod.residual_inf_norm = inf_norm;
  pod.valid = w.observer.valid();
  pod.invalid_reason = static_cast<std::uint8_t>(w.observer.invalid_reason());
  pod.ticks_since_seed = w.observer.ticks_since_seed();

  // Layer 2A. On an unconfigured wiring these stay zero with
  // payload_reason = kNotInitialized, which is what "never asked for" reads
  // like — as opposed to a gate that closed, which names itself.
  const auto& est = w.payload.estimate();
  pod.payload_wrench = est.wrench;
  pod.payload_mass = est.mass;
  pod.payload_sigma_min = est.sigma_min;
  pod.payload_lambda_sq = est.lambda_sq;
  pod.payload_fit_error = est.fit_error;
  pod.payload_valid = w.payload.valid();
  pod.payload_reason = static_cast<std::uint8_t>(w.payload.invalid_reason());

  handle.Push(pod);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_MOMENTUM_OBSERVER_WIRING_HPP_
