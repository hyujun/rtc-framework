#ifndef UDP_HAND_DRIVER_FAKE_HAND_LPF_HPP_
#define UDP_HAND_DRIVER_FAKE_HAND_LPF_HPP_

// Shared first-order LPF joint step for the two complementary fake-hand SIL
// layers, so the dynamics live in exactly one place (no manual "keep in sync"):
//   - UdpHandController::FakeLpfStep — controller-side loop-model SIL
//   - FakeHandFirmware::StepModel    — device-side network-level SIL
// Both drive a commanded target through the same first-order lag and derive
// velocity/effort identically; the two SIL layers differ only in WHERE the step
// runs, not in the math.

namespace udp_hand_driver {

/// One joint's fake-hand sample: filtered position, its numerical derivative
/// (velocity), and a PD-torque effort placeholder.
struct FakeJointSample {
  float pos;
  float vel;
  float effort;
};

/// Pure first-order LPF step for one joint (no state — unit-testable):
///   pos = pos_old + alpha·(target − pos_old),  alpha = dt/(τ+dt)
///   vel = (pos − pos_old)·inv_dt               (inv_dt = 1/dt = step rate)
///   effort = kp·(target − pos) − kd·vel        (PD-torque placeholder)
[[nodiscard]] inline FakeJointSample FakeLpfStep(float target, float pos_old, float alpha,
                                                 float inv_dt, float kp, float kd) noexcept {
  const float pos = pos_old + alpha * (target - pos_old);
  const float vel = (pos - pos_old) * inv_dt;
  return {pos, vel, kp * (target - pos) - kd * vel};
}

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_FAKE_HAND_LPF_HPP_
