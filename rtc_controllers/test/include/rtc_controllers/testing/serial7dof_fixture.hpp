// ── The serial_7dof fixture shared by the #236 extraction suites (test-only) ──
// test_task_accel_core.cpp (S2a) and test_task_vel_core.cpp (S3a) both drive a
// shim against a live adapter and compare every lane BIT FOR BIT, and S3a reuses
// S2a's independent-handle measurement instead of repeating it (plan §S3 R6).
// That reuse is only valid while both files use the SAME fixture and the SAME
// handle path — so the fixture lives here once, rather than as a byte-identical
// copy in each file that a later edit could silently split. A divergence would
// not fail loudly; it would quietly invalidate the premise one file inherits
// from the other.
//
// serial_7dof.urdf: alternating Z/Y joint axes, so g(q) ≢ 0 and the Jacobian is
// well-conditioned. serial_6dof.urdf is all-Z — g ≡ 0 and σ_min ≡ 0, the fixture
// trap recorded in the plan's §제약·함정. nv = 7 > 6 also leaves a non-trivial
// null space, which is what makes the dynamically-consistent null-space branch
// (S2a) and the null-space secondary task (S3a) meaningful rather than inert.
//
// Test-only, NEVER installed: same never-installed, source-tree-path idiom as
// bit_compare.hpp — ament's symlink install ignores install(PATTERN EXCLUDE), so
// a header placed under include/ would ship.
#pragma once

#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"
#include "test_urdf_path.hpp"

#include <cstddef>
#include <memory>
#include <string>

namespace rtc::testing {

/// @brief Absolute path to the shared serial_7dof URDF fixture.
/// @return path resolved through the rtc_urdf_bridge test resolver.
[[nodiscard]] inline std::string Serial7dof() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

/// @brief Build an RtModelHandle over the serial_7dof fixture.
///
/// The extraction shims call this a second time to get a handle independent of
/// the adapter's, which is only a valid bitwise oracle because two independently
/// built handles agree bit for bit — measured by
/// ReferenceDynamics.IndependentHandlesAgreeBitwise (dynamics half,
/// test_task_accel_core.cpp) and
/// ReferenceHandles.IndependentHandlesAgreeOnTheReorderPathBitwise (reorder half,
/// test_task_vel_core.cpp).
///
/// @param model_out receives the full Pinocchio model the handle was built from.
/// @return owning handle; never null (the builder throws on a bad URDF).
[[nodiscard]] inline std::unique_ptr<rtc_urdf_bridge::RtModelHandle> MakeHandle(
    std::shared_ptr<const pinocchio::Model>& model_out) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = Serial7dof();
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  model_out = builder.GetFullModel();
  return std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_out);
}

/// @brief A single-device ControllerState with device 0 valid and `nc0` channels.
/// @param nc0 channel count for device 0.
/// @param dt  tick period [s].
/// @return zero-initialised state ready for FillSweep.
[[nodiscard]] inline rtc::ControllerState MakeState(int nc0, double dt) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nc0;
  state.devices[0].valid = true;
  return state;
}

/// @brief The measured-state sweep both sides of a cross-check are driven with.
///
/// Slow enough that the adapter's output clamp stays inert — which the suites
/// ASSERT per channel rather than assume, because a fired clamp voids a bitwise
/// comparison instead of failing it informatively — but moving, so the Jacobian,
/// the Coriolis term (S2a), the null-space posture error (S3a) and the finite
/// pose error are all non-trivial. Deterministic in `tick`, so a goal-derivation
/// helper can ask "what is the TCP pose at tick N" without duplicating the
/// formula.
///
/// @param state state whose device-0 lanes are overwritten.
/// @param nv    joint count to fill.
/// @param tick  tick index (the sweep's only time input).
/// @param dt    tick period [s] — velocities are the position step over dt.
inline void FillSweep(rtc::ControllerState& state, int nv, int tick, double dt) {
  constexpr double kStep = 0.0004;
  for (int j = 0; j < nv; ++j) {
    const auto uj = static_cast<std::size_t>(j);
    state.devices[0].positions[uj] = 0.12 * (1.0 + 0.2 * j) + kStep * tick * (1.0 + 0.3 * j);
    state.devices[0].velocities[uj] = kStep * (1.0 + 0.3 * j) / dt;
  }
}

}  // namespace rtc::testing
