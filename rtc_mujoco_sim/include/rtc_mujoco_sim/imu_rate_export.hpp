// ── imu_rate_export.hpp ──────────────────────────────────────────────────────
// What the simulator's gyro and accelerometer export carries for one step
// (`robot_response.<group>.imu_rate_export`).
//
// A MuJoCo gyro or accelerometer reads one instant — the forward pass at the
// start of the step's last substep — and its accelerometer is the explicit
// qacc of that pass. Under an implicit integrator that qacc is not the
// velocity change the step goes on to realise, so integrating the exported
// specific force does not reproduce the motion, by far more than any IMU
// noise. A real IMU's sample is the mean, over its sample period, of the
// motion that happened.
//
// "tick_mean" builds that mean from two snapshots of the IMU site taken one
// step apart — its orientation R (world ← site) and the world linear velocity
// v of its origin — assuming the body rate ω and the site-frame specific
// force f are constant over the interval:
//
//   φ = Log(R_prevᵀ R)                        gyro  = φ / Δt
//   R(t) = R_prev Exp(ω t)  ⇒  Δv = R_prev Γ₁(φ) f Δt + g Δt
//                                              accel = Γ₁(φ)⁻¹ R_prevᵀ (Δv/Δt − g)
//
// with Γ₁ the left Jacobian of SO(3). That is the exact inverse of a
// strapdown propagation v⁺ = v + R Γ₁(ωΔt) f Δt + g Δt, so a noise-free
// export propagates back to the recorded velocities. Orientation is a state,
// not a rate: nothing here differences or averages it.
//
// Free of physics: snapshots in, values out, so the identities are testable
// without stepping a model.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_IMU_RATE_EXPORT_HPP_
#define RTC_MUJOCO_SIM_IMU_RATE_EXPORT_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace rtc {

/// `robot_response.<group>.imu_rate_export`.
enum class ImuRateExport : std::uint8_t {
  kSample,   ///< "sample": mjData::sensordata as MuJoCo computes it (the default)
  kTickMean  ///< "tick_mean": the mean over the step of the site's motion
};

/// @brief Parse the YAML spelling of an ImuRateExport.
/// @param text "sample" or "tick_mean" (exact, case-sensitive).
/// @return The mode, or std::nullopt for anything else.
[[nodiscard]] std::optional<ImuRateExport> ParseImuRateExport(std::string_view text) noexcept;

/// @brief The YAML spelling of `mode` ("sample" / "tick_mean").
[[nodiscard]] const char* ToString(ImuRateExport mode) noexcept;

/// One IMU site at one instant.
struct ImuSiteSnapshot {
  std::array<double, 9> rot{1.0, 0.0, 0.0, 0.0, 1.0,
                            0.0, 0.0, 0.0, 1.0};  ///< world ← site, row-major
  std::array<double, 3> vel{};  ///< world-frame linear velocity of the site origin [m/s]
  double time{0.0};             ///< sim time [s]
};

/// @brief The rotation vector of a rotation matrix.
/// @param rot Row-major rotation matrix.
/// @return φ = Log(R) [rad], |φ| ≤ π.
[[nodiscard]] std::array<double, 3> RotationLog(const std::array<double, 9>& rot) noexcept;

/// @brief Mean body rate over [prev, cur]: φ/Δt with φ = Log(R_prevᵀ R).
/// @param prev The earlier snapshot.
/// @param cur  The later snapshot.
/// @param out  Receives the rate [rad/s] in the site frame (3 values).
/// @return false (and `out` untouched) when Δt = cur.time − prev.time is not
///         positive and finite, or `out` is null.
[[nodiscard]] bool TickMeanGyro(const ImuSiteSnapshot& prev, const ImuSiteSnapshot& cur,
                                double* out) noexcept;

/// @brief Mean specific force over [prev, cur]: Γ₁(φ)⁻¹ R_prevᵀ (Δv/Δt − g).
/// @param prev    The earlier snapshot.
/// @param cur     The later snapshot.
/// @param gravity World gravity vector [m/s²] (mjOption::gravity, 3 values).
/// @param out     Receives the specific force [m/s²] in the site frame (3 values).
/// @return false (and `out` untouched) when Δt is not positive and finite,
///         or a pointer is null.
[[nodiscard]] bool TickMeanAccel(const ImuSiteSnapshot& prev, const ImuSiteSnapshot& cur,
                                 const double* gravity, double* out) noexcept;

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_IMU_RATE_EXPORT_HPP_
