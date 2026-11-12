#ifndef RTC_MPC_REFERENCE_SURFACE_FRAME_HPP_
#define RTC_MPC_REFERENCE_SURFACE_FRAME_HPP_

/// @file surface_frame.hpp
/// @brief The support surface a foot lands on: a plane in world coordinates.
///
/// Today the surface is flat (a value, @ref FlatSurface) and the one reader
/// is the stage reference builder, which asks @ref HeightAt for the terrain
/// height under each contact's measured position. A terrain estimator would
/// write a tilted plane here; the swing planner then also needs the lift-off
/// and touch-down heights told apart (it takes one height per swing today) —
/// that is the seam's second step, not a second implementation.
///
/// Everything but the loader is constexpr / noexcept and heap-free.
/// @ref HeightAt expects finite `x · y` (the builder checks its snapshot first).

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>

namespace rtc::mpc {

/// @brief A plane through `point` with unit normal `normal` (world axes, SI).
struct SurfaceFrame {
  std::array<double, 3> point{0.0, 0.0, 0.0};
  std::array<double, 3> normal{0.0, 0.0, 1.0};
};

/// @brief Smallest accepted `normal[2]`: @ref HeightAt divides by it (NUM-2).
inline constexpr double kMinSurfaceNormalZ = 1e-9;

/// @brief Largest accepted `|‖normal‖² − 1|`.
inline constexpr double kSurfaceNormalTolerance = 1e-9;

/// @brief Failure modes of @ref ValidateSurfaceFrame and @ref LoadTerrainConfig.
enum class SurfaceFrameError {
  kNoError = 0,
  kNonFinite,         ///< a NaN / ±inf component (or YAML value)
  kNotUnitNormal,     ///< `|‖normal‖² − 1| > kSurfaceNormalTolerance`
  kDegenerateNormal,  ///< `normal[2] ≤ kMinSurfaceNormalZ` (a wall, or facing down)
  kNotAMap,           ///< the `terrain:` section is absent or not a map
  kUnknownKey,        ///< a key outside the schema (a misspelt key must not fall back)
  kMissingKey,        ///< a schema key without a value
  kWrongType,         ///< a value of the wrong kind
};

/// @brief The horizontal plane `z = height`.
[[nodiscard]] constexpr SurfaceFrame FlatSurface(double height) noexcept {
  return SurfaceFrame{{0.0, 0.0, height}, {0.0, 0.0, 1.0}};
}

[[nodiscard]] inline SurfaceFrameError ValidateSurfaceFrame(const SurfaceFrame& s) noexcept {
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(s.point[i]) || !std::isfinite(s.normal[i])) {
      return SurfaceFrameError::kNonFinite;
    }
  }
  const double nn =
      s.normal[0] * s.normal[0] + s.normal[1] * s.normal[1] + s.normal[2] * s.normal[2];
  if (std::fabs(nn - 1.0) > kSurfaceNormalTolerance) {
    return SurfaceFrameError::kNotUnitNormal;
  }
  if (s.normal[2] <= kMinSurfaceNormalZ) {
    return SurfaceFrameError::kDegenerateNormal;
  }
  return SurfaceFrameError::kNoError;
}

/// @brief The surface height at world `(x, y)`: the plane solved for z,
///        `p_z − (n_x (x − p_x) + n_y (y − p_y)) / n_z`. On a flat surface this
///        is `p_z` exactly. `s` must have passed @ref ValidateSurfaceFrame.
[[nodiscard]] constexpr double HeightAt(const SurfaceFrame& s, double x, double y) noexcept {
  return s.point[2] -
         (s.normal[0] * (x - s.point[0]) + s.normal[1] * (y - s.point[1])) / s.normal[2];
}

/// @brief Parse the `terrain:` map (`height: 0.0  # [m] finite`) into the
///        flat surface `z = height`. `out` is written only on `kNoError`,
///        which also implies @ref ValidateSurfaceFrame passed.
[[nodiscard]] SurfaceFrameError LoadTerrainConfig(const YAML::Node& node,
                                                  SurfaceFrame& out) noexcept;

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_SURFACE_FRAME_HPP_
