// ── imu_rate_export.cpp ──────────────────────────────────────────────────────
#include "rtc_mujoco_sim/imu_rate_export.hpp"

#include <cmath>
#include <cstddef>

namespace rtc {

namespace {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major

[[nodiscard]] Vec3 Cross(const Vec3& a, const Vec3& b) noexcept {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

/// Aᵀ B.
[[nodiscard]] Mat3 TransposeTimes(const Mat3& a, const Mat3& b) noexcept {
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      out[i * 3 + j] = a[i] * b[j] + a[3 + i] * b[3 + j] + a[6 + i] * b[6 + j];
    }
  }
  return out;
}

/// Aᵀ v.
[[nodiscard]] Vec3 TransposeTimes(const Mat3& a, const Vec3& v) noexcept {
  return {a[0] * v[0] + a[3] * v[1] + a[6] * v[2], a[1] * v[0] + a[4] * v[1] + a[7] * v[2],
          a[2] * v[0] + a[5] * v[1] + a[8] * v[2]};
}

[[nodiscard]] bool ValidInterval(const ImuSiteSnapshot& prev, const ImuSiteSnapshot& cur,
                                 double& dt) noexcept {
  dt = cur.time - prev.time;
  return std::isfinite(dt) && dt > 0.0;
}

}  // namespace

std::optional<ImuRateExport> ParseImuRateExport(std::string_view text) noexcept {
  if (text == "sample")
    return ImuRateExport::kSample;
  if (text == "tick_mean")
    return ImuRateExport::kTickMean;
  return std::nullopt;
}

const char* ToString(ImuRateExport mode) noexcept {
  return mode == ImuRateExport::kTickMean ? "tick_mean" : "sample";
}

Vec3 RotationLog(const Mat3& r) noexcept {
  // Shepperd's quaternion, then the angle from atan2 (accurate at small and
  // at large angles, unlike acos of the trace).
  const double tr = r[0] + r[4] + r[8];
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (tr > 0.0) {
    const double s = 2.0 * std::sqrt(tr + 1.0);
    w = 0.25 * s;
    x = (r[7] - r[5]) / s;
    y = (r[2] - r[6]) / s;
    z = (r[3] - r[1]) / s;
  } else if (r[0] > r[4] && r[0] > r[8]) {
    const double s = 2.0 * std::sqrt(1.0 + r[0] - r[4] - r[8]);
    w = (r[7] - r[5]) / s;
    x = 0.25 * s;
    y = (r[1] + r[3]) / s;
    z = (r[2] + r[6]) / s;
  } else if (r[4] > r[8]) {
    const double s = 2.0 * std::sqrt(1.0 + r[4] - r[0] - r[8]);
    w = (r[2] - r[6]) / s;
    x = (r[1] + r[3]) / s;
    y = 0.25 * s;
    z = (r[5] + r[7]) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + r[8] - r[0] - r[4]);
    w = (r[3] - r[1]) / s;
    x = (r[2] + r[6]) / s;
    y = (r[5] + r[7]) / s;
    z = 0.25 * s;
  }
  if (w < 0.0) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }
  const double n = std::sqrt(x * x + y * y + z * z);
  if (n == 0.0)
    return {0.0, 0.0, 0.0};
  const double k = 2.0 * std::atan2(n, w) / n;
  return {k * x, k * y, k * z};
}

bool TickMeanGyro(const ImuSiteSnapshot& prev, const ImuSiteSnapshot& cur, double* out) noexcept {
  double dt = 0.0;
  if (out == nullptr || !ValidInterval(prev, cur, dt))
    return false;
  const Vec3 phi = RotationLog(TransposeTimes(prev.rot, cur.rot));
  for (std::size_t a = 0; a < 3; ++a)
    out[a] = phi[a] / dt;
  return true;
}

bool TickMeanAccel(const ImuSiteSnapshot& prev, const ImuSiteSnapshot& cur, const double* gravity,
                   double* out) noexcept {
  double dt = 0.0;
  if (out == nullptr || gravity == nullptr || !ValidInterval(prev, cur, dt))
    return false;
  Vec3 a_world{};
  for (std::size_t a = 0; a < 3; ++a)
    a_world[a] = (cur.vel[a] - prev.vel[a]) / dt - gravity[a];
  const Vec3 b = TransposeTimes(prev.rot, a_world);
  const Vec3 phi = RotationLog(TransposeTimes(prev.rot, cur.rot));
  // Γ₁(φ)⁻¹ = I − ½ φ^ + c₂ φ^², c₂ = (1 − (θ/2) cot(θ/2)) / θ². The closed
  // form cancels at small θ; its series is used there (the next term,
  // θ⁶/1209600, is below rounding at the switch).
  const double th2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
  const double th = std::sqrt(th2);
  const double c2 = th < 1e-2 ? 1.0 / 12.0 + th2 / 720.0 + th2 * th2 / 30240.0
                              : (1.0 - 0.5 * th / std::tan(0.5 * th)) / th2;
  const Vec3 pb = Cross(phi, b);
  const Vec3 ppb = Cross(phi, pb);
  for (std::size_t a = 0; a < 3; ++a)
    out[a] = b[a] - 0.5 * pb[a] + c2 * ppb[a];
  return true;
}

}  // namespace rtc
