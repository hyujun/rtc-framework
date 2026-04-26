#ifndef RTC_BASE_UTILS_CLAMP_COMMANDS_HPP_
#define RTC_BASE_UTILS_CLAMP_COMMANDS_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

#include "rtc_base/types/types.hpp"

namespace rtc::utils {

// Symmetric clamp: each command i is clamped to [-limit_i, +limit_i] using
// `limits[i]` when within range, otherwise `default_limit`. RT-safe (no
// allocation, no exceptions).
inline void ClampSymmetric(std::array<double, kMaxDeviceChannels> &commands,
                           int n, std::span<const double> limits,
                           double default_limit) noexcept {
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lim = (i < limits.size()) ? limits[i] : default_limit;
    commands[i] = std::clamp(commands[i], -lim, lim);
  }
}

// Asymmetric clamp: each command i is clamped to [lower_i, upper_i] using
// `lower[i]` / `upper[i]` when within range, otherwise the supplied defaults.
// RT-safe.
inline void ClampRange(std::array<double, kMaxDeviceChannels> &commands, int n,
                       std::span<const double> lower,
                       std::span<const double> upper, double default_lower,
                       double default_upper) noexcept {
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lo = (i < lower.size()) ? lower[i] : default_lower;
    const double hi = (i < upper.size()) ? upper[i] : default_upper;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

} // namespace rtc::utils

#endif // RTC_BASE_UTILS_CLAMP_COMMANDS_HPP_
