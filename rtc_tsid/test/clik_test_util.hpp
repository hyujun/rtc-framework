// Small helpers the CLIK suites share: bit-level comparison (the suites pin
// outputs bit for bit), a contact-free PinocchioCache, and a number format for
// test properties.

#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ios>
#include <memory>
#include <string>

namespace rtc::tsid::test {

/// The IEEE-754 bits of `x`.
[[nodiscard]] inline std::uint64_t Bits(double x) {
  std::uint64_t b = 0;
  std::memcpy(&b, &x, sizeof(b));
  return b;
}

/// Equal sizes and equal bits in every entry (NaN payloads and the sign of
/// zero included).
[[nodiscard]] inline ::testing::AssertionResult BitEqual(const Eigen::VectorXd& a,
                                                         const Eigen::VectorXd& b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "sizes " << a.size() << " vs " << b.size();
  }
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    if (Bits(a(i)) != Bits(b(i))) {
      return ::testing::AssertionFailure()
             << "index " << i << ": " << std::hexfloat << a(i) << " vs " << b(i);
    }
  }
  return ::testing::AssertionSuccess();
}

/// A double as a test property: std::to_string keeps six decimals, which
/// prints a residual of 1e-7 as 0.000000.
[[nodiscard]] inline std::string Sci(double x) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.3e", x);
  return buf.data();
}

/// `cache` initialised on `model` with no contact frames.
inline void InitContactFreeCache(PinocchioCache& cache,
                                 const std::shared_ptr<const pinocchio::Model>& model) {
  ContactManagerConfig contact_cfg;
  contact_cfg.max_contacts = 0;
  cache.Init(model, rtc::tsid::ContactFrameIds(contact_cfg));
}

}  // namespace rtc::tsid::test
