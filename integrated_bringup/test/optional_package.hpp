// ── optional_package.hpp — "is this package installed?" guard for test fixtures ──
//
// A few fixtures here resolve a robot model from `hand_description`, which is
// deliberately NOT a dependency of this workspace: it is a separate project that
// only exists on developer machines (no package.xml dep, no find_package, not in
// deps.repos, not in .github/ci-packages.yml). CI runners cannot acquire it.
//
// Left unguarded, that absence is an *invisible* red: `integrated_bringup` runs
// in the `test_cpp_besteffort` (continue-on-error) lane, so the throw never turns
// a PR red and the only symptom is a codecov patch % that reads as "no tests were
// written" — which is exactly how #452's joint-order oracle went unrun outside one
// machine for its whole life (#454).
//
// ament_index has no non-throwing query, so the only way to ask is to call and
// catch. The catch is deliberately NARROW — PackageNotFoundError alone — so a
// genuine ament_index fault still surfaces as a failure instead of a silent skip.
//
// A skip is an honest report of the gap, not a repair: the closed-chain paths
// these fixtures cover still do not run in CI. #457 closes that by checking a
// real-scale closed-chain model into `robot_descriptions`; when it lands, these
// guards go away with the `hand_description` references.
#pragma once

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <string>

namespace integrated_bringup::testfx {

// Share directory of `pkg`, or nullopt when it is not installed.
inline std::optional<std::string> FindPackageShare(const std::string& pkg) {
  try {
    return ament_index_cpp::get_package_share_directory(pkg);
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    return std::nullopt;
  }
}

inline bool HasPackage(const std::string& pkg) {
  return FindPackageShare(pkg).has_value();
}

}  // namespace integrated_bringup::testfx

// Skip the current test when `pkg` is absent. Use this in every test that
// resolves a package outside this workspace — validate_test_fixtures.py blocks
// an unguarded `get_package_share_directory` on such a package.
#define RTC_SKIP_IF_PACKAGE_MISSING(pkg)                                            \
  do {                                                                              \
    if (!::integrated_bringup::testfx::HasPackage(pkg)) {                           \
      GTEST_SKIP() << "package '" << (pkg) << "' is not installed — it is not a " \
                   << "dependency of this workspace (separate project). The "       \
                   << "closed-chain path this case covers is UNVERIFIED here; "     \
                   << "#457 puts a closed-chain model in robot_descriptions.";      \
    }                                                                               \
  } while (false)
