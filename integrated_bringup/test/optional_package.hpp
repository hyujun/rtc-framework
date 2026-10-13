// ── optional_package.hpp — "is this package installed?" guard for test fixtures ──
//
// A fixture here may resolve a robot model from a package that is deliberately
// NOT a dependency of this workspace — `hand_description`, a separate project
// (no package.xml dep, no find_package, not in deps.repos) that holds the G1
// model. A clean checkout cannot acquire it.
//
// Left unguarded, that absence is a throw in every test that reaches the
// fixture. The rule (repo_scripts/scripts/validate_test_fixtures.py, #454): a
// test may resolve such a package only if it SKIPS when the package is absent.
// A skip is an honest report of the gap, not a repair — the path the test
// covers is unverified wherever it skips.
//
// ament_index has no non-throwing query, so the only way to ask is to call and
// catch. The catch is deliberately NARROW — PackageNotFoundError alone — so a
// genuine ament_index fault still surfaces as a failure instead of a silent skip.
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
                   << "dependency of this workspace (a separate project). What "    \
                   << "this case covers is UNVERIFIED here.";                       \
    }                                                                               \
  } while (false)
