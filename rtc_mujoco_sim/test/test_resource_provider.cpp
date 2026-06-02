// ── test_resource_provider.cpp ───────────────────────────────────────────────
// Pure-string + ament tests for ResolveModelPath ("package://" URI resolution).
// No MuJoCo runtime, no MJCF load.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/ros2_resource_provider.hpp"

#include <gtest/gtest.h>

#include <string>

namespace rtc {
namespace {

// Non-"package://" paths are returned verbatim — the resolver is a no-op for
// plain filesystem paths.
TEST(ResolveModelPath, PlainPathPassThrough) {
  const std::string plain = "/abs/path/to/model.xml";
  EXPECT_EQ(ResolveModelPath(plain), plain);
  EXPECT_EQ(ResolveModelPath("relative/model.xml"), "relative/model.xml");
}

// Empty input is not a "package://" URI, so it passes through unchanged.
TEST(ResolveModelPath, EmptyInputPassThrough) {
  EXPECT_EQ(ResolveModelPath(""), "");
}

// A well-formed URI naming a non-existent package resolves to "" (the
// PackageNotFoundError path), NOT the original string.
TEST(ResolveModelPath, UnknownPackageResolvesEmpty) {
  EXPECT_EQ(ResolveModelPath("package://__no_such_pkg__/config/scene.xml"), "");
}

// "package://<pkg>/<rel>" → "<share_dir>/<rel>". rtc_mujoco_sim installs a
// config/ directory, so the resolved path is non-empty, references the package,
// and ends with the relative segment.
TEST(ResolveModelPath, KnownPackageWithRelPath) {
  const std::string resolved = ResolveModelPath("package://rtc_mujoco_sim/config");
  EXPECT_FALSE(resolved.empty());
  EXPECT_NE(resolved.find("rtc_mujoco_sim"), std::string::npos);
  EXPECT_TRUE(resolved.ends_with("/config"));
}

// "package://<pkg>" with no relative path → the bare share directory.
TEST(ResolveModelPath, KnownPackageNoRelPath) {
  const std::string resolved = ResolveModelPath("package://rtc_mujoco_sim");
  EXPECT_FALSE(resolved.empty());
  EXPECT_TRUE(resolved.ends_with("rtc_mujoco_sim"));
}

}  // namespace
}  // namespace rtc
