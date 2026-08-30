// ── test_object_pool_sampling.cpp ─────────────────────────────────────────────
// Pure helpers of the object pool: directory scan, allowlist resolution,
// orientation conversion, pose sampling, object selection.
//
// No MJCF fixture and no mjModel — everything here is testable with the
// simulator uninitialised, which is what keeps these assertions fast enough to
// be worth running on every edit.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/object_pool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>
#include <string>
#include <vector>

#ifndef OBJECT_POOL_SCAN_FIXTURE_DIR
#error "OBJECT_POOL_SCAN_FIXTURE_DIR must be defined by CMake"
#endif

namespace rtc {
namespace {

constexpr double kTight = 1e-12;

// ── Orientation ──────────────────────────────────────────────────────────────

/// R = Rz(yaw) * Ry(pitch) * Rx(roll), written out from first principles.
///
/// Deliberately NOT expressed via any MuJoCo or rtc_math helper: this is the
/// independent definition that RpyZyxToQuat is checked against. Building it
/// from the same library it is meant to validate would make the test agree with
/// whatever the implementation does.
void ReferenceZyx(double roll, double pitch, double yaw, double out[9]) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double rx[9] = {1, 0, 0, 0, cr, -sr, 0, sr, cr};
  const double ry[9] = {cp, 0, sp, 0, 1, 0, -sp, 0, cp};
  const double rz[9] = {cy, -sy, 0, sy, cy, 0, 0, 0, 1};
  double tmp[9];
  mju_mulMatMat(tmp, rz, ry, 3, 3, 3);
  mju_mulMatMat(out, tmp, rx, 3, 3, 3);
}

double MaxAbsDiff(const double a[9], const double b[9]) {
  double d = 0.0;
  for (int i = 0; i < 9; ++i) {
    d = std::max(d, std::fabs(a[i] - b[i]));
  }
  return d;
}

// The angle triple is asymmetric on purpose, and no two components share a
// magnitude. A symmetric triple (or any pair of equal angles) is satisfied by
// several wrong conventions at once: with roll==yaw, "zyx" and "XYZ" agree, and
// a roll/yaw transposition is invisible. This triple separates them — the
// lowercase "zyx" spelling one would naturally reach for is ~1 rad off here and
// still yields a perfectly plausible rotation matrix, so nothing but a
// numerical comparison catches it.
constexpr double kRoll = 0.3;
constexpr double kPitch = -0.7;
constexpr double kYaw = 1.9;

TEST(ObjectPoolOrientation, RpyZyxMatchesRzRyRxOnAsymmetricTriple) {
  const std::array<double, 4> q = RpyZyxToQuat({kRoll, kPitch, kYaw});
  double actual[9];
  mju_quat2Mat(actual, q.data());

  double expected[9];
  ReferenceZyx(kRoll, kPitch, kYaw, expected);

  EXPECT_LT(MaxAbsDiff(actual, expected), kTight)
      << "RpyZyxToQuat does not implement R = Rz(yaw)*Ry(pitch)*Rx(roll)";
}

TEST(ObjectPoolOrientation, RollPitchYawAreNotInterchangeable) {
  // Guards the argument ORDER, which the convention test above cannot: a
  // implementation that read (yaw, pitch, roll) would still be some valid ZYX
  // rotation. Permuting the inputs must change the answer.
  const std::array<double, 4> q = RpyZyxToQuat({kRoll, kPitch, kYaw});
  const std::array<double, 4> swapped = RpyZyxToQuat({kYaw, kPitch, kRoll});
  double a[9], b[9];
  mju_quat2Mat(a, q.data());
  mju_quat2Mat(b, swapped.data());
  EXPECT_GT(MaxAbsDiff(a, b), 0.1) << "roll and yaw appear to be interchangeable";
}

TEST(ObjectPoolOrientation, ZeroRpyIsIdentityQuaternion) {
  const std::array<double, 4> q = RpyZyxToQuat({0.0, 0.0, 0.0});
  EXPECT_NEAR(q[0], 1.0, kTight);
  EXPECT_NEAR(q[1], 0.0, kTight);
  EXPECT_NEAR(q[2], 0.0, kTight);
  EXPECT_NEAR(q[3], 0.0, kTight);
}

TEST(ObjectPoolOrientation, QuaternionIsUnitNorm) {
  const std::array<double, 4> q = RpyZyxToQuat({kRoll, kPitch, kYaw});
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  EXPECT_NEAR(n, 1.0, 1e-15);
}

// ── Pose sampling ────────────────────────────────────────────────────────────

ObjectPoolConfig MakePoseConfig() {
  ObjectPoolConfig cfg;
  cfg.enabled = true;
  cfg.position = {0.70, 0.00, 0.10};
  cfg.position_variation = {0.10, 0.20, 0.00};
  cfg.rpy = {0.0, 0.0, 0.0};
  cfg.rpy_variation = {0.0, 0.0, 3.0};
  return cfg;
}

TEST(ObjectPoolPose, FixedModeReturnsTheConfiguredCentreExactly) {
  ObjectPoolConfig cfg = MakePoseConfig();
  cfg.pose = PoseSampling::kFixed;
  cfg.rpy = {kRoll, kPitch, kYaw};
  std::mt19937_64 rng(12345);

  for (int i = 0; i < 16; ++i) {
    const SampledPose p = SampleObjectPose(cfg, rng);
    EXPECT_DOUBLE_EQ(p.position[0], cfg.position[0]);
    EXPECT_DOUBLE_EQ(p.position[1], cfg.position[1]);
    EXPECT_DOUBLE_EQ(p.position[2], cfg.position[2]);
    double actual[9], expected[9];
    mju_quat2Mat(actual, p.quat.data());
    ReferenceZyx(kRoll, kPitch, kYaw, expected);
    EXPECT_LT(MaxAbsDiff(actual, expected), kTight);
  }
}

TEST(ObjectPoolPose, RandomModeStaysInsideRangeAndCoversIt) {
  ObjectPoolConfig cfg = MakePoseConfig();
  cfg.pose = PoseSampling::kRandom;
  std::mt19937_64 rng(2026);

  constexpr int kSamples = 4000;
  std::array<double, 3> lo{1e9, 1e9, 1e9};
  std::array<double, 3> hi{-1e9, -1e9, -1e9};

  for (int i = 0; i < kSamples; ++i) {
    const SampledPose p = SampleObjectPose(cfg, rng);
    for (std::size_t a = 0; a < 3; ++a) {
      const double lo_bound = cfg.position[a] - cfg.position_variation[a];
      const double hi_bound = cfg.position[a] + cfg.position_variation[a];
      // Containment alone is a weak oracle: it passes for an implementation
      // that returns the centre every time.
      EXPECT_GE(p.position[a], lo_bound);
      EXPECT_LE(p.position[a], hi_bound);
      lo[a] = std::min(lo[a], p.position[a]);
      hi[a] = std::max(hi[a], p.position[a]);
    }
  }

  // ...so also require the observed spread to cover most of the declared
  // range. This is the assertion that fails when sampling silently degenerates.
  for (std::size_t a = 0; a < 2; ++a) {  // axes 0,1 have non-zero variation
    const double declared = 2.0 * cfg.position_variation[a];
    const double observed = hi[a] - lo[a];
    EXPECT_GT(observed, 0.9 * declared)
        << "axis " << a << " covered only " << observed << " of " << declared;
  }
  // Axis 2 has zero variation and must therefore be pinned to the centre.
  EXPECT_DOUBLE_EQ(lo[2], cfg.position[2]);
  EXPECT_DOUBLE_EQ(hi[2], cfg.position[2]);
}

TEST(ObjectPoolPose, RandomYawActuallyRotates) {
  ObjectPoolConfig cfg = MakePoseConfig();
  cfg.pose = PoseSampling::kRandom;
  std::mt19937_64 rng(7);

  double min_w = 2.0;
  for (int i = 0; i < 2000; ++i) {
    const SampledPose p = SampleObjectPose(cfg, rng);
    min_w = std::min(min_w, std::fabs(p.quat[0]));
  }
  // yaw variation of 3 rad means |w| = |cos(yaw/2)| must reach well below 1.
  EXPECT_LT(min_w, 0.2) << "orientation never left the neighbourhood of identity";
}

TEST(ObjectPoolPose, SameSeedReproducesTheSequence) {
  ObjectPoolConfig cfg = MakePoseConfig();
  cfg.pose = PoseSampling::kRandom;

  std::mt19937_64 a(99), b(99), c(100);
  bool differs_across_seeds = false;
  for (int i = 0; i < 32; ++i) {
    const SampledPose pa = SampleObjectPose(cfg, a);
    const SampledPose pb = SampleObjectPose(cfg, b);
    const SampledPose pc = SampleObjectPose(cfg, c);
    for (std::size_t k = 0; k < 3; ++k) {
      EXPECT_DOUBLE_EQ(pa.position[k], pb.position[k]);
      if (pa.position[k] != pc.position[k]) {
        differs_across_seeds = true;
      }
    }
  }
  EXPECT_TRUE(differs_across_seeds) << "a different seed produced an identical stream";
}

TEST(ObjectPoolPose, ZeroWidthAxisStillConsumesItsDraw) {
  // The implementation draws for every axis, including zero-width ones, so the
  // RNG stream position does not depend on WHICH axes vary. Without that,
  // widening one axis would silently shift the values sampled for the others,
  // and a "reproducible" seeded run would stop being comparable across configs.
  ObjectPoolConfig wide = MakePoseConfig();
  wide.pose = PoseSampling::kRandom;
  ObjectPoolConfig narrow = wide;
  narrow.position_variation[0] = 0.0;  // only axis 0 differs

  std::mt19937_64 ra(4242), rb(4242);
  for (int i = 0; i < 16; ++i) {
    const SampledPose pa = SampleObjectPose(wide, ra);
    const SampledPose pb = SampleObjectPose(narrow, rb);
    // Axis 1 draws the same underlying random number in both configs.
    EXPECT_DOUBLE_EQ(pa.position[1], pb.position[1]);
    EXPECT_DOUBLE_EQ(pa.quat[0], pb.quat[0]);
  }
}

// ── Object selection ─────────────────────────────────────────────────────────

TEST(ObjectPoolSelection, FixedModeAlwaysReturnsTheFixedIndex) {
  std::mt19937_64 rng(1);
  for (int i = 0; i < 32; ++i) {
    EXPECT_EQ(SelectObjectIndex(ObjectSelection::kFixed, 5, 3, 2, true, rng), 2u);
  }
}

TEST(ObjectPoolSelection, FixedIndexIsClampedIntoRange) {
  std::mt19937_64 rng(1);
  EXPECT_EQ(SelectObjectIndex(ObjectSelection::kFixed, 3, 0, 99, true, rng), 2u);
}

TEST(ObjectPoolSelection, RandomModeCoversEveryCandidate) {
  std::mt19937_64 rng(31337);
  constexpr std::size_t kCount = 6;
  std::set<std::size_t> seen;
  std::size_t current = kCount;  // "none active"
  for (int i = 0; i < 600; ++i) {
    current = SelectObjectIndex(ObjectSelection::kRandom, kCount, current, 0, false, rng);
    ASSERT_LT(current, kCount);
    seen.insert(current);
  }
  EXPECT_EQ(seen.size(), kCount) << "random selection never reached some candidates";
}

TEST(ObjectPoolSelection, AvoidRepeatNeverPicksTheCurrentIndex) {
  std::mt19937_64 rng(555);
  constexpr std::size_t kCount = 4;
  std::size_t current = 0;
  std::set<std::size_t> seen;
  for (int i = 0; i < 400; ++i) {
    const std::size_t next =
        SelectObjectIndex(ObjectSelection::kRandom, kCount, current, 0, true, rng);
    ASSERT_LT(next, kCount);
    EXPECT_NE(next, current) << "avoid_repeat returned the active object";
    seen.insert(next);
    current = next;
  }
  // The shift-past-current trick must not make any index unreachable.
  EXPECT_EQ(seen.size(), kCount);
}

TEST(ObjectPoolSelection, AvoidRepeatDegradesGracefullyWithOneCandidate) {
  // Must return the only candidate rather than spinning looking for a
  // different one.
  std::mt19937_64 rng(9);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(SelectObjectIndex(ObjectSelection::kRandom, 1, 0, 0, true, rng), 0u);
  }
}

// ── Enum parsing ─────────────────────────────────────────────────────────────

TEST(ObjectPoolParse, KnownSpellingsParse) {
  ObjectSelection sel = ObjectSelection::kRandom;
  EXPECT_TRUE(ParseObjectSelection("fixed", sel));
  EXPECT_EQ(sel, ObjectSelection::kFixed);
  EXPECT_TRUE(ParseObjectSelection("random", sel));
  EXPECT_EQ(sel, ObjectSelection::kRandom);

  PoseSampling pose = PoseSampling::kRandom;
  EXPECT_TRUE(ParsePoseSampling("fixed", pose));
  EXPECT_EQ(pose, PoseSampling::kFixed);
  EXPECT_TRUE(ParsePoseSampling("random", pose));
  EXPECT_EQ(pose, PoseSampling::kRandom);
}

TEST(ObjectPoolParse, UnknownSpellingFailsInsteadOfDefaulting) {
  // A typo that silently means "fixed" is a randomisation harness reporting
  // green while randomising nothing — the failure this repo calls false green.
  ObjectSelection sel = ObjectSelection::kRandom;
  EXPECT_FALSE(ParseObjectSelection("Random", sel));
  EXPECT_FALSE(ParseObjectSelection("rand", sel));
  EXPECT_FALSE(ParseObjectSelection("", sel));
  EXPECT_EQ(sel, ObjectSelection::kRandom) << "out param was clobbered on failure";

  PoseSampling pose = PoseSampling::kFixed;
  EXPECT_FALSE(ParsePoseSampling("uniform", pose));
  EXPECT_EQ(pose, PoseSampling::kFixed);
}

// ── Directory scan ───────────────────────────────────────────────────────────

class ObjectPoolScan : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::path(OBJECT_POOL_SCAN_FIXTURE_DIR);
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
    std::filesystem::create_directories(root_, ec);
    ASSERT_FALSE(ec) << "could not create scan fixture at " << root_;

    // Created out of alphabetical order so a passing sort assertion cannot be
    // an accident of creation order.
    MakeObject("zebra");
    MakeObject("apple");
    MakeObject("mango");
    // A directory without the expected file, and a stray regular file: both
    // must be skipped.
    std::filesystem::create_directories(root_ / "not_an_object", ec);
    std::ofstream(root_ / "loose_file.xml") << "<mujoco/>";
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  void MakeObject(const std::string& name) {
    std::error_code ec;
    std::filesystem::create_directories(root_ / name, ec);
    std::ofstream(root_ / name / "object.xml") << "<mujoco/>";
  }

  std::filesystem::path root_;
};

TEST_F(ObjectPoolScan, FindsOnlyDirectoriesHoldingTheObjectFileAndSortsThem) {
  std::string error;
  const std::vector<std::string> found =
      ScanObjectDirectory(root_.string(), "object.xml", error);
  EXPECT_TRUE(error.empty()) << error;
  ASSERT_EQ(found.size(), 3u);
  EXPECT_EQ(found[0], "apple");
  EXPECT_EQ(found[1], "mango");
  EXPECT_EQ(found[2], "zebra");
}

TEST_F(ObjectPoolScan, MissingDirectoryIsAnErrorNotAnEmptyPool) {
  std::string error;
  const std::vector<std::string> found =
      ScanObjectDirectory((root_ / "nope").string(), "object.xml", error);
  EXPECT_TRUE(found.empty());
  EXPECT_FALSE(error.empty()) << "a missing directory reported success";
}

TEST_F(ObjectPoolScan, EmptyDirectoryPathIsAnError) {
  std::string error;
  const std::vector<std::string> found = ScanObjectDirectory("", "object.xml", error);
  EXPECT_TRUE(found.empty());
  EXPECT_FALSE(error.empty());
}

// ── Allowlist resolution ─────────────────────────────────────────────────────

TEST(ObjectPoolAllowlist, EmptyAllowlistTakesEverything) {
  const std::vector<std::string> found{"apple", "mango", "zebra"};
  std::string error;
  const std::vector<std::string> out = ResolveCandidates(found, {}, error);
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(out, found);
}

TEST(ObjectPoolAllowlist, AllowlistSelectsAndPreservesItsOwnOrder) {
  const std::vector<std::string> found{"apple", "mango", "zebra"};
  std::string error;
  const std::vector<std::string> out = ResolveCandidates(found, {"zebra", "apple"}, error);
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0], "zebra");
  EXPECT_EQ(out[1], "apple");
}

TEST(ObjectPoolAllowlist, UnknownNameIsAnErrorNotASilentSkip) {
  const std::vector<std::string> found{"apple", "mango"};
  std::string error;
  const std::vector<std::string> out = ResolveCandidates(found, {"apple", "bananna"}, error);
  EXPECT_TRUE(out.empty());
  ASSERT_FALSE(error.empty()) << "a mistyped object name quietly shrank the pool";
  EXPECT_NE(error.find("bananna"), std::string::npos) << "error does not name the bad entry";
}

TEST(ObjectPoolAllowlist, DuplicateEntryCollapsesToOneSlot) {
  const std::vector<std::string> found{"apple", "mango"};
  std::string error;
  const std::vector<std::string> out = ResolveCandidates(found, {"apple", "apple"}, error);
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], "apple");
}

}  // namespace
}  // namespace rtc
