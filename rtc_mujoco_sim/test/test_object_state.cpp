// ── test_object_state.cpp ────────────────────────────────────────────────────
// Pins the object_state lane: which bodies count as objects, and what frame
// their poses come out in.
//
// The two axes fail differently and are tested separately.
//
//   SELECTION is a set, and its failures are silent adds and silent drops — a
//   robot link published as an object, a static table published as an object,
//   a parked pool candidate published 50 m under the floor. So the selection
//   tests assert the exact name set, never just its size: a test that counts
//   passes when one wrong body replaces one right one.
//
//   THE TRANSFORM is arithmetic, and its failures are plausible-looking wrong
//   numbers. So its oracle is literal — the expected position is written out,
//   and the expected orientation is derived through quaternion algebra, which
//   is an independent path from the rotation-matrix one under test
//   (xmat -> mju_mulMatTMat -> mju_mat2Quat).
// ────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#ifndef OBJECT_STATE_MJCF_PATH
#error "OBJECT_STATE_MJCF_PATH must be defined by CMake"
#endif
#ifndef OBJECT_STATE_POOL_OBJECTS_DIR
#error "OBJECT_STATE_POOL_OBJECTS_DIR must be defined by CMake"
#endif

namespace rtc {
namespace {

// Fixture literals. Keep in sync with object_state_scene.xml.
constexpr std::array<double, 3> kRefPos{0.1, 0.2, 0.3};
constexpr std::array<double, 4> kRefQuat{0.5, -0.5, -0.5, 0.5};
constexpr std::array<double, 3> kBoxPos{1.0, 2.0, 3.0};
constexpr std::array<double, 4> kBoxQuat{0.76506218, 0.29689154, -0.21567241, 0.52916981};

// ObjectPool attaches each candidate under the prefix "<cfg.prefix><dir>_", so
// the body named "object" inside cube_a/object.xml compiles under this name.
// Spelled out rather than asked of the pool at runtime: the published
// child_frame_id IS this string, and a test that derives it from the same
// accessor the code uses would not notice the naming changing underfoot.
constexpr const char* kActivePoolBody = "pool_cube_a_object";

// Hamilton product, MuJoCo (w, x, y, z) order. Local to the test on purpose:
// borrowing the simulator's own rotation path would make the oracle agree with
// the code by construction.
[[nodiscard]] std::array<double, 4> QuatMul(const std::array<double, 4>& a,
                                            const std::array<double, 4>& b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

[[nodiscard]] std::array<double, 4> QuatConj(const std::array<double, 4>& q) {
  return {q[0], -q[1], -q[2], -q[3]};
}

// q and -q are the same rotation, so compare through the sign that makes the
// two agree. Asserting raw components would make this test depend on
// mju_mat2Quat's branch choice rather than on the rotation.
void ExpectSameRotation(const std::array<double, 4>& actual, const std::array<double, 4>& expected,
                        double tol) {
  const double dot = (actual[0] * expected[0]) + (actual[1] * expected[1]) +
                     (actual[2] * expected[2]) + (actual[3] * expected[3]);
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(sign * actual[i], expected[i], tol) << "quaternion component " << i;
  }
}

[[nodiscard]] std::vector<std::string> PublishedNames(const MuJoCoSimulator& sim) {
  const auto& infos = sim.GetObjectStateInfos();
  const auto& samples = sim.GetObjectStateSamplesForTest();
  std::vector<std::string> names;
  for (std::size_t i = 0; i < infos.size() && i < samples.size(); ++i) {
    if (samples[i].active) {
      names.push_back(infos[i].name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

MuJoCoSimulator::Config MakeConfig(bool enabled, const std::string& reference_body,
                                   bool with_pool) {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = OBJECT_STATE_MJCF_PATH;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = 10.0;
  cfg.max_rtf = 0.0;
  cfg.n_substeps = 1;
  cfg.viewer_refresh_rate = 60.0;

  cfg.object_state.enabled = enabled;
  cfg.object_state.topic = "object_transforms";
  cfg.object_state.reference_body = reference_body;

  if (with_pool) {
    cfg.object_pool.enabled = true;
    cfg.object_pool.directory = OBJECT_STATE_POOL_OBJECTS_DIR;
    cfg.object_pool.selection = ObjectSelection::kFixed;
    cfg.object_pool.pose = PoseSampling::kFixed;
    cfg.object_pool.fixed_object = "cube_a";
    cfg.object_pool.position = {0.4, 0.0, 0.6};
    cfg.object_pool.spawn_on_start = true;
    cfg.object_pool.seed = 42;
  }

  JointGroupConfig grp;
  grp.name = "arm";
  grp.command_joint_names = {"j1", "j2"};
  grp.state_joint_names = {"j1", "j2"};
  grp.command_topic = "/arm/cmd";
  grp.state_topic = "/arm/state";
  grp.is_robot = true;
  cfg.groups.push_back(grp);
  return cfg;
}

// ── 1. Disabled = inert. ────────────────────────────────────────────────────
TEST(ObjectState, DisabledDiscoversNothing) {
  MuJoCoSimulator sim(MakeConfig(false, "ref_frame", /*with_pool=*/false));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_FALSE(sim.HasObjectStates());
  EXPECT_TRUE(sim.GetObjectStateInfos().empty());
  EXPECT_TRUE(sim.GetObjectStateFrameId().empty());
}

// ── 2. Selection: freejoint bodies only. ────────────────────────────────────
//
// The exact set, not the count. arm_base / arm_link are hinge-jointed, and
// static_prop and ref_frame have no joint at all — every one of them has a
// name and geoms, and a rule that keyed off anything but the joint would let
// at least one of them through.
TEST(ObjectState, OnlyFreejointBodiesAreObjects) {
  MuJoCoSimulator sim(MakeConfig(true, "ref_frame", /*with_pool=*/false));
  ASSERT_TRUE(sim.Initialize());
  ASSERT_TRUE(sim.HasObjectStates());

  const auto& infos = sim.GetObjectStateInfos();
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos[0].name, "loose_box");

  sim.StepForTest();
  EXPECT_EQ(PublishedNames(sim), std::vector<std::string>{"loose_box"});
}

// ── 3. Reference frame: the transform, against a literal oracle. ────────────
TEST(ObjectState, PoseIsExpressedInTheReferenceBodyFrame) {
  MuJoCoSimulator sim(MakeConfig(true, "ref_frame", /*with_pool=*/false));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetObjectStateFrameId(), "ref_frame");

  sim.StepForTest();
  const auto& samples = sim.GetObjectStateSamplesForTest();
  ASSERT_EQ(samples.size(), 1u);
  ASSERT_TRUE(samples[0].active);

  // R_ref maps (x,y,z)_ref onto (-z, x, -y)_world, so
  //   p_rel = R_ref^T (p_obj - p_ref) = (dy, -dz, -dx)
  // with (dx,dy,dz) = (0.9, 1.8, 2.7) → (1.8, -2.7, -0.9). Written as literals
  // rather than recomputed here: an oracle that re-derives the transform can
  // re-derive it wrong the same way the code does.
  EXPECT_NEAR(samples[0].position[0], 1.8, 1e-9);
  EXPECT_NEAR(samples[0].position[1], -2.7, 1e-9);
  EXPECT_NEAR(samples[0].position[2], -0.9, 1e-9);

  ExpectSameRotation(samples[0].quat, QuatMul(QuatConj(kRefQuat), kBoxQuat), 1e-7);
}

// ── 4. No reference_body = world, and world is NOT the same answer. ─────────
//
// The inequality is the point. Without it, a reference_body that silently
// resolved to world would pass test 3's frame_id check and every selection
// test in this file.
TEST(ObjectState, EmptyReferenceBodyMeansWorldAndDiffersFromTheReferenceFrame) {
  MuJoCoSimulator sim(MakeConfig(true, "", /*with_pool=*/false));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetObjectStateFrameId(), "world");

  sim.StepForTest();
  const auto& samples = sim.GetObjectStateSamplesForTest();
  ASSERT_EQ(samples.size(), 1u);

  // World frame = the MJCF pose verbatim.
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(samples[0].position[i], kBoxPos[i], 1e-9) << "position component " << i;
  }
  ExpectSameRotation(samples[0].quat, kBoxQuat, 1e-7);

  // And that is a different answer from the ref_frame one, so the reference
  // transform is doing something rather than being a no-op.
  EXPECT_GT(std::abs(kBoxPos[0] - 1.8), 0.1);
}

// ── 5. An unknown reference_body fails Initialize rather than falling back. ─
TEST(ObjectState, UnknownReferenceBodyFailsInitialize) {
  MuJoCoSimulator sim(MakeConfig(true, "no_such_body", /*with_pool=*/false));
  EXPECT_FALSE(sim.Initialize());
}

// ── 6. frame_id override wins over the reference body's MJCF name. ─────────
TEST(ObjectState, ExplicitFrameIdOverridesTheBodyName) {
  auto cfg = MakeConfig(true, "ref_frame", /*with_pool=*/false);
  cfg.object_state.frame_id = "base";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetObjectStateFrameId(), "base");
}

// ── 7. Parked pool candidates drop out; the active one stays in. ────────────
//
// Every pool slot is a freejoint body, so all three are discovered — the
// filtering is a per-tick decision, and this is the test that says so. The
// name set is asserted before AND after a refresh because the interesting
// failure is a set frozen at Initialize, which would keep publishing the
// object that was just parked away and would pass any count-only check.
TEST(ObjectState, ParkedPoolCandidatesAreOmittedAndFollowRefresh) {
  MuJoCoSimulator sim(MakeConfig(true, "ref_frame", /*with_pool=*/true));
  ASSERT_TRUE(sim.Initialize());

  // 3 pool candidates + loose_box.
  const auto& infos = sim.GetObjectStateInfos();
  EXPECT_EQ(infos.size(), 4u);

  sim.StepForTest();
  const auto first = PublishedNames(sim);
  // spawn_on_start with selection=fixed puts cube_a in the scene; cube_b and
  // cube_c stay parked. The body name is the pool's, not the directory's:
  // ObjectPool attaches with prefix "<pool prefix><directory>_", so the body
  // "object" inside cube_a/object.xml compiles as "pool_cube_a_object".
  EXPECT_EQ(first, (std::vector<std::string>{"loose_box", kActivePoolBody}));

  // Switch to random selection so a refresh actually changes the active slot,
  // then drive the same flag->handler edge the viewer's 'o' key uses.
  MuJoCoSimulator::Config random_cfg = MakeConfig(true, "ref_frame", /*with_pool=*/true);
  random_cfg.object_pool.selection = ObjectSelection::kRandom;
  random_cfg.object_pool.avoid_repeat = true;
  MuJoCoSimulator random_sim(std::move(random_cfg));
  ASSERT_TRUE(random_sim.Initialize());
  random_sim.StepForTest();
  const auto before = PublishedNames(random_sim);
  ASSERT_EQ(before.size(), 2u);

  random_sim.RefreshObjectForTest();
  const auto after = PublishedNames(random_sim);
  ASSERT_EQ(after.size(), 2u);
  // avoid_repeat guarantees a different candidate, so the published set must
  // have moved with it.
  EXPECT_NE(before, after);
  EXPECT_EQ(after[0], "loose_box");  // sorted; the non-pool body is unaffected
}

// ── 8. The active pool object's pose is the spawn pose, in the ref frame. ──
//
// Ties the two axes together: an object that is in the set must also carry a
// pose that went through the reference transform, not a raw world one.
TEST(ObjectState, ActivePoolObjectCarriesTheReferenceFramePose) {
  MuJoCoSimulator sim(MakeConfig(true, "ref_frame", /*with_pool=*/true));
  ASSERT_TRUE(sim.Initialize());
  sim.StepForTest();

  const auto& infos = sim.GetObjectStateInfos();
  const auto& samples = sim.GetObjectStateSamplesForTest();
  ASSERT_EQ(infos.size(), samples.size());

  bool checked = false;
  for (std::size_t i = 0; i < infos.size(); ++i) {
    if (infos[i].name != kActivePoolBody) {
      continue;
    }
    ASSERT_TRUE(samples[i].active);
    // Spawn centre (0.4, 0.0, 0.6) with pose=fixed, so
    //   d = (0.3, -0.2, 0.3) and p_rel = (dy, -dz, -dx) = (-0.2, -0.3, -0.3).
    EXPECT_NEAR(samples[i].position[0], -0.2, 1e-9);
    EXPECT_NEAR(samples[i].position[1], -0.3, 1e-9);
    EXPECT_NEAR(samples[i].position[2], -0.3, 1e-9);
    checked = true;
  }
  EXPECT_TRUE(checked) << "pool_cube_a was not among the discovered objects";
}

}  // namespace
}  // namespace rtc
