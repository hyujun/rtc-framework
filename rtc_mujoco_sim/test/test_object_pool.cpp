// ── test_object_pool.cpp ──────────────────────────────────────────────────────
// Object pool against a real compiled model: attachment, parking, spawning,
// refresh, reset interaction, and the failure modes that must not be silent.
//
// The two controls this file is built around:
//   POSITIVE — the active object falls and comes to rest on the floor. Only a
//              body that is genuinely in the scene, in collision, and under
//              gravity does that; every weaker assertion (a name changed, an
//              id resolved) passes for a pool that spawned nothing.
//   NEGATIVE — a parked object does not move AT ALL over the same run. This is
//              what catches parking that disables contacts but leaves the body
//              falling or drifting, which is invisible to any test that only
//              looks at the active object.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "rtc_mujoco_sim/object_pool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifndef POOL_SCENE_MJCF_PATH
#error "POOL_SCENE_MJCF_PATH must be defined by CMake"
#endif
#ifndef POOL_OBJECTS_DIR
#error "POOL_OBJECTS_DIR must be defined by CMake"
#endif

namespace rtc {
namespace {

constexpr double kParkZ = -50.0;
constexpr double kSpawnX = 0.60;
constexpr double kSpawnY = 0.00;
constexpr double kSpawnZ = 0.50;
/// Collision half-extent of the cube fixtures; a settled cube's centre sits
/// here above the floor.
constexpr double kCubeHalf = 0.03;

MuJoCoSimulator::Config MakeConfig(bool pool_enabled) {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = POOL_SCENE_MJCF_PATH;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = 10.0;
  cfg.max_rtf = 0.0;
  cfg.n_substeps = 1;

  JointGroupConfig g;
  g.name = "arm";
  g.command_joint_names = {"j1", "j2"};
  g.state_joint_names = {"j1", "j2"};
  g.command_topic = "/arm/cmd";
  g.state_topic = "/arm/state";
  g.is_robot = true;
  cfg.groups.push_back(g);

  cfg.object_pool.enabled = pool_enabled;
  cfg.object_pool.directory = POOL_OBJECTS_DIR;
  cfg.object_pool.position = {kSpawnX, kSpawnY, kSpawnZ};
  cfg.object_pool.park_position = {0.0, 0.0, kParkZ};
  cfg.object_pool.selection = ObjectSelection::kFixed;
  cfg.object_pool.pose = PoseSampling::kFixed;
  cfg.object_pool.spawn_on_start = true;
  cfg.object_pool.seed = 20260830;  // fixed: these assertions must be repeatable
  return cfg;
}

/// Freejoint qpos of candidate `index` (7 values: xyz + wxyz).
std::array<double, 7> SlotQpos(const MuJoCoSimulator& sim, std::size_t index) {
  std::array<double, 7> out{};
  const int adr = sim.GetObjectPool().QposAdrAt(index);
  EXPECT_GE(adr, 0);
  if (adr < 0) {
    return out;
  }
  for (int i = 0; i < 7; ++i) {
    out[static_cast<std::size_t>(i)] = sim.GetData()->qpos[adr + i];
  }
  return out;
}

bool IsParked(const std::array<double, 7>& q) {
  return q[2] < kParkZ + 1.0;
}

std::size_t ActiveIndex(const MuJoCoSimulator& sim) {
  const auto& pool = sim.GetObjectPool();
  for (std::size_t i = 0; i < pool.Size(); ++i) {
    if (pool.BodyIdAt(i) == pool.ActiveBodyId()) {
      return i;
    }
  }
  return pool.Size();
}

// ── Model shape ──────────────────────────────────────────────────────────────

TEST(ObjectPoolModel, DisabledPoolLeavesTheCompiledModelUntouched) {
  // Guards the "costs nothing when off" claim, and doubles as the regression
  // gate for splitting mj_loadXML into mj_parseXML + mj_compile: if that split
  // changed anything about compilation, these counts would move.
  MuJoCoSimulator off(MakeConfig(false));
  ASSERT_TRUE(off.Initialize());
  const int nq0 = static_cast<int>(off.GetModel()->nq);
  const int nbody0 = static_cast<int>(off.GetModel()->nbody);
  const int ngeom0 = static_cast<int>(off.GetModel()->ngeom);

  EXPECT_EQ(off.GetObjectPool().Size(), 0u);
  EXPECT_EQ(off.GetActiveObjectName(), "none");

  MuJoCoSimulator on(MakeConfig(true));
  ASSERT_TRUE(on.Initialize());
  // 3 fixtures x (freejoint = 7 qpos, 1 body, 2 geoms)
  EXPECT_EQ(static_cast<int>(on.GetModel()->nq), nq0 + 21);
  EXPECT_EQ(static_cast<int>(on.GetModel()->nbody), nbody0 + 3);
  EXPECT_EQ(static_cast<int>(on.GetModel()->ngeom), ngeom0 + 6);
}

TEST(ObjectPoolModel, EveryCandidateIsAttachedWithAFreejoint) {
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const auto& pool = sim.GetObjectPool();
  ASSERT_EQ(pool.Size(), 3u);
  const std::vector<std::string> expected{"cube_a", "cube_b", "cube_c"};
  EXPECT_EQ(pool.CandidateNames(), expected) << "candidates are not in sorted order";
  for (std::size_t i = 0; i < pool.Size(); ++i) {
    EXPECT_GE(pool.BodyIdAt(i), 0);
    EXPECT_GE(pool.QposAdrAt(i), 0);
  }
}

TEST(ObjectPoolModel, KeyframesCarryTheParkPoseRatherThanTheWorldOrigin) {
  // mjs_attach pads existing keyframes with (0,0,0, 1,0,0,0). Left alone, a
  // keyframe reset would drop every pool object at the world origin — on top
  // of the robot — instead of leaving them parked.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const mjModel* m = sim.GetModel();
  ASSERT_GT(m->nkey, 0) << "fixture lost its keyframe; this test would be vacuous";

  const auto& pool = sim.GetObjectPool();
  for (int k = 0; k < m->nkey; ++k) {
    for (std::size_t i = 0; i < pool.Size(); ++i) {
      const int adr = pool.QposAdrAt(i);
      ASSERT_GE(adr, 0);
      const mjtNum* q = m->key_qpos + static_cast<std::ptrdiff_t>(k) * m->nq + adr;
      EXPECT_DOUBLE_EQ(q[2], kParkZ) << "keyframe " << k << " slot " << i;
      EXPECT_DOUBLE_EQ(q[3], 1.0) << "keyframe quaternion is not identity";
    }
  }
}

TEST(ObjectPoolModel, UnparkRestoresEachGeomsOwnContactFilter) {
  // The fixtures give the visual geom contype=0 and the collision geom
  // contype=1, exactly as object_sim does. A blanket restore to 1 would put
  // the visual shell into collision, which only a non-uniform fixture reveals.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const mjModel* m = sim.GetModel();
  const int body = sim.GetObjectPool().ActiveBodyId();
  ASSERT_GE(body, 0);

  int zero_filter = 0;
  int live_filter = 0;
  for (int g = m->body_geomadr[body]; g < m->body_geomadr[body] + m->body_geomnum[body]; ++g) {
    if (m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0) {
      ++zero_filter;
    } else {
      ++live_filter;
    }
  }
  EXPECT_EQ(zero_filter, 1) << "the visual geom's contype=0 was not preserved on unpark";
  EXPECT_EQ(live_filter, 1) << "the collision geom was not restored on unpark";
}

// ── Spawning ─────────────────────────────────────────────────────────────────

TEST(ObjectPoolSpawn, SpawnOnStartActivatesExactlyOneObjectAtTheConfiguredPose) {
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const auto& pool = sim.GetObjectPool();
  EXPECT_NE(sim.GetActiveObjectName(), "none");

  int unparked = 0;
  for (std::size_t i = 0; i < pool.Size(); ++i) {
    const auto q = SlotQpos(sim, i);
    if (!IsParked(q)) {
      ++unparked;
      EXPECT_DOUBLE_EQ(q[0], kSpawnX);
      EXPECT_DOUBLE_EQ(q[1], kSpawnY);
      EXPECT_DOUBLE_EQ(q[2], kSpawnZ);
      EXPECT_DOUBLE_EQ(q[3], 1.0);  // identity orientation for rpy = 0
    }
  }
  EXPECT_EQ(unparked, 1);
}

TEST(ObjectPoolSpawn, SpawnOnStartCanBeDisabled) {
  auto cfg = MakeConfig(true);
  cfg.object_pool.spawn_on_start = false;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetActiveObjectName(), "none");
  for (std::size_t i = 0; i < sim.GetObjectPool().Size(); ++i) {
    EXPECT_TRUE(IsParked(SlotQpos(sim, i))) << "slot " << i << " spawned unbidden";
  }
}

TEST(ObjectPoolSpawn, ActiveObjectFallsAndComesToRestOnTheFloor) {
  // POSITIVE CONTROL — see the file header.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const std::size_t active = ActiveIndex(sim);
  ASSERT_LT(active, sim.GetObjectPool().Size());
  const double z_start = SlotQpos(sim, active)[2];

  for (int i = 0; i < 1500; ++i) {
    sim.StepForTest();
  }
  const double z_mid = SlotQpos(sim, active)[2];
  for (int i = 0; i < 500; ++i) {
    sim.StepForTest();
  }
  const double z_end = SlotQpos(sim, active)[2];

  EXPECT_LT(z_end, z_start - 0.3) << "the spawned object never fell";
  EXPECT_NEAR(z_end, kCubeHalf, 0.02) << "it did not settle on the floor";
  EXPECT_NEAR(z_end, z_mid, 1e-3) << "it never came to rest";
}

TEST(ObjectPoolSpawn, ParkedObjectsDoNotMoveAtAll) {
  // NEGATIVE CONTROL — see the file header. Parking that only disables
  // collisions leaves the body in free fall; parking that cancels gravity but
  // keeps residual velocity leaves it drifting. Both show up here and nowhere
  // else, so the tolerance is exact equality rather than a near-check.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const auto& pool = sim.GetObjectPool();
  const std::size_t active = ActiveIndex(sim);

  std::vector<std::array<double, 7>> before;
  for (std::size_t i = 0; i < pool.Size(); ++i) {
    before.push_back(SlotQpos(sim, i));
  }

  for (int i = 0; i < 2000; ++i) {
    sim.StepForTest();
  }

  int checked = 0;
  for (std::size_t i = 0; i < pool.Size(); ++i) {
    if (i == active) {
      continue;
    }
    const auto after = SlotQpos(sim, i);
    for (std::size_t k = 0; k < 7; ++k) {
      EXPECT_DOUBLE_EQ(after[k], before[i][k]) << "parked slot " << i << " qpos[" << k << "] moved";
    }
    ++checked;
  }
  EXPECT_EQ(checked, 2) << "no parked object was actually examined";
}

TEST(ObjectPoolSpawn, ParkingStopsAnObjectThatWasAlreadyMoving) {
  // The specific failure the velocity reset exists for: refreshing while the
  // active object is mid-fall. With collisions off there is nothing to stop it,
  // so leftover momentum would carry it away from the park position forever.
  auto cfg = MakeConfig(true);
  cfg.object_pool.selection = ObjectSelection::kRandom;
  cfg.object_pool.avoid_repeat = true;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  const std::size_t falling = ActiveIndex(sim);
  for (int i = 0; i < 120; ++i) {  // build up real downward velocity
    sim.StepForTest();
  }
  ASSERT_LT(SlotQpos(sim, falling)[2], kSpawnZ - 0.01) << "the object was not moving yet";

  sim.RefreshObjectForTest();
  const auto parked_at = SlotQpos(sim, falling);
  ASSERT_TRUE(IsParked(parked_at));

  for (int i = 0; i < 1000; ++i) {
    sim.StepForTest();
  }
  const auto later = SlotQpos(sim, falling);
  for (std::size_t k = 0; k < 7; ++k) {
    EXPECT_DOUBLE_EQ(later[k], parked_at[k])
        << "an object parked while moving drifted (qpos[" << k << "])";
  }
}

// ── Refresh ──────────────────────────────────────────────────────────────────

TEST(ObjectPoolRefresh, RefreshSwapsTheObjectAndParksThePreviousOne) {
  auto cfg = MakeConfig(true);
  cfg.object_pool.selection = ObjectSelection::kRandom;
  cfg.object_pool.avoid_repeat = true;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  for (int round = 0; round < 8; ++round) {
    const std::string previous_name = sim.GetActiveObjectName();
    const std::size_t previous = ActiveIndex(sim);

    sim.RefreshObjectForTest();

    EXPECT_NE(sim.GetActiveObjectName(), previous_name) << "round " << round;
    EXPECT_TRUE(IsParked(SlotQpos(sim, previous)))
        << "the previous object was not parked in round " << round;
    const std::size_t now = ActiveIndex(sim);
    ASSERT_LT(now, sim.GetObjectPool().Size());
    EXPECT_FALSE(IsParked(SlotQpos(sim, now)));

    int unparked = 0;
    for (std::size_t i = 0; i < sim.GetObjectPool().Size(); ++i) {
      if (!IsParked(SlotQpos(sim, i))) {
        ++unparked;
      }
    }
    EXPECT_EQ(unparked, 1) << "more than one object was live in round " << round;
  }
}

TEST(ObjectPoolRefresh, RequestFlagIsDrainedByTheStepPath) {
  // Exercises the flag -> handler edge the viewer's 'o' key uses, rather than
  // the direct call the other tests make: RequestObjectRefresh only raises an
  // atomic, and it is the step path that must consume it.
  auto cfg = MakeConfig(true);
  cfg.object_pool.selection = ObjectSelection::kRandom;
  cfg.object_pool.avoid_repeat = true;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  const std::string before = sim.GetActiveObjectName();
  sim.StepForTest();
  EXPECT_EQ(sim.GetActiveObjectName(), before) << "a step spawned without being asked";

  sim.RequestObjectRefresh();
  sim.StepForTest();
  EXPECT_NE(sim.GetActiveObjectName(), before) << "the pending refresh flag was never drained";

  const std::string after = sim.GetActiveObjectName();
  sim.StepForTest();
  EXPECT_EQ(sim.GetActiveObjectName(), after) << "the flag was consumed more than once";
}

TEST(ObjectPoolRefresh, FixedSelectionAlwaysSpawnsTheNamedObject) {
  // Modes 1 and 2 ("same object").
  auto cfg = MakeConfig(true);
  cfg.object_pool.selection = ObjectSelection::kFixed;
  cfg.object_pool.fixed_object = "cube_b";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  EXPECT_EQ(sim.GetActiveObjectName(), "cube_b");
  for (int i = 0; i < 8; ++i) {
    sim.RefreshObjectForTest();
    EXPECT_EQ(sim.GetActiveObjectName(), "cube_b");
  }
}

TEST(ObjectPoolRefresh, RandomPoseStaysInRangeAndCoversIt) {
  // Modes 2 and 4 ("random pose"). Fixed object so the sampled pose can be
  // read from one slot without chasing the active index.
  auto cfg = MakeConfig(true);
  cfg.object_pool.selection = ObjectSelection::kFixed;
  cfg.object_pool.fixed_object = "cube_a";
  cfg.object_pool.pose = PoseSampling::kRandom;
  cfg.object_pool.position_variation = {0.10, 0.20, 0.0};
  cfg.object_pool.rpy_variation = {0.0, 0.0, 3.0};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  const std::size_t slot = ActiveIndex(sim);
  ASSERT_LT(slot, sim.GetObjectPool().Size());

  double lo_x = 1e9, hi_x = -1e9, lo_y = 1e9, hi_y = -1e9, min_w = 2.0;
  for (int i = 0; i < 400; ++i) {
    sim.RefreshObjectForTest();
    const auto q = SlotQpos(sim, slot);
    EXPECT_GE(q[0], kSpawnX - 0.10);
    EXPECT_LE(q[0], kSpawnX + 0.10);
    EXPECT_GE(q[1], kSpawnY - 0.20);
    EXPECT_LE(q[1], kSpawnY + 0.20);
    EXPECT_DOUBLE_EQ(q[2], kSpawnZ) << "a zero-variation axis moved";
    lo_x = std::min(lo_x, q[0]);
    hi_x = std::max(hi_x, q[0]);
    lo_y = std::min(lo_y, q[1]);
    hi_y = std::max(hi_y, q[1]);
    min_w = std::min(min_w, std::fabs(q[3]));
  }
  // Containment alone passes for a spawner stuck at the centre.
  EXPECT_GT(hi_x - lo_x, 0.9 * 0.20) << "x barely varied";
  EXPECT_GT(hi_y - lo_y, 0.9 * 0.40) << "y barely varied";
  EXPECT_LT(min_w, 0.2) << "yaw never actually rotated the object";
}

TEST(ObjectPoolRefresh, FixedPoseNeverVaries) {
  // Modes 1 and 3 ("fixed pose") — the complement of the test above.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  for (int i = 0; i < 8; ++i) {
    sim.RefreshObjectForTest();
    const auto q = SlotQpos(sim, ActiveIndex(sim));
    EXPECT_DOUBLE_EQ(q[0], kSpawnX);
    EXPECT_DOUBLE_EQ(q[1], kSpawnY);
    EXPECT_DOUBLE_EQ(q[2], kSpawnZ);
  }
}

// ── Reset interaction ────────────────────────────────────────────────────────

TEST(ObjectPoolReset, ResetRestoresTheActiveObjectInsteadOfEmptyingTheScene) {
  // mj_resetData restores qpos0, which parks EVERY slot. Without the explicit
  // re-apply, pressing R would leave the scene with no object at all — and
  // nothing about the robot state would look wrong.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  const std::string name = sim.GetActiveObjectName();
  const std::size_t active = ActiveIndex(sim);
  const auto spawned = SlotQpos(sim, active);

  for (int i = 0; i < 400; ++i) {
    sim.StepForTest();
  }
  ASSERT_LT(SlotQpos(sim, active)[2], spawned[2] - 0.05) << "the object never moved";

  sim.ResetForTest();

  EXPECT_EQ(sim.GetActiveObjectName(), name) << "reset changed which object is active";
  const auto after = SlotQpos(sim, active);
  for (std::size_t k = 0; k < 7; ++k) {
    EXPECT_DOUBLE_EQ(after[k], spawned[k]) << "qpos[" << k << "] not restored";
  }
  int unparked = 0;
  for (std::size_t i = 0; i < sim.GetObjectPool().Size(); ++i) {
    if (!IsParked(SlotQpos(sim, i))) {
      ++unparked;
    }
  }
  EXPECT_EQ(unparked, 1);
}

TEST(ObjectPoolReset, ObjectStillFallsAfterAReset) {
  // Guards the model-side half of the restore: re-applying qpos but leaving
  // gravcomp at 1 or the contact filters at 0 would look correct in the pose
  // assertions above and be inert in physics.
  MuJoCoSimulator sim(MakeConfig(true));
  ASSERT_TRUE(sim.Initialize());
  sim.ResetForTest();
  const std::size_t active = ActiveIndex(sim);
  const double z0 = SlotQpos(sim, active)[2];
  for (int i = 0; i < 1500; ++i) {
    sim.StepForTest();
  }
  const double z1 = SlotQpos(sim, active)[2];
  EXPECT_LT(z1, z0 - 0.3) << "the object stopped responding to gravity after a reset";
  EXPECT_NEAR(z1, kCubeHalf, 0.02) << "it no longer collides with the floor after a reset";
}

// ── Failure modes must be loud ───────────────────────────────────────────────

TEST(ObjectPoolFailure, EnabledPoolWithNoCandidatesFailsInitialize) {
  // An enabled-but-empty pool is a randomiser that reports success while
  // randomising nothing.
  auto cfg = MakeConfig(true);
  cfg.object_pool.directory = std::string(POOL_OBJECTS_DIR) + "/cube_a";  // holds no candidates
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(ObjectPoolFailure, MissingDirectoryFailsInitialize) {
  auto cfg = MakeConfig(true);
  cfg.object_pool.directory = std::string(POOL_OBJECTS_DIR) + "/does_not_exist";
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(ObjectPoolFailure, UnknownFixedObjectFailsInitialize) {
  auto cfg = MakeConfig(true);
  cfg.object_pool.fixed_object = "cube_z";
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(ObjectPoolFailure, UnknownAllowlistEntryFailsInitialize) {
  auto cfg = MakeConfig(true);
  cfg.object_pool.objects = {"cube_a", "cube_typo"};
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(ObjectPoolFailure, AllowlistNarrowsThePool) {
  MuJoCoSimulator baseline(MakeConfig(false));
  ASSERT_TRUE(baseline.Initialize());
  const int nq0 = static_cast<int>(baseline.GetModel()->nq);

  auto cfg = MakeConfig(true);
  cfg.object_pool.objects = {"cube_c"};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetObjectPool().Size(), 1u);
  EXPECT_EQ(sim.GetActiveObjectName(), "cube_c");
  EXPECT_EQ(static_cast<int>(sim.GetModel()->nq), nq0 + 7)
      << "the allowlist did not actually shrink the compiled model";
}

TEST(ObjectPoolFailure, RefreshIsANoOpWhenThePoolIsDisabled) {
  // 'o' in a scene with no pool must not cost a step: DrainPendingObjectRefresh
  // reporting "handled" would make the SimLoop `continue` past a state publish
  // on every press of a key that does nothing.
  MuJoCoSimulator sim(MakeConfig(false));
  ASSERT_TRUE(sim.Initialize());
  sim.RequestObjectRefresh();
  const double t_before = sim.GetData()->time;
  sim.StepForTest();
  EXPECT_GT(sim.GetData()->time, t_before) << "a disabled pool swallowed a physics step";
  EXPECT_EQ(sim.GetActiveObjectName(), "none");
}

TEST(ObjectPoolConcurrency, ObserversStayConsistentWhileTheRefreshPathRuns) {
  // Contract pin for the viewer overlay, which calls Enabled()/ActiveName()
  // every frame from its own thread while the SimLoop runs Refresh().
  //
  // HONESTY NOTE: this does NOT go red if active_ loses its atomic. An
  // unsynchronised size_t is undefined behaviour, not a deterministic failure,
  // so under a normal build this stays green either way. What it pins in CI is
  // the part that IS deterministic: ActiveName() and ActiveBodyId() must never
  // disagree with each other or return something outside the candidate set.
  // Both used to read active_ twice — once to bounds-check, once to index — so
  // a writer landing between the two reads could index a slot the check had
  // just cleared. They read it once now, and that is what breaks here.
  //
  // The race itself was verified out-of-band with ThreadSanitizer, which does
  // go red: build this package into a scratch tree with
  //   --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1"
  //                -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread
  // and run `setarch $(uname -m) -R ./test_object_pool` (ASLR off, or TSAN
  // aborts with "unexpected memory mapping" on this kernel). With active_ a
  // plain size_t that reports a write in ObjectPool::Refresh racing a read in
  // ObjectPool::ActiveName; with the atomic it is clean.

  // Random selection, not the fixed default: with a fixed object the index the
  // reader is racing against never changes value, and the whole test degrades
  // into "does calling an accessor crash".
  auto cfg = MakeConfig(true);
  cfg.object_pool.selection = ObjectSelection::kRandom;
  cfg.object_pool.avoid_repeat = true;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  const auto& pool = sim.GetObjectPool();
  const std::vector<std::string> candidates = pool.CandidateNames();
  ASSERT_GT(candidates.size(), 1u) << "a single-candidate pool cannot exercise a change";

  std::atomic<bool> stop{false};
  std::atomic<int> reads{0};
  std::atomic<bool> saw_bad_name{false};
  std::atomic<bool> saw_bad_pairing{false};
  std::mutex seen_mutex;
  std::set<std::string> seen;

  std::thread viewer([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      // The reference is safe to hold across the two reads: it points either at
      // a slot name (slots_ is immutable after Resolve) or at the "none"
      // sentinel. What can move underneath is the *index*, which is the point.
      const std::string& name = pool.ActiveName();
      const int body = pool.ActiveBodyId();
      const bool known = name == "none" ||
                         std::find(candidates.begin(), candidates.end(), name) != candidates.end();
      if (!known) {
        saw_bad_name.store(true, std::memory_order_relaxed);
      }
      // "none" and a valid body id are mutually exclusive; a torn or
      // double-read index is exactly what would break the pairing.
      if ((name == "none") != (body < 0)) {
        saw_bad_pairing.store(true, std::memory_order_relaxed);
      }
      {
        const std::lock_guard<std::mutex> lock(seen_mutex);
        seen.insert(name);
      }
      reads.fetch_add(1, std::memory_order_relaxed);
    }
  });

  for (int i = 0; i < 300; ++i) {
    sim.RefreshObjectForTest();
    sim.StepForTest();
  }
  stop.store(true, std::memory_order_relaxed);
  viewer.join();
  const std::set<std::string> distinct = seen;

  EXPECT_GT(reads.load(), 0) << "the reader thread never ran; the test observed nothing";
  EXPECT_GT(distinct.size(), 1u)
      << "the active object never changed; this test raced against a constant";
  EXPECT_FALSE(saw_bad_name.load()) << "ActiveName() returned a name outside the candidate set";
  EXPECT_FALSE(saw_bad_pairing.load()) << "ActiveName() and ActiveBodyId() disagreed";
}

}  // namespace
}  // namespace rtc
