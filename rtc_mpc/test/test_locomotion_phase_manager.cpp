/// @file test_locomotion_phase_manager.cpp
/// @brief The gait phase manager on the public G1 with the shipped walk: its
///        loader (every refusal), the context it hands the OCP (the horizon
///        equal byte for byte to the scheduler and builder driven directly
///        with the same inputs, accepted by the kinodynamic OCP), the ticks
///        that give no horizon, the command slot written from another thread,
///        the forwarded requests, the measured planar velocity, and the node
///        grid the tick time is quantised to (node times from the grid index,
///        a node's time and mode the same on every tick that holds it).

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/phase/locomotion_phase_manager.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"
#include "test_utils/loco_reference_fixture.hpp"
#include "test_utils/shipped_walk_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::FilteredCommand;
using rtc::mpc::GaitSchedulerError;
using rtc::mpc::LocomotionPhaseError;
using rtc::mpc::LocomotionPhaseManager;
using rtc::mpc::PhaseContext;
using rtc::mpc::ReferenceError;

/// The shipped command of a straight walk (filtered, valid).
FilteredCommand WalkCommand(std::uint64_t sequence = 11) {
  FilteredCommand c = tu::RegisteredCommand();
  c.v_y = 0.0;
  c.omega_z = 0.0;
  c.base_height = 0.7925;
  c.sequence = sequence;
  return c;
}

class LocomotionPhaseManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    stand_ = tu::G1Stand(model_);
  }

  [[nodiscard]] LocomotionPhaseError LoadWith(const YAML::Node& cfg) const {
    LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
    return m.Load(cfg);
  }

  /// The tick of the thread: the snapshot and the vectors copied from it.
  [[nodiscard]] PhaseContext Tick(LocomotionPhaseManager& m, const rtc::mpc::MPCStateSnapshot& snap,
                                  double t) const {
    const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(snap.q.data(), model_.nq);
    const Eigen::VectorXd v = Eigen::Map<const Eigen::VectorXd>(snap.v.data(), model_.nv);
    rtc::mpc::PhaseManagerBase& base = m;
    return base.Update(snap, q, v, Eigen::VectorXd(), pinocchio::SE3::Identity(), t);
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  Eigen::VectorXd stand_;
};

// ── Loader ──────────────────────────────────────────────────────────────────

TEST_F(LocomotionPhaseManagerTest, LoadsTheShippedSections) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  EXPECT_TRUE(m.Initialised());
  EXPECT_EQ(m.horizon_nodes(), tu::kKinoNodes);
  EXPECT_EQ(m.dt(), tu::kKinoDt);
  EXPECT_EQ(m.builder().n_nodes(), tu::kKinoNodes + 1);
  EXPECT_EQ(m.scheduler().table().n_gaits, 3U);
  EXPECT_EQ(m.CurrentPhaseId(), 0);
  EXPECT_EQ(m.CurrentPhaseName(), "stance");
  EXPECT_EQ(m.LastUpdateError(), LocomotionPhaseError::kNotInitialised) << "no Update yet";
  EXPECT_NO_THROW(m.Init(tu::ShippedLocomotionYaml()));
}

TEST_F(LocomotionPhaseManagerTest, EveryKeyIsRequiredAndAnUnknownOneRefused) {
  for (const char* key : {"horizon_nodes", "dt", "gait", "swing", "terrain", "reference"}) {
    YAML::Node n = tu::ShippedLocomotionYaml();
    n.remove(key);
    EXPECT_EQ(LoadWith(n), LocomotionPhaseError::kMissingKey) << key;
  }
  YAML::Node extra = tu::ShippedLocomotionYaml();
  extra["command"] = 1.0;
  EXPECT_EQ(LoadWith(extra), LocomotionPhaseError::kUnknownKey);
  EXPECT_EQ(LoadWith(YAML::Load("[1, 2]")), LocomotionPhaseError::kNotAMap);
}

TEST_F(LocomotionPhaseManagerTest, ABadValueIsRefusedByItsSection) {
  struct Case {
    const char* key;
    const char* value;
    LocomotionPhaseError expect;
  };

  for (const Case c :
       {Case{"horizon_nodes", "0", LocomotionPhaseError::kInvalidHorizon},
        Case{"horizon_nodes", "2.5", LocomotionPhaseError::kInvalidHorizon},
        Case{"horizon_nodes", "100000", LocomotionPhaseError::kInvalidHorizon},
        Case{"dt", "0.0", LocomotionPhaseError::kInvalidHorizon},
        Case{"dt", ".nan", LocomotionPhaseError::kInvalidHorizon},
        Case{"gait", "{sequence: []}", LocomotionPhaseError::kInvalidGait},
        Case{"swing", "{height: 0.08}", LocomotionPhaseError::kInvalidSwing},
        Case{"terrain", "{depth: 0.0}", LocomotionPhaseError::kInvalidTerrain},
        Case{"reference", "{q_default: {}}", LocomotionPhaseError::kInvalidReference}}) {
    YAML::Node n = tu::ShippedLocomotionYaml();
    n[c.key] = YAML::Load(c.value);
    EXPECT_EQ(LoadWith(n), c.expect) << c.key << ": " << c.value;
  }
  LocomotionPhaseManager one(handler_, {"left_foot"});
  EXPECT_EQ(one.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kContactNameMismatch);
  EXPECT_FALSE(one.Initialised());
  EXPECT_THROW(one.Init(tu::ShippedLocomotionYaml()), std::runtime_error);

  rtc::mpc::RobotModelHandler empty;
  LocomotionPhaseManager none(empty, tu::ShippedContactNames());
  EXPECT_EQ(none.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kModelNotInitialised);
}

// ── Per tick ────────────────────────────────────────────────────────────────

// The manager's horizon is the one the scheduler and the builder give when
// driven directly with the same table, configs, times, snapshot and command
// (the shipped walk taken over from stance), byte for byte, tick after tick.
TEST_F(LocomotionPhaseManagerTest, TheHorizonIsTheDirectlyBuiltOne) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);

  rtc::mpc::GaitScheduler scheduler;
  rtc::mpc::StageReferenceBuilder builder;
  rtc::mpc::GaitTable table;
  ASSERT_EQ(
      rtc::mpc::LoadGaitTable(YAML::Load(tu::kShippedGaitYaml), tu::ShippedContactNames(), table),
      rtc::mpc::GaitTableError::kNoError);
  ASSERT_EQ(scheduler.Init(table, tu::kKinoNodes * tu::kKinoDt), GaitSchedulerError::kNoError);
  rtc::mpc::ReferenceConfig ref;
  ASSERT_EQ(rtc::mpc::LoadReferenceConfig(YAML::Load(tu::kShippedReferenceYaml),
                                          tu::ActuatedJointNames(model_), ref),
            ReferenceError::kNoError);
  ASSERT_EQ(builder.Init(handler_, ref, tu::ShippedSwing(), rtc::mpc::FlatSurface(0.0),
                         tu::kKinoNodes + 1, tu::kKinoDt),
            ReferenceError::kNoError);

  const FilteredCommand cmd = WalkCommand();
  m.SetCommand(cmd);
  m.SetPolicyEnabled(false);
  scheduler.SetPolicyEnabled(false);
  rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);

  std::vector<double> times = {16.0, 16.1, 17.0, 18.0, 19.0, 20.0};
  for (int n = 1; n <= 40; ++n) {
    times.push_back(20.0 + n * tu::kKinoDt);
  }
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (i == 1) {
      m.RequestGait(2);
      scheduler.RequestGait(2);
    }
    snap.sequence = 100 + i;
    const PhaseContext ctx = Tick(m, snap, times[i]);
    ASSERT_EQ(scheduler.Advance(times[i], cmd, rtc::mpc::PlanarVelocity{}),
              GaitSchedulerError::kNoError);
    ASSERT_EQ(builder.Build(scheduler, snap, cmd), ReferenceError::kNoError);

    ASSERT_EQ(m.LastUpdateError(), LocomotionPhaseError::kNoError) << "t " << times[i];
    ASSERT_NE(ctx.horizon, nullptr);
    EXPECT_EQ(ctx.horizon, &m.builder().active());
    EXPECT_EQ(std::memcmp(ctx.horizon, &builder.active(), sizeof(rtc::mpc::HorizonReference)), 0)
        << "t " << times[i];
    EXPECT_EQ(ctx.horizon->observation_sequence, snap.sequence);
    EXPECT_EQ(ctx.horizon->command_sequence, cmd.sequence);
    EXPECT_EQ(ctx.ocp_type, "kinodynamic");
    EXPECT_FALSE(ctx.phase_changed);
    EXPECT_EQ(ctx.phase_id, static_cast<int>(scheduler.current_gait()));
    EXPECT_TRUE(ctx.phase_name.empty());
    EXPECT_EQ(ctx.cost_config.horizon_length, tu::kKinoNodes);
    EXPECT_EQ(ctx.cost_config.dt, tu::kKinoDt);
  }
  EXPECT_EQ(m.CurrentPhaseId(), 2);
  EXPECT_EQ(m.CurrentPhaseName(), "walk");
}

// ── Node grid ───────────────────────────────────────────────────────────────

// The tick time is quantised to the node grid anchored at the first tick:
// node 0 sits at the grid node at or before t, within one node interval, and
// every node's time is the grid's own (from the integer index). 1000 random
// tick times (seed 20261114) and the grid nodes themselves, in time order.
TEST_F(LocomotionPhaseManagerTest, TheTickTimeIsQuantisedToTheNodeGrid) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  m.SetCommand(WalkCommand());
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  const double anchor = 3.0;
  const double dt = tu::kKinoDt;
  ASSERT_NE(Tick(m, snap, anchor).horizon, nullptr);
  const rtc::mpc::NodeGrid grid = m.grid();
  EXPECT_EQ(grid.anchor, anchor);
  EXPECT_EQ(grid.dt, dt);

  std::mt19937_64 rng(20261114);
  std::uniform_real_distribution<double> uniform(0.0, 100.0);
  std::uniform_int_distribution<int> node(1, 2800);
  std::vector<double> times;
  for (int i = 0; i < 1000; ++i) {
    times.push_back(anchor + uniform(rng));
    times.push_back(grid.Time(node(rng)));  // a tick exactly on a grid node
  }
  std::sort(times.begin(), times.end());
  int bad = 0;
  for (const double t : times) {
    const PhaseContext ctx = Tick(m, snap, t);
    ASSERT_NE(ctx.horizon, nullptr)
        << "t " << t << " error " << static_cast<int>(m.LastUpdateError());
    ASSERT_TRUE(ctx.on_node_grid);
    const double t0 = ctx.horizon->t0;
    bad += !(t0 == grid.Time(ctx.node_index));
    bad += !(t - t0 >= 0.0 && t - t0 < dt * (1.0 - 1e-12));
    bad += !(ctx.horizon_offset == t - t0);
    bad += !(m.scheduler().schedule().t0 == t0);
    for (std::uint32_t k = 0; k < ctx.horizon->n_stages; ++k) {
      bad += !(ctx.horizon->stages[k].time == grid.Time(ctx.node_index + k));
    }
  }
  EXPECT_EQ(bad, 0);
}

// At 60 Hz (t = n/60 from 0) the horizon moves 0 or 1 node a tick and, over
// ten seconds, by as many nodes as the grid index of the last tick.
TEST_F(LocomotionPhaseManagerTest, AtSixtyHertzTheHorizonMovesZeroOrOneNode) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  m.SetCommand(WalkCommand());
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  std::int64_t prev = 0;
  std::int64_t sum = 0;
  int zeros = 0;
  int ones = 0;
  int other = 0;
  int floor_mismatch = 0;
  int floor_ties = 0;
  for (int n = 0; n <= 600; ++n) {
    const double t = n / 60.0;
    const PhaseContext ctx = Tick(m, snap, t);
    ASSERT_NE(ctx.horizon, nullptr)
        << "n " << n << " error " << static_cast<int>(m.LastUpdateError());
    if (n > 0) {
      const std::int64_t k = ctx.node_index - prev;
      zeros += k == 0;
      ones += k == 1;
      other += k != 0 && k != 1;
      sum += k;
    }
    prev = ctx.node_index;
    // ⌊t / dt⌋ rounds in the division: at a tick exactly on a node's grid time
    // (Time(i) == t) it may say i − 1; the cell is i there.
    if (ctx.node_index != static_cast<std::int64_t>(std::floor(t / tu::kKinoDt))) {
      ++(m.grid().Time(ctx.node_index) == t ? floor_ties : floor_mismatch);
    }
  }
  std::printf(
      "[report] 60 Hz over 10 s: %d ticks moved 0 nodes, %d moved 1, sum %lld; ticks on a node "
      "time where floor(t / dt) rounds one below: %d\n",
      zeros, ones, static_cast<long long>(sum), floor_ties);
  EXPECT_EQ(other, 0);
  EXPECT_EQ(sum, prev);
  EXPECT_EQ(prev, static_cast<std::int64_t>(std::floor(10.0 / tu::kKinoDt)));
  EXPECT_EQ(floor_mismatch, 0) << "the cell is floor(t / dt) except on a node's own time";
}

// The shipped walk ticked on the grid with shifts of 0, 1 and 2 nodes: a node
// index keeps its time and its mode from one tick to the next (the events of
// a gait whose phases are not multiples of dt still fall on some grid nodes;
// computed from the index, such a node can no longer read either side).
TEST_F(LocomotionPhaseManagerTest, AGridNodeKeepsItsTimeAndModeFromTickToTick) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  m.SetCommand(WalkCommand());
  m.SetPolicyEnabled(false);
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  const double dt = tu::kKinoDt;
  ASSERT_NE(Tick(m, snap, 16.0).horizon, nullptr);
  m.RequestGait(2);
  rtc::mpc::HorizonReference prev{};
  std::int64_t prev_index = 0;
  bool have_prev = false;
  std::int64_t index = 1;
  int compared = 0;
  int changed = 0;
  int mode_changes_in_window = 0;
  const int steps[] = {1, 1, 0, 2, 1};
  for (int n = 0; n < 400; ++n) {
    index += steps[n % 5];
    const PhaseContext ctx = Tick(m, snap, m.grid().Time(index) + 0.4 * dt);
    ASSERT_NE(ctx.horizon, nullptr) << "n " << n;
    ASSERT_EQ(ctx.node_index, index);
    const rtc::mpc::HorizonReference& h = *ctx.horizon;
    for (std::uint32_t k = 1; k < h.n_stages; ++k) {
      mode_changes_in_window += h.stages[k].mode != h.stages[k - 1].mode;
    }
    // From the tick after the walk's switch is committed the window only moves.
    if (have_prev && m.scheduler().current_gait() == 2U && !m.scheduler().switch_pending()) {
      const std::int64_t shift = index - prev_index;
      for (std::uint32_t k = 0; k + shift < h.n_stages; ++k) {
        const auto& was = prev.stages[k + static_cast<std::uint32_t>(shift)];
        ++compared;
        changed += was.time != h.stages[k].time || was.mode != h.stages[k].mode;
      }
    }
    std::memcpy(&prev, &h, sizeof(prev));
    prev_index = index;
    have_prev = true;
  }
  std::printf("[report] grid walk: %d node pairs compared, %d mode changes inside windows\n",
              compared, mode_changes_in_window);
  EXPECT_GT(compared, 5000);
  EXPECT_GT(mode_changes_in_window, 0);
  EXPECT_EQ(changed, 0);
}

// What the manager hands over is what the kinodynamic OCP takes: the same
// horizon length and node spacing as its config, a horizon it accepts.
TEST_F(LocomotionPhaseManagerTest, TheKinodynamicOcpAcceptsTheContext) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  m.SetCommand(WalkCommand());
  const PhaseContext ctx = Tick(m, tu::SnapshotAt(model_, stand_), 20.0);
  ASSERT_NE(ctx.horizon, nullptr);
  rtc::mpc::KinodynamicOCPConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(), handler_, cfg),
            rtc::mpc::KinodynamicOCPConfigError::kNoError);
  rtc::mpc::KinodynamicOCP ocp(cfg);
  ASSERT_EQ(ocp.Build(ctx, handler_, {}), rtc::mpc::OCPBuildError::kNoError);
  const PhaseContext next = Tick(m, tu::SnapshotAt(model_, stand_), 20.0 + tu::kKinoDt);
  EXPECT_EQ(ocp.UpdateReferences(next), rtc::mpc::OCPBuildError::kNoError);
}

TEST_F(LocomotionPhaseManagerTest, ATickWithoutAReferenceGivesNoHorizon) {
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  EXPECT_EQ(Tick(m, snap, 1.0).horizon, nullptr) << "before Load";
  EXPECT_EQ(m.LastUpdateError(), LocomotionPhaseError::kNotInitialised);
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);

  // No command yet: the scheduler runs on a command that is not valid, and the
  // builder refuses its zero base height.
  PhaseContext ctx = Tick(m, snap, 1.0);
  EXPECT_EQ(ctx.horizon, nullptr);
  EXPECT_EQ(m.LastUpdateError(), LocomotionPhaseError::kReferenceBuildFailed);
  EXPECT_FALSE(m.command().valid);
  EXPECT_EQ(m.command().sequence, 0U);
  EXPECT_EQ(m.scheduler().schedule().t0, 1.0) << "the scheduler advanced";
  EXPECT_EQ(ctx.ocp_type, "kinodynamic");

  m.SetCommand(WalkCommand());
  ctx = Tick(m, snap, 1.1);
  EXPECT_NE(ctx.horizon, nullptr);
  EXPECT_EQ(m.LastUpdateError(), LocomotionPhaseError::kNoError);

  // Time going back: the scheduler refuses the tick.
  ctx = Tick(m, snap, 1.05);
  EXPECT_EQ(ctx.horizon, nullptr);
  EXPECT_EQ(m.LastUpdateError(), LocomotionPhaseError::kAdvanceFailed);

  // The overload without a snapshot has no provenance to build from.
  rtc::mpc::PhaseManagerBase& base = m;
  ctx = base.Update(Eigen::VectorXd::Zero(model_.nq), Eigen::VectorXd::Zero(model_.nv),
                    Eigen::VectorXd(), pinocchio::SE3::Identity(), 2.0);
  EXPECT_EQ(ctx.horizon, nullptr);
  EXPECT_EQ(m.LastUpdateError(), LocomotionPhaseError::kNoSnapshot);
}

TEST_F(LocomotionPhaseManagerTest, ACommandFromAnotherThreadReachesTheNextUpdate) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  std::thread([&] { m.SetCommand(WalkCommand(7)); }).join();
  const PhaseContext ctx = Tick(m, snap, 3.0);
  ASSERT_NE(ctx.horizon, nullptr);
  EXPECT_EQ(m.command().sequence, 7U);
  EXPECT_TRUE(m.command().valid);
  EXPECT_EQ(ctx.horizon->command_sequence, 7U);

  // A writer storing while the owner ticks: every command read is one whole
  // store (its v_x matches its sequence) and the sequence never goes back.
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    for (std::uint64_t k = 8; !stop.load(std::memory_order_relaxed); ++k) {
      FilteredCommand c = WalkCommand(k);
      c.v_x = 1e-4 * static_cast<double>(k % 4000);
      m.SetCommand(c);
    }
  });
  std::uint64_t last = 7;
  int torn = 0;
  int back = 0;
  for (int i = 0; i < 200; ++i) {
    static_cast<void>(Tick(m, snap, 3.0 + 0.01 * (i + 1)));
    const FilteredCommand& c = m.command();
    // (sequence 7 is the command stored before the writer started)
    torn += c.sequence != 7 && c.v_x != 1e-4 * static_cast<double>(c.sequence % 4000);
    back += c.sequence < last;
    last = c.sequence;
  }
  stop.store(true, std::memory_order_relaxed);
  writer.join();
  EXPECT_EQ(torn, 0);
  EXPECT_EQ(back, 0);
  EXPECT_GT(last, 7U);
}

TEST_F(LocomotionPhaseManagerTest, RequestsReachTheScheduler) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  m.SetCommand(WalkCommand());
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  ASSERT_NE(Tick(m, snap, 5.0).horizon, nullptr);
  EXPECT_TRUE(m.scheduler().policy_enabled());

  m.SetPolicyEnabled(false);
  EXPECT_FALSE(m.scheduler().policy_enabled());
  m.SetPolicyEnabled(true);
  EXPECT_TRUE(m.scheduler().policy_enabled());

  // A forced gait: committed at the next tick (policy off), running after
  // its switch.
  m.ForcePhase(1);
  EXPECT_FALSE(m.scheduler().policy_enabled());
  ASSERT_NE(Tick(m, snap, 5.1).horizon, nullptr);
  EXPECT_EQ(m.scheduler().target_gait(), 1U);
  ASSERT_NE(Tick(m, snap, 8.0).horizon, nullptr);
  EXPECT_EQ(m.CurrentPhaseId(), 1);
  EXPECT_EQ(m.CurrentPhaseName(), "slow_walk");
  m.RequestGait(2);
  ASSERT_NE(Tick(m, snap, 8.1).horizon, nullptr);
  EXPECT_EQ(m.scheduler().target_gait(), 2U);

  // A reset re-anchors gait 0.
  const std::uint64_t epoch = m.scheduler().schedule().epoch;
  m.RequestReset();
  ASSERT_NE(Tick(m, snap, 8.2).horizon, nullptr);
  EXPECT_GT(m.scheduler().schedule().epoch, epoch);
  EXPECT_EQ(m.CurrentPhaseId(), 0);

  m.ForcePhase(-1);  // dropped
  ASSERT_NE(Tick(m, snap, 8.3).horizon, nullptr);
  EXPECT_EQ(m.CurrentPhaseId(), 0);
}

// The scheduler's measured velocity: the LOCAL linear velocity on the
// yaw-aligned axes and the world yaw rate.
TEST(LocomotionPhaseManagerVelocity, TheSnapshotVelocityOnTheYawAlignedAxes) {
  rtc::mpc::MPCStateSnapshot s{};
  s.nq = 7;
  s.nv = 6;
  // A yawed base: the yaw-aligned axes are its own.
  const double psi = 0.3;
  s.q[3] = 0.0;
  s.q[4] = 0.0;
  s.q[5] = std::sin(0.5 * psi);
  s.q[6] = std::cos(0.5 * psi);
  s.v[0] = 0.2;
  s.v[1] = 0.1;
  s.v[2] = 0.05;
  s.v[5] = 0.4;
  rtc::mpc::PlanarVelocity m = LocomotionPhaseManager::MeasuredPlanarVelocity(s);
  EXPECT_NEAR(m.v_x, 0.2, 1e-15);
  EXPECT_NEAR(m.v_y, 0.1, 1e-15);
  EXPECT_NEAR(m.omega_z, 0.4, 1e-15);

  // A pitched base (θ about its y): a forward LOCAL velocity is cos θ forward
  // on the ground, and a LOCAL x rotation adds −sin θ to the world yaw rate.
  const double theta = 0.2;
  s.q[4] = std::sin(0.5 * theta);
  s.q[5] = 0.0;
  s.q[6] = std::cos(0.5 * theta);
  s.v = {};
  s.v[0] = 1.0;
  s.v[3] = 1.0;
  m = LocomotionPhaseManager::MeasuredPlanarVelocity(s);
  EXPECT_NEAR(m.v_x, std::cos(theta), 1e-15);
  EXPECT_NEAR(m.v_y, 0.0, 1e-15);
  EXPECT_NEAR(m.omega_z, -std::sin(theta), 1e-15);

  s.q = {};  // no rotation at all
  m = LocomotionPhaseManager::MeasuredPlanarVelocity(s);
  EXPECT_EQ(m.v_x, 0.0);
  EXPECT_EQ(m.omega_z, 0.0);
}

}  // namespace
