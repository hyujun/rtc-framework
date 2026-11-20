// The estimator replay (controllers/locomotion/estimator_replay.hpp) on the
// installed g1_loco model:
//   - the shared model pieces: BuildBodyModel is the controller's dynamics model
//     bit for bit, and the profile's `urdf` block reads into its ModelConfig;
//   - one tick rebuilt: the receipts from the logged ages and skew;
//   - in-process equivalence: the inputs a log records, handed back unrounded,
//     give the online source's outputs bit for bit, twice over;
//   - the rig's own CSV: a controller run on the inekf source writes its logs
//     through the real writers, and the replay of those files reproduces the
//     logged estimate (discrete telemetry exactly, the base within bounds the
//     log's 9 digits allow).
#include "csv_log_fixture.hpp"
#include "integrated_bringup/controllers/locomotion/estimator_replay.hpp"
#include "integrated_bringup/logging/joint_feedforward_log_pod.hpp"
#include "integrated_bringup/logging/locomotion_diag_log_pod.hpp"
#include "integrated_bringup/support/profile_params.hpp"
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace loco = integrated_bringup::locomotion;
namespace fs = std::filesystem;
using integrated_bringup::LocomotionControllerTestAccess;
using loco::EstimatorReplay;
using loco::EstimatorTelemetry;
using loco::InekfStateSource;
using loco::MeasuredRobotState;
using loco::ReplayInput;
using loco::ReplayTruth;
using rtc::testing::BitsEqual;

constexpr int64_t kT = 2'000'000;  // the shipped control period [ns]

// Every field of two telemetry records, bit for bit (doubles by their bits).
void ExpectSameTelemetry(const EstimatorTelemetry& a, const EstimatorTelemetry& b, int t) {
  EXPECT_EQ(a.initialized, b.initialized) << t;
  EXPECT_EQ(a.reset_reason, b.reset_reason) << t;
  EXPECT_EQ(a.reset_count, b.reset_count) << t;
  EXPECT_EQ(a.imu_sequence, b.imu_sequence) << t;
  EXPECT_EQ(a.periods, b.periods) << t;
  EXPECT_EQ(a.skew_ns, b.skew_ns) << t;
  EXPECT_EQ(a.hold_ticks, b.hold_ticks) << t;
  EXPECT_EQ(a.step_status, b.step_status) << t;
  EXPECT_EQ(a.core_status, b.core_status) << t;
  EXPECT_EQ(a.report_valid, b.report_valid) << t;
  EXPECT_EQ(a.num_slots, b.num_slots) << t;
  EXPECT_EQ(a.planned_mask, b.planned_mask) << t;
  EXPECT_EQ(a.estimated_mask, b.estimated_mask) << t;
  EXPECT_EQ(a.mode, b.mode) << t;
  EXPECT_EQ(a.reason, b.reason) << t;
  EXPECT_EQ(a.update, b.update) << t;
  EXPECT_EQ(a.no_anchor_ticks, b.no_anchor_ticks) << t;
  EXPECT_EQ(a.gated_ticks, b.gated_ticks) << t;
  EXPECT_EQ(a.refused_ticks, b.refused_ticks) << t;
  EXPECT_EQ(a.unobservable, b.unobservable) << t;
  EXPECT_EQ(a.imu_valid, b.imu_valid) << t;
  EXPECT_EQ(a.imu_age_ns, b.imu_age_ns) << t;
  for (std::size_t c = 0; c < a.evidence.size(); ++c) {
    EXPECT_TRUE(BitsEqual(a.evidence[c], b.evidence[c])) << t;
    EXPECT_TRUE(BitsEqual(a.speed[c], b.speed[c])) << t;
    EXPECT_TRUE(BitsEqual(a.m2[c], b.m2[c])) << t;
  }
  for (std::size_t i = 0; i < a.p_diag.size(); ++i) {
    EXPECT_TRUE(BitsEqual(a.p_diag[i], b.p_diag[i])) << t << " p " << i;
  }
  for (std::size_t i = 0; i < a.imu.size(); ++i) {
    EXPECT_TRUE(BitsEqual(a.imu[i], b.imu[i])) << t;
  }
}

class EstimatorReplayTest : public loco_rig::LocomotionControllerTest {
 protected:
  // The replay set up as the estimator_replay tool sets it up, on the rig's
  // configured controller (its model, its estimator section, its roster).
  loco::ReplaySetup Setup(const YAML::Node& cfg) const {
    loco::ReplaySetup s;
    s.params.body_max_age_ns =
        static_cast<int64_t>(cfg["estimator"]["body_max_age_ms"].as<double>() * 1e6);
    s.params.hand_max_age_ns =
        static_cast<int64_t>(cfg["estimator"]["hand_max_age_ms"].as<double>() * 1e6);
    s.params.imu_site = 0;  // model.imu.primary_site is sites[0] (asserted by the tests)
    s.params.config = loco::ParseInekfSourceConfig(cfg["estimator"]["inekf"], "inekf");
    s.model = std::make_shared<const pinocchio::Model>(ctrl_->ReducedModel());
    for (const auto& c : cfg["model"]["contacts"]) {
      s.contact_names.push_back(c["name"].as<std::string>());
      s.contact_frames.push_back(c["frame"].as<std::string>());
    }
    s.roster = Roster();
    return s;
  }

  // The body device's roster: channel c's joint name.
  std::vector<std::string> Roster() const {
    const auto order = loco::JointOrder(ctrl_->FullModel());
    const auto& idx = ctrl_->Indices();
    std::vector<std::string> names;
    for (int c = 0; c < idx.body.num_channels; ++c) {
      names.push_back(order[static_cast<std::size_t>(idx.body.joint[static_cast<std::size_t>(c)])]);
    }
    return names;
  }

  void ExpectPrimaryIsFirstSite() const {
    const YAML::Node imu = cfg_["model"]["imu"];
    ASSERT_EQ(imu["primary_site"].as<std::string>(), imu["sites"][0].as<std::string>());
  }
};

// ── the shared model pieces ─────────────────────────────────────────────────

TEST_F(EstimatorReplayTest, TheBodyModelIsTheControllersDynamicsModelBitForBit) {
  Configure(loco_rig::ShippedConfig());
  const auto builder = integrated_bringup::testfx::ShippedModelBuilder(loco_rig::kProfile);
  const auto full = builder->GetActuatedModel();
  ASSERT_TRUE(full);
  const auto tree = builder->GetTreeModel("g1");
  ASSERT_TRUE(tree);
  const loco::BodyModel body = loco::BuildBodyModel(*full, *tree);
  const pinocchio::Model& ctrl = ctrl_->ReducedModel();
  EXPECT_TRUE(body.locked) << "the hand is locked";
  EXPECT_EQ(loco::JointOrder(ctrl), body.joint_names);
  EXPECT_EQ(LocomotionControllerTestAccess::ReducedToFull(*ctrl_), body.reduced_to_full);
  ASSERT_EQ(ctrl.nq, body.reduced->nq);
  ASSERT_EQ(ctrl.nv, body.reduced->nv);
  ASSERT_EQ(ctrl.nframes, body.reduced->nframes);
  for (int f = 0; f < ctrl.nframes; ++f) {
    const auto& a = ctrl.frames[static_cast<std::size_t>(f)];
    const auto& b = body.reduced->frames[static_cast<std::size_t>(f)];
    EXPECT_EQ(a.name, b.name);
    for (int i = 0; i < 3; ++i) {
      EXPECT_TRUE(BitsEqual(a.placement.translation()(i), b.placement.translation()(i))) << a.name;
      for (int j = 0; j < 3; ++j) {
        EXPECT_TRUE(BitsEqual(a.placement.rotation()(i, j), b.placement.rotation()(i, j)))
            << a.name;
      }
    }
  }
  for (std::size_t j = 0; j < ctrl.inertias.size(); ++j) {
    EXPECT_TRUE(BitsEqual(ctrl.inertias[j].mass(), body.reduced->inertias[j].mass())) << j;
  }
}

TEST(ProfileParams, TheUrdfBlockReadsIntoTheModelConfigAndRefusesMalformedOnes) {
  const std::string dir = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/g1_loco/";
  const YAML::Node params =
      integrated_bringup::LoadProfileParams({dir + "_base.yaml", dir + "sim.yaml"});
  const YAML::Node urdf = params["urdf"];
  ASSERT_TRUE(urdf && urdf.IsMap());
  const auto cfg = integrated_bringup::ModelConfigFromUrdfParams(urdf, "/u", "/c");
  EXPECT_EQ("/u", cfg.urdf_path);
  EXPECT_EQ("floating", cfg.root_joint_type);
  EXPECT_EQ("/c", cfg.closure_yaml_path) << "g1_loco is an extended model";
  ASSERT_EQ(2U, cfg.tree_models.size());
  EXPECT_EQ("g1", cfg.tree_models[0].name);
  EXPECT_EQ("pelvis", cfg.tree_models[0].root_link);
  EXPECT_EQ(4U, cfg.tree_models[0].tip_links.size());
  EXPECT_TRUE(cfg.sub_models.empty());

  EXPECT_THROW(static_cast<void>(integrated_bringup::ModelConfigFromUrdfParams(urdf, "/u", "")),
               std::invalid_argument)
      << "an extended model without its sidecar";
  EXPECT_THROW(
      static_cast<void>(integrated_bringup::ModelConfigFromUrdfParams(YAML::Node(1), "/u", "")),
      std::invalid_argument);
  YAML::Node bad = YAML::Clone(urdf);
  bad["extended"] = false;
  bad["extra_frames"]["f"]["parent"] = "pelvis";
  bad["extra_frames"]["f"]["xyz"] = std::vector<double>{0.0, 0.0};
  EXPECT_THROW(static_cast<void>(integrated_bringup::ModelConfigFromUrdfParams(bad, "/u", "")),
               std::invalid_argument);
  bad["extra_frames"]["f"]["xyz"] = std::vector<double>{0.1, 0.0, 0.2};
  const auto ok = integrated_bringup::ModelConfigFromUrdfParams(bad, "/u", "");
  ASSERT_EQ(1U, ok.extra_frames.size());
  EXPECT_EQ(0.2, ok.extra_frames[0].xyz.z());
  EXPECT_TRUE(ok.extra_frames[0].provisional);
  EXPECT_TRUE(ok.closure_yaml_path.empty());
  bad["tree_models"] = 3;
  EXPECT_THROW(static_cast<void>(integrated_bringup::ModelConfigFromUrdfParams(bad, "/u", "")),
               std::invalid_argument);
}

// ── one tick rebuilt ────────────────────────────────────────────────────────

TEST(RebuildState, TheReceiptsComeFromTheAgesAndTheSkew) {
  ReplayInput in;
  in.tick = 7;
  in.num_joints = 2;
  in.q = {0.1, 0.2};
  in.qd = {1.0, 2.0};
  in.imu = {0.01, 0.02, 0.03, 0.1, 0.2, 9.8};
  in.imu_valid = true;
  in.imu_sequence = 41;
  in.imu_age_us = 250.5;
  in.skew_us = -120.25;
  in.source_age_us = 999.0;  // not read when the IMU gives the receipt
  rtc::ControllerState s;
  constexpr int64_t kNow = 5'000'000'000;
  EXPECT_TRUE(loco::RebuildState(in, 1, kNow, s));
  const auto& d = s.devices[0];
  EXPECT_EQ(1, s.num_devices);
  EXPECT_EQ(7U, s.iteration);
  EXPECT_TRUE(d.valid);
  EXPECT_EQ(2, d.num_channels);
  EXPECT_EQ(0.2, d.positions[1]);
  EXPECT_EQ(2.0, d.velocities[1]);
  EXPECT_EQ(2, d.num_imu_sites);
  EXPECT_FALSE(d.imu_valid[0]);
  EXPECT_TRUE(d.imu_valid[1]);
  EXPECT_EQ(41U, d.imu_sequence[1]);
  EXPECT_EQ(9.8, d.imu[1][5]);
  EXPECT_EQ(kNow - 250'500, d.imu_recv_steady_ns[1]);
  EXPECT_EQ(kNow - 250'500 - 120'250, d.joint_recv_steady_ns);

  in.imu_valid = false;  // the joint receipt from the state's own age
  EXPECT_FALSE(loco::RebuildState(in, 1, kNow, s));
  EXPECT_EQ(kNow - 999'000, s.devices[0].joint_recv_steady_ns);
  in.source_age_us = -3.0;  // stamped after the clock read
  EXPECT_FALSE(loco::RebuildState(in, 1, kNow, s));
  EXPECT_EQ(kNow + 3'000, s.devices[0].joint_recv_steady_ns);
  in.source_age_us = -1.0;  // no receipt
  in.imu_age_us = -1.0;
  EXPECT_FALSE(loco::RebuildState(in, 1, kNow, s));
  EXPECT_EQ(0, s.devices[0].joint_recv_steady_ns);
  EXPECT_EQ(0, s.devices[0].imu_recv_steady_ns[1]);
  in.source_status = static_cast<uint8_t>(loco::StateSourceStatus::kBodyUnreadable);
  static_cast<void>(loco::RebuildState(in, 1, kNow, s));
  EXPECT_FALSE(s.devices[0].valid);
}

// ── reading the logs ────────────────────────────────────────────────────────
//
// Files written by the real writers: two sims in one folder (tick restarts),
// a joint_ff row lost (an unmatched diag row), a diag row lost, a CRLF blank
// line — the joins, the counts and the kept run.

void WriteLogs(const fs::path& dir, bool second_joint_run) {
  const std::vector<std::string> contacts{"l", "r"};
  const std::vector<std::string> joints{"j0", "j1"};
  std::ofstream diag(dir / "locomotion_diag.csv");
  std::ofstream ff(dir / "joint_ff.csv");
  const auto dcols = integrated_bringup::LocomotionDiagLogColumnsFor(contacts, joints);
  const auto fcols = integrated_bringup::JointFeedforwardLogColumnsFor(joints);
  integrated_bringup::WriteLocomotionDiagLogHeader(diag, contacts, joints, dcols);
  diag << '\n';
  integrated_bringup::WriteJointFeedforwardLogHeader(ff, joints, fcols);
  ff << "\r\n\r\n";  // CRLF, and a blank line
  const auto run = [&](uint64_t first, bool with_joints) {
    for (uint64_t k = 0; k < 10; ++k) {
      const uint64_t tick = first + k;
      integrated_bringup::LocomotionDiagLogPod d{};
      d.tick = tick;
      d.tick_count = k + 1;
      d.num_joints = 2;
      d.v = {};
      d.v[0] = 0.5;
      if (!(first == 5 && tick == 12)) {  // a diag row lost
        integrated_bringup::WriteLocomotionDiagLogRow(diag, d, dcols);
        diag << '\n';
      }
      if (with_joints && !(first == 5 && tick == 8)) {  // a joint_ff row lost
        integrated_bringup::JointFeedforwardLogPod j{};
        j.tick = tick;
        j.num_joints = 2;
        j.q[0] = 0.01 * static_cast<double>(tick);
        j.q[1] = -1.0;
        integrated_bringup::WriteJointFeedforwardLogRow(ff, j, fcols);
        ff << '\n';
      }
    }
  };
  run(10, true);
  run(5, second_joint_run);
}

TEST(AddJointNoise, NoisesTheJointPositionsOnlyWithSigmaAndDrawsNothingAtZero) {
  ReplayInput clean;
  clean.tick = 9;
  clean.num_joints = 3;
  for (std::size_t j = 0; j < 5; ++j) {
    clean.q[j] = 0.1 * static_cast<double>(j);
    clean.qd[j] = 0.2;
  }
  clean.imu[0] = 0.3;
  std::mt19937_64 rng(11);
  double sum2 = 0.0;
  constexpr int kTicks = 4000;
  for (int k = 0; k < kTicks; ++k) {
    ReplayInput in = clean;
    loco::AddJointNoise(in, 1e-3, rng);
    for (std::size_t j = 0; j < 5; ++j) {
      ASSERT_EQ(in.qd[j], clean.qd[j]) << "velocities stay";
      if (j >= 3) {
        ASSERT_EQ(in.q[j], clean.q[j]) << "past num_joints stays";
      } else {
        sum2 += (in.q[j] - clean.q[j]) * (in.q[j] - clean.q[j]);
      }
    }
    ASSERT_EQ(in.imu, clean.imu);
    ASSERT_EQ(in.tick, clean.tick);
  }
  EXPECT_NEAR(std::sqrt(sum2 / (3.0 * kTicks)), 1e-3, 5e-5);

  std::mt19937_64 a(5);
  std::mt19937_64 b(5);
  ReplayInput zero = clean;
  loco::AddJointNoise(zero, 0.0, a);
  EXPECT_EQ(0, std::memcmp(zero.q.data(), clean.q.data(), sizeof(clean.q)));
  EXPECT_EQ(a(), b()) << "σ = 0 draws nothing";
}

TEST(LoadReplayLog, JoinsTheLogsPerRunAndCountsWhatIsMissing) {
  const fs::path dir = fs::path(::testing::TempDir()) / "replay_log_runs";
  fs::create_directories(dir);
  WriteLogs(dir, true);
  const auto diag = (dir / "locomotion_diag.csv").string();
  const auto ff = (dir / "joint_ff.csv").string();

  const loco::ReplayLog last = loco::LoadReplayLog(diag, ff, "", {});
  EXPECT_EQ((std::vector<std::string>{"j0", "j1"}), last.roster);
  EXPECT_EQ(2U, last.runs);
  EXPECT_EQ(1, last.run);
  EXPECT_EQ(19U, last.diag_rows);
  EXPECT_EQ(1U, last.unmatched);
  ASSERT_EQ(8U, last.ticks.size());  // ticks 5 … 14 without 8 (no joints) and 12 (no diag row)
  EXPECT_EQ(5U, last.ticks.front().tick);
  EXPECT_EQ(1U, last.ticks.front().tick_count);
  EXPECT_EQ(0U, last.ticks.front().gap);
  EXPECT_EQ(2U, last.gaps);
  EXPECT_EQ(2U, last.missing_ticks);
  EXPECT_EQ(9U, last.ticks[3].tick);
  EXPECT_EQ(1U, last.ticks[3].gap);
  EXPECT_DOUBLE_EQ(0.09, last.ticks[3].q[0]);
  EXPECT_EQ(-1.0, last.ticks[3].q[1]);
  EXPECT_EQ(0.5, last.ticks[3].qd[0]);

  const loco::ReplayLog first = loco::LoadReplayLog(diag, ff, "", {}, 0);
  ASSERT_EQ(10U, first.ticks.size());
  EXPECT_EQ(10U, first.ticks.front().tick);
  EXPECT_EQ(0U, first.gaps);
  const loco::ReplayLog all = loco::LoadReplayLog(diag, ff, "", {}, loco::kAllRuns);
  EXPECT_EQ(18U, all.ticks.size());
  EXPECT_THROW(static_cast<void>(loco::LoadReplayLog(diag, ff, "", {}, 2)), std::runtime_error);

  WriteLogs(dir, false);  // the joint log of the second sim is missing
  EXPECT_THROW(static_cast<void>(loco::LoadReplayLog(diag, ff, "", {})), std::runtime_error);
}

// ── in-process equivalence (i) ──────────────────────────────────────────────
//
// The online source steps a synthetic run; each tick is recorded the way the
// logs record it — the same fields, unrounded — and the replay of those
// records gives the online outputs bit for bit. Every path the rebuild has is
// in it: a held sample, missed samples, an invalid IMU, the joints before and
// after the IMU, the prior switching.

TEST_F(EstimatorReplayTest, TheReplayOfTheRecordedInputsIsTheOnlineSourceBitForBit) {
  Configure(loco_rig::ShippedConfig());
  ExpectPrimaryIsFirstSite();
  MakeStanding();
  const loco::ReplaySetup setup = Setup(cfg_);
  InekfStateSource online(setup.params,
                          std::make_unique<loco::ImuFrameKinematics>(
                              setup.model, setup.contact_frames, setup.params.config.imu_frame),
                          LocomotionControllerTestAccess::ReducedToFull(*ctrl_));
  online.Reset();  // the activation's

  auto& body = state_.devices[loco_rig::kBody];
  const Eigen::Vector3d g(body.imu[0][3], body.imu[0][4], body.imu[0][5]);
  std::mt19937_64 rng(20261120U);
  std::normal_distribution<double> n(0.0, 1.0);
  int64_t now = 1'000'000'000'000;
  std::vector<ReplayInput> inputs;
  std::vector<MeasuredRobotState> outs;
  const auto nc = static_cast<std::size_t>(body.num_channels);
  int valid = 0;
  for (int t = 0; t < 2000; ++t) {
    now += kT;
    body.imu[0] = {5e-4 * n(rng),         5e-4 * n(rng),         5e-4 * n(rng),
                   g.x() + 1e-2 * n(rng), g.y() + 1e-2 * n(rng), g.z() + 1e-2 * n(rng)};
    for (std::size_t c = 0; c < nc; ++c) {
      body.positions[c] += 1e-5 * n(rng);
      body.velocities[c] = 1e-3 * n(rng);
    }
    const uint64_t k = (t % 97 == 50) ? 0 : (t % 89 == 40 ? 3 : 1);
    body.imu_sequence[0] += k;
    body.imu_valid[0] = t % 211 != 100;
    body.imu_recv_steady_ns[0] = now;
    const int64_t joint_after = t % 53 == 7 ? 300'000 : (t % 59 == 9 ? -250'000 : 0);
    body.joint_recv_steady_ns = now + joint_after;
    state_.devices[loco_rig::kHand].joint_recv_steady_ns = now;
    const int64_t read_at = now + std::max<int64_t>(0, joint_after);
    const uint32_t planned = (t / 300) % 2 == 0 ? 0b11U : 0b01U;
    MeasuredRobotState out{};
    valid += online.Update(state_, ctrl_->Indices(), {planned}, read_at, out) ? 1 : 0;
    outs.push_back(out);

    // What the diagnostic and joint_ff rows hold, unrounded.
    ReplayInput in;
    in.tick = static_cast<uint64_t>(t + 1);
    in.tick_count = static_cast<uint64_t>(t + 1);
    in.source_status = static_cast<uint8_t>(out.status);
    in.joint_sequence = out.sequence;
    in.source_age_us =
        out.stamp_steady_ns > 0 ? static_cast<double>(read_at - out.stamp_steady_ns) * 1e-3 : -1.0;
    in.num_joints = static_cast<int>(nc);
    for (std::size_t c = 0; c < nc; ++c) {
      in.q[c] = body.positions[c];
      in.qd[c] = body.velocities[c];
    }
    const auto& e = out.estimator;
    in.imu = e.imu;
    in.imu_valid = e.imu_valid;
    in.imu_sequence = e.imu_sequence;
    in.imu_age_us = e.imu_age_ns < 0 ? -1.0 : static_cast<double>(e.imu_age_ns) * 1e-3;
    in.skew_us = static_cast<double>(e.skew_ns) * 1e-3;
    in.planned_mask = e.planned_mask;
    inputs.push_back(in);
  }
  EXPECT_GT(valid, 1000) << "the estimate ran";

  const auto replay_all = [&](std::vector<MeasuredRobotState>& got) {
    EstimatorReplay replay(setup);
    got.clear();
    for (const auto& in : inputs) {
      got.push_back(replay.Step(in, nullptr));
      EXPECT_TRUE(replay.last_joint_exact() || !in.imu_valid);
    }
  };
  std::vector<MeasuredRobotState> a;
  replay_all(a);
  const auto& to_full = LocomotionControllerTestAccess::ReducedToFull(*ctrl_);
  for (std::size_t t = 0; t < outs.size(); ++t) {
    const auto& want = outs[t];
    const auto& got = a[t];
    const int ti = static_cast<int>(t);
    ASSERT_EQ(want.valid, got.valid) << t;
    EXPECT_EQ(want.status, got.status) << t;
    EXPECT_EQ(want.state_epoch, got.state_epoch) << t;
    EXPECT_EQ(want.contact_mask, got.contact_mask) << t;
    EXPECT_EQ(want.fresh_mask & (loco::kBodyJointsBit | loco::kBaseBit),
              got.fresh_mask & (loco::kBodyJointsBit | loco::kBaseBit))
        << t;
    EXPECT_EQ(want.valid_mask & (loco::kBodyJointsBit | loco::kBaseBit),
              got.valid_mask & (loco::kBodyJointsBit | loco::kBaseBit))
        << t;
    for (std::size_t i = 0; i < 7; ++i) {
      EXPECT_TRUE(BitsEqual(want.q[i], got.q[i])) << t << " q " << i;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(want.v[i], got.v[i])) << t << " v " << i;
    }
    for (std::size_t j = 0; j < to_full.size(); ++j) {
      const auto a_full = static_cast<std::size_t>(to_full[j]);
      EXPECT_TRUE(BitsEqual(want.q[7 + a_full], got.q[7 + j])) << t;
    }
    ExpectSameTelemetry(want.estimator, got.estimator, ti);
    if (::testing::Test::HasFailure()) {
      FAIL() << "first difference at tick " << t;
    }
  }
  std::vector<MeasuredRobotState> b;
  replay_all(b);
  for (std::size_t t = 0; t < a.size(); ++t) {
    ExpectSameTelemetry(a[t].estimator, b[t].estimator, static_cast<int>(t));
    for (std::size_t i = 0; i < 7; ++i) {
      ASSERT_TRUE(BitsEqual(a[t].q[i], b[t].q[i])) << t;
    }
  }

  // Joint noise (WD-333) reaches the source's input only: the truth contact
  // velocity is the recorded joints', bit for bit; the estimate moves; the
  // same seed replays the same estimate.
  ReplayTruth truth;
  truth.base_valid = true;
  truth.base_q = {0.0, 0.0, 0.75, 0.0, 0.0, 0.0, 1.0};
  truth.base_v = {0.1, -0.05, 0.0, 0.0, 0.0, 0.2};
  auto noisy_setup = setup;
  noisy_setup.joint_noise_sigma = 1e-3;
  noisy_setup.joint_noise_seed = 3;
  EstimatorReplay clean(setup);
  EstimatorReplay noisy(noisy_setup);
  EstimatorReplay again(noisy_setup);
  bool moved = false;
  for (const auto& in : inputs) {
    const MeasuredRobotState c = clean.Step(in, &truth);
    const MeasuredRobotState n = noisy.Step(in, &truth);
    const MeasuredRobotState r = again.Step(in, &truth);
    for (std::size_t k = 0; k < 2; ++k) {
      for (std::size_t i = 0; i < 3; ++i) {
        ASSERT_TRUE(
            BitsEqual(clean.truth_contact_velocity()[k][i], noisy.truth_contact_velocity()[k][i]))
            << in.tick;
      }
    }
    for (std::size_t i = 0; i < 7; ++i) {
      ASSERT_TRUE(BitsEqual(n.q[i], r.q[i])) << in.tick;
      moved = moved || !BitsEqual(n.q[i], c.q[i]);
    }
  }
  EXPECT_TRUE(std::isfinite(clean.truth_contact_velocity()[0][0])) << "the truth path ran";
  EXPECT_TRUE(moved) << "the noise never reached the estimate";
  for (const double bad : {-1e-3, std::nan("")}) {
    auto bad_setup = setup;
    bad_setup.joint_noise_sigma = bad;
    EXPECT_THROW(EstimatorReplay{bad_setup}, std::invalid_argument) << bad;
  }
}

// ── the rig's own CSV (B2) ──────────────────────────────────────────────────

std::string Sci(double x) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.3e", x);
  return buf.data();
}

double QuatAngle(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  const Eigen::Quaterniond qa(a[6], a[3], a[4], a[5]);
  const Eigen::Quaterniond qb(b[6], b[3], b[4], b[5]);
  return qa.normalized().angularDistance(qb.normalized());
}

class EstimatorReplayCsvTest : public EstimatorReplayTest {
 protected:
  rtc::testing::ScopedSessionDir session_;
};

TEST_F(EstimatorReplayCsvTest, TheReplayOfTheRigsLogsReproducesTheLoggedEstimate) {
  YAML::Node cfg = loco_rig::ShippedConfig();
  cfg["estimator"]["source"] = "inekf";
  ConfigureWithDevices(cfg, loco_rig::ShippedLaneDevices(cfg));
  ExpectPrimaryIsFirstSite();
  Activate();
  MakeStanding();
  auto& body = state_.devices[loco_rig::kBody];
  const std::array<double, 6> still = body.imu[0];
  const std::vector<double> stand = loco_rig::StandPose();
  std::mt19937_64 rng(20261121U);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int t = 0; t < 600; ++t) {
    for (std::size_t i = 0; i < 6; ++i) {
      body.imu[0][i] = still[i] + (i < 3 ? 5e-4 : 1e-2) * n(rng);
    }
    for (std::size_t c = 0; c < stand.size(); ++c) {
      body.positions[c] = stand[c] + 1e-4 * n(rng);
      body.velocities[c] = 1e-3 * n(rng);
    }
    last_out_ = Tick();
    if (t % 128 == 127) {
      LocomotionControllerTestAccess::DrainLogs(*ctrl_);  // the ring holds fewer rows
    }
  }
  LocomotionControllerTestAccess::DrainLogs(*ctrl_);
  ASSERT_EQ(LocomotionControllerTestAccess::LogDrops(*ctrl_), 0U);
  const fs::path diag = LocomotionControllerTestAccess::LogPath(
      *ctrl_, integrated_bringup::kLocomotionDiagLogInstance);
  const fs::path jff = LocomotionControllerTestAccess::LogPath(
      *ctrl_, integrated_bringup::kJointFeedforwardLogInstance);

  const loco::ReplayLog log = loco::LoadReplayLog(diag.string(), jff.string(), "", {});
  EXPECT_EQ(Roster(), log.roster);
  EXPECT_EQ(600U, log.diag_rows);
  EXPECT_EQ(600U, log.ticks.size());
  EXPECT_EQ(0U, log.unmatched);
  EXPECT_EQ(0U, log.gaps);
  EXPECT_EQ(1U, log.runs);

  EstimatorReplay replay(Setup(cfg));
  int started = 0;
  int stance = 0;
  double worst_p = 0.0;
  double worst_r = 0.0;
  double worst_v = 0.0;
  double worst_w = 0.0;
  for (const auto& in : log.ticks) {
    const MeasuredRobotState& out = replay.Step(in, nullptr);
    const auto& e = out.estimator;
    const auto& o = in.online;
    const auto t = in.tick;
    ASSERT_EQ(o.initialized, e.initialized) << t;
    ASSERT_EQ(o.reset_count, e.reset_count) << t;
    ASSERT_EQ(o.reset_reason, e.reset_reason) << t;
    ASSERT_EQ(o.periods, e.periods) << t;
    ASSERT_EQ(o.hold_ticks, e.hold_ticks) << t;
    ASSERT_EQ(o.step_status, e.step_status) << t;
    ASSERT_EQ(o.report_valid, e.report_valid) << t;
    ASSERT_EQ(o.estimated_mask, e.estimated_mask) << t;
    for (std::size_t c = 0; c < 2; ++c) {
      ASSERT_EQ(o.mode[c], e.mode[c]) << t;
      ASSERT_EQ(o.update[c], e.update[c]) << t;
    }
    if (!e.initialized) {
      continue;
    }
    ++started;
    stance += e.estimated_mask == 0b11U ? 1 : 0;
    std::array<double, 7> q{};
    std::copy(out.q.begin(), out.q.begin() + 7, q.begin());
    double dp = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      dp = std::max(dp, std::abs(q[i] - o.base_q[i]));
      worst_v = std::max(worst_v, std::abs(out.v[i] - o.base_v[i]));
      worst_w = std::max(worst_w, std::abs(out.v[3 + i] - o.base_v[3 + i]));
    }
    worst_p = std::max(worst_p, dp);
    worst_r = std::max(worst_r, QuatAngle(q, o.base_q));
  }
  EXPECT_GT(started, 300) << "the estimate ran for most of the run";
  EXPECT_GT(stance, 200) << "both soles reached stance";
  RecordProperty("worst_p_m", Sci(worst_p));
  RecordProperty("worst_rot_rad", Sci(worst_r));
  RecordProperty("worst_v_mps", Sci(worst_v));
  RecordProperty("worst_w_radps", Sci(worst_w));
  EXPECT_LE(worst_p, 1e-6);
  EXPECT_LE(worst_r, 1e-6);
  EXPECT_LE(worst_v, 1e-6);
  EXPECT_LE(worst_w, 1e-6);
}

}  // namespace
