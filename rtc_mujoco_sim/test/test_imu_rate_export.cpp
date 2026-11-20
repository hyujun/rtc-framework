// ── test_imu_rate_export.cpp ─────────────────────────────────────────────────
// robot_response.<group>.imu_rate_export inside the simulator. The reference
// for a tick-mean sample is built from the frame sensors on the IMU site in
// the same exports (framequat, framelinvel — raw sensordata, of the instant
// the snapshot is taken at), differenced over the step:
//   sample (default) — the export is mjData::sensordata, bit for bit
//   tick_mean        — StepForTest (one substep) and the SimLoop with
//                      n_substeps 3 both match the reference, cutoff applied;
//                      sensordata, the other export entries and the contact
//                      lane are bit-identical to a sample run
//   raw fallbacks    — the first two exports, the first two after a reset, a
//                      second read without a step, RK4 and the pair across a
//                      world-gravity switch are sensordata
//   noise            — the same draws as in a sample run; reset replays
// ──────────────────────────────────────────────────────────────────────────────
#include "sim_config_fixture.hpp"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef IMU_RATE_MJCF_PATH
#error "IMU_RATE_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

using namespace std::chrono_literals;

constexpr int kSteps = 400;
constexpr double kTol = 1e-9;
constexpr double kAccelCutoff = 14.0;  // imu_rate_scene.xml
constexpr double kGravity[3] = {0.0, 0.0, -9.81};

ImuNoiseConfig Noise(bool enabled) {
  ImuNoiseConfig n;
  n.enabled = enabled;
  n.seed = 20261114;
  n.gyro = {5e-4, 1e-3, 1e-5};
  n.accel = {1e-2, 5e-2, 1e-4};
  return n;
}

MuJoCoSimulator::Config SceneConfig(ImuRateExport mode, bool noise = false) {
  auto cfg = test::HeadlessConfig(IMU_RATE_MJCF_PATH, 10.0);
  cfg.reset_keyframe = "start";
  auto g = test::ContactWrenchGroup("arm", {"j1"}, "/arm");
  g.initial_qpos = {0.0};
  g.imu_noise = Noise(noise);
  g.imu_rate_export = mode;
  cfg.groups.push_back(std::move(g));
  return cfg;
}

struct Trace {
  std::vector<std::vector<double>> exported;
  std::vector<std::vector<double>> raw;
  std::vector<std::vector<double>> wrench;
};

void Step(MuJoCoSimulator& sim, Trace& t) {
  sim.StepForTest();
  t.exported.push_back(sim.GetSensorExportForTest(0));
  t.raw.push_back(sim.GetSensorDataForTest(0));
  std::vector<double> w;
  for (const auto& s : sim.GetContactWrenchSamplesForTest(0)) {
    w.push_back(s.found ? 1.0 : 0.0);
    w.insert(w.end(), s.force.begin(), s.force.end());
    w.insert(w.end(), s.torque.begin(), s.torque.end());
  }
  t.wrench.push_back(std::move(w));
}

Trace Simulate(MuJoCoSimulator& sim, int steps) {
  Trace t;
  for (int k = 0; k < steps; ++k)
    Step(sim, t);
  return t;
}

bool BitEqual(const std::vector<double>& a, const std::vector<double>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

/// Export offsets of the scene's sensors, by name.
struct Layout {
  std::size_t gyro{0}, accel{0}, quat{0}, linvel{0};
  std::vector<bool> imu;  // per export entry: a gyro / accelerometer entry
};

Layout LayoutOf(const MuJoCoSimulator& sim) {
  Layout l;
  std::size_t off = 0;
  for (const auto& si : sim.GetSensorInfos(0)) {
    if (si.name == "imu_gyro")
      l.gyro = off;
    if (si.name == "imu_accel")
      l.accel = off;
    if (si.name == "imu_quat")
      l.quat = off;
    if (si.name == "imu_linvel")
      l.linvel = off;
    const bool imu = si.type == mjSENS_GYRO || si.type == mjSENS_ACCELEROMETER;
    l.imu.insert(l.imu.end(), static_cast<std::size_t>(si.dim), imu);
    off += static_cast<std::size_t>(si.dim);
  }
  return l;
}

/// The IMU site at the instant of one export, from its frame sensors.
ImuSiteSnapshot SnapshotOf(const std::vector<double>& e, const Layout& l, double time) {
  ImuSiteSnapshot s;
  double q[4];
  std::copy_n(e.begin() + static_cast<std::ptrdiff_t>(l.quat), 4, q);
  mju_quat2Mat(s.rot.data(), q);
  std::copy_n(e.begin() + static_cast<std::ptrdiff_t>(l.linvel), 3, s.vel.begin());
  s.time = time;
  return s;
}

struct Check {
  int compared{0};
  int clamped{0};
  double max_gyro{0.0};
};

/// Every export from `first` on against the frame-sensor difference over `dt`.
void ExpectReference(const std::vector<std::vector<double>>& exports, const Layout& l, double dt,
                     std::size_t first, Check& c, const double* gravity = kGravity) {
  for (std::size_t k = first; k < exports.size(); ++k) {
    const auto prev = SnapshotOf(exports[k - 1], l, dt * static_cast<double>(k - 1));
    const auto cur = SnapshotOf(exports[k], l, dt * static_cast<double>(k));
    double gyro[3];
    double accel[3];
    ASSERT_TRUE(TickMeanGyro(prev, cur, gyro));
    ASSERT_TRUE(TickMeanAccel(prev, cur, gravity, accel));
    for (std::size_t a = 0; a < 3; ++a) {
      if (std::abs(accel[a]) > kAccelCutoff)
        ++c.clamped;
      accel[a] = std::clamp(accel[a], -kAccelCutoff, kAccelCutoff);
      c.max_gyro = std::max(c.max_gyro, std::abs(gyro[a]));
      ASSERT_NEAR(exports[k][l.gyro + a], gyro[a], kTol) << "export " << k << " gyro " << a;
      ASSERT_NEAR(exports[k][l.accel + a], accel[a], kTol) << "export " << k << " accel " << a;
    }
    ++c.compared;
  }
}

TEST(ImuRateExport, SampleIsSensordataBitForBit) {
  MuJoCoSimulator sim(SceneConfig(ImuRateExport::kSample));
  ASSERT_TRUE(sim.Initialize());
  const auto t = Simulate(sim, kSteps);
  for (std::size_t k = 0; k < t.exported.size(); ++k)
    ASSERT_TRUE(BitEqual(t.exported[k], t.raw[k])) << "step " << k;
}

TEST(ImuRateExport, OneSubstepStepsMatchTheFrameSensorDifference) {
  MuJoCoSimulator sim(SceneConfig(ImuRateExport::kTickMean));
  ASSERT_TRUE(sim.Initialize());
  const auto l = LayoutOf(sim);
  const auto t = Simulate(sim, kSteps);
  ASSERT_TRUE(BitEqual(t.exported[0], t.raw[0])) << "one snapshot only: raw";
  for (std::size_t k = 0; k < t.exported.size(); ++k) {
    for (std::size_t i = 0; i < l.imu.size(); ++i) {
      if (!l.imu[i]) {
        ASSERT_EQ(std::memcmp(&t.exported[k][i], &t.raw[k][i], sizeof(double)), 0)
            << "non-IMU entry " << i << ", step " << k;
      }
    }
  }
  Check c;
  ExpectReference(t.exported, l, 0.002, 1, c);
  EXPECT_EQ(c.compared, kSteps - 1);
  EXPECT_GT(c.max_gyro, 1.0) << "the site barely turns — the comparison is weak";
}

TEST(ImuRateExport, TickMeanLeavesPhysicsTheOtherEntriesAndTheContactLaneAlone) {
  MuJoCoSimulator mean(SceneConfig(ImuRateExport::kTickMean, true));
  MuJoCoSimulator sample(SceneConfig(ImuRateExport::kSample, true));
  ASSERT_TRUE(mean.Initialize());
  ASSERT_TRUE(sample.Initialize());
  ASSERT_TRUE(mean.HasContactWrenches(0));
  const auto l = LayoutOf(mean);
  const auto tm = Simulate(mean, kSteps);
  const auto ts = Simulate(sample, kSteps);
  bool touched = false;
  for (std::size_t k = 0; k < tm.raw.size(); ++k) {
    ASSERT_TRUE(BitEqual(tm.raw[k], ts.raw[k])) << "sensordata, step " << k;
    ASSERT_TRUE(BitEqual(tm.wrench[k], ts.wrench[k])) << "contact wrench, step " << k;
    for (std::size_t i = 0; i < l.imu.size(); ++i) {
      if (!l.imu[i]) {
        ASSERT_EQ(std::memcmp(&tm.exported[k][i], &ts.exported[k][i], sizeof(double)), 0)
            << "non-IMU entry " << i << ", step " << k;
      }
    }
    touched = touched || (!tm.wrench[k].empty() && tm.wrench[k][0] != 0.0);
  }
  EXPECT_TRUE(touched) << "the tip never touched the floor — the lane check is vacuous";
}

TEST(ImuRateExport, NoiseIsTheSameDrawsAsInASampleRun) {
  // The noise is export(noise on) − export(noise off) of the same mode; the
  // two modes noise values of different size, so the comparison has a
  // rounding tolerance rather than bit equality.
  MuJoCoSimulator mean_on(SceneConfig(ImuRateExport::kTickMean, true));
  MuJoCoSimulator mean_off(SceneConfig(ImuRateExport::kTickMean, false));
  MuJoCoSimulator sample_on(SceneConfig(ImuRateExport::kSample, true));
  MuJoCoSimulator sample_off(SceneConfig(ImuRateExport::kSample, false));
  for (auto* s : {&mean_on, &mean_off, &sample_on, &sample_off})
    ASSERT_TRUE(s->Initialize());
  const auto l = LayoutOf(mean_on);
  const auto a = Simulate(mean_on, kSteps);
  const auto b = Simulate(mean_off, kSteps);
  const auto c = Simulate(sample_on, kSteps);
  const auto d = Simulate(sample_off, kSteps);
  for (std::size_t k = 0; k < a.exported.size(); ++k) {
    for (std::size_t i = 0; i < l.imu.size(); ++i) {
      if (!l.imu[i])
        continue;
      const double n_mean = a.exported[k][i] - b.exported[k][i];
      const double n_sample = c.exported[k][i] - d.exported[k][i];
      ASSERT_NE(n_sample, 0.0) << "step " << k << " entry " << i;
      ASSERT_NEAR(n_mean, n_sample, 1e-12) << "step " << k << " entry " << i;
    }
  }
}

TEST(ImuRateExport, ResetStartsThePairAgainAndReplaysTheRunStart) {
  MuJoCoSimulator sim(SceneConfig(ImuRateExport::kTickMean, true));
  ASSERT_TRUE(sim.Initialize());
  const auto start = Simulate(sim, 100);
  static_cast<void>(Simulate(sim, 137));
  sim.ResetForTest();
  const auto again = Simulate(sim, 100);
  for (std::size_t k = 0; k < start.exported.size(); ++k)
    ASSERT_TRUE(BitEqual(again.exported[k], start.exported[k])) << "export " << k;

  MuJoCoSimulator quiet(SceneConfig(ImuRateExport::kTickMean, false));
  ASSERT_TRUE(quiet.Initialize());
  static_cast<void>(Simulate(quiet, 50));
  quiet.ResetForTest();
  const auto after = Simulate(quiet, 2);
  EXPECT_TRUE(BitEqual(after.exported[0], after.raw[0])) << "first export after a reset is raw";
}

TEST(ImuRateExport, RepublishWithoutPhysicsIsSensordataAndTheNextStepPairsAgain) {
  MuJoCoSimulator sim(SceneConfig(ImuRateExport::kTickMean, false));
  ASSERT_TRUE(sim.Initialize());
  const auto l = LayoutOf(sim);
  auto t = Simulate(sim, 20);
  sim.RepublishSensorsForTest();
  EXPECT_TRUE(BitEqual(sim.GetSensorExportForTest(0), sim.GetSensorDataForTest(0)));
  EXPECT_FALSE(BitEqual(t.exported.back(), t.raw.back())) << "step 20 was a tick-mean value";
  Step(sim, t);
  Check c;
  ExpectReference({t.exported[t.exported.size() - 2], t.exported.back()}, l, 0.002, 1, c);
  EXPECT_EQ(c.compared, 1);
}

TEST(ImuRateExport, Rk4ExportsSensordataAndTheNextIntegratorPairsAgain) {
  // RK4's stages leave the kinematics after mj_step off the step's start.
  MuJoCoSimulator sim(SceneConfig(ImuRateExport::kTickMean, false));
  ASSERT_TRUE(sim.Initialize());
  const auto l = LayoutOf(sim);
  sim.SetIntegrator(mjINT_RK4);
  const auto rk4 = Simulate(sim, 50);
  for (std::size_t k = 0; k < rk4.exported.size(); ++k)
    ASSERT_TRUE(BitEqual(rk4.exported[k], rk4.raw[k])) << "RK4 step " << k;
  sim.SetIntegrator(mjINT_IMPLICITFAST);
  const auto back = Simulate(sim, 50);
  EXPECT_TRUE(BitEqual(back.exported[0], back.raw[0])) << "one snapshot after RK4: raw";
  Check c;
  ExpectReference(back.exported, l, 0.002, 1, c);
  EXPECT_EQ(c.compared, 49);
}

TEST(ImuRateExport, APairAcrossAWorldGravitySwitchIsSensordata) {
  MuJoCoSimulator sim(SceneConfig(ImuRateExport::kTickMean, false));
  ASSERT_TRUE(sim.Initialize());
  const auto l = LayoutOf(sim);
  static_cast<void>(Simulate(sim, 20));
  sim.EnableWorldGravity(false);
  const auto off = Simulate(sim, 50);
  EXPECT_TRUE(BitEqual(off.exported[0], off.raw[0]))
      << "the velocity change across the switch is not one specific force";
  constexpr double kNoGravity[3] = {0.0, 0.0, 0.0};
  Check c;
  ExpectReference(off.exported, l, 0.002, 1, c, kNoGravity);
  EXPECT_EQ(c.compared, 49);
}

/// The exports a free-running SimLoop hands the sensor callback, in order.
std::vector<std::vector<double>> SimLoopExports(int n_substeps, std::size_t wanted) {
  auto cfg = SceneConfig(ImuRateExport::kTickMean);
  cfg.n_substeps = n_substeps;
  cfg.sync_timeout_ms = 1.0;
  cfg.max_rtf = 0.0;
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_TRUE(sim.Initialize());
  std::mutex mutex;
  std::vector<std::vector<double>> got;
  sim.SetSensorCallback(
      0, [&](const std::vector<JointGroup::SensorInfo>&, const std::vector<double>& values) {
        std::lock_guard lock(mutex);
        if (got.size() < wanted)
          got.push_back(values);
      });
  sim.Start();
  const auto deadline = std::chrono::steady_clock::now() + 30s;
  for (;;) {
    {
      std::lock_guard lock(mutex);
      if (got.size() >= wanted)
        break;
    }
    if (std::chrono::steady_clock::now() > deadline)
      break;
    std::this_thread::sleep_for(5ms);
  }
  sim.Stop();
  std::lock_guard lock(mutex);
  return got;
}

TEST(ImuRateExport, SimLoopWithThreeSubstepsMatchesTheDifferenceOverTheStep) {
  // The snapshot sits inside the substep loop after the last mj_step: the
  // frame sensors in the same export describe that instant, so any other
  // placement (the first substep, the loop top after mj_forward) misses.
  MuJoCoSimulator probe(SceneConfig(ImuRateExport::kTickMean));
  ASSERT_TRUE(probe.Initialize());
  const auto l = LayoutOf(probe);
  const auto exports = SimLoopExports(3, kSteps);
  ASSERT_EQ(exports.size(), static_cast<std::size_t>(kSteps));
  Check c;
  ExpectReference(exports, l, 0.002, 2, c);  // the XML timestep is the step; 3 substeps split it
  EXPECT_EQ(c.compared, kSteps - 2);
  EXPECT_GT(c.max_gyro, 1.0) << "the site barely turns — the comparison is weak";
  EXPECT_GT(c.clamped, 0) << "the accelerometer never reached its cutoff — the clamp is untested";
}

}  // namespace
}  // namespace rtc
