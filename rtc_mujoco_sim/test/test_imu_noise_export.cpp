// ── test_imu_noise_export.cpp ────────────────────────────────────────────────
// robot_response.<group>.imu_noise inside the simulator: what reaches the
// sensor export, and what must not move. One sample is one ReadSensors, read
// as export − sensordata (the noise). In floating point (x + n) − x still
// depends on x, so the bit-equality checks also rely on the physics itself
// replaying bit for bit (same model, same steps); they compare the noise of
// runs whose sensordata is the same, not noise alone.
//   off (default)  — the export is mjData::sensordata, bit for bit
//   on             — same seed, same noise; sensordata itself, the frame and
//                    contact sensors in the export and the contact wrench
//                    lane are bit-identical to an off run
//   reset          — the first samples after a reset replay the run start
//   publish rate   — a decimated group draws the same noise as an undecimated
//                    one: the model advances on every step
//   noise= warning — counted once per group that exports a sensor with an
//                    MJCF noise= that stays exact (imu_noise off, or a
//                    non-IMU sensor)
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
#include <thread>
#include <vector>

#ifndef IMU_NOISE_MJCF_PATH
#error "IMU_NOISE_MJCF_PATH must be defined by CMake"
#endif
#ifndef CONTACT_MJCF_PATH
#error "CONTACT_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

using namespace std::chrono_literals;

constexpr int kSteps = 500;
constexpr int kResetWindow = 100;

ImuNoiseConfig ShippedNoise(bool enabled) {
  ImuNoiseConfig n;
  n.enabled = enabled;
  n.seed = 20261114;
  n.gyro = {5e-4, 1e-3, 1e-5};
  n.accel = {1e-2, 5e-2, 1e-4};
  return n;
}

MuJoCoSimulator::Config SceneConfig(const ImuNoiseConfig& noise) {
  auto cfg = test::HeadlessConfig(IMU_NOISE_MJCF_PATH, 10.0);
  auto g = test::ContactWrenchGroup("arm", {"j1"}, "/arm");
  g.imu_noise = noise;
  cfg.groups.push_back(std::move(g));
  return cfg;
}

struct Trace {
  std::vector<std::vector<double>> exported;  // per step: the sensor export
  std::vector<std::vector<double>> raw;       // per step: sensordata, same layout
  std::vector<std::vector<double>> wrench;    // per step: contact wrench lane
};

/// Drive the pendulum through a swing so the gyro and accelerometer move and
/// the tip makes and breaks contact. `k` restarts at 0 in every Run, so a
/// post-reset run replays the same command sequence.
void Step(MuJoCoSimulator& sim, int k, Trace& t) {
  sim.SetCommand(0, {0.6 * std::sin(0.02 * static_cast<double>(k))});
  sim.StepForTest();
  t.exported.push_back(sim.GetSensorExportForTest(0));
  t.raw.push_back(sim.GetSensorDataForTest(0));
  std::vector<double> w;
  for (const auto& s : sim.GetContactWrenchSamplesForTest(0)) {
    w.push_back(s.found ? 1.0 : 0.0);
    w.insert(w.end(), s.force.begin(), s.force.end());
    w.insert(w.end(), s.torque.begin(), s.torque.end());
    w.insert(w.end(), s.point_world.begin(), s.point_world.end());
    w.push_back(s.dist);
  }
  t.wrench.push_back(std::move(w));
}

Trace Simulate(MuJoCoSimulator& sim, int steps) {
  Trace t;
  for (int k = 0; k < steps; ++k)
    Step(sim, k, t);
  return t;
}

/// Which export entries belong to a gyro or an accelerometer.
std::vector<bool> ImuMask(const MuJoCoSimulator& sim) {
  std::vector<bool> mask;
  for (const auto& si : sim.GetSensorInfos(0)) {
    const bool imu = si.type == mjSENS_GYRO || si.type == mjSENS_ACCELEROMETER;
    mask.insert(mask.end(), static_cast<std::size_t>(si.dim), imu);
  }
  return mask;
}

std::vector<double> Noise(const Trace& t, std::size_t k) {
  std::vector<double> d(t.exported[k].size());
  for (std::size_t i = 0; i < d.size(); ++i)
    d[i] = t.exported[k][i] - t.raw[k][i];
  return d;
}

bool BitEqual(const std::vector<double>& a, const std::vector<double>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

TEST(ImuNoiseExport, DisabledExportIsSensordataBitForBit) {
  MuJoCoSimulator sim(SceneConfig(ShippedNoise(false)));
  ASSERT_TRUE(sim.Initialize());
  const auto t = Simulate(sim, kSteps);
  ASSERT_FALSE(t.exported[0].empty());
  for (std::size_t k = 0; k < t.exported.size(); ++k) {
    ASSERT_TRUE(BitEqual(t.exported[k], t.raw[k])) << "step " << k;
  }
}

TEST(ImuNoiseExport, SameSeedTwoRunsGiveTheSameNoiseOnTheImuChannelsOnly) {
  MuJoCoSimulator a(SceneConfig(ShippedNoise(true)));
  MuJoCoSimulator b(SceneConfig(ShippedNoise(true)));
  ASSERT_TRUE(a.Initialize());
  ASSERT_TRUE(b.Initialize());
  const auto mask = ImuMask(a);
  ASSERT_EQ(std::count(mask.begin(), mask.end(), true), 6) << "one gyro + one accelerometer";
  const auto ta = Simulate(a, kSteps);
  const auto tb = Simulate(b, kSteps);
  for (std::size_t k = 0; k < ta.exported.size(); ++k) {
    const auto na = Noise(ta, k);
    ASSERT_TRUE(BitEqual(na, Noise(tb, k))) << "step " << k;
    for (std::size_t i = 0; i < na.size(); ++i) {
      if (mask[i]) {
        ASSERT_NE(na[i], 0.0) << "step " << k << " entry " << i << ": IMU entry not noised";
      } else {
        ASSERT_EQ(ta.exported[k][i], ta.raw[k][i]) << "step " << k << " entry " << i;
      }
    }
  }
}

TEST(ImuNoiseExport, OnAndOffRunsShareSensordataFrameContactAndWrench) {
  MuJoCoSimulator on(SceneConfig(ShippedNoise(true)));
  MuJoCoSimulator off(SceneConfig(ShippedNoise(false)));
  ASSERT_TRUE(on.Initialize());
  ASSERT_TRUE(off.Initialize());
  ASSERT_TRUE(on.HasContactWrenches(0));
  const auto mask = ImuMask(on);
  const auto t_on = Simulate(on, kSteps);
  const auto t_off = Simulate(off, kSteps);
  bool touched = false;
  for (std::size_t k = 0; k < t_on.raw.size(); ++k) {
    ASSERT_TRUE(BitEqual(t_on.raw[k], t_off.raw[k])) << "sensordata, step " << k;
    ASSERT_TRUE(BitEqual(t_on.wrench[k], t_off.wrench[k])) << "contact wrench, step " << k;
    for (std::size_t i = 0; i < mask.size(); ++i) {
      if (!mask[i]) {
        ASSERT_EQ(std::memcmp(&t_on.exported[k][i], &t_off.exported[k][i], sizeof(double)), 0)
            << "non-IMU export entry " << i << ", step " << k;
      }
    }
    for (double f : t_on.wrench[k])
      touched = touched || f != 0.0;
  }
  EXPECT_TRUE(touched) << "the swing never loaded the contact lane — the check above is vacuous";
}

TEST(ImuNoiseExport, ResetReplaysTheFirstSamplesOfTheRun) {
  MuJoCoSimulator sim(SceneConfig(ShippedNoise(true)));
  ASSERT_TRUE(sim.Initialize());
  const auto start = Simulate(sim, kResetWindow);
  static_cast<void>(Simulate(sim, 137));
  sim.ResetForTest();
  const auto again = Simulate(sim, kResetWindow);
  for (std::size_t k = 0; k < static_cast<std::size_t>(kResetWindow); ++k) {
    ASSERT_TRUE(BitEqual(Noise(again, k), Noise(start, k))) << "sample " << k;
  }
}

/// The export each sensor callback received, in order, from a free-running
/// SimLoop (no controller: each step runs on the sync timeout).
std::vector<std::vector<double>> CallbackExports(int divisor, std::size_t wanted) {
  auto cfg = SceneConfig(ShippedNoise(true));
  cfg.sync_timeout_ms = 1.0;
  cfg.groups[0].state_publish_divisor = divisor;
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
  const auto deadline = std::chrono::steady_clock::now() + 20s;
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

TEST(ImuNoiseExport, DecimatedGroupDrawsTheNoiseOfEveryStep) {
  // Physics with no command is identical in both runs, so the published
  // exports differ only if the noise did not advance on the unpublished steps.
  constexpr int kDivisor = 5;
  constexpr std::size_t kPublished = 20;
  const auto every = CallbackExports(1, kDivisor * (kPublished - 1) + 1);
  const auto decimated = CallbackExports(kDivisor, kPublished);
  ASSERT_EQ(every.size(), kDivisor * (kPublished - 1) + 1);
  ASSERT_EQ(decimated.size(), kPublished);
  for (std::size_t k = 0; k < kPublished; ++k) {
    EXPECT_TRUE(BitEqual(decimated[k], every[k * kDivisor])) << "published sample " << k;
  }
}

TEST(ImuNoiseExport, DeclaredNoiseThatStaysExactWarnsOncePerGroup) {
  // The scene declares noise= on the gyro, the accelerometer and a framepos.
  MuJoCoSimulator off(SceneConfig(ShippedNoise(false)));
  ASSERT_TRUE(off.Initialize());
  EXPECT_EQ(off.ImuNoiseWarningCount(), 1) << "three noise= sensors, one group: one warning";

  MuJoCoSimulator on(SceneConfig(ShippedNoise(true)));
  ASSERT_TRUE(on.Initialize());
  EXPECT_EQ(on.ImuNoiseWarningCount(), 1) << "the framepos stays exact with imu_noise on";

  auto imu_only = SceneConfig(ShippedNoise(true));
  imu_only.groups[0].sensor_names = {"imu_gyro", "imu_accel", "imu_quat"};
  imu_only.groups[0].contact_wrench.enabled = false;
  MuJoCoSimulator covered(std::move(imu_only));
  ASSERT_TRUE(covered.Initialize());
  EXPECT_EQ(covered.ImuNoiseWarningCount(), 0) << "every noise= sensor exported is noised";

  auto cfg = test::HeadlessConfig(CONTACT_MJCF_PATH, 10.0);
  cfg.groups.push_back(test::ContactWrenchGroup("arm", {"j1"}, "/arm", "_ft_site"));
  MuJoCoSimulator plain(std::move(cfg));
  ASSERT_TRUE(plain.Initialize());
  EXPECT_EQ(plain.ImuNoiseWarningCount(), 0) << "no sensor declares noise=";
}

TEST(ImuNoiseExport, NegativeOrNonFiniteSigmaFailsInitialize) {
  for (const double bad : {-1e-3, std::nan("")}) {
    auto noise = ShippedNoise(true);
    noise.accel.bias_walk_sigma = bad;
    MuJoCoSimulator sim(SceneConfig(noise));
    EXPECT_FALSE(sim.Initialize()) << bad;
  }
}

}  // namespace
}  // namespace rtc
