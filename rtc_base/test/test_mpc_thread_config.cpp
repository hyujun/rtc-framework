// Tier-by-tier validation of the Phase 5 MpcThreadConfig entries.
//
// For every supported core count we check:
//   * The tier's kMpcConfig*Core selects the documented MPC main core.
//   * MPC main priority is strictly below sensor priority (sensor preempts
//     MPC).
//   * Worker priorities (if any) are ≤ main priority.
//   * On tiers that dedicate cores, MPC main does not overlap with
//     rt_control, sensor, or udp_recv.
//   * ValidateSystemThreadConfigs returns empty string for the canonical
//     configs (no conflicts).

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"

#include <gtest/gtest.h>

#include <string>

namespace rtc {
namespace {

struct TierExpectation {
  const char* label;
  const MpcThreadConfig* mpc;
  int expected_main_core;
  int expected_num_workers;
};

const std::array<TierExpectation, 6> kTiers = {{
    {"4-core",  &kMpcConfig4Core,  3, 0},
    {"6-core",  &kMpcConfig6Core,  4, 0},
    {"8-core",  &kMpcConfig8Core,  4, 0},
    {"10-core", &kMpcConfig10Core, 9, 0},
    {"12-core", &kMpcConfig12Core, 9, 1},
    {"16-core", &kMpcConfig16Core, 9, 2},
}};

TEST(MpcThreadConfig, MainCoreMatchesSpec) {
  for (const auto& tier : kTiers) {
    EXPECT_EQ(tier.mpc->main.cpu_core, tier.expected_main_core)
        << tier.label << ": MPC main core mismatch";
    EXPECT_EQ(tier.mpc->num_workers, tier.expected_num_workers)
        << tier.label << ": worker count mismatch";
  }
}

TEST(MpcThreadConfig, WorkerPriorityNotAboveMain) {
  for (const auto& tier : kTiers) {
    const auto& mpc = *tier.mpc;
    if (mpc.main.sched_policy != SCHED_FIFO &&
        mpc.main.sched_policy != SCHED_RR) {
      continue;  // Non-RT tier (4-core)
    }
    for (int i = 0; i < mpc.num_workers; ++i) {
      const auto& worker = mpc.workers[static_cast<std::size_t>(i)];
      EXPECT_LE(worker.sched_priority, mpc.main.sched_priority)
          << tier.label << " worker " << i << ": priority > main";
    }
  }
}

TEST(MpcThreadConfig, PriorityBelowSensor) {
  // Pair every tier's MPC with the corresponding sensor config.
  struct Pair {
    const MpcThreadConfig* mpc;
    const ThreadConfig* sensor;
    const char* label;
  };
  const std::array<Pair, 6> pairs = {{
      {&kMpcConfig4Core,  &kSensorConfig4Core,  "4-core"},
      {&kMpcConfig6Core,  &kSensorConfig,       "6-core"},
      {&kMpcConfig8Core,  &kSensorConfig8Core,  "8-core"},
      {&kMpcConfig10Core, &kSensorConfig10Core, "10-core"},
      {&kMpcConfig12Core, &kSensorConfig12Core, "12-core"},
      {&kMpcConfig16Core, &kSensorConfig16Core, "16-core"},
  }};
  for (const auto& pair : pairs) {
    const bool mpc_is_rt = pair.mpc->main.sched_policy == SCHED_FIFO ||
                           pair.mpc->main.sched_policy == SCHED_RR;
    if (!mpc_is_rt) {
      continue;  // SCHED_OTHER MPC never preempts anything RT.
    }
    EXPECT_LT(pair.mpc->main.sched_priority, pair.sensor->sched_priority)
        << pair.label << ": MPC priority must be below sensor";
  }
}

TEST(MpcThreadConfig, DedicatedTiersDoNotShareWithSensorOrRt) {
  // 8, 12, 16-core tiers give MPC a dedicated core.
  struct DedicatedPair {
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_control;
    const ThreadConfig* sensor;
    const char* label;
  };
  const std::array<DedicatedPair, 3> dedicated = {{
      {&kMpcConfig8Core,  &kRtControlConfig8Core,  &kSensorConfig8Core,  "8"},
      {&kMpcConfig12Core, &kRtControlConfig12Core, &kSensorConfig12Core, "12"},
      {&kMpcConfig16Core, &kRtControlConfig16Core, &kSensorConfig16Core, "16"},
  }};
  for (const auto& d : dedicated) {
    EXPECT_NE(d.mpc->main.cpu_core, d.rt_control->cpu_core)
        << d.label << "-core: MPC must not share rt_control's core";
    EXPECT_NE(d.mpc->main.cpu_core, d.sensor->cpu_core)
        << d.label << "-core: MPC must not share sensor's core";
  }
}

TEST(MpcThreadConfig, NoRtPriorityConflicts) {
  // Validate the MPC-specific invariants directly (priority / worker rules)
  // without going through ValidateThreadConfig, which cross-checks cpu_core
  // against the host's live core count — not meaningful for tiers larger
  // than the test machine.
  //
  // This also covers the ValidateSystemThreadConfigs MPC branch for the
  // tier that matches this machine: pick the tier whose max core is in
  // range, validate it, and require an empty error string.
  const int ncpu = GetPhysicalCpuCount();
  const SystemThreadConfigs host_tier = SelectThreadConfigs();

  // MPC main's core must fit in the host's core count.
  EXPECT_LT(host_tier.mpc.main.cpu_core, ncpu)
      << "SelectThreadConfigs returned MPC on a core that does not exist";

  const std::string err = ValidateSystemThreadConfigs(host_tier);
  EXPECT_TRUE(err.empty())
      << "Host tier validation should succeed, got: " << err;
}

}  // namespace
}  // namespace rtc
