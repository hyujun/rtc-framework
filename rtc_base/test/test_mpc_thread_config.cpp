// Tier-by-tier validation of the Phase 5 MpcThreadConfig entries.
//
// For every supported core count we check:
//   * The tier's kMpcConfig*Core selects the documented MPC main core.
//   * MPC main priority is strictly below rt_inbound priority (rt_inbound preempts
//     MPC).
//   * Worker priorities (if any) are ≤ main priority.
//   * On tiers that dedicate cores, MPC main does not overlap with
//     rt_control, rt_inbound, or udp_recv.
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

const std::array<TierExpectation, 7> kTiers = {{
    {"4-core", &kMpcConfig4Core, 3, 0},
    {"6-core", &kMpcConfig6Core, 4, 0},
    {"8-core", &kMpcConfig8Core, 4, 0},
    {"10-core", &kMpcConfig10Core, 4, 1},
    {"12-core", &kMpcConfig12Core, 4, 2},
    {"14-core", &kMpcConfig14Core, 4, 2},
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
    if (mpc.main.sched_policy != SCHED_FIFO && mpc.main.sched_policy != SCHED_RR) {
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
  // Pair every tier's MPC with the corresponding rt_inbound config.
  struct Pair {
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_inbound;
    const char* label;
  };

  const std::array<Pair, 7> pairs = {{
      {&kMpcConfig4Core, &kRtInboundConfig4Core, "4-core"},
      {&kMpcConfig6Core, &kRtInboundConfig, "6-core"},
      {&kMpcConfig8Core, &kRtInboundConfig8Core, "8-core"},
      {&kMpcConfig10Core, &kRtInboundConfig10Core, "10-core"},
      {&kMpcConfig12Core, &kRtInboundConfig12Core, "12-core"},
      {&kMpcConfig14Core, &kRtInboundConfig14Core, "14-core"},
      {&kMpcConfig16Core, &kRtInboundConfig16Core, "16-core"},
  }};
  for (const auto& pair : pairs) {
    const bool mpc_is_rt =
        pair.mpc->main.sched_policy == SCHED_FIFO || pair.mpc->main.sched_policy == SCHED_RR;
    if (!mpc_is_rt) {
      continue;  // SCHED_OTHER MPC never preempts anything RT.
    }
    EXPECT_LT(pair.mpc->main.sched_priority, pair.rt_inbound->sched_priority)
        << pair.label << ": MPC priority must be below rt_inbound";
  }
}

TEST(MpcThreadConfig, DedicatedTiersDoNotShareWithSensorOrRt) {
  // 8/10/12/14/16-core tiers give MPC a dedicated main core.
  struct DedicatedPair {
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_inbound;
    const char* label;
  };

  const std::array<DedicatedPair, 5> dedicated = {{
      {&kMpcConfig8Core, &kRtControlConfig8Core, &kRtInboundConfig8Core, "8"},
      {&kMpcConfig10Core, &kRtControlConfig10Core, &kRtInboundConfig10Core, "10"},
      {&kMpcConfig12Core, &kRtControlConfig12Core, &kRtInboundConfig12Core, "12"},
      {&kMpcConfig14Core, &kRtControlConfig14Core, &kRtInboundConfig14Core, "14"},
      {&kMpcConfig16Core, &kRtControlConfig16Core, &kRtInboundConfig16Core, "16"},
  }};
  for (const auto& d : dedicated) {
    EXPECT_NE(d.mpc->main.cpu_core, d.rt_control->cpu_core)
        << d.label << "-core: MPC must not share rt_control's core";
    EXPECT_NE(d.mpc->main.cpu_core, d.rt_inbound->cpu_core)
        << d.label << "-core: MPC must not share rt_inbound's core";
  }
}

// Monotonicity invariant: as physical core count grows, RT/MPC isolation
// quality must never regress. Specifically, once MPC gets a dedicated main
// core and N worker threads, a larger tier must have at least as many
// workers and must keep MPC main, UDP recv, logging, and nrt_callback all on
// distinct cores. This is the test that would have caught the pre-unified
// 10-core regression (where Core 9 hosted udp_recv + nrt_logging + nrt_callback +
// publish + mpc_main together).
TEST(MpcThreadConfig, TierIsolationMonotonicity) {
  struct TierSnapshot {
    int ncpu;
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_inbound;
    const ThreadConfig* udp_recv;
    const ThreadConfig* nrt_logging;
    const ThreadConfig* nrt_callback;
  };

  const std::array<TierSnapshot, 5> tiers = {{
      {8, &kMpcConfig8Core, &kRtControlConfig8Core, &kRtInboundConfig8Core, &kUdpRecvConfig8Core,
       &kNrtLoggingConfig8Core, &kNrtCallbackConfig8Core},
      {10, &kMpcConfig10Core, &kRtControlConfig10Core, &kRtInboundConfig10Core, &kUdpRecvConfig10Core,
       &kNrtLoggingConfig10Core, &kNrtCallbackConfig10Core},
      {12, &kMpcConfig12Core, &kRtControlConfig12Core, &kRtInboundConfig12Core, &kUdpRecvConfig12Core,
       &kNrtLoggingConfig12Core, &kNrtCallbackConfig12Core},
      {14, &kMpcConfig14Core, &kRtControlConfig14Core, &kRtInboundConfig14Core, &kUdpRecvConfig14Core,
       &kNrtLoggingConfig14Core, &kNrtCallbackConfig14Core},
      {16, &kMpcConfig16Core, &kRtControlConfig16Core, &kRtInboundConfig16Core, &kUdpRecvConfig16Core,
       &kNrtLoggingConfig16Core, &kNrtCallbackConfig16Core},
  }};

  auto distinct_rt_core_count = [](const TierSnapshot& t) {
    // Count distinct cores across the RT-critical thread set.
    std::array<int, 8> cores{t.rt_control->cpu_core,
                             t.rt_inbound->cpu_core,
                             t.mpc->main.cpu_core,
                             t.udp_recv->cpu_core,
                             t.nrt_logging->cpu_core,
                             t.nrt_callback->cpu_core,
                             -1,
                             -1};
    if (t.mpc->num_workers >= 1)
      cores[6] = t.mpc->workers[0].cpu_core;
    if (t.mpc->num_workers >= 2)
      cores[7] = t.mpc->workers[1].cpu_core;
    int unique = 0;
    for (std::size_t i = 0; i < cores.size(); ++i) {
      if (cores[i] < 0)
        continue;
      bool seen = false;
      for (std::size_t j = 0; j < i; ++j) {
        if (cores[j] == cores[i]) {
          seen = true;
          break;
        }
      }
      if (!seen)
        ++unique;
    }
    return unique;
  };

  for (std::size_t i = 1; i < tiers.size(); ++i) {
    const TierSnapshot& lo = tiers[i - 1];
    const TierSnapshot& hi = tiers[i];

    EXPECT_GE(hi.mpc->num_workers, lo.mpc->num_workers)
        << hi.ncpu << "-core has fewer MPC workers than " << lo.ncpu << "-core";

    EXPECT_GE(distinct_rt_core_count(hi), distinct_rt_core_count(lo))
        << hi.ncpu << "-core regresses RT isolation (distinct-core count) vs " << lo.ncpu
        << "-core";

    // Every tier from 10 onwards must keep MPC main, UDP, nrt_logging on
    // distinct cores (the 10-core regression crammed all three onto
    // Core 9).
    if (hi.ncpu >= 10) {
      EXPECT_NE(hi.mpc->main.cpu_core, hi.udp_recv->cpu_core)
          << hi.ncpu << "-core: MPC main shares UDP core";
      EXPECT_NE(hi.mpc->main.cpu_core, hi.nrt_logging->cpu_core)
          << hi.ncpu << "-core: MPC main shares nrt_logging core";
      EXPECT_NE(hi.udp_recv->cpu_core, hi.nrt_logging->cpu_core)
          << hi.ncpu << "-core: UDP recv shares nrt_logging core";
    }
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

  // The smallest supported tier (kMpcConfig4Core) uses Core 3, which requires
  // at least 4 physical cores.  CI runners (e.g. GitHub Actions) typically
  // have only 2 cores, so skip rather than report a false failure.
  if (ncpu < 4) {
    GTEST_SKIP() << "Host has only " << ncpu << " cores; minimum 4-core tier requires >= 4";
  }

  const SystemThreadConfigs host_tier = SelectThreadConfigs();

  // MPC main's core must fit in the host's core count.
  EXPECT_LT(host_tier.mpc.main.cpu_core, ncpu)
      << "SelectThreadConfigs returned MPC on a core that does not exist";

  const std::string err = ValidateSystemThreadConfigs(host_tier);
  EXPECT_TRUE(err.empty()) << "Host tier validation should succeed, got: " << err;
}

}  // namespace
}  // namespace rtc
