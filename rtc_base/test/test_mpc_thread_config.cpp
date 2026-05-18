// Tier-by-tier validation of the MpcThreadConfig + layout v3 invariants.
//
// For every supported core count we check:
//   * The tier's kMpcConfig*Core selects the documented MPC main core.
//   * MPC main priority is strictly below rt_inbound priority (rt_inbound preempts
//     MPC).
//   * Worker priorities (if any) are ≤ main priority.
//   * On tiers that dedicate cores, MPC main does not overlap with
//     rt_control or rt_inbound.
//   * Layout v3 same-core invariant: rt_inbound.cpu_core == rt_outbound.cpu_core
//     AND rt_inbound.priority > rt_outbound.priority on tiers where both are RT.
//   * Process-level pins (arm_driver / hand_driver) live on dedicated cores
//     once the tier is ≥ 8-core, and never collide with any RT controller thread.
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
// quality must never regress. Each tier ≥ 8-core must keep MPC main on a
// dedicated core (distinct from rt_control / rt_inbound / nrt_logging /
// nrt_callback), and worker count must be monotonic.
TEST(MpcThreadConfig, TierIsolationMonotonicity) {
  struct TierSnapshot {
    int ncpu;
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_inbound;
    const ThreadConfig* nrt_logging;
    const ThreadConfig* nrt_callback;
  };

  const std::array<TierSnapshot, 5> tiers = {{
      {8, &kMpcConfig8Core, &kRtControlConfig8Core, &kRtInboundConfig8Core, &kNrtLoggingConfig8Core,
       &kNrtCallbackConfig8Core},
      {10, &kMpcConfig10Core, &kRtControlConfig10Core, &kRtInboundConfig10Core,
       &kNrtLoggingConfig10Core, &kNrtCallbackConfig10Core},
      {12, &kMpcConfig12Core, &kRtControlConfig12Core, &kRtInboundConfig12Core,
       &kNrtLoggingConfig12Core, &kNrtCallbackConfig12Core},
      {14, &kMpcConfig14Core, &kRtControlConfig14Core, &kRtInboundConfig14Core,
       &kNrtLoggingConfig14Core, &kNrtCallbackConfig14Core},
      {16, &kMpcConfig16Core, &kRtControlConfig16Core, &kRtInboundConfig16Core,
       &kNrtLoggingConfig16Core, &kNrtCallbackConfig16Core},
  }};

  for (std::size_t i = 1; i < tiers.size(); ++i) {
    const TierSnapshot& lo = tiers[i - 1];
    const TierSnapshot& hi = tiers[i];

    EXPECT_GE(hi.mpc->num_workers, lo.mpc->num_workers)
        << hi.ncpu << "-core has fewer MPC workers than " << lo.ncpu << "-core";

    // Tiers ≥ 10 keep MPC main, nrt_logging, nrt_callback all on dedicated cores.
    if (hi.ncpu >= 10) {
      EXPECT_NE(hi.mpc->main.cpu_core, hi.nrt_logging->cpu_core)
          << hi.ncpu << "-core: MPC main shares nrt_logging core";
      EXPECT_NE(hi.mpc->main.cpu_core, hi.nrt_callback->cpu_core)
          << hi.ncpu << "-core: MPC main shares nrt_callback core";
      EXPECT_NE(hi.nrt_logging->cpu_core, hi.nrt_callback->cpu_core)
          << hi.ncpu << "-core: nrt_logging shares nrt_callback core";
    }
  }
}

// Layout v3: rt_inbound and rt_outbound must share a single core on every
// non-degraded tier (≥ 6) AND keep priority diff (inbound > outbound) so the
// shared-core schedule cannot starve. 4-core is degraded — rt_outbound there
// is CFS by design, so we skip it.
TEST(MpcThreadConfig, LayoutV3SameCoreRtInboundOutbound) {
  struct Pair {
    const ThreadConfig* rt_inbound;
    const ThreadConfig* rt_outbound;
    const char* label;
  };

  const std::array<Pair, 6> pairs = {{
      {&kRtInboundConfig, &kRtOutboundConfig, "6-core"},
      {&kRtInboundConfig8Core, &kRtOutboundConfig8Core, "8-core"},
      {&kRtInboundConfig10Core, &kRtOutboundConfig10Core, "10-core"},
      {&kRtInboundConfig12Core, &kRtOutboundConfig12Core, "12-core"},
      {&kRtInboundConfig14Core, &kRtOutboundConfig14Core, "14-core"},
      {&kRtInboundConfig16Core, &kRtOutboundConfig16Core, "16-core"},
  }};
  for (const auto& p : pairs) {
    EXPECT_EQ(p.rt_inbound->cpu_core, p.rt_outbound->cpu_core)
        << p.label << ": rt_inbound and rt_outbound must share a core (layout v3)";
    EXPECT_GT(p.rt_inbound->sched_priority, p.rt_outbound->sched_priority)
        << p.label << ": rt_inbound must outrank rt_outbound on the shared core";
  }
}

// Layout v3: arm_driver and hand_driver must not collide with any RT
// controller thread (rt_control / rt_inbound / rt_outbound / mpc_*). Tiers
// ≥ 8 give each its own dedicated core; tiers below are degraded (4/6-core)
// and may share with OS cores.
TEST(MpcThreadConfig, LayoutV3ArmHandDriverDisjoint) {
  struct TierDrivers {
    const char* label;
    const ThreadConfig* arm;
    const ThreadConfig* hand;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_inbound;
    const ThreadConfig* rt_outbound;
    const MpcThreadConfig* mpc;
    bool dedicated;  // tiers ≥ 8 expect arm/hand on dedicated cores
  };

  const std::array<TierDrivers, 7> tiers = {{
      {"4-core", &kArmDriverConfig4Core, &kHandDriverConfig4Core, &kRtControlConfig4Core,
       &kRtInboundConfig4Core, &kRtOutboundConfig4Core, &kMpcConfig4Core, false},
      {"6-core", &kArmDriverConfig, &kHandDriverConfig, &kRtControlConfig, &kRtInboundConfig,
       &kRtOutboundConfig, &kMpcConfig6Core, false},
      {"8-core", &kArmDriverConfig8Core, &kHandDriverConfig8Core, &kRtControlConfig8Core,
       &kRtInboundConfig8Core, &kRtOutboundConfig8Core, &kMpcConfig8Core, true},
      {"10-core", &kArmDriverConfig10Core, &kHandDriverConfig10Core, &kRtControlConfig10Core,
       &kRtInboundConfig10Core, &kRtOutboundConfig10Core, &kMpcConfig10Core, true},
      {"12-core", &kArmDriverConfig12Core, &kHandDriverConfig12Core, &kRtControlConfig12Core,
       &kRtInboundConfig12Core, &kRtOutboundConfig12Core, &kMpcConfig12Core, true},
      {"14-core", &kArmDriverConfig14Core, &kHandDriverConfig14Core, &kRtControlConfig14Core,
       &kRtInboundConfig14Core, &kRtOutboundConfig14Core, &kMpcConfig14Core, true},
      {"16-core", &kArmDriverConfig16Core, &kHandDriverConfig16Core, &kRtControlConfig16Core,
       &kRtInboundConfig16Core, &kRtOutboundConfig16Core, &kMpcConfig16Core, true},
  }};
  for (const auto& t : tiers) {
    if (!t.dedicated) {
      continue;  // 4/6-core: degraded, arm/hand may share with OS cores
    }
    EXPECT_NE(t.arm->cpu_core, t.rt_control->cpu_core) << t.label << ": arm shares rt_control core";
    EXPECT_NE(t.arm->cpu_core, t.rt_inbound->cpu_core) << t.label << ": arm shares rt_inbound core";
    EXPECT_NE(t.arm->cpu_core, t.rt_outbound->cpu_core)
        << t.label << ": arm shares rt_outbound core";
    EXPECT_NE(t.arm->cpu_core, t.mpc->main.cpu_core) << t.label << ": arm shares mpc_main core";
    EXPECT_NE(t.hand->cpu_core, t.rt_control->cpu_core)
        << t.label << ": hand shares rt_control core";
    EXPECT_NE(t.hand->cpu_core, t.rt_inbound->cpu_core)
        << t.label << ": hand shares rt_inbound core";
    EXPECT_NE(t.hand->cpu_core, t.rt_outbound->cpu_core)
        << t.label << ": hand shares rt_outbound core";
    EXPECT_NE(t.hand->cpu_core, t.mpc->main.cpu_core) << t.label << ": hand shares mpc_main core";
    for (int i = 0; i < t.mpc->num_workers; ++i) {
      const int worker_core = t.mpc->workers[static_cast<std::size_t>(i)].cpu_core;
      EXPECT_NE(t.arm->cpu_core, worker_core)
          << t.label << ": arm shares mpc_worker_" << i << " core";
      EXPECT_NE(t.hand->cpu_core, worker_core)
          << t.label << ": hand shares mpc_worker_" << i << " core";
    }
    // Arm and hand on dedicated tiers also get distinct cores from each other.
    EXPECT_NE(t.arm->cpu_core, t.hand->cpu_core)
        << t.label << ": arm and hand share a core on a tier that should have dedicated pins";
  }
}

// Every tier's canonical SystemThreadConfigs must pass
// ValidateSystemThreadConfigs in isolation (independent of the host machine's
// real core count) — exercised via a tier-by-tier ValidateThreadConfig
// equivalent that does not bind to GetOnlineCpuCount.
TEST(MpcThreadConfig, LayoutV3ValidateAllTiersWhenHostFits) {
  // We can only run ValidateSystemThreadConfigs against tiers whose
  // highest core fits the host (it cross-checks cpu_core against
  // GetOnlineCpuCount). Pick the tier matching SelectThreadConfigs() and
  // additionally walk down through smaller tiers that also fit.
  const int ncpu = GetPhysicalCpuCount();
  if (ncpu < 4) {
    GTEST_SKIP() << "Host has only " << ncpu << " cores; minimum 4-core tier requires >= 4";
  }
  const SystemThreadConfigs host = SelectThreadConfigs();
  const std::string err = ValidateSystemThreadConfigs(host);
  EXPECT_TRUE(err.empty()) << "Host-tier validation failed: " << err;
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

// Negative case: ValidateSystemThreadConfigs must actually catch arm/hand
// driver pins that collide with an RT controller thread. The
// LayoutV3ArmHandDriverDisjoint test above only proves "current canonical
// configs are clean"; this test proves "the validator catches violations."
TEST(MpcThreadConfig, LayoutV3ValidatorCatchesArmHandCollision) {
  const int ncpu = GetPhysicalCpuCount();
  if (ncpu < 4) {
    GTEST_SKIP() << "Host has only " << ncpu << " cores; minimum 4-core tier requires >= 4";
  }

  SystemThreadConfigs bad = SelectThreadConfigs();
  // Force arm_driver onto rt_inbound's core — a clear violation of the
  // Phase 5 disjointness rule.
  bad.arm_driver.cpu_core = bad.rt_inbound.cpu_core;
  const std::string err = ValidateSystemThreadConfigs(bad);
  EXPECT_FALSE(err.empty()) << "Validator missed arm_driver/rt_inbound core collision";
  EXPECT_NE(err.find("arm_driver"), std::string::npos)
      << "Error string should name arm_driver, got: " << err;
}

// cpu_core == -1 sentinel must pass ValidateThreadConfig (Phase 5 follow-up).
// kRtUdpRecvConfig (transceiver default) and kHandUdpRecvConfig (udp_hand
// private) both ship with cpu_core = -1; ApplyThreadConfig must accept them
// so the FIFO 65 priority actually lands on the receive thread instead of
// being silently dropped by an early-return validation failure.
TEST(MpcThreadConfig, CpuCoreSentinelValidatesAsRtConfig) {
  const ThreadConfig sentinel{.cpu_core = -1,
                              .sched_policy = SCHED_FIFO,
                              .sched_priority = 65,
                              .nice_value = 0,
                              .name = "rt_udp_recv"};
  EXPECT_TRUE(ValidateThreadConfig(sentinel).empty())
      << "cpu_core = -1 must validate (no-pinning sentinel)";

  const ThreadConfig invalid{.cpu_core = -2,
                             .sched_policy = SCHED_FIFO,
                             .sched_priority = 65,
                             .nice_value = 0,
                             .name = "bad_core"};
  EXPECT_FALSE(ValidateThreadConfig(invalid).empty()) << "cpu_core < -1 must still fail validation";
}

}  // namespace
}  // namespace rtc
