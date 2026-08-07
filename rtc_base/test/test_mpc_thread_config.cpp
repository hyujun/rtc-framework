// Tier-by-tier validation of the MpcThreadConfig + layout v4 invariants.
//
// For every supported core count we check:
//   * The tier's kMpcConfig*Core selects the documented MPC main core.
//   * MPC main priority is strictly below rt_callback priority (rt_callback preempts
//     MPC).
//   * Worker priorities (if any) are ≤ main priority.
//   * On tiers that dedicate cores, MPC main does not overlap with
//     rt_control or rt_callback.
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
};

const std::array<TierExpectation, 7> kTiers = {{
    {"4-core", &kMpcConfig4Core, 3},
    {"6-core", &kMpcConfig6Core, 3},
    {"8-core", &kMpcConfig8Core, 3},
    {"10-core", &kMpcConfig10Core, 3},
    {"12-core", &kMpcConfig12Core, 3},
    {"14-core", &kMpcConfig14Core, 3},
    {"16-core", &kMpcConfig16Core, 3},
}};

TEST(MpcThreadConfig, MainCoreMatchesSpec) {
  for (const auto& tier : kTiers) {
    EXPECT_EQ(tier.mpc->main.cpu_core, tier.expected_main_core)
        << tier.label << ": MPC main core mismatch";
  }
}

TEST(MpcThreadConfig, PriorityBelowSensor) {
  // Pair every tier's MPC with the corresponding rt_callback config.
  struct Pair {
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_callback;
    const char* label;
  };

  const std::array<Pair, 7> pairs = {{
      {&kMpcConfig4Core, &kRtCallbackConfig4Core, "4-core"},
      {&kMpcConfig6Core, &kRtCallbackConfig, "6-core"},
      {&kMpcConfig8Core, &kRtCallbackConfig8Core, "8-core"},
      {&kMpcConfig10Core, &kRtCallbackConfig10Core, "10-core"},
      {&kMpcConfig12Core, &kRtCallbackConfig12Core, "12-core"},
      {&kMpcConfig14Core, &kRtCallbackConfig14Core, "14-core"},
      {&kMpcConfig16Core, &kRtCallbackConfig16Core, "16-core"},
  }};
  for (const auto& pair : pairs) {
    const bool mpc_is_rt =
        pair.mpc->main.sched_policy == SCHED_FIFO || pair.mpc->main.sched_policy == SCHED_RR;
    if (!mpc_is_rt) {
      continue;  // SCHED_OTHER MPC never preempts anything RT.
    }
    EXPECT_LT(pair.mpc->main.sched_priority, pair.rt_callback->sched_priority)
        << pair.label << ": MPC priority must be below rt_callback";
  }
}

TEST(MpcThreadConfig, DedicatedTiersDoNotShareWithSensorOrRt) {
  // 8/10/12/14/16-core tiers give MPC a dedicated main core.
  struct DedicatedPair {
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_callback;
    const char* label;
  };

  const std::array<DedicatedPair, 5> dedicated = {{
      {&kMpcConfig8Core, &kRtControlConfig8Core, &kRtCallbackConfig8Core, "8"},
      {&kMpcConfig10Core, &kRtControlConfig10Core, &kRtCallbackConfig10Core, "10"},
      {&kMpcConfig12Core, &kRtControlConfig12Core, &kRtCallbackConfig12Core, "12"},
      {&kMpcConfig14Core, &kRtControlConfig14Core, &kRtCallbackConfig14Core, "14"},
      {&kMpcConfig16Core, &kRtControlConfig16Core, &kRtCallbackConfig16Core, "16"},
  }};
  for (const auto& d : dedicated) {
    EXPECT_NE(d.mpc->main.cpu_core, d.rt_control->cpu_core)
        << d.label << "-core: MPC must not share rt_control's core";
    EXPECT_NE(d.mpc->main.cpu_core, d.rt_callback->cpu_core)
        << d.label << "-core: MPC must not share rt_callback's core";
  }
}

// Every tier from 8-core up keeps MPC solve off both CFS lanes, so a long
// solve never contends with logging or the non-RT executor.
//
// This was a monotonicity fixture until #380: it walked consecutive tier pairs
// because worker count had to be non-decreasing as cores grew. With the worker
// slots gone the only surviving property is per-tier, so the loop no longer
// holds a `lo` snapshot and the name says what the body checks. The guard was
// also `>= 10` while the comment above it claimed `>= 8`; the tier list starts
// at 8 and the 8-core tier satisfies the rule (mpc_main slot 3, nrt lanes on
// the aux slot 2), so the check now covers what was always claimed.
TEST(MpcThreadConfig, DedicatedTiersDoNotShareWithNrtLanes) {
  struct TierSnapshot {
    int ncpu;
    const MpcThreadConfig* mpc;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_callback;
    const ThreadConfig* nrt_logging;
    const ThreadConfig* nrt_callback;
  };

  const std::array<TierSnapshot, 5> tiers = {{
      {8, &kMpcConfig8Core, &kRtControlConfig8Core, &kRtCallbackConfig8Core,
       &kNrtLoggingConfig8Core, &kNrtCallbackConfig8Core},
      {10, &kMpcConfig10Core, &kRtControlConfig10Core, &kRtCallbackConfig10Core,
       &kNrtLoggingConfig10Core, &kNrtCallbackConfig10Core},
      {12, &kMpcConfig12Core, &kRtControlConfig12Core, &kRtCallbackConfig12Core,
       &kNrtLoggingConfig12Core, &kNrtCallbackConfig12Core},
      {14, &kMpcConfig14Core, &kRtControlConfig14Core, &kRtCallbackConfig14Core,
       &kNrtLoggingConfig14Core, &kNrtCallbackConfig14Core},
      {16, &kMpcConfig16Core, &kRtControlConfig16Core, &kRtCallbackConfig16Core,
       &kNrtLoggingConfig16Core, &kNrtCallbackConfig16Core},
  }};

  // What this does NOT assert is that the two nrt lanes sit on cores of their
  // own: layout v5 (#349) deliberately folds nrt_logging / nrt_callback /
  // nrt_publish onto the rt_callback slot, so `nrt_logging != nrt_callback`
  // became a statement about the OLD layout rather than about isolation
  // quality. Removed under PROC-6 / E-6 with the spec change, not to make new
  // code pass -- the property that still matters (MPC solve never shares a
  // core with a CFS lane) is exactly what the two assertions below pin.
  for (const TierSnapshot& tier : tiers) {
    EXPECT_NE(tier.mpc->main.cpu_core, tier.nrt_logging->cpu_core)
        << tier.ncpu << "-core: MPC main shares nrt_logging core";
    EXPECT_NE(tier.mpc->main.cpu_core, tier.nrt_callback->cpu_core)
        << tier.ncpu << "-core: MPC main shares nrt_callback core";
  }
}

// Layout v4.1: rt_callback is the only RT inbound/outbound thread. Pin it to
// Core 2 on every tier (RT cluster starts at Core 1; Core 0 = OS only).
TEST(MpcThreadConfig, LayoutV4RtCallbackPinning) {
  struct TierPin {
    const ThreadConfig* rt_callback;
    int expected_core;
    const char* label;
  };

  const std::array<TierPin, 7> tiers = {{
      {&kRtCallbackConfig4Core, 2, "4-core"},
      {&kRtCallbackConfig, 2, "6-core"},
      {&kRtCallbackConfig8Core, 2, "8-core"},
      {&kRtCallbackConfig10Core, 2, "10-core"},
      {&kRtCallbackConfig12Core, 2, "12-core"},
      {&kRtCallbackConfig14Core, 2, "14-core"},
      {&kRtCallbackConfig16Core, 2, "16-core"},
  }};
  for (const auto& t : tiers) {
    EXPECT_EQ(t.rt_callback->cpu_core, t.expected_core)
        << t.label << ": rt_callback must pin to Core " << t.expected_core;
    EXPECT_EQ(t.rt_callback->sched_policy, SCHED_FIFO)
        << t.label << ": rt_callback must be SCHED_FIFO (layout v4)";
    EXPECT_EQ(t.rt_callback->sched_priority, 70)
        << t.label << ": rt_callback must be FIFO 70 (layout v4)";
  }
}

// Layout v4.1 invariant: Core 0 is reserved for OS / DDS / IRQ on every tier
// ≥ 6-core. nrt_logging / nrt_callback must NOT pin to Core 0 on those tiers.
// 4-core fallback (degraded) is exempt — it intentionally collapses everything
// onto Core 0 due to capacity limits.
TEST(MpcThreadConfig, LayoutV41NrtNotOnCoreZero) {
  struct TierNrt {
    const char* label;
    const ThreadConfig* nrt_logging;
    const ThreadConfig* nrt_callback;
    bool enforce;  // false for 4-core fallback (degraded)
  };

  const std::array<TierNrt, 7> tiers = {{
      {"4-core", &kNrtLoggingConfig4Core, &kNrtCallbackConfig4Core, false},
      {"6-core", &kNrtLoggingConfig, &kNrtCallbackConfig, true},
      {"8-core", &kNrtLoggingConfig8Core, &kNrtCallbackConfig8Core, true},
      {"10-core", &kNrtLoggingConfig10Core, &kNrtCallbackConfig10Core, true},
      {"12-core", &kNrtLoggingConfig12Core, &kNrtCallbackConfig12Core, true},
      {"14-core", &kNrtLoggingConfig14Core, &kNrtCallbackConfig14Core, true},
      {"16-core", &kNrtLoggingConfig16Core, &kNrtCallbackConfig16Core, true},
  }};
  for (const auto& t : tiers) {
    if (!t.enforce) {
      continue;  // 4-core fallback: degraded, nrt may collapse onto Core 0
    }
    EXPECT_NE(t.nrt_logging->cpu_core, 0)
        << t.label << ": nrt_logging must not pin to Core 0 (OS/DDS/IRQ only, v4.1)";
    EXPECT_NE(t.nrt_callback->cpu_core, 0)
        << t.label << ": nrt_callback must not pin to Core 0 (OS/DDS/IRQ only, v4.1)";
  }
}

// Layout v4: arm_driver and hand_driver must not collide with any RT
// controller thread (rt_control / rt_callback / mpc_*). Tiers ≥ 8 give each
// its own dedicated core; tiers below are degraded (4/6-core) and may share
// with OS cores.
TEST(MpcThreadConfig, LayoutV4ArmHandDriverDisjoint) {
  struct TierDrivers {
    const char* label;
    const ThreadConfig* arm;
    const ThreadConfig* hand;
    const ThreadConfig* rt_control;
    const ThreadConfig* rt_callback;
    const MpcThreadConfig* mpc;
    bool dedicated;  // tiers ≥ 8 expect arm/hand on dedicated cores
  };

  const std::array<TierDrivers, 7> tiers = {{
      {"4-core", &kArmDriverConfig4Core, &kHandDriverConfig4Core, &kRtControlConfig4Core,
       &kRtCallbackConfig4Core, &kMpcConfig4Core, false},
      {"6-core", &kArmDriverConfig, &kHandDriverConfig, &kRtControlConfig, &kRtCallbackConfig,
       &kMpcConfig6Core, false},
      {"8-core", &kArmDriverConfig8Core, &kHandDriverConfig8Core, &kRtControlConfig8Core,
       &kRtCallbackConfig8Core, &kMpcConfig8Core, true},
      {"10-core", &kArmDriverConfig10Core, &kHandDriverConfig10Core, &kRtControlConfig10Core,
       &kRtCallbackConfig10Core, &kMpcConfig10Core, true},
      {"12-core", &kArmDriverConfig12Core, &kHandDriverConfig12Core, &kRtControlConfig12Core,
       &kRtCallbackConfig12Core, &kMpcConfig12Core, true},
      {"14-core", &kArmDriverConfig14Core, &kHandDriverConfig14Core, &kRtControlConfig14Core,
       &kRtCallbackConfig14Core, &kMpcConfig14Core, true},
      {"16-core", &kArmDriverConfig16Core, &kHandDriverConfig16Core, &kRtControlConfig16Core,
       &kRtCallbackConfig16Core, &kMpcConfig16Core, true},
  }};
  for (const auto& t : tiers) {
    if (!t.dedicated) {
      continue;  // 4/6-core: degraded, arm/hand may share with OS cores
    }
    EXPECT_NE(t.arm->cpu_core, t.rt_control->cpu_core) << t.label << ": arm shares rt_control core";
    EXPECT_NE(t.arm->cpu_core, t.rt_callback->cpu_core)
        << t.label << ": arm shares rt_callback core";
    EXPECT_NE(t.arm->cpu_core, t.mpc->main.cpu_core) << t.label << ": arm shares mpc_main core";
    EXPECT_NE(t.hand->cpu_core, t.rt_control->cpu_core)
        << t.label << ": hand shares rt_control core";
    EXPECT_NE(t.hand->cpu_core, t.rt_callback->cpu_core)
        << t.label << ": hand shares rt_callback core";
    EXPECT_NE(t.hand->cpu_core, t.mpc->main.cpu_core) << t.label << ": hand shares mpc_main core";
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
  // Validate the MPC-specific invariants directly (priority rules)
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
  // Force arm_driver onto rt_callback's core — a clear violation of the
  // Phase 5 disjointness rule.
  bad.arm_driver.cpu_core = bad.rt_callback.cpu_core;
  const std::string err = ValidateSystemThreadConfigs(bad);
  EXPECT_FALSE(err.empty()) << "Validator missed arm_driver/rt_callback core collision";
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
