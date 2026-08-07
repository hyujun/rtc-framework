// Value oracle for the C++ side of the generated thread layout (issue #153 M1).
//
// The tier table now lives in repo_scripts/config/thread_layout.yaml and is
// generated into thread_config_generated.hpp; the shell helpers and the Python
// launch mirror come from the same manifest. `gen_thread_layout.py --check`
// proves the three encodings are in sync, but it cannot prove they are
// *correct* -- a wrong manifest entry matches itself in all three languages.
// Each language therefore keeps a hand-written literal oracle. This is the C++
// one.
//
// It complements test_mpc_thread_config.cpp rather than duplicating it. That
// file pins relations (mpc < rt_callback, nrt off Core 0, arm/hand disjoint
// from RT, monotonicity across tiers) plus the MPC main core. What no C++ test
// pinned before M1 was the *exact* per-tier
// slot of arm_driver / hand_driver / nrt_logging / nrt_callback -- verified by
// mutation: moving nrt_logging from slot 8 to 7 on the 12-core tier left every
// C++ test green while the shell oracle went red on three axes.
//
// SelectThreadConfigsForCoreCount() takes the core count explicitly (the
// generated dispatch; SelectThreadConfigs() is the runtime-probing wrapper), so
// this sweeps every tier including the ones this machine does not have.

#include "rtc_base/threading/thread_config.hpp"
// ValidateSystemThreadConfigs / SelectThreadConfigs / GetPhysicalCpuCount: the
// runtime side stayed in thread_utils.hpp when the tier table moved out.
#include "rtc_base/threading/thread_utils.hpp"

#include <gtest/gtest.h>
#include <sched.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace {

struct Expected {
  int slot;
  int policy;
  int priority;
  int nice;
};

struct TierExpectation {
  int ncpu;  // physical core count fed to the dispatch
  Expected rt_control;
  Expected rt_callback;
  Expected mpc_main;
  Expected arm_driver;
  Expected hand_driver;
  Expected nrt_logging;
  Expected nrt_callback;
  Expected sim_thread;
  Expected viewer;
};

constexpr int kOther = SCHED_OTHER;
constexpr int kFifo = SCHED_FIFO;

// Every core count in 1..17 plus 24, so both sides of each tier boundary are
// covered (5/6, 7/8, 9/10, 11/12, 13/14, 15/16) along with the clamp above 16.
const std::vector<TierExpectation>& AllTiers() {
  static const std::vector<TierExpectation> kTiers = {
      // ── 4-core fallback (ncpu <= 5): degraded, mpc demoted to CFS, nrt and
      //    both driver processes collapse onto the OS slot.
      {1,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kOther, 0, -5},
       {0, kOther, 0, 0},
       {0, kOther, 0, 0},
       {0, kOther, 0, -5},
       {0, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {4,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kOther, 0, -5},
       {0, kOther, 0, 0},
       {0, kOther, 0, 0},
       {0, kOther, 0, -5},
       {0, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {5,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kOther, 0, -5},
       {0, kOther, 0, 0},
       {0, kOther, 0, 0},
       {0, kOther, 0, -5},
       {0, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 6-core (6-7): mpc gains FIFO 60; arm/hand share a slot. nrt_* sit on
      //    the v5 aux slot 2 like every tier above (issue #349).
      {6,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {4, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {7,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {4, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 8-core (8-9): arm/hand get dedicated slots (4-5). nrt_* stay on the
      //    aux slot, so slots 6-7 are spare here (v4.1 spent them on nrt).
      //    Since v6 (#383) every tier above repeats this exact pair.
      {8,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {9,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 10-core (10-11): identical to 8-core. Slot 4 held mpc_worker_0 until
      //    #380 and stood empty until #383 moved arm_driver into it, so nothing
      //    shifts between tiers any more -- the rows below 10 and above it are
      //    the same assignment, and only the spare tail grows.
      {10,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {11,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 12-core (12-13): primary target. Slots 4-5 are the driver pair
      //    since v6 (#383); they held mpc_worker_0/1 until #380. On NUC13
      //    (4P+8E) that is logical 8,9 -- E-cores, outside the shield.
      {12,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {13,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 14-core (14-15) and 16-core (16+): same assignment as 12; the extra
      //    slots stay spare.
      {14,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {15,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {16,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {17,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {24,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {2, kOther, 0, -5},
       {2, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
  };
  return kTiers;
}

void ExpectMatches(const std::string& label, const rtc::ThreadConfig& actual,
                   const Expected& expected) {
  EXPECT_EQ(actual.cpu_core, expected.slot) << label << ": cpu_core";
  EXPECT_EQ(actual.sched_policy, expected.policy) << label << ": sched_policy";
  EXPECT_EQ(actual.sched_priority, expected.priority) << label << ": sched_priority";
  EXPECT_EQ(actual.nice_value, expected.nice) << label << ": nice_value";
}

}  // namespace

// Every role's slot / policy / priority / nice, on every core count in
// 1..17 and 24. A single wrong manifest entry fails here even when the three
// generated encodings agree with each other.
TEST(ThreadLayoutTiers, EveryRoleMatchesTheSpecOnEveryTier) {
  for (const TierExpectation& tier : AllTiers()) {
    const rtc::SystemThreadConfigs cfgs = rtc::SelectThreadConfigsForCoreCount(tier.ncpu);
    const std::string ncpu = "ncpu=" + std::to_string(tier.ncpu);

    ExpectMatches(ncpu + " rt_control", cfgs.rt_control, tier.rt_control);
    ExpectMatches(ncpu + " rt_callback", cfgs.rt_callback, tier.rt_callback);
    ExpectMatches(ncpu + " mpc_main", cfgs.mpc.main, tier.mpc_main);
    ExpectMatches(ncpu + " arm_driver", cfgs.arm_driver, tier.arm_driver);
    ExpectMatches(ncpu + " hand_driver", cfgs.hand_driver, tier.hand_driver);
    ExpectMatches(ncpu + " nrt_logging", cfgs.nrt_logging, tier.nrt_logging);
    ExpectMatches(ncpu + " nrt_callback", cfgs.nrt_callback, tier.nrt_callback);
    // nrt_publish is specified *relative* to nrt_callback — same slot, same
    // policy, same priority, different name — so it is pinned that way rather
    // than by an 18th column of literals repeating the one beside it. The
    // value is still nailed down: tier.nrt_callback above is a literal, so a
    // wrong manifest slot for either role fails one of these two checks.
    // Should nrt_publish ever earn its own core, this is the assertion that
    // forces the change to be deliberate.
    ExpectMatches(ncpu + " nrt_publish", cfgs.nrt_publish, tier.nrt_callback);
    ExpectMatches(ncpu + " sim_thread", cfgs.sim_thread, tier.sim_thread);
    ExpectMatches(ncpu + " viewer", cfgs.viewer, tier.viewer);
  }
}

// Thread names reach pthread_setname_np and are what verify_rt_runtime.sh
// matches against /proc/<pid>/task/<tid>/comm. A renamed thread makes the
// verifier report a missing thread on a perfectly healthy box.
TEST(ThreadLayoutTiers, ThreadNamesAreStableAndWithinTaskCommLen) {
  for (const TierExpectation& tier : AllTiers()) {
    const rtc::SystemThreadConfigs cfgs = rtc::SelectThreadConfigsForCoreCount(tier.ncpu);
    const std::string ncpu = "ncpu=" + std::to_string(tier.ncpu);

    EXPECT_STREQ(cfgs.rt_control.name, "rt_control") << ncpu;
    EXPECT_STREQ(cfgs.rt_callback.name, "rt_callback") << ncpu;
    EXPECT_STREQ(cfgs.nrt_logging.name, "nrt_logging") << ncpu;
    EXPECT_STREQ(cfgs.nrt_callback.name, "nrt_callback") << ncpu;
    // The whole reason nrt_publish exists as its own role: a distinct comm.
    // If this ever equals "nrt_callback" again, verify_rt_runtime.sh silently
    // goes back to checking one of the two threads (issue #349 D15).
    EXPECT_STREQ(cfgs.nrt_publish.name, "nrt_publish") << ncpu;
    EXPECT_STRNE(cfgs.nrt_publish.name, cfgs.nrt_callback.name) << ncpu;
    EXPECT_STREQ(cfgs.arm_driver.name, "arm_driver") << ncpu;
    EXPECT_STREQ(cfgs.hand_driver.name, "hand_driver") << ncpu;
    EXPECT_STREQ(cfgs.sim_thread.name, "sim_thread") << ncpu;
    EXPECT_STREQ(cfgs.viewer.name, "viewer") << ncpu;
    EXPECT_STREQ(cfgs.mpc.main.name, "mpc_main") << ncpu;

    // TASK_COMM_LEN - 1. A longer name is silently truncated by the kernel, so
    // the verifier would match a prefix it never expects.
    for (const rtc::ThreadConfig* cfg :
         {&cfgs.rt_control, &cfgs.rt_callback, &cfgs.nrt_logging, &cfgs.nrt_callback,
          &cfgs.nrt_publish, &cfgs.arm_driver, &cfgs.hand_driver, &cfgs.sim_thread, &cfgs.viewer,
          &cfgs.mpc.main}) {
      EXPECT_LE(std::string(cfg->name).size(), 15U) << ncpu << ": " << cfg->name;
    }
  }
}

// A manifest edit must not introduce an RT/RT collision, an arm/hand pin that
// lands on an RT controller core, or a co-residency violation (issue #349).
//
// One pre-existing property of the validator still bounds what this can
// assert, and it is not something M1 introduced (M1 changed no layout value):
// ValidateThreadConfig() range-checks cpu_core against *this host's* online CPU
// count, so a tier above the host's size always fails. That is why
// test_mpc_thread_config.cpp gates its own validation on the host tier; the
// same gate applies here.
//
// The second bound is gone. Tiers without MPC workers used to be skipped
// because the zero-initialised workers[] entries sat on cpu_core 0 and the
// arm/hand disjointness sweep classified them by hard-coded name, reporting
// collisions against workers that do not exist. #349 gated those entries out
// of the NamedConfig table; #380 removed the worker slots outright, so the
// shape cannot recur. Either way the 4-core tier -- the one that parks
// arm/hand on slot 0, and the one this fixture could not look at -- is now
// checked like any other, and `err.empty()` below is a strictly stronger
// assertion than the dedicated #349 regression test it replaced.
TEST(ThreadLayoutTiers, TiersThatFitThisHostPassTheValidator) {
  const int host_cores = rtc::GetPhysicalCpuCount();
  int checked = 0;
  int skipped_too_large = 0;
  for (const TierExpectation& tier : AllTiers()) {
    const rtc::SystemThreadConfigs cfgs = rtc::SelectThreadConfigsForCoreCount(tier.ncpu);

    // Skip tiers whose slots this host cannot represent.
    const int highest =
        std::max({cfgs.rt_control.cpu_core, cfgs.rt_callback.cpu_core, cfgs.mpc.main.cpu_core,
                  cfgs.arm_driver.cpu_core, cfgs.hand_driver.cpu_core, cfgs.nrt_logging.cpu_core,
                  cfgs.nrt_callback.cpu_core});
    if (highest >= host_cores) {
      ++skipped_too_large;
      continue;
    }

    const std::string err = rtc::ValidateSystemThreadConfigs(cfgs);
    EXPECT_TRUE(err.empty()) << "ncpu=" << tier.ncpu << " failed validation: " << err;
    ++checked;
  }
  // A fixture that silently checks nothing is the failure mode this guards --
  // but on this axis "checked nothing" is a property of the *host*, not of the
  // code under review. The lowest tier is now reachable (highest slot 3), so
  // this only trips on a host with fewer than four physical cores. Failing
  // there would turn the gated merge job red for an environment reason.
  // Report it as a skip instead -- visible in the test report, never a silent
  // pass -- and note where the axis stays covered: every rule asserted here is
  // also asserted at the manifest level by `gen_thread_layout.py --self-test`,
  // which needs no host at all.
  if (checked == 0) {
    GTEST_SKIP() << "no tier fits this " << host_cores << "-core host (" << skipped_too_large
                 << " tiers larger than the host); the same rules are covered "
                    "host-independently by gen_thread_layout.py --self-test";
  }
}

// Drops the "Invalid CPU core slot ..." entries from a validator verdict.
//
// That single rule range-checks cpu_core against *this host* (see
// ValidateThreadConfig in thread_utils.hpp) instead of against the layout, so
// a box with fewer physical slots than a tier uses trips it for an
// environment reason. Every other thing the validator says -- co-residency,
// RT-priority ordering, driver collisions, nice range, names -- is a property
// of the layout alone and must hold on any host.
//
// Entries are appended with a "; " terminator. One message ("carries two RT
// roles (...); at most one is allowed; ") contains that separator internally
// and therefore splits into two fragments; both are kept, because only a
// fragment that *starts* with the host-range prefix is dropped.
std::string DropHostRangeErrors(const std::string& errors) {
  std::string kept;
  std::size_t pos = 0;
  while (pos < errors.size()) {
    const std::size_t sep = errors.find("; ", pos);
    const std::size_t next = (sep == std::string::npos) ? errors.size() : sep + 2;
    const std::string entry = errors.substr(pos, next - pos);
    if (entry.rfind("Invalid CPU core slot", 0) != 0) {
      kept += entry;
    }
    pos = next;
  }
  return kept;
}

// The 4-core tier still needs its own unconditional check. It is the one that
// parks arm/hand on slot 0, and the host gate above skips whatever a small box
// cannot represent -- so on a large host this is redundant and on a 3-core one
// it is the only thing left. It replaces the #349 regression test that pinned
// the inactive-worker false positive: that shape is gone with the worker slots
// (#380), and asserting the rest of the validator is silent subsumes asserting
// that it merely omits two names.
//
// The verdict is filtered rather than compared against "" because slot 3 does
// not exist on a host with fewer than four physical cores -- the gated CI
// runner is exactly that box, and comparing against "" turned the merge job
// red there while nothing was wrong with the layout. The predecessor test
// asserted substring *absence* and was host-independent for that reason; the
// filter keeps the stronger claim (no error of any kind, not just two names)
// without reintroducing the host dependency this file warns about twenty
// lines below, where MakeCoResidencyFixture documents the same trap.
TEST(ThreadLayoutTiers, FourCoreTierWithDriversOnTheOsSlotPassesTheValidator) {
  const rtc::SystemThreadConfigs cfgs = rtc::SelectThreadConfigsForCoreCount(4);
  ASSERT_EQ(cfgs.arm_driver.cpu_core, 0) << "4-core tier is expected to park arm_driver on slot 0";
  ASSERT_EQ(cfgs.hand_driver.cpu_core, 0)
      << "4-core tier is expected to park hand_driver on slot 0";

  const std::string err = rtc::ValidateSystemThreadConfigs(cfgs);
  EXPECT_EQ(DropHostRangeErrors(err), "") << "4-core tier failed validation: " << err;
}

// DropHostRangeErrors is what keeps the test above honest on a box smaller
// than the lowest tier -- which is exactly the box this test suite does not
// run on when it would matter. Pin the filter itself against synthetic
// verdicts so the mechanism is covered on every host, not just small ones.
TEST(ThreadLayoutTiers, DropHostRangeErrorsRemovesOnlyTheHostRangeRule) {
  const std::string range = "Invalid CPU core slot 3 (valid range: -1, 0-1); ";

  EXPECT_EQ(DropHostRangeErrors(""), "");
  EXPECT_EQ(DropHostRangeErrors(range), "");
  EXPECT_EQ(DropHostRangeErrors(range + range), "");

  // A layout-level finding survives, alone and mixed with host-range noise.
  // If it did not, the filter would have turned the 4-core check vacuous --
  // the failure mode that matters more than the red job it replaced.
  const std::string real = "'nrt_publish' shares rt_control's core 1; ";
  EXPECT_EQ(DropHostRangeErrors(real), real);
  EXPECT_EQ(DropHostRangeErrors(range + real), real);
  EXPECT_EQ(DropHostRangeErrors(real + range), real);

  // The one validator message that embeds "; " internally must survive whole.
  const std::string two_rt = "core 2 carries two RT roles ('a', 'b'); at most one is allowed; ";
  EXPECT_EQ(DropHostRangeErrors(two_rt), two_rt);
  EXPECT_EQ(DropHostRangeErrors(range + two_rt), two_rt);
}

// A synthetic layout for the co-residency cases below, built by hand instead
// of taken from SelectThreadConfigsForCoreCount(host).
//
// Binding those cases to the host's own tier made two of them host-dependent:
//
//   - The 4/5-core tiers demote mpc_main to SCHED_OTHER, so "collide
//     rt_callback with mpc_main" puts two RT roles on one core on a 12-core
//     box and one RT role next to a CFS one on a 4-core box. The rule
//     correctly stays silent there and the assertion fails -- for an
//     environment reason, on a machine where nothing is wrong.
//   - ValidateThreadConfig range-checks cpu_core against *this host*, so on a
//     box with fewer cores than the lowest tier's highest slot the
//     precondition "the base layout is clean" is false before any mutation.
//
// The shape below mirrors the 4-core tier (RT on slots 1-3, everything CFS
// collapsed onto the OS slot), so it is clean on any host with four physical
// cores and, being fixed, exercises the same rule on every machine. Each case
// asserts its own substring appears only *after* the mutation rather than
// asserting the whole error string is empty, so a host smaller than four
// cores adds "Invalid CPU core slot" noise without flipping any verdict.
// Whether the shipped tiers themselves are clean is a separate question,
// covered by TiersThatFitThisHostPassTheValidator (host-gated) and by
// gen_thread_layout.py --self-test (host-independent, every tier).
rtc::SystemThreadConfigs MakeCoResidencyFixture() {
  const auto cfs = [](int slot, int nice, const char* name) {
    return rtc::ThreadConfig{.cpu_core = slot,
                             .sched_policy = SCHED_OTHER,
                             .sched_priority = 0,
                             .nice_value = nice,
                             .name = name};
  };
  rtc::SystemThreadConfigs cfg{};
  cfg.rt_control = {.cpu_core = 1,
                    .sched_policy = SCHED_FIFO,
                    .sched_priority = 90,
                    .nice_value = 0,
                    .name = "rt_control"};
  cfg.rt_callback = {.cpu_core = 2,
                     .sched_policy = SCHED_FIFO,
                     .sched_priority = 70,
                     .nice_value = 0,
                     .name = "rt_callback"};
  cfg.mpc.main = {.cpu_core = 3,
                  .sched_policy = SCHED_FIFO,
                  .sched_priority = 60,
                  .nice_value = 0,
                  .name = "mpc_main"};
  cfg.nrt_logging = cfs(rtc::kOsSlot, -5, "nrt_logging");
  cfg.nrt_callback = cfs(rtc::kOsSlot, 0, "nrt_callback");
  cfg.nrt_publish = cfs(rtc::kOsSlot, 0, "nrt_publish");
  cfg.arm_driver = cfs(rtc::kOsSlot, 0, "arm_driver");
  cfg.hand_driver = cfs(rtc::kOsSlot, 0, "hand_driver");
  cfg.sim_thread = cfs(-1, 0, "sim_thread");
  cfg.viewer = cfs(-1, 0, "viewer");
  return cfg;
}

// Co-residency rules (issue #349 D-co). These mirror
// gen_thread_layout.py::run_self_test; the C++ copy is what a deployment box
// evaluates at startup for its own tier, the Python copy is what CI can reach
// for every tier. Both are needed -- neither covers the other's blind spot.
TEST(ThreadLayoutTiers, ValidatorRejectsCoResidencyViolations) {
  const rtc::SystemThreadConfigs base = MakeCoResidencyFixture();

  // Negative control: the fixture must not already carry the signatures the
  // mutations below look for, or every case would pass without mutating
  // anything. Asserted per-substring rather than on emptiness so this stays
  // meaningful on a host too small to represent slot 3.
  const std::string clean = rtc::ValidateSystemThreadConfigs(base);
  for (const char* signature :
       {"shares rt_control's core", "carries two RT roles", "is RT on the OS/DDS/IRQ core"}) {
    ASSERT_EQ(clean.find(signature), std::string::npos)
        << "fixture already trips '" << signature << "': " << clean;
  }

  // 1. Nothing may share rt_control's core.
  {
    rtc::SystemThreadConfigs bad = base;
    bad.nrt_logging.cpu_core = bad.rt_control.cpu_core;
    EXPECT_NE(rtc::ValidateSystemThreadConfigs(bad).find("shares rt_control's core"),
              std::string::npos);
  }
  // 1b. Including nrt_publish. It is the role #349 added, it shares
  //     nrt_callback's slot on every shipped tier, and it was invisible to
  //     this validator until it joined the all_configs table -- so the rule
  //     that guards the control loop's core skipped it entirely.
  {
    rtc::SystemThreadConfigs bad = base;
    bad.nrt_publish.cpu_core = bad.rt_control.cpu_core;
    EXPECT_NE(rtc::ValidateSystemThreadConfigs(bad).find("'nrt_publish' shares rt_control's core"),
              std::string::npos);
  }
  // 2. Two RT roles on one core, even at different priorities.
  {
    rtc::SystemThreadConfigs bad = base;
    bad.rt_callback.cpu_core = bad.mpc.main.cpu_core;
    EXPECT_NE(rtc::ValidateSystemThreadConfigs(bad).find("carries two RT roles"),
              std::string::npos);
  }
  // 3. An RT role on the OS / DDS / IRQ core.
  {
    rtc::SystemThreadConfigs bad = base;
    bad.rt_callback.cpu_core = rtc::kOsSlot;
    EXPECT_NE(rtc::ValidateSystemThreadConfigs(bad).find("is RT on the OS/DDS/IRQ core"),
              std::string::npos);
  }
  // 4. Per-config validation reaches nrt_publish too: an out-of-range slot on
  //    that role used to slip past ValidateSystemThreadConfigs entirely and
  //    only surface as an ApplyThreadConfig failure at thread start.
  {
    rtc::SystemThreadConfigs bad = base;
    bad.nrt_publish.cpu_core = 1 << 20;
    EXPECT_NE(rtc::ValidateSystemThreadConfigs(bad).find("Invalid CPU core slot"),
              std::string::npos);
  }
}

// The wrapper must agree with the explicit-count overload for this host, or
// production would run a different layout than every test above checks.
TEST(ThreadLayoutTiers, RuntimeWrapperAgreesWithTheExplicitOverload) {
  const rtc::SystemThreadConfigs probed = rtc::SelectThreadConfigs();
  const rtc::SystemThreadConfigs explicit_count =
      rtc::SelectThreadConfigsForCoreCount(rtc::GetPhysicalCpuCount());
  EXPECT_EQ(probed.rt_control.cpu_core, explicit_count.rt_control.cpu_core);
  EXPECT_EQ(probed.rt_callback.cpu_core, explicit_count.rt_callback.cpu_core);
  EXPECT_EQ(probed.mpc.main.cpu_core, explicit_count.mpc.main.cpu_core);
  EXPECT_EQ(probed.nrt_logging.cpu_core, explicit_count.nrt_logging.cpu_core);
  EXPECT_EQ(probed.nrt_callback.cpu_core, explicit_count.nrt_callback.cpu_core);
  EXPECT_EQ(probed.arm_driver.cpu_core, explicit_count.arm_driver.cpu_core);
  EXPECT_EQ(probed.hand_driver.cpu_core, explicit_count.hand_driver.cpu_core);
}
