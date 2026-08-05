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
// file pins relations (worker <= main, mpc < rt_callback, nrt off Core 0,
// arm/hand disjoint from RT, monotonicity across tiers) plus the MPC main core
// and worker count. What no C++ test pinned before M1 was the *exact* per-tier
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
  int num_workers;
  Expected worker0;  // ignored when num_workers < 1
  Expected worker1;  // ignored when num_workers < 2
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
       0,
       {},
       {},
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
       0,
       {},
       {},
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
       0,
       {},
       {},
       {0, kOther, 0, 0},
       {0, kOther, 0, 0},
       {0, kOther, 0, -5},
       {0, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 6-core (6-7): mpc gains FIFO 60; arm/hand share a slot, nrt_* share one.
      {6,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       0,
       {},
       {},
       {4, kOther, 0, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, -5},
       {5, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {7,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       0,
       {},
       {},
       {4, kOther, 0, 0},
       {4, kOther, 0, 0},
       {5, kOther, 0, -5},
       {5, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 8-core (8-9): arm/hand get dedicated slots, nrt_* split. No workers.
      {8,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       0,
       {},
       {},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {6, kOther, 0, -5},
       {7, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {9,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       0,
       {},
       {},
       {4, kOther, 0, 0},
       {5, kOther, 0, 0},
       {6, kOther, 0, -5},
       {7, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 10-core (10-11): first parallel MPC worker.
      {10,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       1,
       {4, kFifo, 55, 0},
       {},
       {5, kOther, 0, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, -5},
       {8, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {11,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       1,
       {4, kFifo, 55, 0},
       {},
       {5, kOther, 0, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, -5},
       {8, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 12-core (12-13): primary target, both workers.
      {12,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {13,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},

      // ── 14-core (14-15) and 16-core (16+): same assignment as 12; the extra
      //    slots stay spare.
      {14,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {15,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {16,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {17,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
       {-1, kOther, 0, 0},
       {-1, kOther, 0, 0}},
      {24,
       {1, kFifo, 90, 0},
       {2, kFifo, 70, 0},
       {3, kFifo, 60, 0},
       2,
       {4, kFifo, 55, 0},
       {5, kFifo, 55, 0},
       {6, kOther, 0, 0},
       {7, kOther, 0, 0},
       {8, kOther, 0, -5},
       {9, kOther, 0, 0},
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
    ExpectMatches(ncpu + " sim_thread", cfgs.sim_thread, tier.sim_thread);
    ExpectMatches(ncpu + " viewer", cfgs.viewer, tier.viewer);

    EXPECT_EQ(cfgs.mpc.num_workers, tier.num_workers) << ncpu << ": mpc.num_workers";
    if (tier.num_workers >= 1) {
      ExpectMatches(ncpu + " mpc_worker_0", cfgs.mpc.workers[0], tier.worker0);
    }
    if (tier.num_workers >= 2) {
      ExpectMatches(ncpu + " mpc_worker_1", cfgs.mpc.workers[1], tier.worker1);
    }
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
    EXPECT_STREQ(cfgs.arm_driver.name, "arm_driver") << ncpu;
    EXPECT_STREQ(cfgs.hand_driver.name, "hand_driver") << ncpu;
    EXPECT_STREQ(cfgs.sim_thread.name, "sim_thread") << ncpu;
    EXPECT_STREQ(cfgs.viewer.name, "viewer") << ncpu;
    EXPECT_STREQ(cfgs.mpc.main.name, "mpc_main") << ncpu;
    for (int i = 0; i < cfgs.mpc.num_workers; ++i) {
      EXPECT_EQ(std::string(cfgs.mpc.workers[static_cast<std::size_t>(i)].name),
                "mpc_worker_" + std::to_string(i))
          << ncpu;
    }

    // TASK_COMM_LEN - 1. A longer name is silently truncated by the kernel, so
    // the verifier would match a prefix it never expects.
    for (const rtc::ThreadConfig* cfg :
         {&cfgs.rt_control, &cfgs.rt_callback, &cfgs.nrt_logging, &cfgs.nrt_callback,
          &cfgs.arm_driver, &cfgs.hand_driver, &cfgs.sim_thread, &cfgs.viewer, &cfgs.mpc.main}) {
      EXPECT_LE(std::string(cfg->name).size(), 15U) << ncpu << ": " << cfg->name;
    }
  }
}

// A manifest edit must not introduce an RT/RT same-priority collision or an
// arm/hand pin that lands on an RT controller core.
//
// Two pre-existing properties of the validator bound what this can assert, and
// neither is something M1 introduced (M1 changed no layout value):
//
//  1. ValidateThreadConfig() range-checks cpu_core against *this host's* online
//     CPU count, so a tier above the host's size always fails. That is why
//     test_mpc_thread_config.cpp gates its own validation on the host tier;
//     the same gate applies here.
//  2. On tiers with num_workers == 0 the inactive workers[] entries are
//     zero-initialised, so their cpu_core is 0 -- and the arm/hand
//     disjointness sweep keys off the hard-coded NamedConfig *name*
//     ("mpc_worker_0"), not on whether that worker is active. On every tier
//     where arm/hand also sit on slot 0 (ncpu <= 5) the validator therefore
//     reports a collision against a worker that does not exist. The comment
//     above that array reasons only about the RT/RT same-priority check, which
//     the all-zero policy does make unreachable -- the disjointness check was
//     not considered. Reported on issue #153; fixing it changes validator
//     behaviour, which is outside M1's "pure representation change" scope.
TEST(ThreadLayoutTiers, TiersThatFitThisHostPassTheValidator) {
  const int host_cores = rtc::GetPhysicalCpuCount();
  int checked = 0;
  for (const TierExpectation& tier : AllTiers()) {
    const rtc::SystemThreadConfigs cfgs = rtc::SelectThreadConfigsForCoreCount(tier.ncpu);

    // (1) skip tiers whose slots this host cannot represent.
    const int highest =
        std::max({cfgs.rt_control.cpu_core, cfgs.rt_callback.cpu_core, cfgs.mpc.main.cpu_core,
                  cfgs.arm_driver.cpu_core, cfgs.hand_driver.cpu_core, cfgs.nrt_logging.cpu_core,
                  cfgs.nrt_callback.cpu_core});
    if (highest >= host_cores) {
      continue;
    }
    // (2) skip the degraded tiers where the inactive-worker false positive fires.
    if (cfgs.mpc.num_workers == 0 && cfgs.arm_driver.cpu_core == 0) {
      continue;
    }

    const std::string err = rtc::ValidateSystemThreadConfigs(cfgs);
    EXPECT_TRUE(err.empty()) << "ncpu=" << tier.ncpu << " failed validation: " << err;
    ++checked;
  }
  // A fixture that silently checks nothing is the failure mode this guards:
  // every host is at least 4-core, so the 4-core tier is always reachable...
  // except that (2) excludes it. Assert we still exercised the validator on at
  // least one tier, otherwise this test is decoration.
  EXPECT_GT(checked, 0) << "no tier fit this " << host_cores
                        << "-core host -- the validator was never exercised";
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
  EXPECT_EQ(probed.mpc.num_workers, explicit_count.mpc.num_workers);
  EXPECT_EQ(probed.nrt_logging.cpu_core, explicit_count.nrt_logging.cpu_core);
  EXPECT_EQ(probed.nrt_callback.cpu_core, explicit_count.nrt_callback.cpu_core);
  EXPECT_EQ(probed.arm_driver.cpu_core, explicit_count.arm_driver.cpu_core);
  EXPECT_EQ(probed.hand_driver.cpu_core, explicit_count.hand_driver.cpu_core);
}
