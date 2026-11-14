/// @file test_mpc_solver_pool.cpp
/// @brief The MPC thread's OpenMP solver pool: created, named and pinned at
///        loop entry, refused when it cannot come up as configured, and absent
///        on the one-thread path.
///
/// The pool does not depend on the solver: an MPC thread with any Solve() runs
/// the same loop entry, so the mock thread (and a GEMM thread for Eigen's own
/// pool) exercise it without Aligator. Threads are found through
/// /proc/self/task by name, and their placement is read back from the kernel.
/// The pins use SCHED_OTHER configs, so the suite needs no RT permission; the
/// policy rows therefore compare SCHED_OTHER with SCHED_OTHER here, and the
/// FIFO case is judged on the control PCs.

#include "rtc_base/testing/wait_until.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/thread/mock_mpc_thread.hpp"
#include "rtc_mpc/thread/mpc_thread.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sched.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace rtc::mpc {
namespace {

using rtc::testing::WaitUntil;

constexpr int kNq = 3;
constexpr int kNv = 3;

struct Task {
  pid_t tid{0};
  std::string comm;
  std::vector<int> cpus;
  int policy{-1};
};

std::string Comm(const std::filesystem::path& task_dir) {
  std::ifstream in(task_dir / "comm");
  std::string s;
  std::getline(in, s);
  return s;
}

/// This process's threads whose name starts with `prefix`.
std::vector<Task> Tasks(const std::string& prefix) {
  std::vector<Task> out;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task", ec)) {
    Task t;
    t.comm = Comm(entry.path());
    if (t.comm.rfind(prefix, 0) != 0) {
      continue;
    }
    t.tid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(t.tid, sizeof(set), &set) == 0) {
      for (int c = 0; c < CPU_SETSIZE; ++c) {
        if (CPU_ISSET(static_cast<std::size_t>(c), &set)) {
          t.cpus.push_back(c);
        }
      }
    }
    t.policy = sched_getscheduler(t.tid);
    out.push_back(t);
  }
  std::sort(out.begin(), out.end(), [](const Task& a, const Task& b) { return a.comm < b.comm; });
  return out;
}

std::set<pid_t> Tids(const std::vector<Task>& tasks) {
  std::set<pid_t> s;
  for (const Task& t : tasks) {
    s.insert(t.tid);
  }
  return s;
}

int HostSlots() {
  const auto& slots = rtc::GetCpuTopology().physical_core_slots;
  return slots.empty() ? static_cast<int>(rtc::GetOnlineCpuCount())
                       : static_cast<int>(slots.size());
}

MpcThreadLaunchConfig Launch(const char* name, int threads, std::vector<int> worker_slots) {
  MpcThreadLaunchConfig l{};
  l.main.cpu_core = 0;
  l.main.sched_policy = SCHED_OTHER;
  l.main.sched_priority = 0;
  l.main.nice_value = 0;
  l.main.name = name;
  l.target_frequency_hz = 500.0;
  l.solver_threads = threads;
  for (std::size_t i = 0; i < worker_slots.size(); ++i) {
    l.workers.slots[i] = worker_slots[i];
  }
  l.workers.count = static_cast<int>(worker_slots.size());
  return l;
}

class SolverPool : public ::testing::Test {
 protected:
  void SetUp() override {
    YAML::Node cfg;
    cfg["enabled"] = true;
    mgr_.Init(cfg, kNq, kNv, 0);
    mgr_.WriteState(Eigen::VectorXd::Zero(kNq), Eigen::VectorXd::Zero(kNv), 0);
    mock_.Configure(kNq, kNv, /*horizon=*/10, /*dt_node=*/0.01);
    mock_.SetTarget(Eigen::VectorXd::Zero(kNq));
  }

  MPCSolutionManager mgr_;
  MockMPCThread mock_;
};

TEST_F(SolverPool, ComesUpNamedAndPinnedAndStaysThroughTicksAndRestarts) {
  if (HostSlots() < 3) {
    GTEST_SKIP() << "needs 3 slots for the MPC thread and two workers (host has " << HostSlots()
                 << ")";
  }
  const std::vector<int> slots = {1, 2};
  mock_.Init(mgr_, Launch("mpc_pool_a", 3, slots));
  mock_.Start();
  ASSERT_TRUE(WaitUntil([&] { return mock_.PoolState() != SolverPoolState::kPending; }));
  ASSERT_EQ(mock_.PoolState(), SolverPoolState::kReady);

  const std::vector<Task> workers = Tasks("mpc_worker_");
  ASSERT_EQ(workers.size(), 2U) << "exactly solver_threads - 1 workers";
  for (std::size_t i = 0; i < workers.size(); ++i) {
    EXPECT_EQ(workers[i].comm, "mpc_worker_" + std::to_string(i));
    EXPECT_EQ(workers[i].cpus, std::vector<int>{rtc::SlotToLogicalCpu(slots[i])})
        << workers[i].comm << " is not on exactly its slot's CPU";
    EXPECT_EQ(workers[i].policy, SCHED_OTHER) << workers[i].comm;
  }
  // The workers are their own threads, not renamed copies of the MPC thread.
  const std::vector<Task> main = Tasks("mpc_pool_a");
  ASSERT_EQ(main.size(), 1U) << "only the MPC thread itself keeps its name";
  EXPECT_EQ(Tids(workers).count(main.front().tid), 0U);

  // A hundred ticks later the pool is the same set of threads.
  const std::set<pid_t> before = Tids(workers);
  ASSERT_TRUE(WaitUntil([&] { return mock_.TickCount() >= 100; }, std::chrono::seconds(5)));
  EXPECT_EQ(Tids(Tasks("mpc_worker_")), before);

  // A restart builds a fresh pool on the fresh loop thread; the old one goes.
  mock_.StopAndJoin();
  EXPECT_TRUE(WaitUntil([&] { return Tasks("mpc_worker_").empty(); }))
      << "the pool outlived its MPC thread";
  mock_.Start();
  ASSERT_TRUE(WaitUntil([&] { return mock_.PoolState() == SolverPoolState::kReady; }));
  const std::vector<Task> again = Tasks("mpc_worker_");
  EXPECT_EQ(again.size(), 2U);
  for (const Task& t : again) {
    EXPECT_EQ(before.count(t.tid), 0U) << "a worker of the stopped pool is still alive";
  }
  mock_.StopAndJoin();
}

// Each refusal ends the loop before its first tick and names the verdict.
TEST_F(SolverPool, RefusesFewerSlotsThanWorkers) {
  mock_.Init(mgr_, Launch("mpc_pool_b", 3, {1}));
  mock_.Start();
  ASSERT_TRUE(WaitUntil([&] { return mock_.PoolState() != SolverPoolState::kPending; }));
  EXPECT_EQ(mock_.PoolState(), SolverPoolState::kFailed);
  EXPECT_TRUE(WaitUntil([&] { return !mock_.Running(); }));
  EXPECT_EQ(mock_.TickCount(), 0U);
  EXPECT_TRUE(Tasks("mpc_worker_").empty()) << "a refused set-up must not start a pool";
  mock_.StopAndJoin();
}

TEST_F(SolverPool, RefusesWhenTheMpcThreadsOwnConfigFailed) {
  MpcThreadLaunchConfig l = Launch("mpc_pool_c", 2, {1});
  l.main.cpu_core = 100000;  // no host has this slot
  mock_.Init(mgr_, l);
  mock_.Start();
  ASSERT_TRUE(WaitUntil([&] { return mock_.PoolState() != SolverPoolState::kPending; }));
  EXPECT_EQ(mock_.PoolState(), SolverPoolState::kFailed);
  EXPECT_TRUE(WaitUntil([&] { return !mock_.Running(); }));
  EXPECT_EQ(mock_.TickCount(), 0U);
  mock_.StopAndJoin();
}

TEST_F(SolverPool, RefusesAWorkerThatCannotBePinned) {
  mock_.Init(mgr_, Launch("mpc_pool_d", 2, {100000}));
  mock_.Start();
  ASSERT_TRUE(WaitUntil([&] { return mock_.PoolState() != SolverPoolState::kPending; }));
  EXPECT_EQ(mock_.PoolState(), SolverPoolState::kFailed);
  EXPECT_TRUE(WaitUntil([&] { return !mock_.Running(); }));
  EXPECT_EQ(mock_.TickCount(), 0U);
  mock_.StopAndJoin();
}

// The one-thread path places nothing, refuses nothing (a dev box without RT
// permission keeps running, as before), and starts no pool.
TEST_F(SolverPool, OneThreadStartsNoPoolAndToleratesAFailedConfig) {
  MpcThreadLaunchConfig l = Launch("mpc_pool_e", 1, {});
  l.main.cpu_core = 100000;
  mock_.Init(mgr_, l);
  mock_.Start();
  ASSERT_TRUE(WaitUntil([&] { return mock_.TickCount() >= 5; }));
  EXPECT_EQ(mock_.PoolState(), SolverPoolState::kReady);
  EXPECT_TRUE(Tasks("mpc_worker_").empty());
  mock_.StopAndJoin();
}

// An MPC thread whose solve multiplies two matrices large enough for Eigen to
// parallelise. Eigen sizes its OpenMP team from this thread's ICVs; on the
// one-thread path the loop entry pins those to one, so the product must not
// grow a pool of unnamed threads (they would carry this thread's name).
class GemmThread final : public MPCThread {
 public:
  explicit GemmThread(int n = 256) : a_(Eigen::MatrixXd::Random(n, n)), b_(a_), c_(a_) {}

 protected:
  bool Solve(const MPCStateSnapshot& /*state*/, MPCSolution& /*out*/) override {
    c_.noalias() = a_ * b_;
    return false;
  }

 private:
  Eigen::MatrixXd a_;
  Eigen::MatrixXd b_;
  Eigen::MatrixXd c_;
};

TEST_F(SolverPool, OneThreadKeepsEigenOffAPool) {
  const char* env = std::getenv("OMP_NUM_THREADS");
  if (env != nullptr) {
    std::printf(
        "[report] OMP_NUM_THREADS=%s is set: the process default already pins Eigen's "
        "team, so this run cannot tell the loop entry's ICVs from the environment's\n",
        env);
  }
  GemmThread t;
  MpcThreadLaunchConfig l = Launch("mpc_pool_gemm", 1, {});
  // Unpinned: with dynamic teams on (this suite's environment) a one-CPU mask
  // would shrink any team to one thread by itself and hide what is checked.
  l.main.cpu_core = -1;
  t.Init(mgr_, l);
  t.Start();
  ASSERT_TRUE(WaitUntil([&] { return t.TickCount() >= 20; }, std::chrono::seconds(10)));
  EXPECT_EQ(Tasks("mpc_pool_gemm").size(), 1U)
      << "Eigen started an OpenMP team on the one-thread path";
  t.StopAndJoin();
}

// The pool must stay the threads the loop entry pinned. libgomp keeps one
// pool per thread and lets it shrink: a region with a smaller team retires the
// surplus workers, and a later full-size region then spawns fresh ones — with
// the MPC thread's own affinity, unnamed, on its core. Eigen opens such regions
// by itself on the MPC thread (a product sized for two threads, here) unless
// the thread's team size keeps it serial. The solver's own regions ask for
// their team size explicitly and are unaffected.
TEST_F(SolverPool, ASolveThatMultipliesMatricesKeepsThePinnedPool) {
  if (HostSlots() < 3) {
    GTEST_SKIP() << "needs 3 slots (host has " << HostSlots() << ")";
  }
  GemmThread t(48);  // 48^3 multiply-adds: Eigen sizes that product for two threads
  t.Init(mgr_, Launch("mpc_pool_g", 3, {1, 2}));
  t.Start();
  ASSERT_TRUE(WaitUntil([&] { return t.PoolState() != SolverPoolState::kPending; }));
  ASSERT_EQ(t.PoolState(), SolverPoolState::kReady);
  const std::set<pid_t> pinned = Tids(Tasks("mpc_worker_"));
  ASSERT_EQ(pinned.size(), 2U);
  ASSERT_TRUE(WaitUntil([&] { return t.TickCount() >= 50; }, std::chrono::seconds(10)));
  EXPECT_EQ(Tids(Tasks("mpc_worker_")), pinned) << "a pinned worker was retired";
  EXPECT_EQ(Tasks("mpc_pool_g").size(), 1U) << "an unnamed thread joined the MPC thread's core";
  t.StopAndJoin();
}

// Slot → logical CPU through the layout's own translation, on topologies the
// host does not have: a worker never lands on the MPC thread's core, on
// another worker's core, or on an SMT sibling (a slot is a physical core's
// first logical by construction).
TEST(SolverPoolLayout, WorkerSlotsMapToDistinctPhysicalCores) {
  struct Case {
    const char* name;
    std::vector<int> physical_core_slots;
    int ncpu;
    const char* profile;
  };

  const std::vector<Case> cases = {
      // 4P (HT) + 8E: P primaries 0,2,4,6, E-cores 8..15.
      {"hybrid 4P+8E", {0, 2, 4, 6, 8, 9, 10, 11, 12, 13, 14, 15}, 12, "mpc_par"},
      {"hybrid 4P+8E sim", {0, 2, 4, 6, 8, 9, 10, 11, 12, 13, 14, 15}, 12, "mpc_par_sim"},
      // 8C/16T SMT, siblings numbered +8: primaries 0..7.
      {"SMT 8C/16T", {0, 1, 2, 3, 4, 5, 6, 7}, 8, "mpc_par"},
      {"SMT 8C/16T sim", {0, 1, 2, 3, 4, 5, 6, 7}, 8, "mpc_par_sim"},
      // 6P (HT) + 8E + 2LPE: P primaries 0..10 step 2.
      {"hybrid 6P+8E+2LPE",
       {0, 2, 4, 6, 8, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21},
       16,
       "mpc_par"},
  };
  for (const Case& c : cases) {
    rtc::CpuTopology topo;
    topo.physical_core_slots = c.physical_core_slots;
    const rtc::SystemThreadConfigs cfg = rtc::SelectThreadConfigsForCoreCount(c.ncpu);
    const rtc::MpcWorkerSlots w = rtc::SelectMpcWorkerSlotsForCoreCount(c.ncpu, c.profile);
    ASSERT_GT(w.count, 0) << c.name;
    const int main_cpu = rtc::SlotToLogicalCpu(cfg.mpc.main.cpu_core, topo);
    std::set<int> seen = {main_cpu};
    for (int i = 0; i < w.count; ++i) {
      const int cpu = rtc::SlotToLogicalCpu(w.slots[static_cast<std::size_t>(i)], topo);
      EXPECT_TRUE(std::find(c.physical_core_slots.begin(), c.physical_core_slots.end(), cpu) !=
                  c.physical_core_slots.end())
          << c.name << ": worker " << i << " maps to " << cpu << ", not a core's first logical";
      EXPECT_TRUE(seen.insert(cpu).second) << c.name << ": worker " << i << " shares CPU " << cpu;
    }
  }
}

// A thread whose solver runs a different thread count than its launch config
// was built for: refused before the first tick.
class MismatchedThread final : public MPCThread {
 protected:
  bool Solve(const MPCStateSnapshot& /*state*/, MPCSolution& /*out*/) override { return false; }

  [[nodiscard]] bool SolverThreadsMatch() const noexcept override { return false; }
};

TEST_F(SolverPool, RefusesASolverWhoseThreadCountIsNotTheLaunchConfigs) {
  MismatchedThread t;
  t.Init(mgr_, Launch("mpc_pool_f", 1, {}));
  t.Start();
  ASSERT_TRUE(WaitUntil([&] { return t.PoolState() != SolverPoolState::kPending; }));
  EXPECT_EQ(t.PoolState(), SolverPoolState::kFailed);
  EXPECT_TRUE(WaitUntil([&] { return !t.Running(); }));
  EXPECT_EQ(t.TickCount(), 0U);
  t.StopAndJoin();
}

}  // namespace
}  // namespace rtc::mpc
