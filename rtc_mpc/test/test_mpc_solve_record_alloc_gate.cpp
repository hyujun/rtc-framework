/// @file test_mpc_solve_record_alloc_gate.cpp
/// @brief The MPC thread's loop outside the handler's own solve allocates
///        nothing with a solve-record sink set, on every outcome: the publish
///        rules, the record's stash, `PublishSolution`, `OnTickEnd`'s write
///        into the sink, the loop's timing and the next tick's state read,
///        forward kinematics and phase manager.
///
/// How: the gate (C `malloc`, which `operator new` reaches too; thread-local)
/// is opened on the MPC thread when the stub handler's `Solve` returns and
/// closed when the next one starts, so it spans exactly the thread's own code
/// between two solves. The first windows (lazy statics, the pool's first
/// tick) are not counted.
///
/// This header may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_base/testing/wait_until.hpp"
#include "rtc_mpc/comm/triple_buffer.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_manager_base.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/thread/solve_record.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "test_utils/panda_fixture.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string_view>
#include <thread>

namespace {

using rtc::mpc::HandlerMPCThread;
using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCSolverConfig;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::PhaseContext;
using rtc::mpc::SolveOutcome;
using rtc::mpc::SolveRecord;

constexpr int kHorizon = 10;
constexpr int kWarmWindows = 5;  // not counted: first-touch statics, the first ticks

/// Solves nothing; closes the gate window at entry, opens the next at return.
class GatingStub final : public rtc::mpc::MPCHandlerBase {
 public:
  GatingStub(int nq, int nv, MPCSolverConfig cfg) : nq_(nq), nv_(nv), cfg_(cfg) {}

  rtc::mpc::MPCInitError Init(const MPCSolverConfig&, const rtc::mpc::RobotModelHandler&,
                              const rtc::mpc::OCPLimits&, const PhaseContext&) noexcept override {
    return rtc::mpc::MPCInitError::kNoError;
  }

  MPCSolveError Solve(const PhaseContext&, const MPCStateSnapshot&,
                      MPCSolution& out) noexcept override {
    if (gate_) {
      const std::size_t n = gate_->count();
      gate_.reset();
      if (windows_.fetch_add(1) >= kWarmWindows) {
        std::size_t prev = worst_.load();
        while (n > prev && !worst_.compare_exchange_weak(prev, n)) {
        }
        counted_.fetch_add(1);
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us.load()));
    MPCSolveError rc = static_cast<MPCSolveError>(error.load());
    if (rc == MPCSolveError::kNoError) {
      out.horizon_length = kHorizon;
      out.dt_node = 0.035;
      out.nq = nq_;
      out.nv = nv_;
      out.nu = nv_;
      out.nx = nq_ + nv_;
      out.ndx = 2 * nv_;
      rtc::mpc::MarkSolved(out, false);
      out.cold_start = cold.load();
      for (auto& r : out.node_prim_inf) {
        r = residual.load();
      }
    }
    gate_.emplace();
    return rc;
  }

  [[nodiscard]] bool Initialised() const noexcept override { return true; }

  [[nodiscard]] const MPCSolverConfig& solver_config() const noexcept override { return cfg_; }

  [[nodiscard]] bool InitialSolvePending() const noexcept override { return false; }

  [[nodiscard]] std::string_view ocp_type() const noexcept override { return "stub"; }

  [[nodiscard]] int horizon_length() const noexcept override { return kHorizon; }

  [[nodiscard]] int nq() const noexcept override { return nq_; }

  [[nodiscard]] int nv() const noexcept override { return nv_; }

  [[nodiscard]] int nu() const noexcept override { return nv_; }

  [[nodiscard]] int n_contact_vars() const noexcept override { return 0; }

  void SeedWarmStart(const MPCSolution&) noexcept override {}

  /// Close the window the thread left open (call after StopAndJoin).
  void CloseGate() noexcept { gate_.reset(); }

  std::atomic<int> sleep_us{0};
  std::atomic<int> error{0};
  std::atomic<bool> cold{false};
  std::atomic<double> residual{0.01};
  std::atomic<std::size_t> worst_{0};
  std::atomic<std::uint64_t> counted_{0};

 private:
  int nq_;
  int nv_;
  MPCSolverConfig cfg_;
  std::optional<rtc::testing::ScopedMallocGate> gate_;  // the MPC thread's only
  std::atomic<std::uint64_t> windows_{0};
};

class PassivePhaseManager final : public rtc::mpc::PhaseManagerBase {
 public:
  void Init(const YAML::Node&) override {}

  PhaseContext Update(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
                      const pinocchio::SE3&, double) override {
    PhaseContext ctx;
    ctx.ocp_type = "stub";
    return ctx;
  }

  void SetTaskTarget(const YAML::Node&) override {}

  [[nodiscard]] int CurrentPhaseId() const override { return 0; }

  [[nodiscard]] std::string CurrentPhaseName() const override { return "stub"; }

  void ForcePhase(int) override {}
};

TEST(MpcSolveRecordAllocGate, TheLoopAroundTheSolveAllocatesNothingOnEveryOutcome) {
  if (!std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, model);
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, YAML::Load(rtc::mpc::test_utils::kPandaTwoFingerYaml)),
            rtc::mpc::RobotModelInitError::kNoError);
  rtc::mpc::MPCSolutionManager mgr;
  YAML::Node mgr_cfg;
  mgr_cfg["enabled"] = true;
  mgr.Init(mgr_cfg, handler.nq(), handler.nv(), 0);
  mgr.WriteState(pinocchio::neutral(model), Eigen::VectorXd::Zero(handler.nv()), 0);

  MPCSolverConfig cfg;
  cfg.publish_deadline_ms = 5.0;
  cfg.publish_prim_tol = 0.05;
  auto owned = std::make_unique<GatingStub>(handler.nq(), handler.nv(), cfg);
  GatingStub* stub = owned.get();
  rtc::mpc::TripleBuffer<SolveRecord> sink;
  sink.StartWrite() = SolveRecord{};
  sink.FinishWrite();
  HandlerMPCThread thread;
  thread.Configure(handler, std::move(owned), std::make_unique<PassivePhaseManager>());
  rtc::mpc::MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;
  launch.main.sched_policy = 0;
  launch.target_frequency_hz = 50.0;
  thread.Init(mgr, launch);
  thread.SetSolveRecordSink(&sink);
  thread.Start();

  const auto count = [&](SolveOutcome o) {
    static_cast<void>(sink.TryAcquireLatest());
    return sink.Current().counts[static_cast<std::size_t>(o)];
  };
  const auto reach = [&](SolveOutcome o) {
    const std::uint64_t from = count(o);
    return rtc::testing::WaitUntil([&] { return count(o) >= from + 4U; });
  };
  ASSERT_TRUE(reach(SolveOutcome::kPublished));
  stub->sleep_us = 6000;
  ASSERT_TRUE(reach(SolveOutcome::kDeadline));
  stub->sleep_us = 0;
  stub->residual = 5.0;
  ASSERT_TRUE(reach(SolveOutcome::kWithheld));
  stub->residual = 0.01;
  stub->cold = true;
  ASSERT_TRUE(reach(SolveOutcome::kColdWithheld));
  stub->cold = false;
  stub->error = static_cast<int>(MPCSolveError::kSolverException);
  ASSERT_TRUE(reach(SolveOutcome::kFailed));
  thread.StopAndJoin();
  stub->CloseGate();

  EXPECT_GE(stub->counted_.load(), 10U) << "windows measured";
  EXPECT_EQ(stub->worst_.load(), 0U) << "allocations between two solves on the MPC thread";
}

// The gate is armed on a thread of its own and sees an allocation made inside a
// library there (pinocchio::Data — the header's contract item 4).
TEST(MpcSolveRecordAllocGate, PositiveControlTheGateCountsOnItsThread) {
  if (!std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, model);
  std::size_t n = 0;
  std::thread t([&n, &model] {
    rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data data(model);
    n = gate.count();
  });
  t.join();
  EXPECT_GE(n, 1U);
}

}  // namespace
