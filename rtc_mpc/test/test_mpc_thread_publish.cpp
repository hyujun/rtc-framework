/// @file test_mpc_thread_publish.cpp
/// @brief The MPC thread's publish rules and its overrun split, on a stub
///        handler (no solver): a solve past the deadline and a
///        budget-exhausted solve whose consumed segment is over the
///        tolerance are not published and not counted as failures; the
///        standing solve is exempt from the deadline; a converged solve is
///        always published; both rules are off by default; an online solve
///        that started cold is published only once converged; an overrun
///        after a standing-solve tick — failed or not — is counted apart
///        from an online one.
///
/// The model is the generic Panda fixture (the thread runs forward
/// kinematics every tick); nothing here solves an OCP.

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

#include "rtc_base/testing/wait_until.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_manager_base.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "test_utils/panda_fixture.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <thread>

namespace {

using rtc::mpc::HandlerMPCThread;
using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCSolverConfig;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::PhaseContext;
using namespace std::chrono_literals;

constexpr int kStubHorizon = 10;
constexpr double kStubDt = 0.035;

/// A handler that solves nothing: each `Solve` sleeps, then packs a finite
/// solution whose status, standing flag and per-node residuals the test sets.
class StubHandler final : public rtc::mpc::MPCHandlerBase {
 public:
  StubHandler(int nq, int nv, MPCSolverConfig cfg) : nq_(nq), nv_(nv), cfg_(cfg) {}

  rtc::mpc::MPCInitError Init(const MPCSolverConfig&, const rtc::mpc::RobotModelHandler&,
                              const rtc::mpc::OCPLimits&, const PhaseContext&) noexcept override {
    return rtc::mpc::MPCInitError::kNoError;
  }

  MPCSolveError Solve(const PhaseContext&, const MPCStateSnapshot&,
                      MPCSolution& out) noexcept override {
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us.load()));
    calls.fetch_add(1);
    if (error.load() != 0) {
      return static_cast<MPCSolveError>(error.load());
    }
    out.horizon_length = kStubHorizon;
    out.dt_node = kStubDt;
    out.nq = nq_;
    out.nv = nv_;
    out.nu = nv_;
    out.nx = nq_ + nv_;
    out.ndx = 2 * nv_;
    rtc::mpc::MarkSolved(out, converged.load());
    out.initial_solve = initial.load();
    out.cold_start = cold.load();
    for (std::size_t k = 0; k < out.node_prim_inf.size(); ++k) {
      out.node_prim_inf[k] = static_cast<int>(k) < big_from.load() ? small_residual.load() : 5.0;
    }
    return MPCSolveError::kNoError;
  }

  [[nodiscard]] bool Initialised() const noexcept override { return true; }

  [[nodiscard]] const MPCSolverConfig& solver_config() const noexcept override { return cfg_; }

  [[nodiscard]] bool InitialSolvePending() const noexcept override { return initial.load(); }

  [[nodiscard]] std::string_view ocp_type() const noexcept override { return "stub"; }

  [[nodiscard]] int horizon_length() const noexcept override { return kStubHorizon; }

  [[nodiscard]] int nq() const noexcept override { return nq_; }

  [[nodiscard]] int nv() const noexcept override { return nv_; }

  [[nodiscard]] int nu() const noexcept override { return nv_; }

  [[nodiscard]] int n_contact_vars() const noexcept override { return 0; }

  void SeedWarmStart(const MPCSolution&) noexcept override { seeds.fetch_add(1); }

  std::atomic<int> sleep_us{0};
  std::atomic<bool> converged{false};
  std::atomic<bool> initial{false};  ///< the standing budget pending, and the solve's flag
  std::atomic<bool> cold{false};
  std::atomic<int> error{0};  ///< an MPCSolveError to return instead of a solution
  std::atomic<int> big_from{kStubHorizon + 1};  ///< first node with residual 5.0
  std::atomic<double> small_residual{0.01};
  std::atomic<std::uint64_t> calls{0};
  std::atomic<std::uint64_t> seeds{0};

 private:
  int nq_;
  int nv_;
  MPCSolverConfig cfg_;
};

/// A phase manager whose context always names the stub handler's type.
class StubPhaseManager final : public rtc::mpc::PhaseManagerBase {
 public:
  StubPhaseManager() = default;

  void Init(const YAML::Node&) override {}

  PhaseContext Update(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
                      const pinocchio::SE3&, double) override {
    PhaseContext ctx;
    ctx.ocp_type = "stub";
    return ctx;
  }

  void SetTaskTarget(const YAML::Node&) override {}

  [[nodiscard]] int CurrentPhaseId() const override { return 0; }

  [[nodiscard]] std::string CurrentPhaseName() const override { return {}; }

  void ForcePhase(int) override {}
};

class MpcThreadPublishTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, model_);
    ASSERT_EQ(model_handler_.Init(model_, YAML::Load(rtc::mpc::test_utils::kPandaTwoFingerYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    YAML::Node mgr_cfg;
    mgr_cfg["enabled"] = true;
    mgr_.Init(mgr_cfg, model_handler_.nq(), model_handler_.nv(), 0);
    mgr_.WriteState(pinocchio::neutral(model_), Eigen::VectorXd::Zero(model_handler_.nv()), 0);
  }

  /// Start a thread at @p hz over a stub handler with @p cfg; returns the stub.
  StubHandler* Start(HandlerMPCThread& thread, const MPCSolverConfig& cfg, double hz) {
    auto stub = std::make_unique<StubHandler>(model_handler_.nq(), model_handler_.nv(), cfg);
    StubHandler* raw = stub.get();
    thread.Configure(model_handler_, std::move(stub), std::make_unique<StubPhaseManager>());
    rtc::mpc::MpcThreadLaunchConfig launch{};
    launch.main.cpu_core = -1;
    launch.main.sched_policy = 0;
    launch.target_frequency_hz = hz;
    thread.Init(mgr_, launch);
    return raw;
  }

  [[nodiscard]] std::uint64_t Published() const { return mgr_.GetSolveStats().count; }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler model_handler_;
  rtc::mpc::MPCSolutionManager mgr_;
};

// Both rules off (the default): a slow, budget-exhausted solve with a large
// residual everywhere is published.
TEST_F(MpcThreadPublishTest, WithTheRulesOffEverySolveIsPublished) {
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, MPCSolverConfig{}, 50.0);
  stub->sleep_us = 5000;
  stub->big_from = 0;
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= 5U; }));
  thread.StopAndJoin();
  EXPECT_EQ(thread.DeadlineMissCount(), 0U);
  EXPECT_EQ(thread.WithheldCount(), 0U);
  EXPECT_EQ(thread.FailedSolves(), 0U);
  EXPECT_EQ(Published(), thread.TotalSolves());
}

// A solve that returns 1 ms past the deadline is not published, counted as a
// deadline miss and not as a failure, and the handler is not touched (no
// seed: its iterates stay the next tick's warm start). Back inside the
// deadline, solves are published again.
TEST_F(MpcThreadPublishTest, ALateSolveIsNotPublishedAndNotAFailure) {
  MPCSolverConfig cfg;
  cfg.publish_deadline_ms = 5.0;
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, cfg, 20.0);
  stub->sleep_us = 6000;
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return thread.DeadlineMissCount() >= 3U; }));
  EXPECT_EQ(Published(), 0U);
  EXPECT_EQ(thread.FailedSolves(), 0U);
  EXPECT_EQ(thread.WithheldCount(), 0U);
  stub->sleep_us = 0;
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= 3U; }));
  thread.StopAndJoin();
  EXPECT_EQ(thread.DeadlineMissCount() + Published(), thread.TotalSolves());
  EXPECT_EQ(stub->seeds.load(), 0U);
  EXPECT_EQ(stub->calls.load(), thread.TotalSolves());
}

// The standing solve runs to convergence by design: it is published however
// long it took.
TEST_F(MpcThreadPublishTest, TheStandingSolveIsExemptFromTheDeadline) {
  MPCSolverConfig cfg;
  cfg.publish_deadline_ms = 2.0;
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, cfg, 20.0);
  stub->sleep_us = 6000;
  stub->initial = true;
  stub->converged = true;
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= 3U; }));
  thread.StopAndJoin();
  EXPECT_EQ(thread.DeadlineMissCount(), 0U);
  EXPECT_EQ(thread.InitialSolveTicks(), thread.TotalSolves());
}

// A budget-exhausted solve is published when the nodes a consumer reads
// before the next plan (0 … ⌈period / dt⌉ + 1 = 3 at 20 Hz, dt 0.035 s) are
// within the tolerance, whatever the rest of the horizon; withheld (not a
// failure) when one of them is not; a converged solve always is published.
TEST_F(MpcThreadPublishTest, ABudgetExhaustedSolveNeedsItsConsumedSegmentWithinTheTolerance) {
  MPCSolverConfig cfg;
  cfg.publish_prim_tol = 0.05;
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, cfg, 20.0);
  stub->big_from = 4;  // nodes 0-3 at 0.01, the rest 5.0
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= 3U; }));
  EXPECT_EQ(thread.WithheldCount(), 0U);

  stub->big_from = 3;  // node 3 over the tolerance
  const std::uint64_t published = Published();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return thread.WithheldCount() >= 3U; }));
  EXPECT_LE(Published(), published + 1U) << "at most the tick in flight";

  stub->converged = true;  // a converged solve is published whatever its residual
  stub->big_from = 0;
  const std::uint64_t withheld = thread.WithheldCount();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= published + 4U; }));
  thread.StopAndJoin();
  EXPECT_LE(thread.WithheldCount(), withheld + 1U);
  EXPECT_EQ(thread.FailedSolves(), 0U);
  EXPECT_EQ(thread.DeadlineMissCount(), 0U);
}

// The consumed segment: nodes 0 … ⌈period / dt⌉ + 1, a period of a whole
// number of nodes counting as that number, clamped to the horizon.
TEST(MpcThreadConsumedSegment, TheSegmentEndsOneNodePastThePeriod) {
  auto sol = std::make_unique<MPCSolution>();
  sol->horizon_length = 8;
  sol->dt_node = 0.035;
  for (int k = 0; k <= 8; ++k) {
    sol->node_prim_inf[static_cast<std::size_t>(k)] = 1.0 + k;  // the max is the last node's
  }
  sol->node_prim_inf[9] = 100.0;  // past the horizon: never read
  const auto ns = [](double s) { return static_cast<std::uint64_t>(s * 1e9 + 0.5); };
  EXPECT_EQ(HandlerMPCThread::ConsumedPrimInf(*sol, ns(1.0 / 60.0)), 3.0);  // nodes 0-2
  EXPECT_EQ(HandlerMPCThread::ConsumedPrimInf(*sol, ns(0.035)), 3.0);       // nodes 0-2
  EXPECT_EQ(HandlerMPCThread::ConsumedPrimInf(*sol, ns(0.05)), 4.0);        // nodes 0-3
  EXPECT_EQ(HandlerMPCThread::ConsumedPrimInf(*sol, ns(0.1)), 5.0);         // nodes 0-4
  EXPECT_EQ(HandlerMPCThread::ConsumedPrimInf(*sol, ns(10.0)), 9.0);        // clamped: 0-8
  sol->dt_node = 0.0;
  EXPECT_EQ(HandlerMPCThread::ConsumedPrimInf(*sol, ns(0.035)),
            std::numeric_limits<double>::infinity());
}

// An overrun after a standing-solve tick is counted apart from one after an
// online tick; together they are the base's overrun count.
TEST_F(MpcThreadPublishTest, AStandingTicksOverrunIsCountedApart) {
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, MPCSolverConfig{}, 100.0);
  stub->sleep_us = 25000;  // 2.5 periods: the wait after it finds an overrun
  stub->initial = true;
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return thread.StandingOverrunCount() >= 2U; }));
  EXPECT_EQ(thread.OnlineOverrunCount(), 0U);
  stub->initial = false;
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return thread.OnlineOverrunCount() >= 2U; }));
  thread.StopAndJoin();
  EXPECT_EQ(thread.OnlineOverrunCount() + thread.StandingOverrunCount(), thread.OverrunCount());
}

// An online solve that started from a cold seed (a node-grid jump) and did
// not converge is not published and is counted apart; once converged it is.
TEST_F(MpcThreadPublishTest, AColdOnlineSolveIsPublishedOnlyOnceConverged) {
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, MPCSolverConfig{}, 50.0);
  stub->cold = true;
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return thread.ColdWithheldCount() >= 3U; }));
  EXPECT_EQ(Published(), 0U);
  stub->converged = true;
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= 3U; }));
  const std::uint64_t cold_withheld = thread.ColdWithheldCount();
  stub->converged = false;
  stub->cold = false;  // warm again: published as before
  const std::uint64_t published = Published();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return Published() >= published + 3U; }));
  thread.StopAndJoin();
  EXPECT_LE(thread.ColdWithheldCount(), cold_withheld + 1U) << "at most the tick in flight";
  EXPECT_EQ(thread.WithheldCount(), 0U);
  EXPECT_EQ(thread.FailedSolves(), 0U);
}

// A standing-solve tick whose solve fails still counts its overrun as a
// standing one: the tick is marked from the handler's pending standing
// budget before it solves.
TEST_F(MpcThreadPublishTest, AFailedStandingTicksOverrunIsCountedApart) {
  HandlerMPCThread thread;
  StubHandler* stub = Start(thread, MPCSolverConfig{}, 100.0);
  stub->sleep_us = 25000;
  stub->initial = true;
  stub->error = static_cast<int>(MPCSolveError::kSolverException);
  thread.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return thread.StandingOverrunCount() >= 2U; }));
  thread.StopAndJoin();
  EXPECT_EQ(thread.OnlineOverrunCount(), 0U);
  EXPECT_GT(thread.FailedSolves(), 0U);
  EXPECT_EQ(thread.OnlineOverrunCount() + thread.StandingOverrunCount(), thread.OverrunCount());
}

}  // namespace
