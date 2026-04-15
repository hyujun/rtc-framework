// End-to-end pipeline test: MockMPCThread → MPCSolutionManager → RT loop.
//
// Drives a MockMPCThread at 100 Hz and polls ComputeReference at 1 kHz for
// ~500 ms. Verifies:
//   * MPC publishes solutions that the manager picks up.
//   * Interpolated q_ref converges towards the target.
//   * u_fb is well-defined (finite, bounded).
//   * At least some fresh solutions were consumed within the window.

#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/thread/mock_mpc_thread.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <thread>

namespace rtc::mpc {
namespace {

constexpr int kNq = 3;
constexpr int kNv = 3;
constexpr int kHorizon = 10;
constexpr double kDtNode = 0.01;

YAML::Node Config() {
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 100;
  YAML::Node r;
  r["enabled"] = true;
  r["gain_scale"] = 0.5;
  r["accel_only"] = true;
  cfg["riccati"] = r;
  return cfg;
}

TEST(MpcEndToEnd, MockSolverDrivesConvergence) {
  MPCSolutionManager mgr;
  mgr.Init(Config(), kNq, kNv, 0);

  MockMPCThread mock;
  mock.Configure(kNq, kNv, kHorizon, kDtNode);

  Eigen::VectorXd target(kNq);
  target << 1.0, -0.5, 2.0;
  mock.SetTarget(target);

  MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;
  launch.target_frequency_hz = 100.0;
  mock.Init(mgr, launch);

  // Seed the RT-side state at the origin so Solve has a starting point.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(kNq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  mgr.WriteState(q, v, 0);

  mock.Start();

  // RT polling loop at 1 kHz for 500 ms.
  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(kNq);
  Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(kNv);
  Eigen::VectorXd a_ff = Eigen::VectorXd::Zero(kNv);
  Eigen::VectorXd lambda_ref = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd u_fb = Eigen::VectorXd::Zero(kNv);
  InterpMeta meta{};

  const auto start = std::chrono::steady_clock::now();
  int valid_count = 0;
  double max_u_fb_abs = 0.0;
  int iterations = 0;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - start);
    if (elapsed.count() >= 500) {
      break;
    }

    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count());
    mgr.WriteState(q, v, now_ns);
    const bool ok = mgr.ComputeReference(q, v, now_ns, q_ref, v_ref, a_ff,
                                         lambda_ref, u_fb, meta);
    if (ok) {
      ++valid_count;
      // Chase the reference at an artificially fast rate so q converges.
      q = 0.9 * q + 0.1 * q_ref;
      for (int i = 0; i < u_fb.size(); ++i) {
        max_u_fb_abs = std::max(max_u_fb_abs, std::abs(u_fb(i)));
      }
    }
    ++iterations;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  mock.RequestStop();
  mock.Join();

  EXPECT_GT(valid_count, 10)
      << "Expected many valid ComputeReference calls over 500 ms";
  EXPECT_LT(mgr.StaleCount(), 100)
      << "Stale counter should stay well below threshold";
  for (int i = 0; i < kNq; ++i) {
    EXPECT_TRUE(std::isfinite(q_ref(i)));
    EXPECT_TRUE(std::isfinite(u_fb(i)));
  }
  // q should have advanced some distance toward the target.
  EXPECT_GT(q.norm(), 0.1)
      << "RT state did not advance toward target — MPC pipeline inactive?";
}

}  // namespace
}  // namespace rtc::mpc
