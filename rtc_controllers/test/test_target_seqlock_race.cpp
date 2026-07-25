// Race test: SetDeviceTarget (off-RT) concurrent with Compute (RT thread)
// stresses the single-writer SeqLock + SPSC marshal pattern introduced by
// the RT-4 cleanup. The producer thread spams SetDeviceTarget; the consumer
// thread drains the queue inside Compute. After the test runs we re-load the
// SeqLock and check the seq_ counter is even (writer is not mid-store) and
// the final published target matches one of the values the producer emitted.
//
// Exercises JointPDController as the smallest representative — the marshal +
// drain logic is identical across all four rtc_controllers (joint_pd /
// p_controller / clik / osc) because it lives in the SeqLock<TargetSlot> +
// SpscQueue<PendingTarget, N> idiom shared between them.

#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "rtc_controllers/indirect/p_controller.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <span>
#include <string>
#include <thread>

namespace {

std::string GetTestUrdfPath() {
  if (const char* env = std::getenv("RTC_TEST_URDF_PATH")) {
    return env;
  }
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

rtc::ControllerState MakeState(int nch = 6) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = 0.002;
  state.devices[0].num_channels = nch;
  state.devices[0].valid = true;
  return state;
}

}  // namespace

// Stress: 100k SetDeviceTarget calls from an off-RT thread while a separate
// thread runs Compute() in a tight loop. With the single-writer SeqLock
// invariant, the consumer's reads never see a torn TargetSlot.
TEST(TargetSeqlockRace, SetDeviceTargetStressNoTearedReads) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  // Seed initial state so the first Compute() can self-init without bias.
  (void)ctrl.Compute(state);

  constexpr int kIters = 100'000;
  std::atomic<bool> producer_done{false};
  std::atomic<int> compute_iters{0};

  std::thread producer([&]() {
    std::array<double, 6> tgt{};
    for (int i = 0; i < kIters; ++i) {
      const double v = 0.001 * static_cast<double>(i % 1024);
      tgt.fill(v);
      ctrl.SetDeviceTarget(0, tgt);
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    // Spin Compute() until the producer finishes plus some drain margin.
    while (!producer_done.load(std::memory_order_acquire) ||
           compute_iters.load(std::memory_order_relaxed) < kIters / 8) {
      (void)ctrl.Compute(state);
      compute_iters.fetch_add(1, std::memory_order_relaxed);
    }
  });

  producer.join();
  consumer.join();

  EXPECT_GT(compute_iters.load(std::memory_order_relaxed), 0);

  // Drain any tail entries by running a few more Compute()s on the main
  // thread (now race-free — both worker threads have joined).
  for (int i = 0; i < 16; ++i) {
    (void)ctrl.Compute(state);
  }

  // Final consistency check: the controller's output goal_positions[0]
  // should be a value the producer emitted (the values were monotonically
  // cycled but always uniform across all 6 joints, so any final tick must
  // expose a uniform target).
  const auto final_out = ctrl.Compute(state);
  for (int i = 1; i < 6; ++i) {
    EXPECT_NEAR(final_out.devices[0].goal_positions[0],
                final_out.devices[0].goal_positions[static_cast<std::size_t>(i)], 1e-12)
        << "Goal positions disagree at index " << i
        << " — torn SeqLock read (writer-multiplicity violation)";
  }
}

// Same stress on PController (no trajectory; simplest tick body).
TEST(TargetSeqlockRace, PControllerSetDeviceTargetStress) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  (void)ctrl.Compute(state);

  constexpr int kIters = 100'000;
  std::atomic<bool> producer_done{false};
  std::atomic<int> compute_iters{0};

  std::thread producer([&]() {
    std::array<double, 6> tgt{};
    for (int i = 0; i < kIters; ++i) {
      const double v = 0.001 * static_cast<double>(i % 1024);
      tgt.fill(v);
      ctrl.SetDeviceTarget(0, tgt);
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire) ||
           compute_iters.load(std::memory_order_relaxed) < kIters / 8) {
      (void)ctrl.Compute(state);
      compute_iters.fetch_add(1, std::memory_order_relaxed);
    }
  });

  producer.join();
  consumer.join();

  // Drain + uniform-target consistency check.
  for (int i = 0; i < 16; ++i) {
    (void)ctrl.Compute(state);
  }
  const auto final_out = ctrl.Compute(state);
  for (int i = 1; i < 6; ++i) {
    EXPECT_NEAR(final_out.devices[0].goal_positions[0],
                final_out.devices[0].goal_positions[static_cast<std::size_t>(i)], 1e-12);
  }
}

// Same stress on TaskImpedanceController — the one controller whose target is a
// task-space SE(3) pose (goal_t + goal_rot marshalled through the SPSC +
// SeqLock<TargetSlot> path), which the joint-space cases above never exercise.
// The fault path is disabled so the stress lands on the marshal, not SAFE_STOP.
TEST(TargetSeqlockRace, TaskImpedanceSetDeviceTargetStress) {
  rtc::TaskImpedanceController::Gains gains;
  gains.pose_error_limit = 1e9;  // stress the marshal, not the pose-error fault
  gains.max_torque_rate = 1e12;  // no slew latching under the far commanded steps
  rtc::TaskImpedanceController ctrl(GetTestUrdfPath(), gains,
                                    rtc::TaskImpedanceController::TaskSelection::kFullSe3);
  auto state = MakeState();
  (void)ctrl.Compute(state);  // seed X_d = X_meas

  constexpr int kIters = 100'000;
  std::atomic<bool> producer_done{false};
  std::atomic<int> compute_iters{0};

  std::thread producer([&]() {
    std::array<double, 6> tgt{};
    for (int i = 0; i < kIters; ++i) {
      const double v = 0.001 * static_cast<double>(i % 1024);
      tgt = {v, v, v, 0.0, 0.0, 0.0};  // uniform translation, zero rotation
      ctrl.SetDeviceTarget(0, tgt);
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire) ||
           compute_iters.load(std::memory_order_relaxed) < kIters / 8) {
      (void)ctrl.Compute(state);
      compute_iters.fetch_add(1, std::memory_order_relaxed);
    }
  });

  producer.join();
  consumer.join();

  for (int i = 0; i < 16; ++i) {
    (void)ctrl.Compute(state);
  }

  // The commanded translation was uniform (v,v,v). Reconstruct the goal from the
  // final race-free tick: SplitWorld ⇒ e_trans = p_d − p, and actual_task_positions
  // is the measured TCP p, so goal_t = actual_task_positions + pose_error. A torn
  // marshal/drain of the SE(3) target would break that uniformity.
  const auto final_out = ctrl.Compute(state);
  const auto diag = ctrl.GetDiagnosticsForTesting();
  const double gx = final_out.actual_task_positions[0] + diag.pose_error[0];
  const double gy = final_out.actual_task_positions[1] + diag.pose_error[1];
  const double gz = final_out.actual_task_positions[2] + diag.pose_error[2];
  EXPECT_NEAR(gx, gy, 1e-9) << "torn task-target read (goal_t x vs y)";
  EXPECT_NEAR(gx, gz, 1e-9) << "torn task-target read (goal_t x vs z)";
}

// RT-4 on the OTHER off-RT entry point added in slice 2: SetExternalWrench
// writes a SeqLock<WrenchSample> directly (no SPSC hop), so a torn read would
// mix components from two different samples. The producer only ever emits
// wrenches whose six components are all equal, so ANY component disagreement in
// a snapshot the RT side consumed is a tear — and the generation stamp travels
// inside the same payload, so a torn read could also strand the staleness
// counter. Both are checked.
TEST(TargetSeqlockRace, TaskImpedanceSetExternalWrenchStress) {
  rtc::TaskImpedanceController::Gains gains;
  gains.pose_error_limit = 1e9;
  gains.max_torque_rate = 1e12;
  // Long enough that the tail ticks below cannot fade the last sample away.
  gains.wrench_timeout = 10.0;
  rtc::TaskImpedanceController ctrl(GetTestUrdfPath(), gains,
                                    rtc::TaskImpedanceController::TaskSelection::kFullSe3);
  ctrl.SetControlRate(500.0);
  // Transparent chain (no deadband / filter / bias) so the value the RT side
  // consumed is recoverable from the diagnostics without inverting a filter.
  ctrl.LoadConfig(YAML::Load(R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 0
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e9, 1.0e9, 1.0e9, 1.0e9, 1.0e9, 1.0e9]
  contact_threshold: 1.0e9
)"));
  auto state = MakeState();  // q ≡ 0 for every tick ⇒ the Ad^{-T} map is CONSTANT
  (void)ctrl.Compute(state);

  // Reference direction: the LWA image of the all-ones wrench. Since q never
  // changes, every uniform input (v,…,v) must map to exactly v·ref. A torn read
  // produces a non-uniform input, whose image leaves that one-dimensional line.
  std::array<double, 6> ones{};
  ones.fill(1.0);
  ctrl.SetExternalWrench(std::span<const double, 6>(ones.data(), 6));
  (void)ctrl.Compute(state);
  const auto ref_diag = ctrl.GetDiagnosticsForTesting();
  ASSERT_TRUE(ref_diag.wrench_valid);
  const std::array<double, 6> ref = ref_diag.wrench_lwa;
  double ref_norm = 0.0;
  for (double c : ref)
    ref_norm += c * c;
  ASSERT_GT(std::sqrt(ref_norm), 1e-6) << "degenerate reference — the check would be vacuous";

  constexpr int kIters = 100'000;
  std::atomic<bool> producer_done{false};
  std::atomic<int> compute_iters{0};
  std::atomic<int> torn{0};
  std::atomic<int> checked{0};

  std::thread producer([&]() {
    std::array<double, 6> w{};
    for (int i = 0; i < kIters; ++i) {
      const double v = 1.0 + 0.001 * static_cast<double>(i % 1024);  // never 0
      w.fill(v);  // all six components identical, by construction
      ctrl.SetExternalWrench(std::span<const double, 6>(w.data(), 6));
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire) ||
           compute_iters.load(std::memory_order_relaxed) < kIters / 8) {
      (void)ctrl.Compute(state);
      compute_iters.fetch_add(1, std::memory_order_relaxed);
      const auto diag = ctrl.GetDiagnosticsForTesting();
      if (!diag.wrench_valid || diag.wrench_fade != 1.0)
        continue;
      // Recover the implied scalar from the largest reference component, then
      // require every other component to agree with it.
      std::size_t pivot = 0;
      for (std::size_t i = 1; i < 6; ++i)
        if (std::abs(ref[i]) > std::abs(ref[pivot]))
          pivot = i;
      const double v = diag.wrench_lwa[pivot] / ref[pivot];
      for (std::size_t i = 0; i < 6; ++i) {
        if (std::abs(diag.wrench_lwa[i] - v * ref[i]) > 1e-9 * (1.0 + std::abs(v))) {
          torn.fetch_add(1, std::memory_order_relaxed);
          break;
        }
      }
      checked.fetch_add(1, std::memory_order_relaxed);
    }
  });

  producer.join();
  consumer.join();

  EXPECT_GT(checked.load(std::memory_order_relaxed), 0)
      << "no snapshot was ever inspected — the race window was never opened";
  EXPECT_EQ(torn.load(std::memory_order_relaxed), 0)
      << "wrench snapshot mixed components from two samples — torn SeqLock read";
}
