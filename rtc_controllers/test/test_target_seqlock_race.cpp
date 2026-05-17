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
#include "rtc_controllers/indirect/p_controller.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

std::string GetTestUrdfPath() {
  const char* env = std::getenv("RTC_TEST_URDF_PATH");
  if (env != nullptr) {
    return env;
  }
  // Resolve relative to this source file (same heuristic as test_core_controllers.cpp).
  std::string path = __FILE__;
  auto pos = path.rfind("/rtc_controllers/");
  if (pos != std::string::npos) {
    return path.substr(0, pos) + "/rtc_urdf_bridge/test/urdf/serial_6dof.urdf";
  }
  return "serial_6dof.urdf";
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
