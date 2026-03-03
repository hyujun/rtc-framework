#ifndef UR5E_RT_CONTROLLER_HAND_UDP_RECEIVER_H_
#define UR5E_RT_CONTROLLER_HAND_UDP_RECEIVER_H_

#include <atomic>
#include <array>
#include <cstddef>
#include <functional>
#include <mutex>
#include <span>
#include <thread>

#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include "ur5e_rt_controller/thread_config.hpp"
#include "ur5e_rt_controller/thread_utils.hpp"

namespace ur5e_rt_controller {

// Receives hand state packets over UDP and exposes them via callback and
// snapshot accessor. Uses std::jthread (C++20) for cooperative cancellation.
class HandUdpReceiver {
 public:
  // Callback invoked from the receive thread on every valid packet.
  using DataCallback =
      std::function<void(std::span<const double, kNumHandJoints>)>;

  // thread_cfg defaults to kUdpRecvConfig (Core 3, SCHED_FIFO 65).
  // Pass a custom config or kLoggingConfig4Core for 4-core systems.
  explicit HandUdpReceiver(
      int port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept;
  ~HandUdpReceiver();

  HandUdpReceiver(const HandUdpReceiver&)            = delete;
  HandUdpReceiver& operator=(const HandUdpReceiver&) = delete;
  HandUdpReceiver(HandUdpReceiver&&)                 = delete;
  HandUdpReceiver& operator=(HandUdpReceiver&&)      = delete;

  // Opens the socket and starts the receive thread. Returns false on error.
  [[nodiscard]] bool Start();

  // Requests stop via stop_token and joins the thread (idempotent).
  void Stop() noexcept;

  [[nodiscard]] bool IsRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
  }

  void SetCallback(DataCallback callback) noexcept {
    callback_ = std::move(callback);
  }

  // Thread-safe snapshot of the most recently received data.
  [[nodiscard]] std::array<double, kNumHandJoints> GetLatestData() const;

  [[nodiscard]] std::size_t packet_count() const noexcept {
    return packet_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] double GetUpdateRate() const noexcept;

 private:
  int          port_;
  int          socket_fd_{-1};
  ThreadConfig thread_cfg_;
  std::atomic<bool> running_{false};

  DataCallback callback_;

  mutable std::mutex                      data_mutex_;
  std::array<double, kNumHandJoints> latest_data_{};
  std::atomic<std::size_t>               packet_count_{0};

  // C++20: jthread owns a stop_source; destructor requests stop and joins.
  std::jthread receive_thread_;

  void ReceiveLoop(std::stop_token stop_token);
  [[nodiscard]] bool ParsePacket(std::span<const char> buffer) noexcept;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_HAND_UDP_RECEIVER_H_
