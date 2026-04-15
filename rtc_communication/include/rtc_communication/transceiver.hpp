#ifndef RTC_COMMUNICATION_TRANSCEIVER_HPP_
#define RTC_COMMUNICATION_TRANSCEIVER_HPP_

// Generic transceiver: owns a receive thread + send path via TransportInterface.
//
// Template parameter Codec must satisfy the PacketCodec concept
// (see packet_codec.hpp). The transceiver:
//   - Opens a transport and runs a jthread decode loop
//   - Maintains a thread-safe snapshot of the latest decoded state
//   - Provides a Send() path using the same transport (allocation-free)
//
// All hot-path operations (recv, decode, send) are allocation-free
// and safe for SCHED_FIFO real-time threads.

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_communication/packet_codec.hpp"
#include "rtc_communication/transport_interface.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace rtc {

template <typename Codec>
  requires PacketCodec<Codec>
class Transceiver {
 public:
  using RecvPacket = typename Codec::RecvPacket;
  using SendPacket = typename Codec::SendPacket;
  using State      = typename Codec::State;
  using StateCallback = std::function<void(const State&)>;

  // transport: the transport layer to use for send/recv.
  // thread_cfg: RT thread config for the receive loop.
  explicit Transceiver(
      std::unique_ptr<TransportInterface> transport,
      const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept
      : transport_(std::move(transport)), thread_cfg_(thread_cfg) {}

  ~Transceiver() { Stop(); }

  Transceiver(const Transceiver&)            = delete;
  Transceiver& operator=(const Transceiver&) = delete;
  Transceiver(Transceiver&&)                 = delete;
  Transceiver& operator=(Transceiver&&)      = delete;

  // -- Receive path ----------------------------------------------------------

  // Opens the transport and starts the recv jthread. Returns false on failure.
  [[nodiscard]] bool StartRecv() {
    if (!transport_ || !transport_->Open()) return false;

    running_.store(true, std::memory_order_release);
    recv_thread_ = std::jthread([this](std::stop_token st) {
      RecvLoop(std::move(st));
    });
    return true;
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    recv_thread_.request_stop();
    if (transport_) {
      transport_->Close();
    }
    // jthread destructor joins automatically.
  }

  void SetCallback(StateCallback cb) noexcept { callback_ = std::move(cb); }

  // Snapshot of the most recently decoded State.
  //
  // @note **Not RT-safe.** Acquires a std::mutex shared with the recv thread
  // and may block briefly while a decode completes. Intended for non-RT
  // consumers (controllers running off of a SeqLock-published snapshot,
  // diagnostic threads, ROS2 callback groups). RT-path callers must obtain
  // the state through an upstream non-blocking primitive (SeqLock<T>, SPSC
  // queue, atomic snapshot) — never call this from a SCHED_FIFO loop.
  [[nodiscard]] State GetLatestState() const {
    std::lock_guard lock(data_mutex_);
    return latest_state_;
  }

  [[nodiscard]] bool IsRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::size_t recv_count() const noexcept {
    return recv_count_.load(std::memory_order_relaxed);
  }

  // -- Send path -------------------------------------------------------------

  // Sends a pre-built packet. Allocation-free.
  //
  // @note **RT-safe.** Performs one stack-buffer memcpy + a single sendto()
  // syscall, no locks taken, noexcept. Suitable for the 500 Hz RT loop.
  // The caller owns the timing budget — sendto() may still block if the
  // kernel send buffer is full and the socket is configured blocking.
  [[nodiscard]] bool Send(const SendPacket& pkt) noexcept {
    if (!transport_) return false;
    std::array<uint8_t, sizeof(SendPacket)> buf{};
    std::memcpy(buf.data(), &pkt, sizeof(SendPacket));
    const bool ok = transport_->Send(buf) >= 0;
    if (ok) {
      send_count_.fetch_add(1, std::memory_order_relaxed);
    }
    return ok;
  }

  [[nodiscard]] std::size_t send_count() const noexcept {
    return send_count_.load(std::memory_order_relaxed);
  }

  // -- Accessors -------------------------------------------------------------

  TransportInterface* transport() const noexcept { return transport_.get(); }

 private:
  void RecvLoop(std::stop_token stop_token) {
    (void)ApplyThreadConfig(thread_cfg_);  // best-effort; non-RT users may lack rtprio

    // Buffer slightly larger than the recv packet to detect oversized datagrams.
    std::array<uint8_t, sizeof(RecvPacket) + 64> buffer{};

    while (!stop_token.stop_requested()) [[likely]] {
      const ssize_t n = transport_->Recv(buffer);
      if (n < 0) [[unlikely]] continue;  // timeout or transport closed
      if (static_cast<std::size_t>(n) < sizeof(RecvPacket)) [[unlikely]] continue;

      State state{};
      if (Codec::Decode(
              std::span<const uint8_t>(buffer.data(),
                                       static_cast<std::size_t>(n)),
              state)) {
        {
          std::lock_guard lock(data_mutex_);
          latest_state_ = state;
        }
        if (callback_) {
          callback_(state);
        }
        recv_count_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  std::unique_ptr<TransportInterface> transport_;
  ThreadConfig thread_cfg_;

  std::atomic<bool>        running_{false};
  StateCallback            callback_;
  mutable std::mutex       data_mutex_;
  State                    latest_state_{};
  std::atomic<std::size_t> recv_count_{0};
  std::atomic<std::size_t> send_count_{0};

  std::jthread recv_thread_;
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_TRANSCEIVER_HPP_
