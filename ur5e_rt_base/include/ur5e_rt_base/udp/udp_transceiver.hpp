#ifndef UR5E_RT_BASE_UDP_TRANSCEIVER_HPP_
#define UR5E_RT_BASE_UDP_TRANSCEIVER_HPP_

// Generic UDP transceiver: owns a receive thread + send socket.
//
// Template parameter Codec must satisfy the UdpPacketCodec concept
// (see udp_codec.hpp). The transceiver:
//   - Binds a recv socket and runs a jthread decode loop
//   - Maintains a thread-safe snapshot of the latest decoded state
//   - Provides a Send() path using a second socket (allocation-free)
//
// All hot-path operations (recv, decode, send) are allocation-free
// and safe for SCHED_FIFO real-time threads.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

#include "ur5e_rt_base/threading/thread_config.hpp"
#include "ur5e_rt_base/threading/thread_utils.hpp"
#include "ur5e_rt_base/udp/udp_codec.hpp"
#include "ur5e_rt_base/udp/udp_socket.hpp"

namespace ur5e_rt_controller {

template <typename Codec>
  requires UdpPacketCodec<Codec>
class UdpTransceiver {
 public:
  using RecvPacket = typename Codec::RecvPacket;
  using SendPacket = typename Codec::SendPacket;
  using State      = typename Codec::State;
  using StateCallback = std::function<void(const State&)>;

  // recv_port: port to bind for incoming packets.
  // thread_cfg: RT thread config for the receive loop.
  explicit UdpTransceiver(
      int recv_port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept
      : recv_port_(recv_port), thread_cfg_(thread_cfg) {}

  ~UdpTransceiver() { Stop(); }

  UdpTransceiver(const UdpTransceiver&)            = delete;
  UdpTransceiver& operator=(const UdpTransceiver&) = delete;
  UdpTransceiver(UdpTransceiver&&)                 = delete;
  UdpTransceiver& operator=(UdpTransceiver&&)      = delete;

  // ── Receive path ───────────────────────────────────────────────────────

  // Starts the recv socket and jthread. Returns false on bind failure.
  [[nodiscard]] bool StartRecv() {
    if (!recv_socket_.Bind(recv_port_)) return false;

    constexpr int kRecvBufSize = 256 * 1024;
    recv_socket_.SetRecvBufferSize(kRecvBufSize);
    recv_socket_.SetRecvTimeout(100);  // 100 ms for stop_token check

    running_.store(true, std::memory_order_release);
    recv_thread_ = std::jthread([this](std::stop_token st) {
      RecvLoop(std::move(st));
    });
    return true;
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    recv_thread_.request_stop();
    recv_socket_.Close();
    send_socket_.Close();
    // jthread destructor joins automatically.
  }

  void SetCallback(StateCallback cb) noexcept { callback_ = std::move(cb); }

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

  // ── Send path ──────────────────────────────────────────────────────────

  // Initializes the send socket with the target address.
  [[nodiscard]] bool InitSend(std::string_view target_ip, int target_port) {
    return send_socket_.Connect(target_ip, target_port);
  }

  // Sends a pre-built packet. Allocation-free.
  [[nodiscard]] bool Send(const SendPacket& pkt) noexcept {
    std::array<uint8_t, sizeof(SendPacket)> buf{};
    std::memcpy(buf.data(), &pkt, sizeof(SendPacket));
    return send_socket_.Send(buf) >= 0;
  }

  [[nodiscard]] std::size_t send_count() const noexcept {
    return send_count_.load(std::memory_order_relaxed);
  }

 private:
  void RecvLoop(std::stop_token stop_token) {
    ApplyThreadConfig(thread_cfg_);

    // Buffer slightly larger than the recv packet to detect oversized datagrams.
    std::array<char, sizeof(RecvPacket) + 64> buffer{};

    while (!stop_token.stop_requested()) {
      const ssize_t n = recv_socket_.Recv(buffer);
      if (n < 0) continue;  // timeout or socket closed — recheck stop
      if (static_cast<std::size_t>(n) < sizeof(RecvPacket)) continue;

      State state{};
      if (Codec::Decode(
              std::span<const char>(buffer.data(), static_cast<std::size_t>(n)),
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

  int          recv_port_;
  ThreadConfig thread_cfg_;

  UdpSocket recv_socket_;
  UdpSocket send_socket_;

  std::atomic<bool>        running_{false};
  StateCallback            callback_;
  mutable std::mutex       data_mutex_;
  State                    latest_state_{};
  std::atomic<std::size_t> recv_count_{0};
  std::atomic<std::size_t> send_count_{0};

  std::jthread recv_thread_;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_UDP_TRANSCEIVER_HPP_
