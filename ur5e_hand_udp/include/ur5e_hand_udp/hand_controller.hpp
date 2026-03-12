#ifndef UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
#define UR5E_HAND_UDP_HAND_CONTROLLER_HPP_

// High-level hand controller: request-response driver over UDP.
//
// Wraps UdpTransceiver<HandCodec> and exposes a simple API:
//   - Start()       → binds recv socket, launches jthread
//   - InitSend()    → resolves target address for commands
//   - SendCommand() → encodes and sends motor commands (allocation-free)
//   - GetLatestState() → thread-safe snapshot of last decoded HandState
//
// Replaces the separate HandUdpReceiver + HandUdpSender pair with a single
// object that owns both directions of the protocol.

#include <array>
#include <cstddef>
#include <cstring>
#include <functional>
#include <span>
#include <string_view>

#include "ur5e_rt_base/thread_config.hpp"
#include "ur5e_rt_base/udp_transceiver.hpp"
#include "ur5e_hand_udp/hand_packets.hpp"

namespace ur5e_rt_controller {

class HandController {
 public:
  using StateCallback = std::function<void(const HandState&)>;

  explicit HandController(
      int recv_port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept
      : transceiver_(recv_port, thread_cfg) {}

  ~HandController() = default;

  HandController(const HandController&)            = delete;
  HandController& operator=(const HandController&) = delete;
  HandController(HandController&&)                 = delete;
  HandController& operator=(HandController&&)      = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  // Starts receiving hand state packets.
  [[nodiscard]] bool Start() { return transceiver_.StartRecv(); }

  // Initializes the send path (target IP:port for motor commands).
  [[nodiscard]] bool InitSend(std::string_view target_ip, int target_port) {
    return transceiver_.InitSend(target_ip, target_port);
  }

  void Stop() noexcept { transceiver_.Stop(); }

  // ── Callback ───────────────────────────────────────────────────────────

  void SetCallback(StateCallback cb) noexcept {
    transceiver_.SetCallback(std::move(cb));
  }

  // ── Send (allocation-free) ─────────────────────────────────────────────

  [[nodiscard]] bool SendCommand(
      std::span<const double, kNumHandJoints> commands) noexcept {
    hand_packets::HandSendPacket pkt{};
    std::memcpy(pkt.motor_commands.data(), commands.data(),
                kNumHandJoints * sizeof(double));
    return transceiver_.Send(pkt);
  }

  // ── State access ───────────────────────────────────────────────────────

  [[nodiscard]] HandState GetLatestState() const {
    return transceiver_.GetLatestState();
  }

  [[nodiscard]] std::array<double, kNumHandJoints> GetLatestPositions() const {
    return transceiver_.GetLatestState().motor_positions;
  }

  [[nodiscard]] bool IsRunning() const noexcept {
    return transceiver_.IsRunning();
  }

  [[nodiscard]] std::size_t recv_count() const noexcept {
    return transceiver_.recv_count();
  }

  [[nodiscard]] std::size_t send_count() const noexcept {
    return transceiver_.send_count();
  }

 private:
  UdpTransceiver<HandCodec> transceiver_;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
