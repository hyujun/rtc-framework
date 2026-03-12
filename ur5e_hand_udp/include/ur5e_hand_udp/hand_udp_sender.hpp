#ifndef UR5E_HAND_UDP_HAND_UDP_SENDER_H_
#define UR5E_HAND_UDP_HAND_UDP_SENDER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include <netinet/in.h>

#include "ur5e_rt_base/types.hpp"
#include "ur5e_hand_udp/hand_udp_codec.hpp"

namespace ur5e_rt_controller {

// Sends hand position commands over UDP to the hand controller.
// All send-path operations are allocation-free for RT safety.
class HandUdpSender {
 public:
  HandUdpSender(std::string target_ip, int target_port) noexcept;
  ~HandUdpSender();

  HandUdpSender(const HandUdpSender&)            = delete;
  HandUdpSender& operator=(const HandUdpSender&) = delete;
  HandUdpSender(HandUdpSender&&)                 = delete;
  HandUdpSender& operator=(HandUdpSender&&)      = delete;

  // Creates the UDP socket and resolves the target address.
  [[nodiscard]] bool Initialize() noexcept;

  // Encodes and sends a kNumHandJoints position command. Returns false on
  // socket error. Zero heap allocation — uses pre-allocated send buffer.
  [[nodiscard]] bool SendCommand(
      std::span<const double, kNumHandJoints> positions) noexcept;

  [[nodiscard]] std::size_t send_count() const noexcept { return send_count_; }

 private:
  int         socket_fd_{-1};
  std::string target_ip_;
  int         target_port_;
  sockaddr_in target_addr_{};
  std::size_t send_count_{0};

  // Pre-allocated send buffer — eliminates per-call heap allocation.
  std::array<uint8_t, hand_udp_codec::kSendBytes> send_buffer_{};
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_UDP_SENDER_H_
