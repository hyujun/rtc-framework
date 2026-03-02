#ifndef UR5E_RT_CONTROLLER_HAND_UDP_SENDER_H_
#define UR5E_RT_CONTROLLER_HAND_UDP_SENDER_H_

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <netinet/in.h>

#include "ur5e_rt_controller/rt_controller_interface.hpp"

namespace ur5e_rt_controller {

// Sends hand position commands over UDP to the hand controller.
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
  // socket error.
  [[nodiscard]] bool SendCommand(
      std::span<const double, kNumHandJoints> positions) noexcept;

  [[nodiscard]] std::size_t send_count() const noexcept { return send_count_; }

 private:
  int         socket_fd_{-1};
  std::string target_ip_;
  int         target_port_;
  sockaddr_in target_addr_{};
  std::size_t send_count_{0};

  // Encodes positions into a raw byte packet (little-endian doubles).
  [[nodiscard]] static std::vector<uint8_t> EncodePacket(
      std::span<const double, kNumHandJoints> positions) noexcept;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_HAND_UDP_SENDER_H_
