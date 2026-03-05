#include "ur5e_hand_udp/hand_udp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace ur5e_rt_controller {

HandUdpSender::HandUdpSender(std::string target_ip, int target_port) noexcept
    : target_ip_(std::move(target_ip)), target_port_(target_port) {}

HandUdpSender::~HandUdpSender() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

bool HandUdpSender::Initialize() noexcept {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  std::memset(&target_addr_, 0, sizeof(target_addr_));
  target_addr_.sin_family = AF_INET;
  target_addr_.sin_port   = htons(static_cast<uint16_t>(target_port_));

  if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) <= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  return true;
}

bool HandUdpSender::SendCommand(
    std::span<const double, kNumHandJoints> positions) noexcept {
  if (socket_fd_ < 0) {
    return false;
  }

  const auto packet = EncodePacket(positions);
  const ssize_t n = sendto(socket_fd_, packet.data(), packet.size(), 0,
                            reinterpret_cast<const sockaddr*>(&target_addr_),
                            sizeof(target_addr_));
  if (n < 0) {
    return false;
  }
  ++send_count_;
  return true;
}

std::vector<uint8_t> HandUdpSender::EncodePacket(
    std::span<const double, kNumHandJoints> positions) noexcept {
  // Encode kNumHandJoints doubles as little-endian bytes (host byte order on
  // x86 is already little-endian — memcpy is sufficient).
  constexpr std::size_t kPacketSize = kNumHandJoints * sizeof(double);
  std::vector<uint8_t> packet(kPacketSize);

  std::memcpy(packet.data(), positions.data(), kPacketSize);
  return packet;
}

}  // namespace ur5e_rt_controller
