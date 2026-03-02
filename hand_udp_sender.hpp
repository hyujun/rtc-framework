// hand_udp_sender.hpp - v1
#pragma once

#include <string>
#include <vector>
#include <array>
#include <sys/socket.h>
#include <netinet/in.h>

namespace ur5e_controller {

class HandUdpSender {
public:
  static constexpr size_t NUM_HAND_JOINTS = 4;
  
  HandUdpSender(const std::string& target_ip, int target_port);
  ~HandUdpSender();
  
  // Disable copy
  HandUdpSender(const HandUdpSender&) = delete;
  HandUdpSender& operator=(const HandUdpSender&) = delete;
  
  bool initialize();
  bool send_command(const std::array<double, NUM_HAND_JOINTS>& positions);
  
  size_t get_send_count() const { return send_count_; }

private:
  int socket_fd_{-1};
  std::string target_ip_;
  int target_port_;
  sockaddr_in target_addr_{};
  
  size_t send_count_{0};
  
  std::vector<uint8_t> encode_packet(const std::array<double, NUM_HAND_JOINTS>& positions);
};

}  // namespace ur5e_controller
