// hand_udp_receiver.hpp - v1
#pragma once

#include <string>
#include <vector>
#include <array>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>

namespace ur5e_controller {

class HandUdpReceiver {
public:
  static constexpr size_t NUM_HAND_JOINTS = 4;
  
  using DataCallback = std::function<void(const std::array<double, NUM_HAND_JOINTS>&)>;
  
  explicit HandUdpReceiver(int port);
  ~HandUdpReceiver();
  
  // Disable copy
  HandUdpReceiver(const HandUdpReceiver&) = delete;
  HandUdpReceiver& operator=(const HandUdpReceiver&) = delete;
  
  bool start();
  void stop();
  bool is_running() const { return running_; }
  
  void set_callback(DataCallback callback) {
    callback_ = std::move(callback);
  }
  
  // Get latest received data
  std::array<double, NUM_HAND_JOINTS> get_latest_data() const;
  
  // Statistics
  size_t get_packet_count() const { return packet_count_; }
  double get_update_rate() const;

private:
  int socket_fd_{-1};
  int port_;
  bool running_{false};
  
  DataCallback callback_;
  
  std::array<double, NUM_HAND_JOINTS> latest_data_{};
  size_t packet_count_{0};
  
  void receive_loop();
  bool parse_packet(const char* buffer, size_t length);
};

}  // namespace ur5e_controller
