#ifndef RTC_COMMUNICATION_UDP_UDP_TRANSPORT_HPP_
#define RTC_COMMUNICATION_UDP_UDP_TRANSPORT_HPP_

#include "rtc_communication/transport_interface.hpp"
#include "rtc_communication/udp/udp_socket.hpp"

#include <string>

namespace rtc {

struct UdpTransportConfig {
  std::string bind_address{"0.0.0.0"};
  int bind_port{0};
  std::string target_address;
  int target_port{0};
  int recv_buffer_size{256 * 1024};
  int recv_timeout_ms{100};
};

class UdpTransport : public TransportInterface {
 public:
  explicit UdpTransport(const UdpTransportConfig& config) noexcept
      : config_(config) {}

  [[nodiscard]] bool Open() override {
    if (config_.bind_port > 0) {
      if (!recv_socket_.Bind(config_.bind_port)) return false;
      recv_socket_.SetRecvBufferSize(config_.recv_buffer_size);
      recv_socket_.SetRecvTimeout(config_.recv_timeout_ms);
    }
    if (!config_.target_address.empty() && config_.target_port > 0) {
      if (!send_socket_.Connect(config_.target_address, config_.target_port))
        return false;
    }
    return true;
  }

  void Close() noexcept override {
    recv_socket_.Close();
    send_socket_.Close();
  }

  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept override {
    return send_socket_.Send(data);
  }

  [[nodiscard]] ssize_t Recv(std::span<uint8_t> buffer) noexcept override {
    // UdpSocket::Recv takes span<char>, so reinterpret
    return recv_socket_.Recv(
        std::span<char>(reinterpret_cast<char*>(buffer.data()), buffer.size()));
  }

  void SetRecvTimeout(int timeout_ms) noexcept override {
    recv_socket_.SetRecvTimeout(timeout_ms);
  }

  void SetRecvBufferSize(int size) noexcept override {
    recv_socket_.SetRecvBufferSize(size);
  }

  [[nodiscard]] bool is_open() const noexcept override {
    return recv_socket_.is_open() || send_socket_.is_open();
  }

  // Direct access for legacy code migration
  UdpSocket& recv_socket() noexcept { return recv_socket_; }
  UdpSocket& send_socket() noexcept { return send_socket_; }

 private:
  UdpTransportConfig config_;
  UdpSocket recv_socket_;
  UdpSocket send_socket_;
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_UDP_UDP_TRANSPORT_HPP_
