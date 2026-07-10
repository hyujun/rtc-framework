#ifndef RTC_COMMUNICATION_CAN_CAN_TRANSPORT_HPP_
#define RTC_COMMUNICATION_CAN_CAN_TRANSPORT_HPP_

#include "rtc_communication/can/can_socket.hpp"
#include "rtc_communication/transport_interface.hpp"

#include <linux/can.h>

#include <algorithm>
#include <cstring>
#include <string>

namespace rtc {

// Payload-only classic CAN transport: Send() writes the payload as a single
// frame with the configured tx_can_id; Recv() returns the payload of the next
// filter-passing frame. Frame metadata (CAN ID, flags) is fixed by config, so
// one instance maps to one tx ID / one rx filter. Payload is limited to
// CAN_MAX_DLEN (8 bytes) — a PacketCodec used over this transport must keep
// sizeof(SendPacket)/sizeof(RecvPacket) within that bound.
struct CanTransportConfig {
  std::string interface_name{"can0"};
  canid_t tx_can_id{0};
  canid_t rx_can_id{0};
  // Filter is installed only when rx_can_mask != 0; the default 0 accepts all
  // frames (kernel default). Note rx_can_id has no effect while rx_can_mask
  // stays 0. RTR frames are always dropped in Recv() (payload-only).
  canid_t rx_can_mask{0};
  bool extended_frame{false};
  bool receive_own_messages{false};  // kernel default: off
  bool loopback{true};               // kernel default: on
  int recv_timeout_ms{100};
};

class CanTransport : public TransportInterface {
 public:
  explicit CanTransport(const CanTransportConfig& config) noexcept
      : config_(config), tx_id_(config.tx_can_id | (config.extended_frame ? CAN_EFF_FLAG : 0)) {}

  [[nodiscard]] bool Open() override {
    if (!socket_.Bind(config_.interface_name))
      return false;
    if (config_.rx_can_mask != 0) {
      // Include EFF/RTR in the mask so the filter distinguishes frame types
      // (kernel-recommended for single-ID filters); RTR frames are rejected.
      const canid_t filter_id = config_.rx_can_id | (config_.extended_frame ? CAN_EFF_FLAG : 0);
      const canid_t filter_mask = config_.rx_can_mask | CAN_EFF_FLAG | CAN_RTR_FLAG;
      if (!socket_.SetFilter(filter_id, filter_mask)) {
        socket_.Close();
        return false;
      }
    }
    if (!socket_.SetLoopback(config_.loopback) ||
        !socket_.SetRecvOwnMessages(config_.receive_own_messages)) {
      socket_.Close();
      return false;
    }
    socket_.SetRecvTimeout(config_.recv_timeout_ms);
    return true;
  }

  void Close() noexcept override { socket_.Close(); }

  // Sends data as one classic CAN frame. Returns payload bytes sent, or -1 on
  // error (including payload > CAN_MAX_DLEN).
  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept override {
    if (data.size() > CAN_MAX_DLEN)
      return -1;
    can_frame frame{};
    frame.can_id = tx_id_;
    frame.len = static_cast<__u8>(data.size());
    std::memcpy(frame.data, data.data(), data.size());
    return (socket_.SendFrame(frame) < 0) ? -1 : static_cast<ssize_t>(data.size());
  }

  // Receives one frame and copies its payload into buffer (truncating if the
  // buffer is smaller). RTR frames are dropped (their data has no meaning).
  // Returns payload bytes copied, or -1 on error/timeout/RTR.
  [[nodiscard]] ssize_t Recv(std::span<uint8_t> buffer) noexcept override {
    can_frame frame{};
    if (socket_.RecvFrame(frame) < static_cast<ssize_t>(sizeof(frame)))
      return -1;
    if (frame.can_id & CAN_RTR_FLAG)
      return -1;
    const std::size_t copy_len = std::min(static_cast<std::size_t>(frame.len), buffer.size());
    std::memcpy(buffer.data(), frame.data, copy_len);
    return static_cast<ssize_t>(copy_len);
  }

  void SetRecvTimeout(int timeout_ms) noexcept override { socket_.SetRecvTimeout(timeout_ms); }

  void SetRecvBufferSize(int size) noexcept override { socket_.SetRecvBufferSize(size); }

  [[nodiscard]] bool is_open() const noexcept override { return socket_.is_open(); }

 private:
  CanTransportConfig config_;
  canid_t tx_id_;
  CanSocket socket_;
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_CAN_CAN_TRANSPORT_HPP_
