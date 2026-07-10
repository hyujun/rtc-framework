#ifndef RTC_COMMUNICATION_CAN_CANFD_TRANSPORT_HPP_
#define RTC_COMMUNICATION_CAN_CANFD_TRANSPORT_HPP_

#include "rtc_communication/can/can_socket.hpp"
#include "rtc_communication/transport_interface.hpp"

#include <linux/can.h>

#include <algorithm>
#include <cstring>
#include <string>

namespace rtc {

// Payload-only CAN FD transport: same contract as CanTransport but with
// CAN_RAW_FD_FRAMES enabled and payload up to CANFD_MAX_DLEN (64 bytes) — a
// PacketCodec used over this transport must keep sizeof(SendPacket)/
// sizeof(RecvPacket) within that bound. An FD-enabled socket receives both
// classic and FD frames; Recv() handles both. Non-standard payload lengths
// (not in the CAN FD DLC set 0-8/12/16/20/24/32/48/64) are padded up by the
// kernel when sent to real controllers.
struct CanFdTransportConfig {
  std::string interface_name{"can0"};
  canid_t tx_can_id{0};
  canid_t rx_can_id{0};
  // Filter is installed only when rx_can_mask != 0; the default 0 accepts all
  // frames (kernel default). RTR frames are always excluded (payload-only).
  canid_t rx_can_mask{0};
  bool extended_frame{false};
  bool bitrate_switch{false};         // CANFD_BRS: transmit data phase at higher bitrate
  bool error_state_indicator{false};  // CANFD_ESI
  bool receive_own_messages{false};   // kernel default: off
  bool loopback{true};                // kernel default: on
  int recv_timeout_ms{100};
};

class CanFdTransport : public TransportInterface {
 public:
  explicit CanFdTransport(const CanFdTransportConfig& config) noexcept
      : config_(config),
        tx_id_(config.tx_can_id | (config.extended_frame ? CAN_EFF_FLAG : 0)),
        tx_flags_(static_cast<__u8>((config.bitrate_switch ? CANFD_BRS : 0) |
                                    (config.error_state_indicator ? CANFD_ESI : 0))) {}

  [[nodiscard]] bool Open() override {
    if (!socket_.Bind(config_.interface_name))
      return false;
    if (!socket_.SetFdFrames(true)) {
      socket_.Close();
      return false;
    }
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

  // Sends data as one CAN FD frame. Returns payload bytes sent, or -1 on
  // error (including payload > CANFD_MAX_DLEN).
  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept override {
    if (data.size() > CANFD_MAX_DLEN)
      return -1;
    canfd_frame frame{};
    frame.can_id = tx_id_;
    frame.flags = tx_flags_;
    frame.len = static_cast<__u8>(data.size());
    std::memcpy(frame.data, data.data(), data.size());
    return (socket_.SendFdFrame(frame) < 0) ? -1 : static_cast<ssize_t>(data.size());
  }

  // Receives one frame — classic (CAN_MTU) or FD (CANFD_MTU), both arrive on
  // an FD-enabled socket — and copies its payload into buffer (truncating if
  // the buffer is smaller). Returns payload bytes copied, or -1 on
  // error/timeout.
  [[nodiscard]] ssize_t Recv(std::span<uint8_t> buffer) noexcept override {
    canfd_frame frame{};
    const ssize_t n = socket_.RecvFdFrame(frame);
    // can_frame and canfd_frame share the can_id/len/data layout, so a
    // classic frame read into a canfd_frame is directly usable.
    if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU))
      return -1;
    const std::size_t copy_len = std::min(static_cast<std::size_t>(frame.len), buffer.size());
    std::memcpy(buffer.data(), frame.data, copy_len);
    return static_cast<ssize_t>(copy_len);
  }

  void SetRecvTimeout(int timeout_ms) noexcept override { socket_.SetRecvTimeout(timeout_ms); }

  void SetRecvBufferSize(int size) noexcept override { socket_.SetRecvBufferSize(size); }

  [[nodiscard]] bool is_open() const noexcept override { return socket_.is_open(); }

 private:
  CanFdTransportConfig config_;
  canid_t tx_id_;
  __u8 tx_flags_;
  CanSocket socket_;
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_CAN_CANFD_TRANSPORT_HPP_
