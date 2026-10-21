#ifndef RTC_COMMUNICATION_CAN_CANFD_TRANSPORT_HPP_
#define RTC_COMMUNICATION_CAN_CANFD_TRANSPORT_HPP_

#include "rtc_communication/can/can_socket.hpp"
#include "rtc_communication/can/can_transport.hpp"
#include "rtc_communication/transport_interface.hpp"

#include <linux/can.h>
#include <time.h>

#include <algorithm>
#include <cstring>

namespace rtc {

// Payload-only CAN FD transport: same contract as CanTransport but with
// CAN_RAW_FD_FRAMES enabled and payload up to CANFD_MAX_DLEN (64 bytes) — a
// PacketCodec used over this transport must keep sizeof(SendPacket)/
// sizeof(RecvPacket) within that bound. An FD-enabled socket receives both
// classic and FD frames; Recv() handles both. Non-standard payload lengths
// (not in the CAN FD DLC set 0-8/12/16/20/24/32/48/64) are padded up by the
// kernel when sent to real controllers.
//
// Beside the payload-only TransportInterface contract, the frame API below
// (SendFrame / RecvFrameWithTimeout / RecvFrameNonBlocking) exposes whole
// canfd_frames for protocols that drive several CAN IDs over one socket.
//
// Shared fields (interface, tx/rx IDs, filter, loopback, timeout) are
// inherited from CanTransportConfig and documented there.
struct CanFdTransportConfig : CanTransportConfig {
  bool bitrate_switch{false};         // CANFD_BRS: transmit data phase at higher bitrate
  bool error_state_indicator{false};  // CANFD_ESI
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
    if (!socket_.SetFdFrames(true) || !ApplyCanSocketConfig(socket_, config_)) {
      socket_.Close();
      return false;
    }
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
  // the buffer is smaller). RTR frames are dropped (their data has no
  // meaning). Returns payload bytes copied, or -1 on error/timeout/RTR.
  [[nodiscard]] ssize_t Recv(std::span<uint8_t> buffer) noexcept override {
    canfd_frame frame{};
    const ssize_t n = socket_.RecvFdFrame(frame);
    // can_frame and canfd_frame share the can_id/len/data layout, so a
    // classic frame read into a canfd_frame is directly usable.
    if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU))
      return -1;
    if (frame.can_id & CAN_RTR_FLAG)
      return -1;
    const std::size_t copy_len = std::min(static_cast<std::size_t>(frame.len), buffer.size());
    std::memcpy(buffer.data(), frame.data, copy_len);
    return static_cast<ssize_t>(copy_len);
  }

  // -- Frame API: ID / len / flags chosen per call ----------------------------
  // The config's tx_can_id / bitrate_switch / error_state_indicator are NOT
  // applied here: the caller fills can_id (with CAN_EFF_FLAG for a 29-bit ID),
  // len and flags (CANFD_BRS / CANFD_ESI). Reception passes the configured rx
  // filter and hands the frame over as-is (RTR frames are not dropped); the
  // frame size tells classic (CAN_MTU) from FD (CANFD_MTU) on both sides.
  // Send/Recv are noexcept, allocation-free and never sleep in the kernel
  // beyond the given timeout; errno is left as the failing syscall set it.

  // Sends one FD frame (CANFD_MTU: the FDF bit is set even for len <= 8).
  // Returns false on error — EINVAL for len > CANFD_MAX_DLEN, ENOBUFS when the
  // tx queue is full, EAGAIN when the socket send buffer is full, ENETDOWN.
  [[nodiscard]] bool SendFrame(const canfd_frame& frame) noexcept {
    return socket_.SendFdFrameNonBlocking(frame) >= 0;
  }

  // Sends one classic frame (CAN_MTU, no FDF bit) on the same socket — for a
  // peer ID that speaks classic CAN beside the FD stream. Same errors.
  [[nodiscard]] bool SendFrame(const can_frame& frame) noexcept {
    return socket_.SendFrameNonBlocking(frame) >= 0;
  }

  // Waits up to `timeout` (relative, ppoll) for a frame and reads it without
  // blocking. Returns CAN_MTU / CANFD_MTU, 0 when nothing arrived, -1 on error
  // (EBADF when closed).
  [[nodiscard]] ssize_t RecvFrameWithTimeout(canfd_frame& frame, const timespec& timeout) noexcept {
    return socket_.RecvFdFrameWithTimeout(frame, timeout);
  }

  // Reads one queued frame without blocking. Returns CAN_MTU / CANFD_MTU, 0
  // when nothing is queued, -1 on error (EBADF when closed).
  [[nodiscard]] ssize_t RecvFrameNonBlocking(canfd_frame& frame) noexcept {
    return socket_.RecvFdFrameNonBlocking(frame);
  }

  // Sets SO_PRIORITY (tx qdisc band / skb priority; values > 6 need
  // CAP_NET_ADMIN). Setup-time call: after Open(), before the RT loop starts.
  [[nodiscard]] bool SetPriority(int priority) noexcept { return socket_.SetPriority(priority); }

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
