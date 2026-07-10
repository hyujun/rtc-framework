#ifndef RTC_COMMUNICATION_RS485_RS485_TRANSPORT_HPP_
#define RTC_COMMUNICATION_RS485_RS485_TRANSPORT_HPP_

#include "rtc_communication/rs485/serial_port.hpp"
#include "rtc_communication/transport_interface.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace rtc {

// TransportInterface over an RS485 serial line.
//
// A serial link is a raw byte stream with no kernel framing, so unlike the UDP
// and CAN transports this class runs a stateful framer inside Recv() to restore
// packet boundaries. The framer is protocol-neutral: it is parameterised by a
// sync prefix and a length field, so it fits any header/length-prefixed wire
// format (e.g. ROBOTIS Dynamixel Protocol 2.0: sync {FF FF FD 00}, id[1],
// length[2, little-endian], then length bytes of instruction/params/CRC).
//
// Payload contract: Recv() returns one complete raw frame (sync .. trailer)
// per call; wire-integrity checks (CRC) and field parsing belong to the
// PacketCodec, keeping this transport free of any vendor-specific logic. Send()
// writes a caller-encoded frame verbatim.
struct Rs485TransportConfig {
  std::string device{"/dev/ttyUSB0"};
  int baud_rate{57600};
  int data_bits{8};
  SerialParity parity{SerialParity::kNone};
  int stop_bits{1};

  // RS485 half-duplex direction control. When rs485_enabled the kernel toggles
  // DE/RE via TIOCSRS485; if the ioctl is unsupported the transport falls back
  // to draining after each Send (drain_after_send) so the caller can rely on
  // one path or the other. Adapters with hardware auto-direction need neither.
  bool rs485_enabled{true};
  bool rts_on_send{true};
  bool rts_after_send{false};
  int delay_rts_before_us{0};
  int delay_rts_after_us{0};
  bool drain_after_send{true};

  // -- Framer parameters -----------------------------------------------------
  // sync_pattern: leading byte sequence marking a frame start. Empty selects
  //   fixed-length framing (see length_adjust).
  // length_offset: byte offset of the length field from the frame start.
  // length_size: width of the length field (0, 1, or 2 bytes). 0 => fixed.
  // length_little_endian: endianness of a 2-byte length field.
  // length_adjust: added to the parsed length so that
  //   frame_total = length_offset + length_size + length_value + length_adjust.
  //   For Dynamixel 2.0 (length counts inst+params+crc) this is 0. For pure
  //   fixed-length framing, leave sync_pattern empty, length_size 0, and set
  //   length_adjust to the frame size.
  std::vector<uint8_t> sync_pattern{0xFF, 0xFF, 0xFD, 0x00};
  std::size_t length_offset{5};
  std::size_t length_size{2};
  bool length_little_endian{true};
  int length_adjust{0};
  // Upper bound on a single frame; frames larger than this (or than the
  // physical buffer) are treated as corruption and resynced byte-by-byte.
  std::size_t max_frame_size{512};

  int recv_timeout_ms{100};
};

class Rs485Transport : public TransportInterface {
 public:
  // Physical reassembly buffer bound.
  static constexpr std::size_t kBufferCapacity = 1024;

  explicit Rs485Transport(const Rs485TransportConfig& config) noexcept : config_(config) {
    if (config_.max_frame_size > kBufferCapacity)
      config_.max_frame_size = kBufferCapacity;
  }

  [[nodiscard]] bool Open() override {
    if (!port_.Open(config_.device))
      return false;
    if (!port_.ConfigureLine(config_.baud_rate, config_.data_bits, config_.parity,
                             config_.stop_bits)) {
      port_.Close();
      return false;
    }
    port_.SetRecvTimeout(config_.recv_timeout_ms);
    // RS485 direction control is best-effort: many USB bridges auto-direct and
    // reject TIOCSRS485. On failure we rely on drain_after_send instead.
    if (config_.rs485_enabled) {
      (void)port_.EnableRs485(config_.rts_on_send, config_.rts_after_send,
                              config_.delay_rts_before_us, config_.delay_rts_after_us);
    }
    have_ = 0;
    return true;
  }

  void Close() noexcept override {
    port_.Close();
    have_ = 0;
  }

  // Writes a caller-encoded frame verbatim. On a half-duplex line without
  // kernel direction control, drains before returning so the caller may flip to
  // receive. Returns bytes written, or -1 on error.
  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept override {
    const ssize_t n = port_.Write(data);
    if (n < 0)
      return -1;
    if (config_.drain_after_send)
      port_.Drain();
    return n;
  }

  // Returns exactly one complete frame per call (truncated to buffer.size()),
  // running the stateful framer over the byte stream. Returns -1 on
  // timeout/error; partially received bytes are retained for the next call.
  [[nodiscard]] ssize_t Recv(std::span<uint8_t> buffer) noexcept override {
    for (;;) {
      // 1. Resync: align buf_ so it starts with sync_pattern.
      if (!config_.sync_pattern.empty() && !Resync()) {
        if (!ReadMore())
          return -1;
        continue;
      }

      // 2. If the length field is buffered, compute and check the frame total.
      const std::size_t header_end = config_.length_offset + config_.length_size;
      if (have_ >= header_end) {
        const std::size_t total = FrameTotal();
        if (total == 0 || total > config_.max_frame_size) {
          DropFront(1);  // corrupt length: drop one byte and resync.
          continue;
        }
        if (have_ >= total) {
          const std::size_t copy_len = std::min(total, buffer.size());
          std::memcpy(buffer.data(), buf_.data(), copy_len);
          DropFront(total);
          return static_cast<ssize_t>(copy_len);
        }
      }

      // 3. Need more bytes.
      if (!ReadMore())
        return -1;
    }
  }

  void SetRecvTimeout(int timeout_ms) noexcept override { port_.SetRecvTimeout(timeout_ms); }

  // No-op: a tty has no SO_RCVBUF equivalent. Kept for interface completeness.
  void SetRecvBufferSize(int /*size*/) noexcept override {}

  [[nodiscard]] bool is_open() const noexcept override { return port_.is_open(); }

 private:
  // Reads more bytes into the reassembly buffer. Returns false on timeout/error
  // (Recv returns -1, keeping the partial bytes). When the buffer is full
  // without a valid frame, drops one byte to make progress and returns true.
  [[nodiscard]] bool ReadMore() noexcept {
    if (have_ >= kBufferCapacity) {
      DropFront(1);
      return true;
    }
    const ssize_t n = port_.Read(std::span<uint8_t>(buf_.data() + have_, kBufferCapacity - have_));
    if (n <= 0)
      return false;  // timeout (0) or error (-1)
    have_ += static_cast<std::size_t>(n);
    return true;
  }

  // Aligns buf_ so it begins with sync_pattern. Returns true when a full sync
  // prefix is at the front, false when more bytes are needed (buf_ trimmed to
  // the longest possible partial prefix at the tail).
  [[nodiscard]] bool Resync() noexcept {
    const auto& pat = config_.sync_pattern;
    for (std::size_t i = 0; i < have_; ++i) {
      const std::size_t avail = have_ - i;
      const std::size_t cmp = std::min(avail, pat.size());
      if (std::memcmp(buf_.data() + i, pat.data(), cmp) != 0)
        continue;
      if (i > 0)
        DropFront(i);
      return cmp == pat.size();  // full match => synced; partial => need more.
    }
    // No match anywhere: keep only the last (pat.size()-1) bytes as a possible
    // partial prefix straddling the next read.
    if (have_ >= pat.size())
      DropFront(have_ - (pat.size() - 1));
    return false;
  }

  // Parses the length field and returns the total frame size in bytes (0 if the
  // parsed total is non-positive).
  [[nodiscard]] std::size_t FrameTotal() const noexcept {
    std::size_t len_value = 0;
    if (config_.length_size == 1) {
      len_value = buf_[config_.length_offset];
    } else if (config_.length_size == 2) {
      const std::size_t b0 = buf_[config_.length_offset];
      const std::size_t b1 = buf_[config_.length_offset + 1];
      len_value = config_.length_little_endian ? (b0 | (b1 << 8)) : ((b0 << 8) | b1);
    }
    const long total = static_cast<long>(config_.length_offset) +
                       static_cast<long>(config_.length_size) + static_cast<long>(len_value) +
                       static_cast<long>(config_.length_adjust);
    return (total <= 0) ? 0 : static_cast<std::size_t>(total);
  }

  // Discards the first n bytes, shifting the remainder to the front.
  void DropFront(std::size_t n) noexcept {
    n = std::min(n, have_);
    if (n == 0)
      return;
    const std::size_t rest = have_ - n;
    if (rest > 0)
      std::memmove(buf_.data(), buf_.data() + n, rest);
    have_ = rest;
  }

  Rs485TransportConfig config_;
  SerialPort port_;
  std::array<uint8_t, kBufferCapacity> buf_{};
  std::size_t have_{0};
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_RS485_RS485_TRANSPORT_HPP_
