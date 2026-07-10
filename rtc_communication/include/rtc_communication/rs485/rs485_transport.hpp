#ifndef RTC_COMMUNICATION_RS485_RS485_TRANSPORT_HPP_
#define RTC_COMMUNICATION_RS485_RS485_TRANSPORT_HPP_

#include "rtc_communication/rs485/serial_port.hpp"
#include "rtc_communication/transport_interface.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
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
// per call, or -1 on timeout, error, or when the frame exceeds the caller's
// buffer (the oversize frame is dropped, never silently truncated).
// wire-integrity checks (CRC) and field parsing belong to the PacketCodec,
// keeping this transport free of any vendor-specific logic. Send() writes a
// caller-encoded frame verbatim (completing short tty writes).
struct Rs485TransportConfig {
  std::string device{"/dev/ttyUSB0"};
  int baud_rate{57600};
  int data_bits{8};
  SerialParity parity{SerialParity::kNone};
  int stop_bits{1};

  // RS485 half-duplex direction control. When rs485_enabled the kernel toggles
  // DE/RE via TIOCSRS485 and Open() disables drain_after_send (the driver
  // bounds the turnaround); if the ioctl is unsupported the transport falls
  // back to draining after each Send, so exactly one path is active. Adapters
  // with hardware auto-direction need neither (set both false). NOTE: on the
  // drain fallback path Send() blocks until the frame has physically left the
  // UART (~frame_bytes * 10 / baud seconds) — budget for this if Send() is
  // called from a periodic RT tick.
  bool rs485_enabled{true};
  bool rts_on_send{true};
  bool rts_after_send{false};
  int delay_rts_before_us{0};
  int delay_rts_after_us{0};
  bool drain_after_send{true};

  // On 2-wire buses whose receiver stays enabled during transmit (no
  // TIOCSRS485, no hardware echo suppression) every transmitted frame is
  // echoed back into the rx path and would be framed by Recv() as if it were
  // a device response. When true, Send() drains and then reads back and
  // discards exactly the transmitted byte count. Enable only on hardware that
  // echoes every transmitted byte — on a non-echoing line the discard read
  // costs one receive timeout per Send and may consume response bytes.
  bool discard_echo{false};

  // -- Framer parameters -----------------------------------------------------
  // sync_pattern: leading byte sequence marking a frame start. Empty selects
  //   fixed-length framing (see length_adjust).
  // length_offset: byte offset of the length field from the frame start.
  // length_size: width of the length field (0, 1, or 2 bytes). 0 => fixed.
  // length_little_endian: endianness of a 2-byte length field.
  // length_adjust: added to the parsed length so that
  //   frame_total = length_offset + length_size + length_value + length_adjust.
  //   For Dynamixel 2.0 (length counts inst+params+crc) this is 0. For pure
  //   fixed-length framing, leave sync_pattern empty, set length_offset 0,
  //   length_size 0, and length_adjust to the frame size (every field in the
  //   frame_total formula above must be accounted for — a non-zero
  //   length_offset still contributes to the total).
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
    // reject TIOCSRS485. When the kernel accepts it the driver bounds the
    // half-duplex turnaround, so the blocking drain fallback is redundant and
    // is disabled; on failure we rely on drain_after_send instead.
    if (config_.rs485_enabled &&
        port_.EnableRs485(config_.rts_on_send, config_.rts_after_send, config_.delay_rts_before_us,
                          config_.delay_rts_after_us)) {
      config_.drain_after_send = false;
    }
    have_ = 0;
    return true;
  }

  // Only closes the fd. The framer state (have_/buf_) is reset by Open(), not
  // here: Transceiver::Stop() calls Close() from the control thread to wake a
  // recv thread that may be blocked inside Recv(), so Close() must not touch
  // state that Recv() is concurrently reading/writing.
  void Close() noexcept override { port_.Close(); }

  // Writes a caller-encoded frame verbatim (completing short tty writes). On a
  // half-duplex line without kernel direction control, drains before returning
  // so the caller may flip to receive; with discard_echo, also consumes the
  // self-echo of the transmitted bytes. Returns bytes written, or -1 on error.
  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept override {
    const ssize_t n = port_.Write(data);
    if (n < 0)
      return -1;
    if (config_.drain_after_send || config_.discard_echo)
      port_.Drain();
    if (config_.discard_echo)
      DiscardEcho(data.size());
    return n;
  }

  // Returns exactly one complete frame per call, running the stateful framer
  // over the byte stream. Returns -1 on timeout, error, or when the frame is
  // larger than buffer.size() (the oversize frame is dropped, never silently
  // truncated); partially received bytes are retained for the next call. The
  // timeout bounds the WHOLE call, not just each read — a trickle of non-frame
  // bytes cannot pin the caller (worst case one extra VTIME window past the
  // deadline). With the timeout disabled (<= 0) Recv() blocks until a frame.
  [[nodiscard]] ssize_t Recv(std::span<uint8_t> buffer) noexcept override {
    const bool bounded = config_.recv_timeout_ms > 0;
    const int64_t deadline_ns =
        bounded ? MonotonicNs() + int64_t{config_.recv_timeout_ms} * 1'000'000 : 0;
    for (;;) {
      // 1. Resync: align buf_ so it starts with sync_pattern.
      if (!config_.sync_pattern.empty() && !Resync()) {
        if (!ReadMore(bounded, deadline_ns))
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
          if (total > buffer.size()) {
            DropFront(total);  // caller buffer too small: drop, report error.
            return -1;
          }
          std::memcpy(buffer.data(), buf_.data(), total);
          DropFront(total);
          return static_cast<ssize_t>(total);
        }
      }

      // 3. Need more bytes.
      if (!ReadMore(bounded, deadline_ns))
        return -1;
    }
  }

  void SetRecvTimeout(int timeout_ms) noexcept override {
    config_.recv_timeout_ms = timeout_ms;  // keeps the Recv() deadline in sync
    port_.SetRecvTimeout(timeout_ms);
  }

  // No-op: a tty has no SO_RCVBUF equivalent. Kept for interface completeness.
  void SetRecvBufferSize(int /*size*/) noexcept override {}

  [[nodiscard]] bool is_open() const noexcept override { return port_.is_open(); }

 private:
  // Reads more bytes into the reassembly buffer. Returns false on
  // deadline/timeout/error (Recv returns -1, keeping the partial bytes). When
  // the buffer is full without a valid frame, drops one byte to make progress
  // and returns true.
  [[nodiscard]] bool ReadMore(bool bounded, int64_t deadline_ns) noexcept {
    if (bounded && MonotonicNs() >= deadline_ns)
      return false;
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

  // Consumes the self-echo of a just-transmitted frame (2-wire buses whose
  // receiver stays enabled during transmit). Runs after Drain(), so the echoed
  // bytes are already buffered; never consumes more than n bytes and gives up
  // on the first timed-out read.
  void DiscardEcho(std::size_t n) noexcept {
    std::array<uint8_t, 64> scratch{};
    std::size_t discarded = 0;
    while (discarded < n) {
      const std::size_t chunk = std::min(scratch.size(), n - discarded);
      const ssize_t r = port_.Read(std::span<uint8_t>(scratch.data(), chunk));
      if (r <= 0)
        return;
      discarded += static_cast<std::size_t>(r);
    }
  }

  [[nodiscard]] static int64_t MonotonicNs() noexcept {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
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
