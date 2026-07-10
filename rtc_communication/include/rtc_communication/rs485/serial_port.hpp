#ifndef RTC_COMMUNICATION_RS485_SERIAL_PORT_HPP_
#define RTC_COMMUNICATION_RS485_SERIAL_PORT_HPP_

// RAII wrapper for a POSIX serial (tty) file descriptor used for RS485 links.
//
// Designed for real-time use:
//   - No dynamic allocation after construction
//   - All I/O methods are noexcept
//   - Deterministic cleanup in destructor
//
// Unlike UDP/CAN sockets, a serial line is a raw byte stream with no kernel
// framing: a single Read() returns whatever bytes are buffered, not one logical
// packet. Frame reassembly is therefore the transport's responsibility (see
// rs485_transport.hpp) — this class only owns the line: open, termios line
// configuration, optional RS485 half-duplex direction control, and raw I/O.
//
// The socket_options.hpp helpers are NOT reused here: SO_RCVBUF / SO_RCVTIMEO
// are socket-only and have no effect on a tty. The receive timeout is realised
// through termios VMIN/VTIME instead.

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>

namespace rtc {

// Line framing parameters (data bits / parity / stop bits). RS485 with
// Dynamixel Protocol 2.0 uses 8N1; the enum keeps the wrapper generic.
enum class SerialParity { kNone, kEven, kOdd };

class SerialPort {
 public:
  SerialPort() noexcept = default;

  ~SerialPort() { Close(); }

  // Non-copyable, non-movable (owns raw fd).
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  SerialPort(SerialPort&&) = delete;
  SerialPort& operator=(SerialPort&&) = delete;

  // -- Line lifecycle --------------------------------------------------------

  // Opens the tty device (e.g. "/dev/ttyUSB0"). The fd is left in blocking
  // mode; the receive timeout is governed by VMIN/VTIME (SetRecvTimeout()).
  // Returns false on failure.
  [[nodiscard]] bool Open(std::string_view device) noexcept {
    if (fd_ >= 0)
      return true;  // already open

    char path_buf[kMaxPath]{};
    const auto len = std::min(device.size(), sizeof(path_buf) - 1);
    std::memcpy(path_buf, device.data(), len);

    fd_ = ::open(path_buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
      return false;

    // Drop O_NONBLOCK so reads honour VMIN/VTIME instead of returning EAGAIN.
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK) < 0) {
      Close();
      return false;
    }
    return true;
  }

  // Applies raw-mode termios settings: baud, data bits, parity, stop bits.
  // Returns false if the fd is closed or the baud rate is unsupported.
  [[nodiscard]] bool ConfigureLine(int baud_rate, int data_bits, SerialParity parity,
                                   int stop_bits) noexcept {
    if (fd_ < 0)
      return false;

    speed_t speed{};
    if (!BaudToSpeed(baud_rate, speed))
      return false;

    termios tio{};
    if (::tcgetattr(fd_, &tio) < 0)
      return false;

    cfmakeraw(&tio);

    // Data bits.
    tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    switch (data_bits) {
      case 5:
        tio.c_cflag |= CS5;
        break;
      case 6:
        tio.c_cflag |= CS6;
        break;
      case 7:
        tio.c_cflag |= CS7;
        break;
      case 8:
        tio.c_cflag |= CS8;
        break;
      default:
        return false;
    }

    // Parity.
    switch (parity) {
      case SerialParity::kNone:
        tio.c_cflag &= ~static_cast<tcflag_t>(PARENB);
        break;
      case SerialParity::kEven:
        tio.c_cflag |= PARENB;
        tio.c_cflag &= ~static_cast<tcflag_t>(PARODD);
        break;
      case SerialParity::kOdd:
        tio.c_cflag |= PARENB | PARODD;
        break;
    }

    // Stop bits.
    if (stop_bits == 2)
      tio.c_cflag |= CSTOPB;
    else if (stop_bits == 1)
      tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
    else
      return false;

    // Local line, receiver enabled; no flow control.
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);

    // Blocking read with an overall timeout comes from VMIN/VTIME
    // (SetRecvTimeout()); start from a non-blocking poll default.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) < 0 || cfsetospeed(&tio, speed) < 0)
      return false;

    return ::tcsetattr(fd_, TCSANOW, &tio) == 0;
  }

  // Enables kernel RS485 half-duplex direction control (TIOCSRS485). The driver
  // toggles the DE/RE line around each write. Not all adapters/drivers support
  // it (many USB-RS485 bridges auto-direct in hardware); returns false when the
  // ioctl is rejected, in which case the caller may proceed without it.
  [[nodiscard]] bool EnableRs485(bool rts_on_send, bool rts_after_send, int delay_before_us,
                                 int delay_after_us) noexcept {
    if (fd_ < 0)
      return false;
    serial_rs485 rs485{};
    rs485.flags = SER_RS485_ENABLED;
    if (rts_on_send)
      rs485.flags |= SER_RS485_RTS_ON_SEND;
    if (rts_after_send)
      rs485.flags |= SER_RS485_RTS_AFTER_SEND;
    rs485.delay_rts_before_send = static_cast<__u32>(std::max(0, delay_before_us));
    rs485.delay_rts_after_send = static_cast<__u32>(std::max(0, delay_after_us));
    return ::ioctl(fd_, TIOCSRS485, &rs485) == 0;
  }

  void Close() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // -- Options ---------------------------------------------------------------

  // Sets the receive timeout via termios. timeout_ms <= 0 disables the timeout
  // (VMIN=1/VTIME=0: Read() blocks until at least one byte arrives), matching
  // the socket transports' SO_RCVTIMEO=0 semantics. Positive values use
  // VMIN=0/VTIME: Read() blocks until at least one byte arrives or the timeout
  // expires (returning 0). VTIME has decisecond granularity, so the value is
  // rounded UP to the next 100 ms and capped at 25.5 s.
  void SetRecvTimeout(int timeout_ms) noexcept {
    if (fd_ < 0)
      return;
    termios tio{};
    if (::tcgetattr(fd_, &tio) < 0)
      return;
    if (timeout_ms <= 0) {
      tio.c_cc[VMIN] = 1;
      tio.c_cc[VTIME] = 0;
    } else {
      tio.c_cc[VMIN] = 0;
      const int deciseconds = (timeout_ms + 99) / 100;
      tio.c_cc[VTIME] = static_cast<cc_t>(std::min(deciseconds, 255));
    }
    ::tcsetattr(fd_, TCSANOW, &tio);
  }

  // -- I/O (allocation-free, noexcept) ---------------------------------------

  // Reads up to buf.size() bytes. Returns bytes read (0 on VTIME timeout with
  // no data; blocks indefinitely when the timeout is disabled), or -1 on error.
  [[nodiscard]] ssize_t Read(std::span<uint8_t> buf) noexcept {
    return ::read(fd_, buf.data(), buf.size());
  }

  // Writes all bytes in data, looping over short writes (full tty tx buffer)
  // and retrying on EINTR — a tty write is not frame-atomic like a UDP/CAN
  // send, so a single ::write could otherwise put a torn frame on the wire.
  // Returns data.size() on success, or -1 on error.
  [[nodiscard]] ssize_t Write(std::span<const uint8_t> data) noexcept {
    std::size_t written = 0;
    while (written < data.size()) {
      const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
      if (n < 0) {
        if (errno == EINTR)
          continue;
        return -1;
      }
      written += static_cast<std::size_t>(n);
    }
    return static_cast<ssize_t>(written);
  }

  // Blocks until the kernel tx buffer has been transmitted. On a half-duplex
  // RS485 line without hardware auto-direction this bounds the turnaround
  // before the caller flips to receive. noexcept, no allocation.
  void Drain() noexcept {
    if (fd_ >= 0)
      ::tcdrain(fd_);
  }

  // -- Accessors -------------------------------------------------------------

  [[nodiscard]] int fd() const noexcept { return fd_; }

  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

 private:
  static constexpr std::size_t kMaxPath = 256;

  // Maps a numeric baud rate to a termios speed_t constant. Covers the rates
  // Dynamixel and common USB-RS485 bridges use.
  [[nodiscard]] static bool BaudToSpeed(int baud, speed_t& out) noexcept {
    switch (baud) {
      case 9600:
        out = B9600;
        return true;
      case 19200:
        out = B19200;
        return true;
      case 38400:
        out = B38400;
        return true;
      case 57600:
        out = B57600;
        return true;
      case 115200:
        out = B115200;
        return true;
      case 230400:
        out = B230400;
        return true;
      case 460800:
        out = B460800;
        return true;
      case 500000:
        out = B500000;
        return true;
      case 921600:
        out = B921600;
        return true;
      case 1000000:
        out = B1000000;
        return true;
      case 2000000:
        out = B2000000;
        return true;
      case 3000000:
        out = B3000000;
        return true;
      case 4000000:
        out = B4000000;
        return true;
      default:
        return false;
    }
  }

  int fd_{-1};
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_RS485_SERIAL_PORT_HPP_
