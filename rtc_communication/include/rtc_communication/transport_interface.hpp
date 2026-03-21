#ifndef RTC_COMMUNICATION_TRANSPORT_INTERFACE_HPP_
#define RTC_COMMUNICATION_TRANSPORT_INTERFACE_HPP_

#include <cstddef>
#include <cstdint>
#include <span>
#include <sys/types.h>

namespace rtc {

class TransportInterface {
 public:
  virtual ~TransportInterface() = default;

  TransportInterface(const TransportInterface&) = delete;
  TransportInterface& operator=(const TransportInterface&) = delete;
  TransportInterface(TransportInterface&&) = delete;
  TransportInterface& operator=(TransportInterface&&) = delete;

  [[nodiscard]] virtual bool Open() = 0;
  virtual void Close() noexcept = 0;

  [[nodiscard]] virtual ssize_t Send(
      std::span<const uint8_t> data) noexcept = 0;
  [[nodiscard]] virtual ssize_t Recv(
      std::span<uint8_t> buffer) noexcept = 0;

  virtual void SetRecvTimeout(int timeout_ms) noexcept = 0;
  virtual void SetRecvBufferSize(int size) noexcept = 0;
  [[nodiscard]] virtual bool is_open() const noexcept = 0;

 protected:
  TransportInterface() = default;
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_TRANSPORT_INTERFACE_HPP_
