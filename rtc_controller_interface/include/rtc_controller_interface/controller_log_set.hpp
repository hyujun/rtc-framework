#ifndef RTC_CONTROLLER_INTERFACE_CONTROLLER_LOG_SET_HPP_
#define RTC_CONTROLLER_INTERFACE_CONTROLLER_LOG_SET_HPP_

// ControllerLogSet — opt-in helper for controllers that own one or more
// CSV data logs (Phase C, Q-MSG-2(d) + Q-CORE locks).
//
// Holds N producer / logger pairs, one per (PodT, instance) registration.
// CM is intentionally not involved: each controller's own LifecycleNode
// owns this set, drives the drain timer in its own callback group, and
// closes the files on deactivate.
//
// Storage uses a polymorphic base (LogChannelBase) so adding a new robot
// (KUKA, Franka, …) only requires defining the robot's POD type plus
// header/row writers — no edit to this header. The controller code sees
// a strongly-typed LogHandle<PodT> with a non-virtual Push().
//
// Threading contract:
//   - Push site: ONLY the RT thread that runs the controller's Compute()
//     (Q-ACTIVITY-GATING). Inactive controllers must not push — CM's
//     dispatch already gates this for free.
//   - Drain site: the controller's non-RT drain timer (one timer for the
//     whole set, single-threaded — no inter-channel locking needed).
//
// File path: <session>/controllers/<config_key>/<instance>.csv —
// resolved at RegisterLog() time relative to the session dir.
//
// Header-only.

#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/logging/thread_csv_logger.hpp"
#include "rtc_base/logging/thread_csv_producer.hpp"

#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace rtc {

namespace detail {

class LogChannelBase {
public:
  virtual ~LogChannelBase() = default;

  /// Move pending samples FIFO into the CSV file. Returns the number of
  /// samples written. Non-RT.
  virtual std::size_t Drain() noexcept = 0;

  /// Lifetime drop count (overflow on Push).
  [[nodiscard]] virtual std::uint64_t DropCount() const noexcept = 0;

  /// Resolved file path (for diagnostic logging).
  [[nodiscard]] virtual const std::filesystem::path &Path() const noexcept = 0;

  /// Instance string the controller registered under.
  [[nodiscard]] virtual std::string_view Instance() const noexcept = 0;
};

} // namespace detail

/// Controller-side handle to a single registered log channel. Push()
/// from inside Compute() — wait-free, drop-on-full.
template <typename PodT, std::size_t Capacity = 512> class LogHandle {
public:
  using Producer = ThreadCsvProducer<PodT, Capacity>;

  LogHandle() = default;
  explicit LogHandle(Producer *p) noexcept : producer_(p) {}

  /// True if RegisterLog() succeeded for this handle.
  [[nodiscard]] explicit operator bool() const noexcept {
    return producer_ != nullptr;
  }

  /// RT-safe push. No-op (returns false) when handle is unbound — keeps
  /// controller code simple in the "logging disabled" path.
  bool Push(const PodT &pod) noexcept {
    if (producer_ == nullptr) {
      return false;
    }
    return producer_->Push(pod);
  }

private:
  Producer *producer_{nullptr};
};

namespace detail {

template <typename PodT, std::size_t Capacity>
class LogChannel final : public LogChannelBase {
public:
  using Producer = ThreadCsvProducer<PodT, Capacity>;
  using Logger = ThreadCsvLogger<PodT>;
  using HeaderWriter = std::function<void(std::ostream &)>;
  using RowWriter = std::function<void(std::ostream &, const PodT &)>;

  LogChannel(std::string instance, HeaderWriter hdr, RowWriter row)
      : instance_(std::move(instance)), hdr_(std::move(hdr)),
        row_(std::move(row)) {}

  bool Open(const std::filesystem::path &path) noexcept {
    return logger_.Open(path, hdr_, row_);
  }

  Producer *producer_ptr() noexcept { return &producer_; }

  std::size_t Drain() noexcept override {
    return producer_.Drain([&](const PodT &p) { logger_.Log(p); });
  }

  [[nodiscard]] std::uint64_t DropCount() const noexcept override {
    return producer_.DropCount();
  }

  [[nodiscard]] const std::filesystem::path &Path() const noexcept override {
    return logger_.Path();
  }

  [[nodiscard]] std::string_view Instance() const noexcept override {
    return instance_;
  }

private:
  std::string instance_;
  HeaderWriter hdr_;
  RowWriter row_;
  Producer producer_{};
  Logger logger_{};
};

} // namespace detail

class ControllerLogSet {
public:
  /// `config_key` ends up under `<session>/controllers/<config_key>/`.
  /// Empty key resolves to `<session>/controllers/` directly (caller
  /// disambiguates via instance).
  ControllerLogSet() = default;
  explicit ControllerLogSet(std::string_view config_key)
      : config_key_(config_key) {}

  ControllerLogSet(const ControllerLogSet &) = delete;
  ControllerLogSet &operator=(const ControllerLogSet &) = delete;

  /// Register a new (PodT, instance) channel. Header is written on first
  /// open; existing files are appended without duplicate header.
  /// Returns an unbound LogHandle on filesystem error — `if (handle)` is
  /// the caller's check.
  template <typename PodT, std::size_t Capacity = 512>
  LogHandle<PodT, Capacity>
  RegisterLog(std::string_view instance,
              typename detail::LogChannel<PodT, Capacity>::HeaderWriter hdr,
              typename detail::LogChannel<PodT, Capacity>::RowWriter row) {
    auto channel = std::make_unique<detail::LogChannel<PodT, Capacity>>(
        std::string(instance), std::move(hdr), std::move(row));
    auto *raw = channel.get();
    const auto path = ResolvePath(instance);
    if (!channel->Open(path)) {
      return LogHandle<PodT, Capacity>{}; // unbound — caller logs warning
    }
    auto *prod = raw->producer_ptr();
    channels_.emplace_back(std::move(channel));
    return LogHandle<PodT, Capacity>{prod};
  }

  /// Single-pass FIFO drain across every registered channel. Returns the
  /// total number of samples written. Non-RT (drain timer thread).
  std::size_t DrainAll() noexcept {
    std::size_t total = 0;
    for (auto &c : channels_) {
      total += c->Drain();
    }
    return total;
  }

  /// Lifetime drop counts, one entry per channel, in registration order.
  [[nodiscard]] std::vector<std::uint64_t> DropCounts() const noexcept {
    std::vector<std::uint64_t> out;
    out.reserve(channels_.size());
    for (const auto &c : channels_) {
      out.push_back(c->DropCount());
    }
    return out;
  }

  [[nodiscard]] std::size_t size() const noexcept { return channels_.size(); }
  [[nodiscard]] bool empty() const noexcept { return channels_.empty(); }

  /// Diagnostic: list of (instance, path) for each channel.
  [[nodiscard]] std::vector<std::pair<std::string_view, std::filesystem::path>>
  Channels() const {
    std::vector<std::pair<std::string_view, std::filesystem::path>> out;
    out.reserve(channels_.size());
    for (const auto &c : channels_) {
      out.emplace_back(c->Instance(), c->Path());
    }
    return out;
  }

private:
  std::filesystem::path ResolvePath(std::string_view instance) const {
    const auto session = ResolveSessionDir();
    auto dir = session / "controllers";
    if (!config_key_.empty()) {
      dir /= config_key_;
    }
    return dir / (std::string(instance) + ".csv");
  }

  std::string config_key_;
  std::vector<std::unique_ptr<detail::LogChannelBase>> channels_;
};

} // namespace rtc

#endif // RTC_CONTROLLER_INTERFACE_CONTROLLER_LOG_SET_HPP_
