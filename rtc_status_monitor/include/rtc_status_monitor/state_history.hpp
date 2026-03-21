#pragma once

// ── Project headers ──────────────────────────────────────────────────────────
#include "rtc_status_monitor/failure_types.hpp"

// ── C++ stdlib ───────────────────────────────────────────────────────────────
#include <array>
#include <chrono>
#include <cstddef>
#include <vector>

namespace rtc {

// ── StateHistoryEntry ────────────────────────────────────────────────────────
/// Single snapshot recorded at each 10 Hz monitor cycle.
/// Stored in a ring buffer for failure log output.
struct StateHistoryEntry {
  std::chrono::steady_clock::time_point timestamp{};
  int32_t robot_mode{-1};
  int32_t safety_mode{0};
  bool    program_running{false};
  std::array<double, kNumJoints> q{};
  std::array<double, kNumJoints> qd{};
  std::array<double, kNumJoints> tracking_error{};
  bool joint_limit_warning{false};
};

// ── StateHistory ─────────────────────────────────────────────────────────────
/// Fixed-size circular buffer for state snapshots.
/// Capacity defaults to 100 entries (10 seconds at 10 Hz).
///
/// NOT thread-safe — the caller (RtcStatusMonitor) guards access
/// with history_mutex_.
class StateHistory {
public:
  static constexpr std::size_t kDefaultCapacity = 100;

  explicit StateHistory(std::size_t capacity = kDefaultCapacity)
    : entries_(capacity) {}

  /// Append a new entry, overwriting the oldest if full.
  void Push(const StateHistoryEntry& entry) noexcept {
    entries_[write_pos_ % entries_.size()] = entry;
    ++write_pos_;
    if (count_ < entries_.size()) {
      ++count_;
    }
  }

  /// Return all stored entries in chronological order (oldest first).
  [[nodiscard]] std::vector<StateHistoryEntry> GetAll() const {
    std::vector<StateHistoryEntry> result;
    result.reserve(count_);
    if (count_ < entries_.size()) {
      for (std::size_t i = 0; i < count_; ++i) {
        result.push_back(entries_[i]);
      }
    } else {
      const std::size_t start = write_pos_ % entries_.size();
      for (std::size_t i = 0; i < entries_.size(); ++i) {
        result.push_back(entries_[(start + i) % entries_.size()]);
      }
    }
    return result;
  }

  /// Number of entries currently stored.
  [[nodiscard]] std::size_t Count() const noexcept { return count_; }

  /// Clear all entries.
  void Clear() noexcept {
    write_pos_ = 0;
    count_     = 0;
  }

private:
  std::vector<StateHistoryEntry> entries_;
  std::size_t write_pos_{0};
  std::size_t count_{0};
};

}  // namespace rtc
