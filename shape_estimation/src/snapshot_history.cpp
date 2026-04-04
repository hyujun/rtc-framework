#include "shape_estimation/snapshot_history.hpp"

#include <algorithm>

namespace shape_estimation {

SnapshotHistory::SnapshotHistory(double max_duration_sec, size_t max_count)
    : max_duration_sec_(max_duration_sec), max_count_(max_count) {
  buffer_.reserve(max_count);
}

void SnapshotHistory::Push(const ToFSnapshot& snapshot) {
  buffer_.push_back(snapshot);
  PruneOld();
}

const std::vector<ToFSnapshot>& SnapshotHistory::Snapshots() const noexcept {
  return buffer_;
}

void SnapshotHistory::Clear() {
  buffer_.clear();
}

size_t SnapshotHistory::Size() const noexcept {
  return buffer_.size();
}

void SnapshotHistory::PruneOld() {
  // 수량 제한: 초과분 앞에서 제거
  if (buffer_.size() > max_count_) {
    const auto excess = buffer_.size() - max_count_;
    buffer_.erase(buffer_.begin(),
                  buffer_.begin() + static_cast<ptrdiff_t>(excess));
  }

  // 시간 제한: 최신 타임스탬프 기준 만료 항목 제거
  if (buffer_.size() < 2) {
    return;
  }

  const uint64_t newest_ns = buffer_.back().timestamp_ns;
  const auto duration_ns =
      static_cast<uint64_t>(max_duration_sec_ * 1'000'000'000.0);

  if (newest_ns <= duration_ns) {
    return;  // 오버플로 방지
  }

  const uint64_t cutoff_ns = newest_ns - duration_ns;

  auto it = std::lower_bound(
      buffer_.begin(), buffer_.end(), cutoff_ns,
      [](const ToFSnapshot& snap, uint64_t cutoff) {
        return snap.timestamp_ns < cutoff;
      });

  if (it != buffer_.begin()) {
    buffer_.erase(buffer_.begin(), it);
  }
}

}  // namespace shape_estimation
