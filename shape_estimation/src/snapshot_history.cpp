#include "shape_estimation/snapshot_history.hpp"

#include <algorithm>

namespace shape_estimation {

SnapshotHistory::SnapshotHistory(double max_duration_sec, size_t max_count)
    : max_duration_sec_(max_duration_sec), max_count_(max_count) {}

void SnapshotHistory::Push(const ToFSnapshot& snapshot) {
  buffer_.push_back(snapshot);
  PruneOld();
}

std::vector<ToFSnapshot> SnapshotHistory::Snapshots() const {
  return {buffer_.begin(), buffer_.end()};
}

void SnapshotHistory::Clear() {
  buffer_.clear();
}

size_t SnapshotHistory::Size() const noexcept {
  return buffer_.size();
}

void SnapshotHistory::PruneOld() {
  // 수량 제한: 초과분 앞에서 제거 (deque: O(1) per pop_front)
  while (buffer_.size() > max_count_) {
    buffer_.pop_front();
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

  // deque 앞쪽부터 만료된 항목 pop (timestamp 순 정렬이므로)
  while (!buffer_.empty() && buffer_.front().timestamp_ns < cutoff_ns) {
    buffer_.pop_front();
  }
}

}  // namespace shape_estimation
