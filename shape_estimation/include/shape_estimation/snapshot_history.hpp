#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <cstddef>
#include <deque>
#include <vector>

namespace shape_estimation {

// 최근 ToFSnapshot 시계열을 유지하는 버퍼 (Gap 분석용).
// std::deque 기반: 앞쪽 제거 O(1), 뒤쪽 추가 O(1).
class SnapshotHistory {
 public:
  // max_duration_sec: 유지할 최대 시간 (기본 3초)
  // max_count: 최대 저장 수 (기본 300 = 3초 × 100Hz)
  explicit SnapshotHistory(double max_duration_sec = 3.0,
                           size_t max_count = 300);

  void Push(const ToFSnapshot& snapshot);

  /// snapshot을 vector로 반환 (ProtuberanceDetector 인터페이스 호환)
  [[nodiscard]] std::vector<ToFSnapshot> Snapshots() const;

  void Clear();

  [[nodiscard]] size_t Size() const noexcept;

 private:
  double max_duration_sec_;
  size_t max_count_;
  std::deque<ToFSnapshot> buffer_;

  void PruneOld();
};

}  // namespace shape_estimation
