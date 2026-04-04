#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <cstddef>
#include <vector>

namespace shape_estimation {

// 최근 ToFSnapshot 시계열을 유지하는 버퍼 (Gap 분석용).
// NRT 스레드에서 사용하므로 std::vector 동적 할당 허용.
class SnapshotHistory {
 public:
  // max_duration_sec: 유지할 최대 시간 (기본 3초)
  // max_count: 최대 저장 수 (기본 300 = 3초 × 100Hz)
  explicit SnapshotHistory(double max_duration_sec = 3.0,
                           size_t max_count = 300);

  void Push(const ToFSnapshot& snapshot);

  // 시간순 정렬된 snapshot 벡터 (oldest → newest)
  [[nodiscard]] const std::vector<ToFSnapshot>& Snapshots() const noexcept;

  void Clear();

  [[nodiscard]] size_t Size() const noexcept;

 private:
  double max_duration_sec_;
  size_t max_count_;
  std::vector<ToFSnapshot> buffer_;

  void PruneOld();
};

}  // namespace shape_estimation
