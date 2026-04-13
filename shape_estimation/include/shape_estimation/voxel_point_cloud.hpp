#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <cstdint>
#include <map>
#include <span>
#include <unordered_map>
#include <vector>

namespace shape_estimation {

/// Voxel 기반 포인트 클라우드 관리
/// - 공간 해싱으로 중복 제거
/// - 시간 기반 만료
/// - 최대 포인트 수 제한
class VoxelPointCloud {
 public:
  struct Config {
    double voxel_resolution_m{0.002};   // 2mm voxel
    int max_points{2048};
    double expiry_duration_sec{5.0};
  };

  VoxelPointCloud();
  explicit VoxelPointCloud(const Config& config);

  /// valid한 surface_points를 추가. voxel 중복 시 이동 평균 업데이트.
  void AddSnapshot(const ToFSnapshot& snapshot);

  /// 만료된 포인트 제거
  void RemoveExpired(uint64_t current_time_ns);

  /// 모든 유효 포인트 반환
  [[nodiscard]] std::vector<PointWithNormal> GetPoints() const;

  /// 현재 포인트 수
  [[nodiscard]] int Size() const noexcept;

  /// 초기화
  void Clear();

 private:
  struct VoxelEntry {
    PointWithNormal point;
    int update_count{1};  // 이동 평균 카운트
  };

  // 3D 좌표 → 정수 voxel key
  [[nodiscard]] int64_t ComputeVoxelKey(const Eigen::Vector3d& p) const noexcept;

  Config config_;
  std::unordered_map<int64_t, VoxelEntry> voxels_;

  // timestamp → voxel_key 정렬 인덱스: FIFO eviction O(log N)
  // multimap: 동일 timestamp에 여러 voxel 가능
  std::multimap<uint64_t, int64_t> timestamp_index_;

  // 만료 시간 (ns)
  uint64_t expiry_ns_{0};
};

}  // namespace shape_estimation
