#include "shape_estimation/voxel_point_cloud.hpp"
#include "shape_estimation/shape_logging.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

namespace shape_estimation {

namespace {
auto logger() { return ::rtc::shape::logging::VoxelLogger(); }
}  // namespace

VoxelPointCloud::VoxelPointCloud() : VoxelPointCloud(Config{}) {}

VoxelPointCloud::VoxelPointCloud(const Config& config)
    : config_(config),
      expiry_ns_(static_cast<uint64_t>(config.expiry_duration_sec * 1.0e9)) {}

void VoxelPointCloud::AddSnapshot(const ToFSnapshot& snapshot) {
  [[maybe_unused]] const auto size_before = voxels_.size();
  for (int i = 0; i < kTotalSensors; ++i) {
    if (!snapshot.readings[static_cast<size_t>(i)].valid) {
      continue;
    }

    const auto& surface_pt = snapshot.surface_points_world[static_cast<size_t>(i)];
    const int finger_idx = i / kSensorsPerFinger;
    const auto& beam_dir = snapshot.beam_directions_world[static_cast<size_t>(finger_idx)];

    // 법선 = 빔 방향의 반대 (표면이 센서를 향함)
    const Eigen::Vector3d normal = -beam_dir;

    // 해당 손가락의 곡률 (valid한 경우)
    const double curvature = snapshot.curvature_valid[static_cast<size_t>(finger_idx)]
                                 ? snapshot.local_curvatures[static_cast<size_t>(finger_idx)]
                                 : 0.0;

    const int64_t key = ComputeVoxelKey(surface_pt);
    auto it = voxels_.find(key);

    if (it != voxels_.end()) {
      // 기존 voxel: 이동 평균 업데이트
      auto& entry = it->second;
      const int n = entry.update_count;
      const double w_old = static_cast<double>(n) / static_cast<double>(n + 1);
      const double w_new = 1.0 / static_cast<double>(n + 1);

      // timestamp_index_ 에서 이전 timestamp 제거 후 새 timestamp 추가
      const uint64_t old_ts = entry.point.timestamp_ns;
      auto range = timestamp_index_.equal_range(old_ts);
      for (auto ti = range.first; ti != range.second; ++ti) {
        if (ti->second == key) {
          timestamp_index_.erase(ti);
          break;
        }
      }

      entry.point.position = w_old * entry.point.position + w_new * surface_pt;
      entry.point.normal = (w_old * entry.point.normal + w_new * normal).normalized();
      entry.point.curvature = w_old * entry.point.curvature + w_new * curvature;
      entry.point.timestamp_ns = snapshot.timestamp_ns;
      entry.update_count = n + 1;

      timestamp_index_.emplace(snapshot.timestamp_ns, key);
    } else {
      // 신규 추가 (max_points 초과 시 oldest 제거)
      if (static_cast<int>(voxels_.size()) >= config_.max_points) {
        RCLCPP_WARN_ONCE(logger(),
                         "최대 포인트 도달 (%d) → FIFO eviction 시작",
                         config_.max_points);
        // timestamp_index_ 앞쪽 = oldest → O(1) 접근
        auto oldest_ti = timestamp_index_.begin();
        voxels_.erase(oldest_ti->second);
        timestamp_index_.erase(oldest_ti);
      }

      VoxelEntry entry;
      entry.point.position = surface_pt;
      entry.point.normal = normal;
      entry.point.curvature = curvature;
      entry.point.timestamp_ns = snapshot.timestamp_ns;
      entry.update_count = 1;
      voxels_.emplace(key, entry);
      timestamp_index_.emplace(snapshot.timestamp_ns, key);
    }
  }
  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                        ::rtc::shape::logging::kThrottleSlowMs,
                        "AddSnapshot: new=%zu, total=%zu",
                        voxels_.size() - size_before, voxels_.size());
}

void VoxelPointCloud::RemoveExpired(uint64_t current_time_ns) {
  if (expiry_ns_ == 0) {
    return;
  }

  const auto size_before = voxels_.size();
  const uint64_t cutoff = (current_time_ns > expiry_ns_)
                              ? (current_time_ns - expiry_ns_)
                              : 0;

  // timestamp_index_ は timestamp 순 정렬 → 앞쪽부터 순차 제거 O(K)
  auto it = timestamp_index_.begin();
  while (it != timestamp_index_.end() && it->first < cutoff) {
    voxels_.erase(it->second);
    it = timestamp_index_.erase(it);
  }

  const auto removed = size_before - voxels_.size();
  if (removed > 0) {
    static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                          ::rtc::shape::logging::kThrottleSlowMs,
                          "RemoveExpired: removed=%zu, remaining=%zu",
                          removed, voxels_.size());
  }
}

std::vector<PointWithNormal> VoxelPointCloud::GetPoints() const {
  std::vector<PointWithNormal> result;
  result.reserve(voxels_.size());
  for (const auto& [key, entry] : voxels_) {
    result.push_back(entry.point);
  }
  return result;
}

int VoxelPointCloud::Size() const noexcept {
  return static_cast<int>(voxels_.size());
}

void VoxelPointCloud::Clear() {
  voxels_.clear();
  timestamp_index_.clear();
}

int64_t VoxelPointCloud::ComputeVoxelKey(const Eigen::Vector3d& p) const noexcept {
  // 각 축을 21비트 정수로 인코딩 (±1m 범위에서 2mm 해상도 → ±500 → 10비트 충분)
  const double inv_res = 1.0 / config_.voxel_resolution_m;
  const auto ix = static_cast<int64_t>(std::floor(p.x() * inv_res));
  const auto iy = static_cast<int64_t>(std::floor(p.y() * inv_res));
  const auto iz = static_cast<int64_t>(std::floor(p.z() * inv_res));

  // 21비트 마스크로 제한 후 3개를 63비트에 패킹
  constexpr int64_t kMask = (1LL << 21) - 1;
  return ((ix & kMask) << 42) | ((iy & kMask) << 21) | (iz & kMask);
}

}  // namespace shape_estimation
