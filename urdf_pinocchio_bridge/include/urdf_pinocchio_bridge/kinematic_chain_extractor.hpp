// ── KinematicChainExtractor: 서브모델/트리모델 체인 추출 ─────────────────────
#pragma once

#include "urdf_pinocchio_bridge/types.hpp"
#include "urdf_pinocchio_bridge/urdf_analyzer.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace urdf_pinocchio_bridge
{

/// 분석된 URDF 트리에서 서브모델(root→tip 단일 체인) 및
/// 트리모델(root에서 여러 tip으로 분기) 체인을 추출.
///
/// 로봇 비종속: 링크/관절 이름은 모두 인자로 전달받음.
class KinematicChainExtractor
{
public:
  /// @param analyzer 분석 완료된 UrdfAnalyzer (수명: 이 객체보다 길어야 함)
  explicit KinematicChainExtractor(const UrdfAnalyzer & analyzer);

  /// root_link → tip_link 단일 직렬 체인 추출
  /// @throws std::runtime_error 경로가 없거나 링크 이름 무효 시
  [[nodiscard]] SubModelDefinition ExtractSubModel(
    std::string_view name,
    std::string_view root_link,
    std::string_view tip_link) const;

  /// 하나의 root에서 여러 tip으로의 트리 추출 (경로 union)
  /// @throws std::runtime_error 링크 이름 무효 시
  [[nodiscard]] TreeModelDefinition ExtractTreeModel(
    std::string_view name,
    std::string_view root_link,
    const std::vector<std::string> & tip_links) const;

  /// 전체 actuated 관절 중, 주어진 서브모델에 포함되지 않는 관절 이름 반환
  /// (buildReducedModel에 전달할 joints_to_lock 계산용)
  [[nodiscard]] std::vector<std::string> ComputeJointsToLock(
    const SubModelDefinition & sub_model) const;

  /// 트리모델 버전
  [[nodiscard]] std::vector<std::string> ComputeJointsToLock(
    const TreeModelDefinition & tree_model) const;

private:
  const UrdfAnalyzer & analyzer_;

  /// 인접 경로에서 관절 이름 수집 (child 기준 parent_joint)
  void CollectJointsOnPath(
    const std::vector<int> & link_path,
    std::vector<std::string> & actuated_out,
    std::vector<std::string> & all_out,
    std::vector<std::string> & links_out) const;

  /// 관절 목록에서 중복 제거 (순서 보존)
  static void DeduplicatePreserveOrder(std::vector<std::string> & vec);
};

}  // namespace urdf_pinocchio_bridge
