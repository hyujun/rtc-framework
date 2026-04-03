// ── KinematicChainExtractor 구현 ─────────────────────────────────────────────
#include "rtc_urdf_bridge/kinematic_chain_extractor.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

namespace rtc_urdf_bridge
{

KinematicChainExtractor::KinematicChainExtractor(const UrdfAnalyzer & analyzer)
: analyzer_(analyzer)
{
}

// ── 단일 체인 추출 (root → tip) ─────────────────────────────────────────────
SubModelDefinition KinematicChainExtractor::ExtractSubModel(
  std::string_view name,
  std::string_view root_link,
  std::string_view tip_link) const
{
  // 링크 존재 확인 (없으면 GetLinkIndex에서 throw)
  (void)analyzer_.GetLinkIndex(root_link);
  (void)analyzer_.GetLinkIndex(tip_link);

  auto link_path = analyzer_.FindPath(root_link, tip_link);
  if (link_path.empty()) {
    throw std::runtime_error(
      "KinematicChainExtractor: " + std::string(root_link) +
      " → " + std::string(tip_link) + " 경로를 찾을 수 없습니다");
  }

  SubModelDefinition def;
  def.name = name;
  def.root_link = root_link;
  def.tip_link = tip_link;

  CollectJointsOnPath(link_path, def.joint_names, def.all_joint_names, def.link_names);
  return def;
}

// ── 트리 모델 추출 (root에서 여러 tip으로 분기) ─────────────────────────────
TreeModelDefinition KinematicChainExtractor::ExtractTreeModel(
  std::string_view name,
  std::string_view root_link,
  const std::vector<std::string> & tip_links) const
{
  (void)analyzer_.GetLinkIndex(root_link);

  TreeModelDefinition def;
  def.name = name;
  def.root_link = root_link;
  def.tip_links = tip_links;

  // 각 tip까지의 경로 union
  std::unordered_set<int> all_link_indices;
  std::vector<std::vector<int>> all_paths;

  for (const auto & tip : tip_links) {
    auto path = analyzer_.FindPath(root_link, tip);
    if (path.empty()) {
      throw std::runtime_error(
        "KinematicChainExtractor: " + std::string(root_link) +
        " → " + tip + " 경로를 찾을 수 없습니다");
    }
    for (int idx : path) {
      all_link_indices.insert(idx);
    }
    all_paths.push_back(std::move(path));
  }

  // 모든 경로에서 관절/링크 수집
  for (const auto & path : all_paths) {
    CollectJointsOnPath(path, def.joint_names, def.all_joint_names, def.link_names);
  }

  // 중복 제거
  DeduplicatePreserveOrder(def.joint_names);
  DeduplicatePreserveOrder(def.all_joint_names);
  DeduplicatePreserveOrder(def.link_names);

  // 분기점 감지: 2개 이상의 자식이 트리에 포함된 링크
  const auto & nodes = analyzer_.GetLinkNodes();
  for (int idx : all_link_indices) {
    const auto & node = nodes[static_cast<std::size_t>(idx)];
    int children_in_tree = 0;
    for (int child : node.child_indices) {
      if (all_link_indices.count(child) > 0) {
        ++children_in_tree;
      }
    }
    if (children_in_tree >= 2) {
      def.branching_points.push_back(node.link_name);
    }
  }

  return def;
}

// ── 잠글 관절 계산 (서브모델) ───────────────────────────────────────────────
std::vector<std::string> KinematicChainExtractor::ComputeJointsToLock(
  const SubModelDefinition & sub_model) const
{
  const auto & all = analyzer_.GetActuatedJointNames();
  std::unordered_set<std::string> keep(
    sub_model.joint_names.begin(), sub_model.joint_names.end());

  std::vector<std::string> to_lock;
  for (const auto & j : all) {
    if (keep.count(j) == 0) {
      to_lock.push_back(j);
    }
  }
  return to_lock;
}

// ── 잠글 관절 계산 (트리모델) ───────────────────────────────────────────────
std::vector<std::string> KinematicChainExtractor::ComputeJointsToLock(
  const TreeModelDefinition & tree_model) const
{
  const auto & all = analyzer_.GetActuatedJointNames();
  std::unordered_set<std::string> keep(
    tree_model.joint_names.begin(), tree_model.joint_names.end());

  std::vector<std::string> to_lock;
  for (const auto & j : all) {
    if (keep.count(j) == 0) {
      to_lock.push_back(j);
    }
  }
  return to_lock;
}

// ── 경로에서 관절/링크 이름 수집 ────────────────────────────────────────────
void KinematicChainExtractor::CollectJointsOnPath(
  const std::vector<int> & link_path,
  std::vector<std::string> & actuated_out,
  std::vector<std::string> & all_out,
  std::vector<std::string> & links_out) const
{
  const auto & nodes = analyzer_.GetLinkNodes();

  for (std::size_t i = 0; i < link_path.size(); ++i) {
    int idx = link_path[i];
    const auto & node = nodes[static_cast<std::size_t>(idx)];
    links_out.push_back(node.link_name);

    // 인접한 두 노드 사이의 관절을 찾기:
    // 현재 노드의 parent_joint을 사용하되, 경로 내 이전 노드가 parent인 경우만
    if (i > 0) {
      int prev = link_path[i - 1];

      // case 1: prev가 현재 노드의 parent → 현재 노드의 parent_joint 사용
      if (node.parent_index == prev && !node.parent_joint_name.empty()) {
        all_out.push_back(node.parent_joint_name);
        if (node.parent_joint_type != UrdfJointType::kFixed) {
          actuated_out.push_back(node.parent_joint_name);
        }
      }
      // case 2: 현재 노드가 prev의 parent → prev의 parent_joint 사용
      else {
        const auto & prev_node = nodes[static_cast<std::size_t>(prev)];
        if (prev_node.parent_index == idx && !prev_node.parent_joint_name.empty()) {
          all_out.push_back(prev_node.parent_joint_name);
          if (prev_node.parent_joint_type != UrdfJointType::kFixed) {
            actuated_out.push_back(prev_node.parent_joint_name);
          }
        }
      }
    }
  }
}

// ── 중복 제거 (순서 보존) ───────────────────────────────────────────────────
void KinematicChainExtractor::DeduplicatePreserveOrder(std::vector<std::string> & vec)
{
  std::unordered_set<std::string> seen;
  auto end = std::remove_if(vec.begin(), vec.end(),
    [&seen](const std::string & s) {
      return !seen.insert(s).second;
    });
  vec.erase(end, vec.end());
}

}  // namespace rtc_urdf_bridge
