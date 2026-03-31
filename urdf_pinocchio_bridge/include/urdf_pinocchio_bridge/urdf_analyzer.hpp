// ── UrdfAnalyzer: URDF 파싱 + kinematic 토폴로지 분석 ────────────────────────
#pragma once

#include "urdf_pinocchio_bridge/types.hpp"

#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace urdf_pinocchio_bridge
{

/// 임의의 URDF XML을 파싱하여 링크-관절 인접 그래프를 구축하고,
/// 패시브/마이믹 관절, 폐쇄 체인을 자동 감지하는 분석 엔진.
///
/// 로봇 비종속: 하드코딩된 link/joint 이름 없음.
/// 모든 정보는 URDF XML에서 런타임에 추출됨.
class UrdfAnalyzer
{
public:
  /// XML 문자열 생성자 오버로드 구분용 태그
  struct FromXmlTag {};

  /// @brief URDF 파일 경로로 생성
  /// @throws std::runtime_error URDF 파일 읽기/파싱 실패 시
  explicit UrdfAnalyzer(std::string_view urdf_file_path);

  /// @brief URDF XML 문자열로 생성
  /// @throws std::runtime_error XML 파싱 실패 시
  UrdfAnalyzer(std::string_view xml_string, FromXmlTag);

  // ── 그래프 조회 ────────────────────────────────────────────────────────────

  /// 전체 링크 노드 (인접 리스트)
  [[nodiscard]] const std::vector<LinkNode> & GetLinkNodes() const noexcept;

  /// 링크 이름 → 인덱스. 없으면 std::out_of_range throw.
  [[nodiscard]] int GetLinkIndex(std::string_view link_name) const;

  /// 루트 링크 인덱스 (parent가 없는 유일한 링크)
  [[nodiscard]] int GetRootIndex() const noexcept;

  /// 루트 링크 이름
  [[nodiscard]] const std::string & GetRootLinkName() const noexcept;

  /// 전체 링크 개수
  [[nodiscard]] std::size_t GetNumLinks() const noexcept;

  /// 전체 관절 개수 (fixed 포함)
  [[nodiscard]] std::size_t GetNumJoints() const noexcept;

  // ── 관절 분류 ──────────────────────────────────────────────────────────────

  /// 모든 actuated (revolute/continuous/prismatic) 관절 이름
  [[nodiscard]] const std::vector<std::string> & GetActuatedJointNames() const noexcept;

  /// 모든 fixed 관절 이름
  [[nodiscard]] const std::vector<std::string> & GetFixedJointNames() const noexcept;

  /// 패시브 관절 목록 (transmission 없는 non-fixed 관절)
  [[nodiscard]] const std::vector<PassiveJointInfo> & GetPassiveJoints() const noexcept;

  /// 마이믹 관절 목록
  [[nodiscard]] const std::vector<MimicJointInfo> & GetMimicJoints() const noexcept;

  /// 관절 이름 → 타입
  [[nodiscard]] UrdfJointType GetJointType(std::string_view joint_name) const;

  /// 관절 이름 → 메타데이터
  [[nodiscard]] const JointMeta & GetJointMeta(std::string_view joint_name) const;

  /// 관절 이름 → 자식 링크 이름
  [[nodiscard]] const std::string & GetJointChildLink(std::string_view joint_name) const;

  /// 관절 이름 → 부모 링크 이름
  [[nodiscard]] const std::string & GetJointParentLink(std::string_view joint_name) const;

  // ── 폐쇄 체인 감지 ────────────────────────────────────────────────────────

  /// URDF 내 커스텀 <loop_joint> 태그 + heuristic 감지
  [[nodiscard]] std::vector<ClosedChainInfo> DetectClosedChains() const;

  // ── 경로 탐색 ──────────────────────────────────────────────────────────────

  /// link_a → link_b 트리 경로 (링크 인덱스 시퀀스). 경로 없으면 빈 벡터.
  [[nodiscard]] std::vector<int> FindPath(
    std::string_view link_a, std::string_view link_b) const;

  /// 두 링크의 최소 공통 조상 (LCA) 링크 인덱스
  [[nodiscard]] int FindLCA(std::string_view link_a, std::string_view link_b) const;

  // ── URDF 원본 접근 ────────────────────────────────────────────────────────

  /// 전체 URDF XML 문자열
  [[nodiscard]] const std::string & GetUrdfXmlString() const noexcept;

  /// URDF 파일 경로 (파일에서 로드한 경우)
  [[nodiscard]] const std::string & GetUrdfFilePath() const noexcept;

private:
  // 파싱 파이프라인
  void ParseUrdfXml(const std::string & xml);
  void BuildAdjacencyGraph();
  void ClassifyJoints();
  void DetectMimicJoints();
  void DetectPassiveJoints();
  void ComputeDepths();

  // ── 내부 데이터 ────────────────────────────────────────────────────────────
  std::string urdf_file_path_;
  std::string urdf_xml_string_;

  // 인접 리스트 (인덱스 기반 트리)
  std::vector<LinkNode> link_nodes_;
  std::unordered_map<std::string, int> link_name_to_index_;
  int root_index_{-1};

  // 관절 메타데이터 (이름 순서)
  std::unordered_map<std::string, JointMeta> joint_meta_map_;
  std::vector<std::string> all_joint_names_;

  // 관절 분류
  std::vector<std::string> actuated_joint_names_;
  std::vector<std::string> fixed_joint_names_;
  std::vector<PassiveJointInfo> passive_joints_;
  std::vector<MimicJointInfo> mimic_joints_;

  // transmission 태그에 등장하는 관절 이름
  std::unordered_set<std::string> transmitted_joints_;

  // LCA용 깊이 테이블 (BFS 1회로 계산)
  std::vector<int> depth_;
};

}  // namespace urdf_pinocchio_bridge
