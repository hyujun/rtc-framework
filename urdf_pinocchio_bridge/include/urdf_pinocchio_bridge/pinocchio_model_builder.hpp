// ── PinocchioModelBuilder: YAML + URDF → Pinocchio 모델 구축 ────────────────
#pragma once

#include "urdf_pinocchio_bridge/types.hpp"
#include "urdf_pinocchio_bridge/urdf_analyzer.hpp"
#include "urdf_pinocchio_bridge/kinematic_chain_extractor.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/contact-info.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace urdf_pinocchio_bridge
{

/// YAML 설정 + URDF 분석 결과로 Pinocchio 모델 빌드.
///
/// - 전체(full) 모델, 축소(reduced) 서브모델, 트리모델 구축
/// - 폐쇄 체인 RigidConstraintModel 등록
/// - Builder가 모든 Model의 소유권 보유 (shared_ptr)
/// - 외부에는 shared_ptr<const Model> 참조 제공
///
/// 로봇 비종속: URDF 파일 + YAML 설정에서 모든 정보 결정.
class PinocchioModelBuilder
{
public:
  /// @brief YAML 파일 경로로 생성 (ModelConfig 자동 로드)
  /// @throws std::runtime_error 파일/파싱/빌드 실패 시
  explicit PinocchioModelBuilder(std::string_view yaml_config_path);

  /// @brief ModelConfig 직접 전달
  /// @throws std::runtime_error 빌드 실패 시
  explicit PinocchioModelBuilder(const ModelConfig & config);

  // ── 모델 접근 ──────────────────────────────────────────────────────────────

  /// 전체(full) Pinocchio 모델
  [[nodiscard]] std::shared_ptr<const pinocchio::Model> GetFullModel() const noexcept;

  /// 이름으로 축소(reduced) 서브모델 조회
  /// @throws std::out_of_range 이름 미등록 시
  [[nodiscard]] std::shared_ptr<const pinocchio::Model> GetReducedModel(
    std::string_view sub_model_name) const;

  /// 이름으로 트리모델 조회
  /// @throws std::out_of_range 이름 미등록 시
  [[nodiscard]] std::shared_ptr<const pinocchio::Model> GetTreeModel(
    std::string_view tree_model_name) const;

  /// 폐쇄 체인 구속 조건 모델 목록
  [[nodiscard]] const std::vector<pinocchio::RigidConstraintModel> &
    GetConstraintModels() const noexcept;

  // ── 메타데이터 ─────────────────────────────────────────────────────────────

  /// 등록된 서브모델 이름 목록
  [[nodiscard]] std::vector<std::string> GetSubModelNames() const;

  /// 등록된 트리모델 이름 목록
  [[nodiscard]] std::vector<std::string> GetTreeModelNames() const;

  /// 서브모델 정의 조회
  [[nodiscard]] const SubModelDefinition & GetSubModelDefinition(
    std::string_view name) const;

  /// 트리모델 정의 조회
  [[nodiscard]] const TreeModelDefinition & GetTreeModelDefinition(
    std::string_view name) const;

  /// UrdfAnalyzer 접근
  [[nodiscard]] const UrdfAnalyzer & GetAnalyzer() const noexcept;

  /// KinematicChainExtractor 접근
  [[nodiscard]] const KinematicChainExtractor & GetExtractor() const noexcept;

  /// ModelConfig 접근
  [[nodiscard]] const ModelConfig & GetConfig() const noexcept;

  // ── YAML 로드 유틸리티 (static) ────────────────────────────────────────────

  /// YAML 파일 경로 → ModelConfig
  [[nodiscard]] static ModelConfig LoadModelConfig(std::string_view yaml_path);

private:
  void Build();
  void BuildFullModel();
  void BuildReducedModels();
  void BuildTreeModels();
  void RegisterClosedChainConstraints();

  /// mimic 관절을 passive 목록에 추가 (잠금 대상)
  void IncorporateMimicAsPassive();

  /// 잠금 관절의 기준 설정값 구성
  [[nodiscard]] Eigen::VectorXd MakeReferenceConfig() const;

  /// 관절 이름 목록 → Pinocchio JointIndex 목록 (full model 기준)
  [[nodiscard]] std::vector<pinocchio::JointIndex> ResolveJointIndicesToLock(
    const std::vector<std::string> & joint_names_to_lock) const;

  // ── 내부 데이터 ────────────────────────────────────────────────────────────
  ModelConfig config_;
  std::unique_ptr<UrdfAnalyzer> analyzer_;
  std::unique_ptr<KinematicChainExtractor> extractor_;

  // 모델 저장소 (shared_ptr: Handle이 참조)
  std::shared_ptr<pinocchio::Model> full_model_;
  std::unordered_map<std::string, std::shared_ptr<pinocchio::Model>> reduced_models_;
  std::unordered_map<std::string, std::shared_ptr<pinocchio::Model>> tree_models_;

  // 서브모델/트리모델 정의 캐시
  std::unordered_map<std::string, SubModelDefinition> sub_model_defs_;
  std::unordered_map<std::string, TreeModelDefinition> tree_model_defs_;

  // 폐쇄 체인 구속
  std::vector<pinocchio::RigidConstraintModel> constraint_models_;
};

}  // namespace urdf_pinocchio_bridge
