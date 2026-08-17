// ── PinocchioModelBuilder: YAML + URDF → Pinocchio 모델 구축 ────────────────
#pragma once

#include "rtc_urdf_bridge/closure_yaml_loader.hpp"
#include "rtc_urdf_bridge/inertial_validation.hpp"
#include "rtc_urdf_bridge/kinematic_chain_extractor.hpp"
#include "rtc_urdf_bridge/types.hpp"
#include "rtc_urdf_bridge/urdf_analyzer.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rtc_urdf_bridge {

/// YAML 설정 + URDF 분석 결과로 Pinocchio 모델 빌드.
///
/// - 전체(full) 모델, 축소(reduced) 서브모델, 트리모델 구축
/// - 폐쇄 체인 RigidConstraintModel 등록
/// - Builder가 모든 Model의 소유권 보유 (shared_ptr)
/// - 외부에는 shared_ptr<const Model> 참조 제공
///
/// 로봇 비종속: URDF 파일 + YAML 설정에서 모든 정보 결정.
class PinocchioModelBuilder {
 public:
  /// @brief YAML 파일 경로로 생성 (ModelConfig 자동 로드)
  /// @throws std::runtime_error 파일/파싱/빌드 실패 시
  explicit PinocchioModelBuilder(std::string_view yaml_config_path);

  /// @brief ModelConfig 직접 전달
  /// @throws std::runtime_error 빌드 실패 시
  explicit PinocchioModelBuilder(const ModelConfig& config);

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

  /// 폐쇄 체인(extended URDF) 제어용 actuated 모델. loop-passive 관절(sidecar 가
  /// passive 로 규정한 것)만 잠그고 actuated 관절을 전부 movable 로 유지한 축소 모델
  /// 이라 nq==nv 이며 device 가 선언한 actuated 집합과 1:1 대응한다. tree/reduced 모델
  /// 과 달리 root→tip 직렬 경로 밖의 actuated 관절(예: 4-bar 손가락의 active DIP —
  /// passive coupler 가지로 tip 에 도달)도 보존한다. lock 기준값은 tree/reduced 와 동일
  /// 한 MakeReferenceConfig() 이므로 frozen passive 기하가 tree 모델과 일치한다.
  /// @return closure sidecar 미사용(plain/mimic URDF)이면 nullptr — 소비자는 기존
  ///   tree/full 경로로 fallback (byte-for-byte).
  [[nodiscard]] std::shared_ptr<const pinocchio::Model> GetActuatedModel() const noexcept;

  /// 폐쇄 체인 구속 조건 모델 목록
  [[nodiscard]] const std::vector<pinocchio::RigidConstraintModel>& GetConstraintModels()
      const noexcept;

  // ── Extended-URDF closure (sidecar) 결과 ───────────────────────────────────
  // closure_yaml_path 가 설정된 경우에만 채워진다. plain URDF 면 빈/기본값.

  /// loop-consistent 기준 설정값 q_ref (full model, sidecar projection 통과).
  /// closure sidecar 미사용 시 빈 벡터. @see IsClosureReferenceSingular,
  /// IsClosureReferenceConverged
  [[nodiscard]] const Eigen::VectorXd& GetClosureReferenceConfig() const noexcept;

  /// sidecar actuation.actuated_joints → full model JointIndex 목록.
  /// closure sidecar 미사용 시 빈 목록.
  [[nodiscard]] const std::vector<pinocchio::JointIndex>& GetClosureActuatedJointIds()
      const noexcept;

  /// sidecar 가 passive 로 규정한 full-model movable 관절 이름 (= movable − actuated).
  /// 모든 reduced/tree 서브모델에서 잠기는 loop-passive 관절이다 — topology 판정
  /// (프레임이 loop-passive 하류인가) 에 쓴다. closure sidecar 미사용 시 빈 목록.
  [[nodiscard]] const std::vector<std::string>& GetClosurePassiveLockNames() const noexcept;

  /// q_ref projection strict 수렴 (‖φ‖ < strict tolerance) 여부 (singular 여부와 별개 —
  /// full-rank 여도 미수렴일 수 있다). closure sidecar 미사용 시 false.
  /// ⚠ 소비 가능 판정은 @ref IsClosureReferenceAcceptable 이 담당한다 — URDF 좌표 불일치의
  /// residual floor 는 strict 미달이어도 q_ref 로 사용 가능하다 (#250).
  [[nodiscard]] bool IsClosureReferenceConverged() const noexcept;

  /// q_ref 를 소비해도 되는가 (‖φ‖ ≤ acceptance tolerance 1e-6 m, 유한). converged=true 면
  /// 항상 true. false 면 q_ref 를 정상 결과로 쓰지 말 것. closure sidecar 미사용 시 false.
  [[nodiscard]] bool IsClosureReferenceAcceptable() const noexcept;

  /// q_ref 가 rank 결손(특이 조립형상) 인지. true 면 이 형상에서 constraintDynamics 의
  /// KKT 가 특이해져 NaN 을 낼 수 있으므로 operating configuration 으로 바로 쓰면 안 된다.
  [[nodiscard]] bool IsClosureReferenceSingular() const noexcept;

  // ── 관성 게이트 (V5 / V6) 결과 ─────────────────────────────────────────────
  // V6 (물리적으로 불가능) 는 생성자에서 이미 throw 했으므로 여기 도달했다면
  // `fatal` 은 항상 비어 있다. 남는 정보는 V5 레인이다.

  /// full 모델 관성 판정 결과. 순서는 Pinocchio 관절 인덱스 오름차순 = 결정적.
  [[nodiscard]] const InertialValidationReport& GetInertialReport() const noexcept;

  /// full 모델이 동역학 소비에 적합한가 (= 질량을 지지 않는 movable body 가 없는가).
  /// false 여도 모델은 정상 로드된 상태이며 FK / Jacobian / IK 는 유효하다 —
  /// M(q) · Coriolis · 중력 등 **관성에 의존하는 소비자만** 이 술어를 봐야 한다.
  [[nodiscard]] bool IsFullModelDynamicsCapable() const noexcept;

  // ── 메타데이터 ─────────────────────────────────────────────────────────────

  /// 등록된 서브모델 이름 목록
  [[nodiscard]] std::vector<std::string> GetSubModelNames() const;

  /// 등록된 트리모델 이름 목록
  [[nodiscard]] std::vector<std::string> GetTreeModelNames() const;

  /// 서브모델 정의 조회
  [[nodiscard]] const SubModelDefinition& GetSubModelDefinition(std::string_view name) const;

  /// 트리모델 정의 조회
  [[nodiscard]] const TreeModelDefinition& GetTreeModelDefinition(std::string_view name) const;

  /// UrdfAnalyzer 접근
  [[nodiscard]] const UrdfAnalyzer& GetAnalyzer() const noexcept;

  /// KinematicChainExtractor 접근
  [[nodiscard]] const KinematicChainExtractor& GetExtractor() const noexcept;

  /// ModelConfig 접근
  [[nodiscard]] const ModelConfig& GetConfig() const noexcept;

  // ── YAML 로드 유틸리티 (static) ────────────────────────────────────────────

  /// YAML 파일 경로 → ModelConfig
  [[nodiscard]] static ModelConfig LoadModelConfig(std::string_view yaml_path);

 private:
  void Build();
  void BuildFullModel();
  void BuildReducedModels();
  void BuildTreeModels();
  /// Extended-URDF closure 시 actuated 제어 모델 구축 (loop-passive 만 잠금 → nq==nv).
  /// closure_passive_lock_names_ 가 비면 no-op → actuated_model_ 는 nullptr 유지.
  /// LoadClosureSpecAndComputePassiveLocks() 이후에 호출되어야 한다.
  void BuildActuatedModel();
  void RegisterClosedChainConstraints();

  /// full 모델의 composite 관성을 판정해 inertial_report_ 를 채운다.
  /// V6 위반이 하나라도 있으면 std::runtime_error 로 로드를 실패시킨다.
  /// BuildFullModel() 끝에서 호출된다 — 축소·트리 모델은 이 full 모델에서
  /// 파생되므로 여기가 유일한 관문이다.
  void ValidateFullModelInertias();

  /// Extended-URDF sidecar 를 1회 로드해 closure_spec_ 에 캐시하고, loop-passive
  /// 관절 이름 (= full model 의 movable 관절 − sidecar actuated_joints) 을
  /// closure_passive_lock_names_ 에 계산한다. BuildFullModel() 직후·
  /// BuildReducedModels() 전에 호출되어야 한다 (spanning-tree analyzer 는 loop 이
  /// 없어 loop-passive 를 분류하지 못하므로 sidecar 가 유일한 출처).
  /// closure_yaml_path 가 비어 있으면 no-op.
  void LoadClosureSpecAndComputePassiveLocks();

  /// 축소 모델에서 잠금 대상이 될 passive 관절 이름 목록.
  ///   - yaml_passive_override == false: analyzer의 자동 분류 결과 사용
  ///   - yaml_passive_override == true : config_.passive_joints 그대로 사용
  /// 두 경우 모두 closure_passive_lock_names_ (Extended-URDF loop-passive) 를 합집합.
  [[nodiscard]] std::vector<std::string> CollectPassiveLockNames() const;

  /// 잠금 관절의 기준 설정값 구성
  [[nodiscard]] Eigen::VectorXd MakeReferenceConfig() const;

  /// 관절 이름 목록 → Pinocchio JointIndex 목록 (full model 기준)
  [[nodiscard]] std::vector<pinocchio::JointIndex> ResolveJointIndicesToLock(
      const std::vector<std::string>& joint_names_to_lock) const;

  // ── 내부 데이터 ────────────────────────────────────────────────────────────
  ModelConfig config_;
  std::unique_ptr<UrdfAnalyzer> analyzer_;
  std::unique_ptr<KinematicChainExtractor> extractor_;

  // 모델 저장소 (shared_ptr: Handle이 참조)
  std::shared_ptr<pinocchio::Model> full_model_;
  // 관성 게이트 판정 결과 (full 모델 기준). BuildFullModel() 에서 1회 채워진다.
  InertialValidationReport inertial_report_;
  std::unordered_map<std::string, std::shared_ptr<pinocchio::Model>> reduced_models_;
  std::unordered_map<std::string, std::shared_ptr<pinocchio::Model>> tree_models_;
  // 폐쇄 체인 actuated 제어 모델 (loop-passive 만 잠금, nq==nv). plain URDF 면 nullptr.
  std::shared_ptr<pinocchio::Model> actuated_model_;

  // 서브모델/트리모델 정의 캐시
  std::unordered_map<std::string, SubModelDefinition> sub_model_defs_;
  std::unordered_map<std::string, TreeModelDefinition> tree_model_defs_;

  // 폐쇄 체인 구속
  std::vector<pinocchio::RigidConstraintModel> constraint_models_;

  // Extended-URDF sidecar spec 캐시 (closure_yaml_path 설정 시에만 값 보유).
  // BuildFullModel() 후 1회 로드되어 BuildReducedModels()/BuildTreeModels() 의
  // passive-lock 계산과 RegisterClosedChainConstraints() 의 constraint 생성이
  // 이 단일 캐시를 공유한다 (중복 LoadClosureYaml 제거).
  std::optional<ClosureSpec> closure_spec_;
  // sidecar 가 passive 로 규정한 full-model movable 관절 (= movable − actuated).
  // 모든 reduced/tree 서브모델에서 잠긴다. plain URDF 면 빈 목록.
  std::vector<std::string> closure_passive_lock_names_;

  // Extended-URDF sidecar closure 결과 (closure_yaml_path 설정 시에만 채워짐)
  Eigen::VectorXd closure_q_ref_;
  std::vector<pinocchio::JointIndex> closure_actuated_joint_ids_;
  bool closure_q_ref_converged_{false};
  bool closure_q_ref_acceptable_{false};
  bool closure_q_ref_singular_{false};
};

}  // namespace rtc_urdf_bridge
