// ── PinocchioModelBuilder 구현 ───────────────────────────────────────────────
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/closure_yaml_loader.hpp"
#include "rtc_urdf_bridge/constraint_builder.hpp"
#include "rtc_urdf_bridge/urdf_logging.hpp"
#include "rtc_urdf_bridge/xacro_processor.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rtc_urdf_bridge {

namespace {
auto logger() {
  return ::rtc::urdf::logging::BuilderLogger();
}

// Resolve a possibly-relative resource path against the directory of the YAML
// file that declared it. Absolute paths (leading '/') and empty strings pass
// through unchanged.
std::string ResolveRelativeToYaml(std::string path, std::string_view yaml_path) {
  if (path.empty() || path[0] == '/') {
    return path;
  }
  const std::filesystem::path yaml_dir =
      std::filesystem::path(std::string(yaml_path)).parent_path();
  return (yaml_dir / path).string();
}
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 생성자
// ═══════════════════════════════════════════════════════════════════════════════

PinocchioModelBuilder::PinocchioModelBuilder(std::string_view yaml_config_path)
    : config_(LoadModelConfig(yaml_config_path)) {
  Build();
}

PinocchioModelBuilder::PinocchioModelBuilder(const ModelConfig& config) : config_(config) {
  Build();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Build 파이프라인
// ═══════════════════════════════════════════════════════════════════════════════

void PinocchioModelBuilder::Build() {
  // (0) xacro 파일이면 전처리 → XML 문자열로 변환
  if (!config_.urdf_path.empty() && IsXacroFile(config_.urdf_path)) {
    config_.urdf_xml_string = ProcessXacro(config_.urdf_path, config_.xacro_args);
  }

  // (1) URDF 분석 — YAML passive_joints를 Analyzer에 hint로 전달
  std::vector<std::string> passive_hints = config_.passive_joints;
  if (!config_.urdf_xml_string.empty()) {
    analyzer_ = std::make_unique<UrdfAnalyzer>(config_.urdf_xml_string, UrdfAnalyzer::FromXmlTag{},
                                               std::move(passive_hints));
  } else if (!config_.urdf_path.empty()) {
    analyzer_ = std::make_unique<UrdfAnalyzer>(config_.urdf_path, std::move(passive_hints));
  } else {
    RCLCPP_ERROR(logger(), "urdf_path 또는 urdf_xml_string 필요");
    throw std::runtime_error("PinocchioModelBuilder: urdf_path 또는 urdf_xml_string 필요");
  }

  extractor_ = std::make_unique<KinematicChainExtractor>(*analyzer_);

  // (2) Pinocchio 모델 구축
  //     mimic / closed-chain / hint 기반 passive 분류는 Analyzer가 완료.
  //     reduced 모델 lock 대상은 analyzer_->GetPassiveJointNames() 사용.
  BuildFullModel();
  // Extended-URDF sidecar 는 full 모델이 있어야 loop-passive 를 계산할 수 있고,
  // 그 결과가 reduced/tree 서브모델 잠금에 필요하므로 BuildReducedModels() 전에 로드.
  LoadClosureSpecAndComputePassiveLocks();
  BuildReducedModels();
  BuildTreeModels();
  BuildActuatedModel();
  RegisterClosedChainConstraints();
}

// ── Extended-URDF closure spec 로드 + loop-passive 계산 ─────────────────────
// spanning-tree URDF 는 루프가 없어 analyzer 가 loop-passive 를 분류하지 못한다.
// sidecar 의 계약은 "actuated_joints 외 모든 non-fixed movable 관절 = passive" 이므로
// full model 의 movable 관절 집합에서 actuated_joints 를 뺀 나머지가 loop-passive 다
// (fixed 관절은 이미 Pinocchio 모델에서 흡수되어 names 에 없으므로 자동 제외).
void PinocchioModelBuilder::LoadClosureSpecAndComputePassiveLocks() {
  if (config_.closure_yaml_path.empty()) {
    return;
  }
  closure_spec_ = LoadClosureYaml(config_.closure_yaml_path);

  const auto& actuated = closure_spec_->actuated_joints;
  // index 0 은 universe (root) 관절 — movable 아님, 건너뛴다.
  for (std::size_t jid = 1; jid < full_model_->names.size(); ++jid) {
    const std::string& jname = full_model_->names[jid];
    if (std::find(actuated.begin(), actuated.end(), jname) != actuated.end()) {
      continue;
    }
    closure_passive_lock_names_.push_back(jname);
  }

  // locked-count 를 loud 하게 노출 — sidecar actuated_joints 가 불완전하면
  // 컨트롤러 active DoF 가 조용히 바뀌므로 (risk/contract) 잠긴 수를 항상 로깅.
  RCLCPP_INFO(logger(),
              "Extended-URDF closure passive-lock: %zu joint 잠금 "
              "(movable %zu − actuated %zu), sidecar='%s'",
              closure_passive_lock_names_.size(), full_model_->names.size() - 1, actuated.size(),
              config_.closure_yaml_path.c_str());
}

// ── 전체 모델 구축 ──────────────────────────────────────────────────────────
void PinocchioModelBuilder::BuildFullModel() {
  full_model_ = std::make_shared<pinocchio::Model>();
  RCLCPP_DEBUG(logger(), "Pinocchio full 모델 구축 시작 (root_joint_type=%s)",
               config_.root_joint_type.c_str());

  // XML 문자열 우선 (xacro 전처리 결과 포함)
  const bool use_xml = !config_.urdf_xml_string.empty();

  if (config_.root_joint_type == "floating") {
    // 플로팅 베이스 (모바일/휴머노이드)
    if (use_xml) {
      pinocchio::urdf::buildModelFromXML(config_.urdf_xml_string, pinocchio::JointModelFreeFlyer(),
                                         *full_model_);
    } else {
      pinocchio::urdf::buildModel(config_.urdf_path, pinocchio::JointModelFreeFlyer(),
                                  *full_model_);
    }
  } else {
    // 고정 베이스 (기본)
    if (use_xml) {
      pinocchio::urdf::buildModelFromXML(config_.urdf_xml_string, *full_model_);
    } else {
      pinocchio::urdf::buildModel(config_.urdf_path, *full_model_);
    }
  }

  RCLCPP_INFO(logger(), "Full 모델 구축 완료: nq=%d, nv=%d, njoints=%d, nframes=%d",
              full_model_->nq, full_model_->nv, full_model_->njoints, full_model_->nframes);
}

// ── 축소 모델에 공통으로 적용될 passive 잠금 대상 ──────────────────────────
// Analyzer가 이미 mimic / closed-chain / YAML hint 를 반영하여
// passive_joint_names_를 구성해 놓았다.
//   - yaml_passive_override == false (hint): passive_joints = analyzer.passive
//   - yaml_passive_override == true  (override): passive_joints = config 목록만
std::vector<std::string> PinocchioModelBuilder::CollectPassiveLockNames() const {
  std::vector<std::string> names =
      config_.yaml_passive_override ? config_.passive_joints : analyzer_->GetPassiveJointNames();

  // Extended-URDF loop-passive 를 합집합. spanning-tree analyzer 는 loop 이 없어
  // 이 관절들을 분류하지 못하므로 sidecar 유래 목록을 여기서 더한다 (중복 제거).
  for (const auto& pj : closure_passive_lock_names_) {
    if (std::find(names.begin(), names.end(), pj) == names.end()) {
      names.push_back(pj);
    }
  }
  return names;
}

// ── 축소 모델 구축 ──────────────────────────────────────────────────────────
void PinocchioModelBuilder::BuildReducedModels() {
  auto ref_config = MakeReferenceConfig();
  const auto passive_lock = CollectPassiveLockNames();

  for (const auto& sub_cfg : config_.sub_models) {
    auto def = extractor_->ExtractSubModel(sub_cfg.name, sub_cfg.root_link, sub_cfg.tip_link);

    auto joints_to_lock_names = extractor_->ComputeJointsToLock(def);

    for (const auto& pj : passive_lock) {
      auto it = std::find(joints_to_lock_names.begin(), joints_to_lock_names.end(), pj);
      if (it == joints_to_lock_names.end()) {
        joints_to_lock_names.push_back(pj);
      }
      // 체인 내 passive는 def.joint_names(= active 집합)에서 제거
      auto jit = std::find(def.joint_names.begin(), def.joint_names.end(), pj);
      if (jit != def.joint_names.end()) {
        def.joint_names.erase(jit);
      }
    }

    auto joints_to_lock = ResolveJointIndicesToLock(joints_to_lock_names);

    auto reduced = std::make_shared<pinocchio::Model>();
    pinocchio::buildReducedModel(*full_model_, joints_to_lock, ref_config, *reduced);

    RCLCPP_INFO(logger(),
                "서브모델 등록: '%s' (root='%s', tip='%s', nq=%d, nv=%d, "
                "active=%zu, locked=%zu)",
                sub_cfg.name.c_str(), sub_cfg.root_link.c_str(), sub_cfg.tip_link.c_str(),
                reduced->nq, reduced->nv, def.joint_names.size(), joints_to_lock.size());

    reduced_models_[sub_cfg.name] = std::move(reduced);
    sub_model_defs_[sub_cfg.name] = std::move(def);
  }
}

// ── 트리 모델 구축 ──────────────────────────────────────────────────────────
void PinocchioModelBuilder::BuildTreeModels() {
  auto ref_config = MakeReferenceConfig();
  const auto passive_lock = CollectPassiveLockNames();

  for (const auto& tree_cfg : config_.tree_models) {
    auto def = extractor_->ExtractTreeModel(tree_cfg.name, tree_cfg.root_link, tree_cfg.tip_links);

    auto joints_to_lock_names = extractor_->ComputeJointsToLock(def);

    for (const auto& pj : passive_lock) {
      auto it = std::find(joints_to_lock_names.begin(), joints_to_lock_names.end(), pj);
      if (it == joints_to_lock_names.end()) {
        joints_to_lock_names.push_back(pj);
      }
      auto jit = std::find(def.joint_names.begin(), def.joint_names.end(), pj);
      if (jit != def.joint_names.end()) {
        def.joint_names.erase(jit);
      }
    }

    auto joints_to_lock = ResolveJointIndicesToLock(joints_to_lock_names);

    auto tree_model = std::make_shared<pinocchio::Model>();
    pinocchio::buildReducedModel(*full_model_, joints_to_lock, ref_config, *tree_model);

    RCLCPP_INFO(logger(),
                "트리모델 등록: '%s' (root='%s', tips=%zu, nq=%d, nv=%d, "
                "active=%zu, branching=%zu, locked=%zu)",
                tree_cfg.name.c_str(), tree_cfg.root_link.c_str(), tree_cfg.tip_links.size(),
                tree_model->nq, tree_model->nv, def.joint_names.size(), def.branching_points.size(),
                joints_to_lock.size());

    tree_models_[tree_cfg.name] = std::move(tree_model);
    tree_model_defs_[tree_cfg.name] = std::move(def);
  }
}

// ── Actuated 제어 모델 구축 (폐쇄 체인 전용) ─────────────────────────────────
// tree/reduced 모델은 root→tip 직렬 경로 관절만 유지하므로, 4-bar 손가락의 active
// DIP 처럼 passive coupler 가지를 통해 tip 에 도달하는 actuated 관절은 경로 밖이라
// 잠긴다 (proto_1b: device 16 actuated 중 3 DIP 이 잠겨 tree='wbc' 가 13-DoF). 제어
// (TSID/CLIK/MPC) 모델은 device 가 선언한 actuated 관절을 전부 담아야 reorder 가
// 16/16 통과하므로, 여기서는 sidecar 가 passive 로 규정한 loop 관절만 잠그고
// 나머지(= 모든 actuated)를 유지한 nq==nv 모델을 만든다. lock 기준값은 tree/reduced
// 와 동일한 MakeReferenceConfig() 를 써서 frozen passive 기하가 tree 모델과 일치한다.
void PinocchioModelBuilder::BuildActuatedModel() {
  if (closure_passive_lock_names_.empty()) {
    return;  // plain/mimic URDF — 소비자는 tree/full 로 fallback (byte-for-byte)
  }
  auto ref_config = MakeReferenceConfig();
  auto joints_to_lock = ResolveJointIndicesToLock(closure_passive_lock_names_);

  auto actuated = std::make_shared<pinocchio::Model>();
  pinocchio::buildReducedModel(*full_model_, joints_to_lock, ref_config, *actuated);

  RCLCPP_INFO(logger(),
              "Actuated 제어모델 등록: loop-passive %zu개 잠금, nq=%d nv=%d "
              "(full nq=%d nv=%d)",
              joints_to_lock.size(), actuated->nq, actuated->nv, full_model_->nq, full_model_->nv);

  actuated_model_ = std::move(actuated);
}

// ── 폐쇄 체인 구속 등록 ────────────────────────────────────────────────────
// 실제 RigidConstraintModel 생성은 constraint_builder 로 위임한다 (§4b joint-frame
// placement 합성, frame-first/link+origin 경로 지원, legacy→new API 교체 격리점 §7).
//
// 두 소스:
//   (A) Extended-URDF sidecar (closure_yaml_path 설정): 순수 spanning-tree URDF +
//       별도 <name>.closure.yaml 로부터 통합 파이프라인(constraints + q_ref +
//       actuated joints + 특이성 검사)을 full_model_ 위에서 실행. xacro 는 이미
//       BuildFullModel 에서 전처리되었으므로 raw buildModel 대신 여기서 재사용.
//   (B) legacy inline: config_.closed_chains 목록 → constraints 만.
void PinocchioModelBuilder::RegisterClosedChainConstraints() {
  // sidecar 는 LoadClosureSpecAndComputePassiveLocks() 에서 이미 파싱해 캐시했다
  // (closure_yaml_path 비었으면 nullopt). 여기서는 재로드 없이 캐시를 재사용한다.
  if (closure_spec_.has_value()) {
    if (!config_.closed_chains.empty()) {
      RCLCPP_WARN(logger(),
                  "closure_yaml_path 와 inline closed_chains 가 동시 설정됨 — sidecar 를 "
                  "사용하고 inline closed_chains (%zu개) 는 무시합니다",
                  config_.closed_chains.size());
    }
    const ClosureSpec& spec = *closure_spec_;
    ClosedChainData data = BuildClosedChainData(*full_model_, spec);
    constraint_models_ = std::move(data.constraints);
    closure_actuated_joint_ids_ = std::move(data.actuated_joint_ids);
    closure_q_ref_ = std::move(data.q_ref);
    closure_q_ref_converged_ = data.q_ref_converged;
    closure_q_ref_acceptable_ = data.q_ref_acceptable;
    closure_q_ref_singular_ = data.q_ref_singular;
    RCLCPP_INFO(logger(),
                "Extended-URDF closure 로드: '%s' (%zu constraint, %zu actuated joint, "
                "q_ref converged=%d acceptable=%d singular=%d)",
                config_.closure_yaml_path.c_str(), constraint_models_.size(),
                closure_actuated_joint_ids_.size(), static_cast<int>(closure_q_ref_converged_),
                static_cast<int>(closure_q_ref_acceptable_),
                static_cast<int>(closure_q_ref_singular_));
    return;
  }
  if (config_.closed_chains.empty()) {
    return;
  }
  constraint_models_ = BuildRigidConstraints(*full_model_, config_.closed_chains);
}

// ── 기준 설정값 구성 ────────────────────────────────────────────────────────
Eigen::VectorXd PinocchioModelBuilder::MakeReferenceConfig() const {
  Eigen::VectorXd q = pinocchio::neutral(*full_model_);

  for (const auto& [joint_name, value] : config_.lock_reference_config) {
    if (!full_model_->existJointName(joint_name))
      continue;

    auto jid = full_model_->getJointId(joint_name);
    auto q_start = full_model_->idx_qs[jid];
    auto nq_j = full_model_->nqs[jid];

    if (nq_j == 1) {
      q[q_start] = value;
    }
  }

  return q;
}

// ── 관절 이름 → JointIndex 변환 ─────────────────────────────────────────────
std::vector<pinocchio::JointIndex> PinocchioModelBuilder::ResolveJointIndicesToLock(
    const std::vector<std::string>& joint_names_to_lock) const {
  std::vector<pinocchio::JointIndex> indices;
  indices.reserve(joint_names_to_lock.size());

  for (const auto& name : joint_names_to_lock) {
    if (!full_model_->existJointName(name)) {
      // URDF에는 있지만 Pinocchio 모델에 없는 관절 (fixed 등) → 무시
      continue;
    }
    indices.push_back(full_model_->getJointId(name));
  }

  return indices;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 접근자
// ═══════════════════════════════════════════════════════════════════════════════

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetFullModel() const noexcept {
  return full_model_;
}

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetReducedModel(
    std::string_view sub_model_name) const {
  auto it = reduced_models_.find(std::string(sub_model_name));
  if (it == reduced_models_.end()) {
    RCLCPP_ERROR(logger(), "서브모델을 찾을 수 없습니다: %s", std::string(sub_model_name).c_str());
    throw std::out_of_range("PinocchioModelBuilder: 서브모델을 찾을 수 없습니다: " +
                            std::string(sub_model_name));
  }
  return it->second;
}

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetTreeModel(
    std::string_view tree_model_name) const {
  auto it = tree_models_.find(std::string(tree_model_name));
  if (it == tree_models_.end()) {
    RCLCPP_ERROR(logger(), "트리모델을 찾을 수 없습니다: %s", std::string(tree_model_name).c_str());
    throw std::out_of_range("PinocchioModelBuilder: 트리모델을 찾을 수 없습니다: " +
                            std::string(tree_model_name));
  }
  return it->second;
}

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetActuatedModel() const noexcept {
  return actuated_model_;
}

const std::vector<pinocchio::RigidConstraintModel>& PinocchioModelBuilder::GetConstraintModels()
    const noexcept {
  return constraint_models_;
}

const Eigen::VectorXd& PinocchioModelBuilder::GetClosureReferenceConfig() const noexcept {
  return closure_q_ref_;
}

const std::vector<pinocchio::JointIndex>& PinocchioModelBuilder::GetClosureActuatedJointIds()
    const noexcept {
  return closure_actuated_joint_ids_;
}

const std::vector<std::string>& PinocchioModelBuilder::GetClosurePassiveLockNames() const noexcept {
  return closure_passive_lock_names_;
}

bool PinocchioModelBuilder::IsClosureReferenceConverged() const noexcept {
  return closure_q_ref_converged_;
}

bool PinocchioModelBuilder::IsClosureReferenceAcceptable() const noexcept {
  return closure_q_ref_acceptable_;
}

bool PinocchioModelBuilder::IsClosureReferenceSingular() const noexcept {
  return closure_q_ref_singular_;
}

std::vector<std::string> PinocchioModelBuilder::GetSubModelNames() const {
  std::vector<std::string> names;
  names.reserve(reduced_models_.size());
  for (const auto& [k, v] : reduced_models_) {
    names.push_back(k);
  }
  return names;
}

std::vector<std::string> PinocchioModelBuilder::GetTreeModelNames() const {
  std::vector<std::string> names;
  names.reserve(tree_models_.size());
  for (const auto& [k, v] : tree_models_) {
    names.push_back(k);
  }
  return names;
}

const SubModelDefinition& PinocchioModelBuilder::GetSubModelDefinition(
    std::string_view name) const {
  auto it = sub_model_defs_.find(std::string(name));
  if (it == sub_model_defs_.end()) {
    RCLCPP_ERROR(logger(), "서브모델 정의를 찾을 수 없습니다: %s", std::string(name).c_str());
    throw std::out_of_range("PinocchioModelBuilder: 서브모델 정의를 찾을 수 없습니다: " +
                            std::string(name));
  }
  return it->second;
}

const TreeModelDefinition& PinocchioModelBuilder::GetTreeModelDefinition(
    std::string_view name) const {
  auto it = tree_model_defs_.find(std::string(name));
  if (it == tree_model_defs_.end()) {
    RCLCPP_ERROR(logger(), "트리모델 정의를 찾을 수 없습니다: %s", std::string(name).c_str());
    throw std::out_of_range("PinocchioModelBuilder: 트리모델 정의를 찾을 수 없습니다: " +
                            std::string(name));
  }
  return it->second;
}

const UrdfAnalyzer& PinocchioModelBuilder::GetAnalyzer() const noexcept {
  return *analyzer_;
}

const KinematicChainExtractor& PinocchioModelBuilder::GetExtractor() const noexcept {
  return *extractor_;
}

const ModelConfig& PinocchioModelBuilder::GetConfig() const noexcept {
  return config_;
}

// ═══════════════════════════════════════════════════════════════════════════════
// YAML 설정 로드
// ═══════════════════════════════════════════════════════════════════════════════

ModelConfig PinocchioModelBuilder::LoadModelConfig(std::string_view yaml_path) {
  RCLCPP_DEBUG(logger(), "ModelConfig 로드: %s", std::string(yaml_path).c_str());
  YAML::Node root = YAML::LoadFile(std::string(yaml_path));
  ModelConfig cfg;

  // urdf_path — 상대 경로면 YAML 파일 기준으로 해석.
  if (root["urdf_path"]) {
    cfg.urdf_path = ResolveRelativeToYaml(root["urdf_path"].as<std::string>(), yaml_path);
  }

  // closure_yaml_path (Extended-URDF sidecar). 상대 경로면 YAML 파일 기준 해석.
  if (root["closure_yaml_path"]) {
    cfg.closure_yaml_path =
        ResolveRelativeToYaml(root["closure_yaml_path"].as<std::string>(), yaml_path);
  }

  // xacro_args
  if (root["xacro_args"]) {
    for (const auto& kv : root["xacro_args"]) {
      cfg.xacro_args[kv.first.as<std::string>()] = kv.second.as<std::string>();
    }
  }

  // root_joint_type
  if (root["root_joint_type"]) {
    cfg.root_joint_type = root["root_joint_type"].as<std::string>();
  }

  // sub_models
  if (root["sub_models"]) {
    for (const auto& node : root["sub_models"]) {
      SubModelConfig sc;
      sc.name = node["name"].as<std::string>();
      sc.root_link = node["root_link"].as<std::string>();
      sc.tip_link = node["tip_link"].as<std::string>();
      cfg.sub_models.push_back(std::move(sc));
    }
  }

  // tree_models
  if (root["tree_models"]) {
    for (const auto& node : root["tree_models"]) {
      TreeModelConfig tc;
      tc.name = node["name"].as<std::string>();
      tc.root_link = node["root_link"].as<std::string>();
      for (const auto& tip : node["tip_links"]) {
        tc.tip_links.push_back(tip.as<std::string>());
      }
      cfg.tree_models.push_back(std::move(tc));
    }
  }

  // closed_chains
  if (root["closed_chains"]) {
    for (const auto& node : root["closed_chains"]) {
      ClosedChainInfo cc;
      cc.name = node["name"].as<std::string>();
      cc.link_a = node["link_a"].as<std::string>();
      cc.link_b = node["link_b"].as<std::string>();

      if (node["contact_type"]) {
        cc.is_6d = (node["contact_type"].as<std::string>() != "3D");
      }
      if (node["offset_a"]) {
        auto oa = node["offset_a"];
        if (oa["xyz"]) {
          auto xyz = oa["xyz"];
          cc.offset_a_xyz << xyz[0].as<double>(), xyz[1].as<double>(), xyz[2].as<double>();
        }
        if (oa["rpy"]) {
          auto rpy = oa["rpy"];
          cc.offset_a_rpy << rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>();
        }
      }
      if (node["offset_b"]) {
        auto ob = node["offset_b"];
        if (ob["xyz"]) {
          auto xyz = ob["xyz"];
          cc.offset_b_xyz << xyz[0].as<double>(), xyz[1].as<double>(), xyz[2].as<double>();
        }
        if (ob["rpy"]) {
          auto rpy = ob["rpy"];
          cc.offset_b_rpy << rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>();
        }
      }
      if (node["baumgarte_kp"])
        cc.baumgarte_kp = node["baumgarte_kp"].as<double>();
      if (node["baumgarte_kd"])
        cc.baumgarte_kd = node["baumgarte_kd"].as<double>();

      cfg.closed_chains.push_back(std::move(cc));
    }
  }

  // passive_joints
  if (root["passive_joints"]) {
    for (const auto& node : root["passive_joints"]) {
      cfg.passive_joints.push_back(node.as<std::string>());
    }
  }

  // yaml_passive_override (Q4 옵션 A 스위치)
  if (root["yaml_passive_override"]) {
    cfg.yaml_passive_override = root["yaml_passive_override"].as<bool>();
  }

  // lock_reference_config
  if (root["lock_reference_config"]) {
    for (const auto& kv : root["lock_reference_config"]) {
      cfg.lock_reference_config[kv.first.as<std::string>()] = kv.second.as<double>();
    }
  }

  return cfg;
}

}  // namespace rtc_urdf_bridge
