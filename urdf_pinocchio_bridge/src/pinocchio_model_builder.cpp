// ── PinocchioModelBuilder 구현 ───────────────────────────────────────────────
#include "urdf_pinocchio_bridge/pinocchio_model_builder.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace urdf_pinocchio_bridge
{

// ═══════════════════════════════════════════════════════════════════════════════
// 생성자
// ═══════════════════════════════════════════════════════════════════════════════

PinocchioModelBuilder::PinocchioModelBuilder(std::string_view yaml_config_path)
: config_(LoadModelConfig(yaml_config_path))
{
  Build();
}

PinocchioModelBuilder::PinocchioModelBuilder(const ModelConfig & config)
: config_(config)
{
  Build();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Build 파이프라인
// ═══════════════════════════════════════════════════════════════════════════════

void PinocchioModelBuilder::Build()
{
  // (1) URDF 분석
  if (!config_.urdf_path.empty()) {
    analyzer_ = std::make_unique<UrdfAnalyzer>(config_.urdf_path);
  } else if (!config_.urdf_xml_string.empty()) {
    analyzer_ = std::make_unique<UrdfAnalyzer>(
      config_.urdf_xml_string, UrdfAnalyzer::FromXmlTag{});
  } else {
    throw std::runtime_error("PinocchioModelBuilder: urdf_path 또는 urdf_xml_string 필요");
  }

  extractor_ = std::make_unique<KinematicChainExtractor>(*analyzer_);

  // (2) mimic 관절을 passive 목록에 추가
  IncorporateMimicAsPassive();

  // (3) Pinocchio 모델 구축
  BuildFullModel();
  BuildReducedModels();
  BuildTreeModels();
  RegisterClosedChainConstraints();
}

// ── 전체 모델 구축 ──────────────────────────────────────────────────────────
void PinocchioModelBuilder::BuildFullModel()
{
  full_model_ = std::make_shared<pinocchio::Model>();

  if (config_.root_joint_type == "floating") {
    // 플로팅 베이스 (모바일/휴머노이드)
    if (!config_.urdf_path.empty()) {
      pinocchio::urdf::buildModel(
        config_.urdf_path, pinocchio::JointModelFreeFlyer(), *full_model_);
    } else {
      pinocchio::urdf::buildModelFromXML(
        config_.urdf_xml_string, pinocchio::JointModelFreeFlyer(), *full_model_);
    }
  } else {
    // 고정 베이스 (기본)
    if (!config_.urdf_path.empty()) {
      pinocchio::urdf::buildModel(config_.urdf_path, *full_model_);
    } else {
      pinocchio::urdf::buildModelFromXML(config_.urdf_xml_string, *full_model_);
    }
  }
}

// ── mimic 관절을 passive 목록에 추가 ────────────────────────────────────────
void PinocchioModelBuilder::IncorporateMimicAsPassive()
{
  const auto & mimics = analyzer_->GetMimicJoints();
  for (const auto & mi : mimics) {
    // 이미 passive 목록에 있는지 확인
    auto it = std::find(
      config_.passive_joints.begin(), config_.passive_joints.end(), mi.joint_name);
    if (it == config_.passive_joints.end()) {
      config_.passive_joints.push_back(mi.joint_name);
    }
  }
}

// ── 축소 모델 구축 ──────────────────────────────────────────────────────────
void PinocchioModelBuilder::BuildReducedModels()
{
  auto ref_config = MakeReferenceConfig();

  for (const auto & sub_cfg : config_.sub_models) {
    // 체인 추출
    auto def = extractor_->ExtractSubModel(
      sub_cfg.name, sub_cfg.root_link, sub_cfg.tip_link);

    // 잠글 관절 계산
    auto joints_to_lock_names = extractor_->ComputeJointsToLock(def);

    // YAML passive_joints는 무조건 잠금 대상에 추가
    // (체인 내/외 구분 없이 — transmission 없는 URDF에서도 올바르게 동작)
    for (const auto & pj : config_.passive_joints) {
      auto it = std::find(joints_to_lock_names.begin(), joints_to_lock_names.end(), pj);
      if (it == joints_to_lock_names.end()) {
        joints_to_lock_names.push_back(pj);
      }
      // 체인 내 passive 관절이면 actuated 목록에서 제거
      auto jit = std::find(def.joint_names.begin(), def.joint_names.end(), pj);
      if (jit != def.joint_names.end()) {
        def.joint_names.erase(jit);
      }
    }

    auto joints_to_lock = ResolveJointIndicesToLock(joints_to_lock_names);

    auto reduced = std::make_shared<pinocchio::Model>();
    pinocchio::buildReducedModel(*full_model_, joints_to_lock, ref_config, *reduced);

    reduced_models_[sub_cfg.name] = std::move(reduced);
    sub_model_defs_[sub_cfg.name] = std::move(def);
  }
}

// ── 트리 모델 구축 ──────────────────────────────────────────────────────────
void PinocchioModelBuilder::BuildTreeModels()
{
  auto ref_config = MakeReferenceConfig();

  for (const auto & tree_cfg : config_.tree_models) {
    auto def = extractor_->ExtractTreeModel(
      tree_cfg.name, tree_cfg.root_link, tree_cfg.tip_links);

    auto joints_to_lock_names = extractor_->ComputeJointsToLock(def);

    // passive 관절 잠금 (체인 내/외 구분 없이)
    for (const auto & pj : config_.passive_joints) {
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

    tree_models_[tree_cfg.name] = std::move(tree_model);
    tree_model_defs_[tree_cfg.name] = std::move(def);
  }
}

// ── 폐쇄 체인 구속 등록 ────────────────────────────────────────────────────
void PinocchioModelBuilder::RegisterClosedChainConstraints()
{
  for (const auto & cc : config_.closed_chains) {
    // link 이름 → frame ID → parent joint ID
    pinocchio::FrameIndex fid_a = 0;
    pinocchio::FrameIndex fid_b = 0;
    bool found_a = false, found_b = false;

    for (pinocchio::FrameIndex i = 0;
         i < static_cast<pinocchio::FrameIndex>(full_model_->nframes); ++i)
    {
      if (full_model_->frames[i].name == cc.link_a) { fid_a = i; found_a = true; }
      if (full_model_->frames[i].name == cc.link_b) { fid_b = i; found_b = true; }
    }

    if (!found_a || !found_b) {
      throw std::runtime_error(
        "PinocchioModelBuilder: 폐쇄 체인 링크를 찾을 수 없습니다: " +
        cc.link_a + " / " + cc.link_b);
    }

    auto joint1_id = full_model_->frames[fid_a].parentJoint;
    auto joint2_id = full_model_->frames[fid_b].parentJoint;

    // SE3 placement 구성
    auto make_se3 = [](const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy) {
      return pinocchio::SE3(pinocchio::rpy::rpyToMatrix(rpy[0], rpy[1], rpy[2]), xyz);
    };

    auto placement_a = make_se3(cc.offset_a_xyz, cc.offset_a_rpy);
    auto placement_b = make_se3(cc.offset_b_xyz, cc.offset_b_rpy);

    auto contact_type = cc.is_6d ? pinocchio::CONTACT_6D : pinocchio::CONTACT_3D;

    pinocchio::RigidConstraintModel constraint(
      contact_type,
      *full_model_,
      joint1_id,
      placement_a,
      joint2_id,
      placement_b,
      pinocchio::LOCAL);

    // Baumgarte 안정화 파라미터
    constraint.corrector.Kp.setConstant(cc.baumgarte_kp);
    constraint.corrector.Kd.setConstant(cc.baumgarte_kd);

    constraint_models_.push_back(std::move(constraint));
  }
}

// ── 기준 설정값 구성 ────────────────────────────────────────────────────────
Eigen::VectorXd PinocchioModelBuilder::MakeReferenceConfig() const
{
  Eigen::VectorXd q = pinocchio::neutral(*full_model_);

  for (const auto & [joint_name, value] : config_.lock_reference_config) {
    if (!full_model_->existJointName(joint_name)) continue;

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
  const std::vector<std::string> & joint_names_to_lock) const
{
  std::vector<pinocchio::JointIndex> indices;
  indices.reserve(joint_names_to_lock.size());

  for (const auto & name : joint_names_to_lock) {
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

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetFullModel() const noexcept
{
  return full_model_;
}

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetReducedModel(
  std::string_view sub_model_name) const
{
  auto it = reduced_models_.find(std::string(sub_model_name));
  if (it == reduced_models_.end()) {
    throw std::out_of_range(
      "PinocchioModelBuilder: 서브모델을 찾을 수 없습니다: " +
      std::string(sub_model_name));
  }
  return it->second;
}

std::shared_ptr<const pinocchio::Model> PinocchioModelBuilder::GetTreeModel(
  std::string_view tree_model_name) const
{
  auto it = tree_models_.find(std::string(tree_model_name));
  if (it == tree_models_.end()) {
    throw std::out_of_range(
      "PinocchioModelBuilder: 트리모델을 찾을 수 없습니다: " +
      std::string(tree_model_name));
  }
  return it->second;
}

const std::vector<pinocchio::RigidConstraintModel> &
PinocchioModelBuilder::GetConstraintModels() const noexcept
{
  return constraint_models_;
}

std::vector<std::string> PinocchioModelBuilder::GetSubModelNames() const
{
  std::vector<std::string> names;
  names.reserve(reduced_models_.size());
  for (const auto & [k, v] : reduced_models_) {
    names.push_back(k);
  }
  return names;
}

std::vector<std::string> PinocchioModelBuilder::GetTreeModelNames() const
{
  std::vector<std::string> names;
  names.reserve(tree_models_.size());
  for (const auto & [k, v] : tree_models_) {
    names.push_back(k);
  }
  return names;
}

const SubModelDefinition & PinocchioModelBuilder::GetSubModelDefinition(
  std::string_view name) const
{
  auto it = sub_model_defs_.find(std::string(name));
  if (it == sub_model_defs_.end()) {
    throw std::out_of_range(
      "PinocchioModelBuilder: 서브모델 정의를 찾을 수 없습니다: " + std::string(name));
  }
  return it->second;
}

const TreeModelDefinition & PinocchioModelBuilder::GetTreeModelDefinition(
  std::string_view name) const
{
  auto it = tree_model_defs_.find(std::string(name));
  if (it == tree_model_defs_.end()) {
    throw std::out_of_range(
      "PinocchioModelBuilder: 트리모델 정의를 찾을 수 없습니다: " + std::string(name));
  }
  return it->second;
}

const UrdfAnalyzer & PinocchioModelBuilder::GetAnalyzer() const noexcept
{
  return *analyzer_;
}

const KinematicChainExtractor & PinocchioModelBuilder::GetExtractor() const noexcept
{
  return *extractor_;
}

const ModelConfig & PinocchioModelBuilder::GetConfig() const noexcept
{
  return config_;
}

// ═══════════════════════════════════════════════════════════════════════════════
// YAML 설정 로드
// ═══════════════════════════════════════════════════════════════════════════════

ModelConfig PinocchioModelBuilder::LoadModelConfig(std::string_view yaml_path)
{
  YAML::Node root = YAML::LoadFile(std::string(yaml_path));
  ModelConfig cfg;

  // urdf_path
  if (root["urdf_path"]) {
    cfg.urdf_path = root["urdf_path"].as<std::string>();
    // 상대 경로면 YAML 파일 기준으로 해석
    if (!cfg.urdf_path.empty() && cfg.urdf_path[0] != '/') {
      std::filesystem::path yaml_dir =
        std::filesystem::path(std::string(yaml_path)).parent_path();
      cfg.urdf_path = (yaml_dir / cfg.urdf_path).string();
    }
  }

  // root_joint_type
  if (root["root_joint_type"]) {
    cfg.root_joint_type = root["root_joint_type"].as<std::string>();
  }

  // sub_models
  if (root["sub_models"]) {
    for (const auto & node : root["sub_models"]) {
      SubModelConfig sc;
      sc.name = node["name"].as<std::string>();
      sc.root_link = node["root_link"].as<std::string>();
      sc.tip_link = node["tip_link"].as<std::string>();
      cfg.sub_models.push_back(std::move(sc));
    }
  }

  // tree_models
  if (root["tree_models"]) {
    for (const auto & node : root["tree_models"]) {
      TreeModelConfig tc;
      tc.name = node["name"].as<std::string>();
      tc.root_link = node["root_link"].as<std::string>();
      for (const auto & tip : node["tip_links"]) {
        tc.tip_links.push_back(tip.as<std::string>());
      }
      cfg.tree_models.push_back(std::move(tc));
    }
  }

  // closed_chains
  if (root["closed_chains"]) {
    for (const auto & node : root["closed_chains"]) {
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
      if (node["baumgarte_kp"]) cc.baumgarte_kp = node["baumgarte_kp"].as<double>();
      if (node["baumgarte_kd"]) cc.baumgarte_kd = node["baumgarte_kd"].as<double>();

      cfg.closed_chains.push_back(std::move(cc));
    }
  }

  // passive_joints
  if (root["passive_joints"]) {
    for (const auto & node : root["passive_joints"]) {
      cfg.passive_joints.push_back(node.as<std::string>());
    }
  }

  // lock_reference_config
  if (root["lock_reference_config"]) {
    for (const auto & kv : root["lock_reference_config"]) {
      cfg.lock_reference_config[kv.first.as<std::string>()] = kv.second.as<double>();
    }
  }

  return cfg;
}

}  // namespace urdf_pinocchio_bridge
