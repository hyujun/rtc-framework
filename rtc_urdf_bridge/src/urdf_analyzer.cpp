// ── UrdfAnalyzer 구현 ────────────────────────────────────────────────────────
#include "rtc_urdf_bridge/urdf_analyzer.hpp"
#include "rtc_urdf_bridge/urdf_logging.hpp"
#include "rtc_urdf_bridge/xacro_processor.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rtc_urdf_bridge {

namespace {
auto logger() { return ::rtc::urdf::logging::AnalyzerLogger(); }
} // namespace

// ── 생성자 (파일 경로) ───────────────────────────────────────────────────────
UrdfAnalyzer::UrdfAnalyzer(std::string_view urdf_file_path,
                           std::vector<std::string> passive_hints)
    : urdf_file_path_(urdf_file_path) {
  for (auto &h : passive_hints)
    passive_hints_.insert(std::move(h));

  RCLCPP_DEBUG(logger(), "UrdfAnalyzer 로드: %s", urdf_file_path_.c_str());
  if (IsXacroFile(urdf_file_path_)) {
    // xacro 전처리 → URDF XML 문자열
    urdf_xml_string_ = ProcessXacro(urdf_file_path_);
  } else {
    // 일반 URDF 파일 읽기
    std::ifstream ifs(urdf_file_path_);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(logger(), "URDF 파일을 열 수 없습니다: %s",
                   urdf_file_path_.c_str());
      throw std::runtime_error("UrdfAnalyzer: URDF 파일을 열 수 없습니다: " +
                               urdf_file_path_);
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    urdf_xml_string_ = oss.str();
  }

  ParseUrdfXml(urdf_xml_string_);
}

// ── 생성자 (XML 문자열) ─────────────────────────────────────────────────────
UrdfAnalyzer::UrdfAnalyzer(std::string_view xml_string, FromXmlTag,
                           std::vector<std::string> passive_hints)
    : urdf_xml_string_(xml_string) {
  for (auto &h : passive_hints)
    passive_hints_.insert(std::move(h));
  ParseUrdfXml(urdf_xml_string_);
}

// ── URDF XML 파싱 메인 로직 ─────────────────────────────────────────────────
void UrdfAnalyzer::ParseUrdfXml(const std::string &xml) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger(), "XML 파싱 실패 — %s", doc.ErrorStr());
    throw std::runtime_error("UrdfAnalyzer: XML 파싱 실패 — " +
                             std::string(doc.ErrorStr()));
  }

  auto *robot = doc.FirstChildElement("robot");
  if (!robot) {
    RCLCPP_ERROR(logger(), "<robot> 루트 요소가 없습니다");
    throw std::runtime_error("UrdfAnalyzer: <robot> 루트 요소가 없습니다");
  }

  // ── (1) 모든 <link> 수집 ──────────────────────────────────────────────────
  for (auto *link_el = robot->FirstChildElement("link"); link_el != nullptr;
       link_el = link_el->NextSiblingElement("link")) {
    const char *name_attr = link_el->Attribute("name");
    if (!name_attr)
      continue;

    auto idx = static_cast<int>(link_nodes_.size());
    link_name_to_index_[name_attr] = idx;

    LinkNode node;
    node.link_name = name_attr;
    link_nodes_.push_back(std::move(node));
  }

  // ── (2) 모든 <joint> 수집 ─────────────────────────────────────────────────
  for (auto *joint_el = robot->FirstChildElement("joint"); joint_el != nullptr;
       joint_el = joint_el->NextSiblingElement("joint")) {
    const char *name_attr = joint_el->Attribute("name");
    const char *type_attr = joint_el->Attribute("type");
    if (!name_attr || !type_attr)
      continue;

    JointMeta meta;
    meta.joint_name = name_attr;
    meta.type = StringToJointType(type_attr);

    // parent / child link
    auto *parent_el = joint_el->FirstChildElement("parent");
    auto *child_el = joint_el->FirstChildElement("child");
    if (!parent_el || !child_el)
      continue;

    const char *parent_link = parent_el->Attribute("link");
    const char *child_link = child_el->Attribute("link");
    if (!parent_link || !child_link)
      continue;

    meta.parent_link = parent_link;
    meta.child_link = child_link;

    // axis (기본값: Z축)
    if (auto *axis_el = joint_el->FirstChildElement("axis")) {
      const char *xyz = axis_el->Attribute("xyz");
      if (xyz) {
        std::istringstream iss(xyz);
        iss >> meta.axis[0] >> meta.axis[1] >> meta.axis[2];
      }
    }

    // limits
    if (auto *limit_el = joint_el->FirstChildElement("limit")) {
      meta.has_limit_tag = true;
      limit_el->QueryDoubleAttribute("lower", &meta.lower);
      limit_el->QueryDoubleAttribute("upper", &meta.upper);
      limit_el->QueryDoubleAttribute("effort", &meta.effort);
      limit_el->QueryDoubleAttribute("velocity", &meta.velocity);
    }

    // dynamics
    if (auto *dyn_el = joint_el->FirstChildElement("dynamics")) {
      dyn_el->QueryDoubleAttribute("damping", &meta.damping);
      dyn_el->QueryDoubleAttribute("friction", &meta.friction);
    }

    // mimic 태그 감지
    if (auto *mimic_el = joint_el->FirstChildElement("mimic")) {
      const char *mimic_joint = mimic_el->Attribute("joint");
      if (mimic_joint) {
        MimicJointInfo mi;
        mi.joint_name = name_attr;
        mi.mimicked_joint = mimic_joint;
        mi.multiplier = 1.0;
        mi.offset = 0.0;
        mimic_el->QueryDoubleAttribute("multiplier", &mi.multiplier);
        mimic_el->QueryDoubleAttribute("offset", &mi.offset);
        mimic_joints_.push_back(std::move(mi));
      }
    }

    all_joint_names_.push_back(name_attr);
    joint_meta_map_[name_attr] = std::move(meta);
  }

  // ── (3) 그래프 구축 → 깊이 → 폐쇄 체인 참여 관절 → 역할 분류 ─────────────
  BuildAdjacencyGraph();
  ComputeDepths();
  CollectClosedChainJoints();
  ClassifyJointRoles();

  // subtype별 카운트 요약
  std::size_t n_mimic = 0, n_closed = 0, n_free = 0;
  for (const auto &pi : passive_joints_) {
    switch (pi.subtype) {
    case PassiveSubtype::kMimic:
      ++n_mimic;
      break;
    case PassiveSubtype::kClosedChain:
      ++n_closed;
      break;
    case PassiveSubtype::kFree:
      ++n_free;
      break;
    case PassiveSubtype::kNone:
      break;
    }
  }

  RCLCPP_INFO(logger(),
              "URDF 파싱 완료: links=%zu, joints=%zu, "
              "fixed=%zu, active=%zu, passive=%zu (mimic=%zu, "
              "closed_chain=%zu, free=%zu), "
              "root='%s'",
              link_nodes_.size(), joint_meta_map_.size(),
              fixed_joint_names_.size(), active_joint_names_.size(),
              passive_joint_names_.size(), n_mimic, n_closed, n_free,
              root_index_ >= 0 ? GetRootLinkName().c_str() : "(없음)");
}

// ── 인접 그래프 구축 ────────────────────────────────────────────────────────
void UrdfAnalyzer::BuildAdjacencyGraph() {
  // 자식으로 참조되는 링크 집합 (루트 감지용)
  std::unordered_set<std::string> child_links;

  for (const auto &[name, meta] : joint_meta_map_) {
    auto parent_it = link_name_to_index_.find(meta.parent_link);
    auto child_it = link_name_to_index_.find(meta.child_link);
    if (parent_it == link_name_to_index_.end() ||
        child_it == link_name_to_index_.end()) {
      continue;
    }

    int parent_idx = parent_it->second;
    int child_idx = child_it->second;

    link_nodes_[static_cast<std::size_t>(child_idx)].parent_index = parent_idx;
    link_nodes_[static_cast<std::size_t>(child_idx)].parent_joint_name = name;
    link_nodes_[static_cast<std::size_t>(child_idx)].parent_joint_type =
        meta.type;
    link_nodes_[static_cast<std::size_t>(parent_idx)].child_indices.push_back(
        child_idx);

    child_links.insert(meta.child_link);
  }

  // 루트 링크: 어떤 관절의 child도 아닌 링크
  for (std::size_t i = 0; i < link_nodes_.size(); ++i) {
    if (child_links.find(link_nodes_[i].link_name) == child_links.end()) {
      root_index_ = static_cast<int>(i);
      break;
    }
  }
}

// ── Physics 판정 (A+B 조합) ────────────────────────────────────────────────
bool UrdfAnalyzer::CheckPhysics(const JointMeta &meta) const noexcept {
  if (!meta.has_limit_tag)
    return false;
  if (meta.effort <= 0.0)
    return false;
  // continuous joint은 URDF 스펙상 position limit 없이 effort/velocity만 존재
  // 가능
  if (meta.type != UrdfJointType::kContinuous && meta.velocity <= 0.0)
    return false;
  return true;
}

// ── 폐쇄 체인 참여 관절 수집 ───────────────────────────────────────────────
// DetectClosedChains() 결과에서 link_a ↔ link_b 트리 경로 상의 non-fixed
// 관절을 closed_chain_joints_ 집합에 기록.
void UrdfAnalyzer::CollectClosedChainJoints() {
  const auto chains = DetectClosedChains();
  for (const auto &cc : chains) {
    // link 이름이 그래프에 없으면 경고 후 skip
    if (link_name_to_index_.find(cc.link_a) == link_name_to_index_.end() ||
        link_name_to_index_.find(cc.link_b) == link_name_to_index_.end()) {
      RCLCPP_WARN(logger(),
                  "Closed-chain '%s': 링크 '%s'↔'%s' 중 그래프에 없는 것이 "
                  "있어 관절 분류에서 제외합니다",
                  cc.name.c_str(), cc.link_a.c_str(), cc.link_b.c_str());
      continue;
    }

    auto path = FindPath(cc.link_a, cc.link_b);
    if (path.size() < 2)
      continue;

    for (std::size_t i = 1; i < path.size(); ++i) {
      int prev = path[i - 1];
      int cur = path[i];
      const auto &cur_node = link_nodes_[static_cast<std::size_t>(cur)];
      const auto &prev_node = link_nodes_[static_cast<std::size_t>(prev)];

      std::string jname;
      if (cur_node.parent_index == prev &&
          !cur_node.parent_joint_name.empty()) {
        jname = cur_node.parent_joint_name;
      } else if (prev_node.parent_index == cur &&
                 !prev_node.parent_joint_name.empty()) {
        jname = prev_node.parent_joint_name;
      }
      if (jname.empty())
        continue;

      auto it = joint_meta_map_.find(jname);
      if (it == joint_meta_map_.end())
        continue;
      if (it->second.type == UrdfJointType::kFixed)
        continue;

      closed_chain_joints_.insert(jname);
    }
  }
}

// ── 관절 역할 분류 (fixed/active/passive + subtype) ────────────────────────
void UrdfAnalyzer::ClassifyJointRoles() {
  std::unordered_set<std::string> mimic_names;
  for (const auto &mi : mimic_joints_)
    mimic_names.insert(mi.joint_name);

  for (auto &[name, meta] : joint_meta_map_) {
    meta.has_physics = CheckPhysics(meta);

    // 1. fixed
    if (meta.type == UrdfJointType::kFixed) {
      meta.role = JointRole::kFixed;
      meta.passive_subtype = PassiveSubtype::kNone;
      fixed_joint_names_.push_back(name);
      continue;
    }

    const bool is_mimic = mimic_names.count(name) > 0;
    const bool is_closed = closed_chain_joints_.count(name) > 0;
    const bool is_hint = passive_hints_.count(name) > 0;

    // 중복 subtype 검출 (mimic ∧ closed-chain)
    if (is_mimic && is_closed) {
      RCLCPP_WARN(
          logger(),
          "Joint '%s': mimic과 closed-chain 양쪽에 정의됨. kMimic으로 분류.",
          name.c_str());
    }

    if (is_mimic) {
      meta.role = JointRole::kPassive;
      meta.passive_subtype = PassiveSubtype::kMimic;
    } else if (is_closed) {
      meta.role = JointRole::kPassive;
      meta.passive_subtype = PassiveSubtype::kClosedChain;
    } else if (is_hint) {
      meta.role = JointRole::kPassive;
      meta.passive_subtype = PassiveSubtype::kFree;
    } else {
      // 기본값: active. physics 없으면 warning.
      meta.role = JointRole::kActive;
      meta.passive_subtype = PassiveSubtype::kNone;
      if (!meta.has_physics) {
        RCLCPP_WARN(
            logger(),
            "Joint '%s' 는 active로 분류되었으나 physics가 정의되지 않았습니다 "
            "(has_limit=%d, effort=%.3f, velocity=%.3f). "
            "명시적 passive_hints 추가를 고려하세요.",
            name.c_str(), meta.has_limit_tag ? 1 : 0, meta.effort,
            meta.velocity);
      }
    }

    // 이름 목록 누적
    non_fixed_joint_names_.push_back(name);
    if (meta.role == JointRole::kActive) {
      active_joint_names_.push_back(name);
    } else {
      passive_joint_names_.push_back(name);
      PassiveJointInfo pi;
      pi.joint_name = name;
      pi.type = meta.type;
      pi.subtype = meta.passive_subtype;
      pi.damping = meta.damping;
      pi.friction = meta.friction;
      passive_joints_.push_back(std::move(pi));
    }
  }

  std::sort(active_joint_names_.begin(), active_joint_names_.end());
  std::sort(passive_joint_names_.begin(), passive_joint_names_.end());
  std::sort(fixed_joint_names_.begin(), fixed_joint_names_.end());
  std::sort(non_fixed_joint_names_.begin(), non_fixed_joint_names_.end());
  std::sort(passive_joints_.begin(), passive_joints_.end(),
            [](const PassiveJointInfo &a, const PassiveJointInfo &b) {
              return a.joint_name < b.joint_name;
            });

  // passive hint 중 존재하지 않는 이름 경고
  for (const auto &h : passive_hints_) {
    if (joint_meta_map_.find(h) == joint_meta_map_.end()) {
      RCLCPP_WARN(logger(),
                  "passive_hints에 지정된 '%s' 가 URDF에 존재하지 않습니다",
                  h.c_str());
    }
  }
}

// ── 깊이 테이블 계산 (BFS) ──────────────────────────────────────────────────
void UrdfAnalyzer::ComputeDepths() {
  depth_.assign(link_nodes_.size(), -1);
  if (root_index_ < 0)
    return;

  std::deque<int> queue;
  queue.push_back(root_index_);
  depth_[static_cast<std::size_t>(root_index_)] = 0;

  while (!queue.empty()) {
    int cur = queue.front();
    queue.pop_front();
    for (int child : link_nodes_[static_cast<std::size_t>(cur)].child_indices) {
      depth_[static_cast<std::size_t>(child)] =
          depth_[static_cast<std::size_t>(cur)] + 1;
      queue.push_back(child);
    }
  }
}

// ── 그래프 조회 ─────────────────────────────────────────────────────────────
const std::vector<LinkNode> &UrdfAnalyzer::GetLinkNodes() const noexcept {
  return link_nodes_;
}

int UrdfAnalyzer::GetLinkIndex(std::string_view link_name) const {
  auto it = link_name_to_index_.find(std::string(link_name));
  if (it == link_name_to_index_.end()) {
    RCLCPP_ERROR(logger(), "링크를 찾을 수 없습니다: %s",
                 std::string(link_name).c_str());
    throw std::out_of_range("UrdfAnalyzer: 링크를 찾을 수 없습니다: " +
                            std::string(link_name));
  }
  return it->second;
}

int UrdfAnalyzer::GetRootIndex() const noexcept { return root_index_; }

const std::string &UrdfAnalyzer::GetRootLinkName() const noexcept {
  return link_nodes_[static_cast<std::size_t>(root_index_)].link_name;
}

std::size_t UrdfAnalyzer::GetNumLinks() const noexcept {
  return link_nodes_.size();
}

std::size_t UrdfAnalyzer::GetNumJoints() const noexcept {
  return joint_meta_map_.size();
}

// ── 관절 분류 접근 ──────────────────────────────────────────────────────────
const std::vector<std::string> &
UrdfAnalyzer::GetActiveJointNames() const noexcept {
  return active_joint_names_;
}

const std::vector<std::string> &
UrdfAnalyzer::GetPassiveJointNames() const noexcept {
  return passive_joint_names_;
}

const std::vector<std::string> &
UrdfAnalyzer::GetFixedJointNames() const noexcept {
  return fixed_joint_names_;
}

const std::vector<std::string> &
UrdfAnalyzer::GetNonFixedJointNames() const noexcept {
  return non_fixed_joint_names_;
}

std::vector<std::string>
UrdfAnalyzer::GetPassiveJointNamesOfSubtype(PassiveSubtype subtype) const {
  std::vector<std::string> out;
  for (const auto &pi : passive_joints_) {
    if (pi.subtype == subtype)
      out.push_back(pi.joint_name);
  }
  return out;
}

const std::vector<PassiveJointInfo> &
UrdfAnalyzer::GetPassiveJoints() const noexcept {
  return passive_joints_;
}

const std::vector<MimicJointInfo> &
UrdfAnalyzer::GetMimicJoints() const noexcept {
  return mimic_joints_;
}

JointRole UrdfAnalyzer::GetJointRole(std::string_view joint_name) const {
  return GetJointMeta(joint_name).role;
}

PassiveSubtype
UrdfAnalyzer::GetPassiveSubtype(std::string_view joint_name) const {
  return GetJointMeta(joint_name).passive_subtype;
}

UrdfJointType UrdfAnalyzer::GetJointType(std::string_view joint_name) const {
  auto it = joint_meta_map_.find(std::string(joint_name));
  if (it == joint_meta_map_.end()) {
    RCLCPP_ERROR(logger(), "관절을 찾을 수 없습니다: %s",
                 std::string(joint_name).c_str());
    throw std::out_of_range("UrdfAnalyzer: 관절을 찾을 수 없습니다: " +
                            std::string(joint_name));
  }
  return it->second.type;
}

const JointMeta &UrdfAnalyzer::GetJointMeta(std::string_view joint_name) const {
  auto it = joint_meta_map_.find(std::string(joint_name));
  if (it == joint_meta_map_.end()) {
    RCLCPP_ERROR(logger(), "관절을 찾을 수 없습니다: %s",
                 std::string(joint_name).c_str());
    throw std::out_of_range("UrdfAnalyzer: 관절을 찾을 수 없습니다: " +
                            std::string(joint_name));
  }
  return it->second;
}

const std::string &
UrdfAnalyzer::GetJointChildLink(std::string_view joint_name) const {
  return GetJointMeta(joint_name).child_link;
}

const std::string &
UrdfAnalyzer::GetJointParentLink(std::string_view joint_name) const {
  return GetJointMeta(joint_name).parent_link;
}

// ── 폐쇄 체인 감지 ─────────────────────────────────────────────────────────
std::vector<ClosedChainInfo> UrdfAnalyzer::DetectClosedChains() const {
  std::vector<ClosedChainInfo> result;

  // (1) 커스텀 <loop_joint> 태그 파싱
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_xml_string_.c_str());
  auto *robot = doc.FirstChildElement("robot");
  if (robot) {
    for (auto *loop_el = robot->FirstChildElement("loop_joint");
         loop_el != nullptr;
         loop_el = loop_el->NextSiblingElement("loop_joint")) {
      ClosedChainInfo cc;
      const char *name = loop_el->Attribute("name");
      cc.name = name ? name : "loop";
      const char *la = loop_el->Attribute("link1");
      const char *lb = loop_el->Attribute("link2");
      if (la)
        cc.link_a = la;
      if (lb)
        cc.link_b = lb;

      const char *contact = loop_el->Attribute("type");
      if (contact && std::string(contact) == "3D") {
        cc.is_6d = false;
      }
      result.push_back(std::move(cc));
    }
  }

  // (2) Heuristic: joint 이름에 "loop" 포함 + type == fixed
  for (const auto &[name, meta] : joint_meta_map_) {
    if (meta.type == UrdfJointType::kFixed &&
        name.find("loop") != std::string::npos) {
      // 이미 <loop_joint>로 등록된 것과 중복 검사
      bool duplicate = false;
      for (const auto &existing : result) {
        if (existing.link_a == meta.parent_link &&
            existing.link_b == meta.child_link) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        ClosedChainInfo cc;
        cc.name = name;
        cc.link_a = meta.parent_link;
        cc.link_b = meta.child_link;
        cc.is_6d = true;
        result.push_back(std::move(cc));
      }
    }
  }

  return result;
}

// ── LCA 알고리즘 (동시 상승법) ──────────────────────────────────────────────
int UrdfAnalyzer::FindLCA(std::string_view link_a,
                          std::string_view link_b) const {
  int a = GetLinkIndex(link_a);
  int b = GetLinkIndex(link_b);

  int da = depth_[static_cast<std::size_t>(a)];
  int db = depth_[static_cast<std::size_t>(b)];

  // 깊이 맞추기: 더 깊은 쪽을 위로 올림
  while (da > db) {
    a = link_nodes_[static_cast<std::size_t>(a)].parent_index;
    --da;
  }
  while (db > da) {
    b = link_nodes_[static_cast<std::size_t>(b)].parent_index;
    --db;
  }

  // 같은 깊이에서 동시에 올라감
  while (a != b) {
    a = link_nodes_[static_cast<std::size_t>(a)].parent_index;
    b = link_nodes_[static_cast<std::size_t>(b)].parent_index;
  }

  return a;
}

// ── 경로 탐색 (LCA 경유) ───────────────────────────────────────────────────
std::vector<int> UrdfAnalyzer::FindPath(std::string_view link_a,
                                        std::string_view link_b) const {
  int a = GetLinkIndex(link_a);
  int b = GetLinkIndex(link_b);
  int lca = FindLCA(link_a, link_b);

  // a → LCA 경로
  std::vector<int> path_a;
  int cur = a;
  while (cur != lca) {
    path_a.push_back(cur);
    cur = link_nodes_[static_cast<std::size_t>(cur)].parent_index;
  }
  path_a.push_back(lca);

  // b → LCA 경로 (역순)
  std::vector<int> path_b;
  cur = b;
  while (cur != lca) {
    path_b.push_back(cur);
    cur = link_nodes_[static_cast<std::size_t>(cur)].parent_index;
  }

  // 합치기: a→LCA + LCA→b (path_b 역순)
  std::vector<int> result = std::move(path_a);
  for (auto it = path_b.rbegin(); it != path_b.rend(); ++it) {
    result.push_back(*it);
  }

  return result;
}

// ── URDF 원본 접근 ──────────────────────────────────────────────────────────
const std::string &UrdfAnalyzer::GetUrdfXmlString() const noexcept {
  return urdf_xml_string_;
}

const std::string &UrdfAnalyzer::GetUrdfFilePath() const noexcept {
  return urdf_file_path_;
}

} // namespace rtc_urdf_bridge
