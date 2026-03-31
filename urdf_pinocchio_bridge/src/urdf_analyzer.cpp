// ── UrdfAnalyzer 구현 ────────────────────────────────────────────────────────
#include "urdf_pinocchio_bridge/urdf_analyzer.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace urdf_pinocchio_bridge
{

// ── 생성자 (파일 경로) ───────────────────────────────────────────────────────
UrdfAnalyzer::UrdfAnalyzer(std::string_view urdf_file_path)
: urdf_file_path_(urdf_file_path)
{
  // 파일 읽기
  std::ifstream ifs(urdf_file_path_);
  if (!ifs.is_open()) {
    throw std::runtime_error(
      "UrdfAnalyzer: URDF 파일을 열 수 없습니다: " + urdf_file_path_);
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  urdf_xml_string_ = oss.str();

  ParseUrdfXml(urdf_xml_string_);
}

// ── 생성자 (XML 문자열) ─────────────────────────────────────────────────────
UrdfAnalyzer::UrdfAnalyzer(std::string_view xml_string, FromXmlTag)
: urdf_xml_string_(xml_string)
{
  ParseUrdfXml(urdf_xml_string_);
}

// ── URDF XML 파싱 메인 로직 ─────────────────────────────────────────────────
void UrdfAnalyzer::ParseUrdfXml(const std::string & xml)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error(
      "UrdfAnalyzer: XML 파싱 실패 — " + std::string(doc.ErrorStr()));
  }

  auto * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("UrdfAnalyzer: <robot> 루트 요소가 없습니다");
  }

  // ── (1) 모든 <link> 수집 ──────────────────────────────────────────────────
  for (auto * link_el = robot->FirstChildElement("link");
       link_el != nullptr;
       link_el = link_el->NextSiblingElement("link"))
  {
    const char * name_attr = link_el->Attribute("name");
    if (!name_attr) continue;

    auto idx = static_cast<int>(link_nodes_.size());
    link_name_to_index_[name_attr] = idx;

    LinkNode node;
    node.link_name = name_attr;
    link_nodes_.push_back(std::move(node));
  }

  // ── (2) 모든 <joint> 수집 ─────────────────────────────────────────────────
  for (auto * joint_el = robot->FirstChildElement("joint");
       joint_el != nullptr;
       joint_el = joint_el->NextSiblingElement("joint"))
  {
    const char * name_attr = joint_el->Attribute("name");
    const char * type_attr = joint_el->Attribute("type");
    if (!name_attr || !type_attr) continue;

    JointMeta meta;
    meta.joint_name = name_attr;
    meta.type = StringToJointType(type_attr);

    // parent / child link
    auto * parent_el = joint_el->FirstChildElement("parent");
    auto * child_el = joint_el->FirstChildElement("child");
    if (!parent_el || !child_el) continue;

    const char * parent_link = parent_el->Attribute("link");
    const char * child_link = child_el->Attribute("link");
    if (!parent_link || !child_link) continue;

    meta.parent_link = parent_link;
    meta.child_link = child_link;

    // axis (기본값: Z축)
    if (auto * axis_el = joint_el->FirstChildElement("axis")) {
      const char * xyz = axis_el->Attribute("xyz");
      if (xyz) {
        std::istringstream iss(xyz);
        iss >> meta.axis[0] >> meta.axis[1] >> meta.axis[2];
      }
    }

    // limits
    if (auto * limit_el = joint_el->FirstChildElement("limit")) {
      limit_el->QueryDoubleAttribute("lower", &meta.lower);
      limit_el->QueryDoubleAttribute("upper", &meta.upper);
      limit_el->QueryDoubleAttribute("effort", &meta.effort);
      limit_el->QueryDoubleAttribute("velocity", &meta.velocity);
    }

    // dynamics
    if (auto * dyn_el = joint_el->FirstChildElement("dynamics")) {
      dyn_el->QueryDoubleAttribute("damping", &meta.damping);
      dyn_el->QueryDoubleAttribute("friction", &meta.friction);
    }

    // mimic 태그 감지
    if (auto * mimic_el = joint_el->FirstChildElement("mimic")) {
      const char * mimic_joint = mimic_el->Attribute("joint");
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

  // ── (3) <transmission> 태그에서 actuated 관절 이름 수집 ───────────────────
  for (auto * trans_el = robot->FirstChildElement("transmission");
       trans_el != nullptr;
       trans_el = trans_el->NextSiblingElement("transmission"))
  {
    // URDF transmission 스펙: <joint name="..."> 자식 요소
    for (auto * jt_el = trans_el->FirstChildElement("joint");
         jt_el != nullptr;
         jt_el = jt_el->NextSiblingElement("joint"))
    {
      const char * jname = jt_el->Attribute("name");
      if (jname) {
        transmitted_joints_.insert(jname);
      }
    }
  }

  // ── (4) 그래프 구축 및 분류 ───────────────────────────────────────────────
  BuildAdjacencyGraph();
  ClassifyJoints();
  DetectPassiveJoints();
  ComputeDepths();
}

// ── 인접 그래프 구축 ────────────────────────────────────────────────────────
void UrdfAnalyzer::BuildAdjacencyGraph()
{
  // 자식으로 참조되는 링크 집합 (루트 감지용)
  std::unordered_set<std::string> child_links;

  for (const auto & [name, meta] : joint_meta_map_) {
    auto parent_it = link_name_to_index_.find(meta.parent_link);
    auto child_it = link_name_to_index_.find(meta.child_link);
    if (parent_it == link_name_to_index_.end() ||
        child_it == link_name_to_index_.end())
    {
      continue;
    }

    int parent_idx = parent_it->second;
    int child_idx = child_it->second;

    link_nodes_[static_cast<std::size_t>(child_idx)].parent_index = parent_idx;
    link_nodes_[static_cast<std::size_t>(child_idx)].parent_joint_name = name;
    link_nodes_[static_cast<std::size_t>(child_idx)].parent_joint_type = meta.type;
    link_nodes_[static_cast<std::size_t>(parent_idx)].child_indices.push_back(child_idx);

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

// ── 관절 타입별 분류 ────────────────────────────────────────────────────────
void UrdfAnalyzer::ClassifyJoints()
{
  for (const auto & [name, meta] : joint_meta_map_) {
    switch (meta.type) {
      case UrdfJointType::kFixed:
        fixed_joint_names_.push_back(name);
        break;
      case UrdfJointType::kRevolute:
      case UrdfJointType::kContinuous:
      case UrdfJointType::kPrismatic:
      case UrdfJointType::kFloating:
      case UrdfJointType::kPlanar:
        actuated_joint_names_.push_back(name);
        break;
    }
  }

  // 안정적인 순서를 위해 정렬 (URDF 파싱 순서가 아닌 이름 순)
  std::sort(actuated_joint_names_.begin(), actuated_joint_names_.end());
  std::sort(fixed_joint_names_.begin(), fixed_joint_names_.end());
}

// ── 패시브 관절 감지 ────────────────────────────────────────────────────────
void UrdfAnalyzer::DetectPassiveJoints()
{
  // mimic 관절 이름 집합
  std::unordered_set<std::string> mimic_names;
  for (const auto & mi : mimic_joints_) {
    mimic_names.insert(mi.joint_name);
  }

  // transmission에 매핑되지 않은 non-fixed 관절 → passive
  for (const auto & jname : actuated_joint_names_) {
    // mimic 관절은 별도 분류
    if (mimic_names.count(jname) > 0) continue;

    // transmission 있으면 actuated
    if (transmitted_joints_.count(jname) > 0) continue;

    // transmission 없고 mimic도 아닌 non-fixed → passive
    const auto & meta = joint_meta_map_.at(jname);
    PassiveJointInfo pi;
    pi.joint_name = jname;
    pi.type = meta.type;
    pi.damping = meta.damping;
    pi.friction = meta.friction;
    passive_joints_.push_back(std::move(pi));
  }
}

// ── 깊이 테이블 계산 (BFS) ──────────────────────────────────────────────────
void UrdfAnalyzer::ComputeDepths()
{
  depth_.assign(link_nodes_.size(), -1);
  if (root_index_ < 0) return;

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
const std::vector<LinkNode> & UrdfAnalyzer::GetLinkNodes() const noexcept
{
  return link_nodes_;
}

int UrdfAnalyzer::GetLinkIndex(std::string_view link_name) const
{
  auto it = link_name_to_index_.find(std::string(link_name));
  if (it == link_name_to_index_.end()) {
    throw std::out_of_range(
      "UrdfAnalyzer: 링크를 찾을 수 없습니다: " + std::string(link_name));
  }
  return it->second;
}

int UrdfAnalyzer::GetRootIndex() const noexcept
{
  return root_index_;
}

const std::string & UrdfAnalyzer::GetRootLinkName() const noexcept
{
  return link_nodes_[static_cast<std::size_t>(root_index_)].link_name;
}

std::size_t UrdfAnalyzer::GetNumLinks() const noexcept
{
  return link_nodes_.size();
}

std::size_t UrdfAnalyzer::GetNumJoints() const noexcept
{
  return joint_meta_map_.size();
}

// ── 관절 분류 접근 ──────────────────────────────────────────────────────────
const std::vector<std::string> & UrdfAnalyzer::GetActuatedJointNames() const noexcept
{
  return actuated_joint_names_;
}

const std::vector<std::string> & UrdfAnalyzer::GetFixedJointNames() const noexcept
{
  return fixed_joint_names_;
}

const std::vector<PassiveJointInfo> & UrdfAnalyzer::GetPassiveJoints() const noexcept
{
  return passive_joints_;
}

const std::vector<MimicJointInfo> & UrdfAnalyzer::GetMimicJoints() const noexcept
{
  return mimic_joints_;
}

UrdfJointType UrdfAnalyzer::GetJointType(std::string_view joint_name) const
{
  auto it = joint_meta_map_.find(std::string(joint_name));
  if (it == joint_meta_map_.end()) {
    throw std::out_of_range(
      "UrdfAnalyzer: 관절을 찾을 수 없습니다: " + std::string(joint_name));
  }
  return it->second.type;
}

const JointMeta & UrdfAnalyzer::GetJointMeta(std::string_view joint_name) const
{
  auto it = joint_meta_map_.find(std::string(joint_name));
  if (it == joint_meta_map_.end()) {
    throw std::out_of_range(
      "UrdfAnalyzer: 관절을 찾을 수 없습니다: " + std::string(joint_name));
  }
  return it->second;
}

const std::string & UrdfAnalyzer::GetJointChildLink(std::string_view joint_name) const
{
  return GetJointMeta(joint_name).child_link;
}

const std::string & UrdfAnalyzer::GetJointParentLink(std::string_view joint_name) const
{
  return GetJointMeta(joint_name).parent_link;
}

// ── 폐쇄 체인 감지 ─────────────────────────────────────────────────────────
std::vector<ClosedChainInfo> UrdfAnalyzer::DetectClosedChains() const
{
  std::vector<ClosedChainInfo> result;

  // (1) 커스텀 <loop_joint> 태그 파싱
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_xml_string_.c_str());
  auto * robot = doc.FirstChildElement("robot");
  if (robot) {
    for (auto * loop_el = robot->FirstChildElement("loop_joint");
         loop_el != nullptr;
         loop_el = loop_el->NextSiblingElement("loop_joint"))
    {
      ClosedChainInfo cc;
      const char * name = loop_el->Attribute("name");
      cc.name = name ? name : "loop";
      const char * la = loop_el->Attribute("link1");
      const char * lb = loop_el->Attribute("link2");
      if (la) cc.link_a = la;
      if (lb) cc.link_b = lb;

      const char * contact = loop_el->Attribute("type");
      if (contact && std::string(contact) == "3D") {
        cc.is_6d = false;
      }
      result.push_back(std::move(cc));
    }
  }

  // (2) Heuristic: joint 이름에 "loop" 포함 + type == fixed
  for (const auto & [name, meta] : joint_meta_map_) {
    if (meta.type == UrdfJointType::kFixed &&
        name.find("loop") != std::string::npos)
    {
      // 이미 <loop_joint>로 등록된 것과 중복 검사
      bool duplicate = false;
      for (const auto & existing : result) {
        if (existing.link_a == meta.parent_link && existing.link_b == meta.child_link) {
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
int UrdfAnalyzer::FindLCA(std::string_view link_a, std::string_view link_b) const
{
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
std::vector<int> UrdfAnalyzer::FindPath(
  std::string_view link_a, std::string_view link_b) const
{
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
const std::string & UrdfAnalyzer::GetUrdfXmlString() const noexcept
{
  return urdf_xml_string_;
}

const std::string & UrdfAnalyzer::GetUrdfFilePath() const noexcept
{
  return urdf_file_path_;
}

}  // namespace urdf_pinocchio_bridge
