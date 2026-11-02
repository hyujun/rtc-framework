// ── CapsuleModel 구현: YAML 파서 + 모델 무관 검증 ─────────────────────────────
#include "rtc_urdf_bridge/collision/capsule_model.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace rtc_urdf_bridge {

namespace {

[[noreturn]] void Fail(const std::string& what) {
  throw std::runtime_error("collision_capsules: " + what);
}

// 맵의 키가 전부 known 에 있는지 — 철자 오류가 조용히 기본값으로 떨어지지 않게 (#642 D12).
void CheckKeys(const YAML::Node& map, std::initializer_list<const char*> known,
               const std::string& where) {
  if (!map.IsMap()) {
    Fail(where + " 는 맵이어야 합니다");
  }
  for (const auto& kv : map) {
    const std::string key = kv.first.as<std::string>();
    bool found = false;
    for (const char* k : known) {
      if (key == k) {
        found = true;
        break;
      }
    }
    if (!found) {
      Fail(where + " 에 알 수 없는 키 '" + key + "'");
    }
  }
}

double ParseFinite(const YAML::Node& node, const std::string& where) {
  double v = 0.0;
  try {
    v = node.as<double>();
  } catch (const YAML::Exception&) {
    Fail(where + " 는 수여야 합니다");
  }
  if (!std::isfinite(v)) {
    Fail(where + " 는 유한해야 합니다");
  }
  return v;
}

Eigen::Vector3d ParseVec3(const YAML::Node& node, const std::string& where) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    Fail(where + " 는 수 3 개의 목록이어야 합니다");
  }
  Eigen::Vector3d v;
  for (std::size_t i = 0; i < 3; ++i) {
    v[static_cast<Eigen::Index>(i)] = ParseFinite(node[i], where + "[" + std::to_string(i) + "]");
  }
  return v;
}

std::string RequireString(const YAML::Node& map, const char* key, const std::string& where) {
  const YAML::Node n = map[key];
  if (!n || !n.IsScalar()) {
    Fail(where + " 에 필수 필드 '" + key + "' 가 없습니다");
  }
  const std::string s = n.as<std::string>();
  if (s.empty()) {
    Fail(where + " 의 '" + key + "' 가 비어 있습니다");
  }
  return s;
}

CapsuleSpec ParseCapsule(const YAML::Node& node, std::size_t index) {
  const std::string where = "capsules[" + std::to_string(index) + "]";
  CheckKeys(node, {"name", "link", "p0", "p1", "radius", "shape"}, where);
  CapsuleSpec c;
  c.name = RequireString(node, "name", where);
  const std::string named = "capsule '" + c.name + "'";
  c.link = RequireString(node, "link", named);
  if (node["shape"]) {
    const std::string shape = node["shape"].as<std::string>();
    if (shape != "capsule") {
      Fail(named + " 의 shape '" + shape + "' 는 지원하지 않습니다 (capsule 만; 구는 p0 == p1)");
    }
  }
  c.p0 = ParseVec3(node["p0"], named + " 의 p0");
  c.p1 = ParseVec3(node["p1"], named + " 의 p1");
  if (!node["radius"]) {
    Fail(named + " 에 필수 필드 'radius' 가 없습니다");
  }
  c.radius = ParseFinite(node["radius"], named + " 의 radius");
  return c;
}

CapsulePairSpec ParsePair(const YAML::Node& node, std::size_t index) {
  const std::string where = "pairs[" + std::to_string(index) + "]";
  if (!node.IsSequence() || node.size() != 2 || !node[0].IsScalar() || !node[1].IsScalar()) {
    Fail(where + " 는 capsule 이름 2 개의 목록이어야 합니다");
  }
  return CapsulePairSpec{node[0].as<std::string>(), node[1].as<std::string>()};
}

}  // namespace

CapsuleModel::CapsuleModel(CapsuleModelSpec spec) : spec_(std::move(spec)) {
  if (spec_.capsules.empty()) {
    Fail("capsules 가 비어 있습니다");
  }
  if (spec_.capsules.size() > kMaxCapsules) {
    Fail("capsules 가 " + std::to_string(spec_.capsules.size()) + " 개 — 상한 " +
         std::to_string(kMaxCapsules));
  }
  if (spec_.pairs.size() > kMaxCapsulePairs) {
    Fail("pairs 가 " + std::to_string(spec_.pairs.size()) + " 개 — 상한 " +
         std::to_string(kMaxCapsulePairs));
  }
  if (spec_.ball_capsules.size() > kMaxBallCapsules) {
    Fail("ball_capsules 가 " + std::to_string(spec_.ball_capsules.size()) + " 개 — 상한 " +
         std::to_string(kMaxBallCapsules));
  }

  std::unordered_map<std::string, std::size_t> by_name;
  for (std::size_t i = 0; i < spec_.capsules.size(); ++i) {
    const CapsuleSpec& c = spec_.capsules[i];
    const std::string named = "capsule '" + c.name + "'";
    if (c.name.empty() || c.link.empty()) {
      Fail("capsules[" + std::to_string(i) + "] 의 name · link 는 비어 있을 수 없습니다");
    }
    if (!by_name.emplace(c.name, i).second) {
      Fail(named + " 가 중복됩니다");
    }
    if (!(c.radius >= 0.0) || !std::isfinite(c.radius)) {
      Fail(named + " 의 radius 가 음수이거나 유한하지 않습니다");
    }
    if (!c.p0.allFinite() || !c.p1.allFinite()) {
      Fail(named + " 의 끝점이 유한하지 않습니다");
    }
  }

  const auto lookup = [&](const std::string& name, const std::string& where) {
    const auto it = by_name.find(name);
    if (it == by_name.end()) {
      Fail(where + " 가 미등록 capsule '" + name + "' 를 참조합니다");
    }
    return it->second;
  };

  std::set<std::pair<std::size_t, std::size_t>> seen_pairs;
  pair_indices_.reserve(spec_.pairs.size());
  for (std::size_t i = 0; i < spec_.pairs.size(); ++i) {
    const CapsulePairSpec& p = spec_.pairs[i];
    const std::string where = "pairs[" + std::to_string(i) + "] (" + p.a + ", " + p.b + ")";
    const std::size_t ia = lookup(p.a, where);
    const std::size_t ib = lookup(p.b, where);
    if (ia == ib) {
      Fail(where + " 는 같은 capsule 의 쌍입니다");
    }
    const auto key = std::minmax(ia, ib);
    if (!seen_pairs.insert(key).second) {
      Fail(where + " 는 중복 쌍입니다 (순서 무관)");
    }
    pair_indices_.emplace_back(ia, ib);
  }

  std::set<std::size_t> seen_ball;
  ball_indices_.reserve(spec_.ball_capsules.size());
  for (std::size_t i = 0; i < spec_.ball_capsules.size(); ++i) {
    const std::size_t ib =
        lookup(spec_.ball_capsules[i], "ball_capsules[" + std::to_string(i) + "]");
    if (!seen_ball.insert(ib).second) {
      Fail("ball_capsules 에 '" + spec_.ball_capsules[i] + "' 가 중복됩니다");
    }
    ball_indices_.push_back(ib);
  }
}

std::optional<std::size_t> CapsuleModel::FindCapsule(std::string_view name) const noexcept {
  for (std::size_t i = 0; i < spec_.capsules.size(); ++i) {
    if (spec_.capsules[i].name == name) {
      return i;
    }
  }
  return std::nullopt;
}

std::shared_ptr<const CapsuleModel> CapsuleModel::ParseYaml(const std::string& yaml_text,
                                                            std::string_view source_label) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    Fail("YAML 파싱 실패 (" + std::string(source_label) + "): " + e.what());
  }
  CheckKeys(root, {"collision_capsules"}, "최상위 (" + std::string(source_label) + ")");
  const YAML::Node top = root["collision_capsules"];
  if (!top) {
    Fail("최상위 키 'collision_capsules' 가 없습니다 (" + std::string(source_label) + ")");
  }
  CheckKeys(top, {"capsules", "pairs", "ball_capsules"}, "collision_capsules");

  CapsuleModelSpec spec;
  const YAML::Node capsules = top["capsules"];
  if (!capsules || !capsules.IsSequence()) {
    Fail("collision_capsules.capsules 는 목록이어야 합니다");
  }
  for (std::size_t i = 0; i < capsules.size(); ++i) {
    spec.capsules.push_back(ParseCapsule(capsules[i], i));
  }
  if (const YAML::Node pairs = top["pairs"]; pairs) {
    if (!pairs.IsSequence()) {
      Fail("collision_capsules.pairs 는 목록이어야 합니다");
    }
    for (std::size_t i = 0; i < pairs.size(); ++i) {
      spec.pairs.push_back(ParsePair(pairs[i], i));
    }
  }
  if (const YAML::Node balls = top["ball_capsules"]; balls) {
    if (!balls.IsSequence()) {
      Fail("collision_capsules.ball_capsules 는 이름 목록이어야 합니다");
    }
    for (const auto& n : balls) {
      if (!n.IsScalar()) {
        Fail("collision_capsules.ball_capsules 의 항목은 capsule 이름이어야 합니다");
      }
      spec.ball_capsules.push_back(n.as<std::string>());
    }
  }
  return std::make_shared<const CapsuleModel>(std::move(spec));
}

std::shared_ptr<const CapsuleModel> CapsuleModel::LoadYaml(std::string_view yaml_path) {
  const std::string path(yaml_path);
  std::ifstream in(path);
  if (!in) {
    Fail("파일을 열 수 없습니다: " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ParseYaml(ss.str(), path);
}

}  // namespace rtc_urdf_bridge
