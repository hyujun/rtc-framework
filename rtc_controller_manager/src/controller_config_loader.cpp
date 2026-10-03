#include "rtc_controller_manager/controller_config_loader.hpp"

#include <array>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace rtc {
namespace {

constexpr std::string_view kIncludeKey = "include";

// Message parts are appended one by one — these run inside the merge loops.
template <typename... Parts>
[[noreturn]] void Fail(const Parts&... parts) {
  std::string message = "controller config include: ";
  (message.append(parts), ...);
  throw ControllerConfigIncludeError(message);
}

std::string JoinKeyPath(const std::string& parent, const std::string& key) {
  return parent.empty() ? key : parent + "." + key;
}

// Which file first set the node at a key path. Read only to name both files
// in a conflict message — the conflict itself is decided on the nodes.
using Origins = std::map<std::string, std::string>;

void RecordOrigins(const YAML::Node& node, const std::string& path, const std::string& file,
                   Origins& origins) {
  if (!path.empty()) {
    origins.emplace(path, file);
  }
  if (!node.IsMap()) {
    return;
  }
  for (const auto& kv : node) {
    RecordOrigins(kv.second, JoinKeyPath(path, kv.first.Scalar()), file, origins);
  }
}

// Merges the map @p src (from @p src_file) into the map @p dst in place.
// yaml-cpp nodes are handles: `dst` is taken by value and still writes through
// to the composed tree.
void MergeMap(YAML::Node dst, const YAML::Node& src, const std::string& path,
              const std::string& src_file, Origins& origins) {
  // Lookups go through a const view: the non-const operator[] inserts the key.
  const YAML::Node& dst_view = dst;
  for (const auto& kv : src) {
    const std::string key = kv.first.Scalar();
    const std::string child_path = JoinKeyPath(path, key);
    const YAML::Node existing = dst_view[key];
    if (!existing.IsDefined()) {
      dst[key] = kv.second;
      RecordOrigins(kv.second, child_path, src_file, origins);
      continue;
    }
    if (existing.IsMap() && kv.second.IsMap()) {
      MergeMap(existing, kv.second, child_path, src_file, origins);
      continue;
    }
    const auto origin = origins.find(child_path);
    const std::string first_file = origin != origins.end() ? origin->second : "<unknown>";
    if (existing.IsMap() != kv.second.IsMap()) {
      Fail("key '", child_path, "' is a map in one file and a value in the other ('", first_file,
           "' and '", src_file, "')");
    }
    Fail("key '", child_path, "' is set in both '", first_file, "' and '", src_file, "'");
  }
}

std::string ResolveFragmentPath(const std::string& main_path, const std::string& entry) {
  if (entry.empty()) {
    Fail("'", main_path, "' has an empty 'include' entry");
  }
  const std::filesystem::path relative(entry);
  if (relative.is_absolute()) {
    Fail("'", main_path, "' includes '", entry,
         "' — an include path is relative to the including file's directory, not absolute");
  }
  for (const auto& part : relative) {
    if (part == "..") {
      Fail("'", main_path, "' includes '", entry, "' — an include path cannot contain '..'");
    }
  }
  return (std::filesystem::path(main_path).parent_path() / relative).string();
}

YAML::Node LoadFragmentDocument(const std::string& fragment_path, const std::string& main_path) {
  try {
    return YAML::LoadFile(fragment_path);
  } catch (const YAML::BadFile&) {
    Fail("'", main_path, "' includes '", fragment_path, "', which cannot be opened");
  } catch (const YAML::Exception& e) {
    Fail("fragment '", fragment_path, "' (included by '", main_path,
         "') does not parse: ", e.what());
  }
}

// Returns the fragment's `<config_key>` map after checking the fragment holds
// nothing else.
YAML::Node LoadFragment(const std::string& fragment_path, const std::string& main_path,
                        const std::string& config_key) {
  const YAML::Node doc = LoadFragmentDocument(fragment_path, main_path);
  if (!doc.IsMap()) {
    Fail("fragment '", fragment_path, "' must be a map whose only top-level key is '", config_key,
         "'");
  }
  for (const auto& kv : doc) {
    const std::string key = kv.first.Scalar();
    if (key == kIncludeKey) {
      Fail("fragment '", fragment_path, "' has its own 'include' — includes do not nest");
    }
    if (key != config_key) {
      Fail("fragment '", fragment_path, "' has the top-level key '", key,
           "' — a fragment's only top-level key is '", config_key, "'");
    }
  }
  const YAML::Node body = doc[config_key];
  if (!body.IsDefined() || !body.IsMap()) {
    Fail("fragment '", fragment_path, "' has no map under '", config_key, "'");
  }
  return body;
}

// Scalars that YAML reads as null. A reader that keeps scalars as text (the
// Python twin compares in that form) cannot tell a quoted one from a plain one,
// so both sides render the whole set as `null`.
bool IsNullText(const std::string& text) {
  constexpr std::array<std::string_view, 5> kNullTexts = {"", "~", "null", "Null", "NULL"};
  for (const auto candidate : kNullTexts) {
    if (text == candidate) {
      return true;
    }
  }
  return false;
}

std::string RenderInline(const YAML::Node& node) {
  if (node.IsScalar()) {
    return IsNullText(node.Scalar()) ? "null" : node.Scalar();
  }
  if (node.IsSequence()) {
    std::string out = "[";
    bool first = true;
    for (const auto& item : node) {
      out += first ? "" : ", ";
      out += RenderInline(item);
      first = false;
    }
    out += "]";
    return out;
  }
  if (node.IsMap()) {
    std::string out = "{";
    bool first = true;
    for (const auto& kv : node) {
      out += first ? "" : ", ";
      out.append(kv.first.Scalar()).append(": ").append(RenderInline(kv.second));
      first = false;
    }
    out += "}";
    return out;
  }
  return "null";
}

void AppendLeafLines(const YAML::Node& node, const std::string& path,
                     std::vector<std::string>& lines) {
  if (node.IsMap() && node.size() > 0) {
    for (const auto& kv : node) {
      AppendLeafLines(kv.second, JoinKeyPath(path, kv.first.Scalar()), lines);
    }
    return;
  }
  lines.push_back(path + "\t" + RenderInline(node));
}

}  // namespace

YAML::Node LoadControllerConfig(const std::string& yaml_path, const std::string& config_key) {
  YAML::Node file_node = YAML::LoadFile(yaml_path);
  const YAML::Node& file_view = file_node;
  if (!file_node.IsMap() || !file_view[std::string(kIncludeKey)].IsDefined()) {
    return file_node[config_key];
  }

  const YAML::Node includes = file_view[std::string(kIncludeKey)];
  if (!includes.IsSequence()) {
    Fail("'", yaml_path, "': the top-level 'include' must be a list of fragment paths");
  }
  for (const auto& kv : file_view) {
    const std::string key = kv.first.Scalar();
    if (key != kIncludeKey && key != config_key) {
      Fail("'", yaml_path, "' has an 'include' list, so its only other top-level key is '",
           config_key, "' — found '", key, "'");
    }
  }
  YAML::Node composed = file_view[config_key];
  if (!composed.IsDefined() || !composed.IsMap()) {
    Fail("'", yaml_path, "' has an 'include' list but no map under '", config_key, "'");
  }

  Origins origins;
  RecordOrigins(composed, "", yaml_path, origins);
  for (const auto& entry : includes) {
    if (!entry.IsScalar()) {
      Fail("'", yaml_path, "': every 'include' entry must be a path string");
    }
    const std::string fragment_path = ResolveFragmentPath(yaml_path, entry.Scalar());
    const YAML::Node body = LoadFragment(fragment_path, yaml_path, config_key);
    MergeMap(composed, body, "", fragment_path, origins);
  }
  return composed;
}

std::vector<std::string> ControllerConfigLeafLines(const YAML::Node& node) {
  std::vector<std::string> lines;
  if (node.IsMap()) {
    for (const auto& kv : node) {
      AppendLeafLines(kv.second, kv.first.Scalar(), lines);
    }
  }
  return lines;
}

}  // namespace rtc
