#include "rtc_controllers/params/policy_io_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rtc::params {

namespace {

/// Throw with a uniform prefix so a bring-up log line names the schema that
/// refused, not just the key. Every rejection in this file goes through here.
///
/// Variadic + `append` rather than `operator+` chains: the diagnostics are
/// assembled from a literal, a `std::to_string`, and a config-supplied name, and
/// the chained form builds a temporary per `+`. Nothing here is hot — it runs
/// once and then throws — but the concatenation lint fires on the chain and the
/// append form reads no worse.
template <typename... Parts>
[[noreturn]] void Reject(Parts&&... parts) {
  std::string msg = "policy_io: ";
  (msg.append(std::forward<Parts>(parts)), ...);
  throw std::invalid_argument(msg);
}

/// Read a `[a, b, ...]` shape, refusing anything that is not a non-empty
/// sequence of strictly positive dimensions.
///
/// A zero or negative dimension is refused rather than clamped because the
/// element count is a product: a single 0 collapses the whole tensor to zero
/// elements, and the feature-sum check below would then demand that the feature
/// list be empty too — a second, more confusing failure for the same typo.
std::vector<std::int64_t> ParseShape(const YAML::Node& node, const std::string& where) {
  if (!node || !node.IsSequence() || node.size() == 0) {
    Reject(where, " must be a non-empty sequence of positive dimensions");
  }
  std::vector<std::int64_t> shape;
  shape.reserve(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    const auto dim = node[i].as<std::int64_t>(0);
    if (dim <= 0) {
      Reject(where, "[", std::to_string(i), "] must be > 0 (got ", std::to_string(dim), ")");
    }
    shape.push_back(dim);
  }
  return shape;
}

std::size_t Numel(const std::vector<std::int64_t>& shape) noexcept {
  std::size_t n = 1;
  for (const auto dim : shape) {
    n *= static_cast<std::size_t>(dim);
  }
  return n;
}

/// Read an optional affine lane. Absent or `[]` means identity, which is how a
/// model trained on raw units is configured; anything present must cover the
/// whole tensor, because a partial lane would normalise a prefix of the
/// observation and leave the rest raw — finite, plausible, and wrong.
std::vector<float> ParseAffineLane(const YAML::Node& node, std::size_t numel,
                                   const std::string& where) {
  if (!node || node.IsNull()) {
    return {};
  }
  if (!node.IsSequence()) {
    Reject(where, " must be a sequence (omit the key for identity)");
  }
  if (node.size() == 0) {
    return {};
  }
  if (node.size() != numel) {
    Reject(where, " has ", std::to_string(node.size()), " entries but the input tensor has ",
           std::to_string(numel), " elements (use [] or omit for identity)");
  }
  std::vector<float> lane;
  lane.reserve(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    const auto v = node[i].as<float>(std::numeric_limits<float>::quiet_NaN());
    if (!std::isfinite(v)) {
      Reject(where, "[", std::to_string(i), "] must be finite");
    }
    lane.push_back(v);
  }
  return lane;
}

}  // namespace

std::size_t PolicyIoParams::InputNumel() const noexcept {
  return Numel(input_shape);
}

PolicyIoParams ParsePolicyIoParams(const YAML::Node& cfg, const FeatureSizeFn& feature_size) {
  if (!cfg || !cfg.IsMap()) {
    Reject("schema node is missing or not a map");
  }
  if (!feature_size) {
    Reject("no feature-size resolver supplied (binding bug, not a config error)");
  }

  PolicyIoParams out;

  // ── Shapes ────────────────────────────────────────────────────────────────
  out.input_shape = ParseShape(cfg["input_shape"], "input_shape");

  const YAML::Node heads = cfg["output_shapes"];
  if (!heads || !heads.IsSequence() || heads.size() == 0) {
    Reject("output_shapes must be a non-empty sequence of shapes (one per head)");
  }
  out.output_shapes.reserve(heads.size());
  for (std::size_t h = 0; h < heads.size(); ++h) {
    std::string where = "output_shapes[";
    where.append(std::to_string(h)).append("]");
    out.output_shapes.push_back(ParseShape(heads[h], where));
  }

  const std::size_t in_numel = out.InputNumel();

  // ── Input features → segments ─────────────────────────────────────────────
  // Offsets are a prefix sum over the YAML order. This is the ONLY place the
  // order becomes addresses, which is what makes "reorder the list, the tensor
  // reorders" a property a test can pin rather than an emergent behaviour.
  const YAML::Node features = cfg["input_features"];
  if (!features || !features.IsSequence() || features.size() == 0) {
    Reject("input_features must be a non-empty sequence of feature ids");
  }
  int cursor = 0;
  out.input_features.reserve(features.size());
  out.input_segments.reserve(features.size());
  for (std::size_t i = 0; i < features.size(); ++i) {
    const std::string idx = std::to_string(i);
    auto id = features[i].as<std::string>("");
    if (id.empty()) {
      Reject("input_features[", idx, "] is empty");
    }
    if (std::find(out.input_features.begin(), out.input_features.end(), id) !=
        out.input_features.end()) {
      Reject("input_features[", idx, "] repeats id '", id, "'");
    }
    const int count = feature_size(id);
    if (count <= 0) {
      Reject("input_features[", idx, "] has unknown id '", id, "'");
    }
    out.input_segments.push_back({cursor, count});
    out.input_features.push_back(std::move(id));
    cursor += count;
  }
  if (static_cast<std::size_t>(cursor) != in_numel) {
    Reject("input_features sum to ", std::to_string(cursor), " elements but input_shape declares ",
           std::to_string(in_numel));
  }

  // ── Affine normalisation ──────────────────────────────────────────────────
  out.input_offset = ParseAffineLane(cfg["input_offset"], in_numel, "input_offset");
  out.input_scale = ParseAffineLane(cfg["input_scale"], in_numel, "input_scale");

  // ── Output features → slices ──────────────────────────────────────────────
  const YAML::Node outs = cfg["output_features"];
  if (!outs || !outs.IsSequence() || outs.size() == 0) {
    Reject("output_features must be a non-empty sequence of {name, head, offset, count}");
  }
  out.output_names.reserve(outs.size());
  out.output_slices.reserve(outs.size());
  for (std::size_t i = 0; i < outs.size(); ++i) {
    std::string at = "output_features[";
    at.append(std::to_string(i)).append("]");
    const YAML::Node entry = outs[i];
    if (!entry || !entry.IsMap()) {
      Reject(at, " must be a map with keys {name, head, offset, count}");
    }
    auto name = entry["name"].as<std::string>("");
    if (name.empty()) {
      Reject(at, " is missing 'name'");
    }
    if (std::find(out.output_names.begin(), out.output_names.end(), name) !=
        out.output_names.end()) {
      Reject(at, " repeats name '", name, "'");
    }
    const int head = entry["head"].as<int>(-1);
    const int offset = entry["offset"].as<int>(-1);
    const int count = entry["count"].as<int>(0);
    if (head < 0 || static_cast<std::size_t>(head) >= out.output_shapes.size()) {
      Reject(at, " names head ", std::to_string(head), " but the model declares ",
             std::to_string(out.output_shapes.size()), " head(s)");
    }
    if (offset < 0) {
      Reject(at, " must declare offset >= 0");
    }
    if (count <= 0) {
      Reject(at, " must declare count > 0");
    }
    const std::size_t head_numel = Numel(out.output_shapes[static_cast<std::size_t>(head)]);
    if (static_cast<std::size_t>(offset) + static_cast<std::size_t>(count) > head_numel) {
      Reject(at, " slices [", std::to_string(offset), ", ", std::to_string(offset + count),
             ") past head ", std::to_string(head), "'s ", std::to_string(head_numel), " elements");
    }
    // Overlap is a per-head question: two heads starting at 0 is normal, two
    // slices of ONE head sharing an element means at least one is not reading
    // what its name claims.
    for (std::size_t p = 0; p < out.output_slices.size(); ++p) {
      const auto& prev = out.output_slices[p];
      if (prev.head != head) {
        continue;
      }
      if (offset < prev.offset + prev.count && prev.offset < offset + count) {
        Reject(at, " overlaps output_features[", std::to_string(p), "] ('", out.output_names[p],
               "') on head ", std::to_string(head));
      }
    }
    out.output_slices.push_back({head, offset, count});
    out.output_names.push_back(std::move(name));
  }

  // ── Decimation ────────────────────────────────────────────────────────────
  out.decimation = cfg["decimation"].as<int>(1);
  if (out.decimation < 1) {
    Reject("decimation must be >= 1 (got ", std::to_string(out.decimation), ")");
  }

  return out;
}

}  // namespace rtc::params
