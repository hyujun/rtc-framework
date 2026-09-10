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

/// "inputs[2]" — the location prefix every per-tensor diagnostic carries.
std::string At(const char* key, std::size_t index) {
  std::string s = key;
  s.append("[").append(std::to_string(index)).append("]");
  return s;
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
    Reject(where, " has ", std::to_string(node.size()), " entries but the tensor has ",
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

/// The tensor `name:` every declaration must carry (#511 D-1).
///
/// Empty is refused rather than defaulted to the position, because an empty
/// name is precisely what asks `rtc::InferenceEngine` for POSITIONAL binding —
/// the mode in which an export that reordered two same-shaped tensors loads
/// cleanly and computes the wrong thing.
std::string ParseTensorName(const YAML::Node& entry, const std::string& where,
                            std::vector<std::string>& seen) {
  auto name = entry["name"].as<std::string>("");
  if (name.empty()) {
    Reject(where,
           " is missing 'name' — the .onnx tensor name is mandatory, because an unnamed "
           "tensor is bound by POSITION and a re-export that swapped two tensors of the "
           "same shape would then load cleanly");
  }
  const auto dup = std::find(seen.begin(), seen.end(), name);
  if (dup != seen.end()) {
    Reject(where, " repeats tensor name '", name, "' (already declared at index ",
           std::to_string(static_cast<std::size_t>(dup - seen.begin())), ")");
  }
  return name;
}

/// Name the migration when a pre-#511 flat schema turns up.
///
/// Without this the old config fails as "inputs must be a non-empty sequence",
/// which is true and useless: the operator's file HAS an input declaration, it
/// just has the shape this parser stopped accepting (#511 D-7 clean break).
void RejectLegacyFlatSchema(const YAML::Node& cfg) {
  static constexpr const char* kGone[] = {"input_shape", "output_shapes", "input_features"};
  for (const char* key : kGone) {
    if (cfg[key]) {
      Reject("'", key,
             "' is the pre-#511 flat schema. Inputs and outputs are now declared as `inputs:` / "
             "`outputs:` lists of {name, shape}, each input carrying its own `features:` and "
             "affine lane, and `output_features` referring to an output by `tensor: <name>` "
             "instead of `head: <index>`");
    }
  }
}

}  // namespace

std::size_t InputTensorSpec::Numel() const noexcept {
  return rtc::params::Numel(shape);
}

std::size_t OutputTensorSpec::Numel() const noexcept {
  return rtc::params::Numel(shape);
}

PolicyIoParams ParsePolicyIoParams(const YAML::Node& cfg, const FeatureSizeFn& feature_size) {
  if (!cfg || !cfg.IsMap()) {
    Reject("schema node is missing or not a map");
  }
  if (!feature_size) {
    Reject("no feature-size resolver supplied (binding bug, not a config error)");
  }
  RejectLegacyFlatSchema(cfg);

  PolicyIoParams out;

  // ── Input tensors ─────────────────────────────────────────────────────────
  // Offsets are a prefix sum over each tensor's OWN feature list. This is the
  // only place YAML order becomes addresses, which is what makes "reorder the
  // list, the tensor reorders" a property a test can pin rather than an
  // emergent behaviour — and keeping the sum per tensor is what stops a feature
  // added to one tensor from shifting every offset in the next.
  const YAML::Node inputs = cfg["inputs"];
  if (!inputs || !inputs.IsSequence() || inputs.size() == 0) {
    Reject("inputs must be a non-empty sequence of {name, shape, features}");
  }

  // Feature ids are unique across the WHOLE policy (#511 D-6): the same id in
  // two tensors is a copy-paste artefact far more often than it is deliberate,
  // so the message has to name both places rather than just the second.
  std::vector<std::string> seen_features;
  std::vector<std::string> seen_feature_where;
  std::vector<std::string> input_names;

  out.inputs.reserve(inputs.size());
  input_names.reserve(inputs.size());
  for (std::size_t t = 0; t < inputs.size(); ++t) {
    const std::string where = At("inputs", t);
    const YAML::Node entry = inputs[t];
    if (!entry || !entry.IsMap()) {
      Reject(where, " must be a map with keys {name, shape, features}");
    }
    InputTensorSpec spec;
    spec.name = ParseTensorName(entry, where, input_names);
    input_names.push_back(spec.name);

    std::string shape_where = where;
    shape_where.append(".shape");
    spec.shape = ParseShape(entry["shape"], shape_where);
    const std::size_t numel = spec.Numel();

    // Exactly one filler. A tensor with both would have its features overwritten
    // by the feedback every step after the first — the observation would simply
    // stop arriving, and the policy would keep producing finite actions from a
    // hidden state and nothing else. A tensor with neither is never written at
    // all, so it reads whatever the engine's allocation held.
    const YAML::Node source = entry["source"];
    const YAML::Node features = entry["features"];
    const bool has_features = features && features.IsSequence() && features.size() > 0;
    if (source) {
      const auto kind = source.as<std::string>("");
      if (kind != "recurrent") {
        Reject(where, " ('", spec.name, "') declares source '", kind,
               "' — the only source other than observation features is \"recurrent\"");
      }
      spec.recurrent = true;
    }
    if (spec.recurrent == has_features) {
      Reject(where, " ('", spec.name,
             "') must declare EITHER a non-empty `features:` sequence OR `source: recurrent`, "
             "not both and not neither");
    }
    if (spec.recurrent) {
      if (entry["offset"] || entry["scale"]) {
        // Normalisation constants come from the statistics of an observed
        // quantity. A hidden state is the policy's own representation; there is
        // no training-time distribution to centre it against.
        Reject(where, " ('", spec.name,
               "') is recurrent and cannot carry an affine lane — `offset`/`scale` normalise "
               "observations, and a policy's internal state is not one");
      }
      out.inputs.push_back(std::move(spec));
      continue;
    }
    int cursor = 0;
    spec.features.reserve(features.size());
    spec.segments.reserve(features.size());
    for (std::size_t i = 0; i < features.size(); ++i) {
      std::string at = where;
      at.append(".features[").append(std::to_string(i)).append("]");
      auto id = features[i].as<std::string>("");
      if (id.empty()) {
        Reject(at, " is empty");
      }
      const auto dup = std::find(seen_features.begin(), seen_features.end(), id);
      if (dup != seen_features.end()) {
        Reject(at, " repeats id '", id, "', already declared at ",
               seen_feature_where[static_cast<std::size_t>(dup - seen_features.begin())]);
      }
      const int count = feature_size(id);
      if (count <= 0) {
        Reject(at, " has unknown id '", id, "'");
      }
      spec.segments.push_back({static_cast<int>(t), cursor, count});
      spec.features.push_back(id);
      seen_features.push_back(std::move(id));
      seen_feature_where.push_back(at);
      cursor += count;
    }
    if (static_cast<std::size_t>(cursor) != numel) {
      Reject(where, " ('", spec.name, "') features sum to ", std::to_string(cursor),
             " elements but its shape declares ", std::to_string(numel));
    }

    std::string offset_where = where;
    offset_where.append(".offset");
    std::string scale_where = where;
    scale_where.append(".scale");
    spec.offset = ParseAffineLane(entry["offset"], numel, offset_where);
    spec.scale = ParseAffineLane(entry["scale"], numel, scale_where);

    out.inputs.push_back(std::move(spec));
  }

  // ── Output tensors ────────────────────────────────────────────────────────
  const YAML::Node outputs = cfg["outputs"];
  if (!outputs || !outputs.IsSequence() || outputs.size() == 0) {
    Reject("outputs must be a non-empty sequence of {name, shape}");
  }
  std::vector<std::string> output_tensor_names;
  out.outputs.reserve(outputs.size());
  output_tensor_names.reserve(outputs.size());
  for (std::size_t t = 0; t < outputs.size(); ++t) {
    const std::string where = At("outputs", t);
    const YAML::Node entry = outputs[t];
    if (!entry || !entry.IsMap()) {
      Reject(where, " must be a map with keys {name, shape}");
    }
    OutputTensorSpec spec;
    spec.name = ParseTensorName(entry, where, output_tensor_names);
    output_tensor_names.push_back(spec.name);

    std::string shape_where = where;
    shape_where.append(".shape");
    spec.shape = ParseShape(entry["shape"], shape_where);
    spec.feeds = entry["feeds"].as<std::string>("");
    out.outputs.push_back(std::move(spec));
  }

  // ── Recurrent links ───────────────────────────────────────────────────────
  // Resolved after BOTH sides are known, because a link is a statement about a
  // pair. Whole-tensor by D-2.
  for (std::size_t t = 0; t < out.outputs.size(); ++t) {
    const auto& src = out.outputs[t];
    if (src.feeds.empty()) {
      continue;
    }
    const std::string where = At("outputs", t);
    std::size_t target = out.inputs.size();
    for (std::size_t i = 0; i < out.inputs.size(); ++i) {
      if (out.inputs[i].name == src.feeds) {
        target = i;
        break;
      }
    }
    if (target == out.inputs.size()) {
      Reject(where, " ('", src.name, "') feeds '", src.feeds,
             "' but no such input tensor is declared");
    }
    const auto& dst = out.inputs[target];
    if (!dst.recurrent) {
      Reject(where, " ('", src.name, "') feeds input '", dst.name,
             "', which is filled by observation features — writing the previous step's output "
             "over it would replace this tick's observation with the policy's own last answer");
    }
    if (src.Numel() != dst.Numel()) {
      Reject(where, " ('", src.name, "') has ", std::to_string(src.Numel()),
             " elements but feeds input '", dst.name, "', which has ", std::to_string(dst.Numel()));
    }
    for (const auto& link : out.recurrent_links) {
      if (static_cast<std::size_t>(link.input_tensor) == target) {
        Reject(where, " ('", src.name, "') feeds input '", dst.name, "', which output '",
               out.outputs[static_cast<std::size_t>(link.output_tensor)].name,
               "' already feeds — the state would be whichever copy ran last");
      }
    }
    out.recurrent_links.push_back({static_cast<int>(t), static_cast<int>(target), src.Numel()});
  }
  // A recurrent input nothing feeds is never written after the initial zero, so
  // the policy reads a constant zero state forever while looking exactly like a
  // working recurrent policy.
  for (std::size_t i = 0; i < out.inputs.size(); ++i) {
    if (!out.inputs[i].recurrent) {
      continue;
    }
    const bool fed = std::any_of(out.recurrent_links.begin(), out.recurrent_links.end(),
                                 [i](const rtc::inference::RecurrentLink& l) {
                                   return static_cast<std::size_t>(l.input_tensor) == i;
                                 });
    if (!fed) {
      Reject(At("inputs", i), " ('", out.inputs[i].name,
             "') is recurrent but no output declares `feeds: \"", out.inputs[i].name,
             "\"` — nothing would ever write it");
    }
  }

  // ── Output features → slices ──────────────────────────────────────────────
  const YAML::Node outs = cfg["output_features"];
  if (!outs || !outs.IsSequence() || outs.size() == 0) {
    Reject(
        "output_features must be a non-empty sequence of {tensor, role, device, offset?, "
        "count?}");
  }
  out.output_features.reserve(outs.size());
  for (std::size_t i = 0; i < outs.size(); ++i) {
    const std::string at = At("output_features", i);
    const YAML::Node entry = outs[i];
    if (!entry || !entry.IsMap()) {
      Reject(at, " must be a map with keys {tensor, role, device, offset?, count?}");
    }

    // Declared, never inferred (#511 D-4). The binding this replaces read the
    // role off a name suffix, which made two devices declaring the same role —
    // the natural shape of a multi-output policy — resolve to whichever came
    // last, with both commands still finite and inside the joint limits.
    OutputCommandSpec spec;
    spec.device = entry["device"].as<std::string>("");
    spec.role = entry["role"].as<std::string>("");
    if (spec.device.empty()) {
      Reject(at, " must declare the device group it drives (`device: <name>`)");
    }
    if (spec.role.empty()) {
      Reject(at, " ('", spec.device, "') must declare what the slice means (`role: <name>`)");
    }
    for (std::size_t p = 0; p < out.output_features.size(); ++p) {
      if (out.output_features[p].device == spec.device &&
          out.output_features[p].role == spec.role) {
        Reject(at, " repeats ", spec.device, "/", spec.role, ", already declared at ",
               At("output_features", p),
               " — one device cannot be driven twice in the same role, and silently keeping one "
               "of the two is how the pre-#511 binding lost an arm command to a hand command");
      }
    }
    const std::string label = spec.device + "/" + spec.role;

    // By NAME, not by index. `head: 0` would reintroduce exactly the positional
    // reference that #511 D-1 removed from the tensor declarations themselves.
    const auto tensor_name = entry["tensor"].as<std::string>("");
    if (tensor_name.empty()) {
      Reject(at, " (", label, ") must name the output tensor it slices (`tensor: <name>`)");
    }
    const auto found =
        std::find(output_tensor_names.begin(), output_tensor_names.end(), tensor_name);
    if (found == output_tensor_names.end()) {
      Reject(at, " (", label, ") names output tensor '", tensor_name,
             "' but no such tensor is declared under `outputs:`");
    }
    const auto tensor = static_cast<int>(found - output_tensor_names.begin());
    const auto& tensor_spec = out.outputs[static_cast<std::size_t>(tensor)];
    if (!tensor_spec.feeds.empty()) {
      Reject(at, " (", label, ") slices output tensor '", tensor_name,
             "', which feeds recurrent input '", tensor_spec.feeds,
             "' — that tensor is the policy's internal state, and reading joint targets out of "
             "it would be interpreting hidden units as radians");
    }
    const std::size_t tensor_numel = tensor_spec.Numel();

    // Both keys are optional and default to "the whole tensor", because a
    // policy whose tensor IS one command is the common case and spelling out
    // `offset: 0, count: 6` there is a second place for the width to drift
    // from the shape above it.
    const int offset = entry["offset"].as<int>(0);
    const int count =
        entry["count"] ? entry["count"].as<int>(0) : static_cast<int>(tensor_numel) - offset;
    if (offset < 0) {
      Reject(at, " must declare offset >= 0");
    }
    if (count <= 0) {
      Reject(at, " must declare count > 0");
    }
    if (static_cast<std::size_t>(offset) + static_cast<std::size_t>(count) > tensor_numel) {
      Reject(at, " slices [", std::to_string(offset), ", ", std::to_string(offset + count),
             ") past tensor '", tensor_name, "'s ", std::to_string(tensor_numel), " elements");
    }
    // Overlap is a per-tensor question: two tensors starting at 0 is normal,
    // two slices of ONE tensor sharing an element means at least one is not
    // reading what its name claims.
    for (std::size_t p = 0; p < out.output_features.size(); ++p) {
      const auto& prev = out.output_features[p].slice;
      if (prev.tensor != tensor) {
        continue;
      }
      if (offset < prev.offset + prev.count && prev.offset < offset + count) {
        Reject(at, " (", label, ") overlaps output_features[", std::to_string(p), "] (",
               out.output_features[p].device, "/", out.output_features[p].role, ") on tensor '",
               tensor_name, "'");
      }
    }
    spec.slice = {tensor, offset, count};
    out.output_features.push_back(std::move(spec));
  }

  // ── Decimation ────────────────────────────────────────────────────────────
  out.decimation = cfg["decimation"].as<int>(1);
  if (out.decimation < 1) {
    Reject("decimation must be >= 1 (got ", std::to_string(out.decimation), ")");
  }

  return out;
}

}  // namespace rtc::params
