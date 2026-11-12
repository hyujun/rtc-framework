// ── How a YAML node is READ, shared by every `ParseXxxParams` ───────────────
// What is shared is strictly how a node is read — which YAML shapes count as
// "absent", which key of a map is unknown or missing, whether a sequence has
// the declared length and finite entries, which name of a closed set a scalar
// is. What is NOT shared, on purpose, is how a problem is REPORTED: every
// helper returns a classification and never throws, so each parser keeps its
// own wording and its own mechanism (throw at configure, or collect into a
// report). Nothing here knows a key name, a range or a message prefix — a
// parser that needs a rule not here generalises this header rather than
// forking a private copy (P5).
//
// Non-RT: yaml-cpp allocates. Call at configure time only.
#pragma once

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>

namespace rtc::params {

// ── Section access ──────────────────────────────────────────────────────────

/// @brief What `ReadSectionNode` found under the key.
enum class SectionKind {
  kAbsent,   ///< parent absent/not a map, or `key` absent, or an empty `key:`
  kMap,      ///< `key` is a map — `node` is it
  kNotAMap,  ///< `key` is present but is a scalar/sequence — the caller refuses
};

/// @brief Result of @ref ReadSectionNode.
struct SectionRead {
  SectionKind kind{SectionKind::kAbsent};
  /// The section for `kMap`; otherwise an EMPTY node, so that reading a key
  /// under an absent section yields that key's default. Subscript it only
  /// through a CONST reference: the const overload of `operator[]` hands back
  /// an undefined (falsy) node, whereas a default-constructed `YAML::Node` is
  /// itself DEFINED. Going through this rather than `parent[key][...]` also
  /// matters: yaml-cpp throws `YAML::InvalidNode`, not `std::invalid_argument`,
  /// when a missing node is subscripted.
  YAML::Node node{};
};

/// @brief Classify the child `key` of `parent` as absent, a map, or not a map.
[[nodiscard]] inline SectionRead ReadSectionNode(const YAML::Node& parent, const char* key) {
  if (!parent.IsMap()) {
    return {};  // the parent section itself was absent
  }
  const YAML::Node child = parent[key];
  if (!child || child.IsNull()) {
    return {};
  }
  if (!child.IsMap()) {
    return {SectionKind::kNotAMap, YAML::Node()};
  }
  return {SectionKind::kMap, child};
}

// ── Key sets ────────────────────────────────────────────────────────────────

/// @brief The first key of the map `sec` that is not in `allowed`.
/// @return the key by name (a non-scalar key reads as "<non-scalar key>");
///         nullopt when `sec` is not a map or every key is known. The caller
///         words the refusal — a misspelt key would otherwise fall back to its
///         default without a word (#642 D12).
[[nodiscard]] inline std::optional<std::string> FindUnknownKey(
    const YAML::Node& sec, std::initializer_list<const char*> allowed) {
  if (!sec || !sec.IsMap()) {
    return std::nullopt;
  }
  for (const auto& kv : sec) {
    const YAML::Node k = kv.first;
    const std::string name = k.IsScalar() ? k.Scalar() : std::string("<non-scalar key>");
    bool known = false;
    for (const char* a : allowed) {
      known = known || name == a;
    }
    if (!known) {
      return name;
    }
  }
  return std::nullopt;
}

/// @brief The first key of `required` that `sec` does not carry with a value.
/// @return the key; nullopt when every one is present. An empty `key:` (null)
///         counts as missing, so is a non-map `sec` (every key is then missing,
///         and the first is returned). Unknown keys are @ref FindUnknownKey's
///         layer — check both, and word the two differently.
[[nodiscard]] inline std::optional<std::string> FindMissingKey(
    const YAML::Node& sec, std::initializer_list<const char*> required) {
  for (const char* r : required) {
    if (!sec || !sec.IsMap()) {
      return std::string(r);
    }
    const YAML::Node v = sec[r];
    if (!v || v.IsNull()) {
      return std::string(r);
    }
  }
  return std::nullopt;
}

// ── Fixed-length numeric vector ─────────────────────────────────────────────

/// @brief What @ref ReadFixedVector found.
enum class VectorReadKind {
  kAbsent,        ///< no node (or an empty value) — the caller applies its default
  kRead,          ///< `value` holds the entries
  kNotASequence,  ///< present but a scalar or a map
  kWrongLength,   ///< a sequence whose length is not the declared one (`length` has it)
  kNotANumber,    ///< an entry is not a number (`index` has it)
  kNonFinite,     ///< an entry is NaN/inf and finiteness was required (`index` has it)
};

/// @brief Result of @ref ReadFixedVector.
struct VectorRead {
  VectorReadKind kind{VectorReadKind::kAbsent};
  Eigen::VectorXd value{};
  std::size_t length{0};  ///< the sequence's length as written (for kWrongLength)
  std::size_t index{0};   ///< the offending entry (for kNotANumber / kNonFinite)
};

/// @brief Read `node` as a sequence of exactly `n` numbers.
/// @param require_finite refuse NaN/inf entries (YAML `.nan` / `.inf`)
[[nodiscard]] inline VectorRead ReadFixedVector(const YAML::Node& node, std::size_t n,
                                                bool require_finite = true) {
  if (!node || node.IsNull()) {
    return {};
  }
  if (!node.IsSequence()) {
    return {VectorReadKind::kNotASequence, {}, 0, 0};
  }
  if (node.size() != n) {
    return {VectorReadKind::kWrongLength, {}, node.size(), 0};
  }
  VectorRead out{VectorReadKind::kRead, Eigen::VectorXd(static_cast<Eigen::Index>(n)), n, 0};
  for (std::size_t i = 0; i < n; ++i) {
    double d = 0.0;
    try {
      d = node[i].as<double>();
    } catch (const YAML::Exception&) {
      return {VectorReadKind::kNotANumber, {}, n, i};
    }
    if (require_finite && !std::isfinite(d)) {
      return {VectorReadKind::kNonFinite, {}, n, i};
    }
    out.value[static_cast<Eigen::Index>(i)] = d;
  }
  return out;
}

// ── Closed set of names ─────────────────────────────────────────────────────

/// @brief The index of the scalar `node` in `names`.
/// @return nullopt when `node` is absent, not a scalar, or not one of `names`
///         (the caller tells absent from wrong with `node` itself and quotes
///         it through @ref Spelling).
[[nodiscard]] inline std::optional<std::size_t> ReadEnum(const YAML::Node& node,
                                                         std::initializer_list<const char*> names) {
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  std::size_t i = 0;
  for (const char* name : names) {
    if (node.Scalar() == name) {
      return i;
    }
    ++i;
  }
  return std::nullopt;
}

// ── Message support ─────────────────────────────────────────────────────────

/// @brief The value as the YAML file spells it, for a message that quotes what
///        was written rather than a reformatted double (1e-10 through
///        `std::to_string` is "0.000000", which reads as a different complaint
///        than the one being made).
[[nodiscard]] inline std::string Spelling(const YAML::Node& v) {
  if (v.IsScalar()) {
    return "'" + v.Scalar() + "'";
  }
  if (v.IsSequence()) {
    return "a sequence";
  }
  if (v.IsMap()) {
    return "a map";
  }
  return "an empty value";
}

}  // namespace rtc::params
