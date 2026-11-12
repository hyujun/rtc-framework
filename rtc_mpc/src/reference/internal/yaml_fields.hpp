#ifndef RTC_MPC_SRC_REFERENCE_INTERNAL_YAML_FIELDS_HPP_
#define RTC_MPC_SRC_REFERENCE_INTERNAL_YAML_FIELDS_HPP_

// Strict, no-throw field reads shared by the reference/ YAML loaders. Every
// schema key is required and an unknown key is refused, so a misspelt key
// cannot fall back to a default.
//
// HasOnlyKeys is the yes/no form of rtc_controllers/params/yaml_read.hpp's
// FindUnknownKey (rtc_mpc does not depend on rtc_controllers; the loaders here
// return an enum, not the key name). ReadScalar is the strict form of
// phase/phase_cost_config.cpp's TryReadScalar (that one lets null and non-finite
// through). All three are candidates to promote into one rtc_base helper.
//
// Reads go through const nodes only (yaml-cpp 0.8: a non-const subscript inserts
// the missing key, and a missed const subscript returns a node on which only
// IsDefined() is safe).

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string>

namespace rtc::mpc::internal {

enum class FieldRead { kOk, kMissing, kWrongType, kNonFinite };

// A FieldRead as the loader's own error enum — every reference/ loader enum
// carries the four names below.
template <class E>
[[nodiscard]] constexpr E ToError(FieldRead r) noexcept {
  switch (r) {
    case FieldRead::kOk:
      return E::kNoError;
    case FieldRead::kMissing:
      return E::kMissingKey;
    case FieldRead::kWrongType:
      return E::kWrongType;
    case FieldRead::kNonFinite:
      return E::kNonFinite;
  }
  return E::kWrongType;
}

// true when every key of the map `node` is in `allowed` (a non-map is the
// caller's kNotAMap, checked first).
[[nodiscard]] inline bool HasOnlyKeys(const YAML::Node& node,
                                      std::initializer_list<const char*> allowed) {
  for (const auto& kv : node) {
    if (!kv.first.IsScalar()) {
      return false;
    }
    const std::string& name = kv.first.Scalar();
    bool known = false;
    for (const char* a : allowed) {
      known = known || name == a;
    }
    if (!known) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] inline YAML::Node Child(const YAML::Node& map, const char* key) {
  const YAML::Node& cmap = map;
  return cmap[key];
}

[[nodiscard]] inline bool Present(const YAML::Node& v) {
  return v.IsDefined() && !v.IsNull();
}

[[nodiscard]] inline FieldRead ReadScalar(const YAML::Node& v, double& out) {
  if (!Present(v)) {
    return FieldRead::kMissing;
  }
  if (!v.IsScalar()) {
    return FieldRead::kWrongType;
  }
  double x = 0.0;
  try {
    x = v.as<double>();
  } catch (...) {
    return FieldRead::kWrongType;
  }
  if (!std::isfinite(x)) {
    return FieldRead::kNonFinite;
  }
  out = x;
  return FieldRead::kOk;
}

[[nodiscard]] inline FieldRead ReadBool(const YAML::Node& v, bool& out) {
  if (!Present(v)) {
    return FieldRead::kMissing;
  }
  if (!v.IsScalar()) {
    return FieldRead::kWrongType;
  }
  try {
    out = v.as<bool>();
  } catch (...) {
    return FieldRead::kWrongType;
  }
  return FieldRead::kOk;
}

template <std::size_t N>
[[nodiscard]] FieldRead ReadArray(const YAML::Node& v, std::array<double, N>& out) {
  if (!Present(v)) {
    return FieldRead::kMissing;
  }
  if (!v.IsSequence() || v.size() != N) {
    return FieldRead::kWrongType;
  }
  std::array<double, N> tmp{};
  for (std::size_t i = 0; i < N; ++i) {
    const FieldRead r = ReadScalar(v[i], tmp[i]);
    if (r != FieldRead::kOk) {
      return r == FieldRead::kMissing ? FieldRead::kWrongType : r;
    }
  }
  out = tmp;
  return FieldRead::kOk;
}

}  // namespace rtc::mpc::internal

#endif  // RTC_MPC_SRC_REFERENCE_INTERNAL_YAML_FIELDS_HPP_
