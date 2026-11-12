#include "rtc_mpc/reference/gait_table.hpp"

#include "reference/internal/yaml_fields.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rtc::mpc {
namespace {

using internal::FieldRead;

[[nodiscard]] GaitTableError ToError(FieldRead r) noexcept {
  switch (r) {
    case FieldRead::kOk:
      return GaitTableError::kNoError;
    case FieldRead::kMissing:
      return GaitTableError::kMissingKey;
    case FieldRead::kWrongType:
      return GaitTableError::kWrongType;
    case FieldRead::kNonFinite:
      return GaitTableError::kNonFinite;
  }
  return GaitTableError::kWrongType;
}

[[nodiscard]] bool NonNegative(const std::array<double, 2>& a) noexcept {
  return a[0] >= 0.0 && a[1] >= 0.0;
}

// One gait entry → `g`. `bits` maps contact names to bit positions.
[[nodiscard]] GaitTableError LoadGait(const YAML::Node& node,
                                      std::span<const std::string> contact_names, GaitTemplate& g) {
  if (!node.IsDefined() || !node.IsMap()) {
    return GaitTableError::kNotAMap;
  }
  if (!internal::HasOnlyKeys(node, {"modes", "switching_times", "faster_above", "slower_below",
                                    "measured_tolerance"})) {
    return GaitTableError::kUnknownKey;
  }
  const YAML::Node modes = internal::Child(node, "modes");
  const YAML::Node times = internal::Child(node, "switching_times");
  if (!internal::Present(modes) || !internal::Present(times)) {
    return GaitTableError::kMissingKey;
  }
  if (!modes.IsSequence() || !times.IsSequence()) {
    return GaitTableError::kWrongType;
  }
  if (modes.size() == 0) {
    return GaitTableError::kNoPhase;
  }
  if (modes.size() > kMaxGaitPhases) {
    return GaitTableError::kTooManyPhases;
  }
  if (times.size() != modes.size() + 1) {
    return GaitTableError::kLengthMismatch;
  }
  GaitTemplate tmp;
  tmp.n_phases = static_cast<std::uint32_t>(modes.size());
  for (std::size_t j = 0; j < modes.size(); ++j) {
    const YAML::Node m = modes[j];
    if (!m.IsDefined() || !m.IsSequence()) {
      return GaitTableError::kWrongType;  // `[]` is a sequence: flight
    }
    std::uint32_t mask = 0;
    for (std::size_t k = 0; k < m.size(); ++k) {
      if (!m[k].IsScalar()) {
        return GaitTableError::kWrongType;
      }
      const auto it = std::find(contact_names.begin(), contact_names.end(), m[k].Scalar());
      if (it == contact_names.end()) {
        return GaitTableError::kUnknownContact;
      }
      const std::uint32_t bit = 1U << static_cast<unsigned>(it - contact_names.begin());
      if ((mask & bit) != 0U) {
        return GaitTableError::kRepeatedContact;
      }
      mask |= bit;
    }
    tmp.mode[j] = mask;
  }
  for (std::size_t j = 0; j < times.size(); ++j) {
    const FieldRead r = internal::ReadScalar(times[j], tmp.switching_time[j]);
    if (r != FieldRead::kOk) {
      return r == FieldRead::kMissing ? GaitTableError::kWrongType : ToError(r);
    }
  }
  tmp.period = tmp.switching_time[tmp.n_phases];
  const FieldRead reads[] = {
      internal::ReadArray(internal::Child(node, "faster_above"), tmp.faster_above),
      internal::ReadArray(internal::Child(node, "slower_below"), tmp.slower_below),
      internal::ReadArray(internal::Child(node, "measured_tolerance"), tmp.measured_tolerance),
  };
  for (const FieldRead r : reads) {
    if (r != FieldRead::kOk) {
      return ToError(r);
    }
  }
  g = tmp;
  return GaitTableError::kNoError;
}

[[nodiscard]] GaitTableError ValidateGait(const GaitTemplate& g,
                                          std::uint32_t n_contacts) noexcept {
  if (g.n_phases == 0) {
    return GaitTableError::kNoPhase;
  }
  if (g.n_phases > kMaxGaitPhases) {
    return GaitTableError::kTooManyPhases;
  }
  for (std::size_t j = 0; j <= g.n_phases; ++j) {
    if (!std::isfinite(g.switching_time[j])) {
      return GaitTableError::kNonFinite;
    }
  }
  if (g.switching_time[0] != 0.0 || g.period != g.switching_time[g.n_phases]) {
    return GaitTableError::kBadSwitchingTimes;
  }
  for (std::size_t j = 0; j < g.n_phases; ++j) {
    if (!(g.switching_time[j + 1] > g.switching_time[j])) {
      return GaitTableError::kBadSwitchingTimes;
    }
  }
  const std::uint32_t allowed = n_contacts >= kMaxContactBits ? ~0U : ((1U << n_contacts) - 1U);
  if ((ModeUnion(g) & ~allowed) != 0U) {
    return GaitTableError::kModeOutOfRange;
  }
  for (std::size_t i = 0; i < 2; ++i) {
    if (!std::isfinite(g.faster_above[i]) || !std::isfinite(g.slower_below[i]) ||
        !std::isfinite(g.measured_tolerance[i])) {
      return GaitTableError::kNonFinite;
    }
  }
  if (!NonNegative(g.faster_above) || !NonNegative(g.slower_below) ||
      !NonNegative(g.measured_tolerance)) {
    return GaitTableError::kNegativeValue;
  }
  return GaitTableError::kNoError;
}

}  // namespace

double MinPhase(const GaitTable& table) noexcept {
  double m = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < table.n_gaits && k < kMaxGaits; ++k) {
    const GaitTemplate& g = table.gaits[k];
    for (std::size_t j = 0; j < g.n_phases && j < kMaxGaitPhases; ++j) {
      m = std::min(m, g.switching_time[j + 1] - g.switching_time[j]);
    }
  }
  return std::isfinite(m) ? m : 0.0;
}

GaitTableError ValidateGaitTable(const GaitTable& t) noexcept {
  if (t.n_contacts == 0 || t.n_contacts > kMaxContactBits) {
    return GaitTableError::kBadContactList;
  }
  if (t.n_gaits == 0) {
    return GaitTableError::kBadSequence;
  }
  if (t.n_gaits > kMaxGaits) {
    return GaitTableError::kTooManyGaits;
  }
  if (!std::isfinite(t.min_dwell) || !std::isfinite(t.switch_lead)) {
    return GaitTableError::kNonFinite;
  }
  if (t.min_dwell < 0.0 || t.switch_lead < 0.0) {
    return GaitTableError::kNegativeValue;
  }
  for (std::size_t k = 0; k < t.n_gaits; ++k) {
    const GaitTableError e = ValidateGait(t.gaits[k], t.n_contacts);
    if (e != GaitTableError::kNoError) {
      return e;
    }
  }
  for (std::size_t k = 0; k + 1 < t.n_gaits; ++k) {
    for (std::size_t i = 0; i < 2; ++i) {
      if (!(t.gaits[k].faster_above[i] > t.gaits[k + 1].slower_below[i])) {
        return GaitTableError::kHysteresisInversion;
      }
    }
  }
  return GaitTableError::kNoError;
}

GaitTableError LoadGaitTable(const YAML::Node& node, std::span<const std::string> contact_names,
                             GaitTable& out, std::vector<std::string>* gait_names) noexcept {
  try {
    if (contact_names.empty() || contact_names.size() > kMaxContactBits) {
      return GaitTableError::kBadContactList;
    }
    for (std::size_t i = 0; i < contact_names.size(); ++i) {
      for (std::size_t j = i + 1; j < contact_names.size(); ++j) {
        if (contact_names[i] == contact_names[j]) {
          return GaitTableError::kBadContactList;
        }
      }
    }
    if (!node.IsDefined() || !node.IsMap()) {
      return GaitTableError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(node, {"sequence", "min_dwell", "switch_lead", "gaits"})) {
      return GaitTableError::kUnknownKey;
    }
    GaitTable t;
    t.n_contacts = static_cast<std::uint32_t>(contact_names.size());
    const FieldRead scalars[] = {
        internal::ReadScalar(internal::Child(node, "min_dwell"), t.min_dwell),
        internal::ReadScalar(internal::Child(node, "switch_lead"), t.switch_lead),
    };
    for (const FieldRead r : scalars) {
      if (r != FieldRead::kOk) {
        return ToError(r);
      }
    }
    const YAML::Node seq = internal::Child(node, "sequence");
    const YAML::Node gaits = internal::Child(node, "gaits");
    if (!internal::Present(seq) || !internal::Present(gaits)) {
      return GaitTableError::kMissingKey;
    }
    if (!seq.IsSequence()) {
      return GaitTableError::kWrongType;
    }
    if (!gaits.IsMap()) {
      return GaitTableError::kNotAMap;
    }
    if (seq.size() == 0) {
      return GaitTableError::kBadSequence;
    }
    if (seq.size() > kMaxGaits) {
      return GaitTableError::kTooManyGaits;
    }
    std::vector<std::string> names;
    for (std::size_t k = 0; k < seq.size(); ++k) {
      if (!seq[k].IsScalar()) {
        return GaitTableError::kWrongType;
      }
      const std::string name = seq[k].Scalar();
      if (std::find(names.begin(), names.end(), name) != names.end()) {
        return GaitTableError::kBadSequence;
      }
      const YAML::Node entry = internal::Child(gaits, name.c_str());
      if (!entry.IsDefined()) {
        return GaitTableError::kBadSequence;
      }
      const GaitTableError e = LoadGait(entry, contact_names, t.gaits[k]);
      if (e != GaitTableError::kNoError) {
        return e;
      }
      names.push_back(name);
    }
    if (gaits.size() != names.size()) {
      return GaitTableError::kBadSequence;  // a gait defined but not in `sequence`
    }
    t.n_gaits = static_cast<std::uint32_t>(names.size());
    const GaitTableError e = ValidateGaitTable(t);
    if (e != GaitTableError::kNoError) {
      return e;
    }
    out = t;
    if (gait_names != nullptr) {
      *gait_names = std::move(names);
    }
    return GaitTableError::kNoError;
  } catch (...) {
    return GaitTableError::kWrongType;
  }
}

}  // namespace rtc::mpc
