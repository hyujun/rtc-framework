#pragma once

// A YAML value that is either one scalar for every entry or exactly `n`
// values — the shape the constraints' per-joint / per-axis options take.
// Init-time only (throws).

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::tsid {

// `node[key]` as an n-vector: a scalar fills every entry, a sequence must have
// exactly `n`. Every entry must be finite and `>= lower` (`> lower` when
// `strict`). `who` prefixes the error. The key must be present.
[[nodiscard]] inline Eigen::VectorXd ReadScalarOrVector(const YAML::Node& node, const char* key,
                                                        Eigen::Index n, double lower, bool strict,
                                                        const char* who) {
  const YAML::Node v = node[key];
  Eigen::VectorXd out(n);
  if (v.IsScalar()) {
    out.setConstant(v.as<double>());
  } else if (v.IsSequence() && static_cast<Eigen::Index>(v.size()) == n) {
    for (Eigen::Index i = 0; i < n; ++i) {
      out(i) = v[static_cast<std::size_t>(i)].as<double>();
    }
  } else {
    throw std::invalid_argument(std::string(who) + "." + key + ": a scalar or exactly " +
                                std::to_string(n) + " values");
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    const bool ok = std::isfinite(out(i)) && (strict ? out(i) > lower : out(i) >= lower);
    if (!ok) {
      throw std::invalid_argument(std::string(who) + "." + key +
                                  ": every value must be finite and " + (strict ? "> " : ">= ") +
                                  std::to_string(lower));
    }
  }
  return out;
}

}  // namespace rtc::tsid
