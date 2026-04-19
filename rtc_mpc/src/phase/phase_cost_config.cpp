#include "rtc_mpc/phase/phase_cost_config.hpp"

#include <string>
#include <vector>

namespace rtc::mpc {

namespace {

// Parse a scalar of type T from a YAML node; returns std::nullopt on type
// mismatch or missing-required. Required-vs-optional handling lives in the
// caller so error codes can be specific.
template <typename T>
bool TryReadScalar(const YAML::Node &node, T &out) noexcept {
  if (!node.IsDefined() || !node.IsScalar()) {
    return false;
  }
  try {
    out = node.as<T>();
    return true;
  } catch (...) {
    return false;
  }
}

bool TryReadSequenceToVectorXd(const YAML::Node &node,
                               Eigen::VectorXd &out) noexcept {
  if (!node.IsDefined() || !node.IsSequence()) {
    return false;
  }
  try {
    Eigen::VectorXd tmp(static_cast<Eigen::Index>(node.size()));
    for (std::size_t i = 0; i < node.size(); ++i) {
      tmp[static_cast<Eigen::Index>(i)] = node[i].as<double>();
    }
    out = std::move(tmp);
    return true;
  } catch (...) {
    return false;
  }
}

bool AnyNegative(double x) noexcept { return x < 0.0; }

} // namespace

PhaseCostConfigError
PhaseCostConfig::LoadFromYaml(const YAML::Node &cfg,
                              const RobotModelHandler &model,
                              PhaseCostConfig &out) noexcept {
  if (!model.Initialised()) {
    return PhaseCostConfigError::kModelNotInitialised;
  }
  if (!cfg.IsMap()) {
    return PhaseCostConfigError::kInvalidYamlSchema;
  }

  PhaseCostConfig tmp{};

  // Horizon / timing ──────────────────────────────────────────────────────
  if (!TryReadScalar<int>(cfg["horizon_length"], tmp.horizon_length)) {
    return PhaseCostConfigError::kInvalidYamlSchema;
  }
  if (tmp.horizon_length <= 0) {
    return PhaseCostConfigError::kInvalidHorizon;
  }
  if (!TryReadScalar<double>(cfg["dt"], tmp.dt)) {
    return PhaseCostConfigError::kInvalidYamlSchema;
  }
  if (tmp.dt <= 0.0) {
    return PhaseCostConfigError::kInvalidDt;
  }

  // Scalar weights ────────────────────────────────────────────────────────
  const std::pair<const char *, double *> scalar_weights[] = {
      {"w_frame_placement", &tmp.w_frame_placement},
      {"w_state_reg", &tmp.w_state_reg},
      {"w_control_reg", &tmp.w_control_reg},
      {"w_contact_force", &tmp.w_contact_force},
      {"w_centroidal_momentum", &tmp.w_centroidal_momentum},
  };
  for (const auto &[key, dst] : scalar_weights) {
    if (!TryReadScalar<double>(cfg[key], *dst)) {
      return PhaseCostConfigError::kInvalidYamlSchema;
    }
    if (AnyNegative(*dst)) {
      return PhaseCostConfigError::kInvalidWeightSign;
    }
  }

  // W_placement (fixed 6-vector) ──────────────────────────────────────────
  {
    const YAML::Node node = cfg["W_placement"];
    if (!node.IsDefined() || !node.IsSequence()) {
      return PhaseCostConfigError::kInvalidYamlSchema;
    }
    if (node.size() != 6) {
      return PhaseCostConfigError::kPlacementWeightDimMismatch;
    }
    try {
      for (std::size_t i = 0; i < 6; ++i) {
        tmp.W_placement[static_cast<Eigen::Index>(i)] = node[i].as<double>();
      }
    } catch (...) {
      return PhaseCostConfigError::kInvalidYamlSchema;
    }
    for (Eigen::Index i = 0; i < 6; ++i) {
      if (tmp.W_placement[i] < 0.0) {
        return PhaseCostConfigError::kInvalidWeightSign;
      }
    }
  }

  // q_posture_ref (nq entries) ────────────────────────────────────────────
  if (!TryReadSequenceToVectorXd(cfg["q_posture_ref"], tmp.q_posture_ref)) {
    return PhaseCostConfigError::kInvalidYamlSchema;
  }
  if (tmp.q_posture_ref.size() != static_cast<Eigen::Index>(model.nq())) {
    return PhaseCostConfigError::kPostureRefDimMismatch;
  }

  // F_target (Σ contact dims entries) — optional iff zero contacts ────────
  const int force_dim_sum = [&]() noexcept {
    int s = 0;
    for (const auto &cf : model.contact_frames()) {
      s += cf.dim;
    }
    return s;
  }();
  {
    const YAML::Node node = cfg["F_target"];
    if (force_dim_sum == 0) {
      // No contacts declared; F_target may be absent or an empty sequence.
      if (node.IsDefined() && !node.IsSequence()) {
        return PhaseCostConfigError::kInvalidYamlSchema;
      }
      if (node.IsDefined() && node.size() != 0) {
        return PhaseCostConfigError::kForceTargetDimMismatch;
      }
      tmp.F_target.resize(0);
    } else {
      if (!TryReadSequenceToVectorXd(node, tmp.F_target)) {
        return PhaseCostConfigError::kInvalidYamlSchema;
      }
      if (tmp.F_target.size() != static_cast<Eigen::Index>(force_dim_sum)) {
        return PhaseCostConfigError::kForceTargetDimMismatch;
      }
    }
  }

  // custom_weights (optional) ─────────────────────────────────────────────
  {
    const YAML::Node node = cfg["custom_weights"];
    if (node.IsDefined()) {
      if (!node.IsMap()) {
        return PhaseCostConfigError::kInvalidYamlSchema;
      }
      try {
        for (const auto &kv : node) {
          auto key = kv.first.as<std::string>();
          const auto val = kv.second.as<double>();
          if (val < 0.0) {
            return PhaseCostConfigError::kInvalidWeightSign;
          }
          tmp.custom_weights.emplace(std::move(key), val);
        }
      } catch (...) {
        return PhaseCostConfigError::kInvalidYamlSchema;
      }
    }
  }

  out = std::move(tmp);
  return PhaseCostConfigError::kNoError;
}

} // namespace rtc::mpc
