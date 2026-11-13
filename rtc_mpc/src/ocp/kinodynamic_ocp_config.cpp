#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"

#include "reference/internal/yaml_fields.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <tuple>
#include <utility>
#include <vector>

namespace rtc::mpc {

namespace {

using internal::Child;
using internal::FieldRead;
using internal::HasOnlyKeys;
using internal::Present;
using internal::ReadArray;
using internal::ReadBool;
using internal::ReadScalar;
using internal::ToError;
using Error = KinodynamicOCPConfigError;

[[nodiscard]] Error ReadWeight3(const YAML::Node& map, const char* key,
                                std::array<double, 3>& out) {
  const FieldRead r = ReadArray(Child(map, key), out);
  if (r != FieldRead::kOk) {
    return ToError<Error>(r);
  }
  for (const double w : out) {
    if (w < 0.0) {
      return Error::kNegativeWeight;
    }
  }
  return Error::kNoError;
}

/// Every model joint after the root gets its map entry, written at
/// `offset + idx_v − n_base_v` of @p out (all its velocity entries).
[[nodiscard]] Error ReadJointWeights(const YAML::Node& map, const char* key,
                                     const RobotModelHandler& model, Eigen::Index offset,
                                     Eigen::VectorXd& out) {
  const YAML::Node joints = Child(map, key);
  if (!Present(joints)) {
    return Error::kMissingKey;
  }
  if (!joints.IsMap()) {
    return Error::kNotAMap;
  }
  const pinocchio::Model& m = model.model();
  const int n_base_v = model.n_base_v();
  // Joint 0 is the universe, joint 1 the free-flyer root.
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(m.njoints); ++j) {
    double w = 0.0;
    const FieldRead r = ReadScalar(Child(joints, m.names[j].c_str()), w);
    if (r == FieldRead::kMissing) {
      return Error::kMissingJoint;
    }
    if (r != FieldRead::kOk) {
      return ToError<Error>(r);
    }
    if (w < 0.0) {
      return Error::kNegativeWeight;
    }
    const auto& joint = m.joints[j];
    out.segment(offset + joint.idx_v() - n_base_v, joint.nv()).setConstant(w);
  }
  // A name the model lacks is ignored, but a value under it is still checked:
  // a malformed entry is a malformed file whichever model reads it.
  for (const auto& kv : joints) {
    double w = 0.0;
    const FieldRead r = ReadScalar(kv.second, w);
    if (r != FieldRead::kOk) {
      return r == FieldRead::kMissing ? Error::kWrongType : ToError<Error>(r);
    }
    if (w < 0.0) {
      return Error::kNegativeWeight;
    }
  }
  return Error::kNoError;
}

/// One state-weight map onto the tangent `[δq; δv]` (size `2·nv`).
[[nodiscard]] Error ReadStateWeights(const YAML::Node& node, const char* key,
                                     const RobotModelHandler& model, Eigen::VectorXd& out) {
  const YAML::Node map = Child(node, key);
  if (!Present(map)) {
    return Error::kMissingKey;
  }
  if (!map.IsMap()) {
    return Error::kNotAMap;
  }
  if (!HasOnlyKeys(map, {"base_position", "base_orientation", "joints", "base_linear_velocity",
                         "base_angular_velocity", "joint_velocities"})) {
    return Error::kUnknownKey;
  }
  const Eigen::Index nv = model.nv();
  Eigen::VectorXd w = Eigen::VectorXd::Zero(2 * nv);
  const std::array<std::pair<const char*, Eigen::Index>, 4> base = {{
      {"base_position", 0},
      {"base_orientation", 3},
      {"base_linear_velocity", nv},
      {"base_angular_velocity", nv + 3},
  }};
  for (const auto& [name, at] : base) {
    std::array<double, 3> v{};
    const Error e = ReadWeight3(map, name, v);
    if (e != Error::kNoError) {
      return e;
    }
    w.segment<3>(at) = Eigen::Map<const Eigen::Vector3d>(v.data());
  }
  const Eigen::Index n_base_v = model.n_base_v();
  Error e = ReadJointWeights(map, "joints", model, n_base_v, w);
  if (e != Error::kNoError) {
    return e;
  }
  e = ReadJointWeights(map, "joint_velocities", model, nv + n_base_v, w);
  if (e != Error::kNoError) {
    return e;
  }
  out = std::move(w);
  return Error::kNoError;
}

[[nodiscard]] Error ReadInputWeights(const YAML::Node& node, const RobotModelHandler& model,
                                     Eigen::VectorXd& out) {
  const YAML::Node map = Child(node, "input_weights");
  if (!Present(map)) {
    return Error::kMissingKey;
  }
  if (!map.IsMap()) {
    return Error::kNotAMap;
  }
  if (!HasOnlyKeys(map, {"contact_force", "contact_moment", "joint_accelerations"})) {
    return Error::kUnknownKey;
  }
  std::array<double, 3> force{};
  std::array<double, 3> moment{};
  Error e = ReadWeight3(map, "contact_force", force);
  if (e != Error::kNoError) {
    return e;
  }
  e = ReadWeight3(map, "contact_moment", moment);
  if (e != Error::kNoError) {
    return e;
  }
  const WbLayout& layout = model.layout();
  Eigen::VectorXd w = Eigen::VectorXd::Zero(layout.nu);
  const auto& contacts = model.contact_frames();
  for (std::size_t k = 0; k < contacts.size(); ++k) {
    const Eigen::Index at = layout.contact_u_offset[k];
    w.segment<3>(at) = Eigen::Map<const Eigen::Vector3d>(force.data());
    if (contacts[k].dim == 6) {
      w.segment<3>(at + 3) = Eigen::Map<const Eigen::Vector3d>(moment.data());
    }
  }
  e = ReadJointWeights(map, "joint_accelerations", model, layout.acc_u_offset, w);
  if (e != Error::kNoError) {
    return e;
  }
  out = std::move(w);
  return Error::kNoError;
}

[[nodiscard]] Error ReadContactTracking(const YAML::Node& node,
                                        Eigen::Matrix<double, kContactTrackingRows, 1>& out) {
  const YAML::Node map = Child(node, "contact_tracking");
  if (!Present(map)) {
    return Error::kMissingKey;
  }
  if (!map.IsMap()) {
    return Error::kNotAMap;
  }
  // The residual's block order.
  constexpr std::array<const char*, 6> kBlocks = {
      "position",         "orientation",         "linear_velocity",
      "angular_velocity", "linear_acceleration", "angular_acceleration"};
  if (!HasOnlyKeys(map, {kBlocks[0], kBlocks[1], kBlocks[2], kBlocks[3], kBlocks[4], kBlocks[5]})) {
    return Error::kUnknownKey;
  }
  Eigen::Matrix<double, kContactTrackingRows, 1> w;
  for (std::size_t b = 0; b < kBlocks.size(); ++b) {
    std::array<double, 3> v{};
    const Error e = ReadWeight3(map, kBlocks[b], v);
    if (e != Error::kNoError) {
      return e;
    }
    w.segment<3>(static_cast<Eigen::Index>(3 * b)) = Eigen::Map<const Eigen::Vector3d>(v.data());
  }
  out = w;
  return Error::kNoError;
}

/// A `constraints:` scalar: finite, then `> 0` (@p positive) or `≥ 0`.
[[nodiscard]] Error ReadValue(const YAML::Node& map, const char* key, bool positive, double& out) {
  const FieldRead r = ReadScalar(Child(map, key), out);
  if (r != FieldRead::kOk) {
    return ToError<Error>(r);
  }
  if (positive && !(out > 0.0)) {
    return Error::kNonPositive;
  }
  if (!positive && out < 0.0) {
    return Error::kNegativeWeight;
  }
  return Error::kNoError;
}

/// @p section (a child just read) is a map with only @p keys. Each section is
/// bound to a node of its own: assigning one yaml-cpp node to another
/// rebinds the assigned node inside the caller's tree.
[[nodiscard]] Error CheckSection(const YAML::Node& section,
                                 std::initializer_list<const char*> keys) {
  if (!Present(section)) {
    return Error::kMissingKey;
  }
  if (!section.IsMap()) {
    return Error::kNotAMap;
  }
  if (!HasOnlyKeys(section, keys)) {
    return Error::kUnknownKey;
  }
  return Error::kNoError;
}

[[nodiscard]] Error ReadBarrier(const YAML::Node& map, PolynomialBarrier& out) {
  Error e = ReadValue(map, "mu", false, out.mu);
  if (e != Error::kNoError) {
    return e;
  }
  return ReadValue(map, "delta", true, out.delta);
}

[[nodiscard]] Error ReadFrame(const YAML::Node& pair, const char* key,
                              const pinocchio::Model& model, pinocchio::FrameIndex& out) {
  const YAML::Node v = Child(pair, key);
  if (!Present(v)) {
    return Error::kMissingKey;
  }
  if (!v.IsScalar()) {
    return Error::kWrongType;
  }
  if (!model.existFrame(v.Scalar())) {
    return Error::kUnknownFrame;
  }
  out = model.getFrameId(v.Scalar());
  return Error::kNoError;
}

[[nodiscard]] Error ReadPairs(const YAML::Node& collision, const pinocchio::Model& model,
                              std::vector<SpherePair>& out) {
  const YAML::Node pairs = Child(collision, "pairs");
  if (!Present(pairs)) {
    return Error::kMissingKey;
  }
  if (!pairs.IsSequence()) {
    return Error::kWrongType;
  }
  std::vector<SpherePair> read;
  read.reserve(pairs.size());
  for (const auto& p : pairs) {
    if (!p.IsMap()) {
      return Error::kNotAMap;
    }
    if (!HasOnlyKeys(p, {"frame_a", "offset_a", "radius_a", "frame_b", "offset_b", "radius_b"})) {
      return Error::kUnknownKey;
    }
    SpherePair sp;
    std::array<double, 3> oa{};
    std::array<double, 3> ob{};
    Error e = ReadFrame(p, "frame_a", model, sp.frame_a);
    if (e == Error::kNoError) {
      e = ReadFrame(p, "frame_b", model, sp.frame_b);
    }
    if (e != Error::kNoError) {
      return e;
    }
    for (const auto& [key, arr] :
         {std::pair<const char*, std::array<double, 3>*>{"offset_a", &oa},
          std::pair<const char*, std::array<double, 3>*>{"offset_b", &ob}}) {
      const FieldRead r = ReadArray(Child(p, key), *arr);
      if (r != FieldRead::kOk) {
        return ToError<Error>(r);
      }
    }
    e = ReadValue(p, "radius_a", false, sp.radius_a);
    if (e == Error::kNoError) {
      e = ReadValue(p, "radius_b", false, sp.radius_b);
    }
    if (e != Error::kNoError) {
      return e;
    }
    sp.offset_a = Eigen::Map<const Eigen::Vector3d>(oa.data());
    sp.offset_b = Eigen::Map<const Eigen::Vector3d>(ob.data());
    read.push_back(sp);
  }
  out = std::move(read);
  return Error::kNoError;
}

[[nodiscard]] Error ReadConstraints(const YAML::Node& node, const RobotModelHandler& model,
                                    KinodynamicConstraintsConfig& out) {
  const YAML::Node map = Child(node, "constraints");
  Error e = CheckSection(
      map, {"foot_constraint", "friction", "contact_rectangle", "joint_limits", "collision"});
  if (e != Error::kNoError) {
    return e;
  }
  KinodynamicConstraintsConfig c;

  const YAML::Node foot = Child(map, "foot_constraint");
  e = CheckSection(foot, {"position_z", "orientation", "linear_velocity_xy", "linear_velocity_z",
                          "angular_velocity", "linear_acceleration_xy", "linear_acceleration_z",
                          "angular_acceleration"});
  if (e != Error::kNoError) {
    return e;
  }
  FootConstraintGains& g = c.foot_constraint;
  const std::array<std::tuple<const char*, bool, double*>, 8> gains = {{
      {"position_z", false, &g.position_z},
      {"orientation", false, &g.orientation},
      {"linear_velocity_xy", false, &g.linear_velocity_xy},
      {"linear_velocity_z", false, &g.linear_velocity_z},
      {"angular_velocity", false, &g.angular_velocity},
      {"linear_acceleration_xy", true, &g.linear_acceleration_xy},
      {"linear_acceleration_z", true, &g.linear_acceleration_z},
      {"angular_acceleration", true, &g.angular_acceleration},
  }};
  for (const auto& [key, positive, dst] : gains) {
    e = ReadValue(foot, key, positive, *dst);
    if (e != Error::kNoError) {
      return e;
    }
  }

  const YAML::Node friction = Child(map, "friction");
  e = CheckSection(friction, {"mu", "grip_force", "regularization"});
  if (e == Error::kNoError) {
    e = ReadValue(friction, "mu", true, c.friction.mu);
  }
  if (e == Error::kNoError) {
    e = ReadValue(friction, "grip_force", false, c.friction.grip_force);
  }
  if (e == Error::kNoError) {
    e = ReadValue(friction, "regularization", true, c.friction.regularization);
  }
  if (e != Error::kNoError) {
    return e;
  }

  const YAML::Node rectangle = Child(map, "contact_rectangle");
  e = CheckSection(rectangle, {"x_min", "x_max", "y_min", "y_max"});
  if (e != Error::kNoError) {
    return e;
  }
  ContactRectangle& rect = c.contact_rectangle;
  for (const auto& [key, dst] : {std::pair<const char*, double*>{"x_min", &rect.x_min},
                                 std::pair<const char*, double*>{"x_max", &rect.x_max},
                                 std::pair<const char*, double*>{"y_min", &rect.y_min},
                                 std::pair<const char*, double*>{"y_max", &rect.y_max}}) {
    const FieldRead r = ReadScalar(Child(rectangle, key), *dst);
    if (r != FieldRead::kOk) {
      return ToError<Error>(r);
    }
  }
  if (!(rect.x_min < 0.0 && 0.0 < rect.x_max && rect.y_min < 0.0 && 0.0 < rect.y_max)) {
    return Error::kInvalidRectangle;
  }

  const YAML::Node limits = Child(map, "joint_limits");
  e = CheckSection(limits, {"mu", "delta"});
  if (e == Error::kNoError) {
    e = ReadBarrier(limits, c.joint_limits);
  }
  if (e != Error::kNoError) {
    return e;
  }
  const pinocchio::Model& m = model.model();
  // The joints the barrier limits need a non-empty range.
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(m.njoints); ++j) {
    if (!JointLimitBarrierCost::IsLimitedJoint(m, j)) {
      continue;
    }
    const auto iq = m.joints[j].idx_q();
    if (!(m.lowerPositionLimit[iq] < m.upperPositionLimit[iq])) {
      return Error::kInvalidJointLimit;
    }
  }

  const YAML::Node collision = Child(map, "collision");
  e = CheckSection(collision, {"enabled", "mu", "delta", "pairs"});
  if (e != Error::kNoError) {
    return e;
  }
  const FieldRead r = ReadBool(Child(collision, "enabled"), c.collision_enabled);
  if (r != FieldRead::kOk) {
    return ToError<Error>(r);
  }
  e = ReadBarrier(collision, c.collision);
  if (e == Error::kNoError) {
    e = ReadPairs(collision, m, c.collision_pairs);
  }
  if (e != Error::kNoError) {
    return e;
  }
  out = std::move(c);
  return Error::kNoError;
}

[[nodiscard]] Error Load(const YAML::Node& node, const RobotModelHandler& model,
                         KinodynamicOCPConfig& out) {
  if (!model.Initialised()) {
    return Error::kModelNotInitialised;
  }
  // The base blocks below sit at fixed tangent offsets: a free-flyer root.
  if (!model.floating_base() || model.n_base_v() != kFreeFlyerNv) {
    return Error::kFixedBase;
  }
  if (!Present(node)) {
    return Error::kMissingKey;
  }
  if (!node.IsMap()) {
    return Error::kNotAMap;
  }
  if (!HasOnlyKeys(node,
                   {"horizon_nodes", "dt", "integrator", "terminal_scaling", "state_weights",
                    "terminal_weights", "input_weights", "contact_tracking", "constraints"})) {
    return Error::kUnknownKey;
  }
  KinodynamicOCPConfig cfg;

  const YAML::Node horizon = Child(node, "horizon_nodes");
  if (!Present(horizon)) {
    return Error::kMissingKey;
  }
  if (!horizon.IsScalar()) {
    return Error::kWrongType;
  }
  try {
    cfg.horizon_nodes = horizon.as<int>();
  } catch (...) {
    return Error::kWrongType;
  }
  if (cfg.horizon_nodes < 1 || cfg.horizon_nodes > kMaxHorizon) {
    return Error::kInvalidHorizon;
  }

  FieldRead r = ReadScalar(Child(node, "dt"), cfg.dt);
  if (r != FieldRead::kOk) {
    return ToError<Error>(r);
  }
  if (cfg.dt <= 0.0) {
    return Error::kInvalidDt;
  }

  const YAML::Node integrator = Child(node, "integrator");
  if (!Present(integrator)) {
    return Error::kMissingKey;
  }
  if (!integrator.IsScalar()) {
    return Error::kWrongType;
  }
  if (ParseKinodynamicIntegrator(integrator.Scalar(), cfg.integrator) !=
      KinodynamicError::kNoError) {
    return Error::kUnknownIntegrator;
  }

  r = ReadScalar(Child(node, "terminal_scaling"), cfg.terminal_scaling);
  if (r != FieldRead::kOk) {
    return ToError<Error>(r);
  }
  if (cfg.terminal_scaling < 0.0) {
    return Error::kNegativeWeight;
  }

  Error e = ReadStateWeights(node, "state_weights", model, cfg.state_weights);
  if (e != Error::kNoError) {
    return e;
  }
  e = ReadStateWeights(node, "terminal_weights", model, cfg.terminal_weights);
  if (e != Error::kNoError) {
    return e;
  }
  e = ReadInputWeights(node, model, cfg.input_weights);
  if (e != Error::kNoError) {
    return e;
  }
  e = ReadContactTracking(node, cfg.contact_tracking_weights);
  if (e != Error::kNoError) {
    return e;
  }
  e = ReadConstraints(node, model, cfg.constraints);
  if (e != Error::kNoError) {
    return e;
  }
  out = std::move(cfg);
  return Error::kNoError;
}

}  // namespace

KinodynamicOCPConfigError LoadKinodynamicOCPConfig(const YAML::Node& node,
                                                   const RobotModelHandler& model,
                                                   KinodynamicOCPConfig& out) noexcept {
  try {
    return Load(node, model, out);
  } catch (...) {
    // yaml-cpp iteration or an allocation; nothing has been written to out.
    return Error::kWrongType;
  }
}

}  // namespace rtc::mpc
