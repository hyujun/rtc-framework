/// @file locomotion_phase_manager.cpp
/// @brief The gait phase manager of the kinodynamic OCP (header comment).

#include "rtc_mpc/phase/locomotion_phase_manager.hpp"

#include "reference/internal/yaml_fields.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc::mpc {

namespace {

// The dispatch key is returned by value every tick: it must stay inside the
// library's short-string buffer, or every Update allocates.
static_assert(KinodynamicOCP::kType.size() <= 15, "the ocp_type must fit the short-string buffer");

/// `horizon_nodes`: an integer in [1, kMaxHorizon].
[[nodiscard]] bool ReadNodeCount(const YAML::Node& v, int& out) {
  if (!internal::Present(v) || !v.IsScalar()) {
    return false;
  }
  int n = 0;
  try {
    n = v.as<int>();
  } catch (...) {
    return false;
  }
  if (n < 1 || n > kMaxHorizon) {
    return false;
  }
  out = n;
  return true;
}

}  // namespace

LocomotionPhaseManager::LocomotionPhaseManager(const RobotModelHandler& model,
                                               std::vector<std::string> contact_names)
    : model_(&model), contact_names_(std::move(contact_names)) {}

LocomotionPhaseError LocomotionPhaseManager::Load(const YAML::Node& cfg) noexcept {
  initialised_ = false;
  SetLastError(LocomotionPhaseError::kNotInitialised);
  try {
    if (!cfg.IsDefined() || !cfg.IsMap()) {
      return LocomotionPhaseError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(cfg,
                               {"horizon_nodes", "dt", "gait", "swing", "terrain", "reference"})) {
      return LocomotionPhaseError::kUnknownKey;
    }
    for (const char* key : {"horizon_nodes", "dt", "gait", "swing", "terrain", "reference"}) {
      if (!internal::Present(internal::Child(cfg, key))) {
        return LocomotionPhaseError::kMissingKey;
      }
    }
    int nodes = 0;
    double dt = 0.0;
    if (!ReadNodeCount(internal::Child(cfg, "horizon_nodes"), nodes) ||
        internal::ReadScalar(internal::Child(cfg, "dt"), dt) != internal::FieldRead::kOk ||
        !(dt > 0.0)) {
      return LocomotionPhaseError::kInvalidHorizon;
    }
    if (!model_->Initialised()) {
      return LocomotionPhaseError::kModelNotInitialised;
    }
    if (static_cast<int>(contact_names_.size()) != model_->n_contacts()) {
      return LocomotionPhaseError::kContactNameMismatch;
    }

    GaitTable table;
    std::vector<std::string> gait_names;
    if (LoadGaitTable(internal::Child(cfg, "gait"), contact_names_, table, &gait_names) !=
        GaitTableError::kNoError) {
      return LocomotionPhaseError::kInvalidGait;
    }
    SwingConfig swing;
    if (LoadSwingConfig(internal::Child(cfg, "swing"), swing) != SwingError::kNoError) {
      return LocomotionPhaseError::kInvalidSwing;
    }
    SurfaceFrame terrain;
    if (LoadTerrainConfig(internal::Child(cfg, "terrain"), terrain) !=
        SurfaceFrameError::kNoError) {
      return LocomotionPhaseError::kInvalidTerrain;
    }
    // The joints the reference names: every model joint after the root.
    const pinocchio::Model& model = model_->model();
    std::vector<std::string> joints;
    for (std::size_t j = 2; j < static_cast<std::size_t>(model.njoints); ++j) {
      joints.push_back(model.names[j]);
    }
    ReferenceConfig reference;
    if (LoadReferenceConfig(internal::Child(cfg, "reference"), joints, reference) !=
        ReferenceError::kNoError) {
      return LocomotionPhaseError::kInvalidReference;
    }

    if (scheduler_.Init(table, static_cast<double>(nodes) * dt) != GaitSchedulerError::kNoError) {
      return LocomotionPhaseError::kSchedulerInitFailed;
    }
    if (builder_.Init(*model_, reference, swing, terrain, nodes + 1, dt) !=
        ReferenceError::kNoError) {
      return LocomotionPhaseError::kReferenceInitFailed;
    }
    gait_names_ = std::move(gait_names);
    horizon_nodes_ = nodes;
    dt_ = dt;
    grid_ = NodeGrid{0.0, dt};
    node_index_ = 0;
    anchored_ = false;
  } catch (...) {
    return LocomotionPhaseError::kNotAMap;  // only an allocation failure reaches here
  }
  current_gait_.store(scheduler_.current_gait(), std::memory_order_relaxed);
  initialised_ = true;
  return LocomotionPhaseError::kNoError;
}

void LocomotionPhaseManager::Init(const YAML::Node& cfg) {
  const LocomotionPhaseError e = Load(cfg);
  if (e != LocomotionPhaseError::kNoError) {
    throw std::runtime_error("LocomotionPhaseManager::Init failed, error code " +
                             std::to_string(static_cast<int>(e)) +
                             " — see LocomotionPhaseError in locomotion_phase_manager.hpp");
  }
}

PlanarVelocity LocomotionPhaseManager::MeasuredPlanarVelocity(
    const MPCStateSnapshot& state) noexcept {
  const Eigen::Quaterniond quat(state.q[6], state.q[3], state.q[4], state.q[5]);
  if (!(quat.squaredNorm() > 0.0)) {
    return PlanarVelocity{};
  }
  const Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
  const Eigen::Vector3d v_lin(state.v[0], state.v[1], state.v[2]);
  const Eigen::Vector3d omega(state.v[3], state.v[4], state.v[5]);
  const Eigen::Vector3d v_world = R * v_lin;
  const Eigen::Vector3d w_world = R * omega;
  const double psi = std::atan2(R(1, 0), R(0, 0));
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  PlanarVelocity out;
  out.v_x = c * v_world.x() + s * v_world.y();
  out.v_y = -s * v_world.x() + c * v_world.y();
  out.omega_z = w_world.z();
  return out;
}

PhaseContext LocomotionPhaseManager::Context(const HorizonReference* horizon) const {
  PhaseContext ctx;
  ctx.phase_id = static_cast<int>(current_gait_.load(std::memory_order_relaxed));
  ctx.phase_changed = false;
  ctx.ocp_type = KinodynamicOCP::kType;
  ctx.cost_config.horizon_length = horizon_nodes_;
  ctx.cost_config.dt = dt_;
  ctx.horizon = horizon;
  return ctx;
}

PhaseContext LocomotionPhaseManager::Update(const MPCStateSnapshot& state,
                                            const Eigen::VectorXd& /*q*/,
                                            const Eigen::VectorXd& /*v*/,
                                            const Eigen::VectorXd& /*sensor*/,
                                            const pinocchio::SE3& /*tcp*/, double t) {
  if (!initialised_) {
    SetLastError(LocomotionPhaseError::kNotInitialised);
    return Context(nullptr);
  }
  command_slot_.LoadInto(command_);
  measured_ = MeasuredPlanarVelocity(state);
  if (!anchored_ && std::isfinite(t)) {
    grid_.anchor = t;
    anchored_ = true;
  }
  std::int64_t m = 0;
  if (!anchored_ || !NodeIndexAt(grid_, t, m)) {
    SetLastError(LocomotionPhaseError::kAdvanceFailed);  // a non-finite tick time
    return Context(nullptr);
  }
  const double t_q = grid_.Time(m);
  if (scheduler_.Advance(t_q, command_, measured_) != GaitSchedulerError::kNoError) {
    SetLastError(LocomotionPhaseError::kAdvanceFailed);
    return Context(nullptr);
  }
  current_gait_.store(scheduler_.current_gait(), std::memory_order_relaxed);
  if (builder_.Build(scheduler_, state, command_, grid_, m) != ReferenceError::kNoError) {
    SetLastError(LocomotionPhaseError::kReferenceBuildFailed);
    return Context(nullptr);
  }
  node_index_ = m;
  SetLastError(LocomotionPhaseError::kNoError);
  return GridContext(&builder_.active(), m, t - t_q);
}

PhaseContext LocomotionPhaseManager::GridContext(const HorizonReference* horizon,
                                                 std::int64_t index, double offset) const {
  PhaseContext ctx = Context(horizon);
  ctx.on_node_grid = true;
  ctx.node_index = index;
  ctx.horizon_offset = offset;
  return ctx;
}

PhaseContext LocomotionPhaseManager::Update(const Eigen::VectorXd& /*q*/,
                                            const Eigen::VectorXd& /*v*/,
                                            const Eigen::VectorXd& /*sensor*/,
                                            const pinocchio::SE3& /*tcp*/, double /*t*/) {
  SetLastError(initialised_ ? LocomotionPhaseError::kNoSnapshot
                            : LocomotionPhaseError::kNotInitialised);
  return Context(nullptr);
}

void LocomotionPhaseManager::SetTaskTarget(const YAML::Node& /*target*/) {}

int LocomotionPhaseManager::CurrentPhaseId() const {
  return static_cast<int>(current_gait_.load(std::memory_order_relaxed));
}

std::string LocomotionPhaseManager::CurrentPhaseName() const {
  const std::uint32_t i = current_gait_.load(std::memory_order_relaxed);
  return i < gait_names_.size() ? gait_names_[i] : std::string{};
}

void LocomotionPhaseManager::ForcePhase(int phase_id) {
  if (phase_id >= 0) {
    scheduler_.RequestGait(static_cast<std::uint32_t>(phase_id));
  }
}

}  // namespace rtc::mpc
