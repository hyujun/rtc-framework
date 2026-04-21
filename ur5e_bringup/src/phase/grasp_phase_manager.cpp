#include "ur5e_bringup/phase/grasp_phase_manager.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

namespace ur5e_bringup::phase {

namespace {

// SSO guard — PhaseContext::phase_name is copied by value each Update tick.
// libstdc++ inlines strings up to 15 chars; violation would allocate per
// tick and break the Phase 6 alloc tracer when the grasp FSM is wired in.
static_assert(sizeof("idle") <= 16, "phase name 'idle' exceeds SSO");
static_assert(sizeof("approach") <= 16, "phase name 'approach' exceeds SSO");
static_assert(sizeof("pre_grasp") <= 16, "phase name 'pre_grasp' exceeds SSO");
static_assert(sizeof("closure") <= 16, "phase name 'closure' exceeds SSO");
static_assert(sizeof("hold") <= 16, "phase name 'hold' exceeds SSO");
static_assert(sizeof("manipulate") <= 16,
              "phase name 'manipulate' exceeds SSO");
static_assert(sizeof("retreat") <= 16, "phase name 'retreat' exceeds SSO");
static_assert(sizeof("release") <= 16, "phase name 'release' exceeds SSO");
static_assert(sizeof("light_contact") <= 16,
              "ocp_type 'light_contact' exceeds SSO");
static_assert(sizeof("contact_rich") <= 16,
              "ocp_type 'contact_rich' exceeds SSO");

constexpr std::array<const char *, kNumGraspPhases> kPhaseNames = {
    "idle", "approach",   "pre_grasp", "closure",
    "hold", "manipulate", "retreat",   "release",
};

double Distance(const pinocchio::SE3 &a, const pinocchio::SE3 &b) noexcept {
  return (a.translation() - b.translation()).norm();
}

bool IsValidOcpType(const std::string &s) noexcept {
  return s == "light_contact" || s == "contact_rich";
}

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

} // namespace

GraspPhaseManager::GraspPhaseManager(
    const rtc::mpc::RobotModelHandler &model) noexcept
    : model_(model) {}

std::string_view GraspPhaseManager::NameFor(int id) noexcept {
  if (id < 0 || id >= kNumGraspPhases) {
    return "";
  }
  return kPhaseNames[static_cast<std::size_t>(id)];
}

std::string_view GraspPhaseManager::OcpTypeFor(int id) const noexcept {
  if (!initialised_ || id < 0 || id >= kNumGraspPhases) {
    return "";
  }
  return phases_[static_cast<std::size_t>(id)].ocp_type;
}

GraspPhaseInitError
GraspPhaseManager::LoadTransition(const YAML::Node &cfg,
                                  GraspTransitionConfig &out) const noexcept {
  if (!cfg.IsDefined() || !cfg.IsMap()) {
    return GraspPhaseInitError::kInvalidYamlSchema;
  }
  GraspTransitionConfig tmp{};
  if (!TryReadScalar<double>(cfg["approach_tolerance"],
                             tmp.approach_tolerance) ||
      !TryReadScalar<double>(cfg["pregrasp_tolerance"],
                             tmp.pregrasp_tolerance) ||
      !TryReadScalar<double>(cfg["force_threshold"], tmp.force_threshold) ||
      !TryReadScalar<int>(cfg["max_failures"], tmp.max_failures)) {
    return GraspPhaseInitError::kInvalidYamlSchema;
  }
  if (tmp.approach_tolerance <= 0.0 || tmp.pregrasp_tolerance <= 0.0 ||
      tmp.force_threshold < 0.0 || tmp.max_failures <= 0) {
    return GraspPhaseInitError::kInvalidThreshold;
  }
  out = tmp;
  return GraspPhaseInitError::kNoError;
}

GraspPhaseInitError GraspPhaseManager::LoadPhases(
    const YAML::Node &phases_cfg,
    std::array<PhaseSlot, kNumGraspPhases> &out) const noexcept {
  if (!phases_cfg.IsDefined() || !phases_cfg.IsMap()) {
    return GraspPhaseInitError::kInvalidYamlSchema;
  }

  std::array<PhaseSlot, kNumGraspPhases> tmp{};

  for (int i = 0; i < kNumGraspPhases; ++i) {
    const auto &name = kPhaseNames[static_cast<std::size_t>(i)];
    const auto node = phases_cfg[name];
    if (!node.IsDefined() || !node.IsMap()) {
      return GraspPhaseInitError::kMissingPhase;
    }

    // ocp_type
    std::string ocp_type;
    if (!TryReadScalar<std::string>(node["ocp_type"], ocp_type)) {
      return GraspPhaseInitError::kInvalidYamlSchema;
    }
    if (!IsValidOcpType(ocp_type)) {
      return GraspPhaseInitError::kInvalidOcpType;
    }

    // active_contact_indices (indices into model.contact_frames())
    const auto idx_node = node["active_contact_indices"];
    std::vector<int> active_ids;
    if (idx_node.IsDefined()) {
      if (!idx_node.IsSequence()) {
        return GraspPhaseInitError::kInvalidYamlSchema;
      }
      try {
        for (const auto &elt : idx_node) {
          active_ids.push_back(elt.as<int>());
        }
      } catch (...) {
        return GraspPhaseInitError::kInvalidYamlSchema;
      }
    }

    const auto &frames = model_.contact_frames();
    for (int id : active_ids) {
      if (id < 0 || id >= static_cast<int>(frames.size())) {
        return GraspPhaseInitError::kInvalidContactIndex;
      }
    }

    // Build ContactPlan: all model frames known; phases declares the active
    // subset for this grasp-phase as a single infinite interval. Downstream
    // consumers (rtc_mpc OCP handlers) pick `frames[j]` when
    // `j ∈ active_frame_ids`.
    rtc::mpc::ContactPlan plan{};
    plan.frames = frames;
    rtc::mpc::ContactPhase cp{};
    cp.t_start = 0.0;
    cp.t_end = std::numeric_limits<double>::infinity();
    cp.active_frame_ids.reserve(active_ids.size());
    for (int id : active_ids) {
      cp.active_frame_ids.push_back(
          frames[static_cast<std::size_t>(id)].frame_id);
    }
    plan.phases.push_back(std::move(cp));

    // cost
    rtc::mpc::PhaseCostConfig cost;
    const auto cost_node = node["cost"];
    if (!cost_node.IsDefined() || !cost_node.IsMap()) {
      return GraspPhaseInitError::kInvalidYamlSchema;
    }
    const auto cost_err =
        rtc::mpc::PhaseCostConfig::LoadFromYaml(cost_node, model_, cost);
    if (cost_err != rtc::mpc::PhaseCostConfigError::kNoError) {
      return GraspPhaseInitError::kInvalidPhaseCost;
    }

    auto &slot = tmp[static_cast<std::size_t>(i)];
    slot.ocp_type = std::move(ocp_type);
    slot.cost = std::move(cost);
    slot.contact_plan = std::move(plan);
  }

  out = std::move(tmp);
  return GraspPhaseInitError::kNoError;
}

GraspPhaseInitError GraspPhaseManager::Load(const YAML::Node &cfg) noexcept {
  initialised_ = false;
  if (!model_.Initialised()) {
    return GraspPhaseInitError::kModelNotInitialised;
  }
  if (!cfg.IsDefined() || !cfg.IsMap()) {
    return GraspPhaseInitError::kInvalidYamlSchema;
  }

  GraspTransitionConfig t{};
  if (const auto err = LoadTransition(cfg["transition"], t);
      err != GraspPhaseInitError::kNoError) {
    return err;
  }

  std::array<PhaseSlot, kNumGraspPhases> slots{};
  if (const auto err = LoadPhases(cfg["phases"], slots);
      err != GraspPhaseInitError::kNoError) {
    return err;
  }

  thresholds_ = t;
  phases_ = std::move(slots);
  initialised_ = true;

  current_id_.store(static_cast<int>(GraspPhaseId::kIdle),
                    std::memory_order_relaxed);
  forced_phase_id_.store(kNoForcedPhase, std::memory_order_relaxed);
  command_.store(static_cast<int>(GraspCommand::kNone),
                 std::memory_order_relaxed);
  failure_count_ = 0;

  return GraspPhaseInitError::kNoError;
}

void GraspPhaseManager::Init(const YAML::Node &cfg) {
  const auto err = Load(cfg);
  if (err != GraspPhaseInitError::kNoError) {
    throw std::runtime_error(
        "GraspPhaseManager::Init failed, error code " +
        std::to_string(static_cast<int>(err)) +
        " — inspect YAML; see GraspPhaseInitError in grasp_phase_manager.hpp");
  }
}

void GraspPhaseManager::SetTaskTarget(const YAML::Node &target) {
  if (!target.IsDefined() || !target.IsMap()) {
    std::fprintf(stderr,
                 "[grasp_phase] SetTaskTarget: expected YAML map; ignored\n");
    return;
  }

  GraspTarget t{};
  try {
    const auto tr = target["grasp_translation"];
    if (!tr.IsDefined() || !tr.IsSequence() || tr.size() != 3) {
      std::fprintf(stderr, "[grasp_phase] SetTaskTarget: grasp_translation "
                           "must be 3-vector; ignored\n");
      return;
    }
    Eigen::Vector3d p{tr[0].as<double>(), tr[1].as<double>(),
                      tr[2].as<double>()};

    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    const auto rot = target["grasp_rotation"];
    if (rot.IsDefined()) {
      if (!rot.IsSequence() || rot.size() != 4) {
        std::fprintf(stderr, "[grasp_phase] SetTaskTarget: grasp_rotation "
                             "must be 4-vector (wxyz); ignored\n");
        return;
      }
      q = Eigen::Quaterniond(rot[0].as<double>(), rot[1].as<double>(),
                             rot[2].as<double>(), rot[3].as<double>());
      q.normalize();
    }
    t.grasp_pose = pinocchio::SE3(q.toRotationMatrix(), p);

    Eigen::Vector3d offset_local = Eigen::Vector3d::Zero();
    const auto off = target["pregrasp_offset_local"];
    if (off.IsDefined()) {
      if (!off.IsSequence() || off.size() != 3) {
        std::fprintf(stderr, "[grasp_phase] SetTaskTarget: "
                             "pregrasp_offset_local must be 3-vector; "
                             "ignored\n");
        return;
      }
      offset_local = Eigen::Vector3d{off[0].as<double>(), off[1].as<double>(),
                                     off[2].as<double>()};
    }
    pinocchio::SE3 off_se3(Eigen::Matrix3d::Identity(), offset_local);
    t.pregrasp_pose = t.grasp_pose.act(off_se3);

    // approach_start is snapshotted from TCP at IDLE→APPROACH edge; use
    // pregrasp as a safe default until then.
    t.approach_start = t.pregrasp_pose;
  } catch (...) {
    std::fprintf(stderr,
                 "[grasp_phase] SetTaskTarget: YAML parse failed; ignored\n");
    return;
  }

  SetTaskTarget(t);
}

void GraspPhaseManager::SetTaskTarget(const GraspTarget &target) noexcept {
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_ = target;
  has_target_.store(true, std::memory_order_release);
}

void GraspPhaseManager::SetCommand(GraspCommand cmd) noexcept {
  command_.store(static_cast<int>(cmd), std::memory_order_release);
}

int GraspPhaseManager::CurrentPhaseId() const {
  return current_id_.load(std::memory_order_relaxed);
}

std::string GraspPhaseManager::CurrentPhaseName() const {
  return std::string{NameFor(CurrentPhaseId())};
}

void GraspPhaseManager::ForcePhase(int phase_id) {
  if (phase_id < 0 || phase_id >= kNumGraspPhases) {
    return;
  }
  forced_phase_id_.store(phase_id, std::memory_order_release);
}

int GraspPhaseManager::EvaluateTransition(int current_id,
                                          const pinocchio::SE3 &tcp,
                                          const Eigen::VectorXd &sensor,
                                          GraspCommand cmd) noexcept {
  if (cmd == GraspCommand::kAbort) {
    failure_count_ = 0;
    return static_cast<int>(GraspPhaseId::kIdle);
  }

  const auto id = static_cast<GraspPhaseId>(current_id);

  // Snapshot target once per tick (non-RT mutex on the MPC thread — phase
  // transitions are off the 500 Hz RT path).
  GraspTarget tgt;
  bool have_target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    tgt = target_;
    have_target = has_target_.load(std::memory_order_acquire);
  }

  switch (id) {
  case GraspPhaseId::kIdle:
    if (cmd == GraspCommand::kApproach && have_target) {
      // Snapshot TCP as approach_start so RETREAT can come back to it.
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_.approach_start = tcp;
      }
      failure_count_ = 0;
      return static_cast<int>(GraspPhaseId::kApproach);
    }
    break;

  case GraspPhaseId::kApproach:
    if (have_target &&
        Distance(tcp, tgt.pregrasp_pose) < thresholds_.approach_tolerance) {
      return static_cast<int>(GraspPhaseId::kPreGrasp);
    }
    break;

  case GraspPhaseId::kPreGrasp:
    if (have_target &&
        Distance(tcp, tgt.grasp_pose) < thresholds_.pregrasp_tolerance) {
      failure_count_ = 0;
      return static_cast<int>(GraspPhaseId::kClosure);
    }
    break;

  case GraspPhaseId::kClosure: {
    double force_sum = 0.0;
    for (Eigen::Index i = 0; i < sensor.size(); ++i) {
      force_sum += sensor[i];
    }
    if (force_sum > thresholds_.force_threshold) {
      failure_count_ = 0;
      return static_cast<int>(GraspPhaseId::kHold);
    }
    ++failure_count_;
    if (failure_count_ >= thresholds_.max_failures) {
      failure_count_ = 0;
      return static_cast<int>(GraspPhaseId::kIdle);
    }
    break;
  }

  case GraspPhaseId::kHold:
    if (cmd == GraspCommand::kManipulate) {
      return static_cast<int>(GraspPhaseId::kManipulate);
    }
    break;

  case GraspPhaseId::kManipulate:
    if (cmd == GraspCommand::kRetreat) {
      return static_cast<int>(GraspPhaseId::kRetreat);
    }
    break;

  case GraspPhaseId::kRetreat:
    if (have_target &&
        Distance(tcp, tgt.approach_start) < thresholds_.approach_tolerance) {
      return static_cast<int>(GraspPhaseId::kRelease);
    }
    break;

  case GraspPhaseId::kRelease:
    if (cmd == GraspCommand::kRelease) {
      has_target_.store(false, std::memory_order_release);
      return static_cast<int>(GraspPhaseId::kIdle);
    }
    break;
  }

  return current_id;
}

rtc::mpc::PhaseContext GraspPhaseManager::BuildContext(int id,
                                                       bool changed) const {
  rtc::mpc::PhaseContext ctx{};
  ctx.phase_id = id;
  ctx.phase_name = NameFor(id);
  ctx.phase_changed = changed;

  if (!initialised_ || id < 0 || id >= kNumGraspPhases) {
    // Safe default: empty light-contact context. Consumers should not hit
    // this unless Init failed silently (which we guard against above).
    ctx.ocp_type = "light_contact";
    return ctx;
  }

  const auto &slot = phases_[static_cast<std::size_t>(id)];
  ctx.contact_plan = slot.contact_plan;
  ctx.cost_config = slot.cost;
  ctx.ocp_type = slot.ocp_type;

  // EE target: grasp_pose for CLOSURE/HOLD/MANIPULATE, pregrasp_pose for
  // APPROACH/PRE_GRASP, approach_start for RETREAT/RELEASE, identity for
  // IDLE.
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (has_target_.load(std::memory_order_acquire)) {
      switch (static_cast<GraspPhaseId>(id)) {
      case GraspPhaseId::kApproach:
      case GraspPhaseId::kPreGrasp:
        ctx.ee_target = target_.pregrasp_pose;
        break;
      case GraspPhaseId::kClosure:
      case GraspPhaseId::kHold:
      case GraspPhaseId::kManipulate:
        ctx.ee_target = target_.grasp_pose;
        break;
      case GraspPhaseId::kRetreat:
      case GraspPhaseId::kRelease:
        ctx.ee_target = target_.approach_start;
        break;
      case GraspPhaseId::kIdle:
      default:
        ctx.ee_target = pinocchio::SE3::Identity();
        break;
      }
    }
  }

  return ctx;
}

rtc::mpc::PhaseContext GraspPhaseManager::Update(const Eigen::VectorXd & /*q*/,
                                                 const Eigen::VectorXd & /*v*/,
                                                 const Eigen::VectorXd &sensor,
                                                 const pinocchio::SE3 &tcp,
                                                 double /*t*/) {
  if (!initialised_) {
    // Return a safe empty context; BuildContext handles the uninit fallback.
    return BuildContext(static_cast<int>(GraspPhaseId::kIdle), false);
  }

  const int prev_id = current_id_.load(std::memory_order_relaxed);

  // Forced override wins over guard-based transitions.
  const int forced =
      forced_phase_id_.exchange(kNoForcedPhase, std::memory_order_acq_rel);
  int next_id = prev_id;
  if (forced != kNoForcedPhase && forced >= 0 && forced < kNumGraspPhases) {
    next_id = forced;
    failure_count_ = 0;
  } else {
    const auto cmd =
        static_cast<GraspCommand>(command_.load(std::memory_order_acquire));
    next_id = EvaluateTransition(prev_id, tcp, sensor, cmd);
    // One-shot commands clear on consumption so the FSM doesn't re-trigger.
    if (next_id != prev_id && cmd != GraspCommand::kNone) {
      command_.store(static_cast<int>(GraspCommand::kNone),
                     std::memory_order_release);
    }
  }

  const bool changed = (next_id != prev_id);
  if (changed) {
    current_id_.store(next_id, std::memory_order_relaxed);
  }

  return BuildContext(next_id, changed);
}

} // namespace ur5e_bringup::phase
