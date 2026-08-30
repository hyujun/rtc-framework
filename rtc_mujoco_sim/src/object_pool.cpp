// ── object_pool.cpp ───────────────────────────────────────────────────────────
// Implementation of the pre-compiled object pool. See object_pool.hpp for the
// design rationale (why a pool rather than mj_recompile, why parking needs all
// three of contact/gravcomp/velocity, why MuJoCo's sleep feature is not used).
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/object_pool.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace rtc {

namespace {

/// Uniform in [-|half_width|, +|half_width|].
///
/// Always draws, even for a zero-width axis. Skipping the draw would make the
/// RNG stream depend on WHICH axes vary, so adding a variation to one axis
/// would silently change the values sampled for the others — turning a
/// seeded-reproducible run into one that only looks reproducible.
double SampleSymmetric(double half_width, std::mt19937_64& rng) noexcept {
  const double h = std::fabs(half_width);
  std::uniform_real_distribution<double> dist(-h, h);
  return dist(rng);
}

}  // namespace

// ── Enum parsing ─────────────────────────────────────────────────────────────

bool ParseObjectSelection(std::string_view text, ObjectSelection& out) noexcept {
  if (text == "fixed") {
    out = ObjectSelection::kFixed;
    return true;
  }
  if (text == "random") {
    out = ObjectSelection::kRandom;
    return true;
  }
  return false;
}

bool ParsePoseSampling(std::string_view text, PoseSampling& out) noexcept {
  if (text == "fixed") {
    out = PoseSampling::kFixed;
    return true;
  }
  if (text == "random") {
    out = PoseSampling::kRandom;
    return true;
  }
  return false;
}

// ── Pure helpers ─────────────────────────────────────────────────────────────

std::vector<std::string> ScanObjectDirectory(const std::string& dir,
                                             const std::string& object_file,
                                             std::string& error) {
  error.clear();
  std::vector<std::string> names;

  if (dir.empty()) {
    error = "object_pool.directory is empty";
    return names;
  }

  std::error_code ec;
  const std::filesystem::path root(dir);
  if (!std::filesystem::is_directory(root, ec) || ec) {
    error = "object_pool.directory is not a directory: " + dir;
    return names;
  }

  for (std::filesystem::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
    if (!it->is_directory(ec) || ec) {
      ec.clear();
      continue;
    }
    std::error_code file_ec;
    if (!std::filesystem::exists(it->path() / object_file, file_ec) || file_ec) {
      continue;
    }
    names.push_back(it->path().filename().string());
  }
  if (ec) {
    error = "failed to scan object_pool.directory: " + dir + " (" + ec.message() + ")";
    names.clear();
    return names;
  }

  // See the header: directory_iterator order is unspecified, so an unsorted
  // list makes a fixed seed pick different objects on different filesystems.
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> ResolveCandidates(const std::vector<std::string>& found,
                                           const std::vector<std::string>& allowlist,
                                           std::string& error) {
  error.clear();
  if (allowlist.empty()) {
    return found;
  }

  std::vector<std::string> out;
  out.reserve(allowlist.size());
  for (const auto& want : allowlist) {
    if (std::find(found.begin(), found.end(), want) == found.end()) {
      error = "object_pool.objects lists '" + want + "', which was not found in the directory";
      return {};
    }
    if (std::find(out.begin(), out.end(), want) != out.end()) {
      continue;  // tolerate a duplicated entry; it names the same slot
    }
    out.push_back(want);
  }
  return out;
}

std::array<double, 4> RpyZyxToQuat(const std::array<double, 3>& rpy) noexcept {
  // "XYZ" is uppercase on purpose — see the header. Lowercase "zyx" with this
  // same argument order silently produces a ~1 rad different rotation.
  const mjtNum euler[3] = {rpy[0], rpy[1], rpy[2]};
  mjtNum q[4] = {1.0, 0.0, 0.0, 0.0};
  mju_euler2Quat(q, euler, "XYZ");
  mju_normalize4(q);
  return {q[0], q[1], q[2], q[3]};
}

SampledPose SampleObjectPose(const ObjectPoolConfig& cfg, std::mt19937_64& rng) noexcept {
  SampledPose out;
  if (cfg.pose == PoseSampling::kFixed) {
    out.position = cfg.position;
    out.quat = RpyZyxToQuat(cfg.rpy);
    return out;
  }

  for (std::size_t i = 0; i < 3; ++i) {
    out.position[i] = cfg.position[i] + SampleSymmetric(cfg.position_variation[i], rng);
  }
  std::array<double, 3> rpy{};
  for (std::size_t i = 0; i < 3; ++i) {
    rpy[i] = cfg.rpy[i] + SampleSymmetric(cfg.rpy_variation[i], rng);
  }
  out.quat = RpyZyxToQuat(rpy);
  return out;
}

std::size_t SelectObjectIndex(ObjectSelection mode, std::size_t count, std::size_t current,
                              std::size_t fixed_index, bool avoid_repeat,
                              std::mt19937_64& rng) noexcept {
  if (count == 0) {
    return 0;
  }
  if (mode == ObjectSelection::kFixed) {
    return std::min(fixed_index, count - 1);
  }

  const bool can_avoid = avoid_repeat && count > 1 && current < count;
  if (can_avoid) {
    // Draw over the count-1 survivors and shift past the excluded index. O(1)
    // and unbiased; a reject-and-retry loop would spin forever at count == 1,
    // which is why that case is filtered out above rather than inside a loop.
    std::uniform_int_distribution<std::size_t> dist(0, count - 2);
    std::size_t idx = dist(rng);
    if (idx >= current) {
      ++idx;
    }
    return idx;
  }

  std::uniform_int_distribution<std::size_t> dist(0, count - 1);
  return dist(rng);
}

// ── ObjectPool ───────────────────────────────────────────────────────────────

bool ObjectPool::Configure(const ObjectPoolConfig& cfg, std::string& error) {
  error.clear();
  cfg_ = cfg;
  names_.clear();
  slots_.clear();
  fixed_index_ = 0;
  SetActiveIndex(kNoActive);
  resolved_ = false;

  if (!cfg_.enabled) {
    return true;
  }

  std::string scan_error;
  const std::vector<std::string> found =
      ScanObjectDirectory(cfg_.directory, cfg_.object_file, scan_error);
  if (!scan_error.empty()) {
    error = scan_error;
    return false;
  }
  if (found.empty()) {
    error = "object_pool.enabled is true but no '" + cfg_.object_file + "' was found under " +
            cfg_.directory;
    return false;
  }

  std::string resolve_error;
  names_ = ResolveCandidates(found, cfg_.objects, resolve_error);
  if (!resolve_error.empty()) {
    error = resolve_error;
    return false;
  }
  if (names_.empty()) {
    error = "object_pool candidate list resolved to empty";
    return false;
  }

  if (!cfg_.fixed_object.empty()) {
    const auto it = std::find(names_.begin(), names_.end(), cfg_.fixed_object);
    if (it == names_.end()) {
      error = "object_pool.fixed_object '" + cfg_.fixed_object +
              "' is not among the resolved candidates";
      return false;
    }
    fixed_index_ = static_cast<std::size_t>(std::distance(names_.begin(), it));
  }

  if (cfg_.seed != 0) {
    rng_.seed(cfg_.seed);
  } else {
    std::random_device rd;
    rng_.seed((static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd()));
  }

  // Memory scales with the candidate count (measured ~610 MB RSS for the 57
  // object_sim meshes). Warn rather than cap: silently truncating the pool
  // would change which objects can appear without saying so.
  constexpr std::size_t kLargePool = 16;
  if (names_.size() > kLargePool) {
    fprintf(stdout,
            "[ObjectPool] NOTE: %zu candidates will all be compiled into the scene. "
            "Startup memory and compile time scale with this count; narrow it with "
            "object_pool.objects if that matters.\n",
            names_.size());
  }
  return true;
}

bool ObjectPool::AttachInto(mjSpec* spec, std::string& error) {
  error.clear();
  if (!cfg_.enabled || names_.empty()) {
    return true;
  }
  if (spec == nullptr) {
    error = "object_pool: null spec";
    return false;
  }

  mjsBody* world = mjs_findBody(spec, "world");
  if (world == nullptr) {
    error = "object_pool: scene has no worldbody";
    return false;
  }

  slots_.clear();
  slots_.reserve(names_.size());

  const std::filesystem::path root(cfg_.directory);
  for (const auto& name : names_) {
    const std::string path = (root / name / cfg_.object_file).string();

    char parse_error[512] = {};
    mjSpec* child = mj_parseXML(path.c_str(), nullptr, parse_error, sizeof(parse_error));
    if (child == nullptr) {
      error = "object_pool: failed to parse " + path + ": " + parse_error;
      return false;
    }
    mjsBody* source = mjs_findBody(child, cfg_.body_name.c_str());
    if (source == nullptr) {
      mj_deleteSpec(child);
      error = "object_pool: " + path + " has no body named '" + cfg_.body_name + "'";
      return false;
    }

    // A frame is the attach point MuJoCo documents for pulling a body out of
    // another spec; the prefix is what makes object_sim's generic mesh names
    // (contact0, contact1, ...) and its `grab`/`object_col` default classes
    // unique across candidates, and what keeps them clear of objects a scene
    // already hard-codes.
    const std::string prefix = cfg_.prefix + name + "_";
    mjsFrame* frame = mjs_addFrame(world, nullptr);
    if (frame == nullptr) {
      mj_deleteSpec(child);
      error = "object_pool: mjs_addFrame failed for " + name;
      return false;
    }
    mjsElement* attached = mjs_attach(frame->element, source->element, prefix.c_str(), "");
    if (attached == nullptr) {
      const char* why = mjs_getError(spec);
      mj_deleteSpec(child);
      error = "object_pool: mjs_attach failed for " + name + ": " + (why ? why : "(no detail)");
      return false;
    }
    mjsBody* body = mjs_asBody(attached);
    if (body == nullptr) {
      mj_deleteSpec(child);
      error = "object_pool: attached element for " + name + " is not a body";
      return false;
    }

    // The park position becomes this body's compiled qpos0, which is what lets
    // mj_resetData re-park the entire pool without any special handling.
    body->pos[0] = cfg_.park_position[0];
    body->pos[1] = cfg_.park_position[1];
    body->pos[2] = cfg_.park_position[2];

    // object_sim bodies carry no joint (they are meant to be included into a
    // scene that adds one), so the freejoint is ours to add.
    if (mjs_addFreeJoint(body) == nullptr) {
      mj_deleteSpec(child);
      error = "object_pool: mjs_addFreeJoint failed for " + name;
      return false;
    }

    // Verified safe on MuJoCo 3.7.0: mjs_attach deep-copies, so the child spec
    // is dead weight from here on.
    mj_deleteSpec(child);

    Slot slot;
    slot.name = name;
    slot.body_name = prefix + cfg_.body_name;
    slots_.push_back(std::move(slot));
  }

  fprintf(stdout, "[ObjectPool] attached %zu candidate object(s) from %s\n", slots_.size(),
          cfg_.directory.c_str());
  return true;
}

bool ObjectPool::Resolve(const mjModel* model, std::string& error) {
  error.clear();
  if (!cfg_.enabled || slots_.empty()) {
    return true;
  }
  if (model == nullptr) {
    error = "object_pool: null model";
    return false;
  }

  for (auto& slot : slots_) {
    slot.body_id = mj_name2id(model, mjOBJ_BODY, slot.body_name.c_str());
    if (slot.body_id < 0) {
      error = "object_pool: compiled model has no body '" + slot.body_name + "'";
      return false;
    }
    const int jnt = model->body_jntadr[slot.body_id];
    if (jnt < 0 || model->jnt_type[jnt] != mjJNT_FREE) {
      error = "object_pool: body '" + slot.body_name + "' has no freejoint";
      return false;
    }
    slot.qpos_adr = model->jnt_qposadr[jnt];
    slot.dof_adr = model->jnt_dofadr[jnt];
    slot.geom_begin = model->body_geomadr[slot.body_id];
    slot.geom_count = model->body_geomnum[slot.body_id];

    // Saved rather than assumed: object_sim marks visual geoms contype=0, so a
    // blanket restore to 1 would drag the visual shell into collision.
    slot.contype.clear();
    slot.conaffinity.clear();
    slot.contype.reserve(static_cast<std::size_t>(slot.geom_count));
    slot.conaffinity.reserve(static_cast<std::size_t>(slot.geom_count));
    for (int g = slot.geom_begin; g < slot.geom_begin + slot.geom_count; ++g) {
      slot.contype.push_back(model->geom_contype[g]);
      slot.conaffinity.push_back(model->geom_conaffinity[g]);
    }
  }

  resolved_ = true;
  SetActiveIndex(kNoActive);
  return true;
}

void ObjectPool::BakeParkIntoKeyframes(mjModel* model) const noexcept {
  if (!resolved_ || model == nullptr || model->nkey <= 0) {
    return;
  }
  const int nq = static_cast<int>(model->nq);
  for (int k = 0; k < model->nkey; ++k) {
    for (const auto& slot : slots_) {
      if (slot.qpos_adr < 0) {
        continue;
      }
      mjtNum* q = model->key_qpos + static_cast<std::ptrdiff_t>(k) * nq + slot.qpos_adr;
      q[0] = cfg_.park_position[0];
      q[1] = cfg_.park_position[1];
      q[2] = cfg_.park_position[2];
      q[3] = 1.0;
      q[4] = 0.0;
      q[5] = 0.0;
      q[6] = 0.0;
    }
  }
}

void ObjectPool::Park(mjModel* model, mjData* data, Slot& slot) const noexcept {
  if (slot.body_id < 0 || slot.qpos_adr < 0) {
    return;
  }
  model->body_gravcomp[slot.body_id] = 1.0;
  for (int k = 0; k < slot.geom_count; ++k) {
    model->geom_contype[slot.geom_begin + k] = 0;
    model->geom_conaffinity[slot.geom_begin + k] = 0;
  }

  mjtNum* q = data->qpos + slot.qpos_adr;
  q[0] = cfg_.park_position[0];
  q[1] = cfg_.park_position[1];
  q[2] = cfg_.park_position[2];
  q[3] = 1.0;
  q[4] = 0.0;
  q[5] = 0.0;
  q[6] = 0.0;

  // Velocity must go too. With collisions off there is nothing to stop a body
  // that was moving when it was parked, so gravcomp alone leaves it drifting at
  // constant velocity forever. qacc_warmstart is cleared for the same reason:
  // a stale warm start reintroduces the motion on the next solve.
  for (int i = 0; i < 6; ++i) {
    data->qvel[slot.dof_adr + i] = 0.0;
    data->qacc_warmstart[slot.dof_adr + i] = 0.0;
  }
  mju_zero(data->xfrc_applied + 6 * slot.body_id, 6);
}

void ObjectPool::Activate(mjModel* model, mjData* data, Slot& slot,
                          const SampledPose& pose) const noexcept {
  if (slot.body_id < 0 || slot.qpos_adr < 0) {
    return;
  }
  model->body_gravcomp[slot.body_id] = 0.0;
  for (int k = 0; k < slot.geom_count; ++k) {
    model->geom_contype[slot.geom_begin + k] = slot.contype[static_cast<std::size_t>(k)];
    model->geom_conaffinity[slot.geom_begin + k] = slot.conaffinity[static_cast<std::size_t>(k)];
  }

  mjtNum* q = data->qpos + slot.qpos_adr;
  q[0] = pose.position[0];
  q[1] = pose.position[1];
  q[2] = pose.position[2];
  q[3] = pose.quat[0];
  q[4] = pose.quat[1];
  q[5] = pose.quat[2];
  q[6] = pose.quat[3];

  for (int i = 0; i < 6; ++i) {
    data->qvel[slot.dof_adr + i] = 0.0;
    data->qacc_warmstart[slot.dof_adr + i] = 0.0;
  }
  mju_zero(data->xfrc_applied + 6 * slot.body_id, 6);
}

void ObjectPool::ParkAll(mjModel* model, mjData* data) noexcept {
  if (!resolved_ || model == nullptr || data == nullptr) {
    return;
  }
  for (auto& slot : slots_) {
    Park(model, data, slot);
  }
  SetActiveIndex(kNoActive);
}

void ObjectPool::Refresh(mjModel* model, mjData* data) noexcept {
  if (!Enabled() || !resolved_ || model == nullptr || data == nullptr) {
    return;
  }
  const std::size_t current = ActiveIndex();
  if (current != kNoActive) {
    Park(model, data, slots_[current]);
  }
  const std::size_t next = SelectObjectIndex(cfg_.selection, slots_.size(), current, fixed_index_,
                                             cfg_.avoid_repeat, rng_);
  last_pose_ = SampleObjectPose(cfg_, rng_);
  Activate(model, data, slots_[next], last_pose_);
  SetActiveIndex(next);

  fprintf(stdout, "[ObjectPool] spawned '%s' at (%.3f, %.3f, %.3f)\n", slots_[next].name.c_str(),
          last_pose_.position[0], last_pose_.position[1], last_pose_.position[2]);
}

void ObjectPool::Reapply(mjModel* model, mjData* data) noexcept {
  if (!Enabled() || !resolved_ || model == nullptr || data == nullptr) {
    return;
  }
  const std::size_t keep = ActiveIndex();
  ParkAll(model, data);
  if (keep == kNoActive) {
    return;
  }
  Activate(model, data, slots_[keep], last_pose_);
  SetActiveIndex(keep);
}

const std::string& ObjectPool::ActiveName() const noexcept {
  static const std::string kNone = "none";
  const std::size_t index = ActiveIndex();
  if (index == kNoActive || index >= slots_.size()) {
    return kNone;
  }
  return slots_[index].name;
}

int ObjectPool::ActiveBodyId() const noexcept {
  const std::size_t index = ActiveIndex();
  if (index == kNoActive || index >= slots_.size()) {
    return -1;
  }
  return slots_[index].body_id;
}

int ObjectPool::BodyIdAt(std::size_t index) const noexcept {
  return index < slots_.size() ? slots_[index].body_id : -1;
}

int ObjectPool::QposAdrAt(std::size_t index) const noexcept {
  return index < slots_.size() ? slots_[index].qpos_adr : -1;
}

}  // namespace rtc
