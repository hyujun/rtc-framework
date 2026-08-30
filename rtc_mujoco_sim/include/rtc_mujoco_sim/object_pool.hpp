// ── object_pool.hpp ───────────────────────────────────────────────────────────
// Randomised object spawning for the MuJoCo scene ("domain randomisation" of
// the manipulated object).
//
// WHAT THIS REPLACES. Scenes that carry manipulable objects have had to spell
// out, in a comment, the five manual steps for adding one by hand: copy the
// mesh entries, prefix every mesh name (object datasets reuse generic names
// like contact0 across every object, so unprefixed ones collide), duplicate the
// body block, and append seven qpos values to every keyframe. This file does
// all four programmatically at load time, so a scene declares a DIRECTORY
// instead of a hand-maintained object list. The package README works one such
// scene through, with the concrete path; naming it here would put a
// robot-specific asset path in an agnostic header (ARCH-1) and would rot the
// day that scene is renamed.
//
// HOW IT WORKS — a pre-compiled pool, not a runtime recompile. Every candidate
// object is attached into the spec BEFORE mj_compile, so mjModel/mjData are
// still immutable-after-Initialize in the sense the rest of the class relies on
// (MuJoCoSimulator::FindBodyId advertises "reads the immutable compiled model
// only"). A refresh then just parks the active object and unparks another one.
//
// The rejected alternative was mj_recompile() on every refresh — a true
// insert/delete. It costs: a SimLoop<->ViewerLoop barrier (mjvScene/mjrContext/
// vis_data must be rebuilt on the thread holding the GL context), re-resolution
// of every cached index (group qpos/actuator/body, sensor adr, contact wrench
// ids), and reallocation of ext_xfrc_/orig_actuator_params_. The pool costs
// startup memory proportional to the candidate count. Measured on MuJoCo 3.7.0
// against a 16-dof arm-plus-hand scene (the scene's own dof count barely moves
// these numbers — the pool's free bodies dominate): 57 objects = +0.072 ms/step
// (0.014 -> 0.086), +0.14 s compile, ~610 MB RSS. At 500 Hz with n_substeps=3
// that is 13% of the 2 ms budget, so the pool wins on every axis except memory.
// The package README carries the same table with the scene named.
//
// PARKING. A parked object gets contype/conaffinity 0 (out of collision),
// body_gravcomp 1 (gravity cancelled), and its freejoint state zeroed. All
// three are load-bearing:
//   - contype alone: the object keeps falling forever, and since nothing can
//     stop it the velocity grows without bound.
//   - contype + gravcomp, but velocity left alone: pressing 'o' while the
//     object is mid-fall parks it with residual momentum and it DRIFTS at
//     constant velocity, because collisions are off. Measured park drift is
//     exactly 0 only when qvel and qacc_warmstart are both cleared.
// Park POSITION additionally goes into the spec body pos, which makes it the
// compiled qpos0 — so mj_resetData (the R key) re-parks the whole pool for
// free, and only the active object needs re-applying.
//
// NOT USED: MuJoCo 3.7 per-tree sleep (mjSLEEP_INIT). It would park objects
// more cheaply, but mjENBL_SLEEP is a GLOBAL enable bit: turning it on lets the
// robot tree sleep too (every robot tree would need an explicit mjSLEEP_NEVER),
// it interacts with the `island` solver option this package exposes in YAML,
// and it exists only in 3.7 while this package supports MuJoCo 3.x. At
// 0.086 ms/step the optimisation buys nothing worth that blast radius.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_OBJECT_POOL_HPP_
#define RTC_MUJOCO_SIM_OBJECT_POOL_HPP_

#include <mujoco/mujoco.h>

// The attach path (mj_parseXML -> mjs_attach -> mjs_addFreeJoint -> mj_compile)
// is mjSpec-era API, newer than the "MuJoCo 3.x" the package README used to
// claim. 3.x spells the macro with seven digits (3.7.0 -> 3007000); the 3-digit
// spelling belongs to 2.x and must not be compared against.
#if defined(mjVERSION_HEADER) && mjVERSION_HEADER < 3002000
#error "rtc_mujoco_sim object pool requires MuJoCo >= 3.2 (mjSpec attach API)"
#endif

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace rtc {

// ── Sampling axes ────────────────────────────────────────────────────────────
// The four modes users ask for ("same object + same pose", "same object +
// random pose", "random object + fixed pose", "random object + random pose")
// are the product of two INDEPENDENT binary axes, so they are stored that way
// rather than as one four-valued enum. A single enum would have to grow
// combinatorially the first time a third axis (scale, colour, mass) appears,
// and -Werror=switch turns every such growth into a repo-wide edit.
enum class ObjectSelection { kFixed, kRandom };
enum class PoseSampling { kFixed, kRandom };

/// Parse the YAML spelling. Returns false on an unknown string rather than
/// falling back to a default: a typo'd mode that silently means "fixed" is a
/// randomisation harness that reports green while randomising nothing.
[[nodiscard]] bool ParseObjectSelection(std::string_view text, ObjectSelection& out) noexcept;
[[nodiscard]] bool ParsePoseSampling(std::string_view text, PoseSampling& out) noexcept;

// ── ObjectPoolConfig ─────────────────────────────────────────────────────────
// Robot-agnostic by construction (ARCH-1): no object name, directory, or pose
// has a default that names a robot or a dataset. `directory` is required when
// enabled, and everything under it is discovered.
struct ObjectPoolConfig {
  bool enabled{false};

  /// Directory scanned for `<name>/<object_file>`. May be a "package://" URI;
  /// the caller resolves it (ResolveModelPath) before handing it over.
  std::string directory;

  /// Optional allowlist of subdirectory names. Empty = every object found.
  /// A name that is not present is an error, never a silent skip.
  std::vector<std::string> objects;

  /// Which object to use when selection == kFixed. Empty = the first candidate
  /// in sorted order.
  std::string fixed_object;

  ObjectSelection selection{ObjectSelection::kFixed};
  PoseSampling pose{PoseSampling::kFixed};

  /// Spawn pose centre, metres, world frame.
  std::array<double, 3> position{0.0, 0.0, 0.2};
  /// Half-width of the uniform position range, metres. Sampled per axis in
  /// [position - variation, position + variation]. Ignored when pose == kFixed.
  std::array<double, 3> position_variation{0.0, 0.0, 0.0};

  /// Spawn orientation centre as ZYX Euler (roll, pitch, yaw), RADIANS.
  /// Radians, not degrees, per CLAUDE.md §10 (SI everywhere; degrees only at an
  /// API boundary). A parallel `_deg` key was considered and dropped: two
  /// spellings of one quantity is the drift this repo keeps paying for.
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
  /// Half-width of the uniform orientation range, radians, per axis.
  /// The common case is yaw-only: [0, 0, pi].
  std::array<double, 3> rpy_variation{0.0, 0.0, 0.0};

  /// Where inactive objects wait. Must be clear of the robot's workspace and
  /// of the floor; it is baked into qpos0, so it is also where mj_resetData
  /// puts everything.
  std::array<double, 3> park_position{0.0, 0.0, -50.0};

  /// 0 = seed from std::random_device (non-reproducible). Non-zero = fixed
  /// seed, which is what makes a randomised run reproducible.
  std::uint64_t seed{0};

  /// Never select the currently active object twice in a row (no-op when only
  /// one candidate exists, which must not spin).
  bool avoid_repeat{true};

  /// Spawn once during Initialize so the scene is not empty before the first
  /// key press.
  bool spawn_on_start{true};

  /// File to look for inside each subdirectory, and the body inside it to
  /// attach. Defaults match the object_sim layout but name no dataset.
  std::string object_file{"object.xml"};
  std::string body_name{"object"};

  /// Prefix applied to every attached element. Keeps pool bodies from
  /// colliding with objects a scene already hard-codes, and is what makes the
  /// generic mesh names (contact0, contact1, ...) unique across candidates.
  std::string prefix{"pool_"};
};

// ── SampledPose ──────────────────────────────────────────────────────────────
struct SampledPose {
  std::array<double, 3> position{0.0, 0.0, 0.0};
  /// MuJoCo/Hamilton order (w, x, y, z).
  std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};
};

// ── Pure helpers (no MuJoCo model state — unit-testable without a fixture) ────

/// Subdirectories of `dir` that contain `object_file`, SORTED.
///
/// The sort is not cosmetic. std::filesystem::directory_iterator yields an
/// unspecified order, so an unsorted scan makes the candidate list depend on
/// the filesystem — and a fixed `seed` would then pick a different object on a
/// different machine while every test still passed on the one that wrote them.
/// On failure `error` is set and the result is empty.
[[nodiscard]] std::vector<std::string> ScanObjectDirectory(const std::string& dir,
                                                           const std::string& object_file,
                                                           std::string& error);

/// Intersect `found` with `allowlist` (empty allowlist = take all), preserving
/// allowlist order when given. Any allowlist entry missing from `found` is an
/// error — a mistyped object name must not quietly shrink the pool.
[[nodiscard]] std::vector<std::string> ResolveCandidates(const std::vector<std::string>& found,
                                                         const std::vector<std::string>& allowlist,
                                                         std::string& error);

/// ZYX Euler (roll, pitch, yaw) in radians -> Hamilton quaternion (w, x, y, z),
/// matching rtc_math::RpyToRotationZyx, i.e. R = Rz(yaw)*Ry(pitch)*Rx(roll).
///
/// Delegates to MuJoCo's mju_euler2Quat rather than re-deriving the three
/// AngleAxis lines a third time in this repo (design-principles P5; the
/// rtc_math version exists precisely because that derivation had already been
/// written twice with different spellings). Using rtc_math directly here would
/// drag Eigen into a package that has none, for one conversion.
///
/// The sequence string is "XYZ" — UPPERCASE, meaning intrinsic — with the
/// angles in (roll, pitch, yaw) order. This is not the spelling one guesses:
/// lowercase "zyx" with the same argument order is wrong by ~1 rad and still
/// returns a perfectly plausible rotation. The equivalence is pinned by a test
/// using an asymmetric angle triple, which is the only thing that separates a
/// sequence mistake from a component transposition.
[[nodiscard]] std::array<double, 4> RpyZyxToQuat(const std::array<double, 3>& rpy) noexcept;

/// Sample a spawn pose. kFixed returns the configured centre exactly.
[[nodiscard]] SampledPose SampleObjectPose(const ObjectPoolConfig& cfg,
                                           std::mt19937_64& rng) noexcept;

/// Pick the next object index.
///   kFixed  -> always `fixed_index`.
///   kRandom -> uniform over [0, count); with `avoid_repeat` the previous index
///              is excluded, which degrades to "repeat allowed" when count < 2
///              rather than spinning.
/// Returns 0 when count == 0 (callers reject an empty pool before this).
[[nodiscard]] std::size_t SelectObjectIndex(ObjectSelection mode, std::size_t count,
                                            std::size_t current, std::size_t fixed_index,
                                            bool avoid_repeat, std::mt19937_64& rng) noexcept;

// ── ObjectPool ───────────────────────────────────────────────────────────────
// Owns the candidate list, the attached slots, and the RNG.
//
// THREADING. Every non-const method mutates mjModel/mjData and must therefore
// run on the SimLoop thread (or on a caller that owns the simulator outright,
// i.e. before Start() or from the *ForTest entry points). This matches the
// ownership the rest of MuJoCoSimulator already assumes: SimLoop is the sole
// writer of model_/data_.
//
// Like the existing SetControlMode / EnableWorldGravity paths, the fields
// written here (geom_contype, geom_conaffinity, body_gravcomp) are read by the
// viewer thread inside mjv_updateScene without a lock. That is a pre-existing
// property of this class, not something the pool introduces — but it is why
// geom_group is deliberately NOT touched: rendering the parked geoms was
// measured at 0.0024 ms/frame for 57 objects, so hiding them would add mutation
// surface for no gain.
class ObjectPool {
 public:
  ObjectPool() = default;

  ObjectPool(const ObjectPool&) = delete;
  ObjectPool& operator=(const ObjectPool&) = delete;
  ObjectPool(ObjectPool&&) = delete;
  ObjectPool& operator=(ObjectPool&&) = delete;

  /// Scan the directory, resolve the allowlist and the fixed object, seed the
  /// RNG. No MuJoCo involvement yet. Returns false with `error` set when the
  /// pool is enabled but unusable — an enabled-but-empty pool is a randomiser
  /// that reports success while doing nothing, so it must not be reachable.
  /// A disabled pool always succeeds and stays inert.
  [[nodiscard]] bool Configure(const ObjectPoolConfig& cfg, std::string& error);

  /// Attach one body per candidate under the spec's worldbody, each with a
  /// freejoint and its body pos set to `park_position` (which becomes qpos0).
  /// Call between mj_parseXML and mj_compile. No-op when disabled.
  [[nodiscard]] bool AttachInto(mjSpec* spec, std::string& error);

  /// Resolve names to ids and cache each slot's original contact filters.
  /// Call once after mj_compile.
  [[nodiscard]] bool Resolve(const mjModel* model, std::string& error);

  /// Overwrite every keyframe's pool-slot qpos with the park pose.
  ///
  /// mjs_attach pads existing keyframes to the new nq with (0,0,0, 1,0,0,0) —
  /// a valid identity quaternion, but at the WORLD ORIGIN rather than at the
  /// park position. Nothing in this package calls mj_resetDataKeyframe today,
  /// so the mismatch is latent; it is fixed here anyway because the scene
  /// comment this feature replaces warns about exactly this failure, and
  /// because a keyframe reset dropping 57 objects on the robot is not a
  /// failure mode worth leaving armed.
  void BakeParkIntoKeyframes(mjModel* model) const noexcept;

  /// Park every slot (no active object afterwards).
  void ParkAll(mjModel* model, mjData* data) noexcept;

  /// Park the active object, choose the next one per the configured axes, and
  /// activate it at a freshly sampled pose. This is what the 'o' key runs.
  void Refresh(mjModel* model, mjData* data) noexcept;

  /// Re-activate the last spawn (same object, same pose) without consuming RNG.
  /// Used after mj_resetData, whose qpos0 has just parked everything: without
  /// this, pressing R would silently empty the scene.
  void Reapply(mjModel* model, mjData* data) noexcept;

  // ── Observers ───────────────────────────────────────────────────────────
  // Every accessor below is safe to call from a thread other than the one
  // running Refresh/Reapply/ParkAll (in practice: the viewer thread reading
  // the status overlay every frame). Two things make that true, and both are
  // load-bearing:
  //   - cfg_, names_ and slots_ are immutable after Resolve(), so the sizes,
  //     names and ids they return never move under a reader;
  //   - active_, the one field the refresh path writes, is atomic.
  // A concurrent reader can therefore observe a *stale* active object for a
  // frame, but never a torn index and never an out-of-range slot. Mutating
  // entry points (Configure/AttachInto/Resolve/Park*/Refresh/Reapply) stay
  // single-threaded and SimLoop-owned.
  [[nodiscard]] bool Enabled() const noexcept { return cfg_.enabled && !slots_.empty(); }

  [[nodiscard]] std::size_t Size() const noexcept { return slots_.size(); }

  [[nodiscard]] bool HasActive() const noexcept { return ActiveIndex() != kNoActive; }

  /// Name of the active object, or "none". Surfaced in the viewer status
  /// overlay — the only observable that distinguishes "refresh worked" from
  /// "refresh silently did nothing" during a GUI check. Read every frame by
  /// the viewer thread; see the concurrency note above.
  [[nodiscard]] const std::string& ActiveName() const noexcept;

  [[nodiscard]] const std::vector<std::string>& CandidateNames() const noexcept { return names_; }

  /// Body id of the active object, or -1. Test/observer use.
  [[nodiscard]] int ActiveBodyId() const noexcept;
  /// Body id of candidate `index`, or -1 when out of range. Test/observer use.
  [[nodiscard]] int BodyIdAt(std::size_t index) const noexcept;
  /// qpos address of candidate `index`'s freejoint, or -1. Test/observer use.
  [[nodiscard]] int QposAdrAt(std::size_t index) const noexcept;

 private:
  static constexpr std::size_t kNoActive = static_cast<std::size_t>(-1);

  // Relaxed is sufficient on both sides: active_ is a self-contained index into
  // a vector that is already immutable when any reader can run, so there is no
  // second write for an acquire/release pair to order against. The atomic is
  // here to make the concurrent read well-defined, not to publish other state.
  [[nodiscard]] std::size_t ActiveIndex() const noexcept {
    return active_.load(std::memory_order_relaxed);
  }

  void SetActiveIndex(std::size_t index) noexcept {
    active_.store(index, std::memory_order_relaxed);
  }

  // One attached candidate. contype/conaffinity are saved per geom rather than
  // assumed to be 1: object_sim marks visual geoms contype=0 already, so a
  // blanket restore to 1 would put the visual shell into collision.
  struct Slot {
    std::string name;       // directory name, e.g. "duck"
    std::string body_name;  // prefixed body name in the compiled model
    int body_id{-1};
    int qpos_adr{-1};
    int dof_adr{-1};
    int geom_begin{0};
    int geom_count{0};
    std::vector<int> contype;
    std::vector<int> conaffinity;
  };

  void Park(mjModel* model, mjData* data, Slot& slot) const noexcept;
  void Activate(mjModel* model, mjData* data, Slot& slot, const SampledPose& pose) const noexcept;

  ObjectPoolConfig cfg_{};
  std::vector<std::string> names_;  // candidate directory names, sorted
  std::vector<Slot> slots_;         // parallel to names_ once Resolve() ran
  std::size_t fixed_index_{0};
  // Written only by the refresh path (SimLoop-owned), read every frame by the
  // viewer thread through ActiveName()/ActiveBodyId(). Atomic so that read is
  // defined rather than a data race; see the observer note above.
  std::atomic<std::size_t> active_{kNoActive};
  SampledPose last_pose_{};
  std::mt19937_64 rng_{};
  bool resolved_{false};
};

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_OBJECT_POOL_HPP_
