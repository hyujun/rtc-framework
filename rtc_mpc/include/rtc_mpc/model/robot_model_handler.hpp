#ifndef RTC_MPC_MODEL_ROBOT_MODEL_HANDLER_HPP_
#define RTC_MPC_MODEL_ROBOT_MODEL_HANDLER_HPP_

/// @file robot_model_handler.hpp
/// @brief Robot-agnostic wrapper over `pinocchio::Model` + YAML metadata.
///
/// `RobotModelHandler` consumes a pre-built `pinocchio::Model` (typically
/// produced by `rtc_urdf_bridge::PinocchioModelBuilder`) and a YAML config
/// describing MPC-relevant frames (contact frames, end-effectors). It
/// resolves frame-name strings to Pinocchio frame ids **once**, at Init
/// time, so the solve path can work purely with integer indices.
///
/// Robot-agnostic contract:
/// - No robot names, joint counts, or HW identifiers are hardcoded here.
///   All topology flows from `pinocchio::Model` (for dims) + YAML (for
///   which frames to expose).
/// - Model ownership stays with the caller — we hold a `const` reference.
/// - `Init` never throws; validation failures return an error code so
///   callers on the solve-preparation path can degrade gracefully.

#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::mpc {

/// @brief Failure modes for @ref RobotModelHandler::Init.
enum class RobotModelInitError {
  kNoError = 0,
  kModelAlreadyInitialised,
  kMissingEndEffectorFrame,         ///< a YAML end-effector frame not in model
  kMissingContactFrame,             ///< a YAML `contact_frames[i].name` not in model
  kInvalidContactDim,               ///< contact frame dim ∉ {3, 6}
  kInvalidYamlSchema,               ///< structure missing required keys
  kMissingBaseFrame,                ///< YAML `base_frame` set but not in model
  kBaseFrameNotUniverseOnFloating,  ///< floating-base model with a `base_frame` other than universe
};

/// YAML schema expected by @ref RobotModelHandler::Init:
///
/// ```yaml
/// end_effector_frame: "<ee_frame_name>"  # one end-effector (URDF frame name), or
/// end_effector_frames:                   # several, in order — exactly one of the
///   - "<ee_frame_0>"                     # two keys is required; the list holds
///   - "<ee_frame_1>"                     # ≥ 1 distinct names
/// base_frame: "<base_frame_name>"        # required (F-4 strict). Pass
///                                        # "universe" to opt into the
///                                        # world-frame fast path.
/// contact_frames:                        # optional (empty list → no contacts)
///   - name: "<contact_frame_0>"
///     dim: 3                             # 3 (point) or 6 (wrench), default 3
///   - name: "<contact_frame_1>"
///     dim: 3
/// ```
///
/// All frame names must exist in the `pinocchio::Model` passed to `Init`. Both
/// end-effector keys, neither, an empty list, a repeated name or a non-string
/// entry is @c kInvalidYamlSchema; an unknown end-effector name is
/// @c kMissingEndEffectorFrame. The scalar key is the one-element list.
/// Missing `base_frame` is reported as @c kInvalidYamlSchema (no implicit
/// universe fallback — that was removed in F-4). On a floating-base model
/// `base_frame` must be `universe`: the base pose is part of the state, so no
/// body frame is q-independent and @ref base_oMf would be meaningless
/// (@c kBaseFrameNotUniverseOnFloating). Task targets relative to the base are
/// the consumer's transform.
class RobotModelHandler {
 public:
  RobotModelHandler() = default;

  RobotModelHandler(const RobotModelHandler&) = delete;
  RobotModelHandler& operator=(const RobotModelHandler&) = delete;
  RobotModelHandler(RobotModelHandler&&) = delete;
  RobotModelHandler& operator=(RobotModelHandler&&) = delete;

  /// @brief Resolve YAML config against the provided Pinocchio model.
  ///
  /// @param model  Pinocchio model (ownership stays with caller — must
  ///               outlive this handler).
  /// @param cfg    YAML node matching the schema documented above.
  /// @return `kNoError` on success, otherwise the first validation failure.
  ///         On failure the handler remains in the uninitialised state.
  [[nodiscard]] RobotModelInitError Init(const pinocchio::Model& model,
                                         const YAML::Node& cfg) noexcept;

  /// @return true once @ref Init has succeeded.
  [[nodiscard]] bool Initialised() const noexcept { return model_ != nullptr; }

  // ── Dimension accessors (all require Initialised()==true) ────────────────
  [[nodiscard]] int nq() const noexcept;
  [[nodiscard]] int nv() const noexcept;

  /// @brief Control dim. Fixed base: identity torque input, `nu == nv`.
  /// Floating base: the kinodynamic input `[W_1 … W_nc; q̈_j]`, so
  /// `nu == Σ contact_frames[i].dim + nv − n_base_v()`. Size controls with
  /// `nu()`, never `nv()`. The arithmetic lives in @ref WbLayout.
  [[nodiscard]] int nu() const noexcept { return layout_.nu; }

  /// @return true when the model's root is a free-flyer.
  [[nodiscard]] bool floating_base() const noexcept { return layout_.floating_base(); }

  /// @return tangent entries the base occupies at the front of `v`: the
  ///         free-flyer's six, or 0 on a fixed base.
  [[nodiscard]] int n_base_v() const noexcept { return layout_.n_base_v; }

  /// @return the whole-body layout of this model and its contact frames,
  ///         computed once at @ref Init (empty before).
  [[nodiscard]] const WbLayout& layout() const noexcept { return layout_; }

  [[nodiscard]] int n_contacts() const noexcept { return static_cast<int>(contact_frames_.size()); }

  [[nodiscard]] const std::vector<ContactFrameInfo>& contact_frames() const noexcept {
    return contact_frames_;
  }

  /// @return frame id of the first end-effector (the only one under the scalar
  ///         key), or −1 before @ref Init. Consumers that track one end-effector
  ///         (frame placement cost, grasp phase manager) read this.
  [[nodiscard]] int end_effector_frame_id() const noexcept {
    return ee_frame_ids_.empty() ? -1 : ee_frame_ids_.front();
  }

  /// @return frame ids of every end-effector in YAML order (≥ 1 after
  ///         @ref Init, empty before); the front is @ref end_effector_frame_id.
  [[nodiscard]] const std::vector<int>& end_effector_frame_ids() const noexcept {
    return ee_frame_ids_;
  }

  /// @return Base frame id used as the reference for SE3 control. Always
  /// the resolved frame id of YAML `base_frame` — F-4 strict mode rejects a
  /// missing key with @c kInvalidYamlSchema, so no implicit universe
  /// fallback path can leak through here.
  [[nodiscard]] int base_frame_id() const noexcept { return base_frame_id_; }

  /// @return true when the resolved `base_frame` is the Pinocchio universe
  /// (frame_id 0). Callers may skip the `oMb⁻¹·oMf` transform on the fast
  /// path.
  [[nodiscard]] bool base_frame_is_universe() const noexcept { return base_is_universe_; }

  /// @return World pose of the base frame, computed at @ref Init with the
  /// model's neutral configuration. Valid because base_frame must be a
  /// fixed (q-independent) frame — placement does not change with q; on a
  /// floating-base model only universe qualifies, which @ref Init enforces.
  /// Identity when @ref base_frame_is_universe is true.
  [[nodiscard]] const pinocchio::SE3& base_oMf() const noexcept { return base_oMf_; }

  [[nodiscard]] const pinocchio::Model& model() const noexcept { return *model_; }

  /// @return Pinocchio frame id for @p name, or `std::nullopt` if absent.
  /// Safe to call on an uninitialised handler (returns nullopt).
  [[nodiscard]] std::optional<int> FrameId(std::string_view name) const noexcept;

 private:
  const pinocchio::Model* model_{nullptr};
  std::vector<int> ee_frame_ids_{};
  WbLayout layout_{};
  // Base frame for SE3 control. Always 0..nframes-1 after Init (F-4 strict).
  // base_is_universe_=true ⇔ base_frame_id_=0 (Pinocchio universe).
  int base_frame_id_{0};
  bool base_is_universe_{true};
  // World pose of base frame at neutral q. base가 fixed라는 invariant 하에
  // q와 무관해 Init 시 1회 계산. universe면 Identity.
  pinocchio::SE3 base_oMf_{pinocchio::SE3::Identity()};
  std::vector<ContactFrameInfo> contact_frames_{};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_MODEL_ROBOT_MODEL_HANDLER_HPP_
