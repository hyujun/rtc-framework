#ifndef RTC_MPC_MODEL_ROBOT_MODEL_HANDLER_HPP_
#define RTC_MPC_MODEL_ROBOT_MODEL_HANDLER_HPP_

/// @file robot_model_handler.hpp
/// @brief Robot-agnostic wrapper over `pinocchio::Model` + YAML metadata.
///
/// `RobotModelHandler` consumes a pre-built `pinocchio::Model` (typically
/// produced by `rtc_urdf_bridge::PinocchioModelBuilder`) and a YAML config
/// describing MPC-relevant frames (contact frames, end-effector). It
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

#include "rtc_mpc/types/contact_plan_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
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
  kMissingEndEffectorFrame, ///< YAML `end_effector_frame` not in model
  kMissingContactFrame,     ///< a YAML `contact_frames[i].name` not in model
  kInvalidContactDim,       ///< contact frame dim ∉ {3, 6}
  kInvalidYamlSchema,       ///< structure missing required keys
};

/// YAML schema expected by @ref RobotModelHandler::Init:
///
/// ```yaml
/// end_effector_frame: "<ee_frame_name>"  # required — URDF frame name
/// contact_frames:                         # optional (empty list → no
/// contacts)
///   - name: "<contact_frame_0>"
///     dim: 3                              # 3 (point) or 6 (wrench), default 3
///   - name: "<contact_frame_1>"
///     dim: 3
/// ```
///
/// All frame names must exist in the `pinocchio::Model` passed to `Init`.
class RobotModelHandler {
public:
  RobotModelHandler() = default;

  RobotModelHandler(const RobotModelHandler &) = delete;
  RobotModelHandler &operator=(const RobotModelHandler &) = delete;
  RobotModelHandler(RobotModelHandler &&) = delete;
  RobotModelHandler &operator=(RobotModelHandler &&) = delete;

  /// @brief Resolve YAML config against the provided Pinocchio model.
  ///
  /// @param model  Pinocchio model (ownership stays with caller — must
  ///               outlive this handler).
  /// @param cfg    YAML node matching the schema documented above.
  /// @return `kNoError` on success, otherwise the first validation failure.
  ///         On failure the handler remains in the uninitialised state.
  [[nodiscard]] RobotModelInitError Init(const pinocchio::Model &model,
                                         const YAML::Node &cfg) noexcept;

  /// @return true once @ref Init has succeeded.
  [[nodiscard]] bool Initialised() const noexcept { return model_ != nullptr; }

  // ── Dimension accessors (all require Initialised()==true) ────────────────
  [[nodiscard]] int nq() const noexcept;
  [[nodiscard]] int nv() const noexcept;
  /// @brief Control dim. For fixed-base manipulators `nu == nv`. When/if
  /// rtc_mpc gains floating-base support this will diverge from `nv`; keep
  /// callers using `nu()` rather than `nv()` for control sizing.
  [[nodiscard]] int nu() const noexcept;

  [[nodiscard]] int n_contacts() const noexcept {
    return static_cast<int>(contact_frames_.size());
  }
  [[nodiscard]] const std::vector<ContactFrameInfo> &
  contact_frames() const noexcept {
    return contact_frames_;
  }
  [[nodiscard]] int end_effector_frame_id() const noexcept {
    return ee_frame_id_;
  }
  [[nodiscard]] const pinocchio::Model &model() const noexcept {
    return *model_;
  }

  /// @return Pinocchio frame id for @p name, or `std::nullopt` if absent.
  /// Safe to call on an uninitialised handler (returns nullopt).
  [[nodiscard]] std::optional<int>
  FrameId(std::string_view name) const noexcept;

private:
  const pinocchio::Model *model_{nullptr};
  int ee_frame_id_{-1};
  std::vector<ContactFrameInfo> contact_frames_{};
};

} // namespace rtc::mpc

#endif // RTC_MPC_MODEL_ROBOT_MODEL_HANDLER_HPP_
