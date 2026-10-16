#pragma once

/// @file task_target_frames.hpp
/// @brief The frames a task goal may name, and the conversion into the arm's
///        base frame (#779 target-frame contract).
///
/// A task goal (rtc_msgs/RobotTarget, goal_type "task") names its frame in
/// header.frame_id. A controller with this contract (WBC, Task, Compliance)
/// takes exactly two names:
///
///   - its arm's root_link — the frame the controller works in; the goal
///     passes through unchanged, bit for bit;
///   - "scene_world" — the scene's world, a model frame the CM adds from
///     urdf.extra_frames (sim.yaml / overlays; absent on a real arm until it
///     is calibrated). The goal is converted into root_link.
///
/// Anything else — an empty frame_id, an unknown name, scene_world on a
/// bring-up that does not define it — is refused with a reason, which the
/// controller returns from SetDeviceTaskTarget so the base class counts it.
///
/// Three states of the world (configure time):
///   - absent    — Configure() finds no scene_world frame: configure succeeds,
///                 world goals are refused at ingress;
///   - malformed — scene_world is not rigidly attached to root_link, or its
///                 placement is not finite: Configure() returns an error and
///                 the controller fails on_configure;
///   - present   — world goals are converted.
/// (A scene_world entry the CM cannot parse already refuses the whole
/// bring-up before any controller configures.)
///
/// Goal layout (x, y, z, roll, pitch, yaw), RPY = Rz(yaw)·Ry(pitch)·Rx(roll)
/// (rtc::math::se3::RpyToRotationZyx), the convention every binding uses. In
/// position-only mode only x, y, z are a pose: entries 3..5 are the binding's
/// own (Task / Compliance read them as null-space posture), so they pass
/// through untouched in every frame.
///
/// Threading: Configure() runs on the non-RT configure path, before the
/// controller creates its target subscription, and nothing writes the object
/// after it. ToRoot() is const, noexcept and allocation-free — it runs on the
/// target ingress callback.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <span>
#include <string>
#include <string_view>

namespace integrated_bringup {

class TaskTargetFrames {
 public:
  /// Model frame name of the scene's world (urdf.extra_frames key).
  static constexpr std::string_view kSceneWorld = "scene_world";

  /// Whether entries 3..5 of a goal are an orientation.
  enum class Layout { kPose6d, kPositionOnly };

  /// Reads root_link and, when the model has one, scene_world's placement in
  /// root_link. Returns an empty string on success (world present or absent —
  /// see WorldDefined()), otherwise the reason the world is malformed. A
  /// root_link the model lacks is the caller's to judge (it is configured
  /// from the device's urdf.root_link and resolved there first); here it
  /// leaves the object unconfigured and is reported.
  [[nodiscard]] std::string Configure(const pinocchio::Model& model, const std::string& root_link);

  /// Configure without a model: root_link only, no world (tests, and
  /// controllers that run without a model).
  void ConfigureWithoutWorld(const std::string& root_link);

  /// `task6` (6 values in `frame_id`) into root_link, written to `out`.
  /// Returns nullptr on success, otherwise a string literal naming why the
  /// goal is refused (`out` is then unspecified).
  [[nodiscard]] const char* ToRoot(std::string_view frame_id, std::span<const double> task6,
                                   Layout layout, std::array<double, 6>& out) const noexcept;

  [[nodiscard]] bool Configured() const noexcept { return !root_link_.empty(); }

  [[nodiscard]] bool WorldDefined() const noexcept { return world_defined_; }

  [[nodiscard]] const std::string& RootLink() const noexcept { return root_link_; }

  /// Placement of scene_world in root_link (identity when not defined).
  [[nodiscard]] const pinocchio::SE3& RootFromWorld() const noexcept { return root_T_world_; }

 private:
  std::string root_link_;
  bool world_defined_{false};
  pinocchio::SE3 root_T_world_{pinocchio::SE3::Identity()};
};

}  // namespace integrated_bringup
