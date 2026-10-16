// ── virtual_tcp_* as runtime controller parameters (#847) ───────────────────
//
// The three demo_shared.yaml keys that place the virtual TCP, exposed as ROS 2
// parameters on the task, compliance and joint controllers so an operator (or
// demo_controller_gui) can read them and change them without a restart.
//
// Split in two on purpose:
//   - ApplyVirtualTcpSet() / ApplyVirtualTcpParameter() are node-free and pure:
//     they validate one rclcpp::Parameter and write it into a VirtualTcpConfig.
//     Each controller's set-parameters callback makes the one ApplyVirtualTcpSet
//     call, so the inactive gate lives here and not in three callbacks.
//     test_virtual_tcp_params pins both.
//   - DeclareVirtualTcpParameters() is the thin node wiring that on_configure
//     runs through DeclareGainParameters().
//
// Contract:
//   - Inactive only. ApplyVirtualTcpSet refuses every virtual_tcp_* entry with
//     kVtcpRefusedActiveReason while the controller is active. The hold seed is
//     taken on the first RT tick after activation, so a value stored while
//     inactive is the frame the arm is seeded in and nothing jumps. Changing the
//     control point while active is #472's epoch problem and stays refused.
//   - Single writer. gains_lock_ is a single-writer SeqLock. Its writers (the
//     set-parameters callback, the lifecycle callbacks, LoadConfig) all run on the
//     controller manager's single-threaded non-RT executor, and that serialisation
//     is the only thing that keeps the `active_` check and the Store from
//     interleaving with on_activate. Moving controllers onto a multi-threaded
//     executor would have to add a lock here and around every other gain.
//   - Persistence. A value set at runtime lives as long as the node: on_cleanup
//     leaves the parameters declared, so the next on_configure reads the stored
//     value back instead of the YAML seed. demo_shared.yaml seeds them again only
//     when the process restarts. This is the same lifetime every other gain has.
//
// kVirtualTcpModeNames below is the one C++ spelling of the modes: the YAML
// parser and its configure-time log line (demo_shared_config.cpp) use it too.
// The parser folds an unknown name into kDisabled; the runtime path refuses it.
// The GUI's copy (demo_gui/vtcp.py) and the refusal text are checked against
// the table by tests.
#pragma once

#include "integrated_bringup/support/virtual_tcp.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <array>
#include <optional>
#include <string_view>
#include <utility>

namespace integrated_bringup {

/// Parameter name for VirtualTcpConfig::mode. Type: string, one of
/// kVirtualTcpModeNames.
inline constexpr const char* kVirtualTcpModeParam = "virtual_tcp_mode";
/// Parameter name for VirtualTcpConfig::offset. Type: double array of length 3,
/// [x, y, z] in the TCP frame [m]. Read by the constant mode only.
inline constexpr const char* kVirtualTcpOffsetParam = "virtual_tcp_offset";
/// Parameter name for VirtualTcpConfig::orientation. Type: double array of
/// length 3, [roll, pitch, yaw] of the virtual TCP in the TCP frame [rad].
/// Read by every mode other than disabled.
inline constexpr const char* kVirtualTcpOrientationParam = "virtual_tcp_orientation";

/// Reason a set-parameters callback returns for a virtual_tcp_* entry while the
/// controller is active. Clients show it verbatim, so it carries no prefix.
inline constexpr const char* kVtcpRefusedActiveReason =
    "virtual_tcp_* can only be set while this controller is inactive; "
    "switch to another controller first";

/// The accepted runtime spellings of VirtualTcpMode, in enum order.
inline constexpr std::array<std::pair<VirtualTcpMode, std::string_view>, 4> kVirtualTcpModeNames{{
    {VirtualTcpMode::kDisabled, "disabled"},
    {VirtualTcpMode::kCentroid, "centroid"},
    {VirtualTcpMode::kWeighted, "weighted"},
    {VirtualTcpMode::kConstant, "constant"},
}};

/// Spelling of @p mode in kVirtualTcpModeNames.
[[nodiscard]] const char* VirtualTcpModeName(VirtualTcpMode mode) noexcept;

/// Inverse of VirtualTcpModeName. Unlike the YAML parser, an unknown name is
/// not folded into kDisabled.
/// @return std::nullopt when @p name is not in kVirtualTcpModeNames
[[nodiscard]] std::optional<VirtualTcpMode> ParseVirtualTcpMode(std::string_view name) noexcept;

/// True when @p name is one of the three virtual_tcp_* parameter names.
[[nodiscard]] bool IsVirtualTcpParameter(std::string_view name) noexcept;

/// Validate one virtual_tcp_* parameter and write it into @p vtcp.
///
/// Checks the type, the whitelist (mode), the length 3 and finiteness (offset,
/// orientation), and that a change to a mode other than disabled has a hand
/// model to compute against: without one the controller skips the virtual TCP
/// in every mode, so switching into one would report a frame that never takes
/// effect. Re-sending the current mode is accepted with or without a hand — a
/// client that sends all three keys at once must be able to change only the
/// offset on a hand-less controller whose YAML names a non-disabled mode.
///
/// @param p        Parameter named kVirtualTcp*Param; any other name is refused
/// @param has_hand Whether the controller built a hand model
/// @param vtcp     In: the current values. Written only on success
/// @return nullptr on success, otherwise a static reason string
[[nodiscard]] const char* ApplyVirtualTcpParameter(const rclcpp::Parameter& p, bool has_hand,
                                                   VirtualTcpConfig& vtcp) noexcept;

/// The set-parameters callback's whole virtual_tcp_* branch: the inactive gate,
/// then ApplyVirtualTcpParameter.
///
/// @param p        Parameter named kVirtualTcp*Param
/// @param active   The controller's activity flag at the time of the set
/// @param has_hand Whether the controller built a hand model
/// @param vtcp     In: the current values. Written only on success
/// @return nullptr on success; kVtcpRefusedActiveReason while @p active;
///         otherwise ApplyVirtualTcpParameter's reason
[[nodiscard]] const char* ApplyVirtualTcpSet(const rclcpp::Parameter& p, bool active, bool has_hand,
                                             VirtualTcpConfig& vtcp) noexcept;

/// Declare the three parameters on @p node, seeded from @p vtcp (the values
/// LoadConfig resolved from demo_shared.yaml).
///
/// A parameter that is already declared keeps its stored value (see the
/// persistence note above), and that value is written into @p vtcp. A stored
/// value can only have bypassed the set-parameters callback by being set while
/// the controller was cleaned up, so it is validated here with
/// ApplyVirtualTcpParameter. On refusal the YAML seed is kept, the parameter is
/// set back to it so the server and the gains agree, and the reason is logged
/// as an error. Never throws: it runs inside the noexcept DeclareGainParameters.
/// If rclcpp throws part way, each key keeps what was resolved for it so far
/// (the seed for any key not reached), which is what the server holds for every
/// key except one whose reset failed — that one is logged.
///
/// @param node     The controller's LifecycleNode
/// @param has_hand Whether the controller built a hand model
/// @param logger   Controller logger for the refusal line
/// @param vtcp     In: YAML seed. Out: the values the controller runs with
void DeclareVirtualTcpParameters(rclcpp_lifecycle::LifecycleNode& node, bool has_hand,
                                 const rclcpp::Logger& logger, VirtualTcpConfig& vtcp) noexcept;

}  // namespace integrated_bringup
