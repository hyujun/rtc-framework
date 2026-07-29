// ── Joint PD controller configuration (G2 schema layer) ─────────────────────
// The gains POD and the YAML schema that fills it, lifted out of
// JointPDController so that both outlive the adapter (#236 S7c-1, D-B/G2).
//
// Why this layer exists at all: the adapters in rtc_controllers/{direct,indirect}
// are scheduled for deletion, but their `Gains` structs are NOT adapter glue —
// they are the parameterisation of the laws in joint/joint_pd_law.hpp, and the
// test harnesses that pin those laws take a `Gains` by value. Leaving the POD
// nested inside the class would make every surviving harness a dependent of a
// deleted type. So the POD moves here and the adapter keeps a `using Gains =`
// alias: every existing spelling still resolves, and nothing about the runtime
// behaviour moves with it.
//
// The split against the two neighbouring layers:
//   - the LAW lives in joint/joint_pd_law.hpp and takes Eigen/span only (D-A);
//   - the GLUE (submodel selection, the SeqLock store, telemetry) stays in the
//     controller and dies with it (G3);
//   - this file is the part in between — a plain POD plus a yaml-cpp parser,
//     non-RT, that a future binding calls once at configure time.
#pragma once

#include <rtc_base/types/types.hpp>

#include <yaml-cpp/yaml.h>

#include <array>

namespace rtc::params {

/// Per-joint PD gains and the compensation switches (§ joint PD law).
/// Arrays are sized to the fixed capacity kMaxRobotDOF so a 7-DOF+ arm is
/// indexable without overrunning; the parser requires the YAML sequence length
/// to equal the model DOF exactly (no silent truncation/padding — issue #172),
/// and rejects a model DOF above that capacity outright, which is what keeps
/// "equal to nv" from meaning "past the end of the array" (#298 S7c-2).
struct JointPdParams {
  std::array<double, kMaxRobotDOF> kp{
      {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  std::array<double, kMaxRobotDOF> kd{
      {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
  bool enable_gravity_compensation{false};
  bool enable_coriolis_compensation{false};
  double trajectory_speed{1.0};
};

/// Parse the joint-PD YAML schema into @p out, updating @p command_type when the
/// `command_type` key is present.
///
/// @param cfg           controller config node; a null node parses nothing (but
///                      does not skip the @p nv capacity check below — that one
///                      is about the model, not the config).
/// @param nv            model DOF, in [0, kMaxRobotDOF]. `kp`/`kd` sequences
///                      must be exactly this long. Pass the EFFECTIVE DOF the
///                      binding will drive, i.e. after any submodel selection.
/// @param out           seeded with the caller's current values; only the keys
///                      present in @p cfg are overwritten.
/// @param command_type  in/out — carries the caller's current command type in so
///                      the cross-field rule below can be evaluated against the
///                      value that will actually be in force.
///
/// @throws std::runtime_error on an @p nv outside the fixed capacity, on a
///         mis-sized gain sequence, or on dynamics compensation requested on a
///         non-torque command. The capacity case is checked first and is the
///         only one an absent @p cfg still reaches — the gain arrays are indexed
///         by @p nv, so accepting a wider model would corrupt memory rather than
///         report a schema error (#298 S7c-2). The third is a cross-FIELD rule,
///         which is why it is validated here with
///         the schema rather than at the use site — g and C·v are N·m and cannot
///         be added to a position/velocity command, and failing fast at configure
///         time beats silently dropping the terms every tick (issue #172).
///
/// Non-RT: called from LoadConfig / on_configure, never from a tick.
void ParseJointPdParams(const YAML::Node& cfg, int nv, JointPdParams& out,
                        CommandType& command_type);

}  // namespace rtc::params
