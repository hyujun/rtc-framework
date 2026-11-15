#pragma once

#include "rtc_tsid/types/wbc_margins.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Classification of a whole-body QP's ticks into a fail-safe state.
//
// A tick is GOOD when its torque is valid (CommandOutput::tau_valid) and its
// margins (WbcMargins, of that same tick) pass every floor of the config. A
// floor that is off still refuses a NaN margin, so with the defaults a good
// tick is a valid tick whose margins are not NaN. Any other tick is BAD — a
// failed or non-finite solve, a reference the owner refused (SkipTick), a
// margin below its floor. Margins are read on valid ticks only.
//
//   state     | a good tick                      | a bad tick
//   ----------|----------------------------------|---------------------------------
//   Nominal   | Nominal                          | Degraded, or Fault when the run
//             |                                  | of bad ticks > max_invalid_ticks
//   Degraded  | Nominal after recover_valid_ticks| Degraded, or Fault as above
//             | good ticks in a row              |
//   Fault     | Nominal after recover_valid_ticks| Fault (the good-tick run restarts)
//             | good ticks in a row              |
//
// max_invalid_ticks < 0 (the default) never enters Fault. Reset() returns to
// Nominal with both runs cleared. It only classifies: the torque is the
// owner's to keep or replace. Single-threaded (the owner's tick); RT-safe.
// ────────────────────────────────────────────────

enum class SolveHealthState : std::uint8_t { kNominal, kDegraded, kFault };

struct SolveHealthConfig {
  // The longest run of bad ticks that stays Degraded; < 0: off (never Fault).
  int max_invalid_ticks{-1};
  // Good ticks in a row that return Degraded / Fault to Nominal (≥ 1).
  int recover_valid_ticks{1};
  // Margin floors (WbcMargins units); −inf = off. max_eom_residual: +inf = off.
  double min_friction_margin{-std::numeric_limits<double>::infinity()};
  double min_cop_margin{-std::numeric_limits<double>::infinity()};
  double min_yaw_margin{-std::numeric_limits<double>::infinity()};
  double min_torque_margin{-std::numeric_limits<double>::infinity()};
  double min_joint_accel_margin{-std::numeric_limits<double>::infinity()};
  double max_eom_residual{std::numeric_limits<double>::infinity()};
};

// The keys of a YAML map over the defaults — max_invalid_ticks (≥ 0),
// recover_valid_ticks (≥ 1), min_friction_margin, min_cop_margin,
// min_yaw_margin, min_torque_margin, min_joint_accel_margin (finite) and
// max_eom_residual (finite, > 0); a missing key keeps the default. A node that
// is not a map, an unknown key, a value that does not convert or is out of
// range throws std::invalid_argument (init time).
[[nodiscard]] SolveHealthConfig ParseSolveHealthConfig(const YAML::Node& node);

class SolveHealth {
 public:
  void Configure(const SolveHealthConfig& config) noexcept {
    config_ = config;
    Reset();
  }

  // One tick: `tau_valid` and, when it is true, that tick's margins.
  SolveHealthState Update(bool tau_valid, const WbcMargins& margins) noexcept;

  void Reset() noexcept {
    state_ = SolveHealthState::kNominal;
    bad_run_ = 0;
    good_run_ = 0;
  }

  [[nodiscard]] SolveHealthState State() const noexcept { return state_; }

  [[nodiscard]] int BadRun() const noexcept { return bad_run_; }

  [[nodiscard]] int GoodRun() const noexcept { return good_run_; }

  [[nodiscard]] const SolveHealthConfig& Config() const noexcept { return config_; }

  // Whether `margins` pass every floor of `config` (a NaN margin never does).
  [[nodiscard]] static bool MarginsPass(const SolveHealthConfig& config,
                                        const WbcMargins& margins) noexcept;

 private:
  SolveHealthConfig config_;
  SolveHealthState state_{SolveHealthState::kNominal};
  int bad_run_{0};
  int good_run_{0};
};

}  // namespace rtc::tsid
