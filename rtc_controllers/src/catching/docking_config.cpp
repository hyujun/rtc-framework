// ── Whether the selected docking functions can run on a profile ──────────────
#include "rtc_controllers/catching/docking_config.hpp"

#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"

#include <cmath>
#include <stdexcept>

namespace rtc::catching {

double EntranceCloseLead(double hand_close_lead, const HandDockingParams& hand,
                         const MpcDockingSegmentCoreParams& core) noexcept {
  const double c = -core.nu_ref.z();
  if (!std::isfinite(hand_close_lead) || !std::isfinite(hand.s_ent) || !std::isfinite(c) ||
      !(c > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();  // DockingConfigRefusal names it
  }
  return hand_close_lead - hand.s_ent / c;
}

std::string DockingConfigRefusal(const DockingConfig& config) {
  const bool nlp = config.nlp;
  const bool docking = config.docking;
  if (!nlp && !docking) {
    return {};  // no docking function runs: none of this is consumed
  }
  if (config.hand == nullptr || (nlp && (config.catching == nullptr || config.search == nullptr)) ||
      (docking && config.planner == nullptr)) {
    throw std::invalid_argument("DockingConfigRefusal: an input the selection reads is null");
  }
  const HandDockingParams& hand = *config.hand;
  // The hand's identified values (E1-F15): nobody may guess one.
  if (const char* unset = hand.FirstUnset(); unset != nullptr) {
    return std::string("'catching.") + unset +
           "' is unset or TBD (the hand's identified capture set)";
  }
  if (config.hand_close_unset) {
    return "'catching.robot.hand.T_close_e2e' or 'T_close_lead' is unset or TBD (the "
           "nominal closure instant is their difference)";
  }
  if (config.ball_mass_unset) {
    return "'catching.core.ball.mass' is unset or TBD (the impact rows)";
  }
  // A selected function's own map first: without it every value below is a
  // code default, and a refusal that names one of those (the lead against a
  // default closing speed) would hide that the map is missing — or nothing
  // would refuse, and the search would run on defaults no profile wrote. The
  // map has no decision key of its own to be unset (the catch box was the one,
  // L3 §4.9), so what is asked for is the map and the core design inside it:
  // that catches a profile without the search's fragment, and an overlay that
  // wrote a leaf or two into the gap. It cannot tell a complete map from one
  // that has a `core:` and little else.
  if (nlp) {
    // Const nodes throughout: a lookup must not create the key it asks about.
    const YAML::Node& catching = *config.catching;
    const YAML::Node planner =
        catching.IsDefined() && catching.IsMap() ? catching["planner"] : YAML::Node();
    const YAML::Node search =
        planner.IsDefined() && planner.IsMap() ? planner["search"] : YAML::Node();
    const YAML::Node map = search.IsDefined() && search.IsMap() ? search["nlp"] : YAML::Node();
    const bool has_map = map.IsDefined() && map.IsMap();
    const YAML::Node core = has_map ? map["core"] : YAML::Node();
    if (!has_map || !core.IsDefined() || !core.IsMap()) {
      return "'catching.planner.search.nlp.core' is not in this profile (the selected search's "
             "map — is its file included?)";
    }
  }
  if (docking && config.planner->n_pre_max < 1) {
    return "'catching.planner.segment.mpc_docking.approach.n_pre_max' is unset (the "
           "planner's pre-catch grid — its map is not in this profile?)";
  }
  if ((nlp && !(config.search_close_lead >= 0.0)) ||
      (docking && !(config.planner_close_lead >= 0.0))) {
    return "'catching.robot.hand.T_close_lead' is shorter than the ball's flight from the "
           "entrance plane to the catch point (robot.hand.docking.s_ent over the "
           "reference closing speed, core.catch.nu_ref): the close would be due after "
           "the entrance crossing";
  }
  // A first segment is planned from the RT state the wake loaded, and the
  // segment planners refuse one older than their bound: a search allowed to
  // run that long leaves no first segment to publish, ever.
  const double age_max_s =
      1e-9 * static_cast<double>(docking ? kMpcDockingMaxRtStateAgeNs : kMpcSegmentMaxRtStateAgeNs);
  if (nlp && !(config.search->budget_s < age_max_s)) {
    return "'catching.planner.search.nlp.budget.budget_s' is " +
           std::to_string(config.search->budget_s) +
           " s, not below the oldest RT state the segment planner plans a first segment "
           "from (" +
           std::to_string(age_max_s) +
           " s): every plan the search found would be withheld as stale";
  }
  if (nlp && docking) {
    // The planner republishes the search's solution after re-evaluating it on
    // its own core: the two grids have to be one.
    if (const char* differ = DockingGridMismatch(*config.search, *config.planner);
        differ != nullptr) {
      return differ;
    }
    if (config.search->continuous_tc) {
      return "'catching.planner.search.nlp.continuous_tc' is true: a solution whose catch "
             "interval has its own length is off the mpc_docking planner's grid, so no "
             "plan of it could be published";
    }
  }
  // The reference closing speed lives in a function's `core:` map, the band
  // it has to lie in under the hand's: the two are checked here, where both
  // are known.
  const auto speed_outside = [&hand](const MpcDockingSegmentCoreParams& core) {
    const double c = -core.nu_ref.z();
    return !(c >= hand.c_min && c <= hand.c_cap_max);
  };
  const auto speed_refusal = [&hand](const char* map, const MpcDockingSegmentCoreParams& core) {
    return std::string("'catching.planner.") + map + ".core.catch.nu_ref' closes at " +
           std::to_string(-core.nu_ref.z()) +
           " m/s, outside 'catching.robot.hand.docking.speed' [c_min, c_cap_max] = [" +
           std::to_string(hand.c_min) + ", " + std::to_string(hand.c_cap_max) + "]";
  };
  if (nlp && speed_outside(config.search->core)) {
    return speed_refusal("search.nlp", config.search->core);
  }
  if (docking && speed_outside(config.planner->core)) {
    return speed_refusal("segment.mpc_docking", config.planner->core);
  }
  return {};
}

}  // namespace rtc::catching
