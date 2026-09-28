// ── sim_config_fixture.hpp ───────────────────────────────────────────────────
// Building blocks for a test's MuJoCoSimulator::Config: headless, and the
// robot / contact-wrench joint groups the MJCF fixtures are written for.
// Needs no fixture-path macro, so any test target can include it
// (test_fixture.hpp adds the minimal-scene configs on top).
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_SIM_CONFIG_FIXTURE_HPP_
#define RTC_MUJOCO_SIM_SIM_CONFIG_FIXTURE_HPP_

#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <string>
#include <vector>

namespace rtc::test {

/// A headless (no viewer) config for an MJCF fixture. Everything else keeps
/// its default — unlimited RTF, one substep, 60 Hz viewer refresh, no YAML
/// servo gains.
inline MuJoCoSimulator::Config HeadlessConfig(const std::string& model_path,
                                              double sync_timeout_ms) {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = model_path;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = sync_timeout_ms;
  return cfg;
}

/// A robot group commanding and reporting `joints`, on /<name>/cmd and
/// /<name>/state.
inline JointGroupConfig RobotGroup(const std::string& name,
                                   const std::vector<std::string>& joints) {
  JointGroupConfig g;
  g.name = name;
  g.command_joint_names = joints;
  g.state_joint_names = joints;
  g.command_topic = "/" + name + "/cmd";
  g.state_topic = "/" + name + "/state";
  g.is_robot = true;
  return g;
}

/// RobotGroup plus every sensor ("auto", which includes mjSENS_CONTACT) and
/// the contact-wrench lane: sensors named *_contact, referenced at the site
/// named *<site_suffix>, published under `topic_prefix`.
inline JointGroupConfig ContactWrenchGroup(const std::string& name,
                                           const std::vector<std::string>& joints,
                                           const std::string& topic_prefix,
                                           const std::string& site_suffix = "_ft_site") {
  JointGroupConfig g = RobotGroup(name, joints);
  g.sensor_names = {"auto"};
  g.contact_wrench.enabled = true;
  g.contact_wrench.topic_prefix = topic_prefix;
  g.contact_wrench.sensor_name_suffixes = {"_contact"};
  g.contact_wrench.reference_site_suffixes = {site_suffix};
  return g;
}

}  // namespace rtc::test

#endif  // RTC_MUJOCO_SIM_SIM_CONFIG_FIXTURE_HPP_
