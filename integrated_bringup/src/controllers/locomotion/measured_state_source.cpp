// Measured state source: device layout, the `estimator` section and the truth
// source. See measured_state_source.hpp.
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"

#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_controllers/params/yaml_read.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace integrated_bringup::locomotion {

namespace {

using rtc::params::FindMissingKey;
using rtc::params::FindUnknownKey;
using rtc::params::Spelling;

[[noreturn]] void Refuse(const std::string& where, const std::string& what) {
  throw std::invalid_argument(where + ": " + what);
}

double ReadAgeMs(const YAML::Node& map, const char* key, const std::string& where) {
  const YAML::Node v = map[key];
  const std::string at = where + "." + key;
  double ms = std::numeric_limits<double>::quiet_NaN();
  try {
    if (v.IsScalar()) {
      ms = v.as<double>();
    }
  } catch (const YAML::Exception&) {
  }
  // NaN fails both comparisons, so a non-number lands here too.
  if (!(ms > 0.0 && ms <= 1000.0)) {
    Refuse(at, "expected a number in (0, 1000] ms, got " + Spelling(v));
  }
  return ms;
}

void MapRoster(DeviceChannelMap& map, int device, const std::vector<std::string>& roster,
               const std::map<std::string, int>& joint_index, std::vector<bool>& mapped,
               const std::string& where) {
  if (roster.empty()) {
    Refuse(where, "the roster is empty");
  }
  if (roster.size() > kMaxStateJoints) {
    Refuse(where, "the roster has " + std::to_string(roster.size()) + " joints, more than " +
                      std::to_string(kMaxStateJoints));
  }
  map.device = device;
  map.num_channels = static_cast<int>(roster.size());
  for (std::size_t c = 0; c < roster.size(); ++c) {
    const auto it = joint_index.find(roster[c]);
    if (it == joint_index.end()) {
      Refuse(where, "joint '" + roster[c] + "' is not in the model");
    }
    const auto a = static_cast<std::size_t>(it->second);
    if (mapped[a]) {
      Refuse(where, "joint '" + roster[c] + "' is on two rosters (or twice on one)");
    }
    mapped[a] = true;
    map.joint[c] = static_cast<int16_t>(it->second);
  }
}

// A lane's two readable terms for both joint-space lanes this source reads.
bool JointLanesReadable(const rtc::DeviceState& dev, int n) noexcept {
  return rtc::IsLaneReadable(dev, rtc::StateLane::kPosition, n) &&
         rtc::IsLaneReadable(dev, rtc::StateLane::kVelocity, n);
}

bool Fresh(int64_t age_ns, int64_t max_age_ns) noexcept {
  return age_ns >= 0 && age_ns <= max_age_ns;
}

}  // namespace

const char* ToString(StateSourceStatus status) noexcept {
  switch (status) {
    case StateSourceStatus::kOk:
      return "ok";
    case StateSourceStatus::kBodyUnreadable:
      return "body_unreadable";
    case StateSourceStatus::kBodyStale:
      return "body_stale";
    case StateSourceStatus::kBaseMissing:
      return "base_missing";
    case StateSourceStatus::kBaseStale:
      return "base_stale";
    case StateSourceStatus::kBaseInvalid:
      return "base_invalid";
    case StateSourceStatus::kNonFinite:
      return "non_finite";
  }
  return "unknown";
}

DeviceIndices BuildDeviceIndices(const std::vector<std::string>& joint_order, int body_device,
                                 const std::vector<std::string>& body_roster,
                                 const std::vector<HandRoster>& hands, const std::string& context) {
  if (joint_order.empty() || joint_order.size() > kMaxStateJoints) {
    Refuse(context, "the model has " + std::to_string(joint_order.size()) +
                        " actuated joints; a state holds 1 to " + std::to_string(kMaxStateJoints));
  }
  if (hands.size() > kMaxStateHands) {
    Refuse(context, std::to_string(hands.size()) + " hand groups, more than " +
                        std::to_string(kMaxStateHands));
  }
  std::map<std::string, int> joint_index;
  for (std::size_t a = 0; a < joint_order.size(); ++a) {
    if (!joint_index.emplace(joint_order[a], static_cast<int>(a)).second) {
      Refuse(context, "the model names joint '" + joint_order[a] + "' twice");
    }
  }

  std::vector<bool> device_used(static_cast<std::size_t>(rtc::ControllerState::kMaxDevices), false);
  const auto claim_device = [&](int device, const std::string& where) {
    if (device < 0 || device >= rtc::ControllerState::kMaxDevices) {
      Refuse(where, "device slot " + std::to_string(device) + " is out of range");
    }
    if (device_used[static_cast<std::size_t>(device)]) {
      Refuse(where, "device slot " + std::to_string(device) + " is used twice");
    }
    device_used[static_cast<std::size_t>(device)] = true;
  };

  DeviceIndices out;
  out.num_joints = static_cast<int>(joint_order.size());
  std::vector<bool> mapped(joint_order.size(), false);
  claim_device(body_device, context + " (body group)");
  MapRoster(out.body, body_device, body_roster, joint_index, mapped, context + " (body group)");
  for (std::size_t h = 0; h < hands.size(); ++h) {
    const std::string where = context + " (hand group " + std::to_string(h) + ")";
    claim_device(hands[h].device, where);
    MapRoster(out.hands[h], hands[h].device, hands[h].joint_names, joint_index, mapped, where);
  }
  out.num_hands = static_cast<int>(hands.size());
  for (std::size_t a = 0; a < joint_order.size(); ++a) {
    if (!mapped[a]) {
      Refuse(context, "joint '" + joint_order[a] + "' is on no device roster");
    }
  }
  return out;
}

EstimatorConfig ParseEstimatorConfig(const YAML::Node& estimator, const std::string& context) {
  const std::initializer_list<const char*> keys = {"source", "body_max_age_ms", "hand_max_age_ms"};
  if (!estimator || !estimator.IsMap()) {
    Refuse(context, "expected a map, got " + Spelling(estimator));
  }
  if (const auto unknown = FindUnknownKey(estimator, keys)) {
    Refuse(context, "unknown key '" + *unknown + "'");
  }
  if (const auto missing = FindMissingKey(estimator, keys)) {
    Refuse(context, "required key '" + *missing + "' is missing");
  }
  EstimatorConfig cfg;
  const YAML::Node source = estimator["source"];
  if (!source.IsScalar() || source.Scalar() != "truth") {
    Refuse(context + ".source", "expected 'truth' (the only source), got " + Spelling(source));
  }
  cfg.source = source.Scalar();
  cfg.body_max_age_ms = ReadAgeMs(estimator, "body_max_age_ms", context);
  cfg.hand_max_age_ms = ReadAgeMs(estimator, "hand_max_age_ms", context);
  return cfg;
}

TruthStateSource::TruthStateSource(const Params& params) : params_(params) {
  if (params_.body_max_age_ns <= 0 || params_.hand_max_age_ns <= 0) {
    throw std::invalid_argument("TruthStateSource: the age limits must be positive");
  }
  if (params_.num_contacts < 0 || params_.num_contacts > kMaxStateContacts) {
    throw std::invalid_argument("TruthStateSource: num_contacts " +
                                std::to_string(params_.num_contacts) + " is outside [0, " +
                                std::to_string(kMaxStateContacts) + "]");
  }
}

void TruthStateSource::Reset() noexcept {
  ++state_epoch_;
  held_hand_q_.fill(0.0);
  hands_seen_ = 0;
}

bool TruthStateSource::Update(const rtc::ControllerState& state, const DeviceIndices& indices,
                              int64_t now_steady_ns, MeasuredRobotState& out) noexcept {
  const auto n = static_cast<std::size_t>(
      std::clamp(indices.num_joints, 0, static_cast<int>(kMaxStateJoints)));
  out.nq = 7 + static_cast<int>(n);
  out.nv = 6 + static_cast<int>(n);
  out.kind = StateSourceKind::kTruth;
  out.model_epoch = params_.model_epoch;
  out.state_epoch = state_epoch_;
  out.contact_mask = params_.num_contacts >= kMaxStateContacts
                         ? ~0U
                         : (1U << static_cast<uint32_t>(params_.num_contacts)) - 1U;
  out.ee_wrench = {};
  out.fresh_mask = 0;
  out.valid_mask = 0;
  out.stamp_steady_ns = 0;
  out.sequence = 0;
  StateSourceStatus status = StateSourceStatus::kOk;
  const auto fail = [&status](StateSourceStatus s) {
    if (status == StateSourceStatus::kOk) {
      status = s;
    }
  };

  // ── body: joints + floating base ──────────────────────────────────────────
  const int body = indices.body.device;
  if (body < 0 || body >= state.num_devices || body >= rtc::ControllerState::kMaxDevices) {
    fail(StateSourceStatus::kBodyUnreadable);
  } else {
    const auto& dev = state.devices[static_cast<std::size_t>(body)];
    const int nc = indices.body.num_channels;
    if (!JointLanesReadable(dev, nc)) {
      fail(StateSourceStatus::kBodyUnreadable);
    } else {
      for (int c = 0; c < nc; ++c) {
        const auto ci = static_cast<std::size_t>(c);
        const auto a = static_cast<std::size_t>(indices.body.joint[ci]);
        out.q[7 + a] = dev.positions[ci];
        out.v[6 + a] = dev.velocities[ci];
      }
      out.valid_mask |= kBodyJointsBit;
      if (Fresh(rtc::JointAgeNs(dev, now_steady_ns), params_.body_max_age_ns)) {
        out.fresh_mask |= kBodyJointsBit;
      } else {
        fail(StateSourceStatus::kBodyStale);
      }
    }
    out.sequence = dev.joint_sequence;

    if (!dev.base_valid || rtc::BaseAgeNs(dev, now_steady_ns) < 0) {
      fail(StateSourceStatus::kBaseMissing);
    } else {
      if (rtc::IsBaseFresh(dev, now_steady_ns, params_.body_max_age_ns)) {
        out.fresh_mask |= kBaseBit;
      } else {
        fail(StateSourceStatus::kBaseStale);
      }
      const auto& p = dev.base_pose;
      const double norm = std::sqrt(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6]);
      // A truth quaternion is unit to rounding; one this far off is a broken
      // producer, not drift to renormalise away. NaN fails the test as well.
      if (!(norm >= 0.5 && norm <= 1.5)) {
        fail(StateSourceStatus::kBaseInvalid);
      } else {
        for (std::size_t i = 0; i < 3; ++i) {
          out.q[i] = p[i];
        }
        for (std::size_t i = 3; i < 7; ++i) {
          out.q[i] = p[i] / norm;
        }
        for (std::size_t i = 0; i < 6; ++i) {
          out.v[i] = dev.base_twist[i];
        }
        out.valid_mask |= kBaseBit;
      }
    }
    out.stamp_steady_ns = std::min(dev.joint_recv_steady_ns, dev.base_recv_steady_ns);
  }

  // ── hands: fresh → copy and hold; stale → the held joints, at rest ────────
  const auto nh =
      static_cast<std::size_t>(std::clamp(indices.num_hands, 0, static_cast<int>(kMaxStateHands)));
  for (std::size_t h = 0; h < nh; ++h) {
    const DeviceChannelMap& map = indices.hands[h];
    const bool present = map.device >= 0 && map.device < state.num_devices &&
                         map.device < rtc::ControllerState::kMaxDevices;
    const rtc::DeviceState* dev =
        present ? &state.devices[static_cast<std::size_t>(map.device)] : nullptr;
    bool fresh = dev != nullptr && JointLanesReadable(*dev, map.num_channels) &&
                 Fresh(rtc::JointAgeNs(*dev, now_steady_ns), params_.hand_max_age_ns);
    // A hand sample with a NaN in it is a stale hand, not a failed state.
    for (int c = 0; fresh && c < map.num_channels; ++c) {
      const auto ci = static_cast<std::size_t>(c);
      fresh = std::isfinite(dev->positions[ci]) && std::isfinite(dev->velocities[ci]);
    }
    for (int c = 0; c < map.num_channels; ++c) {
      const auto ci = static_cast<std::size_t>(c);
      const auto a = static_cast<std::size_t>(map.joint[ci]);
      if (fresh) {
        held_hand_q_[a] = dev->positions[ci];
      }
      out.q[7 + a] = held_hand_q_[a];
      out.v[6 + a] = fresh ? dev->velocities[ci] : 0.0;
    }
    if (fresh) {
      hands_seen_ |= HandBit(h);
      out.fresh_mask |= HandBit(h);
    }
    out.valid_mask |= hands_seen_ & HandBit(h);
  }

  if (status == StateSourceStatus::kOk) {
    for (std::size_t i = 0; i < 7 + n; ++i) {
      if (!std::isfinite(out.q[i])) {
        fail(StateSourceStatus::kNonFinite);
      }
    }
    for (std::size_t i = 0; i < 6 + n; ++i) {
      if (!std::isfinite(out.v[i])) {
        fail(StateSourceStatus::kNonFinite);
      }
    }
  }
  out.status = status;
  out.valid = status == StateSourceStatus::kOk;
  return out.valid;
}

}  // namespace integrated_bringup::locomotion
