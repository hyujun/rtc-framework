// See truth_log.hpp and logging/locomotion_truth_log_pod.hpp.
#include "integrated_bringup/controllers/locomotion/truth_log.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace integrated_bringup::locomotion {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double AgeUs(std::int64_t age_ns) noexcept {
  return age_ns >= 0 ? static_cast<double>(age_ns) * 1e-3 : -1.0;
}

}  // namespace

void FillLocomotionTruthLog(const rtc::DeviceState& body, const TruthLogInputs& in,
                            LocomotionTruthLogPod& pod) noexcept {
  pod.truth_mask = 0;

  // ── base lane ────────────────────────────────────────────────────────────
  pod.base_seq = body.base_sequence;
  pod.base_age_us = AgeUs(rtc::BaseAgeNs(body, in.now_steady_ns));
  const bool base = rtc::IsBaseFresh(body, in.now_steady_ns, in.max_age_ns);
  for (std::size_t i = 0; i < 3; ++i) {
    pod.base_p[i] = base ? body.base_pose[i] : kNaN;
    pod.base_v[i] = base ? body.base_twist[i] : kNaN;
    pod.base_w[i] = base ? body.base_twist[3 + i] : kNaN;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    pod.base_q_xyzw[i] = base ? body.base_pose[3 + i] : kNaN;
  }
  if (base) {
    pod.truth_mask |= LocomotionTruthLogPod::kBaseBit;
  }

  // ── contact lane ─────────────────────────────────────────────────────────
  const std::size_t nc = std::min(in.contacts, LocomotionTruthLogPod::kMaxContacts);
  const auto stride = static_cast<std::size_t>(std::max(in.stride, 0));
  const bool lane = stride >= 4 && body.num_inference_groups == static_cast<int>(nc) &&
                    nc * stride <= body.inference_data.size();
  const bool moment = lane && in.moment && stride >= 7;
  for (std::size_t c = 0; c < LocomotionTruthLogPod::kMaxContacts; ++c) {
    double* w = &pod.grf[c * 6];
    double* cop = &pod.cop[c * 2];
    const bool in_model = c < nc;
    const int group = static_cast<int>(c);
    pod.contact_seq[c] = in_model ? body.inference_sequence[c] : 0;
    pod.contact_age_us[c] =
        in_model ? AgeUs(rtc::SensorGroupAgeNs(body, group, in.now_steady_ns)) : -1.0;
    const bool fresh =
        in_model && lane && rtc::IsSensorGroupFresh(body, group, in.now_steady_ns, in.max_age_ns);
    for (std::size_t k = 0; k < 6; ++k) {
      w[k] = kNaN;
    }
    cop[0] = kNaN;
    cop[1] = kNaN;
    if (!fresh) {
      continue;
    }
    pod.truth_mask |= 1U << static_cast<std::uint32_t>(1 + c);
    // The lane is foot-on-floor; these columns are floor-on-foot.
    const std::size_t at = c * stride;
    for (std::size_t k = 0; k < 3; ++k) {
      w[k] = -static_cast<double>(body.inference_data[at + 1 + k]);
    }
    if (moment) {
      for (std::size_t k = 0; k < 3; ++k) {
        w[3 + k] = -static_cast<double>(body.inference_data[at + 4 + k]);
      }
      // τ = p × f with p = (x, y, 0): τ_x = y f_z, τ_y = −x f_z.
      if (w[2] > in.cop_min_fz) {
        cop[0] = -w[4] / w[2];
        cop[1] = w[3] / w[2];
      }
    }
  }
}

}  // namespace integrated_bringup::locomotion
