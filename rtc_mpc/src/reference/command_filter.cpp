#include "rtc_mpc/reference/command_filter.hpp"

#include "reference/internal/yaml_fields.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::mpc {
namespace {

using internal::FieldRead;

[[nodiscard]] bool AllFinite(const CommandFilterConfig& c) noexcept {
  bool ok = std::isfinite(c.sample_rate_hz) && std::isfinite(c.cutoff_hz) &&
            std::isfinite(c.t_stale) && std::isfinite(c.base_height) && std::isfinite(c.h_min) &&
            std::isfinite(c.h_max);
  for (std::size_t i = 0; i < kCommandAxes; ++i) {
    ok = ok && std::isfinite(c.u_max[i]) && std::isfinite(c.accel_max[i]);
  }
  return ok;
}

}  // namespace

CommandFilterError ValidateCommandFilterConfig(const CommandFilterConfig& c) noexcept {
  if (!AllFinite(c)) {
    return CommandFilterError::kNonFinite;
  }
  if (c.sample_rate_hz <= 0.0) {
    return CommandFilterError::kInvalidSampleRate;
  }
  if (c.cutoff_hz <= 0.0 || c.cutoff_hz >= 0.5 * c.sample_rate_hz) {
    return CommandFilterError::kInvalidCutoff;
  }
  for (std::size_t i = 0; i < kCommandAxes; ++i) {
    if (c.u_max[i] < 0.0 || c.accel_max[i] < 0.0) {
      return CommandFilterError::kNegativeLimit;
    }
  }
  if (!(c.t_stale >= 1.0 / c.sample_rate_hz) || c.t_stale > kMaxStaleTimeout) {
    return CommandFilterError::kInvalidStaleTimeout;
  }
  if (c.h_min > c.h_max || c.base_height < c.h_min || c.base_height > c.h_max) {
    return CommandFilterError::kInvalidHeightRange;
  }
  return CommandFilterError::kNoError;
}

CommandFilterError LoadCommandFilterConfig(const YAML::Node& node,
                                           CommandFilterConfig& out) noexcept {
  try {
    if (!node.IsDefined() || !node.IsMap()) {
      return CommandFilterError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(node, {"sample_rate_hz", "cutoff_hz", "v_max", "hdot_max",
                                      "accel_max", "t_stale", "base_height", "h_min", "h_max"})) {
      return CommandFilterError::kUnknownKey;
    }
    CommandFilterConfig c;
    std::array<double, 3> v_max{};
    double hdot_max = 0.0;
    const FieldRead reads[] = {
        internal::ReadScalar(internal::Child(node, "sample_rate_hz"), c.sample_rate_hz),
        internal::ReadScalar(internal::Child(node, "cutoff_hz"), c.cutoff_hz),
        internal::ReadArray(internal::Child(node, "v_max"), v_max),
        internal::ReadScalar(internal::Child(node, "hdot_max"), hdot_max),
        internal::ReadArray(internal::Child(node, "accel_max"), c.accel_max),
        internal::ReadScalar(internal::Child(node, "t_stale"), c.t_stale),
        internal::ReadScalar(internal::Child(node, "base_height"), c.base_height),
        internal::ReadScalar(internal::Child(node, "h_min"), c.h_min),
        internal::ReadScalar(internal::Child(node, "h_max"), c.h_max),
    };
    for (const FieldRead r : reads) {
      if (r != FieldRead::kOk) {
        return internal::ToError<CommandFilterError>(r);
      }
    }
    c.u_max = {v_max[0], v_max[1], v_max[2], hdot_max};
    const CommandFilterError e = ValidateCommandFilterConfig(c);
    if (e == CommandFilterError::kNoError) {
      out = c;
    }
    return e;
  } catch (...) {
    return CommandFilterError::kWrongType;
  }
}

CommandFilterError CommandFilter::Init(const CommandFilterConfig& cfg) noexcept {
  const CommandFilterError e = ValidateCommandFilterConfig(cfg);
  if (e != CommandFilterError::kNoError) {
    return e;
  }
  rtc::BesselFilterN<kCommandAxes> designed;
  try {
    designed.Init(cfg.cutoff_hz, cfg.sample_rate_hz);  // pre-checked above: cannot throw
  } catch (...) {
    return CommandFilterError::kInvalidCutoff;
  }
  cfg_ = cfg;
  bessel_ = designed;
  t_stale_ns_ = std::llround(cfg.t_stale * 1e9);
  initialized_ = true;
  return Reset(cfg.base_height);
}

CommandFilterError CommandFilter::Reset(double h0) noexcept {
  if (!initialized_) {
    return CommandFilterError::kNotInitialised;
  }
  if (!std::isfinite(h0) || h0 < cfg_.h_min || h0 > cfg_.h_max) {
    return CommandFilterError::kHeightOutOfRange;
  }
  bessel_.Reset();
  accepted_.fill(0.0);
  limited_.fill(0.0);
  out_ = FilteredCommand{};
  out_.base_height = h0;
  return CommandFilterError::kNoError;
}

const FilteredCommand& CommandFilter::Step(const RawVelocityCommand& raw, std::int64_t raw_stamp_ns,
                                           std::int64_t now_ns) noexcept {
  if (!initialized_) {
    return out_;
  }
  ++out_.sequence;
  out_.stamp_ns = now_ns;

  if (raw.angular_x != 0.0 || raw.angular_y != 0.0) {
    ++out_.ignored_axis_count;
  }

  // 1. stale — signed age, either side; an overflowing difference is stale.
  std::int64_t age = 0;
  const bool overflow = __builtin_sub_overflow(now_ns, raw_stamp_ns, &age);
  out_.fresh = !overflow && age <= t_stale_ns_ && age >= -t_stale_ns_;
  if (!out_.fresh) {
    accepted_.fill(0.0);
    ++out_.stale_count;
  } else {
    out_.valid = true;
    // 2. finite gate — a non-finite axis keeps its last accepted value.
    const std::array<double, kCommandAxes> in = {raw.linear_x, raw.linear_y, raw.angular_z,
                                                 raw.linear_z};
    bool any_nonfinite = false;
    for (std::size_t i = 0; i < kCommandAxes; ++i) {
      if (std::isfinite(in[i])) {
        accepted_[i] = in[i];
      } else {
        any_nonfinite = true;
      }
    }
    if (any_nonfinite) {
      ++out_.nonfinite_count;
    }
  }

  // 3. clamp, 4. slew (0 = off), 5. Bessel, 6. final clamp.
  const double dt = 1.0 / cfg_.sample_rate_hz;
  for (std::size_t i = 0; i < kCommandAxes; ++i) {
    const double clamped = std::clamp(accepted_[i], -cfg_.u_max[i], cfg_.u_max[i]);
    limited_[i] = SlewLimit(limited_[i], clamped, cfg_.accel_max[i] * dt);
  }
  std::array<double, kCommandAxes> y = bessel_.Apply(limited_);
  for (std::size_t i = 0; i < kCommandAxes; ++i) {
    y[i] = std::clamp(y[i], -cfg_.u_max[i], cfg_.u_max[i]);
  }

  // 7. integrate ḣ into the clamped height (the state saturates with it).
  out_.v_x = y[0];
  out_.v_y = y[1];
  out_.omega_z = y[2];
  out_.base_height = std::clamp(out_.base_height + y[3] * dt, cfg_.h_min, cfg_.h_max);
  return out_;
}

}  // namespace rtc::mpc
