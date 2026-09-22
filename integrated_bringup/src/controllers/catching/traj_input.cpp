#include "integrated_bringup/controllers/catching/traj_input.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cstring>

namespace integrated_bringup {

namespace {

using rtc::catching::BallTime;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::TrajectorySnapshot;

// The PointField datatype constants are static members of the message class,
// not free constants; naming them once here keeps the checks readable and the
// numbers out of the code (`8` says nothing; FLOAT64 does).

constexpr std::uint8_t kU8 = sensor_msgs::msg::PointField::UINT8;
constexpr std::uint8_t kU32 = sensor_msgs::msg::PointField::UINT32;
constexpr std::uint8_t kF64 = sensor_msgs::msg::PointField::FLOAT64;

constexpr std::uint8_t kValidityValid = 1;  // ball_perception: 0 = NOT_EVALUATED, 1 = VALID

/// Field names in the order the snapshot's p/v/a triples are filled.
constexpr std::array<const char*, 9> kPvaNames = {"x",  "y",  "z",  "vx", "vy",
                                                  "vz", "ax", "ay", "az"};

/// Byte width of one element of a datatype. 0 for anything unrecognised,
/// which fails the bounds check rather than being assumed.
[[nodiscard]] std::uint32_t ElementSize(std::uint8_t datatype) noexcept {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

/// FNV-1a over (name, offset, datatype, count) plus point_step and
/// endianness. Only ever compared with itself, so the choice of hash is
/// irrelevant beyond being cheap and order-sensitive — the field ORDER moving
/// is exactly one of the changes worth noticing.
[[nodiscard]] std::uint64_t LayoutHash(const sensor_msgs::msg::PointCloud2& msg) noexcept {
  std::uint64_t h = 1469598103934665603ULL;
  const auto mix = [&h](std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      h ^= (v >> (8 * i)) & 0xFFULL;
      h *= 1099511628211ULL;
    }
  };
  for (const auto& f : msg.fields) {
    for (const char c : f.name) {
      h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
      h *= 1099511628211ULL;
    }
    mix(f.offset);
    mix(f.datatype);
    mix(f.count);
  }
  mix(msg.point_step);
  mix(msg.is_bigendian ? 1U : 0U);
  return h;
}

/// Locate one field by name and check its datatype, count and bounds.
[[nodiscard]] CloudReject Locate(const sensor_msgs::msg::PointCloud2& msg, const char* name,
                                 std::uint8_t datatype, std::uint32_t count,
                                 FieldSlot& out) noexcept {
  for (const auto& f : msg.fields) {
    if (f.name != name) {
      continue;
    }
    if (f.datatype != datatype || f.count != count) {
      return CloudReject::kFieldType;
    }
    const std::uint32_t elem = ElementSize(f.datatype);
    if (elem == 0) {
      return CloudReject::kFieldType;
    }
    // Bounds BEFORE the map is usable, not at read time: a field that would
    // read past the point is a malformed layout, and catching it here means
    // the per-point loop can memcpy without re-deriving the same check for
    // every point of every message.
    const std::uint64_t end = static_cast<std::uint64_t>(f.offset) +
                              static_cast<std::uint64_t>(elem) * static_cast<std::uint64_t>(count);
    if (end > msg.point_step) {
      return CloudReject::kFieldBounds;
    }
    out.offset = f.offset;
    out.found = true;
    return CloudReject::kNone;
  }
  return CloudReject::kMissingField;
}

/// Read one POD out of a point. `memcpy` rather than a reinterpret_cast: the
/// offsets are publisher-chosen and carry no alignment guarantee, and a
/// misaligned load of a double is undefined behaviour rather than a slow one.
template <typename T>
[[nodiscard]] T ReadAt(const std::uint8_t* point, std::uint32_t offset) noexcept {
  T v{};
  std::memcpy(&v, point + offset, sizeof(T));
  return v;
}

/// The wire form of a uint64: two UINT32s, low then high.
[[nodiscard]] std::uint64_t ReadSplitU64(const std::uint8_t* point, std::uint32_t offset) noexcept {
  const auto low = ReadAt<std::uint32_t>(point, offset);
  const auto high = ReadAt<std::uint32_t>(point, offset + 4);
  return static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high) << 32);
}

}  // namespace

const char* CloudRejectName(CloudReject r) noexcept {
  switch (r) {
    case CloudReject::kNone:
      return "none";
    case CloudReject::kBigEndian:
      return "big_endian";
    case CloudReject::kShape:
      return "shape";
    case CloudReject::kSize:
      return "size";
    case CloudReject::kMissingField:
      return "missing_field";
    case CloudReject::kFieldType:
      return "field_type";
    case CloudReject::kFieldBounds:
      return "field_bounds";
    case CloudReject::kFrameId:
      return "frame_id";
    case CloudReject::kFutureStamp:
      return "future_stamp";
    case CloudReject::kStampOverflow:
      return "stamp_overflow";
    case CloudReject::kNotEvaluated:
      return "not_evaluated";
    case CloudReject::kStaleSequence:
      return "stale_sequence";
    case CloudReject::kInconsistentId:
      return "inconsistent_id";
    case CloudReject::kMalformed:
      return "malformed";
  }
  return "unknown";
}

CloudReject BuildFieldMap(const sensor_msgs::msg::PointCloud2& msg, TrajFieldMap& out) noexcept {
  out = TrajFieldMap{};
  out.point_step = msg.point_step;
  out.layout_hash = LayoutHash(msg);

  for (std::size_t i = 0; i < kPvaNames.size(); ++i) {
    const CloudReject r = Locate(msg, kPvaNames[i], kF64, 1, out.pva[i]);
    if (r != CloudReject::kNone) {
      return r;
    }
  }

  struct Required {
    const char* name;
    std::uint8_t datatype;
    std::uint32_t count;
    FieldSlot* slot;
  };

  const std::array<Required, 5> required = {{
      {"covariance", kF64, CovarianceSnapshot::kElems, &out.covariance},
      {"snapshot_sequence", kU32, 2, &out.snapshot_sequence},
      {"generation", kU32, 2, &out.generation},
      {"horizon_ns", kU32, 1, &out.horizon_ns},
      {"validity", kU8, 1, &out.validity},
  }};
  for (const auto& req : required) {
    const CloudReject r = Locate(msg, req.name, req.datatype, req.count, *req.slot);
    if (r != CloudReject::kNone) {
      return r;
    }
  }
  out.ok = true;
  return CloudReject::kNone;
}

void CatchingTrajInput::Configure(const TrajInputConfig& cfg) noexcept {
  cfg_ = cfg;
  Reset();
}

void CatchingTrajInput::Reset() noexcept {
  order_ = rtc::catching::IngressState{};
  previous_ = TrajectorySnapshot{};
  map_ = TrajFieldMap{};
  diag_ = TrajInputDiagnostics{};
}

std::uint64_t CatchingTrajInput::RejectCount(CloudReject r) const noexcept {
  const auto idx = static_cast<std::size_t>(r);
  return idx < rejects_.size() ? rejects_[idx] : 0;
}

CloudReject CatchingTrajInput::OnCloud(const sensor_msgs::msg::PointCloud2& msg,
                                       std::int64_t recv_steady_ns, std::int64_t recv_wall_ns,
                                       std::uint64_t activation_generation,
                                       TrajectorySnapshot& snap, CovarianceSnapshot& cov) noexcept {
  const auto fail = [this](CloudReject r) {
    rejects_[static_cast<std::size_t>(r)] += 1;
    return r;
  };

  // ── Shape and layout, all of it BEFORE a single point is indexed ─────────
  // The reference implementation indexed first and range-checked afterwards,
  // which ASan found reading past the end on an over-long message (S1.2). The
  // order here is the fix, and it is the reason every check below is a return
  // rather than a flag.
  if (msg.is_bigendian) {
    return fail(CloudReject::kBigEndian);
  }
  if (!cfg_.expected_frame.empty() && msg.header.frame_id != cfg_.expected_frame) {
    // Compared on EVERY message, not once: a publisher that changes frame
    // mid-run is publishing numbers in a different space, and the values stay
    // entirely plausible. (S3.4 measured `world` on both robots, so this is a
    // guard against change rather than a conversion — L1 §4.3's transform is
    // not needed and is not implemented.)
    return fail(CloudReject::kFrameId);
  }
  if (msg.height != 1) {
    return fail(CloudReject::kShape);
  }
  const auto width = static_cast<std::int64_t>(msg.width);
  if (width < cfg_.n_min || width > cfg_.n_max ||
      width > static_cast<std::int64_t>(rtc::catching::kCap)) {
    return fail(CloudReject::kShape);
  }
  const std::uint64_t expected_bytes =
      static_cast<std::uint64_t>(msg.point_step) * static_cast<std::uint64_t>(msg.width);
  if (msg.point_step == 0 || msg.data.size() != expected_bytes ||
      static_cast<std::uint64_t>(msg.row_step) != expected_bytes) {
    return fail(CloudReject::kSize);
  }

  // Rebuild the map only when the layout actually changed. `ok` is part of the
  // condition so a layout that failed to map is retried rather than remembered
  // as broken — the publisher may be mid-restart.
  const std::uint64_t hash = LayoutHash(msg);
  if (!map_.ok || hash != map_.layout_hash) {
    const CloudReject r = BuildFieldMap(msg, map_);
    diag_.layout_rebuilds += 1;
    if (r != CloudReject::kNone) {
      return fail(r);
    }
  }

  // ── Identity and validity ────────────────────────────────────────────────
  const std::uint8_t* const base = msg.data.data();
  const auto point_at = [base, &msg](std::int64_t i) {
    return base + static_cast<std::size_t>(i) * static_cast<std::size_t>(msg.point_step);
  };

  const std::uint64_t generation = ReadSplitU64(point_at(0), map_.generation.offset);
  const std::uint64_t sequence = ReadSplitU64(point_at(0), map_.snapshot_sequence.offset);
  bool all_valid = true;
  for (std::int64_t i = 0; i < width; ++i) {
    const std::uint8_t* p = point_at(i);
    if (ReadAt<std::uint8_t>(p, map_.validity.offset) != kValidityValid) {
      all_valid = false;
      break;
    }
    // The identity fields are per-point on the wire but describe the MESSAGE.
    // A message whose points disagree is not a message with a minor defect —
    // it is one whose identity we cannot state, and accepting it would file
    // some other track's points under this one.
    if (i > 0 && (ReadSplitU64(p, map_.generation.offset) != generation ||
                  ReadSplitU64(p, map_.snapshot_sequence.offset) != sequence)) {
      return fail(CloudReject::kInconsistentId);
    }
  }

  // Order BEFORE decode, and it records what it accepted even though the
  // content check below can still refuse the message. That is deliberate on
  // both counts: the cheap check rejects the duplicates that dominate a
  // backlog without touching a byte of payload, and the memory is "the highest
  // sequence seen in this track", which a re-send of a malformed snapshot has
  // no claim to reopen. Sequences only ever increase in a track, so the case
  // where this matters — vision re-sending the same number with fixed content
  // — does not arise; if it ever did, the next number would be accepted.
  const rtc::catching::IngressReject order =
      rtc::catching::CheckOrder({generation, sequence, all_valid}, order_);
  if (order == rtc::catching::IngressReject::kNotEvaluated) {
    return fail(CloudReject::kNotEvaluated);
  }
  if (order == rtc::catching::IngressReject::kStaleSequence) {
    return fail(CloudReject::kStaleSequence);
  }

  // ── D-2 conversion, once ─────────────────────────────────────────────────
  const std::int64_t stamp_ns = static_cast<std::int64_t>(msg.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(msg.header.stamp.nanosec);
  const auto conv = rtc::catching::ConvertRemoteStamp(rtc::catching::NowReal{recv_steady_ns},
                                                      recv_wall_ns, stamp_ns, cfg_.future_tol_ns);
  diag_.origin_delay_ns = conv.origin_delay_ns;  // diagnostic on BOTH paths (L1 §4.1)
  if (!conv.IsOk()) {
    return fail(conv.status == rtc::catching::StampStatus::kFutureStamp
                    ? CloudReject::kFutureStamp
                    : CloudReject::kStampOverflow);
  }

  // ── Decode into a LOCAL snapshot ─────────────────────────────────────────
  // Local, then assigned on success: writing into the caller's buffer as we go
  // would leave a half-decoded trajectory behind on a late rejection, and the
  // caller has no way to tell that from a whole one.
  TrajectorySnapshot decoded{};
  decoded.n = static_cast<std::int32_t>(width);
  decoded.valid = true;
  CovarianceSnapshot decoded_cov{};
  decoded_cov.n = decoded.n;
  decoded_cov.valid = true;

  for (std::int64_t i = 0; i < width; ++i) {
    const std::uint8_t* p = point_at(i);
    const auto idx = static_cast<std::size_t>(i);
    auto& s = decoded.s[idx];
    for (std::size_t k = 0; k < 3; ++k) {
      s.p[k] = ReadAt<double>(p, map_.pva[k].offset);
      s.v[k] = ReadAt<double>(p, map_.pva[3 + k].offset);
      s.a[k] = ReadAt<double>(p, map_.pva[6 + k].offset);
    }
    // `horizon_ns` is UNSIGNED on the wire and relative to the origin stamp,
    // so the sample instant is the converted origin plus it. SampleBallTime
    // reports the overflow instead of wrapping — a garbage horizon must
    // refuse the message, not produce a ball instant in the past.
    const auto horizon = ReadAt<std::uint32_t>(p, map_.horizon_ns.offset);
    BallTime t{};
    if (!rtc::catching::SampleBallTime(conv.t_ref, static_cast<std::int64_t>(horizon), t)) {
      return fail(CloudReject::kStampOverflow);
    }
    s.t_ns = t.ns;

    std::memcpy(decoded_cov.c[idx].data(), p + map_.covariance.offset,
                sizeof(double) * CovarianceSnapshot::kElems);
  }

  // ── Format check (L2's gate, the single owner of these rules) ────────────
  const rtc::catching::TrajLimits limits{cfg_.n_min, cfg_.n_max, cfg_.dt_min_ns};
  const rtc::catching::TrajCheck check = rtc::catching::Check(decoded, limits);
  if (!check.ok) {
    return fail(CloudReject::kMalformed);
  }

  // ── Token (D-22) and diagnostics ─────────────────────────────────────────
  decoded.token.activation_generation = activation_generation;
  decoded.token.generation = generation;
  decoded.token.snapshot_sequence = sequence;
  decoded.token.traj_recv_ns = recv_steady_ns;
  decoded_cov.token = decoded.token;

  diag_.n = decoded.n;
  diag_.accepted_sequence = sequence;
  diag_.accepted_generation = generation;
  diag_.horizon_ns = check.horizon_ns;
  diag_.horizon_short = rtc::catching::HorizonShort(decoded, cfg_.horizon_min_ns);
  diag_.jump_m = rtc::catching::JumpBetween(previous_, decoded, cfg_.track_eval_offset_ns);

  previous_ = decoded;
  snap = decoded;
  cov = decoded_cov;
  accept_count_ += 1;
  return CloudReject::kNone;
}

}  // namespace integrated_bringup
