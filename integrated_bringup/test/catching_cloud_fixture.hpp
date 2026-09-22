#ifndef INTEGRATED_BRINGUP_TEST_CATCHING_CLOUD_FIXTURE_H_
#define INTEGRATED_BRINGUP_TEST_CATCHING_CLOUD_FIXTURE_H_

// ── A ball_perception prediction message, built to order ────────────────────
//
// Shared by the decode suite (test_catching_traj_input) and the controller's
// end-to-end vision case (test_demo_catching_controller), which need the same
// bytes for different reasons: one asserts what the decoder does with them,
// the other that they survive a real subscription and reach the RT tick.
//
// The layout is ball_perception's `trajectory_layout` (384 B per point),
// reproduced rather than imported. That is deliberate — this is the fixture
// that fails when the publisher's layout moves, and one that imported the
// publisher's header would move with it silently. The decoder finds fields by
// NAME, so an offset-only change is expected to be accepted; `shift_layout`
// exists to assert exactly that.

#include "rtc_controllers/catching/traj_ingress.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace integrated_bringup::testing {

using PointField = sensor_msgs::msg::PointField;
using rtc::catching::CovarianceSnapshot;

// ball_perception `trajectory_layout` (see the header note).
constexpr std::uint32_t kOffX = 0;
constexpr std::uint32_t kOffV = 24;
constexpr std::uint32_t kOffA = 48;
constexpr std::uint32_t kOffCov = 72;
constexpr std::uint32_t kOffSeq = 360;
constexpr std::uint32_t kOffGen = 368;
constexpr std::uint32_t kOffHorizon = 376;
constexpr std::uint32_t kOffValidity = 380;
constexpr std::uint32_t kPointStep = 384;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kRecvSteady = 5'000 * kMs;
constexpr std::int64_t kRecvWall = 1'700'000'000LL * 1'000'000'000LL;  // a plausible epoch
constexpr std::uint64_t kActivation = 3;

struct CloudSpec {
  std::uint32_t n{8};
  std::uint64_t generation{42};
  std::uint64_t sequence{7};
  std::uint32_t step_ns{50'000'000};  // 50 ms, the shipped profile spacing
  /// Origin delay in ns: stamp = recv_wall − this. Negative ⇒ a future stamp.
  std::int64_t origin_delay_ns{30 * kMs};
  std::string frame_id{"world"};
  bool big_endian{false};
  std::uint32_t height{1};
  int invalid_point{-1};       // index whose `validity` is NOT_EVALUATED
  int differing_id_point{-1};  // index whose generation disagrees
  int nan_point{-1};           // index whose x is NaN
  bool non_monotonic{false};   // make point 2 land before point 1
  std::int64_t size_delta{0};  // corrupt data.size() by this many bytes
  bool shift_layout{false};    // same fields, different offsets
  bool drop_validity{false};   // omit the `validity` field entirely
  std::uint8_t validity_type{PointField::UINT8};
  std::uint32_t covariance_count{CovarianceSnapshot::kElems};
  /// Ball position at the message origin [m] and its (constant) velocity
  /// [m/s]. The defaults reproduce the diagonal the decode suite asserts on;
  /// a test that needs the ball inside a robot's workspace supplies its own.
  std::array<double, 3> p0{0.0, 0.0, 0.0};
  std::array<double, 3> vel{1.0, 2.0, 3.0};
};

inline void PutField(std::vector<PointField>& fields, const char* name, std::uint32_t offset,
                     std::uint8_t datatype, std::uint32_t count) {
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = count;
  fields.push_back(f);
}

template <typename T>
void Put(std::vector<std::uint8_t>& data, std::size_t point, std::uint32_t step,
         std::uint32_t offset, const T& value) {
  std::memcpy(data.data() + point * step + offset, &value, sizeof(T));
}

inline sensor_msgs::msg::PointCloud2 MakeCloud(const CloudSpec& spec) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = spec.height;
  msg.width = spec.n;
  msg.is_bigendian = spec.big_endian;
  msg.point_step = kPointStep;
  msg.header.frame_id = spec.frame_id;

  const std::int64_t stamp = kRecvWall - spec.origin_delay_ns;
  msg.header.stamp.sec = static_cast<std::int32_t>(stamp / 1'000'000'000LL);
  msg.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1'000'000'000LL);

  // An offset-only rearrangement: the identity block moves to the front and
  // the kinematics after it. Same names, same types, same point_step.
  const std::uint32_t off_x = spec.shift_layout ? 24 : kOffX;
  const std::uint32_t off_v = spec.shift_layout ? 48 : kOffV;
  const std::uint32_t off_a = spec.shift_layout ? 72 : kOffA;
  const std::uint32_t off_cov = spec.shift_layout ? 96 : kOffCov;
  const std::uint32_t off_seq = spec.shift_layout ? 0 : kOffSeq;
  const std::uint32_t off_gen = spec.shift_layout ? 8 : kOffGen;
  const std::uint32_t off_hor = spec.shift_layout ? 16 : kOffHorizon;
  const std::uint32_t off_val = spec.shift_layout ? 20 : kOffValidity;

  PutField(msg.fields, "x", off_x, PointField::FLOAT64, 1);
  PutField(msg.fields, "y", off_x + 8, PointField::FLOAT64, 1);
  PutField(msg.fields, "z", off_x + 16, PointField::FLOAT64, 1);
  PutField(msg.fields, "vx", off_v, PointField::FLOAT64, 1);
  PutField(msg.fields, "vy", off_v + 8, PointField::FLOAT64, 1);
  PutField(msg.fields, "vz", off_v + 16, PointField::FLOAT64, 1);
  PutField(msg.fields, "ax", off_a, PointField::FLOAT64, 1);
  PutField(msg.fields, "ay", off_a + 8, PointField::FLOAT64, 1);
  PutField(msg.fields, "az", off_a + 16, PointField::FLOAT64, 1);
  PutField(msg.fields, "covariance", off_cov, PointField::FLOAT64, spec.covariance_count);
  PutField(msg.fields, "snapshot_sequence", off_seq, PointField::UINT32, 2);
  PutField(msg.fields, "generation", off_gen, PointField::UINT32, 2);
  PutField(msg.fields, "horizon_ns", off_hor, PointField::UINT32, 1);
  if (!spec.drop_validity) {
    PutField(msg.fields, "validity", off_val, spec.validity_type, 1);
  }

  msg.row_step = kPointStep * spec.n;
  msg.data.assign(static_cast<std::size_t>(kPointStep) * spec.n, 0);

  for (std::uint32_t i = 0; i < spec.n; ++i) {
    std::uint32_t horizon = (i + 1) * spec.step_ns;
    if (spec.non_monotonic && i == 2) {
      horizon = spec.step_ns;  // same instant as point 0 → not increasing
    }
    Put(msg.data, i, kPointStep, off_hor, horizon);

    const double t = static_cast<double>(horizon) * 1e-9;
    const double px = spec.p0[0] + spec.vel[0] * t;
    const double x =
        (static_cast<int>(i) == spec.nan_point) ? std::numeric_limits<double>::quiet_NaN() : px;
    Put(msg.data, i, kPointStep, off_x, x);
    Put(msg.data, i, kPointStep, off_x + 8, spec.p0[1] + spec.vel[1] * t);
    Put(msg.data, i, kPointStep, off_x + 16, spec.p0[2] + spec.vel[2] * t);
    Put(msg.data, i, kPointStep, off_v, spec.vel[0]);
    Put(msg.data, i, kPointStep, off_v + 8, spec.vel[1]);
    Put(msg.data, i, kPointStep, off_v + 16, spec.vel[2]);
    Put(msg.data, i, kPointStep, off_a, 0.0);
    Put(msg.data, i, kPointStep, off_a + 8, 0.0);
    Put(msg.data, i, kPointStep, off_a + 16, -9.81);

    const std::uint64_t gen =
        (static_cast<int>(i) == spec.differing_id_point) ? spec.generation + 1 : spec.generation;
    Put(msg.data, i, kPointStep, off_gen, static_cast<std::uint32_t>(gen & 0xFFFFFFFFU));
    Put(msg.data, i, kPointStep, off_gen + 4, static_cast<std::uint32_t>(gen >> 32));
    Put(msg.data, i, kPointStep, off_seq, static_cast<std::uint32_t>(spec.sequence & 0xFFFFFFFFU));
    Put(msg.data, i, kPointStep, off_seq + 4, static_cast<std::uint32_t>(spec.sequence >> 32));

    const std::uint8_t validity = (static_cast<int>(i) == spec.invalid_point) ? 0 : 1;
    Put(msg.data, i, kPointStep, off_val, validity);

    // Diagonal covariance of (5 mm)², the shipped sim profile's value; the
    // off-diagonals stay 0 and the last element is set to NaN on one point so
    // the "unknown stays unknown" property has something to assert on.
    for (std::size_t k = 0; k < CovarianceSnapshot::kElems; ++k) {
      const bool diagonal = (k % 7) == 0;
      double value = diagonal ? 2.5e-5 : 0.0;
      if (i == 0 && k == CovarianceSnapshot::kElems - 1) {
        value = std::numeric_limits<double>::quiet_NaN();
      }
      Put(msg.data, i, kPointStep, off_cov + static_cast<std::uint32_t>(k * 8), value);
    }
  }
  if (spec.size_delta != 0) {
    msg.data.resize(
        static_cast<std::size_t>(static_cast<std::int64_t>(msg.data.size()) + spec.size_delta));
  }
  return msg;
}

}  // namespace integrated_bringup::testing

#endif  // INTEGRATED_BRINGUP_TEST_CATCHING_CLOUD_FIXTURE_H_
