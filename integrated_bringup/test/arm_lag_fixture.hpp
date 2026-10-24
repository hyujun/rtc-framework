#ifndef INTEGRATED_BRINGUP_TEST_ARM_LAG_FIXTURE_H_
#define INTEGRATED_BRINGUP_TEST_ARM_LAG_FIXTURE_H_

// ── A pure transport delay between command and measurement ──────────────────
//
// The lead compensation reads the prediction at now + T_arm because the command
// it builds is realised T_arm later. With no delay, leading and not leading
// produce the same trajectory and a before/after measurement of the
// compensation is vacuous, so the delay is supplied here, in a fixture, not in
// the runtime. It lives under test/ and is never installed, so no production
// target can reach it (the same isolation `catching_ball_fixture.hpp` uses).
// The sim arm itself is a first-order lag of the servo's kd/kp (0.05 s on
// ur5e_p1b), which the catch_lead sim overlays and the shipped YAML carry.
//
// The model is a PURE delay, not first-order-plus-delay: the lead compensation
// is exactly the inverse of a pure delay, so this fixture measures how much of
// the error the compensation can remove IN PRINCIPLE. A plant with a time
// constant would fold the identification error of that constant into the same
// number, which is a real-hardware question.

#include <array>
#include <cstddef>
#include <deque>

namespace integrated_bringup::testing {

/// Delays each commanded joint vector by a fixed number of control periods.
///
/// The queue is primed with the starting configuration, so the first
/// `delay_ticks` measurements are "the arm has not moved yet" rather than
/// zeros — a plant that reported the origin for its first few ticks would make
/// every controller look like it diverged.
template <std::size_t N>
class ArmLagPlant {
 public:
  ArmLagPlant(int delay_ticks, const std::array<double, N>& initial) {
    for (int i = 0; i < delay_ticks; ++i) {
      queue_.push_back(initial);
    }
    current_ = initial;
  }

  /// Push this tick's command, return this tick's measurement.
  const std::array<double, N>& Step(const std::array<double, N>& command) {
    queue_.push_back(command);
    current_ = queue_.front();
    queue_.pop_front();
    return current_;
  }

  [[nodiscard]] const std::array<double, N>& Measured() const noexcept { return current_; }

 private:
  std::deque<std::array<double, N>> queue_;
  std::array<double, N> current_{};
};

}  // namespace integrated_bringup::testing

#endif  // INTEGRATED_BRINGUP_TEST_ARM_LAG_FIXTURE_H_
