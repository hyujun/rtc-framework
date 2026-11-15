// A walking-like contact schedule for the locomotion rig (loco_wbc_fixture.hpp)
// that keeps the kinematics consistent with the contacts: the base stays where
// q_stand puts it and a leg's joints move only while that leg's foot is in
// swing, so a stance sole never moves in the world. Arms, waist and swing legs
// follow 0.02 rad sines. Optional Gaussian noise on q (σ) and v (10 σ) stands in
// for an estimator. Test-local, not installed.
//
// One cycle is kCycle ticks: double support, slot 1 released, double support,
// slot 0 released. The ticks on which the contact set changes are transitions.
#pragma once

#include "loco_wbc_fixture.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace rtc::tsid::test {

class LocoGait {
 public:
  static constexpr int kCycle = 1100;
  static constexpr int kSlot1Off = 250;  // [kSlot1Off, kDouble2): slot 1 released
  static constexpr int kDouble2 = 550;   // [kDouble2, kSlot0Off): double support
  static constexpr int kSlot0Off = 800;  // [kSlot0Off, kCycle): slot 0 released

  explicit LocoGait(const pinocchio::Model& model, double noise_q = 0.0,
                    std::uint64_t seed = 20261115)
      : model_(model),
        q_stand_(QStand(model)),
        noise_q_(noise_q),
        rng_(seed),
        leg_of_(static_cast<std::size_t>(model.njoints), -1) {
    for (int j = 2; j < model.njoints; ++j) {
      const std::string& name = model.names[static_cast<std::size_t>(j)];
      const bool leg = name.find("hip") != std::string::npos ||
                       name.find("knee") != std::string::npos ||
                       name.find("ankle") != std::string::npos;
      if (leg && name.rfind("left_", 0) == 0) {
        leg_of_[static_cast<std::size_t>(j)] = 0;
      } else if (leg && name.rfind("right_", 0) == 0) {
        leg_of_[static_cast<std::size_t>(j)] = 1;
      }
    }
  }

  static bool IsTransition(int t) {
    const int ct = t % kCycle;
    return (ct == 0 && t > 0) || ct == kSlot1Off || ct == kDouble2 || ct == kSlot0Off;
  }

  // Tick t on `rig`: the contact set, the state (q, v) with the cache updated,
  // and the reference (q_stand at rest, the weight shared by the stance feet).
  void Apply(int t, LocoRig& rig) {
    const int ct = t % kCycle;
    const bool on[2] = {ct < kSlot0Off, ct < kSlot1Off || ct >= kDouble2};
    for (std::size_t i = 0; i < 2 && i < rig.contacts.contacts.size(); ++i) {
      auto& c = rig.contacts.contacts[i];
      c.active = on[i];
      c.activation = on[i] ? 1.0 : 0.0;
      c.activation_target = on[i] ? 1.0 : 0.0;
    }
    rig.contacts.RecomputeActive(rig.b.Contacts());

    const double time = kLocoDt * t;
    Eigen::VectorXd q = q_stand_;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    for (int j = 2; j < model_.njoints; ++j) {
      const auto& joint = model_.joints[static_cast<std::size_t>(j)];
      const int leg = leg_of_[static_cast<std::size_t>(j)];
      const bool moves = leg < 0 || !rig.contacts.contacts[static_cast<std::size_t>(leg)].active;
      if (moves) {
        const double ph = 2.0 * M_PI * 0.5 * time + 0.7 * (j - 2);
        q(joint.idx_q()) += 0.02 * std::sin(ph);
        v(joint.idx_v()) = 0.02 * 2.0 * M_PI * 0.5 * std::cos(ph);
      }
      if (noise_q_ > 0.0) {
        q(joint.idx_q()) += noise_q_ * gauss_(rng_);
        v(joint.idx_v()) += 10.0 * noise_q_ * gauss_(rng_);
      }
    }
    rig.state.q = q;
    rig.state.v = v;
    rig.cache.Update(q, v);
    rig.ref.q_des = q_stand_;
    rig.ref.v_des.setZero();
    rig.ref.a_des.setZero();
    rig.ref.lambda_des.setZero();
    const double share = rig.contacts.active_count > 0 ? pinocchio::computeTotalMass(model_) *
                                                             9.81 / rig.contacts.active_count
                                                       : 0.0;
    int offset = 0;
    for (std::size_t i = 0; i < rig.contacts.contacts.size(); ++i) {
      if (rig.contacts.contacts[i].active) {
        rig.ref.lambda_des(offset + 2) = share;
      }
      offset += rig.b.Contacts().contacts[i].contact_dim;
    }
  }

 private:
  const pinocchio::Model& model_;
  Eigen::VectorXd q_stand_;
  double noise_q_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
  std::vector<int> leg_of_;  // per joint: 0 / 1 for a leg slot, −1 otherwise
};

}  // namespace rtc::tsid::test
