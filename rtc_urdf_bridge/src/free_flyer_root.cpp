// ── free_flyer_root 구현 ─────────────────────────────────────────────────────
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <boost/variant/get.hpp>

namespace rtc_urdf_bridge {

bool IsFreeFlyerRoot(const pinocchio::Model& model) noexcept {
  if (model.joints.size() < 2) {
    return false;
  }
  // A variant type check, not `shortname()` (which builds a std::string): this
  // keeps the predicate allocation-free, so RT-callable helpers may use it.
  return boost::get<pinocchio::JointModelFreeFlyer>(&model.joints[1].toVariant()) != nullptr;
}

}  // namespace rtc_urdf_bridge
