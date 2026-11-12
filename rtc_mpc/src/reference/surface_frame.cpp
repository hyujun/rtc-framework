#include "rtc_mpc/reference/surface_frame.hpp"

#include "reference/internal/yaml_fields.hpp"

namespace rtc::mpc {

SurfaceFrameError LoadTerrainConfig(const YAML::Node& node, SurfaceFrame& out) noexcept {
  try {
    if (!node.IsDefined() || !node.IsMap()) {
      return SurfaceFrameError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(node, {"height"})) {
      return SurfaceFrameError::kUnknownKey;
    }
    double height = 0.0;
    const internal::FieldRead r = internal::ReadScalar(internal::Child(node, "height"), height);
    if (r != internal::FieldRead::kOk) {
      return internal::ToError<SurfaceFrameError>(r);
    }
    const SurfaceFrame s = FlatSurface(height);
    const SurfaceFrameError e = ValidateSurfaceFrame(s);
    if (e == SurfaceFrameError::kNoError) {
      out = s;
    }
    return e;
  } catch (...) {
    return SurfaceFrameError::kWrongType;
  }
}

}  // namespace rtc::mpc
