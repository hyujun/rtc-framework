// ── Learned-policy tensor marshalling ────────────────────────────────────────
//
// The four operations that sit between a controller's state and a policy's
// flat float tensors: pack a run of doubles into the input buffer, apply the
// training-time affine normalisation, unpack a run of one output head, and
// blend two joint postures by a scalar.
//
// WHY THIS IS A CORE AND NOT BINDING GLUE. The boundary rule is "does this code
// have to know `RTControllerInterface` exists?" — none of it does. It sees
// spans of double and float and nothing else: no `ControllerState`, no engine,
// no ONNX. That also keeps `rtc_controllers` free of a dependency on
// `rtc_inference`; the two stay sibling layers and the buffers are the only
// contract between them.
//
// ROBOT-AGNOSTIC BY OMISSION (ARCH-1). There is no feature-name table here.
// Ids like "ur5e.position" are robot facts and live in the binding, which
// resolves them to element counts and hands this layer plain offsets/counts.
//
// RT: every function is noexcept, allocation-free, and branch-simple. The
// descriptors are built once at configure time so a tick only walks them.
#pragma once

#include <cmath>
#include <cstddef>
#include <span>

namespace rtc::inference {

/// One contiguous run of the flattened input tensor.
///
/// `offset`/`count` are element indices into the flattened buffer, not bytes
/// and not tensor dimensions — a policy input of shape [1, 34] is 34 elements
/// and a feature contributing 6 of them is `{offset, 6}` wherever the YAML
/// order happened to place it.
struct InputSegment {
  int offset{0};
  int count{0};
};

/// One contiguous run of one output head.
///
/// `head` indexes the model's output heads in the order they were declared,
/// which is also the order `rtc::InferenceEngine` binds them — positional, so
/// a swap of two heads with the SAME shape is undetectable here and is the
/// model's contract to keep. Heads of differing shape are caught by the
/// engine's own shape validation at load time.
struct OutputSlice {
  int head{0};
  int offset{0};
  int count{0};
};

/// Result of a posture blend. Two flags rather than one because the caller
/// reacts to them differently: `valid == false` is a hold, while `clamped` is
/// a warning that still ships a command (see BlendPosture).
struct PostureBlendReport {
  bool valid{false};    ///< spans agreed in length and `s` was finite
  bool clamped{false};  ///< `s` was outside [0, 1] and was pulled to the bound
};

/// Copy `src` into `buf[seg.offset, +seg.count)`, narrowing double → float.
///
/// Returns false and writes NOTHING when the segment would leave `buf` or when
/// `src` is shorter than `count`. Writing a partial segment would hand the
/// policy an observation that is half this tick and half the last one, which
/// is indistinguishable from a valid one downstream — the caller is expected
/// to turn a false into a hold rather than to run the model anyway.
[[nodiscard]] inline bool PackSegment(std::span<float> buf, const InputSegment& seg,
                                      std::span<const double> src) noexcept {
  if (seg.offset < 0 || seg.count < 0) {
    return false;
  }
  const auto off = static_cast<std::size_t>(seg.offset);
  const auto n = static_cast<std::size_t>(seg.count);
  if (off > buf.size() || n > buf.size() - off || src.size() < n) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    buf[off + i] = static_cast<float>(src[i]);
  }
  return true;
}

/// In-place `normalized = (raw - offset) * scale`, elementwise.
///
/// An EMPTY span is identity for that term, which is how a model that was
/// trained on raw units is configured (both keys omitted). A non-empty span
/// shorter than `buf` leaves the tail untouched rather than reading past its
/// end; the schema parser is what refuses that length mismatch up front, so
/// reaching the short-span path here means the descriptors were built by
/// something other than ParsePolicyIoParams.
///
/// Deliberately NOT finite-guarded: a NaN that arrives in `buf` must stay a
/// NaN so the caller's output/input validation sees it. Scrubbing it here
/// would launder a bad observation into a plausible one.
inline void ApplyAffine(std::span<float> buf, std::span<const float> offset,
                        std::span<const float> scale) noexcept {
  if (offset.empty() && scale.empty()) {
    return;
  }
  for (std::size_t i = 0; i < buf.size(); ++i) {
    float v = buf[i];
    if (i < offset.size()) {
      v -= offset[i];
    }
    if (i < scale.size()) {
      v *= scale[i];
    }
    buf[i] = v;
  }
}

/// Copy `head_buf[slice.offset, +slice.count)` into `out`, widening float →
/// double. `head_size` is the element count of that head's buffer.
///
/// Returns false and writes nothing when the slice leaves the head or `out` is
/// too small — same all-or-nothing reason as PackSegment. A null `head_buf` is
/// a false rather than a crash because that is exactly what the stub inference
/// engine returns, and the caller's hold path is the right response to it.
///
/// Finiteness is NOT checked here. The caller screens the unpacked doubles
/// (`rtc::compliance::AllFinite` over an Eigen::Map) before they reach a
/// command, and doing it in one place keeps the "which stage rejected this"
/// answer unambiguous.
[[nodiscard]] inline bool UnpackSlice(std::span<double> out, const OutputSlice& slice,
                                      const float* head_buf, std::size_t head_size) noexcept {
  if (head_buf == nullptr || slice.offset < 0 || slice.count < 0) {
    return false;
  }
  const auto off = static_cast<std::size_t>(slice.offset);
  const auto n = static_cast<std::size_t>(slice.count);
  if (off > head_size || n > head_size - off || out.size() < n) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = static_cast<double>(head_buf[off + i]);
  }
  return true;
}

/// `out = open + clamp(s, 0, 1) * (close - open)`, elementwise.
///
/// The scalar is a policy output, so nothing guarantees it landed in [0, 1] —
/// a model without a squashing head, or one whose normalisation does not match
/// the YAML, will hand over 1.7 or -0.3. Clamping is the right response
/// (the interpolation stays inside the two configured postures, which are
/// themselves inside the joint limits) but it is NOT silent: `clamped` is
/// reported so the caller can warn. An out-of-range scalar is a normalisation
/// mismatch signal, not a normal operating condition.
///
/// A NON-FINITE `s` is rejected outright instead of being clamped. Comparisons
/// against NaN are all false, so a hand-rolled clamp would pass it straight
/// through into every joint of the posture, and `std::clamp(NaN, lo, hi)`
/// returns NaN for the same reason — either way a fault would arrive at the
/// actuator boundary wearing a plausible shape.
[[nodiscard]] inline PostureBlendReport BlendPosture(std::span<double> out,
                                                     std::span<const double> open,
                                                     std::span<const double> close,
                                                     double s) noexcept {
  PostureBlendReport report;
  const std::size_t n = out.size();
  if (n == 0 || open.size() < n || close.size() < n || !std::isfinite(s)) {
    return report;
  }
  double t = s;
  if (t < 0.0) {
    t = 0.0;
    report.clamped = true;
  } else if (t > 1.0) {
    t = 1.0;
    report.clamped = true;
  }
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = open[i] + (t * (close[i] - open[i]));
  }
  report.valid = true;
  return report;
}

}  // namespace rtc::inference
