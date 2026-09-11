// ── Learned-policy tensor marshalling ────────────────────────────────────────
//
// The four operations that sit between a controller's state and a policy's
// flat float tensors: pack a run of doubles into one input tensor, apply that
// tensor's training-time affine normalisation, unpack a run of one output
// tensor, blend two joint postures by a scalar, and feed a recurrent state
// tensor back into the next step.
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
#include <vector>

namespace rtc::inference {

/// Where ONE feature lands in ONE flattened input tensor — either a contiguous
/// run or, when the tensor names its elements, a scatter.
///
/// `offset`/`count` are element indices into that tensor's flattened buffer,
/// not bytes and not tensor dimensions — a policy input of shape [1, 34] is 34
/// elements and a feature contributing 6 of them is `{tensor, offset, 6}`
/// wherever the YAML order happened to place it.
///
/// `indices`, when non-empty, REPLACES the run: value i goes to element
/// `indices[i]` and `offset` is not consulted (`count == indices.size()`). This
/// is how a feature is placed BY NAME into a tensor that declares
/// `element_names` — an exported state vector that lists 46 joint slots of which
/// the robot fills 16, in an order the export chose. The indices are resolved
/// once at configure; a vector rather than a view into the spec so a copied
/// descriptor can never outlive the storage it points into. The tick only reads
/// it (by const reference), so it never allocates.
///
/// `tensor` indexes the model's input tensors in declaration order, which is
/// the order `rtc::InferenceEngine::input_buffer(model, input_idx)` uses. It is
/// part of the address and not a convenience: a multi-input policy has one
/// independent element space PER tensor, so an offset means nothing without the
/// tensor it belongs to, and a descriptor that lost its tensor would still
/// address a perfectly valid element of the wrong one.
struct InputSegment {
  int tensor{0};
  int offset{0};
  int count{0};
  // `{}` is not redundant whatever clang-tidy says: the NSDMI is what keeps
  // -Wmissing-field-initializers quiet on the three-value `{tensor, offset,
  // count}` initialisations every contiguous caller writes.
  std::vector<int> indices{};  // NOLINT(readability-redundant-member-init)
};

/// One contiguous run of one output tensor.
///
/// `tensor` indexes the model's output tensors in declaration order, mirroring
/// `InputSegment::tensor`. Which .onnx tensor that declaration order actually
/// reaches is settled by NAME at load time (`rtc::TensorSpec::name`), so two
/// outputs of identical shape can no longer be swapped by an export that
/// reordered them — the failure `udp_hand_driver` shipped with (#511 B-1) and
/// the reason the schema layer makes tensor names mandatory.
struct OutputSlice {
  int tensor{0};
  int offset{0};
  int count{0};
};

/// One recurrent link: an output tensor whose contents become an input tensor
/// on the NEXT policy step (`h_out` → `h_in`).
///
/// Whole-tensor by decision D-2: a slice-level link would multiply the
/// validation surface for an export shape almost nobody produces.
///
/// Both indices address the DECLARATION order of their side, which is the
/// order `rtc::InferenceEngine` binds — so a link is a pair of buffer slots and
/// the copy never passes through the controller's own memory. That is why a
/// hidden state of any width costs this binding zero bytes.
struct RecurrentLink {
  int output_tensor{0};
  int input_tensor{0};
  std::size_t numel{0};
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
  const auto n = static_cast<std::size_t>(seg.count);
  if (src.size() < n) {
    return false;
  }
  if (!seg.indices.empty()) {
    // Scatter. Every index is checked BEFORE the first write, for the same
    // all-or-nothing reason as the contiguous branch.
    if (seg.indices.size() != n) {
      return false;
    }
    for (const int idx : seg.indices) {
      if (idx < 0 || static_cast<std::size_t>(idx) >= buf.size()) {
        return false;
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      buf[static_cast<std::size_t>(seg.indices[i])] = static_cast<float>(src[i]);
    }
    return true;
  }
  const auto off = static_cast<std::size_t>(seg.offset);
  if (off > buf.size() || n > buf.size() - off) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    buf[off + i] = static_cast<float>(src[i]);
  }
  return true;
}

/// Overwrite the whole tensor with `pattern` repeated (`buf[i] = pattern[i %
/// pattern.size()]`). An empty pattern is a no-op.
///
/// The filler for the elements a tensor's features do NOT cover, applied BEFORE
/// the features are packed each evaluation — so an uncovered slot always holds
/// the declared value rather than whatever the engine's allocation or a
/// previous tick left there. A pattern longer than one element is what makes
/// "identity quaternion in every unobserved row" expressible: `[0, 0, 0, 1]`
/// over a [29, 4] tensor. It is also the whole implementation of a constant
/// tensor, whose pattern is the tensor itself.
inline void ApplyFill(std::span<float> buf, std::span<const float> pattern) noexcept {
  if (pattern.empty()) {
    return;
  }
  const std::size_t m = pattern.size();
  for (std::size_t i = 0; i < buf.size(); ++i) {
    buf[i] = pattern[i % m];
  }
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

/// Copy `buf[slice.offset, +slice.count)` into `out`, widening float → double.
/// `size` is the element count of that output tensor's buffer.
///
/// Returns false and writes nothing when the slice leaves the tensor or `out`
/// is too small — same all-or-nothing reason as PackSegment. A null `buf` is
/// a false rather than a crash because that is exactly what the stub inference
/// engine returns, and the caller's hold path is the right response to it.
///
/// Finiteness is NOT checked here. The caller screens the unpacked doubles
/// (`rtc::compliance::AllFinite` over an Eigen::Map) before they reach a
/// command, and doing it in one place keeps the "which stage rejected this"
/// answer unambiguous.
[[nodiscard]] inline bool UnpackSlice(std::span<double> out, const OutputSlice& slice,
                                      const float* buf, std::size_t size) noexcept {
  if (buf == nullptr || slice.offset < 0 || slice.count < 0) {
    return false;
  }
  const auto off = static_cast<std::size_t>(slice.offset);
  const auto n = static_cast<std::size_t>(slice.count);
  if (off > size || n > size - off || out.size() < n) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = static_cast<double>(buf[off + i]);
  }
  return true;
}

/// Gather `out[i] = buf[indices[i]]`, widening float → double. `size` is the
/// element count of that output tensor's buffer.
///
/// The output half of by-name placement: a head whose elements are named in an
/// order the export chose (a hand command listed index → ring → thumb → middle)
/// is read into the DEVICE's joint order through indices resolved once at
/// configure. Same contract as UnpackSlice — all-or-nothing, a null buffer is a
/// false, and finiteness is the caller's screen.
[[nodiscard]] inline bool UnpackIndexed(std::span<double> out, std::span<const int> indices,
                                        const float* buf, std::size_t size) noexcept {
  if (buf == nullptr || out.size() < indices.size()) {
    return false;
  }
  for (const int idx : indices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= size) {
      return false;
    }
  }
  for (std::size_t i = 0; i < indices.size(); ++i) {
    out[i] = static_cast<double>(buf[static_cast<std::size_t>(indices[i])]);
  }
  return true;
}

/// Copy `src[0, dst.size())` into `dst`, refusing NON-FINITE values — and on
/// refusal ZEROING `dst` rather than leaving it as it was.
///
/// The recurrent feedback primitive, and the zeroing is the whole point.
/// Freezing a bad state would be permanent death: a NaN that reaches `h_out`
/// becomes the next step's `h_in`, every subsequent output is then NaN, every
/// tick fails its finiteness check and holds — and the hold path is silent, so
/// the robot stops with nothing in the log to say why and no recovery short of
/// re-activation. Zero is not the right state, but it is a state the policy can
/// be driven out of.
///
/// A size mismatch or a null source is the same refusal for the same reason:
/// whatever `dst` holds was written for a step that no longer applies. The
/// mismatch is tested as `!=` and not as "too short" — a LONGER source is not
/// a safe prefix to copy, it is a source that disagrees with `dst` about which
/// tensor this is, and truncating it would feed the policy a silently clipped
/// state. The link parser makes the two numels equal today (#511 D-2 links
/// whole tensors), so this guard is what keeps a slice-level link from
/// arriving as truncation instead of as a refusal.
///
/// RT: noexcept, allocation-free, single pass. Returns false when the state was
/// reset, so the caller can say so once rather than per tick.
[[nodiscard]] inline bool CopyFiniteChecked(std::span<float> dst, const float* src,
                                            std::size_t src_size) noexcept {
  if (src == nullptr || src_size != dst.size()) {
    for (float& v : dst) {
      v = 0.0F;
    }
    return false;
  }
  for (std::size_t i = 0; i < dst.size(); ++i) {
    if (!std::isfinite(src[i])) {
      for (float& v : dst) {
        v = 0.0F;
      }
      return false;
    }
  }
  for (std::size_t i = 0; i < dst.size(); ++i) {
    dst[i] = src[i];
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
