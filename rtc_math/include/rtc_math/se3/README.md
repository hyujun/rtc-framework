# SE(3) Pose/Twist Error Module (`rtc::math::se3`)

Header-only, Eigen-only numeric core for SE(3) pose error and the **matching
velocity (twist) error**. Pinocchio is used only by the optional adapter and the
tests. Lives in `rtc_math` (the workspace's lowest Eigen-only geometric/control
math layer) and is exported as an INTERFACE target — any package that depends on
`rtc_math` gets the headers. The numeric core depends on Eigen only; the
`pinocchio_adapter.hpp` overloads are an optional dependency, compiled and
installed only when Pinocchio is found.

```cpp
#include "rtc_math/se3/pose_error.hpp"
#include "rtc_math/se3/velocity_error.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"  // optional: pinocchio::SE3/Motion overloads
namespace se3 = rtc::math::se3;
```

## Conventions

| Item | Choice |
|---|---|
| Twist/6D ordering | **`[linear(3); angular(3)]`** — identical to `pinocchio::Motion`. (Modern Robotics / Featherstone use `[angular, linear]` — do **not** mix.) |
| Pose type | `Eigen::Isometry3d` (`T`: world ← frame) in the core; `pinocchio::SE3` via the adapter |
| Rotation | Hamilton quaternion internal; `log3` = quaternion + `atan2` (θ∈[0,π], robust at θ≈0,π — `acos` is **not** used) |
| Units | SI (m, rad) |
| RT safety | all `noexcept`, fixed-size Eigen, no heap, no throw |

## ErrorType — definition · frame · rotation scale · source

For a pure rotation of angle θ, the rotation part has the listed magnitude.

| `ErrorType` | Pose error | Frame | Scale | Source |
|---|---|---|---|---|
| `SplitWorld` | `[p_d−p ; log3(R_d Rᵀ)]` | world | θ | Caccavale 1999, Siciliano 2009 |
| `BodyLog6` | `log6(T⁻¹ T_d)` | body screw | θ | MLS 1994, Modern Robotics §11.3 |
| `SpatialLog6` | `log6(T_d T⁻¹)` | spatial screw | θ | MLS 1994 (`= Ad_T · BodyLog6`) |
| `SplitLee` | `[p_d−p ; ½(R_dRᵀ−RR_dᵀ)∨]` | world | sinθ | Lee et al. 2010 |
| `SplitQuat` | `[p_d−p ; 2·sign(w)·vec(q_d⊗q⁻¹)]` | world | 2sin(θ/2) | Nakanishi 2008 |
| `SplitBodyRot` | `[p_d−p ; log3(Rᵀ R_d)]` | base pos + **body** rot | θ | workspace legacy (former `rtc::tsid::ComputeSe3Error`, since removed) |

`isBodyFrame(t)` is `true` only for `BodyLog6` (twist fully in the body frame).

## Velocity error ↔ Pinocchio frame consistency

The twist `ν` lives in the tangent space at `T`, `ν_d` at `T_d`; the naive `ν_d − ν`
is valid only near zero error. `computeVelocityError` applies the correct adjoint
transport per type (Bullo & Murray 1999). **Input-frame premise** (apply a
transport helper first if your twist is in another frame):

| `ErrorType` | input frame | `computeVelocityError` returns | Pinocchio `getFrameVelocity` |
|---|---|---|---|
| `SplitWorld` / `SplitLee` / `SplitQuat` | LOCAL_WORLD_ALIGNED | `ν_d − ν` (rotation part 1st order) | `LOCAL_WORLD_ALIGNED` |
| `BodyLog6` | LOCAL | `Ad_{T⁻¹T_d}·ν_d − ν` | `LOCAL` |
| `SpatialLog6` | WORLD (spatial) | `ν_d − Ad_{T_d T⁻¹}·ν` | `WORLD` |
| `SplitBodyRot` | LWA lin + LOCAL ang | `ν_d − ν` (1st order) | mixed |

Transport helpers: `twistWorldToLocal` / `twistLocalToWorld` (LWA↔LOCAL, **rotation
only** `blockdiag(R,R)`) vs `twistLocalToSpatial` / `twistSpatialToLocal`
(LOCAL↔WORLD, **full adjoint**). Confusing the two is a common bug.

`exactPoseErrorRate(T, T_d, ν, ν_d, type)` returns the **exact** `ė` (Jacobian
included) with `ν, ν_d` as LOCAL (body) twists — for anisotropic-gain
compensation and as the finite-difference test oracle.

### Key identity (why scalar gains "just work")

`ad_ξ ξ = 0` ⇒ every `J(ξ)` acts as identity along `ξ` (`Jlog(F)·log(F) = log(F)`).
So a **scalar** gain `k` with correct Ad transport already gives `ė = −k·e`
*exactly* at large error (`ν = Ad_F ν_d + k·e`, Modern Robotics §11.3). `Jlog`
corrections matter **only for anisotropic gain matrices**:
`e_ν^des channel ← (Jlog6)⁻¹·K·e`.

## Wrench transform (`wrench.hpp`)

A wrench is the **dual** of a twist and does **not** transform by the adjoint.
For `T = ᴬT_B` and a wrench `ᴮf` in frame B, `transformWrench(T, f)` returns

`ᴬf = Ad_{ᴬT_B}^{-T}·ᴮf = adjoint(T.inverse()).transpose()·ᴮf`

(computed via `Ad_T^{-1} = Ad_{T^{-1}}`, so **no** numeric 6×6 inverse). Ordering
is `[force(3); torque(3)]` to match the `[linear; angular]` twist order. The
inverse-transpose is what makes mechanical power `⟨ν, f⟩` frame-invariant — using
`Ad_T^{T}` instead is the classic wrench bug. The expanded block form is
`[[R, 0], [[p]×R, R]]` (moment picks up the lever arm `p×f`); direction is pinned
by the `WrenchTransform.PowerDuality` test, never by assuming a library's
`act`/`actInv` orientation.

## RPY → R at the wire edge (`so3.hpp`)

`RpyToRotationZyx(rpy)` returns `R = Rz(yaw)·Ry(pitch)·Rx(roll)` — intrinsic
Z-Y'-X'', the "ZYX Euler at boundaries" rule of `AGENTS.md` §10. Internal math
stays on quaternions and `log`/`exp`; this exists only where a message or an
operator still speaks `(x, y, z, r, p, y)`.

It takes one `Vec3`, not three scalars: three adjacent `double` parameters are
trivially transposed at a call site and a swapped roll/yaw yields a plausible
rotation rather than an obvious failure. Gimbal lock belongs to the **inverse**
map (`R → rpy`); this direction is total and well conditioned.

The reason it lives here rather than in each binding: the same three
`AngleAxisd` lines were written twice in the controller bindings with different
spellings — exactly the drift class this module exists to remove. Changing the
convention must be one edit. Consumer-side tests deliberately re-derive it
literally instead of calling this — a shared helper on both sides of an
assertion pins nothing.

## Approach-axis alignment (`axis_align.hpp`)

A 2-DoF orientation error for tasks that only constrain where one body axis
points (e.g. a palm normal facing an incoming object) and leave the roll about
that axis free. For unit axes `z` (current) and `a_d` (target):

| Function | Returns |
|---|---|
| `AxisAlignError(z, a_d, sin_eps)` | `e_a = θ·(z×a_d)/‖z×a_d‖`, `θ = atan2(‖z×a_d‖, zᵀa_d)`, so `exp([e_a]×) z = a_d` exactly and `‖e_a‖ = θ` |
| `AxisAlignJacobian(z, a_d, sin_eps, jacobian_sin_floor)` | `J_a` with `ė_a = J_a ω` (ω in the frame of `z`, `a_d` fixed) |
| `AxisAlignOmega(e_a, k_axis, w_max)` | `ω = k_axis·e_a` scaled to `‖ω‖ ≤ w_max`, with a `saturated` flag |

The rotation vector is used instead of `z × a_d` because `‖z × a_d‖ = sinθ`
collapses near 180°, so a reference built on it stalls there and jumps at any
antiparallel threshold. `‖e_a‖ = θ` is continuous and monotone.

Every call returns finite values and reports its branch in `AxisAlignRegion`:

| Region | Condition | `e_a` | `J_a` |
|---|---|---|---|
| `kAlignedDeadband` | `‖z×a_d‖ < sin_eps`, `c > 0` | 0 | 0 |
| `kAntiparallelDeadband` | `‖z×a_d‖ < sin_eps`, `c ≤ 0` | `π·u⊥` (fixed unit axis ⟂ `z`, deterministic in `z`) | 0 |
| `kJacobianCapped` | `c < 0`, `sin_eps ≤ ‖z×a_d‖ < jacobian_sin_floor` | exact | `f`, `f'` evaluated at the floor: `‖J_a‖ ≲ π/floor`, continuous at the floor |
| `kInvalidInput` | non-finite or non-unit input (`|‖v‖−1| > 1e-6`), bound out of range | 0 | 0 |

`J_a` is 0 in both deadbands because `e_a` is constant there; pass the same
`sin_eps` to the error and the Jacobian so their deadbands agree. The θ→π
divergence of `J_a` is a property of the problem (the rotation axis is
undefined), so it is capped rather than hidden. Defaults: `sin_eps` 1e-6,
`jacobian_sin_floor` 1e-3 (the cap starts only above ≈179.94°). The small-angle
series is used only for `c > 0`; `sinθ` also vanishes at θ = π, so a series
keyed on `sinθ` alone would replace the divergent antiparallel value with ≈2.645.
Derivation and background: `docs/dynamic_catching/L4_reference.md` §4.5.

## Choosing a definition

- **WBC / QP residual / impedance** with separately-tuned translation vs rotation
  gains → **`SplitWorld`** (no SE(3) bi-invariant metric exists, Park 1995).
- **Pinocchio-centric geometric control** → **`BodyLog6`** (`+ Jlog6` for
  anisotropic gains).
- **Large rotation** → prefer θ-linear scales (`SplitWorld` / `BodyLog6`); beware
  `SplitLee`'s θ→π stall (sinθ→0; see experiment S2).

## Limits

- Smooth error-based continuous feedback on SO(3) is at best almost-global
  (topological obstruction, Bhat & Bernstein 2000).
- This module is exact to **velocity** level. Acceleration-level exact dynamics of
  the `log` errors need an additional `J̇log` term (not provided).

## Validation & experiment

- `test/test_se3_module.cpp` — the Eigen-only core tests (exp/log identities,
  θ→0/π robustness, error scales, finite-difference `exactPoseErrorRate` for all
  6 types, `J(ξ)ξ=ξ`, scalar-gain exact exponential decay, wrench transform
  `Ad^{-T}` power duality, `RpyToRotationZyx` axis/order/properness) run with
  **no** external dependency. When Pinocchio is found, two extra cross-checks compile in
  (`log3`/`log6` and `Jlog3`/`Jlog6` < 1e-10), gated by `RTC_MATH_HAVE_PINOCCHIO`.
- `test/test_axis_align.cpp` — axis alignment: `exp([e_a]×)z = a_d` < 1e-12,
  1° grid `‖ω‖` continuity, `J_a` vs central difference < 1e-5 over 1–170°,
  finiteness in and around both deadbands, invalid inputs, zero Eigen heap
  allocation (`rtc_base`'s test-only `ScopedNoMalloc`, a test dependency only).
- `examples/se3_error_compare` (+ `scripts/plot_se3_compare.py`) — S1 straight-line
  vs screw, S2 Lee stall, S3 θ=179.999° robustness, S4 anisotropic-gain Jlog
  compensation, S5 transport-map omission.

## References
Murray–Li–Sastry 1994 · Lynch & Park, *Modern Robotics* 2017 §11.3 · Bullo &
Murray, *Automatica* 1999 · Caccavale et al., *IEEE T-RA* 1999 · Nakanishi et al.,
*IJRR* 2008 · Lee et al., *CDC* 2010 · Park, *ASME JMD* 1995 · Bhat & Bernstein,
*SCL* 2000 · Solà et al., arXiv:1812.01537.
