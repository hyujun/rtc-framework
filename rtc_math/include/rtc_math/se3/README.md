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
| `SplitBodyRot` | `[p_d−p ; log3(Rᵀ R_d)]` | base pos + **body** rot | θ | workspace legacy (`rtc::tsid::ComputeSe3Error`) |

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

- `test/test_se3_module.cpp` — 11 tests incl. Pinocchio cross-check (`log6`/`Jlog6`
  < 1e-10), finite-difference `exactPoseErrorRate` (all 6 types), `J(ξ)ξ=ξ`,
  scalar-gain exact exponential decay.
- `examples/se3_error_compare` (+ `scripts/plot_se3_compare.py`) — S1 straight-line
  vs screw, S2 Lee stall, S3 θ=179.999° robustness, S4 anisotropic-gain Jlog
  compensation, S5 transport-map omission.

## References
Murray–Li–Sastry 1994 · Lynch & Park, *Modern Robotics* 2017 §11.3 · Bullo &
Murray, *Automatica* 1999 · Caccavale et al., *IEEE T-RA* 1999 · Nakanishi et al.,
*IJRR* 2008 · Lee et al., *CDC* 2010 · Park, *ASME JMD* 1995 · Bhat & Bernstein,
*SCL* 2000 · Solà et al., arXiv:1812.01537.
