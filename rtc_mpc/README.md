# rtc_mpc

**MPC ↔ RT control interface layer** for the RTC framework.

Robot-agnostic library providing the plumbing between a soft-RT MPC thread
(20 Hz) and the hard-RT control loop (500 Hz): lock-free solution delivery,
cubic Hermite trajectory interpolation, Riccati feedback, and an MPC thread
skeleton. Concrete solver integrations (e.g. Aligator ProxDDP) live in
separate downstream packages and inherit from `rtc::mpc::MPCThread`.

## Module map

| Module | Header | Role |
|--------|--------|------|
| `types/` | `mpc_solution_types.hpp` | `MPCSolution`, `MPCStateSnapshot` (trivially copyable, fixed capacity) |
| `comm/` | `triple_buffer.hpp` | Lock-free triple buffer with zero-copy consumer acquire |
| `interpolation/` | `trajectory_interpolator.hpp` | Cubic Hermite interpolation between OCP nodes |
| `feedback/` | `riccati_feedback.hpp` | `u_fb = gain_scale · K · Δx` with optional accel-only mode |
| `manager/` | `mpc_solution_manager.hpp` | Facade combining TripleBuffer + Interpolator + Feedback + SeqLock |
| `thread/` | `mpc_thread.hpp` | `MPCThread` base (jthread + worker frame) and `MockMPCThread` |

## Dependency graph

```
rtc_mpc ← rtc_base (SeqLock, threading), Eigen3, yaml-cpp
```

`rtc_mpc` does **not** depend on `rtc_tsid`. Downstream controllers
(e.g. `ur5e_bringup::DemoWbcController`) inject MPC-generated references
into TSID tasks themselves.

## Design invariants

- **Robot-agnostic**: no fixed DoF, no robot names. Upper-bound capacity
  constants only.
- **Trivially copyable data**: `MPCSolution` and `MPCStateSnapshot`
  travel through `SeqLock` / `TripleBuffer`, so no dynamic members.
- **Zero-copy consumer**: `TripleBuffer::try_acquire_latest()` returns a
  `const T*` — no memcpy on the RT path.
- **Fixed-base only**: Cubic Hermite assumes Euclidean `q` (revolute
  joints). Floating-base quaternion interpolation is out of scope.

## Status

Step 1 — scaffolding. Library target is `INTERFACE` until `.cpp` sources
arrive in Steps 4-7. See `../CLAUDE.md` Phase 5 plan for the full roadmap.
