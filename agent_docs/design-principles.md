# Design Principles for `rtc_*` Packages

`rtc_*` packages are the **robot-agnostic** backbone of this framework. Any modification must preserve this property. Robot-specific logic, hardware assumptions, and fixed-shape constants belong in `ur5e_*` packages. When in doubt: *"Would this code still make sense on a 7-DOF arm with a 2-finger gripper?"*

## Five Principles

1. **Extensibility** -- Adding a new robot, new DOF count, new transport, or new controller must require **zero source edits** inside `rtc_*`. Achievable via: (a) YAML config, (b) `RTC_REGISTER_CONTROLLER` from downstream, or (c) implementing an abstract interface.

2. **Generality** -- No robot names, joint counts, finger counts, topic names, or hardware identifiers hardcoded in `rtc_*`. Use YAML-injected values, template parameters, or runtime config. Constants like `kNumRobotJoints=6` are **upper-bound capacity**, not per-robot assumptions. Names describe the *role* (`num_joints`), never the *robot* (`ur5e_joints`).

3. **Modularity** -- Respect the dependency graph. Never introduce upward dependencies (e.g., `rtc_base` depending on `rtc_controllers`). Never cross-link siblings the graph doesn't connect. If a feature spans packages: (a) push abstraction down to a shared base, or (b) invert via interface injection.

4. **Interface-first** -- New functionality with multiple implementations MUST define an **abstract class, concept, or pure-virtual interface** before any concrete implementation. Follow: `RTControllerInterface`, `TransportInterface`/`PacketCodec`, `InferenceEngine`, `TaskBase`/`ConstraintBase`. Concrete classes register via factory/registry -- never `#ifdef` or hardcoded switches.

5. **Deduplication & Reuse** -- Before writing utilities, search existing `rtc_*`:
   - Lock-free, filters, logging, threading -> `rtc_base`
   - URDF, Pinocchio -> `rtc_urdf_bridge`
   - Transport, UDP, codecs -> `rtc_communication`
   - ONNX -> `rtc_inference`
   - QP tasks, constraints -> `rtc_tsid`
   If existing doesn't quite fit, **generalize it** -- don't fork.

## Boundary Rules (`rtc_*` vs `ur5e_*`)

| Belongs in `rtc_*` | Belongs in `ur5e_*` |
|--------------------|---------------------|
| Abstract interfaces, concepts, base classes | Concrete implementations via `RTC_REGISTER_CONTROLLER` |
| DOF-generic algorithms (variable `n_joints`) | Fixed-DOF launch files, URDF, MJCF, meshes |
| Transport/codec templates (`Transceiver<T,C>`) | Robot-specific packet structs as template args |
| YAML-driven parameter schemas | YAML files with actual robot values |
| Controller registry, TSID solver core | Demo controllers, BT coordinator, bringup |
| RT threading, SPSC, SeqLock, E-STOP logic | Hardware drivers (UR5e RTDE, hand UDP, ToF UART) |

## When Generalization Requires a Design Change

If you cannot satisfy all five principles with a local edit, STOP and:
1. Report a `[CONCERN] Severity: Warning`
2. Propose an interface refactor or dependency inversion as a separate task
3. Do NOT embed robot-specific logic in `rtc_*` "for now"
