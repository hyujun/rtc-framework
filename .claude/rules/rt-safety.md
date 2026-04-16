---
globs: ["rtc_controller_manager/**/*.cpp", "rtc_controller_manager/**/*.hpp", "rtc_controllers/**/*.cpp", "rtc_controllers/**/*.hpp", "ur5e_hand_driver/**/*.cpp", "ur5e_hand_driver/**/*.hpp"]
---

# RT Safety Rules

These rules apply to RT-critical code paths (500Hz control loop).

## Forbidden on RT Path

| Pattern | Why | Use Instead |
|---------|-----|-------------|
| `new`/`malloc`/`push_back`/`emplace_back`/`resize` | Heap allocation breaks determinism | `std::array`, pre-allocated `Eigen::Matrix<fixed>` |
| `throw`/`catch` | `noexcept` violation = process kill | Error code, `std::optional` |
| `std::cout`/`std::cerr`/`RCLCPP_*` | Blocking I/O | SPSC queue -> logging thread |
| `std::mutex::lock` | Priority inversion | `try_lock`, SeqLock, SPSC |
| `std::shared_ptr` copy | Atomic ref-count contention | Raw ref or `std::shared_ptr` const-ref only |

## Required Patterns

- All functions on RT path must be `noexcept`
- Eigen: pre-allocated buffers, `noalias()`, never `auto` (expression template aliasing)
- Shared state: SeqLock (single-writer/multi-reader), SPSC (wait-free), or atomic
- Separate mutexes: `state_mutex_`, `target_mutex_`, `hand_mutex_` -- never hold more than one
- `jthread` + `stop_token` for cooperative cancellation

## E-STOP Triggers

- Device group timeout (50Hz check)
- Init timeout
- >= 10 consecutive RT overruns
- Sim sync timeout
- `TriggerGlobalEstop`: idempotent via `compare_exchange_strong`
