---
globs: ["rtc_controller_manager/**/*.cpp", "rtc_controller_manager/**/*.hpp", "rtc_controllers/**/*.cpp", "rtc_controllers/**/*.hpp", "ur5e_hand_driver/**/*.cpp", "ur5e_hand_driver/**/*.hpp"]
---

# RT Safety Rules (Scoped Stub)

이 파일은 `rtc_controller_manager` / `rtc_controllers` / `ur5e_hand_driver` 경로에서 자동 로드되는 scoped reminder다.

**전체 규칙**: [../../agent_docs/invariants.md](../../agent_docs/invariants.md)

RT path (500 Hz loop)에서 금지되는 패턴 요약:

1. Heap (`new` / `malloc` / `push_back` / `emplace_back` / `resize`) — RT-1
2. `throw` / `catch` — RT-2
3. `RCLCPP_*` 직접 호출 (one-shot init + RT-safe THROTTLE 제외) — RT-3
4. `std::mutex::lock` / `lock_guard` / `scoped_lock` — RT-4
5. `auto` with Eigen expression — RT-5
6. Quaternion `lerp` / `nlerp` — RT-6
7. 기존 test assertion 수정 — RT-7
8. `std::shared_ptr` 복사 — RT-8

상세 (탐지 grep, 복구 방법, RT-3 예외 규칙) + ARCH / PROC / NUM invariant은 [../../agent_docs/invariants.md](../../agent_docs/invariants.md) 참조. 재발성 실수 패턴은 [../../agent_docs/anti-patterns.md](../../agent_docs/anti-patterns.md).
