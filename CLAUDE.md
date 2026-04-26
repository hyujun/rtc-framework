# CLAUDE.md

## 1. Snapshot

**RTC (Real-Time Control) Framework** — Robot-agnostic real-time control for URDF-based manipulators.

- `rtc_*` packages (14): variable-DOF, 500 Hz–2 kHz, transport abstraction (UDP/CAN-FD/EtherCAT/RS485), lock-free SPSC, E-STOP
- `ur5e_*` packages (4): UR5e + 10-DOF hand drivers, demo controllers, BT coordinator (reference integration)
- `shape_estimation*` packages (2): ToF-based surface estimation (msgs + node)
- **Total: 20 ROS 2 packages**

| Item | Value |
|------|-------|
| Lang | C++20 (GCC 11+/13+), Python 3.10+ |
| OS | Ubuntu 22.04 (PREEMPT_RT optional) / 24.04 |
| Middleware | ROS 2 Humble / Jazzy, CycloneDDS |
| Build | CMake 3.22+, colcon, ament_cmake / ament_python |
| Deps | Eigen 3.4, Pinocchio, ProxSuite, ONNX Runtime |
| Optional | MuJoCo 3.x (sim), BehaviorTree.CPP v4 |
| Test | GTest, pytest — **1104 gtest cases across 10 packages** (최근 실측: 2026-04-24. 단일 출처: [agent_docs/testing-debug.md](agent_docs/testing-debug.md)) |

## 2. Harness Overview

이 저장소의 에이전트 가이드는 harness engineering 4구성요소로 조직되어 있다 (참고: Hashimoto 2026.02 / OpenAI *Harness engineering* 2026.02).

| 구성요소 | 목적 | 진입점 |
|---|---|---|
| **Guides** (feedforward) | 규칙·원칙·패턴 | §3 Invariants, §10 Style, [agent_docs/invariants.md](agent_docs/invariants.md), [agent_docs/design-principles.md](agent_docs/design-principles.md), [agent_docs/conventions.md](agent_docs/conventions.md) |
| **Sensors** (feedback) | 변경 검증 | §5 Sensors, [agent_docs/testing-debug.md](agent_docs/testing-debug.md), `[CONCERN]` 포맷 (§6) |
| **Orchestration** | Workflow | §4 Workflow Loop, [agent_docs/modification-guide.md](agent_docs/modification-guide.md) |
| **Escalation** | Human gate | §6 Escalation Triggers, [agent_docs/invariants.md](agent_docs/invariants.md) "수정 전 CONCERN" 규칙 |

**첫 방문 에이전트**: §3 → §4 → §6 순으로 읽고 작업 시작.
**수정 작업 중**: §5 검증 + §6 escalation 확인. Invariant 위반 의심 시 즉시 §6.

## 3. Invariants (요약)

전체: [agent_docs/invariants.md](agent_docs/invariants.md). 아래는 RT path에서 자주 위반되는 핵심만.

### RT path 절대 금지 (500 Hz 정기 tick)

1. `new` / `malloc` / `push_back` / `emplace_back` / `resize` — pre-allocated fixed-size 사용
2. `throw` / `catch` — error code, `std::optional`, `std::expected`
3. `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 — SPSC → aux thread defer. **예외**: one-shot init, `RCLCPP_*_THROTTLE` with RT-safe msg (단순 format + 기본 타입만; `fmt::format` / `to_string` / string concat 금지)
4. `std::mutex::lock` / `lock_guard` / `scoped_lock` — `try_lock`, `SeqLock`, SPSC, atomic
5. `auto` with Eigen expression — aliasing 버그; 명시 타입
6. Quaternion `lerp` / `nlerp` — `slerp` only
7. 기존 test assertion 수정 — 새 코드를 고쳐라

### Architecture / Process / Numerical

- `rtc_*` 패키지에 robot name / joint count / HW ID 하드코딩 금지 (ARCH-1)
- 의존성 그래프 상향 의존 금지 (ARCH-2, [agent_docs/architecture.md](agent_docs/architecture.md#L84))
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 특이점: damped pseudoinverse (NUM-1), `dt`·`trajectory_speed` zero guard (NUM-2, NUM-4)

위반 필요시 §6 Escalation의 `[CONCERN]` 포맷 보고.

## 4. Workflow Loop

모든 수정 작업은 이 순서. 단계 건너뛰기는 §6 escalation 사유.

```
1. Locate   → grep / Glob (known symbol) OR Explore agent (broad search)
              파일의 RT/aux/robot-specific 역할 판단
2. Read     → package.xml + CMakeLists.txt + target file + 인접 테스트
              §3 invariants 중 영향받는 것 확인
3. Edit     → minimal, single-concern. RT path 여부 재확인.
              auto/lerp/RT-forbidden 자체 grep
4. Build    → ./build.sh -p <pkg> (단일) 또는 ./build.sh full (rtc_base/rtc_msgs 변경 시)
5. Test     → §5 Sensor matrix 참조. 버그 수정 시 회귀 테스트 추가
6. Verify   → agent_docs/modification-guide.md Completion Checklist 8항목 통과
```

실패 시 절대 **"try harder"** 금지. 누락된 capability (test, lint, interface)를 엔지니어링하거나 §6 escalate.

상세: [agent_docs/modification-guide.md](agent_docs/modification-guide.md)

## 5. Sensors (변경 유형별 검증 매트릭스)

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream (PROC-3) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controllers/` RT path | `test_core_controllers` + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | `mpc_solve_timing.csv` 회귀 |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_solve_timing.csv` 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O) | `ros2 launch ur5e_bringup sim.launch.py` smoke |
| `ur5e_bringup/` demo FSM | demo_wbc FSM/integration/output + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `ur5e_hand_driver/` | 단위 gtest + UDP loopback | `ros2 topic hz /hand/joint_states` |
| BT 로직 | `ur5e_bt_coordinator` gtest (tree_validation, condition_nodes 등) | 실제 grasp 시나리오 smoke |
| Launch / YAML | `ros2 launch ... --print` + 짧은 smoke | config 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` gtest + RT perms | `check_rt_setup.sh --summary` |

상세 명령 + Live Debug Topics: [agent_docs/testing-debug.md](agent_docs/testing-debug.md)

## 6. Escalation Triggers

다음 상황에서 코드를 쓰기 **전에** `[CONCERN]` 보고 후 사용자 컨펌 대기.

| # | 상황 | Severity | 근거 |
|---|------|----------|------|
| E-1 | [agent_docs/invariants.md](agent_docs/invariants.md) 규칙을 건드려야 할 것 같음 | Critical | §3 |
| E-2 | `rtc_*` 패키지에 robot-specific 값을 넣어야 함 | Critical | ARCH-1 |
| E-3 | `rtc_msgs` / `shape_estimation_msgs` public ABI 변경 필요 | Critical | 전 downstream 영향 |
| E-4 | Abstract interface 없이 두 번째 구현 추가 필요 | Warning | ARCH-3 |
| E-5 | Optional dep (MuJoCo, aligator) fallback 제거 필요 | Warning | 배포 경로 깨짐 |
| E-6 | 기존 test assertion 수정 필요 | Critical | RT-7 |
| E-7 | Thread model (core 배치, priority) 변경 | Critical | timing 가정 |
| E-8 | E-STOP 경로 수정 | Critical | 안전 |
| E-9 | 문서-코드 불일치를 어느 쪽에 맞출지 결정 필요 | Warning | 사용자 의도 확인 |
| E-10 | [agent_docs/controller-safety-improvements.md](agent_docs/controller-safety-improvements.md) Remaining Phase 진행 | Warning | 계획 순서 |
| E-11 | [agent_docs/rtc_cm_lifecycle_plan.md](agent_docs/rtc_cm_lifecycle_plan.md) Phase 진행 | Warning | RT path / msg ABI 영향, 단계 순서 |

### `[CONCERN]` 포맷

```
[CONCERN] <한 줄 요약>
Severity: Critical | Warning | Info
Detail: <문제의 구체 내용, 영향 범위, 검토한 대안>
Alternative: <우회 안 1개 이상>
```

- **Critical**: 사용자 컨펌 전까지 커밋·PR 금지
- **Warning**: 사용자 판단에 따라 진행, 결정 로그 남김
- **Info**: 기록만, 진행 가능

## 7. Anti-patterns (Top 5)

전체: [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md).

1. **정기 tick에서 `RCLCPP_*` 직접 호출** — SPSC → aux defer 또는 THROTTLE+RT-safe msg (AP-RT-1)
2. **`auto` with Eigen expression** — aliasing 버그 (AP-RT-3)
3. **`rtc_*`에 UR5e 상수 하드코딩** — robot-agnostic 훼손 (AP-ARCH-1)
4. **"✅ complete" 후 실제 미완료** — grep 기반 전수 검증 누락 (AP-PROC-1)
5. **기존 test assertion 수정으로 통과시키기** — 회귀 은폐 (AP-PROC-4)

## 8. Where Things Live

| Package | Role |
|---------|------|
| `rtc_base` | Types, SeqLock, SPSC, threading, filters, DataLogger, CPU topology |
| `rtc_communication` | TransportInterface, UdpSocket, PacketCodec, Transceiver |
| `rtc_controller_interface` | RTControllerInterface base, ControllerRegistry, RTC_REGISTER_CONTROLLER |
| `rtc_controllers` | PController, JointPD, CLIK, OSC, GraspController |
| `rtc_controller_manager` | RtControllerNode: 500 Hz RT loop, SPSC publish, E-STOP |
| `rtc_inference` | InferenceEngine, OnnxEngine |
| `rtc_msgs` | ROS 2 message definitions |
| `rtc_mujoco_sim` | MuJoCo 3.x wrapper |
| `rtc_tsid` | TSID QP (WQP/HQP, tasks, constraints, ProxSuite) |
| `rtc_mpc` | MPC-RT: TripleBuffer, TrajectoryInterpolator, RiccatiFeedback, RobotModelHandler, HandlerMPCThread |
| `rtc_urdf_bridge` | URDF parser + Pinocchio model builder |
| `rtc_tools` | Python GUI / plotting tools |
| `rtc_scripts` | PREEMPT_RT, CPU shield, IRQ affinity, setup_env.sh |
| `rtc_digital_twin` | RViz2 JointState merge |
| `shape_estimation_msgs` | Shape estimation message/action definitions |
| `shape_estimation` | ToF-based surface estimation node |
| `ur5e_description` | URDF + MJCF + meshes |
| `ur5e_hand_driver` | UDP hand driver, ONNX F/T inference |
| `ur5e_bt_coordinator` | BehaviorTree.CPP v4 coordinator |
| `ur5e_bringup` | Launch files, demo controllers, config |

자세한 모듈 구조 / dependency graph: [agent_docs/architecture.md](agent_docs/architecture.md)

## 9. Common Commands

**Every new shell** (interactive `colcon test` / `ros2 launch`): `source rtc_scripts/scripts/setup_env.sh` — ROS 2 + `deps/install` + `.venv` + workspace overlay 순서대로 로드. `build.sh` / `install.sh` 는 자동 source.

```bash
source rtc_scripts/scripts/setup_env.sh   # PWD=src/rtc-framework 에서

./build.sh sim            # simulation packages
./build.sh robot          # real robot
./build.sh full           # all packages
./build.sh -p rtc_base    # single package

ros2 launch ur5e_bringup sim.launch.py
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10

colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

**Isolated deps** (상세: [rtc_scripts/README.md](rtc_scripts/README.md)): fmt 11.1.4 · mimalloc 2.1.7 · aligator 0.19.0 @ `<rtc_ws>/deps/install/` (`build_deps.sh` from `deps.repos`). Pinocchio · ProxSuite · hpp-fcl · eigenpy은 ROS Jazzy apt. Python은 `requirements.lock`.

## 10. Style Cheatsheet

상세: [agent_docs/conventions.md](agent_docs/conventions.md). 요약만:

- **Namespace**: `rtc`
- **Naming**: Google C++ — `snake_case_` members, `PascalCase` types, `kConstant`
- **Units**: SI (m, rad, s, kg, N) — degree는 API 경계에서만
- **Rotation**: quaternion (`Eigen::Quaterniond`, Hamilton) internal, ZYX Euler at boundaries
- **Variable naming**: paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `K_d` (stiffness)
- **RAII**, `noexcept` on RT, `[[nodiscard]]` on status returns
- **Lifecycle**: 5 C++ nodes are `rclcpp_lifecycle::LifecycleNode`, empty constructor, `on_configure` (Tier 1) + `on_activate` (Tier 2)
- **Commits**: Conventional Commits `type(scope): subject` (상세: [agent_docs/conventions.md](agent_docs/conventions.md#L64))

## 11. Post-Task Housekeeping

Commit 완료 또는 사용자가 task 종료를 알린 후:

1. **Memory save** — *surprising / non-obvious* 학습만 auto-memory 타입 규칙(user / feedback / project / reference)에 따라 저장. 코드 패턴, git 추출 가능한 사실, ephemeral state는 스킵
2. **Stale artifact 정리** — `~/.claude/plans/*.md` 완료 task용, repo-root / `/tmp` scratch files. 내용이 다른 곳(git log, `agent_docs/*.md`, `docs/*.md`)에 보존됨을 확인 후 삭제
3. **Memory prune** — 이제 틀리거나 outdated된 항목 정리
4. **보고** — 실제 수행한 항목만 한 줄씩

## 12. Reference Docs (read when relevant)

- [agent_docs/architecture.md](agent_docs/architecture.md) — Threading, data flow, core types, lock-free rules, lifecycle, E-STOP
- [agent_docs/controllers.md](agent_docs/controllers.md) — Controller table, gains layout, GraspController FSM, topics, config files
- [agent_docs/modification-guide.md](agent_docs/modification-guide.md) — Workflow loop, adding controllers/messages/devices/threads, package update checklist
- [agent_docs/design-principles.md](agent_docs/design-principles.md) — `rtc_*` 5 principles, `rtc_*` vs `ur5e_*` boundary rules
- [agent_docs/conventions.md](agent_docs/conventions.md) — Domain / code / commit conventions, documentation requirements
- [agent_docs/testing-debug.md](agent_docs/testing-debug.md) — Test table, sensor matrix, live debug topics, RT permissions
- [agent_docs/invariants.md](agent_docs/invariants.md) — RT / ARCH / PROC / NUM invariants (escalation triggers)
- [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — Recurring mistakes with detection + recovery
- [agent_docs/controller-safety-improvements.md](agent_docs/controller-safety-improvements.md) — Controller safety Phase plan (R/Q/L IDs)
- [agent_docs/rtc_cm_lifecycle_plan.md](agent_docs/rtc_cm_lifecycle_plan.md) — `/rtc_cm/...` srv + per-controller lifecycle 분리 Phase plan (D/F/P/M/OQ IDs)
