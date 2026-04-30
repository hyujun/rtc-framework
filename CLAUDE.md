# CLAUDE.md

## 1. Snapshot

**RTC (Real-Time Control) Framework** — Robot-agnostic real-time control for URDF-based manipulators.

- `rtc_*` packages (13): variable-DOF, 500 Hz–2 kHz, transport abstraction (UDP/CAN-FD/EtherCAT/RS485), lock-free SPSC, E-STOP
- `ur5e_*` packages (4): UR5e + 10-DOF hand drivers, demo controllers, BT coordinator (reference integration)
- `shape_estimation*` packages (2): ToF-based surface estimation (msgs + node)
- `repo_scripts` (1): repo-local convenience tooling (PREEMPT_RT, CPU shield, deps build, env activation) — not part of the rtc_* runtime libraries
- **Total: 20 ROS 2 packages**

| Item | Value |
|------|-------|
| Lang | C++20 (GCC 11+/13+), Python 3.10+ |
| OS | Ubuntu 22.04 (PREEMPT_RT optional) / 24.04 |
| Middleware | ROS 2 Humble / Jazzy, CycloneDDS |
| Build | CMake 3.22+, colcon, ament_cmake / ament_python |
| Deps | Eigen 3.4, Pinocchio, ProxSuite, ONNX Runtime |
| Optional | MuJoCo 3.x (sim), BehaviorTree.CPP v4 |
| Test | GTest, pytest — **1377 gtest cases across 13 packages + 281 pytest** (최근 실측: 2026-04-26. 단일 출처: [agent_docs/testing-debug.md](agent_docs/testing-debug.md)) |

## 2. Harness Overview

이 저장소의 에이전트 가이드는 harness engineering 5구성요소로 조직되어 있다 (참고: Hashimoto 2026.02 / OpenAI *Harness engineering* 2026.02 / Fowler *Harness engineering for coding agent users* 2026.04).

| 구성요소 | 목적 | 진입점 |
|---|---|---|
| **Guides** (feedforward) | 규칙·원칙·패턴 | §3 Invariants, §10 Style, [agent_docs/invariants.md](agent_docs/invariants.md), [agent_docs/design-principles.md](agent_docs/design-principles.md), [agent_docs/conventions.md](agent_docs/conventions.md) |
| **Sensors** (feedback, computational) | 변경 검증 (결정적·빠른) | §5 Sensors, [agent_docs/testing-debug.md](agent_docs/testing-debug.md), `[CONCERN]` 포맷 (§6) |
| **Sensors** (feedback, inferential) | 의미 검증 (LLM-as-judge, on-demand) | §5.5 Inferential Sensors |
| **Orchestration** | Workflow | §4 Workflow Loop, [agent_docs/modification-guide.md](agent_docs/modification-guide.md) |
| **Escalation** | Human gate | §6 Escalation Triggers, [agent_docs/invariants.md](agent_docs/invariants.md) "수정 전 CONCERN" 규칙, §6.5 Sprint Contract |
| **Enforcement** (자동) | 무인 실행/차단 | [.claude/hooks/format-code.sh](.claude/hooks/format-code.sh) (PostToolUse: clang-format / ruff), [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) (Stop: doc·CMake·build·test gate, exit 2 차단), [.claude/rules/rt-safety.md](.claude/rules/rt-safety.md) (RT path 작업 시 scoped 자동 inject) |

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
- 두 번째 구체 구현은 abstract interface / concept 정의 후에만 추가 (ARCH-3) — `#ifdef` / hardcoded switch 금지. 비슷한 기능(컨트롤러·transport·codec·task 등)이 추가될 때 기존 base를 재사용하거나, 부재 시 base를 먼저 만든다
- 새 utility 작성 전 `rtc_base` / `rtc_communication` / `rtc_inference` / `rtc_tsid` / `rtc_urdf_bridge`에 유사 기능 검색 — 맞지 않으면 fork 대신 일반화 ([agent_docs/design-principles.md](agent_docs/design-principles.md) P5)
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 특이점: damped pseudoinverse (NUM-1), `dt`·`trajectory_speed` zero guard (NUM-2, NUM-4)

위반 필요시 §6 Escalation의 `[CONCERN]` 포맷 보고.

## 4. Workflow Loop

모든 수정 작업은 이 순서. 단계 건너뛰기는 §6 escalation 사유.

```
0. Type     → "수정"인가 "추가(새 기능/컨트롤러/메시지/디바이스/스레드)"인가?
              추가라면 [agent_docs/design-principles.md](agent_docs/design-principles.md) 5원칙 +
              [agent_docs/modification-guide.md](agent_docs/modification-guide.md) "Adding a New ..." 절을 먼저 읽는다.
              · rtc_*에 추가 → P1·P2 (zero source edit, robot 상수 금지) +
                ARCH-3 (interface-first; 같은 종류 두 번째 구현이면 base부터)
              · ur5e_* / shape_estimation에 추가 → 재사용 가능한 부분이
                rtc_*에 존재하는지 / 일반화해 끌어올릴 수 있는지 먼저 검토
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

**※ 4·5·6은 [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) Stop hook이 turn 종료 시 자동 실행하고, 실패 시 `exit 2`로 다음 turn까지 차단한다. 사전 수동 실행은 빠른 피드백용. Hook은 변경 패키지만 빌드·테스트(60s timeout per pkg) + README/CMake co-update만 검사 — `package.xml`/YAML/Doxygen은 에이전트가 직접 검증.**

상세: [agent_docs/modification-guide.md](agent_docs/modification-guide.md)

## 5. Sensors (변경 유형별 검증 매트릭스)

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream (PROC-3) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controllers/` RT path | `test_core_controllers` + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | `cm_timing_log.csv` 회귀 |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_timing_log.csv` 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O) | `ros2 launch ur5e_bringup sim.launch.py` smoke |
| `ur5e_bringup/` demo FSM | demo_wbc FSM/integration/output + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `ur5e_hand_driver/` | 단위 gtest + UDP loopback | `ros2 topic hz /hand/joint_states` |
| BT 로직 | `ur5e_bt_coordinator` gtest (tree_validation, condition_nodes 등) | 실제 grasp 시나리오 smoke |
| Launch / YAML | `ros2 launch ... --print` + 짧은 smoke | config 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` gtest + RT perms | `check_rt_setup.sh --summary` |

상세 명령 + Live Debug Topics: [agent_docs/testing-debug.md](agent_docs/testing-debug.md)

## 5.5 Inferential Sensors (LLM-as-judge, 수동 trigger)

§5의 computational sensor (build / test / grep)는 **문법·빌드·기존 테스트 통과**만 검증한다. 의미 회귀 — 설계 일관성, robot-agnostic 위반, abstract interface 누락, 재사용 가능성 — 은 잡지 못한다 (Anthropic 2026.04 *Harness design*: 에이전트의 자기 평가는 신뢰 불가). 다음 상황에서 사용자에게 inferential sensor 실행을 권한다.

| Trigger | 권장 sensor | 근거 |
|---|---|---|
| `rtc_base` / `rtc_msgs` 변경 | `/review` | downstream 전 패키지 영향 (PROC-3) |
| Abstract interface 신설 / 두 번째 구현 추가 (ARCH-3 후보) | `/review` | 설계 적합성 — base 누락·#ifdef 유혹 검출 |
| `rtc_*`에 robot-specific 코드 추가 의심 (ARCH-1 borderline) | `/review` | robot-agnostic 검증 |
| E-STOP 경로 / safety publisher / lifecycle 콜백 수정 | `/security-review` | 안전성 검증 (E-8) |
| PR 준비 (다파일 / 다패키지 commit) | `/ultrareview <PR#>` | 다중 에이전트 클라우드 리뷰 |
| 100+줄 변경 또는 신규 패키지 디렉토리 | `/review` | 단일 에이전트 self-check 신뢰 한계 |

수동 trigger인 이유: inferential은 GPU/cost/지연이 크고 non-deterministic이므로 모든 변경에 자동 적용하면 ROI 음성. 위 trigger는 "false-negative 비용 > inferential 비용" 인 경우만 추렸다.

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
| E-10 | [agent_docs/archive/controller-safety-improvements.md](agent_docs/archive/controller-safety-improvements.md) Remaining Phase 진행 | Warning | 계획 순서 |

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

## 6.5 Sprint Contract (착수 전 성공 기준 협상)

근거: Anthropic 2026.04 *Harness design* — "에이전트 자기 평가는 신뢰 불가, 평가 기준은 generation 시작 *전* 명시화하라."

다음 task에서는 코드 수정 시작 *전* 1~3줄로 성공 기준을 사용자에게 제시하고 컨펌 받는다:

- 다단계 task (PR 단위 / 다파일 / 다패키지 / 신규 디렉토리 / phase로 쪼개진 작업)
- 신규 abstract interface · 새 controller / device group / thread / message 추가
- `rtc_base` / `rtc_msgs` 변경 (downstream broad impact)
- 리팩터 (기능 동등성 유지가 곧 success)

면제: 단일 파일 bug fix, 오타·포매팅, 단일 함수 추가, 사용자 의도가 1줄 메시지에서 자명한 경우.

### Sprint Contract 포맷

```
[SPRINT] <task 한 줄 요약>
Done when:
  - <검증 가능 기준 1>
  - <검증 가능 기준 2>
  - <...>
Out of scope: <명시적으로 하지 않을 것 — drift 방지>
```

기준은 **객관 검증 가능**해야 한다 (예: "test_X 통과", "rtc_*에 ur5e grep 0건", "rtc_cm 빌드 0 warning"). "코드가 깔끔하다", "잘 작동한다" 같은 주관 기준은 금지. 이 컨트랙트는 task 종료 시 §11 보고에서 항목별 충족 여부를 체크한다.

## 6.6 Long-running task — Context reset > Compaction

근거: Anthropic 2026.04 *Harness design* — long-running task에서 context compaction은 "context anxiety"(조기 마무리 / 디테일 손실)를 유발. **clean-slate reset + structured handoff artifact**가 더 안정적.

다음 신호가 보이면 사용자에게 reset을 제안한다:

- 동일 task 안에서 turn 50 이상, 또는 자동 compaction이 1회 이상 발생
- 에이전트가 직전 결정·근거를 다시 묻거나, "방금 한 작업이…"로 시작하는 hallucinated recall 발견
- §6.5 Sprint Contract 기준 중 일부가 "이미 했다"라고 잘못 보고됨

### Reset 절차

1. **Handoff artifact 작성**: `~/.claude/plans/<task-slug>.md`에 다음 박아 두기
   - Sprint Contract (§6.5 그대로 — Done when / Out of scope)
   - 지금까지 commit된 SHA + 한 줄 요약
   - 미완료 항목 (체크박스)
   - 다음 fresh session이 첫 turn에 읽어야 할 파일 경로 5~10개
   - 알려진 함정 / `[CONCERN]` 미해결 사항
2. **사용자에게 통지**: "context reset 권장. handoff는 `<path>`에 저장. 새 세션에서 `Read <path>`로 시작" 한 줄
3. **현 세션 종료**: 사용자 컨펌 후 `/clear` 또는 새 세션 시작은 사용자 결정

### 면제

- 단일 commit으로 끝나는 task — compaction 도달 전에 종료
- Sensor-driven debug task (build/test/log 반복) — handoff보다 직접 진행이 빠름
- 사용자가 명시적으로 "이대로 계속"을 요청한 경우

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
| `repo_scripts` | PREEMPT_RT, CPU shield, IRQ affinity, setup_env.sh |
| `rtc_digital_twin` | RViz2 JointState merge |
| `shape_estimation_msgs` | Shape estimation message/action definitions |
| `shape_estimation` | ToF-based surface estimation node |
| `ur5e_description` | URDF + MJCF + meshes |
| `ur5e_hand_driver` | UDP hand driver, ONNX F/T inference |
| `ur5e_bt_coordinator` | BehaviorTree.CPP v4 coordinator |
| `ur5e_bringup` | Launch files, demo controllers, config |

자세한 모듈 구조 / dependency graph: [agent_docs/architecture.md](agent_docs/architecture.md)

## 9. Common Commands

**Every new shell** (interactive `colcon test` / `ros2 launch`): `source repo_scripts/scripts/setup_env.sh` — ROS 2 + `deps/install` + `.venv` + workspace overlay 순서대로 로드. `build.sh` / `install.sh` 는 자동 source.

```bash
source repo_scripts/scripts/setup_env.sh   # PWD=src/rtc-framework 에서

./build.sh sim            # simulation packages
./build.sh robot          # real robot
./build.sh full           # all packages
./build.sh -p rtc_base    # single package

ros2 launch ur5e_bringup sim.launch.py
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10

colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

**Isolated deps** (상세: [repo_scripts/README.md](repo_scripts/README.md)): fmt 11.1.4 · mimalloc 2.1.7 · aligator 0.19.0 @ `<rtc_ws>/deps/install/` (`build_deps.sh` from `deps.repos`). Pinocchio · ProxSuite · hpp-fcl · eigenpy은 ROS Jazzy apt. Python은 `requirements.lock`.

## 10. Style Cheatsheet

상세: [agent_docs/conventions.md](agent_docs/conventions.md). 요약만:

- **Namespace**: `rtc`
- **Naming**: Google C++ — `snake_case_` members, `PascalCase` types, `kConstant`
- **Units**: SI (m, rad, s, kg, N) — degree는 API 경계에서만
- **Rotation**: quaternion (`Eigen::Quaterniond`, Hamilton) internal, ZYX Euler at boundaries
- **Variable naming**: paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `K_d` (stiffness)
- **RAII**, `noexcept` on RT, `[[nodiscard]]` on status returns
- **Lifecycle**: 5 C++ nodes are `rclcpp_lifecycle::LifecycleNode`, empty constructor, `on_configure` (Tier 1) + `on_activate` (Tier 2)
- **Logger naming** (3-tier, 상세: [agent_docs/conventions.md](agent_docs/conventions.md#L36)): node-owned = `<exec_name>` (예: `ur5e_rt_controller`) / library-level = `<full_package_name>` / controller-level = `<package>.<controller_key>` (예: `ur5e_bringup.demo_wbc_controller`). 점 `.` 1개만 허용.
- **Commits**: Conventional Commits `type(scope): subject` (상세: [agent_docs/conventions.md](agent_docs/conventions.md#L64))

## 11. Post-Task Housekeeping

Commit 완료 또는 사용자가 task 종료를 알린 후:

1. **Memory save** — *surprising / non-obvious* 학습만 auto-memory 타입 규칙(user / feedback / project / reference)에 따라 저장. 코드 패턴, git 추출 가능한 사실, ephemeral state는 스킵
2. **Stale artifact 정리** — `~/.claude/plans/*.md` 완료 task용, repo-root / `/tmp` scratch files. 내용이 다른 곳(git log, `agent_docs/*.md`, `docs/*.md`)에 보존됨을 확인 후 삭제
3. **Memory prune** — 이제 틀리거나 outdated된 항목 정리
4. **Harness pruning 신호 보고** (Anthropic 2026.04: 모델 capability 향상 시 harness 일부는 stripping 대상) — 이번 task에서 (a) invariant·anti-pattern grep이 false-positive로 발현 (정당한 사용을 막음), (b) hook이 정당한 변경을 잘못 차단, (c) 문서 규칙이 두 곳 이상에서 동일 내용 반복으로 drift 유발 — 중 하나라도 관찰됐다면 한 줄로 보고. 사용자가 향후 정리 task로 분리 결정
5. **보고** — 실제 수행한 항목만 한 줄씩

## 12. Reference Docs (read when relevant)

- [agent_docs/architecture.md](agent_docs/architecture.md) — Threading, data flow, core types, lock-free rules, lifecycle, E-STOP
- [agent_docs/controllers.md](agent_docs/controllers.md) — Controller table, gains layout, GraspController FSM, topics, config files
- [agent_docs/modification-guide.md](agent_docs/modification-guide.md) — Workflow loop, adding controllers/messages/devices/threads, package update checklist
- [agent_docs/design-principles.md](agent_docs/design-principles.md) — `rtc_*` 5 principles, `rtc_*` vs `ur5e_*` boundary rules
- [agent_docs/conventions.md](agent_docs/conventions.md) — Domain / code / commit conventions, documentation requirements
- [agent_docs/testing-debug.md](agent_docs/testing-debug.md) — Test table, sensor matrix, live debug topics, RT permissions
- [agent_docs/invariants.md](agent_docs/invariants.md) — RT / ARCH / PROC / NUM invariants (escalation triggers)
- [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — Recurring mistakes with detection + recovery
- [agent_docs/archive/controller-safety-improvements.md](agent_docs/archive/controller-safety-improvements.md) — Controller safety improvements (Phase 0~4 closed; Q-7 E-STOP ramp + Q-8 `contact_stop_release_eps` → YAML done 2026-04-26)
- [agent_docs/sim-shutdown-segv-analysis.md](agent_docs/sim-shutdown-segv-analysis.md) — **Status: Open** — 5s/15s hang fixed (`07ffc70`); `mpc_main` SEGV at exit unresolved. Resume from §3 "Remaining"
