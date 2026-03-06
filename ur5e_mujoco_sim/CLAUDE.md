# CLAUDE.md — ur5e_mujoco_sim

> **Note:** This package is part of the UR5e RT Controller workspace (v5.2.2). Please refer to the [Root CLAUDE.md](../CLAUDE.md) for full workspace context, building instructions, and architecture overview.
MuJoCo 3.x 물리 시뮬레이터 패키지. 실제 UR 드라이버 없이 `ur5e_rt_controller`와 연동 가능.
**선택적 의존성** — MuJoCo가 설치되지 않으면 CMake가 전체 패키지를 건너뜀.

---

## 파일 구조 및 역할

```
include/ur5e_mujoco_sim/
└── mujoco_simulator.hpp      ← MuJoCoSimulator 클래스 전체 (헤더에 인라인 API)

src/
├── mujoco_simulator.cpp      ← Initialize(), Start(), Stop(), SetCommand(), 외부 힘
├── mujoco_sim_loop.cpp       ← SimLoopFreeRun / SimLoopSyncStep, ReadSolverStats()
├── mujoco_viewer.cpp         ← ViewerLoop (GLFW, ~60Hz), 키보드/마우스 입력
└── mujoco_simulator_node.cpp ← ROS2 노드 래퍼

models/ur5e/
├── scene.xml                 ← 지면 + UR5e 포함
└── ur5e.xml                  ← UR5e MJCF 모델

config/
└── mujoco_simulator.yaml
```

---

## MuJoCoSimulator — 핵심 설계

### 스레딩 모델

```
mujoco_simulator_node (ROS2 스핀)
    │
    ├── SimLoop (jthread)
    │     └── SimLoopFreeRun / SimLoopSyncStep
    │         ├── ApplyCommand()      — cmd_mutex_ 하에 data_->ctrl 적용
    │         ├── PreparePhysicsStep() — atomic에서 solver 파라미터 읽어 model_->opt 갱신
    │         ├── mj_step()
    │         ├── ReadState()          — state_mutex_ 하에 positions/velocities/efforts 저장
    │         ├── ReadSolverStats()    — solver_stats_mutex_ 하에 통계 캡처
    │         ├── UpdateVizBuffer()    — viz_mutex_.try_lock() (절대 블로킹 안함)
    │         └── InvokeStateCallback() — 노드의 퍼블리시 콜백 호출
    │
    └── ViewerLoop (jthread, MUJOCO_HAVE_GLFW)
          └── GLFW 3D 렌더링 ~60Hz
              └── viz_mutex_.try_lock() 으로 qpos 스냅샷 읽기
```

### 동기화 원칙

| 뮤텍스/플래그 | 보호 대상 | 규칙 |
|--------------|----------|------|
| `cmd_mutex_` | `pending_cmd_` | SetCommand()와 ApplyCommand() |
| `cmd_pending_` (atomic) | FreeRun fast path | lock 없는 커맨드 존재 확인 |
| `sync_cv_` | SyncStep 대기 | 커맨드 도착 / Resume / Reset 시 notify |
| `state_mutex_` | `latest_positions_/velocities_/efforts_` | ReadState(W), GetPositions(R) |
| `viz_mutex_` | `viz_qpos_`, `viz_ncon_` | **try_lock 전용** — SimLoop는 절대 대기 안함 |
| `pert_mutex_` | `shared_pert_` | 뷰어 스레드 → sim 스레드 perturbation 전달 |
| `solver_stats_mutex_` | `latest_solver_stats_` | ReadSolverStats(W), GetSolverStats(R) |

### ReadSolverStats() — MuJoCo 3.x island 처리

```cpp
// solver_niter는 int* (constraint island별 배열, 크기=data_->nisland)
const int nisland = data_->nisland;
for (int k = 0; k < nisland; ++k) {
    s.iter += data_->solver_niter[k];  // 모든 island 합산
}
if (nisland > 0 && s.iter > 0) {
    s.improvement = data_->solver[0].improvement;  // 첫 번째 island 통계
}
```

UR5e 단독 씬은 `nisland==1`이므로 결과 동일. 다중 물체 씬에서는 올바르게 합산됨.

---

## 시뮬레이션 모드

### kFreeRun (기본값)

`mj_step()` → 퍼블리시 (decimation마다) → RTF throttle (max_rtf > 0 시).
커맨드는 비동기 수신 (`cmd_pending_` atomic 폴링).

### kSyncStep (지연 측정용)

퍼블리시 → `sync_cv_.wait()` (커맨드 대기, sync_timeout_ms) → `mj_step()`.
컨트롤러 Compute() 지연이 시뮬 스텝 주기에 1:1 반영됨.

---

## Config 구조체

```cpp
MuJoCoSimulator::Config cfg{
    .model_path        = "",             // 빈값 → 패키지 기본 scene.xml
    .mode              = SimMode::kFreeRun,
    .enable_viewer     = true,
    .publish_decimation = 1,             // N 스텝마다 StateCallback 호출
    .sync_timeout_ms   = 50.0,
    .max_rtf           = 0.0,            // 0 = 무제한
    .integrator_type   = mjINT_EULER,    // 강성 시스템은 mjINT_IMPLICIT 권장
    .solver_type       = mjSOL_NEWTON,
    .solver_iterations = 100,
    .solver_tolerance  = 1e-8,
};
```

---

## Runtime Controls (thread-safe, atomic)

모든 `Set*()` 메서드는 atomic에 저장. SimLoop가 다음 스텝 전 `PreparePhysicsStep()`에서 읽어 `model_->opt`에 적용:

```cpp
sim->SetIntegrator(mjINT_IMPLICIT);     // 다음 스텝부터 적용
sim->SetSolverType(mjSOL_NEWTON);
sim->SetSolverIterations(200);
sim->EnableGravity(false);
sim->SetContactEnabled(false);
sim->SetMaxRtf(5.0);
sim->SetExternalForce(body_id, {0,0,10, 0,0,0});  // world frame wrench
sim->ClearExternalForce();
sim->Pause() / sim->Resume();
sim->RequestReset();                    // initial_qpos 복원
```

---

## ROS2 노드 (mujoco_simulator_node)

**퍼블리시**:
- `/joint_states` (sensor_msgs/JointState): positions + velocities + efforts
- `/hand/joint_states` (Float64MultiArray): 시뮬레이션 손 상태 (100Hz, 1차 필터)
- `/sim/status` (Float64MultiArray): `[step_count, sim_time_sec, rtf, paused(0/1)]`

**구독**:
- `/forward_position_controller/commands` → `SetCommand()`
- `/hand/command` → 핸드 1차 필터 타겟 갱신

---

## 빌드

```bash
# MuJoCo binary tarball 사용 시 (lib/cmake/mujoco/ 없음)
colcon build --packages-select ur5e_mujoco_sim --symlink-install \
    --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x

# 환경변수 방식
export MUJOCO_DIR=/opt/mujoco-3.x.x
colcon build --packages-select ur5e_mujoco_sim
```

CMakeLists.txt는 2단계 탐지:
1. `find_package(mujoco QUIET)` — 소스 빌드 시
2. `find_library` + `find_path` 폴백 — binary tarball 시

---

## 이 패키지를 수정할 때 주의사항

1. **mujoco_sim_loop.cpp 수정** — SimLoop는 `model_/data_`의 유일한 쓰기 스레드.
   다른 스레드에서 직접 `data_->` 멤버를 읽거나 쓰면 안 된다. 항상 뮤텍스 경유.

2. **ViewerLoop의 `viz_mutex_`** — `try_lock()` 만 사용. `lock()` 호출 금지.
   ViewerLoop에서 블로킹하면 SimLoop RTF 저하.

3. **StateCallback** — SimLoop 컨텍스트에서 호출됨. 콜백 내에서 `state_mutex_` 중첩 잠금 금지.

4. **solver_niter 접근** — MuJoCo 3.x에서 `int*`임. 인덱스 접근 시 반드시 `nisland` 범위 확인.

5. **GLFW 의존성** — `MUJOCO_HAVE_GLFW` 정의 시에만 뷰어 컴파일. headless 환경에서는
   `enable_viewer: false` 설정 또는 `-DMUJOCO_HAVE_GLFW=OFF`.

6. **mujoco_simulator.yaml** — `custom_controller` 오버라이드 섹션 포함:
   FreeRun에서 `enable_estop: false` 필수. SyncStep에서도 timeout을 충분히 늘릴 것.
