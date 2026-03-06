# CLAUDE.md — ur5e_mujoco_sim

> **Note:** This package is part of the UR5e RT Controller workspace (v5.2.2).
> Refer to the [Root CLAUDE.md](../CLAUDE.md) for full workspace context, building
> instructions, and architecture overview.

MuJoCo 3.x 물리 시뮬레이터 패키지. 실제 UR 드라이버 없이 `ur5e_rt_controller`와 동일한
토픽 인터페이스로 연동. **선택적 의존성** — MuJoCo 미설치 시 CMake가 패키지 전체를 스킵.

---

## 빠른 실행

```bash
# 기본 (free_run + GLFW 뷰어)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py

# sync_step: 컨트롤러 Compute() 지연 직접 측정
ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=sync_step

# Headless (SSH / CI)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

# 외부 MuJoCo Menagerie 모델
ros2 launch ur5e_rt_controller mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

# PD 게인 변경
ros2 launch ur5e_rt_controller mujoco_sim.launch.py kp:=10.0 kd:=1.0

# 목표 포즈 전송 (별도 터미널)
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
    "data: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]"

# 상태 모니터링
ros2 topic hz /joint_states                   # 게시 주파수
ros2 topic echo /sim/status                   # step_count, sim_time, rtf, paused
```

---

## 파일 구조

```
ur5e_mujoco_sim/
├── package.xml                               # v5.2.2, ament_cmake
├── CMakeLists.txt                            # 2-stage MuJoCo 탐지 → optional build
│
├── include/ur5e_mujoco_sim/
│   └── mujoco_simulator.hpp                  # MuJoCoSimulator 전체 공개 API
│
├── src/
│   ├── mujoco_simulator.cpp                  # 생명주기 (ctor/dtor, Initialize, Start, Stop)
│   │                                         # + SetCommand / GetState / External forces
│   ├── mujoco_sim_loop.cpp                   # 물리 헬퍼 + SimLoopFreeRun / SimLoopSyncStep
│   ├── mujoco_viewer.cpp                     # ViewerLoop: GLFW 렌더링, 키보드/마우스 콜백
│   └── mujoco_simulator_node.cpp             # ROS2 노드 래퍼 (MuJoCoSimulatorNode)
│
│
├── config/
│   └── mujoco_simulator.yaml                 # 시뮬레이터 파라미터 + controller 오버라이드
│
└── launch/
    └── mujoco_sim.launch.py                  # 3 노드 런치 (sim, controller, monitor)
```

---

## 소스 구조 상세

### `mujoco_simulator.hpp` — 공개 API

`MuJoCoSimulator` 클래스 전체 선언. inline 메서드(atomic getter/setter)도 여기 정의.

**핵심 타입:**
```cpp
namespace ur5e_rt_controller {

enum class SimMode { kFreeRun, kSyncStep };

struct Config {
    std::string model_path;
    SimMode     mode{kFreeRun};
    bool        enable_viewer{true};
    int         publish_decimation{1};    // kFreeRun: N 스텝마다 콜백
    double      sync_timeout_ms{50.0};    // kSyncStep: 명령 대기 타임아웃
    double      max_rtf{0.0};            // 0 = 무제한
    std::array<double,6> initial_qpos{0, -1.5708, 1.5708, -1.5708, -1.5708, 0};

    // Physics solver
    int    integrator_type{mjINT_EULER};  // or RK4 / IMPLICIT / IMPLICITFAST
    int    solver_type{mjSOL_NEWTON};     // or PGS / CG
    int    solver_iterations{100};
    double solver_tolerance{1e-8};        // 0 = 비활성
};

struct SolverStats {
    double improvement{0.0};  // 마지막 반복 constraint violation 개선량
    double gradient{0.0};     // 수렴 시 gradient norm
    int    iter{0};           // 실제 사용된 solver 반복 횟수 (모든 island 합산)
    int    ncon{0};           // 활성 contact 수
};

// 상태 콜백: SimLoop에서 제어 주파수마다 호출
using StateCallback = std::function<void(
    const std::array<double,6>& positions,
    const std::array<double,6>& velocities,
    const std::array<double,6>& efforts)>;  // qfrc_actuator (Nm)
```

**관절 이름 매핑** (`kMjJointNames[]`):
```cpp
static constexpr std::array<const char*, 6> kMjJointNames = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",        "wrist_3_joint",
};
```
`ResolveJointIndices()` 가 `Initialize()` 시점에 MJCF 모델에서 qpos/qvel 인덱스를 확인.
관절을 찾지 못하면 `fprintf(stderr)` 후 static index(0-5) 사용.

---

### `mujoco_simulator.cpp` — 생명주기 & I/O

| 메서드 | 설명 |
|--------|------|
| `MuJoCoSimulator(Config)` | `current_max_rtf_` 초기화, `mjv_defaultPerturb()` |
| `~MuJoCoSimulator()` | `Stop()` → `mj_deleteData` → `mj_deleteModel` |
| `Initialize()` | `mj_loadXML` → `mj_makeData` → solver 파라미터 동기화 → `ResolveJointIndices()` → `mj_forward()` |
| `Start()` | `running_.exchange(true)` → `sim_thread_` (jthread) + 필요 시 `viewer_thread_` |
| `Stop()` | `running_=false` → `sync_cv_.notify_all()` → threads join |
| `SetCommand(cmd)` | `cmd_mutex_` 하에 `pending_cmd_` 저장 → `cmd_pending_.store(true)` → kSyncStep 시 `sync_cv_.notify_one()` |
| `SetExternalForce(body_id, wrench)` | `pert_mutex_` 하에 `ext_xfrc_` 수정, `ext_xfrc_dirty_=true` |
| `ClearExternalForce()` | `ext_xfrc_` 를 0으로 채움, `ext_xfrc_dirty_=false` |
| `UpdatePerturb(pert)` | 뷰어 스레드 → sim 스레드로 mjvPerturb 전달 |

---

### `mujoco_sim_loop.cpp` — 물리 루프

#### 내부 헬퍼 함수

| 함수 | 역할 |
|------|------|
| `ApplyCommand()` | `cmd_mutex_` 하에 `pending_cmd_` → `data_->ctrl[0..nu-1]` |
| `ReadState()` | `state_mutex_` 하에 qpos/qvel/qfrc_actuator → `latest_*` |
| `ReadSolverStats()` | `solver_stats_mutex_` 하에 MuJoCo 3.x island 처리 |
| `InvokeStateCallback()` | `state_mutex_` 하에 포지션 스냅샷 → `state_cb_(pos,vel,eff)` |
| `UpdateVizBuffer()` | **`viz_mutex_.try_lock()` 전용** — qpos → `viz_qpos_`, `viz_ncon_` |
| `UpdateRtf(step)` | 200 스텝마다 wall/sim 시간 비율 계산 → `rtf_` atomic |
| `ThrottleIfNeeded()` | max_rtf 변경 감지 → `this_thread::sleep_for` |
| `PreparePhysicsStep()` | atomic → `model_->opt` (solver/integrator/gravity/contact) + external force/perturb 적용 |
| `ClearContactForces()` | perturb/ext_xfrc 비활성 시 `xfrc_applied` 초기화 |
| `HandleReset()` | `mj_resetData()` → initial_qpos 복원 → pert/ext_xfrc 초기화 → 카운터 리셋 |

#### `PreparePhysicsStep()` — 실행 순서

```
1. model_->opt.integrator  ← solver_integrator_ atomic
2. model_->opt.solver      ← solver_type_ atomic
3. model_->opt.iterations  ← solver_iterations_ atomic
4. model_->opt.tolerance   ← solver_tolerance_ atomic
5. mjDSBL_CONTACT 플래그   ← contacts_enabled_ atomic
6. model_->opt.gravity[2]  ← gravity_enabled_ atomic (0.0 ↔ original_gravity_z_)
7. pert_mutex_.try_lock()  → ext_xfrc_→ xfrc_applied + mjv_applyPerturbForce()
```

#### `ReadSolverStats()` — MuJoCo 3.x island 처리

```cpp
// solver_niter는 int* (island별 배열, 크기 = data_->nisland)
const int nisland = data_->nisland;
for (int k = 0; k < nisland; ++k) {
    s.iter += data_->solver_niter[k];   // 전체 island 합산
}
if (nisland > 0 && s.iter > 0) {
    s.improvement = data_->solver[0].improvement;  // 첫 번째 island
}
```
UR5e 단독 씬은 `nisland==1`. 다중 물체 씬에서도 올바르게 합산.

#### `SimLoopFreeRun` 실행 순서 (per step)

```
Pause check → Reset check → ApplyCommand (lock-free) →
PreparePhysicsStep() → mj_step() → ClearContactForces() →
ReadSolverStats() → step_count_/sim_time_sec_ 갱신 →
decimation마다: ReadState() + InvokeStateCallback() →
UpdateRtf() → ThrottleIfNeeded() →
8스텝마다+viewer ON: UpdateVizBuffer()
```

#### `SimLoopSyncStep` 실행 순서 (per step)

```
Pause check → Reset check →
ReadState() + InvokeStateCallback() →           [1. 현재 상태 퍼블리시]
sync_cv_.wait_for(timeout, cmd_pending_) →      [2. 명령 대기]
ApplyCommand() → PreparePhysicsStep() →
mj_step() → ClearContactForces() →
ReadSolverStats() → step_count_/sim_time_sec_ 갱신 →
UpdateRtf() → ThrottleIfNeeded() →
8스텝마다+viewer ON: UpdateVizBuffer()
```

---

### `mujoco_viewer.cpp` — GLFW 뷰어 (`MUJOCO_HAVE_GLFW`)

`ViewerLoop(stop_token)` 메서드. `MUJOCO_HAVE_GLFW` 미정의 시 즉시 반환.

#### 초기화
- 창 크기: **1280×960**, MSAA 4x, OpenGL 3.3 Compat Profile
- 카메라: `distance=2.5, azimuth=90°, elevation=-20°`
- `vis_data`: 뷰어 전용 mjData (sim 물리 데이터와 별개)
- RTF 프로파일러: `mjvFigure` (200 샘플 rolling buffer, ~3.3s 윈도우)

#### GLFW 콜백 구조

콜백은 캡처리스 람다 → `glfwGetWindowUserPointer()`로 `ViewerState*` 획득.

```cpp
struct ViewerState {
    mjvCamera*, mjvOption*, mjvScene*, model*, vis_data*, sim*;
    bool btn_left, btn_right, ctrl_held;
    double lastx, lasty;
    mjvPerturb pert;                    // 뷰어 로컬 — UpdatePerturb()로 sim에 전달
    bool show_profiler, show_help, show_solver;
    float rtf_history[200];             // RTF rolling buffer
    int rtf_head, rtf_count;
};
```

#### 키보드 단축키 전체 목록

| 키 | 동작 |
|----|------|
| **F1** | help 오버레이 토글 |
| **Space** | Pause / Resume |
| **+** / KP_ADD | max_rtf 2배 (unlimited → 2x) |
| **-** / KP_SUB | max_rtf 절반 (≤0.5x → unlimited) |
| **R** | `RequestReset()` — initial_qpos로 복원 |
| **G** | 중력 토글 |
| **N** | contact constraints 토글 |
| **I** | integrator 순환: Euler → RK4 → Implicit → ImplicitFast |
| **S** | solver 순환: PGS → CG → Newton |
| **]** | solver 반복 횟수 2배 (최대 1000) |
| **[** | solver 반복 횟수 절반 (최소 1) |
| **C** | contact point 마커 토글 |
| **F** | contact force 화살표 토글 |
| **V** | collision geometry(group 0) 토글 |
| **T** | 투명도 토글 |
| **F3** | RTF 프로파일러 그래프 토글 (우하단) |
| **F4** | solver stats 오버레이 토글 (좌하단) |
| **Backspace** | `mjv_defaultOption()` — 시각화 옵션 초기화 |
| **Escape** | 카메라 default 위치 복원 |

#### 마우스 컨트롤

| 동작 | 효과 |
|------|------|
| Left drag | orbit (카메라 회전) |
| Right drag | pan (카메라 이동) |
| Scroll | zoom |
| **Ctrl + Left click** | body 선택 (perturbation 대상) |
| **Ctrl + Left drag** | 선택 body에 spring force 적용 → `UpdatePerturb()` |
| Left button release | `ClearPerturb()` |

#### 뷰어 오버레이 구성

| 위치 | 내용 |
|------|------|
| **우상단** | Mode, RTF, Limit, Sim Time, Steps, Contacts, Gravity, Status, Integrator, Solver, Iterations, Residual |
| **좌하단** | `F1: toggle help` (항상 표시) |
| **좌상단** | F1 토글 시 전체 help (현재 상태값 포함) |
| **좌하단 (F4)** | solver stats: Integrator, Solver, Max/Used iter, Improvement, Gradient, Contacts, Timestep |
| **우하단 (F3)** | RTF profiler 그래프 (화면 1/3 크기) |

#### 렌더 루프 주요 흐름

```
viz_mutex_ lock → viz_qpos_ 복사 → viz_mutex_ unlock →
mj_forward(vis_data) →                      [시각화 kinematics 갱신]
RTF rolling buffer 갱신 →
mjv_updateScene(model_, vis_data, opt, pert, cam) →
mjr_render() →
오버레이(status, help, solver, profiler) →
glfwSwapBuffers / glfwPollEvents →
sleep_for(16ms)                              [~60Hz]
```

---

### `mujoco_simulator_node.cpp` — ROS2 노드

#### 클래스 구조

```cpp
class MuJoCoSimulatorNode : public rclcpp::Node {
    void DeclareAndLoadParameters();  // 파라미터 선언 + 모델 경로 해석
    void CreateSimulator();           // Config 조합 → MuJoCoSimulator 생성 + Initialize()
    void CreatePublishers();          // 3개 publisher 생성
    void CreateSubscriptions();       // 2개 subscriber 생성
    void CreateTimers();              // hand(100Hz) + status(1Hz) 타이머

    void CommandCallback(msg)         // → sim_->SetCommand(cmd)
    void HandCommandCallback(msg)     // → hand_mutex_ 하에 hand_target_ 갱신
    void PublishJointState(pos,vel,eff);  // SimLoop 컨텍스트에서 호출됨
    void PublishHandState();          // 100Hz: 1차 LPF → /hand/joint_states
    void PublishSimStatus();          // 1Hz: step/time/rtf/paused → /sim/status

    std::unique_ptr<MuJoCoSimulator> sim_;
    mutable std::mutex hand_mutex_;
    std::array<double,11> hand_state_{};   // 현재 필터 상태
    std::array<double,11> hand_target_{};  // /hand/command 수신값
};
```

#### 모델 경로 해석 로직

```cpp
if (model_path_.empty() || model_path_[0] != '/') {
    // ur5e_description 패키지의 기본 경로 사용
    model_path_ = share_dir + "/" + (model_path_.empty() ? "robots/ur5e/mjcf/scene.xml" : model_path_);
}
// 절대경로 → 그대로 사용
```

#### 핸드 1차 LPF (100Hz)

```
hand_state_[i] += hand_filter_alpha_ * (hand_target_[i] - hand_state_[i])
```
`hand_filter_alpha=0.1`: τ ≈ 90ms / `0.5`: τ ≈ 18ms.

#### 토픽 인터페이스

| 토픽 | 타입 | 방향 | 내용 |
|------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | Publish | positions + velocities + efforts (qfrc_actuator) |
| `/hand/joint_states` | `Float64MultiArray` | Publish | 11 DOF 시뮬 핸드 상태 (100Hz, LPF) |
| `/sim/status` | `Float64MultiArray` | Publish | `[step_count, sim_time_sec, rtf, paused(0/1)]` @ 1Hz |
| `/forward_position_controller/commands` | `Float64MultiArray` | Subscribe | → `SetCommand()` |
| `/hand/command` | `Float64MultiArray` | Subscribe | → `hand_target_` 갱신 |

> **주의**: `/hand/joint_states` 는 `ur5e_hand_udp`와 토픽 이름 충돌. 동시 실행 금지.

---

## 동기화 구조 전체

| 동기화 프리미티브 | 보호 대상 | 규칙 |
|------------------|----------|------|
| `cmd_mutex_` | `pending_cmd_` | `SetCommand()` ↔ `ApplyCommand()` |
| `cmd_pending_` (atomic) | FreeRun fast path | lock-free 커맨드 확인 |
| `sync_cv_` + `sync_mutex_` | SyncStep 대기 | 커맨드/Resume/Reset 시 notify |
| `state_mutex_` | `latest_positions_/velocities_/efforts_` | `ReadState(W)`, `GetPositions(R)` |
| `viz_mutex_` | `viz_qpos_`, `viz_ncon_` | **try_lock 전용** — SimLoop는 절대 블로킹 안함 |
| `pert_mutex_` | `shared_pert_`, `ext_xfrc_` | 뷰어↔sim 섭동 전달, external force |
| `solver_stats_mutex_` | `latest_solver_stats_` | `ReadSolverStats(W)`, `GetSolverStats(R)` |
| `hand_mutex_` (노드) | `hand_state_`, `hand_target_` | `HandCommandCallback(W)`, `PublishHandState(RW)` |

### 스레딩 다이어그램

```
mujoco_simulator_node (rclcpp::spin)
    ├── CommandCallback()           → sim_->SetCommand()
    ├── HandCommandCallback()       → hand_target_ (hand_mutex_)
    ├── hand_pub_timer_ (100Hz)    → PublishHandState()
    └── status_timer_  (1Hz)       → PublishSimStatus()

sim_thread_ (jthread — SimLoop)
    ├── PreparePhysicsStep()       → model_->opt 갱신 (atomics)
    ├── mj_step(model_, data_)     ← 유일한 data_ 쓰기 스레드
    ├── ReadState()                → state_mutex_ (write)
    ├── ReadSolverStats()          → solver_stats_mutex_ (write)
    ├── UpdateVizBuffer()          → viz_mutex_.try_lock() (write)
    └── InvokeStateCallback()      → PublishJointState() (node 콜백)

viewer_thread_ (jthread — ViewerLoop, MUJOCO_HAVE_GLFW)
    ├── viz_mutex_ lock            → viz_qpos_ 복사
    ├── mj_forward(vis_data)       ← vis_data 전용 (data_ 미접근)
    ├── GLFW events / keyboard     → sim_->Set*() 호출
    └── UpdatePerturb()            → pert_mutex_ (write)
```

---

## 물리 솔버 Runtime Controls

모든 `Set*()` 는 atomic 저장. SimLoop가 다음 `PreparePhysicsStep()`에서 읽어 `model_->opt` 적용.

```cpp
// Integrator (기본: mjINT_EULER)
// 강성 시스템(강한 접촉)은 mjINT_IMPLICIT 권장
sim->SetIntegrator(mjINT_EULER);       // 0: Euler
sim->SetIntegrator(mjINT_RK4);         // 1: Runge-Kutta 4
sim->SetIntegrator(mjINT_IMPLICIT);    // 2: Implicit (강성 안정)
sim->SetIntegrator(mjINT_IMPLICITFAST);// 3: ImplicitFast

// Solver (기본: mjSOL_NEWTON)
sim->SetSolverType(mjSOL_PGS);    // 0: Projected Gauss-Seidel
sim->SetSolverType(mjSOL_CG);     // 1: Conjugate Gradient
sim->SetSolverType(mjSOL_NEWTON); // 2: Newton (정밀, 기본)
sim->SetSolverIterations(200);    // 최대 반복 (1~1000 클램프)
sim->SetSolverTolerance(1e-8);    // 수렴 허용오차 (0 = 비활성)

// Contact
sim->SetContactEnabled(false);    // mjDSBL_CONTACT 플래그 설정

// Physics
sim->EnableGravity(false);        // gravity[2] = 0 으로 설정
sim->SetMaxRtf(5.0);              // 실시간 5x 제한 (0 = 무제한)

// External force [Fx, Fy, Fz, Tx, Ty, Tz] world frame (SI)
sim->SetExternalForce(body_id, {0, 0, 10, 0, 0, 0});
sim->ClearExternalForce();

// Simulation lifecycle
sim->Pause();         // physics 일시정지 (viewer는 계속 렌더)
sim->Resume();        // 재개 + sync_cv_ notify
sim->RequestReset();  // initial_qpos 복원 (다음 루프 iteration에서 실행)
```

---

## 시뮬레이션 모드 비교

| 항목 | `kFreeRun` | `kSyncStep` |
|------|-----------|-------------|
| 물리 진행 | 최대 속도 (max_rtf 제한 가능) | 1 컨트롤러 명령 = 1 스텝 |
| 명령 수신 | 비동기 (`cmd_pending_` polling) | `sync_cv_.wait_for(timeout)` |
| RTF | 수백~수천배 가능 | 컨트롤러 Compute() 시간에 의존 |
| E-STOP | **반드시 disable** (`enable_estop: false`) | timeout 충분히 설정 필요 |
| 용도 | 알고리즘 검증, 궤적 생성 | Compute() 실행시간 직접 측정 |
| decimation | O (`publish_decimation` 파라미터) | 불필요 (매 스텝 퍼블리시) |

---

## Config 구조체 전체

```cpp
MuJoCoSimulator::Config cfg{
    .model_path          = "",              // 빈값 → ur5e_description 기본 scene.xml
    .mode                = SimMode::kFreeRun,
    .enable_viewer       = true,
    .publish_decimation  = 1,              // kFreeRun: N 스텝마다 StateCallback
    .sync_timeout_ms     = 50.0,           // kSyncStep: 명령 대기 최대 시간 (ms)
    .max_rtf             = 0.0,            // 0 = 무제한
    .initial_qpos        = {0, -1.5708, 1.5708, -1.5708, -1.5708, 0},
    .integrator_type     = mjINT_EULER,    // 강성: mjINT_IMPLICIT 권장
    .solver_type         = mjSOL_NEWTON,
    .solver_iterations   = 100,
    .solver_tolerance    = 1e-8,
};
```

---

## YAML 파라미터 레퍼런스 (`mujoco_simulator.yaml`)

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: ""                         # 절대경로 or 빈값(패키지 기본)
    sim_mode: "free_run"                   # "free_run" | "sync_step"
    publish_decimation: 1                  # free_run 전용
    sync_timeout_ms: 50.0                  # sync_step 전용
    max_rtf: 0.0                           # 0 = 무제한
    enable_viewer: true                    # false = headless
    initial_joint_positions: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    enable_hand_sim: true                  # /hand/joint_states LPF 활성화
    hand_filter_alpha: 0.1                 # LPF 계수 (0.1 → τ ≈ 90ms)

# custom_controller 오버라이드 (mujoco_sim.launch.py 적용)
custom_controller:
  ros__parameters:
    control_rate: 500.0
    kp: 5.0
    kd: 0.5
    enable_logging: true
    enable_estop: false                    # free_run에서 필수
    robot_timeout_ms: 10000.0             # 시뮬 연산 여유
    hand_timeout_ms:  10000.0
```

---

## Launch 파일 (`mujoco_sim.launch.py`)

론치되는 3개 노드:

| 노드 | 패키지 | 파라미터 로드 순서 |
|------|--------|-----------------|
| `mujoco_simulator` | `ur5e_mujoco_sim` | `mujoco_simulator.yaml` → 인라인 overrides |
| `custom_controller` | `ur5e_rt_controller` | `ur5e_rt_controller.yaml` → `mujoco_simulator.yaml`(오버라이드) → `kp/kd` |
| `data_health_monitor` | `ur5e_tools` | `check_rate=10, timeout_threshold=1.0` |

**로그 디렉토리**: `~/ur_ws/logging_data/` (하드코딩)

**Launch 인수 전체:**

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `model_path` | `""` | 절대경로 또는 빈값 |
| `sim_mode` | `free_run` | `free_run` \| `sync_step` |
| `enable_viewer` | `true` | GLFW 뷰어 여부 |
| `publish_decimation` | `1` | free_run 전용 N 스텝 decimation |
| `sync_timeout_ms` | `50.0` | sync_step 명령 타임아웃 |
| `max_rtf` | `0.0` | 최대 RTF (0 = 무제한) |
| `kp` | `5.0` | PD 비례 게인 |
| `kd` | `0.5` | PD 미분 게인 |

> **참고**: launch 파일은 `ur5e_rt_controller` 패키지의 `mujoco_sim.launch.py`를
> 가리킴 (이 패키지의 `FindPackageShare('ur5e_rt_controller')` 경유).

---

## 빌드

```bash
# MuJoCo binary tarball (lib/cmake/mujoco/ 없음)
colcon build --packages-select ur5e_mujoco_sim --symlink-install \
    --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x

# 환경변수 방식
export MUJOCO_DIR=/opt/mujoco-3.x.x
colcon build --packages-select ur5e_mujoco_sim --symlink-install

# MuJoCo 소스 빌드 시 (cmake config 존재)
colcon build --packages-select ur5e_mujoco_sim --symlink-install
```

### CMakeLists.txt 2단계 탐지 로직

```cmake
# 1단계: cmake config (소스 빌드)
find_package(mujoco QUIET)

# 2단계: binary tarball 폴백
if(NOT mujoco_FOUND)
    find_library(MUJOCO_LIBRARY mujoco
        HINTS "${mujoco_ROOT}" "$ENV{MUJOCO_DIR}"
        PATH_SUFFIXES lib NO_DEFAULT_PATH)
    find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
        HINTS "${mujoco_ROOT}" "$ENV{MUJOCO_DIR}"
        PATH_SUFFIXES include NO_DEFAULT_PATH)
    if(MUJOCO_LIBRARY AND MUJOCO_INCLUDE_DIR)
        add_library(mujoco::mujoco SHARED IMPORTED)   # IMPORTED 타겟 생성
        set(mujoco_FOUND TRUE)
    endif()
endif()

# GLFW: 선택적 (없으면 headless-only 빌드)
find_package(glfw3 QUIET)
if(glfw3_FOUND)
    target_compile_definitions(... PRIVATE MUJOCO_HAVE_GLFW)
    target_link_libraries(... glfw)
endif()
```

**MuJoCo 설치:**
```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
# 이후: colcon build --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x
```

---

## 모델 파일 (`ur5e_description` 패키지)

이 패키지는 자체 모델 파일을 가지지 않고 `ur5e_description` 패키지의 파일을 런타임에 기본 지정하여 로드합니다:

- `robots/ur5e/mjcf/scene.xml` (기본 로드 경로)
- `robots/ur5e/mjcf/ur5e.xml`

**MuJoCo Menagerie 사용:**
```bash
ros2 launch ur5e_rt_controller mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

---

## 수정 시 필수 주의사항

1. **`data_` 단독 쓰기 스레드**: `SimLoop`만 `data_->` 멤버에 직접 쓸 수 있다.
   뷰어/노드 스레드에서 `data_->` 직접 접근 금지. 반드시 뮤텍스 경유.

2. **`viz_mutex_` — `try_lock` 전용**: `ViewerLoop`와 `SimLoop` 양쪽에서 접근하며,
   `SimLoop`는 절대 block하지 않는다. `lock()` 호출 금지.

3. **`StateCallback` 컨텍스트**: `SimLoop` 스레드에서 호출됨.
   콜백 내부에서 `state_mutex_` 재진입 잠금 금지 (deadlock).

4. **`solver_niter` 인덱스**: MuJoCo 3.x에서 `int*` 배열.
   반드시 `nisland` 범위 확인 후 접근.

5. **`enable_estop: false` 필수**: `free_run` 모드에서 wall-clock 퍼블리시 간격이
   `robot_timeout_ms(100ms)`를 초과할 수 있음. `mujoco_simulator.yaml`이 이를
   `custom_controller` 오버라이드로 처리.

6. **`MUJOCO_HAVE_GLFW` 조건부 컴파일**: headless 환경에서는 `enable_viewer: false`
   또는 GLFW 미설치로 자동 비활성화. `mujoco_viewer.cpp`는 `#ifdef MUJOCO_HAVE_GLFW`
   블록으로 전체 래핑.

7. **Perturbation 적용**: `ext_xfrc_dirty_` 미설정 + `pert_mutex_.try_lock()` 실패 시
   `ClearContactForces()`가 `xfrc_applied`를 0으로 초기화 (stale force 방지).

8. **launch 파일 위치**: `mujoco_sim.launch.py`는 물리적으로
   `ur5e_rt_controller/launch/`에 있음. `ur5e_mujoco_sim/launch/`의 파일은
   이 패키지 전용 run 경우에만 사용.

---

## Compute() 시간 측정 (sync_step)

```bash
# 실행 후 CSV 분석
python3 -c "
import pandas as pd
df = pd.read_csv('~/ur_ws/logging_data/ur5e_control_log_YYMMDD_HHMM.csv')
print(df['compute_time_us'].describe())
print(f'P95: {df[\"compute_time_us\"].quantile(0.95):.1f} us')
print(f'P99: {df[\"compute_time_us\"].quantile(0.99):.1f} us')
print(f'Over 2ms: {(df[\"compute_time_us\"] > 2000).mean()*100:.2f}%')
"
```

sync_step 모드에서 시뮬 스텝 지연 = 컨트롤러 `Compute()` 지연이므로,
RTF = `dt_model / compute_time_wall` 로 해석 가능.
