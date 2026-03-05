# ur5e_mujoco_sim

UR5e RT Controller 스택의 **MuJoCo 3.x 물리 시뮬레이터 패키지**입니다. 실제 UR 드라이버를 대체하여 개발 환경에서 알고리즘 검증 및 테스트를 수행할 수 있습니다.

## 개요

```
ur5e_mujoco_sim/
├── include/ur5e_mujoco_sim/
│   └── mujoco_simulator.hpp      ← 스레드 안전 MuJoCo 3.x 물리 래퍼
├── src/
│   ├── mujoco_simulator.cpp      ← MuJoCoSimulator 구현
│   ├── mujoco_sim_loop.cpp       ← SimLoopFreeRun / SimLoopSyncStep
│   ├── mujoco_viewer.cpp         ← GLFW 뷰어 루프 (~60Hz)
│   └── mujoco_simulator_node.cpp ← ROS2 노드 래퍼
├── models/ur5e/
│   ├── scene.xml                 ← MuJoCo 씬 (지면 + UR5e)
│   └── ur5e.xml                  ← UR5e MJCF 모델
├── config/
│   └── mujoco_simulator.yaml
└── launch/
    └── mujoco_sim.launch.py
```

**의존성 그래프 내 위치:**

```
ur5e_mujoco_sim ← 독립 (ur5e_rt_base, ur5e_rt_controller에 의존하지 않음)
                   단, 런타임에 ur5e_rt_controller와 함께 실행됨
```

> **선택적 의존성**: MuJoCo 3.x가 설치된 경우에만 `mujoco_simulator_node` 빌드. 미설치 시 CMake가 자동으로 건너뜁니다.

---

## 시뮬레이션 모드

### `free_run` (기본값)

`mj_step()`을 최대 속도로 진행합니다. 알고리즘 검증 및 빠른 반복 개발에 적합합니다.

```
SimLoop → mj_step() (가능한 빠르게)
        → /joint_states 퍼블리시 (publish_decimation마다)
        → /forward_position_controller/commands 구독 (비동기)
```

### `sync_step` (동기 스텝)

상태 퍼블리시 → 명령 대기 → 물리 스텝을 순서대로 수행합니다. `Compute()` 지연 시간을 정확히 측정할 수 있습니다.

```
SimLoop → /joint_states 퍼블리시
        → /forward_position_controller/commands 대기 (sync_timeout_ms)
        → mj_step()
        → (반복)
```

---

## ROS2 인터페이스

### 퍼블리시 토픽

| 토픽 | 타입 | 주기 | 설명 |
|------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 물리 속도 또는 데시메이션 | 6-DOF 관절 위치/속도 |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | 100Hz | 시뮬레이션된 손 상태 (1차 필터) |
| `/sim/status` | `std_msgs/Float64MultiArray` | 1Hz | `[step_count, sim_time_sec, rtf, paused(0/1)]` |

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 6개 로봇 위치 명령 (rad) |
| `/hand/command` | `std_msgs/Float64MultiArray` | 11개 손 명령 (0.0–1.0) |

---

## 스레딩 모델

```
mujoco_simulator_node
    │
    ├── SimLoop 스레드 (jthread)
    │     └── SimLoopFreeRun 또는 SimLoopSyncStep
    │         ├── cmd_mutex_ + cmd_pending_ (atomic) — 명령 전달 (잠금-없음 fast path)
    │         ├── sync_cv_ — SyncStep 명령 대기 (조건 변수)
    │         └── state_mutex_ — 최신 상태 스냅샷 보호
    │
    └── ViewerLoop 스레드 (jthread, 선택적)
          └── GLFW 3D 뷰어 ~60Hz
              └── viz_mutex_ (try_lock 전용 — SimLoop 절대 블로킹 안 함)
```

---

## 설정

### `config/mujoco_simulator.yaml`

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: ""             # 빈 값 → <패키지>/models/ur5e/scene.xml
    sim_mode: "free_run"       # "free_run" 또는 "sync_step"
    publish_decimation: 1      # free_run: N 스텝마다 퍼블리시
    sync_timeout_ms: 50.0      # sync_step: 명령 대기 타임아웃 (ms)
    max_rtf: 0.0               # 최대 실시간 비율 (0.0 = 무제한)
    enable_viewer: true        # GLFW 3D 뷰어 활성화
    initial_joint_positions: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    enable_hand_sim: true      # 손 시뮬레이션 (1차 필터)
    hand_filter_alpha: 0.1     # 필터 계수 (10ms 틱당)

# ur5e_rt_controller 파라미터 오버라이드 (mujoco_sim.launch.py에서)
custom_controller:
  ros__parameters:
    enable_estop: false        # free_run에서 false E-STOP 방지
    robot_timeout_ms: 10000.0
    hand_timeout_ms:  10000.0
```

---

## 실행

### MuJoCo 시뮬레이션

```bash
# 기본 (free_run, 뷰어 활성화)
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py

# 동기 스텝 모드
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py sim_mode:=sync_step

# 헤드리스 (CI/서버)
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py enable_viewer:=false

# 외부 MuJoCo 모델 사용
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `model_path` | `""` | scene.xml 절대 경로 (빈값 = 패키지 기본) |
| `sim_mode` | `free_run` | `free_run` 또는 `sync_step` |
| `enable_viewer` | `true` | GLFW 3D 뷰어 창 활성화 |
| `publish_decimation` | `1` | free_run: N 스텝마다 퍼블리시 |
| `sync_timeout_ms` | `50.0` | sync_step: 명령 대기 타임아웃 (ms) |
| `max_rtf` | `0.0` | 최대 실시간 비율 (0.0 = 무제한) |
| `kp` | `5.0` | PD 비례 게인 |
| `kd` | `0.5` | PD 미분 게인 |

---

## 뷰어 단축키

GLFW 3D 뷰어 실행 중 사용 가능한 키보드 단축키:

| 키 | 기능 |
|---|---|
| **F1** | 도움말 오버레이 (모든 키 + ON/OFF 상태) |
| Space | 일시정지 / 재개 |
| + / - | RTF 속도 2배 / 0.5배 |
| R | 초기 자세로 리셋 |
| G | 중력 ON/OFF |
| N | 접촉 제약 ON/OFF |
| I | integrator 순환 (Euler→RK4→Implicit→ImplFast) |
| S | solver 순환 (PGS→CG→Newton) |
| ] / [ | solver 반복 횟수 ×2 / ÷2 |
| C / F | 접촉점 / 힘 화살표 표시 |
| V / T | 충돌 지오메트리 / 투명 모드 |
| F3 / F4 | RTF 프로파일러 / solver 통계 오버레이 |
| Ctrl + Left drag | 물체에 스프링 힘 인가 |

---

## Physics Solver 런타임 제어

`MuJoCoSimulator` API로 런타임에 물리 파라미터를 변경할 수 있습니다:

```cpp
auto sim = std::make_unique<MuJoCoSimulator>(cfg);
sim->Start();

// Integrator 변경 (강성 시스템에 implicit 권장)
sim->SetIntegrator(mjINT_IMPLICIT);

// Solver 변경
sim->SetSolverType(mjSOL_NEWTON);      // 가장 정확
sim->SetSolverIterations(200);
sim->SetSolverTolerance(1e-9);

// 중력 / 접촉 토글
sim->EnableGravity(false);             // 무중력 테스트
sim->SetContactEnabled(false);         // 자유 공간 운동 테스트

// 외부 힘 인가 (wrench: [fx, fy, fz, tx, ty, tz])
sim->SetExternalForce(body_id, {0.0, 0.0, 10.0, 0.0, 0.0, 0.0});
sim->ClearExternalForce();

// RTF 상한 동적 변경
sim->SetMaxRtf(5.0);                   // 실시간 5배

// 시뮬레이터 상태 조회
auto stats = sim->GetStats();          // .step_count, .sim_time, .rtf, .paused
auto snapshot = sim->GetStateSnapshot();
```

---

## 시뮬레이터 상태 모니터링

```bash
# 시뮬 상태 확인 (스텝 수, 시뮬 시간, RTF)
ros2 topic echo /sim/status

# 관절 상태 확인
ros2 topic echo /joint_states

# 제어 주기 확인
ros2 topic hz /forward_position_controller/commands
```

---

## sync_step 컴퓨트 시간 분석

```python
import pandas as pd
df = pd.read_csv('/tmp/ur5e_control_log.csv')
print(df['compute_time_us'].describe())
print(f'P95: {df["compute_time_us"].quantile(0.95):.1f} us')
print(f'P99: {df["compute_time_us"].quantile(0.99):.1f} us')
print(f'Over 2ms: {(df["compute_time_us"] > 2000).mean()*100:.2f}%')
```

---

## 빌드

### 전제 조건: MuJoCo 3.x 설치

```bash
# MuJoCo 3.x 다운로드 및 설치
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/

# CMake로 빌드 시 경로 지정
cmake -Dmujoco_DIR=/opt/mujoco-3.x.x/lib/cmake/mujoco ...
```

### colcon 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_mujoco_sim --symlink-install
source install/setup.bash
```

MuJoCo가 설치되지 않은 경우 `mujoco_simulator_node`는 자동으로 빌드에서 제외됩니다.

---

## MuJoCo Menagerie 사용

```bash
# MuJoCo Menagerie의 UR5e 모델 사용
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

---

## 라이선스

MIT License
