# ur5e_mujoco_sim

> **Note:** This package is part of the UR5e RT Controller workspace (v5.7.0). For full architecture details, installation instructions, and ROS 2 Jazzy compatibility, please refer to the [Root README](../README.md) and [Root CLAUDE.md](../CLAUDE.md).
UR5e RT Controller 스택의 **MuJoCo 3.x 물리 시뮬레이터 패키지**입니다. 실제 UR 드라이버를 대체하여 개발 환경에서 알고리즘 검증 및 테스트를 수행할 수 있습니다.

## 개요

```
ur5e_mujoco_sim/
├── include/ur5e_mujoco_sim/
│   ├── mujoco_simulator.hpp       ← 스레드 안전 MuJoCo 3.x 물리 래퍼
│   └── ros2_resource_provider.hpp ← package:// URI provider 등록 (v5.3.0+)
├── src/
│   ├── mujoco_simulator.cpp       ← MuJoCoSimulator 구현
│   ├── mujoco_sim_loop.cpp        ← SimLoopFreeRun / SimLoopSyncStep
│   ├── viewer/                    ← GLFW 뷰어 (v5.6.0, 4-파일 구조)
│   │   ├── viewer_state.hpp       ←   공유 상태 + 함수 선언 (내부 헤더)
│   │   ├── viewer_loop.cpp        ←   ViewerLoop 멤버 함수 (~60Hz)
│   │   ├── viewer_callbacks.cpp   ←   GLFW 입력 콜백 (키/마우스)
│   │   └── viewer_overlays.cpp    ←   mjr_overlay/mjr_figure 렌더 함수
│   ├── mujoco_simulator_node.cpp  ← ROS2 노드 래퍼
│   └── ros2_resource_provider.cpp ← package:// URI 해석 구현 (v5.3.0+)
├── config/
│   └── mujoco_simulator.yaml
└── launch/
    └── mujoco_sim.launch.py
```

> **로봇 모델 파일** (`scene.xml`, `ur5e.xml`, 메시)은 v5.2.2부터 `ur5e_description` 패키지로 이동되었습니다.
> 기본 모델 경로: `ament_index` 경유 `ur5e_description/robots/ur5e/mjcf/scene.xml`
> `package://` URI는 `Ros2ResourceProvider`가 자동 해석합니다.

**의존성 그래프 내 위치:**

```
ur5e_description  ← 독립 (MJCF/URDF/메시 제공)
    ↑
ur5e_mujoco_sim  ← ur5e_description에서 모델 참조 (ament_index + package:// URI)
                    런타임에 ur5e_rt_controller와 함께 실행됨
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
    max_rtf: 1.0               # 최대 실시간 비율 (0.0 = 무제한)
    enable_viewer: true        # GLFW 3D 뷰어 활성화
    initial_joint_positions: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    enable_hand_sim: true      # 손 시뮬레이션 (1차 필터)
    hand_filter_alpha: 0.1     # 필터 계수 (10ms 틱당)

    # 물리 타임스텝 검증 (XML과 불일치 시 ERROR 출력 후 XML 우선)
    # 0.0 → 검증 없음;  > 0 → XML <option timestep>과 비교
    physics_timestep: 0.002    # seconds (= 500 Hz)

    # Position servo 게인 선택
    # false (기본): XML 원본 gainprm/biasprm 사용
    # true:  servo_kp/kd 기반 YAML gain 적용
    #   gainprm = servo_kp / physics_timestep
    #   force   = servo_kp * dq_cmd - servo_kd * dq_actual
    # position servo 모드 진입 시 gravity가 자동으로 OFF + 잠금됨.
    # torque 모드로 전환 시 gravity가 자동으로 ON됩니다.
    use_yaml_servo_gains: false
    servo_kp: [500.0, 500.0, 500.0, 150.0, 150.0, 150.0]  # Nm·s/rad
    servo_kd: [400.0, 400.0, 400.0, 100.0, 100.0, 100.0]  # Nm·s/rad

# ur5e_rt_controller 파라미터 오버라이드 (시뮬 전용 차이값만)
rt_controller:
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
| `sim_mode` | `""` | `free_run` 또는 `sync_step` (빈값 = YAML) |
| `enable_viewer` | `""` | GLFW 3D 뷰어 창 활성화 (빈값 = YAML) |
| `publish_decimation` | `""` | free_run: N 스텝마다 퍼블리시 (빈값 = YAML) |
| `sync_timeout_ms` | `""` | sync_step: 명령 대기 타임아웃 ms (빈값 = YAML) |
| `max_rtf` | `""` | 최대 실시간 비율 (빈값 = YAML, 0.0 = 무제한) |
| `use_yaml_servo_gains` | `""` | `true`=YAML servo gain, `false`=XML gain (빈값 = YAML) |

---

## 뷰어 단축키 (v5.6.0)

#### 키보드

| 키 | 기능 |
|---|---|
| **F1** | 도움말 오버레이 순환 (페이지 1 → 2 → 닫기) |
| Space | 일시정지 / 재개 |
| Right (일시정지 중) | 한 스텝 진행 |
| + / KP_ADD | RTF 속도 2배 |
| - / KP_SUB | RTF 속도 0.5배 |
| R | 초기 자세로 리셋 |
| TAB | 카메라 모드 순환 (Free → Tracking → Fixed → Free) |
| Esc | 카메라 초기화 (Free 모드로 복귀) |
| G | 중력 ON/OFF (position servo 모드에서는 잠금 — 무시됨) |
| N | 접촉 제약 ON/OFF |
| I | integrator 순환 (Euler → RK4 → Implicit → ImplFast) |
| S | solver 순환 (PGS → CG → Newton) |
| ] / [ | solver 반복 횟수 ×2 / ÷2 |
| C | 접촉점 표시 ON/OFF |
| F | 접촉력 화살표 ON/OFF |
| T | 투명 모드 ON/OFF |
| J | 관절 시각화 ON/OFF |
| U | 액추에이터 시각화 ON/OFF |
| E | 관성 타원체 ON/OFF |
| W | 질량중심(CoM) 마커 ON/OFF |
| L | 조명 시각화 ON/OFF |
| A | 텐던 시각화 ON/OFF |
| X | 볼록 껍질 ON/OFF |
| 0 / V | 지오메트리 그룹 0 ON/OFF |
| 1 – 5 | 지오메트리 그룹 1–5 ON/OFF |
| Backspace | 시각화 플래그 전체 초기화 |
| F3 | RTF 프로파일러 그래프 ON/OFF |
| F4 | Solver 통계 오버레이 ON/OFF |
| F5 | 와이어프레임 ON/OFF |
| F6 | 그림자 ON/OFF |
| F7 | 스카이박스 ON/OFF |
| F8 | 반사 ON/OFF |
| F9 | 센서 값 오버레이 ON/OFF |
| F10 | 모델 통계 오버레이 ON/OFF |
| P | 스크린샷 저장 (`~/ros2_ws/ur5e_ws/logging_data/`) |

#### 마우스

| 조작 | 기능 |
|---|---|
| Left drag | 카메라 궤도 (orbit) |
| Shift + Left drag | 수평 궤도 |
| Right drag | 카메라 패닝 (pan) |
| Shift + Right drag | 수평 패닝 |
| Scroll | 줌 인/아웃 |
| Middle drag | 줌 (드래그) |
| Dbl-click (Left) | 물체 선택 (퍼튜베이션 대상) |
| Ctrl + Left drag | 선택 물체에 토크 인가 |
| Ctrl + Right drag | 선택 물체에 힘 인가 (XZ 평면) |
| Ctrl + Shift + Right drag | 선택 물체에 힘 인가 (XY 평면) |

---

## MuJoCo ROS2 Resource Provider (v5.3.0+)

`package://` URI를 MuJoCo가 네이티브로 해석할 수 있도록 전역 resource provider를 등록합니다.

```cpp
// mujoco_simulator_node.cpp 생성 시 자동 호출
ur5e_rt_controller::RegisterRos2ResourceProvider();
```

MJCF 파일 안에서 다음과 같이 `package://` URI를 별도 변환 없이 사용 가능:
```xml
<mesh file="package://ur5e_description/robots/ur5e/meshes/ur5e.dae"/>
```

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

MuJoCo 공식 binary tarball은 `lib/cmake/mujoco/` 를 포함하지 않으므로 `-Dmujoco_ROOT` 를 사용합니다.

```bash
# MuJoCo 3.x 다운로드 및 설치
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
```

### colcon 빌드

```bash
cd ~/ur_ws

# binary tarball 설치 시 (lib/cmake/mujoco/ 없음)
colcon build --packages-select ur5e_mujoco_sim --symlink-install \
    --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x

# 또는 환경변수 사용
export MUJOCO_DIR=/opt/mujoco-3.x.x
colcon build --packages-select ur5e_mujoco_sim --symlink-install

source install/setup.bash
```

> install.sh를 사용하는 경우 `-Dmujoco_ROOT`가 자동으로 전달됩니다.

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
