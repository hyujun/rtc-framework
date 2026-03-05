# Changelog

모든 주요 변경사항은 이 파일에 기록됩니다.
형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/)를 따릅니다.

---

## [5.0.0] - 2026-03-04

### 변경 — 멀티-패키지 분리 (Breaking: 빌드 구조 변경)

단일 `ur5e_rt_controller` 패키지를 **5개 독립 ROS2 패키지**로 분리하였습니다. 기능 변경은 없으며 패키지 경계와 의존성만 재편성되었습니다.

#### 신규 패키지

| 패키지 | 역할 | 버전 |
|--------|------|------|
| `ur5e_rt_base` | 공유 타입/스레드 유틸/로깅 (헤더-전용) | 1.0.0 |
| `ur5e_rt_controller` | 500Hz 실시간 제어기 | 5.0.0 |
| `ur5e_hand_udp` | UDP 손 브리지 (수신/송신 노드) | 1.0.0 |
| `ur5e_mujoco_sim` | MuJoCo 3.x 물리 시뮬레이터 | 1.0.0 |
| `ur5e_tools` | Python 개발 유틸리티 | 1.0.0 |

#### 패키지 의존성 그래프

```
ur5e_rt_base       ← 독립
    ↑
    ├── ur5e_rt_controller
    └── ur5e_hand_udp

ur5e_mujoco_sim    ← 독립
ur5e_tools         ← 독립
```

#### 이동된 파일

| 구성 요소 | 이전 위치 | 신규 위치 |
|-----------|-----------|-----------|
| 공유 타입 | `rt_controller_interface.hpp` (내장) | `ur5e_rt_base/types.hpp` |
| 스레드 설정/유틸 | `ur5e_rt_controller/thread_*.hpp` | `ur5e_rt_base/thread_*.hpp` |
| 로그 버퍼/로거 | `ur5e_rt_controller/log_buffer.hpp`, `data_logger.hpp` | `ur5e_rt_base/` |
| UDP 브리지 | `ur5e_rt_controller/hand_udp_*.hpp`, `*_node.cpp` | `ur5e_hand_udp/` |
| MuJoCo 시뮬레이터 | `ur5e_rt_controller/mujoco_simulator.hpp`, `*_node.cpp` | `ur5e_mujoco_sim/` |
| Python 스크립트 | `scripts/` | `ur5e_tools/scripts/` |
| MuJoCo 모델 | `models/ur5e/` | `ur5e_mujoco_sim/models/ur5e/` |

#### 빌드 방법 변경

```bash
# 이전 (v4.x)
colcon build --packages-select ur5e_rt_controller

# v5.0.0 이후
colcon build --packages-select ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools
# MuJoCo 포함 시:
colcon build --packages-select ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools ur5e_mujoco_sim
```

#### 헤더 인클루드 경로 변경

```cpp
// 이전 (v4.x)
#include "ur5e_rt_controller/thread_config.hpp"
#include "ur5e_rt_controller/thread_utils.hpp"
#include "ur5e_rt_controller/log_buffer.hpp"
#include "ur5e_rt_controller/data_logger.hpp"
#include "ur5e_rt_controller/hand_udp_receiver.hpp"
#include "ur5e_rt_controller/mujoco_simulator.hpp"

// v5.0.0 이후
#include "ur5e_rt_base/thread_config.hpp"
#include "ur5e_rt_base/thread_utils.hpp"
#include "ur5e_rt_base/log_buffer.hpp"
#include "ur5e_rt_base/data_logger.hpp"
#include "ur5e_hand_udp/hand_udp_receiver.hpp"
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"
```

네임스페이스 `ur5e_rt_controller`는 모든 패키지에서 **변경 없이 유지**됩니다.

---

## [4.5.0] - 2026-03-04

### 추가 (Added) — MuJoCo 시뮬레이터 대폭 개선

#### 인터랙티브 뷰어 — 마우스 / 키보드 제어

`mujoco_simulator.hpp` ViewerLoop에 완전한 인터랙티브 컨트롤 추가:

| 입력 | 동작 |
|---|---|
| Left drag | 카메라 orbit |
| Right drag | 카메라 pan |
| Scroll | 줌 in/out |
| Ctrl + Left drag | 물체에 스프링 힘 적용 (mjvPerturb) |
| **F1** | 키 안내 + 현재 상태 오버레이 토글 (ON/OFF 실시간 표시) |
| Space | 일시정지 / 재개 |
| + / - | RTF 속도 2배 / 0.5배 조절 |
| R | 초기 자세로 리셋 |
| G | 중력 토글 |
| N | 접촉 제약 토글 (mjDSBL_CONTACT) |
| I | integrator 순환 (Euler→RK4→Implicit→ImplFast) |
| S | solver 순환 (PGS→CG→Newton) |
| ] / [ | solver iterations ×2 / ÷2 |
| C / F / V / T | 접촉점 / 힘 화살표 / 충돌 지오메트리 / 투명 토글 |
| F3 | RTF 프로파일러 그래프 토글 |
| F4 | solver 통계 오버레이 토글 |
| Backspace / Esc | 시각화 옵션 리셋 / 카메라 리셋 |

#### 물리 기능 (Physics Features)

- **관절 토크 (Efforts)**: `StateCallback` 3번째 인자로 `data_->qfrc_actuator` 전달
  - `mujoco_simulator_node.cpp`가 `sensor_msgs/JointState.effort` 필드에 실제 토크 발행
- **중력 토글**: `EnableGravity(bool)` — `model_->opt.gravity[2]` 런타임 조정
- **물체 힘 인가 (Perturbation)**: Ctrl+Left-drag → `mjv_select` + `mjv_initPerturb` + `mjv_movePerturb` + `mjv_applyPerturbForce`
  - `UpdatePerturb(mjvPerturb)` / `ClearPerturb()` public API
  - `pert_mutex_`로 뷰어 스레드 → 시뮬 스레드 전달
- **외부 힘 API**: `SetExternalForce(body_id, wrench_world)` / `ClearExternalForce()`
  - `data_->xfrc_applied`에 월드 프레임 6-DOF wrench 적용

#### 물리 Solver 제어 (Physics Solver Controls)

런타임에 MuJoCo physics solver 파라미터를 thread-safe하게 조정:

| API | 설명 |
|---|---|
| `SetIntegrator(int)` | mjINT_EULER(0) / RK4(1) / IMPLICIT(2) / IMPLICITFAST(3) |
| `SetSolverType(int)` | mjSOL_PGS(0) / CG(1) / NEWTON(2) |
| `SetSolverIterations(int)` | 최대 반복 횟수 (1–1000 클램프) |
| `SetSolverTolerance(double)` | 수렴 허용 오차 (0.0 = 비활성화) |
| `SetContactEnabled(bool)` | 접촉 제약 ON/OFF (mjDSBL_CONTACT 플래그) |
| `GetSolverStats()` | `{improvement, gradient, iter, ncon}` — 매 step 캡처 |

- `Config` 구조체에 초기값 필드 추가: `integrator_type`, `solver_type`, `solver_iterations`, `solver_tolerance`
- `PreparePhysicsStep()`에서 매 step 전에 solver 파라미터를 atomic에서 읽어 `model_->opt`에 적용
- `ReadSolverStats()`가 `mj_step()` 직후 `data_->solver[0]` 통계 캡처

#### 오버레이 업데이트

- **상단 우측 상태 패널**: Integrator / Solver / Iterations / Residual 행 추가
- **F4 Solver 통계 오버레이**: Integrator, Solver, Max/Used 반복, Improvement, Gradient, 접촉 수, Timestep
- **F1 도움말 오버레이**: 모든 키를 카테고리별로 정리, 토글 가능 옵션마다 `[ON/OFF]` 실시간 표시

### 추가 (Added) — install.sh 모드 분리

```bash
./install.sh          # 전체 설치 (기본값)
./install.sh sim      # 시뮬레이션 전용
./install.sh robot    # 실제 로봇 전용
./install.sh --help   # 사용법
```

| 모드 | 설치 내용 |
|---|---|
| `sim` | ROS2 빌드 도구 + Pinocchio + MuJoCo 3.x 자동 다운로드 |
| `robot` | ROS2 빌드 도구 + UR 드라이버 + Pinocchio + RT 권한 설정 |
| `full` | 위 모두 |

- MuJoCo 3.x 자동 설치: GitHub에서 tarball 다운로드 → `/opt/`에 압축 해제 → `/etc/ld.so.conf.d/mujoco.conf` 생성
- GLFW / OpenGL 개발 라이브러리 (`libglfw3-dev`, `libgl1-mesa-dev`) 자동 설치
- `colcon build`에 `-Dmujoco_DIR` cmake 인자 자동 전달 (sim/full 모드)
- 모드별 Quick Start 요약 출력

### 사용자 영향

- `StateCallback` 시그니처 변경: `void(positions, velocities)` → `void(positions, velocities, efforts)`
  - `mujoco_simulator_node`는 이미 3인자 버전으로 업데이트됨
  - 직접 `SetStateCallback`을 사용하는 코드는 람다에 3번째 인자 추가 필요
- `mujoco_simulator_node`가 발행하는 `/joint_states`에 실제 액추에이터 토크가 포함됨

---

## [4.4.0] - 2026-03-03

### 추가 (Added) — MuJoCo 시뮬레이션 통합

#### `MuJoCoSimulator` (`include/ur5e_rt_controller/mujoco_simulator.hpp`)

MuJoCo 3.x 물리 모델을 thread-safe하게 래핑한 클래스.

**시뮬레이션 모드**:
- `kFreeRun`: `mj_step()`을 가능한 빠르게 실행 (`max_rtf`로 속도 제한 가능)
- `kSyncStep`: 상태 발행 → 명령 대기 → 1스텝. 지연 ≈ `Compute()` 시간

**스레딩 모델**:
- `SimLoop` 스레드: 물리 연산, `model_/data_`의 유일한 쓰기 스레드
- `ViewerLoop` 스레드: GLFW 3D 뷰어 (~60 Hz, 선택적)
- 호출자 스레드: `SetCommand()`, `GetPositions()` 등

**동기화**:
- `cmd_mutex_` + `cmd_pending_` 원자 플래그 — FreeRun 빠른 경로
- `sync_cv_` — SyncStep에서 명령 대기
- `state_mutex_` — 최신 positions/velocities/efforts
- `viz_mutex_` — try_lock만 사용, SimLoop 블로킹 없음

#### `mujoco_simulator_node` (`src/mujoco_simulator_node.cpp`)

ROS2 노드 래퍼:
- `/joint_states` 발행 (물리 속도 또는 데시메이션)
- `/hand/joint_states` 발행 (100Hz, 1차 필터 시뮬레이션)
- `/sim/status` 발행 — `[step_count, sim_time_sec, rtf, paused]` (Float64MultiArray)
- `/forward_position_controller/commands` 구독
- `/hand/command` 구독

#### `mujoco_sim.launch.py` (`launch/mujoco_sim.launch.py`)

| 인자 | 기본값 | 설명 |
|---|---|---|
| `model_path` | `""` | scene.xml 절대경로 (빈 문자열 = 패키지 기본) |
| `sim_mode` | `free_run` | `free_run` 또는 `sync_step` |
| `enable_viewer` | `true` | GLFW 3D 뷰어 창 열기 |
| `publish_decimation` | `1` | free_run: N 스텝마다 발행 |
| `sync_timeout_ms` | `50.0` | sync_step: 명령 대기 타임아웃 |
| `max_rtf` | `0.0` | 최대 Real-Time Factor (0.0 = 무제한) |
| `kp` / `kd` | `5.0` / `0.5` | PD 제어기 게인 |

#### `ControllerTimingProfiler` (`include/ur5e_rt_controller/controller_timing_profiler.hpp`)

`RTControllerInterface::Compute()` 시간 측정 래퍼:
- `MeasuredCompute()` 호출 시 `steady_clock`으로 compute 시간 측정
- 락프리 히스토그램 (0–2000 µs, 100 µs 버킷) + min/max/mean/stddev/p95/p99
- `Summary()`: 1000 스텝마다 로그 출력 권장
- `LogEntry`에 `compute_time_us` 필드 추가 → CSV에 기록

#### MuJoCo 모델 파일 (`models/ur5e/`)

- `scene.xml`: 지면판 + UR5e MJCF 씬
- `ur5e.xml`: UR5e 로봇 MJCF 모델

#### `config/mujoco_simulator.yaml`

`mujoco_simulator` 노드 파라미터 + `custom_controller` E-STOP 오버라이드:
```yaml
custom_controller:
  ros__parameters:
    enable_estop: false       # free_run 모드에서 오경보 방지
    robot_timeout_ms: 10000.0
    hand_timeout_ms:  10000.0
```

### 추가 (Added) — RTF 측정 및 뷰어 오버레이 (v4.4.0 기본 뷰어)

- RTF (Real-Time Factor) 측정: 200 스텝마다 갱신
- 상태 오버레이 (우측 상단): 모드/RTF/제한/시뮬 시간/스텝 수 표시
- `/sim/status` 토픽: `[step_count, sim_time_sec, rtf]`

### 사용자 영향

- MuJoCo 없이 빌드 시 `mujoco_simulator_node`만 빌드에서 제외됨 — 기존 기능 영향 없음
- `mujoco_sim.launch.py`로 MuJoCo 시뮬레이션 실행 가능

---

## [4.3.0] - 2026-03-03

### 추가 (Added) — Pinocchio 모델 기반 제어기 3종

#### `PinocchioController` (`include/ur5e_rt_controller/controllers/pinocchio_controller.hpp`)
관절공간 PD 제어에 Pinocchio 동역학 모델을 결합한 제어기.

- **제어 법칙**: `command[i] = Kp * e[i] + Kd * ė[i] + g(q)[i] [+ C(q,v)·v[i]]`
- **중력 보상**: `pinocchio::computeGeneralizedGravity(model, data, q)` — Pinocchio RNEA
- **코리올리 보상**: `pinocchio::computeCoriolisMatrix(model, data, q, v)` (선택적, 기본 off)
- **Forward Kinematics**: `data_.oMi[end_id_]`에서 TCP 위치 캐시
- **야코비안**: `pinocchio::computeJointJacobian` — 진단용 접근자 제공
- 모든 Eigen 작업 버퍼 생성자에서 사전 할당 → 500 Hz 경로 힙 할당 없음
- `LDLT<Matrix3d>` (3×3 고정크기) 사용으로 스택 할당 보장
- E-STOP 시 `kSafePosition = [0, -1.57, 1.57, -1.57, -1.57, 0]`으로 수렴

#### `ClikController` (`include/ur5e_rt_controller/controllers/clik_controller.hpp`)
감쇠 야코비안 유사역행렬을 이용한 Closed-Loop IK. **Cartesian 위치(3-DOF) 제어**.

- **타겟 입력**: `[x, y, z, null_q3, null_q4, null_q5]` — 처음 3개는 TCP 위치 (m), 나머지 3개는 null-space 관절 참조값 (rad)
- **1차 태스크**: `dq = kp * J_pos^# * pos_error`
  - 감쇠 유사역행렬: `J_pos^# = J_pos^T (J_pos J_pos^T + λ²I)^{-1}`
- **null-space 2차 태스크**: `dq += null_kp * (I − J_pos^# J_pos) * (q_null − q)`
  - 잉여 DOF를 선호 자세(관절 한계 회피 등)로 유지
- **적분**: `q_cmd = q + clamp(dq, ±v_max) * dt`
- `LDLT<Matrix3d>` (3×3 고정크기) — 스택 할당, RT 경로 힙 할당 없음
- 진단 접근자: `tcp_position()`, `position_error()`

#### `OperationalSpaceController` (`include/ur5e_rt_controller/controllers/operational_space_controller.hpp`)
완전 6-DOF Cartesian PD 제어기. 위치와 자세를 동시에 제어.

- **타겟 입력**: `[x, y, z, roll, pitch, yaw]` — 위치 (m) + 자세 (rad, ZYX 오일러)
- **자세 오차**: `rot_err = log₃(R_des * R_FK(q)^T)` — Pinocchio `pinocchio::log3()` (SO(3) 로그맵)
- **태스크공간 PD 법칙**:
  ```
  task_vel[0:3] = kp_pos * pos_err  −  kd_pos * (J·dq)[0:3]
  task_vel[3:6] = kp_rot * rot_err  −  kd_rot * (J·dq)[3:6]
  ```
- **역운동학**: `dq = J^# * task_vel`, `J^# = J^T (J J^T + λ²I₆)^{-1}`
- **선택적 중력 보상**: `enable_gravity_compensation` 플래그로 활성화
- `PartialPivLU<Matrix<double,6,6>>` (6×6 고정크기) — 스택 할당, RT 경로 힙 할당 없음
- 진단 접근자: `tcp_position()`, `pose_error()`, `jacobian()`
- `SetRobotTarget()` 내에서 RPY → 회전행렬 사전 계산 (센서 스레드, RT 경로 외부)

### 추가 (Added) — 빌드 시스템

- **`CMakeLists.txt`**: `find_package(pinocchio REQUIRED)` + `target_link_libraries(custom_controller pinocchio::pinocchio)` 추가
- **`package.xml`**: `<depend>pinocchio</depend>` 추가
- **`install.sh`**: `ros-humble-pinocchio` apt 설치 단계 추가, 버전 표시 v4.3.0으로 업데이트

### 변경 (Changed)

- **`src/custom_controller.cpp`** 상단 주석 블록: 3가지 Pinocchio 제어기 교체 가이드 표 추가 (4단계 마이그레이션 절차 포함)
- **`README.md`**: Pinocchio 제어기 섹션 신규 추가, 의존성 표 업데이트, 버전 이력 갱신

### 공통 RT 안전성 설계

| 항목 | 내용 |
|---|---|
| 힙 할당 (500 Hz 경로) | 없음 — 생성자에서 모든 Eigen 버퍼 사전 할당 |
| 행렬 분해 | 고정크기 Eigen 타입 사용 (스택 할당) |
| noexcept | 모든 public 인터페이스 메서드 필수 적용 |
| E-STOP | `std::atomic<bool>` 플래그, kSafePosition으로 수렴 |
| URDF 로드 | 생성자에서 1회만 수행 (RT 경로 외부) |

### 사용자 영향
- **기존 동작 유지**: PDController 기본값 그대로 — 아무것도 바꾸지 않아도 됨
- **Pinocchio 설치 필요**: `sudo apt install ros-humble-pinocchio`
- **URDF 경로 지정 필요**: 각 Pinocchio 제어기 생성자에 절대 경로 전달

---

## [4.2.3] - 2026-03-03

### 수정 (Fixed) — RT 안전성 9건

#### CRITICAL: RT 스레드 파일 I/O 제거 (Fix 1)
- **문제**: `ControlLoop()` (SCHED_FIFO 90, Core 2)에서 `DataLogger::LogControlData()` 직접 호출 → `std::ofstream` syscall로 수백 µs 블로킹 가능
- **해결**: `log_buffer.hpp` 신규 추가 — `SpscLogBuffer<512>` lock-free SPSC 링 버퍼
  - `ControlLoop()`는 `log_buffer_.Push(entry)` 만 호출 (O(1), 힙 할당 없음)
  - `drain_timer_` (100Hz, log 스레드 Core 4)가 `logger_->DrainBuffer(log_buffer_)` 호출
  - 파일 I/O가 RT 경로에서 완전히 제거됨

#### CRITICAL: `state_received_` / `target_received_` 데이터 레이스 (Fix 2)
- **문제**: `state_received_`, `target_received_`, `hand_data_received_` 가 plain `bool`로 선언되어 RT 스레드(읽기)와 sensor 스레드(쓰기)가 동시 접근 → C++ Undefined Behavior
- **해결**: `std::atomic<bool>` 으로 교체; 쓰기는 `memory_order_release`, 읽기는 `memory_order_acquire` 사용

#### CRITICAL: `TargetCallback`에서 뮤텍스 해제 후 `target_positions_` 읽기 (Fix 3)
- **문제**: `target_mutex_` 해제 후 `controller_->SetRobotTarget(target_positions_)` 호출 → RT 스레드가 사이에 덮어쓸 수 있음
- **해결**: 뮤텍스 안에서 `local_target = target_positions_` 복사본 생성 후 복사본으로 호출

#### MAJOR: `target_mutex_` 블록 논리 오류 + 로깅 무보호 읽기 (Fix 4)
- **문제**: `ControlLoop()`의 `target_mutex_` 블록이 `target_positions_` 복사 없이 무관한 필드만 설정; 로깅 시 `target_positions_`를 뮤텍스 없이 읽음
- **해결**: `target_snapshot_` 멤버 추가 — `target_mutex_` 안에서 복사 후 루프 전체에서 스냅샷만 사용; `state.robot.dt`, `state.iteration`은 뮤텍스 밖으로 이동

#### MAJOR: RT 스레드에서 `publish()` 힙 할당 (Fix 5)
- **문제**: `std_msgs::msg::Float64MultiArray` 생성 + `vector::assign()` + `publish()` 가 RT 루프에서 힙 할당 유발 가능
- **해결**: `realtime_tools::RealtimePublisher<T>` 도입 — 메시지 사전 할당, `trylock()` / `unlockAndPublish()`로 non-blocking 발행

#### MAJOR: `HandUdpReceiver` jthread RT 스케줄링 미적용 (Fix 6)
- **문제**: `kUdpRecvConfig` (Core 3, SCHED_FIFO 65)가 선언만 되고 jthread에 전혀 적용되지 않음
- **해결**: `HandUdpReceiver` 생성자에 `const ThreadConfig& thread_cfg = kUdpRecvConfig` 파라미터 추가; `thread_cfg_` 멤버 저장 (구현체의 `ReceiveLoop()` 진입 시 `ApplyThreadConfig()` 호출 필요)
- `hand_udp_receiver_node.cpp` main()에 `mlockall()` 추가

#### MAJOR: `mlockall` 호출 순서 오류 (Fix 7)
- **문제**: `rclcpp::init()` 이후 `mlockall(MCL_CURRENT)` 호출 → DDS/RMW가 이미 할당한 페이지 미잠금
- **해결**: `mlockall(MCL_CURRENT | MCL_FUTURE)` → `rclcpp::init()` 순서로 재배치; `MCL_FUTURE`로 이후 할당도 자동 잠금

#### MEDIUM: `GetThreadStats()` include 누락 (Fix 8)
- **문제**: `thread_utils.hpp`가 `std::min_element`, `std::accumulate`, `std::tuple`, `std::vector` 를 사용하나 해당 헤더 미포함 → 다른 TU에서 직접 include 시 컴파일 실패 가능
- **해결**: `<algorithm>`, `<numeric>`, `<tuple>`, `<vector>` 추가; `thread_config.hpp`에 `<sched.h>` 추가 (독립 포함 지원)

#### LOW: 4코어 fallback 자동 미적용 (Fix 9)
- **문제**: `kRtControlConfig4Core` 등이 선언만 되고 `main()`은 항상 6코어 설정 하드코딩 → 4코어 시스템에서 CPU affinity 실패
- **해결**: `thread_utils.hpp`에 `GetOnlineCpuCount()` (`sysconf` 래퍼) + `SelectThreadConfigs()` 추가; `main()`이 런타임에 코어 수를 감지해 적절한 설정 선택

### 추가 (Added)
- `include/ur5e_rt_controller/log_buffer.hpp`: `LogEntry` 구조체 + `SpscLogBuffer<N>` 템플릿 + `ControlLogBuffer` typedef
- `DataLogger::DrainBuffer(ControlLogBuffer&)`: log 스레드 전용 드레인 메서드
- `GetOnlineCpuCount()`, `SelectThreadConfigs()`, `SystemThreadConfigs`: `thread_utils.hpp`에 추가

### 사용자 영향
- **기능 변화 없음**: ROS2 인터페이스(토픽, 파라미터, 서비스) 동일
- **로그 파일 계속 생성**: drain 타이머(100Hz)가 log 스레드에서 CSV 기록 유지
- **4코어 시스템**: 별도 설정 없이 자동으로 4코어 스레드 설정 적용
- **RT 지터 개선**: 파일 I/O 및 힙 할당 제거로 500Hz 루프 안정성 향상

---

## [4.2.2] - 2026-03-02

### 제거 (Removed)
- **organize_files.sh 삭제**: 일회성 파일 정리 스크립트 제거
  - v4.0.0에서 파일 정리 작업 이미 완료
  - 파일명 불일치로 실행 시 오류 발생 (`monitor_data_health_v2.py` 없음)
  - package.xml을 오래된 버전(v4.0.0)으로 덮어쓸 위험

### 변경 (Changed)
- **install.sh v4.2.1 업데이트**:
  - 버전 표시: v4.0.0 → v4.2.1
  - GitHub URL 수정: `your-repo/ur5e_rt_controller` → `hyujun/ur5e-rt-controller`
  - organize_files.sh 호출 제거 (더 이상 불필요)
  - 실행 파일명 수정: `monitor_data_health_v2.py` → `monitor_data_health.py`
  - requirements.txt 사용: `pip3 install --user -r requirements.txt`
  - chmod 명령 제거: CMakeLists.txt의 `install(PROGRAMS)`가 자동 처리
  - **RT 권한 설정 추가** (v4.2.0+ 필수):
    - realtime 그룹 생성 및 사용자 추가
    - `/etc/security/limits.conf`에 rtprio 99, memlock unlimited 설정
    - 로그아웃/로그인 안내 메시지

### 개선 (Improved)
- **install.sh Quick Start 섹션 확장**:
  - 4개 Python 유틸리티 스크립트 모두 예제 포함
    1. monitor_data_health.py
    2. plot_ur_trajectory.py
    3. motion_editor_gui.py
    4. hand_udp_sender_example.py
  - RT_OPTIMIZATION.md 참조 추가
  - v4.2.0+ 병렬 컴퓨팅 설정 안내 추가

### 사용자 영향
- **신규 설치 시**: install.sh가 RT 권한 자동 설정
- **기존 사용자**: 영향 없음 (organize_files.sh 이미 사용 안 함)
- **Quick Start 가이드 개선**: 모든 유틸리티 스크립트 사용법 명확화

---

## [4.2.1] - 2026-03-02

### 제거 (Removed)
- **setup.py 삭제**: ROS2 ament_cmake 표준 구조 준수
  - ROS2 C++ 패키지는 setup.py 불필요 (CMakeLists.txt로 충분)
  - 기존 setup.py는 패키지 디스커버리 실패 (`find_packages()` 0개 발견)
  - entry_points 파일명 불일치 (`monitor_data_health_v2.py` 파일 없음)

### 변경 (Changed)
- **CMakeLists.txt v4.2.1 업데이트**:
  - VERSION: 4.0.0 → 4.2.1 (package.xml과 일치)
  - `install(PROGRAMS)` 섹션에 누락된 스크립트 2개 추가:
    - `scripts/motion_editor_gui.py`
    - `scripts/hand_udp_sender_example.py`
  - 총 4개 Python 유틸리티 스크립트 설치 (`ros2 run` 사용 가능)

### 사용자 영향
- **Python 의존성 설치 방법 변경**:
  ```bash
  # 이전 (작동 안 함)
  pip install -e .  # ❌ 실패
  
  # 변경 후 (명확)
  pip3 install --user -r requirements.txt  # Python 의존성 설치
  colcon build                              # ROS2 패키지 빌드
  ```

- **스크립트 실행 방법 표준화**:
  ```bash
  # ROS2 표준 (권장)
  ros2 run ur5e_rt_controller monitor_data_health.py
  ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv
  ros2 run ur5e_rt_controller motion_editor_gui.py
  ros2 run ur5e_rt_controller hand_udp_sender_example.py
  
  # 또는 직접 실행
  python3 ~/ur_ws/src/ur5e-rt-controller/scripts/plot_ur_trajectory.py <csv_file>
  ```

### 기술적 세부사항
- **requirements.txt 유지**: 문서 목적으로 보관
- **CMakeLists.txt가 모든 설치 처리**: C++ 실행 파일, Python 스크립트, 헤더, 설정 파일
- **ROS2 Best Practice 준수**: ament_cmake 패키지는 setup.py 불필요

---

## [4.2.0] - 2026-03-02

### 추가 (Added)
- **병렬 컴퓨팅 최적화**: CallbackGroup 기반 멀티스레드 executor 아키텍처
  - `thread_config.hpp`: 스레드 설정 구조체 및 사전 정의 설정 (RT, Sensor, Logging, Aux)
  - `thread_utils.hpp`: RT 스케줄링 및 CPU affinity 유틸리티 함수
    - `ApplyThreadConfig()`: SCHED_FIFO, CPU affinity, nice value 설정
    - `VerifyThreadConfig()`: 스레드 설정 검증 및 로깅
    - `GetThreadStats()`: 지터 측정용 통계 함수
- **4개 CallbackGroup 분리** (`custom_controller.cpp`):
  - `cb_group_rt_`: control_timer_(500Hz), timeout_timer_(50Hz) → Core 2, SCHED_FIFO 90
  - `cb_group_sensor_`: joint_state_sub_, target_sub_, hand_state_sub_ → Core 3, SCHED_FIFO 70
  - `cb_group_log_`: 로깅 작업 → Core 4, SCHED_OTHER nice -5
  - `cb_group_aux_`: estop_pub_ → Core 5, SCHED_OTHER
- **mlockall 메모리 잠금**: main()에서 페이지 폴트 방지
- **스레드별 RT 설정 자동화**: 각 executor를 별도 스레드에서 실행하며 스케줄러 정책 자동 적용

### 변경 (Changed)
- **`custom_controller.cpp` 재설계**:
  - SingleThreadedExecutor 4개로 분리 (이전: 1개)
  - 각 executor를 std::thread에서 spin() + RT 설정 적용
  - 콜백 그룹 인자를 create_subscription/create_wall_timer에 명시
- **main() 분리**: mlockall, executor 생성, 스레드 spawn 로직 추가
- **CMakeLists.txt**: C++20 표준 유지, thread_utils.hpp 헤더 install 경로 추가

### 성능 개선
- **제어 지터 목표**: ~500μs → <50μs (10배 개선 예상)
- **E-STOP 반응 시간**: ~100ms → <20ms (5배 개선 예상)
- **Priority inversion 제거**: RT 루프와 I/O가 별도 코어에서 실행
- **CPU 마이그레이션 차단**: CPU affinity로 cache warmup 유지

### 시스템 요구사항 변경
- **RT 권한 필수**: `/etc/security/limits.conf`에 `@realtime - rtprio 99` 설정
- **6-core CPU 권장**: Core 0-1(OS/DDS), 2(RT), 3(Sensor), 4(Log), 5(Aux)
- **4-core fallback 지원**: thread_config.hpp에 4-core 설정 포함
- **PREEMPT_RT 커널 권장**: 최대 RT 성능을 위해 필요

---

## [4.0.0] - 2026-03-02

### 추가 (Added)
- **E-STOP 시스템**: 로봇/핸드 데이터 타임아웃 자동 감지 및 비상 정지 기능
  - `robot_timeout_ms` (기본 100ms), `hand_timeout_ms` (기본 200ms) 파라미터
  - E-STOP 발생 시 `/system/estop_status` 토픽 퍼블리시
  - 타임아웃 감시 타이머 (50Hz, `check_timeouts()`)
- **PDController E-STOP 지원** (`pd_controller.hpp`): `trigger_estop()`, `clear_estop()`, 안전 위치 복귀
- **핸드 E-STOP 분리**: 핸드 데이터 타임아웃 시 로봇은 유지, 핸드 명령만 차단
- **`hand_udp_receiver_node`** (`src/hand_udp_receiver_node.cpp`): UDP 패킷 수신 ROS2 노드
- **`hand_udp_sender_node`** (`src/hand_udp_sender_node.cpp`): UDP 명령 송신 ROS2 노드
- **`hand_udp.launch.py`**: 핸드 UDP 노드 전용 런치 파일 (`udp_port`, `target_ip`, `target_port` 인자)
- **`hand_udp_sender_example.py`** (`scripts/`): 핸드 시뮬레이터 예제 (사인파/고정 포즈 모드)
- **`monitor_data_health.py`** (`scripts/`): 실시간 데이터 헬스 모니터 + JSON 통계 내보내기
- **`install.sh`**: 자동 설치 스크립트 (의존성 설치 → 빌드 → 환경 설정)
- **표준 ROS2 디렉토리 구조** (`organize_files.sh`에 따라 재구성):
  - `config/`, `launch/`, `include/ur5e_rt_controller/controllers/`, `src/`, `scripts/`
  - `docs/`, `rviz/`, `test/`, `resources/` (빈 플레이스홀더)

### 변경 (Changed)
- **프로젝트 구조 전면 재편**: 루트에 산재했던 파일들을 ROS2 표준 레이아웃으로 이동

  | 이동 전 (루트) | 이동 후 |
  |---|---|
  | `ur5e_rt_controller.yaml` | `config/` |
  | `hand_udp_receiver.yaml` | `config/` |
  | `ur_control.launch.py` | `launch/` |
  | `hand_udp.launch.py` | `launch/` |
  | `rt_controller_interface.hpp` | `include/ur5e_rt_controller/` |
  | `data_logger.hpp` | `include/ur5e_rt_controller/` |
  | `hand_udp_receiver.hpp` | `include/ur5e_rt_controller/` |
  | `hand_udp_sender.hpp` | `include/ur5e_rt_controller/` |
  | `p_controller.hpp` | `include/ur5e_rt_controller/controllers/` |
  | `pd_controller.hpp` | `include/ur5e_rt_controller/controllers/` |
  | `custom_controller.cpp` | `src/` |
  | `hand_udp_receiver_node.cpp` | `src/` |
  | `hand_udp_sender_node.cpp` | `src/` |
  | `monitor_data_health.py` | `scripts/` |
  | `plot_ur_trajectory.py` | `scripts/` |
  | `hand_udp_sender_example.py` | `scripts/` |
  | `motion_editor_gui.py` | `scripts/` |

- **`ur_control.launch.py`**: 데이터 헬스 모니터 노드 추가, `use_fake_hardware` 인자 추가
- **`ur5e_rt_controller.yaml`**: E-STOP 설정 섹션 추가 (`estop:`, `safety:`, `logging:`)
- **`package.xml`**: 버전 3.x → 4.0.0, 설명 업데이트
- **`CMakeLists.txt`**: `hand_udp_receiver_node`, `hand_udp_sender_node` 빌드 타겟 추가
- **`README.md`**: 코드 심층 분석 기반으로 전면 재작성
  - 정확한 디렉토리 트리 (이동된 경로 기준)
  - ROS2 토픽 인터페이스 표 (구독/발행, 메시지 타입)
  - UDP 핸드 프로토콜 문서 (77 double / 616 bytes 패킷 형식)
  - 클래스별 파라미터 표 (`CustomController`, `PDController` 등)
  - E-STOP 동작 흐름 설명
  - 아키텍처 다이어그램 (ASCII)

---

## [3.0.0] - 이전 버전

### 추가 (Added)
- `PDController` 기본 구현 (`pd_controller.hpp`)
- `DataLogger` CSV 로깅 (`data_logger.hpp`)
- `HandUdpReceiver` / `HandUdpSender` UDP 인터페이스

### 변경 (Changed)
- `RTControllerInterface`에 `HandState` (11-DOF) 통합
- `ControllerState`에 `RobotState` + `HandState` 결합

---

## [2.0.0] - 이전 버전

### 추가 (Added)
- `DataLogger` CSV 로깅 기능
- 핸드 UDP 통합 기초 구조
- `motion_editor_gui.py` Qt5 모션 에디터 (50포즈)
- `plot_ur_trajectory.py` Matplotlib 시각화

---

## [1.0.0] - 초기 릴리스

### 추가 (Added)
- `RTControllerInterface` Strategy Pattern 기반 제어기 추상 클래스
- `PController` 비례 제어기 구현
- `CustomController` ROS2 노드 (기본 제어 루프)
- `ur_control.launch.py` 런치 파일
- `ur5e_rt_controller.yaml` 기본 설정
