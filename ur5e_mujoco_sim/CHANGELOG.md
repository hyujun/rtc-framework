# 변경 이력 — ur5e_mujoco_sim

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [Unreleased]

### 변경 (Changed) — 문서

- README.md: `/forward_torque_controller/commands` 토픽, effort 필드, QoS 설명 추가
- README.md: 설정 예시 `sim_mode` 기본값을 `"sync_step"`으로 수정 (실제 YAML과 일치)
- README.md: 존재하지 않는 API (`GetStats()`, `GetStateSnapshot()`) 제거, 실제 접근자로 대체
- README.md: 누락 API 메서드 추가 (제어 모드 전환, 일시정지, solver 통계 등)
- README.md: hand 시뮬레이션 10-DOF로 수정 (기존 11개 → 10개)

---

## [5.7.0] - 2026-03-11

### 추가 (Added) — MuJoCo Position Servo 게인 시스템

- `physics_timestep` 파라미터 — XML `<option timestep>`과 비교 검증 (불일치 시 ERROR + XML 우선)
- `use_yaml_servo_gains` / `servo_kp` / `servo_kd` — YAML 기반 position servo 게인 적용
  - `gainprm = servo_kp / physics_timestep`, `force = servo_kp * dq_cmd - servo_kd * dq_actual`
- `PreparePhysicsStep()`: torque / YAML servo / XML servo 3-way 분기
- Gravity 자동 잠금: position servo 진입 시 OFF + 잠금, torque 모드 전환 시 해제 + ON
- `mujoco_sim.launch.py`에 `use_yaml_servo_gains` launch argument 추가
- `/forward_torque_controller/commands` 토크 명령 구독 — 수신 시 자동으로 torque 모드 전환
- `SetControlMode(bool)` API — position servo ↔ torque 모드 전환 (gravity 자동 관리)
- `EnforcePositionServoGravity()` — 위치 명령 수신마다 gravity OFF + lock 재확인 (경량 메서드)
- `/joint_states` effort 필드 퍼블리시 (`qfrc_actuator` — 액추에이터 토크 Nm)
- 위치/토크 명령 구독 QoS를 `BEST_EFFORT`로 설정 (rt_controller 퍼블리셔와 일치)

### 변경 (Changed)

- `mujoco_simulator.yaml` rt_controller 섹션 정리: 중복 파라미터(`control_rate`, `kp`, `kd`, `enable_logging`) 제거, 시뮬 전용 차이값만 유지

### 수정 (Fixed)

- Gravity lock QoS 불일치: rt_controller(`BEST_EFFORT`) ↔ simulator(`RELIABLE`) 간 QoS 불일치로 position command 미수신 → gravity lock 미동작 문제 해결
- Position servo 모드에서 gravity OFF 강제 재적용 (viewer `G` 키 또는 race condition으로 활성화되는 경우 방어)
- `gravity_enabled_` 초기값을 `false`로 수정하여 초기 position servo 모드와 일치

---

## [5.6.2] - 2026-03-10

### 변경 (Changed) — 시뮬레이션 파라미터 우선순위 개선

- C++ `Config`에서 하드코딩된 기본값(Euler/Newton) 제거
- MuJoCo XML에 네이티브로 지정된 물리 solver 값이 우선 적용
- XML에 옵션이 없으면 `mj_loadXML` 내부 기본값이 안전하게 적용

---

## [5.6.1] - 2026-03-10

### 변경 (Changed)

- 워크스페이스 전체 버전 (v5.6.1) 통일

---

## [5.6.0] - 2026-03-10

### 추가 (Added) — MuJoCo 뷰어 전면 확장

- `src/viewer/` 4-파일 구조로 재편: `viewer_state.hpp`, `viewer_loop.cpp`, `viewer_callbacks.cpp`, `viewer_overlays.cpp`
- 2페이지 F1 도움말, 카메라 3모드 (Free/Tracking/Fixed), 마우스 더블클릭 물체 선택
- Ctrl+드래그 힘/토크 인가, 지오메트리 그룹 0-5, 시각화 플래그 12종, 렌더링 플래그 4종
- F9 센서 오버레이, F10 모델 통계 오버레이, P 스크린샷
- `StepOnce()` / `GetSimMode()` API 추가
- 전역 폰트 `mjFONTSCALE_100`으로 축소

---

## [5.3.0] - 2026-03-08

### 추가 (Added) — MuJoCo ROS2 Resource Provider

MJCF 파일 내 `package://` URI를 MuJoCo가 직접 해석할 수 있도록 전역 resource provider를 등록합니다.

#### 신규 파일

| 파일 | 역할 |
|------|------|
| `include/ur5e_mujoco_sim/ros2_resource_provider.hpp` | `RegisterRos2ResourceProvider()` 함수 선언 |
| `src/ros2_resource_provider.cpp` | `mjp_addResourceProvider()` 기반 구현 |

#### 동작 방식

1. `MuJoCoSimulatorNode` 생성 시 `ur5e_rt_controller::RegisterRos2ResourceProvider()` 자동 호출
2. MuJoCo가 파일 로드 시 `package://<pkg>/<path>` 경로를 감지하면 `ament_index_cpp::get_package_share_directory()` 로 절대경로 변환
3. 이후 `mj_loadXML()` / `mj_loadModel()`이 변환된 경로로 파일 접근

#### 이점

- MJCF `<mesh file="package://ur5e_description/robots/ur5e/meshes/..."/>` 직접 사용 가능
- `ur5e_description` 이외 패키지 메시도 동일 방식으로 참조 가능
- 하드코딩 절대경로 의존성 제거 → 워크스페이스 이동/배포 환경에서 자동 해석

#### CMakeLists.txt 변경

- `ros2_resource_provider.cpp` → `mujoco_simulator_node` 빌드 대상 추가
- `ament_index_cpp` 의존성 명시 (`find_package` + `target_link_libraries`)

### 변경 (Changed) — 뷰어 개선 (`mujoco_viewer.cpp`)

- `mujoco_viewer.cpp` 렌더링 루프 안정성 개선 (GLFW 창 소멸 전 뷰어 스레드 경쟁 조건 방지)
- 컨텍스트 전환 타이밍 최적화

---

## [5.2.2] - 2026-03-07

### 추가 (Added) — 소스 파일 분리 (관심사 분리)

단일 `mujoco_simulator_node.cpp`에서 역할별 4개 파일로 분리:

| 파일 | 역할 |
|------|------|
| `mujoco_simulator.cpp` | 생명주기 (`ctor/dtor`, `Initialize`, `Start`, `Stop`), I/O |
| `mujoco_sim_loop.cpp` | `SimLoopFreeRun` / `SimLoopSyncStep` + 물리 헬퍼 함수 |
| `mujoco_viewer.cpp` | `ViewerLoop`: GLFW 렌더링, 키보드/마우스, 오버레이 |
| `mujoco_simulator_node.cpp` | ROS2 노드 래퍼만 (`MuJoCoSimulatorNode`) |

### 변경 (Changed) — 기본 MJCF 모델 경로

- **이전**: `get_package_share_directory("ur5e_mujoco_sim") + "/models/ur5e/scene.xml"`
- **이후**: `get_package_share_directory("ur5e_description") + "/robots/ur5e/mjcf/scene.xml"`
- 모델 파일이 `ur5e_description` 패키지로 이동 (단일 소스 관리)
- 빌드 전 `ur5e_description` 패키지 선행 빌드 필요

### 수정 (Fixed) — `solver_niter` `int*` 타입 오류

- `ReadSolverStats()`에서 `data_->nisland`만큼 반복하여 `solver_niter[k]` 합산
- `nisland > 0` 가드 추가 (빈 씬 크래시 방지)


## [1.0.1] - 2026-03-06

### 수정 (Fixed) — `solver_niter` `int*` 타입 역참조 오류

`mujoco_sim_loop.cpp` `ReadSolverStats()` 에서 `data_->solver_niter`를 `int`에 직접 대입하던 오류를 수정합니다.

**원인**: MuJoCo 3.x에서 `mjData::solver_niter`는 constraint island별 반복 횟수를 담는 **`int*`** 배열입니다 (`data_->nisland` 크기).

**수정 내용**:
- `data_->nisland` 만큼 반복하며 `solver_niter[k]`를 합산 → `SolverStats::iter`에 저장
- `solver[0]` 통계 접근 전 `nisland > 0` 가드 추가 (빈 씬 방어)

**영향 범위**:
- UR5e 단독 씬 (`nisland == 1`): 동작 결과 동일
- 다중 물체 씬 / MuJoCo Menagerie 모델: 모든 island 합산으로 올바른 반복 횟수 반영
- `GetSolverStats().iter` 반환값 및 F4 오버레이 표시값 정확도 향상

---

## [1.0.0] - 2026-03-04

### 추가

- **초기 분리**: `ur5e_rt_controller` 단일 패키지에서 MuJoCo 시뮬레이터를 독립 패키지로 추출
- `include/ur5e_mujoco_sim/mujoco_simulator.hpp` — 스레드 안전 MuJoCo 3.x 물리 래퍼
  - 두 가지 시뮬레이션 모드: `kFreeRun` (최대 속도), `kSyncStep` (명령 동기 스텝)
  - `SimLoop` 스레드 (`jthread`) — 물리 시뮬레이션 실행
  - `ViewerLoop` 스레드 (`jthread`, 선택적) — GLFW 3D 뷰어 ~60Hz
  - RTF(실시간 비율) 측정 및 뷰어 오버레이 표시
  - 동기화: `cmd_mutex_` + `cmd_pending_` (atomic), `sync_cv_`, `state_mutex_`, `viz_mutex_`
  - `viz_mutex_`: `try_lock` 전용 — SimLoop 절대 블로킹 없음
- `src/mujoco_simulator_node.cpp` — `mujoco_simulator_node` ROS2 노드
  - `/joint_states` 퍼블리시 (물리 속도 또는 데시메이션)
  - `/hand/joint_states` 퍼블리시 (100Hz, 1차 필터 `hand_filter_alpha`)
  - `/sim/status` 퍼블리시 (`[step_count, sim_time_sec, rtf]`)
  - `/forward_position_controller/commands` 구독
  - `/hand/command` 구독
  - `get_package_share_directory("ur5e_mujoco_sim")` — 패키지 내 모델 경로 참조
- `models/ur5e/scene.xml` — MuJoCo 씬 (지면 + UR5e 포함)
- `models/ur5e/ur5e.xml` — UR5e MJCF 로봇 모델
- `launch/mujoco_sim.launch.py` — 시뮬레이션 실행 파일
  - `model_path`, `sim_mode`, `enable_viewer`, `publish_decimation`, `sync_timeout_ms`, `max_rtf`, `kp`, `kd` 파라미터
  - `mujoco_simulator_node` (패키지: `ur5e_mujoco_sim`)
  - `custom_controller` (패키지: `ur5e_rt_controller`, E-STOP 비활성 오버라이드)
  - `monitor_data_health.py` (패키지: `ur5e_tools`)
- `config/mujoco_simulator.yaml` — 시뮬레이터 파라미터 + `custom_controller` 오버라이드
- `CMakeLists.txt` — MuJoCo/GLFW 선택적 의존성, 미설치 시 자동 건너뜀
- `package.xml` — `rclcpp`, `std_msgs`, `sensor_msgs`, `ament_index_cpp` 의존성

### 변경

- 인클루드 경로 변경:
  - `#include "ur5e_rt_controller/mujoco_simulator.hpp"` → `#include "ur5e_mujoco_sim/mujoco_simulator.hpp"`
- 헤더 가드 변경: `UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_` → `UR5E_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_`
- 패키지 공유 디렉터리 참조 변경: `get_package_share_directory("ur5e_rt_controller")` → `get_package_share_directory("ur5e_mujoco_sim")`
- launch 파일 내 패키지 참조 분리: `pkg_sim` (ur5e_mujoco_sim) / `pkg_ctrl` (ur5e_rt_controller)

### 참고

이 패키지는 다음 v4.4.0 `ur5e_rt_controller` 파일에서 추출되었습니다:
- `include/ur5e_rt_controller/mujoco_simulator.hpp`
- `src/mujoco_simulator_node.cpp`
- `launch/mujoco_sim.launch.py`
- `config/mujoco_simulator.yaml`
- `models/ur5e/` (전체)

`mujoco_simulator.hpp`는 다른 패키지 헤더를 포함하지 않으므로 (MuJoCo/GLFW/stdlib만 사용) 완전히 독립적입니다.
