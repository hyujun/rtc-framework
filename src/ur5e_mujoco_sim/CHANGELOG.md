# 변경 이력 — ur5e_mujoco_sim

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

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
