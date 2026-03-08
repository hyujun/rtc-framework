# 변경 이력 — ur5e_rt_controller

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.3.0] - 2026-03-08

### 추가 (Added) — 런타임 컨트롤러 전환 및 동적 게인 설정

#### `src/custom_controller.cpp` — 컨트롤러 스위칭 구독자

- `/custom_controller/controller_type` (`std_msgs/Int32`) 구독자 추가
  - 0 = `PController`, 1 = `PDController`, 2 = `PinocchioController`, 3 = `ClikController`, 4 = `OperationalSpaceController`
  - 전환 시 현재 관절 위치로 신규 컨트롤러의 타겟 초기화 (과도 응답 방지)
- `/custom_controller/controller_gains` (`std_msgs/Float64MultiArray`) 구독자 추가
  - 컨트롤러별 게인 페이로드로 `set_gains()` 호출
  - 전환/런타임 중 모두 지원

#### `src/p_controller.cpp` — 단순 P 제어기 소스 추가

- `PController` (헤더 `p_controller.hpp`)의 구현 파일 분리
- 저수준 테스트 및 게인 곡선 확인 용도
- E-STOP 무시, 단순 비례 제어: `command[i] = Kp * e[i]`

#### Pinocchio 제어기 헤더 공통 변경

- `PinocchioController`, `ClikController`, `OperationalSpaceController` 헤더에 게인 수신 후 적용을 위한 `set_gains(const Gains&)` noexcept 메서드 정비
- 스레드 안전성: 게인 업데이트는 sensor 스레드에서, Compute()는 RT 스레드에서 — 게인 구조체는 `std::atomic` 또는 뮤텍스 없이 복사-on-read 방식으로 적용

---

## [5.2.2] - 2026-03-07

### 추가 (Added) — ROS 2 Jazzy 파라미터 파싱 호환성

- `config/ur5e_rt_controller.yaml`: `/**:` + `ros__parameters:` 플랫 구조로 변경
  - ROS 2 Jazzy에서 중첩 YAML 네임스페이스 파싱 오류 해결
  - `enable_logging`, `log_dir`, `max_log_files` 파라미터 추가
  - `log_dir: "~/ros2_ws/ur5e_ws/logging_data"` (launch 파일에서 절대경로로 확장)
  - `max_log_files: 10` (오래된 파일 자동 삭제)

### 변경 (Changed) — Pinocchio 제어기 URDF 경로

- `ur_description` 패키지 경로 → `ur5e_description` 패키지 경로
  - **이전**: `/opt/ros/humble/share/ur_description/urdf/ur5e.urdf`
  - **이후**: `$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf`
- `package.xml`: `<depend>ur5e_description</depend>` 추가


## [5.1.0] - 2026-03-05

### 추가

- **`config/cyclone_dds.xml`** — CycloneDDS 내부 스레드(recv/tev/gc/dq.builtins)를 Core 0-1로 제한하는 XML 설정
- **`scripts/setup_irq_affinity.sh`** — NIC IRQ를 Core 0-1로 고정하는 실행 스크립트 (RT 코어 2-5 보호)

### 변경

- **`launch/ur_control.launch.py`**:
  - `use_cpu_affinity` 인자 추가 (default: true) — 런치 3초 후 `ur_ros2_driver`를 Core 0-1에 taskset
  - `CYCLONEDDS_URI` 환경변수 자동 설정 (`config/cyclone_dds.xml` 경로)
  - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 자동 설정 (기존 수동 export 불필요)

---

## [5.0.0] - 2026-03-04

### 변경

- **멀티-패키지 분리**: 단일 패키지에서 5개 독립 패키지로 리팩터링
  - 공유 인프라(`types`, `thread_config`, `thread_utils`, `log_buffer`, `data_logger`) → `ur5e_rt_base` 패키지로 이동
  - UDP 브리지 노드 → `ur5e_hand_udp` 패키지로 이동
  - MuJoCo 시뮬레이터 → `ur5e_mujoco_sim` 패키지로 이동
  - Python 유틸리티 스크립트 → `ur5e_tools` 패키지로 이동
- `rt_controller_interface.hpp`: 타입 정의를 `ur5e_rt_base/types.hpp`에서 가져오도록 변경 (API 호환 유지)
- `custom_controller.cpp` 인클루드 경로 변경: `ur5e_rt_controller/` → `ur5e_rt_base/`
- `launch/ur_control.launch.py`: `monitor_data_health.py` 노드 패키지 → `ur5e_tools`
- `package.xml`: `<depend>ur5e_rt_base</depend>` 추가
- `CMakeLists.txt`: `find_package(ur5e_rt_base REQUIRED)` 추가, MuJoCo/UDP 빌드 제거

### 제거

- `include/ur5e_rt_controller/hand_udp_receiver.hpp`, `hand_udp_sender.hpp` → `ur5e_hand_udp`로 이동
- `include/ur5e_rt_controller/mujoco_simulator.hpp` → `ur5e_mujoco_sim`으로 이동
- `include/ur5e_rt_controller/thread_config.hpp`, `thread_utils.hpp` → `ur5e_rt_base`로 이동
- `include/ur5e_rt_controller/log_buffer.hpp`, `data_logger.hpp` → `ur5e_rt_base`로 이동
- `src/hand_udp_receiver_node.cpp`, `hand_udp_sender_node.cpp` → `ur5e_hand_udp`로 이동
- `src/mujoco_simulator_node.cpp` → `ur5e_mujoco_sim`으로 이동
- `launch/mujoco_sim.launch.py`, `launch/hand_udp.launch.py` → 각 패키지로 이동
- `scripts/` 디렉터리 → `ur5e_tools`로 이동
- `models/` 디렉터리 → `ur5e_mujoco_sim`으로 이동
- `config/mujoco_simulator.yaml`, `config/hand_udp_receiver.yaml` → 각 패키지로 이동

---

## [4.4.0] - 이전

### 추가

- **MuJoCo 시뮬레이터 통합**
  - `mujoco_simulator.hpp` — 스레드 안전 MuJoCo 3.x 물리 래퍼
  - `mujoco_simulator_node.cpp` — ROS2 노드 래퍼
  - `mujoco_sim.launch.py` — MuJoCo 시뮬레이션 실행 파일
  - 두 가지 시뮬레이션 모드: `free_run` (최대 속도), `sync_step` (1:1 동기)
  - GLFW 3D 뷰어 + RTF(실시간 비율) 오버레이
- **`ControllerTimingProfiler`** — 잠금-없는 Compute() 타이밍 프로파일러
  - 0–2000μs 히스토그램 (100μs 버킷), 릴렉스 아토믹
  - P95/P99 통계, 2ms 예산 임계값
- **`SpscLogBuffer`** — 단일-생산자/단일-소비자 링 버퍼 (512 엔트리)
  - `compute_time_us` LogEntry 필드 추가
- MuJoCo 손 시뮬레이션: 1차 필터 (100Hz, `hand_filter_alpha`)
- `config/mujoco_simulator.yaml` — MuJoCo 파라미터 + 오버라이드

---

## [4.3.0] - 이전

### 추가

- **Pinocchio 기반 컨트롤러 추가**
  - `PinocchioController` — PD + 중력/코리올리 보상 (RNEA)
  - `ClikController` — 폐루프 IK (감쇠 야코비안 유사역행렬, 영공간 관절 센터링)
  - `OperationalSpaceController` — 전체 6-DOF 데카르트 PD (SO(3) `log3()` 방향 오차)
- 모든 Eigen 작업 버퍼: 생성자에서 사전 할당 (500Hz 경로 힙 할당 제로)
- `#pragma GCC diagnostic push/pop` — Pinocchio 헤더 경고 억제

---

## [4.2.3] - 이전

### 수정

- RT 안전성 강화 9개 항목 (`planning_rt.md` 참조)
  - `noexcept` — 모든 가상 메서드 (500Hz 루프 예외 → 프로세스 종료)
  - `std::atomic<bool>` — E-STOP 플래그 (뮤텍스 오버헤드 제거)
  - `[[nodiscard]]` — 상태/계산 반환 함수 전체 적용
  - Eigen `noalias()` — 임시 객체 방지

---

## [4.2.0] - 이전

### 추가

- **멀티-실행기 아키텍처** — 4개 `SingleThreadedExecutor` + 4개 전용 스레드
  - `rt_executor`: Core 2, SCHED_FIFO/90 — 500Hz 제어 루프
  - `sensor_executor`: Core 3, SCHED_FIFO/70 — 센서 구독
  - `log_executor`: Core 4, SCHED_OTHER/nice-5 — CSV 로깅
  - `aux_executor`: Core 5, SCHED_OTHER/0 — E-STOP 퍼블리시
- `mlockall(MCL_CURRENT | MCL_FUTURE)` — 시작 시 페이지 폴트 방지
- CPU 친화성 (`pthread_setaffinity_np`) + 스케줄러 정책 적용
- `ApplyThreadConfig()` / `VerifyThreadConfig()` 유틸리티

### 변경

- 지터 10배 감소 (500μs → <50μs)
- E-STOP 응답 5배 향상 (100ms → <20ms)
- 컨텍스트 스위치 80% 감소 (5000/s → 1000/s)

---

## [1.0.0] - 초기

### 추가

- UR5e 500Hz 실시간 위치 제어기 초기 구현
- `PDController` — 관절 공간 PD + E-STOP
- `PController` — 단순 비례 제어기
- `HandUdpReceiver` / `HandUdpSender` — UDP 11-DOF 손 통신
- E-STOP 시스템: 50Hz 감시자, 100ms 타임아웃
- DataLogger: CSV 로깅 (`timestamp`, 위치, 목표, 명령)
- `ur_control.launch.py` — 전체 시스템 실행 파일
