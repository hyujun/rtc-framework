# 변경 이력 — ur5e_rt_controller

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.17.0] - 2026-03-19

### 변경 (Changed) — 컨트롤러 이름 변경 및 신규 추가

- **`UrFiveEHandController` → `DemoJointController` 이름 변경 (index 4)**
  - 클래스명, 파일명, YAML config key 일괄 변경
  - 기능 변경 없음 (로봇 암 P + 핸드 P 통합 제어)

- **`DemoTaskController` 신규 추가 (index 5)**
  - ClikController의 task-space 제어 (감쇠 야코비안 유사역행렬 + 영공간 관절 센터링)
  - DemoJointController의 핸드 P 제어 (10모터)를 결합
  - 3-DOF / 6-DOF 모드, SE(3) 5차 궤적 추종, E-STOP 지원
  - `UpdateGainsFromMsg` 레이아웃: `[kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1), hand_kp×10]` (20개)
  - YAML: `demo_task_controller.yaml` (enable_ur5e + enable_hand = true)

- **Controller GUI 업데이트**: 인덱스 5 (Demo Task Controller) 추가, 핸드 활성화 조건 확장

---

## [5.16.0] - 2026-03-17

### 변경 (Changed) — RT Loop 아키텍처: clock_nanosleep + SPSC Publish Offload

- **RT 제어 루프를 ROS2 executor에서 전용 jthread로 이동**
  - `create_wall_timer(2ms, ControlLoop(), cb_group_rt_)` 제거
  - `RtLoopEntry()` jthread: `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 기반 500Hz 절대시간 루프
  - `CheckTimeouts()` 매 10틱(50Hz) inline 호출 — `timeout_timer_` 제거
  - Executor epoll 디스패치 지터 ~50-200μs 제거 → 타이머 정밀도 < 10μs

- **Publish 작업을 별도 jthread + SPSC 버퍼로 오프로드**
  - 신규: `ControlPublishBuffer` (SPSC 링 버퍼, 512 slots) — `publish_buffer.hpp`
  - 신규: `PublishLoopEntry()` jthread — SPSC drain → 모든 `publish()` 호출
  - ControlLoop() Phase 3: `publish()` 호출 전체를 SPSC push 1회로 교체
  - DDS 직렬화, `sendto()` 시스템 콜, `std::string` 할당이 RT 경로에서 완전 제거
  - `cmd_pub_mutex_` 제거 (SPSC single-consumer로 대체)

- **Overrun recovery (clock_nanosleep burst 방지)**
  - 놓친 tick 감지 → skip + 다음 정상 주기 경계로 재정렬 (burst 실행 방지)
  - `consecutive_overruns_` ≥ 10 → `TriggerGlobalEstop("consecutive_overrun")`
  - 신규 카운터 분리: `overrun_count_` (RT loop level), `compute_overrun_count_` (ControlLoop level), `skip_count_` (누적 skip tick)

- **Executor 4개 → 3개로 축소**
  - 제거: `rt_executor`, `cb_group_rt_` (→ `RtLoopEntry` jthread가 대체)
  - 유지: `sensor_executor`, `log_executor`, `aux_executor`
  - 신규: `publish_thread_` jthread (Core 5, SCHED_OTHER nice -3)

- **API 변경 (RtControllerNode)**
  - 제거: `GetRtGroup()`, `control_timer_`, `timeout_timer_`, `cmd_pub_mutex_`
  - 추가: `StartRtLoop()`, `StopRtLoop()`, `StartPublishLoop()`, `StopPublishLoop()`
  - 타입 변경: `last_robot_update_` → `steady_clock::time_point` (rclcpp::Time에서 변경)
  - `CheckTimeouts()`: `now()` → `steady_clock::now()`, `lock_guard` → `try_to_lock`

- **스레드 구성 추가 (`thread_config.hpp`)**
  - `kPublishConfig` 상수: 4/6/8/10/12/16-core 전체 레이아웃에 추가
  - `SystemThreadConfigs.publish` 필드 추가
  - `ValidateSystemThreadConfigs()`, `SelectThreadConfigs()` 업데이트

### 추가 (Added)

- `ur5e_rt_base/include/ur5e_rt_base/threading/publish_buffer.hpp`
  - `PublishSnapshot` 구조체: robot/hand commands, state, flags (~640B)
  - `SpscPublishBuffer<N>`: lock-free SPSC 링 버퍼 (log_buffer.hpp 패턴 동일)

### 성능 개선

| 메트릭 | v5.13.0 (executor) | v5.16.0 (clock_nanosleep) |
|--------|-------------------|--------------------------|
| 제어 주기 지터 | ~50-200μs | < 10μs (목표) |
| publish 오버헤드 (RT 경로) | 60-200μs | 0 (SPSC push ~2μs) |
| Context switch 감소 | — | executor dispatch 제거 |

---

## [5.13.0] - 2026-03-14

### 추가 (Added) — ROS2 Parameter Exposure + Topic Remapping 지원

- **토픽 구성 ROS2 파라미터 노출 (읽기 전용)**
  - `ExposeTopicParameters()`: 모든 컨트롤러의 토픽 매핑을 ROS2 파라미터로 노출
  - 파라미터 명명: `controllers.<ctrl_name>.subscribe.<role>`, `controllers.<ctrl_name>.publish.<role>`
  - `ros2 param list/get`으로 런타임 토픽 구성 확인 가능
  - `add_on_set_parameters_callback`으로 런타임 변경 차단 (RT 안전성 유지)

- **ROS2 Topic Remapping 지원 문서화**
  - `--ros-args -r /old_topic:=/new_topic` 네이티브 지원 확인 및 문서화
  - `create_publisher()`/`create_subscription()` 사용으로 자동 호환

---

## [5.12.0] - 2026-03-14

### 추가 (Added) — 4-카테고리 토픽/CSV 체계

- **토픽 Role 재구성 (4-카테고리)**
  - 카테고리 1 — Goal State: `SubscribeRole::kTarget` → `SubscribeRole::kGoal` rename (YAML `"target"` 하위 호환 유지)
  - 카테고리 3 — Control Command: 기존 유지 (`position_command`, `torque_command`, `hand_command`)
  - 카테고리 4 — Logging/Monitoring: `PublishRole::kTrajectoryState`, `PublishRole::kControllerState` 신규 추가

- **신규 모니터링 토픽 2개**
  - `/rt_controller/trajectory_state` (Float64MultiArray, size=18): goal[6] + traj_pos[6] + traj_vel[6]
  - `/rt_controller/controller_state` (Float64MultiArray, size=18): actual_pos[6] + actual_vel[6] + command[6]

- **robot_log.csv 확장 (31 → 49 컬럼)**
  - 신규: `actual_torque_0..5`, `task_pos_0..5`, `command_0..5`, `command_type`
  - rename: `target_pos_*` → `traj_pos_*`, `target_vel_*` → `traj_vel_*`
  - 컬럼 순서: 4-카테고리별 그루핑 (Goal → State → Command → Trajectory)

- **hand_log.csv 컬럼 재배치**
  - 카테고리별 순서: Goal → Current State → Sensor → Command

### 변경 (Changed)

- 모든 컨트롤러 YAML `topics.subscribe` role: `"target"` → `"goal"` (하위 호환 유지)
- 모든 컨트롤러 YAML `topics.publish`에 `trajectory_state`, `controller_state` 토픽 추가
- `ControlLoop()` Phase 3: `kTrajectoryState`, `kControllerState` publish case 추가
- `LogEntry` 채우기: `actual_torques`, `actual_task_positions`, `robot_commands`, `command_type` 추가

---

## [5.11.0] - 2026-03-14

### 변경 (Changed) — HandController sensor_decimation 파라미터 전달

- `DeclareAndLoadParameters()`에 `sensor_decimation` 파라미터 추가 (기본값 1)
- `HandController` 생성 시 `sensor_decimation` 값 전달
- `hand_udp_node.yaml`에서 설정된 값을 launch 파일을 통해 rt_controller_node에 로드

---

## [5.10.0] - 2026-03-14

### 추가 (Added) — 세션 기반 통합 로깅

- **세션 디렉토리 구조 (`logging_data/YYMMDD_HHMM/`)**
  - 모든 로그 파일을 `controller/` 서브디렉토리에 통합
  - `timing_log.csv`, `robot_log.csv`, `hand_log.csv` — 파일 이름에서 타임스탬프 제거 (세션 폴더가 타임스탬프 역할)

- **`UR5E_SESSION_DIR` 환경변수 연동**
  - Launch 파일이 설정한 `UR5E_SESSION_DIR` 자동 감지
  - 폴백: 환경변수 미설정 시 `log_dir` 파라미터 → 자체 세션 생성

- **`session_dir.hpp` 공유 유틸리티 사용** (`ur5e_rt_base`)
  - `ResolveSessionDir()`, `CleanupOldSessions()`, `EnsureSessionSubdirs()` 활용

### 변경 (Changed)

- `log_dir` 기본값: `"~/ros2_ws/ur5e_ws/logging_data"` → `""` (launch 파일이 세션 경로 설정)
- `max_log_files` → `max_log_sessions` 파라미터 이름 변경 (세션 폴더 단위 관리)
- `GenerateLogTimestamp()`, `CleanupOldLogFiles()` 제거 → `session_dir.hpp` 유틸리티로 대체
- CSV 로그 경로: `logging_data/{type}_log_YYMMDD_HHMM.csv` → `logging_data/YYMMDD_HHMM/controller/{type}_log.csv`
- `ur_control.launch.py`: 세션 디렉토리 생성, `UR5E_SESSION_DIR` 환경변수 설정, `max_log_sessions` 지원

---

## [5.8.0] - 2026-03-14

### 추가 (Added) — RT 안전성 및 모니터링 강화

- **M1: 제어 루프 단계별 타이밍 계측**
  - `LogEntry`에 `t_state_acquire_us`, `t_compute_us`, `t_publish_us`, `t_total_us`, `jitter_us` 필드 추가
  - `ControlLoop()`에서 `steady_clock` 기반 4단계 타이밍 측정
  - CSV 로그에 타이밍 컬럼 자동 기록

- **C2: ControlLoop 벽시계 오버런 감지**
  - `budget_us_` (= 1e6 / control_rate) 초과 시 `overrun_count_` 원자적 증가
  - 타이밍 요약 로그에 오버런 횟수 출력

- **C4: 궤적 레이스 컨디션 수정 (try_lock 패턴)**
  - `JointPDController`, `ClikController`, `OperationalSpaceController`에 `target_mutex_` 추가
  - `SetRobotTarget()`: `lock_guard` 보호, `Compute()`: `try_to_lock` (실패 시 다음 틱에서 처리)
  - RT 스레드 블로킹 없이 `SetRobotTarget()`/`Compute()` 동시 접근 방지

- **H3: 글로벌 E-Stop 플래그 및 전파**
  - `std::atomic<bool> global_estop_` + `TriggerGlobalEstop(reason)` 메서드
  - 모든 서브시스템(컨트롤러, 핸드, 상태 모니터)에 E-Stop 전파
  - E-Stop 사유 및 타임스탬프 로그 기록

- **H1: UR5e 상태 모니터 통합**
  - `ur5e_status_monitor` 패키지를 `RtControllerNode`에 컴포지션
  - YAML `enable_status_monitor: true`로 활성화 (기본 false)
  - 10Hz `std::jthread`로 안전 상태, 추적 오차, 관절 한계 모니터링
  - 실패 감지 시 `TriggerGlobalEstop()` 콜백 연결

- **H2: 핸드 실패 감지기 (C++ HandFailureDetector)**
  - 50Hz `std::jthread`로 모터/센서 데이터 이상 감지
  - 전-영점 데이터 × N회 연속, 중복 데이터 × N회 연속 감지
  - YAML `failure_detector.enable: true`로 활성화

- **H4: 초기화 타임아웃**
  - `init_timeout_sec` (기본 5.0초) 내 로봇/핸드 데이터 미수신 시 E-Stop + 종료
  - `init_wait_ticks_` 카운터 기반 RT-safe 구현

- **H5: 관절 토크 데이터 (`RobotState.torques`)**
  - `RobotState`에 `std::array<double, 6> torques{}` 추가
  - `/joint_states` effort 필드에서 토크 복사 (가용 시)

- **M2: 핸드 상태 CSV 로깅**
  - `LogEntry`에 `hand_positions[10]`, `hand_velocities[10]`, `hand_sensors[44]`, `hand_valid` 추가
  - CSV 헤더 및 데이터 행에 핸드 상태 자동 기록

### 변경 (Changed)

- `CheckTimeouts()` → `TriggerGlobalEstop()` 호출로 변경 (기존 개별 E-Stop 대신 글로벌)
- `DeclareAndLoadParameters()`에 `init_timeout_sec`, `enable_status_monitor` 파라미터 추가
- `CMakeLists.txt`에 `ur5e_status_monitor` 의존성 추가

---

## [5.7.0] - 2026-03-11

### 수정 (Fixed) — PController 정상상태 오차 제거

- **원인**: `command = kp * error`를 절대 위치 명령으로 전송 → MuJoCo 서보가 `ctrl=kp*error` 위치로 구동
  - 평형: `q* = kp/(kp+1) * target` (예: kp=1 → target 90° 시 45°에 정착)
- **수정**: 증분 위치 스텝 방식으로 변경
  ```cpp
  output.robot_commands[i] = state.robot.positions[i]
                            + gains_.kp[i] * error * state.robot.dt;
  ```
- **평형**: `q* = target` (정상상태 오차 없음)

---

## [5.6.1] - 2026-03-10

### 변경 (Changed) — 궤적 생성 개선

- `PinocchioController`에 `JointSpaceTrajectory<6>` 추가 (직접 점프 → 5차 관절공간 궤적, `trajectory_speed` 파라미터)
- `OperationalSpaceController`에 `TaskSpaceTrajectory` 추가 (SE(3) 5차 스플라인, `trajectory_speed` + `trajectory_angular_speed`)
- `ClikController`에서 미사용 `trajectory_angular_speed` 제거 (dead-code)

---

## [5.5.0] - 2026-03-09

### 변경 (Changed) — 소스 분할 및 클래스명 변경

- `rt_controller.cpp` → `rt_controller_node.hpp` + `rt_controller_node.cpp` + `rt_controller_main.cpp` 3분할
- 클래스명 변경: `CustomController` → `RtControllerNode`
- `SpscLogBuffer` 비트 AND 연산 최적화

---

## [5.4.0] - 2026-03-09

### 추가 (Added) — Controller Registry 패턴

- `MakeControllerEntries()` 팩토리 리스트: 새 컨트롤러 등록 시 한 줄만 추가
- `RTControllerInterface`에 `LoadConfig()` / `UpdateGainsFromMsg()` 훅 추가
- 컨트롤러별 YAML 로딩·게인 업데이트를 각 컨트롤러 자신이 담당
- 기존 `switch`/`dynamic_cast` 코드 제거

---

## [5.3.0] - 2026-03-08

### 추가 (Added) — 런타임 컨트롤러 전환

- P/PD/Pinocchio/CLIK/OSC 컨트롤러를 `/rt_controller/controller_type` 토픽으로 런타임 전환
- `/rt_controller/controller_gains` 토픽으로 동적 게인 업데이트
- 5차 다항식 궤적 생성 서브시스템 (`trajectory/`): `QuinticPolynomial`, `TaskSpaceTrajectory`, `JointSpaceTrajectory<N>`

---

## [5.0.0] - 2026-03-07

### 추가

- **초기 분리**: 단일 패키지에서 500Hz RT 제어기를 독립 패키지로 추출
- SCHED_FIFO 멀티스레드 아키텍처 (4 executor, CPU 코어 할당)
- 전략 패턴 기반 컨트롤러: `PDController`, `PController`, `PinocchioController`, `ClikController`, `OperationalSpaceController`
- E-STOP 안전 시스템 (50Hz 워치독)
- 잠금-없는 `SpscLogBuffer` + `DataLogger` CSV 로깅
- `ControllerTimingProfiler` 잠금-없는 타이밍 프로파일러
