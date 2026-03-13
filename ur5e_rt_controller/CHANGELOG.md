# 변경 이력 — ur5e_rt_controller

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

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
