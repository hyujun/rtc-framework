# 변경 이력 — ur5e_rt_base

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.13.0] - 2026-03-14

### 추가 (Added) — Role → 문자열 변환 유틸리티

- **`SubscribeRoleToString()` / `PublishRoleToString()`** (`types.hpp`)
  - `SubscribeRole` / `PublishRole` enum → 문자열 변환 inline 함수
  - ROS2 파라미터 이름 생성 및 디버그 로깅에 활용

---

## [5.12.0] - 2026-03-14

### 변경 (Changed) — 4-카테고리 토픽/CSV 체계

- **`SubscribeRole` enum** (`types.hpp`)
  - `kTarget` → `kGoal` rename (카테고리 1: Goal State)

- **`PublishRole` enum** (`types.hpp`)
  - `kTrajectoryState` 추가 (카테고리 4: 궤적 보간 상태)
  - `kControllerState` 추가 (카테고리 4: 제어기 내부 상태)

- **`LogEntry` 구조체** (`log_buffer.hpp`)
  - 4-카테고리별 필드 그루핑 (Goal → State → Command → Trajectory)
  - 신규 필드: `actual_torques[6]`, `actual_task_positions[6]`, `robot_commands[6]`, `command_type`
  - rename: `target_positions` → `trajectory_positions`, `target_velocities` → `trajectory_velocities`

- **`DataLogger` CSV 포맷** (`data_logger.hpp`)
  - robot_log: 31 → 49 컬럼 (torque, task_pos, command, command_type 추가)
  - hand_log: 컬럼 순서 4-카테고리별 재배치 (Goal → State → Sensor → Command)

---

## [5.10.0] - 2026-03-14

### 추가 (Added) — 세션 디렉토리 관리 유틸리티

- **`logging/session_dir.hpp` 신규** — 모든 C++ 패키지에서 공유하는 세션 디렉토리 헬퍼
  - `GenerateSessionTimestamp()` — `YYMMDD_HHMM` 형식 타임스탬프 생성
  - `EnsureSessionSubdirs()` — 표준 6개 서브디렉토리 생성 (controller, monitor, hand, sim, plots, motions)
  - `ResolveSessionDir()` — `UR5E_SESSION_DIR` 환경변수 우선, 폴백 시 자체 생성
  - `CleanupOldSessions()` — `max_sessions` 초과 시 오래된 세션 삭제
  - `ListSessionDirs()` — `YYMMDD_HHMM` 패턴 디렉토리 정렬 목록

---

## [5.8.0] - 2026-03-14

### 추가 (Added) — 타입, 로깅, 스레드 구성 확장

- **`RobotState`에 `torques` 추가** (`types.hpp`)
  - `std::array<double, 6> torques{}` — `/joint_states` effort 필드에서 복사

- **`LogEntry` 확장** (`log_buffer.hpp`)
  - 타이밍 필드: `t_state_acquire_us`, `t_compute_us`, `t_publish_us`, `t_total_us`, `jitter_us`
  - 핸드 상태 필드: `hand_positions[10]`, `hand_velocities[10]`, `hand_sensors[44]`, `hand_valid`

- **`DataLogger` CSV 포맷 확장** (`data_logger.hpp`)
  - 타이밍 컬럼 5개 + 핸드 상태 컬럼 65개 추가
  - 기존 `compute_time_us` → `t_compute_us`로 대체

- **모니터링 스레드 구성** (`thread_config.hpp`)
  - `kStatusMonitorConfig` / `kHandFailureConfig` (6-core: Core 4, SCHED_OTHER, nice -2)
  - `kStatusMonitorConfig8Core` / `kHandFailureConfig8Core` (8-core: Core 6)
  - `kStatusMonitorConfig4Core` / `kHandFailureConfig4Core` (4-core: Core 3)

- **`SystemThreadConfigs` 확장** (`thread_utils.hpp`)
  - 5 → 7 필드: `status_monitor`, `hand_failure` 추가
  - `ValidateSystemThreadConfigs()` 7개 스레드 검증
  - `SelectThreadConfigs()` 모든 코어 레이아웃에 모니터링 스레드 포함

---

## [5.7.0] - 2026-03-11

### 변경
- 워크스페이스 전체 버전 (v5.7.0) 통일

---

## [5.5.0] - 2026-03-09

### 변경 (Changed) — SPSC 링 버퍼 최적화

- `SpscLogBuffer`: 비트 AND 연산(`& (N-1)`)으로 인덱스 래핑 최적화
- `cached_tail_` / `cached_head_` 로컬 인덱스 캐싱으로 False Sharing 방지
- `alignas(kCacheLineSize)` 하드웨어 캐시 라인 크기 동적 결정

---

## [5.2.0] - 2026-03-07

### 추가 (Added) — 디지털 신호 필터 라이브러리

- `filters/bessel_filter.hpp`: 4차 Bessel 저역통과 필터 (N채널, noexcept, RT 안전)
  - 최대 선형 군지연 — 위상 왜곡 없는 궤적 평활화
  - 쌍선형 변환 + 컷오프 prewarping → 정확한 디지털 -3 dB 점
- `filters/kalman_filter.hpp`: 이산-시간 Kalman 필터 (N채널, 위치+속도 동시 추정)
  - 상수-속도 운동 모델, 2×2 공분산 스칼라 저장 (Eigen 의존성 없음)
  - `PredictAndUpdate()` 단일 호출 또는 `Predict()` / `Update()` 분리 호출 가능
- 타입 별칭: `BesselFilter6`, `BesselFilter11`, `KalmanFilter6`, `KalmanFilter11` 등

---

## [5.1.0] - 2026-03-07

### 변경 (Changed) — CPU 코어 할당 최적화

- `kUdpRecvConfig`: Core 3 → Core 5로 이동 (sensor_io 경합 방지)
- 8코어 시스템 레이아웃 추가 (`*8Core` 상수 5종)
- `SelectThreadConfigs()`: 런타임 CPU 수 자동 감지 → 최적 config 집합 반환

---

## [5.0.0] - 2026-03-07

### 추가

- **초기 분리**: `ur5e_rt_controller` 단일 패키지에서 공유 헤더를 독립 패키지로 추출
- `types/types.hpp`: `RobotState`, `HandState`, `ControllerState`, `ControllerOutput`, 컴파일-시간 상수
- `threading/thread_config.hpp`: `ThreadConfig` 구조체, 4/6코어 사전 정의 상수
- `threading/thread_utils.hpp`: `ApplyThreadConfig()`, `VerifyThreadConfig()`, `SelectThreadConfigs()`
- `logging/log_buffer.hpp`: SPSC 링 버퍼 (512 entries, 잠금-없음)
- `logging/data_logger.hpp`: 비-RT CSV 로거 (`DrainAndWrite()`)
- `udp/udp_socket.hpp`, `udp_codec.hpp`, `udp_transceiver.hpp`: UDP 통신 인프라
