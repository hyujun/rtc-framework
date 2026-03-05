# 변경 이력 — ur5e_rt_base

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [1.0.0] - 2026-03-04

### 추가

- **초기 분리**: `ur5e_rt_controller` 단일 패키지에서 공유 인프라를 독립 헤더-전용 패키지로 추출
- `include/ur5e_rt_base/types.hpp` — 공유 데이터 구조체 및 컴파일-시간 상수
  - `kNumRobotJoints = 6`, `kNumHandJoints = 11`, `kNumHandSensors = 44`
  - `RobotState`, `HandState`, `ControllerState`, `ControllerOutput` 구조체
  - `NonNegativeFloat` C++20 컨셉
- `include/ur5e_rt_base/thread_config.hpp` — `ThreadConfig` 구조체 및 사전 정의 RT 스레드 상수
  - 6코어용: `kRtControlConfig` (FIFO/90), `kSensorConfig` (FIFO/70), `kUdpRecvConfig` (FIFO/65), `kLoggingConfig` (OTHER/nice-5), `kAuxConfig` (OTHER/0)
  - 4코어 대체 상수: `kRtControlConfig4Core`, `kSensorConfig4Core`, `kLoggingConfig4Core`
- `include/ur5e_rt_base/thread_utils.hpp` — `ApplyThreadConfig()`, `VerifyThreadConfig()` 함수
  - `pthread_setaffinity_np`, `pthread_setschedparam`, `pthread_setname_np` 통합
  - 권한 부족 시 `false` 반환 (노드는 SCHED_OTHER로 계속 동작)
- `include/ur5e_rt_base/log_buffer.hpp` — `SpscLogBuffer<T, N>` SPSC 링 버퍼
  - 단일-생산자/단일-소비자, 잠금-없음, 512 엔트리 기본값
  - RT 경로에서 절대 블로킹/동적 할당 없음 (`Push()` noexcept)
  - `LogEntry` 구조체 (`compute_time_us` 포함)
- `include/ur5e_rt_base/data_logger.hpp` — 비-RT CSV 로거 (`DataLogger`)
  - 이동 전용 (복사 불가)
  - `DrainAndWrite()` — SpscLogBuffer에서 드레인하여 CSV 기록
- `CMakeLists.txt` — 헤더-전용 패키지 설정 (`install(DIRECTORY include/)`, `ament_export_include_directories`)
- `package.xml` — `ament_cmake` 빌드 의존성만 포함 (최소 의존성)

### 참고

이 패키지는 다음 v4.4.0 `ur5e_rt_controller` 헤더에서 추출되었습니다:
- `include/ur5e_rt_controller/rt_controller_interface.hpp` → `types.hpp` (타입 부분만)
- `include/ur5e_rt_controller/thread_config.hpp` → `thread_config.hpp`
- `include/ur5e_rt_controller/thread_utils.hpp` → `thread_utils.hpp`
- `include/ur5e_rt_controller/log_buffer.hpp` → `log_buffer.hpp`
- `include/ur5e_rt_controller/data_logger.hpp` → `data_logger.hpp`

모든 코드는 기능 변경 없이 네임스페이스(`ur5e_rt_controller`)를 유지한 채 이동되었습니다.
