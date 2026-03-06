# 변경 이력 — ur5e_rt_base

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.2.2] - 2026-03-07

### 변경 (Changed) — 로깅 경로 동적 해석

- `data_logger.hpp`: 파일명에 타임스탬프 접미사 추가 (`ur5e_control_log_YYMMDD_HHMM.csv`)
- `log_dir` 파라미터에서 `~/` 를 절대경로로 자동 확장
- `max_log_files` 파라미터 지원 — N개 초과 시 가장 오래된 파일 자동 삭제
- ROS 2 Jazzy 지원 대응 및 문서 갱신


## [1.2.0] - 2026-03-05

### 추가 — 디지털 신호 필터 (`include/ur5e_rt_base/filters/`)

500Hz RT 루프에서 직접 사용 가능한 헤더-전용 필터 라이브러리를 추가했습니다.
두 필터 모두 `Init()` 이후 모든 처리 메서드가 `noexcept`이며, 힙 할당이 없습니다.

#### `filters/bessel_filter.hpp`

- **클래스**: `BesselFilterN<N>` — N채널 독립 4차 Bessel 저역통과 필터
- **구조**: 2개의 biquad (Direct Form II Transposed) 직렬 연결
- **아날로그 프로토타입**: 4차 Bessel, -3 dB 정규화
  - 켤레 쌍 1: ω₀=1.4301691433, Q=0.5219356105
  - 켤레 쌍 2: ω₀=1.6033574829, Q=0.8055342053
- **쌍선형 변환**: 컷오프 prewarping 포함 → 디지털 -3 dB 점이 정확히 `cutoff_hz`에 위치
- **API 요약**:

  | 메서드 | 설명 | RT 안전 |
  |--------|------|---------|
  | `Init(cutoff_hz, sample_rate_hz)` | 계수 계산, 상태 초기화 | 아니오 (생성 시만) |
  | `Apply(array<double,N>)` | N채널 필터링, 출력 반환 | **예** |
  | `ApplyScalar(double, channel)` | 단일 채널 스칼라 버전 | **예** |
  | `Reset()` | 지연 소자 0으로 초기화 | **예** |
  | `cutoff_hz()` / `sample_rate_hz()` | 파라미터 접근자 | **예** |

- **편의 별칭**: `BesselFilter6`, `BesselFilter11`, `BesselFilter1`

#### `filters/kalman_filter.hpp`

- **클래스**: `KalmanFilterN<N>` — N채널 독립 이산-시간 Kalman 필터
- **상태 모델**: 상수-속도 (Constant Velocity), 상태 `x = [pos, vel]ᵀ` (채널당 2×1)
- **전이 행렬**: `F = [[1, dt], [0, 1]]`
- **관측 행렬**: `H = [1, 0]` (위치만 측정)
- **공분산**: 2×2 행렬을 `{p00, p01, p11}` 스칼라 3개로 저장 (Eigen 의존성 없음)
- **API 요약**:

  | 메서드 | 설명 | RT 안전 |
  |--------|------|---------|
  | `Init(q_pos, q_vel, r, dt)` | 파라미터 설정, 상태 초기화 | 아니오 (생성 시만) |
  | `Init(Params)` | 구조체 버전 | 아니오 |
  | `Predict()` | 상태 예측 (매 틱) | **예** |
  | `Update(array<double,N>)` | 측정 융합, 필터 위치 반환 | **예** |
  | `PredictAndUpdate(array<double,N>)` | 예측+업데이트 단일 호출 | **예** |
  | `UpdateScalar(double, channel)` | 단일 채널 버전 | **예** |
  | `SetInitialPositions(array)` | 초기 상태 시드 | **예** |
  | `Reset()` | 전체 상태/공분산 초기화 | **예** |
  | `position(i)` / `velocity(i)` | 필터링된 위치/속도 | **예** |
  | `positions()` / `velocities()` | 전체 채널 배열 | **예** |
  | `kalman_gain(i)` | 마지막 칼만 이득 (진단용) | **예** |
  | `position_variance(i)` | 위치 불확실도 P₀₀ | **예** |

- **편의 별칭**: `KalmanFilter6`, `KalmanFilter11`, `KalmanFilter1`

---

## [1.1.0] - 2026-03-05

### 변경 — CPU 코어 할당 최적화

- **`thread_config.hpp`** `kUdpRecvConfig.cpu_core` 3 → **5**
  - `sensor_io`(Core 3)와 `udp_recv`의 코어 경합 제거
  - UDP 버스트 시 `JointStateCallback` 지연 위험 해소

- **`thread_config.hpp`** 8코어 전용 config 5종 추가:
  - `kRtControlConfig8Core` (Core 2, FIFO 90)
  - `kSensorConfig8Core` (Core 3, FIFO 70)
  - `kUdpRecvConfig8Core` (Core 4, FIFO 65) — 완전 전용 코어
  - `kLoggingConfig8Core` (Core 5, OTHER nice -5)
  - `kAuxConfig8Core` (Core 6, OTHER 0)

- **`thread_config.hpp`** `kUdpRecvConfig4Core` 추가 (Core 2, FIFO 65)
  - 4코어 폴백에서 `udp_recv`의 명시적 config 제공 (기존 암묵적 fallback 개선)

- **`thread_utils.hpp`** `SystemThreadConfigs` 구조체에 `udp_recv` 필드 추가

- **`thread_utils.hpp`** `SelectThreadConfigs()` 분기 개선:
  - 이전: ≥6코어 / <6코어 2단계
  - 이후: **≥8코어 / ≥6코어 / <6코어** 3단계 자동 선택

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
