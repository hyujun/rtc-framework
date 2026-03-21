# rtc_base

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **헤더 전용(header-only) 실시간 인프라 라이브러리**입니다. 500 Hz 결정론적 제어 루프에서 안전하게 사용할 수 있도록 설계되었으며, 공유 데이터 타입, 락-프리 동기화 프리미티브, 신호 처리 필터, 스레드 구성, 로깅 인프라를 제공합니다.

**핵심 설계 원칙:**
- 헤더 전용 — 링크 타임 의존성 없음
- RT 경로에서 힙 할당 및 시스템 콜 금지
- 모든 RT-safe 함수에 `noexcept` 보장
- 캐시 라인 정렬로 false sharing 방지

---

## 패키지 구조

```
rtc_base/
├── CMakeLists.txt
├── package.xml
└── include/rtc_base/
    ├── types/
    │   └── types.hpp              ← 공유 데이터 타입, 상수, 열거형
    ├── logging/
    │   ├── log_buffer.hpp         ← 락-프리 SPSC 로그 링 버퍼
    │   ├── data_logger.hpp        ← CSV 파일 로거
    │   └── session_dir.hpp        ← 세션 디렉토리 관리
    ├── filters/
    │   ├── bessel_filter.hpp      ← 4차 Bessel 저역통과 필터
    │   └── kalman_filter.hpp      ← 이산시간 칼만 필터
    └── threading/
        ├── thread_config.hpp      ← CPU 코어별 스레드 레이아웃 프리셋
        ├── thread_utils.hpp       ← 스레드 구성/검증 유틸리티
        ├── publish_buffer.hpp     ← 락-프리 SPSC 퍼블리시 버퍼
        └── seqlock.hpp            ← 락-프리 단일 쓰기/다중 읽기 동기화
```

---

## 모듈 상세

### 타입 (`types/types.hpp`)

프레임워크 전체에서 공유하는 컴파일 타임 상수, 데이터 구조, 열거형을 정의합니다.

#### 주요 상수

| 상수 | 값 | 설명 |
|------|---|------|
| `kNumRobotJoints` | 6 | UR5e 관절 수 |
| `kMaxRobotDOF` | 12 | 최대 로봇 자유도 |
| `kNumHandMotors` | 10 | 핸드 모터 수 |
| `kDefaultNumFingertips` | 4 | 기본 핑거팁 수 |
| `kMaxFingertips` | 8 | 최대 핑거팁 수 |
| `kBarometerCount` | 8 | 핑거팁 당 기압 센서 수 |
| `kTofCount` | 3 | 핑거팁 당 ToF 센서 수 |
| `kMaxHandSensors` | 88 | 최대 핸드 센서 채널 수 |
| `kMaxInferenceValues` | 64 | 최대 추론 출력 크기 |
| `kFTValuesPerFingertip` | 13 | 핑거팁 당 F/T 추론 출력 수 |
| `kFTHistoryLength` | 12 | F/T 추론 FIFO 히스토리 길이 |

#### 주요 구조체

| 구조체 | 설명 |
|--------|------|
| `RobotState` | 6-DOF 관절 위치, 속도, 토크, TCP 위치, dt, 반복 카운터 |
| `HandState` | 모터 위치/속도, 센서 데이터 (raw + filtered), 핑거팁 수, 유효성 |
| `ControllerState` | `RobotState` + `HandState` 통합, 동기화된 dt/iteration |
| `ControllerOutput` | 로봇/핸드 커맨드, 목표/타겟 위치, 궤적, 커맨드 타입 |
| `FingertipFTState` | 핑거팁 힘/토크 추론 결과 |
| `TopicConfig` | 디바이스별 구독/퍼블리시 토픽 라우팅 설정 |
| `DeviceEnableFlags` | 전역 디바이스(UR5e/핸드) 활성화 플래그 |
| `PerControllerDeviceFlags` | 컨트롤러별 디바이스 활성화 오버라이드 |

#### 열거형

| 열거형 | 값 | 설명 |
|--------|---|------|
| `CommandType` | `kPosition`, `kTorque` | 커맨드 모드 |
| `SubscribeRole` | `kJointState`, `kHandState`, `kGoal` | 구독 역할 |
| `PublishRole` | `kPositionCommand`, `kTorqueCommand`, `kHandCommand`, `kTaskPosition`, `kTrajectoryState`, `kControllerState` | 퍼블리시 역할 |

> **RT 안전 설계:** 모든 구조체는 zero-initialized, trivially copyable로 설계되어 SeqLock/SPSC 버퍼와 호환됩니다.

---

### 필터 (`filters/`)

#### Bessel 필터 (`bessel_filter.hpp`)

4차 Bessel 저역통과 필터 — 최대 평탄 군지연 특성으로 신호 형상을 보존합니다.

- 2개 연속 biquad 섹션 (bilinear transform + 주파수 prewarping)
- N개 독립 채널 동시 필터링
- RT-safe: `Apply()` 함수는 `noexcept`, 할당 없음

| 타입 별칭 | N | 용도 |
|-----------|---|------|
| `BesselFilter6` | 6 | 로봇 관절 |
| `BesselFilter11` | 11 | 핑거팁 센서 |
| `BesselFilterBaro` | 64 | 기압 센서 |
| `BesselFilterTof` | 24 | ToF 센서 |
| `BesselFilter1` | 1 | 스칼라 |

```cpp
BesselFilter6 filter;
filter.Init(30.0, 500.0);  // 30 Hz 차단, 500 Hz 샘플링
auto filtered = filter.Apply(raw_positions);  // RT-safe
```

#### 칼만 필터 (`kalman_filter.hpp`)

이산시간 칼만 필터 — 등속 운동 모델 기반 위치/속도 추정기입니다.

- 상태: [position, velocity]ᵀ (채널당)
- 관측: 위치만 관측 (H = [1, 0])
- RT-safe: `PredictAndUpdate()` 함수는 `noexcept`, 할당 없음

| 타입 별칭 | N | 용도 |
|-----------|---|------|
| `KalmanFilter6` | 6 | 로봇 관절 |
| `KalmanFilter11` | 11 | 핑거팁 센서 |
| `KalmanFilter1` | 1 | 스칼라 |

```cpp
KalmanFilter6 kf;
kf.Init(1e-3, 1e-2, 1e-1, 0.002);  // q_pos, q_vel, r, dt
auto filtered = kf.PredictAndUpdate(measurements);  // RT-safe
```

---

### 스레딩 (`threading/`)

#### 스레드 구성 (`thread_config.hpp`)

CPU 코어 수에 따른 스레드 레이아웃 프리셋을 제공합니다. `SelectThreadConfigs()`가 물리 코어 수를 감지하여 자동 선택합니다.

**6코어 레이아웃 (기본):**

| 스레드 | 코어 | 스케줄러 | 우선순위 |
|--------|------|----------|----------|
| rt_control | 2 | SCHED_FIFO | 90 |
| sensor | 3 | SCHED_FIFO | 70 |
| udp_recv | 5 | SCHED_FIFO | 65 |
| logging | 4 | SCHED_OTHER | nice -5 |
| publish | 5 | SCHED_OTHER | nice -3 |
| status_monitor | 4 | SCHED_OTHER | nice -2 |
| aux | 5 | SCHED_OTHER | 0 |
| hand_failure | 4 | SCHED_OTHER | nice -2 |

> 4코어, 8코어, 10코어, 12코어, 16코어 레이아웃도 제공됩니다.

**MuJoCo 시뮬레이션 코어 레이아웃:**

`GetSimCoreLayout(physical_cores)` 함수로 시뮬레이션/뷰어 스레드 코어를 할당합니다.

#### 스레드 유틸리티 (`thread_utils.hpp`)

| 함수 | 설명 | RT-safe |
|------|------|---------|
| `ApplyThreadConfig()` | CPU 어피니티, 스케줄러, 우선순위 설정 | N/A (초기화) |
| `ApplyThreadConfigWithFallback()` | RT 실패 시 SCHED_OTHER 폴백 | N/A (초기화) |
| `CheckThreadHealthFast()` | 비트필드 기반 스레드 상태 검증 | **Yes** |
| `VerifyThreadConfig()` | 현재 스레드 설정 문자열 반환 | No |
| `GetPhysicalCpuCount()` | 물리 코어 수 (SMT 제외) | No |
| `GetOnlineCpuCount()` | 논리 CPU 수 (SMT 포함) | No |
| `SelectThreadConfigs()` | 코어 수 기반 레이아웃 자동 선택 | No |
| `GetThreadMetrics()` | 레이턴시 통계 (min/max/avg/p95/p99) | No |

#### SeqLock (`seqlock.hpp`)

락-프리 단일 쓰기 / 다중 읽기 동기화 프리미티브입니다.

- **Writer**: Wait-free (2개 atomic store + memcpy)
- **Reader**: Lock-free (contention 시 재시도)
- 요구사항: `T`는 `std::is_trivially_copyable_v<T>` 만족 필수

```cpp
SeqLock<ControllerState> state_lock;
state_lock.Store(new_state);           // RT 스레드 (500 Hz)
auto snapshot = state_lock.Load();     // 다른 스레드 (lock-free 읽기)
```

#### SPSC 버퍼 (`log_buffer.hpp`, `publish_buffer.hpp`)

RT 스레드(producer)에서 비-RT 스레드(consumer)로 데이터를 전달하는 락-프리 링 버퍼입니다.

| 버퍼 | 데이터 | 용량 | 용도 |
|------|--------|------|------|
| `ControlLogBuffer` | `LogEntry` | 512 (~1초 @500Hz) | CSV 로깅 |
| `ControlPublishBuffer` | `PublishSnapshot` | 512 (~1초 @500Hz) | ROS2 퍼블리시 오프로드 |

- Producer: `Push()` — RT-safe, wait-free, `noexcept`
- Consumer: `Pop()` — 비-RT 스레드에서 호출
- 캐시 라인 정렬로 producer/consumer 인덱스 false sharing 방지

---

### 로깅 (`logging/`)

#### 데이터 로거 (`data_logger.hpp`)

3개 CSV 파일에 제어 루프 데이터를 기록합니다.

| CSV 파일 | 내용 |
|----------|------|
| `timing_log.csv` | 타임스탬프, 상태 획득/계산/퍼블리시/전체 시간, 지터 |
| `robot_log.csv` | 목표/실제 위치, 속도, 토크, 태스크 위치, 커맨드, 궤적 |
| `device_log.csv` | 디바이스 목표/실제, 센서 raw/filtered, 추론 출력, 커맨드 |

#### 세션 디렉토리 (`session_dir.hpp`)

| 함수 | 설명 |
|------|------|
| `GenerateSessionTimestamp()` | `YYMMDD_HHMM` 형식 타임스탬프 생성 |
| `ResolveSessionDir()` | `RTC_SESSION_DIR` → `UR5E_SESSION_DIR` → 자동 생성 |
| `EnsureSessionSubdirs()` | controller, monitor, device, sim, plots, motions 하위 폴더 생성 |
| `CleanupOldSessions()` | 최대 세션 수 초과 시 오래된 세션 삭제 |

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |

> **외부 의존성 없음** — 표준 C++ 라이브러리와 POSIX API만 사용합니다.

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_base
source install/setup.bash
```

헤더 전용 라이브러리이므로 컴파일되는 바이너리는 없습니다. 소비자 패키지에서 `#include`하여 사용합니다.

---

## 의존성 그래프 내 위치

**기반 패키지** — 프레임워크의 모든 패키지가 의존합니다.

```
rtc_base  ← 독립 (외부 의존성 없음)
    ↑
    ├── rtc_communication          (스레드 구성)
    ├── rtc_controller_interface   (타입, 토픽 설정)
    ├── rtc_controller_manager     (타입, 로깅, 스레딩, 퍼블리시 버퍼)
    ├── rtc_controllers            (타입, 필터, SeqLock)
    ├── rtc_inference              (타입)
    ├── rtc_status_monitor         (타입, 스레딩)
    ├── rtc_mujoco_sim             (타입, 스레딩)
    ├── ur5e_hand_driver           (타입, 로깅, 필터)
    └── ur5e_bringup               (타입, 로깅, 스레드 구성)
```

### 모듈별 사용 패키지

| 모듈 | 주요 사용 패키지 |
|------|-----------------|
| `types` | 전체 13개 하위 패키지 |
| `filters` | `rtc_controllers`, `ur5e_hand_driver` |
| `threading` | `rtc_controller_manager`, `rtc_communication`, `ur5e_bringup` |
| `logging` | `rtc_controller_manager`, `ur5e_bringup` |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
