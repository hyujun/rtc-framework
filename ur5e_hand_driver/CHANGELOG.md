# 변경 이력 — ur5e_hand_udp

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.15.1] - 2026-03-18

### 변경 (Changed) — Sub-ms recv timeout: SO_RCVTIMEO → ppoll

- **`RecvWithTimeout()` 도입**: `SO_RCVTIMEO` → `ppoll()` 기반 수신 타임아웃
  - `SO_RCVTIMEO`는 `schedule_timeout()` 사용 → jiffies 해상도 (HZ=1000에서 1ms) → sub-ms 불가
  - `ppoll()`은 `struct timespec` (ns 단위) → hrtimer → PREEMPT_RT에서 µs 정밀도 제공
  - 모든 blocking `recv()` 호출을 `RecvWithTimeout()` (ppoll + MSG_DONTWAIT)으로 교체
  - `SO_RCVTIMEO`는 100ms 안전망으로 유지 (ppoll 실패 시 무한 블록 방지)
- **`recv_timeout_ms` stats JSON 추가**: 런타임 적용값 확인용 (`hand_udp_stats.json`)
- **YAML 기본값 변경**: `recv_timeout_ms` 2.0ms → 0.4ms (400µs)
- **`recv_timeout_ms()` 접근자 추가**: `HandController`에서 설정값 조회 가능

---

## [5.15.0] - 2026-03-17

### 추가 (Added) — RT 최적화: SeqLock, Write Echo, printf 제거

- **SeqLock 기반 lock-free 상태 공유**
  - `state_mutex_` → `SeqLock<HandState>` 교체 — writer(EventLoop) wait-free, reader lock-free
  - `ur5e_rt_base/threading/seqlock.hpp` 신규 — 범용 single-writer/multi-reader SeqLock 템플릿
  - Priority inversion 완전 제거 (SCHED_FIFO 스레드 간 mutex 공유 없음)

- **WritePosition echo 활용 (Individual 모드 1 round-trip 절약)**
  - Write echo를 항상 수신하여 소켓 버퍼 오염 방지
  - Individual 모드: echo 데이터를 motor position으로 사용 → `ReadPosition(0x11)` 제거
  - 6 round-trips → 5 round-trips (Individual), Bulk 모드: stale 패킷 문제 해결

- **응답 cmd 검증 추가**
  - `RequestMotorRead()`: response cmd 필드와 request cmd 비교 — stale/echo 패킷 거부
  - `RequestAllMotorRead()`: response cmd == 0x10 검증

### 변경 (Changed)

- **EventLoop에서 printf 완전 제거** — stdout mutex + write(2) syscall로 인한 ms-level 스파이크 제거
  - 센서 디버그 출력 제거 (HandTimingProfiler로 대체)
  - 통신 통계 디버그 출력 제거 (HandCommStats / JSON export로 대체)

- **`enable_write_ack` 파라미터 deprecated** — write echo는 항상 수신
  - 생성자 시그니처 호환성 유지 (파라미터 무시)
  - YAML에서 제거, 기존 설정 파일과의 하위 호환성 유지

- **`DrainStaleResponses()` 호출 제거** — write echo 수신 + cmd 검증으로 stale 패킷 방지
  - Individual 모드: echo 수신 후 velocity read → sensor read (깨끗한 소켓 버퍼)
  - Bulk 모드: echo 수신 후 AllMotorRead → AllSensorRead (cmd 검증으로 안전)

- `HandUdpNode`: `data_mutex_` → `std::atomic<bool>` + `GetLatestState()` (SeqLock) 사용

### 수정 (Fixed)

- **Cascading response 버그 수정** — `enable_write_ack=false`일 때 write echo가 소켓 버퍼에 잔류하여:
  - Individual 모드: ReadPosition이 echo(cmd=0x01)를 수신 → commanded 값을 actual로 오인, velocity에 position 데이터 혼입
  - Bulk 모드: AllMotorRead가 43B echo를 수신(123B 기대) → 항상 실패

---

## [5.14.0] - 2026-03-16

### 변경 (Changed) — 문서 업데이트

- README.md: HandTimingProfiler 섹션 추가
- README.md: JSON 통계 예시에 event_skip_count, timing_stats 추가
- README.md: 버전 v5.14.0 업데이트

---

## [5.11.0] - 2026-03-14

### 추가 (Added) — Event Skip 보호 및 Sensor Decimation

- **Event Skip 보호 (`busy_` flag)**
  - `std::atomic<bool> busy_` — EventLoop 실행 중 `true`, 완료 시 `false`
  - `SendCommandAndRequestStates()` 호출 시 `busy_`이면 이벤트 skip + `event_skip_count_` 증가
  - Skip 시 이전 state 유지 (latest_state_ 변경 없음)
  - `event_skip_count()` 접근자 추가

- **Sensor Decimation (`sensor_decimation` 파라미터)**
  - `sensor_decimation: N` — N cycle마다 센서(4 fingertip) 읽기, 나머지 cycle은 캐시 사용
  - 기본값: `4` (평균 ~1.3ms/cycle, write+pos+vel만 → 500Hz ControlLoop 추종 가능)
  - `sensor_decimation: 1`으로 설정 시 매 cycle 센서 읽기 (기존 동작)
  - `cached_sensor_data` 로컬 버퍼로 센서 데이터 캐싱

- **`HandCommStats` 확장**
  - `event_skip_count` 필드 추가 (comm_stats() 스냅샷에 포함)

### 변경 (Changed)

- `HandController` 생성자에 `sensor_decimation` 파라미터 추가 (기본값 1)
- `hand_udp_node.yaml`에 `sensor_decimation: 4` 파라미터 추가

---

## [5.10.0] - 2026-03-14

### 변경 (Changed) — 세션 기반 로깅 경로

- JSON 통계 출력 경로: `/tmp/hand_udp_stats_YYYYMMDD_HHMMSS.json` → `UR5E_SESSION_DIR/hand/hand_udp_stats.json`
  - 세션 디렉토리 미설정 시 `/tmp/` 폴백
  - 파일 이름에서 타임스탬프 제거 (세션 폴더가 타임스탬프 역할)

---

## [5.8.0] - 2026-03-14

### 추가 (Added) — 실패 감지, 타임아웃 설정, ACK

- **C1: UDP 수신 타임아웃 YAML 설정**
  - `recv_timeout_ms` 파라미터 (기본 10ms, `hand_udp.yaml`에서 설정)
  - `HandController` 생성자에서 `SO_RCVTIMEO` 설정
  - 수신 실패 시 `recv_error_count_` 원자적 카운터 증가

- **H2: HandFailureDetector (C++ 실패 감지기)**
  - `hand_failure_detector.hpp` 신규 — 50Hz `std::jthread` 비-RT 모니터링
  - 전-영점 데이터 및 중복 데이터 연속 N회 감지
  - 모터 + 센서 데이터 개별 검사 설정 (`check_motor`, `check_sensor`)
  - 실패 콜백 등록 → 글로벌 E-Stop 트리거

- **L2: Hand Command ACK 메커니즘**
  - `enable_write_ack` 설정 (기본 false)
  - `WritePosition` 후 조건부 `recvfrom()` ACK 수신

- **글로벌 E-Stop 플래그 연동**
  - `SetEstopFlag(std::atomic<bool>*)` — RT 컨트롤러에서 전파
  - PollLoop에서 E-Stop 시 영점 명령 전송 후 중단

### 변경 (Changed)

- `hand_udp.yaml`에 `recv_timeout_ms`, `enable_write_ack`, `failure_detector` 섹션 추가

---

## [5.7.0] - 2026-03-11

### 변경
- 워크스페이스 전체 버전 (v5.7.0) 통일

---

## [5.3.0] - 2026-03-08

### 변경
- 워크스페이스 전체 버전 (v5.3.0) 통일

---

## [5.1.0] - 2026-03-07

### 변경 (Changed) — UDP 수신 스레드 코어 이동

- `HandUdpReceiver` RT 스레드: Core 3 → Core 5로 이동 (`kUdpRecvConfig`)
- sensor_io 스레드(Core 3)와의 경합 방지

---

## [5.0.0] - 2026-03-07

### 추가

- **초기 분리**: `ur5e_rt_controller` 단일 패키지에서 UDP 핸드 브리지를 독립 패키지로 추출
- `HandUdpReceiver`: UDP 포트 50001에서 616바이트(77 double) 손 상태 패킷 수신, `std::jthread` 사용
- `HandUdpSender`: 11 double 리틀 엔디언 인코딩 모터 명령 송신 (포트 50002)
- `hand_udp_receiver_node`: UDP → `/hand/joint_states` (100Hz) ROS2 브리지
- `hand_udp_sender_node`: `/hand/command` → UDP ROS2 브리지
- `hand_udp.launch.py`: `udp_port`, `target_ip`, `target_port` 파라미터
- `config/hand_udp_receiver.yaml`: 수신 설정 (포트, 버퍼, 타임아웃, 퍼블리시 주기, 통계)
