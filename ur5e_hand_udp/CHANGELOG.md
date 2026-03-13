# 변경 이력 — ur5e_hand_udp

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

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
