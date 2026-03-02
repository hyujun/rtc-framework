# Changelog

모든 주요 변경사항은 이 파일에 기록됩니다.
형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/)를 따릅니다.

---

## [4.0.0] - 2026-03-02

### 추가 (Added)
- **E-STOP 시스템**: 로봇/핸드 데이터 타임아웃 자동 감지 및 비상 정지 기능
  - `robot_timeout_ms` (기본 100ms), `hand_timeout_ms` (기본 200ms) 파라미터
  - E-STOP 발생 시 `/system/estop_status` 토픽 퍼블리시
  - 타임아웃 감시 타이머 (50Hz, `check_timeouts()`)
- **PDController E-STOP 지원** (`pd_controller.hpp`): `trigger_estop()`, `clear_estop()`, 안전 위치 복귀
- **핸드 E-STOP 분리**: 핸드 데이터 타임아웃 시 로봇은 유지, 핸드 명령만 차단
- **`hand_udp_receiver_node`** (`src/hand_udp_receiver_node.cpp`): UDP 패킷 수신 ROS2 노드
- **`hand_udp_sender_node`** (`src/hand_udp_sender_node.cpp`): UDP 명령 송신 ROS2 노드
- **`hand_udp.launch.py`**: 핸드 UDP 노드 전용 런치 파일 (`udp_port`, `target_ip`, `target_port` 인자)
- **`hand_udp_sender_example.py`** (`scripts/`): 핸드 시뮬레이터 예제 (사인파/고정 포즈 모드)
- **`monitor_data_health.py`** (`scripts/`): 실시간 데이터 헬스 모니터 + JSON 통계 내보내기
- **`install.sh`**: 자동 설치 스크립트 (의존성 설치 → 빌드 → 환경 설정)
- **표준 ROS2 디렉토리 구조** (`organize_files.sh`에 따라 재구성):
  - `config/`, `launch/`, `include/ur5e_rt_controller/controllers/`, `src/`, `scripts/`
  - `docs/`, `rviz/`, `test/`, `resources/` (빈 플레이스홀더)

### 변경 (Changed)
- **프로젝트 구조 전면 재편**: 루트에 산재했던 파일들을 ROS2 표준 레이아웃으로 이동

  | 이동 전 (루트) | 이동 후 |
  |---|---|
  | `ur5e_rt_controller.yaml` | `config/` |
  | `hand_udp_receiver.yaml` | `config/` |
  | `ur_control.launch.py` | `launch/` |
  | `hand_udp.launch.py` | `launch/` |
  | `rt_controller_interface.hpp` | `include/ur5e_rt_controller/` |
  | `data_logger.hpp` | `include/ur5e_rt_controller/` |
  | `hand_udp_receiver.hpp` | `include/ur5e_rt_controller/` |
  | `hand_udp_sender.hpp` | `include/ur5e_rt_controller/` |
  | `p_controller.hpp` | `include/ur5e_rt_controller/controllers/` |
  | `pd_controller.hpp` | `include/ur5e_rt_controller/controllers/` |
  | `custom_controller.cpp` | `src/` |
  | `hand_udp_receiver_node.cpp` | `src/` |
  | `hand_udp_sender_node.cpp` | `src/` |
  | `monitor_data_health.py` | `scripts/` |
  | `plot_ur_trajectory.py` | `scripts/` |
  | `hand_udp_sender_example.py` | `scripts/` |
  | `motion_editor_gui.py` | `scripts/` |

- **`ur_control.launch.py`**: 데이터 헬스 모니터 노드 추가, `use_fake_hardware` 인자 추가
- **`ur5e_rt_controller.yaml`**: E-STOP 설정 섹션 추가 (`estop:`, `safety:`, `logging:`)
- **`package.xml`**: 버전 3.x → 4.0.0, 설명 업데이트
- **`CMakeLists.txt`**: `hand_udp_receiver_node`, `hand_udp_sender_node` 빌드 타겟 추가
- **`README.md`**: 코드 심층 분석 기반으로 전면 재작성
  - 정확한 디렉토리 트리 (이동된 경로 기준)
  - ROS2 토픽 인터페이스 표 (구독/발행, 메시지 타입)
  - UDP 핸드 프로토콜 문서 (77 double / 616 bytes 패킷 형식)
  - 클래스별 파라미터 표 (`CustomController`, `PDController` 등)
  - E-STOP 동작 흐름 설명
  - 아키텍처 다이어그램 (ASCII)

---

## [3.0.0] - 이전 버전

### 추가 (Added)
- `PDController` 기본 구현 (`pd_controller.hpp`)
- `DataLogger` CSV 로깅 (`data_logger.hpp`)
- `HandUdpReceiver` / `HandUdpSender` UDP 인터페이스

### 변경 (Changed)
- `RTControllerInterface`에 `HandState` (11-DOF) 통합
- `ControllerState`에 `RobotState` + `HandState` 결합

---

## [2.0.0] - 이전 버전

### 추가 (Added)
- `DataLogger` CSV 로깅 기능
- 핸드 UDP 통합 기초 구조
- `motion_editor_gui.py` Qt5 모션 에디터 (50포즈)
- `plot_ur_trajectory.py` Matplotlib 시각화

---

## [1.0.0] - 초기 릴리스

### 추가 (Added)
- `RTControllerInterface` Strategy Pattern 기반 제어기 추상 클래스
- `PController` 비례 제어기 구현
- `CustomController` ROS2 노드 (기본 제어 루프)
- `ur_control.launch.py` 런치 파일
- `ur5e_rt_controller.yaml` 기본 설정
