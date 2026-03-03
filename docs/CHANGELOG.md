# Changelog

모든 주요 변경사항은 이 파일에 기록됩니다.
형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/)를 따릅니다.

---

## [4.2.3] - 2026-03-03

### 수정 (Fixed) — RT 안전성 9건

#### CRITICAL: RT 스레드 파일 I/O 제거 (Fix 1)
- **문제**: `ControlLoop()` (SCHED_FIFO 90, Core 2)에서 `DataLogger::LogControlData()` 직접 호출 → `std::ofstream` syscall로 수백 µs 블로킹 가능
- **해결**: `log_buffer.hpp` 신규 추가 — `SpscLogBuffer<512>` lock-free SPSC 링 버퍼
  - `ControlLoop()`는 `log_buffer_.Push(entry)` 만 호출 (O(1), 힙 할당 없음)
  - `drain_timer_` (100Hz, log 스레드 Core 4)가 `logger_->DrainBuffer(log_buffer_)` 호출
  - 파일 I/O가 RT 경로에서 완전히 제거됨

#### CRITICAL: `state_received_` / `target_received_` 데이터 레이스 (Fix 2)
- **문제**: `state_received_`, `target_received_`, `hand_data_received_` 가 plain `bool`로 선언되어 RT 스레드(읽기)와 sensor 스레드(쓰기)가 동시 접근 → C++ Undefined Behavior
- **해결**: `std::atomic<bool>` 으로 교체; 쓰기는 `memory_order_release`, 읽기는 `memory_order_acquire` 사용

#### CRITICAL: `TargetCallback`에서 뮤텍스 해제 후 `target_positions_` 읽기 (Fix 3)
- **문제**: `target_mutex_` 해제 후 `controller_->SetRobotTarget(target_positions_)` 호출 → RT 스레드가 사이에 덮어쓸 수 있음
- **해결**: 뮤텍스 안에서 `local_target = target_positions_` 복사본 생성 후 복사본으로 호출

#### MAJOR: `target_mutex_` 블록 논리 오류 + 로깅 무보호 읽기 (Fix 4)
- **문제**: `ControlLoop()`의 `target_mutex_` 블록이 `target_positions_` 복사 없이 무관한 필드만 설정; 로깅 시 `target_positions_`를 뮤텍스 없이 읽음
- **해결**: `target_snapshot_` 멤버 추가 — `target_mutex_` 안에서 복사 후 루프 전체에서 스냅샷만 사용; `state.robot.dt`, `state.iteration`은 뮤텍스 밖으로 이동

#### MAJOR: RT 스레드에서 `publish()` 힙 할당 (Fix 5)
- **문제**: `std_msgs::msg::Float64MultiArray` 생성 + `vector::assign()` + `publish()` 가 RT 루프에서 힙 할당 유발 가능
- **해결**: `realtime_tools::RealtimePublisher<T>` 도입 — 메시지 사전 할당, `trylock()` / `unlockAndPublish()`로 non-blocking 발행

#### MAJOR: `HandUdpReceiver` jthread RT 스케줄링 미적용 (Fix 6)
- **문제**: `kUdpRecvConfig` (Core 3, SCHED_FIFO 65)가 선언만 되고 jthread에 전혀 적용되지 않음
- **해결**: `HandUdpReceiver` 생성자에 `const ThreadConfig& thread_cfg = kUdpRecvConfig` 파라미터 추가; `thread_cfg_` 멤버 저장 (구현체의 `ReceiveLoop()` 진입 시 `ApplyThreadConfig()` 호출 필요)
- `hand_udp_receiver_node.cpp` main()에 `mlockall()` 추가

#### MAJOR: `mlockall` 호출 순서 오류 (Fix 7)
- **문제**: `rclcpp::init()` 이후 `mlockall(MCL_CURRENT)` 호출 → DDS/RMW가 이미 할당한 페이지 미잠금
- **해결**: `mlockall(MCL_CURRENT | MCL_FUTURE)` → `rclcpp::init()` 순서로 재배치; `MCL_FUTURE`로 이후 할당도 자동 잠금

#### MEDIUM: `GetThreadStats()` include 누락 (Fix 8)
- **문제**: `thread_utils.hpp`가 `std::min_element`, `std::accumulate`, `std::tuple`, `std::vector` 를 사용하나 해당 헤더 미포함 → 다른 TU에서 직접 include 시 컴파일 실패 가능
- **해결**: `<algorithm>`, `<numeric>`, `<tuple>`, `<vector>` 추가; `thread_config.hpp`에 `<sched.h>` 추가 (독립 포함 지원)

#### LOW: 4코어 fallback 자동 미적용 (Fix 9)
- **문제**: `kRtControlConfig4Core` 등이 선언만 되고 `main()`은 항상 6코어 설정 하드코딩 → 4코어 시스템에서 CPU affinity 실패
- **해결**: `thread_utils.hpp`에 `GetOnlineCpuCount()` (`sysconf` 래퍼) + `SelectThreadConfigs()` 추가; `main()`이 런타임에 코어 수를 감지해 적절한 설정 선택

### 추가 (Added)
- `include/ur5e_rt_controller/log_buffer.hpp`: `LogEntry` 구조체 + `SpscLogBuffer<N>` 템플릿 + `ControlLogBuffer` typedef
- `DataLogger::DrainBuffer(ControlLogBuffer&)`: log 스레드 전용 드레인 메서드
- `GetOnlineCpuCount()`, `SelectThreadConfigs()`, `SystemThreadConfigs`: `thread_utils.hpp`에 추가

### 사용자 영향
- **기능 변화 없음**: ROS2 인터페이스(토픽, 파라미터, 서비스) 동일
- **로그 파일 계속 생성**: drain 타이머(100Hz)가 log 스레드에서 CSV 기록 유지
- **4코어 시스템**: 별도 설정 없이 자동으로 4코어 스레드 설정 적용
- **RT 지터 개선**: 파일 I/O 및 힙 할당 제거로 500Hz 루프 안정성 향상

---

## [4.2.2] - 2026-03-02

### 제거 (Removed)
- **organize_files.sh 삭제**: 일회성 파일 정리 스크립트 제거
  - v4.0.0에서 파일 정리 작업 이미 완료
  - 파일명 불일치로 실행 시 오류 발생 (`monitor_data_health_v2.py` 없음)
  - package.xml을 오래된 버전(v4.0.0)으로 덮어쓸 위험

### 변경 (Changed)
- **install.sh v4.2.1 업데이트**:
  - 버전 표시: v4.0.0 → v4.2.1
  - GitHub URL 수정: `your-repo/ur5e_rt_controller` → `hyujun/ur5e-rt-controller`
  - organize_files.sh 호출 제거 (더 이상 불필요)
  - 실행 파일명 수정: `monitor_data_health_v2.py` → `monitor_data_health.py`
  - requirements.txt 사용: `pip3 install --user -r requirements.txt`
  - chmod 명령 제거: CMakeLists.txt의 `install(PROGRAMS)`가 자동 처리
  - **RT 권한 설정 추가** (v4.2.0+ 필수):
    - realtime 그룹 생성 및 사용자 추가
    - `/etc/security/limits.conf`에 rtprio 99, memlock unlimited 설정
    - 로그아웃/로그인 안내 메시지

### 개선 (Improved)
- **install.sh Quick Start 섹션 확장**:
  - 4개 Python 유틸리티 스크립트 모두 예제 포함
    1. monitor_data_health.py
    2. plot_ur_trajectory.py
    3. motion_editor_gui.py
    4. hand_udp_sender_example.py
  - RT_OPTIMIZATION.md 참조 추가
  - v4.2.0+ 병렬 컴퓨팅 설정 안내 추가

### 사용자 영향
- **신규 설치 시**: install.sh가 RT 권한 자동 설정
- **기존 사용자**: 영향 없음 (organize_files.sh 이미 사용 안 함)
- **Quick Start 가이드 개선**: 모든 유틸리티 스크립트 사용법 명확화

---

## [4.2.1] - 2026-03-02

### 제거 (Removed)
- **setup.py 삭제**: ROS2 ament_cmake 표준 구조 준수
  - ROS2 C++ 패키지는 setup.py 불필요 (CMakeLists.txt로 충분)
  - 기존 setup.py는 패키지 디스커버리 실패 (`find_packages()` 0개 발견)
  - entry_points 파일명 불일치 (`monitor_data_health_v2.py` 파일 없음)

### 변경 (Changed)
- **CMakeLists.txt v4.2.1 업데이트**:
  - VERSION: 4.0.0 → 4.2.1 (package.xml과 일치)
  - `install(PROGRAMS)` 섹션에 누락된 스크립트 2개 추가:
    - `scripts/motion_editor_gui.py`
    - `scripts/hand_udp_sender_example.py`
  - 총 4개 Python 유틸리티 스크립트 설치 (`ros2 run` 사용 가능)

### 사용자 영향
- **Python 의존성 설치 방법 변경**:
  ```bash
  # 이전 (작동 안 함)
  pip install -e .  # ❌ 실패
  
  # 변경 후 (명확)
  pip3 install --user -r requirements.txt  # Python 의존성 설치
  colcon build                              # ROS2 패키지 빌드
  ```

- **스크립트 실행 방법 표준화**:
  ```bash
  # ROS2 표준 (권장)
  ros2 run ur5e_rt_controller monitor_data_health.py
  ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv
  ros2 run ur5e_rt_controller motion_editor_gui.py
  ros2 run ur5e_rt_controller hand_udp_sender_example.py
  
  # 또는 직접 실행
  python3 ~/ur_ws/src/ur5e-rt-controller/scripts/plot_ur_trajectory.py <csv_file>
  ```

### 기술적 세부사항
- **requirements.txt 유지**: 문서 목적으로 보관
- **CMakeLists.txt가 모든 설치 처리**: C++ 실행 파일, Python 스크립트, 헤더, 설정 파일
- **ROS2 Best Practice 준수**: ament_cmake 패키지는 setup.py 불필요

---

## [4.2.0] - 2026-03-02

### 추가 (Added)
- **병렬 컴퓨팅 최적화**: CallbackGroup 기반 멀티스레드 executor 아키텍처
  - `thread_config.hpp`: 스레드 설정 구조체 및 사전 정의 설정 (RT, Sensor, Logging, Aux)
  - `thread_utils.hpp`: RT 스케줄링 및 CPU affinity 유틸리티 함수
    - `ApplyThreadConfig()`: SCHED_FIFO, CPU affinity, nice value 설정
    - `VerifyThreadConfig()`: 스레드 설정 검증 및 로깅
    - `GetThreadStats()`: 지터 측정용 통계 함수
- **4개 CallbackGroup 분리** (`custom_controller.cpp`):
  - `cb_group_rt_`: control_timer_(500Hz), timeout_timer_(50Hz) → Core 2, SCHED_FIFO 90
  - `cb_group_sensor_`: joint_state_sub_, target_sub_, hand_state_sub_ → Core 3, SCHED_FIFO 70
  - `cb_group_log_`: 로깅 작업 → Core 4, SCHED_OTHER nice -5
  - `cb_group_aux_`: estop_pub_ → Core 5, SCHED_OTHER
- **mlockall 메모리 잠금**: main()에서 페이지 폴트 방지
- **스레드별 RT 설정 자동화**: 각 executor를 별도 스레드에서 실행하며 스케줄러 정책 자동 적용

### 변경 (Changed)
- **`custom_controller.cpp` 재설계**:
  - SingleThreadedExecutor 4개로 분리 (이전: 1개)
  - 각 executor를 std::thread에서 spin() + RT 설정 적용
  - 콜백 그룹 인자를 create_subscription/create_wall_timer에 명시
- **main() 분리**: mlockall, executor 생성, 스레드 spawn 로직 추가
- **CMakeLists.txt**: C++20 표준 유지, thread_utils.hpp 헤더 install 경로 추가

### 성능 개선
- **제어 지터 목표**: ~500μs → <50μs (10배 개선 예상)
- **E-STOP 반응 시간**: ~100ms → <20ms (5배 개선 예상)
- **Priority inversion 제거**: RT 루프와 I/O가 별도 코어에서 실행
- **CPU 마이그레이션 차단**: CPU affinity로 cache warmup 유지

### 시스템 요구사항 변경
- **RT 권한 필수**: `/etc/security/limits.conf`에 `@realtime - rtprio 99` 설정
- **6-core CPU 권장**: Core 0-1(OS/DDS), 2(RT), 3(Sensor), 4(Log), 5(Aux)
- **4-core fallback 지원**: thread_config.hpp에 4-core 설정 포함
- **PREEMPT_RT 커널 권장**: 최대 RT 성능을 위해 필요

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
