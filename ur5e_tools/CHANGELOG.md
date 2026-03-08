# 변경 이력 — ur5e_tools

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.3.0] - 2026-03-08

### 추가 (Added) — `controller_gui.py` 컨트롤러 GUI

`ur5e_tools/controller_gui.py` 신규 추가. tkinter 기반 다크 테마 GUI로 컨트롤러 전환·게인 튜닝·타겟 전송을 단일 창에서 수행합니다.

**ROS2 인터페이스:**
- **구독**: `/joint_states` → 현재 관절 위치 실시간 표시 (5Hz)
- **발행**:
  - `/custom_controller/controller_type` (`Int32`) — 컨트롤러 교체
  - `/custom_controller/controller_gains` (`Float64MultiArray`) — 게인 업데이트
  - `/target_joint_positions` (`Float64MultiArray`) — 타겟 위치 전송

**주요 UI 요소:**
- 컨트롤러 라디오 버튼 (P / PD / Pinocchio / CLIK / OSC)
- 동적 게인 패널 (컨트롤러 전환 시 자동 재구성)
- 3열 위치 테이블 (Axis 이름 / Target 입력 / Current 표시)
- Switch / Apply Gains / Copy Current→Target / Send Command 동작 버튼

**`setup.py` 변경:** `controller_gui.py`를 `lib/ur5e_tools/` 실행 파일로 추가 등록

---

## [5.2.2] - 2026-03-07

### 변경

- 워크스페이스 내 모든 패키지 버전을 `5.2.2`로 통일
- ROS 2 Jazzy 지원 대응 및 문서 갱신
- `plot_ur_trajectory.py`: 기본 CSV 검색 경로를 `~/ros2_ws/ur5e_ws/logging_data/` 디렉터리로 변경 (하드코딩된 `/tmp/` 경로 제거)
- `monitor_data_health.py`: JSON 통계 경로 기본값은 기존 `/tmp/ur5e_stats/` 유지 (임시 데이터)

## [1.0.0] - 2026-03-04

### 추가

- **초기 분리**: `ur5e_rt_controller` 단일 패키지의 `scripts/` 디렉터리를 독립 Python 패키지로 추출
- `scripts/plot_ur_trajectory.py` — Matplotlib 궤적 시각화
  - CSV 로그 파일 입력 (DataLogger 출력 형식)
  - 관절별 위치/목표/명령값 플롯
  - `--joint N` 옵션으로 특정 관절만 표시
- `scripts/monitor_data_health.py` — `DataHealthMonitor` ROS2 노드
  - 4개 토픽 패킷 속도 및 타임아웃 추적
  - 종료 시 JSON 통계를 `/tmp/ur5e_stats/`에 저장
  - 파라미터: `check_rate` (10Hz), `timeout_threshold` (0.2s), `stats_output_dir`, `enable_stats`
- `scripts/motion_editor_gui.py` — Qt5 50-포즈 모션 편집기
  - `/joint_states` 구독으로 현재 각도 캡처
  - `/target_joint_positions` 퍼블리시로 포즈 실행
  - JSON 저장/불러오기, 순차 재생 (2초 딜레이)
- `scripts/hand_udp_sender_example.py` — 합성 UDP 손 데이터 생성기
  - 정현파/정적 패턴의 77개 `double` 패킷 전송 (포트 50001)
  - 개발/테스트 환경에서 실제 손 하드웨어 대체
- `ur5e_tools/__init__.py` — Python 패키지 초기화
- `resource/ur5e_tools` — ament 리소스 파일
- `setup.py`, `setup.cfg` — ament_cmake_python 설치 설정
  - 4개 스크립트를 `lib/ur5e_tools/`에 설치 (실행 파일로)
- `CMakeLists.txt` — `ament_cmake_python` 기반, `ament_python_install_package`
- `package.xml` — `buildtool_depend: ament_cmake_python`, `exec_depend: rclpy, python3-numpy`

### 참고

이 패키지는 다음 v4.4.0 `ur5e_rt_controller` 파일에서 추출되었습니다:
- `scripts/plot_ur_trajectory.py`
- `scripts/monitor_data_health.py`
- `scripts/motion_editor_gui.py`
- `scripts/hand_udp_sender_example.py`

기능 변경 없이 패키지 분리만 수행되었습니다. `ur_control.launch.py`와 `mujoco_sim.launch.py`의 `monitor_data_health.py` 노드 패키지 참조가 `ur5e_tools`로 변경되었습니다.
