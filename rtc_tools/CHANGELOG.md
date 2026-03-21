# 변경 이력 — rtc_tools

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.12.0] - 2026-03-14

### 변경 (Changed) — 궤적 시각화 v3

- **`plot_ur_trajectory.py` v3 업데이트**
  - 새 robot_log.csv 4-카테고리 컬럼 지원 (`traj_pos_*`, `traj_vel_*`, `command_*`, `actual_torque_*`, `task_pos_*`)
  - 레거시 CSV 하위 호환 (`target_pos_*`, `target_vel_*` 자동 감지)
  - 신규 Figure: `--command` (제어 명령), `--torque` (실제 토크), `--task-pos` (TCP 위치)
  - `--all` 플래그: 모든 Figure 한 번에 생성
  - 파일명 감지 패턴: `robot_log_*.csv` → `robot_log*.csv` 확장 (세션 디렉토리 호환)

---

## [5.10.0] - 2026-03-14

### 추가 (Added) — 세션 디렉토리 유틸리티

- **`utils/session_dir.py` 신규** — Python 세션 디렉토리 헬퍼
  - `get_session_dir()` — `RTC_SESSION_DIR` 환경변수에서 세션 경로 읽기
  - `get_session_subdir(subdir)` — 세션 내 서브디렉토리 경로 반환 + 자동 생성

### 변경 (Changed) — 세션 기반 로깅 경로

- `plot_rtc_log.py` (구 `plot_ur_trajectory.py`): `--save-dir` 미지정 시 `RTC_SESSION_DIR/plots/` 기본 경로
- `motion_editor_gui.py`: JSON 저장 기본 경로를 `RTC_SESSION_DIR/motions/`로 변경
- `hand_udp_sender_example.py`: CSV 로깅 기본 경로를 `RTC_SESSION_DIR/hand/`로 변경
- `monitor_data_health.py` 제거 (기능이 `rtc_status_monitor` 및 `ur5e_hand_driver`에 통합됨)

---

## [5.8.0] - 2026-03-14

### 추가 (Added) — 타이밍 통계 분석

- **C3: `monitor_data_health.py` 타이밍 통계 확장**
  - CSV 로그의 새 타이밍 컬럼 파싱: `t_state_acquire_us`, `t_compute_us`, `t_publish_us`, `t_total_us`, `jitter_us`
  - 타이밍 메트릭별 mean/max/std/variance 통계 산출
  - `_make_empty_stats()`에 `"timing"` 섹션 추가

---

## [5.7.0] - 2026-03-11

### 추가 (Added) — MJCF vs URDF 파라미터 비교 도구

- `rtc_tools/validation/compare_mjcf_urdf.py` 신규
- `ur5e_description` 패키지의 MJCF(`ur5e.xml`)와 URDF(`ur5e.urdf`) 물리 파라미터 비교 검증
- 비교 항목: link mass, diagonal inertia, joint position limits, effort limits, axis vector, link origin offset
- MJCF default class 상속 해석 (`size3`, `size3_limited`, `size1`)
- 자동 경로 탐색: `ament_index` 또는 개발 레이아웃 상대 경로
- 종료 코드: mismatch 0이면 `0`, 아니면 `1` (CI 통합 가능)
- `setup.py` entry_points 등록: `ros2 run rtc_tools compare_mjcf_urdf`

---

## [5.6.1] - 2026-03-10

### 변경 (Changed) — GUI 업데이트

- `motion_editor_gui.py`: 퍼블리시 토픽을 `/forward_position_controller/commands` → `/target_joint_positions`로 수정, 재생 간격 스핀박스 추가 (기본 2.0s, 범위 0.1–30.0s)
- `controller_gui.py`: 게인 입력 패널 업데이트 (Pinocchio 15개, OSC 16개 게인)

---

## [5.5.1] - 2026-03-09

### 변경 (Changed)

- 플롯 저장 경로 변경: `~/ur_plots` → `logging_data/ur_plot`

---

## [5.3.0] - 2026-03-08

### 추가 (Added) — 컨트롤러 GUI

- `gui/controller_gui.py` 신규: tkinter 기반 다크 테마 GUI
  - 컨트롤러 선택 (P/PD/Pinocchio/CLIK/OSC), 게인 슬라이더, 타겟 전송을 단일 GUI로 통합
  - `/rt_controller/controller_type`, `/rt_controller/controller_gains`, `/target_joint_positions` 토픽 발행

---

## [5.0.0] - 2026-03-07

### 추가

- **초기 분리**: `rtc_controller_manager` 단일 패키지에서 Python 유틸리티를 독립 패키지로 추출
- `plotting/plot_ur_trajectory.py`: Matplotlib 기반 CSV 제어 로그 궤적 시각화
- `monitoring/monitor_data_health.py`: `DataHealthMonitor` ROS2 노드 — 4개 토픽 패킷 속도/타임아웃 추적, JSON 통계 출력
- `gui/motion_editor_gui.py`: Qt5 기반 50-포즈 모션 편집기 GUI
- `utils/hand_udp_sender_example.py`: 합성 UDP 손 데이터 생성기
