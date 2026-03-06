# CLAUDE.md — ur5e_tools

Python 개발 유틸리티 패키지. **C++ 코드 없음** — 스크립트 4개로 구성된다.
`ament_python` 패키지이며 `ros2 run ur5e_tools <script>` 로 실행한다.

---

## 파일 구조 및 역할

```
scripts/
├── monitor_data_health.py   ← DataHealthMonitor ROS2 노드 (10Hz 상태 감시)
├── plot_ur_trajectory.py    ← CSV 로그 시각화 (CLI, Matplotlib)
├── motion_editor_gui.py     ← Qt5 50-포즈 모션 에디터 GUI
└── hand_udp_sender_example.py ← UDP 핸드 데이터 생성기 (개발/테스트용)
```

---

## monitor_data_health.py

**클래스**: `DataHealthMonitor(Node)` — 노드 이름 `data_health_monitor`.

구독 토픽:
- `/joint_states` (JointState)
- `/hand/joint_states` (Float64MultiArray)
- `/forward_position_controller/commands` (Float64MultiArray)
- `/system/estop_status` (Bool)

파라미터:
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `check_rate` | `10.0` | 감시 타이머 주파수 (Hz) |
| `timeout_threshold` | `0.2` | 토픽 타임아웃 판단 임계값 (s) |
| `stats_output_dir` | `/tmp/ur5e_stats` | JSON 통계 저장 디렉터리 |
| `enable_stats` | `True` | 통계 수집 활성화 |

셧다운 시 `/tmp/ur5e_stats/` 에 타임스탬프 파일명으로 JSON 저장.

```bash
ros2 run ur5e_tools monitor_data_health.py
# 또는 직접
ros2 launch ur5e_rt_controller ur_control.launch.py  # launch에서 자동 시작됨
```

---

## plot_ur_trajectory.py

CSV 로그(`/tmp/ur5e_control_log.csv`)를 시각화하는 **CLI 도구**. ROS2 불필요.

CSV 컬럼 형식:
```
timestamp, current_pos_0..5, target_pos_0..5, command_0..5[, compute_time_us]
```

```bash
# 모든 관절 (6개 subplot)
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv

# 특정 관절만
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --joint 2

# 이미지 저장
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --save-dir ~/plots

# 통계만 출력 (Matplotlib 없이)
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --stats
```

통계 분석 (pandas 직접 사용):
```python
import pandas as pd
df = pd.read_csv('/tmp/ur5e_control_log.csv')
print(df['compute_time_us'].describe())
print(f'P95: {df["compute_time_us"].quantile(0.95):.1f} us')
print(f'Over 2ms: {(df["compute_time_us"] > 2000).mean()*100:.2f}%')
```

---

## motion_editor_gui.py

**의존성**: `PyQt5` — `sudo apt install python3-pyqt5`.

Qt5 GUI 노드, 토픽:
- 구독: `/joint_states` → 현재 관절 각도 표시
- 발행: `/target_joint_positions` → 선택한 포즈 전송

기능:
- 50개 포즈 저장/로드 (JSON 포맷)
- "Save Current Pose": 현재 `/joint_states` 값을 선택 행에 저장
- "Play Motion Sequence": 선택 행들을 2초 간격으로 순차 실행
- File → Save/Load Motion to/from JSON

JSON 포맷:
```json
{"num_poses": 50, "poses": {"pose_0": [q0,q1,q2,q3,q4,q5], ...}, "names": [...]}
```

```bash
sudo apt install python3-pyqt5
ros2 run ur5e_tools motion_editor_gui.py
```

---

## hand_udp_sender_example.py

실제 핸드 하드웨어 없이 `/hand/joint_states` 시뮬레이션용 UDP 패킷 생성기.

**전송 형식**: 77 double (616 bytes) → `ur5e_hand_udp`의 수신 포맷과 동일.

```bash
ros2 run ur5e_tools hand_udp_sender_example.py
# → 1) 사인파 (동적) / 2) 고정 포즈 (정적) 선택 후 500Hz 전송
```

사용 시나리오:
- `hand_udp_receiver_node`가 실행 중일 때 이 스크립트를 실행하면
  `/hand/joint_states` 토픽이 활성화되어 `custom_controller`의 핸드 타임아웃 E-STOP 방지.

---

## 이 패키지를 수정할 때 주의사항

1. **CSV 컬럼 추가** (C++ 측 LogEntry 변경 시) → `plot_ur_trajectory.py`의 컬럼 참조 갱신 필요.

2. **monitor_data_health.py의 타임아웃** — `timeout_threshold` 파라미터 기본값(0.2s)은
   `ur5e_rt_controller.yaml`의 `robot_timeout_ms: 100` 보다 느슨하게 설정되어 있음.
   두 값을 일관성 있게 유지할 것.

3. **motion_editor_gui.py의 포즈 재생 간격** — 현재 2초 하드코딩.
   변경 시 E-STOP watchdog(robot_timeout_ms: 100ms)이 발동하지 않도록 충분한 간격 유지.

4. **hand_udp_sender_example.py 전송 대상** — 기본값 `127.0.0.1:50001`.
   `hand_udp_receiver_node`와 포트 일치 필요. 실제 핸드와 동시 실행 금지.

5. **패키지 빌드 불필요** — `--symlink-install` 사용 시 스크립트 수정이 바로 반영됨.
   Python 의존성: `matplotlib`, `pandas`, `numpy`, `scipy`, `PyQt5`.
