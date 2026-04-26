# Shape Estimation Debugging Guide

shape_estimation 패키지의 문제 진단, 디버깅 명령어, 자주 발생하는 문제와 해결법.

---

## Table of Contents

1. [Diagnostic Commands](#diagnostic-commands)
2. [Common Issues](#common-issues)
3. [RViz Visualization Debugging](#rviz-visualization-debugging)
4. [Exploration Debugging](#exploration-debugging)
5. [Performance Tuning](#performance-tuning)

---

## Diagnostic Commands

### Node Status

```bash
# 노드 실행 확인
ros2 node list | grep shape

# 파라미터 확인
ros2 param list /shape_estimation_node
ros2 param get /shape_estimation_node voxel_resolution

# 토픽 확인
ros2 topic list | grep shape
```

### Data Flow Check

```bash
# ToF 스냅샷 수신 확인 (500Hz expected from RT controller)
ros2 topic hz /tof/snapshot

# 추정 결과 발행 확인 (~10Hz)
ros2 topic hz /shape/estimate

# 추정 결과 내용 확인
ros2 topic echo /shape/estimate

# 포인트 클라우드 발행 확인 (~5Hz)
ros2 topic hz /shape/point_cloud
```

### Shape Estimation Status

```bash
# 현재 형상 추정 결과 (1회)
ros2 topic echo /shape/estimate --once

# confidence 모니터링 (연속)
ros2 topic echo /shape/estimate --field confidence

# shape_type 모니터링
ros2 topic echo /shape/estimate --field shape_type
# 0=UNKNOWN, 1=PLANE, 2=SPHERE, 3=CYLINDER, 4=BOX

# 사용된 포인트 수
ros2 topic echo /shape/estimate --field num_points_used

# 돌출 탐지 상태
ros2 topic echo /shape/estimate --field has_protuberance
ros2 topic echo /shape/estimate --field protuberance_confidence
```

### State Control

```bash
# 추정 시작
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'start'" --once

# 추정 중지
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'stop'" --once

# 일시 정지 / 재개
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'pause'" --once
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'resume'" --once

# 단일 스냅샷 처리 (디버깅용)
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'single'" --once

# 포인트 클라우드 초기화
ros2 service call /shape/clear std_srvs/srv/Trigger
```

### ToF Data Inspection

```bash
# ToF 스냅샷 raw 데이터 확인
ros2 topic echo /tof/snapshot --field distances
ros2 topic echo /tof/snapshot --field valid

# 곡률 확인 (from estimate)
ros2 topic echo /shape/estimate --field local_curvatures
ros2 topic echo /shape/estimate --field curvature_valid
```

### Exploration Diagnostics

```bash
# 탐색 Action 호출
ros2 action send_goal --feedback /shape/explore \
  shape_estimation_msgs/action/ExploreShape \
  "{object_position: {x: 0.4, y: 0.0, z: 0.3}, confidence_threshold: 0.8}"

# 탐색 Action 취소
ros2 action send_goal /shape/explore \
  shape_estimation_msgs/action/ExploreShape \
  "{object_position: {x: 0.0, y: 0.0, z: 0.0}}" --cancel

# E-STOP 상태 확인
ros2 topic echo /system/estop_status

# 현재 EE 위치 확인
ros2 topic echo /ur5e/gui_position --once

# 컨트롤러 전환 확인
ros2 topic echo /rtc_cm/active_controller_name
```

---

## Common Issues

### 1. "No estimate published" — 추정 결과가 나오지 않음

**증상**: `/shape/estimate` 토픽에 메시지가 없거나 `shape_type = 0 (UNKNOWN)`.

**원인 & 해결**:

| Check | Command | Fix |
|-------|---------|-----|
| Node state = Stopped | 로그에 `State: kStopped` | `ros2 topic pub /shape/trigger ... "data: 'start'" --once` |
| ToF snapshot 미수신 | `ros2 topic hz /tof/snapshot` → 0Hz | RT controller 또는 ToF publish 확인 |
| 모든 센서 invalid | `ros2 topic echo /tof/snapshot --field valid` → all false | 센서 하드웨어, FK 계산, range 확인 |
| 포인트 부족 | estimate의 `num_points_used < min_points_for_fitting` | 더 많은 표면 접촉, `min_points_for_fitting` 낮춤 |
| 모든 confidence < 0.3 | estimate의 `confidence = 0` | 측정 노이즈 확인, threshold 조정 |

### 2. "Wrong shape type" — 잘못된 형상 분류

**증상**: 구인데 PLANE으로 분류, 또는 실린더인데 SPHERE로 분류.

**원인 & 해결**:

| Misclassification | Likely Cause | Fix |
|-------------------|-------------|-----|
| 구 → PLANE | 큰 반지름 (>200mm) | `flat_curvature_threshold` 낮춤 (예: 2.0) |
| 평면 → SPHERE | 센서 노이즈 → 곡률 편향 | `flat_curvature_threshold` 높임 (예: 8.0) |
| 실린더 → SPHERE | 균일 곡률로 판정 | 다양한 각도에서 스캔 (tilt phase) |
| 모든 것 → UNKNOWN | 포인트 부족 / 노이즈 | `min_points_for_fitting` 낮춤, 센서 품질 확인 |

**디버깅 절차**:
1. 곡률 값 확인: `ros2 topic echo /shape/estimate --field local_curvatures`
2. RViz에서 curvature text 마커 확인
3. 포인트 클라우드 분포 확인 (한쪽 면만 있는지, 골고루 분포되었는지)

### 3. "Protuberance false positive" — 돌출 구조 오탐

**증상**: 평면이나 구에서 돌출 구조가 탐지됨.

**원인 & 해결**:

| Cause | Symptom | Fix |
|-------|---------|-----|
| 센서 노이즈 | 잔차가 threshold 근처에서 진동 | `residual_threshold` 절대값 증가 (예: -0.010) |
| 포인트 수 부족 | 부정확한 primitive fit → 큰 잔차 | `min_points_for_fitting` 증가 (예: 20) |
| Gap 오탐 | 센서 flickering | `min_gap_invalid_count` 증가 (예: 4) |
| 클러스터 파편화 | 여러 작은 클러스터 | `cluster_radius` 증가 (예: 0.025) |

### 4. "Point cloud not accumulating" — 포인트가 누적되지 않음

**증상**: `num_points_used`가 0이거나 매우 낮음.

**Checklist**:

```bash
# 1. ToF 데이터 수신 확인
ros2 topic hz /tof/snapshot

# 2. 센서 유효성 확인
ros2 topic echo /tof/snapshot --field valid --once
# → 최소 1개 이상 true여야 함

# 3. 거리 범위 확인 (10mm ~ 300mm)
ros2 topic echo /tof/snapshot --field distances --once

# 4. 포인트 만료가 너무 빠른지 확인
ros2 param get /shape_estimation_node point_expiry_sec

# 5. 포인트 클라우드 초기화 후 재시작
ros2 service call /shape/clear std_srvs/srv/Trigger
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'start'" --once
```

### 5. "Exploration fails immediately" — 탐색이 즉시 실패

**증상**: Action result가 `success: false`로 즉시 반환.

| Cause | Check | Fix |
|-------|-------|-----|
| `enable_exploration: false` | YAML 설정 | `enable_exploration: true` |
| E-STOP 활성 | `ros2 topic echo /system/estop_status` | E-STOP 해제 |
| GuiPosition 미수신 | `ros2 topic hz /ur5e/gui_position` | RT controller 실행 확인 |
| 컨트롤러 전환 실패 | `ros2 topic echo /rtc_cm/active_controller_name` | controller_name 설정 확인 |
| 물체 위치 미설정 | goal의 `object_position`이 원점 | 유효한 좌표 입력 |

### 6. "Exploration stuck in phase" — 탐색이 특정 단계에서 멈춤

| Phase | Stuck Condition | Fix |
|-------|----------------|-----|
| **Approach** | ToF가 물체 감지 못함 | `approach_timeout_sec` 확인, 물체 위치/방향 재설정 |
| **Servo** | 수렴하지 않음 | `servo_converge_tol` 완화 (예: 0.005), `servo_step_gain` 조정 |
| **Sweep** | Edge 미도달 | `sweep_width` 감소, sweep_step 확인 |
| **Evaluate** | Confidence 부족 | `confidence_threshold` 낮춤, `max_sweep_cycles` 증가 |

### 7. "High latency / low publish rate" — 지연 또는 낮은 publish rate

**증상**: `/shape/estimate`가 10Hz 미만으로 발행.

**Checklist**:

```bash
# 피팅 포인트 수 확인 (> 1000 이면 느려질 수 있음)
ros2 topic echo /shape/estimate --field num_points_used

# viz_rate가 높으면 부하 증가
ros2 param get /shape_estimation_node viz_rate_hz
```

**해결**:
- `max_points` 감소 (예: 1024)
- `voxel_resolution` 증가 (예: 0.005) → 포인트 수 감소
- `viz_rate_hz` 감소 (예: 2.0)
- `publish_rate_hz` 조정

---

## RViz Visualization Debugging

### 필수 토픽 추가 (RViz Displays)

| Display Type | Topic | Description |
|-------------|-------|-------------|
| PointCloud2 | `/shape/point_cloud` | 누적 포인트 (흰색 점) |
| MarkerArray | `/shape/primitive_marker` | 피팅된 프리미티브 (반투명 형상) |
| MarkerArray | `/shape/tof_beams` | ToF 빔 방향 (화살표) |
| MarkerArray | `/shape/curvature_text` | 곡률 값 (텍스트 라벨) |
| MarkerArray | `/shape/protuberance_marker` | 돌출 구조 (빨간 구/화살표) |
| MarkerArray | `/shape/explore_status` | 탐색 상태 (phase 텍스트) |

### 시각화 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| 마커가 안 보임 | Fixed Frame 불일치 | RViz `Fixed Frame` = `base_link` 설정 |
| 포인트가 이상한 위치 | TF tree 불완전 | `ros2 run tf2_tools view_frames` 확인 |
| 마커 크기 0 | 추정 confidence = 0 | confidence > 0 확인 |
| 이전 마커 잔상 | Marker lifetime | `/shape/clear` 서비스 호출 |

### RViz Color Coding

| Marker | Color | Meaning |
|--------|-------|---------|
| Primitive (구/실린더/평면/박스) | Green (반투명) | 피팅 결과 |
| ToF beam | Blue (valid) / Red (invalid) | 센서 상태 |
| Curvature text | White | 곡률 값 |
| Protuberance | Red | 돌출 구조 위치 |
| Explore status | Yellow | 현재 탐색 phase |

---

## Exploration Debugging

### Phase 별 모니터링

```bash
# Action feedback에서 phase 모니터링
ros2 action send_goal --feedback /shape/explore \
  shape_estimation_msgs/action/ExploreShape \
  "{object_position: {x: 0.4, y: 0.0, z: 0.3}}"

# Feedback 출력 예시:
# current_phase: 1    ← kApproach
# elapsed_sec: 2.3
# num_points_collected: 45
# status_message: "Approaching object..."
```

**Phase 번호 매핑**:

| Value | Phase | Description |
|-------|-------|-------------|
| 0 | kIdle | 대기 |
| 1 | kApproach | 접근 중 |
| 2 | kServo | 거리 서보 |
| 3 | kSweepX | X방향 sweep |
| 4 | kSweepY | Y방향 sweep |
| 5 | kTilt | 틸트 스캔 |
| 6 | kEvaluate | 결과 평가 |
| 7 | kSucceeded | 성공 |
| 8 | kFailed | 실패 |
| 9 | kAborted | 중단 |

### 탐색 시 안전 문제

```bash
# E-STOP 실시간 모니터링
ros2 topic echo /system/estop_status

# 현재 발행되는 목표 위치 확인
ros2 topic echo /ur5e/joint_goal

# active 컨트롤러의 게인 파라미터 확인
ros2 param list /demo_task_controller
ros2 param get /demo_task_controller trajectory_speed
```

### 탐색 Action 취소

```bash
# CLI에서 Ctrl+C로 취소 (ros2 action send_goal 사용 시)
# 또는 프로그래밍 방식:
# goal_handle.cancel_goal()
```

---

## Performance Tuning

### CPU 사용량 최적화

```bash
# shape_estimation_node CPU 사용량 확인
top -p $(pgrep -f shape_estimation)
```

| Factor | Impact | Mitigation |
|--------|--------|------------|
| `max_points` > 2048 | 피팅 O(N) 증가 | 필요한 만큼만 설정 |
| `viz_rate_hz` > 5 | 마커 생성 부하 | 디버깅 후 2~5Hz로 낮춤 |
| `publish_rate_hz` > 20 | DDS 직렬화 부하 | 10Hz면 대부분 충분 |
| 6개 센서 모두 valid | 매 cycle 6 voxel update | 정상 동작, 최적화 불필요 |

### Memory 사용량

| Component | Memory | Note |
|-----------|--------|------|
| VoxelPointCloud (2048) | ~200 KB | `PointWithNormal` * 2048 + hash overhead |
| SnapshotHistory (300) | ~100 KB | `ToFSnapshot` * 300 |
| Markers | Variable | 포인트 수에 비례 |

### QoS Tuning

shape_estimation_node의 QoS는 코드에 하드코딩:

| Subscription | QoS | Reason |
|-------------|-----|--------|
| `/tof/snapshot` | SensorData (BEST_EFFORT, keep_last=5) | RT 데이터 손실 허용, 최신 우선 |
| `/shape/trigger` | Reliable (keep_last=10) | 명령 손실 방지 |
| `/system/estop_status` | Reliable | 안전 신호 |

Publication은 모두 기본 QoS (Reliable, keep_last=10).

---

## Log Messages Reference

### INFO Level

| Message | Meaning |
|---------|---------|
| `State changed to: Running` | trigger "start" 처리 |
| `Cleared N points` | clear 서비스 처리 |
| `Exploration started` | Action goal 수락 |
| `Phase: APPROACH → SERVO` | FSM 전이 |
| `Exploration succeeded: confidence=0.85` | 탐색 성공 |

### WARN Level

| Message | Meaning | Action |
|---------|---------|--------|
| `Goal rejected: step too large` | ValidateGoal 실패 | max_step_size 확인 |
| `Servo: insufficient valid sensors` | 유효 센서 < min | 센서 범위/방향 확인 |
| `Exploration timeout` | max_total_time 초과 | 시간 증가 또는 threshold 낮춤 |

### ERROR Level

| Message | Meaning | Action |
|---------|---------|--------|
| `E-STOP active, aborting exploration` | E-STOP 수신 | E-STOP 원인 해결 |
| `No GUI position available` | EE 위치 미수신 | RT controller 확인 |
| `Controller switch failed` | 컨트롤러 전환 실패 | controller_name 확인 |
