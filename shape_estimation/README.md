# shape_estimation

**Version 5.17.0** | `ament_cmake` | C++20

ToF(Time-of-Flight) 센서 기반 물체 형상 추정 패키지. UR5e + 커스텀 핸드 시스템의 핑거팁 ToF 센서(6개)로부터 포인트 클라우드를 누적하고, 최소제곱 프리미티브 피팅(Sphere, Cylinder, Plane, Box)과 돌출 구조 탐지를 수행한다. 자율 탐색 모션 생성기를 내장하여 Action Server를 통한 능동적 물체 탐색도 지원한다.

---

## Architecture

```
[RT Controller 500Hz]                      [BT / GUI]
    |  /tof/snapshot (ToFSnapshot)              |  /shape/explore (Action)
    v                                           v
+==========================================+
| ShapeEstimationNode (LifecycleNode)      |
|  State: Stopped/Running/Paused/SingleShot|
|                                          |
|  +-- VoxelPointCloud ---- 2mm voxel accumulation + expiry
|  +-- SnapshotHistory ---- time-series buffer (gap analysis)
|  +-- FastShapeClassifier - curvature-based rule classifier
|  +-- PrimitiveFitter ----- SVD/PCA least-squares fitting
|  +-- ProtuberanceDetector  residual clustering + gap correlation
|  +-- ExplorationMotionGenerator  FSM waypoint generator
|                                          |
|  Pub: /shape/estimate, /shape/point_cloud, /shape/primitive_marker
|       /shape/tof_beams, /shape/curvature_text, /shape/protuberance_marker
|  Sub: /tof/snapshot, /shape/trigger, /ur5e/gui_position, /system/estop_status
|  Srv: /shape/clear                       |
|  Act: /shape/explore (ExploreShape)      |
+==========================================+
```

### 3-Layer Structure

| Layer | Library | Role | ROS Dependency |
|-------|---------|------|----------------|
| **Core** | `shape_estimation_core` | Voxel, classifier, fitter, protuberance, exploration FSM | None (Eigen only) |
| **ROS** | `shape_estimation_ros` | Message conversion, RViz markers, node logic | rclcpp, visualization_msgs, etc. |
| **Executable** | `shape_estimation_node` | Entry point | rclcpp |

---

## Core Components

### VoxelPointCloud

Spatial hashing 기반 포인트 클라우드 관리자.

- **2mm voxel**: 중복 제거 + 이동 평균 업데이트 (`n/(n+1)` old + `1/(n+1)` new)
- **시간 기반 만료**: `point_expiry_sec` (기본 5s) 이후 자동 제거
- **최대 포인트 제한**: `max_points` (기본 2048) 초과 시 oldest 제거
- **64-bit hash**: 21-bit per axis (`(ix << 42) | (iy << 21) | iz`)

### FastShapeClassifier

매 스냅샷마다 호출 가능한 저비용 곡률 기반 rule-based 분류기.

| Condition | Result |
|-----------|--------|
| `max_curvature < 5.0 m^-1` | PLANE (반지름 >= 200mm) |
| Positive uniform curvature (`std < 2.0 m^-1`) | SPHERE |
| Index/middle curvature similar, thumb different | CYLINDER |
| Otherwise | UNKNOWN |

### PrimitiveFitter

최소제곱 기반 프리미티브 피팅 엔진. 4가지 후보를 모두 피팅한 후 최고 confidence를 선택.

| Primitive | Algorithm | Min Points | Confidence |
|-----------|-----------|------------|------------|
| Sphere | Algebraic SVD (`Ax = b`) | 4 | `1 - RMS/radius` |
| Cylinder | PCA axis + 2D circle LS | 5 | `1 - RMS/radius` |
| Plane | SVD/PCA (min eigenvalue) | 3 | `0.5*thickness + 0.5*normal_consistency` |
| Box | PCA-based OBB | 6 | `1 - RMS/avg_dim` |

### ProtuberanceDetector

피팅된 primitive 표면의 돌출 구조를 탐지하는 4-step 파이프라인.

1. **Signed residual**: primitive 표면에서의 부호 거리 (음수 = 돌출)
2. **Union-Find clustering**: 음의 잔차 포인트 공간 클러스터링
3. **Gap detection**: ToF 시계열에서 센서 invalid 구간 분석
4. **Protuberance build**: 클러스터 + gap 상관 -> 돌출 구조 생성

Confidence: `0.3*min(N/10, 1) + 0.4*min(depth/0.02, 1) + 0.3*[has_gap]`

### ExplorationMotionGenerator

ROS 비의존 순수 C++ FSM 기반 탐색 모션 생성기. `enable_exploration: true` 시 Action Server와 연동.

| Phase | Purpose | Transition |
|-------|---------|------------|
| **Approach** | 물체 방향 step-by-step 접근 | ToF 감지 or timeout(5s) |
| **Servo** | ToF 거리 기반 접근 서보 | 수렴 or timeout(3s) |
| **SweepX** | X방향 lateral sweep (+-30mm) | Sweep 완료 |
| **SweepY** | Y방향 lateral sweep | Sweep 완료 |
| **Tilt** | 손목 틸트 스캔 (+-15deg) | Steps 완료 |
| **Evaluate** | Confidence/points 평가 | 성공 or 재시도(max 3 cycles) |

---

## ROS 2 Interfaces

### Subscriptions

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/tof/snapshot` | `rtc_msgs/ToFSnapshot` | SensorData(5) | ToF + fingertip pose @500Hz |
| `/shape/trigger` | `std_msgs/String` | Reliable | `start`/`stop`/`pause`/`resume`/`single` |
| `/ur5e/gui_position` | `rtc_msgs/GuiPosition` | SensorData | Current EE pose (exploration) |
| `/system/estop_status` | `std_msgs/Bool` | Reliable | E-STOP signal |
| `/object/pose_estimate` | `geometry_msgs/PoseStamped` | Reliable | Object position override |

### Publications

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/shape/estimate` | `ShapeEstimate` | ~10Hz | Best shape fit + protuberance |
| `/shape/point_cloud` | `PointCloud2` | ~5Hz | Accumulated voxel cloud |
| `/shape/primitive_marker` | `MarkerArray` | ~5Hz | Primitive visualization |
| `/shape/tof_beams` | `MarkerArray` | ~5Hz | ToF beam arrows |
| `/shape/curvature_text` | `MarkerArray` | ~5Hz | Curvature labels |
| `/shape/protuberance_marker` | `MarkerArray` | ~5Hz | Protrusion visualization |
| `/shape/explore_status` | `MarkerArray` | ~10Hz | Exploration phase/stats |

### Service

| Service | Direction | Type | Description |
|---------|-----------|------|-------------|
| `/shape/clear` | server | `std_srvs/Trigger` | Clear point cloud & reset |
| `/rtc_cm/switch_controller` | client | `rtc_msgs/srv/SwitchController` | Switch RT controller before exploration starts (sync, single-active per D-A1) |

### Action (enable_exploration: true)

| Action | Type | Description |
|--------|------|-------------|
| `/shape/explore` | `ExploreShape` | Autonomous exploration |

> **2026-04-26 migration note.** The legacy publisher to
> `/<robot_ns>/controller_type` (which the action server used to switch the
> RT controller before launching exploration) was retired in `55b10f5` —
> the topic itself was removed, so for a brief window between commits
> `55b10f5` and `F-1` the action's controller switch was a silent no-op
> against a non-existent topic. The action now uses the `SwitchController`
> srv synchronously and aborts cleanly on `ok=false`.

---

## Quick Start

```bash
# Launch (node + RViz2)
ros2 launch shape_estimation shape_estimation.launch.py

# Without RViz
ros2 launch shape_estimation shape_estimation.launch.py use_rviz:=false

# Custom config
ros2 launch shape_estimation shape_estimation.launch.py config_file:=/path/to/custom.yaml

# State control
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'start'" --once
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'stop'" --once
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'pause'" --once
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'single'" --once

# Clear point cloud
ros2 service call /shape/clear std_srvs/srv/Trigger

# Autonomous exploration
ros2 action send_goal /shape/explore shape_estimation_msgs/action/ExploreShape \
  "{object_position: {x: 0.4, y: 0.0, z: 0.3}, confidence_threshold: 0.8}"
```

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `shape_estimation_msgs` | Shape/Explore message & action types (3 msg + 1 action) |
| `rtc_msgs` | GuiPosition, RobotTarget, ToFSnapshot messages |
| `Eigen3` | Linear algebra (SVD, PCA, geometry) |
| `rclcpp` / `rclcpp_action` | ROS 2 C++ client + action server |
| `sensor_msgs` | PointCloud2 |
| `visualization_msgs` | MarkerArray |
| `std_srvs` | Trigger service |

---

## Tests

```bash
colcon build --packages-select shape_estimation --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select shape_estimation
colcon test-result --verbose --all
```

3 test suites: `test_tof_shape`, `test_exploration_motion`, `test_protuberance_detector`

---

## Logging

### 분류 독트린

| 레벨 | 용도 | 예시 |
|------|------|------|
| `FATAL` | 프로세스를 계속 실행할 수 없는 상태 | (현재 없음 — 치명적 초기화 실패 시 예약) |
| `ERROR` | 복구 불가능한 실패, 사용자 개입 필요 | 탐색 전체 타임아웃, action 실패 |
| `WARN` | 복구 가능한 이상 상태, 자동 재시도 중 | E-STOP, approach 타임아웃, 수치적 비정상 (r²<0), 알 수 없는 trigger 명령 |
| `INFO` | 사용자가 알아야 할 1 Hz 미만 상태 전환 | 노드 초기화, trigger 명령 수신, FSM phase 전이, 탐색 성공/취소 |
| `DEBUG` | 개발자 진단용 (기본 꺼짐), 고빈도 허용 | ToF 콜백 상태, voxel 업데이트, fit 결과 상세 |

**핵심 규칙**:

- ToF 콜백 안의 로그는 **`DEBUG`로 격리 + `_THROTTLE` 적용**. 기본 실행 시 조용해야 한다.
- 초기 포인트 축적 중 발생하는 "포인트 부족" 메시지는 `DEBUG`. 누적되면 자동 해소되는 정상 상태.
- 수치적 비정상 (음수 `r²`, 음수 `radius²` 등)은 `WARN`. 피팅 알고리즘의 실제 오류 신호.
- 메시지 본문에 수동 `[ModuleName]` / `"XX 피팅:"` 접두사를 붙이지 않는다. 서브-로거 이름이 곧 식별자다.
- THROTTLE 주기는 매직넘버 대신 `shape_logging.hpp`의 표준 상수를 사용한다.

### 서브-로거 네임스페이스

모든 로그는 단일 노드 logger가 아닌 계층적 서브-로거를 사용한다. 이로써 런타임에 서브시스템별 필터링이 가능하다.

| 서브-로거 | 사용처 |
|-----------|--------|
| `shape.node` | `ShapeEstimationNode` (콜백, action server, trigger 서비스, 초기화) |
| `shape.voxel` | `VoxelPointCloud`, `SnapshotHistory` (🔴 핫패스 — 기본 꺼짐 권장) |
| `shape.classify` | `FastShapeClassifier` (곡률 기반 rule 분류) |
| `shape.fit` | `PrimitiveFitter` (Sphere/Cylinder/Plane/Box SVD/PCA) |
| `shape.protus` | `ProtuberanceDetector` (signed residual, clustering, gap) |
| `shape.explore` | `ExplorationMotionGenerator` (5-phase FSM) |

### THROTTLE 주기 표준

`rtc::shape::logging` 네임스페이스에 정의된 상수만 사용한다 (`include/shape_estimation/shape_logging.hpp`):

| 상수 | 값 [ms] | 용도 |
|------|---------|------|
| `kThrottleFastMs` | 500 | 빠른 진행 상태 (fit 결과, EstimateShape 선택) |
| `kThrottleSlowMs` | 2000 | 일반 반복 경고 (빈 snapshot, voxel update 통계) |
| `kThrottleIdleMs` | 10000 | 장기 유휴 상태 |

### 실시간 필터링 예시

```bash
# 핫패스(voxel) DEBUG만 활성화
ros2 service call /shape_estimation_node/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'shape.voxel', level: 10}]}"

# 피팅 전체(sphere/cylinder/plane/box 모두) DEBUG
ros2 service call /shape_estimation_node/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'shape.fit', level: 10}]}"

# 탐색 FSM 전이만 조용히 (INFO → WARN)
ros2 service call /shape_estimation_node/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'shape.explore', level: 30}]}"
```

콘솔 출력에 로거 이름을 표시하려면 환경변수 설정:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

---

## Additional Documentation

| Document | Contents |
|----------|----------|
| [docs/ALGORITHMS.md](docs/ALGORITHMS.md) | Core algorithm details (voxel, fitting, classifier, protuberance, exploration FSM) |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | All parameters with tuning guide and scenario-specific presets |
| [docs/DEBUGGING.md](docs/DEBUGGING.md) | Troubleshooting, diagnostic commands, common issues |

---

## License

MIT License
