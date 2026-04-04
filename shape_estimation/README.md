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
| ShapeEstimationNode                      |
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
| `/tof/snapshot` | `shape_estimation_msgs/ToFSnapshot` | SensorData(5) | ToF + fingertip pose @500Hz |
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

| Service | Type | Description |
|---------|------|-------------|
| `/shape/clear` | `std_srvs/Trigger` | Clear point cloud & reset |

### Action (enable_exploration: true)

| Action | Type | Description |
|--------|------|-------------|
| `/shape/explore` | `ExploreShape` | Autonomous exploration |

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
| `shape_estimation_msgs` | ToF/Shape/Explore message & action types (4 msg + 1 action) |
| `rtc_msgs` | GuiPosition, RobotTarget messages |
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

## Additional Documentation

| Document | Contents |
|----------|----------|
| [docs/ALGORITHMS.md](docs/ALGORITHMS.md) | Core algorithm details (voxel, fitting, classifier, protuberance, exploration FSM) |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | All parameters with tuning guide and scenario-specific presets |
| [docs/DEBUGGING.md](docs/DEBUGGING.md) | Troubleshooting, diagnostic commands, common issues |

---

## License

MIT License
