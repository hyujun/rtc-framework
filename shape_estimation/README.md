# shape_estimation

핑거팁 ToF 센서로 물체 형상(Sphere · Cylinder · Plane · Box)과 돌출 구조를 추정하는 lifecycle 노드. **유형**: 노드 패키지

## 개요

active 컨트롤러가 발행하는 ToF 스냅샷(핑거팁 ToF 6개 + 핑거팁 pose)을 voxel 포인트 클라우드로 누적하고, 곡률 기반 분류와 최소제곱 프리미티브 피팅으로 형상을 추정한다. `enable_exploration: true` 이면 능동 탐색 모션을 생성하는 `/shape/explore` action server 도 제공한다.

- 구성: 알고리즘 코어(`shape_estimation_core`, ROS 노드 API 비의존) · ROS 변환과 RViz marker(`shape_estimation_ros`) · 실행파일(`shape_estimation_node`).
- 알고리즘 5개(voxel · 분류기 · 피팅 · 돌출 탐지 · 탐색 FSM)의 수식과 임계는 [docs/ALGORITHMS.md](docs/ALGORITHMS.md) 가 갖는다.
- 소비자: [ur5e_bt_coordinator](../ur5e_bt_coordinator/README.md) 의 shape 노드군(`TriggerShapeEstimation` · `WaitShapeResult` · `CheckShapeType`). 메시지: [shape_estimation_msgs](../shape_estimation_msgs/README.md).

```
active 컨트롤러 ──/<ctrl>/tof/snapshot──► ShapeEstimationNode ──/shape/estimate──► BT / GUI
BT / GUI ──/shape/trigger, /shape/explore (action)──►        └─ /shape/* RViz marker · point_cloud
```

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. active 컨트롤러가 ToF 스냅샷을 내고 있어야 한다.

```bash
ros2 launch shape_estimation shape_estimation.launch.py                      # 노드 + RViz2
ros2 launch shape_estimation shape_estimation.launch.py use_rviz:=false
ros2 launch shape_estimation shape_estimation.launch.py config_file:=/path/to/custom.yaml
ros2 action send_goal /shape/explore shape_estimation_msgs/action/ExploreShape \
  "{object_position: {x: 0.4, y: 0.0, z: 0.3}, confidence_threshold: 0.8}"     # 능동 탐색
```

노드는 launch 가 configure · activate 까지 자동으로 진행한다. 상태 제어(`start` · `stop` · `pause` · `resume` · `single`), point cloud clear, 진단 명령은 [docs/DEBUGGING.md](docs/DEBUGGING.md).

## 인터페이스

### 실행파일 · launch

| 이름 | 설명 |
|---|---|
| `shape_estimation_node` | 형상 추정 `LifecycleNode` |
| `shape_estimation.launch.py` | 노드 + RViz2. 인자 `use_rviz` (기본 `true`) · `config_file` · `rviz_config` |
| `shape_estimation_node.launch.py` | 노드만. 인자 `config_file` |
| `shape_estimation_rviz.launch.py` | RViz2 만 |

### 구독

| Topic | 타입 | 설명 |
|---|---|---|
| `/<ctrl>/tof/snapshot` | `rtc_msgs/ToFSnapshot` | ToF + 핑거팁 pose. SensorData QoS, depth 5. `/rtc_cm/active_controller_name` 이 바뀔 때마다 `<ctrl>` 을 다시 묶는다 |
| `/shape/trigger` | `std_msgs/String` | `start` · `stop` · `pause` · `resume` · `single` |
| `/system/estop_status` | `std_msgs/Bool` | E-STOP (transient_local) |
| `/rtc_cm/active_controller_name` | `std_msgs/String` | 활성 컨트롤러 이름 (latched) |
| `/object/pose_estimate` | `geometry_msgs/PoseStamped` | 물체 위치 override (탐색 활성 시, transient_local) |

TCP pose 는 토픽이 아니라 tf2 lookup(`base` → `tool0_actual`)으로 읽는다. 탐색 모션이 사용한다.

### 발행

| Topic | 타입 | 주기 | 설명 |
|---|---|---|---|
| `/shape/estimate` | `shape_estimation_msgs/ShapeEstimate` | ~10 Hz | 최선의 형상 피팅 + 돌출 구조 |
| `/shape/point_cloud` | `sensor_msgs/PointCloud2` | ~5 Hz | 누적 voxel 클라우드 |
| `/shape/primitive_marker` · `/shape/tof_beams` · `/shape/curvature_text` · `/shape/protuberance_marker` | `MarkerArray` | ~5 Hz | RViz 시각화 |
| `/shape/explore_status` | `MarkerArray` | ~10 Hz | 탐색 phase · 통계 |

### 서비스 · action

| 이름 | 종류 | 타입 | 설명 |
|---|---|---|---|
| `/shape/clear` | server | `std_srvs/Trigger` | 포인트 클라우드 초기화 |
| `/rtc_cm/switch_controller` | client | `rtc_msgs/srv/SwitchController` | 탐색 시작 전 RT 컨트롤러를 동기 전환한다. 실패(`ok=false`)면 탐색을 중단한다 |
| `/shape/explore` | action server (`enable_exploration: true`) | `shape_estimation_msgs/action/ExploreShape` | 능동 탐색 |

## 설정

[config/shape_estimation_node.yaml](config/shape_estimation_node.yaml) — 기본값은 파일이 갖는다. 전체 파라미터, 튜닝 가이드, 시나리오별 프리셋은 [docs/CONFIGURATION.md](docs/CONFIGURATION.md).

| 키 | 의미 |
|---|---|
| `voxel_resolution` · `max_points` · `point_expiry_sec` | voxel 크기 [m] · 최대 포인트 수 · 포인트 만료 [s] |
| `flat_curvature_threshold` · `curvature_uniformity_threshold` | 분류기 곡률 임계 [1/m] |
| `publish_rate_hz` · `viz_rate_hz` | 추정 결과 · 시각화 발행 주기 [Hz] |
| `protuberance.*` | 돌출 탐지 |
| `enable_exploration` · `exploration.*` | `false` 이면 action server 를 만들지 않고 추정만 한다 |

### Logging (로깅)

레벨 분류 · 로거 네이밍 공통 규칙은 [conventions.md](../agent_docs/conventions.md#logging) 를 따른다. 이 패키지 고유 규칙:

- ToF 콜백 안의 로그는 `DEBUG` 로 격리하고 `_THROTTLE` 을 쓴다. 기본 실행은 조용해야 한다. 주기 상수는 [shape_logging.hpp](include/shape_estimation/shape_logging.hpp) 의 `kThrottle*Ms` 만 쓴다.
- 초기 누적 중의 "포인트 부족" 은 정상 상태이므로 `DEBUG` 다.
- 수치적 비정상(음수 `r²` · `radius²`)은 피팅의 실제 오류 신호이므로 `WARN` 이다.

서브-로거는 `shape.node` · `shape.voxel` (핫패스, 기본 꺼짐 권장) · `shape.classify` · `shape.fit` · `shape.protus` · `shape.explore` 다. 이름 단위로 레벨을 정할 수 있다.

```bash
ros2 run shape_estimation shape_estimation_node --ros-args --log-level shape.voxel:=debug
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"   # 로거 이름 표시
```

## 더 읽기

- [docs/ALGORITHMS.md](docs/ALGORITHMS.md) — voxel · 분류기 · 피팅 · 돌출 탐지 · 탐색 FSM 알고리즘
- [docs/CONFIGURATION.md](docs/CONFIGURATION.md) — 전체 파라미터와 튜닝 프리셋
- [docs/DEBUGGING.md](docs/DEBUGGING.md) — 문제 해결 · 진단 명령
- 테스트 target: [CMakeLists.txt](CMakeLists.txt) 의 `BUILD_TESTING` 블록

## 의존성

[package.xml](package.xml) — `shape_estimation_msgs` · `rtc_msgs` · Eigen3 · `rclcpp_action` · `rclcpp_lifecycle` · tf2. MIT License ([LICENSE](../LICENSE)).
