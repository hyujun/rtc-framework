# shape_estimation

**버전 5.17.0** | `ament_cmake` | C++20

ToF(Time-of-Flight) 센서 기반 물체 형상 추정 패키지입니다. UR5e + 커스텀 핸드 시스템의 핑거팁 ToF 센서(6개)로부터 포인트 클라우드를 누적하고, 최소제곱 프리미티브 피팅(구, 실린더, 평면, 박스)을 수행합니다.

---

## 아키텍처

```
[RT Controller 500Hz]
    │  /shape/tof_snapshot (ToFSnapshot)
    ▼
[ShapeEstimationNode]  ← /shape/trigger (start/stop/single_shot/pause)
    │
    ├── VoxelPointCloud       2mm 복셀 기반 포인트 누적 + 시간 만료
    ├── FastShapeClassifier   곡률 기반 rule-based 1차 분류
    └── PrimitiveFitter       SVD/PCA 기반 최소제곱 피팅
    │
    ├──→ /shape/estimate     (ShapeEstimate)
    ├──→ /shape/point_cloud  (PointCloud2)
    ├──→ /shape/primitive    (MarkerArray)
    ├──→ /shape/tof_beams    (MarkerArray)
    └──→ /shape/curvature    (MarkerArray)
```

### 3-레이어 구조

| 레이어 | 라이브러리 | 역할 | ROS 의존성 |
|--------|-----------|------|-----------|
| Core | `shape_estimation_core` | 복셀, 분류, 피팅 알고리즘 | 없음 (Eigen만) |
| ROS | `shape_estimation_ros` | 메시지 변환, RViz 마커 생성 | rclcpp, visualization_msgs 등 |
| Executable | `shape_estimation_node` | ROS 2 노드 진입점 | rclcpp |

---

## 핵심 컴포넌트

### VoxelPointCloud

공간 해싱 기반 포인트 클라우드 관리자입니다.

- **2mm 복셀 해상도**: 중복 제거 + 이동 평균 업데이트
- **시간 기반 만료**: `point_expiry_sec` (기본 5초) 이후 자동 제거
- **최대 포인트 제한**: `max_points` (기본 2048) 초과 시 가장 오래된 포인트 제거

### FastShapeClassifier

곡률 기반 rule-based 1차 형상 분류기입니다. 매 스냅샷마다 호출 가능한 저비용 연산입니다.

- **평면 감지**: 곡률 < 5.0 m⁻¹ (반지름 > 200mm)
- **구 감지**: 양의 균일한 곡률
- **실린더 감지**: 패턴 기반 (index/middle 곡률 유사)

### PrimitiveFitter

최소제곱 기반 프리미티브 피팅 엔진입니다.

| 형상 | 알고리즘 | 최소 포인트 |
|------|---------|-----------|
| Sphere | 대수적 SVD 피팅 | 4 |
| Cylinder | PCA 축 추정 → 2D circle fit | 5 |
| Plane | SVD 기반 | 3 |
| Box | PCA 기반 OBB | 6 |

---

## 토픽 & 서비스

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/shape/tof_snapshot` | `shape_estimation_msgs/ToFSnapshot` | ToF 센서 + 핑거팁 자세 통합 스냅샷 |
| `/shape/trigger` | `std_msgs/String` | 상태 제어 (`start`, `stop`, `pause`, `single_shot`) |

### 발행 토픽

| 토픽 | 타입 | 주기 | 설명 |
|------|------|------|------|
| `/shape/estimate` | `shape_estimation_msgs/ShapeEstimate` | ~10Hz | 형상 추정 결과 |
| `/shape/point_cloud` | `sensor_msgs/PointCloud2` | ~5Hz | 누적 포인트 클라우드 |
| `/shape/primitive` | `visualization_msgs/MarkerArray` | ~5Hz | 프리미티브 마커 (RViz) |
| `/shape/tof_beams` | `visualization_msgs/MarkerArray` | ~5Hz | ToF 빔 시각화 |
| `/shape/curvature` | `visualization_msgs/MarkerArray` | ~5Hz | 곡률 텍스트 라벨 |

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/shape/clear` | `std_srvs/Trigger` | 포인트 클라우드 초기화 |

---

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `voxel_resolution` | double | 0.002 | 복셀 해상도 [m] |
| `max_points` | int | 2048 | 최대 포인트 수 |
| `point_expiry_sec` | double | 5.0 | 포인트 만료 시간 [s] |
| `flat_curvature_threshold` | double | 5.0 | 평면 판정 곡률 임계값 [1/m] |
| `curvature_uniformity_threshold` | double | 2.0 | 곡률 균일성 임계값 |
| `min_points_for_fitting` | int | 10 | 피팅 최소 포인트 수 |
| `publish_rate_hz` | double | 10.0 | 추정 결과 발행 주기 |
| `viz_rate_hz` | double | 5.0 | 시각화 발행 주기 |
| `frame_id` | string | `base_link` | 기준 좌표 프레임 |

---

## 실행

```bash
# launch 파일 (shape_estimation_node + RViz2)
ros2 launch shape_estimation shape_estimation.launch.py

# RViz 없이 노드만 실행
ros2 launch shape_estimation shape_estimation.launch.py use_rviz:=false

# 상태 제어
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'start'" --once
ros2 topic pub /shape/trigger std_msgs/msg/String "data: 'stop'" --once

# 포인트 클라우드 초기화
ros2 service call /shape/clear std_srvs/srv/Trigger
```

---

## 의존성

| 패키지 | 용도 |
|--------|------|
| `shape_estimation_msgs` | ToF/형상 메시지 타입 |
| `Eigen3` | 선형대수 연산 |
| `rclcpp` | ROS 2 C++ 클라이언트 |
| `sensor_msgs` | PointCloud2 |
| `visualization_msgs` | RViz MarkerArray |
| `std_srvs` | Trigger 서비스 |

---

## 테스트

```bash
colcon build --packages-select shape_estimation --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select shape_estimation
colcon test-result --verbose --all
```

---

## 라이선스

MIT License
