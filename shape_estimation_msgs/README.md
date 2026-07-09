# shape_estimation_msgs

`ament_cmake` | ROS 2 메시지 패키지

ToF 기반 형상 추정 시스템용 커스텀 ROS 2 메시지 정의 패키지입니다. 핑거팁 ToF 센서 스냅샷, 자세 정보, 형상 프리미티브 추정 결과를 위한 3종의 메시지 타입과 1종의 액션을 제공합니다.

> **Note:** `ToFSnapshot` 메시지는 로봇 독립성을 위해 `rtc_msgs` 패키지로 이동되었습니다.

---

## 메시지 타입

### ToFReadings.msg (reserved / unused)

ToF 센서 원시 데이터용으로 정의되었으나, 현재 repo 전역에 C++/Python 소비자가 없습니다.
런타임 ToF 데이터는 `rtc_msgs/ToFSnapshot`을 사용합니다 (본 패키지 상단 note 참조). 필드는
아래와 같이 남아 있습니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `distances` | `float64[6]` | 6개 센서 거리 [m] (thumb_A, thumb_B, index_A, index_B, middle_A, middle_B) |
| `valid` | `bool[6]` | 6개 센서 유효성 플래그 |

### TipPoses.msg (reserved / unused)

3개 핑거팁의 월드 프레임 SE3 자세 (FK 결과)용으로 정의되었으나, 현재 repo 전역에
C++/Python 소비자가 없습니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `poses` | `geometry_msgs/Pose[3]` | thumb, index, middle 순서 |

### ShapeEstimate.msg

추정된 물체 형상 프리미티브 정보입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `shape_type` | `uint8` | 형상 타입 (0=UNKNOWN, 1=PLANE, 2=SPHERE, 3=CYLINDER, 4=BOX) |
| `confidence` | `float64` | 추정 신뢰도 [0, 1] |
| `center` | `geometry_msgs/Point` | 프리미티브 중심 위치 (월드 프레임) |
| `axis` | `geometry_msgs/Vector3` | 주축 방향 (cylinder 축 / plane 법선) |
| `radius` | `float64` | 반지름 [m] (sphere, cylinder) |
| `dimensions` | `geometry_msgs/Vector3` | Box 치수 [m] (width, height, depth) |
| `num_points_used` | `uint32` | 피팅에 사용된 포인트 수 |
| `local_curvatures` | `float64[3]` | 3개 손가락의 로컬 곡률 |
| `curvature_valid` | `bool[3]` | 곡률 유효성 플래그 |

---

## 액션 타입

### ExploreShape.action

탐색 모션 + 형상 추정 통합 액션입니다. GUI 또는 BT에서 호출하며, 물체 주변을 자동
탐색하면서 ToF 기반 형상 추정을 수행하고 confidence 임계값 도달 시 결과를 반환합니다
(`shape_estimation` 패키지의 `/shape/explore` 액션 서버가 제공).

**Goal**

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `object_position` | `geometry_msgs/Point` | — | `base_link` 기준 물체 위치 |
| `use_current_object_pose` | `bool` | `false` | `true`면 `/object/pose_estimate` 토픽 사용 |
| `confidence_threshold` | `float64` | `0.0` | `0`이면 YAML 기본값 사용 |
| `max_time_sec` | `float64` | `0.0` | `0`이면 YAML 기본값 사용 |

**Result**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 탐색 성공 여부 |
| `message` | `string` | 결과 메시지 |
| `estimate` | `shape_estimation_msgs/ShapeEstimate` | 최종 형상 추정 결과 |
| `elapsed_sec` | `float64` | 경과 시간 [s] |
| `total_snapshots_processed` | `uint32` | 처리된 ToF 스냅샷 수 |
| `sweep_cycles_completed` | `uint8` | 완료된 sweep 사이클 수 |

**Feedback**

| 필드 | 타입 | 설명 |
|------|------|------|
| `current_phase` | `uint8` | `ExplorePhase` enum 값 |
| `current_estimate` | `shape_estimation_msgs/ShapeEstimate` | 현재까지의 형상 추정 결과 |
| `elapsed_sec` | `float64` | 경과 시간 [s] |
| `num_points_collected` | `uint32` | 누적 포인트 수 |
| `status_message` | `string` | 상태 메시지 |

---

## 의존성

| 패키지 | 용도 |
|--------|------|
| `std_msgs` | 표준 메시지 타입 |
| `geometry_msgs` | Pose, Point, Vector3 |
| `builtin_interfaces` | Time 타입 |
| `action_msgs` | Action 타입 (`ExploreShape`) |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select shape_estimation_msgs --symlink-install
```

---

## 라이선스

MIT License
