# shape_estimation_msgs

**버전 5.17.0** | `ament_cmake` | ROS 2 메시지 패키지

ToF 기반 형상 추정 시스템용 커스텀 ROS 2 메시지 정의 패키지입니다. 핑거팁 ToF 센서 스냅샷, 자세 정보, 형상 프리미티브 추정 결과를 위한 4종의 메시지 타입을 제공합니다.

---

## 메시지 타입

### ToFReadings.msg

RT 컨트롤러에서 500Hz로 publish하는 ToF 센서 원시 데이터입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `distances` | `float64[6]` | 6개 센서 거리 [m] (thumb_A, thumb_B, index_A, index_B, middle_A, middle_B) |
| `valid` | `bool[6]` | 6개 센서 유효성 플래그 |

### TipPoses.msg

3개 핑거팁의 월드 프레임 SE3 자세 (FK 결과)입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `poses` | `geometry_msgs/Pose[3]` | thumb, index, middle 순서 |

### ToFSnapshot.msg

ToF 측정값 + 핑거팁 자세를 하나의 메시지로 통합합니다. RT 컨트롤러에서 동일 제어 사이클의 데이터를 묶어 publish합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `distances` | `float64[6]` | ToF 센서 거리 [m] |
| `valid` | `bool[6]` | ToF 센서 유효성 |
| `tip_poses` | `geometry_msgs/Pose[3]` | 핑거팁 SE3 자세 (thumb, index, middle) |

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

## 의존성

| 패키지 | 용도 |
|--------|------|
| `std_msgs` | 표준 메시지 타입 |
| `geometry_msgs` | Pose, Point, Vector3 |
| `builtin_interfaces` | Time 타입 |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select shape_estimation_msgs --symlink-install
```

---

## 라이선스

MIT License
