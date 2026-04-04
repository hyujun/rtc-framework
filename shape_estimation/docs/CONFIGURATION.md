# Shape Estimation Configuration Guide

shape_estimation_node의 모든 파라미터, 튜닝 가이드, 시나리오별 프리셋.

설정 파일: `config/shape_estimation_node.yaml`

---

## Table of Contents

1. [Parameter Reference](#parameter-reference)
2. [Tuning Guide](#tuning-guide)
3. [Scenario Presets](#scenario-presets)
4. [Message Types](#message-types)

---

## Parameter Reference

### Shape Estimation (Core)

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `voxel_resolution` | double | 0.002 | 0.001 ~ 0.01 | Voxel 해상도 [m]. 작을수록 세밀하지만 메모리/포인트 수 증가 |
| `max_points` | int | 2048 | 100 ~ 10000 | 최대 포인트 수. 초과 시 oldest 제거 |
| `point_expiry_sec` | double | 5.0 | 1.0 ~ 60.0 | 포인트 만료 시간 [s]. 짧으면 최근 데이터 위주, 길면 누적 |
| `flat_curvature_threshold` | double | 5.0 | 1.0 ~ 20.0 | 평면 판정 곡률 임계값 [1/m]. 5.0 = 반지름 >= 200mm |
| `curvature_uniformity_threshold` | double | 2.0 | 0.5 ~ 10.0 | 곡률 균일성 임계값 [1/m]. 작으면 엄격한 구 판정 |
| `min_points_for_fitting` | int | 10 | 3 ~ 100 | 최소제곱 피팅 시작 최소 포인트 수 |
| `publish_rate_hz` | double | 10.0 | 1.0 ~ 100.0 | 추정 결과 발행 주기 [Hz] |
| `viz_rate_hz` | double | 5.0 | 1.0 ~ 30.0 | 시각화 마커 발행 주기 [Hz] |
| `frame_id` | string | `base_link` | - | TF 기준 프레임 |

### Protuberance Detection

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `protuberance.residual_threshold` | double | -0.005 | -0.02 ~ -0.001 | 음의 잔차 임계값 [m]. 절대값이 클수록 깊은 돌출만 탐지 |
| `protuberance.min_cluster_points` | int | 3 | 1 ~ 20 | 최소 클러스터 포인트 수. 크면 노이즈 필터링 강화 |
| `protuberance.cluster_radius` | double | 0.015 | 0.005 ~ 0.05 | 클러스터 연결 반경 [m]. 클수록 넓은 돌출 하나로 묶임 |
| `protuberance.gap_distance_jump` | double | 0.020 | 0.005 ~ 0.05 | Gap 전후 거리 차이 임계값 [m] |
| `protuberance.min_gap_invalid_count` | int | 2 | 1 ~ 10 | 연속 invalid 최소 수 |
| `protuberance.curvature_jump_threshold` | double | 15.0 | 5.0 ~ 50.0 | 곡률 급변 임계값 [1/m] |
| `protuberance.gap_cluster_association_radius` | double | 0.020 | 0.005 ~ 0.05 | Gap-클러스터 연결 반경 [m] |

**Confidence 가중치** (코드 내 하드코딩, `ProtuberanceConfig`):

| Weight | Default | Factor |
|--------|---------|--------|
| `weight_num_points` | 0.3 | `min(N/10, 1.0)` |
| `weight_depth` | 0.4 | `min(depth/0.02, 1.0)` |
| `weight_gap` | 0.3 | `has_gap ? 1.0 : 0.0` |

### Exploration Motion

`enable_exploration: true` 일 때만 사용. `false`이면 Action Server 미생성.

#### RT Controller Integration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exploration.robot_namespace` | string | `ur5e` | 토픽 접두사 (`/{ns}/controller_type` 등) |
| `exploration.controller_name` | string | `demo_task_controller` | 탐색 시 활성화할 컨트롤러 |
| `exploration.controller_switch_delay_ms` | int | 200 | 컨트롤러 전환 후 대기 시간 [ms] |

#### Exploration Gains (DemoTaskController, 16 values)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exploration.exploration_gains.kp_translation` | double[3] | [10, 10, 10] | 위치 비례 게인 |
| `exploration.exploration_gains.kp_rotation` | double[3] | [5, 5, 5] | 회전 비례 게인 |
| `exploration.exploration_gains.damping` | double | 0.01 | Damped pseudoinverse 감쇠 |
| `exploration.exploration_gains.null_kp` | double | 0.0 | Null-space 게인 |
| `exploration.exploration_gains.enable_null_space` | bool | false | Null-space 활성화 |
| `exploration.exploration_gains.control_6dof` | bool | true | 6-DOF 제어 (위치+자세) |
| `exploration.exploration_gains.trajectory_speed` | double | 0.05 | 궤적 속도 [m/s] |
| `exploration.exploration_gains.trajectory_angular_speed` | double | 0.3 | 궤적 각속도 [rad/s] |
| `exploration.exploration_gains.hand_trajectory_speed` | double | 0.0 | 핸드 궤적 속도 |
| `exploration.exploration_gains.max_traj_velocity` | double | 0.10 | 최대 궤적 속도 [m/s] |
| `exploration.exploration_gains.max_traj_angular_velocity` | double | 0.5 | 최대 궤적 각속도 [rad/s] |
| `exploration.exploration_gains.hand_max_traj_velocity` | double | 0.0 | 핸드 최대 속도 |

#### Phase Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exploration.approach_step_size` | double | 0.005 | Approach step [m] |
| `exploration.approach_timeout_sec` | double | 5.0 | Approach timeout [s] |
| `exploration.servo_target_distance` | double | 0.030 | 목표 ToF 거리 [m] |
| `exploration.servo_step_gain` | double | 0.5 | 서보 비례 게인 |
| `exploration.servo_max_step` | double | 0.005 | 서보 최대 step [m] |
| `exploration.servo_converge_tol` | double | 0.003 | 수렴 판정 허용 오차 [m] |
| `exploration.servo_timeout_sec` | double | 3.0 | 서보 timeout [s] |
| `exploration.servo_min_valid_sensors` | int | 3 | 서보 최소 유효 센서 수 |
| `exploration.sweep_step_size` | double | 0.003 | Sweep step [m] |
| `exploration.sweep_width` | double | 0.06 | Sweep 폭 [m] (+-30mm) |
| `exploration.sweep_normal_gain` | double | 0.5 | 표면 거리 유지 게인 |
| `exploration.sweep_normal_max_step` | double | 0.003 | 표면 보정 최대 step [m] |
| `exploration.tilt_amplitude_deg` | double | 15.0 | Tilt 진폭 [deg] |
| `exploration.tilt_steps` | int | 10 | Tilt 총 step 수 |

#### Safety & Evaluation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exploration.min_distance` | double | 0.005 | ToF 최소 안전 거리 [m] |
| `exploration.max_step_size` | double | 0.010 | 1회 최대 이동량 [m] |
| `exploration.confidence_threshold` | double | 0.8 | 성공 판정 confidence |
| `exploration.min_points_for_success` | int | 20 | 성공 판정 최소 포인트 수 |
| `exploration.max_total_time_sec` | double | 10.0 | 최대 탐색 시간 [s] |
| `exploration.max_sweep_cycles` | int | 3 | 최대 sweep 재시도 횟수 |
| `exploration.explore_rate_hz` | double | 10.0 | 탐색 루프 주기 [Hz] |

---

## Tuning Guide

### Voxel Resolution

| 목표 | 설정 |
|------|------|
| **정밀 형상 (작은 물체)** | `voxel_resolution: 0.001` + `max_points: 4096` |
| **빠른 추정 (큰 물체)** | `voxel_resolution: 0.005` + `max_points: 1024` |
| **기본 (범용)** | `voxel_resolution: 0.002` + `max_points: 2048` |

Trade-off: 해상도가 높으면 세밀하지만 포인트가 빨리 차고, 피팅 계산량이 증가한다.

### Point Expiry

| 상황 | 설정 |
|------|------|
| **정적 물체 (고정)** | `point_expiry_sec: 30.0` — 오랜 누적으로 정확한 피팅 |
| **이동 중 추정** | `point_expiry_sec: 2.0` — 과거 데이터 빠르게 소거 |
| **탐색 모션 중** | `point_expiry_sec: 5.0` — sweep 데이터 유지 |

### Classifier Thresholds

| 조정 방향 | 설정 |
|-----------|------|
| **평면을 더 엄격하게** | `flat_curvature_threshold: 2.0` (반지름 >= 500mm만 평면) |
| **평면을 더 관대하게** | `flat_curvature_threshold: 10.0` (반지름 >= 100mm도 평면) |
| **구를 더 엄격하게** | `curvature_uniformity_threshold: 1.0` (곡률 편차 1.0 이내) |

### Fitting Parameters

| 상황 | 설정 |
|------|------|
| **노이즈가 많은 센서** | `min_points_for_fitting: 20` — 포인트 수 확보 후 피팅 |
| **빠른 초기 추정** | `min_points_for_fitting: 6` — 빠르지만 부정확할 수 있음 |

### Protuberance Sensitivity

| 목표 | residual_threshold | min_cluster_points | cluster_radius |
|------|-------------------|-------------------|----------------|
| **민감하게** | -0.002 | 2 | 0.020 |
| **기본** | -0.005 | 3 | 0.015 |
| **보수적** | -0.010 | 5 | 0.010 |

민감할수록 false positive 증가, 보수적일수록 작은 돌출을 놓칠 수 있다.

### Exploration Speed/Safety

| 목표 | Key Parameters |
|------|----------------|
| **안전 우선 (느린 탐색)** | `approach_step: 0.003`, `max_step: 0.005`, `traj_speed: 0.03` |
| **빠른 탐색** | `approach_step: 0.008`, `max_step: 0.015`, `traj_speed: 0.08` |
| **좁은 공간** | `sweep_width: 0.03`, `min_distance: 0.008` |

---

## Scenario Presets

### 1. Small Sphere (반지름 < 30mm)

```yaml
voxel_resolution: 0.001
max_points: 4096
point_expiry_sec: 10.0
min_points_for_fitting: 15
flat_curvature_threshold: 3.0
exploration:
  servo_target_distance: 0.020
  sweep_width: 0.04
  sweep_step_size: 0.002
  confidence_threshold: 0.7
```

### 2. Large Flat Object (평면 추정)

```yaml
voxel_resolution: 0.005
max_points: 1024
point_expiry_sec: 10.0
flat_curvature_threshold: 8.0
min_points_for_fitting: 8
exploration:
  servo_target_distance: 0.040
  sweep_width: 0.10
  sweep_step_size: 0.005
```

### 3. Cylinder (파이프, 병 등)

```yaml
voxel_resolution: 0.002
max_points: 2048
curvature_uniformity_threshold: 3.0
min_points_for_fitting: 12
exploration:
  sweep_width: 0.06
  tilt_amplitude_deg: 20.0
  tilt_steps: 12
```

### 4. Object with Protuberance (손잡이 등)

```yaml
voxel_resolution: 0.001
max_points: 4096
point_expiry_sec: 8.0
protuberance:
  residual_threshold: -0.003
  min_cluster_points: 2
  cluster_radius: 0.020
  gap_distance_jump: 0.015
exploration:
  sweep_width: 0.08
  max_sweep_cycles: 4
  confidence_threshold: 0.75
```

### 5. Estimation Only (탐색 비활성)

```yaml
enable_exploration: false
voxel_resolution: 0.002
max_points: 2048
publish_rate_hz: 20.0
viz_rate_hz: 10.0
```

---

## Message Types

### shape_estimation_msgs

#### ToFReadings.msg

RT 컨트롤러에서 500Hz로 publish하는 ToF 센서 원시 데이터.

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | Timestamp |
| `distances` | `float64[6]` | [thumb_A, thumb_B, index_A, index_B, middle_A, middle_B] [m] |
| `valid` | `bool[6]` | 센서별 유효성 |

#### TipPoses.msg

3개 핑거팁의 월드 프레임 SE3 자세 (FK 결과).

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | Timestamp |
| `poses` | `geometry_msgs/Pose[3]` | thumb, index, middle |

#### ToFSnapshot.msg

ToF 측정값 + 핑거팁 자세 통합 메시지. 동일 제어 사이클의 데이터.

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | Timestamp |
| `distances` | `float64[6]` | ToF 센서 거리 [m] |
| `valid` | `bool[6]` | 유효성 |
| `tip_poses` | `geometry_msgs/Pose[3]` | 핑거팁 SE3 자세 |

#### ShapeEstimate.msg

추정된 형상 프리미티브 + 돌출 구조 정보.

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | Time | Timestamp |
| `shape_type` | uint8 | 0=UNKNOWN, 1=PLANE, 2=SPHERE, 3=CYLINDER, 4=BOX |
| `confidence` | float64 | [0, 1] |
| `center` | Point | Primitive 중심 (world frame) |
| `axis` | Vector3 | Cylinder 축 / Plane 법선 |
| `radius` | float64 | Sphere/Cylinder 반지름 [m] |
| `dimensions` | Vector3 | Box (w, h, d) [m] |
| `num_points_used` | uint32 | 피팅 포인트 수 |
| `local_curvatures` | float64[3] | 3개 손가락 로컬 곡률 |
| `curvature_valid` | bool[3] | 곡률 유효성 |
| `has_protuberance` | bool | 돌출 탐지 여부 |
| `protuberance_centroid` | Point | 돌출부 중심 |
| `protuberance_direction` | Vector3 | 돌출 방향 |
| `protuberance_depth` | float64 | 돌출 깊이 [m] |
| `protuberance_extent` | float64 | 표면 방향 크기 [m] |
| `protuberance_confidence` | float64 | 돌출 신뢰도 [0, 1] |
| `protuberance_has_gap` | bool | Gap 동반 여부 |
| `protuberance_num_points` | uint32 | 돌출 클러스터 포인트 수 |

#### ExploreShape.action

| Section | Fields |
|---------|--------|
| **Goal** | `object_position` (Point), `use_current_object_pose` (bool), `confidence_threshold` (float64, 0=YAML default), `max_time_sec` (float64, 0=YAML default) |
| **Result** | `success`, `message`, `estimate` (ShapeEstimate), `elapsed_sec`, `total_snapshots_processed`, `sweep_cycles_completed` |
| **Feedback** | `current_phase` (uint8), `current_estimate`, `elapsed_sec`, `num_points_collected`, `status_message` |
