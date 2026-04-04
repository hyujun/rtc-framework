# Shape Estimation Algorithms

shape_estimation 패키지의 핵심 알고리즘에 대한 상세 설명.

---

## Table of Contents

1. [Sensor Geometry](#sensor-geometry)
2. [Curvature Computation](#curvature-computation)
3. [Voxel Point Cloud](#voxel-point-cloud)
4. [Fast Shape Classifier](#fast-shape-classifier)
5. [Primitive Fitter](#primitive-fitter)
6. [Protuberance Detector](#protuberance-detector)
7. [Exploration Motion Generator](#exploration-motion-generator)
8. [Data Flow](#data-flow)

---

## Sensor Geometry

3개 핑거팁(thumb, index, middle)에 각 2개 ToF 센서(A/B)가 장착되어 총 6개 센서를 사용한다.

```
         Fingertip (top view)
        ┌───────────────────┐
        │    tip_link        │
        │                   │
        │  [A]    [B]       │  ← ToF sensors
        │  +2mm   -2mm      │     beam direction: +z (local)
        │                   │
        └───────────────────┘
             ↓  ↓  (beams)
        ─────────────────── surface
```

### Constants (`tof_shape_types.hpp`)

| Constant | Value | Description |
|----------|-------|-------------|
| `kNumFingers` | 3 | thumb, index, middle |
| `kSensorsPerFinger` | 2 | A (+2mm), B (-2mm) |
| `kTotalSensors` | 6 | 전체 센서 수 |
| `kTofOffsetX` | 0.002 m | 센서 lateral offset |
| `kTofSeparation` | 0.004 m | A-B 간격 |
| `kTofMinRange` | 0.01 m | 최소 유효 거리 |
| `kTofMaxRange` | 0.30 m | 최대 유효 거리 |

### Sensor Index Mapping

```
sensor_idx = finger_id * 2 + side
  0: thumb_A    1: thumb_B
  2: index_A    3: index_B
  4: middle_A   5: middle_B
```

### Surface Point Computation

FK 결과(tip_link pose)에서 센서 위치와 표면 접촉점을 계산한다:

```
sensor_position = tip_pose.position + tip_pose.rotation * [+-kTofOffsetX, 0, 0]
beam_direction  = tip_pose.rotation * [0, 0, 1]   // tip_link +z axis
surface_point   = sensor_position + distance * beam_direction
surface_normal  = -beam_direction
```

---

## Curvature Computation

각 핑거팁의 A/B 센서 거리 차이로 로컬 곡률을 추정한다 (`msg_conversions.cpp`).

### Formula

```
       2 * delta_d
kappa = ─────────────────
       delta_x^2 + delta_d^2

where:
  delta_d = d_A - d_B      (A/B 센서 거리 차이)
  delta_x = 4mm             (A-B 센서 간격 = kTofSeparation)
```

### Interpretation

| Curvature | Meaning | Corresponding Radius |
|-----------|---------|---------------------|
| kappa ~= 0 | 평면 | infinity |
| kappa > 0, uniform | 볼록 곡면 (구) | R = 1/kappa |
| kappa > 0, non-uniform | 비균일 볼록 (실린더 등) | varies |
| kappa < 0 | 오목 곡면 | R = 1/|kappa| |

### Validity

양쪽 센서(A, B) 모두 유효(`valid == true`)하고 range 내(`kTofMinRange` ~ `kTofMaxRange`)일 때만 `curvature_valid = true`.

---

## Voxel Point Cloud

`VoxelPointCloud` 클래스는 공간 해싱 기반으로 포인트를 누적 관리한다.

### Voxel Hashing

3D 좌표를 정수 voxel key로 변환:

```cpp
ix = floor(x / voxel_resolution)
iy = floor(y / voxel_resolution)
iz = floor(z / voxel_resolution)

mask = (1 << 21) - 1     // 21-bit per axis
key = ((ix & mask) << 42) | ((iy & mask) << 21) | (iz & mask)
```

21-bit per axis -> 약 +-2097mm 범위를 커버하며, 2mm 해상도에서 약 +-1048 voxel.

### Moving Average Update

동일 voxel에 새 포인트가 들어오면 이동 평균으로 업데이트:

```
n = update_count
position = position * (n / (n+1)) + new_position * (1 / (n+1))
normal   = normal   * (n / (n+1)) + new_normal   * (1 / (n+1))
curvature = curvature * (n / (n+1)) + new_curvature * (1 / (n+1))
timestamp = max(timestamp, new_timestamp)
update_count = n + 1
```

### Expiry & Capacity Management

1. **시간 만료**: `RemoveExpired()` 호출 시 `timestamp < current_time - expiry_ns` 인 voxel 제거
2. **용량 초과**: `max_points` 초과 시 가장 오래된 timestamp의 voxel부터 제거
3. `AddSnapshot()` 내에서 valid reading만 추가 (range 체크 포함)

### Complexity

| Operation | Time |
|-----------|------|
| AddSnapshot (6 points) | O(6) amortized (hash insert/update) |
| RemoveExpired | O(N) |
| GetPoints | O(N) |

---

## Fast Shape Classifier

`FastShapeClassifier`는 곡률 값만으로 실시간 분류를 수행하는 rule-based 분류기이다. 매 스냅샷(500Hz)마다 호출 가능한 저비용 연산.

### Classification Logic

```
Input: local_curvatures[3], curvature_valid[3]

1. Collect valid curvatures
   - if no valid curvatures → UNKNOWN

2. max_curvature = max(|kappa_i|) for all valid i

3. if max_curvature < flat_curvature_threshold (5.0 m^-1):
   → PLANE
   confidence = 1.0 - max_curvature / flat_curvature_threshold

4. else if avg_curvature > 0 AND std_dev < curvature_uniformity_threshold (2.0 m^-1):
   → SPHERE
   confidence = 1.0 - std_dev / curvature_uniformity_threshold

5. else if |index_curvature - middle_curvature| < uniformity_threshold
        AND thumb differs significantly:
   → CYLINDER
   confidence = 0.6 (fixed)

6. else:
   → UNKNOWN
   confidence = 0.0
```

### Threshold Rationale

| Parameter | Default | Physical Meaning |
|-----------|---------|------------------|
| `flat_curvature_threshold` | 5.0 m^-1 | 곡률 반지름 >= 200mm -> 평면으로 간주 |
| `curvature_uniformity_threshold` | 2.0 m^-1 | 곡률 편차 2.0 m^-1 이내 -> 균일 |

---

## Primitive Fitter

`PrimitiveFitter`는 누적된 포인트 클라우드에 대해 4가지 프리미티브를 최소제곱법으로 피팅한다.

### Sphere Fitting (Algebraic SVD)

**최소 포인트: 4**

선형 시스템으로 변환하여 SVD로 풀이:

```
Minimize: sum |p_i - c|^2 - r^2

Linear system: A * x = b
  A[i] = [2*x_i, 2*y_i, 2*z_i, 1]
  b[i] = x_i^2 + y_i^2 + z_i^2
  x = [cx, cy, cz, cx^2+cy^2+cz^2 - r^2]

Solve via SVD (least-squares): x = A.jacobiSvd(ComputeThinU|ComputeThinV).solve(b)

center = [x[0], x[1], x[2]]
radius = sqrt(x[0]^2 + x[1]^2 + x[2]^2 - x[3])
```

**Confidence**: `1.0 - (RMS_residual / radius)`, 여기서 `RMS_residual = sqrt(mean(|p_i - c| - r)^2)`

### Cylinder Fitting (PCA + 2D Circle)

**최소 포인트: 5**

2단계 절차:

```
Step 1: PCA로 축 방향 추정
  - 포인트 중심화 (mean 제거)
  - 3x3 공분산 행렬의 고유값 분해
  - 최대 고유값의 고유벡터 = 실린더 축 방향

Step 2: 2D 원 피팅 (축 직교 평면)
  - 모든 포인트를 축 직교 평면에 투영
  - 2D algebraic circle fit (SVD)
  - 결과: 2D center (u0, v0) + radius
  - 3D center = mean + u0*e1 + v0*e2 (e1, e2: 직교 기저)
```

**Confidence**: `1.0 - (RMS_residual / radius)`

### Plane Fitting (SVD/PCA)

**최소 포인트: 3**

```
Step 1: 포인트 중심화
  center = mean(p_i)

Step 2: 3x3 공분산 행렬 고유값 분해
  C = (1/N) * sum((p_i - center) * (p_i - center)^T)

Step 3: 최소 고유값의 고유벡터 = 평면 법선
  normal = eigenvector of min(eigenvalue)

Step 4: 두께 계산
  thickness = max(|(p_i - center) . normal|)
```

**Confidence 계산**:
- `thickness_conf = 1.0 - min(thickness / 0.01, 1.0)` (10mm 이내면 높은 점수)
- `normal_consistency = mean(|p_i.normal . plane_normal|)` (포인트 법선과 평면 법선 일치도)
- `confidence = 0.5 * thickness_conf + 0.5 * normal_consistency`

### Box Fitting (PCA-based OBB)

**최소 포인트: 6**

```
Step 1: PCA로 3축 추출
  C = covariance matrix
  eigenvectors = 3 principal axes (descending eigenvalue order)

Step 2: 포인트를 PCA 좌표계로 투영
  projected_i = eigenvectors^T * (p_i - center)

Step 3: 각 축의 min/max extent 계산
  dimensions = [max - min] for each axis

Step 4: OBB 중심 계산
  obb_center = center + eigenvectors * [(min + max) / 2 for each axis]
```

**Confidence**: `1.0 - (RMS_residual / avg_dimension)`, 여기서 `avg_dimension = mean(w, h, d)`

### Best Primitive Selection

`FitBestPrimitive()`는 4가지를 모두 피팅한 후 최적 결과를 선택:

```
1. 충분한 포인트가 있는 모든 프리미티브 피팅
2. confidence가 min_confidence(0.3) 이상인 후보 필터링
3. max(confidence)를 선택, 동점이면 min(num_params) 선택
4. 모든 후보가 min_confidence 미만이면 UNKNOWN 반환
```

---

## Protuberance Detector

피팅된 프리미티브 표면의 돌출 구조를 탐지하는 4단계 파이프라인.

### Step 1: Signed Residual Computation

각 포인트에서 primitive 표면까지의 부호 거리를 계산. **음수 = 표면 바깥 (돌출)**.

| Primitive | Signed Distance |
|-----------|----------------|
| **Sphere** | `radius - |point - center|` |
| **Cylinder** | `radius - perpendicular_distance_to_axis` |
| **Plane** | `-(point - center) . normal` |
| **Box** | `min_margin_to_any_face` (가장 가까운 면까지 거리) |

### Step 2: Negative Residual Clustering

Union-Find 알고리즘으로 돌출 포인트를 공간 클러스터로 그룹화.

```
1. Filter: signed_residual < residual_threshold (-5mm)
2. For each pair of filtered points:
   if |p_i - p_j| < cluster_radius (15mm):
     Union(i, j)
3. Extract connected components
4. Filter: cluster_size >= min_cluster_points (3)
```

### Step 3: Gap Detection

ToF 시계열 데이터에서 센서 invalid 구간을 분석하여 gap event를 추출.

```
For each sensor in time-series:
  Track consecutive invalid readings
  If invalid_count >= min_gap_invalid_count (2):
    If distance_before and distance_after differ by > gap_distance_jump (20mm):
      → GapEvent {
          timestamp, sensor_index,
          position_before, position_after,
          distance_before, distance_after,
          gap_duration
        }
```

Gap은 센서가 물체 edge를 지날 때 발생 — 돌출 구조의 경계를 나타내는 간접 증거.

### Step 4: Protuberance Construction

클러스터와 gap 정보를 통합하여 돌출 구조를 생성.

```
For each cluster:
  centroid    = mean(cluster_points)
  direction   = PrimitiveOutwardNormal(centroid)  // 표면 바깥 방향
  extent      = max pairwise distance in cluster
  depth       = max |signed_residual| in cluster

  has_gap = any gap within gap_cluster_association_radius (20mm)?

  confidence = w_points * min(N/10, 1.0)       // 0.3
             + w_depth  * min(depth/0.02, 1.0)  // 0.4
             + w_gap    * (has_gap ? 1.0 : 0.0) // 0.3
```

### Output

```cpp
struct ProtuberanceResult {
  bool detected;                    // 탐지 여부
  vector<Protuberance> protuberances;  // 다중 돌출 가능
  double base_residual_rms;         // 돌출부 제외 시 잔차 RMS
};
```

---

## Exploration Motion Generator

ROS 비의존 순수 C++ FSM. Action Server(`/shape/explore`)를 통해 호출되며, DemoTaskController와 연동하여 SE3 waypoint를 생성한다.

### State Machine

```
       ┌──────────┐
       │   Idle   │
       └─────┬────┘
     Start() │
       ┌─────▼────┐  ToF detects or timeout(5s)
       │ Approach  ├──────────────────────────────────┐
       └──────────┘                                   │
       ┌─────▼────┐  Distance converges or timeout(3s)│
       │  Servo    ├──────────────────────────────────┐│
       └──────────┘                                   ││
       ┌─────▼────┐  Sweep complete (+-30mm)          ││
       │ Sweep X   ├─────────────────────────────────┐││
       └──────────┘                                  │││
       ┌─────▼────┐  Sweep complete                  ││││
       │ Sweep Y   ├────────────────────────────────┐││││
       └──────────┘                                 │││││
       ┌─────▼────┐  Steps complete                 ││││││
       │   Tilt    ├───────────────────────────────┐│││││
       └──────────┘                                ││││││
       ┌─────▼────┐                                │││││││
       │ Evaluate  │                               │││││││
       └──┬────┬──┘                                │││││││
     ok   │    │  retry (< max_cycles)             │││││││
  ┌───────▼┐  └──→ back to SweepX                 │││││││
  │Succeeded│                                      │││││││
  └────────┘  ┌────────┐                           │││││││
              │ Failed  │  max_cycles exceeded     │││││││
              └────────┘                           │││││││
```

### Phase Details

#### Approach

물체 방향으로 step-by-step 접근.

```
approach_direction = (object_position - ee_position).normalized()
goal.pose = current_pose + approach_step_size * approach_direction
```

**Transition**: ToF가 물체를 감지(mean distance in range) 또는 `approach_timeout_sec` 경과.

#### Servo

ToF 거리를 목표 거리(`servo_target_distance`)로 수렴시키는 비례 서보.

```
error = mean_valid_distance - servo_target_distance
step = clamp(servo_step_gain * error, -servo_max_step, +servo_max_step)
goal.pose = current_pose + step * approach_direction
```

**Transition**: `|error| < servo_converge_tol` 또는 `servo_timeout_sec` 경과.
**Minimum sensors**: `servo_min_valid_sensors` (기본 3) 미만이면 step 생략.

#### Sweep X/Y

표면을 따라 lateral sweep하며 포인트를 수집.

```
sweep_axis = approach_direction x Z  (or computed perpendicular for Y)
sweep_position += direction * sweep_step_size

// 표면 거리 유지 (normal correction)
normal_error = mean_distance - servo_target_distance
normal_step = clamp(sweep_normal_gain * normal_error, +-sweep_normal_max_step)

goal = current + sweep_axis * step + approach_direction * normal_step
```

Edge 도달 시 방향 반전 (`sweep_position` 범위: `[-sweep_width/2, +sweep_width/2]`).

#### Tilt

손목 roll/pitch를 사인파로 틸팅하여 다양한 각도에서 측정.

```
angle = tilt_amplitude * sin(2*pi * step_count / tilt_steps)
if step_count % 2 == 0:
  goal.roll += angle
else:
  goal.pitch += angle
```

#### Evaluate

현재 형상 추정 결과를 평가.

```
if confidence >= confidence_threshold AND num_points >= min_points_for_success:
  → Succeeded
else if sweep_cycles < max_sweep_cycles:
  → back to SweepX (retry)
else:
  → Failed
```

### Safety Validation

모든 생성된 goal은 `ValidateGoal()`로 검증:

```
1. step_distance = |goal_pos - current_pos|
   if step_distance > max_step_size (10mm): → reject

2. predicted_distance = min valid ToF distance - step toward object
   if predicted_distance < min_distance (5mm): → reject
```

---

## Data Flow

### Shape Estimation (Passive Mode)

```
[RT Controller @500Hz]
  └─> ToFSnapshot msg
            ↓
[SnapshotCallback]
  1. Convert ROS msg → internal ToFSnapshot
     - FK + offset → sensor_positions_world
     - sensor + d*beam → surface_points_world
     - A/B diff → local_curvatures

  2. voxel_cloud_.AddSnapshot(snapshot)
     - Hash each valid surface point
     - Existing voxel → moving average update
     - New voxel → insert (evict oldest if full)

  3. snapshot_history_.Push(snapshot)

  4. ShapeEstimate = EstimateShape()
     - fast_classifier_.Classify(curvatures) → fast_result
     - if voxel_cloud_.Size() >= min_points_for_fitting:
         fitter_.FitBestPrimitive(points) → fitted_result
     - return max(fast_result.confidence, fitted_result.confidence)

  5. protuberance_detector_.Detect(estimate, points, history)

  6. Publish /shape/estimate (rate-limited ~10Hz)

[VizTimerCallback @5Hz async]
  - Publish point_cloud, primitive_marker, tof_beams, curvature_text, protuberance_marker
```

### Autonomous Exploration (Active Mode)

```
[Client] → /shape/explore action goal
             ↓
[HandleAccepted]
  1. Publish /{ns}/controller_type → "demo_task_controller"
  2. Publish /{ns}/controller_gains → exploration gains (16 values)
  3. Wait controller_switch_delay_ms (200ms)
  4. motion_generator_.Start(current_pose, object_position)
  5. Start /shape/trigger "start"
  6. state_ = kRunning

[ExploreLoopCallback @10Hz]
  while action_active && !estop:
    step = motion_generator_.Step(snapshot, estimate, pose, dt)

    if step.goal.valid:
      if ValidateGoal(goal, current, snapshot):
        Publish /{ns}/joint_goal → RobotTarget (task_target)
        stats.goals_sent++
      else:
        Log warning, skip this step

    Publish action feedback (phase, estimate, elapsed, num_points)
    Publish /shape/explore_status markers

    if phase == kSucceeded:
      SendActionResult(success=true)
    if phase == kFailed:
      SendActionResult(success=false)
```

---

## Complexity Summary

| Component | Per-call Complexity | Invocation Rate |
|-----------|-------------------|-----------------|
| Voxel AddSnapshot | O(6) amortized | 500Hz |
| Fast Classifier | O(1) | 500Hz |
| Primitive Fitting (all 4) | O(N) per primitive | ~10Hz (rate-limited) |
| Protuberance Detection | O(N^2) clustering | ~10Hz |
| Exploration FSM Step | O(1) | 10Hz |
| Visualization | O(N) point cloud | 5Hz |

N = number of accumulated voxel points (max 2048).
