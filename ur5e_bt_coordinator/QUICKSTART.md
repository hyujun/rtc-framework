# Quick Start Guide

## 1. 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select ur5e_bt_coordinator
source install/setup.bash
```

## 2. 사전 실행

BT coordinator 실행 전에 RT 컨트롤러가 먼저 실행되어 있어야 한다:

```bash
# MuJoCo 시뮬레이션
ros2 launch ur5e_bringup sim.launch.py

# 또는 실제 로봇
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10
```

## 3. 기본 실행

```bash
# Pick-and-Place 태스크 (기본)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# 다른 태스크 실행
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=towel_unfold.xml \
             -p bb.sweep_direction_x:=1.0 \
             -p bb.sweep_direction_y:=0.0 \
             -p bb.sweep_distance:=0.3 \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# Hand Motions Demo
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=hand_motions.xml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml
```

## 4. 런타임 제어

```bash
# 일시 정지 / 재개
ros2 param set /bt_coordinator paused true
ros2 param set /bt_coordinator paused false

# 트리 전환 (재시작 없이)
ros2 param set /bt_coordinator tree_file "towel_unfold.xml"

# 반복 모드 활성화
ros2 param set /bt_coordinator repeat true
```

## 5. 디버깅

```bash
# Step 모드: 한 틱씩 수동 진행
ros2 param set /bt_coordinator step_mode true
ros2 service call /bt_coordinator/step std_srvs/srv/Trigger

# Groot2 시각화 (포트 1667)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p groot2_port:=1667 \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# 오프라인 트리 검증 (ROS 실행 불필요)
ros2 run ur5e_bt_coordinator validate_tree pick_and_place.xml
```

## 6. 포즈 튜닝

`config/poses.yaml`에서 재컴파일 없이 포즈를 수정할 수 있다 (값은 deg 단위, 로드 시 rad 자동 변환):

```yaml
# 예: 엄지-검지 opposition 포즈 조정
hand_pose.thumb_index_oppose: [15.0, 45.0, 35.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0]

# 예: UR5e 데모 자세 조정
arm_pose.demo_pose: [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
```

런타임에도 변경 가능:
```bash
ros2 param set /bt_coordinator hand_pose.home "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

## Troubleshooting

| 증상 | 원인 | 해결 |
|------|------|------|
| `[Watchdog] /ur5e/gui_position: no messages` | RT 컨트롤러 미실행 | `sim.launch.py` 또는 `robot.launch.py` 먼저 시작 |
| `[FAILED] IsObjectDetected` | 비전 미감지 | `/vision/object_pose` 토픽 확인 |
| `[FAILED] MoveToPose` (timeout) | 목표 도달 실패 | tolerance 완화 또는 gains 조정 |
| `[FAILED] IsForceAbove` | 힘 미감지 | threshold_N 낮추기, 물체 위치 확인 |
| `Tree file not found` | 경로 오류 | 절대 경로 사용 또는 trees/ 디렉토리 확인 |
| `E-STOP active, tree paused` | 비상 정지 활성화 | E-STOP 원인 확인 및 해제 후 자동 재개 |
