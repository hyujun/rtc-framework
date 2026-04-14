# ROS2 토픽 인터페이스

BT coordinator가 구독/발행하는 ROS2 토픽과 서비스 목록.

---

## 구독 토픽

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/ur5e/gui_position` | `rtc_msgs/GuiPosition` | RELIABLE/10 | 팔 TCP 포즈 + 관절 위치 |
| `/hand/gui_position` | `rtc_msgs/GuiPosition` | RELIABLE/10 | 손 관절 위치 (10 DOF) |
| `/hand/grasp_state` | `rtc_msgs/GraspState` | RELIABLE/10 | 500Hz 사전 계산된 grasp 상태 |
| `/world_target_info` | `geometry_msgs/Polygon` | RELIABLE/10 | 비전 물체 위치 (points[0] = x,y,z) |
| `/ur5e/active_controller_name` | `std_msgs/String` | RELIABLE/1 transient_local | 현재 활성 컨트롤러 이름 |
| `/system/estop_status` | `std_msgs/Bool` | RELIABLE/10 | E-STOP 상태 (true면 트리 일시정지) |
| `/ur5e/current_gains` | `std_msgs/Float64MultiArray` | RELIABLE/10 | SwitchController가 요청한 현재 게인 응답 |
| `/shape/estimate` | `shape_estimation_msgs/ShapeEstimate` | RELIABLE/10 | shape estimation 결과 |
| `/tof/snapshot` | `rtc_msgs/ToFSnapshot` | BEST_EFFORT/100 | 500Hz ToF 센서 데이터 (ToF 수집 모드 시 buffer) |

## 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ur5e/joint_goal` | `rtc_msgs/RobotTarget` | 팔 task-space 또는 joint-space 목표 |
| `/hand/joint_goal` | `rtc_msgs/RobotTarget` | 손 10-DoF 모터 목표 |
| `/ur5e/controller_gains` | `std_msgs/Float64MultiArray` | 게인 업데이트 (DemoTask 21개 / DemoJoint 9개 요소) |
| `/ur5e/controller_type` | `std_msgs/String` | 컨트롤러 전환 명령 |
| `/ur5e/request_gains` | `std_msgs/Bool` | 현재 게인 요청 (SwitchController에서 사용) |
| `/shape/trigger` | `std_msgs/String` | shape estimation 제어 (start/stop/pause/resume) |

## 서비스 클라이언트

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/shape/clear` | `std_srvs/Trigger` | shape estimation 누적 데이터 초기화 |
