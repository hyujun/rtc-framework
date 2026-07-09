# ROS2 토픽 인터페이스

BT coordinator가 구독/발행하는 ROS2 토픽과 서비스 목록.

---

## 구독 토픽

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/rtc_cm/<arm_group>/joint_states` | `sensor_msgs/JointState` | RELIABLE/10 | 팔 관절 위치 (fixed path, active controller 와 무관) |
| `/rtc_cm/<hand_group>/joint_states` | `sensor_msgs/JointState` | RELIABLE/10 | 손 관절 위치 (10 DOF, fixed path, active controller 와 무관) |
| `<ns>/<hand_group>/grasp_state` | `rtc_msgs/GraspState` | RELIABLE/10 | 500Hz 사전 계산된 grasp 상태 (controller-owned) |
| `<ns>/<hand_group>/wbc_state` | `rtc_msgs/WbcState` | RELIABLE/10 | 500Hz WBC FSM phase + 진단 (controller-owned) |
| `/world_target_info` | `geometry_msgs/Polygon` | RELIABLE/10 | 비전 물체 위치 (points[0] = x,y,z) |
| `/rtc_cm/active_controller_name` | `std_msgs/String` | RELIABLE/1 transient_local | 현재 활성 컨트롤러 이름 (rewire 트리거) |
| `/system/estop_status` | `std_msgs/Bool` | RELIABLE/10 | E-STOP 상태 (true면 트리 일시정지) |
| `/shape/estimate` | `shape_estimation_msgs/ShapeEstimate` | RELIABLE/10 | shape estimation 결과 |
| `<ns>/tof/snapshot` | `rtc_msgs/ToFSnapshot` | BEST_EFFORT/100 | 500Hz ToF 센서 데이터 (ToF 수집 모드 시 buffer, controller-owned) |

`<ns>`는 active controller namespace (예: `/demo_task_controller`), `<arm_group>`/`<hand_group>`는 robot-agnostic 파라미터 세그먼트 (default `ur5e`/`hand`).

TCP 포즈는 토픽이 아닌 `tf2_ros` lookup으로 얻는다: `base` → `tool0_actual` (active controller가 발행하는 transforms 토픽을 자동 수집).

## 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `<ns>/<arm_group>/joint_goal` | `rtc_msgs/RobotTarget` | 팔 task-space 또는 joint-space 목표 (controller-owned) |
| `<ns>/<hand_group>/joint_goal` | `rtc_msgs/RobotTarget` | 손 10-DoF 모터 목표 (controller-owned) |
| `/shape/trigger` | `std_msgs/String` | shape estimation 제어 (start/stop/pause/resume) |

게인 변경 / 컨트롤러 전환 / Force-PI grasp는 srv & parameter API 사용 — 아래 §서비스 클라이언트 참조.

## 서비스 클라이언트

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/rtc_cm/switch_controller` | `rtc_msgs/srv/SwitchController` | 활성 컨트롤러 전환 (sync, single-active, E-STOP guard) |
| `/rtc_cm/list_controllers` | `rtc_msgs/srv/ListControllers` | 등록된 컨트롤러 목록 조회 |
| `/<active>/<active>/set_parameters_atomically` | (parameter API) | active 컨트롤러 LifecycleNode의 게인 ROS 2 parameter 설정 |
| `/<active>/grasp_command` | `rtc_msgs/srv/GraspCommand` | Force-PI one-shot transition (GRASP/RELEASE) |
| `/shape/clear` | `std_srvs/Trigger` | shape estimation 누적 데이터 초기화 |
