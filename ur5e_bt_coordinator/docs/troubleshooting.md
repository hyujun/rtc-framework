# 트러블슈팅

---

## 증상별 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| `[Watchdog] /ur5e/gui_position: no messages` | RT 컨트롤러 미실행 | `sim.launch.py` 또는 `robot.launch.py` 먼저 시작 |
| 트리가 즉시 FAILURE | 비전 토픽 미발행 | `/world_target_info` 발행 확인 |
| `[FAILED] IsObjectDetected` | 비전 미감지 | `/world_target_info` 토픽 확인 |
| `[FAILED] MoveToPose` (timeout) | 목표 도달 실패 | tolerance 완화 또는 gains 조정 |
| `[FAILED] IsForceAbove` | 힘 미감지 | threshold_N 낮추기, 물체 위치 확인 |
| 팔이 움직이지 않음 | RT Controller 미실행 또는 E-STOP 활성 | 컨트롤러 상태 및 E-STOP 확인 |
| 파지 타임아웃 | 힘 임계값이 너무 높거나 센서 미연결 | `threshold_N` 조정, `/hand/grasp_state` 확인 |
| 게인 변경이 반영 안 됨 | `/ur5e/controller_gains` 토픽 QoS 불일치 | RELIABLE QoS 확인 |
| "Tree completed with FAILURE" 로그 | 시퀀스 중 하나의 노드가 실패 | Groot2로 트리 실행 추적하여 실패 노드 확인 |
| `Tree file not found` | 경로 오류 | 절대 경로 사용 또는 trees/ 디렉토리 확인 |
| `E-STOP active, tree paused` | 비상 정지 활성화 | E-STOP 원인 확인 및 해제 후 자동 재개 |

---

## 디버깅 명령

```bash
# 토픽 모니터링
ros2 topic echo /ur5e/gui_position
ros2 topic echo /hand/grasp_state
ros2 topic echo /world_target_info

# 파라미터 확인
ros2 param list /bt_coordinator
ros2 param get /bt_coordinator tree_file

# 활성 컨트롤러 확인
ros2 topic echo /ur5e/active_controller_name

# Step 모드로 한 틱씩 디버깅
ros2 param set /bt_coordinator step_mode true
ros2 service call /bt_coordinator/step std_srvs/srv/Trigger

# Groot2 시각화 (포트 1667)
# 노드 시작 시 -p groot2_port:=1667 옵션 사용, Groot2 GUI에서 localhost:1667 연결

# 오프라인 트리 검증 (ROS 실행 불필요)
ros2 run ur5e_bt_coordinator validate_tree pick_and_place.xml
ros2 run ur5e_bt_coordinator validate_tree pick_and_place_contact_stop.xml
ros2 run ur5e_bt_coordinator validate_tree pick_and_place_force_pi.xml
ros2 run ur5e_bt_coordinator validate_tree shape_inspect.xml
ros2 run ur5e_bt_coordinator validate_tree shape_inspect_simple.xml
ros2 run ur5e_bt_coordinator validate_tree towel_unfold.xml
```
