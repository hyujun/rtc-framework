# 공통 파라미터와 손 배선 (`demo_shared.yaml`)

`DemoJointController` · `DemoTaskController` · `DemoComplianceController` · `DemoWbcController` 가 공유하는 파라미터와, 이 네 컨트롤러가 공통으로 따르는 팔 끝·손끝 배선 규칙을 모은 문서다. 값의 SSoT 는 로봇별 `config/<robot>/controllers/demo_shared.yaml` 이고, 네 컨트롤러가 `LoadConfig()` 때 같은 로더로 읽는다 (`include/integrated_bringup/support/demo_shared_config.hpp`).

## 파라미터

| 파라미터 | 설명 |
|---|---|
| `virtual_tcp_mode` / `virtual_tcp_offset` / `virtual_tcp_orientation` | Virtual TCP. 동작은 [demo-task-controller.md](demo-task-controller.md#virtual-tcp). Joint · Task · Compliance 는 ROS 파라미터로도 선언하며 Inactive 일 때만 set 을 받는다 |
| `grasp_contact_threshold` / `grasp_force_threshold` / `grasp_min_fingertips` | grasp 감지 임계. 어느 키를 쓰는지는 센서 종류가 정한다 ([capability matrix](../README.md#grasp-threshold-capability-matrix)) |
| `grasp_controller_type` | `"contact_stop"` · `"force_pi"` · `"none"` (손 무개입, GraspState 발행은 유지). 그 외 YAML 값은 `on_configure` FAILURE, 파라미터로 들어온 그 외 값은 ERROR 로그 후 기존 값 유지 |
| `hand_finger_joint_map` · `hand_idx_*` | 손가락별 모터 index (force_pi 가 손가락 관절을 찾고, contact_stop release 게이트가 MCP_FE 를 본다) |
| `force_pi_grasp.*` | Force-PI grasp 게인과 손가락별 `q_open`/`q_close`. `fingers.finger_names` 의 순서가 thumb 슬롯을 정한다 — 목록에 `"thumb"` 이 없으면 슬롯 0 을 쓰고 WARN. 튜닝: [grasp_tuning_guide.md](../../rtc_controllers/docs/grasp_tuning_guide.md) |
| `pull_estimator.*` | in-plane pull-force 추정. 필터, plane normal source (`fixed` · `pinch_geometry`), baseline 차감, `required_roles`, `tips.<role>.*`. 키는 `rtc::grasp::PullEstimatorParams` 필드와 1:1 |
| `momentum_observer.*` | 일반화 운동량 관측기. 블록이 없으면 비활성. `gains` 는 스칼라 또는 정확히 arm dof 길이의 리스트. 출하값은 `enabled: false` |
| `momentum_observer.payload_estimator.*` | 관측기 잔차에서 payload 질량과 wrench 를 역산. 서브블록이 없으면 비활성, 출하값은 `enabled: false` |

계약의 세부는 각 헤더 주석이 갖는다: [pull_estimator_wiring.hpp](../include/integrated_bringup/support/pull_estimator_wiring.hpp) · [momentum_observer_wiring.hpp](../include/integrated_bringup/support/momentum_observer_wiring.hpp).

### 알아 둘 계약

- **pull estimator 의 힘 부호.** estimator 는 finger-on-object 입력을 합산하고 `force_sign` 기본값은 `+1` 이다. 부호가 뒤집히면 파지 중에도 추정이 0 으로 나온다.
- **`tips.tip_names`.** 물리적으로 참여할 수 있는 tip 전체를 적는다. WBC 는 같은 link 를 `tsid.contacts` 로도 풀어야 하므로 tsid contact 이 없는 tip 은 추가할 수 없다 (`ur5e_p1a` · `ur5e_p1b` 는 ring 제외, `iiwa7_leap` 은 포함). role 이름이 중복이면 `on_configure` FAILURE.
- **`momentum_observer` 입력.** arm (device 0) 위에서만 돈다. position · velocity · effort lane 중 하나라도 읽을 수 없으면 잔차를 0 으로 덮지 않고 직전 값으로 동결한다. `efforts` 는 관절 토크 [N·m] 여야 한다 — 전류를 재는 backend 는 경계에서 변환한다.
- **`payload_estimator` 의 정확도.** 대수는 단위 테스트로 고정돼 있으나 sim 에 알려진 질량을 매달 수단이 없어 `m̂` 의 정확도는 검증된 적이 없다. sim 의 게이트 임계는 실기에서 재튜닝해야 한다.

## Override 우선순위

`demo_shared.yaml` (기본) → `demo_*_controller.yaml` (컨트롤러별) 순이다. 컨트롤러 YAML 에 같은 키를 쓰면 그 컨트롤러만 덮어쓴다.

```yaml
# demo_task_controller.yaml 에서만 grasp 임계를 바꿀 때
demo_task_controller:
  grasp_force_threshold: 2.0   # demo_shared.yaml 의 1.0 을 override
```

## 팔 끝 frame

팔 끝 frame 은 `OnDeviceConfigsSet` 에서 군 0 의 device config `urdf.tip_link` 로 한 번 찾는다 ([support/arm_tip_resolution.hpp](../include/integrated_bringup/support/arm_tip_resolution.hpp)). 팔 모델이 있는데 풀리지 않으면 `on_configure` 가 FAILURE 이고, ERROR 가 원인을 찍는다.

| 원인 | 조건 |
|---|---|
| 이름이 모델의 frame 이 아니다 | `urdf.tip_link` (또는 `urdf.sub_models.<군>.tip_link`) 가 모델에 없다 |
| link 가 주어지지 않았다 | device config 에 `tip_link` 가 없다 — 군 이름이 `urdf.sub_models` 의 어느 항목과도 맞지 않아 `arm` 사슬로 대체된 구성이 그렇다 |

팔 모델이 없는 구성 (URDF 없이 올린 컨트롤러) 은 통과한다. 이 검사는 이름이 모델의 frame 인지만 본다 — 팔 끝이 아닌 link 를 적어도 풀리므로 `tip_link` 가 실제 팔 끝인지는 직접 확인한다.

군 0 의 모델은 그 device 이름으로 `urdf.sub_models` → `urdf.tree_models` → 이름 `arm` 의 사슬 순으로 찾는다. tree 인 군 (`g1_p1b` 의 `g1`) 은 팔 끝이 **손이 붙는 link = 군 1 tree 의 `root_link`** 다. 군 0 의 `joint_state_names` 가 모델에 없는 이름을 가지면 `on_configure` FAILURE 다.

## 손끝 FK 배선

손끝 pose 는 `T_root_fingertip = T_root_armtip(q_arm) · T_tip_mount · T_handroot_fingertip(q_hand)` 다. `T_tip_mount` 는 팔 끝 link 에서 손 root link 로 가는 상수 변환이고 (`iiwa7_leap` 은 5 mm · 90°), 팔 끝이 손 root 자신이면 항등이다 ([support/hand_fk_wiring.hpp](../include/integrated_bringup/support/hand_fk_wiring.hpp)). 손 모델이 있는 구성에서 다음은 `on_configure` FAILURE 다.

| 거부 사유 | 조건 |
|---|---|
| 관절 이름 | 손 device 의 `joint_state_names` 에 손 모델에 없는 이름이 있다 |
| 관절 폭 | 그 이름들이 손 모델의 관절을 빠짐없이 한 번씩 덮지 않는다 |
| 장착 | 손 tree 의 `root_link` 가 모델에 없거나, 팔 끝 link 와 같은 관절에 붙어 있지 않다 |

- 손 tree 모델의 `tree_models.<손>.tip_links` 에는 손가락을 전부 적는다. 빠진 손가락의 관절은 "모델에 없는 이름" 이 된다.
- 손 모델이 없는 구성 (군 1 이 없거나 같은 이름의 tree 모델이 없음) 은 배선할 것이 없어 통과한다.
- closed-chain hand FK 가 active 인 손은 이름·폭 검사를 적용하지 않는다 ([demo-task-controller.md](demo-task-controller.md#closed-chain-hand-fk)).

## Grasp mode 전환 (Joint · Task · Compliance)

활성 모드는 `grasp_controller_type` ROS 파라미터로 읽고 바꾼다. `demo_wbc_controller` 는 이 파라미터를 선언하지 않는다 (자체 FSM).

```bash
ros2 param get /demo_joint_controller/demo_joint_controller grasp_controller_type
ros2 param set /demo_joint_controller/demo_joint_controller grasp_controller_type force_pi
```

- **손이 quiet 할 때만 수락한다.** contact_stop latch 가 걸려 있지 않고 force_pi FSM 이 `kIdle` 이어야 한다. 아니면 사유와 함께 거부된다 (`GraspModeChangeRejectReason`). 이미 활성인 모드를 다시 요청하면 항상 수락한다.
- `force_pi` 는 `force_pi_grasp` 블록이 있는 config 에서만 가능하다. 블록 유무가 capability 이고 모드가 지금 어느 법칙이 도는가다 ([agent_docs/controllers.md](../../agent_docs/controllers.md#graspcontroller-force-pi-internal-only)).
- Force-PI grasp / release 는 `/<config_key>/grasp_command` 서비스 (`rtc_msgs/srv/GraspCommand`) 로 한다. 활성 모드가 `force_pi` 가 아니면 `ok=false` 와 사유 (`GraspCommandRejectReason`) 로 명시적으로 거부한다.
- GUI Grasp 탭의 `▶ Grasp` / `■ Release` 도 같은 서비스를 쓴다. 선택한 컨트롤러의 실제 모드를 읽어 `mode: force_pi` 일 때만 버튼을 활성화하고, `Grasp mode:` 콤보박스와 `Set Mode` 로 위 파라미터를 바꾼다. 거부 사유는 그대로 표시한다 ([demo-controller-gui.md](demo-controller-gui.md)).
- force_pi 모드에서는 phase 전이가 `[grasp:force_pi] phase X -> Y target_force=...N` 로, 상태 스냅샷이 2 초 간격 `[grasp] type=... active=.../... max_force=...N` 로 로그에 나온다.
