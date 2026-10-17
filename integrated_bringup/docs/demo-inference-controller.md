# DemoInferenceController

ONNX 정책으로 UR5e 와 proto_1b 손을 구동하는 position 컨트롤러다. 입출력 텐서 · 정규화 · 프레임 · 관절 부호 · 모델 경로가 전부 YAML 이라 모델을 바꿔도 재컴파일이 없다. config_key 는 `demo_inference_controller`, YAML 은 `ur5e_p1b` 만 싣는다 (`config/ur5e_p1b/controllers/demo_inference_controller.yaml`). 계약은 [agent_docs/controllers.md](../../agent_docs/controllers.md), 이유는 [controllers-rationale.md](../../docs/reference/controllers-rationale.md#demoinferencecontroller-근거) 에 있다.

## 실행

정책 모델은 저장소 밖에 둔다. `RTC_POLICY_DIR` 환경변수가 모델 디렉토리를 가리킨다.

```bash
export RTC_POLICY_DIR=/path/to/ObjectHandGraspDeployMulti5-Export-v0
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py \
    sim_overlay:=inference_pole use_cpu_affinity:=false

# 기동 후 (기본 활성은 demo_joint_controller):
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_inference_controller'], \
    deactivate_controllers: ['demo_joint_controller'], strictness: 1, timeout: {sec: 1}}"
```

`sim_overlay:=inference_pole` 은 정책의 학습 씬과 맞는 장면이다 ([README](../README.md) 의 sim overlay 절).

- **모델 없음.** `RTC_POLICY_DIR` 가 설정돼 있지 않으면 `inference.allow_missing_model` 에 따라 hold 모드로 올라오고 경고에 변수 이름이 찍힌다. 변수가 있는데 파일이 없거나 이름 · shape 이 YAML 선언과 다르면 configure 실패다.
- **정책 주기.** `inference.decimation: 5` — 500 Hz 에서 5 tick 마다 정책 1 step (100 Hz).
- **입력.** 정책 root frame 은 `inference.policy_frame: "base"`, 물체 pose 는 `inference.object_pose.topic` (`/sim/object_transforms`) 의 TFMessage 에서 읽는다. 외부 `joint_goal` 은 무시된다 (정책이 명령을 소유한다).
- **출력.** 정책 목표를 joint command tail (`joint_limit_margin`) 로 clamp · rate-bound 해 낸다. `command_type` 은 `"position"` 만 받는다.

## 로그

`<session>/controllers/demo_inference_controller/` 아래 `ur5e_state.csv` · `p1b_state.csv` · `inference_diag.csv` 가 매 tick 한 행으로 남는다 (state 의 `joint_goal` 열이 정책 목표). 이 컨트롤러는 모든 실패가 hold 로 수렴해 (입력 읽기 불가 · `Run()` 실패 · 비유한 출력 · closed-chain held · 물체 stale) 바깥에서는 정상 동작과 구분되지 않는다. 그래서 `inference_diag.csv` 가 매 tick `held` 와 사유 코드 (`hold_code` / `hold_reason`) 를 남기고, 정책이 실제로 본 것을 함께 둔다.

| 열 | 뜻 |
|---|---|
| `policy_step` · `inference_count` | 정책 step 과 실행 횟수 (주기가 decimation 대로인지) |
| `reach_phase` · `tip_distance` · `reach_hold` | reach gate 상태 |
| `object_valid` · `object_age_s` · `object_x..z` | 정책이 본 물체 (`policy_frame` 기준 — 프레임이 어긋나면 부호로 드러난다) |
| `closed_held` · `closed_held_ticks` · `closed_singular` · `closure_error` | closed-chain 사영 상태 |
| `arm_lag_max` | 정책 목표와 측정 관절각의 최대 차 (sim 서보의 지연이 여기 실린다) |
| `force_<tip>` | reach gate 가 읽는 손끝별 힘 |

컬럼의 정확한 뜻은 [inference_diag_log_pod.hpp](../include/integrated_bringup/logging/inference_diag_log_pod.hpp) 가 갖는다. 파일이 없으면 `logs:` 블록에 그 채널이 없다는 뜻이다. 전체 목록은 [controller-logs.md](controller-logs.md).
