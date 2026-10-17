# DemoComplianceController

[DemoTaskController](demo-task-controller.md) 에 외부 렌치 task-admittance 를 더한 position 출력 컨트롤러다. 팔은 admittance 로 모양 잡은 CLIK, 손은 P 제어다. 렌치가 없으면 DemoTask 와 같은 명령을 낸다. config_key 는 `demo_compliance_controller`, YAML 은 `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` 이 싣는다.

공통 파라미터 · 팔 끝 frame · 손끝 FK 배선은 [demo-shared-params.md](demo-shared-params.md) 가 갖는다. 법칙의 규범은 [compliance-conventions.md](../../rtc_controllers/docs/compliance-conventions.md), 근거는 [controllers-rationale.md](../../docs/reference/controllers-rationale.md) 에 있다.

## 설정

- YAML 은 같은 로봇의 `demo_task_controller.yaml` 의 사본에서 최상위 키를 바꾸고 이 컨트롤러 고유 블록을 더한 것이다. 고유 블록 밖의 값은 두 파일이 같아야 한다 (`test_compliance_task_equivalence.cpp` 가 비교한다).
- task 게인의 철자가 다르다 — `ik_kp_pos` · `ik_kp_rot` · `nullspace_kp` (Task 의 `kp_translation` · `kp_rotation` · `null_kp`). 같은 양이고 값도 같게 둔다.
- 쓰여 있는 admittance 수치는 하드웨어에서 튜닝한 값이 아니라 코어 기본값을 명시한 것이다.
- **외부 렌치 source.** `external_wrench.source` 는 필수 키이고 지금 받는 값은 `"pull_estimator"` (손 레인의 pull 추정) 하나다. 그 외 값은 오타와 같이 거부된다.
- **활성화 순서.** 이 컨트롤러는 손이 닫히기 **전에** 활성화한다. 활성화가 손 목표를 측정 자세로 다시 잡으므로, `grasp_controller_type: "none"` 에서 손이 쥐고 있던 물체는 활성화 순간 놓이고 pull 추정의 입력이 사라진다.
- 목표 입력은 Task 와 같다: 팔은 task 목표, 손은 joint 목표 ([Task 목표의 기준 frame](../README.md#task-목표의-기준-frame-779)).

## 로그

`compliance_diag.csv` 는 매 tick 한 행이다. 컬럼은 소비된 wrench (`wrench_f*` / `wrench_t*`), 소스 상태 (`wrench_source` · `quality_low` · `invalid_reason`), 파이프라인 상태 (`wrench_age` · `fade` · `stale` · `in_contact` · `bias_*` · `rejected_samples`), 법칙 상태 (`fsm_state` · `alpha` · `x_tilde_*` · `nu_c_*` · `task_origin_*` · `disp_limited` · `vel_limited` · `adm_finite`), 매 tick 파라미터 스냅샷 (`kp_*` · `kd_*` · `md_*` · `max_disp_*`) 이다. 평평한 x̃ 가 수렴인지 §7.5 박스에 pin 된 것인지는 `disp_limited` 가 가른다. 법칙이 안 돈 tick 은 `valid=0` 행과 값 0 이다. 전체 파일 목록은 [controller-logs.md](controller-logs.md).

## compliance §7.3 관절 밴드 판독

위치 출력 바인딩 둘 (`demo_compliance` · `demo_task`) 은 명령을 스스로 적분하고 compliance §7.3 관절 tail ([joint_command_tail.hpp](../../rtc_controllers/include/rtc_controllers/compliance/joint_command_tail.hpp)) 로 바운드한다. 그 tail 이 이번 tick 에 몇 개 관절을 움직였는지를 돌려준다. 이 값이 없으면 밴드에 눌려 멈춘 팔과 정착한 팔이 같은 평평한 궤적으로 보인다. task 축은 `compliance_diag.csv` 의 `disp_limited` 가, 관절 축은 아래 로그가 답한다.

채널이 둘인 이유는 종료 경로다. CM 의 `main()` 은 SIGINT 에서 executor 만 정리하고 리턴하므로 Ctrl-C 는 lifecycle 훅을 타지 않는다. 요약만 두면 Ctrl-C 로 끝낸 bring-up 에서는 아무것도 안 나온다.

| 채널 | 레벨 | 언제 | 읽는 법 |
|---|---|---|---|
| 발동 중 (`... joint band engaged: N joint(s) position-clamped, M rate-rebounded`) | `WARN`, `kThrottleSlowMs` | clamp 나 rate 재바운드가 실제로 걸린 tick 에서만 | 이 줄이 찍히는 구간의 평평한 관절 궤적은 정착이 아니라 pin 이다. 타임스탬프가 곧 "언제 눌렸나" |
| 활성화 요약 (`... joint band engaged this activation: ...`) | `INFO` | `on_deactivate` — 컨트롤러 전환 및 명시적 `ros2 lifecycle set … deactivate/shutdown`. Ctrl-C 에는 안 뜬다 | tick 창 (`ticks A..B`) 을 그 run 의 다른 로그 · CSV 와 대조한다. 한 번도 안 걸렸으면 줄 자체가 없다 |

`demo_compliance` 는 §7 의 `joint_limit_margin` (δ) 을 함께 찍는다. `demo_task` 는 δ 노브가 없어 raw device 한계에서 clamp 하므로 그 사실을 문구로 밝힌다 — 후자의 clamp 는 soft margin 이 아니라 실제 관절 한계 도달이라 더 무거운 판독이다. 누적기는 [support/joint_tail_stats.hpp](../include/integrated_bringup/support/joint_tail_stats.hpp).
