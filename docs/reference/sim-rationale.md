# rtc_mujoco_sim — 근거·실측·결정 이력

> **이 문서는 헌법이 아니다.** 사용법 (토픽 · 서비스 · 파라미터 · 뷰어) 은 [rtc_mujoco_sim/README.md](../../rtc_mujoco_sim/README.md) 가 갖고, 여기는 그 동작이 지금 모양이 된 이유와 측정이다. 둘이 어긋나면 README 와 YAML 이 옳다. 수치는 기록 시점의 것이므로 인용하기 전에 다시 잰다.

## 동기 루프와 controller tick

- sim 은 **모든 robot 그룹**의 command 를 기다린 뒤 한 step 을 진행한다. 첫 그룹만 기다리던 시기에는 손 명령이 한 step 늦게 적용될 수 있었다. 기다리지 않을 그룹은 `wait_for_command: false` 로 뺀다. 안 빼면 매 step 이 `sync_timeout_ms` 를 기다린다.
- `reset` 요청은 이 대기를 끊지 않는다. 이번 step 의 명령이 다 온 뒤에 reset 하므로 pre-reset state 에 대한 명령이 reset 뒤로 새지 않는다. 대기 중 도착한 공 reset · object refresh 는 적용한 뒤 받은 명령으로 그대로 step 한다.
- `use_sim_time_sync: true` 인 controller manager 는 state 가 도착하면 eventfd 로 깨어난다 (round-trip 약 1 ms → 0.35 ms). **tick 은 state 메시지가 아니라 sim step 마다 한 번**이다. group 마다 state 를 따로 발행하므로 CM 은 `sim_sync_tick_devices` 의 state 가 전부 새로 도착했을 때 한 번 tick 한다.
- 메시지마다 tick 하던 시기에는 group 이 2 개인 씬 (ur5e_p1b) 에서 step 500 Hz 당 838 Hz 로 돌았고, `state.dt` 를 적분하는 법칙이 sim 보다 약 1.67 배 빨리 흘렀다. 그 시기의 sim 측정 (궤적 시간 · 서보 지연 · 포구 기준 오프셋) 은 이 오차를 품고 있다.
- `control_rate` 와 `physics_timestep` 이 다르면 기동 FATAL 인 이유: lock-step 에서 step 하나가 tick 하나이므로 physics 가 더 빠르면 `dt` 적분이 sim 시간과 어긋난다.

## 스레딩 모델 (기록 시점)

`SimLoop` 스레드 (동기: state → wait → step → throttle) 와 선택적 `ViewerLoop` 스레드 (GLFW, 약 60 Hz) 두 개다. SimLoop 은 그룹별 `cmd_mutex` + `cmd_pending` (atomic) 으로 명령을 받고, `sync_cv_` 로 모든 robot 그룹의 명령을 기다린다. 그룹별 `state_mutex` 가 최신 상태 스냅샷을, `solver_stats_mutex_` 가 solver 통계를 지킨다. `pert_mutex_` (퍼튜베이션 · 외력) 와 viewer 의 `viz_mutex_` 는 `try_lock` 전용이라 SimLoop 을 블로킹하지 않는다. fake 그룹은 노드의 100 Hz 타이머가 `AdvanceFakeLPF` / `GetFakeState` 를 부른다.

로그 prefix 는 출력 스레드를 식별한다: `[MuJoCoSimulatorNode]` (executor 콜백), `[MuJoCoSimulator]` (Initialize · VFS resolver), `[SimLoop]`, `[Viewer]`. 새 메시지는 스레드에 맞는 prefix 와 Sentence-case 를 쓴다.

## Constraint solver 설정

shipped 기본값의 SSoT 는 `config/solver_param.yaml` 이다. 코드 fallback (`declare_parameter`) 은 파라미터가 어디서도 설정되지 않을 때만 쓰이고 shipped 와 다르다 (예: `cone` pyramidal vs elliptic, `integrator` Euler vs implicitfast, `iterations` 100 vs 20, `multiccd` false vs true, `n_substeps` 1 vs 3). 표로 둘을 함께 박제하면 drift 하므로 README 는 YAML 을 가리킨다.

XML 우선순위 감지는 `Initialize()` 가 tinyxml2 로 원본 XML 을 파싱해 명시된 속성 집합을 얻는 방식이다. tinyxml2 는 `package://` 를 해석하지 못하므로 `ResolveModelPath()` 가 ament 로 절대 경로를 만든 뒤 넘긴다. 이 단계가 fallback 만 동작하면 모든 solver 파라미터가 YAML 로 대체되어 XML 우선순위가 사라진다.

contact point 시각화 크기는 `ApplySolverConfig()` 가 모델 로드 직후 MuJoCo 기본 (`contactwidth=0.3`, `contactheight=0.1`) 의 1/10 로 줄인다. manipulator scale 에서 과도하게 크기 때문이며 physics 와 무관하다.

## 지오메트리 group 가시성 함정

`mjv_defaultOption()` 은 `geomgroup` 을 `[1, 1, 1, 0, 0, 0]` 으로 초기화한다 (MuJoCo 3.7.0 실측). group 3 이상은 launch 직후 숨겨진다. robot-only MJCF 의 main `<default>` 가 group 3 이면, 그 파일을 `<include>` 하는 scene 의 floor · table 같은 visual geom 이 group 을 명시하지 않을 때 default 를 상속받아 함께 사라진다. "뷰어에 X 가 안 보인다" 는 `m->geom_group[i]` 가 0..2 인지 보는 것부터 시작한다 (모델 정보 오버레이 `F10`).

## 제어 모드와 effort

- Position servo 모드는 MuJoCo 의 per-body `body_gravcomp` 를 그룹의 link body 에 1.0 으로 둔다. 전역 `opt.gravity` 는 그대로라 같은 씬의 free body 는 정상적으로 떨어진다.
- `MapGroupIndices()` 는 그룹 command joint 가 매달린 body 를 seed 로 잡고 child 방향 BFS 로 `body_jntnum == 0` (fixed link) descendant 를 그룹에 넣는다. mount · wrist adapter · tool · hand 의 중력이 leaf joint 에 외부 토크로 작용해 PD 정상상태 처짐을 만드는 것을 막는다. 다른 group 의 joint 를 가진 body 에서 멈추므로 multi-group 소유권은 깨끗하다. 초기화 로그 `gravcomp body chain: N body(ies) (seed S + fixed-link descendants D)` 가 두 값을 나눠 보여준다.
- MuJoCo `mj_passive()` 는 `mjModel.ngravcomp == 0` 이면 gravcomp 루프를 건너뛴다. MJCF 가 `gravcomp` 를 명시하지 않으면 컴파일 시 카운트가 0 이라 런타임에 `body_gravcomp[]` 만 써도 효과가 없다. 그래서 `Init` / `PreparePhysicsStep` 에서 mutate 한 직후 `RefreshNgravcomp()` 가 다시 센다. 회귀 가드는 `test_gravcomp_scene` 의 `GravcompForceActuallyAppliedAfterForward` 다.
- **effort 정의**: `qfrc_actuator + qfrc_applied + qfrc_gravcomp`. `DeviceState::effort` lane 계약이 실기 · sim 양쪽에 동일하게 걸리고 실기는 command mode 와 무관하게 관절 토크를 보고하므로 sim 도 모드에 따라 의미가 달라지면 안 된다. `gravcomp` 항이 빠지면 position 모드의 effort 는 "중력이 상쇄된 뒤 PD 가 추가로 낸 토크" 가 되어 조용히 다른 물리량이 된다. gravcomp 가 OFF 인 모드에서 MuJoCo 는 `qfrc_gravcomp` 를 0 으로 기입하므로 (`mj_passive` 는 early-out 전에 배열을 0 으로 덮는다) 합산에 모드 분기가 필요 없다. `qfrc_passive` 전체를 더하지 않는 이유는 spring / damper / fluid 가 섞여 있어서다 (iiwa7 은 관절 damping 1.0–3.0 이라 `qvel=0.5` 에서 1.5 N·m) — 그것은 모델의 마찰이고 관측기 residual 에 남는다.
- torque 모드에서 서보 (`<position>` · `<velocity>`) actuator 의 `ctrlrange` 는 서보 **목표** 범위 (rad · rad/s) 이지 토크 한계가 아니므로 모드에 있는 동안 clamp 를 끄고 나오면 XML 값으로 되돌린다. 토크를 묶는 것은 `forcerange` 와 관절 `actuatorfrcrange` 이고, 둘 다 없는 서보 actuator 는 torque 모드에서 한계가 없다. `<motor>` 의 `ctrlrange` 는 원래 토크 한계라 그대로 걸린다. 다루지 않는 것: `<damper>`, 내부 dynamics 가 있는 actuator, `gear` ≠ 1 인 서보.
- `pd_feedforward`: `tau_ff` 를 `qfrc_applied[dofadr]` 에 주입하고 `body_gravcomp` 는 0 이다. 컨트롤러의 `feedforward` 가 중력 보상까지 포함해야 하며, 빠지면 팔이 조용히 처진다. 프레임워크 전역 `rtc::CommandType` enum 이 2-값 (`position` / `torque`) 이라 rtc 컨트롤러 → 백엔드 스택은 이 값을 발행하지 못하고 sim 테스트나 직접 publish 로만 도달한다. `command_type` 은 수신자별로 해석되므로 (`udp_hand_driver` 는 `values` 를 항상 position 으로 읽고, proto2 CAN FD 만 `feedforward` 를 전류 명령으로 덧붙인다) 공유 토픽에 `pd_feedforward` 를 발행해도 hand 의 위치 명령은 그대로다.
- `use_yaml_servo_gains: true` (또는 런타임 게인) 일 때 actuator 의 bias 타입을 affine 으로 맞춘다. 설치되는 서보는 actuator 단위 (길이 = gear · q, ctrl 은 `ctrlrange` 로 clamp) 라 `gear` ≠ 1 이거나 `ctrlrange` 를 가진 `<motor>` 는 목표가 1/gear 로 줄고 그 범위로 잘린다. 그래서 게인을 설치할 때 그룹당 한 번 경고하며, 토크용 `<motor>` 는 `gear="1"` + `forcerange` 로 쓴다.

## 외력 주입 (`/sim/set_external_wrench`)

계약의 SSoT 는 `rtc_msgs/srv/SetExternalWrench.srv` 주석이다. 여기는 측정 근거만 둔다.

- MuJoCo `xfrc_applied` 는 body 질량중심 (`xipos`) 에 걸린다. 순수 힘을 걸고 `qfrc_smooth` 차분을 자코비안과 대조한 실측: `J(xipos)ᵀw` 와 1e-14, `J(xpos)ᵀw` 와는 1.7e-1.
- 질량중심은 body 프레임 원점과 다르다. iiwa7_leap 에서는 모든 body 가 0 이 아닌 오프셋을 갖고 (`ee_link` 35 mm, 최대 13 cm) 질량 0 인 body 도 그렇다. 요청을 그대로 전달하면 10 N 당 0.25 N·m 의 모멘트가 붙어 payload 추정기 노이즈 바닥 (0.048 N·m) 의 5 배가 되고 "실재하지 않는 CoM 오프셋을 가진 물체" 로 읽힌다.
- 그래서 SimLoop 이 **매 tick** `xpos` / `xipos` / `xmat` 에서 CoM 기준 등가 wrench 를 다시 만든다. 요청 시점에 한 번 계산하면 팔이 움직이는 순간 stale 이다. `point: {0,0,0}` 은 body 프레임 원점이고 같은 이름의 pinocchio 프레임과 일치한다 (home 자세의 `ee_link` 에서 1e-9 m 이내).
- 없는 body 와 `world` 는 거부한다. 조용히 무시하면 호출자는 부하가 걸렸다고 믿고 추정기는 아무것도 못 본다. `world` 는 고정 베이스가 흡수해 아무 일도 일어나지 않는다.
- `JointState.effort` 에 이 외력이 들어가지 않는 것은 의도다 (`xfrc_applied` 는 effort 합산 항이 아니다). 관측기가 이를 미지의 외란 토크로 봐야 한다 — effort 에 섞이면 관측기 입력 `τ_m` 이 이미 외력을 포함해 residual 이 0 으로 남는 조용한 실패가 된다.

## Contact wrench lane

- **부호 (link-on-environment)**: `rtc_msgs/FingertipSensor.f` 가 finger-on-object 이고 (proto-1b firmware, 1a ONNX) `rtc::grasp::PullContactConfig::force_sign` 기본값이 `+1` 이므로 sim lane 과 실기 센서 lane 이 같은 부호여야 `demo_shared.yaml` 의 `pull_estimator` 블록이 양쪽에서 그대로 맞는다. MuJoCo `reduce="netforce"` 가 내는 geom1-on-environment 값을 그대로 통과시킨다 (`<contact body1=...>` 의 body 가 손끝). per-tip 반전 (`force_sign: -1.0`) 이 들어가면 파지 중에도 `f_n` 이 전 contact 음수가 되어 all-zero estimate 가 나간다. 두 oracle 이 부호를 고정한다: `test_contact_wrench_known_load` (알려진 외력이 lane 의 world 합으로 되나타나는가) 와 `ShippedPullEstimator` (출하 프로필로 파싱해 in-plane 성분을 뽑는가).
- **`reference_frame`**: `"body"` 와 `"site"` 는 site 가 body 대비 non-identity quat 을 가질 때만 정확히 그 회전만큼 갈라진다. 회전된 wrench 도 그럴듯하므로 크기 · 부호 · NaN 검사로 구별되지 않는다. `"site"` 가 필요한 경우는 소비자가 `R_link(q)` 를 적용하는 URDF 프레임이 site 가 매달린 MJCF body 와 다를 때뿐이다 (ur5e_p1b 의 hand: index / middle / ring 90°, thumb 은 복합). 기본이 `"body"` 인 것은 하위호환이다 — 기존 씬은 ft_site 에 quat 이 없어 수치는 같지만 `frame_id` 가 다르다. 오타는 fallback 없이 Initialize 실패다.
- **`publish_debug`**: wrench 는 손끝이 얼마나 세게 누르는지를 각자 프레임으로 말하므로, 두 손끝이 물체를 마주 눌러 가두는지 같은 쪽으로 미는지 구별하지 못한다 (180° 로 마주 선 1.5 N 두 개와 90° 로 벌어진 1.5 N 두 개는 크기가 같다). 그 구별은 접촉점이 정하는데 폐쇄 체인 손에서는 `joint_states` 로 재구성할 수 없다 (수동 linkage 관절 때문에 허상이 나온다 — [testing-debug.md](../../agent_docs/testing-debug.md)). `frame_id` 가 `"world"` 인 것은 `object_state` 기본 fallback 과 같은 문자열이라 접촉점과 물체 pose 를 변환 없이 같은 표에 놓을 수 있다. 기본 OFF 인 이유는 진단이기 때문이다.
- **viewer 화살표**: 기본 ON 인 이유는 센서를 설정했다면 그 값을 보는 것이 기대 동작이고 "0 을 발행 중" 과 "발행 자체가 없음" 은 눈으로 보기 전까지 구별되지 않기 때문이다. 맨 `F` (`mjVIS_CONTACTFORCE`) 는 뷰어 전용 `mjData` (qpos 만 동기화, qvel · ctrl 은 0) 에서 다시 푼 접촉이라 sim 의 값도 센서의 netforce 도 아니다. 그래서 화살표는 물리 스레드가 `qpos` 와 같은 임계구역에서 넘긴 스냅샷으로 그린다. 길이 0.3 m clamp 와 0.1 N 미만 미표시는 렌더러 상수다.

## `initial_qpos`

씬에 keyframe 이 없으면 `qpos0` 에서 시작하는데 arm 은 그것이 뻗은 자세이고, MJCF 가 다른 패키지 소유라 keyframe 을 추가할 수 없는 bringup 도 있다. 반대로 keyframe 이 있는 씬에 이 키를 쓰면 keyframe 은 그 그룹에 한해 무시되므로 두 값을 같이 두려면 테스트로 고정한다 (`integrated_bringup/test/test_shipped_initial_qpos.py`). 소수점이 없는 `[0, -1, 1]` 은 정수 배열이 되어 선언된 double 배열과 타입이 달라 `on_configure` 전, 노드 생성 시점에 예외로 죽는다 (ROS 2 params YAML 은 시퀀스 타입을 원소에서 추론한다).

## Object Pool

- 목적: [scene_right_with_object.xml](../../robot_descriptions/robots/iiwa7_leap/mjcf/scene_right_with_object.xml) 에 주석으로 박힌 5단계 수작업 (mesh 항목 복사, mesh 이름 prefix, body 블록 복제, 모든 keyframe 에 freejoint qpos 7 개 append) 중 4 단계를 로드 시점에 자동화한다. object_sim 은 `contact0` 같은 generic 이름을 57 개 object 가 공유해서 prefix 가 필요하다.
- 후보 전부를 `mj_compile` **이전에** spec 에 attach 한다. `mjModel` / `mjData` 가 `Initialize` 이후 불변이라는 클래스 전제가 유지되고 viewer 스레드의 `mjvScene` / `mjrContext` 를 재생성하지 않는다. `o` 는 현재 object 를 park 하고 다른 것을 unpark 할 뿐이다.
- park 는 세 가지를 동시에 한다: `contype` / `conaffinity` = 0 (빠지면 충돌은 없어지지만 영원히 낙하), `body_gravcomp` = 1 (위와 동일), freejoint `qvel` · `qacc_warmstart` = 0 (빠지면 낙하 중 `o` 에서 등속 표류). park 위치는 spec `pos` 로 들어가 `qpos0` 가 되므로 `R` 이 pool 을 자동으로 park 로 되돌린다. reset 이 주사위를 다시 굴리면 "리셋하고 재시도" 가 같은 실험의 반복이 아니므로 재추첨하지 않는다.
- 실패는 전부 Initialize 실패다 (후보 0 개, 없는 `directory`, 후보에 없는 `fixed_object`, 디렉토리에 없는 `objects:` 항목, 허용 밖 `selection` / `pose`, 3 원소가 아닌 `position` 계열). 활성화됐는데 빈 pool 은 "무작위화하는 척하며 아무것도 안 하는" 상태이기 때문이다. `objects: []` 는 `rcl_yaml_param_parser` 가 원소 타입을 추론하지 못해 `PARAMETER_NOT_SET` 으로 노드 생성 시점에 죽으므로 키를 생략한다.
- 비용은 후보 수에 비례한다 (물리 step · 컴파일 시간 · RSS). 16 개를 넘으면 경고한다. `enabled: false` 면 `nq` / `nbody` / `ngeom` 이 이 블록이 없을 때와 같고 테스트로 고정돼 있다.
- 씬이 이미 object 를 하드코딩하면 (`apple_01`) prefix 가 달라 충돌하지는 않지만 사과가 두 개가 된다. `<sensor><contact>` 는 `geom1=` (fingertip geom) 로 필터링하므로 어떤 object 가 활성이든 contact wrench 파이프라인은 그대로다.

## Object State

- 규칙은 "freejoint 를 가진 body 전부, 단 park 된 pool 후보와 park 된 projectile ball 은 제외" 이며 이름 목록이 없다. park 된 후보는 모델에 컴파일돼 있지만 바닥 아래 50 m 에 있어서 발행하면 소비자의 프레임 집합에 유령 object 가 생긴다. park 판정은 매 tick 다시 해야 `o` 키 refresh 와 공의 발사 · reset 이 즉시 반영된다.
- `/tf` 가 아닌 이유: sim ground truth 는 실기가 발행하는 트리에 섞이지 않아야 한다. RViz 에서 보려면 이 토픽을 `/tf` 로 재발행하는 노드를 붙인다.
- `reference_body` 선택이 의식적이어야 하는 이유: base 에 180° yaw 를 주는 배치가 흔하고 (`ur5e_p1b` MJCF base 는 `quat "0 0 0 -1"`) 그러면 world 와 base 는 다른 프레임이다. 헷갈리면 모든 object 가 base 를 통과한 거울상 위치에 찍히는데 값은 그럴듯해서 소비자가 알아챌 수 없다. `""` (world) 를 고르면 `world` 는 대개 bringup 의 TF 트리에 없다 (`robot_state_publisher` 루트는 로봇 base 이고 world→base 를 내는 노드는 없다). 출하 프로필 ur5e_p1b 는 world 이며 sim ground truth 를 실기 TF 트리에 잇지 않고 절대 좌표를 낸다.

## Projectile Ball

### 물성 preset 의 근거

| 물성 | `tennis` | `hard` | `beanbag` |
|---|---|---|---|
| I / (m r²) | 0.55 [Cross 2003] | 0.378 [Brody] | 0.4 (추정 — 충전물이 움직임) |
| 반발계수 (강체 바닥) | 0.75 [ITF 낙하 시험 0.745] | 0.55 [ASTM F1887 0.546 @27 m/s; 저속은 더 높음, 추정] | 0.1 (추정) |
| 미끄럼 마찰 μ | 0.6 [Cross 2003, hard court 0.6–0.75] | 0.5 [Sawicki 2003] | 0.5 (추정) |
| 비틀림 마찰 / r | 0.05 (추정) | 0.02 (추정) | 0.3 (추정) |
| 구름 저항 / r | 0.02 (추정) | 0.005 (추정) | 0.5 (추정 — 거의 안 구름) |
| 항력 Cd | 0.55 [자유비행 0.51 – 풍동 0.65] | 0.5 [Cross, Kensrud: 아임계 Re] | 0.5 (추정) |
| 양력 C_L(S) | S / (0.981 + 2.022 S) [Stepanek 1988] | S / (0.667 + 1.48 S) [Sawicki 2003 bilinear 에 맞춤] | 없음 |

S = r·|ω⊥| / |v|. 던지는 속도 (2–10 m/s, Re ≈ 10⁴–6×10⁴) 는 drag crisis 아래이므로 Cd 는 상수다. 값을 바꾸려면 `src/projectile_ball.cpp` 의 preset 상수와 README 표를 함께 고친다. 비틀림 · 구름 비율은 측정값을 찾지 못한 추정이다. 반발 · 마찰 · 구름은 강체 바닥 기준이다.

### 접촉과 공력

- 공 geom 은 `condim=6` (비틀림 · 구름 마찰이 실제로 작동) + `priority=100` 이라 바닥 · 테이블 · 손끝 어느 상대와 부딪혀도 공의 반발 · 마찰이 상대 geom 의 solref / friction 과 섞이지 않고 그대로 적용된다. priority ≥ 100 인 geom 이 씬에 있으면 Initialize 가 실패한다.
- 반발은 direct 형식 `solref = (−k, −b)` 이고, k 는 접촉 반주기가 physics substep 12 개 (`kContactSubsteps`) 가 되도록, b 는 누르기만 하는 spring-damper 의 반발이 목표값이 되도록 정한다. 8 개는 이미 낮게 치우치고 3–4 개는 1.0 을 넘는다. MuJoCo 3.7.0 실측으로 2–6 m/s 충돌에서 목표 ±0.04 (substep 0.67–2 ms) 다. 대가로 접촉이 실제 공보다 부드러워 6 m/s 충돌 시 약 1 cm 파고든다.
- 공력은 항력 −½ρC_d·A·|v|v 와 Magnus 양력 ½ρC_L·A·|v|² (ω×v 방향) 을 공의 `qfrc_applied` 에만 매 substep 쓴다 (ρ = 1.204 kg/m³). MuJoCo 내장 fluid 모델은 전역 `density` 를 켜야 해서 로봇 링크까지 끌리므로 쓰지 않는다. 57 g 테니스공을 5 m/s · 45° 로 던지면 2 m 지점에서 진공 궤적보다 약 5 cm 낮고, 가벼운 공일수록 차이가 크다.
- `solver.contact_override.enable: true` 이면 MuJoCo 가 모든 접촉을 override 값으로 덮어 preset 반발이 적용되지 않는다. `solver.island: false` 에서는 씬 어딘가에 깊게 파묻힌 접촉 (바닥에 10 cm 박힌 링크) 이 같은 solve 를 공유해 공 접촉을 흔든다 (fixture 에서 20 mm 튐, 멈추지 않는 구름으로 실측). 공이 이유 없이 튀면 씬의 관통부터 확인한다.
- park 는 세 가지를 함께 한다 (object pool 과 같은 이유). MuJoCo plane 은 halfspace 라 바닥 아래 park 위치에 접촉이 켜진 공을 두면 관통으로 판정돼 수백 m/s 로 튕겨 나간다 (접촉 끄기). 접촉을 끄면 붙잡는 것이 없어 자유낙하한다 (gravcomp). gravcomp 만으로는 움직이던 공이 등속으로 흘러간다 (속도 · warm start · 공력 지우기).
- 공 geom 을 **group 0 · 노란색**으로 붙이는 이유: main default 가 group 3 인 씬에서 그 default 를 물려받으면 viewer 가 기본으로 숨기는 그룹이 되어 물리는 도는데 공이 그려지지 않는다.
- `seed` 는 발사 샘플링과 카메라 노이즈에 함께 쓰이지만 노이즈 쪽은 stream tag 를 섞어 두 난수열이 겹치지 않는다. variation 이 0 인 항목은 난수를 소비하지 않는다.

### stamp — sim 시간축을 wall 에 사상한 값

발행 gate 가 sim 시간 기준 (`ShouldPublishProjectileBallSample`) 이라 샘플은 sim 축에서 정확히 `1/sample_rate_hz` 간격이다. `header.stamp` 도 그 간격을 말해야 한다. 발행 순간의 `now()` 를 찍으면 stepper 가 여러 step 을 몰아 돌고 자는 리듬이 stamp 에 실린다 (2026-09-22 실측, `max_rtf` 1.0 · RTF 1.0x: 10 ms 주기의 stamp 간격이 p05 2.7 / p95 33.6 / max 52 ms). stamp 를 capture time 으로 믿는 소비자 (ball_perception `stamp_is_capture_time: true` 의 init 창 "5 점을 0.1 s 안에") 는 이 지터를 초기화 실패로 겪는다.

그래서 두 공 토픽의 stamp 는 `ProjectileBallSample::stamp_steady_ns` = 발사 wall 시각 + (sim 시각 − 발사 sim 시각) / `max_rtf` 를 발행 시점의 steady↔system 오프셋으로 ROS 시각에 변환한 값이다.

- **epoch 은 wall 그대로** (ROS system time, `/clock` 없음) 이고 바뀐 것은 지터뿐이다. wall 과의 차이는 그 비행 안에서 stepper 가 뒤처지거나 (stamp 가 wall 보다 늦음) 따라잡은 (앞섬) 양, 즉 D-3 의 위상 오차 δ 이며 wake 지터가 아니다.
- stamp 는 wall 을 수십 ms 앞설 수 있다. sim 에서는 소비자의 future 허용치가 그 폭을 덮어야 한다 — ball_perception profile `max_future_skew_s` 0.1 s, 컨트롤러 `io.future_tol` 도 sim overlay 에서는 같은 자릿수 (L1 §6).
- **기각한 변형**: 앞서지 않게 발행 순간으로 clamp. sim 이 발사 pace 를 앞설 때마다 stamp 가 wall 로 떨어져 wake 지터가 그대로 돌아온다 (2026-09-22 재실측: 25 ms 초과 3.8 %, T_det max 0.41 s). 소비자의 stale 판정은 수신 steady 시각 기준 (D-2) 이라 뒤처진 stamp 는 계약에 어긋나지 않는다.
- 발사 전에 쌓인 sim↔wall 지연은 비행의 stamp 에 들어가지 않는다. anchor 가 발사 순간이므로 sync timeout 으로만 step 하던 구간이나 pause 로 sim 이 초 단위로 뒤처져 있어도 stamp 는 wall 에서 시작한다. loop 시작 기준 anchor 였다면 그 지연이 모든 stamp 에 박혀 컨트롤러가 계산하는 포구 시각이 그만큼 이르게 틀린다.
- `max_rtf: 0` (무제한) 이면 대응 기준이 없어 발행 순간의 `now()` 를 쓴다. 이때는 stamp 축의 동역학 자체가 틀리므로 추정기를 붙이는 run 은 `max_rtf: 1.0` 으로 돌린다.
- 두 토픽이 같은 stamp 를 받으므로 ball_perception 의 truth↔camera stamp 짝짓기는 영향이 없다.
- **stamp 를 sim 시각 epoch 으로 바꾸지 않는 이유**: ball_perception 의 `recorder_node` · `evaluator_node` 가 `/sim/ball/ground_truth` 와 카메라 토픽을 stamp 로 짝짓기 때문에 (`evaluation.time_alignment_tolerance_s`) 한쪽만 축을 바꾸면 둘 다 조용히 깨진다. 스택 전체를 sim epoch 으로 옮기려면 rtc 가 `/clock` 을 내고 모두가 `use_sim_time` 으로 돌아야 하며 그것은 S5/S6 결정이다.

### `/sim/launch_ball_at` 이 필요했던 이유

`std_srvs/Trigger` 는 요청 필드가 없어 한 투척을 고정한 채 다른 변수만 바꿔 재실행할 수 없었다 (D-14). 같은 writer 로 가는 두 번째 경로이며 RNG 를 소비하지 않으므로 시드 sweep 중간에 끼워 넣어도 sweep 이 동일하게 재생된다. writer 가 warm start · 적용력을 0 으로, 자세를 단위 quaternion 으로 되돌려 같은 숫자를 두 번 보내면 비트 단위로 같은 궤적이다. 필드 · 거부 집합의 SSoT 는 `rtc_msgs/srv/LaunchBall.srv` 주석이다.

## Clock phase lane (D-3 측정)

`/sim/status` 가 `sim_time` 을 싣지만 1 Hz wall 타이머라 위상 오차에 500 배 느리고 샘플링 클럭 자체가 틀렸다. 이 패키지에는 파일 writer 도 없었다. lane 은 완료된 step 마다 `(step, sim_time_sec, steady_ns, dropped_total)` 을 남긴다.

- 물리 스레드는 링에 넣기만 하고 파일 열기 · 포맷 · 쓰기는 노드 executor 가 한다.
- 링이 가득 차면 샘플을 버리되 세고, 누적값을 **매 행에** 적는다. 끝에 한 번만 적으면 언제 넘쳤는지 알 수 없는데 넘친 구간이 위상 오차가 가장 컸을 구간이다. `dropped_total > 0` 이면 분포의 꼬리를 신뢰할 수 없으므로 `drain_rate_hz` 를 올려 다시 잰다.
- `csv_path` 가 비면 거부한다. 측정 run 의 유일한 산출물을 아무도 지정하지 않은 경로에 쓰지 않는다.
- 이 lane 은 sim↔steady 대응표를 준다. "sim 축으로 보간" 에 실제로 필요한 것과 D-3 위상 오차의 측정이 그것이다.

## Ball contact truth (S3.3)

기존 contact lane 은 `mjSENS_CONTACT` 센서 기반이고 로봇 group 범위라 어느 group 에도 속하지 않는 공을 보지 못한다. 그래서 공이 닿아 있는 연속 substep 구간 하나를 episode 로 보고 CSV 한 줄을 남긴다.

- 충격량은 substep 루프 **안에서** 적산한다. `mjData::contact` 는 매 `mj_step` 에 다시 만들어지므로 루프 밖에서 읽으면 마지막 substep 만 보여 충격량이 조용히 `n_substeps` 배만큼 어긋난다 (튐 하나가 여러 substep 에 걸치는 일이 흔하다). 테스트가 실제 SimLoop 을 `n_substeps=4` 로 돌려 그 구현을 배제한다.
- 부호는 "공에 가해진 힘" 이다. `mj_contactForce` 는 쌍의 `geom[1]` 에 가해지는 힘을 주는데 공은 씬에 따라 어느 쪽에도 올 수 있다 (공은 spec 에 pool 물체보다 먼저 붙으므로 공↔pool 물체 접촉에서는 `geom[0]`). 뒤집지 않으면 절반의 경우 바닥으로 끌리는 충격량이 그럴듯한 숫자로 나온다.
- 열 정의: `step` (episode 가 끝난 step), `begin/end_sim_time_sec`, `substeps`, `impulse_{x,y,z}` [N·s, world], `peak_force_n` (동시 접촉력 합의 최대 크기), `pos_*` · `vel_*` (첫 접촉 순간 공의 world 위치 · 속도), `first_body`, `distinct_bodies`, `dropped_total`.
