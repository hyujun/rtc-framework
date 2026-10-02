# Testing & Debugging — sensor matrix · 레시피 · 계측 함정

> **이 문서는 헌법이 아니다.** 테스트·검증의 **규범** (무엇을 반드시 돌리고, 결과를 무엇으로 판정하는가) 은 [agent_docs/testing-debug.md](../agent_docs/testing-debug.md) 가 갖는다. 여기는 그 규범을 실행하는 데 필요한 것 — 패키지별 sensor 표, 명령, 측정 레시피, 함정이 실제로 발현한 사례, 런타임 디버그 토픽 — 이다. 둘이 어긋나면 agent_docs 쪽이 옳다. 패키지·테스트 이름과 수치는 기록 시점의 것이다.

## Sensor Matrix (변경 유형별 필수 검증)

변경 위치에 따라 필수 sensor + 추가 sensor 를 실행한다 ([AGENTS.md](../AGENTS.md) §5).

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` (`test_rt_heap` 은 `ConfigureRtHeap` 전용 바이너리 — mallopt 가 프로세스 전역이라 분리) | 전체 downstream ([invariants.md](../agent_docs/invariants.md) PROC-3) |
| `rtc_math/` | `colcon test --packages-select rtc_math` (Pinocchio 발견 시 `test_se3_module` log6/Jlog6 교차검증 + 유한차분) | `se3_error_compare` S1–S5 실험 + `plot_se3_compare.py` (선택) |
| `rtc_msgs/` | 위 + `./build.sh --tests full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | `colcon test --packages-select rtc_controller_interface` (registry·interface·mailbox·output-validation gtest) | downstream controller 빌드 |
| `rtc_controllers/` RT path | core 법칙 gtest (`test_*_core` + `test_dls_convergence` — ScopedAllocGate/ScopedNoMalloc 무장 TU) + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controllers/` catching core (`catching/`, dynamic_catching S1) | `colcon test --packages-select rtc_controllers --ctest-args -R test_catching` (7 스위트 — 게이트 ID 는 각 파일 헤더, 결과 기록은 `docs/dynamic_catching/IMPLEMENTATION_PLAN.md` §4.3) | ASan/UBSan: repo 에 전용 수단이 없어 **별도 build/install base** 로 `colcon build --packages-select rtc_controllers --build-base <scratch>/build --install-base <scratch>/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS" "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined"` (ws root, env source 후) → 바이너리 직접 실행. `ScopedAllocGate` TU 는 gate 의 new→malloc 교체가 ASan nothrow new 와 alloc-dealloc-mismatch 를 내므로 **그 TU 만** `ASAN_OPTIONS=alloc_dealloc_mismatch=0` (다른 검사는 유지). **Pinocchio·ProxSuite 를 쓰는 테스트** (rtc_tsid 등) 는 두 가지를 더한다: (1) `CMAKE_CXX_FLAGS` 에 `-DEIGEN_MALLOC_ALREADY_ALIGNED=1` — ASan 빌드에서 Eigen 은 `__SANITIZE_ADDRESS__` 를 보고 자체 aligned allocator 로 바꾸므로, prebuilt pinocchio 가 malloc 으로 잡은 Eigen 객체를 우리 TU 가 해제할 때 heap-buffer-overflow 가 난다 (코드 결함이 아니다). (2) **`-fsanitize-recover=bool,enum`** 을 더하고 `UBSAN_OPTIONS=print_stacktrace=1` 로 보고를 분류한다 — third-party UB 두 종이 기존 테스트에서도 나고, `-fno-sanitize-recover=undefined` 만으로는 **첫 발현에서 프로세스가 죽어 스위트가 한 줄도 안 돈다**: ProxSuite (header-only) `global_dual_residual` 이 초기화 안 된 bool active-set 을 읽는 것(`bool`), 그리고 Eigen 의 `LLT::info()`·`SelfAdjointEigenSolver::info()` 가 초기화 전 `ComputationInfo` 를 읽는 것(`enum`). `enum` 이 빠지면 Eigen 분해를 쓰는 스위트(`test_catch_pose_ik` 등)가 **첫 테스트에서 abort** 하므로 증상이 "테스트 실패" 가 아니라 "결과 없음" 이다. 판정은 "그 보고들 외 0 건" 이다 |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | (MPC CSV는 `<session>/controllers/<config_key>/...` 경로로 컨트롤러가 자체 기록) |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_timing_log.csv` p50/p99/max 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O, contact_wrench, known-load positive control, object_pool) + `test_motor_servo_gains` (`<motor>` actuator 위의 위치 서보. 다른 fixture 는 전부 `<position>` — 이미 affine — 이라 서보 lane 이 bias 타입을 바꾸는지 볼 수 없다. **`motor_arm.xml` 의 actuator 를 `<position>` 으로 바꾸면 이 테스트는 수정 전 코드에서도 통과한다** — 첫 케이스가 그 전제를 단언한다. `motor_arm_geared.xml` 은 gear ≠ 1 인 motor 와 ctrlrange 가 있는 motor 를 **하나씩** 둬서, 경고가 두 조건을 다 세는지를 개수로 본다) | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` smoke. Contact wrench: `ros2 topic hz /<prefix>/<target>/contact_wrench` 후 fingertip 으로 객체 접촉 → magnitude 가시화. Object pool: 출하 설정으로 켜져 있는 프로필은 `ur5e_p1b` 하나 (`ros2 launch integrated_bringup sim_ur5e_p1b.launch.py`) — 뷰어에서 `o` 를 눌러 status overlay 의 `Object` 행이 바뀌고 새 object 가 낙하하는지 본다 (`Object` 행이 안 보이면 pool 이 꺼진 것이고, 이름만 바뀌고 아무것도 안 떨어지면 park/unpark 가 깨진 것). 대조군은 `object_pool:=false` 이며 기동 로그의 `nq` 가 pool 없는 씬 값으로 돌아와야 한다 (p1b: 61 → 26). 초기 자세는 같은 로그의 그룹별 `initial positions from ...` 줄이 어느 소스를 썼는지 알려준다 |
| `rtc_urdf_bridge/` | gtest (URDF/model parsing, xacro, chain extractor) + `test_frame_jacobian_fd_oracle` (`GetFrameJacobian` 행/열 계약. 자코비안을 **전혀 쓰지 않는** 중심차분 oracle 이어야 하는 이유는 기존 `test_rt_model_handle` 이 유한성 + 자기일치뿐이라 행 블록을 맞바꿔도 green 이기 때문이다. 픽스처 `test/urdf/mixed_prismatic_revolute.urdf` 는 **네 비대칭 — 축 직교 · origin 3축 offset · tool rpy(LOCAL≠LWA) · prismatic 열의 zero angular — 을 유지해야** 대조가 판별력을 갖는다. 기하를 고치면 각 테스트의 positive control 이 green 아닌 red 로 알린다) + `test_inertial_validation` / `test_real_model_inertial_gate` (관성 물리 실현가능성 게이트 V5·V6. 레인은 심각도가 아니라 종류로 갈린다 — 질량·관성이 서로 모순이면(대표적으로 `mass==0` 인데 관성 ≠ 0) V6, 둘 다 0 이면 V5 다. 판정 대상은 **fixed joint 흡수 후의 composite `Model::inertias[]`** — M(q) 가 실제로 조립되는 값이라 게이트 의미가 "이 모델로 동역학을 돌려도 되는가" 로 떨어진다. tol 은 **주모멘트 크기로 정규화**해야 한다 — 절대 tol 은 실모델 주모멘트의 decade 폭을 양 끝에서 동시에 만족시키지 못한다. **등호는 허용**(얇은 판 I1+I2=I3). 임계값의 실측 근거는 `inertial_validation.hpp` 의 `kInertialRelTol` 주석이 SSoT 이므로 그 숫자를 여기 복제하지 않는다. `schunk_svh_hand_{left,right}` 는 통과 대상이 아니라 **실세계 negative fixture** 이며, 값을 지어내 고치지 않는 것이 #413 규율이다. 통과 목록은 **이 저장소가 소유한 모델만** 담는다 — `robots/ur5e_p1b/` 는 `hand_description` 이 소유한 모델의 사본 (테스트 입력) 이라 넣지 않는다 (#682). 비유한 값은 urdfdom 이 파싱에서 먼저 거부하므로 그 가드는 **프로그램적 모델 오염** 경로로만 도달 가능한 심층 방어다 — 현재 그 경로를 타는 production 소비자는 없다. URDF→Model 진입점은 `PinocchioModelBuilder` 와 `BuildClosedChainModelFromExtendedUrdf` 둘이고 **양쪽 다** 게이트를 지나므로, 새 진입점을 열면 `EnforceInertialGate` 호출을 함께 넣는다) | 실제 URDF 파싱 smoke |
| `rtc_inference/` | ONNX engine unit test | 실제 모델 로드 smoke |
| `rtc_communication/` | UDP loopback + CAN/CANFD loopback (vcan0 없으면 skip) + RS485 serial loopback (PTY, 상시 실행) + Transceiver lifecycle/decode/callback | vcan0 셋업 후 CAN 테스트 실행 (`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`), RS485는 PTY라 셋업 불필요, 실제 HW UDP/CAN/RS485(USB-RS485+Dynamixel) 테스트 (선택) |
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/rtc_cm/{group}/joint_states` hz |
| `rtc_tools/` (pytest) | pytest. **`robot_descriptions/` 의 MJCF/URDF 를 고쳤다면 `test_real_model_pairs.py` 가 실제 게이트다** — `robots/model_pairs.yaml` 의 쌍마다 `compare_mjcf_urdf` 를 발사한다 (#392). 로컬 colcon 은 pytest 를 `/usr/bin/python3` 로 돌려 mujoco 를 못 보므로 **skip 된다** — 실제로 돌리려면 `PYTHONPATH=rtc_tools:$PYTHONPATH .venv/bin/python -m pytest rtc_tools/test/test_real_model_pairs.py` | GUI/plot 수동 smoke |
| `repo_scripts/` | `test_rt_common` + `test_install_deps` (ONNX tarball digest 검증) + `test_install_python` (venv base · CMake Python = 배포판 python) + `test_build_jobs` (빌드 병렬도 — 패키지 1개씩 · make job 수 knob · 메모리 상한 scope (가용 메모리 기준 · swap 허용 · 쓸 수 없는 값 거부) 와 OOM 판정 (systemd 의 Result 와 cgroup `memory.events` 양쪽) · knob 을 `-c` 보다 먼저 검증 · 테스트 제외가 기본이고 `-DBUILD_TESTING` 을 매 빌드 명시 · ccache launcher · `install.sh` 의 ccache 설치와 호스트 요약; stub colcon / systemd-run 이 실제로 받은 인자·`MAKEFLAGS` 로 판정. **실제 cgroup 강제는 못 본다** — 그건 `RTC_BUILD_MEM_MAX=150M ./build.sh -p <pkg>` 로 직접 확인한다) + shell unit test. CI 는 `docs-validate.yml` 이 매 PR 돌린다 — `test/test_*.sh` · 도메인·fixture 검사기 · 추적되는 `*.sh` 전체 shellcheck (Stop hook 이 셸 파일을 통째로 검사해도 되는 전제). 도메인·fixture 검사기는 Stop hook 도 돌린다 (트리거·범위: hook 헤더 Phase 1d) — CI 에서만 돌 때는 다른 패키지와의 충돌이 PR 에서야 드러났다 (#513, #571) | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape_estimation/snapshot` topic echo |
| `integrated_bringup/` demo FSM | demo_wbc FSM/integration/output + demo_joint grasp·URDF paths + demo_task CLIK/contact_stop (URDF-backed iiwa7_leap fixture 공유, 노드 비생성) + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `integrated_bringup/` demo_joint 의 군 0 모델 (#635) | `test_joint_tree_primary_group` (군 0 이 tree 인 경우: 팔 끝 = 손 장착 link, 손끝 합성, E-STOP tick 의 팔 끝 = 정상 tick, TF slot 이름, 이름이 안 풀리거나 팔 끝을 정할 수 없으면 configure 거부 — 거부 케이스마다 같은 rig 의 통과 대조군이 붙는다. 같은 URDF 를 **사슬**로 선언한 rig 로 E-STOP tick 의 관절 순서 매핑과 이름 거부가 사슬 군에도 성립하는지 본다. lifecycle 케이스는 `PreConfigure` → device config → `on_configure` 순서로 올린다: `LoadConfig` 를 직접 부르고 `on_configure` 로 가면 base 가 config 를 한 번 더 읽어 arm handle 을 다시 만들고, device config 가 건 관절 순서 map 이 사라진다 (`TheConfiguredControllerStillMapsTheJointOrder` 가 그 상태를 잡는다). oracle 은 **전체 모델 FK 를 관절 이름으로** 계산한 것이라 reduced 모델 · frame id · 관절 순서 가정을 공유하지 않는다. fixture `rtc_urdf_bridge/test/urdf/dual_arm_tree_hand.urdf` 의 비대칭 — root ≠ world · 두 팔의 길이와 축 · 항등이 아닌 손 장착 · body device 순서 ≠ 모델 순서 — 이 각 단언의 판별력이므로 **고치면 `FixtureCanTellTheCasesApart` 가 알린다**. 군 0 을 고정하는 rig 의 손 device 순서는 모델 순서 그대로이고, 손 순서를 뒤섞은 rig (`ShuffledHandTreeRig`) 가 따로 있어 손끝이 device 순서를 따르는지를 node 없는 순서와 controller manager 의 순서에서 본다 (둘 다 `OnDeviceConfigsSet` 이 거는 자리다 — config 를 다시 읽는 순서는 `test_hand_fk_wiring` 이 본다) — #685) + `test_joint_tf_slot_frames` (기존 두 로봇 fixture 의 TF slot · payload frame 이름 고정 — 그 이름을 읽는 자리를 옮길 때의 회귀 기준) | `ros2 launch integrated_bringup sim_g1_p1b.launch.py enable_viewer:=false use_cpu_affinity:=false` 후 `/demo_joint_controller/{g1,p1b}/joint_goal` 에 목표를 보내고 `/demo_joint_controller/transforms` 의 `base_adapter_actual` · 손끝을 MuJoCo 의 body · site 위치와 비교 (폐쇄 체인 손과 tree 의 결합은 gtest 가 아니라 **여기서만** 본다 — G1 모델을 읽는 gtest 는 없다) |
| `integrated_bringup/` 손끝 FK 의 배선 (#685) | `test_hand_fk_wiring` — (A) `support/hand_fk_wiring` 단위: device 순서 ≠ 모델 순서인 손의 FK, 거부 (모델에 없는 이름 · 관절을 다 덮지 못하는 목록 · 같은 이름 두 번 · 팔 끝과 관절로 떨어진 손 root · 없는 손 root), 장착 변환 = 전체 모델 FK 의 상대 placement. 합성 fixture `dual_arm_tree_hand.urdf`. (B) joint · task · compliance 를 `iiwa7_leap` 에 올려 손끝 = **전체 모델 FK (이름순)** 1e-9, centroid virtual TCP = 그 손끝들의 중심, 네 컨트롤러의 configure 거부 (사유 문구까지 단언, 같은 rig 의 통과 대조군). wbc 의 손끝은 `test_demo_wbc_tsid_path` 의 `WbcHandFkWiring.*` 가 본다 — wbc 는 TSID 가 서야 pose 를 낸다. **두 bring-up 순서를 모두 돈다** (`PreConfigure` → device config → `on_configure`, 그리고 `LoadConfig` → device config → config 를 다시 읽는 `on_configure`): 손 handle 이 device config 보다 먼저 만들어지는지 뒤에 만들어지는지가 달라, 한 자리에서만 건 관절 순서는 한쪽에서만 남는다. **손 관절마다 다른 값을 준다** (`iiwa7_leap_hand_fk_oracle.hpp` 의 `kLeapQ`) — 손이 0 이거나 전 관절이 같은 값이면 순서가 뒤바뀌어도 FK 가 같다; `FixtureCanTellTheCasesApart` 가 device 순서 ≠ 모델 순서 · 장착 ≠ 항등 · 값 중복 없음을 확인한다 | sim: `iiwa7_leap` 에서 손 목표를 한 번 보내고 `/<ctrl>/transforms` 의 손끝을 같은 관절 값의 MuJoCo body 위치와 비교 (네 컨트롤러), `ur5e_p1a` 는 pinocchio 이름순 FK 와 비교 |
| `integrated_bringup/` 팔 끝 frame 의 configure 거부 (#688) | `test_arm_tip_resolution` — 판정 함수의 사유 문구 (이름이 모델에 없음 / link 가 주어지지 않음 + 고칠 자리), 그리고 joint · task · compliance · wbc 를 `iiwa7_leap` 에 올려: 모델에 없는 `tip_link` → FAILURE, link 없는 device config → FAILURE, 같은 rig 의 맞는 이름 → SUCCESS, 모델 없는 컨트롤러 → SUCCESS (task · compliance 는 이 경로에서 null handle 을 역참조했다). 거부는 세 겹으로 고정한다 — 같은 rig 의 통과 대조군, 컨트롤러 자신의 판정이 그 원인을 말하는지, 읽을 수 있는 다른 configure 오류가 비어 있는지 (다른 이유로 실패한 `on_configure` 도 FAILURE 다). 두 bring-up 순서를 돈다: 판정은 `on_configure` 가 그 자리의 상태로 하므로, config 를 다시 읽을 때 frame id 를 지우는 변경은 한쪽 순서의 통과 대조군에서 red 가 된다. **wbc 는 `tsid:` 없는 YAML 로 올린다** — TSID 가 서 있으면 CLIK 초기화 실패가 같은 구성을 먼저 거부해 이 검사를 빼도 green 이다 | — |
| `integrated_bringup/` compliance 바인딩 (#469) | `test_compliance_admittance_coupling` (§7 법칙의 **배선**: 발행된 렌치 → 파이프라인 → α → 적분기 → X_c → 팔. 코어 ODE 는 rtc_controllers 소관이라 재검증하지 않고, 두 반쪽에 각각 정확한 oracle 을 붙인다 — 적분기 반쪽은 로컬 `AdmittanceIntegrator` 와 **bit-identical**, 파이프라인 반쪽은 부호·축·레버암 전달) + `test_compliance_task_equivalence` (**렌치를 withhold 하는 동안에만** demo_task 와 bit-identical — 그 전제 자체를 tick 마다 단언한다)  + `ComplianceJointTail.*` / `TaskControllerJointTail.*` (compliance §7.3 관절 tail: 명령이 밴드 안에 남는가 **그리고** 그 사실이 보고되는가 — 각각 출하 밴드 대조군과 쌍으로, clamp 가 스텝을 *넓히는* 경우까지 포함해 순서를 바인딩에서 고정)| 4개 per-controller 스위트의 `demo_compliance_controller` 항목 (device-readability · gate-closure · activation-generation · MO-embedding) + **sim 런타임** — `ur5e_p1b` 에서 외부 렌치가 실제로 들어오는 것이 2026-09-12 에 확인됐다 (`compliance_diag.csv` 의 `wrench_*`·`alpha`·`x_tilde_*`). 파지를 세우는 절차와 그 제약은 verify skill 이 소유한다 |
| `integrated_bringup/` catching 바인딩 (dynamic_catching S4.0) | `test_demo_catching_controller` (팔 latch 가 측정이 움직여도 bit-불변 · 손 계단이 `clamp(target)` 과 **bit-equal** 로 나가는가 (G6-B 의 바인딩 반쪽; CM 반쪽은 `test_rt_loop_pipeline` 이 이미 고정) · 비-`mujoco_native` backend·`hand_step: false`·팔 target 거부 · 비활성 중 받은 target 의 세대 게이트 · 출하 프로파일 2벌이 손 device 폭과 YAML 한계 안인가) + `test_demo_catching_alloc` (`Compute` 할당 0, positive control 포함 — `operator new` 치환이라 별도 바이너리) | **4개 per-controller 스위트 중 2개만 편입한다**: `activation-generation` 은 자기 케이스가 있고 `registered-controllers-have-shipped-config` 는 레지스트리 순회라 자동이다. `device-readability` 는 계약의 절반이 E-STOP safe-position 램프인데 이 컨트롤러는 E-STOP 훅을 **하나도 override 하지 않는다** (그것이 E-8 결정 자체다) — 적용되는 부분(unreadable 축 침묵·parked 텔레메트리·wide device 무해)은 자기 스위트에 있다. `MO-embedding` 은 팔 동역학도 observer 도 없어 해당 없음. 둘 다 S5.1 에서 법칙이 붙을 때 재판정한다 |
| `integrated_bringup/` inference 바인딩 (`demo_inference_controller`) | `test_demo_inference_controller` (roster fixture — 스키마·decimation·hold/latch·recurrent·object lane + 이름 배치·fill·constant·관절 규약·named head gather·seed·**직전 명령 기준 tail**) + `test_demo_inference_urdf` (실 ur5e_p1b + **출하 YAML** 을 그대로 로드: MuJoCo oracle 대비 palm ±1 mm · fingertip ±2 mm(closed-chain) · 규약 부호 · **정책 프레임** (출하 = URDF `base`; oracle 은 MJCF `base` = URDF `base_link` 에서 측정한 값을 그대로 두고 비교 지점에서 `fx::BaseFromBaseLink` 로 반바퀴를 건넌다 — 부호를 뒤집어 재타이핑하면 그 숫자의 출처가 사라진다) · 물체 lane 의 회전은 **source_frame_link 만 옮긴 fixture** 로 고정 (출하 config 는 source = policy frame 이라 회전이 identity 다) · reach gate 배선(policy step 계수·엄지 우선) · `inference_diag` 행이 정책이 본 것과 일치하는지 · 폐쇄 체인 stall 경고 (평범한 walk-in 은 침묵, 임계를 낮춘 fixture 에서 활성화당 정확히 1회) · 출하 키 presence 와 값을 따로) + `test_demo_inference_alloc` (operator-new gate. 출하 경로 — link FK·closed chain·reach gate·seed 리셋 — 포함, **Eigen 의 malloc 은 못 본다**) | `test_demo_inference_real_model` — **로컬 전용**: `RTC_POLICY_DIR` 이 없으면 전 케이스 skip (= 미검증; CI 는 항상 skip). 실 ORT 로 출하 YAML configure (이름·shape, 입력 이름 1개 오타 → 엔진 대조표와 함께 FAILURE) · pregrasp 에서 첫 실 액션이 측정 자세에서 튀지 않음 · `Run()`/정책 tick 지연과 operator-new 수를 XML `RecordProperty` 로 기록 (**ORT `Run()` 은 매 호출 할당한다 — 이 수는 하한**) · ORT `Run()` RT-1 수용 조건 중 둘을 **단언**: 정상상태 heap 무성장 (`mallinfo2`, `ConfigureRtHeap` 적용) · 정책 tick 할당 == `Run()` 할당 (컨트롤러 몫 0) — [invariants.md](../agent_docs/invariants.md) RT 절 · `RTC_INFERENCE_DUMP_DIR` 을 주면 입출력 텐서를 raw 로 덤프 (정책 파생물이라 repo 안 경로는 거부) → 오프라인 ORT python 대조. 실행: `( cd <rtc_ws> && source <repo>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && RTC_POLICY_DIR=<dir> colcon test --packages-select integrated_bringup --ctest-args -R real_model )`. + sim 런타임 (verify skill) |
| `integrated_bringup/` momentum observer (#135) | `test_momentum_observer_wiring` (좌표 계약 + lane 게이트, 실제 ur5e sub-model) + `test_momentum_observer_embedding` (네 컨트롤러 × `momentum_observer.csv` 왕복 — 이 layer 의 잔차는 **CSV 말고 관측면이 없다**: 소비자는 Layer 2A 이고 토픽은 D12 로 없다. 배선만 검사하면 push·등록 경로가 통째로 안 돌고, unbound handle 에 대고 쓴 행 수 단언은 push 를 지워도 통과한다) | sim negative control — `momentum_observer.csv` 의 무부하 정지 `residual_inf_norm`. 읽는 수단은 `ros2 run rtc_tools plot_rtc_log <그 CSV> --stats` 이며 **‖r‖∞ 통계는 `valid=1` 행만** 쓴다 (held 행은 직전 잔차가 동결된 값이라 측정이 아니다). 재시딩이 있었으면 그 직후 구간은 0 에서 수렴 중이라 작은 ‖r‖ 이 아직 무부하가 아니다 — 통계가 `Re-seeds:` 줄로 경고한다 |
| `rtc_controllers/` payload estimator (#135 Layer 2A) | `test_payload_estimator` (코어 — oracle 을 **정방향으로** 조립한다: 원하는 질량/CoM/중력축에서 wrench 를 만들고 `r = Jᵀw` 로 내린 뒤 역산시킨다. 역방향으로 쓰면 부호 뒤집힘에도 green 이라 #135 가 명시적으로 요구한 AC1 이 무효가 된다) + `test_momentum_observer_wiring` 의 `PayloadEstimatorWiring.*` (배선 — **device order 를 뒤집은** 채 알려진 질량을 매단다. `GetFrameJacobian` 은 pinocchio order, `residual()` 은 device order 라 두 순서가 같은 fixture 에서는 뒤섞어도 통과한다) | 무부하 sim 에서 `payload_mass ≈ 0` + `payload_reason` 분포 (게이트가 무엇 때문에 닫히는지) |
| `udp_hand_driver/` | 단위 gtest (hand_packets, codec, FT, failure detector) + UDP loopback | `ros2 topic hz /p1a/joint_states` (ur5e_p1a; 드라이버 standalone 기본은 `/hand/`) |
| `ur5e_bt_coordinator/` | BT gtest — Tier-2 는 inject(DDS-free, `inject_fixture.hpp`)/e2e(real-DDS, `test_helpers.hpp`) 분리, suite 목록은 CMakeLists `TIER2_INJECT`/`TIER2_E2E` (#154) | 실제 grasp 시나리오 smoke |
| Launch (`integrated_bringup/launch/*.py`) | `colcon test --packages-select integrated_bringup --ctest-args -R 'test_launch_'` — 평가 센서 `test_launch_description_evaluates` (5개 launch × 4 인자 조합을 `LaunchContext` 만으로 실제 평가; ROS 그래프 불요, ~1.4 s) + AST 센서 `test_launch_shield_wiring` · `test_launch_hand_affinity_wiring` (어느 헬퍼를 부르는가). **import 스모크(`rtc_tools/test/test_launch_imports.py`)로는 `OpaqueFunction` 본문이 안 돈다** — #397 이 그 틈으로 sim 3개를 죽인 채 전 배터리 green 이었다 | 짧은 실기/sim smoke (`ros2 launch ...`) |
| Launch / YAML config | 위 + 변경 YAML parse | config 로드 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT perms | `check_rt_setup.sh --summary` |
| RT 회귀 의심 / RT path 미상 | `mpc_timing_log.csv`·`cm_timing_log.csv` p99 검사 | `enable_tracing:=true` + Perfetto 분석 (§Tracing) |
| RT host 환경 검증 | `check_rt_setup.sh --summary` + `verify_rt_runtime.sh` | `cyclictest --mlockall --smp -p 80 -i 200` / `rtla osnoise top` — [invariants.md](../agent_docs/invariants.md) §RT Host / Runtime Preconditions RT-HOST-1~3 |

## Test Commands

```bash
# 사전: 테스트는 기본으로 빌드되지 않는다 — 먼저 `./build.sh --tests [-p <pkg>]`.
# 테스트 없이 빌드한 패키지는 실패가 아니라 "0 tests" 를 보고한다.

# All tests
colcon test --event-handlers console_direct+
colcon test-result --verbose

# Single package
colcon test --packages-select ur5e_bt_coordinator --event-handlers console_direct+

# Single test (C++)
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller

# Single test (Python)
colcon test --packages-select rtc_digital_twin --pytest-args -k test_urdf_parser
```

## Revert-verification — 새 가드를 추가했을 때

가드(검증·거부 경로)와 그 테스트를 함께 추가하면 **테스트 통과는 가드가 동작한다는 증거가 아니다** — 그 가드를 지워도 통과할 수 있다. 추가한 가드마다 **하나씩 원복 → 대응 테스트가 실제로 실패하는지 확인 → 복구**. [AGENTS.md](../AGENTS.md) §5.5 의 "에이전트 자기 평가는 신뢰 불가" 가 자기가 쓴 테스트에도 그대로 적용된다.

**가장 흔한 false green — 층이 겹치는 가드.** 새 가드 A·B 가 같은 잘못된 입력을 모두 거부하면 B 의 테스트는 A 에 흡수되어, B 를 통째로 지워도 통과한다. B 는 커버리지가 있는 것처럼 보이면서 실제로는 자유롭게 삭제 가능한 상태다. 이때는 테스트를 "throw 하는가" 가 아니라 **그 가드만이 만드는 관측 가능한 차이** (진단 메시지 내용, 거부 시점, 부작용 유무) 로 옮겨야 pin 이 성립한다.

> 실측 (#204): 신규 가드 4개 중 flat `publish:` 탐지가 false green 이었다 — 원복해도 전 테스트 통과. group-shape 가드가 같은 config 을 이미 거부하고 있었다. 테스트를 "마이그레이션 진단을 주는가" 로 옮겨 pin 을 성립시켰다. 나머지 3개는 각각 자기 테스트만 정확히 실패. 거부 시점을 pin 한 예도 같은 PR 에 있다 — backend `Configure()` 카운터로 "controller 는 생성됐지만 device wiring 전" 경계를 고정.
>
> **같은 false green 이 형제 절반에서 재발했다 (#204 post-review).** 위 수정은 flat 탐지의 `publish` 쪽만 pin 했고, `subscribe` 쪽은 `EXPECT_THROW` 로 남아 똑같이 원복해도 통과했다. 층이 겹치는 가드를 하나 고쳤으면 **대칭 위치의 나머지 절반도 같은 기준으로 다시 측정**한다 — 한쪽을 진단 pin 으로 옮겼다는 사실 자체가 다른 쪽도 흡수되고 있다는 신호다.

**두 번째 false green — 관측 채널이 fallback 에 가려질 때.** 가드를 원복(mutation-check)해도, 테스트가 assert 하는 값이 다른 경로로도 같은 값을 내면 vacuous 다 — 이때는 가드가 아니라 *관측 지점*이 잘못된 것이다. E-STOP recovery drain(#242)이 그 예: OSC/TaskImpedance 는 gravity-comp 컨트롤러라 정지(q̇=0) 시 hold torque ≈ ĝ(q) 인데, E-STOP 중 큐잉돼 leak 된 target 이 충분히 멀면 recovery tick 에서 SAFE_STOP 을 latch 시키고 그 hold 역시 ĝ(q) 를 내므로 **joint torque 로 assert 하면 leak 유무와 무관하게 통과**한다. 관측을 fallback 이 건드리지 않는 채널로 옮겨야 pin 이 성립한다 — OSC 는 goal echo(`task_goal_positions`, drain 이 쓰는 slot 을 직접 반영), TaskImpedance 는 `diag.pose_error`(SAFE_STOP step *이전에* 기록되고 `ComputeEstop` handoff 로 보존). 실측(#242): torque assertion 으로 두 번 vacuous 를 거친 뒤 채널을 옮겨 pin.

**세 번째 false green — 게이트가 닫힌 쪽에서 *수치적으로 inert* 할 때.** "게이트를 닫은 케이스를 넣었다" 는 것만으로는 그 게이트가 고정되지 않는다. 게이트를 지웠을 때 실행되는 경로가 **닫힌 것과 같은 값**을 내면 출력 대조는 원복해도 통과한다. 실측 (#236 S5): `nullspace_active = (nv > 6) && (nullspace_kp != 0.0)` 에서 `&& nullspace_kp != 0.0` 을 지워도 비트 레인 **전부 green** — `nullspace_kp = 0` 이면 게이트 없는 경로가 `0.0·Δq` 의 signed zero 를 만들고, 그것을 사영해 더하는 것은 exact no-op 이기 때문이다. 게이트 자체를 관측하는 출력 (여기서는 `diag.nullspace_active`) 을 tick 마다 대조하도록 옮겨야 pin 이 성립한다. **곱셈으로 꺼지는 게인·플래그 (`k = 0`, `α = 0`) 는 전부 이 형태**이므로, 그런 게이트는 출력이 아니라 진단 플래그로 pin 한다 — 그리고 그 플래그는 `EXPECT_FALSE` 만으로는 배선이 고정되지 않으니 양성 케이스를 함께 둔다.

원복은 **파일 단위 restore** 로 되돌린다 — `git checkout -- .` 은 아직 커밋하지 않은 작업까지 함께 날린다 (#204 에서 실제 발생).

단 **검증 대상 파일 자체가 미커밋일 때** (가드를 방금 썼고 아직 커밋 전 — revert-verification 의 표준 상황) 는 `git checkout -- <file>` 도 그 작업을 날린다. 명시적 백업 사본에서 복구해야 하는데, 여기에 함정이 하나 더 있다:

> **mtime 을 보존하는 복사로 복구하면 (`cp -p`, `shutil.copy2`) make 가 재컴파일을 건너뛴다.** 복구된 소스는 최신인데 빌드 트리에는 **원복된 바이너리**가 남아, 그 상태로 테스트하면 결과를 반대로 읽는다 (#204 post-review 에서 실제 발생 — 복구 후 "2 failures" 를 보고 회귀로 오독). 초록으로 오독되는 방향도 똑같이 가능하다. 백업 복구 후에는 반드시 `touch <파일>` 하고 재빌드한다.

```bash
# 사전: 미커밋 작업이 있으면 명시적 백업
cp <파일> /tmp/<파일>.orig          # -p 금지 (mtime 보존 → 아래 touch 를 잊으면 stale 바이너리)

# 가드 1개 원복 → 빌드 → 해당 실행파일만 → 복구
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select <pkg> --ctest-args -R <test_exe>
cp /tmp/<파일>.orig <파일> && touch <파일>   # 미커밋 작업이 없으면 git checkout -- <파일>
```

빌드가 실제로 돌았는지는 `colcon build` 의 소요 시간으로 확인된다 — 0.x 초로 끝났으면 재컴파일이 안 된 것이다.

## Test fixtures — robot URDF 해석

robot 모델이 필요한 gtest fixture 는 URDF 를 **`robot_descriptions/robots/<name>/`** (repo 체크인) 에서 해석한다. **`deps/src/...` 나 `/usr/local/...` 경로를 박지 말 것** — deps 격리 정책상 clean 환경에는 `deps/src` 도 `/usr/local` 설치도 없을 수 있어, 두 경로 모두 fixture `buildModel` throw 또는 `GTEST_SKIP` 을 유발한다 (당시 CI 에서 0 coverage 로 발현 (rtc_tsid/rtc_mpc/integrated_bringup 에서 실제 발현, panda.urdf vendor 로 해소). 해석 방식: compile macro (`RTC_PANDA_URDF_PATH`, CMake 가 repo-상대 `robot_descriptions` 경로 주입, env/`-D` override) 또는 `ament_index_cpp::get_package_share_directory("robot_descriptions")`.

> **repo 밖 패키지를 resolve 하는 fixture 는 clean 환경에서 전부 red 다 — 그리고 그게 안 보인다.** (아래 사례의 "CI" 는 2026-09-19 에 제거된 C++ CI 다 — 지금은 그 환경을 재현할 CI 가 없어 이 절의 센서가 유일한 방어선이다.) `ur5e_p1b_test_fixture.hpp` 가 `get_package_share_directory("hand_description")` 로 URDF 를 찾던 시절, 그 패키지는 이 repo 에도 `deps.repos` 에도 없어서 (로컬 ws 에만 있었다) 그 픽스처를 쓰는 gtest 는 CI 에서 model-backed 케이스가 전부 `package 'hand_description' not found` 로 던졌다. **`integrated_bringup` 은 `test_cpp_besteffort` (continue-on-error) 라 PR 은 green 으로 남는다** — 즉 이 위반의 유일한 증상은 codecov patch % 뿐이고, 그것도 "테스트를 안 썼다" 로 오독된다 (#452 에서 실제 발현: `test_momentum_observer_wiring` 14개 중 13개 red, joint-order oracle 이 한 번도 CI 에서 안 돌았음). **#457 이 그 모델을 `robot_descriptions/robots/ur5e_p1b/` 로 vendor 해 닫았고, 지금 test 소스가 여는 패키지는 전부 in-repo 다** (`validate_test_fixtures.py --list` 가 전수). 새 fixture 도 **in-repo 패키지만** resolve 해야 하고, 이미 있는 fixture 를 재사용할 때는 그 fixture 가 무엇을 여는지부터 본다. 검증은 `AMENT_PREFIX_PATH` 에서 그 prefix 를 빼고 바이너리를 돌리는 것 — 로컬 통과는 증거가 아니다. **격리가 실제로 걸렸다는 positive control 은 그 셸에서 `ros2 pkg prefix <pkg>` 가 실패하는 것**이다 (예전에는 `test_wbc_closed_chain_projection_sharing` 이 던지는 것을 대조군으로 썼으나, #457 이후 그 테스트는 격리에서도 통과한다 — 던지는 테스트를 대조군으로 삼으면 그 결함이 고쳐질 때 대조군이 조용히 사라진다).

> **규칙은 있었고 센서가 없었다 — 이제 있다 (#454).** `repo_scripts/scripts/validate_test_fixtures.py` 가 test 소스의 package resolution 을 훑어 **repo 에도 `deps.repos` 에도 없는 패키지**를 열면 차단한다 (`repo_scripts` 의 `test_fixture_package_resolution`; `--list` 로 현재 판정 전체를 본다). 리터럴 스캔이 아니다 — #457 이전 5개 파라미터화 fixture 는 `get_package_share_directory(ec.urdf_pkg)` 처럼 **struct 필드 경유**로 열어서, 리터럴만 보면 8곳 중 2곳만 잡혔다. 따라서 **비-리터럴 인자도 unresolvable 로 친다**, 그리고 fixture 가 CMake source list 에 없는 헤더에 살기 때문에 **include 를 따라가** 위반을 그 헤더에 도달하는 모든 test `.cpp` 에 부과한다. 그 규칙이 남아 있으므로 지금 그 fixture 들은 패키지를 **리터럴로** 적는다 — `ok` 판정은 경로가 어디로 가는지를 말해주지만 `guarded` 는 "틀려도 테스트가 죽지 않는다" 만 말해준다.

> **그 게이트가 강제한 skip 은 #457 로 전부 사라졌다.** skip 은 공백을 메우는 게 아니라 보이게 하는 것이었고, 공백 자체는 `robot_descriptions/robots/ur5e_p1b/` (UR5e + proto_1b, 5-loop, `n_a=16`) 를 vendor 해 닫혔다 — 그래서 `--list` 는 이제 `guarded` 없이 `ok` 만 낸다. 게이트의 발화 증명은 그만큼 **코퍼스에 의존할 수 없게 됐고**, `--self-test` 안의 합성 코퍼스(ok / guarded / bare / include 경유 4케이스)로 옮겼다 — 코퍼스를 positive control 로 쓰면 "위반 하나를 영원히 살려둬라" 가 되기 때문이다. 다시 repo 밖 모델이 정말 불가피하면 게이트가 남은 요구를 실패 메시지로 알려준다 (`GTEST_SKIP` + `PackageNotFoundError` 만 좁게 catch).

> **install set ≠ local — cross-package ament lookup 은 stub/mock 필수.** (제거된 CI 의 사례) **Python Test** 잡은 `rtc_tools` / `rtc_msgs` / `rtc_digital_twin` 만 install/source 한다. `rtc_tools` 테스트가 **다른 패키지**를 `ament_index` 로 resolve 하면 (예: `get_package_share_directory("repo_scripts")` — 런치 pinning 의 `rt_common_path()` / `cpu_shield_path()`) CI 에서만 `PackageNotFoundError` 로 실패한다. **로컬은 전 패키지가 install 돼 있어 통과하므로 이 회귀는 로컬 sensor 로 안 잡힌다** (#151 에서 실제 발현). 해석 경로 자체가 아니라 렌더된 산출물(bash snippet 등)만 검증하는 테스트는 경로 헬퍼를 monkeypatch 로 stub 한다 — production 런치는 전 패키지 install 상태라 무해. (동일 축의 C++ 판이 바로 위 URDF fixture 함정.)

## 비동기 결과를 기다리는 법 — sleep 대기와 executor pump 는 다른 원시다

고정 sleep 대신 **관측 가능한 진행** (tick / solve / recv 카운터) 을 폴링한다. 공유 헬퍼는 `rtc::testing::WaitUntil` (`rtc_base/test/include/rtc_base/testing/wait_until.hpp`, `rtc_base/test/test_wait_until.cpp` 가 계약을 pin) 이며, 설치되지 않으므로 소비자는 `../rtc_base/test/include` **소스 트리 경로**로 가져온다 (`no_malloc_scope.hpp` 와 동일 레이아웃; ament symlink install 이 `install(PATTERN EXCLUDE)` 를 무시해 `include/` 에 두면 런타임 트리로 실려 나간다).

이 헬퍼는 **오직 잔다**. Executor 를 pump 해야 하는 테스트는 자기 TU 에 local spin 헬퍼를 두며, 둘을 섞지 않는 것이 [invariants.md](../agent_docs/invariants.md) **PROC-8** 이다 (규칙·탐지는 그쪽, 근거와 양쪽 실패 모드는 [invariants-rationale.md](reference/invariants-rationale.md) §Process 의 PROC-8 행).

Suite 고유의 poll 예산이 있으면 헬퍼를 감싸지 말고 **인자로 넘긴다** — `WaitUntil(pred, timeout, poll)`. 예산이 assertion 옆에 보이는 편이 낫고, 같은 이름의 wrapper 는 `using namespace rtc::testing;` 이 들어오는 순간 모호해진다. 호출 지점이 많아 예산을 한 곳에 묶어야 한다면 **다른 이름**으로 얇게 감싸고 그 값의 근거를 상수 옆에 남긴다 (`udp_hand_driver` 의 `PollUntil` = CommLoop 한 tick).

## 컨트롤러 CSV 채널을 검정할 때 — 관측 창은 512행이고, 넘치면 값 불일치로 보인다

`ControllerLogSet::RegisterLog` 의 SPSC ring 은 기본 **512 entry** (`rtc_controller_interface/include/rtc_controller_interface/controller_log_set.hpp`, `Capacity = 512`). production 에서는 컨트롤러의 100 ms drain 타이머가 계속 비우지만, **`Compute()` 를 루프로 도는 gtest 에는 그 타이머가 없다** — 512 tick 을 넘기는 프로그램은 tail 을 잃는다.

**증상이 "행이 없다" 가 아니라는 점이 이 함정을 비싸게 만든다.** 파일은 멀쩡히 512행이 있고, 테스트가 "마지막 행" 이라고 읽은 것은 링이 가득 찬 tick 의 행이다. 그래서 마지막 행을 프로브와 대조하는 단언이 **값이 어긋난 것처럼** 실패한다 — #469 S4 에서 1000 tick 프로그램의 `wrench_fx`·`x_tilde_x`·`task_origin_x` 가 한꺼번에 어긋났고, 코드에는 결함이 없었다. 같은 파일의 480 tick 짜리 케이스는 통과하고 있어서 "행 수는 맞는데 값만 틀리다" 로 읽혔다.

처방 둘, 둘 다 필요하다:

- **production 처럼 주기적으로 drain 한다** — 드라이버 헬퍼가 N tick 마다 `log_set.DrainAll()` 을 부르게 한다. 끝에서 한 번만 부르는 것은 512행짜리 관측 창을 쓰겠다는 뜻이다.
- **행 수 단언 옆에 `EXPECT_EQ(log_set.TotalDropCount(), 0U)` 를 둔다** — 이것이 "잘렸다" 와 "push 가 애초에 안 됐다" 를 가르는 유일한 신호다. 없으면 나중에 Capacity 가 바뀌거나 프로그램이 길어질 때 같은 실패가 다시 값 불일치로 위장한다. `DrainControllerLogs` 는 drop 을 WARN 으로 알리지만 **테스트가 직접 부르는 `DrainAll()` 은 아무 말도 하지 않는다.**

## RT-1 zero-allocation 게이트 — 세 종류이고 서로의 맹점을 덮는다

RT tick 이 heap 을 안 만진다는 주장(RT-1)을 재는 sensor 는 **세 개**이고, 어느 것을 쓸지는 취향이 아니라 두 질문으로 결정된다: **(a) 측정 대상이 Eigen 을 쓰는가, (b) 측정 대상 코드가 테스트와 같은 TU 에 인스턴스화되는가.**

| 게이트 | 보는 것 | 못 보는 것 |
|---|---|---|
| `rtc::testing::ScopedAllocGate` (`rtc_controllers/test/include/rtc_controllers/testing/alloc_gate.hpp`) — 전역 `operator new` 교체 | `operator new` 를 타는 모든 할당 (`std::vector`, header-inline helper, 다른 TU 포함) | **Eigen 할당 전부** — `internal::aligned_malloc` 이 `std::malloc` 을 직접 부르고 `operator new` 를 타지 않는다. **그리고 C 라이브러리 `malloc` 전부** — `libddsc` 의 `ddsrt_malloc` 처럼 C 코드가 부르는 할당은 `operator new` 를 안 타므로 이 게이트에 안 잡힌다 (#222 가 이 구멍에서 tick 당 156 B 를 찾았다; 재려면 그 이슈 부록의 `LD_PRELOAD` interposer) |
| `rtc::testing::ScopedNoMalloc` (`rtc_base/test/include/rtc_base/testing/no_malloc_scope.hpp`) — `eigen_assert` + `EIGEN_RUNTIME_NO_MALLOC` | Eigen 동적 할당 (Release 에서도 유효) | non-Eigen heap; **그리고 다른 TU 의 Eigen** — 이 매크로는 정의된 TU 안의 Eigen inline 만 계측한다 |
| `rtc::testing::ScopedMallocGate` (`rtc_controllers/test/include/rtc_controllers/testing/malloc_gate.hpp`) — 실행 파일이 `malloc`·`calloc`·`realloc`·aligned 계열을 **정의**해 glibc `__libc_*` 로 넘기며 센다 | C 수준 할당 전부 — **공유 라이브러리 안 포함** (동적 링커가 실행 파일의 정의를 먼저 잡는다). Pinocchio `.so`, `rtc_tsid` 의 ProxQP, 다른 TU 의 Eigen, `operator new` (malloc 을 거치므로) | glibc 가 아닌 C 라이브러리 (링크 실패로 드러난다); `free` 는 세지 않는다 |

따라서:

- **순수 Eigen 코어 (header-inline 법칙)** → **둘 다** 무장한다. 하나만 쓰면 가장 유력한 RT-1 회귀 (인자·임시가 `Eigen::VectorXd` 같은 runtime-sized 로 퇴화) 가 green 으로 통과한다. `test_task_accel_core` / `test_task_vel_core` 가 이 형태.
- **Eigen-free 코어** → `ScopedAllocGate` 만. Eigen 트립와이어는 여기서 진짜로 vacuous 하다 (`test_joint_pd_core`).
- **컨트롤러 `Compute()` 처럼 라이브러리 TU 에 컴파일된 코드** → `ScopedAllocGate` 만. Eigen 트립와이어는 관측 대상이 다른 TU 라 vacuous 하다 (integrated_bringup 의 `test_task_dls_convergence` — 바인딩 `compute.cpp` 경로가 이 형태).
- **Pinocchio · ProxQP 같은 라이브러리를 부르는 코드** → `ScopedAllocGate` + `ScopedMallocGate`. 앞의 둘만으로는 라이브러리 안의 Eigen/C 할당이 **보이지 않는다** — `test_catching_decel_mpc` 가 이 게이트로 wrapper 경유 ProxQP `update()`·`solve()` 의 호출당 할당을 처음 잡았다 (operator-new 게이트는 같은 구간에서 0). positive control 은 **라이브러리 안** 할당 (`pinocchio::Data` 생성) 이어야 한다 — 테스트 TU 의 할당은 operator-new 게이트가 이미 증명하는 것만 증명한다. 교체 `operator new` 와 한 TU 에 둘 수 있다.

**게이트는 반드시 RAII 로 무장한다** — `g_alloc_active = true; … = false;` 같은 맨 대입은 측정 구역에 `ASSERT_*` 가 들어오는 순간 disarm 이 실행되지 않고, 읽는 곳이 없으므로 이후 모든 테스트가 계수되는 상태가 조용히 남는다.

**추가한 게이트는 mutation 으로 fail-closed 를 확인한다** — 측정 구역에 `std::vector<double>`(→ operator-new 게이트만 발동) 과 runtime-sized `Eigen::VectorXd`(→ Eigen 게이트만 발동) 를 각각 넣어 본다. `new` + 즉시 `delete` 쌍은 컴파일러가 elide 하므로 mutation 이 성립하지 않는데, **외부 sink 로 포인터를 흘리는 것만으로는 부족하다** — gcc `-O3` 는 `malloc(상수)`+`memset`+`free` 를 volatile 전역에 포인터를 저장해도 통째로 지웠다 (#222 의 계측기 self-test 가 그래서 "할당 0" 을 보고했다). 살리려면 **크기도 volatile 로 두고 `asm volatile("" ::: "memory")`** 를 건다. 확인은 `nm -D <binary> | grep -w malloc` — 참조가 0이면 재려던 호출이 바이너리에 없다.

`alloc_gate.hpp` 는 교체 `operator new` 를 **정의**하므로 바이너리당 정확히 한 TU 에서만 include 한다 (두 번째 TU 는 링크 에러 — fail-closed). 해당 타깃에는 `-Wno-mismatched-new-delete` 가 필요하다 (`new`→`malloc` / `delete`→`free` 짝에 GCC 가 program-wide false-positive).

## Test 측정

테스트 카운트·suite 목록은 박제하지 않는다 ([invariants.md](../agent_docs/invariants.md) AP-DOC-1). 최신 카운트·suite 명은 직접 측정:

```bash
rm -rf build/<pkg>/test_results                 # ← 누적 XML 제거 (아래 함정)
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

**계측 함정 — 비교 가능한 수치를 낼 때 필수:**

- **`colcon test-result` 는 디렉토리에 남아 있는 XML 을 전부 합산한다.** CMakeLists 에서 타깃을 빼거나 브랜치를 되돌린 뒤 재측정하면 **사라진 타깃의 옛 결과가 그대로 더해진다** — 숫자가 그럴듯해서 자체 검산 없이는 안 걸린다 (#236 슬라이스 3 에서 baseline 을 298 대신 300 으로 오측하고 존재하지 않는 drift 원인까지 보고했다).
- **같은 이유로 `colcon test` 가 돌지 않아도 `colcon test-result` 는 답한다.** `lint && colcon build && colcon test; colcon test-result` 처럼 체인의 앞이 실패하면 테스트는 실행되지 않고, 뒤의 `test-result` 는 **이전 실행의 결과**를 요약한다 (#647: 관련 없는 파일의 `ruff check` 실패가 빌드·테스트를 건너뛰었는데 `1179 tests, 0 failures` 가 찍혔다). 판정 전에 같은 출력에 `colcon test` 의 `Summary: … packages finished` 줄이 있는지 보고, 없으면 그 총계를 쓰지 않는다.
- **`colcon test` 의 종료 코드는 테스트 실패를 말하지 않는다 — 실패한 테스트가 있어도 0 이다.** 종료 코드로 판정하려면 `--return-code-on-test-failure` 를 붙인다 (실패 시 1). 그리고 **`colcon test-result` 에는 `--packages-select` 가 없다** — 붙이면 usage 오류(exit 2)만 나오고 결과는 한 줄도 안 읽히며, 그 오류문에는 "N failures" 가 없으므로 출력을 정규식으로 훑는 검사는 이를 **실패 없음**으로 읽는다. Stop hook 의 패키지별 판정이 처음부터 이 형태였고, 테스트를 대신하는 seam 이 두 호출을 통째로 가려 suite 도 보지 못했다 (2026-10-01 — 실패 테스트가 있는 패키지를 green 으로 기록). 패키지 하나의 결과만 보려면 `colcon test-result --test-result-base build/<pkg>` 이고, 그 범위에는 지난 실행의 `Testing/<stamp>/Test.xml` 이 함께 들어온다 (아래 범위 항).
- **`--test-result-base` 의 *범위*가 총계를 바꾼다 — 같은 트리, 같은 실행인데도.** `build/<pkg>` 를 주면 `build/<pkg>/Testing/<타임스탬프>/Test.xml`(CTest, 실행마다 **새 디렉토리로 누적**)까지 합산하고, `build/<pkg>/test_results` 를 주면 gtest/pytest/lint XML 만 센다. 실측: `integrated_bringup` 이 각각 **639 / 599**. 어느 쪽도 틀리지 않았고 **단위가 다를 뿐**이므로, 회귀 비교는 반드시 **같은 범위**로 한다. 옛 수치와 안 맞을 때 stale 로 단정하기 전에 범위부터 맞춰 볼 것 — 실제로 이 차이를 stale XML 로 오진한 적이 있다.
- **gtest case 수와 ctest entry 수를 함께 센다.** `rtc_controllers 333` = gtest 315 + ctest 18. 옛 수치와 비교할 땐 **단위가 같은지** 먼저 확인한다.
- **lint 도 소스 파일당 1 entry 를 낸다.** `cppcheck.xunit.xml` 의 `tests="N"` 은 그 패키지의 소스 파일 수이고 전부 **skipped** 로 집계된다. 따라서 테스트 `.cpp` 를 한 개 추가하면 총계는 **1(케이스가 1개일 때) + 1(ctest 타깃) + 1(cppcheck 파일)** 로 오르고 skipped 도 +1 이 된다 — 델타를 gtest 케이스 수만으로 예측하면 매번 어긋난다. 증감을 "삭제·신설 목록과 1:1" 로 설명해야 하는 슬라이스에서는 이 세 항을 분리해 적는다.

- **timeout·crash 는 gtest XML 에 `<failure>` 로 안 남는다 — CTest `Test.xml` 에만 잡힌다.** 타임아웃난 바이너리는 죽을 때 자기 `--gtest_output` XML 을 **아예 못 쓰거나 부분만 쓰므로**, `test_results/*.xml` 을 `<failure>` 로 훑는 검사는 그 런을 **clean 으로 판정한다**. 실측 (#345 검증): 22패키지 병렬 `colcon test` 에서 `ur5e_bt_coordinator::test_condition_nodes` 가 60 s CTest timeout 을 냈는데(단독 재실행 **0.47 s** 통과 — 부하 flake), gtest XML 전체에 `<failure>` 가 **0건**이었다. 따라서 **판정은 `colcon test-result` 의 요약(errors 를 포함)이나 `build/<pkg>/Testing/*/Test.xml` 로 하고**, gtest XML 직접 grep 을 유일 센서로 쓰지 않는다. 위 `--test-result-base` 범위 항과 짝이다: `build/<pkg>` 범위여야 `Test.xml` 이 합산에 들어온다.
- **`--test-result-base <경로>` 를 기본값 아닌 곳으로 주면 총계가 조용히 바이너리 수로 줄어든다.** 각 gtest 타깃의 `--gtest_output=xml:...` 경로는 **configure 시점에 `build/<pkg>/test_results/` 로 박히므로** 이 옵션을 따라오지 않는다. 커스텀 경로에는 CTest 의 `Test.xml` 만 떨어지고, `colcon test-result --test-result-base <그 경로>` 는 그것만 합산해 **바이너리 1개당 1건** 을 보고한다 (실측: 189 대신 20). 실패가 아니라 *그럴듯하게 작은 수*라 자체 검산 없이는 회귀로 오독하기 쉽다. AGENTS.md §9.1 cwd drift 를 피하려고 절대경로 result-base 를 습관화하면 정확히 이 함정을 밟는다 — cwd 는 `cd <rtc_ws> &&` 로 고정하고 **result-base 는 건드리지 않는 것**이 맞다. 굳이 분리하려면 판정을 `build/<pkg>/test_results/*.gtest.xml` 의 per-file `tests="N"` 으로 한다.

- **flake 는 재현 전에 `build/<pkg>/Testing/Temporary/` 부터 연다.** ctest 는 실행마다 `LastTest_<UTC stamp>.log` 를 남기고 **지우지 않으므로** 몇 주 전 실패가 로그·스택·타이밍째 그대로 있다 — 관측은 이미 공짜로 쌓여 있다. `Test time = 60.0x sec` 는 crash 가 아니라 `ament_add_test` 기본 TIMEOUT 이라는 판독이고 (#401 이 재현 없이 이것으로 종결), 실패 블록(`Testing: <name>` ~ `Test time =`)의 마지막 `[ RUN ]`·마지막 로그 줄이 멈춘 지점(SetUp/TearDown)을 가르며, 같은 시간 창에서 죽은 다른 프로세스가 없으면 "다른 테스트가 죽여서 오염" 가설은 그 자리에서 무너진다. 재현 하네스는 그 다음에 짠다.
- **한 gtest 바이너리가 ctest 에 여러 번 등록될 수 있다** (`ament_add_gtest_executable` + `ament_add_gtest_test` × N, `ENV "GTEST_FILTER=..."` — 등록 형태·여집합 필터 규칙·XML 분리는 [integrated_bringup/CMakeLists.txt](../integrated_bringup/CMakeLists.txt) 의 인라인 주석이 SSoT). 이런 바이너리를 ctest 밖에서 **맨손으로 돌리면** 필터 없이 두 등록의 케이스가 한 프로세스에 섞여, 프로세스 분리를 전제한 쪽이 실패한다 — 코드가 아니라 실행 방식이 만든 red 다. 전 바이너리 스윕에서 hit 이 나오면 `ctest -N` 으로 그 이름이 몇 번 등록됐는지부터 보고 `GTEST_FILTER` 를 등록대로 주어 재현한다 (#454).

신규 테스트 개수를 주장할 땐 총계 차이가 아니라 `grep -c '^TEST(' <파일>` 또는 per-target XML 의 `tests="N"` 으로 교차검증한다.

대표 suite 명은 `<pkg>/CMakeLists.txt` 에서 `ament_add_gtest()` / `ament_add_pytest_test()` grep — 코드 자체가 SSoT 이므로 문서 박제 불필요.

### Coverage 측정 (gcov/gcovr)

build.sh wrapper 는 없고, runtime PC 에 `lcov`/`gcovr` 가 없을 수 있다 (`gcov` 만 존재). gcovr 은 venv 에 설치 (`pip` 부재 — venv 는 `uv` 기반):

```bash
source repo_scripts/scripts/setup_env.sh
uv pip install gcovr                           # 분석 전용 도구 — runtime 에 영향 없음
# coverage 빌드: 패키지 CMakeLists 의 ENABLE_COVERAGE 옵션 사용 (--coverage 플래그 주입)
colcon build --packages-select <pkg> --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
find build/<pkg> -name '*.gcda' -delete         # 누적 카운트 초기화 (정확한 측정)
colcon test  --packages-select <pkg>
SRC=src/rtc-framework/<pkg>
gcovr --root "$SRC" --object-directory build/<pkg> --filter "$SRC/src/" --print-summary
# 측정 후: clean Release 재빌드로 install tree 원복 (coverage 빌드가 install 을 덮음 →
# downstream 이 gcov 심볼을 링크하게 됨)
rm -rf build/<pkg> install/<pkg> && colcon build --packages-select <pkg>
```

> `ENABLE_COVERAGE` 옵션은 패키지 CMakeLists 에 개별 정의한다 (보유 패키지: `grep -rl ENABLE_COVERAGE src/rtc-framework/*/CMakeLists.txt`). 미보유 패키지 측정 시 동일 `option(ENABLE_COVERAGE ... OFF)` + `add_compile_options(--coverage ...)` 블록을 추가한다. header-only 패키지는 `--filter "$SRC/include/"`, src 기반은 `--filter "$SRC/src/"`.

> **Python (ament_python) 패키지는 gcov/gcovr 가 아니라 `coverage`** (venv 설치, `uv pip install coverage`): `gcovr` 은 C++ 전용이라 `.py` 를 계측하지 못한다. CMakeLists/`ENABLE_COVERAGE` 도 무관 (ament_python 은 CMake 없음 — colcon 이 `test/test_*.py` 자동 발견). 측정: `cd <pkg> && python3 -m coverage run --source=<pkg_module> -m pytest test/ -q && python3 -m coverage report -m`. 측정 후 `.coverage` 아티팩트 삭제. 보유 Python 패키지: `rtc_digital_twin`, `rtc_tools`.

> **`rtc_mpc` 측정 함정**: 위 recipe 의 `-DCMAKE_BUILD_TYPE=Debug` 를 `rtc_mpc` 에 그대로 쓰면 안 된다. (1) Debug 빌드가 proxsuite all-zero-C assert 를 표면화한다 (Release 는 NDEBUG 로 숨김). (2) aligator 0.19.0 의 contact_rich MPC 는 `LD_PRELOAD=$DEPS/install/lib/libmimalloc.so` 없이는 `free(): invalid pointer` 로 죽는다 (`rtc_mpc/README.md`). `ENABLE_COVERAGE` 옵션 추가 자체(default OFF)는 무해하나, 실제 측정은 두 우회를 모두 적용해야 한다.

> **`integrated_bringup` 측정 함정 — Debug 에서 `test_demo_task_controller` 가 abort 한다.** `NonFiniteJacobianHoldsInsteadOfSolvingWithAStaleInverse` 가 의도적으로 non-finite Jacobian 을 주입하는데, 그 결과 회전행렬이 비유니터리가 되어 pinocchio `rpy.hxx` 의 `assert(R.isUnitary())` 가 발동한다 (Release 는 NDEBUG 로 무력 — 위 proxsuite 항과 **같은 범주**: Debug-only assert 가 coverage 빌드에서만 표면화). 증상은 그 바이너리 하나가 통째로 죽어 `test-result` 총계가 **그 바이너리의 케이스 수만큼** 줄고 gcda 도 안 남는 것이다 (절대 수치는 박제하지 않는다 — 그 XML 의 `tests="N"` 을 직접 읽는다). **coverage run 의 이 실패는 회귀가 아니므로 자기 변경 탓으로 오진하지 말 것** — 판정은 Release 재실행으로 한다.

> **정규 트리를 오염시키지 말고 `--build-base`/`--install-base` 를 분리하라.** 위 recipe 의 "측정 후 clean Release 재빌드" 대신, coverage 빌드를 **scratchpad 절대경로**의 별도 트리로 내보내면 (`--build-base /tmp/.../cov/build --install-base /tmp/.../cov/install`) ws-root incremental cache 가 애초에 안 더러워지므로 원복 재빌드가 불필요하다 (AGENTS.md §9.1 은 그대로 — 호출은 여전히 ws root 에서). 이때 `gcovr --object-directory` 도 그 트리를 가리켜야 한다. `gcovr` 없이 `gcov` 만 있을 때는 `gcov -b -p <build>/CMakeFiles/<target>.dir/<path>/<file>.cpp.gcno` — 확장자 포함 `*.cpp.gcno` 를 **직접** 넘겨야 한다 (`-o <dir>` + 소스 경로 조합은 `<file>.gcno` 를 찾아 "cannot open notes file" 로 실패).

**LifecycleNode 노드(예: `rtc_controller_manager`) 유닛 커버리지 패턴**: `on_configure` 파이프라인 (params 로딩·device backend·publisher 생성) 과 private RT 헬퍼(`CheckTimeouts`/`CreateDeviceBackends`)는 friend accessor (`rtc::ControllerLifecycleTestAccess`, 헤더의 `friend` 선언으로 이름 고정) 로 private 멤버를 주입·호출해 **실 robot/RT 권한 없이** 구동한다 — `CreateDeviceBackends` 등은 `group_slot_map_`/`device_name_configs_` 주입 + registry fake backend 만으로 동작하고, `on_activate` 의 RT 루프는 `ApplyThreadConfig` 반환값을 버리므로 (SCHED_OTHER fallback) 테스트 샌드박스에서도 안전하다.

> **ROS 노드를 만드는 테스트는 전용 `ROS_DOMAIN_ID` 로 격리** (`ament_add_gtest(... ENV ROS_DOMAIN_ID=<n>)`): `colcon test` 는 패키지를 병렬 실행하므로, 노드를 생성·소멸하는 테스트가 기본 도메인 0 을 공유하면 discovery 버스와 Fast DDS SHM port 객체 (`/dev/shm/fastrtps_port<N>` + 그 named mutex) 를 함께 쓴다. 피해는 두 층이다 — **타 패키지의 pub/sub 매칭 테스트가 깨지고** (PR #187: `ur5e_bt_coordinator` `test_rewire_gate` 가 피해자, 실패 시각이 가해 테스트 창과 일치), 심하면 **rmw endpoint 생성·소멸에서 그대로 멈춘다** (#401: ctest 가 60 s 에 죽여 결과 파일이 안 남는다. 부하 하 재현율 7/18, 격리 후 0). 후자는 "느려짐" 이 아니라 전 스레드 `futex_do_wait` 정지이므로 timeout 상향으로는 안 없어진다.
>
> **배정 단위는 패키지 하나에 도메인 하나**다 — colcon 이 병렬화하는 것은 패키지이고 한 패키지 안에서 ctest 는 직렬로 돌기 때문에 (패키지가 스스로 병렬을 켜지 않은 한 — 아래 "패키지 안 병렬"), 같은 패키지의 타깃들이 한 번호를 공유하는 것은 안전하다. **번호를 손으로 고르고 주석에 "어디까지 찼다" 를 적는 방식은 실패했다** — `udp_hand_driver` 가 "44-52 are taken" 을 근거로 53 을 골랐는데 `integrated_bringup` 이 이미 쓰고 있었다 (#401). 현재 배정은 `python3 repo_scripts/scripts/validate_test_domains.py --list` 가 소스에서 **파생**하고 (박제된 표는 없다 — AP-DOC-1), 같은 스크립트가 `repo_scripts` 의 `test_domain_allocation` 으로 등록되어 차단한다. 값은 반드시 literal 이어야 한다 — `ENV ROS_DOMAIN_ID=${VAR}` 는 게이트가 못 보므로 그 패키지가 아무것도 주장하지 않은 것처럼 보인다.
>
> **이 규칙은 이제 강제된다 — 그전까지는 문서에만 있었고 지켜지지 않았다.** 게이트가 원래 보던 것은 "주장된 번호가 겹치는가" 뿐이라 **아무것도 주장하지 않은 테스트들이 전부 도메인 0 을 공유하는 경우**가 안 보였다. #401 이 `ur5e_bt_coordinator` 를 55 로 뺀 뒤에도 노드를 만드는 타깃 15개가 5개 패키지에 걸쳐 0 에 남아 있었다 (피해자만 버스에서 뺐고 버스는 그대로였다). 지금은 게이트가 **테스트 소스**를 읽어 (`rclcpp::init(` / `rclpy.init(`, fixture 헤더까지 include 추적) 참가자를 여는 타깃이 도메인을 주장하지 않으면 차단한다. **주석·docstring 의 산문은 코드가 아니다** — 첫 실행에서 `thread_config.hpp` 의 `//` 주석 하나와 launch 테스트의 docstring 이 오탐 5건을 냈다.
>
> **`ament_python` 패키지는 substrate 가 다르다** — CMakeLists 가 아예 없어 `ENV` 를 못 쓴다. claim 은 `test/conftest.py` 의 `os.environ["ROS_DOMAIN_ID"] = "<n>"` 로 적고 (pytest 가 테스트 모듈보다 먼저 import 하므로 `rclpy.init()` 전에 잡힌다), 게이트가 그 값을 같은 배정 공간에서 검사한다 — python claim 이 CMake claim 과 충돌하면 red 다.
>
> **패키지 안 병렬 (`colcon.pkg`)**: 패키지 루트의 `colcon.pkg` 에 `ctest-args: ["-j", "<n>"]` 를 두면 colcon 이 그 패키지의 모든 `colcon test` 에 그 인자를 덧붙여 ctest 가 테스트를 n 개씩 겹쳐 돌린다 (CLI 의 `--ctest-args -R …` 뒤에 붙으므로 함께 써도 된다). 켠 패키지는 `validate_test_domains.py --list` 가 보여준다 (실측 2026-10-01, 6C/12T, `-j 4`: `rtc_controllers` 48 s → 23 s, `integrated_bringup` 61 s → 27 s, 각 5회 이상 연속 green). 켜는 순간 위의 "패키지 안은 직렬" 전제가 사라지므로 그 패키지의 CMakeLists 에 두 가지를 적는다. (1) **`ROS_DOMAIN_ID` 를 주장하는 테스트는 전부** `set_tests_properties(<이름들> PROPERTIES RESOURCE_LOCK ros_domain_<n>)` 에 literal 로 올린다 — 같은 lock 을 쥔 테스트는 겹쳐 돌지 않으므로 한 도메인에 participant 가 둘 뜨지 않는다. 빠뜨리면 게이트가 차단한다 (같은 스크립트의 규칙 3; lock 이름이 도메인 번호를 담으므로 claim 과 lock 이 따로 놀 수 없다). (2) **측정한 wall-clock 을 단언하는 테스트** (planner p99, solve-time 최대값, "N ms 안에 돌아온다") 는 `RUN_SERIAL TRUE` 로 혼자 돌린다 — 형제 테스트와 코어를 나누면 단언이 코드가 아니라 부하를 잰다. 이쪽은 게이트가 볼 수 없으므로 테스트를 추가하는 사람이 판정한다. 단언 없이 **기록만 하는 duration** (`RecordProperty` 의 `worst_*_ns` 류) 은 보호되지 않는다 — 인용할 값은 `--ctest-args -R <이름>` 단독 실행에서 읽는다. **직렬로 되돌려 돌리기** (실패가 병렬 탓인지 가를 때): CLI 의 `--ctest-args -j 1` 은 듣지 않는다 (`colcon.pkg` 의 `-j` 가 뒤에 붙어 이긴다). `COLCON_EXTENSION_BLOCKLIST=colcon_core.package_augmentation.colcon_pkg colcon test …` 로 `colcon.pkg` 를 읽지 않게 한다. **`ament_python` 패키지의 같은 장치는 `pytest-args: ["-n", "<n>"]`** (pytest-xdist worker) 다 — `rtc_tools` 105 s → 37 s (`-n 4`, 통과·skip 집합 동일). xdist 는 가속 장치이지 요구사항이 아니다: 없는 호스트에서는 그 패키지의 `test/conftest.py` 가 `-n` 을 받아 무시하고 하나씩 돌리며 report header 에 그 사실을 적는다 (`install.sh` 가 `python3-pytest-xdist` 를 선택 설치한다). xdist 에는 lock 이 없으므로 **participant 를 여는 python 패키지는 worker 를 요청할 수 없다** (게이트가 차단). worker 는 테스트를 케이스 단위로 나눠 갖는다 — module/session fixture 는 worker 마다 다시 만들어지고, 고정 경로에 쓰는 테스트는 `tmp_path` 로 옮겨야 겹쳐 돌 수 있다.

> **컨트롤러를 configure 하는 테스트는 세션 디렉토리도 격리한다.** 출하 YAML 의 `logs:` 블록을 가진 컨트롤러는 on_configure 에서 CSV 를 열고, `RTC_SESSION_DIR` 이 없으면 세션 resolver 가 워크스페이스의 **실제 `logging_data/<YYMMDD_HHMM>`** 로 떨어진다 — 테스트 행이 운영자 세션 사이에 남고, 같은 분에 띄운 sim 실험 세션과 섞인다 (2026-09-24 S8-B: Stop hook 의 `colcon test` 가 실험 세션 하나의 diag 헤더를 오염). `integrated_bringup` 은 이를 **구조로 강제**한다 — CMakeLists 의 test 블록 끝이 `test/session_dir_test_env.cpp` (바이너리마다 임시 디렉토리 하나, 끝나면 삭제) 를 모든 `test_*` gtest 실행 파일에 링크하므로 새 테스트가 따로 등록할 것이 없다 (파일마다 등록하던 방식은 7 개 바이너리가 빠뜨렸다). 로그 파일을 읽어야 하는 테스트는 `testfx::SessionDir().Dir()` 를 쓴다. 다른 패키지에 같은 종류의 테스트를 넣으면 같은 격리가 필요하다 — 확인은 `colcon test` 전후로 `logging_data/` 에 새 세션이 없는지 본다.

**`.venv` 격리**: 규칙은 [AGENTS.md](../AGENTS.md) §9.2 가 갖는다. 신호 (`Testing/Temporary/LastTest.log` Start/End 동일 초) 가 재발하면 `env -i` 깨끗한 셸에서 `setup_env.sh` source 후 `sys.path` 순서 점검부터.

이 격리에는 **로컬 전용 false-green 방향**도 있다: `colcon` 자체와 colcon 이 생성하는 console script 의 shebang 이 `/usr/bin/python3` 라, venv 를 activate 해도 `colcon test` 의 pytest 는 시스템 python 으로 돌아 `.venv` 전용 패키지를 import 못 하고 (`pytest.importorskip` 테스트가 **조용히 skip** — 실측: `rtc_tools` mujoco 의존 11개 전부), `ros2 run` 도 venv 를 못 봐 같은 검증이 `.venv/bin/python` 직접 실행과 **다른 답**을 낸다. 실제 인터프리터는 `log/latest_test/<pkg>/command.log` 가 확정해 준다. CI 는 venv 없이 `pip install` 이라 영향이 없으므로 방심 방향이 반대다 — CI green 을 근거로 로컬 skip 을 무시하지 말 것. 확인이 필요하면 (AGENTS.md §9.2 우회 금지 하에) `PYTHONPATH=<pkg> .venv/bin/python -m pytest …` 로 직접 돌리고, 자작 게이트에는 "검사가 아예 안 돌았음" 을 통과와 구분하는 플래그를 둔다.

## Live Debug Topics

런타임 문제 탐지용 토픽. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상.

| Topic | 발행 주체 | 언제 보나 |
|-------|----------|----------|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 파악 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | Controller switch 확인. BT / GUI / digital_twin은 이 토픽으로 리와이어 |
| `/<config_key>/<config_key>/get_parameters` (srv) | active 데모 컨트롤러의 LifecycleNode | Runtime gain 값 조회 (`ros2 param get`) |
| `/forward_position_controller/commands` | **robot 모드 + `ur_driver_native` backend 전용** | RT loop 건강성 — `ros2 topic hz` 로 설정된 `control_rate` (default 500 Hz) 매칭 확인. **sim 에는 이 토픽이 없다** — sim 의 커맨드 lane 은 device group 당 하나이고 (`devices.<group>.backend.command_topic`, 예: `/ur5e/joint_command` + `/p1a/joint_command`) robot 당 하나가 아니므로, sim 에서 이 토픽의 침묵을 "RT loop 정지" 로 읽으면 오진이다 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop @ `control_rate` (`rtc::ThreadTimingProducer<RtTickTimingPayload>`) drained by `DrainLog()` log thread | RT loop per-tick timing — 8 cols `t_wall_ns,tick_count,run_id,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`. p50/p99 등은 post-process 계산. **`run_id` 로 먼저 그룹핑한다** — 세션 디렉토리가 분 해상도라 같은 분의 재기동이 같은 파일에 append 되고, 파일 전체로 `n / span` 을 내면 어느 런에도 없던 레이트가 나온다 (#376; `plot_rtc_log` 는 마지막 런을 자동 선택하고 무엇을 버렸는지 출력한다. `--run-id` 로 다른 런 선택). **Sim 모드 (`use_sim_time_sync=true`) 에서는 `jitter_us` 컬럼이 항상 0.0** — CV wakeup 이라 `\|actual_period − budget\|`이 sim cadence 잡음일 뿐 RT 지표가 아니기 때문 (`PeriodicRtThread::JitterMeaningful()` override). 다른 6개 컬럼은 robot/sim 동일 의미. **세 phase 열은 `t_total_us` 로 합산되지 않는다** — CM 은 publish phase 를 SPSC/eventfd 인계 끝에서 끊으므로 남는 `t_total_us − (t_state+t_compute+t_publish)` 가 그 tick 의 post-publish tail (뒤따르는 per-tick 작업 + 스레드가 돌지 못한 시간) 이다. **긴 tick 을 "publish 때문" 이라 읽기 전에 이 잔차부터 본다** — #222 에서 traced 최악 overrun 이 publish 28 µs + tail 3.4 ms 였고, 잔차가 없던 시절의 CSV 는 그 3.4 ms 를 `t_publish_us` 에 얹어 "publish-dominated" 로 보고했다. `plot_rtc_log` 는 이 잔차를 `Tail (unattributed)` 밴드·통계로 낸다. tail 로 판명되면 그것이 `CM::PublishHandoff` 안인지 밖인지는 trace 가 가른다 (§Tracing) |
| `<session>/timing/mpc_timing_log.csv` | per-controller LifecycleNode 1 Hz aux drains `MPCThread::TimingProducer()` | **Per-MPC-tick raw 샘플** — CM과 동일한 8-col 스키마 (`run_id` + RtTickTimingPayload). 한 row = 한 main-loop iteration. p50/p99/max는 post-process로 계산 (예: `awk` / pandas). aggregate INFO 라인은 controller 로그에 10 s마다 출력 (handler self-report `solve_duration_ns` 256-sample 윈도우). 두 CSV 모두 같은 generic infra + 동일 payload (`rtc_base/timing/rt_tick_timing_sample.hpp`) — 새 thread 추가 시 payload 재사용 |
| `<session>/timing/rt_callback_timing_log.csv` | rt_callback thread (slot 2, FIFO 70) — 각 device state 콜백의 `DeviceBackend::StateLaneTimingScope`, drained by 같은 `DrainLog()` | **Per-state-callback raw 샘플** (tick 당이 아니라 **콜백 당** 1행: arm joint + hand joint/motor/sensor). 같은 8-col 스키마지만 의미가 lane-specific — `t_state_us`=decode, `t_publish_us`=mailbox hand-off (notify 안 하는 hand motor/sensor 는 0), `t_total_us`=slot 2 duty 분자, `t_compute_us`/`jitter_us`=0. **dispatch 간격은 연속 행 `t_wall_ns` 차분**으로 본다. 이 lane 이 존재하는 이유는 slot 2 가 aux 통합(#349)의 대상인데 계측이 없었기 때문 — "93% 유휴"는 slot 1 수치다 |
| `/rtc_cm/{group}/joint_states` | CM (per-group, RELIABLE) | Device 그룹별 건강성; `rtc_digital_twin`이 merge |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | Sim 건강성 — 중단 시 sim sync timeout E-STOP |
| `<contact_wrench.topic_prefix>/<target>/contact_wrench` | `rtc_mujoco_sim` per-target (`mjSENS_CONTACT` netforce + world→link transform, **link-on-environment** 부호 — `FingertipSensor.f` 와 동일) | Fingertip 접촉 force/torque 확인. `RViz2 → WrenchStamped` display 또는 `ros2 topic echo`. 비접촉 시 0 발행. 활성화 조건: 그룹 YAML `contact_wrench.enabled: true` + MJCF 에 `<sensor><contact>` (data=`found force torque dist pos normal tangent` num=1 reduce=netforce) |
| `/p1a/joint_states`, `/p1a/motor_states`, `/p1a/sensor_states` | `udp_hand_driver` (ur5e_p1a; generic driver default `/hand/`) | Hand UDP 건강성 |
| `/shape_estimation/snapshot` (action feedback) | `shape_estimation` | ToF 기반 추정 진행 상황 |

## Debugging

| Symptom | Fix |
|---------|-----|
| `ApplyThreadConfig()` warns | `sudo usermod -aG realtime $USER` + re-login |
| E-STOP on startup | Set `init_timeout_sec: 0.0` for sim |
| High jitter (>200us) | Check `taskset` pinning, verify `isolcpus`, `check_rt_setup.sh --summary` |
| Hand timeout E-STOP | Check UDP link, `recv_timeout_ms: 0.4` |
| Controller not found | Use config_key (e.g. "demo_task_controller") or Name() |
| 팔이 멈췄는데 궤적이 "정착"인지 "관절 밴드에 pin"인지 모르겠다 | 컨트롤러 로그의 `joint band engaged:` WARN (2 s throttle, 발동 tick 에서만) — 그 줄이 찍히는 구간의 평평한 관절 궤적은 pin 이다. `on_deactivate` 요약이 tick 창을 준다. **Ctrl-C 로 끝냈으면 요약은 없다** (CM `main()` 이 lifecycle 훅을 안 탄다) — WARN scrollback 이 유일 증거. 판독법: `integrated_bringup/README.md` compliance §7.3 관절 밴드 판독 |
| `ament_cmake_test` missing on `colcon test` | `.venv` overlay가 system site-packages를 가림. 재활성화 + ROS 2 환경 재로드 |
| 0.01초 만의 configure 실패 `Failed to find <repo>/install/<pkg>/.../package.sh` | 잘못된 cwd 로 돈 colcon (AGENTS.md §9.1) — repo 안 `build/`·`install/`·`log/` 삭제 후 ws root 에서 재실행 |
| 빌드 성공 직후 테스트 바이너리 `No such file or directory` | 위와 동일 — ws-root 트리와 repo-안 트리가 갈라진 상태. `ls src/rtc-framework/build` 로 확정 |
| env 미source 로 전 바이너리 일괄 실패 | 회귀 아님 — AGENTS.md §9.1 서브셸 표준형으로 재실행. python `subprocess` 는 `executable="/bin/bash"` 명시 (`/bin/sh` 에는 `source` 가 없어 체인이 첫 항에서 죽는다) |
| `ignoring unknown package '<pkg>' in --packages-select` + `0 packages finished` | ws 밖(scratchpad 등) cwd — 직전 run 의 stale XML 이 green 으로 읽히므로 판정 전 결과 XML mtime 확인 |
| `No rule to make target '/opt/ros/<distro>/lib/lib<X>.so.<옛 버전>'` (코드 변경과 무관하게 여러 패키지) | 회귀 아님 — ROS apt 업그레이드가 so 버전을 올렸고 (`/var/log/apt/history.log`), configure 때 생성된 link 규칙이 옛 절대경로를 들고 있다. `colcon build --cmake-force-configure` (명령 형태는 [repo_scripts/README.md](../repo_scripts/README.md) "Plain `colcon build` 호환성") — CMakeCache 는 유지되고 link 규칙만 재생성된다 |
| `ament_cmake_symlink_install_files() can't find '.../rosidl_generator_type_description/<pkg>/msg/<Msg>.json'` | 그 메시지 패키지의 생성물 일부가 빠졌는데 생성 단계는 최신으로 기록된 상태 (중단된 빌드 뒤 관측) — 그 패키지만 `colcon build --packages-select <pkg> --cmake-clean-first` |

```bash
# exec name = ROS node name = "integrated_rt_controller" (only exec from integrated_bringup;
# rtc_controller_manager is library-only). Use the same name for pgrep and
# lifecycle calls:
#   ros2 lifecycle list /integrated_rt_controller
PID=$(pgrep -f integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
# RT loop 건강성: robot + ur_driver_native 면 아래 토픽, sim 이면 device group 별 command_topic
ros2 topic hz /forward_position_controller/commands
ros2 topic echo /system/estop_status
./repo_scripts/scripts/check_rt_setup.sh --summary
```

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Optional: isolcpus, nohz_full, or cpu_shield.sh
```

## CPU Shield (cset) 검증 — issue #151

`cpu_shield.sh` 는 cpuset 을 *만들기만* 하고, 런치가 CM 을 그 안으로 `adopt` 한다.
격리가 실제로 서는지는 **실기(SMT/hybrid 호스트)** 에서만 검증된다 —
sim 단일 실행으로 대체 불가 ([testing-debug.md](../agent_docs/testing-debug.md) §런타임 판독).

```bash
# 1) shield cpuset 이 CM 전체 span 을 덮는가 (기대 집합 = get_cm_shield_cpus <profile> 출력; 값은 박제하지 않는다)
sudo ./repo_scripts/scripts/cpu_shield.sh on --robot
cset shield -s          # "user" == get_cm_shield_cpus 출력과 일치해야
# 2) 런치(shield-on) 후 CM 이 user cpuset 에 들어갔는가
CM=$(pgrep -nf integrated_rt_controller)
grep Cpus_allowed_list /proc/$CM/status      # == get_cm_shield_cpus 출력과 동일 (부분집합 비교는 shield 축소를 놓친다)
# 3) activate 후 RT/nrt 스레드가 제대로 pin·FIFO 되었는가
ps -eLo comm,psr,cls,rtprio -p $CM | grep -E "rt_control|rt_callback|nrt_"
#   기대값은 layout SSoT 에서 — psr: get_role_slot <role> 의 slot→logical, rtprio: get_role_priority <role>,
#   cls: get_role_policy <role> (SCHED_FIFO → FF, SCHED_OTHER → TS — nrt_* 는 CFS 라 TS 가 정상)
#   (repo_scripts/README.md "RT/MPC 코어 레이아웃 함수"; 머신별 숫자를 여기 박제하지 않는다)
# 4) EINVAL 회귀 없음 (shield 가 pin 을 깨뜨리지 않음)
grep -rE "rc=22|setaffinity failed|Thread config failed" ~/.ros/log/<run>/  # 결과 없어야
# 5) 게이트가 활성 shield 를 재활성 안 함 (cset-aware)
#   두 번째 런치 로그에 "CPU shield already active (cset user cpuset present)"
```

## Tracing

CSV timing logs (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv`) 은 **per-tick 총 시간** 만 기록한다. 어느 thread 가 어느 core 에서 언제 run 했는지 / 어떤 callback 이 시간을 쓰는지 알아내려면 LTTng 트레이스를 캡처해 분석한다.

세부 명령·event 선택·permission 분기·뷰어 사용법은 [tracing.md](tracing.md) 참조 (ros2_tracing / babeltrace2 / Perfetto operational guide).

핵심 진입점만:

```bash
./install.sh --tracing                                              # 1회 setup
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true   # 캡처
./repo_scripts/scripts/timeline.sh                                  # Perfetto JSON 변환
```
