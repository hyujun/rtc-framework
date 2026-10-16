# Testing — 근거·실측·사례

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/testing-debug.md](../../agent_docs/testing-debug.md) 가 갖고, 실행 방법은 [docs/testing.md](../testing.md) 가 갖는다. 여기는 그 규칙이 *왜* 생겼는지, 함정이 실제로 어떻게 발현했는지, 무엇을 실측했는지를 기록한다. 둘이 어긋나면 testing-debug.md 가 옳다. 수치·테스트 이름은 **기록 시점의 것**이며 다시 재야 한다.

## Sensor Matrix 보충

[docs/testing.md](../testing.md) 표가 한 줄이라 못 담은 "이 테스트가 무엇을 판별하는가" 를 행별로 적는다. 공통 원칙은 하나다: **fixture 는 판별하려는 차이를 일부러 비대칭으로 만들고, 그 비대칭이 깨지면 스스로 알린다.** 대칭 fixture 는 틀린 구현도 통과시킨다.

- **catching core + sanitizer** — 전용 ASan/UBSan 수단이 없어 별도 `--build-base` / `--install-base` 에 `RelWithDebInfo` + `-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer` 로 빌드하고 바이너리를 직접 실행한다. `ScopedAllocGate` TU 는 `new`→`malloc` 교체가 ASan 과 alloc-dealloc-mismatch 를 내므로 그 TU 만 `ASAN_OPTIONS=alloc_dealloc_mismatch=0` 으로 돌린다. `malloc_gate.hpp` 를 쓰는 TU (`ScopedMallocGate`: catching 의 planner · mpc segment · docking · nlp search · follower 계열, `test_clik_multiframe_malloc`) 는 실행 파일이 `malloc` 계열을 정의해 ASan 할당기와 섞이므로 ASan 아래에서 **실행할 수 없다**. 그 경로의 sanitizer 판정은 gate 없는 짝 테스트 (`test_clik_multiframe`) 가 대신한다. `test_clik_pink_reference` 의 기준값은 `rtc_tsid/test/golden/gen_pink_reference.py` docstring 절차로 만든다.
- **`rtc_mujoco_sim` 서보** — `test_motor_servo_gains` 는 `<motor>` 위의 위치 서보를 본다. 다른 fixture 는 전부 `<position>` (이미 affine) 이라 서보 lane 의 bias 타입 변경을 볼 수 없다. `test_torque_mode_ctrl_range` 는 ctrl clamp 의 해제·복귀를 flag 가 아니라 **actuator 힘** 으로 본다 (flag 만 보면 엉뚱한 actuator 에 clamp 가 걸려도 통과).
- **`rtc_urdf_bridge`** — `test_frame_jacobian_fd_oracle` 는 자코비안을 전혀 쓰지 않는 중심차분 oracle 로 행/열 계약을 본다 (기존 테스트는 유한성 + 자기일치뿐이라 행 블록을 맞바꿔도 green). 픽스처 `mixed_prismatic_revolute.urdf` 의 네 비대칭 (축 직교 · origin 3축 offset · tool rpy · prismatic 열의 zero angular) 이 판별력이다. 관성 게이트 (`test_inertial_validation` · `test_real_model_inertial_gate`) 의 판정 대상은 fixed joint 흡수 **후** composite 관성이고, tol 은 주모멘트 크기로 정규화한다 (절대 tol 은 실모델의 decade 폭을 양 끝에서 만족 못 한다). 임계값의 근거는 `inertial_validation.hpp` 의 `kInertialRelTol` 주석이 SSoT.
- **`rtc_tools` 모델 쌍** — 로컬 colcon 은 pytest 를 `/usr/bin/python3` 로 돌려 mujoco 를 못 보므로 `test_real_model_pairs.py` 가 **skip** 된다. 실제로 돌리려면 `PYTHONPATH=rtc_tools:$PYTHONPATH .venv/bin/python -m pytest rtc_tools/test/test_real_model_pairs.py rtc_tools/test/test_mjcf_compile_semantics.py`. `compare_mjcf_urdf` 의 MJCF 판독을 고치면 `test_mjcf_compile_semantics.py` 가 sensor 다 (합성 MJCF 의 컴파일 값을 리터럴로 적어 둔다; 절반은 mujoco 없이, 절반은 있어야 돈다).
- **demo 바인딩 테스트 공통 설계** — (1) oracle 은 전체 모델 FK 를 관절 **이름** 으로 계산해 reduced 모델 · frame id · 관절 순서 가정을 공유하지 않는다. (2) bring-up 순서가 둘 (`PreConfigure` → device config → `on_configure` / `LoadConfig` → device config → 다시 읽는 `on_configure`) 인 이유는 한 자리에서만 건 관절 순서 map 이 한쪽에서만 남기 때문이다. (3) fixture 의 비대칭 (root ≠ world, 팔 길이·축, 항등 아닌 장착, device 순서 ≠ 모델 순서) 은 `FixtureCanTellTheCasesApart` 류 케이스가 확인한다. (4) wbc 는 `tsid:` 없는 YAML 로 올린다 — TSID 가 서 있으면 CLIK 초기화 실패가 먼저 거부해 이 검사를 빼도 green.
- **compliance · catching** — `test_compliance_admittance_coupling` 은 법칙의 **배선** 만 본다 (코어 ODE 는 `rtc_controllers` 소관): 적분기 반쪽은 로컬 적분기와 bit-identical, 파이프라인 반쪽은 부호·축·레버암. `test_compliance_task_equivalence` 는 렌치를 withhold 하는 동안에만 demo_task 와 bit-identical 이고 그 전제를 tick 마다 단언한다. catching 바인딩은 공유 per-controller 스위트 4개 중 2개만 편입한다 — 이 컨트롤러는 E-STOP 훅을 하나도 override 하지 않고 (E-8 결정 자체), 팔 동역학도 observer 도 없다.
- **inference · observer** — `test_demo_inference_urdf` 는 출하 YAML 을 그대로 로드해 MuJoCo oracle 과 대조한다 (palm ±1 mm, fingertip ±2 mm). `test_demo_inference_alloc` 은 Eigen 의 malloc 을 못 본다. `test_demo_inference_real_model` 은 `RTC_POLICY_DIR` 이 없으면 전 케이스 skip — 로컬 전용이고 skip 은 미검증이다. observer 잔차는 CSV 말고 관측면이 없어서 `test_momentum_observer_embedding` 이 네 컨트롤러 × `momentum_observer.csv` 왕복을 본다 (unbound handle 에 쓴 행 수 단언은 push 를 지워도 통과). sim 통계는 `valid=1` 행만 쓴다 (held 행은 동결값). `test_payload_estimator` 의 oracle 은 wrench 를 **정방향** 으로 만들고 `r = Jᵀw` 로 내려 역산시킨다 — 역방향이면 부호 뒤집힘에도 green. 배선 테스트는 device order 를 뒤집은 채 알려진 질량을 매단다.
- **Launch** — `test_launch_description_evaluates` 는 `LaunchContext` 만으로 launch 를 실제 평가한다. import 스모크 (`test_launch_imports.py`) 는 `OpaqueFunction` 본문이 안 돌아, sim 세 개가 죽은 채 전 배터리가 green 이었던 적이 있다.

## Revert-verification

가드(검증·거부 경로)와 그 테스트를 함께 추가하면 **테스트 통과는 가드가 동작한다는 증거가 아니다**. 가드마다 하나씩 원복 → 대응 테스트가 실제로 실패하는지 확인 → 복구한다. 에이전트의 자기 평가를 믿지 않는다는 원칙이 자기가 쓴 테스트에도 그대로 적용된다. false green 은 세 가지 형태로 관측됐다.

**1. 층이 겹치는 가드.** 가드 A·B 가 같은 잘못된 입력을 모두 거부하면 B 의 테스트는 A 에 흡수되어 B 를 지워도 통과한다. 처방은 테스트를 "throw 하는가" 가 아니라 **그 가드만이 만드는 관측 가능한 차이** (진단 메시지, 거부 시점, 부작용 유무) 로 옮기는 것이다. 실측: 신규 가드 4개 중 flat `publish:` 탐지가 false green 이었다 (group-shape 가드가 같은 config 을 이미 거부). 테스트를 "마이그레이션 진단을 주는가" 로 옮겨 pin 했다. 형제 절반 (`subscribe` 쪽) 에서 같은 false green 이 재발했으므로 **한쪽을 고쳤으면 대칭 위치도 다시 잰다.**

**2. 관측 채널이 fallback 에 가려질 때.** 가드를 원복해도 테스트가 보는 값이 다른 경로로 같은 값을 내면 vacuous 다 — 잘못된 것은 가드가 아니라 관측 지점이다. E-STOP recovery drain 이 예다: OSC/TaskImpedance 는 gravity-comp 컨트롤러라 정지 시 hold torque ≈ ĝ(q) 인데, leak 된 target 이 멀면 recovery tick 에서 SAFE_STOP 이 latch 되고 그 hold 도 ĝ(q) 를 낸다. 그래서 joint torque 로 assert 하면 leak 유무와 무관하게 통과한다. 관측을 fallback 이 건드리지 않는 채널로 옮긴다 — OSC 는 goal echo, TaskImpedance 는 SAFE_STOP step *이전에* 기록되는 `diag.pose_error`.

**3. 게이트가 닫힌 쪽에서 수치적으로 inert 일 때.** `nullspace_active = (nv > 6) && (nullspace_kp != 0.0)` 에서 뒷항을 지워도 비트 레인이 전부 green 이었다. `nullspace_kp = 0` 이면 게이트 없는 경로가 `0.0·Δq` 의 signed zero 를 만들고 그것을 사영해 더하는 것은 exact no-op 이기 때문이다. 곱셈으로 꺼지는 게인·플래그 (`k = 0`, `α = 0`) 는 전부 이 형태이므로, 출력이 아니라 **진단 플래그를 tick 마다 대조**해 pin 하고, `EXPECT_FALSE` 만으로는 배선이 고정되지 않으니 양성 케이스를 함께 둔다.

**원복·복구의 함정.** `git checkout -- .` 은 아직 커밋하지 않은 작업까지 날린다 (실제 발생). 검증 대상이 미커밋이면 명시적 백업 사본에서 복구해야 하는데, **mtime 을 보존하는 복사 (`cp -p`, `shutil.copy2`) 로 복구하면 make 가 재컴파일을 건너뛴다.** 소스는 최신인데 빌드 트리에는 원복된 바이너리가 남아 결과를 반대로 읽게 된다 ("2 failures" 를 회귀로 오독한 사례). 복구 후에는 `touch` 하고 재빌드한다. 재컴파일이 실제로 돌았는지는 빌드 소요 시간으로 본다 — 0.x 초면 안 돈 것이다.

```bash
cp <파일> <scratch>/<파일>.orig              # -p 금지
# 가드 1개 원복 → 빌드 → 해당 실행파일만 → 복구
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select <pkg> --ctest-args -R <test_exe>
cp <scratch>/<파일>.orig <파일> && touch <파일>   # 미커밋 작업이 없으면 git checkout -- <파일>
```

## Test fixtures

robot 모델이 필요한 gtest fixture 는 URDF 를 `robot_descriptions/robots/<name>/` (repo 체크인) 에서 해석한다. `deps/src/...` 나 `/usr/local/...` 를 박으면 clean 환경에서 `buildModel` throw 나 `GTEST_SKIP` 이 난다. 해석은 compile macro (`RTC_PANDA_URDF_PATH`) 나 `ament_index_cpp::get_package_share_directory("robot_descriptions")` 로 한다.

**repo 밖 패키지를 resolve 하는 fixture 는 clean 환경에서 전부 red 인데 그게 안 보인다.** 사건: ur5e_p1b fixture 가 `hand_description` 패키지를 찾았는데, 그 패키지는 이 repo 에도 `deps.repos` 에도 없어 (로컬 ws 에만 있었다) CI 에서 model-backed 케이스가 전부 던졌다. best-effort 잡이라 PR 은 green 이었고 유일한 증상인 codecov patch % 는 "테스트를 안 썼다" 로 오독됐다 (observer 배선 테스트 14개 중 13개 red). 모델을 `robot_descriptions/robots/ur5e_p1b/` 로 vendor 해 닫았고 지금 test 소스가 여는 패키지는 전부 in-repo 다. (그 C++ CI 는 이후 제거돼 이 절의 센서가 clean 환경을 재현하는 유일한 방어선이다.)

- **센서 설계.** `validate_test_fixtures.py` 는 test 소스의 package resolution 을 훑어 repo 에도 `deps.repos` 에도 없는 패키지를 열면 차단한다. 리터럴 스캔으로는 부족했다 — 파라미터화 fixture 가 `get_package_share_directory(ec.urdf_pkg)` 처럼 struct 필드 경유로 열어 8곳 중 2곳만 잡혔다. 그래서 비-리터럴 인자도 unresolvable 로 치고, fixture 가 CMake source list 밖 헤더에 살기 때문에 include 를 따라가 그 헤더에 도달하는 모든 test `.cpp` 에 위반을 부과한다.
- **발화 증명은 코퍼스에 의존하지 않는다.** vendor 로 위반 코퍼스가 사라진 뒤 발화 증명은 `--self-test` 의 합성 코퍼스 (ok / guarded / bare / include 경유) 로 옮겼다. 실제 코퍼스를 positive control 로 쓰면 "위반 하나를 영원히 살려둬라" 가 된다. 격리 검증의 대조군도 같은 이유로, 던지는 테스트가 아니라 `AMENT_PREFIX_PATH` 에서 prefix 를 뺀 셸에서 `ros2 pkg prefix <pkg>` 가 실패하는 것이다.
- **install set ≠ local.** Python 테스트 잡은 `rtc_tools` / `rtc_msgs` / `rtc_digital_twin` 만 install 했다. 다른 패키지를 `ament_index` 로 resolve 하는 `rtc_tools` 테스트는 CI 에서만 `PackageNotFoundError` 가 난다. 렌더된 산출물만 검증하는 테스트는 경로 헬퍼를 monkeypatch 로 stub 한다.

## ROS_DOMAIN_ID 격리와 병렬

`colcon test` 는 패키지를 병렬 실행하므로, 노드를 만드는 테스트가 기본 도메인 0 을 공유하면 discovery 버스와 Fast DDS SHM port 객체를 함께 쓴다. 피해는 두 층이다. (1) 타 패키지의 pub/sub 매칭 테스트가 깨진다 (`ur5e_bt_coordinator` 의 `test_rewire_gate` 가 피해자). (2) 심하면 rmw endpoint 생성·소멸에서 멈춘다 — ctest 가 60 s 에 죽여 결과 파일이 안 남고, 부하 하 재현율 7/18, 격리 후 0 이었다. 전 스레드 `futex_do_wait` 정지라 timeout 상향으로 없어지지 않는다.

- **배정 단위는 패키지당 도메인 하나.** 한 패키지 안에서 ctest 는 직렬이므로 같은 패키지의 타깃은 한 번호를 공유해도 안전하다.
- **손으로 번호를 고르는 방식은 실패했다.** `udp_hand_driver` 가 "44-52 are taken" 주석을 근거로 53 을 골랐는데 `integrated_bringup` 이 이미 쓰고 있었다. 지금 배정은 `validate_test_domains.py --list` 가 소스에서 파생하고, 같은 스크립트가 `test_domain_allocation` 으로 등록돼 차단한다. 값은 literal 이어야 한다 (`${VAR}` 는 게이트가 못 본다).
- **"아무것도 주장하지 않은" 테스트도 잡는다.** 예전 게이트는 주장된 번호의 중복만 봐서, 노드를 만드는 타깃 15개가 5개 패키지에 걸쳐 도메인 0 에 남아 있었다. 지금은 테스트 소스에서 `rclcpp::init(` / `rclpy.init(` (fixture 헤더까지 include 추적) 를 찾아 도메인을 주장하지 않으면 차단한다. 주석·docstring 의 산문은 코드가 아니다.
- **`ament_python`** 은 CMakeLists 가 없어 `test/conftest.py` 에서 `os.environ["ROS_DOMAIN_ID"]` 로 claim 하고, 게이트가 같은 배정 공간에서 검사한다.
- **패키지 안 병렬 (`colcon.pkg`).** `ctest-args: ["-j", "<n>"]` 를 두면 그 패키지의 모든 `colcon test` 에 붙는다. 실측 (6C/12T, `-j 4`): `rtc_controllers` 48 s → 23 s, `integrated_bringup` 61 s → 27 s. 켜면 "패키지 안은 직렬" 전제가 사라지므로 (1) 도메인을 주장하는 테스트는 전부 `RESOURCE_LOCK ros_domain_<n>` 에 literal 로 올리고 (빠뜨리면 게이트가 차단), (2) wall-clock 을 단언하는 테스트 (planner p99, solve-time 최대값) 는 `RUN_SERIAL TRUE` 로 둔다 (게이트가 못 보므로 추가하는 사람이 판정). 기록만 하는 duration 은 `--ctest-args -R <이름>` 단독 실행에서 읽는다.
- **직렬로 되돌리기.** CLI 의 `-j 1` 은 `colcon.pkg` 의 `-j` 가 뒤에 붙어 듣지 않는다. `COLCON_EXTENSION_BLOCKLIST=colcon_core.package_augmentation.colcon_pkg colcon test …` 로 `colcon.pkg` 를 읽지 않게 한다. `ament_python` 은 `pytest-args: ["-n", "<n>"]` (pytest-xdist, `rtc_tools` 105 s → 37 s) 이고 없는 호스트에서는 `conftest.py` 가 무시한다. lock 이 없어 participant 를 여는 python 패키지는 worker 를 요청할 수 없다.

## 세션 디렉토리 격리

출하 YAML 의 `logs:` 블록을 가진 컨트롤러는 `on_configure` 에서 CSV 를 열고, `RTC_SESSION_DIR` 이 없으면 세션 resolver 가 workspace 의 **실제 `logging_data/<YYMMDD_HHMM>`** 로 떨어진다. 테스트 행이 운영자 세션 사이에 남고 같은 분에 띄운 sim 실험 세션과 섞인다 (Stop hook 의 `colcon test` 가 실험 세션 하나의 diag 헤더를 오염시킨 사례). `integrated_bringup` 은 이를 구조로 강제한다: CMakeLists 의 test 블록 끝이 `test/session_dir_test_env.cpp` (바이너리마다 임시 디렉토리 하나, 끝나면 삭제) 를 모든 `test_*` gtest 실행 파일에 링크하므로 새 테스트가 따로 등록할 것이 없다 (파일마다 등록하던 방식은 7개 바이너리가 빠뜨렸다). 로그를 읽어야 하는 테스트는 `testfx::SessionDir().Dir()` 를 쓴다. 다른 패키지에 같은 종류의 테스트를 넣으면 같은 격리가 필요하다 — `colcon test` 전후로 `logging_data/` 에 새 세션이 없는지 본다.

## 계측 함정

비교 가능한 수치를 낼 때 필수다. 각 항은 실제로 잘못된 보고를 낳은 경로다.

- **`colcon test-result` 는 남은 XML 을 전부 합산한다.** 타깃을 빼거나 브랜치를 되돌린 뒤 재측정하면 사라진 타깃의 옛 결과가 더해진다 (baseline 을 298 대신 300 으로 오측한 사례). 재기 전에 `build/<pkg>/test_results` 를 지운다.
- **`colcon test` 가 돌지 않아도 `colcon test-result` 는 답한다.** `lint && build && test; test-result` 체인의 앞이 실패하면 뒤의 `test-result` 는 이전 실행을 요약한다 (관련 없는 `ruff check` 실패가 빌드·테스트를 건너뛰었는데 `1179 tests, 0 failures` 가 찍혔다). 같은 출력에 `Summary: … packages finished` 가 있는지 본다.
- **`colcon test` 의 종료 코드는 테스트 실패를 말하지 않는다** (`--return-code-on-test-failure` 를 붙이면 1). `colcon test-result` 에는 `--packages-select` 가 없다 — 붙이면 usage 오류 (exit 2) 만 나오는데 그 오류문에는 "N failures" 가 없어, 출력을 정규식으로 훑는 검사는 이를 **실패 없음** 으로 읽는다. Stop hook 판정이 이 형태여서 실패 패키지가 green 으로 기록된 적이 있다. 패키지 하나는 `--test-result-base build/<pkg>`.
- **`--test-result-base` 의 범위가 총계를 바꾼다.** `build/<pkg>` 는 CTest 의 `Testing/<stamp>/Test.xml` (실행마다 누적) 까지 합산하고 `build/<pkg>/test_results` 는 gtest/pytest/lint XML 만 센다 (`integrated_bringup` 639 / 599). 둘 다 맞고 단위가 다를 뿐이니 비교는 같은 범위로 한다. 기본값 아닌 경로를 주면 gtest XML 경로가 configure 시점에 박혀 있어 총계가 조용히 바이너리 수로 줄어든다 (189 대신 20) — cwd 는 `cd <rtc_ws> &&` 로 고정하고 result-base 는 건드리지 않는다.
- **단위를 함께 센다.** `rtc_controllers 333` = gtest 315 + ctest 18. lint 도 소스 파일당 1 entry (`cppcheck.xunit.xml`, 전부 skipped) 라서 테스트 `.cpp` 하나를 추가하면 총계는 케이스 수 + 1 (ctest 타깃) + 1 (cppcheck) 오르고 skipped 도 +1 이다. 개수 주장은 소스의 `TEST(` 개수나 per-target XML 의 `tests="N"` 으로 교차검증한다.
- **timeout · crash 는 gtest XML 에 `<failure>` 로 안 남는다.** 타임아웃난 바이너리는 XML 을 못 쓰거나 부분만 쓰므로 `<failure>` 로 훑는 검사는 clean 으로 판정한다. 22패키지 병렬 실행에서 `test_condition_nodes` 가 60 s CTest timeout 을 냈는데 (단독 재실행 0.47 s) gtest XML 에 `<failure>` 가 0건이었다. 판정은 `colcon test-result` 요약이나 `Testing/*/Test.xml` 로 한다.
- **flake 는 재현 전에 `build/<pkg>/Testing/Temporary/` 부터 연다.** ctest 는 `LastTest_<UTC stamp>.log` 를 지우지 않아 몇 주 전 실패가 로그째 있다. `Test time = 60.0x sec` 는 crash 가 아니라 `ament_add_test` 기본 TIMEOUT 이고, 마지막 `[ RUN ]` 과 마지막 로그 줄이 멈춘 지점을 가른다.
- **한 gtest 바이너리가 ctest 에 여러 번 등록될 수 있다** (`GTEST_FILTER`). 맨손으로 돌리면 두 등록의 케이스가 한 프로세스에 섞여 코드가 아닌 실행 방식이 만든 red 가 난다. `ctest -N` 으로 등록 횟수를 보고 필터를 등록대로 주어 재현한다. 등록 형태는 `integrated_bringup/CMakeLists.txt` 의 인라인 주석이 SSoT.
- **등록되지 않은 test 파일은 어떤 수치에도 나타나지 않는다.** `ament_cmake` 패키지의 `test/test_*` 는 `CMakeLists.txt` 가 이름으로 부르지 않으면 안 돌고, 빌드는 통과하며 직접 돌리면 통과한다 (추가되고 commit 7개 뒤에야 등록된 파일이 있었다). `validate_test_registration.py` 가 차단한다 (`test_registration`, Stop hook, CI). 토큰 읽기라 거짓인 `if()` 안의 등록은 통과한다. `colcon test` 가 아닌 것이 돌리는 파일은 `# test-registration-exempt: <파일> -- <누가 대신 돌리는가>` 로 적는다. 새로 등록한 테스트는 재빌드 전에는 없다 (`colcon test` 는 reconfigure 하지 않는다).

## `.venv` 격리의 false-green

규칙은 [AGENTS.md](../../AGENTS.md) §9.2 가 갖는다. 신호 (`Testing/Temporary/LastTest.log` Start/End 동일 초) 가 재발하면 `env -i` 깨끗한 셸에서 `setup_env.sh` 를 source 한 뒤 `sys.path` 순서부터 점검한다. 이 격리에는 **로컬 전용 false-green 방향**이 있다: `colcon` 과 colcon 이 생성하는 console script 의 shebang 이 `/usr/bin/python3` 라, venv 를 activate 해도 `colcon test` 의 pytest 는 시스템 python 으로 돌아 `.venv` 전용 패키지를 import 못 하고 (`pytest.importorskip` 테스트가 **조용히 skip** — `rtc_tools` 의 mujoco 의존 11개 전부), `ros2 run` 도 venv 를 못 봐 `.venv/bin/python` 직접 실행과 다른 답을 낸다. 실제 인터프리터는 `log/latest_test/<pkg>/command.log` 가 확정해 준다. CI 는 venv 없이 `pip install` 이라 영향이 없으므로 방심 방향이 반대다 — CI green 을 근거로 로컬 skip 을 무시하지 않는다. 확인하려면 (우회 금지 하에) `PYTHONPATH=<pkg> .venv/bin/python -m pytest …` 로 직접 돌리고, 자작 게이트에는 "검사가 아예 안 돌았음" 을 통과와 구분하는 플래그를 둔다.

## RT-1 zero-allocation 게이트

RT tick 이 heap 을 안 만진다는 주장 (RT-1) 을 재는 sensor 는 세 개이고, 선택은 두 질문으로 정해진다: **(a) 측정 대상이 Eigen 을 쓰는가, (b) 측정 대상 코드가 테스트와 같은 TU 에 인스턴스화되는가.**

| 게이트 | 보는 것 | 못 보는 것 |
|---|---|---|
| `ScopedAllocGate` (`rtc_controllers/test/include/rtc_controllers/testing/alloc_gate.hpp`) — 전역 `operator new` 교체 | `operator new` 를 타는 모든 할당 (`std::vector`, header-inline helper, 다른 TU) | **Eigen 할당 전부** (`aligned_malloc` 이 `std::malloc` 직접 호출). **C 라이브러리 `malloc` 전부** (`libddsc` 의 `ddsrt_malloc` 이 tick 당 156 B 를 만든 사례 — `LD_PRELOAD` interposer 로 잰다) |
| `ScopedNoMalloc` (`rtc_base/test/include/rtc_base/testing/no_malloc_scope.hpp`) — `EIGEN_RUNTIME_NO_MALLOC` | Eigen 동적 할당 (Release 에서도 유효) | non-Eigen heap, **다른 TU 의 Eigen** (매크로는 정의된 TU 의 inline 만 계측) |
| `ScopedMallocGate` (`rtc_base/test/include/rtc_base/testing/malloc_gate.hpp`) — 실행 파일이 `malloc` 계열을 정의해 glibc `__libc_*` 로 넘기며 센다 | C 수준 할당 전부, **공유 라이브러리 안 포함** (Pinocchio `.so`, ProxQP, 다른 TU 의 Eigen, `operator new`) | glibc 가 아닌 C 라이브러리 (링크 실패로 드러남), `free` |

선택 규칙:

- **순수 Eigen 코어 (header-inline 법칙)** → 둘 다 무장한다. 하나만 쓰면 가장 유력한 회귀 (인자·임시가 `Eigen::VectorXd` 같은 runtime-sized 로 퇴화) 가 통과한다. `test_task_accel_core` / `test_task_vel_core` 가 이 형태.
- **Eigen-free 코어** → `ScopedAllocGate` 만. Eigen 트립와이어는 vacuous (`test_joint_pd_core`).
- **라이브러리 TU 에 컴파일된 `Compute()`** → `ScopedAllocGate` 만. Eigen 트립와이어는 관측 대상이 다른 TU 라 vacuous.
- **Pinocchio · ProxQP 를 부르는 코드** → `ScopedAllocGate` + `ScopedMallocGate`. 앞의 둘로는 라이브러리 안 할당이 보이지 않는다 — `test_catching_mpc_segment_core` 가 wrapper 경유 ProxQP `update()`·`solve()` 의 호출당 할당을 처음 잡았다 (operator-new 게이트는 같은 구간에서 0). positive control 은 **라이브러리 안** 할당 (`pinocchio::Data` 생성) 이어야 한다. 테스트 TU 의 할당은 operator-new 게이트가 이미 증명하는 것만 증명한다. 교체 `operator new` 와 한 TU 에 둘 수 있다.

**RAII 로 무장한다.** `g_alloc_active = true; … = false;` 같은 맨 대입은 측정 구역에 `ASSERT_*` 가 들어오는 순간 disarm 이 실행되지 않고, 이후 모든 테스트가 계수되는 상태가 조용히 남는다.

**mutation 으로 fail-closed 를 확인한다.** 측정 구역에 `std::vector<double>` (operator-new 게이트만 발동) 과 runtime-sized `Eigen::VectorXd` (Eigen 게이트만 발동) 를 각각 넣어 본다. `new` + 즉시 `delete` 쌍은 컴파일러가 elide 해 mutation 이 성립하지 않고, 외부 sink 로 포인터를 흘리는 것만으로도 부족하다 — gcc `-O3` 는 `malloc(상수)` + `memset` + `free` 를 volatile 전역에 포인터를 저장해도 통째로 지웠다 (계측기 self-test 가 그래서 "할당 0" 을 보고했다). **크기도 volatile 로 두고 `asm volatile("" ::: "memory")`** 를 건다. 확인은 `nm -D <binary> | grep -w malloc` — 참조가 0 이면 재려던 호출이 바이너리에 없다.

`alloc_gate.hpp` 는 교체 `operator new` 를 정의하므로 바이너리당 한 TU 에서만 include 한다 (두 번째는 링크 에러). 해당 타깃에는 `-Wno-mismatched-new-delete` 가 필요하다.

## Coverage 함정

- **도구.** runtime PC 에는 `lcov`/`gcovr` 가 없을 수 있다 (`gcov` 만 존재). gcovr 은 venv 에 `uv pip install gcovr` 로 설치한다 (`pip` 가 없는 `uv` 기반). `ENABLE_COVERAGE` 는 패키지 CMakeLists 마다 개별 정의이고, header-only 패키지는 `--filter "$SRC/include/"`, src 기반은 `--filter "$SRC/src/"`. 측정 전 `*.gcda` 를 지워 누적을 막는다. `gcov` 만 있으면 확장자 포함 `*.cpp.gcno` 를 직접 넘긴다.
- **정규 트리를 오염시키지 않는다.** coverage 빌드가 install 을 덮으면 downstream 이 gcov 심볼을 링크하게 되므로, 처음부터 scratchpad 의 별도 `--build-base` / `--install-base` 로 내보내고 `gcovr --object-directory` 도 그 트리를 가리킨다.
- **Python (`ament_python`) 은 `coverage`.** gcovr 은 C++ 전용이다. `cd <pkg> && python3 -m coverage run --source=<module> -m pytest test/ -q && python3 -m coverage report -m` 후 `.coverage` 를 지운다.
- **`rtc_mpc` 는 Debug 로 재면 안 된다.** Debug 가 proxsuite all-zero-C assert 를 표면화하고, aligator 0.19.0 의 contact_rich MPC 는 `LD_PRELOAD=$DEPS/install/lib/libmimalloc.so` 없이 `free(): invalid pointer` 로 죽는다 (`rtc_mpc/README.md`).
- **`integrated_bringup` 은 Debug 에서 `test_demo_task_controller` 가 abort 한다.** 의도적으로 non-finite Jacobian 을 주입하는 케이스가 회전행렬을 비유니터리로 만들어 pinocchio 의 `assert(R.isUnitary())` 가 발동한다 (Release 는 NDEBUG). 총계가 그 바이너리의 케이스 수만큼 줄고 gcda 도 안 남는다. 회귀가 아니니 판정은 Release 재실행으로 한다.
- **LifecycleNode 유닛 커버리지.** `on_configure` 와 private RT 헬퍼는 friend accessor (`rtc::ControllerLifecycleTestAccess`) 로 주입·호출해 실 robot / RT 권한 없이 구동한다.

## Live Debug 보충

- **sim 에는 `/forward_position_controller/commands` 가 없다.** sim 의 커맨드 lane 은 device group 당 하나 (`devices.<group>.backend.command_topic`) 라 그 침묵을 "RT loop 정지" 로 읽으면 오진이다.
- **`cm_timing_log.csv` 는 `run_id` 로 먼저 묶는다.** 세션 디렉토리가 분 해상도라 같은 분의 재기동이 같은 파일에 append 되고, 파일 전체로 `n / span` 을 내면 어느 런에도 없던 레이트가 나온다. `plot_rtc_log` 는 마지막 런을 자동 선택한다 (`--run-id` 로 변경). sim 모드의 `jitter_us` 는 CV wakeup 의 잡음이라 RT 지표가 아니고 항상 0.0 이다.
- **세 phase 열은 `t_total_us` 로 합산되지 않는다.** `t_total_us − (t_state + t_compute + t_publish)` 가 post-publish tail (뒤따르는 per-tick 작업 + 스레드가 돌지 못한 시간) 이다. 긴 tick 을 "publish 때문" 이라 읽기 전에 이 잔차부터 본다 — traced 최악 overrun 이 publish 28 µs + tail 3.4 ms 였는데, 잔차가 없던 CSV 는 그 3.4 ms 를 `t_publish_us` 에 얹어 "publish-dominated" 로 보고했다. 
- **`rt_callback_timing_log.csv`** 는 콜백당 1행, 같은 8열이지만 의미가 lane-specific 이다 (`t_state_us` = decode, `t_publish_us` = mailbox hand-off, `t_total_us` = slot 2 duty 분자). dispatch 간격은 연속 행 `t_wall_ns` 차분으로 본다. slot 2 가 aux 통합의 대상인데 계측이 없어서 만들었다.
- **관절 밴드 판독.** `joint band engaged:` WARN 은 2 s throttle 로 발동 tick 에서만 찍힌다. `on_deactivate` 요약이 tick 창을 주지만 Ctrl-C 로 끝내면 요약이 없다 (CM `main()` 이 lifecycle 훅을 안 탄다). 판독법은 `integrated_bringup/README.md` compliance 절.
