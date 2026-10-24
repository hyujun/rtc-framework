# Testing — 근거·실측·사례

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/testing-debug.md](../../agent_docs/testing-debug.md) 가 갖고, 실행 방법은 [docs/testing.md](../testing.md) 가 갖는다. 여기는 그 규칙이 *왜* 생겼는지, 함정이 실제로 어떻게 발현했는지, 무엇을 실측했는지를 기록한다. 둘이 어긋나면 testing-debug.md 가 옳다. 수치·테스트 이름은 **기록 시점의 것**이며 다시 재야 한다.

## Sensor Matrix 보충

테스트별 "무엇을 판별하는가" 는 각 테스트 파일의 헤더 주석이 갖는다. 공통 원칙은 하나다: **fixture 는 판별하려는 차이를 일부러 비대칭으로 만들고, 그 비대칭이 깨지면 스스로 알린다.** 대칭 fixture 는 틀린 구현도 통과시킨다.

- **sanitizer 빌드** — 전용 수단이 없어 별도 `--build-base` / `--install-base` 에 `RelWithDebInfo` + `-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer` 로 빌드하고 바이너리를 직접 실행한다. `ScopedAllocGate` TU 는 `new`→`malloc` 교체가 alloc-dealloc-mismatch 를 내므로 그 TU 만 `ASAN_OPTIONS=alloc_dealloc_mismatch=0` 으로 돌린다.
- **`malloc_gate.hpp` TU 는 ASan 아래에서 실행할 수 없다** — 실행 파일이 `malloc` 계열을 정의해 ASan 할당기와 섞인다. 그 경로의 sanitizer 판정은 gate 없는 짝 테스트 (`test_clik_multiframe`) 가 대신한다.
- 로컬 colcon 의 pytest 는 `/usr/bin/python3` 라 mujoco 를 못 봐 `rtc_tools` 의 `test_real_model_pairs.py` 가 skip 된다. `test_demo_inference_real_model` 도 `RTC_POLICY_DIR` 이 없으면 전 케이스 skip 이다 — skip 은 미검증이다.
- 예: import 스모크 (`test_launch_imports.py`) 는 `OpaqueFunction` 본문이 안 돌아, sim 세 개가 죽은 채 전 배터리가 green 이었던 적이 있다 (#397, `test_launch_description_evaluates` 가 닫음).

## Revert-verification

가드와 그 테스트를 함께 추가하면 **테스트 통과는 가드가 동작한다는 증거가 아니다.** 절차는 [testing-debug.md](../../agent_docs/testing-debug.md) §Revert-verification. false green 은 세 가지 형태로 관측됐다.

**1. 층이 겹치는 가드.** 가드 A·B 가 같은 입력을 거부하면 B 의 테스트는 A 에 흡수되어 B 를 지워도 통과한다. 실측: 신규 가드 4개 중 flat `publish:` 탐지가 false green 이었다 (group-shape 가드가 같은 config 을 이미 거부). 테스트를 "마이그레이션 진단을 주는가" 로 옮겨 pin 했다. 형제 절반 (`subscribe` 쪽) 에서 같은 false green 이 재발했다.

**2. 관측 채널이 fallback 에 가려질 때.** E-STOP recovery drain: OSC/TaskImpedance 는 gravity-comp 컨트롤러라 정지 시 hold torque ≈ ĝ(q) 인데, leak 된 target 이 멀면 recovery tick 에서 SAFE_STOP 이 latch 되고 그 hold 도 ĝ(q) 를 낸다. joint torque 로 assert 하면 leak 유무와 무관하게 통과한다. 관측을 fallback 이 건드리지 않는 채널로 옮겼다 — OSC 는 goal echo, TaskImpedance 는 SAFE_STOP step *이전에* 기록되는 `diag.pose_error`.

**3. 게이트가 닫힌 쪽에서 수치적으로 inert 일 때.** `nullspace_active = (nv > 6) && (nullspace_kp != 0.0)` 에서 뒷항을 지워도 비트 레인이 전부 green 이었다. `nullspace_kp = 0` 이면 게이트 없는 경로가 `0.0·Δq` 의 signed zero 를 만들고 그것을 사영해 더하는 것은 exact no-op 이기 때문이다. 곱셈으로 꺼지는 게인·플래그 (`k = 0`, `α = 0`) 는 전부 이 형태다.

**원복·복구의 함정.** `git checkout -- .` 은 아직 커밋하지 않은 작업까지 날린다 (실제 발생). 미커밋 대상을 백업 사본에서 복구할 때 **mtime 을 보존하는 복사 (`cp -p`, `shutil.copy2`) 는 make 가 재컴파일을 건너뛰게 한다.** 소스는 최신인데 빌드 트리에는 원복된 바이너리가 남아 결과를 반대로 읽는다 ("2 failures" 를 회귀로 오독한 사례). 재컴파일이 돌았는지는 빌드 소요 시간으로 본다 — 0.x 초면 안 돈 것이다.

## Test fixtures

규칙은 [testing-debug.md](../../agent_docs/testing-debug.md) §Test fixtures. **repo 밖 패키지를 resolve 하는 fixture 는 clean 환경에서 전부 red 인데 그게 안 보인다.** 사건: ur5e_p1b fixture 가 `hand_description` 패키지를 찾았는데, 그 패키지는 이 repo 에도 `deps.repos` 에도 없어 (로컬 ws 에만 있었다) CI 에서 model-backed 케이스가 전부 던졌다. best-effort 잡이라 PR 은 green 이었고 유일한 증상인 codecov patch % 는 "테스트를 안 썼다" 로 오독됐다 (observer 배선 테스트 14개 중 13개 red). 모델을 `robot_descriptions/robots/ur5e_p1b/` 로 vendor 해 닫았다. 그 C++ CI 는 이후 제거돼 `validate_test_fixtures.py` 가 clean 환경을 재현하는 유일한 방어선이다.

- **skip 허용은 없앴다 (#952).** 게이트는 처음에 "없으면 skip 하면 허용" 이었고, vendor 할 수 없던 G1 모델의 catching 테스트 11개가 그 guard 로 clean 환경에서 조용히 skip 됐다. 그리고 vendor 한 p1b 사본은 공개하면 안 되는 손 파일이었다. 둘 다 합성 대리 손 (`robots/p1b_surrogate/`, 이름 · loop 구조만 같고 치수 · 관성은 지어낸 값) 으로 바꾸고 guard 를 금지했다 — vendor 도 skip 도 안 되는 모델에 남는 길은 대리 모델뿐이다. 이때 게이트 (리터럴 · C++) 가 못 보는 경로가 넷 더 나왔다 — 출하 프로파일 디렉토리를 도구에 넘기면 도구가 그 `urdf.package` 를 resolve 하는 Python 테스트다. 격리 셸 (`AMENT_PREFIX_PATH` 에서 prefix 를 뺀 colcon · venv 두 lane) 만이 그것을 잡았다.

- **리터럴 스캔은 부족했다.** 파라미터화 fixture 가 `get_package_share_directory(ec.urdf_pkg)` 처럼 struct 필드 경유로 열어 8곳 중 2곳만 잡혔다. 그래서 비-리터럴 인자도 unresolvable 로 치고, fixture 가 CMake source list 밖 헤더에 살기 때문에 include 를 따라가 그 헤더에 도달하는 모든 test `.cpp` 에 위반을 부과한다.
- **발화 증명은 코퍼스에 의존하지 않는다.** vendor 로 위반 코퍼스가 사라진 뒤 발화 증명은 `--self-test` 의 합성 코퍼스로 옮겼다. 실제 코퍼스를 positive control 로 쓰면 "위반 하나를 영원히 살려둬라" 가 된다. 격리 검증의 대조군도 같은 이유로, `AMENT_PREFIX_PATH` 에서 prefix 를 뺀 셸에서 `ros2 pkg prefix <pkg>` 가 실패하는 것이다.
- **install set ≠ local.** Python 테스트 잡은 `rtc_tools` / `rtc_msgs` / `rtc_digital_twin` 만 install 했다. 다른 패키지를 `ament_index` 로 resolve 하는 `rtc_tools` 테스트는 CI 에서만 `PackageNotFoundError` 가 났다.

## ROS_DOMAIN_ID 격리와 병렬

`colcon test` 는 패키지를 병렬 실행하므로, 노드를 만드는 테스트가 기본 도메인 0 을 공유하면 discovery 버스와 Fast DDS SHM port 객체를 함께 쓴다. 피해는 두 층이다. (1) 타 패키지의 pub/sub 매칭 테스트가 깨진다 (`ur5e_bt_coordinator` 의 `test_rewire_gate` 가 피해자). (2) 심하면 rmw endpoint 생성·소멸에서 멈춘다 — ctest 가 60 s 에 죽여 결과 파일이 안 남고, 부하 하 재현율 7/18, 격리 후 0 이었다. 전 스레드 `futex_do_wait` 정지라 timeout 상향으로 없어지지 않는다.

- **손으로 번호를 고르는 방식은 실패했다.** `udp_hand_driver` 가 "44-52 are taken" 주석을 근거로 53 을 골랐는데 `integrated_bringup` 이 이미 쓰고 있었다. 지금 배정은 `validate_test_domains.py --list` 가 소스에서 파생한다.
- **"아무것도 주장하지 않은" 테스트도 잡는다.** 예전 게이트는 주장된 번호의 중복만 봐서, 노드를 만드는 타깃 15개가 5개 패키지에 걸쳐 도메인 0 에 남아 있었다.
- **패키지 안 병렬 (`colcon.pkg` `-j`).** 실측 (6C/12T, `-j 4`): `rtc_controllers` 48 s → 23 s, `integrated_bringup` 61 s → 27 s. 켜면 "패키지 안은 직렬" 전제가 사라진다 — 그에 따른 규칙은 [testing-debug.md](../../agent_docs/testing-debug.md) §테스트 격리. `ament_python` 의 pytest-xdist 는 `rtc_tools` 105 s → 37 s (설정은 `rtc_tools/colcon.pkg` 주석), 직렬 복귀는 [docs/testing.md](../testing.md#test-commands).

## 세션 디렉토리 격리

출하 YAML 의 `logs:` 블록을 가진 컨트롤러는 `on_configure` 에서 CSV 를 열고, `RTC_SESSION_DIR` 이 없으면 세션 resolver 가 workspace 의 **실제 `logging_data/<YYMMDD_HHMM>`** 로 떨어진다. 테스트 행이 운영자 세션 사이에 남고 같은 분에 띄운 sim 실험 세션과 섞인다 (Stop hook 의 `colcon test` 가 실험 세션 하나의 diag 헤더를 오염시킨 사례). 세션을 쓰는 테스트가 있는 패키지 (`integrated_bringup` · `rtc_controller_manager` · `rtc_controller_interface` · `rtc_mpc` · `udp_hand_driver`) 는 CMakeLists 의 test 블록 끝에서 `rtc_base_isolate_test_sessions()` 를 불러 Environment 등록 TU 를 모든 `test_*` gtest 실행 파일에 넣어 이를 구조로 강제한다 — 파일마다 등록하던 방식은 7개 바이너리가 빠뜨렸다. Environment 본체는 `rtc_base/testing/isolated_session_dir.hpp` 하나다 (#932). CM 은 `enable_logging` 기본값이 true 이고 configure 가 세션의 **부모**에서 retention 을 돌리므로, 세션을 `<mkdtemp>/logging_data/session` 으로 한 단계 내려 retention 이 공용 `/tmp` 가 아니라 바이너리 전용 루트를 훑게 한다. 테스트 하나가 빈 세션을 따로 가져야 하면 같은 헤더의 `ScopedSessionDir` 가 그 루트 아래 `<tag>_<n>` 을 만들고 소멸 때 이전 값을 되돌린다 — 손으로 `setenv("RTC_SESSION_DIR")` 하던 5개 파일은 공용 `/tmp` 에 디렉토리를 만들었다 (#935).

## `.venv` 격리의 false-green

규칙은 [AGENTS.md](../../AGENTS.md) §9.2. 이 격리에는 **로컬 전용 false-green 방향**이 있다: `colcon` 과 colcon 이 생성하는 console script 의 shebang 이 `/usr/bin/python3` 라, venv 를 activate 해도 `colcon test` 의 pytest 는 시스템 python 으로 돌아 `.venv` 전용 패키지를 import 못 하고 (`pytest.importorskip` 테스트가 **조용히 skip** — `rtc_tools` 의 mujoco 의존 11개 전부), `ros2 run` 도 venv 를 못 봐 `.venv/bin/python` 직접 실행과 다른 답을 낸다. 실제 인터프리터는 `log/latest_test/<pkg>/command.log` 가 확정한다. CI 는 venv 없이 `pip install` 이라 영향이 없으므로 방심 방향이 반대다 — CI green 을 근거로 로컬 skip 을 무시하지 않는다. 신호는 `Testing/Temporary/LastTest.log` 의 Start/End 동일 초다. 확인용 직접 실행 (명령은 [docs/testing.md](../testing.md#test-commands); `PYTHONPATH=rtc_tools` 를 앞에 두는 이유는 repo 루트에서 바깥 `rtc_tools/` 디렉토리가 namespace package 로 잡혀 안쪽 `rtc_tools` 패키지를 가리기 때문이고, `rtc_tools.analysis` 를 import 하는 다른 패키지의 테스트도 같은 값이 필요하다) 은 우회 금지 하의 진단 용도이며, 자작 게이트에는 "검사가 아예 안 돌았음" 을 통과와 구분하는 플래그를 둔다.

## 계측 함정

비교 가능한 수치를 낼 때 필수다. 각 항은 실제로 잘못된 보고를 낳은 경로다 (측정 규칙은 [testing-debug.md](../../agent_docs/testing-debug.md) §Test 측정).

| 실측 | 원인 |
|---|---|
| baseline 298 을 300 으로 오측 | `colcon test-result` 는 남은 XML 을 전부 합산한다 — 빼거나 되돌린 타깃의 옛 결과가 더해진다. `build/<pkg>/test_results` 를 먼저 지운다 |
| `1179 tests, 0 failures` | `lint && build && test; test-result` 체인에서 무관한 `ruff check` 실패가 빌드·테스트를 건너뛰었는데 `test-result` 는 이전 실행을 요약했다. 같은 출력에 `Summary: … packages finished` 가 있는지 본다 |
| 실패 패키지가 green 으로 기록 | `colcon test` 종료 코드는 테스트 실패를 말하지 않는다. `colcon test-result` 에는 `--packages-select` 가 없어 usage 오류 (exit 2) 만 나는데 그 문구에 "N failures" 가 없어 정규식 검사가 **실패 없음** 으로 읽었다 (Stop hook 판정 사례). 패키지 하나는 `--test-result-base build/<pkg>` |
| `integrated_bringup` 639 / 599 | `build/<pkg>` 는 CTest 의 `Testing/<stamp>/Test.xml` (실행마다 누적) 까지 합산, `build/<pkg>/test_results` 는 gtest/pytest/lint XML 만 센다. 둘 다 맞고 단위가 다르다 |
| 189 대신 20 | 기본값 아닌 result-base 를 주면 gtest XML 경로가 configure 시점에 박혀 있어 총계가 조용히 바이너리 수로 줄어든다 |
| `rtc_controllers 333` = gtest 315 + ctest 18 | 테스트 `.cpp` 하나를 추가하면 총계는 케이스 수 + 1 (ctest 타깃) + 1 (cppcheck, 전부 skipped) 오른다. 소스의 `TEST(` 개수나 per-target XML 의 `tests="N"` 으로 교차검증한다 |
| `<failure>` 0건인데 timeout | 22패키지 병렬에서 `test_condition_nodes` 가 60 s CTest timeout 을 냈다 (단독 재실행 0.47 s). 타임아웃난 바이너리는 XML 을 못 쓰거나 부분만 써서 `<failure>` 로 훑는 검사는 clean 이다. `Test time = 60.0x sec` 는 crash 가 아니라 `ament_add_test` 기본 TIMEOUT 이다 |
| 추가되고 commit 7개 뒤에야 등록된 test 파일 | 등록되지 않은 `test/test_*` 는 빌드를 통과하고 직접 돌려도 통과하며 어떤 수치에도 나타나지 않는다 (`validate_test_registration.py` 가 차단, 토큰 읽기라 거짓인 `if()` 안의 등록은 통과) |

- **flake 는 재현 전에 `build/<pkg>/Testing/Temporary/` 부터 연다.** ctest 는 `LastTest_<UTC stamp>.log` 를 지우지 않아 몇 주 전 실패가 로그째 있다. 마지막 `[ RUN ]` 과 마지막 로그 줄이 멈춘 지점을 가른다.
- **한 gtest 바이너리가 ctest 에 여러 번 등록될 수 있다** (`GTEST_FILTER`). 맨손으로 돌리면 두 등록의 케이스가 한 프로세스에 섞여 코드가 아닌 실행 방식이 만든 red 가 난다. 등록 형태는 `integrated_bringup/CMakeLists.txt` 의 인라인 주석이 SSoT.

## RT-1 zero-allocation 게이트

선택 규칙·무장 방식은 [testing-debug.md](../../agent_docs/testing-debug.md) §RT-1 zero-allocation 게이트. 게이트마다 못 보는 것이 있다는 것을 아래 사건이 보였다.

- **`libddsc` 156 B/tick.** 전역 `operator new` 교체 게이트와 Eigen 게이트는 C 라이브러리 `malloc` (`ddsrt_malloc`) 을 못 본다. tick 당 156 B 는 `LD_PRELOAD` interposer 로 잡았고, 이후 `malloc_gate.hpp` 가 공유 라이브러리 안까지 센다.
- **ProxQP 는 malloc 게이트가 처음 잡았다.** `test_catching_mpc_segment_core` 에서 wrapper 경유 ProxQP `update()`·`solve()` 의 호출당 할당을 `ScopedMallocGate` 가 잡았다 (operator-new 게이트는 같은 구간에서 0).
- **gcc `-O3` 가 mutation 을 지웠다.** `malloc(상수)` + `memset` + `free` 는 volatile 전역에 포인터를 저장해도 통째로 사라졌고, 계측기 self-test 가 그래서 "할당 0" 을 보고했다. 크기도 volatile 로 두고 `asm volatile("" ::: "memory")` 를 건다. 확인은 `nm -D <binary> | grep -w malloc` — 참조가 0 이면 재려던 호출이 바이너리에 없다.

## CLIK 제동 거리 한계의 실측

`test_clik_multiframe` (17-DoF tree) 에서 `Config::brake_from_torque` 를 켜고 끈 비교다. 5 관절을 위치 한계 사이로 1000 tick 왕복하면 끄면 실패 4 회, 켜면 0 회. 한 관절만 풀고 나머지를 잠근 접근 20 회에서는 끄면 한계 직전의 실패 132 회, 켜면 `brake_margin` 1.0 · 0.9 · 0.8 모두 0 회. 한계에서 0.15 – 0.2 rad 떨어진 가속 → 제동 전환 tick 의 실패는 `brake_margin` 1.0 에서 2 회, 0.9 에서 0 회, 0.8 에서 1 회 남는다. 앞의 것은 상태에 따라 `a` 가 변하는 효과이고, 뒤의 것은 그 fixture 에서 토크 행이 자기 box 행과 평행해 ProxQP 가 실행 가능한 문제에 PRIMAL_INFEASIBLE 을 내는 것이다 (#654). 식과 휴리스틱의 한계는 `clik_reference.hpp` 머리 주석.

## Coverage 함정

- **도구.** runtime PC 에는 `lcov`/`gcovr` 가 없을 수 있다 (`gcov` 만 존재). gcovr 은 venv 에 `uv pip install gcovr` 로 설치한다. `ENABLE_COVERAGE` 는 패키지 CMakeLists 마다 개별 정의이고, header-only 패키지는 `--filter "$SRC/include/"`, src 기반은 `--filter "$SRC/src/"`. 측정 전 `*.gcda` 를 지운다. `gcov` 만 있으면 확장자 포함 `*.cpp.gcno` 를 직접 넘긴다. 빌드 트리 분리는 [testing-debug.md](../../agent_docs/testing-debug.md) §Coverage · sanitizer 빌드.
- **Python (`ament_python`) 은 `coverage`.** gcovr 은 C++ 전용이다. `cd <pkg> && python3 -m coverage run --source=<module> -m pytest test/ -q && python3 -m coverage report -m` 후 `.coverage` 를 지운다.
- **`rtc_mpc` 는 Debug 로 재면 안 된다.** Debug 가 proxsuite all-zero-C assert 를 표면화하고, aligator 0.19.0 의 contact_rich MPC 는 `LD_PRELOAD=$DEPS/install/lib/libmimalloc.so` 없이 `free(): invalid pointer` 로 죽는다 ([rtc_mpc/README.md](../../rtc_mpc/README.md) §문제 해결).
- **`integrated_bringup` 은 Debug 에서 `test_demo_task_controller` 가 abort 한다.** 의도적으로 non-finite Jacobian 을 주입하는 케이스가 회전행렬을 비유니터리로 만들어 pinocchio 의 `assert(R.isUnitary())` 가 발동한다 (Release 는 NDEBUG). 총계가 그 바이너리의 케이스 수만큼 줄고 gcda 도 안 남는다. 회귀가 아니니 판정은 Release 재실행으로 한다.
- **LifecycleNode 유닛 커버리지.** `on_configure` 와 private RT 헬퍼는 friend accessor (`rtc::ControllerLifecycleTestAccess`) 로 주입·호출해 실 robot / RT 권한 없이 구동한다.
