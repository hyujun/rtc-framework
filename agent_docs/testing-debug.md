# Testing & Debugging

테스트·검증의 **규범**이다 — 무엇을 반드시 돌리고, 결과를 무엇으로 판정하며, 어떤 green 을 믿지 않는가. 패키지별 sensor 표 · 명령 · 측정 레시피 · 함정이 발현한 사례 · 런타임 디버그 토픽은 헌법 밖 [docs/testing.md](../docs/testing.md) 가 갖는다 (같은 절 이름으로 찾는다).

## Sensor Matrix

- 변경한 위치의 행을 [docs/testing.md](../docs/testing.md) §Sensor Matrix 에서 찾아 **필수 sensor + 추가 sensor** 를 모두 실행한다 ([AGENTS.md](../AGENTS.md) §5). 패키지나 테스트 구성을 바꾸면 그 행을 같은 변경에서 갱신한다 (PROC-1).
- 테스트는 기본으로 빌드되지 않는다 — `./build.sh --tests [-p <pkg>]` 가 먼저다. 테스트 없이 빌드한 패키지는 실패가 아니라 "0 tests" 를 보고한다.
- `rtc_base` / `rtc_msgs` 변경은 전체 downstream 이다 (PROC-3).
- **skip 은 통과가 아니라 미검증이다.** 정책 파일 · optional dep · HW 가 있어야 도는 테스트가 skip 됐으면 그 사실을 보고한다.
- `robot_descriptions/` 의 MJCF / URDF 를 고쳤으면 `rtc_tools` 의 `test_real_model_pairs.py` 가 실제 게이트다. 로컬 colcon 은 pytest 를 시스템 python 으로 돌려 이를 skip 하므로 `.venv` python 으로 직접 돌린다.
- launch 를 고쳤으면 평가 센서 (`test_launch_description_evaluates`) 를 돌린다 — import 스모크로는 `OpaqueFunction` 본문이 실행되지 않는다.
- 컨트롤러·프로필·backend 를 추가했으면 per-X 공유 스위트가 그 이름을 아는지 확인한다 (AP-PROC-7).

## 테스트 설계

- **바인딩 테스트는 법칙을 재검증하지 않고 배선을 검증한다** — 코어 법칙은 그것을 소유한 패키지가 검증하고, 바인딩은 입력에서 출력까지의 배선 (부호 · 축 · 순서 · 전달) 에 정확한 oracle 을 붙인다.
- **oracle 은 검사 대상과 독립이어야 한다.** 자코비안 계약은 자코비안을 전혀 쓰지 않는 중심차분 oracle 로, 역산기는 원하는 물리량에서 입력을 **정방향으로** 조립한 oracle 로 검사한다 (역방향으로 쓰면 부호가 뒤집혀도 green 이다).
- **픽스처는 표현의 대칭을 깨야 판별력이 있다** — pinocchio order ≠ device order, 직교하지 않는 축, 0 이 아닌 offset, LOCAL ≠ LWA 인 프레임. 두 순서가 같은 픽스처에서는 뒤섞어도 통과한다. 픽스처의 기하를 고칠 때 그 비대칭을 유지한다.
- **positive control 은 합성 입력으로 만든다.** 지금 던지는 테스트나 코퍼스에 남은 실제 위반을 대조군으로 삼으면 그 결함이 고쳐질 때 대조군이 조용히 사라진다 — 게이트의 발화 증명은 `--self-test` 의 합성 코퍼스가 갖는다.
- 실세계 negative fixture (`schunk_svh_hand_*` 의 관성) 는 통과 대상이 아니다 — 값을 지어내 고치지 않는다.
- URDF → Model 진입점을 새로 열면 `EnforceInertialGate` 호출을 함께 넣는다.
- 추정·잔차 통계는 `valid=1` 행만 쓴다 (held 행은 직전 값이 동결된 것이라 측정이 아니다).

## Revert-verification — 새 가드를 추가했을 때

가드 (검증·거부 경로) 와 그 테스트를 함께 추가하면 **테스트 통과는 가드가 동작한다는 증거가 아니다**. 추가한 가드마다 **하나씩 원복 → 대응 테스트가 실제로 실패하는지 확인 → 복구** 한다. 원복해도 green 인 형태는 셋이다:

- **층이 겹치는 가드** — 다른 가드가 같은 입력을 이미 거부한다. 테스트를 "throw 하는가" 가 아니라 **그 가드만이 만드는 관측 가능한 차이** (진단 메시지, 거부 시점, 부작용) 로 옮긴다. 하나를 고쳤으면 대칭 위치의 나머지 절반 (publish ↔ subscribe 등) 도 같은 기준으로 다시 측정한다.
- **관측 채널이 fallback 에 가려진다** — assert 하는 값이 다른 경로로도 같은 값을 낸다. 관측을 fallback 이 건드리지 않는 채널로 옮긴다.
- **게이트가 닫힌 쪽에서 수치적으로 inert 하다** — 곱셈으로 꺼지는 게인·플래그 (`k = 0`, `α = 0`) 는 게이트를 지워도 같은 출력을 낸다. 출력이 아니라 게이트 자체를 드러내는 진단 플래그로 pin 하고, 양성 케이스를 함께 둔다.

원복 절차:

- 원복은 **파일 단위 restore** 로 되돌린다 — `git checkout -- .` 은 미커밋 작업까지 날린다. 검증 대상 파일 자체가 미커밋이면 명시적 백업 사본에서 복구한다.
- mtime 을 보존하는 복사 (`cp -p`, `shutil.copy2`) 로 복구하지 않는다 — make 가 재컴파일을 건너뛰어 원복된 바이너리가 남는다. 복구 뒤 반드시 `touch <파일>` 하고 재빌드하며, 빌드가 실제로 돌았는지 소요 시간으로 확인한다.

## Test fixtures — robot URDF 해석

- robot 모델이 필요한 fixture 는 URDF 를 **`robot_descriptions/robots/<name>/`** (repo 체크인) 에서 해석한다 — compile macro 또는 `ament_index_cpp::get_package_share_directory("robot_descriptions")`. `deps/src/...` 나 `/usr/local/...` 경로를 박지 않는다.
- fixture 는 **in-repo 패키지만** resolve 한다. 패키지 이름은 **리터럴로** 적는다 — 게이트 (`repo_scripts/scripts/validate_test_fixtures.py`) 는 비-리터럴 인자도 unresolvable 로 친다. 기존 fixture 를 재사용할 때는 그것이 무엇을 여는지부터 본다.
- 로컬 통과는 증거가 아니다. 검증은 `AMENT_PREFIX_PATH` 에서 그 prefix 를 빼고 바이너리를 돌리는 것이고, 격리가 걸렸다는 positive control 은 그 셸에서 `ros2 pkg prefix <pkg>` 가 실패하는 것이다.
- 다른 패키지를 `ament_index` 로 resolve 하는 테스트는 install set 이 다른 환경에서 깨진다 — 해석 경로 자체를 검증하는 것이 아니면 경로 헬퍼를 stub 한다.

## 비동기 결과를 기다리는 법

- 고정 sleep 대신 **관측 가능한 진행** (tick / solve / recv 카운터) 을 폴링한다. 공유 헬퍼는 `rtc::testing::WaitUntil` 이다.
- 이 헬퍼는 **오직 잔다**. Executor pump 가 필요한 테스트는 자기 TU 에 local spin 헬퍼를 둔다 ([invariants.md](invariants.md) PROC-8).
- suite 고유의 poll 예산은 헬퍼를 감싸지 말고 인자로 넘긴다. 한 곳에 묶어야 하면 **다른 이름** 으로 얇게 감싸고 값의 근거를 상수 옆에 남긴다.

## 컨트롤러 CSV 채널을 검정할 때

`Compute()` 를 루프로 도는 gtest 에는 production 의 drain 타이머가 없다 — log ring 이 차면 tail 을 잃고, 증상은 "행이 없다" 가 아니라 **값 불일치** 로 보인다.

- production 처럼 주기적으로 drain 한다 (N tick 마다 `log_set.DrainAll()`).
- 행 수 단언 옆에 `EXPECT_EQ(log_set.TotalDropCount(), 0U)` 를 둔다 — "잘렸다" 와 "push 가 안 됐다" 를 가르는 유일한 신호다.

## RT-1 zero-allocation 게이트

RT tick 이 heap 을 안 만진다는 주장을 재는 sensor 는 셋이고, 선택은 두 질문으로 결정된다: **(a) 측정 대상이 Eigen 을 쓰는가, (b) 측정 대상 코드가 테스트와 같은 TU 에 인스턴스화되는가.**

| 게이트 | 보는 것 | 못 보는 것 |
|---|---|---|
| `rtc::testing::ScopedAllocGate` — 전역 `operator new` 교체 | `operator new` 를 타는 모든 할당 (다른 TU 포함) | Eigen 할당 전부, C 라이브러리 `malloc` 전부 |
| `rtc::testing::ScopedNoMalloc` — `EIGEN_RUNTIME_NO_MALLOC` | 같은 TU 의 Eigen 동적 할당 | non-Eigen heap, 다른 TU 의 Eigen |
| `rtc::testing::ScopedMallocGate` — 실행 파일이 `malloc` 계열을 정의 | C 수준 할당 전부 (공유 라이브러리 안 포함) | glibc 가 아닌 C 라이브러리, `free` |

- **순수 Eigen 코어 (header-inline 법칙)** → `ScopedAllocGate` + `ScopedNoMalloc` 둘 다.
- **Eigen-free 코어** → `ScopedAllocGate` 만.
- **라이브러리 TU 에 컴파일된 코드** (컨트롤러 `Compute()`) → `ScopedAllocGate` 만 — Eigen 트립와이어는 다른 TU 를 못 본다.
- **Pinocchio · ProxQP 같은 라이브러리를 부르는 코드** → `ScopedAllocGate` + `ScopedMallocGate`. positive control 은 **라이브러리 안** 할당이어야 한다.
- **게이트는 RAII 로 무장한다** — 맨 대입은 측정 구역에 `ASSERT_*` 가 들어오는 순간 disarm 이 실행되지 않는다.
- **추가한 게이트는 mutation 으로 fail-closed 를 확인한다.** `new` + 즉시 `delete` 는 컴파일러가 지우고, 포인터를 외부 sink 로 흘리는 것만으로도 부족하다 — 크기를 volatile 로 두고 `asm volatile("" ::: "memory")` 를 건 뒤 `nm -D <binary>` 로 호출이 바이너리에 남았는지 확인한다.
- `alloc_gate.hpp` 는 교체 `operator new` 를 정의하므로 바이너리당 정확히 한 TU 에서만 include 한다.

## Test 측정

테스트 카운트·suite 목록은 박제하지 않는다 (AP-DOC-1) — 직접 측정한다. 비교 가능한 수치를 낼 때:

- 재측정 전에 `build/<pkg>/test_results` 의 누적 XML 을 지운다 — `colcon test-result` 는 남아 있는 XML 을 전부 합산하고, `colcon test` 가 돌지 않았어도 이전 결과로 답한다. 같은 출력에 `colcon test` 의 `Summary: … packages finished` 줄이 있을 때만 그 총계를 쓴다.
- `colcon test` 의 종료 코드는 테스트 실패를 말하지 않는다 — 종료 코드로 판정하려면 `--return-code-on-test-failure` 를 붙인다.
- `colcon test-result` 에는 `--packages-select` 가 없다. 패키지 하나는 `--test-result-base build/<pkg>` 로 본다.
- 회귀 비교는 **같은 범위 · 같은 단위** 로 한다 — `build/<pkg>` 와 `build/<pkg>/test_results` 는 총계가 다르고, 총계에는 gtest case · ctest entry · lint entry 가 섞여 있다. 옛 수치와 안 맞으면 stale 로 단정하기 전에 범위부터 맞춘다.
- `--test-result-base` 를 기본값이 아닌 곳으로 옮기지 않는다 (gtest XML 경로는 configure 시점에 박혀 따라오지 않는다) — cwd 는 `cd <rtc_ws> &&` 로 고정한다.
- **timeout · crash 는 gtest XML 에 `<failure>` 로 남지 않는다.** 판정은 `colcon test-result` 의 요약 (errors 포함) 이나 `build/<pkg>/Testing/*/Test.xml` 로 하고, gtest XML grep 을 유일한 센서로 쓰지 않는다.
- flake 는 재현하기 전에 `build/<pkg>/Testing/Temporary/` 의 `LastTest_*.log` 부터 연다.
- 한 gtest 바이너리가 ctest 에 여러 번 등록될 수 있다 — ctest 밖에서 돌릴 때는 `ctest -N` 으로 등록을 확인하고 `GTEST_FILTER` 를 등록대로 준다.
- 신규 테스트 개수는 총계 차이가 아니라 `grep -c '^TEST(' <파일>` 또는 per-target XML 의 `tests="N"` 으로 교차검증한다.

### Coverage · sanitizer 빌드

- coverage · sanitizer 빌드는 scratchpad 절대경로의 별도 `--build-base` / `--install-base` 로 내보내 정규 트리를 오염시키지 않는다 (호출은 여전히 ws root — AGENTS.md §9.1).
- Debug-only assert 가 coverage 빌드에서만 표면화한 실패는 회귀가 아니다 — 판정은 Release 재실행으로 한다.

### 테스트 격리

- **노드를 만드는 테스트는 전용 `ROS_DOMAIN_ID` 로 격리한다** — 배정 단위는 패키지 하나에 도메인 하나이고 값은 literal 이다. 번호를 손으로 고르지 않는다: 현재 배정과 충돌 검사는 `python3 repo_scripts/scripts/validate_test_domains.py --list` 가 소스에서 파생한다. `ament_python` 패키지는 `test/conftest.py` 의 `os.environ["ROS_DOMAIN_ID"]` 로 주장한다.
- **패키지 안 병렬** (`colcon.pkg` 의 `ctest-args: ["-j", n]`) 을 켠 패키지는 `ROS_DOMAIN_ID` 를 주장하는 테스트 전부를 `RESOURCE_LOCK ros_domain_<n>` 에 올리고 (게이트가 차단), 측정한 wall-clock 을 단언하는 테스트는 `RUN_SERIAL TRUE` 로 혼자 돌린다 (게이트가 못 본다 — 테스트를 추가하는 사람이 판정). 기록만 하는 duration 을 인용할 때는 단독 실행 값을 쓴다. participant 를 여는 python 패키지는 xdist worker 를 요청할 수 없다.
- **컨트롤러를 configure 하는 테스트는 세션 디렉토리도 격리한다** (`RTC_SESSION_DIR`) — 격리하지 않으면 테스트 행이 워크스페이스의 실제 `logging_data/` 세션에 섞인다.
- `.venv` 격리는 AGENTS.md §9.2. 그 격리의 false-green 방향: `colcon test` 의 pytest 는 시스템 python 으로 돌아 `.venv` 전용 패키지에 의존하는 테스트가 **조용히 skip** 된다. CI green 을 근거로 로컬 skip 을 무시하지 않고, 자작 게이트에는 "검사가 아예 안 돌았음" 을 통과와 구분하는 플래그를 둔다.

## 런타임 판독

토픽·CSV 컬럼·증상별 처방은 [docs/testing.md](../docs/testing.md) §Live Debug Topics · §Debugging. 판독할 때:

- timing CSV 는 **`run_id` 로 먼저 그룹핑** 한다 — 같은 분의 재기동이 같은 파일에 append 된다.
- sim 모드 (`use_sim_time_sync=true`) 의 `jitter_us` 는 RT 지표가 아니다.
- timing CSV 의 phase 열은 `t_total_us` 로 합산되지 않는다. 긴 tick 을 특정 phase 탓으로 읽기 전에 **잔차** (`t_total_us − phase 합`) 부터 본다.
- sim 의 커맨드 lane 은 device group 당 하나다 (`devices.<group>.backend.command_topic`). robot 모드 전용 토픽의 침묵을 sim 에서 "RT loop 정지" 로 읽지 않는다.
- CPU shield 격리가 서는지는 **실기 (SMT / hybrid 호스트)** 에서만 검증된다 — sim 단일 실행으로 대체할 수 없고, cpuset 비교는 부분집합이 아니라 동일성으로 한다.

## Tracing

timing CSV 는 per-tick 총 시간만 기록한다. 어느 thread 가 어느 core 에서 언제 돌았는지, 어떤 callback 이 시간을 쓰는지가 필요하면 LTTng trace 를 캡처한다 — 절차는 [docs/tracing.md](../docs/tracing.md).
