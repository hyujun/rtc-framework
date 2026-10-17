# 빌드 병렬도·메모리·시간 — 근거·실측

> **이 문서는 헌법이 아니다.** 사용자가 쓰는 knob 과 증상 감별은 [repo_scripts/README.md](../../repo_scripts/README.md) "빌드 병렬도와 메모리" · "빌드 시간 줄이기" · "흔한 실패와 감별" 이 갖고, colcon cwd · `.venv` 규칙은 [AGENTS.md](../../AGENTS.md) §9 가 갖는다. 여기는 그 기본값이 *왜* 이런 모양인지 · 무엇을 재서 정했는지를 기록한다. 둘이 어긋나면 README 와 AGENTS.md 가 옳다. 수치는 **기록 시점의 것** (6C/12T · 32 GB · Release, 이 repo 의 21개 패키지 클린 빌드) 이고 인용하기 전에 다시 잰다.

## colcon 기본 병렬도가 호스트를 세우는 이유

colcon 의 병렬도는 두 층이다 — 동시에 빌드하는 **패키지 수** (`--parallel-workers`) 와 패키지 안의 **make job 수**. colcon-cmake 는 `MAKEFLAGS` 에 `-j`/`-l` 이 없으면 패키지마다 `-j<논리 코어> -l<논리 코어>` 를 붙이고, `-l` 은 loadavg 만 볼 뿐 메모리는 보지 않는다. 그래서 colcon 기본값의 동시 컴파일 수 상한은 *패키지 수 × 논리 코어* 이고, 코어가 많고 RAM 이 그만큼 크지 않은 호스트는 그것만으로 메모리가 바닥난다 (물리 16코어 / 32 GB 호스트가 빌드 중 섰다). 이 repo 가 패키지를 한 번에 하나씩 (`.colcon/defaults.yaml` 의 `parallel-workers: 1`) 빌드하고 병렬도를 make job 수 하나로 정하는 출발점이다.

## 기본값 `min(물리 코어, RAM / 4 GB)` 의 실측

컴파일러·링커 RSS 합의 최대:

| 패키지 × make job | 시간 | 빌드 메모리 최대 |
|---|---|---|
| 1 × `-j6` | 18분 49초 | 11.0 GB |
| 2 × `-j3` | 20분 51초 | 8.7 GB |
| 1 × `-j8` | 17분 23초 | 14.4 GB |
| 2 × `-j4` | 17분 11초 | 11.5 GB |

- **패키지를 나누지 않는 이유**: 같은 예산을 두 패키지로 나눠도 클린 빌드는 빨라지지 않고 (무거운 패키지들이 의존성 사슬로 이어져 혼자 빌드되는 구간이 길다), 평소 하는 패키지 하나 재빌드는 job 이 절반이라 훨씬 느리다 (`integrated_bringup`: 1×`-j6` 5분 18초, 2×`-j3` 8분 30초).
- **SMT 형제를 세지 않는 이유**: `-j8` 은 `-j6` 보다 8% 빠르고 31% 더 쓴다.
- **job 당 4 GB 인 이유**: TU 하나의 컴파일러 메모리는 대부분 2 GB 아래지만 2 GB 를 넘는 TU 가 51개, 최악은 3.5 GB 다 (`rtc_urdf_bridge/src/rt_model_handle.cpp`). 비용은 최적화가 아니라 템플릿 인스턴스화에서 나온다 — `-fsyntax-only` 만으로도 메모리의 90% 이상이 든다 — 그래서 `-O` 수준을 낮춰도 줄지 않는다.
- **`-l` 을 넣지 않는 이유**: load 제한은 빌드와 무관한 시스템 부하까지 예산에서 깎는다 (1×`-j6 -l6` 은 20분 44초).
- **`--no-warn-unused-cli`**: `build.sh` 는 한 인자 묶음을 모든 패키지에 넘기므로 C/C++ target 이 없는 패키지 (`repo_scripts` · `robot_descriptions`) 는 reconfigure 마다 `Manually-specified variables were not used` 를 냈다. 구조적으로 잡음이라 끈다.
- **knob 을 빌드 전에 전부 검증하는 이유**: `build.sh` 는 `-c` 가 트리를 지우고 CPU shield 를 풀기 **전에** 모든 knob (`RTC_BUILD_TESTS` · `RTC_CCACHE` · `RTC_BUILD_JOBS` · `RTC_BUILD_MEM_MAX`) 을 해석한다 — 쓸 수 없는 값이 설치본 없는 워크스페이스를 남기지 않게.
- **패키지 하나 재빌드의 비용** (`.claude/hooks/verify-changes.sh --run` 이 치르는 경로, `-p integrated_bringup --tests`): 변경 없음 3초, TU 하나 수정 24초 (`-j6` 과 `-j12` 가 같다 — 링크가 지배한다), 패키지 전체 재컴파일 279초 (`-j12` 는 245초). 마지막 경우는 Stop hook 이 turn 끝에서 빌드하던 때의 180초 상한을 넘었다 — 그래서 빌드·테스트를 turn 끝에서 빼고 `--run` 으로 옮겼고, 그 상한은 900초다 (hook 헤더 Limits).

## 메모리 상한의 설계

job 수는 평균을 맞추는 장치다 — job 당 4 GB 는 최악 TU 하나가 들어가는 값이지, 모든 job 이 동시에 최악인 경우의 합 (8 × 3.5 GB = 28 GB) 이 아니고, 빌드 밖에서 메모리를 쓰는 프로그램도 계산에 없다. 그 꼬리는 `MemoryMax` 가 걸린 systemd user scope 로 막는다 (`rt_common.sh` `build_mem_scope_prefix`).

- **기본 RAM 상한이 두 값 중 작은 쪽인 이유**: RAM 의 75%, 그리고 빌드를 시작하는 순간 비어 있는 메모리 (`MemAvailable`) 의 90%. sim · IDE · 브라우저가 이미 10 GB 를 쓰는 32 GB 호스트에서 "RAM 의 75%" 는 빌드에 없는 24 GB 를 약속한다 — scope 한도에 닿기 전에 호스트가 먼저 swap 에 빠진다.
- **swap 을 RAM 의 25% 까지 허용하는 이유**: 0 으로 막으면 RAM 이 작은 호스트의 기본 빌드가 성립하지 않는다 — 4 GB 호스트는 상한이 3 GB 인데 최악 TU 가 3.5 GB 다. RAM 상한 + swap 허용 = RAM 전체가 job 산정식이 가정한 예산 (job × 4 GB) 과 맞는다.
- **`RTC_BUILD_MEM_MAX` 를 직접 주면 swap 이 0 인 이유**: 사용자가 준 값이 전부여야 한다. 단위 없는 수는 systemd 에서 바이트이므로 (`24` = 24 바이트) 거부하고, K 단위 · 100% 초과 · 64M 미만도 거부한다 (이전에는 통과해 probe 가 죽고 "user session 없음" 으로 오진된 채 상한 없이 빌드됐다).
- **plain `colcon` 에 걸지 않는 이유**: scope 로 감싸는 것은 `build.sh` · `build_deps.sh` 의 몫이다. 같은 보호가 필요하면 `systemd-run --user --scope -p MemoryMax=75% -p MemorySwapMax=8G colcon build …` 로 직접 감싼다 (`MemoryMax` 의 % 는 RAM 대비지만 `MemorySwapMax` 의 % 는 **swap 크기** 대비다 — 크기로 적는다).
- **상한을 걸 수 없는 호스트**: 이유를 알리고 상한 없이 빌드한다 (job 수 제한은 그대로). 이유는 둘로 가른다 — `no systemd user session` (컨테이너 · CI · lingering 없는 ssh) 과 `this host does not enforce it` (scope 는 만들어지지만 한도가 안 걸림: cgroup v1/hybrid, memory controller 미위임).

### OOM 판정이 systemd 버전에 따라 다른 이유

한도를 넘으면 그 빌드만 실패하고, `build.sh` 는 컴파일 에러를 메모리 탓으로 돌리지 않는다 (`build_mem_scope_end`). 판정 근거가 둘이다.

- systemd ≥ 253 (Ubuntu 24.04): scope 의 `OOMPolicy=stop` 이 프로세스 하나가 죽는 순간 scope 전체 (colcon · make · 컴파일러) 를 정리하고 unit 의 `Result` 가 `oom-kill` 이 된다.
- 그 이전 (Ubuntu 22.04 는 249): scope 에 `OOMPolicy` 가 없다 — 죽은 컴파일러 하나 때문에 make 가 `Killed signal terminated program cc1plus` 로 실패하고 scope 는 스스로 끝난다. `Result` 는 `oom-kill` 이 되지 않으므로 scope cgroup 의 `memory.events` (`oom_kill`) 로 판정한다.

22.04 호스트에서 돌려 보지는 못했다 — 24.04 에서 `OOMPolicy=continue` 로 같은 동작을 만들어 확인했다. 쓰는 옵션 (`MemoryMax` · `MemorySwapMax` · `--collect` · `systemctl show --value`) 은 249 에도 있다.

## 테스트를 기본으로 빌드하지 않는 이유

컴파일 시간의 절반 이상이 테스트다 — 메시지 패키지를 뺀 597개 TU 중 362개가 테스트 코드이고 컴파일 시간의 58% 를 차지한다 (`integrated_bringup` 은 156개 중 115개).

| 클린 빌드 (6C/12T, 1 × `-j6`) | 시간 | 빌드 메모리 최대 |
|---|---|---|
| 테스트 포함 (`--tests`) | 18분 49초 | 11.0 GB |
| 기본 | 9분 44초 | 10.7 GB |

메모리 최대는 줄지 않는다 — 가장 무거운 TU 들은 제품 코드 쪽에 있다.

**`-DBUILD_TESTING` 을 매번 명시하는 이유**: CMake 가 이 값을 캐시하므로 한쪽을 생략하면 그 방향으로는 직전 빌드의 선택이 남는다. 그래서 `build.sh` 는 매 빌드 `ON`/`OFF` 를 넘기고 `Tests:` 줄로 알린다. Stop hook 의 `--run` 은 `--tests` 로 빌드하고 (`verify-changes.sh` `run_build`), 그와 별개로 빌드 뒤 CMake 캐시가 `BUILD_TESTING=OFF` 인 패키지는 테스트하지 않고 UNVERIFIED 로 막는다 (`pkgs_built_without_tests`) — 판정이 모든 빌드 경로가 플래그를 기억하는 데 기대지 않게. 켜고 끄는 것은 그 패키지의 reconfigure 만 일으키고, 이미 컴파일된 테스트 object 는 남아 있어 소스가 그대로면 다시 켤 때 재컴파일하지 않는다.

## ccache

얻는 것은 **같은 TU 를 다시 컴파일하는 빌드** (`build.sh -c`, 브랜치를 오가는 재빌드, 별도 build base) 이고, 평소의 증분 빌드는 make 가 이미 건너뛰므로 달라지지 않는다. 공개 헤더를 고쳐 하위 TU 의 전처리 결과가 바뀌는 재컴파일도 캐시로 못 막는다.

| 클린 빌드 (`rtc_tsid` 까지 5개 패키지, 128 TU, 1 × `-j6`) | 시간 | 캐시 적중 |
|---|---|---|
| ccache 없음 | 4분 36초 | — |
| ccache, 빈 캐시 | 5분 2초 | 5% |
| ccache, 채워진 캐시 | 18초 | 100% |

빈 캐시의 첫 빌드는 9% 느리다 (해시 + 저장). launcher 도 `BUILD_TESTING` 처럼 매 빌드 명시한다 — 빈 값 포함: ccache 로 configure 한 트리는 `--no-ccache` 뒤에도, ccache 를 지운 뒤에도 (그때는 `ccache: not found` 로 실패하며) 계속 그것을 부른다. clangd 는 `compile_commands.json` 의 ccache prefix 를 스스로 걷어낸다 (clangd 18 `--check` 로 확인).

## colcon cwd · `source` 파이프 사고의 경로

[AGENTS.md](../../AGENTS.md) §9.1 의 서브셸 표준형이 생긴 이유다.

- **cwd drift**: 실제 위반은 룰을 몰라서가 아니라 cwd drift 로 재발한다 — 편집하러 패키지 dir 로 `cd` 한 shell 에서 그대로 colcon 을 치거나, `cd src/rtc-framework && source src/rtc-framework/...` 처럼 한 줄에 체이닝해 상대경로 source 가 silent fail 한 채 빌드가 repo 안에서 도는 경로다. 에이전트 shell 의 cwd 는 호출 간에 유지되므로 "독립 call 로 내면 된다" 는 오해다. `cd <rtc_ws> &&` 로 시작하기만 하면 규칙은 지켜지지만 그 다음 호출부터 cwd 가 ws root 에 남고, 그 부작용이 아래층에서 "cd 금지" 라는 반대 규율을 만들어 두 규칙이 충돌한 채 시작한 세션이 있었다 — 서브셸 표준형은 그 충돌 자체를 없앤다.
- **증상이 빌드 실패가 아니라는 점이 비싸다**: repo 안에 별도 `build/`·`install/`·`log/` 트리가 생기고 (`.clangd` 의 CompilationDatabase 가 그 트리를 가리키며 ws-root incremental cache 와 분리되어 추적 불가한 stale state 가 누적된다) 그 뒤로는 호출마다 두 트리를 오가므로 같은 세션의 검증들이 서로 모순되는 결과를 낸다 — 한 test 는 수정 후 코드로, 다른 test 는 stale 바이너리로 도는 식이다. 코드에 없는 논리 버그를 쫓게 되니, 검증 결과가 설명 불가하게 엇갈리면 코드를 의심하기 전에 `ls src/rtc-framework/build` 부터 친다.
- **`source` 를 파이프에 넣는 함정**: `source setup_env.sh 2>&1 | tail -2 && colcon build …` 는 source 가 subshell 에서 실행돼 env 가 부모 셸에 반영되지 않는다. `colcon` 자체는 profile PATH 에 있어 빌드가 정상 시작하고, 한참 뒤 CMake 안에서 `ModuleNotFoundError: No module named 'ament_package'` 같은 원인을 가리키지 않는 에러로 죽는다 (`ament_package` 는 dpkg 가 아니라 ROS distro 의 `site-packages` 에 있고 `PYTHONPATH` 로만 노출되므로). 이 실패를 `-DPython3_EXECUTABLE` 탓으로 오진하기 쉬운데 (venv carve-out 은 정상이다), 감별은 `echo $VIRTUAL_ENV` 또는 `/usr/bin/python3 -c "import ament_package"` 한 줄이면 된다. 서브셸 표준형은 `source` 와 `colcon` 이 같은 서브셸 안에 있으므로 이 함정과 상충하지 않는다 — 문제는 서브셸이 아니라 *파이프*가 만드는 서브셸에 colcon 이 안 들어가는 것이다.
- **`.venv` carve-out 의 범위**: `.venv` 규칙은 self-contained 하다 — 실패 시 격리를 무력화 (gtest 직접 실행 / venv deactivate / `PYTHONPATH` 우회) 하지 말고 sys.path / shebang / wrapper / dep resolution 을 디버그한다. 유일한 예외는 `colcon build` 의 configure 단계이며, `colcon test` / `ros2 run` / `ros2 launch` 실패를 덮는 deactivate 는 runtime PC 에서 재현될 결함을 숨기는 것이다.

## Plain `colcon build` 에서 CLI `--cmake-args` 가 defaults 를 대체하는 함정

`--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` 만 넘기면 새 빌드 트리가 `CMAKE_BUILD_TYPE` 없이 — 최적화 없이 — configure 되고 `compile_commands.json` 도 안 나온다. 기존 트리는 CMakeCache 가 옛 값을 기억해 차이가 드러나지 않는다. 인터프리터를 venv 유무와 무관하게 고정해야 하는 이유: `.venv` 가 활성이면 CMake `FindPython` 이 venv python 을 잡아 eigenpy/pinocchio configure 가 깨질 수 있고, venv 가 없어도 `FindPython` 은 PATH 디렉토리 순서로 찾으므로 PATH 앞의 다른 `python3.X` (예: `uv python install` 의 `~/.local/bin/python3.12`) 를 잡아 `catkin_pkg` 를 못 본다. `deactivate` 는 앞의 경우만 막는다.

## 빌드 보호 장치가 기대는 시스템 패키지

`install.sh` 가 직접 설치하는 것은 ccache 와 pytest-xdist 둘이고, 나머지 (`util-linux` · `mawk` · `coreutils` · `systemd` · `libpam-systemd` · `dbus-user-session`) 는 Ubuntu 기본 설치에 들어 있다 (dpkg priority `required` / `important`). 최소 컨테이너처럼 그것이 빠진 호스트에서는 해당 장치만 꺼지고 빌드는 된다. 메모리 상한 줄은 scope 를 실제로 만들어 그 cgroup 의 `memory.max` 를 읽어 본 결과다 — `systemd-run` 이 성공해도 한도가 걸리지 않는 호스트 (cgroup v1/hybrid, 위임 없음) 를 걸러낸다.

## 기록 시점의 이력

- CI: C++/Python 빌드·테스트 CI (`ros2-advanced-ci`) 와 CodeQL 은 2026-09-19 에 제거했다 (소요 시간 대비 효용이 낮다는 사용자 판단). 남은 CI 는 `docs-validate.yml` 하나이고, 빌드·테스트의 게이트는 로컬 `./build.sh` · `colcon test` 뿐이다 ([AGENTS.md](../../AGENTS.md) §4).
- 격리 의존성 (fmt/mimalloc/aligator) 의 소스 빌드 격리는 2026-04-21 에 들어갔다 (`repo_scripts/scripts/build_deps.sh`, prefix `<rtc_ws>/deps/install`). pinocchio · proxsuite 는 ROS distribution 의 apt 패키지만 쓰며 robotpkg fallback 은 없다 — 격리 계획이 ROS distribution 의 것을 전제하기 때문이다.
