# repo_scripts


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **로봇 비의존(robot-agnostic) RT 시스템 구성·검증 + 의존성 격리 스크립트 모음**입니다. PREEMPT_RT 커널 빌드, CPU 격리, IRQ 어피니티, 네트워크 최적화, NVIDIA 공존 설정, 환경 검증, LTTng 트레이싱, 그리고 ABI 민감 C++ 의존성(fmt/mimalloc/aligator) 소스 빌드 및 격리 관리를 위한 스크립트 모음과 공유 라이브러리를 제공합니다 (전체 목록·개수는 아래 "패키지 구조"의 실제 트리를 참조 — README에 개수를 박제하지 않음, `CMakeLists.txt`의 `install(PROGRAMS ...)`가 SSoT).

**설계 목표:**
- 500 Hz 실시간 제어에서 200 us 이하 지터 달성
- 2코어 VM부터 16코어+ 시스템까지 자동 적응
- 멱등(idempotent) -- 재실행 시 이미 적용된 설정 건너뜀
- 설정과 검증 분리 (setup vs. check/verify)
- 관심사 분리 -- 각 스크립트가 단일 책임 수행
- 시스템 의존성과 격리 (Level 3 -- `../deps.repos` + `<rtc_ws>/deps/install/`; 상세 배포 가이드는 본 문서 하단 참조)

---

## 패키지 구조

```
repo_scripts/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── thread_layout.yaml               <- **CPU 레이아웃 SSoT** (선언형 manifest, issue #153 M1)
└── scripts/
    ├── gen_thread_layout.py             <- manifest -> C++/shell/Python/README 생성 + --check 드리프트 게이트
    ├── lib/
    │   ├── rt_common.sh                  <- 공유 유틸리티 라이브러리 (모든 스크립트가 source)
    │   ├── thread_layout_generated.sh    <- **생성물** (직접 편집 금지) — tier -> slot/policy/priority 표
    │   ├── rt_report.sh                  <- check_rt_setup.sh / verify_rt_runtime.sh 공유 PASS/WARN/FAIL 리포트 인프라
    │   ├── bootstrap.sh                  <- build.sh/install.sh 공통 entry-script 초기화 (setup_env.sh 자동 source)
    │   │
    │   │   # ── install.sh 모듈형 helper (install.sh가 source) ──────
    │   ├── install_ros2.sh               <- ROS2/Ubuntu prerequisites + workspace 셋업
    │   ├── install_python.sh             <- .venv 생성 + requirements.lock 동기화 (uv pip sync)
    │   ├── install_uv.sh                 <- uv 부트스트랩 (Astral installer)
    │   ├── install_deps.sh               <- C++ 외부 의존성 (ur_driver/pinocchio/proxsuite/behaviortree/ONNX Runtime 1.17.1 (sha256 pin)/MuJoCo 3.7.0 tarball installer)
    │   ├── install_dev.sh                <- VS Code GDB 디버그 툴 + clangd code intelligence (clangd/libomp-dev) + LTTng 트레이싱 툴체인 설치
    │   └── install_rt.sh                 <- 7개 setup_*.sh RT 스크립트 sudo wrapper + RT 권한/sudoers/cset
    │
    │   # ── RT 시스템 구성/검증 ─────────────────────────────────────
    ├── build_rt_kernel.sh                <- PREEMPT_RT 커널 빌드/설치
    ├── setup_grub_rt.sh                  <- GRUB RT 커널 파라미터 관리
    ├── setup_irq_affinity.sh             <- IRQ 어피니티 설정
    ├── setup_udp_optimization.sh         <- NIC/네트워크 스택 최적화
    ├── setup_cpu_governor.sh             <- CPU governor performance 설정
    ├── setup_nvidia_rt.sh                <- NVIDIA GPU + RT 공존 설정
    ├── setup_display_rt.sh               <- RT 환경 디스플레이 최적화
    ├── cpu_shield.sh                     <- 동적 CPU 격리 (cset/cgroup)
    ├── check_rt_setup.sh                 <- 정적 RT 환경 검증 (9개 카테고리)
    ├── verify_rt_runtime.sh              <- 런타임 스레드 검증 (7개 카테고리)
    │
    │   # ── LTTng 트레이싱 ──────────────────────────────────────────
    ├── timeline.sh                       <- LTTng CTF trace -> Chrome Trace JSON 변환 (스레드/CPU swimlane)
    ├── enroll_lttng_mok.sh               <- Secure Boot 환경에서 lttng-modules DKMS 빌드에 MOK 서명
    │
    │   # ── 의존성 격리 (배포 가이드 참조) ────────────────────────
    ├── setup_env.sh                      <- 개발 쉘 env 활성화 (ROS + deps/install + venv + overlay)
    ├── build_deps.sh                     <- fmt/mimalloc/aligator 소스 빌드 -> deps/install/
    └── verify_isolation.sh               <- ELF RPATH/ldd 격리 검증
```

`scripts/lib/` 중 `install(PROGRAMS ...)`로 배포되는 것은 `CMakeLists.txt` 기준 SSoT — 정확한 install 대상 목록은 `CMakeLists.txt`를 직접 참조.

---

## 스크립트 분류

### 설정 스크립트 (Setup) -- 1회 실행

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `build_rt_kernel.sh` | mainline PREEMPT_RT 커널(6.12+) 다운로드/검증/빌드/설치 (7단계, profile 분기) | 필수 |
| `setup_grub_rt.sh` | GRUB RT 커널 파라미터 관리 + sched_rt_runtime_us 설정 | 필수 |
| `setup_irq_affinity.sh` | 하드웨어 IRQ를 OS 코어에 고정 (SMT-aware) | 필수 |
| `setup_udp_optimization.sh` | NIC 코얼레싱, 오프로드 비활성화, sysctl 최적화 + systemd 서비스 | 필수 |
| `setup_cpu_governor.sh` | CPU governor -> performance 모드 + systemd 서비스 | 필수 |
| `setup_nvidia_rt.sh` | NVIDIA GPU DKMS, modprobe, IRQ, persistence (7단계) | 필수 |
| `setup_display_rt.sh` | X11 안티 티어링 + compositor 우선순위 부스트 | 필수 |

### 런타임 스크립트 (Runtime) -- 실행 시마다

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `cpu_shield.sh` | 런타임 CPU 격리 (Tier 1/2, robot/sim 모드) | on/off 시 필수 |

### 검증 스크립트 (Verification) -- 필요 시

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `check_rt_setup.sh` | 정적 환경 검증 -- 커널, CPU, IRQ, 네트워크 등 (9개 카테고리) | 선택 |
| `verify_rt_runtime.sh` | 실행 중 스레드 스케줄링/어피니티/메모리 검증 (7개 카테고리) | 선택 |

> **External driver 프로세스 검증 (`arm_driver` / `hand_driver`).** 이 둘은 컨트롤러 내부 스레드가 아니라 launch 가 taskset 으로 pin 하는 **별도 프로세스**다 (`arm_driver`=`ros2_control_node`, `hand_driver`=`udp_hand_node`; thread_config.hpp §process-level threads). `verify_rt_runtime.sh` 는 이들을 process comm 으로 발견해 검증하되 **축이 다르다**: `arm_driver` 는 **RT 루프** — 설정된 우선순위(50)의 SCHED_FIFO 스레드가 존재하고 그것이 arm 코어에 있는지. `hand_driver` 는 **per-thread layout** — `hand_udp_recv` 는 hand 코어에 SCHED_FIFO 65, `hand_aux_io` 는 aux 코어(= OS slot)에 CFS, 그 외 TID(executor, DDS)는 hand 코어에 있어야 한다 (issue #345). 이름별 policy/priority 를 보는 것은 external driver 에 대해 이 검사가 처음이다 — `check_scheduling_policy` 는 컨트롤러 *내부* 스레드만 순회한다. 판정 우선순위는 `MISSING` > `WRONG_SCHED` > `WRONG_CPU` 이며, 스레드가 아예 없거나 스케줄러가 틀린 것이 코어 오배치보다 나쁜 발견이다. aux slot 은 `RTC_HAND_AUX_SLOT` 로 override 하며 기본값은 `get_os_cores()`(= 0) 로 노드의 `aux_cpu_slot` 기본값과 같다. ≤5코어 tier 에서는 hand slot 과 OS slot 이 둘 다 0 이라 두 기대가 같은 논리 CPU 로 풀려 그대로 통과한다(예외 처리 아님). hand 의 옛 전-스레드 affinity 검사(`taskset -a` pin, issue #245)는 **폐기됐다** — 프로세스 안에서 스레드마다 다른 코어에 붙으므로 균일 검사는 정상 배치를 FAIL 로 만든다. arm 은 프로세스 affinity 를 봐서는 안 된다: main thread 는 executor 이고 500 Hz 루프는 별도 스레드이며, 예전에는 launch 가 그 main thread 를 핀하고 검증기가 같은 스레드를 읽어 서로를 확인해 주고 있었다 (issue #343). FIFO 스레드 부재는 WARN 이 아니라 FAIL 이다 — FIFO 요청이 EPERM 으로 거부돼도 affinity 는 적용되므로, 위치만 보는 검사는 CFS 로 강등된 제어 루프를 통과시킨다. 우선순위 기대값은 `RTC_ARM_DRIVER_RT_PRIO` 로 override. sim launch 는 mujoco 만 pin 하고 driver 프로세스가 없으므로 **SKIP** 처리된다. 다른 로봇/드라이버는 `RTC_ARM_DRIVER_COMM` / `RTC_HAND_DRIVER_COMM` env (공백 구분 comm 후보 리스트) 로 override.

> **Intel hybrid 감지 (`[2.5/9]`) 전제조건 — `cpuid` 패키지 (`sudo apt install -y cpuid`).**
> Raptor/Meteor/Arrow Lake 등 P+E 하이브리드는 3-path 캐스케이드로 감지한다: ① primary = `/sys/.../cpu/types/` sysfs (**커널 >= 6.9 + `nuc` 프로파일 재빌드** 필요 — `build_rt_kernel.sh` 참조), ② fallback = CPUID leaf 0x1A (**`cpuid` 툴 필요**), ③ fallback = cpuinfo_max_freq 클러스터링. 세 경로가 모두 실패하면 hybrid CPU가 "homogeneous" 로 **오검출**된다. `cpuid` 는 재이미징 시 누락되기 쉬운 필수 도구이므로 신규/재설치 머신 프로비저닝에 반드시 포함한다 (미설치 시 `check_rt_setup.sh` 가 Intel CPU 에 한해 오검출 경고를 출력한다).

#### invariants 연관 매핑

[agent_docs/invariants.md](../agent_docs/invariants.md) §RT Host / Runtime Preconditions RT-HOST-1~3 은 본 패키지 스크립트가 검증한다:

- RT-HOST-1 (`mlockall`) ↔ `verify_rt_runtime.sh` (`VmLck` 검증)
- RT-HOST-2 (SCHED_FIFO priority) ↔ `verify_rt_runtime.sh` (thread sched policy 검증)
- RT-HOST-3 (CPU affinity) ↔ `cpu_shield.sh` (런타임 격리) + `setup_irq_affinity.sh` (IRQ 격리)
- System-level (PREEMPT_RT kernel, `@realtime` group, isolated cores) ↔ `check_rt_setup.sh` (9 카테고리 정적 검증)

세부 invariant 본문 / grep 패턴 / 복구는 [invariants.md](../agent_docs/invariants.md) 가 SSoT.

### 의존성 격리 스크립트 (Isolation) -- Level 3

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `setup_env.sh` | 매 쉘 활성화 -- ROS Jazzy + `deps/install` (fmt/mimalloc/aligator) + `.venv` + `install/` overlay. `COLCON_DEFAULTS_FILE` 도 주입 | 불필요 |
| `build_deps.sh` | `../deps.repos` 기반으로 fmt 11.1.4 / mimalloc 2.1.7 / aligator 0.19.0 을 `<rtc_ws>/deps/install/` 로 소스 빌드. deps/src 가 비어있으면 `vcs import` 자동 실행 | 불필요 |
| `verify_isolation.sh` | `install/` 의 모든 ELF 를 훑어 `/usr/local` · `/opt/openrobots` 등 격리 외부 경로 누수 검사 | 불필요 |

`build.sh` / `install.sh` 는 `RTC_DEPS_PREFIX` 미설정 시 `setup_env.sh` 를 자동으로 source 하므로, 수동 실행 없이 바로 동작합니다. 그러나 `ros2 launch` · `colcon test` 등 대화형 커맨드에서는 각자 `source setup_env.sh` 필요.

---

## 공유 라이브러리 (`rt_common.sh`)

모든 스크립트에서 `source`하는 핵심 유틸리티입니다. `build.sh`, `install.sh`에서도 공유됩니다. 직접 실행할 수 없으며, 중복 source 방지 가드가 포함되어 있습니다.

### CPU 감지 함수

| 함수 | 설명 |
|------|------|
| `get_physical_cores()` | 물리 코어 수 감지 (SMT/HT 제외). lscpu -> sysfs -> nproc 순 |
| `compute_cpu_layout()` | 코어 수 기반 RT 레이아웃 계산 (OS/RT 코어 범위, IRQ 마스크) — 내부에서 IRQ affinity bitmask도 함께 산출 |
| `compute_expected_isolated()` | 비-OS 코어 전체 범위 (RT + nrt + driver, SMT 시블링 포함). cset shield 검증 (`check_rt_setup.sh`) 에 사용 — nohz_full / rcu_nocbs 는 `get_rt_cores_with_siblings()` 를 사용 (좁은 범위) |
| `get_os_logical_cpus()` | OS 물리 코어에 속하는 논리 CPU 번호 목록 |

### NIC/네트워크 함수

| 함수 | 설명 |
|------|------|
| `detect_physical_nic()` | 물리 NIC 자동 탐색 (가상 인터페이스 제외, UP 상태 우선) |

### 로깅 함수

| 함수 | 설명 |
|------|------|
| `setup_colors()` | 터미널 색상 변수 초기화 (비-TTY 환경에서는 빈 문자열). source 시 자동 호출되므로 보통 직접 호출 불필요 |
| `make_logger PREFIX [STYLE]` | 로깅 프리픽스+스타일 설정. `bracket` (기본): `[PREFIX] msg`, `emoji`: 이모지 형식 |
| `info()` / `warn()` / `error()` / `success()` / `section()` | 색상별 로깅 |
| `fatal()` | 에러 메시지 출력 후 `exit 1` (error + exit 통합) |

### 파일/시스템 유틸리티

| 함수 | 설명 |
|------|------|
| `require_root()` | root 권한 확인 (실패 시 `fatal`) |
| `write_file_if_changed()` | 멱등 atomic 파일 쓰기 (same-dir `mktemp` + `mv`, 기존 mode 보존, 다를 시 백업) |
| `with_temporary_disable()` | postinst hook 일시 비활성화 후 명령 실행, `trap EXIT` 으로 실패해도 hook 복원 (e.g. DKMS hook + `dpkg -i`) |
| `auto_release_cpu_shield()` | 빌드 전 CPU shield 자동 해제 (cset 감지 시 해제, isolcpus는 경고만) |
| `check_workspace_structure()` | ROS2 워크스페이스 디렉토리 구조 검증 (`src/` 하위 확인) |
| `ensure_ros2_sourced()` | ROS2 환경 자동 탐색 및 소싱 (jazzy 우선, humble fallback — setup_env.sh와 동일 priority) |

### 패키지 리스트 함수

| 함수 | 설명 |
|------|------|
| `get_base_packages()` | 기본 RTC 패키지 리스트 (build.sh/install.sh 공유). Phase 5에서 `rtc_mpc`가 `rtc_urdf_bridge`와 `rtc_tsid` 사이에 추가됨. |
| `get_robot_packages()` | 로봇 전용 패키지 리스트 |

### RT/MPC 코어 레이아웃 함수 (Layout SSoT)

**값의 SSoT 는 [config/thread_layout.yaml](config/thread_layout.yaml)** 이고, 아래 tier 의존 헬퍼들은 그 manifest 에서 [scripts/lib/thread_layout_generated.sh](scripts/lib/thread_layout_generated.sh) 로 **생성**됩니다 (issue #153 M1 — 그 전에는 같은 표가 여기 셋, C++, Python, 검증기에 손으로 인코딩돼 있었습니다). `rt_common.sh` 가 그 파일을 source 하므로 호출부는 달라진 게 없고, 생성 파일을 직접 편집하면 `gen_thread_layout.py --check` 가 CI 와 `colcon test` 에서 차단합니다. 이 파일들은 순수 함수(코어 수 in, slot out)라 `install.sh` 가 workspace 빌드 **전에** source 할 수 있습니다.

각 함수는 `$1` 로 물리 코어 수를 받을 수 있습니다 (미지정 시 `get_physical_cores`) — 이 머신에 없는 tier 를 테스트·출력할 때 씁니다. v4.1 Layout SSoT 통합 후:

- `cpu_shield.sh::compute_shield_cores()` 는 `get_cm_shield_cpus()` 를 호출 (자체 tier 분기 없음). CM 프로세스 전체가 cpuset 에 들어가야 하므로 shield 는 RT ∪ nrt span 을 덮는다 (issue #151). shield 는 cpuset 만 만들고, 런치가 `cpu_shield.sh adopt <pid>` 로 CM 을 그 안에 넣는다. **`adopt` 의 종료 코드가 계약이다** (issue #344): 양성 no-op(cset 미설치 / shield 비활성 / PID 부재)은 0, *필요했는데 실패*하면 non-zero 이고 런치는 그때 ACTIVATE 를 거부한다 — 그대로 활성화하면 RT 스레드가 affinity 와 SCHED_FIFO 를 함께 잃은 채 돈다.
- `setup_grub_rt.sh` 는 `get_rt_cores_with_siblings()` 를 호출하여 `nohz_full` / `rcu_nocbs` 값으로 RT thread 가 실행되는 코어만 (SMT 시 sibling 포함) 한정.
- `setup_irq_affinity.sh` / `check_rt_setup.sh` / `verify_rt_runtime.sh` 는 `compute_cpu_layout()` 기반으로 동작하여 tier 분기가 없습니다 (OS/RT 코어 범위가 layout v4.1 에서 모든 tier 공통: OS=0, RT=1..N-1).

| 함수 | 출처 | 설명 |
|------|------|------|
| `get_role_spec()` / `get_role_slot()` / `get_role_policy()` / `get_role_priority()` / `get_role_nice()` | 생성 | `<role> [ncpu]` 로 임의 role 의 배치를 조회. 그 tier 에 없는 role (예: 8코어의 `mpc_worker_0`) 은 **비0 종료** — 없는 스레드를 기대 목록에 넣지 않기 위한 계약이다 |
| `get_mpc_cores()` | 생성 | MPC 코어 (main + workers) CSV. 첫 항목이 항상 MPC main |
| `get_mpc_main_core()` | 파생 | MPC main 코어만 (`get_mpc_cores` 의 첫 항목) |
| `get_rt_cores()` | 생성 | RT 그룹 전체 집합 (rt_control + rt_callback + MPC). `get_rt_cores_with_siblings()` / `get_rt_shield_cpus()` 의 base |
| `get_nrt_cores()` | 생성 | nrt_logging + nrt_callback + nrt_publish 슬롯 (중복 제거, 오름차순). `get_cm_shield_cpus()` 가 RT 와 union. layout v5 부터 tier ≥ 6 은 aux slot (= rt_callback slot 2) 이라 RT 의 부분집합이고, degraded tier 는 Core 0 공유 — 어느 쪽이든 union 이 RT span 을 넘지 않는다 |
| `get_arm_driver_slot()` / `get_hand_driver_slot()` | 생성 | 외부 driver 프로세스의 slot (launch taskset / in-process self-pin 대상) |
| `get_os_cores()` | 생성 | OS/DDS/IRQ 코어 (Core 0 단일, layout v4.1). `get_cm_shield_cpus()` 가 shield 에서 제외할 slot 판정에 사용 |
| `rtc_expected_threads()` | 생성 | `verify_rt_runtime.sh` 의 기대 표 (`name:slot:policy:priority[:optional]`). 컨트롤러 **in-process 스레드만** — arm/hand 는 별도 프로세스라 여기 있으면 항상 false-WARN 이다 (#353) |
| `get_rt_cores_with_siblings()` | 파생 | `get_rt_cores()` 출력에 SMT HT 시블링까지 포함, range-collapse. `setup_grub_rt.sh` 의 `nohz_full` / `rcu_nocbs` 값. non-SMT 시 입력과 동일 cpu 집합 (range 표기) |
| `get_rt_shield_cpus()` | 파생 | RT 슬롯 → **logical cpu** (slot→logical) + HT 시블링. RT-only 격리/검증 (verify_rt_runtime) |
| `get_cm_shield_cpus()` | 파생 | CM 프로세스 전체 span = `get_rt_shield_cpus()` ∪ nrt (logical + 시블링, OS slot 제외). `cpu_shield.sh` 의 cset "user" cpuset 범위 (issue #151). 예: non-SMT 12c `1-5,8-9` · NUC13 12c hybrid `2-9,12-13` |

**"생성"** = manifest 에서 나온다 (직접 편집 금지). **"파생"** = `rt_common.sh` 에 손으로 쓰여 있고 위 생성 함수를 호출한다. tier 별 반환값은 아래 생성된 표가 SSoT 이므로 여기 중복해 적지 않는다 — 예전엔 이 열이 tier 표의 일곱 번째 사본이었다.

Tier별 매핑 (SSoT: `repo_scripts/config/thread_layout.yaml` — 아래 표는 그 manifest 에서 생성된다):

<!-- BEGIN GENERATED: thread-layout-tiers -->
<!-- Generated from repo_scripts/config/thread_layout.yaml by repo_scripts/scripts/gen_thread_layout.py. Do not edit by hand. -->

| tier (물리 코어) | `get_rt_cores()` | `get_mpc_cores()` | `get_nrt_cores()` | `get_os_cores()` | arm / hand slot |
|---|---|---|---|---|---|
| ≤5 | `1,2,3` | `3` | `0` | `0` | 0 / 0 |
| 6–7 | `1,2,3` | `3` | `2` | `0` | 4 / 4 |
| 8–9 | `1,2,3` | `3` | `2` | `0` | 4 / 5 |
| 10–11 | `1,2,3,4` | `3,4` | `2` | `0` | 5 / 6 |
| 12–13 | `1,2,3,4,5` | `3,4,5` | `2` | `0` | 6 / 7 |
| 14–15 | `1,2,3,4,5` | `3,4,5` | `2` | `0` | 6 / 7 |
| 16+ | `1,2,3,4,5` | `3,4,5` | `2` | `0` | 6 / 7 |

<!-- END GENERATED: thread-layout-tiers -->

- **≤5코어 / 6–7코어는 degraded** — RT 결정성 보장 X. 전자는 nrt·driver 가 전부 OS Core 0 으로 접히고 mpc 가 CFS 로 강등되며, 후자는 arm/hand 가 한 코어를, nrt_logging + nrt_callback 이 또 한 코어를 공유하고 mpc_worker 가 없다.
- **8코어 이상**에서 arm_driver / hand_driver 가 전용 슬롯을 얻고, **10코어 이상**에서 mpc_worker_0, **12코어 이상**에서 mpc_worker_1 이 붙는다. 14/16 tier 는 12 와 같은 배치이고 남는 슬롯이 spare 다 (16c 의 과거 "user cset shield Core 4-8" 잔재는 v4.1 에서 제거).

**hand aux lane (issue #345)**: `udp_hand_node` 는 위 `hand_driver` 코어에 더해 **OS slot(Core 0)에 `hand_aux_io` 스레드 하나**를 둔다 — 타이밍 CSV drain(1 Hz, 버스트당 최대 512행)과 stats JSON 저장 같은 blocking 파일 I/O 전용 CFS 레인이다. shield 밖 `system` cpuset 안이라 cpuset 재설계가 필요 없고 `cset shield` 활성 상태에서도 EINVAL 이 나지 않는다. slot 은 노드 param `aux_cpu_slot`(기본 0) 이며 shell 쪽 SSoT 는 `get_os_cores()` 다. ≤5코어 tier 에서는 `hand_driver` slot 도 0 이라 두 레인이 같은 코어로 합쳐진다 (그 tier 는 원래 Core 0 에 전부 모이는 degraded 배치다).

**v4.1 의 핵심 변화 (모든 tier 공통)**: RT cluster 가 Core 1 부터 시작 (Core 0 = OS/DDS/IRQ 전용), nrt_logging / nrt_callback 이 모든 ≥ 6c tier 에서 Core 0 와 분리, arm/hand 알파벳 순, sim_thread / viewer 가 모든 tier 에서 `cpu_core = -1`. 단조성 불변식은 `rtc_base/test/test_mpc_thread_config.cpp` 의 tier 쌍 + sentinel 처리 테스트가 회귀 방지.

---

## 스크립트 상세

### build_rt_kernel.sh

mainline PREEMPT_RT 커널(6.12+)을 소스에서 빌드/검증/설치합니다. 7단계 파이프라인으로 체크포인트 기반 재개를 지원합니다. PREEMPT_RT 는 Linux 6.12 에서 mainline 병합되어 외부 RT 패치 없이 `CONFIG_PREEMPT_RT=y` 만으로 RT 활성화됩니다.

```bash
sudo ./build_rt_kernel.sh                  # 대화형 빌드 (완료 단계 자동 스킵)
sudo ./build_rt_kernel.sh --batch          # 비대화형 (menuconfig 건너뜀)
sudo ./build_rt_kernel.sh --batch --with-nvidia   # 데스크탑 (NVIDIA DKMS 포함)
sudo ./build_rt_kernel.sh --profile nuc --batch   # Intel NUC hybrid config 강제
sudo ./build_rt_kernel.sh --kernel-version 6.12.91  # 커널 버전 지정 (6.12+)
sudo ./build_rt_kernel.sh --dry-run        # 다운로드·검증·압축해제·설정까지만
sudo ./build_rt_kernel.sh --status         # 진행 상태만 확인
sudo ./build_rt_kernel.sh --verify         # 각 단계별 세부 적용 상태 진단
sudo ./build_rt_kernel.sh --skip-verify    # GPG 서명 검증 건너뜀
sudo ./build_rt_kernel.sh --force-step 5   # 5단계부터 강제 재시작
sudo ./build_rt_kernel.sh --clean          # 빌드 소스 정리 후 처음부터
sudo ./build_rt_kernel.sh --build-dir /path  # 빌드 디렉토리 지정
```

**7단계 구성:**

| 단계 | 내용 |
|------|------|
| 1/7 | 필수 빌드 패키지 설치 |
| 2/7 | 커널 소스 다운로드 + GPG 서명 검증 |
| 3/7 | 압축 해제 |
| 4/7 | 커널 설정 (PREEMPT_RT + profile별 config) |
| 5/7 | 커널 빌드 (`make bindeb-pkg`) |
| 6/7 | .deb 패키지 설치 |
| 7/7 | GRUB 등록 확인 및 기본 부팅 설정 |

**커널 / profile:**

- **커널**: 6.12.x LTS (mainline PREEMPT_RT, 외부 패치 불요) — Ubuntu 22.04 / 24.04 공통, OS 독립. 기본 `6.12.91`, `--kernel-version` 으로 override (6.12+). 두 머신 재현성을 위해 pin.
- **profile** (`--profile auto`, CPU 자동 감지 — rt_common `detect_hybrid_capability`):
  - `desktop-amd` — AMD Ryzen (`amd_pstate`, SCHED_CLUSTER)
  - `desktop-intel` — 일반 Intel (`intel_pstate`, non-hybrid)
  - `nuc` — Intel hybrid NUC 13/14/15 (SCHED_MC_PRIO / SCHED_CLUSTER / INTEL_HFI_THERMAL)
- **NVIDIA**: 기본 분리 — `setup_nvidia_rt.sh` 가 담당. `--with-nvidia` 일 때만 커널 빌드 단계에서 DKMS 직접 빌드.
- **Secure Boot**: 활성 시 미서명 custom 커널 부팅 차단 경고 (감지만).

---

### setup_grub_rt.sh

GRUB RT 커널 파라미터의 **단일 진실 원천(single source of truth)**입니다. `install.sh`와 `setup_nvidia_rt.sh`에서 호출됩니다.

```bash
sudo ./setup_grub_rt.sh
sudo ./setup_grub_rt.sh --help
```

**관리하는 GRUB 파라미터:**

| 파라미터 | 값 | 효과 |
|---------|-----|------|
| `nohz_full` | RT thread core (SMT sibling 포함) | SCHED_FIFO 가 실제로 실행되는 코어에서 타이머 틱 제거. 값은 `get_rt_cores_with_siblings()` SSoT — 예: 6c "1-3", 12c "1-5", SMT 6C/12T 시 "1-3,7-9". nrt / driver (CFS) 는 제외. |
| `rcu_nocbs` | RT thread core (SMT sibling 포함) | RCU 콜백을 RT 코어 밖으로 오프로드 (동일 범위). |
| `processor.max_cstate` | 1 | 깊은 C-state 진입 방지 |
| `clocksource` | tsc | HPET 대비 50-100x 빠른 타이머 |
| `tsc` | reliable | TSC 불안정 감지 비활성화 |
| `nmi_watchdog` | 0 | NMI 인터럽트 제거 |
| `threadirqs` | (없음) | IRQ 핸들러 스레드화 |
| `nosoftlockup` | (없음) | soft lockup 경고 방지 |

**sysctl 설정:** `kernel.sched_rt_runtime_us=-1` 즉시 + `/etc/sysctl.d/99-rt-sched.conf`로 영구 적용

참고: `isolcpus`는 포함하지 않습니다. 런타임 동적 격리를 위해 `cpu_shield.sh`(cset shield)로 대체되었습니다. SMT 시 RT 물리 코어의 HT 시블링은 `nohz_full` / `rcu_nocbs` 에 자동 포함되어 (sibling tick → RT 코어 L1/L2 invalidation 방지) cache 일관성을 유지합니다.

---

### setup_irq_affinity.sh

하드웨어 IRQ를 OS 코어에 고정하여 RT 코어를 인터럽트 지터로부터 보호합니다.

```bash
sudo ./setup_irq_affinity.sh              # NIC 자동 감지
sudo ./setup_irq_affinity.sh enp3s0       # NIC 지정
```

- **SMT-aware**: HT 시블링까지 포함한 IRQ affinity mask 계산
- NIC 전용 IRQ 우선 고정 후, 나머지 모든 하드웨어 IRQ도 OS 코어에 고정
- 타이머 IRQ 0은 제외
- 적용 후 RT 코어에 잔여 IRQ가 없는지 자동 검증
- 적용 후 코어별 스레드 레이아웃(thread_config.hpp 기준) 출력

---

### setup_udp_optimization.sh

NIC 하드웨어 및 커널 네트워크 스택을 실시간 UDP/DDS 통신에 최적화합니다.

```bash
sudo ./setup_udp_optimization.sh          # NIC 자동 감지
sudo ./setup_udp_optimization.sh enp1s0   # NIC 지정
```

| 단계 | 설정 | 효과 |
|------|------|------|
| 1 | 인터럽트 코얼레싱 비활성화 (rx-usecs=0, adaptive=off) | 즉시 인터럽트 전달 |
| 2 | GRO/LRO/TSO/GSO 비활성화 | 패킷 병합 방지, 지연 시간 감소 |
| 3 | RX/TX 링 버퍼 1024 | DDS 패킷에 적합한 균형 |
| 4 | sysctl rmem_max/wmem_max=2GB, default=212992, backlog=5000 | DDS 소켓 버퍼 상한 확대 |
| 5 | systemd oneshot 서비스 생성 (`rtc-udp-optimization.service`) | 부팅 시 ethtool 설정 자동 재적용 |

- sysctl 설정: `/etc/sysctl.d/99-ros2-udp.conf`로 영구 적용
- ethtool 설정: `/usr/local/bin/rtc-udp-optimization.sh` 스크립트 + systemd 서비스로 부팅 시 자동 재적용

> CycloneDDS `SocketReceiveBufferSize`(8MB)는 `rmem_max`(2GB) 이하여야 합니다.

---

### setup_cpu_governor.sh

CPU governor를 performance 모드로 설정합니다.

```bash
sudo ./setup_cpu_governor.sh
sudo ./setup_cpu_governor.sh --help
```

| 단계 | 내용 |
|------|------|
| 1 | 모든 CPU governor 상태 확인 및 performance로 즉시 변경 (sysfs 직접 설정) |
| 2 | cpupower 도구 자동 설치 시도 (미설치 시) |
| 3 | systemd 서비스 생성 (`cpu-governor-performance.service`) -- 부팅 시 자동 적용 |
| 4 | Intel P-state 드라이버 및 Turbo Boost 상태 리포팅 |

검증: `cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`

---

### setup_nvidia_rt.sh

NVIDIA GPU와 PREEMPT_RT 커널의 안정적 공존을 설정합니다 (7단계). Ubuntu 22.04/24.04 지원.

```bash
sudo ./setup_nvidia_rt.sh
sudo ./setup_nvidia_rt.sh --help
```

**7단계 프로세스:**

| 단계 | 내용 |
|------|------|
| 1/7 | Pre-flight checks (Ubuntu 버전, RT 커널, NVIDIA GPU, CPU 레이아웃) |
| 2/7 | NVIDIA modprobe 설정 (`NVreg_EnableGpuFirmware=0` 등) |
| 3/7 | NVIDIA IRQ affinity systemd 서비스 |
| 4/7 | nvidia-smi persistence mode 활성화 |
| 5/7 | nouveau 블랙리스트 (활성 시) |
| 6/7 | NVIDIA DKMS 모듈 빌드 (RT 커널용) |
| 7/7 | 연관 스크립트 호출 + 검증 요약 |

> Stage 7에서 `setup_grub_rt.sh`, `setup_display_rt.sh`, `setup_cpu_governor.sh`를 자동 호출합니다.

---

### setup_display_rt.sh

RT 환경에서 디스플레이 안정성을 확보합니다.

```bash
sudo ./setup_display_rt.sh
sudo ./setup_display_rt.sh --help
```

| 단계 | 내용 |
|------|------|
| 1/2 | X11 안티 티어링 (`ForceFullCompositionPipeline`, TripleBuffer) -- `/etc/X11/xorg.conf.d/20-nvidia-antitear.conf` |
| 2/2 | Compositor 우선순위 부스트 (Xorg/Xwayland/gnome-shell/kwin을 renice -10) -- systemd 서비스 등록 |

- Wayland/X11/headless 자동 감지
- headless 환경에서도 설정 파일 설치 (다음 GUI 부팅 시 적용)
- Wayland 환경에서는 XWayland 창에만 anti-tearing 적용

---

### cpu_shield.sh

런타임 CPU 격리를 동적으로 관리합니다. `isolcpus` GRUB 파라미터 없이 재부팅 불필요합니다.

```bash
sudo cpu_shield.sh on --robot    # 전체 격리 (Tier 1+2)
sudo cpu_shield.sh on --sim      # 경량 격리 (Tier 1만)
sudo cpu_shield.sh off           # 격리 해제
cpu_shield.sh status             # 상태 확인 (sudo 불필요)
```

**격리 방식:** cset shield (필수). 과거 cgroup v2/v1 cpuset fallback 은 태스크 이주·complement cpuset 없이 빈 `rt_shield` 만 만들어 **실질 격리를 제공하지 못했으므로 비활성화**되었다 — `cset` 미설치 시 shield 는 허상 success 대신 명확히 실패한다 (`sudo apt-get install -y cpuset` 로 설치; `install_cset_tools` 가 robot/full 설치 시 자동 처리).

**Tier 모델:**

| Tier | 스레드 | 격리 |
|------|--------|------|
| Tier 1 (RT-critical) | rt_control + rt_callback (FIFO 70) + mpc_main + mpc_workers | 항상 |
| Tier 2 (driver / IO) | arm_driver, hand_driver, hand_aux_io, nrt_logging, nrt_callback | SCHED_OTHER — shield 밖 dedicated core. arm 은 CM 파라미터로 내부 RT 루프만 FIFO 50 + pin (issue #343). hand 는 프로세스가 스스로 pin 하고 (`use_cpu_affinity` param) 내부 `hand_udp_recv` 만 FIFO 65, `hand_aux_io` 는 OS slot 으로 분리 (issue #345); launch 는 DDS 스레드만 co-pin 한다. sim 은 taskset |
| Tier 3 (Flexible) | sim_thread, viewer, monitoring, build | 격리 안 함 (`cpu_core = -1`, no pin) |

> Layout v4.1 에서 `--robot` / `--sim` 두 모드의 shield 범위가 동일해짐 (RT + MPC only). driver / IO 코어는 shield 밖에서 SCHED_OTHER 로 직접 핀 — 별도 tier 격리 불필요. 두 옵션은 forward-compat 용으로 유지.

**코어 수별 격리 범위 (`--robot` / `--sim` 동일):**

| 코어 수 | shield cores |
|---------|--------------|
| 4코어 이하 | 1,3 |
| 6-9코어 | 1-3 |
| 10-11코어 | 1-4 |
| 12코어+ | 1-5 |

> Launch 파일 (`ur_control.launch.py`, `mujoco_sim.launch.py`)에서 자동 호출됩니다.
> `build.sh` / `install.sh`에서 빌드 전 자동 해제됩니다.

---

### check_rt_setup.sh

실시간 환경의 정적 검증을 수행합니다 (컨트롤러 실행 전, read-only).

```bash
./check_rt_setup.sh              # 상세 출력
./check_rt_setup.sh --summary    # 카테고리별 한 줄 요약
./check_rt_setup.sh --json       # CI/CD용 JSON 출력
./check_rt_setup.sh --fix        # 실패 항목별 수정 명령 표시
./check_rt_setup.sh --benchmark  # cyclictest 지터 측정 (sudo 필요)
```

**9개 검증 카테고리:**

| # | 카테고리 | 검증 내용 |
|---|---------|----------|
| 1 | RT 커널 | `PREEMPT_RT` 활성 (`uname -v`) |
| 2 | CPU 격리 | `isolcpus` 또는 cset shield 상태 |
| 3 | 스케줄러 & 메모리 | clocksource=tsc, sched_rt_runtime_us=-1, THP=never |
| 4 | GRUB 파라미터 | nohz_full, rcu_nocbs, processor.max_cstate, nmi_watchdog |
| 5 | RT 권한 | ulimit -r >= 90, ulimit -l=unlimited, realtime 그룹 |
| 6 | IRQ 어피니티 | 모든 IRQ가 OS 코어에 고정 |
| 7 | 네트워크/UDP | rmem_max=2GB, 코얼레싱=0 |
| 8 | NVIDIA | persistence 모드, IRQ 어피니티, nouveau 차단 |
| 9 | CPU 주파수 | governor=performance |

`--benchmark` 옵션 사용 시 cyclictest를 통한 RT 지터 실측을 추가 수행합니다 (sudo 필요).

**종료 코드:** 0=PASS, 1=경고, 2=실패

---

### verify_rt_runtime.sh

RT 컨트롤러 실행 중 스레드 상태를 검증합니다.

```bash
./verify_rt_runtime.sh              # 상세 출력
./verify_rt_runtime.sh --summary    # 카테고리별 요약
./verify_rt_runtime.sh --json       # JSON 출력
./verify_rt_runtime.sh --watch      # 3초 간격 연속 모니터링
./verify_rt_runtime.sh --watch 5    # 5초 간격 연속 모니터링
```

**7개 검증 카테고리:**

| # | 카테고리 | 검증 내용 |
|---|---------|----------|
| 1 | 프로세스 탐색 | robot bringup exec (예: `integrated_rt_controller`) PID, 스레드 이름 매칭 |
| 2 | 스케줄링 정책 | SCHED_FIFO/OTHER + 우선순위 (`chrt -p`) |
| 3 | CPU 어피니티 | 예상 코어에 고정 (`taskset -p`) |
| 4 | 메모리 잠금 | `mlockall()` 적용 (VmLck > 0) |
| 5 | 컨텍스트 스위치 | 비자발적 전환 빈도 |
| 6 | CPU 마이그레이션 | RT 스레드 코어 이동 없음 |
| 7 | RT 스로틀링 | `sched_rt_runtime_us=-1`, 스로틀링 누적 없음 |

**종료 코드:** 0=PASS, 1=경고, 2=실패, 3=컨트롤러 미실행

---

### setup_env.sh

개발 쉘 1회 활성화. `build.sh`/`install.sh` 에서는 자동으로 source 되지만, `ros2 launch`, `colcon test` 등 대화형 커맨드에서는 매 쉘마다 명시적으로 source 필요.

```bash
source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh
```

**환경 변수 export:**

| 변수 | 값 | 용도 |
|------|-----|-----|
| `RTC_DEPS_PREFIX` | `<rtc_ws>/deps/install` | aligator/fmt/mimalloc 격리 prefix |
| `CMAKE_PREFIX_PATH` | `$RTC_DEPS_PREFIX:...:/opt/onnxruntime` | CMake 의존성 검색 순서 (ONNX Runtime 포함) |
| `LD_LIBRARY_PATH` | `$RTC_DEPS_PREFIX/lib:...` | 런타임 lib resolve |
| `PKG_CONFIG_PATH` | `$RTC_DEPS_PREFIX/lib/pkgconfig:...` | .pc 기반 설정 |
| `MUJOCO_DIR` | `/opt/mujoco-3.x.x` (자동 탐색) | `rtc_mujoco_sim` 의 fallback |
| `mujoco_ROOT` | `$MUJOCO_DIR` | `find_package(mujoco)` cmake hint (build.sh 와 동등) |
| `COLCON_DEFAULTS_FILE` | `<repo>/.colcon/defaults.yaml` | cwd 와 무관하게 colcon 기본 옵션 적용 |
| `VIRTUAL_ENV` | `<rtc_ws>/.venv` | Python venv 활성화 |

**Source 순서**: ROS Jazzy → deps/install (+ ONNX Runtime) → .venv → workspace overlay (`install/setup.bash`, 있을 때만).

**Plain `colcon build` 호환성** (build.sh 우회 워크플로): `setup_env.sh` 만 source 하면 `cd <rtc_ws> && colcon build --symlink-install` 로 단독 빌드가 가능하다. ONNX Runtime · MuJoCo · deps/install prefix 모두 환경변수로 주입되며, `.colcon/defaults.yaml` 이 `--symlink-install` / `Release` / `compile_commands` 를 자동 적용한다. 단 `.venv` 가 활성 상태면 CMake `FindPython` 이 venv python 을 잡아 eigenpy/pinocchio configure 가 깨질 수 있으므로, `colcon` 직접 호출 전에는 `deactivate` 하거나 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` 를 명시한다 (build.sh 는 이를 자동 처리). build.sh 가 추가로 수행하는 모드별 패키지 셀렉션 · `compile_commands.json` 머지 · `check_rt_setup.sh` 호출은 colcon 단독에서는 빠진다.

---

### build_deps.sh

`../deps.repos` 기반으로 fmt/mimalloc/aligator 를 소스 빌드하여 `<rtc_ws>/deps/install/` 에 설치합니다.

```bash
source setup_env.sh
~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/build_deps.sh
```

**동작:**
1. `deps/src/aligator/.git` 가 없으면 `vcs import deps/src < ../deps.repos` + `git submodule update` 자동 실행
2. 위상 순서로 각각 cmake configure + build + install (CPU 병렬):
   - `fmt 11.1.4` (`-DFMT_TEST=OFF -DFMT_DOC=OFF`)
   - `mimalloc 2.1.7` (`-DMI_BUILD_TESTS=OFF -DMI_BUILD_OBJECT=OFF`)
   - `aligator 0.19.0` (`-DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_EXAMPLES=OFF` + `-Dhpp-fcl_DIR=/opt/ros/.../hpp-fcl` + `-Dfmt_DIR=$DEPS_PREFIX/lib/cmake/fmt`)
3. `RPATH`: `$ORIGIN/../lib:$DEPS_PREFIX/lib` — 시스템 `/usr/local` 참조 없음

**소요 시간**: 처음 빌드 시 ~15-20분 (aligator 가 주 병목). 재실행 시 cmake configure 단계만 통과하며 빠르게 종료.

---

### verify_isolation.sh

`install/` 하위 모든 ELF 바이너리·공유 라이브러리가 격리된 경로로만 의존성을 해결하는지 검사합니다.

```bash
source setup_env.sh
~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/verify_isolation.sh
```

**허용 경로:**
- `/opt/ros/jazzy/` (ROS distribution)
- `<rtc_ws>/deps/install/` (격리된 fmt/mimalloc/aligator)
- `<rtc_ws>/install/` (workspace overlay)
- `/lib/x86_64-linux-gnu`, `/usr/lib/x86_64-linux-gnu` (libc, libstdc++, libgomp 등)
- `/opt/onnxruntime*`, `/opt/mujoco-*` (승인된 바이너리 drop)

**종료 코드:** 0=모든 ELF 격리 통과, 1=하나 이상 누수 (상세 출력)

---

## 권장 설정 워크플로

```
[1회 설정 -- 순서대로 실행]
  1. sudo ./build_rt_kernel.sh --batch    # PREEMPT_RT 커널 빌드/설치
     -> 재부팅 (RT 커널로 부팅)

  2. sudo ./setup_grub_rt.sh              # GRUB RT 파라미터 + sched_rt_runtime_us
     -> 재부팅 (GRUB 파라미터 적용)

  3. sudo ./setup_irq_affinity.sh         # IRQ를 OS 코어에 고정
  4. sudo ./setup_udp_optimization.sh     # NIC/네트워크 최적화

  5. sudo ./setup_cpu_governor.sh         # CPU governor -> performance

  (NVIDIA GPU가 있는 경우)
  6. sudo ./setup_nvidia_rt.sh            # NVIDIA 설정 + 위 3/4/5 자동 호출
     -> 재부팅 (DKMS 모듈 적용)

  (선택사항)
  7. sudo ./setup_display_rt.sh           # 디스플레이 최적화

[빌드 전 검증]
  ./check_rt_setup.sh --summary

[로봇 실행]
  sudo cpu_shield.sh on --robot           # Tier 1+2 격리
  ros2 launch integrated_bringup ur_control.launch.py
  ./verify_rt_runtime.sh --watch 3        # 런타임 모니터링

[시뮬레이션 실행]
  sudo cpu_shield.sh on --sim             # Tier 1만 격리
  ros2 launch integrated_bringup mujoco_sim.launch.py

[종료]
  sudo cpu_shield.sh off                  # 격리 해제
```

> `setup_nvidia_rt.sh`를 실행하면 Stage 7에서 `setup_grub_rt.sh`, `setup_display_rt.sh`, `setup_cpu_governor.sh`를 자동 호출하므로 별도 실행이 불필요합니다.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | ROS2 빌드 시스템 |

**시스템 요구사항:** bash 4.0+, `ethtool`, `lscpu`, `sysctl`, `cset` (cpuset), `cyclictest` (rt-tests, 벤치마크 시)

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select repo_scripts
source install/setup.bash
```

스크립트만 포함된 패키지이므로 바이너리는 생성되지 않습니다. `colcon build` 시 스크립트가 `install/repo_scripts/lib/repo_scripts/`에 복사됩니다.

---

## 의존성 그래프 내 위치

**독립 인프라 패키지** -- RT 환경을 구성하고 검증합니다.

```
repo_scripts  <- 독립 (ament_cmake만 의존)
    ^
    |-- build.sh / install.sh  (rt_common.sh 공유 함수 사용)
    |-- integrated_bringup           (launch 파일에서 cpu_shield.sh 호출)
    '-- 빌드 시스템            (build.sh에서 check_rt_setup.sh 호출)
```

---

## 격리 환경 배포 가이드

개발 PC 이외의 머신(신규 개발용 / 실 RT 제어 PC / CI runner) 에 동일 격리 환경을 재현할 때 참고합니다.

### 신규 머신 (수동) 재현 순서

```bash
# 1. 저장소 clone
mkdir -p ~/ros2_ws/rtc_ws/src
cd ~/ros2_ws/rtc_ws
git clone <repo-url> src/rtc-framework

# 2. apt 의존성 (install.sh 에 정의됨 — 빌드까지 하려면 그냥 install.sh 실행)
./src/rtc-framework/install.sh --skip-build   # deps/install 빌드 + apt 설치만

# 3. Python venv (uv 사용 — install.sh 가 자동 부트스트랩하지만 수동 시:)
#    uv 가 없으면: curl -LsSf https://astral.sh/uv/install.sh | sh
#    --python 3.12 명시: ROS Jazzy / Ubuntu 24.04 SSoT. runtime PC 에 다른 control project
#    용 python3.9/3.10 이 PATH 앞에 있어도 uv 가 그걸 잡지 않도록 고정.
uv venv --python 3.12 --system-site-packages .venv
source src/rtc-framework/repo_scripts/scripts/setup_env.sh
uv pip sync src/rtc-framework/requirements.lock

# 4. workspace 빌드
./src/rtc-framework/build.sh sim        # 또는 robot / full
```

`install.sh` 가 `repo_scripts/scripts/setup_env.sh` 를 자동 source 하고 `repo_scripts/scripts/build_deps.sh` 를 호출하므로, 단계 1-2 는 명령 하나로 가능합니다.

### 실 RT 제어 PC 차이점

| 항목 | 개발 PC | 실 RT PC |
|------|--------|---------|
| 커널 | vanilla `6.x-generic` | `linux-image-rt-generic` (`-rt` 접미사 필수) |
| CPU 격리 | 해제 | `isolcpus=...` + `nohz_full=...` (setup_grub_rt.sh) |
| IRQ affinity | 기본 | `setup_irq_affinity.sh` 실행 |
| 지터 기준 | 수~수십 us | 수 us (cyclictest 99.99 %ile) |
| GPU / Python 패키지 | CUDA+GPU 휠 | `+cpu` 휠로 재컴파일 (`requirements.in` 에 `--extra-index-url https://download.pytorch.org/whl/cpu` 추가 후 `uv pip compile` 재실행) |

RT PC 에서 위 RT 준비가 끝나면 `cyclictest -p 90 -t 1 -n -i 1000 -D 300s` 로 jitter 기준선을 잡은 뒤 워크스페이스 RT 루프 실행.

### CI 파이프라인 스니펫 (GitHub Actions)

```yaml
jobs:
  build:
    runs-on: ubuntu-24.04
    container: ros:jazzy-ros-base
    steps:
      - uses: actions/checkout@v4
        with: { path: src/rtc-framework }

      - name: Install apt deps
        run: |
          apt-get update
          apt-get install -y python3.12 python3.12-venv python3-dev \
            git curl ca-certificates \
            libeigen3-dev libyaml-cpp-dev libtinyxml2-dev \
            ros-jazzy-pinocchio ros-jazzy-proxsuite ros-jazzy-hpp-fcl \
            ros-jazzy-eigenpy ros-jazzy-behaviortree-cpp \
            ros-jazzy-ament-cmake-gtest python3-colcon-common-extensions \
            python3-vcstool
          # numpy/scipy/matplotlib/pandas/PyQt5 는 requirements.lock 에서 venv 안으로 설치

      - name: Import + build isolated deps
        run: |
          mkdir -p deps/src
          vcs import deps/src < src/rtc-framework/deps.repos
          (cd deps/src/aligator && git submodule update --init --recursive --depth 1)
          bash src/rtc-framework/repo_scripts/scripts/build_deps.sh

      - name: Install uv + Python venv (hash-verified sync)
        run: |
          curl -LsSf https://astral.sh/uv/install.sh | sh
          export PATH="$HOME/.local/bin:$PATH"
          uv venv --python 3.12 --system-site-packages .venv
          . .venv/bin/activate
          uv pip sync src/rtc-framework/requirements.lock

      - name: colcon build + test
        run: |
          . src/rtc-framework/repo_scripts/scripts/setup_env.sh
          colcon build
          colcon test && colcon test-result --verbose
```

`actions/cache` 로 `deps/install/` 전체를 캐싱하면 aligator 재빌드 (~15-20분) 를 스킵할 수 있습니다.

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
