# repo_scripts

RT 호스트 구성·검증, 개발 쉘 환경, 의존성 격리를 맡는 셸 스크립트 모음. **유형**: 스크립트 폴더

## 개요

PREEMPT_RT 커널 빌드, CPU 격리, IRQ affinity, 네트워크 최적화, NVIDIA 공존, 환경 검증, 그리고 ABI 에 민감한 C++ 의존성 (fmt · mimalloc · aligator) 의 소스 빌드·격리 검증을 로봇에 의존하지 않고 제공한다. 모든 `setup_*.sh` 는 멱등이다.

**대부분의 사용자는 이 폴더를 직접 만질 일이 없다. [`setup_env.sh`](scripts/setup_env.sh) 와 [`check_rt_setup.sh`](scripts/check_rt_setup.sh) 두 개만 알면 된다.** 나머지는 `install.sh` · `build.sh` · launch 가 부르거나 RT PC 를 처음 세울 때 한 번 쓴다.

소비자: 루트의 [`build.sh`](../build.sh) · [`install.sh`](../install.sh) (`rt_common.sh` 공유), `integrated_bringup` 의 launch (`cpu_shield.sh` 호출).

## 빠른 시작

아래 `./` 로 시작하는 명령은 전부 `repo_scripts/scripts/` 안에서 실행한다고 본다.

```bash
cd ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts
```

### 개발 쉘 준비

`build.sh` · `install.sh` 는 `setup_env.sh` 를 자동으로 source 한다. `ros2 launch` · `colcon test` 같은 대화형 명령은 쉘마다 직접 source 해야 한다.

```bash
source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh
./check_rt_setup.sh --summary     # RT 환경 점검 (카테고리별 한 줄)
```

### RT PC 처음 세우기

한 번에 끝내려면 저장소 루트에서 `./install.sh robot --rt` ([docs/rt-optimization.md](../docs/rt-optimization.md) §1, 커널 설치와 재부팅은 제외) 를 쓰고, 한 단계씩 하려면 아래 순서로 실행한다.

```text
[1회 설정 -- 순서대로]
  1. sudo ./build_rt_kernel.sh --batch   # PREEMPT_RT 커널 빌드/설치 -> 재부팅
  2. sudo ./setup_grub_rt.sh             # GRUB RT 파라미터          -> 재부팅
  3. sudo ./setup_irq_affinity.sh        # IRQ 를 OS 코어에 고정
  4. sudo ./setup_udp_optimization.sh    # NIC/네트워크 최적화
  5. sudo ./setup_cpu_governor.sh        # governor -> performance
  (NVIDIA GPU 가 있으면 -- 2 · 5 와 display 설정을 자동 호출, 재부팅)
  6. sudo ./setup_nvidia_rt.sh
  (선택)
  7. sudo ./setup_display_rt.sh          # 디스플레이 최적화
  (CAN FD 손 -- udp_hand_driver protocol_version 2)
  8. ./install_ixxat_socketcan.sh build && sudo ./install_ixxat_socketcan.sh install
  9. sudo ./setup_can_interface.sh       # can0 1M/8M fd on + systemd + can-utils

[빌드 전 검증]   ./check_rt_setup.sh --summary
[실행]           ros2 launch integrated_bringup robot_ur5e_p1a.launch.py
                 ./verify_rt_runtime.sh --watch 3   # 실행 중 스레드 모니터링
```

`cpu_shield.sh` 는 launch 가 먼저 알아서 무장하므로 따로 켤 필요가 없다 (수동: `sudo ./cpu_shield.sh on --robot` / `off`).

### 격리 환경 배포 가이드

개발 PC 이외의 머신 (신규 개발용 · 실 RT 제어 PC · CI runner) 에 같은 격리 환경을 재현할 때 쓴다.

```bash
# 1. 저장소 clone
mkdir -p ~/ros2_ws/rtc_ws/src
cd ~/ros2_ws/rtc_ws
git clone <repo-url> src/rtc-framework

# 2. apt 의존성 + deps/install + .venv 생성·sync (빌드 제외)
#    venv 를 손으로 만들 때: README.md "Python 의존성 sync"
./src/rtc-framework/install.sh --skip-build

# 3. workspace 빌드 (테스트 제외가 기본 — 개발용이면 --tests)
./src/rtc-framework/build.sh sim        # 또는 robot / full
```

`--skip-build` 를 빼면 단계 2-3 은 `install.sh` 하나로 끝난다. `./install.sh verify` 는 이 호스트의 job 수 · 메모리 상한 · ccache 를 알린다.

**빌드가 기대는 시스템 패키지** — 없으면 해당 장치만 꺼지고 빌드는 된다. 메모리 상한은 `systemd-run` + user session (`systemd` · `libpam-systemd` · `dbus-user-session`) 이 필요하고 없으면 `Memory cap: unavailable` 경고 후 상한 없이 빌드한다. ccache 와 `python3-pytest-xdist` 는 `install.sh` 가 설치한다.

**실 RT 제어 PC** 는 커널이 `linux-image-rt-generic` (`-rt` 접미사 필수) 이고 위 RT 설정 전부가 필요하며, 지터 기준이 개발 PC 의 수~수십 us 에서 수 us (cyclictest 99.99 %ile) 로 올라간다. Python 은 GPU 휠 대신 `+cpu` 휠로 다시 컴파일한다 (`requirements.in` 에 `--extra-index-url https://download.pytorch.org/whl/cpu` 추가 후 `uv pip compile` 재실행). 준비가 끝나면 `cyclictest -p 90 -t 1 -n -i 1000 -D 300s` 로 지터 기준선을 잡는다.

## 인터페이스

### 스크립트 목록

RT 호스트 설정의 절차와 근거는 [docs/rt-optimization.md](../docs/rt-optimization.md) §2 가 갖는다. `setup_grub_rt.sh` · `setup_cpu_governor.sh` · `setup_nvidia_rt.sh` · `setup_display_rt.sh` 는 `--help` 를 받는다.

**RT 호스트 설정 (1회, 전부 sudo)**

| 스크립트 | 용도 |
|---------|------|
| `build_rt_kernel.sh` | PREEMPT_RT 커널 (6.12+) 빌드·설치. `--batch` · `--profile <auto·desktop-amd·desktop-intel·nuc>` · `--status` · `--force-step N` · `--with-nvidia`. [§2.2](../docs/rt-optimization.md#22-rt-커널) |
| `setup_grub_rt.sh` | GRUB RT 파라미터의 단일 출처 (`nohz_full` 값은 `get_rt_cores_with_siblings()` 가 정한다. `isolcpus` 는 쓰지 않고 `cpu_shield.sh` 가 대체). [§2.3](../docs/rt-optimization.md#23-grub-옵션) |
| `setup_irq_affinity.sh [NIC]` | 하드웨어 IRQ 를 OS 코어에 고정하고 RT 코어에 잔여 IRQ 가 없는지 검증. [§2.5](../docs/rt-optimization.md#25-irq-affinity) |
| `setup_udp_optimization.sh [NIC]` | 코얼레싱 · GRO/LRO/TSO/GSO 끔, 링 버퍼 1024, `rmem_max`/`wmem_max`=2GB, `rtc-udp-optimization.service` 로 재적용. CycloneDDS `SocketReceiveBufferSize` (8MB) 는 `rmem_max` 이하여야 한다 |
| `setup_cpu_governor.sh` | governor 를 performance 로 (`cpu-governor-performance.service`). [§2.6](../docs/rt-optimization.md#26-cpu-governor) |
| `setup_nvidia_rt.sh` | NVIDIA 와 RT 커널의 공존 (7단계). 마지막에 `setup_grub_rt.sh` · `setup_display_rt.sh` · `setup_cpu_governor.sh` 를 자동 호출. [§2.7](../docs/rt-optimization.md#27-nvidia-gpu-가-있을-때) |
| `setup_display_rt.sh` | X11 안티 티어링 (`/etc/X11/xorg.conf.d/20-nvidia-antitear.conf`) + compositor renice -10 서비스. headless 에서도 설정 파일을 설치 |

**CAN FD 호스트 (proto2 손, `udp_hand_driver`)**

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `setup_vcan.sh [IFACE]` | 개발 PC 의 가상 CAN FD (`vcan0`, MTU 72) — `rtc_communication` CAN 테스트와 proto2 SIL 이 요구. `rtc-vcan-<iface>.service` 가 재부팅 뒤 재생성 | 필수 |
| `install_ixxat_socketcan.sh <build·install·status>` | IXXAT USB-to-CAN FD 드라이버 `ix_usb_can` 을 실행 커널 헤더로 빌드 · 설치 (DKMS 아님 — 커널을 바꾸면 다시). 소스는 `<rtc_ws>/deps/src/` | `install` 만 |
| `run_hand_canfd_gate.sh [--sil] [--iface · --sides · --duration · --rate · --recv-timeout]` | 손 CAN FD 게이트 한 run: 버스 기록 · 노드 기동 · E-Stop 종료 · 손마다 판정 ([testing.md](../docs/testing.md)) | 불필요 |
| `setup_can_interface.sh [IFACE] [--bitrate · --dbitrate · --txqueuelen · --restart-ms]` | `can0` 를 1 M / 8 M BRS `fd on` 으로 올리고 `rtc-can-<iface>.service` + udev 로 재부팅 · 재연결마다 재적용, can-utils 설치. 기본값은 제안값이고 실기 측정이 고친다 | 필수 |

**런타임 · 검증**

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `cpu_shield.sh` | 런타임 CPU 격리 (`on` · `off` · `status` · `adopt <pid>`) | `status` 외 필수 |
| `check_rt_setup.sh` | 정적 RT 환경 검증 (9개 카테고리, read-only) | `--benchmark` 만 |
| `verify_rt_runtime.sh` | 실행 중 스레드의 스케줄링 · affinity · 메모리 잠금 검증 (7개 카테고리) | 선택 |
| `with_verify_hold.sh <명령> [인자…]` | 명령이 도는 동안 Stop hook 의 빌드·테스트 요구를 미룬다 (Claude Code 전용 보조, install 대상 아님) | 불필요 |

**개발 쉘 · 의존성 격리 (sudo 불필요)**

| 스크립트 | 용도 |
|---------|------|
| `setup_env.sh` | 쉘 활성화: ROS + `deps/install` + `.venv` + `install/` overlay + colcon defaults |
| `build_deps.sh` | `../deps.repos` 의 fmt · mimalloc · aligator 를 `<rtc_ws>/deps/install/` 로 소스 빌드 |
| `verify_isolation.sh` | `install/` 의 모든 ELF 가 격리 밖 경로를 참조하지 않는지 검사 |

**트레이싱.** `timeline.sh` (LTTng CTF → Chrome Trace JSON) 와 `enroll_lttng_mok.sh` (Secure Boot 에서 lttng-modules DKMS 서명) 의 사용법은 [docs/tracing.md](../docs/tracing.md).


### cpu_shield.sh

`isolcpus` 없이 재부팅 없이 RT 코어를 격리한다 (cset shield 필수 — `cset` 이 없으면 실패한다: `sudo apt-get install -y cpuset`).

```bash
sudo ./cpu_shield.sh on --robot [--profile <id>]   # 활성화 (--sim 도 범위는 같다)
sudo ./cpu_shield.sh off                           # 해제
./cpu_shield.sh status                             # 상태 (sudo 불필요)
```

- 범위는 `get_cm_shield_cpus()` (RT ∪ nrt, OS 코어 제외) 이고 non-SMT 는 전 tier `1-3` 이다. `arm_driver` · `hand_driver` 는 shield 밖 전용 코어의 별도 프로세스다.
- `integrated_bringup` 의 launch 5개 (`robot_ur5e_p1a` · `robot_ur5e_p1b` · `sim_ur5e_p1a` · `sim_ur5e_p1b` · `sim_iiwa7_leap`) 가 노드 기동 **전에** 자동 호출하므로 launch 시작이 cset 소요만큼 늦다 (정상). `rtc_mujoco_sim` 단독 launch 는 호출하지 않고, `build.sh` · `install.sh` 는 빌드 전에 shield 를 푼다.
- `adopt <pid>` (launch 가 CM 을 shield 안으로 옮길 때) 의 종료 코드가 계약이다: 양성 no-op (cset 미설치 · shield 비활성 · PID 부재) 은 0, 필요했는데 실패하면 non-zero 이며 launch 는 ACTIVATE 를 거부한다.

### check_rt_setup.sh · verify_rt_runtime.sh

```bash
./check_rt_setup.sh --summary    # 카테고리별 한 줄 (--json: CI용, --fix: 수정 명령, --benchmark: cyclictest)
./verify_rt_runtime.sh --watch 5 # 5초 간격 연속 모니터링 (--summary · --json 도 가능)
```

| 스크립트 | 검증 | 종료 코드 |
|---|---|---|
| `check_rt_setup.sh` (정적, read-only) | 커널 · CPU 격리 · 스케줄러 · GRUB · RT 권한 · IRQ · 네트워크 · NVIDIA · 주파수 (9개) | 0 PASS · 1 경고 · 2 실패 |
| `verify_rt_runtime.sh` (실행 중) | 프로세스 · 스케줄링 정책 · affinity · 메모리 잠금 · 컨텍스트 스위치 · 마이그레이션 · 스로틀링 (7개) | 0 · 1 · 2 · 3 컨트롤러 미실행 |

`arm_driver` · `hand_driver` 는 별도 프로세스라 process comm 으로 찾아 따로 검증한다 (sim 은 driver 가 없어 SKIP). 기대값 override: `RTC_ARM_DRIVER_RT_PRIO` (기본 50) · `RTC_HAND_AUX_SLOT` · `RTC_ARM_DRIVER_COMM` / `RTC_HAND_DRIVER_COMM` (다른 드라이버의 comm 후보, 공백 구분). 판정 로직은 `rt_common.sh` 의 `rt_classify_external_*` 주석, 해석은 [rt-optimization.md §4](../docs/rt-optimization.md#4-확인).

### setup_env.sh

```bash
source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh
```

Source 순서는 ROS Jazzy → `deps/install` (+ ONNX Runtime) → `.venv` → workspace overlay (있을 때만) 다.

| export | 값 | 용도 |
|------|-----|-----|
| `RTC_DEPS_PREFIX` | `<rtc_ws>/deps/install` | aligator · fmt · mimalloc 격리 prefix |
| `CMAKE_PREFIX_PATH` · `LD_LIBRARY_PATH` · `PKG_CONFIG_PATH` | `$RTC_DEPS_PREFIX` 하위 + `/opt/onnxruntime` | 의존성 검색 |
| `MUJOCO_DIR` · `mujoco_ROOT` | `/opt/mujoco-3.x.x` (자동 탐색) | `find_package(mujoco)` |
| `COLCON_DEFAULTS_FILE` | `<repo>/.colcon/defaults.yaml` | cwd 와 무관하게 colcon 기본 옵션 적용 |
| `MAKEFLAGS` | `-j<N>` (이미 `-j` 가 있으면 그대로) | plain `colcon build` 의 make job 수 — "빌드 병렬도와 메모리" |
| `VIRTUAL_ENV` | `<rtc_ws>/.venv` | Python venv 활성화 |

**Plain `colcon build` 호환성** (`build.sh` 우회 워크플로). `setup_env.sh` 만 source 하면 `colcon build --symlink-install` 로 단독 빌드할 수 있다. `.colcon/defaults.yaml` 이 `--symlink-install` · Release · `compile_commands` · 테스트 제외 (`-DBUILD_TESTING=OFF`) 를 자동 적용한다. 단 CMake 가 쓸 인터프리터는 venv 유무와 무관하게 고정해야 한다 (`build.sh` 는 `append_cmake_python_args` 로 자동 처리).

CLI 의 `--cmake-args` 는 defaults 의 `cmake-args` 목록에 덧붙지 않고 **그 목록을 통째로 대체**한다. 그래서 `-DPython3_EXECUTABLE` 만 넘기면 새 빌드 트리가 `CMAKE_BUILD_TYPE` 없이 (최적화 없이) configure 된다. defaults 목록을 함께 적는다 (`.colcon/defaults.yaml` 과 같게 유지):

```bash
( cd ~/ros2_ws/rtc_ws && source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 )
```

`build.sh` 의 모드별 패키지 셀렉션 · `compile_commands.json` 머지 · `check_rt_setup.sh` 호출은 colcon 단독에서는 빠진다. 근거는 [build-rationale.md](../docs/reference/build-rationale.md).

#### workspace 에 다른 저장소가 함께 있을 때

이 저장소의 빌드 스크립트는 다른 저장소의 패키지 이름을 갖지 않는다. `<rtc_ws>/src` 에 함께 둔 저장소는 경로에 따라 다르게 다뤄진다.

| 경로 | 빌드 대상 | 다른 저장소의 패키지 |
|---|---|---|
| `build.sh` | `rt_common.sh` 의 고정 목록 (`--packages-select`) | 빌드하지 않는다 |
| ws root 의 plain `colcon build` | `src/` 아래의 모든 패키지 | 함께 빌드한다 |

- ws root 의 첫 빌드는 plain `colcon build` 로 한다. `build.sh -c` 는 ws root 의 `build/` · `install/` · `log/` 를 **통째로** 지워 다른 저장소의 설치본도 사라지므로, 그 뒤에는 plain `colcon build` 로 전체를 다시 빌드한다.
- 한 저장소만 고르려면 `--base-paths src/<저장소>` 를 쓴다 (`colcon test` · `test-result` 도 같다). 패키지 이름을 적지 않으므로 패키지가 늘거나 줄어도 명령이 바뀌지 않는다.
- 자기 빌드 절차와 환경을 가진 저장소는 별도 workspace 에 두고, 그 패키지를 쓰는 터미널에서만 그 `install/setup.bash` 를 이 workspace 다음에 source 한다. 같은 workspace 에서 빌드하면 colcon defaults 와 `.venv` 가 섞인다.
- 로봇 config 가 `package://<패키지>/...` 로 가리키는 패키지가 없으면 launch 가 `package '<패키지>' not found` 로 멈춘다 ([integrated_bringup/README.md](../integrated_bringup/README.md)).

### build_deps.sh · verify_isolation.sh

```bash
source setup_env.sh
./build_deps.sh          # fmt 11.1.4 · mimalloc 2.1.7 · aligator 0.19.0 -> <rtc_ws>/deps/install/
./verify_isolation.sh    # install/ 의 ELF RPATH/ldd 누수 검사 (0 통과 · 1 누수)
```

`build_deps.sh` 는 `deps/src` 가 비어 있으면 `vcs import` 를 자동으로 하고, RPATH 는 시스템 `/usr/local` 을 보지 않게 잡는다. 처음은 15~20분 (aligator 가 병목) 이고 job 수는 `build.sh` 와 같은 knob 을 쓴다. `verify_isolation.sh` 는 ROS · `deps/install` · `install/` · 시스템 lib · `/opt/onnxruntime*` · `/opt/mujoco-*` 밖을 참조하면 누수로 본다.

### rt_common.sh

모든 스크립트가 `source` 하는 라이브러리 (`build.sh` · `install.sh` 포함) 이며 직접 실행할 수 없다. 함수마다의 계약은 파일 안 주석이 갖는다. 개발 쉘에서 직접 부를 만한 것은 아래와 같다.

예: `source repo_scripts/scripts/lib/rt_common.sh; get_role_slot rt_control "$(get_physical_cores)"`

| 함수 | 용도 |
|------|------|
| `get_physical_cores` · `detect_physical_nic` | 물리 코어 수 (SMT 제외) · 물리 NIC 자동 탐색 |
| `get_default_build_jobs` · `resolve_build_makeflags [N]` | 기본 make job 수 · export 할 `MAKEFLAGS` (CLI `-j` > `RTC_BUILD_JOBS` > 기존 `-j` > 기본값) |
| `get_build_mem_max` · `print_build_host_summary` | 빌드 RAM 상한 · job 수 / 메모리 상한 / ccache 가용성 보고 |
| `get_system_python` | CMake 와 `.venv` 가 쓰는 배포판 python (`RTC_SYSTEM_PYTHON` 으로 덮어씀) |
| `shield_isolation_method` | 지금 격리를 거는 것: `cset <cpus>` / `isolcpus <cpus>` / `none` |
| `get_base_packages` · `get_robot_packages` · `print_thread_layout` | `build.sh` · `install.sh` 공유 패키지 목록 · 이 호스트의 코어별 스레드 레이아웃 출력 |

#### RT/MPC 코어 레이아웃 함수

값의 SSoT 는 [config/thread_layout.yaml](config/thread_layout.yaml) 이고 tier 의존 헬퍼는 그 manifest 에서 [thread_layout_generated.sh](scripts/lib/thread_layout_generated.sh) 로 **생성**된다 (직접 고치면 `gen_thread_layout.py --check` 가 막는다). 모두 `$1` 로 물리 코어 수를 받아 이 머신에 없는 tier 도 출력할 수 있고, 반환값은 kernel logical CPU 가 아니라 **slot** 이다 (`slot_to_logical_cpu` 로 변환). launch profile 의 이력은 [architecture-rationale.md](../docs/reference/architecture-rationale.md).

| 함수 | 설명 |
|------|------|
| `get_role_slot` · `get_role_policy` · `get_role_priority` | `<role> [ncpu]` 로 한 role 의 배치 조회 (그 tier 에 없는 role 은 **비0 종료**) |
| `get_rt_cores` · `get_mpc_cores` · `get_nrt_cores` · `get_os_cores` · `get_arm_driver_slot` · `get_hand_driver_slot` | RT 그룹 · MPC · nrt lane · OS/DDS/IRQ 코어 (Core 0) · 외부 driver 프로세스의 slot |
| `rtc_expected_threads` · `rtc_forbidden_threads` | `verify_rt_runtime.sh` 의 기대 스레드 · profile 이 떨어뜨려 존재하면 FAIL 인 스레드 |
| `rtc_layout_profiles` · `rtc_default_profile` · `rtc_is_layout_profile` | launch profile 목록 · 기본값 (`mpc_on`) · 유효성 (모르는 id 는 비0) |
| `get_rt_cores_with_siblings` | `nohz_full` / `rcu_nocbs` 값 (SMT 형제 포함, profile 을 타지 않는 최광 집합) |
| `get_rt_shield_cpus` · `get_cm_shield_cpus` | RT 만 / CM 프로세스 전체 (RT ∪ nrt) 의 **logical CPU** 격리 범위 |

profile 은 `get_rt_cores` · `get_mpc_cores` · `rtc_*_threads` 에서 `$2`, shield 계열에서 `$1` 로 받는다. 표의 `(mpc_off)` 열은 MPC 를 쓰지 않는 launch 에서 `mpc_main` 코어가 빠진 결과다.

tier 별 값 (SSoT: `repo_scripts/config/thread_layout.yaml` — 아래 표는 그 manifest 에서 생성된다):

<!-- BEGIN GENERATED: thread-layout-tiers -->
<!-- Generated from repo_scripts/config/thread_layout.yaml by repo_scripts/scripts/gen_thread_layout.py. Do not edit by hand. -->

| tier (물리 코어) | `get_rt_cores()` | `get_rt_cores()` (mpc_off) | `get_mpc_cores()` | `get_nrt_cores()` | `get_os_cores()` | arm / hand slot |
|---|---|---|---|---|---|---|
| ≤5 | `1,2,3` | `1,2` | `3` | `0` | `0` | 0 / 0 |
| 6–7 | `1,2,3` | `1,2` | `3` | `2` | `0` | 4 / 4 |
| 8–9 | `1,2,3` | `1,2` | `3` | `2` | `0` | 4 / 5 |
| 10–11 | `1,2,3` | `1,2` | `3` | `2` | `0` | 4 / 5 |
| 12–13 | `1,2,3` | `1,2` | `3` | `2` | `0` | 4 / 5 |
| 14–15 | `1,2,3` | `1,2` | `3` | `2` | `0` | 4 / 5 |
| 16+ | `1,2,3` | `1,2` | `3` | `2` | `0` | 4 / 5 |

<!-- END GENERATED: thread-layout-tiers -->

≤5코어 / 6–7코어는 degraded 라 RT 결정성을 보장하지 않는다 (nrt · driver 가 Core 0 이나 서로의 코어를 공유하고 mpc 가 CFS 로 강등된다). 8코어 이상부터 arm_driver / hand_driver 가 전용 슬롯 (**4 · 5**) 을 얻고, 드라이버는 shield 에서 제외돼 있어 `nohz_full` 과 무관하다. Core 0 은 OS/DDS/IRQ 전용이고 RT cluster 는 Core 1 부터 시작한다.

## 설정

### `thread_layout.yaml`

CPU 레이아웃의 선언형 manifest ([config/thread_layout.yaml](config/thread_layout.yaml)). tier → slot / policy / priority 와 launch profile 을 정하고, `python3 repo_scripts/scripts/gen_thread_layout.py --write` 가 C++ · shell · Python · 이 README 의 생성 블록을 만든다 (`--check` 는 드리프트 게이트).

### 빌드 병렬도와 메모리

이 repo 는 패키지를 **한 번에 하나씩** 빌드하고 (`.colcon/defaults.yaml` 의 `parallel-workers: 1`) 병렬도를 make job 수 하나로 정한다. 기본값은 `min(물리 코어, RAM / 4 GB)` 다 (6C/32GB → 6, 16C/32GB → 8).

| knob | 적용 범위 |
|------|-----------|
| `./build.sh -j N` · `./install.sh -j N` | 그 실행 (`install.sh` 는 `build_deps.sh` 에도 건다) |
| `RTC_BUILD_JOBS=N` | `build.sh` · `build_deps.sh` 는 실행 시점에, plain `colcon` 은 `setup_env.sh` 를 source 하는 시점에 읽는다. 이미 source 한 셸은 `MAKEFLAGS=-jN colcon build …` |
| `MAKEFLAGS` 의 `-j` | 위 둘이 없으면 그대로 존중하며 그 값이 **전체** 병렬도다 (셸 rc 에 `MAKEFLAGS=-j2` 가 남아 있으면 2 job) |
| `RTC_BUILD_MEM_MAX` | 빌드 RAM 상한: `24G` · `20000M` · `60%` / `off`. 직접 주면 swap 없이 엄격하다. 단위 없는 수 · K 단위 · 100% 초과 · 64M 미만은 거부 |
| 기본 메모리 상한 | RAM 의 75% 와 시작 시점 `MemAvailable` 의 90% 중 작은 쪽 + RAM 25% 의 swap |

- `build.sh` · `build_deps.sh` 는 빌드를 `MemoryMax` 가 걸린 systemd user scope 안에서 돌려, 한도를 넘으면 호스트가 아니라 **그 빌드만** 끝난다. 상한을 걸 수 없는 호스트는 이유를 알리고 상한 없이 빌드한다.
- plain `colcon` 에는 상한이 없다: `systemd-run --user --scope -p MemoryMax=75% -p MemorySwapMax=8G colcon build …` (`MemorySwapMax` 의 % 는 swap 크기 대비라 크기로 적는다). 실제 값은 `build.sh` 의 `Parallelism:` · `Memory cap:` 줄, plain `colcon` 은 `echo $MAKEFLAGS` 로 본다.
- 기본값의 근거 (실측 표 · job 당 4 GB · OOM 판정) 는 [build-rationale.md](../docs/reference/build-rationale.md).

### 빌드 시간 줄이기

**테스트는 기본으로 빌드하지 않는다.** 컴파일 시간의 절반 이상이 테스트라서 `./build.sh` · `./install.sh` · plain `colcon build` 모두 `-DBUILD_TESTING=OFF` 가 기본이다 (6C/12T 클린 빌드 18분 49초 → 9분 44초). 켜는 법은 다음과 같다.

- `build.sh` · `install.sh` — `--tests` (한 번), 또는 개발 셸에서 `export RTC_BUILD_TESTS=on`
- plain `colcon` — `--cmake-args … -DBUILD_TESTING=ON`. defaults 목록을 대체하므로 "Plain `colcon build` 호환성" 의 목록을 함께 적는다

**`colcon test` 전에는 `--tests` 로 빌드한다.** 테스트 없이 빌드한 패키지의 `colcon test` 는 실패가 아니라 **테스트 0개**를 보고해 통과처럼 읽힌다 (증상이 이것이면 `./build.sh --tests` 로 다시 빌드한다). `build.sh` 는 매 빌드 `Tests:` 줄로 선택을 알리고 `-DBUILD_TESTING` 을 `ON`/`OFF` 로 명시한다 (`RTC_BUILD_TESTS` 는 `on`·`1`·`true` / `off`·`0`·`false`).

**ccache.** 깔려 있으면 `build.sh` 가 자동으로 쓴다 (`--no-ccache` · `RTC_CCACHE=off` 로 끈다). 이득은 `build.sh -c` · 브랜치 전환처럼 같은 TU 를 다시 컴파일할 때뿐이다.

## 디렉토리

```
repo_scripts/
├── config/    레이아웃 manifest · 문서/README 예산
├── scripts/   실행 스크립트 (위 목록) · 검증기 · 생성기
│   └── lib/   source 전용 라이브러리 · install.sh helper
├── test/      셸 단위 테스트 · fixture
├── CMakeLists.txt
└── package.xml
```

스크립트마다의 설명은 각 파일 상단 주석이, 배포 대상은 `CMakeLists.txt` 의 `install(PROGRAMS …)` 가 갖는다.

## 문제 해결

### 흔한 실패와 감별 (colcon cwd · source · venv)

[AGENTS.md](../AGENTS.md) §9 의 두 hard rule 이 실제로 깨지는 경로다. 규칙은 헌법이, 재발 경로의 서술은 [build-rationale.md](../docs/reference/build-rationale.md) 가 갖는다.

| 증상 | 원인 | 조치 |
|---|---|---|
| repo 안에 `build/` · `install/` · `log/` 가 생겼다. 검증 결과가 설명 불가하게 엇갈린다 | 패키지 dir 에서 colcon 을 쳤다 (cwd drift). 상대경로 source 가 조용히 실패한 경우도 있다 | `ls src/rtc-framework/{build,install,log}` 로 확인해 지운다. 항상 서브셸 표준형: `( cd ~/ros2_ws/rtc_ws && source <절대경로>/setup_env.sh >/dev/null 2>&1 && colcon … )` |
| 빌드가 정상 시작했다가 CMake 에서 `No module named 'ament_package'` | `source setup_env.sh … \| tail` 처럼 `source` 를 파이프에 넣어 env 가 부모 셸에 안 남았다 | 출력은 `>/dev/null 2>&1` 로 줄이고 파이프는 뒤따르는 명령에만 건다. 감별: `/usr/bin/python3 -c "import ament_package"` |
| configure 가 `No module named 'catkin_pkg'` 로 죽는다 | `FindPython` 이 venv 나 PATH 앞의 다른 `python3.X` 를 잡았다 | `-DPython3_EXECUTABLE=/usr/bin/python3` 로 고정 ("Plain `colcon build` 호환성"). `build.sh` 는 자동 |
| venv 에서 `colcon test` / `ros2 run` / `ros2 launch` 가 실패한다 | `.venv` 의 sys.path · shebang · 의존성 해석 문제 | 근본 원인을 고친다. `deactivate` · gtest 직접 실행 · `PYTHONPATH` 우회는 금지 (예외는 configure 단계뿐) |
| `Build stopped: it needed more than the memory cap (…)`. 상한 없는 빌드 (plain `colcon`) 라면 화면이 멈추거나 `Killed signal terminated program cc1plus` | 빌드가 메모리 상한에 닿았다 (또는 상한 없이 메모리가 바닥났다) | 다른 프로그램의 메모리를 비우거나 `-j N` 을 낮춘다. `Memory cap:` 줄이 평소보다 작으면 호스트가 이미 차 있다. `echo $MAKEFLAGS` 가 비면 `setup_env.sh` 를 source 하지 않은 셸이다 |
| Intel CPU 인데 `check_rt_setup.sh` 가 hybrid 오검출을 경고한다 | `cpuid` 미설치로 CPUID 감지 경로가 막혔다 | `sudo apt install -y cpuid`. 다른 경로는 [rt-optimization.md §6](../docs/rt-optimization.md#6-hybrid-cpu-nuc) |

## 더 읽기

- [docs/rt-optimization.md](../docs/rt-optimization.md) — RT 호스트 설정 절차와 확인, [rt-setup-rationale.md](../docs/reference/rt-setup-rationale.md) 는 그 근거
- [build-rationale.md](../docs/reference/build-rationale.md) — 빌드 병렬도 · 메모리 상한 · 테스트 · ccache 의 근거와 실측
- [docs/tracing.md](../docs/tracing.md) — `timeline.sh` · `enroll_lttng_mok.sh`
- [agent_docs/invariants.md](../agent_docs/invariants.md) §RT Host — RT-HOST-1~3 규범. `verify_rt_runtime.sh` 가 검증한다
- [agent_docs/architecture.md](../agent_docs/architecture.md) — Execution Contexts · 스레드 모델

## 의존성

[package.xml](package.xml) — `ament_cmake` 만 의존한다. 시스템 요구사항은 bash 4.0+, `ethtool`, `lscpu`, `sysctl`, `cset` (cpuset), `cyclictest` (rt-tests, 벤치마크 시). MIT License ([LICENSE](../LICENSE)).
