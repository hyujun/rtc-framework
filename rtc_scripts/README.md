# rtc_scripts

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **로봇 비의존(robot-agnostic) RT 시스템 구성·검증 + 의존성 격리 스크립트 모음**입니다. PREEMPT_RT 커널 빌드, CPU 격리, IRQ 어피니티, 네트워크 최적화, NVIDIA 공존 설정, 환경 검증, 그리고 ABI 민감 C++ 의존성(fmt/mimalloc/aligator) 소스 빌드 및 격리 관리를 위한 **14개 스크립트**와 공유 라이브러리를 제공합니다.

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
rtc_scripts/
├── CMakeLists.txt
├── package.xml
└── scripts/
    ├── lib/
    │   └── rt_common.sh                  <- 공유 유틸리티 라이브러리
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
    │   # ── 의존성 격리 (배포 가이드 참조) ────────────────────────
    ├── setup_env.sh                      <- 개발 쉘 env 활성화 (ROS + deps/install + venv + overlay)
    ├── build_deps.sh                     <- fmt/mimalloc/aligator 소스 빌드 -> deps/install/
    ├── verify_isolation.sh               <- ELF RPATH/ldd 격리 검증
    └── uninstall_system_deps.sh          <- robotpkg/stale /usr/local 대화형 제거 (§8 A-D)
```

---

## 스크립트 분류

### 설정 스크립트 (Setup) -- 1회 실행

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `build_rt_kernel.sh` | PREEMPT_RT 커널 다운로드/패치/빌드/설치 (7단계) | 필수 |
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

### 의존성 격리 스크립트 (Isolation) -- Level 3

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `setup_env.sh` | 매 쉘 활성화 -- ROS Jazzy + `deps/install` (fmt/mimalloc/aligator) + `.venv` + `install/` overlay. `COLCON_DEFAULTS_FILE` 도 주입 | 불필요 |
| `build_deps.sh` | `../deps.repos` 기반으로 fmt 11.1.4 / mimalloc 2.1.7 / aligator 0.19.0 을 `<rtc_ws>/deps/install/` 로 소스 빌드. deps/src 가 비어있으면 `vcs import` 자동 실행 | 불필요 |
| `verify_isolation.sh` | `install/` 의 모든 ELF 를 훑어 `/usr/local` · `/opt/openrobots` 등 격리 외부 경로 누수 검사 | 불필요 |
| `uninstall_system_deps.sh` | robotpkg 15개 · `/opt/openrobots` · `/opt/rti.com` · stale `/usr/local` 잔재 제거 (§8 A-D 대화형) | 필수 |

`build.sh` / `install.sh` 는 `RTC_DEPS_PREFIX` 미설정 시 `setup_env.sh` 를 자동으로 source 하므로, 수동 실행 없이 바로 동작합니다. 그러나 `ros2 launch` · `colcon test` 등 대화형 커맨드에서는 각자 `source setup_env.sh` 필요.

---

## 공유 라이브러리 (`rt_common.sh`)

모든 스크립트에서 `source`하는 핵심 유틸리티입니다. `build.sh`, `install.sh`에서도 공유됩니다. 직접 실행할 수 없으며, 중복 source 방지 가드가 포함되어 있습니다.

### CPU 감지 함수

| 함수 | 설명 |
|------|------|
| `get_physical_cores()` | 물리 코어 수 감지 (SMT/HT 제외). lscpu -> sysfs -> nproc 순 |
| `compute_cpu_layout()` | 코어 수 기반 RT 레이아웃 계산 (OS/RT 코어 범위, IRQ 마스크) |
| `compute_irq_affinity_mask()` | SMT-aware IRQ affinity bitmask 계산 (HT 시블링 포함) |
| `compute_expected_isolated()` | `isolcpus` 기대값 계산 (SMT 시블링 포함 범위 표기) |
| `get_os_logical_cpus()` | OS 물리 코어에 속하는 논리 CPU 번호 목록 |

### NIC/네트워크 함수

| 함수 | 설명 |
|------|------|
| `detect_physical_nic()` | 물리 NIC 자동 탐색 (가상 인터페이스 제외, UP 상태 우선) |

### 로깅 함수

| 함수 | 설명 |
|------|------|
| `setup_colors()` | 터미널 색상 변수 초기화 (비-TTY 환경에서는 빈 문자열) |
| `make_logger PREFIX [STYLE]` | 로깅 프리픽스+스타일 설정. `bracket` (기본): `[PREFIX] msg`, `emoji`: 이모지 형식 |
| `info()` / `warn()` / `error()` / `success()` / `section()` | 색상별 로깅 |
| `fatal()` | 에러 메시지 출력 후 `exit 1` (error + exit 통합) |

### 파일/시스템 유틸리티

| 함수 | 설명 |
|------|------|
| `require_root()` | root 권한 확인 (실패 시 `fatal`) |
| `write_file_if_changed()` | 멱등 파일 쓰기 (동일 시 skip, 다를 시 백업+덮어쓰기) |
| `create_oneshot_service()` | systemd oneshot 서비스 생성 헬퍼 |
| `auto_release_cpu_shield()` | 빌드 전 CPU shield 자동 해제 (cset 감지 시 해제, isolcpus는 경고만) |
| `check_workspace_structure()` | ROS2 워크스페이스 디렉토리 구조 검증 (`src/` 하위 확인) |
| `ensure_ros2_sourced()` | ROS2 환경 자동 탐색 및 소싱 (jazzy/humble/iron/rolling 순) |

### 패키지 리스트 함수

| 함수 | 설명 |
|------|------|
| `get_base_packages()` | 기본 RTC 패키지 리스트 (build.sh/install.sh 공유). Phase 5에서 `rtc_mpc`가 `rtc_urdf_bridge`와 `rtc_tsid` 사이에 추가됨. |
| `get_robot_packages()` | 로봇 전용 패키지 리스트 |

### MPC 코어 레이아웃 함수 (Phase 5)

쉘 스크립트와 `rtc_base/threading/thread_config.hpp` 사이에 **단일 소스 진실**을 유지하기 위한 헬퍼입니다. `cpu_shield.sh`, `setup_grub_rt.sh`, `setup_irq_affinity.sh`, `check_rt_setup.sh`, `verify_rt_runtime.sh`가 이 함수들을 통해 tier별 코어 배치를 질의합니다.

| 함수 | 설명 | 반환 예시 (6 / 8 / 10 / 12 / 14 / 16 코어) |
|------|------|--------------------------------------------|
| `get_mpc_cores()` | 현재 물리 코어 수에 맞는 MPC 코어 (main + workers) CSV 반환. 첫 항목이 항상 MPC main 코어. | `4` / `4` / `4,5` / `4,5,6` / `4,5,6` / `9,10,11` |
| `get_mpc_main_core()` | MPC main 코어만 (get_mpc_cores의 첫 항목). | `4` / `4` / `4` / `4` / `4` / `9` |
| `get_rt_cores()` | RT 스레드 전체 집합 (rt_control + sensor_io + udp_recv + MPC). GRUB `nohz_full`/`rcu_nocbs` 인자에 사용. | `2,3,5,4` / `2,3,5,4` / `2,3,6,4,5` / `2,3,7,4,5,6` / `2,3,7,4,5,6` / `2,3,12,9,10,11` |
| `get_os_cores()` | OS/DDS/IRQ 코어 (전체 - RT). IRQ affinity 고정 대상. | `0,1` / `0,1` / `0,1` / `0,1` / `0,1` / `0,1` |

Tier별 매핑 (2026-04 unified layout):
- **≤4코어**: MPC `SCHED_OTHER nice=-5`, Core 3 공유 (degraded).
- **5–7코어**: Core 4 공유 (MPC FIFO 60 > logger OTHER).
- **8–9코어**: **Core 4 dedicated** (udp_recv 5, logger 6, aux+publish 7).
- **10–11코어**: **Core 4 main + Core 5 worker 0** (dedicated UDP 6 / logger 7 / aux+pub 8, Core 9 spare).
- **12–13코어**: **Core 4 main + Core 5-6 workers** (dedicated UDP 7 / logger 8 / aux+pub 9, Cores 10-11 spare).
- **14–15코어**: 12-core와 동일한 RT 배치 + **Core 10 MuJoCo sim dedicated**, Cores 11-13 spare.
- **16+코어**: **Core 9 main + Core 10-11 workers** (legacy Option A: udp→12, logger→13, aux/publish→14, sim→15).

이전 10/12-core 구성은 cset shield "user"를 Core 2-6에 고정하고 rtc_controller_manager를 system cpuset(Core 7-9)로 몰아넣어 **8-core보다 적은 3코어만** RT에 쓰는 구조였습니다. 14-core는 12-core 분기에 흡수되어 Cores 12-13을 완전히 미사용. 새 레이아웃은 단조성 불변식 — "물리 코어가 증가하면 per-thread 격리는 절대 감소하지 않는다" — 을 `rtc_base/test/test_mpc_thread_config.cpp::TierIsolationMonotonicity`로 회귀 방지합니다.

---

## 스크립트 상세

### build_rt_kernel.sh

PREEMPT_RT 커널을 소스에서 빌드하고 설치합니다. 7단계 파이프라인으로 체크포인트 기반 재개를 지원합니다.

```bash
sudo ./build_rt_kernel.sh                # 대화형 빌드 (완료 단계 자동 스킵)
sudo ./build_rt_kernel.sh --batch        # 비대화형 (menuconfig 건너뜀)
sudo ./build_rt_kernel.sh --dry-run      # 다운로드 및 패치까지만 실행
sudo ./build_rt_kernel.sh --status       # 진행 상태만 확인
sudo ./build_rt_kernel.sh --verify       # 각 단계별 세부 적용 상태 진단
sudo ./build_rt_kernel.sh --force-step 5 # 5단계부터 강제 재시작
sudo ./build_rt_kernel.sh --clean        # 빌드 소스 정리 후 처음부터
sudo ./build_rt_kernel.sh --build-dir /path  # 빌드 디렉토리 지정
```

**7단계 구성:**

| 단계 | 내용 |
|------|------|
| 1/7 | 필수 빌드 패키지 설치 |
| 2/7 | 커널 소스 + RT 패치 다운로드 |
| 3/7 | 압축 해제 및 패치 적용 |
| 4/7 | 커널 설정 (PREEMPT_RT 활성화) |
| 5/7 | 커널 빌드 (`make bindeb-pkg`) |
| 6/7 | .deb 패키지 설치 |
| 7/7 | GRUB 등록 확인 및 기본 부팅 설정 |

**지원 커널:**

| Ubuntu | 커널 | RT 패치 |
|--------|------|---------|
| 24.04 | 6.8.2 | 6.8.2-rt11 |
| 22.04 | 6.6.127 | 6.6.127-rt69 |

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
| `nohz_full` | RT 코어 범위 | RT 코어에서 타이머 틱 제거 |
| `rcu_nocbs` | RT 코어 범위 | RT 코어에서 RCU 콜백 제거 |
| `processor.max_cstate` | 1 | 깊은 C-state 진입 방지 |
| `clocksource` | tsc | HPET 대비 50-100x 빠른 타이머 |
| `tsc` | reliable | TSC 불안정 감지 비활성화 |
| `nmi_watchdog` | 0 | NMI 인터럽트 제거 |
| `threadirqs` | (없음) | IRQ 핸들러 스레드화 |
| `nosoftlockup` | (없음) | soft lockup 경고 방지 |

**sysctl 설정:** `kernel.sched_rt_runtime_us=-1` 즉시 + `/etc/sysctl.d/99-rt-sched.conf`로 영구 적용

참고: `isolcpus`는 포함하지 않습니다. 런타임 동적 격리를 위해 `cpu_shield.sh`(cset shield)로 대체되었습니다.

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

**격리 방식:** cset shield 우선 -> cgroup v2/v1 cpuset fallback

**Tier 모델:**

| Tier | 스레드 | 격리 |
|------|--------|------|
| Tier 1 (RT-critical) | rt_control + sensor_io | 항상 |
| Tier 2 (RT-support) | udp_recv + logging + aux | `--robot` 모드만 |
| Tier 3 (Flexible) | sim, monitoring, build | 격리 안 함 |

**코어 수별 격리 범위:**

| 코어 수 | `--robot` | `--sim` |
|---------|-----------|---------|
| 4코어 이하 | 1-3 | 1-2 |
| 5-7코어 | 2-5 | 2-3 |
| 8-15코어 | 2-6 | 2-3 |
| 16코어+ | 4-8 | 4-5 |

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
| 1 | 프로세스 탐색 | robot bringup exec (예: `ur5e_rt_controller`) PID, 스레드 이름 매칭 |
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
source ~/ros2_ws/rtc_ws/src/rtc-framework/rtc_scripts/scripts/setup_env.sh
```

**환경 변수 export:**

| 변수 | 값 | 용도 |
|------|-----|-----|
| `RTC_DEPS_PREFIX` | `<rtc_ws>/deps/install` | aligator/fmt/mimalloc 격리 prefix |
| `CMAKE_PREFIX_PATH` | `$RTC_DEPS_PREFIX:...` | CMake 의존성 검색 순서 |
| `LD_LIBRARY_PATH` | `$RTC_DEPS_PREFIX/lib:...` | 런타임 lib resolve |
| `PKG_CONFIG_PATH` | `$RTC_DEPS_PREFIX/lib/pkgconfig:...` | .pc 기반 설정 |
| `MUJOCO_DIR` | `/opt/mujoco-3.x.x` (자동 탐색) | `rtc_mujoco_sim` 의 fallback |
| `COLCON_DEFAULTS_FILE` | `<repo>/.colcon/defaults.yaml` | cwd 와 무관하게 colcon 기본 옵션 적용 |
| `VIRTUAL_ENV` | `<rtc_ws>/.venv` | Python venv 활성화 |

**Source 순서**: ROS Jazzy → deps/install → .venv → workspace overlay (`install/setup.bash`, 있을 때만).

---

### build_deps.sh

`../deps.repos` 기반으로 fmt/mimalloc/aligator 를 소스 빌드하여 `<rtc_ws>/deps/install/` 에 설치합니다.

```bash
source setup_env.sh
~/ros2_ws/rtc_ws/src/rtc-framework/rtc_scripts/scripts/build_deps.sh
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
~/ros2_ws/rtc_ws/src/rtc-framework/rtc_scripts/scripts/verify_isolation.sh
```

**허용 경로:**
- `/opt/ros/jazzy/` (ROS distribution)
- `<rtc_ws>/deps/install/` (격리된 fmt/mimalloc/aligator)
- `<rtc_ws>/install/` (workspace overlay)
- `/lib/x86_64-linux-gnu`, `/usr/lib/x86_64-linux-gnu` (libc, libstdc++, libgomp 등)
- `/opt/onnxruntime*`, `/opt/mujoco-*` (승인된 바이너리 drop)

**종료 코드:** 0=모든 ELF 격리 통과, 1=하나 이상 누수 (상세 출력)

---

### uninstall_system_deps.sh

대화형 시스템 정리. 기존 dual-install 상태 (robotpkg + /usr/local 소스 빌드) 에서 Level 3 격리로 전환할 때 **1회** 실행. 각 단계마다 `[y/N]` 확인. 이 저장소가 이미 격리 상태로 구성돼 있다면 실행 불필요.

```bash
sudo ~/ros2_ws/rtc_ws/src/rtc-framework/rtc_scripts/scripts/uninstall_system_deps.sh
```

| 단계 | 내용 |
|------|------|
| A | `/usr/local/` stale 잔재 제거 (구 hpp-fcl/pinocchio 3.0.x, gtsam, teaser, example-robot-data, libmujoco 등) |
| B | `/usr/local/` 의 fmt/mimalloc/aligator (install_manifest 기반 uninstall) + `~/libs/` 소스 트리 삭제 |
| C-1 | `ros-jazzy-proxsuite` 설치 + `robotpkg-proxsuite` 제거 + `rtc_tsid` 재빌드 테스트 |
| C-2 | 나머지 robotpkg 14개 purge |
| C-3 | robotpkg apt source + `/opt/openrobots` 제거 |
| D | `/opt/rti.com` 제거 |

**Pre-flight**: `deps/install` + `install/` (colcon build 완료) 가 존재해야만 진행. 실패 시 자동 롤백.

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
  ros2 launch ur5e_bringup ur_control.launch.py
  ./verify_rt_runtime.sh --watch 3        # 런타임 모니터링

[시뮬레이션 실행]
  sudo cpu_shield.sh on --sim             # Tier 1만 격리
  ros2 launch ur5e_bringup mujoco_sim.launch.py

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
colcon build --packages-select rtc_scripts
source install/setup.bash
```

스크립트만 포함된 패키지이므로 바이너리는 생성되지 않습니다. `colcon build` 시 스크립트가 `install/rtc_scripts/lib/rtc_scripts/`에 복사됩니다.

---

## 의존성 그래프 내 위치

**독립 인프라 패키지** -- RT 환경을 구성하고 검증합니다.

```
rtc_scripts  <- 독립 (ament_cmake만 의존)
    ^
    |-- build.sh / install.sh  (rt_common.sh 공유 함수 사용)
    |-- ur5e_bringup           (launch 파일에서 cpu_shield.sh 호출)
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

# 3. Python venv
python3 -m venv --system-site-packages .venv
source src/rtc-framework/rtc_scripts/scripts/setup_env.sh
pip install -r src/rtc-framework/requirements.lock

# 4. workspace 빌드
./src/rtc-framework/build.sh sim        # 또는 robot / full
```

`install.sh` 가 `rtc_scripts/scripts/setup_env.sh` 를 자동 source 하고 `rtc_scripts/scripts/build_deps.sh` 를 호출하므로, 단계 1-2 는 명령 하나로 가능합니다.

### 실 RT 제어 PC 차이점

| 항목 | 개발 PC | 실 RT PC |
|------|--------|---------|
| 커널 | vanilla `6.x-generic` | `linux-image-rt-generic` (`-rt` 접미사 필수) |
| CPU 격리 | 해제 | `isolcpus=...` + `nohz_full=...` (setup_grub_rt.sh) |
| IRQ affinity | 기본 | `setup_irq_affinity.sh` 실행 |
| 지터 기준 | 수~수십 us | 수 us (cyclictest 99.99 %ile) |
| GPU / Python 패키지 | CUDA+GPU 휠 | `+cpu` 휠로 재컴파일 (`requirements.in` 에 `--extra-index-url https://download.pytorch.org/whl/cpu` 추가 후 `pip compile` 재실행) |

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
          apt-get install -y python3-venv python3-pip git \
            libeigen3-dev libyaml-cpp-dev libtinyxml2-dev \
            ros-jazzy-pinocchio ros-jazzy-proxsuite ros-jazzy-hpp-fcl \
            ros-jazzy-eigenpy ros-jazzy-behaviortree-cpp \
            ros-jazzy-ament-cmake-gtest python3-colcon-common-extensions \
            python3-vcstool

      - name: Import + build isolated deps
        run: |
          mkdir -p deps/src
          vcs import deps/src < src/rtc-framework/deps.repos
          (cd deps/src/aligator && git submodule update --init --recursive --depth 1)
          bash src/rtc-framework/rtc_scripts/scripts/build_deps.sh

      - name: Python venv + requirements.lock
        run: |
          python3 -m venv --system-site-packages .venv
          . .venv/bin/activate
          pip install --upgrade pip
          pip install -r src/rtc-framework/requirements.lock

      - name: colcon build + test
        run: |
          . src/rtc-framework/rtc_scripts/scripts/setup_env.sh
          colcon build
          colcon test && colcon test-result --verbose
```

`actions/cache` 로 `deps/install/` 전체를 캐싱하면 aligator 재빌드 (~15-20분) 를 스킵할 수 있습니다.

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
