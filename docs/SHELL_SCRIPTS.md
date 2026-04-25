# 쉘 스크립트 가이드

RTC (Real-Time Controller) 워크스페이스의 빌드, 설치, 실시간 환경 설정 쉘 스크립트 레퍼런스입니다.

---

## 개요

| 스크립트 | 위치 | 줄 수 | 설명 |
|----------|------|-------|------|
| `build.sh` | `/` (루트) | ~359 | 패키지 빌드 (sim/robot/full 모드 선택, CPU shield 관리) |
| `install.sh` | `/` (루트) | ~1163 | 전체 설치 파이프라인 (의존성, RT 권한, 커널, 시스템 설정) |
| `rt_common.sh` | `rtc_scripts/scripts/lib/` | ~462 | 공유 유틸리티 라이브러리 (로깅, CPU 감지, NIC 감지, 패키지 목록, systemd 헬퍼) |
| `build_rt_kernel.sh` | `rtc_scripts/scripts/` | ~1057 | PREEMPT_RT 커널 빌드 및 설치 (7단계) |
| `cpu_shield.sh` | `rtc_scripts/scripts/` | ~339 | 동적 CPU 격리 관리 (cset shield 기반) |
| `setup_irq_affinity.sh` | `rtc_scripts/scripts/` | ~187 | 하드웨어 IRQ를 OS 코어에 고정 (SMT-aware 마스크) |
| `setup_udp_optimization.sh` | `rtc_scripts/scripts/` | ~230 | 네트워크/UDP 실시간 최적화 + systemd 영구 적용 |
| `setup_grub_rt.sh` | `rtc_scripts/scripts/` | ~228 | GRUB RT 커널 파라미터 통합 관리 (single source of truth) |
| `setup_display_rt.sh` | `rtc_scripts/scripts/` | ~235 | X11 anti-tearing + compositor 우선순위 부스트 |
| `setup_cpu_governor.sh` | `rtc_scripts/scripts/` | ~250 | CPU governor performance 모드 + systemd 서비스 |
| `setup_nvidia_rt.sh` | `rtc_scripts/scripts/` | ~744 | NVIDIA GPU + RT 커널 공존 설정 (7단계) |
| `check_rt_setup.sh` | `rtc_scripts/scripts/` | ~1072 | RT 환경 검증 (9개 카테고리) |
| `verify_rt_runtime.sh` | `rtc_scripts/scripts/` | ~1019 | 제어기 구동 중 RT 런타임 검증 (7개 카테고리) |

모든 RT 설정 스크립트는 공유 유틸리티 라이브러리 `lib/rt_common.sh`를 사용합니다.

---

## 루트 스크립트

### build.sh

**목적**: 패키지 빌드 — 유연한 모드 선택과 CPU shield 자동 관리

**인자**:

| 인자 | 설명 |
|------|------|
| `robot` | 실제 로봇 패키지만 빌드 (MuJoCo 제외) |
| `sim` | 시뮬레이션 패키지만 빌드 (MuJoCo 필요) |
| `full` | 전체 빌드 (기본값) |
| `-d`, `--debug` | Debug 빌드 타입 |
| `-r`, `--release` | Release 빌드 타입 (기본값) |
| `-c`, `--clean` | 클린 빌드 (build/install/log 삭제 후 빌드) |
| `-p`, `--packages` | 빌드할 패키지를 콤마로 구분하여 지정 (모드 기본 패키지 대체) |
| `-j N` | 병렬 빌드 워커 수 지정 |
| `-e`, `--export-compile-commands` | `compile_commands.json` 생성 (IntelliSense용) |
| `--mujoco <path>` | MuJoCo 설치 경로 지정 |
| `--no-symlink` | `--symlink-install` 비활성화 |

**주요 기능**:
- `rt_common.sh` 공유 라이브러리 활용 (`make_logger "BUILD" emoji`, `get_base_packages()`, `auto_release_cpu_shield()` 등)
- ROS2 환경 자동 감지 (Humble/Jazzy)
- 빌드 전 CPU shield 자동 해제 (cset shield 활성 시 전체 코어로 빌드)
- 가상 환경 Python 감지 시 시스템 Python 강제 사용 (pinocchio/eigenpy 호환)
- `compile_commands.json` 생성 (Debug 빌드 또는 `--export-compile-commands` 시 자동 활성화)
- ONNX Runtime `/opt/onnxruntime` 자동 감지 및 CMAKE_PREFIX_PATH 전파
- 기존 workspace overlay 자동 소싱 (패키지 의존성 해결)
- 빌드 후 `check_rt_setup.sh --summary` 자동 실행

**사용 예시**:
```bash
./build.sh                              # 전체 빌드 (기본)
./build.sh sim -c -j 4                  # 시뮬레이션 클린 빌드 (4 워커)
./build.sh robot                        # 로봇 전용 빌드 (MuJoCo 제외)
./build.sh sim --mujoco /opt/mujoco-3.2.4  # MuJoCo 경로 지정
./build.sh -d --export-compile-commands # 디버그 빌드 + IntelliSense DB
```

---

### install.sh

**목적**: 전체 설치 파이프라인 — 의존성 설치, RT 권한 설정, 커널 설정, 시스템 구성

**인자**:

| 인자 | 설명 |
|------|------|
| `sim` | 시뮬레이션 전용 설치 |
| `robot` | 실제 로봇 전용 설치 |
| `full` | 전체 설치 (기본값) |
| `--all` | 의존성 + 빌드 + RT 시스템 설정 전부 실행 |
| `--rt` | RT 시스템 설정만 실행 (의존성/빌드 건너뛰기) |
| `-d`, `--debug` | Debug 빌드 타입 |
| `-r`, `--release` | Release 빌드 타입 (기본값) |
| `-c`, `--clean` | 클린 빌드 |
| `-p`, `--packages` | 빌드할 패키지를 콤마로 구분하여 지정 |
| `-j`, `--jobs N` | 병렬 빌드 워커 수 지정 |
| `--skip-deps` | apt 의존성 설치 건너뛰기 |
| `--skip-build` | 빌드 단계 건너뛰기 |
| `--skip-rt` | RT 시스템 설정 건너뛰기 (`--all` 무시) |
| `--skip-debug` | 디버그 도구 설치 건너뛰기 |
| `--ptrace-scope` | ptrace 범위 설정 (VS Code Attach 디버깅용) |
| `--mujoco <path>` | MuJoCo 설치 경로 지정 |

**주요 기능**:
- `rt_common.sh` 공유 라이브러리 활용 (`make_logger "INSTALL" emoji`, `get_base_packages()`, `get_robot_packages()` 등)
- Ubuntu 버전 자동 감지 (22.04/24.04) 및 ROS2 디스트로 매칭 (Humble/Jazzy)
- 물리 코어 vs 논리 코어 감지 (SMT/HT 구분, `rt_common.sh` 활용)
- NVIDIA GPU 자동 감지 및 RT 공존 설정
- Pinocchio (역기구학 라이브러리) 설치
- MuJoCo 자동 다운로드 및 설치 (`/opt/mujoco-*`, 아키텍처 자동 감지)
- RT 권한 설정 (`realtime` 그룹, `limits.conf`)
- GRUB RT 파라미터: `setup_grub_rt.sh`에 위임 (단일 진실 공급원)
- CPU governor: `setup_cpu_governor.sh`에 위임
- `apt-get update` 이중 호출 방지 (5분 캐시)

**사용 예시**:
```bash
./install.sh                    # 전체 설치 (의존성 + 빌드, RT 설정 제외)
./install.sh sim                # 시뮬레이션 전용 (의존성 + 빌드)
./install.sh robot --all        # 실제 로봇 (의존성 + 빌드 + RT 설정)
./install.sh robot --rt         # RT 설정만 실행 (이미 빌드된 경우)
./install.sh full -c -j 4       # 클린 빌드, 4 워커
./install.sh --all --skip-rt    # 의존성 + 빌드, RT 설정 제외
```

---

## RT 설정 스크립트 (`rtc_scripts/scripts/`)

### lib/rt_common.sh --- 공유 유틸리티 라이브러리

모든 RT 스크립트에서 `source`하여 사용하는 공통 함수 라이브러리입니다. 직접 실행할 수 없으며, 중복 `source` 방지 가드(`_RT_COMMON_LOADED`)가 포함되어 있습니다.

**로깅 함수**:

| 함수 | 설명 |
|------|------|
| `setup_colors` | 터미널 색상 변수 초기화 (비-TTY 환경에서는 빈 문자열) |
| `make_logger PREFIX [STYLE]` | 로깅 프리픽스 설정. STYLE: `bracket`(기본, `[PREFIX] ...`) 또는 `emoji`(`▶ ✔ ⚠ ✘` 형식) |
| `info` | 일반 정보 메시지 (녹색 프리픽스) |
| `warn` | 경고 메시지 (노란색 프리픽스) |
| `error` | 오류 메시지 (빨간색 프리픽스, stderr) |
| `fatal` | 오류 메시지 출력 후 즉시 `exit 1` (error+exit 패턴 대체) |
| `success` | 성공 메시지 (녹색 프리픽스) |
| `section` | 섹션 구분 메시지 (파란색 프리픽스) |

**CPU 감지 함수**:

| 함수 | 설명 |
|------|------|
| `get_physical_cores` | 물리 코어 수 반환 (SMT/HT 제외). `lscpu` → `sysfs` → `nproc` 순서로 시도 |
| `compute_cpu_layout` | `thread_config.hpp`와 일치하는 IRQ affinity mask, OS/RT 코어 범위 계산. `TOTAL_CORES`, `LOGICAL_CORES`, `IRQ_AFFINITY_MASK`, `OS_CORES_DESC`, `RT_CORES_START`, `RT_CORES_END` 등 전역 변수 설정 |
| `compute_irq_affinity_mask` | SMT/HT 시블링을 포함한 IRQ affinity 16진수 마스크 계산. OS 물리 코어 + 해당 HT 시블링 모두 포함 |
| `get_os_logical_cpus` | OS 물리 코어에 속하는 논리 CPU 번호 목록 반환 (SMT 시블링 포함) |
| `compute_expected_isolated` | `isolcpus` GRUB 파라미터 기대값 계산 (SMT 시블링 포함 범위 표기) |

**NIC 감지 함수**:

| 함수 | 설명 |
|------|------|
| `detect_physical_nic` | UP 상태의 물리 NIC 자동 감지 (가상 인터페이스 제외). `/sys/class/net/<iface>/device` 심볼릭 링크로 물리 NIC 판별 |

**빌드/워크스페이스 함수**:

| 함수 | 설명 |
|------|------|
| `auto_release_cpu_shield` | cset shield 활성 시 자동 해제 (빌드 전 전체 코어 확보) |
| `check_workspace_structure DIR` | 워크스페이스 디렉토리 구조 검증 (`src/` 하위 여부 확인, `WORKSPACE` 전역변수 설정) |
| `ensure_ros2_sourced` | ROS2 환경 소싱 확인 및 자동 소싱 |
| `get_base_packages` | 기본 패키지 목록 반환 (단일 진실 공급원). Phase 5에서 `rtc_mpc`가 `rtc_urdf_bridge` ↔ `rtc_tsid` 사이에 추가됨. |
| `get_robot_packages` | 로봇 전용 패키지 목록 반환 (단일 진실 공급원) |

**MPC 코어 레이아웃 함수 (Phase 5)** — `rtc_base/threading/thread_config.hpp`와 단일 소스 진실을 공유합니다. `cpu_shield.sh`/`setup_grub_rt.sh`/`setup_irq_affinity.sh`/`check_rt_setup.sh`/`verify_rt_runtime.sh`가 호출:

| 함수 | 설명 | 6코어 / 8코어 / 12코어 / 16코어 |
|------|------|--------------------------------|
| `get_mpc_cores` | MPC main + workers CSV. 첫 항목이 main. | `4` / `4` / `9,10` / `9,10,11` |
| `get_mpc_main_core` | MPC main 코어만. | `4` / `4` / `9` / `9` |
| `get_rt_cores` | RT 스레드 합집합 (`rt_control + sensor_io + udp_recv + MPC`). GRUB `nohz_full` / `rcu_nocbs` 인자에 사용. | `2,3,5,4` / `2,3,5,4` / `7,8,9,10` / `2,3,9,10,11` |
| `get_os_cores` | OS/DDS/IRQ 코어. IRQ affinity 고정 대상. | `0,1` |

**파일/시스템 유틸리티**:

| 함수 | 설명 |
|------|------|
| `write_file_if_changed FILE CONTENT [backup]` | 멱등 파일 쓰기 — 내용 동일 시 skip, 다를 시 백업 후 덮어쓰기 |
| `require_root` | root 권한 확인 (`EUID != 0`이면 에러 종료) |
| `create_oneshot_service NAME DESC EXEC` | systemd oneshot 서비스 생성 (부팅 시 자동 실행, 멱등) |

---

### build_rt_kernel.sh --- PREEMPT_RT 커널 빌드

**목적**: Ubuntu에서 PREEMPT_RT 패치된 커널을 빌드하고 설치합니다. 최초 1회 실행 후 재부팅이 필요합니다.

**7단계 프로세스**:

| 단계 | 내용 |
|------|------|
| 1 | 빌드 의존성 패키지 설치 |
| 2 | 커널 소스 + RT 패치 다운로드 |
| 3 | 소스 추출 및 RT 패치 적용 |
| 4 | 커널 설정 (`make menuconfig` 또는 `--batch`로 자동) |
| 5 | 커널 빌드 (`make bindeb-pkg`) |
| 6 | `.deb` 패키지 설치 |
| 7 | GRUB 등록 및 기본 부팅 커널 설정 |

**인자**:

| 인자 | 설명 |
|------|------|
| `--batch` | 비대화형 모드 (`menuconfig` 건너뜀) |
| `--dry-run` | 다운로드 및 패치까지만 실행 |
| `--status` | 진행 상태만 확인 (실행 안 함) |
| `--verify` | 설치된 RT 커널 검증만 수행 |
| `--force-step N` | N단계부터 강제 재실행 |
| `--clean` | 빌드 소스 정리 후 처음부터 |
| `--build-dir DIR` | 빌드 디렉토리 지정 (기본: `~/rt_kernel_build`) |

**Ubuntu 버전별 커널/패치 자동 선택**:

| Ubuntu | 커널 버전 | RT 패치 |
|--------|-----------|---------|
| 22.04 | 6.6.127 | rt69 |
| 24.04 | 6.8.2 | rt11 |

**주요 기능**:
- NVIDIA DKMS 처리 (`IGNORE_PREEMPT_RT_PRESENCE=1` 환경변수)
- 지능형 단계 재개 (이전 실행에서 완료된 단계 자동 감지 및 건너뛰기)
- 소요 시간: 30분 ~ 2시간 (CPU 성능에 따라 다름)

**사용 예시**:
```bash
sudo ./build_rt_kernel.sh                 # 대화형 빌드 (완료 단계 자동 스킵)
sudo ./build_rt_kernel.sh --batch         # 비대화형 빌드
sudo ./build_rt_kernel.sh --status        # 진행 상태 확인
sudo ./build_rt_kernel.sh --force-step 5  # 5단계부터 재실행
sudo ./build_rt_kernel.sh --clean         # 처음부터 다시 빌드
```

**검증**:
```bash
uname -v | grep PREEMPT_RT   # 재부팅 후 RT 커널 확인
```

---

### cpu_shield.sh --- 동적 CPU 격리

**목적**: `cset shield`를 사용하여 런타임에 CPU 격리를 동적으로 on/off합니다. `isolcpus` GRUB 파라미터 없이도 RT 코어를 보호할 수 있습니다.

**명령**:

| 명령 | 설명 |
|------|------|
| `on [--robot\|--sim]` | CPU 격리 활성화 (기본: `--robot`) |
| `off` | CPU 격리 해제 |
| `status` | 현재 격리 상태 확인 (sudo 불필요) |

**Tier 격리 모델**:

| Tier | 스레드 | 격리 조건 |
|------|--------|-----------|
| Tier 1 (RT-critical) | `rt_control` + `sensor_io` | 항상 격리 |
| Tier 2 (RT-support) | `udp_recv` + `logging` + `aux` + **`mpc_main`/`mpc_worker_*`** (Phase 5) | `--robot` 모드에서만 격리 |
| Tier 3 (Flexible) | `sim` / `monitoring` / `build` | 격리하지 않음 |

**모드별 격리 차이**:
- `--robot` (기본): Tier 1 + Tier 2 격리 (최대 RT 성능). MPC 코어가 user/system cpuset 어느 쪽에 위치하든 cset shield 범위에 자연스럽게 포함됨.
- `--sim`: Tier 1만 격리 (Tier 2 해제, MuJoCo 성능 확보). MPC가 사용되지 않거나 SCHED_OTHER로 동작.

**코어 수별 격리 범위 (robot 모드)** — 2026-04 unified layout: 10/12/14-core tier에서 shield 범위가 RT 쓰레드 전 영역을 포함하도록 확장되었습니다. 이전 10-core 구조(shield 2-6 / RT 7-9)는 8-core 대비 격리 품질 퇴행이 있었으며, 이를 수정한 결과입니다.

| 물리 코어 | 격리 범위 | MPC 코어 위치 |
|-----------|-----------|---------------|
| 4코어 이하 | Core 1-3 | Core 3 (degraded, SCHED_OTHER) |
| 5-7코어 | Core 2-5 | Core 4 (user shield 내) |
| 8-9코어 | Core 2-6 | Core 4 (dedicated) |
| 10-11코어 | Core 2-8 | Core 4 main + Core 5 worker_0 |
| 12-13코어 | Core 2-9 | Core 4 main + Core 5-6 workers |
| 14-15코어 | Core 2-10 | Core 4 main + Core 5-6 workers + Core 10 sim |
| 16코어 이상 | Core 4-8 | Core 9-11 (legacy layout — RT 2-3 below shield, MPC above) |

**자동 호출 시점**:
- 로봇 런치: `ur5e_bringup` launch 파일에서 자동 호출
- 시뮬 런치: `mujoco_sim.launch.py`에서 자동 호출
- 빌드 전: `build.sh` / `install.sh`에서 자동 해제

**`cset shield` vs `isolcpus` 비교**:

| 방식 | 재부팅 | 동적 변경 | 우선 사용 |
|------|--------|-----------|-----------|
| `cset shield` | 불필요 | 가능 | 우선 |
| `isolcpus` (GRUB) | 필요 | 불가 | 차선 |

**사용 예시**:
```bash
sudo ./cpu_shield.sh on             # 로봇 모드 격리 (기본)
sudo ./cpu_shield.sh on --sim       # 시뮬레이션 모드 격리
sudo ./cpu_shield.sh off            # 격리 해제
./cpu_shield.sh status              # 상태 확인
```

---

### setup_irq_affinity.sh --- IRQ 어피니티 설정

**목적**: 모든 하드웨어 IRQ를 OS 코어로 고정하여 RT 코어를 인터럽트로부터 보호합니다.

**인자**:

| 인자 | 설명 |
|------|------|
| `[NIC_NAME]` | NIC 이름 (선택, 미지정 시 자동 감지) |

**주요 기능**:
- 모든 하드웨어 IRQ를 OS 코어에 고정
- NIC 자동 감지 (`detect_physical_nic` 활용)
- SMT/HT-aware affinity 마스크 (`compute_irq_affinity_mask()`로 HT 시블링 포함)
- 멱등성 (이미 올바르게 설정된 IRQ는 건너뜀)
- `require_root` — root 권한 필요
- 에러 시 `fatal()` 즉시 종료

**사용 예시**:
```bash
sudo ./setup_irq_affinity.sh            # NIC 자동 감지
sudo ./setup_irq_affinity.sh eth0       # NIC 이름 명시
sudo ./setup_irq_affinity.sh enp3s0     # PCIe NIC 예시
```

**검증**:
```bash
cat /proc/interrupts | grep eth         # NIC IRQ 번호 확인
cat /proc/irq/<NUM>/smp_affinity        # "3" = Core 0,1
```

---

### setup_udp_optimization.sh --- 네트워크/UDP 최적화

**목적**: 실시간 UDP/ROS2(DDS) 통신을 위한 NIC 및 커널 네트워크 스택 최적화.

**인자**:

| 인자 | 설명 |
|------|------|
| `[NIC_NAME]` | NIC 이름 (선택, 미지정 시 자동 감지) |

**최적화 항목**:

| 카테고리 | 설정 | 값 |
|----------|------|-----|
| Interrupt coalescing | `rx-usecs` | 0 (즉시 인터럽트) |
| Offload 비활성화 | GRO, TSO, GSO | off |
| sysctl 버퍼 | `rmem_max`, `wmem_max` | 2GB (DDS가 setsockopt으로 확보) |
| sysctl 버퍼 | `rmem_default`, `wmem_default` | 212992 (Linux 기본값 유지) |
| sysctl 네트워크 | `netdev_max_backlog` | 5000 |

> CycloneDDS의 `SocketReceiveBufferSize`(8MB)는 이 스크립트가 설정하는 `rmem_max`(2GB) 이하여야 합니다.
> CycloneDDS 설정은 `rtc_controller_manager/config/cyclone_dds.xml` 참조.

**영구 적용**:
- `rtc-udp-optimization` systemd oneshot 서비스로 부팅 시 ethtool 설정 자동 적용
- `/etc/sysctl.d/99-ros2-udp.conf` 설정 파일로 sysctl 값 영구 저장

**멱등성**: 이미 적용된 설정은 건너뛰고, 변경이 필요한 경우에만 적용합니다.

**사용 예시**:
```bash
sudo ./setup_udp_optimization.sh           # NIC 자동 감지
sudo ./setup_udp_optimization.sh enp1s0    # NIC 이름 명시
```

**검증**:
```bash
ethtool -c <NIC>           # coalescing 설정 확인
ethtool -k <NIC>           # offload 설정 확인
sysctl net.core.rmem_max   # 커널 버퍼 크기 확인
```

---

### setup_grub_rt.sh --- GRUB RT 커널 파라미터 통합 관리

**목적**: GRUB RT 커널 파라미터의 단일 진실 공급원(single source of truth). `setup_nvidia_rt.sh`와 `install.sh`에서 분리된 GRUB 파라미터 설정을 통합합니다.

**관리 파라미터**:

> Phase 5 노트: `RT_CORES`는 `compute_expected_isolated`가 반환하는 모든 비-OS 코어 집합입니다. MPC 코어(`get_mpc_cores`)는 비-OS 코어의 부분집합이므로 자동으로 `nohz_full`/`rcu_nocbs`에 포함됩니다 — 별도 머지 단계 불필요.

| 파라미터 | 설명 |
|----------|------|
| `nohz_full=<RT_CORES>` | RT 코어에서 타이머 틱 비활성화 (Phase 5 MPC 코어 자동 포함) |
| `rcu_nocbs=<RT_CORES>` | RT 코어에서 RCU 콜백 오프로드 (Phase 5 MPC 코어 자동 포함) |
| `processor.max_cstate=1` | 깊은 C-state 진입 방지 (wake-up latency 제거) |
| `clocksource=tsc` | TSC 클럭소스 명시 (HPET 대비 50-100x 빠름) |
| `tsc=reliable` | TSC 안정성 마킹 (fallback 방지) |
| `nmi_watchdog=0` | NMI watchdog 비활성화 (100-350µs jitter 제거) |
| `threadirqs` | 하드웨어 IRQ 핸들러 스레드화 |
| `nosoftlockup` | RT 스레드 장시간 점유 시 soft lockup 경고 방지 |

**추가 설정**: `sched_rt_runtime_us=-1` sysctl (RT 스레드 CPU 100% 점유 허용)

**멱등성**: 이미 설정된 파라미터는 건너뛰고, `/etc/default/grub` 수정 전 자동 백업 생성.

**사용 예시**:
```bash
sudo ./setup_grub_rt.sh        # GRUB RT 파라미터 설정
sudo ./setup_grub_rt.sh --help # 도움말 출력
```

**검증**:
```bash
cat /proc/cmdline                   # 현재 부팅 파라미터 확인
sysctl kernel.sched_rt_runtime_us   # -1이면 RT 스레드 무제한 허용
```

---

### setup_display_rt.sh --- 디스플레이/GUI RT 설정

**목적**: RT 커널 환경의 디스플레이 최적화. NVIDIA GPU가 아닌 환경에서도 compositor 우선순위 부스트는 유효합니다.

**주요 기능**:
- X11 anti-tearing 설정 (`ForceFullCompositionPipeline`, `/etc/X11/xorg.conf.d/20-nvidia-antitear.conf`)
- 데스크톱 compositor 우선순위 부스트 (`rt-compositor-boost` systemd 서비스)
- NVIDIA GPU 환경과 일반 환경 모두 지원

**원본**: `setup_nvidia_rt.sh` Stage 7(X11 anti-tearing) + Stage 8(compositor 우선순위 부스트) 분리

**사용 예시**:
```bash
sudo ./setup_display_rt.sh          # 전체 설정 적용
sudo ./setup_display_rt.sh --help   # 도움말
```

**검증**:
```bash
cat /etc/X11/xorg.conf.d/20-nvidia-antitear.conf  # X11 anti-tearing
systemctl status rt-compositor-boost               # compositor 부스트 서비스
```

---

### setup_cpu_governor.sh --- CPU governor performance 모드

**목적**: CPU governor를 `performance`로 설정하여 최대 클럭을 유지합니다. `powersave` governor는 클럭을 동적으로 낮춰 RT 제어 루프 jitter를 증가시킵니다.

**주요 기능**:
1. 모든 CPU의 현재 governor 확인 및 `performance` 전환
2. `cpupower` 도구 설치 (미설치 시)
3. 재부팅 시 자동 적용을 위한 `cpu-governor-performance` systemd oneshot 서비스 생성
4. Intel P-state 드라이버 및 Turbo Boost 상태 보고

**원본**: `setup_nvidia_rt.sh` Stage 10 + `install.sh` `setup_cpu_governor()` 분리

**사용 예시**:
```bash
sudo ./setup_cpu_governor.sh          # governor 설정 + 서비스 등록
sudo ./setup_cpu_governor.sh --help   # 도움말
```

**검증**:
```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor  # 모든 CPU governor 확인
systemctl status cpu-governor-performance                  # systemd 서비스 상태
```

---

### setup_nvidia_rt.sh --- NVIDIA GPU + RT 커널 공존

**목적**: NVIDIA GPU(디스플레이 전용)와 PREEMPT_RT 커널이 안정적으로 공존하도록 전체 시스템을 설정합니다. 500Hz 제어 루프에서 최대 지터 < 200us를 목표로 합니다.

**7단계 프로세스** (v5.17.0에서 11→7단계로 리팩토링, GRUB/디스플레이/governor는 별도 스크립트로 분리):

| 단계 | 내용 |
|------|------|
| 1 | Pre-flight checks (Ubuntu 버전, RT 커널, NVIDIA GPU, CPU 레이아웃) |
| 2 | NVIDIA modprobe 설정 (`/etc/modprobe.d/nvidia-rt.conf`) |
| 3 | NVIDIA IRQ affinity systemd 서비스 |
| 4 | nvidia-smi persistence mode 서비스 |
| 5 | nouveau 블랙리스트 (활성 시) |
| 6 | NVIDIA DKMS 모듈 빌드 (RT 커널용) |
| 7 | 연관 스크립트 호출 (`setup_grub_rt.sh`, `setup_display_rt.sh`, `setup_cpu_governor.sh`) + 검증 요약 |

**주요 modprobe 옵션**:
- `NVreg_EnableGpuFirmware=0` — GPU 펌웨어 비활성화 (RT 호환성)
- `NVreg_EnableMSI=1` — MSI 인터럽트 활성화

**대상 환경**: Ubuntu 22.04 / 24.04, PREEMPT_RT 커널, NVIDIA GPU (CUDA 미사용)

**사용 예시**:
```bash
sudo ./setup_nvidia_rt.sh          # 전체 설정 적용
sudo ./setup_nvidia_rt.sh --help   # 도움말
```

**검증**:
```bash
cyclictest --mlockall --smp --priority=80 --interval=2000 --duration=60
cat /proc/interrupts | grep nvidia   # NVIDIA IRQ 확인
nvidia-smi -q -d PERFORMANCE         # persistence mode 확인
```

---

### check_rt_setup.sh --- RT 환경 검증

**목적**: RT 제어 루프(500Hz) 실행 전 시스템 설정 상태를 9개 카테고리로 점검합니다. sudo 불필요 (read-only).

**인자**:

| 인자 | 설명 |
|------|------|
| `--verbose` | 상세 출력 (기본값) |
| `--summary` | 카테고리당 1줄 요약 (`build.sh`에서 호출용) |
| `--json` | CI용 JSON 출력 |
| `--fix` | 실패 항목별 수정 명령 제안 |
| `--benchmark` | cyclictest RT 지터 실측 (sudo 필요) |

**검증 카테고리 (9개)**:

| # | 카테고리 | 점검 내용 |
|---|----------|-----------|
| 1 | RT Kernel | PREEMPT_RT 커널 활성 여부 |
| 2 | CPU Isolation | isolcpus 설정 확인 (`thread_config.hpp` 매칭) |
| 3 | Scheduler & Memory | clocksource, sched_rt_runtime_us, THP 등 |
| 4 | GRUB Parameters | isolcpus, nohz_full, rcu_nocbs, threadirqs 등 |
| 5 | RT Permissions | ulimit rtprio/memlock, realtime 그룹, limits.conf |
| 6 | IRQ Affinity | 모든 IRQ가 OS 코어에 고정 여부 |
| 7 | Network/UDP | sysctl 버퍼, NIC coalescing/offload |
| 8 | NVIDIA (optional) | persistence mode, IRQ affinity, modprobe 설정 |
| 9 | CPU Frequency | governor=performance 여부 |

> Phase 5 부가 정보: `--verbose` 모드에서 `MPC cores (tier <N>): <CSV>`를 추가로 출력하여 `get_mpc_cores`의 현재 tier 결과를 보여줍니다 (검증 카테고리는 아니며 정보 표시용). 실제 MPC 스레드의 코어 핀닝 검증은 `verify_rt_runtime.sh`가 담당.

**종료 코드**:

| 코드 | 의미 |
|------|------|
| 0 | 모든 항목 통과 |
| 1 | 경고 존재 (실행 가능하지만 최적이 아님) |
| 2 | 실패 존재 (수정 필요) |

**자동 호출**: `build.sh`에서 빌드 후 `--summary` 모드로 자동 실행

**사용 예시**:
```bash
./check_rt_setup.sh              # 상세 검증
./check_rt_setup.sh --summary    # 요약 출력
./check_rt_setup.sh --json       # JSON 형식 출력
./check_rt_setup.sh --fix        # 수정 명령 제안 포함
```

---

### verify_rt_runtime.sh --- 제어기 RT 런타임 검증

**목적**: 제어기 (실행 파일 = ROS 노드 이름 = `ur5e_rt_controller`, robot bringup이 소유)가 실행 중일 때, 각 스레드의 RT 설정이 `thread_config.hpp`에 정의된 대로 올바르게 적용되었는지 실시간 확인합니다. `check_rt_setup.sh`가 정적 시스템 설정을 검증하는 반면, 이 스크립트는 실행 중인 프로세스의 런타임 상태를 검증합니다.

**`check_rt_setup.sh`와의 차이점**:

| 항목 | `check_rt_setup.sh` | `verify_rt_runtime.sh` |
|------|---------------------|------------------------|
| 검증 시점 | 제어기 실행 **전** | 제어기 실행 **중** |
| 대상 | 시스템 설정 (커널, GRUB, sysctl 등) | 프로세스/스레드 상태 |
| sudo 필요 | 불필요 (read-only) | 불필요 (read-only) |
| 제어기 필요 | 불필요 | **필수** (`ur5e_rt_controller` 또는 다른 robot bringup exec 실행 중) |

**인자**:

| 인자 | 설명 |
|------|------|
| `--verbose` | 상세 출력 (기본값) |
| `--summary` | 카테고리당 1줄 요약 |
| `--json` | CI용 JSON 출력 (스레드 상세 포함) |
| `--watch [N]` | N초 간격 반복 모니터링 (기본 3초, Ctrl+C 종료) |

**검증 카테고리 (7개)**:

| # | 카테고리 | 점검 내용 |
|---|----------|-----------|
| 1 | Process Discovery | `(ur5e_)?rt_controller` 패턴(robot bringup exec) 감지, `thread_config.hpp` 스레드 이름 매칭 |
| 2 | Scheduling Policy | 각 스레드의 SCHED_FIFO/OTHER 정책 및 우선순위 검증 |
| 3 | CPU Affinity | 스레드별 코어 할당이 `thread_config.hpp` 기대값과 일치하는지 확인 |
| 4 | Memory Locking | `mlockall` 적용 여부 (VmLck), major/minor page fault 추적 |
| 5 | Context Switches | RT 스레드의 비자발적 컨텍스트 스위치 비율 감시 |
| 6 | CPU Migration | RT 스레드가 기대 코어에서 실행 중인지 (코어 이동 감지) |
| 7 | RT Throttling | `sched_rt_runtime_us` 쓰로틀링 발생 여부, 누적 대기 시간 |

**종료 코드**:

| 코드 | 의미 |
|------|------|
| 0 | 모든 항목 통과 |
| 1 | 경고 존재 |
| 2 | 실패 존재 |
| 3 | 제어기 미실행 |

**사용 예시**:
```bash
./verify_rt_runtime.sh                # 상세 검증
./verify_rt_runtime.sh --summary      # 요약 출력
./verify_rt_runtime.sh --json         # JSON 형식 (CI 파이프라인용)
./verify_rt_runtime.sh --watch        # 3초 간격 실시간 모니터링
./verify_rt_runtime.sh --watch 5      # 5초 간격 모니터링
```

**JSON 출력 예시** (주요 필드):
```json
{
  "controller_pid": 12345,
  "cpu_cores": 6,
  "categories": { "scheduling_policy": {"status": "PASS", "detail": "3/3 correct"} },
  "threads": {
    "rt_control": {"tid": 12346, "expected_cpu": 2, "actual_affinity": "0x4", "actual_policy": "SCHED_FIFO", "actual_priority": 90}
  }
}
```

---

## CPU 코어 레이아웃

`thread_config.hpp`에 정의된 코어 할당 레이아웃입니다. `cpu_shield.sh`와 `setup_irq_affinity.sh`는 이 레이아웃에 맞춰 동작합니다.

### 4-코어 시스템 (fallback)

| 코어 | 역할 |
|------|------|
| 0 | OS / DDS / IRQ |
| 1 | RT Control (500Hz) |
| 2 | Sensor I/O + UDP recv (공유) |
| 3 | Logging + Aux + Monitoring |

### 6-코어 시스템

| 코어 | 역할 |
|------|------|
| 0 | OS / DDS / IRQ |
| 1 | OS / DDS / IRQ |
| 2 | RT Control (500Hz) |
| 3 | Sensor I/O |
| 4 | Logging + Monitoring |
| 5 | UDP recv + Aux + Publish |

### 8-코어 시스템

| 코어 | 역할 |
|------|------|
| 0 | OS / DDS / IRQ |
| 1 | OS / DDS / IRQ |
| 2 | RT Control (500Hz) |
| 3 | Sensor I/O |
| 4 | MPC main (SCHED_FIFO 60) |
| 5 | UDP recv (전용) |
| 6 | Logging |
| 7 | Aux + Publish |

### 10-코어 시스템 (unified layout)

| 코어 | 역할 |
|------|------|
| 0-1 | OS / DDS / NIC IRQ |
| 2 | RT Control (SCHED_FIFO 90) |
| 3 | Sensor I/O (SCHED_FIFO 70) |
| 4 | MPC main (SCHED_FIFO 60) |
| 5 | MPC worker 0 (SCHED_FIFO 55) |
| 6 | UDP recv (SCHED_FIFO 65, dedicated) |
| 7 | Logging |
| 8 | Aux + Publish |
| 9 | MuJoCo sim / spare |

### 12-코어 시스템 (unified layout, MPC + 2 workers)

| 코어 | 역할 |
|------|------|
| 0-1 | OS / DDS / NIC IRQ |
| 2 | RT Control (SCHED_FIFO 90) |
| 3 | Sensor I/O (SCHED_FIFO 70) |
| 4 | MPC main (SCHED_FIFO 60) |
| 5 | MPC worker 0 (SCHED_FIFO 55) |
| 6 | MPC worker 1 (SCHED_FIFO 55) |
| 7 | UDP recv (SCHED_FIFO 65, dedicated) |
| 8 | Logging |
| 9 | Aux + Publish |
| 10 | MuJoCo sim |
| 11 | Spare / user shield |

### 14-코어 시스템 (unified layout, dedicated sim)

| 코어 | 역할 |
|------|------|
| 0-1 | OS / DDS / NIC IRQ |
| 2 | RT Control (SCHED_FIFO 90) |
| 3 | Sensor I/O (SCHED_FIFO 70) |
| 4 | MPC main (SCHED_FIFO 60) |
| 5 | MPC worker 0 (SCHED_FIFO 55) |
| 6 | MPC worker 1 (SCHED_FIFO 55) |
| 7 | UDP recv (SCHED_FIFO 65) |
| 8 | Logging |
| 9 | Aux + Publish |
| 10 | MuJoCo sim (dedicated) |
| 11-13 | Spare / user shield / viewer |

### 16-코어 시스템 (legacy layout, MPC + 2 workers)

| 코어 | 역할 |
|------|------|
| 0-1 | OS / DDS / NIC IRQ |
| 2 | RT Control (SCHED_FIFO 90) |
| 3 | Sensor I/O (SCHED_FIFO 70) |
| 4-8 | cset shield "user" (예약) |
| 9 | MPC main (SCHED_FIFO 60) |
| 10 | MPC worker 0 (SCHED_FIFO 55) |
| 11 | MPC worker 1 (SCHED_FIFO 55) |
| 12 | UDP recv (SCHED_FIFO 65) |
| 13 | Logging |
| 14 | Aux + Publish |
| 15 | MuJoCo sim |

---

## 관련 문서

- [RT_OPTIMIZATION.md](RT_OPTIMIZATION.md) --- 실시간 최적화 가이드 (CPU governor, IRQ affinity, NVIDIA DKMS, 커널 파라미터)
- [VSCODE_DEBUGGING.md](VSCODE_DEBUGGING.md) --- VS Code + GDB 디버깅 가이드
- [README (루트)](../README.md) --- 프로젝트 개요 및 빠른 시작
