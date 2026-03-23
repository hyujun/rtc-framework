# 쉘 스크립트 가이드

RTC (Real-Time Controller) 워크스페이스의 빌드, 설치, 실시간 환경 설정 쉘 스크립트 레퍼런스입니다.

---

## 개요

| 스크립트 | 위치 | 줄 수 | 설명 |
|----------|------|-------|------|
| `build.sh` | `/` (루트) | ~425 | 패키지 빌드 (sim/robot/full 모드 선택, CPU shield 관리) |
| `install.sh` | `/` (루트) | ~1128 | 전체 설치 파이프라인 (의존성, RT 권한, 커널, 시스템 설정) |
| `rt_common.sh` | `rtc_scripts/scripts/lib/` | ~260 | 공유 유틸리티 라이브러리 (로깅, CPU 감지, NIC 감지) |
| `build_rt_kernel.sh` | `rtc_scripts/scripts/` | ~1057 | PREEMPT_RT 커널 빌드 및 설치 (7단계) |
| `cpu_shield.sh` | `rtc_scripts/scripts/` | ~323 | 동적 CPU 격리 관리 (cset shield 기반) |
| `setup_irq_affinity.sh` | `rtc_scripts/scripts/` | ~190 | 하드웨어 IRQ를 OS 코어에 고정 |
| `setup_udp_optimization.sh` | `rtc_scripts/scripts/` | ~188 | 네트워크/UDP 실시간 최적화 |
| `setup_nvidia_rt.sh` | `rtc_scripts/scripts/` | ~1155 | NVIDIA GPU + RT 커널 공존 설정 (11단계) |
| `check_rt_setup.sh` | `rtc_scripts/scripts/` | ~1072 | RT 환경 검증 (9개 카테고리) |
| `verify_rt_runtime.sh` | `rtc_scripts/scripts/` | ~976 | 제어기 구동 중 RT 런타임 검증 (7개 카테고리) |

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
| `-c`, `--clean` | 클린 빌드 (build/install/log 삭제 후 빌드) |
| `-j N` | 병렬 빌드 워커 수 지정 |
| `-e`, `--export-compile-commands` | `compile_commands.json` 생성 (IntelliSense용) |
| `--mujoco <path>` | MuJoCo 설치 경로 지정 |
| `--no-symlink` | `--symlink-install` 비활성화 |

**주요 기능**:
- ROS2 환경 자동 감지 (Humble/Jazzy)
- 빌드 전 CPU shield 자동 해제 (cset shield 활성 시 전체 코어로 빌드)
- 가상 환경 Python 감지 및 경로 설정
- `compile_commands.json` 생성 및 루트 심볼릭 링크
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
| `-d`, `--debug` | Debug 빌드 타입 |
| `-c`, `--clean` | 클린 빌드 |
| `--skip-deps` | apt 의존성 설치 건너뛰기 |
| `--skip-build` | 빌드 단계 건너뛰기 |
| `--skip-rt` | RT 권한/커널 설정 건너뛰기 |
| `--skip-debug` | 디버그 도구 설치 건너뛰기 |
| `--ptrace-scope` | ptrace 범위 설정 (GDB 디버깅용) |
| `--mujoco <path>` | MuJoCo 설치 경로 지정 |

**주요 기능**:
- Ubuntu 버전 자동 감지 (22.04/24.04) 및 ROS2 디스트로 매칭 (Humble/Jazzy)
- 물리 코어 vs 논리 코어 감지 (SMT/HT 구분, `rt_common.sh` 활용)
- NVIDIA GPU 자동 감지 및 RT 공존 설정
- Pinocchio (역기구학 라이브러리) 설치
- MuJoCo 3.2.4 자동 다운로드 및 설치 (`/opt/mujoco-3.2.4`)
- RT 권한 설정 (`realtime` 그룹, `limits.conf`)
- CPU governor `performance` 모드 systemd 서비스 등록
- `apt-get update` 이중 호출 방지 (5분 캐시)

**사용 예시**:
```bash
./install.sh                    # 전체 설치
./install.sh sim --skip-deps    # 시뮬레이션 전용, apt 설치 건너뛰기
./install.sh robot              # 실제 로봇 전용
./install.sh full --skip-rt     # 전체 설치, RT 설정 건너뛰기
```

---

## RT 설정 스크립트 (`rtc_scripts/scripts/`)

### lib/rt_common.sh --- 공유 유틸리티 라이브러리

모든 RT 스크립트에서 `source`하여 사용하는 공통 함수 라이브러리입니다. 직접 실행할 수 없으며, 중복 `source` 방지 가드(`_RT_COMMON_LOADED`)가 포함되어 있습니다.

**로깅 함수**:

| 함수 | 설명 |
|------|------|
| `setup_colors` | 터미널 색상 변수 초기화 (비-TTY 환경에서는 빈 문자열) |
| `make_logger PREFIX` | 로깅 프리픽스 설정 (예: `make_logger "IRQ"` → `[IRQ] ...`) |
| `info` | 일반 정보 메시지 (녹색 프리픽스) |
| `warn` | 경고 메시지 (노란색 프리픽스) |
| `error` | 오류 메시지 (빨간색 프리픽스, stderr) |
| `success` | 성공 메시지 (녹색 프리픽스) |
| `section` | 섹션 구분 메시지 (파란색 프리픽스) |

**CPU 감지 함수**:

| 함수 | 설명 |
|------|------|
| `get_physical_cores` | 물리 코어 수 반환 (SMT/HT 제외). `lscpu` → `sysfs` → `nproc` 순서로 시도 |
| `compute_cpu_layout` | `thread_config.hpp`와 일치하는 IRQ affinity mask, OS/RT 코어 범위 계산. `TOTAL_CORES`, `LOGICAL_CORES`, `IRQ_AFFINITY_MASK`, `OS_CORES_DESC`, `RT_CORES_START`, `RT_CORES_END` 등 전역 변수 설정 |
| `get_os_logical_cpus` | OS 물리 코어에 속하는 논리 CPU 번호 목록 반환 (SMT 시블링 포함) |
| `compute_expected_isolated` | `isolcpus` GRUB 파라미터 기대값 계산 (SMT 시블링 포함 범위 표기) |

**NIC 감지 함수**:

| 함수 | 설명 |
|------|------|
| `detect_physical_nic` | UP 상태의 물리 NIC 자동 감지 (가상 인터페이스 제외). `/sys/class/net/<iface>/device` 심볼릭 링크로 물리 NIC 판별 |

**파일 유틸리티**:

| 함수 | 설명 |
|------|------|
| `write_file_if_changed FILE CONTENT [backup]` | 멱등 파일 쓰기 — 내용 동일 시 skip, 다를 시 백업 후 덮어쓰기 |
| `require_root` | root 권한 확인 (`EUID != 0`이면 에러 종료) |

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
| 5 | 커널 빌드 (`make -j$(nproc)`) |
| 6 | 커널 설치 (`make install` + `make modules_install`) |
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
| Tier 2 (RT-support) | `udp_recv` + `logging` + `aux` | `--robot` 모드에서만 격리 |
| Tier 3 (Flexible) | `sim` / `monitoring` / `build` | 격리하지 않음 |

**모드별 격리 차이**:
- `--robot` (기본): Tier 1 + Tier 2 격리 (최대 RT 성능)
- `--sim`: Tier 1만 격리 (Tier 2 해제, MuJoCo 성능 확보)

**코어 수별 격리 범위 (robot 모드)**:

| 물리 코어 | 격리 범위 |
|-----------|-----------|
| 4코어 이하 | Core 1-3 |
| 5-7코어 | Core 2-5 |
| 8-9코어 | Core 2-6 |
| 10-15코어 | Core 2-6 |
| 16코어 이상 | Core 4-8 |

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
- 모든 하드웨어 IRQ를 OS 코어(Core 0-1)에 고정
- NIC 자동 감지 (`detect_physical_nic` 활용)
- SMT/HT 인식 (물리 코어 기준 레이아웃 적용)
- 멱등성 (이미 올바르게 설정된 IRQ는 건너뜀)
- `require_root` — root 권한 필요

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
| sysctl 버퍼 | `rmem_max`, `wmem_max` | 증가 |
| sysctl 네트워크 | `netdev_max_backlog` | 증가 |

**영구 적용**:
- `systemd` 서비스로 부팅 시 자동 적용
- `/etc/sysctl.d/` 설정 파일로 sysctl 값 영구 저장

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

### setup_nvidia_rt.sh --- NVIDIA GPU + RT 커널 공존

**목적**: NVIDIA GPU(디스플레이 전용)와 PREEMPT_RT 커널이 안정적으로 공존하도록 전체 시스템을 설정합니다. 500Hz 제어 루프에서 최대 지터 < 200us를 목표로 합니다.

**11단계 프로세스**:

| 단계 | 내용 |
|------|------|
| 1 | Pre-flight checks (Ubuntu 버전, RT 커널, NVIDIA GPU, CPU 레이아웃) |
| 2 | NVIDIA modprobe 설정 (`/etc/modprobe.d/nvidia-rt.conf`) |
| 3 | GRUB 커널 파라미터 (isolcpus, nohz_full, rcu_nocbs 등) |
| 4 | NVIDIA IRQ affinity systemd 서비스 |
| 5 | NVIDIA persistence mode 활성화 |
| 6 | nouveau 드라이버 블랙리스트 |
| 7 | X11 anti-tearing 설정 (ForceFullCompositionPipeline) |
| 8 | 데스크톱 compositor 우선순위 설정 |
| 9 | NVIDIA DKMS RT bypass (`IGNORE_PREEMPT_RT_PRESENCE`) |
| 10 | CPU governor `performance` 모드 설정 |
| 11 | 설정 검증 |

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

**목적**: RT 제어 루프(500Hz) 실행 전 시스템 설정 상태를 8개 카테고리로 점검합니다. sudo 불필요 (read-only).

**인자**:

| 인자 | 설명 |
|------|------|
| `--verbose` | 상세 출력 (기본값) |
| `--summary` | 카테고리당 1줄 요약 (`build.sh`에서 호출용) |
| `--json` | CI용 JSON 출력 |
| `--fix` | 실패 항목별 수정 명령 제안 |

**검증 카테고리 (8개)**:

| # | 카테고리 | 점검 내용 |
|---|----------|-----------|
| 1 | RT Kernel | PREEMPT_RT 커널 활성 여부 |
| 2 | CPU Isolation | isolcpus 설정 확인 (`thread_config.hpp` 매칭) |
| 3 | GRUB Parameters | isolcpus, nohz_full, rcu_nocbs, threadirqs 등 |
| 4 | RT Permissions | ulimit rtprio/memlock, realtime 그룹, limits.conf |
| 5 | IRQ Affinity | 모든 IRQ가 OS 코어에 고정 여부 |
| 6 | Network/UDP | sysctl 버퍼, NIC coalescing/offload |
| 7 | NVIDIA (optional) | persistence mode, IRQ affinity, modprobe 설정 |
| 8 | CPU Frequency | governor=performance 여부 |

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

**목적**: 제어기(`rt_controller`)가 실행 중일 때, 각 스레드의 RT 설정이 `thread_config.hpp`에 정의된 대로 올바르게 적용되었는지 실시간 확인합니다. `check_rt_setup.sh`가 정적 시스템 설정을 검증하는 반면, 이 스크립트는 실행 중인 프로세스의 런타임 상태를 검증합니다.

**`check_rt_setup.sh`와의 차이점**:

| 항목 | `check_rt_setup.sh` | `verify_rt_runtime.sh` |
|------|---------------------|------------------------|
| 검증 시점 | 제어기 실행 **전** | 제어기 실행 **중** |
| 대상 | 시스템 설정 (커널, GRUB, sysctl 등) | 프로세스/스레드 상태 |
| sudo 필요 | 불필요 (read-only) | 불필요 (read-only) |
| 제어기 필요 | 불필요 | **필수** (rt_controller 실행 중이어야 함) |

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
| 1 | Process Discovery | `rt_controller` 프로세스 감지, `thread_config.hpp` 스레드 이름 매칭 |
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

### 4-코어 시스템

| 코어 | 역할 |
|------|------|
| 0 | OS + IRQ |
| 1 | OS + ROS2 |
| 2 | RT Control (500Hz) |
| 3 | Sensor I/O + Logger |

### 6-코어 시스템

| 코어 | 역할 |
|------|------|
| 0 | OS + IRQ |
| 1 | OS + ROS2 |
| 2 | RT Control (500Hz) |
| 3 | Sensor I/O |
| 4 | Logger |
| 5 | UDP Recv / Aux |

### 8-코어 시스템

| 코어 | 역할 |
|------|------|
| 0 | OS + IRQ |
| 1 | OS + ROS2 |
| 2 | RT Control (500Hz) |
| 3 | Sensor I/O |
| 4 | Logger |
| 5 | UDP Recv |
| 6 | Aux Executor |
| 7 | Viewer / Monitoring |

---

## 관련 문서

- [RT_OPTIMIZATION.md](RT_OPTIMIZATION.md) --- 실시간 최적화 가이드 (CPU governor, IRQ affinity, NVIDIA DKMS, 커널 파라미터)
- [VSCODE_DEBUGGING.md](VSCODE_DEBUGGING.md) --- VS Code + GDB 디버깅 가이드
- [README (루트)](../README.md) --- 프로젝트 개요 및 빠른 시작
