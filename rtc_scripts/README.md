# rtc_scripts

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **로봇 비의존(robot-agnostic) RT 시스템 구성 및 검증 스크립트 모음**입니다. PREEMPT_RT 커널 빌드, CPU 격리, IRQ 어피니티, 네트워크 최적화, NVIDIA 공존 설정, 환경 검증을 위한 **10개 핵심 스크립트**와 공유 라이브러리를 제공합니다.

**설계 목표:**
- 500 Hz 실시간 제어에서 200 µs 이하 지터 달성
- 2코어 VM부터 16코어+ 시스템까지 자동 적응
- 멱등(idempotent) — 재실행 시 이미 적용된 설정 건너뜀
- 설정과 검증 분리 (setup vs. check/verify)
- 관심사 분리 — 각 스크립트가 단일 책임 수행

---

## 패키지 구조

```
rtc_scripts/
├── CMakeLists.txt
├── package.xml
└── scripts/
    ├── lib/
    │   └── rt_common.sh                  ← 공유 유틸리티 라이브러리
    ├── build_rt_kernel.sh                ← PREEMPT_RT 커널 빌드/설치
    ├── setup_irq_affinity.sh             ← IRQ 어피니티 설정
    ├── setup_udp_optimization.sh         ← NIC/네트워크 스택 최적화
    ├── setup_grub_rt.sh                  ← GRUB RT 커널 파라미터 관리 (NEW)
    ├── setup_display_rt.sh               ← RT 환경 디스플레이 최적화 (NEW)
    ├── setup_cpu_governor.sh             ← CPU governor performance 설정 (NEW)
    ├── setup_nvidia_rt.sh                ← NVIDIA GPU + RT 공존 설정
    ├── cpu_shield.sh                     ← 동적 CPU 격리 (cset/cgroup)
    ├── check_rt_setup.sh                 ← 정적 RT 환경 검증 (9개 카테고리)
    └── verify_rt_runtime.sh              ← 런타임 스레드 검증 (7개 카테고리)
```

---

## 스크립트 요약

### 설정 스크립트 (Setup)

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `build_rt_kernel.sh` | PREEMPT_RT 커널 다운로드/패치/빌드/설치 (7단계) | 필수 |
| `setup_irq_affinity.sh` | 하드웨어 IRQ를 OS 코어에 고정 (SMT-aware) | 필수 |
| `setup_udp_optimization.sh` | NIC 코얼레싱, 오프로드 비활성화, sysctl 최적화 + systemd 서비스 | 필수 |
| `setup_grub_rt.sh` | GRUB RT 커널 파라미터 관리 + sched_rt_runtime_us 설정 | 필수 |
| `setup_display_rt.sh` | X11 안티 티어링 + compositor 우선순위 부스트 | 필수 |
| `setup_cpu_governor.sh` | CPU governor → performance 모드 + systemd 서비스 | 필수 |
| `setup_nvidia_rt.sh` | NVIDIA GPU DKMS, modprobe, IRQ, persistence (7단계) | 필수 |

### 런타임 스크립트 (Runtime)

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `cpu_shield.sh` | 런타임 CPU 격리 (Tier 1/2, robot/sim 모드) | on/off 시 |

### 검증 스크립트 (Verification)

| 스크립트 | 용도 | sudo |
|---------|------|------|
| `check_rt_setup.sh` | 정적 환경 검증 — 커널, CPU, IRQ, 네트워크 등 (9개 카테고리) | 선택 |
| `verify_rt_runtime.sh` | 실행 중 스레드 스케줄링/어피니티/메모리 검증 (7개 카테고리) | 선택 |

---

## 공유 라이브러리 (`rt_common.sh`)

모든 스크립트에서 사용하는 핵심 유틸리티입니다. `build.sh`, `install.sh`에서도 공유됩니다.

### CPU 감지 함수

| 함수 | 설명 |
|------|------|
| `get_physical_cores()` | 물리 코어 수 감지 (SMT/HT 제외) |
| `compute_cpu_layout()` | 코어 수 기반 RT 레이아웃 계산 (IRQ 마스크, OS/RT 코어) |
| `compute_irq_affinity_mask()` | SMT-aware IRQ affinity bitmask 계산 (HT 시블링 포함) |
| `compute_expected_isolated()` | `isolcpus` 기대값 계산 (SMT 시블링 포함 범위 표기) |
| `get_os_logical_cpus()` | OS 물리 코어에 속하는 논리 CPU 번호 목록 |

### NIC/네트워크 함수

| 함수 | 설명 |
|------|------|
| `detect_physical_nic()` | 물리 NIC 자동 탐색 (가상 인터페이스 제외) |

### 로깅 함수

| 함수 | 설명 |
|------|------|
| `setup_colors()` | 터미널 색상 변수 초기화 (비-TTY 환경에서는 빈 문자열) |
| `make_logger PREFIX [STYLE]` | 로깅 프리픽스+스타일 설정. `bracket` (기본): `[PREFIX] msg`, `emoji`: `▶ msg` |
| `info()` / `warn()` / `error()` / `success()` / `section()` | 색상별 로깅 |
| `fatal()` | 에러 메시지 출력 후 `exit 1` (error + exit 통합) |

### 파일/시스템 유틸리티

| 함수 | 설명 |
|------|------|
| `require_root()` | root 권한 확인 (실패 시 `fatal`) |
| `write_file_if_changed()` | 멱등 파일 쓰기 (동일 시 skip, 다를 시 백업+덮어쓰기) |
| `create_oneshot_service()` | systemd oneshot 서비스 생성 헬퍼 |
| `auto_release_cpu_shield()` | 빌드 전 CPU shield 자동 해제 |
| `check_workspace_structure()` | ROS2 워크스페이스 디렉토리 구조 검증 |
| `ensure_ros2_sourced()` | ROS2 환경 자동 탐색 및 소싱 |

### 패키지 리스트 함수

| 함수 | 설명 |
|------|------|
| `get_base_packages()` | 기본 RTC 패키지 리스트 (build.sh/install.sh 공유) |
| `get_robot_packages()` | 로봇 전용 패키지 리스트 |

---

## 스크립트 상세

### build_rt_kernel.sh

PREEMPT_RT 커널을 소스에서 빌드하고 설치합니다. 7단계 파이프라인으로 체크포인트 기반 재개를 지원합니다.

```bash
sudo ./build_rt_kernel.sh              # 대화형 빌드
sudo ./build_rt_kernel.sh --batch      # 비대화형 (menuconfig 건너뜀)
sudo ./build_rt_kernel.sh --status     # 진행 상태 확인
sudo ./build_rt_kernel.sh --verify     # 단계별 진단
sudo ./build_rt_kernel.sh --force-step 5  # 5단계부터 강제 재시작
```

**지원 커널:**

| Ubuntu | 커널 | RT 패치 |
|--------|------|---------|
| 24.04 | 6.8.2 | 6.8.2-rt11 |
| 22.04 | 6.6.127 | 6.6.127-rt69 |

---

### setup_grub_rt.sh (NEW)

GRUB RT 커널 파라미터의 **단일 진실 원천(single source of truth)**입니다. `install.sh`와 `setup_nvidia_rt.sh`에서 호출됩니다.

```bash
sudo ./setup_grub_rt.sh
```

**관리하는 파라미터:**

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

추가: `sched_rt_runtime_us=-1` 즉시 + 영구 적용

---

### setup_irq_affinity.sh

하드웨어 IRQ를 OS 코어에 고정하여 RT 코어를 인터럽트 지터로부터 보호합니다.

```bash
sudo ./setup_irq_affinity.sh              # NIC 자동 감지
sudo ./setup_irq_affinity.sh enp3s0       # NIC 지정
```

- **SMT-aware**: HT 시블링까지 포함한 IRQ affinity mask 계산
- NIC 및 모든 하드웨어 IRQ를 OS 코어에 고정
- 타이머 IRQ 0은 제외
- 적용 후 RT 코어에 잔여 IRQ가 없는지 검증

---

### setup_udp_optimization.sh

NIC 하드웨어 및 커널 네트워크 스택을 실시간 UDP/DDS 통신에 최적화합니다.

```bash
sudo ./setup_udp_optimization.sh          # NIC 자동 감지
```

| 단계 | 설정 | 효과 |
|------|------|------|
| 1 | 인터럽트 코얼레싱 비활성화 | 즉시 인터럽트 전달 |
| 2 | GRO/LRO/TSO/GSO 비활성화 | 지연 시간 감소 |
| 3 | RX/TX 링 버퍼 1024 | DDS 패킷에 적합 |
| 4 | sysctl rmem_max/wmem_max=2GB, default=212992, backlog=5000 | DDS 소켓 버퍼 상한 확대 |
| 5 | systemd 서비스 생성 | 부팅 시 ethtool 설정 자동 재적용 |

sysctl 설정은 `/etc/sysctl.d/99-ros2-udp.conf`로 영구 적용됩니다.
ethtool 설정은 systemd oneshot 서비스로 부팅 시 자동 재적용됩니다.

> CycloneDDS `SocketReceiveBufferSize`(8MB)는 `rmem_max`(2GB) 이하여야 합니다. CycloneDDS 설정: `rtc_controller_manager/config/cyclone_dds.xml`

---

### setup_display_rt.sh (NEW)

RT 환경에서 디스플레이 안정성을 확보합니다.

```bash
sudo ./setup_display_rt.sh
```

- X11 안티 티어링 (`ForceFullCompositionPipeline`)
- Compositor 우선순위 부스트 (nice -10)
- Wayland/X11/headless 자동 감지

---

### setup_cpu_governor.sh (NEW)

CPU governor를 performance 모드로 설정합니다.

```bash
sudo ./setup_cpu_governor.sh
```

- 모든 CPU governor를 performance로 즉시 변경
- systemd 서비스로 부팅 시 자동 적용
- cpupower 도구 자동 설치 시도
- Intel P-state / Turbo Boost 상태 리포팅

---

### setup_nvidia_rt.sh

NVIDIA GPU와 PREEMPT_RT 커널의 안정적 공존을 설정합니다 (7단계).

```bash
sudo ./setup_nvidia_rt.sh
```

**7단계 프로세스:**

| 단계 | 내용 |
|------|------|
| 1/7 | Pre-flight checks (Ubuntu, RT 커널, NVIDIA GPU, CPU 레이아웃) |
| 2/7 | NVIDIA modprobe 설정 (`NVreg_EnableGpuFirmware=0`) |
| 3/7 | NVIDIA IRQ affinity systemd 서비스 |
| 4/7 | nvidia-smi persistence mode 활성화 |
| 5/7 | nouveau 블랙리스트 |
| 6/7 | NVIDIA DKMS 모듈 빌드 (RT 커널용) |
| 7/7 | 연관 스크립트 호출 + 검증 요약 |

> Stage 7에서 `setup_grub_rt.sh`, `setup_display_rt.sh`, `setup_cpu_governor.sh`를 자동 호출합니다.

---

### cpu_shield.sh

런타임 CPU 격리를 동적으로 관리합니다. `isolcpus` GRUB 파라미터 없이 재부팅 불필요합니다.

```bash
sudo cpu_shield.sh on --robot    # 전체 격리 (Tier 1+2)
sudo cpu_shield.sh on --sim      # 경량 격리 (Tier 1만)
sudo cpu_shield.sh off           # 격리 해제
cpu_shield.sh status             # 상태 확인 (sudo 불필요)
```

**Tier 모델:**

| Tier | 스레드 | 격리 |
|------|--------|------|
| Tier 1 (RT-critical) | rt_control + sensor_io | 항상 |
| Tier 2 (RT-support) | udp_recv + logging + aux | `--robot` 모드만 |
| Tier 3 (Flexible) | sim, monitoring, build | 격리 안 함 |

> Launch 파일 (`robot.launch.py`, `sim.launch.py`)에서 자동 호출됩니다.

---

### check_rt_setup.sh

실시간 환경의 정적 검증을 수행합니다 (컨트롤러 실행 전).

```bash
./check_rt_setup.sh              # 상세 출력
./check_rt_setup.sh --summary    # 카테고리별 한 줄 요약
./check_rt_setup.sh --json       # CI/CD용 JSON 출력
./check_rt_setup.sh --fix        # 수정 명령 표시
./check_rt_setup.sh --benchmark  # cyclictest 지터 측정 (sudo)
```

**9개 검증 카테고리:**

| # | 카테고리 | 검증 내용 |
|---|---------|----------|
| 1 | RT 커널 | `PREEMPT_RT` 활성 (`uname -v`) |
| 2 | CPU 격리 | `isolcpus` 또는 cset shield 상태 |
| 3 | 스케줄러 & 메모리 | clocksource=tsc, sched_rt_runtime_us=-1, THP=never |
| 4 | GRUB 파라미터 | nohz_full, rcu_nocbs, processor.max_cstate, nmi_watchdog |
| 5 | RT 권한 | ulimit -r ≥90, ulimit -l=unlimited, realtime 그룹 |
| 6 | IRQ 어피니티 | 모든 IRQ가 OS 코어에 고정 |
| 7 | 네트워크/UDP | rmem_max=2GB, 코얼레싱=0 |
| 8 | NVIDIA | persistence 모드, IRQ 어피니티, nouveau 차단 |
| 9 | CPU 주파수 | governor=performance |

**종료 코드:** 0=PASS, 1=경고, 2=실패

---

### verify_rt_runtime.sh

RT 컨트롤러 실행 중 스레드 상태를 검증합니다.

```bash
./verify_rt_runtime.sh              # 상세 출력
./verify_rt_runtime.sh --summary    # 카테고리별 요약
./verify_rt_runtime.sh --json       # JSON 출력
./verify_rt_runtime.sh --watch 5    # 5초 간격 연속 모니터링
```

**7개 검증 카테고리:**

| # | 카테고리 | 검증 내용 |
|---|---------|----------|
| 1 | 프로세스 탐색 | rt_controller PID, 스레드 이름 매칭 |
| 2 | 스케줄링 정책 | SCHED_FIFO/OTHER + 우선순위 (`chrt -p`) |
| 3 | CPU 어피니티 | 예상 코어에 고정 (`taskset -p`) |
| 4 | 메모리 잠금 | `mlockall()` 적용 (VmLck > 0) |
| 5 | 컨텍스트 스위치 | 비자발적 전환 빈도 |
| 6 | CPU 마이그레이션 | RT 스레드 코어 이동 없음 |
| 7 | RT 스로틀링 | `sched_rt_runtime_us=-1`, 스로틀링 누적 없음 |

**종료 코드:** 0=PASS, 1=경고, 2=실패, 3=컨트롤러 미실행

---

## 프레임워크 연동 흐름

```
[1회 설정]
  build_rt_kernel.sh → 재부팅
  setup_grub_rt.sh              ← GRUB RT 파라미터
  setup_irq_affinity.sh
  setup_udp_optimization.sh
  setup_cpu_governor.sh         ← CPU governor
  setup_nvidia_rt.sh (선택)     ← NVIDIA GPU 설정 + 위 3개 자동 호출
  setup_display_rt.sh (선택)    ← 디스플레이 최적화

[빌드 전 검증]
  check_rt_setup.sh --summary

[로봇 실행]
  robot.launch.py → cpu_shield.sh on --robot → rt_controller 시작

[시뮬레이션 실행]
  sim.launch.py → cpu_shield.sh on --sim → mujoco + rt_controller 시작

[런타임 모니터링]
  verify_rt_runtime.sh --watch 3

[종료]
  cpu_shield.sh off
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |

**시스템 요구사항:** bash 4.0+, `ethtool`, `lscpu`, `sysctl`, `cset` (cpuset), `cyclictest` (rt-tests)

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_scripts
source install/setup.bash
```

스크립트만 포함된 패키지이므로 바이너리는 생성되지 않습니다.

---

## 의존성 그래프 내 위치

**독립 인프라 패키지** — RT 환경을 구성하고 검증합니다.

```
rtc_scripts  ← 독립 (ament_cmake만 의존)
    ↑
    ├── build.sh / install.sh  (rt_common.sh 공유 함수 사용)
    ├── ur5e_bringup           (launch 파일에서 cpu_shield.sh 호출)
    └── 빌드 시스템            (build.sh에서 check_rt_setup.sh 호출)
```

---

## 최적화 내역 (v5.17.0)

| 영역 | 변경 내용 |
|------|----------|
| **관심사 분리** | `setup_nvidia_rt.sh`에서 GRUB/디스플레이/governor를 별도 스크립트로 분리 (11→7단계) |
| **코드 중복 제거** | `auto_release_cpu_shield()`, `check_workspace_structure()`, `ensure_ros2_sourced()` → `rt_common.sh`로 통합 |
| **에러 처리 통일** | `fatal()` 함수 추가, 모든 스크립트에서 일관된 exit 동작 |
| **SMT-aware IRQ mask** | `compute_irq_affinity_mask()` — HT 시블링까지 포함한 정확한 affinity mask |
| **로깅 스타일 통합** | `make_logger PREFIX [bracket\|emoji]` — 스타일 선택 가능 |
| **패키지 리스트 통합** | `get_base_packages()`, `get_robot_packages()` — build.sh/install.sh 공유 |
| **ethtool 영구 적용** | `setup_udp_optimization.sh` — systemd 서비스로 부팅 시 자동 재적용 |
| **systemd 헬퍼** | `create_oneshot_service()` — 서비스 생성 보일러플레이트 제거 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
