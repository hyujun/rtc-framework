# 실시간 최적화 가이드

RT 제어 루프 (기본 500 Hz) 가 지터 없이 돌도록 **호스트 PC 를 설정하고 확인하는 방법**이다. 스레드·코어 배치의 설계 근거는 [architecture.md](../agent_docs/architecture.md), 이 문서가 권하는 설정의 근거·사건은 [rt-setup-rationale.md](reference/rt-setup-rationale.md) 에 있다.

## 목차

- [0. 전체 흐름](#0-전체-흐름)
- [1. install.sh 로 한 번에](#1-installsh-로-한-번에)
- [2. 단계별 수동 설정](#2-단계별-수동-설정)
- [3. 코어 배치](#3-코어-배치)
- [4. 확인](#4-확인)
- [5. 문제 해결](#5-문제-해결)
- [6. Hybrid CPU (NUC)](#6-hybrid-cpu-nuc)
- [7. 고급·참고](#7-고급참고)

## 0. 전체 흐름

RT 루프는 `clock_nanosleep` 절대시간으로 직접 돈다. 이 루프가 OS 의 방해를 받지 않게 하려면 **커널 · 부팅 옵션 · 런타임 격리 · 인터럽트 · 전원 정책** 을 맞춰야 한다.

```text
권한 (realtime 그룹)        RT 스레드를 SCHED_FIFO / mlockall 로 띄울 자격
  -> RT 커널                PREEMPT_RT 또는 lowlatency (선택, 최대 성능)
  -> GRUB 옵션 (재부팅)     nohz_full / rcu_nocbs 로 RT 코어의 tick 제거
  -> cpu_shield (런치마다)  cset 으로 RT 코어에서 다른 프로세스 쫓아냄
  -> IRQ affinity           하드웨어 인터럽트를 OS 코어로 모음
  -> governor = performance 주파수 변동 제거
  -> (NVIDIA 만) DKMS       RT 커널에서 드라이버 빌드
  -> check_rt_setup.sh -> verify_rt_runtime.sh -> 지터 측정
```

처음이면 [1절](#1-installsh-로-한-번에) 한 줄로 끝내고 [4절](#4-확인)으로 검증한다. 2절은 한 단계만 다시 하거나 원인을 파고들 때 쓴다.

## 1. install.sh 로 한 번에

```bash
cd ~/ros2_ws/rtc_ws/src/rtc-framework
./install.sh robot --rt        # RT 설정만 (의존성·빌드 생략)
./install.sh --all --skip-rt   # 반대로 RT 설정만 빼고 설치
```

`--rt` 는 realtime 그룹·limits·sudoers, cset 도구, GRUB 옵션, IRQ affinity, UDP 버퍼, NVIDIA (GPU 가 있을 때), CPU governor 를 순서대로 적용한다. 단계가 실패해도 설치는 계속되고 마지막에 실패 목록을 보여준다. sim 모드는 RT 설정을 하지 않는다.

**`--rt` 가 하지 않는 것**: RT 커널 설치 ([2.2](#22-rt-커널))와 재부팅. 그룹 변경은 재로그인, GRUB 변경은 재부팅 후에 적용된다.

## 2. 단계별 수동 설정

`sudo` 로 도는 스크립트는 모두 멱등이라 다시 실행해도 된다.

### 2.1 RT 권한

`install.sh` 가 `realtime` 그룹을 만들고 사용자를 넣고 `/etc/security/limits.conf` 에 `rtprio 99` · `memlock unlimited` 를 쓴다. 적용은 재로그인 후:

```bash
groups | grep realtime
ulimit -r   # 99
ulimit -l   # unlimited
```

런치가 cset 을 비밀번호 없이 부르도록 `/etc/sudoers.d/` 항목도 `install.sh --rt` 가 만든다 (없으면 컨트롤러가 격리 영역으로 옮겨지지 못해 `inactive` 에 머문다).

### 2.2 RT 커널

최대 성능을 원할 때만 필요하다. 둘 중 하나를 고른다.

**A. lowlatency 커널 (쉬움)**

```bash
sudo apt install linux-lowlatency-hwe-24.04
sudo reboot
uname -v    # ...~24.04... PREEMPT_DYNAMIC ... lowlatency
```

**B. PREEMPT_RT 커널 (빌드, 최대 성능)**

PREEMPT_RT 는 Linux 6.12 부터 mainline 이라 외부 패치 없이 빌드된다. 스크립트가 vanilla 6.12.x 를 받아 서명을 검증하고 CPU 에 맞는 config 로 빌드·설치한다. 30분~2시간 걸린다.

```bash
sudo ./repo_scripts/scripts/build_rt_kernel.sh              # 대화형 (menuconfig 포함)
sudo ./repo_scripts/scripts/build_rt_kernel.sh --batch      # 비대화형
sudo ./repo_scripts/scripts/build_rt_kernel.sh --batch --with-nvidia   # NVIDIA 모듈까지
sudo ./repo_scripts/scripts/build_rt_kernel.sh --dry-run    # 다운로드·설정까지만
sudo ./repo_scripts/scripts/build_rt_kernel.sh --status     # 진행 상태 확인
sudo reboot
uname -v | grep PREEMPT_RT
```

- `--profile auto|desktop|nuc` : 기본 `auto` 가 CPU 를 감지한다 (hybrid Intel -> `nuc`, AMD · 일반 Intel -> 데스크탑 config). `nuc` 만 hybrid 스케줄링 옵션 (SCHED_MC_PRIO · SCHED_CLUSTER · INTEL_HFI_THERMAL) 을 켠다.
- `--kernel-version V` : 기본값은 스크립트에 pin 되어 있다. 6.12 미만은 거부한다.
- 이미 끝난 단계는 건너뛰고, `--force-step N` 으로 N 단계부터 다시 한다.
- Secure Boot 가 켜져 있으면 미서명 커널이 부팅되지 않는다 (스크립트가 경고만 한다).

#### 구버전 커널은 외부 RT 패치가 필요

6.12 미만 커널에는 PREEMPT_RT 가 들어 있지 않다. `build_rt_kernel.sh` 는 이 경우를 지원하지 않고 `--kernel-version` 을 거부한다. 선택지는 둘이다.

1. **6.12 이상으로 올린다** (권장). 위 B 절차를 그대로 쓴다.
2. 구버전을 꼭 써야 하면 해당 버전의 외부 PREEMPT_RT 패치를 직접 적용해 빌드한다. 이 경우 스크립트·NVIDIA 자동 처리·검증은 보장되지 않는다.

### 2.3 GRUB 옵션

```bash
sudo ./repo_scripts/scripts/setup_grub_rt.sh
sudo reboot
cat /proc/cmdline    # nohz_full=... rcu_nocbs=... 가 보여야 한다
```

스크립트가 `/etc/default/grub` 을 백업한 뒤 `nohz_full` · `rcu_nocbs` (RT 코어, SMT 면 형제 포함) 와 `processor.max_cstate=1` · `clocksource=tsc` · `nmi_watchdog=0` · `threadirqs` 등을 넣고 `update-grub` 을 돈다. `sched_rt_runtime_us=-1` sysctl 도 같이 영구 등록한다. 값은 코어 수에서 계산되므로 직접 편집하지 않는다.

`isolcpus` 는 쓰지 않는다. 빌드 중에도 코어가 묶이고 재부팅 없이 풀 수 없어서, 대신 2.4 의 `cpu_shield` 로 런타임에 격리한다.

### 2.4 cpu_shield (런타임 격리)

```bash
sudo ./repo_scripts/scripts/cpu_shield.sh on --robot   # 격리 켜기 (--sim 도 범위 동일)
./repo_scripts/scripts/cpu_shield.sh status            # 상태 + 이 PC 의 코어 배치 (sudo 불필요)
sudo ./repo_scripts/scripts/cpu_shield.sh off          # 해제
```

보통 직접 부를 일이 없다. 런치 파일이 켜고, 컨트롤러를 격리 영역 안으로 옮긴 뒤에야 activate 한다. 옮기지 못하면 컨트롤러는 `inactive` 에 머문다 (조용히 RT 없이 도는 것을 막기 위함). 빌드 전에는 `build.sh` / `install.sh` 가 자동 해제한다.

### 2.5 IRQ affinity

```bash
sudo ./repo_scripts/scripts/setup_irq_affinity.sh          # NIC 자동 감지
sudo ./repo_scripts/scripts/setup_irq_affinity.sh enp3s0   # NIC 지정
```

하드웨어 IRQ 를 OS 코어로 모아 RT 코어를 인터럽트에서 보호한다. 어느 코어가 OS 코어인지는 스크립트가 계산해 마지막에 배치를 출력한다. 마스크를 직접 `/proc/irq/*/smp_affinity` 에 쓰지 않는다.

### 2.6 CPU governor

```bash
sudo ./repo_scripts/scripts/setup_cpu_governor.sh
```

**모든** 코어를 `performance` 로 바꾸고 재부팅 후에도 유지되게 systemd 서비스 (`cpu-governor-performance.service`) 를 등록한다. OS 코어가 `powersave` 면 화면 프레임이 떨어지므로 RT 코어만 바꾸면 안 된다. NVIDIA 시스템은 2.7 이 이 스크립트를 이어서 부른다.

### 2.7 NVIDIA (GPU 가 있을 때)

NVIDIA 드라이버는 PREEMPT_RT 커널에서 빌드를 거부한다. 스크립트가 이를 우회하고 modprobe · IRQ · persistence mode · nouveau 블랙리스트까지 한 번에 처리한다.

```bash
sudo ./repo_scripts/scripts/setup_nvidia_rt.sh
nvidia-smi
dkms status nvidia    # 현재 커널에 installed
```

스크립트는 사전 점검 · modprobe 설정 · IRQ affinity 서비스 · persistence mode · nouveau 블랙리스트 · DKMS 빌드 · 요약 순서로 진행하고, 끝에서 GRUB · 디스플레이 · CPU governor 스크립트를 이어 부른다. DKMS 빌드가 막히는 세 층과 우회는 [setup_nvidia_rt.sh](../repo_scripts/scripts/setup_nvidia_rt.sh) 의 DKMS 단계 주석이 갖는다.

## 3. 코어 배치

- 코어 번호는 **slot** 이다. 커널 logical CPU 번호가 아니며, SMT 형제와 hybrid 의 E-core 를 피하도록 실제 CPU 로 변환된다.
- 물리 코어는 sysfs 형제 목록 (`core_cpus_list`, 없으면 `thread_siblings_list`, 둘 다 없을 때만 `core_id`) 으로 묶고, 가장 낮은 logical CPU 순으로 센 N 번째가 slot N 이다 (hybrid 는 P-core 다음 E-core). `core_id` 번호는 slot 이 아니다 — 모든 CPU 에 같은 값을 내는 SoC, 번호에 빈칸이 있거나 cpu0 의 코어를 중간에 두는 펌웨어가 있다.
- OS · DDS · IRQ 는 slot 0 하나를 쓴다. RT 스레드 (`rt_control` · `rt_callback` · `mpc_main`) 는 그 위의 연속 코어에 고정되고, 드라이버 프로세스와 non-RT 스레드는 그 밖에 놓인다.
- 코어 수가 늘어도 RT 클러스터는 같고, 달라지는 것은 드라이버 쪽 배치뿐이다. 4 · 6 코어는 일부 스레드가 코어를 공유하는 degraded 구성이다.
- 값의 SSoT 는 [thread_layout.yaml](../repo_scripts/config/thread_layout.yaml), tier 별 표는 [rtc_base/README.md](../rtc_base/README.md). **이 PC 에 실제로 적용되는 배치**는 `cpu_shield.sh status` 가 출력한다. 스크립트에서 한 역할의 slot 만 보려면:

```bash
source repo_scripts/scripts/lib/rt_common.sh
get_role_slot rt_control "$(get_physical_cores)"
```

## 4. 확인

순서대로 본다. 앞 단계가 FAIL 이면 뒤는 의미가 없다.

### 4.1 정적 설정: check_rt_setup.sh

```bash
./repo_scripts/scripts/check_rt_setup.sh            # 상세
./repo_scripts/scripts/check_rt_setup.sh --fix      # 실패 항목의 수정 명령 제안
./repo_scripts/scripts/check_rt_setup.sh --summary  # 카테고리당 1줄
./repo_scripts/scripts/check_rt_setup.sh --benchmark   # cyclictest 지터 실측 (sudo)
```

RT 커널 · CPU isolation · scheduler/메모리 · GRUB · 권한 · IRQ · 네트워크 · NVIDIA · governor 를 점검한다. 컨트롤러가 꺼져 있어도 된다.

### 4.2 실행 중 상태: verify_rt_runtime.sh

컨트롤러를 띄운 뒤 각 스레드의 정책 · priority · affinity · memlock · migration 을 기대값과 대조한다.

```bash
./repo_scripts/scripts/verify_rt_runtime.sh              # 상세
./repo_scripts/scripts/verify_rt_runtime.sh --summary    # 1줄 요약
./repo_scripts/scripts/verify_rt_runtime.sh --json       # CI 용
./repo_scripts/scripts/verify_rt_runtime.sh --watch 5    # 5초 간격
```

종료 코드: 0 모두 통과 / 1 경고만 / 2 실패 / 3 컨트롤러 미실행. 필수 스레드를 못 찾으면 FAIL 이다. sim 에서는 driver 프로세스 검사를 건너뛴다.

특정 스레드를 직접 보려면:

```bash
PID=$(pidof -s integrated_rt_controller)
ps -T -p $PID -o tid,comm,rtprio,psr,cls,pri   # CLS: FF=SCHED_FIFO, TS=CFS
taskset -cp <TID>
```

`rt_control` > `rt_callback` > `mpc_main` 순으로 priority 가 높고 모두 `FF` 여야 한다 (4 코어 fallback 만 `mpc_main` 이 CFS). 기대 코어는 3절의 방법으로 조회해 비교한다.

### 4.3 지터 측정

**cm_timing_log.csv (권장)** : RT 루프가 매 tick 기록하는 값이다.

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py    # 실기는 robot_ur5e_p1a.launch.py
# 30초 이상 (500 Hz 면 15k tick) 돌린 뒤 종료
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv --stats   # 통계만
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv           # plot + 통계
```

파일은 `logging_data/<session_id>/timing/` 에 생긴다. 같은 분에 재기동하면 두 런이 한 CSV 에 이어 붙으므로 도구가 마지막 런만 고른다 (`--run-id` 로 변경). 목표는 `t_total_us` 의 p99 가 주기 (500 Hz 면 2 ms) 의 절반 이하, overrun · skip · drop 카운터가 모두 0, 첫 tick 을 뺀 max 가 p99 의 10 배 이내이다.

**cyclictest (OS 지터만 보는 보조 측정)** :

```bash
sudo apt install rt-tests
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo --interval=2000 --loops=100000 --histogram=200
```

PREEMPT_RT + cset shield 에서 Max 가 50 µs 미만이면 정상이다. `check_rt_setup.sh --benchmark` 가 같은 측정을 감싼다.

**rtla (커널 노이즈 분해, Ubuntu 24.04 기본)** : `rtla osnoise top -P F:1 -c <격리 코어>` 는 OS noise 를, `rtla hwnoise hist` 는 IRQ 를 끈 상태의 hardware noise 를 본다. cyclictest 와 함께 배포 전 1회 · 환경을 바꾼 뒤 다시 재고, 결과는 timing CSV 와 같은 세션 디렉토리에 둔다.

**6 코어 tier** 는 arm 과 hand 가 slot 4 를 공유하고 hand 의 UDP 수신 루프 (FIFO 65) 가 arm 루프 (FIFO 50) 보다 높다. 그 tier 의 지터는 반드시 **arm + hand 동시 구동** 에서 잰다.

**그 밖에**: 제어 주파수는 `ros2 topic hz <명령 토픽>`, 마이그레이션은 `sudo perf stat -e context-switches,cpu-migrations -p $PID sleep 10` (cpu-migrations 가 0 이어야 한다). 스레드 × 코어 타임라인은 [tracing.md](tracing.md).

## 5. 문제 해결

먼저 `check_rt_setup.sh --fix` 를 돌리면 대부분 원인이 나온다.

| 증상 | 원인 | 해결 |
|---|---|---|
| `Thread config failed for 'rt_control' (need realtime permissions)` | realtime 그룹·limits 미적용 | `groups \| grep realtime`, `ulimit -r` 이 99 인지 확인. 안 맞으면 `./install.sh robot --rt` 후 **재로그인** |
| 컨트롤러가 `inactive` 에서 안 넘어감 | cset 격리로 옮기기 실패 (sudoers 없음 · cset 미설치) | `./install.sh robot --rt` 로 sudoers · cset 설치, `cpu_shield.sh status` 확인 |
| RT 스레드의 PSR 이 RT 코어 밖으로 움직임 | 격리 미적용 | `/proc/cmdline` 에 `nohz_full` 이 없으면 `setup_grub_rt.sh` + 재부팅. `cpu_shield.sh status` 로 shield 활성 확인 |
| 지터가 큼 | 일반 커널 | `uname -v \| grep -iE 'lowlatency\|PREEMPT_RT'` 가 비면 2.2 |
| 지터가 큼 | RT 코어로 IRQ 유입 | `/proc/interrupts` 에서 RT 코어 열을 본다. 많으면 `setup_irq_affinity.sh` |
| 지터가 큼 | DDS 스레드가 RT 코어에 있음 | 런치가 DDS 스레드를 `taskset` 으로 고정한다. 스레드별 `chrt -p` · `taskset -cp` 로 확인 |
| 500 Hz 미달 | governor 가 `powersave` | `setup_cpu_governor.sh` (모든 코어가 `performance` 여야 함) |
| `nvidia-smi` 불가 / DKMS `apport` 오류 | RT 커널에서 NVIDIA 빌드 거부 | `setup_nvidia_rt.sh` 재실행 |

DDS 는 CycloneDDS 를 권장한다: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, 튜닝은 [cyclone_dds.xml](../rtc_controller_manager/config/cyclone_dds.xml) (SPDP-only 멀티캐스트, discovery · NACK · heartbeat 주기). CycloneDDS 자체에는 CPU affinity 설정이 없어 고정은 런치의 `taskset` 이 맡는다.

## 6. Hybrid CPU (NUC)

P-core / E-core 가 섞인 Intel CPU 에서는 코어 slot 이 **P-core 의 primary logical CPU** 로 변환된다 (P -> E -> LP-E 순). 별도 설정 없이 RT 스레드가 P-core 에 고정되고 SMT 형제는 피한다. 감지는 [rt_common.sh](../repo_scripts/scripts/lib/rt_common.sh) 의 `detect_hybrid_capability` 와 [cpu_topology.hpp](../rtc_base/include/rtc_base/threading/cpu_topology.hpp) 가 맡는다.

| 세대 | 대표 모델 | 동작 |
|---|---|---|
| Raptor Lake-P | NUC 13 Pro (i5-1340P · i7-1360P · i7-1370P) | 감지 OK, P-core 자동 고정 |
| Meteor Lake | NUC 14 Pro (Core Ultra 7 155H / 165H) | 감지 OK. `check_rt_setup.sh` 의 "지원 예정" WARN 은 감지 단계 안내이며 실패가 아니다 |
| Arrow Lake-H | NUC 15 Pro+ (Core Ultra 7 265H / 9 285H) | 위와 동일 |
| NUC 13 Pro, BIOS HT off | - | `check_rt_setup.sh` FAIL. HT 를 켠다 |
| AMD · 일반 Intel | - | hybrid 분기 없이 물리 코어 순서 |

**BIOS 체크리스트 (Intel NUC)**

- [ ] **Hyper-Threading: Enabled** - slot 변환이 P-core 의 primary / sibling 구분에 의존한다. 설계상 HT 가 없는 CPU (Arrow Lake-S/H/U · Lunar Lake) 는 해당 없고 PASS 로 처리된다.
- [ ] **Intel Turbo Boost: Enabled**
- [ ] **C-states: C1/C2 이하로 제한** (또는 GRUB 에 `intel_idle.max_cstate=1 processor.max_cstate=1`)
- [ ] **Speed Shift (HWP): Enabled** (`intel_pstate=active`)
- [ ] **VT-d / IOMMU: 가능하면 Disabled** (실측 후 결정)
- [ ] **E-core / LP E-core: Enabled 유지** - OS · IRQ 가 쓴다

**`RTC_FORCE_HYBRID_GENERATION`** 은 컨테이너처럼 sysfs 를 못 읽는 환경에서 세대 판정만 덮어쓰는 테스트·CI 용 환경변수다 (`raptor_lake_p` · `meteor_lake` · `arrow_lake_h` · `raptor_lake_p_ht_off` · `none`). **production 에서는 반드시 unset** 한다. 값이 남아 있으면 `check_rt_setup.sh` 가 WARN 한다.

hybrid 감지 확인:

```bash
./repo_scripts/scripts/check_rt_setup.sh    # "Hybrid CPU Detection" 항목
```

## 7. 고급·참고

- 컨테이너 배포: `--cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1` 이 없으면 memlock (RT-HOST-1) 과 SCHED_FIFO (RT-HOST-2) 가 조용히 실패한다
- UDP 버퍼·NIC: `sudo ./repo_scripts/scripts/setup_udp_optimization.sh [NIC]` (`install.sh --rt` 가 이미 호출)
- 사용자 정의 지터 추적: `trace-cmd` (ftrace), LTTng 는 [tracing.md](tracing.md)
- 이 설정의 근거 (cset adopt fail-closed, degraded 격리 예외, RT 코어 집합이 전 tier 에서 같은 이유): [rt-setup-rationale.md](reference/rt-setup-rationale.md)
- 대응 규범 (RT-HOST-1~3): [invariants.md](../agent_docs/invariants.md#rt-host--runtime-preconditions)
- 스레드 모델·실행 컨텍스트: [architecture.md](../agent_docs/architecture.md)
- 외부: [PREEMPT_RT Wiki](https://wiki.linuxfoundation.org/realtime/documentation/start) · [Red Hat RT Tuning Guide](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux_for_real_time/8/html/tuning_guide/index)
