# 실시간 최적화 가이드 (v5.17.0 baseline; current trunk = v5.21.0)

**RTC (Real-Time Controller) 병렬 컴퓨팅 아키텍처 상세 문서**

> **Currency (2026-04-26)**: RT layout (thread tier 4/8/10/12/14/16-core) 은 commit `5f0680c` (2026-04-22, unified rework + `TierIsolationMonotonicity`) 이후 변경되지 않음. v5.18~v5.21 은 controller / MPC / WbcState 추가로 RT subsystem 외 변경. 본 문서는 trunk 와 일치.

---

## 목차

- [개요](#개요)
- [아키텍처](#아키텍처)
- [CPU-코어-할당](#cpu-코어-할당)
- [시스템-설정](#시스템-설정)
- [코드-구조](#코드-구조)
- [검증-방법](#검증-방법)
- [문제-해결](#문제-해결)
- [고급-튜닝](#고급-튜닝)

---

## 개요

v5.17.0은 **clock_nanosleep RT 루프 + SPSC publish 오프로드 + CallbackGroup 기반 멀티스레드 executor** 아키텍처를 사용합니다. RT 제어 루프는 ROS2 executor를 거치지 않고 `clock_nanosleep(TIMER_ABSTIME)` 절대시간 루프로 직접 실행되어 executor dispatch 지터가 제거됩니다.

### 핵심 설계 원칙

1. **clock_nanosleep RT 루프**: `create_wall_timer()` 대신 `clock_nanosleep(TIMER_ABSTIME)` 절대시간 루프 사용 -- executor dispatch 지터 제거
2. **Publish 오프로드**: SPSC 버퍼 + 전용 publish thread -- DDS 직렬화/syscall이 RT 경로에서 제거
3. **Executor 3개**: sensor, log, aux (rt_executor 제거, jthread 대체)
4. **Overrun recovery**: 놓친 tick skip + 다음 경계 재정렬, 연속 10회 시 E-STOP
5. **CPU Affinity**: 각 스레드를 전용 CPU 코어에 고정
6. **RT 스케줄링**: SCHED_FIFO (RT), SCHED_OTHER (non-RT) 명시적 설정
7. **메모리 잠금**: `mlockall(MCL_CURRENT | MCL_FUTURE)`로 페이지 폴트 방지
8. **시뮬레이션 동기 모드**: CV 기반 wakeup으로 시뮬레이터 step과 1:1 동기화

---

## 아키텍처

### 스레드 모델 (v5.17.0)

```
rt_loop (Core 2, SCHED_FIFO prio 90) <- std::jthread, clock_nanosleep
  |- ControlLoop() (500Hz, TIMER_ABSTIME 절대시간)
  |- CheckTimeouts() (매 10틱 = 50Hz, inline)
  +- overrun recovery (skip + 재정렬, consecutive E-STOP)

publish_thread (Core 5, SCHED_OTHER nice -3) <- std::jthread, SPSC drain
  +- ControlPublishBuffer -> 모든 publish() 호출

sensor_executor (Core 3, SCHED_FIFO prio 70) <- ROS2 Executor
  |- joint_state_sub_
  |- target_sub_
  +- hand_state_sub_

log_executor (Core 4, SCHED_OTHER nice -5) <- ROS2 Executor
  +- drain_timer_ (SpscLogBuffer -> CSV)

aux_executor (Core 5, SCHED_OTHER) <- ROS2 Executor
  +- estop_pub_

mpc_main (Phase 5, Core 4, SCHED_FIFO prio 60) <- std::jthread, 20Hz
  |- ReadState() <- rtc::SeqLock<MPCStateSnapshot>::Load()
  |- Solve() <- MockMPCThread (placeholder; Aligator는 후속 패키지)
  +- PublishSolution() -> rtc::mpc::TripleBuffer (zero-copy publish)

mpc_worker_0..1 (Phase 5, 12/16-core only, SCHED_FIFO prio 55)
  +- 솔버가 spawn한 parallel rollout / linear solve 작업 수행
```

> 위 코어 배치는 6-core 기준입니다. 4/8/10/12/16-core 시스템은 자동으로 다른 레이아웃을 선택합니다.
> `SelectThreadConfigs()` 함수가 `GetPhysicalCpuCount()`로 물리 코어 수를 감지하여 적절한 설정을 반환합니다.
> Phase 5에서 `SystemThreadConfigs.mpc` (`MpcThreadConfig`)가 추가되어 `SelectThreadConfigs()`가 tier별 `kMpcConfig{4,6,8,10,12,16}Core`를 함께 반환합니다.

---

## CPU 코어 할당

`thread_config.hpp`에 정의된 코어 배치입니다. `SelectThreadConfigs()`가 물리 코어 수에 따라 자동 선택합니다.

### 6-Core 시스템 (기본)

> `udp_recv`가 Core 5에 배치되어 `sensor_io`(Core 3)가 전용 코어를 확보합니다.
> UDP 버스트 시에도 `JointStateCallback` 지연이 발생하지 않습니다.
> Phase 5: `mpc_main`이 `logger`와 Core 4를 공유하지만 `SCHED_FIFO 60`이므로 `logger`(SCHED_OTHER)는 자동으로 양보됩니다.

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 | 비고 |
|------|------|-----------|---------------|------------|------|
| 0-1 | OS / DDS / NIC IRQ | SCHED_OTHER | - | - | cset shield로 동적 격리 |
| 2 | RT Control | SCHED_FIFO | 90 | `rt_control` | 500Hz + 50Hz E-STOP |
| 3 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` | joint_state, target, hand 콜백 |
| 4 | Logging | SCHED_OTHER | nice -5 | `logger` | 100Hz CSV drain |
| 4 | **MPC main** (Phase 5) | **SCHED_FIFO** | **60** | `mpc_main` | **20Hz solve, logger 코어 공유** |
| 5 | Publish offload | SCHED_OTHER | nice -3 | `rt_publish` | SPSC drain -> publish |
| 5 | UDP recv | SCHED_FIFO | 65 | `udp_recv` | Hand UDP 수신 |
| 5 | Aux | SCHED_OTHER | nice 0 | `aux` | E-STOP publisher |

**GRUB 설정**: `nohz_full=2-5 rcu_nocbs=2-5` (`isolcpus`는 사용하지 않고 `cset shield`로 대체)

### 4-Core 시스템 (폴백)

> Phase 5: 4코어에서 MPC는 `SCHED_OTHER nice=-5`로 강등됩니다 (RT로 돌릴 여유 부족 — 10 Hz로 자동 감속 권장).

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 |
|------|------|-----------|---------------|------------|
| 0 | OS / DDS / IRQ | SCHED_OTHER | - | - |
| 1 | RT Control | SCHED_FIFO | 90 | `rt_control` |
| 2 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` |
| 2 | UDP recv | SCHED_FIFO | 65 | `udp_recv` |
| 3 | Logging | SCHED_OTHER | nice -5 | `logger` |
| 3 | Publish offload | SCHED_OTHER | nice -3 | `rt_publish` |
| 3 | Aux | SCHED_OTHER | nice 0 | `aux` |
| 3 | **MPC main** (Phase 5) | **SCHED_OTHER** | **nice -5** | `mpc_main` (degraded mode) |

**GRUB 설정**: `nohz_full=1-3 rcu_nocbs=1-3`

### 8-Core 시스템 (Phase 5: MPC dedicated)

> Phase 5에서 Core 4가 MPC main 전용으로 배정되어 `udp_recv` 4→5, `logger` 5→6, `aux`/`publish` 6→7로 시프트되었습니다.

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 |
|------|------|-----------|---------------|------------|
| 0-1 | OS / DDS / NIC IRQ | SCHED_OTHER | - | - |
| 2 | RT Control | SCHED_FIFO | 90 | `rt_control` |
| 3 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` |
| 4 | **MPC main** (Phase 5) | **SCHED_FIFO** | **60** | `mpc_main` (dedicated) |
| 5 | UDP recv (shifted from 4) | SCHED_FIFO | 65 | `udp_recv` |
| 6 | Logging (shifted from 5) | SCHED_OTHER | nice -5 | `logger` |
| 7 | Aux (shifted from 6) | SCHED_OTHER | nice 0 | `aux` |
| 7 | Publish offload (shifted from 6) | SCHED_OTHER | nice -3 | `rt_publish` |

**GRUB 설정**: `nohz_full=2-7 rcu_nocbs=2-7`

### 10-Core 시스템 (unified layout, MPC + 1 worker)

> **2026-04 unified layout**: 이전에는 shield 2-6이 "user" 예약 영역이고 rtc_controller_manager가 Core 7-9 (3코어)만 썼기 때문에 8-core보다 격리 품질이 나빴습니다. 새 레이아웃은 RT 쓰레드를 8-core와 동일한 Core 2부터 배치하고 Core 5에 MPC worker 0을 추가합니다. cset shield 2-8이 RT 전 영역을 보호하며, Core 9는 MuJoCo sim / monitoring용 spare.

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 |
|------|------|-----------|---------------|------------|
| 0-1 | OS / DDS / NIC IRQ | SCHED_OTHER | - | - |
| 2 | RT Control | SCHED_FIFO | 90 | `rt_control` |
| 3 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` |
| 4 | **MPC main** | **SCHED_FIFO** | **60** | `mpc_main` (dedicated) |
| 5 | **MPC worker 0** | **SCHED_FIFO** | **55** | `mpc_worker_0` |
| 6 | UDP recv | SCHED_FIFO | 65 | `udp_recv` (dedicated) |
| 7 | Logging | SCHED_OTHER | nice -5 | `logger` |
| 8 | Aux | SCHED_OTHER | nice 0 | `aux` |
| 8 | Publish offload | SCHED_OTHER | nice -3 | `rt_publish` |
| 9 | MuJoCo sim / spare | SCHED_OTHER | - | `sim_thread` (Tier 3) |

### 12-Core 시스템 (unified layout, MPC + 2 workers)

> **2026-04 unified layout**: 이전 12-core는 Core 11에 udp/logger/aux/publish가 모두 몰렸고 shield 2-6 (5코어)은 rtc_controller_manager 관점에서 낭비였습니다. 새 레이아웃은 10-core를 그대로 확장해 MPC worker 1을 Core 6에 추가하고, UDP/logger/aux/publish가 각각 전용 코어를 가집니다. cset shield 2-9이 RT 전 영역을 보호.

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 |
|------|------|-----------|---------------|------------|
| 0-1 | OS / DDS / NIC IRQ | SCHED_OTHER | - | - |
| 2 | RT Control | SCHED_FIFO | 90 | `rt_control` |
| 3 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` |
| 4 | **MPC main** | **SCHED_FIFO** | **60** | `mpc_main` (dedicated) |
| 5 | **MPC worker 0** | **SCHED_FIFO** | **55** | `mpc_worker_0` |
| 6 | **MPC worker 1** | **SCHED_FIFO** | **55** | `mpc_worker_1` |
| 7 | UDP recv | SCHED_FIFO | 65 | `udp_recv` (dedicated) |
| 8 | Logging | SCHED_OTHER | nice -5 | `logger` |
| 9 | Aux | SCHED_OTHER | nice 0 | `aux` |
| 9 | Publish offload | SCHED_OTHER | nice -3 | `rt_publish` |
| 10 | MuJoCo sim | SCHED_OTHER | - | `sim_thread` (Tier 3) |
| 11 | Spare | - | - | user shield / viewer |

### 14-Core 시스템 (unified layout, dedicated sim core)

> **2026-04 신규**: 이전에는 14-core 머신이 12-core 분기에 흡수되어 Core 12-13이 **완전히 미사용**이었습니다. 새 tier는 12-core RT 레이아웃을 그대로 유지하고 Core 10을 MuJoCo sim 전용으로 배정, Core 11-13을 user shield/monitoring spare로 제공.

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 |
|------|------|-----------|---------------|------------|
| 0-1 | OS / DDS / NIC IRQ | SCHED_OTHER | - | - |
| 2 | RT Control | SCHED_FIFO | 90 | `rt_control` |
| 3 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` |
| 4 | **MPC main** | **SCHED_FIFO** | **60** | `mpc_main` (dedicated) |
| 5 | **MPC worker 0** | **SCHED_FIFO** | **55** | `mpc_worker_0` |
| 6 | **MPC worker 1** | **SCHED_FIFO** | **55** | `mpc_worker_1` |
| 7 | UDP recv | SCHED_FIFO | 65 | `udp_recv` (dedicated) |
| 8 | Logging | SCHED_OTHER | nice -5 | `logger` |
| 9 | Aux | SCHED_OTHER | nice 0 | `aux` |
| 9 | Publish offload | SCHED_OTHER | nice -3 | `rt_publish` |
| 10 | MuJoCo sim | SCHED_OTHER | - | `sim_thread` (Tier 3) |
| 11-13 | Spare | - | - | user shield / viewer / monitoring |

### 16-Core 시스템 (cset shield, Phase 5: MPC + 2 workers)

> cset shield Core 4-8. rtc_controller_manager는 Core 0-3, 9+에서 실행.
> Phase 5에서 Core 9–11이 MPC main+workers 전용으로 배정되어 `udp_recv` 9→12, `logger` 10→13, `aux`/`publish` 11→14, MuJoCo `sim_thread_core` 12→15로 시프트되었습니다.

| Core | 용도 | Scheduler | Priority/Nice | 스레드 이름 |
|------|------|-----------|---------------|------------|
| 0-1 | OS / DDS / NIC IRQ | SCHED_OTHER | - | - |
| 2 | RT Control | SCHED_FIFO | 90 | `rt_control` |
| 3 | Sensor I/O | SCHED_FIFO | 70 | `sensor_io` |
| 4-8 | cset shield "user" | - | - | 예약 |
| 9 | **MPC main** (Phase 5) | **SCHED_FIFO** | **60** | `mpc_main` (dedicated) |
| 10 | **MPC worker 0** (Phase 5) | **SCHED_FIFO** | **55** | `mpc_worker_0` |
| 11 | **MPC worker 1** (Phase 5) | **SCHED_FIFO** | **55** | `mpc_worker_1` |
| 12 | UDP recv (shifted from 9) | SCHED_FIFO | 65 | `udp_recv` |
| 13 | Logging (shifted from 10) | SCHED_OTHER | nice -5 | `logger` |
| 14 | Aux (shifted from 11) | SCHED_OTHER | nice 0 | `aux` |
| 14 | Publish offload (shifted from 11) | SCHED_OTHER | nice -3 | `rt_publish` |
| 15 | MuJoCo sim (shifted from 12) | SCHED_OTHER | - | sim_thread (Tier 3) |

> **단조성 불변식 (Monotonicity Invariant)**: 물리 코어 수가 증가하면 per-thread 격리 품질은 절대 감소하지 않는다. `rtc_base/test/test_mpc_thread_config.cpp::TierIsolationMonotonicity`가 모든 tier 쌍에 대해 (1) MPC worker 수 단조 비감소, (2) 전용 코어 개수 단조 비감소, (3) 10-core 이상에서 `mpc_main` / `udp_recv` / `logger`가 서로 다른 코어에 위치한다는 것을 강제합니다. 새 레이아웃 추가 시 이 테스트를 반드시 업데이트하세요.

### 우선순위 계층

```
SCHED_FIFO prio 90 (rt_control)            <- 최고 우선순위
           |
SCHED_FIFO prio 70 (sensor_io)             <- 센서 데이터 수신
           |
SCHED_FIFO prio 65 (udp_recv)              <- Hand UDP 수신
           |
SCHED_FIFO prio 60 (mpc_main)              <- Phase 5: MPC solve (sensor 미만)
           |
SCHED_FIFO prio 55 (mpc_worker_0..1)       <- Phase 5: MPC parallel solve
           |
SCHED_OTHER nice -5 (logger)               <- I/O bound
           |
SCHED_OTHER nice -3 (rt_publish)           <- publish 오프로드
           |
SCHED_OTHER nice 0  (aux)                  <- 보조 작업
```

**설계 원칙**:
- **RT 작업**: SCHED_FIFO (선점형, 우선순위 고정)
- **I/O 작업**: SCHED_OTHER (CFS, nice value로 조정)
- **우선순위 간격**: 20 (prio 90 vs 70)으로 충분한 여유 확보
- **Phase 5 MPC 우선순위 규칙**: `mpc_main < sensor_io` 가 강제됨 (`ValidateSystemThreadConfigs`에서 검증). 긴 MPC solve가 sensor callback latency를 늘리지 않도록 보장.
- **Phase 5 worker 규칙**: `mpc_worker_*.priority ≤ mpc_main.priority` 강제. parallel solver가 main loop를 역preempt하는 것을 방지.

---

## 시스템 설정

### 1. RT 권한 설정 (필수)

```bash
# realtime 그룹 생성
sudo groupadd realtime

# 현재 사용자를 realtime 그룹에 추가
sudo usermod -aG realtime $USER

# limits.conf 설정
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf

# 로그아웃 후 재로그인 필수
# 또는
newgrp realtime

# 확인
ulimit -r  # 출력: 99 (RT priority limit)
ulimit -l  # 출력: unlimited (memlock)
```

### 2. CPU Isolation (권장)

> **주의: SMT/Hyper-Threading 환경**
>
> `isolcpus`는 **물리 코어** 기준으로 설정해야 합니다. `nproc`은 논리 코어(SMT 포함)를
> 반환하므로, HT가 켜진 시스템에서 `nproc` 값을 그대로 사용하면 과도한 격리가 발생합니다.
>
> | CPU | 물리 코어 | `nproc` | 올바른 `isolcpus` | 잘못된 `isolcpus` |
> |-----|-----------|---------|-------------------|-------------------|
> | i7-8700 (6C/12T) | 6 | 12 | `2-5` | `2-11` (OS에 2코어만 남김) |
> | i5-8250U (4C/8T) | 4 | 8 | `1-3` | `1-7` (OS에 1코어만 남김) |
>
> 물리 코어 수 확인: `lscpu -p=Core,Socket | grep -v '^#' | sort -u | wc -l`
>
> `setup_nvidia_rt.sh`, `setup_irq_affinity.sh`는 자동으로 물리 코어를 감지합니다.

```bash
# /etc/default/grub 편집
sudo nano /etc/default/grub

# 다음 줄 추가 또는 수정 (6-core 기준, 물리 코어 기준)
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5"

# GRUB 업데이트
sudo update-grub

# 재부팅
sudo reboot

# 확인
cat /sys/devices/system/cpu/isolated
# 출력: 2-5

cat /proc/cmdline | grep isolcpus
# 출력: ... isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5 ...
```

**isolcpus 옵션 설명**:
- `isolcpus=2-5`: Core 2-5를 일반 스케줄러에서 제외
- `nohz_full=2-5`: Core 2-5에서 타이머 인터럽트 최소화 (tickless)
- `rcu_nocbs=2-5`: RCU 콜백을 다른 코어에서 처리

### 3. CPU 성능 모드

> **중요**: RT 코어뿐 아니라 **모든 CPU 코어** (OS 코어 포함)의 governor를 `performance`로
> 설정해야 합니다. OS 코어가 `powersave`이면 compositor 프레임 드롭이 발생합니다.

#### 자동 설정 (권장)

```bash
# NVIDIA 시스템: setup_nvidia_rt.sh [10/11] 단계에서 자동 설정
sudo ./rtc_scripts/scripts/setup_nvidia_rt.sh

# 비-NVIDIA 시스템: install.sh가 자동으로 cpu-governor-performance.service 생성
./install.sh robot   # 또는 full
```

`setup_nvidia_rt.sh`는 다음을 수행합니다:
1. sysfs를 통해 즉시 모든 코어를 `performance`로 변경
2. `cpu-governor-performance.service` (systemd oneshot) 생성 및 활성화
3. 재부팅 시 자동 적용 (`cpupower` 사용 가능하면 사용, 아니면 sysfs fallback)

#### 수동 설정

```bash
# cpufrequtils 설치
sudo apt install cpufrequtils

# 모든 코어를 performance 모드로 설정
sudo cpupower frequency-set -g performance

# 확인
cpupower frequency-info | grep "current policy"
# 출력: current policy: frequency should be within ... (performance)

# 부팅 시 자동 적용 (방법 1: cpufrequtils)
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils

# 부팅 시 자동 적용 (방법 2: systemd service -- setup_nvidia_rt.sh가 생성하는 방식)
sudo systemctl status cpu-governor-performance.service
```

#### 확인

```bash
# 모든 코어의 governor 확인 (powersave가 있으면 문제)
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo "$(basename $(dirname $(dirname $f))): $(cat $f)"
done

# systemd service 상태 확인
sudo systemctl status cpu-governor-performance.service
```

### 4. NVIDIA + RT 커널 공존 (NVIDIA GPU 사용 시)

NVIDIA 드라이버는 기본적으로 `CONFIG_PREEMPT_RT` 커널에서 빌드를 거부합니다. 커스텀 RT 커널에서
NVIDIA DKMS 모듈을 빌드하려면 **3가지 차단 레이어**를 모두 우회해야 합니다:

| 차단 레이어 | 원인 | 해결 |
|-------------|------|------|
| `dkms autoinstall` apport 검증 | 커스텀 커널명 거부 (`linux-headers-*-rt-custom` 미지원) | `dkms build -m nvidia -v VERSION -k KERNEL` 직접 호출 |
| `conftest.sh` RT 감지 | `CONFIG_PREEMPT_RT` 감지 -> 빌드 거부 | `IGNORE_PREEMPT_RT_PRESENCE=1` 환경변수 |
| `dkms.conf` BUILD_EXCLUSIVE | `BUILD_EXCLUSIVE_CONFIG="!CONFIG_PREEMPT_RT"` | sed로 주석 처리 |

#### 자동 설정 (권장)

```bash
# setup_nvidia_rt.sh [9/11] 단계에서 자동 처리
sudo ./rtc_scripts/scripts/setup_nvidia_rt.sh

# 또는 RT 커널 빌드 시 자동 처리
sudo ./rtc_scripts/scripts/build_rt_kernel.sh
```

#### 수동 설정

```bash
# 1. NVIDIA DKMS 버전 확인
NVIDIA_DKMS_VER=$(dkms status nvidia 2>/dev/null | head -1 | awk -F'[,/]' '{print $2}' | tr -d ' ')
KERNEL_VER=$(uname -r)

# 2. dkms.conf의 BUILD_EXCLUSIVE_CONFIG 주석 처리
sudo sed -i 's/^\(BUILD_EXCLUSIVE_CONFIG=.*PREEMPT_RT\)/#\1/' \
  /usr/src/nvidia-${NVIDIA_DKMS_VER}/dkms.conf

# 3. RT 감지 우회 + 빌드
sudo IGNORE_PREEMPT_RT_PRESENCE=1 IGNORE_CC_MISMATCH=1 \
  dkms build -m nvidia -v ${NVIDIA_DKMS_VER} -k ${KERNEL_VER}
sudo dkms install -m nvidia -v ${NVIDIA_DKMS_VER} -k ${KERNEL_VER}

# 4. 모듈 로드
sudo modprobe nvidia

# 5. 영구 설정 (/etc/environment에 추가)
echo "IGNORE_PREEMPT_RT_PRESENCE=1" | sudo tee -a /etc/environment
```

#### 검증

```bash
nvidia-smi                        # GPU 인식 확인
dkms status nvidia                # 현재 커널에 installed 확인
grep IGNORE_PREEMPT_RT /etc/environment  # 영구 설정 확인
```

### 5. PREEMPT_RT 커널 (선택, 최대 성능)

#### Option A: LowLatency 커널 (더 쉬운 방법)
```bash
sudo apt install linux-lowlatency-hwe-22.04
sudo reboot

# 확인
uname -v
# 출력: #XX~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC ... lowlatency
```

#### Option B: PREEMPT_RT 커널 (최대 성능, 빌드 필요)

자동화 스크립트 제공:
```bash
# 대화형 (menuconfig 포함)
sudo ./rtc_scripts/scripts/build_rt_kernel.sh

# 비대화형 (CI/자동화용)
sudo ./rtc_scripts/scripts/build_rt_kernel.sh --batch

# 다운로드 및 패치만 (빌드 전 확인용)
sudo ./rtc_scripts/scripts/build_rt_kernel.sh --dry-run
```

지원 버전:
- Ubuntu 24.04: 커널 6.8.2 + RT patch 6.8.2-rt11
- Ubuntu 22.04: 커널 6.6.127 + RT patch 6.6.127-rt69

```bash
# 재부팅 후 확인
uname -v | grep PREEMPT_RT
```

참고: https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/start

### 6. IRQ Affinity (고급)

자동화 스크립트 제공:
```bash
# 자동 설정 (NIC IRQ를 Core 0-1로 pin)
sudo ./rtc_scripts/scripts/setup_irq_affinity.sh
```

수동 설정:
```bash
# 모든 IRQ를 Core 0-1로 제한 (RT 코어 2-5를 보호)
for irq in $(ls /proc/irq/); do
    [ -d "/proc/irq/$irq" ] && echo 3 | sudo tee /proc/irq/$irq/smp_affinity > /dev/null 2>&1
done
# 0x3 = 0b0011 = Core 0, 1

# 확인
cat /proc/interrupts | grep -E "(CPU0|CPU1)"  # 대부분의 IRQ가 Core 0-1에 집중
```

---

## 코드 구조

### CallbackGroup 생성 (`rt_controller_node.cpp`)

```cpp
void RtControllerNode::CreateCallbackGroups() {
  // cb_group_rt_ 제거 (v5.17.0) -- ControlLoop() + CheckTimeouts()는
  // RtLoopEntry() jthread에서 clock_nanosleep으로 직접 실행
  cb_group_sensor_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_log_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_aux_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
}
```

**MutuallyExclusive**: 같은 그룹 내 콜백은 순차 실행 (thread-safe)

### RT 루프 (`rt_controller_node.cpp`, clock_nanosleep 기반)

```cpp
void RtControllerNode::RtLoopEntry(const ThreadConfig& cfg) {
  ApplyThreadConfig(cfg);  // Core 2, SCHED_FIFO 90
  rt_loop_running_.store(true, std::memory_order_release);

  uint32_t timeout_tick = 0;

  if (use_sim_time_sync_) {
    // === 시뮬레이션 모드: CV 기반 wakeup ===
    // state_cv_.wait_for()로 /joint_states 도착 대기
    // 타임아웃 시 TriggerGlobalEstop("sim_sync_timeout") + shutdown
    // ...
  } else {
    // === 실로봇 모드: deterministic clock_nanosleep 500Hz ===
    const int64_t period_ns = static_cast<int64_t>(1.0e9 / control_rate_);
    struct timespec next_wake{};
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    while (rt_loop_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
      // 다음 절대 시간으로 advance
      next_wake.tv_nsec += period_ns;
      if (next_wake.tv_nsec >= 1'000'000'000L) {
        next_wake.tv_sec  += next_wake.tv_nsec / 1'000'000'000L;
        next_wake.tv_nsec %= 1'000'000'000L;
      }

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);

      // Overrun 감지 & 복구
      struct timespec now_ts{};
      clock_gettime(CLOCK_MONOTONIC, &now_ts);
      const int64_t lag_ns = now_ns - wake_ns;

      if (lag_ns > period_ns) {
        // 놓친 tick 수만큼 next_wake를 advance (burst 방지)
        const int64_t missed_ticks = lag_ns / period_ns;
        next_wake.tv_nsec += missed_ticks * period_ns;
        // ... normalize ...
        overrun_count_.fetch_add(1, std::memory_order_relaxed);
        skip_count_.fetch_add(missed_ticks, std::memory_order_relaxed);

        // 연속 overrun 감지 -> E-STOP
        if (consecutive_overruns_ >= kMaxConsecutiveOverruns) {  // 10
          TriggerGlobalEstop("consecutive_overrun");
        }
      } else {
        consecutive_overruns_.store(0, std::memory_order_relaxed);
      }

      ControlLoop();

      // 50Hz watchdog (매 10틱)
      static constexpr int kWatchdogCheckDivisor = 10;
      if (enable_estop_ && ++timeout_tick % kWatchdogCheckDivisor == 0) {
        CheckTimeouts();
      }
    }
  }
}
```

> `clock_nanosleep(TIMER_ABSTIME)` 절대시간 루프로 executor dispatch 지터 제거.
> 시뮬레이션 모드에서는 `condition_variable` wakeup으로 시뮬레이터와 1:1 동기화.
> CheckTimeouts()는 매 10틱(50Hz) inline 호출.

### 스레드 설정 (`thread_config.hpp`)

```cpp
namespace rtc {

struct ThreadConfig {
  int         cpu_core;         // CPU affinity (0-based core index)
  int         sched_policy;     // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int         sched_priority;   // 1-99 for SCHED_FIFO/RR, ignored for OTHER
  int         nice_value;       // -20 to 19 for SCHED_OTHER, ignored for FIFO/RR
  const char* name;             // Thread name for debugging (max 15 chars)
};

// 사전 정의된 설정 (6-core)
inline const ThreadConfig kRtControlConfig{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig{
    .cpu_core       = 5,       // Core 3 -> Core 5 (sensor_io와 분리)
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig{
    .cpu_core       = 4,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig{
    .cpu_core       = 5,       // udp_recv와 Core 5 공유 (aux는 이벤트 기반, 경량)
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig{
    .cpu_core       = 5,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// ... kRtControlConfig4Core, kRtControlConfig8Core, kRtControlConfig10Core,
//     kRtControlConfig12Core, kRtControlConfig16Core 등도 동일 구조로 정의
}
```

### 런타임 코어 선택 (`thread_utils.hpp`)

```cpp
struct SystemThreadConfigs {
  ThreadConfig rt_control;
  ThreadConfig sensor;
  ThreadConfig udp_recv;
  ThreadConfig logging;
  ThreadConfig aux;
  ThreadConfig publish;
};

inline SystemThreadConfigs SelectThreadConfigs() noexcept {
  const int ncpu = GetPhysicalCpuCount();
  if (ncpu >= 16) return { ...16Core configs... };
  if (ncpu >= 12) return { ...12Core configs... };
  if (ncpu >= 10) return { ...10Core configs... };
  if (ncpu >= 8)  return { ...8Core configs... };
  if (ncpu >= 6)  return { ...6Core (default) configs... };
  return { ...4Core fallback configs... };
}
```

### 스레드 적용 (`thread_utils.hpp`)

```cpp
[[nodiscard]] inline bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept {
  // 1. CPU affinity 설정
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cfg.cpu_core, &cpuset);
  if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
    return false;
  }

  // 2. Scheduler policy 설정
  sched_param param{};
  if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) {
    param.sched_priority = cfg.sched_priority;
    if (pthread_setschedparam(pthread_self(), cfg.sched_policy, &param) != 0) {
      return false;  // RT 권한 부족
    }
  } else {
    setpriority(PRIO_PROCESS, 0, cfg.nice_value);
  }

  // 3. Thread name 설정 (15자 제한)
  char name_buf[16];
  std::strncpy(name_buf, cfg.name, sizeof(name_buf) - 1);
  name_buf[sizeof(name_buf) - 1] = '\0';
  pthread_setname_np(pthread_self(), name_buf);
  return true;
}
```

### main() 함수 (`rt_controller_main_impl.cpp`)

```cpp
int RtControllerMain(int argc, char** argv) {
  // 1. 메모리 잠금 (페이지 폴트 방지) -- rclcpp::init 보다 먼저 호출
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] mlockall failed\n");
  }

  // 2. CPU isolation 상태 확인 (경고만, 실패 시 계속)
  // /sys/devices/system/cpu/isolated 읽어서 격리 여부 출력

  rclcpp::init(argc, argv);
  auto node = std::make_shared<RtControllerNode>();

  // 3. 물리 코어 수에 따라 스레드 설정 자동 선택
  const auto cfgs = SelectThreadConfigs();

  // 5. RT loop + Publish offload (jthread, executor 미사용)
  node->StartRtLoop(cfgs.rt_control);       // clock_nanosleep 500Hz
  node->StartPublishLoop(cfgs.publish);      // SPSC drain -> publish

  // 6. ROS2 Executor 생성 (3개)
  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  sensor_executor.add_callback_group(node->GetSensorGroup(), ...);
  log_executor.add_callback_group(node->GetLogGroup(), ...);
  aux_executor.add_callback_group(node->GetAuxGroup(), ...);

  // 7. Executor 스레드 생성 + RT 설정 (ApplyThreadConfig 호출)
  auto t_sensor = make_thread(sensor_executor, cfgs.sensor);
  auto t_log    = make_thread(log_executor,    cfgs.logging);
  auto t_aux    = make_thread(aux_executor,    cfgs.aux);

  // 8. Join + 종료
  t_sensor.join(); t_log.join(); t_aux.join();
  node->StopRtLoop();
  node->StopPublishLoop();

  rclcpp::shutdown();
  return 0;
}
```

---

## 검증 방법

### 1. 스레드 설정 확인

```bash
# ur5e_rt_controller 프로세스 찾기
PID=$(pgrep -f ur5e_rt_controller)

# 스레드 목록 + 스케줄러 정책 + CPU 코어
ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

**출력 예시 (6-core)**:
```
  PID   TID CLS RTPRIO PSR COMMAND
 1234  1234  TS      -   0 ur5e_rt_contro (메인 스레드, comm 컬럼은 15자)
 1234  1235  FF     90   2 rt_control         <- Core 2, FIFO 90 (clock_nanosleep jthread)
 1234  1236  TS      -   5 rt_publish         <- Core 5, OTHER   (SPSC publish jthread)
 1234  1237  FF     70   3 sensor_io          <- Core 3, FIFO 70 (ROS2 Executor)
 1234  1238  TS      -   4 logger             <- Core 4, OTHER   (ROS2 Executor)
 1234  1239  TS      -   5 aux                <- Core 5, OTHER   (ROS2 Executor)
```

**CLS 값**:
- `FF`: SCHED_FIFO (실시간)
- `RR`: SCHED_RR (실시간 라운드 로빈)
- `TS`: SCHED_OTHER (일반)

**RTPRIO**: RT 우선순위 (1-99, `-`는 비RT)
**PSR**: 현재 실행 중인 CPU 코어 (0-based)

### 2. CPU Affinity 확인

```bash
# rt_control 스레드의 affinity 확인
taskset -cp 1235  # TID = 1235
# 출력: pid 1235's current affinity list: 2
```

### 3. 실시간 지터 측정

```bash
# cyclictest 설치
sudo apt install rt-tests

# 500Hz (2ms 주기) 테스트, Core 2에서 실행
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo \
    --interval=2000 --loops=100000 --affinity=2 --histogram=200
```

**목표 출력**:
```
T: 0 ( 1235) P:90 I:2000 C: 100000 Min:      3 Act:    5 Avg:    6 Max:      48
                                                           ^
                                                    Max 지터 < 50us
```

### 4. ROS2 제어 주파수

```bash
# 명령 토픽 주파수 확인
ros2 topic hz /forward_position_controller/commands

# 출력:
average rate: 500.123
        min: 0.001998s max: 0.002002s std dev: 0.000001s window: 503
                           ^
                    표준편차 < 1us 목표
```

### 5. Context Switch 횟수

```bash
# 10초 동안 context switch 측정
sudo perf stat -e context-switches,cpu-migrations -p $PID sleep 10

# 출력:
#   context-switches: 1,234  (120/sec)  <- 목표: < 1000/sec
#   cpu-migrations:   0                 <- 목표: 0 (affinity 고정)
```

---

## 문제 해결

> **자동 진단**: `check_rt_setup.sh`를 실행하면 8개 카테고리의 RT 설정을 자동 점검합니다.
> ```bash
> ./rtc_scripts/scripts/check_rt_setup.sh           # 상세 출력
> ./rtc_scripts/scripts/check_rt_setup.sh --fix     # 수정 명령 제안
> ./rtc_scripts/scripts/check_rt_setup.sh --summary # 1줄 요약
> ```
>
> 검증 카테고리: RT Kernel, CPU Isolation, GRUB Parameters, RT Permissions,
> IRQ Affinity, Network/UDP, NVIDIA (DKMS 상태 + persistence mode + `IGNORE_PREEMPT_RT_PRESENCE` 영구 설정),
> CPU Frequency (모든 코어 governor + `cpu-governor-performance.service`)

### 권한 부족 에러

**증상**:
```
[WARN] Thread config failed for 'rt_control' (need realtime permissions)
```

**원인**: RT priority 설정 권한 없음

**해결**:
```bash
# 1. realtime 그룹 확인
groups | grep realtime

# 2. limits.conf 확인
cat /etc/security/limits.conf | grep realtime
# 출력:
# @realtime - rtprio 99
# @realtime - memlock unlimited

# 3. 로그아웃 후 재로그인 (필수)

# 4. 확인
ulimit -r  # 99 출력되어야 함
```

### CPU Isolation 미적용

**증상**: PSR 값이 0-1로 계속 변경됨

**확인**:
```bash
cat /proc/cmdline | grep isolcpus
# 출력 없음 -> 설정 안 됨
```

**해결**:
```bash
sudo nano /etc/default/grub
# GRUB_CMDLINE_LINUX_DEFAULT에 isolcpus=2-5 추가
sudo update-grub
sudo reboot
```

### 여전히 높은 지터

**원인 1**: LowLatency 커널 미사용
```bash
uname -v | grep -i low
# 출력 없음 -> 일반 커널

# 해결
sudo apt install linux-lowlatency-hwe-22.04
sudo reboot
```

**원인 2**: IRQ가 RT 코어에서 발생
```bash
cat /proc/interrupts | grep -E "(CPU2|CPU3|CPU4|CPU5)"
# 많은 IRQ -> 문제

# 해결: IRQ를 Core 0-1로 이동
for irq in $(ls /proc/irq/); do
    [ -d "/proc/irq/$irq" ] && echo 3 | sudo tee /proc/irq/$irq/smp_affinity > /dev/null 2>&1
done
```

**원인 3**: DDS 스레드가 RT 코어 사용
```bash
# CycloneDDS 0.11+ (Jazzy): <Internal><Threads> XML은 더 이상 지원되지 않음.
# 대신 taskset으로 DDS 스레드를 비-RT 코어에 고정 (robot.launch.py에서 자동 처리).
# 수동 확인:
PID=$(pgrep -nf "ur5e_rt_controller")
for TID in $(ls /proc/$PID/task/); do
  COMM=$(cat /proc/$PID/task/$TID/comm 2>/dev/null)
  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo "OTHER")
  CPUS=$(taskset -cp $TID 2>/dev/null | awk -F: '{print $2}')
  echo "  TID=$TID  comm=$COMM  policy=$POLICY  cpus=$CPUS"
done

# CycloneDDS XML 설정은 rtc_controller_manager/config/cyclone_dds.xml 참조
# (멀티캐스트 비활성화, 소켓 버퍼 확대, write batching 등 성능 최적화 포함)
export CYCLONEDDS_URI=file://$(ros2 pkg prefix rtc_controller_manager)/share/rtc_controller_manager/config/cyclone_dds.xml
```

### NVIDIA DKMS 빌드 실패 (RT 커널)

**증상**:
```
ERROR (dkms apport): kernel package linux-headers-6.8.2-rt11-rt-custom is not supported
```
또는 `nvidia-smi` 실행 불가 (NVIDIA 모듈 미로드)

**원인**: NVIDIA DKMS가 커스텀 RT 커널에서 빌드를 거부 (3가지 차단 레이어)

**해결**:
```bash
# 자동 해결 (권장)
sudo ./rtc_scripts/scripts/setup_nvidia_rt.sh

# 검증
nvidia-smi
dkms status nvidia
```

자세한 내용: [NVIDIA + RT 커널 공존](#4-nvidia--rt-커널-공존-nvidia-gpu-사용-시)

### 500Hz 미달성

**원인**: CPU frequency scaling (powersave governor)
```bash
# 모든 코어의 governor 확인
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo "$(basename $(dirname $(dirname $f))): $(cat $f)"
done
# 어떤 코어든 powersave -> 문제

# 해결 (자동)
sudo ./rtc_scripts/scripts/setup_nvidia_rt.sh  # [10/11] CPU governor 설정

# 해결 (수동)
sudo cpupower frequency-set -g performance
```

---

## 고급 튜닝

### 1. DDS 최적화

#### CycloneDDS (권장)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix rtc_controller_manager)/share/rtc_controller_manager/config/cyclone_dds.xml
```

**`cyclone_dds.xml`** 주요 최적화 항목 (전체 설정: `rtc_controller_manager/config/cyclone_dds.xml`):

| 설정 | 값 | 효과 |
|------|-----|------|
| `AllowMulticast` | `false` | 단일 호스트에서 IGMP 오버헤드 제거 |
| `LeaseDuration` | `5s` | 죽은 participant 빠른 감지 (기본 10s) |
| `SPDPInterval` | `1s` | 시작 시 빠른 discovery (기본 30s) |
| `WriteBatchFlushInterval` | `8 us` | 시스콜 횟수 감소 |
| `NackDelay` | `10 ms` | 재전송 속도 10x 향상 (기본 100ms) |
| `PreEmptiveAckDelay` | `10 ms` | 갭 감지 시 빠른 응답 |
| `HeartbeatInterval` | `100 ms` | 안정적 손실 감지 |
| `SocketReceiveBufferSize` | `8 MB` | DDS 버스트 수용 (sysctl rmem_max 이하) |
| `SocketSendBufferSize` | `2 MB` | 송신 버퍼 확보 |
| `SynchronousDeliveryLatencyBound` | `inf` | subscriber 콜백 wake-up 지연 제거 |
| `MaxQueuedRexmitMessages` | `256` | 버스트 퍼블리시 시 패킷 병합 |

> **NOTE**: CycloneDDS 0.11+ (Jazzy)에서 `<Internal><Threads>` 제거됨.
> DDS 스레드 affinity는 `taskset`으로 처리 (`robot.launch.py` 참조).

#### Fast DDS
```xml
<!-- fastdds_profile.xml -->
<profiles>
  <participant profile_name="rt_profile">
    <rtps>
      <builtin>
        <discovery_config>
          <leaseDuration>
            <sec>1</sec>
            <nanosec>0</nanosec>
          </leaseDuration>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

### 2. 커널 파라미터 튜닝

```bash
# /etc/sysctl.d/99-realtime.conf 생성
sudo tee /etc/sysctl.d/99-realtime.conf << 'EOF'
# RT 스케줄러 설정
kernel.sched_rt_runtime_us = -1  # RT 스케줄러 CPU 시간 제한 해제

# 페이지 스왈 비활성화
vm.swappiness = 0

# 커널 타이머 주파수 증가 (선택)
kernel.timer_migration = 0
EOF

sudo sysctl -p /etc/sysctl.d/99-realtime.conf
```

### 3. 네트워크 튜닝

```bash
# UDP 버퍼 크기 증가 (ROS 2 DDS + 핸드 UDP 통신)
# setup_udp_optimization.sh에서 자동 설정됨 (/etc/sysctl.d/99-ros2-udp.conf)
sudo sysctl -w net.core.rmem_max=2147483647     # DDS가 setsockopt으로 필요한 만큼 확보
sudo sysctl -w net.core.rmem_default=212992      # Linux 기본값 유지 (모든 소켓에 거대 버퍼 할당 방지)
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=212992
sudo sysctl -w net.core.netdev_max_backlog=5000
```

### 4. 지터 프로파일링

```bash
# ftrace로 지터 원인 추적
sudo trace-cmd record -e sched:sched_switch -e sched:sched_wakeup \
    -p function_graph -g ControlLoop \
    -P 1235  # rt_control 스레드 TID

sudo trace-cmd report > trace.txt

# trace.txt에서 긴 실행 시간 함수 찾기
grep -E "[0-9]{3,}\.[0-9]{3} us" trace.txt
```

---

## 참고 문서

### 공식 문서
- [ROS2 Executors & Callback Groups](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)
- [Linux RT Application Development (PDF)](https://linutronix.de/PDF/2009_Kuhberg_Linux-RT.pdf)
- [PREEMPT_RT Wiki](https://wiki.linuxfoundation.org/realtime/documentation/start)

### 튜토리얼
- [Linux Realtime Tuning Guide (Red Hat)](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux_for_real_time/8/html/tuning_guide/index)
- [ROS2 Real-Time Working Group](https://github.com/ros-realtime/ros-realtime-rpi4-image)

### 도구
- `cyclictest`: 지터 측정 (`rt-tests` 패키지)
- `perf`: 성능 프로파일링 (`linux-tools-common`)
- `trace-cmd`: ftrace 프론트엔드
- `cpupower`: CPU 주파수 제어

---

**최종 업데이트**: 2026-03-29
**작성자**: RTC Framework Team
**버전**: v5.17.0
