# 실시간 최적화 가이드

**RTC (Real-Time Controller) 병렬 컴퓨팅 아키텍처 상세 문서**

> **Layout 기준**: thread-layout-v3 (PR #97, main `d684c59` 이후). 본 문서의 thread roster · core/priority 표 · 코드 인용 · perf 측정 결과는 모두 v3 기준이다.
>
> **Thread layout SSoT**:
> - [agent_docs/architecture.md](../agent_docs/architecture.md) §Threading Model — RT 정의·priority hierarchy·callback_group↔executor binding 매트릭스
> - [rtc_base/README.md](../rtc_base/README.md) §스레드 구성 — 코어 수별 tier 표 (4/6/8/10/12/14/16)
> - [rtc_base/include/rtc_base/threading/thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp) — `SystemThreadConfigs` 정의 + `SelectThreadConfigs()` 분기
> - 본 문서는 *왜* (rationale + 시스템 설정 + 검증 + 튜닝 가이드) 를 담는다. *무엇* (필드/상수 값) 은 SSoT 가 단일 출처.

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

본 저장소는 **clock_nanosleep RT 루프 + 2-lane SPSC publish 오프로드 + 다중 callback_group/executor** 아키텍처를 사용한다. RT 제어 루프는 ROS 2 executor 를 거치지 않고 `clock_nanosleep(TIMER_ABSTIME)` 절대시간 루프로 직접 실행되어 executor dispatch 지터가 제거된다.

**RT 정의 (v3)**: "RT thread" = controller ↔ hardware/sim 경계의 결정적 tick 만. 구체적으로 `rt_control` (정기 tick) · `rt_inbound` (backend state sub 처리) · `rt_outbound` (backend.WriteCommand 송출) 셋이 SCHED_FIFO 그룹. `nrt_callback` / `nrt_logging` / `arm_driver` / `hand_driver` / `sim_thread` / `viewer` 는 RT 가 아니다 (MPC main/workers 는 별도 RT 그룹).

### 핵심 설계 원칙

1. **clock_nanosleep RT 루프**: `create_wall_timer()` 대신 `clock_nanosleep(TIMER_ABSTIME)` 절대시간 루프 — executor dispatch 지터 제거
2. **2-lane SPSC publish 오프로드**: actuator command 는 `publish_buffer_` (cap 512) → `rt_outbound` (FIFO 65) → `DeviceBackend.WriteCommand`. controller-owned non-RT 토픽 (RobotTarget · Transforms · DigitalTwin · grasp_state · wbc_state · tof_snapshot) 은 `nrt_publish_buffer_` (cap 16) → `nrt_publish_thread` (CFS) → `controller.PublishNonRtSnapshot`. RT loop 가 producer-side 에서 두 큐 모두에 push, by-value (≈12 KB 추가 stack copy)
3. **DeviceBackend cb_group injection**: 모든 backend (UR / udp_hand / mujoco) 의 state/motor/sensor subscription 이 `Configure(node, cfg, state_cb_group)` 로 받은 `cb_group_rt_inbound_` 에 attach — `SubscriptionOptions.callback_group` 으로 RT 경계 subs 가 `rt_inbound_executor` (FIFO 70) 에서 dispatch. MutuallyExclusive 그룹 강제 (SeqLock single-writer 보호)
4. **3 executor 모델**: `rt_inbound_executor` (RT 경계 subs) · `nrt_callback_executor` (controller-owned subs · services · nrt_publish drain) · `nrt_logging_executor` (CSV drain · 지연 E-STOP log). RT loop · rt_outbound · nrt_publish 는 `std::jthread` + eventfd
5. **Overrun recovery**: 놓친 tick skip + 다음 경계 재정렬, 연속 10회 시 E-STOP
6. **CPU Affinity**: 각 스레드를 전용 CPU 코어에 고정. `rt_inbound` 와 `rt_outbound` 는 **same-core** (priority 70 > 65 가 starvation 방지). `arm_driver` / `hand_driver` 는 process-level taskset 으로 분리 코어
7. **RT 스케줄링**: SCHED_FIFO (RT thread), SCHED_OTHER (non-RT) 명시 설정. priority hierarchy `90 > 70 > 65 > 60 > 55` 강제 (`ValidateSystemThreadConfigs` invariant)
8. **메모리 잠금**: `mlockall(MCL_CURRENT | MCL_FUTURE)` 로 페이지 폴트 방지 (rclcpp::init 이전)
9. **시뮬레이션 동기 모드**: CV 기반 wakeup 으로 시뮬레이터 step 과 1:1 동기화 (sim_thread → ControlLoop)

---

## 아키텍처

### 스레드 모델 (layout v3, 6-core 기준)

아래는 **6-core dev box** 기준 thread roster. 4/8/10/12/14/16-core tier 는 §CPU 코어 할당 참조. `SelectThreadConfigs()` 가 `GetPhysicalCpuCount()` 로 물리 코어 수를 감지해 적절한 `SystemThreadConfigs` 를 반환한다.

```
── RT 경계 (controller ↔ hardware/sim) ───────────────────────────────
rt_control (Core 2, SCHED_FIFO prio 90) ← std::jthread, clock_nanosleep
  ├─ ControlLoop() (500Hz default, TIMER_ABSTIME 절대시간)
  ├─ CheckTimeouts() (매 10 tick = 50Hz, inline)
  ├─ producer → publish_buffer_ (cap 512, SPSC)
  ├─ producer → nrt_publish_buffer_ (cap 16, SPSC)
  └─ overrun recovery (skip + 재정렬, 연속 10회 → E-STOP)

rt_inbound_executor (Core 3, SCHED_FIFO prio 70) ← rclcpp::Executor + jthread
  └─ cb_group_rt_inbound_ (MutuallyExclusive)
      ├─ DeviceBackend state subs: /joint_states · hand/{joint,motor,sensor}_states
      └─ ※ 모든 backend (UR/udp_hand/mujoco) 가 Configure(node, cfg, state_cb_group)
         로 cb_group_rt_inbound_ 를 받아 SubscriptionOptions 에 박는다 (Phase 3 invariant)

rt_outbound (Core 3, SCHED_FIFO prio 65) ← std::jthread, eventfd wakeup
  └─ drain publish_buffer_ → DeviceBackend.WriteCommand (actuator only)
     ※ rt_inbound 와 same-core, priority diff 70 > 65 가 starvation 방지

── MPC 그룹 (controller producer / consumer) ────────────────────────
mpc_main (Core 4, SCHED_FIFO prio 60) ← std::jthread, 20-100Hz
  ├─ ReadState() ← rtc::SeqLock<MPCStateSnapshot>::Load()
  ├─ Solve() ← Aligator-based handler
  └─ PublishSolution() → rtc::mpc::TripleBuffer (zero-copy publish)

mpc_worker_0..1 (10+/12+ core only, SCHED_FIFO prio 55)
  └─ 솔버가 spawn 한 parallel rollout / linear solve

── Non-RT executor 그룹 ──────────────────────────────────────────────
nrt_callback_executor (Core 0, SCHED_OTHER nice 0) ← rclcpp::Executor + jthread
  ├─ cb_group_nrt_callback_ (CM-owned: estop_pub_, services, target_sub_ 등)
  ├─ controller LifecycleNode default group (controller-owned RobotTarget subs,
  │     /<active>/grasp_command service)
  └─ nrt_publish_thread (drain nrt_publish_buffer_ → controller.PublishNonRtSnapshot
       — RobotTarget / Transforms / DigitalTwin / grasp_state / wbc_state / tof_snapshot)

nrt_logging_executor (Core 0, SCHED_OTHER nice -5) ← rclcpp::Executor + jthread
  ├─ cm_timing_log.csv drain (ThreadCsvProducer<RtTickTimingPayload, 4096>)
  └─ deferred E-STOP log

── Hardware/sim driver (process-level pin) ──────────────────────────
arm_driver (Core 1, SCHED_OTHER) — UR ros2_control driver, taskset 으로 process-level pin
hand_driver (Core 1, SCHED_OTHER) — udp_hand_node 프로세스, 같은 Core 1 공유 (6-core degraded)
  └─ 내부 kHandUdpRecvConfig (FIFO 65, cpu_core=-1 sentinel — 프로세스 affinity 상속)

sim_thread (cpu_core=-1, SCHED_OTHER) — MuJoCo physics, 6-core 는 cset shield 해제된 코어 roam
viewer    (cpu_core=-1, SCHED_OTHER) — GLFW viewer, 모든 tier 에서 OS 공유
```

> - **RT priority hierarchy**: `90 (rt_control) > 70 (rt_inbound) > 65 (rt_outbound) > 60 (mpc_main) > 55 (mpc_workers)`. `ValidateSystemThreadConfigs()` invariant 가 강제.
> - **rt_inbound + rt_outbound same-core** 는 v3 의 핵심 변화. 이전 (v2) 의 `publish_thread` (SCHED_OTHER nice -3, Core 5) 는 폐기되고, RT 경로의 actuator 송출만 `rt_outbound` (FIFO 65) 로 승격됐다. controller-owned 비 RT 토픽은 `nrt_publish_thread` 로 별도 lane.
> - **6-core 는 degraded mode**: arm/hand_driver 가 Core 1 공유, mpc_workers 없음, sim_thread cpu_core=-1. 결정적 RT 보장은 ≥ 8-core tier 부터.

---

## CPU 코어 할당

`SelectThreadConfigs()` 가 `GetPhysicalCpuCount()` 로 물리 코어 수를 감지해 7-tier (`4 / 6 / 8 / 10 / 12 / 14 / 16+`) 중 적합한 `SystemThreadConfigs` 를 반환한다. 값의 SSoT 는 [rtc_base/include/rtc_base/threading/thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp) — 본 표는 그 미러.

| 스레드 | 4-core¹ | 6-core² | 8-core | 10-core | 12-core | 14-core | 16-core³ |
|---|---|---|---|---|---|---|---|
| **rt_control** (FIFO 90) | Core 1 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 | Core 2 |
| **rt_inbound** (FIFO 70) | Core 2 | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 |
| **rt_outbound** (FIFO 65) | Core 2 (CFS¹) | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 | Core 3 |
| **mpc_main** (FIFO 60) | Core 3 (CFS¹) | Core 4 | Core 4 | Core 4 | Core 4 | Core 4 | Core 9 |
| **mpc_worker_0** (FIFO 55) | — | — | — | Core 5 | Core 5 | Core 5 | Core 10 |
| **mpc_worker_1** (FIFO 55) | — | — | — | — | Core 6 | Core 6 | Core 11 |
| **nrt_logging** (CFS -5) | Core 0 | Core 0 | Core 0 | Core 0 | Core 0 | Core 0 | Core 0 |
| **nrt_callback** (CFS 0) | Core 0 | Core 0 | Core 1 | Core 1 | Core 1 | Core 1 | Core 1 |
| **arm_driver** (CFS 0) | Core 0 | Core 1 | Core 6 | Core 7 | Core 8 | Core 8 | Core 13 |
| **hand_driver** (CFS 0) | Core 0 | Core 1 | Core 5 | Core 6 | Core 7 | Core 7 | Core 12 |
| **sim_thread** (CFS 0) | -1⁴ | -1⁴ | Core 7 | Core 9 | Core 10 | Core 10 | Core 15 |
| **viewer** (CFS 0) | -1⁴ | -1⁴ | -1⁴ | -1⁴ | -1⁴ | -1⁴ | -1⁴ |

> ¹ **4-core 는 degraded mode** — `rt_outbound` / `mpc_main` 이 CFS 로 강등 (RT 자원 부족). 결정적 RT 보장 X, demo / smoke 용도만 권장.
> ² **6-core 는 degraded mode** — `arm_driver` / `hand_driver` 가 Core 1 공유, mpc_workers 없음, `sim_thread` `cpu_core=-1` (cset shield 해제된 코어에서 roam).
> ³ **16-core 는 legacy Option A** — Core 4-8 이 user cset shield, MPC + driver 가 Core 9-13. 향후 NUC hybrid 분석 ([docs/nuc_hybrid_analysis.md](nuc_hybrid_analysis.md)) 에서 Option B (`mpc_main` Core 4) 로 단일화 가능성 검토 중.
> ⁴ **`cpu_core = -1` sentinel** — pthread affinity 호출 skip, scheduler/priority/nice/name 만 적용. process-level taskset (launch script) 가 박은 affinity 를 상속.
>
> **v3 의 핵심 변화**:
> - `rt_inbound + rt_outbound` 가 모든 ≥ 6-core tier 에서 Core 3 **same-core**. priority diff `70 > 65` 가 starvation 방지. ValidateSystemThreadConfigs 에 invariant `rt_outbound.priority < rt_inbound.priority` 강제.
> - v2 의 `publish_thread` (Core 5, SCHED_OTHER nice -3) 폐기 — actuator-only `rt_outbound` (FIFO 65) + controller-owned `nrt_publish_thread` (CFS, nrt_callback core) 두 lane 으로 split. `kUdpRecvConfig*` 7개 상수 삭제 — hand UDP receive 는 `udp_hand_driver` 내부 private `kHandUdpRecvConfig` (cpu_core=-1), 일반 `Transceiver` 는 `kRtUdpRecvConfig` (cpu_core=-1) 를 caller 가 명시 핀.
> - `arm_driver` / `hand_driver` / `sim_thread` / `viewer` 가 `SystemThreadConfigs` 의 1급 필드. Python helper (`rtc_tools.launch.thread_layout.get_{arm,hand}_driver_core() / get_sim_core() / get_viewer_core()`) 가 동일 tier dispatch 미러링 — launch script 가 process-level taskset 으로 사용.
>
> **단조성 불변식**: 물리 코어가 증가하면 per-thread 격리는 절대 감소하지 않는다. `rtc_base/test/test_mpc_thread_config.cpp` 의 `TierIsolationMonotonicity` · `LayoutV3SameCoreRtInboundOutbound` · `LayoutV3ArmHandDriverDisjoint` · `LayoutV3ValidatorCatchesArmHandCollision` · `CpuCoreSentinelValidatesAsRtConfig` 가 tier 쌍 전체 + same-core 결합 + sentinel 처리를 강제한다. drift gate: Python helper 결과 ≡ C++ `SelectThreadConfigs()` 결과는 `rtc_tools/test/test_thread_layout.py` 12 test 로 보장.

### cset shield 범위 (cpu_shield.sh)

`repo_scripts/scripts/cpu_shield.sh` 가 shield 하는 cpuset 범위:

| 물리 코어 | shield cpuset (RT + MPC 영역) | 격리 해제 영역 |
|---|---|---|
| 4 | `1,3` (rt_control + mpc) | Core 0, 2 — OS · driver · nrt |
| 6–9 | `2-4` (rt + mpc_main) | Core 0-1, 5+ — OS · arm/hand_driver · sim |
| 10–11 | `2-5` (+ mpc_worker_0) | Core 0-1, 6-9 |
| 12–15 | `2-6` (+ mpc_worker_1) | Core 0-1, 7+ |
| 16+ | `2-3,9-11` (16-core Option A 의 RT + MPC) | Core 4-8 (user shield), 12+ |

driver core (`arm_driver` / `hand_driver`) 는 SCHED_OTHER 이므로 cset 보호 불요 — process-level taskset 만으로 충분.

### 우선순위 계층

```
SCHED_FIFO prio 90  rt_control          ← controller tick (정기)
                     ↓ preempt
SCHED_FIFO prio 70  rt_inbound          ← backend state subs (joint/motor/sensor)
                     ↓ preempt (same-core)
SCHED_FIFO prio 65  rt_outbound         ← backend.WriteCommand drain
                     ↓ preempt
SCHED_FIFO prio 60  mpc_main            ← MPC solve (rt_inbound 미만)
                     ↓ preempt
SCHED_FIFO prio 55  mpc_worker_0..1     ← parallel rollout / linear solve
─────────────────────── RT / NRT 경계 ───────────────────────
SCHED_FIFO prio 65  kHandUdpRecvConfig  ← hand_driver 프로세스 내부 (분리 프로세스)
SCHED_OTHER nice -5 nrt_logging         ← CSV drain (I/O bound)
SCHED_OTHER nice  0 nrt_callback        ← controller subs · services · nrt_publish_thread
SCHED_OTHER nice  0 arm/hand_driver     ← process-level (taskset only)
SCHED_OTHER nice  0 sim_thread · viewer ← MuJoCo physics · GLFW
```

**설계 원칙**:

- **RT thread = controller↔hardware/sim 경계만**: rt_control · rt_inbound · rt_outbound · mpc_*. 다른 thread 는 NRT.
- **priority 간격 5**: `90/70/65/60/55` — `cgroup.cpu.rt_runtime_us` 가 압박해도 hierarchy 유지. 우선순위 inflation 금지 (CLAUDE.md §3 E-1 escalation).
- **`mpc_main < rt_inbound` 강제** (`ValidateSystemThreadConfigs`): 긴 MPC solve 가 sensor callback latency 를 늘리지 않도록 보장.
- **`mpc_worker_*.priority ≤ mpc_main.priority` 강제**: parallel solver 가 main loop 를 역 preempt 하지 않도록.
- **`rt_outbound.priority < rt_inbound.priority` 강제** (v3 신규): same-core 일 때 starvation 방지. 두 thread 가 다른 core 면 자동 만족 — `ValidateSystemThreadConfigs` 가 `cpu_core` 동일성과 무관히 한쪽이라도 RT 면 check.
- **`arm_driver` / `hand_driver` vs RT controller core disjoint** 강제 (v3 신규): 같은 코어에 배치 시 ValidateSystemThreadConfigs 가 reject. driver 프로세스가 RT thread 코어를 침범하지 못하도록.

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
sudo ./repo_scripts/scripts/setup_nvidia_rt.sh

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
sudo ./repo_scripts/scripts/setup_nvidia_rt.sh

# 또는 RT 커널 빌드 시 자동 처리
sudo ./repo_scripts/scripts/build_rt_kernel.sh
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
sudo ./repo_scripts/scripts/build_rt_kernel.sh

# 비대화형 (CI/자동화용)
sudo ./repo_scripts/scripts/build_rt_kernel.sh --batch

# 다운로드 및 패치만 (빌드 전 확인용)
sudo ./repo_scripts/scripts/build_rt_kernel.sh --dry-run
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
sudo ./repo_scripts/scripts/setup_irq_affinity.sh
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

본 절은 v3 의 핵심 진입점·패턴만 발췌. 코드 자체의 SSoT 는 인용된 파일 — 본 문서가 코드 line / 시그니처 / 상수값을 박제하지 않는다.

### CallbackGroup 생성 ([rt_controller_node.cpp](../rtc_controller_manager/src/rt_controller_node.cpp))

```cpp
void RtControllerNode::CreateCallbackGroups() {
  // cb_group_rt_ 없음 — ControlLoop() + CheckTimeouts() 는 jthread + clock_nanosleep
  // 으로 직접 실행 (executor 미사용)
  cb_group_rt_inbound_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_nrt_logging_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_nrt_callback_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}
```

- **MutuallyExclusive 그룹**: 같은 그룹 내 콜백은 순차 실행 — `rt_inbound_executor` 가 보장하는 SeqLock single-writer 불변식의 근거 (Reentrant 그룹 금지).
- **cb_group → executor binding 매트릭스**는 [agent_docs/architecture.md](../agent_docs/architecture.md) §RtControllerNode 표 참조.

### DeviceBackend cb_group injection ([device_backend.hpp](../rtc_controller_manager/include/rtc_controller_manager/device_backend.hpp))

```cpp
class DeviceBackend {
 public:
  // 모든 backend (UR / udp_hand / mujoco) 구현이 이 시그니처를 따른다.
  // state_cb_group 은 CM 이 만든 cb_group_rt_inbound_ (MutuallyExclusive, FIFO 70 executor).
  virtual void Configure(rclcpp_lifecycle::LifecycleNode* node,
                         const DeviceBackendConfig& config,
                         rclcpp::CallbackGroup::SharedPtr state_cb_group) = 0;
};
```

각 backend 의 `Configure()` 구현은 받은 cb_group 을 자신이 만드는 모든 state/motor/sensor `SubscriptionOptions.callback_group` 에 박는다:

```cpp
// 예: integrated_bringup/src/backends/ur_driver_native_backend.cpp
void UrDriverNativeBackend::Configure(LifecycleNode* node, const DeviceBackendConfig& config,
                                      rclcpp::CallbackGroup::SharedPtr state_cb_group) {
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group;   // ← rt_inbound 로 라우팅

  state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      config.state_topic, state_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); },
      sub_opts);
}
```

- **v3 핵심 invariant**: 모든 backend state-lane sub 은 `cb_group_rt_inbound_` 에 attach. 잊으면 RCL 이 silently 노드 default group (= nrt_callback) 으로 fallback — build 와 unit test 는 통과하지만 RT 경계가 nrt 로 떨어진다.
- **회귀 차단 test**: [test_device_backend_cb_group_injection.cpp](../rtc_controller_manager/test/test_device_backend_cb_group_injection.cpp) 가 `CallbackGroup::find_subscription_ptrs_if` reverse-lookup 으로 sub 가 주입된 group 에 등록됐는지 ABI 수준에서 검증. 새 backend 추가 시 본 test 의 parametrised case 에도 추가할 것.
- ARCH-3 강제: `Configure()` 시그니처 변경이 모든 backend 를 build break — 새 backend 가 cb_group plumbing 을 잊을 수 없다 (Phase 0 audit 옵션 (b)).

### RT 루프 ([rt_controller_node_rt_loop.cpp](../rtc_controller_manager/src/rt_controller_node_rt_loop.cpp))

```cpp
void RtControllerNode::RtLoopEntry(const ThreadConfig& cfg) {
  ApplyThreadConfig(cfg);  // Core 2, SCHED_FIFO 90
  // ... (clock_nanosleep TIMER_ABSTIME 절대시간 루프, sim CV wakeup branch, overrun recovery)

  // 매 tick 끝에 producer-side 2-lane fan-out (Phase 2 of thread-layout-v3)
  static_cast<void>(publish_buffer_.Push(snap));           // → rt_outbound
  if (publish_eventfd_ >= 0) static_cast<void>(eventfd_write(publish_eventfd_, 1));
  static_cast<void>(nrt_publish_buffer_.Push(snap));       // → nrt_publish_thread
  if (nrt_publish_eventfd_ >= 0) static_cast<void>(eventfd_write(nrt_publish_eventfd_, 1));
}
```

- **by-value fan-out**: `snap` (≈6 KB) 을 두 SPSC 큐에 그대로 push. cap 16 의 nrt_publish_buffer 가 saturate 되어도 RT lane (cap 512) 은 별개 — drop 은 `nrt_pub_drops` counter 로 관측.
- **RT-safe push**: `Push` 와 `eventfd_write` 둘 다 nothrow + 결과 `static_cast<void>` 무시. push 실패 (full) = silent drop.
- **2 drain thread**:
  - `rt_outbound` (FIFO 65) → `DeviceBackend.WriteCommand` (actuator)
  - `nrt_publish_thread` (nrt_callback core, CFS) → `controller.PublishNonRtSnapshot` (controller-owned non-RT topic)
- Drain 측 by-value invariant 는 [publish_buffer.hpp](../rtc_base/include/rtc_base/threading/publish_buffer.hpp) header doc 참조.
- Sim CV wakeup · overrun recovery · 연속 10회 → E-STOP 메커니즘은 동일 (PR #97 무관).

### 스레드 설정 ([thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp))

`SystemThreadConfigs` 의 1급 필드 (v3):

```cpp
namespace rtc {

struct SystemThreadConfigs {
  ThreadConfig rt_control;     // FIFO 90
  ThreadConfig rt_inbound;     // FIFO 70
  ThreadConfig rt_outbound;    // FIFO 65 (4-core 만 CFS 강등)
  ThreadConfig nrt_logging;    // CFS nice -5
  ThreadConfig nrt_callback;   // CFS nice  0
  ThreadConfig arm_driver;     // CFS, process-level taskset (launch script 가 박음)
  ThreadConfig hand_driver;    // CFS, 동일
  ThreadConfig sim_thread;     // CFS, cpu_core=-1 sentinel 가능
  ThreadConfig viewer;         // CFS, cpu_core=-1 sentinel 가능
  MpcThreadConfig mpc;         // mpc_main + workers[0..2]
};

inline SystemThreadConfigs SelectThreadConfigs() noexcept {
  const int ncpu = GetPhysicalCpuCount();
  // 7-tier dispatch: >= 16 / 14 / 12 / 10 / 8 / 6 / else 4-core fallback.
  // 각 tier 의 ThreadConfig 상수는 헤더에 inline const 로 정의 (kRtControlConfig*,
  // kRtInboundConfig*, kRtOutboundConfig*, kNrtCallbackConfig*, kNrtLoggingConfig*,
  // kArmDriverConfig*, kHandDriverConfig*, kSimThreadConfig*, kViewerConfig*).
  ...
}

}  // namespace rtc
```

- **`udp_recv` 필드 / `kUdpRecvConfig*` 7개 상수 완전 삭제** (v3 Phase 5). 대체:
  - `udp_hand_driver` 내부 private 상수 [`kHandUdpRecvConfig`](../udp_hand_driver/include/udp_hand_driver/udp_hand_controller.hpp) (FIFO 65, `cpu_core=-1` sentinel — `hand_driver` 프로세스의 taskset affinity 상속).
  - 일반 `rtc_communication::Transceiver` default 는 [`kRtUdpRecvConfig`](../rtc_communication/include/rtc_communication/transceiver.hpp) (FIFO 65, `cpu_core=-1`) — caller 가 명시 핀 필요시 override.
  - `SystemThreadConfigs` 에는 UDP receive 가 보이지 않는다 — hand UDP receive 는 별도 프로세스 (`udp_hand_node`) 의 *내부* thread 이므로 single-process 의 SSoT 표현 대상이 아님.
- **`cpu_core = -1` sentinel**: `ValidateThreadConfig` 가 통과시키고 `ApplyThreadConfig` 가 affinity step 만 skip (scheduler/priority/nice/name 은 정상 적용). 처리 함정은 [feedback_sentinel_introduction_validator_audit](../../.claude/projects/-home-junho-ros2-ws-rtc-ws-src-rtc-framework/memory/feedback_sentinel_introduction_validator_audit.md) 참조.

### 스레드 적용 ([thread_utils.hpp](../rtc_base/include/rtc_base/threading/thread_utils.hpp))

```cpp
[[nodiscard]] inline bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept {
  // 1. CPU affinity (cpu_core >= 0 일 때만 — -1 sentinel 은 skip)
  if (cfg.cpu_core >= 0) {
    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(cfg.cpu_core, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) return false;
  }
  // 2. Scheduler policy + priority / nice
  sched_param param{};
  if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) {
    param.sched_priority = cfg.sched_priority;
    if (pthread_setschedparam(pthread_self(), cfg.sched_policy, &param) != 0) return false;
  } else {
    setpriority(PRIO_PROCESS, 0, cfg.nice_value);
  }
  // 3. Thread name (15자 제한, pthread_setname_np)
  char name_buf[16];
  std::strncpy(name_buf, cfg.name, sizeof(name_buf) - 1);
  name_buf[sizeof(name_buf) - 1] = '\0';
  pthread_setname_np(pthread_self(), name_buf);
  return true;
}
```

`ApplyThreadConfigWithFallback()` 은 RT 권한 부족 시 SCHED_OTHER 로 자동 강등 — degraded mode (4-core / 권한 없음) 에서도 prog 가 죽지 않도록.

### main() 함수 ([rt_controller_main_impl.cpp](../rtc_controller_manager/src/rt_controller_main_impl.cpp))

```cpp
int RtControllerMain(int argc, char** argv, const std::string& node_name) {
  // 1. mlockall — rclcpp::init 이전 (page fault 방지)
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { /* WARN */ }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rtc::RtControllerNode>(node_name);
  // (실제 main 은 lifecycle_executor 로 configure/activate 까지 spin 한 뒤 dedicated
  // executor 로 switch 한다 — 3-phase pattern. 자세한 흐름은 위 파일 참조.)

  const auto cfgs = rtc::SelectThreadConfigs();

  // 2. RT loop + rt_outbound + nrt_publish_thread (jthread, executor 미사용)
  // on_activate 가 StartRtLoop(cfgs.rt_control) / StartPublishLoop(cfgs.rt_outbound)
  // / StartNrtPublishLoop(cfgs.nrt_callback) 를 호출.

  // 3. 3 executor 생성 + cb_group binding
  rclcpp::executors::SingleThreadedExecutor rt_inbound_executor;
  rclcpp::executors::SingleThreadedExecutor nrt_logging_executor;
  rclcpp::executors::SingleThreadedExecutor nrt_callback_executor;

  rt_inbound_executor.add_callback_group(node->GetRtInboundGroup(), node->get_node_base_interface());
  nrt_logging_executor.add_callback_group(node->GetNrtLoggingGroup(), node->get_node_base_interface());
  nrt_callback_executor.add_callback_group(node->GetNrtCallbackGroup(), node->get_node_base_interface());
  // 추가로 nrt_callback_executor 에 CM/controller LifecycleNode default group 도 add_node — 본
  // matrix 는 architecture.md §RtControllerNode 표 참조.

  // 4. Executor 스레드 spawn + ApplyThreadConfig
  std::jthread t_rt_inbound  = make_thread(rt_inbound_executor,  cfgs.rt_inbound);   // FIFO 70 Core 3
  std::jthread t_nrt_logging = make_thread(nrt_logging_executor, cfgs.nrt_logging);  // CFS -5 Core 0
  std::jthread t_nrt_callback= make_thread(nrt_callback_executor,cfgs.nrt_callback); // CFS  0 Core 1

  // 5. Join + lifecycle shutdown (on_deactivate / on_cleanup 가 jthread 들 join + reset)
  rclcpp::shutdown();
  return 0;
}
```

- 본 함수 자체는 `cm_executable` 별 wrapper 가 `RtControllerMain(argc, argv, "integrated_rt_controller")` 처럼 호출. 실제 시그니처와 lifecycle wiring 은 [rt_controller_main_impl.cpp](../rtc_controller_manager/src/rt_controller_main_impl.cpp) 참조.
- **rt_outbound 의 시작/종료** 는 RtControllerNode 의 lifecycle 콜백 (`on_activate` / `on_deactivate`) 에서 — main 이 직접 spawn 하지 않는다.

---

## 검증 방법

### 1. 스레드 설정 확인 (ps -T)

```bash
PID=$(pgrep -f integrated_rt_controller)
ps -T -p $PID -o tid,comm,rtprio,psr,cls,pri
```

**v3 layout 6-core 정상 출력** (PR #97 Phase 8 smoke baseline):

```
    TID COMMAND         RTPRIO PSR CLS PRI
   ... rt_control          90   2  FF 130   ← Core 2, FIFO 90 (clock_nanosleep jthread)
   ... rt_inbound          70   3  FF 110   ← Core 3, FIFO 70 (rt_inbound_executor)
   ... rt_outbound         65   3  FF 105   ← Core 3, FIFO 65 — rt_inbound 와 same-core ✓
   ... mpc_main            60   4  FF 100   ← Core 4, FIFO 60
   ... nrt_callback         -   0  TS   0   ← Core 0, CFS
   ... nrt_callback         -   0  TS   0   ← executor worker (2개일 수 있음)
   ... nrt_logging          -   0  TS   0   ← Core 0, CFS nice -5
   ... dds.shm.*            -   - …          ← FastDDS 내부 (CFS, 코어 임의)
   ... integrated-ust       -   - …          ← LTTng UST (있을 때만)
   ... integrated_rt_c      -   -  TS   0   ← 메인 thread (rclcpp main, idle)
```

**CLS**: `FF`=SCHED_FIFO, `RR`=SCHED_RR, `TS`=SCHED_OTHER (CFS). **RTPRIO**: RT priority (`-` = non-RT). **PSR**: 현재 실행 코어.

**v3 invariant 체크**:
- `rt_control` priority 90 > `rt_inbound` 70 > `rt_outbound` 65 > `mpc_main` 60
- `rt_inbound.PSR == rt_outbound.PSR` (same-core, ≥ 6-core tier)
- `nrt_callback` / `nrt_logging` 은 CLS=TS (RTPRIO=-)
- 어느 thread 도 `sensor_io` / `publish_thread` / `aux` / `logger` / `udp_recv` 이름이면 안 됨 — v2 잔재 (사람 grep 으로 detect)

`udp_hand_node` 는 별도 프로세스이므로 그 PID 로 별도 `ps -T`:
```bash
ps -T -p $(pgrep -f udp_hand_node) -o tid,comm,rtprio,psr,cls
# 내부에 kHandUdpRecvConfig 가 만든 FIFO 65 thread 가 있음 (이름 컨벤션은
# udp_hand_driver/udp_hand_constants.hpp 참조)
```

### 2. CPU Affinity 확인

```bash
# 특정 thread 의 affinity (TID 는 위 ps -T 출력)
taskset -cp <TID>
# 예: rt_outbound TID → "current affinity list: 3"

# 프로세스 전체 (arm/hand_driver process-level pin 검증)
taskset -cp $(pgrep -f udp_hand_node)
taskset -cp $(pgrep -f ur_ros2_control_node)   # UR driver
```

### 3. 실시간 지터 측정

#### 3a. cm_timing_log.csv (in-process, RT loop 직접 측정 — 권장)

`<session>/timing/cm_timing_log.csv` 에 RT loop 의 per-tick `t_state_us` / `t_compute_us` / `t_publish_us` / `t_total_us` / `jitter_us` 가 SPSC drain 으로 기록된다. **PR #97 Phase 8 baseline** (6-core dev box, sim 90s, 59,065 ticks @ 500 Hz):

| metric | value | budget (500 Hz) |
|---|---|---|
| mean | 45.66 µs | 2000 µs |
| p50 | 43.88 µs | — |
| p95 | 66.50 µs | — |
| p99 | 88.65 µs | — |
| max (post-warm) | 238.92 µs | 2000 µs |
| overruns / skips / pub_drops / nrt_pub_drops | **0 / 0 / 0 / 0** | 0 |

분석 스크립트는 `rtc_tools/utils/cm_timing_analysis.py` (median-of-diff budget inference + first-tick init spike filter). max 첫 tick 은 항상 init spike (mlockall page touch + 첫 RCLCPP_INFO) — `filter_outliers=True` 로 제외.

#### 3b. cyclictest (OS jitter 독립 측정 — 보조)

```bash
sudo apt install rt-tests

# 500Hz (2ms 주기) 테스트, RT 코어 2 에서 실행
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo \
    --interval=2000 --loops=100000 --affinity=2 --histogram=200
```

목표: PREEMPT_RT 커널 + cset shield 적용 시 Max < 50 µs.

### 4. ROS 2 제어 주파수

```bash
# UR 의 forward_position_controller 명령 (rt_outbound → DDS)
ros2 topic hz /forward_position_controller/commands

# Hand UDP joint_command (rt_outbound → UDP via udp_hand_node)
ros2 topic hz /hand/joint_command

# 출력:
# average rate: 500.012
#        min: 0.001996s max: 0.002005s std dev: 0.000001s window: 503
#                                       ^ std dev < 5 µs 목표
```

### 5. Context Switch / CPU migration

```bash
sudo perf stat -e context-switches,cpu-migrations -p $PID sleep 10

# 목표:
#   context-switches: < 1000/sec   (jthread sleep 외 거의 없음)
#   cpu-migrations:   = 0          (affinity 고정 ⇒ 마이그레이션 0)
```

### 6. perf flame graph (hotpath 프로파일링)

`repo_scripts/scripts/flame.sh` 가 RT loop hotpath 의 flame graph 생성:

```bash
./repo_scripts/scripts/flame.sh                    # 기본: cycles 이벤트
./repo_scripts/scripts/flame.sh --event task-clock # 30 µs tick 짧으면 task-clock 권장
                                                    # (cycles:P 로는 RT tick 이 안 잡힘)
```

본 저장소의 RT loop 가 30-90 µs 로 매우 짧으므로 cycles:P 샘플링은 sub-sampling 으로 hotspot 을 놓친다 — `task-clock` 이벤트 권장 (memory `reference_perf_tooling` 참조).

---

## 문제 해결

> **자동 진단**: `check_rt_setup.sh`를 실행하면 8개 카테고리의 RT 설정을 자동 점검합니다.
> ```bash
> ./repo_scripts/scripts/check_rt_setup.sh           # 상세 출력
> ./repo_scripts/scripts/check_rt_setup.sh --fix     # 수정 명령 제안
> ./repo_scripts/scripts/check_rt_setup.sh --summary # 1줄 요약
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
PID=$(pgrep -nf "integrated_rt_controller")
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
sudo ./repo_scripts/scripts/setup_nvidia_rt.sh

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
sudo ./repo_scripts/scripts/setup_nvidia_rt.sh  # [10/11] CPU governor 설정

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

**최종 업데이트**: 2026-05-19 — thread-layout-v3 본문 반영 (PR #97 main `d684c59` 이후, issue #98 close).
**SSoT**: [agent_docs/architecture.md](../agent_docs/architecture.md) + [rtc_base/README.md](../rtc_base/README.md) + [thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp).
