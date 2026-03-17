# 실시간 최적화 가이드 (v4.2.0 / v5.1.0 업데이트)

**UR5e RT Controller 병렬 컴퓨팅 아키텍처 상세 문서**

---

## 목차

- [개요](#개요)
- [아키텍처-변경사항](#아키텍처-변경사항)
- [CPU-코어-할당](#cpu-코어-할당)
- [성능-개선](#성능-개선)
- [시스템-설정](#시스템-설정)
- [코드-구조](#코드-구조)
- [검증-방법](#검증-방법)
- [문제-해결](#문제-해결)
- [4-Core-시스템-대응](#4-core-시스템-대응)
- [고급-튜닝](#고급-튜닝)

---

## 개요

v4.2.0은 **CallbackGroup 기반 멀티스레드 executor 아키텍처**를 도입하여 실시간 성능을 대폭 개선했습니다. 단일 스레드 executor의 직렬화 문제를 해결하고, CPU affinity 및 RT 스케줄링으로 제어 지터를 10배 감소시켰습니다.

### 주요 개선사항

| 메트릭 | v4.0.0 | v4.2.0 | 개선율 |
|--------|--------|--------|--------|
| 제어 지터 | ~500μs | <50μs | **10배** |
| E-STOP 반응 | ~100ms | <20ms | **5배** |
| CPU 사용률 | ~30% | ~25% | 17% 감소 |
| Context Switch | ~5000/s | ~1000/s | 80% 감소 |
| Priority Inversion | 발생 가능 | **제거** | - |
| CPU Migration | 빈번 | **차단** | - |

---

## 아키텍처 변경사항

### 기존 (v4.0.0)

```
SingleThreadedExecutor (1개 스레드)
  ├─ control_timer_ (500Hz)  ← RT
  ├─ timeout_timer_ (50Hz)   ← RT
  ├─ logging_timer_ (100Hz)  ← non-RT (파일 I/O)
  ├─ joint_state_sub_
  ├─ target_sub_
  └─ hand_state_sub_
```

**문제점**:
1. **Priority Inversion**: 100Hz 로깅 I/O가 500Hz 제어 루프 dispatch 지연
2. **직렬화**: 모든 콜백이 순차 실행 → 병렬 처리 불가
3. **CPU Migration**: OS가 스레드를 다른 코어로 이동 → cache miss
4. **RT 우선순위 미설정**: SCHED_OTHER 기본 정책 사용

### v4.2.0 → v5.16.0 (현재)

```
rt_loop (Core 2, SCHED_FIFO prio 90) ← std::jthread, clock_nanosleep
  ├─ ControlLoop() (500Hz, TIMER_ABSTIME 절대시간)
  ├─ CheckTimeouts() (매 10틱 = 50Hz, inline)
  └─ overrun recovery (skip + 재정렬, consecutive E-STOP)

publish_thread (Core 5, SCHED_OTHER nice -3) ← std::jthread, SPSC drain
  └─ ControlPublishBuffer → 모든 publish() 호출

sensor_executor (Core 3, SCHED_FIFO prio 70) ← ROS2 Executor
  ├─ joint_state_sub_
  ├─ target_sub_
  └─ hand_state_sub_

log_executor (Core 4, SCHED_OTHER nice -5) ← ROS2 Executor
  └─ drain_timer_ (SpscLogBuffer → CSV)

aux_executor (Core 5, SCHED_OTHER) ← ROS2 Executor
  └─ estop_pub_
```

**v5.16.0 개선사항** (v4.2.0 기반):
1. **clock_nanosleep RT 루프**: `create_wall_timer()` → `clock_nanosleep(TIMER_ABSTIME)` — executor dispatch 지터 제거
2. **Publish 오프로드**: SPSC 버퍼 + 전용 publish thread — DDS 직렬화/syscall RT 경로에서 제거
3. **Executor 축소**: 4개 → 3개 (rt_executor 제거, jthread 대체)
4. **Overrun recovery**: 놓친 tick skip + 다음 경계 재정렬, 연속 10회 시 E-STOP
5. **CPU Affinity**: 각 스레드를 전용 CPU 코어에 고정
6. **RT 스케줄링**: SCHED_FIFO (RT), SCHED_OTHER (non-RT) 명시적 설정
7. **메모리 잠금**: mlockall(MCL_CURRENT | MCL_FUTURE)로 페이지 폴트 방지

---

## CPU 코어 할당

### 6-Core 시스템 (권장)

> **v5.1.0 업데이트**: `udp_recv`가 Core 3 → Core 5로 이동되었습니다. `sensor_io`(Core 3)가 전용 코어를 확보하여 UDP 버스트 시 `JointStateCallback` 지연 및 오발동 E-STOP 위험이 제거되었습니다.

| Core | 용도 | Scheduler | Priority | 스레드 유형 | 주기 |
|------|------|-----------|----------|------------|------|
| 0-1 | OS / DDS / UR 드라이버 / NIC IRQ | SCHED_OTHER | - | - | - |
| 2 | RT Control | SCHED_FIFO | 90 | `std::jthread` (clock_nanosleep) | 500Hz + 50Hz E-STOP |
| 3 | Sensor I/O | SCHED_FIFO | 70 | ROS2 Executor (`cb_group_sensor_`) | 비정기 |
| 4 | Logging | SCHED_OTHER | nice -5 | ROS2 Executor (`cb_group_log_`) | 100Hz drain |
| 4 | Status Monitor | SCHED_OTHER | nice -2 | `std::jthread` | 10Hz |
| 4 | Hand Failure Detector | SCHED_OTHER | nice -2 | `std::jthread` | 50Hz |
| 5 | Publish offload | SCHED_OTHER | nice -3 | `std::jthread` (SPSC drain) | 500Hz (event) |
| 5 | UDP recv + Aux | FIFO/65 + OTHER/0 | - | `std::jthread` + ROS2 Executor | 비정기 |

**isolcpus 설정**: Core 2-5를 OS 스케줄러에서 격리하여 RT 전용으로 사용

### 우선순위 계층

```
SCHED_FIFO prio 90 (rt_control)      ← 최고 우선순위
           ↓
SCHED_FIFO prio 70 (sensor_io)       ← 센서 데이터 수신
           ↓
SCHED_FIFO prio 65 (udp_recv)        ← Hand UDP 수신
           ↓
SCHED_OTHER nice -5 (logger)         ← I/O bound
           ↓
SCHED_OTHER nice -2 (status_mon)     ← 상태 모니터 (10 Hz)
SCHED_OTHER nice -2 (hand_detect)    ← 핸드 이상 감지 (50 Hz)
           ↓
SCHED_OTHER nice 0  (aux)            ← 보조 작업
```

**설계 원칙**:
- **RT 작업**: SCHED_FIFO (선점형, 우선순위 고정)
- **I/O 작업**: SCHED_OTHER (CFS, nice value로 조정)
- **우선순위 간격**: 20 (prio 90 vs 70)으로 충분한 여유 확보

---

## 성능 개선

### 제어 지터 감소

**v4.0.0** (SingleThreadedExecutor):
```
Cycle time histogram (μs):
  200-300: ████████████████████████████  28%
  300-400: ██████████████████████████    26%
  400-500: ████████████████              16%
  500-600: ██████████                    10%  ← 목표 초과
  600+:    ████████                       8%  ← 위험 영역

Max jitter: 892μs
```

**v4.2.0** (Multi-threaded Executors):
```
Cycle time histogram (μs):
    0-10: ██████████████████████████████  30%
   10-20: ████████████████████████████    28%
   20-30: ██████████████████              18%
   30-40: ██████████                      10%
   40-50: ████                             4%
   50+:   ██                               2%  ← 드물게 발생

Max jitter: 48μs  (18배 개선)
```

### E-STOP 반응 시간

| 시나리오 | v4.0.0 | v4.2.0 | 개선율 |
|---------|--------|--------|--------|
| 로봇 데이터 타임아웃 | ~100ms | ~20ms | 5배 |
| 핸드 데이터 타임아웃 | ~200ms | ~40ms | 5배 |
| E-STOP 명령 전파 | ~50ms | ~10ms | 5배 |

**개선 이유**: timeout_timer_ (50Hz)가 전용 RT 스레드에서 실행 → 지연 없음

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
> 설정해야 합니다. OS 코어가 `powersave`이면 compositor 프레임 드롭 → 화면 찢김이 발생합니다.

#### 자동 설정 (권장)

```bash
# NVIDIA 시스템: setup_nvidia_rt.sh [10/11] 단계에서 자동 설정
sudo ./ur5e_rt_controller/scripts/setup_nvidia_rt.sh

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

# 부팅 시 자동 적용 (방법 2: systemd service — setup_nvidia_rt.sh가 생성하는 방식)
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
| `conftest.sh` RT 감지 | `CONFIG_PREEMPT_RT` 감지 → 빌드 거부 | `IGNORE_PREEMPT_RT_PRESENCE=1` 환경변수 |
| `dkms.conf` BUILD_EXCLUSIVE | `BUILD_EXCLUSIVE_CONFIG="!CONFIG_PREEMPT_RT"` | sed로 주석 처리 |

#### 자동 설정 (권장)

```bash
# setup_nvidia_rt.sh [9/11] 단계에서 자동 처리
sudo ./ur5e_rt_controller/scripts/setup_nvidia_rt.sh

# 또는 RT 커널 빌드 시 자동 처리
sudo ./ur5e_rt_controller/scripts/build_rt_kernel.sh
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
sudo ./ur5e_rt_controller/scripts/build_rt_kernel.sh

# 비대화형 (CI/자동화용)
sudo ./ur5e_rt_controller/scripts/build_rt_kernel.sh --batch

# 다운로드 및 패치만 (빌드 전 확인용)
sudo ./ur5e_rt_controller/scripts/build_rt_kernel.sh --dry-run
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
sudo ./ur5e_rt_controller/scripts/setup_irq_affinity.sh
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
  // cb_group_rt_ 제거 (v5.16.0) — ControlLoop() + CheckTimeouts()는
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

### RT 루프 (v5.16.0, clock_nanosleep 기반)

```cpp
void RtControllerNode::RtLoopEntry(const ThreadConfig& cfg) {
  ApplyThreadConfig(cfg);  // Core 2, SCHED_FIFO 90

  const int64_t period_ns = static_cast<int64_t>(1.0e9 / control_rate_);
  struct timespec next_wake{};
  clock_gettime(CLOCK_MONOTONIC, &next_wake);

  while (rt_loop_running_ && rclcpp::ok()) {
    next_wake.tv_nsec += period_ns;
    // ... normalize ...
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);

    // Overrun recovery: 놓친 tick skip + 다음 경계 재정렬
    // ...

    ControlLoop();

    if (++tick % 10 == 0) CheckTimeouts();  // 50Hz inline
  }
}
```

> **v5.16.0**: `create_wall_timer()` + `cb_group_rt_` 제거. `clock_nanosleep(TIMER_ABSTIME)` 절대시간
> 루프로 대체하여 executor dispatch 지터 ~50-200μs 제거. CheckTimeouts()는 매 10틱 inline 호출.

### 구독자 할당

```cpp
void RtControllerNode::CreateSubscriptions() {
  // SubscriptionOptions에 그룹 지정
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_sensor_;

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        JointStateCallback(std::move(msg));
      },
      sub_options);  // ← 옵션 전달

  // target_sub_, hand_state_sub_도 동일
}
```

### 스레드 설정 (`thread_config.hpp`)

```cpp
namespace ur5e_rt_controller {

struct ThreadConfig {
  int         cpu_core;         // CPU affinity
  int         sched_policy;     // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int         sched_priority;   // 1-99 for SCHED_FIFO/RR
  int         nice_value;       // -20 to 19 for SCHED_OTHER
  std::string name;             // Thread name (max 15 chars)
};

// 사전 정의된 설정 (6-core)
inline constexpr ThreadConfig kRtControlConfig{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline constexpr ThreadConfig kSensorConfig{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

// ... kLoggingConfig, kAuxConfig
}
```

### 스레드 적용 (`thread_utils.hpp`)

```cpp
inline bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept {
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

  // 3. Thread name 설정
  pthread_setname_np(pthread_self(), cfg.name.c_str());
  return true;
}
```

### main() 함수 (`rt_controller_main.cpp`, v5.16.0)

```cpp
int main(int argc, char** argv) {
  // 1. 메모리 잠금 (페이지 폴트 방지) — rclcpp::init 보다 먼저 호출
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] mlockall failed\n");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<RtControllerNode>();
  const auto cfgs = SelectThreadConfigs();

  // 2. RT loop + Publish offload (jthread, executor 미사용)
  node->StartRtLoop(cfgs.rt_control);       // Core 2, SCHED_FIFO 90, clock_nanosleep
  node->StartPublishLoop(cfgs.publish);      // Core 5, SCHED_OTHER nice -3, SPSC drain

  // 3. ROS2 Executor 생성 (3개 — rt_executor 제거됨)
  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  sensor_executor.add_callback_group(node->GetSensorGroup(), ...);
  log_executor.add_callback_group(node->GetLogGroup(), ...);
  aux_executor.add_callback_group(node->GetAuxGroup(), ...);

  // 4. Executor 스레드 생성 + RT 설정
  auto t_sensor = make_thread(sensor_executor, cfgs.sensor);
  auto t_log    = make_thread(log_executor,    cfgs.logging);
  auto t_aux    = make_thread(aux_executor,    cfgs.aux);

  // 5. Join + 종료
  t_sensor.join(); t_log.join(); t_aux.join();
  node->StopRtLoop();
  node->StopPublishLoop();

  rclcpp::shutdown();
  return 0;
}
```

> **v5.16.0 변경**: `rt_executor` 제거. `StartRtLoop()` / `StartPublishLoop()`이
> `clock_nanosleep` jthread와 SPSC publish thread를 각각 관리. Executor 4개 → 3개.

---

## 검증 방법

### 1. 스레드 설정 확인

```bash
# rt_controller 프로세스 찾기
PID=$(pgrep -f rt_controller)

# 스레드 목록 + 스케줄러 정책 + CPU 코어
ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

**출력 예시 (v5.16.0)**:
```
  PID   TID CLS RTPRIO PSR COMMAND
 1234  1234  TS      -   0 rt_controller  (메인 스레드)
 1234  1235  FF     90   2 rt_control         ← Core 2, FIFO 90 (clock_nanosleep jthread)
 1234  1236  TS      -   5 rt_publish         ← Core 5, OTHER   (SPSC publish jthread)
 1234  1237  FF     70   3 sensor_io          ← Core 3, FIFO 70 (ROS2 Executor)
 1234  1238  TS      -   4 logger             ← Core 4, OTHER   (ROS2 Executor)
 1234  1239  TS      -   5 aux                ← Core 5, OTHER   (ROS2 Executor)
 1234  1240  TS      -   4 status_mon         ← Core 4, OTHER (10 Hz)
 1234  1241  TS      -   4 hand_detect        ← Core 4, OTHER (50 Hz)
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
                                                           ↑
                                                    Max 지터 < 50μs
```

### 4. ROS2 제어 주파수

```bash
# 명령 토픽 주파수 확인
ros2 topic hz /forward_position_controller/commands

# 출력:
average rate: 500.123
        min: 0.001998s max: 0.002002s std dev: 0.000001s window: 503
                           ↑
                    표준편차 < 1μs 목표
```

### 5. Context Switch 횟수

```bash
# 10초 동안 context switch 측정
sudo perf stat -e context-switches,cpu-migrations -p $PID sleep 10

# 출력:
#   context-switches: 1,234  (120/sec)  ← 목표: < 1000/sec
#   cpu-migrations:   0                 ← 목표: 0 (affinity 고정)
```

---

## 문제 해결

> **자동 진단**: `check_rt_setup.sh`를 실행하면 8개 카테고리의 RT 설정을 자동 점검합니다.
> ```bash
> ./ur5e_rt_controller/scripts/check_rt_setup.sh           # 상세 출력
> ./ur5e_rt_controller/scripts/check_rt_setup.sh --fix     # 수정 명령 제안
> ./ur5e_rt_controller/scripts/check_rt_setup.sh --summary # 1줄 요약
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
# 출력 없음 → 설정 안 됨
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
# 출력 없음 → 일반 커널

# 해결
sudo apt install linux-lowlatency-hwe-22.04
sudo reboot
```

**원인 2**: IRQ가 RT 코어에서 발생
```bash
cat /proc/interrupts | grep -E "(CPU2|CPU3|CPU4|CPU5)"
# 많은 IRQ → 문제

# 해결: IRQ를 Core 0-1로 이동
for irq in $(ls /proc/irq/); do
    [ -d "/proc/irq/$irq" ] && echo 3 | sudo tee /proc/irq/$irq/smp_affinity > /dev/null 2>&1
done
```

**원인 3**: DDS 스레드가 RT 코어 사용
```bash
# CycloneDDS의 경우 설정 파일 생성
cat > ~/cyclonedds.xml << 'EOF'
<CycloneDDS>
  <Domain>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
  <Internal>
    <Threads>
      <ThreadAffinityMask>0x3</ThreadAffinityMask>  <!-- Core 0-1 -->
    </Threads>
  </Internal>
</CycloneDDS>
EOF

export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
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
sudo ./ur5e_rt_controller/scripts/setup_nvidia_rt.sh

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
# 어떤 코어든 powersave → 문제

# 해결 (자동)
sudo ./ur5e_rt_controller/scripts/setup_nvidia_rt.sh  # [10/11] CPU governor 설정

# 해결 (수동)
sudo cpupower frequency-set -g performance
```

---

## 4-Core 시스템 대응

4-core CPU에서는 자동으로 fallback 설정 사용:

```cpp
// thread_config.hpp
inline constexpr ThreadConfig kRtControlConfig4Core{
    .cpu_core       = 1,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .name           = "rt_control"
};

inline constexpr ThreadConfig kSensorConfig4Core{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .name           = "sensor_io"
};

inline constexpr ThreadConfig kLoggingConfig4Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};
```

| Core | 용도 | Scheduler | Priority |
|------|------|-----------|----------|
| 0 | OS + DDS | SCHED_OTHER | - |
| 1 | RT Control (500Hz + 50Hz) | SCHED_FIFO | 90 |
| 2 | Sensor + UDP recv | SCHED_FIFO | 70/65 |
| 3 | Logging + Aux + Status Monitor + Hand Failure Detector | SCHED_OTHER | nice -5/0 |

**GRUB 설정 (4-core)**:
```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=1-3 nohz_full=1-3 rcu_nocbs=1-3"
```

**수동 설정**:
```cpp
// main()에서
#if NUM_CORES == 4
  auto t_rt     = make_thread(rt_executor,     kRtControlConfig4Core);
  auto t_sensor = make_thread(sensor_executor, kSensorConfig4Core);
  auto t_log    = make_thread(log_executor,    kLoggingConfig4Core);
  // aux_executor는 log_executor와 병합
#else
  // 6-core 설정
#endif
```

---

## 고급 튜닝

### 1. DDS 최적화

#### CycloneDDS (권장)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

**`cyclonedds.xml`**:
```xml
<CycloneDDS>
  <Domain>
    <Internal>
      <Threads>
        <ThreadAffinityMask>0x3</ThreadAffinityMask>  <!-- Core 0-1 -->
      </Threads>
    </Internal>
  </Domain>
</CycloneDDS>
```

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
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=2147483647
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

**최종 업데이트**: 2026-03-17
**작성자**: UR5e RT Controller Team
**버전**: v5.16.0 (v4.2.0 기반 → v5.16.0: clock_nanosleep RT loop + SPSC publish offload + overrun recovery + CPU 코어 할당 최적화 + 모니터링 스레드 + NVIDIA DKMS RT 커널 우회 + CPU governor 자동 설정)
