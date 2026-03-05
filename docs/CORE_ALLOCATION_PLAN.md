# 실제 로봇 제어 시 CPU 코어 할당 최적화 계획

## 1. 현황 및 문제점

### 현재 6코어 할당 구조

```
Core 0  ─ OS / 커널 스레드 / DDS 수신 (비격리, 인터럽트 포함)
Core 1  ─ OS / 커널 스레드 / DDS 수신 (비격리)
Core 2  ─ rt_control   SCHED_FIFO 90  (ControlLoop 500Hz + CheckTimeouts 50Hz)
Core 3  ─ sensor_io    SCHED_FIFO 70  (JointStateCallback, TargetCallback, HandStateCallback)
          udp_recv     SCHED_FIFO 65  ← sensor_io와 Core 3 공유 ★문제
Core 4  ─ logger       SCHED_OTHER nice -5
Core 5  ─ aux          SCHED_OTHER 0
```

### 핵심 문제점

| # | 문제 | 영향 |
|---|---|---|
| P1 | **Core 3 경합**: `sensor_io`(FIFO 70)와 `udp_recv`(FIFO 65)가 같은 코어 공유 | UDP 수신 버스트 시 joint_state 콜백 지연 → E-STOP 오발동 위험 |
| P2 | **UR 드라이버 스레드 비제어**: `ur_ros2_driver`가 자체 스레드를 생성하나 CPU 친화성 미설정 → RT 코어(2-3)에 착지 가능 | 500Hz RT 루프 지터 증가 |
| P3 | **DDS 스레드 미제어**: CycloneDDS 내부 송수신 스레드가 Core 0-5 어디에든 스케줄링 가능 | 커널 컨텍스트 전환이 Core 2-3 캐시를 오염 |
| P4 | **NIC 인터럽트 미설정**: UR 드라이버가 사용하는 이더넷 NIC의 IRQ가 RT 코어에 서비스될 수 있음 | Core 2 jitter 최대 수백 µs 추가 |
| P5 | **Hyperthreading 미고려**: HT 활성 시 Core 2/3의 물리 코어를 sibling과 공유 → L1/L2 캐시 오염 | 제어 루프 실행 시간 불규칙 |

---

## 2. 최적화 방안

### 방안 A — 6코어 재배치 (권장, 하드웨어 변경 없음)

**핵심 변경**: `udp_recv`를 Core 3 → Core 5로 이동, `aux`도 Core 5에 같이 배치 (aux는 E-STOP 상태 퍼블리시만 하는 경량 작업)

```
Core 0  ─ OS / 커널 / NIC IRQ / DDS 스레드 (pinned by systemd/taskset)
Core 1  ─ UR 드라이버 프로세스 (taskset 0x2 로 제한)
Core 2  ─ rt_control   SCHED_FIFO 90  (전용, 변경 없음)
Core 3  ─ sensor_io    SCHED_FIFO 70  (전용 ← udp_recv 제거)
Core 4  ─ logger       SCHED_OTHER nice -5
Core 5  ─ udp_recv     SCHED_FIFO 65  (Core 3에서 이동)
          aux          SCHED_OTHER 0  (경량이므로 공존 허용)
```

**변경 파일**: `ur5e_rt_base/include/ur5e_rt_base/thread_config.hpp`
```cpp
// 변경 전
inline constexpr ThreadConfig kUdpRecvConfig{
    .cpu_core = 3,          // ← sensor_io와 공유
    ...
};

// 변경 후
inline constexpr ThreadConfig kUdpRecvConfig{
    .cpu_core = 5,          // ← aux와 공유 (경량 트레이드오프)
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .name           = "udp_recv"
};
```

---

### 방안 B — 8코어 시스템 전용 배치 (최고 성능)

8코어 이상 시스템에서 모든 RT 스레드를 완전 격리.

```
Core 0  ─ OS / 커널 / NIC IRQ
Core 1  ─ DDS recv/send 스레드, UR 드라이버 프로세스
Core 2  ─ rt_control   SCHED_FIFO 90
Core 3  ─ sensor_io    SCHED_FIFO 70
Core 4  ─ udp_recv     SCHED_FIFO 65  (전용 코어)
Core 5  ─ logger       SCHED_OTHER nice -5
Core 6  ─ aux          SCHED_OTHER 0
Core 7  ─ 여유 (모니터링, cyclictest 측정용)
```

**GRUB 격리**: `isolcpus=2-6 nohz_full=2-6 rcu_nocbs=2-6`

---

### 방안 C — UR 드라이버 프로세스 CPU 친화성 고정

`launch/ur_control.launch.py`에서 `ur_ros2_driver` 노드를 Core 0-1로 제한.

**변경 파일**: `ur5e_rt_controller/launch/ur_control.launch.py`

```python
# ur_ros2_driver 노드에 additional_env 또는 prefix로 taskset 적용
ur_driver_node = Node(
    package='ur_robot_driver',
    executable='ur_ros2_driver',
    prefix='taskset -c 0-1',      # ← 추가: Core 0-1으로 제한
    ...
)
```

---

### 방안 D — NIC IRQ 친화성 스크립트 추가

`install.sh` 또는 별도 `scripts/setup_irq_affinity.sh` 스크립트.

```bash
#!/bin/bash
# UR 드라이버가 사용하는 NIC의 IRQ를 Core 0-1로 제한
# RT 코어(2-5)를 인터럽트로부터 보호
NIC=${1:-eth0}  # 사용 NIC 이름 인자 (예: enp3s0)

for irq in $(grep "${NIC}" /proc/interrupts | awk -F: '{print $1}' | tr -d ' '); do
    echo 3 | sudo tee /proc/irq/${irq}/smp_affinity > /dev/null   # 0x3 = Core 0,1
    echo "IRQ ${irq} → Core 0-1"
done

# 기타 모든 IRQ도 Core 0-1으로
for irq in $(ls /proc/irq/); do
    [ -f "/proc/irq/${irq}/smp_affinity" ] && \
        echo 3 | sudo tee /proc/irq/${irq}/smp_affinity > /dev/null 2>&1
done
echo "IRQ affinity set: all IRQs → Core 0-1"
```

---

### 방안 E — CycloneDDS 스레드 코어 제한

`~/.ros/cyclone_dds.xml` (또는 환경변수 `CYCLONEDDS_URI`):

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" multicast="true"/>
      </Interfaces>
    </General>
    <Threads>
      <!-- DDS recv/send 스레드를 Core 0-1으로 제한 -->
      <Thread name="recv">
        <Scheduling>
          <Class>Other</Class>
          <Priority>0</Priority>
        </Scheduling>
      </Thread>
    </Threads>
  </Domain>
</CycloneDDS>
```

환경변수 설정 (`ur_control.launch.py`에 추가):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclone_dds.xml
```

---

## 3. 구현 우선순위

| 우선순위 | 방안 | 난이도 | 효과 | 비고 |
|:---:|---|:---:|:---:|---|
| ★★★ | **A: udp_recv → Core 5 이동** | 낮음 | 높음 | 코드 1줄 변경, 즉시 적용 가능 |
| ★★★ | **D: NIC IRQ 친화성 스크립트** | 낮음 | 높음 | install.sh에 통합 |
| ★★☆ | **C: UR 드라이버 taskset** | 낮음 | 중간 | launch 파일 수정 |
| ★★☆ | **E: CycloneDDS 스레드 제한** | 중간 | 중간 | XML 설정 추가 |
| ★☆☆ | **B: 8코어 전용 배치** | 높음 | 최고 | 하드웨어 의존 |

---

## 4. 적용 후 기대 효과

| 지표 | 현재 (추정) | 개선 후 (추정) |
|---|---|---|
| Core 3 컨텍스트 전환 | sensor_io + udp_recv 경합 | sensor_io 전용 → 경합 제거 |
| Control jitter (P99) | <50µs | <20µs (Core 2 완전 격리 시) |
| UDP 수신 지연 | UR driver 간섭 가능 | Core 5 독립 → 안정적 100Hz |
| E-STOP 오발동 위험 | sensor_io 지연 시 존재 | Core 3 전용화로 제거 |

---

## 5. 검증 방법

```bash
# 1. 스레드 코어 배치 확인
PID=$(pgrep -f custom_controller)
ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
# 기대값: rt_control=Core2, sensor_io=Core3, udp_recv=Core5

# 2. Core 2 jitter 측정 (최적화 전/후 비교)
sudo cyclictest --mlockall --priority=90 --policy=fifo \
    --interval=2000 --loops=200000 --affinity=2 --histogram=200
# 목표: Max < 50µs, P99 < 20µs

# 3. IRQ 친화성 확인
cat /proc/interrupts | grep eth   # NIC IRQ가 Core 0,1에만 서비스되는지 확인

# 4. UR 드라이버 프로세스 코어 확인 (방안 C 적용 시)
PID=$(pgrep -f ur_ros2_driver)
taskset -p $PID  # pid N's current affinity mask: 3  (= 0b11 = Core 0,1)
```

---

## 6. 파일 변경 요약

| 파일 | 변경 내용 | 방안 |
|---|---|---|
| `ur5e_rt_base/include/ur5e_rt_base/thread_config.hpp` | `kUdpRecvConfig.cpu_core` 3 → 5 | A |
| `ur5e_rt_controller/launch/ur_control.launch.py` | UR driver node에 `prefix='taskset -c 0-1'` | C |
| `install.sh` | NIC IRQ 친화성 설정 섹션 추가 | D |
| `scripts/setup_irq_affinity.sh` | 신규: IRQ affinity 스크립트 | D |
| `config/cyclone_dds.xml` | 신규: DDS 스레드 제한 설정 | E |
| `ur5e_rt_controller/launch/ur_control.launch.py` | `CYCLONEDDS_URI` 환경변수 설정 | E |
