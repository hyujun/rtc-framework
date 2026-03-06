# CLAUDE.md — ur5e_rt_base

> **Note:** This package is part of the UR5e RT Controller workspace (v5.2.2). Please refer to the [Root CLAUDE.md](../CLAUDE.md) for full workspace context, building instructions, and architecture overview.
헤더-전용 공유 기반 패키지. `ur5e_rt_controller`와 `ur5e_hand_udp`가 의존한다.
**구현 파일(.cpp)이 없다** — CMakeLists는 `INTERFACE` 라이브러리만 노출한다.

---

## 파일 구조 및 역할

```
include/ur5e_rt_base/
├── types.hpp          ← 공유 상수·구조체 (RobotState, HandState, …)
├── thread_config.hpp  ← ThreadConfig + 4/6/8코어 사전 정의 상수
├── thread_utils.hpp   ← ApplyThreadConfig(), SelectThreadConfigs()
├── log_buffer.hpp     ← SpscLogBuffer<N> lock-free SPSC 링 버퍼
├── data_logger.hpp    ← DataLogger — CSV 파일 기록 (log 스레드 전용)
└── filters/
    ├── bessel_filter.hpp  ← BesselFilterN<N> — 4차 Bessel LPF
    └── kalman_filter.hpp  ← KalmanFilterN<N> — 이산-시간 Kalman 필터
```

---

## types.hpp — 핵심 타입 정의

```cpp
namespace ur5e_rt_controller {

// 컴파일-타임 상수
inline constexpr int kNumRobotJoints = 6;
inline constexpr int kNumHandJoints  = 11;
inline constexpr int kNumHandSensors = 44;  // 4 sensors × 11 joints

struct RobotState  { array<double,6> positions, velocities; array<double,3> tcp_position; double dt; uint64_t iteration; };
struct HandState   { array<double,11> motor_positions, velocities, currents; array<double,44> sensor_data; bool valid; };
struct ControllerState { RobotState robot; HandState hand; double dt; uint64_t iteration; };
struct ControllerOutput { array<double,6> robot_commands; array<double,11> hand_commands; bool valid; };
```

> 네임스페이스는 `ur5e_rt_controller`로 모든 패키지에서 통일. `.cpp`에서는 `namespace urtc = ur5e_rt_controller;`로 별칭 사용.

---

## thread_config.hpp — RT 스레드 배치

| 상수 | 코어 | 정책 | 우선순위 | 용도 |
|------|------|------|---------|------|
| `kRtControlConfig` | 2 | FIFO | 90 | 500Hz 제어 루프 |
| `kSensorConfig` | 3 | FIFO | 70 | 관절 상태·타겟 콜백 |
| `kUdpRecvConfig` | 5 | FIFO | 65 | UDP 핸드 수신 |
| `kLoggingConfig` | 4 | OTHER | nice -5 | CSV 드레인 |
| `kAuxConfig` | 5 | OTHER | 0 | E-STOP 퍼블리셔 |

**4코어 fallback**: Core 1(RT), 2(sensor+udp_recv 공유), 3(log+aux 공유).
**8코어**: udp_recv → Core 4 독립, aux → Core 6.

`SelectThreadConfigs()` — `sysconf(_SC_NPROCESSORS_ONLN)`으로 런타임 자동 선택.

---

## thread_utils.hpp — ApplyThreadConfig()

`pthread_setaffinity_np` + `pthread_setschedparam` + `setpriority` + `pthread_setname_np` 순서로 적용.
실패 시 `false` 반환 (프로세스는 계속 실행, 지터 보장 없음).

**RT 권한 요건**: `@realtime - rtprio 99` in `/etc/security/limits.conf`.

---

## log_buffer.hpp — SpscLogBuffer<512>

RT 스레드(producer)와 log 스레드(consumer) 사이 **lock-free** 단방향 채널.

```
RT 스레드:  Push(LogEntry) → O(1), 블로킹·할당 없음. 버퍼 만료시 drop.
log 스레드: Pop(LogEntry) → 100Hz drain_timer에서 호출.
```

- `head_` (producer 전용) / `tail_` (consumer 전용) — `alignas(64)`로 false-sharing 방지.
- `memory_order_release` / `acquire` 페어로 가시성 보장.
- **`Push()`는 RT 스레드에서만, `Pop()`은 log 스레드에서만** 호출해야 한다.

---

## filters/ — RT-안전 신호 처리

### BesselFilterN<N>

- **Init(cutoff_hz, sample_rate_hz)** — Nyquist 초과 시 assert. RT 경로 외에서 1회 호출.
- **Apply(array<double,N>)** `noexcept` — Direct Form II Transposed 2×biquad 연속.
- 별칭: `BesselFilter6`, `BesselFilter11`, `BesselFilter1`
- 특성: 최대 선형 군지연 → 위상 왜곡 없음. 순수 주파수 기반 튜닝.

### KalmanFilterN<N>

- **Init(q_pos, q_vel, r, dt)** — 공분산 행렬 초기화. RT 경로 외에서 1회 호출.
- **PredictAndUpdate(array<double,N>)** `noexcept` → 필터 위치 반환.
- **velocity(i)** `noexcept` → 채널 i 속도 추정 (미분 없음).
- 상태: `{pos, vel}`, 공분산: `{p00, p01, p11}` 스칼라 3개 — Eigen 불필요.
- 별칭: `KalmanFilter6`, `KalmanFilter11`, `KalmanFilter1`

---

## 이 패키지를 수정할 때 주의사항

1. **types.hpp 구조체 변경** → `ur5e_rt_controller`와 `ur5e_hand_udp` 모두 재빌드 필요.
   `kNumRobotJoints` / `kNumHandJoints` 변경은 모든 `std::array` 크기에 전파된다.

2. **thread_config.hpp 코어 번호 변경** → `mujoco_simulator.yaml`의 E-STOP 설정,
   `ur_control.launch.py`의 taskset, `cyclone_dds.xml` affinity와 일관성 유지 필요.

3. **SpscLogBuffer** — capacity는 `512` (power-of-2 필수). 500Hz × 1초 = 500 entries.
   capacity 줄이면 drain 지연 시 drop 발생.

4. **필터 헤더** — `Init()` 이전에 `Apply()` 호출하면 undefined behavior.
   E-STOP 후 재시작 시 `Reset()` 호출 권장.

5. 이 패키지 자체는 ROS2 의존성이 없다. `rclcpp` 없이도 단독 포함 가능.
