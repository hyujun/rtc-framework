# hand_udp Realtime Optimization Plan

## Overview

3가지 카테고리 수정: Critical RT 위반 제거(A, B), WritePosition echo 활용 최적화(C), 연관 개선(D).

---

## Phase 1: Critical A — EventLoop에서 printf 제거

**파일**: `ur5e_hand_udp/include/ur5e_hand_udp/hand_controller.hpp`

**현상**: EventLoop (SCHED_FIFO/65) 내에서 `std::printf`로 디버그 출력.
- stdout mutex + write(2) syscall → ms-level latency spike
- 1초마다 실행이지만 priority inversion 위험

**수정**:
1. sensor debug print (L903-924, L996-1026)의 `std::printf` 블록을 완전히 제거
2. comm debug print (L1057-1084)의 `std::printf` 블록을 완전히 제거
3. 대체: `HandTimingProfiler`가 이미 per-phase 통계를 수집하고 있으므로 별도 디버그 출력 불필요
4. 필요 시 `HandCommStats`에 last sensor raw 값을 atomic으로 보관하고 외부(non-RT)에서 출력

**영향 범위**: hand_controller.hpp만 수정. API 변경 없음.

---

## Phase 2: Critical B — state_mutex_ → SeqLock (lock-free read/write)

**파일**: `ur5e_hand_udp/include/ur5e_hand_udp/hand_controller.hpp`

**현상**: EventLoop(writer, FIFO/65)와 GetLatestState(reader, FIFO/70)가 `std::mutex`를 공유.
- Reader가 lock 보유 중이면 writer가 blocking → RT deadline miss 가능
- Priority inversion 위험

**수정**:
1. `SeqLock<HandState>` 구현 (또는 triple-buffer)
   - Writer: sequence 홀수 → memcpy write → sequence 짝수 (store-release)
   - Reader: sequence 읽기 → memcpy read → sequence 재확인 (불일치 시 retry)
   - HandState는 trivially-copyable이므로 SeqLock 적합
2. `state_mutex_` 삭제, `SeqLock<HandState> state_seqlock_` 으로 교체
3. `GetLatestState()`: SeqLock read (lock-free, wait-free on writer side)
4. EventLoop state update: SeqLock write (wait-free)

**구현 위치**: `ur5e_rt_base/include/ur5e_rt_base/` 에 `seqlock.hpp` 추가 (재사용 가능)

**SeqLock 설계**:
```cpp
template <typename T>
class SeqLock {
  alignas(64) std::atomic<uint32_t> seq_{0};
  alignas(64) T data_{};
public:
  void Store(const T& val) noexcept {
    seq_.store(seq_.load(std::memory_order_relaxed) + 1, std::memory_order_release);  // 홀수
    std::memcpy(&data_, &val, sizeof(T));
    seq_.store(seq_.load(std::memory_order_relaxed) + 1, std::memory_order_release);  // 짝수
  }
  T Load() const noexcept {
    T result;
    uint32_t s0, s1;
    do {
      s0 = seq_.load(std::memory_order_acquire);
      std::memcpy(&result, &data_, sizeof(T));
      s1 = seq_.load(std::memory_order_acquire);
    } while (s0 != s1 || (s0 & 1));
    return result;
  }
};
```

**영향 범위**: hand_controller.hpp (state_mutex_ → SeqLock), ur5e_rt_base에 seqlock.hpp 추가.
GetLatestState(), GetLatestPositions() API는 동일 시그니처 유지 (내부만 변경).
HandUdpNode의 callback 내 data_mutex_도 동일 패턴 적용 검토.

---

## Phase 3: WritePosition Echo 활용 최적화

**파일**: `ur5e_hand_udp/include/ur5e_hand_udp/hand_controller.hpp`

**현상 분석**:
- WritePosition(0x01) 시 하드웨어가 동일 포맷(43B) echo 반환
- `enable_write_ack_: false` → echo 미수신 → 소켓 버퍼 오염
  - Individual: cascading response 버그 (pos↔vel 데이터 혼동)
  - Bulk: AllMotorRead 실패 (43B echo < 123B expected)
- `enable_write_ack_: true` → echo 수신하지만 데이터 폐기

**수정 (3단계)**:

### 3a. Write echo를 항상 수신 + 디코딩 (enable_write_ack_ 제거)
- `enable_write_ack_` 파라미터 제거 (항상 echo 수신)
- echo 수신 후 `DecodeMotorResponse()`로 디코딩
- echo 데이터를 `write_echo_positions_`에 저장 (디버깅/검증용)
- echo 수신 실패 시 (타임아웃) 기존 동작 유지 (다음 단계로 진행)

### 3b. Individual 모드: echo로 ReadPosition 대체 → 1 round-trip 절약
- Write echo 데이터 = 하드웨어가 적용한 실제 position → ReadPosition(0x11) 불필요
- EventLoop 순서 변경:
  ```
  Before: Write(0x01) → ReadPos(0x11) → ReadVel(0x12) → Sensors
  After:  Write(0x01)+echo → ReadVel(0x12) → Sensors
  ```
- Individual 모드: 6 round-trips → 5 round-trips (→ ~200µs 절감)

### 3c. cmd 검증 추가 (안전장치)
- `RequestMotorRead()` 에 cmd 검증 추가:
  ```cpp
  if (cmd_out != static_cast<uint8_t>(cmd)) return false;  // stale 패킷 거부
  ```
- `RequestAllMotorRead()`에도 동일 검증 추가
- stale 패킷 수신 시 retry (non-blocking recv, 최대 2회)

---

## Phase 4: 연관 개선 — DrainStaleResponses 최소화

**현상**: echo를 항상 수신하면 stale 패킷 발생 원인이 제거됨.

**수정**:
- Bulk 모드: WriteEcho 수신 + AllMotorRead 응답 = cmd 검증으로 stale 방지 → DrainStaleResponses 불필요
- Individual 모드: WriteEcho + ReadVel 응답 = cmd 검증으로 stale 방지
- DrainStaleResponses를 defensive fallback으로만 유지 (센서 읽기 전)
- drain 호출 빈도를 줄이거나 sensor_decimation cycle에서만 호출

---

## 변경 파일 요약

| 파일 | 변경 내용 |
|---|---|
| `ur5e_rt_base/include/ur5e_rt_base/seqlock.hpp` | **신규** — SeqLock 템플릿 |
| `ur5e_hand_udp/include/ur5e_hand_udp/hand_controller.hpp` | Phase 1-4 전체 적용 |
| `ur5e_hand_udp/config/hand_udp_node.yaml` | `enable_write_ack` 제거 |
| `ur5e_hand_udp/src/hand_udp_node.cpp` | `enable_write_ack` 파라미터 제거, SeqLock 적용 |

## 예상 효과

| 항목 | Before | After |
|---|---|---|
| printf spike | ~1ms (1초마다) | 0 |
| state_mutex blocking | ~1-10µs (worst case priority inversion) | 0 (lock-free) |
| Individual round-trips | 6 | 5 (echo 활용) |
| Response cascading 버그 | 있음 (enable_write_ack=false) | 없음 |
| Bulk motor read 실패 | 있음 (echo stale) | 없음 |
| DrainStaleResponses syscalls | 최대 8×recv (sensor cycle마다) | 대폭 감소 |
